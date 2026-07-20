//! Minimal **read-only** FAT12 reader for the external flash FAT partition.
//!
//! Two operations, which is all the player needs: look a file up by its 8.3
//! name, and read an arbitrary byte range out of it.  Nothing here writes,
//! erases or formats — the badge's own firmware owns the filesystem, this
//! firmware is a guest.  All flash access goes through [`crate::flash`]
//! (async-mutex-protected).
//!
//! # FAT12 on-disk layout (1 MiB partition)
//!
//! ```text
//!   ┌──────────────────┐  sector 0
//!   │   Boot sector    │  BPB (BIOS Parameter Block): sector size, cluster
//!   │                  │  size, FAT count, root entry count, etc.
//!   ├──────────────────┤  sector 1  (= reserved_sectors)
//!   │   FAT #1         │  File Allocation Table — linked list of cluster
//!   │                  │  chains.  Each entry is 12 bits (1.5 bytes).
//!   ├──────────────────┤
//!   │   FAT #2         │  Backup copy of FAT #1 (we only read FAT #1).
//!   ├──────────────────┤
//!   │  Root directory  │  Fixed-size array of 32-byte entries.  Each entry
//!   │                  │  holds an 8.3 filename, attributes, first cluster
//!   │                  │  number, and file size.
//!   ├──────────────────┤
//!   │   Data region    │  File contents stored in clusters.  Cluster 2 is
//!   │                  │  the first data cluster (clusters 0 and 1 are
//!   │                  │  reserved in the FAT).
//!   └──────────────────┘
//! ```
//!
//! # FAT12 cluster chain
//!
//! Each FAT entry is 12 bits.  For cluster N:
//! - Byte offset in FAT = N × 3 / 2
//! - If N is even: entry = low 12 bits of the 16-bit word at that offset
//! - If N is odd:  entry = high 12 bits (shift right by 4)
//! - Entry values: 0x000 = free, 0xFF8–0xFFF = end of chain, else = next
//!   cluster
//!
//! # Random access
//!
//! The player never reads a file end to end: it consults the video's frame
//! offset table and jumps to whichever frame the audio clock says is current,
//! and the audio task walks its own file in ADPCM blocks.  So the primitive is
//! [`read_at`] — seek to a byte offset, read a run of bytes, cross cluster
//! boundaries as needed — and the two costs a naive implementation pays per
//! call are cached away:
//!
//! - the boot sector is parsed once and kept in [`PARAMS`], instead of being
//!   re-read on every access;
//! - each open file gets a slot in [`SEEK`] remembering the last
//!   (cluster index, cluster number) it reached, so a forward seek resumes the
//!   chain walk from there instead of restarting at cluster 0.  Playback is
//!   almost entirely forward, which makes the walk O(1) amortised rather than
//!   O(file length) per frame.
//!
//! # Usage
//!
//! ```rust,ignore
//! let video = fat12::find_file(&fat12::to_8_3("BADAPPLE.VID").unwrap()).await?;
//! fat12::read_at(&video, frame_offset, &mut frame_buf).await?;
//! ```

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use crate::flash;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FatError {
    /// No valid FAT12 boot sector found (bad jump byte or zero sector size).
    NoFilesystem,
    /// File not found in root directory.
    FileNotFound,
    /// Flash read failed (QSPI hardware error).
    FlashError,
    /// Corrupt FAT chain or directory entry (unexpected end-of-chain or bad
    /// cluster).
    Corrupt,
}

// ---------------------------------------------------------------------------
// FileRef — lightweight file handle
// ---------------------------------------------------------------------------

/// Handle to a file on the filesystem.  Stores only the first cluster number
/// and the file size — enough to read any part of the file by following the
/// FAT chain.  Obtained from [`find_file`].
///
/// Copy it, keep it in a task, reuse it for any number of reads at any
/// offsets; it never needs the directory to be re-scanned.
#[derive(Clone, Copy)]
pub struct FileRef {
    /// First cluster in the FAT chain (cluster 2 = first data cluster).
    pub(crate) first_cluster: u16,
    /// File size in bytes.
    pub size: u32,
}

impl FileRef {
    /// Empty/invalid handle, used for array initialisation.
    #[allow(dead_code)]
    pub const EMPTY: Self = Self {
        first_cluster: 0,
        size: 0,
    };
}

// ---------------------------------------------------------------------------
// FatParams — boot sector parameters
// ---------------------------------------------------------------------------

/// Geometry parsed from the BPB (BIOS Parameter Block) in the boot sector.
#[derive(Clone, Copy)]
struct FatParams {
    bytes_per_sector: u16,
    sectors_per_cluster: u8,
    reserved_sectors: u16,
    num_fats: u8,
    root_entry_count: u16,
    sectors_per_fat: u16,
}

impl FatParams {
    /// Bytes per cluster (sector_size × sectors_per_cluster).
    fn cluster_bytes(&self) -> u32 {
        self.sectors_per_cluster as u32 * self.bytes_per_sector as u32
    }

    /// Absolute flash address of the first FAT table.
    /// Layout: [boot sector(s)] [FAT #1] [FAT #2] [root dir] [data]
    fn fat_offset(&self) -> u32 {
        flash::FAT_OFFSET + self.reserved_sectors as u32 * self.bytes_per_sector as u32
    }

    /// Absolute flash address of the root directory.
    fn root_dir_offset(&self) -> u32 {
        let fat_size = self.num_fats as u32 * self.sectors_per_fat as u32;
        flash::FAT_OFFSET + (self.reserved_sectors as u32 + fat_size) * self.bytes_per_sector as u32
    }

    /// Number of sectors occupied by the root directory.
    fn root_dir_sectors(&self) -> u32 {
        (self.root_entry_count as u32 * 32).div_ceil(self.bytes_per_sector as u32)
    }

    /// Absolute flash address of the data region (cluster 2 starts here).
    fn data_region_offset(&self) -> u32 {
        self.root_dir_offset() + self.root_dir_sectors() * self.bytes_per_sector as u32
    }

    /// Absolute flash address of the given cluster's data.
    /// Clusters are numbered starting at 2 (0 and 1 are reserved in FAT).
    fn cluster_addr(&self, cluster: u16) -> u32 {
        // Clusters 0/1 are reserved; a corrupt chain could hand us one.
        // saturating_sub avoids the `- 2` underflow (panic in debug, huge
        // wrapped address → OOB flash read in release). saturating_mul/add
        // likewise keep a bogus cluster number from overflowing u32 — geometry
        // is validated in read_params, and read_at rejects first_cluster < 2,
        // but stay defensive here since the input is host-written.
        let idx = (cluster as u32).saturating_sub(2);
        self.data_region_offset()
            .saturating_add(idx.saturating_mul(self.cluster_bytes()))
    }

    /// Reject boot-sector geometry that is out of range or whose derived
    /// offsets overflow / fall outside the FAT partition. Every field here is
    /// host-written (the partition is filled over USB MSC by the stock
    /// firmware), so a crafted BPB — e.g. reserved_sectors=0xFFFF,
    /// bytes_per_sector=0xFFFF — could otherwise overflow the u32 address math
    /// (panic in debug, wrapped OOB flash read in release) or make the
    /// directory scan run far past the real directory. All derived addresses
    /// are computed with checked arithmetic.
    fn validate(&self) -> Result<(), FatError> {
        // bytes_per_sector: standard FAT sector sizes only. Also bounds every
        // product below.
        if !matches!(self.bytes_per_sector, 512 | 1024 | 2048 | 4096) {
            return Err(FatError::NoFilesystem);
        }
        // sectors_per_cluster: power of two, ≥ 1 (cluster_bytes ≤ 4096 × 128).
        if !self.sectors_per_cluster.is_power_of_two() {
            return Err(FatError::NoFilesystem);
        }
        // num_fats: 1 or 2 on real filesystems; reserved_sectors: ≥ the boot
        // sector.
        if !matches!(self.num_fats, 1 | 2) || self.reserved_sectors == 0 {
            return Err(FatError::NoFilesystem);
        }

        // Compute the end of the data region with checked arithmetic; reject on
        // overflow or if it (plus at least one cluster) spills past the
        // partition. Mirrors data_region_offset() but overflow-safe.
        let bps = self.bytes_per_sector as u32;
        let region_end = (|| {
            let fat_size = (self.num_fats as u32).checked_mul(self.sectors_per_fat as u32)?;
            let root_sectors = (self.root_entry_count as u32).checked_mul(32)?.div_ceil(bps);
            let total_sectors = (self.reserved_sectors as u32)
                .checked_add(fat_size)?
                .checked_add(root_sectors)?;
            let data_start = flash::FAT_OFFSET.checked_add(total_sectors.checked_mul(bps)?)?;
            data_start.checked_add(self.cluster_bytes())
        })()
        .ok_or(FatError::NoFilesystem)?;

        if region_end > flash::FAT_OFFSET + flash::FAT_BYTES as u32 {
            return Err(FatError::NoFilesystem);
        }
        Ok(())
    }
}

/// Parsed boot sector, kept after the first read.
///
/// The filesystem is read-only for us and nothing else is running, so the
/// geometry cannot change under our feet.  Caching it turns every subsequent
/// `read_at` / `find_file` into pure data reads instead of paying a 64-byte
/// boot-sector read first — worth it at one seek per frame.
static PARAMS: Mutex<CriticalSectionRawMutex, Option<FatParams>> = Mutex::new(None);

/// Read and parse the boot sector from the FAT partition (cached).
/// Returns the filesystem geometry needed for all other operations.
async fn read_params() -> Result<FatParams, FatError> {
    if let Some(p) = *PARAMS.lock().await {
        return Ok(p);
    }

    let mut buf = [0u8; 64];
    flash::read(flash::FAT_OFFSET, &mut buf)
        .await
        .map_err(|_| FatError::FlashError)?;

    // The first byte of a valid FAT boot sector is a jump instruction:
    // 0xEB (short jump) or 0xE9 (near jump).
    if buf[0] != 0xEB && buf[0] != 0xE9 {
        return Err(FatError::NoFilesystem);
    }
    let bps = u16::from_le_bytes([buf[11], buf[12]]);
    if bps == 0 || buf[13] == 0 {
        return Err(FatError::NoFilesystem);
    }

    let params = FatParams {
        bytes_per_sector: bps,        // BPB offset 11: usually 512
        sectors_per_cluster: buf[13], // BPB offset 13: e.g. 4 for 2K clusters
        reserved_sectors: u16::from_le_bytes([buf[14], buf[15]]), // BPB offset 14
        num_fats: buf[16],            // BPB offset 16: usually 2
        root_entry_count: u16::from_le_bytes([buf[17], buf[18]]), // BPB offset 17
        sectors_per_fat: u16::from_le_bytes([buf[22], buf[23]]), // BPB offset 22
    };
    params.validate()?;

    *PARAMS.lock().await = Some(params);
    Ok(params)
}

// ---------------------------------------------------------------------------
// Cluster chain walking
// ---------------------------------------------------------------------------

/// Follow the FAT12 chain: given a cluster number, return the next cluster.
///
/// FAT12 packs two 12-bit entries into 3 bytes:
///   byte_offset = cluster × 3 / 2
///   even cluster: low 12 bits of u16 at byte_offset
///   odd cluster:  high 12 bits (u16 >> 4)
///
/// Returns `None` for end-of-chain (0xFF8–0xFFF) or free (0x000).
async fn next_cluster(params: &FatParams, cluster: u16) -> Result<Option<u16>, FatError> {
    let fat_addr = params.fat_offset();
    let byte_offset = (cluster as u32 * 3) / 2;
    let mut pair = [0u8; 2];
    flash::read(fat_addr + byte_offset, &mut pair)
        .await
        .map_err(|_| FatError::FlashError)?;

    let val = if cluster & 1 == 0 {
        // Even cluster: take low 12 bits.
        u16::from_le_bytes(pair) & 0x0FFF
    } else {
        // Odd cluster: take high 12 bits.
        u16::from_le_bytes(pair) >> 4
    };

    // 0xFF8..=0xFFF = end of chain, 0x000 = free, 0xFF7 = bad sector.
    if val >= 0xFF8 || val == 0 {
        Ok(None)
    } else {
        Ok(Some(val))
    }
}

/// One remembered position in a file's cluster chain.
#[derive(Clone, Copy)]
struct SeekHint {
    /// Identifies the file (its first cluster is unique per file).
    first_cluster: u16,
    /// Index of `cluster` within the chain (0 = first cluster).
    index: u32,
    /// Cluster number at `index`.
    cluster: u16,
}

/// One hint slot per concurrently-played file: video and audio.  Entries are
/// pure hints derived from immutable on-flash data, so a lost race merely
/// costs a longer walk.
static SEEK: Mutex<CriticalSectionRawMutex, [Option<SeekHint>; 2]> = Mutex::new([None; 2]);

/// Look up the cached chain position for `file`, if it is at or before
/// `target` (a hint past the target is useless — the chain is forward-only).
async fn seek_hint(file: &FileRef, target: u32) -> Option<SeekHint> {
    SEEK.lock()
        .await
        .iter()
        .flatten()
        .find(|h| h.first_cluster == file.first_cluster && h.index <= target)
        .copied()
}

/// Remember where a walk ended up, replacing this file's slot or claiming a
/// free / arbitrary one.
async fn store_hint(hint: SeekHint) {
    let mut slots = SEEK.lock().await;
    let idx = slots
        .iter()
        .position(|s| matches!(s, Some(h) if h.first_cluster == hint.first_cluster))
        .or_else(|| slots.iter().position(|s| s.is_none()))
        .unwrap_or(0);
    slots[idx] = Some(hint);
}

/// Walk to the cluster holding byte `offset` of `file`, using and refreshing
/// the seek hint.  Returns the cluster number and the byte offset within it.
async fn seek(params: &FatParams, file: &FileRef, offset: u32) -> Result<(u16, u32), FatError> {
    let cluster_bytes = params.cluster_bytes();
    let target_index = offset / cluster_bytes;
    let within = offset % cluster_bytes;

    // Start from the cached position when it is behind us, else from the top.
    let (mut cluster, mut index) = match seek_hint(file, target_index).await {
        Some(h) => (h.cluster, h.index),
        None => (file.first_cluster, 0),
    };

    while index < target_index {
        cluster = next_cluster(params, cluster)
            .await?
            .ok_or(FatError::Corrupt)?;
        index += 1;
    }

    store_hint(SeekHint {
        first_cluster: file.first_cluster,
        index,
        cluster,
    })
    .await;

    Ok((cluster, within))
}

// ---------------------------------------------------------------------------
// Directory scan / file lookup by name
// ---------------------------------------------------------------------------

/// Find a file by its 8.3 name.
///
/// Scans the root directory, reading one 32-byte entry from flash at a time —
/// no buffer that grows with the file count.  Skips deleted (`0xE5`) entries,
/// long-filename fragments, volume labels and subdirectories.
///
/// Empty (`0x00`) slots are skipped rather than treated as end-of-directory:
/// the FAT spec says `0x00` terminates the listing, but hosts routinely leave
/// stale `0x00` slots between valid entries after files are added and removed
/// over USB mass storage, and stopping there would hide every file past the
/// hole.
///
/// Use [`to_8_3`] to convert a human-readable name like `"BADAPPLE.VID"` to
/// the 11-byte 8.3 form.
pub async fn find_file(name_8_3: &[u8; 11]) -> Result<FileRef, FatError> {
    let params = read_params().await?;
    let root_addr = params.root_dir_offset();

    let mut raw = [0u8; 32];
    for index in 0..params.root_entry_count as u32 {
        flash::read(root_addr + index * 32, &mut raw)
            .await
            .map_err(|_| FatError::FlashError)?;

        if raw[0] == 0x00 || raw[0] == 0xE5 {
            continue; // empty slot or deleted entry
        }
        if raw[11] & 0x0F == 0x0F {
            continue; // long-filename fragment
        }
        if raw[11] & 0x18 != 0 {
            continue; // volume label or directory
        }
        if &raw[0..11] != name_8_3 {
            continue;
        }

        // Directory entry layout:
        //   [0..11]  8.3 filename (8 name + 3 ext, space-padded, uppercase)
        //   [11]     attributes
        //   [26..28] first cluster number (little-endian u16)
        //   [28..32] file size in bytes (little-endian u32)
        return Ok(FileRef {
            first_cluster: u16::from_le_bytes([raw[26], raw[27]]),
            size: u32::from_le_bytes([raw[28], raw[29], raw[30], raw[31]]),
        });
    }
    Err(FatError::FileNotFound)
}

/// Convert a human-readable filename to 8.3 format.
///
/// `"HELLO.TXT"` → `b"HELLO   TXT"` (space-padded, uppercase).
/// Returns `None` if the name is too long or has no dot.
pub fn to_8_3(name: &str) -> Option<[u8; 11]> {
    let mut result = [b' '; 11];
    let bytes = name.as_bytes();
    let dot = bytes.iter().position(|&b| b == b'.')?;
    if dot > 8 || bytes.len() - dot - 1 > 3 {
        return None;
    }
    for (i, &b) in bytes[..dot].iter().enumerate() {
        result[i] = b.to_ascii_uppercase();
    }
    for (i, &b) in bytes[dot + 1..].iter().enumerate() {
        result[8 + i] = b.to_ascii_uppercase();
    }
    Some(result)
}

// ---------------------------------------------------------------------------
// File reading
// ---------------------------------------------------------------------------

/// Read up to `buf.len()` bytes from `file`, starting at byte `offset`.
///
/// Seeks along the FAT chain to the cluster containing `offset` (resuming from
/// the cached position when the seek is forward), then reads across as many
/// clusters as needed.
///
/// Returns the number of bytes actually read — less than `buf.len()` when the
/// file ends first, and `0` when `offset` is at or past the end of the file.
/// The [`FileRef`] is not consumed: reuse it for any number of reads at any
/// offsets.
pub async fn read_at(file: &FileRef, offset: u32, buf: &mut [u8]) -> Result<usize, FatError> {
    let params = read_params().await?;
    let cluster_bytes = params.cluster_bytes();

    let remaining = file.size.saturating_sub(offset) as usize;
    let to_read = buf.len().min(remaining);
    if to_read == 0 {
        return Ok(0);
    }

    // Clusters 0 and 1 are reserved; a directory entry claiming size > 0 with
    // first_cluster < 2 is corrupt (host-written, so untrusted). Reject rather
    // than reading from the data-region start.
    if file.first_cluster < 2 {
        return Err(FatError::Corrupt);
    }

    let (mut cluster, mut skip) = seek(&params, file, offset).await?;

    let mut bytes_read = 0usize;
    while bytes_read < to_read {
        let addr = params.cluster_addr(cluster) + skip;
        let chunk = (to_read - bytes_read).min((cluster_bytes - skip) as usize);
        flash::read(addr, &mut buf[bytes_read..bytes_read + chunk])
            .await
            .map_err(|_| FatError::FlashError)?;
        bytes_read += chunk;
        skip = 0; // only the first cluster has a skip offset

        if bytes_read < to_read {
            cluster = next_cluster(&params, cluster)
                .await?
                .ok_or(FatError::Corrupt)?;
        }
    }

    Ok(bytes_read)
}
