//! Read-only external QSPI flash access, mutex-protected.
//!
//! The external QSPI flash (2 MiB) on the badge is partitioned into:
//!   - **ekv KV store** — first 1 MiB (0x000000–0x0FFFFF)
//!   - **FAT12 / USB mass storage** — second 1 MiB (0x100000–0x1FFFFF)
//!
//! This firmware only reads the FAT12 half, where the video and audio assets
//! live.  It **never** erases and **never** writes: the first MiB belongs to
//! the stock badge firmware's `ekv` store (contacts, settings, keys) and must
//! survive a run of Bad Apple untouched, so no write or erase primitive is
//! exposed at all — the partition constants below exist purely to document the
//! layout and to bound the FAT12 reader.
//!
//! The flash lives behind an async mutex so the video and audio tasks, which
//! both stream from it concurrently, are serialised automatically.

use embassy_nrf::{Peri, bind_interrupts, peripherals, qspi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

// ---------------------------------------------------------------------------
// Flash geometry
// ---------------------------------------------------------------------------

/// One erase sector (4 KiB).  Nothing here erases; this is the bounce-buffer
/// size and the unit the FAT12 geometry is expressed in.
pub const PAGE_SIZE: usize = 4096;

/// Flash chip capacity in bytes (2 MiB).
pub const FLASH_TOTAL_BYTES: usize = 2 * 1024 * 1024;

/// ekv KV store partition: first 1 MiB (0x000000–0x0FFFFF).  **Off limits.**
pub const KV_BYTES: usize = 1024 * 1024;

/// FAT12 partition: second 1 MiB (0x100000–0x1FFFFF).  The assets live here.
pub const FAT_OFFSET: u32 = KV_BYTES as u32;
pub const FAT_BYTES: usize = FLASH_TOTAL_BYTES - KV_BYTES;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FlashError {
    OutOfBounds,
    Hardware,
    NotInitialised,
}

// ---------------------------------------------------------------------------
// Singleton QSPI instance + aligned DMA buffer
// ---------------------------------------------------------------------------

bind_interrupts!(struct QspiIrqs {
    QSPI => qspi::InterruptHandler<peripherals::QSPI>;
});

#[repr(C, align(4))]
struct AlignedBuf([u8; PAGE_SIZE]);

static FLASH: Mutex<CriticalSectionRawMutex, Option<(qspi::Qspi<'static>, AlignedBuf)>> =
    Mutex::new(None);

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

/// Initialise the QSPI peripheral and verify the flash chip via JEDEC ID.
///
/// Call once at startup before any flash access.  Panics (with the ID read
/// back) if the chip does not answer — without the flash there are no assets
/// to play, so there is nothing sensible to fall back to.
#[allow(clippy::too_many_arguments)]
pub async fn init(
    qspi_periph: Peri<'_, peripherals::QSPI>,
    sck: Peri<'_, peripherals::P0_21>,
    csn: Peri<'_, peripherals::P0_25>,
    io0: Peri<'_, peripherals::P0_20>,
    io1: Peri<'_, peripherals::P0_24>,
    io2: Peri<'_, peripherals::P0_22>,
    io3: Peri<'_, peripherals::P0_23>,
) {
    let mut cfg = qspi::Config::default();
    cfg.capacity = FLASH_TOTAL_BYTES as u32;
    cfg.read_opcode = qspi::ReadOpcode::FASTREAD;
    cfg.write_opcode = qspi::WriteOpcode::PP;
    // Run the bus at the nRF52840 ceiling.  ZD25WQ16C's FAST READ supports
    // up to 80 MHz; nRF QSPI caps at 32 MHz.  4× over the M8 default with
    // no change to read opcodes — FASTREAD already includes a dummy cycle,
    // so timing margin at 32 MHz is comfortable.  Video playback reads a
    // frame per panel refresh, so the headroom is worth having.
    cfg.frequency = qspi::Frequency::M32;

    let mut qspi = qspi::Qspi::new(qspi_periph, QspiIrqs, sck, csn, io0, io1, io2, io3, cfg);

    let mut jedec = [0u8; 3];
    let _ = qspi.blocking_custom_instruction(0x9F, &[], &mut jedec);
    if jedec == [0xFF; 3] || jedec == [0x00; 3] {
        defmt::panic!(
            "QSPI flash not reachable (JEDEC ID: {:02X} {:02X} {:02X})",
            jedec[0],
            jedec[1],
            jedec[2],
        );
    }

    defmt::info!(
        "QSPI flash JEDEC ID: {:02X} {:02X} {:02X}",
        jedec[0],
        jedec[1],
        jedec[2],
    );

    // Safety: init() is called from main() which never returns, so the
    // peripheral borrows outlive every use of the stored instance.
    let qspi: qspi::Qspi<'static> = unsafe { core::mem::transmute(qspi) };

    let mut guard = FLASH.lock().await;
    *guard = Some((qspi, AlignedBuf([0u8; PAGE_SIZE])));
}

// ---------------------------------------------------------------------------
// Flash operations (mutex-protected, read-only)
// ---------------------------------------------------------------------------

/// Read bytes from an absolute flash address.
///
/// Handles QSPI alignment requirements internally: the address and length
/// passed to the hardware are always 4-byte aligned, and the destination is
/// the static aligned bounce buffer, so callers may pass any `&mut [u8]` at
/// any offset.
pub async fn read(addr: u32, data: &mut [u8]) -> Result<(), FlashError> {
    let end = (addr as usize)
        .checked_add(data.len())
        .ok_or(FlashError::OutOfBounds)?;
    if end > FLASH_TOTAL_BYTES {
        return Err(FlashError::OutOfBounds);
    }
    let mut guard = FLASH.lock().await;
    let (qspi, buf) = guard.as_mut().ok_or(FlashError::NotInitialised)?;

    let mut remaining = data.len();
    let mut data_off = 0usize;
    let mut flash_addr = addr;

    while remaining > 0 {
        // Align address down to 4 bytes.
        let aligned_addr = flash_addr & !3;
        let skip = (flash_addr - aligned_addr) as usize;
        // Read enough to cover skip + remaining, rounded up to 4 bytes, capped
        // to the bounce buffer.
        let raw_len = ((skip + remaining + 3) & !3).min(PAGE_SIZE);
        qspi.read(aligned_addr, &mut buf.0[..raw_len])
            .await
            .map_err(|_| FlashError::Hardware)?;
        let usable = (raw_len - skip).min(remaining);
        data[data_off..data_off + usable].copy_from_slice(&buf.0[skip..skip + usable]);
        data_off += usable;
        flash_addr += usable as u32;
        remaining -= usable;
    }
    Ok(())
}
