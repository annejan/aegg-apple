//! Random-access reads over an asset file.
//!
//! Both the video and the audio streams are far too large to hold in RAM, so
//! each is read a piece at a time: the audio in ADPCM blocks, the video a
//! frame at a time via its offset table. The player is written against this
//! trait rather than the FAT12 layer directly, which keeps the codecs
//! testable on the host.

/// A file that can be read at an arbitrary offset.
pub trait AssetRead {
    /// Fill `buf` from `offset` bytes into the file.
    ///
    /// Returns the number of bytes read, which is short only at end of file.
    async fn read_at(&mut self, offset: u32, buf: &mut [u8]) -> usize;
}
