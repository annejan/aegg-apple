//! Thin stage executor: upload LUT, upload planes, trigger one panel activation.

use super::lut::{StageLut, MAX_BODY};
use crate::interface::DisplayInterface;

/// A rectangular RAM window, byte-aligned in X (matching the partial-update path).
#[derive(Clone, Copy, Debug)]
pub struct Region {
    /// First RAM column byte (inclusive), as used by command `0x44`.
    pub x_start_byte: u8,
    /// Last RAM column byte (inclusive), as used by command `0x44`.
    pub x_end_byte: u8,
    /// First RAM row (inclusive), as used by command `0x45`.
    pub y_start: u16,
    /// Last RAM row (inclusive), as used by command `0x45`.
    pub y_end: u16,
}

/// Upload `lut` to the LUT register (cmd 0x32) for the given variant, then push
/// the temperature voltage trailer.
///
/// `apply_trailer` is the crate's existing per-temperature trailer routine
/// (passed in so the executor stays state-free); it is invoked once after the
/// LUT body has been written.
///
/// # Arguments
///
/// * `iface` - the display interface to drive.
/// * `lut` - the stage LUT to encode and upload.
/// * `b_variant` - `true` for SSD1675B layout, `false` for SSD1675A.
/// * `apply_trailer` - closure pushing the temperature voltage trailer.
///
/// # Errors
///
/// Propagates any `I::Error` from the underlying bus writes or the trailer.
pub async fn upload_lut<I, F>(
    iface: &mut I,
    lut: &StageLut,
    b_variant: bool,
    apply_trailer: F,
) -> Result<(), I::Error>
where
    I: DisplayInterface,
    F: AsyncFnOnce(&mut I) -> Result<(), I::Error>,
{
    let mut body = [0u8; MAX_BODY];
    let len = lut.encode(b_variant, &mut body);
    iface.send_command(0x32).await?;
    iface.send_data(&body[..len]).await?;
    apply_trailer(iface).await
}

/// Set the RAM window (cmds 0x44/0x45/0x4E/0x4F) and write both planes
/// (cmd 0x26 RED, then 0x24 BW).  Planes are the already-windowed bytes.
///
/// # Arguments
///
/// * `iface` - the display interface to drive.
/// * `bw_plane` - the windowed black/white plane bytes (cmd 0x24).
/// * `red_plane` - the windowed red plane bytes (cmd 0x26).
/// * `region` - the RAM window to address.
///
/// # Errors
///
/// Propagates any `I::Error` from the underlying bus writes.
pub async fn upload_planes<I: DisplayInterface>(
    iface: &mut I,
    bw_plane: &[u8],
    red_plane: &[u8],
    region: Region,
) -> Result<(), I::Error> {
    iface.send_command(0x44).await?;
    iface.send_data(&[region.x_start_byte, region.x_end_byte]).await?;
    iface.send_command(0x45).await?;
    iface
        .send_data(&[
            region.y_start as u8,
            (region.y_start >> 8) as u8,
            region.y_end as u8,
            (region.y_end >> 8) as u8,
        ])
        .await?;
    iface.send_command(0x4E).await?;
    iface.send_data(&[region.x_start_byte]).await?;
    iface.send_command(0x4F).await?;
    iface.send_data(&[region.y_start as u8, (region.y_start >> 8) as u8]).await?;
    iface.send_command(0x26).await?; // red plane
    iface.send_data(red_plane).await?;
    iface.send_command(0x4E).await?;
    iface.send_data(&[region.x_start_byte]).await?;
    iface.send_command(0x4F).await?;
    iface.send_data(&[region.y_start as u8, (region.y_start >> 8) as u8]).await?;
    iface.send_command(0x24).await?; // bw plane
    iface.send_data(bw_plane).await?;
    Ok(())
}

/// Run one stage: BorderWaveform(0x80) → Option2(0xC7 Mode1) → UpdateDisplay(0x20)
/// → wait for completion.  One call = one panel activation.
///
/// # Arguments
///
/// * `iface` - the display interface to drive.
///
/// # Errors
///
/// Propagates any `I::Error` from the underlying bus writes or the busy wait.
pub async fn trigger_stage<I: DisplayInterface>(iface: &mut I) -> Result<(), I::Error> {
    iface.send_command(0x3C).await?; // BorderWaveform
    iface.send_data(&[0x80]).await?;
    iface.send_command(0x22).await?; // DisplayUpdateOption2
    iface.send_data(&[0xC7]).await?; // Mode1, no temp/LUT reload
    iface.send_command(0x20).await?; // UpdateDisplay
    iface.busy_wait_for_completion().await
}

#[cfg(test)]
mod tests {
    use super::super::mock::{Event, MockInterface};
    use super::*;

    fn block_on<F: core::future::Future>(f: F) -> F::Output {
        super::super::mock::pollster_block_on(f)
    }

    #[test]
    fn trigger_emits_border_option_update_busy() {
        let mut m = MockInterface::default();
        block_on(trigger_stage(&mut m)).unwrap();
        assert_eq!(m.commands(), std::vec![0x3C, 0x22, 0x20]);
        assert_eq!(*m.log.last().unwrap(), Event::BusyWaitForCompletion);
        // Mode1 Option2 value.
        assert!(m.log.contains(&Event::Data(std::vec![0xC7])));
        // Border follow-source.
        assert!(m.log.contains(&Event::Data(std::vec![0x80])));
    }

    #[test]
    fn upload_planes_sets_window_then_writes_both() {
        let mut m = MockInterface::default();
        let region = Region { x_start_byte: 0, x_end_byte: 18, y_start: 0, y_end: 151 };
        block_on(upload_planes(&mut m, &[0xAA, 0xBB], &[0x11, 0x22], region)).unwrap();
        let cmds = m.commands();
        // window, red, window, bw
        assert_eq!(cmds, std::vec![0x44, 0x45, 0x4E, 0x4F, 0x26, 0x4E, 0x4F, 0x24]);
        assert!(m.log.contains(&Event::Data(std::vec![0x11, 0x22])), "red plane bytes");
        assert!(m.log.contains(&Event::Data(std::vec![0xAA, 0xBB])), "bw plane bytes");
    }

    #[test]
    fn upload_lut_writes_0x32_then_body_then_trailer() {
        let mut m = MockInterface::default();
        let lut = StageLut {
            n_phases: 1,
            black: Default::default(),
            white: Default::default(),
            red: None,
            vcom: Default::default(),
            tp: Default::default(),
        };
        let mut trailer_called = false;
        block_on(upload_lut(&mut m, &lut, false, async |iface: &mut MockInterface| {
            trailer_called = true;
            iface.send_command(0x3A).await?; // stand-in trailer cmd
            Ok(())
        }))
        .unwrap();
        assert!(trailer_called);
        assert_eq!(m.commands().first(), Some(&0x32));
        // body length for A variant is 70 bytes.
        if let Event::Data(d) = &m.log[1] {
            assert_eq!(d.len(), 70);
        } else {
            panic!("expected LUT body data");
        }
    }
}
