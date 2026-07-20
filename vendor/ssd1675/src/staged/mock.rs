//! `#[cfg(test)]` recording `DisplayInterface` — captures the command/data
//! stream so executor tests can assert the exact byte sequence sent.

use crate::interface::DisplayInterface;
use std::vec::Vec;

/// One recorded bus event.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Event {
    Cmd(u8),
    Data(Vec<u8>),
    BusyWaitForCompletion,
}

#[derive(Default)]
pub struct MockInterface {
    pub log: Vec<Event>,
}

impl DisplayInterface for MockInterface {
    type Error = core::convert::Infallible;

    async fn send_command(&mut self, command: u8) -> Result<(), Self::Error> {
        self.log.push(Event::Cmd(command));
        Ok(())
    }
    async fn send_data(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.log.push(Event::Data(data.to_vec()));
        Ok(())
    }
    async fn reset(&mut self) {}
    async fn busy_wait(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    async fn busy_wait_for_completion(&mut self) -> Result<(), Self::Error> {
        self.log.push(Event::BusyWaitForCompletion);
        Ok(())
    }
}

impl MockInterface {
    /// Commands sent, in order.
    pub fn commands(&self) -> Vec<u8> {
        self.log
            .iter()
            .filter_map(|e| if let Event::Cmd(c) = e { Some(*c) } else { None })
            .collect()
    }
}

#[test]
fn mock_records_commands() {
    // futures::executor isn't a dep; drive the future with a trivial block_on.
    let mut m = MockInterface::default();
    pollster_block_on(async {
        m.send_command(0x20).await.unwrap();
        m.send_data(&[1, 2, 3]).await.unwrap();
    });
    assert_eq!(m.commands(), std::vec![0x20]);
    assert_eq!(m.log[1], Event::Data(std::vec![1, 2, 3]));
}

/// Minimal no-dep `block_on` for tests (the executor futures here never pend).
///
/// `pub(crate)` so sibling test modules (e.g. the executor's) can reuse it.
pub(crate) fn pollster_block_on<F: core::future::Future>(mut f: F) -> F::Output {
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    fn raw() -> RawWaker {
        fn no_op(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            raw()
        }
        RawWaker::new(core::ptr::null(), &RawWakerVTable::new(clone, no_op, no_op, no_op))
    }
    let waker = unsafe { Waker::from_raw(raw()) };
    let mut cx = Context::from_waker(&waker);
    let mut f = unsafe { core::pin::Pin::new_unchecked(&mut f) };
    loop {
        if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
            return v;
        }
    }
}
