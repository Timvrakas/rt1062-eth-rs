use core::ptr::null;
use core::ptr::addr_of;

#[repr(C)]
#[derive(Clone, Copy)]
#[repr(align(8))]
pub struct RxDescriptor {
    pub len: u16,
    pub flags: u16,
    pub buffer: *const u8,
}

#[repr(C)]
#[derive(Clone, Copy)]
#[repr(align(8))]
pub struct TxDescriptor {
    pub len: u16,
    pub flags: u16,
    pub buffer: *const u8,
}

#[repr(C)]
#[repr(align(64))]
pub struct RxDT<const MTU: usize, const LEN: usize> {
    pub desc: [RxDescriptor; LEN],
    pub bufs: [[u8; MTU]; LEN],
}

#[repr(C)]
#[repr(align(64))]
pub struct TxDT<const MTU: usize, const LEN: usize> {
    pub desc: [TxDescriptor; LEN],
    pub bufs: [[u8; MTU]; LEN],
}

impl<const MTU: usize, const LEN: usize> Default for RxDT<MTU, LEN> {
    fn default() -> RxDT<MTU, LEN> {
        RxDT {
            desc: [RxDescriptor {
                len: 0,
                flags: 0x8000,
                buffer: null(),
            }; LEN],
            bufs: [[0x0; MTU]; LEN],
        }
    }
}

impl<const MTU: usize, const LEN: usize> Default for TxDT<MTU, LEN> {
    fn default() -> TxDT<MTU, LEN> {
        TxDT {
            desc: [TxDescriptor {
                len: 0,
                flags: 0,
                buffer: null(),
            }; LEN],
            bufs: [[0x0; MTU]; LEN],
        }
    }
}

/// The callable `delay_ms` should, when invoked, block execution for the supplied
/// number of milliseconds.
#[allow(dead_code)]
pub fn print_dt<B, const MTU: usize, const TX_LEN: usize, const RX_LEN:usize>(
    mut delay_ms: impl FnMut(B),
    txdt: &TxDT<MTU, TX_LEN>,
    rxdt: &RxDT<MTU, RX_LEN>,
) where
    B: From<u8>,
{
    delay_ms(10.into());

    log::info!("=== Descriptor Tables ===");

    log::info!("TxDT Base Addr: {:#08x}", addr_of!(txdt.desc[0]) as u32);
    delay_ms(10.into());
    for (idx, el) in txdt.desc.iter().enumerate() {
        log::info!(
            "TXDT[{idx:<2}] -- addr: {:#08x}, flags: {:#06x}, len: {:<3}",
            el.buffer as u32,
            el.flags,
            el.len
        );
        delay_ms(10.into());
    }

    log::info!("RxDT Base Addr: {:#08x}", addr_of!(rxdt.desc[0]) as u32);
    delay_ms(10.into());
    for (idx, el) in rxdt.desc.iter().enumerate() {
        log::info!(
            "RXDT[{idx:<2}] -- addr: {:#08x}, flags: {:#06x}, len: {:<3}",
            el.buffer as u32,
            el.flags,
            el.len
        );
        delay_ms(10.into());
    }
    log::info!("=================");
}
