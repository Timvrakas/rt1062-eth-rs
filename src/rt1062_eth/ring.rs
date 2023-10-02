use core::ptr::null;
use core::ptr::addr_of;

use teensy4_bsp as bsp;
use teensy4_bsp::hal::timer::Blocking;

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

#[allow(dead_code)]
pub fn print_dt<const MTU: usize, const TX_LEN: usize, const RX_LEN:usize>(
    delay: &mut Blocking<bsp::hal::gpt::Gpt<1>, 1000>,
    txdt: &TxDT<MTU, TX_LEN>,
    rxdt: &RxDT<MTU, RX_LEN>,
) {
    delay.block_ms(10);

    log::info!("=== Descriptor Tables ===");

    log::info!("TxDT Base Addr: {:#08x}", addr_of!(txdt.desc[0]) as u32);
    delay.block_ms(10);
    for (idx, el) in txdt.desc.iter().enumerate() {
        log::info!(
            "TXDT[{idx:<2}] -- addr: {:#08x}, flags: {:#06x}, len: {:<3}",
            el.buffer as u32,
            el.flags,
            el.len
        );
        delay.block_ms(10);
    }

    log::info!("RxDT Base Addr: {:#08x}", addr_of!(rxdt.desc[0]) as u32);
    delay.block_ms(10);
    for (idx, el) in rxdt.desc.iter().enumerate() {
        log::info!(
            "RXDT[{idx:<2}] -- addr: {:#08x}, flags: {:#06x}, len: {:<3}",
            el.buffer as u32,
            el.flags,
            el.len
        );
        delay.block_ms(10);
    }
    log::info!("=================");
}
