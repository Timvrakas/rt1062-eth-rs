use core::ptr::null;
use core::ptr::addr_of;
use static_assertions::assert_eq_size;
use struct_pad::*;

use struct_pad::PadU16;
use struct_pad::PadU8;
use imxrt_hal as hal;
use hal::timer::Blocking;

#[repr(C)]
#[derive(Clone, Copy)]
#[repr(align(8))]
pub struct RxDescriptor {
    pub len: u16,
    pub flags: u16,
    pub buffer: *const u8,
    pub flags2: u16,
    pub flags3: u16,
    pub checksum: u16,
    pub protocol: u8,
    pub header_len: u8,
    pad: PadU16,
    pad2: PadU8,
    pub bdu: u8,
    pub timestamp: u32,
    pad3: PadU32,
    pad4: PadU32,
}

assert_eq_size!(RxDescriptor, [u8; 32]);

#[repr(C)]
#[derive(Clone, Copy)]
#[repr(align(8))]
pub struct TxDescriptor {
    pub len: u16,
    pub flags: u16,
    pub buffer: *const u8,
    pub flags2: u16,
    pub flags3: u16,
    pad: PadU32,
    pad2: PadU16,
    pad3: PadU8,
    pub bdu: u8,
    pub timestamp: u32,
    pad4: PadU32,
    pad5: PadU32,
}

assert_eq_size!(TxDescriptor, [u8; 32]);

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

impl<const MTU: usize, const LEN: usize> RxDT<MTU, LEN> {
    pub const fn default() -> RxDT<MTU, LEN> {
        RxDT {
            desc: [RxDescriptor {
                len: 0,
                flags: 0x8000,
                buffer: null(),
                flags2: 0x0000,
                flags3: 0x0000,
                checksum: 0x0000,
                protocol: 0x00,
                header_len: 0x00,
                pad: Pad::VALUE,
                pad2: Pad::VALUE,
                bdu: 0x00,
                timestamp: 0,
                pad3: Pad::VALUE,
                pad4: Pad::VALUE
            }; LEN],
            bufs: [[0x0; MTU]; LEN],
        }
    }
}

impl<const MTU: usize, const LEN: usize> TxDT<MTU, LEN> {
    pub const fn default() -> TxDT<MTU, LEN> {
        TxDT {
            desc: [TxDescriptor {
                len: 0,
                flags: 0,
                buffer: null(),
                flags2: 0x0000,
                flags3: 0x0000,
                pad: Pad::VALUE,
                pad2: Pad::VALUE,
                pad3: Pad::VALUE,
                bdu: 0x00,
                timestamp: 0,
                pad4: Pad::VALUE,
                pad5: Pad::VALUE
            }; LEN],
            bufs: [[0x0; MTU]; LEN],
        }
    }
}

#[allow(dead_code)]
pub fn print_dt<const MTU: usize, const TX_LEN: usize, const RX_LEN:usize>(
    delay: &mut Blocking<hal::gpt::Gpt<1>, 1000>,
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
