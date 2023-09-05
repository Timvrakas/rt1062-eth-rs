use smoltcp::phy::{self, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

use core::ptr::addr_of;
use core::ptr::null;

use teensy4_bsp as bsp;
use bsp::ral;
use ral::enet;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct RxDescriptor {
    len: u16,
    flags: u16,
    buffer: *const u8,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct TxDescriptor {
    len: u16,
    flags: u16,
    buffer: *const u8,
}

#[repr(C)]
#[repr(align(64))]
pub struct RxDT {
    rxdt: [RxDescriptor; 12],
}

#[repr(C)]
#[repr(align(64))]
pub struct TxDT {
    txdt: [TxDescriptor; 10],
}

#[repr(C)]
#[repr(align(32))]
pub struct Bufs {
    tx: [[u8; 512]; 10],
    rx: [[u8; 512]; 12],
}

static mut RXDT: RxDT = RxDT {
    rxdt: [RxDescriptor {
        len: 0,
        flags: 0x8000,
        buffer: null(),
    }; 12],
};
static mut TXDT: TxDT = TxDT {
    txdt: [TxDescriptor {
        len: 0,
        flags: 0,
        buffer: null(),
    }; 10],
};

static mut BUFS: Bufs = Bufs {
    tx: [[0x0; 512]; 10],
    rx: [[0x0; 512]; 12],
};

pub struct RT1062Phy {
    tx_pos: usize,
    rx_pos: usize,
}

impl<'a> RT1062Phy {
    pub fn new() -> RT1062Phy {

        let enet1 = unsafe { enet::ENET1::instance() };

        unsafe {
            for (idx, element) in TXDT.txdt.iter_mut().enumerate() {
                element.buffer = addr_of!(BUFS.tx[idx][0]);
            }
    
            TXDT.txdt.last_mut().unwrap().flags = 0x2000;
    
            for (idx, element) in RXDT.rxdt.iter_mut().enumerate() {
                element.buffer = addr_of!(BUFS.rx[idx][0]);
            }
    
            RXDT.rxdt.last_mut().unwrap().flags = 0xA000;
        }

        unsafe {
            ral::write_reg!(enet, enet1, RDSR, addr_of!(RXDT.rxdt[0]) as u32); //circular receive buffer descriptor queue
            ral::write_reg!(enet, enet1, TDSR, addr_of!(TXDT.txdt[0]) as u32); //circular transmit buffer descriptor queue
            ral::write_reg!(enet,enet1,MRBR, R_BUF_SIZE:512); //maximum size of all receive buffers
        }

        log::info!("Set Up Descriptor Tables!");

        RT1062Phy {
            tx_pos: 0,
            rx_pos: 0,
        }
    }
}

impl phy::Device for RT1062Phy {
    type RxToken<'a> = RT1062PhyRxToken<> where Self: 'a;
    type TxToken<'a> = RT1062PhyTxToken<> where Self: 'a;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        unsafe {
            let rxd = &mut RXDT.rxdt[self.rx_pos];
            if (rxd.flags & 0x8000) == 0 {

                let ret = Some((RT1062PhyRxToken(self.rx_pos), RT1062PhyTxToken(self.tx_pos)));
                log::info!("minted RX token {} with echo TX token {}",self.rx_pos,self.tx_pos);

                if self.rx_pos < (RXDT.rxdt.len() - 1) {
                    self.rx_pos += 1;
                } else {
                    self.rx_pos = 0;
                }

                if self.tx_pos < (TXDT.txdt.len() - 1) {
                    self.tx_pos += 1;
                } else {
                    self.tx_pos = 0;
                }

                return ret;
            }else{
                return None
            }

        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> { // this is wrong, because the TxToken might not be used, so we could run out of descriptors :P
        unsafe {
            let desc: &mut TxDescriptor = &mut TXDT.txdt[self.tx_pos];
            if (desc.flags & 0x8000) == 0x0 {
                let tok = RT1062PhyTxToken(self.tx_pos);
                log::info!("minted TX token {}",self.tx_pos);
                if self.tx_pos < (TXDT.txdt.len() - 1) {
                    self.tx_pos += 1;
                } else {
                    self.tx_pos = 0;
                }
                return Some(tok);
            }else{
                return None;
            }
        }
    }

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1536;
        caps.max_burst_size = Some(1);
        caps.medium = Medium::Ethernet;
        caps
    }
}

pub struct RT1062PhyRxToken<>(usize);

impl<'a> phy::RxToken for RT1062PhyRxToken<> {
    fn consume<R, F>(self, f: F) -> R
        where F: FnOnce(&mut [u8]) -> R
    {
        unsafe{
            let rxd = &mut RXDT.rxdt[self.0];
            let result = f(&mut BUFS.rx[self.0]);
            log::info!("consumed RX token {}",self.0);
            rxd.flags |= 0x8000;
            result
        }
    }
}

pub struct RT1062PhyTxToken<>(usize);

impl<'a> phy::TxToken for RT1062PhyTxToken<> {
    fn consume<R, F>(self, len: usize, f: F) -> R
        where F: FnOnce(&mut [u8]) -> R
    {
        unsafe{
            let enet1 = enet::ENET1::instance();
            let result = f(&mut BUFS.tx[self.0]);
            let desc: &mut TxDescriptor = &mut TXDT.txdt[self.0];
            desc.len = len as u16;
            desc.flags |= 0x8C00;
            ral::write_reg!(enet,enet1,TDAR,TDAR:1);
            log::info!("consumed TX token {}",self.0);
            result
        }
    }
}

//This is incomplete, because it dosen't protect against hoarding of tokens,
//and doesn't protect against arbitrary consuption order, which I think could result in a data race.
// every time you mint an RX token, you also mint a TX token which you don't use!