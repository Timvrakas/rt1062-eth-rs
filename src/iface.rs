use smoltcp::phy::{self, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

use core::ptr::addr_of;
use core::ptr::null;

use bsp::ral;
use ral::enet;
use teensy4_bsp as bsp;
use core::sync::atomic;

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
        
        ral::write_reg!(enet,enet1,MSCR,MII_SPEED:9);
        ral::write_reg!(enet, enet1, EIMR, 0x0); //interupt mask: all off
        ral::modify_reg!(enet,enet1,RCR,RMII_MODE:1,MII_MODE:1,LOOP:0,PROM:1,CRCFWD:1,DRT:0,MAX_FL:1522,NLC:1,PADEN:1); //rmii, no loopback, no MAC filter
        ral::modify_reg!(enet,enet1,ECR, DBSWP:1); //swap endianess
        ral::modify_reg!(enet,enet1,TCR,FDEN:1); //enable full-duplex
        ral::modify_reg!(enet,enet1,TFWR,STRFWD:1); // store and fwd

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

        atomic::fence(atomic::Ordering::SeqCst); // let all config/DT memory sync

        //let's light this candle!
        ral::modify_reg!(enet,enet1,ECR,ETHEREN:1); // enable ethernet
        ral::write_reg!(enet,enet1,RDAR,RDAR:1); // RxDT ready for receive

        RT1062Phy::mdio_write(&enet1, 0, 0x18, 0x0280); // LED shows link status, active high
        RT1062Phy::mdio_write(&enet1, 0, 0x17, 0x0081); // config for 50 MHz clock input
    
        let rcsr = RT1062Phy::mdio_read(&enet1, 0, 0x17);
        let ledcr = RT1062Phy::mdio_read(&enet1, 0, 0x18);
        let phycr = RT1062Phy::mdio_read(&enet1, 0, 0x19);
        log::info!("RCSR:{rcsr}, LEDCR:{ledcr}, PHYCR:{phycr}");

        RT1062Phy {
            tx_pos: 0,
            rx_pos: 0,
        }
    }

    pub fn mdio_write(enet1: &ral::Instance<enet::RegisterBlock, 1>, phyaddr: u8, regaddr: u8, data: u16) {
        ral::write_reg!(enet,enet1,MMFR,ST:1,OP:1,TA:0, PA:phyaddr as u32, RA:regaddr as u32, DATA:data as u32);
        while ral::read_reg!(enet, enet1, EIR, MII) == 0 {}
        ral::write_reg!(enet,enet1,EIR,MII:1);
    }
    
    pub fn mdio_read(enet1: &ral::Instance<enet::RegisterBlock, 1>, phyaddr: u8, regaddr: u8) -> u16 {
        ral::write_reg!(enet,enet1,MMFR,ST:1,OP:2,TA:0, PA:phyaddr as u32, RA:regaddr as u32);
        while ral::read_reg!(enet, enet1, EIR, MII) == 0 {}
        let data = ral::read_reg!(enet, enet1, MMFR, DATA) as u16;
        ral::write_reg!(enet,enet1,EIR,MII:1);
        return data;
    }

}

impl phy::Device for RT1062Phy {
    //these statements, and the associated borrow logic, specifies that the token has the lifetime of the RT1062Phy object. This means there can only ever be each of the TX and RX Tokens.
    type RxToken<'a> = RT1062PhyRxToken<'a> where Self: 'a;
    type TxToken<'a> = RT1062PhyTxToken<'a> where Self: 'a;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        unsafe {
            let rxd = &mut RXDT.rxdt[self.rx_pos];
            if (rxd.flags & 0x8000) == 0 {
                let ret = Some((
                    RT1062PhyRxToken {
                        rx_pos: &mut self.rx_pos,
                    },
                    RT1062PhyTxToken {
                        tx_pos: &mut self.tx_pos,
                    },
                ));
                return ret;
            } else {
                return None;
            }
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        unsafe {
            let desc: &mut TxDescriptor = &mut TXDT.txdt[self.tx_pos];
            if (desc.flags & 0x8000) == 0x0 {
                let tok = RT1062PhyTxToken {
                    tx_pos: &mut self.tx_pos,
                };
                return Some(tok);
            } else {
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

pub struct RT1062PhyRxToken<'a> {
    rx_pos: &'a mut usize,
}

impl<'a> phy::RxToken for RT1062PhyRxToken<'a> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        unsafe {
            let rxd = &mut RXDT.rxdt[*self.rx_pos];
            let result = f(&mut BUFS.rx[*self.rx_pos]);
            atomic::fence(atomic::Ordering::SeqCst);
            rxd.flags |= 0x8000;
            if *self.rx_pos < (RXDT.rxdt.len() - 1) {
                *self.rx_pos += 1;
            } else {
                *self.rx_pos = 0;
            }
            result
        }
    }
}

pub struct RT1062PhyTxToken<'a> {
    tx_pos: &'a mut usize,
}

impl<'a> phy::TxToken for RT1062PhyTxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        unsafe {
            let enet1 = enet::ENET1::instance();
            let result = f(&mut BUFS.tx[*self.tx_pos]);
            let desc: &mut TxDescriptor = &mut TXDT.txdt[*self.tx_pos];
            desc.len = len as u16;
            desc.flags |= 0x8C00;
            atomic::fence(atomic::Ordering::SeqCst);
            ral::write_reg!(enet,enet1,TDAR,TDAR:1);

            if *self.tx_pos < (TXDT.txdt.len() - 1) {
                *self.tx_pos += 1;
            } else {
                *self.tx_pos = 0;
            }
            result
        }
    }
}
