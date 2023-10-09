use smoltcp::phy::{self, DeviceCapabilities, Medium};
use smoltcp::time::Instant;

use core::ptr::addr_of;

use bsp::ral;
use core::sync::atomic;
use ral::enet;
use teensy4_bsp as bsp;

pub mod ring;
use ring::*;

pub struct RT1062Phy<'a, const INST: u8, const MTU: usize, const RX_LEN: usize, const TX_LEN: usize>
{
    tx_pos: usize,
    rx_pos: usize,
    enet_inst: enet::Instance<INST>,
    pub txdt: &'a mut TxDT<MTU, TX_LEN>,
    pub rxdt: &'a mut RxDT<MTU, RX_LEN>,
}

impl<'a, const INST: u8, const MTU: usize, const RX_LEN: usize, const TX_LEN: usize>
    RT1062Phy<'a, INST, MTU, RX_LEN, TX_LEN>
{
    pub fn new(
        enet_inst: enet::Instance<INST>,
        rxdt: &'a mut RxDT<MTU, RX_LEN>,
        txdt: &'a mut TxDT<MTU, TX_LEN>,
    ) -> RT1062Phy<'a, INST, MTU, RX_LEN, TX_LEN> {
        let device = RT1062Phy {
            tx_pos: 0,
            rx_pos: 0,
            enet_inst: enet_inst,
            rxdt: rxdt,
            txdt: txdt,
        };

        ral::write_reg!(enet,device.enet_inst,MSCR,MII_SPEED:9);
        ral::write_reg!(enet, device.enet_inst, EIMR, 0x0); //interupt mask: all off
        ral::modify_reg!(enet,device.enet_inst,RCR,RMII_MODE:1,MII_MODE:1,LOOP:0,PROM:1,CRCFWD:1,DRT:0,MAX_FL:1522,NLC:1,PADEN:1); //rmii, no loopback, no MAC filter
        ral::modify_reg!(enet,device.enet_inst,ECR, DBSWP:1); //swap endianess
        ral::modify_reg!(enet,device.enet_inst,TCR,FDEN:1); //enable full-duplex
        ral::modify_reg!(enet,device.enet_inst,TFWR,STRFWD:1); // store and fwd

        for (idx, element) in device.txdt.desc.iter_mut().enumerate() {
            element.buffer = addr_of!(device.txdt.bufs[idx][0]);
        }

        device.txdt.desc.last_mut().unwrap().flags = 0x2000;

        for (idx, element) in device.rxdt.desc.iter_mut().enumerate() {
            element.buffer = addr_of!(device.rxdt.bufs[idx][0]);
        }

        device.rxdt.desc.last_mut().unwrap().flags = 0xA000;

        ral::write_reg!(
            enet,
            device.enet_inst,
            RDSR,
            addr_of!(device.rxdt.desc[0]) as u32
        ); //circular receive buffer descriptor queue
        ral::write_reg!(
            enet,
            device.enet_inst,
            TDSR,
            addr_of!(device.txdt.desc[0]) as u32
        ); //circular transmit buffer descriptor queue

        ral::write_reg!(enet,device.enet_inst,MRBR, R_BUF_SIZE:512); //maximum size of all receive buffers

        log::info!("Set Up Descriptor Tables!");
        log::info!("RXDT start: {:#08x}", addr_of!(device.rxdt.desc[0]) as u32);
        log::info!("TXDT start: {:#08x}", addr_of!(device.txdt.desc[0]) as u32);

        atomic::fence(atomic::Ordering::SeqCst); // let all config/DT memory sync

        //let's light this candle!
        ral::modify_reg!(enet,device.enet_inst,ECR,ETHEREN:1); // enable ethernet
        ral::write_reg!(enet,device.enet_inst,RDAR,RDAR:1); // RxDT ready for receive

        return device;
    }

    pub fn mdio_write(&mut self, phyaddr: u8, regaddr: u8, data: u16) {
        ral::write_reg!(enet,self.enet_inst,MMFR,ST:1,OP:1,TA:0, PA:phyaddr as u32, RA:regaddr as u32, DATA:data as u32);
        while ral::read_reg!(enet, self.enet_inst, EIR, MII) == 0 {}
        ral::write_reg!(enet,self.enet_inst,EIR,MII:1);
    }

    pub fn mdio_read(&mut self, phyaddr: u8, regaddr: u8) -> u16 {
        ral::write_reg!(enet,self.enet_inst,MMFR,ST:1,OP:2,TA:0, PA:phyaddr as u32, RA:regaddr as u32);
        while ral::read_reg!(enet, self.enet_inst, EIR, MII) == 0 {}
        let data = ral::read_reg!(enet, self.enet_inst, MMFR, DATA) as u16;
        ral::write_reg!(enet,self.enet_inst,EIR,MII:1);
        return data;
    }
}

impl<const INST: u8, const MTU: usize, const RX_LEN: usize, const TX_LEN: usize> phy::Device
    for RT1062Phy<'_, INST, MTU, RX_LEN, TX_LEN>
{
    //these statements, and the associated borrow logic, specifies that the token has the lifetime of the RT1062Phy object. This means there can only ever be each of the TX and RX Tokens.
    type RxToken<'a> = RT1062PhyRxToken<'a, MTU, RX_LEN> where Self: 'a;
    type TxToken<'a> = RT1062PhyTxToken<'a, INST, MTU, TX_LEN> where Self: 'a;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let rxd = &mut self.rxdt.desc[self.rx_pos];
        if (rxd.flags & 0x8000) == 0 {
            let ret = Some((
                RT1062PhyRxToken {
                    rx_pos: &mut self.rx_pos,
                    rxdt: &mut self.rxdt,
                },
                RT1062PhyTxToken {
                    tx_pos: &mut self.tx_pos,
                    enet_inst: &mut self.enet_inst,
                    txdt: &mut self.txdt,
                },
            ));
            return ret;
        } else {
            return None;
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        let desc: &mut TxDescriptor = &mut self.txdt.desc[self.tx_pos];
        if (desc.flags & 0x8000) == 0x0 {
            let tok: RT1062PhyTxToken<'_, INST, MTU, TX_LEN> = RT1062PhyTxToken {
                tx_pos: &mut self.tx_pos,
                enet_inst: &mut self.enet_inst,
                txdt: &mut self.txdt,
            };
            return Some(tok);
        } else {
            return None;
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

pub struct RT1062PhyRxToken<'a, const MTU: usize, const RX_LEN: usize> {
    rx_pos: &'a mut usize,
    rxdt: &'a mut RxDT<MTU, RX_LEN>,
}

impl<'a, const MTU: usize, const RX_LEN: usize> phy::RxToken for RT1062PhyRxToken<'a, MTU, RX_LEN> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let rxd = &mut self.rxdt.desc[*self.rx_pos];
        let result = f(&mut self.rxdt.bufs[*self.rx_pos]);
        atomic::fence(atomic::Ordering::SeqCst);
        rxd.flags |= 0x8000;
        if *self.rx_pos < (self.rxdt.desc.len() - 1) {
            *self.rx_pos += 1;
        } else {
            *self.rx_pos = 0;
        }
        result
    }
}

pub struct RT1062PhyTxToken<'a, const INST: u8, const MTU: usize, const TX_LEN: usize> {
    tx_pos: &'a mut usize,
    enet_inst: &'a mut enet::Instance<INST>,
    txdt: &'a mut TxDT<MTU, TX_LEN>,
}

impl<'a, const INST: u8, const MTU: usize, const TX_LEN: usize> phy::TxToken
    for RT1062PhyTxToken<'a, INST, MTU, TX_LEN>
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(&mut self.txdt.bufs[*self.tx_pos][..len]);
        let desc: &mut TxDescriptor = &mut self.txdt.desc[*self.tx_pos];
        desc.len = len as u16;
        desc.flags |= 0x8C00;
        atomic::fence(atomic::Ordering::SeqCst);
        ral::write_reg!(enet,self.enet_inst,TDAR,TDAR:1);

        if *self.tx_pos < (self.txdt.desc.len() - 1) {
            *self.tx_pos += 1;
        } else {
            *self.tx_pos = 0;
        }
        result
    }
}
