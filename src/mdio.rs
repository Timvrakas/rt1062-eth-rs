use teensy4_bsp as bsp;
use bsp::ral;
use ral::enet;

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