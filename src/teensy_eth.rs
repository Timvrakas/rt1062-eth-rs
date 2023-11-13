use crate::bsp::hal;
use crate::bsp::ral;
use hal::iomuxc::PullKeeper;
use ral::iomuxc;

use ral::ccm;
use ral::ccm_analog;
use ral::iomuxc_gpr;
use hal::timer::Blocking;

use crate::RT1062Device;

/* This module contains the MAC and PHY initialization code that's specific to the Teensy 4.1, and it's DP83825  */

pub fn teensy_setup_mac(gpio2: &mut hal::gpio::Port<2>, delay:&mut Blocking<hal::gpt::Gpt<1>, 1000>){

    //Enable clock distro, both to the ethernet MAC and out the PHY pins
    let ccm1 = unsafe { ral::ccm::CCM::instance() };
    let ccm_analog1 = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };
    let mux_gpr1 = unsafe { ral::iomuxc_gpr::IOMUXC_GPR::instance() };
    let mux1 = unsafe { ral::iomuxc::IOMUXC::instance() };

    ral::modify_reg!(ccm,ccm1,CCGR1,CG5:3); // need to fix this for 1062

    // Configure the PLL for 50MHz
    ral::write_reg!(ccm_analog,ccm_analog1,PLL_ENET_CLR,BYPASS_CLK_SRC:3,ENET2_DIV_SELECT:3,DIV_SELECT:3,POWERDOWN:1);
    ral::write_reg!(ccm_analog,ccm_analog1,PLL_ENET_SET,ENET_25M_REF_EN:1,ENABLE:1,BYPASS:1,DIV_SELECT:1);

    // Start the PPL and wait for a lock
    while ral::read_reg!(ccm_analog, ccm_analog1, PLL_ENET_SET, LOCK) == 0 {}
    ral::write_reg!(ccm_analog,ccm_analog1,PLL_ENET_CLR,BYPASS:1);

    // send refclock to pinmux
    ral::modify_reg!(iomuxc_gpr,mux_gpr1,GPR1,ENET1_CLK_SEL:0,ENET_IPG_CLK_S_EN:0,ENET1_TX_CLK_DIR:1);

    let mut pads = hal::iomuxc::into_pads(unsafe { ral::iomuxc::Instance::instance() });

    const ENET_IO_PD: hal::iomuxc::Config = hal::iomuxc::Config::zero()
    .set_pull_keeper(Some(PullKeeper::Pulldown100k))
    .set_speed(hal::iomuxc::Speed::Max)
    .set_drive_strength(hal::iomuxc::DriveStrength::R0_5)
    .set_slew_rate(hal::iomuxc::SlewRate::Fast);

    const ENET_IO_PU: hal::iomuxc::Config = ENET_IO_PD
    .set_pull_keeper(Some(PullKeeper::Pullup22k));

    const XI_CONFIG: hal::iomuxc::Config = hal::iomuxc::Config::zero()
    .set_drive_strength(hal::iomuxc::DriveStrength::R0_6)
    .set_slew_rate(hal::iomuxc::SlewRate::Fast);
    
    hal::iomuxc::configure(&mut pads.gpio_b1.p04, ENET_IO_PD); //RXD0
    hal::iomuxc::alternate(&mut pads.gpio_b1.p04, 3);

    hal::iomuxc::configure(&mut pads.gpio_b1.p05, ENET_IO_PD); //RXD1
    hal::iomuxc::alternate(&mut pads.gpio_b1.p05, 3);

    hal::iomuxc::configure(&mut pads.gpio_b1.p06, ENET_IO_PD); //DV
    hal::iomuxc::alternate(&mut pads.gpio_b1.p06, 3);

    hal::iomuxc::configure(&mut pads.gpio_b1.p11, ENET_IO_PD); //RXER
    hal::iomuxc::alternate(&mut pads.gpio_b1.p11, 3);

    hal::iomuxc::configure(&mut pads.gpio_b1.p07, ENET_IO_PU); //TXD0
    hal::iomuxc::alternate(&mut pads.gpio_b1.p07, 3);

    hal::iomuxc::configure(&mut pads.gpio_b1.p08, ENET_IO_PU); //TXD1
    hal::iomuxc::alternate(&mut pads.gpio_b1.p08, 3);

    hal::iomuxc::configure(&mut pads.gpio_b1.p09, ENET_IO_PU); //TXEN
    hal::iomuxc::alternate(&mut pads.gpio_b1.p09, 3);

    hal::iomuxc::configure(&mut pads.gpio_b1.p10, XI_CONFIG); //XI (To Phy)
    hal::iomuxc::alternate(&mut pads.gpio_b1.p10, 6);
    hal::iomuxc::set_sion(&mut pads.gpio_b1.p10);

    hal::iomuxc::configure(&mut pads.gpio_b1.p15, ENET_IO_PU); //MDIO
    hal::iomuxc::alternate(&mut pads.gpio_b1.p15, 0);

    hal::iomuxc::configure(&mut pads.gpio_b1.p14, ENET_IO_PU); //MDC
    hal::iomuxc::alternate(&mut pads.gpio_b1.p14, 0);

    //this needs iomuxc support
    ral::write_reg!(iomuxc, mux1, ENET_MDIO_SELECT_INPUT, DAISY:2);
    ral::write_reg!(iomuxc, mux1, ENET0_RXDATA_SELECT_INPUT,DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET1_RXDATA_SELECT_INPUT,DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_RXEN_SELECT_INPUT,DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_RXERR_SELECT_INPUT,DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_IPG_CLK_RMII_SELECT_INPUT, DAISY:1);

    let phy_shdn = gpio2.output(pads.gpio_b0.p15);
    let phy_rst = gpio2.output(pads.gpio_b0.p14);

    phy_shdn.clear();
    phy_rst.clear();
    delay.block_ms(50);
    phy_shdn.set();
    delay.block_ms(50);
    phy_rst.set();
}

pub fn teensy_setup_phy(phy: &mut RT1062Device<1, 1514, 12, 12>){
    phy.mdio_write(0, 0x18, 0x0280); // LED shows link status, active high
    phy.mdio_write(0, 0x17, 0x0081); // config for 50 MHz clock input

    let rcsr = phy.mdio_read(0, 0x17);
    let ledcr = phy.mdio_read(0, 0x18);
    let phycr = phy.mdio_read(0, 0x19);
    log::info!("RCSR:{rcsr}, LEDCR:{ledcr}, PHYCR:{phycr}");
}