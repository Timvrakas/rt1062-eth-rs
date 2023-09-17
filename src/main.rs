#![no_std]
#![no_main]

use bsp::board;
use iface::RT1062Phy;
use smoltcp::phy::Device;
// use smoltcp::phy::RxToken;
use smoltcp::phy::TxToken;
use smoltcp::time::Instant;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::hal::timer::Blocking;

use bsp::ral;
use bsp::hal;
use ral::ccm;
use ral::ccm_analog;
use ral::iomuxc;
use ral::iomuxc_gpr;

mod iface;

#[bsp::rt::entry]
fn main() -> ! {
    // These are peripheral instances. Let the board configure these for us.
    // This function can only be called once!
    let instances = board::instances();

    // Driver resources that are configured by the board. For more information,
    // see the `board` documentation.
    let board::Resources {
        // This is a hardware timer. We'll use it for blocking delays.
        mut gpt1,
        // These are low-level USB resources. We'll pass these to a function
        // that sets up USB logging.
        usb,
        ..
    } = board::t41(instances);

    // Configures the GPT1 timer to run at GPT1_FREQUENCY. See the
    // constants below for more information.
    gpt1.disable();
    gpt1.set_divider(GPT1_DIVIDER);
    gpt1.set_clock_source(GPT1_CLOCK_SOURCE);

    // Convenience for blocking delays.
    let mut delay: Blocking<bsp::hal::gpt::Gpt<1>, 1000> =
        Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

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

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_04, PUS:0, PUE:1, PKE:1, SPEED:3, DSE:5, SRE:1); //RXD0
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_04, MUX_MODE:3, SION:0); // RXD0

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_06, PUS:0, PUE:1, PKE:1, SPEED:3, DSE:5, SRE:1); //DV
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_06, MUX_MODE:3, SION:0); // DV

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_05, PUS:2, PUE:1, PKE:1, SPEED:3, DSE:5, SRE:1); //RXD1
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_05, MUX_MODE:3, SION:0); // RXD1

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_11, PUS:0, PUE:1, PKE:1, SPEED:3, DSE:5, SRE:1); //RXER
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_11, MUX_MODE:3, SION:0); // RXER

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_07, PUS:2, PUE:1, PKE:1, SPEED:3, DSE:5, SRE:1); //TXD0
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_07, MUX_MODE:3, SION:0); // TXD0

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_08, PUS:2, PUE:1, PKE:1, SPEED:3, DSE:5, SRE:1); //TXD1
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_08, MUX_MODE:3, SION:0); // TXD0

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_09, PUS:2, PUE:1, PKE:1, SPEED:3, DSE:5, SRE:1); //TXEN
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_09, MUX_MODE:3, SION:0); // TXEN

    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B0_14, MUX_MODE:5); // RESET
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B0_15, MUX_MODE:5); // POWERDOWN/INT

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_10, DSE:6, SRE:1); //XI
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_10, MUX_MODE:6, SION:1); // XI

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_15, PUS:2, PUE:1, PKE:1, DSE:5, SRE:1); //MDIO
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_15, MUX_MODE:0); // MDIO

    ral::write_reg!(iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_B1_14, PUS:2, PUE:1, PKE:1, SPEED:3, DSE:5, SRE:1); //MDC
    ral::write_reg!(iomuxc, mux1, SW_MUX_CTL_PAD_GPIO_B1_14, MUX_MODE:0); // MDC

    ral::write_reg!(iomuxc, mux1, ENET_MDIO_SELECT_INPUT, DAISY:2);
    ral::write_reg!(iomuxc, mux1, ENET0_RXDATA_SELECT_INPUT,DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET1_RXDATA_SELECT_INPUT,DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_RXEN_SELECT_INPUT,DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_RXERR_SELECT_INPUT,DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_IPG_CLK_RMII_SELECT_INPUT, DAISY:1);


    let mut gpio2 = hal::gpio::Port::new(unsafe{ral::gpio::GPIO2::instance()});
    let pads = hal::iomuxc::into_pads(unsafe {ral::iomuxc::Instance::instance()});

    let phy_shdn = gpio2.output(pads.gpio_b0.p15);
    let phy_rst = gpio2.output(pads.gpio_b0.p14);

    phy_shdn.clear();
    phy_rst.clear();
    delay.block_ms(50);
    phy_shdn.set();
    delay.block_ms(50);
    phy_rst.set();
    
    bsp::LoggingFrontend::default_log().register_usb(usb);

    delay.block_ms(2000);

    let mut phy = iface::RT1062Phy::new();

    let mut time: i64 = 0;
    let mut count: i32 = 0;
    loop {
        //log::info!("start loop");
        time += 10;
        log::info!("A");
        for i in 1..10 {
            log::info!("delay {}", i);
            delay.block_ms(10);
        }
        log::info!("B");
        test(&mut phy, time);
        log::info!("send count: {}", count);
        count += 1;
    }
}


fn test(phy: &mut RT1062Phy, time: i64) {
    log::info!("=== create first token ===");
    let token_1 = phy.transmit(Instant::from_millis(time));

    log::info!("=== consume first token ===");
    match token_1 {
        None => (),
        Some(tx) => {
            tx.consume(40, |buf: &mut [u8]| {
                let str = "this-is-a-test-of-ethernet";
                buf[0..6].copy_from_slice(&[0xd8, 0xec, 0x5e, 0x2b, 0x45, 0x07]); //dest mac
                buf[6..12].copy_from_slice(&[0x03, 0x48, 0x46, 0x03, 0x96, 0x21]); //dest mac
                buf[12..14].copy_from_slice(&[0x00, 0x00]); //EtherType
                buf[14..40].copy_from_slice(str.as_bytes()); //Payload
            });
        }
    }

}

// We're responsible for configuring our timers.
// This example uses PERCLK_CLK as the GPT1 clock source,
// and it configures a 1 KHz GPT1 frequency by computing a
// GPT1 divider.
use bsp::hal::gpt::ClockSource;

/// The intended GPT1 frequency (Hz).
const GPT1_FREQUENCY: u32 = 1_000;
/// Given this clock source...
const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
/// ... the root clock is PERCLK_CLK. To configure a GPT1 frequency,
/// we need a divider of...
const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;
