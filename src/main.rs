#![no_std]
#![no_main]

use bsp::board;
use smoltcp::phy::Device;
// use smoltcp::phy::RxToken;
use smoltcp::phy::TxToken;
use smoltcp::time::Instant;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::hal::timer::Blocking;

use bsp::ral;
use ral::ccm;
use ral::ccm_analog;
use ral::enet;
use ral::iomuxc;
use ral::iomuxc_gpr;

mod mdio;
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

    //Enable clock distro, both to the ethernet MAC and out the PHY pi
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

    let enet1 = unsafe { enet::ENET1::instance() };

    // //need to enable pins
    // //why no PHY link?

    let gpio7 = unsafe { ral::gpio::GPIO7::instance() };

    // let powerdown_pin = gpio7.output(iomuxc.gpio_b0.p15);
    // let reset_pin = gpio7.output(iomuxc.gpio_b0.p14);
    ral::modify_reg!(ral::gpio, gpio7, GDIR, |x| x | (1 << 14) | (1 << 15));
    ral::write_reg!(ral::gpio, gpio7, DR_CLEAR, (1 << 14) | (1 << 15));
    delay.block_ms(50);
    ral::write_reg!(ral::gpio, gpio7, DR_SET, 1 << 15);
    delay.block_ms(50);
    ral::write_reg!(ral::gpio, gpio7, DR_SET, 1 << 14);
    delay.block_ms(50);

    ral::write_reg!(enet,enet1,MSCR,MII_SPEED:9);

    bsp::LoggingFrontend::default_log().register_usb(usb);

    delay.block_ms(2000);

    mdio::mdio_write(&enet1, 0, 0x18, 0x0280); // LED shows link status, active high
    mdio::mdio_write(&enet1, 0, 0x17, 0x0081); // config for 50 MHz clock input

    let rcsr = mdio::mdio_read(&enet1, 0, 0x17);
    let ledcr = mdio::mdio_read(&enet1, 0, 0x18);
    let phycr = mdio::mdio_read(&enet1, 0, 0x19);
    log::info!("RCSR:{rcsr}, LEDCR:{ledcr}, PHYCR:{phycr}");

    ral::write_reg!(enet, enet1, EIMR, 0x0); //interupt mask: all off
    ral::modify_reg!(enet,enet1,RCR,RMII_MODE:1,MII_MODE:1,LOOP:0,PROM:1,CRCFWD:1,DRT:0,MAX_FL:1522,NLC:1,PADEN:1); //rmii, no loopback, no MAC filter
    ral::modify_reg!(enet,enet1,ECR,ETHEREN:1, DBSWP:1); //enable, swap bits
    ral::modify_reg!(enet,enet1,TCR,FDEN:1); //enable full-duplex
    ral::modify_reg!(enet,enet1,TFWR,STRFWD:1); // store and fwd
    ral::write_reg!(enet,enet1,RDAR,RDAR:1);

    let mut phy = iface::RT1062Phy::new();

    let mut time: i64 = 0;
    loop {
        time += 10;
        delay.block_ms(100);

        log::info!("=== create first token ===");
        let token_1 = phy.transmit(Instant::from_millis(time));
        log::info!("=== create second token ===");
        let token_2 = phy.transmit(Instant::from_millis(time));

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

        log::info!("=== consume second token ===");
        match token_2 {
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
