#![no_std]
#![no_main]

use bsp::board;
use smoltcp::iface::Config;
use smoltcp::iface::Interface;
use smoltcp::iface::SocketSet;
use smoltcp::socket::udp;
use smoltcp::socket::udp::PacketMetadata;
use smoltcp::time::Instant;
use smoltcp::wire::EthernetAddress;
use smoltcp::wire::IpAddress;
use smoltcp::wire::IpCidr;
use smoltcp::wire::IpEndpoint;
use smoltcp::wire::IpListenEndpoint;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::hal::timer::Blocking;

use bsp::hal;
use bsp::ral;
use ral::ccm;
use ral::ccm_analog;
use ral::enet;
use ral::iomuxc;
use ral::iomuxc_gpr;

use rt1062_eth_rs::RT1062Phy;

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

    let mut gpio2 = hal::gpio::Port::new(unsafe { ral::gpio::GPIO2::instance() });
    let pads = hal::iomuxc::into_pads(unsafe { ral::iomuxc::Instance::instance() });

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

    let mut txdt: rt1062_eth_rs::ring::TxDT<512, 12> = Default::default();
    let mut rxdt: rt1062_eth_rs::ring::RxDT<512, 12> = Default::default();

    rt1062_eth_rs::ring::print_dt(&mut delay, &txdt, &rxdt);

    delay.block_ms(10);

    let mut phy: RT1062Phy<1, 512, 12, 12> =
        RT1062Phy::new(unsafe { enet::ENET1::instance() }, &mut rxdt, &mut txdt);
        delay.block_ms(10);

    rt1062_eth_rs::ring::print_dt(&mut delay, phy.txdt, phy.rxdt);

    delay.block_ms(10);

    {
        phy.mdio_write(0, 0x18, 0x0280); // LED shows link status, active high
        phy.mdio_write(0, 0x17, 0x0081); // config for 50 MHz clock input

        let rcsr = phy.mdio_read(0, 0x17);
        let ledcr = phy.mdio_read(0, 0x18);
        let phycr = phy.mdio_read(0, 0x19);
        log::info!("RCSR:{rcsr}, LEDCR:{ledcr}, PHYCR:{phycr}");
    }

    let mut time: i64 = 0;

    let config = Config::new(EthernetAddress([0x03, 0x48, 0x46, 0x03, 0x96, 0x21]).into());
    let mut iface = Interface::new(config, &mut phy, Instant::from_millis(time));

    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(IpCidr::new(IpAddress::v4(192, 168, 69, 1), 24))
            .unwrap();
    });

    let mut client_socket = {
        static mut RX_HEADER: [PacketMetadata; 2] = [PacketMetadata::EMPTY, PacketMetadata::EMPTY];
        static mut TX_HEADER: [PacketMetadata; 2] = [PacketMetadata::EMPTY, PacketMetadata::EMPTY];
        static mut TCP_CLIENT_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_CLIENT_TX_DATA: [u8; 1024] = [0; 1024];

        let tcp_rx_buffer = udp::PacketBuffer::new(unsafe { &mut RX_HEADER[..] }, unsafe {
            &mut TCP_CLIENT_RX_DATA[..]
        });
        let tcp_tx_buffer = udp::PacketBuffer::new(unsafe { &mut TX_HEADER[..] }, unsafe {
            &mut TCP_CLIENT_TX_DATA[..]
        });

        udp::Socket::new(tcp_rx_buffer, tcp_tx_buffer)
    };

    let mut sockets: [_; 1] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets[..]);

    let result = client_socket.bind(IpListenEndpoint {
        addr: None,
        port: 80,
    });

    match result {
        Ok(_) => (),
        Err(_x) => {
            log::info!("Bind Error!");
        }
    }

    let client_handle = sockets.add(client_socket);

    let test: [u8; 3] = [0x69, 0x69, 0x69];

    loop {
        time += 10;
        delay.block_ms(10);
        let _x = iface.poll(Instant::from_millis(time), &mut phy, &mut sockets);

        if (time % 100) < 10 {
            let z: &mut udp::Socket = sockets.get_mut(client_handle);

            if !z.can_send() {
                //log::info!("can't send!");
            }
            if !z.is_open() {
                log::info!("closed");
            }

            let w = z.send_slice(
                &test,
                IpEndpoint {
                    addr: IpAddress::v4(192, 168, 69, 2),
                    port: 80,
                },
            );

            match w {
                Ok(_) => (),
                Err(_x) => {
                    //    log::info!("SendErr");
                }
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
