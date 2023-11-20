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
use bsp::ral::enet;

use imxrt_eth::{RT1062Device,ring::RxDT,ring::TxDT,teensy_eth};

#[bsp::rt::entry]
fn main() -> ! {
    static mut TXDT: TxDT<1514, 12> = TxDT::default();
    static mut RXDT: RxDT<1514, 12> = RxDT::default();

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
        mut gpio2,
        ..
    } = board::t41(instances);

    // Configures the GPT1 timer to run at GPT1_FREQUENCY. See the
    // constants below for more information.
    gpt1.disable();
    gpt1.set_divider(GPT1_DIVIDER);
    gpt1.set_clock_source(GPT1_CLOCK_SOURCE);

    // Convenience for blocking delays.
    let mut delay: Blocking<hal::gpt::Gpt<1>, GPT1_FREQUENCY> =
        Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);

    bsp::LoggingFrontend::default_log().register_usb(usb);
    delay.block_ms(2000);

    teensy_eth::teensy_setup_mac(&mut gpio2, &mut delay);

    imxrt_eth::ring::print_dt(&mut delay, &TXDT, &RXDT);

    let mut phy: RT1062Device<1, 1514, 12, 12> =
        RT1062Device::new(unsafe { enet::ENET1::instance() }, RXDT, TXDT);

    imxrt_eth::ring::print_dt(&mut delay, phy.txdt, phy.rxdt);

    teensy_eth::teensy_setup_phy(&mut phy);

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

        if (time % 1000) < 10 {
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
