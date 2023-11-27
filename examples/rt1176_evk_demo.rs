#![no_std]
#![no_main]

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

use imxrt_hal as hal;
use imxrt_ral as ral;
use imxrt_rt as rt;

use ral::enet;
use ral::iomuxc_gpr;
use ral::iomuxc;
use hal::timer::Blocking;

use board::GPT1_FREQUENCY;

use imxrt_eth::{RT1062Device,ring::RxDT,ring::TxDT};

#[rt::entry]
fn main() -> ! {
    static mut TXDT: TxDT<1536, 12> = TxDT::default();
    static mut RXDT: RxDT<1536, 12> = RxDT::default();

    let (board::Common { gpt1, mut dma, .. }, board::Specifics { console, .. }) = board::new();

    // Convenience for blocking delays.
    let mut delay: Blocking<hal::gpt::Gpt<1>, GPT1_FREQUENCY> =
        Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);


    let dma_a = dma[board::BOARD_DMA_A_INDEX].take().unwrap();
    let mut poller = board::logging::lpuart(board::logging::Frontend::Log, console, dma_a);

    delay.block_ms(2000);
    log::info!("Here's a log!");
    poller.poll();

    setup_mac(&mut delay);

    imxrt_eth::ring::print_dt(&mut delay, &TXDT, &RXDT);
    poller.poll();

    let mut phy: RT1062Device<0, 1536, 12, 12> =
        RT1062Device::new(unsafe { enet::ENET::instance() }, RXDT, TXDT);

    imxrt_eth::ring::print_dt(&mut delay, phy.txdt, phy.rxdt);
    poller.poll();

    for _ in 1..=100 {
        setup_phy(&mut phy);
        poller.poll();
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

    poller.poll();

    let client_handle = sockets.add(client_socket);

    let test: [u8; 3] = [0x69, 0x69, 0x69];

    loop {
        poller.poll();
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

pub fn setup_mac(delay:&mut Blocking<hal::gpt::Gpt<1>, GPT1_FREQUENCY>){

    //Enable clock distro, both to the ethernet MAC and out the PHY pins
    let mut ccm1 = unsafe { ral::ccm::CCM::instance() };
    let mux_gpr1 = unsafe { ral::iomuxc_gpr::IOMUXC_GPR::instance() };
    let mux1 = unsafe { ral::iomuxc::IOMUXC::instance() };

    board::imxrt11xx::clock_tree::configure_enet1(&mut ccm1);

    ral::modify_reg!(ral::ccm,ccm1,LPCG112_DIRECT,ON:1); //do we need this, or does it happen automatically

    // send refclock to pinmux
    ral::modify_reg!(iomuxc_gpr,mux_gpr1,GPR4,ENET_REF_CLK_DIR:1);

    // confiure pin muxes

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_02,MUX_MODE:1,SION:0); //TXD0
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_02,SRE:0,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_03,MUX_MODE:1,SION:0); //TXD1
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_03,SRE:0,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_04,MUX_MODE:1,SION:0); //TXEN
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_04,SRE:0,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_05,MUX_MODE:2,SION:1); //REF_CLK (50MHz out)
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_05,SRE:1,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_06,MUX_MODE:1,SION:0); //RXD0
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_06,SRE:0,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_07,MUX_MODE:1,SION:0); //RXD1
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_07,SRE:0,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_08,MUX_MODE:1,SION:0); //RX_DV
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_08,SRE:0,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_AD_32,MUX_MODE:3,SION:0); //MDC (1.5k PU on evk)
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_AD_32,SRE:0,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_AD_33,MUX_MODE:3,SION:0); //MDIO (1.5k PU on evk)
    ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_AD_33,SRE:0,DSE:1);
    
    let pads = hal::iomuxc::into_pads(
        unsafe { ral::iomuxc::Instance::instance() },
        unsafe { ral::iomuxc_lpsr::Instance::instance()});

    // const ENET_IO_PD: hal::iomuxc::Config = hal::iomuxc::Config::zero()
    // .set_pull_keeper(Some(PullKeeper::Pulldown100k))
    // .set_speed(hal::iomuxc::Speed::Max)
    // .set_drive_strength(hal::iomuxc::DriveStrength::R0_5)
    // .set_slew_rate(hal::iomuxc::SlewRate::Fast);

    // const ENET_IO_PU: hal::iomuxc::Config = ENET_IO_PD
    // .set_pull_keeper(Some(PullKeeper::Pullup22k));

    // const XI_CONFIG: hal::iomuxc::Config = hal::iomuxc::Config::zero()
    // .set_drive_strength(hal::iomuxc::DriveStrength::R0)
    // .set_slew_rate(hal::iomuxc::SlewRate::Fast);
    
    // hal::iomuxc::configure(&mut pads.gpio_disp_b2.p06, ENET_IO_PD); //RXD0
    // hal::iomuxc::alternate(&mut pads.gpio_disp_b2.p06, 1);

    // hal::iomuxc::configure(&mut pads.gpio_disp_b2.p07, ENET_IO_PD); //RXD1
    // hal::iomuxc::alternate(&mut pads.gpio_disp_b2.p07, 1);

    // hal::iomuxc::configure(&mut pads.gpio_disp_b2.p08, ENET_IO_PD); //DV
    // hal::iomuxc::alternate(&mut pads.gpio_disp_b2.p08, 1);

    // hal::iomuxc::configure(&mut pads.gpio_disp_b2.p09, ENET_IO_PD); //RXER
    // hal::iomuxc::alternate(&mut pads.gpio_disp_b2.p09, 1);

    // hal::iomuxc::configure(&mut pads.gpio_disp_b2.p02, ENET_IO_PU); //TXD0
    // hal::iomuxc::alternate(&mut pads.gpio_disp_b2.p02, 1);

    // hal::iomuxc::configure(&mut pads.gpio_disp_b2.p03, ENET_IO_PU); //TXD1
    // hal::iomuxc::alternate(&mut pads.gpio_disp_b2.p03, 1);

    // hal::iomuxc::configure(&mut pads.gpio_disp_b2.p04, ENET_IO_PU); //TXEN
    // hal::iomuxc::alternate(&mut pads.gpio_disp_b2.p04, 1);

    // hal::iomuxc::configure(&mut pads.gpio_disp_b2.p05, XI_CONFIG); //XI (To Phy)
    // hal::iomuxc::alternate(&mut pads.gpio_disp_b2.p05, 2); // "ENET_REF_CLK"
    // hal::iomuxc::set_sion(&mut pads.gpio_disp_b2.p05);

    // hal::iomuxc::configure(&mut pads.gpio_ad.p33, ENET_IO_PU); //MDIO
    // hal::iomuxc::alternate(&mut pads.gpio_ad.p33, 0);

    // hal::iomuxc::configure(&mut pads.gpio_ad.p32, ENET_IO_PU); //MDC
    // hal::iomuxc::alternate(&mut pads.gpio_ad.p32, 0);

    //this needs iomuxc support
    ral::write_reg!(iomuxc, mux1, ENET_IPG_CLK_RMII_SELECT_INPUT, DAISY:1); //what is this one for?
    ral::write_reg!(iomuxc, mux1, ENET_MAC0_MDIO_SELECT_INPUT, DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_MAC0_RXDATA_SELECT_INPUT_0, DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_MAC0_RXDATA_SELECT_INPUT_1, DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_MAC0_RXEN_SELECT_INPUT, DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_MAC0_RXERR_SELECT_INPUT, DAISY:1);
    ral::write_reg!(iomuxc, mux1, ENET_MAC0_TXCLK_SELECT_INPUT, DAISY:1); //what is this one for?

    let gpio6 = unsafe { ral::gpio::GPIO6::instance() };
    let mut gpio6 = hal::gpio::Port::new(gpio6);

    let phy_rst = gpio6.output(pads.gpio_lpsr.p12);
    phy_rst.clear();
    delay.block_ms(50);
    phy_rst.set();

}

pub fn setup_phy(phy: &mut RT1062Device<0, 1536, 12, 12>){
    let ctrl1 = phy.mdio_read(2, 0x1E);
    let ctrl2 = phy.mdio_read(2, 0x1F);
    let id = phy.mdio_read(2, 0x02);
    log::info!("CTRL1:{ctrl1}, CTRL2:{ctrl2}, id:{id}");
    //phy.mdio_write(0, 0x1F, );
}