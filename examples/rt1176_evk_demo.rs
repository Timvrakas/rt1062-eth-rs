#![no_std]
#![no_main]

use board::logging::Poller;
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

    setup_mac(&mut delay, &mut poller);

    imxrt_eth::ring::print_dt(&mut delay, &TXDT, &RXDT);
    poller.poll();

    let mut phy: RT1062Device<0, 1536, 12, 12> =
        RT1062Device::new(unsafe { enet::ENET::instance() }, RXDT, TXDT);

    imxrt_eth::ring::print_dt(&mut delay, phy.txdt, phy.rxdt);
    poller.poll();

    setup_phy(&mut phy, &mut delay, &mut poller);
    poller.poll();

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

pub fn ai_read_vddsoc2(addr: u32) -> u32{
    let anadig_misc = unsafe {ral::anadig_misc::ANADIG_MISC::instance()};
    let pre_toggle_done = ral::read_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AITOGGLE_DONE_1G);
    ral::write_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AIRWB_1G:1,VDDSOC2PLL_AIADDR_1G:addr);
    let x = ral::read_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AITOGGLE_1G);
    ral::modify_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AITOGGLE_1G:!x); //toggle to write
    loop{
        if ral::read_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AITOGGLE_DONE_1G) != pre_toggle_done {
            break;
        }
    }
    return ral::read_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_RDATA_1G);
}

pub fn ai_write_vddsoc2(addr: u32, data: u32){
    let anadig_misc = unsafe {ral::anadig_misc::ANADIG_MISC::instance()};
    let pre_toggle_done = ral::read_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AITOGGLE_DONE_1G);
    ral::write_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AIRWB_1G:0,VDDSOC2PLL_AIADDR_1G:addr); 
    ral::write_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_WDATA_1G,data);
    let x = ral::read_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AITOGGLE_1G);
    ral::modify_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AITOGGLE_1G:!x); //toggle to write
    loop{
        if ral::read_reg!(ral::anadig_misc,anadig_misc,VDDSOC2PLL_AI_CTRL_1G,VDDSOC2PLL_AITOGGLE_DONE_1G) != pre_toggle_done {
            break;
        }
    }
}

pub fn dump_ai_regs(){
    let ctrl0_read = ai_read_vddsoc2(0);
    let ctrl1_read = ai_read_vddsoc2(16);
    let ctrl2_read = ai_read_vddsoc2(32);
    let ctrl3_read = ai_read_vddsoc2(48);
    log::info!("CTRL0:{:#08x}, CTRL1:{:#08x}, CTRL2:{:#08x}, CTRL3:{:#08x}",ctrl0_read,ctrl1_read,ctrl2_read,ctrl3_read);
    let anadig_pll = unsafe {ral::anadig_pll::ANADIG_PLL::instance()};
    log::info!("SYS_PLL1_CTRL:{:#08x}",ral::read_reg!(ral::anadig_pll,anadig_pll,SYS_PLL1_CTRL));
}

pub fn start_pll1(delay:&mut Blocking<hal::gpt::Gpt<1>, GPT1_FREQUENCY>, poller: &mut Poller){
    let anadig_misc = unsafe {ral::anadig_misc::ANADIG_MISC::instance()};
    let anadig_pmu = unsafe {ral::anadig_pmu::ANADIG_PMU::instance()};
    let anadig_pll = unsafe {ral::anadig_pll::ANADIG_PLL::instance()};

    //ENABLE LDO
    ral::write_reg!(ral::anadig_misc,anadig_misc,VDDSOC_AI_CTRL,VDDSOC_AIRWB:0,VDDSOC_AI_ADDR:0x0); //Setting CTRL0
    ral::write_reg!(ral::anadig_misc,anadig_misc,VDDSOC_AI_WDATA,0x01|0x04|0x100); //CTRL0 Values
    let x = ral::read_reg!(ral::anadig_pmu,anadig_pmu,PMU_LDO_PLL,LDO_PLL_AI_TOGGLE);
    ral::modify_reg!(ral::anadig_pmu,anadig_pmu,PMU_LDO_PLL,LDO_PLL_AI_TOGGLE:!x); //toggle to write
    delay.block_us(100);
    ral::modify_reg!(ral::anadig_pmu,anadig_pmu,PMU_REF_CTRL,EN_PLL_VOL_REF_BUFFER:1); //enable voltage ref buffer

    log::info!("Initial Config");
    dump_ai_regs();

    ai_write_vddsoc2(0, 0x01_0000); // clear register, enable bypass
    ral::modify_reg!(ral::anadig_pll,anadig_pll,SYS_PLL1_CTRL,ENABLE_CLK:1); //enable pll output

    let denominator: u32 = 268435455;
    let _div: u32         = 41;
    let numerator: u32   = 178956970;
    poller.poll();

    ai_write_vddsoc2(48, denominator); //CTRL3
    ai_write_vddsoc2(32, numerator); //CTRL2
    ai_write_vddsoc2(0, 0x01_0029);

    log::info!("Middle Config");
    dump_ai_regs();

    poller.poll();
    delay.block_ms(1);

    ai_write_vddsoc2(0, 0x41_0029); // PLL_REG_EN
    delay.block_us(100);
    ai_write_vddsoc2(0, 0x41_4029); // POWERUP

    ai_write_vddsoc2(0, 0x41_6029); // HOLD_RING_OFF
    delay.block_us(225);
    ai_write_vddsoc2(0, 0x41_4029); // HOLD_RING_OFF


    loop{ // Wait for Stable
        if ral::read_reg!(ral::anadig_pll,anadig_pll,SYS_PLL1_CTRL,SYS_PLL1_STABLE) == 1 {
            break;
        }
    }

    //ai_write_vddsoc2(4, 0x8000); // ENABLE
    ai_write_vddsoc2(0, 0x41_C029);

    ral::modify_reg!(ral::anadig_pll,anadig_pll,SYS_PLL1_CTRL,SYS_PLL1_GATE:0,SYS_PLL1_DIV2:1,SYS_PLL1_DIV5:1); //disable gate, enable div2

    ai_write_vddsoc2(0, 0x40_C029); // disable bypass

    log::info!("Final Config");
    dump_ai_regs();

}

pub fn setup_mac(delay:&mut Blocking<hal::gpt::Gpt<1>, GPT1_FREQUENCY>, poller: &mut Poller){

    //Enable clock distro, both to the ethernet MAC and out the PHY pins
    let mut ccm1 = unsafe { ral::ccm::CCM::instance() };
    let mux_gpr1 = unsafe { ral::iomuxc_gpr::IOMUXC_GPR::instance() };
    let mux1 = unsafe { ral::iomuxc::IOMUXC::instance() };

    start_pll1(delay,poller);

    poller.poll();
    delay.block_ms(1000);
    poller.poll();
    delay.block_ms(1000);

    use hal::ccm::output_source::{clko1};
    use board::clock_out::{CLKO1_SELECTIONS};

    clko1::enable(&mut ccm1, true);
    clko1::set_divider(&mut ccm1,10);
    clko1::set_selection(&mut ccm1, CLKO1_SELECTIONS[7]);

    delay.block_ms(1000);

    board::imxrt11xx::clock_tree::configure_clock_on(51, &mut ccm1);

    ral::modify_reg!(ral::ccm,ccm1,LPCG112_DIRECT,ON:1);
    ral::modify_reg!(ral::ccm,ccm1,LPCG113_DIRECT,ON:1);
    ral::modify_reg!(ral::ccm,ccm1,LPCG114_DIRECT,ON:1);
    ral::modify_reg!(ral::ccm,ccm1,LPCG49_DIRECT,ON:1);
    ral::modify_reg!(ral::ccm,ccm1,LPCG50_DIRECT,ON:1);

    //WE HAVE THE CLOCKS! (we do not) (it's so over) (we're not back at all)

    // send refclock to pinmux
    ral::modify_reg!(iomuxc_gpr,mux_gpr1,GPR4,ENET_REF_CLK_DIR:1);

    // Handle ERR050396. Depending on the runtime config, we might place buffers into TCM.
    ral::modify_reg!(ral::iomuxc_gpr, mux_gpr1, GPR28, CACHE_ENET1G: 0, CACHE_ENET: 0);

    // confiure pin muxes

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_02,MUX_MODE:1,SION:0); //TXD0
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_02,SRE:0,DSE:1);
    ral::write_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_DISP_B2_02, SRE: SRE_0_SLOW_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER, PUE: PUE_0_PULL_DISABLE__HIGHZ, ODE: ODE_0_DISABLED);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_03,MUX_MODE:1,SION:0); //TXD1
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_03,SRE:0,DSE:1);
    ral::write_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_DISP_B2_03, SRE: SRE_0_SLOW_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER, PUE: PUE_0_PULL_DISABLE__HIGHZ, ODE: ODE_0_DISABLED);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_04,MUX_MODE:1,SION:0); //TXEN
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_04,SRE:0,DSE:1);
    ral::write_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_DISP_B2_04, SRE: SRE_0_SLOW_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER, PUE: PUE_0_PULL_DISABLE__HIGHZ, ODE: ODE_0_DISABLED);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_05,MUX_MODE:2,SION:1); //REF_CLK (50MHz out)
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_05,SRE:1,DSE:1);
    ral::write_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_DISP_B2_05, SRE: SRE_1_FAST_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER, PUE: PUE_0_PULL_DISABLE__HIGHZ, ODE: ODE_0_DISABLED);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_06,MUX_MODE:1,SION:1); //RXD0
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_06,SRE:0,DSE:1);
    ral::write_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_DISP_B2_06, SRE: SRE_0_SLOW_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER, PUE: PUE_1_PULL_ENABLE, PUS: PUS_0_WEAK_PULL_DOWN, ODE: ODE_0_DISABLED);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_07,MUX_MODE:1,SION:1); //RXD1
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_07,SRE:0,DSE:1);
    ral::write_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_DISP_B2_07, SRE: SRE_0_SLOW_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER, PUE: PUE_1_PULL_ENABLE, PUS: PUS_0_WEAK_PULL_DOWN, ODE: ODE_0_DISABLED);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_08,MUX_MODE:1,SION:0); //RX_DV
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_08,SRE:0,DSE:1);
    ral::write_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_DISP_B2_08, SRE: SRE_0_SLOW_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER, PUE: PUE_1_PULL_ENABLE, PUS: PUS_0_WEAK_PULL_DOWN, ODE: ODE_0_DISABLED);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_DISP_B2_09,MUX_MODE:1,SION:0); //RX_ER
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_DISP_B2_09,SRE:0,DSE:1);
    ral::write_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_DISP_B2_09, SRE: SRE_0_SLOW_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER, PUE: PUE_1_PULL_ENABLE, PUS: PUS_0_WEAK_PULL_DOWN, ODE: ODE_0_DISABLED);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_AD_32,MUX_MODE:3,SION:0); //MDC (1.5k PU on evk)
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_AD_32,SRE:0,DSE:1);

    ral::write_reg!(ral::iomuxc,mux1,SW_MUX_CTL_PAD_GPIO_AD_33,MUX_MODE:3,SION:0); //MDIO (1.5k PU on evk)
    // ral::write_reg!(ral::iomuxc,mux1,SW_PAD_CTL_PAD_GPIO_AD_33,SRE:0,DSE:1);

    let mut pads = hal::iomuxc::into_pads(
        unsafe { ral::iomuxc::Instance::instance() },
        unsafe { ral::iomuxc_lpsr::Instance::instance()});

    //
    // Input daisies
    //
    ral::write_reg!(ral::iomuxc, mux1, ENET_MAC0_MDIO_SELECT_INPUT, DAISY: SELECT_GPIO_AD_33_ALT3);
    ral::write_reg!(ral::iomuxc, mux1, ENET_IPG_CLK_RMII_SELECT_INPUT, DAISY: SELECT_GPIO_DISP_B2_05_ALT2);
    ral::write_reg!(ral::iomuxc, mux1, ENET_MAC0_RXDATA_SELECT_INPUT_0, DAISY: SELECT_GPIO_DISP_B2_06_ALT1);
    ral::write_reg!(ral::iomuxc, mux1, ENET_MAC0_RXDATA_SELECT_INPUT_1, DAISY: SELECT_GPIO_DISP_B2_07_ALT1);
    ral::write_reg!(ral::iomuxc, mux1, ENET_MAC0_RXEN_SELECT_INPUT, DAISY: SELECT_GPIO_DISP_B2_08_ALT1);
    ral::write_reg!(ral::iomuxc, mux1, ENET_MAC0_RXERR_SELECT_INPUT, DAISY: SELECT_GPIO_DISP_B2_09_ALT1);


    let gpio6 = unsafe { ral::gpio::GPIO6::instance() };
    let mut gpio6 = hal::gpio::Port::new(gpio6);

    let gpio9 = unsafe { ral::gpio::GPIO9::instance() };
    let mut gpio9 = hal::gpio::Port::new(gpio9);

    let phy_rst = gpio6.output(pads.gpio_lpsr.p12);
    let phy_int = gpio9.output(pads.gpio_ad.p12);

    phy_rst.clear();
    phy_int.clear();
    delay.block_ms(500);
    phy_int.set();
    delay.block_ms(500);
    phy_rst.set();
    delay.block_ms(500);

    ral::modify_reg!(ral::iomuxc, mux1, SW_PAD_CTL_PAD_GPIO_AD_14, SRE: SRE_1_FAST_SLEW_RATE, DSE: DSE_1_HIGH_DRIVER);
    hal::iomuxc::alternate(&mut pads.gpio_ad.p14, 9);

}

pub fn setup_phy(phy: &mut RT1062Device<0, 1536, 12, 12>, delay: &mut Blocking<hal::gpt::Gpt<1>, GPT1_FREQUENCY>, poller: &mut Poller){
    loop{
        let ctrl1 = phy.mdio_read(2, 0x1E);
        let ctrl2 = phy.mdio_read(2, 0x1F);
        let bc = phy.mdio_read(2, 0x00);
        let id = phy.mdio_read(2, 0x02);
        let an = phy.mdio_read(2, 0x04);
        log::info!("CTRL1:{:#04x}, CTRL2:{:#04x}, ID:{:#04x}, BC:{:#04x}, AN:{:#04x}",ctrl1,ctrl2,id,bc,an);
        if id == 0x22 {
            break;
        }
    }
    phy.mdio_write(2, 0x00, 0x8000); //RESET
    
    let ctrl2 = phy.mdio_read(2, 0x1F);
    phy.mdio_write(2, 0x1F, ctrl2 | 0x0080 ); //50Mhz RMII REFCLK
    let ctrl2 = phy.mdio_read(2, 0x1F);

    let an = phy.mdio_read(2, 0x04);
    phy.mdio_write(2, 0x04, an | 0x0180 ); //100Meg AN
    let an = phy.mdio_read(2, 0x04);

    phy.mdio_write(2, 0x0, 0x3300 ); //100Meg
    let bc = phy.mdio_read(2, 0x00);

    log::info!("Set CTRL2:{:#04x}, BC:{:#04x}, AN:{:#04x}",ctrl2,bc,an);

    for _ in 0..10{
        delay.block_ms(500);
        let ctrl1 = phy.mdio_read(2, 0x1E);
        log::info!("CTRL1:{:#04x}",ctrl1);
        poller.poll();
    }
}