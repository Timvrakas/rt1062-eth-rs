MDC     K16     GPIO_AD_32
MDIO    H17     GPIO_AD_33
INT     P17     GPIO_AD_12
TXD0    E9      GPIO_DISP_B2_02
TXD1    D7      GPIO_DISP_B2_03
TXEN    C7      GPIO_DISP_B2_04
REF_CLK C9      GPIO_DISP_B2_05 (50Mhz, from MAC)
RXD0    C6      GPIO_DISP_B2_06
RXD1    D6      GPIO_DISP_B2_07
CRS_DV  B5      GPIO_DISP_B2_08
RX_ER   D8      GPIO_DISP_B2_09
RST     U5      GPIO_LPSR_12

Register 1Fh, Bit [7] is set to 1 to select 50 MHz clock mode.

clock_root_config_t rootCfg = {.mux = 4, .div = 10}; /* Generate 50M root clock. */
CLOCK_SetRootClock(kCLOCK_Root_Enet1, &rootCfg);
SYS_PLL1_DIV2