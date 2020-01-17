unit gd32vf103xx;
interface
{$PACKRECORDS 2}
{$GOTO ON}
{$MODESWITCH ADVANCEDRECORDS}

//Interrupt Number Definition
type
  TIRQn_Enum = (
    CLIC_RESERVED_IRQn  = 0,     // RISC-V reserved
    CLIC_SFT_IRQn       = 3,     // Software interrupt
    CLIC_TMR_IRQn       = 7,     // CPU Timer interrupt
    CLIC_BWEI_IRQn      = 17,    // Bus Error interrupt
    CLIC_PMOVI_IRQn     = 18,    // Performance Monitor

    // interruput numbers
    WWDGT_IRQn          = 19,    // window watchDog timer interrupt 
    LVD_IRQn            = 20,    // LVD through EXTI line detect interrupt 
    TAMPER_IRQn         = 21,    // tamper through EXTI line detect 
    RTC_IRQn            = 22,    // RTC alarm interrupt 
    FMC_IRQn            = 23,    // FMC interrupt 
    RCU_CTC_IRQn        = 24,    // RCU and CTC interrupt 
    EXTI0_IRQn          = 25,    // EXTI line 0 interrupts 
    EXTI1_IRQn          = 26,    // EXTI line 1 interrupts 
    EXTI2_IRQn          = 27,    // EXTI line 2 interrupts 
    EXTI3_IRQn          = 28,    // EXTI line 3 interrupts 
    EXTI4_IRQn          = 29,    // EXTI line 4 interrupts 
    DMA0_Channel0_IRQn  = 30,    // DMA0 channel0 interrupt 
    DMA0_Channel1_IRQn  = 31,    // DMA0 channel1 interrupt 
    DMA0_Channel2_IRQn  = 32,    // DMA0 channel2 interrupt 
    DMA0_Channel3_IRQn  = 33,    // DMA0 channel3 interrupt 
    DMA0_Channel4_IRQn  = 34,    // DMA0 channel4 interrupt 
    DMA0_Channel5_IRQn  = 35,    // DMA0 channel5 interrupt 
    DMA0_Channel6_IRQn  = 36,    // DMA0 channel6 interrupt 
    ADC0_1_IRQn         = 37,    // ADC0 and ADC1 interrupt 
    CAN0_TX_IRQn        = 38,    // CAN0 TX interrupts 
    CAN0_RX0_IRQn       = 39,    // CAN0 RX0 interrupts 
    CAN0_RX1_IRQn       = 40,    // CAN0 RX1 interrupts 
    CAN0_EWMC_IRQn      = 41,    // CAN0 EWMC interrupts 
    EXTI5_9_IRQn        = 42,    // EXTI[9:5] interrupts 
    TIMER0_BRK_IRQn     = 43,    // TIMER0 break interrupts 
    TIMER0_UP_IRQn      = 44,    // TIMER0 update interrupts 
    TIMER0_TRG_CMT_IRQn = 45,    // TIMER0 trigger and commutation interrupts 
    TIMER0_Channel_IRQn = 46,    // TIMER0 channel capture compare interrupts 
    TIMER1_IRQn         = 47,    // TIMER1 interrupt 
    TIMER2_IRQn         = 48,    // TIMER2 interrupt 
    TIMER3_IRQn         = 49,    // TIMER3 interrupts 
    I2C0_EV_IRQn        = 50,    // I2C0 event interrupt 
    I2C0_ER_IRQn        = 51,    // I2C0 error interrupt 
    I2C1_EV_IRQn        = 52,    // I2C1 event interrupt 
    I2C1_ER_IRQn        = 53,    // I2C1 error interrupt 
    SPI0_IRQn           = 54,    // SPI0 interrupt 
    SPI1_IRQn           = 55,    // SPI1 interrupt 
    USART0_IRQn         = 56,    // USART0 interrupt 
    USART1_IRQn         = 57,    // USART1 interrupt 
    USART2_IRQn         = 58,    // USART2 interrupt 
    EXTI10_15_IRQn      = 59,    // EXTI[15:10] interrupts 
    RTC_ALARM_IRQn      = 60,    // RTC alarm interrupt EXTI 
    USBFS_WKUP_IRQn     = 61,    // USBFS wakeup interrupt 

    EXMC_IRQn           = 67,    // EXMC global interrupt 

    TIMER4_IRQn         = 69,    // TIMER4 global interrupt 
    SPI2_IRQn           = 70,    // SPI2 global interrupt 
    UART3_IRQn          = 71,    // UART3 global interrupt 
    UART4_IRQn          = 72,    // UART4 global interrupt 
    TIMER5_IRQn         = 73,    // TIMER5 global interrupt 
    TIMER6_IRQn         = 74,    // TIMER6 global interrupt 
    DMA1_Channel0_IRQn  = 75,    // DMA1 channel0 global interrupt 
    DMA1_Channel1_IRQn  = 76,    // DMA1 channel1 global interrupt 
    DMA1_Channel2_IRQn  = 77,    // DMA1 channel2 global interrupt 
    DMA1_Channel3_IRQn  = 78,    // DMA1 channel3 global interrupt 
    DMA1_Channel4_IRQn  = 79,    // DMA1 channel3 global interrupt 

    CAN1_TX_IRQn        = 82,    // CAN1 TX interrupt 
    CAN1_RX0_IRQn       = 83,    // CAN1 RX0 interrupt 
    CAN1_RX1_IRQn       = 84,    // CAN1 RX1 interrupt 
    CAN1_EWMC_IRQn      = 85,    // CAN1 EWMC interrupt 
    USBFS_IRQn          = 86     // USBFS global interrupt 
  );

//Analog to Digital Converter

  TADC_Registers = record
    STAT      : longword;        // ADC status register 
    CTL0      : longword;        // ADC control register 0 
    CTL1      : longword;        // ADC control register 1 
    SAMPT0    : longword;        // ADC sampling time register 0 
    SAMPT1    : longword;        // ADC sampling time register 1 
    IOFF0     : longword;        // ADC inserted channel data offset register 0 
    IOFF1     : longword;        // ADC inserted channel data offset register 1 
    IOFF2     : longword;        // ADC inserted channel data offset register 2 
    IOFF3     : longword;        // ADC inserted channel data offset register 3 
    WDHT      : longword;        // ADC watchdog high threshold register 
    WDLT      : longword;        // ADC watchdog low threshold register 
    RSQ0      : longword;        // ADC regular sequence register 0 
    RSQ1      : longword;        // ADC regular sequence register 1 
    RSQ2      : longword;        // ADC regular sequence register 2 
    ISQ       : longword;        // ADC inserted sequence register 
    IDATA0    : longword;        // ADC inserted data register 0 
    IDATA1    : longword;        // ADC inserted data register 1 
    IDATA2    : longword;        // ADC inserted data register 2 
    IDATA3    : longword;        // ADC inserted data register 3 
    RDATA     : longword;        // ADC regular data register 
    RESERVED0 : array[1..12] of longword;
    OVSCR     : longword;        // ADC oversample control register 
  end;

  TCAN_MAILBOX_Registers = record
    TMI       : longword;        // CAN transmit mailbox0 identifier register 
    TMP       : longword;        // CAN transmit mailbox0 property register 
    TMDATA0   : longword;        // CAN transmit mailbox0 data0 register 
    TMDATA1   : longword;        // CAN transmit mailbox0 data1 register 
  end;

  TCAN_FIFO_Registers = record
    RFIFOMI   : longword;        // CAN receive FIFO0 mailbox identifier register 
    RFIFOMP0  : longword;        // CAN receive FIFO0 mailbox property register 
    RFIFOMDATA0 : longword;      // CAN receive FIFO0 mailbox data0 register 
    RFIFOMDATA1 : longword;      // CAN receive FIFO0 mailbox data1 register 
  end;

  TCAN_FILTER_Registers = record
    DATA0     : longword;
    DATA1     : longword;
  end;

(*
  TCAN_Registers = record
    CTL       : longword;        // CAN control register 
    STAT      : longword;        // CAN status register 
    TSTAT     : longword;        // CAN transmit status register 
    RFIFO0    : longword;        // CAN receive FIFO0 register 
    RFIFO1    : longword;        // CAN receive FIFO1 register 
    INTEN     : longword;        // CAN interrupt enable register 
    ERR       : longword;        // CAN error register 
    BT        : longword;        // CAN bit timing register 
    RESERVED0 : array[1..12] of longword;
    TXMAILBOX : array[0..2] of TCAN_MAILBOX_Registers 
    RXFIFO    : array[0..1] of TCAN_FIFO_Registers 

    1CC        // CAN receive FIFO1 mailbox data1 register 
    RESERVED0 : array[] of longword; 

    FCTL      : longword;        // CAN filter control register 
    FMCFG     : longword;        // CAN filter mode register 
    RESERVED1 : longword;
    FSCFG     : longword;        // CAN filter scale register 
    RESERVED2 : longword;
    FAFIFO    : longword;        // CAN filter associated FIFO register 
    RESERVED3 : longword;
    FW        : longword;    21C // CAN filter working register 

    F : array[0..27] of TCAN_FILTER_Registers;

/* CAN transmit mailbox bank 
    TMI(canx, bank)                REG32((canx) + 0x180U + ((bank) * 0x10U))               // CAN transmit mailbox identifier register 
    TMP(canx, bank)                REG32((canx) + 0x184U + ((bank) * 0x10U))               // CAN transmit mailbox property register 
    TMDATA0(canx, bank)            REG32((canx) + 0x188U + ((bank) * 0x10U))               // CAN transmit mailbox data0 register 
    TMDATA1(canx, bank)            REG32((canx) + 0x18CU + ((bank) * 0x10U))               // CAN transmit mailbox data1 register 

/* CAN filter bank 
    FDATA0 : longword;        // CAN filter data 0 register 
    FDATA1 : longword;        // CAN filter data 1 register 

/* CAN receive fifo mailbox bank 
    RFIFOMI(canx, bank)            REG32((canx) + 0x1B0U + ((bank) * 0x10U))               // CAN receive FIFO mailbox identifier register 
    RFIFOMP(canx, bank)            REG32((canx) + 0x1B4U + ((bank) * 0x10U))               // CAN receive FIFO mailbox property register 
    RFIFOMDATA0(canx, bank)        REG32((canx) + 0x1B8U + ((bank) * 0x10U))               // CAN receive FIFO mailbox data0 register 
    RFIFOMDATA1(canx, bank)        REG32((canx) + 0x1BCU + ((bank) * 0x10U))               // CAN receive FIFO mailbox data1 register 
  end;

*)

  TCRC_Registers = record
    DATA      : longword;        // CRC data register 
    FDATA     : longword;        // CRC free data register 
    CTL       : longword;        // CRC control register 
  end;

  TDAC_Registers = record
    CTL       : longword;        // DAC control register 
    SWT       : longword;        // DAC software trigger register 
    DAC0_R12DH: longword;        // DAC0 12-bit right-aligned data holding register 
    DAC0_L12DH: longword;        // DAC0 12-bit left-aligned data holding register 
    DAC0_R8DH : longword;        // DAC0 8-bit right-aligned data holding register 
    DAC1_R12DH: longword;        // DAC1 12-bit right-aligned data holding register 
    DAC1_L12DH: longword;        // DAC1 12-bit left-aligned data holding register 
    DAC1_R8DH : longword;        // DAC1 8-bit right-aligned data holding register 
    DACC_R12DH: longword;        // DAC concurrent mode 12-bit right-aligned data holding register 
    DACC_L12DH: longword;        // DAC concurrent mode 12-bit left-aligned data holding register 
    DACC_R8DH : longword;        // DAC concurrent mode 8-bit right-aligned data holding register 
    DAC0_DO   : longword;        // DAC0 data output register 
    DAC1_DO   : longword;        // DAC1 data output register   
  end;

//DMA Controller

  TDMA_Channel_Registers = record
    CTL       : longword;        // DMA channel 0 control register 
    CNT       : longword;        // DMA channel 0 counter register 
    PADDR     : longword;        // DMA channel 0 peripheral base address register 
    MADDR     : longword;        // DMA channel 0 memory base address register 
    RESERVED0 : longword;
  end;

  TDMA_Registers = record
    INTF      : longword;        // DMA interrupt flag register 
    INTC      : longword;        // DMA interrupt flag clear register 
    CHANNEL   : array[1..6] of TDMA_Channel_Registers;
  end;

//External Interrupt/Event Controller

  TEXMC_Registers = record
    SNCTL0    : longword;        // EXMC SRAM/NOR flash control register 0 
    SNTCFG0   : longword;        // EXMC SRAM/NOR flash timing configuration register 0 
    RESERVED0 : array[0..$3f0] of longword;
    SNWTCFG0  : longword;        // EXMC SRAM/NOR flash write timing configuration register 0 
  end;

  TEXTI_Registers = record
    INTEN     : longword;        // interrupt enable register 
    EVEN      : longword;        // event enable register 
    RTEN      : longword;        // rising edge trigger enable register 
    FTEN      : longword;        // falling trigger enable register 
    SWIEV     : longword;        // software interrupt event register 
    PD        : longword;        // pending register 
  end;

  TFMC_Registers = record
    WS        : longword;        // FMC wait state register 
    KEY       : longword;        // FMC unlock key register 
    OBKEY     : longword;        // FMC option bytes unlock key register 
    STAT      : longword;        // FMC status register 
    CTL       : longword;        // FMC control register 
    ADDR      : longword;        // FMC address register 
    RESERVED0 : longword;
    OBSTAT    : longword;        // FMC option bytes status register 
    WP        : longword;        // FMC erase/program protection register 
    RESERVED1 : array[0..$df] of longword;
    PID       : longword;        // FMC product ID register 
  end;

  TOB_Registers = record
    SPC       : word;            // option byte security protection value 
    USER      : word;            // option byte user value 
    RESERVED0 : word;
    RESERVED1 : word;
    WP0       : word;            // option byte write protection 0 
    WP1       : word;            // option byte write protection 1 
    WP2       : word;            // option byte write protection 2 
    WP3       : word;            // option byte write protection 3 
  end;

  TFWDGT_Registers = record
    CTL       : longword;        // FWDGT control register 
    PSC       : longword;        // FWDGT prescaler register 
    RLD       : longword;        // FWDGT reload register 
    STAT      : longword;        // FWDGT status register 
  end;

  TGPIO_Registers = record
    CTL0      : longword;        // GPIO port control register 0 
    CTL1      : longword;        // GPIO port control register 1 
    ISTAT     : longword;        // GPIO port input status register 
    OCTL      : longword;        // GPIO port output control register 
    BOP       : longword;        // GPIO port bit operation register 
    BC        : longword;        // GPIO bit clear register 
    LOCK      : longword;        // GPIO port configuration lock register 
  end;

  TAFIO_Registers = record
    EC        : longword;        // AFIO event control register 
    PCF0      : longword;        // AFIO port configuration register 0 
    EXTISS0   : longword;        // AFIO port EXTI sources selection register 0 
    EXTISS1   : longword;        // AFIO port EXTI sources selection register 1 
    EXTISS2   : longword;        // AFIO port EXTI sources selection register 2 
    EXTISS3   : longword;        // AFIO port EXTI sources selection register 3 
    PCF1      : longword;        // AFIO port configuration register 1 
  end;

  TI2C_Registers = record
    CTL0      : longword;        // I2C control register 0 
    CTL1      : longword;        // I2C control register 1 
    SADDR0    : longword;        // I2C slave address register 0 
    SADDR1    : longword;        // I2C slave address register 
    DATA      : longword;        // I2C transfer buffer register 
    STAT0     : longword;        // I2C transfer status register 0 
    STAT1     : longword;        // I2C transfer status register 
    CKCFG     : longword;        // I2C clock configure register 
    RT        : longword;        // I2C rise time register 
    RESERVED0 : array[0..$6f] of longword;
    FMPCFG    : longword;        // I2C fast-mode-plus configure register 
  end;

  TPMU_Registers = record
    CTL       : longword;        // PMU control register 
    CS        : longword;        // PMU control and status register 
  end;

  TRCU_Registers = record
    CTL       : longword;        // control register 
    CFG0      : longword;        // clock configuration register 0 
    INT       : longword;        // clock interrupt register 
    APB2RST   : longword;        // APB2 reset register 
    APB1RST   : longword;        // APB1 reset register 
    AHBEN     : longword;        // AHB1 enable register 
    APB2EN    : longword;        // APB2 enable register 
    APB1EN    : longword;        // APB1 enable register 
    BDCTL     : longword;        // backup domain control register 
    RSTSCK    : longword;        // reset source / clock register 
    AHBRST    : longword;        // AHB reset register 
    CFG1      : longword;        // clock configuration register 1 
    RESERVED0 : longword;
    DSV       : longword;        // deep-sleep mode voltage register 
 end;

//Real-Time Clock

  TRTC_Registers = record
    INTEN     : longword;        // interrupt enable register 
    CTL       : longword;        // control register 
    PSCH      : longword;        // prescaler high register 
    PSCL      : longword;        // prescaler low register 
    DIVH      : longword;        // divider high register 
    DIVL      : longword;        // divider low register 
    CNTH      : longword;        // counter high register 
    CNTL      : longword;        // counter low register 
    ALRMH     : longword;        // alarm high register 
    ALRML     : longword;        // alarm low register 
  end;

//Serial Peripheral Interface

  TSPI_Registers = record
    CTL0      : longword;        // SPI control register 0 
    CTL1      : longword;        // SPI control register 1 
    STAT      : longword;        // SPI status register 
    DATA      : longword;        // SPI data register 
    CRCPOLY   : longword;        // SPI CRC polynomial register 
    RCRC      : longword;        // SPI receive CRC register 
    TCRC      : longword;        // SPI transmit CRC register 
    I2SCTL    : longword;        // SPI I2S control register 
    I2SPSC    : longword;        // SPI I2S clock prescaler register 
  end;

//TIM

  TTIMER_Registers = record
    CTL0      : longword;        // TIMER control register 0 
    CTL1      : longword;        // TIMER control register 1 
    SMCFG     : longword;        // TIMER slave mode configuration register 
    DMAINTEN  : longword;        // TIMER DMA and interrupt enable register 
    INTF      : longword;        // TIMER interrupt flag register 
    SWEVG     : longword;        // TIMER software event generation register 
    CHCTL0    : longword;        // TIMER channel control register 0 
    CHCTL1    : longword;        // TIMER channel control register 1 
    CHCTL2    : longword;        // TIMER channel control register 2 
    CNT       : longword;        // TIMER counter register 
    PSC       : longword;        // TIMER prescaler register 
    CAR       : longword;        // TIMER counter auto reload register 
    CREP      : longword;        // TIMER counter repetition register 
    CH0CV     : longword;        // TIMER channel 0 capture/compare value register 
    CH1CV     : longword;        // TIMER channel 1 capture/compare value register 
    CH2CV     : longword;        // TIMER channel 2 capture/compare value register 
    CH3CV     : longword;        // TIMER channel 3 capture/compare value register 
    CCHP      : longword;        // TIMER channel complementary protection register 
    DMACFG    : longword;        // TIMER DMA configuration register 
    DMATB     : longword;        // TIMER DMA transfer buffer register 
  end;

//Universal Synchronous Asynchronous Receiver Transmitter

  TUSART_Registers = record
    STAT      : longword;        // USART status register 
    DATA      : longword;        // USART data register 
    BAUD      : longword;        // USART baud rate register 
    CTL0      : longword;        // USART control register 0 
    CTL1      : longword;        // USART control register 1 
    CTL2      : longword;        // USART control register 2 
    GP        : longword;        // USART guard time and prescaler register 
  end;

//Window WATCHDOG

  TWWDGT_Registers = record
    CTL       : longword;        // WWDGT control register 
    CFG       : longword;        // WWDGT configuration register 
    STAT      : longword;        // WWDGT status register 
  end;

const
  FLASH_BASE  = $08000000;       // FLASH base address in the alias region
  SRAM_BASE   = $20000000;       // SRAM base address in the alias region
  OB_BASE     = $1FFFF800;       // OB base address
  DBG_BASE    = $E0042000;       // DBG base address
  EXMC_BASE   = $A0000000;       // EXMC register base address

// peripheral memory map
  APB1_BUS_BASE = $40000000;     // apb1 base address
  APB2_BUS_BASE = $40010000;     // apb2 base address
  AHB1_BUS_BASE = $40018000;     // ahb1 base address
  AHB3_BUS_BASE = $60000000;     // ahb3 base address

// advanced peripheral bus 1 memory map
  TIMER_BASE  = APB1_BUS_BASE + $00000000;  // TIMER base address
  TIMER0_BASE = TIMER_BASE + $00012C00;
  TIMER1_BASE = TIMER_BASE + $00000000;
  TIMER2_BASE = TIMER_BASE + $00000400;
  TIMER3_BASE = TIMER_BASE + $00000800;
  TIMER4_BASE = TIMER_BASE + $00000C00;
  TIMER5_BASE = TIMER_BASE + $00001000;
  TIMER6_BASE = TIMER_BASE + $00001400;

  RTC_BASE    = APB1_BUS_BASE + $00002800; // RTC base address
  WWDGT_BASE  = APB1_BUS_BASE + $00002C00; // WWDGT base address
  FWDGT_BASE  = APB1_BUS_BASE + $00003000; // FWDGT base address

  SPI_BASE    = APB1_BUS_BASE + $00003800; // SPI base address
  SPI0_BASE   = SPI_BASE + $0000F800;      // SPI base address
  SPI1_BASE   = SPI_BASE;                  // SPI base address
  SPI2_BASE   = SPI_BASE + $00000400;      // SPI base address

  USART_BASE  = APB1_BUS_BASE + $00004400; // USART base address
  USART0_BASE = USART_BASE+$0000F400;      // USART0 base address 
  USART1_BASE = USART_BASE;                // USART1 base address 
  USART2_BASE = USART_BASE+$00000400;      // USART2 base address 
  UART3_BASE  = USART_BASE+$00000800;      // UART3 base address 
  UART4_BASE  = USART_BASE+$00000C00;      // UART4 base address 

  I2C_BASE    = APB1_BUS_BASE + $00005400; // I2C base address
  I2C0_BASE   = I2C_BASE;                  // I2C0 base address
  I2C1_BASE   = I2C_BASE + $00000400;      // I2C1 base address

  CAN_BASE    = APB1_BUS_BASE + $00006400; // CAN base address
  CAN0_BASE   = CAN_BASE;                  // CAN0 base address */
  CAN1_BASE   = CAN_BASE + $00000400;      // CAN1 base address */

  BKP_BASE    = APB1_BUS_BASE + $00006C00; // BKP base address
  PMU_BASE    = APB1_BUS_BASE + $00007000; // PMU base address
  DAC_BASE    = APB1_BUS_BASE + $00007400; // DAC base address

// advanced peripheral bus 2 memory map
  AFIO_BASE   = APB2_BUS_BASE + $00000000;  // AFIO base address
  EXTI_BASE   = APB2_BUS_BASE + $00000400;  // EXTI base address

  GPIO_BASE   = APB2_BUS_BASE + $00000800;  // GPIO base address
  GPIOA_BASE  = GPIO_BASE + $00000000;
  GPIOB_BASE  = GPIO_BASE + $00000400;
  GPIOC_BASE  = GPIO_BASE + $00000800;
  GPIOD_BASE  = GPIO_BASE + $00000C00;
  GPIOE_BASE  = GPIO_BASE + $00001000;

  ADC_BASE    = APB2_BUS_BASE + $00002400;  // ADC base address
  ADC0_BASE   = ADC_BASE;                   // ADC0 base address
  ADC1_BASE   = ADC_BASE + $00000400;       // ADC1 base address

// advanced high performance bus 1 memory map
  DMA_BASE    = AHB1_BUS_BASE + $00008000;  // DMA base address
  DMA0_BASE   = DMA_BASE;                   // DMA base address
  DMA1_BASE   = DMA_BASE + $00000400;       // DMA base address
  RCU_BASE    = AHB1_BUS_BASE + $00009000;  // RCU base address
  FMC_BASE    = AHB1_BUS_BASE + $0000A000;  // FMC base address
  CRC_BASE    = AHB1_BUS_BASE + $0000B000;  // CRC base address
  USBFS_BASE  = AHB1_BUS_BASE + $0FFE8000;  // USBFS base address

var
  ADC0        : TADC_Registers   absolute ADC0_BASE;
  ADC1        : TADC_Registers   absolute ADC1_BASE;
  CRC         : TCRC_Registers   absolute CRC_BASE;
  DAC         : TDAC_Registers   absolute DAC_BASE;
  DMA0        : TDMA_Registers   absolute DMA1_BASE;
  DMA1        : TDMA_Registers   absolute DMA1_BASE;
  GPIOA       : TGPIO_Registers  absolute GPIOA_BASE;
  GPIOB       : TGPIO_Registers  absolute GPIOB_BASE;
  GPIOC       : TGPIO_Registers  absolute GPIOC_BASE;
  GPIOD       : TGPIO_Registers  absolute GPIOD_BASE;
  GPIOE       : TGPIO_Registers  absolute GPIOE_BASE;
  I2C0        : TI2C_Registers   absolute I2C0_BASE;
  I2C1        : TI2C_Registers   absolute I2C1_BASE;
  OB          : TOB_Registers    absolute OB_BASE;
  RTC         : TRTC_Registers   absolute RTC_BASE;
  SPI0        : TSPI_Registers   absolute SPI0_BASE;
  SPI1        : TSPI_Registers   absolute SPI1_BASE;
  SPI2        : TSPI_Registers   absolute SPI2_BASE;
  TIMER0      : TTIMER_Registers absolute TIMER0_BASE;
  TIMER1      : TTIMER_Registers absolute TIMER1_BASE;
  TIMER2      : TTIMER_Registers absolute TIMER2_BASE;
  TIMER3      : TTIMER_Registers absolute TIMER3_BASE;
  TIMER4      : TTIMER_Registers absolute TIMER4_BASE;
  TIMER5      : TTIMER_Registers absolute TIMER5_BASE;
  TIMER6      : TTIMER_Registers absolute TIMER6_BASE;
  USART0      : TUSART_Registers absolute USART0_BASE;
  USART1      : TUSART_Registers absolute USART1_BASE;
  USART2      : TUSART_Registers absolute USART2_BASE;
  UART3       : TUSART_Registers absolute UART3_BASE;
  UART4       : TUSART_Registers absolute UART4_BASE;
  WWDGT       : TWWDGT_Registers absolute WWDGT_BASE;

implementation
  procedure CLIC_RESERVED_ISR; external name 'CLIC_RESERVED_ISR';
  procedure CLIC_SFT_ISR;      external name 'CLIC_SFT_ISR';
  procedure CLIC_TMR_ISR;      external name 'CLIC_TMR_ISR';
  procedure CLIC_BWEI_ISR;     external name 'CLIC_BWEI_ISR';
  procedure CLIC_PMOVI_ISR;    external name 'CLIC_PMOVI_ISR';
  procedure WWDGT_ISR;         external name 'WWDGT_ISR';
  procedure LVD_ISR;           external name 'LVD_ISR';
  procedure TAMPER_ISR;        external name 'TAMPER_ISR';
  procedure RTC_ISR;           external name 'RTC_ISR';
  procedure FMC_ISR;           external name 'FMC_ISR';
  procedure RCU_CTC_ISR;       external name 'RCU_CTC_ISR';
  procedure EXTI0_ISR;         external name 'EXTI0_ISR';
  procedure EXTI1_ISR;         external name 'EXTI1_ISR';
  procedure EXTI2_ISR;         external name 'EXTI2_ISR';
  procedure EXTI3_ISR;         external name 'EXTI3_ISR';
  procedure EXTI4_ISR;         external name 'EXTI4_ISR';
  procedure DMA0_Channel0_ISR; external name 'DMA0_Channel0_ISR';
  procedure DMA0_Channel1_ISR; external name 'DMA0_Channel1_ISR';
  procedure DMA0_Channel2_ISR; external name 'DMA0_Channel2_ISR';
  procedure DMA0_Channel3_ISR; external name 'DMA0_Channel3_ISR';
  procedure DMA0_Channel4_ISR; external name 'DMA0_Channel4_ISR';
  procedure DMA0_Channel5_ISR; external name 'DMA0_Channel5_ISR';
  procedure DMA0_Channel6_ISR; external name 'DMA0_Channel6_ISR';
  procedure ADC0_1_ISR;        external name 'ADC0_1_ISR';
  procedure CAN0_TX_ISR;       external name 'CAN0_TX_ISR';
  procedure CAN0_RX0_ISR;      external name 'CAN0_RX0_ISR';
  procedure CAN0_RX1_ISR;      external name 'CAN0_RX1_ISR';
  procedure CAN0_EWMC_ISR;     external name 'CAN0_EWMC_ISR';
  procedure EXTI5_9_ISR;       external name 'EXTI5_9_ISR';
  procedure TIMER0_BRK_ISR;    external name 'TIMER0_BRK_ISR';
  procedure TIMER0_UP_ISR;     external name 'TIMER0_UP_ISR';
  procedure TIMER0_TRG_CMT_ISR;external name 'TIMER0_TRG_CMT_ISR';
  procedure TIMER0_Channel_ISR;external name 'TIMER0_Channel_ISR';
  procedure TIMER1_ISR;        external name 'TIMER1_ISR';
  procedure TIMER2_ISR;        external name 'TIMER2_ISR';
  procedure TIMER3_ISR;        external name 'TIMER3_ISR';
  procedure I2C0_EV_ISR;       external name 'I2C0_EV_ISR';
  procedure I2C0_ER_ISR;       external name 'I2C0_ER_ISR';
  procedure I2C1_EV_ISR;       external name 'I2C1_EV_ISR';
  procedure I2C1_ER_ISR;       external name 'I2C1_ER_ISR';
  procedure SPI0_ISR;          external name 'SPI0_ISR';
  procedure SPI1_ISR;          external name 'SPI1_ISR';
  procedure USART0_ISR;        external name 'USART0_ISR';
  procedure USART1_ISR;        external name 'USART1_ISR';
  procedure USART2_ISR;        external name 'USART2_ISR';
  procedure EXTI10_15_ISR;     external name 'EXTI10_15_ISR';
  procedure RTC_ALARM_ISR;     external name 'RTC_ALARM_ISR';
  procedure USBFS_WKUP_ISR;    external name 'USBFS_WKUP_ISR';
  procedure EXMC_ISR;          external name 'EXMC_ISR';
  procedure TIMER4_ISR;        external name 'TIMER4_ISR';
  procedure SPI2_ISR;          external name 'SPI2_ISR';
  procedure UART3_ISR;         external name 'UART3_ISR';
  procedure UART4_ISR;         external name 'UART4_ISR';
  procedure TIMER5_ISR;        external name 'TIMER5_ISR';
  procedure TIMER6_ISR;        external name 'TIMER6_ISR';
  procedure DMA1_Channel0_ISR; external name 'DMA1_Channel0_ISR';
  procedure DMA1_Channel1_ISR; external name 'DMA1_Channel1_ISR';
  procedure DMA1_Channel2_ISR; external name 'DMA1_Channel2_ISR';
  procedure DMA1_Channel3_ISR; external name 'DMA1_Channel3_ISR';
  procedure DMA1_Channel4_ISR; external name 'DMA1_Channel4_ISR';
  procedure CAN1_TX_ISR;       external name 'CAN1_TX_ISR';
  procedure CAN1_RX0_ISR;      external name 'CAN1_RX0_ISR';
  procedure CAN1_RX1_ISR;      external name 'CAN1_RX1_ISR';
  procedure CAN1_EWMC_ISR;     external name 'CAN1_EWMC_ISR';
  procedure USBFS_ISR;         external name 'USBFS_ISR';
  
  {$i riscv32_start.inc}

  procedure Vectors; assembler; nostackframe;
  label interrupt_vectors;
  asm
     .section ".init.interrupt_vectors"
  interrupt_vectors:
    .long CLIC_RESERVED_ISR
    .long 0
    .long 0
    .long CLIC_SFT_ISR
    .long 0
    .long 0
    .long 0
    .long CLIC_TMR_ISR
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long CLIC_BWEI_ISR
    .long CLIC_PMOVI_ISR
    .long WWDGT_ISR
    .long LVD_ISR
    .long TAMPER_ISR
    .long RTC_ISR
    .long FMC_ISR
    .long RCU_CTC_ISR
    .long EXTI0_ISR
    .long EXTI1_ISR
    .long EXTI2_ISR
    .long EXTI3_ISR
    .long EXTI4_ISR
    .long DMA0_Channel0_ISR
    .long DMA0_Channel1_ISR
    .long DMA0_Channel2_ISR
    .long DMA0_Channel3_ISR
    .long DMA0_Channel4_ISR
    .long DMA0_Channel5_ISR
    .long DMA0_Channel6_ISR
    .long ADC0_1_ISR
    .long CAN0_TX_ISR
    .long CAN0_RX0_ISR
    .long CAN0_RX1_ISR
    .long CAN0_EWMC_ISR
    .long EXTI5_9_ISR
    .long TIMER0_BRK_ISR
    .long TIMER0_UP_ISR
    .long TIMER0_TRG_CMT_ISR
    .long TIMER0_Channel_ISR
    .long TIMER1_ISR
    .long TIMER2_ISR
    .long TIMER3_ISR
    .long I2C0_EV_ISR
    .long I2C0_ER_ISR
    .long I2C1_EV_ISR
    .long I2C1_ER_ISR
    .long SPI0_ISR
    .long SPI1_ISR
    .long USART0_ISR
    .long USART1_ISR
    .long USART2_ISR
    .long EXTI10_15_ISR
    .long RTC_ALARM_ISR
    .long USBFS_WKUP_ISR
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    .long EXMC_ISR
    .long 0
    .long TIMER4_ISR
    .long SPI2_ISR
    .long UART3_ISR
    .long UART4_ISR
    .long TIMER5_ISR
    .long TIMER6_ISR
    .long DMA1_Channel0_ISR
    .long DMA1_Channel1_ISR
    .long DMA1_Channel2_ISR
    .long DMA1_Channel3_ISR
    .long DMA1_Channel4_ISR
    .long 0
    .long 0
    .long CAN1_TX_ISR
    .long CAN1_RX0_ISR
    .long CAN1_RX1_ISR
    .long CAN1_EWMC_ISR
    .long USBFS_ISR

    .weak CLIC_RESERVED_ISR
    .weak CLIC_SFT_ISR
    .weak CLIC_TMR_ISR
    .weak CLIC_BWEI_ISR
    .weak CLIC_PMOVI_ISR
    .weak WWDGT_ISR
    .weak LVD_ISR
    .weak TAMPER_ISR
    .weak RTC_ISR
    .weak FMC_ISR
    .weak RCU_CTC_ISR
    .weak EXTI0_ISR
    .weak EXTI1_ISR
    .weak EXTI2_ISR
    .weak EXTI3_ISR
    .weak EXTI4_ISR
    .weak DMA0_Channel0_ISR
    .weak DMA0_Channel1_ISR
    .weak DMA0_Channel2_ISR
    .weak DMA0_Channel3_ISR
    .weak DMA0_Channel4_ISR
    .weak DMA0_Channel5_ISR
    .weak DMA0_Channel6_ISR
    .weak ADC0_1_ISR
    .weak CAN0_TX_ISR
    .weak CAN0_RX0_ISR
    .weak CAN0_RX1_ISR
    .weak CAN0_EWMC_ISR
    .weak EXTI5_9_ISR
    .weak TIMER0_BRK_ISR
    .weak TIMER0_UP_ISR
    .weak TIMER0_TRG_CMT_ISR
    .weak TIMER0_Channel_ISR
    .weak TIMER1_ISR
    .weak TIMER2_ISR
    .weak TIMER3_ISR
    .weak I2C0_EV_ISR
    .weak I2C0_ER_ISR
    .weak I2C1_EV_ISR
    .weak I2C1_ER_ISR
    .weak SPI0_ISR
    .weak SPI1_ISR
    .weak USART0_ISR
    .weak USART1_ISR
    .weak USART2_ISR
    .weak EXTI10_15_ISR
    .weak RTC_ALARM_ISR
    .weak USBFS_WKUP_ISR
    .weak EXMC_ISR
    .weak TIMER4_ISR
    .weak SPI2_ISR
    .weak UART3_ISR
    .weak UART4_ISR
    .weak TIMER5_ISR
    .weak TIMER6_ISR
    .weak DMA1_Channel0_ISR
    .weak DMA1_Channel1_ISR
    .weak DMA1_Channel2_ISR
    .weak DMA1_Channel3_ISR
    .weak DMA1_Channel4_ISR
    .weak CAN1_TX_ISR
    .weak CAN1_RX0_ISR
    .weak CAN1_RX1_ISR
    .weak CAN1_EWMC_ISR
    .weak USBFS_ISR

    .set CLIC_RESERVED_ISR, HaltProc
    .set CLIC_SFT_ISR, HaltProc
    .set CLIC_TMR_ISR, HaltProc
    .set CLIC_BWEI_ISR, HaltProc
    .set CLIC_PMOVI_ISR, HaltProc
    .set WWDGT_ISR, HaltProc
    .set LVD_ISR, HaltProc
    .set TAMPER_ISR, HaltProc
    .set RTC_ISR, HaltProc
    .set FMC_ISR, HaltProc
    .set RCU_CTC_ISR, HaltProc
    .set EXTI0_ISR, HaltProc
    .set EXTI1_ISR, HaltProc
    .set EXTI2_ISR, HaltProc
    .set EXTI3_ISR, HaltProc
    .set EXTI4_ISR, HaltProc
    .set DMA0_Channel0_ISR, HaltProc
    .set DMA0_Channel1_ISR, HaltProc
    .set DMA0_Channel2_ISR, HaltProc
    .set DMA0_Channel3_ISR, HaltProc
    .set DMA0_Channel4_ISR, HaltProc
    .set DMA0_Channel5_ISR, HaltProc
    .set DMA0_Channel6_ISR, HaltProc
    .set ADC0_1_ISR, HaltProc
    .set CAN0_TX_ISR, HaltProc
    .set CAN0_RX0_ISR, HaltProc
    .set CAN0_RX1_ISR, HaltProc
    .set CAN0_EWMC_ISR, HaltProc
    .set EXTI5_9_ISR, HaltProc
    .set TIMER0_BRK_ISR, HaltProc
    .set TIMER0_UP_ISR, HaltProc
    .set TIMER0_TRG_CMT_ISR, HaltProc
    .set TIMER0_Channel_ISR, HaltProc
    .set TIMER1_ISR, HaltProc
    .set TIMER2_ISR, HaltProc
    .set TIMER3_ISR, HaltProc
    .set I2C0_EV_ISR, HaltProc
    .set I2C0_ER_ISR, HaltProc
    .set I2C1_EV_ISR, HaltProc
    .set I2C1_ER_ISR, HaltProc
    .set SPI0_ISR, HaltProc
    .set SPI1_ISR, HaltProc
    .set USART0_ISR, HaltProc
    .set USART1_ISR, HaltProc
    .set USART2_ISR, HaltProc
    .set EXTI10_15_ISR, HaltProc
    .set RTC_ALARM_ISR, HaltProc
    .set USBFS_WKUP_ISR, HaltProc
    .set EXMC_ISR, HaltProc
    .set TIMER4_ISR, HaltProc
    .set SPI2_ISR, HaltProc
    .set UART3_ISR, HaltProc
    .set UART4_ISR, HaltProc
    .set TIMER5_ISR, HaltProc
    .set TIMER6_ISR, HaltProc
    .set DMA1_Channel0_ISR, HaltProc
    .set DMA1_Channel1_ISR, HaltProc
    .set DMA1_Channel2_ISR, HaltProc
    .set DMA1_Channel3_ISR, HaltProc
    .set DMA1_Channel4_ISR, HaltProc
    .set CAN1_TX_ISR, HaltProc
    .set CAN1_RX0_ISR, HaltProc
    .set CAN1_RX1_ISR, HaltProc
    .set CAN1_EWMC_ISR, HaltProc
    .set USBFS_ISR, HaltProc
    .text
  end;
end.
