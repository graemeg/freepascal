unit fe310g000;
interface
{$PACKRECORDS 2}
{$GOTO ON}
{$MODESWITCH ADVANCEDRECORDS}

type
  TIRQn_Enum   = (
    NonMaskableInt_IRQn = -14,        // 2 Non Maskable Interrupt
    MemoryManagement_IRQn = -12,      // 4 Cortex-M4 Memory Management Interrupt
    BusFault_IRQn = -11,              // 5 Cortex-M4 Bus Fault Interrupt
    UsageFault_IRQn = -10,            // 6 Cortex-M4 Usage Fault Interrupt
    SVCall_IRQn = -5,                 // 11 Cortex-M4 SV Call Interrupt
    DebugMonitor_IRQn = -4,           // 12 Cortex-M4 Debug Monitor Interrupt
    PendSV_IRQn = -2,                 // 14 Cortex-M4 Pend SV Interrupt
    SysTick_IRQn = -1,                // 15 Cortex-M4 System Tick Interrupt
    WWDG_IRQn  = 0,                   // Window WatchDog Interrupt
    PVD_IRQn   = 1,                   // PVD through EXTI Line detection Interrupt
    TAMP_STAMP_IRQn = 2,              // Tamper and TimeStamp interrupts through the EXTI line
    RTC_WKUP_IRQn = 3,                // RTC Wakeup interrupt through the EXTI line
    FLASH_IRQn = 4,                   // FLASH global Interrupt
    RCC_IRQn   = 5,                   // RCC global Interrupt
    EXTI0_IRQn = 6,                   // EXTI Line0 Interrupt
    EXTI1_IRQn = 7,                   // EXTI Line1 Interrupt
    EXTI2_IRQn = 8,                   // EXTI Line2 Interrupt
    EXTI3_IRQn = 9,                   // EXTI Line3 Interrupt
    EXTI4_IRQn = 10,                  // EXTI Line4 Interrupt
    DMA1_Stream0_IRQn = 11,           // DMA1 Stream 0 global Interrupt
    DMA1_Stream1_IRQn = 12,           // DMA1 Stream 1 global Interrupt
    DMA1_Stream2_IRQn = 13,           // DMA1 Stream 2 global Interrupt
    DMA1_Stream3_IRQn = 14,           // DMA1 Stream 3 global Interrupt
    DMA1_Stream4_IRQn = 15,           // DMA1 Stream 4 global Interrupt
    DMA1_Stream5_IRQn = 16,           // DMA1 Stream 5 global Interrupt
    DMA1_Stream6_IRQn = 17,           // DMA1 Stream 6 global Interrupt
    ADC_IRQn   = 18,                  // ADC1, ADC2 and ADC3 global Interrupts
    CAN1_TX_IRQn = 19,                // CAN1 TX Interrupt
    CAN1_RX0_IRQn = 20,               // CAN1 RX0 Interrupt
    CAN1_RX1_IRQn = 21,               // CAN1 RX1 Interrupt
    CAN1_SCE_IRQn = 22,               // CAN1 SCE Interrupt
    EXTI9_5_IRQn = 23,                // External Line[9:5] Interrupts
    TIM1_BRK_TIM9_IRQn = 24,          // TIM1 Break interrupt and TIM9 global interrupt
    TIM1_UP_TIM10_IRQn = 25,          // TIM1 Update Interrupt and TIM10 global interrupt
    TIM1_TRG_COM_TIM11_IRQn = 26,     // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    TIM1_CC_IRQn = 27,                // TIM1 Capture Compare Interrupt
    TIM2_IRQn  = 28,                  // TIM2 global Interrupt
    TIM3_IRQn  = 29,                  // TIM3 global Interrupt
    TIM4_IRQn  = 30,                  // TIM4 global Interrupt
    I2C1_EV_IRQn = 31,                // I2C1 Event Interrupt
    I2C1_ER_IRQn = 32,                // I2C1 Error Interrupt
    I2C2_EV_IRQn = 33,                // I2C2 Event Interrupt
    I2C2_ER_IRQn = 34,                // I2C2 Error Interrupt
    SPI1_IRQn  = 35,                  // SPI1 global Interrupt
    SPI2_IRQn  = 36,                  // SPI2 global Interrupt
    USART1_IRQn = 37,                 // USART1 global Interrupt
    USART2_IRQn = 38,                 // USART2 global Interrupt
    USART3_IRQn = 39,                 // USART3 global Interrupt
    EXTI15_10_IRQn = 40,              // External Line[15:10] Interrupts
    RTC_Alarm_IRQn = 41,              // RTC Alarm (A and B) through EXTI Line Interrupt
    OTG_FS_WKUP_IRQn = 42,            // USB OTG FS Wakeup through EXTI line interrupt
    TIM8_BRK_TIM12_IRQn = 43,         // TIM8 Break Interrupt and TIM12 global interrupt
    TIM8_UP_TIM13_IRQn = 44,          // TIM8 Update Interrupt and TIM13 global interrupt
    TIM8_TRG_COM_TIM14_IRQn = 45,     // TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt
    TIM8_CC_IRQn = 46,                // TIM8 Capture Compare Interrupt
    DMA1_Stream7_IRQn = 47,           // DMA1 Stream7 Interrupt
    FSMC_IRQn  = 48,                  // FSMC global Interrupt
    SDIO_IRQn  = 49,                  // SDIO global Interrupt
    TIM5_IRQn  = 50,                  // TIM5 global Interrupt
    SPI3_IRQn  = 51,                  // SPI3 global Interrupt
    UART4_IRQn = 52,                  // UART4 global Interrupt
    UART5_IRQn = 53,                  // UART5 global Interrupt
    TIM6_DAC_IRQn = 54,               // TIM6 global and DAC1&2 underrun error  interrupts
    TIM7_IRQn  = 55,                  // TIM7 global interrupt
    DMA2_Stream0_IRQn = 56,           // DMA2 Stream 0 global Interrupt
    DMA2_Stream1_IRQn = 57,           // DMA2 Stream 1 global Interrupt
    DMA2_Stream2_IRQn = 58,           // DMA2 Stream 2 global Interrupt
    DMA2_Stream3_IRQn = 59,           // DMA2 Stream 3 global Interrupt
    DMA2_Stream4_IRQn = 60,           // DMA2 Stream 4 global Interrupt
    ETH_IRQn   = 61,                  // Ethernet global Interrupt
    ETH_WKUP_IRQn = 62,               // Ethernet Wakeup through EXTI line Interrupt
    CAN2_TX_IRQn = 63,                // CAN2 TX Interrupt
    CAN2_RX0_IRQn = 64,               // CAN2 RX0 Interrupt
    CAN2_RX1_IRQn = 65,               // CAN2 RX1 Interrupt
    CAN2_SCE_IRQn = 66,               // CAN2 SCE Interrupt
    OTG_FS_IRQn = 67,                 // USB OTG FS global Interrupt
    DMA2_Stream5_IRQn = 68,           // DMA2 Stream 5 global interrupt
    DMA2_Stream6_IRQn = 69,           // DMA2 Stream 6 global interrupt
    DMA2_Stream7_IRQn = 70,           // DMA2 Stream 7 global interrupt
    USART6_IRQn = 71,                 // USART6 global interrupt
    I2C3_EV_IRQn = 72,                // I2C3 event interrupt
    I2C3_ER_IRQn = 73,                // I2C3 error interrupt
    OTG_HS_EP1_OUT_IRQn = 74,         // USB OTG HS End Point 1 Out global interrupt
    OTG_HS_EP1_IN_IRQn = 75,          // USB OTG HS End Point 1 In global interrupt
    OTG_HS_WKUP_IRQn = 76,            // USB OTG HS Wakeup through EXTI interrupt
    OTG_HS_IRQn = 77,                 // USB OTG HS global interrupt
    DCMI_IRQn  = 78,                  // DCMI global interrupt
    HASH_RNG_IRQn = 80,               // Hash and RNG global interrupt
    FPU_IRQn   = 81                   // FPU global interrupt
  );

  TGPIO_Registers = record
    VALUE     : longWord;
    INPUT_EN  : longword;
    OUTPUT_EN : longword;
    PORT      : longword;
    PUE       : longword;
    DS        : longword;
    RISE_IE   : longword;
    RISE_IP   : longword;
    FALL_IE   : longword;
    FALL_IP   : longword;
    HIGH_IE   : longword;
    HIGH_IP   : longword;
    LOW_IE    : longword;
    LOW_IP    : longword;
    IOF_EN    : longword;
    IOF_SEL   : longword;
    OUT_XOR   : longword;
  end;

  // Watchdog Registers
  TWDT_Registers = record
    WDOGCFG   : longWord;
    RESERVED0 : longWord;
    WDOGCOUNT : longWord;
    Reserved1 : longWord;
    WDOGS     : longWord;
    RESERVED2 : longWord;
    WDOGFEED  : longWord;
    WDOGKEY   : longWord;
    WDOGCMP   : longWord;
  end;

  // RTC Registers
  TRTCRegisters = record
    RTCCFG    : longWord;
    RESERVED0 : longWord;
    RTCLO_HI  : longWord;
    RESERVED1 : longWord;
    RTCS      : longWord;
    RESERVED2 : longWord;
    RESERVED3 : longWord;
    RESERVED4 : longWord;
    RTCCMP    : longWord;
  end;

  TLFROSC_Registers = record
    LFROSCCFG : longWord;
  end;

  // Backup Registers
  TBackup_Registers = record
    BACKUP : array[0..31] of longWord;
  end;

  // PMU Registers
  TPMU_Registers = record
    PMU_WAKEUP_BASE : longWord;
    RESERVED0       : array[1..7] of longWord;
    PWM_SLEEP_BASE  : longWord;
    RESERVED1       : array[1..7] of longWord;
    PMUIE           : longWord;
    PMUCAUSE        : longWord;
    PMUSLEEP        : longWord;
    PMUKEY          : longWord;
  end;

  TI2C_Registers = record
    PRESCALE_LOW     : longWord;
    PRESCALE_HIGH    : longWord;
    CONTROL          :longWord;
    TRANSMIT_RECEIVE : longWord;
    COMMAND_STATUS   : longWord;
  end;

  TPWM_Registers = record
    PWMCFG    : longWord;
    RESERVED0 : longWord;
    PWMCOUNT  : longWord;
    RESERVED1 : longWord;
    PWMS      : longWord;
    RESERVED2 : array[1..3] of longWord;
    PWMCMP    : array[0..3] of longWord;
  end;

  TSPI_Registers = record
    SCKDIV    : longWord;
    SCKMODE   : longWord;
    RESERVED0 : longWord;
    RESERVED1 : longWord;
    CSID      : longWord;
    CSDEF     : longWord;
    CSMODE    : longWord;
    RESERVED2 : longWord;
    RESERVED3 : array[1..2] of longWord;
    DELAY0    : longWord;
    DELAY1    : longWord;
    RESERVED4 : array[1..4] of longWord;
    FMT       : longWord;
    RESERVED5 : longWord;
    TXDATA    : longWord;
    RXDATA    : longWord;
    TXMARK    : longWord;
    RESERVED6 : longWord;
    RXMARK    : longWord;
    RESERVED7 : array[1..2] of longWord;

    // TODO: Only include if the device supports a direct-map flash interface
    FCTRL     : longWord;
    FFMT      : longWord;
    RESERVED8 : array[1..2] of longWord;
    IE        : longWord;
    RESERVED9 : longWord;
    IP        : longWord;
  end;

  TUART_Registers = record
    TXDATA : longWord;
    RXDATA : longWord;
    TXCTRL : longWord;
    RXCTRL : longWord;
    IE     : longWord;
    IP     : longWord;
    &DIV   : longWord;
  end;

const
  GPIOA_BASE = $10012000;
  UART0_BASE = $10013000;
  SPI0_BASE  = $10014000;
  PWM0_BASE  = $10015000;
  I2C0_BASE  = $10016000;
  UART1_BASE = $10023000;
  SPI1_BASE  = $10024000;
  PWM1_BASE  = $10025000;
  SPI2_BASE  = $10034000;
  PWM2_BASE  = $10035000;
  DTIM_BASE  = $80000000;

var
  GPIOA         : TGPIO_Registers absolute GPIOA_BASE;
  I2C0          : TI2C_Registers absolute I2C0_BASE;
  PWM0          : TPWM_Registers absolute PWM0_BASE;
  PWM1          : TPWM_Registers absolute PWM1_BASE;
  PWM2          : TPWM_Registers absolute PWM2_BASE;
  SPI0          : TSPI_Registers absolute SPI0_BASE;
  SPI1          : TSPI_Registers absolute SPI1_BASE;
  SPI2          : TSPI_Registers absolute SPI2_BASE;
  UART0         : TUART_Registers absolute UART0_BASE;
  UART1         : TUART_Registers absolute UART1_BASE;

implementation

procedure NonMaskableInt_interrupt; external name 'NonMaskableInt_interrupt';
procedure MemoryManagement_interrupt; external name 'MemoryManagement_interrupt';
procedure BusFault_interrupt; external name 'BusFault_interrupt';
procedure UsageFault_interrupt; external name 'UsageFault_interrupt';
procedure SVCall_interrupt; external name 'SVCall_interrupt';
procedure DebugMonitor_interrupt; external name 'DebugMonitor_interrupt';
procedure PendSV_interrupt; external name 'PendSV_interrupt';
procedure SysTick_interrupt; external name 'SysTick_interrupt';
procedure WWDG_interrupt; external name 'WWDG_interrupt';
procedure PVD_interrupt; external name 'PVD_interrupt';
procedure TAMP_STAMP_interrupt; external name 'TAMP_STAMP_interrupt';
procedure RTC_WKUP_interrupt; external name 'RTC_WKUP_interrupt';
procedure FLASH_interrupt; external name 'FLASH_interrupt';
procedure RCC_interrupt; external name 'RCC_interrupt';
procedure EXTI0_interrupt; external name 'EXTI0_interrupt';
procedure EXTI1_interrupt; external name 'EXTI1_interrupt';
procedure EXTI2_interrupt; external name 'EXTI2_interrupt';
procedure EXTI3_interrupt; external name 'EXTI3_interrupt';
procedure EXTI4_interrupt; external name 'EXTI4_interrupt';
procedure DMA1_Stream0_interrupt; external name 'DMA1_Stream0_interrupt';
procedure DMA1_Stream1_interrupt; external name 'DMA1_Stream1_interrupt';
procedure DMA1_Stream2_interrupt; external name 'DMA1_Stream2_interrupt';
procedure DMA1_Stream3_interrupt; external name 'DMA1_Stream3_interrupt';
procedure DMA1_Stream4_interrupt; external name 'DMA1_Stream4_interrupt';
procedure DMA1_Stream5_interrupt; external name 'DMA1_Stream5_interrupt';
procedure DMA1_Stream6_interrupt; external name 'DMA1_Stream6_interrupt';
procedure ADC_interrupt; external name 'ADC_interrupt';
procedure CAN1_TX_interrupt; external name 'CAN1_TX_interrupt';
procedure CAN1_RX0_interrupt; external name 'CAN1_RX0_interrupt';
procedure CAN1_RX1_interrupt; external name 'CAN1_RX1_interrupt';
procedure CAN1_SCE_interrupt; external name 'CAN1_SCE_interrupt';
procedure EXTI9_5_interrupt; external name 'EXTI9_5_interrupt';
procedure TIM1_BRK_TIM9_interrupt; external name 'TIM1_BRK_TIM9_interrupt';
procedure TIM1_UP_TIM10_interrupt; external name 'TIM1_UP_TIM10_interrupt';
procedure TIM1_TRG_COM_TIM11_interrupt; external name 'TIM1_TRG_COM_TIM11_interrupt';
procedure TIM1_CC_interrupt; external name 'TIM1_CC_interrupt';
procedure TIM2_interrupt; external name 'TIM2_interrupt';
procedure TIM3_interrupt; external name 'TIM3_interrupt';
procedure TIM4_interrupt; external name 'TIM4_interrupt';
procedure I2C1_EV_interrupt; external name 'I2C1_EV_interrupt';
procedure I2C1_ER_interrupt; external name 'I2C1_ER_interrupt';
procedure I2C2_EV_interrupt; external name 'I2C2_EV_interrupt';
procedure I2C2_ER_interrupt; external name 'I2C2_ER_interrupt';
procedure SPI1_interrupt; external name 'SPI1_interrupt';
procedure SPI2_interrupt; external name 'SPI2_interrupt';
procedure USART1_interrupt; external name 'USART1_interrupt';
procedure USART2_interrupt; external name 'USART2_interrupt';
procedure USART3_interrupt; external name 'USART3_interrupt';
procedure EXTI15_10_interrupt; external name 'EXTI15_10_interrupt';
procedure RTC_Alarm_interrupt; external name 'RTC_Alarm_interrupt';
procedure OTG_FS_WKUP_interrupt; external name 'OTG_FS_WKUP_interrupt';
procedure TIM8_BRK_TIM12_interrupt; external name 'TIM8_BRK_TIM12_interrupt';
procedure TIM8_UP_TIM13_interrupt; external name 'TIM8_UP_TIM13_interrupt';
procedure TIM8_TRG_COM_TIM14_interrupt; external name 'TIM8_TRG_COM_TIM14_interrupt';
procedure TIM8_CC_interrupt; external name 'TIM8_CC_interrupt';
procedure DMA1_Stream7_interrupt; external name 'DMA1_Stream7_interrupt';
procedure FSMC_interrupt; external name 'FSMC_interrupt';
procedure SDIO_interrupt; external name 'SDIO_interrupt';
procedure TIM5_interrupt; external name 'TIM5_interrupt';
procedure SPI3_interrupt; external name 'SPI3_interrupt';
procedure UART4_interrupt; external name 'UART4_interrupt';
procedure UART5_interrupt; external name 'UART5_interrupt';
procedure TIM6_DAC_interrupt; external name 'TIM6_DAC_interrupt';
procedure TIM7_interrupt; external name 'TIM7_interrupt';
procedure DMA2_Stream0_interrupt; external name 'DMA2_Stream0_interrupt';
procedure DMA2_Stream1_interrupt; external name 'DMA2_Stream1_interrupt';
procedure DMA2_Stream2_interrupt; external name 'DMA2_Stream2_interrupt';
procedure DMA2_Stream3_interrupt; external name 'DMA2_Stream3_interrupt';
procedure DMA2_Stream4_interrupt; external name 'DMA2_Stream4_interrupt';
procedure ETH_interrupt; external name 'ETH_interrupt';
procedure ETH_WKUP_interrupt; external name 'ETH_WKUP_interrupt';
procedure CAN2_TX_interrupt; external name 'CAN2_TX_interrupt';
procedure CAN2_RX0_interrupt; external name 'CAN2_RX0_interrupt';
procedure CAN2_RX1_interrupt; external name 'CAN2_RX1_interrupt';
procedure CAN2_SCE_interrupt; external name 'CAN2_SCE_interrupt';
procedure OTG_FS_interrupt; external name 'OTG_FS_interrupt';
procedure DMA2_Stream5_interrupt; external name 'DMA2_Stream5_interrupt';
procedure DMA2_Stream6_interrupt; external name 'DMA2_Stream6_interrupt';
procedure DMA2_Stream7_interrupt; external name 'DMA2_Stream7_interrupt';
procedure USART6_interrupt; external name 'USART6_interrupt';
procedure I2C3_EV_interrupt; external name 'I2C3_EV_interrupt';
procedure I2C3_ER_interrupt; external name 'I2C3_ER_interrupt';
procedure OTG_HS_EP1_OUT_interrupt; external name 'OTG_HS_EP1_OUT_interrupt';
procedure OTG_HS_EP1_IN_interrupt; external name 'OTG_HS_EP1_IN_interrupt';
procedure OTG_HS_WKUP_interrupt; external name 'OTG_HS_WKUP_interrupt';
procedure OTG_HS_interrupt; external name 'OTG_HS_interrupt';
procedure DCMI_interrupt; external name 'DCMI_interrupt';
procedure HASH_RNG_interrupt; external name 'HASH_RNG_interrupt';
procedure FPU_interrupt; external name 'FPU_interrupt';

{$i riscv32_start.inc}

procedure Vectors; assembler; nostackframe;
label interrupt_vectors;
asm
  .section ".init.interrupt_vectors"
  interrupt_vectors:
  .long _stack_top
  .long Startup
  .long NonMaskableInt_interrupt
  .long 0
  .long MemoryManagement_interrupt
  .long BusFault_interrupt
  .long UsageFault_interrupt
  .long 0
  .long 0
  .long 0
  .long 0
  .long SVCall_interrupt
  .long DebugMonitor_interrupt
  .long 0
  .long PendSV_interrupt
  .long SysTick_interrupt
  .long WWDG_interrupt
  .long PVD_interrupt
  .long TAMP_STAMP_interrupt
  .long RTC_WKUP_interrupt
  .long FLASH_interrupt
  .long RCC_interrupt
  .long EXTI0_interrupt
  .long EXTI1_interrupt
  .long EXTI2_interrupt
  .long EXTI3_interrupt
  .long EXTI4_interrupt
  .long DMA1_Stream0_interrupt
  .long DMA1_Stream1_interrupt
  .long DMA1_Stream2_interrupt
  .long DMA1_Stream3_interrupt
  .long DMA1_Stream4_interrupt
  .long DMA1_Stream5_interrupt
  .long DMA1_Stream6_interrupt
  .long ADC_interrupt
  .long CAN1_TX_interrupt
  .long CAN1_RX0_interrupt
  .long CAN1_RX1_interrupt
  .long CAN1_SCE_interrupt
  .long EXTI9_5_interrupt
  .long TIM1_BRK_TIM9_interrupt
  .long TIM1_UP_TIM10_interrupt
  .long TIM1_TRG_COM_TIM11_interrupt
  .long TIM1_CC_interrupt
  .long TIM2_interrupt
  .long TIM3_interrupt
  .long TIM4_interrupt
  .long I2C1_EV_interrupt
  .long I2C1_ER_interrupt
  .long I2C2_EV_interrupt
  .long I2C2_ER_interrupt
  .long SPI1_interrupt
  .long SPI2_interrupt
  .long USART1_interrupt
  .long USART2_interrupt
  .long USART3_interrupt
  .long EXTI15_10_interrupt
  .long RTC_Alarm_interrupt
  .long OTG_FS_WKUP_interrupt
  .long TIM8_BRK_TIM12_interrupt
  .long TIM8_UP_TIM13_interrupt
  .long TIM8_TRG_COM_TIM14_interrupt
  .long TIM8_CC_interrupt
  .long DMA1_Stream7_interrupt
  .long FSMC_interrupt
  .long SDIO_interrupt
  .long TIM5_interrupt
  .long SPI3_interrupt
  .long UART4_interrupt
  .long UART5_interrupt
  .long TIM6_DAC_interrupt
  .long TIM7_interrupt
  .long DMA2_Stream0_interrupt
  .long DMA2_Stream1_interrupt
  .long DMA2_Stream2_interrupt
  .long DMA2_Stream3_interrupt
  .long DMA2_Stream4_interrupt
  .long ETH_interrupt
  .long ETH_WKUP_interrupt
  .long CAN2_TX_interrupt
  .long CAN2_RX0_interrupt
  .long CAN2_RX1_interrupt
  .long CAN2_SCE_interrupt
  .long OTG_FS_interrupt
  .long DMA2_Stream5_interrupt
  .long DMA2_Stream6_interrupt
  .long DMA2_Stream7_interrupt
  .long USART6_interrupt
  .long I2C3_EV_interrupt
  .long I2C3_ER_interrupt
  .long OTG_HS_EP1_OUT_interrupt
  .long OTG_HS_EP1_IN_interrupt
  .long OTG_HS_WKUP_interrupt
  .long OTG_HS_interrupt
  .long DCMI_interrupt
  .long 0
  .long HASH_RNG_interrupt
  .long FPU_interrupt
  .weak NonMaskableInt_interrupt
  .weak MemoryManagement_interrupt
  .weak BusFault_interrupt
  .weak UsageFault_interrupt
  .weak SVCall_interrupt
  .weak DebugMonitor_interrupt
  .weak PendSV_interrupt
  .weak SysTick_interrupt
  .weak WWDG_interrupt
  .weak PVD_interrupt
  .weak TAMP_STAMP_interrupt
  .weak RTC_WKUP_interrupt
  .weak FLASH_interrupt
  .weak RCC_interrupt
  .weak EXTI0_interrupt
  .weak EXTI1_interrupt
  .weak EXTI2_interrupt
  .weak EXTI3_interrupt
  .weak EXTI4_interrupt
  .weak DMA1_Stream0_interrupt
  .weak DMA1_Stream1_interrupt
  .weak DMA1_Stream2_interrupt
  .weak DMA1_Stream3_interrupt
  .weak DMA1_Stream4_interrupt
  .weak DMA1_Stream5_interrupt
  .weak DMA1_Stream6_interrupt
  .weak ADC_interrupt
  .weak CAN1_TX_interrupt
  .weak CAN1_RX0_interrupt
  .weak CAN1_RX1_interrupt
  .weak CAN1_SCE_interrupt
  .weak EXTI9_5_interrupt
  .weak TIM1_BRK_TIM9_interrupt
  .weak TIM1_UP_TIM10_interrupt
  .weak TIM1_TRG_COM_TIM11_interrupt
  .weak TIM1_CC_interrupt
  .weak TIM2_interrupt
  .weak TIM3_interrupt
  .weak TIM4_interrupt
  .weak I2C1_EV_interrupt
  .weak I2C1_ER_interrupt
  .weak I2C2_EV_interrupt
  .weak I2C2_ER_interrupt
  .weak SPI1_interrupt
  .weak SPI2_interrupt
  .weak USART1_interrupt
  .weak USART2_interrupt
  .weak USART3_interrupt
  .weak EXTI15_10_interrupt
  .weak RTC_Alarm_interrupt
  .weak OTG_FS_WKUP_interrupt
  .weak TIM8_BRK_TIM12_interrupt
  .weak TIM8_UP_TIM13_interrupt
  .weak TIM8_TRG_COM_TIM14_interrupt
  .weak TIM8_CC_interrupt
  .weak DMA1_Stream7_interrupt
  .weak FSMC_interrupt
  .weak SDIO_interrupt
  .weak TIM5_interrupt
  .weak SPI3_interrupt
  .weak UART4_interrupt
  .weak UART5_interrupt
  .weak TIM6_DAC_interrupt
  .weak TIM7_interrupt
  .weak DMA2_Stream0_interrupt
  .weak DMA2_Stream1_interrupt
  .weak DMA2_Stream2_interrupt
  .weak DMA2_Stream3_interrupt
  .weak DMA2_Stream4_interrupt
  .weak ETH_interrupt
  .weak ETH_WKUP_interrupt
  .weak CAN2_TX_interrupt
  .weak CAN2_RX0_interrupt
  .weak CAN2_RX1_interrupt
  .weak CAN2_SCE_interrupt
  .weak OTG_FS_interrupt
  .weak DMA2_Stream5_interrupt
  .weak DMA2_Stream6_interrupt
  .weak DMA2_Stream7_interrupt
  .weak USART6_interrupt
  .weak I2C3_EV_interrupt
  .weak I2C3_ER_interrupt
  .weak OTG_HS_EP1_OUT_interrupt
  .weak OTG_HS_EP1_IN_interrupt
  .weak OTG_HS_WKUP_interrupt
  .weak OTG_HS_interrupt
  .weak DCMI_interrupt
  .weak HASH_RNG_interrupt
  .weak FPU_interrupt
  .set NonMaskableInt_interrupt, HaltProc
  .set MemoryManagement_interrupt, HaltProc
  .set BusFault_interrupt, HaltProc
  .set UsageFault_interrupt, HaltProc
  .set SVCall_interrupt, HaltProc
  .set DebugMonitor_interrupt, HaltProc
  .set PendSV_interrupt, HaltProc
  .set SysTick_interrupt, HaltProc
  .set WWDG_interrupt, HaltProc
  .set PVD_interrupt, HaltProc
  .set TAMP_STAMP_interrupt, HaltProc
  .set RTC_WKUP_interrupt, HaltProc
  .set FLASH_interrupt, HaltProc
  .set RCC_interrupt, HaltProc
  .set EXTI0_interrupt, HaltProc
  .set EXTI1_interrupt, HaltProc
  .set EXTI2_interrupt, HaltProc
  .set EXTI3_interrupt, HaltProc
  .set EXTI4_interrupt, HaltProc
  .set DMA1_Stream0_interrupt, HaltProc
  .set DMA1_Stream1_interrupt, HaltProc
  .set DMA1_Stream2_interrupt, HaltProc
  .set DMA1_Stream3_interrupt, HaltProc
  .set DMA1_Stream4_interrupt, HaltProc
  .set DMA1_Stream5_interrupt, HaltProc
  .set DMA1_Stream6_interrupt, HaltProc
  .set ADC_interrupt, HaltProc
  .set CAN1_TX_interrupt, HaltProc
  .set CAN1_RX0_interrupt, HaltProc
  .set CAN1_RX1_interrupt, HaltProc
  .set CAN1_SCE_interrupt, HaltProc
  .set EXTI9_5_interrupt, HaltProc
  .set TIM1_BRK_TIM9_interrupt, HaltProc
  .set TIM1_UP_TIM10_interrupt, HaltProc
  .set TIM1_TRG_COM_TIM11_interrupt, HaltProc
  .set TIM1_CC_interrupt, HaltProc
  .set TIM2_interrupt, HaltProc
  .set TIM3_interrupt, HaltProc
  .set TIM4_interrupt, HaltProc
  .set I2C1_EV_interrupt, HaltProc
  .set I2C1_ER_interrupt, HaltProc
  .set I2C2_EV_interrupt, HaltProc
  .set I2C2_ER_interrupt, HaltProc
  .set SPI1_interrupt, HaltProc
  .set SPI2_interrupt, HaltProc
  .set USART1_interrupt, HaltProc
  .set USART2_interrupt, HaltProc
  .set USART3_interrupt, HaltProc
  .set EXTI15_10_interrupt, HaltProc
  .set RTC_Alarm_interrupt, HaltProc
  .set OTG_FS_WKUP_interrupt, HaltProc
  .set TIM8_BRK_TIM12_interrupt, HaltProc
  .set TIM8_UP_TIM13_interrupt, HaltProc
  .set TIM8_TRG_COM_TIM14_interrupt, HaltProc
  .set TIM8_CC_interrupt, HaltProc
  .set DMA1_Stream7_interrupt, HaltProc
  .set FSMC_interrupt, HaltProc
  .set SDIO_interrupt, HaltProc
  .set TIM5_interrupt, HaltProc
  .set SPI3_interrupt, HaltProc
  .set UART4_interrupt, HaltProc
  .set UART5_interrupt, HaltProc
  .set TIM6_DAC_interrupt, HaltProc
  .set TIM7_interrupt, HaltProc
  .set DMA2_Stream0_interrupt, HaltProc
  .set DMA2_Stream1_interrupt, HaltProc
  .set DMA2_Stream2_interrupt, HaltProc
  .set DMA2_Stream3_interrupt, HaltProc
  .set DMA2_Stream4_interrupt, HaltProc
  .set ETH_interrupt, HaltProc
  .set ETH_WKUP_interrupt, HaltProc
  .set CAN2_TX_interrupt, HaltProc
  .set CAN2_RX0_interrupt, HaltProc
  .set CAN2_RX1_interrupt, HaltProc
  .set CAN2_SCE_interrupt, HaltProc
  .set OTG_FS_interrupt, HaltProc
  .set DMA2_Stream5_interrupt, HaltProc
  .set DMA2_Stream6_interrupt, HaltProc
  .set DMA2_Stream7_interrupt, HaltProc
  .set USART6_interrupt, HaltProc
  .set I2C3_EV_interrupt, HaltProc
  .set I2C3_ER_interrupt, HaltProc
  .set OTG_HS_EP1_OUT_interrupt, HaltProc
  .set OTG_HS_EP1_IN_interrupt, HaltProc
  .set OTG_HS_WKUP_interrupt, HaltProc
  .set OTG_HS_interrupt, HaltProc
  .set DCMI_interrupt, HaltProc
  .set HASH_RNG_interrupt, HaltProc
  .set FPU_interrupt, HaltProc
  .text
end;
end.
