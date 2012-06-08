{
Register definitions and utility code for STM32F103
Preliminary startup code - TODO: interrupt handler variables

Created by Jeppe Johansen 2009 - jepjoh2@kom.aau.dk

Modified by Anton Rieckert 2012 - anton@riecktron.com
   -- Added interrupt (exception) support
}
unit stm32f103;

{$goto on}
{$define stm32f103}

interface

type
 TBitvector32 = bitpacked array[0..31] of 0..1;

{$PACKRECORDS 2}
const
 PeripheralBase  = $40000000;

 FSMCBase    = $60000000;
 
 APB1Base      = PeripheralBase;
 APB2Base      = PeripheralBase+$10000;
 AHBBase      = PeripheralBase+$20000;

 SCS_BASE        = $E000E000;

 { FSMC }
 FSMCBank1NOR1    = FSMCBase+$00000000;
 FSMCBank1NOR2    = FSMCBase+$04000000;
 FSMCBank1NOR3    = FSMCBase+$08000000;
 FSMCBank1NOR4    = FSMCBase+$0C000000;

 FSMCBank1PSRAM1  = FSMCBase+$00000000;
 FSMCBank1PSRAM2  = FSMCBase+$04000000;
 FSMCBank1PSRAM3  = FSMCBase+$08000000;
 FSMCBank1PSRAM4  = FSMCBase+$0C000000;

 FSMCBank2NAND1 = FSMCBase+$10000000;
 FSMCBank3NAND2 = FSMCBase+$20000000;

 FSMCBank4PCCARD  = FSMCBase+$30000000;

const
  NonMaskableInt_IRQn        = -14; //  2 Non Maskable // interrupt                            
  HardFault_IRQn              = -13;  //  4 Cortex-M3 Memory Management // interrupt              
  MemoryManagement_IRQn      = -12; //  4 Cortex-M3 Memory Management // interrupt              
  BusFault_IRQn              = -11; //  5 Cortex-M3 Bus Fault // interrupt                      
  UsageFault_IRQn            = -10; //  6 Cortex-M3 Usage Fault // interrupt                    
  SVCall_IRQn                = -5;   // 11 Cortex-M3 SV Call // interrupt                      
  DebugMonitor_IRQn          = -4;   // 12 Cortex-M3 Debug Monitor // interrupt                
  PendSV_IRQn                = -2;   // 14 Cortex-M3 Pend SV // interrupt                      
  SysTick_IRQn                = -1;  // 15 Cortex-M3 System Tick // interrupt                  
  WWDG_IRQn                  = 0;   //  Window WatchDog // interrupt                            
  PVD_IRQn                    = 1;    //  PVD through EXTI Line detection // interrupt            
  TAMPER_IRQn                = 2;   //  Tamper // interrupt                                    
  RTC_IRQn                    = 3;    //  RTC global // interrupt                                
  FLASH_IRQn                  = 4;    //  FLASH global // interrupt                              
  RCC_IRQn                    = 5;    //  RCC global // interrupt                                
  EXTI0_IRQn                  = 6;    //  EXTI Line0 // interrupt                                
  EXTI1_IRQn                  = 7;    //  EXTI Line1 // interrupt                                
  EXTI2_IRQn                  = 8;    //  EXTI Line2 // interrupt                                
  EXTI3_IRQn                  = 9;    //  EXTI Line3 // interrupt                                
  EXTI4_IRQn                  = 10;  // EXTI Line4 // interrupt                                
  DMA1_Channel1_IRQn          = 11;  // DMA1 Channel 1 global // interrupt                      
  DMA1_Channel2_IRQn          = 12;  // DMA1 Channel 2 global // interrupt                      
  DMA1_Channel3_IRQn          = 13;  // DMA1 Channel 3 global // interrupt                      
  DMA1_Channel4_IRQn          = 14;  // DMA1 Channel 4 global // interrupt                      
  DMA1_Channel5_IRQn          = 15;  // DMA1 Channel 5 global // interrupt                      
  DMA1_Channel6_IRQn          = 16;  // DMA1 Channel 6 global // interrupt                      
  DMA1_Channel7_IRQn          = 17;  // DMA1 Channel 7 global // interrupt                      
  ADC1_2_IRQn                = 18;   // ADC1 et ADC2 global // interrupt                        
  USB_HP_CAN1_TX_IRQn        = 19;   // USB High Priority or CAN1 TX Interrupts             
  USB_LP_CAN1_RX0_IRQn        = 20;  // USB Low Priority or CAN1 RX0 Interrupts             
  CAN1_RX1_IRQn              = 21;   // CAN1 RX1 // interrupt                                  
  CAN1_SCE_IRQn              = 22;   // CAN1 SCE // interrupt                                  
  EXTI9_5_IRQn                = 23;  // External Line[9:5] Interrupts                       
  TIM1_BRK_IRQn              = 24;   // TIM1 Break // interrupt                                
  TIM1_UP_IRQn                = 25;  // TIM1 Update // interrupt                                
  TIM1_TRG_COM_IRQn          = 26;   // TIM1 Trigger and Commutation // interrupt              
  TIM1_CC_IRQn                = 27;  // TIM1 Capture Compare // interrupt                      
  TIM2_IRQn                  = 28;   // TIM2 global // interrupt                                
  TIM3_IRQn                  = 29;   // TIM3 global // interrupt                                
  TIM4_IRQn                  = 30;   // TIM4 global // interrupt                                
  I2C1_EV_IRQn                = 31;  // I2C1 Event // interrupt                                
  I2C1_ER_IRQn                = 32;  // I2C1 Error // interrupt                                
  I2C2_EV_IRQn                = 33;  // I2C2 Event // interrupt                                
  I2C2_ER_IRQn                = 34;  // I2C2 Error // interrupt                                
  SPI1_IRQn                  = 35;   // SPI1 global // interrupt                                
  SPI2_IRQn                  = 36;   // SPI2 global // interrupt                                
  USART1_IRQn                = 37;   // USART1 global // interrupt                              
  USART2_IRQn                = 38;   // USART2 global // interrupt                              
  USART3_IRQn                = 39;   // USART3 global // interrupt                              
  EXTI15_10_IRQn              = 40;  // External Line[15:10] Interrupts                     
  RTCAlarm_IRQn              = 41;   // RTC Alarm through EXTI Line // interrupt                
  USBWakeUp_IRQn              = 42;  // USB WakeUp from suspend through EXTI Line // interrupt  
  TIM8_BRK_IRQn              = 43;   // TIM8 Break // interrupt                                
  TIM8_UP_IRQn                = 44;  // TIM8 Update // interrupt                                
  TIM8_TRG_COM_IRQn          = 45;   // TIM8 Trigger and Commutation // interrupt              
  TIM8_CC_IRQn                = 46;  // TIM8 Capture Compare // interrupt                      
  ADC3_IRQn                  = 47;   // ADC3 global // interrupt                                
  FSMC_IRQn                  = 48;   // FSMC global // interrupt                                
  SDIO_IRQn                  = 49;   // SDIO global // interrupt                                
  TIM5_IRQn                  = 50;   // TIM5 global // interrupt                                
  SPI3_IRQn                  = 51;   // SPI3 global // interrupt                                
  UART4_IRQn                  = 52;  // UART4 global // interrupt                              
  UART5_IRQn                  = 53;  // UART5 global // interrupt                              
  TIM6_IRQn                  = 54;   // TIM6 global // interrupt                                
  TIM7_IRQn                  = 55;   // TIM7 global // interrupt                                
  DMA2_Channel1_IRQn          = 56;  // DMA2 Channel 1 global // interrupt                      
  DMA2_Channel2_IRQn          = 57;  // DMA2 Channel 2 global // interrupt                      
  DMA2_Channel3_IRQn          = 58;  // DMA2 Channel 3 global // interrupt                      
  DMA2_Channel4_5_IRQn        = 59;  // DMA2 Channel 4 and Channel 5 global // interrupt        

type
 TTimerRegisters = record
  CR1, res1,
  CR2, res2,
  SMCR, res3,
  DIER, res4,
  SR, res5,
  EGR, res,
  CCMR1, res6,
  CCMR2, res7,
  CCER, res8,
  CNT, res9,
  PSC, res10,
  ARR, res11,
  RCR, res12,
  CCR1, res13,
  CCR2, res14,
  CCR3, res15,
  CCR4, res16,
  BDTR, res17,
  DCR, res18,
  DMAR, res19: Word;
 end;

 TRTCRegisters = record
  CRH, res1,
  CRL, res2,
  PRLH, res3,
  PRLL, res4,
  DIVH, res5,
  DIVL, res6,
  CNTH, res7,
  CNTL, res8,
  ALRH, res9,
  ALRL, res10: Word;
 end;

 TIWDGRegisters = record
  KR, res1,
  PR, res2,
  RLR, res3,
  SR, res4: word;
 end;

 TWWDGRegisters = record
  CR, res2,
  CFR, res3,
  SR, res4: word;
 end;

 TSPIRegisters = record
  CR1, res1,
  CR2, res2,
  SR, res3,
  DR, res4,
  CRCPR, res5,
  RXCRCR, res6,
  TXCRCR, res7,
  I2SCFGR, res8,
  I2SPR, res9: Word;
 end;

 TUSARTRegisters = record
  SR, res1,
  DR, res2,
  BRR, res3,
  CR1, res4,
  CR2, res5,
  CR3, res6,
  GTPR, res7: Word;
 end;

 TI2CRegisters = record
  CR1, res1,
  CR2, res2,
  OAR1, res3,
  OAR2, res4,
  DR, res5,
  SR1, res6,
  SR2, res7,
  CCR, res8: word;
  TRISE: byte;
 end;

 TUSBRegisters = record
  EPR: array[0..7] of DWord;

  res: array[0..7] of dword;

  CNTR, res1,
  ISTR, res2,
  FNR, res3: Word;
  DADDR: byte; res4: word; res5: byte;
  BTABLE: Word;
 end;

 TUSBMem = packed array[0..511] of byte;

 TCANMailbox = record
  IR,
  DTR,
  DLR,
  DHR: DWord;
 end;

 TCANRegisters = record
  MCR,
  MSR,
  TSR,
  RF0R,
  RF1R,
  IER,
  ESR,
  BTR: DWord;

  res5: array[$020..$17F] of byte;

  TX: array[0..2] of TCANMailbox;
  RX: array[0..2] of TCANMailbox;

  res6: array[$1D0..$1FF] of byte;

  FMR,
  FM1R,
  res9: DWord;
  FS1R, res10: word;
  res11: DWord;
  FFA1R, res12: word;
  res13: DWord;
  FA1R, res14: word;
  res15: array[$220..$23F] of byte;

  FOR1,
  FOR2: DWord;

  FB: array[1..13] of array[1..2] of DWord;
 end;

 TBKPRegisters = record
  DR: array[1..10] of record data, res: word; end;

  RTCCR,
  CR,
  CSR,
  res1,res2: DWord;

  DR2: array[11..42] of record data, res: word; end;
 end;

 TPwrRegisters = record
  CR, res: word;
  CSR: Word;
 end;

 TDACRegisters = record
  CR,
  SWTRIGR: DWord;

  DHR12R1, res2,
  DHR12L1, res3,
  DHR8R1, res4,
  DHR12R2, res5,
  DHR12L2, res6,
  DHR8R2, res7: word;

  DHR12RD,
  DHR12LD: DWord;

  DHR8RD, res8,

  DOR1, res9,
  DOR2, res10: Word;
 end;

 TAFIORegisters = record
  EVCR,
  MAPR: DWord;
  EXTICR: array[0..3] of DWord;
 end;

 TEXTIRegisters = record
  IMR,
  EMR,
  RTSR,
  FTSR,
  SWIER,
  PR: DWord;
 end;

 TPortRegisters = record
  CRL,
  CRH,
  IDR,
  ODR,
  BSRR,
  BRR,
  LCKR: DWord;
 end;

 TADCRegisters = record
  SR,
  CR1,
  CR2,
  SMPR1,
  SMPR2: DWord;
  JOFR1, res2,
  JOFR2, res3,
  JOFR3, res4,
  JOFR4, res5,
  HTR, res6,
  LTR, res7: word;
  SQR1,
  SQR2,
  SQR3,
  JSQR: DWord;
  JDR1, res8,
  JDR2, res9,
  JDR3, res10,
  JDR4, res11: Word;
  DR: DWord;
 end;

 TSDIORegisters = record
  POWER,
  CLKCR,
  ARG: DWord;
  CMD, res3,
  RESPCMD, res4: Word;
  RESP1,
  RESP2,
  RESP3,
  RESP4,
  DTIMER,
  DLEN: DWord;
  DCTRL, res5: word;
  DCOUNT,
  STA,
  ICR,
  MASK,
  FIFOCNT,
  FIFO: DWord;
 end;

 TDMAChannel = record
  CCR, res1,
  CNDTR, res2: word;
  CPAR,
  CMAR,
  res: DWord;
 end;

 TDMARegisters = record
  ISR,
  IFCR: DWord;
  Channel: array[0..7] of TDMAChannel;
 end;

 TRCCRegisters = record
  CR,
  CFGR,
  CIR,
  APB2RSTR,
  APB1RSTR,
  AHBENR,
  APB2ENR,
  APB1ENR,
  BDCR,
  CSR: DWord;
 end;

 TCRCRegisters = record
  DR: DWord;
  IDR: byte; res1: word; res2: byte;
  CR: byte;
 end;

  TFSMC_Bank1 = record
    BCR1 : dword;
    BTR1 : dword;
    BCR2 : dword;
    BTR2 : dword;
    BCR3 : dword;
    BTR3 : dword;
    BCR4 : dword;
    BTR4 : dword;
  end;

  TFSMC_Bank1E = record
    BWTR1 : dword;
    res1  : dword;
    BWTR2 : dword;
    res2  : dword;
    BWTR3 : dword;
    res3  : dword;
    BWTR4 : dword;
  end;

  TFSMC_Bank2 = record
    PCR2,
    SR2,
    PMEM2,
    PATT2,
    res1,
    ECCR2 : dword
  end;

  TFSMC_Bank3 = record
    PCR3,
    SR3,
    PMEM3,
    PATT3,
    RESERVED0,
    ECCR3 : dword;
  end;

  TFSMC_Bank4 = record
    PCR4,
    SR4,
    PMEM4,
    PATT4,
    PIO4 : dword;
  end;

 TFlashRegisters = record
  ACR,
  KEYR,
  OPTKEYR,
  SR,
  CR,
  AR,
  res,
  OBR,
  WRPR: DWord;
 end;

 TNVICRegisters = packed record
  ISER: array[0..7] of longword;
   reserved0: array[0..23] of longword;
  ICER: array[0..7] of longword;
   reserved1: array[0..23] of longword;
  ISPR: array[0..7] of longword;
   reserved2: array[0..23] of longword;
  ICPR: array[0..7] of longword;
   reserved3: array[0..23] of longword;
  IABR: array[0..7] of longword;
   reserved4: array[0..55] of longword;
  IP: array[0..239] of longword;
   reserved5: array[0..643] of longword;
  STIR: longword;
 end;

 TSCBRegisters = packed record
  CPUID,                            {!< CPU ID Base Register                                     }
  ICSR,                            {!< Interrupt Control State Register                        }
  VTOR,                            {!< Vector Table Offset Register                            }
  AIRCR,                            {!< Application Interrupt / Reset Control Register           }
  SCR,                              {!< System Control Register                                 }
  CCR: longword;                    {!< Configuration Control Register                           }
  SHP: array[0..11] of byte;        {!< System Handlers Priority Registers (4-7, 8-11, 12-15)   }
  SHCSR,                            {!< System Handler Control and State Register               }
  CFSR,                            {!< Configurable Fault Status Register                      }
  HFSR,                            {!< Hard Fault Status Register                              }
  DFSR,                            {!< Debug Fault Status Register                              }
  MMFAR,                            {!< Mem Manage Address Register                             }
  BFAR,                            {!< Bus Fault Address Register                              }
  AFSR: longword;                  {!< Auxiliary Fault Status Register                          }
  PFR: array[0..1] of longword;    {!< Processor Feature Register                              }
  DFR,                              {!< Debug Feature Register                                   }
  ADR: longword;                    {!< Auxiliary Feature Register                               }
  MMFR: array[0..3] of longword;    {!< Memory Model Feature Register                           }
  ISAR: array[0..4] of longword;    {!< ISA Feature Register                                     }
 end;

 TSysTickRegisters = packed record
  Ctrl,
  Load,
  Val,
  Calib: longword;
 end;

{$ALIGN 2}
var
 { Timers }
 Timer1: TTimerRegisters  absolute (APB2Base+$2C00);
 Timer2: TTimerRegisters  absolute (APB1Base+$0000);
 Timer3: TTimerRegisters  absolute (APB1Base+$0400);
 Timer4: TTimerRegisters  absolute (APB1Base+$0800);
 Timer5: TTimerRegisters  absolute (APB1Base+$0C00);
 Timer6: TTimerRegisters  absolute (APB1Base+$1000);
 Timer7: TTimerRegisters  absolute (APB1Base+$1400);
 Timer8: TTimerRegisters  absolute (APB2Base+$3400);

 { RTC }
 RTC: TRTCRegisters      absolute (APB1Base+$2800);

 { WDG }
 WWDG: TWWDGRegisters    absolute (APB1Base+$2C00);
 IWDG: TIWDGRegisters    absolute (APB1Base+$3000);

 { SPI }
 SPI1: TSPIRegisters      absolute (APB2Base+$3000);
 SPI2: TSPIRegisters      absolute (APB1Base+$3800);
 SPI3: TSPIRegisters      absolute (APB1Base+$3C00);

 { USART/UART }
 USART1: TUSARTRegisters  absolute (APB2Base+$3800);
 USART2: TUSARTRegisters  absolute (APB1Base+$4400);
 USART3: TUSARTRegisters  absolute (APB1Base+$4800);
 UART4: TUSARTRegisters  absolute (APB1Base+$4C00);
 UART5: TUSARTRegisters  absolute (APB1Base+$5000);

 { I2C }
 I2C1: TI2CRegisters      absolute (APB1Base+$5400);
 I2C2: TI2CRegisters      absolute (APB1Base+$5800);

 { USB }
 USB: TUSBRegisters    absolute (APB1Base+$5C00);
 USBMem: TUSBMem                        absolute (APB1Base+$6000);

 { CAN }
 CAN: TCANRegisters    absolute (APB1Base+$6800);

 { BKP }
 BKP: TBKPRegisters    absolute (APB1Base+$6C00);

 { PWR }
 PWR: TPwrRegisters    absolute (APB1Base+$7000);

 { DAC }
 DAC: TDACRegisters    absolute (APB1Base+$7400);

 { GPIO }
 AFIO: TAFIORegisters  absolute (APB2Base+$0);
 EXTI: TEXTIRegisters  absolute (APB2Base+$0400);

 PortA: TPortRegisters    absolute (APB2Base+$0800);
 PortB: TPortRegisters    absolute (APB2Base+$0C00);
 PortC: TPortRegisters    absolute (APB2Base+$1000);
 PortD: TPortRegisters    absolute (APB2Base+$1400);
 PortE: TPortRegisters    absolute (APB2Base+$1800);
 PortF: TPortRegisters    absolute (APB2Base+$1C00);
 PortG: TPortRegisters    absolute (APB2Base+$2000);

 { ADC }
 ADC1: TADCRegisters      absolute (APB2Base+$2400);
 ADC2: TADCRegisters      absolute (APB2Base+$2800);
 ADC3: TADCRegisters      absolute (APB2Base+$3C00);

 { SDIO }
 SDIO: TSDIORegisters  absolute (APB2Base+$8000);

 { DMA }
 DMA1: TDMARegisters      absolute (AHBBase+$0000);
 DMA2: TDMARegisters      absolute (AHBBase+$0400);

 { RCC }
 RCC: TRCCRegisters    absolute (AHBBase+$1000);

 { Flash }
 Flash: TFlashRegisters  absolute (AHBBase+$2000);

 { FSMC }
 FSMC_Bank1 : TFSMC_Bank1 absolute (FSMCBase + $40000000);  //     ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
 FSMC_Bank1E : TFSMC_Bank1E absolute (FSMCBase + $40000104);  //     ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
 FSMC_Bank2 : TFSMC_Bank2 absolute (FSMCBase + $40000060);  //     ((FSMC_Bank2_TypeDef *) FSMC_Bank2_R_BASE)
 FSMC_Bank3 : TFSMC_Bank3 absolute (FSMCBase + $40000080);  //     ((FSMC_Bank3_TypeDef *) FSMC_Bank3_R_BASE)
 FSMC_Bank4 : TFSMC_Bank4 absolute (FSMCBase + $400000A0);  //     ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)

 { CRC }
 CRC: TCRCRegisters    absolute (AHBBase+$3000);

 { SCB }
 SCB: TSCBRegisters       absolute (SCS_BASE+$0D00);

 { SysTick }
 SysTick: TSysTickRegisters  absolute (SCS_BASE+$0010);

 { NVIC }
 NVIC: TNVICRegisters     absolute (SCS_BASE+$0100);

implementation

var
  _data: record end; external name '_data';
  _edata: record end; external name '_edata';
  _etext: record end; external name '_etext';
  _bss_start: record end; external name '_bss_start';
  _bss_end: record end; external name '_bss_end';
  _stack_top: record end; external name '_stack_top';

procedure PASCALMAIN; external name 'PASCALMAIN';

procedure DefaultHandler; public name 'DefaultHandler';
begin

end;

procedure _FPC_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhalt:
  b .Lhalt
end;

procedure _FPC_start; assembler; nostackframe; interrupt 0;
label _start;
asm
  .globl _start
_start:
  
  // Copy initialized data to ram
  ldr r1,.L_etext
  ldr r2,.L_data
  ldr r3,.L_edata
.Lcopyloop:
  cmp r2,r3
  ittt ls
  ldrls r0,[r1],#4
  strls r0,[r2],#4
  bls .Lcopyloop

  // clear onboard ram
  ldr r1,.L_bss_start
  ldr r2,.L_bss_end
  mov r0,#0
.Lzeroloop:
  cmp r1,r2
  itt ls
  strls r0,[r1],#4
  bls .Lzeroloop

  bl PASCALMAIN
  b _FPC_haltproc

.L_bss_start:
  .long _bss_start
.L_bss_end:
  .long _bss_end
.L_etext:
  .long _etext
.L_data:
  .long _data
.L_edata:
  .long _edata
end;

end.

