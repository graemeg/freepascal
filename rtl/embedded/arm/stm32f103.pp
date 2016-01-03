{
Register definitions and utility code for STM32F103
Preliminary startup code - TODO: interrupt handler variables

Created by Jeppe Johansen 2009 - jepjoh2@kom.aau.dk
}
unit stm32f103;

{$goto on}
{$define stm32f103}

interface

type
 TBitvector32 = bitpacked array[0..31] of 0..1;

{$PACKRECORDS 2}
const
 PeripheralBase 	= $40000000;

 FSMCBase			= $60000000;

 APB1Base 			= PeripheralBase;
 APB2Base 			= PeripheralBase+$10000;
 AHBBase 			= PeripheralBase+$20000;

 SCS_BASE         = $E000E000;

 { FSMC }
 FSMCBank1NOR1		= FSMCBase+$00000000;
 FSMCBank1NOR2		= FSMCBase+$04000000;
 FSMCBank1NOR3		= FSMCBase+$08000000;
 FSMCBank1NOR4		= FSMCBase+$0C000000;

 FSMCBank1PSRAM1	= FSMCBase+$00000000;
 FSMCBank1PSRAM2	= FSMCBase+$04000000;
 FSMCBank1PSRAM3	= FSMCBase+$08000000;
 FSMCBank1PSRAM4	= FSMCBase+$0C000000;

 FSMCBank2NAND1	= FSMCBase+$10000000;
 FSMCBank3NAND2	= FSMCBase+$20000000;

 FSMCBank4PCCARD	= FSMCBase+$30000000;

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

 TFSMCRegisters = record
  nothingyet: byte;
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
  ICSR,                             {!< Interrupt Control State Register                         }
  VTOR,                             {!< Vector Table Offset Register                             }
  AIRCR,                            {!< Application Interrupt / Reset Control Register           }
  SCR,                              {!< System Control Register                                  }
  CCR: longword;                    {!< Configuration Control Register                           }
  SHP: array[0..11] of byte;        {!< System Handlers Priority Registers (4-7, 8-11, 12-15)    }
  SHCSR,                            {!< System Handler Control and State Register                }
  CFSR,                             {!< Configurable Fault Status Register                       }
  HFSR,                             {!< Hard Fault Status Register                               }
  DFSR,                             {!< Debug Fault Status Register                              }
  MMFAR,                            {!< Mem Manage Address Register                              }
  BFAR,                             {!< Bus Fault Address Register                               }
  AFSR: longword;                   {!< Auxiliary Fault Status Register                          }
  PFR: array[0..1] of longword;     {!< Processor Feature Register                               }
  DFR,                              {!< Debug Feature Register                                   }
  ADR: longword;                    {!< Auxiliary Feature Register                               }
  MMFR: array[0..3] of longword;    {!< Memory Model Feature Register                            }
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
 Timer1: TTimerRegisters 	absolute (APB2Base+$2C00);
 Timer2: TTimerRegisters 	absolute (APB1Base+$0000);
 Timer3: TTimerRegisters 	absolute (APB1Base+$0400);
 Timer4: TTimerRegisters 	absolute (APB1Base+$0800);
 Timer5: TTimerRegisters 	absolute (APB1Base+$0C00);
 Timer6: TTimerRegisters 	absolute (APB1Base+$1000);
 Timer7: TTimerRegisters 	absolute (APB1Base+$1400);
 Timer8: TTimerRegisters 	absolute (APB2Base+$3400);

 { RTC }
 RTC: TRTCRegisters 			absolute (APB1Base+$2800);

 { WDG }
 WWDG: TWWDGRegisters 		absolute (APB1Base+$2C00);
 IWDG: TIWDGRegisters 		absolute (APB1Base+$3000);

 { SPI }
 SPI1: TSPIRegisters			absolute (APB2Base+$3000);
 SPI2: TSPIRegisters			absolute (APB1Base+$3800);
 SPI3: TSPIRegisters			absolute (APB1Base+$3C00);

 { USART/UART }
 USART1: TUSARTRegisters	absolute (APB2Base+$3800);
 USART2: TUSARTRegisters	absolute (APB1Base+$4400);
 USART3: TUSARTRegisters	absolute (APB1Base+$4800);
 UART4: TUSARTRegisters		absolute (APB1Base+$4C00);
 UART5: TUSARTRegisters		absolute (APB1Base+$5000);

 { I2C }
 I2C1: TI2CRegisters			absolute (APB1Base+$5400);
 I2C2: TI2CRegisters			absolute (APB1Base+$5800);

 { USB }
 USB: TUSBRegisters			absolute (APB1Base+$5C00);
 USBMem: TUSBMem				absolute (APB1Base+$5C00);

 { CAN }
 CAN: TCANRegisters			absolute (APB1Base+$6800);

 { BKP }
 BKP: TBKPRegisters			absolute (APB1Base+$6C00);

 { PWR }
 PWR: TPwrRegisters			absolute (APB1Base+$7000);

 { DAC }
 DAC: TDACRegisters			absolute (APB1Base+$7400);

 { GPIO }
 AFIO: TAFIORegisters		absolute (APB2Base+$0);
 EXTI: TEXTIRegisters		absolute (APB2Base+$0400);

 PortA: TPortRegisters		absolute (APB2Base+$0800);
 PortB: TPortRegisters		absolute (APB2Base+$0C00);
 PortC: TPortRegisters		absolute (APB2Base+$1000);
 PortD: TPortRegisters		absolute (APB2Base+$1400);
 PortE: TPortRegisters		absolute (APB2Base+$1800);
 PortF: TPortRegisters		absolute (APB2Base+$1C00);
 PortG: TPortRegisters		absolute (APB2Base+$2000);

 { ADC }
 ADC1: TADCRegisters			absolute (APB2Base+$2400);
 ADC2: TADCRegisters			absolute (APB2Base+$2800);
 ADC3: TADCRegisters			absolute (APB2Base+$3C00);

 { SDIO }
 SDIO: TSDIORegisters		absolute (APB2Base+$8000);

 { DMA }
 DMA1: TDMARegisters			absolute (AHBBase+$0000);
 DMA2: TDMARegisters			absolute (AHBBase+$0400);

 { RCC }
 RCC: TRCCRegisters			absolute (AHBBase+$1000);

 { Flash }
 Flash: TFlashRegisters		absolute (AHBBase+$2000);

 { CRC }
 CRC: TCRCRegisters			absolute (AHBBase+$3000);

 { SCB }
 SCB: TSCBRegisters        absolute (SCS_BASE+$0D00);

 { SysTick }
 SysTick: TSysTickRegisters   absolute (SCS_BASE+$0010);

 { NVIC }
 NVIC: TNVICRegisters      absolute (SCS_BASE+$0100);

var
	NMI_Handler,
	HardFault_Handler,
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	SWI_Handler,
	DebugMonitor_Handler,
	PendingSV_Handler,
	Systick_Handler: pointer;

implementation

var
	_data: record end; external name '_data';
	_edata: record end; external name '_edata';
	_etext: record end; external name '_etext';
	_bss_start: record end; external name '_bss_start';
	_bss_end: record end; external name '_bss_end';
	_stack_top: record end; external name '_stack_top';

procedure PASCALMAIN; external name 'PASCALMAIN';

procedure _FPC_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhalt:
	b .Lhalt
end;

procedure _FPC_start; assembler; nostackframe;
label _start;
asm
	.init
	.balign 16
	
	.long _stack_top	 			// First entry in NVIC table is the new stack pointer
	.long _start+1
	//b   _start					// Reset
	.long _start+1
	//b	 .LNMI_Addr				// Non maskable interrupt. The RCC Clock Security System (CSS) is linked to the NMI vector.
	.long _start+1
	//b	 .LHardFault_Addr		// All class of fault
	.long _start+1
	//b	 .LMemManage_Addr		// Memory management
	.long _start+1
	//b	 .LBusFault_Addr		// Pre-fetch fault, memory access fault
	.long _start+1
	//b	 .LUsageFault_Addr	// Undefined instruction or illegal state
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//b	 .LSWI_Addr				// Software Interrupt vector
	.long _start+1
	//b	 .LDebugMonitor_Addr	// Debug Monitor
	.long _start+1
	//nop							// Reserved
	.long _start+1
	//b	 .LPendingSV_Addr		//	Pendable request for system service
	.long _start+1
	//b	 .LSystick_Addr		// System tick timer
	//17
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	//20
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1
	.long .LDefaultHandler+1

.LNMI_Addr:
	ldr r0,.L1
	ldr pc,[r0]
.LHardFault_Addr:
	ldr r0,.L2
	ldr pc,[r0]
.LMemManage_Addr:
	ldr r0,.L3
	ldr pc,[r0]
.LBusFault_Addr:
	ldr r0,.L4
	ldr pc,[r0]
.LUsageFault_Addr:
	ldr r0,.L5
	ldr pc,[r0]
.LSWI_Addr:
	ldr r0,.L6
	ldr pc,[r0]
.LDebugMonitor_Addr:
	ldr r0,.L7
	ldr pc,[r0]
.LPendingSV_Addr:
	ldr r0,.L8
	ldr pc,[r0]
.LSystick_Addr:
	ldr r0,.L9
	ldr pc,[r0]

.L1:
	.long NMI_Handler
.L2:
	.long HardFault_Handler
.L3:
	.long MemManage_Handler
.L4:
	.long BusFault_Handler
.L5:
	.long UsageFault_Handler
.L6:
	.long SWI_Handler
.L7:
	.long DebugMonitor_Handler
.L8:
	.long PendingSV_Handler
.L9:
	.long Systick_Handler

	.globl _start
	.text
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

	b PASCALMAIN
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
.LDefaultHandlerAddr:
	.long .LDefaultHandler
	// default irq handler just returns
.LDefaultHandler:
	mov pc,r14
end;

end.

