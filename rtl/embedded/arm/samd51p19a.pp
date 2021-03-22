unit samd51p19a;
(*
  Copyright (c) 2020 Microchip Technology Inc.
                   
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the Licence at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*)

interface
{$PACKRECORDS C}
{$GOTO ON}
{$SCOPEDENUMS ON}
{$DEFINE INTERFACE}
{$UNDEF IMPLEMENTATION}
{$DEFINE __CORTEXM4}
{$DEFINE __NVIC_PRIO_BITS3 }

const
  __FPU_PRESENT=1;
  __MPU_PRESENT=1;
  __NVIC_PRIO_BITS=3;

type
  TIRQn_Enum = (
    NonMaskableInt_IRQn       = -14,
    HardFault_IRQn            = -13,
    MemoryManagement_IRQn     = -12,
    BusFault_IRQn             = -11,
    UsageFault_IRQn           = -10,
    SVCall_IRQn               = -5,
    DebugMonitor_IRQn         = -4,
    PendSV_IRQn               = -2,
    SysTick_IRQn              = -1,
    PM_IRQn                   = 0,
    MCLK_IRQn                 = 1,
    OSCCTRL_0_IRQn            = 2,
    OSCCTRL_1_IRQn            = 3,
    OSCCTRL_2_IRQn            = 4,
    OSCCTRL_3_IRQn            = 5,
    OSCCTRL_4_IRQn            = 6,
    OSC32KCTRL_IRQn           = 7,
    SUPC_0_IRQn               = 8,
    SUPC_1_IRQn               = 9,
    WDT_IRQn                  = 10,
    RTC_IRQn                  = 11,
    EIC_0_IRQn                = 12,
    EIC_1_IRQn                = 13,
    EIC_2_IRQn                = 14,
    EIC_3_IRQn                = 15,
    EIC_4_IRQn                = 16,
    EIC_5_IRQn                = 17,
    EIC_6_IRQn                = 18,
    EIC_7_IRQn                = 19,
    EIC_8_IRQn                = 20,
    EIC_9_IRQn                = 21,
    EIC_10_IRQn               = 22,
    EIC_11_IRQn               = 23,
    EIC_12_IRQn               = 24,
    EIC_13_IRQn               = 25,
    EIC_14_IRQn               = 26,
    EIC_15_IRQn               = 27,
    FREQM_IRQn                = 28,
    NVMCTRL_0_IRQn            = 29,
    NVMCTRL_1_IRQn            = 30,
    DMAC_0_IRQn               = 31,
    DMAC_1_IRQn               = 32,
    DMAC_2_IRQn               = 33,
    DMAC_3_IRQn               = 34,
    DMAC_4_IRQn               = 35,
    EVSYS_0_IRQn              = 36,
    EVSYS_1_IRQn              = 37,
    EVSYS_2_IRQn              = 38,
    EVSYS_3_IRQn              = 39,
    EVSYS_4_IRQn              = 40,
    PAC_IRQn                  = 41,
    RAMECC_IRQn               = 45,
    SERCOM0_0_IRQn            = 46,
    SERCOM0_1_IRQn            = 47,
    SERCOM0_2_IRQn            = 48,
    SERCOM0_3_IRQn            = 49,
    SERCOM1_0_IRQn            = 50,
    SERCOM1_1_IRQn            = 51,
    SERCOM1_2_IRQn            = 52,
    SERCOM1_3_IRQn            = 53,
    SERCOM2_0_IRQn            = 54,
    SERCOM2_1_IRQn            = 55,
    SERCOM2_2_IRQn            = 56,
    SERCOM2_3_IRQn            = 57,
    SERCOM3_0_IRQn            = 58,
    SERCOM3_1_IRQn            = 59,
    SERCOM3_2_IRQn            = 60,
    SERCOM3_3_IRQn            = 61,
    SERCOM4_0_IRQn            = 62,
    SERCOM4_1_IRQn            = 63,
    SERCOM4_2_IRQn            = 64,
    SERCOM4_3_IRQn            = 65,
    SERCOM5_0_IRQn            = 66,
    SERCOM5_1_IRQn            = 67,
    SERCOM5_2_IRQn            = 68,
    SERCOM5_3_IRQn            = 69,
    SERCOM6_0_IRQn            = 70,
    SERCOM6_1_IRQn            = 71,
    SERCOM6_2_IRQn            = 72,
    SERCOM6_3_IRQn            = 73,
    SERCOM7_0_IRQn            = 74,
    SERCOM7_1_IRQn            = 75,
    SERCOM7_2_IRQn            = 76,
    SERCOM7_3_IRQn            = 77,
    USB_0_IRQn                = 80,
    USB_1_IRQn                = 81,
    USB_2_IRQn                = 82,
    USB_3_IRQn                = 83,
    TCC0_0_IRQn               = 85,
    TCC0_1_IRQn               = 86,
    TCC0_2_IRQn               = 87,
    TCC0_3_IRQn               = 88,
    TCC0_4_IRQn               = 89,
    TCC0_5_IRQn               = 90,
    TCC0_6_IRQn               = 91,
    TCC1_0_IRQn               = 92,
    TCC1_1_IRQn               = 93,
    TCC1_2_IRQn               = 94,
    TCC1_3_IRQn               = 95,
    TCC1_4_IRQn               = 96,
    TCC2_0_IRQn               = 97,
    TCC2_1_IRQn               = 98,
    TCC2_2_IRQn               = 99,
    TCC2_3_IRQn               = 100,
    TCC3_0_IRQn               = 101,
    TCC3_1_IRQn               = 102,
    TCC3_2_IRQn               = 103,
    TCC4_0_IRQn               = 104,
    TCC4_1_IRQn               = 105,
    TCC4_2_IRQn               = 106,
    TC0_IRQn                  = 107,
    TC1_IRQn                  = 108,
    TC2_IRQn                  = 109,
    TC3_IRQn                  = 110,
    TC4_IRQn                  = 111,
    TC5_IRQn                  = 112,
    TC6_IRQn                  = 113,
    TC7_IRQn                  = 114,
    PDEC_0_IRQn               = 115,
    PDEC_1_IRQn               = 116,
    PDEC_2_IRQn               = 117,
    ADC0_0_IRQn               = 118,
    ADC0_1_IRQn               = 119,
    ADC1_0_IRQn               = 120,
    ADC1_1_IRQn               = 121,
    AC_IRQn                   = 122,
    DAC_0_IRQn                = 123,
    DAC_1_IRQn                = 124,
    DAC_2_IRQn                = 125,
    DAC_3_IRQn                = 126,
    DAC_4_IRQn                = 127,
    I2S_IRQn                  = 128,
    PCC_IRQn                  = 129,
    AES_IRQn                  = 130,
    TRNG_IRQn                 = 131,
    ICM_IRQn                  = 132,
    PUKCC_IRQn                = 133,
    QSPI_IRQn                 = 134,
    SDHC0_IRQn                = 135,
    SDHC1_IRQn                = 136
  );

  TAC_Registers = record
    CTRLA        : byte;                 //0000 Control A
    CTRLB        : byte;                 //0001 Control B
    EVCTRL       : word;                 //0002 Event Control
    INTENCLR     : byte;                 //0004 Interrupt Enable Clear
    INTENSET     : byte;                 //0005 Interrupt Enable Set
    INTFLAG      : byte;                 //0006 Interrupt Flag Status and Clear
    STATUSA      : byte;                 //0007 Status A
    STATUSB      : byte;                 //0008 Status B
    DBGCTRL      : byte;                 //0009 Debug Control
    WINCTRL      : byte;                 //000A Window Control
    RESERVED0    : byte;
    SCALER       : array[0..1] of byte;  //000C Scaler n
    RESERVED1    : word;
    COMPCTRL     : array[0..1] of longWord; //0010 Comparator Control n
    RESERVED2    : array[1..8] of byte;
    SYNCBUSY     : longWord;             //0020 Synchronization Busy
    CALIB        : word;                 //0024 Calibration
  end;

  TADC_Registers = record
    CTRLA        : word;                 //0000 Control A
    EVCTRL       : byte;                 //0002 Event Control
    DBGCTRL      : byte;                 //0003 Debug Control
    INPUTCTRL    : word;                 //0004 Input Control
    CTRLB        : word;                 //0006 Control B
    REFCTRL      : byte;                 //0008 Reference Control
    RESERVED0    : byte;
    AVGCTRL      : byte;                 //000A Average Control
    SAMPCTRL     : byte;                 //000B Sample Time Control
    WINLT        : word;                 //000C Window Monitor Lower Threshold
    WINUT        : word;                 //000E Window Monitor Upper Threshold
    GAINCORR     : word;                 //0010 Gain Correction
    OFFSETCORR   : word;                 //0012 Offset Correction
    SWTRIG       : byte;                 //0014 Software Trigger
    RESERVED1    : array[1..23] of byte;
    INTENCLR     : byte;                 //002C Interrupt Enable Clear
    INTENSET     : byte;                 //002D Interrupt Enable Set
    INTFLAG      : byte;                 //002E Interrupt Flag Status and Clear
    STATUS       : byte;                 //002F Status
    SYNCBUSY     : longWord;             //0030 Synchronization Busy
    DSEQDATA     : longWord;             //0034 DMA Sequencial Data
    DSEQCTRL     : longWord;             //0038 DMA Sequential Control
    DSEQSTAT     : longWord;             //003C DMA Sequencial Status
    RESULT       : word;                 //0040 Result Conversion Value
    RESERVED2    : word;
    RESS         : word;                 //0044 Last Sample Result
    RESERVED3    : word;
    CALIB        : word;                 //0048 Calibration
  end;

  TAES_Registers = record
    CTRLA        : longWord;             //0000 Control A
    CTRLB        : byte;                 //0004 Control B
    INTENCLR     : byte;                 //0005 Interrupt Enable Clear
    INTENSET     : byte;                 //0006 Interrupt Enable Set
    INTFLAG      : byte;                 //0007 Interrupt Flag Status
    DATABUFPTR   : byte;                 //0008 Data buffer pointer
    DBGCTRL      : byte;                 //0009 Debug control
    RESERVED0    : word;
    KEYWORD      : array[0..7] of longWord; //000C Keyword n
    RESERVED1    : array[1..12] of byte;
    INDATA       : longWord;             //0038 Indata
    INTVECTV     : array[0..3] of longWord; //003C Initialisation Vector n
    RESERVED2    : array[1..16] of byte;
    HASHKEY      : array[0..3] of longWord; //005C Hash key n
    GHASH        : array[0..3] of longWord; //006C Galois Hash n
    RESERVED3    : longWord;
    CIPLEN       : longWord;             //0080 Cipher Length
    RANDSEED     : longWord;             //0084 Random Seed
  end;

  TCCL_Registers = record
    CTRL         : byte;                 //0000 Control
    RESERVED0    : array[1..3] of byte;
    SEQCTRL      : array[0..1] of byte;  //0004 SEQ Control x
    RESERVED1    : word;
    LUTCTRL      : array[0..3] of longWord; //0008 LUT Control x
  end;

  TCMCC_Registers = record
    &TYPE        : longWord;             //0000 Cache Type Register
    CFG          : longWord;             //0004 Cache Configuration Register
    CTRL         : longWord;             //0008 Cache Control Register
    SR           : longWord;             //000C Cache Status Register
    LCKWAY       : longWord;             //0010 Cache Lock per Way Register
    RESERVED0    : array[1..12] of byte;
    MAINT0       : longWord;             //0020 Cache Maintenance Register 0
    MAINT1       : longWord;             //0024 Cache Maintenance Register 1
    MCFG         : longWord;             //0028 Cache Monitor Configuration Register
    MEN          : longWord;             //002C Cache Monitor Enable Register
    MCTRL        : longWord;             //0030 Cache Monitor Control Register
    MSR          : longWord;             //0034 Cache Monitor Status Register
  end;

  TDAC_Registers = record
    CTRLA        : byte;                 //0000 Control A
    CTRLB        : byte;                 //0001 Control B
    EVCTRL       : byte;                 //0002 Event Control
    RESERVED0    : byte;
    INTENCLR     : byte;                 //0004 Interrupt Enable Clear
    INTENSET     : byte;                 //0005 Interrupt Enable Set
    INTFLAG      : byte;                 //0006 Interrupt Flag Status and Clear
    STATUS       : byte;                 //0007 Status
    SYNCBUSY     : longWord;             //0008 Synchronization Busy
    DACCTRL      : array[0..1] of word;  //000C DAC n Control
    DATA         : array[0..1] of word;  //0010 DAC n Data
    DATABUF      : array[0..1] of word;  //0014 DAC n Data Buffer
    DBGCTRL      : byte;                 //0018 Debug Control
    RESERVED1    : array[1..3] of byte;
    RESULT       : array[0..1] of word;  //001C Filter Result
  end;

  TDMAC_CHANNEL_Registers = record
    CHCTRLA      : longWord;             //0000 Channel n Control A
    CHCTRLB      : byte;                 //0004 Channel n Control B
    CHPRILVL     : byte;                 //0005 Channel n Priority Level
    CHEVCTRL     : byte;                 //0006 Channel n Event Control
    RESERVED0    : array[1..5] of byte;
    CHINTENCLR   : byte;                 //000C Channel n Interrupt Enable Clear
    CHINTENSET   : byte;                 //000D Channel n Interrupt Enable Set
    CHINTFLAG    : byte;                 //000E Channel n Interrupt Flag Status and Clear
    CHSTATUS     : byte;                 //000F Channel n Status
  end;

  TDMAC_Registers = record
    CTRL         : word;                 //0000 Control
    CRCCTRL      : word;                 //0002 CRC Control
    CRCDATAIN    : longWord;             //0004 CRC Data Input
    CRCCHKSUM    : longWord;             //0008 CRC Checksum
    CRCSTATUS    : byte;                 //000C CRC Status
    DBGCTRL      : byte;                 //000D Debug Control
    RESERVED0    : word;
    SWTRIGCTRL   : longWord;             //0010 Software Trigger Control
    PRICTRL0     : longWord;             //0014 Priority Control 0
    RESERVED1    : array[1..8] of byte;
    INTPEND      : word;                 //0020 Interrupt Pending
    RESERVED2    : word;
    INTSTATUS    : longWord;             //0024 Interrupt Status
    BUSYCH       : longWord;             //0028 Busy Channels
    PENDCH       : longWord;             //002C Pending Channels
    ACTIVE       : longWord;             //0030 Active Channel and Levels
    BASEADDR     : longWord;             //0034 Descriptor Memory Section Base Address
    WRBADDR      : longWord;             //0038 Write-Back Memory Section Base Address
    RESERVED3    : longWord;
    CHANNEL      : array[0..31] of TDMAC_CHANNEL_Registers;  //0040 
  end;

  TDMAC_DESCRIPTOR_Registers = record
    BTCTRL       : word;                 //0000 Block Transfer Control
    BTCNT        : word;                 //0002 Block Transfer Count
    SRCADDR      : longWord;             //0004 Block Transfer Source Address
    DSTADDR      : longWord;             //0008 Block Transfer Destination Address
    DESCADDR     : longWord;             //000C Next Descriptor Address
  end;

  TDSU_Registers = record
    CTRL         : byte;                 //0000 Control
    STATUSA      : byte;                 //0001 Status A
    STATUSB      : byte;                 //0002 Status B
    RESERVED0    : byte;
    ADDR         : longWord;             //0004 Address
    LENGTH       : longWord;             //0008 Length
    DATA         : longWord;             //000C Data
    DCC          : array[0..1] of longWord; //0010 Debug Communication Channel n
    DID          : longWord;             //0018 Device Identification
    CFG          : longWord;             //001C Configuration
    RESERVED1    : array[1..208] of byte;
    DCFG         : array[0..1] of longWord; //00F0 Device Configuration
    RESERVED2    : array[1..3848] of byte;
    ENTRY0       : longWord;             //1000 CoreSight ROM Table Entry 0
    ENTRY1       : longWord;             //1004 CoreSight ROM Table Entry 1
    &END         : longWord;             //1008 CoreSight ROM Table End
    RESERVED3    : array[1..4032] of byte;
    MEMTYPE      : longWord;             //1FCC CoreSight ROM Table Memory Type
    PID4         : longWord;             //1FD0 Peripheral Identification 4
    PID5         : longWord;             //1FD4 Peripheral Identification 5
    PID6         : longWord;             //1FD8 Peripheral Identification 6
    PID7         : longWord;             //1FDC Peripheral Identification 7
    PID0         : longWord;             //1FE0 Peripheral Identification 0
    PID1         : longWord;             //1FE4 Peripheral Identification 1
    PID2         : longWord;             //1FE8 Peripheral Identification 2
    PID3         : longWord;             //1FEC Peripheral Identification 3
    CID0         : longWord;             //1FF0 Component Identification 0
    CID1         : longWord;             //1FF4 Component Identification 1
    CID2         : longWord;             //1FF8 Component Identification 2
    CID3         : longWord;             //1FFC Component Identification 3
  end;

  TEIC_Registers = record
    CTRLA        : byte;                 //0000 Control A
    NMICTRL      : byte;                 //0001 Non-Maskable Interrupt Control
    NMIFLAG      : word;                 //0002 Non-Maskable Interrupt Flag Status and Clear
    SYNCBUSY     : longWord;             //0004 Synchronization Busy
    EVCTRL       : longWord;             //0008 Event Control
    INTENCLR     : longWord;             //000C Interrupt Enable Clear
    INTENSET     : longWord;             //0010 Interrupt Enable Set
    INTFLAG      : longWord;             //0014 Interrupt Flag Status and Clear
    ASYNCH       : longWord;             //0018 External Interrupt Asynchronous Mode
    CONFIG       : array[0..1] of longWord; //001C External Interrupt Sense Configuration
    RESERVED0    : array[1..12] of byte;
    DEBOUNCEN    : longWord;             //0030 Debouncer Enable
    DPRESCALER   : longWord;             //0034 Debouncer Prescaler
    PINSTATE     : longWord;             //0038 Pin State
  end;

  TEVSYS_CHANNEL_Registers = record
    CHANNEL      : longWord;             //0000 Channel n Control
    CHINTENCLR   : byte;                 //0004 Channel n Interrupt Enable Clear
    CHINTENSET   : byte;                 //0005 Channel n Interrupt Enable Set
    CHINTFLAG    : byte;                 //0006 Channel n Interrupt Flag Status and Clear
    CHSTATUS     : byte;                 //0007 Channel n Status
  end;

  TEVSYS_Registers = record
    CTRLA        : byte;                 //0000 Control
    RESERVED0    : array[1..3] of byte;
    SWEVT        : longWord;             //0004 Software Event
    PRICTRL      : byte;                 //0008 Priority Control
    RESERVED1    : array[1..7] of byte;
    INTPEND      : word;                 //0010 Channel Pending Interrupt
    RESERVED2    : word;
    INTSTATUS    : longWord;             //0014 Interrupt Status
    BUSYCH       : longWord;             //0018 Busy Channels
    READYUSR     : longWord;             //001C Ready Users
    CHANNEL      : array[0..31] of TEVSYS_CHANNEL_Registers;  //0020 
    USER         : array[0..66] of longWord; //0120 User Multiplexer n
  end;

  TFREQM_Registers = record
    CTRLA        : byte;                 //0000 Control A Register
    CTRLB        : byte;                 //0001 Control B Register
    CFGA         : word;                 //0002 Config A register
    RESERVED0    : longWord;
    INTENCLR     : byte;                 //0008 Interrupt Enable Clear Register
    INTENSET     : byte;                 //0009 Interrupt Enable Set Register
    INTFLAG      : byte;                 //000A Interrupt Flag Register
    STATUS       : byte;                 //000B Status Register
    SYNCBUSY     : longWord;             //000C Synchronization Busy Register
    VALUE        : longWord;             //0010 Count Value Register
  end;

  TGCLK_Registers = record
    CTRLA        : byte;                 //0000 Control
    RESERVED0    : array[1..3] of byte;
    SYNCBUSY     : longWord;             //0004 Synchronization Busy
    RESERVED1    : array[1..24] of byte;
    GENCTRL      : array[0..11] of longWord; //0020 Generic Clock Generator Control
    RESERVED2    : array[1..48] of byte;
    PCHCTRL      : array[0..47] of longWord; //0080 Peripheral Clock Control
  end;

  THMATRIXB_PRS_Registers = record
    PRAS         : longWord;             //0000 Priority A for Slave
    PRBS         : longWord;             //0004 Priority B for Slave
  end;

  THMATRIXB_Registers = record
    PRS          : array[0..15] of THMATRIXB_PRS_Registers;  //0080 
  end;

  TICM_Registers = record
    CFG          : longWord;             //0000 Configuration
    CTRL         : longWord;             //0004 Control
    SR           : longWord;             //0008 Status
    RESERVED0    : longWord;
    IER          : longWord;             //0010 Interrupt Enable
    IDR          : longWord;             //0014 Interrupt Disable
    IMR          : longWord;             //0018 Interrupt Mask
    ISR          : longWord;             //001C Interrupt Status
    UASR         : longWord;             //0020 Undefined Access Status
    RESERVED1    : array[1..12] of byte;
    DSCR         : longWord;             //0030 Region Descriptor Area Start Address
    HASH         : longWord;             //0034 Region Hash Area Start Address
    UIHVAL       : array[0..7] of longWord; //0038 User Initial Hash Value n
  end;

  TICM_DESCRIPTOR_Registers = record
    RADDR        : longWord;             //0000 Region Start Address
    RCFG         : longWord;             //0004 Region Configuration
    RCTRL        : longWord;             //0008 Region Control
    RNEXT        : longWord;             //000C Region Next Address
  end;

  TI2S_Registers = record
    CTRLA        : byte;                 //0000 Control A
    RESERVED0    : array[1..3] of byte;
    CLKCTRL      : array[0..1] of longWord; //0004 Clock Unit n Control
    INTENCLR     : word;                 //000C Interrupt Enable Clear
    RESERVED1    : word;
    INTENSET     : word;                 //0010 Interrupt Enable Set
    RESERVED2    : word;
    INTFLAG      : word;                 //0014 Interrupt Flag Status and Clear
    RESERVED3    : word;
    SYNCBUSY     : word;                 //0018 Synchronization Status
    RESERVED4    : array[1..6] of byte;
    TXCTRL       : longWord;             //0020 Tx Serializer Control
    RXCTRL       : longWord;             //0024 Rx Serializer Control
    RESERVED5    : array[1..8] of byte;
    TXDATA       : longWord;             //0030 Tx Data
    RXDATA       : longWord;             //0034 Rx Data
  end;

  TMCLK_Registers = record
    INTENCLR     : byte;                 //0001 Interrupt Enable Clear
    INTENSET     : byte;                 //0002 Interrupt Enable Set
    INTFLAG      : byte;                 //0003 Interrupt Flag Status and Clear
    HSDIV        : byte;                 //0004 HS Clock Division
    CPUDIV       : byte;                 //0005 CPU Clock Division
    RESERVED0    : array[1..10] of byte;
    AHBMASK      : longWord;             //0010 AHB Mask
    APBAMASK     : longWord;             //0014 APBA Mask
    APBBMASK     : longWord;             //0018 APBB Mask
    APBCMASK     : longWord;             //001C APBC Mask
    APBDMASK     : longWord;             //0020 APBD Mask
  end;

  TNVMCTRL_Registers = record
    CTRLA        : word;                 //0000 Control A
    RESERVED0    : word;
    CTRLB        : word;                 //0004 Control B
    RESERVED1    : word;
    PARAM        : longWord;             //0008 NVM Parameter
    INTENCLR     : word;                 //000C Interrupt Enable Clear
    INTENSET     : word;                 //000E Interrupt Enable Set
    INTFLAG      : word;                 //0010 Interrupt Flag Status and Clear
    STATUS       : word;                 //0012 Status
    ADDR         : longWord;             //0014 Address
    RUNLOCK      : longWord;             //0018 Lock Section
    PBLDATA      : array[0..1] of longWord; //001C Page Buffer Load Data x
    ECCERR       : longWord;             //0024 ECC Error Status Register
    DBGCTRL      : byte;                 //0028 Debug Control
    RESERVED2    : byte;
    SEECFG       : byte;                 //002A SmartEEPROM Configuration Register
    RESERVED3    : byte;
    SEESTAT      : longWord;             //002C SmartEEPROM Status Register
  end;

  TSW0_FUSES_Registers = record
    SW0_WORD_0   : longWord;             //0000 SW0 Page Word 0
    SW0_WORD_1   : longWord;             //0004 SW0 Page Word 1
  end;

  TTEMP_LOG_FUSES_Registers = record
    TEMP_LOG_WORD_0 : longWord;             //0000 TEMP_LOG Page Word 0
    TEMP_LOG_WORD_1 : longWord;             //0004 TEMP_LOG Page Word 1
    TEMP_LOG_WORD_2 : longWord;             //0008 TEMP_LOG Page Word 2
  end;

  TUSER_FUSES_Registers = record
    USER_WORD_0  : longWord;             //0000 USER Page Word 0
    USER_WORD_1  : longWord;             //0004 USER Page Word 1
    USER_WORD_2  : longWord;             //0008 USER Page Word 2
  end;

  TOSCCTRL_DPLL_Registers = record
    DPLLCTRLA    : byte;                 //0000 DPLL Control A
    RESERVED0    : array[1..3] of byte;
    DPLLRATIO    : longWord;             //0004 DPLL Ratio Control
    DPLLCTRLB    : longWord;             //0008 DPLL Control B
    DPLLSYNCBUSY : longWord;             //000C DPLL Synchronization Busy
    DPLLSTATUS   : longWord;             //0010 DPLL Status
  end;

  TOSCCTRL_Registers = record
    EVCTRL       : byte;                 //0000 Event Control
    RESERVED0    : array[1..3] of byte;
    INTENCLR     : longWord;             //0004 Interrupt Enable Clear
    INTENSET     : longWord;             //0008 Interrupt Enable Set
    INTFLAG      : longWord;             //000C Interrupt Flag Status and Clear
    STATUS       : longWord;             //0010 Status
    XOSCCTRL     : array[0..1] of longWord; //0014 External Multipurpose Crystal Oscillator Control
    DFLLCTRLA    : byte;                 //001C DFLL48M Control A
    RESERVED1    : array[1..3] of byte;
    DFLLCTRLB    : byte;                 //0020 DFLL48M Control B
    RESERVED2    : array[1..3] of byte;
    DFLLVAL      : longWord;             //0024 DFLL48M Value
    DFLLMUL      : longWord;             //0028 DFLL48M Multiplier
    DFLLSYNC     : byte;                 //002C DFLL48M Synchronization
    RESERVED3    : array[1..3] of byte;
    DPLL         : array[0..1] of TOSCCTRL_DPLL_Registers;  //0030 
  end;

  TOSC32KCTRL_Registers = record
    INTENCLR     : longWord;             //0000 Interrupt Enable Clear
    INTENSET     : longWord;             //0004 Interrupt Enable Set
    INTFLAG      : longWord;             //0008 Interrupt Flag Status and Clear
    STATUS       : longWord;             //000C Power and Clocks Status
    RTCCTRL      : byte;                 //0010 RTC Clock Selection
    RESERVED0    : array[1..3] of byte;
    XOSC32K      : word;                 //0014 32kHz External Crystal Oscillator (XOSC32K) Control
    CFDCTRL      : byte;                 //0016 Clock Failure Detector Control
    EVCTRL       : byte;                 //0017 Event Control
    RESERVED1    : longWord;
    OSCULP32K    : longWord;             //001C 32kHz Ultra Low Power Internal Oscillator (OSCULP32K) Control
  end;

  TPAC_Registers = record
    WRCTRL       : longWord;             //0000 Write control
    EVCTRL       : byte;                 //0004 Event control
    RESERVED0    : array[1..3] of byte;
    INTENCLR     : byte;                 //0008 Interrupt enable clear
    INTENSET     : byte;                 //0009 Interrupt enable set
    RESERVED1    : array[1..6] of byte;
    INTFLAGAHB   : longWord;             //0010 Bridge interrupt flag status
    INTFLAGA     : longWord;             //0014 Peripheral interrupt flag status - Bridge A
    INTFLAGB     : longWord;             //0018 Peripheral interrupt flag status - Bridge B
    INTFLAGC     : longWord;             //001C Peripheral interrupt flag status - Bridge C
    INTFLAGD     : longWord;             //0020 Peripheral interrupt flag status - Bridge D
    RESERVED2    : array[1..16] of byte;
    STATUSA      : longWord;             //0034 Peripheral write protection status - Bridge A
    STATUSB      : longWord;             //0038 Peripheral write protection status - Bridge B
    STATUSC      : longWord;             //003C Peripheral write protection status - Bridge C
    STATUSD      : longWord;             //0040 Peripheral write protection status - Bridge D
  end;

  TPCC_Registers = record
    MR           : longWord;             //0000 Mode Register
    IER          : longWord;             //0004 Interrupt Enable Register
    IDR          : longWord;             //0008 Interrupt Disable Register
    IMR          : longWord;             //000C Interrupt Mask Register
    ISR          : longWord;             //0010 Interrupt Status Register
    RHR          : longWord;             //0014 Reception Holding Register
    RESERVED0    : array[1..200] of byte;
    WPMR         : longWord;             //00E0 Write Protection Mode Register
    WPSR         : longWord;             //00E4 Write Protection Status Register
  end;

  TPDEC_Registers = record
    CTRLA        : longWord;             //0000 Control A
    CTRLBCLR     : byte;                 //0004 Control B Clear
    CTRLBSET     : byte;                 //0005 Control B Set
    EVCTRL       : word;                 //0006 Event Control
    INTENCLR     : byte;                 //0008 Interrupt Enable Clear
    INTENSET     : byte;                 //0009 Interrupt Enable Set
    INTFLAG      : byte;                 //000A Interrupt Flag Status and Clear
    RESERVED0    : byte;
    STATUS       : word;                 //000C Status
    RESERVED1    : byte;
    DBGCTRL      : byte;                 //000F Debug Control
    SYNCBUSY     : longWord;             //0010 Synchronization Status
    PRESC        : byte;                 //0014 Prescaler Value
    FILTER       : byte;                 //0015 Filter Value
    RESERVED2    : word;
    PRESCBUF     : byte;                 //0018 Prescaler Buffer Value
    FILTERBUF    : byte;                 //0019 Filter Buffer Value
    RESERVED3    : word;
    COUNT        : longWord;             //001C Counter Value
    CC           : array[0..1] of longWord; //0020 Channel n Compare Value
    RESERVED4    : array[1..8] of byte;
    CCBUF        : array[0..1] of longWord; //0030 Channel Compare Buffer Value
  end;

  TPM_Registers = record
    CTRLA        : byte;                 //0000 Control A
    SLEEPCFG     : byte;                 //0001 Sleep Configuration
    RESERVED0    : word;
    INTENCLR     : byte;                 //0004 Interrupt Enable Clear
    INTENSET     : byte;                 //0005 Interrupt Enable Set
    INTFLAG      : byte;                 //0006 Interrupt Flag Status and Clear
    RESERVED1    : byte;
    STDBYCFG     : byte;                 //0008 Standby Configuration
    HIBCFG       : byte;                 //0009 Hibernate Configuration
    BKUPCFG      : byte;                 //000A Backup Configuration
    RESERVED2    : array[1..7] of byte;
    PWSAKDLY     : byte;                 //0012 Power Switch Acknowledge Delay
  end;

  TPORT_GROUP_Registers = record
    DIR          : longWord;             //0000 Data Direction
    DIRCLR       : longWord;             //0004 Data Direction Clear
    DIRSET       : longWord;             //0008 Data Direction Set
    DIRTGL       : longWord;             //000C Data Direction Toggle
    &OUT         : longWord;             //0010 Data Output Value
    OUTCLR       : longWord;             //0014 Data Output Value Clear
    OUTSET       : longWord;             //0018 Data Output Value Set
    OUTTGL       : longWord;             //001C Data Output Value Toggle
    &IN          : longWord;             //0020 Data Input Value
    CTRL         : longWord;             //0024 Control
    WRCONFIG     : longWord;             //0028 Write Configuration
    EVCTRL       : longWord;             //002C Event Input Control
    PMUX         : array[0..15] of byte;  //0030 Peripheral Multiplexing
    PINCFG       : array[0..31] of byte;  //0040 Pin Configuration
    Res          : array[$60..$7F] of byte;
  end;

  TPORT_Registers = record
    GROUP        : array[0..3] of TPORT_GROUP_Registers;  //0000 
  end;

  TPUKCC_Registers = record
  end;

  TQSPI_Registers = record
    CTRLA        : longWord;             //0000 Control A
    CTRLB        : longWord;             //0004 Control B
    BAUD         : longWord;             //0008 Baud Rate
    RXDATA       : longWord;             //000C Receive Data
    TXDATA       : longWord;             //0010 Transmit Data
    INTENCLR     : longWord;             //0014 Interrupt Enable Clear
    INTENSET     : longWord;             //0018 Interrupt Enable Set
    INTFLAG      : longWord;             //001C Interrupt Flag Status and Clear
    STATUS       : longWord;             //0020 Status Register
    RESERVED0    : array[1..12] of byte;
    INSTRADDR    : longWord;             //0030 Instruction Address
    INSTRCTRL    : longWord;             //0034 Instruction Code
    INSTRFRAME   : longWord;             //0038 Instruction Frame
    RESERVED1    : longWord;
    SCRAMBCTRL   : longWord;             //0040 Scrambling Mode
    SCRAMBKEY    : longWord;             //0044 Scrambling Key
  end;

  TRAMECC_Registers = record
    INTENCLR     : byte;                 //0000 Interrupt Enable Clear
    INTENSET     : byte;                 //0001 Interrupt Enable Set
    INTFLAG      : byte;                 //0002 Interrupt Flag
    STATUS       : byte;                 //0003 Status
    ERRADDR      : longWord;             //0004 Error Address
    RESERVED0    : array[1..7] of byte;
    DBGCTRL      : byte;                 //000F Debug Control
  end;

  TRSTC_Registers = record
    RCAUSE       : byte;                 //0000 Reset Cause
    RESERVED0    : byte;
    BKUPEXIT     : byte;                 //0002 Backup Exit Source
  end;

  TRTCMODE0_Registers = record
    CTRLA        : word;                 //0000 MODE0 Control A
    CTRLB        : word;                 //0002 MODE0 Control B
    EVCTRL       : longWord;             //0004 MODE0 Event Control
    INTENCLR     : word;                 //0008 MODE0 Interrupt Enable Clear
    INTENSET     : word;                 //000A MODE0 Interrupt Enable Set
    INTFLAG      : word;                 //000C MODE0 Interrupt Flag Status and Clear
    DBGCTRL      : byte;                 //000E Debug Control
    RESERVED0    : byte;
    SYNCBUSY     : longWord;             //0010 MODE0 Synchronization Busy Status
    FREQCORR     : byte;                 //0014 Frequency Correction
    RESERVED1    : array[1..3] of byte;
    COUNT        : longWord;             //0018 MODE0 Counter Value
    RESERVED2    : longWord;
    COMP         : array[0..1] of longWord; //0020 MODE0 Compare n Value
    RESERVED3    : array[1..24] of byte;
    GP           : array[0..3] of longWord; //0040 General Purpose
    RESERVED4    : array[1..16] of byte;
    TAMPCTRL     : longWord;             //0060 Tamper Control
    TIMESTAMP    : longWord;             //0064 MODE0 Timestamp
    TAMPID       : longWord;             //0068 Tamper ID
    RESERVED5    : array[1..20] of byte;
    BKUP         : array[0..7] of longWord; //0080 Backup
  end;

  TRTCMODE1_Registers = record
    CTRLA        : word;                 //0000 MODE1 Control A
    CTRLB        : word;                 //0002 MODE1 Control B
    EVCTRL       : longWord;             //0004 MODE1 Event Control
    INTENCLR     : word;                 //0008 MODE1 Interrupt Enable Clear
    INTENSET     : word;                 //000A MODE1 Interrupt Enable Set
    INTFLAG      : word;                 //000C MODE1 Interrupt Flag Status and Clear
    DBGCTRL      : byte;                 //000E Debug Control
    RESERVED0    : byte;
    SYNCBUSY     : longWord;             //0010 MODE1 Synchronization Busy Status
    FREQCORR     : byte;                 //0014 Frequency Correction
    RESERVED1    : array[1..3] of byte;
    COUNT        : word;                 //0018 MODE1 Counter Value
    RESERVED2    : word;
    PER          : word;                 //001C MODE1 Counter Period
    RESERVED3    : word;
    COMP         : array[0..3] of word;  //0020 MODE1 Compare n Value
    RESERVED4    : array[1..24] of byte;
    GP           : array[0..3] of longWord; //0040 General Purpose
    RESERVED5    : array[1..16] of byte;
    TAMPCTRL     : longWord;             //0060 Tamper Control
    TIMESTAMP    : longWord;             //0064 MODE1 Timestamp
    TAMPID       : longWord;             //0068 Tamper ID
    RESERVED6    : array[1..20] of byte;
    BKUP         : array[0..7] of longWord; //0080 Backup
  end;

  TRTCMODE2_Registers = record
    CTRLA        : word;                 //0000 MODE2 Control A
    CTRLB        : word;                 //0002 MODE2 Control B
    EVCTRL       : longWord;             //0004 MODE2 Event Control
    INTENCLR     : word;                 //0008 MODE2 Interrupt Enable Clear
    INTENSET     : word;                 //000A MODE2 Interrupt Enable Set
    INTFLAG      : word;                 //000C MODE2 Interrupt Flag Status and Clear
    DBGCTRL      : byte;                 //000E Debug Control
    RESERVED0    : byte;
    SYNCBUSY     : longWord;             //0010 MODE2 Synchronization Busy Status
    FREQCORR     : byte;                 //0014 Frequency Correction
    RESERVED1    : array[1..3] of byte;
    CLOCK        : longWord;             //0018 MODE2 Clock Value
    RESERVED2    : longWord;
    ALARM0       : longWord;             //0020 MODE2_ALARM Alarm n Value
    MASK0        : byte;                 //0024 MODE2_ALARM Alarm n Mask
    RESERVED3    : array[1..3] of byte;
    ALARM1       : longWord;             //0028 MODE2_ALARM Alarm n Value
    MASK1        : byte;                 //002C MODE2_ALARM Alarm n Mask
    RESERVED4    : array[1..19] of byte;
    GP           : array[0..3] of longWord; //0040 General Purpose
    RESERVED5    : array[1..16] of byte;
    TAMPCTRL     : longWord;             //0060 Tamper Control
    TIMESTAMP    : longWord;             //0064 MODE2 Timestamp
    TAMPID       : longWord;             //0068 Tamper ID
    RESERVED6    : array[1..20] of byte;
    BKUP         : array[0..7] of longWord; //0080 Backup
  end;

  TSDHC_Registers = record
    SSAR         : longWord;             //0000 SDMA System Address / Argument 2
    BSR          : word;                 //0004 Block Size
    BCR          : word;                 //0006 Block Count
    ARG1R        : longWord;             //0008 Argument 1
    TMR          : word;                 //000C Transfer Mode
    CR           : word;                 //000E Command
    RR           : array[0..3] of longWord; //0010 Response
    BDPR         : longWord;             //0020 Buffer Data Port
    PSR          : longWord;             //0024 Present State
    HC1R         : byte;                 //0028 Host Control 1
    PCR          : byte;                 //0029 Power Control
    BGCR         : byte;                 //002A Block Gap Control
    WCR          : byte;                 //002B Wakeup Control
    CCR          : word;                 //002C Clock Control
    TCR          : byte;                 //002E Timeout Control
    SRR          : byte;                 //002F Software Reset
    NISTR        : word;                 //0030 Normal Interrupt Status
    EISTR        : word;                 //0032 Error Interrupt Status
    NISTER       : word;                 //0034 Normal Interrupt Status Enable
    EISTER       : word;                 //0036 Error Interrupt Status Enable
    NISIER       : word;                 //0038 Normal Interrupt Signal Enable
    EISIER       : word;                 //003A Error Interrupt Signal Enable
    ACESR        : word;                 //003C Auto CMD Error Status
    HC2R         : word;                 //003E Host Control 2
    CA0R         : longWord;             //0040 Capabilities 0
    CA1R         : longWord;             //0044 Capabilities 1
    MCCAR        : longWord;             //0048 Maximum Current Capabilities
    RESERVED0    : longWord;
    FERACES      : word;                 //0050 Force Event for Auto CMD Error Status
    FEREIS       : word;                 //0052 Force Event for Error Interrupt Status
    AESR         : byte;                 //0054 ADMA Error Status
    RESERVED1    : array[1..3] of byte;
    ASAR         : longWord;             //0058 ADMA System Address n
    RESERVED2    : longWord;
    PVR          : array[0..7] of word;  //0060 Preset Value n
    RESERVED3    : array[1..140] of byte;
    SISR         : word;                 //00FC Slot Interrupt Status
    HCVR         : word;                 //00FE Host Controller Version
    RESERVED4    : array[1..260] of byte;
    MC1R         : byte;                 //0204 MMC Control 1
    MC2R         : byte;                 //0205 MMC Control 2
    RESERVED5    : word;
    ACR          : longWord;             //0208 AHB Control
    CC2R         : longWord;             //020C Clock Control 2
    RESERVED6    : array[1..32] of byte;
    CACR         : longWord;             //0230 Capabilities Control
    DBGR         : byte;                 //0234 Debug
  end;

  TSERCOMI2CM_Registers = record
    CTRLA        : longWord;             //0000 I2CM Control A
    CTRLB        : longWord;             //0004 I2CM Control B
    CTRLC        : longWord;             //0008 I2CM Control C
    BAUD         : longWord;             //000C I2CM Baud Rate
    RESERVED0    : longWord;
    INTENCLR     : byte;                 //0014 I2CM Interrupt Enable Clear
    RESERVED1    : byte;
    INTENSET     : byte;                 //0016 I2CM Interrupt Enable Set
    RESERVED2    : byte;
    INTFLAG      : byte;                 //0018 I2CM Interrupt Flag Status and Clear
    RESERVED3    : byte;
    STATUS       : word;                 //001A I2CM Status
    SYNCBUSY     : longWord;             //001C I2CM Synchronization Busy
    RESERVED4    : longWord;
    ADDR         : longWord;             //0024 I2CM Address
    DATA         : longWord;             //0028 I2CM Data
    RESERVED5    : longWord;
    DBGCTRL      : byte;                 //0030 I2CM Debug Control
  end;

  TSERCOMI2CS_Registers = record
    CTRLA        : longWord;             //0000 I2CS Control A
    CTRLB        : longWord;             //0004 I2CS Control B
    CTRLC        : longWord;             //0008 I2CS Control C
    RESERVED0    : array[1..8] of byte;
    INTENCLR     : byte;                 //0014 I2CS Interrupt Enable Clear
    RESERVED1    : byte;
    INTENSET     : byte;                 //0016 I2CS Interrupt Enable Set
    RESERVED2    : byte;
    INTFLAG      : byte;                 //0018 I2CS Interrupt Flag Status and Clear
    RESERVED3    : byte;
    STATUS       : word;                 //001A I2CS Status
    SYNCBUSY     : longWord;             //001C I2CS Synchronization Busy
    RESERVED4    : word;
    LENGTH       : word;                 //0022 I2CS Length
    ADDR         : longWord;             //0024 I2CS Address
    DATA         : longWord;             //0028 I2CS Data
  end;

  TSERCOMSPIS_Registers = record
    CTRLA        : longWord;             //0000 SPIS Control A
    CTRLB        : longWord;             //0004 SPIS Control B
    CTRLC        : longWord;             //0008 SPIS Control C
    BAUD         : byte;                 //000C SPIS Baud Rate
    RESERVED0    : array[1..7] of byte;
    INTENCLR     : byte;                 //0014 SPIS Interrupt Enable Clear
    RESERVED1    : byte;
    INTENSET     : byte;                 //0016 SPIS Interrupt Enable Set
    RESERVED2    : byte;
    INTFLAG      : byte;                 //0018 SPIS Interrupt Flag Status and Clear
    RESERVED3    : byte;
    STATUS       : word;                 //001A SPIS Status
    SYNCBUSY     : longWord;             //001C SPIS Synchronization Busy
    RESERVED4    : word;
    LENGTH       : word;                 //0022 SPIS Length
    ADDR         : longWord;             //0024 SPIS Address
    DATA         : longWord;             //0028 SPIS Data
    RESERVED5    : longWord;
    DBGCTRL      : byte;                 //0030 SPIS Debug Control
  end;

  TSERCOMSPIM_Registers = record
    CTRLA        : longWord;             //0000 SPIM Control A
    CTRLB        : longWord;             //0004 SPIM Control B
    CTRLC        : longWord;             //0008 SPIM Control C
    BAUD         : byte;                 //000C SPIM Baud Rate
    RESERVED0    : array[1..7] of byte;
    INTENCLR     : byte;                 //0014 SPIM Interrupt Enable Clear
    RESERVED1    : byte;
    INTENSET     : byte;                 //0016 SPIM Interrupt Enable Set
    RESERVED2    : byte;
    INTFLAG      : byte;                 //0018 SPIM Interrupt Flag Status and Clear
    RESERVED3    : byte;
    STATUS       : word;                 //001A SPIM Status
    SYNCBUSY     : longWord;             //001C SPIM Synchronization Busy
    RESERVED4    : word;
    LENGTH       : word;                 //0022 SPIM Length
    ADDR         : longWord;             //0024 SPIM Address
    DATA         : longWord;             //0028 SPIM Data
    RESERVED5    : longWord;
    DBGCTRL      : byte;                 //0030 SPIM Debug Control
  end;

  TSERCOMUSART_EXT_Registers = record
    CTRLA        : longWord;             //0000 USART_EXT Control A
    CTRLB        : longWord;             //0004 USART_EXT Control B
    CTRLC        : longWord;             //0008 USART_EXT Control C
    BAUD         : word;                 //000C USART_EXT Baud Rate
    RXPL         : byte;                 //000E USART_EXT Receive Pulse Length
    RESERVED0    : array[1..5] of byte;
    INTENCLR     : byte;                 //0014 USART_EXT Interrupt Enable Clear
    RESERVED1    : byte;
    INTENSET     : byte;                 //0016 USART_EXT Interrupt Enable Set
    RESERVED2    : byte;
    INTFLAG      : byte;                 //0018 USART_EXT Interrupt Flag Status and Clear
    RESERVED3    : byte;
    STATUS       : word;                 //001A USART_EXT Status
    SYNCBUSY     : longWord;             //001C USART_EXT Synchronization Busy
    RXERRCNT     : byte;                 //0020 USART_EXT Receive Error Count
    RESERVED4    : byte;
    LENGTH       : word;                 //0022 USART_EXT Length
    RESERVED5    : longWord;
    DATA         : longWord;             //0028 USART_EXT Data
    RESERVED6    : longWord;
    DBGCTRL      : byte;                 //0030 USART_EXT Debug Control
  end;

  TSERCOMUSART_INT_Registers = record
    CTRLA        : longWord;             //0000 USART_INT Control A
    CTRLB        : longWord;             //0004 USART_INT Control B
    CTRLC        : longWord;             //0008 USART_INT Control C
    BAUD         : word;                 //000C USART_INT Baud Rate
    RXPL         : byte;                 //000E USART_INT Receive Pulse Length
    RESERVED0    : array[1..5] of byte;
    INTENCLR     : byte;                 //0014 USART_INT Interrupt Enable Clear
    RESERVED1    : byte;
    INTENSET     : byte;                 //0016 USART_INT Interrupt Enable Set
    RESERVED2    : byte;
    INTFLAG      : byte;                 //0018 USART_INT Interrupt Flag Status and Clear
    RESERVED3    : byte;
    STATUS       : word;                 //001A USART_INT Status
    SYNCBUSY     : longWord;             //001C USART_INT Synchronization Busy
    RXERRCNT     : byte;                 //0020 USART_INT Receive Error Count
    RESERVED4    : byte;
    LENGTH       : word;                 //0022 USART_INT Length
    RESERVED5    : longWord;
    DATA         : longWord;             //0028 USART_INT Data
    RESERVED6    : longWord;
    DBGCTRL      : byte;                 //0030 USART_INT Debug Control
  end;

  TSUPC_Registers = record
    INTENCLR     : longWord;             //0000 Interrupt Enable Clear
    INTENSET     : longWord;             //0004 Interrupt Enable Set
    INTFLAG      : longWord;             //0008 Interrupt Flag Status and Clear
    STATUS       : longWord;             //000C Power and Clocks Status
    BOD33        : longWord;             //0010 BOD33 Control
    RESERVED0    : longWord;
    VREG         : longWord;             //0018 VREG Control
    VREF         : longWord;             //001C VREF Control
    BBPS         : longWord;             //0020 Battery Backup Power Switch
    BKOUT        : longWord;             //0024 Backup Output Control
    BKIN         : longWord;             //0028 Backup Input Control
  end;

  TTCCOUNT8_Registers = record
    CTRLA        : longWord;             //0000 Control A
    CTRLBCLR     : byte;                 //0004 Control B Clear
    CTRLBSET     : byte;                 //0005 Control B Set
    EVCTRL       : word;                 //0006 Event Control
    INTENCLR     : byte;                 //0008 Interrupt Enable Clear
    INTENSET     : byte;                 //0009 Interrupt Enable Set
    INTFLAG      : byte;                 //000A Interrupt Flag Status and Clear
    STATUS       : byte;                 //000B Status
    WAVE         : byte;                 //000C Waveform Generation Control
    DRVCTRL      : byte;                 //000D Control C
    RESERVED0    : byte;
    DBGCTRL      : byte;                 //000F Debug Control
    SYNCBUSY     : longWord;             //0010 Synchronization Status
    COUNT        : byte;                 //0014 COUNT8 Count
    RESERVED1    : array[1..6] of byte;
    PER          : byte;                 //001B COUNT8 Period
    CC           : array[0..1] of byte;  //001C COUNT8 Compare and Capture
    RESERVED2    : array[1..17] of byte;
    PERBUF       : byte;                 //002F COUNT8 Period Buffer
    CCBUF        : array[0..1] of byte;  //0030 COUNT8 Compare and Capture Buffer
  end;

  TTCCOUNT16_Registers = record
    CTRLA        : longWord;             //0000 Control A
    CTRLBCLR     : byte;                 //0004 Control B Clear
    CTRLBSET     : byte;                 //0005 Control B Set
    EVCTRL       : word;                 //0006 Event Control
    INTENCLR     : byte;                 //0008 Interrupt Enable Clear
    INTENSET     : byte;                 //0009 Interrupt Enable Set
    INTFLAG      : byte;                 //000A Interrupt Flag Status and Clear
    STATUS       : byte;                 //000B Status
    WAVE         : byte;                 //000C Waveform Generation Control
    DRVCTRL      : byte;                 //000D Control C
    RESERVED0    : byte;
    DBGCTRL      : byte;                 //000F Debug Control
    SYNCBUSY     : longWord;             //0010 Synchronization Status
    COUNT        : word;                 //0014 COUNT16 Count
    RESERVED1    : array[1..6] of byte;
    CC           : array[0..1] of word;  //001C COUNT16 Compare and Capture
    RESERVED2    : array[1..16] of byte;
    CCBUF        : array[0..1] of word;  //0030 COUNT16 Compare and Capture Buffer
  end;

  TTCCOUNT32_Registers = record
    CTRLA        : longWord;             //0000 Control A
    CTRLBCLR     : byte;                 //0004 Control B Clear
    CTRLBSET     : byte;                 //0005 Control B Set
    EVCTRL       : word;                 //0006 Event Control
    INTENCLR     : byte;                 //0008 Interrupt Enable Clear
    INTENSET     : byte;                 //0009 Interrupt Enable Set
    INTFLAG      : byte;                 //000A Interrupt Flag Status and Clear
    STATUS       : byte;                 //000B Status
    WAVE         : byte;                 //000C Waveform Generation Control
    DRVCTRL      : byte;                 //000D Control C
    RESERVED0    : byte;
    DBGCTRL      : byte;                 //000F Debug Control
    SYNCBUSY     : longWord;             //0010 Synchronization Status
    COUNT        : longWord;             //0014 COUNT32 Count
    RESERVED1    : longWord;
    CC           : array[0..1] of longWord; //001C COUNT32 Compare and Capture
    RESERVED2    : array[1..12] of byte;
    CCBUF        : array[0..1] of longWord; //0030 COUNT32 Compare and Capture Buffer
  end;

  TTCC_Registers = record
    CTRLA        : longWord;             //0000 Control A
    CTRLBCLR     : byte;                 //0004 Control B Clear
    CTRLBSET     : byte;                 //0005 Control B Set
    RESERVED0    : word;
    SYNCBUSY     : longWord;             //0008 Synchronization Busy
    FCTRLA       : longWord;             //000C Recoverable Fault A Configuration
    FCTRLB       : longWord;             //0010 Recoverable Fault B Configuration
    WEXCTRL      : longWord;             //0014 Waveform Extension Configuration
    DRVCTRL      : longWord;             //0018 Driver Control
    RESERVED1    : word;
    DBGCTRL      : byte;                 //001E Debug Control
    RESERVED2    : byte;
    EVCTRL       : longWord;             //0020 Event Control
    INTENCLR     : longWord;             //0024 Interrupt Enable Clear
    INTENSET     : longWord;             //0028 Interrupt Enable Set
    INTFLAG      : longWord;             //002C Interrupt Flag Status and Clear
    STATUS       : longWord;             //0030 Status
    COUNT        : longWord;             //0034 Count
    PATT         : word;                 //0038 Pattern
    RESERVED3    : word;
    WAVE         : longWord;             //003C Waveform Control
    PER          : longWord;             //0040 Period
    CC           : array[0..5] of longWord; //0044 Compare and Capture
    RESERVED4    : array[1..8] of byte;
    PATTBUF      : word;                 //0064 Pattern Buffer
    RESERVED5    : array[1..6] of byte;
    PERBUF       : longWord;             //006C Period Buffer
    CCBUF        : array[0..5] of longWord; //0070 Compare and Capture Buffer
  end;

  TTRNG_Registers = record
    CTRLA        : byte;                 //0000 Control A
    RESERVED0    : array[1..3] of byte;
    EVCTRL       : byte;                 //0004 Event Control
    RESERVED1    : array[1..3] of byte;
    INTENCLR     : byte;                 //0008 Interrupt Enable Clear
    INTENSET     : byte;                 //0009 Interrupt Enable Set
    INTFLAG      : byte;                 //000A Interrupt Flag Status and Clear
    RESERVED2    : array[1..21] of byte;
    DATA         : longWord;             //0020 Output Data
  end;

  TUSB_DEVICE_DESC_BANK_Registers = record
    ADDR         : longWord;             //0000 DEVICE_DESC_BANK Endpoint Bank, Adress of Data Buffer
    PCKSIZE      : longWord;             //0004 DEVICE_DESC_BANK Endpoint Bank, Packet Size
    EXTREG       : word;                 //0008 DEVICE_DESC_BANK Endpoint Bank, Extended
    STATUS_BK    : byte;                 //000A DEVICE_DESC_BANK Enpoint Bank, Status of Bank
  end;

  TUSB_HOST_DESC_BANK_Registers = record
    ADDR         : longWord;             //0000 HOST_DESC_BANK Host Bank, Adress of Data Buffer
    PCKSIZE      : longWord;             //0004 HOST_DESC_BANK Host Bank, Packet Size
    EXTREG       : word;                 //0008 HOST_DESC_BANK Host Bank, Extended
    STATUS_BK    : byte;                 //000A HOST_DESC_BANK Host Bank, Status of Bank
    RESERVED0    : byte;
    CTRL_PIPE    : word;                 //000C HOST_DESC_BANK Host Bank, Host Control Pipe
    STATUS_PIPE  : word;                 //000E HOST_DESC_BANK Host Bank, Host Status Pipe
  end;

  TUSB_DEVICE_ENDPOINT_Registers = record
    EPCFG        : byte;                 //0000 DEVICE_ENDPOINT End Point Configuration
    RESERVED0    : array[1..3] of byte;
    EPSTATUSCLR  : byte;                 //0004 DEVICE_ENDPOINT End Point Pipe Status Clear
    EPSTATUSSET  : byte;                 //0005 DEVICE_ENDPOINT End Point Pipe Status Set
    EPSTATUS     : byte;                 //0006 DEVICE_ENDPOINT End Point Pipe Status
    EPINTFLAG    : byte;                 //0007 DEVICE_ENDPOINT End Point Interrupt Flag
    EPINTENCLR   : byte;                 //0008 DEVICE_ENDPOINT End Point Interrupt Clear Flag
    EPINTENSET   : byte;                 //0009 DEVICE_ENDPOINT End Point Interrupt Set Flag
  end;

  TUSB_HOST_PIPE_Registers = record
    PCFG         : byte;                 //0000 HOST_PIPE End Point Configuration
    RESERVED0    : word;
    BINTERVAL    : byte;                 //0003 HOST_PIPE Bus Access Period of Pipe
    PSTATUSCLR   : byte;                 //0004 HOST_PIPE End Point Pipe Status Clear
    PSTATUSSET   : byte;                 //0005 HOST_PIPE End Point Pipe Status Set
    PSTATUS      : byte;                 //0006 HOST_PIPE End Point Pipe Status
    PINTFLAG     : byte;                 //0007 HOST_PIPE Pipe Interrupt Flag
    PINTENCLR    : byte;                 //0008 HOST_PIPE Pipe Interrupt Flag Clear
    PINTENSET    : byte;                 //0009 HOST_PIPE Pipe Interrupt Flag Set
  end;

  TUSBDEVICE_Registers = record
    CTRLA        : byte;                 //0000 Control A
    RESERVED0    : byte;
    SYNCBUSY     : byte;                 //0002 Synchronization Busy
    QOSCTRL      : byte;                 //0003 USB Quality Of Service
    RESERVED1    : longWord;
    CTRLB        : word;                 //0008 DEVICE Control B
    DADD         : byte;                 //000A DEVICE Device Address
    RESERVED2    : byte;
    STATUS       : byte;                 //000C DEVICE Status
    FSMSTATUS    : byte;                 //000D Finite State Machine Status
    RESERVED3    : word;
    FNUM         : word;                 //0010 DEVICE Device Frame Number
    RESERVED4    : word;
    INTENCLR     : word;                 //0014 DEVICE Device Interrupt Enable Clear
    RESERVED5    : word;
    INTENSET     : word;                 //0018 DEVICE Device Interrupt Enable Set
    RESERVED6    : word;
    INTFLAG      : word;                 //001C DEVICE Device Interrupt Flag
    RESERVED7    : word;
    EPINTSMRY    : word;                 //0020 DEVICE End Point Interrupt Summary
    RESERVED8    : word;
    DESCADD      : longWord;             //0024 Descriptor Address
    PADCAL       : word;                 //0028 USB PAD Calibration
    RESERVED9    : array[1..214] of byte;
    HOST_PIPE    : array[0..7] of TUSB_HOST_PIPE_Registers;  //0100 
  end;

  TUSBHOST_Registers = record
    CTRLA        : byte;                 //0000 Control A
    RESERVED0    : byte;
    SYNCBUSY     : byte;                 //0002 Synchronization Busy
    QOSCTRL      : byte;                 //0003 USB Quality Of Service
    RESERVED1    : longWord;
    CTRLB        : word;                 //0008 HOST Control B
    HSOFC        : byte;                 //000A HOST Host Start Of Frame Control
    RESERVED2    : byte;
    STATUS       : byte;                 //000C HOST Status
    FSMSTATUS    : byte;                 //000D Finite State Machine Status
    RESERVED3    : word;
    FNUM         : word;                 //0010 HOST Host Frame Number
    FLENHIGH     : byte;                 //0012 HOST Host Frame Length
    RESERVED4    : byte;
    INTENCLR     : word;                 //0014 HOST Host Interrupt Enable Clear
    RESERVED5    : word;
    INTENSET     : word;                 //0018 HOST Host Interrupt Enable Set
    RESERVED6    : word;
    INTFLAG      : word;                 //001C HOST Host Interrupt Flag
    RESERVED7    : word;
    PINTSMRY     : word;                 //0020 HOST Pipe Interrupt Summary
    RESERVED8    : word;
    DESCADD      : longWord;             //0024 Descriptor Address
    PADCAL       : word;                 //0028 USB PAD Calibration
    RESERVED9    : array[1..214] of byte;
    HOST_PIPE    : array[0..7] of TUSB_HOST_PIPE_Registers;  //0100 
  end;

  TUSB_DESCRIPTORDEVICE_Registers = record
    HOST_DESC_BANK : array[0..1] of TUSB_HOST_DESC_BANK_Registers;  //0000 
  end;

  TUSB_DESCRIPTORHOST_Registers = record
    HOST_DESC_BANK : array[0..1] of TUSB_HOST_DESC_BANK_Registers;  //0000 
  end;

  TWDT_Registers = record
    CTRLA        : byte;                 //0000 Control
    CONFIG       : byte;                 //0001 Configuration
    EWCTRL       : byte;                 //0002 Early Warning Interrupt Control
    RESERVED0    : byte;
    INTENCLR     : byte;                 //0004 Interrupt Enable Clear
    INTENSET     : byte;                 //0005 Interrupt Enable Set
    INTFLAG      : byte;                 //0006 Interrupt Flag Status and Clear
    RESERVED1    : byte;
    SYNCBUSY     : longWord;             //0008 Synchronization Busy
    CLEAR        : byte;                 //000C Clear
  end;

  TETM_Registers = record
    CR           : longWord;             //0000 ETM Main Control Register
    CCR          : longWord;             //0004 ETM Configuration Code Register
    TRIGGER      : longWord;             //0008 ETM Trigger Event Register
    RESERVED0    : longWord;
    SR           : longWord;             //0010 ETM Status Register
    SCR          : longWord;             //0014 ETM System Configuration Register
    RESERVED1    : array[1..8] of byte;
    TEEVR        : longWord;             //0020 ETM TraceEnable Event Register
    TECR1        : longWord;             //0024 ETM TraceEnable Control 1 Register
    FFLR         : longWord;             //0028 ETM FIFO Full Level Register
    RESERVED2    : array[1..276] of byte;
    CNTRLDVR1    : longWord;             //0140 ETM Free-running Counter Reload Value
    RESERVED3    : array[1..156] of byte;
    SYNCFR       : longWord;             //01E0 ETM Synchronization Frequency Register
    IDR          : longWord;             //01E4 ETM ID Register
    CCER         : longWord;             //01E8 ETM Configuration Code Extension Register
    RESERVED4    : longWord;
    TESSEICR     : longWord;             //01F0 ETM TraceEnable Start/Stop EmbeddedICE Control Register
    RESERVED5    : longWord;
    TSEVT        : longWord;             //01F8 ETM TimeStamp Event Register
    RESERVED6    : longWord;
    TRACEIDR     : longWord;             //0200 ETM CoreSight Trace ID Register
    RESERVED7    : longWord;
    IDR2         : longWord;             //0208 ETM ID Register 2
    RESERVED8    : array[1..264] of byte;
    PDSR         : longWord;             //0314 ETM Device Power-Down Status Register
    RESERVED9    : array[1..3016] of byte;
    ITMISCIN     : longWord;             //0EE0 ETM Integration Test Miscellaneous Inputs
    RESERVED10   : longWord;
    ITTRIGOUT    : longWord;             //0EE8 ETM Integration Test Trigger Out
    RESERVED11   : longWord;
    ITATBCTR2    : longWord;             //0EF0 ETM Integration Test ATB Control 2
    RESERVED12   : longWord;
    ITATBCTR0    : longWord;             //0EF8 ETM Integration Test ATB Control 0
    RESERVED13   : longWord;
    ITCTRL       : longWord;             //0F00 ETM Integration Mode Control Register
    RESERVED14   : array[1..156] of byte;
    CLAIMSET     : longWord;             //0FA0 ETM Claim Tag Set Register
    CLAIMCLR     : longWord;             //0FA4 ETM Claim Tag Clear Register
    RESERVED15   : array[1..8] of byte;
    LAR          : longWord;             //0FB0 ETM Lock Access Register
    LSR          : longWord;             //0FB4 ETM Lock Status Register
    AUTHSTATUS   : longWord;             //0FB8 ETM Authentication Status Register
    RESERVED16   : array[1..16] of byte;
    DEVTYPE      : longWord;             //0FCC ETM CoreSight Device Type Register
    PIDR4        : longWord;             //0FD0 ETM Peripheral Identification Register #4
    PIDR5        : longWord;             //0FD4 ETM Peripheral Identification Register #5
    PIDR6        : longWord;             //0FD8 ETM Peripheral Identification Register #6
    PIDR7        : longWord;             //0FDC ETM Peripheral Identification Register #7
    PIDR0        : longWord;             //0FE0 ETM Peripheral Identification Register #0
    PIDR1        : longWord;             //0FE4 ETM Peripheral Identification Register #1
    PIDR2        : longWord;             //0FE8 ETM Peripheral Identification Register #2
    PIDR3        : longWord;             //0FEC ETM Peripheral Identification Register #3
    CIDR0        : longWord;             //0FF0 ETM Component  Identification Register #0
    CIDR1        : longWord;             //0FF4 ETM Component  Identification Register #1
    CIDR2        : longWord;             //0FF8 ETM Component  Identification Register #2
    CIDR3        : longWord;             //0FFC ETM Component  Identification Register #3
  end;

  TMPU_Registers = record
    &TYPE        : longWord;             //0000 MPU Type Register
    CTRL         : longWord;             //0004 MPU Control Register
    RNR          : longWord;             //0008 MPU Region Number Register
    RBAR         : longWord;             //000C MPU Region Base Address Register
    RASR         : longWord;             //0010 MPU Region Attribute and Size Register
    RBAR_A1      : longWord;             //0014 MPU Alias 1 Region Base Address Register
    RASR_A1      : longWord;             //0018 MPU Alias 1 Region Attribute and Size Register
    RBAR_A2      : longWord;             //001C MPU Alias 2 Region Base Address Register
    RASR_A2      : longWord;             //0020 MPU Alias 2 Region Attribute and Size Register
    RBAR_A3      : longWord;             //0024 MPU Alias 3 Region Base Address Register
    RASR_A3      : longWord;             //0028 MPU Alias 3 Region Attribute and Size Register
  end;

  TSystemControl_Registers = record
    ICTR         : longWord;             //0004 Interrupt Controller Type Register
    ACTLR        : longWord;             //0008 Auxiliary Control Register
    RESERVED0    : array[1..3316] of byte;
    CPUID        : longWord;             //0D00 CPUID Base Register
    ICSR         : longWord;             //0D04 Interrupt Control and State Register
    VTOR         : longWord;             //0D08 Vector Table Offset Register
    AIRCR        : longWord;             //0D0C Application Interrupt and Reset Control Register
    SCR          : longWord;             //0D10 System Control Register
    CCR          : longWord;             //0D14 Configuration and Control Register
    SHPR1        : longWord;             //0D18 System Handler Priority Register 1
    SHPR2        : longWord;             //0D1C System Handler Priority Register 2
    SHPR3        : longWord;             //0D20 System Handler Priority Register 3
    SHCSR        : longWord;             //0D24 System Handler Control and State Register
    CFSR         : longWord;             //0D28 Configurable Fault Status Register
    HFSR         : longWord;             //0D2C HardFault Status Register
    DFSR         : longWord;             //0D30 Debug Fault Status Register
    MMFAR        : longWord;             //0D34 MemManage Fault Address Register
    BFAR         : longWord;             //0D38 BusFault Address Register
    AFSR         : longWord;             //0D3C Auxiliary Fault Status Register
    PFR          : array[0..1] of longWord; //0D40 Processor Feature Register
    DFR          : longWord;             //0D48 Debug Feature Register
    ADR          : longWord;             //0D4C Auxiliary Feature Register
    MMFR         : array[0..3] of longWord; //0D50 Memory Model Feature Register
    ISAR         : array[0..4] of longWord; //0D60 Instruction Set Attributes Register
    RESERVED1    : array[1..20] of byte;
    CPACR        : longWord;             //0D88 Coprocessor Access Control Register
  end;

  TTPIU_Registers = record
    SSPSR        : longWord;             //0000 Supported Parallel Port Size Register
    CSPSR        : longWord;             //0004 Current Parallel Port Size Register
    RESERVED0    : array[1..8] of byte;
    ACPR         : longWord;             //0010 Asynchronous Clock Prescaler Register
    RESERVED1    : array[1..220] of byte;
    SPPR         : longWord;             //00F0 Selected Pin Protocol Register
    RESERVED2    : array[1..524] of byte;
    FFSR         : longWord;             //0300 Formatter and Flush Status Register
    FFCR         : longWord;             //0304 Formatter and Flush Control Register
    FSCR         : longWord;             //0308 Formatter Synchronization Counter Register
    RESERVED3    : array[1..3036] of byte;
    TRIGGER      : longWord;             //0EE8 TRIGGER
    FIFO0        : longWord;             //0EEC Integration ETM Data
    ITATBCTR2    : longWord;             //0EF0 ITATBCTR2
    RESERVED4    : longWord;
    ITATBCTR0    : longWord;             //0EF8 ITATBCTR0
    FIFO1        : longWord;             //0EFC Integration ITM Data
    ITCTRL       : longWord;             //0F00 Integration Mode Control
    RESERVED5    : array[1..156] of byte;
    CLAIMSET     : longWord;             //0FA0 Claim tag set
    CLAIMCLR     : longWord;             //0FA4 Claim tag clear
    RESERVED6    : array[1..32] of byte;
    DEVID        : longWord;             //0FC8 TPIU_DEVID
    DEVTYPE      : longWord;             //0FCC TPIU_DEVTYPE
  end;

  TRTC_Registers = record
  case byte of
    0: ( MODE0 : TRTCMODE0_Registers );
    1: ( MODE1 : TRTCMODE1_Registers );
    2: ( MODE2 : TRTCMODE2_Registers );
    end;

  TSERCOM_Registers = record
  case byte of
    0: ( I2CM : TSERCOMI2CM_Registers );
    1: ( I2CS : TSERCOMI2CS_Registers );
    2: ( SPIS : TSERCOMSPIS_Registers );
    3: ( SPIM : TSERCOMSPIM_Registers );
    4: ( USART_EXT : TSERCOMUSART_EXT_Registers );
    5: ( USART_INT : TSERCOMUSART_INT_Registers );
    end;

  TTC_Registers = record
  case byte of
    0: ( COUNT8 : TTCCOUNT8_Registers );
    1: ( COUNT16 : TTCCOUNT16_Registers );
    2: ( COUNT32 : TTCCOUNT32_Registers );
    end;

  TUSB_Registers = record
  case byte of
    0: ( DEVICE : TUSBDEVICE_Registers );
    1: ( HOST : TUSBHOST_Registers );
    end;

  TUSB_DESCRIPTOR_Registers = record
  case byte of
    0: ( DEVICE : TUSB_DESCRIPTORDEVICE_Registers );
    1: ( HOST : TUSB_DESCRIPTORHOST_Registers );
    end;

const
  AC_BASE             = $42002000;
  ADC0_BASE           = $43001c00;
  ADC1_BASE           = $43002000;
  AES_BASE            = $42002400;
  CCL_BASE            = $42003800;
  CMCC_BASE           = $41006000;
  DAC_BASE            = $43002400;
  DMAC_BASE           = $4100a000;
  DSU_BASE            = $41002000;
  EIC_BASE            = $40002800;
  ETM_BASE            = $e0041000;
  EVSYS_BASE          = $4100e000;
  FREQM_BASE          = $40002c00;
  GCLK_BASE           = $40001c00;
  HMATRIX_BASE        = $4100c000;
  I2S_BASE            = $43002800;
  ICM_BASE            = $42002c00;
  MCLK_BASE           = $40000800;
  MPU_BASE            = $e000ed90;
  NVMCTRL_BASE        = $41004000;
  OSC32KCTRL_BASE     = $40001400;
  OSCCTRL_BASE        = $40001000;
  PAC_BASE            = $40000000;
  PCC_BASE            = $43002c00;
  PDEC_BASE           = $42001c00;
  PM_BASE             = $40000400;
  PORT_BASE           = $41008000;
  QSPI_BASE           = $42003400;
  RAMECC_BASE         = $41020000;
  RSTC_BASE           = $40000c00;
  RTC_BASE            = $40002400;
  SDHC0_BASE          = $45000000;
  SDHC1_BASE          = $46000000;
  SERCOM0_BASE        = $40003000;
  SERCOM1_BASE        = $40003400;
  SERCOM2_BASE        = $41012000;
  SERCOM3_BASE        = $41014000;
  SERCOM4_BASE        = $43000000;
  SERCOM5_BASE        = $43000400;
  SERCOM6_BASE        = $43000800;
  SERCOM7_BASE        = $43000c00;
  SUPC_BASE           = $40001800;
  SW0_FUSES_BASE      = $00800080;
  SystemControl_BASE  = $e000e000;
  TC0_BASE            = $40003800;
  TC1_BASE            = $40003c00;
  TC2_BASE            = $4101a000;
  TC3_BASE            = $4101c000;
  TC4_BASE            = $42001400;
  TC5_BASE            = $42001800;
  TC6_BASE            = $43001400;
  TC7_BASE            = $43001800;
  TCC0_BASE           = $41016000;
  TCC1_BASE           = $41018000;
  TCC2_BASE           = $42000c00;
  TCC3_BASE           = $42001000;
  TCC4_BASE           = $43001000;
  TEMP_LOG_FUSES_BASE = $00800100;
  TPIU_BASE           = $e0040000;
  TRNG_BASE           = $42002800;
  USB_BASE            = $41000000;
  USER_FUSES_BASE     = $00804000;
  WDT_BASE            = $40002000;

var
  AC                  : TAC_Registers        absolute AC_BASE;
  ADC0                : TADC_Registers       absolute ADC0_BASE;
  ADC1                : TADC_Registers       absolute ADC1_BASE;
  AES                 : TAES_Registers       absolute AES_BASE;
  CCL                 : TCCL_Registers       absolute CCL_BASE;
  CMCC                : TCMCC_Registers      absolute CMCC_BASE;
  DAC                 : TDAC_Registers       absolute DAC_BASE;
  DMAC                : TDMAC_Registers      absolute DMAC_BASE;
  DSU                 : TDSU_Registers       absolute DSU_BASE;
  EIC                 : TEIC_Registers       absolute EIC_BASE;
  ETM                 : TETM_Registers       absolute ETM_BASE;
  EVSYS               : TEVSYS_Registers     absolute EVSYS_BASE;
  FREQM               : TFREQM_Registers     absolute FREQM_BASE;
  GCLK                : TGCLK_Registers      absolute GCLK_BASE;
  HMATRIX             : THMATRIXB_Registers  absolute HMATRIX_BASE;
  I2S                 : TI2S_Registers       absolute I2S_BASE;
  ICM                 : TICM_Registers       absolute ICM_BASE;
  MCLK                : TMCLK_Registers      absolute MCLK_BASE;
  MPU                 : TMPU_Registers       absolute MPU_BASE;
  NVMCTRL             : TNVMCTRL_Registers   absolute NVMCTRL_BASE;
  OSC32KCTRL          : TOSC32KCTRL_Registers absolute OSC32KCTRL_BASE;
  OSCCTRL             : TOSCCTRL_Registers   absolute OSCCTRL_BASE;
  PAC                 : TPAC_Registers       absolute PAC_BASE;
  PCC                 : TPCC_Registers       absolute PCC_BASE;
  PDEC                : TPDEC_Registers      absolute PDEC_BASE;
  PM                  : TPM_Registers        absolute PM_BASE;
  PORT                : TPORT_Registers      absolute PORT_BASE;
  QSPI                : TQSPI_Registers      absolute QSPI_BASE;
  RAMECC              : TRAMECC_Registers    absolute RAMECC_BASE;
  RSTC                : TRSTC_Registers      absolute RSTC_BASE;
  RTC                 : TRTC_Registers       absolute RTC_BASE;
  SDHC0               : TSDHC_Registers      absolute SDHC0_BASE;
  SDHC1               : TSDHC_Registers      absolute SDHC1_BASE;
  SERCOM0             : TSERCOM_Registers    absolute SERCOM0_BASE;
  SERCOM1             : TSERCOM_Registers    absolute SERCOM1_BASE;
  SERCOM2             : TSERCOM_Registers    absolute SERCOM2_BASE;
  SERCOM3             : TSERCOM_Registers    absolute SERCOM3_BASE;
  SERCOM4             : TSERCOM_Registers    absolute SERCOM4_BASE;
  SERCOM5             : TSERCOM_Registers    absolute SERCOM5_BASE;
  SERCOM6             : TSERCOM_Registers    absolute SERCOM6_BASE;
  SERCOM7             : TSERCOM_Registers    absolute SERCOM7_BASE;
  SUPC                : TSUPC_Registers      absolute SUPC_BASE;
  SW0_FUSES           : TSW0_FUSES_Registers absolute SW0_FUSES_BASE;
  SystemControl       : TSystemControl_Registers absolute SystemControl_BASE;
  TC0                 : TTC_Registers        absolute TC0_BASE;
  TC1                 : TTC_Registers        absolute TC1_BASE;
  TC2                 : TTC_Registers        absolute TC2_BASE;
  TC3                 : TTC_Registers        absolute TC3_BASE;
  TC4                 : TTC_Registers        absolute TC4_BASE;
  TC5                 : TTC_Registers        absolute TC5_BASE;
  TC6                 : TTC_Registers        absolute TC6_BASE;
  TC7                 : TTC_Registers        absolute TC7_BASE;
  TCC0                : TTCC_Registers       absolute TCC0_BASE;
  TCC1                : TTCC_Registers       absolute TCC1_BASE;
  TCC2                : TTCC_Registers       absolute TCC2_BASE;
  TCC3                : TTCC_Registers       absolute TCC3_BASE;
  TCC4                : TTCC_Registers       absolute TCC4_BASE;
  TEMP_LOG_FUSES      : TTEMP_LOG_FUSES_Registers absolute TEMP_LOG_FUSES_BASE;
  TPIU                : TTPIU_Registers      absolute TPIU_BASE;
  TRNG                : TTRNG_Registers      absolute TRNG_BASE;
  USB                 : TUSB_Registers       absolute USB_BASE;
  USER_FUSES          : TUSER_FUSES_Registers absolute USER_FUSES_BASE;
  WDT                 : TWDT_Registers       absolute WDT_BASE;

implementation

{$DEFINE IMPLEMENTATION}
{$UNDEF INTERFACE}

procedure NonMaskableInt_interrupt;   external name 'NonMaskableInt_interrupt';
procedure HardFault_interrupt;        external name 'HardFault_interrupt';
procedure MemoryManagement_interrupt; external name 'MemoryManagement_interrupt';
procedure BusFault_interrupt;         external name 'BusFault_interrupt';
procedure UsageFault_interrupt;       external name 'UsageFault_interrupt';
procedure SVCall_interrupt;           external name 'SVCall_interrupt';
procedure DebugMonitor_interrupt;     external name 'DebugMonitor_interrupt';
procedure PendSV_interrupt;           external name 'PendSV_interrupt';
procedure SysTick_interrupt;          external name 'SysTick_interrupt';
procedure PM_interrupt;               external name 'PM_interrupt';
procedure MCLK_interrupt;             external name 'MCLK_interrupt';
procedure OSCCTRL_0_interrupt;        external name 'OSCCTRL_0_interrupt';
procedure OSCCTRL_1_interrupt;        external name 'OSCCTRL_1_interrupt';
procedure OSCCTRL_2_interrupt;        external name 'OSCCTRL_2_interrupt';
procedure OSCCTRL_3_interrupt;        external name 'OSCCTRL_3_interrupt';
procedure OSCCTRL_4_interrupt;        external name 'OSCCTRL_4_interrupt';
procedure OSC32KCTRL_interrupt;       external name 'OSC32KCTRL_interrupt';
procedure SUPC_0_interrupt;           external name 'SUPC_0_interrupt';
procedure SUPC_1_interrupt;           external name 'SUPC_1_interrupt';
procedure WDT_interrupt;              external name 'WDT_interrupt';
procedure RTC_interrupt;              external name 'RTC_interrupt';
procedure EIC_0_interrupt;            external name 'EIC_0_interrupt';
procedure EIC_1_interrupt;            external name 'EIC_1_interrupt';
procedure EIC_2_interrupt;            external name 'EIC_2_interrupt';
procedure EIC_3_interrupt;            external name 'EIC_3_interrupt';
procedure EIC_4_interrupt;            external name 'EIC_4_interrupt';
procedure EIC_5_interrupt;            external name 'EIC_5_interrupt';
procedure EIC_6_interrupt;            external name 'EIC_6_interrupt';
procedure EIC_7_interrupt;            external name 'EIC_7_interrupt';
procedure EIC_8_interrupt;            external name 'EIC_8_interrupt';
procedure EIC_9_interrupt;            external name 'EIC_9_interrupt';
procedure EIC_10_interrupt;           external name 'EIC_10_interrupt';
procedure EIC_11_interrupt;           external name 'EIC_11_interrupt';
procedure EIC_12_interrupt;           external name 'EIC_12_interrupt';
procedure EIC_13_interrupt;           external name 'EIC_13_interrupt';
procedure EIC_14_interrupt;           external name 'EIC_14_interrupt';
procedure EIC_15_interrupt;           external name 'EIC_15_interrupt';
procedure FREQM_interrupt;            external name 'FREQM_interrupt';
procedure NVMCTRL_0_interrupt;        external name 'NVMCTRL_0_interrupt';
procedure NVMCTRL_1_interrupt;        external name 'NVMCTRL_1_interrupt';
procedure DMAC_0_interrupt;           external name 'DMAC_0_interrupt';
procedure DMAC_1_interrupt;           external name 'DMAC_1_interrupt';
procedure DMAC_2_interrupt;           external name 'DMAC_2_interrupt';
procedure DMAC_3_interrupt;           external name 'DMAC_3_interrupt';
procedure DMAC_4_interrupt;           external name 'DMAC_4_interrupt';
procedure EVSYS_0_interrupt;          external name 'EVSYS_0_interrupt';
procedure EVSYS_1_interrupt;          external name 'EVSYS_1_interrupt';
procedure EVSYS_2_interrupt;          external name 'EVSYS_2_interrupt';
procedure EVSYS_3_interrupt;          external name 'EVSYS_3_interrupt';
procedure EVSYS_4_interrupt;          external name 'EVSYS_4_interrupt';
procedure PAC_interrupt;              external name 'PAC_interrupt';
procedure RAMECC_interrupt;           external name 'RAMECC_interrupt';
procedure SERCOM0_0_interrupt;        external name 'SERCOM0_0_interrupt';
procedure SERCOM0_1_interrupt;        external name 'SERCOM0_1_interrupt';
procedure SERCOM0_2_interrupt;        external name 'SERCOM0_2_interrupt';
procedure SERCOM0_3_interrupt;        external name 'SERCOM0_3_interrupt';
procedure SERCOM1_0_interrupt;        external name 'SERCOM1_0_interrupt';
procedure SERCOM1_1_interrupt;        external name 'SERCOM1_1_interrupt';
procedure SERCOM1_2_interrupt;        external name 'SERCOM1_2_interrupt';
procedure SERCOM1_3_interrupt;        external name 'SERCOM1_3_interrupt';
procedure SERCOM2_0_interrupt;        external name 'SERCOM2_0_interrupt';
procedure SERCOM2_1_interrupt;        external name 'SERCOM2_1_interrupt';
procedure SERCOM2_2_interrupt;        external name 'SERCOM2_2_interrupt';
procedure SERCOM2_3_interrupt;        external name 'SERCOM2_3_interrupt';
procedure SERCOM3_0_interrupt;        external name 'SERCOM3_0_interrupt';
procedure SERCOM3_1_interrupt;        external name 'SERCOM3_1_interrupt';
procedure SERCOM3_2_interrupt;        external name 'SERCOM3_2_interrupt';
procedure SERCOM3_3_interrupt;        external name 'SERCOM3_3_interrupt';
procedure SERCOM4_0_interrupt;        external name 'SERCOM4_0_interrupt';
procedure SERCOM4_1_interrupt;        external name 'SERCOM4_1_interrupt';
procedure SERCOM4_2_interrupt;        external name 'SERCOM4_2_interrupt';
procedure SERCOM4_3_interrupt;        external name 'SERCOM4_3_interrupt';
procedure SERCOM5_0_interrupt;        external name 'SERCOM5_0_interrupt';
procedure SERCOM5_1_interrupt;        external name 'SERCOM5_1_interrupt';
procedure SERCOM5_2_interrupt;        external name 'SERCOM5_2_interrupt';
procedure SERCOM5_3_interrupt;        external name 'SERCOM5_3_interrupt';
procedure SERCOM6_0_interrupt;        external name 'SERCOM6_0_interrupt';
procedure SERCOM6_1_interrupt;        external name 'SERCOM6_1_interrupt';
procedure SERCOM6_2_interrupt;        external name 'SERCOM6_2_interrupt';
procedure SERCOM6_3_interrupt;        external name 'SERCOM6_3_interrupt';
procedure SERCOM7_0_interrupt;        external name 'SERCOM7_0_interrupt';
procedure SERCOM7_1_interrupt;        external name 'SERCOM7_1_interrupt';
procedure SERCOM7_2_interrupt;        external name 'SERCOM7_2_interrupt';
procedure SERCOM7_3_interrupt;        external name 'SERCOM7_3_interrupt';
procedure USB_0_interrupt;            external name 'USB_0_interrupt';
procedure USB_1_interrupt;            external name 'USB_1_interrupt';
procedure USB_2_interrupt;            external name 'USB_2_interrupt';
procedure USB_3_interrupt;            external name 'USB_3_interrupt';
procedure TCC0_0_interrupt;           external name 'TCC0_0_interrupt';
procedure TCC0_1_interrupt;           external name 'TCC0_1_interrupt';
procedure TCC0_2_interrupt;           external name 'TCC0_2_interrupt';
procedure TCC0_3_interrupt;           external name 'TCC0_3_interrupt';
procedure TCC0_4_interrupt;           external name 'TCC0_4_interrupt';
procedure TCC0_5_interrupt;           external name 'TCC0_5_interrupt';
procedure TCC0_6_interrupt;           external name 'TCC0_6_interrupt';
procedure TCC1_0_interrupt;           external name 'TCC1_0_interrupt';
procedure TCC1_1_interrupt;           external name 'TCC1_1_interrupt';
procedure TCC1_2_interrupt;           external name 'TCC1_2_interrupt';
procedure TCC1_3_interrupt;           external name 'TCC1_3_interrupt';
procedure TCC1_4_interrupt;           external name 'TCC1_4_interrupt';
procedure TCC2_0_interrupt;           external name 'TCC2_0_interrupt';
procedure TCC2_1_interrupt;           external name 'TCC2_1_interrupt';
procedure TCC2_2_interrupt;           external name 'TCC2_2_interrupt';
procedure TCC2_3_interrupt;           external name 'TCC2_3_interrupt';
procedure TCC3_0_interrupt;           external name 'TCC3_0_interrupt';
procedure TCC3_1_interrupt;           external name 'TCC3_1_interrupt';
procedure TCC3_2_interrupt;           external name 'TCC3_2_interrupt';
procedure TCC4_0_interrupt;           external name 'TCC4_0_interrupt';
procedure TCC4_1_interrupt;           external name 'TCC4_1_interrupt';
procedure TCC4_2_interrupt;           external name 'TCC4_2_interrupt';
procedure TC0_interrupt;              external name 'TC0_interrupt';
procedure TC1_interrupt;              external name 'TC1_interrupt';
procedure TC2_interrupt;              external name 'TC2_interrupt';
procedure TC3_interrupt;              external name 'TC3_interrupt';
procedure TC4_interrupt;              external name 'TC4_interrupt';
procedure TC5_interrupt;              external name 'TC5_interrupt';
procedure TC6_interrupt;              external name 'TC6_interrupt';
procedure TC7_interrupt;              external name 'TC7_interrupt';
procedure PDEC_0_interrupt;           external name 'PDEC_0_interrupt';
procedure PDEC_1_interrupt;           external name 'PDEC_1_interrupt';
procedure PDEC_2_interrupt;           external name 'PDEC_2_interrupt';
procedure ADC0_0_interrupt;           external name 'ADC0_0_interrupt';
procedure ADC0_1_interrupt;           external name 'ADC0_1_interrupt';
procedure ADC1_0_interrupt;           external name 'ADC1_0_interrupt';
procedure ADC1_1_interrupt;           external name 'ADC1_1_interrupt';
procedure AC_interrupt;               external name 'AC_interrupt';
procedure DAC_0_interrupt;            external name 'DAC_0_interrupt';
procedure DAC_1_interrupt;            external name 'DAC_1_interrupt';
procedure DAC_2_interrupt;            external name 'DAC_2_interrupt';
procedure DAC_3_interrupt;            external name 'DAC_3_interrupt';
procedure DAC_4_interrupt;            external name 'DAC_4_interrupt';
procedure I2S_interrupt;              external name 'I2S_interrupt';
procedure PCC_interrupt;              external name 'PCC_interrupt';
procedure AES_interrupt;              external name 'AES_interrupt';
procedure TRNG_interrupt;             external name 'TRNG_interrupt';
procedure ICM_interrupt;              external name 'ICM_interrupt';
procedure PUKCC_interrupt;            external name 'PUKCC_interrupt';
procedure QSPI_interrupt;             external name 'QSPI_interrupt';
procedure SDHC0_interrupt;            external name 'SDHC0_interrupt';
procedure SDHC1_interrupt;            external name 'SDHC1_interrupt';

{$i    cortexm4f_start.inc}

procedure Vectors; assembler; nostackframe;
label interrupt_vectors;
asm
  .section ".init.interrupt_vectors"
  interrupt_vectors:
  .long _stack_top
  .long Startup
  .long NonMaskableInt_interrupt;
  .long HardFault_interrupt;
  .long MemoryManagement_interrupt;
  .long BusFault_interrupt;
  .long UsageFault_interrupt;
  .long 0
  .long 0
  .long 0
  .long 0
  .long SVCall_interrupt;
  .long DebugMonitor_interrupt;
  .long 0
  .long PendSV_interrupt;
  .long SysTick_interrupt;
  .long PM_interrupt;
  .long MCLK_interrupt;
  .long OSCCTRL_0_interrupt;
  .long OSCCTRL_1_interrupt;
  .long OSCCTRL_2_interrupt;
  .long OSCCTRL_3_interrupt;
  .long OSCCTRL_4_interrupt;
  .long OSC32KCTRL_interrupt;
  .long SUPC_0_interrupt;
  .long SUPC_1_interrupt;
  .long WDT_interrupt;
  .long RTC_interrupt;
  .long EIC_0_interrupt;
  .long EIC_1_interrupt;
  .long EIC_2_interrupt;
  .long EIC_3_interrupt;
  .long EIC_4_interrupt;
  .long EIC_5_interrupt;
  .long EIC_6_interrupt;
  .long EIC_7_interrupt;
  .long EIC_8_interrupt;
  .long EIC_9_interrupt;
  .long EIC_10_interrupt;
  .long EIC_11_interrupt;
  .long EIC_12_interrupt;
  .long EIC_13_interrupt;
  .long EIC_14_interrupt;
  .long EIC_15_interrupt;
  .long FREQM_interrupt;
  .long NVMCTRL_0_interrupt;
  .long NVMCTRL_1_interrupt;
  .long DMAC_0_interrupt;
  .long DMAC_1_interrupt;
  .long DMAC_2_interrupt;
  .long DMAC_3_interrupt;
  .long DMAC_4_interrupt;
  .long EVSYS_0_interrupt;
  .long EVSYS_1_interrupt;
  .long EVSYS_2_interrupt;
  .long EVSYS_3_interrupt;
  .long EVSYS_4_interrupt;
  .long PAC_interrupt;
  .long 0
  .long 0
  .long 0
  .long RAMECC_interrupt;
  .long SERCOM0_0_interrupt;
  .long SERCOM0_1_interrupt;
  .long SERCOM0_2_interrupt;
  .long SERCOM0_3_interrupt;
  .long SERCOM1_0_interrupt;
  .long SERCOM1_1_interrupt;
  .long SERCOM1_2_interrupt;
  .long SERCOM1_3_interrupt;
  .long SERCOM2_0_interrupt;
  .long SERCOM2_1_interrupt;
  .long SERCOM2_2_interrupt;
  .long SERCOM2_3_interrupt;
  .long SERCOM3_0_interrupt;
  .long SERCOM3_1_interrupt;
  .long SERCOM3_2_interrupt;
  .long SERCOM3_3_interrupt;
  .long SERCOM4_0_interrupt;
  .long SERCOM4_1_interrupt;
  .long SERCOM4_2_interrupt;
  .long SERCOM4_3_interrupt;
  .long SERCOM5_0_interrupt;
  .long SERCOM5_1_interrupt;
  .long SERCOM5_2_interrupt;
  .long SERCOM5_3_interrupt;
  .long SERCOM6_0_interrupt;
  .long SERCOM6_1_interrupt;
  .long SERCOM6_2_interrupt;
  .long SERCOM6_3_interrupt;
  .long SERCOM7_0_interrupt;
  .long SERCOM7_1_interrupt;
  .long SERCOM7_2_interrupt;
  .long SERCOM7_3_interrupt;
  .long 0
  .long 0
  .long USB_0_interrupt;
  .long USB_1_interrupt;
  .long USB_2_interrupt;
  .long USB_3_interrupt;
  .long 0
  .long TCC0_0_interrupt;
  .long TCC0_1_interrupt;
  .long TCC0_2_interrupt;
  .long TCC0_3_interrupt;
  .long TCC0_4_interrupt;
  .long TCC0_5_interrupt;
  .long TCC0_6_interrupt;
  .long TCC1_0_interrupt;
  .long TCC1_1_interrupt;
  .long TCC1_2_interrupt;
  .long TCC1_3_interrupt;
  .long TCC1_4_interrupt;
  .long TCC2_0_interrupt;
  .long TCC2_1_interrupt;
  .long TCC2_2_interrupt;
  .long TCC2_3_interrupt;
  .long TCC3_0_interrupt;
  .long TCC3_1_interrupt;
  .long TCC3_2_interrupt;
  .long TCC4_0_interrupt;
  .long TCC4_1_interrupt;
  .long TCC4_2_interrupt;
  .long TC0_interrupt;
  .long TC1_interrupt;
  .long TC2_interrupt;
  .long TC3_interrupt;
  .long TC4_interrupt;
  .long TC5_interrupt;
  .long TC6_interrupt;
  .long TC7_interrupt;
  .long PDEC_0_interrupt;
  .long PDEC_1_interrupt;
  .long PDEC_2_interrupt;
  .long ADC0_0_interrupt;
  .long ADC0_1_interrupt;
  .long ADC1_0_interrupt;
  .long ADC1_1_interrupt;
  .long AC_interrupt;
  .long DAC_0_interrupt;
  .long DAC_1_interrupt;
  .long DAC_2_interrupt;
  .long DAC_3_interrupt;
  .long DAC_4_interrupt;
  .long I2S_interrupt;
  .long PCC_interrupt;
  .long AES_interrupt;
  .long TRNG_interrupt;
  .long ICM_interrupt;
  .long PUKCC_interrupt;
  .long QSPI_interrupt;
  .long SDHC0_interrupt;
  .long SDHC1_interrupt;

  .weak NonMaskableInt_interrupt;
  .weak HardFault_interrupt;
  .weak MemoryManagement_interrupt;
  .weak BusFault_interrupt;
  .weak UsageFault_interrupt;
  .weak SVCall_interrupt;
  .weak DebugMonitor_interrupt;
  .weak PendSV_interrupt;
  .weak SysTick_interrupt;
  .weak PM_interrupt;
  .weak MCLK_interrupt;
  .weak OSCCTRL_0_interrupt;
  .weak OSCCTRL_1_interrupt;
  .weak OSCCTRL_2_interrupt;
  .weak OSCCTRL_3_interrupt;
  .weak OSCCTRL_4_interrupt;
  .weak OSC32KCTRL_interrupt;
  .weak SUPC_0_interrupt;
  .weak SUPC_1_interrupt;
  .weak WDT_interrupt;
  .weak RTC_interrupt;
  .weak EIC_0_interrupt;
  .weak EIC_1_interrupt;
  .weak EIC_2_interrupt;
  .weak EIC_3_interrupt;
  .weak EIC_4_interrupt;
  .weak EIC_5_interrupt;
  .weak EIC_6_interrupt;
  .weak EIC_7_interrupt;
  .weak EIC_8_interrupt;
  .weak EIC_9_interrupt;
  .weak EIC_10_interrupt;
  .weak EIC_11_interrupt;
  .weak EIC_12_interrupt;
  .weak EIC_13_interrupt;
  .weak EIC_14_interrupt;
  .weak EIC_15_interrupt;
  .weak FREQM_interrupt;
  .weak NVMCTRL_0_interrupt;
  .weak NVMCTRL_1_interrupt;
  .weak DMAC_0_interrupt;
  .weak DMAC_1_interrupt;
  .weak DMAC_2_interrupt;
  .weak DMAC_3_interrupt;
  .weak DMAC_4_interrupt;
  .weak EVSYS_0_interrupt;
  .weak EVSYS_1_interrupt;
  .weak EVSYS_2_interrupt;
  .weak EVSYS_3_interrupt;
  .weak EVSYS_4_interrupt;
  .weak PAC_interrupt;
  .weak RAMECC_interrupt;
  .weak SERCOM0_0_interrupt;
  .weak SERCOM0_1_interrupt;
  .weak SERCOM0_2_interrupt;
  .weak SERCOM0_3_interrupt;
  .weak SERCOM1_0_interrupt;
  .weak SERCOM1_1_interrupt;
  .weak SERCOM1_2_interrupt;
  .weak SERCOM1_3_interrupt;
  .weak SERCOM2_0_interrupt;
  .weak SERCOM2_1_interrupt;
  .weak SERCOM2_2_interrupt;
  .weak SERCOM2_3_interrupt;
  .weak SERCOM3_0_interrupt;
  .weak SERCOM3_1_interrupt;
  .weak SERCOM3_2_interrupt;
  .weak SERCOM3_3_interrupt;
  .weak SERCOM4_0_interrupt;
  .weak SERCOM4_1_interrupt;
  .weak SERCOM4_2_interrupt;
  .weak SERCOM4_3_interrupt;
  .weak SERCOM5_0_interrupt;
  .weak SERCOM5_1_interrupt;
  .weak SERCOM5_2_interrupt;
  .weak SERCOM5_3_interrupt;
  .weak SERCOM6_0_interrupt;
  .weak SERCOM6_1_interrupt;
  .weak SERCOM6_2_interrupt;
  .weak SERCOM6_3_interrupt;
  .weak SERCOM7_0_interrupt;
  .weak SERCOM7_1_interrupt;
  .weak SERCOM7_2_interrupt;
  .weak SERCOM7_3_interrupt;
  .weak USB_0_interrupt;
  .weak USB_1_interrupt;
  .weak USB_2_interrupt;
  .weak USB_3_interrupt;
  .weak TCC0_0_interrupt;
  .weak TCC0_1_interrupt;
  .weak TCC0_2_interrupt;
  .weak TCC0_3_interrupt;
  .weak TCC0_4_interrupt;
  .weak TCC0_5_interrupt;
  .weak TCC0_6_interrupt;
  .weak TCC1_0_interrupt;
  .weak TCC1_1_interrupt;
  .weak TCC1_2_interrupt;
  .weak TCC1_3_interrupt;
  .weak TCC1_4_interrupt;
  .weak TCC2_0_interrupt;
  .weak TCC2_1_interrupt;
  .weak TCC2_2_interrupt;
  .weak TCC2_3_interrupt;
  .weak TCC3_0_interrupt;
  .weak TCC3_1_interrupt;
  .weak TCC3_2_interrupt;
  .weak TCC4_0_interrupt;
  .weak TCC4_1_interrupt;
  .weak TCC4_2_interrupt;
  .weak TC0_interrupt;
  .weak TC1_interrupt;
  .weak TC2_interrupt;
  .weak TC3_interrupt;
  .weak TC4_interrupt;
  .weak TC5_interrupt;
  .weak TC6_interrupt;
  .weak TC7_interrupt;
  .weak PDEC_0_interrupt;
  .weak PDEC_1_interrupt;
  .weak PDEC_2_interrupt;
  .weak ADC0_0_interrupt;
  .weak ADC0_1_interrupt;
  .weak ADC1_0_interrupt;
  .weak ADC1_1_interrupt;
  .weak AC_interrupt;
  .weak DAC_0_interrupt;
  .weak DAC_1_interrupt;
  .weak DAC_2_interrupt;
  .weak DAC_3_interrupt;
  .weak DAC_4_interrupt;
  .weak I2S_interrupt;
  .weak PCC_interrupt;
  .weak AES_interrupt;
  .weak TRNG_interrupt;
  .weak ICM_interrupt;
  .weak PUKCC_interrupt;
  .weak QSPI_interrupt;
  .weak SDHC0_interrupt;
  .weak SDHC1_interrupt;

  .set NonMaskableInt_interrupt,  Haltproc
  .set HardFault_interrupt,       Haltproc
  .set MemoryManagement_interrupt,Haltproc
  .set BusFault_interrupt,        Haltproc
  .set UsageFault_interrupt,      Haltproc
  .set SVCall_interrupt,          Haltproc
  .set DebugMonitor_interrupt,    Haltproc
  .set PendSV_interrupt,          Haltproc
  .set SysTick_interrupt,         Haltproc
  .set PM_interrupt,              Haltproc
  .set MCLK_interrupt,            Haltproc
  .set OSCCTRL_0_interrupt,       Haltproc
  .set OSCCTRL_1_interrupt,       Haltproc
  .set OSCCTRL_2_interrupt,       Haltproc
  .set OSCCTRL_3_interrupt,       Haltproc
  .set OSCCTRL_4_interrupt,       Haltproc
  .set OSC32KCTRL_interrupt,      Haltproc
  .set SUPC_0_interrupt,          Haltproc
  .set SUPC_1_interrupt,          Haltproc
  .set WDT_interrupt,             Haltproc
  .set RTC_interrupt,             Haltproc
  .set EIC_0_interrupt,           Haltproc
  .set EIC_1_interrupt,           Haltproc
  .set EIC_2_interrupt,           Haltproc
  .set EIC_3_interrupt,           Haltproc
  .set EIC_4_interrupt,           Haltproc
  .set EIC_5_interrupt,           Haltproc
  .set EIC_6_interrupt,           Haltproc
  .set EIC_7_interrupt,           Haltproc
  .set EIC_8_interrupt,           Haltproc
  .set EIC_9_interrupt,           Haltproc
  .set EIC_10_interrupt,          Haltproc
  .set EIC_11_interrupt,          Haltproc
  .set EIC_12_interrupt,          Haltproc
  .set EIC_13_interrupt,          Haltproc
  .set EIC_14_interrupt,          Haltproc
  .set EIC_15_interrupt,          Haltproc
  .set FREQM_interrupt,           Haltproc
  .set NVMCTRL_0_interrupt,       Haltproc
  .set NVMCTRL_1_interrupt,       Haltproc
  .set DMAC_0_interrupt,          Haltproc
  .set DMAC_1_interrupt,          Haltproc
  .set DMAC_2_interrupt,          Haltproc
  .set DMAC_3_interrupt,          Haltproc
  .set DMAC_4_interrupt,          Haltproc
  .set EVSYS_0_interrupt,         Haltproc
  .set EVSYS_1_interrupt,         Haltproc
  .set EVSYS_2_interrupt,         Haltproc
  .set EVSYS_3_interrupt,         Haltproc
  .set EVSYS_4_interrupt,         Haltproc
  .set PAC_interrupt,             Haltproc
  .set RAMECC_interrupt,          Haltproc
  .set SERCOM0_0_interrupt,       Haltproc
  .set SERCOM0_1_interrupt,       Haltproc
  .set SERCOM0_2_interrupt,       Haltproc
  .set SERCOM0_3_interrupt,       Haltproc
  .set SERCOM1_0_interrupt,       Haltproc
  .set SERCOM1_1_interrupt,       Haltproc
  .set SERCOM1_2_interrupt,       Haltproc
  .set SERCOM1_3_interrupt,       Haltproc
  .set SERCOM2_0_interrupt,       Haltproc
  .set SERCOM2_1_interrupt,       Haltproc
  .set SERCOM2_2_interrupt,       Haltproc
  .set SERCOM2_3_interrupt,       Haltproc
  .set SERCOM3_0_interrupt,       Haltproc
  .set SERCOM3_1_interrupt,       Haltproc
  .set SERCOM3_2_interrupt,       Haltproc
  .set SERCOM3_3_interrupt,       Haltproc
  .set SERCOM4_0_interrupt,       Haltproc
  .set SERCOM4_1_interrupt,       Haltproc
  .set SERCOM4_2_interrupt,       Haltproc
  .set SERCOM4_3_interrupt,       Haltproc
  .set SERCOM5_0_interrupt,       Haltproc
  .set SERCOM5_1_interrupt,       Haltproc
  .set SERCOM5_2_interrupt,       Haltproc
  .set SERCOM5_3_interrupt,       Haltproc
  .set SERCOM6_0_interrupt,       Haltproc
  .set SERCOM6_1_interrupt,       Haltproc
  .set SERCOM6_2_interrupt,       Haltproc
  .set SERCOM6_3_interrupt,       Haltproc
  .set SERCOM7_0_interrupt,       Haltproc
  .set SERCOM7_1_interrupt,       Haltproc
  .set SERCOM7_2_interrupt,       Haltproc
  .set SERCOM7_3_interrupt,       Haltproc
  .set USB_0_interrupt,           Haltproc
  .set USB_1_interrupt,           Haltproc
  .set USB_2_interrupt,           Haltproc
  .set USB_3_interrupt,           Haltproc
  .set TCC0_0_interrupt,          Haltproc
  .set TCC0_1_interrupt,          Haltproc
  .set TCC0_2_interrupt,          Haltproc
  .set TCC0_3_interrupt,          Haltproc
  .set TCC0_4_interrupt,          Haltproc
  .set TCC0_5_interrupt,          Haltproc
  .set TCC0_6_interrupt,          Haltproc
  .set TCC1_0_interrupt,          Haltproc
  .set TCC1_1_interrupt,          Haltproc
  .set TCC1_2_interrupt,          Haltproc
  .set TCC1_3_interrupt,          Haltproc
  .set TCC1_4_interrupt,          Haltproc
  .set TCC2_0_interrupt,          Haltproc
  .set TCC2_1_interrupt,          Haltproc
  .set TCC2_2_interrupt,          Haltproc
  .set TCC2_3_interrupt,          Haltproc
  .set TCC3_0_interrupt,          Haltproc
  .set TCC3_1_interrupt,          Haltproc
  .set TCC3_2_interrupt,          Haltproc
  .set TCC4_0_interrupt,          Haltproc
  .set TCC4_1_interrupt,          Haltproc
  .set TCC4_2_interrupt,          Haltproc
  .set TC0_interrupt,             Haltproc
  .set TC1_interrupt,             Haltproc
  .set TC2_interrupt,             Haltproc
  .set TC3_interrupt,             Haltproc
  .set TC4_interrupt,             Haltproc
  .set TC5_interrupt,             Haltproc
  .set TC6_interrupt,             Haltproc
  .set TC7_interrupt,             Haltproc
  .set PDEC_0_interrupt,          Haltproc
  .set PDEC_1_interrupt,          Haltproc
  .set PDEC_2_interrupt,          Haltproc
  .set ADC0_0_interrupt,          Haltproc
  .set ADC0_1_interrupt,          Haltproc
  .set ADC1_0_interrupt,          Haltproc
  .set ADC1_1_interrupt,          Haltproc
  .set AC_interrupt,              Haltproc
  .set DAC_0_interrupt,           Haltproc
  .set DAC_1_interrupt,           Haltproc
  .set DAC_2_interrupt,           Haltproc
  .set DAC_3_interrupt,           Haltproc
  .set DAC_4_interrupt,           Haltproc
  .set I2S_interrupt,             Haltproc
  .set PCC_interrupt,             Haltproc
  .set AES_interrupt,             Haltproc
  .set TRNG_interrupt,            Haltproc
  .set ICM_interrupt,             Haltproc
  .set PUKCC_interrupt,           Haltproc
  .set QSPI_interrupt,            Haltproc
  .set SDHC0_interrupt,           Haltproc
  .set SDHC1_interrupt,           Haltproc
  .text
  end;
end.
