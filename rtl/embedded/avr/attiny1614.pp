unit ATtiny1614;

{$goto on}
interface

type
  TAC = object //Analog Comparator
    CTRLA: byte;  //Control A
    Reserved1: byte;
    MUXCTRLA: byte;  //Mux Control A
    Reserved3: byte;
    Reserved4: byte;
    Reserved5: byte;
    INTCTRL: byte;  //Interrupt Control
    STATUS: byte;  //Status
  const
    // Enable
    ENABLEbm = $01;
    // AC_HYSMODE
    HYSMODEmask = $06;
    HYSMODE_OFF = $00;
    HYSMODE_10mV = $02;
    HYSMODE_25mV = $04;
    HYSMODE_50mV = $06;
    // AC_INTMODE
    INTMODEmask = $30;
    INTMODE_BOTHEDGE = $00;
    INTMODE_NEGEDGE = $20;
    INTMODE_POSEDGE = $30;
    // AC_LPMODE
    LPMODEmask = $08;
    LPMODE_DIS = $00;
    LPMODE_EN = $08;
    // Output Buffer Enable
    OUTENbm = $40;
    // Run in Standby Mode
    RUNSTDBYbm = $80;
    // Analog Comparator 0 Interrupt Enable
    CMPbm = $01;
    // Invert AC Output
    INVERTbm = $80;
    // AC_MUXNEG
    MUXNEGmask = $03;
    MUXNEG_PIN0 = $00;
    MUXNEG_PIN1 = $01;
    MUXNEG_VREF = $02;
    MUXNEG_DAC = $03;
    // AC_MUXPOS
    MUXPOSmask = $18;
    MUXPOS_PIN0 = $00;
    MUXPOS_PIN1 = $08;
    MUXPOS_PIN2 = $10;
    MUXPOS_PIN3 = $18;
    // Analog Comparator State
    STATEbm = $10;
  end;

  TADC = object //Analog to Digital Converter
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    CTRLC: byte;  //Control C
    CTRLD: byte;  //Control D
    CTRLE: byte;  //Control E
    SAMPCTRL: byte;  //Sample Control
    MUXPOS: byte;  //Positive mux input
    Reserved7: byte;
    COMMAND: byte;  //Command
    EVCTRL: byte;  //Event Control
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    DBGCTRL: byte;  //Debug Control
    TEMP: byte;  //Temporary Data
    Reserved14: byte;
    Reserved15: byte;
    RES: word;  //ADC Accumulator Result
    WINLT: word;  //Window comparator low threshold
    WINHT: word;  //Window comparator high threshold
    CALIB: byte;  //Calibration
  const
    // ADC_DUTYCYC
    DUTYCYCmask = $01;
    DUTYCYC_DUTY50 = $00;
    DUTYCYC_DUTY25 = $01;
    // Start Conversion Operation
    STCONVbm = $01;
    // ADC Enable
    ENABLEbm = $01;
    // ADC Freerun mode
    FREERUNbm = $02;
    // ADC_RESSEL
    RESSELmask = $04;
    RESSEL_10BIT = $00;
    RESSEL_8BIT = $04;
    // Run standby mode
    RUNSTBYbm = $80;
    // ADC_SAMPNUM
    SAMPNUMmask = $07;
    SAMPNUM_ACC1 = $00;
    SAMPNUM_ACC2 = $01;
    SAMPNUM_ACC4 = $02;
    SAMPNUM_ACC8 = $03;
    SAMPNUM_ACC16 = $04;
    SAMPNUM_ACC32 = $05;
    SAMPNUM_ACC64 = $06;
    // ADC_PRESC
    PRESCmask = $07;
    PRESC_DIV2 = $00;
    PRESC_DIV4 = $01;
    PRESC_DIV8 = $02;
    PRESC_DIV16 = $03;
    PRESC_DIV32 = $04;
    PRESC_DIV64 = $05;
    PRESC_DIV128 = $06;
    PRESC_DIV256 = $07;
    // ADC_REFSEL
    REFSELmask = $30;
    REFSEL_INTREF = $00;
    REFSEL_VDDREF = $10;
    REFSEL_VREFA = $20;
    // Sample Capacitance Selection
    SAMPCAPbm = $40;
    // ADC_ASDV
    ASDVmask = $10;
    ASDV_ASVOFF = $00;
    ASDV_ASVON = $10;
    // ADC_INITDLY
    INITDLYmask = $E0;
    INITDLY_DLY0 = $00;
    INITDLY_DLY16 = $20;
    INITDLY_DLY32 = $40;
    INITDLY_DLY64 = $60;
    INITDLY_DLY128 = $80;
    INITDLY_DLY256 = $A0;
    // Sampling Delay Selection
    SAMPDLY0bm = $01;
    SAMPDLY1bm = $02;
    SAMPDLY2bm = $04;
    SAMPDLY3bm = $08;
    // ADC_WINCM
    WINCMmask = $07;
    WINCM_NONE = $00;
    WINCM_BELOW = $01;
    WINCM_ABOVE = $02;
    WINCM_INSIDE = $03;
    WINCM_OUTSIDE = $04;
    // Debug run
    DBGRUNbm = $01;
    // Start Event Input Enable
    STARTEIbm = $01;
    // Result Ready Interrupt Enable
    RESRDYbm = $01;
    // Window Comparator Interrupt Enable
    WCMPbm = $02;
    // ADC_MUXPOS
    MUXPOSmask = $1F;
    MUXPOS_AIN0 = $00;
    MUXPOS_AIN1 = $01;
    MUXPOS_AIN2 = $02;
    MUXPOS_AIN3 = $03;
    MUXPOS_AIN4 = $04;
    MUXPOS_AIN5 = $05;
    MUXPOS_AIN6 = $06;
    MUXPOS_AIN7 = $07;
    MUXPOS_AIN8 = $08;
    MUXPOS_AIN9 = $09;
    MUXPOS_AIN10 = $0A;
    MUXPOS_AIN11 = $0B;
    MUXPOS_PTC = $1B;
    MUXPOS_DAC0 = $1C;
    MUXPOS_INTREF = $1D;
    MUXPOS_TEMPSENSE = $1E;
    MUXPOS_GND = $1F;
    // Sample lenght
    SAMPLEN0bm = $01;
    SAMPLEN1bm = $02;
    SAMPLEN2bm = $04;
    SAMPLEN3bm = $08;
    SAMPLEN4bm = $10;
    // Temporary
    TEMP0bm = $01;
    TEMP1bm = $02;
    TEMP2bm = $04;
    TEMP3bm = $08;
    TEMP4bm = $10;
    TEMP5bm = $20;
    TEMP6bm = $40;
    TEMP7bm = $80;
  end;

  TBOD = object //Bod interface
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    Reserved2: byte;
    Reserved3: byte;
    Reserved4: byte;
    Reserved5: byte;
    Reserved6: byte;
    Reserved7: byte;
    VLMCTRLA: byte;  //Voltage level monitor Control
    INTCTRL: byte;  //Voltage level monitor interrupt Control
    INTFLAGS: byte;  //Voltage level monitor interrupt Flags
    STATUS: byte;  //Voltage level monitor status
  const
    // BOD_ACTIVE
    ACTIVEmask = $0C;
    ACTIVE_DIS = $00;
    ACTIVE_ENABLED = $04;
    ACTIVE_SAMPLED = $08;
    ACTIVE_ENWAKE = $0C;
    // BOD_SAMPFREQ
    SAMPFREQmask = $10;
    SAMPFREQ_1KHZ = $00;
    SAMPFREQ_125HZ = $10;
    // BOD_SLEEP
    SLEEPmask = $03;
    SLEEP_DIS = $00;
    SLEEP_ENABLED = $01;
    SLEEP_SAMPLED = $02;
    // BOD_LVL
    LVLmask = $07;
    LVL_BODLEVEL0 = $00;
    LVL_BODLEVEL1 = $01;
    LVL_BODLEVEL2 = $02;
    LVL_BODLEVEL3 = $03;
    LVL_BODLEVEL4 = $04;
    LVL_BODLEVEL5 = $05;
    LVL_BODLEVEL6 = $06;
    LVL_BODLEVEL7 = $07;
    // BOD_VLMCFG
    VLMCFGmask = $06;
    VLMCFG_BELOW = $00;
    VLMCFG_ABOVE = $02;
    VLMCFG_CROSS = $04;
    // voltage level monitor interrrupt enable
    VLMIEbm = $01;
    // Voltage level monitor interrupt flag
    VLMIFbm = $01;
    // Voltage level monitor status
    VLMSbm = $01;
    // BOD_VLMLVL
    VLMLVLmask = $03;
    VLMLVL_5ABOVE = $00;
    VLMLVL_15ABOVE = $01;
    VLMLVL_25ABOVE = $02;
  end;

  TCCL = object //Configurable Custom Logic
    CTRLA: byte;  //Control Register A
    SEQCTRL0: byte;  //Sequential Control 0
    Reserved2: byte;
    Reserved3: byte;
    Reserved4: byte;
    LUT0CTRLA: byte;  //LUT Control 0 A
    LUT0CTRLB: byte;  //LUT Control 0 B
    LUT0CTRLC: byte;  //LUT Control 0 C
    TRUTH0: byte;  //Truth 0
    LUT1CTRLA: byte;  //LUT Control 1 A
    LUT1CTRLB: byte;  //LUT Control 1 B
    LUT1CTRLC: byte;  //LUT Control 1 C
    TRUTH1: byte;  //Truth 1
  const
    // Enable
    ENABLEbm = $01;
    // Run in Standby
    RUNSTDBYbm = $40;
    // Clock Source Selection
    CLKSRCbm = $40;
    // CCL_EDGEDET
    EDGEDETmask = $80;
    EDGEDET_DIS = $00;
    EDGEDET_EN = $80;
    // CCL_FILTSEL
    FILTSELmask = $30;
    FILTSEL_DISABLE = $00;
    FILTSEL_SYNCH = $10;
    FILTSEL_FILTER = $20;
    // Output Enable
    OUTENbm = $08;
    // CCL_INSEL0
    INSEL0mask = $0F;
    INSEL0_MASK = $00;
    INSEL0_FEEDBACK = $01;
    INSEL0_LINK = $02;
    INSEL0_EVENT0 = $03;
    INSEL0_EVENT1 = $04;
    INSEL0_IO = $05;
    INSEL0_AC0 = $06;
    INSEL0_TCB0 = $07;
    INSEL0_TCA0 = $08;
    INSEL0_TCD0 = $09;
    INSEL0_USART0 = $0A;
    INSEL0_SPI0 = $0B;
    // CCL_INSEL1
    INSEL1mask = $F0;
    INSEL1_MASK = $00;
    INSEL1_FEEDBACK = $10;
    INSEL1_LINK = $20;
    INSEL1_EVENT0 = $30;
    INSEL1_EVENT1 = $40;
    INSEL1_IO = $50;
    INSEL1_AC0 = $60;
    INSEL1_TCB0 = $70;
    INSEL1_TCA0 = $80;
    INSEL1_TCD0 = $90;
    INSEL1_USART0 = $A0;
    INSEL1_SPI0 = $B0;
    // CCL_INSEL2
    INSEL2mask = $0F;
    INSEL2_MASK = $00;
    INSEL2_FEEDBACK = $01;
    INSEL2_LINK = $02;
    INSEL2_EVENT0 = $03;
    INSEL2_EVENT1 = $04;
    INSEL2_IO = $05;
    INSEL2_AC0 = $06;
    INSEL2_TCB0 = $07;
    INSEL2_TCA0 = $08;
    INSEL2_TCD0 = $09;
    INSEL2_SPI0 = $0B;
    // CCL_SEQSEL
    SEQSELmask = $07;
    SEQSEL_DISABLE = $00;
    SEQSEL_DFF = $01;
    SEQSEL_JK = $02;
    SEQSEL_LATCH = $03;
    SEQSEL_RS = $04;
  end;

  TCLKCTRL = object //Clock controller
    MCLKCTRLA: byte;  //MCLK Control A
    MCLKCTRLB: byte;  //MCLK Control B
    MCLKLOCK: byte;  //MCLK Lock
    MCLKSTATUS: byte;  //MCLK Status
    Reserved4: byte;
    Reserved5: byte;
    Reserved6: byte;
    Reserved7: byte;
    Reserved8: byte;
    Reserved9: byte;
    Reserved10: byte;
    Reserved11: byte;
    Reserved12: byte;
    Reserved13: byte;
    Reserved14: byte;
    Reserved15: byte;
    OSC20MCTRLA: byte;  //OSC20M Control A
    OSC20MCALIBA: byte;  //OSC20M Calibration A
    OSC20MCALIBB: byte;  //OSC20M Calibration B
    Reserved19: byte;
    Reserved20: byte;
    Reserved21: byte;
    Reserved22: byte;
    Reserved23: byte;
    OSC32KCTRLA: byte;  //OSC32K Control A
    Reserved25: byte;
    Reserved26: byte;
    Reserved27: byte;
    XOSC32KCTRLA: byte;  //XOSC32K Control A
  const
    // System clock out
    CLKOUTbm = $80;
    // CLKCTRL_CLKSEL
    CLKSELmask = $03;
    CLKSEL_OSC20M = $00;
    CLKSEL_OSCULP32K = $01;
    CLKSEL_XOSC32K = $02;
    CLKSEL_EXTCLK = $03;
    // CLKCTRL_PDIV
    PDIVmask = $1E;
    PDIV_2X = $00;
    PDIV_4X = $02;
    PDIV_8X = $04;
    PDIV_16X = $06;
    PDIV_32X = $08;
    PDIV_64X = $0A;
    PDIV_6X = $10;
    PDIV_10X = $12;
    PDIV_12X = $14;
    PDIV_24X = $16;
    PDIV_48X = $18;
    // Prescaler enable
    PENbm = $01;
    // lock ebable
    LOCKENbm = $01;
    // External Clock status
    EXTSbm = $80;
    // 20MHz oscillator status
    OSC20MSbm = $10;
    // 32KHz oscillator status
    OSC32KSbm = $20;
    // System Oscillator changing
    SOSCbm = $01;
    // 32.768 kHz Crystal Oscillator status
    XOSC32KSbm = $40;
    // Calibration
    CAL20M0bm = $01;
    CAL20M1bm = $02;
    CAL20M2bm = $04;
    CAL20M3bm = $08;
    CAL20M4bm = $10;
    CAL20M5bm = $20;
    // Lock
    LOCKbm = $80;
    // Oscillator temperature coefficient
    TEMPCAL20M0bm = $01;
    TEMPCAL20M1bm = $02;
    TEMPCAL20M2bm = $04;
    TEMPCAL20M3bm = $08;
    // Run standby
    RUNSTDBYbm = $02;
    // CLKCTRL_CSUT
    CSUTmask = $30;
    CSUT_1K = $00;
    CSUT_16K = $10;
    CSUT_32K = $20;
    CSUT_64K = $30;
    // Enable
    ENABLEbm = $01;
    // Select
    SELbm = $04;
  end;

  TCPU = object //CPU
    Reserved0: byte;
    Reserved1: byte;
    Reserved2: byte;
    Reserved3: byte;
    CCP: byte;  //Configuration Change Protection
    Reserved5: byte;
    Reserved6: byte;
    Reserved7: byte;
    Reserved8: byte;
    Reserved9: byte;
    Reserved10: byte;
    Reserved11: byte;
    Reserved12: byte;
    SPL: byte;  //Stack Pointer Low
    SPH: byte;  //Stack Pointer High
    SREG: byte;  //Status Register
  const
    // CPU_CCP
    CCPmask = $FF;
    CCP_SPM = $9D;
    CCP_IOREG = $D8;
    // Carry Flag
    Cbm = $01;
    // Half Carry Flag
    Hbm = $20;
    // Global Interrupt Enable Flag
    Ibm = $80;
    // Negative Flag
    Nbm = $04;
    // N Exclusive Or V Flag
    Sbm = $10;
    // Transfer Bit
    Tbm = $40;
    // Two's Complement Overflow Flag
    Vbm = $08;
    // Zero Flag
    Zbm = $02;
  end;

  TCPUINT = object //Interrupt Controller
    CTRLA: byte;  //Control A
    STATUS: byte;  //Status
    LVL0PRI: byte;  //Interrupt Level 0 Priority
    LVL1VEC: byte;  //Interrupt Level 1 Priority Vector
  const
    // Compact Vector Table
    CVTbm = $20;
    // Interrupt Vector Select
    IVSELbm = $40;
    // Round-robin Scheduling Enable
    LVL0RRbm = $01;
    // Interrupt Level Priority
    LVL0PRI0bm = $01;
    LVL0PRI1bm = $02;
    LVL0PRI2bm = $04;
    LVL0PRI3bm = $08;
    LVL0PRI4bm = $10;
    LVL0PRI5bm = $20;
    LVL0PRI6bm = $40;
    LVL0PRI7bm = $80;
    // Interrupt Vector with High Priority
    LVL1VEC0bm = $01;
    LVL1VEC1bm = $02;
    LVL1VEC2bm = $04;
    LVL1VEC3bm = $08;
    LVL1VEC4bm = $10;
    LVL1VEC5bm = $20;
    LVL1VEC6bm = $40;
    LVL1VEC7bm = $80;
    // Level 0 Interrupt Executing
    LVL0EXbm = $01;
    // Level 1 Interrupt Executing
    LVL1EXbm = $02;
    // Non-maskable Interrupt Executing
    NMIEXbm = $80;
  end;

  TCRCSCAN = object //CRCSCAN
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    STATUS: byte;  //Status
  const
    // Enable CRC scan
    ENABLEbm = $01;
    // Enable NMI Trigger
    NMIENbm = $02;
    // Reset CRC scan
    RESETbm = $80;
    // CRCSCAN_MODE
    MODEmask = $30;
    MODE_PRIORITY = $00;
    MODE_RESERVED = $10;
    MODE_BACKGROUND = $20;
    MODE_CONTINUOUS = $30;
    // CRCSCAN_SRC
    SRCmask = $03;
    SRC_FLASH = $00;
    SRC_APPLICATION = $01;
    SRC_BOOT = $02;
    // CRC Busy
    BUSYbm = $01;
    // CRC Ok
    OKbm = $02;
  end;

  TDAC = object //Digital to Analog Converter
    CTRLA: byte;  //Control Register A
    DATA: byte;  //DATA Register
  const
    // DAC Enable
    ENABLEbm = $01;
    // Output Buffer Enable
    OUTENbm = $40;
    // Run in Standby Mode
    RUNSTDBYbm = $80;
  end;

  TEVSYS = object //Event System
    ASYNCSTROBE: byte;  //Asynchronous Channel Strobe
    SYNCSTROBE: byte;  //Synchronous Channel Strobe
    ASYNCCH0: byte;  //Asynchronous Channel 0 Generator Selection
    ASYNCCH1: byte;  //Asynchronous Channel 1 Generator Selection
    ASYNCCH2: byte;  //Asynchronous Channel 2 Generator Selection
    ASYNCCH3: byte;  //Asynchronous Channel 3 Generator Selection
    Reserved6: byte;
    Reserved7: byte;
    Reserved8: byte;
    Reserved9: byte;
    SYNCCH0: byte;  //Synchronous Channel 0 Generator Selection
    SYNCCH1: byte;  //Synchronous Channel 1 Generator Selection
    Reserved12: byte;
    Reserved13: byte;
    Reserved14: byte;
    Reserved15: byte;
    Reserved16: byte;
    Reserved17: byte;
    ASYNCUSER0: byte;  //Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER1: byte;  //Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER2: byte;  //Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER3: byte;  //Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER4: byte;  //Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER5: byte;  //Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER6: byte;  //Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER7: byte;  //Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER8: byte;  //Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER9: byte;  //Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER10: byte;  //Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER11: byte;  //Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER12: byte;  //Asynchronous User Ch 12 Input Selection - ADC1
    Reserved31: byte;
    Reserved32: byte;
    Reserved33: byte;
    SYNCUSER0: byte;  //Synchronous User Ch 0 - TCA0
    SYNCUSER1: byte;  //Synchronous User Ch 1 - USART0
  const
    // EVSYS_ASYNCCH0
    ASYNCCH0mask = $FF;
    ASYNCCH0_OFF = $00;
    ASYNCCH0_CCL_LUT0 = $01;
    ASYNCCH0_CCL_LUT1 = $02;
    ASYNCCH0_AC0_OUT = $03;
    ASYNCCH0_TCD0_CMPBCLR = $04;
    ASYNCCH0_TCD0_CMPASET = $05;
    ASYNCCH0_TCD0_CMPBSET = $06;
    ASYNCCH0_TCD0_PROGEV = $07;
    ASYNCCH0_RTC_OVF = $08;
    ASYNCCH0_RTC_CMP = $09;
    ASYNCCH0_PORTA_PIN0 = $0A;
    ASYNCCH0_PORTA_PIN1 = $0B;
    ASYNCCH0_PORTA_PIN2 = $0C;
    ASYNCCH0_PORTA_PIN3 = $0D;
    ASYNCCH0_PORTA_PIN4 = $0E;
    ASYNCCH0_PORTA_PIN5 = $0F;
    ASYNCCH0_PORTA_PIN6 = $10;
    ASYNCCH0_PORTA_PIN7 = $11;
    ASYNCCH0_UPDI = $12;
    ASYNCCH0_AC1_OUT = $13;
    ASYNCCH0_AC2_OUT = $14;
    // EVSYS_ASYNCCH1
    ASYNCCH1mask = $FF;
    ASYNCCH1_OFF = $00;
    ASYNCCH1_CCL_LUT0 = $01;
    ASYNCCH1_CCL_LUT1 = $02;
    ASYNCCH1_AC0_OUT = $03;
    ASYNCCH1_TCD0_CMPBCLR = $04;
    ASYNCCH1_TCD0_CMPASET = $05;
    ASYNCCH1_TCD0_CMPBSET = $06;
    ASYNCCH1_TCD0_PROGEV = $07;
    ASYNCCH1_RTC_OVF = $08;
    ASYNCCH1_RTC_CMP = $09;
    ASYNCCH1_PORTB_PIN0 = $0A;
    ASYNCCH1_PORTB_PIN1 = $0B;
    ASYNCCH1_PORTB_PIN2 = $0C;
    ASYNCCH1_PORTB_PIN3 = $0D;
    ASYNCCH1_PORTB_PIN4 = $0E;
    ASYNCCH1_PORTB_PIN5 = $0F;
    ASYNCCH1_PORTB_PIN6 = $10;
    ASYNCCH1_PORTB_PIN7 = $11;
    ASYNCCH1_AC1_OUT = $12;
    ASYNCCH1_AC2_OUT = $13;
    // EVSYS_ASYNCCH2
    ASYNCCH2mask = $FF;
    ASYNCCH2_OFF = $00;
    ASYNCCH2_CCL_LUT0 = $01;
    ASYNCCH2_CCL_LUT1 = $02;
    ASYNCCH2_AC0_OUT = $03;
    ASYNCCH2_TCD0_CMPBCLR = $04;
    ASYNCCH2_TCD0_CMPASET = $05;
    ASYNCCH2_TCD0_CMPBSET = $06;
    ASYNCCH2_TCD0_PROGEV = $07;
    ASYNCCH2_RTC_OVF = $08;
    ASYNCCH2_RTC_CMP = $09;
    ASYNCCH2_PORTC_PIN0 = $0A;
    ASYNCCH2_PORTC_PIN1 = $0B;
    ASYNCCH2_PORTC_PIN2 = $0C;
    ASYNCCH2_PORTC_PIN3 = $0D;
    ASYNCCH2_PORTC_PIN4 = $0E;
    ASYNCCH2_PORTC_PIN5 = $0F;
    ASYNCCH2_AC1_OUT = $10;
    ASYNCCH2_AC2_OUT = $11;
    // EVSYS_ASYNCCH3
    ASYNCCH3mask = $FF;
    ASYNCCH3_OFF = $00;
    ASYNCCH3_CCL_LUT0 = $01;
    ASYNCCH3_CCL_LUT1 = $02;
    ASYNCCH3_AC0_OUT = $03;
    ASYNCCH3_TCD0_CMPBCLR = $04;
    ASYNCCH3_TCD0_CMPASET = $05;
    ASYNCCH3_TCD0_CMPBSET = $06;
    ASYNCCH3_TCD0_PROGEV = $07;
    ASYNCCH3_RTC_OVF = $08;
    ASYNCCH3_RTC_CMP = $09;
    ASYNCCH3_PIT_DIV8192 = $0A;
    ASYNCCH3_PIT_DIV4096 = $0B;
    ASYNCCH3_PIT_DIV2048 = $0C;
    ASYNCCH3_PIT_DIV1024 = $0D;
    ASYNCCH3_PIT_DIV512 = $0E;
    ASYNCCH3_PIT_DIV256 = $0F;
    ASYNCCH3_PIT_DIV128 = $10;
    ASYNCCH3_PIT_DIV64 = $11;
    ASYNCCH3_AC1_OUT = $12;
    ASYNCCH3_AC2_OUT = $13;
    // EVSYS_ASYNCUSER0
    ASYNCUSER0mask = $FF;
    ASYNCUSER0_OFF = $00;
    ASYNCUSER0_SYNCCH0 = $01;
    ASYNCUSER0_SYNCCH1 = $02;
    ASYNCUSER0_ASYNCCH0 = $03;
    ASYNCUSER0_ASYNCCH1 = $04;
    ASYNCUSER0_ASYNCCH2 = $05;
    ASYNCUSER0_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER1
    ASYNCUSER1mask = $FF;
    ASYNCUSER1_OFF = $00;
    ASYNCUSER1_SYNCCH0 = $01;
    ASYNCUSER1_SYNCCH1 = $02;
    ASYNCUSER1_ASYNCCH0 = $03;
    ASYNCUSER1_ASYNCCH1 = $04;
    ASYNCUSER1_ASYNCCH2 = $05;
    ASYNCUSER1_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER2
    ASYNCUSER2mask = $FF;
    ASYNCUSER2_OFF = $00;
    ASYNCUSER2_SYNCCH0 = $01;
    ASYNCUSER2_SYNCCH1 = $02;
    ASYNCUSER2_ASYNCCH0 = $03;
    ASYNCUSER2_ASYNCCH1 = $04;
    ASYNCUSER2_ASYNCCH2 = $05;
    ASYNCUSER2_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER3
    ASYNCUSER3mask = $FF;
    ASYNCUSER3_OFF = $00;
    ASYNCUSER3_SYNCCH0 = $01;
    ASYNCUSER3_SYNCCH1 = $02;
    ASYNCUSER3_ASYNCCH0 = $03;
    ASYNCUSER3_ASYNCCH1 = $04;
    ASYNCUSER3_ASYNCCH2 = $05;
    ASYNCUSER3_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER4
    ASYNCUSER4mask = $FF;
    ASYNCUSER4_OFF = $00;
    ASYNCUSER4_SYNCCH0 = $01;
    ASYNCUSER4_SYNCCH1 = $02;
    ASYNCUSER4_ASYNCCH0 = $03;
    ASYNCUSER4_ASYNCCH1 = $04;
    ASYNCUSER4_ASYNCCH2 = $05;
    ASYNCUSER4_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER5
    ASYNCUSER5mask = $FF;
    ASYNCUSER5_OFF = $00;
    ASYNCUSER5_SYNCCH0 = $01;
    ASYNCUSER5_SYNCCH1 = $02;
    ASYNCUSER5_ASYNCCH0 = $03;
    ASYNCUSER5_ASYNCCH1 = $04;
    ASYNCUSER5_ASYNCCH2 = $05;
    ASYNCUSER5_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER6
    ASYNCUSER6mask = $FF;
    ASYNCUSER6_OFF = $00;
    ASYNCUSER6_SYNCCH0 = $01;
    ASYNCUSER6_SYNCCH1 = $02;
    ASYNCUSER6_ASYNCCH0 = $03;
    ASYNCUSER6_ASYNCCH1 = $04;
    ASYNCUSER6_ASYNCCH2 = $05;
    ASYNCUSER6_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER7
    ASYNCUSER7mask = $FF;
    ASYNCUSER7_OFF = $00;
    ASYNCUSER7_SYNCCH0 = $01;
    ASYNCUSER7_SYNCCH1 = $02;
    ASYNCUSER7_ASYNCCH0 = $03;
    ASYNCUSER7_ASYNCCH1 = $04;
    ASYNCUSER7_ASYNCCH2 = $05;
    ASYNCUSER7_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER8
    ASYNCUSER8mask = $FF;
    ASYNCUSER8_OFF = $00;
    ASYNCUSER8_SYNCCH0 = $01;
    ASYNCUSER8_SYNCCH1 = $02;
    ASYNCUSER8_ASYNCCH0 = $03;
    ASYNCUSER8_ASYNCCH1 = $04;
    ASYNCUSER8_ASYNCCH2 = $05;
    ASYNCUSER8_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER9
    ASYNCUSER9mask = $FF;
    ASYNCUSER9_OFF = $00;
    ASYNCUSER9_SYNCCH0 = $01;
    ASYNCUSER9_SYNCCH1 = $02;
    ASYNCUSER9_ASYNCCH0 = $03;
    ASYNCUSER9_ASYNCCH1 = $04;
    ASYNCUSER9_ASYNCCH2 = $05;
    ASYNCUSER9_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER10
    ASYNCUSER10mask = $FF;
    ASYNCUSER10_OFF = $00;
    ASYNCUSER10_SYNCCH0 = $01;
    ASYNCUSER10_SYNCCH1 = $02;
    ASYNCUSER10_ASYNCCH0 = $03;
    ASYNCUSER10_ASYNCCH1 = $04;
    ASYNCUSER10_ASYNCCH2 = $05;
    ASYNCUSER10_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER11
    ASYNCUSER11mask = $FF;
    ASYNCUSER11_OFF = $00;
    ASYNCUSER11_SYNCCH0 = $01;
    ASYNCUSER11_SYNCCH1 = $02;
    ASYNCUSER11_ASYNCCH0 = $03;
    ASYNCUSER11_ASYNCCH1 = $04;
    ASYNCUSER11_ASYNCCH2 = $05;
    ASYNCUSER11_ASYNCCH3 = $06;
    // EVSYS_ASYNCUSER12
    ASYNCUSER12mask = $FF;
    ASYNCUSER12_OFF = $00;
    ASYNCUSER12_SYNCCH0 = $01;
    ASYNCUSER12_SYNCCH1 = $02;
    ASYNCUSER12_ASYNCCH0 = $03;
    ASYNCUSER12_ASYNCCH1 = $04;
    ASYNCUSER12_ASYNCCH2 = $05;
    ASYNCUSER12_ASYNCCH3 = $06;
    // EVSYS_SYNCCH0
    SYNCCH0mask = $FF;
    SYNCCH0_OFF = $00;
    SYNCCH0_TCB0 = $01;
    SYNCCH0_TCA0_OVF_LUNF = $02;
    SYNCCH0_TCA0_HUNF = $03;
    SYNCCH0_TCA0_CMP0 = $04;
    SYNCCH0_TCA0_CMP1 = $05;
    SYNCCH0_TCA0_CMP2 = $06;
    SYNCCH0_PORTC_PIN0 = $07;
    SYNCCH0_PORTC_PIN1 = $08;
    SYNCCH0_PORTC_PIN2 = $09;
    SYNCCH0_PORTC_PIN3 = $0A;
    SYNCCH0_PORTC_PIN4 = $0B;
    SYNCCH0_PORTC_PIN5 = $0C;
    SYNCCH0_PORTA_PIN0 = $0D;
    SYNCCH0_PORTA_PIN1 = $0E;
    SYNCCH0_PORTA_PIN2 = $0F;
    SYNCCH0_PORTA_PIN3 = $10;
    SYNCCH0_PORTA_PIN4 = $11;
    SYNCCH0_PORTA_PIN5 = $12;
    SYNCCH0_PORTA_PIN6 = $13;
    SYNCCH0_PORTA_PIN7 = $14;
    SYNCCH0_TCB1 = $15;
    // EVSYS_SYNCCH1
    SYNCCH1mask = $FF;
    SYNCCH1_OFF = $00;
    SYNCCH1_TCB0 = $01;
    SYNCCH1_TCA0_OVF_LUNF = $02;
    SYNCCH1_TCA0_HUNF = $03;
    SYNCCH1_TCA0_CMP0 = $04;
    SYNCCH1_TCA0_CMP1 = $05;
    SYNCCH1_TCA0_CMP2 = $06;
    SYNCCH1_PORTB_PIN0 = $08;
    SYNCCH1_PORTB_PIN1 = $09;
    SYNCCH1_PORTB_PIN2 = $0A;
    SYNCCH1_PORTB_PIN3 = $0B;
    SYNCCH1_PORTB_PIN4 = $0C;
    SYNCCH1_PORTB_PIN5 = $0D;
    SYNCCH1_PORTB_PIN6 = $0E;
    SYNCCH1_PORTB_PIN7 = $0F;
    SYNCCH1_TCB1 = $10;
    // EVSYS_SYNCUSER0
    SYNCUSER0mask = $FF;
    SYNCUSER0_OFF = $00;
    SYNCUSER0_SYNCCH0 = $01;
    SYNCUSER0_SYNCCH1 = $02;
    // EVSYS_SYNCUSER1
    SYNCUSER1mask = $FF;
    SYNCUSER1_OFF = $00;
    SYNCUSER1_SYNCCH0 = $01;
    SYNCUSER1_SYNCCH1 = $02;
  end;

  TFUSE = object //Fuses
    WDTCFG: byte;  //Watchdog Configuration
    BODCFG: byte;  //BOD Configuration
    OSCCFG: byte;  //Oscillator Configuration
    Reserved3: byte;
    TCD0CFG: byte;  //TCD0 Configuration
    SYSCFG0: byte;  //System Configuration 0
    SYSCFG1: byte;  //System Configuration 1
    APPEND: byte;  //Application Code Section End
    BOOTEND: byte;  //Boot Section End
  const
    // FUSE_ACTIVE
    ACTIVEmask = $0C;
    ACTIVE_DIS = $00;
    ACTIVE_ENABLED = $04;
    ACTIVE_SAMPLED = $08;
    ACTIVE_ENWAKE = $0C;
    // FUSE_LVL
    LVLmask = $E0;
    LVL_BODLEVEL0 = $00;
    LVL_BODLEVEL1 = $20;
    LVL_BODLEVEL2 = $40;
    LVL_BODLEVEL3 = $60;
    LVL_BODLEVEL4 = $80;
    LVL_BODLEVEL5 = $A0;
    LVL_BODLEVEL6 = $C0;
    LVL_BODLEVEL7 = $E0;
    // FUSE_SAMPFREQ
    SAMPFREQmask = $10;
    SAMPFREQ_1KHZ = $00;
    SAMPFREQ_125HZ = $10;
    // FUSE_SLEEP
    SLEEPmask = $03;
    SLEEP_DIS = $00;
    SLEEP_ENABLED = $01;
    SLEEP_SAMPLED = $02;
    // FUSE_FREQSEL
    FREQSELmask = $03;
    FREQSEL_16MHZ = $01;
    FREQSEL_20MHZ = $02;
    // Oscillator Lock
    OSCLOCKbm = $80;
    // FUSE_CRCSRC
    CRCSRCmask = $C0;
    CRCSRC_FLASH = $00;
    CRCSRC_BOOT = $40;
    CRCSRC_BOOTAPP = $80;
    CRCSRC_NOCRC = $C0;
    // EEPROM Save
    EESAVEbm = $01;
    // FUSE_RSTPINCFG
    RSTPINCFGmask = $0C;
    RSTPINCFG_GPIO = $00;
    RSTPINCFG_UPDI = $04;
    RSTPINCFG_RST = $08;
    // FUSE_SUT
    SUTmask = $07;
    SUT_0MS = $00;
    SUT_1MS = $01;
    SUT_2MS = $02;
    SUT_4MS = $03;
    SUT_8MS = $04;
    SUT_16MS = $05;
    SUT_32MS = $06;
    SUT_64MS = $07;
    // Compare A Default Output Value
    CMPAbm = $01;
    // Compare A Output Enable
    CMPAENbm = $10;
    // Compare B Default Output Value
    CMPBbm = $02;
    // Compare B Output Enable
    CMPBENbm = $20;
    // Compare C Default Output Value
    CMPCbm = $04;
    // Compare C Output Enable
    CMPCENbm = $40;
    // Compare D Default Output Value
    CMPDbm = $08;
    // Compare D Output Enable
    CMPDENbm = $80;
    // FUSE_PERIOD
    PERIODmask = $0F;
    PERIOD_OFF = $00;
    PERIOD_8CLK = $01;
    PERIOD_16CLK = $02;
    PERIOD_32CLK = $03;
    PERIOD_64CLK = $04;
    PERIOD_128CLK = $05;
    PERIOD_256CLK = $06;
    PERIOD_512CLK = $07;
    PERIOD_1KCLK = $08;
    PERIOD_2KCLK = $09;
    PERIOD_4KCLK = $0A;
    PERIOD_8KCLK = $0B;
    // FUSE_WINDOW
    WINDOWmask = $F0;
    WINDOW_OFF = $00;
    WINDOW_8CLK = $10;
    WINDOW_16CLK = $20;
    WINDOW_32CLK = $30;
    WINDOW_64CLK = $40;
    WINDOW_128CLK = $50;
    WINDOW_256CLK = $60;
    WINDOW_512CLK = $70;
    WINDOW_1KCLK = $80;
    WINDOW_2KCLK = $90;
    WINDOW_4KCLK = $A0;
    WINDOW_8KCLK = $B0;
  end;

  TGPIO = object //General Purpose IO
    GPIOR0: byte;  //General Purpose IO Register 0
    GPIOR1: byte;  //General Purpose IO Register 1
    GPIOR2: byte;  //General Purpose IO Register 2
    GPIOR3: byte;  //General Purpose IO Register 3
  end;

  TLOCKBIT = object //Lockbit
    LOCKBIT: byte;  //Lock bits
  const
    // LOCKBIT_LB
    LBmask = $FF;
    LB_RWLOCK = $3A;
    LB_NOLOCK = $C5;
  end;

  TNVMCTRL = object //Non-volatile Memory Controller
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    STATUS: byte;  //Status
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    Reserved5: byte;
    DATA: word;  //Data
    ADDR: word;  //Address
  const
    // NVMCTRL_CMD
    CMDmask = $07;
    CMD_NONE = $00;
    CMD_PAGEWRITE = $01;
    CMD_PAGEERASE = $02;
    CMD_PAGEERASEWRITE = $03;
    CMD_PAGEBUFCLR = $04;
    CMD_CHIPERASE = $05;
    CMD_EEERASE = $06;
    CMD_FUSEWRITE = $07;
    // Application code write protect
    APCWPbm = $01;
    // Boot Lock
    BOOTLOCKbm = $02;
    // EEPROM Ready
    EEREADYbm = $01;
    // EEPROM busy
    EEBUSYbm = $02;
    // Flash busy
    FBUSYbm = $01;
    // Write error
    WRERRORbm = $04;
  end;

  TPORT = object //I/O Ports
    DIR: byte;  //Data Direction
    DIRSET: byte;  //Data Direction Set
    DIRCLR: byte;  //Data Direction Clear
    DIRTGL: byte;  //Data Direction Toggle
    OUT_: byte;  //Output Value
    OUTSET: byte;  //Output Value Set
    OUTCLR: byte;  //Output Value Clear
    OUTTGL: byte;  //Output Value Toggle
    IN_: byte;  //Input Value
    INTFLAGS: byte;  //Interrupt Flags
    Reserved10: byte;
    Reserved11: byte;
    Reserved12: byte;
    Reserved13: byte;
    Reserved14: byte;
    Reserved15: byte;
    PIN0CTRL: byte;  //Pin 0 Control
    PIN1CTRL: byte;  //Pin 1 Control
    PIN2CTRL: byte;  //Pin 2 Control
    PIN3CTRL: byte;  //Pin 3 Control
    PIN4CTRL: byte;  //Pin 4 Control
    PIN5CTRL: byte;  //Pin 5 Control
    PIN6CTRL: byte;  //Pin 6 Control
    PIN7CTRL: byte;  //Pin 7 Control
  const
    // Pin Interrupt
    INT0bm = $01;
    INT1bm = $02;
    INT2bm = $04;
    INT3bm = $08;
    INT4bm = $10;
    INT5bm = $20;
    INT6bm = $40;
    INT7bm = $80;
    // Inverted I/O Enable
    INVENbm = $80;
    // PORT_ISC
    ISCmask = $07;
    ISC_INTDISABLE = $00;
    ISC_BOTHEDGES = $01;
    ISC_RISING = $02;
    ISC_FALLING = $03;
    ISC_INPUT_DISABLE = $04;
    ISC_LEVEL = $05;
    // Pullup enable
    PULLUPENbm = $08;
  end;

  TPORTMUX = object //Port Multiplexer
    CTRLA: byte;  //Port Multiplexer Control A
    CTRLB: byte;  //Port Multiplexer Control B
    CTRLC: byte;  //Port Multiplexer Control C
    CTRLD: byte;  //Port Multiplexer Control D
  const
    // Event Output 0
    EVOUT0bm = $01;
    // Event Output 1
    EVOUT1bm = $02;
    // Event Output 2
    EVOUT2bm = $04;
    // PORTMUX_LUT0
    LUT0mask = $10;
    LUT0_DEFAULT = $00;
    LUT0_ALTERNATE = $10;
    // PORTMUX_LUT1
    LUT1mask = $20;
    LUT1_DEFAULT = $00;
    LUT1_ALTERNATE = $20;
    // PORTMUX_SPI0
    SPI0mask = $04;
    SPI0_DEFAULT = $00;
    SPI0_ALTERNATE = $04;
    // PORTMUX_TWI0
    TWI0mask = $10;
    TWI0_DEFAULT = $00;
    TWI0_ALTERNATE = $10;
    // PORTMUX_USART0
    USART0mask = $01;
    USART0_DEFAULT = $00;
    USART0_ALTERNATE = $01;
    // PORTMUX_TCA00
    TCA00mask = $01;
    TCA00_DEFAULT = $00;
    TCA00_ALTERNATE = $01;
    // PORTMUX_TCA01
    TCA01mask = $02;
    TCA01_DEFAULT = $00;
    TCA01_ALTERNATE = $02;
    // PORTMUX_TCA02
    TCA02mask = $04;
    TCA02_DEFAULT = $00;
    TCA02_ALTERNATE = $04;
    // PORTMUX_TCA03
    TCA03mask = $08;
    TCA03_DEFAULT = $00;
    TCA03_ALTERNATE = $08;
    // PORTMUX_TCA04
    TCA04mask = $10;
    TCA04_DEFAULT = $00;
    TCA04_ALTERNATE = $10;
    // PORTMUX_TCA05
    TCA05mask = $20;
    TCA05_DEFAULT = $00;
    TCA05_ALTERNATE = $20;
    // PORTMUX_TCB0
    TCB0mask = $01;
    TCB0_DEFAULT = $00;
    TCB0_ALTERNATE = $01;
    // PORTMUX_TCB1
    TCB1mask = $02;
    TCB1_DEFAULT = $00;
    TCB1_ALTERNATE = $02;
  end;

  TRSTCTRL = object //Reset controller
    RSTFR: byte;  //Reset Flags
    SWRR: byte;  //Software Reset
  const
    // Brown out detector Reset flag
    BORFbm = $02;
    // External Reset flag
    EXTRFbm = $04;
    // Power on Reset flag
    PORFbm = $01;
    // Software Reset flag
    SWRFbm = $10;
    // UPDI Reset flag
    UPDIRFbm = $20;
    // Watch dog Reset flag
    WDRFbm = $08;
    // Software reset enable
    SWREbm = $01;
  end;

  TRTC = object //Real-Time Counter
    CTRLA: byte;  //Control A
    STATUS: byte;  //Status
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    TEMP: byte;  //Temporary
    DBGCTRL: byte;  //Debug control
    Reserved6: byte;
    CLKSEL: byte;  //Clock Select
    CNT: word;  //Counter
    PER: word;  //Period
    CMP: word;  //Compare
    Reserved14: byte;
    Reserved15: byte;
    PITCTRLA: byte;  //PIT Control A
    PITSTATUS: byte;  //PIT Status
    PITINTCTRL: byte;  //PIT Interrupt Control
    PITINTFLAGS: byte;  //PIT Interrupt Flags
    Reserved20: byte;
    PITDBGCTRL: byte;  //PIT Debug control
  const
    // RTC_CLKSEL
    CLKSELmask = $03;
    CLKSEL_INT32K = $00;
    CLKSEL_INT1K = $01;
    CLKSEL_TOSC32K = $02;
    CLKSEL_EXTCLK = $03;
    // RTC_PRESCALER
    PRESCALERmask = $78;
    PRESCALER_DIV1 = $00;
    PRESCALER_DIV2 = $08;
    PRESCALER_DIV4 = $10;
    PRESCALER_DIV8 = $18;
    PRESCALER_DIV16 = $20;
    PRESCALER_DIV32 = $28;
    PRESCALER_DIV64 = $30;
    PRESCALER_DIV128 = $38;
    PRESCALER_DIV256 = $40;
    PRESCALER_DIV512 = $48;
    PRESCALER_DIV1024 = $50;
    PRESCALER_DIV2048 = $58;
    PRESCALER_DIV4096 = $60;
    PRESCALER_DIV8192 = $68;
    PRESCALER_DIV16384 = $70;
    PRESCALER_DIV32768 = $78;
    // Enable
    RTCENbm = $01;
    // Run In Standby
    RUNSTDBYbm = $80;
    // Run in debug
    DBGRUNbm = $01;
    // Compare Match Interrupt enable
    CMPbm = $02;
    // Overflow Interrupt enable
    OVFbm = $01;
    // RTC_PERIOD
    PERIODmask = $78;
    PERIOD_OFF = $00;
    PERIOD_CYC4 = $08;
    PERIOD_CYC8 = $10;
    PERIOD_CYC16 = $18;
    PERIOD_CYC32 = $20;
    PERIOD_CYC64 = $28;
    PERIOD_CYC128 = $30;
    PERIOD_CYC256 = $38;
    PERIOD_CYC512 = $40;
    PERIOD_CYC1024 = $48;
    PERIOD_CYC2048 = $50;
    PERIOD_CYC4096 = $58;
    PERIOD_CYC8192 = $60;
    PERIOD_CYC16384 = $68;
    PERIOD_CYC32768 = $70;
    // Enable
    PITENbm = $01;
    // Periodic Interrupt
    PIbm = $01;
    // CTRLA Synchronization Busy Flag
    CTRLBUSYbm = $01;
    // Comparator Synchronization Busy Flag
    CMPBUSYbm = $08;
    // Count Synchronization Busy Flag
    CNTBUSYbm = $02;
    // CTRLA Synchronization Busy Flag
    CTRLABUSYbm = $01;
    // Period Synchronization Busy Flag
    PERBUSYbm = $04;
  end;

  TSIGROW = object //Signature row
    DEVICEID0: byte;  //Device ID Byte 0
    DEVICEID1: byte;  //Device ID Byte 1
    DEVICEID2: byte;  //Device ID Byte 2
    SERNUM0: byte;  //Serial Number Byte 0
    SERNUM1: byte;  //Serial Number Byte 1
    SERNUM2: byte;  //Serial Number Byte 2
    SERNUM3: byte;  //Serial Number Byte 3
    SERNUM4: byte;  //Serial Number Byte 4
    SERNUM5: byte;  //Serial Number Byte 5
    SERNUM6: byte;  //Serial Number Byte 6
    SERNUM7: byte;  //Serial Number Byte 7
    SERNUM8: byte;  //Serial Number Byte 8
    SERNUM9: byte;  //Serial Number Byte 9
    Reserved13: byte;
    Reserved14: byte;
    Reserved15: byte;
    Reserved16: byte;
    Reserved17: byte;
    Reserved18: byte;
    Reserved19: byte;
    Reserved20: byte;
    Reserved21: byte;
    Reserved22: byte;
    Reserved23: byte;
    Reserved24: byte;
    Reserved25: byte;
    Reserved26: byte;
    Reserved27: byte;
    Reserved28: byte;
    Reserved29: byte;
    Reserved30: byte;
    Reserved31: byte;
    TEMPSENSE0: byte;  //Temperature Sensor Calibration Byte 0
    TEMPSENSE1: byte;  //Temperature Sensor Calibration Byte 1
    OSC16ERR3V: byte;  //OSC16 error at 3V
    OSC16ERR5V: byte;  //OSC16 error at 5V
    OSC20ERR3V: byte;  //OSC20 error at 3V
    OSC20ERR5V: byte;  //OSC20 error at 5V
  end;

  TSLPCTRL = object //Sleep Controller
    CTRLA: byte;  //Control
  const
    // Sleep enable
    SENbm = $01;
    // SLPCTRL_SMODE
    SMODEmask = $06;
    SMODE_IDLE = $00;
    SMODE_STDBY = $02;
    SMODE_PDOWN = $04;
  end;

  TSPI = object //Serial Peripheral Interface
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    DATA: byte;  //Data
  const
    // Enable Double Speed
    CLK2Xbm = $10;
    // Data Order Setting
    DORDbm = $40;
    // Enable Module
    ENABLEbm = $01;
    // Master Operation Enable
    MASTERbm = $20;
    // SPI_PRESC
    PRESCmask = $06;
    PRESC_DIV4 = $00;
    PRESC_DIV16 = $02;
    PRESC_DIV64 = $04;
    PRESC_DIV128 = $06;
    // Buffer Mode Enable
    BUFENbm = $80;
    // Buffer Write Mode
    BUFWRbm = $40;
    // SPI_MODE
    MODEmask = $03;
    MODE_0 = $00;
    MODE_1 = $01;
    MODE_2 = $02;
    MODE_3 = $03;
    // Slave Select Disable
    SSDbm = $04;
    // Data Register Empty Interrupt Enable
    DREIEbm = $20;
    // Interrupt Enable
    IEbm = $01;
    // Receive Complete Interrupt Enable
    RXCIEbm = $80;
    // Slave Select Trigger Interrupt Enable
    SSIEbm = $10;
    // Transfer Complete Interrupt Enable
    TXCIEbm = $40;
    // Buffer Overflow
    BUFOVFbm = $01;
    // Data Register Empty Interrupt Flag
    DREIFbm = $20;
    // Receive Complete Interrupt Flag
    RXCIFbm = $80;
    // Slave Select Trigger Interrupt Flag
    SSIFbm = $10;
    // Transfer Complete Interrupt Flag
    TXCIFbm = $40;
    // Interrupt Flag
    IFbm = $80;
    // Write Collision
    WRCOLbm = $40;
  end;

  TSYSCFG = object //System Configuration Registers
    Reserved0: byte;
    REVID: byte;  //Revision ID
    EXTBRK: byte;  //External Break
  const
    // External break enable
    ENEXTBRKbm = $01;
  end;

  TTCA_SINGLE = object //16-bit Timer/Counter Type A - Single Mode
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    CTRLC: byte;  //Control C
    CTRLD: byte;  //Control D
    CTRLECLR: byte;  //Control E Clear
    CTRLESET: byte;  //Control E Set
    CTRLFCLR: byte;  //Control F Clear
    CTRLFSET: byte;  //Control F Set
    Reserved8: byte;
    EVCTRL: byte;  //Event Control
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    Reserved12: byte;
    Reserved13: byte;
    DBGCTRL: byte;  //Degbug Control
    TEMP: byte;  //Temporary data for 16-bit Access
    Reserved16: byte;
    Reserved17: byte;
    Reserved18: byte;
    Reserved19: byte;
    Reserved20: byte;
    Reserved21: byte;
    Reserved22: byte;
    Reserved23: byte;
    Reserved24: byte;
    Reserved25: byte;
    Reserved26: byte;
    Reserved27: byte;
    Reserved28: byte;
    Reserved29: byte;
    Reserved30: byte;
    Reserved31: byte;
    CNT: word;  //Count
    Reserved34: byte;
    Reserved35: byte;
    Reserved36: byte;
    Reserved37: byte;
    PER: word;  //Period
    CMP0: word;  //Compare 0
    CMP1: word;  //Compare 1
    CMP2: word;  //Compare 2
    Reserved46: byte;
    Reserved47: byte;
    Reserved48: byte;
    Reserved49: byte;
    Reserved50: byte;
    Reserved51: byte;
    Reserved52: byte;
    Reserved53: byte;
    PERBUF: word;  //Period Buffer
    CMP0BUF: word;  //Compare 0 Buffer
    CMP1BUF: word;  //Compare 1 Buffer
    CMP2BUF: word;  //Compare 2 Buffer
  const
    // TCA_SINGLE_CLKSEL
    SINGLE_CLKSELmask = $0E;
    SINGLE_CLKSEL_DIV1 = $00;
    SINGLE_CLKSEL_DIV2 = $02;
    SINGLE_CLKSEL_DIV4 = $04;
    SINGLE_CLKSEL_DIV8 = $06;
    SINGLE_CLKSEL_DIV16 = $08;
    SINGLE_CLKSEL_DIV64 = $0A;
    SINGLE_CLKSEL_DIV256 = $0C;
    SINGLE_CLKSEL_DIV1024 = $0E;
    // Module Enable
    ENABLEbm = $01;
    // Auto Lock Update
    ALUPDbm = $08;
    // Compare 0 Enable
    CMP0ENbm = $10;
    // Compare 1 Enable
    CMP1ENbm = $20;
    // Compare 2 Enable
    CMP2ENbm = $40;
    // TCA_SINGLE_WGMODE
    SINGLE_WGMODEmask = $07;
    SINGLE_WGMODE_NORMAL = $00;
    SINGLE_WGMODE_FRQ = $01;
    SINGLE_WGMODE_SINGLESLOPE = $03;
    SINGLE_WGMODE_DSTOP = $05;
    SINGLE_WGMODE_DSBOTH = $06;
    SINGLE_WGMODE_DSBOTTOM = $07;
    // Compare 0 Waveform Output Value
    CMP0OVbm = $01;
    // Compare 1 Waveform Output Value
    CMP1OVbm = $02;
    // Compare 2 Waveform Output Value
    CMP2OVbm = $04;
    // Split Mode Enable
    SPLITMbm = $01;
    // TCA_SINGLE_CMD
    SINGLE_CMDmask = $0C;
    SINGLE_CMD_NONE = $00;
    SINGLE_CMD_UPDATE = $04;
    SINGLE_CMD_RESTART = $08;
    SINGLE_CMD_RESET = $0C;
    // Direction
    DIRbm = $01;
    // Lock Update
    LUPDbm = $02;
    // Compare 0 Buffer Valid
    CMP0BVbm = $02;
    // Compare 1 Buffer Valid
    CMP1BVbm = $04;
    // Compare 2 Buffer Valid
    CMP2BVbm = $08;
    // Period Buffer Valid
    PERBVbm = $01;
    // Debug Run
    DBGRUNbm = $01;
    // Count on Event Input
    CNTEIbm = $01;
    // TCA_SINGLE_EVACT
    SINGLE_EVACTmask = $06;
    SINGLE_EVACT_POSEDGE = $00;
    SINGLE_EVACT_ANYEDGE = $02;
    SINGLE_EVACT_HIGHLVL = $04;
    SINGLE_EVACT_UPDOWN = $06;
    // Compare 0 Interrupt
    CMP0bm = $10;
    // Compare 1 Interrupt
    CMP1bm = $20;
    // Compare 2 Interrupt
    CMP2bm = $40;
    // Overflow Interrupt
    OVFbm = $01;
  end;

  TTCA_SPLIT = object //16-bit Timer/Counter Type A - Split Mode
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    CTRLC: byte;  //Control C
    CTRLD: byte;  //Control D
    CTRLECLR: byte;  //Control E Clear
    CTRLESET: byte;  //Control E Set
    Reserved6: byte;
    Reserved7: byte;
    Reserved8: byte;
    Reserved9: byte;
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    Reserved12: byte;
    Reserved13: byte;
    DBGCTRL: byte;  //Degbug Control
    Reserved15: byte;
    Reserved16: byte;
    Reserved17: byte;
    Reserved18: byte;
    Reserved19: byte;
    Reserved20: byte;
    Reserved21: byte;
    Reserved22: byte;
    Reserved23: byte;
    Reserved24: byte;
    Reserved25: byte;
    Reserved26: byte;
    Reserved27: byte;
    Reserved28: byte;
    Reserved29: byte;
    Reserved30: byte;
    Reserved31: byte;
    LCNT: byte;  //Low Count
    HCNT: byte;  //High Count
    Reserved34: byte;
    Reserved35: byte;
    Reserved36: byte;
    Reserved37: byte;
    LPER: byte;  //Low Period
    HPER: byte;  //High Period
    LCMP0: byte;  //Low Compare
    HCMP0: byte;  //High Compare
    LCMP1: byte;  //Low Compare
    HCMP1: byte;  //High Compare
    LCMP2: byte;  //Low Compare
    HCMP2: byte;  //High Compare
  const
    // TCA_SPLIT_CLKSEL
    SPLIT_CLKSELmask = $0E;
    SPLIT_CLKSEL_DIV1 = $00;
    SPLIT_CLKSEL_DIV2 = $02;
    SPLIT_CLKSEL_DIV4 = $04;
    SPLIT_CLKSEL_DIV8 = $06;
    SPLIT_CLKSEL_DIV16 = $08;
    SPLIT_CLKSEL_DIV64 = $0A;
    SPLIT_CLKSEL_DIV256 = $0C;
    SPLIT_CLKSEL_DIV1024 = $0E;
    // Module Enable
    ENABLEbm = $01;
    // High Compare 0 Enable
    HCMP0ENbm = $10;
    // High Compare 1 Enable
    HCMP1ENbm = $20;
    // High Compare 2 Enable
    HCMP2ENbm = $40;
    // Low Compare 0 Enable
    LCMP0ENbm = $01;
    // Low Compare 1 Enable
    LCMP1ENbm = $02;
    // Low Compare 2 Enable
    LCMP2ENbm = $04;
    // High Compare 0 Output Value
    HCMP0OVbm = $10;
    // High Compare 1 Output Value
    HCMP1OVbm = $20;
    // High Compare 2 Output Value
    HCMP2OVbm = $40;
    // Low Compare 0 Output Value
    LCMP0OVbm = $01;
    // Low Compare 1 Output Value
    LCMP1OVbm = $02;
    // Low Compare 2 Output Value
    LCMP2OVbm = $04;
    // Split Mode Enable
    SPLITMbm = $01;
    // TCA_SPLIT_CMD
    SPLIT_CMDmask = $0C;
    SPLIT_CMD_NONE = $00;
    SPLIT_CMD_UPDATE = $04;
    SPLIT_CMD_RESTART = $08;
    SPLIT_CMD_RESET = $0C;
    // Debug Run
    DBGRUNbm = $01;
    // High Underflow Interrupt Enable
    HUNFbm = $02;
    // Low Compare 0 Interrupt Enable
    LCMP0bm = $10;
    // Low Compare 1 Interrupt Enable
    LCMP1bm = $20;
    // Low Compare 2 Interrupt Enable
    LCMP2bm = $40;
    // Low Underflow Interrupt Enable
    LUNFbm = $01;
  end;

  TTCA = record //16-bit Timer/Counter Type A
    case byte of
      0: (SINGLE: TTCA_SINGLE);
      1: (SPLIT: TTCA_SPLIT);
  end;

  TTCB = object //16-bit Timer Type B
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control Register B
    Reserved2: byte;
    Reserved3: byte;
    EVCTRL: byte;  //Event Control
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    STATUS: byte;  //Status
    DBGCTRL: byte;  //Debug Control
    TEMP: byte;  //Temporary Value
    CNT: word;  //Count
    CCMP: word;  //Compare or Capture
  const
    // TCB_CLKSEL
    CLKSELmask = $06;
    CLKSEL_CLKDIV1 = $00;
    CLKSEL_CLKDIV2 = $02;
    CLKSEL_CLKTCA = $04;
    // Enable
    ENABLEbm = $01;
    // Run Standby
    RUNSTDBYbm = $40;
    // Synchronize Update
    SYNCUPDbm = $10;
    // Asynchronous Enable
    ASYNCbm = $40;
    // Pin Output Enable
    CCMPENbm = $10;
    // Pin Initial State
    CCMPINITbm = $20;
    // TCB_CNTMODE
    CNTMODEmask = $07;
    CNTMODE_INT = $00;
    CNTMODE_TIMEOUT = $01;
    CNTMODE_CAPT = $02;
    CNTMODE_FRQ = $03;
    CNTMODE_PW = $04;
    CNTMODE_FRQPW = $05;
    CNTMODE_SINGLE = $06;
    CNTMODE_PWM8 = $07;
    // Debug Run
    DBGRUNbm = $01;
    // Event Input Enable
    CAPTEIbm = $01;
    // Event Edge
    EDGEbm = $10;
    // Input Capture Noise Cancellation Filter
    FILTERbm = $40;
    // Capture or Timeout
    CAPTbm = $01;
    // Run
    RUNbm = $01;
  end;

  TTCD = object //Timer Counter D
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    CTRLC: byte;  //Control C
    CTRLD: byte;  //Control D
    CTRLE: byte;  //Control E
    Reserved5: byte;
    Reserved6: byte;
    Reserved7: byte;
    EVCTRLA: byte;  //EVCTRLA
    EVCTRLB: byte;  //EVCTRLB
    Reserved10: byte;
    Reserved11: byte;
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    STATUS: byte;  //Status
    Reserved15: byte;
    INPUTCTRLA: byte;  //Input Control A
    INPUTCTRLB: byte;  //Input Control B
    FAULTCTRL: byte;  //Fault Control
    Reserved19: byte;
    DLYCTRL: byte;  //Delay Control
    DLYVAL: byte;  //Delay value
    Reserved22: byte;
    Reserved23: byte;
    DITCTRL: byte;  //Dither Control A
    DITVAL: byte;  //Dither value
    Reserved26: byte;
    Reserved27: byte;
    Reserved28: byte;
    Reserved29: byte;
    DBGCTRL: byte;  //Debug Control
    Reserved31: byte;
    Reserved32: byte;
    Reserved33: byte;
    CAPTUREA: word;  //Capture A
    CAPTUREB: word;  //Capture B
    Reserved38: byte;
    Reserved39: byte;
    CMPASET: word;  //Compare A Set
    CMPACLR: word;  //Compare A Clear
    CMPBSET: word;  //Compare B Set
    CMPBCLR: word;  //Compare B Clear
  const
    // TCD_CLKSEL
    CLKSELmask = $60;
    CLKSEL_20MHZ = $00;
    CLKSEL_EXTCLK = $40;
    CLKSEL_SYSCLK = $60;
    // TCD_CNTPRES
    CNTPRESmask = $18;
    CNTPRES_DIV1 = $00;
    CNTPRES_DIV4 = $08;
    CNTPRES_DIV32 = $10;
    // Enable
    ENABLEbm = $01;
    // TCD_SYNCPRES
    SYNCPRESmask = $06;
    SYNCPRES_DIV1 = $00;
    SYNCPRES_DIV2 = $02;
    SYNCPRES_DIV4 = $04;
    SYNCPRES_DIV8 = $06;
    // TCD_WGMODE
    WGMODEmask = $03;
    WGMODE_ONERAMP = $00;
    WGMODE_TWORAMP = $01;
    WGMODE_FOURRAMP = $02;
    WGMODE_DS = $03;
    // Auto update
    AUPDATEbm = $02;
    // TCD_CMPCSEL
    CMPCSELmask = $40;
    CMPCSEL_PWMA = $00;
    CMPCSEL_PWMB = $40;
    // TCD_CMPDSEL
    CMPDSELmask = $80;
    CMPDSEL_PWMA = $00;
    CMPDSEL_PWMB = $80;
    // Compare output value override
    CMPOVRbm = $01;
    // Fifty percent waveform
    FIFTYbm = $08;
    // Compare A value
    CMPAVAL0bm = $01;
    CMPAVAL1bm = $02;
    CMPAVAL2bm = $04;
    CMPAVAL3bm = $08;
    // Compare B value
    CMPBVAL0bm = $10;
    CMPBVAL1bm = $20;
    CMPBVAL2bm = $40;
    CMPBVAL3bm = $80;
    // Disable at end of cycle
    DISEOCbm = $80;
    // Restart strobe
    RESTARTbm = $04;
    // Software Capture A Strobe
    SCAPTUREAbm = $08;
    // Software Capture B Strobe
    SCAPTUREBbm = $10;
    // synchronize strobe
    SYNCbm = $02;
    // synchronize end of cycle strobe
    SYNCEOCbm = $01;
    // Debug run
    DBGRUNbm = $01;
    // Fault detection
    FAULTDETbm = $04;
    // TCD_DITHERSEL
    DITHERSELmask = $03;
    DITHERSEL_ONTIMEB = $00;
    DITHERSEL_ONTIMEAB = $01;
    DITHERSEL_DEADTIMEB = $02;
    DITHERSEL_DEADTIMEAB = $03;
    // Dither value
    DITHER0bm = $01;
    DITHER1bm = $02;
    DITHER2bm = $04;
    DITHER3bm = $08;
    // TCD_DLYPRESC
    DLYPRESCmask = $30;
    DLYPRESC_DIV1 = $00;
    DLYPRESC_DIV2 = $10;
    DLYPRESC_DIV4 = $20;
    DLYPRESC_DIV8 = $30;
    // TCD_DLYSEL
    DLYSELmask = $03;
    DLYSEL_OFF = $00;
    DLYSEL_INBLANK = $01;
    DLYSEL_EVENT = $02;
    // TCD_DLYTRIG
    DLYTRIGmask = $0C;
    DLYTRIG_CMPASET = $00;
    DLYTRIG_CMPACLR = $04;
    DLYTRIG_CMPBSET = $08;
    DLYTRIG_CMPBCLR = $0C;
    // Delay value
    DLYVAL0bm = $01;
    DLYVAL1bm = $02;
    DLYVAL2bm = $04;
    DLYVAL3bm = $08;
    DLYVAL4bm = $10;
    DLYVAL5bm = $20;
    DLYVAL6bm = $40;
    DLYVAL7bm = $80;
    // TCD_ACTION
    ACTIONmask = $04;
    ACTION_FAULT = $00;
    ACTION_CAPTURE = $04;
    // TCD_CFG
    CFGmask = $C0;
    CFG_NEITHER = $00;
    CFG_FILTER = $40;
    CFG_ASYNC = $80;
    // TCD_EDGE
    EDGEmask = $10;
    EDGE_FALL_LOW = $00;
    EDGE_RISE_HIGH = $10;
    // Trigger event enable
    TRIGEIbm = $01;
    // Compare A value
    CMPAbm = $01;
    // Compare A enable
    CMPAENbm = $10;
    // Compare B value
    CMPBbm = $02;
    // Compare B enable
    CMPBENbm = $20;
    // Compare C value
    CMPCbm = $04;
    // Compare C enable
    CMPCENbm = $40;
    // Compare D vaule
    CMPDbm = $08;
    // Compare D enable
    CMPDENbm = $80;
    // TCD_INPUTMODE
    INPUTMODEmask = $0F;
    INPUTMODE_NONE = $00;
    INPUTMODE_JMPWAIT = $01;
    INPUTMODE_EXECWAIT = $02;
    INPUTMODE_EXECFAULT = $03;
    INPUTMODE_FREQ = $04;
    INPUTMODE_EXECDT = $05;
    INPUTMODE_WAIT = $06;
    INPUTMODE_WAITSW = $07;
    INPUTMODE_EDGETRIG = $08;
    INPUTMODE_EDGETRIGFREQ = $09;
    INPUTMODE_LVLTRIGFREQ = $0A;
    // Overflow interrupt enable
    OVFbm = $01;
    // Trigger A interrupt enable
    TRIGAbm = $04;
    // Trigger B interrupt enable
    TRIGBbm = $08;
    // Command ready
    CMDRDYbm = $02;
    // Enable ready
    ENRDYbm = $01;
    // PWM activity on A
    PWMACTAbm = $40;
    // PWM activity on B
    PWMACTBbm = $80;
  end;

  TTWI = object //Two-Wire Interface
    CTRLA: byte;  //Control A
    Reserved1: byte;
    DBGCTRL: byte;  //Debug Control Register
    MCTRLA: byte;  //Master Control A
    MCTRLB: byte;  //Master Control B
    MSTATUS: byte;  //Master Status
    MBAUD: byte;  //Master Baurd Rate Control
    MADDR: byte;  //Master Address
    MDATA: byte;  //Master Data
    SCTRLA: byte;  //Slave Control A
    SCTRLB: byte;  //Slave Control B
    SSTATUS: byte;  //Slave Status
    SADDR: byte;  //Slave Address
    SDATA: byte;  //Slave Data
    SADDRMASK: byte;  //Slave Address Mask
  const
    // FM Plus Enable
    FMPENbm = $02;
    // TWI_DEFAULT_SDAHOLD
    DEFAULT_SDAHOLDmask = $0C;
    DEFAULT_SDAHOLD_OFF = $00;
    DEFAULT_SDAHOLD_50NS = $04;
    DEFAULT_SDAHOLD_300NS = $08;
    DEFAULT_SDAHOLD_500NS = $0C;
    // TWI_DEFAULT_SDASETUP
    DEFAULT_SDASETUPmask = $10;
    DEFAULT_SDASETUP_4CYC = $00;
    DEFAULT_SDASETUP_8CYC = $10;
    // Debug Run
    DBGRUNbm = $01;
    // Enable TWI Master
    ENABLEbm = $01;
    // Quick Command Enable
    QCENbm = $10;
    // Read Interrupt Enable
    RIENbm = $80;
    // Smart Mode Enable
    SMENbm = $02;
    // TWI_TIMEOUT
    TIMEOUTmask = $0C;
    TIMEOUT_DISABLED = $00;
    TIMEOUT_50US = $04;
    TIMEOUT_100US = $08;
    TIMEOUT_200US = $0C;
    // Write Interrupt Enable
    WIENbm = $40;
    // TWI_ACKACT
    ACKACTmask = $04;
    ACKACT_ACK = $00;
    ACKACT_NACK = $04;
    // Flush
    FLUSHbm = $08;
    // TWI_MCMD
    MCMDmask = $03;
    MCMD_NOACT = $00;
    MCMD_REPSTART = $01;
    MCMD_RECVTRANS = $02;
    MCMD_STOP = $03;
    // Arbitration Lost
    ARBLOSTbm = $08;
    // Bus Error
    BUSERRbm = $04;
    // TWI_BUSSTATE
    BUSSTATEmask = $03;
    BUSSTATE_UNKNOWN = $00;
    BUSSTATE_IDLE = $01;
    BUSSTATE_OWNER = $02;
    BUSSTATE_BUSY = $03;
    // Clock Hold
    CLKHOLDbm = $20;
    // Read Interrupt Flag
    RIFbm = $80;
    // Received Acknowledge
    RXACKbm = $10;
    // Write Interrupt Flag
    WIFbm = $40;
    // Address Enable
    ADDRENbm = $01;
    // Address Mask
    ADDRMASK0bm = $02;
    ADDRMASK1bm = $04;
    ADDRMASK2bm = $08;
    ADDRMASK3bm = $10;
    ADDRMASK4bm = $20;
    ADDRMASK5bm = $40;
    ADDRMASK6bm = $80;
    // Address/Stop Interrupt Enable
    APIENbm = $40;
    // Data Interrupt Enable
    DIENbm = $80;
    // Stop Interrupt Enable
    PIENbm = $20;
    // Promiscuous Mode Enable
    PMENbm = $04;
    // TWI_SCMD
    SCMDmask = $03;
    SCMD_NOACT = $00;
    SCMD_COMPTRANS = $02;
    SCMD_RESPONSE = $03;
    // TWI_AP
    APmask = $01;
    AP_STOP = $00;
    AP_ADR = $01;
    // Address/Stop Interrupt Flag
    APIFbm = $40;
    // Collision
    COLLbm = $08;
    // Data Interrupt Flag
    DIFbm = $80;
    // Read/Write Direction
    DIRbm = $02;
  end;

  TUSART = object //Universal Synchronous and Asynchronous Receiver and Transmitter
    RXDATAL: byte;  //Receive Data Low Byte
    RXDATAH: byte;  //Receive Data High Byte
    TXDATAL: byte;  //Transmit Data Low Byte
    TXDATAH: byte;  //Transmit Data High Byte
    STATUS: byte;  //Status
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    CTRLC: byte;  //Control C
    BAUD: word;  //Baud Rate
    Reserved10: byte;
    DBGCTRL: byte;  //Debug Control
    EVCTRL: byte;  //Event Control
    TXPLCTRL: byte;  //IRCOM Transmitter Pulse Length Control
    RXPLCTRL: byte;  //IRCOM Receiver Pulse Length Control
  const
    // Auto-baud Error Interrupt Enable
    ABEIEbm = $04;
    // Data Register Empty Interrupt Enable
    DREIEbm = $20;
    // Loop-back Mode Enable
    LBMEbm = $08;
    // USART_RS485
    RS485mask = $03;
    RS485_OFF = $00;
    RS485_EXT = $01;
    RS485_INT = $02;
    // Receive Complete Interrupt Enable
    RXCIEbm = $80;
    // Receiver Start Frame Interrupt Enable
    RXSIEbm = $10;
    // Transmit Complete Interrupt Enable
    TXCIEbm = $40;
    // Multi-processor Communication Mode
    MPCMbm = $01;
    // Open Drain Mode Enable
    ODMEbm = $08;
    // Reciever enable
    RXENbm = $80;
    // USART_RXMODE
    RXMODEmask = $06;
    RXMODE_NORMAL = $00;
    RXMODE_CLK2X = $02;
    RXMODE_GENAUTO = $04;
    RXMODE_LINAUTO = $06;
    // Start Frame Detection Enable
    SFDENbm = $10;
    // Transmitter Enable
    TXENbm = $40;
    // USART_MSPI_CMODE
    MSPI_CMODEmask = $C0;
    MSPI_CMODE_ASYNCHRONOUS = $00;
    MSPI_CMODE_SYNCHRONOUS = $40;
    MSPI_CMODE_IRCOM = $80;
    MSPI_CMODE_MSPI = $C0;
    // SPI Master Mode, Clock Phase
    UCPHAbm = $02;
    // SPI Master Mode, Data Order
    UDORDbm = $04;
    // USART_NORMAL_CHSIZE
    NORMAL_CHSIZEmask = $07;
    NORMAL_CHSIZE_5BIT = $00;
    NORMAL_CHSIZE_6BIT = $01;
    NORMAL_CHSIZE_7BIT = $02;
    NORMAL_CHSIZE_8BIT = $03;
    NORMAL_CHSIZE_9BITL = $06;
    NORMAL_CHSIZE_9BITH = $07;
    // USART_NORMAL_CMODE
    NORMAL_CMODEmask = $C0;
    NORMAL_CMODE_ASYNCHRONOUS = $00;
    NORMAL_CMODE_SYNCHRONOUS = $40;
    NORMAL_CMODE_IRCOM = $80;
    NORMAL_CMODE_MSPI = $C0;
    // USART_NORMAL_PMODE
    NORMAL_PMODEmask = $30;
    NORMAL_PMODE_DISABLED = $00;
    NORMAL_PMODE_EVEN = $20;
    NORMAL_PMODE_ODD = $30;
    // USART_NORMAL_SBMODE
    NORMAL_SBMODEmask = $08;
    NORMAL_SBMODE_1BIT = $00;
    NORMAL_SBMODE_2BIT = $08;
    // Autobaud majority voter bypass
    ABMBPbm = $80;
    // Debug Run
    DBGRUNbm = $01;
    // IrDA Event Input Enable
    IREIbm = $01;
    // Buffer Overflow
    BUFOVFbm = $40;
    // Receiver Data Register
    DATA8bm = $01;
    // Frame Error
    FERRbm = $04;
    // Parity Error
    PERRbm = $02;
    // Receive Complete Interrupt Flag
    RXCIFbm = $80;
    // RX Data
    DATA0bm = $01;
    DATA1bm = $02;
    DATA2bm = $04;
    DATA3bm = $08;
    DATA4bm = $10;
    DATA5bm = $20;
    DATA6bm = $40;
    DATA7bm = $80;
    // Receiver Pulse Lenght
    RXPL0bm = $01;
    RXPL1bm = $02;
    RXPL2bm = $04;
    RXPL3bm = $08;
    RXPL4bm = $10;
    RXPL5bm = $20;
    RXPL6bm = $40;
    // Break Detected Flag
    BDFbm = $02;
    // Data Register Empty Flag
    DREIFbm = $20;
    // Inconsistent Sync Field Interrupt Flag
    ISFIFbm = $08;
    // Receive Start Interrupt
    RXSIFbm = $10;
    // Transmit Interrupt Flag
    TXCIFbm = $40;
    // Wait For Break
    WFBbm = $01;
    // Transmit pulse length
    TXPL0bm = $01;
    TXPL1bm = $02;
    TXPL2bm = $04;
    TXPL3bm = $08;
    TXPL4bm = $10;
    TXPL5bm = $20;
    TXPL6bm = $40;
    TXPL7bm = $80;
  end;

  TUSERROW = object //User Row
    USERROW0: byte;  //User Row Byte 0
    USERROW1: byte;  //User Row Byte 1
    USERROW2: byte;  //User Row Byte 2
    USERROW3: byte;  //User Row Byte 3
    USERROW4: byte;  //User Row Byte 4
    USERROW5: byte;  //User Row Byte 5
    USERROW6: byte;  //User Row Byte 6
    USERROW7: byte;  //User Row Byte 7
    USERROW8: byte;  //User Row Byte 8
    USERROW9: byte;  //User Row Byte 9
    USERROW10: byte;  //User Row Byte 10
    USERROW11: byte;  //User Row Byte 11
    USERROW12: byte;  //User Row Byte 12
    USERROW13: byte;  //User Row Byte 13
    USERROW14: byte;  //User Row Byte 14
    USERROW15: byte;  //User Row Byte 15
    USERROW16: byte;  //User Row Byte 16
    USERROW17: byte;  //User Row Byte 17
    USERROW18: byte;  //User Row Byte 18
    USERROW19: byte;  //User Row Byte 19
    USERROW20: byte;  //User Row Byte 20
    USERROW21: byte;  //User Row Byte 21
    USERROW22: byte;  //User Row Byte 22
    USERROW23: byte;  //User Row Byte 23
    USERROW24: byte;  //User Row Byte 24
    USERROW25: byte;  //User Row Byte 25
    USERROW26: byte;  //User Row Byte 26
    USERROW27: byte;  //User Row Byte 27
    USERROW28: byte;  //User Row Byte 28
    USERROW29: byte;  //User Row Byte 29
    USERROW30: byte;  //User Row Byte 30
    USERROW31: byte;  //User Row Byte 31
  end;

  TVPORT = object //Virtual Ports
    DIR: byte;  //Data Direction
    OUT_: byte;  //Output Value
    IN_: byte;  //Input Value
    INTFLAGS: byte;  //Interrupt Flags
  const
    // Pin Interrupt
    INT0bm = $01;
    INT1bm = $02;
    INT2bm = $04;
    INT3bm = $08;
    INT4bm = $10;
    INT5bm = $20;
    INT6bm = $40;
    INT7bm = $80;
  end;

  TVREF = object //Voltage reference
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    CTRLC: byte;  //Control C
    CTRLD: byte;  //Control D
  const
    // VREF_ADC0REFSEL
    ADC0REFSELmask = $70;
    ADC0REFSEL_0V55 = $00;
    ADC0REFSEL_1V1 = $10;
    ADC0REFSEL_2V5 = $20;
    ADC0REFSEL_4V34 = $30;
    ADC0REFSEL_1V5 = $40;
    // VREF_DAC0REFSEL
    DAC0REFSELmask = $07;
    DAC0REFSEL_0V55 = $00;
    DAC0REFSEL_1V1 = $01;
    DAC0REFSEL_2V5 = $02;
    DAC0REFSEL_4V34 = $03;
    DAC0REFSEL_1V5 = $04;
    // ADC0 reference enable
    ADC0REFENbm = $02;
    // ADC1 reference enable
    ADC1REFENbm = $10;
    // DAC0/AC0 reference enable
    DAC0REFENbm = $01;
    // DAC1/AC1 reference enable
    DAC1REFENbm = $08;
    // DAC2/AC2 reference enable
    DAC2REFENbm = $20;
    // VREF_ADC1REFSEL
    ADC1REFSELmask = $70;
    ADC1REFSEL_0V55 = $00;
    ADC1REFSEL_1V1 = $10;
    ADC1REFSEL_2V5 = $20;
    ADC1REFSEL_4V34 = $30;
    ADC1REFSEL_1V5 = $40;
    // VREF_DAC1REFSEL
    DAC1REFSELmask = $07;
    DAC1REFSEL_0V55 = $00;
    DAC1REFSEL_1V1 = $01;
    DAC1REFSEL_2V5 = $02;
    DAC1REFSEL_4V34 = $03;
    DAC1REFSEL_1V5 = $04;
    // VREF_DAC2REFSEL
    DAC2REFSELmask = $07;
    DAC2REFSEL_0V55 = $00;
    DAC2REFSEL_1V1 = $01;
    DAC2REFSEL_2V5 = $02;
    DAC2REFSEL_4V34 = $03;
    DAC2REFSEL_1V5 = $04;
  end;

  TWDT = object //Watch-Dog Timer
    CTRLA: byte;  //Control A
    STATUS: byte;  //Status
  const
    // WDT_PERIOD
    PERIODmask = $0F;
    PERIOD_OFF = $00;
    PERIOD_8CLK = $01;
    PERIOD_16CLK = $02;
    PERIOD_32CLK = $03;
    PERIOD_64CLK = $04;
    PERIOD_128CLK = $05;
    PERIOD_256CLK = $06;
    PERIOD_512CLK = $07;
    PERIOD_1KCLK = $08;
    PERIOD_2KCLK = $09;
    PERIOD_4KCLK = $0A;
    PERIOD_8KCLK = $0B;
    // WDT_WINDOW
    WINDOWmask = $F0;
    WINDOW_OFF = $00;
    WINDOW_8CLK = $10;
    WINDOW_16CLK = $20;
    WINDOW_32CLK = $30;
    WINDOW_64CLK = $40;
    WINDOW_128CLK = $50;
    WINDOW_256CLK = $60;
    WINDOW_512CLK = $70;
    WINDOW_1KCLK = $80;
    WINDOW_2KCLK = $90;
    WINDOW_4KCLK = $A0;
    WINDOW_8KCLK = $B0;
    // Lock enable
    LOCKbm = $80;
    // Syncronization busy
    SYNCBUSYbm = $01;
  end;


const
 Pin0idx = 0;  Pin0bm = 1;
 Pin1idx = 1;  Pin1bm = 2;
 Pin2idx = 2;  Pin2bm = 4;
 Pin3idx = 3;  Pin3bm = 8;
 Pin4idx = 4;  Pin4bm = 16;
 Pin5idx = 5;  Pin5bm = 32;
 Pin6idx = 6;  Pin6bm = 64;
 Pin7idx = 7;  Pin7bm = 128;

var
  VPORTA: TVPORT absolute $0000;
  VPORTB: TVPORT absolute $0004;
  VPORTC: TVPORT absolute $0008;
  GPIO: TGPIO absolute $001C;
  CPU: TCPU absolute $0030;
  RSTCTRL: TRSTCTRL absolute $0040;
  SLPCTRL: TSLPCTRL absolute $0050;
  CLKCTRL: TCLKCTRL absolute $0060;
  BOD: TBOD absolute $0080;
  VREF: TVREF absolute $00A0;
  WDT: TWDT absolute $0100;
  CPUINT: TCPUINT absolute $0110;
  CRCSCAN: TCRCSCAN absolute $0120;
  RTC: TRTC absolute $0140;
  EVSYS: TEVSYS absolute $0180;
  CCL: TCCL absolute $01C0;
  PORTMUX: TPORTMUX absolute $0200;
  PORTA: TPORT absolute $0400;
  PORTB: TPORT absolute $0420;
  ADC0: TADC absolute $0600;
  ADC1: TADC absolute $0640;
  AC0: TAC absolute $0680;
  AC1: TAC absolute $0688;
  AC2: TAC absolute $0690;
  DAC0: TDAC absolute $06A0;
  DAC1: TDAC absolute $06A8;
  DAC2: TDAC absolute $06B0;
  USART0: TUSART absolute $0800;
  TWI0: TTWI absolute $0810;
  SPI0: TSPI absolute $0820;
  TCA0: TTCA absolute $0A00;
  TCB0: TTCB absolute $0A40;
  TCB1: TTCB absolute $0A50;
  TCD0: TTCD absolute $0A80;
  SYSCFG: TSYSCFG absolute $0F00;
  NVMCTRL: TNVMCTRL absolute $1000;
  SIGROW: TSIGROW absolute $1100;
  FUSE: TFUSE absolute $1280;
  LOCKBIT: TLOCKBIT absolute $128A;
  USERROW: TUSERROW absolute $1300;

implementation

{$i avrcommon.inc}

procedure CRCSCAN_NMI_ISR; external name 'CRCSCAN_NMI_ISR'; // Interrupt 1 
procedure BOD_VLM_ISR; external name 'BOD_VLM_ISR'; // Interrupt 2 
procedure PORTA_PORT_ISR; external name 'PORTA_PORT_ISR'; // Interrupt 3 
procedure PORTB_PORT_ISR; external name 'PORTB_PORT_ISR'; // Interrupt 4 
procedure RTC_CNT_ISR; external name 'RTC_CNT_ISR'; // Interrupt 6 
procedure RTC_PIT_ISR; external name 'RTC_PIT_ISR'; // Interrupt 7 
procedure TCA0_LUNF_ISR; external name 'TCA0_LUNF_ISR'; // Interrupt 8 
//procedure TCA0_OVF_ISR; external name 'TCA0_OVF_ISR'; // Interrupt 8 
procedure TCA0_HUNF_ISR; external name 'TCA0_HUNF_ISR'; // Interrupt 9 
procedure TCA0_LCMP0_ISR; external name 'TCA0_LCMP0_ISR'; // Interrupt 10 
//procedure TCA0_CMP0_ISR; external name 'TCA0_CMP0_ISR'; // Interrupt 10 
procedure TCA0_CMP1_ISR; external name 'TCA0_CMP1_ISR'; // Interrupt 11 
//procedure TCA0_LCMP1_ISR; external name 'TCA0_LCMP1_ISR'; // Interrupt 11 
procedure TCA0_CMP2_ISR; external name 'TCA0_CMP2_ISR'; // Interrupt 12 
//procedure TCA0_LCMP2_ISR; external name 'TCA0_LCMP2_ISR'; // Interrupt 12 
procedure TCB0_INT_ISR; external name 'TCB0_INT_ISR'; // Interrupt 13 
procedure TCB1_INT_ISR; external name 'TCB1_INT_ISR'; // Interrupt 14 
procedure TCD0_OVF_ISR; external name 'TCD0_OVF_ISR'; // Interrupt 15 
procedure TCD0_TRIG_ISR; external name 'TCD0_TRIG_ISR'; // Interrupt 16 
procedure AC0_AC_ISR; external name 'AC0_AC_ISR'; // Interrupt 17 
procedure AC1_AC_ISR; external name 'AC1_AC_ISR'; // Interrupt 18 
procedure AC2_AC_ISR; external name 'AC2_AC_ISR'; // Interrupt 19 
procedure ADC0_RESRDY_ISR; external name 'ADC0_RESRDY_ISR'; // Interrupt 20 
procedure ADC0_WCOMP_ISR; external name 'ADC0_WCOMP_ISR'; // Interrupt 21 
procedure ADC1_RESRDY_ISR; external name 'ADC1_RESRDY_ISR'; // Interrupt 22 
procedure ADC1_WCOMP_ISR; external name 'ADC1_WCOMP_ISR'; // Interrupt 23 
procedure TWI0_TWIS_ISR; external name 'TWI0_TWIS_ISR'; // Interrupt 24 
procedure TWI0_TWIM_ISR; external name 'TWI0_TWIM_ISR'; // Interrupt 25 
procedure SPI0_INT_ISR; external name 'SPI0_INT_ISR'; // Interrupt 26 
procedure USART0_RXC_ISR; external name 'USART0_RXC_ISR'; // Interrupt 27 
procedure USART0_DRE_ISR; external name 'USART0_DRE_ISR'; // Interrupt 28 
procedure USART0_TXC_ISR; external name 'USART0_TXC_ISR'; // Interrupt 29 
procedure NVMCTRL_EE_ISR; external name 'NVMCTRL_EE_ISR'; // Interrupt 30 

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  jmp _start
  jmp CRCSCAN_NMI_ISR
  jmp BOD_VLM_ISR
  jmp PORTA_PORT_ISR
  jmp PORTB_PORT_ISR
  jmp RTC_CNT_ISR
  jmp RTC_PIT_ISR
  jmp TCA0_LUNF_ISR
//  jmp TCA0_OVF_ISR
  jmp TCA0_HUNF_ISR
  jmp TCA0_LCMP0_ISR
//  jmp TCA0_CMP0_ISR
  jmp TCA0_CMP1_ISR
//  jmp TCA0_LCMP1_ISR
  jmp TCA0_CMP2_ISR
//  jmp TCA0_LCMP2_ISR
  jmp TCB0_INT_ISR
  jmp TCB1_INT_ISR
  jmp TCD0_OVF_ISR
  jmp TCD0_TRIG_ISR
  jmp AC0_AC_ISR
  jmp AC1_AC_ISR
  jmp AC2_AC_ISR
  jmp ADC0_RESRDY_ISR
  jmp ADC0_WCOMP_ISR
  jmp ADC1_RESRDY_ISR
  jmp ADC1_WCOMP_ISR
  jmp TWI0_TWIS_ISR
  jmp TWI0_TWIM_ISR
  jmp SPI0_INT_ISR
  jmp USART0_RXC_ISR
  jmp USART0_DRE_ISR
  jmp USART0_TXC_ISR
  jmp NVMCTRL_EE_ISR

  {$i start.inc}

  .weak CRCSCAN_NMI_ISR
  .weak BOD_VLM_ISR
  .weak PORTA_PORT_ISR
  .weak PORTB_PORT_ISR
  .weak RTC_CNT_ISR
  .weak RTC_PIT_ISR
  .weak TCA0_LUNF_ISR
//  .weak TCA0_OVF_ISR
  .weak TCA0_HUNF_ISR
  .weak TCA0_LCMP0_ISR
//  .weak TCA0_CMP0_ISR
  .weak TCA0_CMP1_ISR
//  .weak TCA0_LCMP1_ISR
  .weak TCA0_CMP2_ISR
//  .weak TCA0_LCMP2_ISR
  .weak TCB0_INT_ISR
  .weak TCB1_INT_ISR
  .weak TCD0_OVF_ISR
  .weak TCD0_TRIG_ISR
  .weak AC0_AC_ISR
  .weak AC1_AC_ISR
  .weak AC2_AC_ISR
  .weak ADC0_RESRDY_ISR
  .weak ADC0_WCOMP_ISR
  .weak ADC1_RESRDY_ISR
  .weak ADC1_WCOMP_ISR
  .weak TWI0_TWIS_ISR
  .weak TWI0_TWIM_ISR
  .weak SPI0_INT_ISR
  .weak USART0_RXC_ISR
  .weak USART0_DRE_ISR
  .weak USART0_TXC_ISR
  .weak NVMCTRL_EE_ISR

  .set CRCSCAN_NMI_ISR, Default_IRQ_handler
  .set BOD_VLM_ISR, Default_IRQ_handler
  .set PORTA_PORT_ISR, Default_IRQ_handler
  .set PORTB_PORT_ISR, Default_IRQ_handler
  .set RTC_CNT_ISR, Default_IRQ_handler
  .set RTC_PIT_ISR, Default_IRQ_handler
  .set TCA0_LUNF_ISR, Default_IRQ_handler
//  .set TCA0_OVF_ISR, Default_IRQ_handler
  .set TCA0_HUNF_ISR, Default_IRQ_handler
  .set TCA0_LCMP0_ISR, Default_IRQ_handler
//  .set TCA0_CMP0_ISR, Default_IRQ_handler
  .set TCA0_CMP1_ISR, Default_IRQ_handler
//  .set TCA0_LCMP1_ISR, Default_IRQ_handler
  .set TCA0_CMP2_ISR, Default_IRQ_handler
//  .set TCA0_LCMP2_ISR, Default_IRQ_handler
  .set TCB0_INT_ISR, Default_IRQ_handler
  .set TCB1_INT_ISR, Default_IRQ_handler
  .set TCD0_OVF_ISR, Default_IRQ_handler
  .set TCD0_TRIG_ISR, Default_IRQ_handler
  .set AC0_AC_ISR, Default_IRQ_handler
  .set AC1_AC_ISR, Default_IRQ_handler
  .set AC2_AC_ISR, Default_IRQ_handler
  .set ADC0_RESRDY_ISR, Default_IRQ_handler
  .set ADC0_WCOMP_ISR, Default_IRQ_handler
  .set ADC1_RESRDY_ISR, Default_IRQ_handler
  .set ADC1_WCOMP_ISR, Default_IRQ_handler
  .set TWI0_TWIS_ISR, Default_IRQ_handler
  .set TWI0_TWIM_ISR, Default_IRQ_handler
  .set SPI0_INT_ISR, Default_IRQ_handler
  .set USART0_RXC_ISR, Default_IRQ_handler
  .set USART0_DRE_ISR, Default_IRQ_handler
  .set USART0_TXC_ISR, Default_IRQ_handler
  .set NVMCTRL_EE_ISR, Default_IRQ_handler
end;

end.
