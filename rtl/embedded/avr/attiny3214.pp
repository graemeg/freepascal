unit ATtiny3214;

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
    ENABLEidx = $00;  ENABLEbm = $01;  // Enable
    HYSMODE0idx = $01;  // Hysteresis Mode
    HYSMODE1idx = $02;  // Hysteresis Mode
    INTMODE0idx = $04;  // Interrupt Mode
    INTMODE1idx = $05;  // Interrupt Mode
    LPMODEidx = $03;  LPMODEbm = $08;  // Low Power Mode
    OUTENidx = $06;  OUTENbm = $40;  // Output Buffer Enable
    RUNSTDBYidx = $07;  RUNSTDBYbm = $80;  // Run in Standby Mode
    CMPidx = $00;  CMPbm = $01;  // Analog Comparator 0 Interrupt Enable
    INVERTidx = $07;  INVERTbm = $80;  // Invert AC Output
    MUXNEG0idx = $00;  // Negative Input MUX Selection
    MUXNEG1idx = $01;  // Negative Input MUX Selection
    MUXPOS0idx = $03;  // Positive Input MUX Selection
    MUXPOS1idx = $04;  // Positive Input MUX Selection
    STATEidx = $04;  STATEbm = $10;  // Analog Comparator State
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
    DUTYCYCidx = $00;  DUTYCYCbm = $01;  // Duty Cycle
    STCONVidx = $00;  STCONVbm = $01;  // Start Conversion Operation
    ENABLEidx = $00;  ENABLEbm = $01;  // ADC Enable
    FREERUNidx = $01;  FREERUNbm = $02;  // ADC Freerun mode
    RESSELidx = $02;  RESSELbm = $04;  // ADC Resolution
    RUNSTBYidx = $07;  RUNSTBYbm = $80;  // Run standby mode
    SAMPNUM0idx = $00;  // Accumulation Samples
    SAMPNUM1idx = $01;  // Accumulation Samples
    SAMPNUM2idx = $02;  // Accumulation Samples
    PRESC0idx = $00;  // Clock Pre-scaler
    PRESC1idx = $01;  // Clock Pre-scaler
    PRESC2idx = $02;  // Clock Pre-scaler
    REFSEL0idx = $04;  // Reference Selection
    REFSEL1idx = $05;  // Reference Selection
    SAMPCAPidx = $06;  SAMPCAPbm = $40;  // Sample Capacitance Selection
    ASDVidx = $04;  ASDVbm = $10;  // Automatic Sampling Delay Variation
    INITDLY0idx = $05;  // Initial Delay Selection
    INITDLY1idx = $06;  // Initial Delay Selection
    INITDLY2idx = $07;  // Initial Delay Selection
    SAMPDLY0idx = $00;  // Sampling Delay Selection
    SAMPDLY1idx = $01;  // Sampling Delay Selection
    SAMPDLY2idx = $02;  // Sampling Delay Selection
    SAMPDLY3idx = $03;  // Sampling Delay Selection
    WINCM0idx = $00;  // Window Comparator Mode
    WINCM1idx = $01;  // Window Comparator Mode
    WINCM2idx = $02;  // Window Comparator Mode
    DBGRUNidx = $00;  DBGRUNbm = $01;  // Debug run
    STARTEIidx = $00;  STARTEIbm = $01;  // Start Event Input Enable
    RESRDYidx = $00;  RESRDYbm = $01;  // Result Ready Interrupt Enable
    WCMPidx = $01;  WCMPbm = $02;  // Window Comparator Interrupt Enable
    MUXPOS0idx = $00;  // Analog Channel Selection Bits
    MUXPOS1idx = $01;  // Analog Channel Selection Bits
    MUXPOS2idx = $02;  // Analog Channel Selection Bits
    MUXPOS3idx = $03;  // Analog Channel Selection Bits
    MUXPOS4idx = $04;  // Analog Channel Selection Bits
    SAMPLEN0idx = $00;  // Sample lenght
    SAMPLEN1idx = $01;  // Sample lenght
    SAMPLEN2idx = $02;  // Sample lenght
    SAMPLEN3idx = $03;  // Sample lenght
    SAMPLEN4idx = $04;  // Sample lenght
    TEMP0idx = $00;  // Temporary
    TEMP1idx = $01;  // Temporary
    TEMP2idx = $02;  // Temporary
    TEMP3idx = $03;  // Temporary
    TEMP4idx = $04;  // Temporary
    TEMP5idx = $05;  // Temporary
    TEMP6idx = $06;  // Temporary
    TEMP7idx = $07;  // Temporary
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
    ACTIVE0idx = $02;  // Operation in active mode
    ACTIVE1idx = $03;  // Operation in active mode
    SAMPFREQidx = $04;  SAMPFREQbm = $10;  // Sample frequency
    SLEEP0idx = $00;  // Operation in sleep mode
    SLEEP1idx = $01;  // Operation in sleep mode
    LVL0idx = $00;  // Bod level
    LVL1idx = $01;  // Bod level
    LVL2idx = $02;  // Bod level
    VLMCFG0idx = $01;  // Configuration
    VLMCFG1idx = $02;  // Configuration
    VLMIEidx = $00;  VLMIEbm = $01;  // voltage level monitor interrrupt enable
    VLMIFidx = $00;  VLMIFbm = $01;  // Voltage level monitor interrupt flag
    VLMSidx = $00;  VLMSbm = $01;  // Voltage level monitor status
    VLMLVL0idx = $00;  // voltage level monitor level
    VLMLVL1idx = $01;  // voltage level monitor level
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
    ENABLEidx = $00;  ENABLEbm = $01;  // Enable
    RUNSTDBYidx = $06;  RUNSTDBYbm = $40;  // Run in Standby
    CLKSRCidx = $06;  CLKSRCbm = $40;  // Clock Source Selection
    EDGEDETidx = $07;  EDGEDETbm = $80;  // Edge Detection Enable
    FILTSEL0idx = $04;  // Filter Selection
    FILTSEL1idx = $05;  // Filter Selection
    OUTENidx = $03;  OUTENbm = $08;  // Output Enable
    INSEL00idx = $00;  // LUT Input 0 Source Selection
    INSEL01idx = $01;  // LUT Input 0 Source Selection
    INSEL02idx = $02;  // LUT Input 0 Source Selection
    INSEL03idx = $03;  // LUT Input 0 Source Selection
    INSEL10idx = $04;  // LUT Input 1 Source Selection
    INSEL11idx = $05;  // LUT Input 1 Source Selection
    INSEL12idx = $06;  // LUT Input 1 Source Selection
    INSEL13idx = $07;  // LUT Input 1 Source Selection
    INSEL20idx = $00;  // LUT Input 2 Source Selection
    INSEL21idx = $01;  // LUT Input 2 Source Selection
    INSEL22idx = $02;  // LUT Input 2 Source Selection
    INSEL23idx = $03;  // LUT Input 2 Source Selection
    SEQSEL0idx = $00;  // Sequential Selection
    SEQSEL1idx = $01;  // Sequential Selection
    SEQSEL2idx = $02;  // Sequential Selection
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
    CLKOUTidx = $07;  CLKOUTbm = $80;  // System clock out
    CLKSEL0idx = $00;  // clock select
    CLKSEL1idx = $01;  // clock select
    PDIV0idx = $01;  // Prescaler division
    PDIV1idx = $02;  // Prescaler division
    PDIV2idx = $03;  // Prescaler division
    PDIV3idx = $04;  // Prescaler division
    PENidx = $00;  PENbm = $01;  // Prescaler enable
    LOCKENidx = $00;  LOCKENbm = $01;  // lock ebable
    EXTSidx = $07;  EXTSbm = $80;  // External Clock status
    OSC20MSidx = $04;  OSC20MSbm = $10;  // 20MHz oscillator status
    OSC32KSidx = $05;  OSC32KSbm = $20;  // 32KHz oscillator status
    SOSCidx = $00;  SOSCbm = $01;  // System Oscillator changing
    XOSC32KSidx = $06;  XOSC32KSbm = $40;  // 32.768 kHz Crystal Oscillator status
    CAL20M0idx = $00;  // Calibration
    CAL20M1idx = $01;  // Calibration
    CAL20M2idx = $02;  // Calibration
    CAL20M3idx = $03;  // Calibration
    CAL20M4idx = $04;  // Calibration
    CAL20M5idx = $05;  // Calibration
    LOCKidx = $07;  LOCKbm = $80;  // Lock
    TEMPCAL20M0idx = $00;  // Oscillator temperature coefficient
    TEMPCAL20M1idx = $01;  // Oscillator temperature coefficient
    TEMPCAL20M2idx = $02;  // Oscillator temperature coefficient
    TEMPCAL20M3idx = $03;  // Oscillator temperature coefficient
    RUNSTDBYidx = $01;  RUNSTDBYbm = $02;  // Run standby
    CSUT0idx = $04;  // Crystal startup time
    CSUT1idx = $05;  // Crystal startup time
    ENABLEidx = $00;  ENABLEbm = $01;  // Enable
    SELidx = $02;  SELbm = $04;  // Select
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
    CCP0idx = $00;  // CCP signature
    CCP1idx = $01;  // CCP signature
    CCP2idx = $02;  // CCP signature
    CCP3idx = $03;  // CCP signature
    CCP4idx = $04;  // CCP signature
    CCP5idx = $05;  // CCP signature
    CCP6idx = $06;  // CCP signature
    CCP7idx = $07;  // CCP signature
    Cidx = $00;  Cbm = $01;  // Carry Flag
    Hidx = $05;  Hbm = $20;  // Half Carry Flag
    Iidx = $07;  Ibm = $80;  // Global Interrupt Enable Flag
    Nidx = $02;  Nbm = $04;  // Negative Flag
    Sidx = $04;  Sbm = $10;  // N Exclusive Or V Flag
    Tidx = $06;  Tbm = $40;  // Transfer Bit
    Vidx = $03;  Vbm = $08;  // Two's Complement Overflow Flag
    Zidx = $01;  Zbm = $02;  // Zero Flag
  end;

  TCPUINT = object //Interrupt Controller
    CTRLA: byte;  //Control A
    STATUS: byte;  //Status
    LVL0PRI: byte;  //Interrupt Level 0 Priority
    LVL1VEC: byte;  //Interrupt Level 1 Priority Vector
  const
    CVTidx = $05;  CVTbm = $20;  // Compact Vector Table
    IVSELidx = $06;  IVSELbm = $40;  // Interrupt Vector Select
    LVL0RRidx = $00;  LVL0RRbm = $01;  // Round-robin Scheduling Enable
    LVL0PRI0idx = $00;  // Interrupt Level Priority
    LVL0PRI1idx = $01;  // Interrupt Level Priority
    LVL0PRI2idx = $02;  // Interrupt Level Priority
    LVL0PRI3idx = $03;  // Interrupt Level Priority
    LVL0PRI4idx = $04;  // Interrupt Level Priority
    LVL0PRI5idx = $05;  // Interrupt Level Priority
    LVL0PRI6idx = $06;  // Interrupt Level Priority
    LVL0PRI7idx = $07;  // Interrupt Level Priority
    LVL1VEC0idx = $00;  // Interrupt Vector with High Priority
    LVL1VEC1idx = $01;  // Interrupt Vector with High Priority
    LVL1VEC2idx = $02;  // Interrupt Vector with High Priority
    LVL1VEC3idx = $03;  // Interrupt Vector with High Priority
    LVL1VEC4idx = $04;  // Interrupt Vector with High Priority
    LVL1VEC5idx = $05;  // Interrupt Vector with High Priority
    LVL1VEC6idx = $06;  // Interrupt Vector with High Priority
    LVL1VEC7idx = $07;  // Interrupt Vector with High Priority
    LVL0EXidx = $00;  LVL0EXbm = $01;  // Level 0 Interrupt Executing
    LVL1EXidx = $01;  LVL1EXbm = $02;  // Level 1 Interrupt Executing
    NMIEXidx = $07;  NMIEXbm = $80;  // Non-maskable Interrupt Executing
  end;

  TCRCSCAN = object //CRCSCAN
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    STATUS: byte;  //Status
  const
    ENABLEidx = $00;  ENABLEbm = $01;  // Enable CRC scan
    NMIENidx = $01;  NMIENbm = $02;  // Enable NMI Trigger
    RESETidx = $07;  RESETbm = $80;  // Reset CRC scan
    MODE0idx = $04;  // CRC Flash Access Mode
    MODE1idx = $05;  // CRC Flash Access Mode
    SRC0idx = $00;  // CRC Source
    SRC1idx = $01;  // CRC Source
    BUSYidx = $00;  BUSYbm = $01;  // CRC Busy
    OKidx = $01;  OKbm = $02;  // CRC Ok
  end;

  TDAC = object //Digital to Analog Converter
    CTRLA: byte;  //Control Register A
    DATA: byte;  //DATA Register
  const
    ENABLEidx = $00;  ENABLEbm = $01;  // DAC Enable
    OUTENidx = $06;  OUTENbm = $40;  // Output Buffer Enable
    RUNSTDBYidx = $07;  RUNSTDBYbm = $80;  // Run in Standby Mode
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
    ASYNCCH00idx = $00;  // Asynchronous Channel 0 Generator Selection
    ASYNCCH01idx = $01;  // Asynchronous Channel 0 Generator Selection
    ASYNCCH02idx = $02;  // Asynchronous Channel 0 Generator Selection
    ASYNCCH03idx = $03;  // Asynchronous Channel 0 Generator Selection
    ASYNCCH04idx = $04;  // Asynchronous Channel 0 Generator Selection
    ASYNCCH05idx = $05;  // Asynchronous Channel 0 Generator Selection
    ASYNCCH06idx = $06;  // Asynchronous Channel 0 Generator Selection
    ASYNCCH07idx = $07;  // Asynchronous Channel 0 Generator Selection
    ASYNCCH10idx = $00;  // Asynchronous Channel 1 Generator Selection
    ASYNCCH11idx = $01;  // Asynchronous Channel 1 Generator Selection
    ASYNCCH12idx = $02;  // Asynchronous Channel 1 Generator Selection
    ASYNCCH13idx = $03;  // Asynchronous Channel 1 Generator Selection
    ASYNCCH14idx = $04;  // Asynchronous Channel 1 Generator Selection
    ASYNCCH15idx = $05;  // Asynchronous Channel 1 Generator Selection
    ASYNCCH16idx = $06;  // Asynchronous Channel 1 Generator Selection
    ASYNCCH17idx = $07;  // Asynchronous Channel 1 Generator Selection
    ASYNCCH20idx = $00;  // Asynchronous Channel 2 Generator Selection
    ASYNCCH21idx = $01;  // Asynchronous Channel 2 Generator Selection
    ASYNCCH22idx = $02;  // Asynchronous Channel 2 Generator Selection
    ASYNCCH23idx = $03;  // Asynchronous Channel 2 Generator Selection
    ASYNCCH24idx = $04;  // Asynchronous Channel 2 Generator Selection
    ASYNCCH25idx = $05;  // Asynchronous Channel 2 Generator Selection
    ASYNCCH26idx = $06;  // Asynchronous Channel 2 Generator Selection
    ASYNCCH27idx = $07;  // Asynchronous Channel 2 Generator Selection
    ASYNCCH30idx = $00;  // Asynchronous Channel 3 Generator Selection
    ASYNCCH31idx = $01;  // Asynchronous Channel 3 Generator Selection
    ASYNCCH32idx = $02;  // Asynchronous Channel 3 Generator Selection
    ASYNCCH33idx = $03;  // Asynchronous Channel 3 Generator Selection
    ASYNCCH34idx = $04;  // Asynchronous Channel 3 Generator Selection
    ASYNCCH35idx = $05;  // Asynchronous Channel 3 Generator Selection
    ASYNCCH36idx = $06;  // Asynchronous Channel 3 Generator Selection
    ASYNCCH37idx = $07;  // Asynchronous Channel 3 Generator Selection
    ASYNCUSER00idx = $00;  // Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER01idx = $01;  // Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER02idx = $02;  // Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER03idx = $03;  // Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER04idx = $04;  // Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER05idx = $05;  // Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER06idx = $06;  // Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER07idx = $07;  // Asynchronous User Ch 0 Input Selection - TCB0
    ASYNCUSER10idx = $00;  // Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER11idx = $01;  // Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER12idx = $02;  // Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER13idx = $03;  // Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER14idx = $04;  // Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER15idx = $05;  // Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER16idx = $06;  // Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER17idx = $07;  // Asynchronous User Ch 1 Input Selection - ADC0
    ASYNCUSER20idx = $00;  // Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER21idx = $01;  // Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER22idx = $02;  // Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER23idx = $03;  // Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER24idx = $04;  // Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER25idx = $05;  // Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER26idx = $06;  // Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER27idx = $07;  // Asynchronous User Ch 2 Input Selection - CCL LUT0 Event 0
    ASYNCUSER30idx = $00;  // Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER31idx = $01;  // Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER32idx = $02;  // Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER33idx = $03;  // Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER34idx = $04;  // Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER35idx = $05;  // Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER36idx = $06;  // Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER37idx = $07;  // Asynchronous User Ch 3 Input Selection - CCL LUT1 Event 0
    ASYNCUSER40idx = $00;  // Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER41idx = $01;  // Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER42idx = $02;  // Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER43idx = $03;  // Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER44idx = $04;  // Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER45idx = $05;  // Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER46idx = $06;  // Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER47idx = $07;  // Asynchronous User Ch 4 Input Selection - CCL LUT0 Event 1
    ASYNCUSER50idx = $00;  // Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER51idx = $01;  // Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER52idx = $02;  // Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER53idx = $03;  // Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER54idx = $04;  // Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER55idx = $05;  // Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER56idx = $06;  // Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER57idx = $07;  // Asynchronous User Ch 5 Input Selection - CCL LUT1 Event 1
    ASYNCUSER60idx = $00;  // Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER61idx = $01;  // Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER62idx = $02;  // Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER63idx = $03;  // Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER64idx = $04;  // Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER65idx = $05;  // Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER66idx = $06;  // Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER67idx = $07;  // Asynchronous User Ch 6 Input Selection - TCD0 Event 0
    ASYNCUSER70idx = $00;  // Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER71idx = $01;  // Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER72idx = $02;  // Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER73idx = $03;  // Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER74idx = $04;  // Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER75idx = $05;  // Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER76idx = $06;  // Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER77idx = $07;  // Asynchronous User Ch 7 Input Selection - TCD0 Event 1
    ASYNCUSER80idx = $00;  // Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER81idx = $01;  // Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER82idx = $02;  // Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER83idx = $03;  // Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER84idx = $04;  // Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER85idx = $05;  // Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER86idx = $06;  // Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER87idx = $07;  // Asynchronous User Ch 8 Input Selection - Event Out 0
    ASYNCUSER90idx = $00;  // Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER91idx = $01;  // Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER92idx = $02;  // Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER93idx = $03;  // Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER94idx = $04;  // Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER95idx = $05;  // Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER96idx = $06;  // Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER97idx = $07;  // Asynchronous User Ch 9 Input Selection - Event Out 1
    ASYNCUSER100idx = $00;  // Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER101idx = $01;  // Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER102idx = $02;  // Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER103idx = $03;  // Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER104idx = $04;  // Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER105idx = $05;  // Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER106idx = $06;  // Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER107idx = $07;  // Asynchronous User Ch 10 Input Selection - Event Out 2
    ASYNCUSER110idx = $00;  // Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER111idx = $01;  // Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER112idx = $02;  // Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER113idx = $03;  // Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER114idx = $04;  // Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER115idx = $05;  // Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER116idx = $06;  // Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER117idx = $07;  // Asynchronous User Ch 11 Input Selection - TCB1
    ASYNCUSER120idx = $00;  // Asynchronous User Ch 12 Input Selection - ADC0
    ASYNCUSER121idx = $01;  // Asynchronous User Ch 12 Input Selection - ADC0
    ASYNCUSER122idx = $02;  // Asynchronous User Ch 12 Input Selection - ADC0
    ASYNCUSER123idx = $03;  // Asynchronous User Ch 12 Input Selection - ADC0
    ASYNCUSER124idx = $04;  // Asynchronous User Ch 12 Input Selection - ADC0
    ASYNCUSER125idx = $05;  // Asynchronous User Ch 12 Input Selection - ADC0
    ASYNCUSER126idx = $06;  // Asynchronous User Ch 12 Input Selection - ADC0
    ASYNCUSER127idx = $07;  // Asynchronous User Ch 12 Input Selection - ADC0
    SYNCCH00idx = $00;  // Synchronous Channel 0 Generator Selection
    SYNCCH01idx = $01;  // Synchronous Channel 0 Generator Selection
    SYNCCH02idx = $02;  // Synchronous Channel 0 Generator Selection
    SYNCCH03idx = $03;  // Synchronous Channel 0 Generator Selection
    SYNCCH04idx = $04;  // Synchronous Channel 0 Generator Selection
    SYNCCH05idx = $05;  // Synchronous Channel 0 Generator Selection
    SYNCCH06idx = $06;  // Synchronous Channel 0 Generator Selection
    SYNCCH07idx = $07;  // Synchronous Channel 0 Generator Selection
    SYNCCH10idx = $00;  // Synchronous Channel 1 Generator Selection
    SYNCCH11idx = $01;  // Synchronous Channel 1 Generator Selection
    SYNCCH12idx = $02;  // Synchronous Channel 1 Generator Selection
    SYNCCH13idx = $03;  // Synchronous Channel 1 Generator Selection
    SYNCCH14idx = $04;  // Synchronous Channel 1 Generator Selection
    SYNCCH15idx = $05;  // Synchronous Channel 1 Generator Selection
    SYNCCH16idx = $06;  // Synchronous Channel 1 Generator Selection
    SYNCCH17idx = $07;  // Synchronous Channel 1 Generator Selection
    SYNCUSER00idx = $00;  // Synchronous User Ch 0 - TCA0
    SYNCUSER01idx = $01;  // Synchronous User Ch 0 - TCA0
    SYNCUSER02idx = $02;  // Synchronous User Ch 0 - TCA0
    SYNCUSER03idx = $03;  // Synchronous User Ch 0 - TCA0
    SYNCUSER04idx = $04;  // Synchronous User Ch 0 - TCA0
    SYNCUSER05idx = $05;  // Synchronous User Ch 0 - TCA0
    SYNCUSER06idx = $06;  // Synchronous User Ch 0 - TCA0
    SYNCUSER07idx = $07;  // Synchronous User Ch 0 - TCA0
    SYNCUSER10idx = $00;  // Synchronous User Ch 1 - USART0
    SYNCUSER11idx = $01;  // Synchronous User Ch 1 - USART0
    SYNCUSER12idx = $02;  // Synchronous User Ch 1 - USART0
    SYNCUSER13idx = $03;  // Synchronous User Ch 1 - USART0
    SYNCUSER14idx = $04;  // Synchronous User Ch 1 - USART0
    SYNCUSER15idx = $05;  // Synchronous User Ch 1 - USART0
    SYNCUSER16idx = $06;  // Synchronous User Ch 1 - USART0
    SYNCUSER17idx = $07;  // Synchronous User Ch 1 - USART0
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
    ACTIVE0idx = $02;  // BOD Operation in Active Mode
    ACTIVE1idx = $03;  // BOD Operation in Active Mode
    LVL0idx = $05;  // BOD Level
    LVL1idx = $06;  // BOD Level
    LVL2idx = $07;  // BOD Level
    SAMPFREQidx = $04;  SAMPFREQbm = $10;  // BOD Sample Frequency
    SLEEP0idx = $00;  // BOD Operation in Sleep Mode
    SLEEP1idx = $01;  // BOD Operation in Sleep Mode
    FREQSEL0idx = $00;  // Frequency Select
    FREQSEL1idx = $01;  // Frequency Select
    OSCLOCKidx = $07;  OSCLOCKbm = $80;  // Oscillator Lock
    CRCSRC0idx = $06;  // CRC Source
    CRCSRC1idx = $07;  // CRC Source
    EESAVEidx = $00;  EESAVEbm = $01;  // EEPROM Save
    RSTPINCFG0idx = $02;  // Reset Pin Configuration
    RSTPINCFG1idx = $03;  // Reset Pin Configuration
    SUT0idx = $00;  // Startup Time
    SUT1idx = $01;  // Startup Time
    SUT2idx = $02;  // Startup Time
    CMPAidx = $00;  CMPAbm = $01;  // Compare A Default Output Value
    CMPAENidx = $04;  CMPAENbm = $10;  // Compare A Output Enable
    CMPBidx = $01;  CMPBbm = $02;  // Compare B Default Output Value
    CMPBENidx = $05;  CMPBENbm = $20;  // Compare B Output Enable
    CMPCidx = $02;  CMPCbm = $04;  // Compare C Default Output Value
    CMPCENidx = $06;  CMPCENbm = $40;  // Compare C Output Enable
    CMPDidx = $03;  CMPDbm = $08;  // Compare D Default Output Value
    CMPDENidx = $07;  CMPDENbm = $80;  // Compare D Output Enable
    PERIOD0idx = $00;  // Watchdog Timeout Period
    PERIOD1idx = $01;  // Watchdog Timeout Period
    PERIOD2idx = $02;  // Watchdog Timeout Period
    PERIOD3idx = $03;  // Watchdog Timeout Period
    WINDOW0idx = $04;  // Watchdog Window Timeout Period
    WINDOW1idx = $05;  // Watchdog Window Timeout Period
    WINDOW2idx = $06;  // Watchdog Window Timeout Period
    WINDOW3idx = $07;  // Watchdog Window Timeout Period
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
    LB0idx = $00;  // Lock Bits
    LB1idx = $01;  // Lock Bits
    LB2idx = $02;  // Lock Bits
    LB3idx = $03;  // Lock Bits
    LB4idx = $04;  // Lock Bits
    LB5idx = $05;  // Lock Bits
    LB6idx = $06;  // Lock Bits
    LB7idx = $07;  // Lock Bits
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
    CMD0idx = $00;  // Command
    CMD1idx = $01;  // Command
    CMD2idx = $02;  // Command
    APCWPidx = $00;  APCWPbm = $01;  // Application code write protect
    BOOTLOCKidx = $01;  BOOTLOCKbm = $02;  // Boot Lock
    EEREADYidx = $00;  EEREADYbm = $01;  // EEPROM Ready
    EEBUSYidx = $01;  EEBUSYbm = $02;  // EEPROM busy
    FBUSYidx = $00;  FBUSYbm = $01;  // Flash busy
    WRERRORidx = $02;  WRERRORbm = $04;  // Write error
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
    INT0idx = $00;  // Pin Interrupt
    INT1idx = $01;  // Pin Interrupt
    INT2idx = $02;  // Pin Interrupt
    INT3idx = $03;  // Pin Interrupt
    INT4idx = $04;  // Pin Interrupt
    INT5idx = $05;  // Pin Interrupt
    INT6idx = $06;  // Pin Interrupt
    INT7idx = $07;  // Pin Interrupt
    INVENidx = $07;  INVENbm = $80;  // Inverted I/O Enable
    ISC0idx = $00;  // Input/Sense Configuration
    ISC1idx = $01;  // Input/Sense Configuration
    ISC2idx = $02;  // Input/Sense Configuration
    PULLUPENidx = $03;  PULLUPENbm = $08;  // Pullup enable
  end;

  TPORTMUX = object //Port Multiplexer
    CTRLA: byte;  //Port Multiplexer Control A
    CTRLB: byte;  //Port Multiplexer Control B
    CTRLC: byte;  //Port Multiplexer Control C
    CTRLD: byte;  //Port Multiplexer Control D
  const
    EVOUT0idx = $00;  EVOUT0bm = $01;  // Event Output 0
    EVOUT1idx = $01;  EVOUT1bm = $02;  // Event Output 1
    EVOUT2idx = $02;  EVOUT2bm = $04;  // Event Output 2
    LUT0idx = $04;  LUT0bm = $10;  // Configurable Custom Logic LUT0
    LUT1idx = $05;  LUT1bm = $20;  // Configurable Custom Logic LUT1
    SPI0idx = $02;  SPI0bm = $04;  // Port Multiplexer SPI0
    TWI0idx = $04;  TWI0bm = $10;  // Port Multiplexer TWI0
    USART0idx = $00;  USART0bm = $01;  // Port Multiplexer USART0
    TCA00idx = $00;  TCA00bm = $01;  // Port Multiplexer TCA0 Output 0
    TCA01idx = $01;  TCA01bm = $02;  // Port Multiplexer TCA0 Output 1
    TCA02idx = $02;  TCA02bm = $04;  // Port Multiplexer TCA0 Output 2
    TCA03idx = $03;  TCA03bm = $08;  // Port Multiplexer TCA0 Output 3
    TCA04idx = $04;  TCA04bm = $10;  // Port Multiplexer TCA0 Output 4
    TCA05idx = $05;  TCA05bm = $20;  // Port Multiplexer TCA0 Output 5
    TCB0idx = $00;  TCB0bm = $01;  // Port Multiplexer TCB
  end;

  TRSTCTRL = object //Reset controller
    RSTFR: byte;  //Reset Flags
    SWRR: byte;  //Software Reset
  const
    BORFidx = $01;  BORFbm = $02;  // Brown out detector Reset flag
    EXTRFidx = $02;  EXTRFbm = $04;  // External Reset flag
    PORFidx = $00;  PORFbm = $01;  // Power on Reset flag
    SWRFidx = $04;  SWRFbm = $10;  // Software Reset flag
    UPDIRFidx = $05;  UPDIRFbm = $20;  // UPDI Reset flag
    WDRFidx = $03;  WDRFbm = $08;  // Watch dog Reset flag
    SWREidx = $00;  SWREbm = $01;  // Software reset enable
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
    CLKSEL0idx = $00;  // Clock Select
    CLKSEL1idx = $01;  // Clock Select
    PRESCALER0idx = $03;  // Prescaling Factor
    PRESCALER1idx = $04;  // Prescaling Factor
    PRESCALER2idx = $05;  // Prescaling Factor
    PRESCALER3idx = $06;  // Prescaling Factor
    RTCENidx = $00;  RTCENbm = $01;  // Enable
    RUNSTDBYidx = $07;  RUNSTDBYbm = $80;  // Run In Standby
    DBGRUNidx = $00;  DBGRUNbm = $01;  // Run in debug
    CMPidx = $01;  CMPbm = $02;  // Compare Match Interrupt enable
    OVFidx = $00;  OVFbm = $01;  // Overflow Interrupt enable
    PERIOD0idx = $03;  // Period
    PERIOD1idx = $04;  // Period
    PERIOD2idx = $05;  // Period
    PERIOD3idx = $06;  // Period
    PITENidx = $00;  PITENbm = $01;  // Enable
    PIidx = $00;  PIbm = $01;  // Periodic Interrupt
    CTRLBUSYidx = $00;  CTRLBUSYbm = $01;  // CTRLA Synchronization Busy Flag
    CMPBUSYidx = $03;  CMPBUSYbm = $08;  // Comparator Synchronization Busy Flag
    CNTBUSYidx = $01;  CNTBUSYbm = $02;  // Count Synchronization Busy Flag
    CTRLABUSYidx = $00;  CTRLABUSYbm = $01;  // CTRLA Synchronization Busy Flag
    PERBUSYidx = $02;  PERBUSYbm = $04;  // Period Synchronization Busy Flag
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
    SENidx = $00;  SENbm = $01;  // Sleep enable
    SMODE0idx = $01;  // Sleep mode
    SMODE1idx = $02;  // Sleep mode
  end;

  TSPI = object //Serial Peripheral Interface
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    INTCTRL: byte;  //Interrupt Control
    INTFLAGS: byte;  //Interrupt Flags
    DATA: byte;  //Data
  const
    CLK2Xidx = $04;  CLK2Xbm = $10;  // Enable Double Speed
    DORDidx = $06;  DORDbm = $40;  // Data Order Setting
    ENABLEidx = $00;  ENABLEbm = $01;  // Enable Module
    MASTERidx = $05;  MASTERbm = $20;  // Master Operation Enable
    PRESC0idx = $01;  // Prescaler
    PRESC1idx = $02;  // Prescaler
    BUFENidx = $07;  BUFENbm = $80;  // Buffer Mode Enable
    BUFWRidx = $06;  BUFWRbm = $40;  // Buffer Write Mode
    MODE0idx = $00;  // SPI Mode
    MODE1idx = $01;  // SPI Mode
    SSDidx = $02;  SSDbm = $04;  // Slave Select Disable
    DREIEidx = $05;  DREIEbm = $20;  // Data Register Empty Interrupt Enable
    IEidx = $00;  IEbm = $01;  // Interrupt Enable
    RXCIEidx = $07;  RXCIEbm = $80;  // Receive Complete Interrupt Enable
    SSIEidx = $04;  SSIEbm = $10;  // Slave Select Trigger Interrupt Enable
    TXCIEidx = $06;  TXCIEbm = $40;  // Transfer Complete Interrupt Enable
  end;

  TSYSCFG = object //System Configuration Registers
    Reserved0: byte;
    REVID: byte;  //Revision ID
    EXTBRK: byte;  //External Break
  const
    ENEXTBRKidx = $00;  ENEXTBRKbm = $01;  // External break enable
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
    CLKSEL0idx = $01;  // Clock Selection
    CLKSEL1idx = $02;  // Clock Selection
    CLKSEL2idx = $03;  // Clock Selection
    ENABLEidx = $00;  ENABLEbm = $01;  // Module Enable
    ALUPDidx = $03;  ALUPDbm = $08;  // Auto Lock Update
    CMP0ENidx = $04;  CMP0ENbm = $10;  // Compare 0 Enable
    CMP1ENidx = $05;  CMP1ENbm = $20;  // Compare 1 Enable
    CMP2ENidx = $06;  CMP2ENbm = $40;  // Compare 2 Enable
    WGMODE0idx = $00;  // Waveform generation mode
    WGMODE1idx = $01;  // Waveform generation mode
    WGMODE2idx = $02;  // Waveform generation mode
    CMP0OVidx = $00;  CMP0OVbm = $01;  // Compare 0 Waveform Output Value
    CMP1OVidx = $01;  CMP1OVbm = $02;  // Compare 1 Waveform Output Value
    CMP2OVidx = $02;  CMP2OVbm = $04;  // Compare 2 Waveform Output Value
    SPLITMidx = $00;  SPLITMbm = $01;  // Split Mode Enable
    CMD0idx = $02;  // Command
    CMD1idx = $03;  // Command
    DIRidx = $00;  DIRbm = $01;  // Direction
    LUPDidx = $01;  LUPDbm = $02;  // Lock Update
    CMP0BVidx = $01;  CMP0BVbm = $02;  // Compare 0 Buffer Valid
    CMP1BVidx = $02;  CMP1BVbm = $04;  // Compare 1 Buffer Valid
    CMP2BVidx = $03;  CMP2BVbm = $08;  // Compare 2 Buffer Valid
    PERBVidx = $00;  PERBVbm = $01;  // Period Buffer Valid
    DBGRUNidx = $00;  DBGRUNbm = $01;  // Debug Run
    CNTEIidx = $00;  CNTEIbm = $01;  // Count on Event Input
    EVACT0idx = $01;  // Event Action
    EVACT1idx = $02;  // Event Action
    CMP0idx = $04;  CMP0bm = $10;  // Compare 0 Interrupt
    CMP1idx = $05;  CMP1bm = $20;  // Compare 1 Interrupt
    CMP2idx = $06;  CMP2bm = $40;  // Compare 2 Interrupt
    OVFidx = $00;  OVFbm = $01;  // Overflow Interrupt
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
    CLKSEL0idx = $01;  // Clock Selection
    CLKSEL1idx = $02;  // Clock Selection
    CLKSEL2idx = $03;  // Clock Selection
    ENABLEidx = $00;  ENABLEbm = $01;  // Module Enable
    HCMP0ENidx = $04;  HCMP0ENbm = $10;  // High Compare 0 Enable
    HCMP1ENidx = $05;  HCMP1ENbm = $20;  // High Compare 1 Enable
    HCMP2ENidx = $06;  HCMP2ENbm = $40;  // High Compare 2 Enable
    LCMP0ENidx = $00;  LCMP0ENbm = $01;  // Low Compare 0 Enable
    LCMP1ENidx = $01;  LCMP1ENbm = $02;  // Low Compare 1 Enable
    LCMP2ENidx = $02;  LCMP2ENbm = $04;  // Low Compare 2 Enable
    HCMP0OVidx = $04;  HCMP0OVbm = $10;  // High Compare 0 Output Value
    HCMP1OVidx = $05;  HCMP1OVbm = $20;  // High Compare 1 Output Value
    HCMP2OVidx = $06;  HCMP2OVbm = $40;  // High Compare 2 Output Value
    LCMP0OVidx = $00;  LCMP0OVbm = $01;  // Low Compare 0 Output Value
    LCMP1OVidx = $01;  LCMP1OVbm = $02;  // Low Compare 1 Output Value
    LCMP2OVidx = $02;  LCMP2OVbm = $04;  // Low Compare 2 Output Value
    SPLITMidx = $00;  SPLITMbm = $01;  // Split Mode Enable
    CMD0idx = $02;  // Command
    CMD1idx = $03;  // Command
    DBGRUNidx = $00;  DBGRUNbm = $01;  // Debug Run
    HUNFidx = $01;  HUNFbm = $02;  // High Underflow Interrupt Enable
    LCMP0idx = $04;  LCMP0bm = $10;  // Low Compare 0 Interrupt Enable
    LCMP1idx = $05;  LCMP1bm = $20;  // Low Compare 1 Interrupt Enable
    LCMP2idx = $06;  LCMP2bm = $40;  // Low Compare 2 Interrupt Enable
    LUNFidx = $00;  LUNFbm = $01;  // Low Underflow Interrupt Enable
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
    CLKSEL0idx = $01;  // Clock Select
    CLKSEL1idx = $02;  // Clock Select
    ENABLEidx = $00;  ENABLEbm = $01;  // Enable
    RUNSTDBYidx = $06;  RUNSTDBYbm = $40;  // Run Standby
    SYNCUPDidx = $04;  SYNCUPDbm = $10;  // Synchronize Update
    ASYNCidx = $06;  ASYNCbm = $40;  // Asynchronous Enable
    CCMPENidx = $04;  CCMPENbm = $10;  // Pin Output Enable
    CCMPINITidx = $05;  CCMPINITbm = $20;  // Pin Initial State
    CNTMODE0idx = $00;  // Timer Mode
    CNTMODE1idx = $01;  // Timer Mode
    CNTMODE2idx = $02;  // Timer Mode
    DBGRUNidx = $00;  DBGRUNbm = $01;  // Debug Run
    CAPTEIidx = $00;  CAPTEIbm = $01;  // Event Input Enable
    EDGEidx = $04;  EDGEbm = $10;  // Event Edge
    FILTERidx = $06;  FILTERbm = $40;  // Input Capture Noise Cancellation Filter
    CAPTidx = $00;  CAPTbm = $01;  // Capture or Timeout
    RUNidx = $00;  RUNbm = $01;  // Run
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
    CLKSEL0idx = $05;  // clock select
    CLKSEL1idx = $06;  // clock select
    CNTPRES0idx = $03;  // counter prescaler
    CNTPRES1idx = $04;  // counter prescaler
    ENABLEidx = $00;  ENABLEbm = $01;  // Enable
    SYNCPRES0idx = $01;  // Syncronization prescaler
    SYNCPRES1idx = $02;  // Syncronization prescaler
    WGMODE0idx = $00;  // Waveform generation mode
    WGMODE1idx = $01;  // Waveform generation mode
    AUPDATEidx = $01;  AUPDATEbm = $02;  // Auto update
    CMPCSELidx = $06;  CMPCSELbm = $40;  // Compare C output select
    CMPDSELidx = $07;  CMPDSELbm = $80;  // Compare D output select
    CMPOVRidx = $00;  CMPOVRbm = $01;  // Compare output value override
    FIFTYidx = $03;  FIFTYbm = $08;  // Fifty percent waveform
    CMPAVAL0idx = $00;  // Compare A value
    CMPAVAL1idx = $01;  // Compare A value
    CMPAVAL2idx = $02;  // Compare A value
    CMPAVAL3idx = $03;  // Compare A value
    CMPBVAL0idx = $04;  // Compare B value
    CMPBVAL1idx = $05;  // Compare B value
    CMPBVAL2idx = $06;  // Compare B value
    CMPBVAL3idx = $07;  // Compare B value
    DISEOCidx = $07;  DISEOCbm = $80;  // Disable at end of cycle
    RESTARTidx = $02;  RESTARTbm = $04;  // Restart strobe
    SCAPTUREAidx = $03;  SCAPTUREAbm = $08;  // Software Capture A Strobe
    SCAPTUREBidx = $04;  SCAPTUREBbm = $10;  // Software Capture B Strobe
    SYNCidx = $01;  SYNCbm = $02;  // synchronize strobe
    SYNCEOCidx = $00;  SYNCEOCbm = $01;  // synchronize end of cycle strobe
    DBGRUNidx = $00;  DBGRUNbm = $01;  // Debug run
    FAULTDETidx = $02;  FAULTDETbm = $04;  // Fault detection
    DITHERSEL0idx = $00;  // dither select
    DITHERSEL1idx = $01;  // dither select
    DITHER0idx = $00;  // Dither value
    DITHER1idx = $01;  // Dither value
    DITHER2idx = $02;  // Dither value
    DITHER3idx = $03;  // Dither value
    DLYPRESC0idx = $04;  // Delay prescaler
    DLYPRESC1idx = $05;  // Delay prescaler
    DLYSEL0idx = $00;  // Delay select
    DLYSEL1idx = $01;  // Delay select
    DLYTRIG0idx = $02;  // Delay trigger
    DLYTRIG1idx = $03;  // Delay trigger
    DLYVAL0idx = $00;  // Delay value
    DLYVAL1idx = $01;  // Delay value
    DLYVAL2idx = $02;  // Delay value
    DLYVAL3idx = $03;  // Delay value
    DLYVAL4idx = $04;  // Delay value
    DLYVAL5idx = $05;  // Delay value
    DLYVAL6idx = $06;  // Delay value
    DLYVAL7idx = $07;  // Delay value
    ACTIONidx = $02;  ACTIONbm = $04;  // event action
    CFG0idx = $06;  // event config
    CFG1idx = $07;  // event config
    EDGEidx = $04;  EDGEbm = $10;  // edge select
    TRIGEIidx = $00;  TRIGEIbm = $01;  // Trigger event enable
    CMPAidx = $00;  CMPAbm = $01;  // Compare A value
    CMPAENidx = $04;  CMPAENbm = $10;  // Compare A enable
    CMPBidx = $01;  CMPBbm = $02;  // Compare B value
    CMPBENidx = $05;  CMPBENbm = $20;  // Compare B enable
    CMPCidx = $02;  CMPCbm = $04;  // Compare C value
    CMPCENidx = $06;  CMPCENbm = $40;  // Compare C enable
    CMPDidx = $03;  CMPDbm = $08;  // Compare D vaule
    CMPDENidx = $07;  CMPDENbm = $80;  // Compare D enable
    INPUTMODE0idx = $00;  // Input mode
    INPUTMODE1idx = $01;  // Input mode
    INPUTMODE2idx = $02;  // Input mode
    INPUTMODE3idx = $03;  // Input mode
    OVFidx = $00;  OVFbm = $01;  // Overflow interrupt enable
    TRIGAidx = $02;  TRIGAbm = $04;  // Trigger A interrupt enable
    TRIGBidx = $03;  TRIGBbm = $08;  // Trigger B interrupt enable
    CMDRDYidx = $01;  CMDRDYbm = $02;  // Command ready
    ENRDYidx = $00;  ENRDYbm = $01;  // Enable ready
    PWMACTAidx = $06;  PWMACTAbm = $40;  // PWM activity on A
    PWMACTBidx = $07;  PWMACTBbm = $80;  // PWM activity on B
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
    ENABLEidx = $00;  ENABLEbm = $01;  // Enable TWI Master
    QCENidx = $04;  QCENbm = $10;  // Quick Command Enable
    RIENidx = $07;  RIENbm = $80;  // Read Interrupt Enable
    SMENidx = $01;  SMENbm = $02;  // Smart Mode Enable
    TIMEOUT0idx = $02;  // Inactive Bus Timeout
    TIMEOUT1idx = $03;  // Inactive Bus Timeout
    WIENidx = $06;  WIENbm = $40;  // Write Interrupt Enable
    ACKACTidx = $02;  ACKACTbm = $04;  // Acknowledge Action
    FLUSHidx = $03;  FLUSHbm = $08;  // Flush
    MCMD0idx = $00;  // Command
    MCMD1idx = $01;  // Command
    ARBLOSTidx = $03;  ARBLOSTbm = $08;  // Arbitration Lost
    BUSERRidx = $02;  BUSERRbm = $04;  // Bus Error
    BUSSTATE0idx = $00;  // Bus State
    BUSSTATE1idx = $01;  // Bus State
    CLKHOLDidx = $05;  CLKHOLDbm = $20;  // Clock Hold
    RIFidx = $07;  RIFbm = $80;  // Read Interrupt Flag
    RXACKidx = $04;  RXACKbm = $10;  // Received Acknowledge
    WIFidx = $06;  WIFbm = $40;  // Write Interrupt Flag
    ADDRENidx = $00;  ADDRENbm = $01;  // Address Enable
    ADDRMASK0idx = $01;  // Address Mask
    ADDRMASK1idx = $02;  // Address Mask
    ADDRMASK2idx = $03;  // Address Mask
    ADDRMASK3idx = $04;  // Address Mask
    ADDRMASK4idx = $05;  // Address Mask
    ADDRMASK5idx = $06;  // Address Mask
    ADDRMASK6idx = $07;  // Address Mask
    APIENidx = $06;  APIENbm = $40;  // Address/Stop Interrupt Enable
    DIENidx = $07;  DIENbm = $80;  // Data Interrupt Enable
    PIENidx = $05;  PIENbm = $20;  // Stop Interrupt Enable
    PMENidx = $02;  PMENbm = $04;  // Promiscuous Mode Enable
    SCMD0idx = $00;  // Command
    SCMD1idx = $01;  // Command
    APidx = $00;  APbm = $01;  // Slave Address or Stop
    APIFidx = $06;  APIFbm = $40;  // Address/Stop Interrupt Flag
    COLLidx = $03;  COLLbm = $08;  // Collision
    DIFidx = $07;  DIFbm = $80;  // Data Interrupt Flag
    DIRidx = $01;  DIRbm = $02;  // Read/Write Direction
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
    ABEIEidx = $02;  ABEIEbm = $04;  // Auto-baud Error Interrupt Enable
    DREIEidx = $05;  DREIEbm = $20;  // Data Register Empty Interrupt Enable
    LBMEidx = $03;  LBMEbm = $08;  // Loop-back Mode Enable
    RS4850idx = $00;  // RS485 Mode internal transmitter
    RS4851idx = $01;  // RS485 Mode internal transmitter
    RXCIEidx = $07;  RXCIEbm = $80;  // Receive Complete Interrupt Enable
    RXSIEidx = $04;  RXSIEbm = $10;  // Receiver Start Frame Interrupt Enable
    TXCIEidx = $06;  TXCIEbm = $40;  // Transmit Complete Interrupt Enable
    MPCMidx = $00;  MPCMbm = $01;  // Multi-processor Communication Mode
    ODMEidx = $03;  ODMEbm = $08;  // Open Drain Mode Enable
    RXENidx = $07;  RXENbm = $80;  // Reciever enable
    RXMODE0idx = $01;  // Receiver Mode
    RXMODE1idx = $02;  // Receiver Mode
    SFDENidx = $04;  SFDENbm = $10;  // Start Frame Detection Enable
    TXENidx = $06;  TXENbm = $40;  // Transmitter Enable
    ABMBPidx = $07;  ABMBPbm = $80;  // Autobaud majority voter bypass
    DBGRUNidx = $00;  DBGRUNbm = $01;  // Debug Run
    IREIidx = $00;  IREIbm = $01;  // IrDA Event Input Enable
    BUFOVFidx = $06;  BUFOVFbm = $40;  // Buffer Overflow
    DATA8idx = $00;  DATA8bm = $01;  // Receiver Data Register
    FERRidx = $02;  FERRbm = $04;  // Frame Error
    PERRidx = $01;  PERRbm = $02;  // Parity Error
    RXCIFidx = $07;  RXCIFbm = $80;  // Receive Complete Interrupt Flag
    DATA0idx = $00;  // RX Data
    DATA1idx = $01;  // RX Data
    DATA2idx = $02;  // RX Data
    DATA3idx = $03;  // RX Data
    DATA4idx = $04;  // RX Data
    DATA5idx = $05;  // RX Data
    DATA6idx = $06;  // RX Data
    DATA7idx = $07;  // RX Data
    RXPL0idx = $00;  // Receiver Pulse Lenght
    RXPL1idx = $01;  // Receiver Pulse Lenght
    RXPL2idx = $02;  // Receiver Pulse Lenght
    RXPL3idx = $03;  // Receiver Pulse Lenght
    RXPL4idx = $04;  // Receiver Pulse Lenght
    RXPL5idx = $05;  // Receiver Pulse Lenght
    RXPL6idx = $06;  // Receiver Pulse Lenght
    BDFidx = $01;  BDFbm = $02;  // Break Detected Flag
    DREIFidx = $05;  DREIFbm = $20;  // Data Register Empty Flag
    ISFIFidx = $03;  ISFIFbm = $08;  // Inconsistent Sync Field Interrupt Flag
    RXSIFidx = $04;  RXSIFbm = $10;  // Receive Start Interrupt
    TXCIFidx = $06;  TXCIFbm = $40;  // Transmit Interrupt Flag
    WFBidx = $00;  WFBbm = $01;  // Wait For Break
    TXPL0idx = $00;  // Transmit pulse length
    TXPL1idx = $01;  // Transmit pulse length
    TXPL2idx = $02;  // Transmit pulse length
    TXPL3idx = $03;  // Transmit pulse length
    TXPL4idx = $04;  // Transmit pulse length
    TXPL5idx = $05;  // Transmit pulse length
    TXPL6idx = $06;  // Transmit pulse length
    TXPL7idx = $07;  // Transmit pulse length
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
    USERROW32: byte;  //User Row Byte 32
    USERROW33: byte;  //User Row Byte 33
    USERROW34: byte;  //User Row Byte 34
    USERROW35: byte;  //User Row Byte 35
    USERROW36: byte;  //User Row Byte 36
    USERROW37: byte;  //User Row Byte 37
    USERROW38: byte;  //User Row Byte 38
    USERROW39: byte;  //User Row Byte 39
    USERROW40: byte;  //User Row Byte 40
    USERROW41: byte;  //User Row Byte 41
    USERROW42: byte;  //User Row Byte 42
    USERROW43: byte;  //User Row Byte 43
    USERROW44: byte;  //User Row Byte 44
    USERROW45: byte;  //User Row Byte 45
    USERROW46: byte;  //User Row Byte 46
    USERROW47: byte;  //User Row Byte 47
    USERROW48: byte;  //User Row Byte 48
    USERROW49: byte;  //User Row Byte 49
    USERROW50: byte;  //User Row Byte 50
    USERROW51: byte;  //User Row Byte 51
    USERROW52: byte;  //User Row Byte 52
    USERROW53: byte;  //User Row Byte 53
    USERROW54: byte;  //User Row Byte 54
    USERROW55: byte;  //User Row Byte 55
    USERROW56: byte;  //User Row Byte 56
    USERROW57: byte;  //User Row Byte 57
    USERROW58: byte;  //User Row Byte 58
    USERROW59: byte;  //User Row Byte 59
    USERROW60: byte;  //User Row Byte 60
    USERROW61: byte;  //User Row Byte 61
    USERROW62: byte;  //User Row Byte 62
    USERROW63: byte;  //User Row Byte 63
  end;

  TVPORT = object //Virtual Ports
    DIR: byte;  //Data Direction
    OUT_: byte;  //Output Value
    IN_: byte;  //Input Value
    INTFLAGS: byte;  //Interrupt Flags
  const
    INT0idx = $00;  // Pin Interrupt
    INT1idx = $01;  // Pin Interrupt
    INT2idx = $02;  // Pin Interrupt
    INT3idx = $03;  // Pin Interrupt
    INT4idx = $04;  // Pin Interrupt
    INT5idx = $05;  // Pin Interrupt
    INT6idx = $06;  // Pin Interrupt
    INT7idx = $07;  // Pin Interrupt
  end;

  TVREF = object //Voltage reference
    CTRLA: byte;  //Control A
    CTRLB: byte;  //Control B
    CTRLC: byte;  //Control C
    CTRLD: byte;  //Control D
  const
    ADC0REFSEL0idx = $04;  // ADC0 reference select
    ADC0REFSEL1idx = $05;  // ADC0 reference select
    ADC0REFSEL2idx = $06;  // ADC0 reference select
    DAC0REFSEL0idx = $00;  // DAC0/AC0 reference select
    DAC0REFSEL1idx = $01;  // DAC0/AC0 reference select
    DAC0REFSEL2idx = $02;  // DAC0/AC0 reference select
    ADC0REFENidx = $01;  ADC0REFENbm = $02;  // ADC0 reference enable
    ADC1REFENidx = $04;  ADC1REFENbm = $10;  // ADC1 reference enable
    DAC0REFENidx = $00;  DAC0REFENbm = $01;  // DAC0/AC0 reference enable
    DAC1REFENidx = $03;  DAC1REFENbm = $08;  // DAC1/AC1 reference enable
    DAC2REFENidx = $05;  DAC2REFENbm = $20;  // DAC2/AC2 reference enable
    ADC1REFSEL0idx = $04;  // ADC1 reference select
    ADC1REFSEL1idx = $05;  // ADC1 reference select
    ADC1REFSEL2idx = $06;  // ADC1 reference select
    DAC1REFSEL0idx = $00;  // DAC1/AC1 reference select
    DAC1REFSEL1idx = $01;  // DAC1/AC1 reference select
    DAC1REFSEL2idx = $02;  // DAC1/AC1 reference select
    DAC2REFSEL0idx = $00;  // DAC2/AC2 reference select
    DAC2REFSEL1idx = $01;  // DAC2/AC2 reference select
    DAC2REFSEL2idx = $02;  // DAC2/AC2 reference select
  end;

  TWDT = object //Watch-Dog Timer
    CTRLA: byte;  //Control A
    STATUS: byte;  //Status
  const
    PERIOD0idx = $00;  // Period
    PERIOD1idx = $01;  // Period
    PERIOD2idx = $02;  // Period
    PERIOD3idx = $03;  // Period
    WINDOW0idx = $04;  // Window
    WINDOW1idx = $05;  // Window
    WINDOW2idx = $06;  // Window
    WINDOW3idx = $07;  // Window
    LOCKidx = $07;  LOCKbm = $80;  // Lock enable
    SYNCBUSYidx = $00;  SYNCBUSYbm = $01;  // Syncronization busy
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
  PORTC: TPORT absolute $0440;
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
procedure PORTC_PORT_ISR; external name 'PORTC_PORT_ISR'; // Interrupt 5 
procedure RTC_CNT_ISR; external name 'RTC_CNT_ISR'; // Interrupt 6 
procedure RTC_PIT_ISR; external name 'RTC_PIT_ISR'; // Interrupt 7 
procedure TCA0_OVF_ISR; external name 'TCA0_OVF_ISR'; // Interrupt 8 
//procedure TCA0_LUNF_ISR; external name 'TCA0_LUNF_ISR'; // Interrupt 8 
procedure TCA0_HUNF_ISR; external name 'TCA0_HUNF_ISR'; // Interrupt 9 
procedure TCA0_CMP0_ISR; external name 'TCA0_CMP0_ISR'; // Interrupt 10 
//procedure TCA0_LCMP0_ISR; external name 'TCA0_LCMP0_ISR'; // Interrupt 10 
procedure TCA0_LCMP1_ISR; external name 'TCA0_LCMP1_ISR'; // Interrupt 11 
//procedure TCA0_CMP1_ISR; external name 'TCA0_CMP1_ISR'; // Interrupt 11 
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
  jmp PORTC_PORT_ISR
  jmp RTC_CNT_ISR
  jmp RTC_PIT_ISR
  jmp TCA0_OVF_ISR
//  jmp TCA0_LUNF_ISR
  jmp TCA0_HUNF_ISR
  jmp TCA0_CMP0_ISR
//  jmp TCA0_LCMP0_ISR
  jmp TCA0_LCMP1_ISR
//  jmp TCA0_CMP1_ISR
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
  .weak PORTC_PORT_ISR
  .weak RTC_CNT_ISR
  .weak RTC_PIT_ISR
  .weak TCA0_OVF_ISR
//  .weak TCA0_LUNF_ISR
  .weak TCA0_HUNF_ISR
  .weak TCA0_CMP0_ISR
//  .weak TCA0_LCMP0_ISR
  .weak TCA0_LCMP1_ISR
//  .weak TCA0_CMP1_ISR
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
  .set PORTC_PORT_ISR, Default_IRQ_handler
  .set RTC_CNT_ISR, Default_IRQ_handler
  .set RTC_PIT_ISR, Default_IRQ_handler
  .set TCA0_OVF_ISR, Default_IRQ_handler
//  .set TCA0_LUNF_ISR, Default_IRQ_handler
  .set TCA0_HUNF_ISR, Default_IRQ_handler
  .set TCA0_CMP0_ISR, Default_IRQ_handler
//  .set TCA0_LCMP0_ISR, Default_IRQ_handler
  .set TCA0_LCMP1_ISR, Default_IRQ_handler
//  .set TCA0_CMP1_ISR, Default_IRQ_handler
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
