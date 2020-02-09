unit ATmega64HVE2;

{$goto on}
interface

var
  PINA: byte absolute $20;  // Port A Input Pins
  DDRA: byte absolute $21;  // Port A Data Direction Register
  PORTA: byte absolute $22;  // Port A Data Register
  PINB: byte absolute $23;  // Port B Input Pins
  DDRB: byte absolute $24;  // Port B Data Direction Register
  PORTB: byte absolute $25;  // Port B Data Register
  TIFR0: byte absolute $35;  // Timer/Counter Interrupt Flag register
  TIFR1: byte absolute $36;  // Timer/Counter Interrupt Flag register
  PCIFR: byte absolute $3B;  // Pin Change Interrupt Flag Register
  EIFR: byte absolute $3C;  // External Interrupt Flag Register
  EIMSK: byte absolute $3D;  // External Interrupt Mask Register
  GPIOR0: byte absolute $3E;  // General Purpose IO Register 0
  EECR: byte absolute $3F;  // EEPROM Control Register
  EEDR: byte absolute $40;  // EEPROM Data Register
  EEAR: word absolute $41;  // EEPROM Read/Write Access
  EEARL: byte absolute $41;  // EEPROM Read/Write Access
  EEARH: byte absolute $42;  // EEPROM Read/Write Access;
  GTCCR: byte absolute $43;  // General Timer/Counter Control Register
  TCCR0A: byte absolute $44;  // Timer/Counter 0 Control Register A
  TCCR0B: byte absolute $45;  // Timer/Counter0 Control Register B
  TCNT0: word absolute $46;  // Timer Counter 0  Bytes
  TCNT0L: byte absolute $46;  // Timer Counter 0  Bytes
  TCNT0H: byte absolute $47;  // Timer Counter 0  Bytes;
  OCR0A: byte absolute $48;  // Output Compare Register 0A
  OCR0B: byte absolute $49;  // Output Compare Register B
  GPIOR1: byte absolute $4A;  // General Purpose IO Register 1
  GPIOR2: byte absolute $4B;  // General Purpose IO Register 2
  SPCR: byte absolute $4C;  // SPI Control Register
  SPSR: byte absolute $4D;  // SPI Status Register
  SPDR: byte absolute $4E;  // SPI Data Register
  SMCR: byte absolute $53;  // Sleep Mode Control Register
  MCUSR: byte absolute $54;  // MCU Status Register
  MCUCR: byte absolute $55;  // MCU Control Register
  SPMCSR: byte absolute $57;  // Store Program Memory Control and Status Register
  SP: word absolute $5D;  // Stack Pointer 
  SPL: byte absolute $5D;  // Stack Pointer 
  SPH: byte absolute $5E;  // Stack Pointer ;
  SREG: byte absolute $5F;  // Status Register
  WDTCSR: byte absolute $60;  // Watchdog Timer Control Register
  CLKPR: byte absolute $61;  // Clock Prescale Register
  WUTCSR: byte absolute $62;  // Wake-up Timer Control and Status Register
  WDTCLR: byte absolute $63;  // Watchdog Timer Configuration Lock Register
  PRR0: byte absolute $64;  // Power Reduction Register 0
  SOSCCALA: byte absolute $66;  // Slow Oscillator Calibration Register A
  SOSCCALB: byte absolute $67;  // Oscillator Calibration Register B
  PCICR: byte absolute $68;  // Pin Change Interrupt Control Register
  EICRA: byte absolute $69;  // External Interrupt Control Register
  PCMSK0: byte absolute $6B;  // Pin Change Enable Mask Register 0
  PCMSK1: byte absolute $6C;  // Pin Change Enable Mask Register 1
  TIMSK0: byte absolute $6E;  // Timer/Counter Interrupt Mask Register
  TIMSK1: byte absolute $6F;  // Timer/Counter Interrupt Mask Register
  DIDR0: byte absolute $7E;  // Digital Input Disable Register
  TCCR1A: byte absolute $80;  // Timer/Counter 1 Control Register A
  TCCR1B: byte absolute $81;  // Timer/Counter1 Control Register B
  TCNT1: word absolute $84;  // Timer Counter 1  Bytes
  TCNT1L: byte absolute $84;  // Timer Counter 1  Bytes
  TCNT1H: byte absolute $85;  // Timer Counter 1  Bytes;
  OCR1A: byte absolute $88;  // Output Compare Register 1A
  OCR1B: byte absolute $89;  // Output Compare Register B
  LINCR: byte absolute $C0;  // LIN Control Register
  LINSIR: byte absolute $C1;  // LIN Status and Interrupt Register
  LINENIR: byte absolute $C2;  // LIN Enable Interrupt Register
  LINERR: byte absolute $C3;  // LIN Error Register
  LINBTR: byte absolute $C4;  // LIN Bit Timing Register
  LINBRRL: byte absolute $C5;  // LIN Baud Rate Low Register
  LINBRRH: byte absolute $C6;  // LIN Baud Rate High Register
  LINDLR: byte absolute $C7;  // LIN Data Length Register
  LINIDR: byte absolute $C8;  // LIN Identifier Register
  LINSEL: byte absolute $C9;  // LIN Data Buffer Selection Register
  LINDAT: byte absolute $CA;  // LIN Data Register
  BGCSRA: byte absolute $D1;  // Bandgap Control and Status Register A
  BGCRB: byte absolute $D2;  // Band Gap Calibration Register B
  BGCRA: byte absolute $D3;  // Band Gap Calibration Register A
  BGLR: byte absolute $D4;  // Band Gap Lock Register
  PLLCSR: byte absolute $D8;  // PLL Control and Status Register
  PBOV: byte absolute $DC;  // Port B Override
  ADSCSRA: byte absolute $E0;  // ADC Synchronization Control and Status Register
  ADSCSRB: byte absolute $E1;  // ADC Synchronization Control and Status Register
  ADCRA: byte absolute $E2;  // ADC Control Register A
  ADCRB: byte absolute $E3;  // ADC Control Register B
  ADCRC: byte absolute $E4;  // ADC Control Register B
  ADCRD: byte absolute $E5;  // ADC Control Register D
  ADCRE: byte absolute $E6;  // ADC Control Register E
  ADIFR: byte absolute $E7;  // ADC Interrupt Flag Register
  ADIMR: byte absolute $E8;  // ADC Interrupt Mask Register
  CADRCL: word absolute $E9;  // CC-ADC Regulator Current Comparator Threshold Level
  CADRCLL: byte absolute $E9;  // CC-ADC Regulator Current Comparator Threshold Level
  CADRCLH: byte absolute $EA;  // CC-ADC Regulator Current Comparator Threshold Level;
  CADIC: word absolute $EB;  // C-ADC Instantaneous Conversion Result
  CADICL: byte absolute $EB;  // C-ADC Instantaneous Conversion Result
  CADICH: byte absolute $EC;  // C-ADC Instantaneous Conversion Result;
  CADAC0: byte absolute $ED;  // C-ADC Accumulated Conversion Result
  CADAC1: byte absolute $EE;  // C-ADC Accumulated Conversion Result
  CADAC2: byte absolute $EF;  // C-ADC Accumulated Conversion Result
  CADAC3: byte absolute $F0;  // C-ADC Accumulated Conversion Result
  VADIC: word absolute $F1;  // V-ADC Instantaneous Conversion Result
  VADICL: byte absolute $F1;  // V-ADC Instantaneous Conversion Result
  VADICH: byte absolute $F2;  // V-ADC Instantaneous Conversion Result;
  VADAC0: byte absolute $F3;  // V-ADC Accumulated Conversion Result
  VADAC1: byte absolute $F4;  // V-ADC Accumulated Conversion Result
  VADAC2: byte absolute $F5;  // V-ADC Accumulated Conversion Result
  VADAC3: byte absolute $F6;  // V-ADC Accumulated Conversion Result

const
  // Port A Data Register
  PA0 = $00;  
  PA1 = $01;  
  // Port B Data Register
  PB0 = $00;  
  PB1 = $01;  
  PB2 = $02;  
  PB3 = $03;  
  PB4 = $04;  
  PB5 = $05;  
  PB6 = $06;  
  PB7 = $07;  
  // Timer/Counter Interrupt Flag register
  TOV0 = $00;  
  OCF0A = $01;  
  OCF0B = $02;  
  ICF0 = $03;  
  // Timer/Counter Interrupt Flag register
  TOV1 = $00;  
  OCF1A = $01;  
  OCF1B = $02;  
  ICF1 = $03;  
  // Pin Change Interrupt Flag Register
  PCIF0 = $00;  // Pin Change Interrupt Flags
  PCIF1 = $01;  // Pin Change Interrupt Flags
  // External Interrupt Flag Register
  INTF0 = $00;  
  // External Interrupt Mask Register
  INT0 = $00;  
  // EEPROM Control Register
  EERE = $00;  
  EEPE = $01;  
  EEMPE = $02;  
  EERIE = $03;  
  EEPM0 = $04;
  EEPM1 = $05;
  // General Timer/Counter Control Register
  PSRSYNC = $00;  
  TSM = $07;  
  // Timer/Counter 0 Control Register A
  WGM00 = $00;  
  ICS0 = $03;  
  ICES0 = $04;  
  ICNC0 = $05;  
  ICEN0 = $06;  
  TCW0 = $07;  
  // Timer/Counter0 Control Register B
  CS00 = $00;  
  CS01 = $01;  
  CS02 = $02;  
  // SPI Control Register
  SPR0 = $00;  // SPI Clock Rate Selects
  SPR1 = $01;  // SPI Clock Rate Selects
  CPHA = $02;  
  CPOL = $03;  
  MSTR = $04;  
  DORD = $05;  
  SPE = $06;  
  SPIE = $07;  
  // SPI Status Register
  SPI2X = $00;  
  WCOL = $06;  
  SPIF = $07;  
  // Sleep Mode Control Register
  SE = $00;  
  SM0 = $01;  // Sleep Mode Select bits
  SM1 = $02;  // Sleep Mode Select bits
  SM2 = $03;  // Sleep Mode Select bits
  // MCU Status Register
  PORF = $00;  
  EXTRF = $01;  
  BODRF = $02;  
  WDRF = $03;  
  OCDRF = $04;  
  // MCU Control Register
  IVCE = $00;  
  IVSEL = $01;  
  PUD = $04;  
  CKOE = $05;  
  // Store Program Memory Control and Status Register
  SPMEN = $00;  
  PGERS = $01;  
  PGWRT = $02;  
  LBSET = $03;  
  RWWSRE = $04;  
  SIGRD = $05;  
  RWWSB = $06;  
  SPMIE = $07;  
  // Status Register
  C = $00;  
  Z = $01;  
  N = $02;  
  V = $03;  
  S = $04;  
  H = $05;  
  T = $06;  
  I = $07;  
  // Watchdog Timer Control Register
  WDE = $03;  
  WDCE = $04;  
  WDP0 = $00;  // Watchdog Timer Prescaler Bits
  WDP1 = $01;  // Watchdog Timer Prescaler Bits
  WDP2 = $02;  // Watchdog Timer Prescaler Bits
  WDP3 = $05;  // Watchdog Timer Prescaler Bits
  WDIE = $06;  
  WDIF = $07;  
  // Clock Prescale Register
  CLKPS0 = $00;  // Clock Prescaler Select Bits
  CLKPS1 = $01;  // Clock Prescaler Select Bits
  CLKPCE = $07;  
  // Wake-up Timer Control and Status Register
  WUTP0 = $00;  // Wake-up Timer Prescaler Bits
  WUTP1 = $01;  // Wake-up Timer Prescaler Bits
  WUTP2 = $02;  // Wake-up Timer Prescaler Bits
  WUTE = $03;  
  WUTR = $04;  
  WUTIE = $06;  
  WUTIF = $07;  
  // Watchdog Timer Configuration Lock Register
  WDCLE = $00;  
  WDCL0 = $01;  // Watchdog Timer Comfiguration Lock bits
  WDCL1 = $02;  // Watchdog Timer Comfiguration Lock bits
  // Power Reduction Register 0
  PRTIM0 = $00;  
  PRTIM1 = $01;  
  PRSPI = $02;  
  PRLIN = $03;  
  // Pin Change Interrupt Control Register
  PCIE0 = $00;  // Pin Change Interrupt Enables
  PCIE1 = $01;  // Pin Change Interrupt Enables
  // External Interrupt Control Register
  ISC00 = $00;  
  ISC01 = $01;  
  // Timer/Counter Interrupt Mask Register
  TOIE0 = $00;  
  OCIE0A = $01;  
  OCIE0B = $02;  
  ICIE0 = $03;  
  // Timer/Counter Interrupt Mask Register
  TOIE1 = $00;  
  OCIE1A = $01;  
  OCIE1B = $02;  
  ICIE1 = $03;  
  // Digital Input Disable Register
  PA0DID = $00;  
  PA1DID = $01;  
  // Timer/Counter 1 Control Register A
  WGM10 = $00;  
  ICS1 = $03;  
  ICES1 = $04;  
  ICNC1 = $05;  
  ICEN1 = $06;  
  TCW1 = $07;  
  // Timer/Counter1 Control Register B
  CS10 = $00;  // Clock Select1 bis
  CS11 = $01;  // Clock Select1 bis
  CS12 = $02;  // Clock Select1 bis
  // LIN Control Register
  LCMD0 = $00;  // LIN Command and Mode bits
  LCMD1 = $01;  // LIN Command and Mode bits
  LCMD2 = $02;  // LIN Command and Mode bits
  LENA = $03;  
  LCONF0 = $04;  // LIN Configuration bits
  LCONF1 = $05;  // LIN Configuration bits
  LIN13 = $06;  
  LSWRES = $07;  
  // LIN Status and Interrupt Register
  LRXOK = $00;  
  LTXOK = $01;  
  LIDOK = $02;  
  LERR = $03;  
  LBUSY = $04;  
  LIDST0 = $05;  // Identifier Status bits
  LIDST1 = $06;  // Identifier Status bits
  LIDST2 = $07;  // Identifier Status bits
  // LIN Enable Interrupt Register
  LENRXOK = $00;  
  LENTXOK = $01;  
  LENIDOK = $02;  
  LENERR = $03;  
  // LIN Error Register
  LBERR = $00;  
  LCERR = $01;  
  LPERR = $02;  
  LSERR = $03;  
  LFERR = $04;  
  LOVERR = $05;  
  LTOERR = $06;  
  LABORT = $07;  
  // LIN Bit Timing Register
  LBT0 = $00;  // LIN Bit Timing bits
  LBT1 = $01;  // LIN Bit Timing bits
  LBT2 = $02;  // LIN Bit Timing bits
  LBT3 = $03;  // LIN Bit Timing bits
  LBT4 = $04;  // LIN Bit Timing bits
  LBT5 = $05;  // LIN Bit Timing bits
  LDISR = $07;  
  // LIN Baud Rate Low Register
  LDIV0 = $00;
  LDIV1 = $01;
  LDIV2 = $02;
  LDIV3 = $03;
  LDIV4 = $04;
  LDIV5 = $05;
  LDIV6 = $06;
  LDIV7 = $07;
  // LIN Data Length Register
  LRXDL0 = $00;  // LIN Receive Data Length bits
  LRXDL1 = $01;  // LIN Receive Data Length bits
  LRXDL2 = $02;  // LIN Receive Data Length bits
  LRXDL3 = $03;  // LIN Receive Data Length bits
  LTXDL0 = $04;  // LIN Transmit Data Length bits
  LTXDL1 = $05;  // LIN Transmit Data Length bits
  LTXDL2 = $06;  // LIN Transmit Data Length bits
  LTXDL3 = $07;  // LIN Transmit Data Length bits
  // LIN Identifier Register
  LID0 = $00;  // Identifier bit 5 or Data Length bits
  LID1 = $01;  // Identifier bit 5 or Data Length bits
  LID2 = $02;  // Identifier bit 5 or Data Length bits
  LID3 = $03;  // Identifier bit 5 or Data Length bits
  LID4 = $04;  // Identifier bit 5 or Data Length bits
  LID5 = $05;  // Identifier bit 5 or Data Length bits
  LP0 = $06;  // Parity bits
  LP1 = $07;  // Parity bits
  // LIN Data Buffer Selection Register
  LINDX0 = $00;  // FIFO LIN Data Buffer Index bits
  LINDX1 = $01;  // FIFO LIN Data Buffer Index bits
  LINDX2 = $02;  // FIFO LIN Data Buffer Index bits
  LAINC = $03;  
  // LIN Data Register
  LDATA0 = $00;
  LDATA1 = $01;
  LDATA2 = $02;
  LDATA3 = $03;
  LDATA4 = $04;
  LDATA5 = $05;
  LDATA6 = $06;
  LDATA7 = $07;
  // Bandgap Control and Status Register A
  BGSC0 = $00;  // Band Gap Sample Configuration
  BGSC1 = $01;  // Band Gap Sample Configuration
  BGSC2 = $02;  // Band Gap Sample Configuration
  // Band Gap Calibration Register B
  BGCL0 = $00;  // Band Gap Calibration Linear
  BGCL1 = $01;  // Band Gap Calibration Linear
  BGCL2 = $02;  // Band Gap Calibration Linear
  BGCL3 = $03;  // Band Gap Calibration Linear
  BGCL4 = $04;  // Band Gap Calibration Linear
  BGCL5 = $05;  // Band Gap Calibration Linear
  BGCL6 = $06;  // Band Gap Calibration Linear
  BGCL7 = $07;  // Band Gap Calibration Linear
  // Band Gap Calibration Register A
  BGCN0 = $00;  // Band Gap Calibration Nominal
  BGCN1 = $01;  // Band Gap Calibration Nominal
  BGCN2 = $02;  // Band Gap Calibration Nominal
  BGCN3 = $03;  // Band Gap Calibration Nominal
  BGCN4 = $04;  // Band Gap Calibration Nominal
  BGCN5 = $05;  // Band Gap Calibration Nominal
  BGCN6 = $06;  // Band Gap Calibration Nominal
  BGCN7 = $07;  // Band Gap Calibration Nominal
  // Band Gap Lock Register
  BGPL = $00;  
  BGPLE = $01;  
  // PLL Control and Status Register
  PLLCIE = $00;  
  PLLCIF = $01;  
  LOCK = $04;  
  SWEN = $05;  
  // Port B Override
  PBOE0 = $00;  
  PBOE3 = $03;  
  PBOVCE = $07;  
  // ADC Synchronization Control and Status Register
  SCMD0 = $00;  // Synchronization Command
  SCMD1 = $01;  // Synchronization Command
  SBSY = $02;  
  // ADC Synchronization Control and Status Register
  CADICRB = $00;  
  CADACRB = $01;  
  CADICPS = $02;  
  VADICRB = $04;  
  VADACRB = $05;  
  VADICPS = $06;  
  // ADC Control Register A
  CKSEL = $00;  
  ADCMS0 = $01;  // C-ADC Chopper Mode Select
  ADCMS1 = $02;  // C-ADC Chopper Mode Select
  ADPSEL = $03;  
  // ADC Control Register B
  ADADES0 = $00;  // Accumulated Decimation Ratio Select
  ADADES1 = $01;  // Accumulated Decimation Ratio Select
  ADADES2 = $02;  // Accumulated Decimation Ratio Select
  ADIDES0 = $03;  // Instantaneous Decimation Ratio Select
  ADIDES1 = $04;  // Instantaneous Decimation Ratio Select
  // ADC Control Register B
  CADRCT0 = $00;  // C-ADC Regular Current Count Threshold
  CADRCT1 = $01;  // C-ADC Regular Current Count Threshold
  CADRCT2 = $02;  // C-ADC Regular Current Count Threshold
  CADRCT3 = $03;  // C-ADC Regular Current Count Threshold
  CADRCM0 = $04;  // C-ADC Regular Current Comparator Mode
  CADRCM1 = $05;  // C-ADC Regular Current Comparator Mode
  CADEN = $07;  
  // ADC Control Register D
  CADDSEL = $00;  
  CADPDM0 = $01;  // C-ADC Pin Diagnostics Mode
  CADPDM1 = $02;  // C-ADC Pin Diagnostics Mode
  CADG0 = $03;  // C-ADC Gain
  CADG1 = $04;  // C-ADC Gain
  CADG2 = $05;  // C-ADC Gain
  // ADC Control Register E
  VADMUX0 = $00;  // V-ADC Channel Select
  VADMUX1 = $01;  // V-ADC Channel Select
  VADMUX2 = $02;  // V-ADC Channel Select
  VADPDM0 = $03;  // V-ADC Pin Diagnostics Mode
  VADPDM1 = $04;  // V-ADC Pin Diagnostics Mode
  VADREFS = $05;  
  VADEN = $07;  
  // ADC Interrupt Flag Register
  CADICIF = $00;  
  CADACIF = $01;  
  CADRCIF = $02;  
  VADICIF = $04;  
  VADACIF = $05;  
  // ADC Interrupt Mask Register
  CADICIE = $00;  
  CADACIE = $01;  
  CADRCIE = $02;  
  VADICIE = $04;  
  VADACIE = $05;  


implementation

{$i avrcommon.inc}

procedure INT0_ISR; external name 'INT0_ISR'; // Interrupt 1 External Interrupt 0
procedure PCINT0_ISR; external name 'PCINT0_ISR'; // Interrupt 2 Pin Change Interrupt 0
procedure PCINT1_ISR; external name 'PCINT1_ISR'; // Interrupt 3 Pin Change Interrupt 1
procedure WDT_ISR; external name 'WDT_ISR'; // Interrupt 4 Watchdog Timeout Interrupt
procedure WAKEUP_ISR; external name 'WAKEUP_ISR'; // Interrupt 5 Wakeup Timer Overflow
procedure TIMER1_IC_ISR; external name 'TIMER1_IC_ISR'; // Interrupt 6 Timer 1 Input capture
procedure TIMER1_COMPA_ISR; external name 'TIMER1_COMPA_ISR'; // Interrupt 7 Timer 1 Compare Match A
procedure TIMER1_COMPB_ISR; external name 'TIMER1_COMPB_ISR'; // Interrupt 8 Timer 1 Compare Match B
procedure TIMER1_OVF_ISR; external name 'TIMER1_OVF_ISR'; // Interrupt 9 Timer 1 overflow
procedure TIMER0_IC_ISR; external name 'TIMER0_IC_ISR'; // Interrupt 10 Timer 0 Input Capture
procedure TIMER0_COMPA_ISR; external name 'TIMER0_COMPA_ISR'; // Interrupt 11 Timer 0 Comapre Match A
procedure TIMER0_COMPB_ISR; external name 'TIMER0_COMPB_ISR'; // Interrupt 12 Timer 0 Compare Match B
procedure TIMER0_OVF_ISR; external name 'TIMER0_OVF_ISR'; // Interrupt 13 Timer 0 Overflow
procedure LIN_STATUS_ISR; external name 'LIN_STATUS_ISR'; // Interrupt 14 LIN Status Interrupt
procedure LIN_ERROR_ISR; external name 'LIN_ERROR_ISR'; // Interrupt 15 LIN Error Interrupt
procedure SPI_STC_ISR; external name 'SPI_STC_ISR'; // Interrupt 16 SPI Serial transfer complete
procedure VADC_CONV_ISR; external name 'VADC_CONV_ISR'; // Interrupt 17 Voltage ADC Instantaneous Conversion Complete
procedure VADC_ACC_ISR; external name 'VADC_ACC_ISR'; // Interrupt 18 Voltage ADC Accumulated Conversion Complete
procedure CADC_CONV_ISR; external name 'CADC_CONV_ISR'; // Interrupt 19 C-ADC Instantaneous Conversion Complete
procedure CADC_REG_CUR_ISR; external name 'CADC_REG_CUR_ISR'; // Interrupt 20 C-ADC Regular Current
procedure CADC_ACC_ISR; external name 'CADC_ACC_ISR'; // Interrupt 21 C-ADC Accumulated Conversion Complete
procedure EE_READY_ISR; external name 'EE_READY_ISR'; // Interrupt 22 EEPROM Ready
procedure SPM_ISR; external name 'SPM_ISR'; // Interrupt 23 SPM Ready
procedure PLL_ISR; external name 'PLL_ISR'; // Interrupt 24 PLL Lock Change Interrupt

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  jmp _start
  jmp INT0_ISR
  jmp PCINT0_ISR
  jmp PCINT1_ISR
  jmp WDT_ISR
  jmp WAKEUP_ISR
  jmp TIMER1_IC_ISR
  jmp TIMER1_COMPA_ISR
  jmp TIMER1_COMPB_ISR
  jmp TIMER1_OVF_ISR
  jmp TIMER0_IC_ISR
  jmp TIMER0_COMPA_ISR
  jmp TIMER0_COMPB_ISR
  jmp TIMER0_OVF_ISR
  jmp LIN_STATUS_ISR
  jmp LIN_ERROR_ISR
  jmp SPI_STC_ISR
  jmp VADC_CONV_ISR
  jmp VADC_ACC_ISR
  jmp CADC_CONV_ISR
  jmp CADC_REG_CUR_ISR
  jmp CADC_ACC_ISR
  jmp EE_READY_ISR
  jmp SPM_ISR
  jmp PLL_ISR

  {$i start.inc}

  .weak INT0_ISR
  .weak PCINT0_ISR
  .weak PCINT1_ISR
  .weak WDT_ISR
  .weak WAKEUP_ISR
  .weak TIMER1_IC_ISR
  .weak TIMER1_COMPA_ISR
  .weak TIMER1_COMPB_ISR
  .weak TIMER1_OVF_ISR
  .weak TIMER0_IC_ISR
  .weak TIMER0_COMPA_ISR
  .weak TIMER0_COMPB_ISR
  .weak TIMER0_OVF_ISR
  .weak LIN_STATUS_ISR
  .weak LIN_ERROR_ISR
  .weak SPI_STC_ISR
  .weak VADC_CONV_ISR
  .weak VADC_ACC_ISR
  .weak CADC_CONV_ISR
  .weak CADC_REG_CUR_ISR
  .weak CADC_ACC_ISR
  .weak EE_READY_ISR
  .weak SPM_ISR
  .weak PLL_ISR

  .set INT0_ISR, Default_IRQ_handler
  .set PCINT0_ISR, Default_IRQ_handler
  .set PCINT1_ISR, Default_IRQ_handler
  .set WDT_ISR, Default_IRQ_handler
  .set WAKEUP_ISR, Default_IRQ_handler
  .set TIMER1_IC_ISR, Default_IRQ_handler
  .set TIMER1_COMPA_ISR, Default_IRQ_handler
  .set TIMER1_COMPB_ISR, Default_IRQ_handler
  .set TIMER1_OVF_ISR, Default_IRQ_handler
  .set TIMER0_IC_ISR, Default_IRQ_handler
  .set TIMER0_COMPA_ISR, Default_IRQ_handler
  .set TIMER0_COMPB_ISR, Default_IRQ_handler
  .set TIMER0_OVF_ISR, Default_IRQ_handler
  .set LIN_STATUS_ISR, Default_IRQ_handler
  .set LIN_ERROR_ISR, Default_IRQ_handler
  .set SPI_STC_ISR, Default_IRQ_handler
  .set VADC_CONV_ISR, Default_IRQ_handler
  .set VADC_ACC_ISR, Default_IRQ_handler
  .set CADC_CONV_ISR, Default_IRQ_handler
  .set CADC_REG_CUR_ISR, Default_IRQ_handler
  .set CADC_ACC_ISR, Default_IRQ_handler
  .set EE_READY_ISR, Default_IRQ_handler
  .set SPM_ISR, Default_IRQ_handler
  .set PLL_ISR, Default_IRQ_handler
end;

end.
