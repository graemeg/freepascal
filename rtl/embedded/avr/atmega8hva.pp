unit ATmega8HVA;

{$goto on}
interface

var
  PINA: byte absolute $20;  // Port A Input Pins
  DDRA: byte absolute $21;  // Port A Data Direction Register
  PORTA: byte absolute $22;  // Port A Data Register
  PINB: byte absolute $23;  // Input Pins, Port B
  DDRB: byte absolute $24;  // Data Direction Register, Port B
  PORTB: byte absolute $25;  // Data Register, Port B
  PINC: byte absolute $26;  // Port C Input Pins
  PORTC: byte absolute $28;  // Port C Data Register
  TIFR0: byte absolute $35;  // Timer/Counter Interrupt Flag register
  TIFR1: byte absolute $36;  // Timer/Counter Interrupt Flag register
  OSICSR: byte absolute $37;  // Oscillator Sampling Interface Control and Status Register
  EIFR: byte absolute $3C;  // External Interrupt Flag Register
  EIMSK: byte absolute $3D;  // External Interrupt Mask Register
  GPIOR0: byte absolute $3E;  // General Purpose IO Register 0
  EECR: byte absolute $3F;  // EEPROM Control Register
  EEDR: byte absolute $40;  // EEPROM Data Register
  EEAR: byte absolute $41;  // EEPROM Read/Write Access
  GTCCR: byte absolute $43;  // General Timer/Counter Control Register
  TCCR0A: byte absolute $44;  // Timer/Counter0 Control Register
  TCCR0B: byte absolute $45;  // Timer/Counter0 Control Register
  TCNT0: word absolute $46;  // Timer Counter 0  Bytes
  TCNT0L: byte absolute $46;  // Timer Counter 0  Bytes
  TCNT0H: byte absolute $47;  // Timer Counter 0  Bytes;
  OCR0A: byte absolute $48;  // Output compare Register A
  OCR0B: byte absolute $49;  // Output compare Register B
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
  PRR0: byte absolute $64;  // Power Reduction Register 0
  FOSCCAL: byte absolute $66;  // Fast Oscillator Calibration Value
  EICRA: byte absolute $69;  // External Interrupt Control Register
  TIMSK0: byte absolute $6E;  // Timer/Counter Interrupt Mask Register
  TIMSK1: byte absolute $6F;  // Timer/Counter Interrupt Mask Register
  VADC: word absolute $78;  // VADC Data Register  Bytes
  VADCL: byte absolute $78;  // VADC Data Register  Bytes
  VADCH: byte absolute $79;  // VADC Data Register  Bytes;
  VADCSR: byte absolute $7A;  // The VADC Control and Status register
  VADMUX: byte absolute $7C;  // The VADC multiplexer Selection Register
  DIDR0: byte absolute $7E;  // Digital Input Disable Register
  TCCR1A: byte absolute $80;  // Timer/Counter 1 Control Register A
  TCCR1B: byte absolute $81;  // Timer/Counter1 Control Register B
  TCNT1: word absolute $84;  // Timer Counter 1  Bytes
  TCNT1L: byte absolute $84;  // Timer Counter 1  Bytes
  TCNT1H: byte absolute $85;  // Timer Counter 1  Bytes;
  OCR1A: byte absolute $88;  // Output Compare Register 1A
  OCR1B: byte absolute $89;  // Output Compare Register B
  ROCR: byte absolute $C8;  // Regulator Operating Condition Register
  BGCCR: byte absolute $D0;  // Bandgap Calibration Register
  BGCRR: byte absolute $D1;  // Bandgap Calibration of Resistor Ladder
  CADAC0: byte absolute $E0;  // ADC Accumulate Current
  CADAC1: byte absolute $E1;  // ADC Accumulate Current
  CADAC2: byte absolute $E2;  // ADC Accumulate Current
  CADAC3: byte absolute $E3;  // ADC Accumulate Current
  CADCSRA: byte absolute $E4;  // CC-ADC Control and Status Register A
  CADCSRB: byte absolute $E5;  // CC-ADC Control and Status Register B
  CADRC: byte absolute $E6;  // CC-ADC Regular Current
  CADIC: word absolute $E8;  // CC-ADC Instantaneous Current
  CADICL: byte absolute $E8;  // CC-ADC Instantaneous Current
  CADICH: byte absolute $E9;  // CC-ADC Instantaneous Current;
  FCSR: byte absolute $F0;  // FET Control and Status Register
  BPIMSK: byte absolute $F2;  // Battery Protection Interrupt Mask Register
  BPIFR: byte absolute $F3;  // Battery Protection Interrupt Flag Register
  BPSCD: byte absolute $F5;  // Battery Protection Short-Circuit Detection Level Register
  BPDOCD: byte absolute $F6;  // Battery Protection Discharge-Over-current Detection Level Register
  BPCOCD: byte absolute $F7;  // Battery Protection Charge-Over-current Detection Level Register
  BPDHCD: byte absolute $F8;  // Battery Protection Discharge-High-current Detection Level Register
  BPCHCD: byte absolute $F9;  // Battery Protection Charge-High-current Detection Level Register
  BPSCTR: byte absolute $FA;  // Battery Protection Short-current Timing Register
  BPOCTR: byte absolute $FB;  // Battery Protection Over-current Timing Register
  BPHCTR: byte absolute $FC;  // Battery Protection Short-current Timing Register
  BPCR: byte absolute $FD;  // Battery Protection Control Register
  BPPLR: byte absolute $FE;  // Battery Protection Parameter Lock Register

const
  // Port A Data Register
  PA0 = $00;  
  PA1 = $01;  
  // Data Register, Port B
  PB0 = $00;  
  PB1 = $01;  
  PB2 = $02;  
  PB3 = $03;  
  // Port C Data Register
  PC0 = $00;  
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
  // Oscillator Sampling Interface Control and Status Register
  OSIEN = $00;  
  OSIST = $01;  
  OSISEL0 = $04;  
  // External Interrupt Flag Register
  INTF0 = $00;  // External Interrupt Flags
  INTF1 = $01;  // External Interrupt Flags
  INTF2 = $02;  // External Interrupt Flags
  // External Interrupt Mask Register
  INT0 = $00;  // External Interrupt Request 2 Enable
  INT1 = $01;  // External Interrupt Request 2 Enable
  INT2 = $02;  // External Interrupt Request 2 Enable
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
  // Timer/Counter0 Control Register
  WGM00 = $00;  
  ICS0 = $03;  
  ICES0 = $04;  
  ICNC0 = $05;  
  ICEN0 = $06;  
  TCW0 = $07;  
  // Timer/Counter0 Control Register
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
  PUD = $04;  
  CKOE = $05;  
  // Store Program Memory Control and Status Register
  SPMEN = $00;  
  PGERS = $01;  
  PGWRT = $02;  
  RFLB = $03;  
  CTPB = $04;  
  SIGRD = $05;  
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
  // Power Reduction Register 0
  PRVADC = $00;  
  PRTIM0 = $01;  
  PRTIM1 = $02;  
  PRSPI = $03;  
  PRVRM = $05;  
  // External Interrupt Control Register
  ISC00 = $00;  // External Interrupt Sense Control 0 Bits
  ISC01 = $01;  // External Interrupt Sense Control 0 Bits
  ISC10 = $02;  // External Interrupt Sense Control 1 Bits
  ISC11 = $03;  // External Interrupt Sense Control 1 Bits
  ISC20 = $04;  // External Interrupt Sense Control 2 Bits
  ISC21 = $05;  // External Interrupt Sense Control 2 Bits
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
  // The VADC Control and Status register
  VADCCIE = $00;  
  VADCCIF = $01;  
  VADSC = $02;  
  VADEN = $03;  
  // The VADC multiplexer Selection Register
  VADMUX0 = $00;  // Analog Channel and Gain Selection Bits
  VADMUX1 = $01;  // Analog Channel and Gain Selection Bits
  VADMUX2 = $02;  // Analog Channel and Gain Selection Bits
  VADMUX3 = $03;  // Analog Channel and Gain Selection Bits
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
  // Regulator Operating Condition Register
  ROCWIE = $00;  
  ROCWIF = $01;  
  ROCS = $07;  
  // Bandgap Calibration Register
  BGCC0 = $00;  // BG Calibration of PTAT Current Bits
  BGCC1 = $01;  // BG Calibration of PTAT Current Bits
  BGCC2 = $02;  // BG Calibration of PTAT Current Bits
  BGCC3 = $03;  // BG Calibration of PTAT Current Bits
  BGCC4 = $04;  // BG Calibration of PTAT Current Bits
  BGCC5 = $05;  // BG Calibration of PTAT Current Bits
  BGD = $07;  
  // Bandgap Calibration of Resistor Ladder
  BGCR0 = $00;  // Bandgap calibration bits
  BGCR1 = $01;  // Bandgap calibration bits
  BGCR2 = $02;  // Bandgap calibration bits
  BGCR3 = $03;  // Bandgap calibration bits
  BGCR4 = $04;  // Bandgap calibration bits
  BGCR5 = $05;  // Bandgap calibration bits
  BGCR6 = $06;  // Bandgap calibration bits
  BGCR7 = $07;  // Bandgap calibration bits
  // CC-ADC Control and Status Register A
  CADSE = $00;  
  CADSI0 = $01;  // The CADSI bits determine the current sampling interval for the Regular Current detection in Power-down mode. The actual settings remain to be determined.
  CADSI1 = $02;  // The CADSI bits determine the current sampling interval for the Regular Current detection in Power-down mode. The actual settings remain to be determined.
  CADAS0 = $03;  // CC_ADC Accumulate Current Select Bits
  CADAS1 = $04;  // CC_ADC Accumulate Current Select Bits
  CADUB = $05;  
  CADPOL = $06;  
  CADEN = $07;  
  // CC-ADC Control and Status Register B
  CADICIF = $00;  
  CADRCIF = $01;  
  CADACIF = $02;  
  CADICIE = $04;  
  CADRCIE = $05;  
  CADACIE = $06;  
  // FET Control and Status Register
  CFE = $00;  
  DFE = $01;  
  CPS = $02;  
  DUVRD = $03;  
  // Battery Protection Interrupt Mask Register
  CHCIE = $00;  
  DHCIE = $01;  
  COCIE = $02;  
  DOCIE = $03;  
  SCIE = $04;  
  // Battery Protection Interrupt Flag Register
  CHCIF = $00;  
  DHCIF = $01;  
  COCIF = $02;  
  DOCIF = $03;  
  SCIF = $04;  
  // Battery Protection Control Register
  CHCD = $00;  
  DHCD = $01;  
  COCD = $02;  
  DOCD = $03;  
  SCD = $04;  
  // Battery Protection Parameter Lock Register
  BPPL = $00;  
  BPPLE = $01;  


implementation
{$define RELBRANCHES}
{$i avrcommon.inc}

procedure BPINT_ISR; external name 'BPINT_ISR'; // Interrupt 1 Battery Protection Interrupt
procedure VREGMON_ISR; external name 'VREGMON_ISR'; // Interrupt 2 Voltage regulator monitor interrupt
procedure INT0_ISR; external name 'INT0_ISR'; // Interrupt 3 External Interrupt Request 0
procedure INT1_ISR; external name 'INT1_ISR'; // Interrupt 4 External Interrupt Request 1
procedure INT2_ISR; external name 'INT2_ISR'; // Interrupt 5 External Interrupt Request 2
procedure WDT_ISR; external name 'WDT_ISR'; // Interrupt 6 Watchdog Timeout Interrupt
procedure TIMER1_IC_ISR; external name 'TIMER1_IC_ISR'; // Interrupt 7 Timer 1 Input capture
procedure TIMER1_COMPA_ISR; external name 'TIMER1_COMPA_ISR'; // Interrupt 8 Timer 1 Compare Match A
procedure TIMER1_COMPB_ISR; external name 'TIMER1_COMPB_ISR'; // Interrupt 9 Timer 1 Compare Match B
procedure TIMER1_OVF_ISR; external name 'TIMER1_OVF_ISR'; // Interrupt 10 Timer 1 overflow
procedure TIMER0_IC_ISR; external name 'TIMER0_IC_ISR'; // Interrupt 11 Timer 0 Input Capture
procedure TIMER0_COMPA_ISR; external name 'TIMER0_COMPA_ISR'; // Interrupt 12 Timer 0 Comapre Match A
procedure TIMER0_COMPB_ISR; external name 'TIMER0_COMPB_ISR'; // Interrupt 13 Timer 0 Compare Match B
procedure TIMER0_OVF_ISR; external name 'TIMER0_OVF_ISR'; // Interrupt 14 Timer 0 Overflow
procedure SPI_STC_ISR; external name 'SPI_STC_ISR'; // Interrupt 15 SPI Serial transfer complete
procedure VADC_ISR; external name 'VADC_ISR'; // Interrupt 16 Voltage ADC Conversion Complete
procedure CCADC_CONV_ISR; external name 'CCADC_CONV_ISR'; // Interrupt 17 Coulomb Counter ADC Conversion Complete
procedure CCADC_REG_CUR_ISR; external name 'CCADC_REG_CUR_ISR'; // Interrupt 18 Coloumb Counter ADC Regular Current
procedure CCADC_ACC_ISR; external name 'CCADC_ACC_ISR'; // Interrupt 19 Coloumb Counter ADC Accumulator
procedure EE_READY_ISR; external name 'EE_READY_ISR'; // Interrupt 20 EEPROM Ready

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  rjmp _start
  rjmp BPINT_ISR
  rjmp VREGMON_ISR
  rjmp INT0_ISR
  rjmp INT1_ISR
  rjmp INT2_ISR
  rjmp WDT_ISR
  rjmp TIMER1_IC_ISR
  rjmp TIMER1_COMPA_ISR
  rjmp TIMER1_COMPB_ISR
  rjmp TIMER1_OVF_ISR
  rjmp TIMER0_IC_ISR
  rjmp TIMER0_COMPA_ISR
  rjmp TIMER0_COMPB_ISR
  rjmp TIMER0_OVF_ISR
  rjmp SPI_STC_ISR
  rjmp VADC_ISR
  rjmp CCADC_CONV_ISR
  rjmp CCADC_REG_CUR_ISR
  rjmp CCADC_ACC_ISR
  rjmp EE_READY_ISR

  {$i start.inc}

  .weak BPINT_ISR
  .weak VREGMON_ISR
  .weak INT0_ISR
  .weak INT1_ISR
  .weak INT2_ISR
  .weak WDT_ISR
  .weak TIMER1_IC_ISR
  .weak TIMER1_COMPA_ISR
  .weak TIMER1_COMPB_ISR
  .weak TIMER1_OVF_ISR
  .weak TIMER0_IC_ISR
  .weak TIMER0_COMPA_ISR
  .weak TIMER0_COMPB_ISR
  .weak TIMER0_OVF_ISR
  .weak SPI_STC_ISR
  .weak VADC_ISR
  .weak CCADC_CONV_ISR
  .weak CCADC_REG_CUR_ISR
  .weak CCADC_ACC_ISR
  .weak EE_READY_ISR

  .set BPINT_ISR, Default_IRQ_handler
  .set VREGMON_ISR, Default_IRQ_handler
  .set INT0_ISR, Default_IRQ_handler
  .set INT1_ISR, Default_IRQ_handler
  .set INT2_ISR, Default_IRQ_handler
  .set WDT_ISR, Default_IRQ_handler
  .set TIMER1_IC_ISR, Default_IRQ_handler
  .set TIMER1_COMPA_ISR, Default_IRQ_handler
  .set TIMER1_COMPB_ISR, Default_IRQ_handler
  .set TIMER1_OVF_ISR, Default_IRQ_handler
  .set TIMER0_IC_ISR, Default_IRQ_handler
  .set TIMER0_COMPA_ISR, Default_IRQ_handler
  .set TIMER0_COMPB_ISR, Default_IRQ_handler
  .set TIMER0_OVF_ISR, Default_IRQ_handler
  .set SPI_STC_ISR, Default_IRQ_handler
  .set VADC_ISR, Default_IRQ_handler
  .set CCADC_CONV_ISR, Default_IRQ_handler
  .set CCADC_REG_CUR_ISR, Default_IRQ_handler
  .set CCADC_ACC_ISR, Default_IRQ_handler
  .set EE_READY_ISR, Default_IRQ_handler
end;

end.
