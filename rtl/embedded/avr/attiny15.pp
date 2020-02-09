unit ATtiny15;

{$goto on}
interface

var
  ADC: word absolute $04;  // ADC Data Register  Bytes
  ADCL: byte absolute $04;  // ADC Data Register  Bytes
  ADCH: byte absolute $05;  // ADC Data Register  Bytes;
  ADCSR: byte absolute $06;  // The ADC Control and Status register
  ADMUX: byte absolute $07;  // The ADC multiplexer Selection Register
  ACSR: byte absolute $08;  // Analog Comparator Control And Status Register
  PINB: byte absolute $16;  // Input Pins, Port B
  DDRB: byte absolute $17;  // Data Direction Register, Port B
  PORTB: byte absolute $18;  // Data Register, Port B
  EECR: byte absolute $1C;  // EEPROM Control Register
  EEDR: byte absolute $1D;  // EEPROM Data Register
  EEAR: byte absolute $1E;  // EEPROM Read/Write Access
  WDTCR: byte absolute $21;  // Watchdog Timer Control Register
  SFIOR: byte absolute $2C;  // Special Function IO Register
  OCR1B: byte absolute $2D;  // Output Compare Register
  OCR1A: byte absolute $2E;  // Output Compare Register
  TCNT1: byte absolute $2F;  // Timer/Counter Register
  TCCR1: byte absolute $30;  // Timer/Counter Control Register
  OSCCAL: byte absolute $31;  // Status Register
  TCNT0: byte absolute $32;  // Timer Counter 0
  TCCR0: byte absolute $33;  // Timer/Counter0 Control Register
  MCUSR: byte absolute $34;  // MCU Status register
  MCUCR: byte absolute $35;  // MCU Control Register
  TIFR: byte absolute $38;  // Timer/Counter Interrupt Flag Register
  TIMSK: byte absolute $39;  // Timer/Counter Interrupt Mask Register
  GIFR: byte absolute $3A;  // General Interrupt Flag register
  GIMSK: byte absolute $3B;  // General Interrupt Mask Register
  SREG: byte absolute $3F;  // Status Register

const
  // The ADC Control and Status register
  ADPS0 = $00;  // ADC  Prescaler Select Bits
  ADPS1 = $01;  // ADC  Prescaler Select Bits
  ADPS2 = $02;  // ADC  Prescaler Select Bits
  ADIE = $03;  
  ADIF = $04;  
  ADFR = $05;  
  ADSC = $06;  
  ADEN = $07;  
  // The ADC multiplexer Selection Register
  MUX0 = $00;  // Analog Channel and Gain Selection Bits
  MUX1 = $01;  // Analog Channel and Gain Selection Bits
  MUX2 = $02;  // Analog Channel and Gain Selection Bits
  ADLAR = $05;  
  REFS0 = $06;  // Reference Selection Bits
  REFS1 = $07;  // Reference Selection Bits
  // Analog Comparator Control And Status Register
  ACIS0 = $00;  // Analog Comparator Interrupt Mode Select bits
  ACIS1 = $01;  // Analog Comparator Interrupt Mode Select bits
  ACIE = $03;  
  ACI = $04;  
  ACO = $05;  
  ACBG = $06;  
  ACD = $07;  
  // Data Register, Port B
  PB0 = $00;  
  PB1 = $01;  
  PB2 = $02;  
  PB3 = $03;  
  PB4 = $04;  
  // EEPROM Control Register
  EERE = $00;  
  EEWE = $01;  
  EEMWE = $02;  
  EERIE = $03;  
  // Watchdog Timer Control Register
  WDP0 = $00;  // Watch Dog Timer Prescaler bits
  WDP1 = $01;  // Watch Dog Timer Prescaler bits
  WDP2 = $02;  // Watch Dog Timer Prescaler bits
  WDE = $03;  
  WDTOE = $04;  
  // Special Function IO Register
  PSR0 = $00;  
  PSR1 = $01;  
  FOC1A = $02;  
  // Timer/Counter Control Register
  CS10 = $00;  // Clock Select Bits
  CS11 = $01;  // Clock Select Bits
  CS12 = $02;  // Clock Select Bits
  CS13 = $03;  // Clock Select Bits
  COM1A0 = $04;  // Compare Output Mode, Bits
  COM1A1 = $05;  // Compare Output Mode, Bits
  PWM1 = $06;  
  CTC1 = $07;  
  // Status Register
  OSCCAL0 = $00;  // Oscillator Calibration 
  OSCCAL1 = $01;  // Oscillator Calibration 
  OSCCAL2 = $02;  // Oscillator Calibration 
  OSCCAL3 = $03;  // Oscillator Calibration 
  OSCCAL4 = $04;  // Oscillator Calibration 
  OSCCAL5 = $05;  // Oscillator Calibration 
  OSCCAL6 = $06;  // Oscillator Calibration 
  OSCCAL7 = $07;  // Oscillator Calibration 
  // Timer/Counter0 Control Register
  CS00 = $00;  
  CS01 = $01;  
  CS02 = $02;  
  // MCU Status register
  PORF = $00;  
  EXTRF = $01;  
  BORF = $02;  
  WDRF = $03;  
  // MCU Control Register
  ISC00 = $00;  // Interrupt Sense Control 0 bits
  ISC01 = $01;  // Interrupt Sense Control 0 bits
  SM0 = $03;  // Sleep Mode Select Bits
  SM1 = $04;  // Sleep Mode Select Bits
  SE = $05;  
  PUD = $06;  
  // Timer/Counter Interrupt Flag Register
  TOV0 = $01;  
  TOV1 = $02;  
  OCF1A = $06;  
  // Timer/Counter Interrupt Mask Register
  TOIE0 = $01;  
  TOIE1 = $02;  
  OCIE1A = $06;  
  // General Interrupt Flag register
  PCIF = $05;  
  INTF0 = $06;  
  // General Interrupt Mask Register
  PCIE = $05;  
  INT0 = $06;  
  // Status Register
  C = $00;  
  Z = $01;  
  N = $02;  
  V = $03;  
  S = $04;  
  H = $05;  
  T = $06;  
  I = $07;  


implementation
{$define RELBRANCHES}
{$i avrcommon.inc}

procedure INT0_ISR; external name 'INT0_ISR'; // Interrupt 1 External Interrupt 0
procedure IO_PINS_ISR; external name 'IO_PINS_ISR'; // Interrupt 2 External Interrupt Request 0
procedure TIMER1_COMP_ISR; external name 'TIMER1_COMP_ISR'; // Interrupt 3 Timer/Counter1 Compare Match
procedure TIMER1_OVF_ISR; external name 'TIMER1_OVF_ISR'; // Interrupt 4 Timer/Counter1 Overflow
procedure TIMER0_OVF_ISR; external name 'TIMER0_OVF_ISR'; // Interrupt 5 Timer/Counter0 Overflow
procedure EE_RDY_ISR; external name 'EE_RDY_ISR'; // Interrupt 6 EEPROM Ready
procedure ANA_COMP_ISR; external name 'ANA_COMP_ISR'; // Interrupt 7 Analog Comparator
procedure ADC_ISR; external name 'ADC_ISR'; // Interrupt 8 ADC Conversion Ready

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  rjmp _start
  rjmp INT0_ISR
  rjmp IO_PINS_ISR
  rjmp TIMER1_COMP_ISR
  rjmp TIMER1_OVF_ISR
  rjmp TIMER0_OVF_ISR
  rjmp EE_RDY_ISR
  rjmp ANA_COMP_ISR
  rjmp ADC_ISR

  {$i start_noram.inc}

  .weak INT0_ISR
  .weak IO_PINS_ISR
  .weak TIMER1_COMP_ISR
  .weak TIMER1_OVF_ISR
  .weak TIMER0_OVF_ISR
  .weak EE_RDY_ISR
  .weak ANA_COMP_ISR
  .weak ADC_ISR

  .set INT0_ISR, Default_IRQ_handler
  .set IO_PINS_ISR, Default_IRQ_handler
  .set TIMER1_COMP_ISR, Default_IRQ_handler
  .set TIMER1_OVF_ISR, Default_IRQ_handler
  .set TIMER0_OVF_ISR, Default_IRQ_handler
  .set EE_RDY_ISR, Default_IRQ_handler
  .set ANA_COMP_ISR, Default_IRQ_handler
  .set ADC_ISR, Default_IRQ_handler
end;

end.
