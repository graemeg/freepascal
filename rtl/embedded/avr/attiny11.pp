unit ATtiny11;

{$goto on}
interface

var
  ACSR: byte absolute $08;  // Analog Comparator Control And Status Register
  PINB: byte absolute $16;  // Input Pins, Port B
  DDRB: byte absolute $17;  // Data Direction Register, Port B
  PORTB: byte absolute $18;  // Data Register, Port B
  WDTCR: byte absolute $21;  // Watchdog Timer Control Register
  TCNT0: byte absolute $32;  // Timer Counter 0
  TCCR0: byte absolute $33;  // Timer/Counter0 Control Register
  MCUSR: byte absolute $34;  // MCU Status register
  MCUCR: byte absolute $35;  // MCU Control Register
  TIFR: byte absolute $38;  // Timer/Counter Interrupt Flag register
  TIMSK: byte absolute $39;  // Timer/Counter Interrupt Mask Register
  GIFR: byte absolute $3A;  // General Interrupt Flag register
  GIMSK: byte absolute $3B;  // General Interrupt Mask Register
  SREG: byte absolute $3F;  // Status Register

const
  // Analog Comparator Control And Status Register
  ACIS0 = $00;  // Analog Comparator Interrupt Mode Select bits
  ACIS1 = $01;  // Analog Comparator Interrupt Mode Select bits
  ACIE = $03;  
  ACI = $04;  
  ACO = $05;  
  ACD = $07;  
  // Data Register, Port B
  PB0 = $00;  
  PB1 = $01;  
  PB2 = $02;  
  PB3 = $03;  
  PB4 = $04;  
  // Watchdog Timer Control Register
  WDP0 = $00;  // Watch Dog Timer Prescaler bits
  WDP1 = $01;  // Watch Dog Timer Prescaler bits
  WDP2 = $02;  // Watch Dog Timer Prescaler bits
  WDE = $03;  
  WDTOE = $04;  
  // Timer/Counter0 Control Register
  CS00 = $00;  
  CS01 = $01;  
  CS02 = $02;  
  // MCU Status register
  PORF = $00;  
  EXTRF = $01;  
  // MCU Control Register
  ISC00 = $00;  // Interrupt Sense Control 0 bits
  ISC01 = $01;  // Interrupt Sense Control 0 bits
  SM = $04;  
  SE = $05;  
  // Timer/Counter Interrupt Flag register
  TOV0 = $01;  
  // Timer/Counter Interrupt Mask Register
  TOIE0 = $01;  
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
procedure TIMER0_OVF_ISR; external name 'TIMER0_OVF_ISR'; // Interrupt 3 Timer/Counter0 Overflow
procedure ANA_COMP_ISR; external name 'ANA_COMP_ISR'; // Interrupt 4 Analog Comparator

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  rjmp _start
  rjmp INT0_ISR
  rjmp IO_PINS_ISR
  rjmp TIMER0_OVF_ISR
  rjmp ANA_COMP_ISR

  {$i start_noram.inc}

  .weak INT0_ISR
  .weak IO_PINS_ISR
  .weak TIMER0_OVF_ISR
  .weak ANA_COMP_ISR

  .set INT0_ISR, Default_IRQ_handler
  .set IO_PINS_ISR, Default_IRQ_handler
  .set TIMER0_OVF_ISR, Default_IRQ_handler
  .set ANA_COMP_ISR, Default_IRQ_handler
end;

end.
