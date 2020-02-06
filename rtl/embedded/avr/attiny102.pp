unit ATtiny102;

{$goto on}
interface

var
  PINA: byte absolute $00;  // Input Pins, Port A
  DDRA: byte absolute $01;  // Data Direction Register, Port A
  PORTA: byte absolute $02;  // Port A Data register
  PUEA: byte absolute $03;  // Pull-up Enable Control Register for PORTA
  PINB: byte absolute $04;  // Input Pins, Port B
  DDRB: byte absolute $05;  // Data Direction Register, Port B
  PORTB: byte absolute $06;  // Port B Data register
  PUEB: byte absolute $07;  // Pull-up Enable Control Register for PORTB
  UDR: byte absolute $08;  // USART I/O Data Register
  UBRR: word absolute $09;  // USART Baud Rate Register  Bytes
  UBRRL: byte absolute $09;  // USART Baud Rate Register  Bytes
  UBRRH: byte absolute $0A;  // USART Baud Rate Register  Bytes;
  UCSRD: byte absolute $0B;  // USART Control and Status Register D
  UCSRC: byte absolute $0C;  // USART Control and Status Register C
  UCSRB: byte absolute $0D;  // USART Control and Status Register B
  UCSRA: byte absolute $0E;  // USART Control and Status Register A
  PCMSK0: byte absolute $0F;  // Pin Change Mask Register 0
  PCMSK1: byte absolute $10;  // Pin Change Mask Register 1
  PCIFR: byte absolute $11;  // Pin Change Interrupt Flag Register
  PCICR: byte absolute $12;  // Pin Change Interrupt Control Register
  EIMSK: byte absolute $13;  // External Interrupt Mask register
  EIFR: byte absolute $14;  // External Interrupt Flag register
  EICRA: byte absolute $15;  // External Interrupt Control Register A
  PORTCR: byte absolute $16;  // Port Control Register
  DIDR0: byte absolute $17;
  ADCL: byte absolute $19;  // ADC Data Register Low
  ADCH: byte absolute $1A;  // ADC Data Register High
  ADMUX: byte absolute $1B;  // The ADC multiplexer Selection Register
  ADCSRB: byte absolute $1C;  // The ADC Control and Status register B
  ADCSRA: byte absolute $1D;  // The ADC Control and Status register A
  ACSRB: byte absolute $1E;  // Analog Comparator Control And Status Register B
  ACSRA: byte absolute $1F;  // Analog Comparator Control And Status Register A
  ICR0: word absolute $22;  // Input Capture Register  Bytes
  ICR0L: byte absolute $22;  // Input Capture Register  Bytes
  ICR0H: byte absolute $23;  // Input Capture Register  Bytes;
  OCR0B: word absolute $24;  // Timer/Counter0 Output Compare Register B 
  OCR0BL: byte absolute $24;  // Timer/Counter0 Output Compare Register B 
  OCR0BH: byte absolute $25;  // Timer/Counter0 Output Compare Register B ;
  OCR0A: word absolute $26;  // Timer/Counter 0 Output Compare Register A 
  OCR0AL: byte absolute $26;  // Timer/Counter 0 Output Compare Register A 
  OCR0AH: byte absolute $27;  // Timer/Counter 0 Output Compare Register A ;
  TCNT0: word absolute $28;  // Timer/Counter0 
  TCNT0L: byte absolute $28;  // Timer/Counter0 
  TCNT0H: byte absolute $29;  // Timer/Counter0 ;
  TIFR0: byte absolute $2A;  // Overflow Interrupt Enable
  TIMSK0: byte absolute $2B;  // Timer Interrupt Mask Register 0
  TCCR0C: byte absolute $2C;  // Timer/Counter 0 Control Register C
  TCCR0B: byte absolute $2D;  // Timer/Counter 0 Control Register B
  TCCR0A: byte absolute $2E;  // Timer/Counter 0 Control Register A
  GTCCR: byte absolute $2F;  // General Timer/Counter Control Register
  WDTCSR: byte absolute $31;  // Watchdog Timer Control and Status Register
  NVMCSR: byte absolute $32;  // Non-Volatile Memory Control and Status Register
  NVMCMD: byte absolute $33;  // Non-Volatile Memory Command
  VLMCSR: byte absolute $34;  // Vcc Level Monitoring Control and Status Register
  PRR: byte absolute $35;  // Power Reduction Register
  CLKPSR: byte absolute $36;  // Clock Prescale Register
  CLKMSR: byte absolute $37;  // Clock Main Settings Register
  OSCCAL: byte absolute $39;  // Oscillator Calibration Value
  SMCR: byte absolute $3A;  // Sleep Mode Control Register
  RSTFLR: byte absolute $3B;  // Reset Flag Register
  CCP: byte absolute $3C;  // Configuration Change Protection
  SP: word absolute $3D;  // Stack Pointer 
  SPL: byte absolute $3D;  // Stack Pointer 
  SPH: byte absolute $3E;  // Stack Pointer ;
  SREG: byte absolute $3F;  // Status Register

const
  // Port A Data register
  PA0 = $00;  
  PA1 = $01;  
  PA2 = $02;  
  // Port B Data register
  PB1 = $01;
  PB2 = $02;  
  PB3 = $03;  
  // USART Control and Status Register D
  SFDE = $05;  
  RXS = $06;  
  RXSIE = $07;  
  // USART Control and Status Register C
  UCPOL = $00;  
  UCSZ0 = $01;  // Character Size
  UCSZ1 = $02;  // Character Size
  USBS = $03;  
  UPM0 = $04;  // Parity Mode Bits
  UPM1 = $05;  // Parity Mode Bits
  UMSEL0 = $06;  // USART Mode Select
  UMSEL1 = $07;  // USART Mode Select
  // USART Control and Status Register B
  TXB8 = $00;  
  RXB8 = $01;  
  UCSZ2 = $02;  
  TXEN = $03;  
  RXEN = $04;  
  UDRIE = $05;  
  TXCIE = $06;  
  RXCIE = $07;  
  // USART Control and Status Register A
  MPCM = $00;  
  U2X = $01;  
  UPE = $02;  
  DOR = $03;  
  FE = $04;  
  UDRE = $05;  
  TXC = $06;  
  RXC = $07;  
  // Pin Change Mask Register 0
  PCINT0 = $00;  
  PCINT1 = $01;  
  PCINT2 = $02;  
  PCINT3 = $03;  
  PCINT4 = $04;  
  PCINT5 = $05;  
  PCINT6 = $06;  
  PCINT7 = $07;  
  // Pin Change Mask Register 1
  PCINT8 = $00;  
  PCINT9 = $01;  
  PCINT10 = $02;  
  PCINT11 = $03;  
  // Pin Change Interrupt Flag Register
  PCIF0 = $00;  
  PCIF1 = $01;  
  // Pin Change Interrupt Control Register
  PCIE0 = $00;  
  PCIE1 = $01;  
  // External Interrupt Mask register
  INT0 = $00;  
  // External Interrupt Flag register
  INTF0 = $00;  
  // External Interrupt Control Register A
  ISC00 = $00;  
  ISC01 = $01;  
  // Port Control Register
  BBMA = $00;  
  BBMB = $01;  
  ADC0D = $00;  
  AIN0D = $00;  
  ADC1D = $01;  
  AIN1D = $01;  
  ADC2D = $02;  
  ADC3D = $03;  
  ADC4D = $04;  
  ADC5D = $05;  
  ADC6D = $06;  
  ADC7D = $07;  
  // The ADC multiplexer Selection Register
  MUX0 = $00;  // Analog Channel Selection Bits
  MUX1 = $01;  // Analog Channel Selection Bits
  MUX2 = $02;  // Analog Channel Selection Bits
  REFS0 = $06;  // Analog Reference voltage Selection Bits
  REFS1 = $07;  // Analog Reference voltage Selection Bits
  // The ADC Control and Status register B
  ADTS0 = $00;  // ADC Auto Trigger Source bits
  ADTS1 = $01;  // ADC Auto Trigger Source bits
  ADTS2 = $02;  // ADC Auto Trigger Source bits
  ADLAR = $07;  
  // The ADC Control and Status register A
  ADPS0 = $00;  // ADC  Prescaler Select Bits
  ADPS1 = $01;  // ADC  Prescaler Select Bits
  ADPS2 = $02;  // ADC  Prescaler Select Bits
  ADIE = $03;  
  ADIF = $04;  
  ADATE = $05;  
  ADSC = $06;  
  ADEN = $07;  
  // Analog Comparator Control And Status Register B
  ACPMUX = $00;  
  ACOE = $01;  
  // Analog Comparator Control And Status Register A
  ACIS0 = $00;  // Analog Comparator Interrupt Mode Select bits
  ACIS1 = $01;  // Analog Comparator Interrupt Mode Select bits
  ACIC = $02;  
  ACIE = $03;  
  ACI = $04;  
  ACO = $05;  
  ACBG = $06;  
  ACD = $07;  
  // Overflow Interrupt Enable
  TOV0 = $00;  
  OCF0A = $01;  
  OCF0B = $02;  
  ICF0 = $05;  
  // Timer Interrupt Mask Register 0
  TOIE0 = $00;  
  OCIE0A = $01;  
  OCIE0B = $02;  
  ICIE0 = $05;  
  // Timer/Counter 0 Control Register C
  FOC0B = $06;  
  FOC0A = $07;  
  // Timer/Counter 0 Control Register B
  CS00 = $00;  // Clock Select
  CS01 = $01;  // Clock Select
  CS02 = $02;  // Clock Select
  ICES0 = $06;  
  ICNC0 = $07;  
  // Timer/Counter 0 Control Register A
  WGM00 = $00;  // Waveform Generation Mode
  WGM01 = $01;  // Waveform Generation Mode
  COM0B0 = $04;  // Compare Output Mode for Channel B bits
  COM0B1 = $05;  // Compare Output Mode for Channel B bits
  COM0A0 = $06;  // Compare Output Mode for Channel A bits
  COM0A1 = $07;  // Compare Output Mode for Channel A bits
  // General Timer/Counter Control Register
  PSR = $00;  
  REMAP = $01;  
  TSM = $07;  
  // Watchdog Timer Control and Status Register
  WDE = $03;  
  WDP0 = $00;  // Watchdog Timer Prescaler Bits
  WDP1 = $01;  // Watchdog Timer Prescaler Bits
  WDP2 = $02;  // Watchdog Timer Prescaler Bits
  WDP3 = $05;  // Watchdog Timer Prescaler Bits
  WDIE = $06;  
  WDIF = $07;  
  // Non-Volatile Memory Control and Status Register
  NVMBSY = $07;  
  // Vcc Level Monitoring Control and Status Register
  VLM0 = $00;  // Trigger Level of Voltage Level Monitor bits
  VLM1 = $01;  // Trigger Level of Voltage Level Monitor bits
  VLM2 = $02;  // Trigger Level of Voltage Level Monitor bits
  VLMIE = $06;  
  VLMF = $07;  
  // Power Reduction Register
  PRTIM0 = $00;  
  PRADC = $01;  
  PRUSART = $02;  
  // Clock Prescale Register
  CLKPS0 = $00;  // Clock Prescaler Select Bits
  CLKPS1 = $01;  // Clock Prescaler Select Bits
  CLKPS2 = $02;  // Clock Prescaler Select Bits
  CLKPS3 = $03;  // Clock Prescaler Select Bits
  // Clock Main Settings Register
  CLKMS0 = $00;  // Clock Main Select Bits
  CLKMS1 = $01;  // Clock Main Select Bits
  // Sleep Mode Control Register
  SE = $00;  
  SM0 = $01;  // Sleep Mode Select Bits
  SM1 = $02;  // Sleep Mode Select Bits
  SM2 = $03;  // Sleep Mode Select Bits
  // Reset Flag Register
  PORF = $00;  
  EXTRF = $01;  
  WDRF = $03;  
  // Configuration Change Protection
  CCP0 = $00;  // CCP signature
  CCP1 = $01;  // CCP signature
  CCP2 = $02;  // CCP signature
  CCP3 = $03;  // CCP signature
  CCP4 = $04;  // CCP signature
  CCP5 = $05;  // CCP signature
  CCP6 = $06;  // CCP signature
  CCP7 = $07;  // CCP signature
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

procedure INT0_ISR; external name 'INT0_ISR'; // Interrupt 1 External Interrupt Request 0
procedure PCINT0_ISR; external name 'PCINT0_ISR'; // Interrupt 2 Pin Change Interrupt Request 0
procedure PCINT1_ISR; external name 'PCINT1_ISR'; // Interrupt 3 Pin Change Interrupt Request 1
procedure TIM0_CAPT_ISR; external name 'TIM0_CAPT_ISR'; // Interrupt 4 Timer/Counter0 Input Capture
procedure TIM0_OVF_ISR; external name 'TIM0_OVF_ISR'; // Interrupt 5 Timer/Counter0 Overflow
procedure TIM0_COMPA_ISR; external name 'TIM0_COMPA_ISR'; // Interrupt 6 Timer/Counter Compare Match A
procedure TIM0_COMPB_ISR; external name 'TIM0_COMPB_ISR'; // Interrupt 7 Timer/Counter Compare Match B
procedure ANA_COMP_ISR; external name 'ANA_COMP_ISR'; // Interrupt 8 Analog Comparator
procedure WDT_ISR; external name 'WDT_ISR'; // Interrupt 9 Watchdog Time-out
procedure VLM_ISR; external name 'VLM_ISR'; // Interrupt 10 Vcc Voltage Level Monitor
procedure ADC_ISR; external name 'ADC_ISR'; // Interrupt 11 ADC Conversion complete
procedure USART_RXS_ISR; external name 'USART_RXS_ISR'; // Interrupt 12 USART RX Start
procedure USART_RXC_ISR; external name 'USART_RXC_ISR'; // Interrupt 13 USART RX Complete
procedure USART_DRE_ISR; external name 'USART_DRE_ISR'; // Interrupt 14 USART Data register empty
procedure USART_TXC_ISR; external name 'USART_TXC_ISR'; // Interrupt 15 USART Tx Complete

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  rjmp _start
  rjmp INT0_ISR
  rjmp PCINT0_ISR
  rjmp PCINT1_ISR
  rjmp TIM0_CAPT_ISR
  rjmp TIM0_OVF_ISR
  rjmp TIM0_COMPA_ISR
  rjmp TIM0_COMPB_ISR
  rjmp ANA_COMP_ISR
  rjmp WDT_ISR
  rjmp VLM_ISR
  rjmp ADC_ISR
  rjmp USART_RXS_ISR
  rjmp USART_RXC_ISR
  rjmp USART_DRE_ISR
  rjmp USART_TXC_ISR

  {$i start.inc}

  .weak INT0_ISR
  .weak PCINT0_ISR
  .weak PCINT1_ISR
  .weak TIM0_CAPT_ISR
  .weak TIM0_OVF_ISR
  .weak TIM0_COMPA_ISR
  .weak TIM0_COMPB_ISR
  .weak ANA_COMP_ISR
  .weak WDT_ISR
  .weak VLM_ISR
  .weak ADC_ISR
  .weak USART_RXS_ISR
  .weak USART_RXC_ISR
  .weak USART_DRE_ISR
  .weak USART_TXC_ISR

  .set INT0_ISR, Default_IRQ_handler
  .set PCINT0_ISR, Default_IRQ_handler
  .set PCINT1_ISR, Default_IRQ_handler
  .set TIM0_CAPT_ISR, Default_IRQ_handler
  .set TIM0_OVF_ISR, Default_IRQ_handler
  .set TIM0_COMPA_ISR, Default_IRQ_handler
  .set TIM0_COMPB_ISR, Default_IRQ_handler
  .set ANA_COMP_ISR, Default_IRQ_handler
  .set WDT_ISR, Default_IRQ_handler
  .set VLM_ISR, Default_IRQ_handler
  .set ADC_ISR, Default_IRQ_handler
  .set USART_RXS_ISR, Default_IRQ_handler
  .set USART_RXC_ISR, Default_IRQ_handler
  .set USART_DRE_ISR, Default_IRQ_handler
  .set USART_TXC_ISR, Default_IRQ_handler
end;

end.
