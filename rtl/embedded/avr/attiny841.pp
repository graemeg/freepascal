unit ATtiny841;

{$goto on}
interface

var
  ADCSRB: byte absolute $24;  // ADC Control and Status Register B
  ADCSRA: byte absolute $25;  // The ADC Control and Status register
  ADC: word absolute $26;  // ADC Data Register  Bytes
  ADCL: byte absolute $26;  // ADC Data Register  Bytes
  ADCH: byte absolute $27;  // ADC Data Register  Bytes;
  ADMUXB: byte absolute $28;  // The ADC multiplexer Selection Register B
  ADMUXA: byte absolute $29;  // The ADC multiplexer Selection Register A
  ACSR0A: byte absolute $2A;  // Analog Comparator 0 Control And Status Register A
  ACSR0B: byte absolute $2B;  // Analog Comparator 0 Control And Status Register B
  ACSR1A: byte absolute $2C;  // Analog Comparator 1 Control And Status Register A
  ACSR1B: byte absolute $2D;  // Analog Comparator 1 Control And Status Register B
  TIFR1: byte absolute $2E;  // Timer/Counter Interrupt Flag register
  TIMSK1: byte absolute $2F;  // Timer/Counter1 Interrupt Mask Register
  TIFR2: byte absolute $30;  // Timer/Counter Interrupt Flag register
  TIMSK2: byte absolute $31;  // Timer/Counter2 Interrupt Mask Register
  PCMSK0: byte absolute $32;  // Pin Change Enable Mask 0
  GPIOR0: byte absolute $33;  // General Purpose I/O Register 0
  GPIOR1: byte absolute $34;  // General Purpose I/O Register 1
  GPIOR2: byte absolute $35;  // General Purpose I/O Register 2
  PINB: byte absolute $36;  // Port B Data register
  DDRB: byte absolute $37;  // Data Direction Register, Port B
  PORTB: byte absolute $38;  // Input Pins, Port B
  PINA: byte absolute $39;  // Port A Input Pins
  DDRA: byte absolute $3A;  // Data Direction Register, Port A
  PORTA: byte absolute $3B;  // Port A Data Register
  EECR: byte absolute $3C;  // EEPROM Control Register
  EEDR: byte absolute $3D;  // EEPROM Data Register
  EEAR: word absolute $3E;  // EEPROM Address Register  Bytes
  EEARL: byte absolute $3E;  // EEPROM Address Register  Bytes
  EEARH: byte absolute $3F;  // EEPROM Address Register  Bytes;
  PCMSK1: byte absolute $40;  // Pin Change Enable Mask 1
  WDTCSR: byte absolute $41;  // Watchdog Timer Control and Status Register
  TCCR1C: byte absolute $42;  // Timer/Counter1 Control Register C
  GTCCR: byte absolute $43;  // General Timer/Counter Control Register
  ICR1: word absolute $44;  // Timer/Counter1 Input Capture Register  Bytes
  ICR1L: byte absolute $44;  // Timer/Counter1 Input Capture Register  Bytes
  ICR1H: byte absolute $45;  // Timer/Counter1 Input Capture Register  Bytes;
  OCR1B: word absolute $48;  // Timer/Counter1 Output Compare Register B  Bytes
  OCR1BL: byte absolute $48;  // Timer/Counter1 Output Compare Register B  Bytes
  OCR1BH: byte absolute $49;  // Timer/Counter1 Output Compare Register B  Bytes;
  OCR1A: word absolute $4A;  // Timer/Counter1 Output Compare Register A  Bytes
  OCR1AL: byte absolute $4A;  // Timer/Counter1 Output Compare Register A  Bytes
  OCR1AH: byte absolute $4B;  // Timer/Counter1 Output Compare Register A  Bytes;
  TCNT1: word absolute $4C;  // Timer/Counter1  Bytes
  TCNT1L: byte absolute $4C;  // Timer/Counter1  Bytes
  TCNT1H: byte absolute $4D;  // Timer/Counter1  Bytes;
  TCCR1B: byte absolute $4E;  // Timer/Counter1 Control Register B
  TCCR1A: byte absolute $4F;  // Timer/Counter1 Control Register A
  TCCR0A: byte absolute $50;  // Timer/Counter  Control Register A
  TCNT0: byte absolute $52;  // Timer/Counter0
  TCCR0B: byte absolute $53;  // Timer/Counter Control Register B
  MCUSR: byte absolute $54;  // MCU Status Register
  MCUCR: byte absolute $55;  // MCU Control Register
  OCR0A: byte absolute $56;  // Timer/Counter0 Output Compare Register A
  SPMCSR: byte absolute $57;  // Store Program Memory Control and Status Register
  TIFR0: byte absolute $58;  // Timer/Counter0 Interrupt Flag Register
  TIMSK0: byte absolute $59;  // Timer/Counter Interrupt Mask Register
  GIFR: byte absolute $5A;  // General Interrupt Flag register
  GIMSK: byte absolute $5B;  // General Interrupt Mask Register
  OCR0B: byte absolute $5C;  // Timer/Counter0 Output Compare Register B
  SP: word absolute $5D;  // Stack Pointer 
  SPL: byte absolute $5D;  // Stack Pointer 
  SPH: byte absolute $5E;  // Stack Pointer ;
  SREG: byte absolute $5F;  // Status Register
  DIDR0: byte absolute $60;  // Digital Input Disable Register 0
  DIDR1: byte absolute $61;  // Digital Input Disable Register 1
  PUEB: byte absolute $62;  // Pull-up Enable Control Register
  PUEA: byte absolute $63;  // Pull-up Enable Control Register
  PORTCR: byte absolute $64;  // Port Control Register
  REMAP: byte absolute $65;  // Remap Port Pins
  TOCPMCOE: byte absolute $66;  // Timer Output Compare Pin Mux Channel Output Enable
  TOCPMSA0: byte absolute $67;  // Timer Output Compare Pin Mux Selection 0
  TOCPMSA1: byte absolute $68;  // Timer Output Compare Pin Mux Selection 1
  PHDE: byte absolute $6A;  // Port High Drive Enable Register
  PRR: byte absolute $70;  // Power Reduction Register
  CCP: byte absolute $71;  // Configuration Change Protection
  CLKCR: byte absolute $72;  // Clock Control Register
  CLKPR: byte absolute $73;  // Clock Prescale Register
  OSCCAL0: byte absolute $74;  // Oscillator Calibration Register 8MHz
  OSCTCAL0A: byte absolute $75;  // Oscillator Temperature Calibration Register A
  OSCTCAL0B: byte absolute $76;  // Oscillator Temperature Calibration Register B
  OSCCAL1: byte absolute $77;  // Oscillator Calibration Register 32kHz
  UDR0: byte absolute $80;  // USART I/O Data Register
  UBRR0: word absolute $81;  // USART Baud Rate Register Bytes
  UBRR0L: byte absolute $81;  // USART Baud Rate Register Bytes
  UBRR0H: byte absolute $82;  // USART Baud Rate Register Bytes;
  UCSR0D: byte absolute $83;  // USART Control and Status Register D
  UCSR0C: byte absolute $84;  // USART Control and Status Register C
  UCSR0B: byte absolute $85;  // USART Control and Status Register B
  UCSR0A: byte absolute $86;  // USART Control and Status Register A
  UDR1: byte absolute $90;  // USART I/O Data Register
  UBRR1: word absolute $91;  // USART Baud Rate Register Bytes
  UBRR1L: byte absolute $91;  // USART Baud Rate Register Bytes
  UBRR1H: byte absolute $92;  // USART Baud Rate Register Bytes;
  UCSR1D: byte absolute $93;  // USART Control and Status Register D
  UCSR1C: byte absolute $94;  // USART Control and Status Register C
  UCSR1B: byte absolute $95;  // USART Control and Status Register B
  UCSR1A: byte absolute $96;  // USART Control and Status Register A
  TWSD: byte absolute $A0;  // TWI Slave Data Register
  TWSAM: byte absolute $A1;  // TWI Slave Address Mask Register
  TWSA: byte absolute $A2;  // TWI Slave Address Register
  TWSSRA: byte absolute $A3;  // TWI Slave Status Register A
  TWSCRB: byte absolute $A4;  // TWI Slave Control Register B
  TWSCRA: byte absolute $A5;  // TWI Slave Control Register A
  SPDR: byte absolute $B0;  // SPI Data Register
  SPSR: byte absolute $B1;  // SPI Status Register
  SPCR: byte absolute $B2;  // SPI Control Register
  ICR2: word absolute $C0;  // Timer/Counter2 Input Capture Register  Bytes
  ICR2L: byte absolute $C0;  // Timer/Counter2 Input Capture Register  Bytes
  ICR2H: byte absolute $C1;  // Timer/Counter2 Input Capture Register  Bytes;
  OCR2B: word absolute $C2;  // Timer/Counter2 Output Compare Register B  Bytes
  OCR2BL: byte absolute $C2;  // Timer/Counter2 Output Compare Register B  Bytes
  OCR2BH: byte absolute $C3;  // Timer/Counter2 Output Compare Register B  Bytes;
  OCR2A: word absolute $C4;  // Timer/Counter2 Output Compare Register A  Bytes
  OCR2AL: byte absolute $C4;  // Timer/Counter2 Output Compare Register A  Bytes
  OCR2AH: byte absolute $C5;  // Timer/Counter2 Output Compare Register A  Bytes;
  TCNT2: word absolute $C6;  // Timer/Counter2  Bytes
  TCNT2L: byte absolute $C6;  // Timer/Counter2  Bytes
  TCNT2H: byte absolute $C7;  // Timer/Counter2  Bytes;
  TCCR2C: byte absolute $C8;  // Timer/Counter2 Control Register C
  TCCR2B: byte absolute $C9;  // Timer/Counter2 Control Register B
  TCCR2A: byte absolute $CA;  // Timer/Counter2 Control Register A

const
  // ADC Control and Status Register B
  ADTS0 = $00;  // ADC Auto Trigger Sources
  ADTS1 = $01;  // ADC Auto Trigger Sources
  ADTS2 = $02;  // ADC Auto Trigger Sources
  ADLAR = $03;  
  // The ADC Control and Status register
  ADPS0 = $00;  // ADC Prescaler Select Bits
  ADPS1 = $01;  // ADC Prescaler Select Bits
  ADPS2 = $02;  // ADC Prescaler Select Bits
  ADIE = $03;  
  ADIF = $04;  
  ADATE = $05;  
  ADSC = $06;  
  ADEN = $07;  
  // The ADC multiplexer Selection Register B
  GSEL0 = $00;  // Gain Selection Bits
  GSEL1 = $01;  // Gain Selection Bits
  REFS0 = $05;  // Reference Selection Bits
  REFS1 = $06;  // Reference Selection Bits
  REFS2 = $07;  // Reference Selection Bits
  // The ADC multiplexer Selection Register A
  MUX0 = $00;  // Analog Channel and Gain Selection Bits
  MUX1 = $01;  // Analog Channel and Gain Selection Bits
  MUX2 = $02;  // Analog Channel and Gain Selection Bits
  MUX3 = $03;  // Analog Channel and Gain Selection Bits
  MUX4 = $04;  // Analog Channel and Gain Selection Bits
  MUX5 = $05;  // Analog Channel and Gain Selection Bits
  // Analog Comparator 0 Control And Status Register A
  ACIS00 = $00;  // Analog Comparator 0 Interrupt Mode Select bits
  ACIS01 = $01;  // Analog Comparator 0 Interrupt Mode Select bits
  ACIC0 = $02;  
  ACIE0 = $03;  
  ACI0 = $04;  
  ACO0 = $05;  
  ACPMUX2 = $06;  
  ACD0 = $07;  
  // Analog Comparator 0 Control And Status Register B
  ACPMUX0 = $00;  // Analog Comparator 0 Positive Input Multiplexer Bits 1:0
  ACPMUX1 = $01;  // Analog Comparator 0 Positive Input Multiplexer Bits 1:0
  ACNMUX0 = $02;  // Analog Comparator 0 Negative Input Multiplexer
  ACNMUX1 = $03;  // Analog Comparator 0 Negative Input Multiplexer
  ACOE0 = $04;  
  HLEV0 = $06;  
  HSEL0 = $07;  
  // Analog Comparator 1 Control And Status Register A
  ACIS10 = $00;  // Analog Comparator 1 Interrupt Mode Select bits
  ACIS11 = $01;  // Analog Comparator 1 Interrupt Mode Select bits
  ACIC1 = $02;  
  ACIE1 = $03;  
  ACI1 = $04;  
  ACO1 = $05;  
  ACBG1 = $06;  
  ACD1 = $07;  
  // Analog Comparator 1 Control And Status Register B
  ACME1 = $02;  
  ACOE1 = $04;  
  HLEV1 = $06;  
  HSEL1 = $07;  
  // Timer/Counter Interrupt Flag register
  TOV1 = $00;  
  OCF1A = $01;  
  OCF1B = $02;  
  ICF1 = $05;  
  // Timer/Counter1 Interrupt Mask Register
  TOIE1 = $00;  
  OCIE1A = $01;  
  OCIE1B = $02;  
  ICIE1 = $05;  
  // Timer/Counter Interrupt Flag register
  TOV2 = $00;  
  OCF2A = $01;  
  OCF2B = $02;  
  ICF2 = $05;  
  // Timer/Counter2 Interrupt Mask Register
  TOIE2 = $00;  
  OCIE2A = $01;  
  OCIE2B = $02;  
  ICIE2 = $05;  
  // Pin Change Enable Mask 0
  PCINT0 = $00;  
  PCINT1 = $01;  
  PCINT2 = $02;  
  PCINT3 = $03;  
  PCINT4 = $04;  
  PCINT5 = $05;  
  PCINT6 = $06;  
  PCINT7 = $07;  
  // Input Pins, Port B
  PB0 = $00;  
  PB1 = $01;  
  PB2 = $02;  
  PB3 = $03;  
  // Port A Data Register
  PA0 = $00;  
  PA1 = $01;  
  PA2 = $02;  
  PA3 = $03;  
  PA4 = $04;  
  PA5 = $05;  
  PA6 = $06;  
  PA7 = $07;  
  // EEPROM Control Register
  EERE = $00;  
  EEPE = $01;  
  EEMPE = $02;  
  EERIE = $03;  
  EEPM0 = $04;  // EEPROM Programming Mode Bits
  EEPM1 = $05;  // EEPROM Programming Mode Bits
  // Pin Change Enable Mask 1
  PCINT8 = $00;  
  PCINT9 = $01;  
  PCINT10 = $02;  
  PCINT11 = $03;  
  // Watchdog Timer Control and Status Register
  WDE = $03;  
  WDP0 = $00;  // Watchdog Timer Prescaler Bits
  WDP1 = $01;  // Watchdog Timer Prescaler Bits
  WDP2 = $02;  // Watchdog Timer Prescaler Bits
  WDP3 = $05;  // Watchdog Timer Prescaler Bits
  WDIE = $06;  
  WDIF = $07;  
  // Timer/Counter1 Control Register C
  FOC1B = $06;  
  FOC1A = $07;  
  // General Timer/Counter Control Register
  PSR = $00;  
  TSM = $07;  
  // Timer/Counter1 Control Register B
  CS10 = $00;  // Clock Select bits
  CS11 = $01;  // Clock Select bits
  CS12 = $02;  // Clock Select bits
  ICES1 = $06;  
  ICNC1 = $07;  
  // Timer/Counter1 Control Register A
  WGM10 = $00;  // Pulse Width Modulator Select Bits
  WGM11 = $01;  // Pulse Width Modulator Select Bits
  COM1B0 = $04;  // Compare Output Mode 1B, bits
  COM1B1 = $05;  // Compare Output Mode 1B, bits
  COM1A0 = $06;  // Compare Output Mode 1A, bits
  COM1A1 = $07;  // Compare Output Mode 1A, bits
  // Timer/Counter  Control Register A
  WGM00 = $00;  // Waveform Generation Mode bits
  WGM01 = $01;  // Waveform Generation Mode bits
  COM0B0 = $04;  // Compare Match Output B Mode bits
  COM0B1 = $05;  // Compare Match Output B Mode bits
  COM0A0 = $06;  // Compare Match Output A Mode bits
  COM0A1 = $07;  // Compare Match Output A Mode bits
  // Timer/Counter Control Register B
  CS00 = $00;  // Clock Select bits
  CS01 = $01;  // Clock Select bits
  CS02 = $02;  // Clock Select bits
  WGM02 = $03;  
  FOC0B = $06;  
  FOC0A = $07;  
  // MCU Status Register
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
  // Store Program Memory Control and Status Register
  SPMEN = $00;  
  PGERS = $01;  
  PGWRT = $02;  
  RFLB = $03;  
  CTPB = $04;  
  RSIG = $05;  
  // Timer/Counter0 Interrupt Flag Register
  TOV0 = $00;  
  OCF0A = $01;  
  OCF0B = $02;  
  // Timer/Counter Interrupt Mask Register
  TOIE0 = $00;  
  OCIE0A = $01;  
  OCIE0B = $02;  
  // General Interrupt Flag register
  PCIF0 = $04;  // Pin Change Interrupt Flags
  PCIF1 = $05;  // Pin Change Interrupt Flags
  INTF0 = $06;  
  // General Interrupt Mask Register
  PCIE0 = $04;  // Pin Change Interrupt Enables
  PCIE1 = $05;  // Pin Change Interrupt Enables
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
  // Digital Input Disable Register 0
  ADC0D = $00;  
  ADC1D = $01;  
  ADC2D = $02;  
  ADC3D = $03;  
  ADC4D = $04;  
  ADC5D = $05;  
  ADC6D = $06;  
  ADC7D = $07;  
  // Digital Input Disable Register 1
  ADC11D = $00;  
  ADC10D = $01;  
  ADC8D = $02;  
  ADC9D = $03;  
  // Port Control Register
  BBMA = $00;  
  BBMB = $01;  
  // Remap Port Pins
  U0MAP = $00;  
  SPIMAP = $01;  
  // Timer Output Compare Pin Mux Channel Output Enable
  TOCC0OE = $00;  
  TOCC1OE = $01;  
  TOCC2OE = $02;  
  TOCC3OE = $03;  
  TOCC4OE = $04;  
  TOCC5OE = $05;  
  TOCC6OE = $06;  
  TOCC7OE = $07;  
  // Timer Output Compare Pin Mux Selection 0
  TOCC0S0 = $00;  // Timer Output Compare Channel 0 Selection Bits
  TOCC0S1 = $01;  // Timer Output Compare Channel 0 Selection Bits
  TOCC1S0 = $02;  // Timer Output Compare Channel 1 Selection Bits
  TOCC1S1 = $03;  // Timer Output Compare Channel 1 Selection Bits
  TOCC2S0 = $04;  // Timer Output Compare Channel 2 Selection Bits
  TOCC2S1 = $05;  // Timer Output Compare Channel 2 Selection Bits
  TOCC3S0 = $06;  // Timer Output Compare Channel 3 Selection Bits
  TOCC3S1 = $07;  // Timer Output Compare Channel 3 Selection Bits
  // Timer Output Compare Pin Mux Selection 1
  TOCC4S0 = $00;  // Timer Output Compare Channel 4 Selection Bits
  TOCC4S1 = $01;  // Timer Output Compare Channel 4 Selection Bits
  TOCC5S0 = $02;  // Timer Output Compare Channel 5 Selection Bits
  TOCC5S1 = $03;  // Timer Output Compare Channel 5 Selection Bits
  TOCC6S0 = $04;  // Timer Output Compare Channel 6 Selection Bits
  TOCC6S1 = $05;  // Timer Output Compare Channel 6 Selection Bits
  TOCC7S0 = $06;  // Timer Output Compare Channel 7 Selection Bits
  TOCC7S1 = $07;  // Timer Output Compare Channel 7 Selection Bits
  // Port High Drive Enable Register
  PHDEA0 = $00;  // PortA High Drive Enable
  PHDEA1 = $01;  // PortA High Drive Enable
  // Power Reduction Register
  PRADC = $00;  
  PRTIM0 = $01;  
  PRTIM1 = $02;  
  PRTIM2 = $03;  
  PRSPI = $04;  
  PRUSART0 = $05;  
  PRUSART1 = $06;  
  PRTWI = $07;  
  // Clock Control Register
  CKSEL0 = $00;  // Clock Select Bits
  CKSEL1 = $01;  // Clock Select Bits
  CKSEL2 = $02;  // Clock Select Bits
  CKSEL3 = $03;  // Clock Select Bits
  SUT = $04;  
  CKOUTC = $05;  
  CSTR = $06;  
  OSCRDY = $07;  
  // Clock Prescale Register
  CLKPS0 = $00;  // Clock Prescaler Select Bits
  CLKPS1 = $01;  // Clock Prescaler Select Bits
  CLKPS2 = $02;  // Clock Prescaler Select Bits
  CLKPS3 = $03;  // Clock Prescaler Select Bits
  // USART Control and Status Register D
  SFDE0 = $05;  
  RXS0 = $06;  
  RXSIE0 = $07;  
  // USART Control and Status Register C
  UCPOL0 = $00;  
  UCSZ00 = $01;  // Character Size
  UCSZ01 = $02;  // Character Size
  USBS0 = $03;  
  UPM00 = $04;  // Parity Mode Bits
  UPM01 = $05;  // Parity Mode Bits
  UMSEL00 = $06;  // USART Mode Select
  UMSEL01 = $07;  // USART Mode Select
  // USART Control and Status Register B
  TXB80 = $00;  
  RXB80 = $01;  
  UCSZ02 = $02;  
  TXEN0 = $03;  
  RXEN0 = $04;  
  UDRIE0 = $05;  
  TXCIE0 = $06;  
  RXCIE0 = $07;  
  // USART Control and Status Register A
  MPCM0 = $00;  
  U2X0 = $01;  
  UPE0 = $02;  
  DOR0 = $03;  
  FE0 = $04;  
  UDRE0 = $05;  
  TXC0 = $06;  
  RXC0 = $07;  
  // USART Control and Status Register D
  SFDE1 = $05;  
  RXS1 = $06;  
  RXSIE1 = $07;  
  // USART Control and Status Register C
  UCPOL1 = $00;  
  UCSZ10 = $01;  // Character Size
  UCSZ11 = $02;  // Character Size
  USBS1 = $03;  
  UPM10 = $04;  // Parity Mode Bits
  UPM11 = $05;  // Parity Mode Bits
  UMSEL10 = $06;  // USART Mode Select
  UMSEL11 = $07;  // USART Mode Select
  // USART Control and Status Register B
  TXB81 = $00;  
  RXB81 = $01;  
  UCSZ12 = $02;  
  TXEN1 = $03;  
  RXEN1 = $04;  
  UDRIE1 = $05;  
  TXCIE1 = $06;  
  RXCIE1 = $07;  
  // USART Control and Status Register A
  MPCM1 = $00;  
  U2X1 = $01;  
  UPE1 = $02;  
  DOR1 = $03;  
  FE1 = $04;  
  UDRE1 = $05;  
  TXC1 = $06;  
  RXC1 = $07;  
  // TWI Slave Data Register
  TWSD0 = $00;  // TWI slave data bit
  TWSD1 = $01;  // TWI slave data bit
  TWSD2 = $02;  // TWI slave data bit
  TWSD3 = $03;  // TWI slave data bit
  TWSD4 = $04;  // TWI slave data bit
  TWSD5 = $05;  // TWI slave data bit
  TWSD6 = $06;  // TWI slave data bit
  TWSD7 = $07;  // TWI slave data bit
  // TWI Slave Address Mask Register
  TWAE = $00;  
  TWSAM1 = $01;  // TWI Address Mask Bits
  TWSAM2 = $02;  // TWI Address Mask Bits
  TWSAM3 = $03;  // TWI Address Mask Bits
  TWSAM4 = $04;  // TWI Address Mask Bits
  TWSAM5 = $05;  // TWI Address Mask Bits
  TWSAM6 = $06;  // TWI Address Mask Bits
  TWSAM7 = $07;  // TWI Address Mask Bits
  // TWI Slave Status Register A
  TWAS = $00;  
  TWDIR = $01;  
  TWBE = $02;  
  TWC = $03;  
  TWRA = $04;  
  TWCH = $05;  
  TWASIF = $06;  
  TWDIF = $07;  
  // TWI Slave Control Register B
  TWCMD0 = $00;
  TWCMD1 = $01;
  TWAA = $02;  
  TWHNM = $03;  
  // TWI Slave Control Register A
  TWSME = $00;  
  TWPME = $01;  
  TWSIE = $02;  
  TWEN = $03;  
  TWASIE = $04;  
  TWDIE = $05;  
  TWSHE = $07;  
  // SPI Status Register
  SPI2X = $00;  
  WCOL = $06;  
  SPIF = $07;  
  // SPI Control Register
  SPR0 = $00;  // SPI Clock Rate Selects
  SPR1 = $01;  // SPI Clock Rate Selects
  CPHA = $02;  
  CPOL = $03;  
  MSTR = $04;  
  DORD = $05;  
  SPE = $06;  
  SPIE = $07;  
  // Timer/Counter2 Control Register C
  FOC2B = $06;  
  FOC2A = $07;  
  // Timer/Counter2 Control Register B
  CS20 = $00;  // Clock Select bits
  CS21 = $01;  // Clock Select bits
  CS22 = $02;  // Clock Select bits
  ICES2 = $06;  
  ICNC2 = $07;  
  // Timer/Counter2 Control Register A
  WGM20 = $00;  // Pulse Width Modulator Select Bits
  WGM21 = $01;  // Pulse Width Modulator Select Bits
  COM2B0 = $04;  // Compare Output Mode 2B, bits
  COM2B1 = $05;  // Compare Output Mode 2B, bits
  COM2A0 = $06;  // Compare Output Mode 2A, bits
  COM2A1 = $07;  // Compare Output Mode 2A, bits


implementation
{$define RELBRANCHES}
{$i avrcommon.inc}

procedure INT0_ISR; external name 'INT0_ISR'; // Interrupt 1 External Interrupt Request 0
procedure PCINT0_ISR; external name 'PCINT0_ISR'; // Interrupt 2 Pin Change Interrupt Request 0
procedure PCINT1_ISR; external name 'PCINT1_ISR'; // Interrupt 3 Pin Change Interrupt Request 1
procedure WDT_ISR; external name 'WDT_ISR'; // Interrupt 4 Watchdog Time-out Interrupt
procedure TIMER1_CAPT_ISR; external name 'TIMER1_CAPT_ISR'; // Interrupt 5 Timer/Counter1 Capture Event
procedure TIMER1_COMPA_ISR; external name 'TIMER1_COMPA_ISR'; // Interrupt 6 Timer/Counter1 Compare Match A
procedure TIMER1_COMPB_ISR; external name 'TIMER1_COMPB_ISR'; // Interrupt 7 Timer/Counter1 Compare Match B
procedure TIMER1_OVF_ISR; external name 'TIMER1_OVF_ISR'; // Interrupt 8 Timer/Counter1 Overflow
procedure TIMER0_COMPA_ISR; external name 'TIMER0_COMPA_ISR'; // Interrupt 9 TimerCounter0 Compare Match A
procedure TIMER0_COMPB_ISR; external name 'TIMER0_COMPB_ISR'; // Interrupt 10 TimerCounter0 Compare Match B
procedure TIMER0_OVF_ISR; external name 'TIMER0_OVF_ISR'; // Interrupt 11 Timer/Couner0 Overflow
procedure ANA_COMP0_ISR; external name 'ANA_COMP0_ISR'; // Interrupt 12 Analog Comparator 0
procedure ADC_ISR; external name 'ADC_ISR'; // Interrupt 13 ADC Conversion Complete
procedure EE_RDY_ISR; external name 'EE_RDY_ISR'; // Interrupt 14 EEPROM Ready
procedure ANA_COMP1_ISR; external name 'ANA_COMP1_ISR'; // Interrupt 15 Analog Comparator 1
procedure TIMER2_CAPT_ISR; external name 'TIMER2_CAPT_ISR'; // Interrupt 16 Timer/Counter2 Capture Event
procedure TIMER2_COMPA_ISR; external name 'TIMER2_COMPA_ISR'; // Interrupt 17 Timer/Counter2 Compare Match A
procedure TIMER2_COMPB_ISR; external name 'TIMER2_COMPB_ISR'; // Interrupt 18 Timer/Counter2 Compare Match B
procedure TIMER2_OVF_ISR; external name 'TIMER2_OVF_ISR'; // Interrupt 19 Timer/Counter2 Overflow
procedure SPI_ISR; external name 'SPI_ISR'; // Interrupt 20 Serial Peripheral Interface
procedure USART0_START_ISR; external name 'USART0_START_ISR'; // Interrupt 21 USART0, Start
procedure USART0_RX_ISR; external name 'USART0_RX_ISR'; // Interrupt 22 USART0, Rx Complete
procedure USART0_UDRE_ISR; external name 'USART0_UDRE_ISR'; // Interrupt 23 USART0 Data Register Empty
procedure USART0_TX_ISR; external name 'USART0_TX_ISR'; // Interrupt 24 USART0, Tx Complete
procedure USART1_START_ISR; external name 'USART1_START_ISR'; // Interrupt 25 USART1, Start
procedure USART1_RX_ISR; external name 'USART1_RX_ISR'; // Interrupt 26 USART1, Rx Complete
procedure USART1_UDRE_ISR; external name 'USART1_UDRE_ISR'; // Interrupt 27 USART1 Data Register Empty
procedure USART1_TX_ISR; external name 'USART1_TX_ISR'; // Interrupt 28 USART1, Tx Complete
procedure TWI_SLAVE_ISR; external name 'TWI_SLAVE_ISR'; // Interrupt 29 Two-wire Serial Interface

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
  rjmp WDT_ISR
  rjmp TIMER1_CAPT_ISR
  rjmp TIMER1_COMPA_ISR
  rjmp TIMER1_COMPB_ISR
  rjmp TIMER1_OVF_ISR
  rjmp TIMER0_COMPA_ISR
  rjmp TIMER0_COMPB_ISR
  rjmp TIMER0_OVF_ISR
  rjmp ANA_COMP0_ISR
  rjmp ADC_ISR
  rjmp EE_RDY_ISR
  rjmp ANA_COMP1_ISR
  rjmp TIMER2_CAPT_ISR
  rjmp TIMER2_COMPA_ISR
  rjmp TIMER2_COMPB_ISR
  rjmp TIMER2_OVF_ISR
  rjmp SPI_ISR
  rjmp USART0_START_ISR
  rjmp USART0_RX_ISR
  rjmp USART0_UDRE_ISR
  rjmp USART0_TX_ISR
  rjmp USART1_START_ISR
  rjmp USART1_RX_ISR
  rjmp USART1_UDRE_ISR
  rjmp USART1_TX_ISR
  rjmp TWI_SLAVE_ISR

  {$i start.inc}

  .weak INT0_ISR
  .weak PCINT0_ISR
  .weak PCINT1_ISR
  .weak WDT_ISR
  .weak TIMER1_CAPT_ISR
  .weak TIMER1_COMPA_ISR
  .weak TIMER1_COMPB_ISR
  .weak TIMER1_OVF_ISR
  .weak TIMER0_COMPA_ISR
  .weak TIMER0_COMPB_ISR
  .weak TIMER0_OVF_ISR
  .weak ANA_COMP0_ISR
  .weak ADC_ISR
  .weak EE_RDY_ISR
  .weak ANA_COMP1_ISR
  .weak TIMER2_CAPT_ISR
  .weak TIMER2_COMPA_ISR
  .weak TIMER2_COMPB_ISR
  .weak TIMER2_OVF_ISR
  .weak SPI_ISR
  .weak USART0_START_ISR
  .weak USART0_RX_ISR
  .weak USART0_UDRE_ISR
  .weak USART0_TX_ISR
  .weak USART1_START_ISR
  .weak USART1_RX_ISR
  .weak USART1_UDRE_ISR
  .weak USART1_TX_ISR
  .weak TWI_SLAVE_ISR

  .set INT0_ISR, Default_IRQ_handler
  .set PCINT0_ISR, Default_IRQ_handler
  .set PCINT1_ISR, Default_IRQ_handler
  .set WDT_ISR, Default_IRQ_handler
  .set TIMER1_CAPT_ISR, Default_IRQ_handler
  .set TIMER1_COMPA_ISR, Default_IRQ_handler
  .set TIMER1_COMPB_ISR, Default_IRQ_handler
  .set TIMER1_OVF_ISR, Default_IRQ_handler
  .set TIMER0_COMPA_ISR, Default_IRQ_handler
  .set TIMER0_COMPB_ISR, Default_IRQ_handler
  .set TIMER0_OVF_ISR, Default_IRQ_handler
  .set ANA_COMP0_ISR, Default_IRQ_handler
  .set ADC_ISR, Default_IRQ_handler
  .set EE_RDY_ISR, Default_IRQ_handler
  .set ANA_COMP1_ISR, Default_IRQ_handler
  .set TIMER2_CAPT_ISR, Default_IRQ_handler
  .set TIMER2_COMPA_ISR, Default_IRQ_handler
  .set TIMER2_COMPB_ISR, Default_IRQ_handler
  .set TIMER2_OVF_ISR, Default_IRQ_handler
  .set SPI_ISR, Default_IRQ_handler
  .set USART0_START_ISR, Default_IRQ_handler
  .set USART0_RX_ISR, Default_IRQ_handler
  .set USART0_UDRE_ISR, Default_IRQ_handler
  .set USART0_TX_ISR, Default_IRQ_handler
  .set USART1_START_ISR, Default_IRQ_handler
  .set USART1_RX_ISR, Default_IRQ_handler
  .set USART1_UDRE_ISR, Default_IRQ_handler
  .set USART1_TX_ISR, Default_IRQ_handler
  .set TWI_SLAVE_ISR, Default_IRQ_handler
end;

end.
