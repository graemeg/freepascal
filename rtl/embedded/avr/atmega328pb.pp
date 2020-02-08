unit ATmega328PB;

{$goto on}
interface

var
  PINB: byte absolute $23;  // Port B Input Pins
  DDRB: byte absolute $24;  // Port B Data Direction Register
  PORTB: byte absolute $25;  // Port B Data Register
  PINC: byte absolute $26;  // Port C Input Pins
  DDRC: byte absolute $27;  // Port C Data Direction Register
  PORTC: byte absolute $28;  // Port C Data Register
  PIND: byte absolute $29;  // Port D Input Pins
  DDRD: byte absolute $2A;  // Port D Data Direction Register
  PORTD: byte absolute $2B;  // Port D Data Register
  PINE: byte absolute $2C;  // Port E Input Pins
  DDRE: byte absolute $2D;  // Port E Data Direction Register
  PORTE: byte absolute $2E;  // Port E Data Register
  TIFR0: byte absolute $35;  // Timer/Counter0 Interrupt Flag register
  TIFR1: byte absolute $36;  // Timer/Counter Interrupt Flag register
  TIFR2: byte absolute $37;  // Timer/Counter Interrupt Flag Register
  TIFR3: byte absolute $38;  // Timer/Counter3 Interrupt Flag register
  TIFR4: byte absolute $39;  // Timer/Counter4 Interrupt Flag register
  PCIFR: byte absolute $3B;  // Pin Change Interrupt Flag Register
  EIFR: byte absolute $3C;  // External Interrupt Flag Register
  EIMSK: byte absolute $3D;  // External Interrupt Mask Register
  GPIOR0: byte absolute $3E;  // General Purpose I/O Register 0
  EECR: byte absolute $3F;  // EEPROM Control Register
  EEDR: byte absolute $40;  // EEPROM Data Register
  EEAR: word absolute $41;  // EEPROM Address Register Bytes
  EEARL: byte absolute $41;  // EEPROM Address Register Bytes
  EEARH: byte absolute $42;  // EEPROM Address Register Bytes;
  GTCCR: byte absolute $43;  // General Timer/Counter Control Register
  TCCR0A: byte absolute $44;  // Timer/Counter Control Register A
  TCCR0B: byte absolute $45;  // Timer/Counter Control Register B
  TCNT0: byte absolute $46;  // Timer/Counter0
  OCR0A: byte absolute $47;  // Timer/Counter0 Output Compare Register
  OCR0B: byte absolute $48;  // Timer/Counter0 Output Compare Register
  GPIOR1: byte absolute $4A;  // General Purpose I/O Register 1
  GPIOR2: byte absolute $4B;  // General Purpose I/O Register 2
  SPCR0: byte absolute $4C;  // SPI Control Register
  SPSR0: byte absolute $4D;  // SPI Status Register
  SPDR0: byte absolute $4E;  // SPI Data Register
  ACSRB: byte absolute $4F;  // Analog Comparator Control And Status Register-B
  ACSR: byte absolute $50;  // Analog Comparator Control And Status Register
  SMCR: byte absolute $53;  // Sleep Mode Control Register
  MCUSR: byte absolute $54;  // MCU Status Register
  MCUCR: byte absolute $55;  // MCU Control Register
  SPMCSR: byte absolute $57;  // Store Program Memory Control and Status Register
  SP: word absolute $5D;  // Stack Pointer
  SPL: byte absolute $5D;  // Stack Pointer
  SPH: byte absolute $5E;  // Stack Pointer;
  SREG: byte absolute $5F;  // Status Register
  WDTCSR: byte absolute $60;  // Watchdog Timer Control Register
  CLKPR: byte absolute $61;  // Clock Prescale Register
  XFDCSR: byte absolute $62;  // XOSC Failure Detection Control and Status Register
  PRR0: byte absolute $64;  // Power Reduction Register 0
  PRR1: byte absolute $65;  // Power Reduction Register 1
  OSCCAL: byte absolute $66;  // Oscillator Calibration Value
  PCICR: byte absolute $68;  // Pin Change Interrupt Control Register
  EICRA: byte absolute $69;  // External Interrupt Control Register
  PCMSK0: byte absolute $6B;  // Pin Change Mask Register 0
  PCMSK1: byte absolute $6C;  // Pin Change Mask Register 1
  PCMSK2: byte absolute $6D;  // Pin Change Mask Register 2
  TIMSK0: byte absolute $6E;  // Timer/Counter0 Interrupt Mask Register
  TIMSK1: byte absolute $6F;  // Timer/Counter Interrupt Mask Register
  TIMSK2: byte absolute $70;  // Timer/Counter Interrupt Mask register
  TIMSK3: byte absolute $71;  // Timer/Counter Interrupt Mask Register
  TIMSK4: byte absolute $72;  // Timer/Counter4 Interrupt Mask Register
  PCMSK3: byte absolute $73;  // Pin Change Mask Register 3
  ADC: word absolute $78;  // ADC Data Register Bytes
  ADCL: byte absolute $78;  // ADC Data Register Bytes
  ADCH: byte absolute $79;  // ADC Data Register Bytes;
  ADCSRA: byte absolute $7A;  // The ADC Control and Status register A
  ADCSRB: byte absolute $7B;  // The ADC Control and Status register B
  ADMUX: byte absolute $7C;  // The ADC multiplexer Selection Register
  DIDR0: byte absolute $7E;  // Digital Input Disable Register
  DIDR1: byte absolute $7F;  // Digital Input Disable Register 1
  TCCR1A: byte absolute $80;  // Timer/Counter1 Control Register A
  TCCR1B: byte absolute $81;  // Timer/Counter1 Control Register B
  TCCR1C: byte absolute $82;  // Timer/Counter1 Control Register C
  TCNT1: word absolute $84;  // Timer/Counter1 Bytes
  TCNT1L: byte absolute $84;  // Timer/Counter1 Bytes
  TCNT1H: byte absolute $85;  // Timer/Counter1 Bytes;
  ICR1: word absolute $86;  // Timer/Counter1 Input Capture Register Bytes
  ICR1L: byte absolute $86;  // Timer/Counter1 Input Capture Register Bytes
  ICR1H: byte absolute $87;  // Timer/Counter1 Input Capture Register Bytes;
  OCR1A: word absolute $88;  // Timer/Counter1 Output Compare Register Bytes
  OCR1AL: byte absolute $88;  // Timer/Counter1 Output Compare Register Bytes
  OCR1AH: byte absolute $89;  // Timer/Counter1 Output Compare Register Bytes;
  OCR1B: word absolute $8A;  // Timer/Counter1 Output Compare Register Bytes
  OCR1BL: byte absolute $8A;  // Timer/Counter1 Output Compare Register Bytes
  OCR1BH: byte absolute $8B;  // Timer/Counter1 Output Compare Register Bytes;
  TCCR3A: byte absolute $90;  // Timer/Counter3 Control Register A
  TCCR3B: byte absolute $91;  // Timer/Counter3 Control Register B
  TCCR3C: byte absolute $92;  // Timer/Counter3 Control Register C
  TCNT3: word absolute $94;  // Timer/Counter3 Bytes
  TCNT3L: byte absolute $94;  // Timer/Counter3 Bytes
  TCNT3H: byte absolute $95;  // Timer/Counter3 Bytes;
  ICR3: word absolute $96;  // Timer/Counter3 Input Capture Register Bytes
  ICR3L: byte absolute $96;  // Timer/Counter3 Input Capture Register Bytes
  ICR3H: byte absolute $97;  // Timer/Counter3 Input Capture Register Bytes;
  OCR3A: word absolute $98;  // Timer/Counter3 Output Compare Register Bytes
  OCR3AL: byte absolute $98;  // Timer/Counter3 Output Compare Register Bytes
  OCR3AH: byte absolute $99;  // Timer/Counter3 Output Compare Register Bytes;
  OCR3B: word absolute $9A;  // Timer/Counter3 Output Compare Register Bytes
  OCR3BL: byte absolute $9A;  // Timer/Counter3 Output Compare Register Bytes
  OCR3BH: byte absolute $9B;  // Timer/Counter3 Output Compare Register Bytes;
  TCCR4A: byte absolute $A0;  // Timer/Counter4 Control Register A
  TCCR4B: byte absolute $A1;  // Timer/Counter4 Control Register B
  TCCR4C: byte absolute $A2;  // Timer/Counter4 Control Register C
  TCNT4: word absolute $A4;  // Timer/Counter4 Bytes
  TCNT4L: byte absolute $A4;  // Timer/Counter4 Bytes
  TCNT4H: byte absolute $A5;  // Timer/Counter4 Bytes;
  ICR4: word absolute $A6;  // Timer/Counter4 Input Capture Register Bytes
  ICR4L: byte absolute $A6;  // Timer/Counter4 Input Capture Register Bytes
  ICR4H: byte absolute $A7;  // Timer/Counter4 Input Capture Register Bytes;
  OCR4A: word absolute $A8;  // Timer/Counter4 Output Compare Register Bytes
  OCR4AL: byte absolute $A8;  // Timer/Counter4 Output Compare Register Bytes
  OCR4AH: byte absolute $A9;  // Timer/Counter4 Output Compare Register Bytes;
  OCR4B: word absolute $AA;  // Timer/Counter4 Output Compare Register Bytes
  OCR4BL: byte absolute $AA;  // Timer/Counter4 Output Compare Register Bytes
  OCR4BH: byte absolute $AB;  // Timer/Counter4 Output Compare Register Bytes;
  SPCR1: byte absolute $AC;  // SPI Control Register
  SPSR1: byte absolute $AD;  // SPI Status Register
  SPDR1: byte absolute $AE;  // SPI Data Register
  TCCR2A: byte absolute $B0;  // Timer/Counter2 Control Register A
  TCCR2B: byte absolute $B1;  // Timer/Counter2 Control Register B
  TCNT2: byte absolute $B2;  // Timer/Counter2
  OCR2A: byte absolute $B3;  // Timer/Counter2 Output Compare Register A
  OCR2B: byte absolute $B4;  // Timer/Counter2 Output Compare Register B
  ASSR: byte absolute $B6;  // Asynchronous Status Register
  TWBR0: byte absolute $B8;  // TWI Bit Rate register
  TWSR0: byte absolute $B9;  // TWI Status Register
  TWAR0: byte absolute $BA;  // TWI (Slave) Address register
  TWDR0: byte absolute $BB;  // TWI Data register
  TWCR0: byte absolute $BC;  // TWI Control Register
  TWAMR0: byte absolute $BD;  // TWI (Slave) Address Mask Register
  UCSR0A: byte absolute $C0;  // USART Control and Status Register A
  UCSR0B: byte absolute $C1;  // USART Control and Status Register B
  UCSR0C: byte absolute $C2;  // USART Control and Status Register C
  UCSR0D: byte absolute $C3;  // USART Control and Status Register D
  UBRR0: word absolute $C4;  // USART Baud Rate Register Bytes
  UBRR0L: byte absolute $C4;  // USART Baud Rate Register Bytes
  UBRR0H: byte absolute $C5;  // USART Baud Rate Register Bytes;
  UDR0: byte absolute $C6;  // USART I/O Data Register 0
  UCSR1A: byte absolute $C8;  // USART Control and Status Register A
  UCSR1B: byte absolute $C9;  // USART Control and Status Register B
  UCSR1C: byte absolute $CA;  // USART Control and Status Register C
  UCSR1D: byte absolute $CB;  // USART Control and Status Register D
  UBRR1: word absolute $CC;  // USART Baud Rate Register Bytes
  UBRR1L: byte absolute $CC;  // USART Baud Rate Register Bytes
  UBRR1H: byte absolute $CD;  // USART Baud Rate Register Bytes;
  UDR1: byte absolute $CE;  // USART I/O Data Register
  TWBR1: byte absolute $D8;  // TWI Bit Rate register
  TWSR1: byte absolute $D9;  // TWI Status Register
  TWAR1: byte absolute $DA;  // TWI (Slave) Address register
  TWDR1: byte absolute $DB;  // TWI Data register
  TWCR1: byte absolute $DC;  // TWI Control Register
  TWAMR1: byte absolute $DD;  // TWI (Slave) Address Mask Register

const
  // Port B Data Register
  PB0 = $00;  
  PB1 = $01;  
  PB2 = $02;  
  PB3 = $03;  
  PB4 = $04;  
  PB5 = $05;  
  PB6 = $06;  
  PB7 = $07;  
  // Port C Data Register
  PC0 = $00;  
  PC1 = $01;  
  PC2 = $02;  
  PC3 = $03;  
  PC4 = $04;  
  PC5 = $05;  
  PC6 = $06;  
  // Port D Data Register
  PD0 = $00;  
  PD1 = $01;  
  PD2 = $02;  
  PD3 = $03;  
  PD4 = $04;  
  PD5 = $05;  
  PD6 = $06;  
  PD7 = $07;  
  // Port E Data Register
  PE0 = $00;  
  PE1 = $01;  
  PE2 = $02;  
  PE3 = $03;  
  // Timer/Counter0 Interrupt Flag register
  TOV0 = $00;  
  OCF0A = $01;  
  OCF0B = $02;  
  // Timer/Counter Interrupt Flag register
  TOV1 = $00;  
  OCF1A = $01;  
  OCF1B = $02;  
  ICF1 = $05;  
  // Timer/Counter Interrupt Flag Register
  TOV2 = $00;  
  OCF2A = $01;  
  OCF2B = $02;  
  // Timer/Counter3 Interrupt Flag register
  TOV3 = $00;  
  OCF3A = $01;  
  OCF3B = $02;  
  ICF3 = $05;  
  // Timer/Counter4 Interrupt Flag register
  TOV4 = $00;  
  OCF4A = $01;  
  OCF4B = $02;  
  ICF4 = $05;  
  // Pin Change Interrupt Flag Register
  PCIF0 = $00;  // Pin Change Interrupt Flags
  PCIF1 = $01;  // Pin Change Interrupt Flags
  PCIF2 = $02;  // Pin Change Interrupt Flags
  PCIF3 = $03;  // Pin Change Interrupt Flags
  // External Interrupt Flag Register
  INTF0 = $00;  // External Interrupt Flags
  INTF1 = $01;  // External Interrupt Flags
  // External Interrupt Mask Register
  INT0 = $00;  // External Interrupt Request 1 Enable
  INT1 = $01;  // External Interrupt Request 1 Enable
  // EEPROM Control Register
  EERE = $00;  
  EEPE = $01;  
  EEMPE = $02;  
  EERIE = $03;  
  EEPM0 = $04;  // EEPROM Programming Mode Bits
  EEPM1 = $05;  // EEPROM Programming Mode Bits
  // General Timer/Counter Control Register
  PSRSYNC = $00;  
  PSRASY = $01;  
  TSM = $07;  
  // Timer/Counter Control Register A
  WGM00 = $00;  // Waveform Generation Mode
  WGM01 = $01;  // Waveform Generation Mode
  COM0B0 = $04;  // Compare Output Mode, Fast PWM
  COM0B1 = $05;  // Compare Output Mode, Fast PWM
  COM0A0 = $06;  // Compare Output Mode, Phase Correct PWM Mode
  COM0A1 = $07;  // Compare Output Mode, Phase Correct PWM Mode
  // Timer/Counter Control Register B
  CS00 = $00;  // Clock Select
  CS01 = $01;  // Clock Select
  CS02 = $02;  // Clock Select
  WGM02 = $03;  
  FOC0B = $06;  
  FOC0A = $07;  
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
  // Analog Comparator Control And Status Register-B
  ACOE = $00;  
  // Analog Comparator Control And Status Register
  ACIS0 = $00;  // Analog Comparator Interrupt Mode Select bits
  ACIS1 = $01;  // Analog Comparator Interrupt Mode Select bits
  ACIC = $02;  
  ACIE = $03;  
  ACI = $04;  
  ACO = $05;  
  ACBG = $06;  
  ACD = $07;  
  // Sleep Mode Control Register
  SE = $00;  
  SM0 = $01;  // Sleep Mode Select Bits
  SM1 = $02;  // Sleep Mode Select Bits
  SM2 = $03;  // Sleep Mode Select Bits
  // MCU Status Register
  PORF = $00;  
  EXTRF = $01;  
  BORF = $02;  
  WDRF = $03;  
  // MCU Control Register
  IVCE = $00;  
  IVSEL = $01;  
  PUD = $04;  
  BODSE = $05;  
  BODS = $06;  
  // Store Program Memory Control and Status Register
  SPMEN = $00;  
  PGERS = $01;  
  PGWRT = $02;  
  BLBSET = $03;  
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
  CLKPS2 = $02;  // Clock Prescaler Select Bits
  CLKPS3 = $03;  // Clock Prescaler Select Bits
  CLKPCE = $07;  
  // XOSC Failure Detection Control and Status Register
  XFDIE = $00;  
  XFDIF = $01;  
  // Power Reduction Register 0
  PRADC = $00;  
  PRUSART0 = $01;  
  PRSPI0 = $02;  
  PRTIM1 = $03;  
  PRUSART1 = $04;  
  PRTIM0 = $05;  
  PRTIM2 = $06;  
  PRTWI0 = $07;  
  // Power Reduction Register 1
  PRTIM3 = $00;  
  PRSPI1 = $02;  
  PRTIM4 = $03;  
  PRPTC = $04;  
  PRTWI1 = $05;  
  // Oscillator Calibration Value
  OSCCAL0 = $00;  // Oscillator Calibration
  OSCCAL1 = $01;  // Oscillator Calibration
  OSCCAL2 = $02;  // Oscillator Calibration
  OSCCAL3 = $03;  // Oscillator Calibration
  OSCCAL4 = $04;  // Oscillator Calibration
  OSCCAL5 = $05;  // Oscillator Calibration
  OSCCAL6 = $06;  // Oscillator Calibration
  OSCCAL7 = $07;  // Oscillator Calibration
  // Pin Change Interrupt Control Register
  PCIE0 = $00;  // Pin Change Interrupt Enables
  PCIE1 = $01;  // Pin Change Interrupt Enables
  PCIE2 = $02;  // Pin Change Interrupt Enables
  PCIE3 = $03;  // Pin Change Interrupt Enables
  // External Interrupt Control Register
  ISC00 = $00;  // External Interrupt Sense Control 0 Bits
  ISC01 = $01;  // External Interrupt Sense Control 0 Bits
  ISC10 = $02;  // External Interrupt Sense Control 1 Bits
  ISC11 = $03;  // External Interrupt Sense Control 1 Bits
  // Timer/Counter0 Interrupt Mask Register
  TOIE0 = $00;  
  OCIE0A = $01;  
  OCIE0B = $02;  
  // Timer/Counter Interrupt Mask Register
  TOIE1 = $00;  
  OCIE1A = $01;  
  OCIE1B = $02;  
  ICIE1 = $05;  
  // Timer/Counter Interrupt Mask register
  TOIE2 = $00;  
  OCIE2A = $01;  
  OCIE2B = $02;  
  // Timer/Counter Interrupt Mask Register
  TOIE3 = $00;  
  OCIE3A = $01;  
  OCIE3B = $02;  
  ICIE3 = $05;  
  // Timer/Counter4 Interrupt Mask Register
  TOIE4 = $00;  
  OCIE4A = $01;  
  OCIE4B = $02;  
  ICIE4 = $05;  
  // Pin Change Mask Register 3
  PCINT24 = $00;  // Pin Change Enable Masks
  PCINT25 = $01;  // Pin Change Enable Masks
  PCINT26 = $02;  // Pin Change Enable Masks
  PCINT27 = $03;  // Pin Change Enable Masks
  // The ADC Control and Status register A
  ADPS0 = $00;  // ADC Prescaler Select Bits
  ADPS1 = $01;  // ADC Prescaler Select Bits
  ADPS2 = $02;  // ADC Prescaler Select Bits
  ADIE = $03;  
  ADIF = $04;  
  ADATE = $05;  
  ADSC = $06;  
  ADEN = $07;  
  // The ADC Control and Status register B
  ADTS0 = $00;  // ADC Auto Trigger Source bits
  ADTS1 = $01;  // ADC Auto Trigger Source bits
  ADTS2 = $02;  // ADC Auto Trigger Source bits
  ACME = $06;  
  // The ADC multiplexer Selection Register
  MUX0 = $00;  // Analog Channel Selection Bits
  MUX1 = $01;  // Analog Channel Selection Bits
  MUX2 = $02;  // Analog Channel Selection Bits
  MUX3 = $03;  // Analog Channel Selection Bits
  ADLAR = $05;  
  REFS0 = $06;  // Reference Selection Bits
  REFS1 = $07;  // Reference Selection Bits
  // Digital Input Disable Register
  ADC0D = $00;  
  ADC1D = $01;  
  ADC2D = $02;  
  ADC3D = $03;  
  ADC4D = $04;  
  ADC5D = $05;  
  // Digital Input Disable Register 1
  AIN0D = $00;  
  AIN1D = $01;  
  // Timer/Counter1 Control Register A
  WGM10 = $00;  // Waveform Generation Mode
  WGM11 = $01;  // Waveform Generation Mode
  COM1B0 = $04;  // Compare Output Mode 1B, bits
  COM1B1 = $05;  // Compare Output Mode 1B, bits
  COM1A0 = $06;  // Compare Output Mode 1A, bits
  COM1A1 = $07;  // Compare Output Mode 1A, bits
  // Timer/Counter1 Control Register B
  CS10 = $00;  // Prescaler source of Timer/Counter 1
  CS11 = $01;  // Prescaler source of Timer/Counter 1
  CS12 = $02;  // Prescaler source of Timer/Counter 1
  ICES1 = $06;  
  ICNC1 = $07;  
  // Timer/Counter1 Control Register C
  FOC1B = $06;  
  FOC1A = $07;  
  // Timer/Counter3 Control Register A
  WGM30 = $00;  // Waveform Genration Mode
  WGM31 = $01;  // Waveform Genration Mode
  COM3B0 = $04;  // Compare Output Mode bits
  COM3B1 = $05;  // Compare Output Mode bits
  COM3A0 = $06;  // Compare Output Mode bits
  COM3A1 = $07;  // Compare Output Mode bits
  // Timer/Counter3 Control Register B
  CS30 = $00;  // Clock Select bits
  CS31 = $01;  // Clock Select bits
  CS32 = $02;  // Clock Select bits
  WGM32 = $03;  
  WGM33 = $04;  
  ICES3 = $06;  
  ICNC3 = $07;  
  // Timer/Counter3 Control Register C
  FOC3B = $06;  
  FOC3A = $07;  
  // Timer/Counter4 Control Register A
  WGM40 = $00;  // Waveform Generation Mode
  WGM41 = $01;  // Waveform Generation Mode
  COM4B0 = $04;  // Compare Output Mode bits
  COM4B1 = $05;  // Compare Output Mode bits
  COM4A0 = $06;  // Compare Output Mode bits
  COM4A1 = $07;  // Compare Output Mode bits
  // Timer/Counter4 Control Register B
  CS40 = $00;  // Clock Select bits
  CS41 = $01;  // Clock Select bits
  CS42 = $02;  // Clock Select bits
  WGM42 = $03;  
  WGM43 = $04;  
  ICES4 = $06;  
  ICNC4 = $07;  
  // Timer/Counter4 Control Register C
  FOC4B = $06;  
  FOC4A = $07;  
  // SPI Control Register
  SPR10 = $00;  // SPI Clock Rate Selects
  SPR11 = $01;  // SPI Clock Rate Selects
  CPHA1 = $02;  
  CPOL1 = $03;  
  MSTR1 = $04;  
  DORD1 = $05;  
  SPE1 = $06;  
  SPIE1 = $07;  
  // SPI Status Register
  SPI2X1 = $00;  
  WCOL1 = $06;  
  SPIF1 = $07;  
  // Timer/Counter2 Control Register A
  WGM20 = $00;  // Waveform Genration Mode
  WGM21 = $01;  // Waveform Genration Mode
  COM2B0 = $04;  // Compare Output Mode bits
  COM2B1 = $05;  // Compare Output Mode bits
  COM2A0 = $06;  // Compare Output Mode bits
  COM2A1 = $07;  // Compare Output Mode bits
  // Timer/Counter2 Control Register B
  CS20 = $00;  // Clock Select bits
  CS21 = $01;  // Clock Select bits
  CS22 = $02;  // Clock Select bits
  WGM22 = $03;  
  FOC2B = $06;  
  FOC2A = $07;  
  // Asynchronous Status Register
  TCR2BUB = $00;  
  TCR2AUB = $01;  
  OCR2BUB = $02;  
  OCR2AUB = $03;  
  TCN2UB = $04;  
  AS2 = $05;  
  EXCLK = $06;  
  // TWI Status Register
  TWPS0 = $00;  // TWI Prescaler
  TWPS1 = $01;  // TWI Prescaler
  TWS3 = $03;  // TWI Status
  TWS4 = $04;  // TWI Status
  TWS5 = $05;  // TWI Status
  TWS6 = $06;  // TWI Status
  TWS7 = $07;  // TWI Status
  // TWI (Slave) Address register
  TWGCE = $00;  
  TWA0 = $01;  // TWI (Slave) Address register Bits
  TWA1 = $02;  // TWI (Slave) Address register Bits
  TWA2 = $03;  // TWI (Slave) Address register Bits
  TWA3 = $04;  // TWI (Slave) Address register Bits
  TWA4 = $05;  // TWI (Slave) Address register Bits
  TWA5 = $06;  // TWI (Slave) Address register Bits
  TWA6 = $07;  // TWI (Slave) Address register Bits
  // TWI Control Register
  TWIE = $00;  
  TWEN = $02;  
  TWWC = $03;  
  TWSTO = $04;  
  TWSTA = $05;  
  TWEA = $06;  
  TWINT = $07;  
  // TWI (Slave) Address Mask Register
  TWAM0 = $01;
  TWAM1 = $02;
  TWAM2 = $03;
  TWAM3 = $04;
  TWAM4 = $05;
  TWAM5 = $06;
  TWAM6 = $07;
  // USART Control and Status Register A
  MPCM0 = $00;  
  U2X0 = $01;  
  UPE0 = $02;  
  DOR0 = $03;  
  FE0 = $04;  
  UDRE0 = $05;  
  TXC0 = $06;  
  RXC0 = $07;  
  // USART Control and Status Register B
  TXB80 = $00;  
  RXB80 = $01;  
  UCSZ02 = $02;  
  TXEN0 = $03;  
  RXEN0 = $04;  
  UDRIE0 = $05;  
  TXCIE0 = $06;  
  RXCIE0 = $07;  
  // USART Control and Status Register C
  UCPOL0 = $00;  
  UCSZ00 = $01;  // Character Size - together with UCSZ2 in UCSR0B
  UCSZ01 = $02;  // Character Size - together with UCSZ2 in UCSR0B
  USBS0 = $03;  
  UPM00 = $04;  // Parity Mode Bits
  UPM01 = $05;  // Parity Mode Bits
  UMSEL00 = $06;  // USART Mode Select
  UMSEL01 = $07;  // USART Mode Select
  // USART Control and Status Register D
  SFDE = $05;  
  RXS = $06;  
  RXSIE = $07;  
  // USART Control and Status Register A
  MPCM1 = $00;  
  U2X1 = $01;  
  UPE1 = $02;  
  DOR1 = $03;  
  FE1 = $04;  
  UDRE1 = $05;  
  TXC1 = $06;  
  RXC1 = $07;  
  // USART Control and Status Register B
  TXB81 = $00;  
  RXB81 = $01;  
  UCSZ12 = $02;  
  TXEN1 = $03;  
  RXEN1 = $04;  
  UDRIE1 = $05;  
  TXCIE1 = $06;  
  RXCIE1 = $07;  
  // USART Control and Status Register C
  UCPOL1 = $00;  
  UCSZ10 = $01;  // Character Size - together with UCSZ12 in UCSR1B
  UCSZ11 = $02;  // Character Size - together with UCSZ12 in UCSR1B
  USBS1 = $03;  
  UPM10 = $04;  // Parity Mode Bits
  UPM11 = $05;  // Parity Mode Bits
  UMSEL10 = $06;  // USART Mode Select
  UMSEL11 = $07;  // USART Mode Select
  // USART Control and Status Register D
  SFDE1 = $05;  
  RXS1 = $06;  
  RXSIE1 = $07;  
  // TWI Status Register
  TWPS10 = $00;  // TWI Prescaler
  TWPS11 = $01;  // TWI Prescaler
  TWS13 = $03;  // TWI Status
  TWS14 = $04;  // TWI Status
  TWS15 = $05;  // TWI Status
  TWS16 = $06;  // TWI Status
  TWS17 = $07;  // TWI Status
  // TWI Control Register
  TWIE1 = $00;  
  TWEN1 = $02;  
  TWWC1 = $03;  
  TWSTO1 = $04;  
  TWSTA1 = $05;  
  TWEA1 = $06;  
  TWINT1 = $07;  
  // TWI (Slave) Address Mask Register
  TWAM10 = $01;
  TWAM11 = $02;
  TWAM12 = $03;
  TWAM13 = $04;
  TWAM14 = $05;
  TWAM15 = $06;
  TWAM16 = $07;


implementation

{$i avrcommon.inc}

procedure INT0_ISR; external name 'INT0_ISR'; // Interrupt 1 External Interrupt Request 0
procedure INT1_ISR; external name 'INT1_ISR'; // Interrupt 2 External Interrupt Request 1
procedure PCINT0_ISR; external name 'PCINT0_ISR'; // Interrupt 3 Pin Change Interrupt Request 0
procedure PCINT1_ISR; external name 'PCINT1_ISR'; // Interrupt 4 Pin Change Interrupt Request 1
procedure PCINT2_ISR; external name 'PCINT2_ISR'; // Interrupt 5 Pin Change Interrupt Request 2
procedure WDT_ISR; external name 'WDT_ISR'; // Interrupt 6 Watchdog Time-out Interrupt
procedure TIMER2_COMPA_ISR; external name 'TIMER2_COMPA_ISR'; // Interrupt 7 Timer/Counter2 Compare Match A
procedure TIMER2_COMPB_ISR; external name 'TIMER2_COMPB_ISR'; // Interrupt 8 Timer/Counter2 Compare Match B
procedure TIMER2_OVF_ISR; external name 'TIMER2_OVF_ISR'; // Interrupt 9 Timer/Counter2 Overflow
procedure TIMER1_CAPT_ISR; external name 'TIMER1_CAPT_ISR'; // Interrupt 10 Timer/Counter1 Capture Event
procedure TIMER1_COMPA_ISR; external name 'TIMER1_COMPA_ISR'; // Interrupt 11 Timer/Counter1 Compare Match A
procedure TIMER1_COMPB_ISR; external name 'TIMER1_COMPB_ISR'; // Interrupt 12 Timer/Counter1 Compare Match B
procedure TIMER1_OVF_ISR; external name 'TIMER1_OVF_ISR'; // Interrupt 13 Timer/Counter1 Overflow
procedure TIMER0_COMPA_ISR; external name 'TIMER0_COMPA_ISR'; // Interrupt 14 TimerCounter0 Compare Match A
procedure TIMER0_COMPB_ISR; external name 'TIMER0_COMPB_ISR'; // Interrupt 15 TimerCounter0 Compare Match B
procedure TIMER0_OVF_ISR; external name 'TIMER0_OVF_ISR'; // Interrupt 16 Timer/Couner0 Overflow
procedure SPI0_STC_ISR; external name 'SPI0_STC_ISR'; // Interrupt 17 SPI Serial Transfer Complete
procedure USART0_RX_ISR; external name 'USART0_RX_ISR'; // Interrupt 18 USART0 Rx Complete
procedure USART0_UDRE_ISR; external name 'USART0_UDRE_ISR'; // Interrupt 19 USART0, Data Register Empty
procedure USART0_TX_ISR; external name 'USART0_TX_ISR'; // Interrupt 20 USART0 Tx Complete
procedure ADC_ISR; external name 'ADC_ISR'; // Interrupt 21 ADC Conversion Complete
procedure EE_READY_ISR; external name 'EE_READY_ISR'; // Interrupt 22 EEPROM Ready
procedure ANALOG_COMP_ISR; external name 'ANALOG_COMP_ISR'; // Interrupt 23 Analog Comparator
procedure TWI0_ISR; external name 'TWI0_ISR'; // Interrupt 24 Two-wire Serial Interface
procedure SPM_Ready_ISR; external name 'SPM_Ready_ISR'; // Interrupt 25 Store Program Memory Read
procedure USART0_START_ISR; external name 'USART0_START_ISR'; // Interrupt 26 USART0 Start frame detection
procedure PCINT3_ISR; external name 'PCINT3_ISR'; // Interrupt 27 Pin Change Interrupt Request 3
procedure USART1_RX_ISR; external name 'USART1_RX_ISR'; // Interrupt 28 USART1 Rx Complete
procedure USART1_UDRE_ISR; external name 'USART1_UDRE_ISR'; // Interrupt 29 USART1, Data Register Empty
procedure USART1_TX_ISR; external name 'USART1_TX_ISR'; // Interrupt 30 USART1 Tx Complete
procedure USART1_START_ISR; external name 'USART1_START_ISR'; // Interrupt 31 USART1 Start frame detection
procedure TIMER3_CAPT_ISR; external name 'TIMER3_CAPT_ISR'; // Interrupt 32 Timer/Counter3 Capture Event
procedure TIMER3_COMPA_ISR; external name 'TIMER3_COMPA_ISR'; // Interrupt 33 Timer/Counter3 Compare Match A
procedure TIMER3_COMPB_ISR; external name 'TIMER3_COMPB_ISR'; // Interrupt 34 Timer/Counter3 Compare Match B
procedure TIMER3_OVF_ISR; external name 'TIMER3_OVF_ISR'; // Interrupt 35 Timer/Counter3 Overflow
procedure CFD_ISR; external name 'CFD_ISR'; // Interrupt 36 Clock failure detection interrupt
procedure PTC_EOC_ISR; external name 'PTC_EOC_ISR'; // Interrupt 37 PTC End of conversion
procedure PTC_WCOMP_ISR; external name 'PTC_WCOMP_ISR'; // Interrupt 38 PTC Window comparator mode
procedure SPI1_STC_ISR; external name 'SPI1_STC_ISR'; // Interrupt 39 SPI1 Serial Transfer Complete
procedure TWI1_ISR; external name 'TWI1_ISR'; // Interrupt 40 TWI Transfer Complete
procedure TIMER4_CAPT_ISR; external name 'TIMER4_CAPT_ISR'; // Interrupt 41 Timer/Counter4 Capture Event
procedure TIMER4_COMPA_ISR; external name 'TIMER4_COMPA_ISR'; // Interrupt 42 Timer/Counter4 Compare Match A
procedure TIMER4_COMPB_ISR; external name 'TIMER4_COMPB_ISR'; // Interrupt 43 Timer/Counter4 Compare Match B
procedure TIMER4_OVF_ISR; external name 'TIMER4_OVF_ISR'; // Interrupt 44 Timer/Counter4 Overflow

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  jmp _start
  jmp INT0_ISR
  jmp INT1_ISR
  jmp PCINT0_ISR
  jmp PCINT1_ISR
  jmp PCINT2_ISR
  jmp WDT_ISR
  jmp TIMER2_COMPA_ISR
  jmp TIMER2_COMPB_ISR
  jmp TIMER2_OVF_ISR
  jmp TIMER1_CAPT_ISR
  jmp TIMER1_COMPA_ISR
  jmp TIMER1_COMPB_ISR
  jmp TIMER1_OVF_ISR
  jmp TIMER0_COMPA_ISR
  jmp TIMER0_COMPB_ISR
  jmp TIMER0_OVF_ISR
  jmp SPI0_STC_ISR
  jmp USART0_RX_ISR
  jmp USART0_UDRE_ISR
  jmp USART0_TX_ISR
  jmp ADC_ISR
  jmp EE_READY_ISR
  jmp ANALOG_COMP_ISR
  jmp TWI0_ISR
  jmp SPM_Ready_ISR
  jmp USART0_START_ISR
  jmp PCINT3_ISR
  jmp USART1_RX_ISR
  jmp USART1_UDRE_ISR
  jmp USART1_TX_ISR
  jmp USART1_START_ISR
  jmp TIMER3_CAPT_ISR
  jmp TIMER3_COMPA_ISR
  jmp TIMER3_COMPB_ISR
  jmp TIMER3_OVF_ISR
  jmp CFD_ISR
  jmp PTC_EOC_ISR
  jmp PTC_WCOMP_ISR
  jmp SPI1_STC_ISR
  jmp TWI1_ISR
  jmp TIMER4_CAPT_ISR
  jmp TIMER4_COMPA_ISR
  jmp TIMER4_COMPB_ISR
  jmp TIMER4_OVF_ISR

  {$i start.inc}

  .weak INT0_ISR
  .weak INT1_ISR
  .weak PCINT0_ISR
  .weak PCINT1_ISR
  .weak PCINT2_ISR
  .weak WDT_ISR
  .weak TIMER2_COMPA_ISR
  .weak TIMER2_COMPB_ISR
  .weak TIMER2_OVF_ISR
  .weak TIMER1_CAPT_ISR
  .weak TIMER1_COMPA_ISR
  .weak TIMER1_COMPB_ISR
  .weak TIMER1_OVF_ISR
  .weak TIMER0_COMPA_ISR
  .weak TIMER0_COMPB_ISR
  .weak TIMER0_OVF_ISR
  .weak SPI0_STC_ISR
  .weak USART0_RX_ISR
  .weak USART0_UDRE_ISR
  .weak USART0_TX_ISR
  .weak ADC_ISR
  .weak EE_READY_ISR
  .weak ANALOG_COMP_ISR
  .weak TWI0_ISR
  .weak SPM_Ready_ISR
  .weak USART0_START_ISR
  .weak PCINT3_ISR
  .weak USART1_RX_ISR
  .weak USART1_UDRE_ISR
  .weak USART1_TX_ISR
  .weak USART1_START_ISR
  .weak TIMER3_CAPT_ISR
  .weak TIMER3_COMPA_ISR
  .weak TIMER3_COMPB_ISR
  .weak TIMER3_OVF_ISR
  .weak CFD_ISR
  .weak PTC_EOC_ISR
  .weak PTC_WCOMP_ISR
  .weak SPI1_STC_ISR
  .weak TWI1_ISR
  .weak TIMER4_CAPT_ISR
  .weak TIMER4_COMPA_ISR
  .weak TIMER4_COMPB_ISR
  .weak TIMER4_OVF_ISR

  .set INT0_ISR, Default_IRQ_handler
  .set INT1_ISR, Default_IRQ_handler
  .set PCINT0_ISR, Default_IRQ_handler
  .set PCINT1_ISR, Default_IRQ_handler
  .set PCINT2_ISR, Default_IRQ_handler
  .set WDT_ISR, Default_IRQ_handler
  .set TIMER2_COMPA_ISR, Default_IRQ_handler
  .set TIMER2_COMPB_ISR, Default_IRQ_handler
  .set TIMER2_OVF_ISR, Default_IRQ_handler
  .set TIMER1_CAPT_ISR, Default_IRQ_handler
  .set TIMER1_COMPA_ISR, Default_IRQ_handler
  .set TIMER1_COMPB_ISR, Default_IRQ_handler
  .set TIMER1_OVF_ISR, Default_IRQ_handler
  .set TIMER0_COMPA_ISR, Default_IRQ_handler
  .set TIMER0_COMPB_ISR, Default_IRQ_handler
  .set TIMER0_OVF_ISR, Default_IRQ_handler
  .set SPI0_STC_ISR, Default_IRQ_handler
  .set USART0_RX_ISR, Default_IRQ_handler
  .set USART0_UDRE_ISR, Default_IRQ_handler
  .set USART0_TX_ISR, Default_IRQ_handler
  .set ADC_ISR, Default_IRQ_handler
  .set EE_READY_ISR, Default_IRQ_handler
  .set ANALOG_COMP_ISR, Default_IRQ_handler
  .set TWI0_ISR, Default_IRQ_handler
  .set SPM_Ready_ISR, Default_IRQ_handler
  .set USART0_START_ISR, Default_IRQ_handler
  .set PCINT3_ISR, Default_IRQ_handler
  .set USART1_RX_ISR, Default_IRQ_handler
  .set USART1_UDRE_ISR, Default_IRQ_handler
  .set USART1_TX_ISR, Default_IRQ_handler
  .set USART1_START_ISR, Default_IRQ_handler
  .set TIMER3_CAPT_ISR, Default_IRQ_handler
  .set TIMER3_COMPA_ISR, Default_IRQ_handler
  .set TIMER3_COMPB_ISR, Default_IRQ_handler
  .set TIMER3_OVF_ISR, Default_IRQ_handler
  .set CFD_ISR, Default_IRQ_handler
  .set PTC_EOC_ISR, Default_IRQ_handler
  .set PTC_WCOMP_ISR, Default_IRQ_handler
  .set SPI1_STC_ISR, Default_IRQ_handler
  .set TWI1_ISR, Default_IRQ_handler
  .set TIMER4_CAPT_ISR, Default_IRQ_handler
  .set TIMER4_COMPA_ISR, Default_IRQ_handler
  .set TIMER4_COMPB_ISR, Default_IRQ_handler
  .set TIMER4_OVF_ISR, Default_IRQ_handler
end;

end.
