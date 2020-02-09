unit ATmega64RFR2;

{$goto on}
interface

var
  PINA: byte absolute $20;  // Port A Input Pins Address
  DDRA: byte absolute $21;  // Port A Data Direction Register
  PORTA: byte absolute $22;  // Port A Data Register
  PINB: byte absolute $23;  // Port B Input Pins Address
  DDRB: byte absolute $24;  // Port B Data Direction Register
  PORTB: byte absolute $25;  // Port B Data Register
  PINC: byte absolute $26;  // Port C Input Pins Address
  DDRC: byte absolute $27;  // Port C Data Direction Register
  PORTC: byte absolute $28;  // Port C Data Register
  PIND: byte absolute $29;  // Port D Input Pins Address
  DDRD: byte absolute $2A;  // Port D Data Direction Register
  PORTD: byte absolute $2B;  // Port D Data Register
  PINE: byte absolute $2C;  // Port E Input Pins Address
  DDRE: byte absolute $2D;  // Port E Data Direction Register
  PORTE: byte absolute $2E;  // Port E Data Register
  PINF: byte absolute $2F;  // Port F Input Pins Address
  DDRF: byte absolute $30;  // Port F Data Direction Register
  PORTF: byte absolute $31;  // Port F Data Register
  PING: byte absolute $32;  // Port G Input Pins Address
  DDRG: byte absolute $33;  // Port G Data Direction Register
  PORTG: byte absolute $34;  // Port G Data Register
  TIFR0: byte absolute $35;  // Timer/Counter0 Interrupt Flag Register
  TIFR1: byte absolute $36;  // Timer/Counter1 Interrupt Flag Register
  TIFR2: byte absolute $37;  // Timer/Counter Interrupt Flag Register
  TIFR3: byte absolute $38;  // Timer/Counter3 Interrupt Flag Register
  TIFR4: byte absolute $39;  // Timer/Counter4 Interrupt Flag Register
  TIFR5: byte absolute $3A;  // Timer/Counter5 Interrupt Flag Register
  PCIFR: byte absolute $3B;  // Pin Change Interrupt Flag Register
  EIFR: byte absolute $3C;  // External Interrupt Flag Register
  EIMSK: byte absolute $3D;  // External Interrupt Mask Register
  GPIOR0: byte absolute $3E;  // General Purpose IO Register 0
  EECR: byte absolute $3F;  // EEPROM Control Register
  EEDR: byte absolute $40;  // EEPROM Data Register
  EEAR: word absolute $41;  // EEPROM Address Register  Bytes
  EEARL: byte absolute $41;  // EEPROM Address Register  Bytes
  EEARH: byte absolute $42;  // EEPROM Address Register  Bytes;
  GTCCR: byte absolute $43;  // General Timer Counter Control register
  TCCR0A: byte absolute $44;  // Timer/Counter0 Control Register A
  TCCR0B: byte absolute $45;  // Timer/Counter0 Control Register B
  TCNT0: byte absolute $46;  // Timer/Counter0 Register
  OCR0A: byte absolute $47;  // Timer/Counter0 Output Compare Register
  OCR0B: byte absolute $48;  // Timer/Counter0 Output Compare Register B
  GPIOR1: byte absolute $4A;  // General Purpose IO Register 1
  GPIOR2: byte absolute $4B;  // General Purpose I/O Register 2
  SPCR: byte absolute $4C;  // SPI Control Register
  SPSR: byte absolute $4D;  // SPI Status Register
  SPDR: byte absolute $4E;  // SPI Data Register
  ACSR: byte absolute $50;  // Analog Comparator Control And Status Register
  OCDR: byte absolute $51;  // On-Chip Debug Register
  SMCR: byte absolute $53;  // Sleep Mode Control Register
  MCUSR: byte absolute $54;  // MCU Status Register
  MCUCR: byte absolute $55;  // MCU Control Register
  SPMCSR: byte absolute $57;  // Store Program Memory Control Register
  SP: word absolute $5D;  // Stack Pointer 
  SPL: byte absolute $5D;  // Stack Pointer 
  SPH: byte absolute $5E;  // Stack Pointer ;
  SREG: byte absolute $5F;  // Status Register
  WDTCSR: byte absolute $60;  // Watchdog Timer Control Register
  CLKPR: byte absolute $61;  // Clock Prescale Register
  PRR2: byte absolute $63;  // Power Reduction Register 2
  PRR0: byte absolute $64;  // Power Reduction Register0
  PRR1: byte absolute $65;  // Power Reduction Register 1
  OSCCAL: byte absolute $66;  // Oscillator Calibration Value
  BGCR: byte absolute $67;  // Reference Voltage Calibration Register
  PCICR: byte absolute $68;  // Pin Change Interrupt Control Register
  EICRA: byte absolute $69;  // External Interrupt Control Register A
  EICRB: byte absolute $6A;  // External Interrupt Control Register B
  PCMSK0: byte absolute $6B;  // Pin Change Mask Register 0
  PCMSK1: byte absolute $6C;  // Pin Change Mask Register 1
  PCMSK2: byte absolute $6D;  // Pin Change Mask Register 2
  TIMSK0: byte absolute $6E;  // Timer/Counter0 Interrupt Mask Register
  TIMSK1: byte absolute $6F;  // Timer/Counter1 Interrupt Mask Register
  TIMSK2: byte absolute $70;  // Timer/Counter Interrupt Mask register
  TIMSK3: byte absolute $71;  // Timer/Counter3 Interrupt Mask Register
  TIMSK4: byte absolute $72;  // Timer/Counter4 Interrupt Mask Register
  TIMSK5: byte absolute $73;  // Timer/Counter5 Interrupt Mask Register
  NEMCR: byte absolute $75;  // Flash Extended-Mode Control-Register
  ADCSRC: byte absolute $77;  // The ADC Control and Status Register C
  ADC: word absolute $78;  // ADC Data Register  Bytes
  ADCL: byte absolute $78;  // ADC Data Register  Bytes
  ADCH: byte absolute $79;  // ADC Data Register  Bytes;
  ADCSRA: byte absolute $7A;  // The ADC Control and Status Register A
  ADCSRB: byte absolute $7B;  // The ADC Control and Status Register B
  ADMUX: byte absolute $7C;  // The ADC Multiplexer Selection Register
  DIDR2: byte absolute $7D;  // Digital Input Disable Register 2
  DIDR0: byte absolute $7E;  // Digital Input Disable Register 0
  DIDR1: byte absolute $7F;  // Digital Input Disable Register 1
  TCCR1A: byte absolute $80;  // Timer/Counter1 Control Register A
  TCCR1B: byte absolute $81;  // Timer/Counter1 Control Register B
  TCCR1C: byte absolute $82;  // Timer/Counter1 Control Register C
  TCNT1: word absolute $84;  // Timer/Counter1  Bytes
  TCNT1L: byte absolute $84;  // Timer/Counter1  Bytes
  TCNT1H: byte absolute $85;  // Timer/Counter1  Bytes;
  ICR1: word absolute $86;  // Timer/Counter1 Input Capture Register  Bytes
  ICR1L: byte absolute $86;  // Timer/Counter1 Input Capture Register  Bytes
  ICR1H: byte absolute $87;  // Timer/Counter1 Input Capture Register  Bytes;
  OCR1A: word absolute $88;  // Timer/Counter1 Output Compare Register A  Bytes
  OCR1AL: byte absolute $88;  // Timer/Counter1 Output Compare Register A  Bytes
  OCR1AH: byte absolute $89;  // Timer/Counter1 Output Compare Register A  Bytes;
  OCR1B: word absolute $8A;  // Timer/Counter1 Output Compare Register B  Bytes
  OCR1BL: byte absolute $8A;  // Timer/Counter1 Output Compare Register B  Bytes
  OCR1BH: byte absolute $8B;  // Timer/Counter1 Output Compare Register B  Bytes;
  OCR1C: word absolute $8C;  // Timer/Counter1 Output Compare Register C  Bytes
  OCR1CL: byte absolute $8C;  // Timer/Counter1 Output Compare Register C  Bytes
  OCR1CH: byte absolute $8D;  // Timer/Counter1 Output Compare Register C  Bytes;
  TCCR3A: byte absolute $90;  // Timer/Counter3 Control Register A
  TCCR3B: byte absolute $91;  // Timer/Counter3 Control Register B
  TCCR3C: byte absolute $92;  // Timer/Counter3 Control Register C
  TCNT3: word absolute $94;  // Timer/Counter3  Bytes
  TCNT3L: byte absolute $94;  // Timer/Counter3  Bytes
  TCNT3H: byte absolute $95;  // Timer/Counter3  Bytes;
  ICR3: word absolute $96;  // Timer/Counter3 Input Capture Register  Bytes
  ICR3L: byte absolute $96;  // Timer/Counter3 Input Capture Register  Bytes
  ICR3H: byte absolute $97;  // Timer/Counter3 Input Capture Register  Bytes;
  OCR3A: word absolute $98;  // Timer/Counter3 Output Compare Register A  Bytes
  OCR3AL: byte absolute $98;  // Timer/Counter3 Output Compare Register A  Bytes
  OCR3AH: byte absolute $99;  // Timer/Counter3 Output Compare Register A  Bytes;
  OCR3B: word absolute $9A;  // Timer/Counter3 Output Compare Register B  Bytes
  OCR3BL: byte absolute $9A;  // Timer/Counter3 Output Compare Register B  Bytes
  OCR3BH: byte absolute $9B;  // Timer/Counter3 Output Compare Register B  Bytes;
  OCR3C: word absolute $9C;  // Timer/Counter3 Output Compare Register C  Bytes
  OCR3CL: byte absolute $9C;  // Timer/Counter3 Output Compare Register C  Bytes
  OCR3CH: byte absolute $9D;  // Timer/Counter3 Output Compare Register C  Bytes;
  TCCR4A: byte absolute $A0;  // Timer/Counter4 Control Register A
  TCCR4B: byte absolute $A1;  // Timer/Counter4 Control Register B
  TCCR4C: byte absolute $A2;  // Timer/Counter4 Control Register C
  TCNT4: word absolute $A4;  // Timer/Counter4  Bytes
  TCNT4L: byte absolute $A4;  // Timer/Counter4  Bytes
  TCNT4H: byte absolute $A5;  // Timer/Counter4  Bytes;
  ICR4: word absolute $A6;  // Timer/Counter4 Input Capture Register  Bytes
  ICR4L: byte absolute $A6;  // Timer/Counter4 Input Capture Register  Bytes
  ICR4H: byte absolute $A7;  // Timer/Counter4 Input Capture Register  Bytes;
  OCR4A: word absolute $A8;  // Timer/Counter4 Output Compare Register A  Bytes
  OCR4AL: byte absolute $A8;  // Timer/Counter4 Output Compare Register A  Bytes
  OCR4AH: byte absolute $A9;  // Timer/Counter4 Output Compare Register A  Bytes;
  OCR4B: word absolute $AA;  // Timer/Counter4 Output Compare Register B  Bytes
  OCR4BL: byte absolute $AA;  // Timer/Counter4 Output Compare Register B  Bytes
  OCR4BH: byte absolute $AB;  // Timer/Counter4 Output Compare Register B  Bytes;
  OCR4C: word absolute $AC;  // Timer/Counter4 Output Compare Register C  Bytes
  OCR4CL: byte absolute $AC;  // Timer/Counter4 Output Compare Register C  Bytes
  OCR4CH: byte absolute $AD;  // Timer/Counter4 Output Compare Register C  Bytes;
  TCCR2A: byte absolute $B0;  // Timer/Counter2 Control Register A
  TCCR2B: byte absolute $B1;  // Timer/Counter2 Control Register B
  TCNT2: byte absolute $B2;  // Timer/Counter2
  OCR2A: byte absolute $B3;  // Timer/Counter2 Output Compare Register A
  OCR2B: byte absolute $B4;  // Timer/Counter2 Output Compare Register B
  ASSR: byte absolute $B6;  // Asynchronous Status Register
  TWBR: byte absolute $B8;  // TWI Bit Rate Register
  TWSR: byte absolute $B9;  // TWI Status Register
  TWAR: byte absolute $BA;  // TWI (Slave) Address Register
  TWDR: byte absolute $BB;  // TWI Data Register
  TWCR: byte absolute $BC;  // TWI Control Register
  TWAMR: byte absolute $BD;  // TWI (Slave) Address Mask Register
  IRQ_MASK1: byte absolute $BE;  // Transceiver Interrupt Enable Register 1
  IRQ_STATUS1: byte absolute $BF;  // Transceiver Interrupt Status Register 1
  UCSR0A: byte absolute $C0;  // USART0 MSPIM Control and Status Register A
  UCSR0B: byte absolute $C1;  // USART0 MSPIM Control and Status Register B
  UCSR0C: byte absolute $C2;  // USART0 MSPIM Control and Status Register C
  UBRR0: word absolute $C4;  // USART0 Baud Rate Register  Bytes
  UBRR0L: byte absolute $C4;  // USART0 Baud Rate Register  Bytes
  UBRR0H: byte absolute $C5;  // USART0 Baud Rate Register  Bytes;
  UDR0: byte absolute $C6;  // USART0 I/O Data Register
  UCSR1A: byte absolute $C8;  // USART1 MSPIM Control and Status Register A
  UCSR1B: byte absolute $C9;  // USART1 MSPIM Control and Status Register B
  UCSR1C: byte absolute $CA;  // USART1 MSPIM Control and Status Register C
  UBRR1: word absolute $CC;  // USART1 Baud Rate Register  Bytes
  UBRR1L: byte absolute $CC;  // USART1 Baud Rate Register  Bytes
  UBRR1H: byte absolute $CD;  // USART1 Baud Rate Register  Bytes;
  UDR1: byte absolute $CE;  // USART1 I/O Data Register
  SCRSTRLL: byte absolute $D7;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLH: byte absolute $D8;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRHL: byte absolute $D9;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHH: byte absolute $DA;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCCSR: byte absolute $DB;  // Symbol Counter Compare Source Register
  SCCR0: byte absolute $DC;  // Symbol Counter Control Register 0
  SCCR1: byte absolute $DD;  // Symbol Counter Control Register 1
  SCSR: byte absolute $DE;  // Symbol Counter Status Register
  SCIRQM: byte absolute $DF;  // Symbol Counter Interrupt Mask Register
  SCIRQS: byte absolute $E0;  // Symbol Counter Interrupt Status Register
  SCCNTLL: byte absolute $E1;  // Symbol Counter Register LL-Byte
  SCCNTLH: byte absolute $E2;  // Symbol Counter Register LH-Byte
  SCCNTHL: byte absolute $E3;  // Symbol Counter Register HL-Byte
  SCCNTHH: byte absolute $E4;  // Symbol Counter Register HH-Byte
  SCBTSRLL: byte absolute $E5;  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLH: byte absolute $E6;  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRHL: byte absolute $E7;  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHH: byte absolute $E8;  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCTSRLL: byte absolute $E9;  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLH: byte absolute $EA;  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRHL: byte absolute $EB;  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHH: byte absolute $EC;  // Symbol Counter Frame Timestamp Register HH-Byte
  SCOCR3LL: byte absolute $ED;  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LH: byte absolute $EE;  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3HL: byte absolute $EF;  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HH: byte absolute $F0;  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR2LL: byte absolute $F1;  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LH: byte absolute $F2;  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2HL: byte absolute $F3;  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HH: byte absolute $F4;  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR1LL: byte absolute $F5;  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LH: byte absolute $F6;  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1HL: byte absolute $F7;  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HH: byte absolute $F8;  // Symbol Counter Output Compare Register 1 HH-Byte
  SCTSTRLL: byte absolute $F9;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLH: byte absolute $FA;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRHL: byte absolute $FB;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHH: byte absolute $FC;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  MAFCR0: byte absolute $10C;  // Multiple Address Filter Configuration Register 0
  MAFCR1: byte absolute $10D;  // Multiple Address Filter Configuration Register 1
  MAFSA0L: byte absolute $10E;  // Transceiver MAC Short Address Register for Frame Filter 0 (Low Byte)
  MAFSA0H: byte absolute $10F;  // Transceiver MAC Short Address Register for Frame Filter 0 (High Byte)
  MAFPA0L: byte absolute $110;  // Transceiver Personal Area Network ID Register for Frame Filter 0 (Low Byte)
  MAFPA0H: byte absolute $111;  // Transceiver Personal Area Network ID Register for Frame Filter 0 (High Byte)
  MAFSA1L: byte absolute $112;  // Transceiver MAC Short Address Register for Frame Filter 1 (Low Byte)
  MAFSA1H: byte absolute $113;  // Transceiver MAC Short Address Register for Frame Filter 1 (High Byte)
  MAFPA1L: byte absolute $114;  // Transceiver Personal Area Network ID Register for Frame Filter 1 (Low Byte)
  MAFPA1H: byte absolute $115;  // Transceiver Personal Area Network ID Register for Frame Filter 1 (High Byte)
  MAFSA2L: byte absolute $116;  // Transceiver MAC Short Address Register for Frame Filter 2 (Low Byte)
  MAFSA2H: byte absolute $117;  // Transceiver MAC Short Address Register for Frame Filter 2 (High Byte)
  MAFPA2L: byte absolute $118;  // Transceiver Personal Area Network ID Register for Frame Filter 2 (Low Byte)
  MAFPA2H: byte absolute $119;  // Transceiver Personal Area Network ID Register for Frame Filter 2 (High Byte)
  MAFSA3L: byte absolute $11A;  // Transceiver MAC Short Address Register for Frame Filter 3 (Low Byte)
  MAFSA3H: byte absolute $11B;  // Transceiver MAC Short Address Register for Frame Filter 3 (High Byte)
  MAFPA3L: byte absolute $11C;  // Transceiver Personal Area Network ID Register for Frame Filter 3 (Low Byte)
  MAFPA3H: byte absolute $11D;  // Transceiver Personal Area Network ID Register for Frame Filter 3 (High Byte)
  TCCR5A: byte absolute $120;  // Timer/Counter5 Control Register A
  TCCR5B: byte absolute $121;  // Timer/Counter5 Control Register B
  TCCR5C: byte absolute $122;  // Timer/Counter5 Control Register C
  TCNT5: word absolute $124;  // Timer/Counter5  Bytes
  TCNT5L: byte absolute $124;  // Timer/Counter5  Bytes
  TCNT5H: byte absolute $125;  // Timer/Counter5  Bytes;
  ICR5: word absolute $126;  // Timer/Counter5 Input Capture Register  Bytes
  ICR5L: byte absolute $126;  // Timer/Counter5 Input Capture Register  Bytes
  ICR5H: byte absolute $127;  // Timer/Counter5 Input Capture Register  Bytes;
  OCR5A: word absolute $128;  // Timer/Counter5 Output Compare Register A  Bytes
  OCR5AL: byte absolute $128;  // Timer/Counter5 Output Compare Register A  Bytes
  OCR5AH: byte absolute $129;  // Timer/Counter5 Output Compare Register A  Bytes;
  OCR5B: word absolute $12A;  // Timer/Counter5 Output Compare Register B  Bytes
  OCR5BL: byte absolute $12A;  // Timer/Counter5 Output Compare Register B  Bytes
  OCR5BH: byte absolute $12B;  // Timer/Counter5 Output Compare Register B  Bytes;
  OCR5C: word absolute $12C;  // Timer/Counter5 Output Compare Register C  Bytes
  OCR5CL: byte absolute $12C;  // Timer/Counter5 Output Compare Register C  Bytes
  OCR5CH: byte absolute $12D;  // Timer/Counter5 Output Compare Register C  Bytes;
  LLCR: byte absolute $12F;  // Low Leakage Voltage Regulator Control Register
  LLDRL: byte absolute $130;  // Low Leakage Voltage Regulator Data Register (Low-Byte)
  LLDRH: byte absolute $131;  // Low Leakage Voltage Regulator Data Register (High-Byte)
  DRTRAM3: byte absolute $132;  // Data Retention Configuration Register #3
  DRTRAM2: byte absolute $133;  // Data Retention Configuration Register #2
  DRTRAM1: byte absolute $134;  // Data Retention Configuration Register #1
  DRTRAM0: byte absolute $135;  // Data Retention Configuration Register #0
  DPDS0: byte absolute $136;  // Port Driver Strength Register 0
  DPDS1: byte absolute $137;  // Port Driver Strength Register 1
  PARCR: byte absolute $138;  // Power Amplifier Ramp up/down Control Register
  TRXPR: byte absolute $139;  // Transceiver Pin Register
  AES_CTRL: byte absolute $13C;  // AES Control Register
  AES_STATUS: byte absolute $13D;  // AES Status Register
  AES_STATE: byte absolute $13E;  // AES Plain and Cipher Text Buffer Register
  AES_KEY: byte absolute $13F;  // AES Encryption and Decryption Key Buffer Register
  TRX_STATUS: byte absolute $141;  // Transceiver Status Register
  TRX_STATE: byte absolute $142;  // Transceiver State Control Register
  TRX_CTRL_0: byte absolute $143;  // Reserved
  TRX_CTRL_1: byte absolute $144;  // Transceiver Control Register 1
  PHY_TX_PWR: byte absolute $145;  // Transceiver Transmit Power Control Register
  PHY_RSSI: byte absolute $146;  // Receiver Signal Strength Indicator Register
  PHY_ED_LEVEL: byte absolute $147;  // Transceiver Energy Detection Level Register
  PHY_CC_CCA: byte absolute $148;  // Transceiver Clear Channel Assessment (CCA) Control Register
  CCA_THRES: byte absolute $149;  // Transceiver CCA Threshold Setting Register
  RX_CTRL: byte absolute $14A;  // Transceiver Receive Control Register
  SFD_VALUE: byte absolute $14B;  // Start of Frame Delimiter Value Register
  TRX_CTRL_2: byte absolute $14C;  // Transceiver Control Register 2
  ANT_DIV: byte absolute $14D;  // Antenna Diversity Control Register
  IRQ_MASK: byte absolute $14E;  // Transceiver Interrupt Enable Register
  IRQ_STATUS: byte absolute $14F;  // Transceiver Interrupt Status Register
  VREG_CTRL: byte absolute $150;  // Voltage Regulator Control and Status Register
  BATMON: byte absolute $151;  // Battery Monitor Control and Status Register
  XOSC_CTRL: byte absolute $152;  // Crystal Oscillator Control Register
  CC_CTRL_0: byte absolute $153;  // Channel Control Register 0
  CC_CTRL_1: byte absolute $154;  // Channel Control Register 1
  RX_SYN: byte absolute $155;  // Transceiver Receiver Sensitivity Control Register
  TRX_RPC: byte absolute $156;  // Transceiver Reduced Power Consumption Control
  XAH_CTRL_1: byte absolute $157;  // Transceiver Acknowledgment Frame Control Register 1
  FTN_CTRL: byte absolute $158;  // Transceiver Filter Tuning Control Register
  PLL_CF: byte absolute $15A;  // Transceiver Center Frequency Calibration Control Register
  PLL_DCU: byte absolute $15B;  // Transceiver Delay Cell Calibration Control Register
  PART_NUM: byte absolute $15C;  // Device Identification Register (Part Number)
  VERSION_NUM: byte absolute $15D;  // Device Identification Register (Version Number)
  MAN_ID_0: byte absolute $15E;  // Device Identification Register (Manufacture ID Low Byte)
  MAN_ID_1: byte absolute $15F;  // Device Identification Register (Manufacture ID High Byte)
  SHORT_ADDR_0: byte absolute $160;  // Transceiver MAC Short Address Register (Low Byte)
  SHORT_ADDR_1: byte absolute $161;  // Transceiver MAC Short Address Register (High Byte)
  PAN_ID_0: byte absolute $162;  // Transceiver Personal Area Network ID Register (Low Byte)
  PAN_ID_1: byte absolute $163;  // Transceiver Personal Area Network ID Register (High Byte)
  IEEE_ADDR_0: byte absolute $164;  // Transceiver MAC IEEE Address Register 0
  IEEE_ADDR_1: byte absolute $165;  // Transceiver MAC IEEE Address Register 1
  IEEE_ADDR_2: byte absolute $166;  // Transceiver MAC IEEE Address Register 2
  IEEE_ADDR_3: byte absolute $167;  // Transceiver MAC IEEE Address Register 3
  IEEE_ADDR_4: byte absolute $168;  // Transceiver MAC IEEE Address Register 4
  IEEE_ADDR_5: byte absolute $169;  // Transceiver MAC IEEE Address Register 5
  IEEE_ADDR_6: byte absolute $16A;  // Transceiver MAC IEEE Address Register 6
  IEEE_ADDR_7: byte absolute $16B;  // Transceiver MAC IEEE Address Register 7
  XAH_CTRL_0: byte absolute $16C;  // Transceiver Extended Operating Mode Control Register
  CSMA_SEED_0: byte absolute $16D;  // Transceiver CSMA-CA Random Number Generator Seed Register
  CSMA_SEED_1: byte absolute $16E;  // Transceiver Acknowledgment Frame Control Register 2
  CSMA_BE: byte absolute $16F;  // Transceiver CSMA-CA Back-off Exponent Control Register
  TST_CTRL_DIGI: byte absolute $176;  // Transceiver Digital Test Control Register
  TST_RX_LENGTH: byte absolute $17B;  // Transceiver Received Frame Length Register
  TRXFBST: byte absolute $180;  // Start of frame buffer
  TRXFBEND: byte absolute $1FF;  // End of frame buffer

const
  // Port A Data Register
  PA0 = $00;  
  PA1 = $01;  
  PA2 = $02;  
  PA3 = $03;  
  PA4 = $04;  
  PA5 = $05;  
  PA6 = $06;  
  PA7 = $07;  
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
  PC7 = $07;  
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
  PE4 = $04;  
  PE5 = $05;  
  PE6 = $06;  
  PE7 = $07;  
  // Port F Data Register
  PF0 = $00;  
  PF1 = $01;  
  PF2 = $02;  
  PF3 = $03;  
  PF4 = $04;  
  PF5 = $05;  
  PF6 = $06;  
  PF7 = $07;  
  // Port G Data Register
  PG0 = $00;  
  PG1 = $01;  
  PG2 = $02;  
  PG3 = $03;  
  PG4 = $04;  
  PG5 = $05;  
  PG6 = $06;  
  PG7 = $07;  
  // Timer/Counter0 Interrupt Flag Register
  TOV0 = $00;  
  OCF0A = $01;  
  OCF0B = $02;  
  // Timer/Counter1 Interrupt Flag Register
  TOV1 = $00;  
  OCF1A = $01;  
  OCF1B = $02;  
  OCF1C = $03;  
  ICF1 = $05;  
  // Timer/Counter Interrupt Flag Register
  TOV2 = $00;  
  OCF2A = $01;  
  OCF2B = $02;  
  // Timer/Counter3 Interrupt Flag Register
  TOV3 = $00;  
  OCF3A = $01;  
  OCF3B = $02;  
  OCF3C = $03;  
  ICF3 = $05;  
  // Timer/Counter4 Interrupt Flag Register
  TOV4 = $00;  
  OCF4A = $01;  
  OCF4B = $02;  
  OCF4C = $03;  
  ICF4 = $05;  
  // Timer/Counter5 Interrupt Flag Register
  TOV5 = $00;  
  OCF5A = $01;  
  OCF5B = $02;  
  OCF5C = $03;  
  ICF5 = $05;  
  // Pin Change Interrupt Flag Register
  PCIF0 = $00;  // Pin Change Interrupt Flags
  PCIF1 = $01;  // Pin Change Interrupt Flags
  PCIF2 = $02;  // Pin Change Interrupt Flags
  // External Interrupt Flag Register
  INTF0 = $00;  // External Interrupt Flag
  INTF1 = $01;  // External Interrupt Flag
  INTF2 = $02;  // External Interrupt Flag
  INTF3 = $03;  // External Interrupt Flag
  INTF4 = $04;  // External Interrupt Flag
  INTF5 = $05;  // External Interrupt Flag
  INTF6 = $06;  // External Interrupt Flag
  INTF7 = $07;  // External Interrupt Flag
  // External Interrupt Mask Register
  INT0 = $00;  // External Interrupt Request Enable
  INT1 = $01;  // External Interrupt Request Enable
  INT2 = $02;  // External Interrupt Request Enable
  INT3 = $03;  // External Interrupt Request Enable
  INT4 = $04;  // External Interrupt Request Enable
  INT5 = $05;  // External Interrupt Request Enable
  INT6 = $06;  // External Interrupt Request Enable
  INT7 = $07;  // External Interrupt Request Enable
  // General Purpose IO Register 0
  GPIOR00 = $00;  
  GPIOR01 = $01;  
  GPIOR02 = $02;  
  GPIOR03 = $03;  
  GPIOR04 = $04;  
  GPIOR05 = $05;  
  GPIOR06 = $06;  
  GPIOR07 = $07;  
  // EEPROM Control Register
  EERE = $00;  
  EEPE = $01;  
  EEMPE = $02;  
  EERIE = $03;  
  EEPM0 = $04;  // EEPROM Programming Mode
  EEPM1 = $05;  // EEPROM Programming Mode
  // General Timer Counter Control register
  PSRSYNC = $00;  
  PSRASY = $01;  
  TSM = $07;  
  // Timer/Counter0 Control Register A
  WGM00 = $00;  // Waveform Generation Mode
  WGM01 = $01;  // Waveform Generation Mode
  COM0B0 = $04;  // Compare Match Output B Mode
  COM0B1 = $05;  // Compare Match Output B Mode
  COM0A0 = $06;  // Compare Match Output A Mode
  COM0A1 = $07;  // Compare Match Output A Mode
  // Timer/Counter0 Control Register B
  CS00 = $00;  // Clock Select
  CS01 = $01;  // Clock Select
  CS02 = $02;  // Clock Select
  WGM02 = $03;  
  FOC0B = $06;  
  FOC0A = $07;  
  // General Purpose I/O Register 2
  GPIOR20 = $00;  // General Purpose I/O Register 2 Value
  GPIOR21 = $01;  // General Purpose I/O Register 2 Value
  GPIOR22 = $02;  // General Purpose I/O Register 2 Value
  GPIOR23 = $03;  // General Purpose I/O Register 2 Value
  GPIOR24 = $04;  // General Purpose I/O Register 2 Value
  GPIOR25 = $05;  // General Purpose I/O Register 2 Value
  GPIOR26 = $06;  // General Purpose I/O Register 2 Value
  GPIOR27 = $07;  // General Purpose I/O Register 2 Value
  // SPI Control Register
  SPR0 = $00;  // SPI Clock Rate Select 1 and 0
  SPR1 = $01;  // SPI Clock Rate Select 1 and 0
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
  // Analog Comparator Control And Status Register
  ACIS0 = $00;  // Analog Comparator Interrupt Mode Select
  ACIS1 = $01;  // Analog Comparator Interrupt Mode Select
  ACIC = $02;  
  ACIE = $03;  
  ACI = $04;  
  ACO = $05;  
  ACBG = $06;  
  ACD = $07;  
  // On-Chip Debug Register
  OCDR0 = $00;  // On-Chip Debug Register Data
  OCDR1 = $01;  // On-Chip Debug Register Data
  OCDR2 = $02;  // On-Chip Debug Register Data
  OCDR3 = $03;  // On-Chip Debug Register Data
  OCDR4 = $04;  // On-Chip Debug Register Data
  OCDR5 = $05;  // On-Chip Debug Register Data
  OCDR6 = $06;  // On-Chip Debug Register Data
  OCDR7 = $07;  // On-Chip Debug Register Data
  // Sleep Mode Control Register
  SE = $00;  
  SM0 = $01;  // Sleep Mode Select bits
  SM1 = $02;  // Sleep Mode Select bits
  SM2 = $03;  // Sleep Mode Select bits
  // MCU Status Register
  PORF = $00;  
  EXTRF = $01;  
  BORF = $02;  
  WDRF = $03;  
  JTRF = $04;  
  // MCU Control Register
  IVCE = $00;  
  IVSEL = $01;  
  PUD = $04;  
  JTD = $07;  
  // Store Program Memory Control Register
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
  // Power Reduction Register 2
  PRRAM0 = $00;  
  PRRAM1 = $01;  
  PRRAM2 = $02;  
  PRRAM3 = $03;  
  // Power Reduction Register0
  PRADC = $00;  
  PRUSART0 = $01;  
  PRSPI = $02;  
  PRTIM1 = $03;  
  PRPGA = $04;  
  PRTIM0 = $05;  
  PRTIM2 = $06;  
  PRTWI = $07;  
  // Power Reduction Register 1
  PRUSART1 = $00;  
  PRTIM3 = $03;  
  PRTIM4 = $04;  
  PRTIM5 = $05;  
  PRTRX24 = $06;  
  // Oscillator Calibration Value
  CAL0 = $00;  // Oscillator Calibration Tuning Value
  CAL1 = $01;  // Oscillator Calibration Tuning Value
  CAL2 = $02;  // Oscillator Calibration Tuning Value
  CAL3 = $03;  // Oscillator Calibration Tuning Value
  CAL4 = $04;  // Oscillator Calibration Tuning Value
  CAL5 = $05;  // Oscillator Calibration Tuning Value
  CAL6 = $06;  // Oscillator Calibration Tuning Value
  CAL7 = $07;  // Oscillator Calibration Tuning Value
  OSCCAL0 = $00;  // Oscillator Calibration 
  OSCCAL1 = $01;  // Oscillator Calibration 
  OSCCAL2 = $02;  // Oscillator Calibration 
  OSCCAL3 = $03;  // Oscillator Calibration 
  OSCCAL4 = $04;  // Oscillator Calibration 
  OSCCAL5 = $05;  // Oscillator Calibration 
  OSCCAL6 = $06;  // Oscillator Calibration 
  OSCCAL7 = $07;  // Oscillator Calibration 
  // Reference Voltage Calibration Register
  BGCAL0 = $00;  // Coarse Calibration Bits
  BGCAL1 = $01;  // Coarse Calibration Bits
  BGCAL2 = $02;  // Coarse Calibration Bits
  BGCAL_FINE0 = $03;  // Fine Calibration Bits
  BGCAL_FINE1 = $04;  // Fine Calibration Bits
  BGCAL_FINE2 = $05;  // Fine Calibration Bits
  BGCAL_FINE3 = $06;  // Fine Calibration Bits
  // Pin Change Interrupt Control Register
  PCIE0 = $00;  // Pin Change Interrupt Enables
  PCIE1 = $01;  // Pin Change Interrupt Enables
  PCIE2 = $02;  // Pin Change Interrupt Enables
  // External Interrupt Control Register A
  ISC00 = $00;  // External Interrupt 0 Sense Control Bit
  ISC01 = $01;  // External Interrupt 0 Sense Control Bit
  ISC10 = $02;  // External Interrupt 1 Sense Control Bit
  ISC11 = $03;  // External Interrupt 1 Sense Control Bit
  ISC20 = $04;  // External Interrupt 2 Sense Control Bit
  ISC21 = $05;  // External Interrupt 2 Sense Control Bit
  ISC30 = $06;  // External Interrupt 3 Sense Control Bit
  ISC31 = $07;  // External Interrupt 3 Sense Control Bit
  // External Interrupt Control Register B
  ISC40 = $00;  // External Interrupt 4 Sense Control Bit
  ISC41 = $01;  // External Interrupt 4 Sense Control Bit
  ISC50 = $02;  // External Interrupt 5 Sense Control Bit
  ISC51 = $03;  // External Interrupt 5 Sense Control Bit
  ISC60 = $04;  // External Interrupt 6 Sense Control Bit
  ISC61 = $05;  // External Interrupt 6 Sense Control Bit
  ISC70 = $06;  // External Interrupt 7 Sense Control Bit
  ISC71 = $07;  // External Interrupt 7 Sense Control Bit
  // Pin Change Mask Register 2
  PCINT16 = $00;  // Pin Change Enable Mask
  PCINT17 = $01;  // Pin Change Enable Mask
  PCINT18 = $02;  // Pin Change Enable Mask
  PCINT19 = $03;  // Pin Change Enable Mask
  PCINT20 = $04;  // Pin Change Enable Mask
  PCINT21 = $05;  // Pin Change Enable Mask
  PCINT22 = $06;  // Pin Change Enable Mask
  PCINT23 = $07;  // Pin Change Enable Mask
  // Timer/Counter0 Interrupt Mask Register
  TOIE0 = $00;  
  OCIE0A = $01;  
  OCIE0B = $02;  
  // Timer/Counter1 Interrupt Mask Register
  TOIE1 = $00;  
  OCIE1A = $01;  
  OCIE1B = $02;  
  OCIE1C = $03;  
  ICIE1 = $05;  
  // Timer/Counter Interrupt Mask register
  TOIE2 = $00;  
  OCIE2A = $01;  
  OCIE2B = $02;  
  // Timer/Counter3 Interrupt Mask Register
  TOIE3 = $00;  
  OCIE3A = $01;  
  OCIE3B = $02;  
  OCIE3C = $03;  
  ICIE3 = $05;  
  // Timer/Counter4 Interrupt Mask Register
  TOIE4 = $00;  
  OCIE4A = $01;  
  OCIE4B = $02;  
  OCIE4C = $03;  
  ICIE4 = $05;  
  // Timer/Counter5 Interrupt Mask Register
  TOIE5 = $00;  
  OCIE5A = $01;  
  OCIE5B = $02;  
  OCIE5C = $03;  
  ICIE5 = $05;  
  // Flash Extended-Mode Control-Register
  AEAM0 = $04;  // Address for Extended Address Mode of Extra Rows
  AEAM1 = $05;  // Address for Extended Address Mode of Extra Rows
  ENEAM = $06;  
  // The ADC Control and Status Register C
  ADSUT0 = $00;  // ADC Start-up Time
  ADSUT1 = $01;  // ADC Start-up Time
  ADSUT2 = $02;  // ADC Start-up Time
  ADSUT3 = $03;  // ADC Start-up Time
  ADSUT4 = $04;  // ADC Start-up Time
  ADTHT0 = $06;  // ADC Track-and-Hold Time
  ADTHT1 = $07;  // ADC Track-and-Hold Time
  // The ADC Control and Status Register A
  ADPS0 = $00;  // ADC  Prescaler Select Bits
  ADPS1 = $01;  // ADC  Prescaler Select Bits
  ADPS2 = $02;  // ADC  Prescaler Select Bits
  ADIE = $03;  
  ADIF = $04;  
  ADATE = $05;  
  ADSC = $06;  
  ADEN = $07;  
  // The ADC Control and Status Register B
  ADTS0 = $00;  // ADC Auto Trigger Source
  ADTS1 = $01;  // ADC Auto Trigger Source
  ADTS2 = $02;  // ADC Auto Trigger Source
  MUX5 = $03;  
  ACCH = $04;  
  REFOK = $05;  
  ACME = $06;  
  AVDDOK = $07;  
  // The ADC Multiplexer Selection Register
  MUX0 = $00;  // Analog Channel and Gain Selection Bits
  MUX1 = $01;  // Analog Channel and Gain Selection Bits
  MUX2 = $02;  // Analog Channel and Gain Selection Bits
  MUX3 = $03;  // Analog Channel and Gain Selection Bits
  MUX4 = $04;  // Analog Channel and Gain Selection Bits
  ADLAR = $05;  
  REFS0 = $06;  // Reference Selection Bits
  REFS1 = $07;  // Reference Selection Bits
  // Digital Input Disable Register 2
  ADC8D = $00;  
  ADC9D = $01;  
  ADC10D = $02;  
  ADC11D = $03;  
  ADC12D = $04;  
  ADC13D = $05;  
  ADC14D = $06;  
  ADC15D = $07;  
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
  AIN0D = $00;  
  AIN1D = $01;  
  // Timer/Counter1 Control Register A
  WGM10 = $00;  // Waveform Generation Mode
  WGM11 = $01;  // Waveform Generation Mode
  COM1C0 = $02;  // Compare Output Mode for Channel C
  COM1C1 = $03;  // Compare Output Mode for Channel C
  COM1B0 = $04;  // Compare Output Mode for Channel B
  COM1B1 = $05;  // Compare Output Mode for Channel B
  COM1A0 = $06;  // Compare Output Mode for Channel A
  COM1A1 = $07;  // Compare Output Mode for Channel A
  // Timer/Counter1 Control Register B
  CS10 = $00;  // Clock Select
  CS11 = $01;  // Clock Select
  CS12 = $02;  // Clock Select
  ICES1 = $06;  
  ICNC1 = $07;  
  // Timer/Counter1 Control Register C
  FOC1C = $05;  
  FOC1B = $06;  
  FOC1A = $07;  
  // Timer/Counter3 Control Register A
  WGM30 = $00;  // Waveform Generation Mode
  WGM31 = $01;  // Waveform Generation Mode
  COM3C0 = $02;  // Compare Output Mode for Channel C
  COM3C1 = $03;  // Compare Output Mode for Channel C
  COM3B0 = $04;  // Compare Output Mode for Channel B
  COM3B1 = $05;  // Compare Output Mode for Channel B
  COM3A0 = $06;  // Compare Output Mode for Channel A
  COM3A1 = $07;  // Compare Output Mode for Channel A
  // Timer/Counter3 Control Register B
  CS30 = $00;  // Clock Select
  CS31 = $01;  // Clock Select
  CS32 = $02;  // Clock Select
  ICES3 = $06;  
  ICNC3 = $07;  
  // Timer/Counter3 Control Register C
  FOC3C = $05;  
  FOC3B = $06;  
  FOC3A = $07;  
  // Timer/Counter4 Control Register A
  WGM40 = $00;  // Waveform Generation Mode
  WGM41 = $01;  // Waveform Generation Mode
  COM4C0 = $02;  // Compare Output Mode for Channel C
  COM4C1 = $03;  // Compare Output Mode for Channel C
  COM4B0 = $04;  // Compare Output Mode for Channel B
  COM4B1 = $05;  // Compare Output Mode for Channel B
  COM4A0 = $06;  // Compare Output Mode for Channel A
  COM4A1 = $07;  // Compare Output Mode for Channel A
  // Timer/Counter4 Control Register B
  CS40 = $00;  // Clock Select
  CS41 = $01;  // Clock Select
  CS42 = $02;  // Clock Select
  ICES4 = $06;  
  ICNC4 = $07;  
  // Timer/Counter4 Control Register C
  FOC4C = $05;  
  FOC4B = $06;  
  FOC4A = $07;  
  // Timer/Counter2 Control Register A
  WGM20 = $00;  // Waveform Generation Mode
  WGM21 = $01;  // Waveform Generation Mode
  COM2B0 = $04;  // Compare Match Output B Mode
  COM2B1 = $05;  // Compare Match Output B Mode
  COM2A0 = $06;  // Compare Match Output A Mode
  COM2A1 = $07;  // Compare Match Output A Mode
  // Timer/Counter2 Control Register B
  CS20 = $00;  // Clock Select
  CS21 = $01;  // Clock Select
  CS22 = $02;  // Clock Select
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
  EXCLKAMR = $07;  
  // TWI Status Register
  TWPS0 = $00;  // TWI Prescaler Bits
  TWPS1 = $01;  // TWI Prescaler Bits
  TWS3 = $03;  // TWI Status
  TWS4 = $04;  // TWI Status
  TWS5 = $05;  // TWI Status
  TWS6 = $06;  // TWI Status
  TWS7 = $07;  // TWI Status
  // TWI (Slave) Address Register
  TWGCE = $00;  
  TWA0 = $01;  // TWI (Slave) Address
  TWA1 = $02;  // TWI (Slave) Address
  TWA2 = $03;  // TWI (Slave) Address
  TWA3 = $04;  // TWI (Slave) Address
  TWA4 = $05;  // TWI (Slave) Address
  TWA5 = $06;  // TWI (Slave) Address
  TWA6 = $07;  // TWI (Slave) Address
  // TWI Control Register
  TWIE = $00;  
  TWEN = $02;  
  TWWC = $03;  
  TWSTO = $04;  
  TWSTA = $05;  
  TWEA = $06;  
  TWINT = $07;  
  // TWI (Slave) Address Mask Register
  TWAM0 = $01;  // TWI Address Mask
  TWAM1 = $02;  // TWI Address Mask
  TWAM2 = $03;  // TWI Address Mask
  TWAM3 = $04;  // TWI Address Mask
  TWAM4 = $05;  // TWI Address Mask
  TWAM5 = $06;  // TWI Address Mask
  TWAM6 = $07;  // TWI Address Mask
  // Transceiver Interrupt Enable Register 1
  TX_START_EN = $00;  
  MAF_0_AMI_EN = $01;  
  MAF_1_AMI_EN = $02;  
  MAF_2_AMI_EN = $03;  
  MAF_3_AMI_EN = $04;  
  // Transceiver Interrupt Status Register 1
  TX_START = $00;  
  MAF_0_AMI = $01;  
  MAF_1_AMI = $02;  
  MAF_2_AMI = $03;  
  MAF_3_AMI = $04;  
  // USART0 MSPIM Control and Status Register A
  MPCM0 = $00;  
  U2X0 = $01;  
  UPE0 = $02;  
  DOR0 = $03;  
  FE0 = $04;  
  UDRE0 = $05;  
  TXC0 = $06;  
  RXC0 = $07;  
  // USART0 MSPIM Control and Status Register B
  TXB80 = $00;  
  RXB80 = $01;  
  UCSZ02 = $02;  
  TXEN0 = $03;  
  RXEN0 = $04;  
  UDRIE0 = $05;  
  TXCIE0 = $06;  
  RXCIE0 = $07;  
  // USART0 MSPIM Control and Status Register C
  UCPOL0 = $00;  
  UCPHA0 = $01;  
  UDORD0 = $02;  
  UCSZ00 = $01;  // Character Size
  UCSZ01 = $02;  // Character Size
  USBS0 = $03;  
  UPM00 = $04;  // Parity Mode
  UPM01 = $05;  // Parity Mode
  UMSEL00 = $06;  // USART Mode Select
  UMSEL01 = $07;  // USART Mode Select
  // USART1 MSPIM Control and Status Register A
  MPCM1 = $00;  
  U2X1 = $01;  
  UPE1 = $02;  
  DOR1 = $03;  
  FE1 = $04;  
  UDRE1 = $05;  
  TXC1 = $06;  
  RXC1 = $07;  
  // USART1 MSPIM Control and Status Register B
  TXB81 = $00;  
  RXB81 = $01;  
  UCSZ12 = $02;  
  TXEN1 = $03;  
  RXEN1 = $04;  
  UDRIE1 = $05;  
  TXCIE1 = $06;  
  RXCIE1 = $07;  
  // USART1 MSPIM Control and Status Register C
  UCPOL1 = $00;  
  UCPHA1 = $01;  
  UDORD1 = $02;  
  UCSZ10 = $01;  // Character Size
  UCSZ11 = $02;  // Character Size
  USBS1 = $03;  
  UPM10 = $04;  // Parity Mode
  UPM11 = $05;  // Parity Mode
  UMSEL10 = $06;  // USART Mode Select
  UMSEL11 = $07;  // USART Mode Select
  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLL0 = $00;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLL1 = $01;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLL2 = $02;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLL3 = $03;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLL4 = $04;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLL5 = $05;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLL6 = $06;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  SCRSTRLL7 = $07;  // Symbol Counter Received Frame Timestamp Register LL-Byte
  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRLH0 = $00;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRLH1 = $01;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRLH2 = $02;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRLH3 = $03;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRLH4 = $04;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRLH5 = $05;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRLH6 = $06;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  SCRSTRLH7 = $07;  // Symbol Counter Received Frame Timestamp Register LH-Byte
  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHL0 = $00;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHL1 = $01;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHL2 = $02;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHL3 = $03;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHL4 = $04;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHL5 = $05;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHL6 = $06;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  SCRSTRHL7 = $07;  // Symbol Counter Received Frame Timestamp Register HL-Byte
  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCRSTRHH0 = $00;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCRSTRHH1 = $01;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCRSTRHH2 = $02;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCRSTRHH3 = $03;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCRSTRHH4 = $04;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCRSTRHH5 = $05;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCRSTRHH6 = $06;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  SCRSTRHH7 = $07;  // Symbol Counter Received Frame Timestamp Register HH-Byte
  // Symbol Counter Compare Source Register
  SCCS10 = $00;  // Symbol Counter Compare Source select register for Compare Units
  SCCS11 = $01;  // Symbol Counter Compare Source select register for Compare Units
  SCCS20 = $02;  // Symbol Counter Compare Source select register for Compare Unit 2
  SCCS21 = $03;  // Symbol Counter Compare Source select register for Compare Unit 2
  SCCS30 = $04;  // Symbol Counter Compare Source select register for Compare Unit 3
  SCCS31 = $05;  // Symbol Counter Compare Source select register for Compare Unit 3
  // Symbol Counter Control Register 0
  SCCMP1 = $00;  // Symbol Counter Compare Unit 3 Mode select
  SCCMP2 = $01;  // Symbol Counter Compare Unit 3 Mode select
  SCCMP3 = $02;  // Symbol Counter Compare Unit 3 Mode select
  SCTSE = $03;  
  SCCKSEL = $04;  
  SCEN = $05;  
  SCMBTS = $06;  
  SCRES = $07;  
  // Symbol Counter Control Register 1
  SCENBO = $00;  
  SCEECLK = $01;  
  SCCKDIV0 = $02;  // Clock divider for synchronous clock source (16MHz Transceiver Clock)
  SCCKDIV1 = $03;  // Clock divider for synchronous clock source (16MHz Transceiver Clock)
  SCCKDIV2 = $04;  // Clock divider for synchronous clock source (16MHz Transceiver Clock)
  SCBTSM = $05;  
  Res5 = $06;  // Reserved Bit
  Res6 = $07;  // Reserved Bit
  // Symbol Counter Status Register
  SCBSY = $00;  
  // Symbol Counter Interrupt Mask Register
  IRQMCP1 = $00;  // Symbol Counter Compare Match 3 IRQ enable
  IRQMCP2 = $01;  // Symbol Counter Compare Match 3 IRQ enable
  IRQMCP3 = $02;  // Symbol Counter Compare Match 3 IRQ enable
  IRQMOF = $03;  
  IRQMBO = $04;  
  // Symbol Counter Interrupt Status Register
  IRQSCP1 = $00;  // Compare Unit 3 Compare Match IRQ
  IRQSCP2 = $01;  // Compare Unit 3 Compare Match IRQ
  IRQSCP3 = $02;  // Compare Unit 3 Compare Match IRQ
  IRQSOF = $03;  
  IRQSBO = $04;  
  // Symbol Counter Register LL-Byte
  SCCNTLL0 = $00;  // Symbol Counter Register LL-Byte
  SCCNTLL1 = $01;  // Symbol Counter Register LL-Byte
  SCCNTLL2 = $02;  // Symbol Counter Register LL-Byte
  SCCNTLL3 = $03;  // Symbol Counter Register LL-Byte
  SCCNTLL4 = $04;  // Symbol Counter Register LL-Byte
  SCCNTLL5 = $05;  // Symbol Counter Register LL-Byte
  SCCNTLL6 = $06;  // Symbol Counter Register LL-Byte
  SCCNTLL7 = $07;  // Symbol Counter Register LL-Byte
  // Symbol Counter Register LH-Byte
  SCCNTLH0 = $00;  // Symbol Counter Register LH-Byte
  SCCNTLH1 = $01;  // Symbol Counter Register LH-Byte
  SCCNTLH2 = $02;  // Symbol Counter Register LH-Byte
  SCCNTLH3 = $03;  // Symbol Counter Register LH-Byte
  SCCNTLH4 = $04;  // Symbol Counter Register LH-Byte
  SCCNTLH5 = $05;  // Symbol Counter Register LH-Byte
  SCCNTLH6 = $06;  // Symbol Counter Register LH-Byte
  SCCNTLH7 = $07;  // Symbol Counter Register LH-Byte
  // Symbol Counter Register HL-Byte
  SCCNTHL0 = $00;  // Symbol Counter Register HL-Byte
  SCCNTHL1 = $01;  // Symbol Counter Register HL-Byte
  SCCNTHL2 = $02;  // Symbol Counter Register HL-Byte
  SCCNTHL3 = $03;  // Symbol Counter Register HL-Byte
  SCCNTHL4 = $04;  // Symbol Counter Register HL-Byte
  SCCNTHL5 = $05;  // Symbol Counter Register HL-Byte
  SCCNTHL6 = $06;  // Symbol Counter Register HL-Byte
  SCCNTHL7 = $07;  // Symbol Counter Register HL-Byte
  // Symbol Counter Register HH-Byte
  SCCNTHH0 = $00;  // Symbol Counter Register HH-Byte
  SCCNTHH1 = $01;  // Symbol Counter Register HH-Byte
  SCCNTHH2 = $02;  // Symbol Counter Register HH-Byte
  SCCNTHH3 = $03;  // Symbol Counter Register HH-Byte
  SCCNTHH4 = $04;  // Symbol Counter Register HH-Byte
  SCCNTHH5 = $05;  // Symbol Counter Register HH-Byte
  SCCNTHH6 = $06;  // Symbol Counter Register HH-Byte
  SCCNTHH7 = $07;  // Symbol Counter Register HH-Byte
  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLL0 = $00;  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLL1 = $01;  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLL2 = $02;  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLL3 = $03;  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLL4 = $04;  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLL5 = $05;  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLL6 = $06;  // Symbol Counter Beacon Timestamp Register LL-Byte
  SCBTSRLL7 = $07;  // Symbol Counter Beacon Timestamp Register LL-Byte
  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRLH0 = $00;  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRLH1 = $01;  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRLH2 = $02;  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRLH3 = $03;  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRLH4 = $04;  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRLH5 = $05;  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRLH6 = $06;  // Symbol Counter Beacon Timestamp Register LH-Byte
  SCBTSRLH7 = $07;  // Symbol Counter Beacon Timestamp Register LH-Byte
  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHL0 = $00;  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHL1 = $01;  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHL2 = $02;  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHL3 = $03;  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHL4 = $04;  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHL5 = $05;  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHL6 = $06;  // Symbol Counter Beacon Timestamp Register HL-Byte
  SCBTSRHL7 = $07;  // Symbol Counter Beacon Timestamp Register HL-Byte
  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCBTSRHH0 = $00;  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCBTSRHH1 = $01;  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCBTSRHH2 = $02;  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCBTSRHH3 = $03;  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCBTSRHH4 = $04;  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCBTSRHH5 = $05;  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCBTSRHH6 = $06;  // Symbol Counter Beacon Timestamp Register HH-Byte
  SCBTSRHH7 = $07;  // Symbol Counter Beacon Timestamp Register HH-Byte
  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLL0 = $00;  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLL1 = $01;  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLL2 = $02;  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLL3 = $03;  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLL4 = $04;  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLL5 = $05;  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLL6 = $06;  // Symbol Counter Frame Timestamp Register LL-Byte
  SCTSRLL7 = $07;  // Symbol Counter Frame Timestamp Register LL-Byte
  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRLH0 = $00;  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRLH1 = $01;  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRLH2 = $02;  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRLH3 = $03;  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRLH4 = $04;  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRLH5 = $05;  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRLH6 = $06;  // Symbol Counter Frame Timestamp Register LH-Byte
  SCTSRLH7 = $07;  // Symbol Counter Frame Timestamp Register LH-Byte
  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHL0 = $00;  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHL1 = $01;  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHL2 = $02;  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHL3 = $03;  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHL4 = $04;  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHL5 = $05;  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHL6 = $06;  // Symbol Counter Frame Timestamp Register HL-Byte
  SCTSRHL7 = $07;  // Symbol Counter Frame Timestamp Register HL-Byte
  // Symbol Counter Frame Timestamp Register HH-Byte
  SCTSRHH0 = $00;  // Symbol Counter Frame Timestamp Register HH-Byte
  SCTSRHH1 = $01;  // Symbol Counter Frame Timestamp Register HH-Byte
  SCTSRHH2 = $02;  // Symbol Counter Frame Timestamp Register HH-Byte
  SCTSRHH3 = $03;  // Symbol Counter Frame Timestamp Register HH-Byte
  SCTSRHH4 = $04;  // Symbol Counter Frame Timestamp Register HH-Byte
  SCTSRHH5 = $05;  // Symbol Counter Frame Timestamp Register HH-Byte
  SCTSRHH6 = $06;  // Symbol Counter Frame Timestamp Register HH-Byte
  SCTSRHH7 = $07;  // Symbol Counter Frame Timestamp Register HH-Byte
  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LL0 = $00;  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LL1 = $01;  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LL2 = $02;  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LL3 = $03;  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LL4 = $04;  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LL5 = $05;  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LL6 = $06;  // Symbol Counter Output Compare Register 3 LL-Byte
  SCOCR3LL7 = $07;  // Symbol Counter Output Compare Register 3 LL-Byte
  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3LH0 = $00;  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3LH1 = $01;  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3LH2 = $02;  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3LH3 = $03;  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3LH4 = $04;  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3LH5 = $05;  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3LH6 = $06;  // Symbol Counter Output Compare Register 3 LH-Byte
  SCOCR3LH7 = $07;  // Symbol Counter Output Compare Register 3 LH-Byte
  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HL0 = $00;  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HL1 = $01;  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HL2 = $02;  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HL3 = $03;  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HL4 = $04;  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HL5 = $05;  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HL6 = $06;  // Symbol Counter Output Compare Register 3 HL-Byte
  SCOCR3HL7 = $07;  // Symbol Counter Output Compare Register 3 HL-Byte
  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR3HH0 = $00;  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR3HH1 = $01;  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR3HH2 = $02;  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR3HH3 = $03;  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR3HH4 = $04;  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR3HH5 = $05;  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR3HH6 = $06;  // Symbol Counter Output Compare Register 3 HH-Byte
  SCOCR3HH7 = $07;  // Symbol Counter Output Compare Register 3 HH-Byte
  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LL0 = $00;  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LL1 = $01;  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LL2 = $02;  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LL3 = $03;  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LL4 = $04;  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LL5 = $05;  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LL6 = $06;  // Symbol Counter Output Compare Register 2 LL-Byte
  SCOCR2LL7 = $07;  // Symbol Counter Output Compare Register 2 LL-Byte
  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2LH0 = $00;  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2LH1 = $01;  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2LH2 = $02;  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2LH3 = $03;  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2LH4 = $04;  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2LH5 = $05;  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2LH6 = $06;  // Symbol Counter Output Compare Register 2 LH-Byte
  SCOCR2LH7 = $07;  // Symbol Counter Output Compare Register 2 LH-Byte
  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HL0 = $00;  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HL1 = $01;  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HL2 = $02;  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HL3 = $03;  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HL4 = $04;  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HL5 = $05;  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HL6 = $06;  // Symbol Counter Output Compare Register 2 HL-Byte
  SCOCR2HL7 = $07;  // Symbol Counter Output Compare Register 2 HL-Byte
  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR2HH0 = $00;  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR2HH1 = $01;  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR2HH2 = $02;  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR2HH3 = $03;  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR2HH4 = $04;  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR2HH5 = $05;  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR2HH6 = $06;  // Symbol Counter Output Compare Register 2 HH-Byte
  SCOCR2HH7 = $07;  // Symbol Counter Output Compare Register 2 HH-Byte
  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LL0 = $00;  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LL1 = $01;  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LL2 = $02;  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LL3 = $03;  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LL4 = $04;  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LL5 = $05;  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LL6 = $06;  // Symbol Counter Output Compare Register 1 LL-Byte
  SCOCR1LL7 = $07;  // Symbol Counter Output Compare Register 1 LL-Byte
  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1LH0 = $00;  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1LH1 = $01;  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1LH2 = $02;  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1LH3 = $03;  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1LH4 = $04;  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1LH5 = $05;  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1LH6 = $06;  // Symbol Counter Output Compare Register 1 LH-Byte
  SCOCR1LH7 = $07;  // Symbol Counter Output Compare Register 1 LH-Byte
  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HL0 = $00;  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HL1 = $01;  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HL2 = $02;  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HL3 = $03;  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HL4 = $04;  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HL5 = $05;  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HL6 = $06;  // Symbol Counter Output Compare Register 1 HL-Byte
  SCOCR1HL7 = $07;  // Symbol Counter Output Compare Register 1 HL-Byte
  // Symbol Counter Output Compare Register 1 HH-Byte
  SCOCR1HH0 = $00;  // Symbol Counter Output Compare Register 1 HH-Byte
  SCOCR1HH1 = $01;  // Symbol Counter Output Compare Register 1 HH-Byte
  SCOCR1HH2 = $02;  // Symbol Counter Output Compare Register 1 HH-Byte
  SCOCR1HH3 = $03;  // Symbol Counter Output Compare Register 1 HH-Byte
  SCOCR1HH4 = $04;  // Symbol Counter Output Compare Register 1 HH-Byte
  SCOCR1HH5 = $05;  // Symbol Counter Output Compare Register 1 HH-Byte
  SCOCR1HH6 = $06;  // Symbol Counter Output Compare Register 1 HH-Byte
  SCOCR1HH7 = $07;  // Symbol Counter Output Compare Register 1 HH-Byte
  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLL0 = $00;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLL1 = $01;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLL2 = $02;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLL3 = $03;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLL4 = $04;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLL5 = $05;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLL6 = $06;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  SCTSTRLL7 = $07;  // Symbol Counter Transmit Frame Timestamp Register LL-Byte
  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRLH0 = $00;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRLH1 = $01;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRLH2 = $02;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRLH3 = $03;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRLH4 = $04;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRLH5 = $05;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRLH6 = $06;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  SCTSTRLH7 = $07;  // Symbol Counter Transmit Frame Timestamp Register LH-Byte
  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHL0 = $00;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHL1 = $01;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHL2 = $02;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHL3 = $03;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHL4 = $04;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHL5 = $05;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHL6 = $06;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  SCTSTRHL7 = $07;  // Symbol Counter Transmit Frame Timestamp Register HL-Byte
  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  SCTSTRHH0 = $00;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  SCTSTRHH1 = $01;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  SCTSTRHH2 = $02;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  SCTSTRHH3 = $03;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  SCTSTRHH4 = $04;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  SCTSTRHH5 = $05;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  SCTSTRHH6 = $06;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  SCTSTRHH7 = $07;  // Symbol Counter Transmit Frame Timestamp Register HH-Byte
  // Multiple Address Filter Configuration Register 0
  MAF0EN = $00;  
  MAF1EN = $01;  
  MAF2EN = $02;  
  MAF3EN = $03;  
  // Multiple Address Filter Configuration Register 1
  AACK_0_I_AM_COORD = $00;  
  AACK_0_SET_PD = $01;  
  AACK_1_I_AM_COORD = $02;  
  AACK_1_SET_PD = $03;  
  AACK_2_I_AM_COORD = $04;  
  AACK_2_SET_PD = $05;  
  AACK_3_I_AM_COORD = $06;  
  AACK_3_SET_PD = $07;  
  // Transceiver MAC Short Address Register for Frame Filter 0 (Low Byte)
  MAFSA0L0 = $00;  // MAC Short Address low Byte for Frame Filter 0
  MAFSA0L1 = $01;  // MAC Short Address low Byte for Frame Filter 0
  MAFSA0L2 = $02;  // MAC Short Address low Byte for Frame Filter 0
  MAFSA0L3 = $03;  // MAC Short Address low Byte for Frame Filter 0
  MAFSA0L4 = $04;  // MAC Short Address low Byte for Frame Filter 0
  MAFSA0L5 = $05;  // MAC Short Address low Byte for Frame Filter 0
  MAFSA0L6 = $06;  // MAC Short Address low Byte for Frame Filter 0
  MAFSA0L7 = $07;  // MAC Short Address low Byte for Frame Filter 0
  // Transceiver MAC Short Address Register for Frame Filter 0 (High Byte)
  MAFSA0H0 = $00;  // MAC Short Address high Byte for Frame Filter 0
  MAFSA0H1 = $01;  // MAC Short Address high Byte for Frame Filter 0
  MAFSA0H2 = $02;  // MAC Short Address high Byte for Frame Filter 0
  MAFSA0H3 = $03;  // MAC Short Address high Byte for Frame Filter 0
  MAFSA0H4 = $04;  // MAC Short Address high Byte for Frame Filter 0
  MAFSA0H5 = $05;  // MAC Short Address high Byte for Frame Filter 0
  MAFSA0H6 = $06;  // MAC Short Address high Byte for Frame Filter 0
  MAFSA0H7 = $07;  // MAC Short Address high Byte for Frame Filter 0
  // Transceiver Personal Area Network ID Register for Frame Filter 0 (Low Byte)
  MAFPA0L0 = $00;  // MAC Personal Area Network ID low Byte for Frame Filter 0
  MAFPA0L1 = $01;  // MAC Personal Area Network ID low Byte for Frame Filter 0
  MAFPA0L2 = $02;  // MAC Personal Area Network ID low Byte for Frame Filter 0
  MAFPA0L3 = $03;  // MAC Personal Area Network ID low Byte for Frame Filter 0
  MAFPA0L4 = $04;  // MAC Personal Area Network ID low Byte for Frame Filter 0
  MAFPA0L5 = $05;  // MAC Personal Area Network ID low Byte for Frame Filter 0
  MAFPA0L6 = $06;  // MAC Personal Area Network ID low Byte for Frame Filter 0
  MAFPA0L7 = $07;  // MAC Personal Area Network ID low Byte for Frame Filter 0
  // Transceiver Personal Area Network ID Register for Frame Filter 0 (High Byte)
  MAFPA0H0 = $00;  // MAC Personal Area Network ID high Byte for Frame Filter 0
  MAFPA0H1 = $01;  // MAC Personal Area Network ID high Byte for Frame Filter 0
  MAFPA0H2 = $02;  // MAC Personal Area Network ID high Byte for Frame Filter 0
  MAFPA0H3 = $03;  // MAC Personal Area Network ID high Byte for Frame Filter 0
  MAFPA0H4 = $04;  // MAC Personal Area Network ID high Byte for Frame Filter 0
  MAFPA0H5 = $05;  // MAC Personal Area Network ID high Byte for Frame Filter 0
  MAFPA0H6 = $06;  // MAC Personal Area Network ID high Byte for Frame Filter 0
  MAFPA0H7 = $07;  // MAC Personal Area Network ID high Byte for Frame Filter 0
  // Transceiver MAC Short Address Register for Frame Filter 1 (Low Byte)
  MAFSA1L0 = $00;  // MAC Short Address low Byte for Frame Filter 1
  MAFSA1L1 = $01;  // MAC Short Address low Byte for Frame Filter 1
  MAFSA1L2 = $02;  // MAC Short Address low Byte for Frame Filter 1
  MAFSA1L3 = $03;  // MAC Short Address low Byte for Frame Filter 1
  MAFSA1L4 = $04;  // MAC Short Address low Byte for Frame Filter 1
  MAFSA1L5 = $05;  // MAC Short Address low Byte for Frame Filter 1
  MAFSA1L6 = $06;  // MAC Short Address low Byte for Frame Filter 1
  MAFSA1L7 = $07;  // MAC Short Address low Byte for Frame Filter 1
  // Transceiver MAC Short Address Register for Frame Filter 1 (High Byte)
  MAFSA1H0 = $00;  // MAC Short Address high Byte for Frame Filter 1
  MAFSA1H1 = $01;  // MAC Short Address high Byte for Frame Filter 1
  MAFSA1H2 = $02;  // MAC Short Address high Byte for Frame Filter 1
  MAFSA1H3 = $03;  // MAC Short Address high Byte for Frame Filter 1
  MAFSA1H4 = $04;  // MAC Short Address high Byte for Frame Filter 1
  MAFSA1H5 = $05;  // MAC Short Address high Byte for Frame Filter 1
  MAFSA1H6 = $06;  // MAC Short Address high Byte for Frame Filter 1
  MAFSA1H7 = $07;  // MAC Short Address high Byte for Frame Filter 1
  // Transceiver Personal Area Network ID Register for Frame Filter 1 (Low Byte)
  MAFPA1L0 = $00;  // MAC Personal Area Network ID low Byte for Frame Filter 1
  MAFPA1L1 = $01;  // MAC Personal Area Network ID low Byte for Frame Filter 1
  MAFPA1L2 = $02;  // MAC Personal Area Network ID low Byte for Frame Filter 1
  MAFPA1L3 = $03;  // MAC Personal Area Network ID low Byte for Frame Filter 1
  MAFPA1L4 = $04;  // MAC Personal Area Network ID low Byte for Frame Filter 1
  MAFPA1L5 = $05;  // MAC Personal Area Network ID low Byte for Frame Filter 1
  MAFPA1L6 = $06;  // MAC Personal Area Network ID low Byte for Frame Filter 1
  MAFPA1L7 = $07;  // MAC Personal Area Network ID low Byte for Frame Filter 1
  // Transceiver Personal Area Network ID Register for Frame Filter 1 (High Byte)
  MAFPA1H0 = $00;  // MAC Personal Area Network ID high Byte for Frame Filter 1
  MAFPA1H1 = $01;  // MAC Personal Area Network ID high Byte for Frame Filter 1
  MAFPA1H2 = $02;  // MAC Personal Area Network ID high Byte for Frame Filter 1
  MAFPA1H3 = $03;  // MAC Personal Area Network ID high Byte for Frame Filter 1
  MAFPA1H4 = $04;  // MAC Personal Area Network ID high Byte for Frame Filter 1
  MAFPA1H5 = $05;  // MAC Personal Area Network ID high Byte for Frame Filter 1
  MAFPA1H6 = $06;  // MAC Personal Area Network ID high Byte for Frame Filter 1
  MAFPA1H7 = $07;  // MAC Personal Area Network ID high Byte for Frame Filter 1
  // Transceiver MAC Short Address Register for Frame Filter 2 (Low Byte)
  MAFSA2L0 = $00;  // MAC Short Address low Byte for Frame Filter 2
  MAFSA2L1 = $01;  // MAC Short Address low Byte for Frame Filter 2
  MAFSA2L2 = $02;  // MAC Short Address low Byte for Frame Filter 2
  MAFSA2L3 = $03;  // MAC Short Address low Byte for Frame Filter 2
  MAFSA2L4 = $04;  // MAC Short Address low Byte for Frame Filter 2
  MAFSA2L5 = $05;  // MAC Short Address low Byte for Frame Filter 2
  MAFSA2L6 = $06;  // MAC Short Address low Byte for Frame Filter 2
  MAFSA2L7 = $07;  // MAC Short Address low Byte for Frame Filter 2
  // Transceiver MAC Short Address Register for Frame Filter 2 (High Byte)
  MAFSA2H0 = $00;  // MAC Short Address high Byte for Frame Filter 2
  MAFSA2H1 = $01;  // MAC Short Address high Byte for Frame Filter 2
  MAFSA2H2 = $02;  // MAC Short Address high Byte for Frame Filter 2
  MAFSA2H3 = $03;  // MAC Short Address high Byte for Frame Filter 2
  MAFSA2H4 = $04;  // MAC Short Address high Byte for Frame Filter 2
  MAFSA2H5 = $05;  // MAC Short Address high Byte for Frame Filter 2
  MAFSA2H6 = $06;  // MAC Short Address high Byte for Frame Filter 2
  MAFSA2H7 = $07;  // MAC Short Address high Byte for Frame Filter 2
  // Transceiver Personal Area Network ID Register for Frame Filter 2 (Low Byte)
  MAFPA2L0 = $00;  // MAC Personal Area Network ID low Byte for Frame Filter 2
  MAFPA2L1 = $01;  // MAC Personal Area Network ID low Byte for Frame Filter 2
  MAFPA2L2 = $02;  // MAC Personal Area Network ID low Byte for Frame Filter 2
  MAFPA2L3 = $03;  // MAC Personal Area Network ID low Byte for Frame Filter 2
  MAFPA2L4 = $04;  // MAC Personal Area Network ID low Byte for Frame Filter 2
  MAFPA2L5 = $05;  // MAC Personal Area Network ID low Byte for Frame Filter 2
  MAFPA2L6 = $06;  // MAC Personal Area Network ID low Byte for Frame Filter 2
  MAFPA2L7 = $07;  // MAC Personal Area Network ID low Byte for Frame Filter 2
  // Transceiver Personal Area Network ID Register for Frame Filter 2 (High Byte)
  MAFPA2H0 = $00;  // MAC Personal Area Network ID high Byte for Frame Filter 2
  MAFPA2H1 = $01;  // MAC Personal Area Network ID high Byte for Frame Filter 2
  MAFPA2H2 = $02;  // MAC Personal Area Network ID high Byte for Frame Filter 2
  MAFPA2H3 = $03;  // MAC Personal Area Network ID high Byte for Frame Filter 2
  MAFPA2H4 = $04;  // MAC Personal Area Network ID high Byte for Frame Filter 2
  MAFPA2H5 = $05;  // MAC Personal Area Network ID high Byte for Frame Filter 2
  MAFPA2H6 = $06;  // MAC Personal Area Network ID high Byte for Frame Filter 2
  MAFPA2H7 = $07;  // MAC Personal Area Network ID high Byte for Frame Filter 2
  // Transceiver MAC Short Address Register for Frame Filter 3 (Low Byte)
  MAFSA3L0 = $00;  // MAC Short Address low Byte for Frame Filter 3
  MAFSA3L1 = $01;  // MAC Short Address low Byte for Frame Filter 3
  MAFSA3L2 = $02;  // MAC Short Address low Byte for Frame Filter 3
  MAFSA3L3 = $03;  // MAC Short Address low Byte for Frame Filter 3
  MAFSA3L4 = $04;  // MAC Short Address low Byte for Frame Filter 3
  MAFSA3L5 = $05;  // MAC Short Address low Byte for Frame Filter 3
  MAFSA3L6 = $06;  // MAC Short Address low Byte for Frame Filter 3
  MAFSA3L7 = $07;  // MAC Short Address low Byte for Frame Filter 3
  // Transceiver MAC Short Address Register for Frame Filter 3 (High Byte)
  MAFSA3H0 = $00;  // MAC Short Address high Byte for Frame Filter 3
  MAFSA3H1 = $01;  // MAC Short Address high Byte for Frame Filter 3
  MAFSA3H2 = $02;  // MAC Short Address high Byte for Frame Filter 3
  MAFSA3H3 = $03;  // MAC Short Address high Byte for Frame Filter 3
  MAFSA3H4 = $04;  // MAC Short Address high Byte for Frame Filter 3
  MAFSA3H5 = $05;  // MAC Short Address high Byte for Frame Filter 3
  MAFSA3H6 = $06;  // MAC Short Address high Byte for Frame Filter 3
  MAFSA3H7 = $07;  // MAC Short Address high Byte for Frame Filter 3
  // Transceiver Personal Area Network ID Register for Frame Filter 3 (Low Byte)
  MAFPA3L0 = $00;  // MAC Personal Area Network ID low Byte for Frame Filter 3
  MAFPA3L1 = $01;  // MAC Personal Area Network ID low Byte for Frame Filter 3
  MAFPA3L2 = $02;  // MAC Personal Area Network ID low Byte for Frame Filter 3
  MAFPA3L3 = $03;  // MAC Personal Area Network ID low Byte for Frame Filter 3
  MAFPA3L4 = $04;  // MAC Personal Area Network ID low Byte for Frame Filter 3
  MAFPA3L5 = $05;  // MAC Personal Area Network ID low Byte for Frame Filter 3
  MAFPA3L6 = $06;  // MAC Personal Area Network ID low Byte for Frame Filter 3
  MAFPA3L7 = $07;  // MAC Personal Area Network ID low Byte for Frame Filter 3
  // Transceiver Personal Area Network ID Register for Frame Filter 3 (High Byte)
  MAFPA3H0 = $00;  // MAC Personal Area Network ID high Byte for Frame Filter 3
  MAFPA3H1 = $01;  // MAC Personal Area Network ID high Byte for Frame Filter 3
  MAFPA3H2 = $02;  // MAC Personal Area Network ID high Byte for Frame Filter 3
  MAFPA3H3 = $03;  // MAC Personal Area Network ID high Byte for Frame Filter 3
  MAFPA3H4 = $04;  // MAC Personal Area Network ID high Byte for Frame Filter 3
  MAFPA3H5 = $05;  // MAC Personal Area Network ID high Byte for Frame Filter 3
  MAFPA3H6 = $06;  // MAC Personal Area Network ID high Byte for Frame Filter 3
  MAFPA3H7 = $07;  // MAC Personal Area Network ID high Byte for Frame Filter 3
  // Timer/Counter5 Control Register A
  WGM50 = $00;  // Waveform Generation Mode
  WGM51 = $01;  // Waveform Generation Mode
  COM5C0 = $02;  // Compare Output Mode for Channel C
  COM5C1 = $03;  // Compare Output Mode for Channel C
  COM5B0 = $04;  // Compare Output Mode for Channel B
  COM5B1 = $05;  // Compare Output Mode for Channel B
  COM5A0 = $06;  // Compare Output Mode for Channel A
  COM5A1 = $07;  // Compare Output Mode for Channel A
  // Timer/Counter5 Control Register B
  CS50 = $00;  // Clock Select
  CS51 = $01;  // Clock Select
  CS52 = $02;  // Clock Select
  ICES5 = $06;  
  ICNC5 = $07;  
  // Timer/Counter5 Control Register C
  FOC5C = $05;  
  FOC5B = $06;  
  FOC5A = $07;  
  // Low Leakage Voltage Regulator Control Register
  LLENCAL = $00;  
  LLSHORT = $01;  
  LLTCO = $02;  
  LLCAL = $03;  
  LLCOMP = $04;  
  LLDONE = $05;  
  // Low Leakage Voltage Regulator Data Register (Low-Byte)
  LLDRL0 = $00;  // Low-Byte Data Register Bits
  LLDRL1 = $01;  // Low-Byte Data Register Bits
  LLDRL2 = $02;  // Low-Byte Data Register Bits
  LLDRL3 = $03;  // Low-Byte Data Register Bits
  // Low Leakage Voltage Regulator Data Register (High-Byte)
  LLDRH0 = $00;  // High-Byte Data Register Bits
  LLDRH1 = $01;  // High-Byte Data Register Bits
  LLDRH2 = $02;  // High-Byte Data Register Bits
  LLDRH3 = $03;  // High-Byte Data Register Bits
  LLDRH4 = $04;  // High-Byte Data Register Bits
  // Data Retention Configuration Register #0
  ENDRT = $04;  
  DRTSWOK = $05;  
  // Port Driver Strength Register 0
  PBDRV0 = $00;  // Driver Strength Port B
  PBDRV1 = $01;  // Driver Strength Port B
  PDDRV0 = $02;  // Driver Strength Port D
  PDDRV1 = $03;  // Driver Strength Port D
  PEDRV0 = $04;  // Driver Strength Port E
  PEDRV1 = $05;  // Driver Strength Port E
  PFDRV0 = $06;  // Driver Strength Port F
  PFDRV1 = $07;  // Driver Strength Port F
  // Port Driver Strength Register 1
  PGDRV0 = $00;  // Driver Strength Port G
  PGDRV1 = $01;  // Driver Strength Port G
  // Power Amplifier Ramp up/down Control Register
  PARUFI = $00;  
  PARDFI = $01;  
  PALTU0 = $02;  // ext. PA Ramp Up Lead Time
  PALTU1 = $03;  // ext. PA Ramp Up Lead Time
  PALTU2 = $04;  // ext. PA Ramp Up Lead Time
  PALTD0 = $05;  // ext. PA Ramp Down Lead Time
  PALTD1 = $06;  // ext. PA Ramp Down Lead Time
  PALTD2 = $07;  // ext. PA Ramp Down Lead Time
  // Transceiver Pin Register
  TRXRST = $00;  
  SLPTR = $01;  
  // AES Control Register
  AES_IM = $02;  
  AES_DIR = $03;  
  AES_MODE = $05;  
  AES_REQUEST = $07;  
  // AES Status Register
  AES_DONE = $00;  
  AES_ER = $07;  
  // AES Plain and Cipher Text Buffer Register
  AES_STATE0 = $00;  // AES Plain and Cipher Text Buffer
  AES_STATE1 = $01;  // AES Plain and Cipher Text Buffer
  AES_STATE2 = $02;  // AES Plain and Cipher Text Buffer
  AES_STATE3 = $03;  // AES Plain and Cipher Text Buffer
  AES_STATE4 = $04;  // AES Plain and Cipher Text Buffer
  AES_STATE5 = $05;  // AES Plain and Cipher Text Buffer
  AES_STATE6 = $06;  // AES Plain and Cipher Text Buffer
  AES_STATE7 = $07;  // AES Plain and Cipher Text Buffer
  // AES Encryption and Decryption Key Buffer Register
  AES_KEY0 = $00;  // AES Encryption/Decryption Key Buffer
  AES_KEY1 = $01;  // AES Encryption/Decryption Key Buffer
  AES_KEY2 = $02;  // AES Encryption/Decryption Key Buffer
  AES_KEY3 = $03;  // AES Encryption/Decryption Key Buffer
  AES_KEY4 = $04;  // AES Encryption/Decryption Key Buffer
  AES_KEY5 = $05;  // AES Encryption/Decryption Key Buffer
  AES_KEY6 = $06;  // AES Encryption/Decryption Key Buffer
  AES_KEY7 = $07;  // AES Encryption/Decryption Key Buffer
  // Transceiver Status Register
  TRX_STATUS0 = $00;  // Transceiver Main Status
  TRX_STATUS1 = $01;  // Transceiver Main Status
  TRX_STATUS2 = $02;  // Transceiver Main Status
  TRX_STATUS3 = $03;  // Transceiver Main Status
  TRX_STATUS4 = $04;  // Transceiver Main Status
  TST_STATUS = $05;  
  CCA_STATUS = $06;  
  CCA_DONE = $07;  
  // Transceiver State Control Register
  TRX_CMD0 = $00;  // State Control Command
  TRX_CMD1 = $01;  // State Control Command
  TRX_CMD2 = $02;  // State Control Command
  TRX_CMD3 = $03;  // State Control Command
  TRX_CMD4 = $04;  // State Control Command
  TRAC_STATUS0 = $05;  // Transaction Status
  TRAC_STATUS1 = $06;  // Transaction Status
  TRAC_STATUS2 = $07;  // Transaction Status
  // Reserved
  PMU_IF_INV = $04;  
  PMU_START = $05;  
  PMU_EN = $06;  
  Res7 = $07;  
  // Transceiver Control Register 1
  PLL_TX_FLT = $04;  
  TX_AUTO_CRC_ON = $05;  
  IRQ_2_EXT_EN = $06;  
  PA_EXT_EN = $07;  
  // Transceiver Transmit Power Control Register
  TX_PWR0 = $00;  // Transmit Power Setting
  TX_PWR1 = $01;  // Transmit Power Setting
  TX_PWR2 = $02;  // Transmit Power Setting
  TX_PWR3 = $03;  // Transmit Power Setting
  // Receiver Signal Strength Indicator Register
  RSSI0 = $00;  // Receiver Signal Strength Indicator
  RSSI1 = $01;  // Receiver Signal Strength Indicator
  RSSI2 = $02;  // Receiver Signal Strength Indicator
  RSSI3 = $03;  // Receiver Signal Strength Indicator
  RSSI4 = $04;  // Receiver Signal Strength Indicator
  RND_VALUE0 = $05;  // Random Value
  RND_VALUE1 = $06;  // Random Value
  RX_CRC_VALID = $07;  
  // Transceiver Energy Detection Level Register
  ED_LEVEL0 = $00;  // Energy Detection Level
  ED_LEVEL1 = $01;  // Energy Detection Level
  ED_LEVEL2 = $02;  // Energy Detection Level
  ED_LEVEL3 = $03;  // Energy Detection Level
  ED_LEVEL4 = $04;  // Energy Detection Level
  ED_LEVEL5 = $05;  // Energy Detection Level
  ED_LEVEL6 = $06;  // Energy Detection Level
  ED_LEVEL7 = $07;  // Energy Detection Level
  // Transceiver Clear Channel Assessment (CCA) Control Register
  CHANNEL0 = $00;  // RX/TX Channel Selection
  CHANNEL1 = $01;  // RX/TX Channel Selection
  CHANNEL2 = $02;  // RX/TX Channel Selection
  CHANNEL3 = $03;  // RX/TX Channel Selection
  CHANNEL4 = $04;  // RX/TX Channel Selection
  CCA_MODE0 = $05;  // Select CCA Measurement Mode
  CCA_MODE1 = $06;  // Select CCA Measurement Mode
  CCA_REQUEST = $07;  
  // Transceiver CCA Threshold Setting Register
  CCA_ED_THRES0 = $00;  // ED Threshold Level for CCA Measurement
  CCA_ED_THRES1 = $01;  // ED Threshold Level for CCA Measurement
  CCA_ED_THRES2 = $02;  // ED Threshold Level for CCA Measurement
  CCA_ED_THRES3 = $03;  // ED Threshold Level for CCA Measurement
  CCA_CS_THRES0 = $04;  // CS Threshold Level for CCA Measurement
  CCA_CS_THRES1 = $05;  // CS Threshold Level for CCA Measurement
  CCA_CS_THRES2 = $06;  // CS Threshold Level for CCA Measurement
  CCA_CS_THRES3 = $07;  // CS Threshold Level for CCA Measurement
  // Transceiver Receive Control Register
  PDT_THRES0 = $00;  // Receiver Sensitivity Control
  PDT_THRES1 = $01;  // Receiver Sensitivity Control
  PDT_THRES2 = $02;  // Receiver Sensitivity Control
  PDT_THRES3 = $03;  // Receiver Sensitivity Control
  // Start of Frame Delimiter Value Register
  SFD_VALUE0 = $00;  // Start of Frame Delimiter Value
  SFD_VALUE1 = $01;  // Start of Frame Delimiter Value
  SFD_VALUE2 = $02;  // Start of Frame Delimiter Value
  SFD_VALUE3 = $03;  // Start of Frame Delimiter Value
  SFD_VALUE4 = $04;  // Start of Frame Delimiter Value
  SFD_VALUE5 = $05;  // Start of Frame Delimiter Value
  SFD_VALUE6 = $06;  // Start of Frame Delimiter Value
  SFD_VALUE7 = $07;  // Start of Frame Delimiter Value
  // Transceiver Control Register 2
  OQPSK_DATA_RATE0 = $00;  // Data Rate Selection
  OQPSK_DATA_RATE1 = $01;  // Data Rate Selection
  RX_SAFE_MODE = $07;  
  // Antenna Diversity Control Register
  ANT_CTRL0 = $00;  // Static Antenna Diversity Switch Control
  ANT_CTRL1 = $01;  // Static Antenna Diversity Switch Control
  ANT_EXT_SW_EN = $02;  
  ANT_DIV_EN = $03;  
  ANT_SEL = $07;  
  // Transceiver Interrupt Enable Register
  PLL_LOCK_EN = $00;  
  PLL_UNLOCK_EN = $01;  
  RX_START_EN = $02;  
  RX_END_EN = $03;  
  CCA_ED_DONE_EN = $04;  
  AMI_EN = $05;  
  TX_END_EN = $06;  
  AWAKE_EN = $07;  
  // Transceiver Interrupt Status Register
  PLL_LOCK = $00;  
  PLL_UNLOCK = $01;  
  RX_START = $02;  
  RX_END = $03;  
  CCA_ED_DONE = $04;  
  AMI = $05;  
  TX_END = $06;  
  AWAKE = $07;  
  // Voltage Regulator Control and Status Register
  DVDD_OK = $02;  
  DVREG_EXT = $03;  
  AVDD_OK = $06;  
  AVREG_EXT = $07;  
  // Battery Monitor Control and Status Register
  BATMON_VTH0 = $00;  // Battery Monitor Threshold Voltage
  BATMON_VTH1 = $01;  // Battery Monitor Threshold Voltage
  BATMON_VTH2 = $02;  // Battery Monitor Threshold Voltage
  BATMON_VTH3 = $03;  // Battery Monitor Threshold Voltage
  BATMON_HR = $04;  
  BATMON_OK = $05;  
  BAT_LOW_EN = $06;  
  BAT_LOW = $07;  
  // Crystal Oscillator Control Register
  XTAL_TRIM0 = $00;  // Crystal Oscillator Load Capacitance Trimming
  XTAL_TRIM1 = $01;  // Crystal Oscillator Load Capacitance Trimming
  XTAL_TRIM2 = $02;  // Crystal Oscillator Load Capacitance Trimming
  XTAL_TRIM3 = $03;  // Crystal Oscillator Load Capacitance Trimming
  XTAL_MODE0 = $04;  // Crystal Oscillator Operating Mode
  XTAL_MODE1 = $05;  // Crystal Oscillator Operating Mode
  XTAL_MODE2 = $06;  // Crystal Oscillator Operating Mode
  XTAL_MODE3 = $07;  // Crystal Oscillator Operating Mode
  // Channel Control Register 0
  CC_NUMBER0 = $00;  // Channel Number
  CC_NUMBER1 = $01;  // Channel Number
  CC_NUMBER2 = $02;  // Channel Number
  CC_NUMBER3 = $03;  // Channel Number
  CC_NUMBER4 = $04;  // Channel Number
  CC_NUMBER5 = $05;  // Channel Number
  CC_NUMBER6 = $06;  // Channel Number
  CC_NUMBER7 = $07;  // Channel Number
  // Channel Control Register 1
  CC_BAND0 = $00;  // Channel Band
  CC_BAND1 = $01;  // Channel Band
  CC_BAND2 = $02;  // Channel Band
  CC_BAND3 = $03;  // Channel Band
  // Transceiver Receiver Sensitivity Control Register
  RX_PDT_LEVEL0 = $00;  // Reduce Receiver Sensitivity
  RX_PDT_LEVEL1 = $01;  // Reduce Receiver Sensitivity
  RX_PDT_LEVEL2 = $02;  // Reduce Receiver Sensitivity
  RX_PDT_LEVEL3 = $03;  // Reduce Receiver Sensitivity
  RX_OVERRIDE = $06;  
  RX_PDT_DIS = $07;  
  // Transceiver Reduced Power Consumption Control
  XAH_RPC_EN = $00;  
  IPAN_RPC_EN = $01;  
  Res0 = $02;  
  PLL_RPC_EN = $03;  
  PDT_RPC_EN = $04;  
  RX_RPC_EN = $05;  
  RX_RPC_CTRL0 = $06;  // Smart Receiving Mode Timing
  RX_RPC_CTRL1 = $07;  // Smart Receiving Mode Timing
  // Transceiver Acknowledgment Frame Control Register 1
  AACK_PROM_MODE = $01;  
  AACK_ACK_TIME = $02;  
  AACK_UPLD_RES_FT = $04;  
  AACK_FLTR_RES_FT = $05;  
  // Transceiver Filter Tuning Control Register
  FTN_START = $07;  
  // Transceiver Center Frequency Calibration Control Register
  PLL_CF_START = $07;  
  // Transceiver Delay Cell Calibration Control Register
  PLL_DCU_START = $07;  
  // Device Identification Register (Part Number)
  PART_NUM0 = $00;  // Part Number
  PART_NUM1 = $01;  // Part Number
  PART_NUM2 = $02;  // Part Number
  PART_NUM3 = $03;  // Part Number
  PART_NUM4 = $04;  // Part Number
  PART_NUM5 = $05;  // Part Number
  PART_NUM6 = $06;  // Part Number
  PART_NUM7 = $07;  // Part Number
  // Device Identification Register (Version Number)
  VERSION_NUM0 = $00;  // Version Number
  VERSION_NUM1 = $01;  // Version Number
  VERSION_NUM2 = $02;  // Version Number
  VERSION_NUM3 = $03;  // Version Number
  VERSION_NUM4 = $04;  // Version Number
  VERSION_NUM5 = $05;  // Version Number
  VERSION_NUM6 = $06;  // Version Number
  VERSION_NUM7 = $07;  // Version Number
  // Device Identification Register (Manufacture ID Low Byte)
  MAN_ID_00 = $00;  
  MAN_ID_01 = $01;  
  MAN_ID_02 = $02;  
  MAN_ID_03 = $03;  
  MAN_ID_04 = $04;  
  MAN_ID_05 = $05;  
  MAN_ID_06 = $06;  
  MAN_ID_07 = $07;  
  // Device Identification Register (Manufacture ID High Byte)
  MAN_ID_10 = $00;  // Manufacturer ID (High Byte)
  MAN_ID_11 = $01;  // Manufacturer ID (High Byte)
  MAN_ID_12 = $02;  // Manufacturer ID (High Byte)
  MAN_ID_13 = $03;  // Manufacturer ID (High Byte)
  MAN_ID_14 = $04;  // Manufacturer ID (High Byte)
  MAN_ID_15 = $05;  // Manufacturer ID (High Byte)
  MAN_ID_16 = $06;  // Manufacturer ID (High Byte)
  MAN_ID_17 = $07;  // Manufacturer ID (High Byte)
  // Transceiver MAC Short Address Register (Low Byte)
  SHORT_ADDR_00 = $00;  
  SHORT_ADDR_01 = $01;  
  SHORT_ADDR_02 = $02;  
  SHORT_ADDR_03 = $03;  
  SHORT_ADDR_04 = $04;  
  SHORT_ADDR_05 = $05;  
  SHORT_ADDR_06 = $06;  
  SHORT_ADDR_07 = $07;  
  // Transceiver MAC Short Address Register (High Byte)
  SHORT_ADDR_10 = $00;  // MAC Short Address
  SHORT_ADDR_11 = $01;  // MAC Short Address
  SHORT_ADDR_12 = $02;  // MAC Short Address
  SHORT_ADDR_13 = $03;  // MAC Short Address
  SHORT_ADDR_14 = $04;  // MAC Short Address
  SHORT_ADDR_15 = $05;  // MAC Short Address
  SHORT_ADDR_16 = $06;  // MAC Short Address
  SHORT_ADDR_17 = $07;  // MAC Short Address
  // Transceiver Personal Area Network ID Register (Low Byte)
  PAN_ID_00 = $00;  
  PAN_ID_01 = $01;  
  PAN_ID_02 = $02;  
  PAN_ID_03 = $03;  
  PAN_ID_04 = $04;  
  PAN_ID_05 = $05;  
  PAN_ID_06 = $06;  
  PAN_ID_07 = $07;  
  // Transceiver Personal Area Network ID Register (High Byte)
  PAN_ID_10 = $00;  // MAC Personal Area Network ID
  PAN_ID_11 = $01;  // MAC Personal Area Network ID
  PAN_ID_12 = $02;  // MAC Personal Area Network ID
  PAN_ID_13 = $03;  // MAC Personal Area Network ID
  PAN_ID_14 = $04;  // MAC Personal Area Network ID
  PAN_ID_15 = $05;  // MAC Personal Area Network ID
  PAN_ID_16 = $06;  // MAC Personal Area Network ID
  PAN_ID_17 = $07;  // MAC Personal Area Network ID
  // Transceiver MAC IEEE Address Register 0
  IEEE_ADDR_00 = $00;  
  IEEE_ADDR_01 = $01;  
  IEEE_ADDR_02 = $02;  
  IEEE_ADDR_03 = $03;  
  IEEE_ADDR_04 = $04;  
  IEEE_ADDR_05 = $05;  
  IEEE_ADDR_06 = $06;  
  IEEE_ADDR_07 = $07;  
  // Transceiver MAC IEEE Address Register 1
  IEEE_ADDR_10 = $00;  // MAC IEEE Address
  IEEE_ADDR_11 = $01;  // MAC IEEE Address
  IEEE_ADDR_12 = $02;  // MAC IEEE Address
  IEEE_ADDR_13 = $03;  // MAC IEEE Address
  IEEE_ADDR_14 = $04;  // MAC IEEE Address
  IEEE_ADDR_15 = $05;  // MAC IEEE Address
  IEEE_ADDR_16 = $06;  // MAC IEEE Address
  IEEE_ADDR_17 = $07;  // MAC IEEE Address
  // Transceiver Extended Operating Mode Control Register
  SLOTTED_OPERATION = $00;  
  MAX_CSMA_RETRIES0 = $01;  // Maximum Number of CSMA-CA Procedure Repetition Attempts
  MAX_CSMA_RETRIES1 = $02;  // Maximum Number of CSMA-CA Procedure Repetition Attempts
  MAX_CSMA_RETRIES2 = $03;  // Maximum Number of CSMA-CA Procedure Repetition Attempts
  MAX_FRAME_RETRIES0 = $04;  // Maximum Number of Frame Re-transmission Attempts
  MAX_FRAME_RETRIES1 = $05;  // Maximum Number of Frame Re-transmission Attempts
  MAX_FRAME_RETRIES2 = $06;  // Maximum Number of Frame Re-transmission Attempts
  MAX_FRAME_RETRIES3 = $07;  // Maximum Number of Frame Re-transmission Attempts
  // Transceiver CSMA-CA Random Number Generator Seed Register
  CSMA_SEED_00 = $00;  
  CSMA_SEED_01 = $01;  
  CSMA_SEED_02 = $02;  
  CSMA_SEED_03 = $03;  
  CSMA_SEED_04 = $04;  
  CSMA_SEED_05 = $05;  
  CSMA_SEED_06 = $06;  
  CSMA_SEED_07 = $07;  
  // Transceiver Acknowledgment Frame Control Register 2
  CSMA_SEED_10 = $00;  // Seed Value for CSMA Random Number Generator
  CSMA_SEED_11 = $01;  // Seed Value for CSMA Random Number Generator
  CSMA_SEED_12 = $02;  // Seed Value for CSMA Random Number Generator
  AACK_I_AM_COORD = $03;  
  AACK_DIS_ACK = $04;  
  AACK_SET_PD = $05;  
  AACK_FVN_MODE0 = $06;  // Acknowledgment Frame Filter Mode
  AACK_FVN_MODE1 = $07;  // Acknowledgment Frame Filter Mode
  // Transceiver CSMA-CA Back-off Exponent Control Register
  MIN_BE0 = $00;  // Minimum Back-off Exponent
  MIN_BE1 = $01;  // Minimum Back-off Exponent
  MIN_BE2 = $02;  // Minimum Back-off Exponent
  MIN_BE3 = $03;  // Minimum Back-off Exponent
  MAX_BE0 = $04;  // Maximum Back-off Exponent
  MAX_BE1 = $05;  // Maximum Back-off Exponent
  MAX_BE2 = $06;  // Maximum Back-off Exponent
  MAX_BE3 = $07;  // Maximum Back-off Exponent
  // Transceiver Digital Test Control Register
  TST_CTRL_DIG0 = $00;  // Digital Test Controller Register
  TST_CTRL_DIG1 = $01;  // Digital Test Controller Register
  TST_CTRL_DIG2 = $02;  // Digital Test Controller Register
  TST_CTRL_DIG3 = $03;  // Digital Test Controller Register
  // Transceiver Received Frame Length Register
  RX_LENGTH0 = $00;  // Received Frame Length
  RX_LENGTH1 = $01;  // Received Frame Length
  RX_LENGTH2 = $02;  // Received Frame Length
  RX_LENGTH3 = $03;  // Received Frame Length
  RX_LENGTH4 = $04;  // Received Frame Length
  RX_LENGTH5 = $05;  // Received Frame Length
  RX_LENGTH6 = $06;  // Received Frame Length
  RX_LENGTH7 = $07;  // Received Frame Length


implementation

{$i avrcommon.inc}

procedure INT0_ISR; external name 'INT0_ISR'; // Interrupt 1 External Interrupt Request 0
procedure INT1_ISR; external name 'INT1_ISR'; // Interrupt 2 External Interrupt Request 1
procedure INT2_ISR; external name 'INT2_ISR'; // Interrupt 3 External Interrupt Request 2
procedure INT3_ISR; external name 'INT3_ISR'; // Interrupt 4 External Interrupt Request 3
procedure INT4_ISR; external name 'INT4_ISR'; // Interrupt 5 External Interrupt Request 4
procedure INT5_ISR; external name 'INT5_ISR'; // Interrupt 6 External Interrupt Request 5
procedure INT6_ISR; external name 'INT6_ISR'; // Interrupt 7 External Interrupt Request 6
procedure INT7_ISR; external name 'INT7_ISR'; // Interrupt 8 External Interrupt Request 7
procedure PCINT0_ISR; external name 'PCINT0_ISR'; // Interrupt 9 Pin Change Interrupt Request 0
procedure PCINT1_ISR; external name 'PCINT1_ISR'; // Interrupt 10 Pin Change Interrupt Request 1
procedure PCINT2_ISR; external name 'PCINT2_ISR'; // Interrupt 11 Pin Change Interrupt Request 2
procedure WDT_ISR; external name 'WDT_ISR'; // Interrupt 12 Watchdog Time-out Interrupt
procedure TIMER2_COMPA_ISR; external name 'TIMER2_COMPA_ISR'; // Interrupt 13 Timer/Counter2 Compare Match A
procedure TIMER2_COMPB_ISR; external name 'TIMER2_COMPB_ISR'; // Interrupt 14 Timer/Counter2 Compare Match B
procedure TIMER2_OVF_ISR; external name 'TIMER2_OVF_ISR'; // Interrupt 15 Timer/Counter2 Overflow
procedure TIMER1_CAPT_ISR; external name 'TIMER1_CAPT_ISR'; // Interrupt 16 Timer/Counter1 Capture Event
procedure TIMER1_COMPA_ISR; external name 'TIMER1_COMPA_ISR'; // Interrupt 17 Timer/Counter1 Compare Match A
procedure TIMER1_COMPB_ISR; external name 'TIMER1_COMPB_ISR'; // Interrupt 18 Timer/Counter1 Compare Match B
procedure TIMER1_COMPC_ISR; external name 'TIMER1_COMPC_ISR'; // Interrupt 19 Timer/Counter1 Compare Match C
procedure TIMER1_OVF_ISR; external name 'TIMER1_OVF_ISR'; // Interrupt 20 Timer/Counter1 Overflow
procedure TIMER0_COMPA_ISR; external name 'TIMER0_COMPA_ISR'; // Interrupt 21 Timer/Counter0 Compare Match A
procedure TIMER0_COMPB_ISR; external name 'TIMER0_COMPB_ISR'; // Interrupt 22 Timer/Counter0 Compare Match B
procedure TIMER0_OVF_ISR; external name 'TIMER0_OVF_ISR'; // Interrupt 23 Timer/Counter0 Overflow
procedure SPI_STC_ISR; external name 'SPI_STC_ISR'; // Interrupt 24 SPI Serial Transfer Complete
procedure USART0_RX_ISR; external name 'USART0_RX_ISR'; // Interrupt 25 USART0, Rx Complete
procedure USART0_UDRE_ISR; external name 'USART0_UDRE_ISR'; // Interrupt 26 USART0 Data register Empty
procedure USART0_TX_ISR; external name 'USART0_TX_ISR'; // Interrupt 27 USART0, Tx Complete
procedure ANALOG_COMP_ISR; external name 'ANALOG_COMP_ISR'; // Interrupt 28 Analog Comparator
procedure ADC_ISR; external name 'ADC_ISR'; // Interrupt 29 ADC Conversion Complete
procedure EE_READY_ISR; external name 'EE_READY_ISR'; // Interrupt 30 EEPROM Ready
procedure TIMER3_CAPT_ISR; external name 'TIMER3_CAPT_ISR'; // Interrupt 31 Timer/Counter3 Capture Event
procedure TIMER3_COMPA_ISR; external name 'TIMER3_COMPA_ISR'; // Interrupt 32 Timer/Counter3 Compare Match A
procedure TIMER3_COMPB_ISR; external name 'TIMER3_COMPB_ISR'; // Interrupt 33 Timer/Counter3 Compare Match B
procedure TIMER3_COMPC_ISR; external name 'TIMER3_COMPC_ISR'; // Interrupt 34 Timer/Counter3 Compare Match C
procedure TIMER3_OVF_ISR; external name 'TIMER3_OVF_ISR'; // Interrupt 35 Timer/Counter3 Overflow
procedure USART1_RX_ISR; external name 'USART1_RX_ISR'; // Interrupt 36 USART1, Rx Complete
procedure USART1_UDRE_ISR; external name 'USART1_UDRE_ISR'; // Interrupt 37 USART1 Data register Empty
procedure USART1_TX_ISR; external name 'USART1_TX_ISR'; // Interrupt 38 USART1, Tx Complete
procedure TWI_ISR; external name 'TWI_ISR'; // Interrupt 39 2-wire Serial Interface
procedure SPM_READY_ISR; external name 'SPM_READY_ISR'; // Interrupt 40 Store Program Memory Read
procedure TIMER4_CAPT_ISR; external name 'TIMER4_CAPT_ISR'; // Interrupt 41 Timer/Counter4 Capture Event
procedure TIMER4_COMPA_ISR; external name 'TIMER4_COMPA_ISR'; // Interrupt 42 Timer/Counter4 Compare Match A
procedure TIMER4_COMPB_ISR; external name 'TIMER4_COMPB_ISR'; // Interrupt 43 Timer/Counter4 Compare Match B
procedure TIMER4_COMPC_ISR; external name 'TIMER4_COMPC_ISR'; // Interrupt 44 Timer/Counter4 Compare Match C
procedure TIMER4_OVF_ISR; external name 'TIMER4_OVF_ISR'; // Interrupt 45 Timer/Counter4 Overflow
procedure TIMER5_CAPT_ISR; external name 'TIMER5_CAPT_ISR'; // Interrupt 46 Timer/Counter5 Capture Event
procedure TIMER5_COMPA_ISR; external name 'TIMER5_COMPA_ISR'; // Interrupt 47 Timer/Counter5 Compare Match A
procedure TIMER5_COMPB_ISR; external name 'TIMER5_COMPB_ISR'; // Interrupt 48 Timer/Counter5 Compare Match B
procedure TIMER5_COMPC_ISR; external name 'TIMER5_COMPC_ISR'; // Interrupt 49 Timer/Counter5 Compare Match C
procedure TIMER5_OVF_ISR; external name 'TIMER5_OVF_ISR'; // Interrupt 50 Timer/Counter5 Overflow
procedure TRX24_PLL_LOCK_ISR; external name 'TRX24_PLL_LOCK_ISR'; // Interrupt 57 TRX24 - PLL lock interrupt
procedure TRX24_PLL_UNLOCK_ISR; external name 'TRX24_PLL_UNLOCK_ISR'; // Interrupt 58 TRX24 - PLL unlock interrupt
procedure TRX24_RX_START_ISR; external name 'TRX24_RX_START_ISR'; // Interrupt 59 TRX24 - Receive start interrupt
procedure TRX24_RX_END_ISR; external name 'TRX24_RX_END_ISR'; // Interrupt 60 TRX24 - RX_END interrupt
procedure TRX24_CCA_ED_DONE_ISR; external name 'TRX24_CCA_ED_DONE_ISR'; // Interrupt 61 TRX24 - CCA/ED done interrupt
procedure TRX24_XAH_AMI_ISR; external name 'TRX24_XAH_AMI_ISR'; // Interrupt 62 TRX24 - XAH - AMI
procedure TRX24_TX_END_ISR; external name 'TRX24_TX_END_ISR'; // Interrupt 63 TRX24 - TX_END interrupt
procedure TRX24_AWAKE_ISR; external name 'TRX24_AWAKE_ISR'; // Interrupt 64 TRX24 AWAKE - tranceiver is reaching state TRX_OFF
procedure SCNT_CMP1_ISR; external name 'SCNT_CMP1_ISR'; // Interrupt 65 Symbol counter - compare match 1 interrupt
procedure SCNT_CMP2_ISR; external name 'SCNT_CMP2_ISR'; // Interrupt 66 Symbol counter - compare match 2 interrupt
procedure SCNT_CMP3_ISR; external name 'SCNT_CMP3_ISR'; // Interrupt 67 Symbol counter - compare match 3 interrupt
procedure SCNT_OVFL_ISR; external name 'SCNT_OVFL_ISR'; // Interrupt 68 Symbol counter - overflow interrupt
procedure SCNT_BACKOFF_ISR; external name 'SCNT_BACKOFF_ISR'; // Interrupt 69 Symbol counter - backoff interrupt
procedure AES_READY_ISR; external name 'AES_READY_ISR'; // Interrupt 70 AES engine ready interrupt
procedure BAT_LOW_ISR; external name 'BAT_LOW_ISR'; // Interrupt 71 Battery monitor indicates supply voltage below threshold
procedure TRX24_TX_START_ISR; external name 'TRX24_TX_START_ISR'; // Interrupt 72 TRX24 TX start interrupt
procedure TRX24_AMI0_ISR; external name 'TRX24_AMI0_ISR'; // Interrupt 73 Address match interrupt of address filter 0
procedure TRX24_AMI1_ISR; external name 'TRX24_AMI1_ISR'; // Interrupt 74 Address match interrupt of address filter 1
procedure TRX24_AMI2_ISR; external name 'TRX24_AMI2_ISR'; // Interrupt 75 Address match interrupt of address filter 2
procedure TRX24_AMI3_ISR; external name 'TRX24_AMI3_ISR'; // Interrupt 76 Address match interrupt of address filter 3

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  jmp _start
  jmp INT0_ISR
  jmp INT1_ISR
  jmp INT2_ISR
  jmp INT3_ISR
  jmp INT4_ISR
  jmp INT5_ISR
  jmp INT6_ISR
  jmp INT7_ISR
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
  jmp TIMER1_COMPC_ISR
  jmp TIMER1_OVF_ISR
  jmp TIMER0_COMPA_ISR
  jmp TIMER0_COMPB_ISR
  jmp TIMER0_OVF_ISR
  jmp SPI_STC_ISR
  jmp USART0_RX_ISR
  jmp USART0_UDRE_ISR
  jmp USART0_TX_ISR
  jmp ANALOG_COMP_ISR
  jmp ADC_ISR
  jmp EE_READY_ISR
  jmp TIMER3_CAPT_ISR
  jmp TIMER3_COMPA_ISR
  jmp TIMER3_COMPB_ISR
  jmp TIMER3_COMPC_ISR
  jmp TIMER3_OVF_ISR
  jmp USART1_RX_ISR
  jmp USART1_UDRE_ISR
  jmp USART1_TX_ISR
  jmp TWI_ISR
  jmp SPM_READY_ISR
  jmp TIMER4_CAPT_ISR
  jmp TIMER4_COMPA_ISR
  jmp TIMER4_COMPB_ISR
  jmp TIMER4_COMPC_ISR
  jmp TIMER4_OVF_ISR
  jmp TIMER5_CAPT_ISR
  jmp TIMER5_COMPA_ISR
  jmp TIMER5_COMPB_ISR
  jmp TIMER5_COMPC_ISR
  jmp TIMER5_OVF_ISR
  jmp TRX24_PLL_LOCK_ISR
  jmp TRX24_PLL_UNLOCK_ISR
  jmp TRX24_RX_START_ISR
  jmp TRX24_RX_END_ISR
  jmp TRX24_CCA_ED_DONE_ISR
  jmp TRX24_XAH_AMI_ISR
  jmp TRX24_TX_END_ISR
  jmp TRX24_AWAKE_ISR
  jmp SCNT_CMP1_ISR
  jmp SCNT_CMP2_ISR
  jmp SCNT_CMP3_ISR
  jmp SCNT_OVFL_ISR
  jmp SCNT_BACKOFF_ISR
  jmp AES_READY_ISR
  jmp BAT_LOW_ISR
  jmp TRX24_TX_START_ISR
  jmp TRX24_AMI0_ISR
  jmp TRX24_AMI1_ISR
  jmp TRX24_AMI2_ISR
  jmp TRX24_AMI3_ISR

  {$i start.inc}

  .weak INT0_ISR
  .weak INT1_ISR
  .weak INT2_ISR
  .weak INT3_ISR
  .weak INT4_ISR
  .weak INT5_ISR
  .weak INT6_ISR
  .weak INT7_ISR
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
  .weak TIMER1_COMPC_ISR
  .weak TIMER1_OVF_ISR
  .weak TIMER0_COMPA_ISR
  .weak TIMER0_COMPB_ISR
  .weak TIMER0_OVF_ISR
  .weak SPI_STC_ISR
  .weak USART0_RX_ISR
  .weak USART0_UDRE_ISR
  .weak USART0_TX_ISR
  .weak ANALOG_COMP_ISR
  .weak ADC_ISR
  .weak EE_READY_ISR
  .weak TIMER3_CAPT_ISR
  .weak TIMER3_COMPA_ISR
  .weak TIMER3_COMPB_ISR
  .weak TIMER3_COMPC_ISR
  .weak TIMER3_OVF_ISR
  .weak USART1_RX_ISR
  .weak USART1_UDRE_ISR
  .weak USART1_TX_ISR
  .weak TWI_ISR
  .weak SPM_READY_ISR
  .weak TIMER4_CAPT_ISR
  .weak TIMER4_COMPA_ISR
  .weak TIMER4_COMPB_ISR
  .weak TIMER4_COMPC_ISR
  .weak TIMER4_OVF_ISR
  .weak TIMER5_CAPT_ISR
  .weak TIMER5_COMPA_ISR
  .weak TIMER5_COMPB_ISR
  .weak TIMER5_COMPC_ISR
  .weak TIMER5_OVF_ISR
  .weak TRX24_PLL_LOCK_ISR
  .weak TRX24_PLL_UNLOCK_ISR
  .weak TRX24_RX_START_ISR
  .weak TRX24_RX_END_ISR
  .weak TRX24_CCA_ED_DONE_ISR
  .weak TRX24_XAH_AMI_ISR
  .weak TRX24_TX_END_ISR
  .weak TRX24_AWAKE_ISR
  .weak SCNT_CMP1_ISR
  .weak SCNT_CMP2_ISR
  .weak SCNT_CMP3_ISR
  .weak SCNT_OVFL_ISR
  .weak SCNT_BACKOFF_ISR
  .weak AES_READY_ISR
  .weak BAT_LOW_ISR
  .weak TRX24_TX_START_ISR
  .weak TRX24_AMI0_ISR
  .weak TRX24_AMI1_ISR
  .weak TRX24_AMI2_ISR
  .weak TRX24_AMI3_ISR

  .set INT0_ISR, Default_IRQ_handler
  .set INT1_ISR, Default_IRQ_handler
  .set INT2_ISR, Default_IRQ_handler
  .set INT3_ISR, Default_IRQ_handler
  .set INT4_ISR, Default_IRQ_handler
  .set INT5_ISR, Default_IRQ_handler
  .set INT6_ISR, Default_IRQ_handler
  .set INT7_ISR, Default_IRQ_handler
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
  .set TIMER1_COMPC_ISR, Default_IRQ_handler
  .set TIMER1_OVF_ISR, Default_IRQ_handler
  .set TIMER0_COMPA_ISR, Default_IRQ_handler
  .set TIMER0_COMPB_ISR, Default_IRQ_handler
  .set TIMER0_OVF_ISR, Default_IRQ_handler
  .set SPI_STC_ISR, Default_IRQ_handler
  .set USART0_RX_ISR, Default_IRQ_handler
  .set USART0_UDRE_ISR, Default_IRQ_handler
  .set USART0_TX_ISR, Default_IRQ_handler
  .set ANALOG_COMP_ISR, Default_IRQ_handler
  .set ADC_ISR, Default_IRQ_handler
  .set EE_READY_ISR, Default_IRQ_handler
  .set TIMER3_CAPT_ISR, Default_IRQ_handler
  .set TIMER3_COMPA_ISR, Default_IRQ_handler
  .set TIMER3_COMPB_ISR, Default_IRQ_handler
  .set TIMER3_COMPC_ISR, Default_IRQ_handler
  .set TIMER3_OVF_ISR, Default_IRQ_handler
  .set USART1_RX_ISR, Default_IRQ_handler
  .set USART1_UDRE_ISR, Default_IRQ_handler
  .set USART1_TX_ISR, Default_IRQ_handler
  .set TWI_ISR, Default_IRQ_handler
  .set SPM_READY_ISR, Default_IRQ_handler
  .set TIMER4_CAPT_ISR, Default_IRQ_handler
  .set TIMER4_COMPA_ISR, Default_IRQ_handler
  .set TIMER4_COMPB_ISR, Default_IRQ_handler
  .set TIMER4_COMPC_ISR, Default_IRQ_handler
  .set TIMER4_OVF_ISR, Default_IRQ_handler
  .set TIMER5_CAPT_ISR, Default_IRQ_handler
  .set TIMER5_COMPA_ISR, Default_IRQ_handler
  .set TIMER5_COMPB_ISR, Default_IRQ_handler
  .set TIMER5_COMPC_ISR, Default_IRQ_handler
  .set TIMER5_OVF_ISR, Default_IRQ_handler
  .set TRX24_PLL_LOCK_ISR, Default_IRQ_handler
  .set TRX24_PLL_UNLOCK_ISR, Default_IRQ_handler
  .set TRX24_RX_START_ISR, Default_IRQ_handler
  .set TRX24_RX_END_ISR, Default_IRQ_handler
  .set TRX24_CCA_ED_DONE_ISR, Default_IRQ_handler
  .set TRX24_XAH_AMI_ISR, Default_IRQ_handler
  .set TRX24_TX_END_ISR, Default_IRQ_handler
  .set TRX24_AWAKE_ISR, Default_IRQ_handler
  .set SCNT_CMP1_ISR, Default_IRQ_handler
  .set SCNT_CMP2_ISR, Default_IRQ_handler
  .set SCNT_CMP3_ISR, Default_IRQ_handler
  .set SCNT_OVFL_ISR, Default_IRQ_handler
  .set SCNT_BACKOFF_ISR, Default_IRQ_handler
  .set AES_READY_ISR, Default_IRQ_handler
  .set BAT_LOW_ISR, Default_IRQ_handler
  .set TRX24_TX_START_ISR, Default_IRQ_handler
  .set TRX24_AMI0_ISR, Default_IRQ_handler
  .set TRX24_AMI1_ISR, Default_IRQ_handler
  .set TRX24_AMI2_ISR, Default_IRQ_handler
  .set TRX24_AMI3_ISR, Default_IRQ_handler
end;

end.
