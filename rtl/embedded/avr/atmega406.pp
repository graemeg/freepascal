unit ATmega406;

{$goto on}
interface

var
  PINA: byte absolute $20;  // Port A Input Pins
  DDRA: byte absolute $21;  // Port A Data Direction Register
  PORTA: byte absolute $22;  // Port A Data Register
  PINB: byte absolute $23;  // Port B Input Pins
  DDRB: byte absolute $24;  // Port B Data Direction Register
  PORTB: byte absolute $25;  // Port B Data Register
  PORTC: byte absolute $28;  // Port C Data Register
  PIND: byte absolute $29;  // Input Pins, Port D
  DDRD: byte absolute $2A;  // Data Direction Register, Port D
  PORTD: byte absolute $2B;  // Data Register, Port D
  TIFR0: byte absolute $35;  // Timer/Counter Interrupt Flag register
  TIFR1: byte absolute $36;  // Timer/Counter Interrupt Flag register
  PCIFR: byte absolute $3B;  // Pin Change Interrupt Flag Register
  EIFR: byte absolute $3C;  // External Interrupt Flag Register
  EIMSK: byte absolute $3D;  // External Interrupt Mask Register
  GPIOR0: byte absolute $3E;  // General Purpose IO Register 0
  EECR: byte absolute $3F;  // EEPROM Control Register
  EEDR: byte absolute $40;  // EEPROM Data Register
  EEAR: word absolute $41;  // EEPROM Address Register  Bytes
  EEARL: byte absolute $41;  // EEPROM Address Register  Bytes
  EEARH: byte absolute $42;  // EEPROM Address Register  Bytes;
  GTCCR: byte absolute $43;  // General Timer/Counter Control Register
  TCCR0A: byte absolute $44;  // Timer/Counter0 Control Register
  TCCR0B: byte absolute $45;  // Timer/Counter0 Control Register
  TCNT0: byte absolute $46;  // Timer Counter 0
  OCR0A: byte absolute $47;  // Output compare Register A
  OCR0B: byte absolute $48;  // Output compare Register B
  GPIOR1: byte absolute $4A;  // General Purpose IO Register 1
  GPIOR2: byte absolute $4B;  // General Purpose IO Register 2
  SMCR: byte absolute $53;  // Sleep Mode Control Register
  MCUSR: byte absolute $54;  // MCU Status Register
  MCUCR: byte absolute $55;  // MCU Control Register
  SPMCSR: byte absolute $57;  // Store Program Memory Control Register
  SP: word absolute $5D;  // Stack Pointer 
  SPL: byte absolute $5D;  // Stack Pointer 
  SPH: byte absolute $5E;  // Stack Pointer ;
  SREG: byte absolute $5F;  // Status Register
  WDTCSR: byte absolute $60;  // Watchdog Timer Control Register
  WUTCSR: byte absolute $62;  // Wake-up Timer Control Register
  PRR0: byte absolute $64;  // Power Reduction Register 0
  FOSCCAL: byte absolute $66;  // Fast Oscillator Calibration Value
  PCICR: byte absolute $68;  // Pin Change Interrupt Control Register
  EICRA: byte absolute $69;  // External Interrupt Control Register
  PCMSK0: byte absolute $6B;  // Pin Change Enable Mask Register 0
  PCMSK1: byte absolute $6C;  // Pin Change Enable Mask Register 1
  TIMSK0: byte absolute $6E;  // Timer/Counter Interrupt Mask Register
  TIMSK1: byte absolute $6F;  // Timer/Counter Interrupt Mask Register
  VADC: word absolute $78;  // VADC Data Register  Bytes
  VADCL: byte absolute $78;  // VADC Data Register  Bytes
  VADCH: byte absolute $79;  // VADC Data Register  Bytes;
  VADCSR: byte absolute $7A;  // The VADC Control and Status register
  VADMUX: byte absolute $7C;  // The VADC multiplexer Selection Register
  DIDR0: byte absolute $7E;  // Digital Input Disable Register
  TCCR1B: byte absolute $81;  // Timer/Counter1 Control Register B
  TCNT1: word absolute $84;  // Timer Counter 1  Bytes
  TCNT1L: byte absolute $84;  // Timer Counter 1  Bytes
  TCNT1H: byte absolute $85;  // Timer Counter 1  Bytes;
  OCR1AL: byte absolute $88;  // Output Compare Register 1A Low byte
  OCR1AH: byte absolute $89;  // Output Compare Register 1A High byte
  TWBR: byte absolute $B8;  // TWI Bit Rate register
  TWSR: byte absolute $B9;  // TWI Status Register
  TWAR: byte absolute $BA;  // TWI (Slave) Address register
  TWDR: byte absolute $BB;  // TWI Data register
  TWCR: byte absolute $BC;  // TWI Control Register
  TWAMR: byte absolute $BD;  // TWI (Slave) Address Mask Register
  TWBCSR: byte absolute $BE;  // TWI Bus Control and Status Register
  CCSR: byte absolute $C0;  // Clock Control and Status Register
  BGCCR: byte absolute $D0;  // Bandgap Calibration Register
  BGCRR: byte absolute $D1;  // Bandgap Calibration of Resistor Ladder
  CADAC0: byte absolute $E0;  // ADC Accumulate Current
  CADAC1: byte absolute $E1;  // ADC Accumulate Current
  CADAC2: byte absolute $E2;  // ADC Accumulate Current
  CADAC3: byte absolute $E3;  // ADC Accumulate Current
  CADCSRA: byte absolute $E4;  // CC-ADC Control and Status Register A
  CADCSRB: byte absolute $E5;  // CC-ADC Control and Status Register B
  CADRCC: byte absolute $E6;  // CC-ADC Regular Charge Current
  CADRDC: byte absolute $E7;  // CC-ADC Regular Discharge Current
  CADIC: word absolute $E8;  // CC-ADC Instantaneous Current
  CADICL: byte absolute $E8;  // CC-ADC Instantaneous Current
  CADICH: byte absolute $E9;  // CC-ADC Instantaneous Current;
  FCSR: byte absolute $F0;
  CBCR: byte absolute $F1;  // Cell Balancing Control Register
  BPIR: byte absolute $F2;  // Battery Protection Interrupt Register
  BPDUV: byte absolute $F3;  // Battery Protection Deep Under Voltage Register
  BPSCD: byte absolute $F4;  // Battery Protection Short-Circuit Detection Level Register
  BPOCD: byte absolute $F5;  // Battery Protection OverCurrent Detection Level Register
  CBPTR: byte absolute $F6;  // Current Battery Protection Timing Register
  BPCR: byte absolute $F7;  // Battery Protection Control Register
  BPPLR: byte absolute $F8;  // Battery Protection Parameter Lock Register

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
  // Data Register, Port D
  PD0 = $00;  
  PD1 = $01;  
  // Timer/Counter Interrupt Flag register
  TOV0 = $00;  
  OCF0A = $01;  
  OCF0B = $02;  
  // Timer/Counter Interrupt Flag register
  TOV1 = $00;  
  OCF1A = $01;  
  // Pin Change Interrupt Flag Register
  PCIF0 = $00;  // Pin Change Interrupt Flags
  PCIF1 = $01;  // Pin Change Interrupt Flags
  // External Interrupt Flag Register
  INTF0 = $00;  // External Interrupt Flags
  INTF1 = $01;  // External Interrupt Flags
  INTF2 = $02;  // External Interrupt Flags
  INTF3 = $03;  // External Interrupt Flags
  // External Interrupt Mask Register
  INT0 = $00;  // External Interrupt Request 1 Enable
  INT1 = $01;  // External Interrupt Request 1 Enable
  INT2 = $02;  // External Interrupt Request 1 Enable
  INT3 = $03;  // External Interrupt Request 1 Enable
  // EEPROM Control Register
  EERE = $00;  
  EEPE = $01;  
  EEMPE = $02;  
  EERIE = $03;  
  EEPM0 = $04;  // EEPROM Programming Mode Bits
  EEPM1 = $05;  // EEPROM Programming Mode Bits
  // General Timer/Counter Control Register
  PSRSYNC = $00;  
  TSM = $07;  
  // Timer/Counter0 Control Register
  WGM00 = $00;  // Clock Select0 bits
  WGM01 = $01;  // Clock Select0 bits
  COM0B0 = $04;
  COM0B1 = $05;
  COM0A0 = $06;  // Force Output Compare
  COM0A1 = $07;  // Force Output Compare
  // Timer/Counter0 Control Register
  CS00 = $00;  // Clock Select0 bits
  CS01 = $01;  // Clock Select0 bits
  CS02 = $02;  // Clock Select0 bits
  WGM02 = $03;  
  FOC0B = $06;  
  FOC0A = $07;  
  // Output compare Register A
  OCR0A0 = $00;
  OCR0A1 = $01;
  OCR0A2 = $02;
  OCR0A3 = $03;
  OCR0A4 = $04;
  OCR0A5 = $05;
  OCR0A6 = $06;
  OCR0A7 = $07;
  // Output compare Register B
  OCR0B0 = $00;
  OCR0B1 = $01;
  OCR0B2 = $02;
  OCR0B3 = $03;
  OCR0B4 = $04;
  OCR0B5 = $05;
  OCR0B6 = $06;
  OCR0B7 = $07;
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
  // Wake-up Timer Control Register
  WUTP0 = $00;  // Wake-up Timer Prescaler Bits
  WUTP1 = $01;  // Wake-up Timer Prescaler Bits
  WUTP2 = $02;  // Wake-up Timer Prescaler Bits
  WUTE = $03;  
  WUTR = $04;  
  WUTCF = $05;  
  WUTIE = $06;  
  WUTIF = $07;  
  // Power Reduction Register 0
  PRVADC = $00;  
  PRTIM0 = $01;  
  PRTIM1 = $02;  
  PRTWI = $03;  
  // Pin Change Interrupt Control Register
  PCIE0 = $00;  // Pin Change Interrupt Enables
  PCIE1 = $01;  // Pin Change Interrupt Enables
  // External Interrupt Control Register
  ISC00 = $00;  // External Interrupt Sense Control 0 Bits
  ISC01 = $01;  // External Interrupt Sense Control 0 Bits
  ISC10 = $02;  // External Interrupt Sense Control 1 Bits
  ISC11 = $03;  // External Interrupt Sense Control 1 Bits
  ISC20 = $04;  // External Interrupt Sense Control 2 Bits
  ISC21 = $05;  // External Interrupt Sense Control 2 Bits
  ISC30 = $06;  // External Interrupt Sense Control 3 Bits
  ISC31 = $07;  // External Interrupt Sense Control 3 Bits
  // Timer/Counter Interrupt Mask Register
  TOIE0 = $00;  
  OCIE0A = $01;  
  OCIE0B = $02;  
  // Timer/Counter Interrupt Mask Register
  TOIE1 = $00;  
  OCIE1A = $01;  
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
  // Timer/Counter1 Control Register B
  CS10 = $00;  // Clock Select1 bits
  CS11 = $01;  // Clock Select1 bits
  CS12 = $02;  // Clock Select1 bits
  CTC1 = $03;  
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
  // TWI Bus Control and Status Register
  TWBCIP = $00;  
  TWBDT0 = $01;  // TWI Bus Disconnect Time-out Period
  TWBDT1 = $02;  // TWI Bus Disconnect Time-out Period
  TWBCIE = $06;  
  TWBCIF = $07;  
  // Clock Control and Status Register
  ACS = $00;  
  XOE = $01;  
  // Bandgap Calibration Register
  BGCC0 = $00;  // BG Calibration of PTAT Current Bits
  BGCC1 = $01;  // BG Calibration of PTAT Current Bits
  BGCC2 = $02;  // BG Calibration of PTAT Current Bits
  BGCC3 = $03;  // BG Calibration of PTAT Current Bits
  BGCC4 = $04;  // BG Calibration of PTAT Current Bits
  BGCC5 = $05;  // BG Calibration of PTAT Current Bits
  BGD = $07;  
  // CC-ADC Control and Status Register A
  CADSE = $00;  
  CADSI0 = $01;  // The CADSI bits determine the current sampling interval for the Regular Current detection in Power-down mode. The actual settings remain to be determined.
  CADSI1 = $02;  // The CADSI bits determine the current sampling interval for the Regular Current detection in Power-down mode. The actual settings remain to be determined.
  CADAS0 = $03;  // CC_ADC Accumulate Current Select Bits
  CADAS1 = $04;  // CC_ADC Accumulate Current Select Bits
  CADUB = $05;  
  CADEN = $07;  
  // CC-ADC Control and Status Register B
  CADICIF = $00;  
  CADRCIF = $01;  
  CADACIF = $02;  
  CADICIE = $04;  
  CADRCIE = $05;  
  CADACIE = $06;  
  PFD = $00;  
  CFE = $01;  
  DFE = $02;  
  CPS = $03;  
  PWMOPC = $04;  
  PWMOC = $05;  
  // Cell Balancing Control Register
  CBE1 = $00;  // Cell Balancing Enables
  CBE2 = $01;  // Cell Balancing Enables
  CBE3 = $02;  // Cell Balancing Enables
  CBE4 = $03;  // Cell Balancing Enables
  // Battery Protection Interrupt Register
  SCIE = $00;  
  DOCIE = $01;  
  COCIE = $02;  
  DUVIE = $03;  
  SCIF = $04;  
  DOCIF = $05;  
  COCIF = $06;  
  DUVIF = $07;  
  // Battery Protection Deep Under Voltage Register
  DUDL0 = $00;
  DUDL1 = $01;
  DUDL2 = $02;
  DUDL3 = $03;
  DUVT0 = $04;
  DUVT1 = $05;
  // Battery Protection Short-Circuit Detection Level Register
  SCDL0 = $00;
  SCDL1 = $01;
  SCDL2 = $02;
  SCDL3 = $03;
  // Battery Protection OverCurrent Detection Level Register
  CCDL0 = $00;
  CCDL1 = $01;
  CCDL2 = $02;
  CCDL3 = $03;
  DCDL0 = $04;
  DCDL1 = $05;
  DCDL2 = $06;
  DCDL3 = $07;
  // Current Battery Protection Timing Register
  OCPT0 = $00;
  OCPT1 = $01;
  OCPT2 = $02;
  OCPT3 = $03;
  SCPT0 = $04;
  SCPT1 = $05;
  SCPT2 = $06;
  SCPT3 = $07;
  // Battery Protection Control Register
  CCD = $00;  
  DCD = $01;  
  SCD = $02;  
  DUVD = $03;  
  // Battery Protection Parameter Lock Register
  BPPL = $00;  
  BPPLE = $01;  


implementation

{$i avrcommon.inc}

procedure BPINT_ISR; external name 'BPINT_ISR'; // Interrupt 1 Battery Protection Interrupt
procedure INT0_ISR; external name 'INT0_ISR'; // Interrupt 2 External Interrupt Request 0
procedure INT1_ISR; external name 'INT1_ISR'; // Interrupt 3 External Interrupt Request 1
procedure INT2_ISR; external name 'INT2_ISR'; // Interrupt 4 External Interrupt Request 2
procedure INT3_ISR; external name 'INT3_ISR'; // Interrupt 5 External Interrupt Request 3
procedure PCINT0_ISR; external name 'PCINT0_ISR'; // Interrupt 6 Pin Change Interrupt 0
procedure PCINT1_ISR; external name 'PCINT1_ISR'; // Interrupt 7 Pin Change Interrupt 1
procedure WDT_ISR; external name 'WDT_ISR'; // Interrupt 8 Watchdog Timeout Interrupt
procedure WAKE_UP_ISR; external name 'WAKE_UP_ISR'; // Interrupt 9 Wakeup timer overflow
procedure TIM1_COMP_ISR; external name 'TIM1_COMP_ISR'; // Interrupt 10 Timer/Counter 1 Compare Match
procedure TIM1_OVF_ISR; external name 'TIM1_OVF_ISR'; // Interrupt 11 Timer/Counter 1 Overflow
procedure TIM0_COMPA_ISR; external name 'TIM0_COMPA_ISR'; // Interrupt 12 Timer/Counter0 Compare A Match
procedure TIM0_COMPB_ISR; external name 'TIM0_COMPB_ISR'; // Interrupt 13 Timer/Counter0 Compare B Match
procedure TIM0_OVF_ISR; external name 'TIM0_OVF_ISR'; // Interrupt 14 Timer/Counter0 Overflow
procedure TWI_BUS_CD_ISR; external name 'TWI_BUS_CD_ISR'; // Interrupt 15 Two-Wire Bus Connect/Disconnect
procedure TWI_ISR; external name 'TWI_ISR'; // Interrupt 16 Two-Wire Serial Interface
procedure VADC_ISR; external name 'VADC_ISR'; // Interrupt 17 Voltage ADC Conversion Complete
procedure CCADC_CONV_ISR; external name 'CCADC_CONV_ISR'; // Interrupt 18 Coulomb Counter ADC Conversion Complete
procedure CCADC_REG_CUR_ISR; external name 'CCADC_REG_CUR_ISR'; // Interrupt 19 Coloumb Counter ADC Regular Current
procedure CCADC_ACC_ISR; external name 'CCADC_ACC_ISR'; // Interrupt 20 Coloumb Counter ADC Accumulator
procedure EE_READY_ISR; external name 'EE_READY_ISR'; // Interrupt 21 EEPROM Ready
procedure SPM_READY_ISR; external name 'SPM_READY_ISR'; // Interrupt 22 Store Program Memory Ready

procedure _FPC_start; assembler; nostackframe;
label
  _start;
asm
  .init
  .globl _start

  jmp _start
  jmp BPINT_ISR
  jmp INT0_ISR
  jmp INT1_ISR
  jmp INT2_ISR
  jmp INT3_ISR
  jmp PCINT0_ISR
  jmp PCINT1_ISR
  jmp WDT_ISR
  jmp WAKE_UP_ISR
  jmp TIM1_COMP_ISR
  jmp TIM1_OVF_ISR
  jmp TIM0_COMPA_ISR
  jmp TIM0_COMPB_ISR
  jmp TIM0_OVF_ISR
  jmp TWI_BUS_CD_ISR
  jmp TWI_ISR
  jmp VADC_ISR
  jmp CCADC_CONV_ISR
  jmp CCADC_REG_CUR_ISR
  jmp CCADC_ACC_ISR
  jmp EE_READY_ISR
  jmp SPM_READY_ISR

  {$i start.inc}

  .weak BPINT_ISR
  .weak INT0_ISR
  .weak INT1_ISR
  .weak INT2_ISR
  .weak INT3_ISR
  .weak PCINT0_ISR
  .weak PCINT1_ISR
  .weak WDT_ISR
  .weak WAKE_UP_ISR
  .weak TIM1_COMP_ISR
  .weak TIM1_OVF_ISR
  .weak TIM0_COMPA_ISR
  .weak TIM0_COMPB_ISR
  .weak TIM0_OVF_ISR
  .weak TWI_BUS_CD_ISR
  .weak TWI_ISR
  .weak VADC_ISR
  .weak CCADC_CONV_ISR
  .weak CCADC_REG_CUR_ISR
  .weak CCADC_ACC_ISR
  .weak EE_READY_ISR
  .weak SPM_READY_ISR

  .set BPINT_ISR, Default_IRQ_handler
  .set INT0_ISR, Default_IRQ_handler
  .set INT1_ISR, Default_IRQ_handler
  .set INT2_ISR, Default_IRQ_handler
  .set INT3_ISR, Default_IRQ_handler
  .set PCINT0_ISR, Default_IRQ_handler
  .set PCINT1_ISR, Default_IRQ_handler
  .set WDT_ISR, Default_IRQ_handler
  .set WAKE_UP_ISR, Default_IRQ_handler
  .set TIM1_COMP_ISR, Default_IRQ_handler
  .set TIM1_OVF_ISR, Default_IRQ_handler
  .set TIM0_COMPA_ISR, Default_IRQ_handler
  .set TIM0_COMPB_ISR, Default_IRQ_handler
  .set TIM0_OVF_ISR, Default_IRQ_handler
  .set TWI_BUS_CD_ISR, Default_IRQ_handler
  .set TWI_ISR, Default_IRQ_handler
  .set VADC_ISR, Default_IRQ_handler
  .set CCADC_CONV_ISR, Default_IRQ_handler
  .set CCADC_REG_CUR_ISR, Default_IRQ_handler
  .set CCADC_ACC_ISR, Default_IRQ_handler
  .set EE_READY_ISR, Default_IRQ_handler
  .set SPM_READY_ISR, Default_IRQ_handler
end;

end.
