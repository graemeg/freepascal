unit esp8266;

interface

const
  //unit: Hz
  APB_CLK_FREQ = 80*1000000;
  UART_CLK_FREQ = APB_CLK_FREQ;
  //divided by 256
  TIMER_CLK_FREQ = (APB_CLK_FREQ shr 8);

  //Peripheral device base address
  PERIPHS_DPORT_BASEADDR = $3ff00000;
  PERIPHS_GPIO_BASEADDR  = $60000300;
  PERIPHS_TIMER_BASEDDR  = $60000600;
  PERIPHS_RTC_BASEADDR   = $60000700;
  PERIPHS_IO_MUX         = $60000800;

  //Interrupt remap control registers
  EDGE_INT_ENABLE_REG = (PERIPHS_DPORT_BASEADDR+$04);
  // TM1_EDGE_INT_ENABLE() = SET_PERI_REG_MASK(EDGE_INT_ENABLE_REG, $2);
  // TM1_EDGE_INT_DISABLE() = CLEAR_PERI_REG_MASK(EDGE_INT_ENABLE_REG, $2);

  //GPIO reg
  // GPIO_REG_READ(reg) = READ_PERI_REG(PERIPHS_GPIO_BASEADDR + reg);
  // GPIO_REG_WRITE(reg, val)                 = WRITE_PERI_REG(PERIPHS_GPIO_BASEADDR + reg, val);
  GPIO_OUT_ADDRESS = $00;
  GPIO_OUT_W1TS_ADDRESS = $04;
  GPIO_OUT_W1TC_ADDRESS = $08;

  GPIO_ENABLE_ADDRESS = $0c;
  GPIO_ENABLE_W1TS_ADDRESS = $10;
  GPIO_ENABLE_W1TC_ADDRESS = $14;
  GPIO_OUT_W1TC_DATA_MASK = $0000ffff;

  GPIO_IN_ADDRESS = $18;

  GPIO_STATUS_ADDRESS = $1c;
  GPIO_STATUS_W1TS_ADDRESS = $20;
  GPIO_STATUS_W1TC_ADDRESS = $24;
  GPIO_STATUS_INTERRUPT_MASK = $0000ffff;

  GPIO_RTC_CALIB_SYNC = PERIPHS_GPIO_BASEADDR+$6c;
  //first write to zero, then to one to start
  RTC_CALIB_START = $80000000;
  //max 8ms
  RTC_PERIOD_NUM_MASK = $3ff;
  GPIO_RTC_CALIB_VALUE = PERIPHS_GPIO_BASEADDR+$70;
  //after measure, flag to one, when start from zero to one, turn to zero
  RTC_CALIB_RDY_S = 31;
  RTC_CALIB_VALUE_MASK = $fffff;

  GPIO_PIN0_ADDRESS = $28;

  GPIO_ID_PIN0 = 0;
  // GPIO_ID_PIN(n) = (GPIO_ID_PIN0+(n));
  GPIO_LAST_REGISTER_ID = 15;
  GPIO_ID_NONE = $ffffffff;

  GPIO_PIN_COUNT = 16;

  GPIO_PIN_CONFIG_MSB = 12;
  GPIO_PIN_CONFIG_LSB = 11;
  GPIO_PIN_CONFIG_MASK = $00001800;
  // GPIO_PIN_CONFIG_GET(x) = (((x) and GPIO_PIN_CONFIG_MASK)  shr  GPIO_PIN_CONFIG_LSB);
  // GPIO_PIN_CONFIG_SET(x) = (((x)  shl  GPIO_PIN_CONFIG_LSB) and GPIO_PIN_CONFIG_MASK);

  GPIO_WAKEUP_ENABLE = 1;
  GPIO_WAKEUP_DISABLE = (not GPIO_WAKEUP_ENABLE);
  GPIO_PIN_WAKEUP_ENABLE_MSB = 10;
  GPIO_PIN_WAKEUP_ENABLE_LSB = 10;
  GPIO_PIN_WAKEUP_ENABLE_MASK = $00000400;
  // GPIO_PIN_WAKEUP_ENABLE_GET(x) = (((x) and GPIO_PIN_WAKEUP_ENABLE_MASK)  shr  GPIO_PIN_WAKEUP_ENABLE_LSB);
  // GPIO_PIN_WAKEUP_ENABLE_SET(x) = (((x)  shl  GPIO_PIN_WAKEUP_ENABLE_LSB) and GPIO_PIN_WAKEUP_ENABLE_MASK);

  GPIO_PIN_INT_TYPE_MASK = $380;
  GPIO_PIN_INT_TYPE_MSB = 9;
  GPIO_PIN_INT_TYPE_LSB = 7;
  // GPIO_PIN_INT_TYPE_GET(x) = (((x) and GPIO_PIN_INT_TYPE_MASK)  shr  GPIO_PIN_INT_TYPE_LSB);
  // GPIO_PIN_INT_TYPE_SET(x) = (((x)  shl  GPIO_PIN_INT_TYPE_LSB) and GPIO_PIN_INT_TYPE_MASK);

  GPIO_PAD_DRIVER_ENABLE = 1;
  GPIO_PAD_DRIVER_DISABLE = (not GPIO_PAD_DRIVER_ENABLE);
  GPIO_PIN_PAD_DRIVER_MSB = 2;
  GPIO_PIN_PAD_DRIVER_LSB = 2;
  GPIO_PIN_PAD_DRIVER_MASK = $00000004;
  // GPIO_PIN_PAD_DRIVER_GET(x) = (((x) and GPIO_PIN_PAD_DRIVER_MASK)  shr  GPIO_PIN_PAD_DRIVER_LSB);
  // GPIO_PIN_PAD_DRIVER_SET(x) = (((x)  shl  GPIO_PIN_PAD_DRIVER_LSB) and GPIO_PIN_PAD_DRIVER_MASK);

  GPIO_AS_PIN_SOURCE = 0;
  SIGMA_AS_PIN_SOURCE = (not GPIO_AS_PIN_SOURCE);
  GPIO_PIN_SOURCE_MSB = 0;
  GPIO_PIN_SOURCE_LSB = 0;
  GPIO_PIN_SOURCE_MASK = $00000001;
  // GPIO_PIN_SOURCE_GET(x) = (((x) and GPIO_PIN_SOURCE_MASK)  shr  GPIO_PIN_SOURCE_LSB);
  // GPIO_PIN_SOURCE_SET(x) = (((x)  shl  GPIO_PIN_SOURCE_LSB) and GPIO_PIN_SOURCE_MASK);

  // TIMER reg
  // RTC_REG_READ(addr) = READ_PERI_REG(PERIPHS_TIMER_BASEDDR + addr);
  // RTC_REG_WRITE(addr, val) =                WRITE_PERI_REG(PERIPHS_TIMER_BASEDDR + addr, val);
  // RTC_CLR_REG_MASK(reg, mask) =      CLEAR_PERI_REG_MASK(PERIPHS_TIMER_BASEDDR +reg, mask);
  // Returns the current time according to the timer timer.
  // NOW() = RTC_REG_READ(FRC2_COUNT_ADDRESS);

  //load initial_value to timer1
  FRC1_LOAD_ADDRESS = $00;

  //timer1's counter value(count from initial_value to 0)
  FRC1_COUNT_ADDRESS = $04;

  FRC1_CTRL_ADDRESS = $08;

  //clear timer1's interrupt when write this address
  FRC1_INT_ADDRESS = $0c;
  FRC1_INT_CLR_MASK = $00000001;

  //timer2's counter value(count from initial_value to 0)
  FRC2_COUNT_ADDRESS = $24;

  //RTC reg
  REG_RTC_BASE = PERIPHS_RTC_BASEADDR;

  RTC_STORE0 = (REG_RTC_BASE + $030);
  RTC_STORE1 = (REG_RTC_BASE + $034);
  RTC_STORE2 = (REG_RTC_BASE + $038);
  RTC_STORE3 = (REG_RTC_BASE + $03C);

  RTC_GPIO_OUT = (REG_RTC_BASE + $068);
  RTC_GPIO_ENABLE = (REG_RTC_BASE + $074);
  RTC_GPIO_IN_DATA = (REG_RTC_BASE + $08C);
  RTC_GPIO_CONF = (REG_RTC_BASE + $090);
  PAD_XPD_DCDC_CONF = (REG_RTC_BASE + $0A0);

  //PIN Mux reg
  PERIPHS_IO_MUX_FUNC = $13;
  PERIPHS_IO_MUX_FUNC_S = 4;
  PERIPHS_IO_MUX_PULLUP = $80;
  PERIPHS_IO_MUX_PULLUP2 = $40;
  PERIPHS_IO_MUX_SLEEP_PULLUP = $8;
  PERIPHS_IO_MUX_SLEEP_PULLUP2 = $4;
  PERIPHS_IO_MUX_SLEEP_OE = $2;
  PERIPHS_IO_MUX_OE = $1;

  PERIPHS_IO_MUX_CONF_U = (PERIPHS_IO_MUX + $00);
  SPI0_CLK_EQU_SYS_CLK = $100;
  SPI1_CLK_EQU_SYS_CLK = $200;
  PERIPHS_IO_MUX_MTDI_U = (PERIPHS_IO_MUX + $04);
  FUNC_GPIO12 = 3;
  PERIPHS_IO_MUX_MTCK_U = (PERIPHS_IO_MUX + $08);
  FUNC_GPIO13 = 3;
  PERIPHS_IO_MUX_MTMS_U = (PERIPHS_IO_MUX + $0C);
  FUNC_GPIO14 = 3;
  PERIPHS_IO_MUX_MTDO_U = (PERIPHS_IO_MUX + $10);
  FUNC_GPIO15 = 3;
  FUNC_U0RTS = 4;
  PERIPHS_IO_MUX_U0RXD_U = (PERIPHS_IO_MUX + $14);
  FUNC_GPIO3 = 3;
  PERIPHS_IO_MUX_U0TXD_U = (PERIPHS_IO_MUX + $18);
  FUNC_U0TXD = 0;
  FUNC_GPIO1 = 3;
  PERIPHS_IO_MUX_SD_CLK_U = (PERIPHS_IO_MUX + $1c);
  FUNC_SDCLK = 0;
  FUNC_SPICLK = 1;
  PERIPHS_IO_MUX_SD_DATA0_U = (PERIPHS_IO_MUX + $20);
  FUNC_SDDATA0 = 0;
  FUNC_SPIQ = 1;
  FUNC_U1TXD = 4;
  PERIPHS_IO_MUX_SD_DATA1_U = (PERIPHS_IO_MUX + $24);
  FUNC_SDDATA1 = 0;
  FUNC_SPID = 1;
  FUNC_U1RXD = 4;
  FUNC_SDDATA1_U1RXD = 7;
  PERIPHS_IO_MUX_SD_DATA2_U = (PERIPHS_IO_MUX + $28);
  FUNC_SDDATA2 = 0;
  FUNC_SPIHD = 1;
  FUNC_GPIO9 = 3;
  PERIPHS_IO_MUX_SD_DATA3_U = (PERIPHS_IO_MUX + $2c);
  FUNC_SDDATA3 = 0;
  FUNC_SPIWP = 1;
  FUNC_GPIO10 = 3;
  PERIPHS_IO_MUX_SD_CMD_U = (PERIPHS_IO_MUX + $30);
  FUNC_SDCMD = 0;
  FUNC_SPICS0 = 1;
  PERIPHS_IO_MUX_GPIO0_U = (PERIPHS_IO_MUX + $34);
  FUNC_GPIO0 = 0;
  PERIPHS_IO_MUX_GPIO2_U = (PERIPHS_IO_MUX + $38);
  FUNC_GPIO2 = 0;
  FUNC_U1TXD_BK = 2;
  FUNC_U0TXD_BK = 4;
  PERIPHS_IO_MUX_GPIO4_U = (PERIPHS_IO_MUX + $3C);
  FUNC_GPIO4 = 0;
  PERIPHS_IO_MUX_GPIO5_U = (PERIPHS_IO_MUX + $40);
  FUNC_GPIO5 = 0;

  // PIN_PULLUP_DIS(PIN_NAME) = CLEAR_PERI_REG_MASK(PIN_NAME, PERIPHS_IO_MUX_PULLUP);
  // PIN_PULLUP_EN(PIN_NAME) = SET_PERI_REG_MASK(PIN_NAME, PERIPHS_IO_MUX_PULLUP);

implementation

var
  _stack_top: record end; external name '_stack_top';
  _data: record end; external name '_data';
  _edata: record end; external name '_edata';
  _text_start: record end; external name '_text_start';
  _etext: record end; external name '_etext';
  _bss_start: record end; external name '_bss_start';
  _bss_end: record end; external name '_bss_end';

procedure Pascalmain; external name 'PASCALMAIN';

procedure HaltProc; assembler; nostackframe; public name'_haltproc';
asm
.Lloop:
  j .Lloop
end;

procedure Startup;
var
  psrc,pdst,pend: plongword;
begin
  // Copy .text
  psrc:=@_etext;
  pdst:=@_data;
  pend:=@_edata;
  while pdst<pend do
    begin
      pdst^:=psrc^;
      inc(pdst);
      inc(psrc);
    end;

  // Clear .bss
  pend:=@_bss_end;
  while pdst<pend do
    begin
      pdst^:=0;
      inc(pdst);
    end;
  
  PascalMain;
  Haltproc;
end;

procedure LowLevelStartup; assembler; nostackframe;
asm
  j .Lstart
  .align 4
.Lstack_ptr:
  .long _stack_top
  .align 4
.Lstart:
  l32r a1, .Lstack_ptr

  j Startup
end;

end.
