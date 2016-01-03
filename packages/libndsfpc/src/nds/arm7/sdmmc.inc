{$ifdef NDS_INTERFACE}
const
  SDMMC_BASE = $04004800;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  DISPSTAT_CHK_VBLANK = (1 shl 0);
>>>>>>> graemeg/cpstrnew
=======
  DISPSTAT_CHK_VBLANK = (1 shl 0);
>>>>>>> graemeg/cpstrnew
=======
  DISPSTAT_CHK_VBLANK = (1 shl 0);
>>>>>>> graemeg/cpstrnew
=======
  DISPSTAT_CHK_VBLANK = (1 shl 0);
>>>>>>> origin/cpstrnew

  REG_SDSTATUS0	  = $1c;
  REG_SDSTATUS1	  = $1e;
  REG_SDRESET     = $e0;
  REG_SDCLKCTL    = $24; 
  REG_SDOPT       = $28;
  REG_SDCMD       = $00;
  REG_SDCMDARG    = $04;
  REG_SDCMDARG0   = $04;
  REG_SDCMDARG1  	= $06;
  REG_SDSTOP      = $08;
  REG_SDRESP      = $0c;
  
  REG_SDRESP0     = $0c;
  REG_SDRESP1     = $0e;
  REG_SDRESP2     = $10;
  REG_SDRESP3     = $12;
  REG_SDRESP4     = $14;
  REG_SDRESP5     = $16;
  REG_SDRESP6     = $18;
  REG_SDRESP7     = $1a;
  
  REG_SDBLKLEN    = $26;
  REG_SDBLKCOUNT  = $0a;
  REG_SDFIFO      = $30;
  
  TMIO_STAT_CMDRESPEND    = $00000001;
  TMIO_STAT_DATAEND       = $00000004;
  TMIO_STAT_CARD_REMOVE   = $00000008;
  TMIO_STAT_CARD_INSERT   = $00000010;
  TMIO_STAT_SIGSTATE      = $00000020;
  TMIO_STAT_WRPROTECT     = $00000080;
  TMIO_STAT_CARD_REMOVE_A = $00000100;
  TMIO_STAT_CARD_INSERT_A = $00000200;
  TMIO_STAT_SIGSTATE_A    = $00000400;
  TMIO_STAT_CMD_IDX_ERR   = $00010000;
  TMIO_STAT_CRCFAIL       = $00020000;
  TMIO_STAT_STOPBIT_ERR   = $00040000;
  TMIO_STAT_DATATIMEOUT   = $00080000;
  TMIO_STAT_RXOVERFLOW    = $00100000;
  TMIO_STAT_TXUNDERRUN    = $00200000;
  TMIO_STAT_CMDTIMEOUT    = $00400000;
  TMIO_STAT_RXRDY         = $01000000;
  TMIO_STAT_TXRQ          = $02000000;
  TMIO_STAT_ILL_FUNC      = $20000000;
  TMIO_STAT_CMD_BUSY      = $40000000;
  TMIO_STAT_ILL_ACCESS    = $80000000;

procedure sdmmc_controller_init(); cdecl; external;
procedure sdmmc_initirq(); cdecl; external;
function sdmmc_cardinserted(): cint; cdecl; external;

function sdmmc_sdcard_init(): cint; cdecl; external;
procedure sdmmc_sdcard_readsector(sector_no: cuint32; _out: pointer); cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure sdmmc_sdcard_readsectors(sector_no: cuint32; numsectors: cuint32; _out: pointer); cdecl; external;
procedure sdmmc_sdcard_writesector(sector_no: cuint32; _in: pointer); cdecl; external;
procedure sdmmc_sdcard_writesectors(sector_no: cuint32; numsectors: cuint32; _in: pointer); cdecl; external;
=======
procedure sdmmc_sdcard_readsectors(sector_no: cuint32; int numsectors: cint; _out: pointer); cdecl; external;
procedure sdmmc_sdcard_writesector(sector_no: cuint32; _in: pointer); cdecl; external;
procedure sdmmc_sdcard_writesectors(sector_no: cuint32; int numsectors: cint; _in: pointer); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure sdmmc_sdcard_readsectors(sector_no: cuint32; int numsectors: cint; _out: pointer); cdecl; external;
procedure sdmmc_sdcard_writesector(sector_no: cuint32; _in: pointer); cdecl; external;
procedure sdmmc_sdcard_writesectors(sector_no: cuint32; int numsectors: cint; _in: pointer); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure sdmmc_sdcard_readsectors(sector_no: cuint32; int numsectors: cint; _out: pointer); cdecl; external;
procedure sdmmc_sdcard_writesector(sector_no: cuint32; _in: pointer); cdecl; external;
procedure sdmmc_sdcard_writesectors(sector_no: cuint32; int numsectors: cint; _in: pointer); cdecl; external;
>>>>>>> graemeg/cpstrnew
=======
procedure sdmmc_sdcard_readsectors(sector_no: cuint32; int numsectors: cint; _out: pointer); cdecl; external;
procedure sdmmc_sdcard_writesector(sector_no: cuint32; _in: pointer); cdecl; external;
procedure sdmmc_sdcard_writesectors(sector_no: cuint32; int numsectors: cint; _in: pointer); cdecl; external;
>>>>>>> origin/cpstrnew

var
  sdmmc_curdevice: cint; cvar; external;
  sdmmc_cid: pcuint32; cvar; external;

function sdmmc_read16(reg: cuint16): cuint16; inline;
procedure sdmmc_write16(reg, val: cuint16); inline;
procedure sdmmc_mask16(reg, clear, _set: cuint16); inline;

{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function sdmmc_read16(reg: cuint16): cuint16; inline;
begin
  sdmmc_read16 := pcuint16(SDMMC_BASE + reg)^;
end;

procedure sdmmc_write16(reg, val: cuint16); inline;
begin
	pcuint16(SDMMC_BASE + reg)^ := val;
end;

procedure sdmmc_mask16(reg, clear, _set: cuint16); inline;
var
  val: cuint16;
begin
	val := sdmmc_read16(reg);
	val := val and not clear;
	val := val or _set;
	sdmmc_write16(reg, val);
end;
{$endif NDS_IMPLEMENTATION}

