<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}
const  
  // Card bus
  CARD_COMMAND   : pcuint8  = pointer($040001A8);
  REG_ROMCTRL    : pcuint32 = pointer($040001A4);
  REG_AUXSPICNT  : pcuint16 = pointer($040001A0);
  REG_AUXSPICNTH : pcuint8  = pointer($040001A1);
  REG_AUXSPIDATA : pcuint8  = pointer($040001A2);
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: card.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	Copyright (C) 2005
  		Jason Rogers (dovoto)
  		Dave Murphy (WinterMute)
  	
  	This software is provided 'as-is', without any express or implied
  	warranty.  In no event will the authors be held liable for any
  	damages arising from the use of this software.
  	
  	Permission is granted to anyone to use this software for any
  	purpose, including commercial applications, and to alter it and
  	redistribute it freely, subject to the following restrictions:
  	
  	1.	The origin of this software must not be misrepresented; you
  		must not claim that you wrote the original software. If you use
  		this software in a product, an acknowledgment in the product
  		documentation would be appreciated but is not required.
  	
  	2.	Altered source versions must be plainly marked as such, and
  		must not be misrepresented as being the original software.
  	
  	3.	This notice may not be removed or altered from any source
  		distribution.
  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)


{$ifdef NDS_INTERFACE}
const  
  // Card bus
  CARD_CR1       : pcuint16 = pointer($040001A0);
  CARD_CR1H      : pcuint8  = pointer($040001A1);
  CARD_EEPDATA   : pcuint8  = pointer($040001A2);
  CARD_CR2       : pcuint32 = pointer($040001A4);
  CARD_COMMAND   : pcuint8  = pointer($040001A8);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CARD_DATA_RD   : pcuint32 = pointer($04100010);

  CARD_1B0       : pcuint32 = pointer($040001B0);
  CARD_1B4       : pcuint32 = pointer($040001B4);
  CARD_1B8       : pcuint16 = pointer($040001B8);
  CARD_1BA       : pcuint16 = pointer($040001BA);

<<<<<<< HEAD
<<<<<<< HEAD
  CARD_CR1_ENABLE = $80;  // in byte 1, i.e. 0x8000
  CARD_CR1_IRQ    = $40;  // in byte 1, i.e. 0x4000

  // SPI EEPROM COMMANDS
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  SPI_EEPROM_WRSR   = $01;
  SPI_EEPROM_PP     = $02;	// Page Program
  SPI_EEPROM_READ   = $03;
  SPI_EEPROM_WRDI   = $04;  // Write disable
  SPI_EEPROM_RDSR   = $05;  // Read status register
  SPI_EEPROM_WREN   = $06;  // Write enable
  SPI_EEPROM_PW     = $0a;	// Page Write
  SPI_EEPROM_FAST   = $0b;	// Fast Read
  SPI_EEPROM_RDID   = $9f;
  SPI_EEPROM_RDP    = $ab;  // Release from deep power down
  SPI_EEPROM_DPD    = $b9;  // Deep power down


// REG_ROMCTRL:
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  EEPROM_WRSR = $01;
  EEPROM_WRDI = $04;
  EEPROM_RDSR = $05;
  EEPROM_WREN = $06;
  EEPROM_RDID = $9f;

// ROMCTRL register:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  CARD_ACTIVATE   = (1 shl 31);  // when writing, get the ball rolling
  CARD_WR         = (1 shl 30);
  CARD_nRESET     = (1 shl 29);  // value on the /reset pin (1 = high out, not a reset state, 0 = low out = in reset)
  CARD_SEC_LARGE  = (1 shl 28);  // when writing
  CARD_CLK_SLOW   = (1 shl 27);  // when writing

  CARD_SEC_CMD    = (1 shl 22);             // The command transfer will be hardware encrypted (KEY2)
  CARD_SEC_SEED   = (1 shl 15);           // Apply encryption (KEY2) seed to hardware registers
  CARD_SEC_EN     = (1 shl 14);           // Security enable
  CARD_SEC_DAT    = (1 shl 13);           // The data transfer will be hardware encrypted (KEY2)


function CARD_BLK_SIZE(n: cint): cint; inline;
function CARD_DELAY2(n: cint): cint; inline;
function CARD_DELAY1(n: cint): cint; inline;



const  
=======
=======
>>>>>>> origin/fixes_2_2

  CARD_CR1_ENABLE = $80;  // in byte 1, i.e. 0x8000
  CARD_CR1_IRQ    = $40;  // in byte 1, i.e. 0x4000


// CARD_CR2 register:
  CARD_ACTIVATE   = (1 shl 31);  // when writing, get the ball rolling
  // 1<<30
  CARD_nRESET     = (1 shl 29);  // value on the /reset pin (1 = high out, not a reset state, 0 = low out = in reset)
  CARD_28         = (1 shl 28);  // when writing
  CARD_27         = (1 shl 27);  // when writing
  CARD_26         = (1 shl 26); 
  CARD_22         = (1 shl 22); 
  CARD_19         = (1 shl 19); 
  CARD_ENCRYPTED  = (1 shl 14);  // when writing, this command should be encrypted
  CARD_13         = (1 shl 13);  // when writing
  CARD_4          = (1 shl 4);   // when writing
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  // 3 bits in b10..b8 indicate something
  // read bits
  CARD_BUSY       = (1 shl 31);  // when reading, still expecting incomming data?
  CARD_DATA_READY = (1 shl 23);  // when reading, CARD_DATA_RD or CARD_DATA has another word of data and is good to go

<<<<<<< HEAD
<<<<<<< HEAD
  // Card commands
  CARD_CMD_DUMMY          = $9F;
  CARD_CMD_HEADER_READ    = $00;
  CARD_CMD_HEADER_CHIPID  = $90;
  CARD_CMD_ACTIVATE_BF    = $3C; // Go into blowfish (KEY1) encryption mode
  CARD_CMD_ACTIVATE_SEC   = $40; // Go into hardware (KEY2) encryption mode
  CARD_CMD_SECURE_CHIPID  = $10;
  CARD_CMD_SECURE_READ    = $20;
  CARD_CMD_DISABLE_SEC    = $60; // Leave hardware (KEY2) encryption mode
  CARD_CMD_DATA_MODE      = $A0;
  CARD_CMD_DATA_READ      = $B7;
  CARD_CMD_DATA_CHIPID    = $B8;


  // REG_AUXSPICNT
  CARD_ENABLE     = (1 shl 15);
  CARD_SPI_ENABLE = (1 shl 13);
  CARD_SPI_BUSY   = (1 shl 7);
  CARD_SPI_HOLD   = (1 shl 6);

  CARD_SPICNTH_ENABLE = (1 shl 7);  // in byte 1, i.e. 0x8000
  CARD_SPICNTH_IRQ    = (1 shl 6);  // in byte 1, i.e. 0x4000




procedure cardWriteCommand(const command: pcuint8); cdecl; external;
procedure cardPolledTransfer(flags: cuint32;  destination: pcuint32;  length: cuint32; const command: pcuint8); cdecl; external; 
procedure cardStartTransfer(const command: pcuint8;  destination: pcuint32;  channel: cint;  flags: cuint32); cdecl; external;
function cardWriteAndRead(const command: pcuint8;  flags: cuint32): cuint32; cdecl; external;
procedure cardParamCommand(command: cuint8; parameter, flags: cuint32; destination: pcuint32; length: cuint32); cdecl; external;

procedure cardReadHeader(header: pcuint8); cdecl; external;
function cardReadID(flags: cuint32): cuint32; cdecl; external;
procedure cardReset(); cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure eepromWaitBusy(); inline;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

procedure cardReadEeprom(address: cuint32;  data: pcuint8;  length: cuint32;  addrtype: cuint32); cdecl; external; 
procedure cardWriteEeprom(address: cuint32;  data: pcuint8;  length: cuint32;  addrtype: cuint32); cdecl; external; 

function cardEepromReadID(): cuint32; cdecl; external;

function cardEepromCommand(command: cuint8): cuint8; cdecl; external;

=======
=======
>>>>>>> origin/fixes_2_2


procedure cardWriteCommand(const command: pcuint8); cdecl; external;

procedure cardPolledTransfer(flags: cuint32;  destination: pcuint32;  length: cuint32; const command: pcuint8); cdecl; external; 
procedure cardStartTransfer(const command: pcuint8;  destination: pcuint32;  channel: cint;  flags: cuint32); cdecl; external;
function cardWriteAndRead(const command: pcuint8;  flags: cuint32): cuint32; cdecl; external;
procedure cardRead00(address: cuint32;  destination: pcuint32;  length: cuint32;  flags: cuint32); cdecl; external; 
procedure cardReadHeader(header: pcuint8); cdecl; external;
function cardReadID(flags: cuint32): cint; cdecl; external;
procedure cardReadEeprom(address: cuint32;  data: pcuint8;  length: cuint32;  addrtype: cuint32); cdecl; external; 
procedure cardWriteEeprom(address: cuint32;  data: pcuint8;  length: cuint32;  addrtype: cuint32); cdecl; external; 

function cardEepromReadID(i: cuint8): cuint8; cdecl; external;
function cardEepromCommand(command: cuint8; address: cuint32): cuint8; cdecl; external;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function cardEepromGetType({void}): cint; cdecl; external;
function cardEepromGetSize(): cuint32; cdecl; external;
procedure cardEepromChipErase({void}); cdecl; external;
procedure cardEepromSectorErase(address: cuint32); cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD

{$endif NDS_INTERFACE}


{$ifdef NDS_IMPLEMENTATION}

function CARD_BLK_SIZE(n: cint): cint; inline;
begin
  result := (n and $7) shl 24;
end;

function CARD_DELAY2(n: cint): cint; inline;
begin
  result := (n and $3F) shl 16;
end;

function CARD_DELAY1(n: cint): cint; inline;
begin
  result := (n and $1FFF);
end;

procedure eepromWaitBusy(); inline;
begin
  while (REG_AUXSPICNT^ and CARD_SPI_BUSY) <> 0 do;
end;

{$endif NDS_IMPLEMENTATION}
=======
{$endif NDS_INTERFACE}

>>>>>>> graemeg/fixes_2_2
=======
{$endif NDS_INTERFACE}

>>>>>>> origin/fixes_2_2
