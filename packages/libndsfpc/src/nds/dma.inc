<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: dma.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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



<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef NDS_INTERFACE}
const
  DMA0_SRC       : pcuint32 = pointer($040000B0);
  DMA0_DEST      : pcuint32 = pointer($040000B4);
  DMA0_CR        : pcuint32 = pointer($040000B8);

  DMA1_SRC       : pcuint32 = pointer($040000BC);
  DMA1_DEST      : pcuint32 = pointer($040000C0);
  DMA1_CR        : pcuint32 = pointer($040000C4);

  DMA2_SRC       : pcuint32 = pointer($040000C8);
  DMA2_DEST      : pcuint32 = pointer($040000CC);
  DMA2_CR        : pcuint32 = pointer($040000D0);

  DMA3_SRC       : pcuint32 = pointer($040000D4);
  DMA3_DEST      : pcuint32 = pointer($040000D8);
  DMA3_CR        : pcuint32 = pointer($040000DC);
<<<<<<< HEAD
<<<<<<< HEAD

function DMA_SRC(n: cint): pcuint32; inline;
function DMA_DEST(n: cint): pcuint32; inline;
function DMA_CR(n: cint): pcuint32; inline;
{$ifdef ARM9}
function DMA_FILL(n: cint): pcuint32; inline;
{$endif ARM9}


=======
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION}
function DMA_SRC(n: cint): pcuint32; inline;
begin
  DMA_SRC := pcuint32($040000B0 + (n * 12));
end;

function DMA_DEST(n: cint): pcuint32; inline;
begin
  DMA_DEST := pcuint32($040000B4 + (n * 12));
end;

function DMA_CR(n: cint): pcuint32; inline;
begin
  DMA_CR := pcuint32($040000B8 + (n * 12));
end;
{$endif NDS_IMPLEMENTATION}

{$ifdef NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
// DMA control register contents
// The defaults are 16-bit, increment source/dest addresses, no irq
const
  DMA_ENABLE   = (1 shl 31); //BIT(31);
  DMA_BUSY     = (1 shl 31); //BIT(31);
  DMA_IRQ_REQ  = (1 shl 30); //BIT(30);

  DMA_START_NOW   = 0;
  DMA_START_CARD  = (5 shl 27);

{$ifdef ARM7}
  DMA_START_VBL = (1 shl 27); //BIT(27);
{$endif ARM7}

{$ifdef ARM9}
  DMA_START_HBL  = (1 shl 28); //BIT(28);
  DMA_START_VBL  = (1 shl 27); //BIT(27);
<<<<<<< HEAD
<<<<<<< HEAD
//  DMA_START_FIFO  = (7 shl 27);
  DMA_DISP_FIFO   = (4 shl 27);
{$endif ARM9}
DMA_START_FIFO  = (7 shl 27);

=======
  DMA_START_FIFO  = (7 shl 27);
  DMA_DISP_FIFO   = (4 shl 27);
{$endif ARM9}
>>>>>>> graemeg/fixes_2_2
=======
  DMA_START_FIFO  = (7 shl 27);
  DMA_DISP_FIFO   = (4 shl 27);
{$endif ARM9}
>>>>>>> origin/fixes_2_2

  DMA_16_BIT      = 0; 
  DMA_32_BIT      = (1 shl 26); //BIT(26);

  DMA_REPEAT      = (1 shl 25); //BIT(25);

  DMA_SRC_INC     = (0); 
  DMA_SRC_DEC     = (1 shl 23); //BIT(23);
  DMA_SRC_FIX     = (1 shl 24); //BIT(24); 

  DMA_DST_INC     = (0); 
  DMA_DST_DEC     = (1 shl 21); //BIT(21); 
  DMA_DST_FIX     = (1 shl 22); //BIT(22);
  DMA_DST_RESET   = (3 shl 21); 

  DMA_COPY_WORDS      = (DMA_ENABLE or DMA_32_BIT or DMA_START_NOW); 
  DMA_COPY_HALFWORDS  = (DMA_ENABLE or DMA_16_BIT or DMA_START_NOW); 
<<<<<<< HEAD
<<<<<<< HEAD
{ $ifdef ARM9}
  DMA_FIFO            = (DMA_ENABLE or DMA_32_BIT or DMA_DST_FIX or DMA_START_FIFO); 
{ $endif ARM9}

procedure dmaCopyWords(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline; 
procedure dmaCopyHalfWords(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline;
procedure dmaCopy(const source: pointer; dest: pointer; size: cuint32); inline;
procedure dmaCopyWordsAsynch(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline; 
procedure dmaCopyHalfWordsAsynch(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline;
procedure dmaCopyAsynch(const source: pointer; dest: pointer; size: cuint32); inline;
procedure dmaFillWords(value: cuint32; dest: pointer; size: cuint32); inline;
procedure dmaFillHalfWords(value: cuint16; dest: pointer; size: cuint32); inline;
function dmaBusy(channel: cuint8): cint; inline; 

=======
{$ifdef ARM9}
  DMA_FIFO            = (DMA_ENABLE or DMA_32_BIT or DMA_DST_FIX or DMA_START_FIFO); 
{$endif ARM9}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef ARM9}
  DMA_FIFO            = (DMA_ENABLE or DMA_32_BIT or DMA_DST_FIX or DMA_START_FIFO); 
{$endif ARM9}
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}


{$ifdef NDS_IMPLEMENTATION}
<<<<<<< HEAD
<<<<<<< HEAD
function DMA_SRC(n: cint): pcuint32; inline;
begin
  DMA_SRC := pcuint32($040000B0 + (n * 12));
end;

function DMA_DEST(n: cint): pcuint32; inline;
begin
  DMA_DEST := pcuint32($040000B4 + (n * 12));
end;

function DMA_CR(n: cint): pcuint32; inline;
begin
  DMA_CR := pcuint32($040000B8 + (n * 12));
end;

{$ifdef ARM9}
function DMA_FILL(n: cint): pcuint32; inline;
begin
  DMA_FILL := pcuint32($040000E0 + (n * 4));
end;
{$endif ARM9}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

procedure dmaCopyWords(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline; 
begin
	DMA_SRC(channel)^ := cuint32(src);
	DMA_DEST(channel)^ := cuint32(dest);
	DMA_CR(channel)^ := DMA_COPY_WORDS or (size shr 2);
	while (DMA_CR(channel)^ and DMA_BUSY) <> 0 do;
end;

procedure dmaCopyHalfWords(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline;
begin
	DMA_SRC(channel)^ := cuint32(src);
	DMA_DEST(channel)^ := cuint32(dest);
	DMA_CR(channel)^ := DMA_COPY_HALFWORDS or (size shr 1);
	while (DMA_CR(channel)^ and DMA_BUSY) <> 0 do;
end;

procedure dmaCopy(const source: pointer; dest: pointer; size: cuint32); inline;
begin
	DMA_SRC(3)^ := cuint32(source);
	DMA_DEST(3)^ := cuint32(dest);
	DMA_CR(3)^ := DMA_COPY_HALFWORDS or (size shr 1);
	while (DMA_CR(3)^ and DMA_BUSY <> 0) do;
end;

procedure dmaCopyWordsAsynch(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline; 
begin
	DMA_SRC(channel)^ := cuint32(src);
	DMA_DEST(channel)^ := cuint32(dest);
	DMA_CR(channel)^ := DMA_COPY_WORDS or (size shr 2);
end;

procedure dmaCopyHalfWordsAsynch(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline;
begin
	DMA_SRC(channel)^ := cuint32(src);
	DMA_DEST(channel)^ := cuint32(dest);
	DMA_CR(channel)^ := DMA_COPY_HALFWORDS or (size shr 1);
end;

procedure dmaCopyAsynch(const source: pointer; dest: pointer; size: cuint32); inline;
begin
	DMA_SRC(3)^ := cuint32(source);
	DMA_DEST(3)^ := cuint32(dest);
	DMA_CR(3)^ := DMA_COPY_HALFWORDS or (size shr 1);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure dmaFillWords(value: cuint32; dest: pointer; size: cuint32); inline;
begin
  {$ifdef ARM7}	         
    pcuint32($027FFE04)^ := value;	         
    DMA_SRC(3)^ := $027FFE04;	 
  {$else ARM7}
    DMA_FILL(3)^ := value;
//    pcuint32(DMA_SRC(3)^) := pcuint32(DMA_FILL(3));
    DMA_SRC(3)^ := cuint32(DMA_FILL(3));
  {$endif ARM7}
=======
procedure dmaFillWords(const src: pointer; dest: pointer; size: cuint32); inline;
begin
	DMA_SRC(3)^ := cuint32(src);
>>>>>>> graemeg/fixes_2_2
=======
procedure dmaFillWords(const src: pointer; dest: pointer; size: cuint32); inline;
begin
	DMA_SRC(3)^ := cuint32(src);
>>>>>>> origin/fixes_2_2
	DMA_DEST(3)^ := cuint32(dest);
	DMA_CR(3)^ := DMA_SRC_FIX or DMA_COPY_WORDS or (size shr 2);
	while (DMA_CR(3)^ and DMA_BUSY) <> 0 do;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure dmaFillHalfWords(value: cuint16; dest: pointer; size: cuint32); inline;
begin
  {$ifdef ARM7}
    pcuint32($027FFE04)^ := value;	         
    DMA_SRC(3)^ := $027FFE04;	 
  {$else ARM7}
    DMA_FILL(3)^ := value;
//    pcuint32(DMA_SRC(3)^) := pcuint32(DMA_FILL(3));
    DMA_SRC(3)^ := cuint32(DMA_FILL(3));
  {$endif ARM7}
=======
procedure dmaFillHalfWords(const src: pointer; dest: pointer; size: cuint32); inline;
begin
	DMA_SRC(3)^ := cuint32(src);
>>>>>>> graemeg/fixes_2_2
=======
procedure dmaFillHalfWords(const src: pointer; dest: pointer; size: cuint32); inline;
begin
	DMA_SRC(3)^ := cuint32(src);
>>>>>>> origin/fixes_2_2
	DMA_DEST(3)^ := cuint32(dest);
	DMA_CR(3)^ := DMA_SRC_FIX or DMA_COPY_HALFWORDS or (size shr 1);
	while (DMA_CR(3)^ and DMA_BUSY) <> 0 do;
end;

function dmaBusy(channel: cuint8): cint; inline; 
begin
	dmaBusy := (DMA_CR(channel)^ and DMA_BUSY) shr 31;
end;
{$endif NDS_IMPLEMENTATION}
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2


{$ifdef NDS_INTERFACE}
function DMA_SRC(n: cint): pcuint32; inline;
function DMA_DEST(n: cint): pcuint32; inline;
function DMA_CR(n: cint): pcuint32; inline;
procedure dmaCopyWords(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline; 
procedure dmaCopyHalfWords(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline;
procedure dmaCopy(const source: pointer; dest: pointer; size: cuint32); inline;
procedure dmaCopyWordsAsynch(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline; 
procedure dmaCopyHalfWordsAsynch(channel: cuint8; const src: pointer; dest: pointer; size: cuint32); inline;
procedure dmaCopyAsynch(const source: pointer; dest: pointer; size: cuint32); inline;
procedure dmaFillWords(const src: pointer; dest: pointer; size: cuint32); inline;
procedure dmaFillHalfWords(const src: pointer; dest: pointer; size: cuint32); inline;
function dmaBusy(channel: cuint8): cint; inline; 
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
