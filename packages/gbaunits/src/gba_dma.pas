(*
  gba_dma.pas 18/06/2006 4.18.32
  ------------------------------------------------------------------------------
  This lib is a raw porting of libgba library for gba (you can find it at
  http://www.devkitpro.org).
  
  As this is a direct port from c, I'm pretty sure that something could not work
  as you expect. I am even more sure that this code could be written better, so 
  if you think that I have made some mistakes or you have some better 
  implemented functions, let me know [francky74 (at) gmail (dot) com]
  Enjoy!

  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
  
  Copyright (C) 2006  Francesco Lombardi
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
  ------------------------------------------------------------------------------
*)

unit gba_dma;
{$i def.inc}

interface

uses
  gba_types, gba_regs;

const
  DMA_SHIFT       = 16;

  // destination control
  _DMA_DST_INC    = ($0000 shl DMA_SHIFT);
  DMA_DST_DEC     = ($0020 shl DMA_SHIFT);
  DMA_DST_FIX     = ($0040 shl DMA_SHIFT);
  DMA_DST_RESET   = ($0060 shl DMA_SHIFT);
  
  // source control
  _DMA_SRC_INC    = ($0000 shl DMA_SHIFT);
  DMA_SRC_DEC     = ($0080 shl DMA_SHIFT);
  DMA_SRC_FIX     = ($0100 shl DMA_SHIFT);
  DMA_SRC_RESET   = ($0180 shl DMA_SHIFT);

  DMA_REPEAT      = ($0200 shl DMA_SHIFT);
  
  // chunks
  _DMA_16         = ($0000 shl DMA_SHIFT);
  DMA_32          = ($0400 shl DMA_SHIFT);

  // timing
  _DMA_AT_NOW     = ($0000 shl DMA_SHIFT);
  DMA_AT_VBLANK   = ($1000 shl DMA_SHIFT);
  DMA_AT_HBLANK   = ($2000 shl DMA_SHIFT);
  DMA_AT_FIFO     = ($3000 shl DMA_SHIFT);		// for sound ( DMA 1 & 2)
  DMA_AT_REFRESH  = ($3000 shl DMA_SHIFT);		// for video ( DMA 3)

  DMA_IRQ         = ($4000 shl DMA_SHIFT);
  DMA_ON          = ($8000 shl DMA_SHIFT);

  // I want it NOW!
  DMA_NOW         = DMA_ON or _DMA_AT_NOW;
  DMA_16NOW       = DMA_NOW or _DMA_16;
  DMA_32NOW       = DMA_NOW or DMA_32;
  
  // copies
  DMA_CPY16       = DMA_NOW or _DMA_16;
  DMA_CPY32       = DMA_NOW or DMA_32;
  
  // fills
  DMA_FILL16      = DMA_NOW or DMA_SRC_FIX or _DMA_16;
  DMA_FILL32      = DMA_NOW or DMA_SRC_FIX or DMA_32;

  DMA_TRANSFER_ON	    = (1 shl 15);
  DMA_TRANSFER_OFF    = (0 shl 15);

  DMA_INTR_ON         = (1 shl 14);
  DMA_INTR_OFF        = (0 shl 14);

  DMA_TIMING_NOW      = (0 shl 12);
  DMA_TIMING_VBLANK   = (1 shl 12);
  DMA_TIMING_HBLANK   = (2 shl 12);
  DMA_TIMING_FIFO     = (3 shl 12);
  DMA_TIMING_DRAWLINE = (3 shl 12);
  
  DMA_ROM_REQUEST     = (1 shl 11);

  DMA_SIZE_16         = (0 shl 10);
  DMA_SIZE_32         = (1 shl 10);
  
  DMA_REPEAT_ON       = (1 shl 9);
  DMA_REPEAT_OFF      = (0 shl 9);

  DMA_SAD_INC         = (0 shl 7);
  DMA_SAD_DEC         = (1 shl 7);
  DMA_SAD_FIX         = (2 shl 7);

  DMA_DAD_INC         = (0 shl 5);
  DMA_DAD_DEC         = (1 shl 5);
  DMA_DAD_FIX         = (2 shl 5);
  DMA_DAD_RESET       = (3 shl 5);

procedure DMA0Copy(sad: pword; dad: pword; size: dword; mode: dword); inline;
procedure DMA1Copy(sad: pword; dad: pword; size: dword; mode: dword); inline;
procedure DMA2Copy(sad: pword; dad: pword; size: dword; mode: dword); inline;
procedure DMA3Copy(sad: pword; dad: pword; size: dword; mode: dword); inline;

implementation

procedure DMA0Copy(sad: pword; dad: pword; size: dword; mode: dword); inline;
begin
  REG_DM0SAD^ := dword(sad);
  REG_DM0DAD^ := dword(dad);
  REG_DM0CNT^ := size or mode;
end; 

procedure DMA1Copy(sad: pword; dad: pword; size: dword; mode: dword); inline;
begin
  REG_DM1SAD^ := dword(sad);
  REG_DM1DAD^ := dword(dad);
  REG_DM1CNT^ := size or mode;
end; 

procedure DMA2Copy(sad: pword; dad: pword; size: dword; mode: dword); inline;
begin
  REG_DM2SAD^ := dword(sad);
  REG_DM2DAD^ := dword(dad);
  REG_DM2CNT^ := size or mode;
end; 

procedure DMA3Copy(sad: pword; dad: pword; size: dword; mode: dword); inline;
begin
  REG_DM3SAD^ := dword(sad);
  REG_DM3DAD^ := dword(dad);
  REG_DM3CNT^ := size or mode;
end; 

end.
