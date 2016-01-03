(*
  gba_irq.pas 18/06/2006 4.20.45
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
unit gba_irq;
{$i def.inc}

interface

uses
  gba_types, gba_regs;

const
  IRQ_MASTER_ON  = 1;
  IRQ_MASTER_OFF = 0;
  IRQ_BIT_VBLANK  = (1 shl 0);
  IRQ_BIT_HBLANK  = (1 shl 1);
  IRQ_BIT_VCOUNT  = (1 shl 2);
  IRQ_BIT_TIMER0  = (1 shl 3);
  IRQ_BIT_TIMER1  = (1 shl 4);
  IRQ_BIT_TIMER2  = (1 shl 5);
  IRQ_BIT_TIMER3  = (1 shl 6);
  IRQ_BIT_SERIAL  = (1 shl 7);
  IRQ_BIT_DMA0    = (1 shl 8);
  IRQ_BIT_DMA1    = (1 shl 9);
  IRQ_BIT_DMA2    = (1 shl 10);
  IRQ_BIT_DMA3    = (1 shl 11);
  IRQ_BIT_KEYS    = (1 shl 12);
  IRQ_BIT_CART    = (1 shl 13);

procedure SetIRQ(x: dword);

implementation

procedure SetIRQ(x: dword);
begin
  REG_INTERRUPT^ := x;
end;


end.

