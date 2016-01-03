(*
  gba_sio.pas 18/06/2006 4.23.01
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
unit gba_sio;
{$i def.inc}
interface

uses
  gba_types;

const
  SIO_8BIT  = $0000;	// Normal 8-bit communication mode
  SIO_32BIT = $1000;	// Normal 32-bit communication mode
  SIO_MULTI = $2000;	// Multi-play communication mode
  SIO_UART  = $3000;	// UART communication mode
  SIO_IRQ   = $4000;	// Enable serial irq


//---------------------------------------------------------------------------------
// baud rate settings
//---------------------------------------------------------------------------------
  SIO_9600   = $0000;
  SIO_38400  = $0001;
  SIO_57600  = $0002;
  SIO_115200 = $0003;

  SIO_CLK_INT  = (1 shl 0);	// Select internal clock
  SIO_2MHZ_CLK = (1 shl 1);	// Select 2MHz clock
  SIO_RDY      = (1 shl 2);	// Opponent SO state
  SIO_SO_HIGH  = (1 shl 3);	// Our SO state
  SIO_START    = (1 shl 7);


//---------------------------------------------------------------------------------
// SIO modes set with REG_RCNT
//---------------------------------------------------------------------------------
  R_NORMAL  = $0000;
  R_MULTI   = $0000;
  R_UART    = $0000;
  R_GPIO    = $8000;
  R_JOYBUS  = $C000;

//---------------------------------------------------------------------------------
// General purpose mode control bits used with REG_RCNT
//---------------------------------------------------------------------------------
  GPIO_SC  = $0001;	// Data
  GPIO_SD	 = $0002;
  GPIO_SI	 = $0004;
  GPIO_SO	 = $0008;
  GPIO_SC_IO  = $0010;	// Select I/O
  GPIO_SD_IO  = $0020;
  GPIO_SI_IO  = $0040;
  GPIO_SO_IO  = $0080;
  GPIO_SC_INPUT  = $0000;	// Input setting
  GPIO_SD_INPUT  = $0000;
  GPIO_SI_INPUT  = $0000;
  GPIO_SO_INPUT  = $0000;
  GPIO_SC_OUTPUT = $0010;	// Output setting
  GPIO_SD_OUTPUT = $0020;
  GPIO_SI_OUTPUT = $0040;
  GPIO_SO_OUTPUT = $0080;


implementation 

end.
