{
    Free Pascal port of the Hermes C library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

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
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}


{$IFDEF CPU86}
  {$DEFINE I386_ASSEMBLER}
  {$IFNDEF win32}
    Const
      ExternalAsmPrefix='_';
  {$ELSE win32}
    Const
      ExternalAsmPrefix='';
  {$ENDIF win32}
{$ENDIF CPU86}

{$IFDEF VER1_0}
  {$IFDEF ENDIAN_LITTLE}
    {$DEFINE FPC_LITTLE_ENDIAN}
  {$ENDIF ENDIAN_LITTLE}
  {$IFDEF ENDIAN_BIG}
    {$DEFINE FPC_BIG_ENDIAN}
  {$ENDIF ENDIAN_BIG}
  Type
    PtrInt = LongInt;
    PtrUInt = Cardinal;
{$ENDIF VER1_0}

{$IFDEF FPC_LITTLE_ENDIAN}
  {$IFDEF FPC_BIG_ENDIAN}
    {$FATAL Both FPC_LITTLE_ENDIAN and FPC_BIG_ENDIAN defined?!}
  {$ENDIF FPC_BIG_ENDIAN}
{$ELSE FPC_LITTLE_ENDIAN}
  {$IFNDEF FPC_BIG_ENDIAN}
    {$FATAL Neither FPC_LITTLE_ENDIAN, nor FPC_BIG_ENDIAN defined?!}
  {$ENDIF FPC_BIG_ENDIAN}
{$ENDIF FPC_LITTLE_ENDIAN}

{$IFDEF FPC_LITTLE_ENDIAN}
  {$INFO FPC_LITTLE_ENDIAN}
{$ENDIF FPC_LITTLE_ENDIAN}
{$IFDEF FPC_BIG_ENDIAN}
  {$INFO FPC_BIG_ENDIAN}
{$ENDIF FPC_BIG_ENDIAN}
