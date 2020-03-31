{
    Copyright (c) 2001-2002 by Peter Vreman

    Includes the i386 dependent target units

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit cputarg;

{$i fpcdefs.inc}

interface


implementation

    uses
      systems { prevent a syntax error when nothing is included }

{**************************************
             Targets
**************************************}

    {$ifndef NOTARGETANDROID}
      ,t_android
    {$endif}
    {$ifndef NOTARGETLINUX}
      ,t_linux
    {$endif}
    {$ifndef NOTARGETBSD}
      ,t_bsd
    {$endif}
    {$ifndef NOTARGETDARWIN}
      ,t_darwin
    {$endif}
    {$ifndef NOTARGETSUNOS}
      ,t_sunos
    {$endif}
    {$ifndef NOTARGETEMX}
      ,t_emx
    {$endif}
    {$ifndef NOTARGETOS2}
      ,t_os2
    {$endif}
    {$ifndef NOTARGETWIN}
      ,t_win
    {$endif}
    {$ifndef NOTARGETNETWARE}
      ,t_nwm
    {$endif}
    {$ifndef NOTARGETNETWLIBC}
      ,t_nwl
    {$endif}
    {$ifndef NOTARGETGO32V2}
      ,t_go32v2
    {$endif}
    {$ifndef NOTARGETBEOS}
      ,t_beos
    {$endif}
    {$ifndef NOTARGETHAIKU}
      ,t_haiku
    {$endif}
    {$ifndef NOTARGETWDOSX}
      ,t_wdosx
    {$endif}
    {$ifndef NOTARGETWATCOM}
      ,t_watcom
    {$endif}
    {$ifndef NOTARGETSYMBIAN}
      ,t_symbian
    {$endif}
    {$ifndef NOTARGETNATIVENT}
      ,t_nativent
    {$endif}
    {$ifndef NOTARGETEMBEDDED}
      ,t_embed
    {$endif}
    {$ifndef NOTARGETAROS}
      ,t_aros
    {$endif}

{**************************************
             Assemblers
**************************************}

    {$ifndef NOAG386ATT}
      ,agx86att
    {$endif}
    {$ifndef NOAG386NSM}
      ,agx86nsm
    {$endif}
    {$ifndef NOAG386INT}
      ,agx86int
    {$endif}

      ,ogcoff
      ,ogelf
      ,cpuelf

{**************************************
        Assembler Readers
**************************************}

  {$ifndef NoRa386Int}
       ,ra386int
  {$endif NoRa386Int}
  {$ifndef NoRa386Att}
       ,ra386att
  {$endif NoRa386Att}

{**************************************
             Debuginfo
**************************************}

  {$ifndef NoCFIDwarf}
      ,cfidwarf
  {$endif NoCFIDwarf}
  {$ifndef NoDbgStabs}
      ,dbgstabs
  {$endif NoDbgStabs}
  {$ifndef NoDbgDwarf}
      ,dbgdwarf
  {$endif NoDbgDwarf}

      ;

end.
