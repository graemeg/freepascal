{
    Copyright (c) 2001 by Peter Vreman

    Includes the x86-64 dependent target units

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
{ This unit includes the x86-64 dependent target units. }
unit cputarg;
{$i fpcdefs.inc}

{ this unit adds all target relevant units, so this hint is not usefull }
{$WARN 5023 off : Unit "$1" not used in $2}

interface


implementation

    uses
      systems { prevent a syntax error when nothing is included }

{**************************************
             Targets
**************************************}

    {$ifndef NOTARGETLINUX}
      ,t_linux
    {$endif}
    {$ifndef NOTARGETFREEBSD}
      ,t_bsd
    {$endif}
    {$ifndef NOTARGETDARWIN}
      ,t_darwin
    {$endif}
    {$ifndef NOTARGETWIN}
      ,t_win
      ,win64unw
    {$endif}
    {$ifndef NOTARGETSUNOS}
      ,t_sunos
    {$endif}
    {$ifndef NOTARGETAROS}
      ,t_aros
    {$endif}
    {$ifndef NOTARGETHAIKU}
      ,t_haiku
    {$endif}
    {$ifndef NOTARGETEMBEDDED}
      ,t_embed
    {$endif}
    {$ifndef NOTARGETANDROID}
      ,t_android
    {$endif}

{**************************************
             Assemblers
**************************************}

    {$ifndef NOAGX86_64INT}
      ,agx86int
    {$endif}
    {$ifndef NOAGX86_64ATT}
      ,agx86att
    {$endif}
    {$ifndef NOAGX86_64NSM}
      ,agx86nsm
    {$endif}

      ,ogcoff
      ,ogelf
      ,cpuelf

{**************************************
        Assembler Readers
**************************************}

  {$ifndef NoRax64att}
       ,rax64att
  {$endif NoRax64att}
  {$ifndef NoRax64int}
       ,rax64int
  {$endif NoRax64int}

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
