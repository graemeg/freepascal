{
    Copyright (c) 1999-2000 by Michael Van Canneyt

    Unit to read a CDROM disc TOC and get a list of CD Rom devices

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit cdrom;

{$mode objfpc}
{$h+}


interface

Type
  TTocEntry = Record
    min, sec, frame : Integer;
  end;
  PTocEntry = ^TTocEntry;

Function ReadCDTOC(Device : String; Var CDTOC : Array of TTocEntry) : Integer;
Function GetCDRomDevices(Var Devices : Array of string) : Integer;

Implementation

{$ifdef linux}
{$i cdromlin.inc}
{$else}
{$i cdromw32.inc}
{$endif}

end.
