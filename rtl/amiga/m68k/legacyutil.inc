{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 Karoly Balogh, Free Pascal Development team

    Amiga utility.library legacy (OS 1.x/2.x) support functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  This unit implements some of the utility.library functions for OS 1.x,
  where this library is missing, so the legacy OS support can be implemented
  with minimal changes to the normal system unit and common Amiga-like code

  Please note that this code doesn't aim to be API feature complete, just
  functional enough for the RTL code.
}

procedure Amiga2Date(seconds: Cardinal;
                     result : PClockData); public name '_fpc_amiga_amiga2date';
begin
{$warning Amiga2Date unimplemented!}
end;

function Date2Amiga(date: PClockData): Cardinal; public name '_fpc_amiga_date2amiga';
begin
{$warning Date2Amiga unimplemented!}
  Date2Amiga:=0;
end;
