{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 Karoly Balogh, Free Pascal Development team

    Headers for Amiga legacy (OS 1.x/2.x) support functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{* utility.library *}

procedure Amiga2Date(seconds: Cardinal;
                     result : PClockData); external name '_fpc_amiga_amiga2date';
function Date2Amiga(date: PClockData): Cardinal; external name '_fpc_amiga_date2amiga';
