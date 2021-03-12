{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
   This is just a temporary unit I made for some of
   my demos. I hope it will vanish in time.


   Added the define use_amiga_smartlink.
   13 Jan 2003.
   nils.sjoholm@mailbox.swipnet.se
}

unit amigautils;

interface

uses strings;

function ExtractFilePath(FileName: PChar): PChar;
function FileType(thefile :  PChar): Longint;
Function PathAndFile(Path,FName : PChar): PChar;
FUNCTION PathOf(Name : PChar): PChar;

Function LongToStr (I : Longint) : String;

implementation

type
    pDateStamp = ^tDateStamp;
    tDateStamp = record
        ds_Days         : Longint;      { Number of days since Jan. 1, 1978 }
        ds_Minute       : Longint;      { Number of minutes past midnight }
        ds_Tick         : Longint;      { Number of ticks past minute }
    end;

{$PACKRECORDS 4}
Type

{ Returned by Examine() and ExInfo(), must be on a 4 byte boundary }

    pFileInfoBlock = ^tFileInfoBlock;
    tFileInfoBlock = record
        fib_DiskKey      : Longint;
        fib_DirEntryType : Longint;
                        { Type of Directory. If < 0, then a plain file.
                          If > 0 a directory }
        fib_FileName     : Array [0..107] of Char;
                        { Null terminated. Max 30 chars used for now }
        fib_Protection   : Longint;
                        { bit mask of protection, rwxd are 3-0. }
        fib_EntryType    : Longint;
        fib_Size         : Longint;      { Number of bytes in file }
        fib_NumBlocks    : Longint;      { Number of blocks in file }
        fib_Date         : tDateStamp;   { Date file last changed }
        fib_Comment      : Array [0..79] of Char;
                        { Null terminated comment associated with file }
        fib_OwnerUID     : Word;
        fib_OwnerGID     : Word;
        fib_Reserved     : Array [0..31] of Char;
    end;

{$PACKRECORDS 2}

FUNCTION Examine(lock : LONGINT location 'd1'; fileInfoBlock : pFileInfoBlock location 'd2') : LongBool; syscall _DOSBase 102;
FUNCTION Lock(name : pCHAR location 'd1'; type_ : LONGINT location 'd2') : LONGINT; syscall _DOSBase 084;
PROCEDURE UnLock(lock : LONGINT location 'd1'); syscall _DOSBase 090;

FUNCTION PCharCopy(s: PChar; thepos , len : Longint): PChar;
VAR
    dummy : PChar;
BEGIN
    getmem(dummy,len+1);
    dummy := strlcopy(dummy,@s[thepos],len);
    PCharCopy := dummy;
END;


function ExtractFilePath(FileName: PChar): PChar;
var
  I: Longint;
begin
  I := strlen(FileName);
  while (I > 0) and not ((FileName[I] = '/') or (FileName[I] = ':')) do Dec(I);
  ExtractFilePath := PCharCopy(FileName, 0, I+1);
end;

function FileType(thefile :  PChar): Longint;
VAR
   fib  :  pFileInfoBlock;
   mylock : Longint;
   mytype : Longint;
begin
   mytype := 0;
   new(fib);
   mylock := Lock(thefile, -2);
   IF mylock <> 0 THEN begin
       IF Examine(mylock, fib) THEN begin
           mytype := fib^.fib_DirEntryType;
           UnLock(mylock);
       END;
    END;
    dispose(fib);
    FileType := mytype
END;

Function PathAndFile(Path,FName : PChar): PChar;
var
    LastChar : CHAR;
    Temparray : ARRAY [0..255] OF CHAR;
    Temp     : PChar;
BEGIN
    Temp := @Temparray;
    if strlen(Path) > 0 then begin
        strcopy(Temp, Path);
        LastChar := Temp[Pred(strlen(Temp))];
        if (LastChar <> '/') and (LastChar <> ':') then
            strcat(Temp, PChar('/'#0));
        if strlen(FName) > 0 then
            strcat(Temp,FName);
    end;
    if strlen(Temp) > 0 then begin
        PathAndFile := PCharCopy(Temp,0,Strlen(Temp));
    end else begin
        PathAndFile := nil;
    end;
end;

FUNCTION PathOf(Name : PChar): PChar;
begin
    PathOf := ExtractFilePath(Name);
end;

Function LongToStr (I : Longint) : String;
Var
    S : String;
begin
    Str (I,S);
    LongToStr:=S;
end;


end.
