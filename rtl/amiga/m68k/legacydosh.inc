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


{* dos.library *}

function CreateNewProc(tags: PTagItem): PProcess; external name '_fpc_amiga_createproc';
function NameFromLock(lock  : LongInt;
                      buffer: PChar;
                      len   : LongInt): LongBool; external name '_fpc_amiga_namefromlock';
function NameFromFH(fh    : BPTR;
                    buffer: PChar;
                    len   : LongInt): LongBool; external name '_fpc_amiga_namefromfh';
function ExamineFH(fh : BPTR;
                   fib: PFileInfoBlock): LongBool; external name '_fpc_amiga_examinefh';
function LockDosList(flags: Cardinal): PDosList; external name '_fpc_amiga_lockdoslist';
procedure UnLockDosList(flags: Cardinal); external name '_fpc_amiga_unlockdoslist';
function NextDosEntry(dlist: PDosList;
                      flags: Cardinal): PDosList; external name '_fpc_amiga_nextdosentry';
function MatchFirst(pat   : PChar;
                    anchor: PAnchorPath): LongInt; external name '_fpc_amiga_matchfirst';
function MatchNext(anchor: PAnchorPath): LongInt; external name '_fpc_amiga_matchnext';
procedure MatchEnd(anchor: PAnchorPath); external name '_fpc_amiga_matchend';
function SystemTagList(command: PChar;
                       tags   : PTagItem): LongInt; external name '_fpc_amiga_systemtaglist';
function GetVar(name  : PChar;
                buffer: PChar;
                size  : LongInt;
                flags : LongInt): LongInt; external name '_fpc_amiga_getvar';
function SetFileDate(name: PChar;
                     date: PDateStamp): LongBool; external name '_fpc_amiga_setfiledate';
function SetFileSize(fh  : LongInt;
                     pos : LongInt;
                     mode: LongInt): LongInt; external name '_fpc_amiga_setfilesize';
function GetProgramName(buf: PChar;
                        len: LongInt): LongBool; external name '_fpc_amiga_getprogramname';
function GetProgramDir: LongInt; external name '_fpc_amiga_getprogramdir';
function GetArgStr: PChar; external name '_fpc_amiga_getargstr';
