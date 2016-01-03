{
    Copyright (c) 1999-2000 by Michael Van Canneyt, member of
    the Free Pascal development team

    gdbm database routines header translations.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$h+}

unit gdbm;



interface

const
  External_library='gdbm';

{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

{$PACKRECORDS C}

const
  { Parameters to gdbm_open for READERS, WRITERS, and WRITERS who
    can create the database.  }
GDBM_READER = 0;
GDBM_WRITER = 1;
GDBM_WRCREAT = 2;
GDBM_NEWDB = 3;
GDBM_FAST = $10;
GDBM_DOSYNC = $20;  // Was GDBM_SYNC, but conflicts with gdbm_sync !!
GDBM_NOLOCK = $40;
GDBM_INSERT = 0;
GDBM_REPLACE = 1;
GDBM_CACHESIZE = 1;
GDBM_FASTMODE = 2;
GDBM_SYNCMODE = 3;
GDBM_CENTFREE = 4;
GDBM_COALESCEBLKS = 5;
{ Error Codes }
GDBM_NO_ERROR = 0;
GDBM_MALLOC_ERROR = 1;
GDBM_BLOCK_SIZE_ERROR = 2;
GDBM_FILE_OPEN_ERROR = 3;
GDBM_FILE_WRITE_ERROR = 4;
GDBM_FILE_SEEK_ERROR = 5;
GDBM_FILE_READ_ERROR = 6;
GDBM_BAD_MAGIC_NUMBER = 7;
GDBM_EMPTY_DATABASE = 8;
GDBM_CANT_BE_READER = 9;
GDBM_CANT_BE_WRITER = 10;
GDBM_READER_CANT_DELETE = 11;
GDBM_READER_CANT_STORE = 12;
GDBM_READER_CANT_REORGANIZE = 13;
GDBM_UNKNOWN_UPDATE = 14;
GDBM_ITEM_NOT_FOUND = 15;
GDBM_REORGANIZE_FAILED = 16;
GDBM_CANNOT_REPLACE = 17;
GDBM_ILLEGAL_DATA = 18;
GDBM_OPT_ALREADY_SET = 19;
GDBM_OPT_ILLEGAL = 29;


type

  TDatum = record
        dptr : Pchar;
        dsize : longint;
     end;
  PDatum = ^TDatum;

  TGDBM_FILE = record
    dummy : array[0..9] of longint;
  end;
  PGDBM_FILE = ^TGDBM_FILE;

  TGDBMErrorCallBack = Procedure;



var
  gdbm_version : Pchar;cvar; external; {name 'gdbm_version' not accepted ??}

function gdbm_open(para1:Pchar; para2:longint; para3:longint; para4:longint; para5:TGDBMErrorCallBack ):PGDBM_FILE;cdecl;external External_library name 'gdbm_open';
procedure gdbm_close(para1:PGDBM_FILE);cdecl;external External_library name 'gdbm_close';
function gdbm_store(para1:PGDBM_FILE; para2:TDatum; para3:TDatum; para4:longint):longint;cdecl;external External_library name 'gdbm_store';
function gdbm_fetch(para1:PGDBM_FILE; para2:TDatum):TDatum;cdecl;external External_library name 'gdbm_fetch';
function gdbm_delete(para1:PGDBM_FILE; para2:TDatum):longint;cdecl;external External_library name 'gdbm_delete';
function gdbm_firstkey(para1:PGDBM_FILE):TDatum;cdecl;external External_library name 'gdbm_firstkey';
function gdbm_nextkey(para1:PGDBM_FILE; para2:TDatum):TDatum;cdecl;external External_library name 'gdbm_nextkey';
function gdbm_reorganize(para1:PGDBM_FILE):longint;cdecl;external External_library name 'gdbm_reorganize';
procedure gdbm_sync(para1:PGDBM_FILE);cdecl;external External_library name 'gdbm_sync';
function gdbm_exists(para1:PGDBM_FILE; para2:TDatum):longint;cdecl;external External_library name 'gdbm_exists';
function gdbm_setopt(para1:PGDBM_FILE; para2:longint; para3:Plongint; para4:longint):longint;cdecl;external External_library name 'gdbm_setopt';
function gdbm_fdesc(para1:PGDBM_FILE):longint;cdecl;external External_library name 'gdbm_fdesc';

{ Easy Pascal access routines }

function gdbm_open(Const para1:string; para2:longint; para3:longint; para4:longint; para5:TGDBMErrorCallBack ):PGDBM_FILE;
function gdbm_store(para1:PGDBM_FILE; Const para2:string; Const para3:string; para4:longint):Boolean;
function gdbm_fetch(para1:PGDBM_FILE; Const para2:string):string;
function gdbm_delete(para1:PGDBM_FILE; Const para2:string):boolean;
procedure gdbm_firstkey(para1:PGDBM_FILE; var key :string);
function gdbm_nextkey(para1:PGDBM_FILE; Const para2:string):string;
function gdbm_exists(para1:PGDBM_FILE; Const para2:string):boolean;


type
   gdbm_error = longint;
  var
     gdbm_errno : gdbm_error;cvar;external{ 'gdbm_errno'};

function gdbm_strerror(para1:gdbm_error):Pchar;cdecl;external External_library name 'gdbm_strerror';

implementation

function gdbm_open(Const para1:string; para2:longint; para3:longint; para4:longint; para5:TGDBMErrorCallBack ):PGDBM_FILE;

begin
  gdbm_open:=gdbm_open(@para1[1],para2,para3,para4,para5);
end;

procedure cfree (P : pointer);cdecl; external 'c' name 'free';

Function DatumToString(Key : TDatum) : String;

begin
  SetLength(DatumToString,Key.dsize);
  If key.Dsize>0 then
    Move(key.dptr^,DatumToString[1],key.dsize);
  if key.dptr<>Nil then
    cfree(Key.dptr);
end;

Function StringToDatum(Value : String) : TDatum;

begin
  StringToDatum.dptr:=@Value[1];
  StringToDatum.dsize:=Length(Value);
end;


function gdbm_store(para1:PGDBM_FILE; Const para2:string; Const para3:string; para4:longint):Boolean;

Var
  Data,Key : TDatum;

begin
  Data:=StringToDatum(Para3);
  Key:=StringToDatum(Para2);
  gdbm_store:=gdbm_store(para1,key,data,para4)=0;
end;

function gdbm_fetch(para1:PGDBM_FILE; Const para2:string):string;

begin
  gdbm_fetch:=DatumToString(gdbm_fetch(para1,StringToDatum(Para2)));
end;

function gdbm_delete(para1:PGDBM_FILE; Const para2:string):boolean;

begin
  gdbm_delete:=gdbm_delete(Para1,StringToDatum(para2))=0;
end;

Procedure gdbm_firstkey(para1:PGDBM_FILE; var key : String);

begin
  Key:=DatumToString(gdbm_firstkey(para1));
end;

function gdbm_nextkey(para1:PGDBM_FILE; Const Para2 :string):string;

begin
  gdbm_nextkey:=DatumToString(gdbm_nextkey(para1,StringToDatum(para2)));
end;

function gdbm_exists(para1:PGDBM_FILE; const para2:string):boolean;

begin
  gdbm_exists:=gdbm_exists(para1,StringToDatum(para2))<>0;
end;

end.
