{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Sysutils unit for Win16

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$inline on}

unit sysutils;
interface

{$MODE objfpc}
{$MODESWITCH out}
{ force ansistrings }
{$H+}
{$modeswitch typehelpers}
{$modeswitch advancedrecords}

uses
  wintypes;

{$DEFINE HAS_SLEEP}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{ Include platform independent interface part }
{$i sysutilh.inc}


implementation

  uses
    sysconst,dos,winprocs;

{$DEFINE FPC_FEXPAND_UNC} (* UNC paths are supported *)
{$DEFINE FPC_FEXPAND_DRIVES} (* Full paths begin with drive specification *)

{$DEFINE executeprocuni} (* Only 1 byte version of ExecuteProcess is provided by the OS *)

{$DEFINE HASEXTRACTSHORTPATHNAME}
function ExtractShortPathName(Const FileName : RawByteString) : RawByteString;
var
  Regs: registers;
  c: array [0..255] of Char;
begin
  if LFNSupport then
    begin
      Regs.ax:=$7160;
      Regs.cx:=1;
      Regs.ds:=Seg(PChar(FileName)^);
      Regs.si:=Ofs(PChar(FileName)^);
      Regs.es:=Seg(c);
      Regs.di:=Ofs(c);
      MsDos(Regs);
      if (Regs.Flags and fCarry) <> 0 then
        Result:=''
      else
        Result:=StrPas(@c[0]);
    end
  else
    Result:=FileName;
end;

function ExtractShortPathName(Const FileName : UnicodeString) : UnicodeString;
begin
  Result:=ExtractShortPathName(ToSingleByteFileSystemEncodedFileName(FileName));
end;


{ Include platform independent implementation part }
{$i sysutils.inc}

type
  PFarChar=^Char;far;
  PPFarChar=^PFarChar;
var
  dos_env_count:smallint;external name '__dos_env_count';

{ This is implemented inside system unit }
function envp:PPFarChar;external name '__fpc_envp';

{ in protected mode, loading invalid values into segment registers causes an
  exception, so we use this function to initialize our Registers structure }
procedure ZeroSegRegs(var regs: Registers); inline;
begin
  regs.DS:=0;
  regs.ES:=0;
end;

{****************************************************************************
                              File Functions
****************************************************************************}

{ some internal constants }

const
   ofRead        = $0000;    { Open for reading }
   ofWrite       = $0001;    { Open for writing }
   ofReadWrite   = $0002;    { Open for reading/writing }
   faFail        = $0000;    { Fail if file does not exist }
   faCreate      = $0010;    { Create if file does not exist }
   faOpen        = $0001;    { Open if file exists }
   faOpenReplace = $0002;    { Clear if file exists }

Type
  PSearchrec = ^Searchrec;

{  Native OpenFile function.
   if return value <> 0 call failed.  }
function OpenFile(const FileName: RawByteString; var Handle: THandle; Mode, Action: word): longint;
var
   Regs: registers;
begin
  result := 0;
  Handle := UnusedHandle;
  if LFNSupport then
    begin
      Regs.ax := $716c;                    { Use LFN Open/Create API }
      Regs.dx := Action;                   { Action if file does/doesn't exist }
      Regs.si := Ofs(PChar(FileName)^);
      Regs.bx := $2000 + (Mode and $ff);   { File open mode }
    end
  else
    begin
      if (Action and $00f0) <> 0 then
        Regs.ax := $3c00                   { Map to Create/Replace API }
      else
        Regs.ax := $3d00 + (Mode and $ff); { Map to Open_Existing API }
      Regs.dx := Ofs(PChar(FileName)^);
    end;
  Regs.Ds := Seg(PChar(FileName)^);
  Regs.cx := $20;                          { Attributes }
  Regs.Es := 0;  { because protected mode }
  MsDos(Regs);
  if (Regs.Flags and fCarry) <> 0 then
    result := Regs.Ax
  else
    Handle := Regs.Ax;
end;


Function FileOpen (Const FileName : RawByteString; Mode : Integer) : THandle;
var
  e: integer;
Begin
  e := OpenFile(FileName, result, Mode, faOpen);
  if e <> 0 then
    result := -1;
end;


Function FileCreate (Const FileName : RawByteString) : THandle;
var
  e: integer;
begin
  e := OpenFile(FileName, result, ofReadWrite, faCreate or faOpenReplace);
  if e <> 0 then
    result := -1;
end;


Function FileCreate (Const FileName : RawByteString; ShareMode:integer; Rights : integer) : THandle;
begin
  FileCreate:=FileCreate(FileName);
end;


Function FileCreate (Const FileName : RawByteString; Rights:integer) : THandle;
begin
  FileCreate:=FileCreate(FileName);
end;


Function FileRead (Handle : THandle; Out Buffer; Count : longint) : Longint;
var
  regs     : registers;
  size,
  readsize : longint;
begin
  readsize:=0;
  while Count > 0 do
   begin
     if Count>65535 then
      size:=65535
     else
      size:=Count;
     regs.cx:=size;
     regs.dx:=Ofs(Buffer);
     regs.ds:=Seg(Buffer);
     regs.bx:=Handle;
     regs.ax:=$3f00;
     regs.es:=0;  { because protected mode }
     MsDos(regs);
     if (regs.flags and fCarry) <> 0 then
      begin
        Result:=-1;
        exit;
      end;
     inc(readsize,regs.ax);
     dec(Count,regs.ax);
     { stop when not the specified size is read }
     if regs.ax<size then
      break;
   end;
  Result:=readsize;
end;


Function FileWrite (Handle : THandle; const Buffer; Count : Longint) : Longint;
var
  regs      : registers;
  size,
  writesize : longint;
begin
  writesize:=0;
  while Count > 0 do
   begin
     if Count>65535 then
      size:=65535
     else
      size:=Count;
     regs.cx:=size;
     regs.dx:=Ofs(Buffer);
     regs.ds:=Seg(Buffer);
     regs.bx:=Handle;
     regs.ax:=$4000;
     regs.es:=0;  { because protected mode }
     MsDos(regs);
     if (regs.flags and fCarry) <> 0 then
      begin
        Result:=-1;
        exit;
      end;
     inc(writesize,regs.ax);
     dec(Count,regs.ax);
     { stop when not the specified size is written }
     if regs.ax<size then
      break;
   end;
  Result:=WriteSize;
end;


Function FileSeek (Handle : THandle; FOffset, Origin : Longint) : Longint;
var
  Regs: registers;
begin
  Regs.ah := $42;
  Regs.Al := Origin;
  Regs.dx := Lo(FOffset);
  Regs.cx := Hi(FOffset);
  Regs.bx := Handle;
  ZeroSegRegs(Regs);
  MsDos(Regs);
  if Regs.Flags and fCarry <> 0 then
     result := -1
  else begin
     LongRec(result).Lo := Regs.Ax;
     LongRec(result).Hi := Regs.Dx;
     end ;
end;


Function FileSeek (Handle : THandle; FOffset: Int64; Origin: {Integer}Longint) : Int64;
begin
  {$warning need to add 64bit call }
  FileSeek:=FileSeek(Handle,Longint(FOffset),Longint(Origin));
end;


Procedure FileClose (Handle : THandle);
var
  Regs: registers;
begin
  if Handle<=4 then
   exit;
  Regs.ax := $3e00;
  Regs.bx := Handle;
  ZeroSegRegs(Regs);
  MsDos(Regs);
end;


Function FileTruncate (Handle: THandle; Size: Int64) : boolean;
var
  regs : registers;
begin
  if Size > high (longint) then
   FileTruncate := false
  else
   begin
    FileSeek(Handle,Size,0);
    Regs.cx := 0;
    Regs.dx := 0{tb_offset};
    Regs.ds := 0{tb_segment};
    Regs.bx := Handle;
    Regs.ax:=$4000;
    Regs.es := 0;  { because protected mode }
    MsDos(Regs);
    FileTruncate:=(regs.flags and fCarry)=0;
   end;
end;


Function FileAge (Const FileName : RawByteString): Int64;
var Handle: longint;
begin
  Handle := FileOpen(FileName, 0);
  if Handle <> -1 then
   begin
     result := FileGetDate(Handle);
     FileClose(Handle);
   end
  else
   result := -1;
end;


function FileGetSymLinkTarget(const FileName: RawByteString; out SymLinkRec: TRawbyteSymLinkRec): Boolean;
begin
  Result := False;
end;


function FileExists (const FileName: RawByteString; FollowLink : Boolean): boolean;
var
  L: longint;
begin
  if FileName = '' then
   Result := false
  else
   begin
    L := FileGetAttr (FileName);
    Result := (L >= 0) and (L and (faDirectory or faVolumeID) = 0);
(* Neither VolumeIDs nor directories are files. *)
   end;
end;


Function DirectoryExists (Const Directory : RawByteString; FollowLink : Boolean) : Boolean;
Var
  Dir : RawByteString;
  drive : byte;
  FADir, StoredIORes : longint;
begin
  Dir:=Directory;
  if (length(dir)=2) and (dir[2]=':') and
     ((dir[1] in ['A'..'Z']) or (dir[1] in ['a'..'z'])) then
    begin
      { We want to test GetCurDir }
      if dir[1] in ['A'..'Z'] then
        drive:=ord(dir[1])-ord('A')+1
      else
        drive:=ord(dir[1])-ord('a')+1;
{$push}
{$I-}
      StoredIORes:=InOutRes;
      InOutRes:=0;
      GetDir(drive,dir);
      if InOutRes <> 0 then
        begin
          InOutRes:=StoredIORes;
          result:=false;
          exit;
        end;
    end;
{$pop}
  if (Length (Dir) > 1) and
    (Dir [Length (Dir)] in AllowDirectorySeparators) and
(* Do not remove '\' after ':' (root directory of a drive)
   or in '\\' (invalid path, possibly broken UNC path). *)
     not (Dir [Length (Dir) - 1] in (AllowDriveSeparators + AllowDirectorySeparators)) then
    dir:=copy(dir,1,length(dir)-1);
(* FileGetAttr returns -1 on error *)
  FADir := FileGetAttr (Dir);
  Result := (FADir <> -1) and
            ((FADir and faDirectory) = faDirectory);
end;


Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;

Var Sr : PSearchrec;

begin
  //!! Sr := New(PSearchRec);
  getmem(sr,sizeof(searchrec));
  Rslt.FindHandle := Sr;
  DOS.FindFirst(Path, Attr, Sr^);
  result := -DosError;
  if result = 0 then
   begin
     Rslt.Time := Sr^.Time;
     Rslt.Size := Sr^.Size;
     Rslt.Attr := Sr^.Attr;
     Rslt.ExcludeAttr := 0;
     Name := Sr^.Name;
   end ;
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;
var
  Sr: PSearchRec;
begin
  Sr := PSearchRec(Rslt.FindHandle);
  if Sr <> nil then
   begin
     DOS.FindNext(Sr^);
     result := -DosError;
     if result = 0 then
      begin
        Rslt.Time := Sr^.Time;
        Rslt.Size := Sr^.Size;
        Rslt.Attr := Sr^.Attr;
        Rslt.ExcludeAttr := 0;
        Name := Sr^.Name;
      end;
   end;
end;


Procedure InternalFindClose(var Handle: Pointer);
var
  Sr: PSearchRec;
begin
  Sr := PSearchRec(Handle);
  if Sr <> nil then
    begin
      //!! Dispose(Sr);
      // This call is non dummy if LFNSupport is true PM
      DOS.FindClose(SR^);
      freemem(sr,sizeof(searchrec));
    end;
  Handle := nil;
end;


Function FileGetDate (Handle : THandle) : Int64;
var
  Regs: registers;
begin
  //!! for win95 an alternative function is available.
  Regs.bx := Handle;
  Regs.ax := $5700;
  ZeroSegRegs(Regs);
  MsDos(Regs);
  if Regs.Flags and fCarry <> 0 then
   result := -1
  else
   Result:=(Regs.dx shl 16) or Regs.cx;
end;


Function FileSetDate (Handle : THandle; Age : Int64) : Longint;
var
  Regs: registers;
begin
  Regs.bx := Handle;
  Regs.ax := $5701;
  Regs.cx := Lo(dword(Age));
  Regs.dx := Hi(dword(Age));
  ZeroSegRegs(Regs);
  MsDos(Regs);
  if Regs.Flags and fCarry <> 0 then
   result := -Regs.Ax
  else
   result := 0;
end;


Function FileGetAttr (Const FileName : RawByteString) : Longint;
var
  Regs: registers;
begin
  Regs.dx := Ofs(PChar(FileName)^);
  Regs.Ds := Seg(PChar(FileName)^);
  Regs.Es := 0;  { because protected mode }
  if LFNSupport then
   begin
     Regs.Ax := $7143;
     Regs.Bx := 0;
   end
  else
   Regs.Ax := $4300;
  MsDos(Regs);
  if Regs.Flags and fCarry <> 0 then
    result := -1
  else
    result := Regs.Cx;
end;


Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
var
  Regs: registers;
begin
  Regs.dx := Ofs(PChar(FileName)^);
  Regs.Ds := Seg(PChar(FileName)^);
  Regs.Es := 0;  { because protected mode }
  if LFNSupport then
    begin
      Regs.Ax := $7143;
      Regs.Bx := 1;
    end
  else
    Regs.Ax := $4301;
  Regs.Cx := Attr;
  MsDos(Regs);
  if Regs.Flags and fCarry <> 0 then
   result := -Regs.Ax
  else
   result := 0;
end;


Function DeleteFile (Const FileName : RawByteString) : Boolean;
var
  Regs: registers;
begin
  Regs.dx := Ofs(PChar(FileName)^);
  Regs.Ds := Seg(PChar(FileName)^);
  Regs.Es := 0;  { because protected mode }
  if LFNSupport then
    Regs.ax := $7141
  else
    Regs.ax := $4100;
  Regs.si := 0;
  Regs.cx := 0;
  MsDos(Regs);
  result := (Regs.Flags and fCarry = 0);
end;


Function RenameFile (Const OldName, NewName : RawByteString) : Boolean;
var
  Regs: registers;
begin
  Regs.dx := Ofs(PChar(OldName)^);
  Regs.Ds := Seg(PChar(OldName)^);
  Regs.di := Ofs(PChar(NewName)^);
  Regs.Es := Seg(PChar(NewName)^);
  if LFNSupport then
    Regs.ax := $7156
  else
    Regs.ax := $5600;
  Regs.cx := $ff;
  MsDos(Regs);
  result := (Regs.Flags and fCarry = 0);
end;


{****************************************************************************
                              Disk Functions
****************************************************************************}

TYPE  ExtendedFat32FreeSpaceRec=packed Record
         RetSize           : WORD; { (ret) size of returned structure}
         Strucversion      : WORD; {(call) structure version (0000h)
                                    (ret) actual structure version (0000h)}
         SecPerClus,               {number of sectors per cluster}
         BytePerSec,               {number of bytes per sector}
         AvailClusters,            {number of available clusters}
         TotalClusters,            {total number of clusters on the drive}
         AvailPhysSect,            {physical sectors available on the drive}
         TotalPhysSect,            {total physical sectors on the drive}
         AvailAllocUnits,          {Available allocation units}
         TotalAllocUnits : DWORD;  {Total allocation units}
         Dummy,Dummy2    : DWORD;  {8 bytes reserved}
         END;

function do_diskdata(drive : byte; Free : BOOLEAN) : Int64;
VAR S    : String;
    Rec  : ExtendedFat32FreeSpaceRec;
    regs : registers;

  procedure OldDosDiskData;
  begin
   regs.dl:=drive;
   regs.ah:=$36;
   ZeroSegRegs(regs);
   msdos(regs);
   if regs.ax<>$FFFF then
    begin
     if Free then
      Do_DiskData:=int64(regs.ax)*regs.bx*regs.cx
     else
      Do_DiskData:=int64(regs.ax)*regs.cx*regs.dx;
    end
   else
    do_diskdata:=-1;
  end;

BEGIN
 if LFNSupport then
  begin
   S:='C:\'#0;
   if Drive=0 then
    begin
     GetDir(Drive,S);
     Setlength(S,4);
     S[4]:=#0;
    end
   else
    S[1]:=chr(Drive+64);
   Rec.Strucversion:=0;
   Rec.RetSize := 0;
   regs.dx:=Ofs(S[1]);
   regs.ds:=Seg(S[1]);
   regs.di:=Ofs(Rec);
   regs.es:=Seg(Rec);
   regs.cx:=Sizeof(ExtendedFat32FreeSpaceRec);
   regs.ax:=$7303;
   msdos(regs);
   if (regs.flags and fcarry) = 0 then {No error clausule in int except cf}
    begin
     if Rec.RetSize = 0 then (* Error - "FAT32" function not supported! *)
      OldDosDiskData
     else
      if Free then
       Do_DiskData:=int64(rec.AvailAllocUnits)*rec.SecPerClus*rec.BytePerSec
      else
       Do_DiskData:=int64(rec.TotalAllocUnits)*rec.SecPerClus*rec.BytePerSec;
    end
   else
    OldDosDiskData;
  end
 else
  OldDosDiskData;
end;


function diskfree(drive : byte) : int64;
begin
   diskfree:=Do_DiskData(drive,TRUE);
end;


function disksize(drive : byte) : int64;
begin
  disksize:=Do_DiskData(drive,false);
end;


{****************************************************************************
                              Time Functions
****************************************************************************}

Procedure GetLocalTime(var SystemTime: TSystemTime);
var
  Regs: Registers;
begin
  Regs.ah := $2C;
  ZeroSegRegs(Regs);
  MsDos(Regs);
  SystemTime.Hour := Regs.Ch;
  SystemTime.Minute := Regs.Cl;
  SystemTime.Second := Regs.Dh;
  SystemTime.MilliSecond := Regs.Dl*10;
  Regs.ah := $2A;
  MsDos(Regs);
  SystemTime.Year := Regs.Cx;
  SystemTime.Month := Regs.Dh;
  SystemTime.Day := Regs.Dl;
end ;


{****************************************************************************
                              Misc Functions
****************************************************************************}

procedure sysBeep;
begin
end;


{****************************************************************************
                              Locale Functions
****************************************************************************}

{  Codepage constants  }
const
   CP_US = 437;
   CP_MultiLingual = 850;
   CP_SlavicLatin2 = 852;
   CP_Turkish = 857;
   CP_Portugal = 860;
   CP_IceLand = 861;
   CP_Canada = 863;
   CP_NorwayDenmark = 865;

{  CountryInfo   }
type
   TCountryInfo = packed record
      InfoId: byte;
      case integer of
         1: ( Size: word;
              CountryId: word;
              CodePage: word;
              CountryInfo: array[0..33] of byte );
         2: ( UpperCaseTable: longint );
         4: ( FilenameUpperCaseTable: longint );
         5: ( FilecharacterTable: longint );
         6: ( CollatingTable: longint );
         7: ( DBCSLeadByteTable: longint );
   end ;


procedure GetExtendedCountryInfo(InfoId: integer; CodePage, CountryId: word; var CountryInfo: TCountryInfo);

Var Regs: Registers;

begin
  Regs.AH := $65;
  Regs.AL := InfoId;
  Regs.BX := CodePage;
  Regs.DX := CountryId;
  Regs.ES := {transfer_buffer div 16}Seg(CountryInfo);
  Regs.DI := {transfer_buffer and 15}Ofs(CountryInfo);
  Regs.CX := SizeOf(TCountryInfo);
  Regs.DS := 0;  { because protected mode }
  MsDos(Regs);
end;


procedure InitAnsi;
type
  PFarChar = ^char; far;
var
  CountryInfo: TCountryInfo; i: integer;
begin
  {  Fill table entries 0 to 127  }
  for i := 0 to 96 do
    UpperCaseTable[i] := chr(i);
  for i := 97 to 122 do
    UpperCaseTable[i] := chr(i - 32);
  for i := 123 to 127 do
    UpperCaseTable[i] := chr(i);
  for i := 0 to 64 do
    LowerCaseTable[i] := chr(i);
  for i := 65 to 90 do
    LowerCaseTable[i] := chr(i + 32);
  for i := 91 to 255 do
    LowerCaseTable[i] := chr(i);

  {  Get country and codepage info  }
  GetExtendedCountryInfo(1, $FFFF, $FFFF, CountryInfo);
  if CountryInfo.CodePage = 850 then
    begin
    { Special, known case }
    Move(CP850UCT, UpperCaseTable[128], 128);
    Move(CP850LCT, LowerCaseTable[128], 128);
    end
  else
    begin
    { this needs to be checked !!
    this is correct only if UpperCaseTable is
    and Offset:Segment word record (PM) }
    {  get the uppercase table from dosmemory  }
    GetExtendedCountryInfo(2, $FFFF, $FFFF, CountryInfo);
    for i := 128 to 255 do
       begin
       { TODO: do this properly }
       UpperCaseTable[i] := Chr(i){PFarChar(CountryInfo.UpperCaseTable)[i+(2-128)]};
       if UpperCaseTable[i] <> chr(i) then
          LowerCaseTable[ord(UpperCaseTable[i])] := chr(i);
       end;
    end;
end;


Procedure InitInternational;
begin
  InitInternationalGeneric;
  InitAnsi;
end;

function SysErrorMessage(ErrorCode: Integer): String;

begin
  Result:=Format(SUnknownErrorCode,[ErrorCode]);
end;

{****************************************************************************
                              Os utils
****************************************************************************}

{$if defined(FPC_MM_TINY) or defined(FPC_MM_SMALL) or defined(FPC_MM_MEDIUM)}
{ environment handling for near data memory models }

function far_strpas(p: pfarchar): string;
begin
  Result:='';
  if p<>nil then
    while p^<>#0 do
      begin
        Result:=Result+p^;
        Inc(p);
      end;
end;

Function small_FPCGetEnvVarFromP(EP : PPFarChar; EnvVar : String) : String;
var
  hp         : ppfarchar;
  lenvvar,hs : string;
  eqpos      : smallint;
begin
  lenvvar:=upcase(envvar);
  hp:=EP;
  Result:='';
  If (hp<>Nil) then
    while assigned(hp^) do
     begin
       hs:=far_strpas(hp^);
       eqpos:=pos('=',hs);
       if upcase(copy(hs,1,eqpos-1))=lenvvar then
        begin
          Result:=copy(hs,eqpos+1,length(hs)-eqpos);
          exit;
        end;
       inc(hp);
     end;
end;

Function small_FPCGetEnvStrFromP(EP : PPFarChar; Index : SmallInt) : String;
begin
  Result:='';
  while assigned(EP^) and (Index>1) do
    begin
      dec(Index);
      inc(EP);
    end;
  if Assigned(EP^) then
    Result:=far_strpas(EP^);
end;

Function GetEnvironmentVariable(Const EnvVar : String) : String;
begin
  Result:=small_FPCGetEnvVarFromP(envp,EnvVar);
end;

Function GetEnvironmentVariableCount : Integer;
begin
  Result:=dos_env_count;
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};
begin
  Result:=small_FPCGetEnvStrFromP(Envp,Index);
end;
{$else}
{ environment handling for far data memory models }
Function GetEnvironmentVariable(Const EnvVar : String) : String;
begin
  Result:=FPCGetEnvVarFromP(envp,EnvVar);
end;

Function GetEnvironmentVariableCount : Integer;
begin
  Result:=dos_env_count;
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};
begin
  Result:=FPCGetEnvStrFromP(Envp,Index);
end;
{$endif}


function ExecuteProcess(Const Path: RawByteString; Const ComLine: RawByteString;Flags:TExecuteFlags=[]):integer;
var
  e : EOSError;
  CommandLine: RawByteString;

begin
  dos.exec_ansistring(path,comline);

  if (Dos.DosError <> 0) then
    begin
      if ComLine <> '' then
       CommandLine := Path + ' ' + ComLine
      else
       CommandLine := Path;
      e:=EOSError.CreateFmt(SExecuteProcessFailed,[CommandLine,Dos.DosError]);
      e.ErrorCode:=Dos.DosError;
      raise e;
    end;
  Result := DosExitCode;
end;


function ExecuteProcess (const Path: RawByteString;
                                  const ComLine: array of RawByteString;Flags:TExecuteFlags=[]): integer;

var
  CommandLine: RawByteString;
  I: integer;

begin
  Commandline := '';
  for I := 0 to High (ComLine) do
   if Pos (' ', ComLine [I]) <> 0 then
    CommandLine := CommandLine + ' ' + '"' + ComLine [I] + '"'
   else
    CommandLine := CommandLine + ' ' + Comline [I];
  ExecuteProcess := ExecuteProcess (Path, CommandLine);
end;


{*************************************************************************
                                   Sleep
*************************************************************************}

procedure Sleep (MilliSeconds: Cardinal);
var
  ticks: LongInt;
  m: MSG;
begin
  ticks:=GetTickCount;
  repeat
    if PeekMessage(FarAddr(m),0,0,0,1) then
    begin
      TranslateMessage(FarAddr(m));
      DispatchMessage(FarAddr(m));
    end;
  until (GetTickCount-ticks)>=MilliSeconds;
end;

{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;       { Initialize exceptions. OS independent }
  InitInternational;    { Initialize internationalization settings }
  OnBeep:=@SysBeep;
Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
