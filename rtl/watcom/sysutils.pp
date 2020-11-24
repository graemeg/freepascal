{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Sysutils unit for Watcom

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
{$modeswitch out}
{ force ansistrings }
{$H+}
{$modeswitch typehelpers}
{$modeswitch advancedrecords}

uses
  watcom,dos;

{$DEFINE HAS_SLEEP}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{$DEFINE executeprocuni} (* Only 1 byte version of ExecuteProcess is provided by the OS *)

{ Include platform independent interface part }
{$i sysutilh.inc}


implementation

  uses
    sysconst;

{$DEFINE FPC_FEXPAND_UNC} (* UNC paths are supported *)
{$DEFINE FPC_FEXPAND_DRIVES} (* Full paths begin with drive specification *)
{$DEFINE HAS_LOCALTIMEZONEOFFSET}

{ Include platform independent implementation part }
{$i sysutils.inc}


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

{  converts S to a pchar and copies it to the transfer-buffer.   }

procedure StringToTB(const S: RawByteString);
begin
  { include terminating null char }
  SysCopyToDos(longint(pointer(s)), Length(S) + 1);
end ;


{  Native OpenFile function.
   if return value <> 0 call failed.  }
function OpenFile(const FileName: RawByteString; var Handle: longint; Mode, Action: word): longint;
var
  Regs: registers;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  result := 0;
  Handle := 0;
  StringToTB(SystemFileName);
  if LFNSupport then
    Regs.Eax := $716c                       { Use LFN Open/Create API }
  else  { Check if Extended Open/Create API is safe to use }
    if lo(dosversion) < 7 then
      Regs.Eax := $3d00 + (Mode and $ff)    { For now, map to Open API }
    else
      Regs.Eax := $6c00;                    { Use Extended Open/Create API }
  if Regs.Ah = $3d then
    begin
      if (Action and $00f0) <> 0 then
        Regs.Eax := $3c00;                  { Map to Create/Replace API }
      Regs.Ds := tb_segment;
      Regs.Edx := tb_offset;
    end
  else  { LFN or Extended Open/Create API }
    begin
      Regs.Edx := Action;                   { Action if file exists/not exists }
      Regs.Ds := tb_segment;
      Regs.Esi := tb_offset;
      Regs.Ebx := $2000 + (Mode and $ff);   { file open mode }
    end;
  Regs.Ecx := $20;                          { Attributes }
  RealIntr($21, Regs);
  if (Regs.Flags and CarryFlag) <> 0 then
    result := Regs.Ax
  else
    Handle := Regs.Ax;
end;


Function FileOpen (Const FileName : rawbytestring; Mode : Integer) : Longint;
Var
  e: integer;
Begin
  e := OpenFile(FileName, result, Mode, faOpen);
  if e <> 0 then
    result := -1;
end;


Function FileCreate (Const FileName : RawByteString) : Longint;
var
  e: integer;
begin
  e := OpenFile(FileName, result, ofReadWrite, faCreate or faOpenReplace);
  if e <> 0 then
    result := -1;
end;


Function FileCreate (Const FileName : RawByteString; Rights:longint) : Longint;
begin
  FileCreate:=FileCreate(FileName);
end;

Function FileCreate (Const FileName : RawByteString; ShareMode:longint; Rights: Longint) : Longint;
begin
  FileCreate:=FileCreate(FileName);
end;


Function FileRead (Handle : Longint; Out Buffer; Count : longint) : Longint;
var
  regs     : registers;
  size,
  readsize : longint;
begin
  readsize:=0;
  while Count > 0 do
   begin
     if Count>tb_size then
      size:=tb_size
     else
      size:=Count;
     regs.realecx:=size;
     regs.realedx:=tb_offset;
     regs.realds:=tb_segment;
     regs.realebx:=Handle;
     regs.realeax:=$3f00;
     RealIntr($21,regs);
     if (regs.realflags and carryflag) <> 0 then
      begin
        Result:=-1;
        exit;
      end;
     syscopyfromdos(Longint(@Buffer)+readsize,lo(regs.realeax));
     inc(readsize,lo(regs.realeax));
     dec(Count,lo(regs.realeax));
     { stop when not the specified size is read }
     if lo(regs.realeax)<size then
      break;
   end;
  Result:=readsize;
end;


Function FileWrite (Handle : Longint; const Buffer; Count : Longint) : Longint;
var
  regs      : registers;
  size,
  writesize : longint;
begin
  writesize:=0;
  while Count > 0 do
   begin
     if Count>tb_size then
      size:=tb_size
     else
      size:=Count;
     syscopytodos(Longint(@Buffer)+writesize,size);
     regs.realecx:=size;
     regs.realedx:=tb_offset;
     regs.realds:=tb_segment;
     regs.realebx:=Handle;
     regs.realeax:=$4000;
     RealIntr($21,regs);
     if (regs.realflags and carryflag) <> 0 then
      begin
        Result:=-1;
        exit;
      end;
     inc(writesize,lo(regs.realeax));
     dec(Count,lo(regs.realeax));
     { stop when not the specified size is written }
     if lo(regs.realeax)<size then
      break;
   end;
  Result:=WriteSize;
end;


Function FileSeek (Handle, FOffset, Origin : Longint) : Longint;
var
  Regs: registers;
  res: dword;
begin
  Regs.Eax := $4200;
  Regs.Al := Origin;
  Regs.Edx := Lo(FOffset);
  Regs.Ecx := Hi(FOffset);
  Regs.Ebx := Handle;
  RealIntr($21, Regs);
  if Regs.Flags and CarryFlag <> 0 then
     result := -1
  else begin
     LongRec(res).Lo := Regs.Ax;
     LongRec(res).Hi := Regs.Dx;
     result:=res;
     end ;
end;


Function FileSeek (Handle : Longint; FOffset: Int64; Origin: Longint) : Int64;
begin
  {$warning need to add 64bit call }
  FileSeek:=FileSeek(Handle,Longint(FOffset),Longint(Origin));
end;


Procedure FileClose (Handle : Longint);
var
  Regs: registers;
begin
  if Handle<=4 then
   exit;
  Regs.Eax := $3e00;
  Regs.Ebx := Handle;
  RealIntr($21, Regs);
end;


Function FileTruncate (Handle: THandle; Size: Int64) : boolean;
var
  regs : trealregs;
begin
  if Size > high (longint) then
   FileTruncate := false
  else
   begin
    FileSeek(Handle,Size,0);
    Regs.realecx := 0;
    Regs.realedx := tb_offset;
    Regs.ds := tb_segment;
    Regs.ebx := Handle;
    Regs.eax:=$4000;
    RealIntr($21, Regs);
    FileTruncate:=(regs.realflags and carryflag)=0;
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
  { no need to convert to DefaultFileSystemEncoding, FileGetAttr will do that }
  if FileName = '' then
   Result := false
  else
   begin
    L := FileGetAttr (FileName);
    Result := (L >= 0) and (L and (faDirectory or faVolumeID) = 0);
(* Neither VolumeIDs nor directories are files. *)
   end;
end;


function DirectoryExists (const Directory: RawByteString; FollowLink : Boolean): boolean;
var
  L: longint;
begin
  { no need to convert to DefaultFileSystemEncoding, FileGetAttr will do that }
  if Directory = '' then
   Result := false
  else
   begin
    if ((Length (Directory) = 2) or
        (Length (Directory) = 3) and
        (Directory [3] in AllowDirectorySeparators)) and
       (Directory [2] in AllowDriveSeparators) and
       (UpCase (Directory [1]) in ['A'..'Z']) then
(* Checking attributes for 'x:' is not possible but for 'x:.' it is. *)
     L := FileGetAttr (Directory + '.')
    else if (Directory [Length (Directory)] in AllowDirectorySeparators) and
                                              (Length (Directory) > 1) and
(* Do not remove '\' in '\\' (invalid path, possibly broken UNC path). *)
      not (Directory [Length (Directory) - 1] in AllowDirectorySeparators) then
     L := FileGetAttr (Copy (Directory, 1, Length (Directory) - 1))
    else
     L := FileGetAttr (Directory);
    Result := (L > 0) and (L and faDirectory = faDirectory);
   end;
end;


Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;

Var Sr : PSearchrec;

begin
  //!! Sr := New(PSearchRec);
  getmem(sr,sizeof(searchrec));
  Rslt.FindHandle := longint(Sr);
  { FIXME: Dos version has shortstring interface -> discards encoding }
  DOS.FindFirst(Path, Attr, Sr^);
  result := -DosError;
  if result = 0 then
   begin
     Rslt.Time := Sr^.Time;
     Rslt.Size := Sr^.Size;
     Rslt.Attr := Sr^.Attr;
     Rslt.ExcludeAttr := 0;
     Name := Sr^.Name;
     SetCodePage(Name,DefaultFileSystemCodePage,False);
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
        SetCodePage(Name,DefaultFileSystemCodePage,False);
      end;
   end;
end;


Procedure InternalFindClose(var Handle: longint);
var
  Sr: PSearchRec;
begin
  Sr := PSearchRec(PtrUint(Handle));
  if Sr <> nil then
    begin
      //!! Dispose(Sr);
      // This call is non dummy if LFNSupport is true PM
      DOS.FindClose(SR^);
      freemem(sr,sizeof(searchrec));
    end;
  Handle := 0;
end;


Function FileGetDate (Handle : Longint) : Int64;
var
  Regs: registers;
  res: dword;
begin
  //!! for win95 an alternative function is available.
  Regs.Ebx := Handle;
  Regs.Eax := $5700;
  RealIntr($21, Regs);
  if Regs.Flags and CarryFlag <> 0 then
   result := -1
  else
   begin
     LongRec(res).Lo := Regs.cx;
     LongRec(res).Hi := Regs.dx;
     result := res;
   end ;
end;


Function FileSetDate (Handle: longint; Age: Int64) : Longint;
var
  Regs: registers;
begin
  Regs.Ebx := Handle;
  Regs.Eax := $5701;
  Regs.Ecx := Lo(dword(Age));
  Regs.Edx := Hi(dword(Age));
  RealIntr($21, Regs);
  if Regs.Flags and CarryFlag <> 0 then
   result := -Regs.Ax
  else
   result := 0;
end;


Function FileGetAttr (Const FileName : RawByteString) : Longint;
var
  Regs: registers;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  StringToTB(SystemFileName);
  Regs.Edx := tb_offset;
  Regs.Ds := tb_segment;
  if LFNSupport then
   begin
     Regs.Ax := $7143;
     Regs.Bx := 0;
   end
  else
   Regs.Ax := $4300;
  RealIntr($21, Regs);
  if Regs.Flags and CarryFlag <> 0 then
    result := -1
  else
    result := Regs.Cx;
end;


Function FileSetAttr (Const Filename : RawByteString; Attr: longint) : Longint;
var
  Regs: registers;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  StringToTB(SystemFileName);
  Regs.Edx := tb_offset;
  Regs.Ds := tb_segment;
  if LFNSupport then
    begin
      Regs.Ax := $7143;
      Regs.Bx := 1;
    end
  else
    Regs.Ax := $4301;
  Regs.Cx := Attr;
  RealIntr($21, Regs);
  if Regs.Flags and CarryFlag <> 0 then
   result := -Regs.Ax
  else
   result := 0;
end;


Function DeleteFile (Const FileName : RawByteString) : Boolean;
var
  Regs: registers;
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  StringToTB(SystemFileName);
  Regs.Edx := tb_offset;
  Regs.Ds := tb_segment;
  if LFNSupport then
    Regs.Eax := $7141
  else
    Regs.Eax := $4100;
  Regs.Esi := 0;
  Regs.Ecx := 0;
  RealIntr($21, Regs);
  result := (Regs.Flags and CarryFlag = 0);
end;


Function RenameFile (Const OldName, NewName : RawByteString) : Boolean;
var
  Regs: registers;
  OldSystemFileName, NewSystemFileName: RawByteString;
begin
  OldSystemFileName:=ToSingleByteFileSystemEncodedFileName(OldName);
  NewSystemFileName:=ToSingleByteFileSystemEncodedFileName(NewName);
  StringToTB(OldSystemFileName + #0 + NewSystemFileName);
  Regs.Edx := tb_offset;
  Regs.Ds := tb_segment;
  Regs.Edi := tb_offset + Length(OldSystemFileName) + 1;
  Regs.Es := tb_segment;
  if LFNSupport then
    Regs.Eax := $7156
  else
    Regs.Eax := $5600;
  Regs.Ecx := $ff;
  RealIntr($21, Regs);
  result := (Regs.Flags and CarryFlag = 0);
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
   dosmemput(tb_segment,tb_offset,Rec,SIZEOF(ExtendedFat32FreeSpaceRec));
   dosmemput(tb_segment,tb_offset+Sizeof(ExtendedFat32FreeSpaceRec)+1,S[1],4);
   regs.dx:=tb_offset+Sizeof(ExtendedFat32FreeSpaceRec)+1;
   regs.ds:=tb_segment;
   regs.di:=tb_offset;
   regs.es:=tb_segment;
   regs.cx:=Sizeof(ExtendedFat32FreeSpaceRec);
   regs.ax:=$7303;
   msdos(regs);
   if (regs.flags and fcarry) = 0 then {No error clausule in int except cf}
    begin
     copyfromdos(rec,Sizeof(ExtendedFat32FreeSpaceRec));
     if Rec.RetSize = 0 then (* Error - "FAT32" function not supported! *)
      OldDosDiskData
     else
      if Free then
       Do_DiskData:=int64(rec.AvailAllocUnits)*rec.SecPerClus*rec.BytePerSec
      else
       Do_DiskData:=int64(rec.TotalAllocUnits)*rec.SecPerClus*rec.BytePerSec;
    end
   else
    Do_DiskData:=-1;
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

{$I tzenv.inc}

Procedure GetLocalTime(var SystemTime: TSystemTime);
var
  Regs: Registers;
begin
  Regs.ah := $2C;
  RealIntr($21, Regs);
  SystemTime.Hour := Regs.Ch;
  SystemTime.Minute := Regs.Cl;
  SystemTime.Second := Regs.Dh;
  SystemTime.MilliSecond := Regs.Dl*10;
  Regs.ah := $2A;
  RealIntr($21, Regs);
  SystemTime.Year := Regs.Cx;
  SystemTime.Month := Regs.Dh;
  SystemTime.Day := Regs.Dl;
  SystemTime.DayOfWeek := Regs.Al;
end ;


{****************************************************************************
                              Misc Functions
****************************************************************************}

const
  BeepChars: array [1..2] of char = #7'$';

procedure sysBeep;
var
  Regs: Registers;
begin
  Regs.dx := Ofs (BeepChars);
  Regs.ah := 9;
  MsDos (Regs);
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
  Regs.ES := tb div 16;
  Regs.DI := tb and 15;
  Regs.CX := SizeOf(TCountryInfo);
  RealIntr($21, Regs);
  DosMemGet(tb div 16,
            tb and 15,
            CountryInfo, Regs.CX );
end;


procedure InitAnsi;
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
    DosMemGet(CountryInfo.UpperCaseTable shr 16, 2 + CountryInfo.UpperCaseTable and 65535, UpperCaseTable[128], 128);
    for i := 128 to 255 do
       begin
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

Function GetEnvironmentVariable(Const EnvVar : String) : String;

begin
  Result:=FPCGetEnvVarFromP(envp,EnvVar);
end;

Function GetEnvironmentVariableCount : Integer;

begin
  Result:=FPCCountEnvVar(EnvP);
end;

Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};

begin
  Result:=FPCGetEnvStrFromP(Envp,Index);
end;


function ExecuteProcess(Const Path: RawByteString; Const ComLine: RawByteString;Flags:TExecuteFlags=[]):integer;
var
  e : EOSError;
  CommandLine: RawByteString;

begin
  dos.exec(path,comline);

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
                                   Sleep (copied from crt.Delay)
*************************************************************************}

var
  DelayCnt : Longint;


procedure Delayloop;assembler;
asm
.LDelayLoop1:
        subl    $1,%eax
        jc      .LDelayLoop2
        cmpl    %fs:(%edi),%ebx
        je      .LDelayLoop1
.LDelayLoop2:
end;


procedure initdelay;assembler;
asm
        pushl %ebx
        pushl %edi
        { for some reason, using int $31/ax=$901 doesn't work here }
        { and interrupts are always disabled at this point when    }
        { running a program inside gdb(pas). Web bug 1345 (JM)     }
        sti
        movl    $0x46c,%edi
        movl    $-28,%edx
        movl    %fs:(%edi),%ebx
.LInitDel1:
        cmpl    %fs:(%edi),%ebx
        je      .LInitDel1
        movl    %fs:(%edi),%ebx
        movl    %edx,%eax
        call    DelayLoop

        notl    %eax
        xorl    %edx,%edx
        movl    $55,%ecx
        divl    %ecx
        movl    %eax,DelayCnt
        popl %edi
        popl %ebx
end;


procedure Sleep(MilliSeconds: Cardinal);assembler;
asm
        pushl %ebx
        pushl %edi
        movl  MilliSeconds,%ecx
        jecxz   .LDelay2
        movl    $0x400,%edi
        movl    DelayCnt,%edx
        movl    %fs:(%edi),%ebx
.LDelay1:
        movl    %edx,%eax
        call    DelayLoop
        loop    .LDelay1
.LDelay2:
        popl %edi
        popl %ebx
end;

{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;       { Initialize exceptions. OS independent }
  InitInternational;    { Initialize internationalization settings }
  InitDelay;
  OnBeep:=@SysBeep;
  InitTZ;
Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
