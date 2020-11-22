{

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Sysutils unit for EMX

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit sysutils;
interface

{$MODE objfpc}
{$MODESWITCH OUT}
{ force ansistrings }
{$H+}
{$modeswitch typehelpers}
{$modeswitch advancedrecords}

uses
 Dos;

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

{ Include platform independent implementation part }
{$i sysutils.inc}


{****************************************************************************
                        System (imported) calls
****************************************************************************}

(* "uses DosCalls" could not be used here due to type    *)
(* conflicts, so needed parts had to be redefined here). *)

type
        TFileStatus = object
        end;
        PFileStatus = ^TFileStatus;

        TFileStatus3 = object (TFileStatus)
            DateCreation,               {Date of file creation.}
            TimeCreation,               {Time of file creation.}
            DateLastAccess,             {Date of last access to file.}
            TimeLastAccess,             {Time of last access to file.}
            DateLastWrite,              {Date of last modification of file.}
            TimeLastWrite:word;         {Time of last modification of file.}
            FileSize,                   {Size of file.}
            FileAlloc:cardinal;         {Amount of space the file really
                                         occupies on disk.}
            AttrFile:cardinal;          {Attributes of file.}
        end;
        PFileStatus3=^TFileStatus3;

        TFileStatus4=object(TFileStatus3)
            cbList:cardinal;            {Length of entire EA set.}
        end;
        PFileStatus4=^TFileStatus4;

        TFileStatus3L = object (TFileStatus)
            DateCreation,               {Date of file creation.}
            TimeCreation,               {Time of file creation.}
            DateLastAccess,             {Date of last access to file.}
            TimeLastAccess,             {Time of last access to file.}
            DateLastWrite,              {Date of last modification of file.}
            TimeLastWrite:word;         {Time of last modification of file.}
            FileSize,                   {Size of file.}
            FileAlloc:int64;         {Amount of space the file really
                                         occupies on disk.}
            AttrFile:cardinal;          {Attributes of file.}
        end;
        PFileStatus3L=^TFileStatus3L;

        TFileStatus4L=object(TFileStatus3L)
            cbList:cardinal;            {Length of entire EA set.}
        end;
        PFileStatus4L=^TFileStatus4L;

        TFileFindBuf3=object(TFileStatus)
            NextEntryOffset: cardinal;  {Offset of next entry}
            DateCreation,               {Date of file creation.}
            TimeCreation,               {Time of file creation.}
            DateLastAccess,             {Date of last access to file.}
            TimeLastAccess,             {Time of last access to file.}
            DateLastWrite,              {Date of last modification of file.}
            TimeLastWrite:word;         {Time of last modification of file.}
            FileSize,                   {Size of file.}
            FileAlloc:cardinal;         {Amount of space the file really
                                         occupies on disk.}
            AttrFile:cardinal;          {Attributes of file.}
            Name:shortstring;           {Also possible to use as ASCIIZ.
                                         The byte following the last string
                                         character is always zero.}
        end;
        PFileFindBuf3=^TFileFindBuf3;

        TFileFindBuf4=object(TFileStatus)
            NextEntryOffset: cardinal;  {Offset of next entry}
            DateCreation,               {Date of file creation.}
            TimeCreation,               {Time of file creation.}
            DateLastAccess,             {Date of last access to file.}
            TimeLastAccess,             {Time of last access to file.}
            DateLastWrite,              {Date of last modification of file.}
            TimeLastWrite:word;         {Time of last modification of file.}
            FileSize,                   {Size of file.}
            FileAlloc:cardinal;         {Amount of space the file really
                                         occupies on disk.}
            AttrFile:cardinal;          {Attributes of file.}
            cbList:longint;             {Size of the file's extended attributes.}
            Name:shortstring;           {Also possible to use as ASCIIZ.
                                         The byte following the last string
                                         character is always zero.}
        end;
        PFileFindBuf4=^TFileFindBuf4;

        TFileFindBuf3L=object(TFileStatus)
            NextEntryOffset: cardinal;  {Offset of next entry}
            DateCreation,               {Date of file creation.}
            TimeCreation,               {Time of file creation.}
            DateLastAccess,             {Date of last access to file.}
            TimeLastAccess,             {Time of last access to file.}
            DateLastWrite,              {Date of last modification of file.}
            TimeLastWrite:word;         {Time of last modification of file.}
            FileSize,                   {Size of file.}
            FileAlloc:int64;            {Amount of space the file really
                                         occupies on disk.}
            AttrFile:cardinal;          {Attributes of file.}
            Name:shortstring;           {Also possible to use as ASCIIZ.
                                         The byte following the last string
                                         character is always zero.}
        end;
        PFileFindBuf3L=^TFileFindBuf3L;

        TFileFindBuf4L=object(TFileStatus)
            NextEntryOffset: cardinal;  {Offset of next entry}
            DateCreation,               {Date of file creation.}
            TimeCreation,               {Time of file creation.}
            DateLastAccess,             {Date of last access to file.}
            TimeLastAccess,             {Time of last access to file.}
            DateLastWrite,              {Date of last modification of file.}
            TimeLastWrite:word;         {Time of last modification of file.}
            FileSize,                   {Size of file.}
            FileAlloc:int64;            {Amount of space the file really
                                         occupies on disk.}
            AttrFile:cardinal;          {Attributes of file.}
            cbList:cardinal;            {Size of the file's extended attributes.}
            Name:shortstring;           {Also possible to use as ASCIIZ.
                                         The byte following the last string
                                         character is always zero.}
        end;
        PFileFindBuf4L=^TFileFindBuf4L;

 TFSInfo = record
            case word of
             1:
              (File_Sys_ID,
               Sectors_Per_Cluster,
               Total_Clusters,
               Free_Clusters: cardinal;
               Bytes_Per_Sector: word);
             2:                           {For date/time description,
                                           see file searching realted
                                           routines.}
              (Label_Date,                {Date when volume label was created.}
               Label_Time: word;          {Time when volume label was created.}
               VolumeLabel: ShortString); {Volume label. Can also be used
                                           as ASCIIZ, because the byte
                                           following the last character of
                                           the string is always zero.}
           end;
 PFSInfo = ^TFSInfo;

 TCountryCode=record
               Country,            {Country to query info about (0=current).}
               CodePage: cardinal; {Code page to query info about (0=current).}
              end;
 PCountryCode=^TCountryCode;

 TTimeFmt = (Clock12, Clock24);

 TCountryInfo=record
               Country, CodePage: cardinal;  {Country and codepage requested.}
               case byte of
                0:
                 (DateFormat: cardinal;     {1=ddmmyy 2=yymmdd 3=mmddyy}
                  CurrencyUnit: array [0..4] of char;
                  ThousandSeparator: char;  {Thousands separator.}
                  Zero1: byte;              {Always zero.}
                  DecimalSeparator: char;   {Decimals separator,}
                  Zero2: byte;
                  DateSeparator: char;      {Date separator.}
                  Zero3: byte;
                  TimeSeparator: char;      {Time separator.}
                  Zero4: byte;
                  CurrencyFormat,           {Bit field:
                                             Bit 0: 0=indicator before value
                                                    1=indicator after value
                                             Bit 1: 1=insert space after
                                                      indicator.
                                             Bit 2: 1=Ignore bit 0&1, replace
                                                      decimal separator with
                                                      indicator.}
                  DecimalPlace: byte;       {Number of decimal places used in
                                             currency indication.}
                  TimeFormat: TTimeFmt;     {12/24 hour.}
                  Reserve1: array [0..1] of word;
                  DataSeparator: char;      {Data list separator}
                  Zero5: byte;
                  Reserve2: array [0..4] of word);
                1:
                 (fsDateFmt: cardinal;      {1=ddmmyy 2=yymmdd 3=mmddyy}
                  szCurrency: array [0..4] of char;
                                            {null terminated currency symbol}
                  szThousandsSeparator: array [0..1] of char;
                                            {Thousands separator + #0}
                  szDecimal: array [0..1] of char;
                                            {Decimals separator + #0}
                  szDateSeparator: array [0..1] of char;
                                            {Date separator + #0}
                  szTimeSeparator: array [0..1] of char;
                                            {Time separator + #0}
                  fsCurrencyFmt,            {Bit field:
                                             Bit 0: 0=indicator before value
                                                    1=indicator after value
                                             Bit 1: 1=insert space after
                                                      indicator.
                                             Bit 2: 1=Ignore bit 0&1, replace
                                                      decimal separator with
                                                      indicator}
                  cDecimalPlace: byte;      {Number of decimal places used in
                                             currency indication}
                  fsTimeFmt: byte;          {0=12,1=24 hours}
                  abReserved1: array [0..1] of word;
                  szDataSeparator: array [0..1] of char;
                                            {Data list separator + #0}
                  abReserved2: array [0..4] of word);
              end;
 PCountryInfo=^TCountryInfo;

 TRequestData=record
               PID,                {ID of process that wrote element.}
               Data: cardinal;     {Information from process writing the data.}
              end;
 PRequestData=^TRequestData;

{Queue data structure for synchronously started sessions.}
 TChildInfo = record
  case boolean of
   false:
    (SessionID,
     Return: word);  {Return code from the child process.}
   true:
    (usSessionID,
     usReturn: word);     {Return code from the child process.}
 end;
 PChildInfo = ^TChildInfo;

 TStartData=record
  {Note: to omit some fields, use a length smaller than SizeOf(TStartData).}
  Length:word;                {Length, in bytes, of datastructure
                               (24/30/32/50/60).}
  Related:word;               {Independent/child session (0/1).}
  FgBg:word;                  {Foreground/background (0/1).}
  TraceOpt:word;              {No trace/trace this/trace all (0/1/2).}
  PgmTitle:PChar;             {Program title.}
  PgmName:PChar;              {Filename to program.}
  PgmInputs:PChar;            {Command parameters (nil allowed).}
  TermQ:PChar;                {System queue. (nil allowed).}
  Environment:PChar;          {Environment to pass (nil allowed).}
  InheritOpt:word;            {Inherit environment from shell/
                               inherit environment from parent (0/1).}
  SessionType:word;           {Auto/full screen/window/presentation
                               manager/full screen Dos/windowed Dos
                               (0/1/2/3/4/5/6/7).}
  Iconfile:PChar;             {Icon file to use (nil allowed).}
  PgmHandle:cardinal;         {0 or the program handle.}
  PgmControl:word;            {Bitfield describing initial state
                               of windowed sessions.}
  InitXPos,InitYPos:word;     {Initial top coordinates.}
  InitXSize,InitYSize:word;   {Initial size.}
  Reserved:word;
  ObjectBuffer:PChar;         {If a module cannot be loaded, its
                               name will be returned here.}
  ObjectBuffLen:cardinal;     {Size of your buffer.}
 end;
 PStartData=^TStartData;

const
 ilStandard      =  1; (* Use TFileStatus3/TFindFileBuf3 *)
 ilQueryEASize   =  2; (* Use TFileStatus4/TFindFileBuf4 *)
 ilQueryEAs      =  3;
 ilQueryFullName =  5;
 ilStandardL     = 11; (* Use TFileStatus3L/TFindFileBuf3L *)
 ilQueryEASizeL  = 12; (* Use TFileStatus4L/TFindFileBuf4L *)
 ilQueryEAsL     = 13;

 quFIFO     = 0;
 quLIFO     = 1;
 quPriority = 2;

 quNoConvert_Address = 0;
 quConvert_Address   = 4;

{Start the new session independent or as a child.}
 ssf_Related_Independent = 0;    {Start new session independent
                                  of the calling session.}
 ssf_Related_Child       = 1;    {Start new session as a child
                                  session to the calling session.}

{Start the new session in the foreground or in the background.}
 ssf_FgBg_Fore           = 0;    {Start new session in foreground.}
 ssf_FgBg_Back           = 1;    {Start new session in background.}

{Should the program started in the new session
 be executed under conditions for tracing?}
 ssf_TraceOpt_None       = 0;    {No trace.}
 ssf_TraceOpt_Trace      = 1;    {Trace with no notification
                                  of descendants.}
 ssf_TraceOpt_TraceAll   = 2;    {Trace all descendant sessions.
                                  A termination queue must be
                                  supplied and Related must be
                                  ssf_Related_Child (=1).}

{Will the new session inherit open file handles
 and environment from the calling process.}
 ssf_InhertOpt_Shell     = 0;    {Inherit from the shell.}
 ssf_InhertOpt_Parent    = 1;    {Inherit from the calling process.}

{Specifies the type of session to start.}
 ssf_Type_Default        = 0;    {Use program's type.}
 ssf_Type_FullScreen     = 1;    {OS/2 full screen.}
 ssf_Type_WindowableVIO  = 2;    {OS/2 window.}
 ssf_Type_PM             = 3;    {Presentation Manager.}
 ssf_Type_VDM            = 4;    {DOS full screen.}
 ssf_Type_WindowedVDM    = 7;    {DOS window.}
{Additional values for Windows programs}
 Prog_31_StdSeamlessVDM    = 15; {Windows 3.1 program in its
                                  own windowed session.}
 Prog_31_StdSeamlessCommon = 16; {Windows 3.1 program in a
                                  common windowed session.}
 Prog_31_EnhSeamlessVDM    = 17; {Windows 3.1 program in enhanced
                                  compatibility mode in its own
                                  windowed session.}
 Prog_31_EnhSeamlessCommon = 18; {Windows 3.1 program in enhanced
                                  compatibility mode in a common
                                  windowed session.}
 Prog_31_Enh               = 19; {Windows 3.1 program in enhanced
                                  compatibility mode in a full
                                  screen session.}
 Prog_31_Std               = 20; {Windows 3.1 program in a full
                                  screen session.}

{Specifies the initial attributes for a OS/2 window or DOS window session.}
 ssf_Control_Visible      = 0;   {Window is visible.}
 ssf_Control_Invisible    = 1;   {Window is invisible.}
 ssf_Control_Maximize     = 2;   {Window is maximized.}
 ssf_Control_Minimize     = 4;   {Window is minimized.}
 ssf_Control_NoAutoClose  = 8;   {Window will not close after
                                  the program has ended.}
 ssf_Control_SetPos   = 32768;   {Use InitXPos, InitYPos,
                                  InitXSize, and InitYSize for
                                  the size and placement.}


{This is the correct way to call external assembler procedures.}
procedure syscall;external name '___SYSCALL';

function DosSetFileInfo (Handle: THandle; InfoLevel: cardinal; AFileStatus: PFileStatus;
        FileStatusLen: cardinal): cardinal; cdecl; external 'DOSCALLS' index 218;

function DosQueryFSInfo (DiskNum, InfoLevel: cardinal; var Buffer: TFSInfo;
               BufLen: cardinal): cardinal; cdecl; external 'DOSCALLS' index 278;

function DosQueryFileInfo (Handle: THandle; InfoLevel: cardinal;
           AFileStatus: PFileStatus; FileStatusLen: cardinal): cardinal; cdecl;
                                                 external 'DOSCALLS' index 279;

function DosScanEnv (Name: PChar; var Value: PChar): cardinal; cdecl;
                                                 external 'DOSCALLS' index 227;

function DosFindFirst (FileMask: PChar; var Handle: THandle; Attrib: cardinal;
                       AFileStatus: PFileStatus; FileStatusLen: cardinal;
                    var Count: cardinal; InfoLevel: cardinal): cardinal; cdecl;
                                                 external 'DOSCALLS' index 264;

function DosFindNext (Handle: THandle; AFileStatus: PFileStatus;
                FileStatusLen: cardinal; var Count: cardinal): cardinal; cdecl;
                                                 external 'DOSCALLS' index 265;

function DosFindClose (Handle: THandle): cardinal; cdecl;
                                                 external 'DOSCALLS' index 263;

function DosQueryCtryInfo (Size: cardinal; var Country: TCountryCode;
           var Res: TCountryInfo; var ActualSize: cardinal): cardinal; cdecl;
                                                        external 'NLS' index 5;

function DosMapCase (Size: cardinal; var Country: TCountryCode;
                      AString: PChar): cardinal; cdecl; external 'NLS' index 7;

procedure DosSleep (MSec: cardinal); cdecl; external 'DOSCALLS' index 229;

function DosCreateQueue (var Handle: THandle; Priority:longint;
                        Name: PChar): cardinal; cdecl;
                                                  external 'QUECALLS' index 16;

function DosReadQueue (Handle: THandle; var ReqBuffer: TRequestData;
                      var DataLen: cardinal; var DataPtr: pointer;
                      Element, Wait: cardinal; var Priority: byte;
                      ASem: THandle): cardinal; cdecl;
                                                   external 'QUECALLS' index 9;

function DosCloseQueue (Handle: THandle): cardinal; cdecl;
                                                  external 'QUECALLS' index 11;

function DosStartSession (var AStartData: TStartData;
                          var SesID, PID: cardinal): cardinal; cdecl;
                                                    external 'SESMGR' index 37;

function DosFreeMem(P:pointer):cardinal; cdecl; external 'DOSCALLS' index 304;


{****************************************************************************
                              File Functions
****************************************************************************}

const
 ofRead        = $0000;     {Open for reading}
 ofWrite       = $0001;     {Open for writing}
 ofReadWrite   = $0002;     {Open for reading/writing}
 doDenyRW      = $0010;     {DenyAll (no sharing)}
 faCreateNew   = $00010000; {Create if file does not exist}
 faOpenReplace = $00040000; {Truncate if file exists}
 faCreate      = $00050000; {Create if file does not exist, truncate otherwise}

 FindResvdMask = $00003737; {Allowed bits in attribute
                             specification for DosFindFirst call.}

{$ASMMODE INTEL}
function FileOpen (const FileName: pointer; Mode: integer): longint; assembler;
asm
 push ebx
{$IFDEF REGCALL}
 mov ecx, edx
 mov edx, eax
{$ELSE REGCALL}
 mov ecx, Mode
 mov edx, FileName
{$ENDIF REGCALL}
(* DenyNone if sharing not specified. *)
 mov eax, ecx
 xor eax, 112
 jz @FOpenDefSharing
 cmp eax, 64
 jbe @FOpen1
@FOpenDefSharing:
 or ecx, 64
@FOpen1:
 mov eax, 7F2Bh
 call syscall
(* syscall __open() returns -1 in case of error, i.e. exactly what we need *)
 pop ebx
end {['eax', 'ebx', 'ecx', 'edx']};

function FileOpen (const FileName: rawbytestring; Mode: integer): longint;
var
  SystemFileName: RawByteString;
begin
  SystemFileName := ToSingleByteFileSystemEncodedFileName(FileName);
  FileOpen := FileOpen(pointer(SystemFileName),Mode);
end;

function FileCreate (const FileName: RawByteString): longint;
begin
  FileCreate := FileCreate (FileName, ofReadWrite or faCreate or doDenyRW, 777);
                                                       (* Sharing to DenyAll *)
end;


function FileCreate (const FileName: RawByteString; Rights: integer): longint;
begin
  FileCreate := FileCreate (FileName, ofReadWrite or faCreate or doDenyRW,
                                              Rights); (* Sharing to DenyAll *)
end;

function FileCreate (const FileName: Pointer; ShareMode: integer; Rights: integer): longint; assembler;
asm
 push ebx
{$IFDEF REGCALL}
 mov ecx, edx
 mov edx, eax
{$ELSE REGCALL}
 mov ecx, ShareMode
 mov edx, FileName
{$ENDIF REGCALL}
 and ecx, 112
 or ecx, ecx
 jz @FCDefSharing
 cmp ecx, 64
 jbe @FCSharingOK
@FCDefSharing:
 mov ecx, doDenyRW   (* Sharing to DenyAll *)
@FCSharingOK:
 or ecx, ofReadWrite or faCreate
 mov eax, 7F2Bh
 call syscall
 pop ebx
end {['eax', 'ebx', 'ecx', 'edx']};

function FileCreate (const FileName: RawByteString; ShareMode: integer; Rights: integer): longint;
var
  SystemFileName: RawByteString;
begin
  SystemFileName := ToSingleByteFileSystemEncodedFileName(FileName);
  FileCreate := FileCreate(pointer(SystemFileName),ShareMode,Rights);
end;

function FileRead (Handle: longint; Out Buffer; Count: longint): longint;
                                                                     assembler;
asm
 push ebx
{$IFDEF REGCALL}
 mov ebx, eax
{$ELSE REGCALL}
 mov ebx, Handle
 mov ecx, Count
 mov edx, Buffer
{$ENDIF REGCALL}
 mov eax, 3F00h
 call syscall
 jnc @FReadEnd
 mov eax, -1
@FReadEnd:
 pop ebx
end {['eax', 'ebx', 'ecx', 'edx']};


function FileWrite (Handle: longint; const Buffer; Count: longint): longint;
                                                                     assembler;
asm
 push ebx
{$IFDEF REGCALL}
 mov ebx, eax
{$ELSE REGCALL}
 mov ebx, Handle
 mov ecx, Count
 mov edx, Buffer
{$ENDIF REGCALL}
 mov eax, 4000h
 call syscall
 jnc @FWriteEnd
 mov eax, -1
@FWriteEnd:
 pop ebx
end {['eax', 'ebx', 'ecx', 'edx']};


function FileSeek (Handle, FOffset, Origin: longint): longint; assembler;
asm
 push ebx
{$IFDEF REGCALL}
 mov ebx, eax
 mov eax, ecx
{$ELSE REGCALL}
 mov ebx, Handle
 mov eax, Origin
 mov edx, FOffset
{$ENDIF REGCALL}
 mov ah, 42h
 call syscall
 jnc @FSeekEnd
 mov eax, -1
@FSeekEnd:
 pop ebx
end {['eax', 'ebx', 'edx']};

function FileSeek (Handle: longint; FOffset: Int64; Origin: longint): Int64;
begin
  {$warning need to add 64bit call }
  Result:=FileSeek(Handle,Longint(Foffset),Longint(Origin));
end;

procedure FileClose (Handle: longint);
begin
    if (Handle > 4) or ((os_mode = osOS2) and (Handle > 2)) then
        asm
            push ebx
            mov eax, 3E00h
            mov ebx, Handle
            call syscall
            pop ebx
        end ['eax'];
end;


function FileTruncate (Handle: THandle; Size: Int64): boolean; assembler;
asm
 push ebx
{$IFDEF REGCALL}
 mov ebx, eax
{$ELSE REGCALL}
 mov ebx, Handle
{$ENDIF REGCALL}
 mov edx, dword ptr Size
 mov eax, dword ptr Size+4
 or eax, eax
 mov eax, 0
 jz @FTruncEnd  (* file sizes > 4 GB not supported with EMX *)
 mov eax, 7F25h
 push ebx
 call syscall
 pop ebx
 jc @FTruncEnd
 mov eax, 4202h
 mov edx, 0
 call syscall
 mov eax, 0
 jnc @FTruncEnd
 dec eax
@FTruncEnd:
 pop ebx
end {['eax', 'ebx', 'ecx', 'edx']};


function FileAge (const FileName: RawByteString): Int64;
var Handle: longint;
begin
    Handle := FileOpen (FileName, 0);
    if Handle <> -1 then
        begin
            Result := FileGetDate (Handle);
            FileClose (Handle);
        end
    else
        Result := -1;
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


type
  PSearchRec = ^SearchRec;

Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;

var
  SystemEncodedPath: RawByteString;
  SR: PSearchRec;
  FStat: PFileFindBuf3L;
  Count: cardinal;
  Err: cardinal;

begin
  if os_mode = osOS2 then
   begin
    SystemEncodedPath:=ToSingleByteFileSystemEncodedFileName(Path);
    New (FStat);
    Rslt.FindHandle := THandle ($FFFFFFFF);
    Count := 1;
    if FSApi64 then
     Err := DosFindFirst (PChar (SystemEncodedPath), Rslt.FindHandle,
            Attr and FindResvdMask, FStat, SizeOf (FStat^), Count, ilStandardL)
    else
     Err := DosFindFirst (PChar (SystemEncodedPath), Rslt.FindHandle,
            Attr and FindResvdMask, FStat, SizeOf (FStat^), Count, ilStandard);
    if (Err = 0) and (Count = 0) then
     Err := 18;
    InternalFindFirst := -Err;
    if Err = 0 then
     begin
      Rslt.ExcludeAttr := 0;
      Rslt.Time := cardinal (FStat^.DateLastWrite) shl 16 +
                                                          FStat^.TimeLastWrite;
      if FSApi64 then
       begin
        Rslt.Size := FStat^.FileSize;
        Name := FStat^.Name;
        Rslt.Attr := FStat^.AttrFile;
       end
      else
       begin
        Rslt.Size := PFileFindBuf3 (FStat)^.FileSize;
        Name := PFileFindBuf3 (FStat)^.Name;
        Rslt.Attr := PFileFindBuf3 (FStat)^.AttrFile;
       end;
      SetCodePage(Name, DefaultFileSystemCodePage, false);
     end
    else
     InternalFindClose (Rslt.FindHandle);
    Dispose (FStat);
   end
  else
   begin
    Err := DOS.DosError;
    GetMem (SR, SizeOf (SearchRec));
    Rslt.FindHandle := longint(SR);
    DOS.FindFirst (Path, Attr, SR^);
    InternalFindFirst := -DOS.DosError;
    if DosError = 0 then
     begin
      Rslt.Time := SR^.Time;
(* Extend the supported file sizes from 2 GB to 4 GB at least. *)
      Rslt.Size := cardinal (SR^.Size);
      Rslt.Attr := SR^.Attr;
      Rslt.ExcludeAttr := 0;
      Name := SR^.Name;
      SetCodePage(Name, DefaultFileSystemCodePage, false);
     end;
    DOS.DosError := Err;
   end;
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;

var
  SR: PSearchRec;
  FStat: PFileFindBuf3L;
  Count: cardinal;
  Err: cardinal;

begin
  if os_mode = osOS2 then
   begin
    New (FStat);
    Count := 1;
    Err := DosFindNext (Rslt.FindHandle, FStat, SizeOf (FStat^), Count);
    if (Err = 0) and (Count = 0) then
     Err := 18;
    InternalFindNext := -Err;
    if Err = 0 then
     begin
      Rslt.ExcludeAttr := 0;
      Rslt.Time := cardinal (FStat^.DateLastWrite) shl 16 +
                                                          FStat^.TimeLastWrite;
      if FSApi64 then
       begin
        Rslt.Size := FStat^.FileSize;
        Name := FStat^.Name;
        Rslt.Attr := FStat^.AttrFile;
       end
      else
       begin
        Rslt.Size := PFileFindBuf3 (FStat)^.FileSize;
        Name := PFileFindBuf3 (FStat)^.Name;
        Rslt.Attr := PFileFindBuf3 (FStat)^.AttrFile;
       end;
      SetCodePage(Name, DefaultFileSystemCodePage, false);
     end;
    Dispose (FStat);
   end
  else
   begin
    SR := PSearchRec (Rslt.FindHandle);
    if SR <> nil then
     begin
      DOS.FindNext (SR^);
      InternalFindNext := -DosError;
      if DosError = 0 then
       begin
        Rslt.Time := SR^.Time;
(* Extend the supported file sizes from 2 GB to 4 GB at least. *)
        Rslt.Size := cardinal (SR^.Size);
        Rslt.Attr := SR^.Attr;
        Rslt.ExcludeAttr := 0;
        Name := SR^.Name;
        SetCodePage(Name, DefaultFileSystemCodePage, false);
       end;
     end;
   end;
end;


Procedure InternalFindClose(var Handle: THandle);

var SR: PSearchRec;

begin
    if os_mode = osOS2 then
        begin
            DosFindClose (Handle);
        end
    else
        begin
            SR := PSearchRec (Handle);
            DOS.FindClose (SR^);
            FreeMem (SR, SizeOf (SearchRec));
        end;
    Handle := 0;
end;


function FileGetDate (Handle: longint): Int64; assembler;
asm
 push ebx
{$IFDEF REGCALL}
 mov ebx, eax
{$ELSE REGCALL}
 mov ebx, Handle
{$ENDIF REGCALL}
 mov ax, 5700h
 call syscall
 mov eax, -1
 jc @FGetDateEnd
 mov ax, dx
 shld eax, ecx, 16
@FGetDateEnd:
 pop ebx
 xor edx,edx
end {['eax', 'ebx', 'ecx', 'edx']};


function FileSetDate (Handle: longint; Age: Int64): longint;
var FStat: PFileStatus3;
    RC: cardinal;
begin
    if os_mode = osOS2 then
begin
            New (FStat);
            RC := DosQueryFileInfo (Handle, ilStandard, FStat,
                                                              SizeOf (FStat^));
            if RC <> 0 then
                FileSetDate := -1
            else
                begin
                    FStat^.DateLastAccess := Hi (dword (Age));
                    FStat^.DateLastWrite := Hi (dword (Age));
                    FStat^.TimeLastAccess := Lo (dword (Age));
                    FStat^.TimeLastWrite := Lo (dword (Age));
                    RC := DosSetFileInfo (Handle, ilStandard, FStat,
                                                              SizeOf (FStat^));
                    if RC <> 0 then
                        FileSetDate := -1
                    else
                        FileSetDate := 0;
                end;
            Dispose (FStat);
        end
    else
        asm
            push ebx
            mov ax, 5701h
            mov ebx, Handle
            mov cx, word ptr [Age]
            mov dx, word ptr [Age + 2]
            call syscall
            jnc @FSetDateEnd
            mov eax, -1
@FSetDateEnd:
            mov Result, eax
            pop ebx
        end ['eax', 'ecx', 'edx'];
end;


function FileGetAttr (const FileName: rawbytestring): longint; assembler;
asm
{$IFDEF REGCALL}
 mov edx, eax
{$ELSE REGCALL}
 mov edx, FileName
{$ENDIF REGCALL}
 mov ax, 4300h
 call syscall
 jnc @FGetAttrEnd
 mov eax, -1
@FGetAttrEnd:
end {['eax', 'edx']};


function FileSetAttr (const Filename: RawByteString; Attr: longint): longint;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  asm
   mov ecx, Attr
   mov edx, SystemFileName
   mov ax, 4301h
   call syscall
   mov @result, 0
   jnc @FSetAttrEnd
   mov @result, -1
  @FSetAttrEnd:
  end ['eax', 'ecx', 'edx'];
end;

function DeleteFile (const FileName: rawbytestring): boolean;
var
  SystemFileName: RawByteString;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  asm
   mov edx, SystemFileName
   mov ax, 4100h
   call syscall
   mov @result, 0
   jc @FDeleteEnd
   mov @result, 1
  @FDeleteEnd:
  end ['eax', 'edx'];
end;

function RenameFile (const OldName, NewName: rawbytestring): boolean;
var
  OldSystemFileName, NewSystemFileName: RawByteString;
Begin
  OldSystemFileName:=ToSingleByteFileSystemEncodedFileName(OldName);
  NewSystemFileName:=ToSingleByteFileSystemEncodedFileName(NewName);
  asm
   mov edx, OldSystemFileName
   mov edi, NewSystemFileName
   mov ax, 5600h
   call syscall
   mov @result, 0
   jc @FRenameEnd
   mov @result, 1
  @FRenameEnd:
  end ['eax', 'edx', 'edi'];
end;

{****************************************************************************
                              Disk Functions
****************************************************************************}

{$ASMMODE ATT}

function DiskFree (Drive: byte): int64;

var FI: TFSinfo;
    RC: cardinal;

begin
    if (os_mode = osDOS) or (os_mode = osDPMI) then
    {Function 36 is not supported in OS/2.}
        asm
            pushl %ebx
            movb Drive,%dl
            movb $0x36,%ah
            call syscall
            cmpw $-1,%ax
            je .LDISKFREE1
            mulw %cx
            mulw %bx
            shll $16,%edx
            movw %ax,%dx
            movl $0,%eax
            xchgl %edx,%eax
            jmp .LDISKFREE2
         .LDISKFREE1:
            cltd
         .LDISKFREE2:
            popl %ebx
            leave
            ret
        end
    else
        {In OS/2, we use the filesystem information.}
        begin
            RC := DosQueryFSInfo (Drive, 1, FI, SizeOf (FI));
            if RC = 0 then
                DiskFree := int64 (FI.Free_Clusters) *
                   int64 (FI.Sectors_Per_Cluster) * int64 (FI.Bytes_Per_Sector)
            else
                DiskFree := -1;
        end;
end;

function DiskSize (Drive: byte): int64;

var FI: TFSinfo;
    RC: cardinal;

begin
    if (os_mode = osDOS) or (os_mode = osDPMI) then
        {Function 36 is not supported in OS/2.}
        asm
            pushl %ebx
            movb Drive,%dl
            movb $0x36,%ah
            call syscall
            movw %dx,%bx
            cmpw $-1,%ax
            je .LDISKSIZE1
            mulw %cx
            mulw %bx
            shll $16,%edx
            movw %ax,%dx
            movl $0,%eax
            xchgl %edx,%eax
            jmp .LDISKSIZE2
         .LDISKSIZE1:
            cltd
         .LDISKSIZE2:
            popl %ebx
            leave
            ret
        end
    else
        {In OS/2, we use the filesystem information.}
        begin
            RC := DosQueryFSinfo (Drive, 1, FI, SizeOf (FI));
            if RC = 0 then
                DiskSize := int64 (FI.Total_Clusters) *
                   int64 (FI.Sectors_Per_Cluster) * int64 (FI.Bytes_Per_Sector)
            else
                DiskSize := -1;
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


{****************************************************************************
                              Time Functions
****************************************************************************}

{$ASMMODE INTEL}
procedure GetLocalTime (var SystemTime: TSystemTime); assembler;
asm
(* Expects the default record alignment (word)!!! *)
 push edi
{$IFDEF REGCALL}
 push eax
{$ENDIF REGCALL}
 mov ah, 2Ah
 call syscall
{$IFDEF REGCALL}
 pop edi
{$ELSE REGCALL}
 mov edi, SystemTime
{$ENDIF REGCALL}
 xchg ax, cx
 shl eax, 16
 mov al, dh
 stosd
 mov al, dl
 shl eax, 16
 mov al, cl
 stosd
 push edi
 mov ah, 2Ch
 call syscall
 pop edi
 xor eax, eax
 mov al, ch
 shl eax, 16
 mov al, cl
 stosd
 xor eax, eax
 mov al, 10
 mul dl
 shl eax, 16
 mov al, dh
 rol eax, 16
 stosd
 pop edi
end {['eax', 'ecx', 'edx', 'edi']};
{$asmmode default}


{****************************************************************************
                              Misc Functions
****************************************************************************}



{****************************************************************************
                              Locale Functions
****************************************************************************}

procedure InitAnsi;
var I: byte;
    Country: TCountryCode;
begin
    for I := 0 to 255 do
        UpperCaseTable [I] := Chr (I);
    Move (UpperCaseTable, LowerCaseTable, SizeOf (UpperCaseTable));
    if os_mode = osOS2 then
        begin
            FillChar (Country, SizeOf (Country), 0);
            DosMapCase (SizeOf (UpperCaseTable), Country, @UpperCaseTable);
        end
    else
        begin
(* !!! TODO: DOS/DPMI mode support!!! *)
        end;
    for I := 0 to 255 do
        if UpperCaseTable [I] <> Chr (I) then
            LowerCaseTable [Ord (UpperCaseTable [I])] := Chr (I);
end;


procedure InitInternational;
var Country: TCountryCode;
    CtryInfo: TCountryInfo;
    Size: cardinal;
    RC: cardinal;
begin
  Size := 0;
  FillChar (Country, SizeOf (Country), 0);
  FillChar (CtryInfo, SizeOf (CtryInfo), 0);
  RC := DosQueryCtryInfo (SizeOf (CtryInfo), Country, CtryInfo, Size);
  if RC = 0 then
      begin
          DateSeparator := CtryInfo.DateSeparator;
          case CtryInfo.DateFormat of
           1: begin
                  ShortDateFormat := 'd/m/y';
                  LongDateFormat := 'dd" "mmmm" "yyyy';
              end;
           2: begin
                  ShortDateFormat := 'y/m/d';
                  LongDateFormat := 'yyyy" "mmmm" "dd';
              end;
           3: begin
                  ShortDateFormat := 'm/d/y';
                  LongDateFormat := 'mmmm" "dd" "yyyy';
              end;
          end;
          TimeSeparator := CtryInfo.TimeSeparator;
          DecimalSeparator := CtryInfo.DecimalSeparator;
          ThousandSeparator := CtryInfo.ThousandSeparator;
          CurrencyFormat := CtryInfo.CurrencyFormat;
          CurrencyString := PChar (CtryInfo.CurrencyUnit);
      end;
  InitAnsi;
  InitInternationalGeneric;
end;

function SysErrorMessage(ErrorCode: Integer): String;

begin
  Result:=Format(SUnknownErrorCode,[ErrorCode]);
end;


{****************************************************************************
                              OS Utils
****************************************************************************}

Function GetEnvironmentVariable(Const EnvVar : String) : String;

begin
    GetEnvironmentVariable := GetEnvPChar (EnvVar);
end;


Function GetEnvironmentVariableCount : Integer;

begin
(*  Result:=FPCCountEnvVar(EnvP); - the amount is already known... *)
  GetEnvironmentVariableCount := EnvC;
end;


Function GetEnvironmentString(Index : Integer) : {$ifdef FPC_RTL_UNICODE}UnicodeString{$else}AnsiString{$endif};

begin
  Result:=FPCGetEnvStrFromP (EnvP, Index);
end;


{$ASMMODE INTEL}
procedure Sleep (Milliseconds: cardinal);

begin
 if os_mode = osOS2 then DosSleep (Milliseconds) else
  asm
   mov edx, Milliseconds
   mov eax, 7F30h
   call syscall
  end ['eax', 'edx'];
end;
{$ASMMODE DEFAULT}


function ExecuteProcess (const Path: RawByteString; const ComLine: RawByteString;Flags:TExecuteFlags=[]):
                                                                       integer;
var
 HQ: THandle;
 SPID, STID, QName: shortstring;
 SD: TStartData;
 SID, PID: cardinal;
 RD: TRequestData;
 PCI: PChildInfo;
 CISize: cardinal;
 Prio: byte;
 E: EOSError;
 CommandLine: rawbytestring;

begin
 if os_Mode = osOS2 then
  begin
   FillChar (SD, SizeOf (SD), 0);
   SD.Length := 24;
   SD.Related := ssf_Related_Child;
   SD.PgmName := PChar (Path);
   SD.PgmInputs := PChar (ComLine);
   Str (GetProcessID, SPID);
   Str (ThreadID, STID);
   QName := '\QUEUES\FPC_ExecuteProcess_p' + SPID + 't' + STID + '.QUE'#0;
   SD.TermQ := @QName [1];
   Result := DosCreateQueue (HQ, quFIFO or quConvert_Address, @QName [1]);
   if Result = 0 then
    begin
     Result := DosStartSession (SD, SID, PID);
     if (Result = 0) or (Result = 457) then
      begin
       Result := DosReadQueue (HQ, RD, CISize, PCI, 0, 0, Prio, 0);
       if Result = 0 then
       begin
        Result := PCI^.Return;
        DosCloseQueue (HQ);
        DosFreeMem (PCI);
        Exit;
       end;
      end;
     DosCloseQueue (HQ);
    end;
    if ComLine = '' then
     CommandLine := Path
    else
     CommandLine := Path + ' ' + ComLine;
    E := EOSError.CreateFmt (SExecuteProcessFailed, [CommandLine, Result]);
    E.ErrorCode := Result;
    raise E;
  end else
  begin
   Dos.Exec (Path, ComLine);
   if DosError <> 0 then
    begin
    if ComLine = '' then
     CommandLine := Path
    else
     CommandLine := Path + ' ' + ComLine;
      E := EOSError.CreateFmt (SExecuteProcessFailed, [CommandLine, DosError]);
      E.ErrorCode := DosError;
      raise E;
    end;
   ExecuteProcess := DosExitCode;
  end;
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



{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;       { Initialize exceptions. OS independent }
  InitInternational;    { Initialize internationalization settings }
Finalization
  FreeTerminateProcs;
  DoneExceptions;
end.
