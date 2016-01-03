{

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    Sysutils unit for OS/2

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

{$DEFINE HAS_SLEEP}
{$DEFINE HAS_OSERROR}

{ used OS file system APIs use ansistring }
{$define SYSUTILS_HAS_ANSISTR_FILEUTIL_IMPL}
{ OS has an ansistring/single byte environment variable API }
{$define SYSUTILS_HAS_ANSISTR_ENVVAR_IMPL}

{ Include platform independent interface part }
{$i sysutilh.inc}


implementation

  uses
    sysconst, DosCalls;


type
<<<<<<< HEAD
(* Necessary here due to a different definition of TDateTime in DosCalls. *)
  TDateTime = System.TDateTime;
=======
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
            cbList:cardinal;            {Size of the file's extended attributes.}
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

 TResultCodes=record
  TerminateReason,        {0 = Normal termionation.
                           1 = Critical error.
                           2 = Trapped. (GPE, etc.)
                           3 = Killed by DosKillProcess.}
  ExitCode:cardinal;      {Exit code of child.}
 end;
>>>>>>> graemeg/cpstrnew

<<<<<<< HEAD
threadvar
  LastOSError: cardinal;
=======
const
 ilStandard      = 1;
 ilQueryEAsize   = 2;
 ilQueryEAs      = 3;
 ilQueryFullName = 5;

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

function DosDelete(FileName:PChar): cardinal; cdecl;
    external 'DOSCALLS' index 259;

function DosMove(OldFile, NewFile:PChar): cardinal; cdecl;
    external 'DOSCALLS' index 271;

function DosQueryPathInfo(FileName:PChar;InfoLevel:cardinal;
              AFileStatus:PFileStatus;FileStatusLen:cardinal): cardinal; cdecl;
    external 'DOSCALLS' index 223;

function DosSetPathInfo(FileName:PChar;InfoLevel:cardinal;
                        AFileStatus:PFileStatus;FileStatusLen,
                        Options:cardinal):cardinal; cdecl;
    external 'DOSCALLS' index 219;

function DosClose(Handle: THandle): cardinal; cdecl;
    external 'DOSCALLS' index 257;

function DosRead(Handle:THandle; var Buffer; Count: cardinal;
                                      var ActCount: cardinal): cardinal; cdecl;
    external 'DOSCALLS' index 281;

function DosWrite(Handle: THandle; Buffer: pointer; Count: cardinal;
                                      var ActCount: cardinal): cardinal; cdecl;
    external 'DOSCALLS' index 282;

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
>>>>>>> graemeg/fixes_2_2

{$DEFINE FPC_FEXPAND_UNC} (* UNC paths are supported *)
{$DEFINE FPC_FEXPAND_DRIVES} (* Full paths begin with drive specification *)
{$DEFINE FPC_FEXPAND_GETENV_PCHAR}
{$DEFINE HAS_GETTICKCOUNT}
{$DEFINE HAS_GETTICKCOUNT64}

{ Include platform independent implementation part }
{$i sysutils.inc}


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

<<<<<<< HEAD
<<<<<<< HEAD
function FileOpen (const FileName: rawbytestring; Mode: integer): THandle;
=======
function FileOpen (const FileName: string; Mode: integer): THandle;
>>>>>>> graemeg/fixes_2_2
=======
function FileOpen (const FileName: string; Mode: integer): THandle;
>>>>>>> origin/fixes_2_2
Var
  SystemFileName: RawByteString;
  Handle: THandle;
  Rc, Action: cardinal;
begin
<<<<<<< HEAD
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
(* DenyReadWrite if sharing not specified. *)
  if (Mode and 112 = 0) or (Mode and 112 > 64) then
   Mode := Mode or doDenyRW;
  Rc:=Sys_DosOpenL(PChar (SystemFileName), Handle, Action, 0, 0, 1, Mode, nil);
=======
(* DenyNone if sharing not specified. *)
  if Mode and 112 = 0 then Mode:=Mode or 64;
  Rc:=Sys_DosOpenL(PChar (FileName), Handle, Action, 0, 0, 1, Mode, nil);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  If Rc=0 then
    FileOpen:=Handle
  else
   begin
    FileOpen:=feInvalidHandle; //FileOpen:=-RC;
    //should return feInvalidHandle(=-1) if fail, other negative returned value are no more errors
    OSErrorWatch (RC);
   end;
end;

<<<<<<< HEAD
function FileCreate (const FileName: RawByteString): THandle;
begin
  FileCreate := FileCreate (FileName, doDenyRW, 777); (* Sharing to DenyAll *)
end;

function FileCreate (const FileName: RawByteString; Rights: integer): THandle;
begin
  FileCreate := FileCreate (FileName, doDenyRW, Rights);
                                      (* Sharing to DenyAll *)
end;

<<<<<<< HEAD
function FileCreate (const FileName: RawByteString; ShareMode: integer;
                                                     Rights: integer): THandle;
var
  SystemFileName: RawByteString;
  Handle: THandle;
  RC, Action: cardinal;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(FileName);
  ShareMode := ShareMode and 112;
  (* Sharing to DenyAll as default in case of values not allowed by OS/2. *)
  if (ShareMode = 0) or (ShareMode > 64) then
   ShareMode := doDenyRW;
  RC := Sys_DosOpenL (PChar (SystemFileName), Handle, Action, 0, 0, $12,
                                    faCreate or ofReadWrite or ShareMode, nil);
  if RC = 0 then
   FileCreate := Handle
  else
   begin
    FileCreate := feInvalidHandle;
    OSErrorWatch (RC);
   end;
End;

=======
=======
>>>>>>> origin/fixes_2_2
function FileCreate (const FileName: string): THandle;
Const
  Mode = ofReadWrite or faCreate or doDenyRW;   (* Sharing to DenyAll *)
Var
  Handle: THandle;
  RC, Action: cardinal;
Begin
  RC:=Sys_DosOpenL(PChar (FileName), Handle, Action, 0, 0, $12, Mode, Nil);
  If RC=0 then
    FileCreate:=Handle
  else
    FileCreate:=feInvalidHandle;
End;

function FileCreate (const FileName: string; Mode: integer): THandle;
begin
 FileCreate := FileCreate(FileName);
end;
>>>>>>> graemeg/fixes_2_2

<<<<<<< HEAD
=======

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> origin/cpstrnew
function FileRead (Handle: THandle; Out Buffer; Count: longint): longint;
=======
function FileRead (Handle: THandle; var Buffer; Count: longint): longint;
>>>>>>> graemeg/fixes_2_2
=======
function FileRead (Handle: THandle; var Buffer; Count: longint): longint;
>>>>>>> origin/fixes_2_2
Var
  T: cardinal;
  RC: cardinal;
begin
  RC := DosRead (Handle, Buffer, Count, T);
  if RC = 0 then
   FileRead := longint (T)
  else
   begin
    FileRead := -1;
    OSErrorWatch (RC);
   end;
end;

function FileWrite (Handle: THandle; const Buffer; Count: longint): longint;
Var
  T: cardinal;
  RC: cardinal;
begin
  RC := DosWrite (Handle, Buffer, Count, T);
  if RC = 0 then
   FileWrite := longint (T)
  else
   begin
    FileWrite := -1;
    OSErrorWatch (RC);
   end;
end;

function FileSeek (Handle: THandle; FOffset, Origin: longint): longint;
var
  NPos: int64;
<<<<<<< HEAD
<<<<<<< HEAD
  RC: cardinal;
begin
  RC := Sys_DosSetFilePtrL (Handle, FOffset, Origin, NPos);
  if (RC = 0) and (NPos < high (longint)) then
=======
begin
  if (Sys_DosSetFilePtrL (Handle, FOffset, Origin, NPos) = 0)
                                               and (NPos < high (longint)) then
>>>>>>> graemeg/fixes_2_2
=======
begin
  if (Sys_DosSetFilePtrL (Handle, FOffset, Origin, NPos) = 0)
                                               and (NPos < high (longint)) then
>>>>>>> origin/fixes_2_2
    FileSeek:= longint (NPos)
  else
   begin
    FileSeek:=-1;
    OSErrorWatch (RC);
   end;
end;

function FileSeek (Handle: THandle; FOffset: Int64; Origin: Longint): Int64;
var
  NPos: int64;
<<<<<<< HEAD
<<<<<<< HEAD
  RC: cardinal;
begin
  RC := Sys_DosSetFilePtrL (Handle, FOffset, Origin, NPos);
  if RC = 0 then
    FileSeek:= NPos
  else
   begin
    FileSeek:=-1;
    OSErrorWatch (RC);
   end;
end;

procedure FileClose (Handle: THandle);
var
  RC: cardinal;
=======
=======
begin
  if Sys_DosSetFilePtrL (Handle, FOffset, Origin, NPos) = 0 then
    FileSeek:= NPos
  else
    FileSeek:=-1;
end;

procedure FileClose (Handle: THandle);
>>>>>>> origin/fixes_2_2
begin
  if Sys_DosSetFilePtrL (Handle, FOffset, Origin, NPos) = 0 then
    FileSeek:= NPos
  else
    FileSeek:=-1;
end;

<<<<<<< HEAD
procedure FileClose (Handle: THandle);
>>>>>>> graemeg/fixes_2_2
begin
  RC := DosClose (Handle);
  if RC <> 0 then
   OSErrorWatch (RC);
end;

function FileTruncate (Handle: THandle; Size: Int64): boolean;
<<<<<<< HEAD
var
  RC: cardinal;
begin
  RC := Sys_DosSetFileSizeL(Handle, Size);
  FileTruncate := RC = 0;
  if RC = 0 then
   FileSeek(Handle, 0, 2)
  else
   OSErrorWatch (RC);
=======
begin
=======
function FileTruncate (Handle: THandle; Size: Int64): boolean;
begin
>>>>>>> origin/fixes_2_2
  FileTruncate:=Sys_DosSetFileSizeL(Handle, Size)=0;
  FileSeek(Handle, 0, 2);
>>>>>>> graemeg/fixes_2_2
end;

function FileAge (const FileName: RawByteString): longint;
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


function FileExists (const FileName: RawByteString): boolean;
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


type    TRec = record
            T, D: word;
        end;
        PSearchRec = ^TSearchRec;

Function InternalFindFirst (Const Path : RawByteString; Attr : Longint; out Rslt : TAbstractSearchRec; var Name: RawByteString) : Longint;

var SR: PSearchRec;
    FStat: PFileFindBuf3L;
    Count: cardinal;
    Err: cardinal;
    I: cardinal;
    SystemEncodedPath: RawByteString;

begin
  SystemEncodedPath := ToSingleByteFileSystemEncodedFileName(Path);
  New (FStat);
  Rslt.FindHandle := THandle ($FFFFFFFF);
  Count := 1;
  if FSApi64 then
   Err := DosFindFirst (PChar (SystemEncodedPath), Rslt.FindHandle,
            Attr and FindResvdMask, FStat, SizeOf (FStat^), Count, ilStandardL)
  else
   Err := DosFindFirst (PChar (SystemEncodedPath), Rslt.FindHandle,
            Attr and FindResvdMask, FStat, SizeOf (FStat^), Count, ilStandard);
  if Err <> 0 then
   OSErrorWatch (Err)
  else if Count = 0 then
   Err := 18;
  InternalFindFirst := -Err;
  if Err = 0 then
   begin
    Rslt.ExcludeAttr := 0;
    TRec (Rslt.Time).T := FStat^.TimeLastWrite;
    TRec (Rslt.Time).D := FStat^.DateLastWrite;
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
    SetCodePage (Name, DefaultFileSystemCodePage, false);
   end
  else
   InternalFindClose(Rslt.FindHandle);

  Dispose (FStat);
end;


Function InternalFindNext (var Rslt : TAbstractSearchRec; var Name : RawByteString) : Longint;
var
  SR: PSearchRec;
  FStat: PFileFindBuf3L;
  Count: cardinal;
  Err: cardinal;
begin
  New (FStat);
  Count := 1;
  Err := DosFindNext (Rslt.FindHandle, FStat, SizeOf (FStat^), Count);
  if Err <> 0 then
   OSErrorWatch (Err)
  else if Count = 0 then
   Err := 18;
  InternalFindNext := -Err;
  if Err = 0 then
  begin
    Rslt.ExcludeAttr := 0;
    TRec (Rslt.Time).T := FStat^.TimeLastWrite;
    TRec (Rslt.Time).D := FStat^.DateLastWrite;
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
    SetCodePage (Name, DefaultFileSystemCodePage, false);
  end;
  Dispose (FStat);
end;


Procedure InternalFindClose(var Handle: THandle);
var
  SR: PSearchRec;
  RC: cardinal;
begin
  RC := DosFindClose (Handle);
  Handle := 0;
  if RC <> 0 then
   OSErrorWatch (RC);
end;

function FileGetDate (Handle: THandle): longint;
var
  FStat: TFileStatus3;
  Time: Longint;
  RC: cardinal;
begin
  RC := DosQueryFileInfo(Handle, ilStandard, @FStat, SizeOf(FStat));
  if RC = 0 then
  begin
    Time := FStat.TimeLastWrite + longint (FStat.DateLastWrite) shl 16;
    if Time = 0 then
      Time := FStat.TimeCreation + longint (FStat.DateCreation) shl 16;
  end else
   begin
    Time:=0;
    OSErrorWatch (RC);
   end;
  FileGetDate:=Time;
end;

function FileSetDate (Handle: THandle; Age: longint): longint;
var
  FStat: PFileStatus3;
  RC: cardinal;
begin
  New (FStat);
  RC := DosQueryFileInfo (Handle, ilStandard, FStat, SizeOf (FStat^));
  if RC <> 0 then
   begin
    FileSetDate := -1;
    OSErrorWatch (RC);
   end
  else
   begin
    FStat^.DateLastAccess := Hi (Age);
    FStat^.DateLastWrite := Hi (Age);
    FStat^.TimeLastAccess := Lo (Age);
    FStat^.TimeLastWrite := Lo (Age);
    RC := DosSetFileInfo (Handle, ilStandard, FStat, SizeOf (FStat^));
    if RC <> 0 then
     begin
      FileSetDate := -1;
      OSErrorWatch (RC);
     end
    else
     FileSetDate := 0;
   end;
  Dispose (FStat);
end;

function FileGetAttr (const FileName: RawByteString): longint;
var
  FS: PFileStatus3;
  SystemFileName: RawByteString;
  RC: cardinal;
begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  New(FS);
  RC := DosQueryPathInfo(PChar (SystemFileName), ilStandard, FS, SizeOf(FS^));
  if RC = 0 then
   Result := FS^.AttrFile
  else
   begin
    Result := - longint (RC);
    OSErrorWatch (RC);
   end;
  Dispose(FS);
end;

function FileSetAttr (const Filename: RawByteString; Attr: longint): longint;
Var
  FS: PFileStatus3;
  SystemFileName: RawByteString;
  RC: cardinal;
Begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  New(FS);
  RC := DosQueryPathInfo (PChar (SystemFileName), ilStandard, FS, SizeOf (FS^));
  if RC = 0 then
   begin
    FS^.AttrFile:=Attr;
    RC := DosSetPathInfo(PChar (SystemFileName), ilStandard, FS, SizeOf(FS^), 0);
    if RC <> 0 then
     OSErrorWatch (RC);
   end
  else
   OSErrorWatch (RC);
  Result := - longint (RC);
  Dispose(FS);
end;


function DeleteFile (const FileName: RawByteString): boolean;
var
  SystemFileName: RawByteString;
  RC: cardinal;
Begin
  SystemFileName:=ToSingleByteFileSystemEncodedFileName(Filename);
  RC := DosDelete (PChar (SystemFileName));
  if RC <> 0 then
   begin
    Result := false;
    OSErrorWatch (RC);
   end
  else
   Result := true;
End;

function RenameFile (const OldName, NewName: RawByteString): boolean;
var
  OldSystemFileName, NewSystemFileName: RawByteString;
  RC: cardinal;
Begin
  OldSystemFileName:=ToSingleByteFileSystemEncodedFileName(OldName);
  NewSystemFileName:=ToSingleByteFileSystemEncodedFileName(NewName);
  RC := DosMove (PChar (OldSystemFileName), PChar (NewSystemFileName));
  if RC <> 0 then
   begin
    Result := false;
    OSErrorWatch (RC);
   end
  else
   Result := true;
End;

{****************************************************************************
                              Disk Functions
****************************************************************************}

function DiskFree (Drive: byte): int64;

var FI: TFSinfo;
    RC: cardinal;

begin
  {In OS/2, we use the filesystem information.}
  RC := DosQueryFSInfo (Drive, 1, FI, SizeOf (FI));
  if RC = 0 then
   DiskFree := int64 (FI.Free_Clusters) *
                   int64 (FI.Sectors_Per_Cluster) * int64 (FI.Bytes_Per_Sector)
  else
   begin
    DiskFree := -1;
    OSErrorWatch (RC);
   end;
end;

function DiskSize (Drive: byte): int64;

var FI: TFSinfo;
    RC: cardinal;

begin
  {In OS/2, we use the filesystem information.}
  RC := DosQueryFSinfo (Drive, 1, FI, SizeOf (FI));
  if RC = 0 then
   DiskSize := int64 (FI.Total_Clusters) *
                   int64 (FI.Sectors_Per_Cluster) * int64 (FI.Bytes_Per_Sector)
<<<<<<< HEAD
  else
   begin
    DiskSize := -1;
    OSErrorWatch (RC);
   end;
=======
            else
                DiskSize := -1;
end;


function GetCurrentDir: string;
begin
 GetDir (0, Result);
end;


function SetCurrentDir (const NewDir: string): boolean;
begin
{$I-}
{$WARNING Should be rewritten to avoid unit dos dependency!}
 ChDir (NewDir);
 Result := (IOResult = 0);
{$I+}
end;


function CreateDir (const NewDir: string): boolean;
begin
{$I-}
{$WARNING Should be rewritten to avoid unit dos dependency!}
 MkDir (NewDir);
 Result := (IOResult = 0);
{$I+}
end;


function RemoveDir (const Dir: string): boolean;
begin
{$I-}
{$WARNING Should be rewritten to avoid unit dos dependency!}
 RmDir (Dir);
 Result := (IOResult = 0);
 {$I+}
>>>>>>> graemeg/cpstrnew
end;


function DirectoryExists (const Directory: RawByteString): boolean;
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

procedure GetLocalTime (var SystemTime: TSystemTime);
var
  DT: DosCalls.TDateTime;
begin
  DosGetDateTime(DT);
  with SystemTime do
  begin
    Year:=DT.Year;
    Month:=DT.Month;
    Day:=DT.Day;
    Hour:=DT.Hour;
    Minute:=DT.Minute;
    Second:=DT.Second;
    MilliSecond:=DT.Sec100;
  end;
end;

{****************************************************************************
                              Misc Functions
****************************************************************************}
procedure sysbeep;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  DosBeep (800, 250);
end;

=======
  // Maybe implement later on ?
=======
  // Maybe implement later on ?

end;
>>>>>>> graemeg/cpstrnew

=======
  // Maybe implement later on ?

>>>>>>> graemeg/cpstrnew
=======
  // Maybe implement later on ?

>>>>>>> origin/cpstrnew
end;

>>>>>>> graemeg/cpstrnew
{****************************************************************************
                              Locale Functions
****************************************************************************}

var
  Country: TCountryCode;
  CtryInfo: TCountryInfo;

procedure InitAnsi;
var
  I: byte;
  RC: cardinal;
begin
    for I := 0 to 255 do
        UpperCaseTable [I] := Chr (I);
    Move (UpperCaseTable, LowerCaseTable, SizeOf (UpperCaseTable));
            FillChar (Country, SizeOf (Country), 0);
            DosMapCase (SizeOf (UpperCaseTable), Country, @UpperCaseTable);
    for I := 0 to 255 do
        if UpperCaseTable [I] <> Chr (I) then
            LowerCaseTable [Ord (UpperCaseTable [I])] := Chr (I);
end;


procedure InitInternational;
var
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
   end
  else
   OSErrorWatch (RC);
  InitAnsi;
  InitInternationalGeneric;
end;

function SysErrorMessage(ErrorCode: Integer): String;
const
  SysMsgFile: array [0..10] of char = 'OSO001.MSG'#0;
var
  OutBuf: array [0..999] of char;
  RetMsgSize: cardinal;
  RC: cardinal;
begin
  RC := DosGetMessage (nil, 0, @OutBuf [0], SizeOf (OutBuf),
                                       ErrorCode, @SysMsgFile [0], RetMsgSize);
  if RC = 0 then
   begin
    SetLength (Result, RetMsgSize);
    Move (OutBuf [0], Result [1], RetMsgSize);
   end
  else
   begin
    Result:=Format(SUnknownErrorCode,[ErrorCode]);
    OSErrorWatch (RC);
   end;
end;


{****************************************************************************
                              OS Utils
****************************************************************************}

function GetEnvPChar (EnvVar: shortstring): PChar;
(* The assembler version is more than three times as fast as Pascal. *)
var
 P: PChar;
begin
 EnvVar := UpCase (EnvVar);
{$ASMMODE INTEL}
 asm
  cld
  mov edi, Environment
  lea esi, EnvVar
  xor eax, eax
  lodsb
@NewVar:
  cmp byte ptr [edi], 0
  jz @Stop
  push eax        { eax contains length of searched variable name }
  push esi        { esi points to the beginning of the variable name }
  mov ecx, -1     { our character ('=' - see below) _must_ be found }
  mov edx, edi    { pointer to beginning of variable name saved in edx }
  mov al, '='     { searching until '=' (end of variable name) }
  repne
  scasb           { scan until '=' not found }
  neg ecx         { what was the name length? }
  dec ecx         { corrected }
  dec ecx         { exclude the '=' character }
  pop esi         { restore pointer to beginning of variable name }
  pop eax         { restore length of searched variable name }
  push eax        { and save both of them again for later use }
  push esi
  cmp ecx, eax    { compare length of searched variable name with name }
  jnz @NotEqual   { ... of currently found variable, jump if different }
  xchg edx, edi   { pointer to current variable name restored in edi }
  repe
  cmpsb           { compare till the end of variable name }
  xchg edx, edi   { pointer to beginning of variable contents in edi }
  jz @Equal       { finish if they're equal }
@NotEqual:
  xor eax, eax    { look for 00h }
  mov ecx, -1     { it _must_ be found }
  repne
  scasb           { scan until found }
  pop esi         { restore pointer to beginning of variable name }
  pop eax         { restore length of searched variable name }
  jmp @NewVar     { ... or continue with new variable otherwise }
@Stop:
  xor eax, eax
  mov P, eax      { Not found - return nil }
  jmp @End
@Equal:
  pop esi         { restore the stack position }
  pop eax
  mov P, edi      { place pointer to variable contents in P }
@End:
 end ['eax','ecx','edx','esi','edi'];
 GetEnvPChar := P;
end;
{$ASMMODE ATT}


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


procedure Sleep (Milliseconds: cardinal);

begin
 DosSleep (Milliseconds);
end;

function SysTimerTick: QWord;
var
  L: cardinal;
begin
  DosQuerySysInfo (svMsCount, svMsCount, L, 4);
  SysTimerTick := L;
end;

function ExecuteProcess (const Path: AnsiString; const ComLine: AnsiString;Flags:TExecuteFlags=[]):
                                                                       integer;
var
 E: EOSError;
 CommandLine: ansistring;
 Args0, Args: DosCalls.PByteArray;
 ObjNameBuf: PChar;
 ArgSize: word;
 Res: TResultCodes;
 ObjName: shortstring;
 RC: cardinal;
 ExecAppType: cardinal;
 MaxArgsSize: PtrUInt; (* Amount of memory reserved for arguments in bytes. *)
 MaxArgsSizeInc: word;

const
 ObjBufSize = 512;

function StartSession: cardinal;
var
 HQ: THandle;
 SPID, STID, QName: shortstring;
 SID, PID: cardinal;
 SD: TStartData;
 RD: TRequestData;
 PCI: PChildInfo;
 CISize: cardinal;
 Prio: byte;
begin
 Result := $FFFFFFFF;
 FillChar (SD, SizeOf (SD), 0);
 SD.Length := SizeOf (SD);
 SD.Related := ssf_Related_Child;
 if FileExists (Path) then
(* Full path necessary for starting different executable files from current *)
(* directory. *)
  CommandLine := ExpandFileName (Path)
 else
  CommandLine := Path;
 SD.PgmName := PChar (CommandLine);
 if ComLine <> '' then
  SD.PgmInputs := PChar (ComLine);
 if ExecInheritsHandles in Flags then
   SD.InheritOpt := ssf_InhertOpt_Parent;
 Str (GetProcessID, SPID);
 Str (ThreadID, STID);
 QName := '\QUEUES\FPC_ExecuteProcess_p' + SPID + 't' + STID + '.QUE'#0;
 SD.TermQ := @QName [1];
 SD.ObjectBuffer := ObjNameBuf;
 SD.ObjectBuffLen := ObjBufSize;
 RC := DosCreateQueue (HQ, quFIFO or quConvert_Address, @QName [1]);
 if RC <> 0 then
  begin
   Move (QName [1], ObjNameBuf^, Length (QName));
   OSErrorWatch (RC);
  end
 else
  begin
   RC := DosStartSession (SD, SID, PID);
   if (RC = 0) or (RC = 457) then
    begin
     RC := DosReadQueue (HQ, RD, CISize, PCI, 0, 0, Prio, 0);
     if RC = 0 then
      begin
       Result := PCI^.Return;
       RC := DosCloseQueue (HQ);
       if RC <> 0 then
        OSErrorWatch (RC);
       RC := DosFreeMem (PCI);
       if RC <> 0 then
        OSErrorWatch (RC);
       FreeMem (ObjNameBuf, ObjBufSize);
      end
     else
      begin
       OSErrorWatch (RC);
       RC := DosCloseQueue (HQ);
       OSErrorWatch (RC);
      end;
    end
   else
    begin
     OSErrorWatch (RC);
     RC := DosCloseQueue (HQ);
     if RC <> 0 then
      OSErrorWatch (RC);
    end;
  end;
end;

begin
 Result := integer ($FFFFFFFF);
 ObjName := '';
 GetMem (ObjNameBuf, ObjBufSize);
 FillChar (ObjNameBuf^, ObjBufSize, 0);

 RC := DosQueryAppType (PChar (Path), ExecAppType);
 if RC <> 0 then
  begin
   OSErrorWatch (RC);
   if (RC = 190) or (RC = 191) then
    Result := StartSession;
  end
 else
  begin
   if (ApplicationType and 3 = ExecAppType and 3) then
(* DosExecPgm should work... *)
    begin
     MaxArgsSize := Length (ComLine) + Length (Path) + 256; (* More than enough *)
     if MaxArgsSize > high (word) then
      Exit;
     if ComLine = '' then
      begin
       Args0 := nil;
       Args := nil;
      end
     else
      begin
       GetMem (Args0, MaxArgsSize);
       Args := Args0;
(* Work around a bug in OS/2 - argument to DosExecPgm *)
(* should not cross 64K boundary. *)
       while ((PtrUInt (Args) + MaxArgsSize) and $FFFF) < MaxArgsSize do
        begin
         MaxArgsSizeInc := MaxArgsSize -
                                    ((PtrUInt (Args) + MaxArgsSize) and $FFFF);
         Inc (MaxArgsSize, MaxArgsSizeInc);
         if MaxArgsSize > high (word) then
          Exit;
         ReallocMem (Args0, MaxArgsSize);
         Inc (pointer (Args), MaxArgsSizeInc);
        end;
       ArgSize := 0;
       Move (Path [1], Args^ [ArgSize], Length (Path));
       Inc (ArgSize, Length (Path));
       Args^ [ArgSize] := 0;
       Inc (ArgSize);
       {Now do the real arguments.}
       Move (ComLine [1], Args^ [ArgSize], Length (ComLine));
       Inc (ArgSize, Length (ComLine));
       Args^ [ArgSize] := 0;
       Inc (ArgSize);
       Args^ [ArgSize] := 0;
      end;
     Res.ExitCode := $FFFFFFFF;
     RC := DosExecPgm (ObjNameBuf, ObjBufSize, 0, Args, nil, Res,
                                                                 PChar (Path));
     if RC <> 0 then
      OSErrorWatch (RC);
     if Args0 <> nil then
      FreeMem (Args0, MaxArgsSize);
     if RC = 0 then
      begin
       Result := Res.ExitCode;
       FreeMem (ObjNameBuf, ObjBufSize);
      end
    end
  end;
 if RC <> 0 then
  begin
   ObjName := StrPas (ObjNameBuf);
   FreeMem (ObjNameBuf, ObjBufSize);
   if ComLine = '' then
    CommandLine := Path
   else
    CommandLine := Path + ' ' + ComLine;
   if ObjName = '' then
    E := EOSError.CreateFmt (SExecuteProcessFailed, [CommandLine, RC])
   else
    E := EOSError.CreateFmt (SExecuteProcessFailed + ' (' + ObjName + ')', [CommandLine, RC]);
   E.ErrorCode := Result;
   raise E;
  end;
end;


function ExecuteProcess (const Path: AnsiString;
                                  const ComLine: array of AnsiString;Flags:TExecuteFlags=[]): integer;

var
  CommandLine: AnsiString;
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


function GetTickCount: LongWord;
var
  L: cardinal;
begin
  DosQuerySysInfo (svMsCount, svMsCount, L, 4);
  GetTickCount := L;
end;

function GetTickCount64: QWord;
var
  Freq2: cardinal;
  T: QWord;
begin
  DosTmrQueryFreq (Freq2);
  DosTmrQueryTime (T);
  GetTickCount64 := T div (QWord (Freq2) div 1000);
{$NOTE GetTickCount64 takes 20 microseconds on 1GHz CPU, GetTickCount not measurable}
end;

const
  OrigOSErrorWatch: TOSErrorWatch = nil;

procedure TrackLastOSError (Error: cardinal);
begin
  LastOSError := Error;
  OrigOSErrorWatch (Error);
end;

function GetLastOSError: Integer;
begin
  GetLastOSError := Integer (LastOSError);
end;

{****************************************************************************
                              Initialization code
****************************************************************************}

Initialization
  InitExceptions;       { Initialize exceptions. OS independent }
  InitInternational;    { Initialize internationalization settings }
  OnBeep:=@SysBeep;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  LastOSError := 0;
  OrigOSErrorWatch := TOSErrorWatch (SetOSErrorTracking (@TrackLastOSError));
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Finalization
  DoneExceptions;
end.
