{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by the Free Pascal development team

    DOS unit for AmigaOS & clones

    Heavily based on the 1.x Amiga version by Nils Sjoholm and
    Carl Eric Codere

    AmigaOS and MorphOS support by Karoly Balogh
    AROS support by Marcus Sackrow

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$INLINE ON}

unit Dos;

{--------------------------------------------------------------------}
{ LEFT TO DO:                                                        }
{--------------------------------------------------------------------}
{ o DiskFree / Disksize don't work as expected                       }
{ o Implement EnvCount,EnvStr                                        }
{ o FindFirst should only work with correct attributes               }
{--------------------------------------------------------------------}


interface

type
  SearchRec = record
    { platform specific }
    AnchorPtr : Pointer;  { Pointer to the AnchorPath structure }
    AttrArg: Word;        { The initial Attributes argument }

    { generic }
    Attr : BYTE;        { attribute of found file }
    Time : LongInt;     { last modify date of found file }
    Size : LongInt;     { file size of found file }
    Name : String;      { name of found file }
  End;

{$I dosh.inc}

function DeviceByIdx(Idx: Integer): string;
function AddDisk(Const Path: string): Integer;
function RefreshDeviceList: Integer;
function DiskSize(Drive: AnsiString): Int64;
function DiskFree(Drive: AnsiString): Int64;

implementation

{$DEFINE HAS_GETMSCOUNT}
{$DEFINE HAS_GETCBREAK}
{$DEFINE HAS_SETCBREAK}

{$DEFINE FPC_FEXPAND_VOLUMES} (* Full paths begin with drive specification *)
{$DEFINE FPC_FEXPAND_DRIVESEP_IS_ROOT}
{$DEFINE FPC_FEXPAND_NO_DEFAULT_PATHS}
{$DEFINE FPC_FEXPAND_DIRSEP_IS_UPDIR}
{$I dos.inc}


{ * include OS specific functions & definitions * }

{$include execd.inc}
{$include execf.inc}
{$include timerd.inc}
{$include doslibd.inc}
{$include doslibf.inc}
{$include utilf.inc}

{$ifdef cpum68k}
{$if defined(amiga_v1_0_only) or defined(amiga_v1_2_only)}
{$include legacyexech.inc}
{$include legacydosh.inc}
{$include legacyutilh.inc}
{$endif}
{$endif}

{$packrecords default}

const
  DaysPerMonth :  Array[1..12] of ShortInt =
         (031,028,031,030,031,030,031,031,030,031,030,031);
  DaysPerYear  :  Array[1..12] of Integer  =
         (031,059,090,120,151,181,212,243,273,304,334,365);
  DaysPerLeapYear :    Array[1..12] of Integer  =
         (031,060,091,121,152,182,213,244,274,305,335,366);
  SecsPerYear      : LongInt  = 31536000;
  SecsPerLeapYear  : LongInt  = 31622400;
  SecsPerDay       : LongInt  = 86400;
  SecsPerHour      : Integer  = 3600;
  SecsPerMinute    : ShortInt = 60;


{******************************************************************************
                           --- Internal routines ---
******************************************************************************}

{ * PathConv is implemented in the system unit! * }
function PathConv(path: string): string; external name 'PATHCONV';

function dosLock(const name: String;
                 accessmode: Longint) : BPTR;
var
 buffer: array[0..255] of Char;
begin
  move(name[1],buffer,length(name));
  buffer[length(name)]:=#0;
  dosLock:=Lock(buffer,accessmode);
end;

function BADDR(bval: PtrInt): Pointer; Inline;
begin
  {$if defined(AROS)}  // deactivated for now //and (not defined(AROS_BINCOMPAT))}
  BADDR := Pointer(bval);
  {$else}
  BADDR:=Pointer(bval Shl 2);
  {$endif}
end;

function BSTR2STRING(s : Pointer): PChar; Inline;
begin
  {$if defined(AROS)}  // deactivated for now //and (not defined(AROS_BINCOMPAT))}
  BSTR2STRING:=PChar(s);
  {$else}
  BSTR2STRING:=PChar(BADDR(PtrInt(s)))+1;
  {$endif}
end;

function BSTR2STRING(s : PtrInt): PChar; Inline;
begin
  {$if defined(AROS)}  // deactivated for now //and (not defined(AROS_BINCOMPAT))}
  BSTR2STRING:=PChar(s);
  {$else}
  BSTR2STRING:=PChar(BADDR(s))+1;
  {$endif}
end;

function IsLeapYear(Source : Word) : Boolean;
begin
  if (source Mod 400 = 0) or ((source Mod 4 = 0) and (source Mod 100 <> 0)) then
    IsLeapYear:=True
  else
    IsLeapYear:=False;
end;

procedure AmigaDateStampToDateTime(var ds: TDateStamp; var dt: DateTime);
var
  cd: PClockData;
  time: LongInt;
begin
  new(cd);
  time := ds.ds_Days * (24 * 60 * 60) +
          ds.ds_Minute * 60 +
          ds.ds_Tick div TICKS_PER_SECOND;
  Amiga2Date(time,cd);
  with cd^ do
    begin
      dt.year:=year;
      dt.month:=month;
      dt.day:=mday;
      dt.hour:=hour;
      dt.min:=min;
      dt.sec:=sec;
    end;
  dispose(cd);
end;

procedure Amiga2DateStamp(Date : LongInt; var TotalDays,Minutes,Ticks: longint);
{ Converts a value in seconds past 1978 to a value in AMIGA DateStamp format }
{ Taken from SWAG and modified to work with the Amiga format - CEC           }
var
  LocalDate : LongInt;
  Done : Boolean;
  TotDays : Integer;
  Y: Word;
  H: Word;
  Min: Word;
  S : Word;
begin
  Y   := 1978; H := 0; Min := 0; S := 0;
  TotalDays := 0;
  Minutes := 0;
  Ticks := 0;
  LocalDate := Date;
  Done := false;
  while not Done do
  begin
    if LocalDate >= SecsPerYear then
    begin
      Inc(Y,1);
      Dec(LocalDate,SecsPerYear);
      Inc(TotalDays,DaysPerYear[12]);
    end else
      Done := true;
    if (IsLeapYear(Y+1)) and (LocalDate >= SecsPerLeapYear) and
       (Not Done) then
    begin
      Inc(Y,1);
      Dec(LocalDate,SecsPerLeapYear);
      Inc(TotalDays,DaysPerLeapYear[12]);
    end;
  end; { END WHILE }

  TotDays := LocalDate Div SecsPerDay;
  { Total number of days }
  TotalDays := TotalDays + TotDays;
  Dec(LocalDate,TotDays*SecsPerDay);
  { Absolute hours since start of day }
  H := LocalDate Div SecsPerHour;
  { Convert to minutes }
  Minutes := H*60;
  Dec(LocalDate,(H * SecsPerHour));
  { Find the remaining minutes to add }
  Min := LocalDate Div SecsPerMinute;
  Dec(LocalDate,(Min * SecsPerMinute));
  Minutes:=Minutes+Min;
  { Find the number of seconds and convert to ticks }
  S := LocalDate;
  Ticks:=TICKS_PER_SECOND*S;
end;


function dosSetProtection(const name: string; mask:longint): Boolean;
var
  buffer : array[0..255] of Char;
begin
  move(name[1],buffer,length(name));
  buffer[length(name)]:=#0;
  dosSetProtection:=SetProtection(buffer,mask) <> 0;
end;

function dosSetFileDate(const name: string; p : PDateStamp): Boolean;
var
  buffer : array[0..255] of Char;
begin
  move(name[1],buffer,length(name));
  buffer[length(name)]:=#0;
  dosSetFileDate:=SetFileDate(buffer,p);
end;


{******************************************************************************
                        --- Info / Date / Time ---
******************************************************************************}

function DosVersion: Word;
var p: PLibrary;
begin
  p:=PLibrary(AOS_DOSBase);
  DosVersion:= p^.lib_Version or (p^.lib_Revision shl 8);
end;

{ Here are a lot of stuff just for setdate and settime }

var
  TimerBase : Pointer;


procedure NewList (list: pList);
begin
  with list^ do begin
    lh_Head     := pNode(@lh_Tail);
    lh_Tail     := NIL;
    lh_TailPred := pNode(@lh_Head)
  end;
end;

function CreateExtIO (port: pMsgPort; size: Longint): pIORequest;
var
   IOReq: pIORequest;
begin
    IOReq := NIL;
    if port <> NIL then
    begin
        IOReq := execAllocMem(size, MEMF_CLEAR);
        if IOReq <> NIL then
        begin
            IOReq^.io_Message.mn_Node.ln_Type   := 7;
            IOReq^.io_Message.mn_Length    := size;
            IOReq^.io_Message.mn_ReplyPort := port;
        end;
    end;
    CreateExtIO := IOReq;
end;

procedure DeleteExtIO (ioReq: pIORequest);
begin
    if ioReq <> NIL then
    begin
        ioReq^.io_Message.mn_Node.ln_Type := $FF;
        ioReq^.io_Message.mn_ReplyPort    := pMsgPort(-1);
        ioReq^.io_Device                  := pDevice(-1);
        execFreeMem(ioReq, ioReq^.io_Message.mn_Length);
    end
end;

function Createport(name : PChar; pri : longint): pMsgPort;
var
   sigbit : ShortInt;
   port    : pMsgPort;
begin
   sigbit := AllocSignal(-1);
   if sigbit = -1 then CreatePort := nil;
   port := execAllocMem(sizeof(tMsgPort),MEMF_CLEAR);
   if port = nil then begin
      FreeSignal(sigbit);
      CreatePort := nil;
   end;
   with port^ do begin
       if assigned(name) then
       mp_Node.ln_Name := name
       else mp_Node.ln_Name := nil;
       mp_Node.ln_Pri := pri;
       mp_Node.ln_Type := 4;
       mp_Flags := 0;
       mp_SigBit := sigbit;
       mp_SigTask := FindTask(nil);
   end;
   if assigned(name) then AddPort(port)
   else NewList(addr(port^.mp_MsgList));
   CreatePort := port;
end;

procedure DeletePort (port: pMsgPort);
begin
    if port <> NIL then
    begin
        if port^.mp_Node.ln_Name <> NIL then
            RemPort(port);

        port^.mp_Node.ln_Type     := $FF;
        port^.mp_MsgList.lh_Head  := pNode(-1);
        FreeSignal(port^.mp_SigBit);
        execFreeMem(port, sizeof(tMsgPort));
    end;
end;


function Create_Timer(theUnit : longint) : pTimeRequest;
var
  Error : longint;
  TimerPort : pMsgPort;
  TimeReq : pTimeRequest;
begin
  TimerPort := CreatePort(Nil, 0);
  if TimerPort = Nil then
    Create_Timer := Nil;
  TimeReq := pTimeRequest(CreateExtIO(TimerPort,sizeof(tTimeRequest)));
  if TimeReq = Nil then begin
    DeletePort(TimerPort);
    Create_Timer := Nil;
  end;
  Error := OpenDevice(TIMERNAME, theUnit, pIORequest(TimeReq), 0);
  if Error <> 0 then begin
    DeleteExtIO(pIORequest(TimeReq));
    DeletePort(TimerPort);
    Create_Timer := Nil;
  end;
  TimerBase := pointer(TimeReq^.tr_Node.io_Device);
  Create_Timer := pTimeRequest(TimeReq);
end;

Procedure Delete_Timer(WhichTimer : pTimeRequest);
var
    WhichPort : pMsgPort;
begin

    WhichPort := WhichTimer^.tr_Node.io_Message.mn_ReplyPort;
    if assigned(WhichTimer) then begin
        CloseDevice(pIORequest(WhichTimer));
        DeleteExtIO(pIORequest(WhichTimer));
    end;
    if assigned(WhichPort) then
        DeletePort(WhichPort);
end;

function set_new_time(secs, micro : longint): longint;
var
    tr : ptimerequest;
begin
    tr := create_timer(UNIT_MICROHZ);

    { non zero return says error }
    if tr = nil then set_new_time := -1;

    tr^.tr_time.tv_secs := secs;
    tr^.tr_time.tv_micro := micro;
    tr^.tr_node.io_Command := TR_SETSYSTIME;
    DoIO(pIORequest(tr));

    delete_timer(tr);
    set_new_time := 0;
end;

function get_sys_time(tv : ptimeval): longint;
var
    tr : ptimerequest;
begin
    tr := create_timer( UNIT_MICROHZ );

    { non zero return says error }
    if tr = nil then get_sys_time := -1;

    tr^.tr_node.io_Command := TR_GETSYSTIME;
    DoIO(pIORequest(tr));

    { structure assignment }
    tv^ := tr^.tr_time;

    delete_timer(tr);
    get_sys_time := 0;
end;

procedure GetDate(Var Year, Month, MDay, WDay: Word);
var
  cd    : pClockData;
  oldtime : ttimeval;
begin
  new(cd);
  get_sys_time(@oldtime);
  Amiga2Date(oldtime.tv_secs,cd);
  Year  := cd^.year;
  Month := cd^.month;
  MDay  := cd^.mday;
  WDay  := cd^.wday;
  dispose(cd);
end;

procedure SetDate(Year, Month, Day: Word);
var
  cd : pClockData;
  oldtime : ttimeval;
begin
  new(cd);
  get_sys_time(@oldtime);
  Amiga2Date(oldtime.tv_secs,cd);
  cd^.year := Year;
  cd^.month := Month;
  cd^.mday := Day;
  set_new_time(Date2Amiga(cd),0);
  dispose(cd);
end;

procedure GetTime(Var Hour, Minute, Second, Sec100: Word);
var
  cd      : pClockData;
  oldtime : ttimeval;
begin
  new(cd);
  get_sys_time(@oldtime);
  Amiga2Date(oldtime.tv_secs,cd);
  Hour   := cd^.hour;
  Minute := cd^.min;
  Second := cd^.sec;
  Sec100 := oldtime.tv_micro div 10000;
  dispose(cd);
end;


Procedure SetTime(Hour, Minute, Second, Sec100: Word);
var
  cd : pClockData;
  oldtime : ttimeval;
begin
  new(cd);
  get_sys_time(@oldtime);
  Amiga2Date(oldtime.tv_secs,cd);
  cd^.hour := Hour;
  cd^.min := Minute;
  cd^.sec := Second;
  set_new_time(Date2Amiga(cd), Sec100 * 10000);
  dispose(cd);
end;


function GetMsCount: int64;
var
  TV: TTimeVal;
begin
  Get_Sys_Time (@TV);
  GetMsCount := int64 (TV.TV_Secs) * 1000 + TV.TV_Micro div 1000;
end;

{******************************************************************************
                               --- Exec ---
******************************************************************************}


procedure Exec(const Path: PathStr; const ComLine: ComStr);
var
  tmpPath: array[0..515] of char;
  result : longint;
  tmpLock: BPTR;
begin
  DosError:= 0;
  LastDosExitCode:=0;
  tmpPath:=PathConv(Path)+#0+ComLine+#0; // hacky... :)

  { Here we must first check if the command we wish to execute }
  { actually exists, because this is NOT handled by the        }
  { _SystemTagList call (program will abort!!)                 }

  { Try to open with shared lock                               }
  tmpLock:=Lock(tmpPath,SHARED_LOCK);
  if tmpLock<>0 then
    begin
      { File exists - therefore unlock it }
      Unlock(tmpLock);
      tmpPath[length(Path)]:=' '; // hacky... replaces first #0 from above, to get the whole string. :)
      result:=SystemTagList(tmpPath,nil);
      { on return of -1 the shell could not be executed }
      { probably because there was not enough memory    }
      if result = -1 then
        DosError:=8
      else
        LastDosExitCode:=word(result);
    end
  else
    DosError:=3;
end;


procedure GetCBreak(Var BreakValue: Boolean);
begin
  breakvalue := system.BreakOn;
end;

procedure SetCBreak(BreakValue: Boolean);
begin
  system.Breakon := BreakValue;
end;


{******************************************************************************
                               --- Disk ---
******************************************************************************}
const
  PROC_WIN_DISABLE = Pointer(-1);
  PROC_WIN_WB      = Pointer(0);

function SetProcessWinPtr(p: Pointer): Pointer; inline;
var
  MyProc: PProcess;
begin
  MyProc := PProcess(FindTask(Nil));
  SetProcessWinPtr := MyProc^.pr_WindowPtr;
  MyProc^.pr_WindowPtr := p;
end;

{
  The Diskfree and Disksize functions need a file on the specified drive, since this
  is required for the statfs system call.
  These filenames are set in drivestr[0..26], and have been preset to :
   0 - ':'      (default drive - hence current dir is ok.)
   1 - 'DF0:'   (floppy drive 1 - should be adapted to local system )
   2 - 'DF1:'   (floppy drive 2 - should be adapted to local system )
   3 - 'SYS:'   (C: equivalent of dos is the SYS: partition)
   4..26          (can be set by you're own applications)
  ! Use AddDisk() to Add new drives !
  They both return -1 when a failure occurs.
}
var
  DeviceList: array[0..26] of string[20];
  NumDevices: Integer = 0;

const
  IllegalDevices: array[0..12] of string =(
                   'PED:',
                   'PRJ:',
                   'PIPE:',   // Pipes
                   'XPIPE:',  // Extented Pipe
                   'CON:',    // Console
                   'RAW:',    // RAW: Console
                   'KCON:',   // KingCON Console
                   'KRAW:',   // KingCON RAW
                   'SER:',    // serial Ports
                   'SER0:',
                   'SER1:',
                   'PAR:',    // Parallel Porty
                   'PRT:');   // Printer

function IsIllegalDevice(DeviceName: string): Boolean;
var
  i: Integer;
  Str: AnsiString;
begin
  IsIllegalDevice := False;
  Str := UpCase(DeviceName);
  for i := Low(IllegalDevices) to High(IllegalDevices) do
  begin
    if Str = IllegalDevices[i] then
    begin
      IsIllegalDevice := True;
      Exit;
    end;
  end;
end;

function DeviceByIdx(Idx: Integer): string;
begin
  DeviceByIdx := '';
  if (Idx < 0) or (Idx >= NumDevices) then
    Exit;
  DeviceByIdx := DeviceList[Idx];
end;

function AddDisk(const Path: string): Integer;
begin
  // if hit border, restart at 4
  if NumDevices > 26 then
    NumDevices := 4;
  // set the device
  DeviceList[NumDevices] := Copy(Path, 1, 20);
  // return the Index increment for next run
  AddDisk := NumDevices;
  Inc(NumDevices);
end;

function RefreshDeviceList: Integer;
var
  List: PDosList;
  Temp: PChar;
  Str: string;
begin
  NumDevices := 0;
  AddDisk(':');          // Index 0
  AddDisk('DF0:');       // Index 1
  AddDisk('DF1:');       // Index 2
  AddDisk('SYS:');       // Index 3
  // Lock the List
  List := LockDosList(LDF_DEVICES or LDF_READ);
  // Inspect the List
  repeat
    List := NextDosEntry(List, LDF_DEVICES);
    if List <> nil then
    begin
      Temp := BSTR2STRING(List^.dol_Name);
      Str := strpas(Temp) + ':';
      if not IsIllegalDevice(str) then
        AddDisk(Str);
    end;
  until List = nil;
  UnLockDosList(LDF_DEVICES or LDF_READ);
  RefreshDeviceList := NumDevices;
end;

// New easier DiskSize()
//
function DiskSize(Drive: AnsiString): Int64;
var
  DirLock: BPTR;
  Inf: TInfoData;
  OldWinPtr: Pointer;
begin
  DiskSize := -1;
  //
  OldWinPtr:=SetProcessWinPtr(PROC_WIN_DISABLE);
  //
  DirLock := Lock(PChar(Drive), SHARED_LOCK);
  if DirLock <> 0 then
  begin
    if Info(DirLock, @Inf) <> 0 then
      DiskSize := Int64(Inf.id_NumBlocks) * Inf.id_BytesPerBlock;
    UnLock(DirLock);
  end;
  SetProcessWinPtr(OldWinPtr);
end;

function DiskSize(Drive: Byte): Int64;
begin
  DiskSize := -1;
  if (Drive >= NumDevices) then
    Exit;
  DiskSize := DiskSize(DeviceList[Drive]);
end;

// New easier DiskFree()
//
function DiskFree(Drive: AnsiString): Int64;
var
  DirLock: BPTR;
  Inf: TInfoData;
  OldWinPtr: Pointer;
begin
  DiskFree := -1;
  //
  OldWinPtr:=SetProcessWinPtr(PROC_WIN_DISABLE);
  //
  DirLock := Lock(PChar(Drive), SHARED_LOCK);
  if DirLock <> 0 then
  begin
    if Info(DirLock, @Inf) <> 0 then
      DiskFree := Int64(Inf.id_NumBlocks - Inf.id_NumBlocksUsed) * Inf.id_BytesPerBlock;
    UnLock(DirLock);
  end;
  SetProcessWinPtr(OldWinPtr);
end;

function DiskFree(Drive: Byte): Int64;
begin
  DiskFree := -1;
  if (Drive >= NumDevices) then
    Exit;
  DiskFree := DiskFree(DeviceList[Drive]);
end;

procedure FindMatch(Result: LongInt; var f: SearchRec);
var
  quit: boolean;
  dt: DateTime;
begin
  DosError:=0;
  quit:=false;
  while not quit do
    begin
      if Result = ERROR_NO_MORE_ENTRIES then
        DosError:=18
      else
        if Result<>0 then DosError:=3;
      if DosError=0 then
        begin
          { if we're not looking for a directory, but we found one, try to skip it }
          if ((f.AttrArg and Directory) = 0) and (PAnchorPath(f.AnchorPtr)^.ap_Info.fib_DirEntryType > 0) then
            Result:=MatchNext(f.AnchorPtr)
          else
            quit:=true;
        end
      else
        quit:=true;
    end;

  if DosError=0 then begin
    { Fill up the Searchrec information     }
    { and also check if the files are with  }
    { the correct attributes                }
    with PAnchorPath(f.AnchorPtr)^.ap_Info do begin

      { Convert Amiga DateStamp to DOS file time }
      AmigaDateStampToDateTime(fib_Date,dt);
      PackTime(dt,f.time);

      f.attr := 0;
      {*------------------------------------*}
      {* Determine if is a file or a folder *}
      {*------------------------------------*}
      if fib_DirEntryType > 0 then f.attr:=f.attr OR DIRECTORY;

      {*------------------------------------* }
      {* Determine if Read only             *}
      {*  Readonly if R flag on and W flag  *}
      {*   off.                             *}
      {* Should we check also that EXEC     *}
      {* is zero? for read only?            *}
      {*------------------------------------*}
      if ((fib_Protection and FIBF_READ) <> 0) and
         ((fib_Protection and FIBF_WRITE) = 0) then f.attr:=f.attr or READONLY;
      f.Name := strpas(fib_FileName);
      f.Size := fib_Size;
    end; { end with }
  end;
end;

procedure FindFirst(const Path: PathStr; Attr: Word; Var f: SearchRec);
var
 tmpStr: array[0..255] of Char;
 Anchor: PAnchorPath;
begin
  tmpStr:=PathConv(path)+#0;

  new(Anchor);
  FillChar(Anchor^,sizeof(TAnchorPath),#0);

  f.AnchorPtr:=Anchor;
  f.AttrArg:=Attr;

  FindMatch(MatchFirst(@tmpStr,Anchor),f);
end;

procedure FindNext(Var f: SearchRec);
var
 Result: longint;
begin
  FindMatch(MatchNext(f.AnchorPtr),f);
end;

procedure FindClose(Var f: SearchRec);
begin
  MatchEnd(f.AnchorPtr);
  if assigned(f.AnchorPtr) then
    Dispose(PAnchorPath(f.AnchorPtr));
end;


{******************************************************************************
                               --- File ---
******************************************************************************}

function FSearch(path: PathStr; dirlist: String) : PathStr;
var
  p1     : LongInt;
  tmpSR  : SearchRec;
  newdir : PathStr;
begin
  { No wildcards allowed in these things }
  if (pos('?',path)<>0) or (pos('*',path)<>0) or (path='') then
  begin
    FSearch:='';
    exit;
  end;
  { check if the file specified exists }
  findfirst(path,anyfile and not(directory), tmpSR);
  if doserror=0 then
  begin
    findclose(tmpSR);
    fsearch:=path;
    exit;
  end;
  findclose(tmpSR);

  repeat
    p1:=pos(';',dirlist);
    if p1<>0 then
    begin
      newdir:=Copy(dirlist,1,p1-1);
      Delete(dirlist,1,p1);
    end
    else
    begin
      newdir:=dirlist;
      dirlist:='';
    end;
    if (newdir<>'') and (not (newdir[length(newdir)] in [DirectorySeparator, DriveSeparator])) then
      newdir:=newdir+DirectorySeparator;
    FindFirst(newdir+path,anyfile and not(directory),tmpSR);
    if doserror=0 then
      newdir:=newdir+path
    else
      newdir:='';
    findclose(tmpSR);
  until (dirlist='') or (newdir<>'');
  FSearch:=newdir;
end;


Procedure getftime (var f; var time : longint);
{
    This function returns a file's date and time as the number of
    seconds after January 1, 1978 that the file was created.
}
var
    FInfo : pFileInfoBlock;
    FTime : Longint;
    FLock : BPTR;
    Str   : String;
    i     : integer;
begin
    DosError:=0;
    FTime := 0;
{$ifdef FPC_ANSI_TEXTFILEREC}
    Str := strpas(filerec(f).Name);
{$else}
    Str := ToSingleByteFileSystemEncodedFileName(filerec(f).Name);
{$endif}
    DoDirSeparators(Str);
    FLock := dosLock(Str, SHARED_LOCK);
    IF FLock <> 0 then begin
        New(FInfo);
        if Examine(FLock, FInfo) <> 0 then begin
             with FInfo^.fib_Date do
             FTime := ds_Days * (24 * 60 * 60) +
             ds_Minute * 60 +
             ds_Tick div TICKS_PER_SECOND;
        end else begin
             FTime := 0;
        end;
        Unlock(FLock);
        Dispose(FInfo);
    end
    else
     DosError:=6;
    time := FTime;
end;


  Procedure setftime(var f; time : longint);
   var
    DateStamp: pDateStamp;
    Str: String;
    i: Integer;
    Days, Minutes,Ticks: longint;
    FLock: BPTR;
  Begin
    new(DateStamp);
{$ifdef FPC_ANSI_TEXTFILEREC}
    Str := strpas(filerec(f).Name);
{$else}
    Str := ToSingleByteFileSystemEncodedFileName(filerec(f).Name);
{$endif}
    DoDirSeparators(str);
    { Check first of all, if file exists }
    FLock := dosLock(Str, SHARED_LOCK);
    IF FLock <> 0 then
      begin
        Unlock(FLock);
        Amiga2DateStamp(time,Days,Minutes,ticks);
        DateStamp^.ds_Days:=Days;
        DateStamp^.ds_Minute:=Minutes;
        DateStamp^.ds_Tick:=Ticks;
        if dosSetFileDate(Str,DateStamp) then
            DosError:=0
        else
            DosError:=6;
      end
    else
      DosError:=2;
    if assigned(DateStamp) then Dispose(DateStamp);
  End;

procedure getfattr(var f; var attr : word);
var
    info : pFileInfoBlock;
    MyLock : BPTR;
    flags: word;
    Str: String;
    i: integer;
begin
    DosError:=0;
    flags:=0;
    New(info);
{$ifdef FPC_ANSI_TEXTFILEREC}
    Str := strpas(filerec(f).Name);
{$else}
    Str := ToSingleByteFileSystemEncodedFileName(filerec(f).Name);
{$endif}
    DoDirSeparators(str);
    { open with shared lock to check if file exists }
    MyLock:=dosLock(Str,SHARED_LOCK);
    if MyLock <> 0 then
      Begin
        Examine(MyLock,info);
        {*------------------------------------*}
        {* Determine if is a file or a folder *}
        {*------------------------------------*}
        if info^.fib_DirEntryType > 0 then
             flags:=flags OR DIRECTORY;

        {*------------------------------------*}
        {* Determine if Read only             *}
        {*  Readonly if R flag on and W flag  *}
        {*   off.                             *}
        {* Should we check also that EXEC     *}
        {* is zero? for read only?            *}
        {*------------------------------------*}
        if   ((info^.fib_Protection and FIBF_READ) <> 0)
         AND ((info^.fib_Protection and FIBF_WRITE) = 0)
         then
          flags:=flags OR ReadOnly;
        Unlock(mylock);
      end
    else
      DosError:=3;
    attr:=flags;
    Dispose(info);
  End;


procedure setfattr(var f; attr : word);
var
  flags: longint;
  tmpLock : BPTR;
{$ifndef FPC_ANSI_TEXTFILEREC}
  r : rawbytestring;
{$endif not FPC_ANSI_TEXTFILEREC}
  p : pchar;
begin
{$ifdef FPC_ANSI_TEXTFILEREC}
  p := @filerec(f).Name;
{$else}
  r := ToSingleByteFileSystemEncodedFileName(filerec(f).Name);
  p := pchar(r);
{$endif}
  DosError:=0;
  flags:=FIBF_WRITE;

  { By default files are read-write }
  if attr and ReadOnly <> 0 then flags:=FIBF_READ; { Clear the Fibf_write flags }

  { no need for path conversion here, because file opening already }
  { converts the path (KB) }

  { create a shared lock on the file }
  tmpLock:=Lock(p,SHARED_LOCK);
  if tmpLock <> 0 then begin
    Unlock(tmpLock);
    if SetProtection(p,flags) = 0 then DosError:=5;
  end else
    DosError:=3;
end;



{******************************************************************************
                             --- Environment ---
******************************************************************************}

var
  strofpaths : string;

function SystemTags(const command: PChar; const tags: array of PtrUInt): LongInt;
begin
  SystemTags:=SystemTagList(command,@tags);
end;

function getpathstring: string;
var
   f : text;
   s : string;
   found : boolean;
   temp : string[255];
begin
   found := true;
   temp := '';

   { Alternatively, this could use PIPE: handler on systems which
     have this by default (not the case on classic Amiga), but then
     the child process should be started async, which for a simple
     Path command probably isn't worth the trouble. (KB) }
   assign(f,'T:'+HexStr(FindTask(nil))+'_path.tmp');
   rewrite(f);
   { This is a pretty ugly stunt, combining Pascal and Amiga system
     functions, but works... }
   SystemTags('C:Path',[SYS_Input, 0, SYS_Output, TextRec(f).Handle, TAG_END]);
   close(f);

   reset(f);
   { skip the first line, garbage }
   if not eof(f) then readln(f,s);
   while not eof(f) do begin
      readln(f,s);
      if found then begin
         temp := s;
         found := false;
      end else begin
         if (length(s) + length(temp)) < 255 then
            temp := temp + ';' + s;
      end;
   end;
   close(f);
   erase(f);

   getpathstring := temp;
end;

var
  EnvList: array of record
    Name: string;
    Local: Boolean;
    Value: string;
  end;

procedure InitEnvironmentStrings;
Const
  BUFFER_SIZE       = 254;
Var
  ThisProcess: PProcess;
  LocalVars_List: PMinList;  // Local Var structure in struct process (pr_LocalVarsis is actually a minlist
  LocalVar_Node: PLocalVar;
  Buffer: array[0..BUFFER_SIZE] of Char; // Buffer to hold a value for GetVar()
  TempLen: LongInt;      // hold returnlength of GetVar()
  // for env: searching
  Anchor: TAnchorPath;
  Res: Integer;
begin
  SetLength(EnvList, 0);
  ThisProcess := PProcess(FindTask(nil));  //Get the pointer to our process
  LocalVars_List := @(ThisProcess^.pr_LocalVars);  //get the list of pr_LocalVars as pointer
  LocalVar_Node  := pLocalVar(LocalVars_List^.mlh_head); //get the headnode of the LocalVars list

  // loop through the localvar list
  while ( Pointer(LocalVar_Node^.lv_node.ln_Succ) <> Pointer(LocalVars_List^.mlh_Tail)) do
  begin
    // make sure the active node is valid instead of empty
    If not(LocalVar_Node <> nil) then
      break;

    { - process the current node - }
    If (LocalVar_Node^.lv_node.ln_Type = LV_Var) then
    begin
      FillChar(Buffer[0], Length(Buffer), #0); // clear Buffer

      // get active node's name environment variable value ino buffer and make sure it's local
      TempLen := GetVar(LocalVar_Node^.lv_Node.ln_Name, @Buffer[0], BUFFER_SIZE, GVF_LOCAL_ONLY);
      If TempLen <> -1 then
      begin
        SetLength(EnvList, Length(EnvList) + 1);
        EnvList[High(EnvList)].Name := LocalVar_Node^.lv_Node.ln_Name;
        EnvList[High(EnvList)].Value := string(PChar(@Buffer[0]));
        EnvList[High(EnvList)].Local := True;
      end;
    end;
    LocalVar_Node := pLocalVar(LocalVar_Node^.lv_node.ln_Succ); //we need to get the next node
  end;
  // search in env for all Variables
  FillChar(Anchor,sizeof(TAnchorPath),#0);
  Res := MatchFirst('ENV:#?', @Anchor);
  while Res = 0 do
  begin
    if Anchor.ap_Info.fib_DirEntryType <= 0 then
    begin
      SetLength(EnvList, Length(EnvList) + 1);
      EnvList[High(EnvList)].Name := Anchor.ap_Info.fib_FileName;
      EnvList[High(EnvList)].Value := '';
      EnvList[High(EnvList)].Local := False;
    end;
    Res := MatchNext(@Anchor);
  end;
  MatchEnd(@Anchor);
  // add PATH as Fake Variable:
  SetLength(EnvList, Length(EnvList) + 1);
  EnvList[High(EnvList)].Name := 'PATH';
  EnvList[High(EnvList)].Value := '';
  EnvList[High(EnvList)].Local := False;
end;


function EnvCount: Longint;
begin
  InitEnvironmentStrings;
  EnvCount := Length(EnvList);
end;


function GetEnvFromEnv(envvar : String): String;
var
   bufarr : array[0..255] of char;
   strbuffer : array[0..255] of char;
   temp : Longint;
begin
   GetEnvFromEnv := '';
   if UpCase(envvar) = 'PATH' then begin
       if StrOfpaths = '' then StrOfPaths := GetPathString;
       GetEnvFromEnv := StrOfPaths;
   end else begin
      if (Pos(DriveSeparator,envvar) <> 0) or
         (Pos(DirectorySeparator,envvar) <> 0) then exit;
      move(envvar[1],strbuffer,length(envvar));
      strbuffer[length(envvar)] := #0;
      temp := GetVar(strbuffer,bufarr,255,$100);
      if temp <> -1 then
         GetEnvFromEnv := StrPas(bufarr);
   end;
end;

function EnvStr(Index: LongInt): String;
begin
  EnvStr := '';
  if Length(EnvList) = 0 then
    InitEnvironmentStrings;
  if (Index >= 0) and (Index <= High(EnvList)) then
  begin
    if EnvList[Index].Local then
      EnvStr := EnvList[Index].Name + '=' + EnvList[Index].Value
    else
      EnvStr := EnvList[Index].Name + '=' + GetEnvFromEnv(EnvList[Index].Name);
  end;
end;

function GetEnv(envvar : String): String;
var
  EnvVarName: String;
  i: Integer;
begin
  GetEnv := '';
  EnvVarName := UpCase(EnvVar);
  if EnvVarName = 'PATH' then
  begin
    if StrOfpaths = '' then
      StrOfPaths := GetPathString;
    GetEnv := StrOfPaths;
  end else
  begin
    InitEnvironmentStrings;
    for i := 0 to High(EnvList) do
    begin
      if EnvVarName = UpCase(EnvList[i].Name) then
      begin
        if EnvList[i].Local then
          GetEnv := EnvList[i].Value
        else
          GetEnv := GetEnvFromEnv(EnvList[i].Name);
        Break;
      end;
    end;
  end;
end;

begin
  DosError:=0;
  StrOfPaths := '';
  RefreshDeviceList;
end.
