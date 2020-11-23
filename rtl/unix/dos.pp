{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt and Peter Vreman,
    members of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
Unit Dos;
Interface

uses baseunix;

Const
  FileNameLen = 255;

Type
  SearchRec =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
    packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    Record
  {Fill : array[1..21] of byte;  Fill replaced with below}
    SearchPos  : TOff;        {directory position}
    SearchNum  : LongInt;     {to track which search this is}
    DirPtr     : Pointer;     {directory pointer for reading directory}
    SearchType : Byte;        {0=normal, 1=open will close, 2=only 1 file}
    SearchAttr : Byte;        {attribute we are searching for}
    Mode       : Word;
    Fill       : Array[1..1] of Byte; {future use}
  {End of fill}
    Attr       : Byte;        {attribute of found file}
    Time       : LongInt;     {last modify date of found file}
    Size       : LongInt;     {file size of found file}
    Reserved   : Word;        {future use}
    Name       : String[FileNameLen]; {name of found file}
    SearchSpec : String[FileNameLen]; {search pattern}
    NamePos    : Word;        {end of path, start of name position}
  End;

{$DEFINE HAS_FILENAMELEN}
{$i dosh.inc}

{Extra Utils}
function weekday(y,m,d : longint) : longint; platform;
Procedure UnixDateToDt(SecsPast: LongInt; Var Dt: DateTime); platform;
Function  DTToUnixDate(DT: DateTime): LongInt; platform;

{Disk}
Function AddDisk(const path:string) : byte; platform;

Implementation

Uses
  UnixUtil,
  Strings,
  Unix,
  {$ifdef FPC_USE_LIBC}initc{$ELSE}Syscall{$ENDIF};

{$DEFINE HAS_GETMSCOUNT}

{$DEFINE FPC_FEXPAND_TILDE} { Tilde is expanded to home }
{$DEFINE FPC_FEXPAND_GETENVPCHAR} { GetEnv result is a PChar }

{$I dos.inc}


{******************************************************************************
                           --- Link C Lib if set ---
******************************************************************************}

type
  RtlInfoType = Record
    FMode,
    FInode,
    FUid,
    FGid,
    FSize,
    FMTime : LongInt;
  End;


{******************************************************************************
                        --- Info / Date / Time ---
******************************************************************************}

Function DosVersion:Word;
Var
  Buffer : Array[0..255] of Char;
  Tmp2,
  TmpStr : String[40];
  TmpPos,
  SubRel,
  Rel    : LongInt;
  info   : utsname;
Begin
  FPUName(info);
  Move(info.release,buffer[0],40);
  TmpStr:=StrPas(Buffer);
  SubRel:=0;
  TmpPos:=Pos('.',TmpStr);
  if TmpPos>0 then
   begin
     Tmp2:=Copy(TmpStr,TmpPos+1,40);
     Delete(TmpStr,TmpPos,40);
   end;
  TmpPos:=Pos('.',Tmp2);
  if TmpPos>0 then
   Delete(Tmp2,TmpPos,40);
  Val(TmpStr,Rel);
  Val(Tmp2,SubRel);
  DosVersion:=Rel+(SubRel shl 8);
End;

function WeekDay (y,m,d:longint):longint;
{
  Calculates th day of the week. returns -1 on error
}
var
  u,v : longint;
begin
  if (m<1) or (m>12) or (y<1600) or (y>4000) or
     (d<1) or (d>30+((m+ord(m>7)) and 1)-ord(m=2)) or
     ((m*d=58) and (((y mod 4>0) or (y mod 100=0)) and (y mod 400>0))) then
   WeekDay:=-1
  else
   begin
     u:=m;
     v:=y;
     if m<3 then
      begin
        inc(u,12);
        dec(v);
      end;
     WeekDay:=(d+2*u+((3*(u+1)) div 5)+v+(v div 4)-(v div 100)+(v div 400)+1) mod 7;
   end;
end;


Procedure GetDate(Var Year, Month, MDay, WDay: Word);
var
  tz:timeval;
  hour,min,sec : word;
begin
  fpgettimeofday(@tz,nil);
  EpochToLocal(tz.tv_sec,year,month,mday,hour,min,sec);
  Wday:=weekday(Year,Month,MDay);
end;


procedure  SetTime(Hour,Minute,Second,sec100:word);
var
  dow,Year, Month, Day : Word;

  tv : timeval;
begin
  GetDate (Year, Month, Day,dow);
  tv.tv_sec:= LocalToEpoch ( Year, Month, Day, Hour, Minute, Second ) ;
  tv.tv_usec:= Sec100 * 10000;
  fpSettimeofday(@tv,nil);
end;

procedure SetDate(Year,Month,Day:Word);
var
  Hour, Min, Sec, Sec100 : Word;
  tv : timeval;
begin
  GetTime ( Hour, Min, Sec, Sec100 );
  tv.tv_sec:= LocalToEpoch ( Year, Month, Day, Hour, Min, Sec ) ;
  tv.tv_usec:= Sec100 * 10000;
  fpSettimeofday(@tv,nil);
end;


Function SetDateTime(Year,Month,Day,hour,minute,second:Word) : Boolean;
var
  tv : timeval;
begin
  tv.tv_sec:= LocalToEpoch ( Year, Month, Day, Hour, Minute, Second ) ;
  tv.tv_usec:= 0;
  SetDatetime:=fpSettimeofday(@tv,nil)=0;
end;


Procedure GetTime(Var Hour, Minute, Second, Sec100: Word);
var
  tz:timeval;
  year,month,day : word;
begin
  fpgettimeofday(@tz,nil);
  EpochToLocal(tz.tv_sec,year,month,day,hour,minute,second);
  sec100:=tz.tv_usec div 10000;
end;


Procedure UnixDateToDt(SecsPast: LongInt; Var Dt: DateTime);
Begin
  EpochToLocal(SecsPast,dt.Year,dt.Month,dt.Day,dt.Hour,dt.Min,dt.Sec);
End;


Function DTToUnixDate(DT: DateTime): LongInt;
Begin
  DTToUnixDate:=LocalToEpoch(dt.Year,dt.Month,dt.Day,dt.Hour,dt.Min,dt.Sec);
End;


function GetMsCount: int64;
var
   tv : TimeVal;
{  tz : TimeZone;}
begin
  FPGetTimeOfDay (@tv, nil {,tz});
  GetMsCount := int64(tv.tv_Sec) * 1000 + tv.tv_uSec div 1000;
end;


{******************************************************************************
                               --- Exec ---
******************************************************************************}

Procedure Exec (Const Path: PathStr; Const ComLine: ComStr);
var
  pid      : longint; // pid_t?
  cmdline2 : ppchar;
  commandline : RawByteString;
  realpath : ansistring;

// The Error-Checking in the previous Version failed, since halt($7F) gives an WaitPid-status of $7F00
Begin
  LastDosExitCode:=0;
  if Path='' then
    begin
      doserror:=2;
      exit;
    end;
  pid:=fpFork;
  if pid=0 then
   begin
     cmdline2:=nil;
     realpath:=path;
     if Comline<>'' Then
       begin
         CommandLine:=ToSingleByteFileSystemEncodedFileName(ComLine);  // conversion must live till after fpexec!
         cmdline2:=StringtoPPChar(CommandLine,1);
         cmdline2^:=pchar(realPath);
       end
     else
       begin
         getmem(cmdline2,2*sizeof(pchar));
         cmdline2^:=pchar(realPath);
         cmdline2[1]:=nil;
       end;
     {The child does the actual exec, and then exits}
     fpExecv(pchar(realPath),cmdline2);
     {If the execve fails, we return an exitvalue of 127, to let it be known}
     fpExit(127);
   end
  else
   if pid=-1 then         {Fork failed}
    begin
      DosError:=8;
      exit
    end;
  {We're in the parent, let's wait.}
  LastDosExitCode:=WaitProcess(pid); // WaitPid and result-convert
  if (LastDosExitCode>=0) and (LastDosExitCode<>127) then
    DosError:=0
  else
    DosError:=8; // perhaps one time give an better error
End;


{******************************************************************************
                               --- Disk ---
******************************************************************************}

{
  The Diskfree and Disksize functions need a file on the specified drive, since this
  is required for the fpstatfs system call.
  These filenames are set in drivestr[0..26], and have been preset to :
   0 - '.'      (default drive - hence current dir is ok.)
   1 - '/fd0/.'  (floppy drive 1 - should be adapted to local system )
   2 - '/fd1/.'  (floppy drive 2 - should be adapted to local system )
   3 - '/'       (C: equivalent of dos is the root partition)
   4..26          (can be set by you're own applications)
  ! Use AddDisk() to Add new drives !
  They both return -1 when a failure occurs.
}
Const
  FixDriveStr : array[0..3] of pchar=(
    '.',
    '/fd0/.',
    '/fd1/.',
    '/.'
    );
const
  Drives   : byte = 4;
var
  DriveStr : array[4..26] of pchar;

Function AddDisk(const path:string) : byte;
begin
  if not (DriveStr[Drives]=nil) then
   FreeMem(DriveStr[Drives]);
  GetMem(DriveStr[Drives],length(Path)+1);
  StrPCopy(DriveStr[Drives],path);
  AddDisk:=Drives;
  inc(Drives);
  if Drives>26 then
    Drives:=4;
end;



Function DiskFree(Drive: Byte): int64;
var
  fs : tstatfs;
Begin
  if ((Drive<4) and (not (fixdrivestr[Drive]=nil)) and (fpStatFS(fixdrivestr[drive],@fs)<>-1)) or
     ((not (drivestr[Drive]=nil)) and (fpStatFS(drivestr[drive],@fs)<>-1)) then
   Diskfree:=int64(fs.bavail)*int64(fs.bsize)
  else
   Diskfree:=-1;
End;



Function DiskSize(Drive: Byte): int64;
var
  fs : tstatfs;
Begin
  if ((Drive<4) and (not (fixdrivestr[Drive]=nil)) and (fpStatFS(fixdrivestr[drive],@fs)<>-1)) or
     ((not (drivestr[Drive]=nil)) and (fpStatFS(drivestr[drive],@fs)<>-1)) then
   DiskSize:=int64(fs.blocks)*int64(fs.bsize)
  else
   DiskSize:=-1;
End;



Procedure FreeDriveStr;
var
  i: longint;
begin
  for i:=low(drivestr) to high(drivestr) do
    if assigned(drivestr[i]) then
      begin
        freemem(drivestr[i]);
        drivestr[i]:=nil;
      end;
end;

{******************************************************************************
                       --- Findfirst FindNext ---
******************************************************************************}


Function FNMatch(const Pattern,Name:string):Boolean;
Var
  LenPat,LenName : longint;

  Function DoFNMatch(i,j:longint):Boolean;
  Var
    Found : boolean;
  Begin
  Found:=true;
  While Found and (i<=LenPat) Do
   Begin
     Case Pattern[i] of
      '?' : Found:=(j<=LenName);
      '*' : Begin
            {find the next character in pattern, different of ? and *}
              while Found do
                begin
                inc(i);
                if i>LenPat then Break;
                case Pattern[i] of
                  '*' : ;
                  '?' : begin
                          if j>LenName then begin DoFNMatch:=false; Exit; end;
                          inc(j);
                        end;
                else
                  Found:=false;
                end;
               end;
              Assert((i>LenPat) or ( (Pattern[i]<>'*') and (Pattern[i]<>'?') ));
            {Now, find in name the character which i points to, if the * or ?
             wasn't the last character in the pattern, else, use up all the
             chars in name}
              Found:=false;
              if (i<=LenPat) then
              begin
                repeat
                  {find a letter (not only first !) which maches pattern[i]}
                  while (j<=LenName) and (name[j]<>pattern[i]) do
                    inc (j);
                  if (j<LenName) then
                  begin
                    if DoFnMatch(i+1,j+1) then
                    begin
                      i:=LenPat;
                      j:=LenName;{we can stop}
                      Found:=true;
                      Break;
                    end else
                      inc(j);{We didn't find one, need to look further}
                  end else
                  if j=LenName then
                  begin
                    Found:=true;
                    Break;
                  end;
                  { This 'until' condition must be j>LenName, not j>=LenName.
                    That's because when we 'need to look further' and
                    j = LenName then loop must not terminate. }
                until (j>LenName);
              end else
              begin
                j:=LenName;{we can stop}
                Found:=true;
              end;
            end;
     else {not a wildcard character in pattern}
       Found:=(j<=LenName) and (pattern[i]=name[j]);
     end;
     inc(i);
     inc(j);
   end;
  DoFnMatch:=Found and (j>LenName);
  end;

Begin {start FNMatch}
  LenPat:=Length(Pattern);
  LenName:=Length(Name);
  FNMatch:=DoFNMatch(1,1);
End;


Const
  RtlFindSize = 15;
Type
  RtlFindRecType = Record
    DirPtr   : Pointer;
    SearchNum,
    LastUsed : LongInt;
  End;
Var
  RtlFindRecs   : Array[1..RtlFindSize] of RtlFindRecType;
  CurrSearchNum : LongInt;


Procedure FindClose(Var f: SearchRec);
{
  Closes dirptr if it is open
}
Var
  i : longint;
Begin
  if f.SearchType=0 then
   begin
     i:=1;
     repeat
       if (RtlFindRecs[i].SearchNum=f.SearchNum) then
        break;
       inc(i);
     until (i>RtlFindSize);
     If i<=RtlFindSize Then
      Begin
        RtlFindRecs[i].SearchNum:=0;
        if f.dirptr<>nil then
         fpclosedir(pdir(f.dirptr)^);
      End;
   end;
  f.dirptr:=nil;
End;


Function FindGetFileInfo(const s:string;var f:SearchRec):boolean;
var
  DT   : DateTime;
  Info : RtlInfoType;
  st   : baseunix.stat;
begin
  FindGetFileInfo:=false;
  if not fpstat(s,st)>=0 then
   exit;
  info.FSize:=st.st_Size;
  info.FMTime:=st.st_mtime;
  if (st.st_mode and STAT_IFMT)=STAT_IFDIR then
   info.fmode:=$10
  else
   info.fmode:=$0;
  if (st.st_mode and STAT_IWUSR)=0 then
   info.fmode:=info.fmode or 1;
  if s[f.NamePos+1]='.' then
   info.fmode:=info.fmode or $2;

  If ((Info.FMode and Not(f.searchattr))=0) Then
   Begin
     f.Name:=Copy(s,f.NamePos+1,255);
     f.Attr:=Info.FMode;
     f.Size:=Info.FSize;
     f.mode:=st.st_mode;
     UnixDateToDT(Info.FMTime, DT);
     PackTime(DT,f.Time);
     FindGetFileInfo:=true;
   End;
end;


Function  FindLastUsed: Longint;
{
  Find unused or least recently used dirpointer slot in findrecs array
}
Var
  BestMatch,i : Longint;
  Found       : Boolean;
Begin
  BestMatch:=1;
  i:=1;
  Found:=False;
  While (i <= RtlFindSize) And (Not Found) Do
   Begin
     If (RtlFindRecs[i].SearchNum = 0) Then
      Begin
        BestMatch := i;
        Found := True;
      End
     Else
      Begin
        If RtlFindRecs[i].LastUsed > RtlFindRecs[BestMatch].LastUsed Then
         BestMatch := i;
      End;
     Inc(i);
   End;
  FindLastUsed := BestMatch;
End;



Procedure FindNext(Var f: SearchRec);
{
  re-opens dir if not already in array and calls FindWorkProc
}
Var

  DirName  : Array[0..256] of Char;
  i,
  ArrayPos : Longint;
  FName,
  SName    : string;
  Found,
  Finished : boolean;
  p        : pdirent;
Begin
  If f.SearchType=0 Then
   Begin
     ArrayPos:=0;
     For i:=1 to RtlFindSize Do
      Begin
        If RtlFindRecs[i].SearchNum = f.SearchNum Then
         ArrayPos:=i;
        Inc(RtlFindRecs[i].LastUsed);
      End;
     If ArrayPos=0 Then
      Begin
        If f.NamePos = 0 Then
         Begin
           DirName[0] := '.';
           DirName[1] := '/';
           DirName[2] := #0;
         End
        Else
         Begin
           Move(f.SearchSpec[1], DirName[0], f.NamePos);
           DirName[f.NamePos] := #0;
         End;
        f.DirPtr := fpopendir(@DirName[0]);
        If f.DirPtr <> nil Then
         begin
           ArrayPos:=FindLastUsed;
           If RtlFindRecs[ArrayPos].SearchNum > 0 Then
            FpCloseDir((pdir(rtlfindrecs[arraypos].dirptr)^));
           RtlFindRecs[ArrayPos].SearchNum := f.SearchNum;
           RtlFindRecs[ArrayPos].DirPtr := f.DirPtr;
           if f.searchpos>0 then
            seekdir(pdir(f.dirptr), f.searchpos);
         end;
      End;
     if ArrayPos>0 then
       RtlFindRecs[ArrayPos].LastUsed:=0;
   end;
{Main loop}
  SName:=Copy(f.SearchSpec,f.NamePos+1,255);
  Found:=False;
  Finished:=(f.dirptr=nil);
  While Not Finished Do
   Begin
     p:=fpreaddir(pdir(f.dirptr)^);
     if p=nil then
      FName:=''
     else
      FName:=Strpas(@p^.d_name[0]);
     If FName='' Then
      Finished:=True
     Else
      Begin
        If FNMatch(SName,FName) Then
         Begin
           Found:=FindGetFileInfo(Copy(f.SearchSpec,1,f.NamePos)+FName,f);
           if Found then
            Finished:=true;
         End;
      End;
   End;
{Shutdown}
  If Found Then
   Begin
     f.searchpos:=telldir(pdir(f.dirptr));
     DosError:=0;
   End
  Else
   Begin
     FindClose(f);
     DosError:=18;
   End;
End;


Procedure FindFirst(Const Path: PathStr; Attr: Word; Var f: SearchRec);
{
  opens dir and calls FindWorkProc
}
Begin
  fillchar(f,sizeof(f),0);
  if Path='' then
   begin
     DosError:=3;
     exit;
   end;
{Create Info}
  f.SearchSpec := Path;
  {We always also search for readonly and archive, regardless of Attr:}
  f.SearchAttr := Attr or archive or readonly;
  f.SearchPos  := 0;
  f.NamePos := Length(f.SearchSpec);
  while (f.NamePos>0) and (f.SearchSpec[f.NamePos]<>'/') do
   dec(f.NamePos);
{Wildcards?}
  if (Pos('?',Path)=0)  and (Pos('*',Path)=0) then
   begin
     if FindGetFileInfo(Path,f) then
      DosError:=0
     else
      begin
        { According to tdos2 test it should return 18
        if ErrNo=Sys_ENOENT then
         DosError:=3
        else }
         DosError:=18;
      end;
     f.DirPtr:=nil;
     f.SearchType:=1;
     f.searchnum:=-1;
   end
  else
{Find Entry}
   begin
     Inc(CurrSearchNum);
     f.SearchNum:=CurrSearchNum;
     f.SearchType:=0;
     FindNext(f);
   end;
End;


{******************************************************************************
                               --- File ---
******************************************************************************}

Function FSearch(path : pathstr;dirlist : string) : pathstr;
Var
  info : BaseUnix.stat;
Begin
  if (length(Path)>0) and (path[1]='/') and (fpStat(path,info)>=0) and (not fpS_ISDIR(Info.st_Mode)) then
    FSearch:=path
  else
    FSearch:=Unix.FSearch(path,dirlist);
End;

Procedure GetFAttr(var f; var attr : word);
Var
  info    : baseunix.stat;
  LinAttr : longint;
  p       : pchar;
{$ifndef FPC_ANSI_TEXTFILEREC}
  r       : RawByteString;
{$endif not FPC_ANSI_TEXTFILEREC}
Begin
  DosError:=0;
{$ifdef FPC_ANSI_TEXTFILEREC}
  { encoding is already correct }
  p:=@textrec(f).name;
{$else}
  r:=ToSingleByteFileSystemEncodedFileName(textrec(f).name);
  p:=pchar(r);
{$endif}
  { use the pchar rather than the rawbytestring version so that we don't check
    a second time whether the string needs to be converted to the right code
    page
  }
  if FPStat(p,info)<0 then
   begin
     Attr:=0;
     DosError:=3;
     exit;
   end
  else
   LinAttr:=Info.st_Mode;
  if fpS_ISDIR(LinAttr) then
   Attr:=$10
  else
   Attr:=$0;
  if fpAccess(p,W_OK)<0 then
   Attr:=Attr or $1;
  if filerec(f).name[0]='.' then
   Attr:=Attr or $2;
end;

Procedure getftime (var f; var time : longint);
Var
  Info: baseunix.stat;
  DT: DateTime;
Begin
  doserror:=0;
  if fpfstat(filerec(f).handle,info)<0 then
   begin
     Time:=0;
     doserror:=6;
     exit
   end
  else
   UnixDateToDT(Info.st_mTime,DT);
  PackTime(DT,Time);
End;

Procedure setftime(var f; time : longint);

Var
  utim: utimbuf;
  DT: DateTime;
  p : pchar;
{$ifndef FPC_ANSI_TEXTFILEREC}
  r : Rawbytestring;
{$endif not FPC_ANSI_TEXTFILEREC}
Begin
  doserror:=0;
  with utim do
    begin
      actime:=fptime;
      UnPackTime(Time,DT);
      modtime:=DTToUnixDate(DT);
    end;
{$ifdef FPC_ANSI_TEXTFILEREC}
  { encoding is already correct }
  p:=@textrec(f).name;
{$else}
  r:=ToSingleByteFileSystemEncodedFileName(textrec(f).name);
  p:=pchar(r);
{$endif}
  { use the pchar rather than the rawbytestring version so that we don't check
    a second time whether the string needs to be converted to the right code
    page
  }
  if fputime(p,@utim)<0 then
    begin
      Time:=0;
      doserror:=3;
    end;
End;

{******************************************************************************
                             --- Environment ---
******************************************************************************}

Function EnvCount: Longint;
var
  envcnt : longint;
  p      : ppchar;
Begin
  envcnt:=0;
  p:=envp;      {defined in syslinux}
  while (p^<>nil) do
   begin
     inc(envcnt);
     inc(p);
   end;
  EnvCount := envcnt
End;


Function EnvStr (Index: longint): String;
Var
  i : longint;
  p : ppchar;
Begin
  if Index <= 0 then
    envstr:=''
  else
    begin
      p:=envp;      {defined in syslinux}
      i:=1;
      while (i<Index) and (p^<>nil) do
        begin
          inc(i);
          inc(p);
        end;
      if p=nil then
        envstr:=''
      else
        envstr:=strpas(p^)
    end;
end;


Function GetEnv(EnvVar: String): String;
var
  p     : pchar;
Begin
  p:=BaseUnix.fpGetEnv(EnvVar);
  if p=nil then
   GetEnv:=''
  else
   GetEnv:=StrPas(p);
End;


Procedure setfattr (var f;attr : word);
Begin
  {! No Unix equivalent !}
  { Fail for setting VolumeId }
  if (attr and VolumeID)<>0 then
   doserror:=5;
End;



{******************************************************************************
                            --- Initialization ---
******************************************************************************}

Finalization
  FreeDriveStr;
End.
