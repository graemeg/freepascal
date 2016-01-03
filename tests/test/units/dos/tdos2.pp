{******************************************}
{  Used to check the DOS unit              }
{------------------------------------------}
{  Requirements for this unit can be       }
{  found in testdos.htm                    }
{******************************************}
Program TestDos;

Uses Dos;

{**********************************************************************}
{ Some specific OS verifications : }
{ Mainly for file attributes:      }
{ Read-Only                        }
{ Hidden                           }
{ System File                      }
{ only work on Win32, OS/2 and DOS }



{$IFDEF MSDOS}
        {$DEFINE EXTATTR}
{$ENDIF}
{$IFDEF DPMI}
        {$DEFINE EXTATTR}
{$ENDIF}
{$IFDEF GO32V1}
        {$DEFINE EXTATTR}
{$ENDIF}
{$IFDEF GO32V2}
        {$DEFINE EXTATTR}
{$ENDIF}
{$IFDEF OS2}
        {$DEFINE EXTATTR}
{$ENDIF}
{$IFDEF WIN32}
        {$DEFINE EXTATTR}
{$ENDIF}
{$IFDEF ATARI}
        {$DEFINE EXTATTR}
{$ENDIF}
{$IFDEF WINCE}
        {$DEFINE EXTATTR}
{$ENDIF}



{$IFNDEF UNIX}
{$IFDEF LINUX}
        {$DEFINE UNIX}
{$ENDIF}
{$IFDEF QNX}
        {$DEFINE UNIX}
{$ENDIF}
{$IFDEF SOLARIS}
        {$DEFINE UNIX}
{$ENDIF}
{$IFDEF FREEBSD}
        {$DEFINE UNIX}
{$ENDIF}
{$IFDEF BEOS}
        {$DEFINE UNIX}
{$ENDIF}
{$ENDIF}
{**********************************************************************}



CONST
{ what is the root path }
{$ifdef UNIX}
  RootPath = '/';
{$else UNIX}
  {$ifdef WINCE}
    RootPath = '\';
  {$else WINCE}
    RootPath = 'C:\';
  {$endif WINCE}
{$ENDIF}
 Week:Array[0..6] of String =
 ('Sunday','Monday','Tuesday','Wednesday','Thursday','Friday','Saturday');

 TestFName = 'TESTDOS.DAT';  { CASE SENSITIVE DON'T TOUCH! }
 TestFName1 = 'TESTFILE';    { CASE SENSITIVE DON'T TOUCH! }
 TestDir = 'MYDIR';          { CASE SENSITIVE DON'T TOUCH! }
 TestExt   = 'DAT';
 has_errors : boolean = false;


Procedure PauseScreen;
var
 ch: char;
Begin
 { this is the non-interacting version
   so we disable this
 WriteLn('-- Press any key --');
 ReadLn;}
end;

{ verifies that the DOSError variable is equal to }
{ the value requested.                            }
Procedure CheckDosError(err: Integer);
 var
  x : integer;
  s :string;
 Begin
  Write('Verifying value of DOS Error...');
  x := DosError;
  case x of
  0 : s := '(0): No Error.';
  2 : s := '(2): File not found.';
  3 : s := '(3): Path not found.';
  5 : s := '(5): Access Denied.';
  6 : s := '(6): Invalid File Handle.';
  8 : s := '(8): Not enough memory.';
  10 : s := '(10) : Invalid Environment.';
  11 : s := '(11) : Invalid format.';
  18 : s := '(18) : No more files.';
  else
    s := 'INVALID DOSERROR';
  end;
  if err <> x then
    Begin
      WriteLn('FAILURE. (Value should be ',err,' '+s+')');
      has_errors:=true;
    end
  else
    WriteLn('Success.');
 end;








Procedure TestSystemDate;
var
 Year,Month, DayOfWeek, Day: Word;
 Year1,Month1, DayOfWeek1, Day1: Word;
 s: string;
Begin
 WriteLn('----------------------------------------------------------------------');
 WriteLn('                            GETDATE                                   ');
 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: Number of week should be consistent (0 = Sunday)               ');
 WriteLn(' Note: Year should contain full four digits.                          ');
 WriteLn('----------------------------------------------------------------------');
 CheckDosError(0);
 Month:=0;
 Day:=0;
 DayOfWeek:=0;
 Year:=0;
 GetDate(Year,Month,Day,DayOfWeek);
 CheckDosError(0);
 Write('DD-MM-YYYY : ',Day,'-',Month,'-',Year);
 WriteLn(' (',Week[DayOfWeek],')');
 PauseScreen;

 WriteLn('----------------------------------------------------------------------');
 WriteLn('                            SETDATE                                   ');
 WriteLn('----------------------------------------------------------------------');
 { normal call }
 SetDate(Year,Month,Day);
 CheckDosError(0);
 { setdate and settime is not supported on most platforms }
{$ifdef go32v2}
 s:='Testing with invalid year....';
 SetDate(98,Month,Day);
 CheckDosError(0);
 GetDate(Year1,Month1,Day1,DayOfWeek1);
 CheckDosError(0);
 if (Year1 <> Year) or (Month1 <> month) or (Day1 <> Day) then
  Begin
     WriteLn(s+'FAILURE.');
  end
 else
  WriteLn(s+'Success.');

 SetDate(Year,Month,255);
 CheckDosError(0);
 s:='Testing with invalid day.....';
 GetDate(Year1,Month1,Day1,DayOfWeek1);
 CheckDosError(0);
 if (Year1 <> Year) or (Month1 <> month) or (Day1 <> Day) then
  Begin
     WriteLn(s+'FAILURE.');
  end
 else
  WriteLn(s+'Success.');

 SetDate(Year,13,Day);
 CheckDosError(0);
 s:='Testing with invalid month...';
 GetDate(Year1,Month1,Day1,DayOfWeek1);
 CheckDosError(0);
 if (Year1 <> Year) or (Month1 <> month) or (Day1 <> Day) then
  Begin
     WriteLn(s+'FAILURE.');
  end
 else
  WriteLn(s+'Success.');

 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: Date should be 01-01-1998                                      ');
 WriteLn('----------------------------------------------------------------------');
 SetDate(1998,01,01);
 CheckDosError(0);
 GetDate(Year1,Month1,Day1,DayOfWeek1);
 CheckDosError(0);
 WriteLn('DD-MM-YYYY : ',Day1,'-',Month1,'-',Year1);
 SetDate(Year,Month,Day);
 CheckDosError(0);
 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: Date should be restored to previous value                      ');
 WriteLn('----------------------------------------------------------------------');
 GetDate(Year1,Month1,Day1,DayOfWeek1);
 CheckDosError(0);
 WriteLn('DD-MM-YYYY : ',Day1,'-',Month1,'-',Year1);
 PauseScreen;
{$endif}
end;

Procedure TestsystemTime;
Var
 Hour, Minute, Second, Sec100: word;
 Hour1, Minute1, Second1, Sec1001: word;
Begin
 WriteLn('----------------------------------------------------------------------');
 WriteLn('                            GETTIME                                   ');
 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: Hours should be in military format (0..23), and MSec in 0..100 ');
 WriteLn('----------------------------------------------------------------------');
 CheckDosError(0);
 Hour:=0;
 Minute:=0;
 Second:=0;
 Sec100:=0;
 GetTime(Hour,Minute,Second,Sec100);
 CheckDosError(0);
 WriteLn('HH:MIN:SEC (MS): ',Hour,':',Minute,':',Second,' (',Sec100,')');
 WriteLn('----------------------------------------------------------------------');
 WriteLn('                            SETTIME                                   ');
 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: GetTime should return the same value as the previous test.     ');
 WriteLn('----------------------------------------------------------------------');
{$ifndef beos}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef linux}
{$ifndef android}
 {This should be disabled under BeOS : maybe this is a BeOS bug (or a feature ?)
=======
 {This should be disabled under BeOS : maybe this is a BeOS bug (or a feature ?) 
>>>>>>> graemeg/fixes_2_2
=======
 {This should be disabled under BeOS : maybe this is a BeOS bug (or a feature ?) 
>>>>>>> origin/fixes_2_2
  in stime function.
  When you set 36 hours, the time AND the date are changed
  It seems it is a valid value under BeOS, but you have jump in the future :
  36 hours in the future from the begining of the starting day, more or less
  depending on your timezone.
  For example in Paris, in summer (2 hours from GMT time zone),
<<<<<<< HEAD
<<<<<<< HEAD
  this call set the clock to 14:<Minute>:<Second>:<Sec100> the next day !
  Linux shows the same behaviour.
  }
 SetTime(36,Minute,Second,Sec100);
 CheckDosError(0);
{$endif}
{$endif}
{$endif}
=======
  this call set the clock to 14:<Minute>:<Second>:<Sec100> the next day !}
 SetTime(36,Minute,Second,Sec100);
 CheckDosError(0);
{$endif}
>>>>>>> graemeg/fixes_2_2
=======
  this call set the clock to 14:<Minute>:<Second>:<Sec100> the next day !}
 SetTime(36,Minute,Second,Sec100);
 CheckDosError(0);
{$endif}
>>>>>>> origin/fixes_2_2
 GetTime(Hour1,Minute1,Second1,Sec1001);
 CheckDosError(0);
 WriteLn('HH:MIN:SEC ',Hour1,':',Minute1,':',Second1);
 { actual settime is only supported under DOS }
{$ifdef go32v2}
 SetTime(Hour,32000,Second,Sec100);
 CheckDosError(0);
 GetTime(Hour1,Minute1,Second1,Sec1001);
 CheckDosError(0);
 WriteLn('HH:MIN:SEC ',Hour1,':',Minute1,':',Second1);
 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: GetTime should return  0:0:0                                   ');
 WriteLn('----------------------------------------------------------------------');
 SetTime(0,0,0,0);
 CheckDosError(0);
 GetTime(Hour1,Minute1,Second1,Sec1001);
 CheckDosError(0);
 WriteLn('HH:MIN:SEC ',Hour1,':',Minute1,':',Second1);
 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: GetTime should return  approximately the original time         ');
 WriteLn('----------------------------------------------------------------------');
 SetTime(Hour,Minute,Second,Sec1001);
 CheckDosError(0);
 GetTime(Hour1,Minute1,Second1,Sec1001);
 CheckDosError(0);
 WriteLn('HH:MIN:SEC ',Hour1,':',Minute1,':',Second1);
 {$endif}
end;




Procedure TestFTime;
var
 s : string;
 F: File;
 Time: Longint;
 DT: DateTime;
 DT1 : Datetime; { saved values }
Begin
 WriteLn('----------------------------------------------------------------------');
 WriteLn('                         GETFTIME / SETFTIME                          ');
 WriteLn('----------------------------------------------------------------------');
 CheckDosError(0);

 {**********************************************************************}
 {********************** TURBO PASCAL BUG ******************************}
 { The File is not Open and DosError is still zero! THIS SHOULD NOT BE  }
 { SO IN FPC!                                                           }
 {**********************************************************************}
 {********************** TURBO PASCAL BUG ******************************}
 Write('Assigning an invalid file...');
 Assign(f,'x');
 GetFTime(f,Time);
{$ifndef macos}
 CheckDosError(6);
{$else}
 CheckDosError(2); {Since on MacOS, GetFTime works even for non-opened files}
{$endif}

 Write('Trying to open ',TestFName,'...');
 Assign(f,TestFName);
 Reset(f,1);
 GetFTime(f,Time);
 CheckDosError(0);
 UnpackTime(Time,Dt);
 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: Hour should be in military format and year should be a 4 digit ');
 WriteLn('       number.                                                        ');
 WriteLn('----------------------------------------------------------------------');
 WriteLn('DD-MM-YYYY : ',DT.Day,'-',DT.Month,'-',DT.Year);
 WriteLn('HH:MIN:SEC ',DT.Hour,':',DT.Min,':',DT.Sec);

 { SETFTIME / GETFTIME No Range checking is performed so the tests are }
 { very limited.                                                       }
 s:='Setting '+TestFName+' date/time to 01-28-1998:0:0:0...';
 dt1.Year:=1998;
 dt1.Month:=1;
 dt1.Day:=28;
 Dt1.Hour:=0;
 Dt1.Min:=0;
 Dt1.Sec:=0;
 PackTime(DT1,Time);
 CheckDosError(0);
 SetFTime(f,Time);
 CheckDosError(0);
 GetFTime(f,Time);
 CheckDosError(0);
 { Re-initialize the date time file }
 FillChar(Dt1,sizeof(dt1),#0);
 UnpackTime(Time,Dt1);
 if (Dt1.Year <> 1998) or (Dt1.Month<>1) or (Dt1.Day<>28) or
    (Dt1.Hour<>0) or (Dt1.Min <>0) or (Dt1.Sec<>0) then
   Begin
      WriteLn(s+'FAILURE.');
   end
 else
   WriteLn(s+'Success.');

 s:='Restoring old file time stamp...';
 Move(Dt,Dt1,sizeof(Dt));
 PackTime(DT1,Time);
 CheckDosError(0);
 SetFTime(f,Time);
 CheckDosError(0);
 GetFTime(f,Time);
 CheckDosError(0);
 { Re-initialize the date time file }
 FillChar(Dt1,sizeof(dt),#0);
 UnpackTime(Time,Dt1);
 if (Dt1.Year <> Dt.Year) or (Dt1.Month<>Dt.Month) or (Dt1.Day<>Dt.Day) or
    (Dt1.Hour<>Dt.Hour) or (Dt1.Min <> Dt.Min) or (Dt1.Sec<>Dt.Sec) then
   Begin
      WriteLn(s+'FAILURE.');
   end
 else
   WriteLn(s+'Success.');
 Close(f);
end;

Procedure TestFind;
var
 Search: SearchRec;
 DT: Datetime;
 Year, Month, Day, DayOfWeek: Word;
 Failure : Boolean;
 FoundDot, FoundDotDot: boolean;
 FoundDir : boolean;
 s : string;
Begin
 WriteLn('----------------------------------------------------------------------');
 WriteLn('                         FINDFIRST/ FINDNEXT                          ');
 WriteLn('----------------------------------------------------------------------');
 WriteLn(' Note: The full path should NOT be displayed.                         ');
 WriteLn('----------------------------------------------------------------------');
 CheckDosError(0);
 WriteLn('Trying to find an invalid file ('''') with Any Attribute...');
 FindFirst('',AnyFile,Search);
 CheckDosError(3);
{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}

{$ifdef go32v2}
 WriteLn('Trying to find an invalid file ('''') with VolumeID attribute...');
 FindFirst('',VolumeID,Search);
 CheckDosError(3);
{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}
{$endif go32v2}

 WriteLn('Trying to find an invalid file (''''zz.dat'''') with Any Attribute...');
 FindFirst('zz.dat',AnyFile,Search);
 CheckDosError(18);
{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}

 WriteLn('Trying to find an invalid file (''''zz.dat'''') with VolumeID attribute...');
 FindFirst('zz.dat',VolumeID,Search);
 CheckDosError(18);
{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}

 WriteLn('Trying to find an invalid file (''''zz.dat'''') with Directory attribute...');
 FindFirst('zz.dat',Directory,Search);
 CheckDosError(18);
{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}

 s:='Looking for '+TestFName +' with Any Attribute...';
 FindFirst('*.DAT',AnyFile,Search);
 if Search.Name <> TestFName then
  Begin
    repeat
      FindNext(Search);
    until (DosError <> 0) OR (Search.Name = TestFName);
  end;
 if Search.Name <> TestFName then
 { At least testdos.dat should appear }
   WriteLn(s+'FAILURE. ',TestFName,' should be found.')
 else
   WriteLn(s+'Success.');

{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}

 { In addition to normal files          }
 { directory files should also be found }
 s:='Looking for '+TestFName +' with Directory Attribute...';
 FindFirst('*.DAT',Archive+Directory,Search);
 if DosError<> 0 then
   WriteLn(s+'FAILURE. ',TestFName,' should be found.')
 else
   WriteLn(s+'Success.');
 if Search.Name <> TestFName then
  Begin
    repeat
      FindNext(Search);
    until (DosError <> 0) OR (Search.Name = TestFName);
  end;
{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}

 Write('Checking file stats of ',TestFName,'...');
 UnpackTime(Search.Time,DT);
 GetDate(Year, Month, Day, DayOfWeek);
 if (Search.Size <> Sizeof(week)) OR (DT.Year <> Year) OR (DT.Month <> Month)
    OR (DT.Day <> Day)
 then
  Begin
    WriteLn('FAILURE. Size/Date is different.')
  end
 else
   WriteLn('Success.');
 Write('Looking for ',TestFName,'...');
 FindFirst('*.D??',AnyFile,Search);
 { At least testdos.dat should appear }
 if DosError <> 0 then
   WriteLn('FAILURE. ',Testfname,' should be found.')
 else
   WriteLn('Success.');
 if Search.Name <> TestFName then
  Begin
    repeat
      FindNext(Search);
    until (DosError <> 0) OR (Search.Name = TestFName);
  end;
{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}

 Write('Checking file stats of ',TestFName,'...');
 UnpackTime(Search.Time,DT);
 GetDate(Year, Month, Day, DayOfWeek);
 if (Search.Size <> Sizeof(week)) OR (DT.Year <> Year) OR (DT.Month <> Month)
    OR (DT.Day <> Day)
 then
  Begin
    WriteLn('FAILURE. Size/Date is different.')
  end
 else
   WriteLn('Success.');

 { Should show all possible files }
 FoundDot := False;
 FoundDotDot := False;
 Failure := True;
 FoundDir := False;
 s:='Searching using * wildcard (normal files + directories)...';
 FindFirst('*',Archive+Directory,Search);
 WriteLn(#9'Resources found (full path should not be displayed):');
 while DosError = 0 do
 Begin
    If Search.Name = TestDir then
    Begin
      If Search.Attr and Directory <> 0 then
        FoundDir := TRUE;
    end;
    If Search.Name = '.' then
    Begin
      If Search.Attr and Directory <> 0 then
         FoundDot := TRUE;
    End;
    if Search.Name = '..' then
    Begin
      If Search.Attr and Directory <> 0 then
         FoundDotDot := TRUE;
    End;
    { check for both . and .. special files }
    If Search.Name = TestFName1 then
      Failure := FALSE;
    WriteLn(#9+Search.Name);
    FindNext(Search);
 end;
{$IFDEF FPC}
 FindClose(Search);
{$ENDIF}

 if not FoundDir then
   WriteLn(s+'FAILURE. Did not find '+TestDir+' directory')
 else
{$ifndef wince}
 if not FoundDot then
   WriteLn(s+'FAILURE. Did not find special ''''.'''' directory')
 else
 if not FoundDotDot then
   WriteLn(s+'FAILURE. Did not find special ''''..'''' directory')
 else
{$endif wince}
 if Failure then
   WriteLn(s+'FAILURE. Did not find special '+TestFName1+' directory')
 else
   WriteLn(s+'Success.');

{$ifdef go32v2}
 if not LFNSupport then
 begin
   s:='Searching using ??? wildcard (normal files + all special files)...';
   FindFirst('???',AnyFile,Search);
   FoundDot := False;
   FoundDotDot := False;
   WriteLn(#9'Resources found (full path should not be displayed):');
   while DosError = 0 do
   Begin
      If Search.Name = '.' then
      Begin
        If Search.Attr and Directory <> 0 then
           FoundDot := TRUE;
      End;
      if Search.Name = '..' then
      Begin
        If Search.Attr and Directory <> 0 then
           FoundDotDot := TRUE;
      End;
      WriteLn(#9+Search.Name);
      FindNext(Search);
   end;
   if not FoundDot then
     WriteLn(s+'FAILURE. Did not find special ''''.'''' directory')
   else
   if not FoundDotDot then
     WriteLn(s+'FAILURE. Did not find special ''''..'''' directory')
   else
     WriteLn(s+'Success.');
  {$IFDEF FPC}
    FindClose(Search);
  {$ENDIF}
   { search for volume ID }
   s:='Searching using *.* wildcard in ROOT (normal files + volume ID)...';
   FindFirst(RootPath+'*.*',Directory+VolumeID,Search);
   Failure := TRUE;
   WriteLn(#9'Resources found (full path should not be displayed):');
   while DosError = 0 do
   Begin
      If Search.Attr and VolumeID <> 0 then
      Begin
        Failure := FALSE;
        WriteLn(#9'Volume ID: '+Search.Name);
      End
      else
        WriteLn(#9+Search.Name);
      FindNext(Search);
   end;
   If Failure then
     WriteLn(s+'FAILURE. Did not find volume name')
   else
     WriteLn(s+'Success.');
  {$IFDEF FPC}
    FindClose(Search);
  {$ENDIF}
 end;
{$endif}

end;


Procedure TestSplit;
var
 P: PathStr;
 D: DirStr;
 N: NameStr;
 E: ExtStr;
 temp : string;
Begin
 WriteLn('----------------------------------------------------------------------');
 WriteLn('                                FSPLIT                                ');
 WriteLn('----------------------------------------------------------------------');
 Write('Testing invalid filename...');
 { Initialize names ot invalid values! }
 D:='Garbage';
 N:='Garbage';
 E:='GAR';
 { This is the path to be split }
 P:='';
 FSPlit(P,D,N,E);
 IF (length(D) <> 0) OR (length(N) <>0) OR (length(E) <> 0) THEN
   WriteLn('FAILURE. Same length as PATH (now length 0) should be returned.')
 else
   WriteLn('Success.');
 Write('Testing paramstr(0)...');
 { Initialize names ot invalid values! }
 D:='Garbage';
 N:='Garbage';
 E:='GAR';
 { This is the path to be split }
 P:=paramstr(0);
 FSPlit(P,D,N,E);
 IF length(p) <> (length(d)+length(n)+length(e)) then
   WriteLn('FAILURE. Same length as PATH should be returned.')
 else
   WriteLn('Success.');
 temp:=d+n+e;
 Write('Testing paramstr(0)...');
 if temp <> p then
   WriteLn('FAILURE. Concatenated string should be the same.')
 else
   WriteLn('Success.');
 WriteLn('PARAMSTR(0) = ', ParamStr(0));
 WriteLn('DRIVE + NAME + EXT = ',d+n+e);
{$ifdef go32v2}
 if not LFNSupport then
 begin
   Write('Testing invalid path (..)...');
   P:='..';
   FSPlit(P,D,N,E);
   IF (length(D) <> 0) OR (length(N) <>0) OR (E <> P) THEN
     WriteLn('FAILURE. Length of drive and name should be zero and Ext should return Path')
   else
     WriteLn('Success.');
 end;
{$endif}
 Write('Testing invalid path (*)...');
 P:='*';
 FSPlit(P,D,N,E);
 IF (length(D) <> 0) OR (length(e) <>0) OR (N <> P) THEN
   WriteLn('FAILURE. Length of drive and name should be zero and Name should return Path')
 else
   WriteLn('Success.');
end;

{$ifdef go32v2}
procedure TestWithLFN;
begin
 WriteLn('----------------------------------------------------------------------');
 WriteLn('                           Running LFN tests                          ');
 WriteLn('----------------------------------------------------------------------');
 TestFind;
 PauseScreen;
 TestSplit;
 //Force RTL to use non-LFN calls
 FileNameCaseSensitive:=false;
<<<<<<< HEAD
<<<<<<< HEAD
 FileNameCasePreserving:=false;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 AllFilesMask := '*.*';
 LFNSupport:=false;
 WriteLn('----------------------------------------------------------------------');
 WriteLn('                         Running non-LFN tests                        ');
 WriteLn('----------------------------------------------------------------------');
end;
{$endif}

var
 F: File;
 Attr : Word;
Begin
{$IFDEF MACOS}
 pathTranslation:= true;
{$ENDIF}
 TestSystemDate;
 TestSystemTime;

 { Now the file I/O functions                  }
 { Let us create a file that we will play with }
 Assign(f,TestFName);
 Rewrite(f,1);
 BlockWrite(f,Week,sizeof(Week));
 Close(f);
 Assign(f,TestFName1);
 Rewrite(f,1);
 Close(F);
 MkDir(TestDir);
 TestFTime;
 {$ifdef go32v2}
 TestWithLFN;
 {$endif}
 TestFind;
 PauseScreen;
 TestSplit;
 RmDir(TestDir);
 PauseScreen;

 { Cleanup }
 {$I-}
  assign(f,TestFName);
  erase(f);
  assign(f,TestFName1);
  erase(f);
 {$I+}
 if ioresult<>0 then;

 if has_errors then
   halt(1);
end.
