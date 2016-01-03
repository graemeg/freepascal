{ %skiptarget=wince }

{
    This file is part of the Free Pascal test suite.
    Copyright (c) 1999-2004 by the Free Pascal development team.

    Test for possible bugs in Dos.FExpand

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program TFExpand;

{$DEFINE DEBUG}
(* Defining DEBUG causes all the source and target strings *)
(* to be written to the console to make debugging easier.  *)
{ $DEFINE DIRECT}
(* Defining DIRECT causes direct embedding of fexpand.inc instead     *)
(* of using FExpand implementation in (previously compiled) unit Dos. *)

uses
{$ifdef FPC}
 PopupErr,
{$endif FPC}
 Dos;

{$IFDEF DIRECT}
(* For testing purposes on non-native platforms *)
 {$DEFINE VOLUMES}
 {$DEFINE NODOTS}
 { $DEFINE AMIGA}
 { $DEFINE UNIX}
 {$DEFINE MACOS}

 { $DEFINE FPC_FEXPAND_DRIVES}
 { $DEFINE FPC_FEXPAND_UNC}
 {$DEFINE FPC_FEXPAND_VOLUMES}
 {$DEFINE FPC_FEXPAND_NO_DEFAULT_PATHS}
 {$DEFINE FPC_FEXPAND_DRIVESEP_IS_ROOT}
 { $DEFINE FPC_FEXPAND_DIRSEP_IS_UPDIR}
 {$DEFINE FPC_FEXPAND_NO_DOTS_UPDIR}
 { $DEFINE FPC_FEXPAND_NO_CURDIR}
 { $DEFINE FPC_FEXPAND_TILDE}
 { $DEFINE FPC_FEXPAND_MULTIPLE_UPDIR}
 {$DEFINE FPC_FEXPAND_DIRSEP_IS_CURDIR}
 { $DEFINE FPC_FEXPAND_GETENV_PCHAR}
{$ENDIF DIRECT}

{$IFDEF LINUX}
 {$IFNDEF UNIX}
  {$DEFINE UNIX}
 {$ENDIF UNIX}
{$ENDIF LINUX}

{$IFDEF AMIGA}
 {$IFNDEF HASAMIGA}
  {$DEFINE HASAMIGA}
 {$ENDIF HASAMIGA}
{$ENDIF AMIGA}

{$IFDEF HASAMIGA}
 {$DEFINE VOLUMES}
 {$DEFINE NODRIVEC}
 {$DEFINE NODOTS}
{$ENDIF HASAMIGA}

{$IFDEF NETWARE}
 {$DEFINE VOLUMES}
 {$DEFINE NODRIVEC}
{$ENDIF NETWARE}

{$IFDEF UNIX}
 {$DEFINE NODRIVEC}
{$ENDIF UNIX}

{$IFDEF MACOS}
 {$DEFINE VOLUMES}
 {$DEFINE NODRIVEC}
 {$DEFINE NODOTS}
{$ENDIF MACOS}

const
{$IFNDEF NODRIVEC}
 CC = 'C:';
{$ENDIF NODRIVEC}
{$IFNDEF FPC}
 FileNameCasePreserving = false;
 DirectorySeparator = '\';
 DirectorySeparator2 = '\';
 DirSep = '\';
 CDrive = 'C:';
 DriveSep = ':';
{$ELSE FPC}
(* Used for ChDir/MkDir *)
 DirectorySeparator2 = System.DirectorySeparator;
 {$IFDEF DIRECT}
  {$IFDEF MACOS}
 DirectorySeparator = ':';
 LFNSupport = true;
 FileNameCasePreserving = true;
  {$ELSE MACOS}
   {$IFDEF UNIX}
 DirectorySeparator = '/';
 DriveSeparator = '/';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 FileNameCasePreserving = true;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 FileNameCaseSensitive = true;
>>>>>>> graemeg/cpstrnew
   {$ELSE UNIX}
    {$IFDEF HASAMIGA}
 DirectorySeparator = '/';
 FileNameCasePreserving = true;
    {$ELSE HASAMIGA}
 DirectorySeparator = '\';
 FileNameCasePreserving = false;
    {$ENDIF HASAMIGA}
   {$ENDIF UNIX}
  {$ENDIF MACOS}
 {$ENDIF DIRECT}
 DirSep = DirectorySeparator;
 {$IFDEF MACOS}
 DriveSep = '';
 {$ELSE MACOS}
 DriveSep = DriveSeparator;
 {$ENDIF MACOS}
 {$IFDEF UNIX}
 CDrive = '';
 {$ELSE UNIX}
  {$IFDEF MACOS}
 CDrive = 'C';
  {$ELSE MACOS}
   {$IFDEF HASAMIGA}
 CDrive = 'C';
   {$ELSE HASAMIGA}
 CDrive = 'C:';
   {$ENDIF HASAMIGA}
  {$ENDIF MACOS}
 {$ENDIF UNIX}
{$ENDIF FPC}
 TestFileName = 'testfile.tst';
 TestDir1Name = 'TESTDIR1';
 TestDir2Name = 'TESTDIR2';
 HasErrors: boolean = false;

{$IFDEF DIRECT}
procedure XToDirect (var S: string);
var
 I: byte;
begin
 if DirectorySeparator2 <> DirectorySeparator then
  for I := 1 to Length (S) do
   if S [I] = DirectorySeparator2 then
    S [I] := DirectorySeparator;
{$IFNDEF FPC_FEXPAND_DRIVES}
 if DriveSeparator = DirectorySeparator then
  I := Pos (DirectorySeparator + DirectorySeparator, S)
 else
  I := Pos (DriveSeparator, S);
 if I <> 0 then
  Delete (S, 1, I);
{$ENDIF FPC_FEXPAND_DRIVES}
end;

procedure GetDir (Drive: byte; var Directory: string);
begin
 System.GetDir (Drive, Directory);
 XToDirect (Directory);
end;

 {$I fexpand.inc}
{$ENDIF DIRECT}

var
{$IFNDEF NODRIVEC}
 CDir,
{$endif}
 TestDir, TestDir0, OrigDir, OrigTstDir, CurDir, S: DirStr;
 TestDrive: string [2];
 I: byte;
 IOR: longint;
 F: file;

function Translate (S: PathStr): PathStr;
var
 I: byte;
begin
{$IFDEF UNIX}
 if (Length (S) > 1) and (S [2] = ':') then Delete (S, 1, 2);
{$ELSE UNIX}
 for I := 1 to Length (S) do if S [I] = '/' then S [I] := DirSep;
 if (Length (S) > 1) and (S [1] in ['a'..'z']) and (S[2]=DriveSep) then
   S [1] := UpCase (S [1]);
{$ENDIF UNIX}
 if not (FileNameCasePreserving) then
                           for I := 1 to Length (S) do S [I] := UpCase (S [I]);
 Translate := S;
end;

procedure Check (Src, Rslt: PathStr);
var
 Rslt2: PathStr;
begin
{$IFDEF DEBUG}
 WriteLn (Src, '=>', Rslt);
{$ENDIF DEBUG}
 Rslt := Translate (Rslt);
 Rslt2 := FExpand (Src);
{$IFDEF DIRECT}
 {$IFNDEF FPC_FEXPAND_DRIVES}
 I := Pos (System.DriveSeparator, Rslt2);
 if I <> 0 then
  Delete (Rslt2, 1, I);
 {$ENDIF FPC_FEXPAND_DRIVES}
{$ENDIF DIRECT}
{$IFNDEF UNIX}
 if (Length (Rslt2) > 1) and (Rslt2 [1] in ['a'..'z']) and (Rslt2[2]=DriveSep) then
   Rslt2 [1] := UpCase (Rslt2 [1]);
{$ENDIF NDEF UNIX}
 if Rslt <> Rslt2 then
 begin
  WriteLn ('Error: FExpand (', Src, ') should be "', Rslt, '", not "',
                                                                   Rslt2, '"');
  HasErrors := true;
 end;
end;

begin
 if ParamCount <> 1 then
 begin
  WriteLn ('Warning: Parameter missing!');
  WriteLn ('Full path to a directory with write access' +
{$IFNDEF UNIX}
 {$IFNDEF VOLUMES}
                               #13#10'(preferably not on a C: drive)' +
 {$ENDIF VOLUMES}
{$ENDIF UNIX}
                                                                 ' expected.');
  WriteLn ('Trying to use the current directory instead ' +
{$IFDEF UNIX}
                                                         '(not quite ideal).');
{$ELSE UNIX}
                                                    '(problems might arise).');
{$ENDIF UNIX}
{$IFDEF DIRECT}System.{$ENDIF DIRECT}GetDir (0, TestDir);
 end else TestDir := ParamStr (1);
 if TestDir [Length (TestDir)] <> DirectorySeparator2 then
  TestDir := TestDir + DirectorySeparator2;
{$IFDEF DIRECT}System.{$ENDIF DIRECT}GetDir (0, OrigDir);
{$IFDEF NODRIVEC}
 TestDrive := '';
{$ELSE NODRIVEC}
 TestDrive := Copy (TestDir, 1, 2);
 GetDir ((Ord (TestDir [1]) and not ($20)) - 64, OrigTstDir);
{$ENDIF NODRIVEC}
{$I-}
 MkDir (TestDir + TestDir1Name);
 if IOResult <> 0 then ;
 MkDir (TestDir + TestDir1Name + DirectorySeparator2 + TestDir2Name);
 if IOResult <> 0 then ;
{$I+}
 ChDir (TestDir + TestDir1Name + DirectorySeparator2 + TestDir2Name);
{$I-}
 TestDir0 := TestDir;
{$IFDEF DIRECT}
 XToDirect (TestDir);
 {$IFNDEF FPC_FEXPAND_DRIVES}
 I := Pos (System.DriveSeparator, TestDir);
 if I <> 0 then
  Delete (TestDir, 1, I);
 {$ENDIF FPC_FEXPAND_DRIVES}
{$ENDIF DIRECT}
 Assign (F, TestFileName);
 Rewrite (F);
 Close (F);
 if IOResult <> 0 then ;
{$IFNDEF DIRECT}
 Assign (F, FExpand (TestFileName));
{$ENDIF DIRECT}
{$I+}
 GetDir (0, CurDir);
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF DIRECT}
 {$IFNDEF FPC_FEXPAND_DRIVES}
 I := Pos (System.DriveSeparator, CurDir);
 if I <> 0 then
  Delete (CurDir, 1, I);
 {$ENDIF FPC_FEXPAND_DRIVES}
{$ENDIF DIRECT}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$IFNDEF NODRIVEC}
 GetDir (3, CDir);
{$ENDIF NODRIVEC}
 Check (' ', CurDir + DirSep + ' ');
{$IFDEF HASAMIGA}
 Check ('', CurDir);
{$ELSE HASAMIGA}
 Check ('', CurDir + DirSep);
{$ENDIF HASAMIGA}
{$IFDEF MACOS}
 Check (':', CurDir + DirSep);
{$ELSE MACOS}
 Check ('.', CurDir);
{$ENDIF MACOS}

{$IFNDEF NODRIVEC}
if CDir [Length (CDir)] = DirSep then Check ('c:anything', CDir + 'anything')
                         else Check ('c:anything', CDir + DirSep + 'anything');
 Check (CC + DirSep, CDrive + DirSep);
<<<<<<< HEAD
 {$IFDEF NODOTS}
 Check ('C:.', 'C:.');
 Check (CC + DirSep + '.', CDrive + DirSep + '.');
 Check (CC + DirSep + '..', CDrive + DirSep + '..');
 {$ELSE NODOTS}
=======
{$IFDEF NODOTS}
 Check ('C:.', 'C:.');
 Check (CC + DirSep + '.', CDrive + DirSep + '.');
 Check (CC + DirSep + '..', CDrive + DirSep + '..');
{$ELSE NODOTS}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 Check ('C:.', CDir);
 Check (CC + DirSep + '.', CDrive + DirSep);
 Check (CC + DirSep + '..', CDrive + DirSep);
 {$ENDIF NODOTS}
 Check (CC + DirSep + 'DOS', CDrive + DirSep + 'DOS');
 {$IFNDEF NODOTS}
 Check (CC + DirSep + '..' + DirSep + 'DOS', CDrive + DirSep + 'DOS');
 {$ENDIF NODOTS}
 Check (CC + DirSep + 'DOS.', CDrive + DirSep + 'DOS.');
 {$IFDEF HASAMIGA} (* This has no effect - AMIGA has NODRIVEC defined... *)
 Check (CC + DirSep + 'DOS' + DirSep, CDrive + DirSep);
 {$ELSE HASAMIGA}
 Check (CC + DirSep + 'DOS' + DirSep, CDrive + DirSep + 'DOS' + DirSep);
 {$ENDIF HASAMIGA}
 {$IFNDEF NODOTS}
 Check (CC + DirSep + 'DOS' + DirSep + '.', CDrive + DirSep + 'DOS');
 Check (CC + DirSep + 'DOS' + DirSep + '..', CDrive + DirSep);
 Check (CC + DirSep + 'DOS' + DirSep + '..' + DirSep, CDrive + DirSep);
 Check (CC + DirSep + 'DOS' + DirSep + 'TEST' + DirSep + '..', CDrive +
                                                               DirSep + 'DOS');
 Check ('C:' + DirSep + 'DOS' + DirSep + 'TEST' + DirSep + '..' + DirSep,
                                             CDrive + DirSep + 'DOS' + DirSep);
<<<<<<< HEAD
 {$ENDIF NODOTS}
=======
{$ENDIF NODOTS}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF NODRIVEC}

{$IFNDEF MACOS}
 {$IFDEF HASAMIGA}
 Check (DirSep, TestDir + TestDir1Name);
 Check (DirSep + DirSep + TestFileName, TestDir + TestFileName);
 Check (DirSep + 'DOS', TestDir + TestDir1Name + DirSep + 'DOS');
 {$ELSE HASAMIGA}
 Check (DirSep, TestDrive + DirSep);
 Check (DirSep + '.', TestDrive + DirSep);
 Check (DirSep + '..', TestDrive + DirSep);
 Check (DirSep + 'DOS', TestDrive + DirSep + 'DOS');
 {$ENDIF HASAMIGA}
{$ENDIF MACOS}
 Check ('d', CurDir + DirSep + 'd');
{$IFDEF MACOS}
 Check (DirSep + 'd', CurDir + DirSep + 'd');
{$ELSE MACOS}
 {$IFNDEF NODOTS}
 Check ('.' + DirSep + 'd', CurDir + DirSep + 'd');
 {$ENDIF NODOTS}
{$ENDIF MACOS}
 Check ('d' + DirSep + TestFileName, CurDir + DirSep + 'd' + DirSep + TestFileName);
 Check (' d', CurDir + DirSep + ' d');
 Check ('dd', CurDir + DirSep + 'dd');
{$IFDEF MACOS}
 Check (DirSep + 'dd' + DirSep + 'dd', CurDir + DirSep + 'dd' + DirSep + 'dd');
 Check ('dd' + DirSep + 'dd', 'dd' + DirSep + 'dd');
{$ELSE MACOS}
 Check ('dd' + DirSep + 'dd', CurDir + DirSep + 'dd' + DirSep + 'dd');
{$ENDIF MACOS}
 Check ('ddd', CurDir + DirSep + 'ddd');
{$IFDEF MACOS}
 Check ('dddd' + DirSep + 'eeee.ffff', 'dddd' + DirSep + 'eeee.ffff');
{$ELSE MACOS}
 Check ('dddd' + DirSep + 'eeee.ffff', CurDir + DirSep + 'dddd' + DirSep
                                                                + 'eeee.ffff');
{$ENDIF MACOS}
 Check ('.special', CurDir + DirSep + '.special');
 Check ('..special', CurDir + DirSep + '..special');
 Check ('special..', CurDir + DirSep + 'special..');
{$IFDEF HASAMIGA}
 Check ('special.' + DirSep, CurDir + DirSep + 'special.' + DirSep);
{$ELSE HASAMIGA}
 {$IFDEF MACOS}
 Check ('special.' + DirSep, 'special.' + DirSep);
 {$ELSE MACOS}
 Check ('special.' + DirSep, CurDir + DirSep + 'special.' + DirSep);
 {$ENDIF MACOS}
{$ENDIF HASAMIGA}
{$IFDEF MACOS}
 Check (DirSep + DirSep, TestDir + TestDir1Name + DirSep);
 Check (DirSep + DirSep + TestFileName, TestDir + TestDir1Name + DirSep
                                                               + TestFileName);
{$ELSE MACOS}
 Check (DirSep + '.special', TestDrive + DirSep + '.special');
 {$IFNDEF NODOTS}
 Check ('..', TestDir + TestDir1Name);
 Check ('.' + DirSep + '..', TestDir + TestDir1Name);
 Check ('..' + DirSep + '.', TestDir + TestDir1Name);
 {$ENDIF NODOTS}
{$ENDIF MACOS}
{$IFDEF NETWARE}
 Check ('...', TestDir);
{$ELSE NETWARE}
 Check ('...', CurDir + DirSep + '...');
{$ENDIF NETWARE}
 Check (TestFileName, CurDir + DirSep + TestFileName);
{$IFDEF UNIX}
 S := GetEnv ('HOME');
 { On m68k netbsd at least, HOME contains a final slash
   remove it PM }
 if (Length (S) > 1) and (S [Length (S)] = DirSep) then
   S:=Copy(S,1,Length(S)-1);
 if Length (S) = 0 then
  begin
   Check ('~', CurDir);
   Check ('~' + DirSep + '.', DirSep);
  end
 else
  begin
   Check ('~', S);
   Check ('~' + DirSep + '.', S);
  end;
 if (Length (S) > 0) and (S [Length (S)] <> DirSep) then
  S := S + DirSep;
 Check ('~NobodyWithThisNameShouldEverExist.test/nothing', CurDir + DirSep +
                            '~NobodyWithThisNameShouldEverExist.test/nothing');
 Check ('/tmp/~NoSuchUserAgain', '/tmp/~NoSuchUserAgain');
 if Length (S) = 0 then
  begin
   Check ('~' + DirSep, DirSep);
   Check ('~' + DirSep + '.' + DirSep, DirSep);
   Check ('~' + DirSep + 'directory' + DirSep + 'another',
                                    DirSep + 'directory' + DirSep + 'another');
  end
 else
  begin
   Check ('~' + DirSep, S);
   Check ('~' + DirSep + '.' + DirSep, S);
   Check ('~' + DirSep + 'directory' + DirSep + 'another',
                                         S + 'directory' + DirSep + 'another');
  end;
{$ELSE UNIX}
 {$IFNDEF NODRIVEC}
 Check (TestDrive + '..', TestDir + TestDir1Name);
 Check (TestDrive + '..' + DirSep, TestDir + TestDir1Name + DirSep);
 Check (TestDrive + '.' + DirSep + '.', CurDir);
 Check (TestDrive + '.' + DirSep + '..', TestDir + TestDir1Name);
{$I-}
(*
{ $ ifndef unix }
{   avoid a and b drives for
   no unix systems to reduce the
   probablility of getting an alert message box }
 { This should not be needed - unit popuperr should solve this?! TH }
 I := 3;
{$else unix} *)
 I := 1;
{ $ endif unix}
 repeat
  S := '';
  GetDir (I, S);
  IOR := IOResult;
  if IOR = 0 then Inc (I);
 until (I > 26) or (IOR <> 0);
 if I <= 26 then
 begin
  S := Chr (I + 64) + ':ddd';
  Check (S, Chr (I + 64) + ':' + DirSep + 'ddd');
 end else
   WriteLn ('Sorry, cannot test FExpand behaviour for incorrect drives here.');
{$I+}
  {$IFDEF FPC}
 Check ('d\d/d', CurDir + DirSep + 'd' + DirSep + 'd' + DirSep + 'd');
 Check ('\\server\share\directory', '\\server\share\directory');
 Check ('\\server\share\directory1\directory2\..',
                                                  '\\server\share\directory1');
 Check ('\\', '\\');
 Check ('\\.', '\\.\');
 Check ('\\.\', '\\.\');
 Check ('\\.\.', '\\.\.');
 Check ('\\.\..', '\\.\..');
 Check ('\\.\...', '\\.\...');
 Check ('\\.\TEST', '\\.\TEST');
 Check ('\\..\', '\\..\');
 Check ('\\..\TEST', '\\..\TEST');
 Check ('\\..\TEST\.', '\\..\TEST');
 Check ('\\..\TEST1\TEST2\..', '\\..\TEST1');
 Check ('\\..\TEST\..', '\\..\TEST');
 Check ('\\..\TEST\..\..', '\\..\TEST');
  {$ENDIF FPC}
 {$ENDIF NODRIVEC}
{$ENDIF UNIX}
{$IFDEF VOLUMES}
 {$IFDEF HASAMIGA}
 Check ('VolName' + DriveSep + 'DIR1', 'VolName' + DriveSep + 'DIR1');
 {$ELSE HASAMIGA}
 Check ('VolName' + DriveSep + DirSep + 'DIR1', 'VolName' + DriveSep + DirSep + 'DIR1');
 {$ENDIF HASAMIGA}
 {$IFNDEF NODOTS}
 Check ('VolName' + DriveSep + DirSep + 'DIR1' + DirSep + '..', 'VolName' + DriveSep + DirSep);
 Check ('VolName' + DriveSep + DirSep + 'DIR1' + DirSep + '..' + DirSep + '..',
                                                          'VolName' + DriveSep + DirSep);
 Check ('VolName' + DriveSep + DirSep + '.', 'VolName:' + DirSep);
 Check ('VolName' + DriveSep + DirSep + '..', 'VolName:' + DirSep);
 Check ('VolName' + DriveSep + DirSep + '..' + DirSep, 'VolName' + DriveSep + DirSep);
 {$ENDIF NODOTS}
 {$IFDEF NETWARE}
 Check ('SrvName\VolName' + DriveSep + DirSep + 'TEST', 'SrvName' + DirSep + 'VolName' +
                                                         DriveSep + DirSep + 'TEST');
 Check ('SrvName/VolName' + DriveSep + DirSep + 'TEST', 'SrvName' + DirSep + 'VolName' +
                                                         DriveSep + DirSep + 'TEST');
 {$ENDIF NETWARE}
 {$IFDEF HASAMIGA}
  {$IFDEF NODOTS}
 Check ('.', CurDir + DirSep + '.');
  {$ELSE NODOTS}
 Check ('.', CurDir);
  {$ENDIF NODOTS}
 {$ENDIF HASAMIGA}
{$ENDIF VOLUMES}
 Erase (F);
{$IFNDEF NODRIVEC}
 ChDir (OrigTstDir);
{$ENDIF NODRIVEC}
 ChDir (OrigDir);
 RmDir (TestDir0 + TestDir1Name + DirectorySeparator2 + TestDir2Name);
 RmDir (TestDir0 + TestDir1Name);
 if HasErrors then
 begin
  WriteLn ('FExpand doesn''t work correctly.');
  Halt (1);
 end;
end.
