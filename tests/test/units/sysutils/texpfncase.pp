program texpfncase;
{$IFDEF FPC}
 {$MODE DELPHI}
{$ENDIF FPC}
{$H+}

{$APPTYPE CONSOLE}

{$IFDEF FPC}
 {$DEFINE FPCTEST}
{$ENDIF FPC}

{$I+}


uses
  SysUtils
{$IFNDEF FPC}
, StrUtils
{$ENDIF FPC}
          ;


const
  TestFilesNumber = 3;
{$IFDEF UNIX}
  MinPathLength = 1;
{$ELSE UNIX}
  MinPathLength = 3;
{$ENDIF UNIX}
{$ifndef FPC}
  DirectorySeparator = PathDelim;
  AllowDirectorySeparators: set of char = [PathDelim];
{$endif}

type
  TTestFiles = array [1..TestFilesNumber] of shortstring;


const
  TestFiles: TTestFiles = ('testFile1.tst', 'testFile2.tst', 'TestFile2.tst');


{$IFNDEF FPC}
const
  FilenameCaseMatchStr: array [mkNone..mkAmbiguous] of shortstring =
                    ('mkNone', 'mkExactMatch', 'mkSingleMatch', 'mkAmbiguous');
{$ENDIF FPC}

var
 Failed: byte;


procedure TestExpFNC (const FN1, ExpReturn: string; ExpMatch: TFilenameCaseMatch);
var
  FN2: string;
  Match: TFilenameCaseMatch;
begin
  FN2 := ExpandFileNameCase (FN1, Match);
  if (Match <> ExpMatch) or ((ExpReturn <> '') and (FN2 <> ExpReturn) and
     ((Match <> mkAmbiguous) or not (FileNameCaseSensitive) or
                               (UpperCase (FN2) <> UpperCase (ExpReturn)))) then
   begin
    Inc (Failed);
    WriteLn ('Error: Input = ', FN1, ', Output = ', FN2, ' (expected ', ExpReturn, '), MatchFound = ',
{$IFNDEF FPC}
              FileNameCaseMatchStr [
{$ENDIF FPC}
                                    Match
{$IFNDEF FPC}
                                         ]
{$ENDIF FPC}
                                          , ' (expected ',
{$IFNDEF FPC}
                                                           FileNameCaseMatchStr [
{$ENDIF FPC}
                                                                                 ExpMatch
{$IFNDEF FPC}
                                                                                         ]
{$ENDIF FPC}
                                                                                          , ')');
   end
{$IFDEF DEBUG}
  else
   WriteLn ('Input = ', FN1, ', Output = ', FN2, ', MatchFound = ',
{$IFNDEF FPC}
              FileNameCaseMatchStr [
{$ENDIF FPC}
                                    Match
{$IFNDEF FPC}
                                         ]
{$ENDIF FPC}
                                          )
{$ENDIF DEBUG}
      ;
end;


var
  I: byte;
  TempDir, TestDir: string;
  CurDir: string;


begin
 {$IFNDEF FPC}
  TempDir := ExpandFilename (GetEnvironmentVariable ('TEMP'));
 {$ELSE FPC}
  TempDir := ExpandFilename (GetTempDir);
 {$ENDIF FPC}
  if (Length (TempDir) > MinPathLength) and
                  (TempDir [Length (TempDir)] in AllowDirectorySeparators) then
   TempDir := LeftStr (TempDir, Length (TempDir) - 1);

  CurDir := GetCurrentDir;
{$IFDEF DEBUG}
 {$IFDEF FPC}
  WriteLn ('FileNameCaseSensitive = ', FileNameCaseSensitive);
  WriteLn ('IsFileNameCaseSensitive('''+TempDir+''') = ', IsFileNameCaseSensitive(TempDir));
  Writeln('Setting FileNameCaseSensitive to result of IsFileNameCaseSensitive('''+TempDir+''');');
  FileNameCaseSensitive:=IsFileNameCaseSensitive(TempDir);
 {$ENDIF FPC}
  WriteLn ('TempDir = ', TempDir);
  WriteLn ('SetCurrentDir result = ', SetCurrentDir (TempDir));
  WriteLn ('Current directory = ', GetCurrentDir);
{$ELSE DEBUG}
  SetCurrentDir (TempDir);
{$ENDIF DEBUG}
  for I := 1 to TestFilesNumber do
   FileClose (FileCreate (TestFiles [I]));

  TestExpFNC ('*File1.tst', ExpandFileName ('testFile1.tst'), mkExactMatch);
  if FileNameCaseSensitive then
   TestExpFNC ('TestFile1.tst', ExpandFileName ('testFile1.tst'), mkSingleMatch)
  else
   TestExpFNC ('TestFile1.tst', ExpandFileName ('testFile1.tst'), mkExactMatch);
  TestExpFNC ('testFile1.tst', ExpandFileName ('testFile1.tst'), mkExactMatch);
  TestExpFNC ('testFile2.tst', ExpandFileName ('testFile2.tst'), mkExactMatch);
  if FileNameCaseSensitive then
   TestExpFNC ('TestFile2.tst', ExpandFileName ('TestFile2.tst'), mkExactMatch)
  else
   TestExpFNC ('TestFile2.tst', ExpandFileName ('testFile2.tst'), mkExactMatch);
  if FileNameCaseSensitive then
   TestExpFNC ('testfile2.tst', ExpandFileName ('testFile2.tst'), mkAmbiguous)
  else
   TestExpFNC ('testfile2.tst', ExpandFileName ('testFile2.tst'), mkExactMatch);
(* Return value depends on ordering of files in the particular filesystem used thus not checked *)
  TestExpFNC ('*File2.tst', '', mkExactMatch);
  if FileNameCaseSensitive then
   TestExpFNC ('*File*.tst', '', mkExactMatch)
  else
   TestExpFNC ('*File*.tst', '', mkExactMatch);
  TestExpFNC ('..' + DirectorySeparator + '*' + DirectorySeparator + '*File*.tst',
     ExpandFileName ('..' + DirectorySeparator + '*' + DirectorySeparator + '*File*.tst'),
                                                                                    mkNone);
  I := Length (TempDir);
  TestDir := TempDir;
  while (I > 1) and not (TempDir [I] in ['a'..'z','A'..'Z']) do
   Dec (I);
  if I > 0 then
   begin
    if TestDir [I] in ['a'..'z'] then
     TestDir [I] := char (Ord (TestDir [I]) and not $20)
    else
     TestDir [I] := char (Ord (TestDir [I]) or $20);
   end
  else
   WriteLn ('Warning: Cannot perform all required tests; please set TEMP!');
  if FileNameCaseSensitive then
   TestExpFNC (TestDir + DirectorySeparator + 'testFile1.tst',
               ExpandFileName (TempDir + DirectorySeparator + 'testFile1.tst'), mkSingleMatch)
  else
   TestExpFNC (TestDir + DirectorySeparator + 'testFile1.tst',
               ExpandFileName (TestDir + DirectorySeparator + 'testFile1.tst'), mkExactMatch);
  if FileNameCaseSensitive then
   TestExpFNC (TestDir + DirectorySeparator + 'testfile1.tst',
               ExpandFileName (TempDir + DirectorySeparator + 'testFile1.tst'), mkSingleMatch)
  else
   TestExpFNC (TestDir + DirectorySeparator + 'testfile1.tst',
               ExpandFileName (TestDir + DirectorySeparator + 'testFile1.tst'), mkExactMatch);
  for I := 1 to TestFilesNumber do
   if not (DeleteFile (TestFiles [I])) then
    begin
     if FileNameCaseSensitive or (I <> 3) then
      WriteLn ('Warning: Deletion of ', TestFiles [I], ' (file #', I, ') failed - possibly due to case insensitive file system!');
    end;
  SetCurrentDir (CurDir);
  if Failed > 0 then
   begin
    WriteLn (Failed, ' failures!!');
    Halt (Failed);
   end;
end.
