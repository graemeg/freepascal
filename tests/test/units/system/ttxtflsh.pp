{ $DEFINE VERBOSE}
{ $DEFINE DEBUG}
(* Define the following if the test will be run manually in a console       *)
(* (no output redirection) - otherwise the test will fail for some targets. *)
{ $DEFINE TESTINCONSOLE}
{$I-}
uses
 Dos;
var
 T: text;
 IOR: integer;
 TElapsed1, TElapsed2: int64;
 I: longint;
const
 TestFName = 'ttxtflsh.txt';
 NoTestFName = '_NoSuchF.FFF';
{$IF DEFINED(OS2) or DEFINED(WINDOWS) or DEFINED(GO32V2) or DEFINED(WATCOM) or DEFINED(MSDOS)}
 ConsoleDeviceName = 'CON';
 {$DEFINE TESTCONSOLEOK}
{$ELSE}
(* {$IF DEFINED(UNIX)}
{ I don't know whether there's a device on Unix allowing to enforce output to console even if standard output is redirected for the given process... }
 ConsoleDeviceName = '/dev/tty';
  {$DEFINE TESTCONSOLEOK}
 {$ELSE}
*)
  {$IFDEF TESTINCONSOLE}
 ConsoleDeviceName = '';
   {$DEFINE TESTCONSOLEOK}
  {$ENDIF TESTINCONSOLE}
 { $ENDIF}
{$ENDIF}


procedure ChkErr (Err: boolean; MsgOK, MsgErr: string; N: byte);
begin
 if Err then
  begin
   if IOResult = 0 then
    begin
    end;
   WriteLn ('Error: ', MsgErr);
{$IFDEF VERBOSE}
   WriteLn ('Exit value: ', N);
{$ENDIF VERBOSE}
   Halt (N);
  end
{$IFDEF VERBOSE}
 else
  WriteLn (MsgOK)
{$ENDIF VERBOSE}
     ;
end;

function PerfTest: int64;
var
 T1: int64;
begin
 T1 := GetMsCount;
 for I := 0 to 50000 do
  Write (T, I);
 PerfTest := GetMsCount - T1;
 ChkErr (IOResult <> 0, 'Test text output successful.',
  'Test text output failed!', 255);
end;

begin
 Assign (T, NoTestFName);
 Reset (T);
 SetTextAutoFlush (T, true);
 ChkErr (GetTextAutoFlush (T),
  'Set/GetTextAutoFlush call correctly ignored with non-zero InOutRes.',
  'Set/GetTextAutoFlush call not ignored in spite of non-zero InOutRes!', 1);
 if IOResult <> 0 then
  begin
  end;
 SetTextAutoFlush (T, true);
 IOR := IOResult;
{$IFDEF DEBUG}
 WriteLn (StdErr, IOR);
{$ENDIF DEBUG}
 ChkErr (IOR <> 103,
  'SetTextAutoFlush correctly errors out with expected RTE if file not open.',
  'SetTextAutoFlush does not finish with expected RTE if file not open!', 2);
 Assign (T, TestFName);
 Rewrite (T);
 ChkErr (IOResult <> 0, 'Test file ' + TestFName + ' created successfully.',
  'Test file ' + TestFName + ' creation failed!', 3);
 Close (T);
 Reset (T);
 SetTextAutoFlush (T, true);
 IOR := IOResult;
{$IFDEF DEBUG}
 WriteLn (StdErr, IOR);
{$ENDIF DEBUG}
 ChkErr (IOR <> 105,
  'SetTextAutoFlush correctly errors out if file not open for writing.',
  'SetTextAutoFlush does not finish with expected RTE if file not open for writing!', 4);
 Close (T);
 Rewrite (T);
{$IFDEF DEBUG}
 WriteLn (StdErr, Longint (TextRec(T).FlushFunc));
{$ENDIF DEBUG}
 ChkErr (GetTextAutoFlush (T) or (IOResult <> 0),
  'GetTextAutoFlush returns expected default value for a regular file.',
  'GetTextAutoFlush returns unexpected default value for a regular file!', 5);
 SetTextAutoFlush (T, true);
{$IFDEF DEBUG}
 WriteLn (StdErr, Longint (TextRec(T).FlushFunc));
{$ENDIF DEBUG}
 ChkErr (not (GetTextAutoFlush (T)) or (IOResult <> 0),
  'GetTextAutoFlush returns expected modified value after SetTextAutoFlush.',
  'GetTextAutoFlush does not return expected modified value after SetTextAutoFlush!', 6);
 TElapsed1 := PerfTest;
{$IFDEF DEBUG}
 WriteLn (StdErr, 'Run 1: ', TElapsed1, ' ms');
{$ENDIF DEBUG}
 Close (T);
 Rewrite (T);
{$IFDEF DEBUG}
 WriteLn (StdErr, Longint (TextRec(T).FlushFunc));
{$ENDIF DEBUG}
 ChkErr (GetTextAutoFlush (T) or (IOResult <> 0),
  'GetTextAutoFlush returns expected default value after file reopening.',
  'GetTextAutoFlush does not return expected default value after file reopening!', 7);
 TElapsed2 := PerfTest;
{$IFDEF DEBUG}
 WriteLn (StdErr, 'Run 2: ', TElapsed2, ' ms');
{$ENDIF DEBUG}
 ChkErr (TElapsed1 <= TElapsed2,
  'Output performance lower with enforced flushing as expected.',
  'Output performance not lower with enforced flushing!', 11);
 Close (T);
 Append (T);
{$IFDEF DEBUG}
 WriteLn (StdErr, Longint (TextRec(T).FlushFunc));
{$ENDIF DEBUG}
 ChkErr (GetTextAutoFlush (T) or (IOResult <> 0),
  'GetTextAutoFlush returns expected default value after file reopening for appending.',
  'GetTextAutoFlush does not return expected default value after file reopening for appending!', 13);
 Close (T);
 Erase (T);
 if IOResult <> 0 then
  begin
  end;

{$IFDEF TESTCONSOLEOK}
 Assign (T, ConsoleDeviceName);
 Rewrite (T);
{$IFDEF DEBUG}
 WriteLn (StdErr, Longint (TextRec(T).FlushFunc));
{$ENDIF DEBUG}
 ChkErr (not (GetTextAutoFlush (T)) or (IOResult <> 0),
  'GetTextAutoFlush returns expected default value for console output.',
  'GetTextAutoFlush returns unexpected default value for console output!', 8);
 SetTextAutoFlush (T, false);
{$IFDEF DEBUG}
 WriteLn (StdErr, Longint (TextRec(T).FlushFunc));
{$ENDIF DEBUG}
 ChkErr (GetTextAutoFlush (T) or (IOResult <> 0),
  'GetTextAutoFlush returns expected modified value after SetTextAutoFlush with console.',
  'GetTextAutoFlush does not return expected modified value after SetTextAutoFlush with console!', 9);
 TElapsed1 := PerfTest;
{$IFDEF DEBUG}
 WriteLn (StdErr, 'Run 1: ', TElapsed1, ' ms');
{$ENDIF DEBUG}
 Close (T);
 Rewrite (T);
{$IFDEF DEBUG}
 WriteLn (StdErr, Longint (TextRec(T).FlushFunc));
{$ENDIF DEBUG}
 ChkErr (not (GetTextAutoFlush (T)) or (IOResult <> 0),
  'GetTextAutoFlush returns expected default value after file reopening for console.',
  'GetTextAutoFlush returns unexpected default value after file reopening for console!', 10);
 TElapsed2 := PerfTest;
{$IFDEF DEBUG}
 WriteLn (StdErr, 'Run 2: ', TElapsed2, ' ms');
{$ENDIF DEBUG}
 ChkErr (TElapsed1 >= TElapsed2,
  'Output performance higher with disabled flushing as expected.',
  'Output performance not higher with disabled flushing!', 12);
 Close (T);

{$ENDIF TESTCONSOLEOK}

{$IFDEF VERBOSE}
 WriteLn ('TTxtFlsh finished successfully.');
{$ENDIF VERBOSE}
end.
