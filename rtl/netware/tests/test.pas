Program Test;
{$Description Test for FreePascal Netware-RTL}
{$Version 1.1.0}

{$I-}
{$Mode Delphi}

USES Strings, Dos, SysUtils, CRT, Video, Keyboard;

TYPE Str255 = STRING [255];

PROCEDURE ErrorCheck (Action,FN : STRING);
VAR Err : INTEGER;
BEGIN
  Err := IOResult;
  IF Err = 0 THEN
  BEGIN
    WriteLn (' OK');
    EXIT;
  END;
  WriteLn (' ! Error (',Action,' in ',FN,'), IOResult: ',Err);
  HALT;
END;

PROCEDURE FileTest;
CONST TestFN = 'SYS:TEST/TEST.DAT';
      NumBlocks = 100;
      BlockSize = 1024;
VAR F      : FILE;
    Err    : LONGINT;
    Buffer : ARRAY [0..BlockSize-1] OF BYTE;
    Written: LONGINT;
    I      : BYTE;
    J      : LONGINT;
BEGIN
  Write ('Creating ',TestFN);
  Assign (F,TestFN);
  ReWrite (F,1);
  ErrorCheck ('Create',TestFN);
  FOR I := 1 TO NumBlocks DO
  BEGIN
    FillChar (Buffer, SIZEOF (Buffer), CHAR(I));
    Write ('BlockWrite');
    BlockWrite (F,Buffer,SIZEOF(Buffer));
    ErrorCheck ('BlockWrite',TestFN);
  END;
  Write ('Seek');
  Seek (F,0);
  ErrorCheck ('Seek',TestFN);
  FOR I := 1 TO NumBlocks DO
  BEGIN
    Write ('BlockRead');
    BlockRead (F,Buffer,SIZEOF(Buffer));
    ErrorCheck ('BlockRead',TestFN);
    FOR J := LOW (Buffer) TO HIGH (Buffer) DO
      IF Buffer[J] <> I THEN
      BEGIN
        WriteLn ('Verify-Error');
        HALT;
      END;
  END;
  Write ('Close');
  Close (F);
  ErrorCheck ('Close',TestFN);
  Write ('Erase');
  Erase (F);
  ErrorCheck ('Erase',TestFN);
END;

PROCEDURE TextFileTest;
CONST NumLines = 100;
      FN = 'SYS:TEST/TEST.TXT';
VAR I : LONGINT;
    S,S1 : STRING;
    T : TEXT;
BEGIN
  Assign (T,FN);
  ReWrite (T);
  ErrorCheck ('ReWrite',FN);
  FOR I := 1 TO NumLines DO
  BEGIN
    Str (I, S);
    Write ('WriteLn');
    WriteLn (T, S);
    ErrorCheck ('WriteLn',FN);
  END;
  Write ('Close'); Close (T); ErrorCheck ('Close',FN);
  Assign (T,FN);
  Reset (T);
  ErrorCheck ('Reset',FN);
  FOR I := 1 TO NumLines DO
  BEGIN
    Str (I, S1);
    Write ('ReadLn');
    ReadLn (T, S);
    ErrorCheck ('ReadLn',FN);
    IF (S <> S1) THEN
    BEGIN
      WriteLn ('Verify-Error "',S,'" <> "',S1,'"');
      HALT;
    END;
  END;
  Write ('Close'); Close (T); ErrorCheck ('Close',FN);
  Write ('Erase'); Erase (T); ErrorCheck ('Erase',FN);
END;


PROCEDURE MemTest;
CONST NumBlocks = 1000;
      BlockSize = 1024;
VAR I : LONGINT;
    P : ARRAY [0..NumBlocks-1] OF POINTER;
BEGIN
  Write ('GetMem/FreeMem Test');
  FillChar (P, SIZEOF(P), 0);
  FOR I := 0 TO NumBlocks-1 DO
  BEGIN
    Write ('g');
    GetMem (P[I],BlockSize);
    FillChar (P[I]^,BlockSize,$FF);
  END;
  FOR I := 0 TO NumBlocks-1 DO
  BEGIN
    Write ('f');
    FreeMem (P[I],BlockSize);
  END;
  WriteLn (' Ok');
END;

PROCEDURE DosTest;
VAR Year, Month, Day, DayVal, hour, Minute, Second, Sec100 : WORD;
BEGIN
  GetDate (Year,Month, Day, DayVal);
  WriteLn ('GetDate: ',Year,'/',Month,'/',Day);
  GetTime (hour, Minute, Second, Sec100);
  WriteLn ('GetTime: ',Hour,':',Minute,':',Second,':',Sec100);
END;

PROCEDURE ExceptTest;
BEGIN
  TRY
    WriteLn ('Raising Exception');
    Raise (Exception.Create (''));
  EXCEPT
    WriteLn ('Fine, Except-Handler called');
  END;
END;

{PROCEDURE ReadDirTest;
VAR EntryH, DirH : PNWDirEnt;
    T : DateTime;
BEGIN
  DirH := _opendir ('SYS:TEST/*.*');
  IF DirH <> NIL THEN
  BEGIN
    EntryH := _readdir (DirH);
    WHILE (EntryH <> NIL) DO
    BEGIN
      unpacktime (EntryH^.d_time + (LONGINT (EntryH^.d_date) SHL 16),T);
      WriteLn ('Name: "', EntryH^.d_nameDOS,'" size:',EntryH^.d_size,' namespace-name: "',EntryH^.d_name,'" ',T.Day,'.',T.Month,'.',T.Year,'  ',T.Hour,':',T.Min,':',T.Sec);
      EntryH := _readdir (DirH);
    END;
    _closedir (DirH);
  END ELSE
    WriteLn ('opendir failed');
END;}


PROCEDURE FindTest;
VAR f : Dos.SearchRec;
    t : Dos.DateTime;
    s : string [5];
    fh: FILE;
    time: LONGINT;
    attr: word;
BEGIN
  Dos.FindFirst ('SYS:TEST\*.*',anyfile,f);
  WHILE Dos.DosError = 0 DO
  BEGIN
    unpacktime (f.time,t);
    IF f.attr AND directory <> 0 THEN
      S := '<DIR>'
    ELSE
      S := '';
    WriteLn (f.Name:15,f.attr:6,S:6,f.size:6,' ',t.Month:2,'/',t.day:2,'/',t.year,'  ',t.hour:2,':',t.min:2,':',t.sec:2);
    Dos.FindNext (f);
  END;
  Dos.FindClose (f);
  {WriteLn ('Directories:');
  Dos.FindFirst ('SYS:SYSTEM\*.*',directory,f);
  WHILE Dos.DosError = 0 DO
  BEGIN
    WriteLn (f.Name:15);
    Dos.FindNext (f);
  END;
  Dos.FindClose (f);}
  WriteLn;
  Assign (FH,ParamStr(0));
  Reset (FH,1);
  ErrorCheck ('Reset',ParamStr(0));
  Getftime (FH, time);
  Getfattr (FH, attr);
  Close (FH);
  unpacktime (time,t);
  WriteLn (ParamStr(0),attr:6,' ',t.Month:2,'/',t.day:2,'/',t.year,'  ',t.hour:2,':',t.min:2,':',t.sec:2);
  WriteLn ('GetEnv (XX): "',GetEnv ('XX'),'"');
END;

{PROCEDURE VolInfo;
VAR I : LONGINT;
    Buf: ARRAY [0..255] OF CHAR;
    TotalBlocks  : WORD;
    SectorsPerBlock : WORD;
    availableBlocks : WORD;
    totalDirectorySlots : WORD;
    availableDirSlots   : WORD;
    volumeisRemovable   : WORD;
    Err : LONGINT;
BEGIN
  WriteLn ('Number of Volumes: ',_GetNumberOfVolumes);
  FOR I := 0 TO _GetNumberOfVolumes-1 DO
  BEGIN
    _GetVolumeName (I,@Buf);
    WriteLn (I,': "',Buf,'"');
    Err := _GetVolumeInfoWithNumber (I,@Buf,
                                 TotalBlocks,
                                 SectorsPerBlock,
                                 availableBlocks,
                                 totalDirectorySlots,
                                 availableDirSlots,
                                 volumeisRemovable);
    IF Err = 0 THEN
    BEGIN
      WriteLn ('TotalBlocks: ',TotalBlocks,' Sectors/Block: ',SectorsPerBlock,' avail: ',availableBlocks);
    END ELSE
      WriteLn ('Err: ',Err);
  END;
  FOR I := 0 TO 5 DO
  BEGIN
    WriteLn ('DiskFree(',I,'): ',Dos.DiskFree(I));
    WriteLn ('DiskSize(',I,'): ',Dos.DiskSize(I));
  END;

END;}

PROCEDURE CrtTest;
VAR C : CHAR;
    I : INTEGER;

    PROCEDURE KeyTest;
    VAR C : CHAR;
    BEGIN
      WriteLn ('Key-Test, CR will be converted to ausgegeben, End with ESC');
      Repeat
        C := ReadKey;
        CASE C OF
          #0 : Write ('#0');
          #13: Write (#13#10)
          ELSE Write (C);
        END;
      Until C = #27;
    END;

    PROCEDURE FillScreen;
    VAR I : INTEGER;
    BEGIN
      ClrScr;
      TextColor (Green);
      FOR I := 1 TO 24 DO
        Write ('12345678901234567890123456789012345678901234567890123456789012345678901234567890');
      TextColor (Yellow);
      FOR I := 1 TO 25 DO
      BEGIN
        GotoXY (76,I); Write (' ',I,' ');
      END;
      TextColor (LightGray);
    END;

BEGIN
  {GotoXY (1,1); writeln ('Text @ 1,1');
  GotoXY (2,2); writeln ('Text @ 2,2');
  GotoXY (3,3); writeln ('Text @ 3,3');
  GotoXY (4,4); writeln ('Text @ 4,4, Delay 5 Secs');
  GotoXY (1,1);
  IF WhereX <> 1 THEN
  BEGIN
    GotoXY (1,10); Write ('WhereX - ERROR');
  END;
  GotoXY (1,1);
  IF WhereY <> 1 THEN
  BEGIN
    GotoXY (1,11); Write ('WhereY - ERROR');
  END;

  Delay (1000);
  }
  ClrScr;

  WriteLn ('Empty Screen ');
  Delay (1000);
  WriteLn ('Cursoroff '); CursorOff;
  Delay (1000);
  WriteLn ('Cursorbig '); CursorBig;
  Delay (1000);
  WriteLn ('Cursoron '); CursorOn;
  LowVideo; Write ('Low '); HighVideo; Write ('High '); LowVideo; Write ('Low ');
  Delay (1000);
  KeyTest;
  FillScreen;
  Window (10,10,40,15);
  ClrScr; Write ('Window 10,10,20,15');
  KeyTest;
  Window (1,1,80,25);
  FillScreen;
  GotoXY (10,10); ClrEol;
  GotoXY (1,21); Write (' ClrEol @ 10,10 ');
  ReadKey;
  FillScreen;
  GotoXY (10,10); InsLine;
  GotoXY (1,21); Write (' Insline @ 10,10 ');
  ReadKey;
  Write ('Waiting for keypress: ');
  WHILE NOT Keypressed DO
  BEGIN
    Delay (500);
  END;
  Write ('OK'); ReadKey;
  FOR I := 1 TO 5 DO
  BEGIN
    Write (^G); Delay (200);
  END;


  Delay (1000);
  GotoXY (1,25); ClrEol;
END;

{
Function FileSetDate (Handle: longint; Age: Int64) : Longint;
Function FileSetAttr (Const Filename : String; Attr: longint) : Longint;
}
PROCEDURE SysUtilsTest;
VAR H,I,Attr : LONGINT;
    X : ARRAY [0..255] OF CHAR;
    TD: TDateTime;
    SR: TSearchRec;
    ST1,ST2: STRING;
BEGIN
  WriteLn ('FileExists SYS:SYSTEM/CLIB.NLM: ',FileExists ('SYS:SYSTEM/CLIB.NLM'));
  WriteLn ('FileExists SYS:SYSTEM\CLIB.NLM: ',FileExists ('SYS:SYSTEM\CLIB.NLM'));
  WriteLn ('FileExists SYS:SYSTEM/CLIB.N: ',FileExists ('SYS:SYSTEM/CLIB.N'));
  WriteLn ('FileExists SYS:SYSTEM\CLIB.N: ',FileExists ('SYS:SYSTEM\CLIB.N'));
  WriteLn ('FileExists SYS:SYSTEM: ',FileExists ('SYS:SYSTEM\CLIB.N'));

  H := FileOpen ('SYS:TEST/Autoexec.ncf',0);
  IF H >= 0 THEN
  BEGIN
    I := FileRead (H, X, 20); X[20] := #0;
    WriteLn ('FileRead returned ',I,' Buffer: "',X,'"');
  END ELSE
    WriteLn ('FileOpen failed');
  FileClose (H);

  H := FileAge ('SYS:SYSTEM/CLIB.NLM');
  TD := FileDateToDateTime (H);
  WriteLn ('CLIBs file date: ',DateTimeToStr (TD));
  H := FileAge ('SYS:SYSTEM/DSREPAIR.LOG');
  TD := FileDateToDateTime (H);
  WriteLn ('DSREPAIR.LOGs file date: ',DateTimeToStr (TD));
  H := SysUtils.FindFirst ('SYS:SYSTEM/CLIB.nlm',faAnyFile,SR);
  IF H = 0 THEN
  BEGIN
    WriteLn (SR.Name:20,SR.Size:6,'  ',DateTimeToStr (FileDateToDateTime (SR.time)):20,' ',hexstr (SR.attr,8));
  END ELSE WriteLn ('FindFirst failed');
  FindClose (SR);

  H := SysUtils.FindFirst ('SYS:SYSTEM/CLIB.N',faAnyFile,SR);
  IF H = 0 THEN
    WriteLn ('FindFirst on non existing file returned 0 !');
  FindClose (SR);

  H := SysUtils.FindFirst ('SYS:SYSTEM/DSREPAIR.LOG',faAnyFile,SR);
  IF H = 0 THEN
  BEGIN
    WriteLn (SR.Name:20,SR.Size:6,'  ',DateTimeToStr (FileDateToDateTime (SR.time)):20,' ',hexstr (SR.attr,8));
  END ELSE WriteLn ('FindFirst failed');
  FindClose (SR);

  H := FileOpen ('SYS:SYSTEM/DSRepair.log',0);
  IF H >= 0 THEN
  BEGIN
    I := FileGetDate (H);
    FileClose (H);
    TD := FileDateToDateTime (I);
    WriteLn ('DSREPAIR.LOGs file date via FileGetDate: ',DateTimeToStr (TD));
  END ELSE WriteLn ('FileOpen failed');
  Attr := FileGetAttr ('SYS:SYSTEM/CLIB.NLM');
  WriteLn ('Attr of clib: ',hexstr (Attr,8));

  chdir ('sys:test');
  H := FileCreate ('TEST12.DAT');
  IF H >= 0 THEN
  BEGIN
    IF NOT FileExists ('SYS:TEST/TEST12.DAT') THEN
      WriteLn ('FileCreate returned ok but FileExists returned false !');
    FillChar (X,SIZEOF(X),BYTE('X'));
    I := FileWrite (H,X,SIZEOF(X));
    WriteLn ('FileWrite returned ',I);
    IF I = SIZEOF (X) THEN
    BEGIN
      IF NOT FileTruncate (H,SIZEOF(X) DIV 2) THEN
        WriteLn ('FileTruncate failed');
    END;
    FileClose (H);

    I := SysUtils.FindFirst ('TEST12.DAT',faAnyFile,SR);
    IF I <> 0 THEN
      WriteLn ('FindFirst failed')
    ELSE
     IF SR.Size <> (SIZEOF (X) DIV 2) THEN
      WriteLn ('FileTruncate: wrong FileSize after truncate (',SR.Size,')');
    FindClose (SR);

    IF NOT RenameFile ('TEST12.DAT','TEST12.BAK') THEN
      WriteLn ('RenameFile failed')
    ELSE
    BEGIN
      IF NOT FileExists ('SYS:TEST/TEST12.BAK') THEN
        WriteLn ('FileRename returned ok but FileExists returned false');
      IF NOT DeleteFile ('TEST12.BAK') THEN
        WriteLn ('DeleteFile failed')
      ELSE
        IF FileExists ('SYS:TEST/TEST12.BAK') THEN
          WriteLn ('DeleteFile returned ok but FileExists returned true');
    END;

  END ELSE WriteLn ('FileCreate failed');

  H := FileCreate ('TEST12.DAT');
  IF H >= 0 THEN
  BEGIN
    FillChar (X,SIZEOF(X),BYTE('X'));
    FileWrite (H,X,SIZEOF(X));
    I := FileSeek (H,10,fsFromBeginning);
    X[0] := '0';
    FileWrite (H,X,1);
    IF I <> 10 THEN WriteLn ('FileSeek returned wrong result at 10 (',I,')');
    I := FileSeek (H,10,fsFromCurrent);
    X[0] := '1';
    FileWrite (H,X,1);
    IF I <> 21 THEN WriteLn ('FileSeek returned wrong result at 21 (',I,')');
    I := FileSeek (H,-10,fsFromEnd);
    X[0] := '2';
    FileWrite (H,X,1);
    IF I <> SIZEOF(X)-10 THEN WriteLn ('FileSeek returned wrong result at End-10 (',I,')');
    FileClose (H);
  END ELSE WriteLn ('FileCreate failed');

  ST1 := 'SYS:ETC;SYS:TEST;SYS:SYSTEM/;SYS:PUBLIC';
  ST2 := FileSearch ('clib.nlm',ST1);
  WriteLn ('FileSearch (clib.nlm,',ST1,') returned "',ST2,'"');
  WriteLn ('FExpand (TEST12.DAT): "',FExpand ('TEST12.DAT'));
  WriteLn ('FExpand (.\TEST12.DAT): "',FExpand ('.\TEST12.DAT'));
  WriteLn ('FExpand (..\SYSTEM\CLIB.NLM): "',FExpand ('..\SYSTEM\CLIB.NLM'));

END;


PROCEDURE VideoTest;

  PROCEDURE WriteString (S : STRING; X,Y : WORD; Fore,Back: BYTE);
  VAR I : INTEGER;
      W : WORD;
      P : POINTER;
      Textattr : WORD;
  BEGIN
    W := X + (Y * Video.ScreenWidth);
    P := Pointer (@VideoBuf^[W]);
    TextAttr := (Fore and $f) or (Back shl 4);
    FOR I := 1 TO Length (S) DO
    BEGIN
      W := (TextAttr SHL 8) or byte (S[I]);
      PWord(P)^ := w;
      INC (PChar(P),2);
    END;
  END;

BEGIN
  InitVideo;
  Video.ClearScreen;
  WriteString ('Test @ 0,0, LightGray on Black',0,0,LightGray,Black);
  UpdateScreen (false);
  WriteString ('Test @ 10,1, Yellow on Blue',1,1,Yellow,Blue);
  UpdateScreen (false);
  ReadKey;
  Video.ClearScreen;
  WriteString ('Cursor crHidden',0,0,Yellow,Blue);
  SetCursorPos (0,0);
  SetCursorType (crHidden);
  UpdateScreen (false);
  ReadKey;

  Video.ClearScreen;
  WriteString ('Cursor crUnderLine',0,0,Yellow,Blue);
  SetCursorPos (0,0);
  SetCursorType (crUnderLine);
  UpdateScreen (false);
  ReadKey;

  Video.ClearScreen;
  WriteString ('Cursor crBlock',0,0,Yellow,Blue);
  SetCursorPos (0,0);
  SetCursorType (crBlock);
  UpdateScreen (false);
  ReadKey;

  Video.ClearScreen;
  WriteString ('Cursor crHalfBlock',0,0,Yellow,Blue);
  SetCursorPos (0,0);
  SetCursorType (crHalfBlock);
  UpdateScreen (false);
  ReadKey;

  CRT.ClrScr;
  SetCursorType (crUnderLine);
END;

PROCEDURE KeyboardTest;
VAR T : TKeyEvent;
BEGIN
  InitKeyboard;
  WriteLn ('Keyboard-Test, ESC Ends');
  REPEAT
    T := GetKeyEvent;
    WriteLn ('           Event: ',HexStr (T,8),' EventChar: "',GetKeyEventChar(T),'" KeyEventCode: ',HexStr (GetKeyEventCode(T),8));
    T := TranslateKeyEvent (T);
    WriteLn ('Translated Event: ',HexStr (T,8),' EventChar: "',GetKeyEventChar(T),'" KeyEventCode: ',HexStr (GetKeyEventCode(T),8));
    WriteLn;
  UNTIL GetKeyEventChar (T) = #27;
END;


VAR I : LONGINT;
    S : STRING [255];
    C : CHAR;
    P : ^Str255;
BEGIN
  New (P);
  Dispose (P);
  // WriteLn ('Test');
  //__ConsolePrintf ('Ok, this is PASCALMAIN'#13#10,0);
  WriteLn ('Test via WriteLn');
  WriteLn ('No of params: ', ParamCount);
  //__EnterDebugger;
  WriteLn ('ParamStr(0): "', ParamStr(0),'"');
  IF ParamCount > 0 THEN
    FOR I := 1 TO ParamCount DO
      WriteLn (I:6,': "',ParamStr(I),'"');
  GetDir (0, S);
  WriteLn ('Current Directory: "',S,'"');
//  ChDir ('TEST');
//  GetDir (0, S);
//  WriteLn ('Current Directory: "',S,'"');
//  MkDir ('SYS:TEST');
//  IF IOResult <> 0 THEN WriteLn ('MkDir SYS:TEST failed (Ok)');
//  Write ('MkDir'); MkDir ('SYS:TEST/TESTDIR');
//  ErrorCheck ('MkDir','SYS:TEST/TESTDIR');
//  Write ('RmDir'); RmDir ('SYS:TEST/TESTDIR');
//  ErrorCheck ('RmDir','SYS:TEST/TESTDIR');

  REPEAT
    WriteLn;
    WriteLn ('1  : File-Test');
    WriteLn ('2  : Textfile-Test');
    WriteLn ('3  : GetMem/FreeMem Test');
    WriteLn ('4  : DosTest');
    WriteLn ('5  : ExceptTest');
    WriteLn ('6  : Video-Test');
    WriteLn ('7  : Find-Test');
    WriteLn ('8  : SysUtils-Test');
    WriteLn ('9  : CrtTest');
    WriteLn ('K  : Keyboard-Test');
    WriteLn ('E  : Ende');
    WriteLn;
    Write ('?: ');
    C := Crt.ReadKey;
    WriteLn (C);
    CASE upcase(C) OF
      '1' : FileTest;
      '2' : TextfileTest;
      '3' : MemTest;
      '4' : DosTest;
      '5' : ExceptTest;
      '6' : VideoTest;
      '7' : FindTest;
      '8' : SysUtilsTest;
      '9' : CrtTest;
      'K' : KeyboardTest;
    END;
  UNTIL UpCase (C) = 'E';
  (*$IFDEF Netware*)
  PressAnyKeyToContinue;
  (*$ENDIF*)
END.
