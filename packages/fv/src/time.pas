{*********************[ TIME UNIT ]************************}
{                                                          }
{             System independent TIME unit                 }
{                                                          }
{   Copyright (c) 1996, 1997, 1998, 1999 by Leon de Boer   }
{   ldeboer@attglobal.net  - primary e-mail address        }
{   ldeboer@starwon.com.au - backup e-mail address         }
{                                                          }
{****************[ THIS CODE IS FREEWARE ]*****************}
{                                                          }
{     This sourcecode is released for the purpose to       }
{   promote the pascal language on all platforms. You may  }
{   redistribute it and/or modify with the following       }
{   DISCLAIMER.                                            }
{                                                          }
{     This SOURCE CODE is distributed "AS IS" WITHOUT      }
{   WARRANTIES AS TO PERFORMANCE OF MERCHANTABILITY OR     }
{   ANY OTHER WARRANTIES WHETHER EXPRESSED OR IMPLIED.     }
{                                                          }
{*****************[ SUPPORTED PLATFORMS ]******************}
{     16 and 32 Bit compilers                              }
{        DOS      - Turbo Pascal 7.0 +      (16 Bit)       }
{        DPMI     - Turbo Pascal 7.0 +      (16 Bit)       }
{                 - FPC 0.9912+ (GO32V2)    (32 Bit)       }
{        WINDOWS  - Turbo Pascal 7.0 +      (16 Bit)       }
{                 - Delphi 1.0+             (16 Bit)       }
{        WIN95/NT - Delphi 2.0+             (32 Bit)       }
{                 - Virtual Pascal 2.0+     (32 Bit)       }
{                 - Speedsoft Sybil 2.0+    (32 Bit)       }
{                 - FPC 0.9912+             (32 Bit)       }
{        OS2      - Virtual Pascal 1.0+     (32 Bit)       }
{                 - Speed Pascal 1.0+       (32 Bit)       }
{                 - C'T patch to BP         (16 Bit)       }
{                                                          }
{******************[ REVISION HISTORY ]********************}
{  Version  Date        Fix                                }
{  -------  ---------   ---------------------------------  }
{  1.00     06 Dec 96   First multi platform release.      }
{  1.10     06 Jul 97   New functiions added.              }
{  1.20     22 Jul 97   FPC pascal compiler added.         }
{  1.30     29 Aug 97   Platform.inc sort added.           }
{  1.40     13 Oct 97   Delphi 2/3 32 bit code added.      }
{  1.50     06 Nov 97   Speed pascal code added.           }
{  1.60     05 May 98   Virtual pascal 2.0 compiler added. }
{  1.61     07 Jul 99   Speedsoft SYBIL 2.0 code added.    }
{**********************************************************}

UNIT Time;

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                  INTERFACE
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

{====Include file to sort compiler platform out =====================}
{$I platform.inc}
{====================================================================}

{==== Compiler directives ===========================================}

{$IFNDEF PPC_FPC} { FPC doesn't support these switches }
   {$F-} { Short calls are okay }
   {$A+} { Word Align Data }
   {$B-} { Allow short circuit boolean evaluations }
   {$O+} { This unit may be overlaid }
   {$G+} { 286 Code optimization - if you're on an 8088 get a real computer }
   {$E+} {  Emulation is on }
   {$N-} {  No 80x87 code generation }
{$ENDIF}

{$X+} { Extended syntax is ok }
{$R-} { Disable range checking }
{$S-} { Disable Stack Checking }
{$I-} { Disable IO Checking }
{$Q-} { Disable Overflow Checking }
{$V-} { Turn off strict VAR strings }
{====================================================================}

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{-CurrentMinuteOfDay-------------------------------------------------
Returns the number of minutes since midnight of a current system time.
19Jun97 LdB               (Range: 0 - 1439)
---------------------------------------------------------------------}
FUNCTION CurrentMinuteOfDay: Word;

{-CurrentSecondOfDay-------------------------------------------------
Returns the number of seconds since midnight of current system time.
24Jun97 LdB               (Range: 0 - 86399)
---------------------------------------------------------------------}
FUNCTION CurrentSecondOfDay: LongInt;

{-CurrentSec100OfDay-------------------------------------------------
Returns the 1/100ths of a second since midnight of current system time.
24Jun97 LdB               (Range: 0 - 8639999)
---------------------------------------------------------------------}
FUNCTION CurrentSec100OfDay: LongInt;

{-MinuteOfDay--------------------------------------------------------
Returns the number of minutes since midnight of a valid given time.
19Jun97 LdB               (Range: 0 - 1439)
---------------------------------------------------------------------}
FUNCTION MinuteOfDay (Hour24, Minute: Word): Word;

{-SecondOfDay--------------------------------------------------------
Returns the number of seconds since midnight of a valid given time.
19Jun97 LdB               (Range: 0 - 86399)
---------------------------------------------------------------------}
FUNCTION SecondOfDay (Hour24, Minute, Second: Word): LongInt;

{-SetTime------------------------------------------------------------
Set the operating systems time clock to the given values. If values
are invalid this function will fail without notification.
06Nov97 LdB
---------------------------------------------------------------------}
PROCEDURE SetTime (Hour, Minute, Second, Sec100: Word);

{-GetTime------------------------------------------------------------
Returns the current time settings of the operating system.
06Nov97 LdB
---------------------------------------------------------------------}
PROCEDURE GetTime (Var Hour, Minute, Second, Sec100: Word);

{-MinutesToTime------------------------------------------------------
Returns the time in hours and minutes of a given number of minutes.
19Jun97 LdB
---------------------------------------------------------------------}
PROCEDURE MinutesToTime (Md: LongInt; Var Hour24, Minute: Word);

{-SecondsToTime------------------------------------------------------
Returns the time in hours, mins and secs of a given number of seconds.
19Jun97 LdB
---------------------------------------------------------------------}
PROCEDURE SecondsToTime (Sd: LongInt; Var Hour24, Minute, Second: Word);

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                IMPLEMENTATION
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
{$IFDEF OS_WINDOWS}                                   { WIN/NT CODE }

  {$IFNDEF PPC_SPEED}                                 { NON SPEED COMPILER }
    {$IFDEF PPC_FPC}                                  { FPC WINDOWS COMPILER }
    USEs Windows;                                     { Standard unit }
    {$ELSE}                                           { OTHER COMPILERS }
    USES WinTypes, WinProcs;                          { Standard units }
    {$ENDIF}
  {$ELSE}                                             { SPEEDSOFT COMPILER }
  USES WinBase;                                       { Standard unit }
  TYPE TSystemTime = SystemTime;                      { Type fix up }
  {$ENDIF}

{$ENDIF}

{$IFDEF OS_OS2}                                       { OS2 COMPILERS }

  {$IFDEF PPC_VIRTUAL}                                { VIRTUAL PASCAL }
  USES OS2Base;                                       { Standard unit }
  {$ENDIF}

  {$IFDEF PPC_SPEED}                                  { SPEED PASCAL }
  USES BseDos, Os2Def;                                { Standard unit }
  {$ENDIF}

  {$IFDEF PPC_FPC}                                    { FPC }
  USES Dos, DosCalls;                                 { Standard unit }

  TYPE DateTime = TDateTime;                          { Type correction }
  {$ENDIF}

  {$IFDEF PPC_BPOS2}                                  { C'T PATCH TO BP CODE }
  USES DosTypes, DosProcs;                            { Standard unit }

  TYPE DateTime = TDateTime;                          { Type correction }
  {$ENDIF}

{$ENDIF}

{$ifdef OS_UNIX}
  USES Dos;
{$endif OS_UNIX}

{$ifdef OS_GO32}
  USES Dos;
{$endif OS_GO32}

{$ifdef OS_NETWARE}
  USES Dos;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif OS_NETWARE}

{$ifdef OS_AMIGA}
  USES Dos;
{$endif OS_AMIGA}
=======
{$endif OS_GO32}
>>>>>>> graemeg/fixes_2_2
=======
{$endif OS_GO32}
>>>>>>> origin/fixes_2_2

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{---------------------------------------------------------------------------}
{  CurrentMinuteOfDay -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 24Jun97 LdB}
{---------------------------------------------------------------------------}
FUNCTION CurrentMinuteOfDay: Word;
VAR Hour, Minute, Second, Sec100: Word;
BEGIN
   GetTime(Hour, Minute, Second, Sec100);             { Get current time }
   CurrentMinuteOfDay := (Hour * 60) + Minute;        { Minute from midnight }
END;

{---------------------------------------------------------------------------}
{  CurrentSecondOfDay -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 24Jun97 LdB}
{---------------------------------------------------------------------------}
FUNCTION CurrentSecondOfDay: LongInt;
VAR Hour, Minute, Second, Sec100: Word;
BEGIN
   GetTime(Hour, Minute, Second, Sec100);             { Get current time }
   CurrentSecondOfDay := (LongInt(Hour) * 3600) +
     (Minute * 60) + Second;                          { Second from midnight }
END;

{---------------------------------------------------------------------------}
{  CurrentSec100OfDay -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 24Jun97 LdB}
{---------------------------------------------------------------------------}
FUNCTION CurrentSec100OfDay: LongInt;
VAR Hour, Minute, Second, Sec100: Word;
BEGIN
   GetTime(Hour, Minute, Second, Sec100);             { Get current time }
   CurrentSec100OfDay := (LongInt(Hour) * 360000) +
     (LongInt(Minute) * 6000) + (Second*100)+ Sec100; { Sec100 from midnight }
END;

{---------------------------------------------------------------------------}
{  MinuteOfDay -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19Jun97 LdB       }
{---------------------------------------------------------------------------}
FUNCTION MinuteOfDay (Hour24, Minute: Word): Word;
BEGIN
   MinuteOfDay := (Hour24 * 60) + Minute;             { Minute from midnight }
END;

{---------------------------------------------------------------------------}
{  SecondOfDay -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19Jun97 LdB       }
{---------------------------------------------------------------------------}
FUNCTION SecondOfDay (Hour24, Minute, Second: Word): LongInt;
BEGIN
   SecondOfDay := (LongInt(Hour24) * 3600) +
     (Minute * 60) + Second;                          { Second from midnight }
END;

{---------------------------------------------------------------------------}
{  SetTime -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 06Nov97 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE SetTime (Hour, Minute, Second, Sec100: Word);
{$IFDEF OS_DOS}                                       { DOS/DPMI CODE }
   {$IFDEF ASM_BP}                                    { BP COMPATABLE ASM }
   ASSEMBLER;
   ASM
     MOV CH, BYTE PTR Hour;                           { Fetch hour }
     MOV CL, BYTE PTR Minute;                         { Fetch minute }
     MOV DH, BYTE PTR Second;                         { Fetch second }
     MOV DL, BYTE PTR Sec100;                         { Fetch hundredths }
     MOV AX, $2D00;                                   { Set function id }
     PUSH BP;                                         { Safety save register }
     INT $21;                                         { Set the time }
     POP BP;                                          { Restore register }
   END;
   {$ENDIF}
   {$IFDEF ASM_FPC}                                   { FPC COMPATABLE ASM }
   BEGIN
   ASM
     MOVB Hour, %CH;                                  { Fetch hour }
     MOVB Minute, %CL;                                { Fetch minute }
     MOVB Second, %DH;                                { Fetch second }
     MOVB Sec100, %DL;                                { Fetch hundredths }
     MOVW $0x2D00, %AX;                               { Set function id }
     PUSHL %EBP;                                      { Save register }
     INT $0x21;                                       { BIOS set time }
     POPL %EBP;                                       { Restore register }
   END;
   END;
   {$ENDIF}
{$ENDIF}
{$IFDEF OS_WINDOWS}                                   { WIN/NT CODE }
   {$IFDEF BIT_16}                                    { 16 BIT WINDOWS CODE }
   ASSEMBLER;
   ASM
     MOV CH, BYTE PTR Hour;                           { Fetch hour }
     MOV CL, BYTE PTR Minute;                         { Fetch minute }
     MOV DH, BYTE PTR Second;                         { Fetch second }
     MOV DL, BYTE PTR Sec100;                         { Fetch hundredths }
     MOV AX, $2D00;                                   { Set function id }
     PUSH BP;                                         { Safety save register }
     INT $21;                                         { Set the time }
     POP BP;                                          { Restore register }
   END;
   {$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
   {$IFDEF BIT_32_OR_MORE}                            { 32 BIT WINDOWS CODE }
   VAR DT: TSystemTime;
   BEGIN
     {$IFDEF PPC_FPC}                                 { FPC WINDOWS COMPILER }
     GetLocalTime(@DT);                               { Get the date/time }
     {$ELSE}                                          { OTHER COMPILERS }
     GetLocalTime(DT);                                { Get the date/time }
=======
=======
>>>>>>> origin/fixes_2_2
   {$IFDEF BIT_32}                                    { 32 BIT WINDOWS CODE }
   VAR DT: TSystemTime;
   BEGIN
     {$IFDEF PPC_FPC}                                 { FPC WINDOWS COMPILER }
     GetLocalTime(@DT);                              { Get the date/time }
     {$ELSE}                                          { OTHER COMPILERS }
     GetLocalTime(DT);                               { Get the date/time }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     {$ENDIF}
     DT.wHour := Hour;                                { Transfer hour }
     DT.wMinute := Minute;                            { Transfer minute }
     DT.wSecond := Second;                            { Transfer seconds }
     DT.wMilliseconds := Sec100 * 10;                 { Transfer millisecs }
     SetLocalTime(DT);                               { Set the date/time }
   END;
   {$ENDIF}
{$ENDIF}
{$IFDEF OS_OS2}                                       { OS2 CODE }
VAR DT: DateTime;
BEGIN
   DosGetDateTime(DT);                                { Get the date/time }
   DT.Hours := Hour;                                  { Transfer hour }
   DT.Minutes := Minute;                              { Transfer minute }
   DT.Seconds := Second;                              { Transfer seconds }
   DT.Hundredths := Sec100;                           { Transfer hundredths }
   DosSetDateTime(DT);                                { Set the time }
END;
{$ENDIF}
{$ifdef OS_UNIX}
BEGIN
 {settime is dummy in Linux}
END;
{$endif OS_UNIX}
{$IFDEF OS_NETWARE}
BEGIN
 {settime is dummy in Netware (Libc and Clib) }
END;
{$ENDIF OS_NETWARE}
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF OS_AMIGA}
BEGIN
 { settime is dummy on Amiga }
 { probably could be implemented, but it's low pri... (KB) }
END;
{$ENDIF OS_AMIGA}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{---------------------------------------------------------------------------}
{  GetTime -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 06Nov97 LdB           }
{---------------------------------------------------------------------------}
PROCEDURE GetTime (Var Hour, Minute, Second, Sec100: Word);
{$IFDEF OS_DOS}                                       { DOS/DPMI CODE }
   {$IFDEF ASM_BP}                                    { BP COMPATABLE ASM }
   ASSEMBLER;
   ASM
     MOV AX, $2C00;                                   { Set function id }
     PUSH BP;                                         { Safety save register }
     INT $21;                                         { System get time }
     POP BP;                                          { Restore register }
     XOR AH, AH;                                      { Clear register }
     CLD;                                             { Strings go forward }
     MOV AL, DL;                                      { Transfer register }
     LES DI, Sec100;                                  { ES:DI -> hundredths }
     STOSW;                                           { Return hundredths }
     MOV AL, DH;                                      { Transfer register }
     LES DI, Second;                                  { ES:DI -> seconds }
     STOSW;                                           { Return seconds }
     MOV AL, CL;                                      { Transfer register }
     LES DI, Minute;                                  { ES:DI -> minutes }
     STOSW;                                           { Return minutes }
     MOV AL, CH;                                      { Transfer register }
     LES DI, Hour;                                    { ES:DI -> hours }
     STOSW;                                           { Return hours }
   END;
   {$ENDIF}
   {$IFDEF OS_GO32}                                   { FPC COMPATABLE ASM }
   BEGIN
   (* ASM
     MOVW $0x2C00, %AX;                               { Set function id }
     PUSHL %EBP;                                      { Save register }
     INT $0x21;                                       { System get time }
     POPL %EBP;                                       { Restore register }
     XORB %AH, %AH;                                   { Clear register }
     MOVB %DL, %AL;                                   { Transfer register }
     MOVL Sec100, %EDI;                               { EDI -> Sec100 }
     MOVW %AX, (%EDI);                                { Return Sec100 }
     MOVB %DH, %AL;                                   { Transfer register }
     MOVL Second, %EDI;                               { EDI -> Second }
     MOVW %AX, (%EDI);                                { Return Second }
     MOVB %CL, %AL;                                   { Transfer register }
     MOVL Minute, %EDI;                               { EDI -> Minute }
     MOVW %AX, (%EDI);                                { Return minute }
     MOVB %CH, %AL;                                   { Transfer register }
     MOVL Hour, %EDI;                                 { EDI -> Hour }
     MOVW %AX, (%EDI);                                { Return hour }
   END; *)
   { direct call of real interrupt seems to render the system
     unstable on Win2000 because some registers are not properly
     restored if a mouse interrupt is generated while the Dos
     interrupt is called... PM }
     Dos.GetTime(Hour,Minute,Second,Sec100);
   END;
   {$ENDIF}
{$ENDIF}
{$IFDEF OS_WINDOWS}                                   { WIN/NT CODE }
   {$IFDEF BIT_16}                                    { 16 BIT WINDOWS CODE }
   ASSEMBLER;
   ASM
     MOV AX, $2C00;                                   { Set function id }
     PUSH BP;                                         { Safety save register }
     INT $21;                                         { System get time }
     POP BP;                                          { Restore register }
     XOR AH, AH;                                      { Clear register }
     CLD;                                             { Strings go forward }
     MOV AL, DL;                                      { Transfer register }
     LES DI, Sec100;                                  { ES:DI -> hundredths }
     STOSW;                                           { Return hundredths }
     MOV AL, DH;                                      { Transfer register }
     LES DI, Second;                                  { ES:DI -> seconds }
     STOSW;                                           { Return seconds }
     MOV AL, CL;                                      { Transfer register }
     LES DI, Minute;                                  { ES:DI -> minutes }
     STOSW;                                           { Return minutes }
     MOV AL, CH;                                      { Transfer register }
     LES DI, Hour;                                    { ES:DI -> hours }
     STOSW;                                           { Return hours }
   END;
   {$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
   {$IFDEF BIT_32_OR_MORE}                            { 32 BIT WINDOWS CODE }
=======
   {$IFDEF BIT_32}                                    { 32 BIT WINDOWS CODE }
>>>>>>> graemeg/fixes_2_2
=======
   {$IFDEF BIT_32}                                    { 32 BIT WINDOWS CODE }
>>>>>>> origin/fixes_2_2
   VAR DT: TSystemTime;
   BEGIN
     {$IFDEF PPC_FPC}                                 { FPC WINDOWS COMPILER }
     GetLocalTime(@DT);                              { Get the date/time }
     {$ELSE}                                          { OTHER COMPILERS }
     GetLocalTime(DT);                               { Get the date/time }
     {$ENDIF}
     Hour := DT.wHour;                                { Transfer hour }
     Minute := DT.wMinute;                            { Transfer minute }
     Second := DT.wSecond;                            { Transfer seconds }
     Sec100 := DT.wMilliseconds DIV 10;               { Transfer hundredths }
   END;
   {$ENDIF}
{$ENDIF}
{$IFDEF OS_OS2}                                       { OS2 CODE }
VAR DT: DateTime;
BEGIN
   DosGetDateTime(DT);                                { Get the date/time }
   Hour   := DT.Hours;                                { Transfer hour }
   Minute := DT.Minutes;                              { Transfer minute }
   Second := DT.Seconds;                              { Transfer seconds }
   Sec100 := DT.Hundredths;                           { Transfer hundredths }
END;
{$ENDIF}
{$ifdef OS_UNIX}
BEGIN
  Dos.GetTime(Hour,Minute,Second,Sec100);
END;
{$endif OS_UNIX}
{$IFDEF OS_NETWARE}
BEGIN
  Dos.GetTime(Hour,Minute,Second,Sec100);
END;
{$ENDIF OS_NETWARE}
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF OS_AMIGA}
BEGIN
  Dos.GetTime(Hour,Minute,Second,Sec100);
END;
{$ENDIF OS_AMIGA}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{---------------------------------------------------------------------------}
{  MinutesToTime -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19Jun97 LdB     }
{---------------------------------------------------------------------------}
PROCEDURE MinutesToTime (Md: LongInt; Var Hour24, Minute: Word);
BEGIN
   Hour24 := Md DIV 60;                               { Hours of time }
   Minute := Md MOD 60;                               { Minutes of time }
END;

{---------------------------------------------------------------------------}
{  SecondsToTime -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 19Jun97 LdB     }
{---------------------------------------------------------------------------}
PROCEDURE SecondsToTime (Sd: LongInt; Var Hour24, Minute, Second: Word);
BEGIN
   Hour24 := Sd DIV 3600;                             { Hours of time }
   Minute := Sd MOD 3600 DIV 60;                      { Minutes of time }
   Second := Sd MOD 60;                               { Seconds of time }
END;

END.
