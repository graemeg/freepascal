{
    *********************************************************************
    Copyright (C) 1997, 1998 Gertjan Schouten

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

    System Utilities For Free Pascal
}


type
  PDayTable = ^TDayTable;
  TDayTable = array[1..12] of Word;
  TFormatDateTimeOption = (fdoInterval);
  TFormatDateTimeOptions =  set of TFormatDateTimeOption;
  
const
   HoursPerDay = 24;
   MinsPerHour = 60;
   SecsPerMin  = 60;
   MSecsPerSec = 1000;

   MinsPerDay  = HoursPerDay * MinsPerHour;
   SecsPerHour = SecsPerMin * MinsPerHour;
   SecsPerDay  = MinsPerDay * SecsPerMin;
   MSecsPerDay = SecsPerDay * MSecsPerSec;

{TDateTime holds the date as the number of days since 30 Dec 1899, known as
Microsoft Excel epoch}
   JulianEpoch = TDateTime(-2415018.5);
   UnixEpoch = JulianEpoch + TDateTime(2440587.5);

   DateDelta = 693594;        // Days between 1/1/0001 and 12/31/1899
   UnixDateDelta = Trunc(UnixEpoch); //25569


   { True=Leapyear }
   MonthDays: array [Boolean] of TDayTable =
     ((31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31),
      (31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31));

var
   TwoDigitYearCenturyWindow : word absolute DefaultFormatSettings.TwoDigitYearCenturyWindow;
                             { Threshold to be subtracted from year before
                               age-detection.}

   {  date time formatting characters:
      c      : shortdateformat + ' ' + longtimeformat
      d      : day of month
      dd     : day of month (leading zero)
      ddd    : day of week (abbreviation)
      dddd   : day of week (full)
      ddddd  : shortdateformat
      dddddd : longdateformat
      m      : month. If preceded by h or hh specifiers, then minutes.
      mm     : month. If preceded by h or hh specifiers, then minutes (leading zero).
      mmm    : month (abbreviation)
      mmmm   : month (full)
      y      : year (two digits)
      yy     : year (two digits)
      yyyy   : year (four digits, with century)
      h      : hour
      hh     : hour (leading zero)
      [h]    : hours including the hours of the full days, also: [hh]
               Only available if fdoInterval is specified in options argument to FormatDateTime.
      n      : minute
      nn     : minute (leading zero)
      [n]    : minutes including the minutes of the full hours and days,
               also: [nn] (or [m] or [mm] if in time part of the format string)
               Only available if fdoInterval is specified in options argument to FormatDateTime.
      s      : second
      ss     : second (leading zero)
      [s]    : seconds (including the seconds of the full minutes, hours and days.
               also: [ss]. 
               Only available if fdoInterval is specified in options argument to FormatDateTime.
      t      : shorttimeformat
      tt     : longtimeformat
      am/pm  : use 12 hour clock and display am and pm accordingly
      a/p    : use 12 hour clock and display a and p accordingly
      /      : insert date seperator
      :      : insert time seperator
      "xx"   : literal text
      'xx'   : literal text
   }

type
{ windows isn't defined in 2.0.2 (FK) }
{$if defined(win32) or defined(win64) or defined (wince)}
   { Win32/Win64 reuses the struct from the Windows unit }
{$DEFINE HAS_SYSTEMTIME}
{$endif win32 or win64 or wince }

{$IFNDEF HAS_SYSTEMTIME}
   TSystemTime = record
      Year, Month, Day, DayOfWeek: word;
      Hour, Minute, Second, MilliSecond: word;
   end ;
{$ENDIF}

   TTimeStamp = record
      Time: longint;   { Number of milliseconds since midnight }
      Date: longint;   { One plus number of days since 1/1/0001 }
   end ;


function DateTimeToTimeStamp(DateTime: TDateTime): TTimeStamp;
function TimeStampToDateTime(const TimeStamp: TTimeStamp): TDateTime;
function MSecsToTimeStamp(MSecs: Comp): TTimeStamp;
function TimeStampToMSecs(const TimeStamp: TTimeStamp): comp;
function TryEncodeDate(Year, Month, Day: Word; out Date: TDateTime): Boolean;
function TryEncodeTime(Hour, Min, Sec, MSec: Word; out Time: TDateTime): Boolean;
function EncodeDate(Year, Month, Day :word): TDateTime;
function EncodeTime(Hour, Minute, Second, MilliSecond:word): TDateTime;
function ComposeDateTime(Date,Time : TDateTime) : TDateTime;
procedure DecodeDate(Date: TDateTime; out Year, Month, Day: word);
function DecodeDateFully(const DateTime: TDateTime; out Year, Month, Day, DOW: Word): Boolean;
procedure DecodeTime(Time: TDateTime; out Hour, Minute, Second, MilliSecond: word);
procedure DateTimeToSystemTime(DateTime: TDateTime; out SystemTime: TSystemTime);
function SystemTimeToDateTime(const SystemTime: TSystemTime): TDateTime;
function DayOfWeek(DateTime: TDateTime): integer;
function Date: TDateTime;
function Time: TDateTime;
function Now: TDateTime;
function IncMonth(const DateTime: TDateTime; NumberOfMonths: integer = 1 ): TDateTime;
procedure IncAMonth(var Year, Month, Day: Word; NumberOfMonths: Integer = 1);
function IsLeapYear(Year: Word): boolean;
function DateToStr(Date: TDateTime): string;
function DateToStr(Date: TDateTime; const FormatSettings: TFormatSettings): string;
function TimeToStr(Time: TDateTime): string;
function TimeToStr(Time: TDateTime; const FormatSettings: TFormatSettings): string;
function DateTimeToStr(DateTime: TDateTime; ForceTimeIfZero : Boolean = False): string;
function DateTimeToStr(DateTime: TDateTime; const FormatSettings: TFormatSettings; ForceTimeIfZero : Boolean = False): string;
function StrToDate(const S: ShortString): TDateTime;                  {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDate(const S: Ansistring): TDateTime;                   {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDate(const S: ShortString; separator : char): TDateTime;{$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDate(const S: AnsiString; separator : char): TDateTime; {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDate(const S: string; FormatSettings : TFormatSettings): TDateTime;
function StrToTime(const S: Shortstring): TDateTime;                  {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTime(const S: Ansistring): TDateTime;                   {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTime(const S: ShortString; separator : char): TDateTime;{$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTime(const S: AnsiString; separator : char): TDateTime; {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTime(const S: string; FormatSettings : TFormatSettings): TDateTime;
function StrToDate(const S: ShortString; const useformat : string; separator : char = #0): TDateTime;{$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDate(const S: AnsiString; const useformat : string; separator : char = #0): TDateTime;{$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTime(const S: PChar; Len : integer; separator : char = #0): TDateTime;
function StrToDate(const S: PChar; Len : integer; const useformat : string; separator : char = #0): TDateTime;
function StrToDateTime(const S: AnsiString): TDateTime;
function StrToDateTime(const s: ShortString; const FormatSettings : TFormatSettings): TDateTime;
function StrToDateTime(const s: AnsiString; const FormatSettings : TFormatSettings): TDateTime;
function FormatDateTime(const FormatStr: string; DateTime: TDateTime; Options : TFormatDateTimeOptions = []): string;
function FormatDateTime(const FormatStr: string; DateTime: TDateTime; const FormatSettings: TFormatSettings; Options : TFormatDateTimeOptions = []): string;
procedure DateTimeToString(out Result: string; const FormatStr: string; const DateTime: TDateTime; Options : TFormatDateTimeOptions = []);
procedure DateTimeToString(out Result: string; const FormatStr: string; const DateTime: TDateTime; const FormatSettings: TFormatSettings; Options : TFormatDateTimeOptions = []);
Function DateTimeToFileDate(DateTime : TDateTime) : int64;
Function FileDateToDateTime (Filedate : Int64) :TDateTime;
function TryStrToDate(const S: ShortString; out Value: TDateTime): Boolean;                         {$ifdef SYSUTILSINLINE}inline;{$endif}
function TryStrToDate(const S: AnsiString; out Value: TDateTime): Boolean;                         {$ifdef SYSUTILSINLINE}inline;{$endif}
function TryStrToDate(const S: ShortString; out Value: TDateTime; separator : char): Boolean;
function TryStrToDate(const S: AnsiString; out Value: TDateTime; separator : char): Boolean;
function TryStrToTime(const S: ShortString; out Value: TDateTime): Boolean;                         {$ifdef SYSUTILSINLINE}inline;{$endif}
function TryStrToTime(const S: AnsiString; out Value: TDateTime): Boolean;                         {$ifdef SYSUTILSINLINE}inline;{$endif}
function TryStrToTime(const S: ShortString; out Value: TDateTime; separator : char): Boolean;
function TryStrToTime(const S: AnsiString; out Value: TDateTime; separator : char): Boolean;
function TryStrToDate(const S: ShortString; out Value: TDateTime;
                        const useformat : string; separator : char = #0): Boolean;
function TryStrToDate(const S: AnsiString; out Value: TDateTime;
                        const useformat : string; separator : char = #0): Boolean;
function TryStrToDateTime(const S: ShortString; out Value: TDateTime): Boolean;
function TryStrToDateTime(const S: AnsiString; out Value: TDateTime): Boolean;

function TryStrToDate(const S: string; out Value: TDateTime; const FormatSettings: TFormatSettings): Boolean;
function TryStrToTime(const S: string; out Value: TDateTime; const FormatSettings: TFormatSettings): Boolean;
function TryStrToDateTime(const S: string; out Value: TDateTime; const FormatSettings: TFormatSettings): Boolean;

function StrToDateDef(const S: ShortString; const Defvalue : TDateTime): TDateTime;                   {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDateDef(const S: ShortString; const Defvalue : TDateTime; separator : char): TDateTime; {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTimeDef(const S: ShortString; const Defvalue : TDateTime): TDateTime;                   {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTimeDef(const S: ShortString; const Defvalue : TDateTime; separator : char): TDateTime; {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDateTimeDef(const S: ShortString; const Defvalue : TDateTime): TDateTime;               {$ifdef SYSUTILSINLINE}inline;{$endif}

function StrToDateDef(const S: AnsiString; const Defvalue : TDateTime): TDateTime;                   {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDateDef(const S: AnsiString; const Defvalue : TDateTime; separator : char): TDateTime; {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTimeDef(const S: AnsiString; const Defvalue : TDateTime): TDateTime;                   {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToTimeDef(const S: AnsiString; const Defvalue : TDateTime; separator : char): TDateTime; {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDateTimeDef(const S: AnsiString; const Defvalue : TDateTime): TDateTime;               {$ifdef SYSUTILSINLINE}inline;{$endif}
function StrToDateTimeDef(const S: AnsiString; const Defvalue : TDateTime; const FormatSettings: TFormatSettings): TDateTime;               {$ifdef SYSUTILSINLINE}inline;{$endif}

function CurrentYear:Word;
{ FPC Extra }
Procedure GetLocalTime(var SystemTime: TSystemTime);

procedure ReplaceTime(var dati:TDateTime; NewTime : TDateTime); inline;
procedure ReplaceDate(var DateTime: TDateTime; const NewDate: TDateTime); inline;
function GetLocalTimeOffset: Integer;
