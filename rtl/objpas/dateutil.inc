{$mode objfpc}
{$h+}
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Delphi/Kylix compatibility unit, provides Date/Time handling routines.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit dateutils;

interface

{$ifndef FPUNONE}
uses
  SysUtils, Math;

{ ---------------------------------------------------------------------
    Various constants
  ---------------------------------------------------------------------}

const
  DaysPerWeek        = 7;
  WeeksPerFortnight  = 2;
  MonthsPerYear      = 12;
  YearsPerDecade     = 10;
  YearsPerCentury    = 100;
  YearsPerMillennium = 1000;

  // ISO day numbers.
  DayMonday    = 1;
  DayTuesday   = 2;
  DayWednesday = 3;
  DayThursday  = 4;
  DayFriday    = 5;
  DaySaturday  = 6;
  DaySunday    = 7;

  // Fraction of a day
  OneHour        = 1/HoursPerDay;
  OneMinute      = 1/MinsPerDay;
  OneSecond      = 1/SecsPerDay;
  OneMillisecond = 1/MSecsPerDay;

  { This is actual days per year but you need to know if it's a leap year}
  DaysPerYear: array [Boolean] of Word = (365, 366);

  { Used in RecodeDate, RecodeTime and RecodeDateTime for those datetime }
  {  fields you want to leave alone }
  RecodeLeaveFieldAsIs = High(Word);

{ ---------------------------------------------------------------------
    Global variables used in this unit
  ---------------------------------------------------------------------}

Const

  { Average over a 4 year span. Valid for next 100 years }
  ApproxDaysPerMonth: Double = 30.4375;
  ApproxDaysPerYear: Double  = 365.25;



{ ---------------------------------------------------------------------
    Simple trimming functions.
  ---------------------------------------------------------------------}

Function DateOf(const AValue: TDateTime): TDateTime;
Function TimeOf(const AValue: TDateTime): TDateTime;

{ ---------------------------------------------------------------------
    Identification functions.
  ---------------------------------------------------------------------}

Function IsInLeapYear(const AValue: TDateTime): Boolean;
Function IsPM(const AValue: TDateTime): Boolean;
Function IsValidDate(const AYear, AMonth, ADay: Word): Boolean;
Function IsValidTime(const AHour, AMinute, ASecond, AMilliSecond: Word): Boolean;
Function IsValidDateTime(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word): Boolean;
Function IsValidDateDay(const AYear, ADayOfYear: Word): Boolean;
Function IsValidDateWeek(const AYear, AWeekOfYear, ADayOfWeek: Word): Boolean;
Function IsValidDateMonthWeek(const AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word): Boolean;

{ ---------------------------------------------------------------------
    Enumeration functions.
  ---------------------------------------------------------------------}

Function WeeksInYear(const AValue: TDateTime): Word;
Function WeeksInAYear(const AYear: Word): Word;
Function DaysInYear(const AValue: TDateTime): Word;
Function DaysInAYear(const AYear: Word): Word;
Function DaysInMonth(const AValue: TDateTime): Word;
Function DaysInAMonth(const AYear, AMonth: Word): Word;


{ ---------------------------------------------------------------------
    Variations on current date/time.
  ---------------------------------------------------------------------}


Function Today: TDateTime;
Function Yesterday: TDateTime;
Function Tomorrow: TDateTime;
Function IsToday(const AValue: TDateTime): Boolean;
Function IsSameDay(const AValue, ABasis: TDateTime): Boolean;
Function PreviousDayOfWeek (DayOfWeek : Word) : Word;

{ ---------------------------------------------------------------------
    Extraction functions.
  ---------------------------------------------------------------------}

Function YearOf(const AValue: TDateTime): Word;
Function MonthOf(const AValue: TDateTime): Word;
Function WeekOf(const AValue: TDateTime): Word;
Function DayOf(const AValue: TDateTime): Word;
Function HourOf(const AValue: TDateTime): Word;
Function MinuteOf(const AValue: TDateTime): Word;
Function SecondOf(const AValue: TDateTime): Word;
Function MilliSecondOf(const AValue: TDateTime): Word;

{ ---------------------------------------------------------------------
    Start/End of year functions.
  ---------------------------------------------------------------------}

Function StartOfTheYear(const AValue: TDateTime): TDateTime;
Function EndOfTheYear(const AValue: TDateTime): TDateTime;
Function StartOfAYear(const AYear: Word): TDateTime;
Function EndOfAYear(const AYear: Word): TDateTime;

{ ---------------------------------------------------------------------
    Start/End of month functions.
  ---------------------------------------------------------------------}

Function StartOfTheMonth(const AValue: TDateTime): TDateTime;
Function EndOfTheMonth(const AValue: TDateTime): TDateTime;
Function StartOfAMonth(const AYear, AMonth: Word): TDateTime;
Function EndOfAMonth(const AYear, AMonth: Word): TDateTime;

{ ---------------------------------------------------------------------
    Start/End of week functions.
  ---------------------------------------------------------------------}


Function StartOfTheWeek(const AValue: TDateTime): TDateTime;
Function EndOfTheWeek(const AValue: TDateTime): TDateTime;
Function StartOfAWeek(const AYear, AWeekOfYear: Word; const ADayOfWeek: Word): TDateTime;
Function StartOfAWeek(const AYear, AWeekOfYear: Word): TDateTime; // ADayOFWeek 1
Function EndOfAWeek(const AYear, AWeekOfYear: Word; const ADayOfWeek: Word): TDateTime;
Function EndOfAWeek(const AYear, AWeekOfYear: Word): TDateTime; // const ADayOfWeek: Word = 7


{ ---------------------------------------------------------------------
    Start/End of day functions.
  ---------------------------------------------------------------------}

Function StartOfTheDay(const AValue: TDateTime): TDateTime;
Function EndOfTheDay(const AValue: TDateTime): TDateTime;
Function StartOfADay(const AYear, AMonth, ADay: Word): TDateTime; overload;
Function StartOfADay(const AYear, ADayOfYear: Word): TDateTime; overload;
Function EndOfADay(const AYear, AMonth, ADay: Word): TDateTime; overload;
Function EndOfADay(const AYear, ADayOfYear: Word): TDateTime; overload;

{ ---------------------------------------------------------------------
    Part of year functions.
  ---------------------------------------------------------------------}

Function MonthOfTheYear(const AValue: TDateTime): Word;
Function WeekOfTheYear(const AValue: TDateTime): Word; overload;
Function WeekOfTheYear(const AValue: TDateTime; out AYear: Word): Word; overload;
Function DayOfTheYear(const AValue: TDateTime): Word;
Function HourOfTheYear(const AValue: TDateTime): Word;
Function MinuteOfTheYear(const AValue: TDateTime): LongWord;
Function SecondOfTheYear(const AValue: TDateTime): LongWord;
Function MilliSecondOfTheYear(const AValue: TDateTime): Int64;

{ ---------------------------------------------------------------------
    Part of month functions.
  ---------------------------------------------------------------------}

Function WeekOfTheMonth(const AValue: TDateTime): Word; overload;
Function WeekOfTheMonth(const AValue: TDateTime; out AYear, AMonth: Word): Word; overload;
Function DayOfTheMonth(const AValue: TDateTime): Word;
Function HourOfTheMonth(const AValue: TDateTime): Word;
Function MinuteOfTheMonth(const AValue: TDateTime): Word;
Function SecondOfTheMonth(const AValue: TDateTime): LongWord;
Function MilliSecondOfTheMonth(const AValue: TDateTime): LongWord;

{ ---------------------------------------------------------------------
    Part of week functions.
  ---------------------------------------------------------------------}

Function DayOfTheWeek(const AValue: TDateTime): Word;
Function HourOfTheWeek(const AValue: TDateTime): Word;
Function MinuteOfTheWeek(const AValue: TDateTime): Word;
Function SecondOfTheWeek(const AValue: TDateTime): LongWord;
Function MilliSecondOfTheWeek(const AValue: TDateTime): LongWord;

{ ---------------------------------------------------------------------
    Part of day functions.
  ---------------------------------------------------------------------}

Function HourOfTheDay(const AValue: TDateTime): Word;
Function MinuteOfTheDay(const AValue: TDateTime): Word;
Function SecondOfTheDay(const AValue: TDateTime): LongWord;
Function MilliSecondOfTheDay(const AValue: TDateTime): LongWord;

{ ---------------------------------------------------------------------
    Part of hour functions.
  ---------------------------------------------------------------------}

Function MinuteOfTheHour(const AValue: TDateTime): Word;
Function SecondOfTheHour(const AValue: TDateTime): Word;
Function MilliSecondOfTheHour(const AValue: TDateTime): LongWord;

{ ---------------------------------------------------------------------
    Part of minute functions.
  ---------------------------------------------------------------------}


Function SecondOfTheMinute(const AValue: TDateTime): Word;
Function MilliSecondOfTheMinute(const AValue: TDateTime): LongWord;

{ ---------------------------------------------------------------------
    Part of second functions.
  ---------------------------------------------------------------------}

Function MilliSecondOfTheSecond(const AValue: TDateTime): Word;


{ ---------------------------------------------------------------------
    Range checking functions.
  ---------------------------------------------------------------------}

Function WithinPastYears(const ANow, AThen: TDateTime; const AYears: Integer): Boolean;
Function WithinPastMonths(const ANow, AThen: TDateTime; const AMonths: Integer): Boolean;
Function WithinPastWeeks(const ANow, AThen: TDateTime; const AWeeks: Integer): Boolean;
Function WithinPastDays(const ANow, AThen: TDateTime; const ADays: Integer): Boolean;
Function WithinPastHours(const ANow, AThen: TDateTime; const AHours: Int64): Boolean;
Function WithinPastMinutes(const ANow, AThen: TDateTime; const AMinutes: Int64): Boolean;
Function WithinPastSeconds(const ANow, AThen: TDateTime; const ASeconds: Int64): Boolean;
Function WithinPastMilliSeconds(const ANow, AThen: TDateTime; const AMilliSeconds: Int64): Boolean;

{ ---------------------------------------------------------------------
    Period functions.
  ---------------------------------------------------------------------}

Function YearsBetween(const ANow, AThen: TDateTime): Integer;
Function MonthsBetween(const ANow, AThen: TDateTime): Integer;
Function WeeksBetween(const ANow, AThen: TDateTime): Integer;
Function DaysBetween(const ANow, AThen: TDateTime): Integer;
Function HoursBetween(const ANow, AThen: TDateTime): Int64;
Function MinutesBetween(const ANow, AThen: TDateTime): Int64;
Function SecondsBetween(const ANow, AThen: TDateTime): Int64;
Function MilliSecondsBetween(const ANow, AThen: TDateTime): Int64;

{ ---------------------------------------------------------------------
    Timespan in xxx functions.
  ---------------------------------------------------------------------}

{ YearSpan and MonthSpan are approximate values }
Function YearSpan(const ANow, AThen: TDateTime): Double;
Function MonthSpan(const ANow, AThen: TDateTime): Double;
Function WeekSpan(const ANow, AThen: TDateTime): Double;
Function DaySpan(const ANow, AThen: TDateTime): Double;
Function HourSpan(const ANow, AThen: TDateTime): Double;
Function MinuteSpan(const ANow, AThen: TDateTime): Double;
Function SecondSpan(const ANow, AThen: TDateTime): Double;
Function MilliSecondSpan(const ANow, AThen: TDateTime): Double;

{ ---------------------------------------------------------------------
    Increment/decrement functions.
  ---------------------------------------------------------------------}

Function IncYear(const AValue: TDateTime; const ANumberOfYears: Integer ): TDateTime;
Function IncYear(const AValue: TDateTime): TDateTime; // ; const ANumberOfYears: Integer = 1)
// Function IncMonth is in SysUtils
Function IncWeek(const AValue: TDateTime; const ANumberOfWeeks: Integer): TDateTime;
Function IncWeek(const AValue: TDateTime): TDateTime; // ; const ANumberOfWeeks: Integer = 1)
Function IncDay(const AValue: TDateTime; const ANumberOfDays: Integer): TDateTime;
Function IncDay(const AValue: TDateTime): TDateTime; //; const ANumberOfDays: Integer = 1)
Function IncHour(const AValue: TDateTime; const ANumberOfHours: Int64): TDateTime;
Function IncHour(const AValue: TDateTime): TDateTime; //; const ANumberOfHours: Int64 = 1
Function IncMinute(const AValue: TDateTime; const ANumberOfMinutes: Int64): TDateTime;
Function IncMinute(const AValue: TDateTime): TDateTime; // ; const ANumberOfMinutes: Int64 = 1
Function IncSecond(const AValue: TDateTime; const ANumberOfSeconds: Int64): TDateTime;
Function IncSecond(const AValue: TDateTime): TDateTime; // ; const ANumberOfSeconds: Int64 = 1
Function IncMilliSecond(const AValue: TDateTime; const ANumberOfMilliSeconds: Int64): TDateTime;
Function IncMilliSecond(const AValue: TDateTime): TDateTime; // ; const ANumberOfMilliSeconds: Int64 = 1

{ ---------------------------------------------------------------------
    Encode/Decode of complete timestamp
  ---------------------------------------------------------------------}

Function EncodeDateTime(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word): TDateTime;
Procedure DecodeDateTime(const AValue: TDateTime; out AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word);
Function TryEncodeDateTime(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word; out AValue: TDateTime): Boolean;

{ ---------------------------------------------------------------------
    Encode/decode date, specifying week of year and day of week
  ---------------------------------------------------------------------}

Function EncodeDateWeek(const AYear, AWeekOfYear: Word; const ADayOfWeek: Word): TDateTime;
Function EncodeDateWeek(const AYear, AWeekOfYear: Word): TDateTime; //; const ADayOfWeek: Word = 1
Procedure DecodeDateWeek(const AValue: TDateTime; out AYear, AWeekOfYear, ADayOfWeek: Word);
Function TryEncodeDateWeek(const AYear, AWeekOfYear: Word; out AValue: TDateTime; const ADayOfWeek: Word): Boolean;
Function TryEncodeDateWeek(const AYear, AWeekOfYear: Word; out AValue: TDateTime): Boolean; //; const ADayOfWeek: Word = 1

{ ---------------------------------------------------------------------
    Encode/decode date, specifying day of year
  ---------------------------------------------------------------------}

Function EncodeDateDay(const AYear, ADayOfYear: Word): TDateTime;
Procedure DecodeDateDay(const AValue: TDateTime; out AYear, ADayOfYear: Word);
Function TryEncodeDateDay(const AYear, ADayOfYear: Word; out AValue: TDateTime): Boolean;

{ ---------------------------------------------------------------------
    Encode/decode date, specifying week of month
  ---------------------------------------------------------------------}

Function EncodeDateMonthWeek(const AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word): TDateTime;
Procedure DecodeDateMonthWeek(const AValue: TDateTime; out AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word);
Function TryEncodeDateMonthWeek(const AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word; out AValue: TDateTime): Boolean;

{ ---------------------------------------------------------------------
    Replace given element with supplied value.
  ---------------------------------------------------------------------}

Function RecodeYear(const AValue: TDateTime; const AYear: Word): TDateTime;
Function RecodeMonth(const AValue: TDateTime; const AMonth: Word): TDateTime;
Function RecodeDay(const AValue: TDateTime; const ADay: Word): TDateTime;
Function RecodeHour(const AValue: TDateTime; const AHour: Word): TDateTime;
Function RecodeMinute(const AValue: TDateTime; const AMinute: Word): TDateTime;
Function RecodeSecond(const AValue: TDateTime; const ASecond: Word): TDateTime;
Function RecodeMilliSecond(const AValue: TDateTime; const AMilliSecond: Word): TDateTime;
Function RecodeDate(const AValue: TDateTime; const AYear, AMonth, ADay: Word): TDateTime;
Function RecodeTime(const AValue: TDateTime; const AHour, AMinute, ASecond, AMilliSecond: Word): TDateTime;
Function RecodeDateTime(const AValue: TDateTime; const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word): TDateTime;
Function TryRecodeDateTime(const AValue: TDateTime; const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word; out AResult: TDateTime): Boolean;

{ ---------------------------------------------------------------------
    Comparision of date/time
  ---------------------------------------------------------------------}

Function CompareDateTime(const A, B: TDateTime): TValueRelationship;
Function CompareDate(const A, B: TDateTime): TValueRelationship;
Function CompareTime(const A, B: TDateTime): TValueRelationship;
Function SameDateTime(const A, B: TDateTime): Boolean;
Function SameDate(const A, B: TDateTime): Boolean;
Function SameTime(const A, B: TDateTime): Boolean;

{ For a given date these Functions tell you the which day of the week of the
  month (or year).  If its a Thursday, they will tell you if its the first,
  second, etc Thursday of the month (or year).  Remember, even though its
  the first Thursday of the year it doesn't mean its the first week of the
  year.  See ISO 8601 above for more information. }

Function NthDayOfWeek(const AValue: TDateTime): Word;

Procedure DecodeDayOfWeekInMonth(const AValue: TDateTime; out AYear, AMonth, ANthDayOfWeek, ADayOfWeek: Word);

Function EncodeDayOfWeekInMonth(const AYear, AMonth, ANthDayOfWeek,  ADayOfWeek: Word): TDateTime;
Function TryEncodeDayOfWeekInMonth(const AYear, AMonth, ANthDayOfWeek,  ADayOfWeek: Word; out AValue: TDateTime): Boolean;

{ ---------------------------------------------------------------------
    Exception throwing routines
  ---------------------------------------------------------------------}

Procedure InvalidDateTimeError(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word; const ABaseDate: TDateTime);
Procedure InvalidDateTimeError(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word); // const ABaseDate: TDateTime = 0
Procedure InvalidDateWeekError(const AYear, AWeekOfYear, ADayOfWeek: Word);
Procedure InvalidDateDayError(const AYear, ADayOfYear: Word);
Procedure InvalidDateMonthWeekError(const AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word);
Procedure InvalidDayOfWeekInMonthError(const AYear, AMonth, ANthDayOfWeek,  ADayOfWeek: Word);

{ ---------------------------------------------------------------------
    Julian and Modified Julian Date conversion support
  ---------------------------------------------------------------------}

Function DateTimeToJulianDate(const AValue: TDateTime): Double;
Function JulianDateToDateTime(const AValue: Double): TDateTime;
Function TryJulianDateToDateTime(const AValue: Double; out ADateTime: TDateTime): Boolean;

Function DateTimeToModifiedJulianDate(const AValue: TDateTime): Double;
Function ModifiedJulianDateToDateTime(const AValue: Double): TDateTime;
Function TryModifiedJulianDateToDateTime(const AValue: Double; out ADateTime: TDateTime): Boolean;

{ ---------------------------------------------------------------------
    Unix timestamp support.
  ---------------------------------------------------------------------}

Function DateTimeToUnix(const AValue: TDateTime): Int64;
Function UnixToDateTime(const AValue: Int64): TDateTime;
Function UnixTimeStampToMac(const AValue: Int64): Int64;

{ ---------------------------------------------------------------------
    Mac timestamp support.
  ---------------------------------------------------------------------}

Function DateTimeToMac(const AValue: TDateTime): Int64;
Function MacToDateTime(const AValue: Int64): TDateTime;
Function MacTimeStampToUnix(const AValue: Int64): Int64;

<<<<<<< HEAD
<<<<<<< HEAD
{ .....................................................................
    Dos <-> Delphi datetime support
  .....................................................................}

Function DateTimeToDosDateTime(const AValue: TDateTime): longint;
Function DosDateTimeToDateTime( AValue: longint): TDateTime;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ ScanDateTime is a limited inverse of formatdatetime }
function ScanDateTime(const Pattern:string;const s:string;const fmt:TFormatSettings;startpos:integer=1) : tdatetime; overload;
function ScanDateTime(const Pattern:string;const s:string;startpos:integer=1) : tdatetime; overload;

implementation

uses sysconst;

const
  TDateTimeEpsilon = 2.2204460493e-16;

{ ---------------------------------------------------------------------
    Auxiliary routines
  ---------------------------------------------------------------------}

Procedure NotYetImplemented (FN : String);

begin
  Raise Exception.CreateFmt('Function "%s" (dateutils) is not yet implemented',[FN]);
end;

{ ---------------------------------------------------------------------
    Simple trimming functions.
  ---------------------------------------------------------------------}

Function DateOf(const AValue: TDateTime): TDateTime; inline;
begin
  Result:=Trunc(AValue);
end;


Function TimeOf(const AValue: TDateTime): TDateTime; inline;
begin
  Result:=Frac(Avalue);
end;


{ ---------------------------------------------------------------------
    Identification functions.
  ---------------------------------------------------------------------}


Function IsInLeapYear(const AValue: TDateTime): Boolean;
begin
  Result:=IsLeapYear(YearOf(AValue));
end;


Function IsPM(const AValue: TDateTime): Boolean; inline;
begin
  Result:=(HourOf(AValue)>=12);
end;


Function IsValidDate(const AYear, AMonth, ADay: Word): Boolean;
begin
  Result:=(AYear<>0) and (AYear<10000)
          and (AMonth in [1..12])
          and (ADay<>0) and (ADay<=MonthDays[IsleapYear(AYear),AMonth]);
end;


Function IsValidTime(const AHour, AMinute, ASecond, AMilliSecond: Word): Boolean;
begin
  Result:=(AHour=HoursPerDay) and (AMinute=0) and (ASecond=0) and (AMillisecond=0);
  Result:=Result or
          ((AHour<HoursPerDay) and (AMinute<MinsPerHour) and (ASecond<SecsPerMin) and
           (AMillisecond<MSecsPerSec));
end;


Function IsValidDateTime(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word): Boolean;
begin
  Result:=IsValidDate(AYear,AMonth,ADay) and
          IsValidTime(AHour,AMinute,ASecond,AMillisecond)
end;


Function IsValidDateDay(const AYear, ADayOfYear: Word): Boolean;
begin
  Result:=(AYear<>0) and (ADayOfYear<>0) and (AYear<10000) and
          (ADayOfYear<=DaysPerYear[IsLeapYear(AYear)]);
end;


Function IsValidDateWeek(const AYear, AWeekOfYear, ADayOfWeek: Word): Boolean;
begin
  Result:=(AYear<>0) and (AYear<10000)
          and (ADayOfWeek in [1..7])
          and (AWeekOfYear<>0)
          and (AWeekOfYear<=WeeksInaYear(AYear));
  { should we not also check whether the day of the week is not
    larger than the last day of the last week in the year 9999 ?? }
end;


Function IsValidDateMonthWeek(const AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word): Boolean;

begin
  Result:=(AYear<>0) and (AYear<10000)
          and (AMonth in [1..12])
          and (AWeekOfMonth in [1..5])
          and (ADayOfWeek in [1..7]);
end;

{ ---------------------------------------------------------------------
    Enumeration functions.
  ---------------------------------------------------------------------}

Function WeeksInYear(const AValue: TDateTime): Word;
begin
  Result:=WeeksInAYear(YearOf(AValue));
end;


Function WeeksInAYear(const AYear: Word): Word;

Var
  DOW : Word;

begin
  Result:=52;
  DOW:=DayOfTheWeek(StartOfAYear(AYear));
  If (DOW=4) or ((DOW=3) and IsLeapYear(AYear)) then
    Inc(Result);
end;


Function DaysInYear(const AValue: TDateTime): Word;
begin
  Result:=DaysPerYear[IsLeapYear(YearOf(AValue))];
end;


Function DaysInAYear(const AYear: Word): Word;
begin
  Result:=DaysPerYear[Isleapyear(AYear)];
end;


Function DaysInMonth(const AValue: TDateTime): Word;

Var
  Y,M,D : Word;

begin
  Decodedate(AValue,Y,M,D);
  Result:=MonthDays[IsLeapYear(Y),M];
end;


Function DaysInAMonth(const AYear, AMonth: Word): Word;
begin
  Result:=MonthDays[IsLeapYear(AYear),AMonth];
end;


{ ---------------------------------------------------------------------
    Variations on current date/time.
  ---------------------------------------------------------------------}


Function Today: TDateTime; inline;
begin
  Result:=Date();
end;


Function Yesterday: TDateTime;
begin
  Result:=Date()-1;
end;


Function Tomorrow: TDateTime; 
begin
  Result:=Date()+1;
end;


Function IsToday(const AValue: TDateTime): Boolean;
begin
  Result:=IsSameDay(AValue,Date());
end;


Function IsSameDay(const AValue, ABasis: TDateTime): Boolean;

Var
  D : TDateTime;

begin
  D:=AValue-Trunc(ABasis);
  Result:=(D>=0) and (D<1);
end;

const
  DOWMap: array [1..7] of Word = (7, 1, 2, 3, 4, 5, 6);

Function PreviousDayOfWeek (DayOfWeek : Word) : Word;

begin
  If Not (DayOfWeek in [1..7]) then
    Raise EConvertError.CreateFmt(SErrInvalidDayOfWeek,[DayOfWeek]);
  Result:=DOWMap[DayOfWeek];
end;



{ ---------------------------------------------------------------------
    Extraction functions.
  ---------------------------------------------------------------------}


Function YearOf(const AValue: TDateTime): Word;

Var
  D,M : Word;

begin
  DecodeDate(AValue,Result,D,M);
end;


Function MonthOf(const AValue: TDateTime): Word;

Var
  Y,D : Word;

begin
  DecodeDate(AValue,Y,Result,D);
end;


Function WeekOf(const AValue: TDateTime): Word; inline;
begin
  Result:=WeekOfTheYear(AValue);
end;


Function DayOf(const AValue: TDateTime): Word;

Var
  Y,M : Word;

begin
  DecodeDate(AValue,Y,M,Result);
end;


Function HourOf(const AValue: TDateTime): Word;

Var
  N,S,MS : Word;

begin
  DecodeTime(AValue,Result,N,S,MS);
end;


Function MinuteOf(const AValue: TDateTime): Word;

Var
  H,S,MS : Word;

begin
  DecodeTime(AValue,H,Result,S,MS);
end;


Function SecondOf(const AValue: TDateTime): Word;

Var
  H,N,MS : Word;

begin
  DecodeTime(AValue,H,N,Result,MS);
end;


Function MilliSecondOf(const AValue: TDateTime): Word;

Var
  H,N,S : Word;

begin
  DecodeTime(AValue,H,N,S,Result);
end;


{ ---------------------------------------------------------------------
    Start/End of year functions.
  ---------------------------------------------------------------------}


Function StartOfTheYear(const AValue: TDateTime): TDateTime;
begin
  Result:=EncodeDate(YearOf(AValue),1,1);
end;


Function EndOfTheYear(const AValue: TDateTime): TDateTime;
begin
  Result:=EncodeDateTime(YearOf(AValue),12,31,23,59,59,999);
end;


Function StartOfAYear(const AYear: Word): TDateTime;
begin
  Result:=EncodeDate(AYear,1,1);
end;


Function EndOfAYear(const AYear: Word): TDateTime;

begin
  Result:=(EncodeDateTime(AYear,12,31,23,59,59,999));
end;

{ ---------------------------------------------------------------------
    Start/End of month functions.
  ---------------------------------------------------------------------}

Function StartOfTheMonth(const AValue: TDateTime): TDateTime;

Var
  Y,M,D : Word;

begin
  DecodeDate(AValue,Y,M,D);
  Result:=EncodeDate(Y,M,1);
//  MonthDays[IsLeapYear(Y),M])
end;


Function EndOfTheMonth(const AValue: TDateTime): TDateTime;

Var
  Y,M,D : Word;

begin
  DecodeDate(AValue,Y,M,D);
  Result:=EncodeDateTime(Y,M,MonthDays[IsLeapYear(Y),M],23,59,59,999);
end;


Function StartOfAMonth(const AYear, AMonth: Word): TDateTime; inline;
begin
  Result:=EncodeDate(AYear,AMonth,1);
end;


Function EndOfAMonth(const AYear, AMonth: Word): TDateTime;

begin
  Result:=EncodeDateTime(AYear,AMonth,MonthDays[IsLeapYear(AYear),AMonth],23,59,59,999);
end;


{ ---------------------------------------------------------------------
    Start/End of week functions.
  ---------------------------------------------------------------------}


Function StartOfTheWeek(const AValue: TDateTime): TDateTime;
begin
  Result:=Trunc(AValue)-DayOfTheWeek(AValue)+1;
end;


Function EndOfTheWeek(const AValue: TDateTime): TDateTime;
begin
  Result:=EndOfTheDay(AValue-DayOfTheWeek(AValue)+7);
end;


Function StartOfAWeek(const AYear, AWeekOfYear: Word; const ADayOfWeek: Word): TDateTime;
begin
  Result:=EncodeDateWeek(AYear,AWeekOfYear,ADayOfWeek);
end;


Function StartOfAWeek(const AYear, AWeekOfYear: Word): TDateTime; inline; // ADayOFWeek 1
begin
  Result:=StartOfAWeek(AYear,AWeekOfYear,1)
end;


Function EndOfAWeek(const AYear, AWeekOfYear: Word; const ADayOfWeek: Word): TDateTime; inline;
begin
  Result := EndOfTheDay(EncodeDateWeek(AYear, AWeekOfYear, ADayOfWeek));
end;


Function EndOfAWeek(const AYear, AWeekOfYear: Word): TDateTime; // const ADayOfWeek: Word = 7


begin
  Result:=EndOfAWeek(AYear,AWeekOfYear,7);
end;

{ ---------------------------------------------------------------------
    Start/End of day functions.
  ---------------------------------------------------------------------}

Function StartOfTheDay(const AValue: TDateTime): TDateTime; inline;
begin
  StartOfTheDay:=Trunc(Avalue);
end;


Function EndOfTheDay(const AValue: TDateTime): TDateTime;

Var
  Y,M,D : Word;

begin
  DecodeDate(AValue,Y,M,D);
  Result:=EncodeDateTime(Y,M,D,23,59,59,999);
end;


Function StartOfADay(const AYear, AMonth, ADay: Word): TDateTime; inline;
begin
  Result:=EncodeDate(AYear,AMonth,ADay);
end;


Function StartOfADay(const AYear, ADayOfYear: Word): TDateTime;
begin
  Result:=StartOfAYear(AYear)+ADayOfYear-1;
end;


Function EndOfADay(const AYear, AMonth, ADay: Word): TDateTime; inline;
begin
  Result:=EndOfTheDay(EncodeDate(AYear,AMonth,ADay));
end;


Function EndOfADay(const AYear, ADayOfYear: Word): TDateTime;


begin
  Result:=StartOfAYear(AYear)+ADayOfYear-1+EncodeTime(23,59,59,999);
end;

{ ---------------------------------------------------------------------
    Part of year functions.
  ---------------------------------------------------------------------}


Function MonthOfTheYear(const AValue: TDateTime): Word; inline;
begin
  Result:=MonthOf(AValue);
end;


Function WeekOfTheYear(const AValue: TDateTime): Word;

Var
  Y,DOW : Word;

begin
  DecodeDateWeek(AValue,Y,Result,DOW)
end;


Function WeekOfTheYear(const AValue: TDateTime; out AYear: Word): Word;

Var
  DOW : Word;

begin
  DecodeDateWeek(AValue,AYear,Result,DOW);
end;


Function DayOfTheYear(const AValue: TDateTime): Word;
begin
  Result:=Trunc(AValue-StartOfTheYear(AValue)+1);
end;


Function HourOfTheYear(const AValue: TDateTime): Word;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=H+((DayOfTheYear(AValue)-1)*24);
end;


Function MinuteOfTheYear(const AValue: TDateTime): LongWord;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=M+(H+((DayOfTheYear(AValue)-1)*24))*60;
end;


Function SecondOfTheYear(const AValue: TDateTime): LongWord;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=(M+(H+((DayOfTheYear(AValue)-1)*24))*60)*60+S;
end;


Function MilliSecondOfTheYear(const AValue: TDateTime): Int64;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=((M+(H+((int64(DayOfTheYear(AValue))-1)*24))*60)*60+S)*1000+MS;
end;


{ ---------------------------------------------------------------------
    Part of month functions.
  ---------------------------------------------------------------------}


Function WeekOfTheMonth(const AValue: TDateTime): Word;

var
  Y,M,DOW : word;

begin
  DecodeDateMonthWeek(AValue,Y,M,Result,DOW);
end;


Function WeekOfTheMonth(const AValue: TDateTime; out AYear, AMonth: Word): Word;

Var
  DOW : Word;

begin
  DecodeDateMonthWeek(AValue,AYear,AMonth,Result,DOW);
end;


Function DayOfTheMonth(const AValue: TDateTime): Word;

Var
  Y,M : Word;

begin
  DecodeDate(AValue,Y,M,Result);
end;


Function HourOfTheMonth(const AValue: TDateTime): Word;

Var
  Y,M,D,H,N,S,MS : Word;

begin
  DecodeDateTime(AValue,Y,M,D,H,N,S,MS);
  Result:=(D-1)*24+H;
end;


Function MinuteOfTheMonth(const AValue: TDateTime): Word;

Var
  Y,M,D,H,N,S,MS : Word;

begin
  DecodeDateTime(AValue,Y,M,D,H,N,S,MS);
  Result:=((D-1)*24+H)*60+N;
end;


Function SecondOfTheMonth(const AValue: TDateTime): LongWord;

Var
  Y,M,D,H,N,S,MS : Word;

begin
  DecodeDateTime(AValue,Y,M,D,H,N,S,MS);
  Result:=(((D-1)*24+H)*60+N)*60+S;
end;


Function MilliSecondOfTheMonth(const AValue: TDateTime): LongWord;

Var
  Y,M,D,H,N,S,MS : Word;

begin
  DecodeDateTime(AValue,Y,M,D,H,N,S,MS);
  Result:=((((D-1)*24+H)*60+N)*60+S)*1000+MS;
end;

{ ---------------------------------------------------------------------
    Part of week functions.
  ---------------------------------------------------------------------}


Function DayOfTheWeek(const AValue: TDateTime): Word;

begin
  Result:=DowMAP[DayOfWeek(AValue)];
end;


Function HourOfTheWeek(const AValue: TDateTime): Word;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=(DayOfTheWeek(AValue)-1)*24+H;
end;


Function MinuteOfTheWeek(const AValue: TDateTime): Word;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=((DayOfTheWeek(AValue)-1)*24+H)*60+M;
end;


Function SecondOfTheWeek(const AValue: TDateTime): LongWord;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=(((DayOfTheWeek(AValue)-1)*24+H)*60+M)*60+S;
end;


Function MilliSecondOfTheWeek(const AValue: TDateTime): LongWord;


Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=((((DayOfTheWeek(AValue)-1)*24+H)*60+M)*60+S)*1000+MS;
end;

{ ---------------------------------------------------------------------
    Part of day functions.
  ---------------------------------------------------------------------}


Function HourOfTheDay(const AValue: TDateTime): Word; inline;
begin
  Result:=HourOf(AValue);
end;


Function MinuteOfTheDay(const AValue: TDateTime): Word;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=(H*60)+M;
end;


Function SecondOfTheDay(const AValue: TDateTime): LongWord;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=((H*60)+M)*60+S;
end;


Function MilliSecondOfTheDay(const AValue: TDateTime): LongWord;

Var
  H,M,S,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=(((H*60)+M)*60+S)*1000+MS;
end;

{ ---------------------------------------------------------------------
    Part of hour functions.
  ---------------------------------------------------------------------}


Function MinuteOfTheHour(const AValue: TDateTime): Word; inline;
begin
  Result:=MinuteOf(AValue);
end;


Function SecondOfTheHour(const AValue: TDateTime): Word;

Var
  H,S,M,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=M*60+S;
end;


Function MilliSecondOfTheHour(const AValue: TDateTime): LongWord;

Var
  H,S,M,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=(M*60+S)*1000+MS;
end;

{ ---------------------------------------------------------------------
    Part of minute functions.
  ---------------------------------------------------------------------}


Function SecondOfTheMinute(const AValue: TDateTime): Word; inline;
begin
  Result:=SecondOf(AValue);
end;


Function MilliSecondOfTheMinute(const AValue: TDateTime): LongWord;

Var
  H,S,M,MS : Word;

begin
  DecodeTime(AValue,H,M,S,MS);
  Result:=S*1000+MS;
end;

{ ---------------------------------------------------------------------
    Part of second functions.
  ---------------------------------------------------------------------}

Function MilliSecondOfTheSecond(const AValue: TDateTime): Word; inline;
begin
  Result:=MilliSecondOf(AValue);
end;

{ ---------------------------------------------------------------------
    Range checking functions.
  ---------------------------------------------------------------------}

Function WithinPastYears(const ANow, AThen: TDateTime; const AYears: Integer): Boolean; inline;
begin
  Result:=YearsBetween(ANow,AThen)<=AYears;
end;


Function WithinPastMonths(const ANow, AThen: TDateTime; const AMonths: Integer): Boolean; inline;
begin
  Result:=MonthsBetween(ANow,AThen)<=AMonths;
end;


Function WithinPastWeeks(const ANow, AThen: TDateTime; const AWeeks: Integer): Boolean; inline;
begin
  Result:=WeeksBetween(ANow,AThen)<=AWeeks;
end;


Function WithinPastDays(const ANow, AThen: TDateTime; const ADays: Integer): Boolean; inline;
begin
  Result:=DaysBetween(ANow,AThen)<=ADays;
end;


Function WithinPastHours(const ANow, AThen: TDateTime; const AHours: Int64): Boolean; inline;
begin
  Result:=HoursBetween(ANow,AThen)<=AHours;
end;


Function WithinPastMinutes(const ANow, AThen: TDateTime; const AMinutes: Int64): Boolean; inline;
begin
  Result:=MinutesBetween(ANow,AThen)<=AMinutes;
end;


Function WithinPastSeconds(const ANow, AThen: TDateTime; const ASeconds: Int64): Boolean; inline;
begin
  Result:=SecondsBetween(ANow,Athen)<=ASeconds;
end;


Function WithinPastMilliSeconds(const ANow, AThen: TDateTime; const AMilliSeconds: Int64): Boolean; inline;
begin
  Result:=MilliSecondsBetween(ANow,AThen)<=AMilliSeconds;
end;


{ ---------------------------------------------------------------------
    Period functions.
  ---------------------------------------------------------------------}

{
  These functions are declared as approximate by Borland.
  A bit strange, since it can be calculated exactly ?

  -- No, because you need rounding or truncating (JM)
}


Function DateTimeDiff(const ANow, AThen: TDateTime): TDateTime;
begin
  Result:= ANow - AThen;
  if (ANow>0) and (AThen<0) then
    Result:=Result-0.5
  else if (ANow<-1.0) and (AThen>-1.0) then
    Result:=Result+0.5;
end;


Function YearsBetween(const ANow, AThen: TDateTime): Integer;
begin
  Result:=Trunc((Abs(DateTimeDiff(ANow,AThen))+TDateTimeEpsilon)/ApproxDaysPerYear);
end;


Function MonthsBetween(const ANow, AThen: TDateTime): Integer;
begin
  Result:=Trunc((Abs(DateTimeDiff(ANow,AThen))+TDateTimeEpsilon)/ApproxDaysPerMonth);
end;


Function WeeksBetween(const ANow, AThen: TDateTime): Integer;
begin
  Result:=Trunc(Abs(DateTimeDiff(ANow,AThen))+TDateTimeEpsilon) div 7;
end;


Function DaysBetween(const ANow, AThen: TDateTime): Integer;
begin
  Result:=Trunc(Abs(DateTimeDiff(ANow,AThen))+TDateTimeEpsilon);
end;


Function HoursBetween(const ANow, AThen: TDateTime): Int64;
begin
  Result:=Trunc((Abs(DateTimeDiff(ANow,AThen))+TDateTimeEpsilon)*HoursPerDay);
end;


Function MinutesBetween(const ANow, AThen: TDateTime): Int64;
begin
  Result:=Trunc((Abs(DateTimeDiff(ANow,AThen))+TDateTimeEpsilon)*MinsPerDay);
end;


Function SecondsBetween(const ANow, AThen: TDateTime): Int64;
begin
  Result:=Trunc((Abs(DateTimeDiff(ANow,AThen))+TDateTimeEpsilon)*SecsPerDay);
end;


Function MilliSecondsBetween(const ANow, AThen: TDateTime): Int64;
begin
  Result:=Trunc((Abs(DateTimeDiff(ANow,AThen))+TDateTimeEpsilon)*MSecsPerDay);
end;


{ ---------------------------------------------------------------------
    Timespan in xxx functions.
  ---------------------------------------------------------------------}

Function YearSpan(const ANow, AThen: TDateTime): Double;
begin
  Result:=Abs(DateTimeDiff(ANow,AThen))/ApproxDaysPerYear;
end;


Function MonthSpan(const ANow, AThen: TDateTime): Double;
begin
  Result:=Abs(DateTimeDiff(ANow,AThen))/ApproxDaysPerMonth;
end;


Function WeekSpan(const ANow, AThen: TDateTime): Double;
begin
  Result:=Abs(DateTimeDiff(ANow,AThen)) / 7
end;


Function DaySpan(const ANow, AThen: TDateTime): Double;
begin
  Result:=Abs(DateTimeDiff(ANow,AThen));
end;


Function HourSpan(const ANow, AThen: TDateTime): Double;
begin
  Result:=Abs(DateTimeDiff(ANow,AThen))*HoursPerDay;
end;


Function MinuteSpan(const ANow, AThen: TDateTime): Double;
begin
  Result:=Abs(DateTimeDiff(ANow,AThen))*MinsPerDay;
end;


Function SecondSpan(const ANow, AThen: TDateTime): Double;
begin
  Result:=Abs(DateTimeDiff(ANow,AThen))*SecsPerDay;
end;


Function MilliSecondSpan(const ANow, AThen: TDateTime): Double;
begin
  Result:=Abs(DateTimeDiff(ANow,AThen))*MSecsPerDay;
end;


{ ---------------------------------------------------------------------
    Increment/decrement functions.
  ---------------------------------------------------------------------}

Procedure MaybeSkipTimeWarp(OldDate: TDateTime; var NewDate: TDateTime);
begin
  if (OldDate>0) and (NewDate<0) then
    NewDate:=NewDate-0.5
  else if (OldDate<-1.0) and (NewDate>-1.0) then
    NewDate:=NewDate+0.5;
end;


Function IncYear(const AValue: TDateTime; const ANumberOfYears: Integer ): TDateTime;

Var
  Y,M,D,H,N,S,MS : Word;
begin
  DecodeDateTime(AValue,Y,M,D,H,N,S,MS);
  Y:=Y+ANumberOfYears;
  If (M=2) and (D=29) And (Not IsLeapYear(Y)) then
    D:=28;
  Result:=EncodeDateTime(Y,M,D,H,N,S,MS);
end;


Function IncYear(const AValue: TDateTime): TDateTime; // ; const ANumberOfYears: Integer = 1)
begin
  Result:=IncYear(Avalue,1);
end;


Function IncWeek(const AValue: TDateTime; const ANumberOfWeeks: Integer): TDateTime;
begin
  Result:=AValue+ANumberOfWeeks*7;
  MaybeSkipTimeWarp(AValue,Result);
end;


Function IncWeek(const AValue: TDateTime): TDateTime; // ; const ANumberOfWeeks: Integer = 1)
begin
  Result:=IncWeek(Avalue,1);
end;


Function IncDay(const AValue: TDateTime; const ANumberOfDays: Integer): TDateTime;
begin
  Result:=AValue+ANumberOfDays;
  MaybeSkipTimeWarp(AValue,Result);
end;


Function IncDay(const AValue: TDateTime): TDateTime; //; const ANumberOfDays: Integer = 1)
begin
  Result:=IncDay(Avalue,1);
end;


Function IncHour(const AValue: TDateTime; const ANumberOfHours: Int64): TDateTime;
begin
  Result:=AValue+ANumberOfHours/HoursPerDay;
  MaybeSkipTimeWarp(AValue,Result);
end;


Function IncHour(const AValue: TDateTime): TDateTime; //; const ANumberOfHours: Int64 = 1
begin
  Result:=IncHour(AValue,1);
end;


Function IncMinute(const AValue: TDateTime; const ANumberOfMinutes: Int64): TDateTime;
begin
  Result:=AValue+ANumberOfMinutes / MinsPerDay;
  MaybeSkipTimeWarp(AValue,Result);
end;


Function IncMinute(const AValue: TDateTime): TDateTime; // ; const ANumberOfMinutes: Int64 = 1
begin
  Result:=IncMinute(AValue,1);
end;


Function IncSecond(const AValue: TDateTime; const ANumberOfSeconds: Int64): TDateTime;
begin
  Result:=AValue+ANumberOfSeconds / SecsPerDay;
  MaybeSkipTimeWarp(AValue,Result);
end;


Function IncSecond(const AValue: TDateTime): TDateTime; // ; const ANumberOfSeconds: Int64 = 1
begin
  Result:=IncSecond(Avalue,1);
end;


Function IncMilliSecond(const AValue: TDateTime; const ANumberOfMilliSeconds: Int64): TDateTime;
begin
  Result:=AValue+ANumberOfMilliSeconds/MSecsPerDay;
  MaybeSkipTimeWarp(AValue,Result);
end;


Function IncMilliSecond(const AValue: TDateTime): TDateTime; // ; const ANumberOfMilliSeconds: Int64 = 1
begin
  Result:=IncMilliSecond(AValue,1);
end;


{ ---------------------------------------------------------------------
    Encode/Decode of complete timestamp
  ---------------------------------------------------------------------}


Function EncodeDateTime(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word): TDateTime;
begin
  If Not TryEncodeDateTime(AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond,Result) then
    InvalidDateTimeError(AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond)
end;


Procedure DecodeDateTime(const AValue: TDateTime; out AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word);
begin
  DecodeTime(AValue,AHour,AMinute,ASecond,AMilliSecond);
  if AHour=24 then // can happen due rounding issues mantis 17123
    begin
      AHour:=0; // rest is already zero
      DecodeDate(round(AValue),AYear,AMonth,ADay);
    end
  else
    DecodeDate(AValue,AYear,AMonth,ADay);
end;


Function TryEncodeDateTime(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word; out AValue: TDateTime): Boolean;

Var
 tmp : TDateTime;

begin
  Result:=TryEncodeDate(AYear,AMonth,ADay,AValue);
  Result:=Result and TryEncodeTime(AHour,AMinute,ASecond,Amillisecond,Tmp);
  If Result then
    Avalue:=ComposeDateTime(AValue,Tmp);
end;

{ ---------------------------------------------------------------------
    Encode/decode date, specifying week of year and day of week
  ---------------------------------------------------------------------}

Function EncodeDateWeek(const AYear, AWeekOfYear: Word; const ADayOfWeek: Word): TDateTime;
begin
  If Not TryEncodeDateWeek(AYear,AWeekOfYear,Result,ADayOfWeek) then
    InvalidDateWeekError(AYear,AWeekOfYear,ADayOfWeek);
end;


Function EncodeDateWeek(const AYear, AWeekOfYear: Word): TDateTime; //; const ADayOfWeek: Word = 1
begin
  Result := EncodeDateWeek(AYear,AWeekOfYear,1);
end;


Procedure DecodeDateWeek(const AValue: TDateTime; out AYear, AWeekOfYear, ADayOfWeek: Word);

var
  DOY : Integer;
  D: Word;
  YS : TDateTime;
  YSDOW, YEDOW: Word;

begin
  AYear:=YearOf(AValue);
  // Correct to ISO DOW
  ADayOfWeek:=DayOfWeek(AValue)-1;
  If ADAyOfWeek=0 then
    ADayofweek:=7;
  YS:=StartOfAYear(AYear);
  DOY:=Trunc(AValue-YS)+1;
  YSDOW:=DayOfTheWeek(YS);
  // Correct week if later than wednesday. First week never starts later than wednesday
  if (YSDOW<5) then
    Inc(DOY,YSDOW-1)
  else
    Dec(DOY,8-YSDOW);
  if (DOY<=0) then // Day is in last week of previous year.
    DecodeDateWeek(YS-1,AYear,AWeekOfYear,D)
  else
    begin
    AWeekOfYear:=DOY div 7;
    if ((DOY mod 7)<>0) then
      Inc(AWeekOfYear);
    if (AWeekOfYear>52) then // Maybe in first week of next year ?
      begin
      YEDOW:=YSDOW;
      if IsLeapYear(AYear) then
        begin
        Inc(YEDOW);
        if (YEDOW>7) then
          YEDOW:=1
        else
      end;
      if (YEDOW<4) then // Really next year.
        begin
        Inc(AYear);
        AWeekOfYear:=1;
        end;
      end;
  end;
end;



Function TryEncodeDateWeek(const AYear, AWeekOfYear: Word; out AValue: TDateTime; const ADayOfWeek: Word): Boolean;

Var
  DOW : Word;
  Rest : Integer;

begin
  Result:=IsValidDateWeek(Ayear,AWeekOfYear,ADayOfWeek);
  If Result then
    begin
    AValue:=EncodeDate(AYear,1,1)+(7*(AWeekOfYear-1));
    DOW:=DayOfTheWeek(AValue);
    Rest:=ADayOfWeek-DOW;
    If (DOW>4) then
      Inc(Rest,7);
    AValue:=AValue+Rest;
    end;
end;


Function TryEncodeDateWeek(const AYear, AWeekOfYear: Word; out AValue: TDateTime): Boolean; //; const ADayOfWeek: Word = 1
begin
  Result:=TryEncodeDateWeek(AYear,AWeekOfYear,AValue,1);
end;

{ ---------------------------------------------------------------------
    Encode/decode date, specifying day of year
  ---------------------------------------------------------------------}

Function EncodeDateDay(const AYear, ADayOfYear: Word): TDateTime;
begin
  If Not TryEncodeDateDay(AYear,ADayOfYear,Result) then
    InvalidDateDayError(AYear,ADayOfYear);
end;


Procedure DecodeDateDay(const AValue: TDateTime; out AYear, ADayOfYear: Word);

Var
  M,D : Word;

begin
  DecodeDate(AValue,AYear,M,D);
  ADayOfyear:=Trunc(AValue-EncodeDate(AYear,1,1))+1;
end;


Function TryEncodeDateDay(const AYear, ADayOfYear: Word; out AValue: TDateTime): Boolean;
begin
  Result:=(ADayOfYear<>0) and (ADayOfYear<=DaysPerYear [IsleapYear(AYear)]);
  If Result then
    AValue:=EncodeDate(AYear,1,1)+ADayOfYear-1;
end;


{ ---------------------------------------------------------------------
    Encode/decode date, specifying week of month
  ---------------------------------------------------------------------}


Function EncodeDateMonthWeek(const AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word): TDateTime;
begin
  If Not TryEncodeDateMonthWeek(Ayear,AMonth,AWeekOfMonth,ADayOfWeek,Result) then
    InvalidDateMonthWeekError(AYear,AMonth,AWeekOfMonth,ADayOfWeek);
end;

Procedure DecodeDateMonthWeek(const AValue: TDateTime; out AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word);

Var
  D,SDOM,EDOM : Word;
  SOM : TdateTime;
  DOM : Integer;
begin
  DecodeDate(AValue,AYear,AMonth,D);
  ADayOfWeek:=DayOfTheWeek(AValue);
  SOM:=EncodeDate(Ayear,Amonth,1);
  SDOM:=DayOfTheWeek(SOM);
  DOM:=D-1+SDOM;
  If SDOM>4 then
    Dec(DOM,7);
  // Too early in the month. First full week is next week, day is after thursday.
  If DOM<=0 Then
    DecodeDateMonthWeek(SOM-1,AYear,AMonth,AWeekOfMonth,D)
  else
    begin
    AWeekOfMonth:=(DOM div 7)+Ord((DOM mod 7)<>0);
    EDOM:=DayOfTheWeek(EndOfAMonth(Ayear,AMonth));
    // In last days of last long week, so in next month...
    If (EDOM<4) and ((DaysInAMonth(AYear,Amonth)-D)<EDOM) then
      begin
      AWeekOfMonth:=1;
      Inc(AMonth);
      If (AMonth=13) then
        begin
        AMonth:=1;
        Inc(AYear);
        end;
      end;
    end;
end;

Function TryEncodeDateMonthWeek(const AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word; out AValue: TDateTime): Boolean;

var
  S : Word;
  DOM : Integer;

begin
  Result:=IsValidDateMonthWeek(AYear,AMonth,AWeekOfMonth,ADayOfWeek);
  if Result then
    begin
    AValue:=EncodeDate(AYear,AMonth,1);
    DOM:=(AWeekOfMonth-1)*7+ADayOfWeek-1;
    { Correct for first week in last month.}
    S:=DayOfTheWeek(AValue);
    Dec(DOM,S-1);
    if S in [DayFriday..DaySunday] then
      Inc(DOM,7);
    AValue:=AValue+DOM;
    end;
end;


{ ---------------------------------------------------------------------
    Replace given element with supplied value.
  ---------------------------------------------------------------------}

Const
  LFAI = RecodeLeaveFieldAsIS; // Less typing, readable code
{
  Note: We have little choice but to implement it like Borland did:
  If AValue contains some 'wrong' value, it will throw an error.
  To simulate this we'd have to check in each function whether
  both arguments are correct. To avoid it, all is routed through
  the 'central' RecodeDateTime function as in Borland's implementation.
}

Function RecodeYear(const AValue: TDateTime; const AYear: Word): TDateTime;

begin
  Result := RecodeDateTime(AValue,AYear,LFAI,LFAI,LFAI,LFAI,LFAI,LFAI);
end;


Function RecodeMonth(const AValue: TDateTime; const AMonth: Word): TDateTime;
begin
  Result := RecodeDateTime(AValue,LFAI,AMonth,LFAI,LFAI,LFAI,LFAI,LFAI);
end;


Function RecodeDay(const AValue: TDateTime; const ADay: Word): TDateTime;
begin
  Result := RecodeDateTime(AValue,LFAI,LFAI,ADay,LFAI,LFAI,LFAI,LFAI);
end;


Function RecodeHour(const AValue: TDateTime; const AHour: Word): TDateTime;
begin
  Result := RecodeDateTime(AValue,LFAI,LFAI,LFAI,AHour,LFAI,LFAI,LFAI);
end;


Function RecodeMinute(const AValue: TDateTime; const AMinute: Word): TDateTime;
begin
  Result := RecodeDateTime(AValue,LFAI,LFAI,LFAI,LFAI,AMinute,LFAI,LFAI);
end;


Function RecodeSecond(const AValue: TDateTime; const ASecond: Word): TDateTime;
begin
  Result := RecodeDateTime(AValue,LFAI,LFAI,LFAI,LFAI,LFAI,ASecond,LFAI);
end;


Function RecodeMilliSecond(const AValue: TDateTime; const AMilliSecond: Word): TDateTime;
begin
  Result := RecodeDateTime(AValue,LFAI,LFAI,LFAI,LFAI,LFAI,LFAI,AMilliSecond);
end;


Function RecodeDate(const AValue: TDateTime; const AYear, AMonth, ADay: Word): TDateTime;
begin
  Result := RecodeDateTime(AValue,AYear,AMonth,ADay,LFAI,LFAI,LFAI,LFAI);
end;


Function RecodeTime(const AValue: TDateTime; const AHour, AMinute, ASecond, AMilliSecond: Word): TDateTime;
begin
  Result := RecodeDateTime(AValue,LFAI,LFAI,LFAI,AHour,AMinute,ASecond,AMilliSecond);
end;


Function RecodeDateTime(const AValue: TDateTime; const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word): TDateTime;
begin
  If Not TryRecodeDateTime(AValue,AYear,AMonth,ADay,AHour,AMinute,ASecond,AMilliSecond,Result) then
    InvalidDateTimeError(AYear,AMonth,ADay,AHour,AMinute,ASecond,AMilliSecond,AValue);
end;


Function TryRecodeDateTime(const AValue: TDateTime; const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word; out AResult: TDateTime): Boolean;

  Procedure FV (Var AV : Word; Arg : Word);

  begin
    If (Arg<>LFAI) then
      AV:=Arg;
  end;

Var
  Y,M,D,H,N,S,MS : Word;

begin
  DecodeDateTime(AValue,Y,M,D,H,N,S,MS);
  FV(Y,AYear);
  FV(M,AMonth);
  FV(D,ADay);
  FV(H,AHour);
  FV(N,AMinute);
  FV(S,ASecond);
  FV(MS,AMillisecond);
  Result:=TryEncodeDateTime(Y,M,D,H,N,S,MS,AResult);
end;

{ ---------------------------------------------------------------------
    Comparision of date/time
  ---------------------------------------------------------------------}


Function CompareDateTime(const A, B: TDateTime): TValueRelationship;
begin
  If SameDateTime(A,B) then
    Result:=EqualsValue
  else If A>B then
    Result:=GreaterThanValue
  else
    Result:=LessThanValue
end;


Function CompareDate(const A, B: TDateTime): TValueRelationship;
begin
  If SameDate(A,B) then
    Result:=EQualsValue
  else if A<B then
    Result:=LessThanValue
  else
    Result:=GreaterThanValue;
end;


Function CompareTime(const A, B: TDateTime): TValueRelationship;

begin
  If SameTime(A,B) then
    Result:=EQualsValue
  else If Frac(A)<Frac(B) then
    Result:=LessThanValue
  else
    Result:=GreaterThanValue;
end;


Function SameDateTime(const A, B: TDateTime): Boolean;
begin
  Result:=Abs(A-B)<OneMilliSecond;
end;


Function SameDate(const A, B: TDateTime): Boolean; inline;
begin
  Result:=Trunc(A)=Trunc(B);
end;


Function SameTime(const A, B: TDateTime): Boolean;

begin
  Result:=Frac(Abs(A-B))<OneMilliSecond;
end;


Function InternalNthDayOfWeek(DoM : Word) : Word;

begin
  Result:=(Dom-1) div 7 +1;
end;

Function NthDayOfWeek(const AValue: TDateTime): Word;

begin
  Result:=InternalNthDayOfWeek(DayOfTheMonth(AValue));
end;


Procedure DecodeDayOfWeekInMonth(const AValue: TDateTime; out AYear, AMonth, ANthDayOfWeek, ADayOfWeek: Word);

var
  D: Word;

begin
  DecodeDate(AValue,AYear,AMonth,D);
  ADayOfWeek:=DayOfTheWeek(AValue);
  ANthDayOfWeek:=InternalNthDayOfWeek(D);
end;


Function EncodeDayOfWeekInMonth(const AYear, AMonth, ANthDayOfWeek,  ADayOfWeek: Word): TDateTime;
begin
  If Not TryEncodeDayOfWeekInMonth(AYear,AMonth,ANthDayOfWeek,ADayOfWeek,Result) then
    InvalidDayOfWeekInMonthError(AYear,AMonth,ANthDayOfWeek,ADayOfWeek);
end;


Function TryEncodeDayOfWeekInMonth(const AYear, AMonth, ANthDayOfWeek,  ADayOfWeek: Word; out AValue: TDateTime): Boolean;

Var
  SOM,D : Word;

begin
  SOM:=DayOfTheWeek(EncodeDate(Ayear,AMonth,1));
  D:=1+ADayOfWeek-SOM+7*(ANthDayOfWeek-1);
  If SOM>ADayOfWeek then
    D:=D+7; // Clearer would have been Inc(ANthDayOfweek) but it's a const
  Result:=TryEncodeDate(Ayear,AMonth,D,AValue);
end;

{ ---------------------------------------------------------------------
    Exception throwing routines
  ---------------------------------------------------------------------}



Procedure InvalidDateTimeError(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word; const ABaseDate: TDateTime);

  Function DoField(Arg,Def : Word; Unknown: String) : String;

  begin
    If (Arg<>LFAI) then
      Result:=Format('%.*d',[Length(Unknown),Arg])
    else if (ABaseDate=0) then
      Result:=Unknown
    else
      Result:=Format('%.*d',[Length(Unknown),Arg]);
  end;

Var
  Y,M,D,H,N,S,MS : Word;
  Msg : String;

begin
  DecodeDateTime(ABasedate,Y,M,D,H,N,S,MS);
  Msg:=DoField(AYear,Y,'????');
  Msg:=Msg+DefaultFormatSettings.DateSeparator+DoField(AMonth,M,'??');
  Msg:=Msg+DefaultFormatSettings.DateSeparator+DoField(ADay,D,'??');
  Msg:=Msg+' '+DoField(AHour,H,'??');
  Msg:=Msg+DefaultFormatSettings.TimeSeparator+DoField(AMinute,N,'??');
  Msg:=Msg+DefaultFormatSettings.TimeSeparator+Dofield(ASecond,S,'??');
  Msg:=Msg+DefaultFormatSettings.DecimalSeparator+DoField(AMilliSecond,MS,'???');
  Raise EConvertError.CreateFmt(SErrInvalidTimeStamp,[Msg]);
end;


Procedure InvalidDateTimeError(const AYear, AMonth, ADay, AHour, AMinute, ASecond, AMilliSecond: Word); // const ABaseDate: TDateTime = 0
begin
  InvalidDateTimeError(AYear,AMonth,ADay,AHour,AMinute,ASecond,AMilliSecond,0);
end;


Procedure InvalidDateWeekError(const AYear, AWeekOfYear, ADayOfWeek: Word);
begin
  Raise EConvertError.CreateFmt(SErrInvalidDateWeek,[AYear,AWeekOfYear,ADayOfWeek]);
end;


Procedure InvalidDateDayError(const AYear, ADayOfYear: Word);
begin
  Raise EConvertError.CreateFmt(SErrInvalidDayOfYear,[AYear,ADayOfYear]);
end;


Procedure InvalidDateMonthWeekError(const AYear, AMonth, AWeekOfMonth, ADayOfWeek: Word);
begin
  Raise EConvertError.CreateFmt(SErrInvalidDateMonthWeek,[Ayear,AMonth,AWeekOfMonth,ADayOfWeek]);
end;


Procedure InvalidDayOfWeekInMonthError(const AYear, AMonth, ANthDayOfWeek,  ADayOfWeek: Word);

begin
  Raise EConvertError.CreateFmt(SErrInvalidDayOfWeekInMonth,[AYear,AMonth,ANthDayOfWeek,ADayOfWeek]);
end;


{ ---------------------------------------------------------------------
    Julian and Modified Julian Date conversion support
  ---------------------------------------------------------------------}

{$ifopt R+}
{$define RangeCheckWasOn}
{$R-}
{$endif opt R+}

{$ifopt Q+}
{$define OverflowCheckWasOn}
{$Q-}
{$endif opt Q+}

Function DateTimeToJulianDate(const AValue: TDateTime): Double;
var
  day,month,year: word;
  a,y,m: integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  DecodeDate ( AValue, year, month, day );
  a := (14-month) div 12;
  y := year + 4800 - a;
  m := month + (12*a) - 3;
  result := day + ((153*m+2) div 5) + (365*y) + (y div 4) - (y div 100) + (y div 400) - 32045;
  result := result - 0.5;
=======
  DateTimeToJulianDate := AValue - JulianEpoch;
>>>>>>> graemeg/fixes_2_2
=======
  DateTimeToJulianDate := AValue - JulianEpoch;
>>>>>>> origin/fixes_2_2
end;


Function JulianDateToDateTime(const AValue: Double): TDateTime;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if not TryJulianDateToDateTime(AValue, Result) then
    raise EConvertError.CreateFmt(SInvalidJulianDate, [AValue]);
end;


Function TryJulianDateToDateTime(const AValue: Double; out ADateTime: TDateTime): Boolean;
var
  a,b,c,d,e,m:integer;
  day,month,year: word;
begin
  a := trunc(AValue + 32044.5);
  b := (4*a + 3) div 146097;
  c := a - (146097*b div 4);
  d := (4*c + 3) div 1461;
  e := c - (1461*d div 4);
  m := (5*e+2) div 153;
  day := e - ((153*m + 2) div 5) + 1;
  month := m + 3 - 12 *  ( m div 10 );
  year := (100*b) + d - 4800 + ( m div 10 );
  result := TryEncodeDate ( Year, Month, Day, ADateTime );
=======
  JulianDateToDateTime := AValue + JulianEpoch;
  if(AValue <= 0) or (AValue >= 10000)then
    JulianDateToDateTime := NaN;
end;


=======
  JulianDateToDateTime := AValue + JulianEpoch;
  if(AValue <= 0) or (AValue >= 10000)then
    JulianDateToDateTime := NaN;
end;


>>>>>>> origin/fixes_2_2
Function TryJulianDateToDateTime(const AValue: Double; var ADateTime: TDateTime): Boolean;
begin
  ADateTime := JulianDateToDateTime(AValue);
  TryJulianDateToDateTime := ADateTime <> NaN;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Function DateTimeToModifiedJulianDate(const AValue: TDateTime): Double;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  result := DateTimeToJulianDate(AValue) - 2400000.5;
=======
=======
>>>>>>> origin/fixes_2_2
  Result:=0;
  NotYetImplemented('DateTimeToModifiedJulianDate');
>>>>>>> graemeg/fixes_2_2
end;


Function ModifiedJulianDateToDateTime(const AValue: Double): TDateTime;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  result := JulianDateToDateTime(AValue + 2400000.5);
end;


Function TryModifiedJulianDateToDateTime(const AValue: Double; out ADateTime: TDateTime): Boolean;
begin
  Result:=TryJulianDateToDateTime(AValue + 2400000.5, ADateTime);
=======
=======
>>>>>>> origin/fixes_2_2
  Result:=0;
  NotYetImplemented('ModifiedJulianDateToDateTime');
end;


Function TryModifiedJulianDateToDateTime(const AValue: Double; var ADateTime: TDateTime): Boolean;
begin
  Result:=False;
  NotYetImplemented('TryModifiedJulianDateToDateTime');
>>>>>>> graemeg/fixes_2_2
end;

{$ifdef RangeCheckWasOn}
{$R+}
{$undef RangeCheckWasOn}
{$endif}

{$ifdef OverflowCheckWasOn}
{$Q+}
{$undef OverflowCheckWasOn}
{$endif}

{ ---------------------------------------------------------------------
    Unix timestamp support.
  ---------------------------------------------------------------------}

Function DateTimeToUnix(const AValue: TDateTime): Int64;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=Round(DateTimeDiff(AValue,UnixEpoch)*SecsPerDay);
=======
  Result:=SecondsBetween(UnixEpoch, AValue);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=SecondsBetween(UnixEpoch, AValue);
>>>>>>> origin/fixes_2_2
end;


Function UnixToDateTime(const AValue: Int64): TDateTime;
begin
  Result:=IncSecond(UnixEpoch, AValue);
end;


Function UnixTimeStampToMac(const AValue: Int64): Int64;
const
  Epoch=24107 * 24 * 3600;
begin
  Result:=AValue + Epoch;
end;

{ ---------------------------------------------------------------------
    Mac timestamp support.
  ---------------------------------------------------------------------}

Function DateTimeToMac(const AValue: TDateTime): Int64;
var
  Epoch:TDateTime;
begin
  Epoch:=EncodeDateTime( 1904, 1, 1, 0, 0, 0, 0 );
  Result:=SecondsBetween( Epoch, AValue );
end;


Function MacToDateTime(const AValue: Int64): TDateTime;
var
  Epoch:TDateTime;
begin
  Epoch:=EncodeDateTime( 1904, 1, 1, 0, 0, 0, 0 );
  Result:=IncSecond( Epoch, AValue );
end;


Function MacTimeStampToUnix(const AValue: Int64): Int64;
const
  Epoch=24107 * 24 * 3600;
begin
  Result:=AValue - Epoch;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function DateTimeToDosDateTime(const AValue: TDateTime): longint;
var year,month,day,hour,min,sec,msec : word;
    zs : longint;
begin
  decodedatetime(avalue,year,month,day,hour,min,sec,msec);
  result:=-1980;
  result:=result+year and 127;
  result:=result shl 4;
  result:=result+month;
  result:=result shl 5;
  result:=result+day;
  result:=result shl 16;
  zs:=hour;
  zs:=zs shl 6;
  zs:=zs+min;
  zs:=zs shl 5;
  zs:=zs+sec div 2;
  result:=result+(zs and $ffff);
end;

Function DosDateTimeToDateTime( AValue: longint): TDateTime;
var year,month,day,hour,min,sec : integer;
begin
  sec:=(AValue and 31) * 2;
  avalue:=AValue shr 5;
  min:=AValue and 63;
  avalue:=AValue shr 6;
  hour:=AValue and 31;
  avalue:=AValue shr 5;
  day:=AValue and 31;
  avalue:=AValue shr 5;
  month:=AValue and 15;
  avalue:=AValue shr 4;
  year:=AValue+1980;
  result:=EncodeDateTime(year,month,day,hour,min,sec,0);
=======
=======
>>>>>>> origin/fixes_2_2
{
  Inverse of formatdatetime, destined for the dateutils unit of FPC.

  Limitations/implementation details:
  - An inverse of FormatDateTime is not 100% an inverse, simply because one can put e.g. time tokens twice in the format string,
       and scandatetime wouldn't know which time to pick.
  - Strings like hn can't be reversed safely. E.g. 1:2 (2 minutes after 1) delivers 12 which is parsed as 12:00 and then
       misses chars for the "n" part.
  - trailing characters are ignored.
  - no support for Eastern Asian formatting characters since they are windows only.
  - no MBCS support.

  Extensions
  - #9 eats whitespace.
  - whitespace at the end of a pattern is optional.
  - ? matches any char.
  - Quote the above chars to really match the char.
}

const whitespace  = [' ',#13,#10];
      hrfactor    = 1/(24);
      minfactor   = 1/(24*60);
      secfactor   = 1/(24*60*60);
      mssecfactor = 1/(24*60*60*1000);

const AMPMformatting : array[0..2] of string =('am/pm','a/p','ampm');

procedure raiseexception(const s:string);

begin
  raise EConvertError.Create(s);
end;

function scandatetime(const pattern:string;const s:string;const fmt:TFormatSettings;startpos:integer=1) : tdatetime;

var len ,ind  : integer;
    yy,mm,dd  : integer;
    timeval   : TDateTime;
    activequote: char;

procedure intscandate(ptrn:pchar;plen:integer;poffs:integer);
// poffs is the offset to

var
    pind : integer;

function findimatch(const mnts:array of string;p:pchar):integer;
var i : integer;
begin
  result:=-1;
  i:=0;
  while (i<=high(mnts)) and (result=-1) do
    begin
      if AnsiStrLIComp(p,@mnts[i][1],length(mnts[i]))=0 then
        result:=i;
      inc(i);
    end;
end;

procedure arraymatcherror;

begin
  raiseexception(format(SNoArrayMatch,[pind+1,ind]))
end;

function findmatch(const mnts : array of string;const s:string):integer;
<<<<<<< HEAD
=======

begin
  result:=findimatch(mnts,@s[ind]);
  if result=-1 then
    arraymatcherror
  else
    begin
      inc(ind,length(mnts[result])+1);
      inc(pind,length(mnts[result])+1);
      inc(result); // was 0 based.
    end;
end;

var
    pivot,
    i     : integer;

function scanfixedint(maxv:integer):integer;
var c : char;
    oi:integer;
begin
  result:=0;
  oi:=ind;
  c:=ptrn[pind];
  while (pind<plen) and (ptrn[pind]=c) do inc(pind);
  while (maxv>0) and (ind<=len) and (s[ind] IN ['0'..'9']) do
    begin
      result:=result*10+ord(s[ind])-48;
      inc(ind);
      dec(maxv);
    end;
  if oi=ind then
    raiseexception(format(SPatternCharMismatch,[c,oi]));
end;

procedure matchchar(c:char);

begin
  if (ind>len) or (s[ind]<>c) then
    raiseexception(format(SNoCharMatch,[s[ind],c,pind+poffs+1,ind]));
  inc(pind);
  inc(ind);
end;

function scanpatlen:integer;
var c : char;
    lind : Integer;
begin
  result:=pind;
  lind:=pind;
  c:=ptrn[lind];

  while (lind<=plen) and (ptrn[lind]=c) do
      inc(lind);
  result:=lind-result;
end;

procedure matchpattern(const lptr:string);

var len:integer;
begin
  len:=length(lptr);
  if len>0 then
    intscandate(@lptr[1],len,pind+poffs);
end;

var lasttoken,lch : char;

begin
  pind:=0;     lasttoken:=' ';
  while (ind<=len) and (pind<plen) do
     begin
       lch:=upcase(ptrn[pind]);
       if activequote=#0 then
          begin
            if (lch='M') and (lasttoken='H') then
              begin
                i:=scanpatlen;
                if i>2 then
                  raiseexception(format(Shhmmerror,[poffs+pind+1]));
                timeval:=timeval+scanfixedint(2)* minfactor;
              end
            else
            case lch of
               'H':  timeval:=timeval+scanfixedint(2)* hrfactor;
               'D':  begin
                       i:=scanpatlen;
                       case i of
                          1,2 : dd:=scanfixedint(2);
                          3   : dd:=findmatch(fmt.shortDayNames,s);
                          4   : dd:=findmatch(fmt.longDayNames,s);
                          5   : matchpattern(fmt.shortdateformat);
                          6   : matchpattern(fmt.longdateformat);
                         end;
                     end;
               'N':  timeval:=timeval+scanfixedint(2)* minfactor;
               'S':  timeval:=timeval+scanfixedint(2)* secfactor;
               'Z':  timeval:=timeval+scanfixedint(3)* mssecfactor;
               'Y':  begin
                       i:=scanpatlen;
                       yy:=scanfixedint(i);
                       if i<=2 then
                         begin
                           pivot:=YearOf(now)-fmt.TwoDigitYearCenturyWindow;
                           inc(yy, pivot div 100 * 100);
                           if (fmt.TwoDigitYearCenturyWindow > 0) and (yy < pivot) then
                              inc(yy, 100);
                         end;
                      end;
               'M': begin
                       i:=scanpatlen;
                       case i of
                          1,2: mm:=scanfixedint(2);
                          3:   mm:=findmatch(fmt.ShortMonthNames,s);
                          4:   mm:=findmatch(fmt.LongMonthNames,s);
                          end;
                    end;
               'T' : begin
                       i:=scanpatlen;
                       case i of
                       1: matchpattern(fmt.shortdateformat);
                       2: matchpattern(fmt.longtimeformat);
                       end;
                     end;
               'A' : begin
                            i:=findimatch(AMPMformatting,@ptrn[pind]);
                            case i of
                              0: begin
                                   i:=findimatch(['AM','PM'],@s[ind]);
                                   case i of
                                     0: ;
                                     1: timeval:=timeval+12*hrfactor;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[0]));
                                   inc(ind,2);
                                 end;
                              1: begin
                                    case upcase(s[ind]) of
                                     'A' : ;
                                     'P' : timeval:=timeval+12*hrfactor;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[1]));
                                   inc(ind);
                                 end;
                               2: begin
                                    i:=findimatch([fmt.timeamstring,fmt.timepmstring],@s[ind]);
                                    case i of
                                     0: inc(ind,length(fmt.timeamstring));
                                     1: begin
                                          timeval:=timeval+12*hrfactor;
                                          inc(ind,length(fmt.timepmstring));
                                        end;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[2]));
                                   inc(pind,2);
                                   inc(ind,2);
                                 end;
                            else  // no AM/PM match. Assume 'a' is simply a char
                                matchchar(ptrn[pind]);
                             end;
                         end;
               '/' : matchchar(fmt.dateSeparator);
               ':' : begin
                       matchchar(fmt.TimeSeparator);
                       lch:=lasttoken;
                     end;
               #39,'"' : begin
                           activequote:=lch;
                           inc(pind);
                         end;
               'C' : begin
                       intscandate(@fmt.shortdateformat[1],length(fmt.ShortDateFormat),pind+poffs);
                       intscandate(@fmt.longtimeformat[1],length(fmt.longtimeformat),pind+poffs);
                       inc(pind);
                     end;
               '?' : begin
                       inc(pind);
                       inc(ind);
                     end;
               #9  : begin
                       while (ind<=len) and (s[ind] in whitespace) do
                         inc(ind);
                       inc(pind);
                     end;
               else
                 matchchar(ptrn[pind]);
             end; {case}
             lasttoken:=lch;
            end
          else
            begin
              if activequote=lch then
                begin
                  activequote:=#0;
                  inc(pind);
                end
              else
                matchchar(ptrn[pind]);
            end;
     end;
   if (pind<plen) and (plen>0) and (ptrn[plen-1]<>#9) then  // allow omission of trailing whitespace
     RaiseException(format(SFullpattern,[poffs+pind+1]));
end;

var plen:integer;

begin
  activequote:=#0;
  yy:=0; mm:=0; dd:=0;
  timeval:=0.0;
  len:=length(s); ind:=startpos;
  plen:=length(pattern);
  intscandate(@pattern[1],plen,0);
  result:=timeval;
  if (yy>0) and (mm>0) and (dd>0) then
     result:=result+encodedate(yy,mm,dd);
end;


function scandatetime(const pattern:string;const s:string;startpos:integer=1) : tdatetime; overload;

begin
 result:=scandatetime(pattern,s,defaultformatsettings);
end;
>>>>>>> origin/fixes_2_2

begin
  result:=findimatch(mnts,@s[ind]);
  if result=-1 then
    arraymatcherror
  else
    begin
      inc(ind,length(mnts[result])+1);
      inc(pind,length(mnts[result])+1);
      inc(result); // was 0 based.
    end;
end;

var
    pivot,
    i     : integer;

function scanfixedint(maxv:integer):integer;
var c : char;
    oi:integer;
begin
  result:=0;
  oi:=ind;
  c:=ptrn[pind];
  while (pind<plen) and (ptrn[pind]=c) do inc(pind);
  while (maxv>0) and (ind<=len) and (s[ind] IN ['0'..'9']) do
    begin
      result:=result*10+ord(s[ind])-48;
      inc(ind);
      dec(maxv);
    end;
  if oi=ind then
    raiseexception(format(SPatternCharMismatch,[c,oi]));
end;

procedure matchchar(c:char);

begin
  if (ind>len) or (s[ind]<>c) then
    raiseexception(format(SNoCharMatch,[s[ind],c,pind+poffs+1,ind]));
  inc(pind);
  inc(ind);
end;

function scanpatlen:integer;
var c : char;
    lind : Integer;
begin
  result:=pind;
  lind:=pind;
  c:=ptrn[lind];

  while (lind<=plen) and (ptrn[lind]=c) do
      inc(lind);
  result:=lind-result;
end;

procedure matchpattern(const lptr:string);

var len:integer;
begin
  len:=length(lptr);
  if len>0 then
    intscandate(@lptr[1],len,pind+poffs);
end;

var lasttoken,lch : char;

begin
  pind:=0;     lasttoken:=' ';
  while (ind<=len) and (pind<plen) do
     begin
       lch:=upcase(ptrn[pind]);
       if activequote=#0 then
          begin
            if (lch='M') and (lasttoken='H') then
              begin
                i:=scanpatlen;
                if i>2 then
                  raiseexception(format(Shhmmerror,[poffs+pind+1]));
                timeval:=timeval+scanfixedint(2)* minfactor;
              end
            else
            case lch of
               'H':  timeval:=timeval+scanfixedint(2)* hrfactor;
               'D':  begin
                       i:=scanpatlen;
                       case i of
                          1,2 : dd:=scanfixedint(2);
                          3   : dd:=findmatch(fmt.shortDayNames,s);
                          4   : dd:=findmatch(fmt.longDayNames,s);
                          5   : matchpattern(fmt.shortdateformat);
                          6   : matchpattern(fmt.longdateformat);
                         end;
                     end;
               'N':  timeval:=timeval+scanfixedint(2)* minfactor;
               'S':  timeval:=timeval+scanfixedint(2)* secfactor;
               'Z':  timeval:=timeval+scanfixedint(3)* mssecfactor;
               'Y':  begin
                       i:=scanpatlen;
                       yy:=scanfixedint(i);
                       if i<=2 then
                         begin
                           pivot:=YearOf(now)-fmt.TwoDigitYearCenturyWindow;
                           inc(yy, pivot div 100 * 100);
                           if (fmt.TwoDigitYearCenturyWindow > 0) and (yy < pivot) then
                              inc(yy, 100);
                         end;
                      end;
               'M': begin
                       i:=scanpatlen;
                       case i of
                          1,2: mm:=scanfixedint(2);
                          3:   mm:=findmatch(fmt.ShortMonthNames,s);
                          4:   mm:=findmatch(fmt.LongMonthNames,s);
                          end;
                    end;
               'T' : begin
                       i:=scanpatlen;
                       case i of
                       1: matchpattern(fmt.shortdateformat);
                       2: matchpattern(fmt.longtimeformat);
                       end;
                     end;
               'A' : begin
                            i:=findimatch(AMPMformatting,@ptrn[pind]);
                            case i of
                              0: begin
                                   i:=findimatch(['AM','PM'],@s[ind]);
                                   case i of
                                     0: ;
                                     1: timeval:=timeval+12*hrfactor;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[0]));
                                   inc(ind,2);
                                 end;
                              1: begin
                                    case upcase(s[ind]) of
                                     'A' : ;
                                     'P' : timeval:=timeval+12*hrfactor;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[1]));
                                   inc(ind);
                                 end;
                               2: begin
                                    i:=findimatch([fmt.timeamstring,fmt.timepmstring],@s[ind]);
                                    case i of
                                     0: inc(ind,length(fmt.timeamstring));
                                     1: begin
                                          timeval:=timeval+12*hrfactor;
                                          inc(ind,length(fmt.timepmstring));
                                        end;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[2]));
                                   inc(pind,2);
                                   inc(ind,2);
                                 end;
                            else  // no AM/PM match. Assume 'a' is simply a char
                                matchchar(ptrn[pind]);
                             end;
                         end;
               '/' : matchchar(fmt.dateSeparator);
               ':' : begin
                       matchchar(fmt.TimeSeparator);
                       lch:=lasttoken;
                     end;
               #39,'"' : begin
                           activequote:=lch;
                           inc(pind);
                         end;
               'C' : begin
                       intscandate(@fmt.shortdateformat[1],length(fmt.ShortDateFormat),pind+poffs);
                       intscandate(@fmt.longtimeformat[1],length(fmt.longtimeformat),pind+poffs);
                       inc(pind);
                     end;
               '?' : begin
                       inc(pind);
                       inc(ind);
                     end;
               #9  : begin
                       while (ind<=len) and (s[ind] in whitespace) do
                         inc(ind);
                       inc(pind);
                     end;
               else
                 matchchar(ptrn[pind]);
             end; {case}
             lasttoken:=lch;
            end
          else
            begin
              if activequote=lch then
                begin
                  activequote:=#0;
                  inc(pind);
                end
              else
                matchchar(ptrn[pind]);
            end;
     end;
   if (pind<plen) and (plen>0) and (ptrn[plen-1]<>#9) then  // allow omission of trailing whitespace
     RaiseException(format(SFullpattern,[poffs+pind+1]));
end;

var plen:integer;

begin
  activequote:=#0;
  yy:=0; mm:=0; dd:=0;
  timeval:=0.0;
  len:=length(s); ind:=startpos;
  plen:=length(pattern);
  intscandate(@pattern[1],plen,0);
  result:=timeval;
  if (yy>0) and (mm>0) and (dd>0) then
     result:=result+encodedate(yy,mm,dd);
end;


function scandatetime(const pattern:string;const s:string;startpos:integer=1) : tdatetime; overload;

begin
 result:=scandatetime(pattern,s,defaultformatsettings);
>>>>>>> graemeg/fixes_2_2
end;


{
  Inverse of formatdatetime, destined for the dateutils unit of FPC.

  Limitations/implementation details:
  - An inverse of FormatDateTime is not 100% an inverse, simply because one can put e.g. time tokens twice in the format string,
       and scandatetime wouldn't know which time to pick.
  - Strings like hn can't be reversed safely. E.g. 1:2 (2 minutes after 1) delivers 12 which is parsed as 12:00 and then
       misses chars for the "n" part.
  - trailing characters are ignored.
  - no support for Eastern Asian formatting characters since they are windows only.
  - no MBCS support.

  Extensions
  - #9 eats whitespace.
  - whitespace at the end of a pattern is optional.
  - ? matches any char.
  - Quote the above chars to really match the char.
}

const whitespace  = [' ',#13,#10];
      hrfactor    = 1/(24);
      minfactor   = 1/(24*60);
      secfactor   = 1/(24*60*60);
      mssecfactor = 1/(24*60*60*1000);

const AMPMformatting : array[0..2] of string =('am/pm','a/p','ampm');

procedure raiseexception(const s:string);

begin
  raise EConvertError.Create(s);
end;

function scandatetime(const pattern:string;const s:string;const fmt:TFormatSettings;startpos:integer=1) : tdatetime;

var len ,ind  : integer;
    yy,mm,dd  : integer;
    timeval   : TDateTime;
    activequote: char;

procedure intscandate(ptrn:pchar;plen:integer;poffs:integer);
// poffs is the offset to

var
    pind : integer;

function findimatch(const mnts:array of string;p:pchar):integer;
var i : integer;
begin
  result:=-1;
  i:=0;
  while (i<=high(mnts)) and (result=-1) do
    begin
      if AnsiStrLIComp(p,@mnts[i][1],length(mnts[i]))=0 then
        result:=i;
      inc(i);
    end;
end;

procedure arraymatcherror;

begin
  raiseexception(format(SNoArrayMatch,[pind+1,ind]))
end;

function findmatch(const mnts : array of string;const s:string):integer;

begin
  result:=findimatch(mnts,@s[ind]);
  if result=-1 then
    arraymatcherror
  else
    begin
      inc(ind,length(mnts[result])+1);
      inc(pind,length(mnts[result])+1);
      inc(result); // was 0 based.
    end;
end;

var
    pivot,
    i     : integer;

function scanfixedint(maxv:integer):integer;
var c : char;
    oi:integer;
begin
  result:=0;
  oi:=ind;
  c:=ptrn[pind];
  while (pind<plen) and (ptrn[pind]=c) do inc(pind);
  while (maxv>0) and (ind<=len) and (s[ind] IN ['0'..'9']) do
    begin
      result:=result*10+ord(s[ind])-48;
      inc(ind);
      dec(maxv);
    end;
  if oi=ind then
    raiseexception(format(SPatternCharMismatch,[c,oi]));
end;

procedure matchchar(c:char);

begin
  if (ind>len) or (s[ind]<>c) then
    raiseexception(format(SNoCharMatch,[s[ind],c,pind+poffs+1,ind]));
  inc(pind);
  inc(ind);
end;

function scanpatlen:integer;
var c : char;
    lind : Integer;
begin
  result:=pind;
  lind:=pind;
  c:=ptrn[lind];

  while (lind<=plen) and (ptrn[lind]=c) do
      inc(lind);
  result:=lind-result;
end;

procedure matchpattern(const lptr:string);

var len:integer;
begin
  len:=length(lptr);
  if len>0 then
    intscandate(@lptr[1],len,pind+poffs);
end;

var lasttoken,lch : char;

begin
  pind:=0;     lasttoken:=' ';
  while (ind<=len) and (pind<plen) do
     begin
       lch:=upcase(ptrn[pind]);
       if activequote=#0 then
          begin
            if (lch='M') and (lasttoken='H') then
              begin
                i:=scanpatlen;
                if i>2 then
                  raiseexception(format(Shhmmerror,[poffs+pind+1]));
                timeval:=timeval+scanfixedint(2)* minfactor;
              end
            else
            case lch of
               'H':  timeval:=timeval+scanfixedint(2)* hrfactor;
               'D':  begin
                       i:=scanpatlen;
                       case i of
                          1,2 : dd:=scanfixedint(2);
                          3   : dd:=findmatch(fmt.shortDayNames,s);
                          4   : dd:=findmatch(fmt.longDayNames,s);
                          5   : matchpattern(fmt.shortdateformat);
                          6   : matchpattern(fmt.longdateformat);
                         end;
                     end;
               'N':  timeval:=timeval+scanfixedint(2)* minfactor;
               'S':  timeval:=timeval+scanfixedint(2)* secfactor;
               'Z':  timeval:=timeval+scanfixedint(3)* mssecfactor;
               'Y':  begin
                       i:=scanpatlen;
                       yy:=scanfixedint(i);
                       if i<=2 then
                         begin
                           pivot:=YearOf(now)-fmt.TwoDigitYearCenturyWindow;
                           inc(yy, pivot div 100 * 100);
                           if (fmt.TwoDigitYearCenturyWindow > 0) and (yy < pivot) then
                              inc(yy, 100);
                         end;
                      end;
               'M': begin
                       i:=scanpatlen;
                       case i of
                          1,2: mm:=scanfixedint(2);
                          3:   mm:=findmatch(fmt.ShortMonthNames,s);
                          4:   mm:=findmatch(fmt.LongMonthNames,s);
                          end;
                    end;
               'T' : begin
                       i:=scanpatlen;
                       case i of
                       1: matchpattern(fmt.shortdateformat);
                       2: matchpattern(fmt.longtimeformat);
                       end;
                     end;
               'A' : begin
                            i:=findimatch(AMPMformatting,@ptrn[pind]);
                            case i of
                              0: begin
                                   i:=findimatch(['AM','PM'],@s[ind]);
                                   case i of
                                     0: ;
                                     1: timeval:=timeval+12*hrfactor;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[0]));
                                   inc(ind,2);
                                 end;
                              1: begin
                                    case upcase(s[ind]) of
                                     'A' : ;
                                     'P' : timeval:=timeval+12*hrfactor;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[1]));
                                   inc(ind);
                                 end;
                               2: begin
                                    i:=findimatch([fmt.timeamstring,fmt.timepmstring],@s[ind]);
                                    case i of
                                     0: inc(ind,length(fmt.timeamstring));
                                     1: begin
                                          timeval:=timeval+12*hrfactor;
                                          inc(ind,length(fmt.timepmstring));
                                        end;
                                   else
                                     arraymatcherror
                                     end;
                                   inc(pind,length(AMPMformatting[2]));
                                   inc(pind,2);
                                   inc(ind,2);
                                 end;
                            else  // no AM/PM match. Assume 'a' is simply a char
                                matchchar(ptrn[pind]);
                             end;
                         end;
               '/' : matchchar(fmt.dateSeparator);
               ':' : begin
                       matchchar(fmt.TimeSeparator);
                       lch:=lasttoken;
                     end;
               #39,'"' : begin
                           activequote:=lch;
                           inc(pind);
                         end;
               'C' : begin
                       intscandate(@fmt.shortdateformat[1],length(fmt.ShortDateFormat),pind+poffs);
                       intscandate(@fmt.longtimeformat[1],length(fmt.longtimeformat),pind+poffs);
                       inc(pind);
                     end;
               '?' : begin
                       inc(pind);
                       inc(ind);
                     end;
               #9  : begin
                       while (ind<=len) and (s[ind] in whitespace) do
                         inc(ind);
                       inc(pind);
                     end;
               else
                 matchchar(ptrn[pind]);
             end; {case}
             lasttoken:=lch;
            end
          else
            begin
              if activequote=lch then
                begin
                  activequote:=#0;
                  inc(pind);
                end
              else
                matchchar(ptrn[pind]);
            end;
     end;
   if (pind<plen) and (plen>0) and (ptrn[plen-1]<>#9) then  // allow omission of trailing whitespace
     RaiseException(format(SFullpattern,[poffs+pind+1]));
end;

var plen:integer;

begin
  activequote:=#0;
  yy:=0; mm:=0; dd:=0;
  timeval:=0.0;
  len:=length(s); ind:=startpos;
  plen:=length(pattern);
  intscandate(@pattern[1],plen,0);
  result:=timeval;
  if (yy>0) and (mm>0) and (dd>0) then
     result:=result+encodedate(yy,mm,dd);
end;


function scandatetime(const pattern:string;const s:string;startpos:integer=1) : tdatetime; overload;

begin
 result:=scandatetime(pattern,s,defaultformatsettings);
end;

{$else}
implementation
{$endif FPUNONE}
end.
