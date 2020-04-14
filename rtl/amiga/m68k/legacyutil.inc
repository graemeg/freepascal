{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 Karoly Balogh, Free Pascal Development team

    Amiga utility.library legacy (OS 1.x/2.x) support functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  This unit implements some of the utility.library functions for OS 1.x,
  where this library is missing, so the legacy OS support can be implemented
  with minimal changes to the normal system unit and common Amiga-like code

  Please note that this code doesn't aim to be API feature complete, just
  functional enough for the RTL code.
}

const
  // Start day of every month (without leap)
  StartOfMonth: array[0..11] of LongInt = (0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334);
  SecsPerMin = 60;
  SecsPerHour = 60 * SecsPerMin;
  SecsPerDay = 24 * SecsPerHour;
  SecsPerYear = 365 * SecsPerDay;// without leap
  AmigaStartYear = 1978; // Amiga starts @1978;
  DaysPerYear = 365;
  DaysPerLeapYear = DaysPerYear + 1;
  Daysof4Years = DaysPerLeapYear + 3 * DaysPerYear;
  Daysof100Years = 24 * DaysPerLeapYear + 76 * DaysPerYear;
  Daysof400Years = 97 * DaysPerLeapYear + 303 * DaysPerYear;


procedure Amiga2Date(seconds: Cardinal;
                     result : PClockData); public name '_fpc_amiga_amiga2date';
var
  IsLeap: boolean;
  d, y, i: LongWord;
begin// how many days are passed
  d := seconds div SecsPerDay;

  // the easier time part
  Result^.wday := d mod 7;
  Result^.sec := seconds mod 60;
  seconds := seconds div 60;
  Result^.min := seconds mod 60;
  seconds := seconds div 60;
  Result^.hour := seconds mod 24;

  // the leap year correction part
  IsLeap := True;
  //
  // before 2100 easier case (function mostly used for now(), so its usually in this range)
  if d < 92 * DaysPerYear + 30 * DaysPerLeapYear then  // 1978 - 2100 = 92 non leap, 30 leap years
  begin
    d := d + DaysPerLeapYear + DaysPerYear; // we want to start from 1976 (a leap year) so we add 2 more years to the nubmer we have
    y := 4 *(d div Daysof4Years) + 1976; // how many 4 year spans (1 leap + 3 non leap) are there?
    d := d mod Daysof4Years; //( get the day in the 4 year span)
    // the first yoear of such a 4 year span, is always a leap year, all other not (thats the reason we want to start at 1976)
    if d > DaysPerLeapYear then
    begin
      IsLeap := False;
      d := d - 1;
      y := y + d div DaysPerYear;
      d := d mod DaysPerYear;
    end;
  end
  else
  begin
    // more complicated way for dates > 2100 (not tested until now!)
    // we do the same as before but not 4 years together but 400 (because of the special years which are not leap even divided by 4)
    // and we start at 2000
    d := d - 17 * DaysPerYear + 5 * DaysPerLeapYear;
    y := 400 * (d div Daysof400Years) + 2000;
    d := d mod Daysof400Years;
    // first is always NOT leap year.. other we have to test
    if d >= DaysPerLeapYear then
    begin
      // we do the same again, and test for 100 year spans
      d := d - 1; // not a leap year one day down
      IsLeap := False;
      y := y + 100 * (d div Daysof100Years);
      d := d mod Daysof100Years;
      if d >= DaysPerYear then
      begin
        d := d + 1; // a leap year, one day up
        IsLeap := True;
        // and the same as we did before 4 years span
        y := y + 4 * (d div Daysof4Years);
        d := d mod Daysof4Years;
        if d >= DaysPerLeapYear then
        begin
          d := d - 1;
          IsLeap := False;
          y := y + d div DaysPerYear;
          d := d mod DaysPerYear;
        end;
      end;
    end;
  end;
  // the current year is a leap year and we are after Februar
  if IsLeap and (d >= StartOfMonth[2]) then
    d := d + 1;
  // get the actual month
  for i := 1 to High(StartOfMonth) do
  begin
    if StartOfMonth[i] > d then
    begin
      Result^.Month := i;
      d := d - (StartOfMonth[i - 1] + 1);
      break;
    end;
  end;
  Result^.year := y;
  Result^.mday := d;
end;

function Date2Amiga(date: PClockData): Cardinal; public name '_fpc_amiga_date2amiga';
var
  Y: LongInt;
  Leaps, LeapsBefore1978, Res: LongWord;
begin
  // the easy time part
  Res := date^.hour * SecsPerHour + date^.min * SecsPerMin + date^.sec;
  Res := Res + ((date^.mday - 1) + StartOfMonth[date^.month - 1]) * SecsPerDay;
  Res := Res + (date^.year - AmigaStartYear) * SecsPerYear;

  // leap year correction ;)

  // current year
  y := date^.year;
  // from IsLeapYear dos.pp
  if (y mod 400 = 0) or ((y mod 4 = 0) and (y mod 100 <> 0)) then
  begin
    // add a day if its after feb in a leap year
    if date^.month > 2 then
      Res := Res + SecsPerDay;
  end;

  // previous years
  y := date^.year - 1;
  Leaps := (y div 4) - (y div 100) + (y div 400);
  // exclude the ones before 1978
  LeapsBefore1978 := (AmigaStartYear div 4) - (AmigaStartYear div 100) + (AmigaStartYear div 400);

  Date2Amiga := Res + (leaps - leapsBefore1978) * SecsPerDay;
end;

