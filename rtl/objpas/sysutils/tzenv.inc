{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Tomas Hajny,
    member of the Free Pascal development team.

    Support routines for calculation of local timezone and DST time
    offset based on information provided in the environment variable TZ.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


type
 DSTSpecType = (DSTMonthWeekDay, DSTMonthDay, DSTJulian, DSTJulianX);

const
  TZEnvName = 'TZ';
{$IFDEF OS2}
  EMXTZEnvName = 'EMXTZ';
{$ENDIF OS2}
  MaxSecond = 86399;
(* The following values differing from the defaults *)
(* below are not used at the moment. *)
  USDSTStartMonth = 3;
  USDSTStartWeek = 2;
  USDSTEndMonth = 11;
  USDSTEndWeek = 1;
  EUDSTStartMonth = 3;
  EUDSTStartWeek = -1;
(* Initialized to default values, updated after a call to InitTZ *)
  TZName: string = '';
  TZDSTName: string = '';
  TZOffset: longint = 0;
  TZOffsetMin: longint = 0;
  DSTOffset: longint = 0;
  DSTOffsetMin: longint = 0;
  DSTStartMonth: byte = 4;
  DSTStartWeek: shortint = 1;
  DSTStartDay: word = 0;
  DSTStartSec: cardinal = 7200;
  DSTEndMonth: byte = 10;
  DSTEndWeek: shortint = -1;
  DSTEndDay: word = 0;
  DSTEndSec: cardinal = 10800;
  DSTStartSpecType: DSTSpecType = DSTMonthWeekDay;
  DSTEndSpecType: DSTSpecType = DSTMonthWeekDay;

(* The following variables are initialized after a call to InitTZ. *)
var
  RealDSTStartMonth, RealDSTStartDay, RealDSTEndMonth, RealDSTEndDay: byte;

const
  MonthEnds: array [1..12] of word =
                     (31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365);


function LeapDay (Year: word): byte; inline;
begin
  if IsLeapYear (Year) then
   LeapDay := 1
  else
   LeapDay := 0;
end;


function FirstDay (MM: byte; Y: word; Mo: word; D: word; WD: word): byte;
                                                                        inline;
var
  DD: longint;
begin
  if MM < Mo then
   begin
    DD := D + MonthEnds [Pred (Mo)];
    if MM > 1 then
     Dec (DD, MonthEnds [Pred (MM)]);
    if (MM <= 2) and (Mo > 2) then
     Inc (DD, LeapDay (Y));
   end
  else
   if MM > Mo then
    begin
     DD := - MonthDays [false, Mo] + D - MonthEnds [Pred (MM)]
                                                              + MonthEnds [Mo];
     if (Mo <= 2) and (MM > 2) then
      Dec (DD, LeapDay (Y));
    end
   else
(* M = MM *)
    DD := D;
  DD := WD - DD mod 7 + 1;
  if DD < 0 then
   FirstDay := DD + 7
  else
   FirstDay := DD mod 7;
end;


procedure UpdateTimeWithOffset (var SystemTime: TSystemTime; Offset: longint);
                                                                        inline;
var
  Y: longint;
  Mo: longint;
  D: longint;
  WD: word;
  H: longint;
  Mi: longint;
begin
  with SystemTime do
   begin
    Y := Year;
    Mo := Month;
    D := Day;
    WD := DayOfWeek;
    H := Hour;
    Mi := Minute;
   end;
  Mi := Mi + (Offset mod 60);
  H := H + (Offset div 60);
  if Mi < 0 then
   begin
    Inc (Mi, 60);
    Dec (H);
   end;
  if H < 0 then
   begin
    Inc (H, 24);
    if WD = 0 then
     WD := 6
    else
     Dec (WD);
    if D = 1 then
     begin
      if Mo = 1 then
       begin
        Dec (Y);
        Mo := 12;
       end
      else
       Dec (Mo);
      D := MonthDays [IsLeapYear (Y), Mo];
     end
    else
     Dec (D);
   end
  else
   begin
    if Mi > 59 then
     begin
      Dec (Mi, 60);
      Inc (H);
     end;
    if H > 23 then
     begin
      Dec (H, 24);
      if WD = 6 then
       WD := 0
      else
       Inc (WD);
      if D = MonthDays [IsLeapYear (Y), Mo] then
       begin
        D := 1;
        if Mo = 12 then
         begin
          Inc (Y);
          Mo := 1;
         end
        else
         Inc (Mo);
       end
      else
       Inc (D);
     end;
   end;
  with SystemTime do
   begin
    Year := Y;
    Month := Mo;
    Day := D;
    DayOfWeek := WD;
    Hour := H;
    Minute := Mi;
   end;
end;


function InDST (const Time: TSystemTime; const InputIsUTC: boolean): boolean;
var
  AfterDSTStart, BeforeDSTEnd: boolean;
  Y: longint;
  Mo: longint;
  D: longint;
  WD: longint;
  Second: longint;
begin
 InDST := false;
 if DSTOffset <> TZOffset then
  begin
   Second := longint (Time.Hour) * 3600 + Time.Minute * 60 + Time.Second;
   Y := Time.Year;
   Mo := Time.Month;
   D := Time.Day;
   if InputIsUTC and (TZOffset <> 0) then
    begin
     Second := Second - TZOffset;
     if Second < 0 then
      begin
       Second := Second + MaxSecond + 1;
       if D = 1 then
        begin
         if Mo = 1 then
          begin
           Dec (Y);
           Mo := 12;
          end
         else
          Dec (Mo);
         D := MonthDays [IsLeapYear (Y), Mo];
        end
       else
        Dec (D);
      end
     else
      if Second > MaxSecond then
       begin
        Second := Second - MaxSecond - 1;
        if D = MonthDays [IsLeapYear (Y), Mo] then
         begin
          D := 1;
          if Mo = 12 then
           begin
            Inc (Y);
            Mo := 1;
           end
          else
           Inc (Mo);
         end
        else
         Inc (D);
       end;
    end;
   if Mo < RealDSTStartMonth then
    AfterDSTStart := false
   else
    if Mo > RealDSTStartMonth then
     AfterDSTStart := true
    else
     if D < RealDSTStartDay then
      AfterDSTStart := false
     else
      if D > RealDSTStartDay then
       AfterDSTStart := true
      else
       AfterDSTStart := Second > DSTStartSec;
   if Mo > RealDSTEndMonth then
    BeforeDSTEnd := false
   else
    if Mo < RealDSTEndMonth then
     BeforeDSTEnd := true
    else
     if D > RealDSTEndDay then
      BeforeDSTEnd := false
     else
      if D < RealDSTEndDay then
       BeforeDSTEnd := true
      else
       BeforeDSTEnd := Second < DSTEndSec;
   InDST := AfterDSTStart and BeforeDSTEnd;
  end;
end;


function InDST: boolean; inline;
var
  SystemTime: TSystemTime;
begin
 InDST := false;
 if DSTOffset <> TZOffset then
  begin
   GetLocalTime (SystemTime);
   InDST := InDST (SystemTime, false);
  end;
end;


procedure InitTZ0; inline;
var
  TZ, S: string;
  I, J: byte;
  Err: longint;
  GnuFmt: boolean;
  ADSTStartMonth: byte;
  ADSTStartWeek: shortint;
  ADSTStartDay: word;
  ADSTStartSec: cardinal;
  ADSTEndMonth: byte;
  ADSTEndWeek: shortint;
  ADSTEndDay: word;
  ADSTEndSec: cardinal;
  ADSTStartSpecType: DSTSpecType;
  ADSTEndSpecType: DSTSpecType;
  ADSTChangeSec: cardinal;

  function ParseOffset (OffStr: string): longint;
  (* Parse time offset given as [-|+]HH[:MI[:SS]] and return in seconds *)
  var
    TZShiftHH, TZShiftDir: shortint;
    TZShiftMI, TZShiftSS: byte;
    N1, N2: byte;
  begin
    TZShiftHH := 0;
    TZShiftMI := 0;
    TZShiftSS := 0;
    TZShiftDir := 1;
    N1 := 1;
    while (N1 <= Length (OffStr)) and (OffStr [N1] <> ':') do
     Inc (N1);
    Val (Copy (OffStr, 1, Pred (N1)), TZShiftHH, Err);
    if (Err = 0) and (TZShiftHH >= -24) and (TZShiftHH <= 23) then
     begin
(* Normalize the hour offset to -12..11 if necessary *)
      if TZShiftHH > 11 then
       Dec (TZShiftHH, 24) else
      if TZShiftHH < -12 then
       Inc (TZShiftHH, 24);
      if TZShiftHH < 0 then
       TZShiftDir := -1;
      if (N1 <= Length (OffStr)) then
       begin
        N2 := Succ (N1);
        while (N2 <= Length (OffStr)) and (OffStr [N2] <> ':') do
         Inc (N2);
        Val (Copy (OffStr, Succ (N1), N2 - N1), TZShiftMI, Err);
         if (Err = 0) and (TZShiftMI <= 59) then
          begin
           if (N2 <= Length (OffStr)) then
            begin
             Val (Copy (OffStr, Succ (N2), Length (OffStr) - N2), TZShiftSS, Err);
             if (Err <> 0) or (TZShiftSS > 59) then
              TZShiftSS := 0;
            end
          end
         else
          TZShiftMI := 0;
       end;
     end
    else
     TZShiftHH := 0;
    ParseOffset := longint (TZShiftHH) * 3600 +
                           TZShiftDir * (longint (TZShiftMI) * 60 + TZShiftSS);
  end;

begin
  TZ := GetEnvironmentVariable (TZEnvName);
{$IFDEF OS2}
  if TZ = '' then
   TZ := GetEnvironmentVariable (EMXTZEnvName);
{$ENDIF OS2}
  if TZ <> '' then
   begin
    TZ := Upcase (TZ);
(* Timezone name *)
    I := 1;
    while (I <= Length (TZ)) and (TZ [I] in ['A'..'Z']) do
     Inc (I);
    TZName := Copy (TZ, 1, Pred (I));
    if I <= Length (TZ) then
     begin
(* Timezone shift *)
      J := Succ (I);
      while (J <= Length (TZ)) and not (TZ [J] in ['A'..'Z']) do
       Inc (J);
      TZOffset := ParseOffset (Copy (TZ, I, J - I));
(* DST timezone name *)
      I := J;
      while (J <= Length (TZ)) and (TZ [J] in ['A'..'Z']) do
       Inc (J);
      if J > I then
       begin
        TZDSTName := Copy (TZ, I, J - I);
(* DST timezone name provided; if equal to the standard timezone  *)
(* name then DSTOffset is set to be equal to TZOffset by default, *)
(* otherwise it is set to TZOffset - 3600 seconds.                *)
        if TZDSTName <> TZName then
         DSTOffset := -3600 + TZOffset
        else
         DSTOffset := TZOffset;
       end
      else
       begin
        TZDSTName := TZName;
(* No DST timezone name provided => DSTOffset is equal to TZOffset *)
        DSTOffset := TZOffset;
       end;
      if J <= Length (TZ) then
       begin
(* Check if DST offset is specified here;   *)
(* if not, default value set above is used. *)
        if TZ [J] <> ',' then
         begin
          I := J;
          Inc (J);
          while (J <= Length (TZ)) and (TZ [J] <> ',') do
           Inc (J);
          DSTOffset := ParseOffset (Copy (TZ, I, J - I));
         end;
        if J < Length (TZ) then
         begin
          Inc (J);
(* DST switching details *)
          case TZ [J] of
           'M':
            begin
(* Mmonth.week.dayofweek[/StartHour] *)
             ADSTStartSpecType := DSTMonthWeekDay;
             if J >= Length (TZ) then
              Exit;
             Inc (J);
             I := J;
             while (J <= Length (TZ)) and not (TZ [J] in ['.', ',', '/']) do
              Inc (J);
             if (J >= Length (TZ)) or (TZ [J] <> '.') then
              Exit;
             Val (Copy (TZ, I, J - I), ADSTStartMonth, Err);
             if (Err > 0) or (ADSTStartMonth > 12) then
              Exit;
             Inc (J);
             I := J;
             while (J <= Length (TZ)) and not (TZ [J] in ['.', ',', '/']) do
              Inc (J);
             if (J >= Length (TZ)) or (TZ [J] <> '.') then
              Exit;
             Val (Copy (TZ, I, J - I), ADSTStartWeek, Err);
             if (Err > 0) or (ADSTStartWeek < 1) or (ADSTStartWeek > 5) then
              Exit;
             Inc (J);
             I := J;
             while (J <= Length (TZ)) and not (TZ [J] in [',', '/']) do
              Inc (J);
             Val (Copy (TZ, I, J - I), ADSTStartDay, Err);
             if (Err > 0) or (ADSTStartDay > 6) or (J >= Length (TZ)) then
              Exit;
             if TZ [J] = '/' then
              begin
               Inc (J);
               I := J;
               while (J <= Length (TZ)) and (TZ [J] <> ',') do
                Inc (J);
               Val (Copy (TZ, I, J - I), ADSTStartSec, Err);
               if (Err > 0) or (ADSTStartSec > MaxSecond) or (J >= Length (TZ))
                                                                           then
                Exit
               else
                ADSTStartSec := ADSTStartSec * 3600;
              end
             else
              (* Use the preset default *)
              ADSTStartSec := DSTStartSec;
             Inc (J);
            end;
           'J':
            begin
(* Jjulianday[/StartHour] *)
             ADSTStartSpecType := DSTJulianX;
             if J >= Length (TZ) then
              Exit;
             Inc (J);
             I := J;
             while (J <= Length (TZ)) and not (TZ [J] in [',', '/']) do
              Inc (J);
             Val (Copy (TZ, I, J - I), ADSTStartDay, Err);
             if (Err > 0) or (ADSTStartDay = 0) or (ADSTStartDay > 365)
                                                     or (J >= Length (TZ)) then
              Exit;
             if TZ [J] = '/' then
              begin
               Inc (J);
               I := J;
               while (J <= Length (TZ)) and (TZ [J] <> ',') do
                Inc (J);
               Val (Copy (TZ, I, J - I), ADSTStartSec, Err);
               if (Err > 0) or (ADSTStartSec > MaxSecond) or (J >= Length (TZ))
                                                                           then
                Exit
               else
                ADSTStartSec := ADSTStartSec * 3600;
              end
             else
              (* Use the preset default *)
              ADSTStartSec := DSTStartSec;
             Inc (J);
            end
          else
           begin
(* Check the used format first - GNU libc / GCC / EMX expect                 *)
(* "NameOffsetDstname[Dstoffset],Start[/StartHour],End[/EndHour]";           *)
(* if more than one comma (',') is found, the following format is assumed:   *)
(* "NameOffsetDstname[Dstoffset],StartMonth,StartWeek,StartDay,StartSecond,  *)
(*                         EndMonth,EndWeek,EndDay,EndSecond,DSTDifference". *)
            I := J;
            while (J <= Length (TZ)) and (TZ [J] <> ',') do
             Inc (J);
            S := Copy (TZ, I, J - I);
            if J < Length (TZ) then
             begin
              Inc (J);
              I := J;
              while (J <= Length (TZ)) and (TZ [J] <> ',') do
               Inc (J);
              GnuFmt := J > Length (TZ);
             end
            else
             Exit;
            if GnuFmt then
             begin
              ADSTStartSpecType := DSTJulian;
              J := Pos ('/', S);
              if J = 0 then
               begin
                Val (S, ADSTStartDay, Err);
                if (Err > 0) or (ADSTStartDay > 365) then
                 Exit;
                (* Use the preset default *)
                ADSTStartSec := DSTStartSec;
               end
              else
               begin
                if J = Length (S) then
                 Exit;
                Val (Copy (S, 1, Pred (J)), ADSTStartDay, Err);
                if (Err > 0) or (ADSTStartDay > 365) then
                 Exit;
                Val (Copy (S, Succ (J), Length (S) - J), ADSTStartSec, Err);
                if (Err > 0) or (ADSTStartSec > MaxSecond) then
                 Exit
                else
                 ADSTStartSec := ADSTStartSec * 3600;
               end;
              J := I;
             end
            else
             begin
              Val (S, ADSTStartMonth, Err);
              if (Err > 0) or (ADSTStartMonth > 12) then
               Exit;
              Val (Copy (TZ, I, J - I), ADSTStartWeek, Err);
              if (Err > 0) or (ADSTStartWeek < -1) or (ADSTStartWeek > 5) or
                                                        (J >= Length (TZ)) then
               Exit;
              Inc (J);
              I := J;
              while (J <= Length (TZ)) and (TZ [J] <> ',') do
               Inc (J);
              Val (Copy (TZ, I, J - I), ADSTStartDay, Err);
              if (DSTStartWeek = 0) then
               begin
                if (Err > 0) or (ADSTStartDay < 1) or (ADSTStartDay > 31)
                  or (ADSTStartDay > 30) and (ADSTStartMonth in [4, 6, 9, 11])
                           or (ADSTStartMonth = 2) and (ADSTStartDay > 29) then
                 Exit;
                ADSTStartSpecType := DSTMonthDay;
               end
              else
               begin
                if (Err > 0) or (ADSTStartDay > 6) then
                 Exit;
                ADSTStartSpecType := DSTMonthWeekDay;
               end;
              if J >= Length (TZ) then
               Exit;
              Inc (J);
              I := J;
              while (J <= Length (TZ)) and (TZ [J] <> ',') do
               Inc (J);
              Val (Copy (TZ, I, J - I), ADSTStartSec, Err);
              if (Err > 0) or (ADSTStartSec > MaxSecond) or
                                                        (J >= Length (TZ)) then
               Exit;
              Inc (J);
              I := J;
              while (J <= Length (TZ)) and (TZ [J] <> ',') do
               Inc (J);
              Val (Copy (TZ, I, J - I), ADSTEndMonth, Err);
              if (Err > 0) or (ADSTEndMonth > 12) or (J >= Length (TZ)) then
               Exit;
              Inc (J);
              I := J;
              while (J <= Length (TZ)) and (TZ [J] <> ',') do
               Inc (J);
              Val (Copy (TZ, I, J - I), ADSTEndWeek, Err);
              if (Err > 0) or (ADSTEndWeek < -1) or (ADSTEndWeek > 5)
                                                     or (J >= Length (TZ)) then
               Exit;
              Inc (J);
              I := J;
              while (J <= Length (TZ)) and (TZ [J] <> ',') do
               Inc (J);
              Val (Copy (TZ, I, J - I), ADSTEndDay, Err);
              if (DSTEndWeek = 0) then
               begin
                if (Err > 0) or (ADSTEndDay < 1) or (ADSTEndDay > 31)
                   or (ADSTEndDay > 30) and (ADSTEndMonth in [4, 6, 9, 11])
                               or (ADSTEndMonth = 2) and (ADSTEndDay > 29) then
                 Exit;
                ADSTEndSpecType := DSTMonthDay;
               end
              else
               begin
                if (Err > 0) or (ADSTEndDay > 6) then
                 Exit;
                ADSTEndSpecType := DSTMonthWeekDay;
               end;
              if J >= Length (TZ) then
               Exit;
              Inc (J);
              I := J;
              while (J <= Length (TZ)) and (TZ [J] <> ',') do
               Inc (J);
              Val (Copy (TZ, I, J - I), ADSTEndSec, Err);
              if (Err > 0) or (ADSTEndSec > MaxSecond) or
                                                        (J >= Length (TZ)) then
               Exit;
              Val (Copy (TZ, Succ (J), Length (TZ) - J), ADSTChangeSec, Err);
              if (Err = 0) and (ADSTChangeSec < 86400) then
               begin
(* Format complete, all checks successful => accept the parsed values. *)
                DSTStartMonth := ADSTStartMonth;
                DSTStartWeek := ADSTStartWeek;
                DSTStartDay := ADSTStartDay;
                DSTStartSec := ADSTStartSec;
                DSTEndMonth := ADSTEndMonth;
                DSTEndWeek := ADSTEndWeek;
                DSTEndDay := ADSTEndDay;
                DSTEndSec := ADSTEndSec;
                DSTStartSpecType := ADSTStartSpecType;
                DSTEndSpecType := ADSTEndSpecType;
                DSTOffset := TZOffset - ADSTChangeSec;
               end;
(* Parsing finished *)
              Exit;
             end;
           end;
          end;
(* GnuFmt - DST end specification *)
          if TZ [J] = 'M' then
           begin
(* Mmonth.week.dayofweek *)
            ADSTEndSpecType := DSTMonthWeekDay;
            if J >= Length (TZ) then
             Exit;
            Inc (J);
            I := J;
            while (J <= Length (TZ)) and not (TZ [J] in ['.', ',', '/']) do
             Inc (J);
            if (J >= Length (TZ)) or (TZ [J] <> '.') then
             Exit;
            Val (Copy (TZ, I, J - I), ADSTEndMonth, Err);
            if (Err > 0) or (ADSTEndMonth > 12) then
             Exit;
            Inc (J);
            I := J;
            while (J <= Length (TZ)) and not (TZ [J] in ['.', ',', '/']) do
             Inc (J);
            if (J >= Length (TZ)) or (TZ [J] <> '.') then
             Exit;
            Val (Copy (TZ, I, J - I), ADSTEndWeek, Err);
            if (Err > 0) or (ADSTEndWeek < 1) or (ADSTEndWeek > 5) then
             Exit;
            Inc (J);
            I := J;
            while (J <= Length (TZ)) and (TZ [J] <> '/') do
             Inc (J);
            Val (Copy (TZ, I, J - I), ADSTEndDay, Err);
            if (Err > 0) or (ADSTEndDay > 6) then
             Exit;
           end
          else
           begin
            if TZ [J] = 'J' then
             begin
(* Jjulianday *)
              if J = Length (TZ) then
               Exit;
              Inc (J);
              ADSTEndSpecType := DSTJulianX
             end
            else
             ADSTEndSpecType := DSTJulian;
            if J >= Length (TZ) then
             Exit;
            Inc (J);
            I := J;
            while (J <= Length (TZ)) and (TZ [J] <> '/') do
             Inc (J);
            Val (Copy (TZ, I, J - I), ADSTEndDay, Err);
            if (Err > 0) or (ADSTEndDay = 0) and (ADSTEndSpecType = DSTJulianX)
                                                     or (ADSTEndDay > 365) then
             Exit;
           end;
          if (J <= Length (TZ)) and (TZ [J] = '/') then
           begin
            if J = Length (TZ) then
             Exit;
            Val (Copy (TZ, Succ (J), Length (TZ) - J), ADSTEndSec, Err);
            if (Err > 0) or (ADSTEndSec > MaxSecond) then
             Exit
            else
             ADSTEndSec := ADSTEndSec * 3600;
           end
          else
           (* Use the preset default *)
           ADSTEndSec := DSTEndSec;

(* Format complete, all checks successful => accept the parsed values. *)
         if ADSTStartSpecType = DSTMonthWeekDay then
          begin
           DSTStartMonth := ADSTStartMonth;
           DSTStartWeek := ADSTStartWeek;
          end;
         DSTStartDay := ADSTStartDay;
         DSTStartSec := ADSTStartSec;
         if ADSTStartSpecType = DSTMonthWeekDay then
          begin
           DSTEndMonth := ADSTEndMonth;
           DSTEndWeek := ADSTEndWeek;
          end;
          DSTEndDay := ADSTEndDay;
          DSTEndSec := ADSTEndSec;
          DSTStartSpecType := ADSTStartSpecType;
          DSTEndSpecType := ADSTEndSpecType;
         end;
       end
      else
       DSTOffset := -3600 + TZOffset;
     end;
   end;
end;

procedure InitTZ;
var
  L: longint;
  SystemTime: TSystemTime;
  Y: word absolute SystemTime.Year;
  Mo: word absolute SystemTime.Month;
  D: word absolute SystemTime.Day;
  WD: word absolute SystemTime.DayOfWeek;
begin
  InitTZ0;
  TZOffsetMin := TZOffset div 60;
  DSTOffsetMin := DSTOffset div 60;

  if DSTOffset <> TZOffset then
   begin
    GetLocalTime (SystemTime);
    if (DSTStartSpecType = DSTMonthWeekDay) or (DSTStartSpecType = DSTMonthDay)
                                                                           then
     begin
      RealDSTStartMonth := DSTStartMonth;
      if DSTStartSpecType = DSTMonthDay then
       RealDSTStartDay := DSTStartDay
      else
       begin
        RealDSTStartDay := FirstDay (DSTStartMonth, Y, Mo, D, WD);
        if (DSTStartWeek >= 1) and (DSTStartWeek <= 4) then
         if DSTStartDay < RealDSTStartDay then
          RealDSTStartDay := DSTStartWeek * 7 + DSTStartDay - RealDSTStartDay
                                                                            + 1
         else
          RealDSTStartDay := Pred (DSTStartWeek) * 7 + DSTStartDay
                                                          - RealDSTStartDay + 1
        else
(* Last week in month *)
         begin
          RealDSTStartDay := RealDSTStartDay
                                    + MonthDays [false, RealDSTStartMonth] - 1;
          if RealDSTStartMonth = 2 then
           Inc (RealDSTStartDay, LeapDay (Y));
          RealDSTStartDay := RealDSTStartDay mod 7;
          if RealDSTStartDay < DSTStartDay then
           RealDSTStartDay := RealDSTStartDay + 7 - DSTStartDay
          else
           RealDSTStartDay := RealDSTStartDay - DSTStartDay;
          RealDSTStartDay := MonthDays [false, RealDSTStartMonth]
                                                             - RealDSTStartDay;
         end;
       end;
     end
    else
     begin
(* Julian day *)
      L := DSTStartDay;
      if (DSTStartSpecType = DSTJulian) then
(* 0-based *)
       if (L + LeapDay (Y) <= 59) then
        Inc (L)
       else
        L := L + 1 - LeapDay (Y);
      if L <= 31 then
       begin
        RealDSTStartMonth := 1;
        RealDSTStartDay := L;
       end
      else
       if (L <= 59) or
                (DSTStartSpecType = DSTJulian) and (L - LeapDay (Y) <= 59) then
        begin
         RealDSTStartMonth := 2;
         RealDSTStartDay := DSTStartDay - 31;
        end
       else
        begin
         RealDSTStartMonth := 3;
         while (RealDSTStartMonth < 12) and (MonthEnds [RealDSTStartMonth] > L)
                                                                             do
          Inc (RealDSTStartMonth);
         RealDSTStartDay := L - MonthEnds [Pred (RealDSTStartMonth)];
        end;
     end;

    if (DSTEndSpecType = DSTMonthWeekDay) or (DSTEndSpecType = DSTMonthDay) then
     begin
      RealDSTEndMonth := DSTEndMonth;
      if DSTEndSpecType = DSTMonthDay then
       RealDSTEndDay := DSTEndDay
      else
       begin
        RealDSTEndDay := FirstDay (DSTEndMonth, Y, Mo, D, WD);
        if (DSTEndWeek >= 1) and (DSTEndWeek <= 4) then
         if DSTEndDay < RealDSTEndDay then
          RealDSTEndDay := DSTEndWeek * 7 + DSTEndDay - RealDSTEndDay + 1
         else
          RealDSTEndDay := Pred (DSTEndWeek) * 7 + DSTEndDay - RealDSTEndDay
                                                                            + 1
        else
(* Last week in month *)
         begin
          RealDSTEndDay := RealDSTEndDay + MonthDays [false, RealDSTEndMonth]
                                                                           - 1;
          if RealDSTEndMonth = 2 then
           Inc (RealDSTEndDay, LeapDay (Y));
          RealDSTEndDay := RealDSTEndDay mod 7;
          if RealDSTEndDay < DSTEndDay then
           RealDSTEndDay := RealDSTEndDay + 7 - DSTEndDay
          else
           RealDSTEndDay := RealDSTEndDay - DSTEndDay;
          RealDSTEndDay := MonthDays [false, RealDSTEndMonth] - RealDSTEndDay;
         end;
       end;
     end
    else
     begin
(* Julian day *)
      L := DSTEndDay;
      if (DSTEndSpecType = DSTJulian) then
(* 0-based *)
       if (L + LeapDay (Y) <= 59) then
        Inc (L)
       else
        L := L + 1 - LeapDay (Y);
      if L <= 31 then
       begin
        RealDSTEndMonth := 1;
        RealDSTEndDay := L;
       end
      else
       if (L <= 59) or
                  (DSTEndSpecType = DSTJulian) and (L - LeapDay (Y) <= 59) then
        begin
         RealDSTEndMonth := 2;
         RealDSTEndDay := DSTEndDay - 31;
        end
       else
        begin
         RealDSTEndMonth := 3;
         while (RealDSTEndMonth < 12) and (MonthEnds [RealDSTEndMonth] > L) do
          Inc (RealDSTEndMonth);
         RealDSTEndDay := L - MonthEnds [Pred (RealDSTEndMonth)];
        end;
     end;
   end;
end;

{$IFNDEF HAS_DUAL_TZHANDLING}
function GetUniversalTime (var SystemTime: TSystemTime): boolean;
begin
  GetLocalTime (SystemTime);
  UpdateTimeWithOffset (SystemTime, GetLocalTimeOffset);
  GetUniversalTime := true;
end;

function GetLocalTimeOffset: integer;
begin
  if InDST then
   GetLocalTimeOffset := DSTOffsetMin
  else
   GetLocalTimeOffset := TZOffsetMin;
end;
{$ENDIF HAS_DUAL_TZHANDLING}


function GetLocalTimeOffset(const DateTime: TDateTime; const InputIsUTC: boolean; out Offset: integer): boolean;
var
  SystemTime: TSystemTime;
begin
  DateTimeToSystemTime (DateTime, SystemTime);
  if InDST (SystemTime, InputIsUTC) then
   Offset := DSTOffsetMin
  else
   Offset := TZOffsetMin;
  GetLocalTimeOffset := true;
end;
