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

{==============================================================================}
{   internal functions                                                         }
{==============================================================================}

Function DoEncodeDate(Year, Month, Day: Word): longint;

Var
  D : TDateTime;

begin
  If TryEncodeDate(Year,Month,Day,D) then
    Result:=Trunc(D)
  else
    Result:=0;
end;

function DoEncodeTime(Hour, Minute, Second, MilliSecond: word): TDateTime;

begin
  If not TryEncodeTime(Hour,Minute,Second,MilliSecond,Result) then
    Result:=0;
end;

{==============================================================================}
{   Public functions                                                           }
{==============================================================================}

{   ComposeDateTime converts a Date and a Time into one TDateTime   }
function ComposeDateTime(Date,Time : TDateTime) : TDateTime;

begin
  if Date < 0 then Result := trunc(Date) - Abs(frac(Time))
  else Result := trunc(Date) + Abs(frac(Time));
end;

{   DateTimeToTimeStamp converts DateTime to a TTimeStamp   }

function DateTimeToTimeStamp(DateTime: TDateTime): TTimeStamp;

Var
  D : Double;
begin
  D:=DateTime * Single(MSecsPerDay);
  if D<0 then
    D:=D-0.5
  else
    D:=D+0.5;
  result.Time := Abs(Trunc(D)) Mod MSecsPerDay;
  result.Date := DateDelta + Trunc(D) div MSecsPerDay;
end;

{   TimeStampToDateTime converts TimeStamp to a TDateTime value   }

function TimeStampToDateTime(const TimeStamp: TTimeStamp): TDateTime;
begin
  Result := ComposeDateTime(TimeStamp.Date - DateDelta,TimeStamp.Time / MSecsPerDay)
end;

{   MSecsToTimeStamp   }

function MSecsToTimeStamp(MSecs: comp): TTimeStamp;
begin
  result.Date := Trunc(msecs / msecsperday);
  msecs:= msecs-comp(result.date)*msecsperday;
  result.Time := Round(MSecs);
end ;

{   TimeStampToMSecs   }

function TimeStampToMSecs(const TimeStamp: TTimeStamp): comp;
begin
  result := TimeStamp.Time + comp(timestamp.date)*msecsperday;
end ;

Function TryEncodeDate(Year,Month,Day : Word; Out Date : TDateTime) : Boolean;

var
  c, ya: cardinal;
begin
  Result:=(Year>0) and (Year<10000) and
          (Month in [1..12]) and
          (Day>0) and (Day<=MonthDays[IsleapYear(Year),Month]);
 If Result then
   begin
     if month > 2 then
      Dec(Month,3)
     else
      begin
        Inc(Month,9);
        Dec(Year);
      end;
     c:= Year DIV 100;
     ya:= Year - 100*c;
     Date := (146097*c) SHR 2 + (1461*ya) SHR 2 + (153*cardinal(Month)+2) DIV 5 + cardinal(Day);
     // Note that this line can't be part of the line above, since TDateTime is
     // signed and c and ya are not
     Date := Date - 693900;
   end
end;

function TryEncodeTime(Hour, Min, Sec, MSec:word; Out Time : TDateTime) : boolean;

begin
  Result:=(Hour<24) and (Min<60) and (Sec<60) and (MSec<1000);
  If Result then
    Time:=TDateTime(cardinal(Hour)*3600000+cardinal(Min)*60000+cardinal(Sec)*1000+MSec)/MSecsPerDay;
end;

{   EncodeDate packs three variables Year, Month and Day into a
    TDateTime value the result is the number of days since 12/30/1899   }

function EncodeDate(Year, Month, Day: word): TDateTime;

begin
  If Not TryEncodeDate(Year,Month,Day,Result) then
    Raise EConvertError.CreateFmt('%d-%d-%d is not a valid date specification',
                              [Year,Month,Day]);
end;

{   EncodeTime packs four variables Hour, Minute, Second and MilliSecond into
    a TDateTime value     }

function EncodeTime(Hour, Minute, Second, MilliSecond:word):TDateTime;

begin
  If not TryEncodeTime(Hour,Minute,Second,MilliSecond,Result) then
    Raise EConvertError.CreateFmt('%d:%d:%d.%d is not a valid time specification',
                              [Hour,Minute,Second,MilliSecond]);
end;


{   DecodeDate unpacks the value Date into three values:
    Year, Month and Day   }

procedure DecodeDate(Date: TDateTime; out Year, Month, Day: word);
var
  ly,ld,lm,j : cardinal;
begin
  if Date <= -datedelta then  // If Date is before 1-1-1 then return 0-0-0
    begin
    Year := 0;
    Month := 0;
    Day := 0;
    end
  else
    begin
    if Date>0 then
      Date:=Date+1/(msecsperday*2)
    else  
      Date:=Date-1/(msecsperday*2);
    if Date>MaxDateTime then
      Date:=MaxDateTime;
//       Raise EConvertError.CreateFmt('%f is not a valid TDatetime encoding, maximum value is %f.',[Date,MaxDateTime]);
    j := pred((longint(Trunc(System.Int(Date))) + 693900) SHL 2);
    ly:= j DIV 146097;
    j:= j - 146097 * cardinal(ly);
    ld := j SHR 2;
    j:=(ld SHL 2 + 3) DIV 1461;
    ld:= (cardinal(ld) SHL 2 + 7 - 1461*j) SHR 2;
    lm:=(5 * ld-3) DIV 153;
    ld:= (5 * ld +2 - 153*lm) DIV 5;
    ly:= 100 * cardinal(ly) + j;
    if lm < 10 then
     inc(lm,3)
    else
      begin
        dec(lm,9);
        inc(ly);
      end;
    year:=ly;
    month:=lm;
    day:=ld;
    end;
end;


function DecodeDateFully(const DateTime: TDateTime; out Year, Month, Day, DOW: Word): Boolean;
begin
  DecodeDate(DateTime,Year,Month,Day);
  DOW:=DayOfWeek(DateTime);
  Result:=IsLeapYear(Year);
end;


{   DecodeTime unpacks Time into four values:
    Hour, Minute, Second and MilliSecond    }

procedure DecodeTime(Time: TDateTime; out Hour, Minute, Second, MilliSecond: word);
Var
  l : cardinal;
begin
 l := DateTimeToTimeStamp(Time).Time;
 Hour   := l div 3600000;
 l := l mod 3600000;
 Minute := l div 60000;
 l := l mod 60000;
 Second := l div 1000;
 l := l mod 1000;
 MilliSecond := l;
end;

{   DateTimeToSystemTime converts DateTime value to SystemTime   }

procedure DateTimeToSystemTime(DateTime: TDateTime; out SystemTime: TSystemTime);
begin
  DecodeDateFully(DateTime, SystemTime.Year, SystemTime.Month, SystemTime.Day,SystemTime.DayOfWeek);
  DecodeTime(DateTime, SystemTime.Hour, SystemTime.Minute, SystemTime.Second, SystemTime.MilliSecond);
  Dec(SystemTime.DayOfWeek);
end ;

{   SystemTimeToDateTime converts SystemTime to a TDateTime value   }

function SystemTimeToDateTime(const SystemTime: TSystemTime): TDateTime;
begin
  result := ComposeDateTime(DoEncodeDate(SystemTime.Year, SystemTime.Month, SystemTime.Day),
                            DoEncodeTime(SystemTime.Hour, SystemTime.Minute, SystemTime.Second, SystemTime.MilliSecond));
end ;

{   DayOfWeek returns the Day of the week (sunday is day 1)  }

function DayOfWeek(DateTime: TDateTime): integer;
begin
  Result := 1 + ((Trunc(DateTime) - 1) mod 7);
  If (Result<=0) then
    Inc(Result,7);
end;

{   Date returns the current Date   }

function Date: TDateTime;
var
  SystemTime: TSystemTime;
begin
  GetLocalTime(SystemTime);
  result := DoEncodeDate(SystemTime.Year, SystemTime.Month, SystemTime.Day);
end ;

{   Time returns the current Time   }

function Time: TDateTime;
var
  SystemTime: TSystemTime;
begin
  GetLocalTime(SystemTime);
  Result := DoEncodeTime(SystemTime.Hour,SystemTime.Minute,SystemTime.Second,SystemTime.MilliSecond);
end ;

{   Now returns the current Date and Time    }

function Now: TDateTime;
var
  SystemTime: TSystemTime;
begin
  GetLocalTime(SystemTime);
  result := systemTimeToDateTime(SystemTime);
end;

{   IncMonth increments DateTime with NumberOfMonths months,
    NumberOfMonths can be less than zero   }

function IncMonth(const DateTime: TDateTime; NumberOfMonths: integer = 1 ): TDateTime;
var
  Year, Month, Day : word;
begin
  DecodeDate(DateTime, Year, Month, Day);
  IncAMonth(Year, Month, Day, NumberOfMonths);
  result := ComposeDateTime(DoEncodeDate(Year, Month, Day), DateTime);
end ;

{   IncAMonth is the same as IncMonth, but operates on decoded date  }

procedure IncAMonth(var Year, Month, Day: Word; NumberOfMonths: Integer = 1);
var
  TempMonth, S: Integer;
begin
  If NumberOfMonths>=0 then
    s:=1
  else
    s:=-1;
  inc(Year,(NumberOfMonths div 12));
  TempMonth:=Month+(NumberOfMonths mod 12)-1;
  if (TempMonth>11) or
     (TempMonth<0) then
   begin
     Dec(TempMonth, S*12);
     Inc(Year, S);
   end;
  Month:=TempMonth+1;          {   Months from 1 to 12   }
  If (Day>MonthDays[IsLeapYear(Year)][Month]) then
    Day:=MonthDays[IsLeapYear(Year)][Month];
end;

{  IsLeapYear returns true if Year is a leap year   }

function IsLeapYear(Year: Word): boolean;
begin
  Result := (Year mod 4 = 0) and ((Year mod 100 <> 0) or (Year mod 400 = 0));
end;

{  DateToStr returns a string representation of Date using ShortDateFormat   }

function DateToStr(Date: TDateTime): string;
begin
  DateTimeToString(Result, 'ddddd', Date);
end ;

function DateToStr(Date: TDateTime; const FormatSettings: TFormatSettings): string;
begin
  DateTimeToString(result, FormatSettings.ShortDateFormat, Date, FormatSettings);
end;

{  TimeToStr returns a string representation of Time using LongTimeFormat   }

function TimeToStr(Time: TDateTime): string;
begin
  DateTimeToString(Result, 'tt', Time);
end ;

function TimeToStr(Time: TDateTime; const FormatSettings: TFormatSettings): string;
begin
  DateTimeToString(Result, FormatSettings.LongTimeFormat, Time, FormatSettings);
end;

{   DateTimeToStr returns a string representation of DateTime using ShortDateFormat and LongTimeFormat   }

Const
  DateTimeToStrFormat : Array[Boolean] of string = ('c','f');
  
function DateTimeToStr(DateTime: TDateTime; ForceTimeIfZero : Boolean = False): string;
begin
  DateTimeToString(Result, DateTimeToStrFormat[ForceTimeIfZero], DateTime)
end ;

function DateTimeToStr(DateTime: TDateTime; const FormatSettings: TFormatSettings; ForceTimeIfZero : Boolean = False): string;
begin
  DateTimeToString(Result,  DateTimeToStrFormat[ForceTimeIfZero], DateTime ,FormatSettings);
end;

{   StrToDate converts the string S to a TDateTime value
    if S does not represent a valid date value
    an EConvertError will be raised   }

function IntStrToDate(Out ErrorMsg : AnsiString; const S: PChar; Len : integer; const useformat : string; const defs:TFormatSettings; separator : char = #0): TDateTime;

const SInvalidDateFormat = '"%s" is not a valid date format';

procedure FixErrorMsg(const errm :ansistring;const errmarg : ansistring);

begin
  errormsg:=format(errm,[errmarg]);
end;

var
   df:string;
   d,m,y,ly:integer;
   n,i:longint;
   c:word;
   dp,mp,yp,which : Byte;
   s1:string;
   values:array[0..3] of longint;
   LocalTime:tsystemtime;
   YearMoreThenTwoDigits : boolean;
begin
  ErrorMsg:='';   Result:=0;
  While (Len>0) and (S[Len-1] in [' ',#8,#9,#10,#12,#13]) do
    Dec(len);
  if (Len=0) then
    begin
      FixErrorMsg(SInvalidDateFormat,'');
      exit;
    end;
  YearMoreThenTwoDigits := False;
  if separator = #0 then
    separator := defs.DateSeparator;
  df := UpperCase(useFormat);
  { Determine order of D,M,Y }
  yp:=0;
  mp:=0;
  dp:=0;
  Which:=0;
  i:=0;
  while (i<Length(df)) and (Which<3) do
   begin
     inc(i);
     Case df[i] of
       'Y' :
         if yp=0 then
          begin
            Inc(Which);
            yp:=which;
          end;
       'M' :
         if mp=0 then
          begin
            Inc(Which);
            mp:=which;
          end;
       'D' :
         if dp=0 then
          begin
            Inc(Which);
            dp:=which;
          end;
     end;
   end;
  for i := 1 to 3 do
    values[i] := 0;
  s1 := '';
  n := 0;
  dec(len);
  for i := 0 to len do
   begin
     if s[i] in ['0'..'9'] then
      s1 := s1 + s[i];

     { space can be part of the shortdateformat, and is defaultly in slovak
       windows, therefor it shouldn't be taken as separator (unless so specified)
       and ignored }
     if (Separator <> ' ') and (s[i] = ' ') then
       Continue;

     if (s[i] = separator) or ((i = len) and (s[i] in ['0'..'9'])) then
      begin
        inc(n);
        if n>3 then
          begin
            FixErrorMsg(SInvalidDateFormat,s);
            exit;
          end;
         // Check if the year has more then two digits (if n=yp, then we are evaluating the year.)
        if (n=yp) and (length(s1)>2) then YearMoreThenTwoDigits := True;
        val(s1, values[n], c);
//        Writeln(s1,'->',values[n]);
        if c<>0 then
          begin
            FixErrorMsg(SInvalidDateFormat,s);
            Exit;
          end;
        s1 := '';
      end
     else if not (s[i] in ['0'..'9']) then
       begin
         FixErrorMsg(SInvalidDateFormat,s);
         Exit;
       end;
   end ;
   if (Which<3) and (N>Which) then
    begin
    FixErrorMsg(SInvalidDateFormat,s);
    Exit;
    end; 
  // Fill in values.
  getLocalTime(LocalTime);
  ly := LocalTime.Year;
  for I:=1 to 3 do
    if values[i]>high(Word) then
      begin
      errormsg:='Invalid date';
      exit;
      end;
  If N=3 then
   begin
     y:=values[yp];
     m:=values[mp];
     d:=values[dp];
   end
  Else
  begin
    Y:=ly;
    If n<2 then
     begin
       d:=values[1];
       m := LocalTime.Month;
     end
    else
     If dp<mp then
      begin
        d:=values[1];
        m:=values[2];
      end
    else
      begin
        d:=values[2];
        m:=values[1];
      end;
  end;
  if (y >= 0) and (y < 100) and not YearMoreThenTwoDigits then
    begin
    ly := ly - defs.TwoDigitYearCenturyWindow;
    Inc(Y, ly div 100 * 100);
    if (defs.TwoDigitYearCenturyWindow > 0) and (Y < ly) then
      Inc(Y, 100);
    end;

  if not TryEncodeDate(y, m, d, result) then
    errormsg:='Invalid date';
end;

function StrToDate(const S: PChar; Len : integer; const useformat : string; separator : char = #0): TDateTime;

Var
  MSg : AnsiString;

begin
  Result:=IntStrToDate(Msg,S,Len,useFormat,DefaultFormatSettings,Separator);
  If (Msg<>'') then
    Raise EConvertError.Create(Msg);
end;

function StrToDate(const S: string; FormatSettings: TFormatSettings): TDateTime;
var
  Msg: AnsiString;
begin
  Result:=IntStrToDate(Msg,PChar(S),Length(S),FormatSettings.ShortDateFormat,FormatSettings);
  if Msg<>'' then
    raise EConvertError.Create(Msg);
end;

function StrToDate(const S: ShortString; const useformat : string; separator : char = #0): TDateTime;
begin
  // S[1] always exists for shortstring. Length 0 will trigger an error.
  result := StrToDate(@S[1],Length(s),UseFormat,separator);
end;

function StrToDate(const S: AnsiString; const useformat : string; separator : char = #0): TDateTime;
begin
  result := StrToDate(PChar(S),Length(s),UseFormat,separator);
end;

function StrToDate(const S: ShortString; separator : char): TDateTime;
begin
  // S[1] always exists for shortstring. Length 0 will trigger an error.
  result := StrToDate(@S[1],Length(s),DefaultFormatSettings.ShortDateFormat,separator)
end;

function StrToDate(const S: ShortString): TDateTime;
begin
  // S[1] always exists for shortstring. Length 0 will trigger an error.
  result := StrToDate(@S[1],Length(s),DefaultFormatSettings.ShortDateFormat,#0);
end;

function StrToDate(const S: AnsiString; separator : char): TDateTime;
begin
    result := StrToDate(Pchar(S),Length(s),DefaultFormatSettings.ShortDateFormat,separator)
end;

function StrToDate(const S: AnsiString): TDateTime;
begin
  result := StrToDate(Pchar(S),Length(s),DefaultFormatSettings.ShortDateFormat,#0);
end;

{   StrToTime converts the string S to a TDateTime value
    if S does not represent a valid time value an
    EConvertError will be raised   }


function IntStrToTime(Out ErrorMsg : AnsiString; const S: PChar; Len : integer;const defs:TFormatSettings; separator : char = #0): TDateTime;
const
  AMPM_None = 0;
  AMPM_AM = 1;
  AMPM_PM = 2;
  tiHour = 0;
  tiMin = 1;
  tiSec = 2;
  tiMSec = 3;
type
  TTimeValues = array[tiHour..tiMSec] of Word;
var
   AmPm: integer;
   TimeValues: TTimeValues;

    function StrPas(Src : PChar; len: integer = 0) : ShortString;
    begin
        //this is unsafe for len > 255, it will trash memory (I tested this)
        //reducing it is safe, since whenever we use this a string > 255 is invalid anyway
        if len > 255 then len := 255;
        SetLength(Result, len);
        move(src[0],result[1],len);
    end;

   function SplitElements(out TimeValues: TTimeValues; out AmPm: Integer): Boolean;
   //Strict version. It does not allow #32 as Separator, it will treat it as whitespace always
   const
     Digits = ['0'..'9'];
   var
      Cur, Offset, ElemLen, Err, TimeIndex, FirstSignificantDigit: Integer;
      Value: Word;
      DigitPending, MSecPending: Boolean;
      AmPmStr: ShortString;
      CurChar: Char;
   begin
     Result := False;
     AmPm := AMPM_None; //No Am or PM in string found yet
     MSecPending := False;
     TimeIndex := 0; //indicating which TTimeValue must be filled next
     FillChar(TimeValues, SizeOf(TTimeValues), 0);
     Cur := 0;
     //skip leading blanks
     While (Cur < Len) and (S[Cur] =#32) do Inc(Cur);
     Offset := Cur;
     //First non-blank cannot be Separator or DecimalSeparator
     if (Cur > Len - 1) or (S[Cur] = Separator) or (S[Cur] = defs.Decimalseparator) then Exit;
     DigitPending := (S[Cur] in Digits);
     While (Cur < Len) do
     begin
       //writeln;
       //writeln('Main While loop:  Cur = ',Cur,' S[Cur] = "',S[Cur],'" Len = ',Len);
       CurChar := S[Cur];
       if CurChar in Digits then
       begin//Digits
         //HH, MM, SS, or Msec?
         //writeln('Digit');
         //Digits are only allowed after starting Am/PM or at beginning of string or after Separator
         //and TimeIndex must be <= tiMSec
         //Uncomment "or (#32 = Separator)" and it will allllow #32 as separator
         if (not (DigitPending {or (#32 = Separator)})) or (TimeIndex > tiMSec) then Exit;
         OffSet := Cur;
         if (CurChar <> '0') then FirstSignificantDigit := OffSet else FirstSignificantDigit := -1;
         while (Cur < Len -1) and (S[Cur + 1] in Digits) do
         begin
           //Mark first Digit that is not '0'
           if (FirstSignificantDigit = -1) and (S[Cur] <> '0') then FirstSignificantDigit := Cur;
           Inc(Cur);
         end;
         if (FirstSignificantDigit = -1) then FirstSignificantDigit := Cur;
         ElemLen := 1 + Cur - FirstSignificantDigit;
         //writeln('  S[FirstSignificantDigit] = ',S[FirstSignificantDigit], ' S[Cur] = ',S[Cur],' ElemLen = ',ElemLen,' -> ', StrPas(S + Offset, ElemLen));
         //writeln('  Cur = ',Cur);
         //this way we know that Val() will never overflow Value !
         if (ElemLen <= 2) or ((ElemLen <= 3) and (TimeIndex = tiMSec) ) then
         begin
           Val(StrPas(S + FirstSignificantDigit, ElemLen), Value, Err);
           //writeln('  Value = ',Value,' HH = ',TimeValues[0],' MM = ',TimeValues[1],' SS = ',TimeValues[2],' MSec = ',Timevalues[3]);
           //This is safe now, because we know Value < High(Word)
           TimeValues[TimeIndex] := Value;
           Inc(TimeIndex);
           DigitPending := False;
         end
         else  Exit; //Value to big, so it must be a wrong timestring
       end//Digits
       else if (CurChar = #32) then
       begin
         //writeln('#32');
         //just skip, but we must adress this, or it will be parsed by either AM/PM or Separator
       end
       else if (CurChar = Separator) then
       begin
         //writeln('Separator');
         if DigitPending or (TimeIndex > tiSec) then Exit;
         DigitPending := True;
         MSecPending := False;
       end
       else if (CurChar = defs.DecimalSeparator) then
       begin
         //writeln('DecimalSeparator');
         if DigitPending or MSecPending or (TimeIndex <> tiMSec) then Exit;
         DigitPending := True;
         MSecPending := True;
       end
       else
       begin//AM/PM?
         //None of the above, so this char _must_ be the start of AM/PM string
         //If we already have found AM/PM or we expect a digit then then timestring must be wrong at this point
         //writeln('AM/PM?');
         if (AmPm <> AMPM_None) or DigitPending then Exit;
         OffSet := Cur;
         while (Cur < Len -1) and (not (S[Cur + 1] in [Separator, #32, defs.DecimalSeparator]))
           and not (S[Cur + 1] in Digits) do Inc(Cur);
         ElemLen := 1 + Cur - OffSet;
         //writeln('  S[Offset] = ',S[Offset], ' S[Cur] = ',S[Cur],' ElemLen = ',ElemLen,' -> ', StrPas(S + Offset, ElemLen));
         //writeln('  Cur = ',Cur);
         AmPmStr := StrPas(S + OffSet, ElemLen);

         //writeln('AmPmStr = ',ampmstr,' (',length(ampmstr),')');
         //We must compare to TimeAMString before hardcoded 'AM' for delphi compatibility
         //Also it is perfectly legal, though insane to have TimeAMString = 'PM' and vice versa
         if (AnsiCompareText(AmPmStr, defs.TimeAMString) = 0) then AmPm := AMPM_AM
         else if (AnsiCompareText(AmPmStr, defs.TimePMString) = 0) then AmPm := AMPM_PM
         else if (CompareText(AmPmStr, 'AM') = 0) then AmPm := AMPM_AM
         else if (CompareText(AmPmStr, 'PM') = 0) then AmPm := AMPM_PM
         else Exit; //If text does not match any of these, timestring must be wrong;
         //if AM/PM is at beginning of string, then a digit is mandatory after it
         if (TimeIndex = tiHour) then
         begin
           DigitPending := True;
         end
         //otherwise, no more TimeValues allowed after this
         else
         begin
           TimeIndex := tiMSec + 1;
           DigitPending := False;
         end;
       end;//AM/PM
       Inc(Cur)
     end;//while

     //If we arrive here, parsing the elements has been successfull
     //if not at least Hours specified then input is not valid
     //when am/pm is specified Hour must be <= 12 and not 0
     if (TimeIndex = tiHour) or ((AmPm <> AMPM_None) and ((TimeValues[tiHour] > 12) or (TimeValues[tiHour] = 0))) or DigitPending then Exit;
     Result := True;
   end;

begin
  if separator = #0 then
        separator := defs.TimeSeparator;
  AmPm := AMPM_None;
  if not SplitElements(TimeValues, AmPm) then
  begin
    ErrorMsg:=Format(SErrInvalidTimeFormat,[StrPas(S, Len)]);
    Exit;
  end;
  if (AmPm=AMPM_PM) and (TimeValues[tiHour]<>12) then Inc(TimeValues[tiHour], 12)
  else if (AmPm=AMPM_AM) and (TimeValues[tiHour]=12) then TimeValues[tiHour]:=0;

  if not TryEncodeTime(TimeValues[tiHour], TimeValues[tiMin], TimeValues[tiSec], TimeValues[tiMSec], result) Then
    //errormsg:='Invalid time.';
    ErrorMsg:=Format(SErrInvalidTimeFormat,[StrPas(S, Len)]);
end ;

function StrToTime(const S: PChar; Len : integer; separator : char = #0): TDateTime;

Var
  Msg : AnsiString;

begin
  Result:=IntStrToTime(Msg,S,Len,DefaultFormatSettings,Separator);
  If (Msg<>'') then
    Raise EConvertError.Create(Msg);
end;

function StrToTime(const S: string; FormatSettings : TFormatSettings): TDateTime;
Var
  Msg : AnsiString;
begin
  Result:=IntStrToTime(Msg, PChar(S), length(S), FormatSettings, #0);
  If (Msg<>'') then
    Raise EConvertError.Create(Msg);
end;

function StrToTime(const s: ShortString; separator : char): TDateTime;
begin
  // S[1] always exists for shortstring. Length 0 will trigger an error.
  result := StrToTime(@s[1], length(s), separator);
end;

function StrToTime(const s: AnsiString; separator : char): TDateTime;
begin
   result := StrToTime(PChar(S), length(s), separator);
end;

function StrToTime(const s: ShortString): TDateTime;
begin
  // S[1] always exists for shortstring. Length 0 will trigger an error.
   result := StrToTime(@s[1], length(s), #0);
end;

function StrToTime(const s: AnsiString): TDateTime;
begin
   result:= StrToTime(PChar(s), length(s), #0);
end;

{   StrToDateTime converts the string S to a TDateTime value
    if S does not represent a valid date and/or time value
    an EConvertError will be raised   }

function SplitDateTimeStr(DateTimeStr: AnsiString; const FS: TFormatSettings; out DateStr, TimeStr: AnsiString): Integer;

{ Helper function for StrToDateTime
  Pre-condition
    Date is before Time
    If either Date or Time is omitted then see what fits best, a time or a date (issue #0020522)
    Date and Time are separated by whitespace (space Tab, Linefeed or carriage return)
    FS.DateSeparator can be the same as FS.TimeSeparator (issue #0020522)
    If they are both #32 and TrimWhite(DateTimeStr) contains a #32 a date is assumed.
  Post-condition
    DateStr holds date as string or is empty
    TimeStr holds time as string or is empty
    Result = number of strings returned, 0 = error
}
const
  WhiteSpace = [#9,#10,#13,#32];
  Space : String = #32; // String, to avoid error 'Cannot decide what overload to call'

var
  p: Integer;
  DummyDT: TDateTime;
begin
  Result := 0;
  DateStr := '';
  TimeStr := '';
  DateTimeStr := Trim(DateTimeStr);
  if Length(DateTimeStr) = 0 then exit;
  if (FS.DateSeparator = #32) and (FS.TimeSeparator = #32) and (Pos(#32, DateTimeStr) > 0) then 
    begin
    DateStr:=DateTimeStr;
    {
      Assume a date: dd [mm [yy]]. 
      Really fancy would be counting the number of whitespace occurrences and decide 
      and split accordingly
    }
    Exit(1); 
    end;
  p:=1;
  //find separator
  if Pos(Space,FS.DateSeparator)=0 then
    begin  
    while (p<Length(DateTimeStr)) and (not (DateTimeStr[p+1] in WhiteSpace)) do 
      Inc(p);
    end
  else
    begin
    p:=Pos(FS.TimeSeparator, DateTimeStr);
    if (p<>0) then 
      repeat
        Dec(p);
      until (p=0) or (DateTimeStr[p] in WhiteSpace);
    end;
  //Always fill DateStr, it eases the algorithm later
  if (p=0) then 
    p:=Length(DateTimeStr);
  DateStr:=Copy(DateTimeStr,1,p);
  TimeStr:=Trim(Copy(DateTimeStr,p+1,MaxInt));
  if (Length(TimeStr)<>0) then
    Result:=2
  else
    begin
    Result:=1; //found 1 string
    // 2 cases when DateTimeStr only contains a time:
    // Date/time separator differ, and string contains a timeseparator
    // Date/time separators are equal, but transformation to date fails.
    if ((FS.DateSeparator<>FS.TimeSeparator) and (Pos(FS.TimeSeparator,DateStr) > 0))
       or ((FS.DateSeparator=FS.TimeSeparator) and (not TryStrToDate(DateStr, DummyDT, FS)))  then
      begin
      TimeStr := DateStr;
      DateStr := '';
      end;
    end;
end; 


function StrToDateTime(const s: AnsiString; const FormatSettings : TFormatSettings): TDateTime;
var
  TimeStr, DateStr: AnsiString;
  PartsFound: Integer;
begin
  PartsFound := SplitDateTimeStr(S, FormatSettings, DateStr, TimeStr);
  case PartsFound of  
    0: Result:=StrToDate('');
    1: if (Length(DateStr) > 0) then
         Result := StrToDate(DateStr, FormatSettings.ShortDateFormat,FormatSettings.DateSeparator)
       else
         Result := StrToTime(TimeStr, FormatSettings);
    2: Result := ComposeDateTime(StrTodate(DateStr,FormatSettings.ShortDateFormat,FormatSettings.DateSeparator),
                                  StrToTime(TimeStr,FormatSettings));
  end;
end;

function StrToDateTime(const s: AnsiString): TDateTime;
begin
  Result:=StrToDateTime(S,DefaultFormatSettings);
end;

function StrToDateTime(const s: ShortString; const FormatSettings : TFormatSettings): TDateTime;

var
  A : AnsiString;
begin
  A:=S;
  Result:=StrToDateTime(A,FormatSettings);
end;

{   FormatDateTime formats DateTime to the given format string FormatStr   }

function FormatDateTime(const FormatStr: string; DateTime: TDateTime; Options : TFormatDateTimeOptions = []): string;
begin
  DateTimeToString(Result, FormatStr, DateTime, DefaultFormatSettings,Options);
end;

function FormatDateTime(const FormatStr: string; DateTime: TDateTime; const FormatSettings: TFormatSettings; Options : TFormatDateTimeOptions = []): string;
begin
  DateTimeToString(Result, FormatStr, DateTime, FormatSettings,Options);
end;

{   DateTimeToString formats DateTime to the given format in FormatStr   }

procedure DateTimeToString(out Result: string; const FormatStr: string; 
  const DateTime: TDateTime; Options : TFormatDateTimeOptions = []);
begin
  DateTimeToString(Result, FormatStr, DateTime, DefaultFormatSettings, Options);
end;

procedure DateTimeToString(out Result: string; const FormatStr: string; const DateTime: TDateTime; 
  const FormatSettings: TFormatSettings; Options : TFormatDateTimeOptions = []);
var
  ResultLen: integer;
  ResultBuffer: array[0..255] of char;
  ResultCurrent: pchar;
{$if defined(win32) or defined(win64)}
  isEnable_E_Format : Boolean;
  isEnable_G_Format : Boolean;
  eastasiainited : boolean;

  procedure InitEastAsia;
  var     ALCID : LCID;
         PriLangID , SubLangID : Word;

  begin
    ALCID := GetThreadLocale;
    PriLangID := ALCID and $3FF;
    if (PriLangID>0) then
       SubLangID := (ALCID and $FFFF) shr 10
      else
        begin
          PriLangID := SysLocale.PriLangID;
          SubLangID := SysLocale.SubLangID;
        end;
    isEnable_E_Format := (PriLangID = LANG_JAPANESE)
                  or
                  (PriLangID = LANG_KOREAN)
                  or
                  ((PriLangID = LANG_CHINESE)
                   and
                   (SubLangID = SUBLANG_CHINESE_TRADITIONAL)
                  );
    isEnable_G_Format := (PriLangID = LANG_JAPANESE)
                  or
                  ((PriLangID = LANG_CHINESE)
                   and
                   (SubLangID = SUBLANG_CHINESE_TRADITIONAL)
                  );
    eastasiainited :=true;
  end;
{$endif win32 or win64}

  procedure StoreStr(Str: PChar; Len: Integer);
  begin
    if ResultLen + Len < SizeOf(ResultBuffer) then
    begin
      StrMove(ResultCurrent, Str, Len);
      ResultCurrent := ResultCurrent + Len;
      ResultLen := ResultLen + Len;
    end;
  end;

  procedure StoreString(const Str: string);
  var Len: integer;
  begin
   Len := Length(Str);
   if ResultLen + Len < SizeOf(ResultBuffer) then
     begin
       StrMove(ResultCurrent, pchar(Str), Len);
       ResultCurrent := ResultCurrent + Len;
       ResultLen := ResultLen + Len;
     end;
  end;

  procedure StoreInt(Value, Digits: Integer);
  var
    S: string[16];
    Len: integer;
  begin
    System.Str(Value:Digits, S);
    for Len := 1 to Length(S) do
    begin
      if S[Len] = ' ' then
        S[Len] := '0'
      else
        Break;
    end;
    StoreStr(pchar(@S[1]), Length(S));
  end ;

var
  Year, Month, Day, DayOfWeek, Hour, Minute, Second, MilliSecond: word;

  procedure StoreFormat(const FormatStr: string; Nesting: Integer; TimeFlag: Boolean);
  var
    Token, lastformattoken, prevlasttoken: char;
    FormatCurrent: pchar;
    FormatEnd: pchar;
    Count: integer;
    Clock12: boolean;
    P: pchar;
    tmp: integer;
    isInterval: Boolean;

  begin
    if Nesting > 1 then  // 0 is original string, 1 is included FormatString
      Exit;

    FormatCurrent := PChar(FormatStr);
    FormatEnd := FormatCurrent + Length(FormatStr);
    Clock12 := false;
    isInterval := false;
    P := FormatCurrent;
    // look for unquoted 12-hour clock token
    while P < FormatEnd do
    begin
      Token := P^;
      case Token of
        '''', '"':
        begin
          Inc(P);
          while (P < FormatEnd) and (P^ <> Token) do
            Inc(P);
        end;
        'A', 'a':
        begin
          if (StrLIComp(P, 'A/P', 3) = 0) or
             (StrLIComp(P, 'AMPM', 4) = 0) or
             (StrLIComp(P, 'AM/PM', 5) = 0) then
          begin
            Clock12 := true;
            break;
          end;
        end;
      end;  // case
      Inc(P);
    end ;
    token := #255;
    lastformattoken := ' ';
    prevlasttoken := 'H';
    while FormatCurrent < FormatEnd do
    begin
      Token := UpCase(FormatCurrent^);
      Count := 1;
      P := FormatCurrent + 1;
      case Token of
        '''', '"':
        begin
          while (P < FormatEnd) and (p^ <> Token) do
            Inc(P);
          Inc(P);
          Count := P - FormatCurrent;
          StoreStr(FormatCurrent + 1, Count - 2);
        end ;
        'A':
        begin
          if StrLIComp(FormatCurrent, 'AMPM', 4) = 0 then
          begin
            Count := 4;
            if Hour < 12 then
              StoreString(FormatSettings.TimeAMString)
            else
              StoreString(FormatSettings.TimePMString);
          end
          else if StrLIComp(FormatCurrent, 'AM/PM', 5) = 0 then
          begin
            Count := 5;
            if Hour < 12 then StoreStr(FormatCurrent, 2)
                         else StoreStr(FormatCurrent+3, 2);
          end
          else if StrLIComp(FormatCurrent, 'A/P', 3) = 0 then
          begin
            Count := 3;
            if Hour < 12 then StoreStr(FormatCurrent, 1)
                         else StoreStr(FormatCurrent+2, 1);
          end
          else
            raise EConvertError.Create('Illegal character in format string');
        end ;
        '/': StoreStr(@FormatSettings.DateSeparator, 1);
        ':': StoreStr(@FormatSettings.TimeSeparator, 1);
	'[': if (fdoInterval in Options) then isInterval := true else StoreStr(FormatCurrent, 1);
	']': if (fdoInterval in Options) then isInterval := false else StoreStr(FormatCurrent, 1);
        ' ', 'C', 'D', 'H', 'M', 'N', 'S', 'T', 'Y', 'Z', 'F' {$IFDEF MSWindows}, 'G', 'E'{$ENDIF MSWindows} :
        begin
          while (P < FormatEnd) and (UpCase(P^) = Token) do
            Inc(P);
          Count := P - FormatCurrent;
          case Token of
            ' ': StoreStr(FormatCurrent, Count);
            'Y': begin
              if Count > 2 then
                StoreInt(Year, 4)
              else
                StoreInt(Year mod 100, 2);
            end;
            'M': begin
	      if isInterval and ((prevlasttoken = 'H') or TimeFlag) then
	        StoreInt(Minute + (Hour + trunc(abs(DateTime))*24)*60, 0)
	      else
              if (lastformattoken = 'H') or TimeFlag then
              begin
                if Count = 1 then
                  StoreInt(Minute, 0)
                else
                  StoreInt(Minute, 2);
              end
              else
              begin
                case Count of
                  1: StoreInt(Month, 0);
                  2: StoreInt(Month, 2);
                  3: StoreString(FormatSettings.ShortMonthNames[Month]);
                else
                  StoreString(FormatSettings.LongMonthNames[Month]);
                end;
              end;
            end;
            'D': begin
              case Count of
                1: StoreInt(Day, 0);
                2: StoreInt(Day, 2);
                3: StoreString(FormatSettings.ShortDayNames[DayOfWeek]);
                4: StoreString(FormatSettings.LongDayNames[DayOfWeek]);
                5: StoreFormat(FormatSettings.ShortDateFormat, Nesting+1, False);
              else
                StoreFormat(FormatSettings.LongDateFormat, Nesting+1, False);
              end ;
            end ;
            'H': 
	      if isInterval then
	        StoreInt(Hour + trunc(abs(DateTime))*24, 0)
	      else
	      if Clock12 then
              begin
                tmp := hour mod 12;
                if tmp=0 then tmp:=12;
                if Count = 1 then
                  StoreInt(tmp, 0)
                else
                  StoreInt(tmp, 2);
              end
              else begin
                if Count = 1 then
		  StoreInt(Hour, 0)
                else
                  StoreInt(Hour, 2);
              end;
            'N': if isInterval then
	           StoreInt(Minute + (Hour + trunc(abs(DateTime))*24)*60, 0)
		 else
		 if Count = 1 then
                   StoreInt(Minute, 0)
                 else
                   StoreInt(Minute, 2);
            'S': if isInterval then
	           StoreInt(Second + (Minute + (Hour + trunc(abs(DateTime))*24)*60)*60, 0)
	         else 
	         if Count = 1 then
                   StoreInt(Second, 0)
                 else
                   StoreInt(Second, 2);
            'Z': if Count = 1 then
                   StoreInt(MilliSecond, 0)
                 else
		   StoreInt(MilliSecond, 3);
            'T': if Count = 1 then
		   StoreFormat(FormatSettings.ShortTimeFormat, Nesting+1, True)
                 else
	           StoreFormat(FormatSettings.LongTimeFormat, Nesting+1, True);
            'C': begin
                   StoreFormat(FormatSettings.ShortDateFormat, Nesting+1, False);
                   if (Hour<>0) or (Minute<>0) or (Second<>0) then
                     begin
                      StoreString(' ');
                      StoreFormat(FormatSettings.LongTimeFormat, Nesting+1, True);
                     end;
                 end;
            'F': begin
                   StoreFormat(FormatSettings.ShortDateFormat, Nesting+1, False);
                   StoreString(' ');
                   StoreFormat(FormatSettings.LongTimeFormat, Nesting+1, True);
                 end;
{$if defined(win32) or defined(win64)}
            'E':
               begin
                 if not Eastasiainited then InitEastAsia;
                 if Not(isEnable_E_Format) then StoreStr(FormatCurrent, 1)
                  else
                   begin
                     while (P < FormatEnd) and (UpCase(P^) = Token) do
                     P := P + 1;
                     Count := P - FormatCurrent;
                     StoreString(ConvertEraYearString(Count,Year,Month,Day));
                   end;
		 prevlasttoken := lastformattoken;  
                 lastformattoken:=token;
               end;
             'G':
               begin
                 if not Eastasiainited then InitEastAsia;
                 if Not(isEnable_G_Format) then StoreStr(FormatCurrent, 1)
                  else
                   begin
                     while (P < FormatEnd) and (UpCase(P^) = Token) do
                     P := P + 1;
                     Count := P - FormatCurrent;
                     StoreString(ConvertEraString(Count,Year,Month,Day));
                   end;
		 prevlasttoken := lastformattoken;
                 lastformattoken:=token;
               end;
{$endif win32 or win64}
          end;
	  prevlasttoken := lastformattoken;
          lastformattoken := token;
        end;
        else
          StoreStr(@Token, 1);
      end ;
      Inc(FormatCurrent, Count);
    end;
  end;

begin
{$if defined(win32) or defined(win64)}
  eastasiainited:=false;
{$endif win32 or win64}
  DecodeDateFully(DateTime, Year, Month, Day, DayOfWeek);
  DecodeTime(DateTime, Hour, Minute, Second, MilliSecond);
  ResultLen := 0;
  ResultCurrent := @ResultBuffer[0];
  if FormatStr <> '' then
    StoreFormat(FormatStr, 0, False)
  else
    StoreFormat('C', 0, False);
  ResultBuffer[ResultLen] := #0;
  result := StrPas(@ResultBuffer[0]);
end ;


Function DateTimeToFileDate(DateTime : TDateTime) : Int64;

Var YY,MM,DD,H,m,s,msec : Word;

begin
  Decodedate (DateTime,YY,MM,DD);
  DecodeTime (DateTime,h,m,s,msec);
{$ifndef unix}
  If (YY<1980) or (YY>2099) then
    Result:=0
  else
    begin
    Result:=(s shr 1) or (m shl 5) or (h shl 11);
    Result:=Result or longint(DD shl 16 or (MM shl 21) or (word(YY-1980) shl 25));
    end;
{$else unix}
  Result:=LocalToEpoch(yy,mm,dd,h,m,s);
{$endif unix}
end;

function CurrentYear: Word;
var
  SysTime: TSystemTime;
begin
  GetLocalTime(SysTime);
  Result := SysTime.Year;
end;

Function FileDateToDateTime (Filedate : Int64) : TDateTime;

{$ifndef unix}
Var Date,Time : Word;

begin
  Date:=FileDate shr 16;
  Time:=FileDate and $ffff;
  Result:=ComposeDateTime(EncodeDate((Date shr 9) + 1980,(Date shr 5) and 15, Date and 31),
          EncodeTime(Time shr 11, (Time shr 5) and 63, (Time and 31) shl 1,0));
end;
{$else unix}
var
  y, mon, d, h, min, s: word;
begin
  EpochToLocal(FileDate,y,mon,d,h,min,s);
  Result:=ComposeDateTime(EncodeDate(y,mon,d),EncodeTime(h,min,s,0));
end;
{$endif unix}

function TryStrToDate(const S: ShortString; out Value: TDateTime): Boolean;
begin
    result := TryStrToDate(S, Value, #0);
end;

function TryStrToDate(const S: ShortString; out Value: TDateTime;
                    const useformat : string; separator : char = #0): Boolean;

Var
  Msg : Ansistring;

begin
  // S[1] always exists for shortstring. Length 0 will trigger an error.
  Value:=IntStrToDate(Msg,@S[1],Length(S),useformat,defaultformatsettings,separator);
  Result:=(Msg='');
end;

function TryStrToDate(const S: AnsiString; out Value: TDateTime;
                    const useformat : string; separator : char = #0): Boolean;

Var
  Msg : Ansistring;

begin
  Result:=Length(S)<>0;
  If Result then
    begin
    Value:=IntStrToDate(Msg,PChar(S),Length(S),useformat,DefaultFormatSettings,Separator);
    Result:=(Msg='');
    end;
end;

function TryStrToDate(const S: ShortString; out Value: TDateTime; separator : char): Boolean;
begin
  Result:=TryStrToDate(S,Value,DefaultFormatSettings.ShortDateFormat,Separator);
end;


function TryStrToDate(const S: AnsiString; out Value: TDateTime): Boolean;
begin
  Result:=TryStrToDate(S,Value,DefaultFormatSettings.ShortDateFormat,#0);
end;

function TryStrToDate(const S: AnsiString; out Value: TDateTime; separator : char): Boolean;

begin
  Result:=TryStrToDate(S,Value,DefaultFormatSettings.ShortDateFormat,Separator);
end;

function TryStrToDate(const S: string; out Value: TDateTime; const FormatSettings: TFormatSettings): Boolean;
Var
  Msg : Ansistring;

begin
  Result:=Length(S)<>0;
  If Result then
    begin
      Value:=IntStrToDate(Msg,PChar(S),Length(S),FormatSettings.ShortDateFormat,FormatSettings,#0);
      Result:=(Msg='');
    end;
end;

function TryStrToTime(const S: ShortString; out Value: TDateTime; separator : char): Boolean;

Var
  Msg : AnsiString;
begin
  // S[1] always exists for shortstring. Length 0 will trigger an error.
  Value:=IntStrToTime(Msg,@S[1],Length(S),DefaultFormatSettings,Separator);
  result:=(Msg='');
end;

function TryStrToTime(const S: ShortString; out Value: TDateTime): Boolean;
begin
  Result := TryStrToTime(S,Value,#0);
end;

function TryStrToTime(const S: AnsiString; out Value: TDateTime; separator : char): Boolean;
Var
  Msg : AnsiString;
begin
  Result:=Length(S)<>0;
  If Result then
    begin
      Value:=IntStrToTime(Msg,PChar(S),Length(S),DefaultFormatSettings,Separator);
      Result:=(Msg='');
    end;
end;

function TryStrToTime(const S: AnsiString; out Value: TDateTime): Boolean;
begin
    result := TryStrToTime(S,Value,#0);
end;

function TryStrToTime(const S: string; out Value: TDateTime; const FormatSettings: TFormatSettings): Boolean;
Var msg : AnsiString;
begin
  Result:=Length(S)<>0;
  If Result then
    begin
      Value:=IntStrToTime(Msg,PChar(S),Length(S),FormatSettings,#0);
      Result:=(Msg='');
    end;
end;

function TryStrToDateTime(const S: ShortString; out Value: TDateTime): Boolean;
  begin
    result := TryStrToDateTime(S, Value, DefaultFormatSettings);
  end;

function TryStrToDateTime(const S: AnsiString; out Value: TDateTime): Boolean;
  begin
    result := TryStrToDateTime(S, Value, DefaultFormatSettings);
  end;

function TryStrToDateTime(const S: string; out Value: TDateTime; const FormatSettings: TFormatSettings): Boolean;
var
  I: integer;
  dtdate, dttime :TDateTime;
begin
  result:=false;
  I:=Pos(FormatSettings.TimeSeparator,S);
  If (I>0) then
    begin
      While (I>0) and (S[I]<>' ') do
        Dec(I);
      If I>0 then
        begin
          if not TryStrToDate(Copy(S,1,I-1),dtdate,Formatsettings) then
            exit;
          if not TryStrToTime(Copy(S,i+1, Length(S)-i),dttime,Formatsettings) then
            exit;
          Value:=ComposeDateTime(dtdate,dttime);
          result:=true;
        end
      else
         result:=TryStrToTime(s,Value,Formatsettings);
    end
  else
    result:=TryStrToDate(s,Value,Formatsettings);
end;

function StrToDateDef(const S: ShortString; const Defvalue : TDateTime): TDateTime;
begin
   result := StrToDateDef(S,DefValue,#0);
end;

function StrToTimeDef(const S: ShortString; const Defvalue : TDateTime): TDateTime;
begin
   result := StrToTimeDef(S,DefValue,#0);
end;

function StrToDateTimeDef(const S: ShortString; const Defvalue : TDateTime): TDateTime;
begin
  Result:=StrToDateTimeDef(S,DefValue,DefaultFormatSettings);
end;

function StrToDateTimeDef(const S: AnsiString; const Defvalue : TDateTime; const FormatSettings: TFormatSettings): TDateTime;
begin
  if not TryStrToDateTime(s,Result,FormatSettings) Then
    result:=defvalue;
end;

function StrToDateDef(const S: ShortString; const Defvalue : TDateTime; separator : char): TDateTime;
begin
  if not TryStrToDate(s,Result, separator) Then
    result:=defvalue;
end;

function StrToTimeDef(const S: ShortString; const Defvalue : TDateTime; separator : char): TDateTime;
begin
  if not TryStrToTime(s,Result, separator) Then
    result:=defvalue;
end;

function StrToDateDef(const S: AnsiString; const Defvalue : TDateTime): TDateTime;
begin
   result := StrToDateDef(S,DefValue,#0);
end;

function StrToTimeDef(const S: AnsiString; const Defvalue : TDateTime): TDateTime;
begin
   result := StrToTimeDef(S,DefValue,#0);
end;

function StrToDateTimeDef(const S: AnsiString; const Defvalue : TDateTime): TDateTime;
begin
  if not TryStrToDateTime(s,Result) Then
    result:=defvalue;
end;

function StrToDateDef(const S: AnsiString; const Defvalue : TDateTime; separator : char): TDateTime;
begin
  if not TryStrToDate(s,Result, separator) Then
    result:=defvalue;
end;

function StrToTimeDef(const S: AnsiString; const Defvalue : TDateTime; separator : char): TDateTime;
begin
  if not TryStrToTime(s,Result, separator) Then
    result:=defvalue;
end;

procedure ReplaceTime(var dati:TDateTime; NewTime : TDateTime);inline;
begin
  dati:= ComposeDateTime(dati, newtime);
end;

procedure ReplaceDate(var DateTime: TDateTime; const NewDate: TDateTime); inline;
var
  tmp : TDateTime;
begin
  tmp:=NewDate;
  ReplaceTime(tmp,DateTime);
  DateTime:=tmp;
end;

{$IFNDEF HAS_LOCALTIMEZONEOFFSET}
Function GetLocalTimeOffset : Integer;

begin
  Result:=0;
end;
{$ENDIF}
