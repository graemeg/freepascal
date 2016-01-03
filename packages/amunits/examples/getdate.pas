Program GetDate;

{ GetDate v1.0 1995 by Andreas Tetzl }
{ Public Domain }

{
  Translated to fpc pascal
  19 Mar 2001.

  nils.sjoholm@mailbox.swipnet.se
}

uses amigados, strings;

const template : pchar = 'Format/K,Help/S';

      version : pchar = '$VER: GetDate 1.0 (21.2.95)';

VAR DS : tDateStamp;
    DT : tDateTime;
    rda : pRDArgs;
    WeekDay, Date, Time, hours, mins, secs, day, month, year : pchar;
    vec : Array[0..1] of longint;
    i : longint;
    LFormat : pchar;

Procedure PrintFormat;
VAR Str : string;
    tmp : string;
Begin
 Str := strpas(LFormat);
 tmp := '';
 For i:=1 to length(Str) do
  begin

   If Str[i]='%' then
    Begin
     Case UpCase(Str[i+1]) of
      ('D') : tmp := tmp + strpas(Date);
      ('W') : tmp := tmp + strpas(WeekDay);
      ('T') : tmp := tmp + strpas(Time);
      ('H') : tmp := tmp + strpas(hours);
      ('M') : tmp := tmp + strpas(Mins);
      ('S') : tmp := tmp + strpas(Secs);
      ('A') : tmp := tmp + strpas(Day);
      ('O') : tmp := tmp + strpas(Month);
      ('Y') : tmp := tmp + strpas(Year);
     end;
     i:=i+1;
    end
   else
    tmp := tmp + Str[i];
  end;
 Writeln(tmp);
end;

Procedure Help;
Begin
 Writeln(#10'GetDate v1.0 1995 by Andreas Tetzl');
 Writeln('Public Domain'#10);
 Writeln('How to use the placeholders for Format:'#10);
 Writeln(' %d : Datum');
 Writeln(' %w : Weekday');
 Writeln(' %t : Time with Hour, Minutes and Seconds');
 Writeln(' %h : Hour');
 Writeln(' %m : Minutes');
 Writeln(' %s : Seconds');
 Writeln(' %a : Day');
 Writeln(' %o : Month');
 Writeln(' %y : Year'#10);
 Exit;
end;

begin
 For i:=0 to 1 do Vec[i]:=0;

 rda:=ReadArgs(Template,@vec,NIL);
 If rda=NIL then
  Begin
   If PrintFault(IoErr,NIL) then;
   halt(10);
  end;

 LFormat:=StrAlloc(100);

 If StrComp(pointer(vec[0]),pchar('')) <> 0 then StrCopy(LFormat,pointer(vec[0])) else LFormat:=NIL;


 If vec[1]<>0 then Help;

 WeekDay:=StrAlloc(LEN_DATSTRING);
 Date:=StrAlloc(LEN_DATSTRING);
 Time:=StrAlloc(LEN_DATSTRING);
 Hours:=StrAlloc(10);
 Mins:=StrAlloc(10);
 Secs:=StrAlloc(10);
 Day:=StrAlloc(10);
 Month:=StrAlloc(10);
 Year:=StrAlloc(10);

 DateStamp(pDateStamp(@DS));
 DT.dat_Stamp:=DS;
 DT.dat_Format:=Format_DOS;
 DT.dat_StrDay:=WeekDay;
 DT.dat_StrDate:=Date;
 DT.dat_StrTime:=Time;
 If DateToStr(@DT) then begin

 StrlCopy(hours,Time,2);

 StrlCopy(Mins,addr(Time[3]),2);
 StrlCopy(Secs,addr(Time[6]),2);
 StrlCopy(Day,Date,2);
 StrlCopy(Month,addr(Date[3]),3);
 StrlCopy(Year,addr(Date[7]),2);

 { In den deutschen Locale-Strings von OS3.0 scheint ein Fehler zu sein. }
 { Am Datums-String ist hinten noch ein Leerzeichen, also '16-Feb-95 '.  }
 { Hier wird geprüft, ob das letzte Zeichen ein Leerzeichen ist.         }
 { Das Leerzeichen wird dann durch '\0' (Stringende) ersetzt.            }
 If Date[StrLen(Date)-1]=' ' then Date[StrLen(Date)-1]:=#0;
end;
 If LFormat=NIL then
  Writeln(WeekDay,' ',Date,' ',Time)
 else
  PrintFormat;

 StrDispose(LFormat);
 StrDispose(WeekDay);
 StrDispose(date);
 StrDispose(Time);
 StrDispose(hours);
 StrDispose(mins);
 StrDispose(secs);
 StrDispose(Day);
 StrDispose(Month);
 StrDispose(Year);
end.
