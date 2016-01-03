{ %interactive }

uses
{$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
=======
  cwstring,
>>>>>>> graemeg/fixes_2_2
=======
  cwstring,
>>>>>>> origin/fixes_2_2
  clocale,
{$endif}
  SysUtils;

procedure PrintSettings;
var
 i: integer;
begin
  Writeln('Month names:');
  for i := 1 to 12 do
    Writeln('month ',i:5,' ',shortmonthnames[i]:25,' - ',longmonthnames[i]);
  Writeln;
  Writeln('Day names  :');
  for i := 1 to 7 do
     writeln('short day ',i:5,' ',shortdaynames[i]:25,' - ',longdaynames[i]);

  writeln('Dateseparator : ', dateseparator);
  {$ifdef localedebug}
  writeln('orgshortdate  : ', orgformatsettings.shortdateformat);
  {$endif}
  writeln('short date    : ', shortdateformat);
  {$ifdef localedebug}
  writeln('orglongdate   : ', orgformatsettings.longdateformat);
  {$endif}
  writeln('long  date    : ', longdateformat);
 { Time stuff }
  writeln('TimeSeparator : ', timeseparator);
  writeln('TimeAMstring  : ', timeamstring);
  writeln('TimePMstring  : ', timepmstring);  
  {$ifdef localedebug}
  writeln('orgshorttime  : ', orgformatsettings.shorttimeformat);
  {$endif}

 // No support for 12 hour stuff at the moment...
  writeln('short time    : ', shorttimeformat);
  {$ifdef localedebug}
  writeln('orglongtime   : ', orgformatsettings.longtimeformat);
  {$endif}
  writeln('long  time    : ', longtimeformat);

 { Currency stuff }

  {$ifdef localedebug}
  writeln('currency1     : ', orgformatsettings.currencystring1);
  writeln('currency2     : ', orgformatsettings.currencystring2);
  {$endif}

  writeln('currencystring  : ', currencystring);

  writeln('currencyformat  : ', currencyformat);
  writeln('negcurrformat   : ', negcurrformat);

  writeln('decimalseparator : ', decimalseparator);
  writeln('thousandseparator: ', thousandseparator);
  writeln('currencydecimals : ', currencydecimals);
end;

begin
  printsettings;
end.
