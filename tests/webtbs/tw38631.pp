{$mode objfpc}
program msec_test1;
uses sysutils;

var
  D: TDateTime;
  T, T1, T2: TTimeStamp;
  MS: Comp;
begin
  D:=EncodeDate(2021, 03, 16) + EncodeTime(14, 02, 15, 1);
  WriteLn('DATE: ', DateTimeToStr(D));

  T:=DateTimeToTimeStamp(D);
  WriteLn(' T.Date=',T.Date,'   T.Time=', T.Time);
  MS:=TimeStampToMSecs(T);
  T1:=MSecsToTimeStamp(MS);
  WriteLn('T1.Date=',T1.Date,'  T1.Time=', T1.Time);
  
  WriteLn('DATE1: ', DateTimeToStr(TimeStampToDateTime(T1)));
  if TimeStampToDateTime(T1)<>D then
    halt(1);
  writeln('ok')
end.
