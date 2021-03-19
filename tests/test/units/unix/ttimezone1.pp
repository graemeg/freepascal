{ %target=linux,freebsd,openbsd,aix,darwin,netbsd }
uses
  BaseUnix,unix;

function LocalTimeIsDefined(Year, Month, Day, Hour, Minute, Second: word; const UTC: Boolean): Boolean;
var
  UnixTime: Int64;
  lTZInfo: TTZInfo;
begin
  lTZInfo:=Default(TTZInfo);
  UnixTime:=UniversalToEpoch(Year, Month, Day, Hour, Minute, Second);
  Result := GetLocalTimezone(UnixTime,UTC,lTZInfo);
end;

function GetOffset(Year, Month, Day, Hour, Minute, Second: word; const UTC: Boolean): Integer;
var
  UnixTime: Int64;
  lTZInfo: TTZInfo;
begin
  lTZInfo:=Default(TTZInfo);
  UnixTime:=UniversalToEpoch(Year, Month, Day, Hour, Minute, Second);
  if not GetLocalTimezone(UnixTime,UTC,lTZInfo) then
    Halt(1);
  if not UTC then
    UnixTime := UnixTime-lTZInfo.seconds;
  if not ((lTZInfo.validsince<=UnixTime) and (UnixTime<lTZInfo.validuntil)) then
    Halt(2);
  if lTZInfo.seconds mod 3600<>0 then
    Halt(3);
  Result:=lTZInfo.seconds div 3600;
end;

begin
  // check against Europe/Vienna file
  if not ReadTimezoneFile('Europe/Vienna') then
  begin
    writeln('Europe/Vienna timezone file not found');
    halt(10);
  end;

  if GetOffset(2019, 03, 31, 1, 59, 0, False)<>1 then Halt(11);
  // 2019-03-31 02:00-03:00 CET is not defined
  if LocalTimeIsDefined(2019, 03, 31, 2, 30, 0, False) then Halt(19);
  if GetOffset(2019, 03, 31, 3, 0, 0, False)<>2 then Halt(12);

  if GetOffset(2019, 10, 27, 1, 59, 0, False)<>2 then Halt(13);
  // 2019-10-27 02:00-03:00 CET is ambiguos, therefore do not check
  if GetOffset(2019, 10, 27, 3, 0, 0, False)<>1 then Halt(14);

  if GetOffset(2019, 03, 31, 0, 59, 0, True)<>1 then Halt(15);
  if GetOffset(2019, 03, 31, 1, 0, 0, True)<>2 then Halt(16);

  if GetOffset(2019, 10, 27, 0, 59, 0, True)<>2 then Halt(17);
  if GetOffset(2019, 10, 27, 1, 0, 0, True)<>1 then Halt(18);


  // check against Europe/Moscow file
  if not ReadTimezoneFile('Europe/Moscow') then
  begin
    writeln('Europe/Moscow timezone file not found');
    halt(20);
  end;

  {
    https://en.wikipedia.org/wiki/Time_in_Russia

    Daylight saving time was re-introduced in the USSR in 1981, beginning on 1 April and ending on 1 October each year,
    until mid-1984, when the USSR began following European daylight saving time rules, moving clocks forward one hour
    at 02:00 local standard time on the last Sunday in March, and back one hour at 03:00 local daylight time on the last
    Sunday in September until 1995, after which the change back occurred on the last Sunday in October.

    On 27 March 2011, clocks were advanced as usual, but they did not go back on 30 October 2011, effectively making
    Moscow Time UTC+04:00 permanently. On 26 October 2014, following another change in the law, the clocks in most
    of the country were moved back one hour, but summer Daylight Time was not reintroduced; Moscow Time returned
    to UTC+03:00 permanently.
  }

  if GetOffset(1994, 03, 26, 0, 0, 0, True)<>3 then Halt(21);
  if GetOffset(1994, 03, 27, 0, 0, 0, True)<>4 then Halt(22);
  if GetOffset(1994, 09, 24, 0, 0, 0, True)<>4 then Halt(23);
  if GetOffset(1994, 09, 25, 0, 0, 0, True)<>3 then Halt(24);

  if GetOffset(1996, 03, 30, 0, 0, 0, True)<>3 then Halt(25);
  if GetOffset(1996, 03, 31, 0, 0, 0, True)<>4 then Halt(26);
  if GetOffset(1996, 10, 26, 0, 0, 0, True)<>4 then Halt(27);
  if GetOffset(1996, 10, 27, 0, 0, 0, True)<>3 then Halt(28);

  if GetOffset(2011, 03, 26, 0, 0, 0, True)<>3 then Halt(29);
  if GetOffset(2011, 03, 27, 0, 0, 0, True)<>4 then Halt(30);
  if GetOffset(2011, 09, 01, 0, 0, 0, True)<>4 then Halt(31);
  if GetOffset(2011, 11, 01, 0, 0, 0, True)<>4 then Halt(32);

  if GetOffset(2012, 06, 01, 0, 0, 0, True)<>4 then Halt(33);

  if GetOffset(2014, 10, 25, 0, 0, 0, True)<>4 then Halt(34);
  if GetOffset(2014, 10, 26, 0, 0, 0, True)<>3 then Halt(35);

  if GetOffset(2021, 03, 31, 0, 0, 0, True)<>3 then Halt(36);

  writeln('ok');
end.
