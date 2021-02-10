program test_scandatetime_ampm;
{$mode objfpc}
{$h+}
uses
  SysUtils, DateUtils, StrUtils;

Var
 ErrCount : Integer;

function SameDateTime(dt1, dt2: TDateTime): Boolean;
const
  EPS = 1/(24*60*60*100*10);  // 0.1 ms
begin
  Result := abs(dt1 - dt2) < EPS;
end;

procedure Test(AExpected: TDateTime; AFormatStr, ADateTimeStr: String; NeedError : Boolean = False);
var
  dt: TDateTime;
begin
  Write(PadRight(ADateTimeStr, 36), ' --->   ');
  Write(PadRight(FormatDateTime('yyyy-mm-dd hh:nn:ss.zzz', dt), 25));
  try
    dt := ScanDateTime(AFormatStr, ADateTimeStr);
    if dt = AExpected then WriteLn('OK') else 
     begin
     Inc(ErrCount);
     WriteLn('ERROR');
     end;
  except on E:Exception do
    begin
    if not NeedError then
      inc(errcount);
    WriteLn('ERROR: ', E.Message);
    end;
  end;
end;

begin
  errCount:=0;
  WriteLn('Using current format settings...');
  Test(EncodeDateTime(2014, 4, 2, 0, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 12:00 am');
  Test(EncodeDateTime(2014, 4, 2, 0, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 12:00 AM');
  Test(EncodeDateTime(2014, 4, 2, 0, 1, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 12:01 am');
  Test(EncodeDateTime(2014, 4, 2, 1, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 01:00 am');
  Test(EncodeDateTime(2014, 4, 2,11, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 11:00 am');
  Test(EncodeDateTime(2014, 4, 2,11,59, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 11:59 am');
  Test(EncodeDateTime(2014, 4, 2,11,59,59,999), 'mmmm dd??, yyyy, hh:nn:ss.zzz am/pm', 'April 2nd, 2014, 11:59:59.999 am');
  Test(EncodeDateTime(2014, 4, 2,12, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 12:00 pm');
  Test(EncodeDateTime(2014, 4, 2,12, 0, 0,  1), 'mmmm dd??, yyyy, hh:nn:ss.zzz am/pm', 'April 2nd, 2014, 12:00:00.001 pm');
  Test(EncodeDateTime(2014, 4, 2,13, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 01:00 pm');
  Test(EncodeDateTime(2014, 4, 2,13, 1, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 01:01 pm');
  Test(EncodeDateTime(2014, 4, 2,23, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 11:00 pm');
  Test(EncodeDateTime(2014, 4, 2,23,59,59,999), 'mmmm dd??, yyyy, hh:nn:ss.zzz am/pm', 'April 2nd, 2014, 11:59:59.999 pm');

  WriteLn;

  Test(EncodeDateTime(2014, 4, 2, 0, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 12:00 a');
  Test(EncodeDateTime(2014, 4, 2, 0, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 12:00 A');
  Test(EncodeDateTime(2014, 4, 2, 0, 1, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 12:01 a');
  Test(EncodeDateTime(2014, 4, 2, 1, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 01:00 a');
  Test(EncodeDateTime(2014, 4, 2,11, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 11:00 a');
  Test(EncodeDateTime(2014, 4, 2,11,59, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 11:59 a');
  Test(EncodeDateTime(2014, 4, 2,11,59,59,999), 'mmmm dd??, yyyy, hh:nn:ss.zzz a/p', 'April 2nd, 2014, 11:59:59.999 a');
  Test(EncodeDateTime(2014, 4, 2,12, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 12:00 p');
  Test(EncodeDateTime(2014, 4, 2,12, 0, 0,  1), 'mmmm dd??, yyyy, hh:nn:ss.zzz a/p', 'April 2nd, 2014, 12:00:00.001 p');
  Test(EncodeDateTime(2014, 4, 2,13, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 01:00 p');
  Test(EncodeDateTime(2014, 4, 2,13, 1, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 01:01 p');
  Test(EncodeDateTime(2014, 4, 2,23, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn a/p', 'April 2nd, 2014, 11:00 p');
  Test(EncodeDateTime(2014, 4, 2,23,59,59,999), 'mmmm dd??, yyyy, hh:nn:ss.zzz a/p', 'April 2nd, 2014, 11:59:59.999 p');

  WriteLn;

  FormatSettings.TimeAMString := 'vorm';
  FormatSettings.TimePMString := 'nachm';
  WriteLn('Using modified format settings with ampm=', FormatSettings.TimeAMString, '/', FormatSettings.TimePMString);
  Test(EncodeDateTime(2014, 4, 2, 0, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 12:00 vorm');
  Test(EncodeDateTime(2014, 4, 2, 0, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 12:00 VORM');
  Test(EncodeDateTime(2014, 4, 2, 0, 1, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 12:01 vorm');
  Test(EncodeDateTime(2014, 4, 2, 1, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 01:00 vorm');
  Test(EncodeDateTime(2014, 4, 2,11, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 11:00 vorm');
  Test(EncodeDateTime(2014, 4, 2,11,59, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 11:59 vorm');
  Test(EncodeDateTime(2014, 4, 2,11,59,59,999), 'mmmm dd??, yyyy, hh:nn:ss.zzz ampm', 'April 2nd, 2014, 11:59:59.999 vorm');
  Test(EncodeDateTime(2014, 4, 2,12, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 12:00 nachm');
  Test(EncodeDateTime(2014, 4, 2,12, 0, 0,  1), 'mmmm dd??, yyyy, hh:nn:ss.zzz ampm', 'April 2nd, 2014, 12:00:00.001 nachm');
  Test(EncodeDateTime(2014, 4, 2,13, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 01:00 nachm');
  Test(EncodeDateTime(2014, 4, 2,13, 1, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 01:01 nachm');
  Test(EncodeDateTime(2014, 4, 2,23, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 2nd, 2014, 11:00 nachm');
  Test(EncodeDateTime(2014, 4, 2,23,59,59,999), 'mmmm dd??, yyyy, hh:nn:ss.zzz ampm', 'April 2nd, 2014, 11:59:59.999 nachm');
  Test(EncodeDateTime(2014, 4, 3,12, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn ampm', 'April 3rd, 2014, 12:00 nachm');
  WriteLn('The next test should raise an exception.');

  try
    Test(EncodeDateTime(2014, 4, 2,13, 0, 0,  0), 'mmmm dd??, yyyy, hh:nn am/pm', 'April 2nd, 2014, 13:00 pm',True);
   
  except on E:Exception do
    begin
    WriteLn('OK, exception received: ', E.Message);
    end;
  end;

  WriteLn;
  WriteLn('Test complete. Press RETURN to exit.');
  Halt(Ord(errcount>0));
//  ReadLn;
end.
