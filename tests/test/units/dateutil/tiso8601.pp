program tiso8601;

uses
  SysUtils, DateUtils;

const
  sd6 = '2021-05-22T13:57:49.191021Z';
  sd3 = '2021-05-22T13:57:49.191Z';
  sd2 = '2021-05-22T13:57:49.19Z';
  sd1 = '2021-05-22T13:57:49.1Z';

  sc6 = '2021-05-22T13:57:49,191021Z';
  sc3 = '2021-05-22T13:57:49,191Z';
  sc2 = '2021-05-22T13:57:49,19Z';
  sc1 = '2021-05-22T13:57:49,1Z';

var
  dt1, dt2, dt3, dt6: TDateTime;
  hasErrors : boolean;

  procedure Test(s: String);
  var
    dt: TDateTime;
  begin
    Write(s:30, ' ---> ');
    try
      dt := ISO8601ToDate(s, true);
      WriteLn(dt:0:15);
    except
      WriteLn('ERROR');
      hasErrors:=True;
    end;
  end;

begin
  HasErrors:=False;
  Test(sd1);
  Test(sd2);
  Test(sd3);
  Test(sd6);

  Test(sc1);
  Test(sc2);
  Test(sc3);
  Test(sc6);
  Halt(Ord(HasErrors));
end.
