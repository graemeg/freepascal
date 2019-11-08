uses
  bench,sysutils
{$ifdef Unix}
  ,unix
{$endif Unix}
  ;
var
  i : longint;
  command,
  ps : ansistring;
  eticks,
  sticks : int64;
  paramoffset,
  runs,
  code,
  ProcessExitCode : Integer;
begin
  ps:='';
  paramoffset:=0;
  runs:=1;
  if paramcount>0 then
    begin
      if paramstr(1)='-n' then
         begin
           val(paramstr(2),runs,code);
           if code<>0 then
              begin
                writeln('Illegal parameter');
                halt(1);
              end;
            paramoffset:=2;
         end;
      for i:=2+paramoffset to paramcount do
        ps:=ps+' "'+paramstr(i)+'"';
      sticks:=GetMicroSTicks;
      command:=paramstr(1+paramoffset);
      for i:=1 to runs do         
{$ifdef Unix}
        ProcessExitCode:=fpsystem(command+' '+ps);
{$else Unix}
        ProcessExitCode:=ExecuteProcess(command,ps);
{$endif Unix}
      eticks:=GetMicroSTicks;
      write(stderr,(eticks-sticks)/1000:0:3,' ms');
      if runs>1 then
        write(stderr,' (avg: ',(eticks-sticks)/runs/1000:0:3,' ms)');
      writeln(stderr);
      halt(ProcessExitCode);
    end;
end.
