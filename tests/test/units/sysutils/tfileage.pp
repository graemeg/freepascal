uses
  sysutils;
begin
  if 3600*24*(now()-FileDateToDateTime(FileAge(paramstr(0))))>7200 then
    begin
      writeln('FileAge returns: ',FileDateToDateTime(FileAge(paramstr(0))));
      writeln('Compilation time and run time differ too much, SysUtils.FileAge buggy?');
      halt(1);
    end;
end.
