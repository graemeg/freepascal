<<<<<<< HEAD
<<<<<<< HEAD
{%delfiles=texec1}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$mode objfpc}
{$h+}

Uses Sysutils;

var cmd,cmdline : String;
    i           : Longint;

begin
{$ifdef unix}
  cmd:='./texec1';
{$else}
  cmd:='texec1.exe';
{$endif}
  cmdline:='';
  for i:=0 to 10 do
   cmdline:=cmdline+'-Fu/usr/local/lib/fpc/1.0.10/units/freebsd/rtl/* ';

  if ExecuteProcess(cmd,cmdline)<>0 Then
    halt(1);
  // test illegal command
  try
    ExecuteProcess('afsdfdas',cmdline)
  except
    // unknown command should raise an exception
    halt(0);
  end;
  // we should never get here
  halt(1);
end.
