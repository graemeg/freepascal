<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %target=linux,darwin,solaris,freebsd,haiku,beos,aix }
=======
{ %target=linux,darwin,solaris,freebsd,haiku,beos }
>>>>>>> graemeg/cpstrnew
=======
{ %target=linux,darwin,solaris,freebsd,haiku,beos }
>>>>>>> graemeg/cpstrnew
=======
{ %target=linux,darwin,solaris,freebsd,haiku,beos }
>>>>>>> graemeg/cpstrnew
=======
{ %target=linux,darwin,solaris,freebsd,haiku,beos }
>>>>>>> origin/cpstrnew
=======
{ %target=linux,darwin,solaris,freebsd,haiku,beos }
>>>>>>> origin/fixes_2.4

program TestSignal;

{$MODE objfpc}

uses
  BaseUnix,
  sysutils;

var
  Ok: Boolean;

procedure signal_handler(sig: LongInt); cdecl;
begin
  Writeln('4');
  Ok := True;
end;

var
  Pid: pid_t;
begin
  Ok := False;
  Writeln('1');
  fpsignal(SIGUSR1, @signal_handler);
  Writeln('2');
  Pid := fpgetpid;
  Writeln('3');
  fpkill(Pid, SIGUSR1);

  sleep(500);
  if not ok then
    halt(1);

  Writeln('5');
  Writeln('done');
end.

