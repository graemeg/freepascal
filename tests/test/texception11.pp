program texception11;

{$mode objfpc}

uses
  SysUtils;

type
  ETest = class(Exception);

procedure TestExcept(Obj : TObject; Addr : CodePointer; FrameCount:Longint; Frame: PCodePointer);
begin
  if not (Obj is ETest) then
    Halt(1);
  if not (ExceptObject is ETest) then
    Halt(2);
  { explicitely halt with exit code 0 }
  Halt(0);
end;

begin
  ExceptProc := @TestExcept;

  raise ETest.Create('');
end.
