<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %TARGET=win32,win64,wince,linux,android }
=======
{ %TARGET=win32,win64,wince}
>>>>>>> graemeg/cpstrnew
=======
{ %TARGET=win32,win64,wince}
>>>>>>> graemeg/cpstrnew
=======
{ %TARGET=win32,win64,wince}
>>>>>>> graemeg/cpstrnew
=======
{ %TARGET=win32,win64,wince}
>>>>>>> origin/cpstrnew
{$ifdef fpc}
{$mode objfpc}
{$endif}
uses
  SysUtils;
type
  TTest = class
  public
    procedure SomeError; safecall;
    function SafeCallException(ExceptObject: TObject; ExceptAddr: Pointer): HResult; override;
  end;

var
  ExceptObj: TObject;
  Handled: Boolean;

procedure TTest.SomeError; safecall;
begin
  ExceptObj := Exception.Create('SomeException');
  raise ExceptObj;
end;

function TTest.SafeCallException(ExceptObject: TObject; ExceptAddr: Pointer): HResult;
begin
  if ExceptAddr = nil then
    halt(2);
  if ExceptObject <> ExceptObj then
    halt(3);
  Handled := True;
  Result := 0;
end;

begin
  Handled := False;
  TTest.Create.SomeError;
  if not Handled then
    halt(1);
end.
