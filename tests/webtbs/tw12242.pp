{
 staticbug.pas

 With FPC 2.2.2:

 staticbug.lpr(24,31) Error: function header doesn't match the previous declaration "class TMyController.doClose(Pointer, Pointer, Pointer);CDecl"
}
program staticbug;

<<<<<<< HEAD
<<<<<<< HEAD
{$mode delphi}
=======
{$mode delphi}{$STATIC ON}
>>>>>>> graemeg/fixes_2_2
=======
{$mode delphi}{$STATIC ON}
>>>>>>> origin/fixes_2_2

uses
  Classes, SysUtils;

type

  { TMyController }

  TMyController = class
  public
    class procedure doClose(_self: Pointer; _cmd: Pointer; sender: Pointer); cdecl; static;
  end;

class procedure TMyController.doClose(_self: Pointer; _cmd: Pointer; sender: Pointer); cdecl; static;
begin
end;

begin
end.
