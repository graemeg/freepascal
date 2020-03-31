{ %norun }
unit uw36720a;

{$mode objfpc}{$H+}
{$interfaces CORBA}

interface

uses
  Classes, SysUtils;

type
  IInterface = interface
    ['{BD11C7E5-5EF3-4F2A-B047-54FFA8ED43A9}']

    procedure DoSomething;
  end;


implementation

end.


