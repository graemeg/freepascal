{ %norun }
unit uw36720b;

{$mode objfpc}{$H+}
{$interfaces COM}

interface

uses
  Classes, SysUtils,
  uw36720a;

type
  IInterface3 = interface;
  IInterface2 = interface ( IInterface )
    ['{EB2E9267-C542-4784-81AE-A4C6ED044748}']

    function DoSomethingElse: IInterface3;
  end;

  IInterface3 = interface( IInterface2 )
    ['{EB2E9267-C542-4784-81AE-A4C6ED044748}']
    function DoSomethingCompletelyDifferent: Integer;
  end;


implementation

end.

