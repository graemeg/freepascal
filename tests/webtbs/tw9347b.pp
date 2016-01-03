{$ifdef fpc}
{$mode delphi}
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
program tw9347b;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$r-}
uses
  SysUtils, Classes, TypInfo, Variants;

type
  TBla = class(TPersistent)
  private
    fBool: Boolean;
    fint: integer;
  published
    property Bool: Boolean read fBool write fBool;
    property int: integer read fint write fint;
  end;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  { fails in Delphi 6, succeeds in Delphi 7, fails in D2007 }
  try
    SetPropValue(TBla.Create, 'int', 'true');
  except
    on E: EVariantError do
      halt(0);
  end;
  halt(1);
=======
  { fails in Delphi 6, succeeds in Delphi 7 }
  SetPropValue(TBla.Create, 'int', 'true');
>>>>>>> graemeg/fixes_2_2
=======
  { fails in Delphi 6, succeeds in Delphi 7 }
  SetPropValue(TBla.Create, 'int', 'true');
>>>>>>> origin/fixes_2_2
end.
