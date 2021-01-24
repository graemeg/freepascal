{ %norun }
Unit tw38385;

{$mode objfpc}{$H+}

Interface

Uses
  uw38385a, uw38385b, uw38385c;

Type

  { TFoo }

  TFoo = Class(TInterfacedObject, uw38385a.IInterface1, uw38385b.IInterface1, uw38385c.IInterface1)
    Procedure p1();
    Procedure p2();
    Procedure p3();
  End;

Implementation

{ TFoo }

Procedure TFoo.p1();
Begin
  WriteLn('p1');
End;

Procedure TFoo.p2();
Begin
  WriteLn('p2');
End;

Procedure TFoo.p3();
Begin
  WriteLn('p3');
End;

End.

