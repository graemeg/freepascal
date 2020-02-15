{ %fail }
{$mode delphi}
program test;

type
 TFooA = class
 end;

type
 TFooB = class
 end;

type
 TList<T: TFooA> = class
   procedure Foo;
 end;

procedure TList<T: TFooB>.Foo;
begin
end;

begin
end.
