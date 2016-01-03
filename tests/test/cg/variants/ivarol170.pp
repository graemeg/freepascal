{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp170 = double;
{$else FPC_COMP_IS_INT64}
  comp170 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test170(a: boolean); overload;
  begin
    writeln('boolean called instead of widechar');
  end;

procedure test170(a: widechar); overload;
  begin
    writeln('widechar called instead of boolean');
<<<<<<< HEAD
<<<<<<< HEAD
=======
    halt(1)
>>>>>>> graemeg/fixes_2_2
=======
    halt(1)
>>>>>>> origin/fixes_2_2
  end;

var
  x170: boolean;

  y170: widechar;
procedure dotest170;
var
  v: variant;

begin
  try
    v := x170;
    test170(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y170;
    test170(v);
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('Exception expected, but none was raised');
    Halt(1);
  except
    on E : TObject do
      Writeln('Caught exception, as expected : ',E.ClassName);
=======
  except
    on E : TObject do
      halt(1);
>>>>>>> graemeg/fixes_2_2
=======
  except
    on E : TObject do
      halt(1);
>>>>>>> origin/fixes_2_2
  end;
end;

{$ifndef bigfile} begin
  dotest170;
end. {$endif not bigfile}
