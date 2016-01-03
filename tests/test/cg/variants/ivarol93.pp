{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp93 = double;
{$else FPC_COMP_IS_INT64}
  comp93 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test93(a: cardinal); overload;
  begin
    writeln('cardinal called instead of widechar');
  end;

procedure test93(a: widechar); overload;
  begin
    writeln('widechar called instead of cardinal');
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
  x93: cardinal;

  y93: widechar;
procedure dotest93;
var
  v: variant;

begin
  try
    v := x93;
    test93(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y93;
    test93(v);
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('Expected exception, none was raised');
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
  dotest93;
end. {$endif not bigfile}
