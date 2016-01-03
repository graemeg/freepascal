{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp126 = double;
{$else FPC_COMP_IS_INT64}
  comp126 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test126(a: byte); overload;
  begin
    writeln('byte called instead of widechar');
  end;

procedure test126(a: widechar); overload;
  begin
    writeln('widechar called instead of byte');
    halt(1)
  end;

var
  x126: byte;

  y126: widechar;
procedure dotest126;
var
  v: variant;

begin
  try
    v := x126;
    test126(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y126;
    test126(v);
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('No exception caught, exiting');
    Halt(1);
  except
    on E : TObject do
      Writeln('Caught exception, as expected',E.CLassName);
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
  dotest126;
end. {$endif not bigfile}
