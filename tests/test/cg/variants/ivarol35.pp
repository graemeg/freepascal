{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp35 = double;
{$else FPC_COMP_IS_INT64}
  comp35 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test35(a: comp35); overload;
  begin
    writeln('comp35 called instead of widechar');
  end;

procedure test35(a: widechar); overload;
  begin
    writeln('widechar called instead of comp35');
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
  x35: comp35;

  y35: widechar;
procedure dotest35;
var
  v: variant;

begin
  try
    v := x35;
    test35(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y35;
    test35(v);
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('Exception expected, none was raised');
    Halt(1);
  except
    on E : TObject do
      Writeln('Caught exception, as expected: ',E.ClassName);
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
  dotest35;
end. {$endif not bigfile}
