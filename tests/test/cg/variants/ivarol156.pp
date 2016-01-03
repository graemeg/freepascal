{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp156 = double;
{$else FPC_COMP_IS_INT64}
  comp156 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test156(a: single); overload;
  begin
    writeln('single called instead of widechar');
  end;

procedure test156(a: widechar); overload;
  begin
    writeln('widechar called instead of single');
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
  x156: single;

  y156: widechar;
procedure dotest156;
var
  v: variant;

begin
  try
    v := x156;
    test156(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y156;
    test156(v);
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('Exception expected, but none raised');
    Halt(1);
  except
    on E : TObject do
      Writeln('Caught exception as expected: ',E.ClassName);
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
  dotest156;
end. {$endif not bigfile}
