{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp161 = double;
{$else FPC_COMP_IS_INT64}
  comp161 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test161(a: double); overload;
  begin
    writeln('double called instead of widechar');
  end;

procedure test161(a: widechar); overload;
  begin
    writeln('widechar called instead of double');
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
  x161: double;

  y161: widechar;
procedure dotest161;
var
  v: variant;

begin
  try
    v := x161;
    test161(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y161;
    test161(v);
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('Exception expected, but none was raised');
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
  dotest161;
end. {$endif not bigfile}
