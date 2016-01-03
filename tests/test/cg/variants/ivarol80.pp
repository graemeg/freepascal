{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp80 = double;
{$else FPC_COMP_IS_INT64}
  comp80 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test80(a: longint); overload;
  begin
    writeln('longint called instead of widechar');
  end;

procedure test80(a: widechar); overload;
  begin
    writeln('widechar called instead of longint');
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
  x80: longint;

  y80: widechar;
procedure dotest80;
var
  v: variant;

begin
  try
    v := x80;
    test80(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y80;
    test80(v);
<<<<<<< HEAD
<<<<<<< HEAD
    Writeln('Exception expected, but none was raised');
    Halt(1)
  except
    on E : TObject do
      Writeln('Exception caught, as expected: ',E.ClassName);
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
  dotest80;
end. {$endif not bigfile}
