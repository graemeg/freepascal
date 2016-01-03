{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp142 = double;
{$else FPC_COMP_IS_INT64}
  comp142 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test142(a: shortstring); overload;
  begin
    writeln('shortstring called instead of widestring');
<<<<<<< HEAD
<<<<<<< HEAD
    halt(1)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

procedure test142(a: widestring); overload;
  begin
    writeln('widestring called instead of shortstring');
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
  x142: shortstring;

  y142: widestring;
procedure dotest142;
var
  v: variant;

begin
  try
    v := x142;
    test142(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y142;
    test142(v);
  except
    on E : TObject do
      halt(1);
  end;
end;

{$ifndef bigfile} begin
  dotest142;
end. {$endif not bigfile}
