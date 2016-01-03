{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp136 = double;
{$else FPC_COMP_IS_INT64}
  comp136 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test136(a: shortstring); overload;
  begin
    writeln('shortstring called instead of ansistring');
<<<<<<< HEAD
<<<<<<< HEAD
    halt(1)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

procedure test136(a: ansistring); overload;
  begin
    writeln('ansistring called instead of shortstring');
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
  x136: shortstring;

  y136: ansistring;
procedure dotest136;
var
  v: variant;

begin
  try
    v := x136;
    test136(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y136;
    test136(v);
  except
    on E : TObject do
      halt(1);
  end;
end;

{$ifndef bigfile} begin
  dotest136;
end. {$endif not bigfile}
