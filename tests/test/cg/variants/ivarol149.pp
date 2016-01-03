{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp149 = double;
{$else FPC_COMP_IS_INT64}
  comp149 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test149(a: ansistring); overload;
  begin
    writeln('ansistring called instead of widestring');
<<<<<<< HEAD
<<<<<<< HEAD
    halt(1)
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

procedure test149(a: widestring); overload;
  begin
    writeln('widestring called instead of ansistring');
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
  x149: ansistring;

  y149: widestring;
procedure dotest149;
var
  v: variant;

begin
  try
    v := x149;
    test149(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
    v := y149;
    test149(v);
  except
    on E : TObject do
      halt(1);
  end;
end;

{$ifndef bigfile} begin
  dotest149;
end. {$endif not bigfile}
