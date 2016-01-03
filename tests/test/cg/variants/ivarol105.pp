{$ifndef bigfile}
{$ifdef fpc}
{$mode delphi}
{$else fpc}
{$define FPC_HAS_TYPE_EXTENDED}
{$endif fpc}
{$endif bigfile}

type 
{$ifdef FPC_COMP_IS_INT64}
  comp105 = double;
{$else FPC_COMP_IS_INT64}
  comp105 = comp;
{$endif FPC_COMP_IS_INT64}
procedure test105(a: word); overload;
  begin
    writeln('word called instead of widechar');
  end;

procedure test105(a: widechar); overload;
  begin
    writeln('widechar called instead of word');
    halt(1)
  end;

var
  x105: word;

  y105: widechar;
procedure dotest105;
var
  v: variant;

begin
  try
    v := x105;
    test105(v);
  except
    on E : TObject do
      halt(1);
  end;

  try
<<<<<<< HEAD
<<<<<<< HEAD
    y105:='a';
    v := y105;
    test105(v);
    Writeln('Did not catch exception as expected');
    Halt(1)
  except
    on E : TObject do
      begin
      Writeln('Caught exception ',E.ClassName,'as expected');
      end
=======
=======
>>>>>>> origin/fixes_2_2
    v := y105;
    test105(v);
  except
    on E : TObject do
      halt(1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;

{$ifndef bigfile} begin
  dotest105;
end. {$endif not bigfile}
