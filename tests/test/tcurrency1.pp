program tcurrency1;

{ test basic mathematical operations (+,-,*,/) using currency data type }

var
  c1, c2: Currency;
  d: Double;
  i: Integer;
  i64: int64;

begin
  write('Currency and Double ...');
  // addition double
  d := 1;
  c1 := 2;
  c2 := 3;
  if c1+d <> c2 then begin
    writeln('Invalid currency+double=', c1+d, ', but expected ', c2);
    halt(1);
  end;
  // subtraction double
  d := 3;
  c1 := 2;
  c2 := -1;
  if c1-d <> c2 then begin
    writeln('Invalid currency-double=', c1-d, ', but expected ', c2);
    halt(1);
  end;
  // multiplication double
  d := -100;
  c1 := 12.34;
  c2 := -1234;
  if d*c1 <> c2 then begin
    writeln('Invalid currency*double=', d*c1, ', but expected ', c2);
    halt(1);
  end;
  // division double
  d := 100;
  c1 := 12.34;
  c2 := 0.1234;
  if c1/d <> c2 then begin
    writeln('Invalid currency/double=', c1/d, ', but expected ', c2);
    halt(1);
  end;
  writeln(' Passed');

  write('Currency and Integer ...');
  // addition integer
  i := 1;
  c1 := 2;
  c2 := 3;
  if c1+i <> c2 then begin
    writeln('Invalid currency+integer=', c1+i, ', but expected ', c2);
    halt(2);
  end;
  // subtraction integer
  i := 10;
  c1 := -2;
  c2 := -12;
  if c1-i <> c2 then begin
    writeln('Invalid currency-integer=', c1-i, ', but expected ', c2);
    halt(2);
  end;
  // multiplication integer
  i := 100;
  c1 := 12.34;
  c2 := 1234;
  if i*c1 <> c2 then begin
    writeln('Invalid currency*integer=', i*c1, ', but expected ', c2);
    halt(2);
  end;
  i:=10000;
  c1:=92233720368.5477;
  c2:=922337203685477;
  if c1*i <> c2 then begin
    writeln('Invalid currency*integer=', c1*i, ', but expected ', c2);
    halt(2);
  end;
  if i*c1 <> c2 then begin
    writeln('Invalid integer*currency=', i*c1, ', but expected ', c2);
    halt(2);
  end;
  // division integer
  i := 1000;
  c1 := 123.4;
  c2 := 0.1234;
  if c1/i <> c2 then begin
    writeln('Invalid currency/integer=', c1/i, ', but expected ', c2);
    halt(2);
  end;
  writeln(' Passed');

  write('Currency and Int64 ...');
  // addition int64
  i64 := 1;
  c1 := 12.3456;
  c2 := 13.3456;
  if c1+i64 <> c2 then begin
    writeln('Invalid currency+int64=', c1+i64, ', but expected ', c2);
    halt(3);
  end;
  // subtraction int64
  i64 := 100;
  c1 := 12.3456;
  c2 := -87.6544;
  if c1-i64 <> c2 then begin
    writeln('Invalid currency-int64=', c1-i64, ', but expected ', c2);
    halt(3);
  end;
  // multiplication int64
  i64 := -10000;
  c1 := 12.3456;
  c2 := -123456;
  if i64*c1 <> c2 then begin
    writeln('Invalid currency*int64=', i64*c1, ', but expected ', c2);
    halt(3);
  end;
  // division int64
  i64 := -10000;
  c1 := 123456;
  c2 := -12.3456;
  if c1/i64 <> c2 then begin
    writeln('Invalid currency/int64=', c1/i64, ', but expected ', c2);
    halt(3);
  end;
  writeln(' Passed');
end.
