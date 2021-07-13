// A basic tests for FmtBCD unit

{$ifdef fpc}{$mode objfpc}{$h+}{$endif}

uses SysUtils, FmtBCD, Variants;

var
  ErrorCount: integer;
  FS, DFS: TFormatSettings;
  bcd: TBCD;

procedure testBCDSubtract(bcd1,bcd2,bcd3: TBCD);
var bcdsub: TBCD;
begin
  bcdsub:=0;
  BCDSubtract(bcd1,bcd2,bcdsub);
  if (BCDCompare(bcd3,bcdsub) <> 0) or
     (bcdtostr(bcd3) <> bcdtostr(bcdsub)) then
  begin
    writeln(bcdtostr(bcd1), ' - ', bcdtostr(bcd2), ' = ', bcdtostr(bcdsub), ' but expected ', bcdtostr(bcd3));
    inc(ErrorCount);
  end;
end;

procedure testBCDMultiply(bcd1,bcd2,bcd3: TBCD);
var bcdmul: TBCD;
begin
  bcdmul:=0;
  BCDMultiply(bcd1,bcd2,bcdmul);
  if (BCDCompare(bcd3,bcdmul) <> 0) or
     (bcdtostr(bcd3) <> bcdtostr(bcdmul)) then
  begin
    writeln(bcdtostr(bcd1), ' * ', bcdtostr(bcd2), ' = ', bcdtostr(bcdmul), ' but expected ', bcdtostr(bcd3));
    writeln('Expected: (', bcd3.Precision,',',bcd3.SignSpecialPlaces, ') but calculated: (', bcdmul.Precision,',',bcdmul.SignSpecialPlaces,')');
    inc(ErrorCount);
  end;
end;

procedure testBCDDivide(bcd1,bcd2,bcd3: TBCD);
var bcddiv: TBCD;
begin
  bcddiv:=0;
  BCDDivide(bcd1,bcd2,bcddiv);
  if (BCDCompare(bcd3,bcddiv) <> 0) or
     (bcdtostr(bcd3) <> bcdtostr(bcddiv)) then
  begin
    writeln(bcdtostr(bcd1), ' / ', bcdtostr(bcd2), ' = ', bcdtostr(bcddiv), ' but expected ', bcdtostr(bcd3));
    //writeln('Expected: ', bcd3.Precision,',',bcd3.SignSpecialPlaces, ' but calculated: ', bcddiv.Precision,',',bcddiv.SignSpecialPlaces);
    inc(ErrorCount);
  end;
end;

procedure testBCDToStr(const bcd: TBCD; const Output: string);
var s: string;
begin
  s := BCDToStr(bcd);
  if s <> Output then
  begin
    writeln('BCDToStr: ', s, ' Expected: ', Output);
    inc(ErrorCount);
  end;
end;

procedure testBCDToStrF(const s1, s2: string);
begin
  if s1 <> s2 then
  begin
    writeln('BCDToStrF: ', s1, ' Expected: ', s2);
    inc(ErrorCount);
  end;
end;

procedure testFormatBCD(const Format: string; BCD: TBCD; const Output: string);
var s: string;
begin
  s := FormatBCD(Format, BCD);
  if s <> Output then
  begin
    writeln('FormatBCD ''', Format, ''': ', s, ' Expected: ', Output);
    inc(ErrorCount);
  end;
end;

procedure testBCDPrecScale(const s: string; const prec,scale: integer);
var bcd: TBCD;
begin
  bcd := strtobcd(s);
  if (bcd.Precision <> prec) or (BCDScale(bcd) <> scale) then
  begin
    writeln('StrToBcd: ', bcdtostr(bcd), ' (', s, ') Precision:', bcd.Precision, ' Scale: ', BCDScale(bcd));
    inc(ErrorCount);
  end;
end;

procedure testBCDToCurr(const s: string; c1: currency);
var c2: currency;
    b1, b2: boolean;
begin
  b1 := c1<>0;
  b2 := BCDToCurr(StrToBCD(s), c2);
  if b1 <> b2 then
    begin
    writeln('BCDToCurr for ', s, ' returned ', b2,' but expected ', b1);
    inc(ErrorCount);
    end
  else if b2 and (c1 <> c2) then
    begin
    writeln('BCDToCurr for ', s, ' returned ', c2,' but expected ', c1);
    inc(ErrorCount);
    end;
end;

procedure testBCDCompare(bcd1,bcd2: TBCD; res: integer);
var ret: integer;
begin
  ret := BCDCompare(bcd1,bcd2);
  if ret <> res then
  begin
    writeln('BCDCompare failed; bcd1:', bcdtostr(bcd1), ' bcd2:', bcdtostr(bcd2), ' returned ', ret, ' but expected ', res);
    inc(ErrorCount);
  end;
end;

procedure testNormalizeBCD(const input, expected: string; Precision,Places: integer; res: boolean);
var outBcd: TBCD;
begin
  outBcd:=0;
  if NormalizeBCD(StrToBCD(input,FS), outBcd, Precision, Places) <> res then
  begin
    writeln('NormalizeBCD for ', input, ' returned ', not res, ' but expected ', res);
    inc(ErrorCount);
  end;
  if StrToBCD(expected,FS) <> outBcd then
  begin
    writeln('NormalizeBCD for ', input, ' returned ', BCDToStr(outBcd,FS), ' but expected ', expected);
    inc(ErrorCount);
  end;
end;

procedure testVariantOp(v1, v2: variant);
var v: variant;
    d: double;
    s1: shortstring;
    s2: ansistring;
    s3: unicodestring;
begin
  //arithmetic op. ... invalid variant operation ?
  v := v1 + v2;
  v := v * v2;
  v := v / v2;
  v := v - v2;
  if VarIsFmtBCD(v1) and not VarIsFmtBCD(v) then inc(ErrorCount);

  //compare op.
  if not(v1=v) or (v1<>v) then
  begin
    writeln('Original variant: ', vartostr(v1), 'recomputed variant: ', vartostr(v));
    inc(ErrorCount);
  end;
  v := v + 1;
  if (v1 >= v) or not(v1 < v) then
  begin
    writeln('Compare2 failed; v1: ', vartostr(v1), ' v: ', vartostr(v));
    inc(ErrorCount);
  end;
  v := v - 1.1;
  if (v1 <= v) or not(v1 > v) then
  begin
    writeln('Compare3 failed; v1: ', vartostr(v1), ' v: ', vartostr(v));
    inc(ErrorCount);
  end;

  //assign op. ... invalid variant typecast ?
  //i := v;
  d := v;
  //s1 := v;
  s2 := v;
  //s3 := v;
end;

begin
  ErrorCount := 0;

  // test BCDToStr:
  DFS:=DefaultFormatSettings;

  FS.DecimalSeparator:=',';
  FS.ThousandSeparator:=#0;
  DefaultFormatSettings:=FS;
  testBCDToStr(0, '0');
  testBCDToStr(-123, '-123');
  testBCDToStr(0.5, '0,5');
  testBCDToStr(-1.03125, '-1,03125');
  testBCDToStr(CurrToBCD(1.2345), '1,2345');
  testBCDToStr(CurrToBCD(-0.0045), '-0,0045');

  // test BCDToStrF:
  FS.DecimalSeparator:=',';
  FS.ThousandSeparator:=' ';
  FS.CurrencyDecimals:=2;
  FS.CurrencyString:='$';
  FS.CurrencyFormat:=3;
  DefaultFormatSettings:=FS;
  bcd:=strtobcd('123456789123456789,12345');
  testBCDToStrF(bcdtostrf(bcd, ffFixed, 30, 4), '123456789123456789,1235'); //no thousand separators
  testBCDToStrF(bcdtostrf(bcd, ffNumber, 30, 5), '123 456 789 123 456 789,12345'); //with thousand separators
  testBCDToStrF(bcdtostrf(bcd, ffCurrency, 30, 2), '123 456 789 123 456 789,12 $'); //with thousand separators
  testBCDToStrF(bcdtostrf(bcd, ffExponent, 9, 2), '1,23456789E+17');

  FS.DecimalSeparator:='.';
  FS.ThousandSeparator:=',';
  FS.CurrencyFormat:=0;
  DefaultFormatSettings:=FS;
  bcd:=strtobcd('123456789123456789.12345');
  testBCDToStrF(bcdtostr(bcd), '123456789123456789.12345');
  testBCDToStrF(bcdtostrf(bcd, ffFixed, 30, 3), '123456789123456789.123'); //no thousand separators
  testBCDToStrF(bcdtostrf(bcd, ffFixed, 30, 0), '123456789123456789');
  testBCDToStrF(bcdtostrf(bcd, ffNumber, 30, 6), '123,456,789,123,456,789.123450'); //with thousand separators
  testBCDToStrF(bcdtostrf(bcd, ffCurrency, 30, 5), '$123,456,789,123,456,789.12345'); //with thousand separators
  testBCDToStrF(bcdtostrf(bcd, ffExponent, 8, 3), '1.2345679E+017');
  bcd:=strtobcd('123456789');
  testBCDToStrF(bcdtostrf(bcd, ffFixed, 10, 0), '123456789');
  testBCDToStrF(bcdtostrf(bcd, ffExponent, 8, 3), '1.2345679E+008');
  bcd:=strtobcd('9.99'); // test rounding
  testBCDToStrF(bcdtostrf(bcd, ffFixed, 10, 1), '10.0');
  testBCDToStrF(bcdtostrf(bcd, ffFixed, 10, 0), '10');
  testBCDToStrF(bcdtostrf(bcd, ffExponent, 8, 3), '9.9900000E+000');
  bcd:=strtobcd('0.09');
  testBCDToStrF(bcdtostrf(bcd, ffFixed, 10, 1), '0.1');
  testBCDToStrF(bcdtostrf(bcd, ffFixed, 10, 0), '0');
  testBCDToStrF(bcdtostrf(bcd, ffExponent, 8, 3), '9.0000000E-002');

  // test FormatBCD:
  bcd:=strtobcd('123456789123456789.12345');
  testFormatBCD('',bcd, '123456789123456789.12345');
  testFormatBCD('0',bcd, '123456789123456789');
  testFormatBCD('0.',bcd, '123456789123456789');
  testFormatBCD('0.0',bcd, '123456789123456789.1');
  testFormatBCD('#.0000',bcd, '123456789123456789.1235');
  testFormatBCD('#.000000',bcd, '123456789123456789.123450');
  testFormatBCD('# ###.000',bcd, '123456789123456 789.123');
  testFormatBCD('#-#-###.0000',bcd, '12345678912345-6-789.1235');
  testFormatBCD('#,#,###.0000',bcd, '123,456,789,123,456,789.1235');
  testFormatBCD('#,#.0000##',bcd, '123,456,789,123,456,789.12345');
  bcd:=strtobcd('-123.455');
  testFormatBCD('0.0',bcd, '-123.5');
  testFormatBCD('00000.0',bcd, '-00123.5');
  testFormatBCD('#####.#',bcd, '-123.5');
  testFormatBCD('.0000',bcd, '-123.4550');
  testFormatBCD('+0.0',bcd, '+-123.5'); // sign is part of number
  testFormatBCD('0.00" $"',bcd, '-123.46 $');
  testFormatBCD('0.0;(neg)0.00',bcd, '(neg)123.46');
  bcd:=strtobcd('0');
  testFormatBCD('0;;0',bcd, '0');
  testFormatBCD('0;;#',bcd, '');
  testFormatBCD('0;;0.#',bcd, '0');
  testFormatBCD('0;;0.00',bcd, '0.00');

  // test StrToBCD:
  testBCDPrecScale(' 1.0000000000000000E-0003 ', 3, 3);
  testBCDPrecScale('0.001', 3, 3);
  testBCDPrecScale('1.001', 4, 3);
  testBCDPrecScale('1001', 4, 0);
  testBCDPrecScale('1001.1001', 8, 4);

  // test BCDToCurr:
  testBCDToCurr( '922337203685477.5807',  922337203685477.5807); // boundary values
  testBCDToCurr('-922337203685477.5807', -922337203685477.5807);
  testBCDToCurr('-922337203685477.5808', StrToCurr('-922337203685477.5808'));
  testBCDToCurr( '922337203685477.5808', 0); // out-of-range values
  testBCDToCurr('-922337203685477.5809', 0);

  // test BCDSubtract:
  testBCDSubtract(CurrToBCD(0), CurrToBCD(-0.1), 0.1);

  DefaultFormatSettings := DFS;

  // test BCDMultiply:
  FS.DecimalSeparator:='.';
  FS.ThousandSeparator:=#0;
  testBCDMultiply(1000, -1000, -1000000);
  testBCDMultiply(-1000, -0.001, 1);
  testBCDMultiply(1000, 0.0001, 0.1);
  testBCDMultiply(strtobcd('12345678901234567890',FS), strtobcd('0.0000000001',FS), strtobcd('1234567890.123456789',FS));

  // test BCDDivide:
  testBCDDivide(1000, 1000, 1);
  testBCDDivide(1000, -100, -10);
  testBCDDivide(-1000, 10, -100);
  testBCDDivide(-1000, -1, 1000);
  testBCDDivide(11000, 11, 1000);
  testBCDDivide(11, 11000, 0.001);

  testBCDDivide(100, -2, -50);
  testBCDDivide(1007, 5, 201.4);

  testBCDDivide(StrToBCD('224518.0639999999994919',FS), IntegerToBCD(6615), StrToBCD('33.94075041572184421646258503401360544217687074829931972789115646',FS)); // bug #33795

  // test BCDCompare:
  testBCDCompare(100, 100, 0);
  testBCDCompare(-100.1, -100.1, 0);
  testBCDCompare(-100.1, 100.1, -1);
  testBCDCompare(-100.1, -100.2, 1);
  testBCDCompare(100, 100.1, -1);
  testBCDCompare(CurrToBcd(0.01), CurrToBcd(0.001), 1); // BCD values with Precision<Scale
  testBCDCompare(CurrToBcd(0.01), 0.01, 0);

  // test NormalizeBCD:
  testNormalizeBCD('100.17', '100.17', 5, 3, True);
  testNormalizeBCD('100.17', '100.17', 5, 2, True);
  testNormalizeBCD('100.17', '100.1' , 5, 1, False); // truncate, not round

  // test Variant support:
  testVariantOp(varFmtBcdCreate(100), varFmtBcdCreate(-100));
  testVariantOp(double(2.5), varFmtBcdCreate(100)); //double on left side
  testVariantOp(varFmtBcdCreate(100), integer(-10));
  testVariantOp(varFmtBcdCreate(-100), shortstring(floattostr(10.2)));
  testVariantOp(varFmtBcdCreate(-100), ansistring(floattostr(0.2)));
  testVariantOp(varFmtBcdCreate(-100), unicodestring(floattostr(-0.2)));

  if ErrorCount<>0 then
  begin
    writeln('FmtBCD test program found ', ErrorCount, ' errors!');
    Halt(ErrorCount);
  end;
end.
