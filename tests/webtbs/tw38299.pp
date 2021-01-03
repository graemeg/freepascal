{ %opt=-O2 -Fcutf8 }
program bug;
const
  cAnsiLineFeed = AnsiChar(#10);
  cAnsiCarriageReturn = AnsiChar(#13);
var
  test: RawByteString;
begin
  test := '123';
  test := test + UTF8Encode('456') + '789' + cAnsiCarriageReturn + cAnsiLineFeed;
  writeln(test);
  if test<>'123456789'#13#10 then
    halt(1);
  writeln('ok');
end.
