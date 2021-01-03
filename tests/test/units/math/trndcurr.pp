uses
  Math;


const
  failure_count : longint = 0;
  first_error : longint = 0;

{$ifndef SKIP_CURRENCY_TEST}
procedure testround(const c, expected: currency; error: longint);
begin
  if round(c)<>expected then
    begin
      writeln('round(',c,') = ',round(c),' instead of ', expected);
      inc(failure_count);
      if first_error=0 then
        first_error:=error;
    end;
end;

{$endif}


begin
{$ifndef SKIP_CURRENCY_TEST}
  if GetRoundMode <> rmNearest then
    begin
      writeln('Starting rounding mode is not rmNearest');
      inc(failure_count);
      if first_error=0 then
        first_error:=200;
    end;
  writeln('Rounding mode: rmNearest (even)');
  testround(0.5,0.0,1);
  testround(1.5,2.0,2);
  testround(-0.5,0.0,3);
  testround(-1.5,-2.0,4);
  testround(0.6,1.0,101);
  testround(1.6,2.0,102);
  testround(-0.6,-1.0,103);
  testround(-1.6,-2.0,104);
  testround(0.4,0.0,151);
  testround(1.4,1.0,152);
  testround(-0.4,-0.0,153);
  testround(-1.4,-1.0,154);

  writeln('Rounding mode: rmUp');
  if SetRoundMode(rmUp)<>rmNearest then
    writeln('Warning: previous mode was not rmNearest');
  if GetRoundMode <> rmUp then
    begin
      writeln('Failed to set rounding mode to rmUp');
      inc(failure_count);
      if first_error=0 then
        first_error:=201;
    end;
  testround(0.5,1.0,5);
  testround(1.5,2.0,6);
  testround(-0.5,0.0,7);
  testround(-1.5,-1.0,8);
  testround(0.6,1.0,105);
  testround(1.6,2.0,106);
  testround(-0.6,0.0,107);
  testround(-1.6,-1.0,108);
  testround(0.4,1.0,155);
  testround(1.4,2.0,156);
  testround(-0.4,0.0,157);
  testround(-1.4,-1.0,158);

  writeln('Rounding mode: rmDown');
  if SetRoundMode(rmDown)<>rmUp then
    writeln('Warning: previous mode was not rmUp');
  if GetRoundMode <> rmDown then
    begin
      writeln('Failed to set rounding mode to rmDown');
      inc(failure_count);
      if first_error=0 then
        first_error:=202;
    end;
  testround(0.5,0.0,9);
  testround(1.5,1.0,10);
  testround(-0.5,-1.0,11);
  testround(-1.5,-2.0,12);
  testround(0.6,0.0,109);
  testround(1.6,1.0,110);
  testround(-0.6,-1.0,111);
  testround(-1.6,-2.0,112);
  testround(0.4,0.0,159);
  testround(1.4,1.0,160);
  testround(-0.4,-1.0,161);
  testround(-1.4,-2.0,162);

  writeln('Rounding mode: rmTruncate');
  if SetRoundMode(rmTruncate)<>rmDown then
    writeln('Warning: previous mode was not rmDown');
  if GetRoundMode <> rmTruncate then
    begin
      writeln('Failed to set rounding mode to rmTruncate');
      inc(failure_count);
      if first_error=0 then
        first_error:=203;
    end;
  testround(0.5,0.0,13);
  testround(1.5,1.0,14);
  testround(-0.5,0.0,15);
  testround(-1.5,-1.0,16);
  testround(0.6,0.0,113);
  testround(1.6,1.0,114);
  testround(-0.6,0.0,115);
  testround(-1.6,-1.0,116);
  testround(0.4,0.0,163);
  testround(1.4,1.0,164);
  testround(-0.4,0.0,165);
  testround(-1.4,-1.0,166);
{$endif}
  if failure_count=0 then
    writeln('SetRoundMode test finished OK')
  else
    halt(first_error);
end.
