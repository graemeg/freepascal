{ this test fails for x86_64-linux with r44578, reason not found yet }
program Project1;

{$mode objfpc}{$H+}

uses
  Classes, SysUtils;

function RandomRange(const low : longint;
                     const high : longint) : longint;
begin
  if (high < low) then
    result := high + random(low - high + 1)
  else
    Result := low + random(high - low + 1);
end;

procedure GetStats(out used: ptruint);
var
  fpcHeapStatus : TFPCHeapStatus;
begin
  fpcHeapStatus := GetFPCHeapStatus();
  used:=fpcHeapStatus.CurrHeapUsed;
  writeln(' heap status: cu=' +
          IntToStr(fpcHeapStatus.CurrHeapUsed) + ', cs=' +
          IntToStr(fpcHeapStatus.CurrHeapSize) + ', cf=' +
          IntToStr(fpcHeapStatus.CurrHeapFree) + ', mu=' +
          IntToStr(fpcHeapStatus.MaxHeapUsed) + ', ms=' +
          IntToStr(fpcHeapStatus.MaxHeapSize));
end;

var
  i : integer;
  a : array of byte;
  u1, u2: ptruint;
begin
  randseed:=1586103426;
  writeln('randseed: ',randseed);
  GetStats(u1);
  for i := 0 to 50 do begin
    SetLength(a, RandomRange(1024,1024*1024*15));
  end;
  SetLength(a, 0);
  GetStats(u2);
  if u1<>u2 then
    halt(1);
end.
