program tthlp29;

{$mode objfpc}
{$modeswitch typehelpers}
{$APPTYPE CONSOLE}

type
  TLongIntHelper = type helper for LongInt
    procedure Test;
  end;

procedure TLongIntHelper.Test;
begin
  Self := Self + 10;
end;

var
  l: LongInt;
  pl: PLongInt;
  pul: PLongWord;
  pb: PByte;

function GetPL: PLongInt;
begin
  Result := @l;
end;

function GetPUL: PLongWord;
begin
  Result := @l;
end;

function GetPB: PByte;
begin
  Result := @l;
end;

begin
  l := 0;
  pl := @l;
  pul := @l;
  pb := @l;
  Writeln(l);
  l.Test;
  Writeln(l);
  if l <> 10 then
    Halt(1);
  pl^.Test;
  Writeln(l);
  if l <> 20 then
    Halt(2);
  GetPL^.Test;
  Writeln(l);
  if l <> 30 then
    Halt(3);
  { type conversions with the same size are ignored }
  LongInt(pul^).Test;
  Writeln(l);
  if l <> 40 then
    Halt(4);
  LongInt(GetPUL^).Test;
  Writeln(l);
  if l <> 50 then
    Halt(5);
  { type conversions with different sizes operate on a tmp }
  LongInt(pb^).Test;
  Writeln(l);
  if l <> 50 then
    Halt(6);
  LongInt(GetPB^).Test;
  Writeln(l);
  if l <> 50 then
    Halt(7);
  Writeln('ok');
end.

