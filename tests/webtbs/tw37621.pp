{ %norun }
program twctest;

{$mode delphi}
{$define InlineFuncs}

type
  REChar = WideChar;
  TRENextOff = PtrInt;
  PRegExprChar = PWideChar;
  TREOp = REChar; // internal p-code type //###0.933

const
  REOpSz = SizeOf(TREOp) div SizeOf(REChar);
  RENextOffSz = (SizeOf(TRENextOff) div SizeOf(REChar));

function CheckCharCategory(AChar: REChar; Ch0, Ch1: REChar): boolean;
// AChar: check this char against opcode
// Ch0, Ch1: opcode operands after OP_*CATEGORY
begin
end;

function MatchOneCharCategory(opnd, scan: PRegExprChar): boolean; {$IFDEF InlineFuncs}inline;{$ENDIF}
// opnd: points to opcode operands after OP_*CATEGORY
// scan: points into InputString
begin
  Result := CheckCharCategory(scan^, opnd^, (opnd + 1)^);
end;

procedure Test;
var
  scan, reginput: PRegExprChar;
begin
  if not MatchOneCharCategory(scan + REOpSz + RENextOffSz, reginput) then Exit;
end;

begin
  Test;
end.
