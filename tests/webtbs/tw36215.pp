program IntArrayCode;
{$MODE OBJFPC}
{$R+}
{$OPTIMIZATION ON}

uses SysUtils;

type
  TIntegerArray = array of Integer;
  TSeparator = (tlSpace=0, tlComma=1, tlTab=2, tlAny, tlDetect);

var
  EmptyArray: TIntegerArray = nil;

function TryReadIntegerTabbedLine(Separator: TSeparator; s, Last: PAnsiChar; V: array of PInteger; out nFields: Integer; var SeparatorCount: TIntegerArray):PAnsiChar;
var
  P, B, Start: PAnsiChar;
  Value: Integer;
  IsString, Negative: Boolean;
begin
  nFields := 0;
  Result := nil;
  if s = nil then
    Exit; { No values read }
  P := s;
  while (nFields < Length(V)) and (V[nFields] <> nil) do begin
    while (P <> Last) and (P^ = ' ') do
      Inc(p);
    if (P = Last) or (P^ = #0) then
      Exit
    else if P^ in [#10,#13] then
      Break
    else if (P^ = #9) then begin
      if Separator in [tlComma, tlSpace] then
        Continue { Treat as white space }
      else if Separator = tlDetect then
        Inc(SeparatorCount[Ord(tlTab)]);
      Inc(P) { Empty field }
    end else if P^ = ',' then begin
      if Separator in [tlSpace, tlTab] then
        Break { Start of string field }
      else if Separator = tlDetect then
        Inc(SeparatorCount[Ord(tlComma)]);
      Inc(P) { Empty field }
    end else begin
      Start := P;
      IsString := False;
      Value := 0;
      Negative := P^ = '-';
      if P^ in ['+', '-'] then
        Inc(P);
      while not IsString and (P <> Last) and (P^ in ['0'..'9']) do begin
        if Value <= High(Integer) div 10 then
          Value := Value*10 + Ord(P^)-Ord('0')
        else
          IsString := True;
        Inc(P);
      end;
      while (P <> Last) and not (P^ in [#0, ' ', #9, ',', #10, #13]) do begin
        IsString := True;
        Inc(P);
      end;
      if not IsString then begin
        if Negative then
          Value := -Value;
        V[nFields]^ := Value;
      end;
      B := P;
      while (P <> Last) and ((P^ = ' ') or ((Separator in [tlSpace, tlComma]) and (P^ = #9))) do
        Inc(P);
      if (P <> Last) and not IsString then
        if P^ = #9 then begin
          if Separator = tlDetect then
            Inc(SeparatorCount[Ord(tlTab)]);
          Inc(P); { Non-empty field }
        end else if P^ = ',' then begin
          if Separator in [tlSpace, tlTab] then
            Break { Start of string field }
          else if Separator = tlDetect then
            Inc(SeparatorCount[Ord(tlComma)]);
          Inc(P); { Non-empty field }
        end else if (Separator = tlDetect) and (B <> P) then
          Inc(SeparatorCount[Ord(tlSpace)]);
      if IsString then begin
        P := Start;
        Break;
      end;
    end;
    Inc(nFields);
  end;
  if (P = Last) or (P^ = #0) then
    Result := nil
  else
    Result := P;
end;

function SkipLineEnding(var P: PAnsiChar):Boolean;
begin
  Result := True;
  case P^ of
    #10: if (P+1)^ = #13 then
           Inc(P,2)
         else
           Inc(P);
    #13: if (P+1)^ = #10 then
           Inc(P,2)
         else
           Inc(P);
  else
    Result := False;
  end;
end;

function ReadIntegerArray(S: AnsiString): TIntegerArray;
var
  P: PAnsiChar;
  V, nRows, nFields: Integer;
begin
  Result := nil;
  nRows := 0;
  P := PAnsiChar(S);
  while P^ <> #0 do begin
    P := TryReadIntegerTabbedLine(tlAny, P, nil, [@V], nFields, EmptyArray);
    if nFields > 0 then begin
      if nRows >= Length(Result) then
        if Length(Result) = 0 then { Length read from wrong placer? }
          SetLength(Result, 16)
        else
          SetLength(Result, Length(Result)*2);
      Result[nRows] := V;
      Inc(nRows);
    end;
    if P = nil then
      Break;
    while not (P^ in [#10, #13]) do
      Inc(P);
    SkipLineEnding(P);
  end;
  SetLength(Result, nRows);
end;

begin
  WriteLn(Length(ReadIntegerArray('-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10'-1'#10)));
end.

