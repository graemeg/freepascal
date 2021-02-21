{ Parser and code generator for the EastAsianWidth.

  Copyright (C) 2021 Nikolay Nikolov <nickysn@users.sourceforge.net>

  This source is free software; you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This code is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  A copy of the GNU General Public License is available on the World Wide Web
  at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
  to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor,
  Boston, MA 02110-1335, USA.
}

program eawparser;

{$mode objfpc}{$H+}

uses
  SysUtils, StrUtils;

type
  TEastAsianWidth = (
    eawN,
    eawA,
    eawF,
    eawH,
    eawNa,
    eawW);

  TRange = record
    RangeLo, RangeHi: UCS4Char;
  end;
  TRanges = array of TRange;

var
  EastAsianWidths: array [UCS4Char] of TEastAsianWidth;
  EAWStats: array [TEastAsianWidth] of record
    Exists: Boolean;
    Handled: Boolean;
    MinValue: UCS4Char;
    MaxValue: UCS4Char;
    Count: LongInt;
    Ranges: TRanges;
  end;

function ParseEastAsianWidth(S: string): TEastAsianWidth;
begin
  S := Trim(S);
  case S of
    'N':
      Result := eawN;
    'A':
      Result := eawA;
    'F':
      Result := eawF;
    'H':
      Result := eawH;
    'Na':
      Result := eawNa;
    'W':
      Result := eawW;
    else
      raise EArgumentException('Unknown east asian width: ''' + S + '''');
  end;
end;

procedure ParseRange(S: string; out RangeLo, RangeHi: UCS4Char);
var
  dp: SizeInt;
begin
  S := Trim(S);
  dp := Pos('..', S);
  if dp > 0 then
  begin
    RangeLo := StrToInt('$' + LeftStr(S, dp - 1));
    RangeHi := StrToInt('$' + Copy(S, dp + 2, Length(S) - dp + 3));
  end
  else
  begin
    RangeLo := StrToInt('$' + S);
    RangeHi := RangeLo;
  end;
end;

procedure ParseEastAsianWidths(const FileName: string);
var
  InF: TextFile;
  S: string;
  SplitS: TStringArray;
  LineNr: Integer = 0;
  eaw: TEastAsianWidth;
  RangeLo, RangeHi, R: UCS4Char;
begin
  { - All code points, assigned or unassigned, that are not listed
      explicitly are given the value "N". }
  for R in UCS4Char do
    EastAsianWidths[R] := eawN;
  { - The unassigned code points in the following blocks default to "W":
           CJK Unified Ideographs Extension A: U+3400..U+4DBF
           CJK Unified Ideographs:             U+4E00..U+9FFF
           CJK Compatibility Ideographs:       U+F900..U+FAFF }
  for R := $3400 to $4DBF do
    EastAsianWidths[R] := eawW;
  for R := $4E00 to $9FFF do
    EastAsianWidths[R] := eawW;
  for R := $F900 to $FAFF do
    EastAsianWidths[R] := eawW;
  { - All undesignated code points in Planes 2 and 3, whether inside or
        outside of allocated blocks, default to "W":
           Plane 2:                            U+20000..U+2FFFD
           Plane 3:                            U+30000..U+3FFFD }
  for R := $20000 to $2FFFD do
    EastAsianWidths[R] := eawW;
  for R := $30000 to $3FFFD do
    EastAsianWidths[R] := eawW;

  if not FileExists(FileName) then
  begin
    Writeln('File doesn''t exist: ', FileName);
    Halt(1);
  end;
  AssignFile(InF, FileName);
  Reset(InF);
  while not EoF(InF) do
  begin
    Inc(LineNr);
    Readln(InF, S);
    S := Trim(S);
    if Pos('#', S) > 0 then
      S := LeftStr(S, Pos('#', S) - 1);
    if S <> '' then
    begin
      SplitS := S.Split([';']);
      if Length(SplitS) <> 2 then
        raise Exception.Create('Invalid number of ; separators on line ' + IntToStr(LineNr));
      ParseRange(SplitS[0], RangeLo, RangeHi);
      eaw := ParseEastAsianWidth(SplitS[1]);
      for R := RangeLo to RangeHi do
        EastAsianWidths[R] := eaw;
    end;
  end;
  CloseFile(InF);
end;

procedure CalcStatsAndRanges;
var
  Ch: UCS4Char;
  eaw, prev_eaw: TEastAsianWidth;
begin
  FillChar(EAWStats, SizeOf(EAWStats), 0);
  eaw := Low(TEastAsianWidth);
  for Ch := Low(UCS4Char) to High(UCS4Char) do
  begin
    prev_eaw := eaw;
    eaw := EastAsianWidths[Ch];
    with EAWStats[eaw] do
    begin
      if not Exists then
      begin
        Exists := True;
        MinValue := Ch;
        MaxValue := Ch;
        Count := 1;
        SetLength(Ranges, 1);
        Ranges[0].RangeLo := Ch;
        Ranges[0].RangeHi := Ch;
      end
      else
      begin
        MaxValue := Ch;
        Inc(Count);
        if prev_eaw <> eaw then
        begin
          SetLength(Ranges, Length(Ranges) + 1);
          with Ranges[High(Ranges)] do
          begin
            RangeLo := Ch;
            RangeHi := Ch;
          end;
        end
        else
          Ranges[High(Ranges)].RangeHi := Ch;
      end;
    end;
  end;
end;

procedure MaybeCoalesceRanges(RLo, RHi: UCS4Char);
var
  eaw: TEastAsianWidth;
  RI: Integer;
begin
  for eaw := Succ(Low(TEastAsianWidth)) to High(TEastAsianWidth) do
    if EAWStats[eaw].Exists and (not EAWStats[eaw].Handled) then
    begin
      for RI := 0 to High(EAWStats[eaw].Ranges) - 1 do
        if (EAWStats[eaw].Ranges[RI].RangeHi = (RLo - 1)) and
           (EAWStats[eaw].Ranges[RI + 1].RangeLo = (RHi + 1)) then
        begin
          EAWStats[eaw].Ranges[RI].RangeHi := EAWStats[eaw].Ranges[RI + 1].RangeHi;
          Delete(EAWStats[eaw].Ranges, RI + 1, 1);
          exit;
        end;
    end;
end;

function FindMinRangeCount: Integer;
var
  eaw: TEastAsianWidth;
begin
  Result := High(Integer);
  for eaw := Succ(Low(TEastAsianWidth)) to High(TEastAsianWidth) do
    if EAWStats[eaw].Exists and (not EAWStats[eaw].Handled) and (Length(EAWStats[eaw].Ranges) < Result) then
      Result := Length(EAWStats[eaw].Ranges);
end;

procedure GenCode(const OutFileName: string);
const
  RangeCountThreshold = 30{400};
var
  eaw: TEastAsianWidth;
  RI, NextRangeCount: Integer;
  OutFile: TextFile;
begin
  Writeln('Generating file: ', OutFileName);

  AssignFile(OutFile, OutFileName);
  Rewrite(OutFile);

  Writeln(OutFile, '{ do not edit, this file is autogenerated by the eawparser tool }');

  { unused properties are already handled }
  for eaw := Succ(Low(TEastAsianWidth)) to High(TEastAsianWidth) do
    if not EAWStats[eaw].Exists then
      EAWStats[eaw].Handled := True;

  { handle single codepoints first }
  for eaw := Succ(Low(TEastAsianWidth)) to High(TEastAsianWidth) do
    if (not EAWStats[eaw].Handled) and (EAWStats[eaw].Count = 1) then
    begin
      if EAWStats[eaw].MinValue <> EAWStats[eaw].MaxValue then
        raise Exception.Create('Internal error');
      Writeln(OutFile, 'if Ch=', EAWStats[eaw].MinValue, 'then result:=',eaw,' else');
      EAWStats[eaw].Handled := True;
      MaybeCoalesceRanges(EAWStats[eaw].MinValue, EAWStats[eaw].MaxValue);
    end;

  { handle single range codepoints next }
  while FindMinRangeCount = 1 do
    for eaw := Succ(Low(TEastAsianWidth)) to High(TEastAsianWidth) do
      if (not EAWStats[eaw].Handled) and (Length(EAWStats[eaw].Ranges) = 1) then
      begin
        Writeln(OutFile, 'if(Ch>=', EAWStats[eaw].MinValue, ')and(Ch<=', EAWStats[eaw].MaxValue, ')then result:=',eaw,' else');
        EAWStats[eaw].Handled := True;
        MaybeCoalesceRanges(EAWStats[eaw].MinValue, EAWStats[eaw].MaxValue);
      end;

  repeat
    NextRangeCount := FindMinRangeCount;
    if NextRangeCount <= RangeCountThreshold then
      for eaw := Succ(Low(TEastAsianWidth)) to High(TEastAsianWidth) do
      begin
        if not EAWStats[eaw].Handled and (Length(EAWStats[eaw].Ranges) <= NextRangeCount) then
        begin
          EAWStats[eaw].Handled := True;
          Write(OutFile, 'if');
          for RI := 0 to High(EAWStats[eaw].Ranges) do
          begin
            if RI <> 0 then
              Writeln(OutFile, 'or');
            with EAWStats[eaw].Ranges[RI] do
            begin
              if RangeLo = RangeHi then
                Write(OutFile, '(Ch=', RangeLo, ')')
              else
                Write(OutFile, '((Ch>=', RangeLo, ')and(Ch<=', RangeHi, '))');
              MaybeCoalesceRanges(RangeLo, RangeHi);
            end;
          end;
          Writeln(OutFile, 'then result:=',eaw,' else');
        end;
      end;
  until NextRangeCount > RangeCountThreshold;

  if NextRangeCount <> High(Integer) then
  begin
    //for eaw := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
    //  if not EAWStats[eaw].Handled then
    //    Writeln(eaw, ' ', EAWStats[eaw].MinValue, '..', EAWStats[eaw].MaxValue, ' ', EAWStats[eaw].Count, ' ', Length(EAWStats[eaw].Ranges), ' ', (EAWStats[eaw].MaxValue - EAWStats[eaw].MinValue + 7) div 8);
    Writeln(OutFile, 'case Ch of');
    for eaw := Succ(Low(TEastAsianWidth)) to High(TEastAsianWidth) do
    begin
      if not EAWStats[eaw].Handled then
      begin
        EAWStats[eaw].Handled := True;
        for RI := 0 to High(EAWStats[eaw].Ranges) do
        begin
          if RI <> 0 then
            Writeln(OutFile, ',');
          with EAWStats[eaw].Ranges[RI] do
          begin
            if RangeLo = RangeHi then
              Write(OutFile, RangeLo)
            else
              Write(OutFile, RangeLo, '..', RangeHi);
          end;
        end;
        Writeln(OutFile, ':result:=', eaw, ';');
      end;
    end;
    Writeln(OutFile, 'else result:=eawN end');
  end
  else
    Writeln(OutFile, 'result:=eawN');

  CloseFile(OutFile);
end;

begin
  ParseEastAsianWidths('data/UCD/EastAsianWidth.txt');
  CalcStatsAndRanges;
  GenCode('eastasianwidth_code.inc');
  Writeln('Done');
end.

