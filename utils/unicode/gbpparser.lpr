{ Parser and code generator for the GraphemeBreakProperty.

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


program gbpparser;

{$mode objfpc}{$H+}

uses
  SysUtils, StrUtils;

type
  TGraphemeBreakProperty = (
    gbpOther,
    gbpPrepend,
    gbpCR,
    gbpLF,
    gbpControl,
    gbpExtend,
    gpbRegional_Indicator,
    gbpSpacingMark,
    gbpL,
    gbpV,
    gbpT,
    gbpLV,
    gbpLVT,
    gbpE_Base,
    gbpE_Modifier,
    gbpZWJ,
    gbpGlue_After_Zwj,
    gbpE_Base_GAZ);

  TRange = record
    RangeLo, RangeHi: UCS4Char;
  end;
  TRanges = array of TRange;

var
  GraphemeBreakProperties: array [UCS4Char] of TGraphemeBreakProperty;
  GBPStats: array [TGraphemeBreakProperty] of record
    Exists: Boolean;
    Handled: Boolean;
    MinValue: UCS4Char;
    MaxValue: UCS4Char;
    Count: LongInt;
    Ranges: TRanges;
  end;

function ParseGraphemeBreakProperty(S: string): TGraphemeBreakProperty;
begin
  S := Trim(S);
  case S of
    'Prepend':
      Result := gbpPrepend;
    'CR':
      Result := gbpCR;
    'LF':
      Result := gbpLF;
    'Control':
      Result := gbpControl;
    'Extend':
      Result := gbpExtend;
    'Regional_Indicator':
      Result := gpbRegional_Indicator;
    'SpacingMark':
      Result := gbpSpacingMark;
    'L':
      Result := gbpL;
    'V':
      Result := gbpV;
    'T':
      Result := gbpT;
    'LV':
      Result := gbpLV;
    'LVT':
      Result := gbpLVT;
    'E_Base':
      Result := gbpE_Base;
    'E_Modifier':
      Result := gbpE_Modifier;
    'ZWJ':
      Result := gbpZWJ;
    'Glue_After_Zwj':
      Result := gbpGlue_After_Zwj;
    'E_Base_GAZ':
      Result := gbpE_Base_GAZ;
    else
      raise EArgumentException('Unknown grapheme break property: ''' + S + '''');
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

procedure ParseGraphemeBreakProperties(const FileName: string);
var
  InF: TextFile;
  S: string;
  SplitS: TStringArray;
  LineNr: Integer = 0;
  gbp: TGraphemeBreakProperty;
  RangeLo, RangeHi, R: UCS4Char;
begin
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
      gbp := ParseGraphemeBreakProperty(SplitS[1]);
      for R := RangeLo to RangeHi do
        GraphemeBreakProperties[R] := gbp;
    end;
  end;
  CloseFile(InF);
end;

procedure CalcStatsAndRanges;
var
  Ch: UCS4Char;
  gbp, prev_gbp: TGraphemeBreakProperty;
begin
  FillChar(GBPStats, SizeOf(GBPStats), 0);
  gbp := Low(TGraphemeBreakProperty);
  for Ch := Low(UCS4Char) to High(UCS4Char) do
  begin
    prev_gbp := gbp;
    gbp := GraphemeBreakProperties[Ch];
    with GBPStats[gbp] do
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
        if prev_gbp <> gbp then
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
  gbp: TGraphemeBreakProperty;
  RI: Integer;
begin
  for gbp := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
    if GBPStats[gbp].Exists and (not GBPStats[gbp].Handled) then
    begin
      for RI := 0 to High(GBPStats[gbp].Ranges) - 1 do
        if (GBPStats[gbp].Ranges[RI].RangeHi = (RLo - 1)) and
           (GBPStats[gbp].Ranges[RI + 1].RangeLo = (RHi + 1)) then
        begin
          GBPStats[gbp].Ranges[RI].RangeHi := GBPStats[gbp].Ranges[RI + 1].RangeHi;
          Delete(GBPStats[gbp].Ranges, RI + 1, 1);
          exit;
        end;
    end;
end;

function FindMinRangeCount: Integer;
var
  gbp: TGraphemeBreakProperty;
begin
  Result := High(Integer);
  for gbp := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
    if GBPStats[gbp].Exists and (not GBPStats[gbp].Handled) and (Length(GBPStats[gbp].Ranges) < Result) then
      Result := Length(GBPStats[gbp].Ranges);
end;

function ApplyLV_LVTCompression: Boolean;
const
  RangeLo = 44032;
  RangeHi = 55203;
var
  Ch: UCS4Char;
begin
  Result := False;
  if (GBPStats[gbpLV].MinValue <> RangeLo) or (GBPStats[gbpLV].MaxValue <> (RangeHi - 27)) or
     (GBPStats[gbpLVT].MinValue <> (RangeLo + 1)) or (GBPStats[gbpLVT].MaxValue <> RangeHi) then
    exit;
  for Ch := RangeLo to RangeHi do
  begin
    if ((Ch - RangeLo) mod 28) = 0 then
    begin
      if GraphemeBreakProperties[Ch] <> gbpLV then
        exit;
    end
    else
    begin
      if GraphemeBreakProperties[Ch] <> gbpLVT then
        exit;
    end;
  end;
  Result := True;
end;

procedure GenCode(const OutFileName: string);
const
  RangeCountThreshold = 30{400};
var
  gbp: TGraphemeBreakProperty;
  RI, NextRangeCount: Integer;
  OutFile: TextFile;
begin
  Writeln('Generating file: ', OutFileName);

  AssignFile(OutFile, OutFileName);
  Rewrite(OutFile);

  Writeln(OutFile, '{ do not edit, this file is autogenerated by the gbpparser tool }');

  { unused properties are already handled }
  for gbp := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
    if not GBPStats[gbp].Exists then
      GBPStats[gbp].Handled := True;

  { handle single codepoints first }
  for gbp := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
    if (not GBPStats[gbp].Handled) and (GBPStats[gbp].Count = 1) then
    begin
      if GBPStats[gbp].MinValue <> GBPStats[gbp].MaxValue then
        raise Exception.Create('Internal error');
      Writeln(OutFile, 'if Ch=', GBPStats[gbp].MinValue, 'then result:=',gbp,' else');
      GBPStats[gbp].Handled := True;
      MaybeCoalesceRanges(GBPStats[gbp].MinValue, GBPStats[gbp].MaxValue);
    end;

  { handle single range codepoints next }
  while FindMinRangeCount = 1 do
    for gbp := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
      if (not GBPStats[gbp].Handled) and (Length(GBPStats[gbp].Ranges) = 1) then
      begin
        Writeln(OutFile, 'if(Ch>=', GBPStats[gbp].MinValue, ')and(Ch<=', GBPStats[gbp].MaxValue, ')then result:=',gbp,' else');
        GBPStats[gbp].Handled := True;
        MaybeCoalesceRanges(GBPStats[gbp].MinValue, GBPStats[gbp].MaxValue);
      end;

  if ApplyLV_LVTCompression then
  begin
    Writeln(OutFile, 'if(Ch>=44032)and(Ch<=55203)then begin if((Ch-44032)mod 28)=0then result:=gbpLV else result:=gbpLVT end else');
    GBPStats[gbpLV].Handled := True;
    GBPStats[gbpLVT].Handled := True;
  end;

  repeat
    NextRangeCount := FindMinRangeCount;
    if NextRangeCount <= RangeCountThreshold then
      for gbp := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
      begin
        if not GBPStats[gbp].Handled and (Length(GBPStats[gbp].Ranges) <= NextRangeCount) then
        begin
          GBPStats[gbp].Handled := True;
          Write(OutFile, 'if');
          for RI := 0 to High(GBPStats[gbp].Ranges) do
          begin
            if RI <> 0 then
              Writeln(OutFile, 'or');
            with GBPStats[gbp].Ranges[RI] do
            begin
              if RangeLo = RangeHi then
                Write(OutFile, '(Ch=', RangeLo, ')')
              else
                Write(OutFile, '((Ch>=', RangeLo, ')and(Ch<=', RangeHi, '))');
              MaybeCoalesceRanges(RangeLo, RangeHi);
            end;
          end;
          Writeln(OutFile, 'then result:=',gbp,' else');
        end;
      end;
  until NextRangeCount > RangeCountThreshold;

  if NextRangeCount <> High(Integer) then
  begin
    //for gbp := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
    //  if not GBPStats[gbp].Handled then
    //    Writeln(gbp, ' ', GBPStats[gbp].MinValue, '..', GBPStats[gbp].MaxValue, ' ', GBPStats[gbp].Count, ' ', Length(GBPStats[gbp].Ranges), ' ', (GBPStats[gbp].MaxValue - GBPStats[gbp].MinValue + 7) div 8);
    Writeln(OutFile, 'case Ch of');
    for gbp := Succ(Low(TGraphemeBreakProperty)) to High(TGraphemeBreakProperty) do
    begin
      if not GBPStats[gbp].Handled then
      begin
        GBPStats[gbp].Handled := True;
        for RI := 0 to High(GBPStats[gbp].Ranges) do
        begin
          if RI <> 0 then
            Writeln(OutFile, ',');
          with GBPStats[gbp].Ranges[RI] do
          begin
            if RangeLo = RangeHi then
              Write(OutFile, RangeLo)
            else
              Write(OutFile, RangeLo, '..', RangeHi);
          end;
        end;
        Writeln(OutFile, ':result:=', gbp, ';');
      end;
    end;
    Writeln(OutFile, 'else result:=gbpOther end');
  end
  else
    Writeln(OutFile, 'result:=gbpOther');

  CloseFile(OutFile);
end;

begin
  FillChar(GraphemeBreakProperties, SizeOf(GraphemeBreakProperties), 0);
  ParseGraphemeBreakProperties('data/UCD/auxiliary/GraphemeBreakProperty.txt');
  CalcStatsAndRanges;
  GenCode('graphemebreakproperty_code.inc');
  Writeln('Done');
end.

