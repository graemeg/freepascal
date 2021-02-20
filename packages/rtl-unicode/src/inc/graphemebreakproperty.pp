{ GraphemeBreakProperty Unicode data unit.

  Copyright (C) 2021 Nikolay Nikolov <nickysn@users.sourceforge.net>

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version with the following modification:

  As a special exception, the copyright holders of this library give you
  permission to link this library with independent modules to produce an
  executable, regardless of the license terms of these independent modules,and
  to copy and distribute the resulting executable under terms of your choice,
  provided that you also meet, for each linked independent module, the terms
  and conditions of the license of that module. An independent module is a
  module which is not derived from or based on this library. If you modify
  this library, you may extend this exception to your version of the library,
  but you are not obligated to do so. If you do not wish to do so, delete this
  exception statement from your version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
  Inc., 51 Franklin Street - Fifth Floor, Boston, MA 02110-1335, USA.
}

unit graphemebreakproperty;

{$MODE objfpc}

interface

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

  { TUnicodeStringExtendedGraphemeClustersEnumerator }

  TUnicodeStringExtendedGraphemeClustersEnumerator = class
  private
    FStr: UnicodeString;
    FCurrentIndexStart: SizeInt;
    FCurrentIndexEnd: SizeInt;
    FNextIndexEnd: SizeInt;
    FNextGBP: TGraphemeBreakProperty;
    FNextCodePoint: UCS4Char;
    FCurrentGBP: TGraphemeBreakProperty;
    FCurrentCodePoint: UCS4Char;
    FRI_Sequence_Length: Integer;
    FE_Base_EBG_Extend_Sequence: Boolean;
    function GetCurrent: UnicodeString;
    procedure FetchNextChar;
  public
    constructor Create(const S: UnicodeString);
    function GetEnumerator: TUnicodeStringExtendedGraphemeClustersEnumerator;
    function MoveNext: Boolean;
    property Current: UnicodeString read GetCurrent;
  end;

function GetGraphemeBreakProperty(Ch: UCS4Char): TGraphemeBreakProperty;

implementation

function GetGraphemeBreakProperty(Ch: UCS4Char): TGraphemeBreakProperty;
begin
  {$I graphemebreakproperty_code.inc}
end;

{ TUnicodeStringExtendedGraphemeClustersEnumerator }

function TUnicodeStringExtendedGraphemeClustersEnumerator.GetCurrent: UnicodeString;
begin
  Result := Copy(FStr, FCurrentIndexStart, FCurrentIndexEnd - FCurrentIndexStart + 1);
end;

procedure TUnicodeStringExtendedGraphemeClustersEnumerator.FetchNextChar;
begin
  Inc(FNextIndexEnd);
  if FNextIndexEnd <= Length(FStr) then
  begin
    FNextCodePoint := Ord(FStr[FNextIndexEnd]);
    { high surrogate, followed by low surrogate? }
    if (FNextCodePoint >= $D800) and (FNextCodePoint <= $DBFF) and ((FNextIndexEnd + 1) <= Length(FStr)) and
       (Ord(FStr[FNextIndexEnd + 1]) >= $DC00) and (Ord(FStr[FNextIndexEnd + 1]) <= $DFFF) then
    begin
      Inc(FNextIndexEnd);
      FNextCodePoint := $10000 + (((FNextCodePoint - $D800) shl 10) or (Ord(FStr[FNextIndexEnd]) - $DC00));
    end;
  end
  else
    FNextCodePoint := 0;
  FNextGBP := GetGraphemeBreakProperty(FNextCodePoint);
end;

constructor TUnicodeStringExtendedGraphemeClustersEnumerator.Create(const S: UnicodeString);
begin
  FStr := S;
  FCurrentIndexStart := 0;
  FCurrentIndexEnd := 0;
  FNextIndexEnd := 0;
  FRI_Sequence_Length := 0;
  FE_Base_EBG_Extend_Sequence := False;
  FetchNextChar;
end;

function TUnicodeStringExtendedGraphemeClustersEnumerator.GetEnumerator: TUnicodeStringExtendedGraphemeClustersEnumerator;
begin
  Result := Self;
end;

function TUnicodeStringExtendedGraphemeClustersEnumerator.MoveNext: Boolean;
begin
  FCurrentIndexStart := FCurrentIndexEnd + 1;
  if FCurrentIndexStart > Length(FStr) then
    Exit(false);
  repeat
    FCurrentGBP := FNextGBP;
    FCurrentCodePoint := FNextCodePoint;
    FCurrentIndexEnd := FNextIndexEnd;
    if FCurrentGBP = gpbRegional_Indicator then
      Inc(FRI_Sequence_Length)
    else
      FRI_Sequence_Length := 0;
    FE_Base_EBG_Extend_Sequence := (FCurrentGBP in [gbpE_Base, gbpE_Base_GAZ]) or (FE_Base_EBG_Extend_Sequence and (FCurrentGBP = gbpExtend));
    FetchNextChar;
    if FNextIndexEnd > Length(FStr) then
      Exit(True);

    { Do not break between a CR and LF. Otherwise, break before and after controls. }
    if (FCurrentGBP = gbpCR) and (FNextGBP = gbpLF) then
      continue
    else if (FCurrentGBP in [gbpControl, gbpCR, gbpLF]) or (FNextGBP in [gbpControl, gbpCR, gbpLF]) then
      Exit(True)
    { Do not break Hangul syllable sequences. }
    else if ((FCurrentGBP = gbpL) and (FNextGBP in [gbpL, gbpV, gbpLV, gbpLVT])) or
            ((FCurrentGBP in [gbpLV, gbpV]) and (FNextGBP in [gbpV, gbpT])) or
            ((FCurrentGBP in [gbpLVT, gbpT]) and (FNextGBP = gbpT)) then
      continue
    { Do not break before extending characters or ZWJ. }
    else if FNextGBP in [gbpExtend, gbpZWJ] then
      continue
    { Only for extended grapheme clusters:
      Do not break before SpacingMarks, or after Prepend characters. }
    else if (FCurrentGBP = gbpPrepend) or (FNextGBP = gbpSpacingMark) then
      continue
    { Do not break within emoji modifier sequences or emoji zwj sequences. }
    else if ((FCurrentGBP = gbpZWJ) and (FNextGBP in [gbpGlue_After_Zwj, gbpE_Base_GAZ])) or
            (FE_Base_EBG_Extend_Sequence and (FNextGBP = gbpE_Modifier)) then
      continue
    { Do not break within emoji flag sequences. That is, do not break between regional indicator (RI) symbols if there is an odd number of RI characters before the break point. }
    else if (FCurrentGBP = gpbRegional_Indicator) and (FNextGBP = gpbRegional_Indicator) and Odd(FRI_Sequence_Length) then
      continue
    { Otherwise, break everywhere. }
    else
      Exit(True);
  until False;
end;

end.
