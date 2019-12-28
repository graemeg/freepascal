{
 ***************************************************************************
                    maskutils.pas:    Author: Bart Broersma
 ***************************************************************************}

{
 *****************************************************************************
 *                                                                           *
 *  This file is part of the Free Component Library (FCL)                    *
 *                                                                           *
 *  See the file COPYING.FPC, included in this distribution,                 *
 *  for details about the copyright.                                         *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                     *
 *                                                                           *
 *****************************************************************************
}


unit MaskUtils;

{$mode objfpc}{$H+}
{.$define debug_maskutils}

interface

uses
  SysUtils;


function FormatMaskText(const EditMask: string; const AValue: string): string;
function FormatMaskInput(const EditMask: string): string;
function MaskDoFormatText(const EditMask: string; const AValue: string; ASpaceChar: Char): string;


type
  TEditMask = type string;
  TMaskeditTrimType = (metTrimLeft, metTrimRight);

  { Type for mask (internal) }
  tMaskedType = (Char_Start,
                 Char_Number,
                 Char_NumberFixed,
                 Char_NumberPlusMin,
                 Char_Letter,
                 Char_LetterFixed,
                 Char_LetterUpCase,
                 Char_LetterDownCase,
                 Char_LetterFixedUpCase,
                 Char_LetterFixedDownCase,
                 Char_AlphaNum,
                 Char_AlphaNumFixed,
                 Char_AlphaNumUpCase,
                 Char_AlphaNumDownCase,
                 Char_AlphaNumFixedUpCase,
                 Char_AlphaNumFixedDownCase,
                 Char_All,
                 Char_AllFixed,
                 Char_AllUpCase,
                 Char_AllDownCase,
                 Char_AllFixedUpCase,
                 Char_AllFixedDownCase,
                 Char_HourSeparator,
                 Char_DateSeparator,
                 Char_Stop);


{ TMaskUtils }

type
  TMaskUtils = class(TObject)
  private
    FRealMask: String;
    FMask: String;  // internal representatio of the mask
    FValue: String;
    FMaskLength: Integer;
    FMaskSave: Boolean;
    FSpaceChar: Char;
    FTrimType: TMaskeditTrimType;
    procedure AddToMask(Ch: Char);
    function  MaskToChar(AValue: tMaskedType) : Char;
    function  CharToMask(Ch: Char) : tMaskedType;

    function  CharMatchesMask(const Ch: Char; const Position: Integer): Boolean;
    function  ClearChar(Position : Integer) : Char;
    procedure SetMask(AValue: String);
    function GetInputMask: String;
    function GetTextWithoutMask(AValue: String) : String;
    function GetTextWithoutSpaceChar(AValue: String) : String;
    function  IsLiteral(Ch: Char): Boolean;
    function  IsMaskChar(Ch : Char) : Boolean;
    procedure SetValue(AValue: String);
    function  TextIsValid(const AValue: String): Boolean;
  protected
    function ApplyMaskToText(AValue: String): String;
  public
    function ValidateInput : String;
    function TryValidateInput(out ValidatedString: String): Boolean;
    property Mask : String read FRealMask write SetMask;
    property Value : String read FValue write SetValue;
    property InputMask : String read GetInputMask;
  end;


implementation

resourcestring
  exValidationFailed = 'TMaskUtils.ValidateInput failed.';

const
  { Mask Type }
  cMask_SpecialChar   = '\'; // after this you can set an arbitrary char
  cMask_UpperCase     = '>'; // after this the chars is in upper case
  cMask_LowerCase     = '<'; // after this the chars is in lower case
  cMask_Letter        = 'l'; // only a letter but not necessary
  cMask_LetterFixed   = 'L'; // only a letter
  cMask_AlphaNum      = 'a'; // an alphanumeric char (['A'..'Z','a..'z','0'..'9']) but not necessary
  cMask_AlphaNumFixed = 'A'; // an alphanumeric char
  cMask_AllChars      = 'c'; // any Utf8 char but not necessary
  cMask_AllCharsFixed = 'C'; // any Utf8 char #32 - #255
  cMask_Number        = '9'; // only a number but not necessary
  cMask_NumberFixed   = '0'; // only a number
  cMask_NumberPlusMin = '#'; // only a number or + or -, but not necessary
  cMask_HourSeparator = ':'; // automatically put the hour separator char
  cMask_DateSeparator = '/'; // automatically but the date separator char
{ cMask_SpaceOnly     = '_'; // automatically put a space          //not Delphi compatible        }
  cMask_NoLeadingBlanks = '!'; //Trim leading blanks, otherwise trim trailing blanks from the data

  {Delphi compatibility: user can change these at runtime}
  DefaultBlank: Char = '_';
  MaskFieldSeparator: Char = ';';
  MaskNoSave: Char = '0';





procedure SplitEditMask(AEditMask: String; out AMaskPart: String; out AMaskSave: Boolean; out ASpaceChar: Char);
{
  Retrieve the separate fields for a given EditMask:
  Given an AEditMask of '999.999;0;_'  it will return
  - AMaskPart = '999.999'
  - AMaskSave = False
  - ASpaceChar = '_'
}
begin
  {
    First see if AEditMask is multifield and if we can extract a value for
    AMaskSave and/or ASpaceChar
    If so, extract and remove from AMask (so we know that the remaining part of
    AMask _IS_ the mask to be set)

    A value for SpaceChar is only valid if also a value for MaskSave is specified
    (as by Delphi specifications), so Mask must be at least 4 characters
    These must be the last 2 or 4 characters of EditMask (and there must not be
    an escape character in front!)
  }
  //Assume no SpaceChar and no MaskSave is defined in new mask, so first set it to DefaultBlank and True
  ASpaceChar := DefaultBlank;
  AMaskSave := True;
  //MaskFieldseparator, MaskNoSave, SpaceChar and cMask_SpecialChar are defined as Char (=AnsiChar)
  //so in this case we can use Length (instead of Utf8length) and iterate single chars in the string
  if (Length(AEditMask) >= 4) and (AEditMask[Length(AEditMask)-1] = MaskFieldSeparator) and
     (AEditMask[Length(AEditMask)-3] = MaskFieldSeparator) and
     (AEditMask[Length(AEditMask)-2] <> cMask_SpecialChar) and
     //Length = 4 is OK (AEditMask = ";1;_" for example), but if Length > 4 there must be no escape charater in front
     ((Length(AEditMask) = 4) or ((Length(AEditMask) > 4) and (AEditMask[Length(AEditMask)-4] <> cMask_SpecialChar))) then
  begin
    ASpaceChar := AEditMask[Length(AEditMask)];
    AMaskSave := (AEditMask[Length(AEditMask)-2] <> MaskNosave);
    System.Delete(AEditMask,Length(AEditMask)-3,4);
  end
  //If not both FMaskSave and FSPaceChar are specified, then see if only FMaskSave is specified
  else if (Length(AEditMask) >= 2) and (AEditMask[Length(AEditMask)-1] = MaskFieldSeparator) and
          //Length = 2 is OK, but if Length > 2 there must be no escape charater in front
          ((Length(AEditMask) = 2) or ((Length(AEditMask) > 2) and (AEditMask[Length(AEditMask)-2] <> cMask_SpecialChar))) then
  begin
    AMaskSave := (AEditMask[Length(AEditMask)] <> MaskNoSave);
    //Remove this bit from Mask
    System.Delete(AEditMask,Length(AEditMask)-1,2);
  end;
  //Whatever is left of AEditMask at this point is the MaskPart
  AMaskPart := AEditMask;
end;




function FormatMaskText(const EditMask: string; const AValue: string): string;
var
  Mu: TMaskUtils;
begin
  Mu := TMaskUtils.Create;
  try
    Mu.Mask := EditMask;
    Mu.Value := AValue;
    Result := Mu.ApplyMaskToText(AValue);
    Result := Mu.GetTextWithoutSpaceChar(Result);
  finally
    Mu.Free;
  end;
end;

function FormatMaskInput(const EditMask: string): string;
var
  Mu : TMaskUtils;
begin
  Result := '';
  Mu := TMaskUtils.Create;
  try
    Mu.Mask := EditMask;
    Result := Mu.InputMask;
  finally
    Mu.Free;
  end;
end;

{
  Format Value string using EditMask, dont use 2d and 3d fields of EditMask,
  set own SpaceChar and MaskSave = True ('1')
}
function MaskDoFormatText(const EditMask: string; const AValue: string; ASpaceChar: Char): string;
var
  Mu : TMaskUtils;
  AMaskPart: String;
  OldMaskSave: Boolean;
  OldSpaceChar: Char;
begin
  Result := '';
  SplitEditMask(EditMask, AMaskPart, OldMaskSave, OldSpaceChar);
  Mu := TMaskUtils.Create;
  try
    Mu.Mask := AMaskPart + ';1;'+ASpaceChar;
    Mu.Value := AValue;
    Result := Mu.ValidateInput;
  finally
    Mu.Free;
  end;
end;

{ TMaskUtils }

procedure TMaskUtils.AddToMask(Ch: Char);
begin
  FMask := FMask + Ch;
  FMaskLength := Length(FMask);
end;

function TMaskUtils.MaskToChar(AValue: tMaskedType): Char;
begin
  Result := Char(Ord(AValue));
end;

function TMaskUtils.CharToMask(Ch: Char): tMaskedType;
begin
  Result := Char_Start;
  if (Ord(Ch) > Ord(Char_Start)) and
     (Ord(Ch) < Ord(Char_Stop) )
     then
       Result := tMaskedType(Ord(Ch));
end;

function TMaskUtils.CharMatchesMask(const Ch: Char; const Position: Integer): Boolean;
var
  Current: tMaskedType;
  Ok: Boolean;
begin
  Result := False;
  if (Position < 1) or (Position > FMaskLength) then Exit;
  Current := CharToMask(FMask[Position]);
  case Current Of
    Char_Number              : OK := (Ch in ['0'..'9',#32]);
    Char_NumberFixed         : OK := (Ch in ['0'..'9']);
    Char_NumberPlusMin       : OK := (Ch in ['0'..'9','+','-',#32]);
    Char_Letter              : OK := (Ch in ['a'..'z', 'A'..'Z',#32]);
    Char_LetterFixed         : OK := (Ch in ['a'..'z', 'A'..'Z']);
    Char_LetterUpCase        : OK := (Ch in ['A'..'Z',#32]);
    Char_LetterDownCase      : OK := (Ch in ['a'..'z',#32]);
    Char_LetterFixedUpCase   : OK := (Ch in ['A'..'Z']);
    Char_LetterFixedDownCase : OK := (Ch in ['a'..'z']);
    Char_AlphaNum            : OK := (Ch in ['a'..'z', 'A'..'Z', '0'..'9',#32]);
    Char_AlphaNumFixed       : OK := (Ch in ['a'..'z', 'A'..'Z', '0'..'9']);
    Char_AlphaNumUpCase      : OK := (Ch in ['A'..'Z', '0'..'9',#32]);
    Char_AlphaNumDownCase    : OK := (Ch in ['a'..'z', '0'..'9',#32]);
    Char_AlphaNumFixedUpCase : OK := (Ch in ['A'..'Z', '0'..'9']);
    Char_AlphaNumFixedDowncase:OK := (Ch in ['a'..'z', '0'..'9']);
    Char_All                 : OK := True; //Ch in [#32..#126]; //True;
    Char_AllFixed            : OK := True; //Ch in [#32..#126]; //True;
    Char_AllUpCase           : OK := True; //Ch in [#32..#126]; // (Utf8UpperCase(Ch) = Ch);             ???????
    Char_AllDownCase         : OK := True; //Ch in [#32..#126]; // (Utf8LowerCase(Ch) = Ch);             ???????
    Char_AllFixedUpCase      : OK := True; //Ch in [#32..#126]; // (Utf8UpperCase(Ch) = Ch);             ???????
    Char_AllFixedDownCase    : OK := True; //Ch in [#32..#126]; // (Utf8LowerCase(Ch) = Ch);             ???????
   {Char_Space               : OK := (Length(Ch) = 1) and (Ch in [' ', '_']);  //not Delphi compatible, see notes above}
    Char_HourSeparator       : OK := (Ch = DefaultFormatSettings.TimeSeparator);
    Char_DateSeparator       : OK := (Ch = DefaultFormatSettings.DateSeparator);
    else//it's a literal
    begin
      OK := (Ch = FMask[Position]);
    end;
  end;//case
  //DebugLn('Position = ',DbgS(Position),' Current = ',MaskCharToChar[Current],' Ch = "',Ch,'" Ok = ',DbgS(Ok));
  Result := Ok;
end;

// Clear (virtually) a single char in position Position
function TMaskUtils.ClearChar(Position: Integer): Char;
begin
  //For Delphi compatibilty, only literals remain, all others will be blanked
  case CharToMask(FMask[Position]) Of
    Char_Number,
    Char_NumberFixed,
    Char_NumberPlusMin,
    Char_Letter,
    Char_LetterFixed,
    Char_LetterUpCase,
    Char_LetterDownCase,
    Char_LetterFixedUpCase,
    Char_LetterFixedDownCase,
    Char_AlphaNum,
    Char_AlphaNumFixed,
    Char_AlphaNumUpCase,
    Char_AlphaNumDownCase,
    Char_AlphaNumFixedUpcase,
    Char_AlphaNuMFixedDownCase,
    Char_All,
    Char_AllFixed,
    Char_AllUpCase,
    Char_AllDownCase,
    Char_AllFixedUpCase,
    Char_AllFixedDownCase: Result := FSpaceChar;
    Char_HourSeparator: Result := DefaultFormatSettings.TimeSeparator;
    Char_DateSeparator: Result := DefaultFormatSettings.DateSeparator;
  else
    Result := FMask[Position];
  end;
end;


procedure TMaskUtils.SetMask(AValue: String);
Var
  S, AMaskPart : String;
  I            : Integer;
  InUp, InDown : Boolean;
  Special      : Boolean;
  Ch           : Char;
begin
  if FRealMask <> AValue then
  begin
    FRealMask := AValue;
    FMask := '';
    SplitEditMask(FRealMask, AMaskPart, FMaskSave, FSpaceChar);

    // Construct Actual Internal Mask
    // init
    FTrimType := metTrimRight;
    // Init: No UpCase, No LowerCase, No Special Char
    InUp      := False;
    InDown    := False;
    Special   := False;
    S         := AMaskPart;
    for I := 1 to Length(S) do
    begin
      Ch := S[I];
      // Must insert a special char
      if Special then
      begin
        AddToMask(Ch);
        Special := False;
      end
      else
      begin
        // Check the char to insert

        case Ch Of
             cMask_SpecialChar: Special := True;
             cMask_UpperCase: begin
               if (I > 1) and (S[I-1] = cMask_LowerCase) then
               begin// encountered <>, so no case checking after this
                 InUp := False;
                 InDown := False
               end else
               begin
                 InUp    := True;
                 InDown := False;
               end;
             end;

             cMask_LowerCase: begin
                InDown  := True;
                InUp := False;
                // <> is catched by next cMask_Uppercase
             end;

             cMask_Letter: begin
                if InUp
                then
                  AddToMask(MaskToChar(Char_LetterUpCase))
                else
                  if InDown
                  then
                    AddToMask(MaskToChar(Char_LetterDownCase))
                  else
                    AddToMask(MaskToChar(Char_Letter))
             end;

             cMask_LetterFixed: begin
                if InUp
                then
                  AddToMask(MaskToChar(Char_LetterFixedUpCase))
                else
                  if InDown
                  then
                    AddToMask(MaskToChar(Char_LetterFixedDownCase))
                  else
                    AddToMask(MaskToChar(Char_LetterFixed))
             end;

             cMask_AlphaNum: begin
                 if InUp
                 then
                   AddToMask(MaskToChar(Char_AlphaNumUpcase))
                 else
                   if InDown
                   then
                     AddToMask(MaskToChar(Char_AlphaNumDownCase))
                   else
                     AddToMask(MaskToChar(Char_AlphaNum))
             end;

             cMask_AlphaNumFixed: begin
                 if InUp
                 then
                   AddToMask(MaskToChar(Char_AlphaNumFixedUpcase))
                 else
                   if InDown
                   then
                     AddToMask(MaskToChar(Char_AlphaNumFixedDownCase))
                   else
                     AddToMask(MaskToChar(Char_AlphaNumFixed))
             end;

             cMask_AllChars: begin
                if InUp
                then
                  AddToMask(MaskToChar(Char_AllUpCase))
                else
                  if InDown
                  then
                    AddToMask(MaskToChar(Char_AllDownCase))
                  else
                    AddToMask(MaskToChar(Char_All))
             end;

             cMask_AllCharsFixed: begin
                if InUp
                then
                  AddToMask(MaskToChar(Char_AllFixedUpCase))
                else
                  if InDown
                  then
                    AddToMask(MaskToChar(Char_AllFixedDownCase))
                  else
                    AddToMask(MaskToChar(Char_AllFixed))
             end;

             cMask_Number: AddToMask(MaskToChar(Char_Number));

             cMask_NumberFixed: AddToMask(MaskToChar(Char_NumberFixed));

             cMask_NumberPlusMin: AddToMask(MaskToChar(Char_NumberPlusMin));

             cMask_HourSeparator: AddToMask(MaskToChar(Char_HourSeparator));

             cMask_DateSeparator: AddToMask(MaskToChar(Char_DateSeparator));

             cMask_NoLeadingBlanks:
             begin
               FTrimType := metTrimLeft;
             end;

             else
             begin
               //It's a MaskLiteral
               AddToMask(Ch);
             end;
        end;
      end;
    end;
  end;
end;


function TMaskUtils.GetInputMask: String;
var
  i: Integer;
begin
  Result := '';
  for i := 1 to length(FMask) do
  begin
    case CharToMask(FMask[i]) of
      Char_Number,
      Char_NumberFixed,
      Char_NumberPlusMin,
      Char_Letter,
      Char_LetterFixed,
      Char_LetterUpCase,
      Char_LetterDownCase,
      Char_LetterFixedUpCase,
      Char_LetterFixedDownCase,
      Char_AlphaNum,
      Char_AlphaNumFixed,
      Char_AlphaNumUpCase,
      Char_AlphaNumDownCase,
      Char_AlphaNumFixedUpCase,
      Char_AlphaNumFixedDownCase,
      Char_All,
      Char_AllFixed,
      Char_AllUpCase,
      Char_AllDownCase,
      Char_AllFixedUpCase,
      Char_AllFixedDownCase: Result := Result + #32;
      Char_HourSeparator: Result := Result + DefaultFormatSettings.TimeSeparator;
      Char_DateSeparator: Result := Result + DefaultFormatSettings.DateSeparator;
      else Result := Result + FMask[i];   //it's a literal
    end;
  end;
end;

function TMaskUtils.GetTextWithoutMask(AValue: String): String;
{
  Replace al FSPaceChars with #32
  If FMaskSave = False then do trimming of spaces and remove all maskliterals
}
var
  S: String;
  i: Integer;
Begin
  S := StringReplace(AValue, FSpaceChar, #32, [rfReplaceAll]);
  //FSpaceChar can be used as a literal in the mask, so put it back
  for i := 1 to FMaskLength do
  begin
    if IsLiteral(FMask[i]) and (FMask[i] = FSpaceChar) then
    begin
      S[i] := FSpaceChar;
    end;
  end;
  if not FMaskSave then
  begin
    for i := 1 to FMaskLength do
    begin
      if IsLiteral(FMask[i]) then S[i] := #1;
    end;
    S := StringReplace(S, #1, '', [rfReplaceAll]);
    //Trimming only occurs if FMaskSave = False
    case FTrimType of
      metTrimLeft : S := TrimLeft(S);
      metTrimRight: S := TrimRight(S);
    end;//case
  end;
  Result := S;
End;


function TMaskUtils.GetTextWithoutSpaceChar(AValue: String): String;
var
  i: Integer;
begin
  Result := StringReplace(AValue, FSpaceChar, #32, [rfReplaceAll]);
  //FSpaceChar can be used as a literal in the mask, so put it back
  for i := 1 to FMaskLength do
  begin
    if IsLiteral(FMask[i]) and (FMask[i] = FSpaceChar) then
    begin
      Result[i] := FSpaceChar;
    end;
  end;
end;

function TMaskUtils.IsLiteral(Ch: Char): Boolean;
begin
  Result := (not IsMaskChar(Ch)) or
    (IsMaskChar(Ch) and (CharToMask(Ch) in [Char_HourSeparator, Char_DateSeparator]))
end;

function TMaskUtils.IsMaskChar(Ch: Char): Boolean;
begin
  Result := (CharToMask(Ch) <> Char_Start);
end;

procedure TMaskUtils.SetValue(AValue: String);
begin
  if FValue = AValue then Exit;
  FValue := AValue;
end;

function TMaskUtils.TextIsValid(const AValue: String): Boolean;
var
  i: Integer;
begin
  Result := False;
  if (Length(AValue) <> FMaskLength) then
  begin
    {$ifdef debug_maskutils}
    writeln('Length(AValue) = ',Length(AValue),' FMaskLength = ',FMaskLength);
    {$endif}
    Exit; //Actually should never happen??
  end;
  for i := 1 to FMaskLength do
  begin
    if not CharMatchesMask(AValue[i], i) then
    begin
      {$ifdef debug_maskutils}
      writeln('Fail: CharMatchesMask(',AValue[i],',',i,')  [',AValue,']');
      {$endif}
      Exit;
    end;
  end;
  Result := True;
end;

function TMaskUtils.ApplyMaskToText(AValue: String): String;
{ This tries to mimic Delphi behaviour (D3):
  - if mask contains no literals text is set, if necessary padded with blanks,
    LTR or RTL depending on FTrimType
  - if mask contains literals then we search for matching literals in text and
    process each "segment" between matching maskliterals, trimming or padding
    LTR or RTL depending on FTrimType, until there is no more matching maskliteral
    Some examples to clarify:
    EditMask        Text to be set    Result
    99              1                 1_
    !99             1                 _1
    cc-cc           1-2               1_-2_
    !cc-cc          1-2               _1-_2
    cc-cc@cc        1-2@3             1_-2_@3_
                    12@3              12-__@__
    cc-cc@cc        123-456@789       12-45@78
    !cc-cc@cc       123-456@789       23-56@89
    This feauture seems to be invented for easy use of dates:

    99/99/00        23/1/2009         23/1_/20  <- if your locale DateSeparator = '/'
    !99/99/00       23/1/2009         23/_1/09  <- if your locale DateSeparator = '/'

  - The resulting text will always have length = FMaskLength
  - The text that is set, does not need to validate
}
//Helper functions
  Function FindNextMaskLiteral(const StartAt: Integer; out FoundAt: Integer; out ALiteral: Char): Boolean;
  var i: Integer;
  begin
    Result := False;
    for i := StartAt to FMaskLength do
    begin
      if IsLiteral(FMask[i]) then
      begin
        FoundAt := i;
        ALiteral := ClearChar(i);
        Result := True;
        Exit;
      end;
    end;
  end;
  Function FindMatchingLiteral(const Value: String; const ALiteral: Char; out FoundAt: Integer): Boolean;
  begin
    FoundAt := Pos(ALiteral, Value);
    Result := (FoundAt > 0);
  end;

Var
  S                   : String;
  I, J                : Integer;
  mPrevLit, mNextLit  : Integer; //Position of Previous and Next literal in FMask
  vNextLit            : Integer; //Position of next matching literal in AValue
  HasNextLiteral,
  HasMatchingLiteral,
  Stop                : Boolean;
  Literal             : Char;
  Sub                 : String;
begin
  //First setup a "blank" string that contains all literals in the mask
  S := '';
  for I := 1 To FMaskLength do  S := S + ClearChar(I);

  if FMaskSave then
  begin
    mPrevLit := 0;
    Stop := False;
    HasNextLiteral := FindNextMaskLiteral(mPrevLit+1, mNextLit, Literal);
    //if FMask starts with a literal, then the first CodePoint of AValue must be that literal
    if HasNextLiteral and (mNextLit = 1) and (AValue[1] <> Literal) then Stop := True;
    //debugln('HasNextLiteral = ',dbgs(hasnextliteral),', Stop = ',dbgs(stop));
    While not Stop do
    begin
      if HasNextLiteral then
      begin
        HasMatchingLiteral := FindMatchingLiteral(AValue, Literal, vNextLit);
        //debugln('mPrevLit = ',dbgs(mprevlit),' mNextLit = ',dbgs(mnextlit));
        //debugln('HasMatchingLiteral = ',dbgs(hasmatchingliteral));
        if HasMatchingLiteral then
        begin
          //debugln('vNextLit = ',dbgs(vnextlit));
          Sub := Copy(AValue, 1, vNextLit - 1); //Copy up to, but not including matching literal
          Delete(AValue, 1, vNextLit); //Remove this bit from AValue (including matching literal)
          if (Length(AValue) = 0) then Stop := True;
          //debugln('Sub = "',Sub,'", Value = "',AValue,'"');
        end
        else
        begin//HasMatchingLiteral = False
          Stop := True;
          Sub := AValue;
          AValue := '';
          //debugln('Sub = "',Sub,'", Value = "',AValue,'"');
        end;
        //fill S between vPrevLit + 1 and vNextLit - 1, LTR or RTL depending on FTrimType
        if (FTrimType = metTrimRight) then
        begin
          j := 1;
          for i := (mPrevLit + 1) to (mNextLit - 1) do
          begin
            if (J > Length(Sub)) then Break;
            if (Sub[j] = #32) then S[i] := FSpaceChar else S[i] := Sub[j];
            Inc(j);
          end;
        end
        else
        begin//FTrimType = metTrimLeft
          j := Length(Sub);
          for i := (mNextLit - 1) downto (mPrevLit + 1) do
          begin
            if (j < 1) then Break;
            if (Sub[j] = #32) then S[i] := FSpaceChar else S[i] :=  Sub[j];
            Dec(j);
          end;
        end;
        //debugln('S = ',S);
      end
      else
      begin//HasNextLiteral = False
        //debugln('No more MaskLiterals at this point');
        //debugln('mPrevLit = ',dbgs(mprevlit));
        Stop := True;
        Sub := AValue;
        AValue := '';
        //debugln('Sub = "',Sub,'", Value = "',AValue,'"');
        //fill S from vPrevLit + 1 until end of FMask, LTR or RTL depending on FTrimType
        if (FTrimType = metTrimRight) then
        begin
          j := 1;
          for i := (mPrevLit + 1) to FMaskLength do
          begin
            //debugln('  i = ',dbgs(i),'  j = ',dbgs(j));
            if (j > Length(Sub)) then Break;
            if (Sub[j] = #32) then S[i] := FSpaceChar else S[i] := Sub[j];
            //debugln('  Sub[j] = "',Sub[j],'" -> S = ',S);
            Inc(j);
          end;
        end
        else
        begin//FTrimType = metTrimLeft
          j := Length(Sub);
          for i := FMaskLength downto (mPrevLit + 1) do
          begin
            //debugln('  i = ',dbgs(i),'  j = ',dbgs(j));
            if (j < 1) then Break;
            if (Sub[j] = #32) then S[i] := FSpaceChar else S[i] := Sub[j];
            //debugln('  Sub[j] = "',Sub[j],'" -> S = ',S);
            Dec(j);
          end;
        end;
        //debugln('S = ',S);
      end;
      //debugln('Stop = ',dbgs(stop));
      if not Stop then
      begin
        mPrevLit := mNextLit;
        HasNextLiteral := FindNextMaskLiteral(mPrevLit + 1, mNextLit, Literal);
      end;
    end;//while not Stop
  end//FMaskSave = True
  else
  begin//FMaskSave = False
    if FTrimType = metTrimRight then
    begin
      //fill text from left to rigth, skipping MaskLiterals
      j := 1;
      for i := 1 to FMaskLength do
      begin
        if not IsLiteral(FMask[i]) then
        begin
          if (AValue[j] = #32) then S[i]:= FSpaceChar else S[i] := AValue[j];
          Inc(j);
          if j > Length(AValue) then Break;
        end;
      end;
    end
    else
    begin
      //fill text from right to left, skipping MaskLiterals
      j := Length(AValue);
      for i := FMaskLength downto 1 do
      begin
        if not IsLiteral(FMask[i]) then
        begin
          if (AValue[j] = #32) then S[i] := FSpaceChar else S[i] := AValue[j];
          Dec(j);
          if j < 1 then Break;
        end;
      end;
    end;
  end;//FMaskSave = False
  Result := S;
end;

function TMaskUtils.ValidateInput: String;
begin
  if not TryValidateInput(Result) then
    raise Exception.Create(exValidationFailed);
end;

function TMaskUtils.TryValidateInput(out ValidatedString: String): Boolean;
var
  SMaskApplied, SMaskRemoved: String;
  _MaskSave: Boolean;
begin
  _MaskSave := FMaskSave;
  //Note: applying the mask and then removing it is not reciprocal!
  SMaskApplied := ApplyMaskToText(Value);
  FMaskSave := True;
  SMaskRemoved := GetTextWithoutMask(SMaskApplied);
  FMaskSave := _MaskSave;
  Result := TextIsValid(SMaskRemoved);
  if Result then
    ValidatedString := GetTextWithoutSpaceChar(SMaskApplied);
end;



end.

