{
 /***************************************************************************
                                  maskutils.pas
                                  ---------

 ***************************************************************************/

 *****************************************************************************
 *                                                                           *
 *  This file is part of the Lazarus Component Library (LCL)                 *
 *                                                                           *
 *  See the file COPYING.modifiedLGPL, included in this distribution,        *
 *  for details about the copyright.                                         *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                     *
 *                                                                           *
 *****************************************************************************
 

 Author: Boguslaw Brandys
 
 Abstract:
    FormatMaskText implementation
 
}


unit maskutils;

{$mode objfpc}{$H+}
{.$define DebugMaskUtils}




interface

uses
Classes
,SysUtils
 {$ifdef DebugMaskUtils}
,lclproc
 {$endif};



function FormatMaskText(const EditMask: string; const Value: string): string;
function FormatMaskInput(const EditMask: string): string;
<<<<<<< HEAD
<<<<<<< HEAD
function MaskDoFormatText(const EditMask: string; const Value: string; Blank: Char): string;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2



type
  TStepState =
  (
  stLeading,  //? not used currently
  stUpper,    //use uppercase
  stLower,    //use lowercase
  stSpecial,  //use escape character
  stArbitrary //put arbitrary character
  );

  TParseState = set of TStepState;
<<<<<<< HEAD
<<<<<<< HEAD
  
  TEditMask = type string;
=======


>>>>>>> graemeg/fixes_2_2
=======


>>>>>>> origin/fixes_2_2

{ TMaskUtils }

type
  TMaskUtils = class(TObject)
<<<<<<< HEAD
<<<<<<< HEAD
  private
=======
    private
>>>>>>> graemeg/fixes_2_2
=======
    private
>>>>>>> origin/fixes_2_2
    FValue: string;
    SourcePosition,Position : Integer;
    FEditMask,FMask : string;
    SourceVal,ExitVal : string;
<<<<<<< HEAD
<<<<<<< HEAD
    FMatched : Boolean;
    FMissChar : Char;
=======
    Matched : Boolean;
    MissChar : Char;
>>>>>>> graemeg/fixes_2_2
=======
    Matched : Boolean;
    MissChar : Char;
>>>>>>> origin/fixes_2_2
    State : TParseState;
    procedure EvaluateExit;
    procedure EvaluateMissing;
    procedure DoFillRest;
    procedure DoLiteral;
    procedure DoLiteralInputMask;
    procedure DoToken;
    procedure DoTokenInputMask;
    procedure DoUpper;
    procedure DoLower;
    procedure DoNumeric(Required : Boolean);
    procedure DoAlpha(Required : Boolean);
    procedure DoAlphaNumeric(Required : Boolean);
    procedure DoNumericPlusMinus;
    procedure DoArbitrary(Required : Boolean);
    procedure DoTime;
    procedure DoDate;
    function GetInputMask: string;
    procedure SetMask(const AValue: string);
    procedure SetValue(const AValue: string);
<<<<<<< HEAD
<<<<<<< HEAD
  protected
=======
    protected
>>>>>>> graemeg/fixes_2_2
=======
    protected
>>>>>>> origin/fixes_2_2
    procedure RaiseError;
    procedure ExtractMask;
    function MaskPtr : Char;
    function SourcePtr : Char;
<<<<<<< HEAD
<<<<<<< HEAD
    property Matched: Boolean read FMatched write FMatched;
    property MissChar: Char read FMissChar write FMissChar;
  public
=======
    public
>>>>>>> graemeg/fixes_2_2
=======
    public
>>>>>>> origin/fixes_2_2
    function ValidateInput : string;
    property Mask : string read FEditMask write SetMask;
    property Value : string read FValue write SetValue;
    property InputMask : string read GetInputMask;
  end;





implementation


resourcestring

//exInvalidMaskValue = 'Input mask value incorrect';
exInvalidMaskValue = 'FormatMaskText function failed!';
//replace above text when all bugs will be fixed!








function IsNumeric(const C : Char) : Boolean;
begin
  Result := C In ['0'..'9'];
end;



function IsAlpha(const C : Char) : Boolean;
begin
  //Fix it later if better way is possible
  Result := AnsiUpperCase(C) <> AnsiLowerCase(C);
end;


function IsToken(const C : Char) : Boolean;
begin
  Result := C In ['!','>','<','\','L','l','A','a','C','c','0','9','#',':',
            '/',';'];
end;



{ TMaskUtils }





procedure TMaskUtils.SetMask(const AValue: string);
begin
  if FEditMask = AValue then Exit;
  FEditMask := AValue;
  ExtractMask;
end;

procedure TMaskUtils.SetValue(const AValue: string);
begin
  if SourceVal=AValue then exit;
  SourceVal := AValue;
end;



function TMaskUtils.ValidateInput : string;
begin
 {Prepare}
  ExitVal := '';
  Position := 1;
  SourcePosition := 1;
  State := [];

 {Process}
  while (Position <= Length(FMask)) do
    begin
      if (IsToken(MaskPtr) and not (stSpecial In State)) then
       DoToken
      else
        DoLiteral;

      Inc(Position);
    end;

  DoFillRest;
  Result := ExitVal;
end;






procedure TMaskUtils.EvaluateMissing;
begin
  ExitVal := ExitVal + MissChar;
  Inc(SourcePosition);
end;


procedure TMaskUtils.RaiseError;inline;
begin
  if SourcePosition > Length(SourceVal) then
    EvaluateMissing
  else
    raise Exception.CreateFmtHelp(exInvalidMaskValue,[],Position);
end;



function TMaskUtils.MaskPtr : Char;
begin
  Result := FMask[Position];
end;



function TMaskUtils.SourcePtr : Char;
begin
  if SourcePosition <= Length(SourceVal) then
    Result := SourceVal[SourcePosition]
  else Result := #0;
end;






{Extract mask from input parameter}
procedure TMaskUtils.ExtractMask;
var
  P : Integer;
  s : string;
begin
  { TODO:  Implement clear, UTF8 compliant parsing ? }
  MissChar := #32;
  Matched := false;
  s := Copy(FEditMask,1,Length(FEditMask));
  P := LastDelimiter(';',s);
  if P = 0 then FMask := s
  else
    begin
      MissChar := PChar(Copy(s,P+1,1))^;
      Delete(s,P,2);
      P := LastDelimiter(';',s);
      Matched := (Copy(s,P+1,1) <> '0');
      Delete(s,P,2);
      FMask := s;
    end;
end;




procedure TMaskUtils.EvaluateExit;
begin
  if stUpper in State then
   ExitVal := ExitVal + UpperCase(SourcePtr)
  else
    if stLower in State then
     ExitVal := ExitVal + LowerCase(SourcePtr)
  else
    ExitVal := ExitVal + SourcePtr;
  Inc(SourcePosition);
end;




procedure TMaskUtils.DoUpper;
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoUpper',',Position=',Position]);
 {$endif}
  if stLower in State then
   Exclude(State,stLower)
  else
    Include(State,stUpper);

  {Ugly check for '<>' sequence. Is that required ?}
  if (Position > 1) and (FMask[Position-1] = '<') then
    begin
      Exclude(State,stLower);
      Exclude(State,stUpper);
    end;
end;

procedure TMaskUtils.DoLower;
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoLower',',Position=',Position]);
 {$endif}
  if stUpper in State then
   Exclude(State,stUpper)
  else
    Include(State,stLower);
end;


procedure TMaskUtils.DoAlphaNumeric(Required : Boolean);
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoAlphaNumeric',',Position=',Position]);
 {$endif}
  if Required then
    begin
      if (IsAlpha(SourcePtr) or IsNumeric(SourcePtr)) then EvaluateExit
      else
        RaiseError;
    end
  else
    begin
      if (IsAlpha(SourcePtr) or IsNumeric(SourcePtr)) then EvaluateExit
      else
        EvaluateMissing;
    end;

end;

procedure TMaskUtils.DoArbitrary(Required : Boolean);
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoArbitrary',',Position=',Position]);
 {$endif}
  Include(State,stArbitrary);
  if Required then
    begin
      if Position > Length(SourceVal) then  RaiseError;
    end
  else
    begin
      if Position > Length(SourceVal) then  EvaluateMissing
      else
        EvaluateExit;
    end;
end;


procedure TMaskUtils.DoNumeric(Required : Boolean);
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoNumeric',',Position=',Position]);
 {$endif}
  if Required then
    begin
      if IsNumeric(SourcePtr) then EvaluateExit
      else
        RaiseError;
    end
  else
    begin
      if IsNumeric(SourcePtr) then EvaluateExit
      else
        EvaluateMissing;
    end;
end;

procedure TMaskUtils.DoNumericPlusMinus;
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoNumericPlusMinus',',Position=',Position]);
 {$endif}
  if (IsNumeric(SourcePtr)) or
     (SourcePtr = '+') or
     (SourcePtr = '-') then
     EvaluateExit
  else
    EvaluateMissing;
end;


procedure TMaskUtils.DoTime;
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoTime',',Position=',Position]);
 {$endif}
  ExitVal := ExitVal + TimeSeparator;
end;



procedure TMaskUtils.DoDate;
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoDate',',Position=',Position]);
 {$endif}
  ExitVal := ExitVal + DateSeparator;
end;

function TMaskUtils.GetInputMask: string;
begin
 {Prepare}
  ExitVal := '';
  Position := 1;
  State := [];

 {Process}
  while (Position <= Length(FMask)) do
    begin
      if (IsToken(MaskPtr) and not (stSpecial In State)) then
       DoTokenInputMask
      else
        DoLiteralInputMask;

      Inc(Position);
    end;

  Result := ExitVal;
end;





procedure TMaskUtils.DoAlpha(Required : Boolean);
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoAlpha',',Position=',Position]);
 {$endif}
  if Required then
    begin
      if IsAlpha(SourcePtr) then
      EvaluateExit
      else
        RaiseError;
    end
  else
    begin
      if IsAlpha(SourcePtr) then
      EvaluateExit
      else
        EvaluateMissing;
    end;
end;


procedure TMaskUtils.DoToken;
begin
  if stArbitrary in State then Exclude(State,stArbitrary);

  case MaskPtr of
    '!' : Include(State,stLeading);
    '>' : DoUpper;
    '<' : DoLower;
    '\' : Include(State,stSpecial);
    'L' : DoAlpha(true);
    'l' : DoAlpha(false);
    'A' : DoAlphaNumeric(true);
    'a' : DoAlphaNumeric(false);
    'C' : DoArbitrary(true);
    'c' : DoArbitrary(false);
    '0' : DoNumeric(true);
    '9' : DoNumeric(false);
    '#' : DoNumericPlusMinus;
    ':' : DoTime;
    '/' : DoDate;
  end;

end;



procedure TMaskUtils.DoTokenInputMask;
begin

  case MaskPtr of
    '!',
    '>',
    '<' : ;{nothing}
    '\' : Include(State,stSpecial);
    'L',
    'l',
    'A',
    'a',
    'C',
    'c',
    '0',
    '9',
    '#' : ExitVal := ExitVal + MissChar;
    ':' : DoTime;
    '/' : DoDate;
  end;

end;


procedure TMaskUtils.DoLiteral;
begin
 {$ifdef DebugMaskUtils}
  DebugLn(['DoLiteral',',Position=',Position]);
 {$endif}
  if stSpecial in State then
   Exclude(State,stSpecial);
  if Matched and (MaskPtr <> SourcePtr) then
   RaiseError;
  if Matched or not (IsAlpha(SourcePtr) or IsNumeric(SourcePtr)) then
   Inc(SourcePosition);
  ExitVal := ExitVal + MaskPtr;
end;


procedure TMaskUtils.DoLiteralInputMask;
begin
  if stSpecial in State then
   Exclude(State,stSpecial);
  ExitVal := ExitVal + MaskPtr;
end;



procedure TMaskUtils.DoFillRest;
var
  i : Integer;
begin

{Fill rest of exit value because source is longer then mask
and the last mask character permit arbitrary char.
Compatibility with delphi}

  if (stArbitrary in State) then
    begin
      i := Length(SourceVal) - Length(FMask);
      while i >= 0 do
        begin
          EvaluateExit;
          Dec(i);
        end;
    end;
end;




function FormatMaskText(const EditMask: string; const Value: string): string;
var
  msk : TMaskUtils;
begin
  Result := '';
  msk := TMaskUtils.Create;
  try
    msk.Mask := EditMask;
    msk.Value := Value;
    Result := msk.ValidateInput;
  finally
    msk.Free;
  end;
end;

{Returns preprocessed mask (without escape characters, with currect locale date
and time separators) }
function FormatMaskInput(const EditMask: string): string;
var
  msk : TMaskUtils;
begin
  Result := '';
  msk := TMaskUtils.Create;
  try
    msk.Mask := EditMask;
    Result := msk.InputMask;
  finally
    msk.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
{
  Format Value string using EditMask, dont use 2d and 3d fields of EditMask,
  set own Blank char and Matched = False
}
function MaskDoFormatText(const EditMask: string; const Value: string; Blank: Char): string;
var
  msk : TMaskUtils;
begin
  Result := '';
  msk := TMaskUtils.Create;
  try
    msk.Mask := EditMask;
    msk.Value := Value;
    msk.Matched := False;
    msk.MissChar := Blank;
    Result := msk.ValidateInput;
  finally
    msk.Free;
  end;
end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

end.

