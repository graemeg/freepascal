{
    *********************************************************************
    Copyright (C) 1997, 1998 Gertjan Schouten

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

    System Utilities For Free Pascal
}

{==============================================================================}
{   standard functions                                                         }
{==============================================================================}

type
   PString = ObjPas.PString;
   TLocaleOptions = (loInvariantLocale, loUserLocale);

   { For FloatToText }
   TFloatFormat = (ffGeneral, ffExponent, ffFixed, ffNumber, ffCurrency);
   TFloatValue = (fvExtended, fvCurrency, fvSingle, fvReal, fvDouble, fvComp);
   TReplaceFlags = set of (rfReplaceAll, rfIgnoreCase);

   TFloatRec = Record
      Exponent: Integer;
      Negative: Boolean;
      Digits: Array[0..18] Of Char;
   End;

const
  { For floattodatetime and VariantToDate }
{$ifndef FPUNONE}
  MinDateTime: TDateTime =  -693593.99999999; { 01/01/0001 12:00:00.000 AM }
  MaxDateTime: TDateTime =  2958465.99999999;  { 12/31/9999 11:59:59.999 PM }

{$if defined(FPC_HAS_TYPE_EXTENDED) or defined(FPC_HAS_TYPE_FLOAT128)}
  MinCurrency: Currency = -922337203685477.5808;
  MaxCurrency: Currency =  922337203685477.5807;
{$else}
  MinCurrency: Currency = -922337203685477.0000;
  MaxCurrency: Currency =  922337203685477.0000;
{$endif}
{$else}
  MinCurrency: Currency = -922337203685477;
  MaxCurrency: Currency =  922337203685477;
{$endif}

Const
  LeadBytes: set of Char = [];
  EmptyStr : string = '';
  NullStr : PString = @EmptyStr;

  EmptyWideStr : WideString = '';
//  NullWideStr : PWideString = @EmptyWideStr;

Var TrueBoolStrs,
    FalseBoolStrs : Array of String;

// declaring this breaks delphi compatibility and e.g. tw3721.pp
// function NewStr(Const S: ShortString): PShortString; overload;
function NewStr(const S: string): PString; overload;
procedure DisposeStr(S: PString); overload;
procedure DisposeStr(S: PShortString); overload;
procedure AssignStr(var P: PString; const S: string); {$ifdef SYSUTILSINLINE}inline;{$endif}
procedure AppendStr(var Dest: String; const S: string);{$ifdef SYSUTILSINLINE}inline;{$endif}
function UpperCase(const s: string): string; overload;
function UpperCase(const s: string; LocaleOptions: TLocaleOptions): string; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
function LowerCase(const s: string): string; overload;
function LowerCase(const s: string; LocaleOptions: TLocaleOptions): string; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
{ the compiler can't decide else if it should use the char or the ansistring
  version for a variant }
function LowerCase(const V: variant): string; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
function CompareStr(const S1, S2: string): Integer; overload;
function CompareStr(const S1, S2: string; LocaleOptions: TLocaleOptions): Integer; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
function CompareMemRange(P1, P2: Pointer; Length: PtrUInt): integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
function CompareMem(P1, P2: Pointer; Length: PtrUInt): Boolean;{$ifdef SYSUTILSINLINE}inline;{$endif}
function CompareText(const S1, S2: string): Integer; overload;
function CompareText(const S1, S2: string; LocaleOptions: TLocaleOptions): Integer; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
function SameText(const s1,s2:String):Boolean; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
function SameText(const s1,s2:String; LocaleOptions: TLocaleOptions):Boolean; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
function SameStr(const s1,s2:String):Boolean; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
function SameStr(const s1,s2:String; LocaleOptions: TLocaleOptions):Boolean; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}

function AnsiUpperCase(const s: string): string;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiLowerCase(const s: string): string;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiCompareStr(const S1, S2: string): Integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiCompareText(const S1, S2: string): Integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiSameText(const s1,s2:String):Boolean;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiSameStr(const s1,s2:String):Boolean;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiStrComp(S1, S2: PChar): Integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiStrIComp(S1, S2: PChar): Integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiStrLComp(S1, S2: PChar; MaxLen: SizeUInt): Integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiStrLIComp(S1, S2: PChar; MaxLen: SizeUInt): Integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiStrLower(Str: PChar): PChar;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiStrUpper(Str: PChar): PChar;{$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiLastChar(const S: string): PChar;
function AnsiStrLastChar(Str: PChar): PChar;

function Trim(const S: string): string;
function TrimLeft(const S: string): string;
function TrimRight(const S: string): string;
function QuotedStr(const S: string): string; {$ifdef SYSUTILSINLINE}inline;{$endif}
function AnsiQuotedStr(const S: string; Quote: char): string;
function AnsiDequotedStr(const S: string; AQuote: Char): string;
function AnsiExtractQuotedStr(var Src: PChar; Quote: Char): string;
function AdjustLineBreaks(const S: string): string;
function AdjustLineBreaks(const S: string; Style: TTextLineBreakStyle): string;
function IsValidIdent(const Ident: string; AllowDots: Boolean = False; StrictDots: Boolean = False): Boolean;
function IntToStr(Value: Longint): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToStr(Value: Int64): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToStr(Value: QWord): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function UIntToStr(Value: QWord): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function UIntToStr(Value: Cardinal): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: Longint; Digits: integer): string;
function IntToHex(Value: Int64; Digits: integer): string;
function IntToHex(Value: QWord; Digits: integer): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: Int8): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: UInt8): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: Int16): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: UInt16): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: Int32): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: UInt32): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: Int64): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function IntToHex(Value: UInt64): string; {$ifdef SYSUTILSINLINE}inline;{$ENDIF}
function StrToInt(const s: string): Longint;
function StrToDWord(const s: string): DWord;
function StrToUInt(const s: string): Cardinal;
function StrToInt64(const s: string): int64;
function StrToQWord(const s: string): QWord; 
function StrToUInt64(const s: string): UInt64; inline;
function TryStrToInt(const s: string; Out i : Longint) : boolean;
function TryStrToDWord(const s: string; Out D : DWord) : boolean;
function TryStrToUInt(const s: string; out C: Cardinal): Boolean;
function TryStrToInt64(const s: string; Out i : int64) : boolean;
function TryStrToQWord(const s: string; Out Q : QWord) : boolean;
function TryStrToUInt64(const s: string; Out u : UInt64) : boolean; inline;
function StrToIntDef(const S: string; Default: Longint): Longint;
function StrToDWordDef(const S: string; Default: DWord): DWord;
function StrToUIntDef(const S: string; Default: Cardinal): Cardinal;
function StrToInt64Def(const S: string; Default: int64): int64;
function StrToQWordDef(const S: string; Default: QWord): QWord;
function StrToUInt64Def(const S: string; Default: UInt64): UInt64; inline;
function LoadStr(Ident: integer): string;{$ifdef SYSUTILSINLINE}inline;{$ENDIF}
// function FmtLoadStr(Ident: integer; const Args: array of const): string;
Function Format (Const Fmt : String; const Args : Array of const) : String;
Function Format (Const Fmt: string; const Args: array of const; const FormatSettings: TFormatSettings): string;
Function FormatBuf (Var Buffer; BufLen : Cardinal; Const Fmt; fmtLen : Cardinal; Const Args : Array of const) : Cardinal;
Function FormatBuf (Var Buffer; BufLen : Cardinal; Const Fmt; fmtLen : Cardinal; Const Args : Array of const; Const FormatSettings: TFormatSettings) : Cardinal;
Function StrFmt(Buffer,Fmt : PChar; Const args: Array of const) : Pchar;
Function StrFmt(Buffer,Fmt : PChar; Const Args: Array of const; Const FormatSettings: TFormatSettings): PChar;
Function StrLFmt(Buffer : PCHar; Maxlen : Cardinal;Fmt : PChar; Const args: Array of const) : Pchar;
Function StrLFmt(Buffer : PCHar; Maxlen : Cardinal;Fmt : PChar; Const args: Array of const; Const FormatSettings: TFormatSettings) : Pchar;
Procedure FmtStr(Var Res: String; Const Fmt : String; Const args: Array of const);
Procedure FmtStr(Var Res: string; const Fmt : string; Const args: Array of const; Const FormatSettings: TFormatSettings);
{$ifndef FPUNONE}
{$ifdef FPC_HAS_TYPE_EXTENDED}
Function FloatToStrF(Value: Extended; format: TFloatFormat; Precision, Digits: Integer): String;
Function FloatToStrF(Value: Extended; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
{$endif FPC_HAS_TYPE_EXTENDED}
Function FloatToStrF(Value: Double; format: TFloatFormat; Precision, Digits: Integer): String;
Function FloatToStrF(Value: Double; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
Function FloatToStrF(Value: Single; format: TFloatFormat; Precision, Digits: Integer): String;
Function FloatToStrF(Value: Single; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
Function FloatToStrF(Value: Comp; format: TFloatFormat; Precision, Digits: Integer): String;
Function FloatToStrF(Value: Comp; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
Function FloatToStrF(Value: Currency; format: TFloatFormat; Precision, Digits: Integer): String;
Function FloatToStrF(Value: Currency; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
{$ifndef FPC_COMP_IS_INT64}
Function FloatToStrF(Value: Int64; format: TFloatFormat; Precision, Digits: Integer): String;
Function FloatToStrF(Value: Int64; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
{$endif FPC_COMP_IS_INT64}
Function CurrToStrF(Value: Currency; Format: TFloatFormat; Digits: Integer): string;
Function CurrToStrF(Value: Currency; Format: TFloatFormat; Digits: Integer; Const FormatSettings: TFormatSettings): string;
{$ifdef FPC_HAS_TYPE_EXTENDED}
Function FloatToStr(Value: Extended): String;
Function FloatToStr(Value: Extended; Const FormatSettings: TFormatSettings): String;
{$endif FPC_HAS_TYPE_EXTENDED}
Function FloatToStr(Value: Double): String;
Function FloatToStr(Value: Double; Const FormatSettings: TFormatSettings): String;
Function FloatToStr(Value: Single): String;
Function FloatToStr(Value: Single; Const FormatSettings: TFormatSettings): String;
Function FloatToStr(Value: Currency): String;
Function FloatToStr(Value: Currency; Const FormatSettings: TFormatSettings): String;
Function FloatToStr(Value: Comp): String;
Function FloatToStr(Value: Comp; Const FormatSettings: TFormatSettings): String;
{$ifndef FPC_COMP_IS_INT64}
Function FloatToStr(Value: Int64): String;
Function FloatToStr(Value: Int64; Const FormatSettings: TFormatSettings): String;
{$endif FPC_COMP_IS_INT64}
Function StrToFloat(Const S : String) : Extended;
Function StrToFloat(Const S : String; Const FormatSettings: TFormatSettings) : Extended;
Function StrToFloatDef(Const S: String; Const Default: Extended): Extended;
Function StrToFloatDef(Const S: String; Const Default: Extended; Const FormatSettings: TFormatSettings): Extended;
Function TryStrToFloat(Const S : String; Out Value: Single): Boolean;
Function TryStrToFloat(Const S : String; Out Value: Single; Const FormatSettings: TFormatSettings): Boolean;
Function TryStrToFloat(Const S : String; Out Value: Double): Boolean;
Function TryStrToFloat(Const S : String; Out Value: Double; Const FormatSettings: TFormatSettings): Boolean;
{$ifdef FPC_HAS_TYPE_EXTENDED}
Function TryStrToFloat(Const S : String; Out Value: Extended): Boolean;
Function TryStrToFloat(Const S : String; Out Value: Extended; Const FormatSettings: TFormatSettings): Boolean;
{$endif FPC_HAS_TYPE_EXTENDED}
Function TextToFloat(Buffer: PChar; Out Value: Extended): Boolean;
Function TextToFloat(Buffer: PChar; Out Value: Extended; Const FormatSettings: TFormatSettings): Boolean;
Function TextToFloat(Buffer: PChar; Out Value; ValueType: TFloatValue): Boolean;
Function TextToFloat(Buffer: PChar; Out Value; ValueType: TFloatValue; Const FormatSettings: TFormatSettings): Boolean;
Function FloatToText(Buffer: PChar; Value: Extended; format: TFloatFormat; Precision, Digits: Integer): Longint;
Function FloatToText(Buffer: PChar; Value: Extended; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): Longint;
Function FloatToDateTime (Const Value : Extended) : TDateTime;
Function FloattoCurr (Const Value : Extended) : Currency;
function TryFloatToCurr(const Value: Extended; var AResult: Currency): Boolean;

Function CurrToStr(Value: Currency): string;
Function CurrToStr(Value: Currency; Const FormatSettings: TFormatSettings): string;
function StrToCurr(const S: string): Currency;
function StrToCurr(const S: string; Const FormatSettings: TFormatSettings): Currency;
function TryStrToCurr(const S: string;Out Value : Currency): Boolean;
function TryStrToCurr(const S: string;Out Value : Currency; Const FormatSettings: TFormatSettings): Boolean;
function StrToCurrDef(const S: string; Default : Currency): Currency;
function StrToCurrDef(const S: string; Default : Currency; Const FormatSettings: TFormatSettings): Currency;

Function FloatToTextFmt(Buffer: PChar; Value: Extended; format: PChar; FormatSettings : TFormatSettings): Integer;
Function FloatToTextFmt(Buffer: PChar; Value: Extended; format: PChar): Integer;
Procedure FloatToDecimal(Out Result: TFloatRec; const Value; ValueType: TFloatValue; Precision, Decimals : integer);
Procedure FloatToDecimal(Out Result: TFloatRec; Value: Extended; Precision, Decimals : integer);
Function FormatFloat(Const Format : String; Value : Extended) : String;
Function FormatFloat(Const Format : String; Value : Extended; Const FormatSettings: TFormatSettings) : String;

function FormatCurr(const Format: string; Value: Currency): string;
Function FormatCurr(const Format: string; Value: Currency; Const FormatSettings: TFormatSettings): string;
{$endif}

function StrToBool(const S: string): Boolean;
function StrToBool(const S: string; Const FormatSettings: TFormatSettings): Boolean;
function BoolToStr(B: Boolean;UseBoolStrs:Boolean=False): string;
function BoolToStr(B: Boolean;const TrueS,FalseS:string): string; inline;
function StrToBoolDef(const S: string; Default: Boolean): Boolean;
function StrToBoolDef(const S: string; Default: Boolean; Const FormatSettings: TFormatSettings): Boolean;
function TryStrToBool(const S: string; out Value: Boolean): Boolean;
function TryStrToBool(const S: string; out Value: Boolean; Const FormatSettings: TFormatSettings): Boolean;

function LastDelimiter(const Delimiters, S: string): SizeInt;
function StringReplace(const S, OldPattern, NewPattern: string;  Flags: TReplaceFlags; Out aCount : Integer): string;
function StringReplace(const S, OldPattern, NewPattern: string;  Flags: TReplaceFlags): string;
Function IsDelimiter(const Delimiters, S: string; Index: SizeInt): Boolean;

function SScanf(const s: string; const fmt : string;const Pointers : array of Pointer) : Integer;

{// MBCS Functions. No MBCS yet, so mostly these are calls to the regular counterparts.}
Type
  TMbcsByteType = (mbSingleByte, mbLeadByte, mbTrailByte);

Function ByteType(const S: string; Index: SizeUInt): TMbcsByteType;
Function StrByteType(Str: PChar; Index: SizeUInt): TMbcsByteType;
Function ByteToCharLen(const S: string; MaxLen: SizeInt): SizeInt;
Function CharToByteLen(const S: string; MaxLen: SizeInt): SizeInt;
Function ByteToCharIndex(const S: string; Index: SizeInt): SizeInt;
Function StrCharLength(const Str: PChar): SizeInt;
function StrNextChar(const Str: PChar): PChar;
function IsLeadChar(C: AnsiChar): Boolean; inline; overload;
function IsLeadChar(B: Byte): Boolean; inline; overload;


const
{$ifndef unix}
  SwitchChars = ['/','-'];
{$else}
  SwitchChars = ['-'];
{$endif}

Type
  TSysCharSet = Set of AnsiChar;
  PSysCharSet = ^TSysCharSet;

Function FindCmdLineSwitch(const Switch: string; const Chars: TSysCharSet;IgnoreCase: Boolean): Boolean;
Function FindCmdLineSwitch(const Switch: string; IgnoreCase: Boolean): Boolean;
Function FindCmdLineSwitch(const Switch: string): Boolean;

function WrapText(const Line, BreakStr: string; const BreakChars: TSysCharSet;  MaxCol: Integer): string;
function WrapText(const Line: string; MaxCol: Integer): string;


{==============================================================================}
{   extra functions                                                            }
{==============================================================================}

function LeftStr(const S: string; Count: integer): string;
function RightStr(const S: string; Count: integer): string;
function BCDToInt(Value: integer): integer;

Type
  {==============================================================================}
  {   TStringBuilder                                                             }
  {==============================================================================}

  // Ansi version implementation

{$MACRO ON}
{$UNDEF SBUNICODE}
{$define SBChar:=AnsiChar}
{$define SBString:=AnsiString}
{$define TSBCharArray:=Array of SBChar}
{$define PSBChar:=PAnsiChar}
{$define SBRAWString:=RawByteString}
{$define TGenericStringBuilder:=TAnsiStringBuilder}

{$i syssbh.inc}
{$undef SBChar}
{$undef SBString}
{$undef TSBCharArray}
{$undef PSBChar}
{$undef SBRAWString}
{$undef TGenericStringBuilder}

// Unicode version implementation

{$define SBUNICODE}
{$define SBChar:=WideChar}
{$define SBString:=UnicodeString}
{$define TSBCharArray:=Array of SBChar}
{$define PSBChar:=PWideChar}
{$define SBRAWString:=UnicodeString}
{$define TGenericStringBuilder:=TUnicodeStringBuilder}
{$i syssbh.inc}
{$undef SBChar}
{$undef SBString}
{$undef TSBCharArray}
{$undef PSBChar}
{$undef SBRAWString}
{$undef TGenericStringBuilder}
{$undef SBUNICODE}

Type
  TStringBuilder = TAnsiStringBuilder;

