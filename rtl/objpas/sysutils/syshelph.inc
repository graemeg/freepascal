{%mainunit syshelpers.pp}

Type
  generic TArray<T> = array of T;
  TStringArray = Array of string;
  TCharArray = Array of char;
  TEndian = ObjPas.TEndian;
  TByteBitIndex = 0..7;
  TShortIntBitIndex = 0..7;
  TWordBitIndex = 0..15;
  TSmallIntBitIndex = 0..15;
  TCardinalBitIndex = 0..31;
  TIntegerBitIndex = 0..31;
  TLongIntBitIndex = TIntegerBitIndex;
  TQwordBitIndex = 0..63;
  TInt64BitIndex = 0..63;
{$IFDEF cpu16}
  TNativeUIntBitIndex = TWordBitIndex;
  TNativeIntBitIndex = TSmallIntBitIndex;
{$ENDIF}
{$IFDEF cpu32}
  TNativeUIntBitIndex = TCardinalBitIndex;
  TNativeIntBitIndex = TIntegerBitIndex;
{$ENDIF}
{$IFDEF cpu64}
  TNativeUIntBitIndex = TQWordBitIndex;
  TNativeIntBitIndex = TInt64BitIndex;
{$ENDIF}

  
Const
  CPUEndian = {$IFDEF FPC_LITTLE_ENDIAN}TEndian.Little{$ELSE}TEndian.Big{$ENDIF};

Type
  TGuidHelper = record helper for TGUID
    Class function Create(const Data; BigEndian: Boolean): TGUID; overload; static;
    class function Create(const Data: array of Byte; AStartIndex: Cardinal; BigEndian: Boolean): TGUID; overload; static;
    Class Function Create(const Data; DataEndian: TEndian = CPUEndian): TGUID; overload; static; inline;
    Class Function Create(const B: TBytes; DataEndian: TEndian = CPUEndian): TGUID; overload; static; inline;
    Class Function Create(const B: TBytes; AStartIndex: Cardinal; DataEndian: TEndian = CPUEndian): TGUID; overload; static;
    Class Function Create(const S: string): TGUID; overload; static;
    Class Function Create(A: Integer; B: SmallInt; C: SmallInt; const D: TBytes): TGUID; overload; static;
    Class Function Create(A: Integer; B: SmallInt; C: SmallInt; D, E, F, G, H, I, J, K: Byte): TGUID; overload; static;
    Class Function Create(A: Cardinal; B: Word; C: Word; D, E, F, G, H, I, J, K: Byte): TGUID; overload; static;
    Class Function NewGuid: TGUID; static;
    Function ToByteArray(DataEndian: TEndian = CPUEndian): TBytes;
    Function ToString(SkipBrackets: Boolean = False): string;
  end;


  TCompareOption = system.TCompareOption;
  TCompareOptions = system.TCompareOptions;

{$SCOPEDENUMS ON}
  TStringSplitOptions = (None, ExcludeEmpty);
{$SCOPEDENUMS OFF}

  { TStringHelper }

  TStringHelper = Type Helper for AnsiString
  Private
    Function GetChar(AIndex : SizeInt) : Char;
    Function GetLength : SizeInt;
  public
    const Empty = '';
    // Methods
    Class Function Compare(const A: string; const B: string): Integer; overload; static; //inline;
    Class Function Compare(const A: string; const B: string; IgnoreCase: Boolean): Integer; overload; static; //inline; //deprecated 'Use same with TCompareOptions';
    Class Function Compare(const A: string; const B: string; Options: TCompareOptions): Integer; overload; static; // inline;
    Class Function Compare(const A: string; IndexA: SizeInt; const B: string; IndexB: SizeInt; ALen: SizeInt): Integer; overload; static; // inline;
    Class Function Compare(const A: string; IndexA: SizeInt; const B: string; IndexB: SizeInt; ALen: SizeInt; IgnoreCase: Boolean): Integer; overload; static; // inline; //deprecated 'Use same with TCompareOptions';
    Class Function Compare(const A: string; IndexA: SizeInt; const B: string; IndexB: SizeInt; ALen: SizeInt; Options: TCompareOptions): Integer; overload; static;//  inline;
    Class Function CompareOrdinal(const A: string; const B: string): Integer; overload; static;
    Class Function CompareOrdinal(const A: string; IndexA: SizeInt; const B: string; IndexB: SizeInt; ALen: SizeInt): Integer; overload; static;
    Class Function CompareText(const A: string; const B: string): Integer; static; inline;
    Class Function Copy(const Str: string): string; inline; static;
    Class Function Create(AChar: Char; ACount: SizeInt): string; overload; inline; static;
    Class Function Create(const AValue: array of Char): string; overload; static;
    Class Function Create(const AValue: array of Char; StartIndex: SizeInt; ALen: SizeInt): string; overload; static;
    Class Function EndsText(const ASubText, AText: string): Boolean; static;
    Class Function Equals(const a: string; const b: string): Boolean; overload; static;
    Class Function Format(const AFormat: string; const args: array of const): string; overload; static;
    Class Function IsNullOrEmpty(const AValue: string): Boolean; static;
    Class Function IsNullOrWhiteSpace(const AValue: string): Boolean; static;
    Class Function Join(const Separator: string; const Values: array of const): string; overload; static;
    Class Function Join(const Separator: string; const Values: array of string): string; overload; static;
    Class Function Join(const Separator: string; const Values: array of string; StartIndex: SizeInt; ACount: SizeInt): string; overload; static;
    Class Function LowerCase(const S: string): string; overload; static; inline;
    Class Function Parse(const AValue: Boolean): string; overload; static; inline;
    Class Function Parse(const AValue: Extended): string; overload; static;inline;
    Class Function Parse(const AValue: Int64): string; overload; static; inline;
    Class Function Parse(const AValue: Integer): string; overload; static; inline;
    Class Function ToBoolean(const S: string): Boolean; overload; static; inline;
    Class Function ToDouble(const S: string): Double; overload; static; inline;
    Class Function ToExtended(const S: string): Extended; overload; static; inline;
    Class Function ToInt64(const S: string): Int64; overload; static; inline;
    Class Function ToInteger(const S: string): Integer; overload; static; inline;
    Class Function ToSingle(const S: string): Single; overload; static; inline;
    Class Function UpperCase(const S: string): string; overload; static; inline;
    Function CompareTo(const B: string): Integer;
    Function Contains(const AValue: string): Boolean;
    procedure CopyTo(SourceIndex: SizeInt; var destination: array of Char; DestinationIndex: SizeInt; ACount: SizeInt);
    Function CountChar(const C: Char): SizeInt;
    Function DeQuotedString: string; overload;
    Function DeQuotedString(const AQuoteChar: Char): string; overload;
    Function EndsWith(const AValue: string): Boolean; overload; inline;
    Function EndsWith(const AValue: string; IgnoreCase: Boolean): Boolean; overload;
    Function Equals(const AValue: string): Boolean; overload;
    Function Format(const args: array of const): string; overload;
    Function GetHashCode: Integer;
    Function IndexOf(AValue: Char): SizeInt; overload; inline;
    Function IndexOf(const AValue: string): SizeInt; overload; inline;
    Function IndexOf(AValue: Char; StartIndex: SizeInt): SizeInt; overload;
    Function IndexOf(const AValue: string; StartIndex: SizeInt): SizeInt; overload;
    Function IndexOf(AValue: Char; StartIndex: SizeInt; ACount: SizeInt): SizeInt; overload;
    Function IndexOf(const AValue: string; StartIndex: SizeInt; ACount: SizeInt): SizeInt; overload;
    Function IndexOfUnQuoted(const AValue: string; StartQuote, EndQuote: Char; StartIndex: SizeInt = 0): SizeInt; overload;
    Function IndexOfAny(const AnyOf: array of Char): SizeInt; overload;
    Function IndexOfAny(const AnyOf: array of Char; StartIndex: SizeInt): SizeInt; overload;
    Function IndexOfAny(const AnyOf: array of Char; StartIndex: SizeInt; ACount: SizeInt): SizeInt; overload;
    Function IndexOfAny(const AnyOf: array of String): SizeInt; overload;
    Function IndexOfAny(const AnyOf: array of String; StartIndex: SizeInt): SizeInt; overload;
    Function IndexOfAny(const AnyOf: array of String; StartIndex: SizeInt; ACount: SizeInt): SizeInt; overload;
    Function IndexOfAny(const AnyOf: array of String; StartIndex: SizeInt; ACount: SizeInt; Out AMatch : SizeInt): SizeInt; overload;
    Function IndexOfAnyUnquoted(const AnyOf: array of Char; StartQuote, EndQuote: Char): SizeInt; overload;
    Function IndexOfAnyUnquoted(const AnyOf: array of Char; StartQuote, EndQuote: Char; StartIndex: SizeInt): SizeInt; overload;
    Function IndexOfAnyUnquoted(const AnyOf: array of Char; StartQuote, EndQuote: Char; StartIndex: SizeInt; ACount: SizeInt): SizeInt; overload;
    function IndexOfAnyUnquoted(const AnyOf: array of string; StartQuote, EndQuote: Char; StartIndex: SizeInt; Out Matched: SizeInt): SizeInt; overload;
    Function Insert(StartIndex: SizeInt; const AValue: string): string;
    Function IsDelimiter(const Delimiters: string; Index: SizeInt): Boolean;
    Function IsEmpty: Boolean;
    Function LastDelimiter(const Delims: string): SizeInt;
    Function LastIndexOf(AValue: Char): SizeInt; overload;
    Function LastIndexOf(const AValue: string): SizeInt; overload;
    Function LastIndexOf(AValue: Char; AStartIndex: SizeInt): SizeInt; overload;
    Function LastIndexOf(const AValue: string; AStartIndex: SizeInt): SizeInt; overload;
    Function LastIndexOf(AValue: Char; AStartIndex: SizeInt; ACount: SizeInt): SizeInt; overload;
    Function LastIndexOf(const AValue: string; AStartIndex: SizeInt; ACount: SizeInt): SizeInt; overload;
    Function LastIndexOfAny(const AnyOf: array of Char): SizeInt; overload;
    Function LastIndexOfAny(const AnyOf: array of Char; AStartIndex: SizeInt): SizeInt; overload;
    Function LastIndexOfAny(const AnyOf: array of Char; AStartIndex: SizeInt; ACount: SizeInt): SizeInt; overload;
    Function PadLeft(ATotalWidth: SizeInt): string; overload; inline;
    Function PadLeft(ATotalWidth: SizeInt; PaddingChar: Char): string; overload; inline;
    Function PadRight(ATotalWidth: SizeInt): string; overload; inline;
    Function PadRight(ATotalWidth: SizeInt; PaddingChar: Char): string; overload; inline;
    Function QuotedString: string; overload;
    Function QuotedString(const AQuoteChar: Char): string; overload;
    Function Remove(StartIndex: SizeInt): string; overload; inline;
    Function Remove(StartIndex: SizeInt; ACount: SizeInt): string; overload; inline;
    Function Replace(OldChar: Char; NewChar: Char): string; overload;
    Function Replace(OldChar: Char; NewChar: Char; ReplaceFlags: TReplaceFlags): string; overload;
    Function Replace(const OldValue: string; const NewValue: string): string; overload;
    Function Replace(const OldValue: string; const NewValue: string; ReplaceFlags: TReplaceFlags): string; overload;
    Function Split(const Separators: array of Char): TStringArray; overload;
    Function Split(const Separators: array of Char; ACount: SizeInt): TStringArray; overload;
    Function Split(const Separators: array of Char; Options: TStringSplitOptions): TStringArray; overload;
    Function Split(const Separators: array of Char; ACount: SizeInt; Options: TStringSplitOptions): TStringArray; overload;
    Function Split(const Separators: array of string): TStringArray; overload;
    Function Split(const Separators: array of string; ACount: SizeInt): TStringArray; overload;
    Function Split(const Separators: array of string; Options: TStringSplitOptions): TStringArray; overload;
    Function Split(const Separators: array of string; ACount: SizeInt; Options: TStringSplitOptions): TStringArray; overload;
    Function Split(const Separators: array of Char; AQuote: Char): TStringArray; overload;
    Function Split(const Separators: array of Char; AQuoteStart, AQuoteEnd: Char): TStringArray; overload;
    Function Split(const Separators: array of Char; AQuoteStart, AQuoteEnd: Char; Options: TStringSplitOptions): TStringArray; overload;
    Function Split(const Separators: array of Char; AQuoteStart, AQuoteEnd: Char; ACount: SizeInt): TStringArray; overload;
    Function Split(const Separators: array of Char; AQuoteStart, AQuoteEnd: Char; ACount: SizeInt; Options: TStringSplitOptions): TStringArray; overload;
    Function Split(const Separators: array of string; AQuote: Char): TStringArray; overload;
    Function Split(const Separators: array of string; AQuoteStart, AQuoteEnd: Char): TStringArray; overload;
    Function Split(const Separators: array of string; AQuoteStart, AQuoteEnd: Char; Options: TStringSplitOptions): TStringArray; overload;
    Function Split(const Separators: array of string; AQuoteStart, AQuoteEnd: Char; ACount: SizeInt): TStringArray; overload;
    Function Split(const Separators: array of string; AQuoteStart, AQuoteEnd: Char; ACount: SizeInt; Options: TStringSplitOptions): TStringArray; overload;
    Function StartsWith(const AValue: string): Boolean; overload; inline;
    Function StartsWith(const AValue: string; IgnoreCase: Boolean): Boolean; overload;
    Function Substring(AStartIndex: SizeInt): string; overload;
    Function Substring(AStartIndex: SizeInt; ALen: SizeInt): string; overload;
    Function ToBoolean: Boolean; overload; inline;
    Function ToInteger: Integer; overload; inline;
    Function ToInt64: Int64; overload; inline;
    Function ToSingle: Single; overload; inline;
    Function ToDouble: Double; overload; inline;
    Function ToExtended: Extended; overload; inline;
    Function ToCharArray: TCharArray; overload;
    Function ToCharArray(AStartIndex: SizeInt; ALen: SizeInt): TCharArray; overload;
    Function ToLower: string; overload; inline;
    Function ToLowerInvariant: string;
    Function ToUpper: string; overload; inline;
    Function ToUpperInvariant: string; inline;
    Function Trim: string; overload;
    Function TrimLeft: string; overload;
    Function TrimRight: string; overload;
    Function Trim(const ATrimChars: array of Char): string; overload;
    Function TrimLeft(const ATrimChars: array of Char): string; overload;
    Function TrimRight(const ATrimChars: array of Char): string; overload;
    Function TrimEnd(const ATrimChars: array of Char): string; deprecated 'Use TrimRight';
    Function TrimStart(const ATrimChars: array of Char): string; deprecated 'Use TrimLeft';
    property Chars[AIndex: SizeInt]: Char read GetChar;
    property Length: SizeInt read GetLength;
  end;

{$IFDEF FPC_HAS_TYPE_SINGLE}
  TSingleHelper = Type Helper for Single
  Private
    Function GetB(AIndex: Cardinal): Byte;
    Function GetW(AIndex: Cardinal): Word;
    Function GetE: QWord; inline;
    Function GetF: QWord; inline;
    Function GetS: Boolean; inline;
    procedure SetB(AIndex: Cardinal; const AValue: Byte);
    procedure SetW(AIndex: Cardinal; const AValue: Word);
    procedure SetE(AValue: QWord);
    procedure SetF(AValue: QWord);
    procedure SetS(AValue: Boolean);
  public
{$push}
{$R-}
{$Q-}
    const
      Epsilon          : Single = 1.4012984643248170709e-45;
      MaxValue         : Single = 340282346638528859811704183484516925440.0;
      MinValue         : Single = -340282346638528859811704183484516925440.0;
      PositiveInfinity : Single = 1.0/0.0;
      NegativeInfinity : Single = -1.0/0.0;
      NaN              : Single = 0.0/0.0;
{$POP}
    Class Function IsNan(const AValue: Single): Boolean; overload; inline; static;
    Class Function IsInfinity(const AValue: Single): Boolean; overload; inline; static;
    Class Function IsNegativeInfinity(const AValue: Single): Boolean; overload; inline; static;
    Class Function IsPositiveInfinity(const AValue: Single): Boolean; overload; inline; static;
    Class Function Parse(const AString: string): Single; overload; inline; static;
    Class Function Parse(const AString: string; const AFormatSettings: TFormatSettings): Single; overload; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Single): string; overload; inline; static;
    Class Function ToString(const AValue: Single; const AFormatSettings: TFormatSettings): string; overload; inline; static;
    Class Function ToString(const AValue: Single; const AFormat: TFloatFormat; const APrecision, ADigits: Integer): string; overload; inline; static;
    Class Function ToString(const AValue: Single; const AFormat: TFloatFormat; const APrecision, ADigits: Integer;
                               const AFormatSettings: TFormatSettings): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Single): Boolean; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Single; const AFormatSettings: TFormatSettings): Boolean; overload; inline; static;

    Procedure BuildUp(const ASignFlag: Boolean; const AMantissa: QWord; const AExponent: Integer);
    Function Exponent: Integer;
    Function Fraction: Extended;
    Function IsInfinity: Boolean; overload; inline;
    Function IsNan: Boolean; overload; inline;
    Function IsNegativeInfinity: Boolean; overload; inline;
    Function IsPositiveInfinity: Boolean; overload; inline;
    Function Mantissa: QWord;
    Function SpecialType: TFloatSpecial;
    Function ToString(const AFormat: TFloatFormat; const APrecision, ADigits: Integer): string; overload; inline;
    Function ToString(const AFormat: TFloatFormat; const APrecision, ADigits: Integer; const AFormatSettings: TFormatSettings): string; overload; inline;
    Function ToString(const AFormatSettings: TFormatSettings): string; overload; inline;
    Function ToString: string; overload; inline;

    property Bytes[AIndex: Cardinal]: Byte read GetB write SetB;
    property Words[AIndex: Cardinal]: Word read GetW write SetW;
    property Sign: Boolean read GetS write SetS;
    property Exp: QWord read GetE write SetE;
    property Frac: QWord read GetF write SetF;
  end;
{$ENDIF FPC_HAS_TYPE_SINGLE}

{$IFDEF FPC_HAS_TYPE_DOUBLE}
  TDoubleHelper = Type Helper for Double
  private
    Function GetB(AIndex: Cardinal): Byte;
    Function GetW(AIndex: Cardinal): Word;
    Function GetE: QWord; inline;
    Function GetF: QWord; inline;
    Function GetS: Boolean; inline;
    procedure SetB(AIndex: Cardinal; const AValue: Byte);
    procedure SetW(AIndex: Cardinal; const AValue: Word);
    procedure SetE(AValue: QWord);
    procedure SetF(AValue: QWord);
    procedure SetS(AValue: Boolean);
  public
    const
    {$push}
    {$R-}
    {$Q-}
      Epsilon          : Double = 4.9406564584124654418e-324;
      MaxValue         : Double = 1.7976931348623157081e+308;
      MinValue         : Double = -1.7976931348623157081e+308;
      PositiveInfinity : Double = 1.0/0.0;
      NegativeInfinity : Double = -1.0/0.0;
      NaN              : Double = 0.0/0.0;
    {$POP}
    Class Function IsInfinity(const AValue: Double): Boolean; overload; inline; static;
    Class Function IsNan(const AValue: Double): Boolean; overload; inline; static;
    Class Function IsNegativeInfinity(const AValue: Double): Boolean; overload; inline; static;
    Class Function IsPositiveInfinity(const AValue: Double): Boolean; overload; inline; static;
    Class Function Parse(const AString: string): Double; overload; inline; static;
    Class Function Parse(const AString: string; const AFormatSettings: TFormatSettings): Double; overload; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Double): string; overload; inline; static;
    Class Function ToString(const AValue: Double; const AFormat: TFloatFormat; const APrecision, ADigits: Integer): string; overload; inline; static;
    Class Function ToString(const AValue: Double; const AFormat: TFloatFormat; const APrecision, ADigits: Integer;  const AFormatSettings: TFormatSettings): string; overload; inline; static;
    Class Function ToString(const AValue: Double; const AFormatSettings: TFormatSettings): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Double): Boolean; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Double; const AFormatSettings: TFormatSettings): Boolean; overload; inline; static;

    Procedure BuildUp(const ASignFlag: Boolean; const AMantissa: QWord; const AExponent: Integer);
    Function Exponent: Integer;
    Function Fraction: Extended;
    Function IsInfinity: Boolean; overload; inline;
    Function IsNan: Boolean; overload; inline;
    Function IsNegativeInfinity: Boolean; overload; inline;
    Function IsPositiveInfinity: Boolean; overload; inline;
    Function Mantissa: QWord;
    Function SpecialType: TFloatSpecial;
    Function ToString(const AFormat: TFloatFormat; const APrecision, ADigits: Integer): string; overload; inline;
    Function ToString(const AFormat: TFloatFormat; const APrecision, ADigits: Integer; const AFormatSettings: TFormatSettings): string; overload; inline;
    Function ToString(const AFormatSettings: TFormatSettings): string; overload; inline;
    Function ToString: string; overload; inline;

    property Bytes[AIndex: Cardinal]: Byte read GetB write SetB;  // 0..7
    property Words[AIndex: Cardinal]: Word read GetW write SetW; // 0..3
    property Sign: Boolean read GetS write SetS;
    property Exp: QWord read GetE write SetE;
    property Frac: QWord read GetF write SetF;
  end;
{$ENDIF FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
  TExtendedHelper = Type Helper for Extended
  private
    Function GetB(AIndex: Cardinal): Byte;
    Function GetW(AIndex: Cardinal): Word;
    Function GetE: QWord; inline;
    Function GetF: QWord; inline;
    Function GetS: Boolean; inline;
    procedure SetB(AIndex: Cardinal; const AValue: Byte);
    procedure SetW(AIndex: Cardinal; const AValue: Word);
    procedure SetE(AValue: QWord);
    procedure SetF(AValue: QWord);
    procedure SetS(AValue: Boolean);
  public
    {$push}
    {$R-}
    {$Q-}
    const
      Epsilon           : Extended = 3.64519953188247460253e-4951;
      MaxValue          : Extended = 1.18973149535723176505e+4932;
      MinValue          : Extended = -1.18973149535723176505e+4932;
      PositiveInfinity  : Extended = 1.0/0.0;
      NegativeInfinity  : Extended = -1.0/0.0;
      NaN               : Extended = 0.0/0.0;
    {$POP}
    Class Function ToString(const AValue: Extended): string; overload; inline; static;
    Class Function ToString(const AValue: Extended; const AFormatSettings: TFormatSettings): string; overload; inline; static;
    Class Function ToString(const AValue: Extended; const AFormat: TFloatFormat; const APrecision, ADigits: Integer): string; overload; inline; static;
    Class Function ToString(const AValue: Extended; const AFormat: TFloatFormat; const APrecision, ADigits: Integer; const AFormatSettings: TFormatSettings): string; overload; inline; static;
    Class Function Parse(const AString: string): Extended; overload; inline; static;
    Class Function Parse(const AString: string; const AFormatSettings: TFormatSettings): Extended; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Extended): Boolean; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Extended; const AFormatSettings: TFormatSettings): Boolean; overload; inline; static;
    Class Function IsNan(const AValue: Extended): Boolean; overload; inline; static;
    Class Function IsInfinity(const AValue: Extended): Boolean; overload; inline; static;
    Class Function IsNegativeInfinity(const AValue: Extended): Boolean; overload; inline; static;
    Class Function IsPositiveInfinity(const AValue: Extended): Boolean; overload; inline; static;
    Class Function Size: Integer; inline; static;

    procedure BuildUp(const ASignFlag: Boolean; const AMantissa: QWord; Const AExponent: Integer);
    Function Exponent: Integer;
    Function Fraction: Extended;
    Function IsInfinity: Boolean; overload; inline;
    Function IsNan: Boolean; overload; inline;
    Function IsNegativeInfinity: Boolean; overload; inline;
    Function IsPositiveInfinity: Boolean; overload; inline;
    Function Mantissa: QWord;
    Function SpecialType: TFloatSpecial;
    Function ToString(const AFormat: TFloatFormat; const APrecision, ADigits: Integer): string; overload; inline;
    Function ToString(const AFormat: TFloatFormat; const APrecision, ADigits: Integer;  const AFormatSettings: TFormatSettings): string; overload; inline;
    Function ToString(const AFormatSettings: TFormatSettings): string; overload; inline;
    Function ToString: string; overload; inline;

    property Bytes[AIndex: Cardinal]: Byte read GetB write SetB;
    property Words[AIndex: Cardinal]: Word read GetW write SetW;
    property Sign: Boolean read GetS write SetS;
    property Exp: QWord read GetE write SetE;
    property Frac: QWord read GetF write SetF;
  end;
{$ENDIF FPC_HAS_TYPE_EXTENDED}

  TByteHelper = Type Helper for Byte
  public
    const
      MaxValue = 255;
      MinValue = 0;
  public
    Class Function Parse(const AString: string): Byte; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Byte): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Byte): Boolean; inline; static;
  Public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TByteBitIndex) : Byte; inline;
    Function ClearBit(const Index: TByteBitIndex) : Byte; inline;
    Function ToggleBit(const Index: TByteBitIndex) : Byte; inline;
    Function TestBit(const Index:TByteBitIndex):Boolean; inline;
  end;

  TShortIntHelper = Type Helper for ShortInt
  public
    const
      MaxValue = 127;
      MinValue = -128;
  public
    Class Function Parse(const AString: string): ShortInt; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: ShortInt): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: ShortInt): Boolean; inline; static;
  public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TShortIntBitIndex): Shortint; inline;
    Function ClearBit(const Index: TShortIntBitIndex): Shortint; inline;
    Function ToggleBit(const Index: TShortIntBitIndex): Shortint; inline;
    Function TestBit(const Index:TShortIntBitIndex):Boolean;
  end;

  TSmallIntHelper = Type Helper for SmallInt
  public
    const
      MaxValue = 32767;
      MinValue = -32768;
  public
    Class Function Parse(const AString: string): SmallInt; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: SmallInt): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: SmallInt): Boolean; inline; static;
  public
    Function ToString: string; overload; inline;
    Function ToBoolean: Boolean; inline;
    Function ToBinString:string; inline;
    Function ToHexString: string; overload; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function SetBit(const Index: TSmallIntBitIndex) : Smallint; inline;
    Function ClearBit(const Index: TSmallIntBitIndex) : Smallint; inline;
    Function ToggleBit(const Index: TSmallIntBitIndex) : Smallint; inline;
    Function TestBit(const Index:TSmallIntBitIndex):Boolean;    
  end;

  TWordHelper = Type Helper for Word
  public
    const
      MaxValue = 65535;
      MinValue = 0;
  Public
    Class Function Parse(const AString: string): Word; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Word): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Word): Boolean; inline; static;
  Public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TWordBitIndex) : Word; inline;
    Function ClearBit(const Index: TWordBitIndex) : Word; inline;
    Function ToggleBit(const Index: TWordBitIndex) : Word; inline;
    Function TestBit(const Index:TWordBitIndex):Boolean; inline;    
  end;

  TCardinalHelper = Type Helper for Cardinal { for LongWord Type too }
  public
    const
      MaxValue = 4294967295;
      MinValue = 0;
  Public
    Class Function Parse(const AString: string): Cardinal; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Cardinal): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Cardinal): Boolean; inline; static;
  Public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TCardinalBitIndex) : Cardinal; inline;
    Function ClearBit(const Index: TCardinalBitIndex) : Cardinal; inline;
    Function ToggleBit(const Index: TCardinalBitIndex) : Cardinal; inline;
    Function TestBit(const Index:TCardinalBitIndex):Boolean; inline;   
  end;

  TIntegerHelper = Type Helper for Integer { for LongInt Type too }
  public
    const
      MaxValue = 2147483647;
      MinValue = -2147483648;
  Public
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Integer): string; overload; inline; static;
    Class Function Parse(const AString: string): Integer; inline; static;
    Class Function TryParse(const AString: string; out AValue: Integer): Boolean; inline; static;
  Public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TIntegerBitIndex) : Integer; inline;
    Function ClearBit(const Index: TIntegerBitIndex) : Integer; inline;
    Function ToggleBit(const Index: TIntegerBitIndex) : Integer; inline;
    Function TestBit(const Index:TIntegerBitIndex):Boolean; inline;   
  end;

  TInt64Helper = Type Helper for Int64
  public
    const
      MaxValue = 9223372036854775807;
      MinValue = -9223372036854775808;
  Public
    Class Function Parse(const AString: string): Int64; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Int64): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: Int64): Boolean; inline; static;
  Public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TInt64BitIndex) : Int64; inline;
    Function ClearBit(const Index: TInt64BitIndex) : Int64; inline;
    Function ToggleBit(const Index: TInt64BitIndex) : Int64; inline;
    Function TestBit(const Index:TInt64BitIndex):Boolean; inline; 
  end;

  TQWordHelper = Type Helper for QWord
  public
    const
      MaxValue = 18446744073709551615;
      MinValue = 0;
  Public
    Class Function Parse(const AString: string): QWord; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: QWord): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: QWord): Boolean; inline; static;
  Public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TQwordBitIndex) : QWord; inline;
    Function ClearBit(const Index: TQwordBitIndex) : QWord; inline;
    Function ToggleBit(const Index: TQwordBitIndex) : QWord; inline;
    Function TestBit(const Index:TQwordBitIndex):Boolean; inline; 
  end;

  TNativeIntHelper = Type Helper for NativeInt
  public
    const
      MaxValue = High(NativeInt);
      MinValue = Low(NativeInt);
  Public
    Class Function Parse(const AString: string): NativeInt; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: NativeInt): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: NativeInt): Boolean; inline; static;
  Public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TNativeIntBitIndex) : NativeInt; inline;
    Function ClearBit(const Index: TNativeIntBitIndex) : NativeInt; inline;
    Function ToggleBit(const Index: TNativeIntBitIndex) : NativeInt; inline;
    Function TestBit(const Index:TNativeIntBitIndex):Boolean; inline; 
  end;

  TNativeUIntHelper = Type Helper for NativeUInt
  public
    const
      MaxValue = High(NativeUInt);
      MinValue = 0;
  Public
    Class Function Parse(const AString: string): NativeUInt; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: NativeUInt): string; overload; inline; static;
    Class Function TryParse(const AString: string; out AValue: NativeUInt): Boolean; inline; static;
  Public
    Function ToBoolean: Boolean; inline;
    Function ToDouble: Double; inline;
    Function ToExtended: Extended; inline;
    Function ToBinString:string; inline;
    Function ToHexString(const AMinDigits: Integer): string; overload; inline;
    Function ToHexString: string; overload; inline;
    Function ToSingle: Single; inline;
    Function ToString: string; overload; inline;
    Function SetBit(const Index: TNativeUIntBitIndex) : NativeUint; inline;
    Function ClearBit(const Index: TNativeUIntBitIndex): NativeUint; inline;
    Function ToggleBit(const Index: TNativeUIntBitIndex) : NativeUint; inline;
    Function TestBit(const Index:TNativeUIntBitIndex) :Boolean; inline;
  end;

  {$SCOPEDENUMS ON}
    TUseBoolStrs = (False, True);
  {$SCOPEDENUMS OFF}

  TBooleanHelper = Type Helper for Boolean
  public
    Class Function Parse(const S: string): Boolean; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Boolean; UseBoolStrs: TUseBoolStrs = TUseBoolStrs.False): string; overload; inline; static;
    Class Function TryToParse(const S: string; out AValue: Boolean): Boolean; inline; static;
  Public
    Function ToInteger: Integer; inline;
    Function ToString(UseBoolStrs: TUseBoolStrs = TUseBoolStrs.False): string; overload; inline;
  end;

  TByteBoolHelper = Type Helper for ByteBool
  public
    Class Function Parse(const S: string): Boolean; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Boolean; UseBoolStrs: TUseBoolStrs = TUseBoolStrs.False): string; overload; inline; static;
    Class Function TryToParse(const S: string; out AValue: Boolean): Boolean; inline; static;
  Public
    Function ToInteger: Integer; inline;
    Function ToString(UseBoolStrs: TUseBoolStrs = TUseBoolStrs.False): string; overload; inline;
  end;

  TWordBoolHelper = Type Helper for WordBool
  public
    Class Function Parse(const S: string): Boolean; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Boolean; UseBoolStrs: TUseBoolStrs = TUseBoolStrs.False): string; overload; inline; static;
    Class Function TryToParse(const S: string; out AValue: Boolean): Boolean; inline; static;
  Public
    Function ToInteger: Integer; inline;
    Function ToString(UseBoolStrs: TUseBoolStrs = TUseBoolStrs.False): string; overload; inline;
  end;

  TLongBoolHelper = Type Helper for LongBool
  public
    Class Function Parse(const S: string): Boolean; inline; static;
    Class Function Size: Integer; inline; static;
    Class Function ToString(const AValue: Boolean; UseBoolStrs: TUseBoolStrs = TUseBoolStrs.False): string; overload; inline; static;
    Class Function TryToParse(const S: string; out AValue: Boolean): Boolean; inline; static;
  public
    Function ToInteger: Integer; inline;
    Function ToString(UseBoolStrs: TUseBoolStrs = TUseBoolStrs.False): string; overload; inline;
  end;
