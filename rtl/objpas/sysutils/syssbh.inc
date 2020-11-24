
  { TGenericStringBuilder }

  TGenericStringBuilder = class
  private
    const
      DefaultCapacity = 64;
  private
    Function  GetCapacity: Integer;
    Procedure SetCapacity(AValue: Integer);
    Function  GetC(Index: Integer): SBChar;
    Procedure SetC(Index: Integer; AValue: SBChar);
    Function  GetLength: Integer; inline;
    Procedure SetLength(AValue: Integer);
  protected
    FData: TSBCharArray;
    FLength: Integer;
    FMaxCapacity: Integer;
    // Raise error on range check.
    Procedure CheckRange(Idx,Count,MaxLen : Integer);inline;
    Procedure CheckNegative(Const AValue : Integer; Const AName: SBString); inline;
    // All appends/inserts pass through here.

    Procedure DoAppend(Const S : {$IFDEF SBUNICODE}SBString{$ELSE}RawByteString{$ENDIF});virtual;
    Procedure DoAppend(const AValue: TSBCharArray; Idx, aCount: Integer); virtual;
    Procedure DoInsert(Index: Integer; const AValue: SBString); virtual;
    Procedure DoInsert(Index: Integer; const AValue: TSBCharArray; StartIndex, SBCharCount: Integer); virtual;
    Procedure DoReplace(Index: Integer; const Old, New: SBString); virtual;
    Procedure Grow;
    Procedure Shrink;
  public
    Constructor Create;
    Constructor Create(aCapacity: Integer);
    Constructor Create(const AValue: SBString);
    Constructor Create(aCapacity: Integer; aMaxCapacity: Integer);
    Constructor Create(const AValue: SBString; aCapacity: Integer);
    Constructor Create(const AValue: SBString; StartIndex: Integer; aLength: Integer; aCapacity: Integer);

    Function Append(const AValue: Boolean): TGenericStringBuilder;
    Function Append(const AValue: Byte): TGenericStringBuilder;
    Function Append(const AValue: SBChar): TGenericStringBuilder;
    Function Append(const AValue: Currency): TGenericStringBuilder;
    Function Append(const AValue: Double): TGenericStringBuilder;
    Function Append(const AValue: Smallint): TGenericStringBuilder;
    Function Append(const AValue: LongInt): TGenericStringBuilder;
    Function Append(const AValue: Int64): TGenericStringBuilder;
    Function Append(const AValue: TObject): TGenericStringBuilder;
    Function Append(const AValue: Shortint): TGenericStringBuilder;
    Function Append(const AValue: Single): TGenericStringBuilder;
    Function Append(const AValue: UInt64): TGenericStringBuilder;
    Function Append(const AValue: TSBCharArray): TGenericStringBuilder;
    Function Append(const AValue: Word): TGenericStringBuilder;
    Function Append(const AValue: Cardinal): TGenericStringBuilder;
    Function Append(const AValue: PSBChar): TGenericStringBuilder;
{$IFDEF SBUNICODE}
    // Do not use SBRawstring, we need 2 versions in case of unicode
    Function Append(const AValue: SBString): TGenericStringBuilder;
{$ENDIF}
    Function Append(const AValue: RawByteString): TGenericStringBuilder;
    Function Append(const AValue: SBChar; RepeatCount: Integer): TGenericStringBuilder;
    Function Append(const AValue: TSBCharArray; StartIndex: Integer; SBCharCount: Integer): TGenericStringBuilder;
    Function Append(const AValue: SBString; StartIndex: Integer; Count: Integer): TGenericStringBuilder;

    Function Append(const Fmt: SBString; const Args: array of const): TGenericStringBuilder;
    Function AppendFormat(const Fmt: SBString; const Args: array of const): TGenericStringBuilder;
    Function AppendLine: TGenericStringBuilder;
    Function AppendLine(const AValue: RawByteString): TGenericStringBuilder;

    Procedure Clear;
    Procedure CopyTo(SourceIndex: Integer; Var Destination: TSBCharArray; DestinationIndex: Integer; Count: Integer);
    Function EnsureCapacity(aCapacity: Integer): Integer;
    Function Equals(StringBuilder: TGenericStringBuilder): Boolean; reintroduce;

    Function Insert(Index: Integer; const AValue: Boolean): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Byte): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: SBChar): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Currency): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Double): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Smallint): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: LongInt): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: TSBCharArray): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Int64): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: TObject): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Shortint): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Single): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: SBString): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Word): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: Cardinal): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: UInt64): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: SBString; const aRepeatCount: Integer): TGenericStringBuilder;
    Function Insert(Index: Integer; const AValue: TSBCharArray; startIndex: Integer; SBCharCount: Integer): TGenericStringBuilder;

    Function Remove(StartIndex: Integer; RemLength: Integer): TGenericStringBuilder;

    Function Replace(const OldChar, NewChar: SBChar): TGenericStringBuilder;
    Function Replace(const OldChar, NewChar: SBChar; StartIndex: Integer; Count: Integer): TGenericStringBuilder;
    Function Replace(const OldValue, NewValue: SBRawString): TGenericStringBuilder;
    Function Replace(const OldValue, NewValue: SBRawString; StartIndex: Integer; Count: Integer): TGenericStringBuilder;
{$IFDEF SBUNICODE}
    Function ToString: SBString;
{$ELSE}
    Function ToString: SBString; override;
{$ENDIF}
    Function ToString(aStartIndex: Integer; aLength: Integer): SBString; reintroduce;

    property Chars[index: Integer]: SBChar read GetC write SetC; default;
    property Length: Integer read GetLength write SetLength;
    property Capacity: Integer read GetCapacity write SetCapacity;
    property MaxCapacity: Integer read FMaxCapacity;
  end;


