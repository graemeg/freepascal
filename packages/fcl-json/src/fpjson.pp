{
    This file is part of the Free Component Library

    JSON Data structures
    Copyright (c) 2007 by Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit fpjson;

interface

uses
  variants,
  SysUtils,
  classes,
  contnrs;

type

  TJSONtype = (jtUnknown, jtNumber, jtString, jtBoolean, jtNull, jtArray, jtObject);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TJSONInstanceType = (jitUnknown, jitNumberInteger,jitNumberInt64,jitNumberQWord,jitNumberFloat,
                       jitString, jitBoolean, jitNull, jitArray, jitObject);
  TJSONFloat = Double;
  TJSONStringType = UTF8String;
  TJSONUnicodeStringType = Unicodestring;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TJSONFloat = Double;
  TJSONStringType = AnsiString;
>>>>>>> graemeg/cpstrnew
  TJSONCharType = AnsiChar;
  PJSONCharType = ^TJSONCharType;
  TFormatOption = (foSingleLineArray,   // Array without CR/LF : all on one line
                   foSingleLineObject,  // Object without CR/LF : all on one line
                   foDoNotQuoteMembers, // Do not quote object member names.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   foUseTabchar,        // Use tab characters instead of spaces.
                   foSkipWhiteSpace);   // Do not use whitespace at all
=======
                   foUseTabchar);       // Use tab characters instead of spaces.
>>>>>>> graemeg/cpstrnew
=======
                   foUseTabchar);       // Use tab characters instead of spaces.
>>>>>>> graemeg/cpstrnew
=======
                   foUseTabchar);       // Use tab characters instead of spaces.
>>>>>>> graemeg/cpstrnew
=======
                   foUseTabchar);       // Use tab characters instead of spaces.
>>>>>>> origin/cpstrnew
  TFormatOptions = set of TFormatOption;

Const
  DefaultIndentSize = 2;
  DefaultFormat     = [];
  AsJSONFormat      = [foSingleLineArray,foSingleLineObject]; // These options make FormatJSON behave as AsJSON
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AsCompressedJSON  = [foSingleLineArray,foSingleLineObject,foskipWhiteSpace]; // These options make FormatJSON behave as AsJSON with TJSONData.CompressedJSON=True
  AsCompactJSON     = [foSingleLineArray,foSingleLineObject,foskipWhiteSpace,foDoNotQuoteMembers]; // These options make FormatJSON behave as AsJSON with TJSONData.CompressedJSON=True and TJSONObject.UnquotedMemberNames=True
  ValueJSONTypes    = [jtNumber, jtString, jtBoolean, jtNull];
  ActualValueJSONTypes = ValueJSONTypes - [jtNull];
  StructuredJSONTypes  = [jtArray,jtObject];

Type
  TJSONData = Class;

  { TMJBaseObjectEnumerator }
  TJSONEnum = Record
    Key : TJSONStringType;
    KeyNum : Integer;
    Value : TJSONData;
  end;

  TBaseJSONEnumerator = class
  public
    function GetCurrent: TJSONEnum; virtual; abstract;
    function MoveNext : Boolean; virtual; abstract;
    property Current: TJSONEnum read GetCurrent;
  end;

  { TMJObjectEnumerator }


=======
  
Type

>>>>>>> graemeg/cpstrnew
=======
  
Type

>>>>>>> graemeg/cpstrnew
=======
  
Type

>>>>>>> graemeg/cpstrnew
=======
  
Type

>>>>>>> origin/cpstrnew
  { TJSONData }
  
  TJSONData = class(TObject)
  private
    Const
      ElementSeps  : Array[Boolean] of TJSONStringType = (', ',',');
    Class Var FCompressedJSON : Boolean;
    Class Var FElementSep : TJSONStringType;
    class procedure DetermineElementSeparators;
    class function GetCompressedJSON: Boolean; static;
    class procedure SetCompressedJSON(AValue: Boolean); static;
  protected
    Class Procedure DoError(Const Msg : String);
    Class Procedure DoError(Const Fmt : String; const Args : Array of const);
    Function DoFindPath(Const APath : TJSONStringType; Out NotFound : TJSONStringType) : TJSONdata; virtual;
    function GetAsBoolean: Boolean; virtual; abstract;
    function GetAsFloat: TJSONFloat; virtual; abstract;
    function GetAsInteger: Integer; virtual; abstract;
    function GetAsInt64: Int64; virtual; abstract;
    function GetAsQWord: QWord; virtual; abstract;
=======
=======
>>>>>>> origin/fixes_2_2
  TJSONFloat = Extended;
  TJSONStringType = AnsiString;

  { TJSONData }

  TJSONData = class(TObject)
  protected
    function GetAsBoolean: Boolean; virtual; abstract;
    function GetAsFloat: TJSONFloat; virtual; abstract;
    function GetAsInteger: Integer; virtual; abstract;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function GetIsNull: Boolean; virtual;
    procedure SetAsBoolean(const AValue: Boolean); virtual; abstract;
    procedure SetAsFloat(const AValue: TJSONFloat); virtual; abstract;
    procedure SetAsInteger(const AValue: Integer); virtual; abstract;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SetAsInt64(const AValue: Int64); virtual; abstract;
    procedure SetAsQword(const AValue: QWord); virtual; abstract;
    function GetAsJSON: TJSONStringType; virtual; abstract;
    function GetAsString: TJSONStringType; virtual; abstract;
    procedure SetAsString(const AValue: TJSONStringType); virtual; abstract;
    function GetAsUnicodeString: TJSONUnicodeStringType; virtual; 
    procedure SetAsUnicodeString(const AValue: TJSONUnicodeStringType); virtual;
=======
    function GetAsJSON: TJSONStringType; virtual; abstract;
    function GetAsString: TJSONStringType; virtual; abstract;
    procedure SetAsString(const AValue: TJSONStringType); virtual; abstract;
>>>>>>> graemeg/fixes_2_2
=======
    function GetAsJSON: TJSONStringType; virtual; abstract;
    function GetAsString: TJSONStringType; virtual; abstract;
    procedure SetAsString(const AValue: TJSONStringType); virtual; abstract;
>>>>>>> origin/fixes_2_2
    function GetValue: variant; virtual; abstract;
    procedure SetValue(const AValue: variant); virtual; abstract;
    function GetItem(Index : Integer): TJSONData; virtual;
    procedure SetItem(Index : Integer; const AValue: TJSONData); virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    Function DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; virtual;
    function GetCount: Integer; virtual;
  Public
    Class function JSONType: TJSONType; virtual;
    Class Property CompressedJSON : Boolean Read GetCompressedJSON Write SetCompressedJSON;
  public
    Constructor Create; virtual;
    Procedure Clear;  virtual; Abstract;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure DumpJSON(S : TStream);
    // Get enumerator
    function GetEnumerator: TBaseJSONEnumerator; virtual;
    Function FindPath(Const APath : TJSONStringType) : TJSONdata;
    Function GetPath(Const APath : TJSONStringType) : TJSONdata;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Function Clone : TJSONData; virtual; abstract;
    Function FormatJSON(Options : TFormatOptions = DefaultFormat; Indentsize : Integer = DefaultIndentSize) : TJSONStringType; 
=======
=======
>>>>>>> origin/fixes_2_2
    function GetCount: Integer; virtual;
  public
    Constructor Create; virtual;
    Class function JSONType: TJSONType; virtual;
    Procedure Clear;  virtual; Abstract;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property Count: Integer read GetCount;
    property Items[Index: Integer]: TJSONData read GetItem write SetItem;
    property Value: variant read GetValue write SetValue;
    Property AsString : TJSONStringType Read GetAsString Write SetAsString;
<<<<<<< HEAD
<<<<<<< HEAD
    Property AsUnicodeString : TJSONUnicodeStringType Read GetAsUnicodeString Write SetAsUnicodeString;
    Property AsFloat : TJSONFloat Read GetAsFloat Write SetAsFloat;
    Property AsInteger : Integer Read GetAsInteger Write SetAsInteger;
    Property AsInt64 : Int64 Read GetAsInt64 Write SetAsInt64;
    Property AsQWord : QWord Read GetAsQWord Write SetAsQword;
    Property AsBoolean : Boolean Read GetAsBoolean Write SetAsBoolean;
    Property IsNull : Boolean Read GetIsNull;
    Property AsJSON : TJSONStringType Read GetAsJSON;
  end;

  TJSONDataClass = Class of TJSONData;
  TJSONNumberType = (ntFloat,ntInteger,ntInt64,ntQWord);
=======
=======
>>>>>>> origin/fixes_2_2
    Property AsFloat : TJSONFloat Read GetAsFloat Write SetAsFloat;
    Property AsInteger : Integer Read GetAsInteger Write SetAsInteger;
    Property AsBoolean : Boolean Read GetAsBoolean Write SetAsBoolean;
    Property IsNull : Boolean Read GetIsNull;
    Property AsJSON : String Read GetAsJSON;
  end;

  TJSONDataClass = Class of TJSONData;
  TJSONNumberType = (ntFloat,ntInteger);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  TJSONNumber = class(TJSONData)
  protected
  public
    class function JSONType: TJSONType; override;
    class function NumberType : TJSONNumberType; virtual; abstract;
  end;

  { TJSONFloatNumber }

  TJSONFloatNumber = class(TJSONNumber)
  Private
    FValue : TJSONFloat;
  protected
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> origin/fixes_2_2
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
    function GetValue: variant; override;
    procedure SetValue(const AValue: variant); override;
  public
<<<<<<< HEAD
<<<<<<< HEAD
    Constructor Create(AValue : TJSONFloat); reintroduce;
    class function NumberType : TJSONNumberType; override;
    Procedure Clear;  override;
    Function Clone : TJSONData; override;
  end;
  TJSONFloatNumberClass = Class of TJSONFloatNumber;

=======
=======
>>>>>>> origin/fixes_2_2
    Constructor Create(AValue : TJSONFloat);
    class function NumberType : TJSONNumberType; override;
    Procedure Clear;  override;
  end;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { TJSONIntegerNumber }

  TJSONIntegerNumber = class(TJSONNumber)
  Private
    FValue : Integer;
  protected
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
    function GetValue: variant; override;
    procedure SetValue(const AValue: variant); override;
  public
    Constructor Create(AValue : Integer); reintroduce;
    class function NumberType : TJSONNumberType; override;
    Procedure Clear;  override;
    Function Clone : TJSONData; override;
  end;
  TJSONIntegerNumberClass = Class of TJSONIntegerNumber;

  { TJSONInt64Number }

  TJSONInt64Number = class(TJSONNumber)
  Private
    FValue : Int64;
  protected
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
    function GetValue: variant; override;
    procedure SetValue(const AValue: variant); override;
  public
    Constructor Create(AValue : Int64); reintroduce;
    class function NumberType : TJSONNumberType; override;
    Procedure Clear;  override;
    Function Clone : TJSONData; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  end;
  TJSONInt64NumberClass = Class of TJSONInt64Number;

  { TJSONQWordNumber }

  TJSONQWordNumber = class(TJSONNumber)
  Private
    FValue : Qword;
  protected
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> origin/fixes_2_2
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
    function GetValue: variant; override;
    procedure SetValue(const AValue: variant); override;
  public
<<<<<<< HEAD
<<<<<<< HEAD
    Constructor Create(AValue : QWord); reintroduce;
    class function NumberType : TJSONNumberType; override;
    Procedure Clear;  override;
    Function Clone : TJSONData; override;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;
  TJSONQWordNumberClass = Class of TJSONQWordNumber;

=======
=======
>>>>>>> origin/fixes_2_2
    Constructor Create(AValue : Integer);
    class function NumberType : TJSONNumberType; override;
    Procedure Clear;  override;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { TJSONString }

  TJSONString = class(TJSONData)
  Private
    FValue: TJSONStringType;
  protected
    function GetValue: Variant; override;
    procedure SetValue(const AValue: Variant); override;
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> origin/fixes_2_2
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
  public
<<<<<<< HEAD
<<<<<<< HEAD
    Constructor Create(const AValue : TJSONStringType); reintroduce;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Constructor Create(const AValue : TJSONUnicodeStringType); reintroduce;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    class function JSONType: TJSONType; override;
    Procedure Clear;  override;
    Function Clone : TJSONData; override;
  end;
  TJSONStringClass = Class of TJSONString;
=======
=======
>>>>>>> origin/fixes_2_2
    Constructor Create(AValue : TJSONStringType);
    class function JSONType: TJSONType; override;
    Procedure Clear;  override;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { TJSONboolean }

  TJSONBoolean = class(TJSONData)
  Private
    FValue: Boolean;
  protected
    function GetValue: Variant; override;
    procedure SetValue(const AValue: Variant); override;
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> origin/fixes_2_2
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
  public
<<<<<<< HEAD
<<<<<<< HEAD
    Constructor Create(AValue : Boolean); reintroduce;
    class function JSONType: TJSONType; override;
    Procedure Clear;  override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Function  Clone : TJSONData; override;
=======
    Function Clone : TJSONData; override;
>>>>>>> graemeg/cpstrnew
=======
    Function Clone : TJSONData; override;
>>>>>>> graemeg/cpstrnew
=======
    Function Clone : TJSONData; override;
>>>>>>> graemeg/cpstrnew
=======
    Function Clone : TJSONData; override;
>>>>>>> origin/cpstrnew
  end;
  TJSONBooleanClass = Class of TJSONBoolean;
=======
=======
>>>>>>> origin/fixes_2_2
    Constructor Create(AValue : Boolean);
    class function JSONType: TJSONType; override;
    Procedure Clear;  override;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { TJSONnull }

  TJSONNull = class(TJSONData)
  protected
    Procedure Converterror(From : Boolean);
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function GetIsNull: Boolean; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
    function GetValue: variant; override;
    procedure SetValue(const AValue: variant); override;
  public
    class function JSONType: TJSONType; override;
    Procedure Clear;  override;
<<<<<<< HEAD
<<<<<<< HEAD
    Function Clone : TJSONData; override;
  end;
  TJSONNullClass = Class of TJSONNull;
=======
  end;
>>>>>>> graemeg/fixes_2_2
=======
  end;
>>>>>>> origin/fixes_2_2

  TJSONArrayIterator = procedure(Item: TJSONData; Data: TObject; var Continue: Boolean) of object;

  { TJSONArray }
  TJSONObject = Class;

  TJSONArray = class(TJSONData)
  Private
    FList : TFPObjectList;
    function GetArrays(Index : Integer): TJSONArray;
    function GetBooleans(Index : Integer): Boolean;
    function GetFloats(Index : Integer): TJSONFloat;
    function GetIntegers(Index : Integer): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetInt64s(Index : Integer): Int64;
    function GetNulls(Index : Integer): Boolean;
    function GetObjects(Index : Integer): TJSONObject;
    function GetQWords(Index : Integer): QWord;
    function GetStrings(Index : Integer): TJSONStringType;
    function GetUnicodeStrings(Index : Integer): TJSONUnicodeStringType;
=======
    function GetNulls(Index : Integer): Boolean;
    function GetObjects(Index : Integer): TJSONObject;
    function GetStrings(Index : Integer): TJSONStringType;
>>>>>>> graemeg/fixes_2_2
=======
    function GetNulls(Index : Integer): Boolean;
    function GetObjects(Index : Integer): TJSONObject;
    function GetStrings(Index : Integer): TJSONStringType;
>>>>>>> origin/fixes_2_2
    function GetTypes(Index : Integer): TJSONType;
    procedure SetArrays(Index : Integer; const AValue: TJSONArray);
    procedure SetBooleans(Index : Integer; const AValue: Boolean);
    procedure SetFloats(Index : Integer; const AValue: TJSONFloat);
    procedure SetIntegers(Index : Integer; const AValue: Integer);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SetInt64s(Index : Integer; const AValue: Int64);
    procedure SetObjects(Index : Integer; const AValue: TJSONObject);
    procedure SetQWords(Index : Integer; AValue: QWord);
    procedure SetStrings(Index : Integer; const AValue: TJSONStringType);
    procedure SetUnicodeStrings(Index : Integer; const AValue: TJSONUnicodeStringType);
  protected
    Function DoFindPath(Const APath : TJSONStringType; Out NotFound : TJSONStringType) : TJSONdata; override;
=======
    procedure SetObjects(Index : Integer; const AValue: TJSONObject);
    procedure SetStrings(Index : Integer; const AValue: TJSONStringType);
  protected
>>>>>>> graemeg/fixes_2_2
=======
    procedure SetObjects(Index : Integer; const AValue: TJSONObject);
    procedure SetStrings(Index : Integer; const AValue: TJSONStringType);
  protected
>>>>>>> origin/fixes_2_2
    Procedure Converterror(From : Boolean);
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> origin/fixes_2_2
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
    function GetValue: variant; override;
    procedure SetValue(const AValue: variant); override;
    function GetCount: Integer; override;
    function GetItem(Index : Integer): TJSONData; override;
    procedure SetItem(Index : Integer; const AValue: TJSONData); override;
<<<<<<< HEAD
<<<<<<< HEAD
    Function DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; override;
  public
    Constructor Create; overload; reintroduce;
    Constructor Create(const Elements : Array of Const); overload;
    Destructor Destroy; override;
    class function JSONType: TJSONType; override;
    Function Clone : TJSONData; override;
    // Examine
    procedure Iterate(Iterator : TJSONArrayIterator; Data: TObject);
    function IndexOf(obj: TJSONData): Integer;
    function GetEnumerator: TBaseJSONEnumerator; override;
=======
=======
>>>>>>> origin/fixes_2_2
  public
    Constructor Create; overload;
    Constructor Create(const Elements : Array of Const); overload;
    Destructor Destroy; override;
    class function JSONType: TJSONType; override;
    // Examine
    procedure Iterate(Iterator : TJSONArrayIterator; Data: TObject);
    function IndexOf(obj: TJSONData): Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // Manipulate
    Procedure Clear;  override;
    function Add(Item : TJSONData): Integer;
    function Add(I : Integer): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
    function Add(I : Int64): Int64;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function Add(I : QWord): QWord;
    function Add(const S : String): Integer;
    function Add(const S : UnicodeString): Integer;
=======
    function Add(const S : String): Integer;
>>>>>>> graemeg/cpstrnew
=======
    function Add(const S : String): Integer;
>>>>>>> graemeg/cpstrnew
=======
    function Add(const S : String): Integer;
>>>>>>> graemeg/cpstrnew
=======
    function Add(const S : String): Integer;
>>>>>>> origin/cpstrnew
=======
    function Add(S : String): Integer;
>>>>>>> graemeg/fixes_2_2
=======
    function Add(S : String): Integer;
>>>>>>> origin/fixes_2_2
    function Add: Integer;
    function Add(F : TJSONFloat): Integer;
    function Add(B : Boolean): Integer;
    function Add(AnArray : TJSONArray): Integer;
    function Add(AnObject: TJSONObject): Integer;
    Procedure Delete(Index : Integer);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure Exchange(Index1, Index2: Integer);
    function Extract(Item: TJSONData): TJSONData;
    function Extract(Index : Integer): TJSONData;
    procedure Insert(Index: Integer);
    procedure Insert(Index: Integer; Item : TJSONData);
    procedure Insert(Index: Integer; I : Integer);
    procedure Insert(Index: Integer; I : Int64);
    procedure Insert(Index: Integer; I : QWord);
    procedure Insert(Index: Integer; const S : String);
    procedure Insert(Index: Integer; const S : UnicodeString);
    procedure Insert(Index: Integer; F : TJSONFloat);
    procedure Insert(Index: Integer; B : Boolean);
    procedure Insert(Index: Integer; AnArray : TJSONArray);
    procedure Insert(Index: Integer; AnObject: TJSONObject);
    procedure Move(CurIndex, NewIndex: Integer);
    Procedure Remove(Item : TJSONData);
    Procedure Sort(Compare: TListSortCompare);
=======
    Procedure Remove(Item : TJSONData);
>>>>>>> graemeg/fixes_2_2
=======
    Procedure Remove(Item : TJSONData);
>>>>>>> origin/fixes_2_2
    // Easy Access Properties.
    property Items;default;
    Property Types[Index : Integer] : TJSONType Read GetTypes;
    Property Nulls[Index : Integer] : Boolean Read GetNulls;
    Property Integers[Index : Integer] : Integer Read GetIntegers Write SetIntegers;
<<<<<<< HEAD
<<<<<<< HEAD
    Property Int64s[Index : Integer] : Int64 Read GetInt64s Write SetInt64s;
    Property QWords[Index : Integer] : QWord Read GetQWords Write SetQWords;
    Property Strings[Index : Integer] : TJSONStringType Read GetStrings Write SetStrings;
    Property UnicodeStrings[Index : Integer] : TJSONUnicodeStringType Read GetUnicodeStrings Write SetUnicodeStrings;
=======
    Property Strings[Index : Integer] : TJSONStringType Read GetStrings Write SetStrings;
>>>>>>> graemeg/fixes_2_2
=======
    Property Strings[Index : Integer] : TJSONStringType Read GetStrings Write SetStrings;
>>>>>>> origin/fixes_2_2
    Property Floats[Index : Integer] : TJSONFloat Read GetFloats Write SetFloats;
    Property Booleans[Index : Integer] : Boolean Read GetBooleans Write SetBooleans;
    Property Arrays[Index : Integer] : TJSONArray Read GetArrays Write SetArrays;
    Property Objects[Index : Integer] : TJSONObject Read GetObjects Write SetObjects;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  TJSONArrayClass = Class of TJSONArray;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  TJSONObjectIterator = procedure(Const AName : TJSONStringType; Item: TJSONData; Data: TObject; var Continue: Boolean) of object;

  { TJSONObject }

  TJSONObject = class(TJSONData)
  private
<<<<<<< HEAD
<<<<<<< HEAD
    Const
      ElementStart   : Array[Boolean] of TJSONStringType = ('"','');
      SpacedQuoted   : Array[Boolean] of TJSONStringType = ('" : ',' : ');
      UnSpacedQuoted : Array[Boolean] of TJSONStringType = ('":',':');
      ObjStartSeps   : Array[Boolean] of TJSONStringType = ('{ ','{');
      ObjEndSeps     : Array[Boolean] of TJSONStringType = (' }','}');
    Class var FUnquotedMemberNames: Boolean;
    Class var FObjStartSep,FObjEndSep,FElementEnd,FElementStart : TJSONStringType;
    Class procedure DetermineElementQuotes;
  Private
    FHash : TFPHashObjectList; // Careful : Names limited to 255 chars.
    function GetArrays(const AName : String): TJSONArray;
    function GetBooleans(const AName : String): Boolean;
    function GetElements(const AName: string): TJSONData;
    function GetFloats(const AName : String): TJSONFloat;
    function GetIntegers(const AName : String): Integer;
    function GetInt64s(const AName : String): Int64;
    function GetIsNull(const AName : String): Boolean; reintroduce;
    function GetNameOf(Index : Integer): TJSONStringType;
    function GetObjects(const AName : String): TJSONObject;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function GetQWords(AName : String): QWord;
    function GetStrings(const AName : String): TJSONStringType;
    function GetUnicodeStrings(const AName : String): TJSONUnicodeStringType;
=======
    function GetStrings(const AName : String): TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
    function GetStrings(const AName : String): TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
    function GetStrings(const AName : String): TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
    function GetStrings(const AName : String): TJSONStringType;
>>>>>>> origin/cpstrnew
    function GetTypes(const AName : String): TJSONType;
    procedure SetArrays(const AName : String; const AValue: TJSONArray);
    procedure SetBooleans(const AName : String; const AValue: Boolean);
    procedure SetElements(const AName: string; const AValue: TJSONData);
    procedure SetFloats(const AName : String; const AValue: TJSONFloat);
    procedure SetIntegers(const AName : String; const AValue: Integer);
    procedure SetInt64s(const AName : String; const AValue: Int64);
    procedure SetIsNull(const AName : String; const AValue: Boolean);
    procedure SetObjects(const AName : String; const AValue: TJSONObject);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SetQWords(AName : String; AValue: QWord);
    procedure SetStrings(const AName : String; const AValue: TJSONStringType);
    procedure SetUnicodeStrings(const AName : String; const AValue: TJSONUnicodeStringType);
    class function GetUnquotedMemberNames: Boolean; static;
    class procedure SetUnquotedMemberNames(AValue: Boolean); static;
=======
    procedure SetStrings(const AName : String; const AValue: TJSONStringType);
>>>>>>> graemeg/cpstrnew
=======
    procedure SetStrings(const AName : String; const AValue: TJSONStringType);
>>>>>>> graemeg/cpstrnew
=======
    procedure SetStrings(const AName : String; const AValue: TJSONStringType);
>>>>>>> graemeg/cpstrnew
=======
    procedure SetStrings(const AName : String; const AValue: TJSONStringType);
>>>>>>> origin/cpstrnew
  protected
    Function DoFindPath(Const APath : TJSONStringType; Out NotFound : TJSONStringType) : TJSONdata; override;
=======
=======
>>>>>>> origin/fixes_2_2
    FHash : TFPHashObjectList; // Careful : Names limited to 255 chars.
    function GetArrays(AName : String): TJSONArray;
    function GetBooleans(AName : String): Boolean;
    function GetElements(AName: string): TJSONData;
    function GetFloats(AName : String): TJSONFloat;
    function GetIntegers(AName : String): Integer;
    function GetIsNull(AName : String): Boolean;
    function GetNameOf(Index : Integer): TJSONStringType;
    function GetObjects(AName : String): TJSONObject;
    function GetStrings(AName : String): TJSONStringType;
    function GetTypes(AName : String): TJSONType;
    procedure SetArrays(AName : String; const AValue: TJSONArray);
    procedure SetBooleans(AName : String; const AValue: Boolean);
    procedure SetElements(AName: string; const AValue: TJSONData);
    procedure SetFloats(AName : String; const AValue: TJSONFloat);
    procedure SetIntegers(AName : String; const AValue: Integer);
    procedure SetIsNull(AName : String; const AValue: Boolean);
    procedure SetObjects(AName : String; const AValue: TJSONObject);
    procedure SetStrings(AName : String; const AValue: TJSONStringType);
  protected
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Procedure Converterror(From : Boolean);
    function GetAsBoolean: Boolean; override;
    function GetAsFloat: TJSONFloat; override;
    function GetAsInteger: Integer; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function GetAsInt64: Int64; override;
    function GetAsQWord: QWord; override;
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
    procedure SetAsInt64(const AValue: Int64); override;
    procedure SetAsQword(const AValue: QWord); override;
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure SetAsBoolean(const AValue: Boolean); override;
    procedure SetAsFloat(const AValue: TJSONFloat); override;
    procedure SetAsInteger(const AValue: Integer); override;
>>>>>>> origin/fixes_2_2
    function GetAsJSON: TJSONStringType; override;
    function GetAsString: TJSONStringType; override;
    procedure SetAsString(const AValue: TJSONStringType); override;
    function GetValue: variant; override;
    procedure SetValue(const AValue: variant); override;
    function GetCount: Integer; override;
    function GetItem(Index : Integer): TJSONData; override;
    procedure SetItem(Index : Integer; const AValue: TJSONData); override;
<<<<<<< HEAD
<<<<<<< HEAD
    Function DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; override;
  public
    constructor Create; reintroduce;
    Constructor Create(const Elements : Array of Const); overload;
    destructor Destroy; override;
    class function JSONType: TJSONType; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Class Property UnquotedMemberNames : Boolean Read GetUnquotedMemberNames Write SetUnquotedMemberNames;
    Function Clone : TJSONData; override;
    function GetEnumerator: TBaseJSONEnumerator; override;
=======
    Function Clone : TJSONData; override;
>>>>>>> graemeg/cpstrnew
=======
    Function Clone : TJSONData; override;
>>>>>>> graemeg/cpstrnew
=======
    Function Clone : TJSONData; override;
>>>>>>> graemeg/cpstrnew
=======
    Function Clone : TJSONData; override;
>>>>>>> origin/cpstrnew
    // Examine
    procedure Iterate(Iterator : TJSONObjectIterator; Data: TObject);
    function IndexOf(Item: TJSONData): Integer;
    Function IndexOfName(const AName: TJSONStringType; CaseInsensitive : Boolean = False): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Function Find(Const AName : String) : TJSONData; overload;
    Function Find(Const AName : String; AType : TJSONType) : TJSONData; overload;
    Function Get(Const AName : String) : Variant;
    Function Get(Const AName : String; ADefault : TJSONFloat) : TJSONFloat;
    Function Get(Const AName : String; ADefault : Integer) : Integer;
    Function Get(Const AName : String; ADefault : Int64) : Int64;
    Function Get(Const AName : String; ADefault : QWord) : QWord;
    Function Get(Const AName : String; ADefault : Boolean) : Boolean;
    Function Get(Const AName : String; ADefault : TJSONStringType) : TJSONStringType;
    Function Get(Const AName : String; ADefault : TJSONUnicodeStringType) : TJSONUnicodeStringType;
    Function Get(Const AName : String; ADefault : TJSONArray) : TJSONArray;
    Function Get(Const AName : String; ADefault : TJSONObject) : TJSONObject;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  public
    constructor Create;
    Constructor Create(const Elements : Array of Const); overload;
    destructor Destroy; override;
    class function JSONType: TJSONType; override;
    // Examine
    procedure Iterate(Iterator : TJSONObjectIterator; Data: TObject);
    function IndexOf(Item: TJSONData): Integer;
    Function IndexOfName(const AName: TJSONStringType): Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    // Manipulate
    Procedure Clear;  override;
    function Add(const AName: TJSONStringType; AValue: TJSONData): Integer; overload;
    function Add(const AName: TJSONStringType; AValue: Boolean): Integer; overload;
    function Add(const AName: TJSONStringType; AValue: TJSONFloat): Integer; overload;
<<<<<<< HEAD
<<<<<<< HEAD
    function Add(const AName, AValue: TJSONStringType): Integer; overload;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function Add(const AName : String; AValue: TJSONUnicodeStringType): Integer; overload;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function Add(const AName: TJSONStringType; Avalue: Integer): Integer; overload;
    function Add(const AName: TJSONStringType; Avalue: Int64): Integer; overload;
    function Add(const AName: TJSONStringType; Avalue: QWord): Integer; overload;
    function Add(const AName: TJSONStringType): Integer; overload;
    function Add(const AName: TJSONStringType; AValue : TJSONArray): Integer; overload;
    procedure Delete(Index : Integer);
    procedure Delete(Const AName : string);
    procedure Remove(Item : TJSONData);
    Function Extract(Index : Integer) : TJSONData;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Function Extract(Const AName : string) : TJSONData;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    function Add(const AName: TJSONStringType; AValue: TJSONStringType): Integer; overload;
    function Add(const AName: TJSONStringType; Avalue: Integer): Integer; overload;
    function Add(const AName: TJSONStringType): Integer; overload;
    function Add(const AName: TJSONStringType; AValue : TJSONArray): Integer; overload;
    procedure Delete(Index : Integer);
    procedure Remove(Item : TJSONData);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    // Easy access properties.
    property Names[Index : Integer] : TJSONStringType read GetNameOf;
    property Elements[AName: string] : TJSONData read GetElements write SetElements; default;

    Property Types[AName : String] : TJSONType Read GetTypes;
    Property Nulls[AName : String] : Boolean Read GetIsNull Write SetIsNull;
    Property Floats[AName : String] : TJSONFloat Read GetFloats Write SetFloats;
    Property Integers[AName : String] : Integer Read GetIntegers Write SetIntegers;
<<<<<<< HEAD
<<<<<<< HEAD
    Property Int64s[AName : String] : Int64 Read GetInt64s Write SetInt64s;
    Property QWords[AName : String] : QWord Read GetQWords Write SetQWords;
    Property Strings[AName : String] : TJSONStringType Read GetStrings Write SetStrings;
    Property UnicodeStrings[AName : String] : TJSONUnicodeStringType Read GetUnicodeStrings Write SetUnicodeStrings;
=======
    Property Strings[AName : String] : TJSONStringType Read GetStrings Write SetStrings;
>>>>>>> graemeg/fixes_2_2
=======
    Property Strings[AName : String] : TJSONStringType Read GetStrings Write SetStrings;
>>>>>>> origin/fixes_2_2
    Property Booleans[AName : String] : Boolean Read GetBooleans Write SetBooleans;
    Property Arrays[AName : String] : TJSONArray Read GetArrays Write SetArrays;
    Property Objects[AName : String] : TJSONObject Read GetObjects Write SetObjects;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  TJSONObjectClass = Class of TJSONObject;

  EJSON = Class(Exception);
<<<<<<< HEAD
=======
  
Function StringToJSONString(const S : TJSONStringType) : TJSONStringType;
Function JSONStringToString(const S : TJSONStringType) : TJSONStringType;
Function JSONTypeName(JSONType : TJSONType) : String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

  TJSONParserHandler = Procedure(AStream : TStream; Const AUseUTF8 : Boolean; Out Data : TJSONData);

Function SetJSONInstanceType(AType : TJSONInstanceType; AClass : TJSONDataClass) : TJSONDataClass;
Function GetJSONInstanceType(AType : TJSONInstanceType) : TJSONDataClass;

Function StringToJSONString(const S : TJSONStringType) : TJSONStringType;
Function JSONStringToString(const S : TJSONStringType) : TJSONStringType;
Function JSONTypeName(JSONType : TJSONType) : String;

// These functions create JSONData structures, taking into account the instance types
Function CreateJSON : TJSONNull;
Function CreateJSON(Data : Boolean) : TJSONBoolean;
Function CreateJSON(Data : Integer) : TJSONIntegerNumber;
Function CreateJSON(Data : Int64) : TJSONInt64Number;
Function CreateJSON(Data : QWord) : TJSONQWordNumber;
Function CreateJSON(Data : TJSONFloat) : TJSONFloatNumber;
Function CreateJSON(Data : TJSONStringType) : TJSONString;
Function CreateJSON(Data : TJSONUnicodeStringType) : TJSONString;
Function CreateJSONArray(Data : Array of const) : TJSONArray;
Function CreateJSONObject(Data : Array of const) : TJSONObject;

// These functions rely on a callback. If the callback is not set, they will raise an error.
// When the jsonparser unit is included in the project, the callback is automatically set.
Function GetJSON(Const JSON : TJSONStringType; Const UseUTF8 : Boolean = True) : TJSONData;
Function GetJSON(Const JSON : TStream; Const UseUTF8 : Boolean = True) : TJSONData;
Function SetJSONParserHandler(AHandler : TJSONParserHandler) : TJSONParserHandler;
Function GetJSONParserHandler : TJSONParserHandler;
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew

implementation

Uses typinfo;
=======
=======
>>>>>>> origin/fixes_2_2

  EJSON = Class(Exception);
  
Function StringToJSONString(S : TJSONStringType) : TJSONStringType;
Function JSONStringToString(S : TJSONStringType) : TJSONStringType;



implementation
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Resourcestring
  SErrCannotConvertFromNull = 'Cannot convert data from Null value';
  SErrCannotConvertToNull = 'Cannot convert data to Null value';
  SErrCannotConvertFromArray = 'Cannot convert data from array value';
  SErrCannotConvertToArray = 'Cannot convert data to array value';
  SErrCannotConvertFromObject = 'Cannot convert data from object value';
  SErrCannotConvertToObject = 'Cannot convert data to object value';
  SErrInvalidFloat = 'Invalid float value : %s';
  SErrInvalidInteger = 'Invalid float value : %s';
  SErrCannotSetNotIsNull = 'IsNull cannot be set to False';
  SErrCannotAddArrayTwice = 'Adding an array object to an array twice is not allowed';
  SErrCannotAddObjectTwice = 'Adding an object to an array twice is not allowed';
  SErrUnknownTypeInConstructor = 'Unknown type in JSON%s constructor: %d';
  SErrNotJSONData = 'Cannot add object of type %s to TJSON%s';
  SErrPointerNotNil = 'Cannot add non-nil pointer to JSON%s';
  SErrOddNumber = 'TJSONObject must be constructed with name,value pairs';
  SErrNameMustBeString = 'TJSONObject constructor element name at pos %d is not a string';
<<<<<<< HEAD
<<<<<<< HEAD
  SErrNonexistentElement = 'Unknown object member: "%s"';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SErrPathElementNotFound = 'Path "%s" invalid: element "%s" not found.';
  SErrWrongInstanceClass = 'Cannot set instance class: %s does not descend from %s.';
  SErrNoParserHandler = 'No JSON parser handler installed. Recompile your project with the jsonparser unit included';

Var
  DefaultJSONInstanceTypes :
    Array [TJSONInstanceType] of TJSONDataClass = (TJSONData, TJSONIntegerNumber,
    TJSONInt64Number, TJSONQWordNumber, TJSONFloatNumber, TJSONString, TJSONBoolean, TJSONNull, TJSONArray,
    TJSONObject);
Const
  MinJSONInstanceTypes :
    Array [TJSONInstanceType] of TJSONDataClass = (TJSONData, TJSONIntegerNumber,
    TJSONInt64Number, TJSONQWordNumber, TJSONFloatNumber, TJSONString, TJSONBoolean, TJSONNull, TJSONArray,
    TJSONObject);

function SetJSONInstanceType(AType: TJSONInstanceType; AClass: TJSONDataClass): TJSONDataClass;
begin
  if AClass=Nil then
    TJSONData.DoError(SErrWrongInstanceClass,['Nil',MinJSONInstanceTypes[AType].ClassName]);
  if Not AClass.InheritsFrom(MinJSONINstanceTypes[AType]) then
    TJSONData.DoError(SErrWrongInstanceClass,[AClass.ClassName,MinJSONInstanceTypes[AType].ClassName]);
  Result:=DefaultJSONInstanceTypes[AType];
  DefaultJSONINstanceTypes[AType]:=AClass;
end;

function GetJSONInstanceType(AType: TJSONInstanceType): TJSONDataClass;
begin
  Result:=DefaultJSONInstanceTypes[AType]
end;

function StringToJSONString(const S: TJSONStringType): TJSONStringType;
=======
  
Function StringToJSONString(const S : TJSONStringType) : TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
  
Function StringToJSONString(const S : TJSONStringType) : TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
  
Function StringToJSONString(const S : TJSONStringType) : TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
  
Function StringToJSONString(const S : TJSONStringType) : TJSONStringType;
>>>>>>> origin/cpstrnew

Var
  I,J,L : Integer;
  P : PJSONCharType;
=======
=======
>>>>>>> origin/fixes_2_2
  
  
Function StringToJSONString(S : TJSONStringType) : TJSONStringType;

Var
  I,J,L : Integer;
  P : Pchar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  I:=1;
  J:=1;
  Result:='';
  L:=Length(S);
<<<<<<< HEAD
<<<<<<< HEAD
  P:=PJSONCharType(S);
  While I<=L do
    begin
    if (AnsiChar(P^) in ['"','/','\',#8,#9,#10,#12,#13]) then
=======
=======
>>>>>>> origin/fixes_2_2
  P:=PChar(S);
  While I<=L do
    begin
    if (P^ in ['"','/','\',#8,#9,#10,#12,#13]) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      begin
      Result:=Result+Copy(S,J,I-J);
      Case P^ of
        '\' : Result:=Result+'\\';
        '/' : Result:=Result+'\/';
        '"' : Result:=Result+'\"';
        #8  : Result:=Result+'\b';
        #9  : Result:=Result+'\t';
        #10 : Result:=Result+'\n';
        #12 : Result:=Result+'\f';
        #13 : Result:=Result+'\r';
      end;
      J:=I+1;
      end;
    Inc(I);
    Inc(P);
    end;
  Result:=Result+Copy(S,J,I-1);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function JSONStringToString(const S: TJSONStringType): TJSONStringType;
=======
Function JSONStringToString(const S : TJSONStringType) : TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
Function JSONStringToString(const S : TJSONStringType) : TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
Function JSONStringToString(const S : TJSONStringType) : TJSONStringType;
>>>>>>> graemeg/cpstrnew
=======
Function JSONStringToString(const S : TJSONStringType) : TJSONStringType;
>>>>>>> origin/cpstrnew

Var
  I,J,L : Integer;
  P : PJSONCharType;
=======
=======
>>>>>>> origin/fixes_2_2
Function JSONStringToString(S : TJSONStringType) : TJSONStringType;

Var
  I,J,L : Integer;
  P : PChar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  w : String;

begin
  I:=1;
  J:=1;
  L:=Length(S);
  Result:='';
<<<<<<< HEAD
<<<<<<< HEAD
  P:=PJSONCharType(S);
=======
  P:=PChar(S);
>>>>>>> graemeg/fixes_2_2
=======
  P:=PChar(S);
>>>>>>> origin/fixes_2_2
  While (I<=L) do
    begin
    if (P^='\') then
      begin
      Result:=Result+Copy(S,J,I-J);
      Inc(P);
      If (P^<>#0) then
        begin
        Inc(I);
<<<<<<< HEAD
<<<<<<< HEAD
        Case AnsiChar(P^) of
=======
        Case P^ of
>>>>>>> graemeg/fixes_2_2
=======
        Case P^ of
>>>>>>> origin/fixes_2_2
          '\','"','/'
              : Result:=Result+P^;
          'b' : Result:=Result+#8;
          't' : Result:=Result+#9;
          'n' : Result:=Result+#10;
          'f' : Result:=Result+#12;
          'r' : Result:=Result+#13;
          'u' : begin
                W:=Copy(S,I+1,4);
                Inc(I,4);
                Inc(P,4);
                Result:=Result+WideChar(StrToInt('$'+W));
                end;
        end;
        end;
      J:=I+1;
      end;
    Inc(I);
    Inc(P);
    end;
  Result:=Result+Copy(S,J,I-J+1);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function JSONTypeName(JSONType: TJSONType): String;
begin
  Result:=GetEnumName(TypeInfo(TJSONType),Ord(JSONType));
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
=======
=======

>>>>>>> origin/cpstrnew

>>>>>>> graemeg/cpstrnew

>>>>>>> graemeg/cpstrnew

>>>>>>> graemeg/cpstrnew

function CreateJSON: TJSONNull;
begin
  Result:=TJSONNullClass(DefaultJSONInstanceTypes[jitNull]).Create
end;

function CreateJSON(Data: Boolean): TJSONBoolean;
begin
  Result:=TJSONBooleanClass(DefaultJSONInstanceTypes[jitBoolean]).Create(Data);
end;

function CreateJSON(Data: Integer): TJSONIntegerNumber;
begin
  Result:=TJSONIntegerNumberCLass(DefaultJSONInstanceTypes[jitNumberInteger]).Create(Data);
end;

function CreateJSON(Data: Int64): TJSONInt64Number;
begin
  Result:=TJSONInt64NumberCLass(DefaultJSONInstanceTypes[jitNumberInt64]).Create(Data);
end;

function CreateJSON(Data: QWord): TJSONQWordNumber;
begin
  Result:=TJSONQWordNumberClass(DefaultJSONInstanceTypes[jitNumberQWord]).Create(Data);
end;

function CreateJSON(Data: TJSONFloat): TJSONFloatNumber;
begin
  Result:=TJSONFloatNumberCLass(DefaultJSONInstanceTypes[jitNumberFloat]).Create(Data);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function CreateJSON(Data: TJSONStringType): TJSONString;
=======
Function TJSONData.FormatJSON(Options : TFormatOptions = DefaultFormat; IndentSize : Integer = DefaultIndentSize) : TJSONStringType;
=======
Function TJSONData.FormatJSON(Options : TFormatOptions = DefaultFormat; IndentSize : Integer = DefaultIndentSize) : TJSONStringType;
=======
Function TJSONData.FormatJSON(Options : TFormatOptions = DefaultFormat; IndentSize : Integer = DefaultIndentSize) : TJSONStringType;

begin
  Result:=DoFormatJSON(Options,0,IndentSize);
end;

Function TJSONData.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 

begin
  Result:=AsJSON;
end;

{ TJSONnumber }
>>>>>>> origin/cpstrnew

begin
  Result:=DoFormatJSON(Options,0,IndentSize);
end;

Function TJSONData.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 

begin
  Result:=AsJSON;
end;

{ TJSONnumber }
>>>>>>> graemeg/cpstrnew

begin
  Result:=DoFormatJSON(Options,0,IndentSize);
end;

<<<<<<< HEAD
Function TJSONData.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 

=======
function TJSONString.Clone: TJSONData;

begin
  Result:=TJSONString.Create(Self.FValue);
end;

function TJSONstring.GetValue: Variant;
>>>>>>> origin/cpstrnew
begin
  Result:=AsJSON;
end;

{ TJSONnumber }

<<<<<<< HEAD
class function TJSONnumber.JSONType: TJSONType;
>>>>>>> graemeg/cpstrnew
=======
function TJSONString.Clone: TJSONData;

begin
  Result:=TJSONString.Create(Self.FValue);
end;

function TJSONstring.GetValue: Variant;
>>>>>>> graemeg/cpstrnew
begin
  Result:=TJSONStringCLass(DefaultJSONInstanceTypes[jitString]).Create(Data);
end;

function CreateJSON(Data: TJSONUnicodeStringType): TJSONString;
begin
  Result:=TJSONStringCLass(DefaultJSONInstanceTypes[jitString]).Create(Data);
end;

<<<<<<< HEAD
function CreateJSONArray(Data: array of const): TJSONArray;
begin
  Result:=TJSONArrayCLass(DefaultJSONInstanceTypes[jitArray]).Create(Data);
end;
=======
Function TJSONData.FormatJSON(Options : TFormatOptions = DefaultFormat; IndentSize : Integer = DefaultIndentSize) : TJSONStringType;

<<<<<<< HEAD
=======
function TJSONString.Clone: TJSONData;

begin
  Result:=TJSONString.Create(Self.FValue);
end;

function TJSONstring.GetValue: Variant;
>>>>>>> graemeg/cpstrnew
begin
  Result:=DoFormatJSON(Options,0,IndentSize);
end;

Function TJSONData.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 

begin
  Result:=AsJSON;
end;

{ TJSONnumber }
>>>>>>> graemeg/cpstrnew

function CreateJSONObject(Data: array of const): TJSONObject;
begin
  Result:=TJSONObjectCLass(DefaultJSONInstanceTypes[jitObject]).Create(Data);
end;

Var
  JPH : TJSONParserHandler;

function GetJSON(const JSON: TJSONStringType; const UseUTF8: Boolean
  ): TJSONData;

Var
  SS : TStringStream;
begin
  SS:=TStringStream.Create(JSON);
  try
    Result:=GetJSON(SS,UseUTF8);
  finally
    SS.Free;
  end;
end;

function GetJSON(const JSON: TStream; const UseUTF8: Boolean): TJSONData;

begin
  Result:=Nil;
  If (JPH=Nil) then
    TJSONData.DoError(SErrNoParserHandler);
  JPH(JSON,UseUTF8,Result);
end;

<<<<<<< HEAD
function SetJSONParserHandler(AHandler: TJSONParserHandler): TJSONParserHandler;
=======
function TJSONString.Clone: TJSONData;

begin
  Result:=TJSONString.Create(Self.FValue);
end;

function TJSONstring.GetValue: Variant;
>>>>>>> graemeg/cpstrnew
begin
  Result:=JPH;
  JPH:=AHandler;
end;

function GetJSONParserHandler: TJSONParserHandler;
begin
  Result:=JPH;
end;

<<<<<<< HEAD
Type
  { TJSONEnumerator }

  TJSONEnumerator = class(TBaseJSONEnumerator)
  Private
    FData : TJSONData;
  public
    Constructor Create(AData : TJSONData);
    function GetCurrent: TJSONEnum; override;
    function MoveNext : Boolean; override;
  end;

<<<<<<< HEAD
  { TJSONArrayEnumerator }
=======
=======
>>>>>>> graemeg/cpstrnew
constructor TJSONstring.Create(const AValue: TJSONStringType);
begin
  FValue:=AValue;
end;
>>>>>>> graemeg/cpstrnew

  TJSONArrayEnumerator = class(TBaseJSONEnumerator)
  Private
    FData : TJSONArray;
    FCurrent : Integer;
  public
    Constructor Create(AData : TJSONArray);
    function GetCurrent: TJSONEnum; override;
    function MoveNext : Boolean; override;
  end;

  { TJSONObjectEnumerator }

  TJSONObjectEnumerator = class(TBaseJSONEnumerator)
  Private
    FData : TJSONObject;
    FCurrent : Integer;
  public
    Constructor Create(AData : TJSONObject);
    function GetCurrent: TJSONEnum; override;
    function MoveNext : Boolean; override;
  end;

<<<<<<< HEAD
{ TJSONQWordNumber }

function TJSONQWordNumber.GetAsBoolean: Boolean;
=======
constructor TJSONstring.Create(const AValue: TJSONStringType);
>>>>>>> origin/cpstrnew
begin
  Result:=FValue<>0;
end;

function TJSONQWordNumber.GetAsFloat: TJSONFloat;
begin
  Result:= FValue;
end;

function TJSONQWordNumber.GetAsInteger: Integer;
begin
  Result := FValue;
end;

<<<<<<< HEAD
function TJSONQWordNumber.GetAsInt64: Int64;
begin
  Result := FValue;
end;
=======
function TJSONBoolean.Clone: TJSONData;
begin
  Result:=TJSONBoolean.Create(Self.Fvalue);
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
function TJSONBoolean.Clone: TJSONData;
begin
  Result:=TJSONBoolean.Create(Self.Fvalue);
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
function TJSONBoolean.Clone: TJSONData;
begin
  Result:=TJSONBoolean.Create(Self.Fvalue);
end;

>>>>>>> origin/cpstrnew

function TJSONQWordNumber.GetAsQWord: QWord;
begin
  Result := FValue;
end;

procedure TJSONQWordNumber.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=Ord(AValue);
end;

procedure TJSONQWordNumber.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=Round(AValue);
end;

procedure TJSONQWordNumber.SetAsInteger(const AValue: Integer);
begin
  FValue:=AValue;
end;

procedure TJSONQWordNumber.SetAsInt64(const AValue: Int64);
begin
  FValue := AValue;
end;

procedure TJSONQWordNumber.SetAsQword(const AValue: QWord);
begin
  FValue:=AValue;
end;

<<<<<<< HEAD
function TJSONQWordNumber.GetAsJSON: TJSONStringType;
=======
constructor TJSONstring.Create(const AValue: TJSONStringType);
>>>>>>> graemeg/cpstrnew
begin
  Result:=AsString;
end;

function TJSONQWordNumber.GetAsString: TJSONStringType;
begin
  Result:=IntToStr(FValue);
end;

procedure TJSONQWordNumber.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=StrToQWord(AValue);
end;

function TJSONQWordNumber.GetValue: variant;
begin
<<<<<<< HEAD
  Result:=FValue;
=======
  If FValue then
    Result:='true'
  else
    Result:='false';
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TJSONQWordNumber.SetValue(const AValue: variant);
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FValue:=AValue;
=======
  Result:=BoolToStr(FValue, True);
>>>>>>> graemeg/cpstrnew
=======
  Result:=BoolToStr(FValue, True);
>>>>>>> graemeg/cpstrnew
=======
  Result:=BoolToStr(FValue, True);
>>>>>>> origin/cpstrnew
end;

constructor TJSONQWordNumber.Create(AValue: QWord);
begin
  FValue := AValue;
end;

<<<<<<< HEAD
class function TJSONQWordNumber.NumberType: TJSONNumberType;
=======
function TJSONBoolean.Clone: TJSONData;
begin
  Result:=TJSONBoolean.Create(Self.Fvalue);
end;


procedure TJSONboolean.SetValue(const AValue: Variant);
>>>>>>> graemeg/cpstrnew
begin
  Result:=ntQWord;
end;

procedure TJSONQWordNumber.Clear;
begin
  FValue:=0;
end;

function TJSONQWordNumber.Clone: TJSONData;
begin
  Result:=TJSONQWordNumberClass(ClassType).Create(Self.FValue);
end;

constructor TJSONObjectEnumerator.Create(AData: TJSONObject);
begin
  FData:=AData;
  FCurrent:=-1;
end;

function TJSONObjectEnumerator.GetCurrent: TJSONEnum;
begin
  Result.KeyNum:=FCurrent;
  Result.Key:=FData.Names[FCurrent];
  Result.Value:=FData.Items[FCurrent];
end;

function TJSONObjectEnumerator.MoveNext: Boolean;
begin
  Inc(FCurrent);
  Result:=FCurrent<FData.Count;
end;

{ TJSONArrayEnumerator }

constructor TJSONArrayEnumerator.Create(AData: TJSONArray);
begin
  FData:=AData;
  FCurrent:=-1;
end;

function TJSONArrayEnumerator.GetCurrent: TJSONEnum;
begin
  Result.KeyNum:=FCurrent;
  Result.Key:=IntToStr(FCurrent);
  Result.Value:=FData.Items[FCurrent];
end;

function TJSONArrayEnumerator.MoveNext: Boolean;
begin
  Inc(FCurrent);
  Result:=FCurrent<FData.Count;
end;

  { TJSONEnumerator }

constructor TJSONEnumerator.Create(AData: TJSONData);
begin
<<<<<<< HEAD
  FData:=AData;
=======
  If FValue then
    Result:='true'
  else
    Result:='false';
>>>>>>> graemeg/cpstrnew
end;

function TJSONEnumerator.GetCurrent: TJSONEnum;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result.Key:='';
  Result.KeyNum:=0;
  Result.Value:=FData;
  FData:=Nil;
=======
  Result:=BoolToStr(FValue, True);
>>>>>>> graemeg/cpstrnew
=======
  Result:='null';
>>>>>>> graemeg/cpstrnew
=======
  Result:='null';
>>>>>>> graemeg/cpstrnew
=======
  Result:='null';
>>>>>>> origin/cpstrnew
end;

function TJSONEnumerator.MoveNext: Boolean;
begin
  Result:=FData<>Nil;
end;



{ TJSONData }

function TJSONData.GetAsUnicodeString: TJSONUnicodeStringType; 

begin
  Result:=UTF8Decode(AsString);
end;

procedure TJSONData.SetAsUnicodeString(const AValue: TJSONUnicodeStringType); 

begin
  AsString:=UTF8Encode(AValue);
end;

function TJSONData.GetItem(Index : Integer): TJSONData;
begin
  Result:=nil;
end;

function TJSONData.GetCount: Integer;
begin
  Result:=0;
end;

constructor TJSONData.Create;
begin
  Clear;
end;
<<<<<<< HEAD
=======

function TJSONNull.Clone: TJSONData;
begin
  Result:=TJSONNull.Create;
end;

<<<<<<< HEAD
=======
function TJSONNull.Clone: TJSONData;
begin
  Result:=TJSONNull.Create;
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
function TJSONNull.Clone: TJSONData;
begin
  Result:=TJSONNull.Create;
end;

>>>>>>> origin/cpstrnew
{$warnings on}
>>>>>>> graemeg/cpstrnew

procedure TJSONData.DumpJSON(S: TStream);

  Procedure W(T : String);

  begin
    if (T<>'') then
      S.WriteBuffer(T[1],Length(T)*SizeOf(Char));
  end;

Var
  I,C : Integer;
  O : TJSONObject;

begin
  Case JSONType of
    jtObject :
      begin
      O:=TJSONObject(Self);
      W('{');
      For I:=0 to O.Count-1 do
        begin
        if (I>0) then
          W(',');
        W('"');
        W(StringToJSONString(O.Names[i]));
        W('":');
        O.Items[I].DumpJSON(S);
        end;
      W('}');
      end;
    jtArray :
      begin
      W('[');
      For I:=0 to Count-1 do
        begin
        if (I>0) then
          W(',');
        Items[I].DumpJSON(S);
        end;
      W(']');
      end
  else
    W(AsJSON)
  end;
end;

class function TJSONData.GetCompressedJSON: Boolean; static;
begin
  Result:=FCompressedJSON;
end;

class procedure TJSONData.DetermineElementSeparators;


begin
  FElementSep:=ElementSeps[FCompressedJSON];
end;

class procedure TJSONData.SetCompressedJSON(AValue: Boolean); static;


begin
  if AValue=FCompressedJSON then exit;
  FCompressedJSON:=AValue;
  DetermineElementSeparators;
  TJSONObject.DetermineElementQuotes;
end;

class procedure TJSONData.DoError(const Msg: String);
begin
  Raise EJSON.Create(Msg);
end;

class procedure TJSONData.DoError(const Fmt: String;
  const Args: array of const);
begin
  Raise EJSON.CreateFmt(Fmt,Args);
end;

function TJSONData.DoFindPath(const APath: TJSONStringType; out
  NotFound: TJSONStringType): TJSONdata;
begin
<<<<<<< HEAD
  If APath<>'' then
    begin
    NotFound:=APath;
    Result:=Nil;
    end
  else
    Result:=Self;
=======
  Result:='null';
>>>>>>> graemeg/cpstrnew
end;

function TJSONData.GetIsNull: Boolean;
begin
  Result:=False;
end;

class function TJSONData.JSONType: TJSONType;
begin
  JSONType:=jtUnknown;
end;

function TJSONData.GetEnumerator: TBaseJSONEnumerator;
begin
  Result:=TJSONEnumerator.Create(Self);
end;

function TJSONData.FindPath(const APath: TJSONStringType): TJSONdata;

Var
  M : TJSONStringType;

begin
  Result:=DoFindPath(APath,M);
end;

function TJSONData.GetPath(const APath: TJSONStringType): TJSONdata;

Var
  M : TJSONStringType;
begin
  Result:=DoFindPath(APath,M);
  If Result=Nil then
    DoError(SErrPathElementNotFound,[APath,M]);
end;

procedure TJSONData.SetItem(Index : Integer; const AValue:
  TJSONData);
begin
  // Do Nothing
end;
<<<<<<< HEAD
=======

function TJSONNull.Clone: TJSONData;
begin
  Result:=TJSONNull.Create;
end;

{$warnings on}

>>>>>>> graemeg/cpstrnew

function TJSONData.FormatJSON(Options: TFormatOptions; Indentsize: Integer
  ): TJSONStringType;

begin
  Result:=DoFormatJSON(Options,0,IndentSize);
end;

function TJSONData.DoFormatJSON(Options: TFormatOptions; CurrentIndent,
  Indent: Integer): TJSONStringType;

begin
  Result:=AsJSON;
end;

{ TJSONnumber }

class function TJSONnumber.JSONType: TJSONType;
begin
  Result:=jtNumber;
end;


{ TJSONstring }

class function TJSONString.JSONType: TJSONType;
begin
  Result:=jtString;
end;

procedure TJSONString.Clear;
begin
  FValue:='';
end;

function TJSONString.Clone: TJSONData;

begin
  Result:=TJSONStringClass(ClassType).Create(Self.FValue);
end;

function TJSONString.GetValue: Variant;
begin
  Result:=FValue;
end;

procedure TJSONString.SetValue(const AValue: Variant);
begin
  FValue:=AValue;
end;


function TJSONString.GetAsBoolean: Boolean;
begin
  Result:=StrToBool(FValue);
end;

function TJSONString.GetAsFloat: TJSONFloat;

Var
  C : Integer;

begin
  Val(FValue,Result,C);
  If (C<>0) then
    If Not TryStrToFloat(FValue,Result) then
      Raise EConvertError.CreateFmt(SErrInvalidFloat,[FValue]);
end;

function TJSONString.GetAsInteger: Integer;
begin
  Result:=StrToInt(FValue);
end;

function TJSONString.GetAsInt64: Int64;
begin
  Result:=StrToInt64(FValue);
end;

function TJSONString.GetAsQWord: QWord;
begin
  Result:=StrToQWord(FValue);
end;

procedure TJSONString.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=BoolToStr(AValue);
end;

procedure TJSONString.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=FloatToStr(AValue);
end;

procedure TJSONString.SetAsInteger(const AValue: Integer);
begin
  FValue:=IntToStr(AValue);
end;

procedure TJSONString.SetAsInt64(const AValue: Int64);
begin
  FValue:=IntToStr(AValue);
end;

procedure TJSONString.SetAsQword(const AValue: QWord);
begin
  FValue:=IntToStr(AValue);
end;

function TJSONString.GetAsJSON: TJSONStringType;
begin
  Result:='"'+StringToJSONString(FValue)+'"';
end;

function TJSONString.GetAsString: TJSONStringType;
begin
  Result:=FValue;
end;

procedure TJSONString.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=AValue;
end;

constructor TJSONString.Create(const AValue: TJSONStringType);
begin
  FValue:=AValue;
end;

constructor TJSONString.Create(const AValue: TJSONUnicodeStringType);
begin
  FValue:=UTF8Encode(AValue);
end;

{ TJSONboolean }


function TJSONBoolean.GetValue: Variant;
begin
  Result:=FValue;
end;

class function TJSONBoolean.JSONType: TJSONType;
begin
  Result:=jtBoolean;
end;

procedure TJSONBoolean.Clear;
begin
  FValue:=False;
end;

function TJSONBoolean.Clone: TJSONData;
begin
  Result:=TJSONBooleanClass(Self.ClassType).Create(Self.Fvalue);
end;


procedure TJSONBoolean.SetValue(const AValue: Variant);
begin
  FValue:=boolean(AValue);
end;

function TJSONBoolean.GetAsBoolean: Boolean;
begin
  Result:=FValue;
end;

function TJSONBoolean.GetAsFloat: TJSONFloat;
begin
  Result:=Ord(FValue);
end;

function TJSONBoolean.GetAsInteger: Integer;
begin
  Result:=Ord(FValue);
end;

function TJSONBoolean.GetAsInt64: Int64;
begin
  Result:=Ord(FValue);
end;

function TJSONBoolean.GetAsQWord: QWord;
begin
  Result:=Ord(FValue);
end;

procedure TJSONBoolean.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=AValue;
end;

procedure TJSONBoolean.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=(AValue<>0)
end;

procedure TJSONBoolean.SetAsInteger(const AValue: Integer);
begin
  FValue:=(AValue<>0)
end;

procedure TJSONBoolean.SetAsInt64(const AValue: Int64);
begin
  FValue:=(AValue<>0)
end;

procedure TJSONBoolean.SetAsQword(const AValue: QWord);
begin
  FValue:=(AValue<>0)
end;

function TJSONBoolean.GetAsJSON: TJSONStringType;
begin
  If FValue then
    Result:='true'
  else
    Result:='false';
end;

function TJSONBoolean.GetAsString: TJSONStringType;
begin
  Result:=BoolToStr(FValue, True);
end;

procedure TJSONBoolean.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=StrToBool(AValue);
end;


constructor TJSONBoolean.Create(AValue: Boolean);
begin
  FValue:=AValue;
end;

{ TJSONnull }

procedure TJSONNull.Converterror(From: Boolean);
begin
  If From then
    DoError(SErrCannotConvertFromNull)
  else
    DoError(SErrCannotConvertToNull);
end;

{$warnings off}
function TJSONNull.GetAsBoolean: Boolean;
begin
  ConvertError(True);
end;

function TJSONNull.GetAsFloat: TJSONFloat;
begin
  ConvertError(True);
end;

function TJSONNull.GetAsInteger: Integer;
begin
  ConvertError(True);
end;

function TJSONNull.GetAsInt64: Int64;
begin
  ConvertError(True);
end;

function TJSONNull.GetAsQWord: QWord;
begin
  ConvertError(True);
end;

function TJSONNull.GetIsNull: Boolean;
begin
  Result:=True;
end;

procedure TJSONNull.SetAsBoolean(const AValue: Boolean);
begin
  ConvertError(False);
end;

procedure TJSONNull.SetAsFloat(const AValue: TJSONFloat);
begin
  ConvertError(False);
end;

procedure TJSONNull.SetAsInteger(const AValue: Integer);
begin
  ConvertError(False);
end;

procedure TJSONNull.SetAsInt64(const AValue: Int64);
begin
  ConvertError(False);
end;

procedure TJSONNull.SetAsQword(const AValue: QWord);
begin
  ConvertError(False);
end;

function TJSONNull.GetAsJSON: TJSONStringType;
begin
  Result:='null';
end;

function TJSONNull.GetAsString: TJSONStringType;
begin
  ConvertError(True);
end;

procedure TJSONNull.SetAsString(const AValue: TJSONStringType);
begin
  ConvertError(True);
end;


function TJSONNull.GetValue: variant;
begin
  Result:=variants.Null;
end;

procedure TJSONNull.SetValue(const AValue: variant);
begin
  ConvertError(False);
end;

class function TJSONNull.JSONType: TJSONType;
begin
  Result:=jtNull;
end;

procedure TJSONNull.Clear;
begin
  // Do nothing
end;

function TJSONNull.Clone: TJSONData;
begin
  Result:=TJSONNullClass(Self.ClassType).Create;
end;

{$warnings on}



{ TJSONFloatNumber }

function TJSONFloatNumber.GetAsBoolean: Boolean;
begin
  Result:=(FValue<>0);
end;

function TJSONFloatNumber.GetAsFloat: TJSONFloat;
begin
  Result:=FValue;
end;

function TJSONFloatNumber.GetAsInteger: Integer;
begin
  Result:=Round(FValue);
end;

function TJSONFloatNumber.GetAsInt64: Int64;
begin
  Result:=Round(FValue);
end;

function TJSONFloatNumber.GetAsQWord: QWord;
begin
  Result:=Round(FValue);
end;

procedure TJSONFloatNumber.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=Ord(AValue);
end;

procedure TJSONFloatNumber.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=AValue;
end;

procedure TJSONFloatNumber.SetAsInteger(const AValue: Integer);
begin
  FValue:=AValue;
end;

procedure TJSONFloatNumber.SetAsInt64(const AValue: Int64);
begin
  FValue:=AValue;
end;

procedure TJSONFloatNumber.SetAsQword(const AValue: QWord);
begin
  FValue:=AValue;
end;

function TJSONFloatNumber.GetAsJSON: TJSONStringType;
begin
  Result:=AsString;
end;

function TJSONFloatNumber.GetAsString: TJSONStringType;
begin
  Str(FValue,Result);
  // Str produces a ' ' in front where the - can go.
  if (Result<>'') and (Result[1]=' ') then
    Delete(Result,1,1);
end;

procedure TJSONFloatNumber.SetAsString(const AValue: TJSONStringType);
Var
  C : Integer;
begin
  Val(AValue,FValue,C);
  If (C<>0) then
    Raise EConvertError.CreateFmt(SErrInvalidFloat,[AValue]);
end;


function TJSONFloatNumber.GetValue: variant;
begin
  Result:=FValue;
end;

procedure TJSONFloatNumber.SetValue(const AValue: variant);
begin
  FValue:=AValue;
end;

constructor TJSONFloatNumber.Create(AValue: TJSONFloat);
begin
  FValue:=AValue;
end;

class function TJSONFloatNumber.NumberType: TJSONNumberType;
begin
  Result:=ntFloat;
end;

procedure TJSONFloatNumber.Clear;
=======


{ TJSONData }


function TJSONData.GetItem(Index : Integer): TJSONData;
begin
  Result:=nil;
end;

function TJSONData.GetCount: Integer;
begin
  Result:=0;
end;

constructor TJSONData.Create;
begin
  Clear;
end;

function TJSONData.GetIsNull: Boolean;
begin
  Result:=False;
end;

class function TJSONData.JSONType: TJSONType;
begin
  JSONType:=jtUnknown;
end;

procedure TJSONData.SetItem(Index : Integer; const AValue:
  TJSONData);
begin
  // Do Nothing
end;

{ TJSONnumber }

class function TJSONnumber.JSONType: TJSONType;
begin
  Result:=jtNumber;
end;


{ TJSONstring }

class function TJSONstring.JSONType: TJSONType;
begin
  Result:=jtString;
end;

procedure TJSONString.Clear;
begin
  FValue:='';
end;

function TJSONstring.GetValue: Variant;
begin
  Result:=FValue;
end;

procedure TJSONstring.SetValue(const AValue: Variant);
begin
  FValue:=AValue;
end;


function TJSONstring.GetAsBoolean: Boolean;
begin
  Result:=StrToBool(FValue);
end;

function TJSONstring.GetAsFloat: TJSONFloat;

Var
  C : Integer;

begin
  Val(FValue,Result,C);
  If (C<>0) then
    If Not TryStrToFloat(FValue,Result) then
      Raise EConvertError.CreateFmt(SErrInvalidFloat,[FValue]);
end;

function TJSONstring.GetAsInteger: Integer;
begin
  Result:=StrToInt(FValue);
end;

procedure TJSONstring.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=BoolToStr(AValue);
end;

procedure TJSONstring.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=FloatToStr(AValue);
end;

procedure TJSONstring.SetAsInteger(const AValue: Integer);
begin
  FValue:=IntToStr(AValue);
end;

function TJSONstring.GetAsJSON: TJSONStringType;
begin
  Result:='"'+StringToJSONString(FValue)+'"';
end;

function TJSONstring.GetAsString: TJSONStringType;
begin
  Result:=FValue;
end;

procedure TJSONstring.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=AValue;
end;

constructor TJSONstring.Create(AValue: TJSONStringType);
begin
  FValue:=AValue;
end;

{ TJSONboolean }


function TJSONboolean.GetValue: Variant;
begin
  Result:=FValue;
end;

class function TJSONboolean.JSONType: TJSONType;
begin
  Result:=jtBoolean;
end;

procedure TJSONBoolean.Clear;
begin
  FValue:=False;
end;


procedure TJSONboolean.SetValue(const AValue: Variant);
begin
  FValue:=boolean(AValue);
end;

function TJSONboolean.GetAsBoolean: Boolean;
begin
  Result:=FValue;
end;

function TJSONboolean.GetAsFloat: TJSONFloat;
begin
  Result:=Ord(FValue);
end;

function TJSONboolean.GetAsInteger: Integer;
begin
  Result:=Ord(FValue);
end;


procedure TJSONboolean.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=AValue;
end;

procedure TJSONboolean.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=(AValue<>0)
end;

procedure TJSONboolean.SetAsInteger(const AValue: Integer);
begin
  FValue:=(AValue<>0)
end;

function TJSONboolean.GetAsJSON: TJSONStringType;
begin
  If FValue then
    Result:='True'
  else
    Result:='False';
end;

function TJSONboolean.GetAsString: TJSONStringType;
begin
  Result:=BoolToStr(FValue);
end;

procedure TJSONboolean.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=StrToBool(AValue);
end;

constructor TJSONboolean.Create(AValue: Boolean);
begin
  FValue:=AValue;
end;

{ TJSONnull }

procedure TJSONnull.Converterror(From : Boolean);
begin
  If From then
    Raise EJSON.Create(SErrCannotConvertFromNull)
  else
    Raise EJSON.Create(SErrCannotConvertToNull);
end;

function TJSONnull.GetAsBoolean: Boolean;
begin
  ConvertError(True);
end;

function TJSONnull.GetAsFloat: TJSONFloat;
begin
  ConvertError(True);
end;

function TJSONnull.GetAsInteger: Integer;
begin
  ConvertError(True);
end;

function TJSONnull.GetIsNull: Boolean;
begin
  Result:=True;
end;

procedure TJSONnull.SetAsBoolean(const AValue: Boolean);
begin
  ConvertError(False);
end;

procedure TJSONnull.SetAsFloat(const AValue: TJSONFloat);
begin
  ConvertError(False);
end;

procedure TJSONnull.SetAsInteger(const AValue: Integer);
begin
  ConvertError(False);
end;

function TJSONnull.GetAsJSON: TJSONStringType;
begin
  Result:='Null';
end;

function TJSONnull.GetAsString: TJSONStringType;
begin
  ConvertError(True);
end;

procedure TJSONnull.SetAsString(const AValue: TJSONStringType);
begin
  ConvertError(True);
end;

function TJSONnull.GetValue: Variant;
begin
  Result:=variants.Null;
end;

procedure TJSONnull.SetValue(const AValue: variant);
begin
  ConvertError(False);
end;

class function TJSONnull.JSONType: TJSONType;
begin
  Result:=jtNull;
end;

procedure TJSONNull.Clear;
begin
  // Do nothing
end;



{ TJSONFloatNumber }

function TJSONFloatNumber.GetAsBoolean: Boolean;
begin
  Result:=(FValue<>0);
end;

function TJSONFloatNumber.GetAsFloat: TJSONFloat;
begin
  Result:=FValue;
end;

function TJSONFloatNumber.GetAsInteger: Integer;
begin
  Result:=Round(FValue);
end;

procedure TJSONFloatNumber.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=Ord(AValue);
end;

procedure TJSONFloatNumber.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=AValue;
end;

procedure TJSONFloatNumber.SetAsInteger(const AValue: Integer);
begin
  FValue:=AValue;
end;

function TJSONFloatNumber.GetAsJSON: TJSONStringType;
begin
  Result:=AsString;
end;

function TJSONFloatNumber.GetAsString: TJSONStringType;
begin
  Str(FValue,Result);
end;

procedure TJSONFloatNumber.SetAsString(const AValue: TJSONStringType);

Var
  C : Integer;

begin
  Val(AValue,FValue,C);
  If (C<>0) then
    Raise EConvertError.CreateFmt(SErrInvalidFloat,[AValue]);
end;

function TJSONFloatNumber.GetValue: variant;
begin
  Result:=FValue;
end;

procedure TJSONFloatNumber.SetValue(const AValue: variant);
begin
  FValue:=AValue;
end;

constructor TJSONFloatNumber.Create(AValue: TJSONFloat);
begin
  FValue:=AValue;
end;

class function TJSONFloatNumber.NumberType: TJSONNumberType;
begin
  Result:=ntFloat;
end;

procedure TJSONFloatNumber.Clear;
>>>>>>> graemeg/fixes_2_2
begin
  FValue:=0;
end;

<<<<<<< HEAD
function TJSONFloatNumber.Clone: TJSONData;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TJSONFloatNumberClass(ClassType).Create(Self.FValue);
=======
  Result:=TJSONFloatNumber.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONFloatNumber.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONFloatNumber.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONFloatNumber.Create(Self.FValue);
>>>>>>> origin/cpstrnew
end;

=======
>>>>>>> graemeg/fixes_2_2
{ TJSONIntegerNumber }

function TJSONIntegerNumber.GetAsBoolean: Boolean;
begin
  Result:=FValue<>0;
end;

function TJSONIntegerNumber.GetAsFloat: TJSONFloat;
begin
  Result:=Ord(FValue);
end;

function TJSONIntegerNumber.GetAsInteger: Integer;
begin
  Result:=FValue;
end;

<<<<<<< HEAD
function TJSONIntegerNumber.GetAsInt64: Int64;
begin
  Result:=FValue;
end;

function TJSONIntegerNumber.GetAsQWord: QWord;
begin
  result:=FValue;
end;

=======
>>>>>>> graemeg/fixes_2_2
procedure TJSONIntegerNumber.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=Ord(AValue);
end;

procedure TJSONIntegerNumber.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=Round(AValue);
end;

procedure TJSONIntegerNumber.SetAsInteger(const AValue: Integer);
begin
  FValue:=AValue;
end;

<<<<<<< HEAD
procedure TJSONIntegerNumber.SetAsInt64(const AValue: Int64);
begin
  FValue:=AValue;
end;

procedure TJSONIntegerNumber.SetAsQword(const AValue: QWord);
begin
  FValue:=AValue;
end;

=======
>>>>>>> graemeg/fixes_2_2
function TJSONIntegerNumber.GetAsJSON: TJSONStringType;
begin
  Result:=AsString;
end;

<<<<<<< HEAD
function TJSONIntegerNumber.GetAsString: TJSONStringType;
begin
  Result:=IntToStr(FValue)
end;

procedure TJSONIntegerNumber.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=StrToInt(AValue);
end;


function TJSONIntegerNumber.GetValue: variant;
begin
  Result:=FValue;
end;

procedure TJSONIntegerNumber.SetValue(const AValue: variant);
begin
  FValue:=AValue;
end;

constructor TJSONIntegerNumber.Create(AValue: Integer);
begin
  FValue:=AValue;
end;

class function TJSONIntegerNumber.NumberType: TJSONNumberType;
begin
  Result:=ntInteger;
end;

procedure TJSONIntegerNumber.Clear;
begin
  FValue:=0;
end;

function TJSONIntegerNumber.Clone: TJSONData;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TJSONIntegerNumberClass(ClassType).Create(Self.FValue);
=======
  Result:=TJSONIntegerNumber.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONIntegerNumber.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONIntegerNumber.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONIntegerNumber.Create(Self.FValue);
>>>>>>> origin/cpstrnew
end;

{ TJSONInt64Number }

function TJSONInt64Number.GetAsInt64: Int64;
begin
  Result := FValue;
end;

function TJSONInt64Number.GetAsQWord: QWord;
begin
  Result := FValue;
end;

procedure TJSONInt64Number.SetAsInt64(const AValue: Int64);
begin
  FValue := AValue;
end;

procedure TJSONInt64Number.SetAsQword(const AValue: QWord);
begin
  FValue := AValue;
end;

function TJSONInt64Number.GetAsBoolean: Boolean;
begin
  Result:=FValue<>0;
end;

function TJSONInt64Number.GetAsFloat: TJSONFloat;
begin
  Result:= FValue;
end;

function TJSONInt64Number.GetAsInteger: Integer;
begin
  Result := FValue;
end;

procedure TJSONInt64Number.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=Ord(AValue);
end;

procedure TJSONInt64Number.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=Round(AValue);
end;

procedure TJSONInt64Number.SetAsInteger(const AValue: Integer);
begin
  FValue:=AValue;
end;

function TJSONInt64Number.GetAsJSON: TJSONStringType;
begin
  Result:=AsString;
end;

function TJSONInt64Number.GetAsString: TJSONStringType;
=======
function TJSONIntegerNumber.GetAsString: TJSONStringType;
>>>>>>> graemeg/fixes_2_2
begin
  Result:=IntToStr(FValue)
end;

<<<<<<< HEAD
procedure TJSONInt64Number.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=StrToInt64(AValue);
end;

function TJSONInt64Number.GetValue: variant;
=======
procedure TJSONIntegerNumber.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=StrToInt(AValue);
end;

function TJSONIntegerNumber.GetValue: variant;
>>>>>>> graemeg/fixes_2_2
begin
  Result:=FValue;
end;

<<<<<<< HEAD
procedure TJSONInt64Number.SetValue(const AValue: variant);
=======
procedure TJSONIntegerNumber.SetValue(const AValue: variant);
>>>>>>> graemeg/fixes_2_2
begin
  FValue:=AValue;
end;

<<<<<<< HEAD
constructor TJSONInt64Number.Create(AValue: Int64);
begin
  FValue := AValue;
end;

class function TJSONInt64Number.NumberType: TJSONNumberType;
begin
  Result:=ntInt64;
end;

procedure TJSONInt64Number.Clear;
=======
constructor TJSONIntegerNumber.Create(AValue: Integer);
begin
  FValue:=AValue;
end;

class function TJSONIntegerNumber.NumberType: TJSONNumberType;
begin
  Result:=ntInteger;
end;

procedure TJSONIntegerNumber.Clear;
>>>>>>> graemeg/fixes_2_2
begin
  FValue:=0;
end;

<<<<<<< HEAD
function TJSONInt64Number.Clone: TJSONData;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=TJSONInt64NumberClass(ClassType).Create(Self.FValue);
=======
  Result:=TJSONInt64Number.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONInt64Number.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONInt64Number.Create(Self.FValue);
>>>>>>> graemeg/cpstrnew
=======
  Result:=TJSONInt64Number.Create(Self.FValue);
>>>>>>> origin/cpstrnew
end;
=======
>>>>>>> graemeg/fixes_2_2

{ TJSONArray }

function TJSONArray.GetBooleans(Index : Integer): Boolean;
begin
  Result:=Items[Index].AsBoolean;
end;

function TJSONArray.GetArrays(Index : Integer): TJSONArray;
begin
  Result:=Items[Index] as TJSONArray;
end;

function TJSONArray.GetFloats(Index : Integer): TJSONFloat;
begin
  Result:=Items[Index].AsFloat;
end;

function TJSONArray.GetIntegers(Index : Integer): Integer;
begin
  Result:=Items[Index].AsInteger;
end;

<<<<<<< HEAD
function TJSONArray.GetInt64s(Index : Integer): Int64;
begin
  Result:=Items[Index].AsInt64;
end;

=======
>>>>>>> graemeg/fixes_2_2
function TJSONArray.GetNulls(Index : Integer): Boolean;
begin
  Result:=Items[Index].IsNull;
end;

function TJSONArray.GetObjects(Index : Integer): TJSONObject;
begin
  Result:=Items[Index] as TJSONObject;
end;

<<<<<<< HEAD
function TJSONArray.GetQWords(Index : Integer): QWord;
begin
  Result:=Items[Index].AsQWord;
end;

=======
>>>>>>> graemeg/fixes_2_2
function TJSONArray.GetStrings(Index : Integer): TJSONStringType;
begin
  Result:=Items[Index].AsString;
end;

<<<<<<< HEAD
function TJSONArray.GetUnicodeStrings(Index : Integer): TJSONUnicodeStringType;
begin
  Result:=Items[Index].AsUnicodeString;
end;

=======
>>>>>>> graemeg/fixes_2_2
function TJSONArray.GetTypes(Index : Integer): TJSONType;
begin
  Result:=Items[Index].JSONType;
end;

procedure TJSONArray.SetArrays(Index : Integer; const AValue: TJSONArray);
begin
  Items[Index]:=AValue;
end;

procedure TJSONArray.SetBooleans(Index : Integer; const AValue: Boolean);

begin
<<<<<<< HEAD
  Items[Index]:=CreateJSON(AValue);
=======
  Items[Index]:=TJSonBoolean.Create(AValue);
>>>>>>> graemeg/fixes_2_2
end;

procedure TJSONArray.SetFloats(Index : Integer; const AValue: TJSONFloat);
begin
<<<<<<< HEAD
  Items[Index]:=CreateJSON(AValue);
=======
  Items[Index]:=TJSONFloatNumber.Create(AValue);
>>>>>>> graemeg/fixes_2_2
end;

procedure TJSONArray.SetIntegers(Index : Integer; const AValue: Integer);
begin
<<<<<<< HEAD
  Items[Index]:=CreateJSON(AValue);
end;

procedure TJSONArray.SetInt64s(Index : Integer; const AValue: Int64);
begin
  Items[Index]:=CreateJSON(AValue);
=======
  Items[Index]:=TJSONIntegerNumber.Create(AValue);
>>>>>>> graemeg/fixes_2_2
end;

procedure TJSONArray.SetObjects(Index : Integer; const AValue: TJSONObject);
begin
  Items[Index]:=AValue;
end;

<<<<<<< HEAD
procedure TJSONArray.SetQWords(Index : Integer; AValue: QWord);
begin
  Items[Index]:=CreateJSON(AValue);
end;

procedure TJSONArray.SetStrings(Index : Integer; const AValue: TJSONStringType);
begin
  Items[Index]:=CreateJSON(AValue);
end;

procedure TJSONArray.SetUnicodeStrings(Index: Integer;
  const AValue: TJSONUnicodeStringType);
begin
  Items[Index]:=CreateJSON(AValue);
end;

function TJSONArray.DoFindPath(const APath: TJSONStringType; out
  NotFound: TJSONStringType): TJSONdata;

Var
  P,I : integer;
  E : String;

begin
  if (APath<>'') and (APath[1]='[') then
    begin
    P:=Pos(']',APath);
    I:=-1;
    If (P>2) then
      I:=StrToIntDef(Copy(APath,2,P-2),-1);
    If (I>=0) and (I<Count) then
       begin
       E:=APath;
       System.Delete(E,1,P);
       Result:=Items[i].DoFindPath(E,NotFound);
       end
    else
       begin
       Result:=Nil;
       If (P>0) then
         NotFound:=Copy(APath,1,P)
       else
         NotFound:=APath;
       end;
    end
  else
    Result:=inherited DoFindPath(APath, NotFound);
=======
procedure TJSONArray.SetStrings(Index : Integer; const AValue: TJSONStringType);
begin
  Items[Index]:=TJSONString.Create(AValue);
>>>>>>> graemeg/fixes_2_2
end;

procedure TJSONArray.Converterror(From: Boolean);
begin
  If From then
<<<<<<< HEAD
    DoError(SErrCannotConvertFromArray)
  else
    DoError(SErrCannotConvertToArray);
end;

{$warnings off}
=======
    Raise EJSON.Create(SErrCannotConvertFromArray)
  else
    Raise EJSON.Create(SErrCannotConvertToArray);
end;

>>>>>>> graemeg/fixes_2_2
function TJSONArray.GetAsBoolean: Boolean;
begin
  ConvertError(True);
end;

function TJSONArray.GetAsFloat: TJSONFloat;
begin
  ConvertError(True);
end;

function TJSONArray.GetAsInteger: Integer;
begin
  ConvertError(True);
end;

<<<<<<< HEAD
function TJSONArray.GetAsInt64: Int64;
begin
  ConvertError(True);
end;

function TJSONArray.GetAsQWord: QWord;
begin
  ConvertError(True);
end;

=======
>>>>>>> graemeg/fixes_2_2
procedure TJSONArray.SetAsBoolean(const AValue: Boolean);
begin
  ConvertError(False);
end;

procedure TJSONArray.SetAsFloat(const AValue: TJSONFloat);
begin
  ConvertError(False);
end;

procedure TJSONArray.SetAsInteger(const AValue: Integer);
begin
  ConvertError(False);
end;

<<<<<<< HEAD
procedure TJSONArray.SetAsInt64(const AValue: Int64);
begin
  ConvertError(False);
end;

procedure TJSONArray.SetAsQword(const AValue: QWord);
begin
  ConvertError(False);
end;

{$warnings on}


=======
>>>>>>> graemeg/fixes_2_2
function TJSONArray.GetAsJSON: TJSONStringType;

Var
  I : Integer;
<<<<<<< HEAD
  Sep : String;

begin
  Sep:=TJSONData.FElementSep;
=======

begin
>>>>>>> graemeg/fixes_2_2
  Result:='[';
  For I:=0 to Count-1 do
    begin
    Result:=Result+Items[i].AsJSON;
    If (I<Count-1) then
<<<<<<< HEAD
      Result:=Result+Sep;
    end;
  Result:=Result+']';
end;

{$warnings off}

Function IndentString(Options : TFormatOptions; Indent : Integer) : TJSONStringType;

begin
  If (foUseTabChar in Options) then
    Result:=StringofChar(#9,Indent)
  else
    Result:=StringOfChar(' ',Indent);  
end;

<<<<<<< HEAD
function TJSONArray.DoFormatJSON(Options: TFormatOptions; CurrentIndent,
  Indent: Integer): TJSONStringType;

Var
  I : Integer;
  MultiLine : Boolean;
  SkipWhiteSpace : Boolean;
  Ind : String;
  
begin
  Result:='[';
  MultiLine:=Not (foSingleLineArray in Options);
  SkipWhiteSpace:=foSkipWhiteSpace in Options;
  Ind:=IndentString(Options, CurrentIndent+Indent);
  if MultiLine then
    Result:=Result+sLineBreak;
  For I:=0 to Count-1 do
    begin
    if MultiLine then
      Result:=Result+Ind;
    Result:=Result+Items[i].DoFormatJSON(Options,CurrentIndent+Indent,Indent);
    If (I<Count-1) then
      if MultiLine then
        Result:=Result+','
      else
        Result:=Result+ElementSeps[SkipWhiteSpace];
    if MultiLine then
      Result:=Result+sLineBreak
    end;
  if MultiLine then
=======
Function TJSONArray.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 

Var
  I : Integer;
  
begin
  Result:='[';
  if not (foSingleLineArray in Options) then
    Result:=Result+sLineBreak;
  For I:=0 to Count-1 do
    begin
    if not (foSingleLineArray in Options) then
      Result:=Result+IndentString(Options, CurrentIndent+Indent);
    Result:=Result+Items[i].DoFormatJSON(Options,CurrentIndent+Indent,Indent);
    If (I<Count-1) then
      if (foSingleLineArray in Options) then
        Result:=Result+', '
      else
        Result:=Result+',';
    if not (foSingleLineArray in Options) then
      Result:=Result+sLineBreak
    end;
 if not (foSingleLineArray in Options) then
>>>>>>> graemeg/cpstrnew
    Result:=Result+IndentString(Options, CurrentIndent);
  Result:=Result+']';
end;

<<<<<<< HEAD
=======
{$warnings off}

Function IndentString(Options : TFormatOptions; Indent : Integer) : TJSONStringType;

begin
  If (foUseTabChar in Options) then
    Result:=StringofChar(#9,Indent)
  else
    Result:=StringOfChar(' ',Indent);  
end;

Function TJSONArray.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 

Var
  I : Integer;
  
begin
  Result:='[';
  if not (foSingleLineArray in Options) then
    Result:=Result+sLineBreak;
  For I:=0 to Count-1 do
    begin
    if not (foSingleLineArray in Options) then
      Result:=Result+IndentString(Options, CurrentIndent+Indent);
    Result:=Result+Items[i].DoFormatJSON(Options,CurrentIndent+Indent,Indent);
    If (I<Count-1) then
      if (foSingleLineArray in Options) then
        Result:=Result+', '
      else
        Result:=Result+',';
    if not (foSingleLineArray in Options) then
      Result:=Result+sLineBreak
    end;
 if not (foSingleLineArray in Options) then
    Result:=Result+IndentString(Options, CurrentIndent);
  Result:=Result+']';
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$warnings off}

Function IndentString(Options : TFormatOptions; Indent : Integer) : TJSONStringType;

begin
  If (foUseTabChar in Options) then
    Result:=StringofChar(#9,Indent)
  else
    Result:=StringOfChar(' ',Indent);  
end;

Function TJSONArray.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 

Var
  I : Integer;
  
begin
  Result:='[';
  if not (foSingleLineArray in Options) then
    Result:=Result+sLineBreak;
  For I:=0 to Count-1 do
    begin
    if not (foSingleLineArray in Options) then
      Result:=Result+IndentString(Options, CurrentIndent+Indent);
    Result:=Result+Items[i].DoFormatJSON(Options,CurrentIndent+Indent,Indent);
    If (I<Count-1) then
      if (foSingleLineArray in Options) then
        Result:=Result+', '
      else
        Result:=Result+',';
    if not (foSingleLineArray in Options) then
      Result:=Result+sLineBreak
    end;
 if not (foSingleLineArray in Options) then
    Result:=Result+IndentString(Options, CurrentIndent);
  Result:=Result+']';
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
{$warnings off}

Function IndentString(Options : TFormatOptions; Indent : Integer) : TJSONStringType;

begin
  If (foUseTabChar in Options) then
    Result:=StringofChar(#9,Indent)
  else
    Result:=StringOfChar(' ',Indent);  
end;

Function TJSONArray.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 

Var
  I : Integer;
  
begin
  Result:='[';
  if not (foSingleLineArray in Options) then
    Result:=Result+sLineBreak;
  For I:=0 to Count-1 do
    begin
    if not (foSingleLineArray in Options) then
      Result:=Result+IndentString(Options, CurrentIndent+Indent);
    Result:=Result+Items[i].DoFormatJSON(Options,CurrentIndent+Indent,Indent);
    If (I<Count-1) then
      if (foSingleLineArray in Options) then
        Result:=Result+', '
      else
        Result:=Result+',';
    if not (foSingleLineArray in Options) then
      Result:=Result+sLineBreak
    end;
 if not (foSingleLineArray in Options) then
    Result:=Result+IndentString(Options, CurrentIndent);
  Result:=Result+']';
end;

>>>>>>> origin/cpstrnew

=======
=======


{ TJSONData }


function TJSONData.GetItem(Index : Integer): TJSONData;
begin
  Result:=nil;
end;

function TJSONData.GetCount: Integer;
begin
  Result:=0;
end;

constructor TJSONData.Create;
begin
  Clear;
end;

function TJSONData.GetIsNull: Boolean;
begin
  Result:=False;
end;

class function TJSONData.JSONType: TJSONType;
begin
  JSONType:=jtUnknown;
end;

procedure TJSONData.SetItem(Index : Integer; const AValue:
  TJSONData);
begin
  // Do Nothing
end;

{ TJSONnumber }

class function TJSONnumber.JSONType: TJSONType;
begin
  Result:=jtNumber;
end;


{ TJSONstring }

class function TJSONstring.JSONType: TJSONType;
begin
  Result:=jtString;
end;

procedure TJSONString.Clear;
begin
  FValue:='';
end;

function TJSONstring.GetValue: Variant;
begin
  Result:=FValue;
end;

procedure TJSONstring.SetValue(const AValue: Variant);
begin
  FValue:=AValue;
end;


function TJSONstring.GetAsBoolean: Boolean;
begin
  Result:=StrToBool(FValue);
end;

function TJSONstring.GetAsFloat: TJSONFloat;

Var
  C : Integer;

begin
  Val(FValue,Result,C);
  If (C<>0) then
    If Not TryStrToFloat(FValue,Result) then
      Raise EConvertError.CreateFmt(SErrInvalidFloat,[FValue]);
end;

function TJSONstring.GetAsInteger: Integer;
begin
  Result:=StrToInt(FValue);
end;

procedure TJSONstring.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=BoolToStr(AValue);
end;

procedure TJSONstring.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=FloatToStr(AValue);
end;

procedure TJSONstring.SetAsInteger(const AValue: Integer);
begin
  FValue:=IntToStr(AValue);
end;

function TJSONstring.GetAsJSON: TJSONStringType;
begin
  Result:='"'+StringToJSONString(FValue)+'"';
end;

function TJSONstring.GetAsString: TJSONStringType;
begin
  Result:=FValue;
end;

procedure TJSONstring.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=AValue;
end;

constructor TJSONstring.Create(AValue: TJSONStringType);
begin
  FValue:=AValue;
end;

{ TJSONboolean }


function TJSONboolean.GetValue: Variant;
begin
  Result:=FValue;
end;

class function TJSONboolean.JSONType: TJSONType;
begin
  Result:=jtBoolean;
end;

procedure TJSONBoolean.Clear;
begin
  FValue:=False;
end;


procedure TJSONboolean.SetValue(const AValue: Variant);
begin
  FValue:=boolean(AValue);
end;

function TJSONboolean.GetAsBoolean: Boolean;
begin
  Result:=FValue;
end;

function TJSONboolean.GetAsFloat: TJSONFloat;
begin
  Result:=Ord(FValue);
end;

function TJSONboolean.GetAsInteger: Integer;
begin
  Result:=Ord(FValue);
end;


procedure TJSONboolean.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=AValue;
end;

procedure TJSONboolean.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=(AValue<>0)
end;

procedure TJSONboolean.SetAsInteger(const AValue: Integer);
begin
  FValue:=(AValue<>0)
end;

function TJSONboolean.GetAsJSON: TJSONStringType;
begin
  If FValue then
    Result:='True'
  else
    Result:='False';
end;

function TJSONboolean.GetAsString: TJSONStringType;
begin
  Result:=BoolToStr(FValue);
end;

procedure TJSONboolean.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=StrToBool(AValue);
end;

constructor TJSONboolean.Create(AValue: Boolean);
begin
  FValue:=AValue;
end;

{ TJSONnull }

procedure TJSONnull.Converterror(From : Boolean);
begin
  If From then
    Raise EJSON.Create(SErrCannotConvertFromNull)
  else
    Raise EJSON.Create(SErrCannotConvertToNull);
end;

function TJSONnull.GetAsBoolean: Boolean;
begin
  ConvertError(True);
end;

function TJSONnull.GetAsFloat: TJSONFloat;
begin
  ConvertError(True);
end;

function TJSONnull.GetAsInteger: Integer;
begin
  ConvertError(True);
end;

function TJSONnull.GetIsNull: Boolean;
begin
  Result:=True;
end;

procedure TJSONnull.SetAsBoolean(const AValue: Boolean);
begin
  ConvertError(False);
end;

procedure TJSONnull.SetAsFloat(const AValue: TJSONFloat);
begin
  ConvertError(False);
end;

procedure TJSONnull.SetAsInteger(const AValue: Integer);
begin
  ConvertError(False);
end;

function TJSONnull.GetAsJSON: TJSONStringType;
begin
  Result:='Null';
end;

function TJSONnull.GetAsString: TJSONStringType;
begin
  ConvertError(True);
end;

procedure TJSONnull.SetAsString(const AValue: TJSONStringType);
begin
  ConvertError(True);
end;

function TJSONnull.GetValue: Variant;
begin
  Result:=variants.Null;
end;

procedure TJSONnull.SetValue(const AValue: variant);
begin
  ConvertError(False);
end;

class function TJSONnull.JSONType: TJSONType;
begin
  Result:=jtNull;
end;

procedure TJSONNull.Clear;
begin
  // Do nothing
end;



{ TJSONFloatNumber }

function TJSONFloatNumber.GetAsBoolean: Boolean;
begin
  Result:=(FValue<>0);
end;

function TJSONFloatNumber.GetAsFloat: TJSONFloat;
begin
  Result:=FValue;
end;

function TJSONFloatNumber.GetAsInteger: Integer;
begin
  Result:=Round(FValue);
end;

procedure TJSONFloatNumber.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=Ord(AValue);
end;

procedure TJSONFloatNumber.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=AValue;
end;

procedure TJSONFloatNumber.SetAsInteger(const AValue: Integer);
begin
  FValue:=AValue;
end;

function TJSONFloatNumber.GetAsJSON: TJSONStringType;
begin
  Result:=AsString;
end;

function TJSONFloatNumber.GetAsString: TJSONStringType;
begin
  Str(FValue,Result);
end;

procedure TJSONFloatNumber.SetAsString(const AValue: TJSONStringType);

Var
  C : Integer;

begin
  Val(AValue,FValue,C);
  If (C<>0) then
    Raise EConvertError.CreateFmt(SErrInvalidFloat,[AValue]);
end;

function TJSONFloatNumber.GetValue: variant;
begin
  Result:=FValue;
end;

procedure TJSONFloatNumber.SetValue(const AValue: variant);
begin
  FValue:=AValue;
end;

constructor TJSONFloatNumber.Create(AValue: TJSONFloat);
begin
  FValue:=AValue;
end;

class function TJSONFloatNumber.NumberType: TJSONNumberType;
begin
  Result:=ntFloat;
end;

procedure TJSONFloatNumber.Clear;
begin
  FValue:=0;
end;

{ TJSONIntegerNumber }

function TJSONIntegerNumber.GetAsBoolean: Boolean;
begin
  Result:=FValue<>0;
end;

function TJSONIntegerNumber.GetAsFloat: TJSONFloat;
begin
  Result:=Ord(FValue);
end;

function TJSONIntegerNumber.GetAsInteger: Integer;
begin
  Result:=FValue;
end;

procedure TJSONIntegerNumber.SetAsBoolean(const AValue: Boolean);
begin
  FValue:=Ord(AValue);
end;

procedure TJSONIntegerNumber.SetAsFloat(const AValue: TJSONFloat);
begin
  FValue:=Round(AValue);
end;

procedure TJSONIntegerNumber.SetAsInteger(const AValue: Integer);
begin
  FValue:=AValue;
end;

function TJSONIntegerNumber.GetAsJSON: TJSONStringType;
begin
  Result:=AsString;
end;

function TJSONIntegerNumber.GetAsString: TJSONStringType;
begin
  Result:=IntToStr(FValue)
end;

procedure TJSONIntegerNumber.SetAsString(const AValue: TJSONStringType);
begin
  FValue:=StrToInt(AValue);
end;

function TJSONIntegerNumber.GetValue: variant;
begin
  Result:=FValue;
end;

procedure TJSONIntegerNumber.SetValue(const AValue: variant);
begin
  FValue:=AValue;
end;

constructor TJSONIntegerNumber.Create(AValue: Integer);
begin
  FValue:=AValue;
end;

class function TJSONIntegerNumber.NumberType: TJSONNumberType;
begin
  Result:=ntInteger;
end;

procedure TJSONIntegerNumber.Clear;
begin
  FValue:=0;
end;


{ TJSONArray }

function TJSONArray.GetBooleans(Index : Integer): Boolean;
begin
  Result:=Items[Index].AsBoolean;
end;

function TJSONArray.GetArrays(Index : Integer): TJSONArray;
begin
  Result:=Items[Index] as TJSONArray;
end;

function TJSONArray.GetFloats(Index : Integer): TJSONFloat;
begin
  Result:=Items[Index].AsFloat;
end;

function TJSONArray.GetIntegers(Index : Integer): Integer;
begin
  Result:=Items[Index].AsInteger;
end;

function TJSONArray.GetNulls(Index : Integer): Boolean;
begin
  Result:=Items[Index].IsNull;
end;

function TJSONArray.GetObjects(Index : Integer): TJSONObject;
begin
  Result:=Items[Index] as TJSONObject;
end;

function TJSONArray.GetStrings(Index : Integer): TJSONStringType;
begin
  Result:=Items[Index].AsString;
end;

function TJSONArray.GetTypes(Index : Integer): TJSONType;
begin
  Result:=Items[Index].JSONType;
end;

procedure TJSONArray.SetArrays(Index : Integer; const AValue: TJSONArray);
begin
  Items[Index]:=AValue;
end;

procedure TJSONArray.SetBooleans(Index : Integer; const AValue: Boolean);

begin
  Items[Index]:=TJSonBoolean.Create(AValue);
end;

procedure TJSONArray.SetFloats(Index : Integer; const AValue: TJSONFloat);
begin
  Items[Index]:=TJSONFloatNumber.Create(AValue);
end;

procedure TJSONArray.SetIntegers(Index : Integer; const AValue: Integer);
begin
  Items[Index]:=TJSONIntegerNumber.Create(AValue);
end;

procedure TJSONArray.SetObjects(Index : Integer; const AValue: TJSONObject);
begin
  Items[Index]:=AValue;
end;

procedure TJSONArray.SetStrings(Index : Integer; const AValue: TJSONStringType);
begin
  Items[Index]:=TJSONString.Create(AValue);
end;

procedure TJSONArray.Converterror(From: Boolean);
begin
  If From then
    Raise EJSON.Create(SErrCannotConvertFromArray)
  else
    Raise EJSON.Create(SErrCannotConvertToArray);
end;

function TJSONArray.GetAsBoolean: Boolean;
begin
  ConvertError(True);
end;

function TJSONArray.GetAsFloat: TJSONFloat;
begin
  ConvertError(True);
end;

function TJSONArray.GetAsInteger: Integer;
begin
  ConvertError(True);
end;

procedure TJSONArray.SetAsBoolean(const AValue: Boolean);
begin
  ConvertError(False);
end;

procedure TJSONArray.SetAsFloat(const AValue: TJSONFloat);
begin
  ConvertError(False);
end;

procedure TJSONArray.SetAsInteger(const AValue: Integer);
begin
  ConvertError(False);
end;

function TJSONArray.GetAsJSON: TJSONStringType;

Var
  I : Integer;

begin
  Result:='[';
  For I:=0 to Count-1 do
    begin
    Result:=Result+Items[i].AsJSON;
    If (I<Count-1) then
>>>>>>> origin/fixes_2_2
      Result:=Result+', '
    end;
  Result:=Result+']';
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TJSONArray.GetAsString: TJSONStringType;
begin
  ConvertError(True);
end;

procedure TJSONArray.SetAsString(const AValue: TJSONStringType);
begin
  ConvertError(False);
end;

function TJSONArray.GetValue: variant;
begin
  ConvertError(True);
end;

procedure TJSONArray.SetValue(const AValue: variant);
begin
<<<<<<< HEAD
  ConvertError(False);
end;
<<<<<<< HEAD
{$warnings on}
=======
>>>>>>> graemeg/fixes_2_2

function TJSONArray.GetCount: Integer;
begin
  Result:=Flist.Count;
end;

function TJSONArray.GetItem(Index: Integer): TJSONData;
begin
  Result:=FList[Index] as TJSONData;
end;

procedure TJSONArray.SetItem(Index: Integer; const AValue: TJSONData);
begin
  If (Index=FList.Count) then
    FList.Add(AValue)
  else
    FList[Index]:=AValue;
end;

constructor TJSONArray.Create;
begin
  Flist:=TFPObjectList.Create(True);
end;

<<<<<<< HEAD
Function VarRecToJSON(Const Element : TVarRec; const SourceType : String) : TJSONData;
=======
Function VarRecToJSON(Const Element : TVarRec; SourceType : String) : TJSONData;
>>>>>>> graemeg/fixes_2_2

begin
  Result:=Nil;
  With Element do
    case VType of
<<<<<<< HEAD
      vtInteger    : Result:=CreateJSON(VInteger);
      vtBoolean    : Result:=CreateJSON(VBoolean);
      vtChar       : Result:=CreateJSON(VChar);
      vtExtended   : Result:=CreateJSON(VExtended^);
      vtString     : Result:=CreateJSON(vString^);
      vtAnsiString : Result:=CreateJSON(AnsiString(vAnsiString));
      vtPChar      : Result:=CreateJSON(StrPas(VPChar));
      vtPointer    : If (VPointer<>Nil) then
                       TJSONData.DoError(SErrPointerNotNil,[SourceType])
                     else
                       Result:=CreateJSON();
      vtCurrency   : Result:=CreateJSON(vCurrency^);
      vtInt64      : Result:=CreateJSON(vInt64^);
      vtObject     : if (VObject is TJSONData) then
                       Result:=TJSONData(VObject)
                     else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                       TJSONData.DoError(SErrNotJSONData,[VObject.ClassName,SourceType]);
=======
                       Raise EJSON.CreateFmt(SErrNotJSONData,[VObject.ClassName,SourceType]);
>>>>>>> graemeg/cpstrnew
=======
                       Raise EJSON.CreateFmt(SErrNotJSONData,[VObject.ClassName,SourceType]);
>>>>>>> graemeg/cpstrnew
=======
                       Raise EJSON.CreateFmt(SErrNotJSONData,[VObject.ClassName,SourceType]);
>>>>>>> graemeg/cpstrnew
=======
                       Raise EJSON.CreateFmt(SErrNotJSONData,[VObject.ClassName,SourceType]);
>>>>>>> origin/cpstrnew
      //vtVariant    :
    else
      TJSONData.DoError(SErrUnknownTypeInConstructor,[SourceType,VType])
    end;
end;

constructor TJSONArray.Create(const Elements: array of const);

Var
  I : integer;
  J : TJSONData;

begin
  Create;
  For I:=Low(Elements) to High(Elements) do
    begin
    J:=VarRecToJSON(Elements[i],'Array');
    Add(J);
    end;
end;

destructor TJSONArray.Destroy;
begin
  FreeAndNil(FList);
  inherited Destroy;
end;

class function TJSONArray.JSONType: TJSONType;
begin
  Result:=jtArray;
end;

function TJSONArray.Clone: TJSONData;

Var
  A : TJSONArray;
  I : Integer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  A:=TJSONArrayClass(ClassType).Create;
=======
  A:=TJSONArray.Create;
>>>>>>> graemeg/cpstrnew
=======
  A:=TJSONArray.Create;
>>>>>>> graemeg/cpstrnew
=======
  A:=TJSONArray.Create;
>>>>>>> graemeg/cpstrnew
=======
  A:=TJSONArray.Create;
>>>>>>> origin/cpstrnew
  try
    For I:=0 to Count-1 do
      A.Add(Self.Items[I].Clone);
    Result:=A;
  except
    A.Free;
    Raise;
  end;
end;

procedure TJSONArray.Iterate(Iterator: TJSONArrayIterator; Data: TObject);

Var
  I : Integer;
  Cont : Boolean;
  
begin
  I:=0;
  Cont:=True;
  While (I<FList.Count) and cont do
    begin
    Iterator(Items[i],Data,Cont);
    Inc(I);
    end;
end;

function TJSONArray.IndexOf(obj: TJSONData): Integer;
begin
  Result:=FList.IndexOf(Obj);
end;

function TJSONArray.GetEnumerator: TBaseJSONEnumerator;
begin
  Result:=TJSONArrayEnumerator.Create(Self);
end;

procedure TJSONArray.Clear;
begin
  FList.Clear;
end;

function TJSONArray.Add(Item: TJSONData): Integer;
begin
  Result:=FList.Add(Item);
end;

function TJSONArray.Add(I: Integer): Integer;
begin
  Result:=Add(CreateJSON(I));
end;

function TJSONArray.Add(I: Int64): Int64;
begin
  Result:=Add(CreateJSON(I));
end;

function TJSONArray.Add(I: QWord): QWord;
begin
  Result:=Add(CreateJSON(I));
end;

function TJSONArray.Add(const S: String): Integer;
begin
  Result:=Add(CreateJSON(S));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONArray.Add(const S: UnicodeString): Integer;
=======
function TJSONArray.Add(const S: String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function TJSONArray.Add(const S: String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function TJSONArray.Add(const S: String): Integer;
>>>>>>> graemeg/cpstrnew
begin
  Result:=Add(CreateJSON(S));
end;

function TJSONArray.Add: Integer;
begin
  Result:=Add(CreateJSON);
end;

function TJSONArray.Add(F: TJSONFloat): Integer;
begin
  Result:=Add(CreateJSON(F));
end;

function TJSONArray.Add(B: Boolean): Integer;
begin
  Result:=Add(CreateJSON(B));
end;

function TJSONArray.Add(AnArray: TJSONArray): Integer;
begin
  If (IndexOf(AnArray)<>-1) then
    DoError(SErrCannotAddArrayTwice);
  Result:=Add(TJSONData(AnArray));
end;

function TJSONArray.Add(AnObject: TJSONObject): Integer;
begin
  If (IndexOf(AnObject)<>-1) then
    DoError(SErrCannotAddObjectTwice);
  Result:=Add(TJSONData(AnObject));
end;

procedure TJSONArray.Delete(Index: Integer);
begin
  FList.Delete(Index);
end;

procedure TJSONArray.Exchange(Index1, Index2: Integer);
begin
  FList.Exchange(Index1, Index2);
end;

function TJSONArray.Extract(Item: TJSONData): TJSONData;
begin
  Result := TJSONData(FList.Extract(Item));
end;

function TJSONArray.Extract(Index: Integer): TJSONData;
begin
  Result := TJSONData(FList.Extract(FList.Items[Index]));
end;

procedure TJSONArray.Insert(Index: Integer);
begin
  Insert(Index,CreateJSON);
end;

procedure TJSONArray.Insert(Index: Integer; Item: TJSONData);
begin
  FList.Insert(Index, Item);
end;

procedure TJSONArray.Insert(Index: Integer; I: Integer);
begin
  FList.Insert(Index, CreateJSON(I));
end;

procedure TJSONArray.Insert(Index: Integer; I: Int64);
begin
  FList.Insert(Index, CreateJSON(I));
end;

procedure TJSONArray.Insert(Index: Integer; I: QWord);
begin
  FList.Insert(Index, CreateJSON(I));
end;

procedure TJSONArray.Insert(Index: Integer; const S: String);
begin
  FList.Insert(Index, CreateJSON(S));
end;

procedure TJSONArray.Insert(Index: Integer; const S: UnicodeString);
begin
  FList.Insert(Index, CreateJSON(S));
end;

procedure TJSONArray.Insert(Index: Integer; F: TJSONFloat);
begin
  FList.Insert(Index, CreateJSON(F));
end;

procedure TJSONArray.Insert(Index: Integer; B: Boolean);
begin
  FList.Insert(Index, CreateJSON(B));
end;

procedure TJSONArray.Insert(Index: Integer; AnArray: TJSONArray);
begin
  if (IndexOf(AnArray)<>-1) then
    DoError(SErrCannotAddArrayTwice);
  FList.Insert(Index, AnArray);
end;

procedure TJSONArray.Insert(Index: Integer; AnObject: TJSONObject);
begin
  if (IndexOf(AnObject)<>-1) then
    DoError(SErrCannotAddObjectTwice);
  FList.Insert(Index, AnObject);
end;

procedure TJSONArray.Move(CurIndex, NewIndex: Integer);
begin
  FList.Move(CurIndex, NewIndex);
=======
=======
  ConvertError(False);
end;

function TJSONArray.GetCount: Integer;
begin
  Result:=Flist.Count;
end;

function TJSONArray.GetItem(Index: Integer): TJSONData;
begin
  Result:=FList[Index] as TJSONData;
end;

procedure TJSONArray.SetItem(Index: Integer; const AValue: TJSONData);
begin
  If (Index=FList.Count) then
    FList.Add(AValue)
  else
    FList[Index]:=AValue;
end;

constructor TJSONArray.Create;
begin
  Flist:=TFPObjectList.Create(True);
end;

Function VarRecToJSON(Const Element : TVarRec; SourceType : String) : TJSONData;

begin
  Result:=Nil;
  With Element do
    case VType of
>>>>>>> origin/fixes_2_2
      vtInteger    : Result:=TJSONIntegerNumber.Create(VInteger);
      vtBoolean    : Result:=TJSONBoolean.Create(VBoolean);
      vtChar       : Result:=TJSONString.Create(VChar);
      vtExtended   : Result:=TJSONFloatNumber.Create(VExtended^);
      vtString     : Result:=TJSONString.Create(vString^);
      vtAnsiString : Result:=TJSONString.Create(AnsiString(vAnsiString));
      vtPChar      : Result:=TJSONString.Create(StrPas(VPChar));
      vtPointer    : If (VPointer<>Nil) then
                       Raise EJSON.CreateFmt(SErrPointerNotNil,[SourceType])
                     else
                       Result:=TJSONNull.Create;
      vtCurrency   : Result:=TJSONFloatNumber.Create(vCurrency^);
      vtInt64      : Result:=TJSONFloatNumber.Create(vInt64^);
      vtObject     : if (VObject is TJSONData) then
                       Result:=TJSONData(VObject)
                     else
                       Raise EJSON.CreateFmt(SErrNotJSONData,[SourceType,VObject.ClassName]);
      //vtVariant    :
    else
      Raise EJSON.CreateFmt(SErrUnknownTypeInConstructor,[SourceType,VType])
    end;
end;

constructor TJSONArray.Create(Const Elements: array of const);

Var
  I : integer;
  J : TJSONData;

begin
  Create;
  For I:=Low(Elements) to High(Elements) do
    begin
    J:=VarRecToJSON(Elements[i],'Array');
    Add(J);
    end;
end;

Destructor TJSONArray.Destroy;
begin
  FreeAndNil(FList);
  inherited Destroy;
end;

class function TJSONArray.JSONType: TJSONType;
begin
  Result:=jtArray;
end;

procedure TJSONArray.Iterate(Iterator: TJSONArrayIterator; Data: TObject);

Var
  I : Integer;
  Cont : Boolean;
  
begin
  I:=0;
  Cont:=True;
  While (I<FList.Count) and cont do
    begin
    Iterator(Items[i],Data,Cont);
    Inc(I);
    end;
end;

function TJSONArray.IndexOf(obj: TJSONData): Integer;
begin
  Result:=FList.IndexOf(Obj);
end;

procedure TJSONArray.Clear;
begin
  FList.Clear;
end;

function TJSONArray.Add(Item: TJSONData): Integer;
begin
  Result:=FList.Add(Item);
end;

function TJSONArray.Add(I: Integer): Integer;
begin
  Result:=Add(TJSONIntegerNumber.Create(I));
end;

function TJSONArray.Add(S: String): Integer;
begin
  Result:=Add(TJSONString.Create(S));
end;

function TJSONArray.Add: Integer;
begin
  Result:=Add(TJSONNull.Create);
end;

function TJSONArray.Add(F: TJSONFloat): Integer;
begin
  Result:=Add(TJSONFloatNumber.Create(F));
end;

function TJSONArray.Add(B: Boolean): Integer;
begin
  Result:=Add(TJSONBoolean.Create(B));
end;

function TJSONArray.Add(AnArray: TJSONArray): Integer;
begin
  If (IndexOf(AnArray)<>-1) then
    Raise EJSON.Create(SErrCannotAddArrayTwice);
  Result:=Add(TJSONData(AnArray));
end;

function TJSONArray.Add(AnObject: TJSONObject): Integer;
begin
  If (IndexOf(AnObject)<>-1) then
    Raise EJSON.Create(SErrCannotAddObjectTwice);
  Result:=Add(TJSONData(AnObject));
end;

procedure TJSONArray.Delete(Index: Integer);
begin
  FList.Delete(Index);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TJSONArray.Remove(Item: TJSONData);
begin
  FList.Remove(Item);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONArray.Sort(Compare: TListSortCompare);
begin
  FList.Sort(Compare);
end;

{ TJSONObject }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetArrays(const AName: String): TJSONArray;
=======
function TJSONObject.GetArrays(Const AName : String): TJSONArray;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetArrays(Const AName : String): TJSONArray;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetArrays(Const AName : String): TJSONArray;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetArrays(Const AName : String): TJSONArray;
>>>>>>> origin/cpstrnew
=======
{ TJSONObject }

function TJSONObject.GetArrays(AName : String): TJSONArray;
>>>>>>> graemeg/fixes_2_2
=======
{ TJSONObject }

function TJSONObject.GetArrays(AName : String): TJSONArray;
>>>>>>> origin/fixes_2_2
begin
  Result:=GetElements(AName) as TJSONArray;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetBooleans(const AName: String): Boolean;
=======
function TJSONObject.GetBooleans(Const AName : String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetBooleans(Const AName : String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetBooleans(Const AName : String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetBooleans(Const AName : String): Boolean;
>>>>>>> origin/cpstrnew
=======
function TJSONObject.GetBooleans(AName : String): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONObject.GetBooleans(AName : String): Boolean;
>>>>>>> origin/fixes_2_2
begin
  Result:=GetElements(AName).AsBoolean;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetElements(const AName: string): TJSONData;
begin
  Result:=TJSONData(FHash.Find(AName));
  If (Result=Nil) then
    DoError(SErrNonexistentElement,[AName]);
end;

function TJSONObject.GetFloats(const AName: String): TJSONFloat;
=======
=======
>>>>>>> graemeg/cpstrnew
function TJSONObject.GetElements(Const AName: string): TJSONData;
begin
  Result:=TJSONData(FHash.Find(AName));
  If (Result=Nil) then
    Raise EJSON.CreateFmt(SErrNonexistentElement,[AName]);
end;

function TJSONObject.GetFloats(Const AName : String): TJSONFloat;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
function TJSONObject.GetElements(Const AName: string): TJSONData;
begin
  Result:=TJSONData(FHash.Find(AName));
  If (Result=Nil) then
    Raise EJSON.CreateFmt(SErrNonexistentElement,[AName]);
end;

function TJSONObject.GetFloats(Const AName : String): TJSONFloat;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
function TJSONObject.GetElements(AName: string): TJSONData;
begin
  Result:=TJSONData(FHash.Find(AName));
end;

function TJSONObject.GetFloats(AName : String): TJSONFloat;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Result:=GetElements(AName).AsFloat;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetIntegers(const AName: String): Integer;
=======
function TJSONObject.GetIntegers(Const AName : String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetIntegers(Const AName : String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetIntegers(Const AName : String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetIntegers(Const AName : String): Integer;
>>>>>>> origin/cpstrnew
=======
function TJSONObject.GetIntegers(AName : String): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONObject.GetIntegers(AName : String): Integer;
>>>>>>> origin/fixes_2_2
begin
  Result:=GetElements(AName).AsInteger;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetInt64s(const AName: String): Int64;
=======
function TJSONObject.GetInt64s(Const AName : String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetInt64s(Const AName : String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetInt64s(Const AName : String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetInt64s(Const AName : String): Int64;
>>>>>>> origin/cpstrnew
begin
  Result:=GetElements(AName).AsInt64;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetIsNull(const AName: String): Boolean;
=======
function TJSONObject.GetIsNull(Const AName : String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetIsNull(Const AName : String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetIsNull(Const AName : String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TJSONObject.GetIsNull(Const AName : String): Boolean;
>>>>>>> origin/cpstrnew
=======
function TJSONObject.GetIsNull(AName : String): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONObject.GetIsNull(AName : String): Boolean;
>>>>>>> origin/fixes_2_2
begin
  Result:=GetElements(AName).IsNull;
end;

function TJSONObject.GetNameOf(Index: Integer): TJSONStringType;
begin
  Result:=FHash.NameOfIndex(Index);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetObjects(const AName : String): TJSONObject;
=======
function TJSONObject.GetObjects(AName : String): TJSONObject;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONObject.GetObjects(AName : String): TJSONObject;
>>>>>>> origin/fixes_2_2
begin
  Result:=GetElements(AName) as TJSONObject;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetQWords(AName : String): QWord;
begin
  Result:=GetElements(AName).AsQWord;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TJSONObject.GetStrings(const AName : String): TJSONStringType;
=======
function TJSONObject.GetStrings(AName : String): TJSONStringType;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONObject.GetStrings(AName : String): TJSONStringType;
>>>>>>> origin/fixes_2_2
begin
  Result:=GetElements(AName).AsString;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetUnicodeStrings(const AName: String
  ): TJSONUnicodeStringType;
begin
  Result:=GetElements(AName).AsUnicodeString;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TJSONObject.GetTypes(const AName : String): TJSONType;
=======
function TJSONObject.GetTypes(AName : String): TJSONType;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONObject.GetTypes(AName : String): TJSONType;
>>>>>>> origin/fixes_2_2
begin
  Result:=Getelements(Aname).JSONType;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
class function TJSONObject.GetUnquotedMemberNames: Boolean; static;
begin
  Result:=FUnquotedMemberNames;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TJSONObject.SetArrays(const AName : String; const AValue: TJSONArray);
=======
procedure TJSONObject.SetArrays(AName : String; const AValue: TJSONArray);
>>>>>>> graemeg/fixes_2_2
=======
procedure TJSONObject.SetArrays(AName : String; const AValue: TJSONArray);
>>>>>>> origin/fixes_2_2

begin
  SetElements(AName,AVAlue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONObject.SetBooleans(const AName : String; const AValue: Boolean);
begin
  SetElements(AName,CreateJSON(AVAlue));
end;

procedure TJSONObject.SetElements(const AName: string; const AValue: TJSONData);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TJSONObject.SetBooleans(AName : String; const AValue: Boolean);
begin
  SetElements(AName,TJSONBoolean.Create(AVAlue));
end;

procedure TJSONObject.SetElements(AName: string; const AValue: TJSONData);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Var
  Index : Integer;

begin
  Index:=FHash.FindIndexOf(AName);
  If (Index=-1) then
    FHash.Add(AName,AValue)
  else
    FHash.Items[Index]:=AValue; // Will free the previous value.
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONObject.SetFloats(const AName : String; const AValue: TJSONFloat);
begin
  SetElements(AName,CreateJSON(AVAlue));
end;

procedure TJSONObject.SetIntegers(const AName : String; const AValue: Integer);
begin
  SetElements(AName,CreateJSON(AVAlue));
end;

procedure TJSONObject.SetInt64s(const AName : String; const AValue: Int64);
begin
  SetElements(AName,CreateJSON(AVAlue));
end;

procedure TJSONObject.SetIsNull(const AName : String; const AValue: Boolean);
begin
  If Not AValue then
    DoError(SErrCannotSetNotIsNull);
  SetElements(AName,CreateJSON);
end;

procedure TJSONObject.SetObjects(const AName : String; const AValue: TJSONObject);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TJSONObject.SetFloats(AName : String; const AValue: TJSONFloat);
begin
  SetElements(AName,TJSONFloatNumber.Create(AVAlue));
end;

procedure TJSONObject.SetIntegers(AName : String; const AValue: Integer);
begin
  SetElements(AName,TJSONIntegerNumber.Create(AVAlue));
end;

procedure TJSONObject.SetIsNull(AName : String; const AValue: Boolean);
begin
  If Not AValue then
    Raise EJSON.Create(SErrCannotSetNotIsNull);
  SetElements(AName,TJSONNull.Create);
end;

procedure TJSONObject.SetObjects(AName : String; const AValue: TJSONObject);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  SetElements(AName,AValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONObject.SetQWords(AName : String; AValue: QWord);
begin
  SetElements(AName,CreateJSON(AVAlue));
end;

procedure TJSONObject.SetStrings(const AName : String; const AValue: TJSONStringType);
begin
  SetElements(AName,CreateJSON(AValue));
end;

procedure TJSONObject.SetUnicodeStrings(const AName: String;
  const AValue: TJSONUnicodeStringType);
begin
  SetElements(AName,CreateJSON(AValue));
end;

class procedure TJSONObject.DetermineElementQuotes;

begin
  FObjStartSep:=ObjStartSeps[TJSONData.FCompressedJSON];
  FObjEndSep:=ObjEndSeps[TJSONData.FCompressedJSON];
  if TJSONData.FCompressedJSON then
    FElementEnd:=UnSpacedQuoted[FUnquotedMemberNames]
  else
    FElementEnd:=SpacedQuoted[FUnquotedMemberNames];
  FElementStart:=ElementStart[FUnquotedMemberNames]
end;

class procedure TJSONObject.SetUnquotedMemberNames(AValue: Boolean); static;

=======
procedure TJSONObject.SetStrings(const AName : String; const AValue: TJSONStringType);
>>>>>>> graemeg/cpstrnew
=======
procedure TJSONObject.SetStrings(const AName : String; const AValue: TJSONStringType);
>>>>>>> graemeg/cpstrnew
=======
procedure TJSONObject.SetStrings(const AName : String; const AValue: TJSONStringType);
>>>>>>> graemeg/cpstrnew
=======
procedure TJSONObject.SetStrings(const AName : String; const AValue: TJSONStringType);
>>>>>>> origin/cpstrnew
begin
  if FUnquotedMemberNames=AValue then exit;
  FUnquotedMemberNames:=AValue;
  DetermineElementQuotes;
end;

function TJSONObject.DoFindPath(const APath: TJSONStringType; out
  NotFound: TJSONStringType): TJSONdata;

Var
  N: TJSONStringType;
  L,P,P2 : Integer;

begin
  If (APath='') then
    Exit(Self);
  N:=APath;
  L:=Length(N);
  P:=1;
  While (P<L) and (N[P]='.') do
    inc(P);
  P2:=P;
  While (P2<=L) and (Not (N[P2] in ['.','['])) do
    inc(P2);
   N:=Copy(APath,P,P2-P);
   If (N='') then
     Result:=Self
   else
     begin
     Result:=Find(N);
     If Result=Nil then
       NotFound:=N+Copy(APath,P2,L-P2)
     else
       begin
       N:=Copy(APath,P2,L-P2+1);
       Result:=Result.DoFindPath(N,NotFound);
       end;
     end;
=======
procedure TJSONObject.SetStrings(AName : String; const AValue: TJSONStringType);
begin
  SetElements(AName,TJSONString.Create(AVAlue));
>>>>>>> graemeg/fixes_2_2
=======
procedure TJSONObject.SetStrings(AName : String; const AValue: TJSONStringType);
begin
  SetElements(AName,TJSONString.Create(AVAlue));
>>>>>>> origin/fixes_2_2
end;

procedure TJSONObject.Converterror(From: Boolean);
begin
  If From then
<<<<<<< HEAD
<<<<<<< HEAD
    DoError(SErrCannotConvertFromObject)
  else
    DoError(SErrCannotConvertToObject);
end;

{$warnings off}
=======
=======
>>>>>>> origin/fixes_2_2
    Raise EJSON.Create(SErrCannotConvertFromObject)
  else
    Raise EJSON.Create(SErrCannotConvertToObject);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TJSONObject.GetAsBoolean: Boolean;
begin
  ConvertError(True);
end;

function TJSONObject.GetAsFloat: TJSONFloat;
begin
  ConvertError(True);
end;

function TJSONObject.GetAsInteger: Integer;
begin
  ConvertError(True);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetAsInt64: Int64;
begin
  ConvertError(True);
end;

function TJSONObject.GetAsQWord: QWord;
begin
  ConvertError(True);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TJSONObject.SetAsBoolean(const AValue: Boolean);
begin
  ConvertError(False);
end;

procedure TJSONObject.SetAsFloat(const AValue: TJSONFloat);
begin
  ConvertError(False);
end;

procedure TJSONObject.SetAsInteger(const AValue: Integer);
begin
  ConvertError(False);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONObject.SetAsInt64(const AValue: Int64);
begin
  ConvertError(False);
end;

procedure TJSONObject.SetAsQword(const AValue: QWord);
begin
  ConvertError(False);
end;

{$warnings on}

function TJSONObject.GetAsJSON: TJSONStringType;


Var
  I : Integer;
  Sep : String;

begin
  Sep:=TJSONData.FElementSep;
=======
=======
>>>>>>> origin/fixes_2_2
function TJSONObject.GetAsJSON: TJSONStringType;

Var
  I : Integer;

begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result:='';
  For I:=0 to Count-1 do
    begin
    If (Result<>'') then
<<<<<<< HEAD
<<<<<<< HEAD
      Result:=Result+Sep;
    Result:=Result+FElementStart+StringToJSONString(Names[i])+FElementEnd+Items[I].AsJSON;
    end;
  If (Result<>'') then
    Result:=FObjStartSep+Result+FObjEndSep
=======
=======
>>>>>>> origin/fixes_2_2
      Result:=Result+', ';
    Result:=Result+'"'+StringToJSONString(Names[i])+'" : '+Items[I].AsJSON;
    end;
  If (Result<>'') then
    Result:='{ '+Result+' }'
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  else
    Result:='{}';
end;

<<<<<<< HEAD
<<<<<<< HEAD
{$warnings off}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TJSONObject.GetAsString: TJSONStringType;
begin
  ConvertError(True);
end;

procedure TJSONObject.SetAsString(const AValue: TJSONStringType);
begin
  ConvertError(False);
end;

function TJSONObject.GetValue: variant;
begin
  ConvertError(True);
end;

procedure TJSONObject.SetValue(const AValue: variant);
begin
  ConvertError(False);
end;
<<<<<<< HEAD
<<<<<<< HEAD
{$warnings on}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function TJSONObject.GetCount: Integer;
begin
  Result:=FHash.Count;
end;

function TJSONObject.GetItem(Index: Integer): TJSONData;
begin
  Result:=TJSONData(FHash.Items[Index]);
end;

procedure TJSONObject.SetItem(Index: Integer; const AValue: TJSONData);
begin
  FHash.Items[Index]:=AValue;
end;

constructor TJSONObject.Create;
begin
  FHash:=TFPHashObjectList.Create(True);
end;



constructor TJSONObject.Create(const Elements: array of const);

Var
  I : integer;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  F : TJSONFloat;
>>>>>>> graemeg/fixes_2_2
=======
  F : TJSONFloat;
>>>>>>> origin/fixes_2_2
  AName : String;
  J : TJSONData;

begin
  Create;
  If ((High(Elements)-Low(Elements)) mod 2)=0 then
<<<<<<< HEAD
<<<<<<< HEAD
    DoError(SErrOddNumber);
=======
    Raise EJSON.Create(SErrOddNumber);
>>>>>>> graemeg/fixes_2_2
=======
    Raise EJSON.Create(SErrOddNumber);
>>>>>>> origin/fixes_2_2
  I:=Low(Elements);
  While I<=High(Elements) do
    begin
    With Elements[i] do
      Case VType of
        vtChar       : AName:=VChar;
        vtString     : AName:=vString^;
        vtAnsiString : AName:=(AnsiString(vAnsiString));
        vtPChar      : AName:=StrPas(VPChar);
      else
<<<<<<< HEAD
<<<<<<< HEAD
        DoError(SErrNameMustBeString,[I+1]);
      end;
    If (ANAme='') then
      DoError(SErrNameMustBeString,[I+1]);
=======
=======
>>>>>>> origin/fixes_2_2
        Raise EJSON.CreateFmt(SErrNameMustBeString,[I+1]);
      end;
    If (ANAme='') then
      Raise EJSON.CreateFmt(SErrNameMustBeString,[I+1]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Inc(I);
    J:=VarRecToJSON(Elements[i],'Object');
    Add(AName,J);
    Inc(I);
    end;
end;


destructor TJSONObject.Destroy;
begin
  FreeAndNil(FHash);
  inherited Destroy;
end;

class function TJSONObject.JSONType: TJSONType;
begin
  Result:=jtObject;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.Clone: TJSONData;

Var
  O : TJSONObject;
  I: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

begin
  O:=TJSONObjectClass(ClassType).Create;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  N : TJSONStringType;

begin
  O:=TJSONObject.Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  try
    For I:=0 to Count-1 do
      O.Add(Self.Names[I],Self.Items[I].Clone);
    Result:=O;
  except
    FreeAndNil(O);
    Raise;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.GetEnumerator: TBaseJSONEnumerator;
begin
  Result:=TJSONObjectEnumerator.Create(Self);
end;


function TJSONObject.DoFormatJSON(Options: TFormatOptions; CurrentIndent,
  Indent: Integer): TJSONStringType;
=======

Function TJSONObject.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 
>>>>>>> graemeg/cpstrnew
=======

Function TJSONObject.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 
>>>>>>> graemeg/cpstrnew
=======

Function TJSONObject.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 
>>>>>>> graemeg/cpstrnew
=======

Function TJSONObject.DoFormatJSON(Options : TFormatOptions; CurrentIndent, Indent : Integer) : TJSONStringType; 
>>>>>>> origin/cpstrnew

Var
  i : Integer;
  S : TJSONStringType;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  MultiLine,UseQuotes, SkipWhiteSpace : Boolean;
  NSep,Sep,Ind : String;
begin
  Result:='';
  UseQuotes:=Not (foDoNotQuoteMembers in options);
  MultiLine:=Not (foSingleLineObject in Options);
  SkipWhiteSpace:=foSkipWhiteSpace in Options;
  CurrentIndent:=CurrentIndent+Indent;
  Ind:=IndentString(Options, CurrentIndent);
  If SkipWhiteSpace then
    NSep:=':'
  else
    NSep:=' : ';
  If MultiLine then
    Sep:=','+SLineBreak+Ind
  else if SkipWhiteSpace then
    Sep:=','
  else
    Sep:=', ';
  For I:=0 to Count-1 do
    begin
    If (I>0) then
      Result:=Result+Sep
    else If MultiLine then
      Result:=Result+Ind;
    S:=StringToJSONString(Names[i]);
    If UseQuotes then
      S:='"'+S+'"';
    Result:=Result+S+NSep+Items[I].DoFormatJSON(Options,CurrentIndent,Indent);
    end;
  If (Result<>'') then
    begin
    if MultiLine then
      Result:='{'+sLineBreak+Result+sLineBreak+indentString(options,CurrentIndent-Indent)+'}'
    else
      Result:=ObjStartSeps[SkipWhiteSpace]+Result+ObjEndSeps[SkipWhiteSpace]
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


begin
  CurrentIndent:=CurrentIndent+Indent;  
  For I:=0 to Count-1 do
    begin
    If (Result<>'') then
      begin
      If (foSingleLineObject in Options) then
        Result:=Result+', '
      else
        Result:=Result+','+SLineBreak;
      end;
    If not (foSingleLineObject in Options) then    
      Result:=Result+IndentString(Options,CurrentIndent);
    S:=StringToJSONString(Names[i]);
    If not (foDoNotQuoteMembers in options) then
      S:='"'+S+'"';
    Result:=Result+S+' : '+Items[I].DoFormatJSON(Options,CurrentIndent,Indent);
    end;
  If (Result<>'') then
    begin
    if (foSingleLineObject in Options) then
      Result:='{ '+Result+' }'
    else  
      Result:='{'+sLineBreak+Result+sLineBreak+indentString(options,CurrentIndent-Indent)+'}'
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end
  else
    Result:='{}';
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TJSONObject.Iterate(Iterator: TJSONObjectIterator; Data: TObject);

Var
  I : Integer;
  Cont : Boolean;

begin
  I:=0;
  Cont:=True;
  While (I<FHash.Count) and cont do
    begin
    Iterator(Names[I],Items[i],Data,Cont);
    Inc(I);
    end;
end;

function TJSONObject.IndexOf(Item: TJSONData): Integer;
begin
  Result:=FHash.IndexOf(Item);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.IndexOfName(const AName: TJSONStringType; CaseInsensitive : Boolean = False): Integer;

begin
  Result:=FHash.FindIndexOf(AName);
  if (Result=-1) and CaseInsensitive then
    begin
    Result:=Count-1;
    While (Result>=0) and (CompareText(Names[Result],AName)<>0) do
      Dec(Result);
    end;
=======
function TJSONObject.IndexOfName(const AName: TJSONStringType): Integer;
begin
  Result:=FHash.FindIndexOf(AName);
>>>>>>> graemeg/fixes_2_2
=======
function TJSONObject.IndexOfName(const AName: TJSONStringType): Integer;
begin
  Result:=FHash.FindIndexOf(AName);
>>>>>>> origin/fixes_2_2
end;

procedure TJSONObject.Clear;
begin
  FHash.Clear;
end;

function TJSONObject.Add(const AName: TJSONStringType; AValue: TJSONData
  ): Integer;
begin
  Result:=FHash.Add(AName,AValue);
end;

function TJSONObject.Add(const AName: TJSONStringType; AValue: Boolean
  ): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=Add(AName,CreateJSON(AValue));
=======
  Result:=Add(AName,TJSONBoolean.Create(AValue));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Add(AName,TJSONBoolean.Create(AValue));
>>>>>>> origin/fixes_2_2
end;

function TJSONObject.Add(const AName: TJSONStringType; AValue: TJSONFloat): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=Add(AName,CreateJSON(AValue));
end;

function TJSONObject.Add(const AName, AValue: TJSONStringType): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=Add(AName,CreateJSON(AValue));
end;

function TJSONObject.Add(const AName: String; AValue: TJSONUnicodeStringType
  ): Integer;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  Result:=Add(AName,CreateJSON(AValue));
=======
=======
>>>>>>> origin/fixes_2_2
  Result:=Add(AName,TJSONFloatNumber.Create(AValue));
end;

function TJSONObject.Add(const AName: TJSONStringType; AValue: TJSONStringType): Integer;
begin
  Result:=Add(AName,TJSONString.Create(AValue));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function TJSONObject.Add(const AName: TJSONStringType; Avalue: Integer): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=Add(AName,CreateJSON(AValue));
end;

function TJSONObject.Add(const AName: TJSONStringType; Avalue: Int64): Integer;
begin
  Result:=Add(AName,CreateJSON(AValue));
end;

function TJSONObject.Add(const AName: TJSONStringType; Avalue: QWord): Integer;
begin
  Result:=Add(AName,CreateJSON(AValue));
=======
  Result:=Add(AName,TJSONIntegerNumber.Create(AValue));
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Add(AName,TJSONIntegerNumber.Create(AValue));
>>>>>>> origin/fixes_2_2
end;

function TJSONObject.Add(const AName: TJSONStringType): Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=Add(AName,CreateJSON);
=======
  Result:=Add(AName,TJSONNull.Create);
>>>>>>> graemeg/fixes_2_2
=======
  Result:=Add(AName,TJSONNull.Create);
>>>>>>> origin/fixes_2_2
end;

function TJSONObject.Add(const AName: TJSONStringType; AValue: TJSONArray
  ): Integer;
begin
  Result:=Add(AName,TJSONData(AValue));
end;

procedure TJSONObject.Delete(Index: Integer);
begin
  FHash.Delete(Index);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONObject.Delete(const AName: string);

Var
  I : Integer;

begin
  I:=IndexOfName(AName);
  if (I<>-1) then
    Delete(I);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TJSONObject.Remove(Item: TJSONData);
begin
  FHash.Remove(Item);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.Extract(Index: Integer): TJSONData;
begin
  Result:=Items[Index];
  FHash.Extract(Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TJSONObject.Extract(const AName: string): TJSONData;

Var
  I : Integer;

begin
  I:=IndexOfName(AName);
  if (I<>-1) then
    Result:=Extract(I)
  else
    Result:=Nil
end;

function TJSONObject.Get(const AName: String): Variant;
Var
  I : Integer;

begin
  I:=IndexOfName(AName);
  If (I<>-1) then
    Result:=Items[i].Value
  else
    Result:=Null;
end;

function TJSONObject.Get(const AName: String; ADefault: TJSONFloat
  ): TJSONFloat;

Var
  D : TJSONData;

begin
  D:=Find(AName,jtNumber);
  If D<>Nil then
    Result:=D.AsFloat
  else
    Result:=ADefault;
end;

function TJSONObject.Get(const AName: String; ADefault: Integer
  ): Integer;

Var
  D : TJSONData;

begin
  D:=Find(AName,jtNumber);
  If D<>Nil then
    Result:=D.AsInteger
  else
    Result:=ADefault;
end;

function TJSONObject.Get(const AName: String; ADefault: Int64): Int64;
Var
  D : TJSONData;

begin
  D:=Find(AName,jtNumber);
  If D<>Nil then
    Result:=D.AsInt64
  else
    Result:=ADefault;
end;

function TJSONObject.Get(const AName: String; ADefault: QWord): QWord;
Var
  D : TJSONData;

begin
  D:=Find(AName,jtNumber);
  If D<>Nil then
    Result:=D.AsQWord
  else
    Result:=ADefault;
end;

function TJSONObject.Get(const AName: String; ADefault: Boolean
  ): Boolean;
Var
  D : TJSONData;

begin
  D:=Find(AName,jtBoolean);
  If D<>Nil then
    Result:=D.AsBoolean
  else
    Result:=ADefault;
end;

function TJSONObject.Get(const AName: String; ADefault: TJSONStringType
  ): TJSONStringType;
Var
  D : TJSONData;

begin
  D:=Find(AName,jtString);
  If (D<>Nil) then
    Result:=D.AsString
  else
    Result:=ADefault;
end;

function TJSONObject.Get(const AName: String; ADefault: TJSONUnicodeStringType
  ): TJSONUnicodeStringType;
Var
  D : TJSONData;

begin
  D:=Find(AName,jtString);
  If (D<>Nil) then
    Result:=D.AsUnicodeString
  else
    Result:=ADefault;
end;

function TJSONObject.Get(const AName: String; ADefault: TJSONArray
  ): TJSONArray;
Var
  D : TJSONData;

begin
  D:=Find(AName,jtArray);
  If (D<>Nil) then
    Result:=TJSONArray(D)
  else
    Result:=ADefault;
end;

function TJSONObject.Get(const AName: String; ADefault: TJSONObject
  ): TJSONObject;
Var
  D : TJSONData;

begin
  D:=Find(AName,jtObject);
  If (D<>Nil) then
    Result:=TJSONObject(D)
  else
    Result:=ADefault;
end;

function TJSONObject.Find(const AName: String): TJSONData;

Var
  I : Integer;

begin
  I:=IndexOfName(AName);
  If (I<>-1) then
    Result:=Items[i]
  else
    Result:=Nil;
end;

function TJSONObject.Find(const AName: String; AType: TJSONType): TJSONData;
begin
  Result:=Find(AName);
  If Assigned(Result) and (Result.JSONType<>AType) then
    Result:=Nil;
end;

initialization
  // Need to force initialization;
  TJSONData.DetermineElementSeparators;
  TJSONObject.DetermineElementQuotes;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.

