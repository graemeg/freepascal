{
    This file is part of the Free Component Library

    XML reading routines.
    Copyright (c) 1999-2000 by Sebastian Guenther, sg@freepascal.org
    Modified in 2006 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit XMLRead;

{$ifdef fpc}
{$MODE objfpc}{$H+}
{$endif}

interface

uses
  SysUtils, Classes, DOM, xmlutils, xmlreader, xmltextreader;

type
<<<<<<< HEAD
<<<<<<< HEAD
  TErrorSeverity = xmlreader.TErrorSeverity;
  EXMLReadError = xmlreader.EXMLReadError;
  TXMLInputSource = xmlreader.TXMLInputSource;

<<<<<<< HEAD
const
  esWarning = xmlreader.esWarning;
  esError = xmlreader.esError;
  esFatal = xmlreader.esFatal;

procedure ReadXMLFile(out ADoc: TXMLDocument; const AFilename: String); overload;
procedure ReadXMLFile(out ADoc: TXMLDocument; var f: Text); overload;
procedure ReadXMLFile(out ADoc: TXMLDocument; f: TStream); overload;
procedure ReadXMLFile(out ADoc: TXMLDocument; f: TStream; const ABaseURI: String); overload;

procedure ReadXMLFragment(AParentNode: TDOMNode; const AFilename: String); overload;
procedure ReadXMLFragment(AParentNode: TDOMNode; var f: Text); overload;
procedure ReadXMLFragment(AParentNode: TDOMNode; f: TStream); overload;
procedure ReadXMLFragment(AParentNode: TDOMNode; f: TStream; const ABaseURI: String); overload;

procedure ReadDTDFile(out ADoc: TXMLDocument; const AFilename: String);  overload;
procedure ReadDTDFile(out ADoc: TXMLDocument; var f: Text); overload;
procedure ReadDTDFile(out ADoc: TXMLDocument; f: TStream); overload;
procedure ReadDTDFile(out ADoc: TXMLDocument; f: TStream; const ABaseURI: String); overload;

type
  TXMLErrorEvent = xmlreader.TXMLErrorEvent;
  TDOMParseOptions = xmlreader.TXMLReaderSettings;

  // NOTE: DOM 3 LS ACTION_TYPE enumeration starts at 1
  TXMLContextAction = (
    xaAppendAsChildren = 1,
    xaReplaceChildren,
    xaInsertBefore,
    xaInsertAfter,
    xaReplace);
=======
=======
  TErrorSeverity = (esWarning, esError, esFatal);

>>>>>>> graemeg/fixes_2_2
=======
  TErrorSeverity = (esWarning, esError, esFatal);

>>>>>>> origin/fixes_2_2
  EXMLReadError = class(Exception)
  private
    FSeverity: TErrorSeverity;
    FErrorMessage: string;
    FLine: Integer;
    FLinePos: Integer;
  public
    property Severity: TErrorSeverity read FSeverity;
    property ErrorMessage: string read FErrorMessage;
    property Line: Integer read FLine;
    property LinePos: Integer read FLinePos;
  end;

procedure ReadXMLFile(out ADoc: TXMLDocument; const AFilename: String); overload;
procedure ReadXMLFile(out ADoc: TXMLDocument; var f: Text); overload;
procedure ReadXMLFile(out ADoc: TXMLDocument; f: TStream); overload;
procedure ReadXMLFile(out ADoc: TXMLDocument; f: TStream; const ABaseURI: String); overload;

procedure ReadXMLFragment(AParentNode: TDOMNode; const AFilename: String); overload;
procedure ReadXMLFragment(AParentNode: TDOMNode; var f: Text); overload;
procedure ReadXMLFragment(AParentNode: TDOMNode; f: TStream); overload;
procedure ReadXMLFragment(AParentNode: TDOMNode; f: TStream; const ABaseURI: String); overload;

procedure ReadDTDFile(out ADoc: TXMLDocument; const AFilename: String);  overload;
procedure ReadDTDFile(out ADoc: TXMLDocument; var f: Text); overload;
procedure ReadDTDFile(out ADoc: TXMLDocument; f: TStream); overload;
procedure ReadDTDFile(out ADoc: TXMLDocument; f: TStream; const ABaseURI: String); overload;

type
  TDOMParseOptions = class(TObject)
  private
    FValidate: Boolean;
    FPreserveWhitespace: Boolean;
    FExpandEntities: Boolean;
    FIgnoreComments: Boolean;
    FCDSectionsAsText: Boolean;
    FResolveExternals: Boolean;
    FNamespaces: Boolean;
    FDisallowDoctype: Boolean;
    FCanonical: Boolean;
    FMaxChars: Cardinal;
    function GetCanonical: Boolean;
    procedure SetCanonical(aValue: Boolean);
  public
    property Validate: Boolean read FValidate write FValidate;
    property PreserveWhitespace: Boolean read FPreserveWhitespace write FPreserveWhitespace;
    property ExpandEntities: Boolean read FExpandEntities write FExpandEntities;
    property IgnoreComments: Boolean read FIgnoreComments write FIgnoreComments;
    property CDSectionsAsText: Boolean read FCDSectionsAsText write FCDSectionsAsText;
    property ResolveExternals: Boolean read FResolveExternals write FResolveExternals;
    property Namespaces: Boolean read FNamespaces write FNamespaces;
    property DisallowDoctype: Boolean read FDisallowDoctype write FDisallowDoctype;
    property MaxChars: Cardinal read FMaxChars write FMaxChars;
    property CanonicalForm: Boolean read GetCanonical write SetCanonical;
  end;

  // NOTE: DOM 3 LS ACTION_TYPE enumeration starts at 1
  TXMLContextAction = (
    xaAppendAsChildren = 1,
    xaReplaceChildren,
    xaInsertBefore,
    xaInsertAfter,
    xaReplace);

  TXMLErrorEvent = procedure(Error: EXMLReadError) of object;

  TXMLInputSource = class(TObject)
  private
    FStream: TStream;
    FStringData: string;
    FBaseURI: WideString;
    FSystemID: WideString;
    FPublicID: WideString;
//    FEncoding: string;
  public
    constructor Create(AStream: TStream); overload;
    constructor Create(const AStringData: string); overload;
    property Stream: TStream read FStream;
    property StringData: string read FStringData;
    property BaseURI: WideString read FBaseURI write FBaseURI;
    property SystemID: WideString read FSystemID write FSystemID;
    property PublicID: WideString read FPublicID write FPublicID;
//    property Encoding: string read FEncoding write FEncoding;
  end;

  TDOMParser = class(TObject)
  private
    FOptions: TDOMParseOptions;
    FOnError: TXMLErrorEvent;
  public
    constructor Create;
    destructor Destroy; override;
    procedure Parse(Src: TXMLInputSource; out ADoc: TXMLDocument);
    procedure ParseUri(const URI: WideString; out ADoc: TXMLDocument);
    function ParseWithContext(Src: TXMLInputSource; Context: TDOMNode;
      Action: TXMLContextAction): TDOMNode;
    property Options: TDOMParseOptions read FOptions;
    property OnError: TXMLErrorEvent read FOnError write FOnError;
  end;

  TDecoder = record
    Context: Pointer;
    Decode: function(Context: Pointer; InBuf: PChar; var InCnt: Cardinal; OutBuf: PWideChar; var OutCnt: Cardinal): Integer; stdcall;
    Cleanup: procedure(Context: Pointer); stdcall;
  end;

  TGetDecoderProc = function(const AEncoding: string; out Decoder: TDecoder): Boolean; stdcall;

procedure RegisterDecoder(Proc: TGetDecoderProc);

type
  TDOMParseOptions = class(TObject)
  private
    FValidate: Boolean;
    FPreserveWhitespace: Boolean;
    FExpandEntities: Boolean;
    FIgnoreComments: Boolean;
    FCDSectionsAsText: Boolean;
    FResolveExternals: Boolean;
    FNamespaces: Boolean;
  public
    property Validate: Boolean read FValidate write FValidate;
    property PreserveWhitespace: Boolean read FPreserveWhitespace write FPreserveWhitespace;
    property ExpandEntities: Boolean read FExpandEntities write FExpandEntities;
    property IgnoreComments: Boolean read FIgnoreComments write FIgnoreComments;
    property CDSectionsAsText: Boolean read FCDSectionsAsText write FCDSectionsAsText;
    property ResolveExternals: Boolean read FResolveExternals write FResolveExternals;
    property Namespaces: Boolean read FNamespaces write FNamespaces;
  end;

  // NOTE: DOM 3 LS ACTION_TYPE enumeration starts at 1
  TXMLContextAction = (
    xaAppendAsChildren = 1,
    xaReplaceChildren,
    xaInsertBefore,
    xaInsertAfter,
    xaReplace);

  TXMLErrorEvent = procedure(Error: EXMLReadError) of object;

  TXMLInputSource = class(TObject)
  private
    FStream: TStream;
    FStringData: string;
//    FBaseURI: WideString;
    FSystemID: WideString;
    FPublicID: WideString;
//    FEncoding: string;
  public
    constructor Create(AStream: TStream); overload;
    constructor Create(const AStringData: string); overload;
    property Stream: TStream read FStream;
    property StringData: string read FStringData;
//    property BaseURI: WideString read FBaseURI write FBaseURI;
    property SystemID: WideString read FSystemID write FSystemID;
    property PublicID: WideString read FPublicID write FPublicID;
//    property Encoding: string read FEncoding write FEncoding;
  end;

  TDOMParser = class(TObject)
  private
    FOptions: TDOMParseOptions;
    FOnError: TXMLErrorEvent;
  public
    constructor Create;
    destructor Destroy; override;
    procedure Parse(Src: TXMLInputSource; out ADoc: TXMLDocument);
    procedure ParseUri(const URI: WideString; out ADoc: TXMLDocument);
    function ParseWithContext(Src: TXMLInputSource; Context: TDOMNode;
      Action: TXMLContextAction): TDOMNode;
    property Options: TDOMParseOptions read FOptions;
    property OnError: TXMLErrorEvent read FOnError write FOnError;
  end;


type
  TDOMParseOptions = class(TObject)
  private
    FValidate: Boolean;
    FPreserveWhitespace: Boolean;
    FExpandEntities: Boolean;
    FIgnoreComments: Boolean;
    FCDSectionsAsText: Boolean;
    FResolveExternals: Boolean;
    FNamespaces: Boolean;
  public
    property Validate: Boolean read FValidate write FValidate;
    property PreserveWhitespace: Boolean read FPreserveWhitespace write FPreserveWhitespace;
    property ExpandEntities: Boolean read FExpandEntities write FExpandEntities;
    property IgnoreComments: Boolean read FIgnoreComments write FIgnoreComments;
    property CDSectionsAsText: Boolean read FCDSectionsAsText write FCDSectionsAsText;
    property ResolveExternals: Boolean read FResolveExternals write FResolveExternals;
    property Namespaces: Boolean read FNamespaces write FNamespaces;
  end;

  // NOTE: DOM 3 LS ACTION_TYPE enumeration starts at 1
  TXMLContextAction = (
    xaAppendAsChildren = 1,
    xaReplaceChildren,
    xaInsertBefore,
    xaInsertAfter,
    xaReplace);

  TXMLErrorEvent = procedure(Error: EXMLReadError) of object;

  TXMLInputSource = class(TObject)
  private
    FStream: TStream;
    FStringData: string;
//    FBaseURI: WideString;
    FSystemID: WideString;
    FPublicID: WideString;
//    FEncoding: string;
  public
    constructor Create(AStream: TStream); overload;
    constructor Create(const AStringData: string); overload;
    property Stream: TStream read FStream;
    property StringData: string read FStringData;
//    property BaseURI: WideString read FBaseURI write FBaseURI;
    property SystemID: WideString read FSystemID write FSystemID;
    property PublicID: WideString read FPublicID write FPublicID;
//    property Encoding: string read FEncoding write FEncoding;
  end;

  TDOMParser = class(TObject)
  private
    FOptions: TDOMParseOptions;
    FOnError: TXMLErrorEvent;
  public
    constructor Create;
    destructor Destroy; override;
    procedure Parse(Src: TXMLInputSource; out ADoc: TXMLDocument);
    procedure ParseUri(const URI: WideString; out ADoc: TXMLDocument);
    function ParseWithContext(Src: TXMLInputSource; Context: TDOMNode;
      Action: TXMLContextAction): TDOMNode;
    property Options: TDOMParseOptions read FOptions;
    property OnError: TXMLErrorEvent read FOnError write FOnError;
  end;


// =======================================================

implementation

uses
<<<<<<< HEAD
<<<<<<< HEAD
  UriParser, xmlutils, dtdmodel;
=======
  UriParser, xmlutils;
>>>>>>> graemeg/fixes_2_2
=======
  UriParser, xmlutils;
>>>>>>> origin/fixes_2_2

const
  PubidChars: TSetOfChar = [' ', #13, #10, 'a'..'z', 'A'..'Z', '0'..'9',
    '-', '''', '(', ')', '+', ',', '.', '/', ':', '=', '?', ';', '!', '*',
    '#', '@', '$', '_', '%'];

type
<<<<<<< HEAD
<<<<<<< HEAD
  TDOMDocumentTypeEx = class(TDOMDocumentType);
  TDOMTopNodeEx = class(TDOMNode_TopLevel);

  TXMLSourceKind = (skNone, skInternalSubset, skManualPop);

  TLocation = xmlutils.TLocation;

  TDOMEntityEx = class(TDOMEntity);
=======
=======
>>>>>>> origin/fixes_2_2
  TDOMNotationEx = class(TDOMNotation);
  TDOMDocumentTypeEx = class(TDOMDocumentType);
  TDOMElementDef = class;
  TDOMAttrDef = class;

  TDTDSubsetType = (dsNone, dsInternal, dsExternal);

  // This may be augmented with ByteOffset, UTF8Offset, etc.
  TLocation = record
    Line: Integer;
    LinePos: Integer;
  end;

  TDOMEntityEx = class(TDOMEntity)
  protected
    FExternallyDeclared: Boolean;
    FResolved: Boolean;
    FOnStack: Boolean;
    FBetweenDecls: Boolean;
    FReplacementText: DOMString;
    FStartLocation: TLocation;
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  TXMLReader = class;

  TXMLCharSource = class(TObject)
  private
    FBuf: PWideChar;
    FBufEnd: PWideChar;
    FReader: TXMLReader;
    FParent: TXMLCharSource;
<<<<<<< HEAD
<<<<<<< HEAD
    FEntity: TEntityDecl;
=======
    FEntity: TObject;   // weak reference
>>>>>>> graemeg/fixes_2_2
=======
    FEntity: TObject;   // weak reference
>>>>>>> origin/fixes_2_2
    FLineNo: Integer;
    LFPos: PWideChar;
    FXML11Rules: Boolean;
    FSystemID: WideString;
    FCharCount: Cardinal;
    FStartNesting: Integer;
    FXMLVersion: TXMLVersion;
    FXMLEncoding: WideString;
    function GetSystemID: WideString;
  protected
    function Reload: Boolean; virtual;
  public
<<<<<<< HEAD
<<<<<<< HEAD
    Kind: TXMLSourceKind;
    constructor Create(const AData: WideString);
    procedure NextChar;
    procedure NewLine; virtual;
    function SkipUntil(var ToFill: TWideCharBuf; const Delim: TSetOfChar;
      wsflag: PBoolean = nil): WideChar; virtual;
=======
    DTDSubsetType: TDTDSubsetType;
    constructor Create(const AData: WideString);
    function NextChar: WideChar;
>>>>>>> graemeg/fixes_2_2
=======
    DTDSubsetType: TDTDSubsetType;
    constructor Create(const AData: WideString);
    function NextChar: WideChar;
>>>>>>> origin/fixes_2_2
    procedure Initialize; virtual;
    function SetEncoding(const AEncoding: string): Boolean; virtual;
    function Matches(const arg: WideString): Boolean;
    property SystemID: WideString read GetSystemID write FSystemID;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
  TXMLDecodingSource = class;
  TDecoder = function(Src: TXMLDecodingSource): WideChar;
>>>>>>> graemeg/fixes_2_2
=======
  TXMLDecodingSource = class;
  TDecoder = function(Src: TXMLDecodingSource): WideChar;
>>>>>>> origin/fixes_2_2
  TXMLDecodingSource = class(TXMLCharSource)
  private
    FCharBuf: PChar;
    FCharBufEnd: PChar;
    FBufStart: PWideChar;
    FDecoder: TDecoder;
<<<<<<< HEAD
    FHasBOM: Boolean;
    FFixedUCS2: string;
    FBufSize: Integer;
=======
    FSeenCR: Boolean;
    FFixedUCS2: string;
    FBufSize: Integer;
    FSurrogate: WideChar;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure DecodingError(const Msg: string);
  protected
    function Reload: Boolean; override;
    procedure FetchData; virtual;
  public
    procedure AfterConstruction; override;
    destructor Destroy; override;
    function SetEncoding(const AEncoding: string): Boolean; override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure NewLine; override;
    function SkipUntil(var ToFill: TWideCharBuf; const Delim: TSetOfChar;
      wsflag: PBoolean = nil): WideChar; override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure Initialize; override;
  end;

  TXMLStreamInputSource = class(TXMLDecodingSource)
  private
    FAllocated: PChar;
    FStream: TStream;
    FCapacity: Integer;
    FOwnStream: Boolean;
    FEof: Boolean;
  public
    constructor Create(AStream: TStream; AOwnStream: Boolean);
    destructor Destroy; override;
    procedure FetchData; override;
  end;

  TXMLFileInputSource = class(TXMLDecodingSource)
  private
    FFile: ^Text;
    FString: string;
    FTmp: string;
  public
    constructor Create(var AFile: Text);
    procedure FetchData; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  PForwardRef = ^TForwardRef;
  TForwardRef = record
    Value: WideString;
    Loc: TLocation;
  end;

  TElementValidator = object
    FElementDef: TElementDecl;
    FCurCP: TContentParticle;
    FFailed: Boolean;
    FSaViolation: Boolean;
    FContentType: TElementContentType;       // =ctAny when FElementDef is nil
    function IsElementAllowed(Def: TElementDecl): Boolean;
    function Incomplete: Boolean;
  end;

  TNodeDataDynArray = array of TNodeData;
  TValidatorDynArray = array of TElementValidator;

  TXMLReadState = (rsProlog, rsDTD, rsAfterDTD, rsRoot, rsEpilog);

  TCheckNameFlags = set of (cnOptional, cnToken);

  TXMLToken = (xtNone, xtEOF, xtText, xtWhitespace, xtElement, xtEndElement,
    xtCDSect, xtComment, xtPI, xtDoctype, xtEntity, xtEntityEnd, xtPopElement,
    xtPopEmptyElement, xtPushElement, xtPushEntity, xtPopEntity);

  TLiteralType = (ltPlain, ltPubid, ltEntity);
=======
  PWideCharBuf = ^TWideCharBuf;
  TWideCharBuf = record
    Buffer: PWideChar;
    Length: Integer;
    MaxLength: Integer;
  end;

  PForwardRef = ^TForwardRef;
  TForwardRef = record
    Value: WideString;
    Loc: TLocation;
  end;

  TCPType = (ctName, ctChoice, ctSeq);
  TCPQuant = (cqOnce, cqZeroOrOnce, cqZeroOrMore, cqOnceOrMore);

  TContentParticle = class(TObject)
  private
    FParent: TContentParticle;
    FChildren: TFPList;
    FIndex: Integer;
    function GetChildCount: Integer;
    function GetChild(Index: Integer): TContentParticle;
  public
    CPType: TCPType;
    CPQuant: TCPQuant;
    Def: TDOMElementDef;
    destructor Destroy; override;
    function Add: TContentParticle;
    function IsRequired: Boolean;
    function FindFirst(aDef: TDOMElementDef): TContentParticle;
    function FindNext(aDef: TDOMElementDef; ChildIdx: Integer): TContentParticle;
    function MoreRequired(ChildIdx: Integer): Boolean;
    property ChildCount: Integer read GetChildCount;
    property Children[Index: Integer]: TContentParticle read GetChild;
  end;

  TElementValidator = object
    FElementDef: TDOMElementDef;
    FCurCP: TContentParticle;
    FFailed: Boolean;
    function IsElementAllowed(Def: TDOMElementDef): Boolean;
    function Incomplete: Boolean;
  end;

  TXMLReadState = (rsProlog, rsDTD, rsRoot, rsEpilog);

  TAttrDefault = (
    adImplied,
    adDefault,
    adRequired,
    adFixed
  );

  TElementContentType = (
    ctAny,
    ctEmpty,
    ctMixed,
    ctChildren
  );

  TCheckNameFlags = set of (cnOptional, cnToken);
>>>>>>> graemeg/fixes_2_2
=======
  PWideCharBuf = ^TWideCharBuf;
  TWideCharBuf = record
    Buffer: PWideChar;
    Length: Integer;
    MaxLength: Integer;
  end;

  PForwardRef = ^TForwardRef;
  TForwardRef = record
    Value: WideString;
    Loc: TLocation;
  end;

  TCPType = (ctName, ctChoice, ctSeq);
  TCPQuant = (cqOnce, cqZeroOrOnce, cqZeroOrMore, cqOnceOrMore);

  TContentParticle = class(TObject)
  private
    FParent: TContentParticle;
    FChildren: TFPList;
    FIndex: Integer;
    function GetChildCount: Integer;
    function GetChild(Index: Integer): TContentParticle;
  public
    CPType: TCPType;
    CPQuant: TCPQuant;
    Def: TDOMElementDef;
    destructor Destroy; override;
    function Add: TContentParticle;
    function IsRequired: Boolean;
    function FindFirst(aDef: TDOMElementDef): TContentParticle;
    function FindNext(aDef: TDOMElementDef; ChildIdx: Integer): TContentParticle;
    function MoreRequired(ChildIdx: Integer): Boolean;
    property ChildCount: Integer read GetChildCount;
    property Children[Index: Integer]: TContentParticle read GetChild;
  end;

  TElementValidator = object
    FElementDef: TDOMElementDef;
    FCurCP: TContentParticle;
    FFailed: Boolean;
    function IsElementAllowed(Def: TDOMElementDef): Boolean;
    function Incomplete: Boolean;
  end;

  TXMLReadState = (rsProlog, rsDTD, rsRoot, rsEpilog);

  TAttrDefault = (
    adImplied,
    adDefault,
    adRequired,
    adFixed
  );

  TElementContentType = (
    ctAny,
    ctEmpty,
    ctMixed,
    ctChildren
  );

  TCheckNameFlags = set of (cnOptional, cnToken);
>>>>>>> origin/fixes_2_2

  TXMLReader = class
  private
    FSource: TXMLCharSource;
<<<<<<< HEAD
<<<<<<< HEAD
    FNameTable: THashTable;
    FCtrl: TDOMParser;
    FXML11: Boolean;
    FState: TXMLReadState;
    FHavePERefs: Boolean;
    FInsideDecl: Boolean;
=======
=======
>>>>>>> origin/fixes_2_2
    FCtrl: TDOMParser;
    FCurChar: WideChar;
    FXML11: Boolean;
    FState: TXMLReadState;
    FRecognizePE: Boolean;
    FHavePERefs: Boolean;
    FInsideDecl: Boolean;
    FDocNotValid: Boolean;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FValue: TWideCharBuf;
    FEntityValue: TWideCharBuf;
    FName: TWideCharBuf;
    FTokenStart: TLocation;
<<<<<<< HEAD
<<<<<<< HEAD
    FStandalone: Boolean;
    FNamePages: PByteArray;
    FDocType: TDTDModel;
    FPEMap: THashTable;
    FForwardRefs: TFPList;
    FDTDStartPos: PWideChar;
    FIntSubset: TWideCharBuf;
    FAttrTag: Cardinal;
    FDTDProcessed: Boolean;
    FFragmentMode: Boolean;
    FNext: TXMLToken;
    FCurrEntity: TEntityDecl;
    FIDMap: THashTable;

    FNSHelper: TNSSupport;
    FNsAttHash: TDblHashArray;
    FStdPrefix_xml: PHashItem;
    FStdPrefix_xmlns: PHashItem;
    FStdUri_xml: PHashItem;
    FStdUri_xmlns: PHashItem;
=======
=======
>>>>>>> origin/fixes_2_2
    FStandalone: Boolean;          // property of Doc ?
    FNamePages: PByteArray;
    FDocType: TDOMDocumentTypeEx;  // a shortcut
    FPEMap: TDOMNamedNodeMap;
    FIDRefs: TFPList;
    FNotationRefs: TFPList;
    FCurrContentType: TElementContentType;
    FSaViolation: Boolean;
    FDTDStartPos: PWideChar;
    FIntSubset: TWideCharBuf;
    FAttrTag: Cardinal;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    FColonPos: Integer;
    FValidate: Boolean;            // parsing options, copy of FCtrl.Options
    FPreserveWhitespace: Boolean;
    FExpandEntities: Boolean;
    FIgnoreComments: Boolean;
    FCDSectionsAsText: Boolean;
    FResolveExternals: Boolean;
    FNamespaces: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    FDisallowDoctype: Boolean;
    FCanonical: Boolean;
    FMaxChars: Cardinal;

    procedure SkipQuote(out Delim: WideChar; required: Boolean = True);
    procedure Initialize(ASource: TXMLCharSource);
    procedure NSPrepare;
    procedure EntityToSource(AEntity: TEntityDecl; out Src: TXMLCharSource);
    function ContextPush(AEntity: TEntityDecl): Boolean;
    function ContextPop(Forced: Boolean = False): Boolean;
    procedure XML11_BuildTables;
    function ParseQuantity: TCPQuant;
    procedure StoreLocation(out Loc: TLocation);
    function ValidateAttrSyntax(AttrDef: TAttributeDef; const aValue: WideString): Boolean;
    procedure ValidateAttrValue(AttrDef: TAttributeDef; attrData: PNodeData);
    procedure AddForwardRef(Buf: PWideChar; Length: Integer);
    procedure ClearForwardRefs;
    procedure ValidateIdRefs;
    procedure StandaloneError(LineOffs: Integer = 0);
    procedure CallErrorHandler(E: EXMLReadError);
    function  FindOrCreateElDef: TElementDecl;
    function  SkipUntilSeq(const Delim: TSetOfChar; c1: WideChar; c2: WideChar = #0): Boolean;
    procedure CheckMaxChars(ToAdd: Cardinal);
    function AllocNodeData(AIndex: Integer): PNodeData;
    function AllocAttributeData: PNodeData;
    function AllocAttributeValueChunk(APrev: PNodeData): PNodeData;
    procedure CleanupAttribute(aNode: PNodeData);
    procedure CleanupAttributes;
    procedure SetNodeInfoWithValue(typ: TXMLNodeType; AName: PHashItem = nil);
    function AddId(aNodeData: PNodeData): Boolean;
  protected
    FNesting: Integer;
    FCurrNode: PNodeData;
    FAttrCount: Integer;
    FPrefixedAttrs: Integer;
    FSpecifiedAttrs: Integer;
    FNodeStack: TNodeDataDynArray;
    FValidatorNesting: Integer;
    FValidators: TValidatorDynArray;
    FAttrChunks: TFPList;
    FFreeAttrChunk: PNodeData;
    FAttrCleanupFlag: Boolean;

    procedure DoError(Severity: TErrorSeverity; const descr: string; LineOffs: Integer=0);
    procedure DoErrorPos(Severity: TErrorSeverity; const descr: string;
      const ErrPos: TLocation); overload;
    procedure DoErrorPos(Severity: TErrorSeverity; const descr: string;
      const args: array of const; const ErrPos: TLocation); overload;
=======
=======
>>>>>>> origin/fixes_2_2

    procedure RaiseExpectedQmark;
    procedure GetChar;
    procedure Initialize(ASource: TXMLCharSource);
    function DoParseAttValue(Delim: WideChar): Boolean;
    procedure DoParseFragment;
    function ContextPush(AEntity: TDOMEntityEx): Boolean; overload;
    procedure ContextPush(ASrc: TXMLCharSource); overload;
    function ContextPop: Boolean;
    procedure XML11_BuildTables;
    procedure ParseQuantity(CP: TContentParticle);
    procedure StoreLocation(out Loc: TLocation);
    function ValidateAttrSyntax(AttrDef: TDOMAttrDef; const aValue: WideString): Boolean;
    procedure ValidateAttrValue(Attr: TDOMAttr; const aValue: WideString);
    procedure AddForwardRef(aList: TFPList; Buf: PWideChar; Length: Integer);
    procedure ClearRefs(aList: TFPList);
    procedure ValidateIdRefs;
    procedure StandaloneError(LineOffs: Integer = 0);
    procedure CallErrorHandler(E: EXMLReadError);
    function  FindOrCreateElDef: TDOMElementDef;
  protected
    FCursor: TDOMNode_WithChildren;
    FNesting: Integer;
    FValidator: array of TElementValidator;

    procedure DoError(Severity: TErrorSeverity; const descr: string; LineOffs: Integer=0);
    procedure DoErrorPos(Severity: TErrorSeverity; const descr: string;
      const ErrPos: TLocation);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure FatalError(const descr: String; LineOffs: Integer=0); overload;
    procedure FatalError(const descr: string; const args: array of const; LineOffs: Integer=0); overload;
    procedure FatalError(Expected: WideChar); overload;
    function  SkipWhitespace(PercentAloneIsOk: Boolean = False): Boolean;
    function  SkipS(required: Boolean = False): Boolean;
    procedure ExpectWhitespace;
    procedure ExpectString(const s: String);
    procedure ExpectChar(wc: WideChar);
    function  CheckForChar(c: WideChar): Boolean;

    procedure RaiseNameNotFound;
    function  CheckName(aFlags: TCheckNameFlags = []): Boolean;
    procedure CheckNCName;
    function  ExpectName: WideString;                                   // [5]
<<<<<<< HEAD
<<<<<<< HEAD
    function ParseLiteral(var ToFill: TWideCharBuf; aType: TLiteralType;
      Required: Boolean): Boolean;
    procedure ExpectAttValue(attrData: PNodeData; NonCDATA: Boolean);   // [10]
    procedure ParseComment(discard: Boolean);                           // [15]
=======
=======
>>>>>>> origin/fixes_2_2
    procedure SkipQuotedLiteral(out Literal: WideString; required: Boolean = True);
    procedure ExpectAttValue;                                           // [10]
    procedure SkipPubidLiteral(out Literal: WideString);                // [12]
    procedure ParseComment;                                             // [15]
>>>>>>> graemeg/fixes_2_2
    procedure ParsePI;                                                  // [16]
    function CreatePINode: TDOMNode;
    procedure ParseXmlOrTextDecl(TextDecl: Boolean);
    procedure ExpectEq;
    procedure ParseDoctypeDecl;                                         // [28]
    procedure ParseMarkupDecl;                                          // [29]
<<<<<<< HEAD
    procedure ParseStartTag;                                            // [39]
    procedure ParseEndTag;                                              // [42]
    function DoStartElement: TDOMElement;
    procedure HandleEntityStart;
    procedure HandleEntityEnd;
    procedure ResolveEntity;
    procedure DoStartEntity;
    procedure ParseAttribute(ElDef: TElementDecl);
    procedure ParseContent(parent: TDOMNode_WithChildren);              // [43]
    function  Read: Boolean;
    function  ResolvePredefined: Boolean;
    function  EntityCheck(NoExternals: Boolean = False): TEntityDecl;
    procedure LoadEntity(AEntity: TEntityDecl);
    function PrefetchEntity(AEntity: TEntityDecl): Boolean;
    procedure StartPE;
    function  ParseRef(var ToFill: TWideCharBuf): Boolean;              // [67]
=======
    procedure ParseElement;                                             // [39]
    procedure ParseAttribute(Elem: TDOMElement; ElDef: TDOMElementDef);
    procedure ParseContent;                                             // [43]
    function  ResolvePredefined: Boolean;
    procedure IncludeEntity(InAttr: Boolean);
    procedure StartPE;
    function  ParseCharRef(var ToFill: TWideCharBuf): Boolean;        // [66]
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function  ParseExternalID(out SysID, PubID: WideString;             // [75]
      SysIdOptional: Boolean): Boolean;

    procedure BadPENesting(S: TErrorSeverity = esError);
    procedure ParseEntityDecl;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure ParseAttlistDecl;
    procedure ExpectChoiceOrSeq(CP: TContentParticle; MustEndIn: TObject);
    procedure ParseElementDecl;
    procedure ParseNotationDecl;
    function ResolveResource(const ASystemID, APublicID, ABaseURI: WideString; out Source: TXMLCharSource): Boolean;
    procedure ProcessDefaultAttributes(ElDef: TElementDecl);
    procedure ProcessNamespaceAtts;
    function AddBinding(attrData: PNodeData): Boolean;

    procedure PushVC(aElDef: TElementDecl);
    procedure PopVC;
    procedure ValidateDTD;
    procedure ValidateCurrentNode;
    procedure ValidationError(const Msg: string; const args: array of const; LineOffs: Integer = -1);
    procedure ValidationErrorWithName(const Msg: string; LineOffs: Integer = -1);
    procedure DTDReloadHook;
    procedure ConvertSource(SrcIn: TXMLInputSource; out SrcOut: TXMLCharSource);
    function DoCDSect(ch: PWideChar; Count: Integer): TDOMNode;
=======
=======
>>>>>>> origin/fixes_2_2
    function  ParseEntityDeclValue(Delim: WideChar): Boolean;
    procedure ParseAttlistDecl;
    procedure ExpectChoiceOrSeq(CP: TContentParticle);
    procedure ParseElementDecl;
    procedure ParseNotationDecl;
    function ResolveEntity(const SystemID, PublicID: WideString; out Source: TXMLCharSource): Boolean;
    procedure ProcessDefaultAttributes(Element: TDOMElement; ElDef: TDOMElementDef);

    procedure PushVC(aElDef: TDOMElementDef);
    procedure PopVC;
    procedure UpdateConstraints;
    procedure ValidateDTD;
    procedure ValidateRoot;
    procedure ValidationError(const Msg: string; const args: array of const; LineOffs: Integer = -1);
    procedure DoAttrText(ch: PWideChar; Count: Integer);
    procedure DTDReloadHook;
    procedure ConvertSource(SrcIn: TXMLInputSource; out SrcOut: TXMLCharSource);
    // Some SAX-alike stuff (at a very early stage)
    procedure DoText(ch: PWideChar; Count: Integer; Whitespace: Boolean=False);
    procedure DoComment(ch: PWideChar; Count: Integer);
    procedure DoCDSect(ch: PWideChar; Count: Integer);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure DoNotationDecl(const aName, aPubID, aSysID: WideString);
  public
    doc: TDOMDocument;
    constructor Create; overload;
    constructor Create(AParser: TDOMParser); overload;
    destructor Destroy; override;
    procedure ProcessXML(ASource: TXMLCharSource);                // [1]
    procedure ProcessFragment(ASource: TXMLCharSource; AOwner: TDOMNode);
    procedure ProcessDTD(ASource: TXMLCharSource);               // ([29])
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  end;

  // Attribute/Element declarations

  TDOMAttrDef = class(TDOMAttr)
  private
    FTag: Cardinal;
  protected
    FExternallyDeclared: Boolean;
    FDefault: TAttrDefault;
    FEnumeration: array of WideString;
    function AddEnumToken(Buf: DOMPChar; Len: Integer): Boolean;
    function HasEnumToken(const aValue: WideString): Boolean;
    function Clone(AElement: TDOMElement): TDOMAttr;
  public
    property Tag: Cardinal read FTag write FTag;
<<<<<<< HEAD
=======
  end;

  TDOMElementDef = class(TDOMElement)
  public
    FExternallyDeclared: Boolean;
    ContentType: TElementContentType;
    HasElementDecl: Boolean;
    IDAttr: TDOMAttrDef;
    NotationAttr: TDOMAttrDef;
    RootCP: TContentParticle;
    destructor Destroy; override;
  end;

const
  NullLocation: TLocation = (Line: 0; LinePos: 0);

function Decode_UCS2(Src: TXMLDecodingSource): WideChar;
begin
  Result := PWideChar(Src.FCharBuf)^;
  Inc(Src.FCharBuf, sizeof(WideChar));
end;

function Decode_UCS2_Swapped(Src: TXMLDecodingSource): WideChar;
begin
  Result := WideChar((ord(Src.FCharBuf^) shl 8) or ord(Src.FCharBuf[1]));
  Inc(Src.FCharBuf, sizeof(WideChar));
end;


function Decode_UTF8_mb(Src: TXMLDecodingSource; First: WideChar): WideChar;
const
  MaxCode: array[0..3] of Cardinal = ($7F, $7FF, $FFFF, $1FFFFF);
var
  Value: Cardinal;
  I, bc: Integer;
begin
  if ord(First) and $40 = 0 then
    Src.DecodingError('Invalid UTF-8 sequence start byte');
  bc := 1;
  if ord(First) and $20 <> 0 then
  begin
    Inc(bc);
    if ord(First) and $10 <> 0 then
    begin
      Inc(bc);
      if ord(First) and $8 <> 0 then
        Src.DecodingError('UCS4 character out of supported range');
    end;
  end;
  // DONE: (?) check that bc bytes available
  if Src.FCharBufEnd-Src.FCharBuf < bc then
    Src.FetchData;

  Value := ord(First);
  I := bc;  // note: I is never zero
  while bc > 0 do
  begin
    if Src.FCharBuf^ in [#$80..#$BF] then
      Value := (Value shl 6) or (Cardinal(Src.FCharBuf^) and $3F)
    else
      Src.DecodingError('Invalid byte in UTF-8 sequence');
    Inc(Src.FCharBuf);
    Dec(bc);
  end;
  Value := Value and MaxCode[I];
  // RFC2279 check
  if Value <= MaxCode[I-1] then
    Src.DecodingError('Invalid UTF-8 sequence');
  case Value of
    0..$D7FF, $E000..$FFFF:
      begin
        Result := WideChar(Value);
        Exit;
      end;
    $10000..$10FFFF:
      begin
        Result := WideChar($D7C0 + (Value shr 10));
        Src.FSurrogate := WideChar($DC00 xor (Value and $3FF));
        Exit;
      end;
>>>>>>> origin/fixes_2_2
  end;
  Src.DecodingError('UCS4 character out of supported range');
  Result := #0; // supress warning
end;

<<<<<<< HEAD
  TDOMElementDef = class(TDOMElement)
  public
    FExternallyDeclared: Boolean;
    ContentType: TElementContentType;
    HasElementDecl: Boolean;
    IDAttr: TDOMAttrDef;
    NotationAttr: TDOMAttrDef;
    RootCP: TContentParticle;
    destructor Destroy; override;
  end;

const
  NullLocation: TLocation = (Line: 0; LinePos: 0);

function Decode_UCS2(Src: TXMLDecodingSource): WideChar;
begin
  Result := PWideChar(Src.FCharBuf)^;
  Inc(Src.FCharBuf, sizeof(WideChar));
=======
function Decode_UTF8(Src: TXMLDecodingSource): WideChar;
begin
  Result := WideChar(byte(Src.FCharBuf^));
  Inc(Src.FCharBuf);
  if Result >= #$80 then
    Result := Decode_UTF8_mb(Src, Result);
end;

function Decode_8859_1(Src: TXMLDecodingSource): WideChar;
begin
  Result := WideChar(ord(Src.FCharBuf^));
  Inc(Src.FCharBuf);
end;

function Is_8859_1(const AEncoding: string): Boolean;
begin
  Result := SameText(AEncoding, 'ISO-8859-1') or
            SameText(AEncoding, 'ISO_8859-1') or
            SameText(AEncoding, 'latin1') or
            SameText(AEncoding, 'iso-ir-100') or
            SameText(AEncoding, 'l1') or
            SameText(AEncoding, 'IBM819') or
            SameText(AEncoding, 'CP819') or
            SameText(AEncoding, 'csISOLatin1') or
// This one is not in character-sets.txt, but used in most FPC documentation...
            SameText(AEncoding, 'ISO8859-1');
end;

// TODO: List of registered/supported decoders
function FindDecoder(const Encoding: string): TDecoder;
begin
  if Is_8859_1(Encoding) then
    Result := @Decode_8859_1
  else
    Result := nil;
>>>>>>> origin/fixes_2_2
end;

function Decode_UCS2_Swapped(Src: TXMLDecodingSource): WideChar;
begin
<<<<<<< HEAD
  Result := WideChar((ord(Src.FCharBuf^) shl 8) or ord(Src.FCharBuf[1]));
  Inc(Src.FCharBuf, sizeof(WideChar));
end;


function Decode_UTF8_mb(Src: TXMLDecodingSource; First: WideChar): WideChar;
const
  MaxCode: array[0..3] of Cardinal = ($7F, $7FF, $FFFF, $1FFFFF);
var
  Value: Cardinal;
  I, bc: Integer;
=======
  ABuffer.MaxLength := ALength;
  ABuffer.Length := 0;
  ABuffer.Buffer := AllocMem(ABuffer.MaxLength*SizeOf(WideChar));
end;

procedure BufAppend(var ABuffer: TWideCharBuf; wc: WideChar);
>>>>>>> origin/fixes_2_2
begin
  if ord(First) and $40 = 0 then
    Src.DecodingError('Invalid UTF-8 sequence start byte');
  bc := 1;
  if ord(First) and $20 <> 0 then
  begin
<<<<<<< HEAD
    Inc(bc);
    if ord(First) and $10 <> 0 then
    begin
      Inc(bc);
      if ord(First) and $8 <> 0 then
        Src.DecodingError('UCS4 character out of supported range');
    end;
=======
    ReallocMem(ABuffer.Buffer, ABuffer.MaxLength * 2 * SizeOf(WideChar));
    FillChar(ABuffer.Buffer[ABuffer.MaxLength], ABuffer.MaxLength * SizeOf(WideChar),0);
    ABuffer.MaxLength := ABuffer.MaxLength * 2;
>>>>>>> origin/fixes_2_2
  end;
  // DONE: (?) check that bc bytes available
  if Src.FCharBufEnd-Src.FCharBuf < bc then
    Src.FetchData;

  Value := ord(First);
  I := bc;  // note: I is never zero
  while bc > 0 do
  begin
    if Src.FCharBuf^ in [#$80..#$BF] then
      Value := (Value shl 6) or (Cardinal(Src.FCharBuf^) and $3F)
    else
      Src.DecodingError('Invalid byte in UTF-8 sequence');
    Inc(Src.FCharBuf);
    Dec(bc);
  end;
  Value := Value and MaxCode[I];
  // RFC2279 check
  if Value <= MaxCode[I-1] then
    Src.DecodingError('Invalid UTF-8 sequence');
  case Value of
    0..$D7FF, $E000..$FFFF:
      begin
        Result := WideChar(Value);
        Exit;
      end;
    $10000..$10FFFF:
      begin
        Result := WideChar($D7C0 + (Value shr 10));
        Src.FSurrogate := WideChar($DC00 xor (Value and $3FF));
        Exit;
      end;
>>>>>>> graemeg/fixes_2_2
  end;
  Src.DecodingError('UCS4 character out of supported range');
  Result := #0; // supress warning
end;

<<<<<<< HEAD
<<<<<<< HEAD
const
  NullLocation: TLocation = (Line: 0; LinePos: 0);
=======
function Decode_UTF8(Src: TXMLDecodingSource): WideChar;
begin
  Result := WideChar(byte(Src.FCharBuf^));
  Inc(Src.FCharBuf);
  if Result >= #$80 then
    Result := Decode_UTF8_mb(Src, Result);
end;

function Decode_8859_1(Src: TXMLDecodingSource): WideChar;
begin
  Result := WideChar(ord(Src.FCharBuf^));
  Inc(Src.FCharBuf);
end;

function Is_8859_1(const AEncoding: string): Boolean;
begin
  Result := SameText(AEncoding, 'ISO-8859-1') or
            SameText(AEncoding, 'ISO_8859-1') or
            SameText(AEncoding, 'latin1') or
            SameText(AEncoding, 'iso-ir-100') or
            SameText(AEncoding, 'l1') or
            SameText(AEncoding, 'IBM819') or
            SameText(AEncoding, 'CP819') or
            SameText(AEncoding, 'csISOLatin1') or
// This one is not in character-sets.txt, but used in most FPC documentation...
            SameText(AEncoding, 'ISO8859-1');
end;

// TODO: List of registered/supported decoders
function FindDecoder(const Encoding: string): TDecoder;
begin
  if Is_8859_1(Encoding) then
    Result := @Decode_8859_1
  else
    Result := nil;
=======
procedure BufAppendChunk(var ABuf: TWideCharBuf; p: PWideChar; Len: Integer);
begin
  if Len + ABuf.Length >= ABuf.MaxLength then
  begin
    ABuf.MaxLength := (Len + ABuf.Length)*2;
    // note: memory clean isn't necessary here.
    // To avoid garbage, control Length field.
    ReallocMem(ABuf.Buffer, ABuf.MaxLength * sizeof(WideChar));
  end;
  Move(p^, ABuf.Buffer[ABuf.Length], Len * sizeof(WideChar));
  Inc(ABuf.Length, Len);
end;

function BufEquals(const ABuf: TWideCharBuf; const Arg: WideString): Boolean;
begin
  Result := (ABuf.Length = Length(Arg)) and
    CompareMem(ABuf.Buffer, Pointer(Arg), ABuf.Length*sizeof(WideChar));
end;

{ TXMLInputSource }

constructor TXMLInputSource.Create(AStream: TStream);
begin
  inherited Create;
  FStream := AStream;
end;

constructor TXMLInputSource.Create(const AStringData: string);
begin
  inherited Create;
  FStringData := AStringData;
>>>>>>> origin/fixes_2_2
end;
>>>>>>> graemeg/fixes_2_2

<<<<<<< HEAD
{ Decoders }

<<<<<<< HEAD
var
  Decoders: array of TGetDecoderProc;

procedure RegisterDecoder(Proc: TGetDecoderProc);
var
  L: Integer;
begin
  L := Length(Decoders);
  SetLength(Decoders, L+1);
  Decoders[L] := Proc;
end;

function FindDecoder(const AEncoding: string; out Decoder: TDecoder): Boolean;
var
  I: Integer;
begin
  Result := False;
  for I := 0 to High(Decoders) do
    if Decoders[I](AEncoding, Decoder) then
    begin
      Result := True;
      Exit;
    end;
end;


function Is_8859_1(const AEncoding: string): Boolean;
begin
  Result := SameText(AEncoding, 'ISO-8859-1') or
            SameText(AEncoding, 'ISO_8859-1') or
            SameText(AEncoding, 'latin1') or
            SameText(AEncoding, 'iso-ir-100') or
            SameText(AEncoding, 'l1') or
            SameText(AEncoding, 'IBM819') or
            SameText(AEncoding, 'CP819') or
            SameText(AEncoding, 'csISOLatin1') or
// This one is not in character-sets.txt, but was used in FPC documentation,
// and still being used in fcl-registry package
            SameText(AEncoding, 'ISO8859-1');
end;


{ TDOMParseOptions }

function TDOMParseOptions.GetCanonical: Boolean;
begin
  Result := FCanonical and FExpandEntities and FCDSectionsAsText and
  { (not normalizeCharacters) and } FNamespaces and
  { namespaceDeclarations and } FPreserveWhitespace;
end;

procedure TDOMParseOptions.SetCanonical(aValue: Boolean);
begin
  FCanonical := aValue;
  if aValue then
  begin
    FExpandEntities := True;
    FCDSectionsAsText := True;
    FNamespaces := True;
    FPreserveWhitespace := True;
    { normalizeCharacters := False; }
    { namespaceDeclarations := True; }
    { wellFormed := True; }
  end;
end;

{ TXMLInputSource }

constructor TXMLInputSource.Create(AStream: TStream);
begin
  inherited Create;
  FStream := AStream;
end;

constructor TXMLInputSource.Create(const AStringData: string);
begin
  inherited Create;
  FStringData := AStringData;
end;

{ TDOMParser }

constructor TDOMParser.Create;
begin
  FOptions := TDOMParseOptions.Create;
end;

destructor TDOMParser.Destroy;
=======
{ TDOMParser }

constructor TDOMParser.Create;
begin
  FOptions := TDOMParseOptions.Create;
end;

destructor TDOMParser.Destroy;
begin
  FOptions.Free;
  inherited Destroy;
end;

procedure TDOMParser.Parse(Src: TXMLInputSource; out ADoc: TXMLDocument);
var
  InputSrc: TXMLCharSource;
begin
  with TXMLReader.Create(Self) do
  try
    ConvertSource(Src, InputSrc);  // handles 'no-input-specified' case
    ProcessXML(InputSrc)
  finally
    ADoc := TXMLDocument(doc);
    Free;
  end;
end;

procedure TDOMParser.ParseUri(const URI: WideString; out ADoc: TXMLDocument);
var
  Src: TXMLCharSource;
begin
  ADoc := nil;
  with TXMLReader.Create(Self) do
  try
    if ResolveEntity(URI, '', Src) then
      ProcessXML(Src)
    else
      DoErrorPos(esFatal, 'The specified URI could not be resolved', NullLocation);
  finally
    ADoc := TXMLDocument(doc);
    Free;
  end;
end;

function TDOMParser.ParseWithContext(Src: TXMLInputSource;
  Context: TDOMNode; Action: TXMLContextAction): TDOMNode;
var
  InputSrc: TXMLCharSource;
  Frag: TDOMDocumentFragment;
  node: TDOMNode;
begin
  if Action in [xaInsertBefore, xaInsertAfter, xaReplace] then
    node := Context.ParentNode
  else
    node := Context;
  // TODO: replacing document isn't yet supported
  if (Action = xaReplaceChildren) and (node.NodeType = DOCUMENT_NODE) then
    raise EDOMNotSupported.Create('DOMParser.ParseWithContext');

  if not (node.NodeType in [ELEMENT_NODE, DOCUMENT_FRAGMENT_NODE]) then
    raise EDOMHierarchyRequest.Create('DOMParser.ParseWithContext');

  with TXMLReader.Create(Self) do
  try
    ConvertSource(Src, InputSrc);    // handles 'no-input-specified' case
    Frag := Context.OwnerDocument.CreateDocumentFragment;
    try
      ProcessFragment(InputSrc, Frag);
      Result := Frag.FirstChild;
      case Action of
        xaAppendAsChildren: Context.AppendChild(Frag);

        xaReplaceChildren: begin
          Context.TextContent := '';     // removes children
          Context.ReplaceChild(Frag, Context.FirstChild);
        end;
        xaInsertBefore: node.InsertBefore(Frag, Context);
        xaInsertAfter:  node.InsertBefore(Frag, Context.NextSibling);
        xaReplace:      node.ReplaceChild(Frag, Context);
      end;
    finally
      Frag.Free;
    end;
  finally
    Free;
  end;
end;

// TODO: These classes still cannot be considered as the final solution...

{ TXMLInputSource }

constructor TXMLCharSource.Create(const AData: WideString);
begin
  inherited Create;
  FLineNo := 1;
  FBuf := PWideChar(AData);
  FBufEnd := FBuf + Length(AData);
  LFPos := FBuf-1;
end;

procedure TXMLCharSource.Initialize;
begin
end;

function TXMLCharSource.NextChar: WideChar;
begin
  Inc(FBuf);
  Result := FBuf^;
  if Result = #0 then
  begin
    if FBuf < FBufEnd then
      Exit;
    if Reload then
      Result := FBuf^;
  end;
end;

function TXMLCharSource.SetEncoding(const AEncoding: string): Boolean;
begin
  Result := True; // always succeed
end;

function TXMLCharSource.GetPublicID: WideString;
>>>>>>> origin/fixes_2_2
begin
  FOptions.Free;
  inherited Destroy;
end;

<<<<<<< HEAD
procedure TDOMParser.Parse(Src: TXMLInputSource; out ADoc: TXMLDocument);
var
  InputSrc: TXMLCharSource;
begin
  with TXMLReader.Create(Self) do
  try
    ConvertSource(Src, InputSrc);  // handles 'no-input-specified' case
    ProcessXML(InputSrc)
  finally
    ADoc := TXMLDocument(doc);
=======
procedure BufAllocate(var ABuffer: TWideCharBuf; ALength: Integer);
begin
  ABuffer.MaxLength := ALength;
  ABuffer.Length := 0;
  ABuffer.Buffer := AllocMem(ABuffer.MaxLength*SizeOf(WideChar));
end;

procedure BufAppend(var ABuffer: TWideCharBuf; wc: WideChar);
begin
  if ABuffer.Length >= ABuffer.MaxLength then
  begin
    ReallocMem(ABuffer.Buffer, ABuffer.MaxLength * 2 * SizeOf(WideChar));
    FillChar(ABuffer.Buffer[ABuffer.MaxLength], ABuffer.MaxLength * SizeOf(WideChar),0);
    ABuffer.MaxLength := ABuffer.MaxLength * 2;
  end;
  ABuffer.Buffer[ABuffer.Length] := wc;
  Inc(ABuffer.Length);
end;

procedure BufAppendChunk(var ABuf: TWideCharBuf; p: PWideChar; Len: Integer);
begin
  if Len + ABuf.Length >= ABuf.MaxLength then
  begin
    ABuf.MaxLength := (Len + ABuf.Length)*2;
    // note: memory clean isn't necessary here.
    // To avoid garbage, control Length field.
    ReallocMem(ABuf.Buffer, ABuf.MaxLength * sizeof(WideChar));
  end;
  Move(p^, ABuf.Buffer[ABuf.Length], Len * sizeof(WideChar));
  Inc(ABuf.Length, Len);
end;

function BufEquals(const ABuf: TWideCharBuf; const Arg: WideString): Boolean;
begin
  Result := (ABuf.Length = Length(Arg)) and
    CompareMem(ABuf.Buffer, Pointer(Arg), ABuf.Length*sizeof(WideChar));
=======
function TXMLCharSource.GetSystemID: WideString;
begin
  if FSystemID <> '' then
    Result := FSystemID
  else if Assigned(FParent) then
    Result := FParent.SystemID
  else
    Result := '';
end;

function TXMLCharSource.Reload: Boolean;
begin
  Result := False;
end;

function TXMLCharSource.Matches(const arg: WideString): Boolean;
begin
  Result := False;
  if (FBufEnd >= FBuf + Length(arg)) or Reload then
    Result := CompareMem(Pointer(arg), FBuf, Length(arg)*sizeof(WideChar));
  if Result then
    Inc(FBuf, Length(arg));
  FReader.FCurChar := FBuf^;
end;

{ TXMLDecodingSource }

procedure TXMLDecodingSource.AfterConstruction;
begin
  inherited AfterConstruction;
  FBufStart := AllocMem(4096);
  FBuf := FBufStart;
  FBufEnd := FBuf;
  LFPos := FBuf-1;
end;

destructor TXMLDecodingSource.Destroy;
begin
  FreeMem(FBufStart);
  inherited Destroy;
end;

procedure TXMLDecodingSource.FetchData;
begin
end;

procedure TXMLDecodingSource.DecodingError(const Msg: string);
begin
// count line endings to obtain correct error location
  while FBuf < FBufEnd do
  begin
    if FBuf^ = #10 then
    begin
      LFPos := FBuf;
      Inc(FLineNo);
    end;
    Inc(FBuf);
  end;
  FReader.FatalError(Msg);
end;

function TXMLDecodingSource.Reload: Boolean;
var
<<<<<<< HEAD
  c: WideChar;
  r: Integer;
=======
  Remainder: PtrInt;
  r, inLeft: Cardinal;
  rslt: Integer;
>>>>>>> origin/fixes_2.4
begin
  if DTDSubsetType = dsInternal then
    FReader.DTDReloadHook;
  Remainder := FBufEnd - FBuf;
  if Remainder > 0 then
    Move(FBuf^, FBufStart^, Remainder * sizeof(WideChar));
  Dec(LFPos, FBuf-FBufStart);
  FBuf := FBufStart;
  FBufEnd := FBufStart + Remainder;

  while FBufEnd < FBufStart + FBufSize do
  begin
    if FCharBufEnd <= FCharBuf then
    begin
      FetchData;
      if FCharBufEnd <= FCharBuf then
        Break;
    end;
    if FSurrogate <> #0 then
    begin
      c := FSurrogate;
      FSurrogate := #0;
    end
    else
    begin
      c := FDecoder(Self);
      case c of
      #9: ;
      #10: if FSeenCR then
           begin
             FSeenCR := False;
             Continue;
           end;
      #13: begin
             FSeenCR := True;
             c := #10;
           end;
      #$85, #$2028: if FXML11Rules then
           begin
             if FSeenCR and (c = #$85) then
             begin
               FSeenCR := False;
               Continue;
             end;
             c := #10;
           end;
      else
        if (c < #32) or (c >= #$FFFE) or
         (FXML11Rules and (c >= #$7F) and (c <= #$9F)) then
        DecodingError('Invalid character');
      end; //case
    end;

    FBufEnd^ := c;
    Inc(FBufEnd);
  end;
  FBufEnd^ := #0;
  Result := FBuf < FBufEnd;
end;

const
  XmlSign: array [0..4] of WideChar = ('<', '?', 'x', 'm', 'l');

procedure TXMLDecodingSource.Initialize;
begin
  inherited;
  FLineNo := 1;
  FXml11Rules := FReader.FXML11;
  FDecoder := @Decode_UTF8;
  FFixedUCS2 := '';
  if FCharBufEnd-FCharBuf > 1 then
  begin
    if (FCharBuf[0] = #$FE) and (FCharBuf[1] = #$FF) then
    begin
      FFixedUCS2 := 'UTF-16BE';
      FDecoder := {$IFNDEF ENDIAN_BIG} @Decode_UCS2_Swapped {$ELSE} @Decode_UCS2 {$ENDIF};
    end
    else if (FCharBuf[0] = #$FF) and (FCharBuf[1] = #$FE) then
    begin
      FFixedUCS2 := 'UTF-16LE';
      FDecoder := {$IFDEF ENDIAN_BIG} @Decode_UCS2_Swapped {$ELSE} @Decode_UCS2 {$ENDIF};
    end;
  end;
  FBufSize := 6;             //  possible BOM and '<?xml'
  Reload;
  if FBuf^ = #$FEFF then
    Inc(FBuf);
  LFPos := FBuf-1;
  if CompareMem(FBuf, @XmlSign[0], sizeof(XmlSign)) then
  begin
    FBufSize := 3;           // don't decode past XML declaration
    Inc(FBuf, 4);
    FReader.ParseXmlOrTextDecl(FParent <> nil);
  end;
  FBufSize := 2047;
end;

function TXMLDecodingSource.SetEncoding(const AEncoding: string): Boolean;
var
  NewDecoder: TDecoder;
begin
  Result := True;
  if (FFixedUCS2 = '') and SameText(AEncoding, 'UTF-8') then
    Exit;
  if FFixedUCS2 <> '' then
  begin
    Result := SameText(AEncoding, FFixedUCS2) or
       SameText(AEncoding, 'UTF-16') or
       SameText(AEncoding, 'unicode');
    Exit;
  end;
  NewDecoder := FindDecoder(AEncoding);
  if Assigned(NewDecoder) then
    FDecoder := NewDecoder
  else
    Result := False;
>>>>>>> origin/fixes_2_2
end;

{ TXMLInputSource }

<<<<<<< HEAD
constructor TXMLInputSource.Create(AStream: TStream);
=======
{ TXMLStreamInputSource }

const
  Slack = 16;

constructor TXMLStreamInputSource.Create(AStream: TStream; AOwnStream: Boolean);
>>>>>>> origin/fixes_2_2
begin
  inherited Create;
  FStream := AStream;
<<<<<<< HEAD
=======
  FCapacity := 4096;
  GetMem(FAllocated, FCapacity+Slack);
  FCharBuf := FAllocated+(Slack-4);
  FCharBufEnd := FCharBuf;
  FOwnStream := AOwnStream;
  FetchData;
end;

destructor TXMLStreamInputSource.Destroy;
begin
  FreeMem(FAllocated);
  if FOwnStream then
    FStream.Free;
  inherited Destroy;
>>>>>>> origin/fixes_2_2
end;

constructor TXMLInputSource.Create(const AStringData: string);
begin
<<<<<<< HEAD
  inherited Create;
  FStringData := AStringData;
=======
  Assert(FCharBufEnd - FCharBuf < Slack-4);

  OldBuf := FCharBuf;
  Remainder := FCharBufEnd - FCharBuf;
  if Remainder < 0 then
    Remainder := 0;
  FCharBuf := FAllocated+Slack-4-Remainder;
  if Remainder > 0 then
    Move(OldBuf^, FCharBuf^, Remainder);
  BytesRead := FStream.Read(FAllocated[Slack-4], FCapacity);
  FCharBufEnd := FAllocated + (Slack-4) + BytesRead;
  Unaligned(PWideChar(FCharBufEnd)^) := #0;
>>>>>>> origin/fixes_2_2
end;

{ TDOMParser }

constructor TDOMParser.Create;
begin
<<<<<<< HEAD
  FOptions := TDOMParseOptions.Create;
=======
  FFile := @AFile;
  FetchData;
>>>>>>> origin/fixes_2_2
end;

destructor TDOMParser.Destroy;
begin
<<<<<<< HEAD
  FOptions.Free;
  inherited Destroy;
end;

procedure TDOMParser.Parse(Src: TXMLInputSource; out ADoc: TXMLDocument);
var
  InputSrc: TXMLCharSource;
begin
  with TXMLReader.Create(Self) do
  try
    ConvertSource(Src, InputSrc);  // handles 'no-input-specified' case
    ProcessXML(InputSrc)
  finally
    ADoc := TXMLDocument(doc);
    Free;
  end;
end;

procedure TDOMParser.ParseUri(const URI: WideString; out ADoc: TXMLDocument);
var
  Src: TXMLCharSource;
begin
  ADoc := nil;
  with TXMLReader.Create(Self) do
  try
    if ResolveEntity(URI, '', Src) then
      ProcessXML(Src)
    else
      DoErrorPos(esFatal, 'The specified URI could not be resolved', NullLocation);
  finally
    ADoc := TXMLDocument(doc);
    Free;
=======
  if not Eof(FFile^) then
  begin
    ReadLn(FFile^, FString);
    FString := FString + #10;    // bad solution...
    FCharBuf := PChar(FString);
    FCharBufEnd := FCharBuf + Length(FString);
  end;
end;

{ helper that closes handle upon destruction }
type
  THandleOwnerStream = class(THandleStream)
  public
    destructor Destroy; override;
>>>>>>> origin/fixes_2_2
  end;

<<<<<<< HEAD
function TDOMParser.ParseWithContext(Src: TXMLInputSource;
  Context: TDOMNode; Action: TXMLContextAction): TDOMNode;
var
  InputSrc: TXMLCharSource;
  Frag: TDOMDocumentFragment;
  node: TDOMNode;
begin
  if Action in [xaInsertBefore, xaInsertAfter, xaReplace] then
    node := Context.ParentNode
  else
    node := Context;
  // TODO: replacing document isn't yet supported
  if (Action = xaReplaceChildren) and (node.NodeType = DOCUMENT_NODE) then
    raise EDOMNotSupported.Create('DOMParser.ParseWithContext');

  if not (node.NodeType in [ELEMENT_NODE, DOCUMENT_FRAGMENT_NODE]) then
    raise EDOMHierarchyRequest.Create('DOMParser.ParseWithContext');

  with TXMLReader.Create(Self) do
  try
    ConvertSource(Src, InputSrc);    // handles 'no-input-specified' case
    Frag := Context.OwnerDocument.CreateDocumentFragment;
    try
      ProcessFragment(InputSrc, Frag);
      Result := Frag.FirstChild;
      case Action of
        xaAppendAsChildren: Context.AppendChild(Frag);

        xaReplaceChildren: begin
          Context.TextContent := '';     // removes children
          Context.ReplaceChild(Frag, Context.FirstChild);
        end;
        xaInsertBefore: node.InsertBefore(Frag, Context);
        xaInsertAfter:  node.InsertBefore(Frag, Context.NextSibling);
        xaReplace:      node.ReplaceChild(Frag, Context);
      end;
    finally
      Frag.Free;
    end;
  finally
>>>>>>> graemeg/fixes_2_2
    Free;
  end;
end;

<<<<<<< HEAD
procedure TDOMParser.ParseUri(const URI: WideString; out ADoc: TXMLDocument);
var
  Src: TXMLCharSource;
begin
  ADoc := nil;
  with TXMLReader.Create(Self) do
  try
    if ResolveResource(URI, '', '', Src) then
      ProcessXML(Src)
    else
      DoErrorPos(esFatal, 'The specified URI could not be resolved', NullLocation);
  finally
    ADoc := TXMLDocument(doc);
    Free;
  end;
end;

function TDOMParser.ParseWithContext(Src: TXMLInputSource;
  Context: TDOMNode; Action: TXMLContextAction): TDOMNode;
var
  InputSrc: TXMLCharSource;
  Frag: TDOMDocumentFragment;
  node: TDOMNode;
begin
  if Action in [xaInsertBefore, xaInsertAfter, xaReplace] then
    node := Context.ParentNode
  else
    node := Context;
  // TODO: replacing document isn't yet supported
  if (Action = xaReplaceChildren) and (node.NodeType = DOCUMENT_NODE) then
    raise EDOMNotSupported.Create('DOMParser.ParseWithContext');

  if not (node.NodeType in [ELEMENT_NODE, DOCUMENT_FRAGMENT_NODE]) then
    raise EDOMHierarchyRequest.Create('DOMParser.ParseWithContext');

  with TXMLReader.Create(Self) do
  try
    ConvertSource(Src, InputSrc);    // handles 'no-input-specified' case
    Frag := Context.OwnerDocument.CreateDocumentFragment;
    try
      ProcessFragment(InputSrc, Frag);
      Result := Frag.FirstChild;
      case Action of
        xaAppendAsChildren: Context.AppendChild(Frag);

        xaReplaceChildren: begin
          Context.TextContent := '';     // removes children
          Context.ReplaceChild(Frag, Context.FirstChild);
        end;
        xaInsertBefore: node.InsertBefore(Frag, Context);
        xaInsertAfter:  node.InsertBefore(Frag, Context.NextSibling);
        xaReplace:      node.ReplaceChild(Frag, Context);
      end;
    finally
      Frag.Free;
    end;
  finally
    Free;
=======
destructor THandleOwnerStream.Destroy;
begin
  if Handle >= 0 then FileClose(Handle);
  inherited Destroy;
end;

{ TXMLReader }

procedure TXMLReader.ConvertSource(SrcIn: TXMLInputSource; out SrcOut: TXMLCharSource);
begin
  SrcOut := nil;
  if Assigned(SrcIn) then
  begin
    if Assigned(SrcIn.FStream) then
      SrcOut := TXMLStreamInputSource.Create(SrcIn.FStream, False)
    else if SrcIn.FStringData <> '' then
      SrcOut := TXMLStreamInputSource.Create(TStringStream.Create(SrcIn.FStringData), True)
    else if (SrcIn.SystemID <> '') then
      ResolveEntity(SrcIn.SystemID, SrcIn.PublicID, SrcOut);
  end;
  if (SrcOut = nil) and (FSource = nil) then
    DoErrorPos(esFatal, 'No input source specified', NullLocation);
end;

procedure TXMLReader.StoreLocation(out Loc: TLocation);
begin
  Loc.Line := FSource.FLineNo;
  Loc.LinePos := FSource.FBuf-FSource.LFPos;
end;

function TXMLReader.ResolveEntity(const SystemID, PublicID: WideString; out Source: TXMLCharSource): Boolean;
var
  AbsSysID: WideString;
  Filename: string;
  Stream: TStream;
  fd: THandle;
begin
  Source := nil;
  Result := False;
  if not Assigned(FSource) then
    AbsSysID := SystemID
  else
    if not ResolveRelativeURI(FSource.SystemID, SystemID, AbsSysID) then
      Exit;
  { TODO: alternative resolvers
    These may be 'internal' resolvers or a handler set by application.
    Internal resolvers should probably produce a TStream
    ( so that internal classes need not be exported ).
    External resolver will produce TXMLInputSource that should be converted.
    External resolver must NOT be called for root entity.
    External resolver can return nil, in which case we do the default }
  if URIToFilename(AbsSysID, Filename) then
  begin
    fd := FileOpen(Filename, fmOpenRead + fmShareDenyWrite);
    if fd <> THandle(-1) then
    begin
      Stream := THandleOwnerStream.Create(fd);
      Source := TXMLStreamInputSource.Create(Stream, True);
      Source.SystemID := AbsSysID;    // <- Revisit: Really need absolute sysID?
      Source.PublicID := PublicID;
    end;
>>>>>>> origin/fixes_2_2
  end;
  Result := Assigned(Source);
end;

<<<<<<< HEAD
{ TXMLCharSource }

constructor TXMLCharSource.Create(const AData: WideString);
begin
  inherited Create;
  FLineNo := 1;
  FBuf := PWideChar(AData);
  FBufEnd := FBuf + Length(AData);
  LFPos := FBuf-1;
  FCharCount := Length(AData);
end;

procedure TXMLCharSource.Initialize;
begin
end;

function TXMLCharSource.SetEncoding(const AEncoding: string): Boolean;
=======
// TODO: These classes still cannot be considered as the final solution...

{ TXMLInputSource }

constructor TXMLCharSource.Create(const AData: WideString);
begin
  inherited Create;
  FLineNo := 1;
  FBuf := PWideChar(AData);
  FBufEnd := FBuf + Length(AData);
  LFPos := FBuf-1;
end;

procedure TXMLCharSource.Initialize;
begin
end;

function TXMLCharSource.NextChar: WideChar;
begin
  Inc(FBuf);
  Result := FBuf^;
  if Result = #0 then
  begin
    if FBuf < FBufEnd then
      Exit;
    if Reload then
      Result := FBuf^;
  end;
end;

function TXMLCharSource.SetEncoding(const AEncoding: string): Boolean;
begin
  Result := True; // always succeed
end;

function TXMLCharSource.GetPublicID: WideString;
>>>>>>> graemeg/fixes_2_2
begin
  Result := True; // always succeed
end;

function TXMLCharSource.GetSystemID: WideString;
begin
  if FSystemID <> '' then
    Result := FSystemID
  else if Assigned(FParent) then
    Result := FParent.SystemID
  else
    Result := '';
end;

function TXMLCharSource.Reload: Boolean;
begin
  Result := False;
end;

<<<<<<< HEAD
procedure TXMLCharSource.NewLine;
begin
  Inc(FLineNo);
  LFPos := FBuf;
end;

function TXMLCharSource.SkipUntil(var ToFill: TWideCharBuf; const Delim: TSetOfChar;
  wsflag: PBoolean): WideChar;
var
  old: PWideChar;
  nonws: Boolean;
begin
  old := FBuf;
  nonws := False;
  repeat
    if FBuf^ = #10 then
      NewLine;
    if (FBuf^ < #255) and (Char(ord(FBuf^)) in Delim) then
      Break;
    if (FBuf^ > #32) or not (Char(ord(FBuf^)) in [#32, #9, #10, #13]) then
      nonws := True;
    Inc(FBuf);
  until False;
  Result := FBuf^;
  BufAppendChunk(ToFill, old, FBuf);
  if Assigned(wsflag) then
    wsflag^ := wsflag^ or nonws;
end;

=======
>>>>>>> graemeg/fixes_2_2
function TXMLCharSource.Matches(const arg: WideString): Boolean;
begin
  Result := False;
  if (FBufEnd >= FBuf + Length(arg)) or Reload then
    Result := CompareMem(Pointer(arg), FBuf, Length(arg)*sizeof(WideChar));
  if Result then
<<<<<<< HEAD
=======
procedure TXMLReader.Initialize(ASource: TXMLCharSource);
begin
  FSource := ASource;
  FSource.FReader := Self;
  FSource.Initialize;
  FCurChar := FSource.FBuf^;
end;

procedure TXMLReader.GetChar;
begin
  if FCurChar = #10 then
  begin
    Inc(FSource.FLineNo);
    FSource.LFPos := FSource.FBuf;
  end;
  FCurChar := FSource.NextChar;
end;

procedure TXMLReader.RaiseExpectedQmark;
begin
  FatalError('Expected single or double quote');
end;

procedure TXMLReader.FatalError(Expected: WideChar);
begin
// FIX: don't output what is found - anything may be found, including exploits...
  FatalError('Expected "%1s"', [string(Expected)]);
end;

procedure TXMLReader.FatalError(const descr: String; LineOffs: Integer);
begin
  DoError(esFatal, descr, LineOffs);
end;

procedure TXMLReader.FatalError(const descr: string; const args: array of const; LineOffs: Integer);
begin
  DoError(esFatal, Format(descr, args), LineOffs);
end;

procedure TXMLReader.ValidationError(const Msg: string; const Args: array of const; LineOffs: Integer);
begin
  FDocNotValid := True;
  if FValidate then
    DoError(esError, Format(Msg, Args), LineOffs);
end;

procedure TXMLReader.DoError(Severity: TErrorSeverity; const descr: string; LineOffs: Integer);
var
  Loc: TLocation;
begin
  StoreLocation(Loc);
  if LineOffs >= 0 then
  begin
    Dec(Loc.LinePos, LineOffs);
    DoErrorPos(Severity, descr, Loc);
  end
  else
    DoErrorPos(Severity, descr, FTokenStart);
end;

procedure TXMLReader.DoErrorPos(Severity: TErrorSeverity; const descr: string; const ErrPos: TLocation);
var
  E: EXMLReadError;
begin
  if Assigned(FSource) then
    E := EXMLReadError.CreateFmt('In ''%s'' (line %d pos %d): %s', [FSource.SystemID, ErrPos.Line, ErrPos.LinePos, descr])
  else
    E := EXMLReadError.Create(descr);
  E.FSeverity := Severity;
  E.FErrorMessage := descr;
  E.FLine := ErrPos.Line;
  E.FLinePos := ErrPos.LinePos;
  CallErrorHandler(E);
  // No 'finally'! If user handler raises exception, control should not get here
  // and the exception will be freed in CallErrorHandler (below)
  E.Free;
end;

procedure TXMLReader.CallErrorHandler(E: EXMLReadError);
begin
  try
    if Assigned(FCtrl) and Assigned(FCtrl.FOnError) then
      FCtrl.FOnError(E);
    if E.Severity = esFatal then
      raise E;
  except
    if ExceptObject <> E then
      E.Free;
    raise;
  end;
end;

function TXMLReader.SkipWhitespace(PercentAloneIsOk: Boolean): Boolean;
begin
  Result := False;
  repeat
    Result := SkipS or Result;
    case FCurChar of
      #0: begin
        Result := True;
        if not ContextPop then
          Break;
      end;

      '%': begin
        if not FRecognizePE then
          Break;
// This is the only case where look-ahead is needed
        if FSource.FBuf > FSource.FBufEnd-2 then
          FSource.Reload;
        if (not PercentAloneIsOk) or
          (Byte(FSource.FBuf[1]) in NamingBitmap[FNamePages^[hi(Word(FSource.FBuf[1]))]]) or
          (FXML11 and (FSource.FBuf[1] >= #$D800) and (FSource.FBuf[1] <= #$DB7F)) then
        begin
          Inc(FSource.FBuf);    // skip '%'
          FCurChar := FSource.FBuf^;
          CheckName;
          ExpectChar(';');
          StartPE;
          Result := True;        // report whitespace upon entering the PE
        end
        else Break;
      end
    else
      Break;
    end;
  until False;
end;

function TXMLReader.SkipS(required: Boolean): Boolean;
begin
  Result := False;
  while (FCurChar = #32) or (FCurChar = #10) or (FCurChar = #9) or (FCurChar = #13) do
>>>>>>> origin/fixes_2_2
  begin
    Inc(FBuf, Length(arg));
    if FBuf >= FBufEnd then
      Reload;
  end;
<<<<<<< HEAD
=======
    Inc(FBuf, Length(arg));
  FReader.FCurChar := FBuf^;
>>>>>>> graemeg/fixes_2_2
end;

{ TXMLDecodingSource }
=======
  if not Result and required then
    FatalError('Expected whitespace');
end;

procedure TXMLReader.ExpectWhitespace;
begin
  if not SkipWhitespace then
    FatalError('Expected whitespace');
end;
>>>>>>> origin/fixes_2_2

procedure TXMLDecodingSource.AfterConstruction;
begin
<<<<<<< HEAD
  inherited AfterConstruction;
  FBufStart := AllocMem(4096);
  FBuf := FBufStart;
  FBufEnd := FBuf;
  LFPos := FBuf-1;
=======
  if FCurChar = wc then
    GetChar
  else
    FatalError(wc);
>>>>>>> origin/fixes_2_2
end;

destructor TXMLDecodingSource.Destroy;
begin
<<<<<<< HEAD
  FreeMem(FBufStart);
<<<<<<< HEAD
  if Assigned(FDecoder.Cleanup) then
    FDecoder.Cleanup(FDecoder.Context);
=======
>>>>>>> graemeg/fixes_2_2
  inherited Destroy;
=======
  for I := 1 to Length(s) do
  begin
    if FCurChar <> WideChar(ord(s[i])) then
      FatalError('Expected "%s"', [s], i-1);
    GetChar;
  end;
>>>>>>> origin/fixes_2_2
end;

procedure TXMLDecodingSource.FetchData;
begin
end;

<<<<<<< HEAD
procedure TXMLDecodingSource.DecodingError(const Msg: string);
begin
// count line endings to obtain correct error location
  while FBuf < FBufEnd do
  begin
<<<<<<< HEAD
    if (FBuf^ = #10) or (FBuf^ = #13) or (FXML11Rules and ((FBuf^ = #$85) or (FBuf^ = #$2028))) then
    begin
      if (FBuf^ = #13) and (FBuf < FBufEnd-1) and
      ((FBuf[1] = #10) or (FXML11Rules and (FBuf[1] = #$85))) then
        Inc(FBuf);
=======
    if FBuf^ = #10 then
    begin
>>>>>>> graemeg/fixes_2_2
      LFPos := FBuf;
      Inc(FLineNo);
    end;
    Inc(FBuf);
  end;
  FReader.FatalError(Msg);
end;
<<<<<<< HEAD

function TXMLDecodingSource.Reload: Boolean;
var
  Remainder: PtrInt;
  r, inLeft: Cardinal;
  rslt: Integer;
begin
  if Kind = skInternalSubset then
    FReader.DTDReloadHook;
  Remainder := FBufEnd - FBuf;
  if Remainder > 0 then
    Move(FBuf^, FBufStart^, Remainder * sizeof(WideChar));
  Dec(LFPos, FBuf-FBufStart);
  FBuf := FBufStart;
  FBufEnd := FBufStart + Remainder;

  repeat
    inLeft := FCharBufEnd - FCharBuf;
    if inLeft < 4 then                      // may contain an incomplete char
    begin
      FetchData;
      inLeft := FCharBufEnd - FCharBuf;
      if inLeft <= 0 then
        Break;
    end;
    r := FBufStart + FBufSize - FBufEnd;
    if r = 0 then
      Break;
    rslt := FDecoder.Decode(FDecoder.Context, FCharBuf, inLeft, FBufEnd, r);
    { Sanity checks: r and inLeft must not increase. }
    if inLeft + FCharBuf <= FCharBufEnd then
      FCharBuf := FCharBufEnd - inLeft
    else
      DecodingError('Decoder error: input byte count out of bounds');
    if r + FBufEnd <= FBufStart + FBufSize then
      FBufEnd := FBufStart + FBufSize - r
    else
      DecodingError('Decoder error: output char count out of bounds');

    if rslt = 0 then
      Break
    else if rslt < 0 then
      DecodingError('Invalid character in input stream')
    else
      FReader.CheckMaxChars(rslt);
  until False;

=======

function TXMLDecodingSource.Reload: Boolean;
var
  c: WideChar;
  r: Integer;
begin
  if DTDSubsetType = dsInternal then
    FReader.DTDReloadHook;
  r := FBufEnd - FBuf;
  if r > 0 then
    Move(FBuf^, FBufStart^, r * sizeof(WideChar));
  Dec(LFPos, FBuf-FBufStart);
  FBuf := FBufStart;
  FBufEnd := FBufStart + r;

  while FBufEnd < FBufStart + FBufSize do
  begin
    if FCharBufEnd <= FCharBuf then
    begin
      FetchData;
      if FCharBufEnd <= FCharBuf then
        Break;
    end;
    if FSurrogate <> #0 then
    begin
      c := FSurrogate;
      FSurrogate := #0;
    end
    else
    begin
      c := FDecoder(Self);
      case c of
      #9: ;
      #10: if FSeenCR then
           begin
             FSeenCR := False;
             Continue;
           end;
      #13: begin
             FSeenCR := True;
             c := #10;
           end;
      #$85, #$2028: if FXML11Rules then
           begin
             if FSeenCR and (c = #$85) then
             begin
               FSeenCR := False;
               Continue;
             end;
             c := #10;
           end;
      else
        if (c < #32) or (c >= #$FFFE) or
         (FXML11Rules and (c >= #$7F) and (c <= #$9F)) then
        DecodingError('Invalid character');
      end; //case
    end;

    FBufEnd^ := c;
    Inc(FBufEnd);
  end;
>>>>>>> graemeg/fixes_2_2
  FBufEnd^ := #0;
  Result := FBuf < FBufEnd;
=======
constructor TXMLReader.Create;
begin
  inherited Create;
  BufAllocate(FName, 128);
  BufAllocate(FValue, 512);
  FIDRefs := TFPList.Create;
  FNotationRefs := TFPList.Create;

  // Set char rules to XML 1.0
  FNamePages := @NamePages;
  SetLength(FValidator, 16);
end;

constructor TXMLReader.Create(AParser: TDOMParser);
begin
  Create;
  FCtrl := AParser;
  FValidate := FCtrl.Options.Validate;
  FPreserveWhitespace := FCtrl.Options.PreserveWhitespace;
  FExpandEntities := FCtrl.Options.ExpandEntities;
  FCDSectionsAsText := FCtrl.Options.CDSectionsAsText;
  FIgnoreComments := FCtrl.Options.IgnoreComments;
  FResolveExternals := FCtrl.Options.ResolveExternals;
  FNamespaces := FCtrl.Options.Namespaces;
>>>>>>> origin/fixes_2_2
end;

const
  XmlSign: array [0..4] of WideChar = ('<', '?', 'x', 'm', 'l');

procedure TXMLDecodingSource.Initialize;
begin
<<<<<<< HEAD
  inherited;
  FLineNo := 1;
<<<<<<< HEAD
  FDecoder.Decode := @Decode_UTF8;

=======
  FXml11Rules := FReader.FXML11;
  FDecoder := @Decode_UTF8;
>>>>>>> graemeg/fixes_2_2
  FFixedUCS2 := '';
  if FCharBufEnd-FCharBuf > 1 then
  begin
    if (FCharBuf[0] = #$FE) and (FCharBuf[1] = #$FF) then
    begin
      FFixedUCS2 := 'UTF-16BE';
<<<<<<< HEAD
      FDecoder.Decode := {$IFNDEF ENDIAN_BIG} @Decode_UCS2_Swapped {$ELSE} @Decode_UCS2 {$ENDIF};
=======
      FDecoder := {$IFNDEF ENDIAN_BIG} @Decode_UCS2_Swapped {$ELSE} @Decode_UCS2 {$ENDIF};
>>>>>>> graemeg/fixes_2_2
    end
    else if (FCharBuf[0] = #$FF) and (FCharBuf[1] = #$FE) then
    begin
      FFixedUCS2 := 'UTF-16LE';
<<<<<<< HEAD
      FDecoder.Decode := {$IFDEF ENDIAN_BIG} @Decode_UCS2_Swapped {$ELSE} @Decode_UCS2 {$ENDIF};
=======
      FDecoder := {$IFDEF ENDIAN_BIG} @Decode_UCS2_Swapped {$ELSE} @Decode_UCS2 {$ENDIF};
>>>>>>> graemeg/fixes_2_2
    end;
  end;
  FBufSize := 6;             //  possible BOM and '<?xml'
  Reload;
  if FBuf^ = #$FEFF then
<<<<<<< HEAD
  begin
    FHasBOM := True;
    Inc(FBuf);
  end;
=======
    Inc(FBuf);
>>>>>>> graemeg/fixes_2_2
  LFPos := FBuf-1;
  if CompareMem(FBuf, @XmlSign[0], sizeof(XmlSign)) then
  begin
    FBufSize := 3;           // don't decode past XML declaration
<<<<<<< HEAD
    Inc(FBuf, Length(XmlSign));
    FReader.ParseXmlOrTextDecl((FParent <> nil) or (FReader.FState <> rsProlog));
  end;
  FBufSize := 2047;
  if FReader.FXML11 then
    FReader.XML11_BuildTables;
=======
    Inc(FBuf, 4);
    FReader.ParseXmlOrTextDecl(FParent <> nil);
  end;
  FBufSize := 2047;
>>>>>>> graemeg/fixes_2_2
end;

function TXMLDecodingSource.SetEncoding(const AEncoding: string): Boolean;
var
  NewDecoder: TDecoder;
begin
  Result := True;
  if (FFixedUCS2 = '') and SameText(AEncoding, 'UTF-8') then
    Exit;
  if FFixedUCS2 <> '' then
  begin
    Result := SameText(AEncoding, FFixedUCS2) or
       SameText(AEncoding, 'UTF-16') or
       SameText(AEncoding, 'unicode');
    Exit;
  end;
<<<<<<< HEAD
// TODO: must fail when a byte-based stream is labeled as word-based.
// see rmt-e2e-61, it now fails but for a completely different reason.
  FillChar(NewDecoder, sizeof(TDecoder), 0);
  if Is_8859_1(AEncoding) then
    FDecoder.Decode := @Decode_8859_1
  else if FindDecoder(AEncoding, NewDecoder) then
=======
  NewDecoder := FindDecoder(AEncoding);
  if Assigned(NewDecoder) then
>>>>>>> graemeg/fixes_2_2
    FDecoder := NewDecoder
  else
    Result := False;
end;

procedure TXMLDecodingSource.NewLine;
begin
  case FBuf^ of
    #10: ;
    #13: begin
      // Reload trashes the buffer, it should be consumed beforehand
      if (FBufEnd >= FBuf+2) or Reload then
      begin
        if (FBuf[1] = #10) or (FXML11Rules and (FBuf[1] = #$85)) then
          Inc(FBuf);
      end;
      FBuf^ := #10;
    end;
    #$85, #$2028: if FXML11Rules then
      FBuf^ := #10
    else
      Exit;
  else
    Exit;
  end;
  Inc(FLineNo);
  LFPos := FBuf;
end;

{ TXMLStreamInputSource }

const
  Slack = 16;

constructor TXMLStreamInputSource.Create(AStream: TStream; AOwnStream: Boolean);
begin
  FStream := AStream;
  FCapacity := 4096;
  GetMem(FAllocated, FCapacity+Slack);
  FCharBuf := FAllocated+(Slack-4);
  FCharBufEnd := FCharBuf;
  FOwnStream := AOwnStream;
  FetchData;
end;

destructor TXMLStreamInputSource.Destroy;
begin
  FreeMem(FAllocated);
  if FOwnStream then
    FStream.Free;
  inherited Destroy;
end;

procedure TXMLStreamInputSource.FetchData;
var
  Remainder, BytesRead: Integer;
  OldBuf: PChar;
begin
  Assert(FCharBufEnd - FCharBuf < Slack-4);
<<<<<<< HEAD
  if FEof then
    Exit;
=======

>>>>>>> graemeg/fixes_2_2
  OldBuf := FCharBuf;
  Remainder := FCharBufEnd - FCharBuf;
  if Remainder < 0 then
    Remainder := 0;
  FCharBuf := FAllocated+Slack-4-Remainder;
  if Remainder > 0 then
    Move(OldBuf^, FCharBuf^, Remainder);
  BytesRead := FStream.Read(FAllocated[Slack-4], FCapacity);
<<<<<<< HEAD
  if BytesRead < FCapacity then
    FEof := True;
  FCharBufEnd := FAllocated + (Slack-4) + BytesRead;
  { Null-termination has been removed:
    1) Built-in decoders don't need it because they respect the buffer length.
    2) It was causing unaligned access errors on ARM CPUs.
  }
  //PWideChar(FCharBufEnd)^ := #0;
=======
  FCharBufEnd := FAllocated + (Slack-4) + BytesRead;
  Unaligned(PWideChar(FCharBufEnd)^) := #0;
>>>>>>> graemeg/fixes_2_2
end;

{ TXMLFileInputSource }

constructor TXMLFileInputSource.Create(var AFile: Text);
begin
  FFile := @AFile;
<<<<<<< HEAD
  SystemID := FilenameToURI(TTextRec(AFile).Name);
=======
>>>>>>> graemeg/fixes_2_2
  FetchData;
end;

procedure TXMLFileInputSource.FetchData;
var
  Remainder: Integer;
begin
  if not Eof(FFile^) then
  begin
    Remainder := FCharBufEnd - FCharBuf;
    if Remainder > 0 then
      SetString(FTmp, FCharBuf, Remainder);
    ReadLn(FFile^, FString);
    FString := FString + #10;    // bad solution...
<<<<<<< HEAD
    if Remainder > 0 then
      Insert(FTmp, FString, 1);
=======
>>>>>>> graemeg/fixes_2_2
    FCharBuf := PChar(FString);
    FCharBufEnd := FCharBuf + Length(FString);
  end;
end;

{ helper that closes handle upon destruction }
type
  THandleOwnerStream = class(THandleStream)
  public
    destructor Destroy; override;
  end;

destructor THandleOwnerStream.Destroy;
begin
  if Handle >= 0 then FileClose(Handle);
  inherited Destroy;
end;

{ TXMLReader }

procedure TXMLReader.ConvertSource(SrcIn: TXMLInputSource; out SrcOut: TXMLCharSource);
begin
  SrcOut := nil;
  if Assigned(SrcIn) then
  begin
    if Assigned(SrcIn.FStream) then
      SrcOut := TXMLStreamInputSource.Create(SrcIn.FStream, False)
    else if SrcIn.FStringData <> '' then
      SrcOut := TXMLStreamInputSource.Create(TStringStream.Create(SrcIn.FStringData), True)
    else if (SrcIn.SystemID <> '') then
<<<<<<< HEAD
      ResolveResource(SrcIn.SystemID, SrcIn.PublicID, SrcIn.BaseURI, SrcOut);
=======
      ResolveEntity(SrcIn.SystemID, SrcIn.PublicID, SrcOut);
>>>>>>> graemeg/fixes_2_2
  end;
  if (SrcOut = nil) and (FSource = nil) then
    DoErrorPos(esFatal, 'No input source specified', NullLocation);
=======
  if Assigned(FEntityValue.Buffer) then
    FreeMem(FEntityValue.Buffer);
  FreeMem(FName.Buffer);
  FreeMem(FValue.Buffer);
  if Assigned(FSource) then
    while ContextPop do;     // clean input stack
  FSource.Free;
  FPEMap.Free;
  ClearRefs(FNotationRefs);
  ClearRefs(FIDRefs);
  FNotationRefs.Free;
  FIDRefs.Free;
  inherited Destroy;
end;

procedure TXMLReader.XML11_BuildTables;
begin
  FNamePages := Xml11NamePages;
  FXML11 := True;
  FSource.FXml11Rules := True;
end;

procedure TXMLReader.ProcessXML(ASource: TXMLCharSource);
begin
  doc := TXMLDocument.Create;
  FCursor := doc;
  FState := rsProlog;
  FNesting := 0;
  Initialize(ASource);
  DoParseFragment;              // case FCurChar <> #0 is handled

  if FState < rsRoot then
    FatalError('Root element is missing');

  if FValidate and Assigned(FDocType) then
    ValidateIdRefs;
end;

procedure TXMLReader.ProcessFragment(ASource: TXMLCharSource; AOwner: TDOMNode);
begin
  doc := AOwner.OwnerDocument;
  FCursor := AOwner as TDOMNode_WithChildren;
  FState := rsRoot;
  Initialize(ASource);
  FXML11 := doc.InheritsFrom(TXMLDocument) and (TXMLDocument(doc).XMLVersion = '1.1');
  DoParseFragment;
end;

function TXMLReader.CheckName(aFlags: TCheckNameFlags): Boolean;
var
  p: PWideChar;
  NameStartFlag: Boolean;
begin
  p := FSource.FBuf;
  FName.Length := 0;
  FColonPos := -1;
  NameStartFlag := not (cnToken in aFlags);

  repeat
    if NameStartFlag then
    begin
      if (Byte(p^) in NamingBitmap[FNamePages^[hi(Word(p^))]]) or
        ((p^ = ':') and (not FNamespaces)) then
        Inc(p)
      else if FXML11 and ((p^ >= #$D800) and (p^ <= #$DB7F) and
        (p[1] >= #$DC00) and (p[1] <= #$DFFF)) then
        Inc(p, 2)
      else
      begin
  // here we come either when first char of name is bad (it may be a colon),
  // or when a colon is not followed by a valid NameStartChar
        FSource.FBuf := p;
        Result := False;
        Break;
      end;
      NameStartFlag := False;
    end;

    if FXML11 then
    repeat
      if Byte(p^) in NamingBitmap[FNamePages^[$100+hi(Word(p^))]] then
        Inc(p)
      else if ((p^ >= #$D800) and (p^ <= #$DB7F) and
        (p[1] >= #$DC00) and (p[1] <= #$DFFF)) then
        Inc(p,2)
      else
        Break;
    until False
    else
    while Byte(p^) in NamingBitmap[FNamePages^[$100+hi(Word(p^))]] do
      Inc(p);

    if p^ = ':' then
    begin
      if (cnToken in aFlags) or not FNamespaces then  // colon has no specific meaning
      begin
        Inc(p);
        if p^ <> #0 then Continue;
      end
      else if FColonPos = -1 then       // this is the first colon, remember it
      begin
        FColonPos := p-FSource.FBuf+FName.Length;
        NameStartFlag := True;
        Inc(p);
        if p^ <> #0 then Continue;
      end;
    end;

    BufAppendChunk(FName, FSource.FBuf, p-FSource.FBuf);

    FSource.FBuf := p;
    if (p^ <> #0) or not FSource.Reload then
      Break;

    p := FSource.FBuf;
  until False;
  Result := (FName.Length > 0);
  FCurChar := FSource.FBuf^;
  if not (Result or (cnOptional in aFlags)) then
    RaiseNameNotFound;
end;

procedure TXMLReader.CheckNCName;
begin
  if FNamespaces and (FColonPos <> -1) then
    FatalError('Names of entities, notations and processing instructions may not contain colons', FName.Length);
>>>>>>> origin/fixes_2_2
end;

procedure TXMLReader.StoreLocation(out Loc: TLocation);
begin
<<<<<<< HEAD
  Loc.Line := FSource.FLineNo;
  Loc.LinePos := FSource.FBuf-FSource.LFPos;
=======
  if FColonPos <> -1 then
    FatalError('Bad QName syntax, local part is missing')
  else
  // Coming at no cost, this allows more user-friendly error messages
  if (FCurChar = #32) or (FCurChar = #10) or (FCurChar = #9) or (FCurChar = #13) then
    FatalError('Whitespace is not allowed here')
  else
    FatalError('Name starts with invalid character');
>>>>>>> origin/fixes_2_2
end;

<<<<<<< HEAD
function TXMLReader.ResolveResource(const ASystemID, APublicID, ABaseURI: WideString; out Source: TXMLCharSource): Boolean;
=======
function TXMLReader.ResolveEntity(const SystemID, PublicID: WideString; out Source: TXMLCharSource): Boolean;
>>>>>>> graemeg/fixes_2_2
var
  AbsSysID: WideString;
  Filename: string;
  Stream: TStream;
  fd: THandle;
begin
<<<<<<< HEAD
  Source := nil;
  Result := False;
<<<<<<< HEAD
  if not ResolveRelativeURI(ABaseURI, ASystemID, AbsSysID) then
    Exit;
=======
  if not Assigned(FSource) then
    AbsSysID := SystemID
  else
    if not ResolveRelativeURI(FSource.SystemID, SystemID, AbsSysID) then
      Exit;
>>>>>>> graemeg/fixes_2_2
  { TODO: alternative resolvers
    These may be 'internal' resolvers or a handler set by application.
    Internal resolvers should probably produce a TStream
    ( so that internal classes need not be exported ).
    External resolver will produce TXMLInputSource that should be converted.
    External resolver must NOT be called for root entity.
    External resolver can return nil, in which case we do the default }
  if URIToFilename(AbsSysID, Filename) then
  begin
    fd := FileOpen(Filename, fmOpenRead + fmShareDenyWrite);
    if fd <> THandle(-1) then
    begin
      Stream := THandleOwnerStream.Create(fd);
      Source := TXMLStreamInputSource.Create(Stream, True);
      Source.SystemID := AbsSysID;    // <- Revisit: Really need absolute sysID?
<<<<<<< HEAD
=======
      Source.PublicID := PublicID;
>>>>>>> graemeg/fixes_2_2
    end;
  end;
  Result := Assigned(Source);
end;

procedure TXMLReader.Initialize(ASource: TXMLCharSource);
begin
<<<<<<< HEAD
  ASource.FParent := FSource;
  FSource := ASource;
  FSource.FReader := Self;
  FSource.FStartNesting := FNesting;
  FSource.Initialize;
end;

procedure TXMLReader.FatalError(Expected: WideChar);
begin
// FIX: don't output what is found - anything may be found, including exploits...
  FatalError('Expected "%1s"', [string(Expected)]);
end;

procedure TXMLReader.FatalError(const descr: String; LineOffs: Integer);
begin
  DoError(esFatal, descr, LineOffs);
end;

procedure TXMLReader.FatalError(const descr: string; const args: array of const; LineOffs: Integer);
begin
  DoError(esFatal, Format(descr, args), LineOffs);
end;

procedure TXMLReader.ValidationError(const Msg: string; const Args: array of const; LineOffs: Integer);
begin
  if FValidate then
    DoError(esError, Format(Msg, Args), LineOffs);
end;

procedure TXMLReader.ValidationErrorWithName(const Msg: string; LineOffs: Integer);
var
  ws: WideString;
begin
  SetString(ws, FName.Buffer, FName.Length);
  ValidationError(Msg, [ws], LineOffs);
end;

procedure TXMLReader.DoError(Severity: TErrorSeverity; const descr: string; LineOffs: Integer);
var
  Loc: TLocation;
=======
  CheckName;
  SetString(Result, FName.Buffer, FName.Length);
end;

function TXMLReader.ResolvePredefined: Boolean;
var
  wc: WideChar;
begin
  Result := False;
  if BufEquals(FName, 'amp') then
    wc := '&'
  else if BufEquals(FName, 'apos') then
    wc := ''''
  else if BufEquals(FName, 'gt') then
    wc := '>'
  else if BufEquals(FName, 'lt') then
    wc := '<'
  else if BufEquals(FName, 'quot') then
    wc := '"'
  else
    Exit;
  BufAppend(FValue, wc);
  Result := True;
end;

function TXMLReader.ParseCharRef(var ToFill: TWideCharBuf): Boolean;           // [66]
var
  Value: Integer;
begin
  StoreLocation(FTokenStart);
  GetChar;   // skip '&'
  Result := FCurChar = '#';
  if Result then
  begin
    GetChar;
    Value := 0;
    if CheckForChar('x') then
    repeat
      case FCurChar of
        '0'..'9': Value := Value * 16 + Ord(FCurChar) - Ord('0');
        'a'..'f': Value := Value * 16 + Ord(FCurChar) - (Ord('a') - 10);
        'A'..'F': Value := Value * 16 + Ord(FCurChar) - (Ord('A') - 10);
      else
        Break;
      end;
      GetChar;
    until False
    else
    repeat
      case FCurChar of
        '0'..'9': Value := Value * 10 + Ord(FCurChar) - Ord('0');
      else
        Break;
      end;
      GetChar;
    until False;

    case Value of
      $01..$08, $0B..$0C, $0E..$1F:
        if FXML11 then
          BufAppend(ToFill, WideChar(Value))
        else
          FatalError('Invalid character reference');
      $09, $0A, $0D, $20..$D7FF, $E000..$FFFD:
        BufAppend(ToFill, WideChar(Value));
      $10000..$10FFFF:
        begin
          BufAppend(ToFill, WideChar($D7C0 + (Value shr 10)));
          BufAppend(ToFill, WideChar($DC00 xor (Value and $3FF)));
        end;
    else
      FatalError('Invalid character reference');
    end;
  end
  else CheckName;
  ExpectChar(';');
end;

function TXMLReader.DoParseAttValue(Delim: WideChar): Boolean;
>>>>>>> origin/fixes_2_2
begin
  StoreLocation(Loc);
  if LineOffs >= 0 then
  begin
<<<<<<< HEAD
    Dec(Loc.LinePos, LineOffs);
    DoErrorPos(Severity, descr, Loc);
  end
  else
    DoErrorPos(Severity, descr, FTokenStart);
end;

procedure TXMLReader.DoErrorPos(Severity: TErrorSeverity; const descr: string;
  const args: array of const; const ErrPos: TLocation);
begin
  DoErrorPos(Severity, Format(descr, args), ErrPos);
=======
  FSource := ASource;
  FSource.FReader := Self;
  FSource.Initialize;
  FCurChar := FSource.FBuf^;
end;

procedure TXMLReader.GetChar;
begin
  if FCurChar = #10 then
  begin
    Inc(FSource.FLineNo);
    FSource.LFPos := FSource.FBuf;
  end;
  FCurChar := FSource.NextChar;
=======
    if FCurChar = '<' then
      FatalError('Character ''<'' is not allowed in attribute value')
    else if FCurChar <> '&' then
    begin
      if (FCurChar = #10) or (FCurChar = #9) or (FCurChar = #13) then
        BufAppend(FValue, #32)  // don't change FCurChar, needed for correct location reporting
      else
        BufAppend(FValue, FCurChar);
      GetChar;
    end
    else
    begin
      if ParseCharRef(FValue) or ResolvePredefined then
        Continue;
      // have to insert entity or reference
      if FValue.Length > 0 then
      begin
        DoAttrText(FValue.Buffer, FValue.Length);
        FValue.Length := 0;
      end;
      IncludeEntity(True);
    end;
  end; // while
  if FValue.Length > 0 then
  begin
    DoAttrText(FValue.Buffer, FValue.Length);
    FValue.Length := 0;
  end;
  Result := FCurChar <> #0;
>>>>>>> origin/fixes_2_2
end;

procedure TXMLReader.RaiseExpectedQmark;
begin
<<<<<<< HEAD
  FatalError('Expected single or double quote');
end;

procedure TXMLReader.FatalError(Expected: WideChar);
begin
// FIX: don't output what is found - anything may be found, including exploits...
  FatalError('Expected "%1s"', [string(Expected)]);
end;
=======
  // SAX: ContentHandler.StartDocument() - here?
  ParseContent;
  if FCurChar <> #0 then
    FatalError('End-tag is not allowed here');
  // SAX: ContentHandler.EndDocument() - here? or somewhere in destructor?
end;

function TXMLReader.ContextPush(AEntity: TDOMEntityEx): Boolean;
var
  Src: TXMLCharSource;
begin
  if AEntity.SystemID <> '' then
  begin
    Result := ResolveEntity(AEntity.SystemID, AEntity.PublicID, Src);
    if not Result then
    begin
      // TODO: a detailed message like SysErrorMessage(GetLastError) would be great here
      ValidationError('Unable to resolve external entity ''%s''', [AEntity.NodeName]);
      Exit;
    end;
  end
  else
  begin
    Src := TXMLCharSource.Create(AEntity.FReplacementText);
    Src.FLineNo := AEntity.FStartLocation.Line;
    Src.LFPos := Src.FBuf - AEntity.FStartLocation.LinePos;
  end;
>>>>>>> origin/fixes_2_2

procedure TXMLReader.FatalError(const descr: String; LineOffs: Integer);
begin
  DoError(esFatal, descr, LineOffs);
end;

<<<<<<< HEAD
procedure TXMLReader.FatalError(const descr: string; const args: array of const; LineOffs: Integer);
begin
  DoError(esFatal, Format(descr, args), LineOffs);
end;

procedure TXMLReader.ValidationError(const Msg: string; const Args: array of const; LineOffs: Integer);
begin
  FDocNotValid := True;
  if FValidate then
    DoError(esError, Format(Msg, Args), LineOffs);
end;

procedure TXMLReader.DoError(Severity: TErrorSeverity; const descr: string; LineOffs: Integer);
var
  Loc: TLocation;
begin
  StoreLocation(Loc);
  if LineOffs >= 0 then
  begin
    Dec(Loc.LinePos, LineOffs);
    DoErrorPos(Severity, descr, Loc);
  end
  else
    DoErrorPos(Severity, descr, FTokenStart);
>>>>>>> graemeg/fixes_2_2
end;

procedure TXMLReader.DoErrorPos(Severity: TErrorSeverity; const descr: string; const ErrPos: TLocation);
var
  E: EXMLReadError;
<<<<<<< HEAD
  sysid: WideString;
begin
  if Assigned(FSource) then
  begin
    sysid := FSource.FSystemID;
    if (sysid = '') and Assigned(FSource.FEntity) then
      sysid := FSource.FEntity.FURI;
    E := EXMLReadError.CreateFmt('In ''%s'' (line %d pos %d): %s', [sysid, ErrPos.Line, ErrPos.LinePos, descr]);
  end
=======
begin
  if Assigned(FSource) then
    E := EXMLReadError.CreateFmt('In ''%s'' (line %d pos %d): %s', [FSource.SystemID, ErrPos.Line, ErrPos.LinePos, descr])
>>>>>>> graemeg/fixes_2_2
  else
    E := EXMLReadError.Create(descr);
  E.FSeverity := Severity;
  E.FErrorMessage := descr;
  E.FLine := ErrPos.Line;
  E.FLinePos := ErrPos.LinePos;
  CallErrorHandler(E);
  // No 'finally'! If user handler raises exception, control should not get here
  // and the exception will be freed in CallErrorHandler (below)
  E.Free;
end;

<<<<<<< HEAD
procedure TXMLReader.CheckMaxChars(ToAdd: Cardinal);
var
  src: TXMLCharSource;
  total: Cardinal;
begin
  Inc(FSource.FCharCount, ToAdd);
  if FMaxChars = 0 then
    Exit;
  src := FSource;
  total := 0;
  repeat
    Inc(total, src.FCharCount);
    if total > FMaxChars then
      FatalError('Exceeded character count limit');
    src := src.FParent;
  until src = nil;
end;

procedure TXMLReader.CallErrorHandler(E: EXMLReadError);
begin
  try
    if Assigned(FCtrl) and Assigned(FCtrl.FOnError) then
      FCtrl.FOnError(E);
    if E.Severity = esFatal then
      raise E;
  except
    if ExceptObject <> E then
      E.Free;
    raise;
  end;
end;

function TXMLReader.SkipWhitespace(PercentAloneIsOk: Boolean): Boolean;
begin
  Result := False;
  repeat
    Result := SkipS or Result;
    if FSource.FBuf^ = #0 then
    begin
      Result := True;      // report whitespace upon exiting the PE
      if not ContextPop then
        Break;
    end
    else if FSource.FBuf^ = '%' then
    begin
      if (FState <> rsDTD) or ((FSource.Kind = skInternalSubset) and FInsideDecl) then
        Break;
// This is the only case where look-ahead is needed
      if FSource.FBuf > FSource.FBufEnd-2 then
        FSource.Reload;

      if (not PercentAloneIsOk) or (Byte(FSource.FBuf[1]) in NamingBitmap[FNamePages^[$100+hi(Word(FSource.FBuf[1]))]]) or
        (FXML11 and (FSource.FBuf[1] >= #$D800) and (FSource.FBuf[1] <= #$DB7F)) then
      begin
        Inc(FSource.FBuf);    // skip '%'
        CheckName;
        ExpectChar(';');
        StartPE;
        Result := True;        // report whitespace upon entering the PE
      end
      else Break;
    end
    else
      Break;
  until False;
=======
procedure TXMLReader.CallErrorHandler(E: EXMLReadError);
begin
  try
    if Assigned(FCtrl) and Assigned(FCtrl.FOnError) then
      FCtrl.FOnError(E);
    if E.Severity = esFatal then
      raise E;
  except
    if ExceptObject <> E then
      E.Free;
    raise;
  end;
end;

function TXMLReader.SkipWhitespace(PercentAloneIsOk: Boolean): Boolean;
begin
  Result := False;
  repeat
    Result := SkipS or Result;
    case FCurChar of
      #0: begin
        Result := True;
        if not ContextPop then
          Break;
      end;

      '%': begin
        if not FRecognizePE then
          Break;
// This is the only case where look-ahead is needed
        if FSource.FBuf > FSource.FBufEnd-2 then
          FSource.Reload;
        if (not PercentAloneIsOk) or
          (Byte(FSource.FBuf[1]) in NamingBitmap[FNamePages^[hi(Word(FSource.FBuf[1]))]]) or
          (FXML11 and (FSource.FBuf[1] >= #$D800) and (FSource.FBuf[1] <= #$DB7F)) then
        begin
          Inc(FSource.FBuf);    // skip '%'
          FCurChar := FSource.FBuf^;
          CheckName;
          ExpectChar(';');
          StartPE;
          Result := True;        // report whitespace upon entering the PE
        end
        else Break;
      end
    else
      Break;
    end;
  until False;
end;

function TXMLReader.SkipS(required: Boolean): Boolean;
begin
  Result := False;
  while (FCurChar = #32) or (FCurChar = #10) or (FCurChar = #9) or (FCurChar = #13) do
  begin
    GetChar;
    Result := True;
  end;
  if not Result and required then
    FatalError('Expected whitespace');
>>>>>>> graemeg/fixes_2_2
end;

procedure TXMLReader.ExpectWhitespace;
begin
  if not SkipWhitespace then
    FatalError('Expected whitespace');
end;

function TXMLReader.SkipS(Required: Boolean): Boolean;
var
  p: PWideChar;
begin
<<<<<<< HEAD
  Result := False;
  repeat
    p := FSource.FBuf;
    repeat
      if (p^ = #10) or (p^ = #13) or (FXML11 and ((p^ = #$85) or (p^ = #$2028))) then
      begin
        FSource.FBuf := p;
        FSource.NewLine;
        p := FSource.FBuf;
      end
      else if (p^ <> #32) and (p^ <> #9) then
        Break;
      Inc(p);
      Result := True;
    until False;
    FSource.FBuf := p;
  until (FSource.FBuf < FSource.FBufEnd) or (not FSource.Reload);
  if (not Result) and Required then
    FatalError('Expected whitespace');
=======
  if FCurChar = wc then
    GetChar
  else
    FatalError(wc);
>>>>>>> graemeg/fixes_2_2
end;

procedure TXMLReader.ExpectString(const s: String);
var
  I: Integer;
begin
  for I := 1 to Length(s) do
  begin
<<<<<<< HEAD
    if FSource.FBuf^ <> WideChar(ord(s[i])) then
      FatalError('Expected "%s"', [s], i-1);
    FSource.NextChar;
=======
    if FCurChar <> WideChar(ord(s[i])) then
      FatalError('Expected "%s"', [s], i-1);
    GetChar;
>>>>>>> graemeg/fixes_2_2
  end;
end;

function TXMLReader.CheckForChar(c: WideChar): Boolean;
begin
  Result := (FSource.FBuf^ = c);
  if Result then
  begin
    Inc(FSource.FBuf);
    if FSource.FBuf >= FSource.FBufEnd then
      FSource.Reload;
  end;  
end;

<<<<<<< HEAD
procedure TXMLReader.SkipQuote(out Delim: WideChar; required: Boolean);
begin
  Delim := #0;
  if (FSource.FBuf^ = '''') or (FSource.FBuf^ = '"') then
  begin
    Delim := FSource.FBuf^;
    FSource.NextChar;  // skip quote
    StoreLocation(FTokenStart);
  end
  else if required then
    FatalError('Expected single or double quote');
end;

const
  PrefixDefault: array[0..4] of WideChar = ('x','m','l','n','s');

=======
>>>>>>> graemeg/fixes_2_2
constructor TXMLReader.Create;
begin
  inherited Create;
  BufAllocate(FName, 128);
  BufAllocate(FValue, 512);
<<<<<<< HEAD
  FForwardRefs := TFPList.Create;
  FAttrChunks := TFPList.Create;

  // Set char rules to XML 1.0
  FNamePages := @NamePages;
  SetLength(FNodeStack, 16);
  SetLength(FValidators, 16);
=======
  FIDRefs := TFPList.Create;
  FNotationRefs := TFPList.Create;

  // Set char rules to XML 1.0
  FNamePages := @NamePages;
  SetLength(FValidator, 16);
>>>>>>> graemeg/fixes_2_2
end;

constructor TXMLReader.Create(AParser: TDOMParser);
begin
  Create;
  FCtrl := AParser;
<<<<<<< HEAD
  if FCtrl = nil then
    Exit;
=======
>>>>>>> graemeg/fixes_2_2
  FValidate := FCtrl.Options.Validate;
  FPreserveWhitespace := FCtrl.Options.PreserveWhitespace;
  FExpandEntities := FCtrl.Options.ExpandEntities;
  FCDSectionsAsText := FCtrl.Options.CDSectionsAsText;
  FIgnoreComments := FCtrl.Options.IgnoreComments;
  FResolveExternals := FCtrl.Options.ResolveExternals;
  FNamespaces := FCtrl.Options.Namespaces;
<<<<<<< HEAD
  FDisallowDoctype := FCtrl.Options.DisallowDoctype;
  FCanonical := FCtrl.Options.CanonicalForm;
  FMaxChars := FCtrl.Options.MaxChars;
=======
>>>>>>> graemeg/fixes_2_2
end;

destructor TXMLReader.Destroy;
var
  i: Integer;
begin
<<<<<<< HEAD
  for i := FAttrChunks.Count-1 downto 0 do
    Dispose(PNodeData(FAttrChunks.List^[i]));
=======
>>>>>>> graemeg/fixes_2_2
  if Assigned(FEntityValue.Buffer) then
    FreeMem(FEntityValue.Buffer);
  FreeMem(FName.Buffer);
  FreeMem(FValue.Buffer);
  if Assigned(FSource) then
<<<<<<< HEAD
    while ContextPop(True) do;     // clean input stack
  FSource.Free;
  FPEMap.Free;
  ClearForwardRefs;
  FNsAttHash.Free;
  FNSHelper.Free;
  FDocType.Release;
  FIDMap.Free;
  FForwardRefs.Free;
  FAttrChunks.Free;
=======
    while ContextPop do;     // clean input stack
  FSource.Free;
  FPEMap.Free;
  ClearRefs(FNotationRefs);
  ClearRefs(FIDRefs);
  FNotationRefs.Free;
  FIDRefs.Free;
>>>>>>> graemeg/fixes_2_2
  inherited Destroy;
end;

procedure TXMLReader.XML11_BuildTables;
begin
  FNamePages := Xml11NamePages;
  FXML11 := True;
  FSource.FXml11Rules := True;
end;

<<<<<<< HEAD
{ Must be executed after doc has been set.
  After introducing own NameTable, merge this into constructor }
procedure TXMLReader.NSPrepare;
begin
  if FNamespaces then
  begin
    FNSHelper := TNSSupport.Create;
    FNsAttHash := TDblHashArray.Create;
    FStdPrefix_xml := FNSHelper.GetPrefix(@PrefixDefault, 3);
    FStdPrefix_xmlns := FNSHelper.GetPrefix(@PrefixDefault, 5);

    FStdUri_xmlns := FNameTable.FindOrAdd(PWideChar(stduri_xmlns), Length(stduri_xmlns));
    FStdUri_xml := FNameTable.FindOrAdd(PWideChar(stduri_xml), Length(stduri_xml));
  end;
end;

procedure TXMLReader.ProcessXML(ASource: TXMLCharSource);
begin
  doc := TXMLDocument.Create;
  doc.documentURI := ASource.SystemID;  // TODO: to be changed to URI or BaseURI
  FNameTable := doc.Names;
  FState := rsProlog;
  FNesting := 0;
  FValidatorNesting := 0;
  FCurrNode := @FNodeStack[0];
  FFragmentMode := False;
  NSPrepare;
  Initialize(ASource);
  if FSource.FXMLVersion <> xmlVersionUnknown then
    TDOMTopNodeEx(TDOMNode(doc)).FXMLVersion := FSource.FXMLVersion;
  TDOMTopNodeEx(TDOMNode(doc)).FXMLEncoding := FSource.FXMLEncoding;
  FNext := xtText;
  ParseContent(doc);

  if FState < rsRoot then
    FatalError('Root element is missing');

  if FValidate then
    ValidateIdRefs;

  doc.IDs := FIDMap;
  FIDMap := nil;
end;

procedure TXMLReader.ProcessFragment(ASource: TXMLCharSource; AOwner: TDOMNode);
var
  DoctypeNode: TDOMDocumentTypeEx;
begin
  doc := AOwner.OwnerDocument;
  FNameTable := doc.Names;
  FState := rsRoot;
  FNesting := 0;
  FValidatorNesting := 0;
  FCurrNode := @FNodeStack[0];
  FFragmentMode := True;
  FXML11 := doc.XMLVersion = '1.1';
  NSPrepare;
  Initialize(ASource);
  { Get doctype from the owner's document, but only if it is not already assigned
   (It is set directly when parsing children of an Entity, see LoadEntity procedure) }
  if FDocType = nil then
  begin
    DoctypeNode := TDOMDocumentTypeEx(doc.DocType);
    if Assigned(DoctypeNode) then
      FDocType := DocTypeNode.FModel.Reference;
  end;
  if AOwner is TDOMEntity then
  begin
    TDOMTopNodeEx(AOwner).FXMLVersion := FSource.FXMLVersion;
    TDOMTopNodeEx(AOwner).FXMLEncoding := FSource.FXMLEncoding;
  end;
  FNext := xtText;
  ParseContent(aOwner as TDOMNode_WithChildren);
end;

=======
procedure TXMLReader.ProcessXML(ASource: TXMLCharSource);
begin
  doc := TXMLDocument.Create;
  FCursor := doc;
  FState := rsProlog;
  FNesting := 0;
  Initialize(ASource);
  DoParseFragment;              // case FCurChar <> #0 is handled

  if FState < rsRoot then
    FatalError('Root element is missing');

  if FValidate and Assigned(FDocType) then
    ValidateIdRefs;
end;

procedure TXMLReader.ProcessFragment(ASource: TXMLCharSource; AOwner: TDOMNode);
begin
  doc := AOwner.OwnerDocument;
  FCursor := AOwner as TDOMNode_WithChildren;
  FState := rsRoot;
  Initialize(ASource);
  FXML11 := doc.InheritsFrom(TXMLDocument) and (TXMLDocument(doc).XMLVersion = '1.1');
  DoParseFragment;
end;

>>>>>>> graemeg/fixes_2_2
function TXMLReader.CheckName(aFlags: TCheckNameFlags): Boolean;
var
  p: PWideChar;
  NameStartFlag: Boolean;
begin
  p := FSource.FBuf;
  FName.Length := 0;
  FColonPos := -1;
  NameStartFlag := not (cnToken in aFlags);

  repeat
    if NameStartFlag then
    begin
      if (Byte(p^) in NamingBitmap[FNamePages^[hi(Word(p^))]]) or
        ((p^ = ':') and (not FNamespaces)) then
        Inc(p)
      else if FXML11 and ((p^ >= #$D800) and (p^ <= #$DB7F) and
        (p[1] >= #$DC00) and (p[1] <= #$DFFF)) then
        Inc(p, 2)
      else
      begin
  // here we come either when first char of name is bad (it may be a colon),
  // or when a colon is not followed by a valid NameStartChar
        FSource.FBuf := p;
        Result := False;
        Break;
      end;
      NameStartFlag := False;
    end;

    if FXML11 then
    repeat
      if Byte(p^) in NamingBitmap[FNamePages^[$100+hi(Word(p^))]] then
        Inc(p)
      else if ((p^ >= #$D800) and (p^ <= #$DB7F) and
        (p[1] >= #$DC00) and (p[1] <= #$DFFF)) then
        Inc(p,2)
      else
        Break;
    until False
    else
    while Byte(p^) in NamingBitmap[FNamePages^[$100+hi(Word(p^))]] do
      Inc(p);

    if p^ = ':' then
    begin
      if (cnToken in aFlags) or not FNamespaces then  // colon has no specific meaning
      begin
        Inc(p);
        if p^ <> #0 then Continue;
      end
      else if FColonPos = -1 then       // this is the first colon, remember it
      begin
        FColonPos := p-FSource.FBuf+FName.Length;
        NameStartFlag := True;
        Inc(p);
        if p^ <> #0 then Continue;
      end;
    end;

<<<<<<< HEAD
    BufAppendChunk(FName, FSource.FBuf, p);
    Result := (FName.Length > 0);
=======
    BufAppendChunk(FName, FSource.FBuf, p-FSource.FBuf);
>>>>>>> graemeg/fixes_2_2

    FSource.FBuf := p;
    if (p^ <> #0) or not FSource.Reload then
      Break;

    p := FSource.FBuf;
  until False;
<<<<<<< HEAD
=======
  Result := (FName.Length > 0);
  FCurChar := FSource.FBuf^;
>>>>>>> graemeg/fixes_2_2
  if not (Result or (cnOptional in aFlags)) then
    RaiseNameNotFound;
end;

procedure TXMLReader.CheckNCName;
begin
  if FNamespaces and (FColonPos <> -1) then
    FatalError('Names of entities, notations and processing instructions may not contain colons', FName.Length);
end;

procedure TXMLReader.RaiseNameNotFound;
begin
  if FColonPos <> -1 then
    FatalError('Bad QName syntax, local part is missing')
  else
  // Coming at no cost, this allows more user-friendly error messages
<<<<<<< HEAD
  with FSource do
  if (FBuf^ = #32) or (FBuf^ = #10) or (FBuf^ = #9) or (FBuf^ = #13) then
=======
  if (FCurChar = #32) or (FCurChar = #10) or (FCurChar = #9) or (FCurChar = #13) then
>>>>>>> graemeg/fixes_2_2
    FatalError('Whitespace is not allowed here')
  else
    FatalError('Name starts with invalid character');
end;

function TXMLReader.ExpectName: WideString;
begin
  CheckName;
  SetString(Result, FName.Buffer, FName.Length);
end;

function TXMLReader.ResolvePredefined: Boolean;
var
  wc: WideChar;
begin
  Result := False;
<<<<<<< HEAD
  with FName do
  begin
    if (Length = 2) and (Buffer[1] = 't') then
    begin
      if Buffer[0] = 'l' then
        wc := '<'
      else if Buffer[0] = 'g' then
        wc := '>'
      else Exit;
    end
    else if Buffer[0] = 'a' then
    begin
      if (Length = 3) and (Buffer[1] = 'm') and (Buffer[2] = 'p') then
        wc := '&'
      else if (Length = 4) and (Buffer[1] = 'p') and (Buffer[2] = 'o') and
       (Buffer[3] = 's') then
        wc := ''''
      else Exit;  
    end
    else if (Length = 4) and (Buffer[0] = 'q') and (Buffer[1] = 'u') and
      (Buffer[2] = 'o') and (Buffer[3] ='t') then
      wc := '"'
    else
      Exit;
  end; // with
=======
  if BufEquals(FName, 'amp') then
    wc := '&'
  else if BufEquals(FName, 'apos') then
    wc := ''''
  else if BufEquals(FName, 'gt') then
    wc := '>'
  else if BufEquals(FName, 'lt') then
    wc := '<'
  else if BufEquals(FName, 'quot') then
    wc := '"'
  else
    Exit;
>>>>>>> graemeg/fixes_2_2
  BufAppend(FValue, wc);
  Result := True;
end;

<<<<<<< HEAD
function TXMLReader.ParseRef(var ToFill: TWideCharBuf): Boolean;  // [67]
=======
function TXMLReader.ParseCharRef(var ToFill: TWideCharBuf): Boolean;           // [66]
>>>>>>> graemeg/fixes_2_2
var
  Code: Integer;
begin
<<<<<<< HEAD
  FSource.NextChar;   // skip '&'
  Result := CheckForChar('#');
  if Result then
  begin
    Code := 0;
=======
  StoreLocation(FTokenStart);
  GetChar;   // skip '&'
  Result := FCurChar = '#';
  if Result then
  begin
    GetChar;
    Value := 0;
>>>>>>> graemeg/fixes_2_2
    if CheckForChar('x') then
    repeat
      case FSource.FBuf^ of
        '0'..'9': Code := Code * 16 + Ord(FSource.FBuf^) - Ord('0');
        'a'..'f': Code := Code * 16 + Ord(FSource.FBuf^) - (Ord('a') - 10);
        'A'..'F': Code := Code * 16 + Ord(FSource.FBuf^) - (Ord('A') - 10);
      else
        Break;
      end;
<<<<<<< HEAD
      FSource.NextChar;
    until Code > $10FFFF
=======
      GetChar;
    until False
>>>>>>> graemeg/fixes_2_2
    else
    repeat
      case FSource.FBuf^ of
        '0'..'9': Code := Code * 10 + Ord(FSource.FBuf^) - Ord('0');
      else
        Break;
      end;
<<<<<<< HEAD
      FSource.NextChar;
    until Code > $10FFFF;

    case Code of
      $01..$08, $0B..$0C, $0E..$1F:
        if FXML11 then
          BufAppend(ToFill, WideChar(Code))
        else
          FatalError('Invalid character reference');
      $09, $0A, $0D, $20..$D7FF, $E000..$FFFD:
        BufAppend(ToFill, WideChar(Code));
      $10000..$10FFFF:
        begin
          BufAppend(ToFill, WideChar($D7C0 + (Code shr 10)));
          BufAppend(ToFill, WideChar($DC00 xor (Code and $3FF)));
=======
      GetChar;
    until False;

    case Value of
      $01..$08, $0B..$0C, $0E..$1F:
        if FXML11 then
          BufAppend(ToFill, WideChar(Value))
        else
          FatalError('Invalid character reference');
      $09, $0A, $0D, $20..$D7FF, $E000..$FFFD:
        BufAppend(ToFill, WideChar(Value));
      $10000..$10FFFF:
        begin
          BufAppend(ToFill, WideChar($D7C0 + (Value shr 10)));
          BufAppend(ToFill, WideChar($DC00 xor (Value and $3FF)));
>>>>>>> graemeg/fixes_2_2
        end;
    else
      FatalError('Invalid character reference');
    end;
  end
  else CheckName;
  ExpectChar(';');
end;

<<<<<<< HEAD
const
  AttrDelims: TSetOfChar = [#0, '<', '&', '''', '"', #9, #10, #13];
  GT_Delim: TSetOfChar = [#0, '>'];

{ Parse attribute literal, producing plain string value in AttrData.FValueStr.
  If entity references are encountered and FExpandEntities=False, also builds
  a node chain starting from AttrData.FNext. Node chain is built only for the
  first level. If NonCDATA=True, additionally normalizes whitespace in string value. }

procedure TXMLReader.ExpectAttValue(AttrData: PNodeData; NonCDATA: Boolean);
var
  wc: WideChar;
  Delim: WideChar;
  ent: TEntityDecl;
  start: TObject;
  curr: PNodeData;
  StartPos: Integer;
=======
function TXMLReader.DoParseAttValue(Delim: WideChar): Boolean;
>>>>>>> graemeg/fixes_2_2
begin
  SkipQuote(Delim);
  curr := AttrData;
  FValue.Length := 0;
<<<<<<< HEAD
  StartPos := 0;
  start := FSource.FEntity;
  repeat
    wc := FSource.SkipUntil(FValue, AttrDelims);
    if wc = '<' then
      FatalError('Character ''<'' is not allowed in attribute value')
    else if wc = '&' then
    begin
      if ParseRef(FValue) or ResolvePredefined then
        Continue;

      ent := EntityCheck(True);
      if ((ent = nil) or (not FExpandEntities)) and (FSource.FEntity = start) then
      begin
        if FValue.Length > StartPos then
        begin
          curr := AllocAttributeValueChunk(curr);
          curr^.FNodeType := ntText;
          // without PWideChar typecast and in {$T-}, FPC treats '@' result as PAnsiChar...
          SetString(curr^.FValueStr, PWideChar(@FValue.Buffer[StartPos]), FValue.Length-StartPos);
        end;
        curr := AllocAttributeValueChunk(curr);
        curr^.FNodeType := ntEntityReference;
        // TODO: this probably should be placed to 'name'
        if ent = nil then
          SetString(curr^.FValueStr, FName.Buffer, FName.Length)
        else
          curr^.FValueStr := ent.FName;
      end;
      StartPos := FValue.Length;
      if Assigned(ent) then
        ContextPush(ent);
    end
    else if wc <> #0 then
    begin
      FSource.NextChar;
      if (wc = Delim) and (FSource.FEntity = start) then
        Break;
      if (wc = #10) or (wc = #9) or (wc = #13) then
        wc := #32;
      BufAppend(FValue, wc);
    end
    else
    begin
      if (FSource.FEntity = start) or not ContextPop then    // #0
        FatalError('Literal has no closing quote', -1);
      StartPos := FValue.Length;
=======
  while (FCurChar <> Delim) and (FCurChar <> #0) do
  begin
    if FCurChar = '<' then
      FatalError('Character ''<'' is not allowed in attribute value')
    else if FCurChar <> '&' then
    begin
      if (FCurChar = #10) or (FCurChar = #9) or (FCurChar = #13) then
        BufAppend(FValue, #32)  // don't change FCurChar, needed for correct location reporting
      else
        BufAppend(FValue, FCurChar);
      GetChar;
    end
    else
    begin
      if ParseCharRef(FValue) or ResolvePredefined then
        Continue;
      // have to insert entity or reference
      if FValue.Length > 0 then
      begin
        DoAttrText(FValue.Buffer, FValue.Length);
        FValue.Length := 0;
      end;
      IncludeEntity(True);
>>>>>>> graemeg/fixes_2_2
    end;
  until False;
  if Assigned(attrData^.FNext) then
  begin
<<<<<<< HEAD
    FAttrCleanupFlag := True;
    if FValue.Length > StartPos then
    begin
      curr := AllocAttributeValueChunk(curr);
      curr^.FNodeType := ntText;
      SetString(curr^.FValueStr, PWideChar(@FValue.Buffer[StartPos]), FValue.Length-StartPos);
    end;
  end;
  if nonCDATA then
    BufNormalize(FValue, attrData^.FDenormalized)
  else
    attrData^.FDenormalized := False;
  SetString(attrData^.FValueStr, FValue.Buffer, FValue.Length);
end;

const
  PrefixChar: array[Boolean] of string = ('', '%');

procedure TXMLReader.EntityToSource(AEntity: TEntityDecl; out Src: TXMLCharSource);
=======
    DoAttrText(FValue.Buffer, FValue.Length);
    FValue.Length := 0;
  end;
  Result := FCurChar <> #0;
end;

procedure TXMLReader.DoParseFragment;
begin
  // SAX: ContentHandler.StartDocument() - here?
  ParseContent;
  if FCurChar <> #0 then
    FatalError('End-tag is not allowed here');
  // SAX: ContentHandler.EndDocument() - here? or somewhere in destructor?
end;

function TXMLReader.ContextPush(AEntity: TDOMEntityEx): Boolean;
var
  Src: TXMLCharSource;
>>>>>>> graemeg/fixes_2_2
begin
  if AEntity.FOnStack then
    FatalError('Entity ''%s%s'' recursively references itself', [PrefixChar[AEntity.FIsPE], AEntity.FName]);

  if (AEntity.FSystemID <> '') and not AEntity.FPrefetched then
  begin
<<<<<<< HEAD
    if not ResolveResource(AEntity.FSystemID, AEntity.FPublicID, AEntity.FURI, Src) then
    begin
      // TODO: a detailed message like SysErrorMessage(GetLastError) would be great here
      ValidationError('Unable to resolve external entity ''%s''', [AEntity.FName]);
      Src := nil;
=======
    Result := ResolveEntity(AEntity.SystemID, AEntity.PublicID, Src);
    if not Result then
    begin
      // TODO: a detailed message like SysErrorMessage(GetLastError) would be great here
      ValidationError('Unable to resolve external entity ''%s''', [AEntity.NodeName]);
>>>>>>> graemeg/fixes_2_2
      Exit;
    end;
  end
  else
  begin
    Src := TXMLCharSource.Create(AEntity.FReplacementText);
    Src.FLineNo := AEntity.FStartLocation.Line;
    Src.LFPos := Src.FBuf - AEntity.FStartLocation.LinePos;
<<<<<<< HEAD
    // needed in case of prefetched external PE
    if AEntity.FSystemID <> '' then
      Src.SystemID := AEntity.FURI;
=======
>>>>>>> graemeg/fixes_2_2
  end;

  AEntity.FOnStack := True;
  Src.FEntity := AEntity;
end;

<<<<<<< HEAD
function TXMLReader.ContextPush(AEntity: TEntityDecl): Boolean;
var
  Src: TXMLCharSource;
begin
  EntityToSource(AEntity, Src);
  Result := Assigned(Src);
  if Result then
    Initialize(Src);
end;

function TXMLReader.ContextPop(Forced: Boolean): Boolean;
=======
  ContextPush(Src);
  Result := True;
end;

procedure TXMLReader.ContextPush(ASrc: TXMLCharSource);
begin
  ASrc.FParent := FSource;
  Initialize(ASrc);
end;

function TXMLReader.ContextPop: Boolean;
>>>>>>> graemeg/fixes_2_2
var
  Src: TXMLCharSource;
  Error: Boolean;
begin
<<<<<<< HEAD
  Result := Assigned(FSource.FParent) and (Forced or (FSource.Kind = skNone));
=======
  Result := Assigned(FSource.FParent) and (FSource.DTDSubsetType = dsNone);
>>>>>>> graemeg/fixes_2_2
  if Result then
  begin
    Src := FSource.FParent;
    Error := False;
    if Assigned(FSource.FEntity) then
    begin
<<<<<<< HEAD
      FSource.FEntity.FOnStack := False;
      FSource.FEntity.FCharCount := FSource.FCharCount;
// [28a] PE that was started between MarkupDecls may not end inside MarkupDecl
      Error := FSource.FEntity.FBetweenDecls and FInsideDecl;
    end;
    FSource.Free;
    FSource := Src;
=======
      TDOMEntityEx(FSource.FEntity).FOnStack := False;
// [28a] PE that was started between MarkupDecls may not end inside MarkupDecl
      Error := TDOMEntityEx(FSource.FEntity).FBetweenDecls and FInsideDecl;
    end;
    FSource.Free;
    FSource := Src;
    FCurChar := FSource.FBuf^;
>>>>>>> graemeg/fixes_2_2
// correct position of this error is after PE reference
    if Error then
      BadPENesting(esFatal);
  end;
end;

<<<<<<< HEAD
function TXMLReader.EntityCheck(NoExternals: Boolean): TEntityDecl;
var
  RefName: WideString;
  cnt: Integer;
begin
  Result := nil;
  SetString(RefName, FName.Buffer, FName.Length);
  cnt := FName.Length+2;

  if Assigned(FDocType) then
    Result := FDocType.Entities.Get(FName.Buffer, FName.Length) as TEntityDecl;

  if Result = nil then
  begin
    if FStandalone or (FDocType = nil) or not (FHavePERefs or (FDocType.FSystemID <> '')) then
      FatalError('Reference to undefined entity ''%s''', [RefName], cnt)
    else
      ValidationError('Undefined entity ''%s'' referenced', [RefName], cnt);
    Exit;
  end;

  if FStandalone and Result.ExternallyDeclared then
    FatalError('Standalone constraint violation', cnt);
  if Result.FNotationName <> '' then
    FatalError('Reference to unparsed entity ''%s''', [RefName], cnt);

  if NoExternals and (Result.FSystemID <> '') then
    FatalError('External entity reference is not allowed in attribute value', cnt);

  if not Result.FResolved then
    LoadEntity(Result);

  // at this point we know the charcount of the entity being included
  if Result.FCharCount >= cnt then
    CheckMaxChars(Result.FCharCount - cnt);
=======
procedure TXMLReader.IncludeEntity(InAttr: Boolean);
var
  AEntity: TDOMEntityEx;
  RefName: WideString;
  Child: TDOMNode;
  SaveCursor: TDOMNode_WithChildren;
begin
=======
  ContextPush(Src);
  Result := True;
end;

procedure TXMLReader.ContextPush(ASrc: TXMLCharSource);
begin
  ASrc.FParent := FSource;
  Initialize(ASrc);
end;

function TXMLReader.ContextPop: Boolean;
var
  Src: TXMLCharSource;
  Error: Boolean;
begin
  Result := Assigned(FSource.FParent) and (FSource.DTDSubsetType = dsNone);
  if Result then
  begin
    Src := FSource.FParent;
    Error := False;
    if Assigned(FSource.FEntity) then
    begin
      TDOMEntityEx(FSource.FEntity).FOnStack := False;
// [28a] PE that was started between MarkupDecls may not end inside MarkupDecl
      Error := TDOMEntityEx(FSource.FEntity).FBetweenDecls and FInsideDecl;
    end;
    FSource.Free;
    FSource := Src;
    FCurChar := FSource.FBuf^;
// correct position of this error is after PE reference
    if Error then
      BadPENesting(esFatal);
  end;
end;

procedure TXMLReader.IncludeEntity(InAttr: Boolean);
var
  AEntity: TDOMEntityEx;
  RefName: WideString;
  Child: TDOMNode;
  SaveCursor: TDOMNode_WithChildren;
begin
>>>>>>> origin/fixes_2_2
  AEntity := nil;
  SetString(RefName, FName.Buffer, FName.Length);

  if Assigned(FDocType) then
    AEntity := FDocType.Entities.GetNamedItem(RefName) as TDOMEntityEx;

  if AEntity = nil then
  begin
    if FStandalone or (FDocType = nil) or not (FHavePERefs or (FDocType.SystemID <> '')) then
      FatalError('Reference to undefined entity ''%s''', [RefName], FName.Length+2)
    else
      ValidationError('Undefined entity ''%s'' referenced', [RefName], FName.Length+2);
    FCursor.AppendChild(doc.CreateEntityReference(RefName));
    Exit;
  end;

  if InAttr and (AEntity.SystemID <> '') then
    FatalError('External entity reference is not allowed in attribute value', FName.Length+2);
  if FStandalone and AEntity.FExternallyDeclared then
    FatalError('Standalone constraint violation', FName.Length+2);
  if AEntity.NotationName <> '' then
    FatalError('Reference to unparsed entity ''%s''', [RefName], FName.Length+2);

  if not AEntity.FResolved then
  begin
    if AEntity.FOnStack then
      FatalError('Entity ''%s'' recursively references itself', [RefName]);
<<<<<<< HEAD

    if ContextPush(AEntity) then
    begin
      SaveCursor := FCursor;
      FCursor := AEntity;         // build child node tree for the entity
      try
        AEntity.SetReadOnly(False);
        if InAttr then
          DoParseAttValue(#0)
        else
          DoParseFragment;
        AEntity.FResolved := True;
      finally
        AEntity.SetReadOnly(True);
        ContextPop;
        FCursor := SaveCursor;
        FValue.Length := 0;
      end;
    end;
  end;
  if (not FExpandEntities) or (not AEntity.FResolved) then
  begin
    // This will clone Entity children
    FCursor.AppendChild(doc.CreateEntityReference(RefName));
    Exit;
  end;

  Child := AEntity.FirstChild;  // clone the entity node tree
  while Assigned(Child) do
  begin
    FCursor.AppendChild(Child.CloneNode(True));
    Child := Child.NextSibling;
  end;
>>>>>>> graemeg/fixes_2_2
end;

procedure TXMLReader.StartPE;
var
  PEnt: TEntityDecl;
begin
<<<<<<< HEAD
  PEnt := nil;
  if Assigned(FPEMap) then
    PEnt := FPEMap.Get(FName.Buffer, FName.Length) as TEntityDecl;
  if PEnt = nil then
  begin
    ValidationErrorWithName('Undefined parameter entity ''%s'' referenced', FName.Length+2);
    // cease processing declarations, unless document is standalone.
    FDTDProcessed := FStandalone;
    Exit;
  end;

  { cache an external PE so it's only fetched once }
  if (PEnt.FSystemID <> '') and (not PEnt.FPrefetched) and (not PrefetchEntity(PEnt)) then
  begin
    FDTDProcessed := FStandalone;
    Exit;
  end;
  CheckMaxChars(PEnt.FCharCount);
=======
  SetString(PEName, FName.Buffer, FName.Length);
  PEnt := nil;
  if Assigned(FPEMap) then
    PEnt := FPEMap.GetNamedItem(PEName) as TDOMEntityEx;
  if PEnt = nil then    // TODO -cVC: Referencing undefined PE
  begin                 // (These are classified as 'optional errors'...)
//    ValidationError('Undefined parameter entity referenced: %s', [PEName]);
    Exit;
  end;

  if PEnt.FOnStack then
    FatalError('Entity ''%%%s'' recursively references itself', [PEnt.NodeName]);
>>>>>>> graemeg/fixes_2_2

  PEnt.FBetweenDecls := not FInsideDecl;
  ContextPush(PEnt);
  FHavePERefs := True;
end;

<<<<<<< HEAD
function TXMLReader.PrefetchEntity(AEntity: TEntityDecl): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result := ContextPush(AEntity);
  if Result then
  try
    FValue.Length := 0;
    FSource.SkipUntil(FValue, [#0]);
    SetString(AEntity.FReplacementText, FValue.Buffer, FValue.Length);
    AEntity.FCharCount := FValue.Length;
    AEntity.FStartLocation.Line := 1;
    AEntity.FStartLocation.LinePos := 1;
    AEntity.FURI := FSource.SystemID;    // replace base URI with absolute one
  finally
    ContextPop;
    AEntity.FPrefetched := True;
    FValue.Length := 0;
  end;
end;

const
  LiteralDelims: array[TLiteralType] of TSetOfChar = (
    [#0, '''', '"'],                          // ltPlain
    [#0, '''', '"', #13, #10],                // ltPubid
    [#0, '%', '&', '''', '"']                 // ltEntity
  );

function TXMLReader.ParseLiteral(var ToFill: TWideCharBuf; aType: TLiteralType;
  Required: Boolean): Boolean;
var
  start: TObject;
  wc, Delim: WideChar;
  dummy: Boolean;
begin
  SkipQuote(Delim, Required);
  Result := (Delim <> #0);
  if not Result then
    Exit;
  ToFill.Length := 0;
  start := FSource.FEntity;
  repeat
    wc := FSource.SkipUntil(ToFill, LiteralDelims[aType]);
    if wc = '%' then       { ltEntity only }
    begin
      FSource.NextChar;
      CheckName;
      ExpectChar(';');
      if FSource.Kind = skInternalSubset then
        FatalError('PE reference not allowed here in internal subset', FName.Length+2);
      StartPE;
    end
    else if wc = '&' then  { ltEntity }
    begin
      if ParseRef(ToFill) then   // charRefs always expanded
        Continue;
      BufAppend(ToFill, '&');
      BufAppendChunk(ToFill, FName.Buffer, FName.Buffer + FName.Length);
      BufAppend(ToFill, ';');
    end
    else if wc <> #0 then
    begin
      FSource.NextChar;
      if (wc = #10) or (wc = #13) then
        wc := #32
      // terminating delimiter must be in the same context as the starting one
      else if (wc = Delim) and (start = FSource.FEntity) then
        Break;
      BufAppend(ToFill, wc);
    end
    else if (FSource.FEntity = start) or not ContextPop then    // #0
      FatalError('Literal has no closing quote', -1);
  until False;
  if aType = ltPubid then
    BufNormalize(ToFill, dummy);
end;

function TXMLReader.SkipUntilSeq(const Delim: TSetOfChar; c1: WideChar; c2: WideChar = #0): Boolean;
var
  wc: WideChar;
begin
=======
begin
=======
begin
>>>>>>> graemeg/cpstrnew
=======
begin
>>>>>>> origin/cpstrnew
  Result := ContextPush(AEntity);
  if Result then
  try
    FValue.Length := 0;
    FSource.SkipUntil(FValue, [#0]);
    SetString(AEntity.FReplacementText, FValue.Buffer, FValue.Length);
    AEntity.FCharCount := FValue.Length;
    AEntity.FStartLocation.Line := 1;
    AEntity.FStartLocation.LinePos := 1;
    AEntity.FURI := FSource.SystemID;    // replace base URI with absolute one
  finally
    ContextPop;
    AEntity.FPrefetched := True;
    FValue.Length := 0;
  end;
end;

const
  LiteralDelims: array[TLiteralType] of TSetOfChar = (
    [#0, '''', '"'],                          // ltPlain
    [#0, '''', '"', #13, #10],                // ltPubid
    [#0, '%', '&', '''', '"']                 // ltEntity
  );

function TXMLReader.ParseLiteral(var ToFill: TWideCharBuf; aType: TLiteralType;
  Required: Boolean): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
var
  start: TObject;
  wc, Delim: WideChar;
  dummy: Boolean;
begin
  SkipQuote(Delim, Required);
  Result := (Delim <> #0);
  if not Result then
    Exit;
  ToFill.Length := 0;
  start := FSource.FEntity;
  repeat
    wc := FSource.SkipUntil(ToFill, LiteralDelims[aType]);
    if wc = '%' then       { ltEntity only }
    begin
      FSource.NextChar;
      CheckName;
      ExpectChar(';');
      if FSource.Kind = skInternalSubset then
        FatalError('PE reference not allowed here in internal subset', FName.Length+2);
      StartPE;
    end
    else if wc = '&' then  { ltEntity }
    begin
      if ParseRef(ToFill) then   // charRefs always expanded
        Continue;
      BufAppend(ToFill, '&');
      BufAppendChunk(ToFill, FName.Buffer, FName.Buffer + FName.Length);
      BufAppend(ToFill, ';');
    end
    else if wc <> #0 then
    begin
      FSource.NextChar;
      if (wc = #10) or (wc = #13) then
        wc := #32
      // terminating delimiter must be in the same context as the starting one
      else if (wc = Delim) and (start = FSource.FEntity) then
        Break;
      BufAppend(ToFill, wc);
    end
    else if (FSource.FEntity = start) or not ContextPop then    // #0
      FatalError('Literal has no closing quote', -1);
  until False;
  if aType = ltPubid then
    BufNormalize(ToFill, dummy);
end;

function TXMLReader.SkipUntilSeq(const Delim: TSetOfChar; c1: WideChar; c2: WideChar = #0): Boolean;
var
  wc: WideChar;
begin
>>>>>>> graemeg/cpstrnew
=======
var
  start: TObject;
  wc, Delim: WideChar;
  dummy: Boolean;
begin
=======
var
  start: TObject;
  wc, Delim: WideChar;
  dummy: Boolean;
begin
>>>>>>> origin/cpstrnew
  SkipQuote(Delim, Required);
  Result := (Delim <> #0);
  if not Result then
    Exit;
  ToFill.Length := 0;
  start := FSource.FEntity;
  repeat
    wc := FSource.SkipUntil(ToFill, LiteralDelims[aType]);
    if wc = '%' then       { ltEntity only }
    begin
      FSource.NextChar;
      CheckName;
      ExpectChar(';');
      if FSource.Kind = skInternalSubset then
        FatalError('PE reference not allowed here in internal subset', FName.Length+2);
      StartPE;
    end
    else if wc = '&' then  { ltEntity }
    begin
      if ParseRef(ToFill) then   // charRefs always expanded
        Continue;
      BufAppend(ToFill, '&');
      BufAppendChunk(ToFill, FName.Buffer, FName.Buffer + FName.Length);
      BufAppend(ToFill, ';');
    end
    else if wc <> #0 then
    begin
      FSource.NextChar;
      if (wc = #10) or (wc = #13) then
        wc := #32
      // terminating delimiter must be in the same context as the starting one
      else if (wc = Delim) and (start = FSource.FEntity) then
        Break;
      BufAppend(ToFill, wc);
    end
    else if (FSource.FEntity = start) or not ContextPop then    // #0
      FatalError('Literal has no closing quote', -1);
  until False;
  if aType = ltPubid then
    BufNormalize(ToFill, dummy);
=======
procedure TXMLReader.ExpectAttValue;    // [10]
var
  Delim: WideChar;
begin
  if (FCurChar <> '''') and (FCurChar <> '"') then
    RaiseExpectedQmark;
  Delim := FCurChar;
  GetChar;  // skip quote
  StoreLocation(FTokenStart);
  if not DoParseAttValue(Delim) then
    FatalError('Literal has no closing quote',-1);
  GetChar;
end;

procedure TXMLReader.SkipQuotedLiteral(out Literal: WideString; required: Boolean);
var
  Delim: WideChar;
begin
  if (FCurChar = '''') or (FCurChar = '"') then
  begin
    Delim := FCurChar;
    GetChar;  // skip quote
    StoreLocation(FTokenStart);
    FValue.Length := 0;
    while (FCurChar <> Delim) and (FCurChar <> #0) do
    begin
      BufAppend(FValue, FCurChar);
      GetChar;
    end;
    if not CheckForChar(Delim) then
      FatalError('Literal has no closing quote', -1);
    SetString(Literal, FValue.Buffer, FValue.Length);
  end
  else if required then
    RaiseExpectedQMark;
end;

procedure TXMLReader.SkipPubidLiteral(out Literal: WideString);         // [12]
var
  I: Integer;
  wc: WideChar;
begin
  SkipQuotedLiteral(Literal);
  for I := 1 to Length(Literal) do
  begin
    wc := Literal[I];
    if (wc > #255) or not (Char(ord(wc)) in PubidChars) then
      FatalError('Illegal Public ID literal', -1);
    if (wc = #10) or (wc = #13) then
      Literal[I] := #32;
  end;
>>>>>>> graemeg/fixes_2_2
end;

function TXMLReader.SkipUntilSeq(const Delim: TSetOfChar; c1: WideChar; c2: WideChar = #0): Boolean;
var
  wc: WideChar;
begin
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Result := False;
  StoreLocation(FTokenStart);
  repeat
    wc := FSource.SkipUntil(FValue, Delim);
    if wc <> #0 then
    begin
      FSource.NextChar;
      if (FValue.Length > ord(c2 <> #0)) then
      begin
        if (FValue.Buffer[FValue.Length-1] = c1) and
          ((c2 = #0) or ((c2 <> #0) and (FValue.Buffer[FValue.Length-2] = c2))) then
        begin
          Dec(FValue.Length, ord(c2 <> #0) + 1);
          Result := True;
          Exit;
        end;
      end;
      BufAppend(FValue, wc);
    end;
  until wc = #0;
=======
  ExpectString('--');
  StoreLocation(FTokenStart);
  FValue.Length := 0;
  repeat
    BufAppend(FValue, FCurChar);
    GetChar;
    with FValue do
      if (Length >= 2) and (Buffer[Length-1] = '-') and
      (Buffer[Length-2] = '-') then
      begin
        ExpectChar('>');
        Dec(Length, 2);
        DoComment(Buffer, Length);
        Exit;
      end;
  until FCurChar = #0;
  FatalError('Unterminated comment', -1);
>>>>>>> graemeg/fixes_2_2
end;

procedure TXMLReader.ParseComment(discard: Boolean);    // [15]
var
<<<<<<< HEAD
  SaveLength: Integer;
begin
  ExpectString('--');
  SaveLength := FValue.Length;
  if not SkipUntilSeq([#0, '-'], '-') then
    FatalError('Unterminated comment', -1);
  ExpectChar('>');

  if not discard then
  begin
    FCurrNode := @FNodeStack[FNesting+1];
    FCurrNode^.FNodeType := ntComment;
    FCurrNode^.FQName := nil;
    FCurrNode^.FValueStart := @FValue.Buffer[SaveLength];
    FCurrNode^.FValueLength := FValue.Length-SaveLength;
  end;
  FValue.Length := SaveLength;
end;

procedure TXMLReader.ParsePI;                    // [16]
begin
  FSource.NextChar;      // skip '?'
  CheckName;
=======
  Name, Value: WideString;
  PINode: TDOMProcessingInstruction;
begin
  GetChar;      // skip '?'
  Name := ExpectName;
>>>>>>> graemeg/fixes_2_2
  CheckNCName;
  with FName do
    if (Length = 3) and
     ((Buffer[0] = 'X') or (Buffer[0] = 'x')) and
     ((Buffer[1] = 'M') or (Buffer[1] = 'm')) and
     ((Buffer[2] = 'L') or (Buffer[2] = 'l')) then
  begin
<<<<<<< HEAD
    if not BufEquals(FName, 'xml') then
=======
    if Name <> 'xml' then
>>>>>>> graemeg/fixes_2_2
      FatalError('''xml'' is a reserved word; it must be lowercase', FName.Length)
    else
      FatalError('XML declaration is not allowed here', FName.Length);
  end;

<<<<<<< HEAD
  if FSource.FBuf^ <> '?' then
    SkipS(True);

  FValue.Length := 0;
  if not SkipUntilSeq(GT_Delim, '?') then
    FatalError('Unterminated processing instruction', -1);
  SetNodeInfoWithValue(ntProcessingInstruction,
    FNameTable.FindOrAdd(FName.Buffer, FName.Length));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
end;

function TXMLReader.CreatePINode: TDOMNode;
var
  NameStr, ValueStr: WideString;
begin
  SetString(NameStr, FName.Buffer, FName.Length);
  SetString(ValueStr, FValue.Buffer, FValue.Length);
  result := Doc.CreateProcessingInstruction(NameStr, ValueStr);
end;

=======
end;

function TXMLReader.CreatePINode: TDOMNode;
var
  NameStr, ValueStr: WideString;
begin
  SetString(NameStr, FName.Buffer, FName.Length);
  SetString(ValueStr, FValue.Buffer, FValue.Length);
  result := Doc.CreateProcessingInstruction(NameStr, ValueStr);
end;

>>>>>>> graemeg/cpstrnew
=======
end;

function TXMLReader.CreatePINode: TDOMNode;
var
  NameStr, ValueStr: WideString;
begin
  SetString(NameStr, FName.Buffer, FName.Length);
  SetString(ValueStr, FValue.Buffer, FValue.Length);
  result := Doc.CreateProcessingInstruction(NameStr, ValueStr);
end;

>>>>>>> graemeg/cpstrnew
=======
end;

function TXMLReader.CreatePINode: TDOMNode;
var
  NameStr, ValueStr: WideString;
begin
  SetString(NameStr, FName.Buffer, FName.Length);
  SetString(ValueStr, FValue.Buffer, FValue.Length);
  result := Doc.CreateProcessingInstruction(NameStr, ValueStr);
end;

>>>>>>> origin/cpstrnew
const
  vers: array[Boolean] of TXMLVersion = (xmlVersion10, xmlVersion11);

=======
  if FCurChar <> '?' then
    SkipS(True);

  FValue.Length := 0;
  StoreLocation(FTokenStart);
  repeat
    BufAppend(FValue, FCurChar);
    GetChar;
    with FValue do
      if (Length >= 2) and (Buffer[Length-1] = '>') and
        (Buffer[Length-2] = '?') then
      begin
        Dec(Length, 2);
        SetString(Value, Buffer, Length);
        // SAX: ContentHandler.ProcessingInstruction(Name, Value);

        if FCurrContentType = ctEmpty then
            ValidationError('Processing instructions are not allowed within EMPTY elements', []);

        PINode := Doc.CreateProcessingInstruction(Name, Value);
        if Assigned(FCursor) then
          FCursor.AppendChild(PINode)
        else  // to comply with certain tests, insert PI from DTD before DTD
          Doc.InsertBefore(PINode, FDocType);
        Exit;
      end;
  until FCurChar = #0;
  FatalError('Unterminated processing instruction', -1);
end;

>>>>>>> graemeg/fixes_2_2
procedure TXMLReader.ParseXmlOrTextDecl(TextDecl: Boolean);
var
  Delim: WideChar;
  buf: array[0..31] of WideChar;
  I: Integer;
begin
<<<<<<< HEAD
  SkipS(True);
  // [24] VersionInfo: optional in TextDecl, required in XmlDecl
  if (not TextDecl) or (FSource.FBuf^ = 'v') then
=======
  FCurChar := FSource.NextChar;  // don't update location here
  SkipS(True);
  // VersionInfo: optional in TextDecl, required in XmlDecl
  if (not TextDecl) or (FCurChar = 'v') then
>>>>>>> graemeg/fixes_2_2
  begin
    ExpectString('version');
    ExpectEq;
<<<<<<< HEAD
    SkipQuote(Delim);
    I := 0;
    while (I < 3) and (FSource.FBuf^ <> Delim) do
    begin
      buf[I] := FSource.FBuf^;
      Inc(I);
      FSource.NextChar;
    end;
    if (I <> 3) or (buf[0] <> '1') or (buf[1] <> '.') or
      ((buf[2] <> '0') and (buf[2] <> '1')) then
      FatalError('Illegal version number', -1);

    ExpectChar(Delim);
    FSource.FXMLVersion := vers[buf[2] = '1'];

    if TextDecl and (FSource.FXMLVersion = xmlVersion11) and not FXML11 then
      FatalError('XML 1.0 document cannot invoke XML 1.1 entities', -1);

    if TextDecl or (FSource.FBuf^ <> '?') then
=======
    SkipQuotedLiteral(TmpStr);
    IsXML11 := False;
    if TmpStr = '1.1' then     // Checking for bad chars is implied
      IsXML11 := True
    else if TmpStr <> '1.0' then
    { should be no whitespace in these literals, but that isn't checked now }
      FatalError('Illegal version number', -1);

    if not TextDecl then
    begin
      if doc.InheritsFrom(TXMLDocument) then
        TXMLDocument(doc).XMLVersion := TmpStr;
      if IsXML11 then
        XML11_BuildTables;
    end
    else   // parsing external entity
      if IsXML11 and not FXML11 then
        FatalError('XML 1.0 document cannot invoke XML 1.1 entities', -1);

    if FCurChar <> '?' then
>>>>>>> graemeg/fixes_2_2
      SkipS(True);
  end;

  // [80] EncodingDecl: required in TextDecl, optional in XmlDecl
  if TextDecl or (FSource.FBuf^ = 'e') then
  begin
    ExpectString('encoding');
    ExpectEq;
<<<<<<< HEAD
    SkipQuote(Delim);
    I := 0;
    while (I < 30) and (FSource.FBuf^ <> Delim) and (FSource.FBuf^ < #127) and
      ((Char(ord(FSource.FBuf^)) in ['A'..'Z', 'a'..'z']) or
      ((I > 0) and (Char(ord(FSource.FBuf^)) in ['0'..'9', '.', '-', '_']))) do
    begin
      buf[I] := FSource.FBuf^;
      Inc(I);
      FSource.NextChar;
    end;
    if not CheckForChar(Delim) then
      FatalError('Illegal encoding name', i);

    SetString(FSource.FXMLEncoding, buf, i);
    if not FSource.SetEncoding(FSource.FXMLEncoding) then  // <-- Wide2Ansi conversion here
      FatalError('Encoding ''%s'' is not supported', [FSource.FXMLEncoding], i+1);

    if FSource.FBuf^ <> '?' then
      SkipS(not TextDecl);
=======
    SkipQuotedLiteral(TmpStr);

    if not IsValidXmlEncoding(TmpStr) then
      FatalError('Illegal encoding name', -1);

    if not FSource.SetEncoding(TmpStr) then  // <-- Wide2Ansi conversion here
      FatalError('Encoding ''%s'' is not supported', [TmpStr], -1);
    // getting here means that specified encoding is supported
    // TODO: maybe assign the 'preferred' encoding name?
    if not TextDecl and doc.InheritsFrom(TXMLDocument) then
      TXMLDocument(doc).Encoding := TmpStr;

    if FCurChar <> '?' then
      SkipS(True);
>>>>>>> graemeg/fixes_2_2
  end;

  // [32] SDDecl: forbidden in TextDecl, optional in XmlDecl
  if (not TextDecl) and (FSource.FBuf^ = 's') then
  begin
    ExpectString('standalone');
    ExpectEq;
<<<<<<< HEAD
    SkipQuote(Delim);
    if FSource.Matches('yes') then
      FStandalone := True
    else if not FSource.Matches('no') then
      FatalError('Only "yes" or "no" are permitted as values of "standalone"', -1);
    ExpectChar(Delim);
=======
    SkipQuotedLiteral(TmpStr);
    if TmpStr = 'yes' then
      FStandalone := True
    else if TmpStr <> 'no' then
      FatalError('Only "yes" or "no" are permitted as values of "standalone"', -1);
>>>>>>> graemeg/fixes_2_2
    SkipS;
  end;

  ExpectString('?>');
  { Switch to 1.1 rules only after declaration is parsed completely. This is to
    ensure that NEL and LSEP within declaration are rejected (rmt-056, rmt-057) }
  if FSource.FXMLVersion = xmlVersion11 then
    FXML11 := True;
end;

procedure TXMLReader.DTDReloadHook;
var
  p: PWideChar;
begin
{ FSource converts CR, NEL and LSEP linebreaks to LF, and CR-NEL sequences to CR-LF.
  We must further remove the CR chars and have only LF's left. }
  p := FDTDStartPos;
  while p < FSource.FBuf do
  begin
    while (p < FSource.FBuf) and (p^ <> #13) do
      Inc(p);
    BufAppendChunk(FIntSubset, FDTDStartPos, p);
    if p^ = #13 then
      Inc(p);
    FDTDStartPos := p;
  end;
  FDTDStartPos := TXMLDecodingSource(FSource).FBufStart;
end;

procedure TXMLReader.DTDReloadHook;
begin
  BufAppendChunk(FIntSubset, FDTDStartPos, FSource.FBuf-FDTDStartPos);
  FDTDStartPos := TXMLDecodingSource(FSource).FBufStart + (FSource.FBufEnd-FSource.FBuf);
end;

procedure TXMLReader.ParseDoctypeDecl;    // [28]
var
  Src: TXMLCharSource;
begin
  if FState >= rsDTD then
    FatalError('Markup declaration is not allowed here');
<<<<<<< HEAD
  if FDisallowDoctype then
    FatalError('Document type is prohibited by parser settings');

  ExpectString('DOCTYPE');
  SkipS(True);

  FDocType := TDTDModel.Create(FNameTable);
  FDTDProcessed := True;    // assume success
  FState := rsDTD;

  FDocType.FName := ExpectName;
  SkipS(True);
  ParseExternalID(FDocType.FSystemID, FDocType.FPublicID, False);
  SkipS;

  if CheckForChar('[') then
  begin
    BufAllocate(FIntSubset, 256);
    FSource.Kind := skInternalSubset;
    try
      FDTDStartPos := FSource.FBuf;
      ParseMarkupDecl;
      DTDReloadHook;     // fetch last chunk
      SetString(FDocType.FInternalSubset, FIntSubset.Buffer, FIntSubset.Length);
    finally
      FreeMem(FIntSubset.Buffer);
      FSource.Kind := skNone;
    end;
    ExpectChar(']');
    SkipS;
  end;
  ExpectChar('>');

  if (FDocType.FSystemID <> '') then
  begin
    if ResolveResource(FDocType.FSystemID, FDocType.FPublicID, FSource.SystemID, Src) then
    begin
      Initialize(Src);
      try
        Src.Kind := skManualPop;
        ParseMarkupDecl;
      finally
        ContextPop(True);
      end;
    end
    else
    begin
      ValidationError('Unable to resolve external DTD subset', []);
      FDTDProcessed := FStandalone;
    end;
  end;
  FState := rsAfterDTD;
  FCurrNode^.FNodeType := ntDocumentType;
end;

procedure TXMLReader.ExpectEq;   // [25]
begin
  if FSource.FBuf^ <> '=' then
    SkipS;
  if FSource.FBuf^ <> '=' then
    FatalError('Expected "="');
  FSource.NextChar;
  SkipS;
end;


{ DTD stuff }

procedure TXMLReader.BadPENesting(S: TErrorSeverity);
begin
  if (S = esFatal) or FValidate then
    DoError(S, 'Parameter entities must be properly nested');
end;

procedure TXMLReader.StandaloneError(LineOffs: Integer);
begin
  ValidationError('Standalone constriant violation', [], LineOffs);
end;

function TXMLReader.ParseQuantity: TCPQuant;
begin
  case FSource.FBuf^ of
    '?': Result := cqZeroOrOnce;
    '*': Result := cqZeroOrMore;
    '+': Result := cqOnceOrMore;
  else
    Result := cqOnce;
    Exit;
  end;
  FSource.NextChar;
end;

function TXMLReader.FindOrCreateElDef: TElementDecl;
var
  p: PHashItem;
begin
  CheckName;
  p := FNameTable.FindOrAdd(FName.Buffer, FName.Length);
  Result := TElementDecl(p^.Data);
  if Result = nil then
  begin
    Result := TElementDecl.Create;
    p^.Data := Result;
  end;
end;

procedure TXMLReader.ExpectChoiceOrSeq(CP: TContentParticle; MustEndIn: TObject);     // [49], [50]
var
  Delim: WideChar;
=======

  ExpectString('DOCTYPE');
  SkipS(True);

  FDocType := TDOMDocumentTypeEx(TDOMDocumentType.Create(doc));
  FState := rsDTD;
  try
    FDocType.FName := ExpectName;
    SkipS(True);
    ParseExternalID(FDocType.FSystemID, FDocType.FPublicID, False);
    SkipS;
  finally
    // DONE: append node after its name has been set; always append to avoid leak
    Doc.AppendChild(FDocType);
    FCursor := nil;
  end;

  if CheckForChar('[') then
  begin
    BufAllocate(FIntSubset, 256);
    FSource.DTDSubsetType := dsInternal;
    try
      FDTDStartPos := FSource.FBuf;
      ParseMarkupDecl;
      DTDReloadHook;     // fetch last chunk
      SetString(FDocType.FInternalSubset, FIntSubset.Buffer, FIntSubset.Length);
    finally
      FreeMem(FIntSubset.Buffer);
      FSource.DTDSubsetType := dsNone;
    end;
    ExpectChar(']');
    SkipS;
  end;
  ExpectChar('>');

  if (FDocType.SystemID <> '') then
  begin
    if ResolveEntity(FDocType.SystemID, FDocType.PublicID, Src) then
    begin
      ContextPush(Src);
      try
        Src.DTDSubsetType := dsExternal;
        ParseMarkupDecl;
      finally
        Src.DTDSubsetType := dsNone;
        ContextPop;
      end;
    end
    else
      ValidationError('Unable to resolve external DTD subset', []);
  end;
  FCursor := Doc;
  ValidateDTD;
  FDocType.SetReadOnly(True);
end;

procedure TXMLReader.ExpectEq;   // [25]
begin
  if FSource.FBuf^ <> '=' then
    SkipS;
  if FSource.FBuf^ <> '=' then
    FatalError('Expected "="');
  GetChar;
  SkipS;
end;


{ DTD stuff }

procedure TXMLReader.BadPENesting(S: TErrorSeverity);
begin
  if (S = esFatal) or FValidate then
    DoError(S, 'Parameter entities must be properly nested');
end;

procedure TXMLReader.StandaloneError(LineOffs: Integer);
begin
  ValidationError('Standalone constriant violation', [], LineOffs);
end;

procedure TXMLReader.ParseQuantity(CP: TContentParticle);
begin
  if CheckForChar('?') then
    CP.CPQuant := cqZeroOrOnce
  else if CheckForChar('*') then
    CP.CPQuant := cqZeroOrMore
  else if CheckForChar('+') then
    CP.CPQuant := cqOnceOrMore;
end;

function TXMLReader.FindOrCreateElDef: TDOMElementDef;
var
  Token: WideString;
begin
  Token := ExpectName;
  Result := TDOMElementDef(FDocType.ElementDefs.GetNamedItem(Token));
  if Result = nil then
  begin
    Result := TDOMElementDef.Create(doc);
    Result.FNodeName := Token;
    FDocType.ElementDefs.SetNamedItem(Result);
  end;
end;

procedure TXMLReader.ExpectChoiceOrSeq(CP: TContentParticle);                  // [49], [50]
var
  Delim: WideChar;
  CurrentEntity: TObject;
>>>>>>> graemeg/fixes_2_2
  CurrentCP: TContentParticle;
begin
  Delim := #0;
  repeat
    CurrentCP := CP.Add;
    SkipWhitespace;
    if CheckForChar('(') then
<<<<<<< HEAD
      ExpectChoiceOrSeq(CurrentCP, FSource.FEntity)
    else
      CurrentCP.Def := FindOrCreateElDef;
=======
    begin
      CurrentEntity := FSource.FEntity;
      ExpectChoiceOrSeq(CurrentCP);
      if CurrentEntity <> FSource.FEntity then
        BadPENesting;
      GetChar;
    end
    else
      CurrentCP.Def := FindOrCreateElDef;

    ParseQuantity(CurrentCP);
>>>>>>> graemeg/fixes_2_2

    CurrentCP.CPQuant := ParseQuantity;
    SkipWhitespace;
    if FSource.FBuf^ = ')' then
      Break;
    if Delim = #0 then
    begin
      if (FSource.FBuf^ = '|') or (FSource.FBuf^ = ',') then
        Delim := FSource.FBuf^
      else
        FatalError('Expected pipe or comma delimiter');
    end
    else
<<<<<<< HEAD
      if FSource.FBuf^ <> Delim then
        FatalError(Delim);
    FSource.NextChar; // skip delimiter
  until False;
  if MustEndIn <> FSource.FEntity then
    BadPENesting;
  FSource.NextChar;

=======
      if FCurChar <> Delim then
        FatalError(Delim);
    GetChar; // skip delimiter
  until False;
>>>>>>> graemeg/fixes_2_2
  if Delim = '|' then
    CP.CPType := ctChoice
  else
    CP.CPType := ctSeq;    // '(foo)' is a sequence!
end;

procedure TXMLReader.ParseElementDecl;            // [45]
var
<<<<<<< HEAD
  ElDef: TElementDecl;
=======
  ElDef: TDOMElementDef;
>>>>>>> graemeg/fixes_2_2
  CurrentEntity: TObject;
  I: Integer;
  CP: TContentParticle;
  Typ: TElementContentType;
  ExtDecl: Boolean;
begin
  CP := nil;
<<<<<<< HEAD
  Typ := ctUndeclared;         // satisfy compiler
  ExpectWhitespace;
  ElDef := FindOrCreateElDef;
  if ElDef.ContentType <> ctUndeclared then
    ValidationErrorWithName('Duplicate declaration of element ''%s''', FName.Length);

  ExtDecl := FSource.Kind <> skInternalSubset;
=======
  Typ := ctAny;         // satisfy compiler
  ExpectWhitespace;
  ElDef := FindOrCreateElDef;
  if ElDef.HasElementDecl then
    ValidationError('Duplicate declaration of element ''%s''', [ElDef.TagName], FName.Length);

  ExtDecl := FSource.DTDSubsetType <> dsInternal;
>>>>>>> graemeg/fixes_2_2

  ExpectWhitespace;
  if FSource.Matches('EMPTY') then
    Typ := ctEmpty
  else if FSource.Matches('ANY') then
    Typ := ctAny
  else if CheckForChar('(') then
  begin
    CP := TContentParticle.Create;
    try
      CurrentEntity := FSource.FEntity;
      SkipWhitespace;
      if FSource.Matches('#PCDATA') then       // Mixed section [51]
      begin
        SkipWhitespace;
        Typ := ctMixed;
<<<<<<< HEAD
        while FSource.FBuf^ <> ')' do
=======
        while FCurChar <> ')' do
>>>>>>> graemeg/fixes_2_2
        begin
          ExpectChar('|');
          SkipWhitespace;

          with CP.Add do
          begin
            Def := FindOrCreateElDef;
            for I := CP.ChildCount-2 downto 0 do
              if Def = CP.Children[I].Def then
                ValidationError('Duplicate token in mixed section', [], FName.Length);
          end;
          SkipWhitespace;
        end;
        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
<<<<<<< HEAD
        FSource.NextChar;
        if (not CheckForChar('*')) and (CP.ChildCount > 0) then
          FatalError(WideChar('*'));
        CP.CPQuant := cqZeroOrMore;
        CP.CPType := ctChoice;
=======
        GetChar;
        if (not CheckForChar('*')) and (CP.ChildCount > 0) then
          FatalError(WideChar('*'));
>>>>>>> graemeg/fixes_2_2
      end
      else       // Children section [47]
      begin
        Typ := ctChildren;
<<<<<<< HEAD
        ExpectChoiceOrSeq(CP, CurrentEntity);
        CP.CPQuant := ParseQuantity;
=======
        ExpectChoiceOrSeq(CP);
        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        GetChar;
        ParseQuantity(CP);
>>>>>>> graemeg/fixes_2_2
      end;
    except
      CP.Free;
      raise;
    end;
  end
  else
    FatalError('Invalid content specification');
  // SAX: DeclHandler.ElementDecl(name, model);
<<<<<<< HEAD
  if FDTDProcessed and (ElDef.ContentType = ctUndeclared) then
  begin
    ElDef.ExternallyDeclared := ExtDecl;
=======
  if not ElDef.HasElementDecl then
  begin
    ElDef.HasElementDecl := True;
    ElDef.FExternallyDeclared := ExtDecl;
>>>>>>> graemeg/fixes_2_2
    ElDef.ContentType := Typ;
    ElDef.RootCP := CP;
  end
  else
    CP.Free;
end;


procedure TXMLReader.ParseNotationDecl;        // [82]
var
<<<<<<< HEAD
  NameStr, SysID, PubID: WideString;
begin
  ExpectWhitespace;
  NameStr := ExpectName;
=======
  Name, SysID, PubID: WideString;
begin
  ExpectWhitespace;
  Name := ExpectName;
>>>>>>> graemeg/fixes_2_2
  CheckNCName;
  ExpectWhitespace;
  if not ParseExternalID(SysID, PubID, True) then
    FatalError('Expected external or public ID');
<<<<<<< HEAD
  if FDTDProcessed then
    DoNotationDecl(NameStr, PubID, SysID);
=======
  DoNotationDecl(Name, PubID, SysID);
>>>>>>> graemeg/fixes_2_2
end;

const
  AttrDataTypeNames: array[TAttrDataType] of WideString = (
    'CDATA',
    'ID',
    'IDREF',
    'IDREFS',
    'ENTITY',
    'ENTITIES',
    'NMTOKEN',
    'NMTOKENS',
    'NOTATION'
  );

procedure TXMLReader.ParseAttlistDecl;         // [52]
var
<<<<<<< HEAD
  ElDef: TElementDecl;
  AttDef: TAttributeDef;
  dt: TAttrDataType;
  Found, DiscardIt: Boolean;
  Offsets: array [Boolean] of Integer;
  attrName: PHashItem;
=======
  ElDef: TDOMElementDef;
  AttDef: TDOMAttrDef;
  dt: TAttrDataType;
  Found, DiscardIt: Boolean;
>>>>>>> graemeg/fixes_2_2
begin
  ExpectWhitespace;
  ElDef := FindOrCreateElDef;
  SkipWhitespace;
  while FSource.FBuf^ <> '>' do
  begin
    CheckName;
    ExpectWhitespace;
<<<<<<< HEAD
    attrName := FNameTable.FindOrAdd(FName.Buffer, FName.Length);
    AttDef := TAttributeDef.Create(attrName, FColonPos);
    try
      AttDef.ExternallyDeclared := FSource.Kind <> skInternalSubset;
// In case of duplicate declaration of the same attribute, we must discard it,
// not modifying ElDef, and suppressing certain validation errors.
      DiscardIt := (not FDTDProcessed) or Assigned(ElDef.GetAttrDef(attrName));

      if CheckForChar('(') then     // [59]
      begin
        AttDef.DataType := dtNmToken;
=======
    AttDef := TDOMAttrDef.Create(doc);
    try
      AttDef.FExternallyDeclared := FSource.DTDSubsetType <> dsInternal;
      SetString(AttDef.FName, FName.Buffer, FName.Length);
// In case of duplicate declaration of the same attribute, we must discard it,
// not modifying ElDef, and suppressing certain validation errors.
      DiscardIt := Assigned(ElDef.GetAttributeNode(AttDef.Name));
      if not DiscardIt then
        ElDef.SetAttributeNode(AttDef);

      if CheckForChar('(') then     // [59]
      begin
        AttDef.FDataType := dtNmToken;
>>>>>>> graemeg/fixes_2_2
        repeat
          SkipWhitespace;
          CheckName([cnToken]);
          if not AttDef.AddEnumToken(FName.Buffer, FName.Length) then
            ValidationError('Duplicate token in enumerated attibute declaration', [], FName.Length);
          SkipWhitespace;
        until not CheckForChar('|');
        ExpectChar(')');
        ExpectWhitespace;
      end
      else
      begin
        StoreLocation(FTokenStart);
        // search topside-up so that e.g. NMTOKENS is matched before NMTOKEN
        for dt := dtNotation downto dtCData do
        begin
          Found := FSource.Matches(AttrDataTypeNames[dt]);
          if Found then
            Break;
        end;
        if Found and SkipWhitespace then
        begin
<<<<<<< HEAD
          AttDef.DataType := dt;
=======
          AttDef.FDataType := dt;
>>>>>>> graemeg/fixes_2_2
          if (dt = dtId) and not DiscardIt then
          begin
            if Assigned(ElDef.IDAttr) then
              ValidationError('Only one attribute of type ID is allowed per element',[])
            else
              ElDef.IDAttr := AttDef;
          end
          else if dt = dtNotation then          // no test cases for these ?!
          begin
            if not DiscardIt then
            begin
              if Assigned(ElDef.NotationAttr) then
                ValidationError('Only one attribute of type NOTATION is allowed per element',[])
              else
                ElDef.NotationAttr := AttDef;
              if ElDef.ContentType = ctEmpty then
                ValidationError('NOTATION attributes are not allowed on EMPTY elements',[]);
            end;
            ExpectChar('(');
            repeat
              SkipWhitespace;
<<<<<<< HEAD
              StoreLocation(FTokenStart);
=======
>>>>>>> graemeg/fixes_2_2
              CheckName;
              CheckNCName;
              if not AttDef.AddEnumToken(FName.Buffer, FName.Length) then
                ValidationError('Duplicate token in NOTATION attribute declaration',[], FName.Length);

<<<<<<< HEAD
              if (not DiscardIt) and FValidate then
                AddForwardRef(FName.Buffer, FName.Length);
=======
              if not DiscardIt then
                AddForwardRef(FNotationRefs, FName.Buffer, FName.Length);
>>>>>>> graemeg/fixes_2_2
              SkipWhitespace;
            until not CheckForChar('|');
            ExpectChar(')');
            ExpectWhitespace;
          end;
        end
        else if Found then
          ExpectWhitespace
        else
<<<<<<< HEAD
        begin
          // don't report 'expected whitespace' if token does not match completely
          Offsets[False] := 0;
          Offsets[True] := Length(AttrDataTypeNames[dt]);
          if Found and (FSource.FBuf^ < 'A') then
            ExpectWhitespace
          else
            FatalError('Illegal attribute type for ''%s''', [attrName^.Key], Offsets[Found]);
        end;
      end;
      StoreLocation(FTokenStart);
      if FSource.Matches('#REQUIRED') then
        AttDef.Default := adRequired
      else if FSource.Matches('#IMPLIED') then
        AttDef.Default := adImplied
      else if FSource.Matches('#FIXED') then
      begin
        AttDef.Default := adFixed;
        ExpectWhitespace;
      end
      else
        AttDef.Default := adDefault;

      if AttDef.Default in [adDefault, adFixed] then
=======
          FatalError('Illegal attribute type for ''%s''', [AttDef.Name]);
      end;
      StoreLocation(FTokenStart);
      if FSource.Matches('#REQUIRED') then
        AttDef.FDefault := adRequired
      else if FSource.Matches('#IMPLIED') then
        AttDef.FDefault := adImplied
      else if FSource.Matches('#FIXED') then
      begin
        AttDef.FDefault := adFixed;
        ExpectWhitespace;
      end
      else
        AttDef.FDefault := adDefault;

      if AttDef.FDefault in [adDefault, adFixed] then
>>>>>>> graemeg/fixes_2_2
      begin
        if AttDef.DataType = dtId then
          ValidationError('An attribute of type ID cannot have a default value',[]);

<<<<<<< HEAD
// See comments to valid-sa-094: PE expansion should be disabled in AttDef.
        ExpectAttValue(AttDef.Data, dt <> dtCDATA);

        if not ValidateAttrSyntax(AttDef, AttDef.Data^.FValueStr) then
          ValidationError('Default value for attribute ''%s'' has wrong syntax', [attrName^.Key]);
      end;
      // SAX: DeclHandler.AttributeDecl(...)
      if DiscardIt then
        AttDef.Free
      else
        ElDef.AddAttrDef(AttDef);
=======
        FCursor := AttDef;
// See comments to valid-sa-094: PE expansion should be disabled in AttDef.
// ExpectAttValue() does not recognize PEs anyway, so setting FRecognizePEs isn't needed
// Saving/restoring FCursor is also redundant because it is always nil here.
        ExpectAttValue;
        FCursor := nil;
        if not ValidateAttrSyntax(AttDef, AttDef.NodeValue) then
          ValidationError('Default value for attribute ''%s'' has wrong syntax', [AttDef.Name]);
      end;
      // SAX: DeclHandler.AttributeDecl(...)
      if DiscardIt then
        AttDef.Free;
>>>>>>> graemeg/fixes_2_2
    except
      if AttDef.OwnerElement = nil then
        AttDef.Free;
      raise;
    end;
    SkipWhitespace;
  end;
end;

<<<<<<< HEAD
procedure TXMLReader.ParseEntityDecl;        // [70]
var
  IsPE, Exists: Boolean;
  Entity: TEntityDecl;
  Map: THashTable;
  Item: PHashItem;
begin
  if not SkipWhitespace(True) then
    FatalError('Expected whitespace');
  IsPE := CheckForChar('%');
  if IsPE then                  // [72]
  begin
    ExpectWhitespace;
    if FPEMap = nil then
      FPEMap := THashTable.Create(64, True);
    Map := FPEMap;
  end
  else
    Map := FDocType.Entities;

  Entity := TEntityDecl.Create;
  try
    Entity.ExternallyDeclared := FSource.Kind <> skInternalSubset;
    Entity.FIsPE := IsPE;
    CheckName;
    CheckNCName;
    Item := Map.FindOrAdd(FName.Buffer, FName.Length, Exists);
    ExpectWhitespace;

    // remember where the entity is declared
    Entity.FURI := FSource.SystemID;

    if FEntityValue.Buffer = nil then
      BufAllocate(FEntityValue, 256);

    if ParseLiteral(FEntityValue, ltEntity, False) then
    begin
      SetString(Entity.FReplacementText, FEntityValue.Buffer, FEntityValue.Length);
      Entity.FCharCount := FEntityValue.Length;
      Entity.FStartLocation := FTokenStart;
    end
    else
    begin
      if not ParseExternalID(Entity.FSystemID, Entity.FPublicID, False) then
        FatalError('Expected entity value or external ID');

      if not IsPE then                // [76]
      begin
        if FSource.FBuf^ <> '>' then
          ExpectWhitespace;
        if FSource.Matches('NDATA') then
        begin
          ExpectWhitespace;
          StoreLocation(FTokenStart);
          Entity.FNotationName := ExpectName;
          if FValidate then
            AddForwardRef(FName.Buffer, FName.Length);
          // SAX: DTDHandler.UnparsedEntityDecl(...);
        end;
      end;
    end;
  except
    Entity.Free;
    raise;
  end;

  // Repeated declarations of same entity are legal but must be ignored
  if FDTDProcessed and not Exists then
  begin
    Item^.Data := Entity;
    Entity.FName := Item^.Key;
  end
  else
    Entity.Free;
end;


procedure TXMLReader.ParseMarkupDecl;        // [29]
var
  IncludeLevel: Integer;
  IgnoreLevel: Integer;
  CurrentEntity: TObject;
  IncludeLoc: TLocation;
  IgnoreLoc: TLocation;
  wc: WideChar;
  CondType: (ctUnknown, ctInclude, ctIgnore);
begin
  IncludeLevel := 0;
  IgnoreLevel := 0;
  repeat
    SkipWhitespace;

    if (FSource.FBuf^ = ']') and (IncludeLevel > 0) then
    begin
      ExpectString(']]>');
      Dec(IncludeLevel);
      Continue;
    end;

    if not CheckForChar('<') then
      Break;

    CurrentEntity := FSource.FEntity;

    if FSource.FBuf^ = '?' then
    begin
      ParsePI;
      doc.AppendChild(CreatePINode);
    end
    else
    begin
      ExpectChar('!');
      if FSource.FBuf^ = '-' then
        ParseComment(True)
      else if CheckForChar('[') then
      begin
        if FSource.Kind = skInternalSubset then
          FatalError('Conditional sections are not allowed in internal subset', 1);

        SkipWhitespace;

        CondType := ctUnknown;  // satisfy compiler
        if FSource.Matches('INCLUDE') then
          CondType := ctInclude
        else if FSource.Matches('IGNORE') then
          CondType := ctIgnore
        else
          FatalError('Expected "INCLUDE" or "IGNORE"');

        SkipWhitespace;
        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        ExpectChar('[');
        if CondType = ctInclude then
        begin
          if IncludeLevel = 0 then
            StoreLocation(IncludeLoc);
          Inc(IncludeLevel);
        end
        else if CondType = ctIgnore then
        begin
          StoreLocation(IgnoreLoc);
          IgnoreLevel := 1;
          repeat
            FValue.Length := 0;
            wc := FSource.SkipUntil(FValue, [#0, '<', ']']);
            if FSource.Matches('<![') then
              Inc(IgnoreLevel)
            else if FSource.Matches(']]>') then
              Dec(IgnoreLevel)
            else if wc <> #0 then
              FSource.NextChar
            else // PE's aren't recognized in ignore section, cannot ContextPop()
              DoErrorPos(esFatal, 'IGNORE section is not closed', IgnoreLoc);
          until IgnoreLevel=0;
        end;
      end
      else
      begin
        FInsideDecl := True;
        if FSource.Matches('ELEMENT') then
          ParseElementDecl
        else if FSource.Matches('ENTITY') then
          ParseEntityDecl
        else if FSource.Matches('ATTLIST') then
          ParseAttlistDecl
        else if FSource.Matches('NOTATION') then
          ParseNotationDecl
        else
          FatalError('Illegal markup declaration');

        SkipWhitespace;

        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        ExpectChar('>');
        FInsideDecl := False;
      end;
    end;
  until False;
  if IncludeLevel > 0 then
    DoErrorPos(esFatal, 'INCLUDE section is not closed', IncludeLoc);
  if (FSource.Kind = skInternalSubset) and (FSource.FBuf^ = ']') then
    Exit;
  if FSource.FBuf^ <> #0 then
    FatalError('Illegal character in DTD');
end;

procedure TXMLReader.ProcessDTD(ASource: TXMLCharSource);
begin
  doc := TXMLDocument.Create;
  FNameTable := doc.Names;
  FDocType := TDTDModel.Create(FNameTable);
  // TODO: DTD labeled version 1.1 will be rejected - must set FXML11 flag
  doc.AppendChild(TDOMDocumentType.Create(doc, FDocType));
  NSPrepare;
  Initialize(ASource);
  ParseMarkupDecl;
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/cpstrnew

procedure TXMLReader.LoadEntity(AEntity: TEntityDecl);
var
  InnerReader: TXMLReader;
  Src: TXMLCharSource;
  Ent: TDOMEntityEx;
  DoctypeNode: TDOMDocumentType;
<<<<<<< HEAD
begin
  DoctypeNode := doc.DocType;
  if DoctypeNode = nil then
    Exit;
  Ent := TDOMEntityEx(DocTypeNode.Entities.GetNamedItem(AEntity.FName));
  if Ent = nil then
    Exit;
  InnerReader := TXMLReader.Create(FCtrl);
  try
    InnerReader.FAttrTag := FAttrTag;
    InnerReader.FDocType := FDocType.Reference;
    EntityToSource(AEntity, Src);
    Ent.SetReadOnly(False);
    if Assigned(Src) then
      InnerReader.ProcessFragment(Src, Ent);
    AEntity.FResolved := True;
  finally
    FAttrTag := InnerReader.FAttrTag;
    InnerReader.Free;
    AEntity.FOnStack := False;
    Ent.SetReadOnly(True);
=======
begin
  DoctypeNode := doc.DocType;
  if DoctypeNode = nil then
    Exit;
  Ent := TDOMEntityEx(DocTypeNode.Entities.GetNamedItem(AEntity.FName));
  if Ent = nil then
    Exit;
  InnerReader := TXMLReader.Create(FCtrl);
  try
    InnerReader.FAttrTag := FAttrTag;
    InnerReader.FDocType := FDocType.Reference;
    EntityToSource(AEntity, Src);
    Ent.SetReadOnly(False);
    if Assigned(Src) then
      InnerReader.ProcessFragment(Src, Ent);
    AEntity.FResolved := True;
  finally
    FAttrTag := InnerReader.FAttrTag;
    InnerReader.Free;
    AEntity.FOnStack := False;
    Ent.SetReadOnly(True);
  end;
end;

procedure TXMLReader.ValidateCurrentNode;
var
  ElDef: TElementDecl;
  AttDef: TAttributeDef;
  attr: PNodeData;
  i: Integer;
begin
  case FCurrNode^.FNodeType of
    ntElement:
      begin
        if (FNesting = 1) and (not FFragmentMode) then
        begin
          if Assigned(FDocType) then
          begin
            if FDocType.FName <> FCurrNode^.FQName^.Key then
              DoErrorPos(esError, 'Root element name does not match DTD', FCurrNode^.FLoc);
          end
          else
            DoErrorPos(esError, 'Missing DTD', FCurrNode^.FLoc);
        end;
        ElDef := TElementDecl(FCurrNode^.FQName^.Data);
        if (ElDef = nil) or (ElDef.ContentType = ctUndeclared) then
          DoErrorPos(esError, 'Using undeclared element ''%s''',[FCurrNode^.FQName^.Key], FCurrNode^.FLoc);

        if not FValidators[FValidatorNesting].IsElementAllowed(ElDef) then
          DoErrorPos(esError, 'Element ''%s'' is not allowed in this context',[FCurrNode^.FQName^.Key], FCurrNode^.FLoc);

        PushVC(ElDef);

        { Validate attributes }
        for i := 1 to FAttrCount do
        begin
          attr := @FNodeStack[FNesting+i];
          AttDef := TAttributeDef(attr^.FTypeInfo);
          if AttDef = nil then
            DoErrorPos(esError, 'Using undeclared attribute ''%s'' on element ''%s''',
              [attr^.FQName^.Key, FCurrNode^.FQName^.Key], attr^.FLoc)
          else if ((AttDef.DataType <> dtCdata) or (AttDef.Default = adFixed)) then
          begin
            if FStandalone and AttDef.ExternallyDeclared then
              { TODO: perhaps should use different and more descriptive messages }
              if attr^.FDenormalized then
                DoErrorPos(esError, 'Standalone constraint violation', attr^.FLoc2)
              else if i > FSpecifiedAttrs then
                DoError(esError, 'Standalone constraint violation');

            // TODO: what about normalization of AttDef.Value? (Currently it IS normalized)
            if (AttDef.Default = adFixed) and (AttDef.Data^.FValueStr <> attr^.FValueStr) then
              DoErrorPos(esError, 'Value of attribute ''%s'' does not match its #FIXED default',[attr^.FQName^.Key], attr^.FLoc2);
            if not ValidateAttrSyntax(AttDef, attr^.FValueStr) then
              DoErrorPos(esError, 'Attribute ''%s'' type mismatch', [attr^.FQName^.Key], attr^.FLoc2);
            ValidateAttrValue(AttDef, attr);
          end;
        end;
      end;

    ntEndElement:
      begin
        if FValidators[FValidatorNesting].Incomplete then
          ValidationError('Element ''%s'' is missing required sub-elements', [FCurrNode^.FQName^.Key], -1);
        if FValidatorNesting > 0 then
          Dec(FValidatorNesting);
      end;

    ntText, ntSignificantWhitespace:
      case FValidators[FValidatorNesting].FContentType of
        ctChildren:
          if FCurrNode^.FNodeType = ntText then
            ValidationError('Character data is not allowed in element-only content',[])
          else
          begin
            if FValidators[FValidatorNesting].FSaViolation then
              StandaloneError(-1);
            FCurrNode^.FNodeType := ntWhitespace;
          end;
        ctEmpty:
          ValidationError('Character data is not allowed in EMPTY elements', []);
      end;

    ntCDATA:
      if FValidators[FValidatorNesting].FContentType = ctChildren then
        ValidationError('CDATA sections are not allowed in element-only content',[]);

    ntProcessingInstruction:
      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('Processing instructions are not allowed within EMPTY elements', []);

    ntComment:
      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('Comments are not allowed within EMPTY elements', []);

    ntDocumentType:
      ValidateDTD;
  end;
end;

procedure TXMLReader.HandleEntityStart;
begin
  { FNesting+1 is available due to overallocation in AllocNodeData() }
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := ntEntityReference;
  FCurrNode^.FQName := FNameTable.FindOrAdd(FName.Buffer, FName.Length);
  FCurrNode^.FValueStart := nil;
  FCurrNode^.FValueLength := 0;
end;

procedure TXMLReader.HandleEntityEnd;
begin
  ContextPop(True);
  if FNesting > 0 then Dec(FNesting);
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := ntEndEntity;
  // TODO: other properties of FCurrNode
  FNext := xtText;
end;

procedure TXMLReader.ResolveEntity;
begin
  if FCurrNode^.FNodeType <> ntEntityReference then
    raise EInvalidOperation.Create('Wrong node type');

  {... here must actually call EntityCheck, but it's called in main loop}

  FNext := xtPushEntity;
end;

procedure TXMLReader.DoStartEntity;
var
  src: TXMLCharSource;
begin
  Inc(FNesting);
  FCurrNode := AllocNodeData(FNesting);
  if Assigned(FCurrEntity) then
    ContextPush(FCurrEntity)
  else
  begin
  // Undefined entity -- use a dummy inputsource, in order to get a matching EndEntity event
    src := TXMLCharSource.Create('');
    src.Kind := skManualPop;
    Initialize(src);
  end;
  FNext := xtText;
end;

function TXMLReader.DoStartElement: TDOMElement;
var
  Attr: TDOMAttr;
  i: Integer;
begin
  with FCurrNode^.FQName^ do
    Result := doc.CreateElementBuf(PWideChar(Key), Length(Key));
  if Assigned(FCurrNode^.FNsUri) then
    Result.SetNSI(FCurrNode^.FNsUri^.Key, FCurrNode^.FColonPos+1);

  for i := 1 to FAttrCount do
  begin
    Attr := LoadAttribute(doc, @FNodeStack[FNesting+i]);
    Result.SetAttributeNode(Attr);
    // Attach element to ID map entry if necessary
    if Assigned(FNodeStack[FNesting+i].FIDEntry) then
      FNodeStack[FNesting+i].FIDEntry^.Data := Result;
>>>>>>> origin/cpstrnew
  end;
end;

procedure TXMLReader.ValidateCurrentNode;
var
  ElDef: TElementDecl;
  AttDef: TAttributeDef;
  attr: PNodeData;
  i: Integer;
begin
  case FCurrNode^.FNodeType of
    ntElement:
      begin
        if (FNesting = 1) and (not FFragmentMode) then
        begin
          if Assigned(FDocType) then
          begin
            if FDocType.FName <> FCurrNode^.FQName^.Key then
              DoErrorPos(esError, 'Root element name does not match DTD', FCurrNode^.FLoc);
          end
          else
            DoErrorPos(esError, 'Missing DTD', FCurrNode^.FLoc);
        end;
        ElDef := TElementDecl(FCurrNode^.FQName^.Data);
        if (ElDef = nil) or (ElDef.ContentType = ctUndeclared) then
          DoErrorPos(esError, 'Using undeclared element ''%s''',[FCurrNode^.FQName^.Key], FCurrNode^.FLoc);

        if not FValidators[FValidatorNesting].IsElementAllowed(ElDef) then
          DoErrorPos(esError, 'Element ''%s'' is not allowed in this context',[FCurrNode^.FQName^.Key], FCurrNode^.FLoc);

        PushVC(ElDef);

        { Validate attributes }
        for i := 1 to FAttrCount do
        begin
          attr := @FNodeStack[FNesting+i];
          AttDef := TAttributeDef(attr^.FTypeInfo);
          if AttDef = nil then
            DoErrorPos(esError, 'Using undeclared attribute ''%s'' on element ''%s''',
              [attr^.FQName^.Key, FCurrNode^.FQName^.Key], attr^.FLoc)
          else if ((AttDef.DataType <> dtCdata) or (AttDef.Default = adFixed)) then
          begin
            if FStandalone and AttDef.ExternallyDeclared then
              { TODO: perhaps should use different and more descriptive messages }
              if attr^.FDenormalized then
                DoErrorPos(esError, 'Standalone constraint violation', attr^.FLoc2)
              else if i > FSpecifiedAttrs then
                DoError(esError, 'Standalone constraint violation');

            // TODO: what about normalization of AttDef.Value? (Currently it IS normalized)
            if (AttDef.Default = adFixed) and (AttDef.Data^.FValueStr <> attr^.FValueStr) then
              DoErrorPos(esError, 'Value of attribute ''%s'' does not match its #FIXED default',[attr^.FQName^.Key], attr^.FLoc2);
            if not ValidateAttrSyntax(AttDef, attr^.FValueStr) then
              DoErrorPos(esError, 'Attribute ''%s'' type mismatch', [attr^.FQName^.Key], attr^.FLoc2);
            ValidateAttrValue(AttDef, attr);
          end;
        end;
      end;

    ntEndElement:
      begin
        if FValidators[FValidatorNesting].Incomplete then
          ValidationError('Element ''%s'' is missing required sub-elements', [FCurrNode^.FQName^.Key], -1);
        if FValidatorNesting > 0 then
          Dec(FValidatorNesting);
      end;

    ntText, ntSignificantWhitespace:
      case FValidators[FValidatorNesting].FContentType of
        ctChildren:
          if FCurrNode^.FNodeType = ntText then
            ValidationError('Character data is not allowed in element-only content',[])
          else
          begin
            if FValidators[FValidatorNesting].FSaViolation then
              StandaloneError(-1);
            FCurrNode^.FNodeType := ntWhitespace;
          end;
        ctEmpty:
          ValidationError('Character data is not allowed in EMPTY elements', []);
      end;

    ntCDATA:
      if FValidators[FValidatorNesting].FContentType = ctChildren then
        ValidationError('CDATA sections are not allowed in element-only content',[]);

    ntProcessingInstruction:
      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('Processing instructions are not allowed within EMPTY elements', []);

    ntComment:
      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('Comments are not allowed within EMPTY elements', []);

    ntDocumentType:
      ValidateDTD;
  end;
end;

procedure TXMLReader.HandleEntityStart;
begin
  { FNesting+1 is available due to overallocation in AllocNodeData() }
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := ntEntityReference;
  FCurrNode^.FQName := FNameTable.FindOrAdd(FName.Buffer, FName.Length);
  FCurrNode^.FValueStart := nil;
  FCurrNode^.FValueLength := 0;
end;

procedure TXMLReader.HandleEntityEnd;
begin
  ContextPop(True);
  if FNesting > 0 then Dec(FNesting);
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := ntEndEntity;
  // TODO: other properties of FCurrNode
  FNext := xtText;
end;

procedure TXMLReader.ResolveEntity;
begin
  if FCurrNode^.FNodeType <> ntEntityReference then
    raise EInvalidOperation.Create('Wrong node type');

  {... here must actually call EntityCheck, but it's called in main loop}

  FNext := xtPushEntity;
end;

procedure TXMLReader.DoStartEntity;
var
  src: TXMLCharSource;
begin
  Inc(FNesting);
  FCurrNode := AllocNodeData(FNesting);
  if Assigned(FCurrEntity) then
    ContextPush(FCurrEntity)
  else
  begin
  // Undefined entity -- use a dummy inputsource, in order to get a matching EndEntity event
    src := TXMLCharSource.Create('');
    src.Kind := skManualPop;
    Initialize(src);
  end;
  FNext := xtText;
end;

function TXMLReader.DoStartElement: TDOMElement;
var
  Attr: TDOMAttr;
  i: Integer;
begin
  with FCurrNode^.FQName^ do
    Result := doc.CreateElementBuf(PWideChar(Key), Length(Key));
  if Assigned(FCurrNode^.FNsUri) then
    Result.SetNSI(FCurrNode^.FNsUri^.Key, FCurrNode^.FColonPos+1);

  for i := 1 to FAttrCount do
  begin
    Attr := LoadAttribute(doc, @FNodeStack[FNesting+i]);
    Result.SetAttributeNode(Attr);
    // Attach element to ID map entry if necessary
    if Assigned(FNodeStack[FNesting+i].FIDEntry) then
      FNodeStack[FNesting+i].FIDEntry^.Data := Result;
  end;
end;

// The code below does the bulk of the parsing, and must be as fast as possible.
// To minimize CPU cache effects, methods from different classes are kept together
>>>>>>> graemeg/cpstrnew

procedure TXMLReader.LoadEntity(AEntity: TEntityDecl);
var
  InnerReader: TXMLReader;
  Src: TXMLCharSource;
  Ent: TDOMEntityEx;
  DoctypeNode: TDOMDocumentType;
begin
  DoctypeNode := doc.DocType;
  if DoctypeNode = nil then
    Exit;
  Ent := TDOMEntityEx(DocTypeNode.Entities.GetNamedItem(AEntity.FName));
  if Ent = nil then
    Exit;
  InnerReader := TXMLReader.Create(FCtrl);
  try
    InnerReader.FAttrTag := FAttrTag;
    InnerReader.FDocType := FDocType.Reference;
    EntityToSource(AEntity, Src);
    Ent.SetReadOnly(False);
    if Assigned(Src) then
      InnerReader.ProcessFragment(Src, Ent);
    AEntity.FResolved := True;
  finally
    FAttrTag := InnerReader.FAttrTag;
    InnerReader.Free;
    AEntity.FOnStack := False;
    Ent.SetReadOnly(True);
  end;
end;

<<<<<<< HEAD
procedure TXMLReader.ValidateCurrentNode;
var
  ElDef: TElementDecl;
  AttDef: TAttributeDef;
  attr: PNodeData;
  i: Integer;
begin
  case FCurrNode^.FNodeType of
    ntElement:
      begin
        if (FNesting = 1) and (not FFragmentMode) then
        begin
          if Assigned(FDocType) then
          begin
            if FDocType.FName <> FCurrNode^.FQName^.Key then
              DoErrorPos(esError, 'Root element name does not match DTD', FCurrNode^.FLoc);
          end
          else
            DoErrorPos(esError, 'Missing DTD', FCurrNode^.FLoc);
        end;
        ElDef := TElementDecl(FCurrNode^.FQName^.Data);
        if (ElDef = nil) or (ElDef.ContentType = ctUndeclared) then
          DoErrorPos(esError, 'Using undeclared element ''%s''',[FCurrNode^.FQName^.Key], FCurrNode^.FLoc);

        if not FValidators[FValidatorNesting].IsElementAllowed(ElDef) then
          DoErrorPos(esError, 'Element ''%s'' is not allowed in this context',[FCurrNode^.FQName^.Key], FCurrNode^.FLoc);

        PushVC(ElDef);

        { Validate attributes }
        for i := 1 to FAttrCount do
        begin
          attr := @FNodeStack[FNesting+i];
          AttDef := TAttributeDef(attr^.FTypeInfo);
          if AttDef = nil then
            DoErrorPos(esError, 'Using undeclared attribute ''%s'' on element ''%s''',
              [attr^.FQName^.Key, FCurrNode^.FQName^.Key], attr^.FLoc)
          else if ((AttDef.DataType <> dtCdata) or (AttDef.Default = adFixed)) then
          begin
            if FStandalone and AttDef.ExternallyDeclared then
              { TODO: perhaps should use different and more descriptive messages }
              if attr^.FDenormalized then
                DoErrorPos(esError, 'Standalone constraint violation', attr^.FLoc2)
              else if i > FSpecifiedAttrs then
                DoError(esError, 'Standalone constraint violation');

            // TODO: what about normalization of AttDef.Value? (Currently it IS normalized)
            if (AttDef.Default = adFixed) and (AttDef.Data^.FValueStr <> attr^.FValueStr) then
              DoErrorPos(esError, 'Value of attribute ''%s'' does not match its #FIXED default',[attr^.FQName^.Key], attr^.FLoc2);
            if not ValidateAttrSyntax(AttDef, attr^.FValueStr) then
              DoErrorPos(esError, 'Attribute ''%s'' type mismatch', [attr^.FQName^.Key], attr^.FLoc2);
            ValidateAttrValue(AttDef, attr);
          end;
        end;
      end;

    ntEndElement:
      begin
        if FValidators[FValidatorNesting].Incomplete then
          ValidationError('Element ''%s'' is missing required sub-elements', [FCurrNode^.FQName^.Key], -1);
        if FValidatorNesting > 0 then
          Dec(FValidatorNesting);
      end;

    ntText, ntSignificantWhitespace:
      case FValidators[FValidatorNesting].FContentType of
        ctChildren:
          if FCurrNode^.FNodeType = ntText then
            ValidationError('Character data is not allowed in element-only content',[])
          else
          begin
            if FValidators[FValidatorNesting].FSaViolation then
              StandaloneError(-1);
            FCurrNode^.FNodeType := ntWhitespace;
          end;
        ctEmpty:
          ValidationError('Character data is not allowed in EMPTY elements', []);
      end;

    ntCDATA:
      if FValidators[FValidatorNesting].FContentType = ctChildren then
        ValidationError('CDATA sections are not allowed in element-only content',[]);

    ntProcessingInstruction:
      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('Processing instructions are not allowed within EMPTY elements', []);

    ntComment:
      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('Comments are not allowed within EMPTY elements', []);

    ntDocumentType:
      ValidateDTD;
  end;
end;

procedure TXMLReader.HandleEntityStart;
begin
  { FNesting+1 is available due to overallocation in AllocNodeData() }
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := ntEntityReference;
  FCurrNode^.FQName := FNameTable.FindOrAdd(FName.Buffer, FName.Length);
  FCurrNode^.FValueStart := nil;
  FCurrNode^.FValueLength := 0;
end;

procedure TXMLReader.HandleEntityEnd;
begin
  ContextPop(True);
  if FNesting > 0 then Dec(FNesting);
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := ntEndEntity;
  // TODO: other properties of FCurrNode
  FNext := xtText;
end;

procedure TXMLReader.ResolveEntity;
<<<<<<< HEAD
begin
  if FCurrNode^.FNodeType <> ntEntityReference then
    raise EInvalidOperation.Create('Wrong node type');

  {... here must actually call EntityCheck, but it's called in main loop}

  FNext := xtPushEntity;
end;

procedure TXMLReader.DoStartEntity;
var
  src: TXMLCharSource;
begin
=======
begin
  if FCurrNode^.FNodeType <> ntEntityReference then
    raise EInvalidOperation.Create('Wrong node type');

  {... here must actually call EntityCheck, but it's called in main loop}

  FNext := xtPushEntity;
end;

procedure TXMLReader.DoStartEntity;
var
  src: TXMLCharSource;
begin
>>>>>>> graemeg/cpstrnew
  Inc(FNesting);
  FCurrNode := AllocNodeData(FNesting);
  if Assigned(FCurrEntity) then
    ContextPush(FCurrEntity)
  else
  begin
  // Undefined entity -- use a dummy inputsource, in order to get a matching EndEntity event
    src := TXMLCharSource.Create('');
    src.Kind := skManualPop;
    Initialize(src);
  end;
  FNext := xtText;
end;

function TXMLReader.DoStartElement: TDOMElement;
var
  Attr: TDOMAttr;
  i: Integer;
begin
  with FCurrNode^.FQName^ do
    Result := doc.CreateElementBuf(PWideChar(Key), Length(Key));
  if Assigned(FCurrNode^.FNsUri) then
    Result.SetNSI(FCurrNode^.FNsUri^.Key, FCurrNode^.FColonPos+1);

  for i := 1 to FAttrCount do
  begin
    Attr := LoadAttribute(doc, @FNodeStack[FNesting+i]);
    Result.SetAttributeNode(Attr);
    // Attach element to ID map entry if necessary
    if Assigned(FNodeStack[FNesting+i].FIDEntry) then
      FNodeStack[FNesting+i].FIDEntry^.Data := Result;
  end;
end;

// The code below does the bulk of the parsing, and must be as fast as possible.
// To minimize CPU cache effects, methods from different classes are kept together

function TXMLDecodingSource.SkipUntil(var ToFill: TWideCharBuf; const Delim: TSetOfChar;
  wsflag: PBoolean): WideChar;
var
  old: PWideChar;
  nonws: Boolean;
  wc: WideChar;
begin
  nonws := False;
  repeat
    old := FBuf;
    repeat
      wc := FBuf^;
      if (wc = #10) or (wc = #13) or (FXML11Rules and ((wc = #$85) or
        (wc = #$2028))) then
      begin
// strictly this is needed only for 2-byte lineendings
        BufAppendChunk(ToFill, old, FBuf);
        NewLine;
        old := FBuf;
        wc := FBuf^
      end
      else if ((wc < #32) and (not ((wc = #0) and (FBuf >= FBufEnd))) and
        (wc <> #9)) or (wc > #$FFFD) or
        (FXML11Rules and (wc >= #$7F) and (wc <= #$9F)) then
             FReader.FatalError('Invalid character');
      if (wc < #255) and (Char(ord(wc)) in Delim) then
        Break;
// the checks above filter away everything below #32 that isn't a whitespace
      if wc > #32 then
        nonws := True;
      Inc(FBuf);
    until False;
    Result := wc;
    BufAppendChunk(ToFill, old, FBuf);
  until (Result <> #0) or (not Reload);
  if Assigned(wsflag) then
    wsflag^ := wsflag^ or nonws;
end;

const
  TextDelims: array[Boolean] of TSetOfChar = (
    [#0, '<', '&', '>'],
    [#0, '>']
  );

  textNodeTypes: array[Boolean] of TXMLNodeType = (
    ntSignificantWhitespace,
    ntText
  );

procedure TXMLReader.ParseContent(parent: TDOMNode_WithChildren);
<<<<<<< HEAD
<<<<<<< HEAD
var
  cursor: TDOMNode_WithChildren;
  element: TDOMElement;
begin
  cursor := parent;
  while Read do
  begin
    if FValidate then
      ValidateCurrentNode;

    case FCurrNode^.FNodeType of
      ntText:
        cursor.InternalAppend(doc.CreateTextNodeBuf(FValue.Buffer, FValue.Length, False));

      ntWhitespace, ntSignificantWhitespace:
        if FPreserveWhitespace then
          cursor.InternalAppend(doc.CreateTextNodeBuf(FValue.Buffer, FValue.Length, FCurrNode^.FNodeType = ntWhitespace));

=======
const
  TextDelims: array[Boolean] of TSetOfChar = (
    [#0, '<', '&', '>'],
    [#0, '>']
  );

  textNodeTypes: array[Boolean] of TXMLNodeType = (
    ntSignificantWhitespace,
    ntText
  );

procedure TXMLReader.ParseContent(parent: TDOMNode_WithChildren);
var
  cursor: TDOMNode_WithChildren;
  element: TDOMElement;
begin
  cursor := parent;
  while Read do
  begin
    if FValidate then
      ValidateCurrentNode;

    case FCurrNode^.FNodeType of
      ntText:
        cursor.InternalAppend(doc.CreateTextNodeBuf(FValue.Buffer, FValue.Length, False));

      ntWhitespace, ntSignificantWhitespace:
        if FPreserveWhitespace then
          cursor.InternalAppend(doc.CreateTextNodeBuf(FValue.Buffer, FValue.Length, FCurrNode^.FNodeType = ntWhitespace));

>>>>>>> graemeg/cpstrnew
      ntCDATA:
        cursor.InternalAppend(DoCDSect(FValue.Buffer, FValue.Length));

      ntProcessingInstruction:
        cursor.InternalAppend(CreatePINode);

      ntComment:
        if not FIgnoreComments then
          cursor.InternalAppend(doc.CreateCommentBuf(FCurrNode^.FValueStart, FCurrNode^.FValueLength));

      ntElement:
        begin
          element := DoStartElement;
          cursor.InternalAppend(element);
          cursor := element;
        end;

      ntEndElement:
          cursor := TDOMNode_WithChildren(cursor.ParentNode);

      ntDocumentType:
        if not FCanonical then
          cursor.InternalAppend(TDOMDocumentType.Create(doc, FDocType));

      ntEntityReference:
        cursor.InternalAppend(doc.CreateEntityReference(FCurrNode^.FQName^.Key));
<<<<<<< HEAD
    end;
  end;
end;

function TXMLReader.Read: Boolean;
var
  nonWs: Boolean;
  wc: WideChar;
  InCDATA: Boolean;
  tok: TXMLToken;
begin
  if FNext = xtPopEmptyElement then
  begin
    FNext := xtPopElement;
    FCurrNode^.FNodeType := ntEndElement;
    if FAttrCleanupFlag then
      CleanupAttributes;
    FAttrCount := 0;
    Result := True;
    Exit;
  end;
  if FNext = xtPushElement then
  begin
    if FAttrCleanupFlag then
      CleanupAttributes;
    FAttrCount := 0;
    FNext := xtText;
  end
  else if FNext = xtPopElement then
  begin
    if FNamespaces then
      FNSHelper.EndElement;
    PopVC;
  end
  else if FNext = xtPushEntity then
    DoStartEntity;

  InCDATA := (FNext = xtCDSect);
  StoreLocation(FTokenStart);
  nonWs := False;
  FValue.Length := 0;

  if FNext in [xtCDSect, xtText] then
  repeat
    wc := FSource.SkipUntil(FValue, TextDelims[InCDATA], @nonWs);
    if wc = '<' then
    begin
      Inc(FSource.FBuf);
      if FSource.FBufEnd < FSource.FBuf + 2 then
        FSource.Reload;
      if FSource.FBuf^ = '/' then
        tok := xtEndElement
      else if CheckName([cnOptional]) then
        tok := xtElement
      else if FSource.FBuf^ = '!' then
      begin
        Inc(FSource.FBuf);
        if FSource.FBuf^ = '[' then
        begin
          ExpectString('[CDATA[');
          if FState <> rsRoot then
            FatalError('Illegal at document level');
          StoreLocation(FTokenStart);
          InCDATA := True;
          if FCDSectionsAsText or (FValue.Length = 0) then
            Continue;
          tok := xtCDSect;
        end
        else if FSource.FBuf^ = '-' then
        begin
        { Ignoring comments is tricky in validating mode; discarding a comment which
          is the only child of an EMPTY element will make that element erroneously appear
          as valid. Therefore, at this point we discard only comments which are preceded
          by some text (since presence of text already renders an EMPTY element invalid).
          Other comments should be reported to validation part and discarded there. }
          if FIgnoreComments and (FValue.Length > 0) then
          begin
            ParseComment(True);
            Continue;
          end;
          tok := xtComment;
        end
        else
          tok := xtDoctype;
      end
      else if FSource.FBuf^ = '?' then
        tok := xtPI
      else
        RaiseNameNotFound;
    end
    else if wc = #0 then
    begin
      if InCDATA then
        FatalError('Unterminated CDATA section', -1);
      if FNesting > FSource.FStartNesting then
        FatalError('End-tag is missing for ''%s''', [FNodeStack[FNesting].FQName^.Key]);

      if Assigned(FSource.FParent) then
      begin
        if FExpandEntities and ContextPop then
          Continue
        else
          tok := xtEntityEnd;
      end
      else
        tok := xtEOF;
    end
    else if wc = '>' then
    begin
      BufAppend(FValue, wc);
      FSource.NextChar;

      if (FValue.Length <= 2) or (FValue.Buffer[FValue.Length-2] <> ']') or
        (FValue.Buffer[FValue.Length-3] <> ']') then Continue;

      if InCData then   // got a ']]>' separator
      begin
        Dec(FValue.Length, 3);
        InCDATA := False;
        if FCDSectionsAsText then
          Continue;
        SetNodeInfoWithValue(ntCDATA);
        FNext := xtText;
        Result := True;
        Exit;
      end
      else
        FatalError('Literal '']]>'' is not allowed in text', 3);
    end
    else if wc = '&' then
    begin
      if FState <> rsRoot then
        FatalError('Illegal at document level');

      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('References are illegal in EMPTY elements', []);

      if ParseRef(FValue) or ResolvePredefined then
      begin
        nonWs := True; // CharRef to whitespace is not considered whitespace
        Continue;
      end
      else
      begin
        FCurrEntity := EntityCheck;
        if Assigned(FCurrEntity) and FExpandEntities then
        begin
          ContextPush(FCurrEntity);
          Continue;
        end;
        tok := xtEntity;
      end;
    end;
    if FValue.Length <> 0 then
    begin
      if FState <> rsRoot then
        if nonWs then
          FatalError('Illegal at document level', -1)
        else
          Break;

      SetNodeInfoWithValue(textNodeTypes[nonWs]);
      FNext := tok;
      Result := True;
      Exit;
    end;
    Break;
  until False
  else   // not (FNext in [xtText, xtCDSect])
    tok := FNext;

  FNext := xtText;

  case tok of
    xtEntity:     HandleEntityStart;
    xtEntityEnd:  HandleEntityEnd;
    xtElement:    ParseStartTag;
    xtEndElement: ParseEndTag;
    xtPI:         ParsePI;
    xtDoctype:    ParseDoctypeDecl;
    xtComment:    ParseComment(False);
  end;
  Result := tok <> xtEOF;
=======
var
  cursor: TDOMNode_WithChildren;
  element: TDOMElement;
begin
=======
var
  cursor: TDOMNode_WithChildren;
  element: TDOMElement;
begin
>>>>>>> origin/cpstrnew
  cursor := parent;
  while Read do
  begin
    if FValidate then
      ValidateCurrentNode;
<<<<<<< HEAD

    case FCurrNode^.FNodeType of
      ntText:
        cursor.InternalAppend(doc.CreateTextNodeBuf(FValue.Buffer, FValue.Length, False));

      ntWhitespace, ntSignificantWhitespace:
        if FPreserveWhitespace then
          cursor.InternalAppend(doc.CreateTextNodeBuf(FValue.Buffer, FValue.Length, FCurrNode^.FNodeType = ntWhitespace));

      ntCDATA:
        cursor.InternalAppend(DoCDSect(FValue.Buffer, FValue.Length));

      ntProcessingInstruction:
        cursor.InternalAppend(CreatePINode);

      ntComment:
        if not FIgnoreComments then
          cursor.InternalAppend(doc.CreateCommentBuf(FCurrNode^.FValueStart, FCurrNode^.FValueLength));

      ntElement:
        begin
          element := DoStartElement;
          cursor.InternalAppend(element);
          cursor := element;
        end;

      ntEndElement:
          cursor := TDOMNode_WithChildren(cursor.ParentNode);

      ntDocumentType:
        if not FCanonical then
          cursor.InternalAppend(TDOMDocumentType.Create(doc, FDocType));

      ntEntityReference:
        cursor.InternalAppend(doc.CreateEntityReference(FCurrNode^.FQName^.Key));
    end;
  end;
>>>>>>> graemeg/cpstrnew
end;

function TXMLReader.Read: Boolean;
var
  nonWs: Boolean;
  wc: WideChar;
  InCDATA: Boolean;
  tok: TXMLToken;
begin
  if FNext = xtPopEmptyElement then
  begin
    FNext := xtPopElement;
    FCurrNode^.FNodeType := ntEndElement;
    if FAttrCleanupFlag then
      CleanupAttributes;
    FAttrCount := 0;
    Result := True;
    Exit;
  end;
  if FNext = xtPushElement then
  begin
    if FAttrCleanupFlag then
      CleanupAttributes;
    FAttrCount := 0;
    FNext := xtText;
  end
  else if FNext = xtPopElement then
  begin
    if FNamespaces then
      FNSHelper.EndElement;
    PopVC;
  end
  else if FNext = xtPushEntity then
    DoStartEntity;

  InCDATA := (FNext = xtCDSect);
  StoreLocation(FTokenStart);
  nonWs := False;
  FValue.Length := 0;

  if FNext in [xtCDSect, xtText] then
  repeat
    wc := FSource.SkipUntil(FValue, TextDelims[InCDATA], @nonWs);
    if wc = '<' then
    begin
      Inc(FSource.FBuf);
      if FSource.FBufEnd < FSource.FBuf + 2 then
        FSource.Reload;
      if FSource.FBuf^ = '/' then
        tok := xtEndElement
      else if CheckName([cnOptional]) then
        tok := xtElement
      else if FSource.FBuf^ = '!' then
      begin
        Inc(FSource.FBuf);
        if FSource.FBuf^ = '[' then
        begin
          ExpectString('[CDATA[');
          if FState <> rsRoot then
            FatalError('Illegal at document level');
          StoreLocation(FTokenStart);
          InCDATA := True;
          if FCDSectionsAsText or (FValue.Length = 0) then
            Continue;
          tok := xtCDSect;
        end
        else if FSource.FBuf^ = '-' then
        begin
        { Ignoring comments is tricky in validating mode; discarding a comment which
          is the only child of an EMPTY element will make that element erroneously appear
          as valid. Therefore, at this point we discard only comments which are preceded
          by some text (since presence of text already renders an EMPTY element invalid).
          Other comments should be reported to validation part and discarded there. }
          if FIgnoreComments and (FValue.Length > 0) then
          begin
            ParseComment(True);
            Continue;
          end;
          tok := xtComment;
        end
        else
          tok := xtDoctype;
      end
      else if FSource.FBuf^ = '?' then
        tok := xtPI
      else
        RaiseNameNotFound;
    end
    else if wc = #0 then
    begin
      if InCDATA then
        FatalError('Unterminated CDATA section', -1);
      if FNesting > FSource.FStartNesting then
        FatalError('End-tag is missing for ''%s''', [FNodeStack[FNesting].FQName^.Key]);

      if Assigned(FSource.FParent) then
      begin
        if FExpandEntities and ContextPop then
          Continue
        else
          tok := xtEntityEnd;
      end
      else
        tok := xtEOF;
    end
    else if wc = '>' then
    begin
      BufAppend(FValue, wc);
      FSource.NextChar;

      if (FValue.Length <= 2) or (FValue.Buffer[FValue.Length-2] <> ']') or
        (FValue.Buffer[FValue.Length-3] <> ']') then Continue;

      if InCData then   // got a ']]>' separator
      begin
        Dec(FValue.Length, 3);
        InCDATA := False;
        if FCDSectionsAsText then
          Continue;
        SetNodeInfoWithValue(ntCDATA);
        FNext := xtText;
        Result := True;
        Exit;
      end
      else
        FatalError('Literal '']]>'' is not allowed in text', 3);
    end
    else if wc = '&' then
    begin
      if FState <> rsRoot then
        FatalError('Illegal at document level');

      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('References are illegal in EMPTY elements', []);

      if ParseRef(FValue) or ResolvePredefined then
      begin
        nonWs := True; // CharRef to whitespace is not considered whitespace
        Continue;
      end
      else
      begin
        FCurrEntity := EntityCheck;
        if Assigned(FCurrEntity) and FExpandEntities then
        begin
          ContextPush(FCurrEntity);
          Continue;
        end;
        tok := xtEntity;
      end;
    end;
    if FValue.Length <> 0 then
    begin
      if FState <> rsRoot then
        if nonWs then
          FatalError('Illegal at document level', -1)
        else
          Break;

      SetNodeInfoWithValue(textNodeTypes[nonWs]);
      FNext := tok;
      Result := True;
      Exit;
    end;
    Break;
  until False
  else   // not (FNext in [xtText, xtCDSect])
    tok := FNext;

  FNext := xtText;

  case tok of
    xtEntity:     HandleEntityStart;
    xtEntityEnd:  HandleEntityEnd;
    xtElement:    ParseStartTag;
    xtEndElement: ParseEndTag;
    xtPI:         ParsePI;
    xtDoctype:    ParseDoctypeDecl;
    xtComment:    ParseComment(False);
  end;
  Result := tok <> xtEOF;
=======
    end;
  end;
>>>>>>> graemeg/cpstrnew
end;

function TXMLReader.Read: Boolean;
var
  nonWs: Boolean;
  wc: WideChar;
  InCDATA: Boolean;
  tok: TXMLToken;
begin
  if FNext = xtPopEmptyElement then
  begin
    FNext := xtPopElement;
    FCurrNode^.FNodeType := ntEndElement;
    if FAttrCleanupFlag then
      CleanupAttributes;
    FAttrCount := 0;
    Result := True;
    Exit;
  end;
  if FNext = xtPushElement then
  begin
    if FAttrCleanupFlag then
      CleanupAttributes;
    FAttrCount := 0;
    FNext := xtText;
  end
  else if FNext = xtPopElement then
  begin
    if FNamespaces then
      FNSHelper.EndElement;
    PopVC;
  end
  else if FNext = xtPushEntity then
    DoStartEntity;

  InCDATA := (FNext = xtCDSect);
  StoreLocation(FTokenStart);
  nonWs := False;
  FValue.Length := 0;

  if FNext in [xtCDSect, xtText] then
  repeat
    wc := FSource.SkipUntil(FValue, TextDelims[InCDATA], @nonWs);
    if wc = '<' then
    begin
      Inc(FSource.FBuf);
      if FSource.FBufEnd < FSource.FBuf + 2 then
        FSource.Reload;
      if FSource.FBuf^ = '/' then
        tok := xtEndElement
      else if CheckName([cnOptional]) then
        tok := xtElement
      else if FSource.FBuf^ = '!' then
      begin
        Inc(FSource.FBuf);
        if FSource.FBuf^ = '[' then
        begin
          ExpectString('[CDATA[');
          if FState <> rsRoot then
            FatalError('Illegal at document level');
          StoreLocation(FTokenStart);
          InCDATA := True;
          if FCDSectionsAsText or (FValue.Length = 0) then
            Continue;
          tok := xtCDSect;
        end
        else if FSource.FBuf^ = '-' then
        begin
        { Ignoring comments is tricky in validating mode; discarding a comment which
          is the only child of an EMPTY element will make that element erroneously appear
          as valid. Therefore, at this point we discard only comments which are preceded
          by some text (since presence of text already renders an EMPTY element invalid).
          Other comments should be reported to validation part and discarded there. }
          if FIgnoreComments and (FValue.Length > 0) then
          begin
            ParseComment(True);
            Continue;
          end;
          tok := xtComment;
        end
        else
          tok := xtDoctype;
      end
      else if FSource.FBuf^ = '?' then
        tok := xtPI
      else
        RaiseNameNotFound;
    end
    else if wc = #0 then
    begin
      if InCDATA then
        FatalError('Unterminated CDATA section', -1);
      if FNesting > FSource.FStartNesting then
        FatalError('End-tag is missing for ''%s''', [FNodeStack[FNesting].FQName^.Key]);

      if Assigned(FSource.FParent) then
      begin
        if FExpandEntities and ContextPop then
          Continue
        else
          tok := xtEntityEnd;
      end
      else
        tok := xtEOF;
    end
    else if wc = '>' then
    begin
      BufAppend(FValue, wc);
      FSource.NextChar;

      if (FValue.Length <= 2) or (FValue.Buffer[FValue.Length-2] <> ']') or
        (FValue.Buffer[FValue.Length-3] <> ']') then Continue;

      if InCData then   // got a ']]>' separator
      begin
        Dec(FValue.Length, 3);
        InCDATA := False;
        if FCDSectionsAsText then
          Continue;
        SetNodeInfoWithValue(ntCDATA);
        FNext := xtText;
        Result := True;
        Exit;
      end
      else
        FatalError('Literal '']]>'' is not allowed in text', 3);
    end
    else if wc = '&' then
    begin
      if FState <> rsRoot then
        FatalError('Illegal at document level');

      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('References are illegal in EMPTY elements', []);

      if ParseRef(FValue) or ResolvePredefined then
      begin
        nonWs := True; // CharRef to whitespace is not considered whitespace
        Continue;
      end
      else
      begin
        FCurrEntity := EntityCheck;
        if Assigned(FCurrEntity) and FExpandEntities then
        begin
          ContextPush(FCurrEntity);
          Continue;
        end;
        tok := xtEntity;
      end;
    end;
    if FValue.Length <> 0 then
    begin
      if FState <> rsRoot then
        if nonWs then
          FatalError('Illegal at document level', -1)
        else
          Break;

      SetNodeInfoWithValue(textNodeTypes[nonWs]);
      FNext := tok;
      Result := True;
      Exit;
    end;
    Break;
  until False
  else   // not (FNext in [xtText, xtCDSect])
    tok := FNext;

  FNext := xtText;

  case tok of
    xtEntity:     HandleEntityStart;
    xtEntityEnd:  HandleEntityEnd;
    xtElement:    ParseStartTag;
    xtEndElement: ParseEndTag;
    xtPI:         ParsePI;
    xtDoctype:    ParseDoctypeDecl;
    xtComment:    ParseComment(False);
  end;
  Result := tok <> xtEOF;
=======

    case FCurrNode^.FNodeType of
      ntText:
        cursor.InternalAppend(doc.CreateTextNodeBuf(FValue.Buffer, FValue.Length, False));

      ntWhitespace, ntSignificantWhitespace:
        if FPreserveWhitespace then
          cursor.InternalAppend(doc.CreateTextNodeBuf(FValue.Buffer, FValue.Length, FCurrNode^.FNodeType = ntWhitespace));

      ntCDATA:
        cursor.InternalAppend(DoCDSect(FValue.Buffer, FValue.Length));

      ntProcessingInstruction:
        cursor.InternalAppend(CreatePINode);

      ntComment:
        if not FIgnoreComments then
          cursor.InternalAppend(doc.CreateCommentBuf(FCurrNode^.FValueStart, FCurrNode^.FValueLength));

      ntElement:
        begin
          element := DoStartElement;
          cursor.InternalAppend(element);
          cursor := element;
        end;

      ntEndElement:
          cursor := TDOMNode_WithChildren(cursor.ParentNode);

      ntDocumentType:
        if not FCanonical then
          cursor.InternalAppend(TDOMDocumentType.Create(doc, FDocType));

      ntEntityReference:
        cursor.InternalAppend(doc.CreateEntityReference(FCurrNode^.FQName^.Key));
    end;
  end;
>>>>>>> origin/cpstrnew
end;

function TXMLReader.Read: Boolean;
var
  nonWs: Boolean;
  wc: WideChar;
  InCDATA: Boolean;
  tok: TXMLToken;
begin
  if FNext = xtPopEmptyElement then
  begin
    FNext := xtPopElement;
    FCurrNode^.FNodeType := ntEndElement;
    if FAttrCleanupFlag then
      CleanupAttributes;
    FAttrCount := 0;
    Result := True;
    Exit;
  end;
  if FNext = xtPushElement then
  begin
    if FAttrCleanupFlag then
      CleanupAttributes;
    FAttrCount := 0;
    FNext := xtText;
  end
  else if FNext = xtPopElement then
  begin
    if FNamespaces then
      FNSHelper.EndElement;
    PopVC;
  end
  else if FNext = xtPushEntity then
    DoStartEntity;

  InCDATA := (FNext = xtCDSect);
  StoreLocation(FTokenStart);
  nonWs := False;
  FValue.Length := 0;

  if FNext in [xtCDSect, xtText] then
  repeat
    wc := FSource.SkipUntil(FValue, TextDelims[InCDATA], @nonWs);
    if wc = '<' then
    begin
      Inc(FSource.FBuf);
      if FSource.FBufEnd < FSource.FBuf + 2 then
        FSource.Reload;
      if FSource.FBuf^ = '/' then
        tok := xtEndElement
      else if CheckName([cnOptional]) then
        tok := xtElement
      else if FSource.FBuf^ = '!' then
      begin
        Inc(FSource.FBuf);
        if FSource.FBuf^ = '[' then
        begin
          ExpectString('[CDATA[');
          if FState <> rsRoot then
            FatalError('Illegal at document level');
          StoreLocation(FTokenStart);
          InCDATA := True;
          if FCDSectionsAsText or (FValue.Length = 0) then
            Continue;
          tok := xtCDSect;
        end
        else if FSource.FBuf^ = '-' then
        begin
        { Ignoring comments is tricky in validating mode; discarding a comment which
          is the only child of an EMPTY element will make that element erroneously appear
          as valid. Therefore, at this point we discard only comments which are preceded
          by some text (since presence of text already renders an EMPTY element invalid).
          Other comments should be reported to validation part and discarded there. }
          if FIgnoreComments and (FValue.Length > 0) then
          begin
            ParseComment(True);
            Continue;
          end;
          tok := xtComment;
        end
        else
          tok := xtDoctype;
      end
      else if FSource.FBuf^ = '?' then
        tok := xtPI
      else
        RaiseNameNotFound;
    end
    else if wc = #0 then
    begin
      if InCDATA then
        FatalError('Unterminated CDATA section', -1);
      if FNesting > FSource.FStartNesting then
        FatalError('End-tag is missing for ''%s''', [FNodeStack[FNesting].FQName^.Key]);

      if Assigned(FSource.FParent) then
      begin
        if FExpandEntities and ContextPop then
          Continue
        else
          tok := xtEntityEnd;
      end
      else
        tok := xtEOF;
    end
    else if wc = '>' then
    begin
      BufAppend(FValue, wc);
      FSource.NextChar;

      if (FValue.Length <= 2) or (FValue.Buffer[FValue.Length-2] <> ']') or
        (FValue.Buffer[FValue.Length-3] <> ']') then Continue;

      if InCData then   // got a ']]>' separator
      begin
        Dec(FValue.Length, 3);
        InCDATA := False;
        if FCDSectionsAsText then
          Continue;
        SetNodeInfoWithValue(ntCDATA);
        FNext := xtText;
        Result := True;
        Exit;
      end
      else
        FatalError('Literal '']]>'' is not allowed in text', 3);
    end
    else if wc = '&' then
    begin
      if FState <> rsRoot then
        FatalError('Illegal at document level');

      if FValidators[FValidatorNesting].FContentType = ctEmpty then
        ValidationError('References are illegal in EMPTY elements', []);

      if ParseRef(FValue) or ResolvePredefined then
      begin
        nonWs := True; // CharRef to whitespace is not considered whitespace
        Continue;
      end
      else
      begin
        FCurrEntity := EntityCheck;
        if Assigned(FCurrEntity) and FExpandEntities then
        begin
          ContextPush(FCurrEntity);
          Continue;
        end;
        tok := xtEntity;
      end;
    end;
    if FValue.Length <> 0 then
    begin
      if FState <> rsRoot then
        if nonWs then
          FatalError('Illegal at document level', -1)
        else
          Break;

      SetNodeInfoWithValue(textNodeTypes[nonWs]);
      FNext := tok;
      Result := True;
      Exit;
    end;
    Break;
  until False
  else   // not (FNext in [xtText, xtCDSect])
    tok := FNext;

  FNext := xtText;

  case tok of
    xtEntity:     HandleEntityStart;
    xtEntityEnd:  HandleEntityEnd;
    xtElement:    ParseStartTag;
    xtEndElement: ParseEndTag;
    xtPI:         ParsePI;
    xtDoctype:    ParseDoctypeDecl;
    xtComment:    ParseComment(False);
  end;
  Result := tok <> xtEOF;
end;

procedure TXMLCharSource.NextChar;
begin
  Inc(FBuf);
  if FBuf >= FBufEnd then
    Reload;
end;

procedure TXMLReader.ExpectChar(wc: WideChar);
begin
  if FSource.FBuf^ = wc then
    FSource.NextChar
  else
    FatalError(wc);
end;

// Element name already in FNameBuffer
procedure TXMLReader.ParseStartTag;    // [39] [40] [44]
var
  ElDef: TElementDecl;
  IsEmpty: Boolean;
  ElName: PHashItem;
  b: TBinding;
begin
  if FState > rsRoot then
    FatalError('Only one top-level element allowed', FName.Length)
  else if FState < rsRoot then
  begin
    // dispose notation refs from DTD, if any
    ClearForwardRefs;
    FState := rsRoot;
  end;

  // we're about to process a new set of attributes
  Inc(FAttrTag);

  // Get hash entry for element name
  ElName := FNameTable.FindOrAdd(FName.Buffer, FName.Length);
  // Find declaration for this element
  ElDef := TElementDecl(ElName^.Data);

  IsEmpty := False;
  FAttrCount := 0;
  FPrefixedAttrs := 0;
  FSpecifiedAttrs := 0;

  Inc(FNesting);
  FCurrNode := AllocNodeData(FNesting);
  FCurrNode^.FQName := ElName;
  FCurrNode^.FNodeType := ntElement;
  FCurrNode^.FColonPos := FColonPos;
  StoreLocation(FCurrNode^.FLoc);
  Dec(FCurrNode^.FLoc.LinePos, FName.Length);

  if FNamespaces then
  begin
    FNSHelper.StartElement;
    if FColonPos > 0 then
      FCurrNode^.FPrefix := FNSHelper.GetPrefix(FName.Buffer, FColonPos);
  end;

  while (FSource.FBuf^ <> '>') and (FSource.FBuf^ <> '/') do
  begin
    SkipS(True);
    if (FSource.FBuf^ = '>') or (FSource.FBuf^ = '/') then
      Break;
    ParseAttribute(ElDef);
  end;
>>>>>>> graemeg/cpstrnew

  TDOMParser = class(TObject)
  private
    FOptions: TDOMParseOptions;
    function GetOnError: TXMLErrorEvent;
    procedure SetOnError(value: TXMLErrorEvent);
  public
    constructor Create;
    destructor Destroy; override;
    procedure Parse(Src: TXMLInputSource; out ADoc: TXMLDocument);
    procedure ParseUri(const URI: XMLString; out ADoc: TXMLDocument);
    function ParseWithContext(Src: TXMLInputSource; Context: TDOMNode;
      Action: TXMLContextAction): TDOMNode;
    property Options: TDOMParseOptions read FOptions;
    property OnError: TXMLErrorEvent read GetOnError write SetOnError;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TDecoder = xmltextreader.TDecoder;
  TGetDecoderProc = xmltextreader.TGetDecoderProc;

procedure RegisterDecoder(Proc: TGetDecoderProc);

// =======================================================

implementation

uses
  UriParser, dtdmodel;

type
  TLoader = object
    doc: TDOMDocument;
    reader: TXMLTextReader;
    function CreateCDATANode(currnode: PNodeData): TDOMNode;
    function CreatePINode(currnode: PNodeData): TDOMNode;
    procedure ParseContent(cursor: TDOMNode_WithChildren);

    procedure ProcessXML(ADoc: TDOMDocument; AReader: TXMLTextReader);
    procedure ProcessFragment(AOwner: TDOMNode; AReader: TXMLTextReader);
    procedure ProcessDTD(ADoc: TDOMDocument; AReader: TXMLTextReader);
    procedure ProcessEntity(Sender: TXMLTextReader; AEntity: TEntityDecl);
  end;

<<<<<<< HEAD
procedure RegisterDecoder(Proc: TGetDecoderProc);
begin
  xmltextreader.RegisterDecoder(Proc);
end;

{ TDOMParser }

constructor TDOMParser.Create;
begin
  FOptions := TDOMParseOptions.Create;
end;

destructor TDOMParser.Destroy;
begin
  FOptions.Free;
  inherited Destroy;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if Assigned(ElDef) and ElDef.NeedsDefaultPass then
    ProcessDefaultAttributes(ElDef);

  // Adding attributes might have reallocated FNodeStack, so restore FCurrNode once again
  FCurrNode := @FNodeStack[FNesting];

  if FNamespaces then
  begin
    { Assign namespace URIs to prefixed attrs }
    if FPrefixedAttrs <> 0 then
      ProcessNamespaceAtts;
    { Expand the element name }
    if Assigned(FCurrNode^.FPrefix) then
    begin
      b := TBinding(FCurrNode^.FPrefix^.Data);
      if not (Assigned(b) and (b.uri <> '')) then
        DoErrorPos(esFatal, 'Unbound element name prefix "%s"', [FCurrNode^.FPrefix^.Key],FCurrNode^.FLoc);
      FCurrNode^.FNsUri := FNameTable.FindOrAdd(PWideChar(b.uri), Length(b.uri));
    end
    else
    begin
      b := FNSHelper.DefaultNSBinding;
      if Assigned(b) then
        FCurrNode^.FNsUri := FNameTable.FindOrAdd(PWideChar(b.uri), Length(b.uri));
    end;
  end;

  if not IsEmpty then
  begin
    if not FPreserveWhitespace then   // critical for testsuite compliance
      SkipS;
    FNext := xtPushElement;
  end
  else
    FNext := xtPopEmptyElement;
end;

procedure TXMLReader.ParseEndTag;     // [42]
var
  ElName: PHashItem;
begin
  if FNesting <= FSource.FStartNesting then
    FatalError('End-tag is not allowed here');
  Inc(FSource.FBuf);

  FCurrNode := @FNodeStack[FNesting];  // move off the possible child
  FCurrNode^.FNodeType := ntEndElement;
  ElName := FCurrNode^.FQName;

  CheckName;
  if not BufEquals(FName, ElName^.Key) then
    FatalError('Unmatching element end tag (expected "</%s>")', [ElName^.Key], FName.Length);
  if FSource.FBuf^ = '>' then    // this handles majority of cases
    FSource.NextChar
  else
  begin
    SkipS;
    ExpectChar('>');
  end;
  Inc(FTokenStart.LinePos, 2);   // move over '</' chars
  FNext := xtPopElement;
end;

procedure TXMLReader.ParseAttribute(ElDef: TElementDecl);
var
  attrName: PHashItem;
  attrData: PNodeData;
  AttDef: TAttributeDef;
  i: Integer;
begin
  CheckName;
  attrName := FNameTable.FindOrAdd(FName.Buffer, FName.Length);
  attrData := AllocAttributeData;
  attrData^.FQName := attrName;
  attrData^.FColonPos := FColonPos;
  StoreLocation(attrData^.FLoc);
  Dec(attrData^.FLoc.LinePos, FName.Length);
  FSpecifiedAttrs := FAttrCount;

  if Assigned(ElDef) then
  begin
    AttDef := ElDef.GetAttrDef(attrName);
    if Assigned(AttDef) then
      AttDef.Tag := FAttrTag;  // indicates that this one is specified
  end
  else
    AttDef := nil;

  attrData^.FTypeInfo := AttDef;
  // check for duplicates
  for i := 1 to FAttrCount-1 do
    if FNodeStack[FNesting+i].FQName = attrName then
      FatalError('Duplicate attribute', FName.Length);

  if FNamespaces then
  begin
    if ((FName.Length = 5) or (FColonPos = 5)) and
      (FName.Buffer[0] = 'x') and (FName.Buffer[1] = 'm') and
      (FName.Buffer[2] = 'l') and (FName.Buffer[3] = 'n') and
      (FName.Buffer[4] = 's') then
    begin
      if FColonPos > 0 then
        attrData^.FPrefix := FStdPrefix_xmlns;
      attrData^.FNsUri := FStdUri_xmlns;
    end
    else if FColonPos > 0 then
    begin
      attrData^.FPrefix := FNSHelper.GetPrefix(FName.Buffer, FColonPos);
      Inc(FPrefixedAttrs);
    end;
  end;

  ExpectEq;
  ExpectAttValue(attrData, Assigned(AttDef) and (AttDef.DataType <> dtCDATA));
  attrData^.FLoc2 := FTokenStart;

  if Assigned(attrData^.FNsUri) then
  begin
    if (not AddBinding(attrData)) and FCanonical then
    begin
      CleanupAttribute(attrData);
      Dec(FAttrCount);
      Dec(FSpecifiedAttrs);
    end;
  end;
end;

procedure TXMLReader.AddForwardRef(Buf: PWideChar; Length: Integer);
var
  w: PForwardRef;
begin
  New(w);
  SetString(w^.Value, Buf, Length);
  w^.Loc := FTokenStart;
  FForwardRefs.Add(w);
end;

procedure TXMLReader.ClearForwardRefs;
var
  I: Integer;
begin
  for I := 0 to FForwardRefs.Count-1 do
    Dispose(PForwardRef(FForwardRefs.List^[I]));
  FForwardRefs.Clear;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
end;

function TDOMParser.GetOnError: TXMLErrorEvent;
begin
<<<<<<< HEAD
  result := Options.OnError;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;

procedure TDOMParser.SetOnError(value: TXMLErrorEvent);
begin
  Options.OnError := value;
=======
>>>>>>> origin/cpstrnew
end;

procedure TDOMParser.Parse(Src: TXMLInputSource; out ADoc: TXMLDocument);
var
  Reader: TXMLTextReader;
  ldr: TLoader;
begin
<<<<<<< HEAD
  ADoc := TXMLDocument.Create;
  Options.NameTable := ADoc.Names;
  Reader := TXMLTextReader.Create(Src, Options);
  try
    ldr.ProcessXML(ADoc, Reader);
  finally
    Reader.Free;
  end;
end;

procedure TDOMParser.ParseUri(const URI: XMLString; out ADoc: TXMLDocument);
var
  Reader: TXMLTextReader;
  ldr: TLoader;
begin
<<<<<<< HEAD
  ADoc := TXMLDocument.Create;
  Options.NameTable := ADoc.Names;
  Reader := TXMLTextReader.Create(URI, Options);
  try
    ldr.ProcessXML(ADoc, Reader)
  finally
    Reader.Free;
  end;
end;

function TDOMParser.ParseWithContext(Src: TXMLInputSource;
  Context: TDOMNode; Action: TXMLContextAction): TDOMNode;
var
  Frag: TDOMDocumentFragment;
  node: TDOMNode;
  reader: TXMLTextReader;
  ldr: TLoader;
  doc: TDOMDocument;
begin
<<<<<<< HEAD
  if Action in [xaInsertBefore, xaInsertAfter, xaReplace] then
    node := Context.ParentNode
  else
    node := Context;
  // TODO: replacing document isn't yet supported
  if (Action = xaReplaceChildren) and (node.NodeType = DOCUMENT_NODE) then
    raise EDOMNotSupported.Create('DOMParser.ParseWithContext');

  if not (node.NodeType in [ELEMENT_NODE, DOCUMENT_FRAGMENT_NODE]) then
    raise EDOMHierarchyRequest.Create('DOMParser.ParseWithContext');

  if Context.NodeType = DOCUMENT_NODE then
    doc := TDOMDocument(Context)
=======
=======
>>>>>>> graemeg/cpstrnew
  for I := 0 to FForwardRefs.Count-1 do
    with PForwardRef(FForwardRefs.List^[I])^ do
      if (FIDMap = nil) or (FIDMap.Find(PWideChar(Value), Length(Value)) = nil) then
        DoErrorPos(esError, 'The ID ''%s'' does not match any element', [Value], Loc);
  ClearForwardRefs;
end;

<<<<<<< HEAD
=======
  for I := 0 to FForwardRefs.Count-1 do
    with PForwardRef(FForwardRefs.List^[I])^ do
      if (FIDMap = nil) or (FIDMap.Find(PWideChar(Value), Length(Value)) = nil) then
        DoErrorPos(esError, 'The ID ''%s'' does not match any element', [Value], Loc);
  ClearForwardRefs;
end;

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  for I := 0 to FForwardRefs.Count-1 do
    with PForwardRef(FForwardRefs.List^[I])^ do
      if (FIDMap = nil) or (FIDMap.Find(PWideChar(Value), Length(Value)) = nil) then
        DoErrorPos(esError, 'The ID ''%s'' does not match any element', [Value], Loc);
  ClearForwardRefs;
end;

>>>>>>> origin/cpstrnew
procedure TXMLReader.ProcessDefaultAttributes(ElDef: TElementDecl);
var
  I: Integer;
  AttDef: TAttributeDef;
  attrData: PNodeData;
begin
  for I := 0 to ElDef.AttrDefCount-1 do
  begin
    AttDef := ElDef.AttrDefs[I];

    if AttDef.Tag <> FAttrTag then  // this one wasn't specified
    begin
      case AttDef.Default of
        adDefault, adFixed: begin
          attrData := AllocAttributeData;
          attrData^ := AttDef.Data^;
          if FCanonical then
            attrData^.FIsDefault := False;

          if FNamespaces then
          begin
            if AttDef.IsNamespaceDecl then
            begin
              if attrData^.FColonPos > 0 then
                attrData^.FPrefix := FStdPrefix_xmlns;
              attrData^.FNsUri := FStdUri_xmlns;
              if (not AddBinding(attrData)) and FCanonical then
                Dec(FAttrCount);
            end
            else if attrData^.FColonPos > 0 then
            begin
              attrData^.FPrefix := FNSHelper.GetPrefix(PWideChar(attrData^.FQName^.Key), attrData^.FColonPos);
              Inc(FPrefixedAttrs);
            end;
          end;
        end;
        adRequired:
          ValidationError('Required attribute ''%s'' of element ''%s'' is missing',
            [AttDef.Data^.FQName^.Key, FNodeStack[FNesting].FQName^.Key], 0)
      end;
    end;
  end;
end;


function TXMLReader.AddBinding(attrData: PNodeData): Boolean;
var
  nsUri, Pfx: PHashItem;
begin
  nsUri := FNameTable.FindOrAdd(PWideChar(attrData^.FValueStr), Length(attrData^.FValueStr));
  if attrData^.FColonPos > 0 then
    Pfx := FNSHelper.GetPrefix(@attrData^.FQName^.key[7], Length(attrData^.FQName^.key)-6)
  else
    Pfx := FNSHelper.GetPrefix(nil, 0);  { will return the default prefix }
  { 'xml' is allowed to be bound to the correct namespace }
  if ((nsUri = FStduri_xml) <> (Pfx = FStdPrefix_xml)) or
   (Pfx = FStdPrefix_xmlns) or
   (nsUri = FStduri_xmlns) then
  begin
    if (Pfx = FStdPrefix_xml) or (Pfx = FStdPrefix_xmlns) then
      DoErrorPos(esFatal, 'Illegal usage of reserved prefix ''%s''', [Pfx^.Key], attrData^.FLoc)
    else
      DoErrorPos(esFatal, 'Illegal usage of reserved namespace URI ''%s''', [attrData^.FValueStr], attrData^.FLoc2);
  end;
<<<<<<< HEAD
<<<<<<< HEAD

  if (attrData^.FValueStr = '') and not (FXML11 or (Pfx^.Key = '')) then
    DoErrorPos(esFatal, 'Illegal undefining of namespace', attrData^.FLoc2);

=======

  if (attrData^.FValueStr = '') and not (FXML11 or (Pfx^.Key = '')) then
    DoErrorPos(esFatal, 'Illegal undefining of namespace', attrData^.FLoc2);

>>>>>>> graemeg/cpstrnew
=======

  if (attrData^.FValueStr = '') and not (FXML11 or (Pfx^.Key = '')) then
    DoErrorPos(esFatal, 'Illegal undefining of namespace', attrData^.FLoc2);

>>>>>>> origin/cpstrnew
  Result := (Pfx^.Data = nil) or (TBinding(Pfx^.Data).uri <> attrData^.FValueStr);
  if Result then
    FNSHelper.BindPrefix(attrData^.FValueStr, Pfx);
end;

procedure TXMLReader.ProcessNamespaceAtts;
var
  I, J: Integer;
  Pfx, AttrName: PHashItem;
  attrData: PNodeData;
  b: TBinding;
begin
  FNsAttHash.Init(FPrefixedAttrs);
  for I := 1 to FAttrCount do
  begin
    attrData := @FNodeStack[FNesting+i];
    if (attrData^.FColonPos < 1) or Assigned(attrData^.FNsUri) then
      Continue;

    Pfx := attrData^.FPrefix;
    b := TBinding(Pfx^.Data);
    if not (Assigned(b) and (b.uri <> '')) then
      DoErrorPos(esFatal, 'Unbound attribute name prefix "%s"', [Pfx^.Key], attrData^.FLoc);

    { detect duplicates }
    J := attrData^.FColonPos+1;
    AttrName := attrData^.FQName;

    if FNsAttHash.Locate(@b.uri, @AttrName^.Key[J], Length(AttrName^.Key) - J+1) then
      DoErrorPos(esFatal, 'Duplicate prefixed attribute', attrData^.FLoc);

    attrData^.FNsUri := FNameTable.FindOrAdd(PWideChar(b.uri), Length(b.uri));
  end;
end;

function TXMLReader.ParseExternalID(out SysID, PubID: WideString;     // [75]
  SysIdOptional: Boolean): Boolean;
var
  I: Integer;
  wc: WideChar;
begin
  Result := False;
  if FSource.Matches('SYSTEM') then
    SysIdOptional := False
  else if FSource.Matches('PUBLIC') then
  begin
    ExpectWhitespace;
    ParseLiteral(FValue, ltPubid, True);
    SetString(PubID, FValue.Buffer, FValue.Length);
    for I := 1 to Length(PubID) do
    begin
      wc := PubID[I];
      if (wc > #255) or not (Char(ord(wc)) in PubidChars) then
        FatalError('Illegal Public ID literal', -1);
    end;
  end
  else
    Exit;

  if SysIdOptional then
    SkipWhitespace
  else
    ExpectWhitespace;
  if ParseLiteral(FValue, ltPlain, not SysIdOptional) then
    SetString(SysID, FValue.Buffer, FValue.Length);
  Result := True;
end;

function TXMLReader.ValidateAttrSyntax(AttrDef: TAttributeDef; const aValue: WideString): Boolean;
begin
  case AttrDef.DataType of
    dtId, dtIdRef, dtEntity: Result := IsXmlName(aValue, FXML11) and
      ((not FNamespaces) or (Pos(WideChar(':'), aValue) = 0));
    dtIdRefs, dtEntities: Result := IsXmlNames(aValue, FXML11) and
      ((not FNamespaces) or (Pos(WideChar(':'), aValue) = 0));
    dtNmToken: Result := IsXmlNmToken(aValue, FXML11) and AttrDef.HasEnumToken(aValue);
    dtNmTokens: Result := IsXmlNmTokens(aValue, FXML11);
    // IsXmlName() not necessary - enum is never empty and contains valid names
    dtNotation: Result := AttrDef.HasEnumToken(aValue);
>>>>>>> graemeg/cpstrnew
  else
    doc := Context.OwnerDocument;

<<<<<<< HEAD
<<<<<<< HEAD
  Options.NameTable := doc.Names;
  reader := TXMLTextReader.Create(Src, Options);
  try
    Frag := doc.CreateDocumentFragment;
    try
      ldr.ProcessFragment(Frag, reader);
      Result := Frag.FirstChild;
      case Action of
        xaAppendAsChildren: Context.AppendChild(Frag);

<<<<<<< HEAD
        xaReplaceChildren: begin
          Context.TextContent := '';     // removes children
          Context.ReplaceChild(Frag, Context.FirstChild);
        end;
        xaInsertBefore: node.InsertBefore(Frag, Context);
        xaInsertAfter:  node.InsertBefore(Frag, Context.NextSibling);
        xaReplace:      node.ReplaceChild(Frag, Context);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
procedure TXMLReader.ValidateAttrValue(AttrDef: TAttributeDef; attrData: PNodeData);
var
  L, StartPos, EndPos: Integer;
  Entity: TEntityDecl;
begin
  L := Length(attrData^.FValueStr);
  case AttrDef.DataType of
    dtId: begin
      if not AddID(attrData) then
        DoErrorPos(esError, 'The ID ''%s'' is not unique', [attrData^.FValueStr], attrData^.FLoc2);
    end;

    dtIdRef, dtIdRefs: begin
      StartPos := 1;
      while StartPos <= L do
      begin
        EndPos := StartPos;
        while (EndPos <= L) and (attrData^.FValueStr[EndPos] <> #32) do
          Inc(EndPos);
        if (FIDMap = nil) or (FIDMap.Find(@attrData^.FValueStr[StartPos], EndPos-StartPos) = nil) then
          AddForwardRef(@attrData^.FValueStr[StartPos], EndPos-StartPos);
        StartPos := EndPos + 1;
      end;
    end;

    dtEntity, dtEntities: begin
      StartPos := 1;
      while StartPos <= L do
      begin
        EndPos := StartPos;
        while (EndPos <= L) and (attrData^.FValueStr[EndPos] <> #32) do
          Inc(EndPos);
        Entity := TEntityDecl(FDocType.Entities.Get(@attrData^.FValueStr[StartPos], EndPos-StartPos));
        if (Entity = nil) or (Entity.FNotationName = '') then
          ValidationError('Attribute ''%s'' type mismatch', [attrData^.FQName^.Key], -1);
        StartPos := EndPos + 1;
>>>>>>> graemeg/cpstrnew
      end;
    finally
      Frag.Free;
    end;
  finally
    reader.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TLoader.ProcessXML(ADoc: TDOMDocument; AReader: TXMLTextReader);
begin
  doc := ADoc;
  reader := AReader;
  reader.OnEntity := @ProcessEntity;
  doc.documentURI := reader.BaseURI;
  reader.FragmentMode := False;
  ParseContent(doc);
  doc.XMLStandalone := reader.Standalone;

  if reader.Validate then
    reader.ValidateIdRefs;

  doc.IDs := reader.IDMap;
  reader.IDMap := nil;
end;

procedure TLoader.ProcessFragment(AOwner: TDOMNode; AReader: TXMLTextReader);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
procedure TXMLReader.ValidateDTD;
>>>>>>> graemeg/cpstrnew
var
  DoctypeNode: TDOMDocumentType;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  doc := AOwner.OwnerDocument;
  reader := AReader;
  reader.OnEntity := @ProcessEntity;
  reader.FragmentMode := True;
  reader.XML11 := doc.XMLVersion = '1.1';
  DoctypeNode := doc.DocType;
  if Assigned(DoctypeNode) then
    reader.DtdSchemaInfo := DocTypeNode.Model.Reference;
  ParseContent(aOwner as TDOMNode_WithChildren);
end;

procedure TLoader.ProcessEntity(Sender: TXMLTextReader; AEntity: TEntityDecl);
var
  DoctypeNode: TDOMDocumentType;
  Ent: TDOMEntity;
  src: TXMLCharSource;
  InnerReader: TXMLTextReader;
  InnerLoader: TLoader;
begin
  DoctypeNode := TDOMDocument(doc).DocType;
  if DoctypeNode = nil then
    Exit;
  Ent := TDOMEntity(DocTypeNode.Entities.GetNamedItem(AEntity.FName));
  if Ent = nil then
    Exit;
  Sender.EntityToSource(AEntity, Src);
  if Src = nil then
    Exit;
  InnerReader := TXMLTextReader.Create(Src, Sender);
  try
    Ent.SetReadOnly(False);
    InnerLoader.ProcessFragment(Ent, InnerReader);
    AEntity.FResolved := True;
  finally
    InnerReader.Free;
    AEntity.FOnStack := False;
    Ent.SetReadOnly(True);
  end;
end;

<<<<<<< HEAD
procedure TLoader.ParseContent(cursor: TDOMNode_WithChildren);
var
  element: TDOMElement;
  currnodeptr: PPNodeData;
  currnode: PNodeData;
begin
  currnodeptr := (reader as IGetNodeDataPtr).CurrentNodePtr;
  if reader.ReadState = rsInitial then
  begin
    if not reader.Read then
      Exit;
    case cursor.NodeType of
      DOCUMENT_NODE, ENTITY_NODE:
        (cursor as TDOMNode_TopLevel).SetHeaderData(reader.XMLVersion,reader.XMLEncoding);
=======
procedure TXMLReader.ValidateAttrValue(AttrDef: TAttributeDef; attrData: PNodeData);
var
  L, StartPos, EndPos: Integer;
  Entity: TEntityDecl;
begin
  L := Length(attrData^.FValueStr);
  case AttrDef.DataType of
    dtId: begin
      if not AddID(attrData) then
        DoErrorPos(esError, 'The ID ''%s'' is not unique', [attrData^.FValueStr], attrData^.FLoc2);
    end;

    dtIdRef, dtIdRefs: begin
      StartPos := 1;
      while StartPos <= L do
      begin
        EndPos := StartPos;
        while (EndPos <= L) and (attrData^.FValueStr[EndPos] <> #32) do
          Inc(EndPos);
        if (FIDMap = nil) or (FIDMap.Find(@attrData^.FValueStr[StartPos], EndPos-StartPos) = nil) then
          AddForwardRef(@attrData^.FValueStr[StartPos], EndPos-StartPos);
        StartPos := EndPos + 1;
      end;
>>>>>>> origin/cpstrnew
    end;
  end;

<<<<<<< HEAD
  with reader do
  repeat
    if Validate then
      ValidateCurrentNode;

    currnode := currnodeptr^;
    case currnode^.FNodeType of
      ntText:
        cursor.InternalAppend(doc.CreateTextNodeBuf(currnode^.FValueStart, currnode^.FValueLength, False));

      ntWhitespace, ntSignificantWhitespace:
        if PreserveWhitespace then
          cursor.InternalAppend(doc.CreateTextNodeBuf(currnode^.FValueStart, currnode^.FValueLength, currnode^.FNodeType = ntWhitespace));

      ntCDATA:
        cursor.InternalAppend(CreateCDATANode(currnode));

      ntProcessingInstruction:
        cursor.InternalAppend(CreatePINode(currnode));

      ntComment:
        if not IgnoreComments then
          cursor.InternalAppend(doc.CreateCommentBuf(currnode^.FValueStart, currnode^.FValueLength));

      ntElement:
        begin
          element := LoadElement(doc, currnode, reader.AttributeCount);
          cursor.InternalAppend(element);
          cursor := element;
        end;

      ntEndElement:
          cursor := TDOMNode_WithChildren(cursor.ParentNode);

      ntDocumentType:
        cursor.InternalAppend(TDOMDocumentType.Create(doc, DtdSchemaInfo));

      ntEntityReference:
        begin
          cursor.InternalAppend(doc.CreateEntityReference(currnode^.FQName^.Key));
          { Seeing an entity reference while expanding means that the entity
            fails to expand. }
          if not ExpandEntities then
          begin
            { Make reader iterate through contents of the reference,
              to ensure correct validation events and character counts. }
            ResolveEntity;
            while currnodeptr^^.FNodeType <> ntEndEntity do
              Read;
          end;
        end;
=======
    dtEntity, dtEntities: begin
      StartPos := 1;
      while StartPos <= L do
      begin
        EndPos := StartPos;
        while (EndPos <= L) and (attrData^.FValueStr[EndPos] <> #32) do
          Inc(EndPos);
        Entity := TEntityDecl(FDocType.Entities.Get(@attrData^.FValueStr[StartPos], EndPos-StartPos));
        if (Entity = nil) or (Entity.FNotationName = '') then
          ValidationError('Attribute ''%s'' type mismatch', [attrData^.FQName^.Key], -1);
        StartPos := EndPos + 1;
      end;
>>>>>>> origin/cpstrnew
    end;
  until not Read;
end;

<<<<<<< HEAD
function TLoader.CreatePINode(currnode: PNodeData): TDOMNode;
var
  s: DOMString;
begin
  SetString(s, currnode^.FValueStart, currnode^.FValueLength);
  result := Doc.CreateProcessingInstruction(currnode^.FQName^.Key, s);
end;
=======
>>>>>>> origin/fixes_2_2

function TLoader.CreateCDATANode(currnode: PNodeData): TDOMNode;
=======
procedure TXMLReader.ValidateDTD;
>>>>>>> origin/cpstrnew
var
  s: XMLString;
begin
<<<<<<< HEAD
  SetString(s, currnode^.FValueStart, currnode^.FValueLength);
  result := doc.CreateCDATASection(s);
end;



procedure TLoader.ProcessDTD(ADoc: TDOMDocument; AReader: TXMLTextReader);
begin
  AReader.DtdSchemaInfo := TDTDModel.Create(AReader.NameTable);
  // TODO: DTD labeled version 1.1 will be rejected - must set FXML11 flag
  doc.AppendChild(TDOMDocumentType.Create(doc, AReader.DtdSchemaInfo));
  AReader.ParseDTD;
=======
=======
>>>>>>> origin/cpstrnew
  for I := 0 to FForwardRefs.Count-1 do
    with PForwardRef(FForwardRefs[I])^ do
      if FDocType.Notations.Get(PWideChar(Value), Length(Value)) = nil then
        DoErrorPos(esError, 'Notation ''%s'' is not declared', [Value], Loc);
end;


function TXMLReader.DoCDSect(ch: PWideChar; Count: Integer): TDOMNode;
var
  s: WideString;
begin
  Assert(not FCDSectionsAsText, 'Should not be called when CDSectionsAsText=True');

  SetString(s, ch, Count);
  result := doc.CreateCDATASection(s);
<<<<<<< HEAD
=======
  for I := 0 to FForwardRefs.Count-1 do
    with PForwardRef(FForwardRefs[I])^ do
      if FDocType.Notations.Get(PWideChar(Value), Length(Value)) = nil then
        DoErrorPos(esError, 'Notation ''%s'' is not declared', [Value], Loc);
end;


function TXMLReader.DoCDSect(ch: PWideChar; Count: Integer): TDOMNode;
var
  s: WideString;
begin
  Assert(not FCDSectionsAsText, 'Should not be called when CDSectionsAsText=True');

  SetString(s, ch, Count);
  result := doc.CreateCDATASection(s);
=======
  for I := 0 to FForwardRefs.Count-1 do
    with PForwardRef(FForwardRefs[I])^ do
      if FDocType.Notations.Get(PWideChar(Value), Length(Value)) = nil then
        DoErrorPos(esError, 'Notation ''%s'' is not declared', [Value], Loc);
end;


function TXMLReader.DoCDSect(ch: PWideChar; Count: Integer): TDOMNode;
var
  s: WideString;
begin
  Assert(not FCDSectionsAsText, 'Should not be called when CDSectionsAsText=True');

  SetString(s, ch, Count);
  result := doc.CreateCDATASection(s);
end;

procedure TXMLReader.DoNotationDecl(const aName, aPubID, aSysID: WideString);
var
  Notation: TNotationDecl;
  Entry: PHashItem;
begin
  Entry := FDocType.Notations.FindOrAdd(PWideChar(aName), Length(aName));
  if Entry^.Data = nil then
  begin
    Notation := TNotationDecl.Create;
    Notation.FName := aName;
    Notation.FPublicID := aPubID;
    Notation.FSystemID := aSysID;
    Entry^.Data := Notation;
  end
  else
    ValidationError('Duplicate notation declaration: ''%s''', [aName]);
end;

function TXMLReader.AddId(aNodeData: PNodeData): Boolean;
var
  e: PHashItem;
begin
  if FIDMap = nil then
    FIDMap := THashTable.Create(256, False);
  e := FIDMap.FindOrAdd(PWideChar(aNodeData^.FValueStr), Length(aNodeData^.FValueStr), Result);
  Result := not Result;
  if Result then
    aNodeData^.FIDEntry := e;
end;

function TXMLReader.AllocAttributeData: PNodeData;
begin
  Result := AllocNodeData(FNesting + FAttrCount + 1);
  Result^.FNodeType := ntAttribute;
  Result^.FIsDefault := False;
  Inc(FAttrCount);
end;

=======
end;

procedure TXMLReader.DoNotationDecl(const aName, aPubID, aSysID: WideString);
var
  Notation: TNotationDecl;
  Entry: PHashItem;
begin
  Entry := FDocType.Notations.FindOrAdd(PWideChar(aName), Length(aName));
  if Entry^.Data = nil then
  begin
    Notation := TNotationDecl.Create;
    Notation.FName := aName;
    Notation.FPublicID := aPubID;
    Notation.FSystemID := aSysID;
    Entry^.Data := Notation;
  end
  else
    ValidationError('Duplicate notation declaration: ''%s''', [aName]);
=======
function TXMLReader.ParseEntityDeclValue(Delim: WideChar): Boolean;   // [9]
var
  CurrentEntity: TObject;
begin
  CurrentEntity := FSource.FEntity;
  if FEntityValue.Buffer = nil then
    BufAllocate(FEntityValue, 256);
  FEntityValue.Length := 0;
  // "Included in literal": process until delimiter hit IN SAME context
  while not ((FSource.FEntity = CurrentEntity) and CheckForChar(Delim)) do
  if CheckForChar('%') then
  begin
    CheckName;
    ExpectChar(';');
    if FSource.DTDSubsetType = dsInternal then
      FatalError('PE reference not allowed here in internal subset', FName.Length+2);
    StartPE;
  end
  else if FCurChar = '&' then  // CharRefs: include, EntityRefs: bypass
  begin
    if not ParseCharRef(FEntityValue) then
    begin
      BufAppend(FEntityValue, '&');
      BufAppendChunk(FEntityValue, FName.Buffer, FName.Length);
      BufAppend(FEntityValue, ';');
    end;
  end
  else if FCurChar <> #0 then         // Regular character
  begin
    BufAppend(FEntityValue, FCurChar);
    GetChar;
  end
  else if (FSource.FEntity = CurrentEntity) or not ContextPop then         // #0
  begin
    Result := False;
    Exit;
  end;
  Result := True;
>>>>>>> graemeg/fixes_2_2
end;

function TXMLReader.AddId(aNodeData: PNodeData): Boolean;
var
  e: PHashItem;
begin
<<<<<<< HEAD
  if FIDMap = nil then
    FIDMap := THashTable.Create(256, False);
  e := FIDMap.FindOrAdd(PWideChar(aNodeData^.FValueStr), Length(aNodeData^.FValueStr), Result);
  Result := not Result;
  if Result then
    aNodeData^.FIDEntry := e;
end;

function TXMLReader.AllocAttributeData: PNodeData;
begin
  Result := AllocNodeData(FNesting + FAttrCount + 1);
  Result^.FNodeType := ntAttribute;
  Result^.FIsDefault := False;
  Inc(FAttrCount);
end;

>>>>>>> origin/cpstrnew
function TXMLReader.AllocNodeData(AIndex: Integer): PNodeData;
begin
  {make sure we have an extra slot to place child text/comment/etc}
  if AIndex >= Length(FNodeStack)-1 then
    SetLength(FNodeStack, AIndex * 2 + 2);
<<<<<<< HEAD

  Result := @FNodeStack[AIndex];
  Result^.FPrefix := nil;
  Result^.FNsUri := nil;
  Result^.FIDEntry := nil;
end;

=======

  Result := @FNodeStack[AIndex];
  Result^.FPrefix := nil;
  Result^.FNsUri := nil;
  Result^.FIDEntry := nil;
end;
=======
  if not SkipWhitespace(True) then
    FatalError('Expected whitespace');
  NDataAllowed := True;
  Map := FDocType.Entities;
  if CheckForChar('%') then                  // [72]
  begin
    ExpectWhitespace;
    NDataAllowed := False;
    if FPEMap = nil then
      FPEMap := TDOMNamedNodeMap.Create(FDocType, ENTITY_NODE);
    Map := FPEMap;
  end;

  Entity := TDOMEntityEx.Create(Doc);
  Entity.SetReadOnly(True);
  try
    Entity.FExternallyDeclared := FSource.DTDSubsetType <> dsInternal;
    Entity.FName := ExpectName;
    CheckNCName;
    ExpectWhitespace;
>>>>>>> graemeg/fixes_2_2

>>>>>>> origin/cpstrnew
function TXMLReader.AllocAttributeValueChunk(APrev: PNodeData): PNodeData;
begin
  { when parsing DTD, don't take ownership of allocated data }
  if FState = rsDTD then
  begin
    New(result);
    FillChar(result^, sizeof(TNodeData), 0);
  end
  else
  begin
    result := FFreeAttrChunk;
    if Assigned(result) then
    begin
<<<<<<< HEAD
      FFreeAttrChunk := result^.FNext;
      result^.FNext := nil;
    end
    else { no free chunks, create a new one }
    begin
      New(result);
      FillChar(result^, sizeof(TNodeData), 0);
      FAttrChunks.Add(result);
=======
      NDataAllowed := False;
      Delim := FCurChar;
      GetChar;
      StoreLocation(Entity.FStartLocation);
      if not ParseEntityDeclValue(Delim) then
        DoErrorPos(esFatal, 'Literal has no closing quote', Entity.FStartLocation);
      SetString(Entity.FReplacementText, FEntityValue.Buffer, FEntityValue.Length);
    end
    else
      if not ParseExternalID(Entity.FSystemID, Entity.FPublicID, False) then
        FatalError('Expected entity value or external ID');

    if NDataAllowed then                // [76]
    begin
<<<<<<< HEAD
      if FCurChar <> '>' then
        ExpectWhitespace;
      if FSource.Matches('NDATA') then
      begin
        ExpectWhitespace;
        Entity.FNotationName := ExpectName;
        AddForwardRef(FNotationRefs, FName.Buffer, FName.Length);
        // SAX: DTDHandler.UnparsedEntityDecl(...);
      end;
>>>>>>> graemeg/fixes_2_2
    end;
  end;
  APrev^.FNext := result;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

procedure TXMLReader.CleanupAttributes;
var
<<<<<<< HEAD
<<<<<<< HEAD
  Notation: TNotationDecl;
  Entry: PHashItem;
begin
  Entry := FDocType.Notations.FindOrAdd(PWideChar(aName), Length(aName));
  if Entry^.Data = nil then
  begin
    Notation := TNotationDecl.Create;
    Notation.FName := aName;
    Notation.FPublicID := aPubID;
    Notation.FSystemID := aSysID;
    Entry^.Data := Notation;
  end
  else
    ValidationError('Duplicate notation declaration: ''%s''', [aName]);
end;

function TXMLReader.AddId(aNodeData: PNodeData): Boolean;
var
<<<<<<< HEAD
  e: PHashItem;
begin
  if FIDMap = nil then
    FIDMap := THashTable.Create(256, False);
  e := FIDMap.FindOrAdd(PWideChar(aNodeData^.FValueStr), Length(aNodeData^.FValueStr), Result);
  Result := not Result;
  if Result then
    aNodeData^.FIDEntry := e;
end;

function TXMLReader.AllocAttributeData: PNodeData;
begin
  Result := AllocNodeData(FNesting + FAttrCount + 1);
  Result^.FNodeType := ntAttribute;
  Result^.FIsDefault := False;
  Inc(FAttrCount);
end;

function TXMLReader.AllocNodeData(AIndex: Integer): PNodeData;
begin
  {make sure we have an extra slot to place child text/comment/etc}
  if AIndex >= Length(FNodeStack)-1 then
    SetLength(FNodeStack, AIndex * 2 + 2);

  Result := @FNodeStack[AIndex];
  Result^.FPrefix := nil;
  Result^.FNsUri := nil;
  Result^.FIDEntry := nil;
end;
=======
  IncludeLevel: Integer;
  IgnoreLevel: Integer;
  CurrentEntity: TObject;
  IncludeLoc: TLocation;
  IgnoreLoc: TLocation;
  CondType: (ctUnknown, ctInclude, ctIgnore);
begin
  IncludeLevel := 0;
  IgnoreLevel := 0;
  repeat
    FRecognizePE := True;      // PERef between declarations should always be recognized
    SkipWhitespace;
    FRecognizePE := False;
>>>>>>> graemeg/fixes_2_2

function TXMLReader.AllocAttributeValueChunk(APrev: PNodeData): PNodeData;
begin
  { when parsing DTD, don't take ownership of allocated data }
  if FState = rsDTD then
  begin
    New(result);
    FillChar(result^, sizeof(TNodeData), 0);
  end
  else
  begin
    result := FFreeAttrChunk;
    if Assigned(result) then
    begin
<<<<<<< HEAD
      FFreeAttrChunk := result^.FNext;
      result^.FNext := nil;
    end
    else { no free chunks, create a new one }
    begin
      New(result);
      FillChar(result^, sizeof(TNodeData), 0);
      FAttrChunks.Add(result);
=======
  i: Integer;
begin
  {cleanup only specified attributes; default ones are owned by DTD}
  for i := 1 to FSpecifiedAttrs do
    CleanupAttribute(@FNodeStack[FNesting+i]);
  FAttrCleanupFlag := False;
end;

procedure TXMLReader.CleanupAttribute(aNode: PNodeData);
var
  chunk, tmp: PNodeData;
begin
  chunk := aNode^.FNext;
  while Assigned(chunk) do
  begin
    tmp := chunk^.FNext;
    chunk^.FNext := FFreeAttrChunk;
    FFreeAttrChunk := chunk;
    chunk := tmp;
  end;
  aNode^.FNext := nil;
end;

procedure TXMLReader.SetNodeInfoWithValue(typ: TXMLNodeType; AName: PHashItem = nil);
begin
  {FNesting+1 is available due to overallocation in AllocNodeData() }
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := typ;
  FCurrNode^.FQName := AName;
  FCurrNode^.FValueStart := FValue.Buffer;
  FCurrNode^.FValueLength := FValue.Length;
end;

procedure TXMLReader.PushVC(aElDef: TElementDecl);
begin
  Inc(FValidatorNesting);
  if FValidatorNesting >= Length(FValidators) then
    SetLength(FValidators, FValidatorNesting * 2);

  with FValidators[FValidatorNesting] do
  begin
=======
  i: Integer;
begin
  {cleanup only specified attributes; default ones are owned by DTD}
  for i := 1 to FSpecifiedAttrs do
    CleanupAttribute(@FNodeStack[FNesting+i]);
  FAttrCleanupFlag := False;
end;

procedure TXMLReader.CleanupAttribute(aNode: PNodeData);
var
  chunk, tmp: PNodeData;
begin
  chunk := aNode^.FNext;
  while Assigned(chunk) do
  begin
    tmp := chunk^.FNext;
    chunk^.FNext := FFreeAttrChunk;
    FFreeAttrChunk := chunk;
    chunk := tmp;
  end;
  aNode^.FNext := nil;
end;

procedure TXMLReader.SetNodeInfoWithValue(typ: TXMLNodeType; AName: PHashItem = nil);
begin
  {FNesting+1 is available due to overallocation in AllocNodeData() }
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := typ;
  FCurrNode^.FQName := AName;
  FCurrNode^.FValueStart := FValue.Buffer;
  FCurrNode^.FValueLength := FValue.Length;
end;

procedure TXMLReader.PushVC(aElDef: TElementDecl);
begin
  Inc(FValidatorNesting);
  if FValidatorNesting >= Length(FValidators) then
    SetLength(FValidators, FValidatorNesting * 2);

  with FValidators[FValidatorNesting] do
  begin
>>>>>>> origin/cpstrnew
    FElementDef := aElDef;
    FCurCP := nil;
    FFailed := False;
    FContentType := ctAny;
    FSaViolation := False;
    if Assigned(aElDef) then
    begin
      FContentType := aElDef.ContentType;
      FSaViolation := FStandalone and aElDef.ExternallyDeclared;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;
  end;
  APrev^.FNext := result;
>>>>>>> graemeg/cpstrnew
end;

<<<<<<< HEAD
procedure TXMLReader.CleanupAttributes;
var
<<<<<<< HEAD
  Notation: TNotationDecl;
  Entry: PHashItem;
=======
procedure TXMLReader.PopVC;
begin
  if (FNesting = 1) and (not FFragmentMode) then
    FState := rsEpilog;
  if FNesting > 0 then Dec(FNesting);
  FCurrNode := @FNodeStack[FNesting];
  FNext := xtText;
end;

procedure TXMLReader.PopVC;
begin
  if (FNesting = 1) and (not FFragmentMode) then
    FState := rsEpilog;
  if FNesting > 0 then Dec(FNesting);
  FCurrNode := @FNodeStack[FNesting];
  FNext := xtText;
end;

{ TElementValidator }

function TElementValidator.IsElementAllowed(Def: TElementDecl): Boolean;
var
  Next: TContentParticle;
>>>>>>> graemeg/cpstrnew
begin
  Entry := FDocType.Notations.FindOrAdd(PWideChar(aName), Length(aName));
  if Entry^.Data = nil then
  begin
<<<<<<< HEAD
    Notation := TNotationDecl.Create;
    Notation.FName := aName;
    Notation.FPublicID := aPubID;
    Notation.FSystemID := aSysID;
    Entry^.Data := Notation;
  end
  else
    ValidationError('Duplicate notation declaration: ''%s''', [aName]);
end;
=======
    case FElementDef.ContentType of
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

function TXMLReader.AddId(aNodeData: PNodeData): Boolean;
var
  e: PHashItem;
begin
  if FIDMap = nil then
    FIDMap := THashTable.Create(256, False);
  e := FIDMap.FindOrAdd(PWideChar(aNodeData^.FValueStr), Length(aNodeData^.FValueStr), Result);
  Result := not Result;
  if Result then
    aNodeData^.FIDEntry := e;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TXMLReader.AllocAttributeData: PNodeData;
begin
  Result := AllocNodeData(FNesting + FAttrCount + 1);
  Result^.FNodeType := ntAttribute;
  Result^.FIsDefault := False;
  Inc(FAttrCount);
=======
=======
>>>>>>> origin/cpstrnew
      ctChildren, ctMixed: begin
        if FFailed then     // if already detected a mismatch, don't waste time
          Exit;
        if FCurCP = nil then
          Next := FElementDef.RootCP.FindFirst(Def)
        else
          Next := FCurCP.FindNext(Def, 0); { second arg ignored here }
        Result := Assigned(Next);
        if Result then
          FCurCP := Next
        else
          FFailed := True;  // used to prevent extra error at the end of element
      end;
      // ctAny, ctUndeclared: returns True by default
    end;
  end;
>>>>>>> graemeg/cpstrnew
end;
=======
      ExpectString(']]>');
      Dec(IncludeLevel);
      Continue;
    end;

    if not CheckForChar('<') then
      Break;

    CurrentEntity := FSource.FEntity;

    if FCurChar = '?' then
      ParsePI
    else
    begin
      ExpectChar('!');
      if FCurChar = '-' then
        ParseComment
      else if FCurChar = '[' then
      begin
        if FSource.DTDSubsetType = dsInternal then
          FatalError('Conditional sections are not allowed in internal subset');

        FRecognizePE := True;
        GetChar; // skip '['
        SkipWhitespace;

        CondType := ctUnknown;  // satisfy compiler
        if FSource.Matches('INCLUDE') then
          CondType := ctInclude
        else if FSource.Matches('IGNORE') then
          CondType := ctIgnore
        else
          FatalError('Expected "INCLUDE" or "IGNORE"');

        SkipWhitespace;
        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        ExpectChar('[');
        if CondType = ctInclude then
        begin
          if IncludeLevel = 0 then
            StoreLocation(IncludeLoc);
          Inc(IncludeLevel);
        end
        else if CondType = ctIgnore then
        begin
          StoreLocation(IgnoreLoc);
          IgnoreLevel := 1;
          repeat
            FRecognizePE := False;    // PEs not recognized in IGNORE section
            if CheckForChar('<') and CheckForChar('!') and CheckForChar('[') then
              Inc(IgnoreLevel)
            else if CheckForChar(']') and CheckForChar(']') and CheckForChar('>') then
              Dec(IgnoreLevel)
            else GetChar;
          until (IgnoreLevel=0) or (FCurChar = #0);
// Since PE's are not recognized in ignore sections, reaching EOF is fatal.
          if FCurChar = #0 then
            Break;
        end;
      end
      else
      begin
        FRecognizePE := FSource.DTDSubsetType <> dsInternal;
        FInsideDecl := True;
        if FSource.Matches('ELEMENT') then
          ParseElementDecl
        else if FSource.Matches('ENTITY') then
          ParseEntityDecl
        else if FSource.Matches('ATTLIST') then
          ParseAttlistDecl
        else if FSource.Matches('NOTATION') then
          ParseNotationDecl
        else
          FatalError('Illegal markup declaration');

        SkipWhitespace;
        FRecognizePE := False;

        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        ExpectChar('>');
        FInsideDecl := False;
      end;
    end;
  until False;
  FRecognizePE := False;
  if (IncludeLevel > 0) or (IgnoreLevel > 0) then
  begin
    if IncludeLevel > 0 then
      FTokenStart := IncludeLoc
    else
      FTokenStart := IgnoreLoc;
    FatalError('Conditional section is not closed', -1);
=======
      SaveCursor := FCursor;
      FCursor := AEntity;         // build child node tree for the entity
      try
        AEntity.SetReadOnly(False);
        if InAttr then
          DoParseAttValue(#0)
        else
          DoParseFragment;
        AEntity.FResolved := True;
      finally
        AEntity.SetReadOnly(True);
        ContextPop;
        FCursor := SaveCursor;
        FValue.Length := 0;
      end;
    end;
  end;
  if (not FExpandEntities) or (not AEntity.FResolved) then
  begin
    // This will clone Entity children
    FCursor.AppendChild(doc.CreateEntityReference(RefName));
    Exit;
  end;

  Child := AEntity.FirstChild;  // clone the entity node tree
  while Assigned(Child) do
  begin
    FCursor.AppendChild(Child.CloneNode(True));
    Child := Child.NextSibling;
>>>>>>> origin/fixes_2_2
  end;
  if (FSource.DTDSubsetType = dsInternal) and (FCurChar = ']') then
    Exit;
  if FCurChar <> #0 then
    FatalError('Illegal character in DTD');
end;

procedure TXMLReader.ProcessDTD(ASource: TXMLCharSource);
begin
<<<<<<< HEAD
  doc := TXMLDocument.Create;
  FDocType := TDOMDocumentTypeEx.Create(doc);
  // TODO: DTD labeled version 1.1 will be rejected - must set FXML11 flag
  // DONE: It's ok to have FCursor=nil now
  doc.AppendChild(FDocType);
  Initialize(ASource);
  ParseMarkupDecl;
end;

procedure TXMLReader.ParseCDSect;               // [18]
begin
  ExpectString('[CDATA[');
  StoreLocation(FTokenStart);
  if FState <> rsRoot then
    FatalError('Illegal at document level');
  FValue.Length := 0;
  repeat
    BufAppend(FValue, FCurChar);
    GetChar;
    with FValue do
      if (Length >= 3) and (Buffer[Length-1] = '>') and
      (Buffer[Length-2] = ']') and (Buffer[Length-3] = ']') then
    begin
      DoCDSect(Buffer, Length-3);
      Exit;
    end;
  until FCurChar = #0;
  FatalError('Unterminated CDATA section', -1);
end;

procedure TXMLReader.ParseContent;
var
  nonWs: Boolean;
begin
  repeat
    if FCurChar = '<' then
    begin
      GetChar;
      if FCurChar = '/' then  // end-tags are as frequent as start-tags
        Break;
      if CheckName([cnOptional]) then
        ParseElement
      else if FCurChar = '!' then
      begin
        GetChar;
        if FCurChar = '[' then
          ParseCDSect
        else if FCurChar = '-' then
          ParseComment
        else
          ParseDoctypeDecl;
      end
      else if FCurChar = '?' then
        ParsePI
      else
        RaiseNameNotFound;
    end
    else
    begin
      FValue.Length := 0;
      nonWs := False;
      StoreLocation(FTokenStart);
      while (FCurChar <> '<') and (FCurChar <> #0) do
      begin
        if FCurChar <> '&' then
        begin
          if (FCurChar <> #32) and (FCurChar <> #10) and (FCurChar <> #9) and (FCurChar <> #13) then
            nonWs := True;
          BufAppend(FValue, FCurChar);
          if FCurChar = '>' then
          with FValue do
            if (Length >= 3) and (Buffer[Length-2] = ']') and (Buffer[Length-3] = ']') then
              FatalError('Literal '']]>'' is not allowed in text', 2);
          GetChar;
        end
        else
        begin
          if FState <> rsRoot then
            FatalError('Illegal at document level');

          if FCurrContentType = ctEmpty then
            ValidationError('References are illegal in EMPTY elements', []);

          if ParseCharRef(FValue) or ResolvePredefined then
            nonWs := True // CharRef to whitespace is not considered whitespace
          else
          begin
            if (nonWs or FPreserveWhitespace) and (FValue.Length > 0)  then
            begin
              // 'Reference illegal at root' is checked above, no need to check here
              DoText(FValue.Buffer, FValue.Length, not nonWs);
              FValue.Length := 0;
            end;
            IncludeEntity(False);
          end;
        end;
      end; // while
      if FState = rsRoot then
      begin
        if (nonWs or FPreserveWhitespace) and (FValue.Length > 0)  then
        begin
          DoText(FValue.Buffer, FValue.Length, not nonWs);
          FValue.Length := 0;
        end;
      end
      else if nonWs then
        FatalError('Illegal at document level', -1);
    end;
  until FCurChar = #0;
end;

// Element name already in FNameBuffer
procedure TXMLReader.ParseElement;    // [39] [40] [44]
var
  NewElem: TDOMElement;
  ElDef: TDOMElementDef;
  IsEmpty: Boolean;
begin
  if FState > rsRoot then
    FatalError('Only one top-level element allowed', FName.Length)
  else if FState < rsRoot then
  begin
    if FValidate then
      ValidateRoot;
    FState := rsRoot;
  end;

  NewElem := doc.CreateElementBuf(FName.Buffer, FName.Length);
  FCursor.AppendChild(NewElem);
  // we're about to process a new set of attributes
  Inc(FAttrTag);

  // Find declaration for this element
  ElDef := nil;
  if Assigned(FDocType) then
  begin
    ElDef := TDOMElementDef(FDocType.ElementDefs.GetNamedItem(NewElem.TagName));
    if (ElDef = nil) or (not ElDef.HasElementDecl) then
      ValidationError('Using undeclared element ''%s''',[NewElem.TagName], FName.Length);
  end;

  // Check if new element is allowed in current context
  if FValidate and not FValidator[FNesting].IsElementAllowed(ElDef) then
    ValidationError('Element ''%s'' is not allowed in this context',[NewElem.TagName], FName.Length);

  IsEmpty := False;
  while (FSource.FBuf^ <> '>') and (FSource.FBuf^ <> '/') do
  begin
    SkipS(True);
    if (FSource.FBuf^ = '>') or (FSource.FBuf^ = '/') then
      Break;
    ParseAttribute(NewElem, ElDef);
  end;

  if FSource.FBuf^ = '/' then
  begin
    IsEmpty := True;
    GetChar;
  end;
  ExpectChar('>');

  ProcessDefaultAttributes(NewElem, ElDef);

  PushVC(ElDef);
  // SAX: ContentHandler.StartElement(...)
  // SAX: ContentHandler.StartPrefixMapping(...)
>>>>>>> graemeg/fixes_2_2

function TXMLReader.AllocNodeData(AIndex: Integer): PNodeData;
begin
  {make sure we have an extra slot to place child text/comment/etc}
  if AIndex >= Length(FNodeStack)-1 then
    SetLength(FNodeStack, AIndex * 2 + 2);

  Result := @FNodeStack[AIndex];
  Result^.FPrefix := nil;
  Result^.FNsUri := nil;
  Result^.FIDEntry := nil;
end;

function TXMLReader.AllocAttributeValueChunk(APrev: PNodeData): PNodeData;
begin
  { when parsing DTD, don't take ownership of allocated data }
  if FState = rsDTD then
  begin
<<<<<<< HEAD
    New(result);
    FillChar(result^, sizeof(TNodeData), 0);
  end
  else
  begin
    result := FFreeAttrChunk;
    if Assigned(result) then
    begin
      FFreeAttrChunk := result^.FNext;
      result^.FNext := nil;
    end
    else { no free chunks, create a new one }
    begin
      New(result);
      FillChar(result^, sizeof(TNodeData), 0);
      FAttrChunks.Add(result);
    end;
  end;
  APrev^.FNext := result;
=======
  SetString(PEName, FName.Buffer, FName.Length);
  PEnt := nil;
  if Assigned(FPEMap) then
    PEnt := FPEMap.GetNamedItem(PEName) as TDOMEntityEx;
  if PEnt = nil then    // TODO -cVC: Referencing undefined PE
  begin                 // (These are classified as 'optional errors'...)
//    ValidationError('Undefined parameter entity referenced: %s', [PEName]);
    Exit;
  end;

  if PEnt.FOnStack then
    FatalError('Entity ''%%%s'' recursively references itself', [PEnt.NodeName]);

  PEnt.FBetweenDecls := not FInsideDecl;
  ContextPush(PEnt);
  FHavePERefs := True;
end;

procedure TXMLReader.ExpectAttValue;    // [10]
var
  Delim: WideChar;
begin
  if (FCurChar <> '''') and (FCurChar <> '"') then
    RaiseExpectedQmark;
  Delim := FCurChar;
  GetChar;  // skip quote
  StoreLocation(FTokenStart);
  if not DoParseAttValue(Delim) then
    FatalError('Literal has no closing quote',-1);
  GetChar;
end;

procedure TXMLReader.SkipQuotedLiteral(out Literal: WideString; required: Boolean);
var
  Delim: WideChar;
begin
  if (FCurChar = '''') or (FCurChar = '"') then
  begin
    Delim := FCurChar;
    GetChar;  // skip quote
    StoreLocation(FTokenStart);
    FValue.Length := 0;
    while (FCurChar <> Delim) and (FCurChar <> #0) do
    begin
      BufAppend(FValue, FCurChar);
      GetChar;
    end;
    if not CheckForChar(Delim) then
      FatalError('Literal has no closing quote', -1);
    SetString(Literal, FValue.Buffer, FValue.Length);
  end
  else if required then
    RaiseExpectedQMark;
end;

procedure TXMLReader.SkipPubidLiteral(out Literal: WideString);         // [12]
var
  I: Integer;
  wc: WideChar;
begin
  SkipQuotedLiteral(Literal);
  for I := 1 to Length(Literal) do
  begin
    wc := Literal[I];
    if (wc > #255) or not (Char(ord(wc)) in PubidChars) then
      FatalError('Illegal Public ID literal', -1);
    if (wc = #10) or (wc = #13) then
      Literal[I] := #32;
  end;
>>>>>>> origin/fixes_2_2
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLReader.CleanupAttributes;
var
  i: Integer;
begin
<<<<<<< HEAD
  {cleanup only specified attributes; default ones are owned by DTD}
  for i := 1 to FSpecifiedAttrs do
    CleanupAttribute(@FNodeStack[FNesting+i]);
  FAttrCleanupFlag := False;
=======
  ExpectString('--');
  StoreLocation(FTokenStart);
  FValue.Length := 0;
  repeat
    BufAppend(FValue, FCurChar);
    GetChar;
    with FValue do
      if (Length >= 2) and (Buffer[Length-1] = '-') and
      (Buffer[Length-2] = '-') then
      begin
        ExpectChar('>');
        Dec(Length, 2);
        DoComment(Buffer, Length);
        Exit;
      end;
  until FCurChar = #0;
  FatalError('Unterminated comment', -1);
>>>>>>> origin/fixes_2_2
end;

procedure TXMLReader.CleanupAttribute(aNode: PNodeData);
var
<<<<<<< HEAD
  chunk, tmp: PNodeData;
begin
  chunk := aNode^.FNext;
  while Assigned(chunk) do
  begin
    tmp := chunk^.FNext;
    chunk^.FNext := FFreeAttrChunk;
    FFreeAttrChunk := chunk;
    chunk := tmp;
=======
    FCursor := NewElem;
    if not FPreserveWhitespace then   // critical for testsuite compliance
      SkipS;
    ParseContent;
    if FCurChar = '/' then         // Get ETag [42]
    begin
      GetChar;
      StoreLocation(FTokenStart);
      CheckName;
      if not BufEquals(FName, NewElem.TagName) then
        FatalError('Unmatching element end tag (expected "</%s>")', [NewElem.TagName], FName.Length);
      SkipS;
      ExpectChar('>');
    end
    else if FCurChar <> #0 then
      RaiseNameNotFound
    else // End of stream in content
      FatalError('End-tag is missing for ''%s''', [NewElem.TagName]);
=======
  Name, Value: WideString;
  PINode: TDOMProcessingInstruction;
begin
  GetChar;      // skip '?'
  Name := ExpectName;
  CheckNCName;
  with FName do
    if (Length = 3) and
     ((Buffer[0] = 'X') or (Buffer[0] = 'x')) and
     ((Buffer[1] = 'M') or (Buffer[1] = 'm')) and
     ((Buffer[2] = 'L') or (Buffer[2] = 'l')) then
  begin
    if Name <> 'xml' then
      FatalError('''xml'' is a reserved word; it must be lowercase', FName.Length)
    else
      FatalError('XML declaration is not allowed here', FName.Length);
>>>>>>> origin/fixes_2_2
  end;
  // SAX: ContentHandler.EndElement(...)
  // SAX: ContentHandler.EndPrefixMapping(...)
  TDOMNode(FCursor) := NewElem.ParentNode;
  if FCursor = doc then
    FState := rsEpilog;

<<<<<<< HEAD
  if FValidate and FValidator[FNesting].Incomplete then
    ValidationError('Element ''%s'' is missing required sub-elements', [NewElem.TagName]);

  PopVC;
end;

procedure TXMLReader.ParseAttribute(Elem: TDOMElement; ElDef: TDOMElementDef);
var
  attr: TDOMAttr;
  AttDef: TDOMAttrDef;
  OldAttr: TDOMNode;

procedure CheckValue;
=======
  if FCurChar <> '?' then
    SkipS(True);

  FValue.Length := 0;
  StoreLocation(FTokenStart);
  repeat
    BufAppend(FValue, FCurChar);
    GetChar;
    with FValue do
      if (Length >= 2) and (Buffer[Length-1] = '>') and
        (Buffer[Length-2] = '?') then
      begin
        Dec(Length, 2);
        SetString(Value, Buffer, Length);
        // SAX: ContentHandler.ProcessingInstruction(Name, Value);

        if FCurrContentType = ctEmpty then
            ValidationError('Processing instructions are not allowed within EMPTY elements', []);

        PINode := Doc.CreateProcessingInstruction(Name, Value);
        if Assigned(FCursor) then
          FCursor.AppendChild(PINode)
        else  // to comply with certain tests, insert PI from DTD before DTD
          Doc.InsertBefore(PINode, FDocType);
        Exit;
      end;
  until FCurChar = #0;
  FatalError('Unterminated processing instruction', -1);
end;

procedure TXMLReader.ParseXmlOrTextDecl(TextDecl: Boolean);
>>>>>>> origin/fixes_2_2
var
  AttValue, OldValue: WideString;
begin
<<<<<<< HEAD
  if FStandalone and AttDef.FExternallyDeclared then
  begin
    OldValue := Attr.Value;
    TDOMAttrDef(Attr).FDataType := AttDef.FDataType;
    AttValue := Attr.Value;
    if AttValue <> OldValue then
      StandaloneError(-1);
  end
  else
  begin
    TDOMAttrDef(Attr).FDataType := AttDef.FDataType;
    AttValue := Attr.Value;
=======
  FCurChar := FSource.NextChar;  // don't update location here
  SkipS(True);
  // VersionInfo: optional in TextDecl, required in XmlDecl
  if (not TextDecl) or (FCurChar = 'v') then
  begin
    ExpectString('version');                              // [24]
    ExpectEq;
    SkipQuotedLiteral(TmpStr);
    IsXML11 := False;
    if TmpStr = '1.1' then     // Checking for bad chars is implied
      IsXML11 := True
    else if TmpStr <> '1.0' then
    { should be no whitespace in these literals, but that isn't checked now }
      FatalError('Illegal version number', -1);

    if not TextDecl then
    begin
      if doc.InheritsFrom(TXMLDocument) then
        TXMLDocument(doc).XMLVersion := TmpStr;
      if IsXML11 then
        XML11_BuildTables;
    end
    else   // parsing external entity
      if IsXML11 and not FXML11 then
        FatalError('XML 1.0 document cannot invoke XML 1.1 entities', -1);

    if FCurChar <> '?' then
      SkipS(True);
  end;

  // EncodingDecl: required in TextDecl, optional in XmlDecl
  if TextDecl or (FCurChar = 'e') then                    // [80]
  begin
    ExpectString('encoding');
    ExpectEq;
    SkipQuotedLiteral(TmpStr);

    if not IsValidXmlEncoding(TmpStr) then
      FatalError('Illegal encoding name', -1);

    if not FSource.SetEncoding(TmpStr) then  // <-- Wide2Ansi conversion here
      FatalError('Encoding ''%s'' is not supported', [TmpStr], -1);
    // getting here means that specified encoding is supported
    // TODO: maybe assign the 'preferred' encoding name?
    if not TextDecl and doc.InheritsFrom(TXMLDocument) then
      TXMLDocument(doc).Encoding := TmpStr;

    if FCurChar <> '?' then
      SkipS(True);
>>>>>>> origin/fixes_2_2
  end;
  // TODO: what about normalization of AttDef.Value? (Currently it IS normalized)
  if (AttDef.FDefault = adFixed) and (AttDef.Value <> AttValue) then
    ValidationError('Value of attribute ''%s'' does not match its #FIXED default',[AttDef.Name], -1);
  if not ValidateAttrSyntax(AttDef, AttValue) then
    ValidationError('Attribute ''%s'' type mismatch', [AttDef.Name], -1);
  ValidateAttrValue(Attr, AttValue);
end;

begin
  CheckName;
  attr := doc.CreateAttributeBuf(FName.Buffer, FName.Length);

  if Assigned(ElDef) then
  begin
<<<<<<< HEAD
    AttDef := TDOMAttrDef(ElDef.GetAttributeNode(attr.Name));
    if AttDef = nil then
      ValidationError('Using undeclared attribute ''%s'' on element ''%s''',[attr.Name, Elem.TagName], FName.Length)
    else
      AttDef.Tag := FAttrTag;  // indicates that this one is specified
  end
  else
    AttDef := nil;

  // !!cannot use TDOMElement.SetAttributeNode because it will free old attribute
  OldAttr := Elem.Attributes.SetNamedItem(Attr);
  if Assigned(OldAttr) then
  begin
    OldAttr.Free;
    FatalError('Duplicate attribute', FName.Length);
=======
    ExpectString('standalone');
    ExpectEq;
    SkipQuotedLiteral(TmpStr);
    if TmpStr = 'yes' then
      FStandalone := True
    else if TmpStr <> 'no' then
      FatalError('Only "yes" or "no" are permitted as values of "standalone"', -1);
    SkipS;
>>>>>>> origin/fixes_2_2
  end;
  ExpectEq;
  FCursor := attr;
  ExpectAttValue;

  if Assigned(AttDef) and ((AttDef.FDataType <> dtCdata) or (AttDef.FDefault = adFixed)) then
    CheckValue;
end;

<<<<<<< HEAD
procedure TXMLReader.AddForwardRef(aList: TFPList; Buf: PWideChar; Length: Integer);
var
  w: PForwardRef;
begin
  New(w);
  SetString(w^.Value, Buf, Abs(Length));
  if Length > 0 then
  begin
    StoreLocation(w^.Loc);
    Dec(w^.Loc.LinePos, Length);
  end
  else
    w^.Loc := FTokenStart;
  aList.Add(w);
end;

procedure TXMLReader.ClearRefs(aList: TFPList);
var
  I: Integer;
begin
  for I := 0 to aList.Count-1 do
    Dispose(PForwardRef(aList.List^[I]));
  aList.Clear;
end;

procedure TXMLReader.ValidateIdRefs;
var
  I: Integer;
begin
  for I := 0 to FIDRefs.Count-1 do
    with PForwardRef(FIDRefs.List^[I])^ do
      if Doc.GetElementById(Value) = nil then
        DoErrorPos(esError, Format('The ID ''%s'' does not match any element', [Value]), Loc);
  ClearRefs(FIDRefs);
end;

procedure TXMLReader.ProcessDefaultAttributes(Element: TDOMElement; ElDef: TDOMElementDef);
var
  Map: TDOMNamedNodeMap;
  Attr: TDOMAttr;

procedure DoDefaulting;
var
  I: Integer;
  AttDef: TDOMAttrDef;
begin
  Map := ElDef.FAttributes;

  for I := 0 to Map.Length-1 do
  begin
    AttDef := Map[I] as TDOMAttrDef;

    if AttDef.Tag <> FAttrTag then  // this one wasn't specified
    begin
      case AttDef.FDefault of
        adDefault, adFixed: begin
          if FStandalone and AttDef.FExternallyDeclared then
            StandaloneError;
          Attr := AttDef.Clone(Element);
          Element.SetAttributeNode(Attr);
          ValidateAttrValue(Attr, Attr.Value);
        end;
        adRequired:  ValidationError('Required attribute ''%s'' of element ''%s'' is missing',[AttDef.Name, Element.TagName], 0)
      end;
    end;
>>>>>>> graemeg/fixes_2_2
  end;
  aNode^.FNext := nil;
end;

<<<<<<< HEAD
procedure TXMLReader.SetNodeInfoWithValue(typ: TXMLNodeType; AName: PHashItem = nil);
begin
  {FNesting+1 is available due to overallocation in AllocNodeData() }
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := typ;
  FCurrNode^.FQName := AName;
  FCurrNode^.FValueStart := FValue.Buffer;
  FCurrNode^.FValueLength := FValue.Length;
end;

procedure TXMLReader.PushVC(aElDef: TElementDecl);
begin
  Inc(FValidatorNesting);
  if FValidatorNesting >= Length(FValidators) then
    SetLength(FValidators, FValidatorNesting * 2);

  with FValidators[FValidatorNesting] do
  begin
=======
  i: Integer;
begin
  {cleanup only specified attributes; default ones are owned by DTD}
  for i := 1 to FSpecifiedAttrs do
    CleanupAttribute(@FNodeStack[FNesting+i]);
  FAttrCleanupFlag := False;
=======
begin
  if Assigned(ElDef) and Assigned(ElDef.FAttributes) then
    DoDefaulting;
>>>>>>> graemeg/fixes_2_2
end;

procedure TXMLReader.CleanupAttribute(aNode: PNodeData);
var
  chunk, tmp: PNodeData;
begin
<<<<<<< HEAD
  chunk := aNode^.FNext;
  while Assigned(chunk) do
  begin
    tmp := chunk^.FNext;
    chunk^.FNext := FFreeAttrChunk;
    FFreeAttrChunk := chunk;
    chunk := tmp;
  end;
  aNode^.FNext := nil;
end;

procedure TXMLReader.SetNodeInfoWithValue(typ: TXMLNodeType; AName: PHashItem = nil);
begin
  {FNesting+1 is available due to overallocation in AllocNodeData() }
  FCurrNode := @FNodeStack[FNesting+1];
  FCurrNode^.FNodeType := typ;
  FCurrNode^.FQName := AName;
  FCurrNode^.FValueStart := FValue.Buffer;
  FCurrNode^.FValueLength := FValue.Length;
end;

procedure TXMLReader.PushVC(aElDef: TElementDecl);
begin
  Inc(FValidatorNesting);
  if FValidatorNesting >= Length(FValidators) then
    SetLength(FValidators, FValidatorNesting * 2);

  with FValidators[FValidatorNesting] do
  begin
>>>>>>> graemeg/cpstrnew
    FElementDef := aElDef;
    FCurCP := nil;
    FFailed := False;
    FContentType := ctAny;
    FSaViolation := False;
    if Assigned(aElDef) then
    begin
      FContentType := aElDef.ContentType;
      FSaViolation := FStandalone and aElDef.ExternallyDeclared;
    end;
  end;
end;

procedure TXMLReader.PopVC;
begin
  if (FNesting = 1) and (not FFragmentMode) then
    FState := rsEpilog;
  if FNesting > 0 then Dec(FNesting);
  FCurrNode := @FNodeStack[FNesting];
  FNext := xtText;
end;

{ TElementValidator }

function TElementValidator.IsElementAllowed(Def: TElementDecl): Boolean;
var
  Next: TContentParticle;
begin
  Result := True;
  // if element is not declared, non-validity has been already reported, no need to report again...
  if Assigned(Def) and Assigned(FElementDef) then
  begin
    case FElementDef.ContentType of

      ctEmpty: Result := False;

      ctChildren, ctMixed: begin
        if FFailed then     // if already detected a mismatch, don't waste time
          Exit;
        if FCurCP = nil then
          Next := FElementDef.RootCP.FindFirst(Def)
        else
          Next := FCurCP.FindNext(Def, 0); { second arg ignored here }
        Result := Assigned(Next);
        if Result then
          FCurCP := Next
        else
          FFailed := True;  // used to prevent extra error at the end of element
      end;
      // ctAny, ctUndeclared: returns True by default
    end;
  end;
end;

function TElementValidator.Incomplete: Boolean;
begin
  if Assigned(FElementDef) and (FElementDef.ContentType = ctChildren) and (not FFailed) then
  begin
    if FCurCP <> nil then
      Result := FCurCP.MoreRequired(0) { arg ignored here }
    else
      Result := FElementDef.RootCP.IsRequired;
  end
  else
    Result := False;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  if FSource.Matches('SYSTEM') then
  begin
    ExpectWhitespace;
    SkipQuotedLiteral(SysID);
    Result := True;
  end
  else if FSource.Matches('PUBLIC') then
  begin
    ExpectWhitespace;
    SkipPubidLiteral(PubID);
    NormalizeSpaces(PubID);
    if SysIdOptional then
      SkipWhitespace
    else
      ExpectWhitespace;
    SkipQuotedLiteral(SysID, not SysIdOptional);
    Result := True;
  end else
    Result := False;
end;

function TXMLReader.ValidateAttrSyntax(AttrDef: TDOMAttrDef; const aValue: WideString): Boolean;
begin
  case AttrDef.DataType of
    dtId, dtIdRef, dtEntity: Result := IsXmlName(aValue, FXML11) and
      ((not FNamespaces) or (Pos(WideChar(':'), aValue) = 0));
    dtIdRefs, dtEntities: Result := IsXmlNames(aValue, FXML11) and
      ((not FNamespaces) or (Pos(WideChar(':'), aValue) = 0));
    dtNmToken: Result := IsXmlNmToken(aValue, FXML11) and AttrDef.HasEnumToken(aValue);
    dtNmTokens: Result := IsXmlNmTokens(aValue, FXML11);
    // IsXmlName() not necessary - enum is never empty and contains valid names
    dtNotation: Result := AttrDef.HasEnumToken(aValue);
  else
    Result := True;
  end;
end;

procedure TXMLReader.ValidateAttrValue(Attr: TDOMAttr; const aValue: WideString);
var
  L, StartPos, EndPos: Integer;
  Entity: TDOMEntity;
begin
  L := Length(aValue);
  case Attr.DataType of
    dtId: if not Doc.AddID(Attr) then
            ValidationError('The ID ''%s'' is not unique', [aValue], -1);

    dtIdRef, dtIdRefs: begin
      StartPos := 1;
      while StartPos <= L do
      begin
        EndPos := StartPos;
        while (EndPos <= L) and (aValue[EndPos] <> #32) do
          Inc(EndPos);
        // pass negative length, so uses FTokenStart as location
        AddForwardRef(FIDRefs, @aValue[StartPos], StartPos-EndPos);
        StartPos := EndPos + 1;
      end;
    end;

    dtEntity, dtEntities: begin
      StartPos := 1;
      while StartPos <= L do
      begin
        EndPos := StartPos;
        while (EndPos <= L) and (aValue[EndPos] <> #32) do
          Inc(EndPos);
        Entity := TDOMEntity(FDocType.Entities.GetNamedItem(Copy(aValue, StartPos, EndPos-StartPos)));
        if (Entity = nil) or (Entity.NotationName = '') then
          ValidationError('Attribute ''%s'' type mismatch', [Attr.Name], -1);
        StartPos := EndPos + 1;
      end;
=======
procedure TXMLReader.DTDReloadHook;
begin
  BufAppendChunk(FIntSubset, FDTDStartPos, FSource.FBuf-FDTDStartPos);
  FDTDStartPos := TXMLDecodingSource(FSource).FBufStart + (FSource.FBufEnd-FSource.FBuf);
end;

procedure TXMLReader.ParseDoctypeDecl;    // [28]
var
  Src: TXMLCharSource;
begin
  if FState >= rsDTD then
    FatalError('Markup declaration is not allowed here');

  ExpectString('DOCTYPE');
  SkipS(True);

  FDocType := TDOMDocumentTypeEx(TDOMDocumentType.Create(doc));
  FState := rsDTD;
  try
    FDocType.FName := ExpectName;
    SkipS(True);
    ParseExternalID(FDocType.FSystemID, FDocType.FPublicID, False);
    SkipS;
  finally
    // DONE: append node after its name has been set; always append to avoid leak
    Doc.AppendChild(FDocType);
    FCursor := nil;
  end;

  if CheckForChar('[') then
  begin
    BufAllocate(FIntSubset, 256);
    FSource.DTDSubsetType := dsInternal;
    try
      FDTDStartPos := FSource.FBuf;
      ParseMarkupDecl;
      DTDReloadHook;     // fetch last chunk
      SetString(FDocType.FInternalSubset, FIntSubset.Buffer, FIntSubset.Length);
    finally
      FreeMem(FIntSubset.Buffer);
      FSource.DTDSubsetType := dsNone;
    end;
    ExpectChar(']');
    SkipS;
  end;
  ExpectChar('>');

  if (FDocType.SystemID <> '') then
  begin
    if ResolveEntity(FDocType.SystemID, FDocType.PublicID, Src) then
    begin
      ContextPush(Src);
      try
        Src.DTDSubsetType := dsExternal;
        ParseMarkupDecl;
      finally
        Src.DTDSubsetType := dsNone;
        ContextPop;
      end;
    end
    else
      ValidationError('Unable to resolve external DTD subset', []);
  end;
  FCursor := Doc;
  ValidateDTD;
  FDocType.SetReadOnly(True);
end;

procedure TXMLReader.ExpectEq;   // [25]
begin
  if FSource.FBuf^ <> '=' then
    SkipS;
  if FSource.FBuf^ <> '=' then
    FatalError('Expected "="');
  GetChar;
  SkipS;
end;


{ DTD stuff }

procedure TXMLReader.BadPENesting(S: TErrorSeverity);
begin
  if (S = esFatal) or FValidate then
    DoError(S, 'Parameter entities must be properly nested');
end;

procedure TXMLReader.StandaloneError(LineOffs: Integer);
begin
  ValidationError('Standalone constriant violation', [], LineOffs);
end;

procedure TXMLReader.ParseQuantity(CP: TContentParticle);
begin
  if CheckForChar('?') then
    CP.CPQuant := cqZeroOrOnce
  else if CheckForChar('*') then
    CP.CPQuant := cqZeroOrMore
  else if CheckForChar('+') then
    CP.CPQuant := cqOnceOrMore;
end;

function TXMLReader.FindOrCreateElDef: TDOMElementDef;
var
  Token: WideString;
begin
  Token := ExpectName;
  Result := TDOMElementDef(FDocType.ElementDefs.GetNamedItem(Token));
  if Result = nil then
  begin
    Result := TDOMElementDef.Create(doc);
    Result.FNodeName := Token;
    FDocType.ElementDefs.SetNamedItem(Result);
  end;
end;

procedure TXMLReader.ExpectChoiceOrSeq(CP: TContentParticle);                  // [49], [50]
var
  Delim: WideChar;
  CurrentEntity: TObject;
  CurrentCP: TContentParticle;
begin
  Delim := #0;
  repeat
    CurrentCP := CP.Add;
    SkipWhitespace;
    if CheckForChar('(') then
    begin
      CurrentEntity := FSource.FEntity;
      ExpectChoiceOrSeq(CurrentCP);
      if CurrentEntity <> FSource.FEntity then
        BadPENesting;
      GetChar;
    end
    else
      CurrentCP.Def := FindOrCreateElDef;

    ParseQuantity(CurrentCP);

    SkipWhitespace;
    if FCurChar = ')' then
      Break;
    if Delim = #0 then
    begin
      if (FCurChar = '|') or (FCurChar = ',') then
        Delim := FCurChar
      else
        FatalError('Expected pipe or comma delimiter');
    end
    else
      if FCurChar <> Delim then
        FatalError(Delim);
    GetChar; // skip delimiter
  until False;
  if Delim = '|' then
    CP.CPType := ctChoice
  else
    CP.CPType := ctSeq;    // '(foo)' is a sequence!
end;

procedure TXMLReader.ParseElementDecl;            // [45]
var
  ElDef: TDOMElementDef;
  CurrentEntity: TObject;
  I: Integer;
  CP: TContentParticle;
  Typ: TElementContentType;
  ExtDecl: Boolean;
begin
  CP := nil;
  Typ := ctAny;         // satisfy compiler
  ExpectWhitespace;
  ElDef := FindOrCreateElDef;
  if ElDef.HasElementDecl then
    ValidationError('Duplicate declaration of element ''%s''', [ElDef.TagName], FName.Length);

  ExtDecl := FSource.DTDSubsetType <> dsInternal;

  ExpectWhitespace;
  if FSource.Matches('EMPTY') then
    Typ := ctEmpty
  else if FSource.Matches('ANY') then
    Typ := ctAny
  else if CheckForChar('(') then
  begin
    CP := TContentParticle.Create;
    try
      CurrentEntity := FSource.FEntity;
      SkipWhitespace;
      if FSource.Matches('#PCDATA') then       // Mixed section [51]
      begin
        SkipWhitespace;
        Typ := ctMixed;
        while FCurChar <> ')' do
        begin
          ExpectChar('|');
          SkipWhitespace;

          with CP.Add do
          begin
            Def := FindOrCreateElDef;
            for I := CP.ChildCount-2 downto 0 do
              if Def = CP.Children[I].Def then
                ValidationError('Duplicate token in mixed section', [], FName.Length);
          end;
          SkipWhitespace;
        end;
        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        GetChar;
        if (not CheckForChar('*')) and (CP.ChildCount > 0) then
          FatalError(WideChar('*'));
      end
      else       // Children section [47]
      begin
        Typ := ctChildren;
        ExpectChoiceOrSeq(CP);
        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        GetChar;
        ParseQuantity(CP);
      end;
    except
      CP.Free;
      raise;
    end;
  end
  else
    FatalError('Invalid content specification');
  // SAX: DeclHandler.ElementDecl(name, model);
  if not ElDef.HasElementDecl then
  begin
    ElDef.HasElementDecl := True;
    ElDef.FExternallyDeclared := ExtDecl;
    ElDef.ContentType := Typ;
    ElDef.RootCP := CP;
  end
  else
    CP.Free;
end;


procedure TXMLReader.ParseNotationDecl;        // [82]
var
  Name, SysID, PubID: WideString;
begin
  ExpectWhitespace;
  Name := ExpectName;
  CheckNCName;
  ExpectWhitespace;
  if not ParseExternalID(SysID, PubID, True) then
    FatalError('Expected external or public ID');
  DoNotationDecl(Name, PubID, SysID);
end;

const
  AttrDataTypeNames: array[TAttrDataType] of WideString = (
    'CDATA',
    'ID',
    'IDREF',
    'IDREFS',
    'ENTITY',
    'ENTITIES',
    'NMTOKEN',
    'NMTOKENS',
    'NOTATION'
  );

procedure TXMLReader.ParseAttlistDecl;         // [52]
var
  ElDef: TDOMElementDef;
  AttDef: TDOMAttrDef;
  dt: TAttrDataType;
  Found, DiscardIt: Boolean;
begin
  ExpectWhitespace;
  ElDef := FindOrCreateElDef;
  SkipWhitespace;
  while FSource.FBuf^ <> '>' do
  begin
    CheckName;
    ExpectWhitespace;
    AttDef := TDOMAttrDef.Create(doc);
    try
      AttDef.FExternallyDeclared := FSource.DTDSubsetType <> dsInternal;
      SetString(AttDef.FName, FName.Buffer, FName.Length);
// In case of duplicate declaration of the same attribute, we must discard it,
// not modifying ElDef, and suppressing certain validation errors.
      DiscardIt := Assigned(ElDef.GetAttributeNode(AttDef.Name));
      if not DiscardIt then
        ElDef.SetAttributeNode(AttDef);

      if CheckForChar('(') then     // [59]
      begin
        AttDef.FDataType := dtNmToken;
        repeat
          SkipWhitespace;
          CheckName([cnToken]);
          if not AttDef.AddEnumToken(FName.Buffer, FName.Length) then
            ValidationError('Duplicate token in enumerated attibute declaration', [], FName.Length);
          SkipWhitespace;
        until not CheckForChar('|');
        ExpectChar(')');
        ExpectWhitespace;
      end
      else
      begin
        StoreLocation(FTokenStart);
        // search topside-up so that e.g. NMTOKENS is matched before NMTOKEN
        for dt := dtNotation downto dtCData do
        begin
          Found := FSource.Matches(AttrDataTypeNames[dt]);
          if Found then
            Break;
        end;
        if Found and SkipWhitespace then
        begin
          AttDef.FDataType := dt;
          if (dt = dtId) and not DiscardIt then
          begin
            if Assigned(ElDef.IDAttr) then
              ValidationError('Only one attribute of type ID is allowed per element',[])
            else
              ElDef.IDAttr := AttDef;
          end
          else if dt = dtNotation then          // no test cases for these ?!
          begin
            if not DiscardIt then
            begin
              if Assigned(ElDef.NotationAttr) then
                ValidationError('Only one attribute of type NOTATION is allowed per element',[])
              else
                ElDef.NotationAttr := AttDef;
              if ElDef.ContentType = ctEmpty then
                ValidationError('NOTATION attributes are not allowed on EMPTY elements',[]);
            end;
            ExpectChar('(');
            repeat
              SkipWhitespace;
              CheckName;
              CheckNCName;
              if not AttDef.AddEnumToken(FName.Buffer, FName.Length) then
                ValidationError('Duplicate token in NOTATION attribute declaration',[], FName.Length);

              if not DiscardIt then
                AddForwardRef(FNotationRefs, FName.Buffer, FName.Length);
              SkipWhitespace;
            until not CheckForChar('|');
            ExpectChar(')');
            ExpectWhitespace;
          end;
        end
        else if Found then
          ExpectWhitespace
        else
          FatalError('Illegal attribute type for ''%s''', [AttDef.Name]);
      end;
      StoreLocation(FTokenStart);
      if FSource.Matches('#REQUIRED') then
        AttDef.FDefault := adRequired
      else if FSource.Matches('#IMPLIED') then
        AttDef.FDefault := adImplied
      else if FSource.Matches('#FIXED') then
      begin
        AttDef.FDefault := adFixed;
        ExpectWhitespace;
      end
      else
        AttDef.FDefault := adDefault;

      if AttDef.FDefault in [adDefault, adFixed] then
      begin
        if AttDef.DataType = dtId then
          ValidationError('An attribute of type ID cannot have a default value',[]);

        FCursor := AttDef;
// See comments to valid-sa-094: PE expansion should be disabled in AttDef.
// ExpectAttValue() does not recognize PEs anyway, so setting FRecognizePEs isn't needed
// Saving/restoring FCursor is also redundant because it is always nil here.
        ExpectAttValue;
        FCursor := nil;
        if not ValidateAttrSyntax(AttDef, AttDef.NodeValue) then
          ValidationError('Default value for attribute ''%s'' has wrong syntax', [AttDef.Name]);
      end;
      // SAX: DeclHandler.AttributeDecl(...)
      if DiscardIt then
        AttDef.Free;
    except
      if AttDef.OwnerElement = nil then
        AttDef.Free;
      raise;
>>>>>>> origin/fixes_2_2
    end;
  end;
end;

<<<<<<< HEAD
procedure TXMLReader.ValidateRoot;
begin
  if Assigned(FDocType) then
  begin
    if not BufEquals(FName, FDocType.Name) then
      ValidationError('Root element name does not match DTD', [], FName.Length);
  end
  else
    ValidationError('Missing DTD', [], FName.Length);
=======
function TXMLReader.ParseEntityDeclValue(Delim: WideChar): Boolean;   // [9]
var
  CurrentEntity: TObject;
begin
  CurrentEntity := FSource.FEntity;
  if FEntityValue.Buffer = nil then
    BufAllocate(FEntityValue, 256);
  FEntityValue.Length := 0;
  // "Included in literal": process until delimiter hit IN SAME context
  while not ((FSource.FEntity = CurrentEntity) and CheckForChar(Delim)) do
  if CheckForChar('%') then
  begin
    CheckName;
    ExpectChar(';');
    if FSource.DTDSubsetType = dsInternal then
      FatalError('PE reference not allowed here in internal subset', FName.Length+2);
    StartPE;
  end
  else if FCurChar = '&' then  // CharRefs: include, EntityRefs: bypass
  begin
    if not ParseCharRef(FEntityValue) then
    begin
      BufAppend(FEntityValue, '&');
      BufAppendChunk(FEntityValue, FName.Buffer, FName.Length);
      BufAppend(FEntityValue, ';');
    end;
  end
  else if FCurChar <> #0 then         // Regular character
  begin
    BufAppend(FEntityValue, FCurChar);
    GetChar;
  end
  else if (FSource.FEntity = CurrentEntity) or not ContextPop then         // #0
  begin
    Result := False;
    Exit;
  end;
  Result := True;
>>>>>>> origin/fixes_2_2
end;

procedure TXMLReader.ValidateDTD;
var
  I: Integer;
begin
<<<<<<< HEAD
  if FValidate then
    for I := 0 to FNotationRefs.Count-1 do
      with PForwardRef(FNotationRefs[I])^ do
        if FDocType.Notations.GetNamedItem(Value) = nil then
          DoErrorPos(esError, Format('Notation ''%s'' is not declared', [Value]), Loc);
  ClearRefs(FNotationRefs);
end;

procedure TXMLReader.DoText(ch: PWideChar; Count: Integer; Whitespace: Boolean);
var
  TextNode: TDOMText;
begin
  // Validating filter part
  case FCurrContentType of
    ctChildren:
      if not Whitespace then
        ValidationError('Character data is not allowed in element-only content',[])
      else
        if FSaViolation then
          StandaloneError(-1);
    ctEmpty:
      ValidationError('Character data is not allowed in EMPTY elements', []);
=======
  if not SkipWhitespace(True) then
    FatalError('Expected whitespace');
  NDataAllowed := True;
  Map := FDocType.Entities;
  if CheckForChar('%') then                  // [72]
  begin
    ExpectWhitespace;
    NDataAllowed := False;
    if FPEMap = nil then
      FPEMap := TDOMNamedNodeMap.Create(FDocType, ENTITY_NODE);
    Map := FPEMap;
  end;

  Entity := TDOMEntityEx.Create(Doc);
  Entity.SetReadOnly(True);
  try
    Entity.FExternallyDeclared := FSource.DTDSubsetType <> dsInternal;
    Entity.FName := ExpectName;
    CheckNCName;
    ExpectWhitespace;

    if (FCurChar = '"') or (FCurChar = '''') then
    begin
      NDataAllowed := False;
      Delim := FCurChar;
      GetChar;
      StoreLocation(Entity.FStartLocation);
      if not ParseEntityDeclValue(Delim) then
        DoErrorPos(esFatal, 'Literal has no closing quote', Entity.FStartLocation);
      SetString(Entity.FReplacementText, FEntityValue.Buffer, FEntityValue.Length);
    end
    else
      if not ParseExternalID(Entity.FSystemID, Entity.FPublicID, False) then
        FatalError('Expected entity value or external ID');

    if NDataAllowed then                // [76]
    begin
      if FCurChar <> '>' then
        ExpectWhitespace;
      if FSource.Matches('NDATA') then
      begin
        ExpectWhitespace;
        Entity.FNotationName := ExpectName;
        AddForwardRef(FNotationRefs, FName.Buffer, FName.Length);
        // SAX: DTDHandler.UnparsedEntityDecl(...);
      end;
    end;
  except
    Entity.Free;
    raise;
>>>>>>> origin/fixes_2_2
  end;

  // Document builder part
  TextNode := Doc.CreateTextNodeBuf(ch, Count, Whitespace and (FCurrContentType = ctChildren));
  FCursor.AppendChild(TextNode);
end;

procedure TXMLReader.DoAttrText(ch: PWideChar; Count: Integer);
begin
  FCursor.AppendChild(Doc.CreateTextNodeBuf(ch, Count, False));
end;

procedure TXMLReader.DoComment(ch: PWideChar; Count: Integer);
var
<<<<<<< HEAD
  Node: TDOMComment;
begin
  // validation filter part
  if FCurrContentType = ctEmpty then
    ValidationError('Comments are not allowed within EMPTY elements', []);

  // DOM builder part
  if (not FIgnoreComments) and Assigned(FCursor) then
  begin
    Node := Doc.CreateCommentBuf(ch, Count);
    FCursor.AppendChild(Node);
  end;
end;

procedure TXMLReader.DoCDSect(ch: PWideChar; Count: Integer);
var
  s: WideString;
begin
  if FCurrContentType = ctChildren then
    ValidationError('CDATA sections are not allowed in element-only content',[]);
=======
  IncludeLevel: Integer;
  IgnoreLevel: Integer;
  CurrentEntity: TObject;
  IncludeLoc: TLocation;
  IgnoreLoc: TLocation;
  CondType: (ctUnknown, ctInclude, ctIgnore);
begin
  IncludeLevel := 0;
  IgnoreLevel := 0;
  repeat
    FRecognizePE := True;      // PERef between declarations should always be recognized
    SkipWhitespace;
    FRecognizePE := False;
>>>>>>> origin/fixes_2_2

  if not FCDSectionsAsText then
  begin
    SetString(s, ch, Count);
    // SAX: LexicalHandler.StartCDATA;
    // SAX: ContentHandler.Characters(...);
    FCursor.AppendChild(doc.CreateCDATASection(s));
    // SAX: LexicalHandler.EndCDATA;
  end
  else
    FCursor.AppendChild(doc.CreateTextNodeBuf(ch, Count, False));
end;

<<<<<<< HEAD
procedure TXMLReader.DoNotationDecl(const aName, aPubID, aSysID: WideString);
var
  Notation: TDOMNotationEx;
begin
  if FDocType.Notations.GetNamedItem(aName) = nil then
  begin
    Notation := TDOMNotationEx(TDOMNotation.Create(doc));
    Notation.FName := aName;
    Notation.FPublicID := aPubID;
    Notation.FSystemID := aSysID;
    FDocType.Notations.SetNamedItem(Notation);
  end
  else
    ValidationError('Duplicate notation declaration: ''%s''', [aName]);
end;

procedure TXMLReader.PushVC(aElDef: TDOMElementDef);
begin
  Inc(FNesting);
  if FNesting >= Length(FValidator) then
    SetLength(FValidator, FNesting * 2);
  unaligned(FValidator[FNesting].FElementDef) := aElDef;
  unaligned(FValidator[FNesting].FCurCP) := nil;
  unaligned(FValidator[FNesting].FFailed) := False;
  UpdateConstraints;
end;

procedure TXMLReader.PopVC;
begin
  if FNesting > 0 then Dec(FNesting);
  UpdateConstraints;
end;

procedure TXMLReader.UpdateConstraints;
begin
  if FValidate and Assigned(FValidator[FNesting].FElementDef) then
  begin
    FCurrContentType := FValidator[FNesting].FElementDef.ContentType;
    FSaViolation := FStandalone and (FValidator[FNesting].FElementDef.FExternallyDeclared);
  end
  else
  begin
    FCurrContentType := ctAny;
    FSaViolation := False;
  end;
end;

{ TDOMAttrDef }

function TDOMAttrDef.AddEnumToken(Buf: DOMPChar; Len: Integer): Boolean;
var
  I, L: Integer;
begin
  // TODO: this implementaion is the slowest possible...
  Result := False;
  L := Length(FEnumeration);
  for I := 0 to L-1 do
  begin
    if (Len = Length(FEnumeration[I])) and CompareMem(Buf, DOMPChar(FEnumeration[I]), Len*sizeof(WideChar)) then
      Exit;
  end;
  SetLength(FEnumeration, L+1);
  SetString(FEnumeration[L], Buf, Len);
  Result := True;
end;

function TDOMAttrDef.HasEnumToken(const aValue: WideString): Boolean;
var
  I: Integer;
begin
  Result := True;
  if Length(FEnumeration) = 0 then
    Exit;
  for I := 0 to Length(FEnumeration)-1 do
  begin
    if FEnumeration[I] = aValue then
      Exit;
  end;
  Result := False;
end;

type
  TDOMAttrEx = class(TDOMAttr);

function TDOMAttrDef.Clone(AElement: TDOMElement): TDOMAttr;
begin
  Result := TDOMAttr.Create(FOwnerDocument);
  TDOMAttrEx(Result).FName := Self.FName;
  TDOMAttrEx(Result).FDataType := FDataType;
  CloneChildren(Result, FOwnerDocument);
end;

{ TElementValidator }

function TElementValidator.IsElementAllowed(Def: TDOMElementDef): Boolean;
var
  I: Integer;
  Next: TContentParticle;
begin
  Result := True;
  // if element is not declared, non-validity has been already reported, no need to report again...
  if Assigned(Def) and Assigned(FElementDef) then
  begin
    case FElementDef.ContentType of
      ctMixed: begin
        for I := 0 to FElementDef.RootCP.ChildCount-1 do
        begin
          if Def = FElementDef.RootCP.Children[I].Def then
          Exit;
        end;
        Result := False;
      end;

      ctEmpty: Result := False;

      ctChildren: begin
        if FCurCP = nil then
          Next := FElementDef.RootCP.FindFirst(Def)
        else
          Next := FCurCP.FindNext(Def, 0); { second arg ignored here }
        Result := Assigned(Next);
        if Result then
          FCurCP := Next
        else
          FFailed := True;  // used to prevent extra error at the end of element
      end;
      // ctAny: returns True by default
    end;
  end;
end;

function TElementValidator.Incomplete: Boolean;
begin
  if Assigned(FElementDef) and (FElementDef.ContentType = ctChildren) and (not FFailed) then
  begin
    if FCurCP <> nil then
      Result := FCurCP.MoreRequired(0) { arg ignored here }
    else
      Result := FElementDef.RootCP.IsRequired;
  end
  else
    Result := False;
end;

{ TContentParticle }

function TContentParticle.Add: TContentParticle;
begin
  if FChildren = nil then
    FChildren := TFPList.Create;
  Result := TContentParticle.Create;
  Result.FParent := Self;
  Result.FIndex := FChildren.Add(Result);
end;

destructor TContentParticle.Destroy;
var
  I: Integer;
begin
  if Assigned(FChildren) then
    for I := FChildren.Count-1 downto 0 do
      TObject(FChildren[I]).Free;
  FChildren.Free;
  inherited Destroy;
end;

function TContentParticle.GetChild(Index: Integer): TContentParticle;
begin
  Result := TContentParticle(FChildren[Index]);
end;

function TContentParticle.GetChildCount: Integer;
begin
  if Assigned(FChildren) then
    Result := FChildren.Count
  else
    Result := 0;
end;
=======
    if not CheckForChar('<') then
      Break;

    CurrentEntity := FSource.FEntity;

    if FCurChar = '?' then
      ParsePI
    else
    begin
      ExpectChar('!');
      if FCurChar = '-' then
        ParseComment
      else if FCurChar = '[' then
      begin
        if FSource.DTDSubsetType = dsInternal then
          FatalError('Conditional sections are not allowed in internal subset');

        FRecognizePE := True;
        GetChar; // skip '['
        SkipWhitespace;

        CondType := ctUnknown;  // satisfy compiler
        if FSource.Matches('INCLUDE') then
          CondType := ctInclude
        else if FSource.Matches('IGNORE') then
          CondType := ctIgnore
        else
          FatalError('Expected "INCLUDE" or "IGNORE"');

        SkipWhitespace;
        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        ExpectChar('[');
        if CondType = ctInclude then
        begin
          if IncludeLevel = 0 then
            StoreLocation(IncludeLoc);
          Inc(IncludeLevel);
        end
        else if CondType = ctIgnore then
        begin
          StoreLocation(IgnoreLoc);
          IgnoreLevel := 1;
          repeat
            FRecognizePE := False;    // PEs not recognized in IGNORE section
            if CheckForChar('<') and CheckForChar('!') and CheckForChar('[') then
              Inc(IgnoreLevel)
            else if CheckForChar(']') and CheckForChar(']') and CheckForChar('>') then
              Dec(IgnoreLevel)
<<<<<<< HEAD
            else GetChar;
          until (IgnoreLevel=0) or (FCurChar = #0);
=======
            else if wc <> #0 then
              FSource.NextChar;
          until (IgnoreLevel=0) or (wc = #0);
>>>>>>> origin/fixes_2.4
// Since PE's are not recognized in ignore sections, reaching EOF is fatal.
          if FCurChar = #0 then
            Break;
        end;
      end
      else
      begin
        FRecognizePE := FSource.DTDSubsetType <> dsInternal;
        FInsideDecl := True;
        if FSource.Matches('ELEMENT') then
          ParseElementDecl
        else if FSource.Matches('ENTITY') then
          ParseEntityDecl
        else if FSource.Matches('ATTLIST') then
          ParseAttlistDecl
        else if FSource.Matches('NOTATION') then
          ParseNotationDecl
        else
          FatalError('Illegal markup declaration');

        SkipWhitespace;
        FRecognizePE := False;

        if CurrentEntity <> FSource.FEntity then
          BadPENesting;
        ExpectChar('>');
        FInsideDecl := False;
      end;
    end;
  until False;
  FRecognizePE := False;
  if (IncludeLevel > 0) or (IgnoreLevel > 0) then
  begin
    if IncludeLevel > 0 then
      FTokenStart := IncludeLoc
    else
      FTokenStart := IgnoreLoc;
    FatalError('Conditional section is not closed', -1);
  end;
  if (FSource.DTDSubsetType = dsInternal) and (FCurChar = ']') then
    Exit;
  if FCurChar <> #0 then
    FatalError('Illegal character in DTD');
end;

procedure TXMLReader.ProcessDTD(ASource: TXMLCharSource);
begin
  doc := TXMLDocument.Create;
  FDocType := TDOMDocumentTypeEx.Create(doc);
  // TODO: DTD labeled version 1.1 will be rejected - must set FXML11 flag
  // DONE: It's ok to have FCursor=nil now
  doc.AppendChild(FDocType);
  Initialize(ASource);
  ParseMarkupDecl;
end;

procedure TXMLReader.ParseCDSect;               // [18]
begin
  ExpectString('[CDATA[');
  StoreLocation(FTokenStart);
  if FState <> rsRoot then
    FatalError('Illegal at document level');
  FValue.Length := 0;
  repeat
    BufAppend(FValue, FCurChar);
    GetChar;
    with FValue do
      if (Length >= 3) and (Buffer[Length-1] = '>') and
      (Buffer[Length-2] = ']') and (Buffer[Length-3] = ']') then
    begin
      DoCDSect(Buffer, Length-3);
      Exit;
    end;
  until FCurChar = #0;
  FatalError('Unterminated CDATA section', -1);
end;

procedure TXMLReader.ParseContent;
var
  nonWs: Boolean;
begin
  repeat
    if FCurChar = '<' then
    begin
      GetChar;
      if FCurChar = '/' then  // end-tags are as frequent as start-tags
        Break;
      if CheckName([cnOptional]) then
        ParseElement
      else if FCurChar = '!' then
      begin
        GetChar;
        if FCurChar = '[' then
          ParseCDSect
        else if FCurChar = '-' then
          ParseComment
        else
          ParseDoctypeDecl;
      end
      else if FCurChar = '?' then
        ParsePI
      else
        RaiseNameNotFound;
    end
    else
    begin
      FValue.Length := 0;
      nonWs := False;
      StoreLocation(FTokenStart);
      while (FCurChar <> '<') and (FCurChar <> #0) do
      begin
        if FCurChar <> '&' then
        begin
          if (FCurChar <> #32) and (FCurChar <> #10) and (FCurChar <> #9) and (FCurChar <> #13) then
            nonWs := True;
          BufAppend(FValue, FCurChar);
          if FCurChar = '>' then
          with FValue do
            if (Length >= 3) and (Buffer[Length-2] = ']') and (Buffer[Length-3] = ']') then
              FatalError('Literal '']]>'' is not allowed in text', 2);
          GetChar;
        end
        else
        begin
          if FState <> rsRoot then
            FatalError('Illegal at document level');

          if FCurrContentType = ctEmpty then
            ValidationError('References are illegal in EMPTY elements', []);

          if ParseCharRef(FValue) or ResolvePredefined then
            nonWs := True // CharRef to whitespace is not considered whitespace
          else
          begin
            if (nonWs or FPreserveWhitespace) and (FValue.Length > 0)  then
            begin
              // 'Reference illegal at root' is checked above, no need to check here
              DoText(FValue.Buffer, FValue.Length, not nonWs);
              FValue.Length := 0;
            end;
            IncludeEntity(False);
          end;
        end;
      end; // while
      if FState = rsRoot then
      begin
        if (nonWs or FPreserveWhitespace) and (FValue.Length > 0)  then
        begin
          DoText(FValue.Buffer, FValue.Length, not nonWs);
          FValue.Length := 0;
        end;
      end
      else if nonWs then
        FatalError('Illegal at document level', -1);
    end;
  until FCurChar = #0;
end;

// Element name already in FNameBuffer
procedure TXMLReader.ParseElement;    // [39] [40] [44]
var
  NewElem: TDOMElement;
  ElDef: TDOMElementDef;
  IsEmpty: Boolean;
begin
  if FState > rsRoot then
    FatalError('Only one top-level element allowed', FName.Length)
  else if FState < rsRoot then
  begin
    if FValidate then
      ValidateRoot;
    FState := rsRoot;
  end;

  NewElem := doc.CreateElementBuf(FName.Buffer, FName.Length);
  FCursor.AppendChild(NewElem);
  // we're about to process a new set of attributes
  Inc(FAttrTag);

  // Find declaration for this element
  ElDef := nil;
  if Assigned(FDocType) then
  begin
    ElDef := TDOMElementDef(FDocType.ElementDefs.GetNamedItem(NewElem.TagName));
    if (ElDef = nil) or (not ElDef.HasElementDecl) then
      ValidationError('Using undeclared element ''%s''',[NewElem.TagName], FName.Length);
  end;

  // Check if new element is allowed in current context
  if FValidate and not FValidator[FNesting].IsElementAllowed(ElDef) then
    ValidationError('Element ''%s'' is not allowed in this context',[NewElem.TagName], FName.Length);

  IsEmpty := False;
  while (FSource.FBuf^ <> '>') and (FSource.FBuf^ <> '/') do
  begin
    SkipS(True);
    if (FSource.FBuf^ = '>') or (FSource.FBuf^ = '/') then
      Break;
    ParseAttribute(NewElem, ElDef);
  end;

  if FSource.FBuf^ = '/' then
  begin
    IsEmpty := True;
    GetChar;
  end;
  ExpectChar('>');

  ProcessDefaultAttributes(NewElem, ElDef);

  PushVC(ElDef);
  // SAX: ContentHandler.StartElement(...)
  // SAX: ContentHandler.StartPrefixMapping(...)
>>>>>>> origin/fixes_2_2

function TContentParticle.IsRequired: Boolean;
var
  I: Integer;
begin
  Result := (CPQuant = cqOnce) or (CPQuant = cqOnceOrMore);
  // do not return True if all children are optional
  if (CPType <> ctName) and Result then
  begin
<<<<<<< HEAD
    for I := 0 to ChildCount-1 do
    begin
      Result := Children[I].IsRequired;
      if Result then Exit;
    end;
=======
    FCursor := NewElem;
    if not FPreserveWhitespace then   // critical for testsuite compliance
      SkipS;
    ParseContent;
    if FCurChar = '/' then         // Get ETag [42]
    begin
      GetChar;
      StoreLocation(FTokenStart);
      CheckName;
      if not BufEquals(FName, NewElem.TagName) then
        FatalError('Unmatching element end tag (expected "</%s>")', [NewElem.TagName], FName.Length);
      SkipS;
      ExpectChar('>');
    end
    else if FCurChar <> #0 then
      RaiseNameNotFound
    else // End of stream in content
      FatalError('End-tag is missing for ''%s''', [NewElem.TagName]);
  end;
  // SAX: ContentHandler.EndElement(...)
  // SAX: ContentHandler.EndPrefixMapping(...)
  TDOMNode(FCursor) := NewElem.ParentNode;
  if FCursor = doc then
    FState := rsEpilog;

  if FValidate and FValidator[FNesting].Incomplete then
    ValidationError('Element ''%s'' is missing required sub-elements', [NewElem.TagName]);

  PopVC;
end;

procedure TXMLReader.ParseAttribute(Elem: TDOMElement; ElDef: TDOMElementDef);
var
  attr: TDOMAttr;
  AttDef: TDOMAttrDef;
  OldAttr: TDOMNode;

procedure CheckValue;
var
  AttValue, OldValue: WideString;
begin
  if FStandalone and AttDef.FExternallyDeclared then
  begin
    OldValue := Attr.Value;
    TDOMAttrDef(Attr).FDataType := AttDef.FDataType;
    AttValue := Attr.Value;
    if AttValue <> OldValue then
      StandaloneError(-1);
  end
  else
  begin
    TDOMAttrDef(Attr).FDataType := AttDef.FDataType;
    AttValue := Attr.Value;
>>>>>>> origin/fixes_2_2
  end;
  // TODO: what about normalization of AttDef.Value? (Currently it IS normalized)
  if (AttDef.FDefault = adFixed) and (AttDef.Value <> AttValue) then
    ValidationError('Value of attribute ''%s'' does not match its #FIXED default',[AttDef.Name], -1);
  if not ValidateAttrSyntax(AttDef, AttValue) then
    ValidationError('Attribute ''%s'' type mismatch', [AttDef.Name], -1);
  ValidateAttrValue(Attr, AttValue);
end;

begin
  CheckName;
  attr := doc.CreateAttributeBuf(FName.Buffer, FName.Length);

  if Assigned(ElDef) then
  begin
    AttDef := TDOMAttrDef(ElDef.GetAttributeNode(attr.Name));
    if AttDef = nil then
      ValidationError('Using undeclared attribute ''%s'' on element ''%s''',[attr.Name, Elem.TagName], FName.Length)
    else
      AttDef.Tag := FAttrTag;  // indicates that this one is specified
  end
  else
    AttDef := nil;

  // !!cannot use TDOMElement.SetAttributeNode because it will free old attribute
  OldAttr := Elem.Attributes.SetNamedItem(Attr);
  if Assigned(OldAttr) then
  begin
    OldAttr.Free;
    FatalError('Duplicate attribute', FName.Length);
  end;
  ExpectEq;
  FCursor := attr;
  ExpectAttValue;

  if Assigned(AttDef) and ((AttDef.FDataType <> dtCdata) or (AttDef.FDefault = adFixed)) then
    CheckValue;
end;

procedure TXMLReader.AddForwardRef(aList: TFPList; Buf: PWideChar; Length: Integer);
var
  w: PForwardRef;
begin
  New(w);
  SetString(w^.Value, Buf, Abs(Length));
  if Length > 0 then
  begin
    StoreLocation(w^.Loc);
    Dec(w^.Loc.LinePos, Length);
  end
  else
    w^.Loc := FTokenStart;
  aList.Add(w);
end;

<<<<<<< HEAD
function TContentParticle.MoreRequired(ChildIdx: Integer): Boolean;
var
  I: Integer;
begin
  Result := False;
  if CPType = ctSeq then
  begin
    for I := ChildIdx + 1 to ChildCount-1 do
    begin
      Result := Children[I].IsRequired;
      if Result then Exit;
=======
procedure TXMLReader.ClearRefs(aList: TFPList);
var
  I: Integer;
begin
  for I := 0 to aList.Count-1 do
    Dispose(PForwardRef(aList.List^[I]));
  aList.Clear;
end;

procedure TXMLReader.ValidateIdRefs;
var
  I: Integer;
begin
  for I := 0 to FIDRefs.Count-1 do
    with PForwardRef(FIDRefs.List^[I])^ do
      if Doc.GetElementById(Value) = nil then
        DoErrorPos(esError, Format('The ID ''%s'' does not match any element', [Value]), Loc);
  ClearRefs(FIDRefs);
end;

procedure TXMLReader.ProcessDefaultAttributes(Element: TDOMElement; ElDef: TDOMElementDef);
var
  Map: TDOMNamedNodeMap;
  Attr: TDOMAttr;

procedure DoDefaulting;
var
  I: Integer;
  AttDef: TDOMAttrDef;
begin
  Map := ElDef.FAttributes;

  for I := 0 to Map.Length-1 do
  begin
    AttDef := Map[I] as TDOMAttrDef;

    if AttDef.Tag <> FAttrTag then  // this one wasn't specified
    begin
      case AttDef.FDefault of
        adDefault, adFixed: begin
          if FStandalone and AttDef.FExternallyDeclared then
            StandaloneError;
          Attr := AttDef.Clone(Element);
          Element.SetAttributeNode(Attr);
          ValidateAttrValue(Attr, Attr.Value);
        end;
        adRequired:  ValidationError('Required attribute ''%s'' of element ''%s'' is missing',[AttDef.Name, Element.TagName], 0)
      end;
>>>>>>> origin/fixes_2_2
    end;
  end;
  if Assigned(FParent) then
    Result := FParent.MoreRequired(FIndex);
end;

<<<<<<< HEAD
function TContentParticle.FindFirst(aDef: TDOMElementDef): TContentParticle;
var
  I: Integer;
begin
  Result := nil;
  case CPType of
    ctSeq:
      for I := 0 to ChildCount-1 do with Children[I] do
      begin
        Result := FindFirst(aDef);
        if Assigned(Result) or IsRequired then
          Exit;
      end;
    ctChoice:
      for I := 0 to ChildCount-1 do with Children[I] do
      begin
        Result := FindFirst(aDef);
        if Assigned(Result) then
          Exit;
      end;
  else // ctName
    if aDef = Self.Def then
      Result := Self
  end;
=======
begin
  if Assigned(ElDef) and Assigned(ElDef.FAttributes) then
    DoDefaulting;
>>>>>>> origin/fixes_2_2
end;

function TContentParticle.FindNext(aDef: TDOMElementDef;
  ChildIdx: Integer): TContentParticle;
var
  I: Integer;
begin
<<<<<<< HEAD
  Result := nil;
  if CPType = ctSeq then   // search sequence to its end
  begin
    for I := ChildIdx + 1 to ChildCount-1 do with Children[I] do
    begin
      Result := FindFirst(aDef);
      if (Result <> nil) or IsRequired then
        Exit;
    end;
  end;
  if (CPQuant = cqZeroOrMore) or (CPQuant = cqOnceOrMore) then
    Result := FindFirst(aDef);
  if (Result = nil) and Assigned(FParent) then
    Result := FParent.FindNext(aDef, FIndex);
end;

{ TDOMElementDef }

destructor TDOMElementDef.Destroy;
begin
  RootCP.Free;
  inherited Destroy;
end;

>>>>>>> graemeg/fixes_2_2
=======
  if FSource.Matches('SYSTEM') then
  begin
    ExpectWhitespace;
    SkipQuotedLiteral(SysID);
    Result := True;
  end
  else if FSource.Matches('PUBLIC') then
  begin
    ExpectWhitespace;
    SkipPubidLiteral(PubID);
    NormalizeSpaces(PubID);
    if SysIdOptional then
      SkipWhitespace
    else
      ExpectWhitespace;
    SkipQuotedLiteral(SysID, not SysIdOptional);
    Result := True;
  end else
    Result := False;
end;

function TXMLReader.ValidateAttrSyntax(AttrDef: TDOMAttrDef; const aValue: WideString): Boolean;
begin
  case AttrDef.DataType of
    dtId, dtIdRef, dtEntity: Result := IsXmlName(aValue, FXML11) and
      ((not FNamespaces) or (Pos(WideChar(':'), aValue) = 0));
    dtIdRefs, dtEntities: Result := IsXmlNames(aValue, FXML11) and
      ((not FNamespaces) or (Pos(WideChar(':'), aValue) = 0));
    dtNmToken: Result := IsXmlNmToken(aValue, FXML11) and AttrDef.HasEnumToken(aValue);
    dtNmTokens: Result := IsXmlNmTokens(aValue, FXML11);
    // IsXmlName() not necessary - enum is never empty and contains valid names
    dtNotation: Result := AttrDef.HasEnumToken(aValue);
  else
    Result := True;
  end;
end;

procedure TXMLReader.ValidateAttrValue(Attr: TDOMAttr; const aValue: WideString);
var
  L, StartPos, EndPos: Integer;
  Entity: TDOMEntity;
begin
  L := Length(aValue);
  case Attr.DataType of
    dtId: if not Doc.AddID(Attr) then
            ValidationError('The ID ''%s'' is not unique', [aValue], -1);

    dtIdRef, dtIdRefs: begin
      StartPos := 1;
      while StartPos <= L do
      begin
        EndPos := StartPos;
        while (EndPos <= L) and (aValue[EndPos] <> #32) do
          Inc(EndPos);
        // pass negative length, so uses FTokenStart as location
        AddForwardRef(FIDRefs, @aValue[StartPos], StartPos-EndPos);
        StartPos := EndPos + 1;
      end;
    end;

    dtEntity, dtEntities: begin
      StartPos := 1;
      while StartPos <= L do
      begin
        EndPos := StartPos;
        while (EndPos <= L) and (aValue[EndPos] <> #32) do
          Inc(EndPos);
        Entity := TDOMEntity(FDocType.Entities.GetNamedItem(Copy(aValue, StartPos, EndPos-StartPos)));
        if (Entity = nil) or (Entity.NotationName = '') then
          ValidationError('Attribute ''%s'' type mismatch', [Attr.Name], -1);
        StartPos := EndPos + 1;
      end;
    end;
  end;
end;

procedure TXMLReader.ValidateRoot;
begin
  if Assigned(FDocType) then
  begin
    if not BufEquals(FName, FDocType.Name) then
      ValidationError('Root element name does not match DTD', [], FName.Length);
  end
  else
    ValidationError('Missing DTD', [], FName.Length);
end;

procedure TXMLReader.ValidateDTD;
var
  I: Integer;
begin
  if FValidate then
    for I := 0 to FNotationRefs.Count-1 do
      with PForwardRef(FNotationRefs[I])^ do
        if FDocType.Notations.GetNamedItem(Value) = nil then
          DoErrorPos(esError, Format('Notation ''%s'' is not declared', [Value]), Loc);
  ClearRefs(FNotationRefs);
end;

procedure TXMLReader.DoText(ch: PWideChar; Count: Integer; Whitespace: Boolean);
var
  TextNode: TDOMText;
begin
  // Validating filter part
  case FCurrContentType of
    ctChildren:
      if not Whitespace then
        ValidationError('Character data is not allowed in element-only content',[])
      else
        if FSaViolation then
          StandaloneError(-1);
    ctEmpty:
      ValidationError('Character data is not allowed in EMPTY elements', []);
  end;

  // Document builder part
  TextNode := Doc.CreateTextNodeBuf(ch, Count, Whitespace and (FCurrContentType = ctChildren));
  FCursor.AppendChild(TextNode);
end;

procedure TXMLReader.DoAttrText(ch: PWideChar; Count: Integer);
begin
  FCursor.AppendChild(Doc.CreateTextNodeBuf(ch, Count, False));
end;

procedure TXMLReader.DoComment(ch: PWideChar; Count: Integer);
var
  Node: TDOMComment;
begin
  // validation filter part
  if FCurrContentType = ctEmpty then
    ValidationError('Comments are not allowed within EMPTY elements', []);

  // DOM builder part
  if (not FIgnoreComments) and Assigned(FCursor) then
  begin
    Node := Doc.CreateCommentBuf(ch, Count);
    FCursor.AppendChild(Node);
  end;
end;

procedure TXMLReader.DoCDSect(ch: PWideChar; Count: Integer);
var
  s: WideString;
begin
  if FCurrContentType = ctChildren then
    ValidationError('CDATA sections are not allowed in element-only content',[]);

  if not FCDSectionsAsText then
  begin
    SetString(s, ch, Count);
    // SAX: LexicalHandler.StartCDATA;
    // SAX: ContentHandler.Characters(...);
    FCursor.AppendChild(doc.CreateCDATASection(s));
    // SAX: LexicalHandler.EndCDATA;
  end
  else
    FCursor.AppendChild(doc.CreateTextNodeBuf(ch, Count, False));
end;

procedure TXMLReader.DoNotationDecl(const aName, aPubID, aSysID: WideString);
var
  Notation: TDOMNotationEx;
begin
  if FDocType.Notations.GetNamedItem(aName) = nil then
  begin
    Notation := TDOMNotationEx(TDOMNotation.Create(doc));
    Notation.FName := aName;
    Notation.FPublicID := aPubID;
    Notation.FSystemID := aSysID;
    FDocType.Notations.SetNamedItem(Notation);
  end
  else
    ValidationError('Duplicate notation declaration: ''%s''', [aName]);
end;

procedure TXMLReader.PushVC(aElDef: TDOMElementDef);
begin
  Inc(FNesting);
  if FNesting >= Length(FValidator) then
    SetLength(FValidator, FNesting * 2);
  unaligned(FValidator[FNesting].FElementDef) := aElDef;
  unaligned(FValidator[FNesting].FCurCP) := nil;
  unaligned(FValidator[FNesting].FFailed) := False;
  UpdateConstraints;
end;

procedure TXMLReader.PopVC;
begin
  if FNesting > 0 then Dec(FNesting);
  UpdateConstraints;
end;

procedure TXMLReader.UpdateConstraints;
begin
  if FValidate and Assigned(FValidator[FNesting].FElementDef) then
  begin
    FCurrContentType := FValidator[FNesting].FElementDef.ContentType;
    FSaViolation := FStandalone and (FValidator[FNesting].FElementDef.FExternallyDeclared);
  end
  else
  begin
    FCurrContentType := ctAny;
    FSaViolation := False;
  end;
end;

{ TDOMAttrDef }

function TDOMAttrDef.AddEnumToken(Buf: DOMPChar; Len: Integer): Boolean;
var
  I, L: Integer;
begin
  // TODO: this implementaion is the slowest possible...
  Result := False;
  L := Length(FEnumeration);
  for I := 0 to L-1 do
  begin
    if (Len = Length(FEnumeration[I])) and CompareMem(Buf, DOMPChar(FEnumeration[I]), Len*sizeof(WideChar)) then
      Exit;
  end;
  SetLength(FEnumeration, L+1);
  SetString(FEnumeration[L], Buf, Len);
  Result := True;
end;

function TDOMAttrDef.HasEnumToken(const aValue: WideString): Boolean;
var
  I: Integer;
begin
  Result := True;
  if Length(FEnumeration) = 0 then
    Exit;
  for I := 0 to Length(FEnumeration)-1 do
  begin
    if FEnumeration[I] = aValue then
      Exit;
  end;
  Result := False;
end;

type
  TDOMAttrEx = class(TDOMAttr);

function TDOMAttrDef.Clone(AElement: TDOMElement): TDOMAttr;
begin
  Result := TDOMAttr.Create(FOwnerDocument);
  TDOMAttrEx(Result).FName := Self.FName;
  TDOMAttrEx(Result).FDataType := FDataType;
  CloneChildren(Result, FOwnerDocument);
end;

{ TElementValidator }

function TElementValidator.IsElementAllowed(Def: TDOMElementDef): Boolean;
var
  I: Integer;
  Next: TContentParticle;
begin
  Result := True;
  // if element is not declared, non-validity has been already reported, no need to report again...
  if Assigned(Def) and Assigned(FElementDef) then
  begin
    case FElementDef.ContentType of
      ctMixed: begin
        for I := 0 to FElementDef.RootCP.ChildCount-1 do
        begin
          if Def = FElementDef.RootCP.Children[I].Def then
          Exit;
        end;
        Result := False;
      end;

      ctEmpty: Result := False;

      ctChildren: begin
        if FCurCP = nil then
          Next := FElementDef.RootCP.FindFirst(Def)
        else
          Next := FCurCP.FindNext(Def, 0); { second arg ignored here }
        Result := Assigned(Next);
        if Result then
          FCurCP := Next
        else
          FFailed := True;  // used to prevent extra error at the end of element
      end;
      // ctAny: returns True by default
    end;
  end;
end;

function TElementValidator.Incomplete: Boolean;
begin
  if Assigned(FElementDef) and (FElementDef.ContentType = ctChildren) and (not FFailed) then
  begin
    if FCurCP <> nil then
      Result := FCurCP.MoreRequired(0) { arg ignored here }
    else
      Result := FElementDef.RootCP.IsRequired;
  end
  else
    Result := False;
end;

{ TContentParticle }

function TContentParticle.Add: TContentParticle;
begin
  if FChildren = nil then
    FChildren := TFPList.Create;
  Result := TContentParticle.Create;
  Result.FParent := Self;
  Result.FIndex := FChildren.Add(Result);
end;

destructor TContentParticle.Destroy;
var
  I: Integer;
begin
  if Assigned(FChildren) then
    for I := FChildren.Count-1 downto 0 do
      TObject(FChildren[I]).Free;
  FChildren.Free;
  inherited Destroy;
end;

function TContentParticle.GetChild(Index: Integer): TContentParticle;
begin
  Result := TContentParticle(FChildren[Index]);
end;

function TContentParticle.GetChildCount: Integer;
begin
  if Assigned(FChildren) then
    Result := FChildren.Count
  else
    Result := 0;
end;

function TContentParticle.IsRequired: Boolean;
var
  I: Integer;
begin
  Result := (CPQuant = cqOnce) or (CPQuant = cqOnceOrMore);
  // do not return True if all children are optional
  if (CPType <> ctName) and Result then
  begin
    for I := 0 to ChildCount-1 do
    begin
      Result := Children[I].IsRequired;
      if Result then Exit;
    end;
  end;
end;

function TContentParticle.MoreRequired(ChildIdx: Integer): Boolean;
var
  I: Integer;
begin
  Result := False;
  if CPType = ctSeq then
  begin
    for I := ChildIdx + 1 to ChildCount-1 do
    begin
      Result := Children[I].IsRequired;
      if Result then Exit;
    end;
  end;
  if Assigned(FParent) then
    Result := FParent.MoreRequired(FIndex);
end;

function TContentParticle.FindFirst(aDef: TDOMElementDef): TContentParticle;
var
  I: Integer;
begin
  Result := nil;
  case CPType of
    ctSeq:
      for I := 0 to ChildCount-1 do with Children[I] do
      begin
        Result := FindFirst(aDef);
        if Assigned(Result) or IsRequired then
          Exit;
      end;
    ctChoice:
      for I := 0 to ChildCount-1 do with Children[I] do
      begin
        Result := FindFirst(aDef);
        if Assigned(Result) then
          Exit;
      end;
  else // ctName
    if aDef = Self.Def then
      Result := Self
  end;
end;

function TContentParticle.FindNext(aDef: TDOMElementDef;
  ChildIdx: Integer): TContentParticle;
var
  I: Integer;
begin
  Result := nil;
  if CPType = ctSeq then   // search sequence to its end
  begin
    for I := ChildIdx + 1 to ChildCount-1 do with Children[I] do
    begin
      Result := FindFirst(aDef);
      if (Result <> nil) or IsRequired then
        Exit;
    end;
  end;
  if (CPQuant = cqZeroOrMore) or (CPQuant = cqOnceOrMore) then
    Result := FindFirst(aDef);
  if (Result = nil) and Assigned(FParent) then
    Result := FParent.FindNext(aDef, FIndex);
end;

{ TDOMElementDef }

destructor TDOMElementDef.Destroy;
begin
  RootCP.Free;
  inherited Destroy;
end;

>>>>>>> origin/fixes_2_2
{ plain calls }

procedure ReadXMLFile(out ADoc: TXMLDocument; var f: Text);
var
<<<<<<< HEAD
  Reader: TXMLTextReader;
  ldr: TLoader;
=======
  Reader: TXMLReader;
  Src: TXMLCharSource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  ADoc := TXMLDocument.Create;
  Reader := TXMLTextReader.Create(f, ADoc.Names);
  try
    ldr.ProcessXML(ADoc,Reader);
  finally
    Reader.Free;
  end;
end;

procedure ReadXMLFile(out ADoc: TXMLDocument; f: TStream; const ABaseURI: String);
var
<<<<<<< HEAD
  Reader: TXMLTextReader;
  ldr: TLoader;
=======
  Reader: TXMLReader;
  Src: TXMLCharSource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  ADoc := TXMLDocument.Create;
  Reader := TXMLTextReader.Create(f, ABaseURI, ADoc.Names);
  try
    ldr.ProcessXML(ADoc, Reader);
  finally
    Reader.Free;
  end;
end;

procedure ReadXMLFile(out ADoc: TXMLDocument; f: TStream);
begin
  ReadXMLFile(ADoc, f, 'stream:');
end;

procedure ReadXMLFile(out ADoc: TXMLDocument; const AFilename: String);
var
  FileStream: TStream;
begin
  ADoc := nil;
  FileStream := TFileStream.Create(AFilename, fmOpenRead+fmShareDenyWrite);
  try
    ReadXMLFile(ADoc, FileStream, FilenameToURI(AFilename));
  finally
    FileStream.Free;
  end;
end;

procedure ReadXMLFragment(AParentNode: TDOMNode; var f: Text);
var
<<<<<<< HEAD
  Reader: TXMLTextReader;
  ldr: TLoader;
=======
  Reader: TXMLReader;
  Src: TXMLCharSource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Reader := TXMLTextReader.Create(f, AParentNode.OwnerDocument.Names);
  try
    ldr.ProcessFragment(AParentNode, Reader);
  finally
    Reader.Free;
  end;
end;

procedure ReadXMLFragment(AParentNode: TDOMNode; f: TStream; const ABaseURI: String);
var
<<<<<<< HEAD
  Reader: TXMLTextReader;
  ldr: TLoader;
=======
  Reader: TXMLReader;
  Src: TXMLCharSource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  Reader := TXMLTextReader.Create(f, ABaseURI, AParentNode.OwnerDocument.Names);
  try
    ldr.ProcessFragment(AParentNode, Reader);
  finally
    Reader.Free;
  end;
end;

procedure ReadXMLFragment(AParentNode: TDOMNode; f: TStream);
begin
  ReadXMLFragment(AParentNode, f, 'stream:');
end;

procedure ReadXMLFragment(AParentNode: TDOMNode; const AFilename: String);
var
  Stream: TStream;
begin
  Stream := TFileStream.Create(AFilename, fmOpenRead+fmShareDenyWrite);
  try
    ReadXMLFragment(AParentNode, Stream, FilenameToURI(AFilename));
  finally
    Stream.Free;
  end;
end;


procedure ReadDTDFile(out ADoc: TXMLDocument; var f: Text);
var
<<<<<<< HEAD
  Reader: TXMLTextReader;
  ldr: TLoader;
=======
  Reader: TXMLReader;
  Src: TXMLCharSource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  ADoc := TXMLDocument.Create;
  Reader := TXMLTextReader.Create(f, ADoc.Names);
  try
    ldr.ProcessDTD(ADoc,Reader);
  finally
    Reader.Free;
  end;
end;

procedure ReadDTDFile(out ADoc: TXMLDocument; f: TStream; const ABaseURI: String);
var
<<<<<<< HEAD
  Reader: TXMLTextReader;
  ldr: TLoader;
=======
  Reader: TXMLReader;
  Src: TXMLCharSource;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  ADoc := TXMLDocument.Create;
  Reader := TXMLTextReader.Create(f, ABaseURI, ADoc.Names);
  try
    ldr.ProcessDTD(ADoc,Reader);
  finally
    Reader.Free;
  end;
end;

procedure ReadDTDFile(out ADoc: TXMLDocument; f: TStream);
begin
  ReadDTDFile(ADoc, f, 'stream:');
end;

procedure ReadDTDFile(out ADoc: TXMLDocument; const AFilename: String);
var
  Stream: TStream;
begin
  ADoc := nil;
  Stream := TFileStream.Create(AFilename, fmOpenRead+fmShareDenyWrite);
  try
    ReadDTDFile(ADoc, Stream, FilenameToURI(AFilename));
  finally
    Stream.Free;
  end;
end;




end.
