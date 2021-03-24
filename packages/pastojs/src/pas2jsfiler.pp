{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018  Mattias Gaertner  mattias@freepascal.org

    Pascal to Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

Abstract:
  Write and read a precompiled module (pcu, gzipped json).

- Built-In symbols are collected in one array.
- symbols of this module are stored in a tree
- external references are stored in used module trees. They can refer
  recursively to other external references, so they are collected in a Queue.

Works:
- store used source files and checksums
- store compiler flags
- restore module as json
- restore types
- references to built in symbols via Id
- references to module's TPasElement via Id
- resolving forward references
- restore resolver scopes
- restore resolved references and access flags
- useanalyzer: use restored proc references
- write+read compiled proc body
- converter: use precompiled body
- store/restore/use precompiled JS of proc bodies
- store/restore/use precompiled JS of proc local const
- store/restore/use precompiled JS of initialization plus references
- useanalyzer: generate + use initialization/finalization references
- uses section
- indirect used units
- external references
- stop after uses section and continue reading
- WPO uses Proc.References
- gzipped json
- write final switches
- srcmaps for precompiled js
- generics:
  - generic proc bodies are stored with all elements, but without resolver customdata
  - specializations are stored like external elements
  - references of specialized types and their elements:
    same as external references
- specialize:
  - TPCUWriter.WriteSpecializeType: write aliastype+Params+SpecName(Name)
  - TPCUWriter.WriteInlineSpecializeExpr: Name(=NameExpr)+Params
  - TPCUWriter.IsExternalEl: true for specialized elements
  - TPCUWriter.WriteExtRefSignature.WriteMemberIndex
    - for specialized elements: writes 'SpecParams' array with Params
  - TPCUWriter.WriteExternalReference
    - add specializations to 'Specs' array of generic type,
      Note that the generic type can be internal or external

  - TPCUReader.AddPendingSpecialize
  - TPCUReader.Set_SpecializeParam
    - called when a Param of a spezialization was resolved,
    - can trigger Resolver.GetSpecializedEl and ReadExternalReferences
  - TPCUReader.ReadExternalSpecialized
    -
  - TPCUReader.ReadSpecializeType reads a TPasSpecializeType and creates specialized type
  - TPCUReader.ReadInlineSpecializeExpr: create specialized type

Todo:
- store used GUIDs
- distinguish reader errors in fatal and error
- when pcu is bad, unload and use src
- replace GUID with crc
}
unit Pas2JsFiler;

{$mode objfpc}{$H+}

{$IF FPC_FULLVERSION>30200}
{$WARN 6060 off : case statement does not handle all possible cases}
{$ENDIF}

interface

uses
  Classes, Types, SysUtils, contnrs,
  {$ifdef pas2js}
  {$else}
  zstream, AVL_Tree,
  {$endif}
  fpjson, jsonparser, jsonscanner,
  PasTree, PScanner, PParser, PasResolveEval, PasResolver,
  Pas2jsFileUtils, FPPas2Js, jsbase;

const
  PCUMagic = 'Pas2JSCache';
  PCUVersion = 7;
  { Version Changes:
    1: initial version
    2: - TPasProperty.ImplementsFunc:String -> Implements:TPasExprArray
       - pcsfAncestorResolved
       - removed msIgnoreInterfaces
    3: changed records from function to objects (pas2js 1.3)
    4: precompiled JS of initialization section now only contains the statements,
       not the whole $init function (pas2js 1.5)
    5: removed modeswitch ignoreattributes
    6: default DispatchField=Msg, DispatchStrField=MsgStr
    7: InitializationSection JS replaced with Body, Empty
  }

  BuiltInNodeName = 'BuiltIn';

  PCUDefaultParserOptions: TPOptions = po_Pas2js;

  PCUBoolStr: array[boolean] of string = (
    'False',
    'True'
    );

  PCUParserOptionNames: array[TPOption] of string = (
    'delphi',
    'KeepScannerError',
    'CAssignments',
    'ResolveStandardTypes',
    'AsmWhole',
    'NoOverloadedProcs',
    'KeepClassForward',
    'ArrayRangeExpr',
    'SelfToken',
    'CheckModeSwitches',
    'CheckCondFunction',
    'StopOnErrorDirective',
    'ExtClassConstWithoutExpr',
    'StopOnUnitInterface',
    'IgnoreUnknownResource',
    'AsyncProcs',
    'DisableResources');

  PCUDefaultModeSwitches: TModeSwitches = [
    msObjfpc,
    msClass,
    msResult,
    msNestedComment,
    msRepeatForward,
    msInitFinal,
    msOut,
    msDefaultPara,
    msHintDirective,
    msProperty,
    msExcept,
    msDefaultUnicodestring,
    msCBlocks];

  PCUModeSwitchNames: array[TModeSwitch] of string = (
    'None',
    'Fpc',
    'Objfpc',
    'Delphi',
    'DelphiUnicode',
    'TP7',
    'Mac',
    'Iso',
    'Extpas',
    'GPC',
    'Class',
    'Objpas',
    'Result',
    'StringPchar',
    'CVarSupport',
    'NestedComment',
    'TPProcVar',
    'MacProcVar',
    'RepeatForward',
    'Pointer2Procedure',
    'AutoDeref',
    'InitFinal',
    'DefaultAnsistring',
    'Out',
    'DefaultPara',
    'HintDirective',
    'DuplicateNames',
    'Property',
    'DefaultInline',
    'Except',
    'ObjectiveC1',
    'ObjectiveC2',
    'NestedProcVars',
    'NonLocalGoto',
    'AdvancedRecords',
    'ISOLikeUnaryMinus',
    'SystemCodePage',
    'FinalFields',
    'DefaultUnicodestring',
    'TypeHelpers',
    'CBlocks',
    'ISOLikeIO',
    'ISOLikeProgramsPara',
    'ISOLikeMod',
    'ArrayOperators',
    'ExternalClass',
    'PrefixedAttributes',
    'OmitRTTI',
    'MultiHelpers',
    'ImplicitFunctionSpecialization',
    'MultilineStrings'
    ); // Dont forget to update ModeSwitchToInt !

  PCUDefaultBoolSwitches: TBoolSwitches = [
    bsHints,
    bsNotes,
    bsWarnings
    ];
  PCUBoolSwitchNames: array[TBoolSwitch] of string = (
    'None',
    'Align',
    'BoolEval',
    'Assertions',
    'DebugInfo',
    'Extension',
    'ImportedData',
    'LongStrings',
    'IOChecks',
    'WriteableConst',
    'LocalSymbols',
    'TypeInfo',
    'Optimization',
    'OpenStrings',
    'OverflowChecks',
    'RangeChecks',
    'TypedAddress',
    'SafeDivide',
    'VarStringChecks',
    'Stackframes',
    'ExtendedSyntax',
    'ReferenceInfo',
    'Hints',
    'Notes',
    'Warnings',
    'Macro',
    'ScopedEnums',
    'ObjectChecks',
    'PointerMath',
    'Goto'
    );

  PCUMinConverterOptions = [coStoreImplJS,coShortRefGlobals];
  PCUDefaultConverterOptions: TPasToJsConverterOptions =
    PCUMinConverterOptions+[coUseStrict];
  PCUConverterOptions: array[TPasToJsConverterOption] of string = (
    'LowerCase',
    'SwitchStatement',
    'EnumNumbers',
    'UseStrict',
    'NoTypeInfo',
    'EliminateDeadCode',
    'StoreImplJS',
    'RTLVersionCheckMain',
    'RTLVersionCheckSystem',
    'RTLVersionCheckUnit',
    'ShortRefGlobals'
    );

  PCUDefaultTargetPlatform = PlatformBrowser;
  PCUTargetPlatformNames: array[TPasToJsPlatform] of string = (
    'Browser',
    'NodeJS',
    'Electron'
    );

  PCUDefaultTargetProcessor = ProcessorECMAScript5;
  PCUTargetProcessorNames: array[TPasToJsProcessor] of string = (
    'ECMAScript5',
    'ECMAScript6'
    );

  PCUMemberVisibilityNames: array[TPasMemberVisibility] of string = (
    'Default',
    'Private',
    'Protected',
    'Public',
    'Published',
    'Automated',
    'StrictPrivate',
    'StrictProtected',
    'Required',
    'Optional'
    );

  PCUMemberHintNames: array[TPasMemberHint] of string = (
    'Deprecated',
    'Library',
    'Platform',
    'Experimental',
    'Unimplemented'
    );

  PCUDefaultModuleScopeFlags = [pmsfRangeErrorSearched];
  PCUModuleScopeFlagNames: array[TPasModuleScopeFlag] of string = (
    'AssertSearched',
    'RangeErrorNeeded',
    'RangeErrorSearched'
    ) ;

  PCUDefaultIdentifierKind = pikSimple;
  PCUIdentifierKindNames: array[TPasIdentifierKind] of string = (
    'None',
    'BaseType',
    'BuiltInProc',
    'Simple',
    'Proc',
    'Namespace'
    );

  PCUVarModifierNames: array[TVariableModifier] of string = (
    'CVar',
    'External',
    'Public',
    'Export',
    'Class',
    'Static'
    );

  PCUDefaultExprKind = pekIdent;
  PCUExprKindNames: array[TPasExprKind] of string = (
    'Ident',
    'Number',
    'String',
    'Set',
    'Nil',
    'Bool',
    'Range',
    'Unary',
    'Binary',
    'Func',
    'Array',
    'List',
    'Inherited',
    'Self',
    'Specialize',
    'Procedure');

  PCUExprOpCodeNames: array[TExprOpCode] of string = (
    'None',
    'Add',
    'Sub',
    'Mul',
    'DivF',
    'DivI',
    'Mod',
    'Pow',
    'Shr',
    'Shl',
    'Not',
    'And',
    'Or',
    'Xor',
    'Eq',
    'NE',
    'LT',
    'GT',
    'LTE',
    'GTE',
    'In',
    'Is',
    'As',
    'SymDif',
    'Addr',
    'Deref',
    'MemAddr',
    'SubId'
    );

  PCUPackModeNames: array[TPackMode] of string = (
    'None',
    'Packed',
    'BitPacked'
    );

  PCURESetElKindNames : array[TRESetElKind] of string = (
    'None',
    'Enum',
    'Int',
    'Char',
    'Bool'
    );

  PCUObjKindNames: array[TPasObjKind] of string = (
    'Object',
    'Class',
    'Interface',
    'ClassHelper',
    'RecordHelper',
    'TypeHelper',
    'DispInterface',
    'ObjcClass',
    'ObjcCategory',
    'ObjcProtocol'
    );

  PCUClassInterfaceTypeNames: array[TPasClassInterfaceType] of string = (
    'COM',
    'CORBA'
    );

  PCUClassScopeFlagNames: array[TPasClassScopeFlag] of string = (
    'AncestorResolved',
    'Sealed',
    'Published'
    );

  PCUDispatchDefaultField = 'Msg';
  PCUDispatchDefaultStrField = 'MsgStr';

  PCUArgumentAccessNames: array[TArgumentAccess] of string = (
    'Default',
    'Const',
    'Var',
    'Out',
    'ConstRef'
    );

  PCUCallingConventionNames: array[TCallingConvention] of string = (
    'Default',
    'Register',
    'Pascal',
    'CDecl',
    'StdCall',
    'OldFPCCall',
    'SafeCall',
    'SysCall',
    'MWPascal',
    'HardFloat',
    'SysV_ABI_Default',
    'SysV_ABI_CDecl',
    'MS_ABI_Default',
    'MS_ABI_CDecl',
    'VectorCall'
    );

  PCUProcTypeModifierNames: array[TProcTypeModifier] of string = (
    'OfObject',
    'IsNested',
    'Static',
    'Varargs',
    'ReferenceTo',
    'Async',
    'Far',
    'CBlock'
    );

  PCUProcedureMessageTypeNames: array[TProcedureMessageType] of string = (
    'None',
    'Integer',
    'String'
    );

  PCUOperatorTypeNames: array[TOperatorType] of string = (
    'Unknown',
    'Implicit',
    'Explicit',
    'Mul',
    'Plus',
    'Minus',
    'Division',
    'LessThan',
    'Equal',
    'GreaterThan',
    'Assign',
    'NotEqual',
    'LessEqualThan',
    'GreaterEqualThan',
    'Power',
    'SymmetricalDifference',
    'Inc',
    'Dec',
    'Mod',
    'Negative',
    'Positive',
    'BitWiseOr',
    'Div',
    'LeftShift',
    'LogicalOr',
    'BitwiseAnd',
    'bitwiseXor',
    'LogicalAnd',
    'LogicalNot',
    'LogicalXor',
    'RightShift',
    'Enumerator',
    'In',
    'Initialize'
    );

  PCUProcedureModifierNames: array[TProcedureModifier] of string = (
    'Virtual',
    'Dynamic',
    'Abstract',
    'Override',
    'Export',
    'Overload',
    'Message',
    'Reintroduce',
    'Inline',
    'Assembler',
    'Public',
    'CompilerProc',
    'External',
    'Forward',
    'DispId',
    'NoReturn',
    'Far',
    'Final',
    'DiscardResult'
    );
  PCUProcedureModifiersImplProc = [pmInline,pmAssembler,pmCompilerProc,pmNoReturn];

  PCUProcedureScopeFlagNames: array[TPasProcedureScopeFlag] of string = (
    'GrpOverload',
    'Specialized'
    );

  PCUForLoopType: array[TLoopType] of string = (
    'Normal',
    'Down',
    'In'
    );

  PCUAssignKind: array[TAssignKind] of string = (
    'Default',
    'Add',
    'Minus',
    'Mul',
    'Division'
    );

  PCUDefaultPSRefAccess = psraRead;
  PCUPSRefAccessNames: array[TPSRefAccess] of string = (
    'None',
    'Read',
    'Write',
    'ReadWrite',
    'WriteRead',
    'TypeInfo'
    );

  PCUResolvedRefAccessNames: array[TResolvedRefAccess] of string = (
    'None',
    'Read',
    'Assign',
    'ReadAndAssign',
    'VarParam',
    'OutParam',
    'ParamToUnknownProc'
    );

  PCUResolvedReferenceFlagNames: array[TResolvedReferenceFlag] of string = (
    'Dot',
    'ImplicitCall',
    'NoImplicitCall',
    'NewInst',
    'FreeInst',
    'VMT',
    'ConstInh'
    );

  PCUResolverWithExprScopeFlagNames: array[TPasWithExprScopeFlag] of string = (
    'NeedTmpVar',
    'OnlyTypeMembers',
    'IsClassOf',
    'ConstParent'
    );


type
  { TPCUInitialFlags }

  TPCUInitialFlags = class
  public
    ParserOptions: TPOptions;
    ModeSwitches: TModeSwitches;
    BoolSwitches: TBoolSwitches;
    ConverterOptions: TPasToJsConverterOptions;
    TargetPlatform: TPasToJsPlatform;
    TargetProcessor: TPasToJsProcessor;
    // ToDo: defines
    constructor Create;
    procedure Clear;
  end;

type
  TPCUSourceFileType = (
    sftUnit,
    sftInclude
  );
  TPCUSourceFileKinds = set of TPCUSourceFileType;
const
  PCUSourceFileTypeNames: array[TPCUSourceFileType] of string = (
    'Unit',
    'Include'
    );

type
  TPCUSourceFileChecksum = cardinal;
  EPas2JsFilerError = class(Exception)
  public
    Owner: TObject;
  end;
  EPas2JsFilerErrorClass = class of EPas2JsFilerError;
  EPas2JsWriteError = class(EPas2JsFilerError);
  EPas2JsReadError = class(EPas2JsFilerError);

  { TPCUSourceFile }

  TPCUSourceFile = class
  public
    FileType: TPCUSourceFileType;
    Filename: string;
    Checksum: TPCUSourceFileChecksum;
    Index: integer;
  end;
  TPCUSourceFileArray = array of TPCUSourceFile;

  TPCUGetSrcEvent = procedure(Sender: TObject; aFilename: string;
    out p: PChar; out Count: integer) of object;

  { TPCUFilerContext - base class TPCUWriterContext/TPCUReaderContext }

  TPCUFilerContext = class
  public
    ModeSwitches: TModeSwitches;
    BoolSwitches: TBoolSwitches;
    InGeneric: boolean;
  end;

  { TPCUFilerPendingElRef }

  TPCUFilerPendingElRef = class
  public
    Next: TPCUFilerPendingElRef;
    ErrorEl: TPasElement;
  end;

  { TPCUFilerElementRef }

  TPCUFilerElementRef = class
  public
    ParentRef: TPCUFilerElementRef;
    Element: TPasElement;
    Id: integer; // 0 = pending
    Pending: TPCUFilerPendingElRef;
    Obj: TJSONObject;
    Elements: TJSONArray; // for external references
    Specs: TJSONArray; // for specializations
    NextNewExt: TPCUFilerElementRef; // next new external reference
    procedure AddPending(Item: TPCUFilerPendingElRef);
    procedure Clear;
    destructor Destroy; override;
  end;
  TPCUFilerElementRefArray = array of TPCUFilerElementRef;

  { TPCUFiler - base class TPCUWriter/TPCUReader}

  TPCUFiler = class
  private
    FErrorClass: EPas2JsFilerErrorClass;
    FFileVersion: longint;
    FGUID: TGUID;
    FInitialFlags: TPCUInitialFlags;
    FOnGetSrc: TPCUGetSrcEvent;
    FParser: TPasParser;
    FResolver: TPas2JSResolver;
    FScanner: TPascalScanner;
    FSourceFiles: TObjectList;
    function GetSourceFiles(Index: integer): TPCUSourceFile;
  protected
    FElementRefs: TAVLTree; // tree of TPCUFilerElementRef sorted for Element
    procedure RaiseMsg(Id: int64; const Msg: string = ''); virtual; overload;
    procedure RaiseMsg(Id: int64; El: TPasElement; const Msg: string = ''); overload;
    function GetDefaultMemberVisibility(El: TPasElement): TPasMemberVisibility; virtual;
    function GetDefaultPasScopeVisibilityContext(Scope: TPasScope): TPasElement; virtual;
    procedure GetDefaultsPasIdentifierProps(El: TPasElement; out Kind: TPasIdentifierKind; out Name: string); virtual;
    function GetDefaultClassScopeFlags(Scope: TPas2JSClassScope): TPasClassScopeFlags; virtual;
    function GetDefaultProcModifiers(Proc: TPasProcedure): TProcedureModifiers; virtual;
    function GetDefaultProcTypeModifiers(ProcType: TPasProcedureType): TProcTypeModifiers; virtual;
    function GetDefaultExprHasEvalValue(Expr: TPasExpr): boolean; virtual;
    function GetSrcCheckSum(aFilename: string): TPCUSourceFileChecksum; virtual;
    function GetDefaultRefName(El: TPasElement): string; virtual;
    function GetElementReference(El: TPasElement; AutoCreate: boolean = true): TPCUFilerElementRef;
    function CreateElementRef(El: TPasElement): TPCUFilerElementRef; virtual;
    procedure AddedBuiltInRef(Ref: TPCUFilerElementRef); virtual;
  public
    constructor Create; virtual;
    destructor Destroy; override;
    procedure Clear; virtual;
    property Resolver: TPas2JSResolver read FResolver;
    property Parser: TPasParser read FParser;
    property Scanner: TPascalScanner read FScanner;
    property InitialFlags: TPCUInitialFlags read FInitialFlags;
    property OnGetSrc: TPCUGetSrcEvent read FOnGetSrc write FOnGetSrc;
    function SourceFileCount: integer;
    property SourceFiles[Index: integer]: TPCUSourceFile read GetSourceFiles;
    property ElementRefs: TAVLTree read FElementRefs;
    property GUID: TGUID read FGUID write FGUID;
    property ErrorClass: EPas2JsFilerErrorClass read FErrorClass write FErrorClass;
  end;

  { TPCUCustomWriter }

  TPCUCustomWriter = class(TPCUFiler)
  private
    FOnIsElementUsed: TPas2JSIsElementUsedEvent;
  public
    constructor Create; override;
    procedure WritePCU(aResolver: TPas2JSResolver; aConverter: TPasToJSConverter;
      InitFlags: TPCUInitialFlags; aStream: TStream; Compressed: boolean); virtual; abstract;
    property OnIsElementUsed: TPas2JSIsElementUsedEvent read FOnIsElementUsed write FOnIsElementUsed;
  end;
  TPCUWriterClass = class of TPCUWriter;

  { TPCUCustomReader }

  TPCUCustomReader = class(TPCUFiler)
  private
    FPCUFilename: string;
    FSourceFilename: string;
  public
    constructor Create; override;
    procedure ReadPCU(aResolver: TPas2JSResolver; aStream: TStream); virtual; abstract;
    function ReadContinue: boolean; virtual; abstract;  // true=finished
    function ReadCanContinue: boolean; virtual; // true=not finished and no pending used interface
    property SourceFilename: string read FSourceFilename write FSourceFilename; // default value for TPasElement.SourceFilename
    property PCUFilename: string read FPCUFilename write FPCUFilename; // for nicer error messages
  end;
  TPCUReaderClass = class of TPCUCustomReader;


  { TPCUWriterContext }

  TPCUWriterContext = class(TPCUFilerContext)
  public
    Section: TPasSection;
    SectionObj: TJSONObject;
    IndirectUsesArr: TJSONArray;
  end;

  { TPCUWriterPendingElRefObj }

  TPCUWriterPendingElRefObj = class(TPCUFilerPendingElRef)
  public
    Obj: TJSONObject;
    PropName: string;
  end;

  { TPCUWriterPendingElRefArray }

  TPCUWriterPendingElRefArray = class(TPCUFilerPendingElRef)
  public
    Arr: TJSONArray;
    Index: integer;
  end;

  { TPCUWriter }

  TPCUWriter = class(TPCUCustomWriter)
  private
    FConverter: TPasToJSConverter;
    FElementIdCounter: integer;
    FJSON: TJSONObject;
    FSourceFilesSorted: TPCUSourceFileArray;
    FInImplementation: boolean;
    FBuiltInSymbolsArr: TJSONArray;
  protected
    FFirstNewExt, FLastNewExt: TPCUFilerElementRef; // not yet stored external references
    procedure ResolvePendingElRefs(Ref: TPCUFilerElementRef);
    function CheckElScope(El: TPasElement; NotNilId: int64; ScopeClass: TPasScopeClass): TPasScope; virtual;
    procedure AddArrayFlag(Obj: TJSONObject; var Arr: TJSONArray;
      const ArrName, Flag: string; Enable: boolean);
    procedure AddReferenceToArray(Arr: TJSONArray; El: TPasElement; WriteNull: boolean = true); virtual;
    procedure AddReferenceToObj(Obj: TJSONObject; const PropName: string;
      El: TPasElement; WriteNil: boolean = false); virtual;
    procedure CreateAutoElReferenceId(Ref: TPCUFilerElementRef); virtual;
    procedure CreateElReferenceId(Ref: TPCUFilerElementRef); virtual;
    function CreateElementRef(El: TPasElement): TPCUFilerElementRef; override;
    procedure AddedBuiltInRef(Ref: TPCUFilerElementRef); override;
  protected
    procedure WriteHeaderMagic(Obj: TJSONObject); virtual;
    procedure WriteHeaderVersion(Obj: TJSONObject); virtual;
    procedure WriteGUID(Obj: TJSONObject); virtual;
    // compiler directives
    procedure WriteInitialFlags(Obj: TJSONObject); virtual;
    procedure WriteFinalFlags(Obj: TJSONObject); virtual;
    procedure WriteParserOptions(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TPOptions); virtual;
    procedure WriteModeSwitches(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TModeSwitches); virtual;
    procedure WriteBoolSwitches(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TBoolSwitches); virtual;
    procedure WriteConverterOptions(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TPasToJsConverterOptions); virtual;
    procedure WriteSrcFiles(Obj: TJSONObject); virtual;
    // sets
    procedure WriteMemberHints(Obj: TJSONObject; const Value, DefaultValue: TPasMemberHints); virtual;
    procedure WriteVarModifiers(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TVariableModifiers); virtual;
    // scopes
    procedure WritePasScope(Obj: TJSONObject; Scope: TPasScope; aContext: TPCUWriterContext); virtual;
    procedure WriteIdentifierScope(Obj: TJSONObject; Scope: TPasIdentifierScope; aContext: TPCUWriterContext); virtual;
    procedure WriteModuleScopeFlags(Obj: TJSONObject; const Value, DefaultValue: TPasModuleScopeFlags); virtual;
    procedure WriteModuleScope(Obj: TJSONObject; Scope: TPas2JSModuleScope; aContext: TPCUWriterContext); virtual;
    procedure WriteModuleScopeLocalVars(Obj: TJSONObject; Scope: TPas2JSModuleScope); virtual;
    // element utilities
    procedure WriteSrcPos(Obj: TJSONObject; El: TPasElement; aContext: TPCUWriterContext); virtual;
    procedure WritePasElement(Obj: TJSONObject; El: TPasElement; aContext: TPCUWriterContext); virtual;
    procedure WriteModule(Obj: TJSONObject; aModule: TPasModule; aContext: TPCUWriterContext); virtual;
    procedure WriteSection(ParentJSON: TJSONObject; Section: TPasSection;
      const PropName: string; aContext: TPCUWriterContext); virtual;
    procedure WriteDeclarations(ParentJSON: TJSONObject; Decls: TPasDeclarations; aContext: TPCUWriterContext); virtual;
    procedure WriteElementProperty(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; El: TPasElement; aContext: TPCUWriterContext); virtual;
    procedure WriteElementList(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; ListOfElements: TFPList; aContext: TPCUWriterContext;
      ReferencesAllowed: boolean = false); virtual;
    procedure WriteElementArray(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; ArrOfElements: TPasElementArray; aContext: TPCUWriterContext;
      ReferencesAllowed: boolean = false); virtual;
    procedure WriteElType(Obj: TJSONObject; El: TPasElement; const PropName: string; aType: TPasType; aContext: TPCUWriterContext); virtual;
    procedure WriteStrings(Obj: TJSONObject; const PropName: string; aList: TStrings; aContext: TPCUWriterContext); virtual;
    // resolver flags/references
    procedure WriteResolvedRefFlags(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TResolvedReferenceFlags); virtual;
    procedure WriteResolvedReference(Obj: TJSONObject; Ref: TResolvedReference; ErrorEl: TPasElement); virtual;
    // expression
    procedure WriteExprCustomData(Obj: TJSONObject; Expr: TPasExpr; aContext: TPCUWriterContext); virtual;
    procedure WriteExpr(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; Expr: TPasExpr; aContext: TPCUWriterContext); virtual;
    procedure WritePasExpr(Obj: TJSONObject; Expr: TPasExpr;
      DefaultKind: TPasExprKind; DefaultOpCode: TExprOpCode; aContext: TPCUWriterContext); virtual;
    procedure WritePasExprArray(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; const ExprArr: TPasExprArray; aContext: TPCUWriterContext); virtual;
    // references of an impl block which statements are not stored
    procedure WriteScopeReferences(Obj: TJSONObject; References: TPasScopeReferences;
      const PropName: string; aContext: TPCUWriterContext); virtual;
    // extern references
    function IsExternalEl(El: TPasElement): boolean; virtual;
    procedure WriteExtRefSignature(Ref: TPCUFilerElementRef; aContext: TPCUWriterContext); virtual;
    function WriteExternalReference(El: TPasElement; aContext: TPCUWriterContext): TPCUFilerElementRef; virtual;
    procedure WriteExternalReferences(aContext: TPCUWriterContext); virtual;
    // pas elements
    procedure WriteElement(Obj: TJSONObject; El: TPasElement; aContext: TPCUWriterContext); virtual;
    procedure WriteUnaryExpr(Obj: TJSONObject; Expr: TUnaryExpr; aContext: TPCUWriterContext); virtual;
    procedure WriteBinaryExpr(Obj: TJSONObject; Expr: TBinaryExpr; aContext: TPCUWriterContext); virtual;
    procedure WritePrimitiveExpr(Obj: TJSONObject; Expr: TPrimitiveExpr; aContext: TPCUWriterContext); virtual;
    procedure WriteBoolConstExpr(Obj: TJSONObject; Expr: TBoolConstExpr; aContext: TPCUWriterContext); virtual;
    procedure WriteParamsExpr(Obj: TJSONObject; Expr: TParamsExpr; aContext: TPCUWriterContext); virtual;
    procedure WriteProcedureExpr(Obj: TJSONObject; Expr: TProcedureExpr; aContext: TPCUWriterContext); virtual;
    procedure WriteRecordValues(Obj: TJSONObject; Expr: TRecordValues; aContext: TPCUWriterContext); virtual;
    procedure WriteArrayValues(Obj: TJSONObject; Expr: TArrayValues; aContext: TPCUWriterContext); virtual;
    procedure WriteResString(Obj: TJSONObject; El: TPasResString; aContext: TPCUWriterContext); virtual;
    procedure WriteGenericTemplateTypes(Obj: TJSONObject; Parent: TPasElement; GenericTemplateTypes: TFPList; aContext: TPCUWriterContext); virtual;
    procedure WriteAliasType(Obj: TJSONObject; El: TPasAliasType; aContext: TPCUWriterContext); virtual;
    procedure WritePointerType(Obj: TJSONObject; El: TPasPointerType; aContext: TPCUWriterContext); virtual;
    procedure WriteSpecializeType(Obj: TJSONObject; El: TPasSpecializeType; aContext: TPCUWriterContext); virtual;
    procedure WriteInlineSpecializeExpr(Obj: TJSONObject; Expr: TInlineSpecializeExpr; aContext: TPCUWriterContext); virtual;
    procedure WriteRangeType(Obj: TJSONObject; El: TPasRangeType; aContext: TPCUWriterContext); virtual;
    procedure WriteArrayTypeScope(Obj: TJSONObject; Scope: TPas2JSArrayScope; aContext: TPCUWriterContext); virtual;
    procedure WriteArrayType(Obj: TJSONObject; El: TPasArrayType; aContext: TPCUWriterContext); virtual;
    procedure WriteFileType(Obj: TJSONObject; El: TPasFileType; aContext: TPCUWriterContext); virtual;
    procedure WriteEnumValue(Obj: TJSONObject; El: TPasEnumValue; aContext: TPCUWriterContext); virtual;
    procedure WriteEnumTypeScope(Obj: TJSONObject; Scope: TPasEnumTypeScope; aContext: TPCUWriterContext); virtual;
    procedure WriteEnumType(Obj: TJSONObject; El: TPasEnumType; aContext: TPCUWriterContext); virtual;
    procedure WriteSetType(Obj: TJSONObject; El: TPasSetType; aContext: TPCUWriterContext); virtual;
    procedure WriteRecordVariant(Obj: TJSONObject; El: TPasVariant; aContext: TPCUWriterContext); virtual;
    procedure WriteRecordTypeScope(Obj: TJSONObject; Scope: TPas2jsRecordScope; aContext: TPCUWriterContext); virtual;
    procedure WriteRecordType(Obj: TJSONObject; El: TPasRecordType; aContext: TPCUWriterContext); virtual;
    procedure WriteClassScopeFlags(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TPasClassScopeFlags); virtual;
    procedure WriteClassIntfMapProcs(Obj: TJSONObject; Map: TPasClassIntfMap); virtual;
    procedure WriteClassScope(Obj: TJSONObject; Scope: TPas2JSClassScope; aContext: TPCUWriterContext); virtual;
    procedure WriteClassType(Obj: TJSONObject; El: TPasClassType; aContext: TPCUWriterContext); virtual;
    procedure WriteArgument(Obj: TJSONObject; El: TPasArgument; aContext: TPCUWriterContext); virtual;
    procedure WriteProcTypeModifiers(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TProcTypeModifiers); virtual;
    procedure WriteProcTypeScope(Obj: TJSONObject; Scope: TPas2JSProcTypeScope; aContext: TPCUWriterContext); virtual;
    procedure WriteProcedureType(Obj: TJSONObject; El: TPasProcedureType; aContext: TPCUWriterContext); virtual;
    procedure WriteResultElement(Obj: TJSONObject; El: TPasResultElement; aContext: TPCUWriterContext); virtual;
    procedure WriteFunctionType(Obj: TJSONObject; El: TPasFunctionType; aContext: TPCUWriterContext); virtual;
    procedure WriteStringType(Obj: TJSONObject; El: TPasStringType; aContext: TPCUWriterContext); virtual;
    procedure WriteVariable(Obj: TJSONObject; El: TPasVariable; aContext: TPCUWriterContext); virtual;
    procedure WriteExportSymbol(Obj: TJSONObject; El: TPasExportSymbol; aContext: TPCUWriterContext); virtual;
    procedure WriteConst(Obj: TJSONObject; El: TPasConst; aContext: TPCUWriterContext); virtual;
    procedure WritePropertyScope(Obj: TJSONObject; Scope: TPasPropertyScope; aContext: TPCUWriterContext); virtual;
    procedure WriteProperty(Obj: TJSONObject; El: TPasProperty; aContext: TPCUWriterContext); virtual;
    procedure WriteMethodResolution(Obj: TJSONObject; El: TPasMethodResolution; aContext: TPCUWriterContext); virtual;
    procedure WriteGenericTemplateType(Obj: TJSONObject; El: TPasGenericTemplateType; aContext: TPCUWriterContext); virtual;
    procedure WriteProcedureNameParts(Obj: TJSONObject; El: TPasProcedure; aContext: TPCUWriterContext); virtual;
    procedure WriteProcedureModifiers(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TProcedureModifiers); virtual;
    procedure WriteProcScopeFlags(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TPasProcedureScopeFlags); virtual;
    procedure WriteProcedureScope(Obj: TJSONObject; Scope: TPas2JSProcedureScope; aContext: TPCUWriterContext); virtual;
    procedure WriteProcedureBody(Obj: TJSONObject; El: TProcedureBody; aContext: TPCUWriterContext); virtual;
    procedure WriteProcedure(Obj: TJSONObject; El: TPasProcedure; aContext: TPCUWriterContext); virtual;
    procedure WriteOperator(Obj: TJSONObject; El: TPasOperator; aContext: TPCUWriterContext); virtual;
    procedure WriteAttributes(Obj: TJSONObject; El: TPasAttributes; aContext: TPCUWriterContext); virtual;
    procedure WritePrecompiledJS(Obj: TJSONObject; El: TPasElement; ImplJS: TPas2JSPrecompiledJS; aContext: TPCUWriterContext); virtual;
    procedure WriteImplCommand(Obj: TJSONObject; El: TPasImplCommand; aContext: TPCUWriterContext); virtual;
    procedure WriteImplBeginBlock(Obj: TJSONObject; El: TPasImplBeginBlock; aContext: TPCUWriterContext); virtual;
    procedure WriteImplAsmStatement(Obj: TJSONObject; El: TPasImplAsmStatement; aContext: TPCUWriterContext); virtual;
    procedure WriteImplRepeatUntil(Obj: TJSONObject; El: TPasImplRepeatUntil; aContext: TPCUWriterContext); virtual;
    procedure WriteImplIfElse(Obj: TJSONObject; El: TPasImplIfElse; aContext: TPCUWriterContext); virtual;
    procedure WriteImplWhileDo(Obj: TJSONObject; El: TPasImplWhileDo; aContext: TPCUWriterContext); virtual;
    procedure WriteImplWithDo(Obj: TJSONObject; El: TPasImplWithDo; aContext: TPCUWriterContext); virtual;
    procedure WriteImplWithFlags(Obj: TJSONObject; const PropName: string; const Value, DefaultValue: TPasWithExprScopeFlags); virtual;
    procedure WriteImplCaseOf(Obj: TJSONObject; El: TPasImplCaseOf; aContext: TPCUWriterContext); virtual;
    procedure WriteImplCaseStatement(Obj: TJSONObject; El: TPasImplCaseStatement; aContext: TPCUWriterContext); virtual;
    procedure WriteImplCaseElse(Obj: TJSONObject; El: TPasImplCaseElse; aContext: TPCUWriterContext); virtual;
    procedure WriteImplForLoop(Obj: TJSONObject; El: TPasImplForLoop; aContext: TPCUWriterContext); virtual;
    procedure WriteImplAssign(Obj: TJSONObject; El: TPasImplAssign; aContext: TPCUWriterContext); virtual;
    procedure WriteImplSimple(Obj: TJSONObject; El: TPasImplSimple; aContext: TPCUWriterContext); virtual;
    procedure WriteImplTry(Obj: TJSONObject; El: TPasImplTry; aContext: TPCUWriterContext); virtual;
    procedure WriteImplTryHandler(Obj: TJSONObject; El: TPasImplTryHandler; aContext: TPCUWriterContext); virtual;
    procedure WriteImplExceptOn(Obj: TJSONObject; El: TPasImplExceptOn; aContext: TPCUWriterContext); virtual;
    procedure WriteImplRaise(Obj: TJSONObject; El: TPasImplRaise; aContext: TPCUWriterContext); virtual;
  public
    constructor Create; override;
    destructor Destroy; override;
    procedure Clear; override;
    procedure WritePCU(aResolver: TPas2JSResolver; aConverter: TPasToJSConverter;
      InitFlags: TPCUInitialFlags; aStream: TStream; Compressed: boolean); override;
    function WriteJSON(aResolver: TPas2JSResolver; aConverter: TPasToJSConverter;
      InitFlags: TPCUInitialFlags): TJSONObject; virtual;
    function IndexOfSourceFile(const Filename: string): integer;
    property SourceFilesSorted: TPCUSourceFileArray read FSourceFilesSorted;
    property JSON: TJSONObject read FJSON;
    property Converter: TPasToJSConverter read FConverter;
  end;

  { TPCUReaderContext }

  TPCUReaderContext = class(TPCUFilerContext)
  end;

  TOnSetElReference = procedure(El: TPasElement; Data: TObject) of object;

  { TPCUReaderPendingElRef }

  TPCUReaderPendingElRef = class(TPCUFilerPendingElRef)
  public
    Data: TObject;
    Setter: TOnSetElReference;
  end;

  TPCUAddRef = {$IFDEF CheckPasTreeRefCount}String{$ELSE}boolean{$ENDIF};

  { TPCUReaderPendingElListRef }

  TPCUReaderPendingElListRef = class(TPCUFilerPendingElRef)
  public
    List: TFPList;
    Index: integer;
    AddRef: TPCUAddRef;
  end;

  { TPCUReaderPendingElArrRef }

  TPCUReaderPendingElArrRef = class(TPCUFilerPendingElRef)
  public
    Arr: TPasElementArray;
    Index: integer;
    AddRef: TPCUAddRef;
  end;

  { TPCUReaderPendingElScopeRef }

  TPCUReaderPendingElScopeRef = class(TPCUFilerPendingElRef)
  public
    References: TPasScopeReferences;
    Access: TPSRefAccess;
  end;

  { TPCUReaderPendingIdentifierScope }

  TPCUReaderPendingIdentifierScope = class
  public
    Scope: TPasIdentifierScope;
    Arr: TJSONArray;
  end;

  TPCUReaderPendingSpecialized = class;

  { TPCUReaderPendingSpecializedParam }

  TPCUReaderPendingSpecializedParam = class
  public
    Spec: TPCUReaderPendingSpecialized;
    Index: integer; // index in Spec.Params
    Id: integer;
    Element: TPasElement;
  end;

  { TPCUReaderPendingSpecialized }

  TPCUReaderPendingSpecialized = class
  public
    Obj: TJSONObject;
    GenericEl: TPasGenericType;
    Id: integer;
    Params: TFPList; // list of TPCUReaderPendingSpecializedParam
    RefEl: TPasElement; // a TInlineSpecializeExpr, TPasSpecializeType, TPasProcedure or TInitializationSection
    SpecName: string;
    Prev, Next: TPCUReaderPendingSpecialized;
    destructor Destroy; override;
  end;

  { TPCUReader }

  TPCUReader = class(TPCUCustomReader)
  private
    FElementRefsArray: TPCUFilerElementRefArray; // TPCUFilerElementRef by Id
    FJSON: TJSONObject;
    FPendingIdentifierScopes: TObjectList; // list of TPCUReaderPendingIdentifierScope
    FPendingForwardProcs: TFPList; // list of TPasElement waiting for implementation of methods
    procedure Set_Variable_VarType(RefEl: TPasElement; Data: TObject);
    procedure Set_AliasType_DestType(RefEl: TPasElement; Data: TObject);
    procedure Set_PointerType_DestType(RefEl: TPasElement; Data: TObject);
    procedure Set_ArrayType_ElType(RefEl: TPasElement; Data: TObject);
    procedure Set_FileType_ElType(RefEl: TPasElement; Data: TObject);
    procedure Set_SetType_EnumType(RefEl: TPasElement; Data: TObject);
    procedure Set_Variant_Members(RefEl: TPasElement; Data: TObject);
    procedure Set_RecordType_VariantEl(RefEl: TPasElement; Data: TObject);
    procedure Set_RecordScope_DefaultProperty(RefEl: TPasElement; Data: TObject);
    procedure Set_Argument_ArgType(RefEl: TPasElement; Data: TObject);
    procedure Set_ClassScope_NewInstanceFunction(RefEl: TPasElement; Data: TObject);
    procedure Set_ClassScope_DirectAncestor(RefEl: TPasElement; Data: TObject);
    procedure Set_ClassScope_DefaultProperty(RefEl: TPasElement; Data: TObject);
    procedure Set_ClassIntfMap_Intf(RefEl: TPasElement; Data: TObject);
    procedure Set_ClassType_AncestorType(RefEl: TPasElement; Data: TObject);
    procedure Set_ClassType_HelperForType(RefEl: TPasElement; Data: TObject);
    procedure Set_ResultElement_ResultType(RefEl: TPasElement; Data: TObject);
    procedure Set_PasScope_VisibilityContext(RefEl: TPasElement; Data: TObject);
    procedure Set_ModScope_AssertClass(RefEl: TPasElement; Data: TObject);
    procedure Set_ModScope_AssertDefConstructor(RefEl: TPasElement; Data: TObject);
    procedure Set_ModScope_AssertMsgConstructor(RefEl: TPasElement; Data: TObject);
    procedure Set_ModScope_RangeErrorClass(RefEl: TPasElement; Data: TObject);
    procedure Set_ModScope_RangeErrorConstructor(RefEl: TPasElement; Data: TObject);
    procedure Set_ModScope_SystemTVarRec(RefEl: TPasElement; Data: TObject);
    procedure Set_ModScope_SystemVarRecs(RefEl: TPasElement; Data: TObject);
    procedure Set_LocalVar(RefEl: TPasElement; Data: TObject);
    procedure Set_EnumTypeScope_CanonicalSet(RefEl: TPasElement; Data: TObject);
    procedure Set_PropertyScope_AncestorProp(RefEl: TPasElement; Data: TObject);
    procedure Set_ProcedureScope_ImplProc(RefEl: TPasElement; Data: TObject);
    procedure Set_ProcedureScope_Overridden(RefEl: TPasElement; Data: TObject);
    procedure Set_ExceptOn_TypeEl(RefEl: TPasElement; Data: TObject);
    procedure Set_ResolvedReference_Declaration(RefEl: TPasElement; Data: TObject);
    procedure Set_ResolvedReference_CtxConstructor(RefEl: TPasElement; Data: TObject);
    procedure Set_ResolvedReference_CtxAttrProc(RefEl: TPasElement; Data: TObject);
    procedure Set_SpecializeTypeData(RefEl: TPasElement; Data: TObject);
  protected
    // specialize
    FPendingSpecialize: TPCUReaderPendingSpecialized; // chain of TPCUReaderPendingSpecialized
    function FindPendingSpecialize(Id: integer): TPCUReaderPendingSpecialized;
    function AddPendingSpecialize(Id: integer; const SpecName: string): TPCUReaderPendingSpecialized;
    function CreateSpecializedElement(PendSpec: TPCUReaderPendingSpecialized): boolean; // false=param missing, Note: needs ResolvePendingIdentifierScopes
    procedure DeletePendingSpecialize(PendSpec: TPCUReaderPendingSpecialized);
    function PromiseSpecialize(SpecId: integer; const SpecName: string; RefEl, ErrorEl: TPasElement): TPCUReaderPendingSpecialized; virtual;
    procedure ResolveSpecializedElements(Complete: boolean);
    function IsSpecialize(ChildEl: TPasElement): boolean;
  protected
    // json
    procedure RaiseMsg(Id: int64; const Msg: string = ''); overload; override;
    function CheckJSONArray(Data: TJSONData; El: TPasElement; const PropName: string): TJSONArray;
    function CheckJSONObject(Data: TJSONData; Id: int64): TJSONObject;
    function CheckJSONString(Data: TJSONData; Id: int64): String;
    function ReadString(Obj: TJSONObject; const PropName: string; out s: string; El: TPasElement): boolean;
    function ReadInteger(Obj: TJSONObject; const PropName: string; out i: integer; El: TPasElement): boolean;
    function ReadBoolean(Obj: TJSONObject; const PropName: string; out b: boolean; El: TPasElement): boolean;
    function ReadArray(Obj: TJSONObject; const PropName: string; out Arr: TJSONArray; El: TPasElement): boolean;
    function ReadObject(Obj: TJSONObject; const PropName: string; out SubObj: TJSONObject; El: TPasElement): boolean;
    procedure ReadArrayFlags(Data: TJSONData; El: TPasElement; const PropName: string; out Names: TStringDynArray; out Enable: TBooleanDynArray);
    function CreateContext: TPCUReaderContext; virtual;
    // reference
    function GetElReference(Id: integer; ErrorEl: TPasElement): TPCUFilerElementRef; virtual;
    function AddElReference(Id: integer; ErrorEl: TPasElement; El: TPasElement): TPCUFilerElementRef; virtual;
    procedure PromiseSetElReference(Id: integer; const Setter: TOnSetElReference;
      Data: TObject; ErrorEl: TPasElement); virtual;
    procedure PromiseSetElListReference(Id: integer; List: TFPList; Index: integer;
      AddRef: TPCUAddRef; ErrorEl: TPasElement); virtual;
    procedure PromiseSetElArrReference(Id: integer; Arr: TPasElementArray; Index: integer;
      AddRef: TPCUAddRef; ErrorEl: TPasElement); virtual;
    procedure PromiseSetScopeReference(Id: integer; References: TPasScopeReferences;
      Access: TPSRefAccess; ErrorEl: TPasElement); virtual;
    procedure ResolvePendingIdentifierScopes; virtual;
    procedure ResolvePending(Complete: boolean); virtual;
    function GetReferrerEl(PendingElRef: TPCUFilerPendingElRef): TPasElement;
    procedure ReadBuiltInSymbols(Obj: TJSONObject; ErrorEl: TPasElement); virtual;
    // module
    procedure ReadHeaderMagic(Obj: TJSONObject); virtual;
    procedure ReadHeaderVersion(Obj: TJSONObject); virtual;
    procedure ReadGUID(Obj: TJSONObject); virtual;
    procedure ReadHeaderItem(const PropName: string; Data: TJSONData); virtual;

    // compiler directives
    function ReadParserOptions(Obj: TJSONObject; El: TPasElement; const PropName: string; const DefaultValue: TPOptions): TPOptions; virtual;
    function ReadModeSwitches(Obj: TJSONObject; El: TPasElement; const PropName: string; const DefaultValue: TModeSwitches): TModeSwitches; virtual;
    function ReadBoolSwitches(Obj: TJSONObject; El: TPasElement; const PropName: string; const DefaultValue: TBoolSwitches): TBoolSwitches; virtual;
    function ReadConverterOptions(Obj: TJSONObject; El: TPasElement; const PropName: string; const DefaultValue: TPasToJsConverterOptions): TPasToJsConverterOptions; virtual;
    procedure ReadTargetPlatform(Data: TJSONData); virtual;
    procedure ReadTargetProcessor(Data: TJSONData); virtual;
    procedure ReadSrcFiles(Data: TJSONData); virtual;
    // sets
    function ReadMemberHints(Obj: TJSONObject; El: TPasElement; const DefaultValue: TPasMemberHints): TPasMemberHints; virtual;
    // element utilities
    procedure ReadSrcPos(Obj: TJSONObject; El: TPasElement; aContext: TPCUReaderContext); virtual;
    procedure ReadPasElement(Obj: TJSONObject; El: TPasElement; aContext: TPCUReaderContext); virtual;
    procedure ReadExternalMembers(El: TPasElement; Arr: TJSONArray; Members: TFPList); virtual;
    procedure ReadSpecializations(Obj: TJSONObject; El: TPasGenericType); virtual;
    procedure ReadSpecialization(Obj: TJSONObject; GenEl: TPasGenericType; ParamIDs: TJSONArray); virtual;
    procedure ReadExternalReferences(Obj: TJSONObject; El: TPasElement); virtual;
    procedure ReadUsedUnitsInit(Obj: TJSONObject; Section: TPasSection; aContext: TPCUReaderContext); virtual;
    procedure ReadIndirectUsedUnits(Obj: TJSONObject; Section: TPasSection; aComplete: boolean); virtual;
    procedure ReadUsedUnitsFinish(Obj: TJSONObject; Section: TPasSection; aContext: TPCUReaderContext); virtual;
    procedure ReadSectionScope(Obj: TJSONObject; Scope: TPas2JSSectionScope; aContext: TPCUReaderContext); virtual;
    procedure ReadSection(Obj: TJSONObject; Section: TPasSection; aContext: TPCUReaderContext); virtual;
    procedure ReadDeclarations(Obj: TJSONObject; Decls: TPasDeclarations; aContext: TPCUReaderContext); virtual;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement): TPasElement; virtual;
    function ReadElementProperty(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; BaseClass: TPTreeElement; aContext: TPCUReaderContext): TPasElement; virtual;
    procedure ReadElementReference(Obj: TJSONObject; Instance: TPasElementBase;
      const PropName: string; const Setter: TOnSetElReference); virtual;
    procedure ReadElementList(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; ListOfElements: TFPList; AddRef: TPCUAddRef;
      aContext: TPCUReaderContext); virtual;
    procedure ReadElementArray(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; var ArrOfElements: TPasElementArray; AddRef: TPCUAddRef;
      aContext: TPCUReaderContext); virtual;
    procedure ReadElType(Obj: TJSONObject; const PropName: string; El: TPasElement;
      const Setter: TOnSetElReference; aContext: TPCUReaderContext); virtual;
    procedure ReadStrings(Obj: TJSONObject; El: TPasElement; const PropName: string; List: TStrings); virtual;
    // resolver references
    function ReadResolvedRefFlags(Obj: TJSONObject; El: TPasElement;
      const PropName: string; const DefaultValue: TResolvedReferenceFlags): TResolvedReferenceFlags; virtual;
    procedure ReadResolvedReference(Obj: TJSONObject; Ref: TResolvedReference; ErrorEl: TPasElement); virtual;
    // expression utilities
    procedure ReadPasExpr(Obj: TJSONObject; Expr: TPasExpr; DefKind: TPasExprKind; aContext: TPCUReaderContext); virtual;
    procedure ReadExprCustomData(Obj: TJSONObject; Expr: TPasExpr; aContext: TPCUReaderContext); virtual;
    function ReadExpr(Obj: TJSONObject; Parent: TPasElement; const PropName: string;
      aContext: TPCUReaderContext): TPasExpr; virtual;
    procedure ReadPasExprArray(Obj: TJSONObject; Parent: TPasElement;
      const PropName: string; var ExprArr: TPasExprArray; aContext: TPCUReaderContext); virtual;
    // scopes
    procedure ReadPasScope(Obj: TJSONObject; Scope: TPasScope; aContext: TPCUReaderContext); virtual;
    procedure ReadScopeReferences(Obj: TJSONObject; Scope: TPasScope;
      const PropName: string; var References: TPasScopeReferences); virtual;
    procedure ReadIdentifierScopeArray(Arr: TJSONArray; Scope: TPasIdentifierScope); virtual;
    procedure ReadIdentifierScope(Obj: TJSONObject; Scope: TPasIdentifierScope; aContext: TPCUReaderContext); virtual;
    function ReadModuleScopeFlags(Obj: TJSONObject; El: TPasElement; const DefaultValue: TPasModuleScopeFlags): TPasModuleScopeFlags; virtual;
    procedure ReadModuleScope(Obj: TJSONObject; Scope: TPas2JSModuleScope; aContext: TPCUReaderContext); virtual;

    procedure ReadModuleHeader(Data: TJSONData); virtual;
    // elements
    function ReadNewElement(Obj: TJSONObject; Parent: TPasElement): TPasElement; virtual;
    procedure ReadElement(Obj: TJSONObject; El: TPasElement; aContext: TPCUReaderContext); virtual;
    function ReadModule(Obj: TJSONObject; aContext: TPCUReaderContext): boolean; virtual;
    procedure ReadUnaryExpr(Obj: TJSONObject; Expr: TUnaryExpr; aContext: TPCUReaderContext); virtual;
    procedure ReadPrimitiveExpr(Obj: TJSONObject; Expr: TPrimitiveExpr; aContext: TPCUReaderContext); virtual;
    procedure ReadBinaryExpr(Obj: TJSONObject; Expr: TBinaryExpr; aContext: TPCUReaderContext); virtual;
    procedure ReadBoolConstExpr(Obj: TJSONObject; Expr: TBoolConstExpr; aContext: TPCUReaderContext); virtual;
    procedure ReadParamsExpr(Obj: TJSONObject; Expr: TParamsExpr; aContext: TPCUReaderContext); virtual;
    procedure ReadProcedureExpr(Obj: TJSONObject; Expr: TProcedureExpr; aContext: TPCUReaderContext); virtual;
    procedure ReadRecordValues(Obj: TJSONObject; Expr: TRecordValues; aContext: TPCUReaderContext); virtual;
    procedure ReadArrayValues(Obj: TJSONObject; Expr: TArrayValues; aContext: TPCUReaderContext); virtual;
    procedure ReadResString(Obj: TJSONObject; El: TPasResString; aContext: TPCUReaderContext); virtual;
    procedure ReadGenericTemplateTypes(Obj: TJSONObject; Parent: TPasElement; var GenericTemplateTypes: TFPList; aContext: TPCUReaderContext); virtual;
    procedure ReadAliasType(Obj: TJSONObject; El: TPasAliasType; aContext: TPCUReaderContext); virtual;
    procedure ReadPointerType(Obj: TJSONObject; El: TPasPointerType; aContext: TPCUReaderContext); virtual;
    procedure ReadSpecializeType(Obj: TJSONObject; El: TPasSpecializeType; aContext: TPCUReaderContext); virtual;
    procedure ReadInlineSpecializeExpr(Obj: TJSONObject; Expr: TInlineSpecializeExpr; aContext: TPCUReaderContext); virtual;
    procedure ReadRangeType(Obj: TJSONObject; El: TPasRangeType; aContext: TPCUReaderContext); virtual;
    procedure ReadArrayScope(Obj: TJSONObject; Scope: TPas2JSArrayScope; aContext: TPCUReaderContext); virtual;
    procedure ReadArrayType(Obj: TJSONObject; El: TPasArrayType; aContext: TPCUReaderContext); virtual;
    procedure ReadFileType(Obj: TJSONObject; El: TPasFileType; aContext: TPCUReaderContext); virtual;
    procedure ReadEnumValue(Obj: TJSONObject; El: TPasEnumValue; aContext: TPCUReaderContext); virtual;
    procedure ReadEnumTypeScope(Obj: TJSONObject; Scope: TPasEnumTypeScope; aContext: TPCUReaderContext); virtual;
    procedure ReadEnumType(Obj: TJSONObject; El: TPasEnumType; aContext: TPCUReaderContext); virtual;
    procedure ReadSetType(Obj: TJSONObject; El: TPasSetType; aContext: TPCUReaderContext); virtual;
    function ReadPackedMode(Obj: TJSONObject; const PropName: string; ErrorEl: TPasElement): TPackMode; virtual;
    procedure ReadRecordVariant(Obj: TJSONObject; El: TPasVariant; aContext: TPCUReaderContext); virtual;
    procedure ReadRecordScope(Obj: TJSONObject; Scope: TPas2jsRecordScope; aContext: TPCUReaderContext); virtual;
    procedure ReadRecordType(Obj: TJSONObject; El: TPasRecordType; aContext: TPCUReaderContext); virtual;
    function ReadClassInterfaceType(Obj: TJSONObject; const PropName: string; ErrorEl: TPasElement; DefaultValue: TPasClassInterfaceType): TPasClassInterfaceType;
    function ReadClassScopeFlags(Obj: TJSONObject; El: TPasElement;
      const PropName: string; const DefaultValue: TPasClassScopeFlags): TPasClassScopeFlags; virtual;
    procedure ReadClassScopeAbstractProcs(Obj: TJSONObject; Scope: TPas2JSClassScope); virtual;
    procedure ReadClassIntfMapProcs(Obj: TJSONObject; Map: TPasClassIntfMap; OrigIntfType: TPasType); virtual;
    procedure ReadClassIntfMap(Obj: TJSONObject; Scope: TPas2JSClassScope; Map: TPasClassIntfMap; OrigIntfType: TPasType); virtual;
    procedure ReadClassScopeInterfaces(Obj: TJSONObject; Scope: TPas2JSClassScope); virtual;
    procedure ReadClassScopeDispatchProcs(Obj: TJSONObject; Scope: TPas2JSClassScope); virtual;
    procedure ReadClassScope(Obj: TJSONObject; Scope: TPas2JSClassScope; aContext: TPCUReaderContext); virtual;
    procedure ReadClassType(Obj: TJSONObject; El: TPasClassType; aContext: TPCUReaderContext); virtual;
    procedure ReadArgument(Obj: TJSONObject; El: TPasArgument; aContext: TPCUReaderContext); virtual;
    function ReadProcTypeModifiers(Obj: TJSONObject; El: TPasElement;
      const PropName: string; const DefaultValue: TProcTypeModifiers): TProcTypeModifiers; virtual;
    procedure ReadProcTypeScope(Obj: TJSONObject; Scope: TPas2JSProcTypeScope; aContext: TPCUReaderContext); virtual;
    procedure ReadProcedureType(Obj: TJSONObject; El: TPasProcedureType; aContext: TPCUReaderContext); virtual;
    procedure ReadResultElement(Obj: TJSONObject; El: TPasResultElement; aContext: TPCUReaderContext); virtual;
    procedure ReadFunctionType(Obj: TJSONObject; El: TPasFunctionType; aContext: TPCUReaderContext); virtual;
    procedure ReadStringType(Obj: TJSONObject; El: TPasStringType; aContext: TPCUReaderContext); virtual;
    function ReadVarModifiers(Obj: TJSONObject; El: TPasElement;
      const PropName: string; const DefaultValue: TVariableModifiers): TVariableModifiers; virtual;
    procedure ReadVariable(Obj: TJSONObject; El: TPasVariable; aContext: TPCUReaderContext); virtual;
    procedure ReadExportSymbol(Obj: TJSONObject; El: TPasExportSymbol; aContext: TPCUReaderContext); virtual;
    procedure ReadConst(Obj: TJSONObject; El: TPasConst; aContext: TPCUReaderContext); virtual;
    procedure ReadPropertyScope(Obj: TJSONObject; Scope: TPasPropertyScope; aContext: TPCUReaderContext); virtual;
    procedure ReadProperty(Obj: TJSONObject; El: TPasProperty; aContext: TPCUReaderContext); virtual;
    procedure ReadMethodResolution(Obj: TJSONObject; El: TPasMethodResolution; aContext: TPCUReaderContext); virtual;
    procedure ReadGenericTemplateType(Obj: TJSONObject; El: TPasGenericTemplateType; aContext: TPCUReaderContext); virtual;
    procedure ReadProcedureNameParts(Obj: TJSONObject; El: TPasProcedure; aContext: TPCUReaderContext); virtual;
    function ReadProcedureModifiers(Obj: TJSONObject; El: TPasElement;
      const PropName: string; const DefaultValue: TProcedureModifiers): TProcedureModifiers; virtual;
    function ReadProcScopeFlags(Obj: TJSONObject; El: TPasElement;
      const PropName: string; const DefaultValue: TPasProcedureScopeFlags): TPasProcedureScopeFlags; virtual;
    procedure ReadProcedureScope(Obj: TJSONObject; Scope: TPas2JSProcedureScope; aContext: TPCUReaderContext); virtual;
    procedure ReadProcScopeReferences(Obj: TJSONObject; ImplScope: TPas2JSProcedureScope); virtual;
    procedure ReadProcedureBody(Obj: TJSONObject; El: TPasProcedure; aContext: TPCUReaderContext); virtual;
    procedure ReadProcedure(Obj: TJSONObject; El: TPasProcedure; aContext: TPCUReaderContext); virtual;
    procedure ReadOperator(Obj: TJSONObject; El: TPasOperator; aContext: TPCUReaderContext); virtual;
    procedure ReadAttributes(Obj: TJSONObject; El: TPasAttributes; aContext: TPCUReaderContext); virtual;
    procedure ReadPrecompiledJS(Obj: TJSONObject; El: TPasElement; ImplJS: TPas2JSPrecompiledJS; aContext: TPCUReaderContext); virtual;
    procedure ReadImplCommand(Obj: TJSONObject; El: TPasImplCommand; aContext: TPCUReaderContext); virtual;
    procedure ReadImplBeginBlock(Obj: TJSONObject; El: TPasImplBeginBlock; aContext: TPCUReaderContext); virtual;
    procedure ReadImplAsmStatement(Obj: TJSONObject; El: TPasImplAsmStatement; aContext: TPCUReaderContext); virtual;
    procedure ReadImplRepeatUntil(Obj: TJSONObject; El: TPasImplRepeatUntil; aContext: TPCUReaderContext); virtual;
    procedure ReadImplIfElse(Obj: TJSONObject; El: TPasImplIfElse; aContext: TPCUReaderContext); virtual;
    procedure ReadImplWhileDo(Obj: TJSONObject; El: TPasImplWhileDo; aContext: TPCUReaderContext); virtual;
    procedure ReadImplWithDo(Obj: TJSONObject; El: TPasImplWithDo; aContext: TPCUReaderContext); virtual;
    procedure ReadImplCaseOf(Obj: TJSONObject; El: TPasImplCaseOf; aContext: TPCUReaderContext); virtual;
    procedure ReadImplCaseStatement(Obj: TJSONObject; El: TPasImplCaseStatement; aContext: TPCUReaderContext); virtual;
    procedure ReadImplCaseElse(Obj: TJSONObject; El: TPasImplCaseElse; aContext: TPCUReaderContext); virtual;
    procedure ReadImplForLoop(Obj: TJSONObject; El: TPasImplForLoop; aContext: TPCUReaderContext); virtual;
    procedure ReadImplAssign(Obj: TJSONObject; El: TPasImplAssign; aContext: TPCUReaderContext); virtual;
    procedure ReadImplSimple(Obj: TJSONObject; El: TPasImplSimple; aContext: TPCUReaderContext); virtual;
    procedure ReadImplTry(Obj: TJSONObject; El: TPasImplTry; aContext: TPCUReaderContext); virtual;
    procedure ReadImplTryHandler(Obj: TJSONObject; El: TPasImplTryHandler; aContext: TPCUReaderContext); virtual;
    procedure ReadImplExceptOn(Obj: TJSONObject; El: TPasImplExceptOn; aContext: TPCUReaderContext); virtual;
    procedure ReadImplRaise(Obj: TJSONObject; El: TPasImplRaise; aContext: TPCUReaderContext); virtual;
  public
    constructor Create; override;
    destructor Destroy; override;
    procedure Clear; override;
    procedure ReadPCU(aResolver: TPas2JSResolver; aStream: TStream); override; // sets property JSON, reads header and returns
    procedure ReadJSONHeader(aResolver: TPas2JSResolver; Obj: TJSONObject); virtual;
    function ReadContinue: boolean; override; // true=finished
    function GetPCUExt: string; virtual; // without dot
    property FileVersion: longint read FFileVersion;
    property JSON: TJSONObject read FJSON;
  end;

  { TPas2JSPrecompileFormat }

  TPas2JSPrecompileFormat = class
  public
    Ext: string;
    Description: string; // used by -h
    ReaderClass: TPCUReaderClass;
    WriterClass: TPCUWriterClass;
    Enabled: boolean;
  end;

  { TPas2JSPrecompileFormats }

  TPas2JSPrecompileFormats = class
  private
    FItems: TObjectList; // list of TObjectList
    function GetItems(Index: integer): TPas2JSPrecompileFormat;
  public
    constructor Create;
    destructor Destroy; override;
    procedure Clear;
    function Count: integer;
    function Add(aFormat: TPas2JSPrecompileFormat): TPas2JSPrecompileFormats;
    function Add(const Ext, Description: string;
      const Reader: TPCUReaderClass;
      const Writer: TPCUWriterClass
      ): TPas2JSPrecompileFormat;
    function IndexOf(aFormat: TPas2JSPrecompileFormat): integer;
    function FindExt(Ext: string): TPas2JSPrecompileFormat;
    function Remove(aFormat: TPas2JSPrecompileFormat): integer;
    function Delete(Index: integer): TPas2JSPrecompileFormats;
    property Items[Index: integer]: TPas2JSPrecompileFormat read GetItems; default;
  end;

var
  PrecompileFormats: TPas2JSPrecompileFormats = nil;
  PCUFormat: TPas2JSPrecompileFormat = nil;

procedure RegisterPCUFormat;

function ComparePointer(Data1, Data2: Pointer): integer;
function ComparePCUSrcFiles(File1, File2: Pointer): integer;
function ComparePCUFilerElementRef(Ref1, Ref2: Pointer): integer;
function CompareElWithPCUFilerElementRef(El, Ref: Pointer): integer;

function EncodeVLQ(i: TMaxPrecInt): string; overload;
function EncodeVLQ(i: TMaxPrecUInt): string; overload;
function DecodeVLQ(const s: string): TMaxPrecInt; // base256 Variable Length Quantity
function DecodeVLQ(var p: PByte): TMaxPrecInt; // base256 Variable Length Quantity

function ComputeChecksum(p: PChar; Cnt: integer): TPCUSourceFileChecksum;
function crc32(crc: cardinal; buf: Pbyte; len: cardinal): cardinal;

function ModeSwitchToInt(ms: TModeSwitch): byte;
function StrToPasIdentifierKind(const s: string): TPasIdentifierKind;

procedure WriteJSON(aData: TJSONData; TargetStream: TStream; Compressed: boolean);

procedure GrowIdToRefsArray(var IdToRefsArray: TPCUFilerElementRefArray; Id: integer);

function dbgmem(const s: string): string; overload;
function dbgmem(p: PChar; Cnt: integer): string; overload;

implementation

procedure RegisterPCUFormat;
begin
  if PCUFormat=nil then
    PCUFormat:=PrecompileFormats.Add('pcu','all used pcu must match exactly together',TPCUReader,TPCUWriter);
end;

function ComparePointer(Data1, Data2: Pointer): integer;
begin
  if Data1>Data2 then Result:=-1
  else if Data1<Data2 then Result:=1
  else Result:=0;
end;

function ComparePCUSrcFiles(File1, File2: Pointer): integer;
var
  Src1: TPCUSourceFile absolute File1;
  Src2: TPCUSourceFile absolute File2;
begin
  Result:=CompareStr(Src1.Filename,Src2.Filename);
end;

function ComparePCUFilerElementRef(Ref1, Ref2: Pointer): integer;
var
  Reference1: TPCUFilerElementRef absolute Ref1;
  Reference2: TPCUFilerElementRef absolute Ref2;
begin
  Result:=ComparePointer(Reference1.Element,Reference2.Element);
end;

function CompareElWithPCUFilerElementRef(El, Ref: Pointer): integer;
var
  Element: TPasElement absolute El;
  Reference: TPCUFilerElementRef absolute Ref;
begin
  Result:=ComparePointer(Element,Reference.Element);
end;

function EncodeVLQ(i: TMaxPrecInt): string;
{ Convert signed number to base256-VLQ:
  Each byte has 8bit, where the least significant bit is the continuation bit
  (1=there is a next byte).
  The first byte contains the sign bit in the last bit
  and the 6 most significant bits of the number.
  For example:
  0 = %00000000 => 0
  1 = %00000001 => -0
  2 = %00000010 => 1
  130 5 = %10000010 %00000101 = 000010 0000101 = 100000101 = 133
}
var
  digits: integer;
begin
  digits:=0;
  if i<0 then
    begin
    if i=Low(TMaxPrecInt) then
      begin
      Result:=EncodeVLQ(High(TMaxPrecInt)+1);
      Result[1]:=chr(ord(Result[1]) or 1);
      exit;
      end;
    digits:=1;
    i:=-i;
    end;
  inc(digits,(i and %111111) shl 1);
  i:=i shr 6;
  if i>0 then
    inc(digits,%10000000); // need another byte -> set continuation bit
  Result:=chr(digits);
  while i>0 do
    begin
    digits:=i and %1111111;
    i:=i shr 7;
    if i>0 then
      inc(digits,%10000000); // need another byte -> set continuation bit
    Result:=Result+chr(digits);
    end;
end;

function EncodeVLQ(i: TMaxPrecUInt): string;
var
  digits: integer;
begin
  digits:=(i and %111111) shl 1;
  if i>0 then
    inc(digits,%10000000); // need another byte -> set continuation bit
  Result:=chr(digits);
  i:=i shr 6;
  while i>0 do
    begin
    digits:=i and %1111111;
    i:=i shr 7;
    if i>0 then
      inc(digits,%10000000); // need another byte -> set continuation bit
    Result:=Result+chr(digits);
    end;
end;

function DecodeVLQ(const s: string): TMaxPrecInt;
var
  p: PByte;
begin
  if s='' then
    raise EConvertError.Create('DecodeVLQ empty');
  p:=PByte(s);
  Result:=DecodeVLQ(p);
  if p-PByte(s)<>length(s) then
    raise EConvertError.Create('DecodeVLQ waste');
end;

function DecodeVLQ(var p: PByte): TMaxPrecInt;
{ Convert base256-VLQ to signed number,
  For the fomat see EncodeVLQ
}

  procedure RaiseInvalid;
  begin
    raise ERangeError.Create('DecodeVLQ');
  end;

const
  MaxShift = 63; // actually log2(High(TMaxPrecInt))
var
  digit, Shift: Integer;
  Negated: Boolean;
begin
  digit:=p^;
  inc(p);
  Negated:=(digit and 1)>0;
  Result:=(digit shr 1) and %111111;
  Shift:=6;
  while digit>=%10000000 do
    begin
    digit:=p^;
    inc(p);
    if Shift>MaxShift then
      RaiseInvalid;
    inc(Result,TMaxPrecInt(digit and %1111111) shl Shift);
    inc(Shift,7);
    end;
  if Negated then
    Result:=-Result;
end;

function ComputeChecksum(p: PChar; Cnt: integer): TPCUSourceFileChecksum;
var
  SrcP, SrcEndP, SrcLineEndP, SrcLineStartP: PChar;
  l: PtrInt;
  CheckSum, CurLen: Cardinal;
begin
  if Cnt=0 then exit(0);

  // ignore trailing spaces and unify line endings
  SrcP:=p;
  SrcEndP:=p+Cnt;
  while (SrcEndP>SrcP) and (SrcEndP[-1] in [#9,#10,#13,' ']) do
    dec(SrcEndP);
  CheckSum:=crc32(0,nil,0);
  while SrcP<SrcEndP do
    begin
    SrcLineStartP:=SrcP;
    while (SrcP<SrcEndP) and not (SrcP^ in [#10,#13]) do
      inc(SrcP);
    SrcLineEndP:=SrcP;
    while (SrcLineEndP>SrcLineStartP) and (SrcLineEndP[-1] in [#9,' ']) do
      dec(SrcLineEndP);
    l:=SrcLineEndP-SrcLineStartP;
    while l>0 do
      begin
      if l<$8000 then
        CurLen:=l
      else
        CurLen:=$8000;
      CheckSum:=crc32(CheckSum, PByte(SrcLineStartP), CurLen);
      inc(SrcLineStartP,CurLen);
      dec(l,CurLen);
      end;
    while (SrcP<SrcEndP) and (SrcP^ in [#10,#13]) do
      inc(SrcP);
    end;
  Result:=CheckSum;
end;

const
  crc32_table : array[Byte] of cardinal = (
  $00000000, $77073096, $ee0e612c, $990951ba, $076dc419,
  $706af48f, $e963a535, $9e6495a3, $0edb8832, $79dcb8a4,
  $e0d5e91e, $97d2d988, $09b64c2b, $7eb17cbd, $e7b82d07,
  $90bf1d91, $1db71064, $6ab020f2, $f3b97148, $84be41de,
  $1adad47d, $6ddde4eb, $f4d4b551, $83d385c7, $136c9856,
  $646ba8c0, $fd62f97a, $8a65c9ec, $14015c4f, $63066cd9,
  $fa0f3d63, $8d080df5, $3b6e20c8, $4c69105e, $d56041e4,
  $a2677172, $3c03e4d1, $4b04d447, $d20d85fd, $a50ab56b,
  $35b5a8fa, $42b2986c, $dbbbc9d6, $acbcf940, $32d86ce3,
  $45df5c75, $dcd60dcf, $abd13d59, $26d930ac, $51de003a,
  $c8d75180, $bfd06116, $21b4f4b5, $56b3c423, $cfba9599,
  $b8bda50f, $2802b89e, $5f058808, $c60cd9b2, $b10be924,
  $2f6f7c87, $58684c11, $c1611dab, $b6662d3d, $76dc4190,
  $01db7106, $98d220bc, $efd5102a, $71b18589, $06b6b51f,
  $9fbfe4a5, $e8b8d433, $7807c9a2, $0f00f934, $9609a88e,
  $e10e9818, $7f6a0dbb, $086d3d2d, $91646c97, $e6635c01,
  $6b6b51f4, $1c6c6162, $856530d8, $f262004e, $6c0695ed,
  $1b01a57b, $8208f4c1, $f50fc457, $65b0d9c6, $12b7e950,
  $8bbeb8ea, $fcb9887c, $62dd1ddf, $15da2d49, $8cd37cf3,
  $fbd44c65, $4db26158, $3ab551ce, $a3bc0074, $d4bb30e2,
  $4adfa541, $3dd895d7, $a4d1c46d, $d3d6f4fb, $4369e96a,
  $346ed9fc, $ad678846, $da60b8d0, $44042d73, $33031de5,
  $aa0a4c5f, $dd0d7cc9, $5005713c, $270241aa, $be0b1010,
  $c90c2086, $5768b525, $206f85b3, $b966d409, $ce61e49f,
  $5edef90e, $29d9c998, $b0d09822, $c7d7a8b4, $59b33d17,
  $2eb40d81, $b7bd5c3b, $c0ba6cad, $edb88320, $9abfb3b6,
  $03b6e20c, $74b1d29a, $ead54739, $9dd277af, $04db2615,
  $73dc1683, $e3630b12, $94643b84, $0d6d6a3e, $7a6a5aa8,
  $e40ecf0b, $9309ff9d, $0a00ae27, $7d079eb1, $f00f9344,
  $8708a3d2, $1e01f268, $6906c2fe, $f762575d, $806567cb,
  $196c3671, $6e6b06e7, $fed41b76, $89d32be0, $10da7a5a,
  $67dd4acc, $f9b9df6f, $8ebeeff9, $17b7be43, $60b08ed5,
  $d6d6a3e8, $a1d1937e, $38d8c2c4, $4fdff252, $d1bb67f1,
  $a6bc5767, $3fb506dd, $48b2364b, $d80d2bda, $af0a1b4c,
  $36034af6, $41047a60, $df60efc3, $a867df55, $316e8eef,
  $4669be79, $cb61b38c, $bc66831a, $256fd2a0, $5268e236,
  $cc0c7795, $bb0b4703, $220216b9, $5505262f, $c5ba3bbe,
  $b2bd0b28, $2bb45a92, $5cb36a04, $c2d7ffa7, $b5d0cf31,
  $2cd99e8b, $5bdeae1d, $9b64c2b0, $ec63f226, $756aa39c,
  $026d930a, $9c0906a9, $eb0e363f, $72076785, $05005713,
  $95bf4a82, $e2b87a14, $7bb12bae, $0cb61b38, $92d28e9b,
  $e5d5be0d, $7cdcefb7, $0bdbdf21, $86d3d2d4, $f1d4e242,
  $68ddb3f8, $1fda836e, $81be16cd, $f6b9265b, $6fb077e1,
  $18b74777, $88085ae6, $ff0f6a70, $66063bca, $11010b5c,
  $8f659eff, $f862ae69, $616bffd3, $166ccf45, $a00ae278,
  $d70dd2ee, $4e048354, $3903b3c2, $a7672661, $d06016f7,
  $4969474d, $3e6e77db, $aed16a4a, $d9d65adc, $40df0b66,
  $37d83bf0, $a9bcae53, $debb9ec5, $47b2cf7f, $30b5ffe9,
  $bdbdf21c, $cabac28a, $53b39330, $24b4a3a6, $bad03605,
  $cdd70693, $54de5729, $23d967bf, $b3667a2e, $c4614ab8,
  $5d681b02, $2a6f2b94, $b40bbe37, $c30c8ea1, $5a05df1b,
  $2d02ef8d);

function crc32(crc: cardinal; buf: Pbyte; len: cardinal): cardinal;
begin
  if buf = nil then
    exit(0);

  crc := crc xor $FFFFFFFF;
  while (len >= 8) do
  begin
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    dec(len, 8);
  end;

  while (len > 0) do
  begin
    crc := crc32_table[(crc xor buf^) and $ff] xor (crc shr 8);
    inc(buf);
    dec(len);
  end;

  result := crc xor $FFFFFFFF;
end;

function ModeSwitchToInt(ms: TModeSwitch): byte;
begin
  case ms of
    msNone: Result:=0;
    msFpc: Result:=1;
    msObjfpc: Result:=2;
    msDelphi: Result:=3;
    msDelphiUnicode: Result:=4;
    msTP7: Result:=5;
    msMac: Result:=6;
    msIso: Result:=7;
    msExtpas: Result:=8;
    msGPC: Result:=9;
    msClass: Result:=10;
    msObjpas: Result:=11;
    msResult: Result:=12;
    msStringPchar: Result:=13;
    msCVarSupport: Result:=14;
    msNestedComment: Result:=15;
    msTPProcVar: Result:=16;
    msMacProcVar: Result:=17;
    msRepeatForward: Result:=18;
    msPointer2Procedure: Result:=19;
    msAutoDeref: Result:=20;
    msInitFinal: Result:=21;
    msDefaultAnsistring: Result:=22;
    msOut: Result:=23;
    msDefaultPara: Result:=24;
    msHintDirective: Result:=25;
    msDuplicateNames: Result:=26;
    msProperty: Result:=27;
    msDefaultInline: Result:=28;
    msExcept: Result:=29;
    msObjectiveC1: Result:=30;
    msObjectiveC2: Result:=31;
    msNestedProcVars: Result:=32;
    msNonLocalGoto: Result:=33;
    msAdvancedRecords: Result:=34;
    msISOLikeUnaryMinus: Result:=35;
    msSystemCodePage: Result:=36;
    msFinalFields: Result:=37;
    msDefaultUnicodestring: Result:=38;
    msTypeHelpers: Result:=39;
    msCBlocks: Result:=40;
    msISOLikeIO: Result:=41;
    msISOLikeProgramsPara: Result:=42;
    msISOLikeMod: Result:=43;
    msExternalClass: Result:=44;
    msPrefixedAttributes: Result:=45;
    // msIgnoreInterfaces: Result:=46;
    // msIgnoreAttributes: Result:=47;
    msOmitRTTI: Result:=48;
    msMultiHelpers: Result:=49;
    msImplicitFunctionSpec: Result:=50;
    msMultiLineStrings: Result:=51;
  end;
end;

function StrToPasIdentifierKind(const s: string): TPasIdentifierKind;
var
  Kind: TPasIdentifierKind;
begin
  for Kind in TPasIdentifierKind do
    if s=PCUIdentifierKindNames[Kind] then
      exit(Kind);
  Result:=pikNone;
end;

procedure WriteJSON(aData: TJSONData; TargetStream: TStream; Compressed: boolean
  );
var
  CurIndent: integer;
  Spaces: string;

  procedure WriteString(const s: string);
  begin
    if s='' then exit;
    TargetStream.Write(s[1],length(s));
  end;

  procedure WriteChar(const c: char);
  begin
    TargetStream.Write(c,1);
  end;

  procedure WriteLine;
  begin
    WriteString(sLineBreak);
    if CurIndent>0 then
      TargetStream.Write(Spaces[1],CurIndent);
  end;

  procedure Indent;
  begin
    if Compressed then exit;
    inc(CurIndent,2);
    if CurIndent>length(Spaces) then
      Spaces:=Spaces+'  ';
  end;

  procedure Unindent;
  begin
    if Compressed then exit;
    dec(CurIndent,2);
  end;

  procedure WriteData(Data: TJSONData); forward;

  procedure WriteObj(Obj: TJSONObject);
  var
    i: Integer;
    Name: String;
  begin
    WriteChar('{');
    if not Compressed then
      begin
      Indent;
      WriteLine;
      end;
    for i:=0 to Obj.Count-1 do
      begin
      if i>0 then
        begin
        WriteChar(',');
        if not Compressed then
          WriteLine;
        end;
      Name:=Obj.Names[i];
      WriteChar('"');
      if IsValidIdent(Name) then
        WriteString(Name)
      else
        WriteString(StringToJSONString(Name,false));
      WriteString('":');
      WriteData(Obj.Elements[Name]);
      end;
    if not Compressed then
      begin
      Unindent;
      WriteLine;
      end;
    WriteChar('}');
  end;

  procedure WriteArray(Arr: TJSONArray);
  var
    i: Integer;
  begin
    WriteChar('[');
    if not Compressed then
      begin
      Indent;
      WriteLine;
      end;
    for i:=0 to Arr.Count-1 do
      begin
      if i>0 then
        begin
        WriteChar(',');
        if not Compressed then
          WriteLine;
        end;
      WriteData(Arr[i]);
      end;
    if not Compressed then
      begin
      Unindent;
      WriteLine;
      end;
    WriteChar(']');
  end;

  procedure WriteData(Data: TJSONData);
  var
    C: TClass;
  begin
    C:=Data.ClassType;
    if C=TJSONObject then
      WriteObj(TJSONObject(Data))
    else if C=TJSONArray then
      WriteArray(TJSONArray(Data))
    else if C.InheritsFrom(TJSONNumber)
        or (C=TJSONBoolean)
      then
      WriteString(Data.AsString)
    else if (C=TJSONNull) then
      WriteString('null')
    else if C=TJSONString then
      begin
      WriteChar('"');
      WriteString(StringToJSONString(Data.AsString));
      WriteChar('"');
      end
    else
      raise EPas2JsWriteError.Create('unknown JSON data '+GetObjName(Data));
  end;

begin
  CurIndent:=0;
  WriteData(aData);
end;

procedure GrowIdToRefsArray(var IdToRefsArray: TPCUFilerElementRefArray; Id: integer);
var
  OldCapacity, NewCapacity: Integer;
begin
  OldCapacity:=length(IdToRefsArray);
  if Id>=OldCapacity then
    begin
    // grow
    NewCapacity:=OldCapacity;
    if NewCapacity=0 then NewCapacity:=100;
    while NewCapacity<Id+1 do NewCapacity:=NewCapacity*2;
    SetLength(IdToRefsArray,NewCapacity);
    FillByte(IdToRefsArray[OldCapacity],SizeOf(Pointer)*(NewCapacity-OldCapacity),0);
    end;
end;

function dbgmem(const s: string): string;
begin
  if s='' then exit('');
  Result:=dbgmem(PChar(s),length(s));
end;

function dbgmem(p: PChar; Cnt: integer): string;

  procedure AddLine(const Line: string);
  begin
    if Result<>'' then
      Result:=Result+LineEnding;
    Result:=Result+Line;
  end;

var
  c: Char;
  IsTxt: boolean;
  Line: String;
  i: Integer;
begin
  Result:='';
  if (p=nil) or (Cnt<=0) then exit;
  Line:='';
  IsTxt:=false;
  for i:=0 to Cnt-1 do
    begin
    c:=p[i];
    if c in ['a'..'z','A'..'Z','_','/','0'..'9'] then
      begin
      if not IsTxt then
        begin
        Line:=Line+'''';
        IsTxt:=true;
        end;
      Line:=Line+c;
      end
    else
      begin
      if IsTxt then
        begin
        Line:=Line+'''';
        IsTxt:=false;
        end;
      Line:=Line+'#'+HexStr(ord(c),2);
      end;
    if length(Line)>78 then
      begin
      AddLine(Line);
      Line:='';
      end;
    end;
  if Line<>'' then
    AddLine(Line);
end;

{ TPCUCustomWriter }

constructor TPCUCustomWriter.Create;
begin
  inherited Create;
  FErrorClass:=EPas2JsWriteError;
end;

{ TPCUReaderPendingSpecialized }

destructor TPCUReaderPendingSpecialized.Destroy;
var
  i: Integer;
begin
  Obj:=nil;
  GenericEl:=nil;
  RefEl:=nil;
  if Params<>nil then
    begin
    for i:=0 to Params.Count-1 do
      TObject(Params[i]).Free;
    FreeAndNil(Params);
    end;
  inherited Destroy;
end;

{ TPCUCustomReader }

constructor TPCUCustomReader.Create;
begin
  inherited Create;
  FErrorClass:=EPas2JsReadError;
end;

function TPCUCustomReader.ReadCanContinue: boolean;
var
  Module: TPasModule;
  Section: TPasSection;
  Scope: TPas2JSSectionScope;
begin
  Result:=false;
  Module:=Resolver.RootElement;
  if Module=nil then exit(true); // not yet started
  Section:=Resolver.GetLastSection;
  if Section=nil then exit(true); // only header
  Scope:=Section.CustomData as TPas2JSSectionScope;
  if Scope.Finished then exit(false); // finished
  Result:=Section.PendingUsedIntf=nil;
end;

{ TPCUFilerElementRef }

procedure TPCUFilerElementRef.AddPending(Item: TPCUFilerPendingElRef);
begin
  Item.Next:=Pending;
  Pending:=Item;
end;

procedure TPCUFilerElementRef.Clear;
var
  Ref, NextRef: TPCUFilerPendingElRef;
begin
  Elements:=nil;
  Ref:=Pending;
  while Ref<>nil do
    begin
    NextRef:=Ref.Next;
    Ref.Next:=nil;
    Ref.Free;
    Ref:=NextRef;
    end;
  Pending:=nil;
end;

destructor TPCUFilerElementRef.Destroy;
begin
  Clear;
  inherited Destroy;
end;

{ TPCUFiler }

function TPCUFiler.GetSourceFiles(Index: integer): TPCUSourceFile;
begin
  Result:=TPCUSourceFile(FSourceFiles[Index]);
end;

procedure TPCUFiler.RaiseMsg(Id: int64; const Msg: string);
var
  E: EPas2JsFilerError;
begin
  E:=ErrorClass.Create('['+IntToStr(Id)+'] '+Msg);
  E.Owner:=Self;
  {$IFDEF VerbosePCUFiler}
  writeln(ClassName+'/TPCUFiler.RaiseMsg ',E.Message);
  {$ENDIF}
  raise E;
end;

procedure TPCUFiler.RaiseMsg(Id: int64; El: TPasElement; const Msg: string);
var
  Path, s: String;
  CurEl: TPasElement;
begin
  Path:='';
  CurEl:=El;
  while CurEl<>nil do
    begin
    if Path<>'' then Path:='.'+Path;
    s:=CurEl.Name;
    if s='' then
      s:=CurEl.ClassName;
    Path:=s+Path;
    CurEl:=CurEl.Parent;
    end;
  s:=Path+': '+Msg;
  if El.GetModule<>Resolver.RootElement then
    s:='This='+Resolver.RootElement.Name+' El='+s;
  RaiseMsg(Id,s);
end;

function TPCUFiler.GetDefaultMemberVisibility(El: TPasElement
  ): TPasMemberVisibility;
var
  aClass: TPasClassType;
begin
  if El=nil then ;
  Result:=visDefault;
  if El.Parent is TPasClassType then
    begin
    aClass:=TPasClassType(El.Parent);
    case aClass.ObjKind of
    okInterface: Result:=visPublic;
    end;
    end;
end;

function TPCUFiler.GetDefaultPasScopeVisibilityContext(Scope: TPasScope
  ): TPasElement;
var
  El: TPasElement;
begin
  El:=Scope.Element;
  if El is TPasMembersType then
    Result:=El
  else if El is TPasModule then
    Result:=El
  else if (Scope is TPasProcedureScope) and (El.Parent is TPasMembersType) then
    Result:=El.Parent
  else
    Result:=nil;
end;

procedure TPCUFiler.GetDefaultsPasIdentifierProps(El: TPasElement; out
  Kind: TPasIdentifierKind; out Name: string);
begin
  Kind:=PCUDefaultIdentifierKind;
  if El is TPasProcedure then
    Kind:=pikProc;
  Name:=El.Name;
end;

function TPCUFiler.GetDefaultClassScopeFlags(Scope: TPas2JSClassScope
  ): TPasClassScopeFlags;
begin
  if FFileVersion<2 then
    Result:=[]
  else
    Result:=[pcsfAncestorResolved];
  if Scope.AncestorScope<>nil then
    begin
    if pcsfPublished in Scope.AncestorScope.Flags then
      Include(Result,pcsfPublished);
    end;
end;

function TPCUFiler.GetDefaultProcModifiers(Proc: TPasProcedure
  ): TProcedureModifiers;
begin
  Result:=[];
  if Proc.Parent is TPasClassType then
    begin
    if TPasClassType(Proc.Parent).IsExternal then
      Include(Result,pmExternal);
    end;
end;

function TPCUFiler.GetDefaultProcTypeModifiers(ProcType: TPasProcedureType
  ): TProcTypeModifiers;
var
  Proc: TPasProcedure;
begin
  Result:=[];
  if ProcType.Parent is TPasProcedure then
    begin
    Proc:=TPasProcedure(ProcType.Parent);
    if Proc.Parent is TPasClassType then
      Include(Result,ptmOfObject);
    end;
end;

function TPCUFiler.GetDefaultExprHasEvalValue(Expr: TPasExpr): boolean;
var
  C: TClass;
begin
  C:=Expr.Parent.ClassType;
  if C.InheritsFrom(TPasExpr) then exit(false);
  if (C=TPasAliasType)
      or (C=TPasTypeAliasType)
      or (C=TPasPointerType)
      or (C=TPasProperty)
  then
    exit(false);
  C:=Expr.ClassType;
  if C=TArrayValues then exit(false);
  if C=TRecordValues then exit(false);
  Result:=not Resolver.ExprEvaluator.IsSimpleExpr(Expr);
end;

function TPCUFiler.GetSrcCheckSum(aFilename: string): TPCUSourceFileChecksum;
var
  p: PChar;
  Cnt: integer;
begin
  OnGetSrc(Self,aFilename,p,Cnt);
  Result:=ComputeChecksum(p,Cnt);
end;

function TPCUFiler.GetDefaultRefName(El: TPasElement): string;
var
  C: TClass;
begin
  Result:=El.Name;
  if Result<>'' then exit;
  // some elements without name can be referred to:
  C:=El.ClassType;
  if C=TInterfaceSection then
    Result:='Interface'
  else if C=TPasArrayType then
    Result:='Array' // anonymous array
  else if C.InheritsFrom(TPasProcedureType) and (El.Parent is TPasProcedure) then
    Result:='Type'
  else
    Result:='';
end;

function TPCUFiler.GetElementReference(El: TPasElement; AutoCreate: boolean
  ): TPCUFilerElementRef;
var
  Node: TAVLTreeNode;
  MyEl: TPasElement;
  IsBuiltIn: boolean;
begin
  {$IFDEF VerbosePCUFiler}
  //writeln('TPCUFiler.GetElementReference ',GetObjName(El));
  {$ENDIF}
  IsBuiltIn:=El.CustomData is TResElDataBuiltInSymbol;
  if IsBuiltIn then
    begin
    // built-in symbol -> redirect to symbol of this module
    MyEl:=Resolver.FindLocalBuiltInSymbol(El);
    if MyEl=nil then
      RaiseMsg(20180207121004,El,GetObjName(El.CustomData));
    El:=MyEl;
    end
  else if El is TPasUnresolvedSymbolRef then
    RaiseMsg(20180215190054,El,GetObjName(El));

  Node:=FElementRefs.FindKey(El,@CompareElWithPCUFilerElementRef);
  if Node<>nil then
    Result:=TPCUFilerElementRef(Node.Data)
  else if AutoCreate then
    begin
    Result:=CreateElementRef(El);
    if IsBuiltIn then
      AddedBuiltInRef(Result);
    end
  else
    Result:=nil;
end;

function TPCUFiler.CreateElementRef(El: TPasElement): TPCUFilerElementRef;
{$IFDEF MemCheck}
var
  Node: TAVLTreeNode;
{$ENDIF}
begin
  Result:=TPCUFilerElementRef.Create;
  Result.Element:=El;
  {$IFDEF MemCheck}
  Node:=FElementRefs.Add(Result);
  if Node<>FElementRefs.FindKey(El,@CompareElWithPCUFilerElementRef) then
    RaiseMsg(20180711222046,El);
  {$ELSE}
  FElementRefs.Add(Result);
  {$ENDIF}
end;

procedure TPCUFiler.AddedBuiltInRef(Ref: TPCUFilerElementRef);
begin
  if Ref=nil then ;
end;

constructor TPCUFiler.Create;
begin
  FFileVersion:=PCUVersion;
  FSourceFiles:=TObjectList.Create(true);
  FElementRefs:=TAVLTree.Create(@ComparePCUFilerElementRef);
  FElementRefs.SetNodeManager(TAVLTreeNodeMemManager.Create,true); // no shared manager, needed for multithreading
end;

destructor TPCUFiler.Destroy;
begin
  Clear;
  FreeAndNil(FSourceFiles);
  FreeAndNil(FElementRefs);
  inherited Destroy;
end;

procedure TPCUFiler.Clear;
begin
  FElementRefs.FreeAndClear;
  FSourceFiles.Clear;
  FResolver:=nil;
  FParser:=nil;
  FScanner:=nil;
end;

function TPCUFiler.SourceFileCount: integer;
begin
  Result:=FSourceFiles.Count;
end;

{ TPCUInitialFlags }

constructor TPCUInitialFlags.Create;
begin
  Clear;
end;

procedure TPCUInitialFlags.Clear;
begin
  ParserOptions:=PCUDefaultParserOptions;
  ModeSwitches:=PCUDefaultModeSwitches;
  BoolSwitches:=PCUDefaultBoolSwitches;
  ConverterOptions:=PCUDefaultConverterOptions;
  TargetPlatform:=PCUDefaultTargetPlatform;
  TargetProcessor:=PCUDefaultTargetProcessor;
end;

{ TPCUWriter }

procedure TPCUWriter.ResolvePendingElRefs(Ref: TPCUFilerElementRef);
var
  RefItem: TPCUFilerPendingElRef;
  RefObj: TPCUWriterPendingElRefObj;
  RefArr: TPCUWriterPendingElRefArray;
begin
  if Ref.Pending=nil then exit;
  // this element is referenced
  if Ref.Id=0 then
    CreateElReferenceId(Ref);
  // resolve all pending references
  while Ref.Pending<>nil do
    begin
    RefItem:=Ref.Pending;
    if RefItem is TPCUWriterPendingElRefObj then
      begin
      RefObj:=TPCUWriterPendingElRefObj(RefItem);
      RefObj.Obj.Add(RefObj.PropName,Ref.Id);
      end
    else if RefItem is TPCUWriterPendingElRefArray then
      begin
      RefArr:=TPCUWriterPendingElRefArray(RefItem);
      RefArr.Arr.Integers[RefArr.Index]:=Ref.Id;
      end
    else
      RaiseMsg(20180207113335,RefItem.ClassName);
    Ref.Pending:=RefItem.Next;
    RefItem.Next:=nil;
    RefItem.Free;
    end;
end;

function TPCUWriter.CheckElScope(El: TPasElement; NotNilId: int64;
  ScopeClass: TPasScopeClass): TPasScope;
var
  Data: TObject;
begin
  Data:=El.CustomData;
  if Data=nil then
    begin
    if NotNilId>0 then
      RaiseMsg(NotNilId);
    exit(nil);
    end;
  if Data.ClassType<>ScopeClass then
    RaiseMsg(20180206113601,'expected '+ScopeClass.ClassName+', but found '+Data.ClassName);
  Result:=TPasScope(Data);
  if Result.Element<>El then
    RaiseMsg(20180206113723,'El='+GetObjName(El)+' Scope.Element='+GetObjName(Result.Element));
  if Result.Owner<>Resolver then
    RaiseMsg(20180206113750,El,GetObjName(Result));
end;

procedure TPCUWriter.AddArrayFlag(Obj: TJSONObject; var Arr: TJSONArray;
  const ArrName, Flag: string; Enable: boolean);
begin
  if Arr=nil then
    begin
    Arr:=TJSONArray.Create;
    Obj.Add(ArrName,Arr);
    end;
  if Enable then
    Arr.Add(Flag)
  else
    Arr.Add('-'+Flag);
end;

procedure TPCUWriter.AddReferenceToArray(Arr: TJSONArray; El: TPasElement;
  WriteNull: boolean);
var
  Ref: TPCUFilerElementRef;
  Item: TPCUWriterPendingElRefArray;
begin
  if El=nil then
    begin
    if WriteNull then
      Arr.Add(CreateJSON);
    exit;
    end;
  Ref:=GetElementReference(El);
  if (Ref.Obj<>nil) and (Ref.Id=0) then
    CreateElReferenceId(Ref);
  Arr.Add(Ref.Id);
  if Ref.Id<>0 then
    exit;
  // Element was not yet written -> add a pending item to the queue
  Item:=TPCUWriterPendingElRefArray.Create;
  Item.ErrorEl:=El;
  Item.Arr:=Arr;
  Item.Index:=Arr.Count-1;
  Ref.AddPending(Item);
end;

procedure TPCUWriter.AddReferenceToObj(Obj: TJSONObject;
  const PropName: string; El: TPasElement; WriteNil: boolean);
var
  Ref: TPCUFilerElementRef;
  Item: TPCUWriterPendingElRefObj;
begin
  if El=nil then
    begin
    if WriteNil then
      Obj.Add(PropName,0);
    exit;
    end;
  Ref:=GetElementReference(El);
  if (Ref.Obj<>nil) and (Ref.Id=0) then
    CreateElReferenceId(Ref);
  if Ref.Id<>0 then
    Obj.Add(PropName,Ref.Id)
  else
    begin
    // Element was not yet written -> add a pending item to the queue
    Item:=TPCUWriterPendingElRefObj.Create;
    Item.ErrorEl:=El;
    Item.Obj:=Obj;
    Item.PropName:=PropName;
    Ref.AddPending(Item);
    end;
end;

procedure TPCUWriter.CreateAutoElReferenceId(Ref: TPCUFilerElementRef);
begin
  if Ref.Id<>0 then
    RaiseMsg(20180207114300,Ref.Element,IntToStr(Ref.Id));
  inc(FElementIdCounter);
  Ref.Id:=FElementIdCounter;
end;

procedure TPCUWriter.CreateElReferenceId(Ref: TPCUFilerElementRef);
begin
  CreateAutoElReferenceId(Ref);
  Ref.Obj.Add('Id',Ref.Id);
end;

function TPCUWriter.CreateElementRef(El: TPasElement): TPCUFilerElementRef;
begin
  Result:=inherited CreateElementRef(El);
  if IsExternalEl(El) then
    begin
    if FFirstNewExt=nil then
      FFirstNewExt:=Result
    else
      FLastNewExt.NextNewExt:=Result;
    FLastNewExt:=Result;
    {$IF defined(VerbosePCUFiler) or defined(VerbosePJUFiler) or defined(VerbosePas2JS)}
    if (El.Name='') and (GetDefaultRefName(El)='') then
      RaiseMsg(20180623091608,El);
    {$ENDIF}
    end;
end;

procedure TPCUWriter.AddedBuiltInRef(Ref: TPCUFilerElementRef);
var
  ModuleObj, Obj: TJSONObject;
  El: TPasElement;
  Data: TObject;
begin
  El:=Ref.Element;
  // add built-in symbol to BuiltIn array
  if El<>Resolver.FindLocalBuiltInSymbol(El) then
    RaiseMsg(20180207124914,El);
  if FBuiltInSymbolsArr=nil then
    begin
    ModuleObj:=JSON.Find('Module') as TJSONObject;
    FBuiltInSymbolsArr:=TJSONArray.Create;
    ModuleObj.Add(BuiltInNodeName,FBuiltInSymbolsArr);
    end;
  Obj:=TJSONObject.Create;
  FBuiltInSymbolsArr.Add(Obj);
  Obj.Add('Name',El.Name);
  // Ref.Id is written in ResolvePendingElRefs
  Data:=El.CustomData;
  if Data is TResElDataBuiltInProc then
    case TResElDataBuiltInProc(Data).BuiltIn of
    bfStrFunc: Obj.Add('Type','Func');
    end;
  Ref.Obj:=Obj;
  ResolvePendingElRefs(Ref);
end;

procedure TPCUWriter.WriteHeaderMagic(Obj: TJSONObject);
begin
  Obj.Add('FileType',PCUMagic);
end;

procedure TPCUWriter.WriteHeaderVersion(Obj: TJSONObject);
begin
  Obj.Add('Version',PCUVersion);
end;

procedure TPCUWriter.WriteGUID(Obj: TJSONObject);
begin
  Obj.Add('GUID',GUIDToString(GUID));
end;

procedure TPCUWriter.WriteInitialFlags(Obj: TJSONObject);
begin
  WriteParserOptions(Obj,'InitParserOpts',InitialFlags.ParserOptions,PCUDefaultParserOptions);
  WriteModeSwitches(Obj,'InitModeSwitches',InitialFlags.Modeswitches,PCUDefaultModeSwitches);
  WriteBoolSwitches(Obj,'InitBoolSwitches',InitialFlags.BoolSwitches,PCUDefaultBoolSwitches);
  WriteConverterOptions(Obj,'InitConverterOpts',InitialFlags.ConverterOptions,PCUDefaultConverterOptions);
  if InitialFlags.TargetPlatform<>PCUDefaultTargetPlatform then
    Obj.Add('TargetPlatform',PCUTargetPlatformNames[InitialFlags.TargetPlatform]);
  if InitialFlags.TargetProcessor<>PCUDefaultTargetProcessor then
    Obj.Add('TargetProcessor',PCUTargetProcessorNames[InitialFlags.TargetProcessor]);
  // ToDo: write initial flags: used defines, used macros
end;

procedure TPCUWriter.WriteFinalFlags(Obj: TJSONObject);
begin
  WriteParserOptions(Obj,'FinalParserOpts',Parser.Options,InitialFlags.ParserOptions);
  WriteModeSwitches(Obj,'FinalModeSwitches',Scanner.CurrentModeSwitches,InitialFlags.Modeswitches);
  WriteBoolSwitches(Obj,'FinalBoolSwitches',Scanner.CurrentBoolSwitches,InitialFlags.BoolSwitches);
  if InitialFlags.ConverterOptions<>Converter.Options then
    RaiseMsg(20180314185555,'InitialFlags='+dbgs(InitialFlags.ConverterOptions)+' Converter='+dbgs(Converter.Options));
  // ToDo: write final flags: used defines, used macros
end;

procedure TPCUWriter.WriteParserOptions(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TPOptions);
var
  Arr: TJSONArray;
  f: TPOption;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TPOptions do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUParserOptionNames[f],f in Value);
end;

procedure TPCUWriter.WriteModeSwitches(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TModeSwitches);
var
  Arr: TJSONArray;
  f: TModeSwitch;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TModeSwitch do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUModeSwitchNames[f],f in Value);
end;

procedure TPCUWriter.WriteBoolSwitches(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TBoolSwitches);
var
  Arr: TJSONArray;
  f: TBoolSwitch;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TBoolSwitch do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUBoolSwitchNames[f],f in Value);
end;

procedure TPCUWriter.WriteConverterOptions(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TPasToJsConverterOptions);
var
  Arr: TJSONArray;
  f: TPasToJsConverterOption;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TPasToJsConverterOption do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUConverterOptions[f],f in Value);
end;

procedure TPCUWriter.WriteSrcFiles(Obj: TJSONObject);
var
  CurFile: TPCUSourceFile;
  List: TFPList;
  i: Integer;
  SourcesArr: TJSONArray;
  Src: TJSONObject;
begin
  List:=TFPList.Create;
  try
    // get files from scanner
    for i:=0 to Scanner.Files.Count-1 do
      begin
      CurFile:=TPCUSourceFile.Create;
      CurFile.Index:=i;
      CurFile.Filename:=Scanner.Files[i];
      if i=0 then
        CurFile.FileType:=sftUnit
      else
        CurFile.FileType:=sftInclude;
      FSourceFiles.Add(CurFile);
      CurFile.Checksum:=GetSrcCheckSum(CurFile.Filename);
      List.Add(CurFile);
      end;

    // create FSourceFilesSorted
    List.Sort(@ComparePCUSrcFiles);
    SetLength(FSourceFilesSorted,List.Count);
    for i:=0 to List.Count-1 do
      FSourceFilesSorted[i]:=TPCUSourceFile(List[i]);

    // write
    SourcesArr:=TJSONArray.Create;
    Obj.Add('Sources',SourcesArr);
    for i:=0 to FSourceFiles.Count-1 do
      begin
      CurFile:=TPCUSourceFile(FSourceFiles[i]);
      Src:=TJSONObject.Create;
      SourcesArr.Add(Src);
      if (i=0) then
        // the first file is the unit source, no need to write Kind
      else if (CurFile.FileType=sftInclude) then
        // the default file type is include, no need to write Kind
      else
        Src.Add('Type',PCUSourceFileTypeNames[CurFile.FileType]);
      Src.Add('File',CurFile.Filename);
      Src.Add('CheckSum',CurFile.Checksum);
      end;
  finally
    List.Free;
  end;
end;

procedure TPCUWriter.WriteMemberHints(Obj: TJSONObject; const Value,
  DefaultValue: TPasMemberHints);
var
  Arr: TJSONArray;
  f: TPasMemberHint;
begin
  Arr:=nil;
  for f in TPasMemberHints do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,'Hints',PCUMemberHintNames[f],f in Value);
end;

procedure TPCUWriter.WriteVarModifiers(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TVariableModifiers);
var
  Arr: TJSONArray;
  f: TVariableModifier;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TVariableModifier do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUVarModifierNames[f],f in Value);
end;

procedure TPCUWriter.WritePasElement(Obj: TJSONObject; El: TPasElement;
  aContext: TPCUWriterContext);
var
  DefHints: TPasMemberHints;
  DefVisibility: TPasMemberVisibility;
  Ref: TPCUFilerElementRef;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUWriter.WritePasElement ',GetObjName(El));
  {$ENDIF}
  if El.Name<>'' then
    Obj.Add('Name',Resolver.GetOverloadName(El));

  // Id
  Ref:=GetElementReference(El);
  Ref.Obj:=Obj;
  ResolvePendingElRefs(Ref);

  WriteSrcPos(Obj,El,aContext);

  DefVisibility:=GetDefaultMemberVisibility(El);
  if El.Visibility<>DefVisibility then
    Obj.Add('Visibility',PCUMemberVisibilityNames[El.Visibility]);

  DefHints:=[];
  if El.Parent<>nil then
    DefHints:=El.Parent.Hints;
  WriteMemberHints(Obj,El.Hints,DefHints);

  if El.HintMessage<>'' then
    Obj.Add('HintMessage',El.HintMessage);

  // not needed El.DocComment
  if aContext<>nil then ;
end;

procedure TPCUWriter.WriteModuleScopeFlags(Obj: TJSONObject; const Value,
  DefaultValue: TPasModuleScopeFlags);
var
  Arr: TJSONArray;
  f: TPasModuleScopeFlag;
begin
  Arr:=nil;
  for f in TPasModuleScopeFlags do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,'ScopeFlags',PCUModuleScopeFlagNames[f],f in Value);
end;

procedure TPCUWriter.WriteModule(Obj: TJSONObject; aModule: TPasModule;
  aContext: TPCUWriterContext);

  procedure WSection(Section: TPasSection; const PropName: string);
  begin
    if Section=nil then exit;
    if Section.Parent<>aModule then
      RaiseMsg(20180205153912,aModule,PropName);
    aContext.Section:=Section; // set Section before calling virtual WriteSection
    aContext.SectionObj:=nil;
    aContext.IndirectUsesArr:=nil;
    WriteSection(Obj,Section,PropName,aContext);
  end;

  procedure WImplBlock(Block: TPasImplBlock; const PropPrefix: string);
  var
    Scope: TPas2JSInitialFinalizationScope;
    ImplJS: TPas2JSPrecompiledJS;
    Sub: TJSONObject;
  begin
    if Block=nil then exit;
    Scope:=Block.CustomData as TPas2JSInitialFinalizationScope;
    ImplJS:=Scope.ImplJS;
    Sub:=TJSONObject.Create;
    Obj.Add(PropPrefix,Sub);
    WriteScopeReferences(Sub,Scope.References,'Refs',aContext);
    WritePrecompiledJS(Sub,Block,ImplJS,aContext);
  end;

  procedure RaisePending(Ref: TPCUFilerElementRef);
  {$IF defined(VerbosePJUFiler) or defined(VerbosePCUFiler) or defined(VerboseUnitQueue)}
  var
    PendObj: TPCUWriterPendingElRefObj;
    PendArr: TPCUWriterPendingElRefArray;
  {$ENDIF}
  begin
    {$IF defined(VerbosePJUFiler) or defined(VerbosePCUFiler) or defined(VerboseUnitQueue)}
    {AllowWriteln}
    writeln('TPCUWriter.WriteModule Ref.Element=',GetElementDbgPath(Ref.Element),' Pending=',GetObjName(Ref.Pending),' ErrorEl=',GetElementDbgPath(Ref.Pending.ErrorEl));
    if Ref.Pending is TPCUWriterPendingElRefObj then
      begin
      PendObj:=TPCUWriterPendingElRefObj(Ref.Pending);
      writeln('  Obj=',PendObj.Obj<>nil,' PropName=',PendObj.PropName);
      end
    else if Ref.Pending is TPCUWriterPendingElRefArray then
      begin
      PendArr:=TPCUWriterPendingElRefArray(Ref.Pending);
      writeln('  Arr=',PendArr.Arr<>nil,' Index=',PendArr.Index);
      end;
    {AllowWriteln-}
    {$ENDIF}
    RaiseMsg(20180318225558,Ref.Element,GetObjName(Ref.Pending));
  end;

var
  ModScope: TPas2JSModuleScope;
  Node: TAVLTreeNode;
  Ref: TPCUFilerElementRef;
begin
  FInImplementation:=false;
  WritePasElement(Obj,aModule,aContext);

  if aModule.ClassType=TPasModule then
    Obj.Add('Type','Unit')
  else if aModule.ClassType=TPasProgram then
    Obj.Add('Type','Program')
  else if aModule.ClassType=TPasLibrary then
    Obj.Add('Type','Library')
  else
    RaiseMsg(20180203163923);

  // module scope
  ModScope:=TPas2JSModuleScope(CheckElScope(aModule,20180206113855,TPas2JSModuleScope));
  WriteModuleScope(Obj,ModScope,aContext);

  FBuiltInSymbolsArr:=TJSONArray.Create;
  Obj.Add(BuiltInNodeName,FBuiltInSymbolsArr);

  // write sections
  if aModule.ClassType=TPasProgram then
    begin
    WSection(TPasProgram(aModule).ProgramSection,'Program');
    WImplBlock(aModule.InitializationSection,'begin');
    end
  else if aModule.ClassType=TPasLibrary then
    begin
    WSection(TPasLibrary(aModule).LibrarySection,'Library');
    WImplBlock(aModule.InitializationSection,'begin');
    end
  else
    begin
    WSection(aModule.InterfaceSection,'Interface');
    FInImplementation:=true;
    WSection(aModule.ImplementationSection,'Implementation');
    WImplBlock(aModule.InitializationSection,'Init');
    WImplBlock(aModule.FinalizationSection,'Final');
    end;

  WriteModuleScopeLocalVars(Obj,ModScope);

  //writeln('TPCUWriter.WriteModule WriteExternalReferences of implementation ',Resolver.RootElement.Name,' aContext.Section=',GetObjName(aContext.Section));
  WriteExternalReferences(aContext);

  if FBuiltInSymbolsArr.Count=0 then
    begin
    // remove empty BuiltIn symbols array
    Obj.Remove(FBuiltInSymbolsArr);
    FBuiltInSymbolsArr:=nil;
    end;

  // consistency check
  Node:=FElementRefs.FindLowest;
  while Node<>nil do
    begin
    Ref:=TPCUFilerElementRef(Node.Data);
    if Ref.Pending<>nil then
      RaisePending(Ref);
    Node:=FElementRefs.FindSuccessor(Node);
    end;
end;

procedure TPCUWriter.WritePasScope(Obj: TJSONObject; Scope: TPasScope;
  aContext: TPCUWriterContext);
var
  DefVisibilityContext: TPasElement;
begin
  if aContext=nil then ;
  DefVisibilityContext:=GetDefaultPasScopeVisibilityContext(Scope);
  if Scope.VisibilityContext<>DefVisibilityContext then
    AddReferenceToObj(Obj,'VisibilityContext',Scope.VisibilityContext,true);
end;

procedure TPCUWriter.WriteIdentifierScope(Obj: TJSONObject;
  Scope: TPasIdentifierScope; aContext: TPCUWriterContext);
var
  Arr: TJSONArray;

  procedure WriteItem(Item: TPasIdentifier);
  var
    DefKind: TPasIdentifierKind;
    DefName: string;
    Sub: TJSONObject;
  begin
    GetDefaultsPasIdentifierProps(Item.Element,DefKind,DefName);
    if (Item.Kind=DefKind) and (Item.Identifier=DefName) then
    begin
      // add the element Id
      AddReferenceToArray(Arr,Item.Element);
    end
    else begin
      // add a json object
      Sub:=TJSONObject.Create;
      Arr.Add(Sub);
      if Item.Kind<>DefKind then
        Sub.Add('Kind',PCUIdentifierKindNames[Item.Kind]);
      if Item.Identifier<>DefName then
        Sub.Add('Name',Item.Identifier);
      AddReferenceToObj(Sub,'El',Item.Element);
    end;
  end;

var
  Locals: TFPList;
  i, p: Integer;
  Item: TPasIdentifier;
  Ordered: TPasIdentifierArray;
begin
  WritePasScope(Obj,Scope,aContext);
  Arr:=nil;
  if aContext=nil then ;
  Locals:=Scope.GetLocalIdentifiers;
  try
    p:=0;
    Ordered:=nil;
    for i:=0 to Locals.Count-1 do
      begin
      if Arr=nil then
        begin
        Arr:=TJSONArray.Create;
        Obj.Add('SItems',Arr);
        end;
      Item:=TPasIdentifier(Locals[i]);
      if Item.NextSameIdentifier=nil then
        WriteItem(Item)
      else
        begin
        // write in declaration order (i.e. reverse)
        p:=0;
        while Item<>nil do
          begin
          if length(Ordered)<=p then
            SetLength(Ordered,length(Ordered)+4);
          Ordered[p]:=Item;
          inc(p);
          Item:=Item.NextSameIdentifier;
          end;
        while p>0 do
          begin
          dec(p);
          WriteItem(Ordered[p]);
          end;
        end;
      end;
  finally
    Locals.Free;
  end;
end;

procedure TPCUWriter.WriteModuleScope(Obj: TJSONObject;
  Scope: TPas2JSModuleScope; aContext: TPCUWriterContext);
var
  aModule: TPasModule;
begin
  aModule:=Scope.Element as TPasModule;
  if Scope.FirstName<>FirstDottedIdentifier(aModule.Name) then
    RaiseMsg(20180206114233,aModule);
  // write not needed: Scope.FirstName
  WriteModuleScopeFlags(Obj,Scope.Flags,PCUDefaultModuleScopeFlags);
  WriteBoolSwitches(Obj,'BoolSwitches',Scope.BoolSwitches,aContext.BoolSwitches);
  AddReferenceToObj(Obj,'AssertClass',Scope.AssertClass);
  AddReferenceToObj(Obj,'AssertDefConstructor',Scope.AssertDefConstructor);
  AddReferenceToObj(Obj,'AssertMsgConstructor',Scope.AssertMsgConstructor);
  AddReferenceToObj(Obj,'RangeErrorClass',Scope.RangeErrorClass);
  AddReferenceToObj(Obj,'RangeErrorConstructor',Scope.RangeErrorConstructor);
  AddReferenceToObj(Obj,'SystemTVarRec',Scope.SystemTVarRec);
  AddReferenceToObj(Obj,'SystemVarRecs',Scope.SystemVarRecs);

  // Scope.StoreJSLocalVars is written later, because some references need implementation units

  WritePasScope(Obj,Scope,aContext);
end;

procedure TPCUWriter.WriteModuleScopeLocalVars(Obj: TJSONObject;
  Scope: TPas2JSModuleScope);
var
  LocalVars: TPas2JSStoredLocalVarArray;
  SubObj: TJSONObject;
  i: Integer;
  LocalVar: TPas2JSStoredLocalVar;
begin
  // StoreJSLocalVars
  LocalVars:=Scope.StoreJSLocalVars;
  if length(LocalVars)>0 then
    begin
    SubObj:=TJSONObject.Create;
    Obj.Add('LocalVars',SubObj);
    for i:=0 to length(LocalVars)-1 do
      begin
      LocalVar:=LocalVars[i];
      if LocalVar.Name='' then
        RaiseMsg(20201023013605,Scope.Element,GetObjPath(LocalVar.Element));
      if LocalVar.Element=nil then
        RaiseMsg(20201023013954,Scope.Element,LocalVar.Name);
      AddReferenceToObj(SubObj,LocalVar.Name,LocalVar.Element);
      end;
    end;
end;

procedure TPCUWriter.WriteSrcPos(Obj: TJSONObject; El: TPasElement;
  aContext: TPCUWriterContext);
var
  LastLine, LastCol, i, CurLine, CurCol: Integer;
  s: String;
begin
  if aContext=nil then ;
  if (El.Parent=nil) or (El.Parent.SourceFilename<>El.SourceFilename) then
    begin
    if El.SourceFilename<>'' then
      begin
      i:=IndexOfSourceFile(El.SourceFilename);
      if i<0 then
        RaiseMsg(20180205110259,El,El.SourceFilename);
      end
    else
      i:=-1;
    Obj.Add('File',i);
    end;

  if El.Parent=nil then
    begin
    LastLine:=1;
    LastCol:=1;
    end
  else
    Resolver.UnmangleSourceLineNumber(El.Parent.SourceLinenumber,LastLine,LastCol);
  Resolver.UnmangleSourceLineNumber(El.SourceLinenumber,CurLine,CurCol);
  s:='';
  if LastLine<>CurLine then
    s:=IntToStr(CurLine);
  if LastCol<>CurCol then
    s:=s+','+IntToStr(CurCol);
  if s<>'' then
    Obj.Add('Pos',s);
  // not needed: El.SourceEndLinenumber
end;

procedure TPCUWriter.WriteSection(ParentJSON: TJSONObject;
  Section: TPasSection; const PropName: string; aContext: TPCUWriterContext);
var
  Obj, SubObj: TJSONObject;
  Scope, UsesScope: TPas2JSSectionScope;
  i, j: Integer;
  Arr: TJSONArray;
  UsesUnit: TPasUsesUnit;
  Name, InFilename: String;
  Ref: TPCUFilerElementRef;
begin
  if Section=nil then exit;
  Obj:=TJSONObject.Create;
  ParentJSON.Add(PropName,Obj);
  aContext.SectionObj:=Obj;
  aContext.IndirectUsesArr:=nil;
  WritePasElement(Obj,Section,aContext);

  Scope:=TPas2JSSectionScope(CheckElScope(Section,20180206121825,TPas2JSSectionScope));
  if not Scope.Finished then
    RaiseMsg(20180206130333,Section);

  WriteBoolSwitches(Obj,'BoolSwitches',Scope.BoolSwitches,aContext.BoolSwitches);
  aContext.BoolSwitches:=Scope.BoolSwitches;
  WriteModeSwitches(Obj,'ModeSwitches',Scope.ModeSwitches,aContext.ModeSwitches);
  aContext.ModeSwitches:=Scope.ModeSwitches;

  if Scope.UsesScopes.Count<>length(Section.UsesClause) then
    RaiseMsg(20180206122222,Section);
  Arr:=nil;
  for i:=0 to Scope.UsesScopes.Count-1 do
    begin
    UsesUnit:=Section.UsesClause[i];
    UsesScope:=TPas2JSSectionScope(Scope.UsesScopes[i]);
    if UsesScope.Element<>TPasModule(UsesUnit.Module).InterfaceSection then
      RaiseMsg(20180206122459,Section,'usesscope '+IntToStr(i)+' UsesScope.Element='+GetObjName(UsesScope.Element)+' Module='+GetObjName(Section.UsesClause[i].Module));
    if Arr=nil then
      begin
      Arr:=TJSONArray.Create;
      Obj.Add('Uses',Arr);
      end;
    SubObj:=TJSONObject.Create;
    Arr.Add(SubObj);
    if UsesUnit.Expr<>nil then
      Name:=DotExprToName(UsesUnit.Expr)
    else
      begin
      // implicit unit, e.g. system
      Name:=UsesUnit.Module.Name;
      for j:=0 to Parser.ImplicitUses.Count-1 do
        if CompareText(Parser.ImplicitUses[i],Name)=0 then
          begin
          Name:=Parser.ImplicitUses[i];
          break;
          end;
      end;
    if Name='' then
      RaiseMsg(20180307091654,UsesUnit.Expr);
    SubObj.Add('Name',Name);
    if UsesUnit.InFilename<>nil then
      begin
      InFilename:=Resolver.GetUsesUnitInFilename(UsesUnit.InFilename);
      if InFilename='' then
        RaiseMsg(20180307094723,UsesUnit.InFilename);
      SubObj.Add('In',InFilename);
      end;
    if CompareText(UsesUnit.Module.Name,Name)<>0 then
      SubObj.Add('UnitName',UsesUnit.Module.Name);
    // ref object for uses
    Ref:=GetElementReference(UsesUnit);
    Ref.Obj:=SubObj;
    if OnIsElementUsed(Self,UsesUnit.Module) then
      begin
      // ref object for module
      Ref:=GetElementReference(UsesUnit.Module);
      if Ref.Obj=nil then
        begin
        Ref.Obj:=TJSONObject.Create;
        SubObj.Add('Module',Ref.Obj);
        end;
      end;
    end;
  WriteIdentifierScope(Obj,Scope,aContext);

  // not needed: Scope ElevatedLocals
  // not needed: Scope Helpers
  if (length(Scope.Helpers)>0) and not (Scope.Element is TInterfaceSection) then
    RaiseMsg(20190119122007,Section);

  WriteDeclarations(Obj,Section,aContext);
  if Section is TInterfaceSection then
    begin
    if aContext.SectionObj<>Obj then
      RaiseMsg(20180318112544,Section);
    {$IFDEF VerbosePJUFiler}
    //writeln('TPCUWriter.WriteSection WriteExternalReferences of Interface ',GetElementFullPath(Section));
    {$ENDIF}
    WriteExternalReferences(aContext);
    end;
end;

procedure TPCUWriter.WriteDeclarations(ParentJSON: TJSONObject;
  Decls: TPasDeclarations; aContext: TPCUWriterContext);
var
  i: Integer;
  Decl: TPasElement;
  Arr: TJSONArray;
  DeclObj: TJSONObject;
begin
  Arr:=nil;
  for i:=0 to Decls.Declarations.Count-1 do
    begin
    Decl:=TPasElement(Decls.Declarations[i]);
    if Decl.Parent<>Decls then
      RaiseMsg(20180208221915,Decl,'['+IntToStr(i)+']='+GetObjName(Decl)+': '+GetObjName(Decls)+'<>'+GetObjName(Decl.Parent));
    if IsExternalEl(Decl) then
      continue; // e.g. specialization
    if Arr=nil then
      begin
      Arr:=TJSONArray.Create;
      ParentJSON.Add('Declarations',Arr);
      end;
    DeclObj:=TJSONObject.Create;
    Arr.Add(DeclObj);
    WriteElement(DeclObj,Decl,aContext);
    end;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUWriter.WriteDeclarations END ',GetObjName(Decls));
  {$ENDIF}
end;

procedure TPCUWriter.WriteElementProperty(Obj: TJSONObject;
  Parent: TPasElement; const PropName: string; El: TPasElement;
  aContext: TPCUWriterContext);
var
  SubObj: TJSONObject;
begin
  if El=nil then exit;
  if (Parent<>El.Parent) then
    RaiseMsg(20180208221751,El,PropName+': '+GetObjName(Parent)+'<>'+GetObjName(El.Parent));
  SubObj:=TJSONObject.Create;
  Obj.Add(PropName,SubObj);
  WriteElement(SubObj,El,aContext);
end;

procedure TPCUWriter.WriteElementList(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; ListOfElements: TFPList; aContext: TPCUWriterContext;
  ReferencesAllowed: boolean);
var
  Arr: TJSONArray;
  i: Integer;
  SubObj: TJSONObject;
  Item: TPasElement;
begin
  if (ListOfElements=nil) or (ListOfElements.Count=0) then exit;
  Arr:=TJSONArray.Create;
  Obj.Add(PropName,Arr);
  for i:=0 to ListOfElements.Count-1 do
    begin
    Item:=TPasElement(ListOfElements[i]);
    if Item.Parent<>Parent then
      begin
      if not ReferencesAllowed then
        RaiseMsg(20180209191444,Item,GetObjName(Parent)+'<>'+GetObjName(Item.Parent));
      AddReferenceToArray(Arr,Item);
      end
    else
      begin
      SubObj:=TJSONObject.Create;
      Arr.Add(SubObj);
      WriteElement(SubObj,Item,aContext);
      end;
    end;
end;

procedure TPCUWriter.WriteElementArray(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; ArrOfElements: TPasElementArray;
  aContext: TPCUWriterContext; ReferencesAllowed: boolean);
var
  Arr: TJSONArray;
  i: Integer;
  SubObj: TJSONObject;
  Item: TPasElement;
begin
  if length(ArrOfElements)=0 then exit;
  Arr:=TJSONArray.Create;
  Obj.Add(PropName,Arr);
  for i:=0 to length(ArrOfElements)-1 do
    begin
    Item:=ArrOfElements[i];
    if Item.Parent<>Parent then
      begin
      if not ReferencesAllowed then
        RaiseMsg(20180209191444,Item,GetObjName(Parent)+'<>'+GetObjName(Item.Parent));
      AddReferenceToArray(Arr,Item);
      end
    else
      begin
      SubObj:=TJSONObject.Create;
      Arr.Add(SubObj);
      WriteElement(SubObj,Item,aContext);
      end;
    end;
end;

procedure TPCUWriter.WriteElType(Obj: TJSONObject; El: TPasElement;
  const PropName: string; aType: TPasType; aContext: TPCUWriterContext);
begin
  if aType=nil then exit;
  if (aType.Name='') {or (aType.Parent=El)} then
    begin
    // anonymous type
    WriteElementProperty(Obj,El,PropName,aType,aContext);
    end
  else
    begin
    // reference
    AddReferenceToObj(Obj,PropName,aType);
    end;
end;

procedure TPCUWriter.WriteStrings(Obj: TJSONObject; const PropName: string;
  aList: TStrings; aContext: TPCUWriterContext);
var
  Arr: TJSONArray;
  i: Integer;
begin
  if (aList=nil) or (aList.Count=0) then exit;
  Arr:=TJSONArray.Create;
  Obj.Add(PropName,Arr);
  for i:=0 to aList.Count-1 do
    Arr.Add(aList[i]);
  if aContext=nil then ;
end;

procedure TPCUWriter.WriteResolvedRefFlags(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TResolvedReferenceFlags);
var
  Arr: TJSONArray;
  f: TResolvedReferenceFlag;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TResolvedReferenceFlag do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUResolvedReferenceFlagNames[f],f in Value);
end;

procedure TPCUWriter.WriteResolvedReference(Obj: TJSONObject;
  Ref: TResolvedReference; ErrorEl: TPasElement);
var
  Ctx: TResolvedRefContext;
  WithExprScope: TPasWithExprScope;
begin
  WriteResolvedRefFlags(Obj,'RefFlags',Ref.Flags,[]);
  if Ref.Access<>rraRead then
    Obj.Add('RefAccess',PCUResolvedRefAccessNames[Ref.Access]);
  if Ref.WithExprScope<>nil then
    RaiseMsg(20180215132828,ErrorEl);
  if Ref.Context<>nil then
    begin
    Ctx:=Ref.Context;
    if Ctx.ClassType=TResolvedRefCtxConstructor then
      begin
      if TResolvedRefCtxConstructor(Ctx).Typ=nil then
        RaiseMsg(20190222011342,ErrorEl);
      AddReferenceToObj(Obj,'RefConstructorType',TResolvedRefCtxConstructor(Ctx).Typ);
      end
    else if Ctx.ClassType=TResolvedRefCtxAttrProc then
      begin
      if TResolvedRefCtxAttrProc(Ctx).Proc=nil then
        RaiseMsg(20190222011427,ErrorEl);
      AddReferenceToObj(Obj,'RefAttrProc',TResolvedRefCtxAttrProc(Ctx).Proc);
      end
    else
      RaiseMsg(20180215132849,ErrorEl,GetObjName(Ref.Context));
    end;
  AddReferenceToObj(Obj,'RefDecl',Ref.Declaration);
  WithExprScope:=Ref.WithExprScope;
  if WithExprScope<>nil then
    begin
    RaiseMsg(20200113182413,ErrorEl);
    {$IFDEF EnableStoreExprRef}
    AddReferenceToObj(Obj,'WithEl',WithExprScope.WithScope.Element);
    if WithExprScope.Index>0 then
      AddReferenceToObj(Obj,'WithId',WithExprScope.Index);
    {$ENDIF}
    end;
end;

procedure TPCUWriter.WriteExprCustomData(Obj: TJSONObject; Expr: TPasExpr;
  aContext: TPCUWriterContext);

  procedure CheckNext(Data: TObject);
  var
    Value: TResEvalValue;
    DefHasEvalValue: Boolean;
  begin
    DefHasEvalValue:=GetDefaultExprHasEvalValue(Expr);
    //writeln('TPCUWriter.WriteExprCustomData.CheckNext Expr=',GetObjName(Expr),' Parent=',GetObjName(Expr.Parent),' Def=',DefHasEvalValue,' Data=',GetObjName(Data));
    if Data=nil then
      begin
      if DefHasEvalValue then
        Obj.Add('Eval',false);
      end
    else if Data is TResEvalValue then
      begin
      Value:=TResEvalValue(Data);
      if not DefHasEvalValue then
        Obj.Add('Eval',true);
      // value is not stored
      if Value.CustomData<>nil then
        RaiseMsg(20180215143045,Expr,GetObjName(Data));
      end
    else
      RaiseMsg(20180215143108,Expr,GetObjName(Data));
  end;

var
  Ref: TResolvedReference;
begin
  if aContext.InGeneric then
    exit;// not needed by generic code
  if Expr.CustomData is TResolvedReference then
    begin
    Ref:=TResolvedReference(Expr.CustomData);
    WriteResolvedReference(Obj,Ref,Expr);
    CheckNext(Ref.CustomData);
    end
  else
    CheckNext(Expr.CustomData);
  if aContext<>nil then ;
end;

procedure TPCUWriter.WriteExpr(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; Expr: TPasExpr; aContext: TPCUWriterContext);
var
  SubObj: TJSONObject;
begin
  if Expr=nil then exit;
  if Parent<>Expr.Parent then
    RaiseMsg(20180208221051,Parent,PropName+' Expr='+GetObjName(Expr)+' Parent='+GetObjName(Parent)+'<>'+GetObjName(Expr.Parent)+'=Expr.Parent');
  // ToDo: write simple expressions in a compact format
  SubObj:=TJSONObject.Create;
  Obj.Add(PropName,SubObj);
  WriteElement(SubObj,Expr,aContext);
  WriteExprCustomData(SubObj,Expr,aContext);
end;

procedure TPCUWriter.WritePasExpr(Obj: TJSONObject; Expr: TPasExpr;
  DefaultKind: TPasExprKind; DefaultOpCode: TExprOpCode;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,Expr,aContext);
  if Expr.Kind<>DefaultKind then
    Obj.Add('Kind',PCUExprKindNames[Expr.Kind]);
  if Expr.OpCode<>DefaultOpCode then
    Obj.Add('Op',PCUExprOpCodeNames[Expr.OpCode]);
  WriteExpr(Obj,Expr,'Format1',Expr.format1,aContext);
  WriteExpr(Obj,Expr,'Format2',Expr.format2,aContext);
end;

procedure TPCUWriter.WritePasExprArray(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; const ExprArr: TPasExprArray;
  aContext: TPCUWriterContext);
var
  Arr: TJSONArray;
  i: Integer;
  Expr: TPasExpr;
  SubObj: TJSONObject;
begin
  if length(ExprArr)=0 then exit;
  Arr:=TJSONArray.Create;
  Obj.Add(PropName,Arr);
  for i:=0 to length(ExprArr)-1 do
    begin
    Expr:=ExprArr[i];
    if Expr.Parent<>Parent then
      RaiseMsg(20180209191444,Expr,GetObjName(Parent)+'<>'+GetObjName(Expr.Parent));
    SubObj:=TJSONObject.Create;
    Arr.Add(SubObj);
    WriteElement(SubObj,Expr,aContext);
    WriteExprCustomData(SubObj,Expr,aContext);
    end;
end;

procedure TPCUWriter.WriteScopeReferences(Obj: TJSONObject;
  References: TPasScopeReferences; const PropName: string;
  aContext: TPCUWriterContext);
var
  Refs: TFPList;
  Arr: TJSONArray;
  i: Integer;
  PSRef: TPasScopeReference;
  SubObj: TJSONObject;
begin
  if References=nil then exit;
  Refs:=References.GetList;
  try
    if Refs.Count>0 then
      begin
      Arr:=TJSONArray.Create;
      Obj.Add(PropName,Arr);
      for i:=0 to Refs.Count-1 do
        begin
        PSRef:=TPasScopeReference(Refs[i]);
        SubObj:=TJSONObject.Create;
        Arr.Add(SubObj);
        if PSRef.Access<>PCUDefaultPSRefAccess then
          SubObj.Add('Access',PCUPSRefAccessNames[PSRef.Access]);
        AddReferenceToObj(SubObj,'Id',PSRef.Element);
        end;
      end;
  finally
    Refs.Free;
  end;
  if aContext=nil then ;
end;

function TPCUWriter.IsExternalEl(El: TPasElement): boolean;
var
  C: TClass;
begin
  while El<>nil do
    begin
    C:=El.ClassType;
    if C.InheritsFrom(TPasModule) then
      exit(El<>Resolver.RootElement)
    else if C.InheritsFrom(TPasGenericType) then
      begin
      if Resolver.IsSpecialized(TPasGenericType(El)) then
        exit(true);
      end;
    El:=El.Parent;
    end;
end;

procedure TPCUWriter.WriteExtRefSignature(Ref: TPCUFilerElementRef;
  aContext: TPCUWriterContext);

  procedure WriteMemberIndex(Members: TFPList; Member: TPasElement; Obj: TJSONObject);
  var
    i, Index, j: Integer;
    CurEl: TPasElement;
    SpecItem: TPRSpecializedItem;
    Arr: TJSONArray;
    Param: TPasType;
  begin
    SpecItem:=nil;
    if (Member.CustomData is TPasGenericScope) then
      SpecItem:=TPasGenericScope(Member.CustomData).SpecializedFromItem;

    if SpecItem<>nil then
      begin
      Obj.Add('SpecName',SpecItem.SpecializedEl.Name);

      // write specialize params
      Arr:=TJSONArray.Create;
      Obj.Add('SpecParams',Arr);
      for i:=0 to length(SpecItem.Params)-1 do
        begin
        Param:=SpecItem.Params[i];
        if Param=nil then
          RaiseMsg(20200222110205,Member);
        AddReferenceToArray(Arr,Param);
        end;
      end
    else
      begin
      // write member index
      j:=0;
      Index:=-1;
      for i:=0 to Members.Count-1 do
        begin
        CurEl:=TPasElement(Members[i]);
        if CurEl=Member then
          begin
          Index:=j;
          break;
          end
        else if (CurEl is TPasGenericType)
            and Resolver.IsSpecialized(TPasGenericType(CurEl)) then
          // skip specialized type
        else
          inc(j);
        end;
      if Index<0 then
        RaiseMsg(20180309184111,Member);
      if Index>0 then
        Obj.Add('MId',Index);
      end;
  end;

var
  Parent, El: TPasElement;
  C: TClass;
begin
  //writeln('TPCUWriter.WriteExtRefSignature START ',GetObjName(Ref.Element));
  if aContext=nil then ;
  // write member index
  El:=Ref.Element;
  Parent:=El.Parent;
  C:=Parent.ClassType;
  if C.InheritsFrom(TPasDeclarations) then
    WriteMemberIndex(TPasDeclarations(Parent).Declarations,Ref.Element,Ref.Obj)
  else if (C=TPasClassType)
      or (C=TPasRecordType) then
    WriteMemberIndex(TPasMembersType(Parent).Members,Ref.Element,Ref.Obj)
  else if C=TPasEnumType then
    WriteMemberIndex(TPasEnumType(Parent).Values,Ref.Element,Ref.Obj)
  else if C.InheritsFrom(TPasModule) then
    begin
    if Ref.Element is TInterfaceSection then
    else
      RaiseMsg(20180310104857,Parent,GetObjName(Ref.Element));
    end
  else
    RaiseMsg(20180310104810,Parent,GetObjName(Ref.Element));
  //writeln('TPCUWriter.WriteExtRefSignature END ',GetObjName(Ref.Element));
end;

function TPCUWriter.WriteExternalReference(El: TPasElement;
  aContext: TPCUWriterContext): TPCUFilerElementRef;
var
  ParentRef, Ref: TPCUFilerElementRef;
  Parent, NameEl: TPasElement;
  Name: String;
  SpecItem: TPRSpecializedItem;
begin
  Result:=nil;
  if El=nil then exit;
  // check if already written
  Ref:=GetElementReference(El);
  if Ref.Obj<>nil then
    exit(Ref);// already written
  if not IsExternalEl(El) then
    RaiseMsg(20200323121033,El,GetObjName(El));

  //writeln('TPCUWriter.WriteExternalReference ',GetObjPath(El));
  // write Parent first
  Parent:=El.Parent;
  if (El.CustomData is TPasGenericScope) then
    SpecItem:=TPasGenericScope(El.CustomData).SpecializedFromItem
  else
    SpecItem:=nil;

  if SpecItem<>nil then
    ParentRef:=WriteExternalReference(SpecItem.GenericEl,aContext)
  else if IsExternalEl(Parent) then
    ParentRef:=WriteExternalReference(Parent,aContext)
  else
    begin
    // El is external, Parent is not
    RaiseMsg(20200328173009,El,GetObjName(El));
    end;
  if ParentRef=nil then
    if not (El is TPasModule) then
      RaiseMsg(20180308174440,El,GetObjName(El));

  // check name
  NameEl:=El;
  if SpecItem<>nil then
    NameEl:=SpecItem.GenericEl; // specialized -> use generic name
  Name:=Resolver.GetOverloadName(NameEl);
  if Name='' then
    begin
    Name:=GetDefaultRefName(El);
    if Name='' then
      RaiseMsg(20180308174850,El,GetObjName(El));
    end;
  // write
  Ref.Obj:=TJSONObject.Create;
  Ref.Obj.Add('Name',Name);
  if ParentRef<>nil then
    begin
    Ref.ParentRef:=ParentRef;
    // add to parent
    if SpecItem<>nil then
      begin
      if ParentRef.Specs=nil then
        begin
        ParentRef.Specs:=TJSONArray.Create;
        ParentRef.Obj.Add('Specs',ParentRef.Specs);
        end;
      ParentRef.Specs.Add(Ref.Obj);
      if (Ref.Id=0) then
        CreateElReferenceId(Ref); // every specialization needs an ID
      end
    else
      begin
      if ParentRef.Elements=nil then
        begin
        ParentRef.Elements:=TJSONArray.Create;
        ParentRef.Obj.Add('El',ParentRef.Elements);
        end;
      ParentRef.Elements.Add(Ref.Obj);
      end;
    //writeln('TPCUWriter.WriteExternalReference ',GetObjName(El),' WriteExtRefSignature...');
    WriteExtRefSignature(Ref,aContext);
    end
  else if (El.ClassType=TPasModule) or (El is TPasUnitModule) then
    begin
    // indirectly used unit (refs to directly used units are created in WriteSection)
    if aContext.IndirectUsesArr=nil then
       begin
      if aContext.SectionObj=nil then
        RaiseMsg(20180314154428,El);
      //writeln('TPCUWriter.WriteExternalReference ',Resolver.RootElement.Name,' Section=',GetObjName(aContext.Section),' IndirectUses=',El.Name);
      aContext.IndirectUsesArr:=TJSONArray.Create;
      aContext.SectionObj.Add('IndirectUses',aContext.IndirectUsesArr);
      end;
    aContext.IndirectUsesArr.Add(Ref.Obj);
    end
  else
    RaiseMsg(20180314153224,El);
  Result:=Ref;
end;

procedure TPCUWriter.WriteExternalReferences(aContext: TPCUWriterContext);
var
  Ref: TPCUFilerElementRef;
  El: TPasElement;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUWriter.WriteExternalReferences START aContext.Section=',GetObjName(aContext.Section));
  {$ENDIF}
  while FFirstNewExt<>nil do
    begin
    Ref:=FFirstNewExt;
    FFirstNewExt:=Ref.NextNewExt;
    if FFirstNewExt=nil then
      FLastNewExt:=nil;
    if Ref.Pending=nil then
      continue; // not used, e.g. when a child is written, its parents are
                // written too, who might still be in the queue
    El:=Ref.Element;
    //writeln('TPCUWriter.WriteExternalReferences ',GetObjName(El),' ',GetObjPath(El));
    {$IF defined(VerbosePJUFiler) or defined(VerbosePCUFiler) or defined(VerboseUnitQueue)}
    if El.CustomData is TResElDataBuiltInSymbol then
      RaiseMsg(20180314120554,El);
    if El.GetModule=Resolver.RootElement then
      if not IsExternalEl(El) then
        RaiseMsg(20180318120511,El);
    {$ENDIF}
    // external element
    if Ref.Obj=nil then
      WriteExternalReference(El,aContext);
    // Ref.Id is written in ResolvePendingElRefs
    ResolvePendingElRefs(Ref);
    end;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUWriter.WriteExternalReferences END aContext.Section=',GetObjName(aContext.Section));
  {$ENDIF}
end;

procedure TPCUWriter.WriteElement(Obj: TJSONObject;
  El: TPasElement; aContext: TPCUWriterContext);
var
  C: TClass;
  Kind: TPasExprKind;
begin
  C:=El.ClassType;
  if C=TUnaryExpr then
    begin
    Obj.Add('Type','Unary');
    WriteUnaryExpr(Obj,TUnaryExpr(El),aContext);
    end
  else if C=TBinaryExpr then
    begin
    Obj.Add('Type','Binary');
    WriteBinaryExpr(Obj,TBinaryExpr(El),aContext);
    end
  else if C=TPrimitiveExpr then
    begin
    Kind:=TPrimitiveExpr(El).Kind;
    if not (Kind in [pekIdent,pekNumber,pekString]) then
      RaiseMsg(20180210153604,El,PCUExprKindNames[Kind]);
    Obj.Add('Type',PCUExprKindNames[Kind]);
    WritePrimitiveExpr(Obj,TPrimitiveExpr(El),aContext);
    end
  else if C=TBoolConstExpr then
    begin
    if El.CustomData=nil then
      begin
      Obj.Add('Type',PCUBoolStr[TBoolConstExpr(El).Value]);
      WritePasExpr(Obj,TBoolConstExpr(El),pekBoolConst,eopNone,aContext);
      end
    else
      begin
      Obj.Add('Type','Bool');
      WriteBoolConstExpr(Obj,TBoolConstExpr(El),aContext);
      end;
    end
  else if C=TNilExpr then
    begin
    Obj.Add('Type','Nil');
    WritePasExpr(Obj,TNilExpr(El),pekNil,eopNone,aContext);
    end
  else if C=TInheritedExpr then
    begin
    Obj.Add('Type','Inherited');
    WritePasExpr(Obj,TInheritedExpr(El),pekInherited,eopNone,aContext);
    end
  else if C=TSelfExpr then
    begin
    Obj.Add('Type','Self');
    WritePasExpr(Obj,TSelfExpr(El),pekSelf,eopNone,aContext);
    end
  else if C=TParamsExpr then
    begin
    case TParamsExpr(El).Kind of
    pekArrayParams: Obj.Add('Type','A[]');
    pekFuncParams: Obj.Add('Type','F()');
    pekSet: Obj.Add('Type','[]');
    else
      RaiseMsg(20190222012727,El,ExprKindNames[TParamsExpr(El).Kind]);
    end;
    WriteParamsExpr(Obj,TParamsExpr(El),aContext);
    end
  else if C=TProcedureExpr then
    begin
    Obj.Add('Type','ProcExpr');
    WriteProcedureExpr(Obj,TProcedureExpr(El),aContext);
    end
  else if C=TRecordValues then
    begin
    Obj.Add('Type','RecValues');
    WriteRecordValues(Obj,TRecordValues(El),aContext);
    end
  else if C=TArrayValues then
    begin
    Obj.Add('Type','ArrValues');
    WriteArrayValues(Obj,TArrayValues(El),aContext);
    end
  else if C=TPasResString then
    begin
    Obj.Add('Type','ResString');
    WriteResString(Obj,TPasResString(El),aContext);
    end
  else if C=TPasAliasType then
    begin
    Obj.Add('Type','Alias');
    WriteAliasType(Obj,TPasAliasType(El),aContext);
    end
  else if C=TPasPointerType then
    begin
    Obj.Add('Type','Pointer');
    WritePointerType(Obj,TPasPointerType(El),aContext);
    end
  else if C=TPasTypeAliasType then
    begin
    Obj.Add('Type','TypeAlias');
    WriteAliasType(Obj,TPasTypeAliasType(El),aContext);
    end
  else if C=TPasClassOfType then
    begin
    Obj.Add('Type','ClassOf');
    WriteAliasType(Obj,TPasClassOfType(El),aContext);
    end
  else if C=TPasSpecializeType then
    begin
    Obj.Add('Type','Specialize');
    WriteSpecializeType(Obj,TPasSpecializeType(El),aContext);
    end
  else if C=TInlineSpecializeExpr then
    begin
    Obj.Add('Type','InlineSpecialize');
    WriteInlineSpecializeExpr(Obj,TInlineSpecializeExpr(El),aContext);
    end
  else if C=TPasRangeType then
    begin
    Obj.Add('Type','RangeType');
    WriteRangeType(Obj,TPasRangeType(El),aContext);
    end
  else if C=TPasArrayType then
    begin
    if Resolver.IsSpecialized(TPasGenericType(El)) then exit;
    Obj.Add('Type','ArrType');
    WriteArrayType(Obj,TPasArrayType(El),aContext);
    end
  else if C=TPasFileType then
    begin
    Obj.Add('Type','File');
    WriteFileType(Obj,TPasFileType(El),aContext);
    end
  else if C=TPasEnumValue then
    begin
    Obj.Add('Type','EnumV');
    WriteEnumValue(Obj,TPasEnumValue(El),aContext);
    end
  else if C=TPasEnumType then
    begin
    Obj.Add('Type','EnumType');
    WriteEnumType(Obj,TPasEnumType(El),aContext);
    end
  else if C=TPasSetType then
    begin
    Obj.Add('Type','SetType');
    WriteSetType(Obj,TPasSetType(El),aContext);
    end
  else if C=TPasVariant then
    begin
    Obj.Add('Type','RecVariant');
    WriteRecordVariant(Obj,TPasVariant(El),aContext);
    end
  else if C=TPasRecordType then
    begin
    if Resolver.IsSpecialized(TPasGenericType(El)) then exit;
    Obj.Add('Type','Record');
    WriteRecordType(Obj,TPasRecordType(El),aContext);
    end
  else if C=TPasClassType then
    begin
    if Resolver.IsSpecialized(TPasGenericType(El)) then
      exit; // Note: only referenced specializations are stored
    Obj.Add('Type',PCUObjKindNames[TPasClassType(El).ObjKind]);
    WriteClassType(Obj,TPasClassType(El),aContext);
    end
  else if C=TPasArgument then
    begin
    Obj.Add('Type','Arg');
    WriteArgument(Obj,TPasArgument(El),aContext);
    end
  else if C=TPasProcedureType then
    begin
    if Resolver.IsSpecialized(TPasGenericType(El)) then exit;
    Obj.Add('Type','ProcType');
    WriteProcedureType(Obj,TPasProcedureType(El),aContext);
    end
  else if C=TPasResultElement then
    begin
    Obj.Add('Type','Result');
    WriteResultElement(Obj,TPasResultElement(El),aContext);
    end
  else if C=TPasFunctionType then
    begin
    Obj.Add('Type','FuncType');
    WriteFunctionType(Obj,TPasFunctionType(El),aContext);
    end
  else if C=TPasStringType then
    begin
    Obj.Add('Type','StringType');
    WriteStringType(Obj,TPasStringType(El),aContext);
    end
  else if C=TPasVariable then
    begin
    Obj.Add('Type','Var');
    WriteVariable(Obj,TPasVariable(El),aContext);
    end
  else if C=TPasExportSymbol then
    begin
    Obj.Add('Type','Export');
    WriteExportSymbol(Obj,TPasExportSymbol(El),aContext);
    end
  else if C=TPasConst then
    begin
    Obj.Add('Type','Const');
    WriteConst(Obj,TPasConst(El),aContext);
    end
  else if C=TPasProperty then
    begin
    Obj.Add('Type','Property');
    WriteProperty(Obj,TPasProperty(El),aContext);
    end
  else if C=TPasMethodResolution then
    begin
    Obj.Add('Type','MethodRes');
    WriteMethodResolution(Obj,TPasMethodResolution(El),aContext);
    end
  else if C.InheritsFrom(TPasProcedure) then
    begin
    if C.InheritsFrom(TPasOperator) then
      begin
      if C=TPasOperator then
        Obj.Add('Type','Operator')
      else if C=TPasClassOperator then
        Obj.Add('Type','ClassOperator')
      else
        RaiseMsg(20180210130142,El);
      WriteOperator(Obj,TPasOperator(El),aContext);
      exit;
      end;
    if C=TPasProcedure then
      Obj.Add('Type','Procedure')
    else if C=TPasClassProcedure then
      Obj.Add('Type','ClassProcedure')
    else if C=TPasFunction then
      Obj.Add('Type','Function')
    else if C=TPasClassFunction then
      Obj.Add('Type','ClassFunction')
    else if C=TPasConstructor then
      Obj.Add('Type','Constructor')
    else if C=TPasClassConstructor then
      Obj.Add('Type','ClassConstructor')
    else if C=TPasDestructor then
      Obj.Add('Type','Destructor')
    else if C=TPasClassDestructor then
      Obj.Add('Type','ClassDestructor')
    else if C=TPasAnonymousProcedure then
      Obj.Add('Type','AnonymousProcedure')
    else if C=TPasAnonymousFunction then
      Obj.Add('Type','AnonymousFunction')
    else
      RaiseMsg(20180210130202,El);
    WriteProcedure(Obj,TPasProcedure(El),aContext);
    end
  else if C=TPasAttributes then
    begin
    Obj.Add('Type','Attributes');
    WriteAttributes(Obj,TPasAttributes(El),aContext);
    end
  else if C=TPasImplCommand then
    begin
    Obj.Add('Type','ImplCmd');
    WriteImplCommand(Obj,TPasImplCommand(El),aContext);
    end
  else if C=TPasImplBeginBlock then
    begin
    Obj.Add('Type','Begin');
    WriteImplBeginBlock(Obj,TPasImplBeginBlock(El),aContext);
    end
  else if C=TPasImplAsmStatement then
    begin
    Obj.Add('Type','Asm');
    WriteImplAsmStatement(Obj,TPasImplAsmStatement(El),aContext);
    end
  else if C=TPasImplRepeatUntil then
    begin
    Obj.Add('Type','Repeat');
    WriteImplRepeatUntil(Obj,TPasImplRepeatUntil(El),aContext);
    end
  else if C=TPasImplIfElse then
    begin
    Obj.Add('Type','If');
    WriteImplIfElse(Obj,TPasImplIfElse(El),aContext);
    end
  else if C=TPasImplWhileDo then
    begin
    Obj.Add('Type','While');
    WriteImplWhileDo(Obj,TPasImplWhileDo(El),aContext);
    end
  else if C=TPasImplWithDo then
    begin
    Obj.Add('Type','With');
    WriteImplWithDo(Obj,TPasImplWithDo(El),aContext);
    end
  else if C=TPasImplCaseOf then
    begin
    Obj.Add('Type','CaseOf');
    WriteImplCaseOf(Obj,TPasImplCaseOf(El),aContext);
    end
  else if C=TPasImplCaseStatement then
    begin
    Obj.Add('Type','CaseSt');
    WriteImplCaseStatement(Obj,TPasImplCaseStatement(El),aContext);
    end
  else if C=TPasImplCaseElse then
    begin
    Obj.Add('Type','CaseElse');
    WriteImplCaseElse(Obj,TPasImplCaseElse(El),aContext);
    end
  else if C=TPasImplForLoop then
    begin
    Obj.Add('Type','ForLoop');
    WriteImplForLoop(Obj,TPasImplForLoop(El),aContext);
    end
  else if C=TPasImplAssign then
    begin
    Obj.Add('Type','Assign');
    WriteImplAssign(Obj,TPasImplAssign(El),aContext);
    end
  else if C=TPasImplSimple then
    begin
    Obj.Add('Type','Simple');
    WriteImplSimple(Obj,TPasImplSimple(El),aContext);
    end
  else if C=TPasImplTry then
    begin
    Obj.Add('Type','Try');
    WriteImplTry(Obj,TPasImplTry(El),aContext);
    end
  else if C=TPasImplTryFinally then
    begin
    Obj.Add('Type','Finally');
    WriteImplTryHandler(Obj,TPasImplTryFinally(El),aContext);
    end
  else if C=TPasImplTryExcept then
    begin
    Obj.Add('Type','Except');
    WriteImplTryHandler(Obj,TPasImplTryExcept(El),aContext);
    end
  else if C=TPasImplTryExceptElse then
    begin
    Obj.Add('Type','ExceptElse');
    WriteImplTryHandler(Obj,TPasImplTryExceptElse(El),aContext);
    end
  else if C=TPasImplExceptOn then
    begin
    Obj.Add('Type','ExceptOn');
    WriteImplExceptOn(Obj,TPasImplExceptOn(El),aContext);
    end
  else if C=TPasImplRaise then
    begin
    Obj.Add('Type','Raise');
    WriteImplRaise(Obj,TPasImplRaise(El),aContext);
    end
  else
    begin
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUWriter.WriteElement ',GetObjName(El));
    {$ENDIF}
    RaiseMsg(20180205154041,El,GetObjName(El));
    end;
end;

procedure TPCUWriter.WriteUnaryExpr(Obj: TJSONObject; Expr: TUnaryExpr;
  aContext: TPCUWriterContext);
begin
  WritePasExpr(Obj,Expr,pekUnary,eopAdd,aContext);
  WriteExpr(Obj,Expr,'Operand',Expr.Operand,aContext);
end;

procedure TPCUWriter.WriteBinaryExpr(Obj: TJSONObject; Expr: TBinaryExpr;
  aContext: TPCUWriterContext);
begin
  WritePasExpr(Obj,Expr,pekBinary,eopAdd,aContext);
  WriteExpr(Obj,Expr,'Left',Expr.left,aContext);
  WriteExpr(Obj,Expr,'Right',Expr.right,aContext);
end;

procedure TPCUWriter.WritePrimitiveExpr(Obj: TJSONObject; Expr: TPrimitiveExpr;
  aContext: TPCUWriterContext);
begin
  WritePasExpr(Obj,Expr,Expr.Kind,eopNone,aContext);
  if Expr.Value<>'' then
    Obj.Add('Value',Expr.Value);
end;

procedure TPCUWriter.WriteBoolConstExpr(Obj: TJSONObject; Expr: TBoolConstExpr;
  aContext: TPCUWriterContext);
begin
  WritePasExpr(Obj,Expr,pekBoolConst,eopNone,aContext);
  if Expr.Value then
    Obj.Add('Value',true);
end;

procedure TPCUWriter.WriteParamsExpr(Obj: TJSONObject; Expr: TParamsExpr;
  aContext: TPCUWriterContext);
begin
  WritePasExpr(Obj,Expr,Expr.Kind,eopNone,aContext);
  WriteExpr(Obj,Expr,'Value',Expr.Value,aContext);
  WritePasExprArray(Obj,Expr,'Params',Expr.Params,aContext);
end;

procedure TPCUWriter.WriteProcedureExpr(Obj: TJSONObject; Expr: TProcedureExpr;
  aContext: TPCUWriterContext);
begin
  WritePasExpr(Obj,Expr,Expr.Kind,eopNone,aContext);
  WriteElementProperty(Obj,Expr,'Proc',Expr.Proc,aContext);
end;

procedure TPCUWriter.WriteRecordValues(Obj: TJSONObject; Expr: TRecordValues;
  aContext: TPCUWriterContext);
var
  Arr: TJSONArray;
  i: Integer;
  SubObj: TJSONObject;
  RecValue: TRecordValuesItem;
begin
  WritePasExpr(Obj,Expr,pekListOfExp,eopNone,aContext);
  if length(Expr.Fields)>0 then
    begin
    Arr:=TJSONArray.Create;
    Obj.Add('Fields',Arr);
    for i:=0 to length(Expr.Fields)-1 do
      begin
      RecValue:=Expr.Fields[i];
      SubObj:=TJSONObject.Create;
      Arr.Add(SubObj);
      SubObj.Add('Name',RecValue.Name);
      WriteExpr(SubObj,Expr,'NameExpr',RecValue.NameExp,aContext);
      WriteExpr(SubObj,Expr,'ValueExpr',RecValue.ValueExp,aContext);
      end;
    end;
end;

procedure TPCUWriter.WriteArrayValues(Obj: TJSONObject; Expr: TArrayValues;
  aContext: TPCUWriterContext);
begin
  WritePasExpr(Obj,Expr,pekListOfExp,eopNone,aContext);
  WritePasExprArray(Obj,Expr,'Values',Expr.Values,aContext);
end;

procedure TPCUWriter.WriteResString(Obj: TJSONObject; El: TPasResString;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'Expr',El.Expr,aContext);
end;

procedure TPCUWriter.WriteGenericTemplateTypes(Obj: TJSONObject;
  Parent: TPasElement; GenericTemplateTypes: TFPList;
  aContext: TPCUWriterContext);
var
  Arr: TJSONArray;
  i: Integer;
  Templ: TPasGenericTemplateType;
  TemplObj: TJSONObject;
begin
  if Parent=nil then ;
  if (GenericTemplateTypes=nil) or (GenericTemplateTypes.Count=0) then exit;
  Arr:=TJSONArray.Create;
  Obj.Add('GenericTemplateTypes',Arr);
  for i:=0 to GenericTemplateTypes.Count-1 do
    begin
    Templ:=TPasGenericTemplateType(GenericTemplateTypes[i]);
    TemplObj:=TJSONObject.Create;
    Arr.Add(TemplObj);
    WritePasElement(TemplObj,Templ,aContext);
    WriteElementArray(TemplObj,Templ,'Constraints',Templ.Constraints,aContext,true);
    end;
end;

procedure TPCUWriter.WriteAliasType(Obj: TJSONObject; El: TPasAliasType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElType(Obj,El,'Dest',El.DestType,aContext);
  WriteExpr(Obj,El,'Expr',El.Expr,aContext);
end;

procedure TPCUWriter.WritePointerType(Obj: TJSONObject; El: TPasPointerType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElType(Obj,El,'Dest',El.DestType,aContext);
end;

procedure TPCUWriter.WriteSpecializeType(Obj: TJSONObject;
  El: TPasSpecializeType; aContext: TPCUWriterContext);
var
  SpecTypeData: TPasSpecializeTypeData;
  SpecType: TPasGenericType;
begin
  WriteAliasType(Obj,El,aContext);
  WriteElementList(Obj,El,'Params',El.Params,aContext,true);
  if El.CustomData=nil then
    exit; // SpecTypeData can be nil, when a generic A<T> refers to a generic B<T>
  if not (El.CustomData is TPasSpecializeTypeData) then
    RaiseMsg(20200219122421,El,GetObjName(El.CustomData));
  SpecTypeData:=TPasSpecializeTypeData(El.CustomData);
  SpecType:=SpecTypeData.SpecializedType;
  if SpecType=nil then
    RaiseMsg(20201203093316,El);
  WriteElType(Obj,El,'SpecType',SpecType,aContext);
  Obj.Add('SpecTypeName',SpecType.Name);
end;

procedure TPCUWriter.WriteInlineSpecializeExpr(Obj: TJSONObject;
  Expr: TInlineSpecializeExpr; aContext: TPCUWriterContext);
begin
  WritePasExpr(Obj,Expr,pekSpecialize,eopNone,aContext);
  WriteExpr(Obj,Expr,'ISEName',Expr.NameExpr,aContext);
  WriteElementList(Obj,Expr,'ISEParams',Expr.Params,aContext,true);
end;

procedure TPCUWriter.WriteRangeType(Obj: TJSONObject; El: TPasRangeType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'Range',El.RangeExpr,aContext);
end;

procedure TPCUWriter.WriteArrayTypeScope(Obj: TJSONObject;
  Scope: TPas2JSArrayScope; aContext: TPCUWriterContext);
begin
  WriteIdentifierScope(Obj,Scope,aContext);
end;

procedure TPCUWriter.WriteArrayType(Obj: TJSONObject; El: TPasArrayType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteGenericTemplateTypes(Obj,El,El.GenericTemplateTypes,aContext);
  WritePasExprArray(Obj,El,'Ranges',El.Ranges,aContext);
  if El.PackMode<>pmNone then
    Obj.Add('Packed',PCUPackModeNames[El.PackMode]);
  WriteElType(Obj,El,'ElType',El.ElType,aContext);
  if El.CustomData is TPas2JSArrayScope then
    WriteArrayTypeScope(Obj,TPas2JSArrayScope(El.CustomData),aContext);
end;

procedure TPCUWriter.WriteFileType(Obj: TJSONObject; El: TPasFileType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElType(Obj,El,'ElType',El.ElType,aContext);
end;

procedure TPCUWriter.WriteEnumValue(Obj: TJSONObject; El: TPasEnumValue;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'Value',El.Value,aContext);
end;

procedure TPCUWriter.WriteEnumTypeScope(Obj: TJSONObject;
  Scope: TPasEnumTypeScope; aContext: TPCUWriterContext);
begin
  WriteIdentifierScope(Obj,Scope,aContext);
  WriteElType(Obj,Scope.Element,'CanonicalSet',Scope.CanonicalSet,aContext);
end;

procedure TPCUWriter.WriteEnumType(Obj: TJSONObject; El: TPasEnumType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementList(Obj,El,'Values',El.Values,aContext);
  WriteEnumTypeScope(Obj,EL.CustomData as TPasEnumTypeScope,aContext);
end;

procedure TPCUWriter.WriteSetType(Obj: TJSONObject; El: TPasSetType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElType(Obj,El,'EnumType',El.EnumType,aContext);
  if El.IsPacked then
    Obj.Add('Packed',true);
end;

procedure TPCUWriter.WriteRecordVariant(Obj: TJSONObject; El: TPasVariant;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementList(Obj,El,'Values',El.Values,aContext);
  WriteElType(Obj,El,'Members',El.Members,aContext);
end;

procedure TPCUWriter.WriteRecordTypeScope(Obj: TJSONObject;
  Scope: TPas2jsRecordScope; aContext: TPCUWriterContext);
begin
  AddReferenceToObj(Obj,'DefaultProperty',Scope.DefaultProperty);
  WriteIdentifierScope(Obj,Scope,aContext);
end;

procedure TPCUWriter.WriteRecordType(Obj: TJSONObject; El: TPasRecordType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteGenericTemplateTypes(Obj,El,El.GenericTemplateTypes,aContext);
  if El.PackMode<>pmNone then
    Obj.Add('Packed',PCUPackModeNames[El.PackMode]);
  WriteElementList(Obj,El,'Members',El.Members,aContext);
  // VariantEl: TPasElement can be TPasVariable or TPasType
  if El.VariantEl is TPasType then
    WriteElType(Obj,El,'VariantEl',TPasType(El.VariantEl),aContext)
  else
    WriteElementProperty(Obj,El,'VariantEl',El.VariantEl,aContext);
  WriteElementList(Obj,El,'Variants',El.Variants,aContext);

  WriteRecordTypeScope(Obj,El.CustomData as TPas2jsRecordScope,aContext);
end;

procedure TPCUWriter.WriteClassScopeFlags(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TPasClassScopeFlags);
var
  Arr: TJSONArray;
  f: TPasClassScopeFlag;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TPasClassScopeFlag do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUClassScopeFlagNames[f],f in Value);
end;

procedure TPCUWriter.WriteClassIntfMapProcs(Obj: TJSONObject;
  Map: TPasClassIntfMap);
var
  Procs: TFPList;
  Arr: TJSONArray;
  i: Integer;
begin
  Procs:=Map.Procs;
  if Procs<>nil then
    begin
    Arr:=TJSONArray.Create;
    Obj.Add('Procs',Arr);
    for i:=0 to Procs.Count-1 do
      AddReferenceToArray(Arr,TPasProcedure(Procs[i]));
    end;
end;

procedure TPCUWriter.WriteClassScope(Obj: TJSONObject;
  Scope: TPas2JSClassScope; aContext: TPCUWriterContext);

  procedure WriteMap(SubObj: TJSONObject; Map: TPasClassIntfMap);
  var
    AncObj: TJSONObject;
  begin
    if Map.Element=nil then
      RaiseMsg(20180325131134,Scope.Element);
    if Map.Intf=nil then
      RaiseMsg(20180325131135,Scope.Element);
    AddReferenceToObj(SubObj,'Intf',Map.Intf);
    WriteClassIntfMapProcs(SubObj,Map);
    if Map.AncestorMap<>nil then
      begin
      AncObj:=TJSONObject.Create;
      SubObj.Add('AncestorMap',AncObj);
      WriteMap(AncObj,Map.AncestorMap);
      end;
  end;

var
  Arr: TJSONArray;
  i: Integer;
  aClass: TPasClassType;
  CanonicalClassOf: TPasClassOfType;
  ScopeIntf: TFPList;
  o: TObject;
  SubObj: TJSONObject;
  Ref: TPCUFilerElementRef;
begin
  WriteIdentifierScope(Obj,Scope,aContext);
  aClass:=Scope.Element as TPasClassType;
  AddReferenceToObj(Obj,'NewInstanceFunction',Scope.NewInstanceFunction);
  // AncestorScope can be derived from DirectAncestor
  // CanonicalClassOf is autogenerated
  CanonicalClassOf:=Scope.CanonicalClassOf;
  if aClass.ObjKind in ([okClass]+okAllHelpers) then
    begin
    if CanonicalClassOf=nil then
      RaiseMsg(20180217143821,aClass);
    if CanonicalClassOf.Name<>'Self' then
      RaiseMsg(20180217143822,aClass);
    if CanonicalClassOf.DestType<>aClass then
      RaiseMsg(20180217143834,aClass);
    if CanonicalClassOf.Visibility<>visStrictPrivate then
      RaiseMsg(20180217143844,aClass);
    if CanonicalClassOf.SourceFilename<>aClass.SourceFilename then
      RaiseMsg(20180217143857,aClass);
    if CanonicalClassOf.SourceLinenumber<>aClass.SourceLinenumber then
      RaiseMsg(20180217143905,aClass);
    Ref:=GetElementReference(CanonicalClassOf);
    CreateAutoElReferenceId(Ref);
    Obj.Add('ClassOf',Ref.Id);
    ResolvePendingElRefs(Ref);
    end
  else if CanonicalClassOf<>nil then
    RaiseMsg(20180329110817,aClass,GetObjName(CanonicalClassOf));

  AddReferenceToObj(Obj,'DirectAncestor',Scope.DirectAncestor);
  AddReferenceToObj(Obj,'DefaultProperty',Scope.DefaultProperty);
  WriteClassScopeFlags(Obj,'SFlags',Scope.Flags,GetDefaultClassScopeFlags(Scope));

  if length(Scope.AbstractProcs)>0 then
    begin
    Arr:=TJSONArray.Create;
    Obj.Add('AbstractProcs',Arr);
    for i:=0 to length(Scope.AbstractProcs)-1 do
      AddReferenceToArray(Arr,Scope.AbstractProcs[i]);
    end;

  if Scope.DispatchField<>PCUDispatchDefaultField then
    Obj.Add('DispatchField',Scope.DispatchField);
  if Scope.DispatchStrField<>PCUDispatchDefaultStrField then
    Obj.Add('DispatchStrField',Scope.DispatchStrField);

  if Scope.GUID<>'' then
    Obj.Add('SGUID',Scope.GUID);

  ScopeIntf:=Scope.Interfaces;
  if (ScopeIntf<>nil) and (ScopeIntf.Count>0) then
    begin
    Arr:=TJSONArray.Create;
    Obj.Add('SInterfaces',Arr);
    for i:=0 to ScopeIntf.Count-1 do
      begin
      o:=TObject(ScopeIntf[i]);
      if o is TPasProperty then
        begin
        // delegation
        AddReferenceToArray(Arr,TPasProperty(o));
        end
      else if o is TPasClassIntfMap then
        begin
        // method resolution
        SubObj:=TJSONObject.Create;
        Arr.Add(SubObj);
        WriteMap(SubObj,TPasClassIntfMap(o));
        end
      else
        RaiseMsg(20180325111939,aClass,IntToStr(i)+':'+GetObjName(TObject(aClass.Interfaces[i]))+' '+GetObjName(o));
      end;
    end;
end;

procedure TPCUWriter.WriteClassType(Obj: TJSONObject; El: TPasClassType;
  aContext: TPCUWriterContext);
var
  Arr: TJSONArray;
  i: Integer;
  Ref: TResolvedReference;
  Scope: TPas2JSClassScope;
begin
  WritePasElement(Obj,El,aContext);
  WriteGenericTemplateTypes(Obj,El,El.GenericTemplateTypes,aContext);
  if El.PackMode<>pmNone then
    Obj.Add('Packed',PCUPackModeNames[El.PackMode]);
  // ObjKind is the 'Type'
  if El.InterfaceType<>citCom then
    Obj.Add('IntfType',PCUClassInterfaceTypeNames[El.InterfaceType]);
  WriteElType(Obj,El,'Ancestor',El.AncestorType,aContext);
  WriteElType(Obj,El,'HelperFor',El.HelperForType,aContext);
  if El.IsForward then
    Obj.Add('Forward',true);
  if El.IsExternal then
    Obj.Add('External',true);
  // not needed IsShortDefinition: Boolean; -> class(anchestor); without end
  WriteExpr(Obj,El,'GUID',El.GUIDExpr,aContext);
  if El.Modifiers.Count>0 then
    begin
    Arr:=TJSONArray.Create;
    Obj.Add('Modifiers',Arr);
    for i:=0 to El.Modifiers.Count-1 do
      Arr.Add(El.Modifiers[i]);
    end;
  if El.ExternalNameSpace<>'' then
    Obj.Add('ExternalNameSpace',El.ExternalNameSpace);
  if El.ExternalName<>'' then
    Obj.Add('ExternalName',El.ExternalName);
  if El.IsForward then
    begin
    Ref:=TResolvedReference(El.CustomData);
    WriteResolvedReference(Obj,Ref,El);
    end
  else
    begin
    Scope:=El.CustomData as TPas2JSClassScope;
    WriteElementList(Obj,El,'Interfaces',El.Interfaces,aContext,true);
    WriteElementList(Obj,El,'Members',El.Members,aContext);
    if Scope<>nil then
      WriteClassScope(Obj,Scope,aContext)
    else
      Obj.Add('Scope',false); // msIgnoreInterfaces
    end;
end;

procedure TPCUWriter.WriteArgument(Obj: TJSONObject; El: TPasArgument;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  if El.Access<>argDefault then
    Obj.Add('Access',PCUArgumentAccessNames[El.Access]);
  if El.ArgType<>nil then
    begin
    if El.ArgType.Parent=El then
      WriteElementProperty(Obj,El,'ArgType',El.ArgType,aContext)
    else
      AddReferenceToObj(Obj,'ArgType',El.ArgType);
    end;
  WriteExpr(Obj,El,'Value',El.ValueExpr,aContext)
end;

procedure TPCUWriter.WriteProcTypeModifiers(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TProcTypeModifiers);
var
  Arr: TJSONArray;
  f: TProcTypeModifier;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TProcTypeModifier do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUProcTypeModifierNames[f],f in Value);
end;

procedure TPCUWriter.WriteProcTypeScope(Obj: TJSONObject;
  Scope: TPas2JSProcTypeScope; aContext: TPCUWriterContext);
begin
  WriteIdentifierScope(Obj,Scope,aContext);
end;

procedure TPCUWriter.WriteProcedureType(Obj: TJSONObject;
  El: TPasProcedureType; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteGenericTemplateTypes(Obj,El,El.GenericTemplateTypes,aContext);
  WriteElementList(Obj,El,'Args',El.Args,aContext);
  if El.CallingConvention<>ccDefault then
    Obj.Add('Call',PCUCallingConventionNames[El.CallingConvention]);
  WriteProcTypeModifiers(Obj,'Modifiers',El.Modifiers,GetDefaultProcTypeModifiers(El));
  if El.CustomData is TPas2JSProcTypeScope then
    WriteProcTypeScope(Obj,TPas2JSProcTypeScope(El.CustomData),aContext);
end;

procedure TPCUWriter.WriteResultElement(Obj: TJSONObject;
  El: TPasResultElement; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElType(Obj,El,'Result',El.ResultType,aContext);
end;

procedure TPCUWriter.WriteFunctionType(Obj: TJSONObject; El: TPasFunctionType;
  aContext: TPCUWriterContext);
begin
  WriteProcedureType(Obj,El,aContext);
  WriteElementProperty(Obj,El,'Result',El.ResultEl,aContext);
end;

procedure TPCUWriter.WriteStringType(Obj: TJSONObject; El: TPasStringType;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  Obj.Add('Length',El.LengthExpr);
end;

procedure TPCUWriter.WriteVariable(Obj: TJSONObject; El: TPasVariable;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  if El.VarType<>nil then
    begin
    if El.VarType.Parent=El then
      // anonymous type
      WriteElementProperty(Obj,El,'VarType',El.VarType,aContext)
    else
      // reference
      AddReferenceToObj(Obj,'VarType',El.VarType);
    end;
  WriteVarModifiers(Obj,'VarMods',El.VarModifiers,[]);
  WriteExpr(Obj,El,'Library',El.LibraryName,aContext);
  WriteExpr(Obj,El,'Export',El.ExportName,aContext);
  WriteExpr(Obj,El,'Absolute',El.AbsoluteExpr,aContext);
  WriteExpr(Obj,El,'Expr',El.Expr,aContext);
end;

procedure TPCUWriter.WriteExportSymbol(Obj: TJSONObject; El: TPasExportSymbol;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'NameExpr',El.NameExpr,aContext);
  WriteExpr(Obj,El,'ExportName',El.ExportName,aContext);
  WriteExpr(Obj,El,'ExportIndex',El.ExportIndex,aContext);
end;

procedure TPCUWriter.WriteConst(Obj: TJSONObject; El: TPasConst;
  aContext: TPCUWriterContext);
begin
  WriteVariable(Obj,El,aContext);
  if El.IsConst<>(El.VarType=nil) then
    Obj.Add('IsConst',El.IsConst);
end;

procedure TPCUWriter.WritePropertyScope(Obj: TJSONObject;
  Scope: TPasPropertyScope; aContext: TPCUWriterContext);
begin
  WriteIdentifierScope(Obj,Scope,aContext);
  AddReferenceToObj(Obj,'AncestorProp',Scope.AncestorProp);
end;

procedure TPCUWriter.WriteProperty(Obj: TJSONObject; El: TPasProperty;
  aContext: TPCUWriterContext);
var
  Scope: TPasPropertyScope;
begin
  Scope:=El.CustomData as TPasPropertyScope;
  WriteVariable(Obj,El,aContext);
  WriteExpr(Obj,El,'Index',El.IndexExpr,aContext);
  WriteExpr(Obj,El,'Read',El.ReadAccessor,aContext);
  WriteExpr(Obj,El,'Write',El.WriteAccessor,aContext);
  WritePasExprArray(Obj,El,'Implements',El.Implements,aContext);
  WriteExpr(Obj,El,'DispId',El.DispIDExpr,aContext);
  WriteExpr(Obj,El,'Stored',El.StoredAccessor,aContext);
  WriteExpr(Obj,El,'DefaultValue',El.DefaultExpr,aContext);
  WriteElementList(Obj,El,'Args',El.Args,aContext);
  //ReadAccessorName: string; // not used by resolver
  //WriteAccessorName: string; // not used by resolver
  //ImplementsName: string; // not used by resolver
  //StoredAccessorName: string; // not used by resolver
  if El.DispIDReadOnly then
    Obj.Add('ReadOnly',true);
  if El.isDefault then
    Obj.Add('Default',true);
  if El.IsNodefault then
    Obj.Add('NoDefault',true);

  if Scope<>nil then
    WritePropertyScope(Obj,Scope,aContext)
  else
    Obj.Add('Scope',false); // msIgnoreInterfaces
end;

procedure TPCUWriter.WriteMethodResolution(Obj: TJSONObject;
  El: TPasMethodResolution; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  if El.ProcClass=TPasProcedure then
    Obj.Add('ProcClass','procedure')
  else if El.ProcClass=TPasFunction then
    // default value
  else
    RaiseMsg(20180329104205,El);
  WriteExpr(Obj,El,'InterfaceName',El.InterfaceName,aContext);
  WriteExpr(Obj,El,'InterfaceProc',El.InterfaceProc,aContext);
  WriteExpr(Obj,El,'ImplementationProc',El.ImplementationProc,aContext);
end;

procedure TPCUWriter.WriteGenericTemplateType(Obj: TJSONObject;
  El: TPasGenericTemplateType; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  if not (El.CustomData is TPasGenericParamsScope) then
    RaiseMsg(20191120175118,El,GetObjName(El.CustomData));
  WriteElementArray(Obj,El,'Constraints',El.Constraints,aContext,true);
end;

procedure TPCUWriter.WriteProcedureNameParts(Obj: TJSONObject;
  El: TPasProcedure; aContext: TPCUWriterContext);
var
  Arr, TemplArr: TJSONArray;
  NamePartObj, TemplObj: TJSONObject;
  i, j: Integer;
  GenType: TPasGenericTemplateType;
  NameParts: TProcedureNameParts;
begin
  NameParts:=El.NameParts;
  if (NameParts=nil) or (NameParts.Count=0) then exit;
  Arr:=TJSONArray.Create;
  Obj.Add('NameParts',Arr);
  for i:=0 to NameParts.Count-1 do
    begin
    NamePartObj:=TJSONObject.Create;
    Arr.Add(NamePartObj);
    with TProcedureNamePart(NameParts[i]) do
      begin
      NamePartObj.Add('Name',Name);
      if Templates<>nil then
        begin
        TemplArr:=TJSONArray.Create;
        NamePartObj.Add('Templates',TemplArr);
        for j:=0 to Templates.Count-1 do
          begin
          GenType:=TPasGenericTemplateType(Templates[j]);
          TemplObj:=TJSONObject.Create;
          TemplArr.Add(TemplObj);
          WriteGenericTemplateType(TemplObj,GenType,aContext);
          end;
        end;
      end;
    end;
end;

procedure TPCUWriter.WriteProcedureModifiers(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TProcedureModifiers);
var
  Arr: TJSONArray;
  f: TProcedureModifier;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TProcedureModifier do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUProcedureModifierNames[f],f in Value);
end;

procedure TPCUWriter.WriteProcScopeFlags(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TPasProcedureScopeFlags);
var
  Arr: TJSONArray;
  f: TPasProcedureScopeFlag;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TPasProcedureScopeFlag do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUProcedureScopeFlagNames[f],f in Value);
end;

procedure TPCUWriter.WriteProcedureScope(Obj: TJSONObject;
  Scope: TPas2JSProcedureScope; aContext: TPCUWriterContext);
begin
  // Not needed, contains only local stuff: WriteIdentifierScope(Obj,Scope,aContext);
  if Scope.ResultVarName<>'' then
    Obj.Add('ResultVarName',Scope.ResultVarName);
  // Scope.OverloadName is stored as 'Name' and ReadProcedureScope reverts it

  if Scope.DeclarationProc<>nil then
    RaiseMsg(20180219135933,Scope.Element);
  AddReferenceToObj(Obj,'ImplProc',Scope.ImplProc);
  AddReferenceToObj(Obj,'Overridden',Scope.OverriddenProc);
  // ClassOrRecordScope is auto derived
  // SelfArg is auto derived
  // Mode is auto derived
  WriteProcScopeFlags(Obj,'SFlags',Scope.Flags,[]);
  WriteBoolSwitches(Obj,'BoolSwitches',Scope.BoolSwitches,aContext.BoolSwitches);
  WriteModeSwitches(Obj,'ModeSwitches',Scope.ModeSwitches,aContext.ModeSwitches);
end;

procedure TPCUWriter.WriteProcedureBody(Obj: TJSONObject; El: TProcedureBody;
  aContext: TPCUWriterContext);
var
  ImplObj: TJSONObject;
begin
  WriteDeclarations(Obj,El,aContext);
  if El.Body<>nil then
    begin
    ImplObj:=TJSONObject.Create;
    Obj.Add('Impl',ImplObj);
    WriteElement(ImplObj,El.Body,aContext);
    end;
end;

procedure TPCUWriter.WriteProcedure(Obj: TJSONObject; El: TPasProcedure;
  aContext: TPCUWriterContext);
var
  DefProcMods, ImplProcMods, DeclProcMods: TProcedureModifiers;
  Scope: TPas2JSProcedureScope;
  DeclProc: TPasProcedure;
  DeclScope: TPas2JsProcedureScope;
  BodyObj: TJSONObject;
  OldInGeneric: Boolean;
begin
  WritePasElement(Obj,El,aContext);

  Scope:=El.CustomData as TPas2JSProcedureScope;
  //writeln('TPCUWriter.WriteProcedure ',GetObjName(El),' ',GetObjName(Scope),' ',Resolver.GetElementSourcePosStr(El));

  if Scope.SpecializedFromItem<>nil then
    begin
    // spezialiations are generated on the fly -> cannot be stored
    RaiseMsg(20191120180305,El,GetObjPath(Scope.SpecializedFromItem.FirstSpecialize));
    end;
  if (Scope.ImplJS<>nil) and (Scope.ImplProc<>nil) then
    RaiseMsg(20180228142831,El);

  if Scope.DeclarationProc=nil then
    begin
    // declaration
    WriteProcedureNameParts(Obj,El,aContext);
    WriteElementProperty(Obj,El,'ProcType',El.ProcType,aContext);
    WriteExpr(Obj,El,'Public',El.PublicName,aContext);
    // e.g. external LibraryExpr name LibrarySymbolName;
    WriteExpr(Obj,El,'Lib',El.LibraryExpr,aContext);
    WriteExpr(Obj,El,'LibName',El.LibrarySymbolName,aContext);
    WriteExpr(Obj,El,'DispId',El.DispIDExpr,aContext);
    if El.AliasName<>'' then
      Obj.Add('Alias',El.AliasName);
    DefProcMods:=GetDefaultProcModifiers(El);
    WriteProcedureModifiers(Obj,'PMods',El.Modifiers,DefProcMods);
    WriteExpr(Obj,El,'Msg',El.MessageExpr,aContext);
    if (El.MessageName<>'') or (El.MessageType<>pmtNone) then
      begin
      Obj.Add('Message',El.MessageName);
      if El.MessageType<>pmtInteger then
        Obj.Add('MessageType',PCUProcedureMessageTypeNames[El.MessageType]);
      end;
    WriteProcedureScope(Obj,Scope,aContext);
    end
  else
    begin
    // implementation
    AddReferenceToObj(Obj,'DeclarationProc',Scope.DeclarationProc);
    end;

  if (Scope.ImplProc=nil) and (El.Body<>nil) then
    begin
    // proc with body
    DeclProc:=Scope.DeclarationProc;
    if DeclProc=nil then
      DeclProc:=El;
    DeclScope:=NoNil(DeclProc.CustomData) as TPas2JSProcedureScope;

    if Resolver.ProcCanBePrecompiled(DeclProc) then
      begin
      // normal procedure: store references and precompiled JS

      // Note: although the References are in the declaration scope,
      //       they are stored with the implementation scope, so that
      //       all references can be resolved immediately by the reader
      WriteScopeReferences(Obj,DeclScope.References,'Refs',aContext);

      // precompiled body
      WritePrecompiledJS(Obj,El,Scope.ImplJS,aContext);
      end
    else
      begin
      // generic function: store pascal elements
      if Scope.ImplJS<>nil then
        RaiseMsg(20191120171941,El);
      ImplProcMods:=El.Modifiers*PCUProcedureModifiersImplProc;
      DeclProcMods:=DeclProc.Modifiers*PCUProcedureModifiersImplProc;
      if ImplProcMods<>DeclProcMods then
        WriteProcedureModifiers(Obj,'PMods',ImplProcMods,DeclProcMods);

      BodyObj:=TJSONObject.Create;
      Obj.Add('Body',BodyObj);
      OldInGeneric:=aContext.InGeneric;
      aContext.InGeneric:=true;
      WriteProcedureBody(BodyObj,El.Body,aContext);
      aContext.InGeneric:=OldInGeneric;
      end;
    end;
end;

procedure TPCUWriter.WriteOperator(Obj: TJSONObject; El: TPasOperator;
  aContext: TPCUWriterContext);
begin
  WriteProcedure(Obj,El,aContext);
  Obj.Add('Operator',PCUOperatorTypeNames[El.OperatorType]);
  if El.TokenBased then
    Obj.Add('TokenBased',El.TokenBased);
end;

procedure TPCUWriter.WriteAttributes(Obj: TJSONObject; El: TPasAttributes;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WritePasExprArray(Obj,El,'Calls',El.Calls,aContext);
end;

procedure TPCUWriter.WritePrecompiledJS(Obj: TJSONObject; El: TPasElement;
  ImplJS: TPas2JSPrecompiledJS; aContext: TPCUWriterContext);
var
  Arr: TJSONArray;
  i: Integer;
begin
  if ImplJS=nil then exit;
  if (ImplJS.BodyJS<>'') then
    begin
    if ImplJS.GlobalJS<>nil then
      begin
      Arr:=TJSONArray.Create;
      Obj.Add('Globals',Arr);
      for i:=0 to ImplJS.GlobalJS.Count-1 do
        Arr.Add(ImplJS.GlobalJS[i]);
      end;
    if ImplJS.ShortRefs<>nil then
      WriteElementList(Obj,El,'ShortRefs',ImplJS.ShortRefs,aContext,true);
    Obj.Add('Body',ImplJS.BodyJS);
    end;
  if ImplJS.EmptyJS then
    Obj.Add('Empty',ImplJS.EmptyJS);
end;

procedure TPCUWriter.WriteImplCommand(Obj: TJSONObject; El: TPasImplCommand;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
end;

procedure TPCUWriter.WriteImplBeginBlock(Obj: TJSONObject;
  El: TPasImplBeginBlock; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementList(Obj,El,'Elements',El.Elements,aContext);
end;

procedure TPCUWriter.WriteImplAsmStatement(Obj: TJSONObject;
  El: TPasImplAsmStatement; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  if El.Elements.Count>0 then
    RaiseMsg(20200104165933,El);
  WriteStrings(Obj,'Tokens',El.Tokens,aContext);
end;

procedure TPCUWriter.WriteImplRepeatUntil(Obj: TJSONObject;
  El: TPasImplRepeatUntil; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementList(Obj,El,'Elements',El.Elements,aContext);
  WriteExpr(Obj,El,'Cond',El.ConditionExpr,aContext);
end;

procedure TPCUWriter.WriteImplIfElse(Obj: TJSONObject; El: TPasImplIfElse;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'Cond',El.ConditionExpr,aContext);
  WriteElementProperty(Obj,El,'Then',El.IfBranch,aContext);
  WriteElementProperty(Obj,El,'Else',El.ElseBranch,aContext);
end;

procedure TPCUWriter.WriteImplWhileDo(Obj: TJSONObject; El: TPasImplWhileDo;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'Cond',El.ConditionExpr,aContext);
  WriteElementProperty(Obj,El,'Body',El.Body,aContext);
end;

procedure TPCUWriter.WriteImplWithDo(Obj: TJSONObject; El: TPasImplWithDo;
  aContext: TPCUWriterContext);
var
  Exprs: TFPList;
  Arr: TJSONArray;
  i: Integer;
  Expr: TPasExpr;
  SubObj: TJSONObject;
begin
  WritePasElement(Obj,El,aContext);

  // expressions
  Exprs:=El.Expressions;
  if (Exprs=nil) or (Exprs.Count=0) then
    RaiseMsg(20200109170419,El);
  Arr:=TJSONArray.Create;
  Obj.Add('Exprs',Arr);
  for i:=0 to Exprs.Count-1 do
    begin
    Expr:=TPasExpr(Exprs[i]);
    SubObj:=TJSONObject.Create;
    Arr.Add(SubObj);
    WriteElement(SubObj,Expr,aContext);
    {$IFDEF EnableStoreExprRef}
    WriteExprCustomData(SubObj,Expr,aContext);
    {$ENDIF}
    end;

  //WriteImplWithScope(Obj,TPasWithScope(EL.CustomData),aContext);

  // body
  WriteElementProperty(Obj,El,'Body',El.Body,aContext);
end;

procedure TPCUWriter.WriteImplWithFlags(Obj: TJSONObject;
  const PropName: string; const Value, DefaultValue: TPasWithExprScopeFlags);
var
  Arr: TJSONArray;
  f: TPasWithExprScopeFlag;
begin
  if Value=DefaultValue then exit;
  Arr:=nil;
  for f in TPasWithExprScopeFlags do
    if (f in Value)<>(f in DefaultValue) then
      AddArrayFlag(Obj,Arr,PropName,PCUResolverWithExprScopeFlagNames[f],f in Value);
end;

procedure TPCUWriter.WriteImplCaseOf(Obj: TJSONObject; El: TPasImplCaseOf;
  aContext: TPCUWriterContext);
var
  Elements: TFPList;
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'Expr',El.CaseExpr,aContext);
  Elements:=El.Elements;
  WriteElementList(Obj,El,'Of',Elements,aContext);
  if El.ElseBranch<>nil then
    begin
    if Elements.Count=0 then
      RaiseMsg(20200104170652,El);
    if Pointer(El.ElseBranch)<>Elements[Elements.Count-1] then
      RaiseMsg(20200104170735,El);
    end
  else if (Elements.Count>0)
      and (TPasElement(Elements[Elements.Count-1]) is TPasImplCaseElse) then
    RaiseMsg(20200105195222,El); // ElseBranch=nil ?
end;

procedure TPCUWriter.WriteImplCaseStatement(Obj: TJSONObject;
  El: TPasImplCaseStatement; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementList(Obj,El,'Exprs',El.Expressions,aContext);
  WriteElementProperty(Obj,El,'Body',El.Body,aContext);
end;

procedure TPCUWriter.WriteImplCaseElse(Obj: TJSONObject; El: TPasImplCaseElse;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementList(Obj,El,'Elements',El.Elements,aContext);
end;

procedure TPCUWriter.WriteImplForLoop(Obj: TJSONObject; El: TPasImplForLoop;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  if El.LoopType<>ltNormal then
    Obj.Add('Loop',PCUForLoopType[El.LoopType]);
  WriteExpr(Obj,El,'Var',El.VariableName,aContext);
  WriteExpr(Obj,El,'Start',El.StartExpr,aContext);
  WriteExpr(Obj,El,'End',El.EndExpr,aContext);
  WriteElementProperty(Obj,El,'Body',El.Body,aContext);
  if El.Variable<>nil then
    RaiseMsg(20200104172120,El);
end;

procedure TPCUWriter.WriteImplAssign(Obj: TJSONObject; El: TPasImplAssign;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  if El.Kind<>akDefault then
    Obj.Add('Kind',PCUAssignKind[El.Kind]);
  WriteExpr(Obj,El,'Left',El.left,aContext);
  WriteExpr(Obj,El,'Right',El.right,aContext);
end;

procedure TPCUWriter.WriteImplSimple(Obj: TJSONObject; El: TPasImplSimple;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'Expr',El.Expr,aContext);
end;

procedure TPCUWriter.WriteImplTry(Obj: TJSONObject; El: TPasImplTry;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementList(Obj,El,'Try',El.Elements,aContext);
  WriteElementProperty(Obj,El,'Finally',El.FinallyExcept,aContext);
  WriteElementProperty(Obj,El,'Else',El.ElseBranch,aContext);
end;

procedure TPCUWriter.WriteImplTryHandler(Obj: TJSONObject;
  El: TPasImplTryHandler; aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementList(Obj,El,'El',El.Elements,aContext);
end;

procedure TPCUWriter.WriteImplExceptOn(Obj: TJSONObject; El: TPasImplExceptOn;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteElementProperty(Obj,El,'Var',El.VarEl,aContext);
  if El.VarEl=nil then
    WriteElType(Obj,El,'VarType',El.TypeEl,aContext);
  WriteElementProperty(Obj,El,'Body',El.Body,aContext);
end;

procedure TPCUWriter.WriteImplRaise(Obj: TJSONObject; El: TPasImplRaise;
  aContext: TPCUWriterContext);
begin
  WritePasElement(Obj,El,aContext);
  WriteExpr(Obj,El,'Obj',El.ExceptObject,aContext);
  WriteExpr(Obj,El,'Addr',El.ExceptAddr,aContext);
end;

constructor TPCUWriter.Create;
begin
  inherited Create;
end;

destructor TPCUWriter.Destroy;
begin
  inherited Destroy;
end;

procedure TPCUWriter.Clear;
begin
  FFirstNewExt:=nil;
  FLastNewExt:=nil;
  FInitialFlags:=nil;
  FElementIdCounter:=0;
  FSourceFilesSorted:=nil;
  FInImplementation:=false;
  inherited Clear;
end;

procedure TPCUWriter.WritePCU(aResolver: TPas2JSResolver;
  aConverter: TPasToJSConverter; InitFlags: TPCUInitialFlags; aStream: TStream;
  Compressed: boolean);
var
  TargetStream: TStream;
var
  aJSON: TJSONObject;
  Comp: Tcompressionstream;
begin
  aJSON:=WriteJSON(aResolver,aConverter,InitFlags);
  TargetStream:=aStream;
  try
    if Compressed then
      TargetStream:=TMemoryStream.Create;
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUWriter.WritePCU create js');
    {$ENDIF}
    Pas2jsFiler.WriteJSON(aJSON,TargetStream,Compressed);
    if Compressed then
      try
        {$IFDEF VerbosePCUFiler}
        writeln('TPCUWriter.WritePCU zip...');
        {$ENDIF}
        Comp:=Tcompressionstream.create(cldefault,aStream);
        try
          Comp.WriteDWord(TargetStream.Size);
          Comp.Write(TMemoryStream(TargetStream).Memory^,TargetStream.Size);
        finally
          Comp.Free;
        end;
      except
        on E: Ecompressionerror do
          RaiseMsg(20180704163113,'compression error: '+E.Message);
      end;
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUWriter.WritePCU END');
    {$ENDIF}
  finally
    if TargetStream<>aStream then
      TargetStream.Free;
    aJSON.Free;
  end;
end;

function TPCUWriter.WriteJSON(aResolver: TPas2JSResolver;
  aConverter: TPasToJSConverter; InitFlags: TPCUInitialFlags): TJSONObject;
var
  Obj, JSMod: TJSONObject;
  aContext: TPCUWriterContext;
begin
  Result:=nil;
  FConverter:=aConverter;
  FResolver:=aResolver;
  FParser:=Resolver.CurrentParser;
  FScanner:=FParser.Scanner;
  FInitialFlags:=InitFlags;

  aContext:=nil;
  Obj:=TJSONObject.Create;
  try
    FJSON:=Obj;
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUWriter.WriteJSON header ...');
    {$ENDIF}
    WriteHeaderMagic(Obj);
    WriteHeaderVersion(Obj);
    WriteGUID(Obj);
    WriteInitialFlags(Obj);
    WriteSrcFiles(Obj);
    // ToDo: WriteUsedModulesPrecompiledChecksums
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUWriter.WriteJSON module ...');
    {$ENDIF}
    aContext:=TPCUWriterContext.Create;
    aContext.ModeSwitches:=InitialFlags.ModeSwitches;
    aContext.BoolSwitches:=InitialFlags.BoolSwitches;
    JSMod:=TJSONObject.Create;
    Obj.Add('Module',JSMod);
    WriteModule(JSMod,aResolver.RootElement,aContext);
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUWriter.WriteJSON footer ...');
    {$ENDIF}
    WriteFinalFlags(Obj);

    Result:=Obj;
  finally
    FJSON:=nil;
    aContext.Free;
    if Result=nil then
      Obj.Free;
  end;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUWriter.WriteJSON END');
  {$ENDIF}
end;

function TPCUWriter.IndexOfSourceFile(const Filename: string): integer;
var
  l, r, m, cmp: Integer;
begin
  l:=0;
  r:=length(FSourceFilesSorted)-1;
  while l<=r do
    begin
    m:=(l+r) div 2;
    cmp:=CompareStr(Filename,FSourceFilesSorted[m].Filename);
    if cmp<0 then
      r:=m-1
    else if cmp>0 then
      l:=m+1
    else
      exit(FSourceFilesSorted[m].Index);
    end;
  Result:=-1;
end;

{ TPCUReader }

procedure TPCUReader.Set_Variable_VarType(RefEl: TPasElement; Data: TObject);
var
  El: TPasVariable absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.VarType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasVariable.VarType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121809,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_AliasType_DestType(RefEl: TPasElement; Data: TObject);
var
  El: TPasAliasType absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.DestType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasAliasType.DestType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121801,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_PointerType_DestType(RefEl: TPasElement; Data: TObject
  );
var
  El: TPasPointerType absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.DestType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasPointerType.DestType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121757,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ArrayType_ElType(RefEl: TPasElement; Data: TObject);
var
  El: TPasArrayType absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.ElType:=TPasType(RefEl);
    if RefEl.Name<>'' then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasArrayType.ElType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121732,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_FileType_ElType(RefEl: TPasElement; Data: TObject);
var
  El: TPasFileType absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.ElType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasFileType.ElType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121726,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_SetType_EnumType(RefEl: TPasElement; Data: TObject);
var
  El: TPasSetType absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.EnumType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasSetType.EnumType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121714,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_Variant_Members(RefEl: TPasElement; Data: TObject);
var
  El: TPasVariant absolute Data;
begin
  if RefEl is TPasRecordType then
    begin
    El.Members:=TPasRecordType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasVariant.Members'){$ENDIF};
    end
  else
    RaiseMsg(20180211121657,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_RecordType_VariantEl(RefEl: TPasElement; Data: TObject
  );
var
  El: TPasRecordType absolute Data;
begin
  if (RefEl is TPasType) or (RefEl.ClassType=TPasVariable) then
    begin
    El.VariantEl:=RefEl;
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasRecordType.VariantEl'){$ENDIF};
    end
  else
    RaiseMsg(20180210205031,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_RecordScope_DefaultProperty(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2jsRecordScope absolute Data;
begin
  if RefEl is TPasProperty then
    Scope.DefaultProperty:=TPasProperty(RefEl) // no AddRef
  else
    RaiseMsg(20190106213412,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_Argument_ArgType(RefEl: TPasElement; Data: TObject);
var
  El: TPasArgument absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.ArgType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasArgument.ArgType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121643,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ClassScope_NewInstanceFunction(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSClassScope absolute Data;
begin
  if RefEl is TPasClassFunction then
    Scope.NewInstanceFunction:=TPasClassFunction(RefEl)
  else
    RaiseMsg(20180214114043,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ClassScope_DirectAncestor(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSClassScope absolute Data;
  AncestorScope: TPas2JSClassScope;
  aClassAncestor: TPasType;
begin
  if not (RefEl is TPasType) then
    RaiseMsg(20180214114823,Scope.Element,GetObjName(RefEl));
  Scope.DirectAncestor:=TPasType(RefEl);
  if Scope.DirectAncestor=nil then exit;

  // set AncestorScope
  aClassAncestor:=Resolver.ResolveAliasType(Scope.DirectAncestor);
  if not (aClassAncestor is TPasClassType) then
    begin
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUReader.Set_ClassScope_DirectAncestor ',GetObjPath(Scope.DirectAncestor),' ClassAnc=',GetObjPath(aClassAncestor));
    {$ENDIF}
    RaiseMsg(20180214114322,Scope.Element,GetObjName(RefEl));
    end;
  AncestorScope:=aClassAncestor.CustomData as TPas2JSClassScope;
  Scope.AncestorScope:=AncestorScope;
  if (AncestorScope<>nil) and (pcsfPublished in Scope.AncestorScope.Flags) then
    Include(Scope.Flags,pcsfPublished);
end;

procedure TPCUReader.Set_ClassScope_DefaultProperty(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSClassScope absolute Data;
begin
  if RefEl is TPasProperty then
    Scope.DefaultProperty:=TPasProperty(RefEl) // no AddRef
  else
    RaiseMsg(20180214115044,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ClassIntfMap_Intf(RefEl: TPasElement; Data: TObject);
var
  Map: TPasClassIntfMap absolute Data;
begin
  if RefEl is TPasClassType then
    Map.Intf:=TPasClassType(RefEl) // no AddRef
  else
    RaiseMsg(20180325125418,Map.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ClassType_AncestorType(RefEl: TPasElement;
  Data: TObject);
var
  El: TPasClassType absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.AncestorType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasClassType.AncestorType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121632,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ClassType_HelperForType(RefEl: TPasElement;
  Data: TObject);
var
  El: TPasClassType absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.HelperForType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasClassType.HelperForType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121612,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ResultElement_ResultType(RefEl: TPasElement; Data: TObject
  );
var
  El: TPasResultElement absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.ResultType:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasResultElement.ResultType'){$ENDIF};
    end
  else
    RaiseMsg(20180211121537,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_PasScope_VisibilityContext(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPasScope absolute Data;
begin
  Scope.VisibilityContext:=RefEl;
end;

procedure TPCUReader.Set_ModScope_AssertClass(RefEl: TPasElement; Data: TObject
  );
var
  Scope: TPas2JSModuleScope absolute Data;
begin
  if RefEl is TPasClassType then
    Scope.AssertClass:=TPasClassType(RefEl)
  else
    RaiseMsg(20180211121441,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ModScope_AssertDefConstructor(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSModuleScope absolute Data;
begin
  if RefEl is TPasConstructor then
    Scope.AssertDefConstructor:=TPasConstructor(RefEl)
  else
    RaiseMsg(20180211123001,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ModScope_AssertMsgConstructor(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSModuleScope absolute Data;
begin
  if RefEl is TPasConstructor then
    Scope.AssertMsgConstructor:=TPasConstructor(RefEl)
  else
    RaiseMsg(20180211123020,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ModScope_RangeErrorClass(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSModuleScope absolute Data;
begin
  if RefEl is TPasClassType then
    Scope.RangeErrorClass:=TPasClassType(RefEl)
  else
    RaiseMsg(20180211123041,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ModScope_RangeErrorConstructor(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSModuleScope absolute Data;
begin
  if RefEl is TPasConstructor then
    Scope.RangeErrorConstructor:=TPasConstructor(RefEl)
  else
    RaiseMsg(20180211123100,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ModScope_SystemTVarRec(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSModuleScope absolute Data;
begin
  if RefEl is TPasRecordType then
    Scope.SystemTVarRec:=TPasRecordType(RefEl)
  else
    RaiseMsg(20190215230826,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ModScope_SystemVarRecs(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSModuleScope absolute Data;
begin
  if RefEl is TPasFunction then
    Scope.SystemVarRecs:=TPasFunction(RefEl)
  else
    RaiseMsg(20190215230857,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_LocalVar(RefEl: TPasElement; Data: TObject);
var
  LocalVar: TPas2JSStoredLocalVar absolute Data;
begin
  LocalVar.Element:=RefEl;
end;

procedure TPCUReader.Set_EnumTypeScope_CanonicalSet(RefEl: TPasElement;
  Data: TObject);
var
  El: TPasEnumType absolute Data;
  Scope: TPasEnumTypeScope;
begin
  if RefEl is TPasSetType then
    begin
    Scope:=El.CustomData as TPasEnumTypeScope;
    Scope.CanonicalSet:=TPasSetType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasEnumTypeScope.CanonicalSet'){$ENDIF};
    end
  else
    RaiseMsg(20180316215238,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_PropertyScope_AncestorProp(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPasPropertyScope absolute Data;
begin
  if RefEl is TPasProperty then
    Scope.AncestorProp:=TPasProperty(RefEl)
  else
    RaiseMsg(20180213214723,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ProcedureScope_ImplProc(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSProcedureScope absolute Data;
begin
  if RefEl is TPasProcedure then
    Scope.ImplProc:=TPasProcedure(RefEl) // no AddRef
  else
    RaiseMsg(20180219140043,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ProcedureScope_Overridden(RefEl: TPasElement;
  Data: TObject);
var
  Scope: TPas2JSProcedureScope absolute Data;
begin
  if RefEl is TPasProcedure then
    Scope.OverriddenProc:=TPasProcedure(RefEl) // no AddRef
  else
    RaiseMsg(20180213215959,Scope.Element,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ExceptOn_TypeEl(RefEl: TPasElement; Data: TObject);
var
  El: TPasImplExceptOn absolute Data;
begin
  if RefEl is TPasType then
    begin
    El.TypeEl:=TPasType(RefEl);
    if RefEl.Parent<>El then
      RefEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasVariable.VarType'){$ENDIF};
    end
  else
    RaiseMsg(20200115214455,El,GetObjName(RefEl));
end;

procedure TPCUReader.Set_ResolvedReference_Declaration(RefEl: TPasElement;
  Data: TObject);
var
  Ref: TResolvedReference absolute Data;
begin
  Ref.Declaration:=RefEl;
end;

procedure TPCUReader.Set_ResolvedReference_CtxConstructor(RefEl: TPasElement;
  Data: TObject);
var
  Ref: TResolvedReference absolute Data;
begin
  if RefEl is TPasType then
    TResolvedRefCtxConstructor(Ref.Context).Typ:=TPasType(RefEl) // no AddRef
  else
    RaiseMsg(20190222010314,Ref.Element,GetObjPath(RefEl));
end;

procedure TPCUReader.Set_ResolvedReference_CtxAttrProc(RefEl: TPasElement;
  Data: TObject);
var
  Ref: TResolvedReference absolute Data;
begin
  if RefEl is TPasConstructor then
    TResolvedRefCtxAttrProc(Ref.Context).Proc:=TPasConstructor(RefEl) // no AddRef
  else
    RaiseMsg(20190222010821,Ref.Element,GetObjPath(RefEl));
end;

procedure TPCUReader.Set_SpecializeTypeData(RefEl: TPasElement; Data: TObject);
var
  SpecData: TPasSpecializeTypeData absolute Data;
begin
  if RefEl is TPasGenericType then
    SpecData.SpecializedType:=TPasGenericType(RefEl) // no AddRef
  else
    RaiseMsg(20200514130809,SpecData.Element,GetObjPath(RefEl));
end;

function TPCUReader.FindPendingSpecialize(Id: integer
  ): TPCUReaderPendingSpecialized;
begin
  Result:=FPendingSpecialize;
  while (Result<>nil) and (Result.Id<>Id) do
    Result:=Result.Next;
end;

function TPCUReader.AddPendingSpecialize(Id: integer; const SpecName: string
  ): TPCUReaderPendingSpecialized;
begin
  if FindPendingSpecialize(Id)<>nil then
    RaiseMsg(20201022214051,SpecName+'='+IntToStr(Id));

  Result:=TPCUReaderPendingSpecialized.Create;
  if FPendingSpecialize<>nil then
    begin
    Result.Next:=FPendingSpecialize;
    FPendingSpecialize.Prev:=Result;
    end;
  Result.Id:=Id;
  Result.SpecName:=SpecName;
  FPendingSpecialize:=Result;
end;

function TPCUReader.CreateSpecializedElement(
  PendSpec: TPCUReaderPendingSpecialized): boolean;
var
  RefParams, ElParams: TFPList;
  i, Id: Integer;
  SpecEl, RefEl: TPasElement;
  Param: TPCUReaderPendingSpecializedParam;
  Ref: TPCUFilerElementRef;
  Obj: TJSONObject;
  GenericEl: TPasGenericType;
begin
  Result:=false;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.CreateSpecializedElement Gen=',GetObjPath(PendSpec.GenericEl),' ',PendSpec.SpecName);
  {$ENDIF}
  RefEl:=PendSpec.RefEl;
  if RefEl=nil then
    begin
    if PendSpec.GenericEl=nil then
      RaiseMsg(20200531101241,PendSpec.SpecName)
    else
      RaiseMsg(20200531101105,PendSpec.GenericEl,PendSpec.SpecName);// nothing uses this specialize
    end;
  if PendSpec.GenericEl=nil then
    // not yet ready
    exit;
  Obj:=PendSpec.Obj;
  if Obj=nil then
    RaiseMsg(20200531101128,PendSpec.GenericEl,PendSpec.SpecName); // specialize missing in JSON

  // resolve params
  RefParams:=PendSpec.Params;
  if RefParams=nil then
    RaiseMsg(20201022215141,PendSpec.GenericEl,PendSpec.SpecName);
  for i:=0 to RefParams.Count-1 do
    begin
    Param:=TPCUReaderPendingSpecializedParam(RefParams[i]);
    if Param.Element<>nil then continue;
    Ref:=GetElReference(Param.Id,RefEl);
    if (Ref=nil) or (Ref.Element=nil) then
      begin
      //writeln('TPCUReader.CreateSpecializedElement SpecName=',PendSpec.SpecName,' Id=',PendSpec.Id,' WAITING for param ',i,': ',Param.Id);
      exit(false);
      end;
    Param.Element:=Ref.Element;
    end;
  // all RefParams resolved -> specialize
  ElParams:=TFPList.Create;
  try
    for i:=0 to RefParams.Count-1 do
      ElParams.Add(TPCUReaderPendingSpecializedParam(RefParams[i]).Element);
    Id:=PendSpec.Id;
    GenericEl:=PendSpec.GenericEl;
    SpecEl:=Resolver.GetSpecializedEl(Resolver.RootElement,GenericEl,ElParams);
    DeletePendingSpecialize(PendSpec);
    AddElReference(Id,RefEl,SpecEl);
  finally
    ElParams.Free;
  end;
  // read child declarations
  ReadExternalReferences(Obj,SpecEl);
  Result:=true;
end;

procedure TPCUReader.DeletePendingSpecialize(
  PendSpec: TPCUReaderPendingSpecialized);
begin
  if FPendingSpecialize=PendSpec then
    FPendingSpecialize:=PendSpec.Next;
  if PendSpec.Prev<>nil then PendSpec.Prev.Next:=PendSpec.Next;
  if PendSpec.Next<>nil then PendSpec.Next.Prev:=PendSpec.Prev;
  PendSpec.Prev:=nil;
  PendSpec.Next:=nil;
  PendSpec.Free;
end;

function TPCUReader.PromiseSpecialize(SpecId: integer; const SpecName: string;
  RefEl, ErrorEl: TPasElement): TPCUReaderPendingSpecialized;
begin
  Result:=FindPendingSpecialize(SpecId);
  if Result=nil then
    Result:=AddPendingSpecialize(SpecId,SpecName)
  else if Result.SpecName<>SpecName then
    RaiseMsg(20200531093342,ErrorEl,'Id='+IntToStr(SpecId)+' Expected SpecName "'+SpecName+'", but was "'+Result.SpecName+'"');

  if Result.RefEl=nil then
    Result.RefEl:=RefEl;
  // Note: cannot specialize before ResolvePendingIdentifierScopes;
end;

procedure TPCUReader.ResolveSpecializedElements(Complete: boolean);

  function GetErrMsg(UnresolvedSpec: TPCUReaderPendingSpecialized): string;
  var
    i: Integer;
    Param: TPCUReaderPendingSpecializedParam;
    Ref: TPCUFilerElementRef;
  begin
    Result:=UnresolvedSpec.SpecName
         +' Id='+IntToStr(UnresolvedSpec.Id)
         +' RefEl='+GetObjPath(UnresolvedSpec.RefEl)
         +' GenericEl='+GetObjPath(UnresolvedSpec.GenericEl)
         +' Params=<';
    for i:=0 to UnresolvedSpec.Params.Count-1 do
      begin
      if i>0 then Result:=Result+',';
      Param:=TPCUReaderPendingSpecializedParam(UnresolvedSpec.Params[i]);
      if Param.Element<>nil then
        Result:=Result+GetObjPath(Param.Element)
      else
        begin
        Result:=Result+'Id='+IntToStr(Param.Id);
        if Param.Id<1 then
          continue;
        Ref:=GetElReference(Param.Id,UnresolvedSpec.GenericEl);
        if Ref=nil then
          begin
          Result:=Result+',Ref=nil';
          continue;
          end;
        Result:=Result+',Ref.Element='+GetObjPath(Ref.Element);
        end;
      end;
    Result:=Result+'>';
  end;

  function PushRefElToParamSpec(PendSpec: TPCUReaderPendingSpecialized): boolean;
  // For example: A<B<...>>
  // B<...> RefEl is A<...>
  // push RefEl of A<...> to B<...>, so that B<...> is created
  var
    i: Integer;
    Param: TPCUReaderPendingSpecializedParam;
    Ref: TPCUFilerElementRef;
    OtherPendSpec: TPCUReaderPendingSpecialized;
  begin
    Result:=false;
    if PendSpec.RefEl=nil then exit;
    for i:=0 to PendSpec.Params.Count-1 do
      begin
      Param:=TPCUReaderPendingSpecializedParam(PendSpec.Params[i]);
      Ref:=GetElReference(Param.Id,PendSpec.GenericEl);
      if (Ref=nil) or (Ref.Element<>nil) then continue;
      OtherPendSpec:=FPendingSpecialize;
      while OtherPendSpec<>nil do
        begin
        if (OtherPendSpec.Id=Param.Id) and (OtherPendSpec.RefEl=nil) then
          begin
          OtherPendSpec.RefEl:=PendSpec.RefEl;
          Result:=true;
          end;
        OtherPendSpec:=OtherPendSpec.Next;
        end;
      end;
  end;

  function FreeTemplateSpecialization(PendSpec: TPCUReaderPendingSpecialized): boolean;
  // checks if PendSpec params are only TPasGenericTemplateType
  // if yes, frees this PendSpec
  var
    i: Integer;
    Param: TPCUReaderPendingSpecializedParam;
    Ref: TPCUFilerElementRef;
  begin
    Result:=true;
    for i:=0 to PendSpec.Params.Count-1 do
      begin
      Param:=TPCUReaderPendingSpecializedParam(PendSpec.Params[i]);
      Ref:=GetElReference(Param.Id,PendSpec.GenericEl);
      if Ref=nil then
        exit(false);
      if not (Ref.Element is TPasGenericTemplateType) then
        exit(false);
      end;
    DeletePendingSpecialize(PendSpec);
  end;

var
  PendSpec, NextPendSpec, UnresolvedSpec: TPCUReaderPendingSpecialized;
  Changed: Boolean;
  Ref: TPCUFilerElementRef;
begin
  repeat
    UnresolvedSpec:=nil;
    Changed:=false;
    PendSpec:=FPendingSpecialize;
    while PendSpec<>nil do
      begin
      NextPendSpec:=PendSpec.Next;

      if PendSpec.RefEl=nil then
        begin
        // no referrer -> use the first element, waiting for this ID
        Ref:=GetElReference(PendSpec.Id,PendSpec.GenericEl);
        if Ref<>nil then
          PendSpec.RefEl:=GetReferrerEl(Ref.Pending);
        end;
      if (PendSpec.RefEl<>nil) and (PendSpec.GenericEl<>nil) then
        begin
        if CreateSpecializedElement(PendSpec) then
          // Note: PendSpec has been freed
          Changed:=true
        else if PushRefElToParamSpec(PendSpec) then
          // one param was a pending specialize waiting for its RefEl
          Changed:=true
        else
          UnresolvedSpec:=PendSpec;
        end
      else if Complete and (PendSpec.RefEl=nil) then
        begin
        if FreeTemplateSpecialization(PendSpec) then
          Changed:=true;
        end;
      PendSpec:=NextPendSpec;
      end;
  until not Changed;
  if Complete then
    UnresolvedSpec:=FPendingSpecialize;
  if UnresolvedSpec<>nil then
    begin
    {$IF defined(VerbosePJUFiler) or defined(VerbosePas2JS)}
    writeln('TPCUReader.ResolveSpecializedElements Complete=',Complete);
    PendSpec:=FPendingSpecialize;
    while PendSpec<>nil do
      begin
      {AllowWriteln}
      writeln('TPCUReader.ResolveSpecializedElements PENDING: ',GetErrMsg(PendSpec));
      {AllowWriteln-}
      PendSpec:=PendSpec.Next;
      end;
    {$ENDIF}
    // a pending specialize cannot resolve its params
    RaiseMsg(20200531101924,UnresolvedSpec.GenericEl,GetErrMsg(UnresolvedSpec));
    end;
end;

function TPCUReader.IsSpecialize(ChildEl: TPasElement): boolean;
begin
  if (ChildEl is TPasGenericType)
      and Resolver.IsSpecialized(TPasGenericType(ChildEl)) then
    exit(true);
  if (ChildEl is TPasProcedure)
      and (TPas2JSProcedureScope(ChildEl.CustomData).SpecializedFromItem<>nil) then
    exit(true);
  Result:=false;
end;

procedure TPCUReader.RaiseMsg(Id: int64; const Msg: string);
var
  E: EPas2JsFilerError;
  s: String;
begin
  s:='['+IntToStr(Id)+'] '+Msg;
  if PCUFilename<>'' then
    s:=s+' file: '+PCUFilename;
  E:=ErrorClass.Create(s);
  E.Owner:=Self;
  {$IFDEF VerbosePCUFiler}
  writeln(ClassName+'/TPCUReader.RaiseMsg ',E.Message);
  {$ENDIF}
  raise E;
end;

function TPCUReader.CheckJSONArray(Data: TJSONData; El: TPasElement;
  const PropName: string): TJSONArray;
begin
  if Data is TJSONArray then exit(TJSONArray(Data));
  if Data=nil then
    RaiseMsg(20180205140943,El,PropName+': nil')
  else
    RaiseMsg(20180205140358,El,PropName+': '+Data.ClassName);
  Result:=nil;
end;

function TPCUReader.CheckJSONObject(Data: TJSONData; Id: int64): TJSONObject;
begin
  if Data is TJSONObject then exit(TJSONObject(Data));
  RaiseMsg(Id);
  Result:=nil;
end;

function TPCUReader.CheckJSONString(Data: TJSONData; Id: int64): String;
begin
  if Data is TJSONString then
    exit(String(Data.AsString));
  RaiseMsg(Id);
  Result:='';
end;

function TPCUReader.ReadString(Obj: TJSONObject; const PropName: string; out
  s: string; El: TPasElement): boolean;
var
  Data: TJSONData;
begin
  s:='';
  Data:=Obj.Find(PropName);
  if Data=nil then exit(false);
  if Data.ClassType=TJSONString then
    begin
    s:=String(Data.AsString);
    exit(true);
    end;
  RaiseMsg(20180205133227,El,PropName+':'+Data.ClassName);
  Result:=false;
end;

function TPCUReader.ReadInteger(Obj: TJSONObject; const PropName: string; out
  i: integer; El: TPasElement): boolean;
var
  C: TClass;
  Data: TJSONData;
begin
  i:=0;
  Data:=Obj.Find(PropName);
  if Data=nil then exit(false);
  C:=Data.ClassType;
  if C=TJSONIntegerNumber then
    begin
    i:=Data.AsInteger;
    exit(true);
    end;
  RaiseMsg(20180205133132,El,PropName+':'+Data.ClassName);
  Result:=false;
end;

function TPCUReader.ReadBoolean(Obj: TJSONObject; const PropName: string; out
  b: boolean; El: TPasElement): boolean;
var
  C: TClass;
  Data: TJSONData;
begin
  b:=false;
  Data:=Obj.Find(PropName);
  if Data=nil then exit(false);
  C:=Data.ClassType;
  if C=TJSONBoolean then
    begin
    b:=Data.AsBoolean;
    exit(true);
    end;
  RaiseMsg(20180207183730,El,PropName+':'+Data.ClassName);
  Result:=false;
end;

function TPCUReader.ReadArray(Obj: TJSONObject; const PropName: string; out
  Arr: TJSONArray; El: TPasElement): boolean;
var
  Data: TJSONData;
begin
  Arr:=nil;
  Data:=Obj.Find(PropName);
  if Data=nil then exit(false);
  if not (Data is TJSONArray) then
    RaiseMsg(20180207144507,El,PropName+':'+Data.ClassName);
  Arr:=TJSONArray(Data);
  Result:=true;
end;

function TPCUReader.ReadObject(Obj: TJSONObject; const PropName: string; out
  SubObj: TJSONObject; El: TPasElement): boolean;
var
  Data: TJSONData;
begin
  SubObj:=nil;
  Data:=Obj.Find(PropName);
  if Data=nil then exit(false);
  if not (Data is TJSONObject) then
    RaiseMsg(20180210212719,El,PropName+':'+Data.ClassName);
  SubObj:=TJSONObject(Data);
  Result:=true;
end;

procedure TPCUReader.ReadArrayFlags(Data: TJSONData; El: TPasElement;
  const PropName: string; out Names: TStringDynArray; out
  Enable: TBooleanDynArray);
const
  IdentStart = ['a'..'z','A'..'Z','_'];
var
  Arr: TJSONArray;
  Cnt, i: Integer;
  s: String;
begin
  Names:=nil;
  Enable:=nil;
  if Data=nil then exit;
  Arr:=CheckJSONArray(Data,El,PropName);
  Cnt:=Arr.Count;
  if Cnt=0 then exit;
  SetLength(Names,Cnt);
  SetLength(Enable,Cnt);
  for i:=0 to Cnt-1 do
    begin
    Data:=Arr[i];
    if not (Data is TJSONString) then
      RaiseMsg(20180202132350,El,PropName+' elements must be string');
    s:=String(TJSONString(Data).AsString);
    if s='' then
      RaiseMsg(20180202133605,El,PropName+' elements must be string');
    if s[1]='-' then
      begin
      Enable[i]:=false;
      system.Delete(s,1,1);
      end
    else
      Enable[i]:=true;
    if not (s[1] in IdentStart) then
      RaiseMsg(20180202133605,El,PropName+' elements must be identifiers');
    Names[i]:=s;
    end;
end;

function TPCUReader.CreateContext: TPCUReaderContext;
begin
  Result:=TPCUReaderContext.Create;
  Result.ModeSwitches:=InitialFlags.ModeSwitches;
  Result.BoolSwitches:=InitialFlags.BoolSwitches;
end;

function TPCUReader.GetElReference(Id: integer; ErrorEl: TPasElement
  ): TPCUFilerElementRef;
begin
  if Id<=0 then
    RaiseMsg(20180221171721,ErrorEl);
  if Id>=length(FElementRefsArray) then
    RaiseMsg(20180221171741,ErrorEl);
  Result:=FElementRefsArray[Id];
end;

function TPCUReader.AddElReference(Id: integer; ErrorEl: TPasElement;
  El: TPasElement): TPCUFilerElementRef;
var
  {$IF defined(VerbosePCUFiler) or defined(memcheck)}
  Node: TAVLTreeNode;
  {$ENDIF}
  Ref: TPCUFilerElementRef;
  RefItem: TPCUFilerPendingElRef;
  PendingElRef: TPCUReaderPendingElRef;
  PendingElListRef: TPCUReaderPendingElListRef;
  PendingElArrRef: TPCUReaderPendingElArrRef;
  PendingElScopeRef: TPCUReaderPendingElScopeRef;
begin
  if Id<=0 then
    RaiseMsg(20180207151233,ErrorEl);
  if Id>1000000 then
    RaiseMsg(20180316090216,ErrorEl,IntToStr(Id));
  if Id>=length(FElementRefsArray) then
    GrowIdToRefsArray(FElementRefsArray,Id);

  Ref:=FElementRefsArray[Id];
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.AddElReference Id=',Id,' El=',GetObjName(El),' ErrorEl=',GetObjName(ErrorEl),' OldRef=',GetObjName(Ref));
  {$ENDIF}
  if Ref=nil then
    begin
    // new target element
    if El<>nil then
      begin
      Ref:=GetElementReference(El,true);
      if Ref.Id=0 then
        Ref.Id:=Id
      else if Ref.Id<>Id then
        RaiseMsg(20180207152251,ErrorEl,IntToStr(Ref.Id)+'<>'+IntToStr(Id));
      end
    else
      begin
      Ref:=TPCUFilerElementRef.Create;
      Ref.Id:=Id;
      end;
    {$IF defined(VerbosePCUFiler) or defined(memcheck)}
    if FElementRefsArray[Id]<>nil then
      RaiseMsg(20180711212859,ErrorEl,IntToStr(Id)+' is not FElementRefsArray[Id]');
    {$ENDIF}
    FElementRefsArray[Id]:=Ref;
    end;
  Result:=Ref;

  if El=nil then
    exit
  else if Ref.Element=nil then
    begin
    Ref.Element:=El;
    {$IF defined(VerbosePCUFiler) or defined(memcheck)}
    Node:=FElementRefs.FindKey(El,@CompareElWithPCUFilerElementRef);
    if Node<>nil then
      RaiseMsg(20180711231646,El,GetObjName(TPCUFilerElementRef(Node.Data).Element));
    {$ENDIF}
    FElementRefs.Add(Ref);

    if Ref.Pending<>nil then
      begin
      // resolve pending references
      while Ref.Pending<>nil do
        begin
        RefItem:=Ref.Pending;
        if RefItem is TPCUReaderPendingElRef then
          begin
          PendingElRef:=TPCUReaderPendingElRef(RefItem);
          PendingElRef.Setter(Ref.Element,PendingElRef.Data);
          end
        else if RefItem is TPCUReaderPendingElListRef then
          begin
          PendingElListRef:=TPCUReaderPendingElListRef(RefItem);
          PendingElListRef.List[PendingElListRef.Index]:=Ref.Element;
          if PendingElListRef.AddRef{$IFDEF CheckPasTreeRefCount}<>''{$ENDIF} then
            Ref.Element.AddRef{$IFDEF CheckPasTreeRefCount}(PendingElListRef.AddRef){$ENDIF};
          end
        else if RefItem is TPCUReaderPendingElArrRef then
          begin
          PendingElArrRef:=TPCUReaderPendingElArrRef(RefItem);
          PendingElArrRef.Arr[PendingElArrRef.Index]:=Ref.Element;
          if PendingElArrRef.AddRef{$IFDEF CheckPasTreeRefCount}<>''{$ENDIF} then
            Ref.Element.AddRef{$IFDEF CheckPasTreeRefCount}(PendingElArrRef.AddRef){$ENDIF};
          end
        else if RefItem is TPCUReaderPendingElScopeRef then
          begin
          PendingElScopeRef:=TPCUReaderPendingElScopeRef(RefItem);
          PendingElScopeRef.References.Add(Ref.Element,PendingElScopeRef.Access);
          end
        else
          RaiseMsg(20180207153056,ErrorEl,RefItem.ClassName);
        Ref.Pending:=RefItem.Next;
        RefItem.Next:=nil;
        RefItem.Free;
        end;
      end;
    end
  else if El<>Ref.Element then
    RaiseMsg(20180207194919,ErrorEl,'Duplicate Id='+IntToStr(Id)+' El='+GetObjName(El)+' Ref.Element='+GetObjName(Ref.Element));
end;

procedure TPCUReader.PromiseSetElReference(Id: integer;
  const Setter: TOnSetElReference; Data: TObject; ErrorEl: TPasElement);
var
  Ref: TPCUFilerElementRef;
  PendingItem: TPCUReaderPendingElRef;
begin
  Ref:=AddElReference(Id,ErrorEl,nil);
  if Ref.Element<>nil then
    begin
    // element was already created -> execute Setter immediately
    Setter(Ref.Element,Data);
    end
  else
    begin
    // element was not yet created -> store Setter
    PendingItem:=TPCUReaderPendingElRef.Create;
    PendingItem.Setter:=Setter;
    PendingItem.Data:=Data;
    PendingItem.ErrorEl:=ErrorEl;
    Ref.AddPending(PendingItem);
    end;
end;

procedure TPCUReader.PromiseSetElListReference(Id: integer; List: TFPList;
  Index: integer; AddRef: TPCUAddRef; ErrorEl: TPasElement);
var
  Ref: TPCUFilerElementRef;
  PendingItem: TPCUReaderPendingElListRef;
begin
  Ref:=AddElReference(Id,ErrorEl,nil);
  if Ref.Element<>nil then
    begin
    // element was already created -> set list item immediately
    List[Index]:=Ref.Element;
    if AddRef{$IFDEF CheckPasTreeRefCount}<>''{$ENDIF} then
      Ref.Element.AddRef{$IFDEF CheckPasTreeRefCount}(AddRef){$ENDIF};
    end
  else
    begin
    // element was not yet created -> store
    PendingItem:=TPCUReaderPendingElListRef.Create;
    PendingItem.List:=List;
    PendingItem.Index:=Index;
    PendingItem.AddRef:=AddRef;
    PendingItem.ErrorEl:=ErrorEl;
    Ref.AddPending(PendingItem);
    end;
end;

procedure TPCUReader.PromiseSetElArrReference(Id: integer;
  Arr: TPasElementArray; Index: integer; AddRef: TPCUAddRef;
  ErrorEl: TPasElement);
var
  Ref: TPCUFilerElementRef;
  PendingItem: TPCUReaderPendingElArrRef;
begin
  Ref:=AddElReference(Id,ErrorEl,nil);
  if Ref.Element<>nil then
    begin
    // element was already created -> set list item immediately
    Arr[Index]:=Ref.Element;
    if AddRef{$IFDEF CheckPasTreeRefCount}<>''{$ENDIF} then
      Ref.Element.AddRef{$IFDEF CheckPasTreeRefCount}(AddRef){$ENDIF};
    end
  else
    begin
    // element was not yet created -> store
    PendingItem:=TPCUReaderPendingElArrRef.Create;
    PendingItem.Arr:=Arr;
    PendingItem.Index:=Index;
    PendingItem.AddRef:=AddRef;
    PendingItem.ErrorEl:=ErrorEl;
    Ref.AddPending(PendingItem);
    end;
end;

procedure TPCUReader.PromiseSetScopeReference(Id: integer;
  References: TPasScopeReferences; Access: TPSRefAccess; ErrorEl: TPasElement);
var
  Ref: TPCUFilerElementRef;
  PendingItem: TPCUReaderPendingElScopeRef;
begin
  Ref:=AddElReference(Id,ErrorEl,nil);
  if Ref.Element<>nil then
    begin
    // element was already created -> add reference immediately
    References.Add(Ref.Element,Access);
    end
  else
    begin
    // element was not yet created -> store
    PendingItem:=TPCUReaderPendingElScopeRef.Create;
    PendingItem.References:=References;
    PendingItem.Access:=Access;
    PendingItem.ErrorEl:=ErrorEl;
    Ref.AddPending(PendingItem);
    end;
end;

procedure TPCUReader.ResolvePendingIdentifierScopes;
var
  i: Integer;
  PendingIdentifierScope: TPCUReaderPendingIdentifierScope;
begin
  for i:=0 to FPendingIdentifierScopes.Count-1 do
    begin
    PendingIdentifierScope:=TPCUReaderPendingIdentifierScope(FPendingIdentifierScopes[i]);
    ReadIdentifierScopeArray(PendingIdentifierScope.Arr,PendingIdentifierScope.Scope);
    end;
  FPendingIdentifierScopes.Clear;
end;

procedure TPCUReader.ResolvePending(Complete: boolean);
var
  Node: TAVLTreeNode;
  Ref: TPCUFilerElementRef;
begin
  ResolvePendingIdentifierScopes;
  ResolveSpecializedElements(Complete);

  // check dangling references
  Node:=FElementRefs.FindLowest;
  while Node<>nil do
    begin
    Ref:=TPCUFilerElementRef(Node.Data);
    Node:=FElementRefs.FindSuccessor(Node);
    if Ref.Pending<>nil then
      begin
      {$IFDEF VerbosePCUFiler}
      writeln('TPCUReader.ResolvePending Ref.Id=',Ref.Id,' Ref.Element=',GetObjName(Ref.Element));
      {$ENDIF}
      if Ref.Pending.ErrorEl<>nil then
        RaiseMsg(20180207194340,Ref.Pending.ErrorEl,IntToStr(Ref.Id))
      else
        RaiseMsg(20180207194341,Ref.Element,IntToStr(Ref.Id))
      end;
    end;
end;

function TPCUReader.GetReferrerEl(PendingElRef: TPCUFilerPendingElRef
  ): TPasElement;
begin
  while PendingElRef<>nil do
    begin
    Result:=PendingElRef.ErrorEl;
    if Result<>nil then exit;
    PendingElRef:=PendingElRef.Next;
    end;
  Result:=nil;
end;

procedure TPCUReader.ReadBuiltInSymbols(Obj: TJSONObject; ErrorEl: TPasElement);
var
  Arr: TJSONArray;
  Data: TJSONData;
  SubObj: TJSONObject;
  aName, s: string;
  bt: TResolverBaseType;
  El: TPasElement;
  Id, i: integer;
  Found: Boolean;
  BuiltInProc: TResElDataBuiltInProc;
  bp: TResolverBuiltInProc;
  pbt: TPas2jsBaseType;
  pbp: TPas2jsBuiltInProc;
begin
  if not ReadArray(Obj,BuiltInNodeName,Arr,ErrorEl) then exit;
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if not (Data is TJSONObject) then
      RaiseMsg(20180215152600,ErrorEl);
    SubObj:=TJSONObject(Data);
    if not ReadString(SubObj,'Name',aName,ErrorEl) then
      RaiseMsg(20180215153027,ErrorEl);
    if not ReadInteger(SubObj,'Id',Id,ErrorEl) then
      RaiseMsg(20180215153028,ErrorEl,aName);
    Found:=false;
    for bt in TResolverBaseType do
      begin
      El:=Resolver.BaseTypes[bt];
      if (El<>nil) and (CompareText(El.Name,aName)=0) then
        begin
        AddElReference(Id,ErrorEl,El);
        Found:=true;
        break;
        end;
      end;
    if not Found then
      begin
      for bp in TResolverBuiltInProc do
        begin
        BuiltInProc:=Resolver.BuiltInProcs[bp];
        if BuiltInProc=nil then continue;
        El:=BuiltInProc.Element;
        if (CompareText(El.Name,aName)=0) then
          begin
          if bp in [bfStrProc,bfStrFunc] then
            begin
            if not ReadString(SubObj,'Type',s,ErrorEl) then
              s:='Proc';
            if (s='Func')<>(bp=bfStrFunc) then continue;
            end;
          AddElReference(Id,ErrorEl,El);
          Found:=true;
          break;
          end;
        end;
      end;
    if not Found then
      begin
      for pbt in TPas2jsBaseType do
        begin
        El:=Resolver.JSBaseTypes[pbt];
        if El=nil then continue;
        if (CompareText(El.Name,aName)=0) then
          begin
          Found:=true;
          AddElReference(Id,ErrorEl,El);
          break;
          end;
        end;
      end;
    if not Found then
      begin
      for pbp in TPas2jsBuiltInProc do
        begin
        BuiltInProc:=Resolver.JSBuiltInProcs[pbp];
        if BuiltInProc=nil then continue;
        El:=BuiltInProc.Element;
        if (CompareText(El.Name,aName)=0) then
          begin
          Found:=true;
          AddElReference(Id,ErrorEl,El);
          break;
          end;
        end;
      end;
    if not Found then
      RaiseMsg(20180216231551,ErrorEl,aName);
    end;
end;

procedure TPCUReader.ReadHeaderMagic(Obj: TJSONObject);
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadHeaderMagic ',Obj.Get('FileType',''));
  {$ENDIF}
  if Obj.Get('FileType','')<>PCUMagic then
    RaiseMsg(20180130201710,'not a PCU file');
end;

procedure TPCUReader.ReadHeaderVersion(Obj: TJSONObject);
begin
  FFileVersion:=Obj.Get('Version',0);
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadHeaderVersion ',FFileVersion);
  {$ENDIF}
  if FFileVersion<1 then
    RaiseMsg(20180130201801,'invalid file version');
  if FFileVersion>PCUVersion then
    RaiseMsg(20180130201822,'file was created by a newer compiler.');
end;

procedure TPCUReader.ReadGUID(Obj: TJSONObject);
var
  s: string;
begin
  if ReadString(Obj,'GUID',s,nil) then
    FGUID:=StringToGUID(s);
end;

procedure TPCUReader.ReadHeaderItem(const PropName: string; Data: TJSONData);
begin
  RaiseMsg(20180202151706,'unknown property "'+PropName+'" '+GetObjName(Data));
end;

function TPCUReader.ReadParserOptions(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TPOptions): TPOptions;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TPOption;
  Found: Boolean;
  i: Integer;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadParserOptions START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TPOption do
      if s=PCUParserOptionNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180202144009,El,'unknown ParserOption "'+s+'"');
    end;
end;

function TPCUReader.ReadModeSwitches(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TModeSwitches): TModeSwitches;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TModeSwitch;
  Found: Boolean;
  i: Integer;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadModeSwitches START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    if (FileVersion<5) and (SameText(s,'multiplescopehelpers')) then
      s:=PCUModeSwitchNames[msMultiHelpers];
    for f in TModeSwitch do
      if s=PCUModeSwitchNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      begin
      if (FileVersion<5) and (SameText(s,'ignoreinterfaces')) then
        // ignore old switch
      else
        RaiseMsg(20180202144054,El,'unknown ModeSwitch "'+s+'"');
      end;
    end;
end;

function TPCUReader.ReadBoolSwitches(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TBoolSwitches): TBoolSwitches;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TBoolSwitch;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadBoolSwitches START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TBoolSwitch do
      if s=PCUBoolSwitchNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180202144116,El,'unknown BoolSwitch "'+s+'"');
    end;
end;

function TPCUReader.ReadConverterOptions(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TPasToJsConverterOptions
  ): TPasToJsConverterOptions;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TPasToJsConverterOption;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadConverterOptions START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TPasToJsConverterOption do
      if s=PCUConverterOptions[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180202144136,'unknown ConverterOption "'+s+'"');
    end;
end;

procedure TPCUReader.ReadTargetPlatform(Data: TJSONData);
var
  p: TPasToJsPlatform;
  s: String;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadTargetPlatform START');
  {$ENDIF}
  s:=CheckJSONString(Data,20180203100215);
  for p in TPasToJsPlatform do
    if s=PCUTargetPlatformNames[p] then
      begin
      InitialFlags.TargetPlatform:=p;
      exit;
      end;
  RaiseMsg(20180202145542,'invalid TargetPlatform');
end;

procedure TPCUReader.ReadTargetProcessor(Data: TJSONData);
var
  p: TPasToJsProcessor;
  s: String;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadTargetProcessor START');
  {$ENDIF}
  s:=CheckJSONString(Data,20180203100235);
  for p in TPasToJsProcessor do
    if s=PCUTargetProcessorNames[p] then
      begin
      InitialFlags.TargetProcessor:=p;
      exit;
      end;
  RaiseMsg(20180202145623,'invalid TargetProcessor');
end;

procedure TPCUReader.ReadSrcFiles(Data: TJSONData);
var
  SourcesArr: TJSONArray;
  i, j: Integer;
  Src: TJSONObject;
  CurFile: TPCUSourceFile;
  Found: Boolean;
  ft: TPCUSourceFileType;
  s: TJSONStringType;
  CurFilename, PropName: string;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadSrcFiles START ');
  {$ENDIF}
  SourcesArr:=CheckJSONArray(Data,nil,'Sources');
  for i:=0 to SourcesArr.Count-1 do
    begin
    Src:=CheckJSONObject(SourcesArr[i],20180203100307);
    CurFile:=TPCUSourceFile.Create;
    FSourceFiles.Add(CurFile);
    if i=0 then
      CurFile.FileType:=sftUnit
    else
      CurFile.FileType:=sftInclude;

    for j:=0 to Src.Count-1 do
      begin
      PropName:=Src.Names[j];
      Data:=Src.Elements[PropName];
      case PropName of
      'Type':
        begin
        s:=CheckJSONString(Data,20180203101322);
        Found:=false;
        for ft in TPCUSourceFileType do
          if s=PCUSourceFileTypeNames[ft] then
            begin
            Found:=true;
            CurFile.FileType:=ft;
            break;
            end;
        if not Found then
          RaiseMsg(20180202144347,'unknown filetype "'+s+'"');
        end;
      'File':
        begin
        CurFilename:=CheckJSONString(Data,20180203100410);
        if CurFilename='' then
          RaiseMsg(20180130203605);
        if length(CurFilename)>MAX_PATH then
          RaiseMsg(20180130203624);
        DoDirSeparators(CurFilename);
        if CurFilename<>ResolveDots(CurFilename) then
          RaiseMsg(20180130203841);
        if ExtractFilenameOnly(CurFilename)='' then
          RaiseMsg(20180130203924);
        CurFile.Filename:=CurFilename;
        end;
      'CheckSum':
        CurFile.Checksum:=Data.AsInt64;
      else
        RaiseMsg(20180202152628,'unknown file property "'+PropName+'"');
      end;
      end;
    end;
end;

function TPCUReader.ReadMemberHints(Obj: TJSONObject; El: TPasElement;
  const DefaultValue: TPasMemberHints): TPasMemberHints;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TPasMemberHint;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadMemberHints START');
  {$ENDIF}
  Data:=Obj.Find('Hints');
  if Data=nil then exit;
  ReadArrayFlags(Data,El,'Hints',Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TPasMemberHint do
      if s=PCUMemberHintNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180205134551,'unknown element Hints "'+s+'"');
    end;
end;

procedure TPCUReader.ReadSrcPos(Obj: TJSONObject; El: TPasElement;
  aContext: TPCUReaderContext);
var
  i, LastLine, LastCol: integer;
  s: string;
  CurLine, CurCol: LongInt;
  p: SizeInt;
begin
  if aContext=nil then ;
  if ReadInteger(Obj,'File',i,El) then
    begin
    if i>=0 then
      El.SourceFilename:=SourceFiles[i].Filename
    else
      El.SourceFilename:='';
    end
  else if El.Parent<>nil then
    El.SourceFilename:=El.Parent.SourceFilename
  else
    El.SourceFilename:='';

  if El.Parent<>nil then
    Resolver.UnmangleSourceLineNumber(El.Parent.SourceLinenumber,LastLine,LastCol)
  else
    begin
    LastLine:=1;
    LastCol:=1;
    end;
  if ReadString(Obj,'Pos',s,El) then
    begin
    p:=Pos(',',s);
    if p>0 then
      begin
      CurLine:=StrToIntDef(LeftStr(s,p-1),LastLine);
      CurCol:=StrToIntDef(copy(s,p+1,length(s)),LastCol);
      end
    else
      begin
      CurLine:=StrToIntDef(s,LastLine);
      CurCol:=LastCol;
      end;
    El.SourceLinenumber:=Resolver.MangleSourceLineNumber(CurLine,CurCol);
    end
  else
    El.SourceLinenumber:=Resolver.MangleSourceLineNumber(LastLine,LastCol);
end;

procedure TPCUReader.ReadPasElement(Obj: TJSONObject; El: TPasElement;
  aContext: TPCUReaderContext);

  function StrToMemberVisibility(const s: string): TPasMemberVisibility;
  var
    vis: TPasMemberVisibility;
  begin
    for vis in TPasMemberVisibility do
      if PCUMemberVisibilityNames[vis]=s then
        exit(vis);
    RaiseMsg(20180205134334,El,s);
  end;

var
  Id: integer;
  s: string;
  DefHints: TPasMemberHints;
begin
  if ReadInteger(Obj,'Id',Id,El) then
    AddElReference(Id,El,El);

  ReadSrcPos(Obj,El,aContext);

  if ReadString(Obj,'Visibility',s,El) then
    El.Visibility:=StrToMemberVisibility(s)
  else
    El.Visibility:=GetDefaultMemberVisibility(El);

  DefHints:=[];
  if El.Parent<>nil then
    DefHints:=El.Parent.Hints;
  El.Hints:=ReadMemberHints(Obj,El,DefHints);

  if ReadString(Obj,'HintMessage',s,El) then
    El.HintMessage:=s;

  if aContext<>nil then ;
end;

procedure TPCUReader.ReadExternalMembers(El: TPasElement; Arr: TJSONArray;
  Members: TFPList);
var
  i, Index, j, k: Integer;
  Data: TJSONData;
  SubObj: TJSONObject;
  Name: string;
  ChildEl: TPasElement;
begin
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if not (Data is TJSONObject) then
      RaiseMsg(20180309173351,El);
    SubObj:=TJSONObject(Data);

    // search element
    if not ReadString(SubObj,'Name',Name,El) then
      RaiseMsg(20180309180233,El,IntToStr(i));
    if not ReadInteger(SubObj,'MId',Index,El) then
      begin
      if SubObj.Find('MId')=nil then
        Index:=0
      else
        RaiseMsg(20180309184629,El,IntToStr(i));
      end;
    if (Index<0) or (Index>=Members.Count) then
      RaiseMsg(20180309184718,El,IntToStr(Index)+' out of bounds 0-'+IntToStr(Members.Count));
    ChildEl:=nil;
    j:=0;
    for k:=0 to Members.Count-1 do
      begin
      ChildEl:=TPasElement(Members[k]);
      if IsSpecialize(ChildEl) then
        // skip specialized type
      else if Index=j then
        break
      else
        inc(j);
      end;
    if Index>j then
      RaiseMsg(20200222102600,El,IntToStr(Index)+' out of bounds');
    if Resolver.GetOverloadName(ChildEl)<>Name then
      RaiseMsg(20180309200800,El,'Expected="'+Name+'", but found "'+Resolver.GetOverloadName(ChildEl)+'" ('+ChildEl.Name+')');

    // read child declarations
    ReadExternalReferences(SubObj,ChildEl);
    end;
end;

procedure TPCUReader.ReadSpecializations(Obj: TJSONObject;
  El: TPasGenericType);
var
  Arr: TJSONArray;
  i: Integer;
  Data: TJSONData;
  SubObj: TJSONObject;
  SpecArr: TJSONArray;
begin
  if (El.GenericTemplateTypes=nil) or (El.GenericTemplateTypes.Count=0) then
    exit;
  if not ReadArray(Obj,'Specs',Arr,El) then
    exit;
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if not (Data is TJSONObject) then
      RaiseMsg(20200512231800,El);
    SubObj:=TJSONObject(Data);
    if not ReadArray(SubObj,'SpecParams',SpecArr,El) then
      RaiseMsg(20200512231927,El,IntToStr(i));

    ReadSpecialization(SubObj,El,SpecArr);
    end;
end;

procedure TPCUReader.ReadSpecialization(Obj: TJSONObject;
  GenEl: TPasGenericType; ParamIDs: TJSONArray);
// called by ReadSpecializations
// create a specialization promise
var
  i, Id: Integer;
  ErrorEl: TPasElement;
  PendSpec: TPCUReaderPendingSpecialized;
  PendParam: TPCUReaderPendingSpecializedParam;
  SpecName: string;
  Ref: TPCUFilerElementRef;
begin
  ErrorEl:=GenEl;
  if ParamIDs.Count=0 then
    RaiseMsg(20200222190934,ErrorEl);
  if not ReadInteger(Obj,'Id',Id,GenEl) then
    RaiseMsg(20200531085133,GenEl);
  if not ReadString(Obj,'SpecName',SpecName,GenEl) then
    RaiseMsg(20200531085134,GenEl);

  PendSpec:=PromiseSpecialize(Id,SpecName,nil,GenEl);
  PendSpec.Obj:=Obj;
  PendSpec.GenericEl:=GenEl;

  Ref:=AddElReference(Id,GenEl,nil);
  Ref.Obj:=Obj;

  PendSpec.Params:=TFPList.Create;
  for i:=0 to ParamIDs.Count-1 do
    begin
    if ParamIDs.Types[i]<>jtNumber then
      RaiseMsg(20200222164327,GenEl,'i='+IntToStr(i)+' '+IntToStr(ord(ParamIDs.Types[i])));
    Id:=ParamIDs[i].AsInteger;
    if Id<=0 then
      RaiseMsg(20200222191724,ErrorEl,IntToStr(i));
    PendParam:=TPCUReaderPendingSpecializedParam.Create;
    PendSpec.Params.Add(PendParam);
    PendParam.Spec:=PendSpec;
    PendParam.Index:=i;
    PendParam.Id:=Id;
    end;

  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadSpecialization Id=',PendSpec.Id,' GenEl=',GetObjPath(PendSpec.GenericEl),' SpecName=',PendSpec.SpecName,' ElRef=',GetObjPath(PendSpec.RefEl));
  {$ENDIF}
  // Note: cannot specialize before ResolvePendingIdentifierScopes;
end;

procedure TPCUReader.ReadExternalReferences(Obj: TJSONObject; El: TPasElement);
var
  Arr: TJSONArray;
  Id: Integer;
  Data: TJSONData;
  SubObj: TJSONObject;
  Intf: TInterfaceSection;
  Name: string;
  Ref: TPCUFilerElementRef;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadExtRefs ',GetObjName(El));
  {$ENDIF}
  if ReadInteger(Obj,'Id',Id,El) then
    begin
    Ref:=AddElReference(Id,El,El);
    if (Ref.Obj<>nil) and (Ref.Obj<>Obj) then
      RaiseMsg(20201025181840,El);
    Ref.Obj:=Obj;
    end;
  if ReadArray(Obj,'El',Arr,El) then
    begin
    if El is TPasDeclarations then
      ReadExternalMembers(El,Arr,TPasDeclarations(El).Declarations)
    else if El is TPasMembersType then
      ReadExternalMembers(El,Arr,TPasMembersType(El).Members)
    else if El is TPasEnumType then
      ReadExternalMembers(El,Arr,TPasEnumType(El).Values)
    else if El is TPasModule then
      begin
      // a Module has only the Interface as child
      if Arr.Count<>1 then
        RaiseMsg(20180309180715,El,IntToStr(Arr.Count));
      Data:=Arr[0];
      if not (Data is TJSONObject) then
        RaiseMsg(20180309180745,El);
      SubObj:=TJSONObject(Data);
      if not ReadString(SubObj,'Name',Name,El) then
        RaiseMsg(20180309180749,El);
      if Name<>'Interface' then
        RaiseMsg(20180309180806,El);
      Intf:=TPasModule(El).InterfaceSection;
      if Intf=nil then
        RaiseMsg(20180309180856,El);
      ReadExternalReferences(SubObj,Intf);
      end
    else
      RaiseMsg(20180309180610,El);
    end;
  if El is TPasGenericType then
    ReadSpecializations(Obj,TPasGenericType(El));
end;

procedure TPCUReader.ReadUsedUnitsInit(Obj: TJSONObject; Section: TPasSection;
  aContext: TPCUReaderContext);
// Note: can be called twice for each section if there are pending used interfaces
var
  Arr: TJSONArray;
  i, Id: Integer;
  Data: TJSONData;
  UsesObj: TJSONObject;
  Name, InFilename, ModuleName: string;
  Use: TPasUsesUnit;
  Module: TPasModule;
begin
  // fetch used units
  if ReadArray(Obj,'Uses',Arr,Section) then
    begin
    SetLength(Section.UsesClause,Arr.Count);
    for i:=0 to length(Section.UsesClause)-1 do
      Section.UsesClause[i]:=nil;
    for i:=0 to Arr.Count-1 do
      begin
      Data:=Arr[i];
      if not (Data is TJSONObject) then
        RaiseMsg(20180307103518,Section,GetObjName(Data));
      UsesObj:=TJSONObject(Data);
      if not ReadString(UsesObj,'Name',Name,Section) then
        RaiseMsg(20180307103629,Section);
      if not IsValidIdent(Name,true,true) then
        RaiseMsg(20180307103937,Section,Name);
      ReadString(UsesObj,'In',InFilename,Section);
      ReadString(UsesObj,'UnitName',ModuleName,Section);
      {$IFDEF VerbosePCUFiler}
      writeln('TPCUReader.ReadUsedUnits ',i,' Name="',Name,'" In="',InFilename,'" ModuleName="',ModuleName,'"');
      {$ENDIF}
      Use:=TPasUsesUnit(CreateElement(TPasUsesUnit,Name,Section));
      Section.UsesClause[i]:=Use;
      // Use.Expr is not needed
      if InFilename<>'' then
        begin
        Use.InFilename:=TPrimitiveExpr(CreateElement(TPrimitiveExpr,'',Use));
        Use.InFilename.Kind:=pekString;
        Use.InFilename.Value:=InFilename;
        end;
      if ModuleName='' then ModuleName:=Name;
      Module:=Resolver.FindModule(Name,Use.Expr,Use.InFilename);
      if Module=nil then
        RaiseMsg(20180307231247,Use);
      Use.Module:=Module;
      Module.AddRef{$IFDEF CheckPasTreeRefCount}('TPasUsesUnit.Module'){$ENDIF};
      if ReadInteger(UsesObj,'Id',Id,Use) then
        AddElReference(Id,Use,Use);
      end;
    Resolver.CheckPendingUsedInterface(Section);
    end;
  if aContext=nil then ;
end;

procedure TPCUReader.ReadIndirectUsedUnits(Obj: TJSONObject;
  Section: TPasSection; aComplete: boolean);
// read external refs from indirectly used units
var
  i: Integer;
  Arr: TJSONArray;
  Data: TJSONData;
  UsesObj: TJSONObject;
  Name: string;
  Module: TPasModule;
  UsedScope: TPas2JSSectionScope;
begin
  if ReadArray(Obj,'IndirectUses',Arr,Section) then
    begin
    for i:=0 to Arr.Count-1 do
      begin
      Data:=Arr[i];
      if not (Data is TJSONObject) then
        RaiseMsg(20180314155716,Section,GetObjName(Data));
      UsesObj:=TJSONObject(Data);
      if not ReadString(UsesObj,'Name',Name,Section) then
        RaiseMsg(20180314155756,Section);
      if not IsValidIdent(Name,true,true) then
        RaiseMsg(20180314155800,Section,Name);
      Module:=Resolver.FindModule(Name,nil,nil);
      if Module=nil then
        RaiseMsg(20180314155840,Section,Name);
      if Module.InterfaceSection=nil then
        begin
        if not aComplete then
          continue;
        {$IF defined(VerbosePCUFiler) or defined(VerbosePJUFiler)}
        writeln('TPCUReader.ReadUsedUnitsFinish Resolver.RootElement=',GetObjPath(Resolver.RootElement),' Section=',GetObjPath(Section));
        {$ENDIF}
        RaiseMsg(20180314155953,Section,'indirect unit "'+Name+'"');
        end;
      UsedScope:=Module.InterfaceSection.CustomData as TPas2JSSectionScope;
      if not UsedScope.Finished then
        RaiseMsg(20180314155954,Section,'indirect unit "'+Name+'"');
      ReadExternalReferences(UsesObj,Module);
      end;
    end;
end;

procedure TPCUReader.ReadUsedUnitsFinish(Obj: TJSONObject;
  Section: TPasSection; aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  Scope, UsedScope: TPas2JSSectionScope;
  i: Integer;
  Use: TPasUsesUnit;
  Module: TPasModule;
  Data: TJSONData;
  UsesObj, ModuleObj: TJSONObject;
begin
  Scope:=Section.CustomData as TPas2JSSectionScope;
  // read external refs from directly used units
  if ReadArray(Obj,'Uses',Arr,Section) then
    begin
    Scope:=Section.CustomData as TPas2JSSectionScope;
    if Scope.UsesFinished then
      RaiseMsg(20180313133931,Section);
    if Section.PendingUsedIntf<>nil then
      RaiseMsg(20180313134142,Section,GetObjName(Section.PendingUsedIntf));
    if Arr.Count<>length(Section.UsesClause) then
      RaiseMsg(20180313134338,IntToStr(Arr.Count)+'<>'+IntToStr(length(Section.UsesClause)));
    for i:=0 to Arr.Count-1 do
    begin
      Data:=Arr[i];
      if not (Data is TJSONObject) then
        RaiseMsg(20180313134409,Section,GetObjName(Data));
      UsesObj:=TJSONObject(Data);
      Use:=Section.UsesClause[i];

      Module:=Use.Module as TPasModule;
      UsedScope:=Module.InterfaceSection.CustomData as TPas2JSSectionScope;
      Scope.UsesScopes.Add(UsedScope);
      if ReadObject(UsesObj,'Module',ModuleObj,Use) then
        ReadExternalReferences(ModuleObj,Module);
      end;
    end;

  // read external refs from indirectly used units
  ReadIndirectUsedUnits(Obj,Section,true);

  Scope.UsesFinished:=true;

  if aContext=nil then ;
end;

procedure TPCUReader.ReadSectionScope(Obj: TJSONObject;
  Scope: TPas2JSSectionScope; aContext: TPCUReaderContext);
begin
  ReadIdentifierScope(Obj,Scope,aContext);
  // not needed: Scope ElevatedLocals
  // not needed: Scope Helpers, autogenerated in ReadClassType
  Scope.BoolSwitches:=ReadBoolSwitches(Obj,Scope.Element,'BoolSwitches',aContext.BoolSwitches);
  Scope.ModeSwitches:=ReadModeSwitches(Obj,Scope.Element,'ModeSwitches',aContext.ModeSwitches);
end;

procedure TPCUReader.ReadSection(Obj: TJSONObject; Section: TPasSection;
  aContext: TPCUReaderContext);
// Note: can be called twice for each section if there are pending used interfaces
var
  Scope: TPas2JSSectionScope;
  i: Integer;
  El: TPasElement;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadSection ',GetObjName(Section));
  {$ENDIF}
  if Section.CustomData=nil then
    begin
    ReadPasElement(Obj,Section,aContext);
    Scope:=TPas2JSSectionScope(Resolver.CreateScope(Section,TPas2JSSectionScope));
    ReadUsedUnitsInit(Obj,Section,aContext);
    if Section.PendingUsedIntf<>nil then exit;
    end
  else
    begin
    Scope:=Section.CustomData as TPas2JSSectionScope;
    if Scope.Finished then
      RaiseMsg(20180308160336,Section);
    if Section.PendingUsedIntf<>nil then
      RaiseMsg(20180308160639,Section,GetObjName(Section.PendingUsedIntf));
    end;
  Resolver.PushScope(Scope);
  try
    // read external references
    ReadUsedUnitsFinish(Obj,Section,aContext);
    // read scope, needs external refs
    ReadSectionScope(Obj,Scope,aContext);
    aContext.BoolSwitches:=Scope.BoolSwitches;
    aContext.ModeSwitches:=Scope.ModeSwitches;
    // read declarations, needs external refs
    ReadDeclarations(Obj,Section,aContext);
  finally
    Resolver.PopScope;
  end;

  Scope.Finished:=true;
  if Section.ClassType=TInterfaceSection then
    begin
    ResolvePending(false);
    Resolver.NotifyPendingUsedInterfaces;
    end
  else if Section.ClassType=TImplementationSection then
    begin
    for i:=0 to FPendingForwardProcs.Count-1 do
      begin
      El:=TPasElement(FPendingForwardProcs[i]);
      Resolver.CheckPendingForwardProcs(El);
      end;
    FPendingForwardProcs.Clear;
    end;
end;

procedure TPCUReader.ReadDeclarations(Obj: TJSONObject;
  Decls: TPasDeclarations; aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  i: Integer;
  Data: TJSONData;
  El: TPasElement;
  C: TClass;
  SubObj: TJSONObject;
begin
  if not ReadArray(Obj,'Declarations',Arr,Decls) then exit;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadDeclarations ',GetObjName(Decls),' ',Arr.Count);
  {$ENDIF}
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if not (Data is TJSONObject) then
      RaiseMsg(20180207182304,Decls,IntToStr(i)+' '+GetObjName(Data));
    SubObj:=TJSONObject(Data);
    El:=ReadNewElement(SubObj,Decls);
    Decls.Declarations.Add(El);
    C:=El.ClassType;
    if C=TPasResString then
      Decls.ResStrings.Add(El)
    else if C=TPasConst then
      Decls.Consts.Add(El)
    else if (C=TPasClassType) or (C=TPasRecordType) then
      Decls.Classes.Add(El)
    else if C.InheritsFrom(TPasType) then
      // not TPasClassType, TPasRecordType !
      Decls.Types.Add(El)
    else if C.InheritsFrom(TPasProcedure) then
      Decls.Functions.Add(El)
    else if C=TPasVariable then
      Decls.Variables.Add(El)
    else if C=TPasProperty then
      Decls.Properties.Add(El)
    else if C=TPasExportSymbol then
      Decls.ExportSymbols.Add(El);
    ReadElement(SubObj,El,aContext);
    end;
end;

function TPCUReader.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement): TPasElement;
var
  Scope: TPasScope;
  Kind: TPasIdentifierKind;
begin
  Result:=AClass.Create(AName,AParent);
  Result.SourceFilename:=SourceFilename;
  {$IFDEF CheckPasTreeRefCount}Result.RefIds.Add('CreateElement');{$ENDIF}
  if (AName<>'')
      and (AClass<>TPasArgument)
      and (AClass<>TPasResultElement)
      and (AClass<>TPasGenericTemplateType) then
    begin
    Scope:=Resolver.TopScope;
    if Scope is TPasIdentifierScope then
      begin
      // add identifier to scope
      // Note: Resolver needs this for specializations
      // The scope identifiers will be later replaced with the values from the
      // pcu, see ResolvePendingIdentifierScopes
      Kind:=PCUDefaultIdentifierKind;
      if Result is TPasProcedure then
        Kind:=pikProc;
      TPasIdentifierScope(Scope).AddIdentifier(AName,Result,Kind);
      end;
    end;
end;

function TPCUReader.ReadElementProperty(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; BaseClass: TPTreeElement; aContext: TPCUReaderContext
  ): TPasElement;
var
  SubObj: TJSONObject;
  s: String;
begin
  if not ReadObject(Obj,PropName,SubObj,Parent) then
    exit(nil);
  Result:=ReadNewElement(SubObj,Parent);
  if not (Result is BaseClass) then
    begin
    s:=GetObjName(Result);
    Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};;
    Result:=nil;
    RaiseMsg(20180211105744,Parent,PropName+' is '+s);
    end;
  ReadElement(SubObj,Result,aContext);
end;

procedure TPCUReader.ReadElementReference(Obj: TJSONObject;
  Instance: TPasElementBase; const PropName: string;
  const Setter: TOnSetElReference);
var
  Data: TJSONData;
  ErrorEl: TPasElement;
  Id: Integer;
begin
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  if Instance is TPasElement then
    ErrorEl:=TPasElement(Instance)
  else if Instance is TResolveData then
    ErrorEl:=TResolveData(Instance).Element
  else if Instance is TPas2JSStoredLocalVar then
    ErrorEl:=TPasElement(TPas2JSStoredLocalVar(Instance).CustomData)
  else
    RaiseMsg(20180211120642,GetObjName(Instance)+'.'+PropName);
  if Data is TJSONIntegerNumber then
    begin
    Id:=Data.AsInteger;
    PromiseSetElReference(Id,Setter,Instance,ErrorEl);
    end
  else
    RaiseMsg(20180211120300,ErrorEl,PropName+' is '+GetObjName(Data));
end;

procedure TPCUReader.ReadElementList(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; ListOfElements: TFPList; AddRef: TPCUAddRef;
  aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  i, Id: Integer;
  Data: TJSONData;
  SubObj: TJSONObject;
  SubEl: TPasElement;
begin
  if not ReadArray(Obj,PropName,Arr,Parent) then exit;
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if Data is TJSONIntegerNumber then
      begin
      // reference
      Id:=Data.AsInteger;
      ListOfElements.Add(nil);
      PromiseSetElListReference(Id,ListOfElements,i,AddRef,Parent);
      end
    else if Data is TJSONObject then
      begin
      SubObj:=TJSONObject(Data);
      SubEl:=ReadNewElement(SubObj,Parent);
      ListOfElements.Add(SubEl);
      ReadElement(SubObj,SubEl,aContext);
      end
    else
      RaiseMsg(20180210201001,Parent,'['+IntToStr(i)+'] is '+GetObjName(Data));
    end;
end;

procedure TPCUReader.ReadElementArray(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; var ArrOfElements: TPasElementArray;
  AddRef: TPCUAddRef; aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  i, Id: Integer;
  Data: TJSONData;
  SubObj: TJSONObject;
  SubEl: TPasElement;
begin
  if not ReadArray(Obj,PropName,Arr,Parent) then exit;
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if Data is TJSONIntegerNumber then
      begin
      // reference
      Id:=Data.AsInteger;
      SetLength(ArrOfElements,i+1);
      ArrOfElements[i]:=nil;
      PromiseSetElArrReference(Id,ArrOfElements,i,AddRef,Parent);
      end
    else if Data is TJSONObject then
      begin
      SubObj:=TJSONObject(Data);
      SubEl:=ReadNewElement(SubObj,Parent);
      SetLength(ArrOfElements,i+1);
      ArrOfElements[i]:=SubEl;
      ReadElement(SubObj,SubEl,aContext);
      end
    else
      RaiseMsg(20180210201001,Parent,'['+IntToStr(i)+'] is '+GetObjName(Data));
    end;
end;

procedure TPCUReader.ReadElType(Obj: TJSONObject; const PropName: string;
  El: TPasElement; const Setter: TOnSetElReference; aContext: TPCUReaderContext
  );
var
  Data: TJSONData;
  Id: Integer;
  SubEl: TPasElement;
  s: String;
  SubObj: TJSONObject;
begin
  if aContext=nil then ;
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  if Data is TJSONIntegerNumber then
    begin
    // reference
    Id:=Data.AsInteger;
    PromiseSetElReference(Id,Setter,El,El);
    end
  else if Data is TJSONObject then
    begin
    // anonymous type
    SubObj:=TJSONObject(Data);
    SubEl:=ReadNewElement(SubObj,El);
    if not (SubEl is TPasType) then
      begin
      s:=GetObjName(SubEl);
      if SubEl<>nil then
        SubEl.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      RaiseMsg(20180210150730,El,PropName+', expected type, but got '+s);
      end;
    ReadElement(SubObj,SubEl,aContext);
    Setter(SubEl,El);
    end
  else
    RaiseMsg(20180207185313,El,PropName+':'+GetObjName(Data));
end;

procedure TPCUReader.ReadStrings(Obj: TJSONObject; El: TPasElement;
  const PropName: string; List: TStrings);
var
  Arr: TJSONArray;
  i: Integer;
  Data: TJSONData;
  s: String;
begin
  if not ReadArray(Obj,PropName,Arr,El) then exit;
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if Data.ClassType=TJSONString then
      begin
      s:=String(Data.AsString);
      List.Add(s);
      end
    else
      RaiseMsg(20200105122556,El,PropName+IntToStr(i)+':'+Data.ClassName);
    end;
end;

function TPCUReader.ReadResolvedRefFlags(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TResolvedReferenceFlags
  ): TResolvedReferenceFlags;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TResolvedReferenceFlag;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadResolvedRefFlags START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TResolvedReferenceFlag do
      if s=PCUResolvedReferenceFlagNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180215134501,'unknown resolvedreference flag "'+s+'"');
    end;
end;

procedure TPCUReader.ReadResolvedReference(Obj: TJSONObject;
  Ref: TResolvedReference; ErrorEl: TPasElement);
var
  Found: Boolean;
  s: string;
  a: TResolvedRefAccess;
begin
  ReadElementReference(Obj,Ref,'RefDecl',@Set_ResolvedReference_Declaration);
  Ref.Flags:=ReadResolvedRefFlags(Obj,ErrorEl,'RefFlags',[]);
  Ref.Access:=rraRead;
  if ReadString(Obj,'RefAccess',s,ErrorEl) then
    begin
    Found:=false;
    for a in TResolvedRefAccess do
      if s=PCUResolvedRefAccessNames[a] then
        begin
        Ref.Access:=a;
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180215134804,ErrorEl,s);
    end;
  if Obj.Find('RefConstructorType')<>nil then
    begin
    Ref.Context:=TResolvedRefCtxConstructor.Create;
    ReadElementReference(Obj,Ref,'RefConstructorType',@Set_ResolvedReference_CtxConstructor);
    end
  else if Obj.Find('RefAttrProc')<>nil then
    begin
    Ref.Context:=TResolvedRefCtxAttrProc.Create;
    ReadElementReference(Obj,Ref,'RefAttrProc',@Set_ResolvedReference_CtxAttrProc);
    end;
  {$IFDEF EnableStoreExprRef}
  if ReadInteger(Obj,'WithEl',i,ErrorEl) then
    begin
    WithElRef:=GetElReference(Id,Scope.Element);
    if (WithElRef=nil) or (WithElRef.Element=nil) then
      RaiseMsg(20200109174947,ErrorEl);
    if not (WithElRef.Element is TPasImplWithDo) then
      RaiseMsg(20200109175135,ErrorEl);
    WithEl:=TPasImplWithDo(WithElRef.Element);
    if not ReadInteger(Obj,'WithId',i,ErrorEl) then
      i:=0;
    if (i<0) or (i>=WithEl.Expressions.Count) then
      RaiseMsg(20200109175240,ErrorEl);
    Ref.WithExprScope:=TPasExpr(WithEl.Expressions[i]);
    end;
  {$ENDIF}
end;

procedure TPCUReader.ReadPasExpr(Obj: TJSONObject; Expr: TPasExpr;
  DefKind: TPasExprKind; aContext: TPCUReaderContext);
var
  Kind: TPasExprKind;
  s: string;
  Op: TExprOpCode;
  Found: Boolean;
begin
  Expr.Kind:=DefKind;
  if ReadString(Obj,'Kind',s,Expr) then
    begin
    Found:=false;
    for Kind in TPasExprKind do
      if s=PCUExprKindNames[Kind] then
        begin
        Expr.Kind:=Kind;
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180208074859,Expr,s);
    end;
  if ReadString(Obj,'Op',s,Expr) then
    begin
    Found:=false;
    for Op in TExprOpCode do
      if s=PCUExprOpCodeNames[Op] then
        begin
        Expr.OpCode:=Op;
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180208074950,Expr,s);
    end;
  Expr.format1:=ReadExpr(Obj,Expr,'format1',aContext);
  Expr.format2:=ReadExpr(Obj,Expr,'format2',aContext);
  ReadPasElement(Obj,Expr,aContext);
end;

procedure TPCUReader.ReadExprCustomData(Obj: TJSONObject; Expr: TPasExpr;
  aContext: TPCUReaderContext);
var
  Ref: TResolvedReference;
  NeedEvalValue: Boolean;
  Value: TResEvalValue;
begin
  if aContext.InGeneric then
    exit;// not needed by generic code
  Ref:=TResolvedReference(Expr.CustomData);
  if Obj.Find('RefDecl')<>nil then
    begin
    Ref:=TResolvedReference.Create;
    Resolver.AddResolveData(Expr,Ref,lkModule);
    ReadResolvedReference(Obj,Ref,Expr);
    end;

  if not ReadBoolean(Obj,'Eval',NeedEvalValue,Expr) then
    NeedEvalValue:=GetDefaultExprHasEvalValue(Expr);
  //writeln('TPCUReader.ReadExprCustomData ',GetElementFullPath(Expr),' ',GetObjName(Expr),' NeedEvalValue=',NeedEvalValue);
  if NeedEvalValue then
    begin
    Value:=Resolver.Eval(Expr,[refAutoConst]);
    if Value<>nil then
      ReleaseEvalValue(Value);
    end;

  if aContext=nil then ;
end;

function TPCUReader.ReadExpr(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; aContext: TPCUReaderContext): TPasExpr;
var
  Data: TJSONData;
  s: string;
  SubObj: TJSONObject;
  El: TPasElement;
begin
  Data:=Obj.Find(PropName);
  if Data=nil then exit(nil);
  if Data is TJSONObject then
    begin
    SubObj:=TJSONObject(Data);
    El:=ReadNewElement(SubObj,Parent);
    if not (El is TPasExpr) then
      begin
      s:=GetObjName(El);
      if El<>nil then
        El.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
      RaiseMsg(20180210152134,Parent,PropName+' got '+s);
      end;
    ReadElement(SubObj,El,aContext);
    Result:=TPasExpr(El);
    // Important: read customdata after parser data
    ReadExprCustomData(SubObj,Result,aContext);
    end
  else
    RaiseMsg(20180207190200,Parent,PropName+':'+GetObjName(Data));
end;

procedure TPCUReader.ReadPasExprArray(Obj: TJSONObject; Parent: TPasElement;
  const PropName: string; var ExprArr: TPasExprArray;
  aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  i: Integer;
  Data: TJSONData;
  SubEl: TPasElement;
  SubObj: TJSONObject;
  Expr: TPasExpr;
begin
  if not ReadArray(Obj,PropName,Arr,Parent) then exit;
  SetLength(ExprArr,Arr.Count);
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if not (Data is TJSONObject) then
      RaiseMsg(20180210173026,Parent,'['+IntToStr(i)+'] is '+GetObjName(Data));
    SubObj:=TJSONObject(Data);
    SubEl:=ReadNewElement(SubObj,Parent);
    if not (SubEl is TPasExpr) then
      RaiseMsg(20180210173026,Parent,'['+IntToStr(i)+'] is '+GetObjName(SubEl));
    Expr:=TPasExpr(SubEl);
    ExprArr[i]:=Expr;
    ReadElement(SubObj,SubEl,aContext);
    // Important: read customdata after parser data
    ReadExprCustomData(SubObj,Expr,aContext);
    end;
end;

procedure TPCUReader.ReadPasScope(Obj: TJSONObject; Scope: TPasScope;
  aContext: TPCUReaderContext);
var
  Data: TJSONData;
  Id: Integer;
begin
  Data:=Obj.Find('VisibilityContext');
  if Data=nil then
    Scope.VisibilityContext:=GetDefaultPasScopeVisibilityContext(Scope)
  else
    begin
    Id:=Data.AsInteger;
    if Id=0 then
      Scope.VisibilityContext:=nil
    else
      ReadElementReference(Obj,Scope,'VisibilityContext',@Set_PasScope_VisibilityContext);
    end;
  if aContext=nil then ;
end;

procedure TPCUReader.ReadScopeReferences(Obj: TJSONObject; Scope: TPasScope;
  const PropName: string; var References: TPasScopeReferences);
var
  Arr: TJSONArray;
  i, Id: Integer;
  Data: TJSONData;
  SubObj: TJSONObject;
  s: string;
  Found: Boolean;
  Access: TPSRefAccess;
  El: TPasElement;
begin
  El:=Scope.Element;
  if References<>nil then
    RaiseMsg(20180302145101,El);
  if not ReadArray(Obj,PropName,Arr,El) then exit;
  References:=TPasScopeReferences.Create(Scope);
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if not (Data is TJSONObject) then
      RaiseMsg(20180221164800,El,GetObjName(Data));
    SubObj:=TJSONObject(Data);
    Data:=SubObj.Find('Id');
    if not (Data is TJSONIntegerNumber) then
      RaiseMsg(20180221171546,El,GetObjName(Data));
    if ReadString(SubObj,'Access',s,El) then
      begin
      Found:=false;
      for Access in TPSRefAccess do
        if s=PCUPSRefAccessNames[Access] then
          begin
          Found:=true;
          break;
          end;
      if not Found then
        RaiseMsg(20180221172333,El,'Access "'+s+'"');
      end
    else
      Access:=PCUDefaultPSRefAccess;
    Id:=Data.AsInteger;
    PromiseSetScopeReference(Id,References,Access,El);
    end;
end;

procedure TPCUReader.ReadIdentifierScopeArray(Arr: TJSONArray;
  Scope: TPasIdentifierScope);
// called after reading module, i.e. all elements are created

  function GetElRef(Id: integer; out DefKind: TPasIdentifierKind;
    out DefName: string): TPCUFilerElementRef;
  begin
    Result:=GetElReference(Id,Scope.Element);
    if (Result=nil) or (Result.Element=nil) then
      RaiseMsg(20180207161358,Scope.Element,'Id not found: '+IntToStr(Id));
    GetDefaultsPasIdentifierProps(Result.Element,DefKind,DefName);
  end;

var
  i, Id: Integer;
  Data: TJSONData;
  ItemObj: TJSONObject;
  s, Name, DefName: string;
  Kind, DefKind: TPasIdentifierKind;
  Ref: TPCUFilerElementRef;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadIdentifierScopeArray ',Arr.Count);
  {$ENDIF}
  Scope.ClearIdentifiers(false);
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if Data is TJSONIntegerNumber then
      begin
      Id:=Data.AsInteger;
      Ref:=GetElRef(Id,DefKind,DefName);
      {$IFDEF VerbosePCUFiler}
      writeln('TPCUReader.ReadIdentifierScopeArray Id=',Id,' ',DefName,' ',DefKind,' ',GetObjName(Ref.Element));
      {$ENDIF}
      Scope.AddIdentifier(DefName,Ref.Element,DefKind);
      end
    else if Data is TJSONObject then
      begin
      ItemObj:=TJSONObject(Data);
      if not ReadInteger(ItemObj,'El',Id,Scope.Element) then
        RaiseMsg(20180207162015,Scope.Element,'missing El:integer');
      Ref:=GetElRef(Id,DefKind,DefName);
      if ReadString(ItemObj,'Kind',s,Scope.Element) then
        Kind:=StrToPasIdentifierKind(s)
      else
        Kind:=DefKind;
      if not ReadString(ItemObj,'Name',Name,Scope.Element) then
        Name:=DefName;
      if Name='' then
        RaiseMsg(20180207162358,Scope.Element,IntToStr(Id));
      Scope.AddIdentifier(Name,Ref.Element,Kind);
      end
    else
      RaiseMsg(20180207154839,Scope.Element,GetObjName(Data));
    end;
end;

procedure TPCUReader.ReadIdentifierScope(Obj: TJSONObject;
  Scope: TPasIdentifierScope; aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  Pending: TPCUReaderPendingIdentifierScope;
begin
  if ReadArray(Obj,'SItems',Arr,Scope.Element) then
    begin
    Pending:=TPCUReaderPendingIdentifierScope.Create;
    Pending.Scope:=Scope;
    Pending.Arr:=Arr;
    FPendingIdentifierScopes.Add(Pending);
    end;
  ReadPasScope(Obj,Scope,aContext);
end;

function TPCUReader.ReadModuleScopeFlags(Obj: TJSONObject; El: TPasElement;
  const DefaultValue: TPasModuleScopeFlags): TPasModuleScopeFlags;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TPasModuleScopeFlag;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadModuleScopeFlags START');
  {$ENDIF}
  Data:=Obj.Find('ScopeFlags');
  if Data=nil then exit;
  ReadArrayFlags(Data,El,'ScopeFlags',Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TPasModuleScopeFlag do
      if s=PCUModuleScopeFlagNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180206114404,'unknown ModuleScopeFlag "'+s+'"');
    end;
end;

procedure TPCUReader.ReadModuleScope(Obj: TJSONObject;
  Scope: TPas2JSModuleScope; aContext: TPCUReaderContext);
var
  aModule: TPasModule;
  SubObj: TJSONObject;
  Cnt, i: Integer;
  LocalVar: TPas2JSStoredLocalVar;
begin
  aModule:=Scope.Element as TPasModule;
  Scope.FirstName:=FirstDottedIdentifier(aModule.Name);
  Scope.Flags:=ReadModuleScopeFlags(Obj,aModule,PCUDefaultModuleScopeFlags);
  Scope.BoolSwitches:=ReadBoolSwitches(Obj,aModule,'BoolSwitches',aContext.BoolSwitches);
  ReadElementReference(Obj,Scope,'AssertClass',@Set_ModScope_AssertClass);
  ReadElementReference(Obj,Scope,'AssertDefConstructor',@Set_ModScope_AssertDefConstructor);
  ReadElementReference(Obj,Scope,'AssertMsgConstructor',@Set_ModScope_AssertMsgConstructor);
  ReadElementReference(Obj,Scope,'RangeErrorClass',@Set_ModScope_RangeErrorClass);
  ReadElementReference(Obj,Scope,'RangeErrorConstructor',@Set_ModScope_RangeErrorConstructor);
  ReadElementReference(Obj,Scope,'SystemTVarRec',@Set_ModScope_SystemTVarRec);
  ReadElementReference(Obj,Scope,'SystemVarRecs',@Set_ModScope_SystemVarRecs);

  if ReadObject(Obj,'LocalVars',SubObj,aModule) then
    begin
    Cnt:=SubObj.Count;
    SetLength(Scope.StoreJSLocalVars,Cnt);
    for i:=0 to Cnt-1 do
      Scope.StoreJSLocalVars[i]:=nil;
    for i:=0 to Cnt-1 do
      begin
      LocalVar:=TPas2JSStoredLocalVar.Create;
      LocalVar.CustomData:=aModule;
      Scope.StoreJSLocalVars[i]:=LocalVar;
      LocalVar.Name:=SubObj.Names[i];
      if not IsValidJSIdentifier(TJSString(LocalVar.Name)) then
        RaiseMsg(20201023015048,aModule);
      ReadElementReference(SubObj,LocalVar,LocalVar.Name,@Set_LocalVar);
      end;
    end;

  ReadPasScope(Obj,Scope,aContext);
end;

procedure TPCUReader.ReadModuleHeader(Data: TJSONData);
var
  Obj: TJSONObject;
  aName, aType: String;
  aModule: TPasModule;
  ModScope: TPas2JSModuleScope;
  aContext: TPCUReaderContext;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadModuleHeader START');
  {$ENDIF}
  CheckJSONObject(Data,20180308140357);
  Obj:=TJSONObject(Data);
  aName:=String(Obj.Get('Name',''));
  aType:=String(Obj.Get('Type',''));
  case aType of
  'Unit': aModule:=TPasModule(CreateElement(TPasModule,aName,nil));
  'Program': aModule:=TPasProgram(CreateElement(TPasProgram,aName,nil));
  'Library': aModule:=TPasLibrary(CreateElement(TPasLibrary,aName,nil));
  else
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUReader.ReadModuleHeader Type="',aType,'"');
    {$ENDIF}
    RaiseMsg(20180203100748);
  end;
  Resolver.RootElement:=aModule;

  aContext:=CreateContext;
  try
    ReadPasElement(Obj,aModule,aContext);

    ModScope:=TPas2JSModuleScope(Resolver.CreateScope(aModule,Resolver.ScopeClass_Module));
    ReadBuiltInSymbols(Obj,aModule);
    ReadModuleScope(Obj,ModScope,aContext);
  finally
    aContext.Free;
  end;

  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadModuleHeader END');
  {$ENDIF}
end;

function TPCUReader.ReadNewElement(Obj: TJSONObject; Parent: TPasElement
  ): TPasElement;

  procedure ReadPrimitive(Kind: TPasExprKind);
  var
    Prim: TPrimitiveExpr;
    Value: string;
  begin
    ReadString(Obj,'Value',Value,Parent);
    Prim:=TPrimitiveExpr(CreateElement(TPrimitiveExpr,'',Parent));
    Prim.Kind:=Kind;
    Prim.Value:=Value;
    Prim.Name:='';
    Result:=Prim;
  end;

  procedure ReadParams(Kind: TPasExprKind);
  begin
    Result:=CreateElement(TParamsExpr,'',Parent);
    TParamsExpr(Result).Kind:=Kind;
  end;

  procedure CreateClassType(Kind: TPasObjKind; const aName: string);
  begin
    Result:=CreateElement(TPasClassType,aName,Parent);
    TPasClassType(Result).ObjKind:=Kind;
  end;

  procedure ReadProc(aClass: TPasProcedureClass; const aName: string);
  begin
    Result:=CreateElement(aClass,aName,Parent);
  end;

  procedure ReadOper(aClass: TPasProcedureClass; const aName: string);
  begin
    Result:=CreateElement(aClass,aName,Parent);
  end;

var
  aType, Name: string;
  ok: Boolean;
begin
  Result:=nil;
  if not ReadString(Obj,'Type',aType,Parent) then
    RaiseMsg(20180210143327,Parent);
  if not ReadString(Obj,'Name',Name,Parent) then
    Name:='';
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadNewElement Parent=',GetObjName(Parent),' Type="',aType,'" Name="',Name,'"');
  {$ENDIF}
  ok:=false;
  try
    case aType of
    'Unary':
      Result:=CreateElement(TUnaryExpr,Name,Parent);
    'Binary':
      begin
      Result:=CreateElement(TBinaryExpr,Name,Parent);
      TBinaryExpr(Result).Kind:=pekBinary;
      TBinaryExpr(Result).OpCode:=eopAdd;
      end;
    'Ident': ReadPrimitive(pekIdent);
    'Number': ReadPrimitive(pekNumber);
    'String': ReadPrimitive(pekString);
    'Bool':
      begin
      Result:=CreateElement(TBoolConstExpr,'',Parent);
      TBoolConstExpr(Result).Kind:=pekBoolConst;
      TBoolConstExpr(Result).Value:=false;
      end;
    'False','True':
      begin
      Result:=CreateElement(TBoolConstExpr,'',Parent);
      TBoolConstExpr(Result).Kind:=pekBoolConst;
      TBoolConstExpr(Result).Value:=aType='True';
      end;
    'Nil':
      begin
      Result:=CreateElement(TNilExpr,'nil',Parent);
      TNilExpr(Result).Kind:=pekNil;
      end;
    'Inherited':
      begin
      Result:=CreateElement(TInheritedExpr,'',Parent);
      TInheritedExpr(Result).Kind:=pekInherited;
      end;
    'Self':
      begin
      Result:=CreateElement(TSelfExpr,'',Parent);
      TSelfExpr(Result).Kind:=pekSelf;
      end;
    'A[]':
      ReadParams(pekArrayParams);
    'F()':
      ReadParams(pekFuncParams);
    '[]':
      ReadParams(pekSet);
    'ProcExpr':
      Result:=CreateElement(TProcedureExpr,Name,Parent);
    'RecValues':
      begin
      Result:=CreateElement(TRecordValues,'',Parent);
      TRecordValues(Result).Kind:=pekListOfExp;
      end;
    'ArrValues':
      begin
      Result:=CreateElement(TArrayValues,'',Parent);
      TArrayValues(Result).Kind:=pekListOfExp;
      end;
    'ResString':
      Result:=CreateElement(TPasResString,Name,Parent);
    'Alias':
      Result:=CreateElement(TPasAliasType,Name,Parent);
    'Pointer':
      Result:=CreateElement(TPasPointerType,Name,Parent);
    'TypeAlias':
      Result:=CreateElement(TPasTypeAliasType,Name,Parent);
    'ClassOf':
      Result:=CreateElement(TPasClassOfType,Name,Parent);
    'Specialize':
      Result:=CreateElement(TPasSpecializeType,Name,Parent);
    'InlineSpecialize':
      Result:=CreateElement(TInlineSpecializeExpr,Name,Parent);
    'RangeType':
      Result:=CreateElement(TPasRangeType,Name,Parent);
    'ArrType':
      Result:=CreateElement(TPasArrayType,Name,Parent);
    'File':
      Result:=CreateElement(TPasFileType,Name,Parent);
    'EnumV':
      Result:=CreateElement(TPasEnumValue,Name,Parent);
    'EnumType':
      Result:=CreateElement(TPasEnumType,Name,Parent);
    'SetType':
      Result:=CreateElement(TPasSetType,Name,Parent);
    'RecVariant':
      Result:=CreateElement(TPasVariant,Name,Parent);
    'Record':
      Result:=CreateElement(TPasRecordType,Name,Parent);
    'Object': CreateClassType(okObject,Name);
    'Class': CreateClassType(okClass,Name);
    'Interface': CreateClassType(okInterface,Name);
    'ClassHelper': CreateClassType(okClassHelper,Name);
    'RecordHelper': CreateClassType(okRecordHelper,Name);
    'TypeHelper': CreateClassType(okTypeHelper,Name);
    'DispInterface': CreateClassType(okDispInterface,Name);
    'Arg':
      Result:=CreateElement(TPasArgument,Name,Parent);
    'ProcType':
      Result:=CreateElement(TPasProcedureType,Name,Parent);
    'FuncType':
      Result:=CreateElement(TPasFunctionType,Name,Parent);
    'Result':
      Result:=CreateElement(TPasResultElement,Name,Parent);
    'StringType':
      Result:=CreateElement(TPasStringType,Name,Parent);
    'Var':
      Result:=CreateElement(TPasVariable,Name,Parent);
    'Export':
      Result:=CreateElement(TPasExportSymbol,Name,Parent);
    'Const':
      Result:=CreateElement(TPasConst,Name,Parent);
    'Property':
      Result:=CreateElement(TPasProperty,Name,Parent);
    'MethodRes':
      Result:=CreateElement(TPasMethodResolution,Name,Parent);
    'Procedure': ReadProc(TPasProcedure,Name);
    'ClassProcedure': ReadProc(TPasClassProcedure,Name);
    'Function': ReadProc(TPasFunction,Name);
    'ClassFunction': ReadProc(TPasClassFunction,Name);
    'Constructor': ReadProc(TPasConstructor,Name);
    'ClassConstructor': ReadProc(TPasClassConstructor,Name);
    'Destructor': ReadProc(TPasDestructor,Name);
    'ClassDestructor': ReadProc(TPasClassDestructor,Name);
    'AnonymousProcedure': ReadProc(TPasAnonymousProcedure,Name);
    'AnonymousFunction': ReadProc(TPasAnonymousFunction,Name);
    'Operator': ReadOper(TPasOperator,Name);
    'ClassOperator': ReadOper(TPasClassOperator,Name);
    'Attributes':
      Result:=CreateElement(TPasAttributes,Name,Parent);
    'ImplCmd':
      Result:=CreateElement(TPasImplCommand,Name,Parent);
    'Begin':
      Result:=CreateElement(TPasImplBeginBlock,Name,Parent);
    'Asm':
      Result:=CreateElement(TPasImplAsmStatement,Name,Parent);
    'Repeat':
      Result:=CreateElement(TPasImplRepeatUntil,Name,Parent);
    'If':
      Result:=CreateElement(TPasImplIfElse,Name,Parent);
    'While':
      Result:=CreateElement(TPasImplWhileDo,Name,Parent);
    'With':
      Result:=CreateElement(TPasImplWithDo,Name,Parent);
    'CaseOf':
      Result:=CreateElement(TPasImplCaseOf,Name,Parent);
    'CaseSt':
      Result:=CreateElement(TPasImplCaseStatement,Name,Parent);
    'CaseElse':
      Result:=CreateElement(TPasImplCaseElse,Name,Parent);
    'ForLoop':
      Result:=CreateElement(TPasImplForLoop,Name,Parent);
    'Assign':
      Result:=CreateElement(TPasImplAssign,Name,Parent);
    'Simple':
      Result:=CreateElement(TPasImplSimple,Name,Parent);
    'Try':
      Result:=CreateElement(TPasImplTry,Name,Parent);
    'Finally':
      Result:=CreateElement(TPasImplTryFinally,Name,Parent);
    'Except':
      Result:=CreateElement(TPasImplTryExcept,Name,Parent);
    'ExceptElse':
      Result:=CreateElement(TPasImplTryExceptElse,Name,Parent);
    'ExceptOn':
      Result:=CreateElement(TPasImplExceptOn,Name,Parent);
    'Raise':
      Result:=CreateElement(TPasImplRaise,Name,Parent);
    else
      RaiseMsg(20200514220001,Parent,'unknown type "'+LeftStr(aType,100)+'"');
    end;
    ok:=true;
  finally
    if not ok then
      if Result<>nil then
        begin
        Result.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
        Result:=nil;
        end;
  end;
end;

procedure TPCUReader.ReadElement(Obj: TJSONObject; El: TPasElement;
  aContext: TPCUReaderContext);
var
  C: TClass;
begin
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadElement El=',GetObjName(El),' Name="',El.Name,'"');
  {$ENDIF}
  C:=El.ClassType;
  if C=TUnaryExpr then
    ReadUnaryExpr(Obj,TUnaryExpr(El),aContext)
  else if C=TBinaryExpr then
    ReadBinaryExpr(Obj,TBinaryExpr(El),aContext)
  else if C=TPrimitiveExpr then
    ReadPrimitiveExpr(Obj,TPrimitiveExpr(El),aContext)
  else if C=TBoolConstExpr then
    ReadBoolConstExpr(Obj,TBoolConstExpr(El),aContext)
  else if C=TNilExpr then
    ReadPasExpr(Obj,TNilExpr(El),pekNil,aContext)
  else if C=TInheritedExpr then
    ReadPasExpr(Obj,TInheritedExpr(El),pekInherited,aContext)
  else if C=TSelfExpr then
    ReadPasExpr(Obj,TSelfExpr(El),pekSelf,aContext)
  else if C=TParamsExpr then
    ReadParamsExpr(Obj,TParamsExpr(El),aContext)
  else if C=TProcedureExpr then
    ReadProcedureExpr(Obj,TProcedureExpr(El),aContext)
  else if C=TRecordValues then
    ReadRecordValues(Obj,TRecordValues(El),aContext)
  else if C=TArrayValues then
    ReadArrayValues(Obj,TArrayValues(El),aContext)
  else if C=TPasResString then
    ReadResString(Obj,TPasResString(El),aContext)
  else if C=TPasAliasType then
    ReadAliasType(Obj,TPasAliasType(El),aContext)
  else if C=TPasPointerType then
    ReadPointerType(Obj,TPasPointerType(El),aContext)
  else if C=TPasTypeAliasType then
    ReadAliasType(Obj,TPasTypeAliasType(El),aContext)
  else if C=TPasClassOfType then
    ReadAliasType(Obj,TPasClassOfType(El),aContext)
  else if C=TPasSpecializeType then
    ReadSpecializeType(Obj,TPasSpecializeType(El),aContext)
  else if C=TInlineSpecializeExpr then
    ReadInlineSpecializeExpr(Obj,TInlineSpecializeExpr(El),aContext)
  else if C=TPasRangeType then
    ReadRangeType(Obj,TPasRangeType(El),aContext)
  else if C=TPasArrayType then
    ReadArrayType(Obj,TPasArrayType(El),aContext)
  else if C=TPasFileType then
    ReadFileType(Obj,TPasFileType(El),aContext)
  else if C=TPasEnumValue then
    ReadEnumValue(Obj,TPasEnumValue(El),aContext)
  else if C=TPasEnumType then
    ReadEnumType(Obj,TPasEnumType(El),aContext)
  else if C=TPasSetType then
    ReadSetType(Obj,TPasSetType(El),aContext)
  else if C=TPasVariant then
    ReadRecordVariant(Obj,TPasVariant(El),aContext)
  else if C=TPasRecordType then
    ReadRecordType(Obj,TPasRecordType(El),aContext)
  else if C=TPasClassType then
    ReadClassType(Obj,TPasClassType(El),aContext)
  else if C=TPasArgument then
    ReadArgument(Obj,TPasArgument(El),aContext)
  else if C=TPasProcedureType then
    ReadProcedureType(Obj,TPasProcedureType(El),aContext)
  else if C=TPasFunctionType then
    ReadFunctionType(Obj,TPasFunctionType(El),aContext)
  else if C=TPasResultElement then
    ReadResultElement(Obj,TPasResultElement(El),aContext)
  else if C=TPasStringType then
    ReadStringType(Obj,TPasStringType(El),aContext)
  else if C=TPasVariable then
    ReadVariable(Obj,TPasVariable(El),aContext)
  else if C=TPasExportSymbol then
    ReadExportSymbol(Obj,TPasExportSymbol(El),aContext)
  else if C=TPasConst then
    ReadConst(Obj,TPasConst(El),aContext)
  else if C=TPasProperty then
    ReadProperty(Obj,TPasProperty(El),aContext)
  else if C=TPasMethodResolution then
    ReadMethodResolution(Obj,TPasMethodResolution(El),aContext)
  else if C.InheritsFrom(TPasProcedure) then
    ReadProcedure(Obj,TPasProcedure(El),aContext)
  else if (C=TPasOperator) or (C=TPasClassOperator) then
    ReadOperator(Obj,TPasOperator(El),aContext)
  else if C=TPasAttributes then
    ReadAttributes(Obj,TPasAttributes(El),aContext)
  else if C=TPasImplCommand then
    ReadImplCommand(Obj,TPasImplCommand(El),aContext)
  else if C=TPasImplBeginBlock then
    ReadImplBeginBlock(Obj,TPasImplBeginBlock(El),aContext)
  else if C=TPasImplAsmStatement then
    ReadImplAsmStatement(Obj,TPasImplAsmStatement(El),aContext)
  else if C=TPasImplRepeatUntil then
    ReadImplRepeatUntil(Obj,TPasImplRepeatUntil(El),aContext)
  else if C=TPasImplIfElse then
    ReadImplIfElse(Obj,TPasImplIfElse(El),aContext)
  else if C=TPasImplWhileDo then
    ReadImplWhileDo(Obj,TPasImplWhileDo(El),aContext)
  else if C=TPasImplWithDo then
    ReadImplWithDo(Obj,TPasImplWithDo(El),aContext)
  else if C=TPasImplCaseOf then
    ReadImplCaseOf(Obj,TPasImplCaseOf(El),aContext)
  else if C=TPasImplCaseStatement then
    ReadImplCaseStatement(Obj,TPasImplCaseStatement(El),aContext)
  else if C=TPasImplCaseElse then
    ReadImplCaseElse(Obj,TPasImplCaseElse(El),aContext)
  else if C=TPasImplForLoop then
    ReadImplForLoop(Obj,TPasImplForLoop(El),aContext)
  else if C=TPasImplAssign then
    ReadImplAssign(Obj,TPasImplAssign(El),aContext)
  else if C=TPasImplSimple then
    ReadImplSimple(Obj,TPasImplSimple(El),aContext)
  else if C=TPasImplTry then
    ReadImplTry(Obj,TPasImplTry(El),aContext)
  else if (C=TPasImplTryFinally)
      or (C=TPasImplTryExcept)
      or (C=TPasImplTryExceptElse) then
    ReadImplTryHandler(Obj,TPasImplTryHandler(El),aContext)
  else if C=TPasImplExceptOn then
    ReadImplExceptOn(Obj,TPasImplExceptOn(El),aContext)
  else if C=TPasImplRaise then
    ReadImplRaise(Obj,TPasImplRaise(El),aContext)
  else
    RaiseMsg(20180210143758,El,'unknown type "'+GetObjPath(El)+'"');
end;

function TPCUReader.ReadModule(Obj: TJSONObject; aContext: TPCUReaderContext
  ): boolean;
var
  aModule: TPasModule;

  function CreateOrContinueSection(const PropName: string; var Section: TPasSection;
     SectionClass: TPasSectionClass; MustExist: boolean): boolean;
  var
    SubObj: TJSONObject;
  begin
    if not ReadObject(Obj,PropName,SubObj,aModule) then
      begin
      if MustExist then
        RaiseMsg(20180308142146,aModule);
      exit;
      end;
    if Section=nil then
      Section:=TPasSection(CreateElement(SectionClass,'',aModule));
    ReadSection(SubObj,Section,aContext);
    Result:=Section.PendingUsedIntf=nil;
  end;

  procedure ReadInitialFinal(Obj: TJSONObject; Block: TPasImplBlock;
    const PropPrefix: string);
  var
    Scope: TPas2JSInitialFinalizationScope;
    ImplJS: TPas2JSPrecompiledJS;
    Sub: TJSONObject;
  begin
    Scope:=TPas2JSInitialFinalizationScope(Resolver.CreateScope(Block,Resolver.ScopeClass_InitialFinalization));
    Block.CustomData:=Scope;
    ImplJS:=TPas2JSPrecompiledJS.Create;
    Scope.ImplJS:=ImplJS;
    if FileVersion<7 then
      begin
      ReadScopeReferences(Obj,Scope,PropPrefix+'Refs',Scope.References);
      ReadString(Obj,PropPrefix+'JS',ImplJS.BodyJS,Block);
      end
    else if ReadObject(Obj,PropPrefix,Sub,Block) then
      begin
      ReadScopeReferences(Sub,Scope,'Refs',Scope.References);
      ReadPrecompiledJS(Sub,Block,ImplJS,aContext);
      end;
  end;

var
  ModScope: TPas2JSModuleScope;
  OldBoolSwitches: TBoolSwitches;
  Prog: TPasProgram;
  Lib: TPasLibrary;
  OldModeSwitches: TModeSwitches;
begin
  Result:=false;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadModule START ');
  {$ENDIF}
  aModule:=Resolver.RootElement;
  ModScope:=aModule.CustomData as TPas2JSModuleScope;

  OldBoolSwitches:=aContext.BoolSwitches;
  aContext.BoolSwitches:=ModScope.BoolSwitches;
  OldModeSwitches:=aContext.ModeSwitches;
  try
    // read sections
    if aModule.ClassType=TPasProgram then
      begin
      // start or continue ProgramSection
      Prog:=TPasProgram(aModule);
      if not CreateOrContinueSection('Program',TPasSection(Prog.ProgramSection),
          TProgramSection,true) then
        exit; // pending uses interfaces -> pause
      end
    else if aModule.ClassType=TPasLibrary then
      begin
      // start or continue LibrarySection
      Lib:=TPasLibrary(aModule);
      if not CreateOrContinueSection('Library',TPasSection(Lib.LibrarySection),
          TLibrarySection,true) then
        exit; // pending uses interfaces -> pause
      end
    else
      begin
      // unit
      if aModule.ImplementationSection=nil then
        begin
        // start or continue unit Interface
        if not CreateOrContinueSection('Interface',TPasSection(aModule.InterfaceSection),
            TInterfaceSection,true) then
          exit; // pending uses interfaces -> pause
        end;
      // start or continue unit Implementation
      if not CreateOrContinueSection('Implementation',TPasSection(aModule.ImplementationSection),
          TImplementationSection,false) then
        exit; // pending uses interfaces -> pause
      end;
    if (Obj.Find('Init')<>nil)
        or ((FileVersion<7) and (Obj.Find('InitJS')<>nil)) then
      begin
      aModule.InitializationSection:=TInitializationSection(CreateElement(TInitializationSection,'',aModule));
      ReadInitialFinal(Obj,aModule.InitializationSection,'Init');
      end;
    if (Obj.Find('Final')<>nil)
        or ((FileVersion<7) and (Obj.Find('FinalJS')<>nil)) then
      begin
      aModule.FinalizationSection:=TFinalizationSection(CreateElement(TFinalizationSection,'',aModule));
      ReadInitialFinal(Obj,aModule.FinalizationSection,'Final');
      end;
  finally
    aContext.BoolSwitches:=OldBoolSwitches;
    aContext.ModeSwitches:=OldModeSwitches;
  end;

  ResolvePending(true);
  Result:=true;
end;

procedure TPCUReader.ReadUnaryExpr(Obj: TJSONObject; Expr: TUnaryExpr;
  aContext: TPCUReaderContext);
begin
  Expr.OpCode:=eopAdd;
  Expr.Kind:=pekUnary;
  ReadPasExpr(Obj,Expr,pekUnary,aContext);
  Expr.Operand:=ReadExpr(Obj,Expr,'Operand',aContext);
end;

procedure TPCUReader.ReadPrimitiveExpr(Obj: TJSONObject; Expr: TPrimitiveExpr;
  aContext: TPCUReaderContext);
begin
  ReadPasExpr(Obj,Expr,Expr.Kind,aContext);
end;

procedure TPCUReader.ReadBinaryExpr(Obj: TJSONObject; Expr: TBinaryExpr;
  aContext: TPCUReaderContext);
begin
  ReadPasExpr(Obj,Expr,pekBinary,aContext);
  Expr.left:=ReadExpr(Obj,Expr,'Left',aContext);
  Expr.right:=ReadExpr(Obj,Expr,'Right',aContext);
end;

procedure TPCUReader.ReadBoolConstExpr(Obj: TJSONObject; Expr: TBoolConstExpr;
  aContext: TPCUReaderContext);
var
  aType: string;
begin
  ReadPasExpr(Obj,Expr,pekBoolConst,aContext);
  if not ReadString(Obj,'Type',aType,Expr) then
    RaiseMsg(20200515150504,Expr);
  if aType='Bool' then
    ReadBoolean(Obj,'Value',Expr.Value,Expr);
end;

procedure TPCUReader.ReadParamsExpr(Obj: TJSONObject; Expr: TParamsExpr;
  aContext: TPCUReaderContext);
begin
  ReadPasExpr(Obj,Expr,Expr.Kind,aContext);
  Expr.Value:=ReadExpr(Obj,Expr,'Value',aContext);
  ReadPasExprArray(Obj,Expr,'Params',Expr.Params,aContext);
end;

procedure TPCUReader.ReadProcedureExpr(Obj: TJSONObject; Expr: TProcedureExpr;
  aContext: TPCUReaderContext);
begin
  ReadPasExpr(Obj,Expr,Expr.Kind,aContext);
  Expr.Proc:=TPasAnonymousProcedure(ReadElementProperty(Obj,Expr,'Proc',TPasAnonymousProcedure,aContext));
end;

procedure TPCUReader.ReadRecordValues(Obj: TJSONObject; Expr: TRecordValues;
  aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  i: Integer;
  Data: TJSONData;
  SubObj: TJSONObject;
  aName: string;
begin
  ReadPasExpr(Obj,Expr,pekListOfExp,aContext);
  if ReadArray(Obj,'Fields',Arr,Expr) then
    begin
    SetLength(Expr.Fields,Arr.Count);
    for i:=0 to Arr.Count-1 do
      begin
      Data:=Arr[i];
      if not (Data is TJSONObject) then
        RaiseMsg(20180210173636,Expr,'['+IntToStr(i)+'] is '+GetObjName(Data));
      SubObj:=TJSONObject(Data);
      if not ReadString(SubObj,'Name',aName,Expr) then
        RaiseMsg(20201204144308,Expr);
      Expr.Fields[i].NameExp:=ReadExpr(SubObj,Expr,'NameExpr',aContext) as TPrimitiveExpr;
      Expr.Fields[i].ValueExp:=ReadExpr(SubObj,Expr,'ValueExpr',aContext);
      end;
    end;
end;

procedure TPCUReader.ReadArrayValues(Obj: TJSONObject; Expr: TArrayValues;
  aContext: TPCUReaderContext);
begin
  ReadPasExpr(Obj,Expr,pekListOfExp,aContext);
  ReadPasExprArray(Obj,Expr,'Values',Expr.Values,aContext);
end;

procedure TPCUReader.ReadResString(Obj: TJSONObject; El: TPasResString;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  El.Expr:=ReadExpr(Obj,El,'Expr',aContext);
end;

procedure TPCUReader.ReadGenericTemplateTypes(Obj: TJSONObject;
  Parent: TPasElement; var GenericTemplateTypes: TFPList;
  aContext: TPCUReaderContext);
var
  TemplArr: TJSONArray;
  i: Integer;
  TemplObj: TJSONObject;
  GenTypeName: string;
  GenType: TPasGenericTemplateType;
begin
  if not ReadArray(Obj,'GenericTemplateTypes',TemplArr,Parent) then exit;
  if GenericTemplateTypes=nil then
    GenericTemplateTypes:=TFPList.Create;
  for i:=0 to TemplArr.Count-1 do
    begin
    TemplObj:=CheckJSONObject(TemplArr[i],20190720224105);
    if not ReadString(TemplObj,'Name',GenTypeName,Parent) or (GenTypeName='') then
      RaiseMsg(20190720224130,Parent,IntToStr(i));
    GenType:=TPasGenericTemplateType(CreateElement(TPasGenericTemplateType,GenTypeName,Parent));
    GenericTemplateTypes.Add(GenType);
    ReadPasElement(TemplObj,GenType,aContext);
    ReadElementArray(TemplObj,GenType,'Constraints',GenType.Constraints,
      {$IFDEF CheckPasTreeRefCount}'TPasGenericTemplateType.Constraints'{$ELSE}true{$ENDIF},
      aContext);
    end;
end;

procedure TPCUReader.ReadAliasType(Obj: TJSONObject; El: TPasAliasType;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElType(Obj,'Dest',El,@Set_AliasType_DestType,aContext);
  El.Expr:=ReadExpr(Obj,El,'Expr',aContext);
end;

procedure TPCUReader.ReadPointerType(Obj: TJSONObject; El: TPasPointerType;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElType(Obj,'Dest',El,@Set_PointerType_DestType,aContext);
end;

procedure TPCUReader.ReadSpecializeType(Obj: TJSONObject;
  El: TPasSpecializeType; aContext: TPCUReaderContext);
var
  GenType: TPasGenericType;
  GenericTemplateTypes: TFPList;
  SpecName: string;
  i, SpecId: Integer;
  Data: TPasSpecializeTypeData;
  PendSpec: TPCUReaderPendingSpecialized;
begin
  ReadAliasType(Obj,El,aContext);
  if not (El.DestType is TPasGenericType) then
    RaiseMsg(20200219121250,El,GetObjName(El.DestType));
  GenType:=TPasGenericType(El.DestType);
  GenericTemplateTypes:=GenType.GenericTemplateTypes;
  if (GenericTemplateTypes=nil) or (GenericTemplateTypes.Count=0) then
    RaiseMsg(20200219121415,El,GetObjPath(El.DestType));

  ReadElementList(Obj,El,'Params',El.Params,
    {$IFDEF CheckPasTreeRefCount}'TPasSpecializeType.Params'{$ELSE}true{$ENDIF},
    aContext);
  if El.Params.Count=0 then
    RaiseMsg(20200219121447,El);
  if El.Params.Count<>GenType.GenericTemplateTypes.Count then
    RaiseMsg(20200219121521,El,GetObjPath(GenType));
  for i:=0 to El.Params.Count-1 do
    if El.Params[i]=nil then
      RaiseMsg(20200512232836,El,GetObjPath(El.DestType)+' Params['+IntToStr(i)+']=nil');

  if not ReadInteger(Obj,'SpecType',SpecId,El) then
    begin
    if Obj.Find('SpecType')<>nil then
      RaiseMsg(20201203092759,El,GetObjName(Obj.Find('SpecType')));
    exit; // generic reference to a generic
    end;

  // El.Data TPasSpecializeTypeData
  Data:=TPasSpecializeTypeData.Create;
  // add to free list
  Resolver.AddResolveData(El,Data,lkModule);

  PromiseSetElReference(SpecId,@Set_SpecializeTypeData,Data,El);

  // check old specialized name
  if not ReadString(Obj,'SpecTypeName',SpecName,El) then
    RaiseMsg(20200219122919,El);
  if SpecName='' then
    RaiseMsg(20200530134152,El);

  if Data.SpecializedType=nil then
    begin
    PendSpec:=PromiseSpecialize(SpecId,SpecName,El,El);
    // specialize now
    CreateSpecializedElement(PendSpec);
    end;
end;

procedure TPCUReader.ReadInlineSpecializeExpr(Obj: TJSONObject;
  Expr: TInlineSpecializeExpr; aContext: TPCUReaderContext);
var
  Parent: TPasElement;
begin
  ReadPasElement(Obj,Expr,aContext);
  Expr.Kind:=pekSpecialize;
  Expr.NameExpr:=ReadExpr(Obj,Expr,'ISEName',aContext);
  ReadElementList(Obj,Expr,'ISEParams',Expr.Params,
    {$IFDEF CheckPasTreeRefCount}'TInlineSpecializeExpr.Params'{$ELSE}true{$ENDIF},
    aContext);
  Parent:=Expr.Parent;
  while Parent<>nil do
    begin
    if Parent is TProcedureBody then exit; // inside generic method -> ok
    Parent:=Parent.Parent;
    end;
  // ToDo: create specialized type
  RaiseMsg(20200512233430,Expr);
end;

procedure TPCUReader.ReadRangeType(Obj: TJSONObject; El: TPasRangeType;
  aContext: TPCUReaderContext);
var
  Expr: TPasExpr;
  s: String;
begin
  ReadPasElement(Obj,El,aContext);
  Expr:=ReadExpr(Obj,El,'Range',aContext);
  if not (Expr is TBinaryExpr) then
    begin
    s:=GetObjName(Expr);
    if Expr<>nil then
      Expr.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
    RaiseMsg(20180216204042,El,s);
    end;
  El.RangeExpr:=TBinaryExpr(Expr);
end;

procedure TPCUReader.ReadArrayScope(Obj: TJSONObject; Scope: TPas2JSArrayScope;
  aContext: TPCUReaderContext);
begin
  ReadIdentifierScope(Obj,Scope,aContext);
  Scope.GenericStep:=psgsImplementationParsed;
end;

procedure TPCUReader.ReadArrayType(Obj: TJSONObject; El: TPasArrayType;
  aContext: TPCUReaderContext);
var
  Scope: TPas2JSArrayScope;
begin
  ReadPasElement(Obj,El,aContext);
  ReadGenericTemplateTypes(Obj,El,El.GenericTemplateTypes,aContext);
  ReadPasExprArray(Obj,El,'Ranges',El.Ranges,aContext);
  if El.PackMode<>pmNone then
    Obj.Add('Packed',PCUPackModeNames[El.PackMode]);
  if (El.GenericTemplateTypes<>nil) and (El.GenericTemplateTypes.Count>0) then
    begin
    Scope:=TPas2JSArrayScope(Resolver.CreateScope(El,TPas2JSArrayScope));
    El.CustomData:=Scope;
    ReadArrayScope(Obj,Scope,aContext);
    end;
  ReadElType(Obj,'ElType',El,@Set_ArrayType_ElType,aContext);

  ReadSpecializations(Obj,El);
end;

procedure TPCUReader.ReadFileType(Obj: TJSONObject; El: TPasFileType;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElType(Obj,'ElType',El,@Set_FileType_ElType,aContext);
end;

procedure TPCUReader.ReadEnumValue(Obj: TJSONObject; El: TPasEnumValue;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  El.Value:=ReadExpr(Obj,El,'Value',aContext);
end;

procedure TPCUReader.ReadEnumTypeScope(Obj: TJSONObject;
  Scope: TPasEnumTypeScope; aContext: TPCUReaderContext);
begin
  ReadElType(Obj,'CanonicalSet',Scope.Element,@Set_EnumTypeScope_CanonicalSet,aContext);
  ReadIdentifierScope(Obj,Scope,aContext);
end;

procedure TPCUReader.ReadEnumType(Obj: TJSONObject; El: TPasEnumType;
  aContext: TPCUReaderContext);
var
  Scope: TPasEnumTypeScope;
begin
  Scope:=TPasEnumTypeScope(Resolver.CreateScope(El,TPasEnumTypeScope));
  El.CustomData:=Scope;

  ReadPasElement(Obj,El,aContext);
  ReadEnumTypeScope(Obj,Scope,aContext);
  Resolver.PushScope(Scope);
  try
    ReadElementList(Obj,El,'Values',El.Values,
      {$IFDEF CheckPasTreeRefCount}'TPasEnumType.Values'{$ELSE}true{$ENDIF},
      aContext);
  finally
    Resolver.PopScope;
  end;
end;

procedure TPCUReader.ReadSetType(Obj: TJSONObject; El: TPasSetType;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElType(Obj,'EnumType',El,@Set_SetType_EnumType,aContext);
  ReadBoolean(Obj,'Packed',El.IsPacked,El);
end;

function TPCUReader.ReadPackedMode(Obj: TJSONObject; const PropName: string;
  ErrorEl: TPasElement): TPackMode;
var
  p: TPackMode;
  s: string;
begin
  Result:=pmNone;
  if not ReadString(Obj,PropName,s,ErrorEl) then exit;
  for p in TPackMode do
    if s=PCUPackModeNames[p] then
      exit(p);
  RaiseMsg(20180210210038,ErrorEl,PropName+' "'+s+'"');
end;

procedure TPCUReader.ReadRecordVariant(Obj: TJSONObject; El: TPasVariant;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElementList(Obj,El,'Values',El.Values,
    {$IFDEF CheckPasTreeRefCount}'TPasVariant.Values'{$ELSE}true{$ENDIF},
    aContext);
  ReadElType(Obj,'Members',El,@Set_Variant_Members,aContext);
end;

procedure TPCUReader.ReadRecordScope(Obj: TJSONObject; Scope: TPas2jsRecordScope;
  aContext: TPCUReaderContext);
begin
  ReadElementReference(Obj,Scope,'DefaultProperty',@Set_RecordScope_DefaultProperty);
  ReadIdentifierScope(Obj,Scope,aContext);
end;

procedure TPCUReader.ReadRecordType(Obj: TJSONObject; El: TPasRecordType;
  aContext: TPCUReaderContext);
var
  Data: TJSONData;
  Id: Integer;
  Scope: TPas2jsRecordScope;
  SubObj: TJSONObject;
begin
  if FileVersion<3 then
    RaiseMsg(20190109214718,El,'record format changed');

  Scope:=TPas2jsRecordScope(Resolver.CreateScope(El,TPas2jsRecordScope));
  El.CustomData:=Scope;

  ReadPasElement(Obj,El,aContext);
  ReadGenericTemplateTypes(Obj,El,El.GenericTemplateTypes,aContext);
  El.PackMode:=ReadPackedMode(Obj,'Packed',El);

  Resolver.PushScope(Scope);
  try
    ReadElementList(Obj,El,'Members',El.Members,
      {$IFDEF CheckPasTreeRefCount}'TPasRecordType.Members'{$ELSE}true{$ENDIF},
      aContext);

    // VariantEl: TPasElement can be TPasVariable or TPasType
    Data:=Obj.Find('VariantEl');
    if Data is TJSONIntegerNumber then
      begin
      Id:=Data.AsInteger;
      PromiseSetElReference(Id,@Set_RecordType_VariantEl,El,El);
      end
    else if Data is TJSONObject then
      begin
      SubObj:=TJSONObject(Data);
      El.VariantEl:=ReadNewElement(SubObj,El);
      ReadElement(SubObj,El.VariantEl,aContext);
      end;

    ReadElementList(Obj,El,'Variants',El.Variants,
      {$IFDEF CheckPasTreeRefCount}'TPasRecordType.Variants'{$ELSE}true{$ENDIF},
      aContext);
  finally
    Resolver.PopScope;
  end;
  ReadRecordScope(Obj,Scope,aContext);
  Resolver.FinishGenericClassOrRecIntf(Scope);
  Resolver.FinishSpecializations(Scope);

  ReadSpecializations(Obj,El);
end;

function TPCUReader.ReadClassInterfaceType(Obj: TJSONObject;
  const PropName: string; ErrorEl: TPasElement;
  DefaultValue: TPasClassInterfaceType): TPasClassInterfaceType;
var
  s: string;
  cit: TPasClassInterfaceType;
begin
  if ReadString(Obj,PropName,s,ErrorEl) then
    begin
    for cit in TPasClassInterfaceType do
      if s=PCUClassInterfaceTypeNames[cit] then
        exit(cit);
    RaiseMsg(20180329105126,ErrorEl,PropName+'='+s);
    end
  else
    Result:=DefaultValue;
end;

function TPCUReader.ReadClassScopeFlags(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TPasClassScopeFlags
  ): TPasClassScopeFlags;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TPasClassScopeFlag;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadClassScopeFlags START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TPasClassScopeFlag do
      if s=PCUClassScopeFlagNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180214115647,'unknown class scope flag "'+s+'"');
    end;
end;

procedure TPCUReader.ReadClassScopeAbstractProcs(Obj: TJSONObject;
  Scope: TPas2JSClassScope);
var
  Arr: TJSONArray;
  Data: TJSONData;
  Id, i: Integer;
  Ref: TPCUFilerElementRef;
begin
  if not ReadArray(Obj,'AbstractProcs',Arr,Scope.Element) then exit;
  SetLength(Scope.AbstractProcs,Arr.Count);
  for i:=0 to Arr.Count-1 do
    begin
    Data:=Arr[i];
    if Data is TJSONIntegerNumber then
      begin
      Id:=Data.AsInteger;
      Ref:=GetElReference(Id,Scope.Element);
      if (Ref=nil) or (Ref.Element=nil) then
        RaiseMsg(20180214121727,Scope.Element,'['+IntToStr(i)+'] missing Id '+IntToStr(Id));
      if Ref.Element is TPasProcedure then
        Scope.AbstractProcs[i]:=TPasProcedure(Ref.Element) // no AddRef
      else
        RaiseMsg(20180214121902,Scope.Element,'['+IntToStr(i)+'] is '+GetObjName(Ref.Element));
      end
    else
      RaiseMsg(20180214121627,Scope.Element,'['+IntToStr(i)+'] is '+GetObjName(Data));
    end;
end;

procedure TPCUReader.ReadClassIntfMapProcs(Obj: TJSONObject;
  Map: TPasClassIntfMap; OrigIntfType: TPasType);
var
  aClass: TPasClassType;
  Arr: TJSONArray;
  i, Id: Integer;
  Data: TJSONData;
  IntfMember: TPasElement;
  Ref: TPCUFilerElementRef;
begin
  aClass:=Map.Element as TPasClassType;
  if ReadArray(Obj,'Procs',Arr,aClass) then
    begin
    if Map.Procs<>nil then
      RaiseMsg(20180329143122,aClass);
    Map.Procs:=TFPList.Create;
    if Arr.Count<>Map.Intf.Members.Count then
      RaiseMsg(20180325130318,aClass,Map.Intf.FullPath+' Expected='+IntToStr(Map.Intf.Members.Count)+', but found '+IntToStr(Arr.Count));
    for i:=0 to Arr.Count-1 do
      begin
      Data:=Arr[i];
      IntfMember:=TPasElement(Map.Intf.Members[i]);
      if (Data is TJSONIntegerNumber) then
        begin
        Id:=Data.AsInteger;
        Ref:=AddElReference(Id,aClass,nil);
        if Ref.Element=nil then
          RaiseMsg(20180325125930,aClass,'missing method resolution of interface '+OrigIntfType.Name);
        if not (Ref.Element is TPasProcedure) then
          RaiseMsg(20180325130108,aClass,'['+IntToStr(i)+']='+OrigIntfType.Name+'.'+GetObjName(IntfMember)+' method expected, but found '+GetObjName(Ref.Element));
        if not (IntfMember is TPasProcedure) then
          RaiseMsg(20180329134354,aClass,'['+IntToStr(i)+']='+OrigIntfType.Name+'.'+GetObjName(IntfMember)+' intf member is not method, mapped proc='+GetObjName(Ref.Element));
        Map.Procs.Add(Ref.Element);
        end
      else if Data is TJSONNull then
        begin
        if IntfMember is TPasProcedure then
          RaiseMsg(20180329132957,aClass,'['+IntToStr(i)+']='+OrigIntfType.Name+'.'+GetObjName(IntfMember)+' intf method expects implementation');
        Map.Procs.Add(nil);
        end
      else
        RaiseMsg(20180325125851,aClass,IntToStr(i)+' '+GetObjName(Data));
      end;
    end
  else if Map.Intf.Members.Count>0 then
    RaiseMsg(20180325130720,aClass,Map.Intf.FullPath+' Expected='+IntToStr(Map.Intf.Members.Count)+', but found 0');
end;

procedure TPCUReader.ReadClassIntfMap(Obj: TJSONObject; Scope: TPas2JSClassScope;
  Map: TPasClassIntfMap; OrigIntfType: TPasType);
var
  aClass: TPasClassType;
  Id: Integer;
  Data: TJSONData;
  Ref: TPCUFilerElementRef;
  AncObj: TJSONObject;
begin
  aClass:=Scope.Element as TPasClassType;
  Map.Element:=aClass;

  // Intf
  Data:=Obj.Find('Intf');
  if not (Data is TJSONIntegerNumber) then
    RaiseMsg(20180325130226,aClass,OrigIntfType.Name);
  Id:=Data.AsInteger;
  Ref:=AddElReference(Id,aClass,nil);
  if not (Ref.Element is TPasClassType) then
    RaiseMsg(20180325131020,aClass,OrigIntfType.Name+' '+GetObjName(Ref.Element));
  Map.Intf:=TPasClassType(Ref.Element);

  // Procs
  ReadClassIntfMapProcs(Obj,Map,OrigIntfType);

  // AncestorMap
  if ReadObject(Obj,'AncestorMap',AncObj,aClass) then
    begin
    Map.AncestorMap:=TPasClassIntfMap.Create;
    ReadClassIntfMap(AncObj,Scope,Map.AncestorMap,OrigIntfType);
    end;
end;

procedure TPCUReader.ReadClassScopeInterfaces(Obj: TJSONObject;
  Scope: TPas2JSClassScope);
var
  aClass: TPasClassType;
  Arr: TJSONArray;
  i, Id: Integer;
  Data: TJSONData;
  Ref: TPCUFilerElementRef;
  OrigIntfType, IntfType: TPasType;
  SubObj: TJSONObject;
  Map: TPasClassIntfMap;
begin
  aClass:=Scope.Element as TPasClassType;
  if ReadArray(Obj,'SInterfaces',Arr,aClass) then
    begin
    if Arr.Count<>aClass.Interfaces.Count then
      RaiseMsg(20180325124134,aClass);
    if Scope.Interfaces=nil then
      Scope.Interfaces:=TFPList.Create;
    if Scope.Interfaces.Count>0 then
      RaiseMsg(20180325124546,aClass);
    for i:=0 to Arr.Count-1 do
      begin
      OrigIntfType:=TPasType(aClass.Interfaces[i]);
      IntfType:=Resolver.ResolveAliasType(OrigIntfType);
      if not (IntfType is TPasClassType) then
        RaiseMsg(20180325124401,aClass,IntToStr(i)+' '+GetObjName(IntfType));
      Data:=Arr[i];
      if Data is TJSONIntegerNumber then
        begin
        // property, interface delegation
        Id:=Data.AsInteger;
        Ref:=AddElReference(Id,aClass,nil);
        if Ref.Element=nil then
          RaiseMsg(20180325124421,aClass,'missing delegation property of interface '+OrigIntfType.Name);
        if not (Ref.Element is TPasProperty) then
          RaiseMsg(20180325124616,aClass,OrigIntfType.Name+' delegate: '+GetObjName(Ref.Element));
        Scope.Interfaces.Add(Ref.Element);
        end
      else if Data is TJSONObject then
        begin
        // map
        SubObj:=TJSONObject(Data);
        Map:=TPasClassIntfMap.Create;
        Scope.Interfaces.Add(Map);
        ReadClassIntfMap(SubObj,Scope,Map,OrigIntfType);
        end
      else
        RaiseMsg(20180325124206,aClass,OrigIntfType.Name);
      end;
    end
  else if aClass.Interfaces.Count>0 then
    begin
    RaiseMsg(20180325131248,aClass);
    end;
end;

procedure TPCUReader.ReadClassScopeDispatchProcs(Obj: TJSONObject;
  Scope: TPas2JSClassScope);
var
  El: TPasClassType;
begin
  El:=TPasClassType(Scope.Element);
  if not ReadString(Obj,'DispatchField',Scope.DispatchField,El) then
    Scope.DispatchField:=PCUDispatchDefaultField;
  if not ReadString(Obj,'DispatchStrField',Scope.DispatchStrField,El) then
    Scope.DispatchStrField:=PCUDispatchDefaultStrField;
end;

procedure TPCUReader.ReadClassScope(Obj: TJSONObject; Scope: TPas2JSClassScope;
  aContext: TPCUReaderContext);
var
  aClass: TPasClassType;
  CanonicalClassOf: TPasClassOfType;
  CanonicalClassOfId: integer;
begin
  aClass:=Scope.Element as TPasClassType;

  if aClass.ObjKind in ([okClass]+okAllHelpers) then
    begin
    CanonicalClassOf:=TPasClassOfType(CreateElement(TPasClassOfType,'Self',aClass));
    Scope.CanonicalClassOf:=CanonicalClassOf;
    CanonicalClassOf.Visibility:=visStrictPrivate;
    CanonicalClassOf.SourceFilename:=aClass.SourceFilename;
    CanonicalClassOf.SourceLinenumber:=aClass.SourceLinenumber;
    CanonicalClassOf.DestType:=aClass;
    aClass.AddRef{$IFDEF CheckPasTreeRefCount}('TPasClassScope.CanonicalClassOf'){$ENDIF};
    if ReadInteger(Obj,'ClassOf',CanonicalClassOfId,CanonicalClassOf) then
      AddElReference(CanonicalClassOfId,CanonicalClassOf,CanonicalClassOf);
    end;

  ReadElementReference(Obj,Scope,'NewInstanceFunction',@Set_ClassScope_NewInstanceFunction);
  ReadElementReference(Obj,Scope,'DirectAncestor',@Set_ClassScope_DirectAncestor);
  ReadElementReference(Obj,Scope,'DefaultProperty',@Set_ClassScope_DefaultProperty);
  Scope.Flags:=ReadClassScopeFlags(Obj,Scope.Element,'SFlags',GetDefaultClassScopeFlags(Scope));
  if not ReadString(Obj,'SGUID',Scope.GUID,aClass) then
    Scope.GUID:='';

  ReadIdentifierScope(Obj,Scope,aContext);
end;

procedure TPCUReader.ReadClassType(Obj: TJSONObject; El: TPasClassType;
  aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  i: Integer;
  Data: TJSONData;
  Scope: TPas2JSClassScope;
  Ref: TResolvedReference;
  Parent: TPasElement;
  SectionScope: TPasSectionScope;
begin
  ReadBoolean(Obj,'Forward',El.IsForward,El);

  if El.IsForward then
    begin
    Scope:=nil;
    Ref:=TResolvedReference.Create;
    Resolver.AddResolveData(El,Ref,lkModule);
    ReadResolvedReference(Obj,Ref,El);
    end
  else
    begin
    if Obj.Find('Scope') is TJSONBoolean then
      Scope:=nil // msIgnoreInterfaces
    else
      begin
      Scope:=TPas2JSClassScope(Resolver.CreateScope(El,Resolver.ScopeClass_Class));
      El.CustomData:=Scope;
      end;
    end;

  ReadPasElement(Obj,El,aContext);
  ReadGenericTemplateTypes(Obj,El,El.GenericTemplateTypes,aContext);
  El.PackMode:=ReadPackedMode(Obj,'Packed',El);
  // ObjKind is the 'Type'

  if El.IsForward then
    exit;

  El.InterfaceType:=ReadClassInterfaceType(Obj,'IntfType',El,citCom);

  ReadElType(Obj,'Ancestor',El,@Set_ClassType_AncestorType,aContext);
  ReadElType(Obj,'HelperFor',El,@Set_ClassType_HelperForType,aContext);
  ReadBoolean(Obj,'External',El.IsExternal,El);
  // not needed IsShortDefinition: Boolean; -> class(anchestor); without end
  El.GUIDExpr:=ReadExpr(Obj,El,'GUID',aContext);

  // Modifiers
  if ReadArray(Obj,'Modifiers',Arr,El) then
    begin
    for i:=0 to Arr.Count-1 do
      begin
      Data:=Arr[i];
      if not (Data is TJSONString) then
        RaiseMsg(20180210211250,El,'Modifiers['+IntToStr(i)+'] '+GetObjName(Data));
      El.Modifiers.Add(String(Data.AsString));
      end;
    end;

  ReadElementList(Obj,El,'Interfaces',El.Interfaces,
    {$IFDEF CheckPasTreeRefCount}'TPasClassType.Interfaces'{$ELSE}true{$ENDIF},
    aContext);
  ReadString(Obj,'ExternalNameSpace',El.ExternalNameSpace,El);
  ReadString(Obj,'ExternalName',El.ExternalName,El);

  if Scope<>nil then
    begin
    Resolver.PushScope(Scope);
    try
      ReadClassScope(Obj,Scope,aContext);

      // read Members
      ReadElementList(Obj,El,'Members',El.Members,
        {$IFDEF CheckPasTreeRefCount}'TPasClassType.Members'{$ELSE}true{$ENDIF},
        aContext);

      ReadClassScopeAbstractProcs(Obj,Scope);
      ReadClassScopeInterfaces(Obj,Scope);
      ReadClassScopeDispatchProcs(Obj,Scope);

      if El.ObjKind in okAllHelpers then
        begin
        // restore cached helpers in interface
        Parent:=El.Parent;
        while Parent<>nil do
          begin
          if Parent.ClassType=TInterfaceSection then
            begin
            SectionScope:=Parent.CustomData as TPasSectionScope;
            Resolver.AddHelper(El,SectionScope.Helpers);
            break;
            end;
          Parent:=Parent.Parent;
          end;
        end;
    finally
      Resolver.PopScope;
    end;
    Resolver.FinishGenericClassOrRecIntf(Scope);
    if (El.GenericTemplateTypes<>nil) and (El.GenericTemplateTypes.Count>0) then
      FPendingForwardProcs.Add(El);
    ReadSpecializations(Obj,El);
    end;
end;

procedure TPCUReader.ReadArgument(Obj: TJSONObject; El: TPasArgument;
  aContext: TPCUReaderContext);
var
  s: string;
  Found: Boolean;
  Arg: TArgumentAccess;
begin
  ReadPasElement(Obj,El,aContext);
  if ReadString(Obj,'Access',s,El) then
    begin
    Found:=false;
    for Arg in TArgumentAccess do
      if s=PCUArgumentAccessNames[Arg] then
        begin
        El.Access:=Arg;
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180210205544,El,'Access "'+s+'"');
    end;
  ReadElType(Obj,'ArgType',El,@Set_Argument_ArgType,aContext);
  El.ValueExpr:=ReadExpr(Obj,El,'Value',aContext);
end;

function TPCUReader.ReadProcTypeModifiers(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TProcTypeModifiers
  ): TProcTypeModifiers;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TProcTypeModifier;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadProcTypeModifiers START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TProcTypeModifier do
      if s=PCUProcTypeModifierNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180210212015,'unknown procedure modifier "'+s+'"');
    end;
end;

procedure TPCUReader.ReadProcTypeScope(Obj: TJSONObject;
  Scope: TPas2JSProcTypeScope; aContext: TPCUReaderContext);
begin
  ReadIdentifierScope(Obj,Scope,aContext);
  Scope.GenericStep:=psgsImplementationParsed;
end;

procedure TPCUReader.ReadProcedureType(Obj: TJSONObject; El: TPasProcedureType;
  aContext: TPCUReaderContext);
var
  s: string;
  Found: Boolean;
  c: TCallingConvention;
  Scope: TPas2JSProcTypeScope;
begin
  ReadPasElement(Obj,El,aContext);
  ReadGenericTemplateTypes(Obj,El,El.GenericTemplateTypes,aContext);

  if (El.GenericTemplateTypes<>nil) and (El.GenericTemplateTypes.Count>0) then
    begin
    Scope:=TPas2JSProcTypeScope(Resolver.CreateScope(El,TPas2JSProcTypeScope));
    El.CustomData:=Scope;
    ReadProcTypeScope(Obj,Scope,aContext);
    end;

  ReadElementList(Obj,El,'Args',El.Args,
    {$IFDEF CheckPasTreeRefCount}'TPasProcedureType.Args'{$ELSE}true{$ENDIF},
    aContext);

  if ReadString(Obj,'Call',s,El) then
    begin
    Found:=false;
    for c in TCallingConvention do
      if s=PCUCallingConventionNames[c] then
        begin
        El.CallingConvention:=c;
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180210212130,El,'Call "'+s+'"');
    end;
  El.Modifiers:=ReadProcTypeModifiers(Obj,El,'Modifiers',GetDefaultProcTypeModifiers(El));

  ReadSpecializations(Obj,El);
end;

procedure TPCUReader.ReadResultElement(Obj: TJSONObject; El: TPasResultElement;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElType(Obj,'Result',El,@Set_ResultElement_ResultType,aContext);
end;

procedure TPCUReader.ReadFunctionType(Obj: TJSONObject; El: TPasFunctionType;
  aContext: TPCUReaderContext);
begin
  ReadProcedureType(Obj,El,aContext);
  El.ResultEl:=TPasResultElement(ReadElementProperty(Obj,El,'Result',TPasResultElement,aContext));
end;

procedure TPCUReader.ReadStringType(Obj: TJSONObject; El: TPasStringType;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadString(Obj,'Length',El.LengthExpr,El);
end;

function TPCUReader.ReadVarModifiers(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TVariableModifiers
  ): TVariableModifiers;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TVariableModifier;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadVarModifiers START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TVariableModifier do
      if s=PCUVarModifierNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180207184723,'unknown var modifier "'+s+'"');
    end;
end;

procedure TPCUReader.ReadVariable(Obj: TJSONObject; El: TPasVariable;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);

  ReadElType(Obj,'VarType',El,@Set_Variable_VarType,aContext);
  El.VarModifiers:=ReadVarModifiers(Obj,El,'VarMods',[]);
  El.LibraryName:=ReadExpr(Obj,El,'Library',aContext);
  El.ExportName:=ReadExpr(Obj,El,'Export',aContext);
  El.AbsoluteExpr:=ReadExpr(Obj,El,'Absolute',aContext);
  El.Expr:=ReadExpr(Obj,El,'Expr',aContext);
end;

procedure TPCUReader.ReadExportSymbol(Obj: TJSONObject; El: TPasExportSymbol;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  El.NameExpr:=ReadExpr(Obj,El,'NameExpr',aContext);
  El.ExportName:=ReadExpr(Obj,El,'ExportName',aContext);
  El.ExportIndex:=ReadExpr(Obj,El,'ExportIndex',aContext);
end;

procedure TPCUReader.ReadConst(Obj: TJSONObject; El: TPasConst;
  aContext: TPCUReaderContext);
begin
  ReadVariable(Obj,El,aContext);
  if not ReadBoolean(Obj,'IsConst',El.IsConst,El) then
    El.IsConst:=Obj.Find('VarType')=nil;
end;

procedure TPCUReader.ReadPropertyScope(Obj: TJSONObject;
  Scope: TPasPropertyScope; aContext: TPCUReaderContext);
begin
  ReadElementReference(Obj,Scope,'AncestorProp',@Set_PropertyScope_AncestorProp);
  ReadIdentifierScope(Obj,Scope,aContext);
end;

procedure TPCUReader.ReadProperty(Obj: TJSONObject; El: TPasProperty;
  aContext: TPCUReaderContext);
var
  Scope: TPasPropertyScope;
  Expr: TPasExpr;
begin
  if Obj.Find('Scope') is TJSONBoolean then
    Scope:=nil // msIgnoreInterfaces
  else
    begin
    Scope:=TPasPropertyScope(Resolver.CreateScope(El,TPasPropertyScope));
    El.CustomData:=Scope;
    end;

  ReadVariable(Obj,El,aContext);
  El.IndexExpr:=ReadExpr(Obj,El,'Index',aContext);
  El.ReadAccessor:=ReadExpr(Obj,El,'Read',aContext);
  El.WriteAccessor:=ReadExpr(Obj,El,'Write',aContext);
  if FileVersion<2 then
    begin
    if Obj.Find('Implements')<>nil then
      begin
      Expr:=ReadExpr(Obj,El,'Implements',aContext);
      SetLength(El.Implements,1);
      El.Implements[0]:=Expr;
      end;
    end
  else
    ReadPasExprArray(Obj,El,'Implements',El.Implements,aContext);
  El.DispIDExpr:=ReadExpr(Obj,El,'DispId',aContext);
  El.StoredAccessor:=ReadExpr(Obj,El,'Stored',aContext);
  El.DefaultExpr:=ReadExpr(Obj,El,'DefaultValue',aContext);

  if Scope<>nil then
    Resolver.PushScope(Scope);
  try
    ReadElementList(Obj,El,'Args',El.Args,
      {$IFDEF CheckPasTreeRefCount}'TPasProperty.Args'{$ELSE}true{$ENDIF},
      aContext);
  finally
    if Scope<>nil then
      Resolver.PopScope;
  end;
  //ReadAccessorName: string; // not used by resolver
  //WriteAccessorName: string; // not used by resolver
  //ImplementsName: string; // not used by resolver
  //StoredAccessorName: string; // not used by resolver
  ReadBoolean(Obj,'ReadOnly',El.DispIDReadOnly,El);
  ReadBoolean(Obj,'Default',El.IsDefault,El);
  ReadBoolean(Obj,'NoDefault',El.IsNodefault,El);

  if Scope<>nil then
    ReadPropertyScope(Obj,Scope,aContext);
end;

procedure TPCUReader.ReadMethodResolution(Obj: TJSONObject;
  El: TPasMethodResolution; aContext: TPCUReaderContext);
var
  s: string;
begin
  ReadPasElement(Obj,El,aContext);
  if ReadString(Obj,'ProcClass',s,El) then
    case s of
    'procedure': El.ProcClass:=TPasProcedure;
    else
      RaiseMsg(20180329104616,El,s);
    end
  else
    El.ProcClass:=TPasFunction;
  El.InterfaceProc:=ReadExpr(Obj,El,'InterfaceProc',aContext);
  El.InterfaceName:=ReadExpr(Obj,El,'InterfaceName',aContext);
  El.ImplementationProc:=ReadExpr(Obj,El,'ImplementationProc',aContext);
end;

procedure TPCUReader.ReadGenericTemplateType(Obj: TJSONObject;
  El: TPasGenericTemplateType; aContext: TPCUReaderContext);
var
  Scope: TPasGenericParamsScope;
begin
  ReadPasElement(Obj,El,aContext);
  Scope:=TPasGenericParamsScope(Resolver.CreateScope(El,TPasGenericParamsScope));
  El.CustomData:=Scope;
  // Scope.GenericType only needed during parsing
  ReadElementArray(Obj,El,'Constraints',El.Constraints,
     {$IFDEF CheckPasTreeRefCount}'TPasGenericTemplateType.Constraints'{$ELSE}true{$ENDIF},
     aContext);
end;

procedure TPCUReader.ReadProcedureNameParts(Obj: TJSONObject;
  El: TPasProcedure; aContext: TPCUReaderContext);
var
  Arr, TemplArr: TJSONArray;
  i, j: Integer;
  NamePartObj, TemplObj: TJSONObject;
  GenTypeName: string;
  GenType: TPasGenericTemplateType;
  NamePart: TProcedureNamePart;
begin
  ReleaseProcNameParts(El.NameParts);
  if ReadArray(Obj,'NameParts',Arr,El) then
    begin
    if El.NameParts=nil then
      El.NameParts:=TProcedureNameParts.Create
    else
      El.NameParts.Clear;
    for i:=0 to Arr.Count-1 do
      begin
      NamePartObj:=CheckJSONObject(Arr[i],20190718113441);
      NamePart:=TProcedureNamePart.Create;
      El.NameParts.Add(NamePart);
      with NamePart do
        begin
        if not ReadString(NamePartObj,'Name',Name,El) then
          RaiseMsg(20190718113739,El,IntToStr(i));
        if ReadArray(NamePartObj,'Templates',TemplArr,El) then
          begin
          Templates:=TFPList.Create;
          for j:=0 to TemplArr.Count-1 do
            begin
            TemplObj:=CheckJSONObject(TemplArr[j],20190718114058);
            if not ReadString(TemplObj,'Name',GenTypeName,El) or (GenTypeName='') then
              RaiseMsg(20190718114244,El,IntToStr(i)+','+IntToStr(j));
            GenType:=TPasGenericTemplateType(CreateElement(TPasGenericTemplateType,GenTypeName,El));
            Templates.Add(GenType);
            ReadGenericTemplateType(TemplObj,GenType,aContext);
            end;
          end;
        end;
      end;
    end;
  if aContext=nil then ;
end;

function TPCUReader.ReadProcedureModifiers(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TProcedureModifiers
  ): TProcedureModifiers;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TProcedureModifier;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadProcedureModifiers START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TProcedureModifier do
      if s=PCUProcedureModifierNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180211110407,'unknown proc modifier "'+s+'"');
    end;
end;

function TPCUReader.ReadProcScopeFlags(Obj: TJSONObject; El: TPasElement;
  const PropName: string; const DefaultValue: TPasProcedureScopeFlags
  ): TPasProcedureScopeFlags;
var
  Names: TStringDynArray;
  Enable: TBooleanDynArray;
  s: String;
  f: TPasProcedureScopeFlag;
  i: Integer;
  Found: Boolean;
  Data: TJSONData;
begin
  Result:=DefaultValue;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadProcedureScopeFlags START');
  {$ENDIF}
  Data:=Obj.Find(PropName);
  if Data=nil then exit;
  ReadArrayFlags(Data,El,PropName,Names,Enable);
  for i:=0 to length(Names)-1 do
    begin
    s:=Names[i];
    Found:=false;
    for f in TPasProcedureScopeFlag do
      if s=PCUProcedureScopeFlagNames[f] then
        begin
        if Enable[i] then
          Include(Result,f)
        else
          Exclude(Result,f);
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180213220601,'unknown proc scope flag "'+s+'"');
    end;
end;

procedure TPCUReader.ReadProcedureScope(Obj: TJSONObject;
  Scope: TPas2JSProcedureScope; aContext: TPCUReaderContext);
var
  Proc: TPasProcedure;
begin
  Proc:=Scope.Element as TPasProcedure;
  ReadString(Obj,'ResultVarName',Scope.ResultVarName,Proc);
  // Scope.OverloadName is already set in ReadProcedure
  ReadElementReference(Obj,Scope,'ImplProc',@Set_ProcedureScope_ImplProc);
  ReadElementReference(Obj,Scope,'Overridden',@Set_ProcedureScope_Overridden);
  if Proc.Parent is TPasMembersType then
    Scope.ClassRecScope:=Proc.Parent.CustomData as TPasClassOrRecordScope // no AddRef
  else
    ; // set via Set_ProcedureScope_ImplProc

  Scope.Flags:=ReadProcScopeFlags(Obj,Proc,'SFlags',[]);
  Scope.BoolSwitches:=ReadBoolSwitches(Obj,Proc,'BoolSwitches',aContext.BoolSwitches);
  Scope.ModeSwitches:=ReadModeSwitches(Obj,Proc,'ModeSwitches',aContext.ModeSwitches);

  // Scope.SelfArg
  if (Scope.ClassRecScope<>nil) and (Scope.DeclarationProc=nil) then
    Resolver.CreateProcSelfArg(Proc);

  //ReadIdentifierScope(Obj,Scope,aContext);
end;

procedure TPCUReader.ReadProcScopeReferences(Obj: TJSONObject;
  ImplScope: TPas2JSProcedureScope);
var
  DeclScope: TPasProcedureScope;
  DeclProc: TPasProcedure;
begin
  // Note: the References are stored in the scope object of the declaration proc,
  //       But TPCUWriter stores them in the implementation scope, so that all
  //       references can be resolved immediately.
  if ImplScope.ImplProc<>nil then
    RaiseMsg(20180318212631,ImplScope.Element);
  DeclProc:=ImplScope.DeclarationProc;
  if DeclProc=nil then
    DeclProc:=ImplScope.Element as TPasProcedure;
  DeclScope:=DeclProc.CustomData as TPasProcedureScope;
  if DeclScope.References<>nil then
    RaiseMsg(20180221172403,DeclProc);
  ReadScopeReferences(Obj,DeclScope,'Refs',DeclScope.References);
end;

procedure TPCUReader.ReadProcedureBody(Obj: TJSONObject; El: TPasProcedure;
  aContext: TPCUReaderContext);
var
  ImplScope: TPas2JSProcedureScope;
  s: string;
  DeclProc: TPasProcedure;
  BodyObj, BodyBodyObj: TJSONObject;
  ProcBody: TProcedureBody;
  ImplEl: TPasElement;
  OldInGeneric: Boolean;
  ImplJS: TPas2JSPrecompiledJS;
begin
  ImplScope:=TPas2JSProcedureScope(El.CustomData);
  if ImplScope.ImplProc<>nil then
    RaiseMsg(20191231152850,El);
  if ImplScope.ImplJS<>nil then
    RaiseMsg(20201018121506,El);
  DeclProc:=ImplScope.DeclarationProc;
  if DeclProc=nil then
    DeclProc:=El;

  Resolver.PushScope(ImplScope);
  try
    if Resolver.ProcCanBePrecompiled(DeclProc) then
      begin
      // normal proc (non generic)
      ImplJS:=TPas2JSPrecompiledJS.Create;
      ImplScope.ImplJS:=ImplJS;
      ReadPrecompiledJS(Obj,El,ImplJS,aContext);
      end
    else
      begin
      // generic proc
      if ReadObject(Obj,'Body',BodyObj,El) then
        begin
        OldInGeneric:=aContext.InGeneric;
        aContext.InGeneric:=true;
        ProcBody:=TProcedureBody(CreateElement(TProcedureBody,'',El));
        El.Body:=ProcBody;
        ProcBody.SourceFilename:=El.SourceFilename;
        ProcBody.SourceLinenumber:=El.SourceLinenumber;
        ProcBody.SourceEndLinenumber:=El.SourceEndLinenumber;
        ReadDeclarations(BodyObj,ProcBody,aContext);
        if ReadObject(BodyObj,'Impl',BodyBodyObj,ProcBody) then
          begin
          ImplEl:=ReadNewElement(BodyBodyObj,ProcBody);
          if not (ImplEl is TPasImplBlock) then
            begin
            s:=GetObjName(ImplEl);
            ImplEl.Release;
            RaiseMsg(20191231171840,ProcBody,s);
            end;
          ProcBody.Body:=TPasImplBlock(ImplEl);
          ReadElement(BodyBodyObj,ImplEl,aContext);
          end;
        aContext.InGeneric:=OldInGeneric;
        end;
      end;
  finally
    Resolver.PopScope;
  end;
end;

procedure TPCUReader.ReadProcedure(Obj: TJSONObject; El: TPasProcedure;
  aContext: TPCUReaderContext);
var
  DefProcMods: TProcedureModifiers;
  t: TProcedureMessageType;
  s: string;
  Found, HasBody: Boolean;
  Scope: TPas2JSProcedureScope;
  DeclProcId: integer;
  Ref: TPCUFilerElementRef;
  DeclProc: TPasProcedure;
  p: SizeInt;
begin
  if Obj.Find('Scope') is TJSONBoolean then
    Scope:=nil // msIgnoreInterfaces
  else
    begin
    Scope:=TPas2JSProcedureScope(Resolver.CreateScope(El,Resolver.ScopeClass_Procedure));
    El.CustomData:=Scope;
    p:=Pos('$',El.Name);
    if p>0 then
      begin
      // overload proc name$2 was stored in 'Name'
      Scope.OverloadName:=El.Name;
      El.Name:=LeftStr(El.Name,p-1);
      end;
    end;

  ReadPasElement(Obj,El,aContext);

  HasBody:=Obj.Find('Body')<>nil;
  if ReadInteger(Obj,'DeclarationProc',DeclProcId,El) then
    begin
    // ImplProc
    Ref:=GetElReference(DeclProcId,El);
    if (Ref=nil) or (Ref.Element=nil) then
      RaiseMsg(20180219140423,El,'missing DeclarationProc '+IntToStr(DeclProcId));
    if not (Ref.Element is TPasProcedure) then
      RaiseMsg(20180219140547,El,'DeclarationProc='+GetObjName(Ref.Element));
    DeclProc:=TPasProcedure(Ref.Element);
    Scope.DeclarationProc:=DeclProc; // no AddRef

    El.ProcType:=TPasProcedureType(CreateElement(TPasProcedureTypeClass(DeclProc.ProcType.ClassType),'',El));
    El.Modifiers:=ReadProcedureModifiers(Obj,El,'PMods',DeclProc.Modifiers*PCUProcedureModifiersImplProc);
    end
  else
    begin
    // declarationproc
    ReadProcedureNameParts(Obj,El,aContext);
    El.PublicName:=ReadExpr(Obj,El,'Public',aContext);
    // e.g. external LibraryExpr name LibrarySymbolName;
    El.LibraryExpr:=ReadExpr(Obj,El,'Lib',aContext);
    El.LibrarySymbolName:=ReadExpr(Obj,El,'LibName',aContext);
    El.DispIDExpr:=ReadExpr(Obj,El,'DispId',aContext);
    ReadString(Obj,'Alias',El.AliasName,El);
    El.MessageExpr:=ReadExpr(Obj,El,'Msg',aContext);
    if ReadString(Obj,'Message',s,El) then
      begin
      El.MessageName:=s;
      El.MessageType:=pmtInteger;
      if ReadString(Obj,'MessageType',s,El) then
        begin
        Found:=false;
        for t in TProcedureMessageType do
          if s=PCUProcedureMessageTypeNames[t] then
            begin
            El.MessageType:=t;
            Found:=true;
            break;
            end;
        if not Found then
          RaiseMsg(20180211104537,El,'MessageType "'+s+'"');
        end;
      end;
    DefProcMods:=GetDefaultProcModifiers(El);
    El.Modifiers:=ReadProcedureModifiers(Obj,El,'PMods',DefProcMods);

    // read ProcType after El.Modifiers
    El.ProcType:=TPasProcedureType(ReadElementProperty(
                                 Obj,El,'ProcType',TPasProcedureType,aContext));

    if Scope<>nil then
      ReadProcedureScope(Obj,Scope,aContext);
    end;

  if (Scope<>nil) and (Obj.Find('ImplProc')=nil) then
    ReadProcScopeReferences(Obj,Scope);

  if HasBody then
    ReadProcedureBody(Obj,El,aContext);
end;

procedure TPCUReader.ReadOperator(Obj: TJSONObject; El: TPasOperator;
  aContext: TPCUReaderContext);
var
  s: string;
  Found, b: Boolean;
  t: TOperatorType;
begin
  ReadProcedure(Obj,El,aContext);
  if ReadString(Obj,'Operator',s,El) then
    begin
    Found:=false;
    for t in TOperatorType do
      if s=PCUOperatorTypeNames[t] then
        begin
        El.OperatorType:=t;
        Found:=true;
        break;
        end;
    if not Found then
      RaiseMsg(20180211110647,El,'Operator "'+s+'"');
    end;
  if ReadBoolean(Obj,'TokenBased',b,El) then
    El.TokenBased:=b;
end;

procedure TPCUReader.ReadAttributes(Obj: TJSONObject; El: TPasAttributes;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadPasExprArray(Obj,El,'Calls',El.Calls,aContext);
end;

procedure TPCUReader.ReadPrecompiledJS(Obj: TJSONObject; El: TPasElement;
  ImplJS: TPas2JSPrecompiledJS; aContext: TPCUReaderContext);
var
  Arr: TJSONArray;
  i: Integer;
  Data: TJSONData;
begin
  ReadString(Obj,'Body',ImplJS.BodyJS,El);
  ReadBoolean(Obj,'Empty',ImplJS.EmptyJS,El);

  if ReadArray(Obj,'Globals',Arr,El) then
    begin
    for i:=0 to Arr.Count-1 do
      begin
      Data:=Arr[i];
      if not (Data is TJSONString) then
        RaiseMsg(20180228231555,El,IntToStr(i)+':'+GetObjName(Data));
      if ImplJS.GlobalJS=nil then
        ImplJS.GlobalJS:=TStringList.Create;
      ImplJS.GlobalJS.Add(Data.AsString);
      end;
    end;

  ImplJS.ShortRefs:=TFPList.Create;
  ReadElementList(Obj,El,'ShortRefs',ImplJS.ShortRefs,false,aContext);
  if ImplJS.ShortRefs.Count=0 then
    FreeAndNil(ImplJS.ShortRefs);
end;

procedure TPCUReader.ReadImplCommand(Obj: TJSONObject; El: TPasImplCommand;
  aContext: TPCUReaderContext);
// an empty statement, e.g. if expr then else ;
begin
  ReadPasElement(Obj,El,aContext);
end;

procedure TPCUReader.ReadImplBeginBlock(Obj: TJSONObject;
  El: TPasImplBeginBlock; aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElementList(Obj,El,'Elements',El.Elements,
    {$IFDEF CheckPasTreeRefCount}'TPasImplBeginBlock.Elements'{$ELSE}true{$ENDIF}
    ,aContext);
end;

procedure TPCUReader.ReadImplAsmStatement(Obj: TJSONObject;
  El: TPasImplAsmStatement; aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadStrings(Obj,El,'Tokens',El.Tokens);
end;

procedure TPCUReader.ReadImplRepeatUntil(Obj: TJSONObject;
  El: TPasImplRepeatUntil; aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElementList(Obj,El,'Elements',El.Elements,
    {$IFDEF CheckPasTreeRefCount}'TPasImplRepeatUntil.Elements'{$ELSE}true{$ENDIF}
    ,aContext);
  El.ConditionExpr:=ReadExpr(Obj,El,'Cond',aContext);
end;

procedure TPCUReader.ReadImplIfElse(Obj: TJSONObject; El: TPasImplIfElse;
  aContext: TPCUReaderContext);
var
  Branch: TPasImplElement;
begin
  ReadPasElement(Obj,El,aContext);
  El.ConditionExpr:=ReadExpr(Obj,El,'Cond',aContext);
  Branch:=TPasImplElement(ReadElementProperty(Obj,El,'Then',TPasImplElement,aContext));
  if Branch<>nil then
    El.AddElement(Branch); // sets El.IfBranch
  Branch:=TPasImplElement(ReadElementProperty(Obj,El,'Else',TPasImplElement,aContext));
  if Branch<>nil then
    El.AddElement(Branch); // sets El.ElseBranch
end;

procedure TPCUReader.ReadImplWhileDo(Obj: TJSONObject; El: TPasImplWhileDo;
  aContext: TPCUReaderContext);
var
  Body: TPasImplElement;
begin
  ReadPasElement(Obj,El,aContext);
  El.ConditionExpr:=ReadExpr(Obj,El,'Cond',aContext);
  Body:=TPasImplElement(ReadElementProperty(Obj,El,'Body',TPasImplElement,aContext));
  if Body<>nil then
    El.AddElement(Body);
end;

procedure TPCUReader.ReadImplWithDo(Obj: TJSONObject; El: TPasImplWithDo;
  aContext: TPCUReaderContext);
var
  Body: TPasImplElement;
begin
  ReadPasElement(Obj,El,aContext);
  ReadElementList(Obj,El,'Exprs',El.Expressions,
    {$IFDEF CheckPasTreeRefCount}'TPasImplWithDo.Elements'{$ELSE}true{$ENDIF}
    ,aContext);
  Body:=TPasImplElement(ReadElementProperty(Obj,El,'Body',TPasImplElement,aContext));
  if Body<>nil then
    El.AddElement(Body);
end;

procedure TPCUReader.ReadImplCaseOf(Obj: TJSONObject; El: TPasImplCaseOf;
  aContext: TPCUReaderContext);
var
  Elements: TFPList;
  Sub: TPasElement;
begin
  ReadPasElement(Obj,El,aContext);
  El.CaseExpr:=ReadExpr(Obj,El,'Expr',aContext);
  Elements:=El.Elements;
  ReadElementList(Obj,El,'Of',Elements,
    {$IFDEF CheckPasTreeRefCount}'TPasImplCaseOf.Elements'{$ELSE}true{$ENDIF}
    ,aContext);
  if (Elements.Count>0) then
    begin
    Sub:=TPasElement(Elements[Elements.Count-1]);
    if Sub is TPasImplCaseElse then
      begin
      El.ElseBranch:=TPasImplCaseElse(Sub);
      Sub.AddRef;
      end;
    end;
end;

procedure TPCUReader.ReadImplCaseStatement(Obj: TJSONObject;
  El: TPasImplCaseStatement; aContext: TPCUReaderContext);
var
  Body: TPasImplElement;
begin
  ReadPasElement(Obj,El,aContext);
  ReadElementList(Obj,El,'Exprs',El.Expressions,
    {$IFDEF CheckPasTreeRefCount}'TPasImplCaseStatement.Elements'{$ELSE}true{$ENDIF}
    ,aContext);
  Body:=TPasImplElement(ReadElementProperty(Obj,El,'Body',TPasImplElement,aContext));
  if Body<>nil then
    El.AddElement(Body);
end;

procedure TPCUReader.ReadImplCaseElse(Obj: TJSONObject; El: TPasImplCaseElse;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElementList(Obj,El,'Elements',El.Elements,
    {$IFDEF CheckPasTreeRefCount}'TPasImplCaseElse.Elements'{$ELSE}true{$ENDIF}
    ,aContext);
end;

procedure TPCUReader.ReadImplForLoop(Obj: TJSONObject; El: TPasImplForLoop;
  aContext: TPCUReaderContext);
var
  s: string;
  Body: TPasImplElement;
begin
  ReadPasElement(Obj,El,aContext);
  if ReadString(Obj,'Loop',s,El) then
    case s of
    'Normal': ;
    'Down': El.LoopType:=ltDown;
    'In': El.LoopType:=ltIn;
    else
      RaiseMsg(20200105195924,El,s);
    end;
  El.VariableName:=ReadExpr(Obj,El,'Var',aContext);
  El.StartExpr:=ReadExpr(Obj,El,'Start',aContext);
  El.EndExpr:=ReadExpr(Obj,El,'End',aContext);
  Body:=TPasImplElement(ReadElementProperty(Obj,El,'Body',TPasImplElement,aContext));
  if Body<>nil then
    El.AddElement(Body);
end;

procedure TPCUReader.ReadImplAssign(Obj: TJSONObject; El: TPasImplAssign;
  aContext: TPCUReaderContext);
var
  s: string;
begin
  ReadPasElement(Obj,El,aContext);
  if ReadString(Obj,'Kind',s,El) then
    case s of
    'Default': ;
    'Add': El.Kind:=akAdd;
    'Minus': El.Kind:=akMinus;
    'Mul': El.Kind:=akMul;
    'Division': El.Kind:=akDivision;
    else
      RaiseMsg(20200105200423,El,s);
    end;
  El.left:=ReadExpr(Obj,El,'Left',aContext);
  El.right:=ReadExpr(Obj,El,'Right',aContext);
end;

procedure TPCUReader.ReadImplSimple(Obj: TJSONObject; El: TPasImplSimple;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  El.Expr:=ReadExpr(Obj,El,'Expr',aContext);
end;

procedure TPCUReader.ReadImplTry(Obj: TJSONObject; El: TPasImplTry;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElementList(Obj,El,'Try',El.Elements,
    {$IFDEF CheckPasTreeRefCount}'TPasImplTry.Elements'{$ELSE}true{$ENDIF}
    ,aContext);
  El.FinallyExcept:=TPasImplTryHandler(ReadElementProperty(Obj,El,'Finally',TPasImplTryHandler,aContext));
  El.ElseBranch:=TPasImplTryExceptElse(ReadElementProperty(Obj,El,'Else',TPasImplTryExceptElse,aContext));
end;

procedure TPCUReader.ReadImplTryHandler(Obj: TJSONObject;
  El: TPasImplTryHandler; aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  ReadElementList(Obj,El,'El',El.Elements,
    {$IFDEF CheckPasTreeRefCount}'TPasImplTryHandler.Elements'{$ELSE}true{$ENDIF}
    ,aContext);
end;

procedure TPCUReader.ReadImplExceptOn(Obj: TJSONObject; El: TPasImplExceptOn;
  aContext: TPCUReaderContext);
var
  Body: TPasImplElement;
begin
  ReadPasElement(Obj,El,aContext);
  El.VarEl:=TPasVariable(ReadElementProperty(Obj,El,'Var',TPasVariable,aContext));
  if El.VarEl<>nil then
    begin
    El.TypeEl:=El.VarEl.VarType;
    El.TypeEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasVariable.VarType'){$ENDIF};
    end
  else
    ReadElType(Obj,'VarType',El,@Set_ExceptOn_TypeEl,aContext);
  Body:=TPasImplElement(ReadElementProperty(Obj,El,'Body',TPasImplElement,aContext));
  if Body<>nil then
    El.AddElement(Body);
end;

procedure TPCUReader.ReadImplRaise(Obj: TJSONObject; El: TPasImplRaise;
  aContext: TPCUReaderContext);
begin
  ReadPasElement(Obj,El,aContext);
  El.ExceptObject:=ReadExpr(Obj,El,'Obj',aContext);
  El.ExceptAddr:=ReadExpr(Obj,El,'Addr',aContext);
end;

constructor TPCUReader.Create;
begin
  inherited Create;
  FInitialFlags:=TPCUInitialFlags.Create;
  FPendingIdentifierScopes:=TObjectList.Create(true);
  FPendingForwardProcs:=TFPList.Create;
end;

destructor TPCUReader.Destroy;
begin
  FreeAndNil(FJSON);
  inherited Destroy;
  FreeAndNil(FPendingForwardProcs);
  FreeAndNil(FPendingIdentifierScopes);
  FreeAndNil(FInitialFlags);
end;

procedure TPCUReader.Clear;
var
  i: Integer;
begin
  for i:=0 to length(FElementRefsArray)-1 do
    if (FElementRefsArray[i]<>nil) and (FElementRefsArray[i].Element=nil) then
      FElementRefsArray[i].Free;
  FElementRefsArray:=nil;
  FPendingIdentifierScopes.Clear;
  while FPendingSpecialize<>nil do
    DeletePendingSpecialize(FPendingSpecialize);
  FPendingForwardProcs.Clear;

  inherited Clear;
  FInitialFlags.Clear;
end;

procedure TPCUReader.ReadPCU(aResolver: TPas2JSResolver; aStream: TStream);
var
  JParser: TJSONParser;
  Data: TJSONData;
  FirstBytes: string;
  Compressed: Boolean;
  Decomp: Tdecompressionstream;
  Count: Cardinal;
  Src: TStream;
begin
  FirstBytes:='';
  SetLength(FirstBytes,4);
  if aStream.Read(FirstBytes[1],4)<4 then
    RaiseMsg(20180313232754,nil);
  aStream.Seek(-4,soCurrent);
  Compressed:=(FirstBytes[1]<>'{') and (FirstBytes<>UTF8BOM+'{');
  JParser:=nil;
  Src:=nil;
  try
    if Compressed then
      begin
      try
        Decomp:=Tdecompressionstream.create(aStream);
        try
          Count:=Decomp.ReadDWord;
          if Count>123456789 then
            RaiseMsg(20180313233209,'too big, invalid format');
          Src:=TMemoryStream.Create;
          Src.Size:=Count;
          Decomp.read(TMemoryStream(Src).Memory^,Src.Size);
        finally
          Decomp.Free;
        end;
      except
        on E: Edecompressionerror do
          RaiseMsg(20180704162214,'decompression error, file corrupt: '+E.Message);
      end;
      Src.Position:=0;
      end
    else
      Src:=aStream;

    {$IFDEF VerbosePCUUncompressed}
    {AllowWriteln}
    writeln('TPCUReader.ReadPCU SRC START====================================');
    SetLength(FirstBytes,Src.Size);
    Src.read(FirstBytes[1],length(FirstBytes));
    writeln(FirstBytes);
    Src.Position:=0;
    writeln('TPCUReader.ReadPCU SRC END======================================');
    {AllowWriteln-}
    {$ENDIF}
    JParser:=TJSONParser.Create(Src,[joUTF8,joStrict]);
    Data:=JParser.Parse;
    if not (Data is TJSONObject) then
      RaiseMsg(20180202130727,'expected JSON object, but found '+JSONTypeName(Data.JSONType));
  finally
    if Src<>aStream then
      Src.Free;
    JParser.Free;
  end;
  ReadJSONHeader(aResolver,TJSONObject(Data));
end;

procedure TPCUReader.ReadJSONHeader(aResolver: TPas2JSResolver;
  Obj: TJSONObject);
var
  aName: String;
  Data: TJSONData;
  i: Integer;
begin
  FResolver:=aResolver;
  FParser:=Resolver.CurrentParser;
  FScanner:=FParser.Scanner;
  FJSON:=Obj;
  {$IF defined(VerbosePCUFiler) or defined(VerboseUnitQueue)}
  writeln('TPCUReader.ReadJSONHeader START ');
  {$ENDIF}

  ReadHeaderMagic(Obj);
  ReadHeaderVersion(Obj);
  ReadGUID(Obj);

  for i:=0 to Obj.Count-1 do
    begin
    aName:=Obj.Names[i];
    {$IFDEF VerbosePCUFiler}
    writeln('TPCUReader.ReadJSONHeader ',aName);
    {$ENDIF}
    Data:=Obj.Elements[aName];
    case aName of
    'FileType': ; // done in ReadHeaderMagic
    'Version': ; // done in ReadHeaderVersion
    'GUID': ; // done in ReadGUID
    'TargetPlatform': ReadTargetPlatform(Data);
    'TargetProcessor': ReadTargetProcessor(Data);
    'Sources': ReadSrcFiles(Data);
    'InitParserOpts': InitialFlags.ParserOptions:=ReadParserOptions(Obj,nil,aName,PCUDefaultParserOptions);
    'InitModeSwitches': InitialFlags.ModeSwitches:=ReadModeSwitches(Obj,nil,aName,PCUDefaultModeSwitches);
    'InitBoolSwitches': InitialFlags.BoolSwitches:=ReadBoolSwitches(Obj,nil,aName,PCUDefaultBoolSwitches);
    'InitConverterOpts': InitialFlags.ConverterOptions:=ReadConverterOptions(Obj,nil,aName,PCUDefaultConverterOptions);
    'FinalParserOpts': Parser.Options:=ReadParserOptions(Obj,nil,aName,InitialFlags.ParserOptions);
    'FinalModeSwitches': Scanner.CurrentModeSwitches:=ReadModeSwitches(Obj,nil,aName,InitialFlags.ModeSwitches);
    'FinalBoolSwitches': Scanner.CurrentBoolSwitches:=ReadBoolSwitches(Obj,nil,aName,InitialFlags.BoolSwitches);
    'Module': ReadModuleHeader(Data);
    else
      ReadHeaderItem(aName,Data);
    end;
    end;
  {$IFDEF VerbosePCUFiler}
  writeln('TPCUReader.ReadJSONHeader END');
  {$ENDIF}
end;

function TPCUReader.ReadContinue: boolean;
var
  Obj, SubObj: TJSONObject;
  aContext: TPCUReaderContext;
begin
  {$IF defined(VerbosePCUFiler) or defined(VerboseUnitQueue)}
  writeln('TPCUReader.ReadContinue START ',Resolver.RootElement.Name);
  {$ENDIF}
  Obj:=JSON;
  if not ReadObject(Obj,'Module',SubObj,nil) then
    RaiseMsg(20180307114005,'missing Module');
  aContext:=CreateContext;
  try
    Result:=ReadModule(SubObj,aContext);
  finally
    aContext.Free;
  end;
  {$IF defined(VerbosePCUFiler) or defined(VerboseUnitQueue)}
  writeln('TPCUReader.ReadContinue END');
  {$ENDIF}
end;

function TPCUReader.GetPCUExt: string;
begin
  Result:=ExtractFileExt(PCUFilename);
  if Result='' then
    Result:='pcu'
  else
    System.Delete(Result,1,1); // remove leading dot
end;

{ TPas2JSPrecompileFormats }

function TPas2JSPrecompileFormats.GetItems(Index: integer
  ): TPas2JSPrecompileFormat;
begin
  Result:=TPas2JSPrecompileFormat(FItems[Index]);
end;

constructor TPas2JSPrecompileFormats.Create;
begin
  FItems:=TObjectList.Create(true);
end;

destructor TPas2JSPrecompileFormats.Destroy;
begin
  Clear;
  FreeAndNil(FItems);
  inherited Destroy;
end;

procedure TPas2JSPrecompileFormats.Clear;
begin
  if (PCUFormat<>nil) and (FItems.IndexOf(PCUFormat)>=0) then
    PCUFormat:=nil;
  FItems.Clear;
end;

function TPas2JSPrecompileFormats.Count: integer;
begin
  Result:=FItems.Count;
end;

function TPas2JSPrecompileFormats.Add(aFormat: TPas2JSPrecompileFormat
  ): TPas2JSPrecompileFormats;
begin
  if FindExt(aFormat.Ext)<>nil then
    begin
    aFormat.Free;
    raise Exception.Create('pas2js precompile extension already exists');
    end;
  FItems.Add(aFormat);
  Result:=Self;
end;

function TPas2JSPrecompileFormats.Add(const Ext, Description: string;
  const Reader: TPCUReaderClass; const Writer: TPCUWriterClass
  ): TPas2JSPrecompileFormat;
begin
  Result:=TPas2JSPrecompileFormat.Create;
  Result.Ext:=Ext;
  Result.Description:=Description;
  Result.ReaderClass:=Reader;
  Result.WriterClass:=Writer;
  Result.Enabled:=true;
  Add(Result);
end;

function TPas2JSPrecompileFormats.IndexOf(aFormat: TPas2JSPrecompileFormat
  ): integer;
begin
  Result:=FItems.IndexOf(aFormat);
end;

function TPas2JSPrecompileFormats.FindExt(Ext: string): TPas2JSPrecompileFormat;
var
  i: Integer;
begin
  Result:=nil;
  if (Ext='') then exit;
  if Ext[1]='.' then
    begin
    system.Delete(Ext,1,1);
    if Ext='' then exit;
    end;
  for i:=0 to Count-1 do
    if CompareText(Ext,Items[i].Ext)=0 then
      exit(Items[i]);
end;

function TPas2JSPrecompileFormats.Remove(aFormat: TPas2JSPrecompileFormat
  ): integer;
begin
  Result:=IndexOf(aFormat);
  if Result>=0 then
    FItems.Delete(Result);
end;

function TPas2JSPrecompileFormats.Delete(Index: integer): TPas2JSPrecompileFormats;
begin
  FItems.Delete(Index);
  Result:=Self;
end;

initialization
  PrecompileFormats:=TPas2JSPrecompileFormats.Create;
finalization
  PrecompileFormats.Free;
  PrecompileFormats:=nil;

end.

