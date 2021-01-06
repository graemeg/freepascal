{
    Copyright (c) 2013 by Yury Sidorov and the FPC Development Team

    Base classes for a custom output of a PPU File

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************}

unit ppuout;
{$mode objfpc}{$H+}
{$I+}

interface

uses SysUtils, cclasses, Classes;

type
  TPpuDefType = (dtNone, dtUnit, dtObject, dtRecord, dtProc, dtField, dtProp, dtParam, dtVar,
                 dtTypeRef, dtConst, dtProcType, dtEnum, dtSet, dtClassRef, dtArray, dtPointer,
                 dtOrd, dtFloat, dtString, dtFile, dtVariant, dtUndefined, dtFormal);

  TPpuDef = class;
  TPpuContainerDef = class;
  TPpuUnitDef = class;

  { TPpuOutput }
  TPpuOutput = class
  private
    FOutFileHandle: THandle;
    FOutBuf: array[0..10000] of char;
    FOutBufPos: integer;
    FIndent: integer;
    FIndentSize: integer;
    FIndStr: string;
    FNoIndent: boolean;
    procedure Flush;
    procedure SetIndent(AValue: integer);
    procedure SetIndentSize(AValue: integer);
  protected
    procedure WriteObjectStart(const AName: string; Def: TPpuDef = nil); virtual;
    procedure WriteObjectEnd(const AName: string; Def: TPpuDef = nil); virtual;
    procedure WriteArrayStart(const AName: string); virtual;
    procedure WriteArrayEnd(const AName: string); virtual;
    procedure WriteStr(const AName, AValue: string); virtual;
    procedure WriteInt(const AName: string; AValue: Int64; Signed: boolean = True); virtual;
    procedure WriteFloat(const AName: string; AValue: extended); virtual;
    procedure WriteBool(const AName: string; AValue: boolean); virtual;
    procedure WriteNull(const AName: string); virtual;
  public
    constructor Create(OutFileHandle: THandle); virtual;
    destructor Destroy; override;
    procedure Write(const s: string);
    procedure WriteLn(const s: string = '');
    procedure IncI; virtual;
    procedure DecI; virtual;
    procedure Init; virtual;
    procedure Done; virtual;
    property Indent: integer read FIndent write SetIndent;
    property IndentSize: integer read FIndentSize write SetIndentSize;
  end;

  { TPpuRef }
  TPpuRef = class
  private
    FId: cardinal;
    function GetId: cardinal;
    function GetIsSymId: boolean;
    procedure SetId(AValue: cardinal);
    procedure SetIsSymId(AValue: boolean);
  public
    UnitIndex: word;
    constructor Create;
    procedure Write(Output: TPpuOutput; const RefName: string);
    property Id: cardinal read GetId write SetId;
    property IsSymId: boolean read GetIsSymId write SetIsSymId;
    function IsCurUnit: boolean; inline;
    function IsNull: boolean; inline;
  end;

  TPpuFilePos = record
    FileIndex: dword;
    Line, Col: integer;
  end;

  TPpuDefVisibility = (dvPublic, dvPublished, dvProtected, dvPrivate, dvHidden);

  TPpuAttr = record
    ParaCount: LongInt;
    TypeSym: TPpuRef;
    TypeConstr: TPpuRef;
  end;

  { TPpuDef }

  TPpuDef = class
  private
    FId: cardinal;
    FParent: TPpuContainerDef;
    FParentUnit: TPpuUnitDef;
    function GetDefTypeName: string;
    function GetId: cardinal;
    function GetParentUnit: TPpuUnitDef;
    procedure SetId(AValue: cardinal);
    procedure SetParent(AValue: TPpuContainerDef);

  protected
    procedure WriteDef(Output: TPpuOutput); virtual;
    procedure Done; virtual;

  public
    DefType: TPpuDefType;
    Name: ansistring;
    FilePos: TPpuFilePos;
    // Symbol/definition reference
    Ref: TPpuRef;
    Visibility: TPpuDefVisibility;
    GenericDummy: Boolean;
    Attrs: array of TPpuAttr;

    constructor Create(AParent: TPpuContainerDef); virtual; reintroduce;
    destructor Destroy; override;
    procedure Write(Output: TPpuOutput; const AttrName: string = '');
    function CanWrite: boolean; virtual;
    procedure SetSymId(AId: integer);
    property Parent: TPpuContainerDef read FParent write SetParent;
    property ParentUnit: TPpuUnitDef read GetParentUnit;
    property Id: cardinal read GetId write SetId;
    property DefTypeName: string read GetDefTypeName;
  end;

  { TPpuContainerDef }
  TPpuContainerDef = class(TPpuDef)
  private
    FItems: TList;
    function GetCount: integer;
    function GetItem(Index: Integer): TPpuDef;
    procedure SetItem(Index: Integer; AValue: TPpuDef);

  protected
    procedure WriteDef(Output: TPpuOutput); override;
    procedure BeforeWriteItems(Output: TPpuOutput); virtual;
    procedure Done; override;

  public
    ItemsName: string;

    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
    function Add(Def: TPpuDef): integer;
    property Items[Index: Integer]: TPpuDef read GetItem write SetItem; default;
    property Count: integer read GetCount;
  end;

  { TPpuTypeRef }
  TPpuTypeRef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  { TPpuUnitDef }
  TPpuUnitDef = class(TPpuContainerDef)
  private
    FIndexById: THashSet;
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    Version: cardinal;
    Crc, IntfCrc: cardinal;
    TargetOS, TargetCPU: string;
    UsedUnits: TPpuContainerDef;
    RefUnits: array of string;
    SourceFiles: TPpuContainerDef;
    LongVersion: Cardinal;

    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
    function FindById(AId: integer; FindSym: boolean = False): TPpuDef;
  end;

  { TPpuSrcFile }
  TPpuSrcFile = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    FileTime: TDateTime;
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  TPpuProcOption = (poProcedure, poFunction, poConstructor, poDestructor, poOperator,
                    poClassMethod, poVirtual, poAbstract, poOverriding, poOverload, poInline);
  TPpuProcOptions = set of TPpuProcOption;

  { TPpuProcDef }
  TPpuProcDef = class(TPpuContainerDef)
  protected
    procedure BeforeWriteItems(Output: TPpuOutput); override;
  public
    ReturnType: TPpuRef;
    Options: TPpuProcOptions;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  { TPpuProcTypeDef }
  TPpuProcTypeDef = class(TPpuProcDef)
  protected
    procedure BeforeWriteItems(Output: TPpuOutput); override;
  public
    MethodPtr: boolean;
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  TPpuConstType = (ctUnknown, ctInt, ctFloat, ctStr, ctSet, ctPtr);

  { TPpuConstDef }
  TPpuConstDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    ConstType: TPpuConstType;
    TypeRef: TPpuRef;
    VInt: Int64;
    VFloat: extended;
    VStr: string;
    VSet: array[0..31] of byte;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
    function CanWrite: boolean; override;
  end;

  { TPpuVarDef }
  TPpuVarDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    VarType: TPpuRef;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  TPpuParamSpez = (psValue, psVar, psOut, psConst, psConstRef, psHidden);

  { TPpuParamDef }
  TPpuParamDef = class(TPpuVarDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    Spez: TPpuParamSpez;
    DefaultValue: TPpuRef;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
    function CanWrite: boolean; override;
  end;

  TPpuObjType = (otUnknown, otClass, otObject, otInterface, otHelper);
  TPpuObjOption = (ooIsAbstract, ooCopied, ooAbstractMethods);
  TPpuObjOptions = set of TPpuObjOption;

  { TPpuObjectDef }
  TPpuObjectDef = class(TPpuContainerDef)
  protected
    procedure BeforeWriteItems(Output: TPpuOutput); override;
  public
    ObjType: TPpuObjType;
    Ancestor: TPpuRef;
    Options: TPpuObjOptions;
    IID: string;
    HelperParent: TPpuRef;
    Size: integer;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
    function CanWrite: boolean; override;
  end;

  { TPpuFieldDef }
  TPpuFieldDef = class(TPpuVarDef)
  public
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  TPpuPropOption = (poDefault);
  TPpuPropOptions = set of TPpuPropOption;

  { TPpuPropDef }
  TPpuPropDef = class(TPpuContainerDef)
  protected
    procedure BeforeWriteItems(Output: TPpuOutput); override;
  public
    PropType: TPpuRef;
    Getter, Setter: TPpuRef;
    Options: TPpuPropOptions;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  { TPpuRecordDef }
  TPpuRecordDef = class(TPpuObjectDef)
  protected
    procedure BeforeWriteItems(Output: TPpuOutput); override;
  public
    constructor Create(AParent: TPpuContainerDef); override;
    function CanWrite: boolean; override;
  end;

  { TPpuClassRefDef }
  TPpuClassRefDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    ClassRef: TPpuRef;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  TPpuArrayOption = (aoDynamic);
  TPpuArrayOptions = set of TPpuArrayOption;

  { TPpuArrayDef }
  TPpuArrayDef = class(TPpuContainerDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    ElType: TPpuRef;
    RangeType: TPpuRef;
    RangeLow, RangeHigh: Int64;
    Options: TPpuArrayOptions;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  { TPpuEnumDef }
  TPpuEnumDef = class(TPpuContainerDef)
  protected
    procedure BeforeWriteItems(Output: TPpuOutput); override;
  public
    ElLow, ElHigh: integer;
    Size: byte;
    CopyFrom: TPpuRef;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  { TPpuSetDef }
  TPpuSetDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    ElType: TPpuRef;
    SetBase, SetMax: integer;
    Size: byte;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  { TPpuPointerDef }
  TPpuPointerDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    Ptr: TPpuRef;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  TPpuOrdType = (otVoid, otUInt, otSInt, otPasBool, otBool, otChar, otCurrency);

  { TPpuOrdDef }
  TPpuOrdDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    OrdType: TPpuOrdType;
    Size: byte;
    RangeLow, RangeHigh: Int64;
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  TPpuFloatType = (pftSingle, pftDouble, pftExtended, pftComp, pftCurrency, pftFloat128);

  { TPpuFloatDef }
  TPpuFloatDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    FloatType: TPpuFloatType;
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  TPpuStrType = (stShort, stAnsi, stWide, stUnicode, stLong);

  { TPpuStringDef }
  TPpuStringDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    StrType: TPpuStrType;
    Len: integer;
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  TPpuFileType = (ftText, ftTyped, ftUntyped);

  { TPpuFileDef }
  TPpuFileDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    FileType: TPpuFileType;
    TypeRef: TPpuRef;
    constructor Create(AParent: TPpuContainerDef); override;
    destructor Destroy; override;
  end;

  { TPpuVariantDef }
  TPpuVariantDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    IsOLE: boolean;
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  { TPpuUndefinedDef }
  TPpuUndefinedDef = class(TPpuDef)
  public
    constructor Create(AParent: TPpuContainerDef); override;
  end;

  { TPpuFormalDef }
  TPpuFormalDef = class(TPpuDef)
  protected
    procedure WriteDef(Output: TPpuOutput); override;
  public
    IsTyped: boolean;
    constructor Create(AParent: TPpuContainerDef); override;
  end;

implementation

const
  DefTypeNames: array[TPpuDefType] of string =
    ('', 'unit', 'obj', 'rec', 'proc', 'field', 'prop', 'param', 'var',
     'type', 'const', 'proctype', 'enum', 'set', 'classref', 'array', 'ptr',
     'ord', 'float', 'string', 'file', 'variant', 'undefined', 'formal');

  ProcOptionNames: array[TPpuProcOption] of string =
    ('procedure', 'function', 'constructor', 'destructor', 'operator',
     'classmethod', 'virtual', 'abstract', 'overriding', 'overload', 'inline');

  DefVisibilityNames: array[TPpuDefVisibility] of string =
    ('public', 'published', 'protected', 'private', '');

  ParamSpezNames: array[TPpuParamSpez] of string =
    ('value', 'var', 'out', 'const', 'constref', '');

  ObjTypeNames: array[TPpuObjType] of string =
    ('', 'class', 'object', 'interface', 'helper');

  ObjOptionNames: array[TPpuObjOption] of string =
    ('abstract','copied','abstract_methods');

  PropOptionNames: array[TPpuPropOption] of string =
    ('default');

  ArrayOptionNames: array[TPpuArrayOption] of string =
    ('dynamic');

  ConstTypeNames: array[TPpuConstType] of string =
    ('unknown', 'int', 'float', 'string', 'set', 'pointer');

  OrdTypeNames: array[TPpuOrdType] of string =
    ('void', 'uint', 'sint', 'pasbool', 'bool', 'char', 'currency');

  FloatTypeNames: array[TPpuFloatType] of string =
    ('single', 'double', 'extended', 'comp', 'currency', 'float128');

  StrTypeNames: array[TPpuStrType] of string =
    ('short', 'ansi', 'wide', 'unicode', 'long');

  FileTypeNames: array[TPpuFileType] of string =
    ('text', 'typed', 'untyped');

  SymIdBit = $80000000;
  InvalidId = cardinal(-1);
  InvalidUnit = word(-1);

function IsSymId(Id: cardinal): boolean; inline;
begin
  Result:=Id and SymIdBit <> 0;
end;

{ TPpuUndefinedDef }

constructor TPpuUndefinedDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtUndefined;
end;

{ TPpuFormalDef }

procedure TPpuFormalDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  Output.WriteBool('IsTyped', IsTyped);
end;

constructor TPpuFormalDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtFormal;
end;

{ TPpuVariantDef }

procedure TPpuVariantDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  if IsOLE then
    Output.WriteBool('OleVariant', True);
end;

constructor TPpuVariantDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtVariant;
end;

{ TPpuFileDef }

procedure TPpuFileDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  Output.WriteStr('FileType', FileTypeNames[FileType]);
  if FileType = ftTyped then
    TypeRef.Write(Output, 'TypeRef');
end;

constructor TPpuFileDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtFile;
  TypeRef:=TPpuRef.Create;
end;

destructor TPpuFileDef.Destroy;
begin
  TypeRef.Free;
  inherited Destroy;
end;

{ TPpuStringDef }

procedure TPpuStringDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  Output.WriteStr('StrType', StrTypeNames[StrType]);
  if Len >= 0 then
    Output.WriteInt('Len', Len);
end;

constructor TPpuStringDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtString;
end;

{ TPpuFloatDef }

procedure TPpuFloatDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  Output.WriteStr('FloatType', FloatTypeNames[FloatType]);
end;

constructor TPpuFloatDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtFloat;
end;

{ TPpuOrdDef }

procedure TPpuOrdDef.WriteDef(Output: TPpuOutput);
var
  Signed: boolean;
begin
  inherited WriteDef(Output);
  with Output do begin
    WriteStr('OrdType', OrdTypeNames[OrdType]);
    WriteInt('Size', Size);
    Signed:=OrdType in [otSInt, otCurrency, otBool];
    WriteInt('Low', RangeLow, Signed);
    WriteInt('High', RangeHigh, Signed);
  end;
end;

constructor TPpuOrdDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtOrd;
end;

{ TPpuPointerDef }

procedure TPpuPointerDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  Ptr.Write(Output, 'Ptr');
end;

constructor TPpuPointerDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtPointer;
  Ptr:=TPpuRef.Create;
end;

destructor TPpuPointerDef.Destroy;
begin
  Ptr.Free;
  inherited Destroy;
end;

{ TPpuSetDef }

procedure TPpuSetDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  with Output do begin
    WriteInt('Size', Size);
    WriteInt('Base', SetBase);
    WriteInt('Max', SetMax);
  end;
  ElType.Write(Output, 'ElType');
end;

constructor TPpuSetDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtSet;
  ElType:=TPpuRef.Create;
end;

destructor TPpuSetDef.Destroy;
begin
  ElType.Free;
  inherited Destroy;
end;

{ TPpuEnumDef }

procedure TPpuEnumDef.BeforeWriteItems(Output: TPpuOutput);
begin
  inherited BeforeWriteItems(Output);
  with Output do begin
    WriteInt('Low', ElLow);
    WriteInt('High', ElHigh);
    WriteInt('Size', Size);
  end;
  if not CopyFrom.IsNull then
    CopyFrom.Write(Output, 'CopyFrom');
end;

constructor TPpuEnumDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtEnum;
  ItemsName:='Elements';
  CopyFrom:=TPpuRef.Create;
end;

destructor TPpuEnumDef.Destroy;
begin
  CopyFrom.Free;
  inherited Destroy;
end;

{ TPpuConstDef }

procedure TPpuConstDef.WriteDef(Output: TPpuOutput);
var
  s, ss: string;
  i: integer;
begin
  inherited WriteDef(Output);
  with Output do begin
    WriteStr('ValType', ConstTypeNames[ConstType]);
    s:='Value';
    case ConstType of
      ctUnknown: ;
      ctInt:
        WriteInt(s, VInt);
      ctFloat:
        WriteFloat(s, VFloat);
      ctStr:
        WriteStr(s, VStr);
      ctPtr:
        if VInt = 0 then
          WriteNull(s)
        else
          if QWord(VInt) > $FFFFFFFF then
            WriteStr(s, hexStr(QWord(VInt), 8))
          else
            WriteStr(s, hexStr(QWord(VInt), 16));
      ctSet:
        begin
          ss:='';
          for i:=Low(VSet) to High(VSet) do
            ss:=ss + hexStr(VSet[i], 2);
          WriteStr(s, ss);
        end;
    end;
  end;
  if not TypeRef.IsNull then
    TypeRef.Write(Output, 'TypeRef');
end;

constructor TPpuConstDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtConst;
  TypeRef:=TPpuRef.Create;
  ConstType:=ctUnknown;
end;

destructor TPpuConstDef.Destroy;
begin
  TypeRef.Free;
  inherited Destroy;
end;

function TPpuConstDef.CanWrite: boolean;
begin
  Result:=inherited CanWrite and (ConstType <> ctUnknown);
end;

{ TPpuArrayDef }

procedure TPpuArrayDef.WriteDef(Output: TPpuOutput);
var
  opt: TPpuArrayOption;
begin
  inherited WriteDef(Output);
  if Options <> [] then begin
    Output.WriteArrayStart('Options');
    for opt:=Low(opt) to High(opt) do
      if opt in Options then
        Output.WriteStr('', ArrayOptionNames[opt]);
    Output.WriteArrayEnd('Options');
  end;
  ElType.Write(Output, 'ElType');
  RangeType.Write(Output, 'RangeType');
  Output.WriteInt('Low', RangeLow);
  Output.WriteInt('High', RangeHigh);
end;

constructor TPpuArrayDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  ItemsName:='Types';
  DefType:=dtArray;
  ElType:=TPpuRef.Create;
  RangeType:=TPpuRef.Create;
end;

destructor TPpuArrayDef.Destroy;
begin
  ElType.Free;
  RangeType.Free;
  inherited Destroy;
end;

{ TPpuClassRefDef }

procedure TPpuClassRefDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  ClassRef.Write(Output, 'Ref');
end;

constructor TPpuClassRefDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtClassRef;
  ClassRef:=TPpuRef.Create;
end;

destructor TPpuClassRefDef.Destroy;
begin
  ClassRef.Free;
  inherited Destroy;
end;

{ TPpuRecordDef }

procedure TPpuRecordDef.BeforeWriteItems(Output: TPpuOutput);
begin
  inherited BeforeWriteItems(Output);
  if ooCopied in Options then
    Ancestor.Write(Output, 'CopyFrom');
end;

constructor TPpuRecordDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtRecord;
end;

function TPpuRecordDef.CanWrite: boolean;
begin
  Result:=True;
end;

{ TPpuPropDef }

procedure TPpuPropDef.BeforeWriteItems(Output: TPpuOutput);
var
  opt: TPpuPropOption;
begin
  inherited BeforeWriteItems(Output);
  PropType.Write(Output, 'PropType');
  Getter.Write(Output, 'Getter');
  Setter.Write(Output, 'Setter');
  if Options <> [] then begin
    Output.WriteArrayStart('Options');
    for opt:=Low(opt) to High(opt) do
      if opt in Options then
        Output.WriteStr('', PropOptionNames[opt]);
    Output.WriteArrayEnd('Options');
  end;
end;

constructor TPpuPropDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtProp;
  ItemsName:='Params';
  PropType:=TPpuRef.Create;
  Getter:=TPpuRef.Create;
  Setter:=TPpuRef.Create;
end;

destructor TPpuPropDef.Destroy;
begin
  Getter.Free;
  Setter.Free;
  PropType.Free;
  inherited Destroy;
end;

{ TPpuTypeRef }

procedure TPpuTypeRef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  Ref.Write(Output, 'Ref');
end;

constructor TPpuTypeRef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtTypeRef;
end;

{ TPpuFieldDef }

constructor TPpuFieldDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtField;
end;

{ TPpuParamDef }

procedure TPpuParamDef.WriteDef(Output: TPpuOutput);
var
  i, j: integer;
  d: TPpuDef;
begin
  inherited WriteDef(Output);
  if Spez <> psValue then
    Output.WriteStr('Spez', ParamSpezNames[Spez]);
  if not DefaultValue.IsNull then begin
    j:=DefaultValue.Id;
    for i:=0 to Parent.Count - 1 do begin
      d:=Parent[i];
      if (d.DefType = dtConst) and (d.Id = j) then begin
        d.Visibility:=dvPublic;
        d.Name:='';
        d.Write(Output, 'Default');
        d.Visibility:=dvHidden;
        break;
      end;
    end;
  end;
end;

constructor TPpuParamDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtParam;
  Spez:=psValue;
  DefaultValue:=TPpuRef.Create;
end;

destructor TPpuParamDef.Destroy;
begin
  DefaultValue.Free;
  inherited Destroy;
end;

function TPpuParamDef.CanWrite: boolean;
begin
  Result:=inherited CanWrite and (Spez <> psHidden);
end;

{ TPpuVarDef }

procedure TPpuVarDef.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  VarType.Write(Output, 'VarType');
end;

constructor TPpuVarDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtVar;
  VarType:=TPpuRef.Create;
end;

destructor TPpuVarDef.Destroy;
begin
  VarType.Free;
  inherited Destroy;
end;

{ TPpuObjectDef }

procedure TPpuObjectDef.BeforeWriteItems(Output: TPpuOutput);
var
  opt: TPpuObjOption;
begin
  inherited BeforeWriteItems(Output);
  if ObjType <> otUnknown then begin
    Output.WriteStr('ObjType', ObjTypeNames[ObjType]);
    Ancestor.Write(Output, 'Ancestor');
  end;
  if Options <> [] then begin
    Output.WriteArrayStart('Options');
    for opt:=Low(opt) to High(opt) do
      if opt in Options then
        Output.WriteStr('', ObjOptionNames[opt]);
    Output.WriteArrayEnd('Options');
  end;
  Output.WriteInt('Size', Size);
  if IID <> '' then
    Output.WriteStr('IID', IID);
  if not HelperParent.IsNull then
    HelperParent.Write(Output, 'HelperParent');
end;

constructor TPpuObjectDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtObject;
  ItemsName:='Fields';
  ObjType:=otUnknown;
  Ancestor:=TPpuRef.Create;
  HelperParent:=TPpuRef.Create;
end;

destructor TPpuObjectDef.Destroy;
begin
  Ancestor.Free;
  HelperParent.Free;
  inherited Destroy;
end;

function TPpuObjectDef.CanWrite: boolean;
begin
  Result:=inherited CanWrite and (ObjType <> otUnknown);
end;

{ TPpuRef }

function TPpuRef.GetId: cardinal;
begin
  if FId = InvalidId then
    Result:=InvalidId
  else
    Result:=FId and not SymIdBit;
end;

function TPpuRef.GetIsSymId: boolean;
begin
  Result:=FId and SymIdBit <> 0;
end;

procedure TPpuRef.SetId(AValue: cardinal);
begin
  if (FId = InvalidId) or (AValue = InvalidId) then
    FId:=AValue
  else
    FId:=AValue or (FId and SymIdBit);
end;

procedure TPpuRef.SetIsSymId(AValue: boolean);
begin
  if AValue then
    FId:=FId or SymIdBit
  else
    FId:=FId and not SymIdBit;
end;

constructor TPpuRef.Create;
begin
  UnitIndex:=InvalidUnit;
  FId:=InvalidId;
end;

procedure TPpuRef.Write(Output: TPpuOutput; const RefName: string);
begin
  with Output do
    if IsNull then
      WriteNull(RefName)
    else begin
      WriteObjectStart(RefName);
      if not IsCurUnit then
        WriteInt('Unit', UnitIndex);
      if IsSymId then
        WriteInt('SymId', Id)
      else
        WriteInt('Id', Id);
      WriteObjectEnd(RefName);
    end;
end;

function TPpuRef.IsCurUnit: boolean;
begin
  Result:=UnitIndex = InvalidUnit;
end;

function TPpuRef.IsNull: boolean;
begin
  Result:=Id = InvalidId;
end;

{ TPpuProcTypeDef }

procedure TPpuProcTypeDef.BeforeWriteItems(Output: TPpuOutput);
begin
  inherited BeforeWriteItems(Output);
  if MethodPtr then
    Output.WriteBool('MethodPtr', MethodPtr);
end;

constructor TPpuProcTypeDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtProcType;
end;

{ TPpuProcDef }

procedure TPpuProcDef.BeforeWriteItems(Output: TPpuOutput);
var
  opt: TPpuProcOption;
begin
  inherited BeforeWriteItems(Output);
  if Options <> [] then begin
    Output.WriteArrayStart('Options');
    for opt:=Low(opt) to High(opt) do
      if opt in Options then
        Output.WriteStr('', ProcOptionNames[opt]);
    Output.WriteArrayEnd('Options');
  end;
  ReturnType.Write(Output, 'RetType');
end;

constructor TPpuProcDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtProc;
  ItemsName:='Params';
  ReturnType:=TPpuRef.Create;
end;

destructor TPpuProcDef.Destroy;
begin
  ReturnType.Free;
  inherited Destroy;
end;

{ TPpuSrcFile }

procedure TPpuSrcFile.WriteDef(Output: TPpuOutput);
begin
  inherited WriteDef(Output);
  Output.WriteStr('Time', FormatDateTime('yyyy"-"mm"-"dd hh":"nn":"ss', FileTime));
end;

constructor TPpuSrcFile.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtFile;
end;

{ TPpuOutput }

procedure TPpuOutput.SetIndent(AValue: integer);
begin
  if FIndent=AValue then Exit;
  FIndent:=AValue;
  if FIndent < 0 then
    FIndent:=0;
  SetLength(FIndStr, FIndent*IndentSize);
  if FIndent > 0 then
    FillChar(FIndStr[1], FIndent*IndentSize, ' ');
end;

procedure TPpuOutput.SetIndentSize(AValue: integer);
begin
  if FIndentSize=AValue then Exit;
  FIndentSize:=AValue;
end;

procedure TPpuOutput.WriteStr(const AName, AValue: string);
begin
end;

procedure TPpuOutput.WriteInt(const AName: string; AValue: Int64; Signed: boolean);
begin
  if Signed then
    WriteStr(AName, IntToStr(AValue))
  else
    WriteStr(AName, IntToStr(QWord(AValue)));
end;

procedure TPpuOutput.WriteFloat(const AName: string; AValue: extended);
var
  s: string;
begin
  Str(AValue, s);
  WriteStr(AName, s);
end;

procedure TPpuOutput.WriteBool(const AName: string; AValue: boolean);
begin
  if AValue then
    WriteStr(AName, '1')
  else
    WriteStr(AName, '0');
end;

procedure TPpuOutput.WriteNull(const AName: string);
begin
  WriteStr(AName, '');
end;

procedure TPpuOutput.WriteArrayStart(const AName: string);
begin
  IncI;
end;

procedure TPpuOutput.WriteArrayEnd(const AName: string);
begin
  DecI;
end;

procedure TPpuOutput.WriteObjectStart(const AName: string; Def: TPpuDef);
begin
  IncI;
  if Def = nil then
    exit;
  if Def.DefType <> dtNone then
    WriteStr('Type', Def.DefTypeName);
  if Def.Name <> '' then
    WriteStr('Name', Def.Name);
end;

procedure TPpuOutput.WriteObjectEnd(const AName: string; Def: TPpuDef);
begin
  DecI;
end;

constructor TPpuOutput.Create(OutFileHandle: THandle);
begin
  FOutFileHandle:=OutFileHandle;
  FIndentSize:=2;
end;

destructor TPpuOutput.Destroy;
begin
  Flush;
  inherited Destroy;
end;

procedure TPpuOutput.Flush;
var
  i, len: integer;
begin
  i:=0;
  while FOutBufPos > 0 do begin
    len:=FileWrite(FOutFileHandle, FOutBuf[i], FOutBufPos);
    if len < 0 then
      raise Exception.CreateFmt('Error writing to file: %s', [ {$if declared(GetLastOSError) } SysErrorMessage(GetLastOSError) {$else} 'I/O error' {$endif} ]);
    Inc(i, len);
    Dec(FOutBufPos, len);
  end;
end;

procedure TPpuOutput.Write(const s: string);
var
  ss: string;
  i, len, len2: integer;
begin
  if not FNoIndent then
    ss:=FIndStr + s
  else
    ss:=s;
  i:=1;
  len:=Length(ss);
  while len > 0 do begin
    len2:=Length(FOutBuf) - FOutBufPos;
    if len2 > 0 then begin
      if len < len2 then
        len2:=len;
      Move(ss[i], FOutBuf[FOutBufPos], len2);
      Inc(FOutBufPos, len2);
    end;
    if FOutBufPos = Length(FOutBuf) then
      Flush;
    Inc(i, len2);
    Dec(len, len2);
  end;
  FNoIndent:=True;
end;

procedure TPpuOutput.WriteLn(const s: string);
begin
  Self.Write(s + LineEnding);
  FNoIndent:=False;
end;

procedure TPpuOutput.IncI;
begin
  Indent:=Indent + 1;
end;

procedure TPpuOutput.DecI;
begin
  Indent:=Indent - 1;
end;

procedure TPpuOutput.Init;
begin
end;

procedure TPpuOutput.Done;
begin
  Flush;
end;

{ TPpuUnitDef }

procedure TPpuUnitDef.WriteDef(Output: TPpuOutput);
var
  i: integer;
begin
  Done;
  with Output do begin
    if Version <> 0 then
      WriteInt('Version', Version);
    if TargetCPU <> '' then
      WriteStr('TargetCPU', TargetCPU);
    if TargetOS <> '' then
      WriteStr('TargetOS', TargetOS);
    if Crc <> 0 then
      WriteStr('CRC', hexStr(Crc, 8));
    if IntfCrc <> 0 then
      WriteStr('InterfaceCRC', hexStr(IntfCrc, 8));
    UsedUnits.WriteDef(Output);
    if Length(RefUnits) > 0 then begin
      WriteArrayStart('Units');
      for i:=0 to High(RefUnits) do
        WriteStr('', RefUnits[i]);
      WriteArrayEnd('Units');
    end;
    SourceFiles.WriteDef(Output);
  end;
  inherited WriteDef(Output);
end;

constructor TPpuUnitDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  DefType:=dtUnit;
  ItemsName:='Interface';
  UsedUnits:=TPpuContainerDef.Create(nil);
  UsedUnits.FParent:=Self;
  UsedUnits.ItemsName:='Uses';
  SourceFiles:=TPpuContainerDef.Create(nil);
  SourceFiles.FParent:=Self;
  SourceFiles.ItemsName:='Files';
  FIndexById:=THashSet.Create(64, True, False);
end;

destructor TPpuUnitDef.Destroy;
begin
  UsedUnits.Free;
  SourceFiles.Free;
  FIndexById.Free;
  inherited Destroy;
end;

function TPpuUnitDef.FindById(AId: integer; FindSym: boolean): TPpuDef;
var
  h: PHashSetItem;
  i: cardinal;
begin
  Result:=nil;
  if AId = -1 then
    exit;
  i:=AId;
  if FindSym then
    i:=i or SymIdBit;
  h:=FIndexById.Find(@i, SizeOf(i));
  if h <> nil then
    Result:=TPpuDef(h^.Data)
  else
    Result:=nil;
end;


{ TPpuContainerDef }

function TPpuContainerDef.GetCount: integer;
begin
  Result:=FItems.Count;
end;

function TPpuContainerDef.GetItem(Index: Integer): TPpuDef;
begin
  Result:=TPpuDef(FItems[Index]);
end;

procedure TPpuContainerDef.SetItem(Index: Integer; AValue: TPpuDef);
begin
  FItems[Index]:=AValue;
end;

procedure TPpuContainerDef.WriteDef(Output: TPpuOutput);
var
  i: integer;
begin
  inherited WriteDef(Output);
  BeforeWriteItems(Output);
  if Count = 0 then
    exit;
  Output.WriteArrayStart(ItemsName);
  for i:=0 to Count - 1 do
    Items[i].Write(Output);
  Output.WriteArrayEnd(ItemsName);
end;

procedure TPpuContainerDef.BeforeWriteItems(Output: TPpuOutput);
begin
end;

procedure TPpuContainerDef.Done;
var
  i: integer;
  d: TPpuDef;
begin
  i:=0;
  while i < Count do begin
    d:=Items[i];
    d.Done;
    if d.Parent = Self then
      Inc(i);
  end;
  inherited Done;
end;

constructor TPpuContainerDef.Create(AParent: TPpuContainerDef);
begin
  inherited Create(AParent);
  FItems:=TList.Create;
  ItemsName:='Contents';
end;

destructor TPpuContainerDef.Destroy;
var
  i: integer;
begin
  for i:=0 to FItems.Count - 1 do
    TObject(FItems[i]).Free;
  FItems.Free;
  inherited Destroy;
end;

function TPpuContainerDef.Add(Def: TPpuDef): integer;
begin
  Result:=FItems.Add(Def);
  Def.FParent:=Self;
end;

{ TPpuDef }

function TPpuDef.GetDefTypeName: string;
begin
  Result:=DefTypeNames[DefType];
end;

function TPpuDef.GetId: cardinal;
begin
  if FId = InvalidId then
    Result:=InvalidId
  else
    Result:=FId and not SymIdBit;
end;

function TPpuDef.GetParentUnit: TPpuUnitDef;
var
  d: TPpuContainerDef;
begin
  if FParentUnit = nil then begin
    d:=Parent;
    while (d <> nil) and (d.DefType <> dtUnit) do
      d:=d.Parent;
    FParentUnit:=TPpuUnitDef(d);
  end;
  Result:=FParentUnit;
end;

procedure TPpuDef.SetId(AValue: cardinal);
var
  h: PHashSetItem;
  u: TPpuUnitDef;
begin
  if FId = AValue then Exit;
  u:=ParentUnit;
  if (FId <> InvalidId) and (u <> nil) then begin
    h:=u.FIndexById.Find(@FId, SizeOf(FId));
    if h <> nil then
      u.FIndexById.Remove(h);
  end;
  FId:=AValue;
  if (FId <> InvalidId) and (u <> nil) then begin;
    h:=u.FIndexById.FindOrAdd(@FId, SizeOf(FId));
    h^.Data:=Self;
  end;
end;

procedure TPpuDef.SetParent(AValue: TPpuContainerDef);
var
  i: cardinal;
begin
  if FParent=AValue then Exit;
  if FParent <> nil then
    raise Exception.Create('Parent can not be modified.');
  AValue.Add(Self);
  if FId <> InvalidId then begin
    i:=FId;
    FId:=InvalidId;
    SetId(i);
  end;
end;

procedure TPpuDef.SetSymId(AId: integer);
begin
  Id:=cardinal(AId) or SymIdBit;
end;

procedure TPpuDef.Done;
var
  symdef: TPpuDef;
begin
  if IsSymId(FId) then
    exit;
  if not Ref.IsNull and Ref.IsCurUnit and (Name = '') then begin
    // If there is no definition name, but there is a symbol ref -
    // get the name from the symbol and move the def to the symbol container
    symdef:=ParentUnit.FindById(Ref.Id, True);
    if symdef <> nil then begin
      Name:=symdef.Name;
      Visibility:=symdef.Visibility;
      Parent.FItems.Remove(Self);
      symdef.Parent.FItems.Add(Self);
      // Hide the symbol, since it is not needed anymore
      symdef.Visibility:=dvHidden;
    end;
  end;
end;

procedure TPpuDef.WriteDef(Output: TPpuOutput);
var
  i: SizeInt;
begin
  with Output do begin
    if FId <> InvalidId then
      if IsSymId(FId) then
        WriteInt('SymId', Id)
      else begin
        WriteInt('Id', Id);
        if not Ref.IsNull then
          WriteInt('SymId', Ref.Id);
      end;
    if FilePos.Line > 0 then begin
      WriteObjectStart('Pos');
      if FilePos.FileIndex > 0 then
        WriteInt('File', FilePos.FileIndex);
      WriteInt('Line', FilePos.Line);
      WriteInt('Col', FilePos.Col);
      WriteObjectEnd('Pos');
    end;
    if Visibility <> dvPublic then
      WriteStr('Visibility', DefVisibilityNames[Visibility]);
    if Length(Attrs) > 0 then begin
      WriteArrayStart('Attributes');
      for i:=0 to High(Attrs) do begin
        WriteObjectStart('');
        Attrs[i].TypeSym.Write(Output, 'TypeSym');
        Attrs[i].TypeConstr.Write(Output, 'TypeConstr');
        WriteInt('ParaCount', Attrs[i].ParaCount, False);
        WriteObjectEnd('');
      end;
      WriteArrayEnd('Attributes');
    end;
  end;
end;

constructor TPpuDef.Create(AParent: TPpuContainerDef);
begin
  FId:=InvalidId;
  Ref:=TPpuRef.Create;
  Visibility:=dvPublic;
  if AParent <> nil then
    AParent.Add(Self);
end;

destructor TPpuDef.Destroy;
begin
  Ref.Free;
  inherited Destroy;
end;

procedure TPpuDef.Write(Output: TPpuOutput; const AttrName: string);
begin
  if not CanWrite then
    exit;
  if Parent <> nil then
    Output.WriteObjectStart(AttrName, Self);
  WriteDef(Output);
  if Parent <> nil then
    Output.WriteObjectEnd(AttrName, Self);
end;

function TPpuDef.CanWrite: boolean;
begin
  Result:=Visibility <> dvHidden;
end;

end.

