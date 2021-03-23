{
    This include file contains the variants
    support for FPC

    This file is part of the Free Pascal run time library.
    Copyright (c) 2001-2005 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$IFDEF fpc}
{$mode objfpc}
{$ENDIF}
{$h+}

{ Using inlining for small system functions/wrappers }
{$inline on}
{$define VARIANTINLINE}

unit Variants;

interface

  uses
    sysutils,sysconst,rtlconsts,typinfo;

type
  EVariantParamNotFoundError = class(EVariantError);
  EVariantInvalidOpError = class(EVariantError);
  EVariantTypeCastError = class(EVariantError);
  EVariantOverflowError = class(EVariantError);
  EVariantInvalidArgError = class(EVariantError);
  EVariantBadVarTypeError = class(EVariantError);
  EVariantBadIndexError = class(EVariantError);
  EVariantArrayLockedError = class(EVariantError);
  EVariantNotAnArrayError = class(EVariantError);
  EVariantArrayCreateError = class(EVariantError);
  EVariantNotImplError = class(EVariantError);
  EVariantOutOfMemoryError = class(EVariantError);
  EVariantUnexpectedError = class(EVariantError);
  EVariantDispatchError = class(EVariantError);
  EVariantRangeCheckError = class(EVariantOverflowError);
  EVariantInvalidNullOpError = class(EVariantInvalidOpError);

  TVariantRelationship = (vrEqual, vrLessThan, vrGreaterThan, vrNotEqual);
  TNullCompareRule = (ncrError, ncrStrict, ncrLoose);
  TBooleanToStringRule = (bsrAsIs, bsrLower, bsrUpper);

Const
  OrdinalVarTypes = [varSmallInt, varInteger, varBoolean, varShortInt,
                     varByte, varWord,varLongWord,varInt64,varQWord];
  FloatVarTypes = [
{$ifndef FPUNONE}
    varSingle, varDouble,
{$endif}
    varCurrency, varDecimal];

{ Variant support procedures and functions }

function VarType(const V: Variant): TVarType; inline;
function VarTypeDeRef(const V: Variant): TVarType; overload;
function VarTypeDeRef(const V: TVarData): TVarType; overload; inline;
function VarAsType(const V: Variant; aVarType: TVarType): Variant;
function VarIsType(const V: Variant; aVarType: TVarType): Boolean; overload; inline;
function VarIsType(const V: Variant; const AVarTypes: array of TVarType): Boolean; overload;
function VarIsByRef(const V: Variant): Boolean; inline;

function VarIsEmpty(const V: Variant): Boolean; inline;
procedure VarCheckEmpty(const V: Variant); inline;
function VarIsNull(const V: Variant): Boolean; inline;
function VarIsClear(const V: Variant): Boolean; inline;

function VarIsCustom(const V: Variant): Boolean; inline;
function VarIsOrdinal(const V: Variant): Boolean; inline;
function VarIsFloat(const V: Variant): Boolean; inline;
function VarIsNumeric(const V: Variant): Boolean; inline;
function VarIsStr(const V: Variant): Boolean;
function VarIsBool(const V: Variant): Boolean; inline;

function VarToStr(const V: Variant): string;
function VarToStrDef(const V: Variant; const ADefault: string): string;
function VarToWideStr(const V: Variant): WideString;
function VarToWideStrDef(const V: Variant; const ADefault: WideString): WideString;
function VarToUnicodeStr(const V: Variant): UnicodeString;
function VarToUnicodeStrDef(const V: Variant; const ADefault: UnicodeString): UnicodeString;

{$ifndef FPUNONE}
function VarToDateTime(const V: Variant): TDateTime;
function VarFromDateTime(const DateTime: TDateTime): Variant;
{$endif}

function VarInRange(const AValue, AMin, AMax: Variant): Boolean;
function VarEnsureRange(const AValue, AMin, AMax: Variant): Variant;

function VarSameValue(const A, B: Variant): Boolean;
function VarCompareValue(const A, B: Variant): TVariantRelationship;

function VarIsEmptyParam(const V: Variant): Boolean; inline;

procedure VarClear(var V: Variant);{$IFDEF VARIANTINLINE}inline;{$ENDIF VARIANTINLINE}
procedure VarClear(var V: OleVariant);{$IFDEF VARIANTINLINE}inline;{$ENDIF VARIANTINLINE}

procedure SetClearVarToEmptyParam(var V: TVarData);

function VarIsError(const V: Variant; out AResult: HRESULT): Boolean;
function VarIsError(const V: Variant): Boolean; inline;
function VarAsError(AResult: HRESULT): Variant;

function VarSupports(const V: Variant; const IID: TGUID; out Intf): Boolean;
function VarSupports(const V: Variant; const IID: TGUID): Boolean;

{ Variant copy support }
procedure VarCopyNoInd(var Dest: Variant; const Source: Variant);

{ Variant array support procedures and functions }

function VarArrayCreate(const Bounds: array of SizeInt; aVarType: TVarType): Variant;
function VarArrayCreate(const Bounds: PVarArrayBoundArray; Dims : SizeInt; aVarType: TVarType): Variant;
function VarArrayOf(const Values: array of Variant): Variant;

function VarArrayAsPSafeArray(const A: Variant): PVarArray;

function VarArrayDimCount(const A: Variant) : LongInt;
function VarArrayLowBound(const A: Variant; Dim : LongInt) : LongInt;
function VarArrayHighBound(const A: Variant; Dim : LongInt) : LongInt;

function VarArrayLock(const A: Variant): Pointer;
procedure VarArrayUnlock(const A: Variant);

function VarArrayRef(const A: Variant): Variant;

function VarIsArray(const A: Variant): Boolean; inline;
function VarIsArray(const A: Variant; AResolveByRef: Boolean): Boolean;

function VarTypeIsValidArrayType(const aVarType: TVarType): Boolean;
function VarTypeIsValidElementType(const aVarType: TVarType): Boolean;

{ Variant <--> Dynamic Arrays }

procedure DynArrayToVariant(var V: Variant; const DynArray: Pointer; TypeInfo: Pointer);
procedure DynArrayFromVariant(var DynArray: Pointer; const V: Variant; TypeInfo: Pointer);

{ Global constants }

function Unassigned: Variant; // Unassigned standard constant
function Null: Variant;       // Null standard constant

var
  EmptyParam: OleVariant;

{ Custom Variant base class }

type
  TVarCompareResult = (crLessThan, crEqual, crGreaterThan);
  TCustomVariantType = class(TObject, IInterface)
  private
    FVarType: TVarType;
  protected
    function QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} IID: TGUID; out Obj): HResult; virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
    function _AddRef: Longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
    function _Release: Longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
    procedure SimplisticClear(var V: TVarData);
    procedure SimplisticCopy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean = False);
    procedure RaiseInvalidOp;
    procedure RaiseCastError;
    procedure RaiseDispError;
    function LeftPromotion(const V: TVarData; const Operation: TVarOp; out RequiredVarType: TVarType): Boolean; virtual;
    function RightPromotion(const V: TVarData; const Operation: TVarOp; out RequiredVarType: TVarType): Boolean; virtual;
    function OlePromotion(const V: TVarData; out RequiredVarType: TVarType): Boolean; virtual;
    procedure DispInvoke(Dest: PVarData; var Source: TVarData; CallDesc: PCallDesc; Params: Pointer); virtual;
    procedure VarDataInit(var Dest: TVarData);
    procedure VarDataClear(var Dest: TVarData);
    procedure VarDataCopy(var Dest: TVarData; const Source: TVarData);
    procedure VarDataCopyNoInd(var Dest: TVarData; const Source: TVarData);
    procedure VarDataCast(var Dest: TVarData; const Source: TVarData);
    procedure VarDataCastTo(var Dest: TVarData; const Source: TVarData; const aVarType: TVarType); overload;
    procedure VarDataCastTo(var Dest: TVarData; const aVarType: TVarType); overload;
    procedure VarDataCastToOleStr(var Dest: TVarData);
    procedure VarDataFromStr(var V: TVarData; const Value: string);
    procedure VarDataFromOleStr(var V: TVarData; const Value: WideString);
    function VarDataToStr(const V: TVarData): string;
    function VarDataIsEmptyParam(const V: TVarData): Boolean;
    function VarDataIsByRef(const V: TVarData): Boolean;
    function VarDataIsArray(const V: TVarData): Boolean;
    function VarDataIsOrdinal(const V: TVarData): Boolean;
    function VarDataIsFloat(const V: TVarData): Boolean;
    function VarDataIsNumeric(const V: TVarData): Boolean;
    function VarDataIsStr(const V: TVarData): Boolean;
  public
    constructor Create; overload;
    constructor Create(RequestedVarType: TVarType); overload;
    destructor Destroy; override;
    function IsClear(const V: TVarData): Boolean; virtual;
    procedure Cast(var Dest: TVarData; const Source: TVarData); virtual;
    procedure CastTo(var Dest: TVarData; const Source: TVarData; const aVarType: TVarType); virtual;
    procedure CastToOle(var Dest: TVarData; const Source: TVarData); virtual;
    procedure Clear(var V: TVarData); virtual; abstract;
    procedure Copy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean); virtual; abstract;
    procedure BinaryOp(var Left: TVarData; const Right: TVarData; const Operation: TVarOp); virtual;
    procedure UnaryOp(var Right: TVarData; const Operation: TVarOp); virtual;
    function CompareOp(const Left, Right: TVarData; const Operation: TVarOp): Boolean; virtual;
    procedure Compare(const Left, Right: TVarData; var Relationship: TVarCompareResult); virtual;
    property VarType: TVarType read FVarType;
  end;
  TCustomVariantTypeClass = class of TCustomVariantType;

  TVarDataArray = array of TVarData;
  IVarInvokeable = interface
    ['{1CB65C52-BBCB-41A6-9E58-7FB916BEEB2D}']
    function DoFunction(var Dest: TVarData; const V: TVarData;
      const Name: string; const Arguments: TVarDataArray): Boolean;
    function DoProcedure(const V: TVarData; const Name: string;
      const Arguments: TVarDataArray): Boolean;
    function GetProperty(var Dest: TVarData; const V: TVarData;
      const Name: string): Boolean;
    function SetProperty(var V: TVarData; const Name: string;
      const Value: TVarData): Boolean;
  end;

  TInvokeableVariantType = class(TCustomVariantType, IVarInvokeable)
  protected
    procedure DispInvoke(Dest: PVarData; var Source: TVarData;
      CallDesc: PCallDesc; Params: Pointer); override;
  public
    { IVarInvokeable }
    function DoFunction(var Dest: TVarData; const V: TVarData;
      const Name: string; const Arguments: TVarDataArray): Boolean; virtual;
    function DoProcedure(const V: TVarData; const Name: string;
      const Arguments: TVarDataArray): Boolean; virtual;
    function GetProperty(var Dest: TVarData; const V: TVarData;
      const Name: string): Boolean; virtual;
    function SetProperty(var V: TVarData; const Name: string;
      const Value: TVarData): Boolean; virtual;
  end;

  IVarInstanceReference = interface
    ['{5C176802-3F89-428D-850E-9F54F50C2293}']
    function GetInstance(const V: TVarData): TObject;
  end;

  TPublishableVariantType = class(TInvokeableVariantType, IVarInstanceReference)
  protected
    { IVarInstanceReference }
    function GetInstance(const V: TVarData): TObject; virtual; abstract;
  public
    function GetProperty(var Dest: TVarData; const V: TVarData;
      const Name: string): Boolean; override;
    function SetProperty(var V: TVarData; const Name: string;
      const Value: TVarData): Boolean; override;
  end;

  function FindCustomVariantType(const aVarType: TVarType;
    out CustomVariantType: TCustomVariantType): Boolean; overload;
  function FindCustomVariantType(const TypeName: string;
    out CustomVariantType: TCustomVariantType): Boolean; overload;

type
  TAnyProc = procedure (var V: TVarData);
  TVarDispProc = procedure (Dest: PVariant; const Source: Variant;
      CallDesc: PCallDesc; Params: Pointer); cdecl;

Const
  CMaxNumberOfCustomVarTypes = $0EFF;
  CMinVarType = $0100;
  CMaxVarType = CMinVarType + CMaxNumberOfCustomVarTypes;
  CIncVarType = $000F;
  CFirstUserType = CMinVarType + CIncVarType;

var
  NullEqualityRule: TNullCompareRule = ncrLoose;
  NullMagnitudeRule: TNullCompareRule = ncrLoose;
  NullStrictConvert: Boolean = true;
  NullAsStringValue: string = '';
  PackVarCreation: Boolean = True;
{$ifndef FPUNONE}
  OleVariantInt64AsDouble: Boolean = False;
{$endif}


  VarDispProc: TVarDispProc;
  ClearAnyProc: TAnyProc;  { Handler clearing a varAny }
  ChangeAnyProc: TAnyProc; { Handler to change any to Variant }
  RefAnyProc: TAnyProc;    { Handler to add a reference to an varAny }
  InvalidCustomVariantType : TCustomVariantType;

procedure VarCastError;
procedure VarCastError(const ASourceType, ADestType: TVarType);
procedure VarCastErrorOle(const ASourceType: TVarType);
procedure VarInvalidOp;
procedure VarInvalidOp(const aLeft, aRight: TVarType; aOpCode: TVarOp);
procedure VarInvalidOp(const aRight: TVarType; aOpCode: TVarOp);
procedure VarInvalidNullOp;
procedure VarBadTypeError;
procedure VarOverflowError;
procedure VarOverflowError(const ASourceType, ADestType: TVarType);
procedure VarBadIndexError;
procedure VarArrayLockedError;
procedure VarNotImplError;
procedure VarOutOfMemoryError;
procedure VarInvalidArgError;
procedure VarInvalidArgError(AType: TVarType);
procedure VarUnexpectedError;
procedure VarRangeCheckError(const AType: TVarType);
procedure VarRangeCheckError(const ASourceType, ADestType: TVarType);
procedure VarArrayCreateError;
procedure VarResultCheck(AResult: HRESULT);{$IFDEF VARIANTINLINE}inline;{$ENDIF VARIANTINLINE}
procedure VarResultCheck(AResult: HRESULT; ASourceType, ADestType: TVarType);
procedure HandleConversionException(const ASourceType, ADestType: TVarType);
function VarTypeAsText(const AType: TVarType): string;
function FindVarData(const V: Variant): PVarData;

const
  VarOpAsText : array[TVarOp] of string = (
    '+',   {opAdd}
    '-',   {opSubtract}
    '*',   {opMultiply}
    '/',   {opDivide}
    'div', {opIntDivide}
    'mod', {opModulus}
    'shl', {opShiftLeft}
    'shr', {opShiftRight}
    'and', {opAnd}
    'or',  {opOr}
    'xor', {opXor}
    '',    {opCompare}
    '-',   {opNegate}
    'not', {opNot}
    '=',   {opCmpEq}
    '<>',  {opCmpNe}
    '<',   {opCmpLt}
    '<=',  {opCmpLe}
    '>',   {opCmpGt}
    '>=',  {opCmpGe}
    '**'   {opPower}
  );

{ Typinfo unit Variant routines have been moved here, so as not to make TypInfo dependent on variants }

Function  GetPropValue(Instance: TObject; PropInfo: PPropInfo; PreferStrings: Boolean): Variant; overload;
Procedure SetPropValue(Instance: TObject; PropInfo: PPropInfo; const Value: Variant); overload;
Function  GetVariantProp(Instance: TObject; PropInfo : PPropInfo): Variant;
Function  GetVariantProp(Instance: TObject; const PropName: string): Variant;
Procedure SetVariantProp(Instance: TObject; const PropName: string; const Value: Variant);
Procedure SetVariantProp(Instance: TObject; PropInfo : PPropInfo; const Value: Variant);


{$IFDEF DEBUG_VARIANTS}
var
  __DEBUG_VARIANTS: Boolean = False;
{$ENDIF}

implementation

uses
  Math,
  VarUtils;

var
  customvarianttypes    : array of TCustomVariantType;
  customvarianttypelock : trtlcriticalsection;
  customvariantcurrtype : LongInt;

const
  { all variants for which vType and varComplexType = 0 do not require
    finalization. }
  varComplexType = $BFE8;

procedure DoVarClearComplex(var v : TVarData); forward;
procedure DoVarCopy(var Dest : TVarData; const Source : TVarData); forward;
procedure DoVarCast(var aDest : TVarData; const aSource : TVarData; aVarType : LongInt); forward;

procedure DoVarClear(var v : TVarData); inline;
begin
  if v.vType and varComplexType <> 0 then
    DoVarClearComplex(v)
  else
    v.vType := varEmpty;
end;

procedure DoVarClearIfComplex(var v : TVarData); inline;
begin
  if v.vType and varComplexType <> 0 then
    DoVarClearComplex(v);
end;

{ ---------------------------------------------------------------------
    String Messages
  ---------------------------------------------------------------------}

ResourceString
  SErrVarIsEmpty = 'Variant is empty';
  SErrInvalidIntegerRange = 'Invalid Integer range: %d';

{ ---------------------------------------------------------------------
    Auxiliary routines
  ---------------------------------------------------------------------}

Procedure VariantError (Const Msg : String); inline;
begin
  Raise EVariantError.Create(Msg);
end;

Procedure NotSupported(Meth: String);
begin
  Raise EVariantError.CreateFmt('Method %s not yet supported.',[Meth]);
end;

type
  TVariantArrayIterator = object
    Bounds : PVarArrayBoundArray;
    Coords : PVarArrayCoorArray;
    Dims   : SizeInt;
    constructor Init(aDims: SizeInt; aBounds : PVarArrayBoundArray);
    destructor Done;

    function Next : Boolean;
    { returns true if the iterator reached the end of the variant array }
    function AtEnd: Boolean;
  end;

{$push}
{$r-}

constructor TVariantArrayIterator.Init(aDims: SizeInt; aBounds : PVarArrayBoundArray);
var
  i : sizeint;
begin
  Dims := aDims;
  Bounds := aBounds;

  GetMem(Coords, SizeOf(SizeInt) * Dims);
  { initialize coordinate counter }
  for i:= 0 to Pred(Dims) do
    Coords^[i] := Bounds^[i].LowBound;
end;


function TVariantArrayIterator.Next: Boolean;
var
  Finished : Boolean;

  procedure IncDim(Dim : SizeInt);
  begin
    if Finished then
      Exit;

    Inc(Coords^[Dim]);
    if Coords^[Dim] >= Bounds^[Dim].LowBound + Bounds^[Dim].ElementCount then begin
      Coords^[Dim]:=Bounds^[Dim].LowBound;
      if Dim > 0 then
        IncDim(Pred(Dim))
      else
        Finished := True;
    end;
  end;


begin
  Finished := False;
  IncDim(Pred(Dims));
  Result := not Finished;
end;


function TVariantArrayIterator.AtEnd: Boolean;
var
  i,l : sizeint;
begin
  result:=false;
  l:=Pred(dims);
  I:=0;
  While (not Result) and (I<=L) do
    begin
    Result:=Coords^[i] >= (Bounds^[i].LowBound + Bounds^[i].ElementCount);
    inc(i);
    end;
end;

{$pop}// {$r-} for TVariantArrayIterator

destructor TVariantArrayIterator.done;
  begin
    FreeMem(Coords);
  end;


type
  tdynarraybounds = array of SizeInt;
  tdynarraycoords = tdynarraybounds;
  tdynarrayelesize = tdynarraybounds;
  tdynarraypositions = array of Pointer;
  tdynarrayiter = object
    Bounds : tdynarraybounds;
    Coords : tdynarraycoords;
    elesize : tdynarrayelesize;
    positions : tdynarraypositions;
    Dims : SizeInt;
    data : Pointer;
    constructor init(d : Pointer;typeInfo : Pointer;_dims: SizeInt;b : tdynarraybounds);
    function next : Boolean;
    destructor done;
  end;


constructor tdynarrayiter.init(d : Pointer;typeInfo : Pointer;_dims: SizeInt;b : tdynarraybounds);
  var
    i : sizeint;
  begin
    Bounds:=b;
    Dims:=_dims;
    SetLength(Coords,Dims);
    SetLength(elesize,Dims);
    SetLength(positions,Dims);
    positions[0]:=d;
    { initialize coordinate counter and elesize }
    for i:=0 to Dims-1 do
      begin
        Coords[i]:=0;
        if i>0 then
          positions[i]:=Pointer(positions[i-1]^);
        { skip kind and name }
        typeInfo:=AlignTypeData(typeInfo+2+Length(PTypeInfo(typeInfo)^.Name));

        elesize[i]:=PTypeData(typeInfo)^.elSize;
        typeInfo:=PTypeData(typeInfo)^.elType2;
      end;
    data:=positions[Dims-1];
  end;


function tdynarrayiter.next : Boolean;
  var
    Finished : Boolean;

  procedure incdim(d : SizeInt);
    begin
      if Finished then
        exit;
      inc(Coords[d]);
      inc(Pointer(positions[d]),elesize[d]);

      if Coords[d]>=Bounds[d] then
        begin
          Coords[d]:=0;
          if d>0 then
            begin
              incdim(d-1);
              positions[d]:=Pointer(positions[d-1]^);
            end
          else
            Finished:=true;
        end;
    end;

  begin
    Finished:=False;
    incdim(Dims-1);
    data:=positions[Dims-1];
    Result:=not(Finished);
  end;


destructor tdynarrayiter.done;
  begin
    Bounds:=nil;
    Coords:=nil;
    elesize:=nil;
    positions:=nil;
  end;

{ ---------------------------------------------------------------------
    VariantManager support
  ---------------------------------------------------------------------}

procedure sysvarinit(var v : Variant);
begin
  TVarData(V).vType := varEmpty;
end;


procedure sysvarclear(var v : Variant);
begin
  if TVarData(v).vType and varComplexType <> 0 then
    VarClearProc(TVarData(V))
  else
    TVarData(v).vType := varEmpty;
end;


function Sysvartoint (const v : Variant) : Longint;
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varInt64)
    else
      Result := 0
  else
    Result := VariantToLongInt(TVarData(V));
end;

function Sysvartoint64 (const v : Variant) : Int64;
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varInt64)
    else
      Result := 0
  else
    Result := VariantToInt64(TVarData(V));
end;


function sysvartoword64 (const v : Variant) : QWord;
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varQWord)
    else
      Result := 0
  else
    Result := VariantToQWord (TVarData(V));
end;


function sysvartobool (const v : Variant) : Boolean;
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varBoolean)
    else
      Result := False
  else
    Result := VariantToBoolean(TVarData(V));
end;


{$ifndef FPUNONE}
function sysvartoreal (const v : Variant) : Extended;
var Handler: TCustomVariantType;
    dest: TVarData;
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varDouble)
    else
      Result := 0
  { TODO: performance: custom variants must be handled after standard ones }
  else if FindCustomVariantType(TVarData(v).vType, Handler) then
  begin
    VariantInit(dest);
    Handler.CastTo(dest, TVarData(v), varDouble);
    Result := dest.vDouble;
  end
  else
    Result := VariantToDouble(TVarData(V));
end;
{$endif}


function sysvartocurr (const v : Variant) : Currency;
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varCurrency)
    else
      Result := 0
  else
    Result := VariantToCurrency(TVarData(V));
end;

function CustomVarToLStr(const v: TVarData; out s: AnsiString): Boolean;
var
  handler: TCustomVariantType;
  temp: TVarData;
begin
  result := FindCustomVariantType(v.vType, handler);
  if result then
  begin
    VariantInit(temp);
    handler.CastTo(temp, v, varString);
    { out-semantic ensures that s is finalized,
      so just copy the pointer and don't finalize the temp }
    Pointer(s) := temp.vString;
  end;
end;

procedure sysvartolstr (var s : AnsiString; const v : Variant);
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varString)
    else
      s := NullAsStringValue
  { TODO: performance: custom variants must be handled after standard ones }
  else if not CustomVarToLStr(TVarData(v), s) then
    S := VariantToAnsiString(TVarData(V));
end;


procedure sysvartopstr (var s; const v : Variant);
var
  tmp: AnsiString;
begin
  sysvartolstr(tmp, v);
  ShortString(s) := tmp;
end;


procedure sysvartowstr (var s : WideString; const v : Variant);
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varOleStr)
    else
      s := NullAsStringValue
  else
    S := VariantToWideString(TVarData(V));
end;


procedure sysvartointf (var Intf : IInterface; const v : Variant);
begin
  case TVarData(v).vType of
    varEmpty:
      Intf := nil;
    varNull:
      if NullStrictConvert then
        VarCastError(varNull, varUnknown)
      else
        Intf := nil;
    varUnknown:
      Intf := IInterface(TVarData(v).vUnknown);
    varUnknown or varByRef:
      Intf := IInterface(TVarData(v).vPointer^);
    varDispatch:
      Intf := IInterface(TVarData(v).vDispatch);
    varDispatch or varByRef:
      Intf := IInterface(TVarData(v).vPointer^);
    varVariant, varVariant or varByRef: begin
      if not Assigned(TVarData(v).vPointer) then
        VarBadTypeError;
      sysvartointf(Intf, Variant(PVarData(TVarData(v).vPointer)^) );
    end;
  else
    VarCastError(TVarData(v).vType, varUnknown);
  end;
end;


procedure sysvartodisp (var Disp : IDispatch; const v : Variant);
begin
  case TVarData(v).vType of
    varEmpty:
      Disp := nil;
    varNull:
      if NullStrictConvert then
        VarCastError(varNull, varDispatch)
      else
        Disp := nil;
    varUnknown:
      if IInterface(TVarData(v).vUnknown).QueryInterface(IDispatch, Disp) <> S_OK then
        VarCastError(varUnknown, varDispatch);
    varUnknown or varByRef:
      if IInterface(TVarData(v).vPointer^).QueryInterface(IDispatch, Disp) <> S_OK then
        VarCastError(varUnknown or varByRef, varDispatch);
    varDispatch:
      Disp := IDispatch(TVarData(v).vDispatch);
    varDispatch or varByRef:
      Disp := IDispatch(TVarData(v).vPointer^);
    varVariant, varVariant or varByRef: begin
      if not Assigned(TVarData(v).vPointer) then
        VarBadTypeError;
      sysvartodisp(Disp, Variant(PVarData(TVarData(v).vPointer)^) );
    end;
  else
    VarCastError(TVarData(v).vType, varDispatch);
  end;
end;

{$ifndef FPUNONE}
function sysvartotdatetime (const v : Variant) : TDateTime;
begin
  if VarType(v) = varNull then
    if NullStrictConvert then
      VarCastError(varNull, varDate)
    else
      Result := 0
  else
    Result:=VariantToDate(TVarData(v));
end;
{$endif}

function DynamicArrayIsRectangular(p : Pointer;TypeInfo : Pointer) : Boolean;
var
  arraysize,i : sizeint;
begin
  Result := False;

  { get TypeInfo of second level }
  { skip kind and name }
  TypeInfo:=AlignTypeData(TypeInfo+2+Length(PTypeInfo(TypeInfo)^.Name));
  TypeInfo:=PTypeData(TypeInfo)^.elType2;

  { check recursively? }
  if assigned(TypeInfo) and (PTypeInfo(TypeInfo)^.kind=tkDynArray) then
    begin
      { set to dimension of first element }
      arraysize:=psizeint(ppointer(p)^-SizeOf(sizeint))^;
      { walk through all elements }
      for i:=1 to psizeint(p-SizeOf(sizeint))^ do
        begin
          { ... and check dimension }
          if psizeint(ppointer(p)^-SizeOf(sizeint))^<>arraysize then
            exit;
          if not(DynamicArrayIsRectangular(ppointer(p)^,TypeInfo)) then
            exit;
          inc(p,SizeOf(Pointer));
        end;
    end;
    Result:=true;
end;

procedure sysvartodynarray (var dynarr : Pointer; const v : Variant; TypeInfo : Pointer);
begin
  DynArrayFromVariant(dynarr, v, TypeInfo);
end;

procedure sysvarfrombool (var Dest : Variant; const Source : Boolean);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varBoolean;
    vBoolean := Source;
  end;
end;

procedure VariantErrorInvalidIntegerRange(Range: LongInt);
begin
  VariantError(Format(SErrInvalidIntegerRange,[Range]));
end;

procedure sysvarfromint (var Dest : Variant; const Source, Range : LongInt);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do
    if PackVarCreation then
      case Range of
        -4 : begin
          vType := varInteger;
          vInteger := Source;
        end;
        -2 : begin
          vType := varSmallInt;
          vSmallInt := Source;
        end;
        -1 : Begin
          vType := varShortInt;
          vshortint := Source;
        end;
        1 : begin
          vType := varByte;
          vByte := Source;
        end;
        2 : begin
          vType := varWord;
          vWord := Source;
        end;
        4 : Begin
          vType := varLongWord;
          {use vInteger, not vLongWord as the value came passed in as an Integer }
          vInteger := Source;
        end;
      else
        VariantErrorInvalidIntegerRange(Range);
      end
    else begin
      vType := varInteger;
      vInteger := Source;
    end;
end;

procedure sysvarfromint64 (var Dest : Variant; const Source : Int64);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varInt64;
    vInt64 := Source;
  end;
end;

procedure sysvarfromword64 (var Dest : Variant; const Source : QWord);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varQWord;
    vQWord := Source;
  end;
end;

{$ifndef FPUNONE}
procedure sysvarfromreal (var Dest : Variant; const Source : Extended);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varDouble;
    vDouble := Source;
  end;
end;

procedure sysvarfromsingle (var Dest : Variant; const Source : single);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varSingle;
    vSingle := Source;
  end;
end;

procedure sysvarfromdouble (var Dest : Variant; const Source : double);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varDouble;
    vDouble := Source;
  end;
end;
{$endif}

procedure sysvarfromcurr (var Dest : Variant; const Source : Currency);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varCurrency;
    vCurrency := Source;
  end;
end;


{$ifndef FPUNONE}
procedure sysvarfromtdatetime (var Dest : Variant; const Source : TDateTime);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varDate;
    vDate := Source;
  end;
end;
{$endif}


procedure sysvarfrompstr (var Dest : Variant; const Source : ShortString);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varString;
    vString := nil;
    AnsiString(vString) := Source;
  end;
end;

procedure sysvarfromlstr (var Dest : Variant; const Source : AnsiString);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varString;
    vString := nil;
    AnsiString(vString) := Source;
  end;
end;


procedure sysvarfromwstr (var Dest : Variant; const Source : WideString);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vType := varOleStr;
    vOleStr := nil;
    WideString(Pointer(vOleStr)) := Source;
  end;
end;

procedure sysvarfromintf(var Dest : Variant; const Source : IInterface);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vUnknown := nil;
    IInterface(vUnknown) := Source;
    vType := varUnknown;
  end;
end;


procedure sysvarfromdisp(var Dest : Variant; const Source : IDispatch);
begin
  DoVarClearIfComplex(TVarData(Dest));
  with TVarData(Dest) do begin
    vUnknown := nil;
    IDispatch(vDispatch) := Source;
    vType := varDispatch;
  end;
end;

type
  TCommonType = (ctEmpty,ctAny,ctError,ctLongInt,ctBoolean,
{$ifndef FPUNONE}
    ctFloat,ctDate,ctCurrency,
{$endif}
    ctInt64,ctNull,ctWideStr,ctString);

  TCommonVarType = varEmpty..varQWord;

const
{$ifdef FPUNONE}
  ctFloat = ctError;
  ctDate = ctError;
  ctCurrency = ctError;
{$endif}

  { get the basic type for a Variant type }
  VarTypeToCommonType : array[TCommonVarType] of TCommonType =
    (ctEmpty,           // varEmpty = 0;
     ctNull,            // varNull = 1;
     ctLongInt,         // varSmallInt = 2;
     ctLongInt,         // varInteger = 3;
     ctFloat,           // varSingle = 4;
     ctFloat,           // varDouble = 5;
     ctCurrency,        // varCurrency = 6;
     ctDate,            // varDate = 7;
     ctWideStr,         // varOleStr = 8;
     ctError,           // varDispatch = 9;
     ctError,           // varError = 10;
     ctBoolean,         // varBoolean = 11;
     ctError,           // varVariant = 12;
     ctError,           // varUnknown = 13;
     ctError,           // ??? 15
     ctError,           // varDecimal = 14;
     ctLongInt,         // varShortInt = 16;
     ctLongInt,         // varByte = 17;
     ctLongInt,         // varWord = 18;
     ctInt64,           // varLongWord = 19;
     ctInt64,           // varInt64 = 20;
     ctInt64            // varQWord = 21;
    );

  { map a basic type back to a Variant type }
{ Not used yet
  CommonTypeToVarType : array[TCommonType] of TVarType =
    (
      varEmpty,
      varany,
      varError,
      varInteger,
      varDouble,
      varBoolean,
      varInt64,
      varNull,
      varOleStr,
      varDate,
      varCurrency,
      varString
    );
}
function MapToCommonType(const vType : TVarType) : TCommonType;
begin
  case vType of
    Low(TCommonVarType)..High(TCommonVarType):
      Result := VarTypeToCommonType[vType];
    varString:
      Result:=ctString;
    varAny:
      Result:=ctAny;
  else
    Result:=ctError;
  end;
end;

const
  FindCmpCommonType : array[TCommonType, TCommonType] of TCommonType = (
     {              ctEmpty    ctAny    ctError  ctLongInt   ctBoolean                         ctFloat    ctDate   ctCurrency           ctInt64     ctNull   ctWideStr   ctString  }
    ({ ctEmpty }    ctEmpty,   ctEmpty, ctError, ctEmpty,    ctEmpty,    {$ifndef FPUNONE}ctEmpty,   ctEmpty, ctEmpty,    {$endif}ctEmpty,    ctEmpty, ctEmpty,    ctEmpty   ),
    ({ ctAny }      ctEmpty,   ctAny,   ctError, ctAny,      ctAny,      {$ifndef FPUNONE}ctAny,     ctAny,   ctAny,      {$endif}ctAny,      ctAny,   ctAny,      ctAny      ),
    ({ ctError }    ctError,   ctError, ctError, ctError,    ctError,    {$ifndef FPUNONE}ctError,   ctError, ctError,    {$endif}ctError,    ctError, ctError,    ctError    ),
    ({ ctLongInt }  ctEmpty,   ctAny,   ctError, ctLongInt,  ctBoolean,  {$ifndef FPUNONE}ctFloat,   ctDate,  ctCurrency, {$endif}ctInt64,    ctNull,  ctFloat,    ctFloat    ),
    ({ ctBoolean }  ctEmpty,   ctAny,   ctError, ctLongInt,  ctBoolean,  {$ifndef FPUNONE}ctFloat,   ctDate,  ctCurrency, {$endif}ctInt64,    ctNull,  ctWideStr,  ctString   ),
{$ifndef FPUNONE}
    ({ ctFloat }    ctEmpty,   ctAny,   ctError, ctFloat,    ctFloat,    ctFloat,   ctDate,  ctCurrency, ctFloat,    ctNull,  ctFloat,    ctFloat    ),
    ({ ctDate }     ctEmpty,   ctAny,   ctError, ctDate,     ctDate,     ctDate,    ctDate,  ctDate,     ctDate,     ctNull,  ctDate,     ctDate     ),
    ({ ctCurrency } ctEmpty,   ctAny,   ctError, ctCurrency, ctCurrency, ctCurrency,ctDate,  ctCurrency, ctCurrency, ctNull,  ctCurrency, ctCurrency ),
{$endif}
    ({ ctInt64 }    ctEmpty,   ctAny,   ctError, ctInt64,    ctInt64,    {$ifndef FPUNONE}ctFloat,   ctDate,  ctCurrency, {$endif}ctInt64,    ctNull,  ctFloat,    ctFloat    ),
    ({ ctNull }     ctEmpty,   ctAny,   ctError, ctNull,     ctNull,     {$ifndef FPUNONE}ctNull,    ctNull,  ctNull,     {$endif}ctNull,     ctNull,  ctNull,     ctNull     ),
    ({ ctWideStr }  ctEmpty,   ctAny,   ctError, ctFloat,    ctWideStr,  {$ifndef FPUNONE}ctFloat,   ctDate,  ctCurrency, {$endif}ctFloat,    ctNull,  ctWideStr,  ctWideStr  ),
    ({ ctString }   ctEmpty,   ctAny,   ctError, ctFloat,    ctString,   {$ifndef FPUNONE}ctFloat,   ctDate,  ctCurrency, {$endif}ctFloat,    ctNull,  ctWideStr,  ctString   )
    );

function DoVarCmpSimple (const Left, Right, Common: TCommonType) : ShortInt; inline;
begin
  if Left = Common then
    if Right = Common then
      Result := 0
    else
      Result := -1
  else
    Result := 1;
end;

function DoVarCmpAny(const Left, Right: TVarData; const OpCode: TVarOp) : ShortInt;
begin
  VarInvalidOp(Left.vType, Right.vType, OpCode);
  Result:=0;
end;

function DoVarCmpLongInt(const Left, Right: LongInt): ShortInt; inline;
begin
  if Left < Right then
    Result := -1
  else if Left > Right then
    Result := 1
  else
    Result := 0;
end;

{$ifndef FPUNONE}
function DoVarCmpFloat(const Left, Right: Double; const OpCode: TVarOp): ShortInt;
begin
  if Left = Right then
    Result := 0
  else if (OpCode in [opCmpEq, opCmpNe]) or (Left < Right) then
    Result := -1
  else
    Result := 1;
end;
{$endif}

function DoVarCmpInt64(const Left, Right: Int64): ShortInt;
begin
  if Left < Right then
    Result := -1
  else if Left > Right then
    Result := 1
  else
    Result := 0;
end;

function DoVarCmpNull(const Left, Right: TCommonType; const OpCode: TVarOp) : ShortInt;
const
  ResultMap: array [Boolean, opCmpEq..opCmpGe] of ShortInt =
    ( ( -1, 0, 0, 1, 0, -1 ), ( 0, -1, -1, -1, 1, 1 ) );
begin
  if OpCode in [opCmpEq, opCmpNe] then
    case NullEqualityRule of
      ncrError:  VarInvalidNullOp;
      ncrStrict: Result := ResultMap[False, OpCode];
      ncrLoose:  Result := ResultMap[(Left = Right) xor (OpCode = opCmpNe), OpCode];
    end
  else
    case NullMagnitudeRule of
      ncrError:  VarInvalidNullOp;
      ncrStrict: Result := ResultMap[False, OpCode];
      ncrLoose:  Result := DoVarCmpSimple(Left, Right, ctNull);
    end;
end;

function DoVarCmpCurr(const Left, Right: Currency): ShortInt;
begin
  if Left < Right then
    Result := -1
  else if Left > Right then
    Result := 1
  else
    Result := 0;
end;

function DoVarCmpWStrDirect(const Left, Right: Pointer; const OpCode: TVarOp): ShortInt; inline;
begin
  { we can do this without ever copying the string }
  if OpCode in [opCmpEq, opCmpNe] then
    if Length(WideString(Left)) <> Length(WideString(Right)) then
      Exit(-1);
  Result := sign(WideCompareStr(
    WideString(Left),
    WideString(Right)
  ));
end;


function DoVarCmpWStr(const Left, Right: TVarData; const OpCode: TVarOp): ShortInt;
begin
  { keep the temps away from the main proc }
  Result := DoVarCmpWStrDirect(Pointer(VariantToWideString(Left)),
    Pointer(VariantToWideString(Right)), OpCode);
end;


function DoVarCmpLStrDirect(const Left, Right: Pointer; const OpCode: TVarOp): ShortInt; inline;
begin
  { we can do this without ever copying the string }
  if OpCode in [opCmpEq, opCmpNe] then
    if Length(AnsiString(Left)) <> Length(AnsiString(Right)) then
      Exit(-1);
  Result := sign(CompareStr(
    AnsiString(Left),
    AnsiString(Right)
  ));
end;


function DoVarCmpLStr(const Left, Right: TVarData; const OpCode: TVarOp): ShortInt;
begin
  { keep the temps away from the main proc }
  Result := DoVarCmpLStrDirect(Pointer(VariantToAnsiString(Left)),
    Pointer(VariantToAnsiString(Right)), OpCode);
end;

function DoVarCmpComplex(const Left, Right: TVarData; const OpCode: TVarOp): ShortInt;
var Handler: TCustomVariantType;
    CmpRes: boolean;
begin
  if (Left.vType=varnull) or (Right.vType=varnull) then
    // don't bother custom variant handlers with conversion to NULL
    begin
    if OpCode in [opCmpEq,opCmpNe] then
      begin
      if (Left.vType=Right.vType) xor (OpCode=opCmpNe) then
        result:=0
      else
        result:=-1;
      end
    else
      if Left.vType=varnull then
        begin
        if Right.vType=varnull then
          Result := 0
        else
          Result := -1;
        end
      else
        Result := 1;
    end
  else
    begin
    if FindCustomVariantType(Left.vType, Handler) then
      CmpRes := Handler.CompareOp(Left, Right, OpCode)
    else if FindCustomVariantType(Right.vType, Handler) then
      CmpRes := Handler.CompareOp(Left, Right, OpCode)
    else
    VarInvalidOp(Left.vType, Right.vType, OpCode);

    case OpCode of
      opCmpEq:
        if CmpRes then
          Result:=0
        else
          Result:=1;
      opCmpNe:
        if CmpRes then
          Result:=1
        else
          Result:=0;
      opCmpLt,
      opCmpLe:
        if CmpRes then
          Result:=-1
        else
          Result:=1;
      opCmpGt,
      opCmpGe:
        if CmpRes then
          Result:=1
        else
          Result:=-1;
    end;
    end;
end;


function DoVarCmp(const vl, vr : TVarData; const OpCode : TVarOp) : ShortInt;
var
  lct: TCommonType;
  rct: TCommonType;
begin
  { as the function in cvarutil.inc can handle varByRef correctly we simply
    resolve the final type }
  lct := MapToCommonType(VarTypeDeRef(vl));
  rct := MapToCommonType(VarTypeDeRef(vr));

  {$IFDEF DEBUG_VARIANTS}
  if __DEBUG_VARIANTS then begin
    WriteLn('DoVarCmp $', IntToHex(Cardinal(@vl),8), ' ', GetEnumName(TypeInfo(TVarOp), Ord(OpCode)) ,' $', IntToHex(Cardinal(@vr),8));
    DumpVariant('DoVarCmp/vl', vl);
    WriteLn('lct ', GetEnumName(TypeInfo(TCommonType), Ord(lct)));

    DumpVariant('DoVarCmp/vr', vr);
    WriteLn('rct ', GetEnumName(TypeInfo(TCommonType), Ord(rct)));

    WriteLn('common ', GetEnumName(TypeInfo(TCommonType), Ord(FindCmpCommonType[lct, rct])));
  end;
  {$ENDIF}

  case FindCmpCommonType[lct, rct] of
    ctEmpty:    Result := DoVarCmpSimple(lct, rct, ctEmpty);
    ctAny:      Result := DoVarCmpAny(vl, vr, OpCode);
    ctLongInt:  Result := DoVarCmpLongInt(VariantToLongInt(vl), VariantToLongInt(vr));
{$ifndef FPUNONE}
    ctFloat:    Result := DoVarCmpFloat(VariantToDouble(vl), VariantToDouble(vr), OpCode);
{$endif}
    ctBoolean:  Result := DoVarCmpLongInt(LongInt(VariantToBoolean(vl)), LongInt(VariantToBoolean(vr)));
    ctInt64:    Result := DoVarCmpInt64(VariantToInt64(vl), VariantToInt64(vr));
    ctNull:     Result := DoVarCmpNull(lct, rct, OpCode);
    ctWideStr:
      if (vl.vType = varOleStr) and (vr.vType = varOleStr) then
        Result := DoVarCmpWStrDirect(Pointer(vl.vOleStr), Pointer(vr.vOleStr), OpCode)
      else
        Result := DoVarCmpWStr(vl, vr, OpCode);
{$ifndef FPUNONE}
    ctDate:     Result := DoVarCmpFloat(VariantToDate(vl), VariantToDate(vr), OpCode);
    ctCurrency: Result := DoVarCmpCurr(VariantToCurrency(vl), VariantToCurrency(vr));
{$endif}
    ctString:
      if (vl.vType = varString) and (vr.vType = varString) then
        Result := DoVarCmpLStrDirect(Pointer(vl.vString), Pointer(vr.vString), OpCode)
      else
        Result := DoVarCmpLStr(vl, vr, OpCode);
  else
    Result := DoVarCmpComplex(vl, vr, OpCode);
  end;
end;

function syscmpop (const Left, Right : Variant; const OpCode : TVarOp) : Boolean;
var
  CmpRes : ShortInt;
begin
  CmpRes:=DoVarCmp(TVarData(Left),TVarData(Right),OpCode);
  case OpCode of
    opCmpEq:
      Result:=CmpRes=0;
    opCmpNe:
      Result:=CmpRes<>0;
    opCmpLt:
      Result:=CmpRes<0;
    opCmpLe:
      Result:=CmpRes<=0;
    opCmpGt:
      Result:=CmpRes>0;
    opCmpGe:
      Result:=CmpRes>=0;
   else
     VarInvalidOp;
  end;
end;


const
  FindOpCommonType : array[TCommonType,TCommonType] of TCommonType = (
     {              ctEmpty  ctAny    ctError  ctLongInt   ctBoolean   ctFloat     ctDate   ctCurrency  ctInt64     ctNull    ctWideStr   ctString  }
    ({ ctEmpty }    ctEmpty, ctAny,   ctError, ctEmpty,    ctEmpty,    {$ifndef FPUNONE}ctEmpty,    ctEmpty, ctEmpty,    {$endif}ctEmpty,    ctEmpty,  ctEmpty,    ctEmpty    ),
    ({ ctAny }      ctAny,   ctAny,   ctError, ctAny,      ctAny,      {$ifndef FPUNONE}ctAny,      ctAny,   ctAny,      {$endif}ctAny,      ctAny,    ctAny,      ctAny      ),
    ({ ctError }    ctError, ctError, ctError, ctError,    ctError,    {$ifndef FPUNONE}ctError,    ctError, ctError,    {$endif}ctError,    ctError,  ctError,    ctError    ),
    ({ ctLongInt }  ctEmpty, ctAny,   ctError, ctLongInt,  ctBoolean,  {$ifndef FPUNONE}ctFloat,    ctDate,  ctCurrency, {$endif}ctInt64,    ctNull,   ctFloat,    ctFloat    ),
    ({ ctBoolean }  ctEmpty, ctAny,   ctError, ctLongInt,  ctBoolean,  {$ifndef FPUNONE}ctFloat,    ctDate,  ctCurrency, {$endif}ctInt64,    ctNull,   ctBoolean,  ctBoolean  ),
{$ifndef FPUNONE}
    ({ ctFloat }    ctEmpty, ctAny,   ctError, ctFloat,    ctFloat,    ctFloat,    ctDate,  ctCurrency, ctFloat,    ctNull,   ctFloat,    ctFloat    ),
    ({ ctDate }     ctEmpty, ctAny,   ctError, ctDate,     ctDate,     ctDate,     ctDate,  ctDate,     ctDate,     ctNull,   ctDate,     ctDate     ),
    ({ ctCurrency } ctEmpty, ctAny,   ctError, ctCurrency, ctCurrency, ctCurrency, ctDate,  ctCurrency, ctCurrency, ctNull,   ctCurrency, ctCurrency ),
{$endif}
    ({ ctInt64 }    ctEmpty, ctAny,   ctError, ctInt64,    ctInt64,    {$ifndef FPUNONE}ctFloat,    ctDate,  ctCurrency, {$endif}ctInt64,    ctNull,   ctFloat,    ctFloat    ),
    ({ ctNull }     ctEmpty, ctAny,   ctError, ctNull,     ctNull,     {$ifndef FPUNONE}ctNull,     ctNull,  ctNull,     {$endif}ctNull,     ctNull,   ctNull,     ctNull     ),
    ({ ctWideStr }  ctEmpty, ctAny,   ctError, ctFloat,    ctBoolean,  {$ifndef FPUNONE}ctFloat,    ctDate,  ctCurrency, {$endif}ctFloat,    ctNull,   ctWideStr,  ctWideStr  ),
    ({ ctString }   ctEmpty, ctAny,   ctError, ctFloat,    ctBoolean,  {$ifndef FPUNONE}ctFloat,    ctDate,  ctCurrency, {$endif}ctFloat,    ctNull,   ctWideStr,  ctString   )
    );

procedure DoVarOpFloat(var vl :TVarData; const vr : TVarData; const OpCode : TVarOp);
{$ifndef FPUNONE}
var
  l, r : Double;
begin
  l := VariantToDouble(vl);
  r := VariantToDouble(vr);
  case OpCode of
    opAdd      :  l := l  + r;
    opSubtract :  l := l  - r;
    opMultiply :  l := l  * r;
    opDivide   :  l := l  / r;
    opPower    :  l := l ** r;
  else
    VarInvalidOp(vl.vType, vr.vType, OpCode);
  end;
  DoVarClearIfComplex(vl);
  vl.vType := varDouble;
  vl.vDouble := l;
{$else}
begin
   VarInvalidOp(vl.vType, vr.vType, OpCode);
{$endif}
end;

procedure DoVarOpAny(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp);
begin
  VarInvalidOp(vl.vType, vr.vType, OpCode);
end;

procedure DoVarOpLongInt(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp);
var
  l, r: LongInt;
begin
  l := VariantToLongint(vl);
  r := VariantToLongint(vr);
  case OpCode of
    opIntDivide  : l := l div r;
    opModulus    : l := l mod r;
    opShiftLeft  : l := l shl r;
    opShiftRight : l := l shr r;
    opAnd        : l := l and r;
    opOr         : l := l  or r;
    opXor        : l := l xor r;
  else
    VarInvalidOp(vl.vType, vr.vType, OpCode);
  end;
  DoVarClearIfComplex(vl);
  vl.vType := varInteger;
  vl.vInteger := l;
end;

procedure DoVarOpInt64(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp);
var
  l, r     : Int64;
  Overflow : Boolean;
begin
  l := VariantToInt64(vl);
  r := VariantToInt64(vr);
  Overflow := False;
  case OpCode of
{$push}
{$R+}{$Q+}
    opAdd..opMultiply,opPower: try
      case OpCode of
        opAdd      :  l := l  + r;
        opSubtract :  l := l  - r;
        opMultiply :  l := l  * r;
{$ifndef FPUNONE}
        opPower    :  l := l ** r;
{$endif}
      end;
    except
      on E: SysUtils.ERangeError do
        Overflow := True;
      on E: SysUtils.EIntOverflow do
        Overflow := True;
    end;
{$pop}
    opIntDivide  : l := l div r;
    opModulus    : l := l mod r;
    opShiftLeft  : l := l shl r;
    opShiftRight : l := l shr r;
    opAnd        : l := l and r;
    opOr         : l := l  or r;
    opXor        : l := l xor r;
  else
    VarInvalidOp(vl.vType, vr.vType, OpCode);
  end;
  if Overflow then
    DoVarOpFloat(vl,vr,OpCode)
  else begin
    DoVarClearIfComplex(vl);
    vl.vType := varInt64;
    vl.vInt64 := l;
  end;
end;

procedure DoVarOpInt64to32(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp);
begin
  { can't do this well without an efficent way to check for overflows,
    let the Int64 version handle it and check the Result if we can downgrade it
    to integer }
  DoVarOpInt64(vl, vr, OpCode);
  with vl do
    if (vType = varInt64) and (vInt64 >= Low(LongInt)) and (vInt64 <= High(LongInt)) then begin
      vInteger := vInt64;
      vType := varInteger;
    end;
end;


procedure DoVarOpBool(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp);
var
  l,r: Boolean;
begin
  l := VariantToBoolean(vl);
  r := VariantToBoolean(vr);
  case OpCode of
    opAnd : l := l and r;
    opOr  : l := l  or r;
    opXor : l := l xor r;
  else
    VarInvalidOp(vl.vType, vr.vType, OpCode);
  end;
  DoVarClearIfComplex(vl);
  vl.vType := varBoolean;
  vl.vBoolean := l;
end;

procedure DoVarOpNull(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp);
begin
  if (OpCode = opAnd) or (OpCode = opOr) then
    if vl.vType = varNull then begin
      if vr.vType = varNull then begin
        {both null, do nothing }
      end else begin
        {Left null, Right not}
        if OpCode = opAnd then begin
          if not VariantToBoolean(vr) then
            VarCopyProc(vl, vr);
        end else {OpCode = opOr} begin
          if VariantToBoolean(vr) then
            VarCopyProc(vl, vr);
        end;
      end;
    end else begin
      if vr.vType = varNull then begin
        {Right null, Left not}
        if OpCode = opAnd then begin
          if VariantToBoolean(vl) then begin
            DoVarClearIfComplex(vl);
            vl.vType := varNull;
          end;
        end else {OpCode = opOr} begin
          if not VariantToBoolean(vl) then begin
            DoVarClearIfComplex(vl);
            vl.vType := varNull;
          end;
        end;
      end else begin
        { both not null, shouldn't happen }
        VarInvalidOp(vl.vType, vr.vType, OpCode);
      end;
    end
  else begin
    DoVarClearIfComplex(vl);
    vl.vType := varNull;
  end;
end;

procedure DoVarOpWStrCat(var vl : TVarData; const vr : TVarData);
var
  ws: WideString;
begin
  ws := VariantToWideString(vl) + VariantToWideString(vr);
  DoVarClearIfComplex(vl);
  vl.vType := varOleStr;
  { transfer the WideString without making a copy }
  Pointer(vl.vOleStr) := Pointer(ws);
  { prevent the WideString from being freed, the reference has been transfered
    from the local to the variant and will be correctly finalized when the
    variant is finalized. }
  Pointer(ws) := nil;
end;

procedure DoVarOpLStrCat(var vl: TVarData; const vr : TVarData);
var
  s: AnsiString;
begin
  s := VariantToAnsiString(vl) + VariantToAnsiString(vr);
  DoVarClearIfComplex(vl);
  vl.vType := varString;
  { transfer the AnsiString without making a copy }
  Pointer(vl.vString) := Pointer(s);
  { prevent the AnsiString from being freed, the reference has been transfered
    from the local to the variant and will be correctly finalized when the
    variant is finalized. }
  Pointer(s) := nil;
end;

procedure DoVarOpDate(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp);
{$ifndef FPUNONE}
var
  l, r : TDateTime;
begin
  l := VariantToDate(vl);
  r := VariantToDate(vr);
  case OpCode of
    opAdd      : l := l + r;
    opSubtract : l := l - r;
  else
    VarInvalidOp(vl.vType, vr.vType, OpCode);
  end;
  DoVarClearIfComplex(vl);
  vl.vType := varDate;
  vl.vDate := l;
{$else}
begin
   VarInvalidOp(vl.vType, vr.vType, OpCode);
{$endif}
end;

procedure DoVarOpCurr(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp; const lct, rct : TCommonType);
{$ifndef FPUNONE}
var
  c  : Currency;
  d  : Double;
begin
  case OpCode of
    opAdd:
      c := VariantToCurrency(vl) + VariantToCurrency(vr);
    opSubtract:
      c := VariantToCurrency(vl) - VariantToCurrency(vr);
    opMultiply:
      if lct = ctCurrency then
        if rct = ctCurrency then {both Currency}
          c := VariantToCurrency(vl) * VariantToCurrency(vr)
        else {Left Currency}
          c := VariantToCurrency(vl) * VariantToDouble(vr)
      else
        if rct = ctCurrency then {rigth Currency}
          c := VariantToDouble(vl) * VariantToCurrency(vr)
        else {non Currency, error}
          VarInvalidOp(vl.vType, vr.vType, OpCode);
    opDivide:
      if lct = ctCurrency then
        if rct = ctCurrency then {both Currency}
          c := VariantToCurrency(vl) / VariantToCurrency(vr)
        else {Left Currency}
          c := VariantToCurrency(vl) / VariantToDouble(vr)
      else
        if rct = ctCurrency then begin {rigth Currency}
          d := VariantToCurrency(vl) / VariantToCurrency(vr);
          DoVarClearIfComplex(vl);
          vl.vType := varDouble;
          vl.vDouble := d;
          Exit;
        end else {non Currency, error}
          VarInvalidOp(vl.vType, vr.vType, OpCode);
    opPower:
      if lct = ctCurrency then
        if rct = ctCurrency then {both Currency}
          c := VariantToCurrency(vl) ** VariantToCurrency(vr)
        else {Left Currency}
          c := VariantToCurrency(vl) ** VariantToDouble(vr)
      else
        if rct = ctCurrency then {rigth Currency}
          c := VariantToDouble(vl) ** VariantToCurrency(vr)
        else {non Currency, error}
          VarInvalidOp(vl.vType, vr.vType, OpCode);
  else
    VarInvalidOp(vl.vType, vr.vType, OpCode);
  end;
  DoVarClearIfComplex(vl);
  vl.vType := varCurrency;
  vl.vCurrency := c;
{$else}
begin
   VarInvalidOp(vl.vType, vr.vType, OpCode);
{$endif}
end;

procedure DoVarOpComplex(var vl : TVarData; const vr : TVarData; const OpCode : TVarOp);
var Handler: TCustomVariantType;
begin
  if FindCustomVariantType(vl.vType, Handler) then
    Handler.BinaryOp(vl, vr, OpCode)
  else if FindCustomVariantType(vr.vType, Handler) then
    Handler.BinaryOp(vl, vr, OpCode)
  else
   VarInvalidOp(vl.vType, vr.vType, OpCode);
end;

procedure SysVarOp(var Left : Variant; const Right : Variant; OpCode : TVarOp);
var
  lct: TCommonType;
  rct: TCommonType;
  {$IFDEF DEBUG_VARIANTS}
  i: Integer;
  {$ENDIF}
begin
  { as the function in cvarutil.inc can handle varByRef correctly we simply
    resolve the final type }
  lct := MapToCommonType(VarTypeDeRef(Left));
  rct := MapToCommonType(VarTypeDeRef(Right));

  {$IFDEF DEBUG_VARIANTS}
  if __DEBUG_VARIANTS then begin
    WriteLn('SysVarOp $', IntToHex(Cardinal(@TVarData(Left)),8), ' ', GetEnumName(TypeInfo(TVarOp), Ord(OpCode)) ,' $', IntToHex(Cardinal(@TVarData(Right)),8));
    DumpVariant('SysVarOp/TVarData(Left)', TVarData(Left));
    WriteLn('lct ', GetEnumName(TypeInfo(TCommonType), Ord(lct)));

    DumpVariant('SysVarOp/TVarData(Right)', TVarData(Right));
    WriteLn('rct ', GetEnumName(TypeInfo(TCommonType), Ord(rct)));

    WriteLn('common ', GetEnumName(TypeInfo(TCommonType), Ord(FindOpCommonType[lct, rct])));
  end;
  {$ENDIF}

  case FindOpCommonType[lct, rct] of
    ctEmpty:
      case OpCode of
        opDivide:
          Error(reZeroDivide);
        opIntDivide, opModulus:
          Error(reDivByZero);
      else
        DoVarClear(TVarData(Left));
      end;
    ctAny:
      DoVarOpAny(TVarData(Left),TVarData(Right),OpCode);
    ctLongInt:
      case OpCode of
        opAdd..opMultiply,opPower:
          DoVarOpInt64to32(TVarData(Left),TVarData(Right),OpCode);
        opDivide:
          DoVarOpFloat(TVarData(Left),TVarData(Right),OpCode);
      else
        DoVarOpLongInt(TVarData(Left),TVarData(Right),OpCode);
      end;
{$ifndef FPUNONE}
    ctFloat:
      if OpCode in [opAdd,opSubtract,opMultiply,opDivide] then
        DoVarOpFloat(TVarData(Left),TVarData(Right),OpCode)
      else
        DoVarOpInt64to32(TVarData(Left),TVarData(Right),OpCode);
{$endif}
    ctBoolean:
      case OpCode of
        opAdd..opMultiply, opPower:
          DoVarOpFloat(TVarData(Left),TVarData(Right),OpCode);
        opIntDivide..opShiftRight:
          DoVarOpLongInt(TVarData(Left),TVarData(Right),OpCode);
        opAnd..opXor:
          DoVarOpBool(TVarData(Left),TVarData(Right),OpCode);
      else
        VarInvalidOp(TVarData(Left).vType, TVarData(Right).vType, OpCode);
      end;
    ctInt64:
      if OpCode <> opDivide then
        DoVarOpInt64(TVarData(Left),TVarData(Right),OpCode)
      else
        DoVarOpFloat(TVarData(Left),TVarData(Right),OpCode);
    ctNull:
      DoVarOpNull(TVarData(Left),TVarData(Right),OpCode);
    ctWideStr:
      case OpCode of
        opAdd:
          DoVarOpWStrCat(TVarData(Left),TVarData(Right));
        opSubtract..opDivide,opPower:
          DoVarOpFloat(TVarData(Left),TVarData(Right),OpCode);
        opIntDivide..opXor:
          DoVarOpInt64to32(TVarData(Left),TVarData(Right),OpCode);
      else
        VarInvalidOp(TVarData(Left).vType, TVarData(Right).vType, OpCode);
      end;
{$ifndef FPUNONE}
    ctDate:
      case OpCode of
        opAdd:
          DoVarOpDate(TVarData(Left),TVarData(Right),OpCode);
        opSubtract: begin
          DoVarOpDate(TVarData(Left),TVarData(Right),OpCode);
            if lct = rct then {both are date}
              TVarData(Left).vType := varDouble;
        end;
        opMultiply, opDivide:
          DoVarOpFloat(TVarData(Left),TVarData(Right),OpCode);
      else
        DoVarOpInt64to32(TVarData(Left),TVarData(Right),OpCode);
      end;
    ctCurrency:
      if OpCode in [opAdd..opDivide, opPower] then
        DoVarOpCurr(TVarData(Left),TVarData(Right),OpCode, lct, rct)
      else
        DoVarOpInt64to32(TVarData(Left),TVarData(Right),OpCode);
{$endif}
    ctString:
      case OpCode of
        opAdd:
          DoVarOpLStrCat(TVarData(Left),TVarData(Right));
        opSubtract..opDivide,opPower:
          DoVarOpFloat(TVarData(Left),TVarData(Right),OpCode);
        opIntDivide..opXor:
          DoVarOpInt64to32(TVarData(Left),TVarData(Right),OpCode);
      else
        VarInvalidOp(TVarData(Left).vType, TVarData(Right).vType, OpCode);
      end;
  else
    { more complex case }
    DoVarOpComplex(TVarData(Left),TVarData(Right),OpCode);
  end;
end;

procedure DoVarNegAny(var v: TVarData);
begin
  VarInvalidOp(v.vType, opNegate);
end;

procedure DoVarNegComplex(var v: TVarData);
begin
  { custom variants? }
  VarInvalidOp(v.vType, opNegate);
end;

procedure sysvarneg(var v: Variant);
const
  BoolMap: array [Boolean] of SmallInt = (0, -1);
begin
  with TVarData(v) do case vType of
    varEmpty: begin
      vSmallInt := 0;
      vType := varSmallInt;
    end;
    varNull:;
    varSmallint: vSmallInt := -vSmallInt;
    varInteger:  vInteger  := -vInteger;
{$ifndef FPUNONE}
    varSingle:   vSingle   := -vSingle;
    varDouble:   vDouble   := -vDouble;
    varCurrency: vCurrency := -vCurrency;
    varDate:     vDate     := -vDate;
    varOleStr:   sysvarfromreal(v, -VariantToDouble(TVarData(v)));
{$else}
    varOleStr:   sysvarfromint64(v, -VariantToInt64(TVarData(v)));
{$endif}
    varBoolean: begin
      vSmallInt := BoolMap[vBoolean];
      vType := varSmallInt;
    end;
    varShortInt: vShortInt := -vShortInt;
    varByte: begin
      vSmallInt := -vByte;
      vType := varSmallInt;
    end;
    varWord: begin
      vInteger := -vWord;
      vType := varInteger;
    end;
    varLongWord:
      if vLongWord and $80000000 <> 0 then begin
        vInt64 := -vLongWord;
        vType := varInt64;
      end else begin
        vInteger := -vLongWord;
        vType := varInteger;
      end;
    varInt64:    vInt64    := -vInt64;
    varQWord: begin
      if vQWord and $8000000000000000 <> 0 then
        VarRangeCheckError(varQWord, varInt64);
      vInt64 := -vQWord;
      vType := varInt64;
    end;
    varVariant:  v         := -Variant(PVarData(vPointer)^);
  else {with TVarData(v) do case vType of}
    case vType of
{$ifndef FPUNONE}
      varString:   sysvarfromreal(v, -VariantToDouble(TVarData(v)));
{$else}
      varString:   sysvarfromint64(v, -VariantToInt64(TVarData(v)));
{$endif}
      varAny:      DoVarNegAny(TVarData(v));
    else {case vType of}
      if (vType and not varTypeMask) = varByRef then
        case vType and varTypeMask of
          varSmallInt: begin
            vSmallInt := -PSmallInt(vPointer)^;
            vType := varSmallInt;
          end;
          varInteger: begin
            vInteger := -PInteger(vPointer)^;
            vType := varInteger;
          end;
{$ifndef FPUNONE}
          varSingle: begin
            vSingle := -PSingle(vPointer)^;
            vType := varSingle;
          end;
          varDouble: begin
            vDouble := -PDouble(vPointer)^;
            vType := varDouble;
          end;
          varCurrency: begin
            vCurrency := -PCurrency(vPointer)^;
            vType := varCurrency;
          end;
          varDate: begin
            vDate := -PDate(vPointer)^;
            vType := varDate;
          end;
          varOleStr: sysvarfromreal(v, -VariantToDouble(TVarData(v)));
{$else}
          varOleStr: sysvarfromint64(v, -VariantToInt64(TVarData(v)));
{$endif}
          varBoolean: begin
            vSmallInt := BoolMap[PWordBool(vPointer)^];
            vType := varSmallInt;
          end;
          varShortInt: begin
            vShortInt := -PShortInt(vPointer)^;
            vType := varShortInt;
          end;
          varByte: begin
            vSmallInt := -PByte(vPointer)^;
            vType := varSmallInt;
          end;
          varWord: begin
            vInteger := -PWord(vPointer)^;
            vType := varInteger;
          end;
          varLongWord:
            if PLongWord(vPointer)^ and $80000000 <> 0 then begin
              vInt64 := -PLongWord(vPointer)^;
              vType := varInt64;
            end else begin
              vInteger := -PLongWord(vPointer)^;
              vType := varInteger;
            end;
          varInt64: begin
            vInt64 := -PInt64(vPointer)^;
            vType := varInt64;
          end;
          varQWord: begin
            if PQWord(vPointer)^ and $8000000000000000 <> 0 then
              VarRangeCheckError(varQWord, varInt64);
            vInt64 := -PQWord(vPointer)^;
            vType := varInt64;
          end;
          varVariant:
            v := -Variant(PVarData(vPointer)^);
        else {case vType and varTypeMask of}
          DoVarNegComplex(TVarData(v));
        end {case vType and varTypeMask of}
      else {if (vType and not varTypeMask) = varByRef}
        DoVarNegComplex(TVarData(v));
    end; {case vType of}
  end; {with TVarData(v) do case vType of}
end;

procedure DoVarNotAny(var v: TVarData);
begin
  VarInvalidOp(v.vType, opNot);
end;

procedure DoVarNotOrdinal(var v: TVarData);
var
  i: Int64;
begin
  { only called for types that do no require finalization }
  i := VariantToInt64(v);
  with v do
    if (i < Low(Integer)) or (i > High(Integer)) then begin
      vInt64 := not i;
      vType := varInt64;
    end else begin
      vInteger := not Integer(i);
      vType := varInteger;
    end
end;

procedure DoVarNotWStr(var v: TVarData; const p: Pointer);
var
  i: Int64;
  e: Word;
  b: Boolean;
begin
  Val(WideString(p), i, e);
  with v do
    if e = 0 then begin
      DoVarClearIfComplex(v);
      if (i < Low(Integer)) or (i > High(Integer)) then begin
        vInt64 := not i;
        vType := varInt64;
      end else begin
        vInteger := not Integer(i);
        vType := varInteger;
      end
    end else begin
      if not TryStrToBool(WideString(p), b) then
        VarInvalidOp(vType, opNot);
      DoVarClearIfComplex(v);
      vBoolean := not b;
      vType := varBoolean;
    end;
end;

procedure DoVarNotLStr(var v: TVarData; const p: Pointer);
var
  i: Int64;
  e: Word;
  b: Boolean;
begin
  Val(AnsiString(p), i, e);
  with v do
    if e = 0 then begin
      DoVarClearIfComplex(v);
      if (i < Low(Integer)) or (i > High(Integer)) then begin
        vInt64 := not i;
        vType := varInt64;
      end else begin
        vInteger := not Integer(i);
        vType := varInteger;
      end
    end else begin
      if not TryStrToBool(AnsiString(p), b) then
        VarInvalidOp(v.vType, opNot);
      DoVarClearIfComplex(v);
      vBoolean := not b;
      vType := varBoolean;
    end;
end;

procedure DoVarNotComplex(var v: TVarData);
begin
  { custom variant support ?}
  VarInvalidOp(v.vType, opNot);
end;

procedure sysvarnot(var v: Variant);
begin
  with TVarData(v) do case vType of
    varEmpty:    v := -1;
    varNull:;
    varSmallint: vSmallInt := not vSmallInt;
    varInteger:  vInteger  := not vInteger;
{$ifndef FPUNONE}
    varSingle,
    varDouble,
    varCurrency,
    varDate:     DoVarNotOrdinal(TVarData(v));
{$endif}
    varOleStr:   DoVarNotWStr(TVarData(v), Pointer(vOleStr));
    varBoolean:  vBoolean := not vBoolean;
    varShortInt: vShortInt := not vShortInt;
    varByte:     vByte := not vByte;
    varWord:     vWord := not vWord;
    varLongWord: vLongWord := not vLongWord;
    varInt64:    vInt64    := not vInt64;
    varQWord:    vQWord    := not vQWord;
    varVariant:  v         := not Variant(PVarData(vPointer)^);
  else {with TVarData(v) do case vType of}
    case vType of
      varString:   DoVarNotLStr(TVarData(v), Pointer(vString));
      varAny:      DoVarNotAny(TVarData(v));
    else {case vType of}
      if (vType and not varTypeMask) = varByRef then
        case vType and varTypeMask of
          varSmallInt: begin
            vSmallInt := not PSmallInt(vPointer)^;
            vType := varSmallInt;
          end;
          varInteger: begin
            vInteger := not PInteger(vPointer)^;
            vType := varInteger;
          end;
{$ifndef FPUNONE}
          varSingle,
          varDouble,
          varCurrency,
          varDate: DoVarNotOrdinal(TVarData(v));
{$endif}
          varOleStr: DoVarNotWStr(TVarData(v), PPointer(vPointer)^);
          varBoolean: begin
            vBoolean := not PWordBool(vPointer)^;
            vType := varBoolean;
          end;
          varShortInt: begin
            vShortInt := not PShortInt(vPointer)^;
            vType := varShortInt;
          end;
          varByte: begin
            vByte := not PByte(vPointer)^;
            vType := varByte;
          end;
          varWord: begin
            vWord := not PWord(vPointer)^;
            vType := varWord;
          end;
          varLongWord: begin
            vLongWord := not PLongWord(vPointer)^;
            vType := varLongWord;
          end;
          varInt64: begin
            vInt64 := not PInt64(vPointer)^;
            vType := varInt64;
          end;
          varQWord: begin
            vQWord := not PQWord(vPointer)^;
            vType := varQWord;
          end;
          varVariant:
            v := not Variant(PVarData(vPointer)^);
        else {case vType and varTypeMask of}
          DoVarNotComplex(TVarData(v));
        end {case vType and varTypeMask of}
      else {if (vType and not varTypeMask) = varByRef}
        DoVarNotComplex(TVarData(v));
    end; {case vType of}
  end; {with TVarData(v) do case vType of}
end;

{
  This procedure is needed to destroy and clear non-standard variant type array elements,
  which can not be handled by SafeArrayDestroy.
  If array element type is varVariant, then clear each element individually before
  calling VariantClear for array. VariantClear just calls SafeArrayDestroy.
}
procedure DoVarClearArray(var VArray: TVarData);
var
  arr: pvararray;
  i, cnt: cardinal;
  data: pvardata;
begin
  if VArray.vtype and varTypeMask = varVariant then begin
    if WordBool(VArray.vType and varByRef) then
      arr:=PVarArray(VArray.vPointer^)
    else
      arr:=VArray.vArray;
    VarResultCheck(SafeArrayAccessData(arr, data));
    try
      { Calculation total number of elements in the array }
      cnt:=1;
{$push}
{ arr^.bounds[] is an array[0..0] }
{$r-}
      for i:=0 to arr^.dimcount - 1 do
        cnt:=cnt*cardinal(arr^.Bounds[i].ElementCount);
{$pop}

      { Clearing each element }
      for i:=1 to cnt do begin
        DoVarClear(data^);
        Inc(pointer(data), arr^.ElementSize);
      end;
    finally
      VarResultCheck(SafeArrayUnaccessData(arr));
    end;
  end;
  VariantClear(VArray);
end;

procedure DoVarClearComplex(var v : TVarData);
var
  Handler : TCustomVariantType;
begin
  with v do
    if vType < varInt64 then
      VarResultCheck(VariantClear(v))
    else if vType = varString then
      begin
        AnsiString(vString) := '';
        vType := varEmpty;
      end
    else if vType = varUString then
      begin
        UnicodeString(vString) := '';
        vType := varEmpty;
      end
    else if vType = varAny then
      ClearAnyProc(v)
    else if vType and varArray <> 0 then
      DoVarClearArray(v)
    else if FindCustomVariantType(vType, Handler) then
      Handler.Clear(v)
    else begin
      { ignore errors, if the OS doesn't know how to free it, we don't either }
      VariantClear(v);
      vType := varEmpty;
    end;
end;

type
  TVarArrayCopyCallback = procedure(var aDest: TVarData; const aSource: TVarData);

procedure DoVarCopyArray(var aDest: TVarData; const aSource: TVarData; aCallback: TVarArrayCopyCallback);
var
  SourceArray : PVarArray;
  SourcePtr   : Pointer;
  DestArray   : PVarArray;
  DestPtr     : Pointer;

  Bounds      : array[0..63] of TVarArrayBound;
  Iterator    : TVariantArrayIterator;

  Dims        : Integer;
  HighBound   : Longint;
  i           : Integer;
begin
  with aSource do begin
    if vType and varArray = 0 then
      VarResultCheck(VAR_INVALIDARG);

    if (vType and varTypeMask) = varVariant then begin

      if (vType and varByRef) <> 0 then
        SourceArray := PVarArray(vPointer^)
      else
        SourceArray := vArray;

      Dims := SourceArray^.DimCount;
      for i := 0 to Pred(Dims) do
        with Bounds[i] do begin
          VarResultCheck(SafeArrayGetLBound(SourceArray, Succ(i), LowBound));
          VarResultCheck(SafeArrayGetUBound(SourceArray, Succ(i), HighBound));
          ElementCount := HighBound - LowBound + 1;
        end;

      DestArray := SafeArrayCreate(varVariant, Dims, PVarArrayBoundArray(@Bounds)^);
      if not Assigned(DestArray) then
        VarArrayCreateError;

      DoVarClearIfComplex(aDest);
      with aDest do begin
        vType := varVariant or varArray;
        vArray := DestArray;
      end;

      Iterator.Init(Dims, @Bounds);
      try
        if not(Iterator.AtEnd) then
          repeat
            VarResultCheck(SafeArrayPtrOfIndex(SourceArray, Iterator.Coords, SourcePtr));
            VarResultCheck(SafeArrayPtrOfIndex(DestArray, Iterator.Coords, DestPtr));
            aCallback(PVarData(DestPtr)^, PVarData(SourcePtr)^);
          until not Iterator.Next;
      finally
        Iterator.Done;
      end;

    end else
      VarResultCheck(VariantCopy(aDest, aSource));
  end;
end;

procedure DoVarCopyComplex(var Dest: TVarData; const Source: TVarData);
var
  Handler: TCustomVariantType;
begin
  DoVarClearIfComplex(Dest);

  with Source do
    if vType < varInt64 then
      VarResultCheck(VariantCopy(Dest, Source))
    else if vType = varString then begin
      Dest.vType := varString;
      Dest.vString := nil;
      AnsiString(Dest.vString) := AnsiString(vString);
    end else if vType = varOleStr then begin
      Dest.vType := varOleStr;
      Dest.vOleStr := nil;
      WideString(Pointer(Dest.vOleStr)) := WideString(Pointer(vOleStr));
    end else if vType = varAny then begin
      Dest := Source;
      RefAnyProc(Dest);
    end else if vType and varArray <> 0 then
      DoVarCopyArray(Dest, Source, @DoVarCopy)
    else if (vType and varByRef <> 0) and
             (((vType xor varByRef) = varString)
               or ((vType xor varByRef)= varOleStr)) then
      Dest := Source
    else if FindCustomVariantType(vType, Handler) then
      Handler.Copy(Dest, Source, False)
    else
      VarResultCheck(VariantCopy(Dest, Source));
end;

procedure DoVarCopy(var Dest : TVarData; const Source : TVarData);
begin
  if @Dest <> @Source then
    if (Source.vType and varComplexType) = 0 then begin
      DoVarClearIfComplex(Dest);
      Dest := Source;
    end else
      DoVarCopyComplex(Dest, Source);
end;

procedure sysvarcopy (var Dest : Variant; const Source : Variant);
begin
  DoVarCopy(TVarData(Dest),TVarData(Source));
end;

procedure DoVarAddRef(var v : TVarData); inline;
var
  Dummy : TVarData;
begin
  Dummy := v;
  v.vType := varEmpty;
  DoVarCopy(v, Dummy);
end;

procedure sysvaraddref(var v : Variant);
begin
  DoVarAddRef(TVarData(v));
end;

procedure DoVarCastWStr(var aDest : TVarData; const aSource : TVarData);
begin
  SysVarFromWStr(Variant(aDest), VariantToWideString(aSource));
end;

procedure DoVarCastLStr(var aDest : TVarData; const aSource : TVarData);
begin
  SysVarFromLStr(Variant(aDest), VariantToAnsiString(aSource));
end;

procedure DoVarCastDispatch(var aDest : TVarData; const aSource : TVarData);
var
  Disp: IDispatch;
begin
  SysVarToDisp(Disp, Variant(aSource));
  SysVarFromDisp(Variant(aDest), Disp);
end;

procedure DoVarCastInterface(var aDest : TVarData; const aSource : TVarData);
var
  Intf: IInterface;
begin
  SysVarToIntf(Intf, Variant(aSource));
  SysVarFromIntf(Variant(aDest), Intf);
end;

procedure DoVarCastAny(var aDest : TVarData; const aSource : TVarData; aVarType : LongInt);
begin
  VarCastError(aSource.vType, aVarType)
end;

procedure DoVarCastFallback(var aDest : TVarData; const aSource : TVarData; aVarType : LongInt);
begin
  if aSource.vType and varTypeMask >= varInt64 then begin
    DoVarCast(aDest, aSource, varOleStr);
    VarResultCheck(VariantChangeTypeEx(aDest, aDest, VAR_LOCALE_USER_DEFAULT,
      0, aVarType), aSource.vType, aVarType);
  end else if aVarType and varTypeMask < varInt64 then
    VarResultCheck(VariantChangeTypeEx(aDest, aSource, VAR_LOCALE_USER_DEFAULT,
      0, aVarType), aSource.vType, aVarType)
  else
    VarCastError(aSource.vType, aVarType);
end;

procedure DoVarCastComplex(var aDest : TVarData; const aSource : TVarData; aVarType : LongInt);
var
  Handler: TCustomVariantType;
begin
  if aSource.vType = varAny then
    DoVarCastAny(aDest, aSource, aVarType)
  else if FindCustomVariantType(aSource.vType, Handler) then
    Handler.CastTo(aDest, aSource, aVarType)
  else if FindCustomVariantType(aVarType, Handler) then
    Handler.Cast(aDest, aSource)
  else
    DoVarCastFallback(aDest, aSource, aVarType);
end;

procedure DoVarCast(var aDest : TVarData; const aSource : TVarData; aVarType : LongInt);
var
  Handler: TCustomVariantType;
begin
  with aSource do
    if vType = aVarType then
      DoVarCopy(aDest, aSource)
    else if FindCustomVariantType(vType, Handler) then
      Handler.CastTo(aDest, aSource, aVarType)
    else begin
      if (vType = varNull) and NullStrictConvert then
        VarCastError(varNull, aVarType);

      case aVarType of
        varEmpty, varNull: begin
          DoVarClearIfComplex(aDest);
          aDest.vType := aVarType;
        end;
        varSmallInt: SysVarFromInt(Variant(aDest), VariantToSmallInt(aSource), -2);
        varInteger:  SysVarFromInt(Variant(aDest), VariantToLongInt(aSource), -4);
{$ifndef FPUNONE}
        varSingle:   SysVarFromSingle(Variant(aDest), VariantToSingle(aSource));
        varDouble:   SysVarFromDouble(Variant(aDest), VariantToDouble(aSource));
        varCurrency: SysVarFromCurr(Variant(aDest), VariantToCurrency(aSource));
        varDate:     SysVarFromTDateTime(Variant(aDest), VariantToDate(aSource));
{$endif}
        varOleStr:   DoVarCastWStr(aDest, aSource);
        varBoolean:  SysVarFromBool(Variant(aDest), VariantToBoolean(aSource));
        varShortInt: SysVarFromInt(Variant(aDest), VariantToShortInt(aSource), -1);
        varByte:     SysVarFromInt(Variant(aDest), VariantToByte(aSource), 1);
        varWord:     SysVarFromInt(Variant(aDest), VariantToLongInt(aSource), 2);
        varLongWord: SysVarFromInt(Variant(aDest), Integer(VariantToCardinal(aSource)), 4);
        varInt64:    SysVarFromInt64(Variant(aDest), VariantToInt64(aSource));
        varQWord:    SysVarFromWord64(Variant(aDest), VariantToQWord(aSource));

        varDispatch: DoVarCastDispatch(aDest, aSource);
        varUnknown:  DoVarCastInterface(aDest, aSource);
      else
        case aVarType of
          varString: DoVarCastLStr(aDest, aSource);
          varAny:    VarCastError(vType, varAny);
        else
          DoVarCastComplex(aDest, aSource, aVarType);
        end;
      end;
    end;

end;

procedure sysvarcast (var aDest : Variant; const aSource : Variant; aVarType : LongInt);
begin
  DoVarCast(TVarData(aDest), TVarData(aSource), aVarType);
end;


procedure sysvarfromdynarray(var Dest : Variant; const Source : Pointer; TypeInfo: Pointer);
begin
  DynArrayToVariant(Dest,Source,TypeInfo);
  if VarIsEmpty(Dest) then
    VarCastError;
end;


procedure sysolevarfrompstr(var Dest : olevariant; const Source : ShortString);
begin
  sysvarfromwstr(Variant(TVarData(Dest)), Source);
end;


procedure sysolevarfromlstr(var Dest : olevariant; const Source : AnsiString);
begin
  sysvarfromwstr(Variant(TVarData(Dest)), Source);
end;

procedure DoOleVarFromAny(var aDest : TVarData; const aSource : TVarData);
begin
  VarCastErrorOle(aSource.vType);
end;

procedure DoOleVarFromVar(var aDest : TVarData; const aSource : TVarData);
var
  Handler: TCustomVariantType;
begin
  with aSource do
    if vType = varByRef or varVariant then
      DoOleVarFromVar(aDest, PVarData(vPointer)^)
    else begin
      case vType of
        varShortInt, varByte, varWord:
          DoVarCast(aDest, aSource, varInteger);
        varLongWord:
          if vLongWord and $80000000 = 0 then
            DoVarCast(aDest, aSource, varInteger)
          else
{$ifndef FPUNONE}
            if OleVariantInt64AsDouble then
              DoVarCast(aDest, aSource, varDouble)
            else
{$endif}
              DoVarCast(aDest, aSource, varInt64);
        varInt64:
          if (vInt64 < Low(Integer)) or (vInt64 > High(Integer)) then
{$ifndef FPUNONE}
            if OleVariantInt64AsDouble then
              DoVarCast(aDest, aSource, varDouble)
            else
{$endif}
              DoVarCast(aDest, aSource, varInt64)
          else
            DoVarCast(aDest, aSource, varInteger);
        varQWord:
          if vQWord > High(Integer) then
{$ifndef FPUNONE}
            if OleVariantInt64AsDouble or (vQWord and $8000000000000000 <> 0) then
              DoVarCast(aDest, aSource, varDouble)
            else
{$endif}
              DoVarCast(aDest, aSource, varInt64)
          else
            DoVarCast(aDest, aSource, varInteger);
        varString:
          DoVarCast(aDest, aSource, varOleStr);
        varAny:
          DoOleVarFromAny(aDest, aSource);
      else
        if (vType and varArray) <> 0 then
          DoVarCopyArray(aDest, aSource, @DoOleVarFromVar)
        else if (vType and varTypeMask) < CFirstUserType then
          DoVarCopy(aDest, aSource)
        else if FindCustomVariantType(vType, Handler) then
          Handler.CastToOle(aDest, aSource)
        else
          VarCastErrorOle(vType);
      end;
    end;
end;

procedure sysolevarfromvar(var aDest : OleVariant; const aSource : Variant);
begin
  DoOleVarFromVar(TVarData(aDest), TVarData(aSource));
end;

procedure sysolevarfromint(var Dest : olevariant; const Source : Int64; const range : ShortInt);
begin
  DoVarClearIfComplex(TVarData(Dest));
  { 64-bit values have their own types, all smaller ones are stored as signed 32-bit value }
  with TVarData(Dest) do begin
    case range of
      -8: begin
        vInt64 := Int64(Source);
        vType := varInt64;
      end;
      8: begin
        vQWord := QWord(Source);
        vType := varQWord;
      end;
      else begin
        vInteger := LongInt(Source);
        vType := varInteger;
      end;
    end;
  end;
end;

procedure DoVarCastOle(var aDest: TVarData; const aSource: TVarData; aVarType: LongInt);
var
  Handler: TCustomVariantType;
begin
  with aSource do
  if vType = varByRef or varVariant then
    DoVarCastOle(aDest, PVarData(VPointer)^, aVarType)
  else
    if (aVarType = varString) or (aVarType = varAny) then
      VarCastError(vType, aVarType)
    else if FindCustomVariantType(vType, Handler) then
      Handler.CastTo(aDest, aSource, aVarType)
    else
      DoVarCast(aDest, aSource, aVarType);
end;

procedure sysvarcastole(var Dest : Variant; const Source : Variant; aVarType : LongInt);
begin
  DoVarCastOle(TVarData(Dest), TVarData(Source), aVarType);
end;


procedure sysdispinvoke(Dest : PVarData; var source : TVarData;calldesc : pcalldesc;params : Pointer);cdecl;
var
  temp  : TVarData;
  tempp : ^TVarData;
  customvarianttype : TCustomVariantType;
begin
  if Source.vType=(varByRef or varVariant) then
    sysdispinvoke(Dest,PVarData(Source.vPointer)^,calldesc,params)
  else
    begin
      try
        { get a defined Result }
        if not(assigned(Dest)) then
          tempp:=nil
        else
          begin
            fillchar(temp,SizeOf(temp),0);
            tempp:=@temp;
          end;
        case Source.vType of
          varDispatch,
          varAny,
          varUnknown,
          varDispatch or varByRef,
          varAny or varByRef,
          varUnknown or varByRef:
            VarDispProc(pvariant(tempp),Variant(Source),calldesc,params);
          else
            begin
              if FindCustomVariantType(Source.vType,customvarianttype) then
                customvarianttype.DispInvoke(tempp,Source,calldesc,params)
              else
                VarInvalidOp;
            end;
        end;
      finally
        if assigned(tempp) then
          begin
            DoVarCopy(Dest^,tempp^);
            DoVarClear(temp);
          end;
      end;
    end;
end;


procedure sysvararrayredim(var a : Variant;highbound : SizeInt);
var
  src : TVarData;
  p : pvararray;
  newbounds : tvararraybound;
begin
  src:=TVarData(a);
  { get final Variant }
  while src.vType=varByRef or varVariant do
    src:=TVarData(src.vPointer^);

  if (src.vType and varArray)<>0 then
    begin
      { get Pointer to the array }
      if (src.vType and varByRef)<>0 then
        p:=pvararray(src.vPointer^)
      else
        p:=src.vArray;

{$push}
{$r-}
      if highbound<p^.Bounds[p^.dimcount-1].LowBound-1 then
        VarInvalidArgError;

      newbounds.LowBound:=p^.Bounds[p^.dimcount-1].LowBound;
{$pop}
      newbounds.ElementCount:=highbound-newbounds.LowBound+1;

      VarResultCheck(SafeArrayRedim(p,newbounds));
    end
  else
    VarInvalidArgError(src.vType);
end;


function getfinalvartype(const v : TVarData) : TVarType;{$IFDEF VARIANTINLINE}inline;{$ENDIF VARIANTINLINE}
var
  p: PVarData;
begin
  p := @v;
  while p^.vType = varByRef or varVariant do
    p := PVarData(p^.vPointer);
  Result := p^.vType;
end;


function sysvararrayget(const a : Variant;indexcount : SizeInt;indices : plongint) : Variant;cdecl;
var
  src : TVarData;
  p : pvararray;
  arraysrc : pvariant;
  arrayelementtype : TVarType;
begin
  src:=TVarData(a);
  { get final Variant }
  while src.vType=varByRef or varVariant do
    src:=TVarData(src.vPointer^);

  if (src.vType and varArray)<>0 then
    begin
      { get Pointer to the array }
      if (src.vType and varByRef)<>0 then
        p:=pvararray(src.vPointer^)
      else
        p:=src.vArray;

      { number of indices ok? }
      if p^.DimCount<>indexcount then
        VarInvalidArgError;

      arrayelementtype:=src.vType and varTypeMask;
      if arrayelementtype=varVariant then
        begin
          VarResultCheck(SafeArrayPtrOfIndex(p,PVarArrayCoorArray(indices),arraysrc));
          Result:=arraysrc^;
        end
      else
        begin
          TVarData(Result).vType:=arrayelementtype;
          VarResultCheck(SafeArrayGetElement(p,PVarArrayCoorArray(indices),@TVarData(Result).vPointer));
        end;
    end
  else
    VarInvalidArgError(src.vType);
end;


procedure sysvararrayput(var a : Variant; const value : Variant;indexcount : SizeInt;indices : plongint);cdecl;
var
  Dest : TVarData;
  p : pvararray;
  arraydest : pvariant;
  valuevtype,
  arrayelementtype : TVarType;
  tempvar : Variant;
begin
  Dest:=TVarData(a);
  { get final Variant }
  while Dest.vType=varByRef or varVariant do
    Dest:=TVarData(Dest.vPointer^);

  valuevtype:=getfinalvartype(TVarData(value));

  if not(VarTypeIsValidElementType(valuevtype)) and
    { varString isn't a valid varArray type but it is converted
      later }
    (valuevtype<>varString) then
    VarCastError(valuevtype,Dest.vType);

  if (Dest.vType and varArray)<>0 then
    begin
      { get Pointer to the array }
      if (Dest.vType and varByRef)<>0 then
        p:=pvararray(Dest.vPointer^)
      else
        p:=Dest.vArray;

      { number of indices ok? }
      if p^.DimCount<>indexcount then
        VarInvalidArgError;

      arrayelementtype:=Dest.vType and varTypeMask;
      if arrayelementtype=varVariant then
        begin
          VarResultCheck(SafeArrayPtrOfIndex(p,PVarArrayCoorArray(indices),arraydest));
          { we can't store ansistrings in Variant arrays so we convert the string to
            an olestring }
          if valuevtype=varString then
            begin
              tempvar:=VarToWideStr(value);
              arraydest^:=tempvar;
            end
          else
            arraydest^:=value;
        end
      else
        begin
          VarCast(tempvar,value,arrayelementtype);
          if arrayelementtype in [varOleStr,varDispatch,varUnknown] then
            VarResultCheck(SafeArrayPutElement(p,PVarArrayCoorArray(indices),TVarData(tempvar).vPointer))
          else
            VarResultCheck(SafeArrayPutElement(p,PVarArrayCoorArray(indices),@TVarData(tempvar).vPointer));
        end;
    end
  else
    VarInvalidArgError(Dest.vType);
end;


{ import from system unit }
Procedure fpc_Write_Text_AnsiStr (Len : LongInt; Var f : Text; S : RawByteString); external name 'FPC_WRITE_TEXT_ANSISTR';


function syswritevariant(var t : text; const v : Variant;width : LongInt) : Pointer;
var
  s : AnsiString;
  variantmanager : tvariantmanager;
begin
  GetVariantManager(variantmanager);
  variantmanager.vartolstr(s,v);
  fpc_write_text_ansistr(width,t,s);
  Result:=nil; // Pointer to what should be returned?
end;


function syswrite0Variant(var t : text; const v : Variant) : Pointer;
var
  s : AnsiString;
  variantmanager : tvariantmanager;
begin
  getVariantManager(variantmanager);
  variantmanager.vartolstr(s,v);
  fpc_write_text_ansistr(-1,t,s);
  Result:=nil; // Pointer to what should be returned?
end;

Const
  SysVariantManager : TVariantManager = (
    vartoint      : @sysvartoint;
    vartoint64    : @sysvartoint64;
    vartoword64   : @sysvartoword64;
    vartobool     : @sysvartobool;
{$ifndef FPUNONE}
    vartoreal     : @sysvartoreal;
    vartotdatetime: @sysvartotdatetime;
{$endif}
    vartocurr     : @sysvartocurr;
    vartopstr     : @sysvartopstr;
    vartolstr     : @sysvartolstr;
    vartowstr     : @sysvartowstr;
    vartointf     : @sysvartointf;
    vartodisp     : @sysvartodisp;
    vartodynarray : @sysvartodynarray;
    varfrombool   : @sysvarfromBool;
    varfromint    : @sysvarfromint;
    varfromint64  : @sysvarfromint64;
    varfromword64 : @sysvarfromword64;
{$ifndef FPUNONE}
    varfromreal   : @sysvarfromreal;
    varfromtdatetime: @sysvarfromtdatetime;
{$endif}
    varfromcurr   : @sysvarfromcurr;
    varfrompstr   : @sysvarfrompstr;
    varfromlstr   : @sysvarfromlstr;
    varfromwstr   : @sysvarfromwstr;
    varfromintf   : @sysvarfromintf;
    varfromdisp   : @sysvarfromdisp;
    varfromdynarray: @sysvarfromdynarray;
    olevarfrompstr: @sysolevarfrompstr;
    olevarfromlstr: @sysolevarfromlstr;
    olevarfromvar : @sysolevarfromvar;
    olevarfromint : @sysolevarfromint;
    varop         : @SysVarOp;
    cmpop         : @syscmpop;
    varneg        : @sysvarneg;
    varnot        : @sysvarnot;
    varinit       : @sysvarinit;
    varclear      : @sysvarclear;
    varaddref     : @sysvaraddref;
    varcopy       : @sysvarcopy;
    varcast       : @sysvarcast;
    varcastole    : @sysvarcastole;
    dispinvoke    : @sysdispinvoke;
    vararrayredim : @sysvararrayredim;
    vararrayget   : @sysvararrayget;
    vararrayput   : @sysvararrayput;
    writevariant  : @syswritevariant;
    write0Variant : @syswrite0variant;
  );

Var
  PrevVariantManager : TVariantManager;

Procedure SetSysVariantManager;

begin
  GetVariantManager(PrevVariantManager);
  SetVariantManager(SysVariantManager);
end;

Procedure UnsetSysVariantManager;

begin
  SetVariantManager(PrevVariantManager);
end;


{ ---------------------------------------------------------------------
   Variant support procedures and functions
  ---------------------------------------------------------------------}


function VarType(const V: Variant): TVarType;

begin
  Result:=TVarData(V).vType;
end;


function VarTypeDeRef(const V: Variant): TVarType;
var
  p: PVarData;
begin
  p := @TVarData(V);
  Result := p^.vType and not varByRef;
  while Result = varVariant do begin
    p := p^.vPointer;
    if not Assigned(p) then
      VarBadTypeError;
    Result := p^.vType and not varByRef;
  end;
end;

function VarTypeDeRef(const V: TVarData): TVarType;
begin
  Result := VarTypeDeRef(Variant(v));
end;

function VarAsType(const V: Variant; aVarType: TVarType): Variant;

begin
  sysvarcast(Result,V,aVarType);
end;



function VarIsType(const V: Variant; aVarType: TVarType): Boolean; overload;

begin
  Result:=((TVarData(V).vType and varTypeMask)=aVarType);
end;


function VarIsType(const V: Variant; const AVarTypes: array of TVarType): Boolean; overload;

Var
  I : Integer;

begin
  I:=Low(AVarTypes);
  Result:=False;
  While Not Result and (I<=High(AVarTypes)) do
    begin
      Result:=((TVarData(V).vType and varTypeMask)=AVarTypes[I]);
      inc(i);
    end;
end;


function VarIsByRef(const V: Variant): Boolean;
begin
  Result:=(TVarData(V).vType and varByRef)<>0;
end;


function VarIsEmpty(const V: Variant): Boolean;
begin
  Result:=TVarData(V).vType=varEmpty;
end;


procedure VarCheckEmpty(const V: Variant);
begin
  If VarIsEmpty(V) Then
    VariantError(SErrVarIsEmpty);
end;


procedure VarClear(var V: Variant);{$IFDEF VARIANTINLINE}inline;{$ENDIF VARIANTINLINE}
begin
  sysvarclear(v);
end;


procedure VarClear(var V: OleVariant);{$IFDEF VARIANTINLINE}inline;{$ENDIF VARIANTINLINE}
begin
  { strange casting using TVarData to avoid call of helper olevariant->Variant }
  sysvarclear(Variant(TVarData(v)));
end;


function VarIsNull(const V: Variant): Boolean;
begin
  Result:=TVarData(V).vType=varNull;
end;


function VarIsClear(const V: Variant): Boolean;

Var
  VT : TVarType;
  CustomType: TCustomVariantType;
begin
  VT:=TVarData(V).vType and varTypeMask;
  if VT<CFirstUserType then
    Result:=(VT=varEmpty) or
            (((VT=varDispatch) or (VT=varUnknown))
             and (TVarData(V).vDispatch=Nil))
   else
     Result:=FindCustomVariantType(VT,CustomType) and CustomType.IsClear(TVarData(V));
end;


function VarIsCustom(const V: Variant): Boolean;

begin
  Result:=TVarData(V).vType>=CFirstUserType;
end;


function VarIsOrdinal(const V: Variant): Boolean;
begin
  Result:=(TVarData(V).vType and varTypeMask) in OrdinalVarTypes;
end;



function VarIsFloat(const V: Variant): Boolean;

begin
  Result:=(TVarData(V).vType and varTypeMask) in FloatVarTypes;
end;


function VarIsNumeric(const V: Variant): Boolean;
begin
  Result:=(TVarData(V).vType and varTypeMask) in (OrdinalVarTypes + FloatVarTypes);
end;



function VarIsStr(const V: Variant): Boolean;

begin
  case (TVarData(V).vType and varTypeMask) of
    varOleStr,
    varUString,
    varString :
      Result:=True;
    else
      Result:=False;
  end;
end;

function VarIsBool(const V: Variant): Boolean;
begin
  Result := (TVarData(V).vType and varTypeMask) = varboolean;
end;


function VarToStr(const V: Variant): string;

begin
  Result:=VarToStrDef(V,'');
end;


function VarToStrDef(const V: Variant; const ADefault: string): string;

begin
  If TVarData(V).vType<>varNull then
    Result:=V
  else
    Result:=ADefault;
end;


function VarToWideStr(const V: Variant): WideString;

begin
  Result:=VarToWideStrDef(V,'');
end;


function VarToWideStrDef(const V: Variant; const ADefault: WideString): WideString;

begin
  If TVarData(V).vType<>varNull then
    Result:=V
  else
    Result:=ADefault;
end;


function VarToUnicodeStr(const V: Variant): UnicodeString;

begin
  Result:=VarToUnicodeStrDef(V,'');
end;


function VarToUnicodeStrDef(const V: Variant; const ADefault: UnicodeString): UnicodeString;

begin
  If TVarData(V).vType<>varNull then
    Result:=V
  else
    Result:=ADefault;
end;


{$ifndef FPUNONE}

function VarToDateTime(const V: Variant): TDateTime;
begin
  Result:=VariantToDate(TVarData(V));
end;


function VarFromDateTime(const DateTime: TDateTime): Variant;

begin
  SysVarClear(Result);
  with TVarData(Result) do
    begin
      vType:=varDate;
      vdate:=DateTime;
    end;
end;

{$endif}


function VarInRange(const AValue, AMin, AMax: Variant): Boolean;
begin
  Result:=(AValue>=AMin) and (AValue<=AMax);
end;


function VarEnsureRange(const AValue, AMin, AMax: Variant): Variant;
begin
  If AValue>AMAx then
    Result:=AMax
  else If AValue<AMin Then
    Result:=AMin
  else
    Result:=AValue;
end;


function VarSameValue(const A, B: Variant): Boolean;
  var
    v1,v2 : TVarData;
  begin
    v1:=FindVarData(a)^;
    v2:=FindVarData(b)^;
    if v1.vType in [varEmpty,varNull] then
      Result:=v1.vType=v2.vType
    else if v2.vType in [varEmpty,varNull] then
      Result:=False
    else
      Result:=A=B;
  end;


function VarCompareValue(const A, B: Variant): TVariantRelationship;
  var
    v1,v2 : TVarData;
  begin
    Result:=vrNotEqual;
    v1:=FindVarData(a)^;
    v2:=FindVarData(b)^;
    if (v1.vType in [varEmpty,varNull]) and (v1.vType=v2.vType) then
      Result:=vrEqual
    else if not(v2.vType in [varEmpty,varNull]) and
            not(v1.vType in [varEmpty,varNull]) then
      begin
        if a=b then
          Result:=vrEqual
        else if a>b then
          Result:=vrGreaterThan
        else
          Result:=vrLessThan;
      end;
  end;


function VarIsEmptyParam(const V: Variant): Boolean;
begin
  Result:=(TVarData(V).vType = varError) and
          (TVarData(V).vError=VAR_PARAMNOTFOUND);
end;


procedure SetClearVarToEmptyParam(var V: TVarData);
begin
  VariantClear(V);
  V.vType := varError;
  V.vError := VAR_PARAMNOTFOUND;
end;


function VarIsError(const V: Variant; out aResult: HRESULT): Boolean;
begin
  Result := TVarData(V).vType = varError;
  if Result then
    aResult := TVarData(v).vError;
end;


function VarIsError(const V: Variant): Boolean;
begin
  Result := TVarData(V).vType = varError;
end;


function VarAsError(AResult: HRESULT): Variant;
  begin
    TVarData(Result).vType:=varError;
    TVarData(Result).vError:=AResult;
  end;


function VarSupports(const V: Variant; const IID: TGUID; out Intf): Boolean;
begin
  case TVarData(v).vType of
    varUnknown:
      Result := Assigned(TVarData(v).vUnknown) and (IInterface(TVarData(v).vUnknown).QueryInterface(IID, Intf) = S_OK);
    varUnknown or varByRef:
      Result := Assigned(TVarData(v).vPointer) and Assigned(pointer(TVarData(v).vPointer^)) and (IInterface(TVarData(v).vPointer^).QueryInterface(IID, Intf) = S_OK);
    varDispatch:
      Result := Assigned(TVarData(v).vDispatch) and (IInterface(TVarData(v).vDispatch).QueryInterface(IID, Intf) = S_OK);
    varDispatch or varByRef:
      Result := Assigned(TVarData(v).vPointer) and Assigned(pointer(TVarData(v).vPointer^)) and (IInterface(TVarData(v).vPointer^).QueryInterface(IID, Intf) = S_OK);
    varVariant, varVariant or varByRef:
      Result := Assigned(TVarData(v).vPointer) and VarSupports(Variant(PVarData(TVarData(v).vPointer)^), IID, Intf);
    else
      Result := False;
  end;
end;


function VarSupports(const V: Variant; const IID: TGUID): Boolean;
var
  Dummy: IInterface;
begin
  Result := VarSupports(V, IID, Dummy);
end;


{ Variant copy support }
{$push}
{$warnings off}
procedure VarCopyNoInd(var Dest: Variant; const Source: Variant);

begin
  NotSupported('VarCopyNoInd');
end;
{$pop}

{****************************************************************************
              Variant array support procedures and functions
 ****************************************************************************}

{$push}
{$r-}

function VarArrayCreate(const Bounds: array of SizeInt; aVarType: TVarType): Variant;
  var
    hp : PVarArrayBoundArray;
    p : pvararray;
    i,lengthb : SizeInt;
  begin
    if not(VarTypeIsValidArrayType(aVarType)) or odd(length(Bounds)) then
      VarArrayCreateError;
    lengthb:=length(Bounds) div 2;
    try
      GetMem(hp,lengthb*SizeOf(TVarArrayBound));
      for i:=0 to lengthb-1 do
        begin
          hp^[i].LowBound:=Bounds[i*2];
          hp^[i].ElementCount:=Bounds[i*2+1]-Bounds[i*2]+1;
        end;
      SysVarClear(Result);

      p:=SafeArrayCreate(aVarType,lengthb,hp^);

      if not(assigned(p)) then
        VarArrayCreateError;

      TVarData(Result).vType:=aVarType or varArray;
      TVarData(Result).vArray:=p;
    finally
      FreeMem(hp);
    end;
  end;

{$pop}

function VarArrayCreate(const Bounds: PVarArrayBoundArray; Dims : SizeInt; aVarType: TVarType): Variant;
  var
    p : pvararray;
  begin
    if not(VarTypeIsValidArrayType(aVarType)) then
      VarArrayCreateError;
    SysVarClear(Result);

    p:=SafeArrayCreate(aVarType,Dims,Bounds^);

    if not(assigned(p)) then
      VarArrayCreateError;

    TVarData(Result).vType:=aVarType or varArray;
    TVarData(Result).vArray:=p;
  end;

function VarArrayOf(const Values: array of Variant): Variant;
  var
    i : SizeInt;
  begin
    Result:=VarArrayCreate([0,high(Values)],varVariant);
    for i:=0 to high(Values) do
      Result[i]:=Values[i];
  end;


function VarArrayAsPSafeArray(const A: Variant): PVarArray;
  var
    v : TVarData;
  begin
    v:=TVarData(a);
    while v.vType=varByRef or varVariant do
      v:=TVarData(v.vPointer^);

    if (v.vType and varArray)=varArray then
      begin
        if (v.vType and varByRef)<>0 then
          Result:=pvararray(v.vPointer^)
        else
          Result:=v.vArray;
      end
    else
      VarResultCheck(VAR_INVALIDARG);
  end;


function VarArrayDimCount(const A: Variant) : LongInt;
  var
    hv : TVarData;
  begin
    hv:=TVarData(a);

    { get final Variant }
    while hv.vType=varByRef or varVariant do
      hv:=TVarData(hv.vPointer^);

    if (hv.vType and varArray)<>0 then
      Result:=hv.vArray^.DimCount
    else
      Result:=0;
  end;


function VarArrayLowBound(const A: Variant; Dim: LongInt) : LongInt;
  begin
    VarResultCheck(SafeArrayGetLBound(VarArrayAsPSafeArray(A),Dim,Result));
  end;


function VarArrayHighBound(const A: Variant; Dim: LongInt) : LongInt;
  begin
    VarResultCheck(SafeArrayGetUBound(VarArrayAsPSafeArray(A),Dim,Result));
  end;


function VarArrayLock(const A: Variant): Pointer;
  begin
    VarResultCheck(SafeArrayAccessData(VarArrayAsPSafeArray(A),Result));
  end;


procedure VarArrayUnlock(const A: Variant);
  begin
    VarResultCheck(SafeArrayUnaccessData(VarArrayAsPSafeArray(A)));
  end;


function VarArrayRef(const A: Variant): Variant;
  begin
    if (TVarData(a).vType and varArray)=0 then
      VarInvalidArgError(TVarData(a).vType);
    TVarData(Result).vType:=TVarData(a).vType or varByRef;
    if (TVarData(a).vType and varByRef)=0 then
      TVarData(Result).vPointer:=@TVarData(a).vArray
    else
      TVarData(Result).vPointer:=@TVarData(a).vPointer;
  end;


function VarIsArray(const A: Variant; AResolveByRef: Boolean): Boolean;
  var
    v : TVarData;
  begin
    v:=TVarData(a);
    if AResolveByRef then
      while v.vType=varByRef or varVariant do
        v:=TVarData(v.vPointer^);

    Result:=(v.vType and varArray)=varArray;
  end;


function VarIsArray(const A: Variant): Boolean;
  begin
    VarIsArray:=VarIsArray(A,true);
  end;


function VarTypeIsValidArrayType(const aVarType: TVarType): Boolean;
  begin
    Result:=aVarType in [varSmallInt,varInteger,
{$ifndef FPUNONE}
      varSingle,varDouble,varDate,
{$endif}
      varCurrency,varOleStr,varDispatch,varError,varBoolean,
      varVariant,varUnknown,varShortInt,varByte,varWord,varLongWord];
  end;


function VarTypeIsValidElementType(const aVarType: TVarType): Boolean;
  var
    customvarianttype : TCustomVariantType;
  begin
    Result:=((aVarType and not(varByRef) and not(varArray)) in [varEmpty,varNull,varSmallInt,varInteger,
{$ifndef FPUNONE}
      varSingle,varDouble,varDate,
{$endif}
      varCurrency,varOleStr,varDispatch,varError,varBoolean,
      varVariant,varUnknown,varShortInt,varByte,varWord,varLongWord,varInt64]) or
    FindCustomVariantType(aVarType,customvarianttype);
  end;


{ ---------------------------------------------------------------------
    Variant <-> Dynamic arrays support
  ---------------------------------------------------------------------}

function DynArrayGetVariantInfo(p : Pointer; var Dims : sizeint) : sizeint;
  begin
    Result:=varNull;
    { skip kind and name }
    p:=AlignTypeData(p+2+Length(PTypeInfo(p)^.Name));

    { search recursive? }
    if PTypeInfo(PTypeData(p)^.elType2)^.kind=tkDynArray then
      Result:=DynArrayGetVariantInfo(PTypeData(p)^.elType2,Dims)
    else
      Result:=PTypeData(p)^.varType;
    inc(Dims);
  end;

{$push}
{$r-}

procedure DynArrayToVariant(var V: Variant; const DynArray: Pointer; TypeInfo: Pointer);
  var
    i,
    Dims           : sizeint;
    vararrtype,
    dynarrvartype  : LongInt;
    vararraybounds : PVarArrayBoundArray;
    iter : TVariantArrayIterator;
    dynarriter : tdynarrayiter;
    p : Pointer;
    temp : Variant;
    dynarraybounds : tdynarraybounds;
  type
    TDynArray = array of Pointer;
  begin
    DoVarClear(TVarData(v));

    Dims:=0;
    dynarrvartype:=DynArrayGetVariantInfo(TypeInfo,Dims);

    vararrtype:=dynarrvartype;

    if (Dims>1) and not(DynamicArrayIsRectangular(DynArray,TypeInfo)) then
      exit;

    { retrieve Bounds array }
    Setlength(dynarraybounds,Dims);
    GetMem(vararraybounds,Dims*SizeOf(TVarArrayBound));
    try
      p:=DynArray;
      for i:=0 to Dims-1 do
        begin
          vararraybounds^[i].LowBound:=0;
          vararraybounds^[i].ElementCount:=length(TDynArray(p));
          dynarraybounds[i]:=length(TDynArray(p));
          if dynarraybounds[i]>0 then
            { we checked that the array is rectangular }
            p:=TDynArray(p)[0];
        end;
      { .. create Variant array }
      V:=VarArrayCreate(vararraybounds,Dims,vararrtype);

      VarArrayLock(V);
      try
        iter.init(Dims,PVarArrayBoundArray(vararraybounds));
        dynarriter.init(DynArray,TypeInfo,Dims,dynarraybounds);
        if not iter.AtEnd then
        repeat
          case vararrtype of
            varSmallInt:
              temp:=PSmallInt(dynarriter.data)^;
            varInteger:
              temp:=PInteger(dynarriter.data)^;
{$ifndef FPUNONE}
            varSingle:
              temp:=PSingle(dynarriter.data)^;
            varDouble:
              temp:=PDouble(dynarriter.data)^;
            varDate:
              temp:=PDouble(dynarriter.data)^;
{$endif}
            varCurrency:
              temp:=PCurrency(dynarriter.data)^;
            varOleStr:
              temp:=PWideString(dynarriter.data)^;
            varDispatch:
              temp:=PDispatch(dynarriter.data)^;
            varError:
              temp:=PError(dynarriter.data)^;
            varBoolean:
              temp:=PBoolean(dynarriter.data)^;
            varVariant:
              temp:=PVariant(dynarriter.data)^;
            varUnknown:
              temp:=PUnknown(dynarriter.data)^;
            varShortInt:
              temp:=PShortInt(dynarriter.data)^;
            varByte:
              temp:=PByte(dynarriter.data)^;
            varWord:
              temp:=PWord(dynarriter.data)^;
            varLongWord:
              temp:=PLongWord(dynarriter.data)^;
            varInt64:
              temp:=PInt64(dynarriter.data)^;
            varQWord:
              temp:=PQWord(dynarriter.data)^;
            else
              VarClear(temp);
          end;
          dynarriter.next;
          VarArrayPut(V,temp,Slice(iter.Coords^,Dims));
        until not(iter.next);
      finally
        iter.done;
        dynarriter.done;
        VarArrayUnlock(V);
      end;
    finally
      FreeMem(vararraybounds);
    end;
  end;


procedure DynArrayFromVariant(var DynArray: Pointer; const V: Variant; TypeInfo: Pointer);
  var
    DynArrayDims,
    VarArrayDims : SizeInt;
    iter : TVariantArrayIterator;
    dynarriter : tdynarrayiter;
    temp : Variant;
    dynarrvartype : LongInt;
    vararraybounds : PVarArrayBoundArray;
    dynarraybounds : tdynarraybounds;
    i : SizeInt;
  begin
    VarArrayDims:=VarArrayDimCount(V);

    DynArrayDims:=0;
    dynarrvartype:=DynArrayGetVariantInfo(TypeInfo,DynArrayDims);

    if (VarArrayDims=0) or (VarArrayDims<>DynArrayDims) then
      VarResultCheck(VAR_INVALIDARG);

    { retrieve Bounds array }
    Setlength(dynarraybounds,VarArrayDims);
    GetMem(vararraybounds,VarArrayDims*SizeOf(TVarArrayBound));
    try
      for i:=0 to VarArrayDims-1 do
        begin
          vararraybounds^[i].LowBound:=VarArrayLowBound(V,i+1);
          vararraybounds^[i].ElementCount:=VarArrayHighBound(V,i+1)-vararraybounds^[i].LowBound+1;
          dynarraybounds[i]:=vararraybounds^[i].ElementCount;
        end;
      DynArraySetLength(DynArray,TypeInfo,VarArrayDims,PSizeInt(dynarraybounds));
      VarArrayLock(V);
      try
        iter.init(VarArrayDims,PVarArrayBoundArray(vararraybounds));
        dynarriter.init(DynArray,TypeInfo,VarArrayDims,dynarraybounds);
        if not iter.AtEnd then
        repeat
          temp:=VarArrayGet(V,Slice(iter.Coords^,VarArrayDims));
          case dynarrvartype of
            varSmallInt:
              PSmallInt(dynarriter.data)^:=temp;
            varInteger:
              PInteger(dynarriter.data)^:=temp;
{$ifndef FPUNONE}
            varSingle:
              PSingle(dynarriter.data)^:=temp;
            varDouble:
              PDouble(dynarriter.data)^:=temp;
            varDate:
              PDouble(dynarriter.data)^:=temp;
{$endif}
            varCurrency:
              PCurrency(dynarriter.data)^:=temp;
            varOleStr:
              PWideString(dynarriter.data)^:=temp;
            varDispatch:
              PDispatch(dynarriter.data)^:=temp;
            varError:
              PError(dynarriter.data)^:=temp;
            varBoolean:
              PBoolean(dynarriter.data)^:=temp;
            varVariant:
              PVariant(dynarriter.data)^:=temp;
            varUnknown:
              PUnknown(dynarriter.data)^:=temp;
            varShortInt:
              PShortInt(dynarriter.data)^:=temp;
            varByte:
              PByte(dynarriter.data)^:=temp;
            varWord:
              PWord(dynarriter.data)^:=temp;
            varLongWord:
              PLongWord(dynarriter.data)^:=temp;
            varInt64:
              PInt64(dynarriter.data)^:=temp;
            varQWord:
              PQWord(dynarriter.data)^:=temp;
            else
              VarCastError;
          end;
          dynarriter.next;
        until not(iter.next);
      finally
        iter.done;
        dynarriter.done;
        VarArrayUnlock(V);
      end;
    finally
      FreeMem(vararraybounds);
    end;
  end;
{$pop}//{$r-} for DynArray[From|To]Variant


function FindCustomVariantType(const aVarType: TVarType; out CustomVariantType: TCustomVariantType): Boolean; overload;
  begin
    Result:=(aVarType>=CMinVarType);
    if Result then
      begin
{$ifdef FPC_HAS_FEATURE_THREADING}
        EnterCriticalSection(customvarianttypelock);
        try
{$endif}
          Result:=(aVarType-CMinVarType)<=high(customvarianttypes);
          if Result then
            begin
              CustomVariantType:=customvarianttypes[aVarType-CMinVarType];
              Result:=assigned(CustomVariantType) and
               (CustomVariantType<>InvalidCustomVariantType);
            end;
{$ifdef FPC_HAS_FEATURE_THREADING}
        finally
          LeaveCriticalSection(customvarianttypelock);
        end;
{$endif}
      end;
  end;


function FindCustomVariantType(const TypeName: string;  out CustomVariantType: TCustomVariantType): Boolean; overload;
  var
    i: Integer;
    tmp: TCustomVariantType;
    ShortTypeName: shortstring;
  begin
    ShortTypeName:=TypeName;  // avoid conversion in the loop
    result:=False;
{$ifdef FPC_HAS_FEATURE_THREADING}
    EnterCriticalSection(customvarianttypelock);
    try
{$endif}
      for i:=low(customvarianttypes) to high(customvarianttypes) do
        begin
          tmp:=customvarianttypes[i];
          result:=Assigned(tmp) and (tmp<>InvalidCustomVariantType) and
            tmp.ClassNameIs(ShortTypeName);
          if result then
            begin
              CustomVariantType:=tmp;
              Exit;
            end;
        end;
{$ifdef FPC_HAS_FEATURE_THREADING}
    finally
      LeaveCriticalSection(customvarianttypelock);
    end;
{$endif}
  end;

function Unassigned: Variant; // Unassigned standard constant
begin
  SysVarClear(Result);
  TVarData(Result).vType := varEmpty;
end;


function Null: Variant;       // Null standard constant
  begin
    SysVarClear(Result);
    TVarData(Result).vType := varNull;
  end;

procedure VarDispInvokeError;
  begin
    raise EVariantDispatchError.Create(SDispatchError);
  end;

{ ---------------------------------------------------------------------
    TCustomVariantType Class.
  ---------------------------------------------------------------------}

{ All TCustomVariantType descendants are singletons, they ignore automatic refcounting. }
function TCustomVariantType.QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} IID: TGUID; out Obj): HResult;  {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
  begin
    if GetInterface(IID, obj) then
      result := S_OK
    else
      result := E_NOINTERFACE;
  end;


function TCustomVariantType._AddRef: Longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
  begin
    result := -1;
  end;


function TCustomVariantType._Release: Longint; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
  begin
    result := -1;
  end;

{$warnings off}
procedure TCustomVariantType.SimplisticClear(var V: TVarData);
begin
  VarDataInit(V);
end;


procedure TCustomVariantType.SimplisticCopy(var Dest: TVarData; const Source: TVarData;  const Indirect: Boolean = False);
begin
  NotSupported('TCustomVariantType.SimplisticCopy');
end;


procedure TCustomVariantType.RaiseInvalidOp;
begin
  VarInvalidOp;
end;


procedure TCustomVariantType.RaiseCastError;
begin
  VarCastError;
end;


procedure TCustomVariantType.RaiseDispError;
begin
  VarDispInvokeError;
end;



function TCustomVariantType.LeftPromotion(const V: TVarData; const Operation: TVarOp; out RequiredVarType: TVarType): Boolean;

begin
  NotSupported('TCustomVariantType.LeftPromotion');
end;


function TCustomVariantType.RightPromotion(const V: TVarData; const Operation: TVarOp;  out RequiredVarType: TVarType): Boolean;

begin
  NotSupported('TCustomVariantType.RightPromotion');
end;


function TCustomVariantType.OlePromotion(const V: TVarData;  out RequiredVarType: TVarType): Boolean;

begin
  NotSupported('TCustomVariantType.OlePromotion');
end;


procedure TCustomVariantType.DispInvoke(Dest: PVarData; var Source: TVarData; CallDesc: PCallDesc; Params: Pointer);

begin
  RaiseDispError;
end;



procedure TCustomVariantType.VarDataInit(var Dest: TVarData);

begin
  FillChar(Dest,SizeOf(Dest),0);
end;


procedure TCustomVariantType.VarDataClear(var Dest: TVarData);

begin
  VarClearProc(Dest);
end;



procedure TCustomVariantType.VarDataCopy(var Dest: TVarData; const Source: TVarData);

begin
  DoVarCopy(Dest,Source)
end;


procedure TCustomVariantType.VarDataCopyNoInd(var Dest: TVarData; const Source: TVarData);

begin
  // This is probably not correct, but there is no DoVarCopyInd
  DoVarCopy(Dest,Source);
end;



procedure TCustomVariantType.VarDataCast(var Dest: TVarData; const Source: TVarData);

begin
  DoVarCast(Dest, Source, VarType);
end;


procedure TCustomVariantType.VarDataCastTo(var Dest: TVarData; const Source: TVarData; const aVarType: TVarType);

begin
  DoVarCast(Dest, Source, AVarType);
end;


procedure TCustomVariantType.VarDataCastTo(var Dest: TVarData; const aVarType: TVarType);

begin
  DoVarCast(Dest,Dest,AVarType);
end;


procedure TCustomVariantType.VarDataCastToOleStr(var Dest: TVarData);

begin
  VarDataCastTo(Dest, Dest, varOleStr);
end;



procedure TCustomVariantType.VarDataFromStr(var V: TVarData; const Value: string);

begin
  sysvarfromlstr(Variant(V),Value);
end;


procedure TCustomVariantType.VarDataFromOleStr(var V: TVarData; const Value: WideString);

begin
  sysvarfromwstr(variant(V),Value);
end;


function TCustomVariantType.VarDataToStr(const V: TVarData): string;

begin
  sysvartolstr(Result,Variant(V));
end;



function TCustomVariantType.VarDataIsEmptyParam(const V: TVarData): Boolean;

begin
  Result:=VarIsEmptyParam(Variant(V));
end;


function TCustomVariantType.VarDataIsByRef(const V: TVarData): Boolean;

begin
  Result:=(V.vType and varByRef)=varByRef;
end;


function TCustomVariantType.VarDataIsArray(const V: TVarData): Boolean;

begin
  Result:=(V.vType and varArray)=varArray;
end;



function TCustomVariantType.VarDataIsOrdinal(const V: TVarData): Boolean;

begin
  Result:=(V.vType and varTypeMask) in OrdinalVarTypes;
end;


function TCustomVariantType.VarDataIsFloat(const V: TVarData): Boolean;

begin
  Result:=(V.vType and varTypeMask) in FloatVarTypes;
end;


function TCustomVariantType.VarDataIsNumeric(const V: TVarData): Boolean;

begin
  Result:=(V.vType and varTypeMask) in (OrdinalVarTypes + FloatVarTypes);
end;


function TCustomVariantType.VarDataIsStr(const V: TVarData): Boolean;

begin
   Result:=
     ((V.vType and varTypeMask) = varOleStr) or
     ((V.vType and varTypeMask) = varString);
end;


procedure RegisterCustomVariantType(obj: TCustomVariantType; RequestedVarType: TVarType;
  UseFirstAvailable: Boolean);
var
  index,L: Integer;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(customvarianttypelock);
  try
{$endif}
    L:=Length(customvarianttypes);
    if UseFirstAvailable then
    begin
      repeat
        inc(customvariantcurrtype);
        if customvariantcurrtype>=CMaxVarType then
          raise EVariantError.Create(SVarTypeTooManyCustom);
      until ((customvariantcurrtype-CMinVarType)>=L) or
        (customvarianttypes[customvariantcurrtype-CMinVarType]=nil);
      RequestedVarType:=customvariantcurrtype;
    end
    else if (RequestedVarType<CFirstUserType) or (RequestedVarType>CMaxVarType) then
      raise EVariantError.CreateFmt(SVarTypeOutOfRangeWithPrefix, ['$', RequestedVarType]);

    index:=RequestedVarType-CMinVarType;
    if index>=L then
      SetLength(customvarianttypes,index+1);
    if Assigned(customvarianttypes[index]) then
    begin
      if customvarianttypes[index]=InvalidCustomVariantType then
        raise EVariantError.CreateFmt(SVarTypeNotUsableWithPrefix, ['$', RequestedVarType])
      else
        raise EVariantError.CreateFmt(SVarTypeAlreadyUsedWithPrefix,
          ['$', RequestedVarType, customvarianttypes[index].ClassName]);
    end;
    customvarianttypes[index]:=obj;
    obj.FVarType:=RequestedVarType;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(customvarianttypelock);
  end;
{$endif}
end;

constructor TCustomVariantType.Create;
begin
  RegisterCustomVariantType(Self,0,True);
end;

constructor TCustomVariantType.Create(RequestedVarType: TVarType);
begin
  RegisterCustomVariantType(Self,RequestedVarType,False);
end;


destructor TCustomVariantType.Destroy;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(customvarianttypelock);
  try
{$endif}
    if FVarType<>0 then
      customvarianttypes[FVarType-CMinVarType]:=InvalidCustomVariantType;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(customvarianttypelock);
  end;
{$endif}
  inherited Destroy;
end;



function TCustomVariantType.IsClear(const V: TVarData): Boolean;
begin
  result:=False;
end;


procedure TCustomVariantType.Cast(var Dest: TVarData; const Source: TVarData);

begin
  DoVarCast(Dest,Source,VarType);
end;


procedure TCustomVariantType.CastTo(var Dest: TVarData; const Source: TVarData; const aVarType: TVarType);

begin
  DoVarCast(Dest,Source,AVarType);
end;


procedure TCustomVariantType.CastToOle(var Dest: TVarData; const Source: TVarData);

begin
  NotSupported('TCustomVariantType.CastToOle');
end;



procedure TCustomVariantType.BinaryOp(var Left: TVarData; const Right: TVarData; const Operation: TVarOp);

begin
  RaiseInvalidOp;
end;


procedure TCustomVariantType.UnaryOp(var Right: TVarData; const Operation: TVarOp);

begin
  RaiseInvalidOp;
end;


function TCustomVariantType.CompareOp(const Left, Right: TVarData; const Operation: TVarOp): Boolean;

begin
  NotSupported('TCustomVariantType.CompareOp');
end;


procedure TCustomVariantType.Compare(const Left, Right: TVarData; var Relationship: TVarCompareResult);

begin
  NotSupported('TCustomVariantType.Compare');
end;
{$warnings on}

{ ---------------------------------------------------------------------
    TInvokeableVariantType implementation
  ---------------------------------------------------------------------}

procedure TInvokeableVariantType.DispInvoke(Dest: PVarData; var Source: TVarData;
  CallDesc: PCallDesc; Params: Pointer);
var
  method_name: ansistring;
  arg_count: byte;
  args: TVarDataArray;
  arg_idx: byte;
  arg_type: byte;
  arg_byref, has_result: boolean;
  arg_ptr: pointer;
  arg_data: PVarData;
  dummy_data: TVarData;
  arg_advanced: boolean;

const
  argtype_mask = $7F;
  argref_mask = $80;
begin
  arg_count := CallDesc^.ArgCount;
  method_name := ansistring(pchar(@CallDesc^.ArgTypes[arg_count]));
  setLength(args, arg_count);
  if arg_count > 0 then
  begin
    arg_ptr := Params;
    for arg_idx := 0 to arg_count - 1 do
    begin
      arg_type := CallDesc^.ArgTypes[arg_idx] and argtype_mask;
      arg_byref := (CallDesc^.ArgTypes[arg_idx] and argref_mask) <> 0;
      arg_data := @args[arg_count - arg_idx - 1];
      case arg_type of
        varUStrArg: arg_data^.vType := varUString;
        varStrArg: arg_data^.vType := varString;
      else
        arg_data^.vType := arg_type
      end;
      if arg_byref then
      begin
        arg_data^.vType := arg_data^.vType or varByRef;
        arg_data^.vPointer := PPointer(arg_ptr)^;
        Inc(arg_ptr,sizeof(Pointer));
      end
      else
        begin
          arg_advanced:=false;
          case arg_type of
            varError:
              begin
                arg_data^.vError:=VAR_PARAMNOTFOUND;
                arg_advanced := true;
              end;
            varVariant:
              arg_data^ := PVarData(PPointer(arg_ptr)^)^;
            varDouble, varCurrency, varDate, varInt64, varQWord:
              begin
                arg_data^.vQWord := PQWord(arg_ptr)^; // 64bit on all platforms
                inc(arg_ptr,sizeof(QWord));
                arg_advanced := true;
              end;
            { values potentially smaller than sizeof(pointer) must be handled
              explicitly to guarantee endian safety and to prevent copying/
              skipping data (they are always copied into a 4 byte element
              by the compiler, although it will still skip sizeof(pointer)
              bytes afterwards) }
            varSingle:
              arg_data^.vSingle := PSingle(arg_ptr)^;
            varSmallint:
              arg_data^.vSmallInt := PLongint(arg_ptr)^;
            varInteger:
              arg_data^.vInteger := PLongint(arg_ptr)^;
            varBoolean:
              arg_data^.vBoolean := WordBool(PLongint(arg_ptr)^);
            varShortInt:
              arg_data^.vShortInt := PLongint(arg_ptr)^;
            varByte:
              arg_data^.vByte := PLongint(arg_ptr)^;
            varWord:
              arg_data^.vWord := PLongint(arg_ptr)^;
            else
              arg_data^.vAny := PPointer(arg_ptr)^; // 32 or 64bit
          end;
          if not arg_advanced then
            inc(arg_ptr,sizeof(pointer));
        end;
    end;
  end;
  has_result := (Dest <> nil);
  if has_result then
    variant(Dest^) := Unassigned;
  case CallDesc^.CallType of

    1:     { DISPATCH_METHOD }
      if has_result then
      begin
        if arg_count = 0 then
        begin
          // no args -- try GetProperty first, then DoFunction
          if not (GetProperty(Dest^,Source,method_name) or
            DoFunction(Dest^,Source,method_name,args)) then
            RaiseDispError
        end
        else
          if not DoFunction(Dest^,Source,method_name,args) then
            RaiseDispError;
      end
      else
      begin
        // may be procedure?
        if not DoProcedure(Source,method_name,args) then
        // may be function?
        try
          dummy_data.VType := varEmpty;
          if not DoFunction(dummy_data,Source,method_name,args) then
            RaiseDispError;
        finally
          VarDataClear(dummy_data)
        end;
      end;

    2:     { DISPATCH_PROPERTYGET -- currently never generated by compiler for Variant Dispatch }
      if has_result then
      begin
        // must be property...
        if not GetProperty(Dest^,Source,method_name) then
          // may be function?
          if not DoFunction(Dest^,Source,method_name,args) then
            RaiseDispError
      end
      else
        RaiseDispError;

    4:    { DISPATCH_PROPERTYPUT }
      if has_result or (arg_count<>1) or  // must be no result and a single arg
        (not SetProperty(Source,method_name,args[0])) then
        RaiseDispError;
  else
    RaiseDispError;
  end;
end;

function TInvokeableVariantType.DoFunction(var Dest: TVarData; const V: TVarData; const Name: string; const Arguments: TVarDataArray): Boolean;

begin
  result := False;
end;

function TInvokeableVariantType.DoProcedure(const V: TVarData; const Name: string; const Arguments: TVarDataArray): Boolean;
begin
  result := False
end;


function TInvokeableVariantType.GetProperty(var Dest: TVarData; const V: TVarData; const Name: string): Boolean;
  begin
    result := False;
  end;


function TInvokeableVariantType.SetProperty(var V: TVarData; const Name: string; const Value: TVarData): Boolean;
  begin
    result := False;
  end;


{ ---------------------------------------------------------------------
    TPublishableVariantType implementation
  ---------------------------------------------------------------------}

function TPublishableVariantType.GetProperty(var Dest: TVarData; const V: TVarData; const Name: string): Boolean;
  begin
    Result:=true;
    Variant(Dest):=TypInfo.GetPropValue(getinstance(v),name);
  end;


function TPublishableVariantType.SetProperty(var V: TVarData; const Name: string; const Value: TVarData): Boolean;
  begin
    Result:=true;
    TypInfo.SetPropValue(getinstance(v),name,Variant(value));
  end;


procedure VarCastError;
  begin
    raise EVariantTypeCastError.Create(SInvalidVarCast);
  end;


procedure VarCastError(const ASourceType, ADestType: TVarType);
  begin
    raise EVariantTypeCastError.CreateFmt(SVarTypeCouldNotConvert,
      [VarTypeAsText(ASourceType),VarTypeAsText(ADestType)]);
  end;


procedure VarCastErrorOle(const ASourceType: TVarType);
  begin
    raise EVariantTypeCastError.CreateFmt(SVarTypeCouldNotConvert,
      [VarTypeAsText(ASourceType),'(OleVariant)']);
  end;


procedure VarInvalidOp;
  begin
    raise EVariantInvalidOpError.Create(SInvalidVarOp);
  end;

procedure VarInvalidOp(const aLeft, aRight: TVarType; aOpCode: TVarOp);
  begin
    raise EVariantInvalidOpError.CreateFmt(SInvalidBinaryVarOp,
      [VarTypeAsText(aLeft),VarOpAsText[aOpCode],VarTypeAsText(aRight)]);
  end;


procedure VarInvalidOp(const aRight: TVarType; aOpCode: TVarOp);
  begin
    raise EVariantInvalidOpError.CreateFmt(SInvalidUnaryVarOp,
      [VarOpAsText[aOpCode],VarTypeAsText(aRight)]);
  end;


procedure VarInvalidNullOp;
  begin
    raise EVariantInvalidOpError.Create(SInvalidvarNullOp);
  end;


procedure VarParamNotFoundError;
  begin
    raise EVariantParamNotFoundError.Create(SVarParamNotFound);
  end;


procedure VarBadTypeError;
  begin
    raise EVariantBadVarTypeError.Create(SVarBadType);
  end;


procedure VarOverflowError;
  begin
    raise EVariantOverflowError.Create(SVarOverflow);
  end;


procedure VarOverflowError(const ASourceType, ADestType: TVarType);
  begin
    raise EVariantOverflowError.CreateFmt(SVarTypeConvertOverflow,
      [VarTypeAsText(ASourceType),VarTypeAsText(ADestType)]);
  end;


procedure VarRangeCheckError(const AType: TVarType);
  begin
    raise EVariantOverflowError.CreateFmt(SVarTypeRangeCheck1,
      [VarTypeAsText(AType)])
  end;


procedure VarRangeCheckError(const ASourceType, ADestType: TVarType);
  begin
    if ASourceType<>ADestType then
      raise EVariantOverflowError.CreateFmt(SVarTypeRangeCheck2,
        [VarTypeAsText(ASourceType),VarTypeAsText(ADestType)])
    else
      VarRangeCheckError(ASourceType);
  end;


procedure VarBadIndexError;
  begin
    raise EVariantBadIndexError.Create(SVarArrayBounds);
  end;


procedure VarArrayLockedError;
  begin
    raise EVariantArrayLockedError.Create(SVarArrayLocked);
  end;


procedure VarNotImplError;
  begin
    raise EVariantNotImplError.Create(SVarNotImplemented);
  end;


procedure VarOutOfMemoryError;
  begin
    raise EVariantOutOfMemoryError.Create(SOutOfMemory);
  end;


procedure VarInvalidArgError;
  begin
    raise EVariantInvalidArgError.Create(SVarInvalid);
  end;


procedure VarInvalidArgError(AType: TVarType);
  begin
    raise EVariantInvalidArgError.CreateFmt(SVarInvalid1,
      [VarTypeAsText(AType)])
  end;


procedure VarUnexpectedError;
  begin
    raise EVariantUnexpectedError.Create(SVarUnexpected);
  end;


procedure VarArrayCreateError;
  begin
    raise EVariantArrayCreateError.Create(SVarArrayCreate);
  end;


procedure RaiseVarException(res : HRESULT);
  begin
    case res of
      VAR_PARAMNOTFOUND:
        VarParamNotFoundError;
      VAR_TYPEMISMATCH:
        VarCastError;
      VAR_BADVARTYPE:
        VarBadTypeError;
      VAR_EXCEPTION:
        VarInvalidOp;
      VAR_OVERFLOW:
        VarOverflowError;
      VAR_BADINDEX:
        VarBadIndexError;
      VAR_ARRAYISLOCKED:
        VarArrayLockedError;
      VAR_NOTIMPL:
        VarNotImplError;
      VAR_OUTOFMEMORY:
        VarOutOfMemoryError;
      VAR_INVALIDARG:
        VarInvalidArgError;
      VAR_UNEXPECTED:
        VarUnexpectedError;
      else
        raise EVariantError.CreateFmt(SInvalidVarOpWithHResultWithPrefix,
          ['$',res,'']);
    end;
  end;


procedure VarResultCheck(AResult: HRESULT);{$IFDEF VARIANTINLINE}inline;{$ENDIF VARIANTINLINE}
  begin
    if AResult<>VAR_OK then
      RaiseVarException(AResult);
  end;


procedure VarResultCheck(AResult: HRESULT; ASourceType, ADestType: TVarType);
  begin
    case AResult of
      VAR_OK:
        ;
      VAR_OVERFLOW:
        VarOverflowError(ASourceType,ADestType);
      VAR_TYPEMISMATCH:
        VarCastError(ASourceType,ADestType);
    else
      RaiseVarException(AResult);
    end;
  end;


procedure HandleConversionException(const ASourceType, ADestType: TVarType);
  begin
    if exceptobject is econverterror then
      VarCastError(asourcetype,adesttype)
    else if (exceptobject is eoverflow) or
      (exceptobject is erangeerror) then
      varoverflowerror(asourcetype,adesttype)
    else
      raise exception(acquireexceptionobject);
  end;


function VarTypeAsText(const AType: TVarType): string;
  var
    customvarianttype : TCustomVariantType;
  const
    names : array[varEmpty..varQWord] of string[8] = (
    'Empty','Null','Smallint','Integer','Single','Double','Currency','Date','OleStr','Dispatch','Error','Boolean','Variant',
    'Unknown','Decimal','???','ShortInt','Byte','Word','DWord','Int64','QWord');
  begin
    if ((AType and varTypeMask)>=low(names)) and ((AType and varTypeMask)<=high(names)) then
      Result:=names[AType and varTypeMask]
    else
      case AType and varTypeMask of
        varString:
          Result:='String';
        varAny:
          Result:='Any';
        else
          begin
            if FindCustomVariantType(AType and varTypeMask,customvarianttype) then
              Result:=customvarianttype.classname
            else
              Result:='$'+IntToHex(AType and varTypeMask,4)
          end;
      end;
    if (AType and vararray)<>0 then
      Result:='Array of '+Result;
    if (AType and varByRef)<>0 then
      Result:='Ref to '+Result;
  end;


function FindVarData(const V: Variant): PVarData;
  begin
    Result:=PVarData(@V);
    while Result^.vType=varVariant or varByRef do
      Result:=PVarData(Result^.vPointer);
  end;

{ ---------------------------------------------------------------------
    Variant properties from typinfo
  ---------------------------------------------------------------------}

function GetVariantProp(Instance : TObject;PropInfo : PPropInfo) : Variant;
type
  TGetVariantProc = function:Variant of object;
  TGetVariantProcIndex = function(Index: integer): Variant of object;
var
  AMethod : TMethod;
begin
  Result:=Null;
  case PropInfo^.PropProcs and 3 of
    ptField:
      Result:=PVariant(Pointer(Instance)+PtrUInt(PropInfo^.GetProc))^;
    ptStatic,
    ptVirtual:
      begin
        if (PropInfo^.PropProcs and 3)=ptStatic then
          AMethod.Code:=PropInfo^.GetProc
        else
          AMethod.Code:=PCodePointer(Pointer(Instance.ClassType)+PtrUInt(PropInfo^.GetProc))^;
        AMethod.Data:=Instance;

        if ((PropInfo^.PropProcs shr 6) and 1)=0 then
          Result:=TGetVariantProc(AMethod)()
        else
          Result:=TGetVariantProcIndex(AMethod)(PropInfo^.Index);
      end;
  end;
end;


Procedure SetVariantProp(Instance : TObject;PropInfo : PPropInfo; const Value : Variant);
type
  TSetVariantProc = procedure(const AValue: Variant) of object;
  TSetVariantProcIndex = procedure(Index: integer; AValue: Variant) of object;
Var
  AMethod : TMethod;
begin
  case (PropInfo^.PropProcs shr 2) and 3 of
    ptfield:
      PVariant(Pointer(Instance)+PtrUInt(PropInfo^.SetProc))^:=Value;
    ptVirtual,ptStatic:
      begin
        if ((PropInfo^.PropProcs shr 2) and 3)=ptStatic then
          AMethod.Code:=PropInfo^.SetProc
        else
          AMethod.Code:=PCodePointer(Pointer(Instance.ClassType)+PtrUInt(PropInfo^.SetProc))^;
        AMethod.Data:=Instance;

	      if ((PropInfo^.PropProcs shr 6) and 1)=0 then
          TSetVariantProc(AMethod)(Value)
        else
          TSetVariantProcIndex(AMethod)(PropInfo^.Index,Value);
      end;
  end;
end;


Function GetVariantProp(Instance: TObject; const PropName: string): Variant;
begin
  Result:=GetVariantProp(Instance,FindPropInfo(Instance,PropName));
end;


Procedure SetVariantProp(Instance: TObject; const PropName: string;  const Value: Variant);
begin
  SetVariantprop(instance,FindpropInfo(Instance,PropName),Value);
end;

{ ---------------------------------------------------------------------
  All properties through Variant.
  ---------------------------------------------------------------------}

Function GetPropValue(Instance: TObject; const PropName: string): Variant;
begin
  Result:=TypInfo.GetPropValue(Instance,PropName,True);
end;


Function GetPropValue(Instance: TObject; PropInfo: PPropInfo; PreferStrings: Boolean): Variant;

begin
  Result := Null; //at worst
  // call the Right GetxxxProp
  case PropInfo^.PropType^.Kind of
    tkInteger, tkChar, tkWChar, tkClass, tkBool:
      Result := GetOrdProp(Instance, PropInfo);
    tkEnumeration:
      if PreferStrings then
        Result := GetEnumProp(Instance, PropInfo)
      else
        Result := GetOrdProp(Instance, PropInfo);
    tkSet:
      if PreferStrings then
        Result := GetSetProp(Instance, PropInfo, False)
      else
        Result := GetOrdProp(Instance, PropInfo);
{$ifndef FPUNONE}
    tkFloat:
      Result := GetFloatProp(Instance, PropInfo);
{$endif}
    tkMethod:
      Result := PropInfo^.PropType^.Name;
    tkString, tkLString, tkAString:
      Result := GetStrProp(Instance, PropInfo);
    tkWString:
      Result := GetWideStrProp(Instance, PropInfo);
    tkUString:
      Result := GetUnicodeStrProp(Instance, PropInfo);
    tkVariant:
      Result := GetVariantProp(Instance, PropInfo);
    tkInt64:
      Result := GetInt64Prop(Instance, PropInfo);
    tkQWord:
      Result := QWord(GetInt64Prop(Instance, PropInfo));
    tkDynArray:
      DynArrayToVariant(Result,GetDynArrayProp(Instance, PropInfo), PropInfo^.PropType);
    else
      raise EPropertyConvertError.CreateFmt('Invalid Property Type: %s',[PropInfo^.PropType^.Name]);
  end;
end;

Procedure SetPropValue(Instance: TObject; PropInfo: PPropInfo;  const Value: Variant);

var
 TypeData: PTypeData;
 O: Integer;
 I64: Int64;
 Qw: QWord;
 S: String;
 B: Boolean;
 dynarr: Pointer;

begin
   TypeData := GetTypeData(PropInfo^.PropType);
   // call Right SetxxxProp
   case PropInfo^.PropType^.Kind of
     tkBool:
       begin
       { to support the strings 'true' and 'false' }
       if (VarType(Value)=varOleStr) or
          (VarType(Value)=varString) or
          (VarType(Value)=varBoolean) then
         begin
           B:=Value;
           SetOrdProp(Instance, PropInfo, ord(B));
         end
       else
         begin
           I64:=Value;
           if (I64<TypeData^.MinValue) or (I64>TypeData^.MaxValue) then
             raise ERangeError.Create(SRangeError);
           SetOrdProp(Instance, PropInfo, I64);
         end;
       end;
     tkInteger, tkChar, tkWChar:
       begin
       I64:=Value;
       if (TypeData^.OrdType=otULong) then
         if (I64<LongWord(TypeData^.MinValue)) or (I64>LongWord(TypeData^.MaxValue)) then
           raise ERangeError.Create(SRangeError)
         else
       else
       if (I64<TypeData^.MinValue) or (I64>TypeData^.MaxValue) then
         raise ERangeError.Create(SRangeError);
       SetOrdProp(Instance, PropInfo, I64);
       end;
     tkEnumeration :
       begin
       if (VarType(Value)=varOleStr) or (VarType(Value)=varString) then
         begin
         S:=Value;
         SetEnumProp(Instance,PropInfo,S);
         end
       else
         begin
         I64:=Value;
         if (I64<TypeData^.MinValue) or (I64>TypeData^.MaxValue) then
           raise ERangeError.Create(SRangeError);
         SetOrdProp(Instance, PropInfo, I64);
         end;
       end;
     tkSet :
       begin
       if (VarType(Value)=varOleStr) or (VarType(Value)=varString) then
         begin
         S:=Value;
         SetSetProp(Instance,PropInfo,S);
         end
       else
         begin
         O:=Value;
         SetOrdProp(Instance, PropInfo, O);
         end;
       end;
{$ifndef FPUNONE}
     tkFloat:
       SetFloatProp(Instance, PropInfo, Value);
{$endif}
     tkString, tkLString, tkAString:
       SetStrProp(Instance, PropInfo, VarToStr(Value));
     tkWString:
       SetWideStrProp(Instance, PropInfo, VarToWideStr(Value));
     tkUString:
       SetUnicodeStrProp(Instance, PropInfo, VarToUnicodeStr(Value));
     tkVariant:
       SetVariantProp(Instance, PropInfo, Value);
     tkInt64:
       begin
         I64:=Value;
         if (I64<TypeData^.MinInt64Value) or (I64>TypeData^.MaxInt64Value) then
           raise ERangeError.Create(SRangeError);
         SetInt64Prop(Instance, PropInfo, I64);
       end;
     tkQWord:
       begin
         Qw:=Value;
         if (Qw<TypeData^.MinQWordValue) or (Qw>TypeData^.MaxQWordValue) then
           raise ERangeError.Create(SRangeError);
         SetInt64Prop(Instance, PropInfo,Qw);
       end;
     tkDynArray:
       begin
         dynarr:=Nil;
         DynArrayFromVariant(dynarr, Value, PropInfo^.PropType);
         SetDynArrayProp(Instance, PropInfo, dynarr);
       end;
   else
     raise EPropertyConvertError.CreateFmt('SetPropValue: Invalid Property Type %s',
                                    [PropInfo^.PropType^.Name]);
   end;
end;

var
  i : LongInt;

Initialization
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitCriticalSection(customvarianttypelock);
{$endif}
  // start with one-less value, so first increment yields CFirstUserType
  customvariantcurrtype:=CFirstUserType-1;
  SetSysVariantManager;
  SetClearVarToEmptyParam(TVarData(EmptyParam));
  VarClearProc:=@DoVarClear;
  VarAddRefProc:=@DoVarAddRef;
  VarCopyProc:=@DoVarCopy;
  // Typinfo Variant support
  OnGetVariantProp:=@GetVariantprop;
  OnSetVariantProp:=@SetVariantprop;
  OnSetPropValue:=@SetPropValue;
  OnGetPropValue:=@GetPropValue;
  InvalidCustomVariantType:=TCustomVariantType(-1);
  SetLength(customvarianttypes,CFirstUserType);
Finalization
{$ifdef FPC_HAS_FEATURE_THREADING}
  EnterCriticalSection(customvarianttypelock);
  try
{$endif}
    for i:=0 to high(customvarianttypes) do
      if customvarianttypes[i]<>InvalidCustomVariantType then
        customvarianttypes[i].Free;
{$ifdef FPC_HAS_FEATURE_THREADING}
  finally
    LeaveCriticalSection(customvarianttypelock);
  end;
{$endif}
  UnSetSysVariantManager;
{$ifdef FPC_HAS_FEATURE_THREADING}
  DoneCriticalSection(customvarianttypelock);
{$endif}
end.
