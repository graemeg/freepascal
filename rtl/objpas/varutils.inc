{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2001 by the Free Pascal development team

    Variant routines for non-windows oses.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ---------------------------------------------------------------------
    Some general stuff: Error handling and so on.
  ---------------------------------------------------------------------}

{ we do ugly things with tvararray here }
{$RANGECHECKS OFF}

Procedure SetUnlockResult (P : PVarArray; Res : HResult);

begin
  If Res=VAR_OK then
    Res:=SafeArrayUnlock(P)
  else
    SafeArrayUnlock(P);
end;

Procedure MakeWideString (Var P : PWideChar; W : WideString);

begin
  P:=PWideChar(W);
end;

Procedure CopyAsWideString (Var PDest : PWideChar; PSource : PWideChar);

begin
  WideString(Pointer(PDest)):=WideString(Pointer(PSource));
end;

{ ---------------------------------------------------------------------
    Basic variant handling.
  ---------------------------------------------------------------------}

procedure VariantInit(var Varg: TVarData); stdcall;
begin
  With Varg do
    begin
      FillChar(VBytes, SizeOf(VBytes), 0);
      VType:=varEmpty;
    end;
end;

function VariantClear(var Varg: TVarData): HRESULT;stdcall;
begin
  With Varg do
    if (VType and varArray)=varArray then
      begin
        Result:=SafeArrayDestroy(VArray);
        if Result<>VAR_OK then
          exit;
      end
    else
      begin
      if (VType and varByRef) = 0 then
        case VType of
          varEmpty, varNull, varSmallint, varInteger, varWord,
{$ifndef FPUNONE}
          varSingle, varDouble, varCurrency, varDate,
{$endif}
          varError, varBoolean, varByte,VarShortInt,
          varInt64, VarLongWord,VarQWord:
            ;
          varOleStr:
            WideString(Pointer(VOleStr)):='';
          varDispatch,
          varUnknown:
            iinterface(vunknown):=nil;
        else
          exit(VAR_BADVARTYPE)
        end;
    end;
  VariantInit(Varg);
  Result:=VAR_OK;
end;

function VariantCopy(var VargDest: TVarData; const VargSrc: TVarData): HRESULT;stdcall;
begin
  if @VargSrc = @VargDest then
    Exit(VAR_OK);
  Result:=VariantClear(VargDest);
  if Result<>VAR_OK then
    exit;
  With VargSrc do
    begin
    if (VType and varArray) <> 0 then
      Result:=SafeArrayCopy(VArray,VargDest.VArray)
    else
      begin
      if (VType and varByRef) <> 0 then
        VArgDest.VPointer:=VPointer
      else
        case (VType and varTypeMask) of
          varEmpty, varNull:;
          varSmallint, varInteger, varWord,
{$ifndef FPUNONE}
          varSingle, varDouble, varCurrency, varDate,
{$endif}
          varError, varBoolean, varByte,VarShortInt,
          varInt64, VarLongWord,VarQWord:
            Move(VBytes, VargDest.VBytes, SizeOf(VargDest.VBytes));
          varOleStr:
            CopyAsWideString(VargDest.VOleStr,VOleStr);
          varDispatch:
            IUnknown(VargDest.vdispatch):=IUnknown(VargSrc.vdispatch);
          varUnknown:
            IUnknown(VargDest.vunknown):=IUnknown(VargSrc.vunknown);
          else
            Exit(VAR_BADVARTYPE);
          end;
        end;
        VargDest.VType:=VType;
    end;
end;

function VariantCopyInd(var VargDest: TVarData; const VargSrc: TVarData): HRESULT;stdcall;

begin
  if (VargSrc.VType and varByRef) = 0 then
    Exit(VariantCopy(VargDest, VargSrc));
  With VargSrc do
    begin
    if (VType and varArray) <> 0 then
      Exit(VAR_INVALIDARG);
    case (VType and varTypeMask) of
      varEmpty, varNull:;
      varSmallint : VargDest.VSmallInt:=PSmallInt(VPointer)^;
      varInteger  : VargDest.VInteger:=PLongint(VPointer)^;
{$ifndef FPUNONE}
      varSingle   : VargDest.VSingle:=PSingle(VPointer)^;
      varDouble   : VargDest.VDouble:=PDouble(VPointer)^;
      varCurrency : VargDest.VCurrency:=PCurrency(VPointer)^;
      varDate     : VargDest.VDate:=PDate(VPointer)^;
{$endif}
      varBoolean  : VargDest.VBoolean:=PWordBool(VPointer)^;
      varError    : VargDest.VError:=PError(VPointer)^;
      varByte     : VargDest.VByte:=PByte(VPointer)^;
      varWord     : VargDest.VWord:=PWord(VPointer)^;
      VarShortInt : VargDest.VShortInt:=PShortInt(VPointer)^;
      VarInt64    : VargDest.VInt64:=PInt64(VPointer)^;
      VarLongWord : VargDest.VLongWord:=PCardinal(VPointer)^;
      VarQWord    : VargDest.VQWord:=PQWord(VPointer)^;
      varVariant  : Variant(VargDest):=Variant(PVarData(VPointer)^);
      varOleStr   : CopyAsWideString(VargDest.VOleStr,PVarData(VPointer)^.VoleStr);
      varDispatch,
      varUnknown  : IInterface(VargDest.vUnknown):=IInterface(PInterface(VargSrc.VPointer)^);
      else
        Exit(VAR_BADVARTYPE);
      end;
    VargDest.VType:=VType and VarTypeMask;
    end;
  Result:=VAR_OK;
end;

Function VariantChangeTypeEx(var VargDest: TVarData; const VargSrc: TVarData;
  LCID: Integer; Flags: Word; VarType: Word): HRESULT; stdcall;
var
  Tmp : TVarData;
begin
  if ((VarType and varArray) <> 0) or
     ((VargSrc.VType and varArray) <> 0) or
     ((VarType and varByRef) <> 0) then
    Exit(VAR_INVALIDARG);
  Result:=VariantCopyInd(Tmp, VargSrc);
  if Result = VAR_OK then
    try
    Result:=VariantClear(VargDest);
    {$RANGECHECKS ON}
    if Result = VAR_OK then
      try
        case Vartype of
          varSmallInt : VargDest.VSmallInt:=VariantToSmallInt(Tmp);
          varInteger  : VargDest.VInteger:=VariantToLongint(Tmp);
{$ifndef FPUNONE}
          varSingle   : VargDest.VSingle:=VariantToSingle(Tmp);
          varDouble   : VargDest.VDouble:=VariantToDouble(Tmp);
          varCurrency : VargDest.VCurrency:=VariantToCurrency(Tmp);
          varDate     : VargDest.VDate:=VariantToDate(tmp);
{$endif}
          varOleStr   : MakeWideString(VargDest.VoleStr, VariantToWideString(tmp));
          varDispatch : Result:=VAR_TYPEMISMATCH;
          varUnknown  : Result:=VAR_TYPEMISMATCH;
          varBoolean  : VargDest.VBoolean:=VariantToBoolean(Tmp);
          varByte     : VargDest.VByte:=VariantToByte(Tmp);
          VarShortInt : VargDest.VShortInt:=VariantToShortInt(Tmp);
          VarInt64    : VargDest.Vint64:=VariantToInt64(Tmp);
          VarLongWord : VargDest.VLongWord:=VariantToCardinal(Tmp);
          VarQWord    : VargDest.VQWord:=VariantToQword(tmp);
       else
          Result:=VAR_BADVARTYPE;
       end;
       If Result = VAR_OK then
         VargDest.VType:=VarType;
      except
        On E : EVariantError do
          Result:=E.ErrCode;
        else
          Result:=VAR_INVALIDARG;
      end;
    finally
      VariantClear(Tmp);
    end;
  {$RANGECHECKS OFF}
end;

{ ---------------------------------------------------------------------
    Variant array support
  ---------------------------------------------------------------------}

Function CheckArrayUnlocked (psa : PVarArray) : HResult;

begin
  If psa^.LockCount = 0 Then
    Result:=VAR_OK
  else
    Result:=VAR_ARRAYISLOCKED;
end;

Function CheckVarArray(psa: PVarArray ): HRESULT;

begin
  If psa=nil then
    Result:=VAR_INVALIDARG
  else
    Result:=VAR_OK;
end;

Function SafeArrayCalculateElementAddress(psa: PVarArray; aElement: SizeInt): Pointer;

begin
  Result:=Pointer(psa^.Data)+(aElement*psa^.ElementSize);
end;

Function CheckVarArrayAndCalculateAddress(psa: PVarArray;
  Indices: PVarArrayCoorArray; var Address: Pointer; Lockit: Boolean): HRESULT;
var
  I,D,Count,Idx : LongInt;

begin
  Result:=CheckVarArray(psa);
  Address:=nil;
  Count:=0;
  If Result<>VAR_OK then
    exit;
  D:=0;
  for I:=0 to psa^.DimCount-1 do
    begin
      Idx:=Indices^[psa^.DimCount-I-1] - psa^.Bounds[I].LowBound;
      if (Idx<0) or (Idx>=psa^.Bounds[I].ElementCount) then
        Exit(VAR_BADINDEX);
      if I=0 then
        Count:=Idx
      else
        Inc(Count,Idx*D);
      Inc(D,psa^.Bounds[I].ElementCount);
    end;
  Address:=SafeArrayCalculateElementAddress(psa, Count);
  if LockIt then
    Result:=SafeArrayLock(psa);
end;

Function SafeArrayElementTotal(psa: PVarArray): Integer;

var
  I: Integer;

begin
  Result:=1;
  With psa^ do
   for I:=0 to DimCount - 1 do
     Result:=Result*Bounds[I].ElementCount;
end;

type
  TVariantArrayType = (vatNormal, vatInterface, vatWideString, vatVariant);

Function VariantArrayType(psa: PVarArray): TVariantArrayType;

begin
  if ((psa^.Flags and ARR_DISPATCH) <> 0) or
     ((psa^.Flags and ARR_UNKNOWN) <> 0) then
    Result:=vatInterface
  else if (psa^.Flags AND ARR_OLESTR) <> 0 then
    Result:=vatWideString
  else if (psa^.Flags and ARR_VARIANT) <> 0 then
    Result := vatVariant
  else
    Result:=vatNormal;
end;

Function SafeArrayClearDataSpace(psa: PVarArray; WipeBytes: Boolean {= True}): HRESULT;

var
  vat: TVariantArrayType;
  P : Pointer;
  J,Count : Integer;
begin
  try
    count:=SafeArrayElementTotal(psa);
    vat:=VariantArrayType(psa);
    case vat of
      vatNormal : FillChar(psa^.Data^,Count*psa^.ElementSize,0);
      vatInterface :
         for j := 0 to Count - 1 do
           begin
           P := SafeArrayCalculateElementAddress(psa,j);
           IUnknown(PUnknown(P)^):=Nil
           end;
      vatWideString :
         for j := 0 to Count - 1 do
           begin
           P := SafeArrayCalculateElementAddress(psa,j);
           WideString(PPointer(P)^):='';
           end;
      vatVariant :
         for j := 0 to Count - 1 do
           begin
           P := SafeArrayCalculateElementAddress(psa,j);
           VariantClear(PVarData(P)^);
           end;
    end;
    Result:=VAR_OK;
  except
    On E : Exception do
      Result:=ExceptionToVariantError (E);
  end;
end;

Function SafeArrayCopyDataSpace(psa, psaOut: PVarArray): HRESULT;
var
  vat: TVariantArrayType;
  P1,P2 : Pointer;
  J,Count : Integer;
begin
  try
    Count:=SafeArrayElementTotal(psa);
    vat:=VariantArrayType(psa);
    case vat of
      vatNormal: Move(psa^.Data^,psaOut^.Data^,Count*psa^.ElementSize);
      vatInterface :
         for j := 0 to Count - 1 do
           begin
           P1 := SafeArrayCalculateElementAddress(psa,j);
           P2 := SafeArrayCalculateElementAddress(psaout,j);
           IUnknown(PUnknown(P2)^):=IUnknown(PUnknown(P1)^);
           end;
      vatWideString :
         for j := 0 to Count - 1 do
           begin
           P1 := SafeArrayCalculateElementAddress(psa,j);
           P2 := SafeArrayCalculateElementAddress(psaOut,j);
           WideString(PPointer(P2)^):=WideString(PPointer(P1)^);
           end;
      vatVariant :
         for j := 0 to Count - 1 do
           begin
           P1 := SafeArrayCalculateElementAddress(psa,j);
           P2 := SafeArrayCalculateElementAddress(psaOut,j);
           VariantCopy(PVarData(P2)^,PVarData(P2)^);
           end;
    end;
    Result:=VAR_OK;
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E);
  end;
end;

Const
  Supportedpsas = [varSmallint,varInteger,
{$ifndef FPUNONE}
     varSingle,varDouble,varCurrency,varDate,
{$endif}
     varOleStr,varDispatch,varError,varBoolean,varVariant,varUnknown,varShortInt,varByte,
     varWord,varLongWord,varInt64,varQWord];
  psaElementFlags : Array [varEmpty..varQWord] of Longint =
    (ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,
     ARR_OLESTR,ARR_DISPATCH,ARR_NONE,ARR_NONE,ARR_VARIANT,ARR_UNKNOWN,
     ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE,ARR_NONE);
  psaElementSizes : Array [varEmpty..varQWord] of Byte =
    (0,0,                  // varempty, varnull
     SizeOf(SmallInt),     // varsmallint
     SizeOf(Integer),      // varinteger
     SizeOf(Single),       // varsingle
     SizeOf(Double),       // vardouble
     SizeOf(Currency),     // varcurrency
     SizeOf(TDateTime),    // vardate
     SizeOf(PWideString),  // varolestr
     SizeOf(IInterface),   // vardispatch
     SizeOf(TError),       // varerror
     SizeOf(Boolean),      // varboolean
     SizeOf(TVarData),     // varvariant
     SizeOf(IUnknown),     // varunknown
     0, // Decimal         // vardecimal
     0, // Unused
     SizeOf(ShortInt),     // varshortint
     SizeOf(Byte),         // varbyte
     SizeOf(Word),         // varword
     SizeOf(LongWord),     // varlongword
     SizeOf(Int64),        // varint64
     SizeOf(QWord));       // varqword

Function SafeArrayCreate(VarType, Dim: DWord; const Bounds: TVarArrayBoundArray): PVarArray;stdcall;
  var
    res : HRESULT;
    I   : DWord;
  begin
    Result:=nil;
    if Not (VarType in Supportedpsas) Then
      exit;
    Res:=SafeArrayAllocDescriptor(Dim, Result);
    if Res<>VAR_OK then
      exit;
    Result^.DimCount:=Dim;
    Result^.Flags:=psaElementFlags[VarType];
    Result^.ElementSize:=psaElementSizes[VarType];
    Result^.LockCount := 0;
    for i:=0 to Dim-1 do
      begin
        Result^.Bounds[i].LowBound:=Bounds[Dim-I-1].LowBound;
        Result^.Bounds[I].ElementCount:=Bounds[Dim-I-1].ElementCount;
      end;
    res:=SafeArrayAllocData(Result);
    if res<>VAR_OK then
      begin
      SafeArrayDestroyDescriptor(Result);
      Result:=nil;
      end;
  end;


Function SafeArrayAllocDescriptor(DimCount: Dword; var psa: PVarArray): HRESULT;stdcall;
begin
  try
    { one bound item is included in TVarArray }
    psa:=GetMem(SizeOf(TVarArray) + SizeOf(TVarArrayBound)*(DimCount-1));
    Result:=VAR_OK;
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E);
  end;
end;

Function SafeArrayAllocData(psa: PVarArray): HRESULT;stdcall;
begin
  try
    With psa^ do
      begin
        Data:=GetMem(SafeArrayElementTotal(psa)*ElementSize);
        fillchar(Data^,SafeArrayElementTotal(psa)*ElementSize,0);
      end;
    Result:=VAR_OK;
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E);
  end;
end;

Function SafeArrayDestroy(psa: PVarArray): HRESULT;stdcall;
begin
  Result:=CheckVarArray(psa);
  if Result<> VAR_OK then
    exit;
  Result:=CheckArrayUnlocked(psa);
  if Result<> VAR_OK then
    exit;
  Result:=SafeArrayDestroyData(psa);
  if Result<>VAR_OK then
    exit;
  Result:=SafeArrayDestroyDescriptor(psa);
end;

Function SafeArrayDestroyDescriptor(psa: PVarArray): HRESULT;stdcall;
begin
  Result:=CheckVarArray(psa);
  if Result<>VAR_OK then
    exit;
  Result:=CheckArrayUnlocked(psa);
  if Result<> VAR_OK then
    exit;
  try
    FreeMem(psa);
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E);
  end;
end;

Function SafeArrayDestroyData(psa: PVarArray): HRESULT;stdcall;
begin
  Result:=CheckVarArray(psa);
  if Result<>VAR_OK then
    exit;
  Result:=CheckArrayUnlocked(psa);
  if Result<> VAR_OK then
    exit;
  try
    Result:=SafeArrayClearDataSpace(psa, False);
    if (Result=VAR_OK) and ((psa^.Flags and ARR_FIXEDSIZE)=0) then
      begin
      FreeMem(psa^.Data);
      psa^.Data:=nil;
      end;
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E);
  end;
end;

Function SafeArrayRedim(psa: PVarArray; const NewBound: TVarArrayBound): HRESULT;stdcall;

var
  vat: TVariantArrayType;
  i, D,j,count : Integer;
  P : Pointer;

begin
  Result:=CheckVarArray(psa);
  if Result <> VAR_OK then
    exit;
  if (psa^.Flags and ARR_FIXEDSIZE) <> 0 then
    Exit(VAR_INVALIDARG);
  Result:=SafeArrayLock(psa);
  if Result<>VAR_OK then
    exit;
  try
    D:=NewBound.ElementCount - psa^.Bounds[0].ElementCount;
    for i:=1 to psa^.DimCount - 1 do
      D:=D*psa^.Bounds[i].ElementCount;
    if D<>0 then
      begin
      Count:=SafeArrayElementTotal(psa);
      if D<0 then
        begin
        vat:=VariantArrayType(psa);
        for j:=Count-1 downto Count+D do
          begin
          P:=SafeArrayCalculateElementAddress(psa,j);
          if vat = vatInterface then
            IUnknown(PPointer(P)^):=Nil
          else if vat=vatWideString then
            WideString(PPointer(P)^):=''
          else if vat=vatVariant then
            VariantClear(PVarData(P)^);
          end;
        end;
      ReAllocMem(psa^.Data,(Count+D)*psa^.ElementSize);
      if D>0 then
        fillchar((PChar(psa^.Data)+Count*psa^.ElementSize)^,D*psa^.ElementSize,0);
      end;
    psa^.Bounds[0].ElementCount:=NewBound.ElementCount;
    psa^.Bounds[0].LowBound:=NewBound.LowBound;
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E);
  end;
  SetUnlockResult(psa,Result);
end;

Function SafeArrayCopy(psa: PVarArray; var psaOut: PVarArray): HRESULT;stdcall;

var
  i : Integer;

begin
  Result:=CheckVarArray(psa);
  if Result<>VAR_OK then
    exit;
  Result:=SafeArrayLock(psa);
  if Result<>VAR_OK then
    exit;
  try
    Result:=SafeArrayAllocDescriptor(psa^.DimCount,psaOut);
    if Result<>VAR_OK then
      Exit;
    try
      With psaOut^ do
        begin
        Flags:=psa^.Flags;
        ElementSize:=psa^.ElementSize;
        LockCount := 0;
        DimCount:=psa^.DimCount;
        for i:=0 to DimCount-1 do
          begin
          Bounds[i].ElementCount:=psa^.Bounds[i].ElementCount;
          Bounds[i].LowBound:=psa^.Bounds[i].LowBound;
          end;
        end;
      Result:=SafeArrayAllocData(psaOut);
      if Result<>VAR_OK then
        exit;
      Result:=SafeArrayCopyDataSpace(psa, psaOut);
    finally
      if Result<>VAR_OK then
        begin
        SafeArrayDestroyDescriptor(psaOut);
        psaOut:=nil;
        end;
    end;
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E)
  end;
  SetUnlockResult(psa,Result);
end;

Function SafeArrayCopyData(psa, psaOut: PVarArray): HRESULT;stdcall;
var
  i : Integer;
begin
  Result:=CheckVarArray(psa);
  if Result<>VAR_OK then
    exit;
  Result:=CheckVarArray(psaOut);
  if Result<>VAR_OK then
    exit;
  Result:=SafeArrayLock(psaOut);
  if Result<>VAR_OK then
    exit;
  try
    Result:=SafeArrayLock(psa);
    if Result<>VAR_OK then
     exit;
    try
      With psaOut^ do
        begin
        if (psa^.Flags<>Flags) or
           (psa^.ElementSize<>ElementSize) or
           (psa^.DimCount<>DimCount) then
          Exit(VAR_INVALIDARG);
        for i:=0 to psa^.DimCount - 1 do
          if (psa^.Bounds[i].LowBound<>Bounds[i].LowBound) or
             (psa^.Bounds[i].ElementCount<>Bounds[i].ElementCount) then
            exit(VAR_INVALIDARG);
        end;
      Result:=SafeArrayClearDataSpace(psaOut,True);
      if Result<> VAR_OK then
        exit;
      Result:=SafeArrayCopyDataSpace(psa, psaOut);
    finally
      SetUnlockResult(psa,Result);
    end;
  finally
    SetUnlockResult(psaOut,Result);
  end;
end;

Function SafeArrayGetLBound(psa: PVarArray; Dim: DWord; var LBound: LongInt): HRESULT;stdcall;
begin
  Result:=CheckVarArray(psa);
  if Result<>VAR_OK then
    exit;
  if (Dim>0) and (Dim<=psa^.DimCount) then
    LBound:=psa^.Bounds[psa^.dimcount-Dim].LowBound
  else
    Result:=VAR_BADINDEX;
end;

Function SafeArrayGetUBound(psa: PVarArray; Dim : DWord; var UBound: LongInt): HRESULT;stdcall;
begin
  Result:=CheckVarArray(psa);
  if Result<>VAR_OK then
    exit;
  if (Dim>0) and (Dim<=psa^.DimCount) then
    UBound:=psa^.Bounds[psa^.dimcount-Dim].LowBound +
            psa^.Bounds[psa^.dimcount-Dim].ElementCount-1
  else
    Result:=VAR_BADINDEX
end;

Function SafeArrayGetDim(psa: PVarArray): HRESULT;stdcall;
begin
  if CheckVarArray(psa)<>VAR_OK then
    Result:=0
  else
    Result:=psa^.DimCount;
end;

Function SafeArrayAccessData(psa: PVarArray; var ppvData: Pointer): HRESULT;stdcall;
begin
  Result:=SafeArrayLock(psa);
  if Result<>VAR_OK then
    ppvData:=nil
  else
    ppvData:=psa^.Data;
end;

Function SafeArrayUnaccessData(psa: PVarArray): HRESULT;stdcall;
begin
  Result:=SafeArrayUnlock(psa);
end;

Function SafeArrayLock(psa: PVarArray): HRESULT;stdcall;
begin
  Result:=CheckVarArray(psa);
  if Result<>VAR_OK then
    exit;
  InterlockedIncrement(psa^.LockCount);
end;

Function SafeArrayUnlock(psa: PVarArray): HRESULT;stdcall;
begin
  Result:=CheckVarArray(psa);
  if (Result<>VAR_OK) then
    exit;
  if InterlockedDecrement(psa^.LockCount)<0 then
    begin
      InterlockedIncrement(psa^.LockCount);
      result:=VAR_UNEXPECTED;
    end;
end;


Function SafeArrayGetElement(psa: PVarArray; Indices: PVarArrayCoorArray;
  Data: Pointer): HRESULT;stdcall;
var
  P: Pointer;
begin
  Result:=CheckVarArrayAndCalculateAddress(psa, Indices, P, True);
  if Result<>VAR_OK then
    exit;
  try
    case VariantArrayType(psa) of
      vatNormal:
        Move(P^, Data^, psa^.ElementSize);
      vatInterface:
        IInterface(PInterface(Data)^) := IInterface(PInterface(P)^);
      vatWideString:
        CopyAsWideString(PWideChar(Data^), PWideChar(P^));
      vatVariant:
        VariantCopy(PVarData(Data)^, PVarData(P)^);
    end;
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E);
  end;
  SetUnlockResult(psa,Result);
end;


Function SafeArrayPutElement(psa: PVarArray; Indices: PVarArrayCoorArray;
  const Data: Pointer): HRESULT;stdcall;
var
  P: Pointer;
begin
  Result:=CheckVarArrayAndCalculateAddress(psa,Indices,P,True);
  if Result<>VAR_OK then
    exit;
  try
    case VariantArrayType(psa) of
      vatNormal:
        Move(Data^,P^,psa^.ElementSize);
      vatInterface:
        IInterface(PInterface(P)^):=IInterface(Data);
      vatWideString:
        CopyAsWideString(PWideChar(P^), PWideChar(Data));
      vatVariant:
        VariantCopy(PVarData(P)^, PVarData(Data)^);    // !! Untested
    end;
  except
    On E : Exception do
      Result:=ExceptionToVariantError(E);
  end;
  SetUnlockResult(psa,Result);
end;


Function SafeArrayPtrOfIndex(psa: PVarArray; Indices: PVarArrayCoorArray;
  var Address: Pointer): HRESULT;stdcall;
begin
  Result:=CheckVarArrayAndCalculateAddress(psa,Indices,Address,False);
end;


Function SafeArrayGetElemSize(psa: PVarArray): LongWord;stdcall;
begin
  if CheckVarArray(psa)<>VAR_OK then
    Result:=0
  else
    Result:=psa^.ElementSize;
end;



