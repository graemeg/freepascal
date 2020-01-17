{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ Run-Time type information routines }

function RTTIArraySize(typeInfo: Pointer): SizeInt;
begin
{$ifdef VER3_0}
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
{$else VER3_0}
  typeInfo:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);
{$endif VER3_0}
  {$ifdef VER2_6}
  result:=PArrayInfo(typeInfo)^.Size*PArrayInfo(typeInfo)^.ElCount;
  {$else}
  result:=PArrayInfo(typeInfo)^.Size;
  {$endif}
end;

function RTTIRecordSize(typeInfo: Pointer): SizeInt;
begin
{$ifdef VER3_0}
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
{$else VER3_0}
  typeInfo:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);
{$endif VER3_0}
  { for size field init table is compatible with rtti table }
  result:=PRecordInfoFull(typeInfo)^.Size;
end;

{$ifndef VER3_0}
function RTTIRecordOp(typeInfo: Pointer; var initrtti: Pointer): PRecordInfoInit; inline;
begin
  { find init table and management operators }
  typeInfo:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);
  result:=typeInfo;

  { check terminator, maybe we are already in init table }
  if Assigned(result^.Terminator) then
  begin
    { point to more optimal initrtti }
    initrtti:=PRecordInfoFull(result)^.InitTable;
    { and point to management operators in our init table }
    result:=aligntoqword(initrtti+2+PByte(initrtti)[1]);
  end
end;
{$else VER3_0}
function RTTIRecordRttiInfoToInitInfo(typeInfo: Pointer): Pointer; inline;
begin
  result:=typeInfo;
{$ifndef VER3_0}
  { find init table }
  typeInfo:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);

  { check terminator, maybe we are already in init table }
  if Assigned(PRecordInfoInit(typeInfo)^.Terminator) then
    { point to more optimal initrtti }
    result:=PRecordInfoFull(typeInfo)^.InitTable;
{$endif VER3_0}
end;
{$endif VER3_0}

{$ifndef VER3_0}
function RTTISizeAndOp(typeInfo: Pointer;
  const expectedManagementOp: TRTTIRecOpType; out hasManagementOp: boolean): SizeInt;
begin
  hasManagementOp:=false;
{$else VER3_0}
function RTTISize(typeInfo: Pointer): SizeInt;
begin
{$endif VER3_0}
  case PTypeKind(typeinfo)^ of
    tkAString,tkWString,tkUString,
    tkInterface,tkDynarray:
      result:=sizeof(Pointer);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      result:=sizeof(TVarData);
{$endif FPC_HAS_FEATURE_VARIANTS}
    tkArray:
      result:=RTTIArraySize(typeinfo);
{$ifndef VER3_0}
    tkObject:
      result:=RTTIRecordSize(typeinfo);
    tkRecord:
      with RTTIRecordOp(typeInfo,typeInfo)^ do
        begin
          result:=Size;
          hasManagementOp := Assigned(RecordOp);
          if hasManagementOp then
            case expectedManagementOp of
              rotInitialize: hasManagementOp:=Assigned(RecordOp^.Initialize);
              rotFinalize: hasManagementOp:=Assigned(RecordOp^.Finalize);
              rotAddRef: hasManagementOp:=Assigned(RecordOp^.AddRef);
              rotCopy: hasManagementOp:=Assigned(RecordOp^.Copy);
            end;
        end;
{$else VER3_0}
    tkObject,tkRecord:
      result:=RTTIRecordSize(typeinfo);
{$endif VER3_0}
  else
    result:=-1;
  end;
end;

{ if you modify this procedure, fpc_copy must be probably modified as well }
procedure RecordRTTI(Data,TypeInfo:Pointer;rttiproc:TRTTIProc);
var
  count,
  i : longint;
begin
{$ifdef VER3_0}
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
{$else VER3_0}
  typeInfo:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);
{$endif VER3_0}
  Count:=PRecordInfoInit(typeInfo)^.Count;
  { Get element info, hacky, but what else can we do? }
  typeInfo:=AlignTypeData(Pointer(@PRecordInfoInit(typeInfo)^.Count)+SizeOf(PRecordInfoInit(typeInfo)^.Count));
  { Process elements }
  for i:=1 to count Do
    begin
      rttiproc(Data+PRecordElement(typeInfo)^.Offset,PRecordElement(typeInfo)^.TypeInfo{$ifndef VER3_0}^{$endif});
      Inc(PRecordElement(typeInfo));
    end;
end;


{$ifndef VER3_0}
function RTTIRecordMopInitTable(ti: Pointer): PRTTIRecordOpOffsetTable;
begin
  ti:=aligntoqword(ti+2+PByte(ti)[1]);
  Result:=PRecordInfoInit(ti)^.InitRecordOpTable;
end;
{$endif VER3_0}


{ if you modify this procedure, fpc_copy must be probably modified as well }
{$ifdef VER2_6}
procedure ArrayRTTI(Data,TypeInfo:Pointer;rttiproc:TRTTIProc);
var
  i: SizeInt;
begin
{$ifdef VER3_0}
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
{$else VER3_0}
  typeInfo:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);
{$endif VER3_0}
  { Process elements }
  for I:=0 to PArrayInfo(typeInfo)^.ElCount-1 do
    rttiproc(Data+(I*PArrayInfo(typeInfo)^.Size),PArrayInfo(typeInfo)^.ElInfo);
end;
{$else}
procedure ArrayRTTI(Data,TypeInfo:Pointer;rttiproc:TRTTIProc);
var  
  i,Count,ElSize: SizeInt;
  Info: Pointer;
begin
{$ifdef VER3_0}
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
{$else VER3_0}
  typeInfo:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);
{$endif VER3_0}
  Count:=PArrayInfo(typeInfo)^.ElCount;
  { no elements to process => exit }
  if Count = 0 then
    Exit;
  ElSize:=PArrayInfo(typeInfo)^.Size div Count;
  Info:=PArrayInfo(typeInfo)^.ElInfo{$ifndef VER3_0}^{$endif};
  { Process elements }
  for I:=0 to Count-1 do
    rttiproc(Data+(I*ElSize),Info);
end;
{$endif}

Procedure fpc_Initialize (Data,TypeInfo : pointer);[Public,Alias : 'FPC_INITIALIZE'];  compilerproc;
begin
  case PTypeKind(TypeInfo)^ of
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
    tkDynArray,
{$endif FPC_HAS_FEATURE_DYNARRAYS}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
    tkAstring,
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
    tkWstring,tkUString,
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkInterface:
      PPchar(Data)^:=Nil;
    tkArray:
      arrayrtti(data,typeinfo,@int_initialize);
{$ifdef FPC_HAS_FEATURE_OBJECTS}
    tkObject,
{$endif FPC_HAS_FEATURE_OBJECTS}
    tkRecord:
{$ifndef VER3_0}
      { if possible try to use more optimal initrtti }
      with RTTIRecordOp(typeinfo, typeinfo)^ do
      begin
        recordrtti(data,typeinfo,@int_initialize);
        if Assigned(recordop) and Assigned(recordop^.Initialize) then
          recordop^.Initialize(data);
      end;
{$else VER3_0}
      begin
        typeinfo:=RTTIRecordRttiInfoToInitInfo(typeinfo);
        recordrtti(data,typeinfo,@int_initialize);
      end;
{$endif VER3_0}
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      variant_init(PVarData(Data)^);
{$endif FPC_HAS_FEATURE_VARIANTS}
  end;
end;


Procedure fpc_finalize (Data,TypeInfo: Pointer);[Public,Alias : 'FPC_FINALIZE'];  compilerproc;
begin
  case PTypeKind(TypeInfo)^ of
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
    tkAstring :
      fpc_AnsiStr_Decr_Ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
    tkUstring :
      fpc_UnicodeStr_Decr_Ref(PPointer(Data)^);
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkWstring :
      fpc_WideStr_Decr_Ref(PPointer(Data)^);
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkArray :
      arrayrtti(data,typeinfo,@int_finalize);
{$ifdef FPC_HAS_FEATURE_OBJECTS}
    tkObject,
{$endif FPC_HAS_FEATURE_OBJECTS}
    tkRecord:
{$ifndef VER3_0}
      { if possible try to use more optimal initrtti }
      with RTTIRecordOp(typeinfo, typeinfo)^ do
      begin
        if Assigned(recordop) and Assigned(recordop^.Finalize) then
          recordop^.Finalize(data);
        recordrtti(data,typeinfo,@int_finalize);
      end;
{$else VER3_0}
      begin
        typeinfo:=RTTIRecordRttiInfoToInitInfo(typeinfo);
        recordrtti(data,typeinfo,@int_finalize);
      end;
{$endif VER3_0}
{$ifdef FPC_HAS_FEATURE_CLASSES}
    tkInterface:
      Intf_Decr_Ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_CLASSES}
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
    tkDynArray:
      fpc_dynarray_clear(PPointer(Data)^,TypeInfo);
{$endif FPC_HAS_FEATURE_DYNARRAYS}
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      variant_clear(PVarData(Data)^);
{$endif FPC_HAS_FEATURE_VARIANTS}
  end;
end;


Procedure fpc_Addref (Data,TypeInfo : Pointer); [Public,alias : 'FPC_ADDREF'];  compilerproc;
begin
  case PTypeKind(TypeInfo)^ of
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
    tkAstring :
      fpc_AnsiStr_Incr_Ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkWstring :
      fpc_WideStr_Incr_Ref(PPointer(Data)^);
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkUstring :
      fpc_UnicodeStr_Incr_Ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkArray :
      arrayrtti(data,typeinfo,@int_addref);
{$ifdef FPC_HAS_FEATURE_OBJECTS}
    tkobject,
{$endif FPC_HAS_FEATURE_OBJECTS}
    tkrecord :
{$ifndef VER3_0}
      { find init table }
      with RTTIRecordOp(typeinfo, typeinfo)^ do
      begin
        recordrtti(data,typeinfo,@int_addref);
        if Assigned(recordop) and Assigned(recordop^.AddRef) then
          recordop^.AddRef(Data);
      end;
{$else VER3_0}
      begin
        typeinfo:=RTTIRecordRttiInfoToInitInfo(typeinfo);
        recordrtti(data,typeinfo,@int_addref);
      end;
{$endif VER3_0}
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
    tkDynArray:
      fpc_dynarray_incr_ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_DYNARRAYS}
{$ifdef FPC_HAS_FEATURE_CLASSES}
    tkInterface:
      Intf_Incr_Ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_CLASSES}
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      variant_addref(pvardata(Data)^);
{$endif FPC_HAS_FEATURE_DYNARRAYS}
  end;
end;


{ define alias for internal use in the system unit }
Function fpc_Copy_internal (Src, Dest, TypeInfo : Pointer) : SizeInt;[external name 'FPC_COPY'];

Function fpc_Copy (Src, Dest, TypeInfo : Pointer) : SizeInt;[Public,alias : 'FPC_COPY']; compilerproc;
var
  Temp: pbyte;
  copiedsize,
  expectedoffset,
  EleCount,
  offset,
  i: SizeInt;
  info: pointer;
begin
  result:=sizeof(pointer);
  case PTypeKind(TypeInfo)^ of
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
    tkAstring:
      fpc_AnsiStr_Assign(PPointer(Dest)^,PPointer(Src)^);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkWstring:
      fpc_WideStr_Assign(PPointer(Dest)^,PPointer(Src)^);
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkUstring:
      fpc_UnicodeStr_Assign(PPointer(Dest)^,PPointer(Src)^);
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkArray:
      begin
{$ifdef VER3_0}
        Temp:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
{$else VER3_0}
        Temp:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);
{$endif VER3_0}
      {$ifdef VER2_6}
        { Process elements }
        for I:=0 to PArrayInfo(Temp)^.ElCount-1 do
          fpc_Copy_internal(Src+(I*PArrayInfo(Temp)^.Size),Dest+(I*PArrayInfo(Temp)^.Size),PArrayInfo(Temp)^.ElInfo);
        Result:=PArrayInfo(Temp)^.Size*PArrayInfo(Temp)^.ElCount;
      {$else}
        Result:=PArrayInfo(Temp)^.Size;
        EleCount:=PArrayInfo(Temp)^.ElCount;
        { no elements to process => exit }
        if EleCount = 0 then
          Exit;
        Info:=PArrayInfo(Temp)^.ElInfo{$ifndef VER3_0}^{$endif};
        copiedsize:=Result div EleCount;
        Offset:=0;
        { Process elements }
        for I:=1 to EleCount do
          begin
            fpc_Copy_internal(Src+Offset,Dest+Offset,Info);
            inc(Offset,copiedsize);
          end;
      {$endif}
      end;
{$ifdef FPC_HAS_FEATURE_OBJECTS}
    tkobject,
{$endif FPC_HAS_FEATURE_OBJECTS}
    tkrecord:
{$ifndef VER3_0}
      { find init table }
      with RTTIRecordOp(typeinfo, typeinfo)^ do
{$endif VER3_0}
      begin
{$ifdef VER3_0}
        typeInfo:=RTTIRecordRttiInfoToInitInfo(typeInfo);
        Temp:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
{$else VER3_0}
        Temp:=aligntoqword(typeInfo+2+PByte(typeInfo)[1]);
{$endif VER3_0}
{$ifndef VER3_0}
        if Assigned(recordop) and Assigned(recordop^.Copy) then
          recordop^.Copy(Src,Dest)
        else
          begin
            Result:=PRecordInfoInit(Temp)^.Size;
            EleCount:=PRecordInfoInit(Temp)^.Count;
            { Get element info, hacky, but what else can we do? }
            Temp:=AlignTypeData(Pointer(@PRecordInfoInit(Temp)^.Count)+SizeOf(PRecordInfoInit(Temp)^.Count));
{$else VER3_0}
            Result:=PRecordInfoFull(Temp)^.Size;
            EleCount:=PRecordInfoFull(Temp)^.Count;
            { Get element info, hacky, but what else can we do? }
            Temp:=Pointer(@PRecordInfoFull(Temp)^.Count)+SizeOf(PRecordInfoFull(Temp)^.Count);
{$endif VER3_0}
            expectedoffset:=0;
            { Process elements with rtti }
            for i:=1 to EleCount Do
              begin
                Info:=PRecordElement(Temp)^.TypeInfo{$ifndef VER3_0}^{$endif};
                Offset:=PRecordElement(Temp)^.Offset;
                Inc(PRecordElement(Temp));
                if Offset>expectedoffset then
                  move((Src+expectedoffset)^,(Dest+expectedoffset)^,Offset-expectedoffset);
                copiedsize:=fpc_Copy_internal(Src+Offset,Dest+Offset,Info);
                expectedoffset:=Offset+copiedsize;
              end;
            { elements remaining? }
            if result>expectedoffset then
              move((Src+expectedoffset)^,(Dest+expectedoffset)^,Result-expectedoffset);
{$ifndef VER3_0}
          end;
{$endif VER3_0}
      end;
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
    tkDynArray:
      fpc_dynarray_assign(PPointer(Dest)^,PPointer(Src)^,typeinfo);
{$endif FPC_HAS_FEATURE_DYNARRAYS}
{$ifdef FPC_HAS_FEATURE_CLASSES}
    tkInterface:
      fpc_intf_assign(PPointer(Dest)^,PPointer(Src)^);
{$endif FPC_HAS_FEATURE_CLASSES}
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      begin
        VarCopyProc(pvardata(dest)^,pvardata(src)^);
        result:=sizeof(tvardata);
      end;
{$endif FPC_HAS_FEATURE_VARIANTS}
  end;
end;


{ For internal use by the compiler, because otherwise $x- can cause trouble. }
{ Generally disabling extended syntax checking for all compilerprocs may     }
{ have unintended side-effects                                               }
procedure fpc_Copy_proc (Src, Dest, TypeInfo : Pointer);compilerproc; inline;
begin
  fpc_copy_internal(src,dest,typeinfo);
end;


procedure fpc_initialize_array(data,typeinfo : pointer;count : SizeInt); [public,alias:'FPC_INITIALIZE_ARRAY']; compilerproc;
  var
    i, size : SizeInt;
{$ifndef VER3_0}
    hasManagementOp: boolean;
  begin
    size:=RTTISizeAndOp(typeinfo, rotInitialize, hasManagementOp);
    if (size>0) or hasManagementOp then
{$else VER3_0}
  begin
    size:=RTTISize(typeInfo);
    if size>0 then
{$endif VER3_0}
      for i:=0 to count-1 do
        int_initialize(data+size*i,typeinfo);
  end;


procedure fpc_finalize_array(data,typeinfo : pointer;count : SizeInt); [Public,Alias:'FPC_FINALIZE_ARRAY'];  compilerproc;
  var
     i, size: SizeInt;
{$ifndef VER3_0}
    hasManagementOp: boolean;
  begin
    size:=RTTISizeAndOp(typeinfo, rotFinalize, hasManagementOp);
    if (size>0) or hasManagementOp then
{$else VER3_0}
  begin
    size:=RTTISize(typeInfo);
    if size>0 then
{$endif VER3_0}
      for i:=0 to count-1 do
        int_finalize(data+size*i,typeinfo);
  end;


procedure fpc_addref_array(data,typeinfo: pointer; count: SizeInt); [public,alias:'FPC_ADDREF_ARRAY']; compilerproc;
  var
    i, size: SizeInt;
{$ifndef VER3_0}
    hasManagementOp: boolean;
  begin
    size:=RTTISizeAndOp(typeinfo, rotAddRef, hasManagementOp);
    if (size>0) or hasManagementOp then
{$else VER3_0}
  begin
    size:=RTTISize(typeInfo);
    if size>0 then
{$endif VER3_0}
      for i:=0 to count-1 do
        int_addref(data+size*i,typeinfo);
  end;

{ The following two procedures are now obsolete, needed only for bootstrapping }
procedure fpc_decref (Data, TypeInfo : Pointer);[Public,alias : 'FPC_DECREF'];  compilerproc;
  begin
    int_finalize(Data,TypeInfo);
  end;

procedure fpc_decref_array(data,typeinfo: pointer; count: SizeInt); [public,alias:'FPC_DECREF_ARRAY']; compilerproc;
  begin
    int_finalizeArray(data,typeinfo,count);
  end;

procedure InitializeArray(p, typeInfo: Pointer; count: SizeInt);
  external name 'FPC_INITIALIZE_ARRAY';

procedure FinalizeArray(p, typeInfo: Pointer; count: SizeInt);
  external name 'FPC_FINALIZE_ARRAY';

procedure CopyArray(dest, source, typeInfo: Pointer; count: SizeInt);
  var
    i, size: SizeInt;
{$ifndef VER3_0}
    hasManagementOp: boolean;
  begin
    size:=RTTISizeAndOp(typeinfo, rotCopy, hasManagementOp);
    if (size>0) or hasManagementOp then
{$else VER3_0}
  begin
    size:=RTTISize(typeInfo);
    if size>0 then
{$endif VER3_0}
      for i:=0 to count-1 do
        fpc_Copy_internal(source+size*i, dest+size*i, typeInfo);
  end;

