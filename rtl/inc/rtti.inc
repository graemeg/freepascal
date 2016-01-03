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

{ the tk* constants are now declared in system.inc }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
  {$define USE_PACKED}
{$endif}
{$ifdef VER2_6}
  {$define USE_PACKED}
{$endif}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

type
  PRecordElement=^TRecordElement;
  TRecordElement=
{$ifdef USE_PACKED}
  packed
{$endif USE_PACKED}
  record
    TypeInfo: Pointer;
    {$ifdef VER2_6}
    Offset: Longint;
    {$else}
    Offset: SizeInt;
    {$endif}
  end;

  PRecordInfo=^TRecordInfo;
  TRecordInfo=
{$ifdef USE_PACKED}
  packed
{$endif USE_PACKED}
  record
    Size: Longint;
    Count: Longint;
    { Elements: array[count] of TRecordElement }
  end;

  PArrayInfo=^TArrayInfo;
  TArrayInfo=
{$ifdef USE_PACKED}
  packed
{$endif USE_PACKED}
  record
    Size: SizeInt;
    ElCount: SizeInt;
    ElInfo: Pointer;
    DimCount: Byte;
    Dims:array[0..255] of Pointer;
  end;

function RTTIArraySize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  {$ifdef VER2_6}
  result:=PArrayInfo(typeInfo)^.Size*PArrayInfo(typeInfo)^.ElCount;
  {$else}
  result:=PArrayInfo(typeInfo)^.Size;
  {$endif}
end;

function RTTIRecordSize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PRecordInfo(typeInfo)^.Size;
end;

function RTTISize(typeInfo: Pointer): SizeInt;
begin
  case PByte(typeinfo)^ of
    tkAString,tkWString,tkUString,
    tkInterface,tkDynarray:
      result:=sizeof(Pointer);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      result:=sizeof(TVarData);
{$endif FPC_HAS_FEATURE_VARIANTS}
    tkArray:
      result:=RTTIArraySize(typeinfo);
    tkObject,tkRecord:
      result:=RTTIRecordSize(typeinfo);
  else
    result:=-1;
  end;
end;

  PRecordElement=^TRecordElement;
  TRecordElement=packed record
    TypeInfo: Pointer;
    Offset: Longint;
  end;

  PRecordInfo=^TRecordInfo;
  TRecordInfo=packed record
    Size: Longint;
    Count: Longint;
    { Elements: array[count] of TRecordElement }
  end;

  PArrayInfo=^TArrayInfo;
  TArrayInfo=packed record
    ElSize: SizeInt;
    ElCount: SizeInt;
    ElInfo: Pointer;
  end;


function RTTIArraySize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PArrayInfo(typeInfo)^.ElSize * PArrayInfo(typeInfo)^.ElCount;
end;

function RTTIRecordSize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PRecordInfo(typeInfo)^.Size;
end;

function RTTISize(typeInfo: Pointer): SizeInt;
begin
  case PByte(typeinfo)^ of
    tkAString,tkWString,tkUString,
    tkInterface,tkDynarray:
      result:=sizeof(Pointer);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      result:=sizeof(TVarData);
{$endif FPC_HAS_FEATURE_VARIANTS}
    tkArray:
      result:=RTTIArraySize(typeinfo);
    tkObject,tkRecord:
      result:=RTTIRecordSize(typeinfo);
  else
    result:=-1;
  end;
end;

  PRecordElement=^TRecordElement;
  TRecordElement=packed record
    TypeInfo: Pointer;
    Offset: Longint;
  end;

  PRecordInfo=^TRecordInfo;
  TRecordInfo=packed record
    Size: Longint;
    Count: Longint;
    { Elements: array[count] of TRecordElement }
  end;

  PArrayInfo=^TArrayInfo;
  TArrayInfo=packed record
    ElSize: SizeInt;
    ElCount: SizeInt;
    ElInfo: Pointer;
  end;


function RTTIArraySize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PArrayInfo(typeInfo)^.ElSize * PArrayInfo(typeInfo)^.ElCount;
end;

function RTTIRecordSize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PRecordInfo(typeInfo)^.Size;
end;

function RTTISize(typeInfo: Pointer): SizeInt;
begin
  case PByte(typeinfo)^ of
    tkAString,tkWString,tkUString,
    tkInterface,tkDynarray:
      result:=sizeof(Pointer);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      result:=sizeof(TVarData);
{$endif FPC_HAS_FEATURE_VARIANTS}
    tkArray:
      result:=RTTIArraySize(typeinfo);
    tkObject,tkRecord:
      result:=RTTIRecordSize(typeinfo);
  else
    result:=-1;
  end;
end;

  PRecordElement=^TRecordElement;
  TRecordElement=packed record
    TypeInfo: Pointer;
    Offset: Longint;
  end;

  PRecordInfo=^TRecordInfo;
  TRecordInfo=packed record
    Size: Longint;
    Count: Longint;
    { Elements: array[count] of TRecordElement }
  end;

<<<<<<< HEAD
  PArrayInfo=^TArrayInfo;
  TArrayInfo=packed record
    ElSize: SizeInt;
    ElCount: SizeInt;
    ElInfo: Pointer;
  end;


function RTTIArraySize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PArrayInfo(typeInfo)^.ElSize * PArrayInfo(typeInfo)^.ElCount;
end;

function RTTIRecordSize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PRecordInfo(typeInfo)^.Size;
end;

function RTTISize(typeInfo: Pointer): SizeInt;
begin
  case PByte(typeinfo)^ of
    tkAString,tkWString,tkUString,
    tkInterface,tkDynarray:
      result:=sizeof(Pointer);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      result:=sizeof(TVarData);
{$endif FPC_HAS_FEATURE_VARIANTS}
    tkArray:
      result:=RTTIArraySize(typeinfo);
    tkObject,tkRecord:
      result:=RTTIRecordSize(typeinfo);
  else
    result:=-1;
  end;
end;

  PRecordElement=^TRecordElement;
  TRecordElement=packed record
    TypeInfo: Pointer;
    Offset: Longint;
  end;

  PRecordInfo=^TRecordInfo;
  TRecordInfo=packed record
    Size: Longint;
    Count: Longint;
    { Elements: array[count] of TRecordElement }
  end;

  PArrayInfo=^TArrayInfo;
  TArrayInfo=packed record
    ElSize: SizeInt;
    ElCount: SizeInt;
    ElInfo: Pointer;
  end;

<<<<<<< HEAD

function RTTIArraySize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PArrayInfo(typeInfo)^.ElSize * PArrayInfo(typeInfo)^.ElCount;
end;

function RTTIRecordSize(typeInfo: Pointer): SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  result:=PRecordInfo(typeInfo)^.Size;
end;

function RTTISize(typeInfo: Pointer): SizeInt;
begin
  case PByte(typeinfo)^ of
    tkAString,tkWString,tkUString,
    tkInterface,tkDynarray:
      result:=sizeof(Pointer);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      result:=sizeof(TVarData);
{$endif FPC_HAS_FEATURE_VARIANTS}
    tkArray:
      result:=RTTIArraySize(typeinfo);
    tkObject,tkRecord:
      result:=RTTIRecordSize(typeinfo);
  else
    result:=-1;
  end;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ if you modify this procedure, fpc_copy must be probably modified as well }
procedure RecordRTTI(Data,TypeInfo:Pointer;rttiproc:TRTTIProc);
var
  count,
  i : longint;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  Count:=PRecordInfo(typeInfo)^.Count;
  Inc(PRecordInfo(typeInfo));
  { Process elements }
  for i:=1 to count Do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      rttiproc(Data+PRecordElement(typeInfo)^.Offset,PRecordElement(typeInfo)^.TypeInfo);
=======
      rttiproc (Data+PRecordElement(typeInfo)^.Offset,PRecordElement(typeInfo)^.TypeInfo);
>>>>>>> graemeg/cpstrnew
=======
      rttiproc (Data+PRecordElement(typeInfo)^.Offset,PRecordElement(typeInfo)^.TypeInfo);
>>>>>>> graemeg/cpstrnew
=======
      rttiproc (Data+PRecordElement(typeInfo)^.Offset,PRecordElement(typeInfo)^.TypeInfo);
>>>>>>> graemeg/cpstrnew
=======
      rttiproc (Data+PRecordElement(typeInfo)^.Offset,PRecordElement(typeInfo)^.TypeInfo);
>>>>>>> origin/cpstrnew
      Inc(PRecordElement(typeInfo));
    end;
end;


{ if you modify this procedure, fpc_copy must be probably modified as well }
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef VER2_6}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure ArrayRTTI(Data,TypeInfo:Pointer;rttiproc:TRTTIProc);
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  i: SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
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
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  Count:=PArrayInfo(typeInfo)^.ElCount;
  { no elements to process => exit }
  if Count = 0 then
    Exit;
  ElSize:=PArrayInfo(typeInfo)^.Size div Count;
  Info:=PArrayInfo(typeInfo)^.ElInfo;
  { Process elements }
  for I:=0 to Count-1 do
    rttiproc(Data+(I*ElSize),Info);
=======
  i : SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  { Process elements }
  for I:=0 to PArrayInfo(typeInfo)^.ElCount-1 do
    rttiproc(Data+(I*PArrayInfo(typeInfo)^.ElSize),PArrayInfo(typeInfo)^.ElInfo);
>>>>>>> graemeg/cpstrnew
=======
  i : SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  { Process elements }
  for I:=0 to PArrayInfo(typeInfo)^.ElCount-1 do
    rttiproc(Data+(I*PArrayInfo(typeInfo)^.ElSize),PArrayInfo(typeInfo)^.ElInfo);
>>>>>>> graemeg/cpstrnew
=======
  i : SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  { Process elements }
  for I:=0 to PArrayInfo(typeInfo)^.ElCount-1 do
    rttiproc(Data+(I*PArrayInfo(typeInfo)^.ElSize),PArrayInfo(typeInfo)^.ElInfo);
>>>>>>> graemeg/cpstrnew
=======
  i : SizeInt;
begin
  typeInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
  { Process elements }
  for I:=0 to PArrayInfo(typeInfo)^.ElCount-1 do
    rttiproc(Data+(I*PArrayInfo(typeInfo)^.ElSize),PArrayInfo(typeInfo)^.ElInfo);
>>>>>>> origin/cpstrnew
end;
{$endif}

Procedure fpc_Initialize (Data,TypeInfo : pointer);[Public,Alias : 'FPC_INITIALIZE'];  compilerproc;
begin
  case PByte(TypeInfo)^ of
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
      recordrtti(data,typeinfo,@int_initialize);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      variant_init(PVarData(Data)^);
{$endif FPC_HAS_FEATURE_VARIANTS}
  end;
end;


Procedure fpc_finalize (Data,TypeInfo: Pointer);[Public,Alias : 'FPC_FINALIZE'];  compilerproc;
begin
  case PByte(TypeInfo)^ of
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
    tkAstring :
<<<<<<< HEAD
      fpc_AnsiStr_Decr_Ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
    tkUstring :
      fpc_UnicodeStr_Decr_Ref(PPointer(Data)^);
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkWstring :
      fpc_WideStr_Decr_Ref(PPointer(Data)^);
=======
      begin
        fpc_AnsiStr_Decr_Ref(PPointer(Data)^);
        PPointer(Data)^:=nil;
      end;
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef VER2_2}
    tkUstring :
      begin
<<<<<<< HEAD
        fpc_UnicodeStr_Decr_Ref(PPointer(Data)^);
=======
        fpc_AnsiStr_Decr_Ref(PPointer(Data)^);
>>>>>>> graemeg/fixes_2_2
        PPointer(Data)^:=nil;
      end;
  {$endif VER2_2}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkWstring :
      begin
        fpc_WideStr_Decr_Ref(PPointer(Data)^);
        PPointer(Data)^:=nil;
      end;
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
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkArray :
      arrayrtti(data,typeinfo,@int_finalize);
{$ifdef FPC_HAS_FEATURE_OBJECTS}
    tkObject,
{$endif FPC_HAS_FEATURE_OBJECTS}
    tkRecord:
      recordrtti(data,typeinfo,@int_finalize);
    tkInterface:
<<<<<<< HEAD
      Intf_Decr_Ref(PPointer(Data)^);
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
    tkDynArray:
      fpc_dynarray_clear(PPointer(Data)^,TypeInfo);
=======
      begin
        Intf_Decr_Ref(PPointer(Data)^);
        PPointer(Data)^:=nil;
      end;
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
    tkDynArray:
      begin
        fpc_dynarray_decr_ref(PPointer(Data)^,TypeInfo);
        PPointer(Data)^:=nil;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
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
{$endif FPC_HAS_FEATURE_DYNARRAYS}
{$ifdef FPC_HAS_FEATURE_VARIANTS}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    tkVariant:
      variant_clear(PVarData(Data)^);
{$endif FPC_HAS_FEATURE_VARIANTS}
  end;
end;


Procedure fpc_Addref (Data,TypeInfo : Pointer); [Public,alias : 'FPC_ADDREF'];  compilerproc;
begin
  case PByte(TypeInfo)^ of
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
    tkAstring :
      fpc_AnsiStr_Incr_Ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkWstring :
      fpc_WideStr_Incr_Ref(PPointer(Data)^);
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tkUstring :
      fpc_UnicodeStr_Incr_Ref(PPointer(Data)^);
=======
  {$ifndef VER2_2}
    tkUstring :
      fpc_UnicodeStr_Incr_Ref(PPointer(Data)^);
  {$endif VER2_2}
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
  {$ifndef VER2_2}
    tkUstring :
      fpc_UnicodeStr_Incr_Ref(PPointer(Data)^);
  {$endif VER2_2}
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  {$ifndef VER2_2}
    tkUstring :
      fpc_UnicodeStr_Incr_Ref(PPointer(Data)^);
  {$endif VER2_2}
>>>>>>> origin/cpstrnew
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkArray :
      arrayrtti(data,typeinfo,@int_addref);
{$ifdef FPC_HAS_FEATURE_OBJECTS}
    tkobject,
{$endif FPC_HAS_FEATURE_OBJECTS}
    tkrecord :
      recordrtti(data,typeinfo,@int_addref);
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
    tkDynArray:
      fpc_dynarray_incr_ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_DYNARRAYS}
    tkInterface:
      Intf_Incr_Ref(PPointer(Data)^);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      variant_addref(pvardata(Data)^);
{$endif FPC_HAS_FEATURE_DYNARRAYS}
  end;
end;


<<<<<<< HEAD
=======
{ alias for internal use }
{ we use another name else the compiler gets puzzled because of the wrong forward def }
procedure fpc_systemDecRef (Data, TypeInfo : Pointer);[external name 'FPC_DECREF'];

Procedure fpc_DecRef (Data, TypeInfo : Pointer);[Public,alias : 'FPC_DECREF'];  compilerproc;
begin
  case PByte(TypeInfo)^ of
    { see AddRef for comment about below construct (JM) }
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
    tkAstring:
      fpc_AnsiStr_Decr_Ref(PPointer(Data)^);
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkWstring:
      fpc_WideStr_Decr_Ref(PPointer(Data)^);
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
  {$ifndef VER2_2}
    tkUString:
      fpc_UnicodeStr_Decr_Ref(PPointer(Data)^);
  {$endif VER2_2}
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkArray:
      arrayrtti(data,typeinfo,@fpc_systemDecRef);
{$ifdef FPC_HAS_FEATURE_OBJECTS}
    tkobject,
{$endif FPC_HAS_FEATURE_OBJECTS}
    tkrecord:
      recordrtti(data,typeinfo,@fpc_systemDecRef);
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
    tkDynArray:
      fpc_dynarray_decr_ref(PPointer(Data)^,TypeInfo);
{$endif FPC_HAS_FEATURE_DYNARRAYS}
    tkInterface:
      Intf_Decr_Ref(PPointer(Data)^);
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      variant_clear(pvardata(data)^);
{$endif FPC_HAS_FEATURE_VARIANTS}
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ define alias for internal use in the system unit }
Function fpc_Copy_internal (Src, Dest, TypeInfo : Pointer) : SizeInt;[external name 'FPC_COPY'];

Function fpc_Copy (Src, Dest, TypeInfo : Pointer) : SizeInt;[Public,alias : 'FPC_COPY']; compilerproc;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Temp: pbyte;
=======
  ArrayInfo: PArrayInfo;
  Temp : pbyte;
>>>>>>> graemeg/cpstrnew
=======
  ArrayInfo: PArrayInfo;
  Temp : pbyte;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  ArrayInfo: PArrayInfo;
  Temp : pbyte;
>>>>>>> graemeg/cpstrnew
=======
  ArrayInfo: PArrayInfo;
  Temp : pbyte;
>>>>>>> origin/cpstrnew
=======
  namelen : byte;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  copiedsize,
  expectedoffset,
  count,
  offset,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  i: SizeInt;
  info: pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  size,
>>>>>>> graemeg/fixes_2_2
=======
  size,
>>>>>>> origin/fixes_2_2
  i : SizeInt;
  info : pointer;
>>>>>>> graemeg/cpstrnew
begin
  result:=sizeof(pointer);
  case PByte(TypeInfo)^ of
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
    tkAstring:
<<<<<<< HEAD
      fpc_AnsiStr_Assign(PPointer(Dest)^,PPointer(Src)^);
=======
      begin
        fpc_AnsiStr_Incr_Ref(PPointer(Src)^);
        fpc_AnsiStr_Decr_Ref(PPointer(Dest)^);
        PPointer(Dest)^:=PPointer(Src)^;
      end;
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
{$endif FPC_HAS_FEATURE_ANSISTRINGS}
{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_WIDESTRING_EQUAL_UNICODESTRING}
    tkWstring:
      fpc_WideStr_Assign(PPointer(Dest)^,PPointer(Src)^);
<<<<<<< HEAD
<<<<<<< HEAD
  {$endif FPC_WIDESTRING_EQUAL_UNICODESTRING}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    tkUstring:
      fpc_UnicodeStr_Assign(PPointer(Dest)^,PPointer(Src)^);
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkArray:
      begin
        Temp:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
      {$ifdef VER2_6}
        { Process elements }
        for I:=0 to PArrayInfo(Temp)^.ElCount-1 do
          fpc_Copy_internal(Src+(I*PArrayInfo(Temp)^.Size),Dest+(I*PArrayInfo(Temp)^.Size),PArrayInfo(Temp)^.ElInfo);
        Result:=PArrayInfo(Temp)^.Size*PArrayInfo(Temp)^.ElCount;
      {$else}
        Result:=PArrayInfo(Temp)^.Size;
        Count:=PArrayInfo(Temp)^.ElCount;
        { no elements to process => exit }
        if Count = 0 then
          Exit;
        Info:=PArrayInfo(Temp)^.ElInfo;
        copiedsize:=Result div Count;
        Offset:=0;
        { Process elements }
        for I:=1 to Count do
          begin
            fpc_Copy_internal(Src+Offset,Dest+Offset,Info);
            inc(Offset,copiedsize);
          end;
      {$endif}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$ifndef VER2_2}
    tkUstring:
      fpc_UnicodeStr_Assign(PPointer(Dest)^,PPointer(Src)^);
  {$endif VER2_2}
{$endif FPC_HAS_FEATURE_WIDESTRINGS}
    tkArray:
      begin
        ArrayInfo:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);
        { Process elements }
        for I:=0 to ArrayInfo^.ElCount-1 do
          fpc_Copy_internal(Src+(I*ArrayInfo^.ElSize),Dest+(I*ArrayInfo^.ElSize),ArrayInfo^.ElInfo);
        Result:=ArrayInfo^.ElSize*ArrayInfo^.ElCount;
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
=======
    tkArray:
      begin
=======
    tkArray:
      begin
>>>>>>> origin/fixes_2_2
        Temp:=PByte(TypeInfo);
        inc(Temp);
        { Skip Name }
        namelen:=Temp^;
        inc(temp,namelen+1);
        temp:=aligntoptr(temp);

        { Element size }
        size:=PSizeInt(Temp)^;
        inc(Temp,sizeof(Size));

        { Element count }
        Count:=PSizeInt(Temp)^;
        inc(Temp,sizeof(Count));
        Info:=PPointer(Temp)^;
        inc(Temp,sizeof(Info));
        { Process elements }
        for I:=0 to Count-1 do
          fpc_Copy_internal(Src+(I*size),Dest+(I*size),Info);
        Result:=size*count;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;
{$ifdef FPC_HAS_FEATURE_OBJECTS}
    tkobject,
{$endif FPC_HAS_FEATURE_OBJECTS}
    tkrecord:
      begin
<<<<<<< HEAD
        Temp:=aligntoptr(typeInfo+2+PByte(typeInfo)[1]);

        Result:=PRecordInfo(Temp)^.Size;
        Count:=PRecordInfo(Temp)^.Count;
        Inc(PRecordInfo(Temp));
=======
        Temp:=PByte(TypeInfo);
        inc(Temp);
        { Skip Name }
        namelen:=Temp^;
        inc(temp,namelen+1);
        temp:=aligntoptr(temp);

        Result:=plongint(temp)^;

        { Skip size }
        inc(Temp,4);

        { Element count }
        Count:=PLongint(Temp)^;
        inc(Temp,sizeof(longint));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        expectedoffset:=0;
        { Process elements with rtti }
        for i:=1 to count Do
          begin
<<<<<<< HEAD
            Info:=PRecordElement(Temp)^.TypeInfo;
            Offset:=PRecordElement(Temp)^.Offset;
            Inc(PRecordElement(Temp));
            if Offset>expectedoffset then
              move((Src+expectedoffset)^,(Dest+expectedoffset)^,Offset-expectedoffset);
=======
            Info:=PPointer(Temp)^;
            inc(Temp,sizeof(Info));
            Offset:=PLongint(Temp)^;
            if Offset>expectedoffset then
              move((Src+expectedoffset)^,(Dest+expectedoffset)^,Offset-expectedoffset);
            inc(Temp,sizeof(longint));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            copiedsize:=fpc_Copy_internal(Src+Offset,Dest+Offset,Info);
            expectedoffset:=Offset+copiedsize;
          end;
        { elements remaining? }
        if result>expectedoffset then
          move((Src+expectedoffset)^,(Dest+expectedoffset)^,Result-expectedoffset);
      end;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    tkDynArray:
<<<<<<< HEAD
      fpc_dynarray_assign(PPointer(Dest)^,PPointer(Src)^,typeinfo);
{$endif FPC_HAS_FEATURE_DYNARRAYS}
    tkInterface:
      fpc_intf_assign(PPointer(Dest)^,PPointer(Src)^);
=======
      begin
        fpc_dynarray_Incr_Ref(PPointer(Src)^);
        fpc_dynarray_Decr_Ref(PPointer(Dest)^,typeinfo);
        PPointer(Dest)^:=PPointer(Src)^;
      end;
{$endif FPC_HAS_FEATURE_DYNARRAYS}
    tkInterface:
      begin
        Intf_Incr_Ref(PPointer(Src)^);
        Intf_Decr_Ref(PPointer(Dest)^);
        PPointer(Dest)^:=PPointer(Src)^;
      end;
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
{$ifdef FPC_HAS_FEATURE_VARIANTS}
    tkVariant:
      begin
        VarCopyProc(pvardata(dest)^,pvardata(src)^);
        result:=sizeof(tvardata);
      end;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_HAS_FEATURE_VARIANTS}
  end;
end;
=======
  end;
end;
=======
  end;
end;


{ For internal use by the compiler, because otherwise $x- can cause trouble. }
{ Generally disabling extended syntax checking for all compilerprocs may     }
{ have unintended side-effects                                               }
procedure fpc_Copy_proc (Src, Dest, TypeInfo : Pointer);compilerproc; inline;
begin
  fpc_copy_internal(src,dest,typeinfo);
end;
>>>>>>> origin/fixes_2_2


{ For internal use by the compiler, because otherwise $x- can cause trouble. }
{ Generally disabling extended syntax checking for all compilerprocs may     }
{ have unintended side-effects                                               }
procedure fpc_Copy_proc (Src, Dest, TypeInfo : Pointer);compilerproc; inline;
begin
  fpc_copy_internal(src,dest,typeinfo);
end;
>>>>>>> graemeg/fixes_2_2


{ For internal use by the compiler, because otherwise $x- can cause trouble. }
{ Generally disabling extended syntax checking for all compilerprocs may     }
{ have unintended side-effects                                               }
procedure fpc_Copy_proc (Src, Dest, TypeInfo : Pointer);compilerproc; inline;
begin
  fpc_copy_internal(src,dest,typeinfo);
end;


procedure fpc_initialize_array(data,typeinfo : pointer;count : SizeInt); [public,alias:'FPC_INITIALIZE_ARRAY'] compilerproc;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  var
     i, size : SizeInt;
  begin
     size:=RTTISize(typeinfo);
     if size>0 then
       for i:=0 to count-1 do
         int_initialize(data+size*i,typeinfo);
  end;


procedure fpc_finalize_array(data,typeinfo : pointer;count : SizeInt); [Public,Alias:'FPC_FINALIZE_ARRAY'];  compilerproc;
<<<<<<< HEAD
<<<<<<< HEAD
  var
     i, size: SizeInt;
  begin
     size:=RTTISize(typeinfo);
     if size>0 then
=======
  var
     i, size : SizeInt;
  begin
     size:=RTTISize(typeinfo);
     if size>0 then
       for i:=0 to count-1 do
         int_initialize(data+size*i,typeinfo);
  end;


procedure fpc_finalize_array(data,typeinfo : pointer;count : SizeInt); [Public,Alias:'FPC_FINALIZE_ARRAY'];  compilerproc;
  var
     i, size: SizeInt;
  begin
     size:=RTTISize(typeinfo);
     if size>0 then
>>>>>>> graemeg/cpstrnew
=======
  var
     i, size: SizeInt;
  begin
     size:=RTTISize(typeinfo);
     if size>0 then
>>>>>>> graemeg/cpstrnew
=======
  var
     i, size: SizeInt;
  begin
     size:=RTTISize(typeinfo);
     if size>0 then
>>>>>>> origin/cpstrnew
       for i:=0 to count-1 do
         int_finalize(data+size*i,typeinfo);
  end;

procedure fpc_addref_array(data,typeinfo: pointer; count: SizeInt); [public,alias:'FPC_ADDREF_ARRAY']; compilerproc;
  var
    i, size: SizeInt;
  begin
    size:=RTTISize(typeinfo);
    if size>0 then
      for i:=0 to count-1 do
        int_addref(data+size*i,typeinfo);
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
  begin
    size:=RTTISize(typeInfo);
    if size>0 then
      for i:=0 to count-1 do
        fpc_Copy_internal(source+size*i, dest+size*i, typeInfo);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure fpc_decref_array(data,typeinfo: pointer; count: SizeInt); [public,alias:'FPC_DECREF_ARRAY']; compilerproc;
  var
    i, size: SizeInt;
  begin
    size:=RTTISize(typeinfo);
    if size>0 then
      for i:=0 to count-1 do
        int_decref(data+size*i,typeinfo);
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
  end;

