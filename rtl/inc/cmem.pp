{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999 by Michael Van Canneyt, member of the
    Free Pascal development team

    Implements a memory manager that uses the C memory management.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit cmem;

interface

Const

{$if defined(go32v2) or defined(wii)}
  {$define USE_STATIC_LIBC}
{$endif}

{$if defined(win32)}
  LibName = 'msvcrt';
{$elseif defined(win64)}
  LibName = 'msvcrt';
{$elseif defined(wince)}
  LibName = 'coredll';
{$elseif defined(netware)}
  LibName = 'clib';
{$elseif defined(netwlibc)}
  LibName = 'libc';
{$elseif defined(macos)}
  LibName = 'StdCLib';
{$elseif defined(beos)}
  LibName = 'root';
{$else}
  LibName = 'c';
{$endif}

{$ifdef USE_STATIC_LIBC}
  {$linklib c}
Function malloc (Size : ptruint) : Pointer;cdecl; external;
Procedure free (P : pointer); cdecl; external;
function realloc (P : Pointer; Size : ptruint) : pointer;cdecl; external;
Function calloc (unitSize,UnitCount : ptruint) : pointer;cdecl; external;
{$else not USE_STATIC_LIBC}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function Malloc (Size : ptruint) : Pointer; cdecl; external LibName name 'malloc';
Procedure Free (P : pointer); cdecl; external LibName name 'free';
function ReAlloc (P : Pointer; Size : ptruint) : pointer; cdecl; external LibName name 'realloc';
Function CAlloc (unitSize,UnitCount : ptruint) : pointer; cdecl; external LibName name 'calloc';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Function Malloc (Size : ptruint) : Pointer; {$ifdef win32}stdcall{$else}cdecl{$endif}; external LibName name 'malloc';
Procedure Free (P : pointer); {$ifdef win32}stdcall{$else}cdecl{$endif}; external LibName name 'free';
function ReAlloc (P : Pointer; Size : ptruint) : pointer; {$ifdef win32}stdcall{$else}cdecl{$endif}; external LibName name 'realloc';
Function CAlloc (unitSize,UnitCount : ptruint) : pointer; {$ifdef win32}stdcall{$else}cdecl{$endif}; external LibName name 'calloc';
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
{$endif not USE_STATIC_LIBC}

implementation

Function CGetMem  (Size : ptruint) : Pointer;

begin
  CGetMem:=Malloc(Size+sizeof(ptruint));
  if (CGetMem <> nil) then
    begin
      Pptruint(CGetMem)^ := size;
      inc(CGetMem,sizeof(ptruint));
    end;
end;

Function CFreeMem (P : pointer) : ptruint;

begin
  if (p <> nil) then
    dec(p,sizeof(ptruint));
  Free(P);
  CFreeMem:=0;
end;

Function CFreeMemSize(p:pointer;Size:ptruint):ptruint;

begin
  if size<=0 then
    exit;
  if (p <> nil) then
    begin
      if (size <> Pptruint(p-sizeof(ptruint))^) then
        runerror(204);
    end;
  CFreeMemSize:=CFreeMem(P);
end;

Function CAllocMem(Size : ptruint) : Pointer;

begin
  CAllocMem:=calloc(Size+sizeof(ptruint),1);
  if (CAllocMem <> nil) then
    begin
      Pptruint(CAllocMem)^ := size;
      inc(CAllocMem,sizeof(ptruint));
    end;
end;

Function CReAllocMem (var p:pointer;Size:ptruint):Pointer;

begin
  if size=0 then
    begin
      if p<>nil then
        begin
          dec(p,sizeof(ptruint));
          free(p);
          p:=nil;
        end;
    end
  else
    begin
      inc(size,sizeof(ptruint));
      if p=nil then
        p:=malloc(Size)
      else
        begin
          dec(p,sizeof(ptruint));
          p:=realloc(p,size);
        end;
      if (p <> nil) then
        begin
          Pptruint(p)^ := size-sizeof(ptruint);
          inc(p,sizeof(ptruint));
        end;
    end;
  CReAllocMem:=p;
end;

Function CMemSize (p:pointer): ptruint;

begin
  CMemSize:=Pptruint(p-sizeof(ptruint))^;
end;

function CGetHeapStatus:THeapStatus;

var res: THeapStatus;

begin
  fillchar(res,sizeof(res),0);
  CGetHeapStatus:=res;
end;

function CGetFPCHeapStatus:TFPCHeapStatus;

begin
  fillchar(CGetFPCHeapStatus,sizeof(CGetFPCHeapStatus),0);
end;

Const
 CMemoryManager : TMemoryManager =
    (
      NeedLock : false;
      GetMem : @CGetmem;
      FreeMem : @CFreeMem;
      FreememSize : @CFreememSize;
      AllocMem : @CAllocMem;
      ReallocMem : @CReAllocMem;
      MemSize : @CMemSize;
      InitThread : nil;
      DoneThread : nil;
      RelocateHeap : nil;
      GetHeapStatus : @CGetHeapStatus;
      GetFPCHeapStatus: @CGetFPCHeapStatus;
    );

Var
  OldMemoryManager : TMemoryManager;

Initialization
  GetMemoryManager (OldMemoryManager);
  SetMemoryManager (CmemoryManager);

Finalization
  SetMemoryManager (OldMemoryManager);
end.
