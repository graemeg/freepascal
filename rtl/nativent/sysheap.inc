{
    Basic heap handling for windows platforms

    This file is part of the Free Pascal run time library.
    Copyright (c) 2001-2005 by Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{*****************************************************************************
      OS Memory allocation / deallocation
 ****************************************************************************}

 { In kernel mode we can either use FPC's build in memory manager or we use a
   custom non-chunking manager. The problem with the build in one is that the
   driver developer has far less control of the allocated memory blocks. }

   { memory functions }
{$ifdef KMODE}
   function ExAllocatePoolWithTag(PoolType: LongInt; NumberOfBytes: PtrUInt; Tag: LongWord): Pointer; stdcall; external ntdll name 'ExAllocatePoolWithTag';
   procedure ExFreePoolWithTag(P: Pointer; Tag: LongWord); stdcall; external ntdll name 'ExFreePoolWithTag';
{$else KMODE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   function RtlAllocateHeap(hHeap : THandle; dwFlags : LongWord; Size : PtrUInt): Pointer;
=======
   function RtlAllocateHeap(hHeap : THandle; dwFlags : LongWord; Size : Longword): Pointer;
>>>>>>> graemeg/cpstrnew
=======
   function RtlAllocateHeap(hHeap : THandle; dwFlags : LongWord; Size : Longword): Pointer;
>>>>>>> graemeg/cpstrnew
=======
   function RtlAllocateHeap(hHeap : THandle; dwFlags : LongWord; Size : Longword): Pointer;
>>>>>>> graemeg/cpstrnew
=======
   function RtlAllocateHeap(hHeap : THandle; dwFlags : LongWord; Size : Longword): Pointer;
>>>>>>> origin/cpstrnew
     stdcall; external ntdll name 'RtlAllocateHeap';
   function  RtlFreeHeap(hHeap : THandle; dwFlags : LongWord; MemoryPointer : Pointer): Boolean;
     stdcall; external ntdll name 'RtlFreeHeap';
   function RtlCreateHeap(Flags: LongWord; Base: Pointer; SizeToReserve: PtrUInt;
     SizeToCommit: PtrUInt; Lock: Pointer; Parameters: Pointer): THandle;
     stdcall; external ntdll name 'RtlCreateHeap';

var
  SysHeap: THandle = 0;

procedure PrepareSysHeap;
begin
  if IsLibrary then
    // create a new heap (flag is HEAP_GROWABLE)
    SysHeap := RtlCreateHeap(2, Nil, 65534, 65534, Nil, Nil)
  else
    // use the heap passed on startup
    SysHeap := THandle(PSimplePEB(CurrentPEB)^.ProcessHeap);
end;

{$endif KMODE}

{$ifndef KMODE}

// default memory manager

function SysOSAlloc(size: ptruint): pointer;
begin
  if SysHeap = 0 then
    PrepareSysHeap;
  SysOSAlloc := RtlAllocateHeap(SysHeap, 0, size);
end;

{$define HAS_SYSOSFREE}

procedure SysOSFree(p: pointer; size: ptruint);
begin
  // if heap isn't set, then nothing was allocated
  if SysHeap <> 0 then
    RtlFreeHeap(SysHeap, 0, p);
end;

{$else KMODE}

// custom non-chunking memory manager for kernel mode

// memory layout:
//   <PtrUInt>: Size of reserved chunk
//   <Tag>:     Tag that was used in ExAllocateFromPoolWithTag (needed in free)
//   <...>:     Userdata

function SysGetMem(Size: PtrUInt): Pointer;
var
  tag: LongWord;
  pooltype: LongInt;
begin
  if HeapUsePagedPool then
    pooltype := 1
  else
    pooltype := 0;
  tag := Ord(HeapPoolTag[1]) + Ord(HeapPoolTag[2]) shl 8 +
         Ord(HeapPoolTag[3]) shl 16 + Ord(HeapPoolTag[4]) shl 24;
  // the kernel keeps track of our memory, but there's no way to ask it
  // so we need to track the size by ourself
  SysGetMem := ExAllocatePoolWithTag(pooltype, Size + SizeOf(PtrUInt) + SizeOf(LongWord), tag);
  // save the size
  PPtrUInt(SysGetMem)^ := Size;
  SysGetMem := SysGetMem + SizeOf(PtrUInt);
  // save the tag
  PLongWord(SysGetMem)^ := tag;
  SysGetMem := SysGetMem + SizeOf(LongWord);
end;

function SysFreeMem(p: Pointer): PtrUInt;
var
  tag: PLongWord;
begin
  tag := p - SizeOf(LongWord);
  // we need to pass the tag we used to allocate the memory (else: BSOD)
  ExFreePoolWithTag(p - SizeOf(PtrUInt) - SizeOf(LongWord), tag^);
  SysFreeMem := 0;
end;

function SysFreeMemSize(p: Pointer; Size: PtrUInt): PtrUInt;
begin
  SysFreeMemSize := 0;
  if (Size > 0) and (p <> nil) then
    Result := SysFreeMem(p);
end;

Function SysAllocMem(Size: PtrUInt): Pointer;
begin
  SysAllocMem := SysGetMem(Size);
  if SysAllocMem <> nil then
    FillChar(SysAllocMem^, Size, 0);
end;

Function SysReAllocMem (var p: pointer; Size: PtrUInt): Pointer;
begin
  SysReAllocMem := SysGetMem(Size);
  Move(p^, SysReAllocMem^, Size);
  p := SysReAllocMem;
end;

function SysTryResizeMem(var p: Pointer; size: PtrUInt): Boolean;
var
  res: pointer;
begin
  res := SysGetMem(Size);
  SysTryResizeMem := (res <> Nil) or (Size = 0);
  if SysTryResizeMem then
    p := res;
end;

function SysMemSize(P : pointer): PtrUInt;
begin
  SysMemSize := PPtrUInt(P - SizeOf(PtrUInt) - SizeOf(LongWord))^;
end;

function SysGetHeapStatus: THeapStatus;
begin
  FillChar(SysGetHeapStatus, SizeOf(SysGetHeapStatus), 0);
end;

function SysGetFPCHeapStatus: TFPCHeapStatus;
begin
  FillChar(SysGetFPCHeapStatus, SizeOf(SysGetHeapStatus), 0);
end;

{$endif KMODE}
