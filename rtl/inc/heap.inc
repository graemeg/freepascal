{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    functions for heap management in the data segment

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************}
{ Do not use standard memory manager }
{ $define HAS_MEMORYMANAGER}

{ Memory manager }
{$ifndef EMBEDDED}
const
  MemoryManager: TMemoryManager = (
    NeedLock: false;  // Obsolete
    GetMem: {$ifndef FPC_NO_DEFAULT_HEAP}@SysGetMem{$else}nil{$endif};
    FreeMem: {$ifndef FPC_NO_DEFAULT_HEAP}@SysFreeMem{$else}nil{$endif};
    FreeMemSize: {$ifndef FPC_NO_DEFAULT_HEAP}@SysFreeMemSize{$else}nil{$endif};
    AllocMem: {$ifndef FPC_NO_DEFAULT_HEAP}@SysAllocMem{$else}nil{$endif};
    ReAllocMem: {$ifndef FPC_NO_DEFAULT_HEAP}@SysReAllocMem{$else}nil{$endif};
    MemSize: {$ifndef FPC_NO_DEFAULT_HEAP}@SysMemSize{$else}nil{$endif};
    InitThread: nil;
    DoneThread: nil;
    RelocateHeap: nil;
    GetHeapStatus: {$ifndef FPC_NO_DEFAULT_HEAP}@SysGetHeapStatus{$else}nil{$endif};
    GetFPCHeapStatus: {$ifndef FPC_NO_DEFAULT_HEAP}@SysGetFPCHeapStatus{$else}nil{$endif};
  );
{$else}
{$ifndef FPC_IN_HEAPMGR}
const
  MemoryManager: TMemoryManager = (
    NeedLock: false;  // Obsolete
    GetMem: nil;
    FreeMem: nil;
    FreeMemSize: nil;
    AllocMem: nil;
    ReAllocMem: nil;
    MemSize: nil;
    InitThread: nil;
    DoneThread: nil;
    RelocateHeap: nil;
    GetHeapStatus: nil;
    GetFPCHeapStatus: nil;
  );public name 'FPC_SYSTEM_MEMORYMANAGER';
{$endif FPC_IN_HEAPMGR}
{$endif EMBEDDED}


{ Try to find the best matching block in general freelist }
{ define BESTMATCH}

{ DEBUG: Dump info when the heap needs to grow }
{ define DUMPGROW}

{ Memory profiling: at moment in time of max heap size usage,
  keep statistics of number of each size allocated 
  (with 16 byte granularity) }
{ define DUMP_MEM_USAGE}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef DUMP_MEM_USAGE}
  {$define SHOW_MEM_USAGE}
{$endif}

{$ifndef HAS_MEMORYMANAGER}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef HAS_MT_MEMORYMANAGER}
  {$define HAS_MEMORYMANAGER}
{$endif HAS_MT_MEMORYMANAGER}

{$ifdef DUMP_MEM_USAGE}
  {$define SHOW_MEM_USAGE}
{$endif}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
{$ifdef CPU64}
  blocksize    = 32;  { at least size of freerecord }
  blockshift   = 5;   { shr value for blocksize=2^blockshift}
  maxblocksize = 512+blocksize; { 1024+8 needed for heaprecord }
{$else}
  blocksize    = 16;  { at least size of freerecord }
  blockshift   = 4;   { shr value for blocksize=2^blockshift}
  maxblocksize = 512+blocksize; { 1024+8 needed for heaprecord }
{$endif}
  maxblockindex = maxblocksize div blocksize; { highest index in array of lists of memchunks }

  { common flags }
  fixedsizeflag  = 1;   { flag if the block is of fixed size }
  { memchunk var flags }
  usedflag       = 2;   { flag if the block is used or not }
  lastblockflag  = 4;   { flag if the block is the last in os chunk }
  firstblockflag = 8;   { flag if the block is the first in os chunk }
  { os chunk flags }
  ocrecycleflag  = 1;
  { above flags stored in size field }
  sizemask = not(blocksize-1);
  fixedoffsetshift = 12;
  fixedsizemask = sizemask and ((1 shl fixedoffsetshift) - 1);
  { After how many successive allocations of oschunks for fixed freelist
    purposes should we double the size of locgrowheapsizesmall for the
    current thread. Since the allocations of oschunks are added together for
    all blocksizes, this is only a fuzzy indication of when the size will be
    doubled rather than a hard and fast boundary. }
  fixedallocthreshold = (maxblocksize shr blockshift) * 8;
  { maximum size to which locgrowheapsizesmall can grow }
  maxgrowheapsizesmall = 256*1024;

{****************************************************************************}

{$ifdef DUMPGROW}
  {$define DUMPBLOCKS}
{$endif}

{
  We use 'fixed' size chunks for small allocations,
  and os chunks with variable sized blocks for big
  allocations.

  * a block is an area allocated by user
  * a chunk is a block plus our bookkeeping
  * an os chunk is a collection of chunks

  Memory layout:
    fixed:                         < chunk size > [ ... user data ... ]
    variable:  < prev chunk size > < chunk size > [ ... user data ... ]

  When all chunks in an os chunk are free, we keep a few around
  but otherwise it will be freed to the OS.

  Fixed os chunks can be converted to variable os chunks and back
  (if not too big). To prevent repeated conversion overhead in case
  of user freeing/allocing same or a small set of sizes, we only do
  the conversion to the new fixed os chunk size format after we
  reuse the os chunk for another fixed size, or variable. Note that
  while the fixed size os chunk is on the freelists.oslist, it is also 
  still present in a freelists.fixedlists, therefore we can easily remove 
  the os chunk from the freelists.oslist if this size is needed again; we 
  don't need to search freelists.oslist in alloc_oschunk, since it won't
  be present anymore if alloc_oschunk is reached. Note that removing
  from the freelists.oslist is not really done, only the recycleflag is
  set, allowing to reset the flag easily. alloc_oschunk will clean up
  the list while passing over it, that was a slow function anyway.
}

<<<<<<< HEAD
=======
{$ifndef HAS_MEMORYMANAGER}

{ 
  We use 'fixed' size chunks for small allocations,
  and os chunks with variable sized blocks for big
  allocations.

  * a block is an area allocated by user
  * a chunk is a block plus our bookkeeping
  * an os chunk is a collection of chunks

  Memory layout:
    fixed:                         < chunk size > [ ... user data ... ]
    variable:  < prev chunk size > < chunk size > [ ... user data ... ]

  When all chunks in an os chunk are free, we keep a few around
  but otherwise it will be freed to the OS.

  Fixed os chunks can be converted to variable os chunks and back
  (if not too big). To prevent repeated conversion overhead in case
  of user freeing/allocing same or a small set of sizes, we only do
  the conversion to the new fixed os chunk size format after we
  reuse the os chunk for another fixed size, or variable. Note that
  while the fixed size os chunk is on the freeoslist, it is also 
  still present in a freelists_fixed, therefore we can easily remove 
  the os chunk from the freeoslist if this size is needed again; we 
  don't need to search freeoslist in alloc_oschunk, since it won't
  be present anymore if alloc_oschunk is reached. Note that removing
  from the freeoslist is not really done, only the recycleflag is
  set, allowing to reset the flag easily. alloc_oschunk will clean up
  the list while passing over it, that was a slow function anyway.
}

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type
  pfreelists = ^tfreelists;

  poschunk = ^toschunk;
  toschunk = record
<<<<<<< HEAD
    size : 0..high(ptrint); {Cannot be ptruint because used field is signed.}
    next_free : poschunk;
    prev_any : poschunk;
    next_any : poschunk;
    used : ptrint;          { 0: free, >0: fixed, -1: var }
    freelists : pfreelists;
    { padding inserted automatically by alloc_oschunk }
  end;

  ppmemchunk_fixed = ^pmemchunk_fixed;
=======
    size : ptrint;
    next : poschunk;
    used : ptrint;
    { padding inserted automatically by alloc_oschunk }
  end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  pmemchunk_fixed = ^tmemchunk_fixed;
  tmemchunk_fixed = record
    { aligning is done automatically in alloc_oschunk }
    size  : ptruint;
    next_fixed,
    prev_fixed : pmemchunk_fixed;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  ppmemchunk_var = ^pmemchunk_var;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  pmemchunk_var = ^tmemchunk_var;
  tmemchunk_var = record
    prevsize : ptruint;
    freelists : pfreelists;
    size  : ptruint;
    next_var,
    prev_var  : pmemchunk_var;
  end;

  { ``header'', ie. size of structure valid when chunk is in use }
  { should correspond to tmemchunk_var_hdr structure starting with the
    last field. Reason is that the overlap is starting from the end of the
    record. }
  tmemchunk_fixed_hdr = record
    { aligning is done automatically in alloc_oschunk }
    size : ptruint;
  end;
  tmemchunk_var_hdr = record
    prevsize : ptruint;
    freelists : pfreelists;
    size : ptruint;
  end;

  pfpcheapstatus = ^tfpcheapstatus;

  tfixedfreelists = array[1..maxblockindex] of pmemchunk_fixed;

  tfreelists = record
    oslist : poschunk;      { os chunks free, available for use }
    fixedlists : tfixedfreelists;
    oscount : dword;        { number of os chunks on oslist }
    { how many oschunks have been allocated in this thread since
      the last time we doubled the locgrowheapsizesmall size }
    fixedallocated: dword;
    { the size of oschunks allocated for fixed allocations in this thread;
      initialised on thread creation with the global growheapsizesmall setting }
    locgrowheapsizesmall: ptruint;
    oslist_all : poschunk;  { all os chunks allocated }
    varlist : pmemchunk_var;
    { chunks waiting to be freed from other thread }
    waitfixed : pmemchunk_fixed;
    waitvar : pmemchunk_var;
    { heap statistics }
    internal_status : TFPCHeapStatus;
  end;

const
  fixedfirstoffset = ((sizeof(toschunk) + sizeof(tmemchunk_fixed_hdr) + $f) 
      and not $f) - sizeof(tmemchunk_fixed_hdr);
  varfirstoffset = ((sizeof(toschunk) + sizeof(tmemchunk_var_hdr) + $f) 
      and not $f) - sizeof(tmemchunk_var_hdr);
{$ifdef BESTMATCH}
  matcheffort = high(longint);
{$else}
  matcheffort = 10;
{$endif}

var
  orphaned_freelists : tfreelists;
{$ifdef FPC_HAS_FEATURE_THREADING}
  heap_lock : trtlcriticalsection;
  heap_lock_use : integer;
{$endif}
threadvar
  freelists : tfreelists;

<<<<<<< HEAD
{$ifdef DUMP_MEM_USAGE}
const
  sizeusageshift = 4;
  sizeusageindex = 2049;
  sizeusagesize = sizeusageindex shl sizeusageshift;
type
  tsizeusagelist = array[0..sizeusageindex] of longint;
threadvar
  sizeusage, maxsizeusage: tsizeusagelist;
{$endif}
=======
  freelists_fixed    : tfreelists;
  freelist_var       : pmemchunk_var;
  freeoslist         : poschunk;
  freeoslistend      : poschunk;
  freeoslistcount    : dword;
>>>>>>> graemeg/fixes_2_2

{$ifdef DUMP_MEM_USAGE}
const
  sizeusageshift = 4;
  sizeusageindex = 2049;
  sizeusagesize = sizeusageindex shl sizeusageshift;
type
  tsizeusagelist = array[0..sizeusageindex] of longint;
var
  sizeusage, maxsizeusage: tsizeusagelist;
{$endif}

{$ifdef DUMP_MEM_USAGE}
const
  sizeusageshift = 4;
  sizeusageindex = 2049;
  sizeusagesize = sizeusageindex shl sizeusageshift;
type
  tsizeusagelist = array[0..sizeusageindex] of longint;
var
  sizeusage, maxsizeusage: tsizeusagelist;
{$endif}

{$endif HAS_MEMORYMANAGER}

{*****************************************************************************
                             Memory Manager
*****************************************************************************}

{$ifndef FPC_IN_HEAPMGR}
procedure GetMemoryManager(var MemMgr:TMemoryManager);
begin
  MemMgr := MemoryManager;
end;


procedure SetMemoryManager(const MemMgr:TMemoryManager);
begin
  MemoryManager := MemMgr;
end;

function IsMemoryManagerSet:Boolean;
begin
{$ifdef HAS_MEMORYMANAGER}
  Result:=false;
{$else HAS_MEMORYMANAGER}
  IsMemoryManagerSet := (MemoryManager.GetMem<>@SysGetMem) 
    or (MemoryManager.FreeMem<>@SysFreeMem);
{$endif HAS_MEMORYMANAGER}
end;

{$ifdef FPC_HAS_FEATURE_HEAP}
procedure GetMem(Out p:pointer;Size:ptruint);
begin
  p := MemoryManager.GetMem(Size);
end;

procedure GetMemory(Out p:pointer;Size:ptruint);
begin
  GetMem(p,size);
end;

procedure FreeMem(p:pointer;Size:ptruint);
begin
  MemoryManager.FreeMemSize(p,Size);
end;

procedure FreeMemory(p:pointer;Size:ptruint);
begin
  FreeMem(p,size);
end;


function GetHeapStatus:THeapStatus;
begin
  Result:=MemoryManager.GetHeapStatus();
end;


function GetFPCHeapStatus:TFPCHeapStatus;
begin
  Result:=MemoryManager.GetFPCHeapStatus();
end;


function MemSize(p:pointer):ptruint;
begin
  MemSize := MemoryManager.MemSize(p);
end;


{ Delphi style }
function FreeMem(p:pointer):ptruint;
begin
  FreeMem := MemoryManager.FreeMem(p);
end;

function FreeMemory(p:pointer):ptruint; cdecl;
begin
  FreeMemory := FreeMem(p);
end;

function GetMem(size:ptruint):pointer;
begin
  GetMem := MemoryManager.GetMem(Size);
end;

function GetMemory(size:ptruint):pointer; cdecl;
begin
  GetMemory := GetMem(size);
end;

function AllocMem(Size:ptruint):pointer;
begin
  AllocMem := MemoryManager.AllocMem(size);
end;


function ReAllocMem(var p:pointer;Size:ptruint):pointer;
begin
  ReAllocMem := MemoryManager.ReAllocMem(p,size);
end;

function ReAllocMemory(p:pointer;Size:ptruint):pointer; cdecl;
begin
  ReAllocMemory := ReAllocMem(p,size);
end;


{ Needed for calls from Assembler }
function fpc_getmem(size:ptruint):pointer;compilerproc;[public,alias:'FPC_GETMEM'];
begin
  fpc_GetMem := MemoryManager.GetMem(size);
end;

procedure fpc_freemem(p:pointer);compilerproc;[public,alias:'FPC_FREEMEM'];
begin
  MemoryManager.FreeMem(p);
end;
{$endif FPC_HAS_FEATURE_HEAP}
{$endif FPC_IN_HEAPMGR}

{$if defined(FPC_HAS_FEATURE_HEAP) or defined(FPC_IN_HEAPMGR)}
{$ifndef HAS_MEMORYMANAGER}
{*****************************************************************************
                               GetHeapStatus
*****************************************************************************}

function SysGetFPCHeapStatus:TFPCHeapStatus;
var
  status: pfpcheapstatus;
begin
  status := @freelists.internal_status;
  status^.CurrHeapFree := status^.CurrHeapSize - status^.CurrHeapUsed;
  result := status^;
end;

function SysGetHeapStatus :THeapStatus;
var
  status: pfpcheapstatus;
begin
  status := @freelists.internal_status;
  status^.CurrHeapFree := status^.CurrHeapSize - status^.CurrHeapUsed;
  result.TotalAllocated   :=status^.CurrHeapUsed;
  result.TotalFree        :=status^.CurrHeapFree;
  result.TotalAddrSpace   :=status^.CurrHeapSize;
  result.TotalUncommitted :=0;
  result.TotalCommitted   :=0;
  result.FreeSmall        :=0;
  result.FreeBig          :=0;
  result.Unused           :=0;
  result.Overhead         :=0;
  result.HeapErrorCode    :=0;
end;


{$ifdef DUMPBLOCKS}   // TODO
procedure DumpBlocks(loc_freelists: pfreelists);
var
  s,i,j : ptruint;
  hpfixed  : pmemchunk_fixed;
  hpvar  : pmemchunk_var;
begin
  { fixed freelist }
  for i := 1 to maxblockindex do
   begin
     hpfixed := loc_freelists^.fixedlists[i];
     j := 0;
     while assigned(hpfixed) do
      begin
        inc(j);
        hpfixed := hpfixed^.next_fixed;
      end;
     writeln('Block ',i*blocksize,': ',j);
   end;
  { var freelist }
  hpvar := loc_freelists^.varlist;
  j := 0;
  s := 0;
  while assigned(hpvar) do
   begin
     inc(j);
     if hpvar^.size>s then
      s := hpvar^.size;
     hpvar := hpvar^.next_var;
   end;
  writeln('Variable: ',j,' maxsize: ',s);
end;
{$endif}


{*****************************************************************************
                                Forwards
*****************************************************************************}

procedure finish_waitfixedlist(loc_freelists: pfreelists); forward;
procedure finish_waitvarlist(loc_freelists: pfreelists); forward;
function  try_finish_waitfixedlist(loc_freelists: pfreelists): boolean; forward;
procedure try_finish_waitvarlist(loc_freelists: pfreelists); forward;

{*****************************************************************************
                                List adding/removal
*****************************************************************************}

procedure append_to_list_var(pmc: pmemchunk_var); inline;
var
  varlist: ppmemchunk_var;
begin
  varlist := @pmc^.freelists^.varlist;
  pmc^.prev_var := nil;
  pmc^.next_var := varlist^;
  if varlist^<>nil then
    varlist^^.prev_var := pmc;
  varlist^ := pmc;
end;

{$ifdef HEAP_DEBUG}

<<<<<<< HEAD
<<<<<<< HEAD
function find_fixed_mc(loc_freelists: pfreelists; chunkindex: ptruint; 
  pmc: pmemchunk_fixed): boolean;
var
  pmc_temp: pmemchunk_fixed;
begin
  pmc_temp := loc_freelists^.fixedlists[chunkindex];
=======
=======
>>>>>>> origin/fixes_2_2
function find_fixed_mc(chunkindex: ptrint; pmc: pmemchunk_fixed): boolean;
var
  pmc_temp: pmemchunk_fixed;
begin
  pmc_temp := freelists_fixed[chunkindex];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  while pmc_temp <> nil do
  begin
    if pmc_temp = pmc then exit(true);
    pmc_temp := pmc_temp^.next_fixed;
  end;
  result := false;
end;

{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
procedure remove_from_list_fixed(pmc: pmemchunk_fixed; fixedlist: ppmemchunk_fixed); inline;
=======
=======
>>>>>>> origin/fixes_2_2
procedure remove_from_list_fixed(blockindex: ptrint; pmc: pmemchunk_fixed); inline;
>>>>>>> graemeg/fixes_2_2
begin
  if assigned(pmc^.next_fixed) then
    pmc^.next_fixed^.prev_fixed := pmc^.prev_fixed;
  if assigned(pmc^.prev_fixed) then
    pmc^.prev_fixed^.next_fixed := pmc^.next_fixed
  else
    fixedlist^ := pmc^.next_fixed;
end;

procedure remove_from_list_var(pmc: pmemchunk_var); inline;
begin
  if assigned(pmc^.next_var) then
    pmc^.next_var^.prev_var := pmc^.prev_var;
  if assigned(pmc^.prev_var) then
    pmc^.prev_var^.next_var := pmc^.next_var
  else
    pmc^.freelists^.varlist := pmc^.next_var;
end;

procedure remove_freed_fixed_chunks(poc: poschunk);
  { remove all fixed chunks from the fixed free list, as this os chunk
    is going to be used for other purpose }
var
  pmc, pmc_end: pmemchunk_fixed;
  fixedlist: ppmemchunk_fixed;
  chunksize: ptruint;
begin
  { exit if this is a var size os chunk, function only applicable to fixed size }
  if poc^.used < 0 then
    exit;
  pmc := pmemchunk_fixed(pointer(poc)+fixedfirstoffset);
  chunksize := pmc^.size and fixedsizemask;
  pmc_end := pmemchunk_fixed(pointer(poc)+(poc^.size and sizemask)-chunksize);
  fixedlist := @poc^.freelists^.fixedlists[chunksize shr blockshift];
  repeat
    remove_from_list_fixed(pmc, fixedlist);
    pmc := pointer(pmc)+chunksize;
  until pmc > pmc_end;
end;

procedure free_oschunk(loc_freelists: pfreelists; poc: poschunk);
var
  pocsize: ptruint;
begin
  remove_freed_fixed_chunks(poc);
  if assigned(poc^.prev_any) then
    poc^.prev_any^.next_any := poc^.next_any
  else
    loc_freelists^.oslist_all := poc^.next_any;
  if assigned(poc^.next_any) then
    poc^.next_any^.prev_any := poc^.prev_any;
  if poc^.used >= 0 then
    dec(loc_freelists^.fixedallocated);
  pocsize := poc^.size and sizemask;
  dec(loc_freelists^.internal_status.currheapsize, pocsize);
  SysOSFree(poc, pocsize);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure append_to_oslist(poc: poschunk);
var
  loc_freelists: pfreelists;
begin
  loc_freelists := poc^.freelists;
  { check if already on list }
  if (poc^.size and ocrecycleflag) <> 0 then
    begin
      inc(loc_freelists^.oscount);
=======
=======
>>>>>>> origin/fixes_2_2
procedure remove_all_from_list_fixed(chunksize: ptrint; poc: poschunk);
var
  pmc, pmc_end: pmemchunk_fixed;
  chunkindex: ptrint;
begin
  pmc := pmemchunk_fixed(pointer(poc)+fixedfirstoffset);
  pmc_end := pmemchunk_fixed(pointer(poc)+(poc^.size and sizemask)-chunksize);
  chunkindex := chunksize shr blockshift;
  repeat
    remove_from_list_fixed(chunkindex, pmc);
    pmc := pointer(pmc)+chunksize;
  until pmc > pmc_end;
end;

procedure append_to_oslist(poc: poschunk; chunksize: ptrint);
var
  pocsize: ptrint;
begin
  { check if already on list }
  if (poc^.size and ocrecycleflag) <> 0 then
    begin
      inc(freeoslistcount);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      poc^.size := poc^.size and not ocrecycleflag;
      exit;
    end;
  { decide whether to free block or add to list }
{$ifdef HAS_SYSOSFREE}
<<<<<<< HEAD
<<<<<<< HEAD
  if (loc_freelists^.oscount >= MaxKeptOSChunks) or
     ((poc^.size and sizemask) > growheapsize2) then
    begin
      free_oschunk(loc_freelists, poc);
=======
=======
>>>>>>> origin/fixes_2_2
  pocsize := poc^.size and sizemask;
  if (freeoslistcount >= MaxKeptOSChunks) or
     (pocsize > growheapsize2) then
    begin
      if chunksize <> 0 then
        remove_all_from_list_fixed(chunksize, poc);
      dec(internal_status.currheapsize, pocsize);
      SysOSFree(poc, pocsize);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end
  else
    begin
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
      poc^.next_free := loc_freelists^.oslist;
      loc_freelists^.oslist := poc;
      inc(loc_freelists^.oscount);
=======
=======
>>>>>>> origin/fixes_2_2
      if freeoslistend = nil then
        freeoslistend := poc
      else
        freeoslistend^.next := poc;
      freeoslist := poc;
      inc(freeoslistcount);
>>>>>>> graemeg/fixes_2_2
{$ifdef HAS_SYSOSFREE}
    end;
{$endif}
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
procedure clear_oschunk_on_freelist_fixed_flag(poc: poschunk); inline;
  { prevent thinking this os chunk is on the fixed freelists }
begin
  pmemchunk_fixed(pointer(poc) + fixedfirstoffset)^.size := 0;
end;

>>>>>>> graemeg/fixes_2_2
procedure append_to_oslist_var(pmc: pmemchunk_var);
var
  poc: poschunk;
begin
  // block eligable for freeing
  poc := pointer(pmc)-varfirstoffset;
  remove_from_list_var(pmc);
<<<<<<< HEAD
<<<<<<< HEAD
  append_to_oslist(poc);
end;

procedure modify_oschunk_freelists(poc: poschunk; new_freelists: pfreelists);
var
  pmcv: pmemchunk_var;
begin
  poc^.freelists := new_freelists;
  { only if oschunk contains var memchunks, we need additional assignments }
  if poc^.used <> -1 then exit;
  pmcv := pmemchunk_var(pointer(poc)+varfirstoffset);
  repeat
    pmcv^.freelists := new_freelists;
    if (pmcv^.size and lastblockflag) <> 0 then
      break;
    pmcv := pmemchunk_var(pointer(pmcv)+(pmcv^.size and sizemask));
  until false;
end;

function modify_freelists(loc_freelists, new_freelists: pfreelists): poschunk;
var
  poc: poschunk;
begin
  poc := loc_freelists^.oslist_all;
  if assigned(poc) then
  begin
    repeat
      { fixed and var freelist for orphaned freelists do not need maintenance }
      { we assume the heap is not severely fragmented at thread exit }
      modify_oschunk_freelists(poc, new_freelists);
      if not assigned(poc^.next_any) then
        exit(poc);
      poc := poc^.next_any;
    until false;
  end;
  modify_freelists := nil;
=======
  clear_oschunk_on_freelist_fixed_flag(poc);
  append_to_oslist(poc, 0);
>>>>>>> graemeg/fixes_2_2
=======
  clear_oschunk_on_freelist_fixed_flag(poc);
  append_to_oslist(poc, 0);
>>>>>>> origin/fixes_2_2
end;

{*****************************************************************************
                         Split block
*****************************************************************************}

function split_block(pcurr: pmemchunk_var; size: ptruint): ptruint;
var
  pcurr_tmp : pmemchunk_var;
  size_flags, oldsize, sizeleft: ptruint;
begin
  size_flags := pcurr^.size;
  oldsize := size_flags and sizemask;
  sizeleft := oldsize-size;
  if sizeleft>=sizeof(tmemchunk_var) then
    begin
      pcurr_tmp := pmemchunk_var(pointer(pcurr)+size);
      { update prevsize of block to the right }
      if (size_flags and lastblockflag) = 0 then
        pmemchunk_var(pointer(pcurr)+oldsize)^.prevsize := sizeleft;
      { inherit the lastblockflag }
      pcurr_tmp^.size := sizeleft or (size_flags and lastblockflag);
      pcurr_tmp^.prevsize := size;
      pcurr_tmp^.freelists := pcurr^.freelists;
      { the block we return is not the last one anymore (there's now a block after it) }
      { decrease size of block to new size }
      pcurr^.size := size or (size_flags and (not sizemask and not lastblockflag));
      { insert the block in the freelist }
      append_to_list_var(pcurr_tmp);
      result := size;
    end
  else
    result := oldsize;
end;


{*****************************************************************************
                         Try concat freerecords
*****************************************************************************}

procedure concat_two_blocks(mc_left, mc_right: pmemchunk_var);
var
  mc_tmp : pmemchunk_var;
  size_right : ptruint;
begin
  // mc_right can't be a fixed size block
  if mc_right^.size and fixedsizeflag<>0 then
    HandleError(204);
  // left block free, concat with right-block
  size_right := mc_right^.size and sizemask;
  inc(mc_left^.size, size_right);
  // if right-block was last block, copy flag
  if (mc_right^.size and lastblockflag) <> 0 then
    begin
      mc_left^.size := mc_left^.size or lastblockflag;
    end
  else
    begin
      // there is a block to the right of the right-block, adjust it's prevsize
      mc_tmp := pmemchunk_var(pointer(mc_right)+size_right);
      mc_tmp^.prevsize := mc_left^.size and sizemask;
    end;
  // remove right-block from doubly linked list
  remove_from_list_var(mc_right);
end;

function try_concat_free_chunk_forward(mc: pmemchunk_var): boolean;
var
  mc_tmp : pmemchunk_var;
begin
  { try concat forward }
  result := false;
  if (mc^.size and lastblockflag) = 0 then
   begin
     mc_tmp := pmemchunk_var(pointer(mc)+(mc^.size and sizemask));
     if (mc_tmp^.size and usedflag) = 0 then
       begin
         // next block free: concat
         concat_two_blocks(mc, mc_tmp);
         result := true;
       end;
   end;
end;

function try_concat_free_chunk(mc: pmemchunk_var): pmemchunk_var;
var
  mc_tmp : pmemchunk_var;
begin
  try_concat_free_chunk_forward(mc);

  { try concat backward }
  if (mc^.size and firstblockflag) = 0 then
    begin
      mc_tmp := pmemchunk_var(pointer(mc)-mc^.prevsize);
      if (mc_tmp^.size and usedflag) = 0 then
        begin
          // prior block free: concat
          concat_two_blocks(mc_tmp, mc);
          mc := mc_tmp;
        end;
    end;

  result := mc;
end;


{*****************************************************************************
                                Grow Heap
*****************************************************************************}

function find_free_oschunk(loc_freelists: pfreelists; 
  minsize, maxsize: ptruint; var size: ptruint): poschunk;
var
  prev_poc, poc: poschunk;
  pocsize: ptruint;
begin
  poc := loc_freelists^.oslist;
  prev_poc := nil;
  while poc <> nil do
    begin
      if (poc^.size and ocrecycleflag) <> 0 then
      begin
        { oops! we recycled this chunk; remove it from list }
        poc^.size := poc^.size and not ocrecycleflag;
        poc := poc^.next_free;
        if prev_poc = nil then
          loc_freelists^.oslist := poc
        else
          prev_poc^.next_free := poc;
        continue;
      end;
      pocsize := poc^.size and sizemask;
      if (pocsize >= minsize) and
         (pocsize <= maxsize) then
        begin
          size := pocsize;
          if prev_poc = nil then
            loc_freelists^.oslist := poc^.next_free
          else
            prev_poc^.next_free := poc^.next_free;
          dec(loc_freelists^.oscount);
          remove_freed_fixed_chunks(poc);
          break;
        end;
      prev_poc := poc;
      poc := poc^.next_free;
    end;
  result := poc;
end;

function alloc_oschunk(loc_freelists: pfreelists; chunkindex, size: ptruint): pointer;
var
  pmc,
  pmc_next  : pmemchunk_fixed;
  pmcv      : pmemchunk_var;
  poc       : poschunk;
  prev_poc  : poschunk;
  minsize,
  maxsize,
<<<<<<< HEAD
  i         : ptruint;
  chunksize : ptruint;
  status    : pfpcheapstatus;
=======
  i         : ptrint;
  chunksize : ptrint;
  pocsize   : ptrint;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  { increase size by size needed for os block header }
  minsize := size + varfirstoffset;
  { for fixed size chunks we keep offset from os chunk to mem chunk in
    upper bits, so maximum os chunk size is 64K on 32bit for fixed size }
  if chunkindex<>0 then
    maxsize := 1 shl (32-fixedoffsetshift)
  else
    maxsize := high(ptruint);
  poc:=nil;
  { blocks available in freelist? }
<<<<<<< HEAD
  { do not reformat fixed size chunks too quickly }
  if loc_freelists^.oscount >= MaxKeptOSChunks then
    poc := find_free_oschunk(loc_freelists, minsize, maxsize, size);
  { if none available, try to recycle orphaned os chunks }
  if not assigned(poc) and (assigned(orphaned_freelists.waitfixed)
      or assigned(orphaned_freelists.waitvar) or (orphaned_freelists.oscount > 0)) then
    begin
{$ifdef FPC_HAS_FEATURE_THREADING}
      entercriticalsection(heap_lock);
{$endif}
      finish_waitfixedlist(@orphaned_freelists);
      finish_waitvarlist(@orphaned_freelists);
      if orphaned_freelists.oscount > 0 then
        begin
          { blocks available in orphaned freelist ? }
          poc := find_free_oschunk(@orphaned_freelists, minsize, maxsize, size);
          if assigned(poc) then
            begin
              { adopt this os chunk }
              poc^.freelists := loc_freelists;
              if assigned(poc^.prev_any) then
                poc^.prev_any^.next_any := poc^.next_any
              else
                orphaned_freelists.oslist_all := poc^.next_any;
              if assigned(poc^.next_any) then
                poc^.next_any^.prev_any := poc^.prev_any;
              poc^.next_any := loc_freelists^.oslist_all;
              if assigned(loc_freelists^.oslist_all) then
                loc_freelists^.oslist_all^.prev_any := poc;
              poc^.prev_any := nil;
              loc_freelists^.oslist_all := poc;
            end;
        end;
{$ifdef FPC_HAS_FEATURE_THREADING}
      leavecriticalsection(heap_lock);
{$endif}
=======
  poc := freeoslist;
  prev_poc := nil;
  while poc <> nil do
    begin
      if (poc^.size and ocrecycleflag) <> 0 then
      begin
        { oops! we recycled this chunk; remove it from list }
        poc^.size := poc^.size and not ocrecycleflag;
        poc := poc^.next;
        if prev_poc = nil then
          freeoslist := poc
        else
          prev_poc^.next := poc;
        if poc = nil then
          freeoslistend := nil;
        continue;
      end;
      pocsize := poc^.size and sizemask;
      if (pocsize >= minsize) and
         (pocsize <= maxsize) then
        begin
          size := pocsize;
          if prev_poc = nil then
            freeoslist := poc^.next
          else
            prev_poc^.next := poc^.next;
          if poc^.next = nil then
            freeoslistend := nil;
          dec(freeoslistcount);
          pmc := pmemchunk_fixed(pointer(poc)+fixedfirstoffset);
          if pmc^.size <> 0 then
            remove_all_from_list_fixed(pmc^.size and fixedsizemask, poc);
          break;
        end;
      prev_poc := poc;
      poc := poc^.next;
>>>>>>> graemeg/fixes_2_2
    end;
  if poc = nil then
    begin
{$ifdef DUMPGROW}
      writeln('growheap(',size,')  allocating ',(size+sizeof(toschunk)+$ffff) and not $ffff);
      DumpBlocks(loc_freelists);
{$endif}
      { allocate by 64K size }
      size := (size+varfirstoffset+$ffff) and not $ffff;
      { allocate smaller blocks for fixed-size chunks }
      if chunkindex<>0 then
        begin
          poc := SysOSAlloc(loc_freelists^.LocGrowHeapSizeSmall);
          if poc<>nil then
            size := loc_freelists^.LocGrowHeapSizeSmall;
        end
    { first try 256K (default) }
      else if size<=GrowHeapSize1 then
        begin
          poc := SysOSAlloc(GrowHeapSize1);
          if poc<>nil then
            size := GrowHeapSize1;
        end
    { second try 1024K (default) }
      else if size<=GrowHeapSize2 then
        begin
          poc := SysOSAlloc(GrowHeapSize2);
          if poc<>nil then
            size := GrowHeapSize2;
        end
    { else allocate the needed bytes }
      else
        poc := SysOSAlloc(size);
    { try again }
      if poc=nil then
      begin
        poc := SysOSAlloc(size);
        if poc=nil then
          begin
            if ReturnNilIfGrowHeapFails then
              begin
                result := nil;
                exit
              end
            else
              HandleError(203);
          end;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
      poc^.freelists := loc_freelists;
      poc^.prev_any := nil;
      poc^.next_any := loc_freelists^.oslist_all;
      if assigned(loc_freelists^.oslist_all) then
        loc_freelists^.oslist_all^.prev_any := poc;
      loc_freelists^.oslist_all := poc;
=======
      { prevent thinking this os chunk is on some freelist }
      clear_oschunk_on_freelist_fixed_flag(poc);
      poc^.next := nil;
>>>>>>> graemeg/fixes_2_2
=======
      { prevent thinking this os chunk is on some freelist }
      clear_oschunk_on_freelist_fixed_flag(poc);
      poc^.next := nil;
>>>>>>> origin/fixes_2_2
      { set the total new heap size }
      status := @loc_freelists^.internal_status;
      inc(status^.currheapsize, size);
      if status^.currheapsize > status^.maxheapsize then
        status^.maxheapsize := status^.currheapsize;
    end;
  { initialize os-block }
  poc^.size := size;
  if chunkindex<>0 then
    begin
      poc^.used := 0;
      { chop os chunk in fixedsize parts,
        maximum of $ffff elements are allowed, otherwise
        there will be an overflow }
      chunksize := chunkindex shl blockshift;
      if ptruint(size-chunksize)>maxsize then
        HandleError(204);
      { we need to align the user pointers to 8 byte at least for
        mmx/sse and doubles on sparc, align to 16 bytes }
      i := fixedfirstoffset;
      result := pointer(poc) + i;
      pmc := pmemchunk_fixed(result);
      pmc^.prev_fixed := nil;
      repeat
        pmc^.size := fixedsizeflag or chunksize or (i shl fixedoffsetshift);
        inc(i, chunksize);
<<<<<<< HEAD
<<<<<<< HEAD
        if i > ptruint(size - chunksize) then break;
=======
        if i > size - chunksize then break;
>>>>>>> graemeg/fixes_2_2
=======
        if i > size - chunksize then break;
>>>>>>> origin/fixes_2_2
        pmc_next := pmemchunk_fixed(pointer(pmc)+chunksize);
        pmc^.next_fixed := pmc_next;
        pmc_next^.prev_fixed := pmc;
        pmc := pmc_next;
      until false;
      pmc_next := loc_freelists^.fixedlists[chunkindex];
      pmc^.next_fixed := pmc_next;
      if pmc_next<>nil then
        pmc_next^.prev_fixed := pmc;
      loc_freelists^.fixedlists[chunkindex] := pmemchunk_fixed(result);
      { check whether we should increase the size of the fixed freelist blocks }
      inc(loc_freelists^.fixedallocated);
      if loc_freelists^.fixedallocated > fixedallocthreshold then
        begin
          if loc_freelists^.locgrowheapsizesmall < maxgrowheapsizesmall then
            inc(loc_freelists^.locgrowheapsizesmall, loc_freelists^.locgrowheapsizesmall);
          { also set to zero in case we did not grow the blocksize to
            prevent oveflows of this counter in case the rtl is compiled
            range/overflow checking }
          loc_freelists^.fixedallocated := 0;
        end;
    end
  else
    begin
      poc^.used := -1;
      { we need to align the user pointers to 8 byte at least for
        mmx/sse and doubles on sparc, align to 16 bytes }
      result := pointer(poc)+varfirstoffset;
      pmcv := pmemchunk_var(result);
      pmcv^.size := (ptruint(size-varfirstoffset) and sizemask) or (firstblockflag or lastblockflag);
      pmcv^.prevsize := 0;
      pmcv^.freelists := loc_freelists;
      append_to_list_var(pmcv);
    end;
end;

{*****************************************************************************
                                 SysGetMem
*****************************************************************************}

function SysGetMem_Fixed(chunksize: ptruint): pointer;
var
  pmc, pmc_next: pmemchunk_fixed;
  poc: poschunk;
  chunkindex: ptruint;
  loc_freelists: pfreelists;
begin
  { try to find a block in one of the freelists per size }
  chunkindex := chunksize shr blockshift;
<<<<<<< HEAD
  loc_freelists := @freelists;
  pmc := loc_freelists^.fixedlists[chunkindex];
  { no free blocks ? }
  if assigned(pmc) then
=======
  pmc := freelists_fixed[chunkindex];
  { no free blocks ? }
  if assigned(pmc) then
<<<<<<< HEAD
=======
    begin
      { remove oschunk from free list in case we recycle it }
      poc := poschunk(pointer(pmc) - (pmc^.size shr fixedoffsetshift));
      if poc^.used = 0 then
        begin
          poc^.size := poc^.size or ocrecycleflag;
          dec(freeoslistcount);
        end;
    end
  else
>>>>>>> origin/fixes_2_2
    begin
      { remove oschunk from free list in case we recycle it }
      poc := poschunk(pointer(pmc) - (pmc^.size shr fixedoffsetshift));
      if poc^.used = 0 then
        begin
          poc^.size := poc^.size or ocrecycleflag;
          dec(freeoslistcount);
        end;
    end
  else
>>>>>>> graemeg/fixes_2_2
    begin
      { remove oschunk from free list in case we recycle it }
      poc := poschunk(pointer(pmc) - (pmc^.size shr fixedoffsetshift));
      if poc^.used = 0 then
        begin
          poc^.size := poc^.size or ocrecycleflag;
          dec(loc_freelists^.oscount);
        end;
    end
  else if try_finish_waitfixedlist(loc_freelists) then
      { freed some to-be freed chunks, retry allocation }
    exit(SysGetMem_Fixed(chunksize))
  else
    begin
      pmc := alloc_oschunk(loc_freelists, chunkindex, chunksize);
      if not assigned(pmc) then
        exit(nil);
      poc := poschunk(pointer(pmc)-fixedfirstoffset);
    end;
  prefetch(poc^.used);
  { get a pointer to the block we should return }
  result := pointer(pmc)+sizeof(tmemchunk_fixed_hdr);
  { update freelist }
  pmc_next := pmc^.next_fixed;
  loc_freelists^.fixedlists[chunkindex] := pmc_next;
  prefetch((pointer(@chunksize)-4)^);
  if assigned(pmc_next) then
    pmc_next^.prev_fixed := nil;
<<<<<<< HEAD
<<<<<<< HEAD
  { statistics }
  with loc_freelists^.internal_status do
  begin
    inc(currheapused, chunksize);
    if currheapused > maxheapused then
    begin
      maxheapused := currheapused;
{$ifdef DUMP_MEM_USAGE}        
      maxsizeusage := sizeusage;
{$endif}        
    end;
  end;
  inc(poc^.used);
=======
=======
>>>>>>> origin/fixes_2_2
  inc(poc^.used);
  { statistics }
  inc(internal_status.currheapused,chunksize);
  if internal_status.currheapused>internal_status.maxheapused then
  begin
    internal_status.maxheapused:=internal_status.currheapused;
{$ifdef DUMP_MEM_USAGE}        
    maxsizeusage := sizeusage;
{$endif}        
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

function SysGetMem_Var(size: ptruint): pointer;
var
  pcurr : pmemchunk_var;
  pbest : pmemchunk_var;
<<<<<<< HEAD
<<<<<<< HEAD
  loc_freelists : pfreelists;
  iter : cardinal;
begin
  result:=nil;
  { check for maximum possible allocation (everything is rounded up to the
    next multiple of 64k) }
  if (size>high(ptruint)-$ffff) then
    if ReturnNilIfGrowHeapFails then
      exit
    else
      HandleError(204);
  { free pending items }
  loc_freelists := @freelists;
  try_finish_waitvarlist(loc_freelists);
  pbest := nil;
  pcurr := loc_freelists^.varlist;
  iter := high(iter);
  while assigned(pcurr) and (iter>0) do
  begin
    if (pcurr^.size>=size) then
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  iter : longint;
begin
  result:=nil;
  pbest := nil;
  pcurr := freelist_var;
  iter := high(longint);
  while assigned(pcurr) and (iter>0) do
  begin
    if (pcurr^.size>size) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
    begin
      if not assigned(pbest) or (pcurr^.size<pbest^.size) then
      begin
        pbest := pcurr;
        if pcurr^.size = size then
          break;
<<<<<<< HEAD
<<<<<<< HEAD
        iter := matcheffort;
      end;
=======
      end;
      iter := matcheffort;
>>>>>>> graemeg/fixes_2_2
=======
      end;
      iter := matcheffort;
>>>>>>> origin/fixes_2_2
    end;
    pcurr := pcurr^.next_var;
    dec(iter);
  end;
  pcurr := pbest;

  if not assigned(pcurr) then
   begin
    // all os-chunks full, allocate a new one
    pcurr := alloc_oschunk(loc_freelists, 0, size);
    if not assigned(pcurr) then
      exit;
   end;

  { get pointer of the block we should return }
  result := pointer(pcurr)+sizeof(tmemchunk_var_hdr);
  { remove the current block from the freelist }
  remove_from_list_var(pcurr);
  { create the left over freelist block, if at least 16 bytes are free }
  size := split_block(pcurr, size);
  { flag block as used }
  pcurr^.size := pcurr^.size or usedflag;
  { statistics }
<<<<<<< HEAD
  with loc_freelists^.internal_status do
  begin
    inc(currheapused, size);
    if currheapused > maxheapused then
    begin
      maxheapused := currheapused;
{$ifdef DUMP_MEM_USAGE}        
      maxsizeusage := sizeusage;
{$endif}        
    end;
=======
  inc(internal_status.currheapused,size);
  if internal_status.currheapused>internal_status.maxheapused then
  begin
    internal_status.maxheapused:=internal_status.currheapused;
{$ifdef DUMP_MEM_USAGE}        
    maxsizeusage := sizeusage;
{$endif}        
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end;

function SysGetMem(size : ptruint):pointer;
begin
{ Something to allocate ? }
  if size=0 then
    { we always need to allocate something, using heapend is not possible,
      because heappend can be changed by growheap (PFV) }
    size := 1;
{ calc to multiple of 16 after adding the needed bytes for memchunk header }
  if size <= (maxblocksize - sizeof(tmemchunk_fixed_hdr)) then
    begin
      size := (size+(sizeof(tmemchunk_fixed_hdr)+(blocksize-1))) and fixedsizemask;
      result := sysgetmem_fixed(size);
    end
  else
    begin
      if size < high(ptruint)-((sizeof(tmemchunk_var_hdr)+(blocksize-1))) then
        size := (size+(sizeof(tmemchunk_var_hdr)+(blocksize-1))) and sizemask;
      result := sysgetmem_var(size);
    end;

{$ifdef DUMP_MEM_USAGE}
  size := sysmemsize(result);
  if size > sizeusagesize then
    inc(sizeusage[sizeusageindex])
  else
    inc(sizeusage[size shr sizeusageshift]);
{$endif}
end;


{*****************************************************************************
                               SysFreeMem
*****************************************************************************}

procedure waitfree_fixed(pmc: pmemchunk_fixed; poc: poschunk);
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  entercriticalsection(heap_lock);
{$endif}
  pmc^.next_fixed := poc^.freelists^.waitfixed;
  poc^.freelists^.waitfixed := pmc;
{$ifdef FPC_HAS_FEATURE_THREADING}
  leavecriticalsection(heap_lock);
{$endif}
end;

procedure waitfree_var(pmcv: pmemchunk_var);
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  entercriticalsection(heap_lock);
{$endif}
  pmcv^.next_var := pmcv^.freelists^.waitvar;
  pmcv^.freelists^.waitvar := pmcv;
{$ifdef FPC_HAS_FEATURE_THREADING}
  leavecriticalsection(heap_lock);
{$endif}
end;

function SysFreeMem_Fixed(loc_freelists: pfreelists; pmc: pmemchunk_fixed): ptruint;
var
  chunkindex,
  chunksize: ptruint;
  poc: poschunk;
  pmc_next: pmemchunk_fixed;
  pocfreelists: pfreelists;
begin
  poc := poschunk(pointer(pmc)-(pmc^.size shr fixedoffsetshift));
  { start memory access to poc^.freelists already }
  pocfreelists := poc^.freelists;
  chunksize := pmc^.size and fixedsizemask;
  if loc_freelists = pocfreelists then
    begin
<<<<<<< HEAD
      { decrease used blocks count (well in advance of poc^.used check below,
        to avoid stalling due to a dependency) }
      dec(poc^.used);

      { insert the block in its freelist }
      chunkindex := chunksize shr blockshift;
      pmc_next := loc_freelists^.fixedlists[chunkindex];
      pmc^.prev_fixed := nil;
      pmc^.next_fixed := pmc_next;
      if assigned(pmc_next) then
        pmc_next^.prev_fixed := pmc;
      loc_freelists^.fixedlists[chunkindex] := pmc;

      dec(loc_freelists^.internal_status.currheapused, chunksize);

      if poc^.used <= 0 then
        begin
          { decrease used blocks count }
          if poc^.used<0 then
            HandleError(204);
          { osblock can be freed? }
          append_to_oslist(poc);
        end;
    end
  else
    begin
      { deallocated in wrong thread! add to to-be-freed list of correct thread }
      waitfree_fixed(pmc, poc);
=======
      { decrease used blocks count }
      if poc^.used=-1 then
        HandleError(204);
      { osblock can be freed? }
      append_to_oslist(poc, chunksize);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
  result := chunksize;
end;

function SysFreeMem_Var(loc_freelists: pfreelists; pmcv: pmemchunk_var): ptruint;
var
  chunksize: ptruint;
begin
  chunksize := pmcv^.size and sizemask;
  if loc_freelists <> pmcv^.freelists then
  begin
    { deallocated in wrong thread! add to to-be-freed list of correct thread }
    waitfree_var(pmcv);
    exit(chunksize);
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { insert the block in its freelist }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { insert the block in it's freelist }
>>>>>>> graemeg/cpstrnew
  pmcv^.size := pmcv^.size and (not usedflag);
  append_to_list_var(pmcv);
  pmcv := try_concat_free_chunk(pmcv);
  if (pmcv^.size and (firstblockflag or lastblockflag)) = (firstblockflag or lastblockflag) then
    append_to_oslist_var(pmcv);
  dec(loc_freelists^.internal_status.currheapused, chunksize);
  result := chunksize;
end;


function SysFreeMem(p: pointer): ptruint;
var
  pmc: pmemchunk_fixed;
<<<<<<< HEAD
<<<<<<< HEAD
  loc_freelists: pfreelists;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef DUMP_MEM_USAGE}
  size: sizeint;
{$endif}
begin
  pmc := pmemchunk_fixed(p-sizeof(tmemchunk_fixed_hdr));
  prefetch(pmc^.size);
  if p=nil then
    begin
      result:=0;
      exit;
    end;
{$ifdef DUMP_MEM_USAGE}
  size := sysmemsize(p);
  if size > sizeusagesize then
    dec(sizeusage[sizeusageindex])
  else
    dec(sizeusage[size shr sizeusageshift]);
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
  { loc_freelists is a threadvar, so it can be worth it to prefetch }
  loc_freelists := @freelists;
  prefetch(loc_freelists^.internal_status.currheapused);
=======
=======
>>>>>>> origin/fixes_2_2
  pmc := pmemchunk_fixed(p-sizeof(tmemchunk_fixed_hdr));
>>>>>>> graemeg/fixes_2_2
  { check if this is a fixed- or var-sized chunk }
  if (pmc^.size and fixedsizeflag) = 0 then
    result := sysfreemem_var(loc_freelists, pmemchunk_var(p-sizeof(tmemchunk_var_hdr)))
  else
    result := sysfreemem_fixed(loc_freelists, pmc);
end;

procedure finish_waitfixedlist(loc_freelists: pfreelists);
  { free to-be-freed chunks, return whether we freed anything }
var
  pmc: pmemchunk_fixed;
begin
  while loc_freelists^.waitfixed <> nil do
  begin
    { keep next_fixed, might be destroyed }
    pmc := loc_freelists^.waitfixed;
    loc_freelists^.waitfixed := pmc^.next_fixed;
    SysFreeMem_Fixed(loc_freelists, pmc);
  end;
end;

function try_finish_waitfixedlist(loc_freelists: pfreelists): boolean;
begin
  if loc_freelists^.waitfixed = nil then 
    exit(false);
{$ifdef FPC_HAS_FEATURE_THREADING}
  entercriticalsection(heap_lock);
{$endif}
  finish_waitfixedlist(loc_freelists);
{$ifdef FPC_HAS_FEATURE_THREADING}
  leavecriticalsection(heap_lock);
{$endif}
  result := true;
end;

procedure finish_waitvarlist(loc_freelists: pfreelists);
  { free to-be-freed chunks, return whether we freed anything }
var
  pmcv: pmemchunk_var;
begin
  while loc_freelists^.waitvar <> nil do
  begin
    { keep next_var, might be destroyed }
    pmcv := loc_freelists^.waitvar;
    loc_freelists^.waitvar := pmcv^.next_var;
    SysFreeMem_Var(loc_freelists, pmcv);
  end;
end;

procedure try_finish_waitvarlist(loc_freelists: pfreelists);
begin
  if loc_freelists^.waitvar = nil then 
    exit;
{$ifdef FPC_HAS_FEATURE_THREADING}
  entercriticalsection(heap_lock);
{$endif}
  finish_waitvarlist(loc_freelists);
{$ifdef FPC_HAS_FEATURE_THREADING}
  leavecriticalsection(heap_lock);
{$endif}
end;

{*****************************************************************************
                              SysFreeMemSize
*****************************************************************************}

Function SysFreeMemSize(p: pointer; size: ptruint):ptruint;
begin
  if size=0 then
    exit(0);
  { can't free partial blocks, ignore size }
  result := SysFreeMem(p);
end;


{*****************************************************************************
                                 SysMemSize
*****************************************************************************}

function SysMemSize(p: pointer): ptruint;
begin
  result := pmemchunk_fixed(pointer(p)-sizeof(tmemchunk_fixed_hdr))^.size;
  if (result and fixedsizeflag) = 0 then
    begin
      result := result and sizemask;
      dec(result, sizeof(tmemchunk_var_hdr));
    end
  else
    begin
      result := result and fixedsizemask;
      dec(result, sizeof(tmemchunk_fixed_hdr));
    end;
end;


{*****************************************************************************
                                 SysAllocMem
*****************************************************************************}

function SysAllocMem(size: ptruint): pointer;
begin
  result := MemoryManager.GetMem(size);
  if result<>nil then
    FillChar(result^,MemoryManager.MemSize(result),0);
end;


{*****************************************************************************
                                 SysResizeMem
*****************************************************************************}

function SysTryResizeMem(var p: pointer; size: ptruint): boolean;
var
  chunksize,
  oldsize,
  currsize : ptruint;
  pcurr : pmemchunk_var;
  loc_freelists : pfreelists;
begin
  SysTryResizeMem := false;

  { fix p to point to the heaprecord }
  chunksize := pmemchunk_fixed(p-sizeof(tmemchunk_fixed_hdr))^.size;

  { handle fixed memchuncks separate. Only allow resizes when the
    new size fits in the same block }
  if (chunksize and fixedsizeflag) <> 0 then
    begin
      currsize := chunksize and fixedsizemask;

      { 1. Resizing to smaller sizes will never allocate a new block. We just keep the current block. This
           is needed for the expectations that resizing to a small block will not move the contents of
           a memory block
        2. For resizing to greater size first check if the size fits in the fixed block range to prevent
           "truncating" the size by the fixedsizemask }
      if ((size <= (maxblocksize - sizeof(tmemchunk_fixed_hdr))) and
          ((size+(sizeof(tmemchunk_fixed_hdr)+(blocksize-1))) and sizemask <= currsize)) then
        begin
          systryresizemem:=true;
          exit;
        end;

      { we need to allocate a new fixed or var memchunck }
      exit;
    end;

  { var memchunk }

  { do not fragment the heap with small shrinked blocks }
  {  also solves problem with var sized chunks smaller than sizeof(tmemchunk_var) }
  if size < maxblocksize div 2 then
    exit(false);

  currsize := chunksize and sizemask;
  size := (size+sizeof(tmemchunk_var_hdr)+(blocksize-1)) and sizemask;

  { is the allocated block still correct? }
  if (currsize>=size) and (size>ptruint(currsize-blocksize)) then
    begin
      SysTryResizeMem := true;
      exit;
    end;

  { get pointer to block }
  loc_freelists := @freelists;
  pcurr := pmemchunk_var(pointer(p)-sizeof(tmemchunk_var_hdr));
  if pcurr^.freelists <> loc_freelists then
    exit;
  oldsize := currsize;

  { do we need to allocate more memory ? }
  if try_concat_free_chunk_forward(pcurr) then
    currsize := pcurr^.size and sizemask;
  if size>currsize then
    begin
      { adjust statistics (try_concat_free_chunk_forward may have merged a free
        block into the current block, which we will subsequently free (so the
        combined size will be freed -> make sure the combined size is marked as
        used) }
      with loc_freelists^.internal_status do
      begin
        inc(currheapused, currsize-oldsize);
        if currheapused > maxheapused then
          maxheapused := currheapused;
      end;
      { the size is bigger than the previous size, we need to allocate more mem
        but we could not concatenate with next block or not big enough }
      exit;
    end
  else
  { is the size smaller then we can adjust the block to that size and insert
    the other part into the freelist }
  if currsize>size then
    currsize := split_block(pcurr, size);

  with loc_freelists^.internal_status do
  begin
    inc(currheapused, currsize-oldsize);
    if currheapused > maxheapused then
      maxheapused := currheapused;
  end;
  SysTryResizeMem := true;
end;


{*****************************************************************************
                                 SysResizeMem
*****************************************************************************}

function SysReAllocMem(var p: pointer; size: ptruint):pointer;
var
  newsize,
  oldsize,
  minsize : ptruint;
  p2 : pointer;
begin
  { Free block? }
  if size=0 then
   begin
     if p<>nil then
      begin
        MemoryManager.FreeMem(p);
        p := nil;
      end;
   end
  else
   { Allocate a new block? }
   if p=nil then
    begin
      p := MemoryManager.GetMem(size);
    end
  else
   begin
    { Resize block }
{$ifdef DUMP_MEM_USAGE}
    oldsize:=SysMemSize(p);
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
{$endif}    
>>>>>>> graemeg/fixes_2_2
=======
{$endif}    
>>>>>>> origin/fixes_2_2
    if not SysTryResizeMem(p,size) then
    begin
      oldsize:=MemoryManager.MemSize(p);
      { Grow with bigger steps to prevent the need for
        multiple getmem/freemem calls for fixed blocks. It might cost a bit
        of extra memory, but in most cases a reallocmem is done multiple times. }
      if oldsize<maxblocksize then
        begin
          newsize:=oldsize*2+blocksize;
          if size>newsize then
            newsize:=size;
        end
      else
        newsize:=size;
      { calc size of data to move }
      minsize:=oldsize;
      if newsize < minsize then
        minsize := newsize;
      p2 := MemoryManager.GetMem(newsize);
      if p2<>nil then
        Move(p^,p2^,minsize);
      MemoryManager.FreeMem(p);
      p := p2;
{$ifdef DUMP_MEM_USAGE}
    end else begin
      size := sysmemsize(p);
      if size <> oldsize then
      begin
        if oldsize > sizeusagesize then
          dec(sizeusage[sizeusageindex])
        else if oldsize >= 0 then
          dec(sizeusage[oldsize shr sizeusageshift]);
        if size > sizeusagesize then
          inc(sizeusage[sizeusageindex])
        else if size >= 0 then
          inc(sizeusage[size shr sizeusageshift]);
      end;
{$endif}
    end;
   end;
  SysReAllocMem := p;
end;

{$endif HAS_MEMORYMANAGER}

{$ifndef HAS_MEMORYMANAGER}

{*****************************************************************************
                                 InitHeap
*****************************************************************************}

{ This function will initialize the Heap manager and need to be called from
  the initialization of the system unit }
{$ifdef FPC_HAS_FEATURE_THREADING}
procedure InitHeapThread;
var
  loc_freelists: pfreelists;
begin
  if heap_lock_use > 0 then
  begin
    entercriticalsection(heap_lock);
    inc(heap_lock_use);
    leavecriticalsection(heap_lock);
  end;
  loc_freelists := @freelists;
  fillchar(loc_freelists^,sizeof(tfreelists),0);
  { initialise the local blocksize for allocating oschunks for fixed
    freelists with the default starting value }
  loc_freelists^.locgrowheapsizesmall:=growheapsizesmall;
{$ifdef DUMP_MEM_USAGE}
  fillchar(sizeusage,sizeof(sizeusage),0);
  fillchar(maxsizeusage,sizeof(sizeusage),0);
{$endif}
end;
{$endif}

procedure InitHeap; public name '_FPC_InitHeap';
var
  loc_freelists: pfreelists;
begin
{$ifdef FPC_HAS_FEATURE_THREADING}
  { we cannot initialize the locks here yet, thread support is
    not loaded yet }
  heap_lock_use := 0;
{$endif}
  loc_freelists := @freelists;
  fillchar(loc_freelists^,sizeof(tfreelists),0);
  { initialise the local blocksize for allocating oschunks for fixed
    freelists with the default starting value }
  loc_freelists^.locgrowheapsizesmall:=growheapsizesmall;
  fillchar(orphaned_freelists,sizeof(orphaned_freelists),0);
end;

<<<<<<< HEAD
procedure RelocateHeap;
var
  loc_freelists: pfreelists;
begin
  { this function should be called in main thread context }
{$ifdef FPC_HAS_FEATURE_THREADING}
  if heap_lock_use > 0 then
    exit;
  heap_lock_use := 1;
  initcriticalsection(heap_lock);
{$endif}
  loc_freelists := @freelists;
  { loc_freelists still points to main thread's freelists, but they
    have a reference to the global main freelists, fix them to point
    to the main thread specific variable }
  modify_freelists(loc_freelists, loc_freelists);
  if MemoryManager.RelocateHeap <> nil then
    MemoryManager.RelocateHeap();
=======
{$ifndef HAS_MEMORYMANAGER}

{*****************************************************************************
                                 InitHeap
*****************************************************************************}

{ This function will initialize the Heap manager and need to be called from
  the initialization of the system unit }
procedure InitHeap;
begin
  FillChar(freelists_fixed,sizeof(tfreelists),0);
  freelist_var := nil;
  freeoslist := nil;
  freeoslistcount := 0;
  fillchar(internal_status,sizeof(internal_status),0);
{$ifdef DUMP_MEM_USAGE}
  fillchar(sizeusage,sizeof(sizeusage),0);
  fillchar(maxsizeusage,sizeof(sizeusage),0);
{$endif}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure FinalizeHeap;
var
<<<<<<< HEAD
  poc, poc_next: poschunk;
  loc_freelists: pfreelists;
{$ifdef FPC_HAS_FEATURE_THREADING}
  last_thread: boolean;
{$endif}
{$ifdef DUMP_MEM_USAGE}
=======
  poc : poschunk;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  i : longint;
{$endif}
begin
<<<<<<< HEAD
<<<<<<< HEAD
  { Do not try to do anything if the heap manager already reported an error }
  if (errorcode=203) or (errorcode=204) then
    exit;
  loc_freelists := @freelists;
{$ifdef FPC_HAS_FEATURE_THREADING}
  if heap_lock_use > 0 then
  begin
    entercriticalsection(heap_lock);
    finish_waitfixedlist(loc_freelists);
    finish_waitvarlist(loc_freelists);
  end;
{$endif}
{$ifdef HAS_SYSOSFREE}
  poc := loc_freelists^.oslist;
  while assigned(poc) do
  begin
    poc_next := poc^.next_free;
    { check if this os chunk was 'recycled' i.e. taken in use again }
    if (poc^.size and ocrecycleflag) = 0 then
      free_oschunk(loc_freelists, poc)
    else
      poc^.size := poc^.size and not ocrecycleflag;
    poc := poc_next;
  end;
  loc_freelists^.oslist := nil;
  loc_freelists^.oscount := 0;
{$endif HAS_SYSOSFREE}
{$ifdef FPC_HAS_FEATURE_THREADING}
  if heap_lock_use > 0 then
  begin
    poc := modify_freelists(loc_freelists, @orphaned_freelists);
    if assigned(poc) then
    begin
      poc^.next_any := orphaned_freelists.oslist_all;
      if assigned(orphaned_freelists.oslist_all) then
        orphaned_freelists.oslist_all^.prev_any := poc;
      orphaned_freelists.oslist_all := loc_freelists^.oslist_all;
    end;
    dec(heap_lock_use);
    last_thread := heap_lock_use = 0;
    leavecriticalsection(heap_lock);
    if last_thread then
      donecriticalsection(heap_lock);
  end;
{$endif}
{$ifdef SHOW_MEM_USAGE}
  writeln('Max heap used/size: ', loc_freelists^.internal_status.maxheapused, '/', 
    loc_freelists^.internal_status.maxheapsize);
  flush(output);
{$endif}
{$ifdef DUMP_MEM_USAGE}
  for i := 0 to sizeusageindex-1 do
    if maxsizeusage[i] <> 0 then
      writeln('size ', i shl sizeusageshift, ' usage ', maxsizeusage[i]);
  writeln('size >', sizeusagesize, ' usage ', maxsizeusage[sizeusageindex]);
  flush(output);
{$endif}
=======
{$ifdef SHOW_MEM_USAGE}
  writeln('Max heap used/size: ', internal_status.maxheapused, '/', 
    internal_status.maxheapsize);
{$endif}
{$ifdef DUMP_MEM_USAGE}
  for i := 0 to sizeusageindex-1 do
    if maxsizeusage[i] <> 0 then
      writeln('size ', i shl sizeusageshift, ' usage ', maxsizeusage[i]);
  writeln('size >', sizeusagesize, ' usage ', maxsizeusage[sizeusageindex]);
{$endif}
{$ifdef HAS_SYSOSFREE}
=======
{$ifdef SHOW_MEM_USAGE}
  writeln('Max heap used/size: ', internal_status.maxheapused, '/', 
    internal_status.maxheapsize);
{$endif}
{$ifdef DUMP_MEM_USAGE}
  for i := 0 to sizeusageindex-1 do
    if maxsizeusage[i] <> 0 then
      writeln('size ', i shl sizeusageshift, ' usage ', maxsizeusage[i]);
  writeln('size >', sizeusagesize, ' usage ', maxsizeusage[sizeusageindex]);
{$endif}
{$ifdef HAS_SYSOSFREE}
>>>>>>> origin/fixes_2_2
  while assigned(freeoslist) do
    begin
      poc:=freeoslist^.next;
      SysOSFree(freeoslist, freeoslist^.size and sizemask);
      dec(freeoslistcount);
      freeoslist:=poc;
    end;
  freeoslistend:=nil;
{$endif HAS_SYSOSFREE}
  { release mutex }
  MemoryMutexManager.MutexDone;
>>>>>>> graemeg/fixes_2_2
end;

{$endif HAS_MEMORYMANAGER}
{$endif defined(FPC_HAS_FEATURE_HEAP) or defined(FPC_IN_HEAPMGR)}

