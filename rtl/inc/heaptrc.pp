{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    Heap tracer

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$checkpointer off}
unit heaptrc;
interface

{$inline on}

{$ifdef FPC_HEAPTRC_EXTRA}
  {$define EXTRA}
  {$inline off}
{$endif FPC_HEAPTRC_EXTRA}

{$TYPEDADDRESS on}

{$if defined(win32) or defined(wince)}
  {$define windows}
{$endif}


Procedure DumpHeap;
Procedure DumpHeap(SkipIfNoLeaks : Boolean);

{ define EXTRA to add more
  tests :
   - keep all memory after release and
   check by CRC value if not changed after release
   WARNING this needs extremely much memory (PM) }

type
   tFillExtraInfoProc = procedure(p : pointer);
   tdisplayextrainfoProc = procedure (var ptext : text;p : pointer);

{ Allows to add info pre memory block, see ppheap.pas of the compiler
  for example source }
procedure SetHeapExtraInfo(size : ptruint;fillproc : tfillextrainfoproc;displayproc : tdisplayextrainfoproc);

{ Redirection of the output to a file }
procedure SetHeapTraceOutput(const name : string);overload;
procedure SetHeapTraceOutput(var ATextOutput : Text);overload;

procedure CheckPointer(p : pointer);

const
  { tracing level
    splitted in two if memory is released !! }
{$ifdef EXTRA}
  tracesize = 32;
{$else EXTRA}
  tracesize = 16;
{$endif EXTRA}
  { install heaptrc memorymanager }
  useheaptrace : boolean=true;
  { less checking }
  quicktrace : boolean=true;
  { calls halt() on error by default !! }
  HaltOnError : boolean = true;
  { Halt on exit if any memory was not freed }
  HaltOnNotReleased : boolean = false;

  { set this to true if you suspect that memory
    is freed several times }
{$ifdef EXTRA}
  keepreleased : boolean=true;
{$else EXTRA}
  keepreleased : boolean=false;
{$endif EXTRA}
  { add a small footprint at the end of memory blocks, this
    can check for memory overwrites at the end of a block }
  add_tail : boolean = true;
  tail_size : longint = sizeof(ptruint);

  { put crc in sig
    this allows to test for writing into that part }
  usecrc : boolean = true;

  printleakedblock: boolean = false;
  printfaultyblock: boolean = false;
  maxprintedblocklength: integer = 128;

  GlobalSkipIfNoLeaks : Boolean = False;

implementation

const
  { allows to add custom info in heap_mem_info, this is the size that will
    be allocated for this information }
  extra_info_size : ptruint = 0;
  exact_info_size : ptruint = 0;
  EntryMemUsed    : ptruint = 0;
  { function to fill this info up }
  fill_extra_info_proc : TFillExtraInfoProc = nil;
  display_extra_info_proc : TDisplayExtraInfoProc = nil;
  { indicates where the output will be redirected }
  { only set using environment variables          }
  outputstr : shortstring = '';
  ReleaseSig = $AAAAAAAA;
  AllocateSig = $DEADBEEF;
  CheckSig = $12345678;

type
  pheap_extra_info = ^theap_extra_info;
  theap_extra_info = record
    check       : cardinal;  { used to check if the procvar is still valid }
    fillproc    : tfillextrainfoProc;
    displayproc : tdisplayextrainfoProc;
    data : record
           end;
  end;

  ppheap_mem_info = ^pheap_mem_info;
  pheap_mem_info = ^theap_mem_info;

  { warning the size of theap_mem_info
    must be a multiple of 8
    because otherwise you will get
    problems when releasing the usual memory part !!
    sizeof(theap_mem_info = 16+tracesize*4 so
    tracesize must be even !! PM }
  theap_mem_info = record
    previous,
    next     : pheap_mem_info;
    todolist : ppheap_mem_info;
    todonext : pheap_mem_info;
    size     : ptruint;
    sig      : longword;
{$ifdef EXTRA}
    release_sig : longword;
    prev_valid  : pheap_mem_info;
{$endif EXTRA}
    calls    : array [1..tracesize] of codepointer;
    exact_info_size : word;
    extra_info_size : word;
    extra_info      : pheap_extra_info;
  end;

  pheap_info = ^theap_info;
  theap_info = record
{$ifdef EXTRA}
    heap_valid_first,
    heap_valid_last : pheap_mem_info;
{$endif EXTRA}
    heap_mem_root : pheap_mem_info;
    heap_free_todo : pheap_mem_info;
    getmem_cnt,
    freemem_cnt   : ptruint;
    getmem_size,
    freemem_size  : ptruint;
    getmem8_size,
    freemem8_size : ptruint;
    error_in_heap : boolean;
    inside_trace_getmem : boolean;
  end;

var
  useownfile, useowntextoutput : boolean;
  ownfile : text;
{$ifdef EXTRA}
  error_file : text;
{$endif EXTRA}
  main_orig_todolist: ppheap_mem_info;
  main_relo_todolist: ppheap_mem_info;
  orphaned_info: theap_info;
  todo_lock: trtlcriticalsection;
  textoutput : ^text;
{$ifdef FPC_HAS_FEATURE_THREADING}
threadvar
{$else}
var
{$endif}
  heap_info: theap_info;

{*****************************************************************************
                                   Crc 32
*****************************************************************************}

var
  Crc32Tbl : array[0..255] of longword;
const
  Crc32Seed = $ffffffff;
  Crc32Pattern = $edb88320;

procedure MakeCRC32Tbl;
var
  crc : longword;
  i,n : byte;
begin
  for i:=0 to 255 do
   begin
     crc:=i;
     for n:=1 to 8 do
      if odd(crc) then
       crc:=(crc shr 1) xor longword(CRC32Pattern)
      else
       crc:=crc shr 1;
     Crc32Tbl[i]:=crc;
   end;
end;


Function UpdateCrc32(InitCrc:longword;var InBuf;InLen:ptruint):longword;
var
  i : ptruint;
  p : pchar;
begin
  p:=@InBuf;
  for i:=1 to InLen do
   begin
     InitCrc:=Crc32Tbl[byte(InitCrc) xor byte(p^)] xor (InitCrc shr 8);
     inc(p);
   end;
  UpdateCrc32:=InitCrc;
end;

Function calculate_sig(p : pheap_mem_info) : longword;
var
   crc : longword;
   pl : pptruint;
begin
   crc:=longword(CRC32Seed);
   crc:=UpdateCrc32(crc,p^.size,sizeof(ptruint));
   crc:=UpdateCrc32(crc,p^.calls,tracesize*sizeof(codepointer));
   if p^.extra_info_size>0 then
     crc:=UpdateCrc32(crc,p^.extra_info^,p^.exact_info_size);
   if add_tail then
     begin
        { Check also 4 bytes just after allocation !! }
        pl:=pointer(p)+sizeof(theap_mem_info)+p^.size;
        crc:=UpdateCrc32(crc,pl^,tail_size);
     end;
   calculate_sig:=crc;
end;

{$ifdef EXTRA}
Function calculate_release_sig(p : pheap_mem_info) : longword;
var
   crc : longword;
   pl : pptruint;
begin
   crc:=longword(CRC32Seed);
   crc:=UpdateCrc32(crc,p^.size,sizeof(ptruint));
   crc:=UpdateCrc32(crc,p^.calls,tracesize*sizeof(codepointer));
   if p^.extra_info_size>0 then
     crc:=UpdateCrc32(crc,p^.extra_info^,p^.exact_info_size);
   { Check the whole of the whole allocation }
   pl:=pointer(p)+p^.extra_info_size+sizeof(theap_mem_info);
   crc:=UpdateCrc32(crc,pl^,p^.size);
   { Check also 4 bytes just after allocation !! }
   if add_tail then
     begin
        { Check also 4 bytes just after allocation !! }
        pl:=pointer(p)+p^.extra_info_size+sizeof(theap_mem_info)+p^.size;
        crc:=UpdateCrc32(crc,pl^,tail_size);
     end;
   calculate_release_sig:=crc;
end;
{$endif EXTRA}


{*****************************************************************************
                                Helpers
*****************************************************************************}

function InternalFreeMemSize(loc_info: pheap_info; p: pointer; pp: pheap_mem_info;
  size: ptruint; release_todo_lock: boolean): ptruint; forward;
function TraceFreeMem(p: pointer): ptruint; forward;

procedure printhex(p : pointer; const size : PtrUInt; var ptext : text);
var s: PtrUInt;
 i: Integer;
begin
  s := size;
  if s > maxprintedblocklength then
    s := maxprintedblocklength;

  for i:=0 to s-1 do
    write(ptext, hexstr(pbyte(p + i)^,2));

  if size > maxprintedblocklength then
    writeln(ptext,'.. - ')
  else
    writeln(ptext, ' - ');

  for i:=0 to s-1 do
    if pchar(p + sizeof(theap_mem_info) + i)^ < ' ' then
      write(ptext, ' ')
    else
      write(ptext, pchar(p + i)^);

  if size > maxprintedblocklength then
    writeln(ptext,'..')
  else
    writeln(ptext);
end;

procedure call_stack(pp : pheap_mem_info;var ptext : text);
var
  i  : ptruint;
begin
  writeln(ptext,'Call trace for block $',hexstr(pointer(pp)+sizeof(theap_mem_info)),' size ',pp^.size);
  if printleakedblock then
    begin
      write(ptext, 'Block content: ');
      printhex(pointer(pp) + sizeof(theap_mem_info), pp^.size, ptext);
    end;

  for i:=1 to tracesize do
   if pp^.calls[i]<>nil then
     writeln(ptext,BackTraceStrFunc(pp^.calls[i]));

  { the check is done to be sure that the procvar is not overwritten }
  if assigned(pp^.extra_info) and
     (pp^.extra_info^.check=cardinal(CheckSig)) and
     assigned(pp^.extra_info^.displayproc) then
   pp^.extra_info^.displayproc(ptext,@pp^.extra_info^.data);
end;


procedure call_free_stack(pp : pheap_mem_info;var ptext : text);
var
  i  : ptruint;
begin
  writeln(ptext,'Call trace for block at $',hexstr(pointer(pp)+sizeof(theap_mem_info)),' size ',pp^.size);
  for i:=1 to tracesize div 2 do
   if pp^.calls[i]<>nil then
     writeln(ptext,BackTraceStrFunc(pp^.calls[i]));
  writeln(ptext,' was released at ');
  for i:=(tracesize div 2)+1 to tracesize do
   if pp^.calls[i]<>nil then
     writeln(ptext,BackTraceStrFunc(pp^.calls[i]));
  { the check is done to be sure that the procvar is not overwritten }
  if assigned(pp^.extra_info) and
     (pp^.extra_info^.check=cardinal(CheckSig)) and
     assigned(pp^.extra_info^.displayproc) then
   pp^.extra_info^.displayproc(ptext,@pp^.extra_info^.data);
end;


procedure dump_already_free(p : pheap_mem_info;var ptext : text);
begin
  Writeln(ptext,'Marked memory at $',HexStr(pointer(p)+sizeof(theap_mem_info)),' released');
  call_free_stack(p,ptext);
  Writeln(ptext,'freed again at');
  dump_stack(ptext,1);
end;

procedure dump_error(p : pheap_mem_info;var ptext : text);
begin
  Writeln(ptext,'Marked memory at $',HexStr(pointer(p)+sizeof(theap_mem_info)),' invalid');
  Writeln(ptext,'Wrong signature $',hexstr(p^.sig,8),' instead of ',hexstr(calculate_sig(p),8));
  if printfaultyblock then
    begin
      write(ptext, 'Block content: ');
      printhex(pointer(p) + sizeof(theap_mem_info), p^.size, ptext);
    end;
  dump_stack(ptext,1);
end;

function released_modified(p : pheap_mem_info;var ptext : text) : boolean;
 var pl : pdword;
     pb : pbyte;
     i : longint;
begin
  released_modified:=false;
  { Check tail_size bytes just after allocation !! }
  pl:=pointer(p)+sizeof(theap_mem_info)+p^.size;
  pb:=pointer(p)+sizeof(theap_mem_info);
  for i:=0 to p^.size-1 do
    if pb[i]<>$F0 then
      begin
        Writeln(ptext,'offset',i,':$',hexstr(i,2*sizeof(pointer)),'"',hexstr(pb[i],2),'"');
        released_modified:=true;
      end;
  for i:=1 to (tail_size div sizeof(dword)) do
    begin
      if unaligned(pl^) <> AllocateSig then
        begin
          released_modified:=true;
          writeln(ptext,'Tail modified after release at pos ',i*sizeof(ptruint));
          printhex(pointer(p)+p^.extra_info_size+sizeof(theap_mem_info)+p^.size,tail_size,ptext);
          break;
        end;
      inc(pointer(pl),sizeof(dword));
    end;
  if released_modified then
    begin
      dump_already_free(p,ptext);
      if @stderr<>@ptext then
        dump_already_free(p,stderr);
    end;
end;

{$ifdef EXTRA}
procedure dump_change_after(p : pheap_mem_info;var ptext : text);
 var pp : pchar;
     i : ptruint;
begin
  Writeln(ptext,'Marked memory at $',HexStr(pointer(p)+sizeof(theap_mem_info)),' invalid');
  Writeln(ptext,'Wrong release CRC $',hexstr(p^.release_sig,8),' instead of ',hexstr(calculate_release_sig(p),8));
  Writeln(ptext,'This memory was changed after call to freemem !');
  call_free_stack(p,ptext);
  pp:=pointer(p)+sizeof(theap_mem_info);
  for i:=0 to p^.size-1 do
    if byte(pp[i])<>$F0 then
      Writeln(ptext,'offset',i,':$',hexstr(i,2*sizeof(pointer)),'"',pp[i],'"');
end;
{$endif EXTRA}

procedure dump_wrong_size(p : pheap_mem_info;size : ptruint;var ptext : text);
begin
  Writeln(ptext,'Marked memory at $',HexStr(pointer(p)+sizeof(theap_mem_info)),' invalid');
  Writeln(ptext,'Wrong size : ',p^.size,' allocated ',size,' freed');
  dump_stack(ptext,1);
  { the check is done to be sure that the procvar is not overwritten }
  if assigned(p^.extra_info) and
     (p^.extra_info^.check=cardinal(CheckSig)) and
     assigned(p^.extra_info^.displayproc) then
   p^.extra_info^.displayproc(ptext,@p^.extra_info^.data);
  call_stack(p,ptext);
end;

function is_in_getmem_list (loc_info: pheap_info; p : pheap_mem_info) : boolean;
var
  i  : ptruint;
  pp : pheap_mem_info;
begin
  is_in_getmem_list:=false;
  pp:=loc_info^.heap_mem_root;
  i:=0;
  while pp<>nil do
   begin
     if ((pp^.sig<>longword(AllocateSig)) or usecrc) and
        ((pp^.sig<>calculate_sig(pp)) or not usecrc) and
        (pp^.sig <>longword(ReleaseSig)) then
      begin
        if useownfile then
          writeln(ownfile,'error in linked list of heap_mem_info')
        else
          writeln(textoutput^,'error in linked list of heap_mem_info');
        RunError(204);
      end;
     if pp=p then
      is_in_getmem_list:=true;
     pp:=pp^.previous;
     inc(i);
     if i>loc_info^.getmem_cnt-loc_info^.freemem_cnt then
       if useownfile then
         writeln(ownfile,'error in linked list of heap_mem_info')
       else
         writeln(textoutput^,'error in linked list of heap_mem_info');
   end;
end;

procedure finish_heap_free_todo_list(loc_info: pheap_info);
var
  bp: pointer;
  pp: pheap_mem_info;
  list: ppheap_mem_info;
begin
  list := @loc_info^.heap_free_todo;
  repeat
    pp := list^;
    list^ := list^^.todonext;
    bp := pointer(pp)+sizeof(theap_mem_info);
    InternalFreeMemSize(loc_info,bp,pp,pp^.size,false);
  until list^ = nil;
end;

procedure try_finish_heap_free_todo_list(loc_info: pheap_info);
begin
  if loc_info^.heap_free_todo <> nil then
  begin
{$ifdef FPC_HAS_FEATURE_THREADING}
    entercriticalsection(todo_lock);
{$endif}
    finish_heap_free_todo_list(loc_info);
{$ifdef FPC_HAS_FEATURE_THREADING}
    leavecriticalsection(todo_lock);
{$endif}
  end;
end;


{*****************************************************************************
                               TraceGetMem
*****************************************************************************}

Function TraceGetMem(size:ptruint):pointer;
var
  i, allocsize : ptruint;
  pl : pdword;
  p  : pointer;
  pp : pheap_mem_info;
  loc_info: pheap_info;
begin
  loc_info := @heap_info;
  try_finish_heap_free_todo_list(loc_info);
{ Do the real GetMem, but alloc also for the info block }
{$ifdef cpuarm}
  allocsize:=(size + 3) and not 3+sizeof(theap_mem_info)+extra_info_size;
{$else cpuarm}
  allocsize:=size+sizeof(theap_mem_info)+extra_info_size;
{$endif cpuarm}
  if add_tail then
    inc(allocsize,tail_size);
  { if ReturnNilIfGrowHeapFails is true
    SysGetMem can return nil }
  p:=SysGetMem(allocsize);
  if (p=nil) then
    begin
      TraceGetMem:=nil;
      exit;
    end;
  pp:=pheap_mem_info(p);
  inc(p,sizeof(theap_mem_info));
  { Update getmem_size and getmem8_size only after successful call 
    to SysGetMem }
  inc(loc_info^.getmem_size,size);
  inc(loc_info^.getmem8_size,(size+7) and not 7);
{ Create the info block }
  pp^.sig:=longword(AllocateSig);
  pp^.todolist:=@loc_info^.heap_free_todo;
  pp^.todonext:=nil;
  pp^.size:=size;
  pp^.extra_info_size:=extra_info_size;
  pp^.exact_info_size:=exact_info_size;
  fillchar(pp^.calls[1],sizeof(pp^.calls),#0);
  {
    the end of the block contains:
    <tail>   4 bytes
    <extra_info>   X bytes
  }
  if extra_info_size>0 then
   begin
     pp^.extra_info:=pointer(pp)+allocsize-extra_info_size;
     fillchar(pp^.extra_info^,extra_info_size,0);
     pp^.extra_info^.check:=cardinal(CheckSig);
     pp^.extra_info^.fillproc:=fill_extra_info_proc;
     pp^.extra_info^.displayproc:=display_extra_info_proc;
     if assigned(fill_extra_info_proc) then
      begin
        loc_info^.inside_trace_getmem:=true;
        fill_extra_info_proc(@pp^.extra_info^.data);
        loc_info^.inside_trace_getmem:=false;
      end;
   end
  else
   pp^.extra_info:=nil;
  if add_tail then
    begin
      { Calculate position from start because of arm
        specific alignment }
      pl:=pointer(pp)+sizeof(theap_mem_info)+pp^.size;
      for i:=1 to tail_size div sizeof(dword) do
        begin
          unaligned(pl^):=dword(AllocateSig);
          inc(pointer(pl),sizeof(dword));
        end;
    end;
  { clear the memory }
  fillchar(p^,size,#255);
  { retrieve backtrace info }
  CaptureBacktrace(1,tracesize,@pp^.calls[1]);

  { insert in the linked list }
  if loc_info^.heap_mem_root<>nil then
   loc_info^.heap_mem_root^.next:=pp;
  pp^.previous:=loc_info^.heap_mem_root;
  pp^.next:=nil;
{$ifdef EXTRA}
  pp^.prev_valid:=loc_info^.heap_valid_last;
  loc_info^.heap_valid_last:=pp;
  if not assigned(loc_info^.heap_valid_first) then
    loc_info^.heap_valid_first:=pp;
{$endif EXTRA}
  loc_info^.heap_mem_root:=pp;
  { must be changed before fill_extra_info is called
    because checkpointer can be called from within
    fill_extra_info PM }
  inc(loc_info^.getmem_cnt);
  { update the signature }
  if usecrc then
    pp^.sig:=calculate_sig(pp);
  TraceGetmem:=p;
end;


{*****************************************************************************
                                TraceFreeMem
*****************************************************************************}

function CheckFreeMemSize(loc_info: pheap_info; pp: pheap_mem_info;
  size, ppsize: ptruint): boolean; inline;
var
  ptext : ^text;
{$ifdef EXTRA}
  pp2 : pheap_mem_info;
{$endif}
begin
  if useownfile then
    ptext:=@ownfile
  else
    ptext:=textoutput;
  inc(loc_info^.freemem_size,size);
  inc(loc_info^.freemem8_size,(size+7) and not 7);
  if not quicktrace then
    begin
      if not(is_in_getmem_list(loc_info, pp)) then
       RunError(204);
    end;
  if (pp^.sig=longword(ReleaseSig)) then
    begin
       loc_info^.error_in_heap:=true;
       dump_already_free(pp,ptext^);
       if haltonerror then halt(1);
    end
  else if ((pp^.sig<>longword(AllocateSig)) or usecrc) and
        ((pp^.sig<>calculate_sig(pp)) or not usecrc) then
    begin
       loc_info^.error_in_heap:=true;
       dump_error(pp,ptext^);
{$ifdef EXTRA}
       dump_error(pp,error_file);
{$endif EXTRA}
       { don't release anything in this case !! }
       if haltonerror then halt(1);
       exit;
    end
  else if pp^.size<>size then
    begin
       loc_info^.error_in_heap:=true;
       dump_wrong_size(pp,size,ptext^);
{$ifdef EXTRA}
       dump_wrong_size(pp,size,error_file);
{$endif EXTRA}
       if haltonerror then halt(1);
       { don't release anything in this case !! }
       exit;
    end;
  { now it is released !! }
  pp^.sig:=longword(ReleaseSig);
  if not keepreleased then
    begin
       if pp^.next<>nil then
         pp^.next^.previous:=pp^.previous;
       if pp^.previous<>nil then
         pp^.previous^.next:=pp^.next;
       if pp=loc_info^.heap_mem_root then
         loc_info^.heap_mem_root:=loc_info^.heap_mem_root^.previous;
    end
  else
    CaptureBacktrace(1,(tracesize div 2)-1,@pp^.calls[(tracesize div 2)+1]);

  inc(loc_info^.freemem_cnt);
  { clear the memory, $F0 will lead to GFP if used as pointer ! }
  fillchar((pointer(pp)+sizeof(theap_mem_info))^,size,#240);
  { this way we keep all info about all released memory !! }
  if keepreleased then
    begin
{$ifdef EXTRA}
       { We want to check if the memory was changed after release !! }
       pp^.release_sig:=calculate_release_sig(pp);
       if pp=loc_info^.heap_valid_last then
         begin
            loc_info^.heap_valid_last:=pp^.prev_valid;
            if pp=loc_info^.heap_valid_first then
              loc_info^.heap_valid_first:=nil;
            exit(false);
         end;
       pp2:=loc_info^.heap_valid_last;
       while assigned(pp2) do
         begin
            if pp2^.prev_valid=pp then
              begin
                 pp2^.prev_valid:=pp^.prev_valid;
                 if pp=loc_info^.heap_valid_first then
                   loc_info^.heap_valid_first:=pp2;
                 exit(false);
              end
            else
              pp2:=pp2^.prev_valid;
         end;
{$endif EXTRA}
       exit(false);
    end;
  CheckFreeMemSize:=true;
end;

function InternalFreeMemSize(loc_info: pheap_info; p: pointer; pp: pheap_mem_info;
  size: ptruint; release_todo_lock: boolean): ptruint;
var
  i,ppsize : ptruint;
  extra_size: ptruint;
  release_mem: boolean;
begin
  { save old values }
  extra_size:=pp^.extra_info_size;
  ppsize:= size+sizeof(theap_mem_info)+pp^.extra_info_size;
  if add_tail then
    inc(ppsize,tail_size);
  { do various checking }
  release_mem := CheckFreeMemSize(loc_info, pp, size, ppsize);
{$ifdef FPC_HAS_FEATURE_THREADING}
  if release_todo_lock then
    leavecriticalsection(todo_lock);
{$endif}
  if release_mem then
  begin
    { release the normal memory at least }
    i:=SysFreeMemSize(pp,ppsize);
    { return the correct size }
    dec(i,sizeof(theap_mem_info)+extra_size);
    if add_tail then
      dec(i,tail_size);
    InternalFreeMemSize:=i;
  end else
    InternalFreeMemSize:=size;
end;

function TraceFreeMemSize(p:pointer;size:ptruint):ptruint;
var
  loc_info: pheap_info;
  pp: pheap_mem_info;
  release_lock: boolean;
begin
  if p=nil then
    begin
      TraceFreeMemSize:=0;
      exit;
    end;
  loc_info:=@heap_info;
  pp:=pheap_mem_info(p-sizeof(theap_mem_info));
  release_lock:=false;
  if @loc_info^.heap_free_todo <> pp^.todolist then
  begin
    if pp^.todolist = main_orig_todolist then
      pp^.todolist := main_relo_todolist;
{$ifdef FPC_HAS_FEATURE_THREADING}
    entercriticalsection(todo_lock);
{$endif}
    release_lock:=true;
    if pp^.todolist = @orphaned_info.heap_free_todo then
    begin
      loc_info := @orphaned_info;
    end else
    if pp^.todolist <> @loc_info^.heap_free_todo then
    begin
      { allocated in different heap, push to that todolist }
      pp^.todonext := pp^.todolist^;
      pp^.todolist^ := pp;
      TraceFreeMemSize := pp^.size;
{$ifdef FPC_HAS_FEATURE_THREADING}
      leavecriticalsection(todo_lock);
{$endif}
      exit;
    end;
  end;
  TraceFreeMemSize:=InternalFreeMemSize(loc_info,p,pp,size,release_lock);
end;


function TraceMemSize(p:pointer):ptruint;
var
  pp : pheap_mem_info;
begin
  pp:=pheap_mem_info(p-sizeof(theap_mem_info));
  TraceMemSize:=pp^.size;
end;


function TraceFreeMem(p:pointer):ptruint;
var
  l  : ptruint;
  pp : pheap_mem_info;
begin
  if p=nil then
    begin
      TraceFreeMem:=0;
      exit;
    end;
  pp:=pheap_mem_info(p-sizeof(theap_mem_info));
  l:=SysMemSize(pp);
  dec(l,sizeof(theap_mem_info)+pp^.extra_info_size);
  if add_tail then
   dec(l,tail_size);
  { this can never happend normaly }
  if pp^.size>l then
   begin
     if useownfile then
       dump_wrong_size(pp,l,ownfile)
     else
       dump_wrong_size(pp,l,textoutput^);

{$ifdef EXTRA}
     dump_wrong_size(pp,l,error_file);
{$endif EXTRA}
   end;
  TraceFreeMem:=TraceFreeMemSize(p,pp^.size);
end;


{*****************************************************************************
                                ReAllocMem
*****************************************************************************}

function TraceReAllocMem(var p:pointer;size:ptruint):Pointer;
var
  newP: pointer;
  i, allocsize,
  movesize  : ptruint;
  pl : pdword;
  pp,prevpp{$ifdef EXTRA},ppv{$endif} : pheap_mem_info;
  oldsize,
  oldextrasize,
  oldexactsize : ptruint;
  old_fill_extra_info_proc : tfillextrainfoproc;
  old_display_extra_info_proc : tdisplayextrainfoproc;
  loc_info: pheap_info;
begin
{ Free block? }
  if size=0 then
   begin
     if p<>nil then
      TraceFreeMem(p);
     p:=nil;
     TraceReallocMem:=P;
     exit;
   end;
{ Allocate a new block? }
  if p=nil then
   begin
     p:=TraceGetMem(size);
     TraceReallocMem:=P;
     exit;
   end;
{ Resize block }
  loc_info:=@heap_info;
  pp:=pheap_mem_info(p-sizeof(theap_mem_info));
  { test block }
  if ((pp^.sig<>longword(AllocateSig)) or usecrc) and
     ((pp^.sig<>calculate_sig(pp)) or not usecrc) then
   begin
     loc_info^.error_in_heap:=true;
     if useownfile then
       dump_error(pp,ownfile)
     else
       dump_error(pp,textoutput^);
{$ifdef EXTRA}
     dump_error(pp,error_file);
{$endif EXTRA}
     { don't release anything in this case !! }
     if haltonerror then halt(1);
     exit;
   end;
  { save info }
  oldsize:=pp^.size;
  oldextrasize:=pp^.extra_info_size;
  oldexactsize:=pp^.exact_info_size;
  if pp^.extra_info_size>0 then
   begin
     old_fill_extra_info_proc:=pp^.extra_info^.fillproc;
     old_display_extra_info_proc:=pp^.extra_info^.displayproc;
   end;
  { Do the real ReAllocMem, but alloc also for the info block }
{$ifdef cpuarm}
  allocsize:=(size + 3) and not 3+sizeof(theap_mem_info)+pp^.extra_info_size;
{$else cpuarm}
  allocsize:=size+sizeof(theap_mem_info)+pp^.extra_info_size;
{$endif cpuarm}
  if add_tail then
   inc(allocsize,tail_size);
  { Try to resize the block, if not possible we need to do a
    getmem, move data, freemem }
  prevpp:=pp;
  if not SysTryResizeMem(pp,allocsize) then
   begin
     { get a new block }
     newP := TraceGetMem(size);
     { move the data }
     if newP <> nil then
      begin
        movesize:=TraceMemSize(p);
        {if the old size is larger than the new size,
         move only the new size}
        if movesize>size then
          movesize:=size;
        move(p^,newP^,movesize);
      end;
     { release p }
     traceFreeMem(p);
     { return the new pointer }
     p:=newp;
     traceReAllocMem := newp;
     exit;
   end
  else
   begin
     if (pp<>prevpp) then
       begin
         { We need to update the previous/next chains }
         if assigned(pp^.previous) then
           pp^.previous^.next:=pp;
         if assigned(pp^.next) then
           pp^.next^.previous:=pp;
         if prevpp=loc_info^.heap_mem_root then
           loc_info^.heap_mem_root:=pp;
{$ifdef EXTRA}
         { remove prevpp from prev_valid chain }
         ppv:=loc_info^.heap_valid_last;
         if (ppv=prevpp) then
           loc_info^.heap_valid_last:=pp^.prev_valid
         else
           begin
             while assigned(ppv) do
               begin
                 if (ppv^.prev_valid=prevpp) then
                   begin
                     ppv^.prev_valid:=pp^.prev_valid;
                     if prevpp=loc_info^.heap_valid_first then
                       loc_info^.heap_valid_first:=ppv;
                     ppv:=nil;
                   end
                 else
                   ppv:=ppv^.prev_valid;
               end;
           end;
         { Reinsert new value in last position }
         pp^.prev_valid:=loc_info^.heap_valid_last;
         loc_info^.heap_valid_last:=pp;
         if not assigned(loc_info^.heap_valid_first) then
           loc_info^.heap_valid_first:=pp;
{$endif EXTRA}
       end;
   end;
{ Recreate the info block }
  pp^.sig:=longword(AllocateSig);
  pp^.size:=size;
  pp^.extra_info_size:=oldextrasize;
  pp^.exact_info_size:=oldexactsize;
  { add the new extra_info and tail }
  if pp^.extra_info_size>0 then
   begin
     pp^.extra_info:=pointer(pp)+allocsize-pp^.extra_info_size;
     fillchar(pp^.extra_info^,extra_info_size,0);
     pp^.extra_info^.check:=cardinal(CheckSig);
     pp^.extra_info^.fillproc:=old_fill_extra_info_proc;
     pp^.extra_info^.displayproc:=old_display_extra_info_proc;
     if assigned(pp^.extra_info^.fillproc) then
      pp^.extra_info^.fillproc(@pp^.extra_info^.data);
   end
  else
   pp^.extra_info:=nil;
  if add_tail then
    begin
      { Calculate position from start because of arm
        specific alignment }
      pl:=pointer(pp)+sizeof(theap_mem_info)+pp^.size;
      for i:=1 to tail_size div sizeof(dword) do
        begin
          unaligned(pl^):=dword(AllocateSig);
          inc(pointer(pl),sizeof(dword));
        end;
   end;
  { adjust like a freemem and then a getmem, so you get correct
    results in the summary display }
  inc(loc_info^.freemem_size,oldsize);
  inc(loc_info^.freemem8_size,(oldsize+7) and not 7);
  inc(loc_info^.getmem_size,size);
  inc(loc_info^.getmem8_size,(size+7) and not 7);
  { generate new backtrace }
  CaptureBacktrace(1,tracesize,@pp^.calls[1]);
  { regenerate signature }
  if usecrc then
    pp^.sig:=calculate_sig(pp);
  { return the pointer }
  p:=pointer(pp)+sizeof(theap_mem_info);
  TraceReAllocmem:=p;
end;



{*****************************************************************************
                              Check pointer
*****************************************************************************}

{$ifndef Unix}
  {$S-}
{$endif}

{$ifdef go32v2}
var
   __stklen : longword;external name '__stklen';
   __stkbottom : longword;external name '__stkbottom';
   ebss : longword; external name 'end';
{$endif go32v2}

{$ifdef linux}
var
   etext: ptruint; external name '_etext';
   edata : ptruint; external name '_edata';
   eend : ptruint; external name '_end';
{$endif}

{$ifdef freebsd}
var
   text_start: ptruint; external name '__executable_start';
   etext: ptruint; external name '_etext';
   eend : ptruint; external name '_end';
{$endif}

{$ifdef os2}
(* Currently still EMX based - possibly to be changed in the future. *)
var
   etext: ptruint; external name '_etext';
   edata : ptruint; external name '_edata';
   eend : ptruint; external name '_end';
{$endif}

{$ifdef windows}
var
   sdata : ptruint; external name '__data_start__';
   edata : ptruint; external name '__data_end__';
   sbss : ptruint; external name '__bss_start__';
   ebss : ptruint; external name '__bss_end__';
   TLSKey : PDWord; external name '_FPC_TlsKey';
   TLSSize : DWord; external name '_FPC_TlsSize';

function TlsGetValue(dwTlsIndex : DWord) : pointer;
  {$ifdef wince}cdecl{$else}stdcall{$endif};external KernelDLL name 'TlsGetValue';
{$endif}

{$ifdef BEOS}
const
  B_ERROR = -1;

type
  area_id   = Longint;

function area_for(addr : Pointer) : area_id;
            cdecl; external 'root' name 'area_for';
{$endif BEOS}

procedure CheckPointer(p : pointer); [public, alias : 'FPC_CHECKPOINTER'];
var
  i  : ptruint;
  pp : pheap_mem_info;
  loc_info: pheap_info;
{$ifdef go32v2}
  get_ebp,stack_top : longword;
  bss_end : longword;
{$endif go32v2}
{$ifdef windows}
  datap : pointer;
{$endif windows}
  ptext : ^text;
begin
  if p=nil then
    runerror(204);

  i:=0;
  loc_info:=@heap_info;
  if useownfile then
    ptext:=@ownfile
  else
    ptext:=textoutput;

{$ifdef go32v2}
  if ptruint(p)<$1000 then
    runerror(216);
  asm
     movl %ebp,get_ebp
     leal ebss,%eax
     movl %eax,bss_end
  end;
  stack_top:=__stkbottom+__stklen;
  { allow all between start of code and end of bss }
  if ptruint(p)<=bss_end then
    exit;
  { stack can be above heap !! }

  if (ptruint(p)>=get_ebp) and (ptruint(p)<=stack_top) then
    exit;
{$endif go32v2}

  { I don't know where the stack is in other OS !! }
{$ifdef windows}
  { inside stack ? }
  if (ptruint(p)>ptruint(get_frame)) and
     (p<StackTop) then
    exit;
  { inside data, rdata ... bss }
  if (ptruint(p)>=ptruint(@sdata)) and (ptruint(p)<ptruint(@ebss)) then
    exit;
  { is program multi-threaded and p inside Threadvar range? }
  if TlsKey^<>-1 then
    begin
      datap:=TlsGetValue(tlskey^);
      if ((ptruint(p)>=ptruint(datap)) and
          (ptruint(p)<ptruint(datap)+TlsSize)) then
        exit;
    end;
{$endif windows}

{$IFDEF OS2}
  { inside stack ? }
  if (PtrUInt (P) > PtrUInt (Get_Frame)) and
     (PtrUInt (P) < PtrUInt (StackTop)) then
    exit;
  { inside data or bss ? }
  if (PtrUInt (P) >= PtrUInt (@etext)) and (PtrUInt (P) < PtrUInt (@eend)) then
    exit;
{$ENDIF OS2}

{$ifdef linux}
  { inside stack ? }
  if (ptruint(p)>ptruint(get_frame)) and
     (ptruint(p)<ptruint(StackTop)) then
    exit;
  { inside data or bss ? }
  if (ptruint(p)>=ptruint(@etext)) and (ptruint(p)<ptruint(@eend)) then
    exit;
{$endif linux}

{$ifdef freebsd}
  { inside stack ? }
  if (ptruint(p)>ptruint(get_frame)) and
     (ptruint(p)<ptruint(StackTop)) then
    exit;
  { inside data or bss ? }
  if (ptruint(p)>=ptruint(@text_start)) and (ptruint(p)<ptruint(@eend)) then
    exit;
{$endif linux}
{$ifdef morphos}
  { inside stack ? }
  if (ptruint(p)<ptruint(StackTop)) and (ptruint(p)>ptruint(StackBottom)) then
    exit;
  { inside data or bss ? }
  {$WARNING data and bss checking missing }
{$endif morphos}

  {$ifdef darwin}
  {$warning No checkpointer support yet for Darwin}
  exit;
  {$endif}

{$ifdef BEOS}
  // if we find the address in a known area in our current process,
  // then it is a valid one
  if area_for(p) <> B_ERROR then
    exit;
{$endif BEOS}

  { first try valid list faster }

{$ifdef EXTRA}
  pp:=loc_info^.heap_valid_last;
  while pp<>nil do
   begin
     { inside this valid block ! }
     { we can be changing the extrainfo !! }
     if (ptruint(p)>=ptruint(pp)+sizeof(theap_mem_info){+extra_info_size}) and
        (ptruint(p)<=ptruint(pp)+sizeof(theap_mem_info)+extra_info_size+pp^.size) then
       begin
          { check allocated block }
          if ((pp^.sig=longword(AllocateSig)) and not usecrc) or
             ((pp^.sig=calculate_sig(pp)) and usecrc) or
          { special case of the fill_extra_info call }
             ((pp=loc_info^.heap_valid_last) and usecrc and (pp^.sig=longword(AllocateSig))
              and loc_info^.inside_trace_getmem) then
            exit
          else
            begin
              writeln(ptext^,'corrupted heap_mem_info');
              dump_error(pp,ptext^);
              halt(1);
            end;
       end
     else
       pp:=pp^.prev_valid;
     inc(i);
     if i>loc_info^.getmem_cnt-loc_info^.freemem_cnt then
      begin
         writeln(ptext^,'error in linked list of heap_mem_info');
         halt(1);
      end;
   end;
  i:=0;
{$endif EXTRA}
  pp:=loc_info^.heap_mem_root;
  while pp<>nil do
   begin
     { inside this block ! }
     if (ptruint(p)>=ptruint(pp)+sizeof(theap_mem_info)+ptruint(extra_info_size)) and
        (ptruint(p)<=ptruint(pp)+sizeof(theap_mem_info)+ptruint(extra_info_size)+ptruint(pp^.size)) then
        { allocated block }
       if ((pp^.sig=longword(AllocateSig)) and not usecrc) or
          ((pp^.sig=calculate_sig(pp)) and usecrc) then
          exit
       else
         begin
            writeln(ptext^,'pointer $',hexstr(p),' points into invalid memory block');
            dump_error(pp,ptext^);
            runerror(204);
         end;
     pp:=pp^.previous;
     inc(i);
     if i>loc_info^.getmem_cnt then
      begin
         writeln(ptext^,'error in linked list of heap_mem_info');
         halt(1);
      end;
   end;
  writeln(ptext^,'pointer $',hexstr(p),' does not point to valid memory block');
  dump_stack(ptext^,1);
  runerror(204);
end;

{*****************************************************************************
                              Dump Heap
*****************************************************************************}

procedure dumpheap;

begin
  DumpHeap(GlobalSkipIfNoLeaks);
end;

const
{$ifdef BSD}   // dlopen is in libc on FreeBSD.
  LibDL = 'c';
{$else}
  {$ifdef HAIKU}
    LibDL = 'root';
  {$else}
    LibDL = 'dl';
  {$endif}
{$endif}
{$if defined(LINUX) or defined(BSD)}
type
  Pdl_info = ^dl_info;
  dl_info = record
    dli_fname      : Pchar;
    dli_fbase      : pointer;
    dli_sname      : Pchar;
    dli_saddr      : pointer;
  end;

  function _dladdr(Lib:pointer; info: Pdl_info): Longint; cdecl; external LibDL name 'dladdr';
{$elseif defined(MSWINDOWS)}
  function _GetModuleFileNameA(hModule:HModule;lpFilename:PAnsiChar;nSize:cardinal):cardinal;stdcall; external 'kernel32' name 'GetModuleFileNameA';
{$endif}

function GetModuleName:string;
{$ifdef MSWINDOWS}
var
  sz:cardinal;
  buf:array[0..8191] of char;
{$endif}
{$if defined(LINUX) or defined(BSD)}
var
  res:integer;
  dli:dl_info;
{$endif}
begin
  GetModuleName:='';
{$if defined(LINUX) or defined(BSD)}
  res:=_dladdr(@ParamStr,@dli); { get any non-eliminated address in SO space }
  if res<=0 then 
    exit;
  if Assigned(dli.dli_fname) then
    GetModuleName:=PAnsiChar(dli.dli_fname);
{$elseif defined(MSWINDOWS)}
  sz:=_GetModuleFileNameA(hInstance,PChar(@buf),sizeof(buf));
  if sz>0 then
    setstring(GetModuleName,PAnsiChar(@buf),sz)
{$else}
  GetModuleName:=ParamStr(0);
{$endif}
end;

procedure dumpheap(SkipIfNoLeaks : Boolean);
var
  pp : pheap_mem_info;
  i : ptrint;
  ExpectedHeapFree : ptruint;
  status : TFPCHeapStatus;
  ptext : ^text;
  loc_info: pheap_info;
begin
  loc_info:=@heap_info;
  if useownfile then
    ptext:=@ownfile
  else
    ptext:=textoutput;
  pp:=loc_info^.heap_mem_root;
  if ((loc_info^.getmem_size-loc_info^.freemem_size)=0) and SkipIfNoLeaks then
    exit;
  Writeln(ptext^,'Heap dump by heaptrc unit of "'+GetModuleName()+'"');
  Writeln(ptext^,loc_info^.getmem_cnt, ' memory blocks allocated : ',
    loc_info^.getmem_size,'/',loc_info^.getmem8_size);
  Writeln(ptext^,loc_info^.freemem_cnt,' memory blocks freed     : ',
    loc_info^.freemem_size,'/',loc_info^.freemem8_size);
  Writeln(ptext^,loc_info^.getmem_cnt-loc_info^.freemem_cnt,
    ' unfreed memory blocks : ',loc_info^.getmem_size-loc_info^.freemem_size);
  status:=SysGetFPCHeapStatus;
  Write(ptext^,'True heap size : ',status.CurrHeapSize);
  if EntryMemUsed > 0 then
    Writeln(ptext^,' (',EntryMemUsed,' used in System startup)')
  else
    Writeln(ptext^);
  Writeln(ptext^,'True free heap : ',status.CurrHeapFree);
  ExpectedHeapFree:=status.CurrHeapSize
    -(loc_info^.getmem8_size-loc_info^.freemem8_size)
    -(loc_info^.getmem_cnt-loc_info^.freemem_cnt)*(sizeof(theap_mem_info)+extra_info_size)
    -EntryMemUsed;
  If ExpectedHeapFree<>status.CurrHeapFree then
    Writeln(ptext^,'Should be : ',ExpectedHeapFree);
  i:=loc_info^.getmem_cnt-loc_info^.freemem_cnt;
  while pp<>nil do
   begin
     if i<0 then
       begin
          Writeln(ptext^,'Error in heap memory list');
          Writeln(ptext^,'More memory blocks than expected');
          exit;
       end;
     if ((pp^.sig=longword(AllocateSig)) and not usecrc) or
        ((pp^.sig=calculate_sig(pp)) and usecrc) then
       begin
          { this one was not released !! }
          if exitcode<>203 then
            call_stack(pp,ptext^);
          dec(i);
       end
     else if pp^.sig<>longword(ReleaseSig) then
       begin
          dump_error(pp,ptext^);
          if @stderr<>ptext then
            dump_error(pp,stderr);
{$ifdef EXTRA}
          dump_error(pp,error_file);
{$endif EXTRA}
          loc_info^.error_in_heap:=true;
       end
{$ifdef EXTRA}
     else if pp^.release_sig<>calculate_release_sig(pp) then
       begin
          dump_change_after(pp,ptext^);
          dump_change_after(pp,error_file);
          loc_info^.error_in_heap:=true;
       end
{$else not EXTRA}
     else
       begin
         if released_modified(pp,ptext^) then
           exitcode:=203;
       end;
{$endif EXTRA}
       ;
     pp:=pp^.previous;
   end;
  if HaltOnNotReleased and (loc_info^.getmem_cnt<>loc_info^.freemem_cnt) then
    exitcode:=203;
end;


{*****************************************************************************
                                AllocMem
*****************************************************************************}

function TraceAllocMem(size:ptruint):Pointer;
begin
  TraceAllocMem := TraceGetMem(size);
  if Assigned(TraceAllocMem) then
    FillChar(TraceAllocMem^, TraceMemSize(TraceAllocMem), 0);
end;


{*****************************************************************************
                            No specific tracing calls
*****************************************************************************}

procedure TraceInitThread;
var
  loc_info: pheap_info;
begin
  loc_info := @heap_info;
{$ifdef EXTRA}
  loc_info^.heap_valid_first := nil;
  loc_info^.heap_valid_last := nil;
{$endif}
  loc_info^.heap_mem_root := nil;
  loc_info^.getmem_cnt := 0;
  loc_info^.freemem_cnt := 0;
  loc_info^.getmem_size := 0;
  loc_info^.freemem_size := 0;
  loc_info^.getmem8_size := 0;
  loc_info^.freemem8_size := 0;
  loc_info^.error_in_heap := false;
  loc_info^.inside_trace_getmem := false;
  EntryMemUsed := SysGetFPCHeapStatus.CurrHeapUsed;
end;

procedure TraceRelocateHeap;
begin
  main_relo_todolist := @heap_info.heap_free_todo;
{$ifdef FPC_HAS_FEATURE_THREADING}
  initcriticalsection(todo_lock);
{$endif}
end;

procedure move_heap_info(src_info, dst_info: pheap_info);
var
  heap_mem: pheap_mem_info;
begin
  if src_info^.heap_free_todo <> nil then
    finish_heap_free_todo_list(src_info);
  if dst_info^.heap_free_todo <> nil then
    finish_heap_free_todo_list(dst_info);
  heap_mem := src_info^.heap_mem_root;
  if heap_mem <> nil then
  begin
    repeat
      heap_mem^.todolist := @dst_info^.heap_free_todo;
      if heap_mem^.previous = nil then break;
      heap_mem := heap_mem^.previous;
    until false;
    heap_mem^.previous := dst_info^.heap_mem_root;
    if dst_info^.heap_mem_root <> nil then
      dst_info^.heap_mem_root^.next := heap_mem;
    dst_info^.heap_mem_root := src_info^.heap_mem_root;
  end;
  inc(dst_info^.getmem_cnt, src_info^.getmem_cnt);
  inc(dst_info^.getmem_size, src_info^.getmem_size);
  inc(dst_info^.getmem8_size, src_info^.getmem8_size);
  inc(dst_info^.freemem_cnt, src_info^.freemem_cnt);
  inc(dst_info^.freemem_size, src_info^.freemem_size);
  inc(dst_info^.freemem8_size, src_info^.freemem8_size);
  dst_info^.error_in_heap := dst_info^.error_in_heap or src_info^.error_in_heap;
{$ifdef EXTRA}
  if assigned(dst_info^.heap_valid_first) then
    dst_info^.heap_valid_first^.prev_valid := src_info^.heap_valid_last
  else
    dst_info^.heap_valid_last := src_info^.heap_valid_last;
  dst_info^.heap_valid_first := src_info^.heap_valid_first;
{$endif}
end;

procedure TraceExitThread;
var
  loc_info: pheap_info;
begin
  loc_info := @heap_info;
{$ifdef FPC_HAS_FEATURE_THREADING}
  entercriticalsection(todo_lock);
{$endif}
  move_heap_info(loc_info, @orphaned_info);
{$ifdef FPC_HAS_FEATURE_THREADING}
  leavecriticalsection(todo_lock);
{$endif}
end;

function TraceGetHeapStatus:THeapStatus;
begin
  TraceGetHeapStatus:=SysGetHeapStatus;
end;

function TraceGetFPCHeapStatus:TFPCHeapStatus;
begin
    TraceGetFPCHeapStatus:=SysGetFPCHeapStatus;
end;


{*****************************************************************************
                             Program Hooks
*****************************************************************************}

Procedure SetHeapTraceOutput(const name : string);
var i : ptruint;
begin
   if useownfile then
     begin
       useownfile:=false;
       close(ownfile);
     end;
   assign(ownfile,name);
{$I-}
   append(ownfile);
   if IOResult<>0 then
     begin
       Rewrite(ownfile);
       if IOResult<>0 then
         begin
           Writeln(textoutput^,'[heaptrc] Unable to open "',name,'", writing output to stderr instead.');
           useownfile:=false;
           exit;
         end;
     end;
{$I+}
   useownfile:=true;
   for i:=0 to Paramcount do
     write(ownfile,paramstr(i),' ');
   writeln(ownfile);
end;

procedure SetHeapTraceOutput(var ATextOutput : Text);
Begin
  useowntextoutput := True;
  textoutput := @ATextOutput;
end;

procedure SetHeapExtraInfo( size : ptruint;fillproc : tfillextrainfoproc;displayproc : tdisplayextrainfoproc);
begin
  { the total size must stay multiple of 8, also allocate 2 pointers for
    the fill and display procvars }
  exact_info_size:=size + sizeof(theap_extra_info);
  extra_info_size:=(exact_info_size+7) and not 7;
  fill_extra_info_proc:=fillproc;
  display_extra_info_proc:=displayproc;
end;


{*****************************************************************************
                           Install MemoryManager
*****************************************************************************}

const
  TraceManager:TMemoryManager=(
    NeedLock : true;
    Getmem  : @TraceGetMem;
    Freemem : @TraceFreeMem;
    FreememSize : @TraceFreeMemSize;
    AllocMem : @TraceAllocMem;
    ReAllocMem : @TraceReAllocMem;
    MemSize : @TraceMemSize;
    InitThread: @TraceInitThread;
    DoneThread: @TraceExitThread;
    RelocateHeap: @TraceRelocateHeap;
    GetHeapStatus : @TraceGetHeapStatus;
    GetFPCHeapStatus : @TraceGetFPCHeapStatus;
  );

var
  PrevMemoryManager : TMemoryManager;

procedure TraceInit;
begin
  textoutput := @stderr;
  useowntextoutput := false;
  MakeCRC32Tbl;
  main_orig_todolist := @heap_info.heap_free_todo;
  main_relo_todolist := nil;
  TraceInitThread;
  GetMemoryManager(PrevMemoryManager);
  SetMemoryManager(TraceManager);
  useownfile:=false;
  if outputstr <> '' then
     SetHeapTraceOutput(outputstr);
{$ifdef EXTRA}
{$i-}
  Assign(error_file,'heap.err');
  Rewrite(error_file);
{$i+}
  if IOResult<>0 then
    begin
      writeln('[heaptrc] Unable to create heap.err extra log file, writing output to screen.');
      Assign(error_file,'');
      Rewrite(error_file);
    end;
{$endif EXTRA}
  { if multithreading was initialized before heaptrc gets initialized (this is currently
    the case for windows dlls), then RelocateHeap gets never called and the lock
    must be initialized already here,

    however, IsMultithread is not set in this case on windows,
    it is set only if a new thread is started
  }
{$IfNDef WINDOWS}
  if IsMultithread then
{$EndIf WINDOWS}
    TraceRelocateHeap;
end;

procedure TraceExit;
begin
  { no dump if error
    because this gives long long listings }
  { clear inoutres, in case the program that quit didn't }
  ioresult;
  if (exitcode<>0) and (erroraddr<>nil) then
    begin
       if useownfile then
         begin
           Writeln(ownfile,'No heap dump by heaptrc unit');
           Writeln(ownfile,'Exitcode = ',exitcode);
         end
       else
         begin
           Writeln(textoutput^,'No heap dump by heaptrc unit');
           Writeln(textoutput^,'Exitcode = ',exitcode);
         end;
       if useownfile then
         begin
           useownfile:=false;
           close(ownfile);
         end;
       exit;
    end;
  { Disable heaptrc memory manager to avoid problems }
  SetMemoryManager(PrevMemoryManager);
  move_heap_info(@orphaned_info, @heap_info);
  dumpheap;
  if heap_info.error_in_heap and (exitcode=0) then
    exitcode:=203;
{$ifdef FPC_HAS_FEATURE_THREADING}
  if main_relo_todolist <> nil then
    donecriticalsection(todo_lock);
{$endif}
{$ifdef EXTRA}
  Close(error_file);
{$endif EXTRA}
   if useownfile then
     begin
       useownfile:=false;
       close(ownfile);
     end;
  if useowntextoutput then
  begin
    useowntextoutput := false;
    close(textoutput^);
  end;
end;

{$if defined(win32) or defined(win64)}
   function GetEnvironmentStrings : pchar; stdcall;
     external 'kernel32' name 'GetEnvironmentStringsA';
   function FreeEnvironmentStrings(p : pchar) : longbool; stdcall;
     external 'kernel32' name 'FreeEnvironmentStringsA';
Function  GetEnv(envvar: string): string;
var
   s : string;
   i : ptruint;
   hp,p : pchar;
begin
   getenv:='';
   p:=GetEnvironmentStrings;
   hp:=p;
   while hp^<>#0 do
     begin
        s:=strpas(hp);
        i:=pos('=',s);
        if upcase(copy(s,1,i-1))=upcase(envvar) then
          begin
             getenv:=copy(s,i+1,length(s)-i);
             break;
          end;
        { next string entry}
        hp:=hp+strlen(hp)+1;
     end;
   FreeEnvironmentStrings(p);
end;
{$elseif defined(wince)}
Function GetEnv(P:string):Pchar;
begin
  { WinCE does not have environment strings.
    Add some way to specify heaptrc options? }
  GetEnv:=nil;
end;
{$elseif defined(msdos)}
   type
     PFarChar=^Char;far;
     PPFarChar=^PFarChar;
   var
     envp: PPFarChar;external name '__fpc_envp';
Function GetEnv(P:string):string;
var
  ep    : ppfarchar;
  pc    : pfarchar;
  i     : smallint;
  found : boolean;
Begin
  getenv:='';
  p:=p+'=';            {Else HOST will also find HOSTNAME, etc}
  ep:=envp;
  found:=false;
  if ep<>nil then
    begin
      while (not found) and (ep^<>nil) do
        begin
          found:=true;
          for i:=1 to length(p) do
            if p[i]<>ep^[i-1] then
              begin
                found:=false;
                break;
              end;
          if not found then
            inc(ep);
        end;
    end;
  if found then
    begin
      pc:=ep^+length(p);
      while pc^<>#0 do
        begin
          getenv:=getenv+pc^;
          Inc(pc);
        end;
    end;
end;
{$else}
Function GetEnv(P:string):Pchar;
{
  Searches the environment for a string with name p and
  returns a pchar to it's value.
  A pchar is used to accomodate for strings of length > 255
}
var
  ep    : ppchar;
  i     : ptruint;
  found : boolean;
Begin
  p:=p+'=';            {Else HOST will also find HOSTNAME, etc}
  ep:=envp;
  found:=false;
  if ep<>nil then
   begin
     while (not found) and (ep^<>nil) do
      begin
        found:=true;
        for i:=1 to length(p) do
         if p[i]<>ep^[i-1] then
          begin
            found:=false;
            break;
          end;
        if not found then
         inc(ep);
      end;
   end;
  if found then
   getenv:=ep^+length(p)
  else
   getenv:=nil;
end;
{$endif}

procedure LoadEnvironment;
var
  i,j : ptruint;
  s,s2   : string;
  err : word;
begin
  s:=Getenv('HEAPTRC');
  if pos('keepreleased',s)>0 then
   keepreleased:=true;
  if pos('disabled',s)>0 then
   useheaptrace:=false;
  if pos('nohalt',s)>0 then
   haltonerror:=false;
  if pos('haltonnotreleased',s)>0 then
   HaltOnNotReleased :=true;
  if pos('skipifnoleaks',s)>0 then
   GlobalSkipIfNoLeaks :=true;
  if pos('tail_size=',s)>0 then
    begin
      i:=pos('tail_size=',s)+length('tail_size=');
      s2:='';
      while (i<=length(s)) and (s[i] in ['0'..'9']) do
        begin
          s2:=s2+s[i];
          inc(i);
        end;
      val(s2,tail_size,err);
      if err=0 then
        tail_size:=((tail_size + sizeof(ptruint)-1) div sizeof(ptruint)) * sizeof(ptruint)
      else
        tail_size:=sizeof(ptruint);
      add_tail:=(tail_size > 0);
    end;
  i:=pos('log=',s);
  if i>0 then
   begin
     outputstr:=copy(s,i+4,255);
     j:=pos(' ',outputstr);
     if j=0 then
      j:=length(outputstr)+1;
     delete(outputstr,j,255);
   end;
end;


Initialization
  LoadEnvironment;
  { heaptrc can be disabled from the environment }
  if useheaptrace then
   TraceInit;
finalization
  if useheaptrace then
   TraceExit;
end.
