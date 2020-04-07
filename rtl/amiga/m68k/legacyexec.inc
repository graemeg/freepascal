{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 Karoly Balogh, Free Pascal Development team

    Amiga exec.library legacy (OS 1.x/2.x) support functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  This unit implements some missing functions of OS 1.x (and some OS 2.x)
  exec.library, so the legacy OS support can be implemented with minimal
  changes to the normal system unit and common Amiga-like code

  Please note that this code doesn't aim to be API feature complete, just
  functional enough for the RTL code.
}


function AllocVec(byteSize    : Cardinal;
                  requirements: Cardinal): Pointer; public name '_fpc_amiga_allocvec';
var
  p: pointer;
begin
  p:=execAllocMem(byteSize + sizeof(DWord), requirements);
  if p <> nil then
    begin
      PDWord(p)^:=byteSize + sizeof(DWord);
      inc(p, sizeof(DWord));
    end;
  AllocVec:=p;
end;

procedure FreeVec(memoryBlock: Pointer); public name '_fpc_amiga_freevec';
begin
  if memoryBlock <> nil then
    begin
      dec(memoryBlock, sizeof(DWord));
      execFreeMem(memoryBlock,PDWord(memoryBlock)^);
    end;
end;

type
  TAmigaLegacyPoolEntry = record
    pe_node: TMinNode;
    pe_size: dword;
  end;
  PAmigaLegacyPoolEntry = ^TAmigaLegacyPoolEntry;

  TAmigaLegacyPool = record
    pool_requirements: cardinal;
    pool_chain: PAmigaLegacyPoolEntry;
  end;
  PAmigaLegacyPool = ^TAmigaLegacyPool;


function CreatePool(requirements: Cardinal;
                    puddleSize  : Cardinal;
                    threshSize  : Cardinal): Pointer; public name '_fpc_amiga_createpool';
var
  p: PAmigaLegacyPool;
begin
  p:=execAllocMem(sizeof(TAmigaLegacyPool),requirements);
  if p <> nil then
    begin
      p^.pool_requirements:=requirements;
      p^.pool_chain:=nil;
    end;
  CreatePool:=p;
end;

function AllocPooled(poolHeader: Pointer;
                     memSize   : Cardinal): Pointer; public name '_fpc_amiga_allocpooled';
var
  p: PAmigaLegacyPoolEntry;
  ph: PAmigaLegacyPool absolute poolHeader;
begin
  p:=execAllocMem(memSize + sizeof(TAmigaLegacyPoolEntry), ph^.pool_requirements);
  if p <> nil then
    begin
      if ph^.pool_chain <> nil then
        ph^.pool_chain^.pe_node.mln_Pred:=PMinNode(p);
      p^.pe_node.mln_Succ:=PMinNode(ph^.pool_chain);
      p^.pe_node.mln_Pred:=nil;
      p^.pe_size:=memSize + sizeof(TAmigaLegacyPoolEntry);
      ph^.pool_chain:=p;
      inc(pointer(p),sizeof(TAmigaLegacyPoolEntry));
    end;
  AllocPooled:=p;
end;

procedure FreePooled(poolHeader: Pointer;
                    memory    : Pointer;
                    memSize   : Cardinal); public name '_fpc_amiga_freepooled';
var
  p: PAmigaLegacyPoolEntry;
  ph: PAmigaLegacyPool absolute poolHeader;
begin
  if memory <> nil then
    begin
      p:=PAmigaLegacyPoolEntry(memory-sizeof(TAmigaLegacyPoolEntry));
      if p^.pe_node.mln_Succ <> nil then
        PAmigaLegacyPoolEntry(p^.pe_node.mln_Succ)^.pe_node.mln_Pred:=p^.pe_node.mln_Pred;
      if p^.pe_node.mln_Pred <> nil then
        PAmigaLegacyPoolEntry(p^.pe_node.mln_Pred)^.pe_node.mln_Succ:=p^.pe_node.mln_Succ;
      if p = ph^.pool_chain then
        ph^.pool_chain:=PAmigaLegacyPoolEntry(p^.pe_node.mln_Succ);
      execFreeMem(p,p^.pe_size);
    end;
end;

procedure DeletePool(poolHeader: Pointer); public name '_fpc_amiga_deletepool';
var
  p: PAmigaLegacyPool absolute poolHeader;
  pe: PAmigaLegacyPoolEntry;
begin
  if p <> nil then
    begin
      while p^.pool_chain <> nil do
        begin
          pe:=p^.pool_chain;
          FreePooled(poolHeader, pointer(pe) + sizeof(TAmigaLegacyPoolEntry), pe^.pe_size);
        end;
      execFreeMem(p,sizeof(TAmigaLegacyPool));
    end;
end;

procedure StackSwap(newStack: PStackSwapStruct); assembler; nostackframe; public name '_fpc_amiga_stackswap';
asm
    move.l   a6,-(sp)
    move.l   newStack,-(sp)

    move.l   AOS_ExecBase,a6
    sub.l    a1,a1
    jsr      -294(a6)    // FindTask()
    move.l   d0,-(sp)

    move.l   AOS_ExecBase,a6
    jsr      -120(a6)    // Disable()

    move.l   (sp)+,a1    // task
    move.l   (sp)+,a0    // newStack

    move.l   58(a1),d0   // task^.tc_SPLower
    move.l   (a0),58(a1)
    move.l   d0,(a0)+

    move.l   62(a1),d0   // task^.tc_SPUpper
    move.l   (a0),62(a1)
    move.l   d0,(a0)+

    move.l   (sp)+,a6
    move.l   (sp)+,d0     // return address

    move.l   (a0),d1
    move.l   sp,(a0)
    move.l   d1,sp

    move.l   d0,-(sp)
    move.l   a6,-(sp)

    move.l   AOS_ExecBase,a6
    jsr      -126(a6)    // Enable()

    move.l   (sp)+,a6
    rts
end;

procedure ObtainSemaphoreShared(sigSem: PSignalSemaphore); public name '_fpc_amiga_obtainsemaphoreshared';
begin
  { NOTE: this still needs v33+ (OS v1.2 or later) }
  { ObtainSemaphoreShared is used by athreads, and simply replacing
    it by ObtainSemaphore works, just with a slight performance hit,
    at least in the way it's currently used in athreads. }
  ObtainSemaphore(sigSem);
end;
