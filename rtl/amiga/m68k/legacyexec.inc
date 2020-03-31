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
                  requirements: Cardinal): Pointer;
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

procedure FreeVec(memoryBlock: Pointer);
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
                    threshSize  : Cardinal): Pointer;
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

procedure DeletePool(poolHeader: Pointer);
begin
  
{$warning DeletePool unimplemented!}
end;

function AllocPooled(poolHeader: Pointer;
                     memSize   : Cardinal): Pointer;
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
                    memSize   : Cardinal);
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

procedure StackSwap(newStack: PStackSwapStruct);
begin
{$warning StackSwap unimplemented!}
end;

procedure ObtainSemaphoreShared(sigSem: PSignalSemaphore);
begin
  { NOTE: this still needs v33+ (OS v1.2 or later) }
  { ObtainSemaphoreShared is used by athreads, and simply replacing
    it by ObtainSemaphore works, just with a slight performance hit,
    at least in the way it's currently used in athreads. }
  ObtainSemaphore(sigSem);
end;
