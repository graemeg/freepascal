{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2011 by the Free Pascal development team.

    Tiny heap manager for the FPC FreeRTOS target

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$modeswitch result}
Unit heapmgr;

  interface

    procedure RegisterHeapBlock(AAddress: pointer; ASize: ptruint);
    
    function GetAlignedMem(Size, Alignment: ptruint): pointer;

  implementation

    const
      MinBlock = 16;

    type
      PHeapBlock = ^THeapBlock;
      THeapBlock = record
        Size: ptruint;
        Next: PHeapBlock;
        EndAddr: pointer;
      end;

    var
      Blocks: PHeapBlock = nil;

    procedure InternalFreeMem(Addr: Pointer; Size: ptruint); forward;

    function FindSize(p: pointer): ptruint; inline;
      begin
        FindSize := PPtrUInt(p)[-1];
      end;

    function SysGetMem(Size: ptruint): pointer;
      var
        p, prev: PHeapBlock;
        AllocSize, RestSize: ptruint;
      begin
        if size+sizeof(PtrUInt)<MinBlock then
          AllocSize := MinBlock
        else
          AllocSize := align(size+sizeof(PtrUInt), sizeof(pointer));

        p := Blocks;
        prev := nil;
        while assigned(p) and (p^.Size < AllocSize) do
          begin
            prev := p;
            p := p^.Next;
          end;

        if assigned(p) then
          begin
            result := @pptruint(p)[1];

            if (p^.size > AllocSize) and
               (p^.Size-AllocSize >= MinBlock) then
              RestSize := p^.Size-AllocSize
            else
              begin
                AllocSize := p^.Size;
                RestSize := 0;
              end;

            if prev = nil then
              Blocks := p^.Next
            else
              prev^.next := p^.next;

            pptruint(p)^ := size;

            InternalFreemem(pointer(ptruint(p)+AllocSize), RestSize);
          end
        else
          begin
            if ReturnNilIfGrowHeapFails then
              Result := nil
            else
              RunError(203);
          end;
      end;

    function GetAlignedMem(Size, Alignment: ptruint): pointer;
      var
        mem: Pointer;
        memp: ptruint;
      begin
        if alignment <= sizeof(pointer) then
          result := GetMem(size)
        else
          begin
            mem := GetMem(Size+Alignment-1+MinBlock);
            memp := align(ptruint(mem)+MinBlock, Alignment);
            InternalFreemem(mem, ptruint(memp)-ptruint(mem));
            result := pointer(memp);
          end;
      end;

    procedure InternalFreeMem(Addr: Pointer; Size: ptruint);
      var
        b, p, prev: PHeapBlock;
        concatenated: boolean;
      begin
        if size<=0 then
          exit;

        concatenated := true;
        while concatenated do
          begin
            concatenated := false;
            b := addr;

            b^.Next := Blocks;
            b^.Size := Size;
            b^.EndAddr := pointer(ptruint(addr)+size);

            if Blocks = nil then
              Blocks := b
            else
              begin
                p := Blocks;
                prev := nil;

                while assigned(p) do
                  begin
                    if p^.EndAddr = addr then
                      begin
                        addr:=p;
                        size:=p^.size+size;
                        if prev = nil then
                          blocks:=p^.next
                        else
                          prev^.next:=p^.next;
                        concatenated:=true;
                        break;
                      end
                    else if p = b^.EndAddr then
                      begin
                        size:=p^.size+size;
                        if prev = nil then
                          blocks:=p^.next
                        else
                          prev^.next:=p^.next;
                        concatenated:=true;
                        break;
                      end;

                    prev := p;
                    p := p^.next;
                  end;

                if not concatenated then
                  begin
                    p := Blocks;
                    prev := nil;

                    while assigned(p) and (p^.Size < size) do
                      begin
                        prev := p;
                        p := p^.Next;
                      end;

                    if assigned(prev) then
                      begin
                        b^.Next := p;
                        prev^.Next := b;
                      end
                    else
                      Blocks := b;
                  end;
              end;
          end;
      end;

    function SysFreeMem(Addr: Pointer): ptruint;
      var
        sz: ptruint;
      begin
        if addr=nil then
          begin
            result:=0;
            exit;
          end;
        sz := Align(FindSize(addr)+SizeOf(pointer), sizeof(pointer));

        InternalFreeMem(@pptruint(addr)[-1], sz);

        result := sz;
      end;

    function SysFreeMemSize(Addr: Pointer; Size: Ptruint): ptruint;
      begin
        result := SysFreeMem(addr);
      end;

    function SysMemSize(p: pointer): ptruint;
      begin
        result := findsize(p);
      end;

    function SysAllocMem(size: ptruint): pointer;
      begin
        result := SysGetMem(size);
        if result<>nil then
          FillChar(pbyte(result)^,size,0);
      end;

    function SysReAllocMem(var p: pointer; size: ptruint):pointer;
      var
        sz: ptruint;
      begin
        if size=0 then
          begin
            SysFreeMem(p);
            result := nil;
            p := nil;
          end
        else if p=nil then
          begin
            result := AllocMem(size);
            p := result;
          end
        else
          begin
            result := AllocMem(size);
            if result <> nil then
              begin
                if p <> nil then
                  begin
                    sz := FindSize(p);
                    if sz > size then
                      sz := size;
                    move(pbyte(p)^, pbyte(result)^, sz);
                  end;
              end;
            SysFreeMem(p);
            p := result;
          end;
      end;

    procedure RegisterHeapBlock(AAddress: pointer; ASize: ptruint);
      begin
        InternalFreeMem(AAddress, ASize);
      end;

    { avoid that programs crash due to a heap status request }
    function SysGetFPCHeapStatus : TFPCHeapStatus;
      begin
        FillChar(Result,SizeOf(Result),0);
      end;

    { avoid that programs crash due to a heap status request }
    function SysGetHeapStatus : THeapStatus;
      begin
        FillChar(Result,SizeOf(Result),0);
      end;

    const
      MyMemoryManager: TMemoryManager = (
        NeedLock: false;  // Obsolete
        GetMem: @SysGetMem;
        FreeMem: @SysFreeMem;
        FreeMemSize: @SysFreeMemSize;
        AllocMem: @SysAllocMem;
        ReAllocMem: @SysReAllocMem;
        MemSize: @SysMemSize;
        InitThread: nil;
        DoneThread: nil;
        RelocateHeap: nil;
        GetHeapStatus: @SysGetHeapStatus;
        GetFPCHeapStatus: @SysGetFPCHeapStatus;
      );

var
  initialheap : record end; external name '__fpc_initialheap';
  heapsize : PtrInt; external name '__heapsize';


initialization
  SetMemoryManager(MyMemoryManager);
  RegisterHeapBlock(@initialheap,heapsize);
finalization
  //FinalizeHeap;
end.
