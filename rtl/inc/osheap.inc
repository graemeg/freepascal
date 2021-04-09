{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2021 by the Free Pascal development team.

    OS heap manager for small targets

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ 
  The OS heap manager is a small heap manager for smaller targets with an
  operating system. It's similar in comcept to the "cmem" memory manager
  for systems with libc support, but it aims systems that have a direct
  heap management API (Sinclair QL, AmigaOS, MacOS Classic, some
  embedded systems, etc), but not necessarily libc. It's also designed
  to be included in the System unit directly, unlike cmem.

  The main advantage is that it's much smaller than the regular heap
  manager, and it's a thinner layer towards the OS as well, therefore it
  can be slightly faster on resource constrained systems. 

  The disadvantage is that it may not be well suited for bigger Pascal
  programs that rely heavily on consistent heap performance, as the
  properties of the OS heap manager like alignment and size constraints,
  and allocation time are also directly exposed towards the Pascal code.
  Additionally, querying the heap status is not supported.

  Its main difference to tinyheap is that it can release the allocated
  memory back to the system, therefore it suits systems that can run more
  than one software in parallel better.

  OSHeap needs SysOSAlloc and SysOSFree implemented in the system-specific
  code, and nothing else.
}

    function SysGetMem(Size: ptruint): pointer;
      begin
        Inc(size,sizeof(size));
        result := SysOSAlloc(size);
        if assigned(result) then
          begin
            pptruint(result)[0] := size;
            Inc(result,sizeof(size));
          end;
      end;

    function SysFreeMem(p: Pointer): ptruint;
      begin
        if assigned(p) then
          begin
            Dec(p,sizeof(ptruint));
            SysOSFree(p,pptruint(p)[0]);
          end;
        result := 0;
      end;

    function SysFreeMemSize(p: Pointer; Size: Ptruint): ptruint;
      begin
        result := SysFreeMem(p);
      end;

    function SysMemSize(p: pointer): ptruint;
      begin
        Dec(p,sizeof(ptruint));
        result:=pptruint(p)[0]-sizeof(ptruint);
      end;

    function SysTryResizeMem(var p: pointer; size: ptruint) : boolean;
      begin
        result := false;
      end;

    function SysAllocMem(size: ptruint): pointer;
      begin
        result := SysGetMem(size);
        if not assigned(result) then
          FillChar(result^,SysMemSize(result),0);
      end;

    function SysReAllocMem(var p: pointer; size: ptruint):pointer;
      var
        oldsize: ptruint;
      begin
        result := nil;
        if assigned(p) then
          begin
            oldsize := SysMemSize(p);
            if size <> oldsize then
              begin
                if size > 0 then
                  begin
                    result := SysGetMem(size);
                    if assigned(result) then
                      begin
                        if size < oldsize then
                          oldsize := size;
                        Move(p^,result^,oldsize);
                      end;
                  end;
                SysFreeMem(p);
              end
            else
              result := p;
          end
        else
          result := SysGetMem(size);

        p := result;
      end;

    function SysGetFPCHeapStatus : TFPCHeapStatus;
      begin
        FillChar(result,sizeof(result),0);
      end;

    function SysGetHeapStatus : THeapStatus;
      begin
        FillChar(result,sizeof(result),0);
      end;
