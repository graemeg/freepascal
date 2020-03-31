type
  __darwin_natural_t = cardinal;
  __darwin_mach_port_name_t = __darwin_natural_t;
  __darwin_mach_port_t = __darwin_mach_port_name_t;
{$ifdef cpu64}
  boolean_t = cardinal;
  vm_offset_t = ptruint;
  vm_size_t = ptruint;
{$else}
  boolean_t = longint;
  vm_offset_t = __darwin_natural_t;
  vm_size_t = __darwin_natural_t;
{$endif}
  mach_port_t = __darwin_mach_port_t;
  vm_map_t = mach_port_t;
  vm_address_t = vm_offset_t;
  pvm_address_t = ^vm_address_t;
  vm_prot_t = longint;
  pvm_prot_t = ^vm_prot_t;
  vm_inherit_t = cardinal;
  kern_return_t = longint;

const
  KERN_SUCCESS                       = 0;

  VM_FLAGS_FIXED                     = $0000;
  VM_FLAGS_ANYWHERE                  = $0001;
  VM_FLAGS_PURGABLE                  = $0002;
(*
  flags that are different between 10.4 and 10.12; fortunately, we don't need them

  VM_FLAGS_RANDOM_ADDR               = $0008;
  VM_FLAGS_NO_CACHE                  = $0010;
  VM_FLAGS_RESILIENT_CODESIGN        = $0020;
  VM_FLAGS_RESILIENT_MEDIA           = $0040;
  VM_FLAGS_OVERWRITE                 = $4000;   { delete any existing mappings first }
  {*
   * VM_FLAGS_SUPERPAGE_MASK
   *      3 bits that specify whether large pages should be used instead of
   *      base pages (!=0), as well as the requested page size.
   *}
  VM_FLAGS_SUPERPAGE_MASK            = $70000;  {* bits 0x10000, 0x20000, 0x40000 *}
  VM_FLAGS_RETURN_DATA_ADDR          = $100000; {* Return address of target data, rather than base of page *}
  VM_FLAGS_RETURN_4K_DATA_ADDR       = $800000; {* Return 4K aligned address of target data *}
*)

  VM_INHERIT_SHARE                   = vm_inherit_t(0); {* share with child *}
  VM_INHERIT_COPY                    = vm_inherit_t(1); {* copy into child *}
  VM_INHERIT_NONE                    = vm_inherit_t(2); {* absent from child *}
  VM_INHERIT_DONATE_COPY             = vm_inherit_t(3); {* copy and delete *}

  VM_INHERIT_DEFAULT                 = VM_INHERIT_COPY;
  VM_INHERIT_LAST_VALID              = VM_INHERIT_NONE;

  VM_MEMORY_MALLOC                   = 1;
  VM_MEMORY_MALLOC_SMALL             = 2;
  VM_MEMORY_MALLOC_LARGE             = 3;
  VM_MEMORY_MALLOC_HUGE              = 4;
  VM_MEMORY_SBRK                     = 5; // uninteresting -- no one should call
  VM_MEMORY_REALLOC                  = 6;
  VM_MEMORY_MALLOC_TINY              = 7;


var
  mach_task_self_: mach_port_t; cvar; external;

  darwin_page_size: vm_size_t;


  function getpagesize: longint; cdecl; external;

  procedure darwin_init_page_size;
    begin
      darwin_page_size:=getpagesize;
    end;


  function mach_task_self: mach_port_t; inline;
    begin
      result:=mach_task_self_;
    end;


  function vm_allocate(target: vm_map_t;
                       address: pvm_address_t;
                       size: vm_size_t;
                       flags: longint): kern_return_t; cdecl; external;

  function vm_deallocate(target: vm_map_t;
                         address: vm_address_t;
                         size: vm_size_t) : kern_return_t; cdecl; external;

  function vm_remap(target_task: vm_map_t;
                         target_address: pvm_address_t;
                         size: vm_size_t;
                         mask: vm_offset_t;
                         flags: longint;
                         src_task: vm_map_t;
                         src_address: vm_address_t;
                         copy: boolean_t;
                         cur_protection, max_protection: pvm_prot_t;
                         inheritance: vm_inherit_t): kern_return_t; cdecl; external;

  function DARWIN_VM_MAKE_TAG(tag: longint): longint; inline;
    begin
      result:=longint(cardinal(tag) shl 24);
    end;

  function darwin_round_page(size: vm_size_t): vm_size_t; inline;
    var
      local_page_size: vm_size_t;
    begin
      local_page_size:=darwin_page_size;
      result:=(size+local_page_size-1) and not(local_page_size-1);
    end;

  {$define HAS_SYSOSALLOC}
  function SysOSAlloc(size: ptruint): pointer;
  var
    addr: vm_address_t;
    tag: longint;
  begin
    addr:=0;
    if size<=growheapsizesmall then
      tag:=DARWIN_VM_MAKE_TAG(VM_MEMORY_MALLOC_TINY) or VM_FLAGS_ANYWHERE
    else if size<=growheapsize2 then
      tag:=DARWIN_VM_MAKE_TAG(VM_MEMORY_MALLOC) or VM_FLAGS_ANYWHERE
    else
      tag:=DARWIN_VM_MAKE_TAG(VM_MEMORY_MALLOC_LARGE) or VM_FLAGS_ANYWHERE;

    if vm_allocate(mach_task_self(), @addr, darwin_round_page(size), tag)=KERN_SUCCESS then
      result:=pointer(addr)
    else
      result:=nil;
  end;


  {$define HAS_SYSOSFREE}
  procedure SysOSFree(p: pointer; size: ptruint);
  begin
    if vm_deallocate(mach_task_self(), vm_address_t(p), darwin_round_page(size))<>KERN_SUCCESS then
      HandleError(204);
  end;


{$define FPC_SYSTEM_HAS_SYSOSREALLOC}
function SysOSRealloc(p: pointer;oldsize,newsize: ptruint): pointer;
  var
    addr: vm_address_t;
    oldsize_rounded: vm_address_t;
  begin
    oldsize_rounded:=darwin_round_page(oldsize);
    addr:=vm_address_t(p)+oldsize_rounded;
    { fits in the previously allocated block -> return directly}
    if (vm_address_t(p+newsize)<=addr) or
    { otherwise try to map extra space at the end }
       (vm_allocate(mach_task_self(), @addr, darwin_round_page(newsize-oldsize_rounded), DARWIN_VM_MAKE_TAG(VM_MEMORY_REALLOC))=KERN_SUCCESS) then
      result:=p
    else
      result:=nil;
  end;

