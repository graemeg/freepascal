{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    Include file with set operations called by the compiler

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
{ the following code is exactly big endian set-related, but specific to the old
  scheme whereby sets were either 4 or 32 bytes. I've left the routines here
  so if someone wants to, they can create equivalents of the new varset helpers
  from rtl/inc/genset.inc
}

{$ifdef FPC_OLD_BIGENDIAN_SETS}
=======
{$ifndef FPC_NEW_BIGENDIAN_SETS}
>>>>>>> graemeg/fixes_2_2
=======
{$ifndef FPC_NEW_BIGENDIAN_SETS}
>>>>>>> origin/fixes_2_2

{$define FPC_SYSTEM_HAS_FPC_SET_LOAD_SMALL}
function fpc_set_load_small(l: fpc_small_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_LOAD_SMALL']; compilerproc;
{
  load a normal set p from a smallset l
}
var
  saveedi : longint;
asm
        movl    %edi,saveedi
        movl    __RESULT,%edi
        movl    l,%eax
{$ifdef FPC_ENABLED_CLD}
        cld
{$endif FPC_ENABLED_CLD}
        stosl
        xorl    %eax,%eax
        movl    $7,%ecx
        rep
        stosl
        movl    saveedi,%edi
end;

{$define FPC_SYSTEM_HAS_FPC_SET_CREATE_ELEMENT}

function fpc_set_create_element(b : byte): fpc_normal_set;assembler;[public,alias:'FPC_SET_CREATE_ELEMENT']; compilerproc;
{
  create a new set in p from an element b
}
var
  saveedi : longint;
asm
        movl    %edi,saveedi
        movl    __RESULT,%edi
        movzbl  b,%edx
        xorl    %eax,%eax
        movl    $8,%ecx
{$ifdef FPC_ENABLED_CLD}
        cld
{$endif FPC_ENABLED_CLD}
        rep
        stosl
        leal    -32(%edi),%eax
        btsl    %edx,(%eax)
        movl    saveedi,%edi
end;


{$define FPC_SYSTEM_HAS_FPC_SET_SET_BYTE}
function fpc_set_set_byte(const source: fpc_normal_set; b : byte): fpc_normal_set;assembler; compilerproc;
{
  add the element b to the set pointed by source
}
var
  saveesi,saveedi : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
        movl    source,%esi
        movl    __RESULT,%edi
        movzbl  b,%edx
        movl    $8,%ecx
{$ifdef FPC_ENABLED_CLD}
        cld
{$endif FPC_ENABLED_CLD}
        rep
        movsl
        leal    -32(%edi),%eax
        btsl    %edx,(%eax)
        movl    saveedi,%edi
        movl    saveesi,%esi
end;

{$define FPC_SYSTEM_HAS_FPC_SET_UNSET_BYTE}
function fpc_set_unset_byte(const source: fpc_normal_set; b : byte): fpc_normal_set;assembler; compilerproc;
{
  add the element b to the set pointed by source
}
var
  saveesi,saveedi : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
        movl    source,%esi
        movl    __RESULT,%edi
        movzbl  b,%edx
        movl    $8,%ecx
{$ifdef FPC_ENABLED_CLD}
        cld
{$endif FPC_ENABLED_CLD}
        rep
        movsl
        leal    -32(%edi),%eax
        btrl    %edx,(%eax)
        movl    saveedi,%edi
        movl    saveesi,%esi
end;

{$define FPC_SYSTEM_HAS_FPC_SET_SET_RANGE}

function fpc_set_set_range(const orgset: fpc_normal_set; l,h : byte): fpc_normal_set;assembler; compilerproc;
{
  adds the range [l..h] to the set pointed to by p
}
var
  saveh : byte;
  saveesi,saveedi,saveebx : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
        movl    %ebx,saveebx
        movl   __RESULT,%edi        // target set address in edi
        movl   orgset, %esi         // source set address in esi
        movzbl l,%eax               // lowest bit to be set in eax
        movzbl h,%ebx               // highest in ebx
        movb   %bl,saveh
        movl   $8,%ecx              // we have to copy 32 bytes
        cmpl   %eax,%ebx            // high < low?
{$ifdef FPC_ENABLED_CLD}
        cld
{$endif FPC_ENABLED_CLD}
        rep                         // copy source to dest (it's possible to do the range
        movsl                       // setting and copying simultanuously of course, but
                                    // that would result in many more jumps and code)
        movl   %eax,%ecx            // lowest also in ecx
        jb     .Lset_range_done     // if high > low, then dest := source
        shrl   $3,%eax              // divide by 8 to get starting and ending byte
        shrl   $3,%ebx              // address
        andb   $31,%cl              // low five bits of lo determine start of bit mask
        andl   $0x0fffffffc,%eax    // clear two lowest bits to get start/end longint
        subl   $32,%edi             // get back to start of dest
        andl   $0x0fffffffc,%ebx    // address * 4
        movl   $0x0ffffffff,%edx    // edx = bitmask to be inserted
        shll   %cl,%edx             // shift bitmask to clear bits below lo
        addl   %eax,%edi            // go to starting pos in set
        subl   %eax,%ebx            // are bit lo and hi in the same longint?
        jz     .Lset_range_hi       // yes, keep current mask and adjust for hi bit
        orl    %edx,(%edi)          // no, store current mask
        movl   $0x0ffffffff,%edx    // new mask
        addl   $4,%edi              // next longint of set
        subl   $4,%ebx              // bit hi in this longint?
        jz     .Lset_range_hi       // yes, keep full mask and adjust for hi bit
.Lset_range_loop:
        movl   %edx,(%edi)          // no, fill longints in between with full mask
        addl   $4,%edi
        subl   $4,%ebx
        jnz    .Lset_range_loop
.Lset_range_hi:
        movb   saveh,%cl                // this is ok, h is on the stack
        movl   %edx,%ebx            // save current bitmask
        andb   $31,%cl
        subb   $31,%cl              // cl := (31 - (hi and 31)) = shift count to
        negb   %cl                  // adjust bitmask for hi bit
        shrl   %cl,%edx             // shift bitmask to clear bits higher than hi
        andl   %edx,%ebx            // combine both bitmasks
        orl    %ebx,(%edi)          // store to set
.Lset_range_done:
        movl    saveedi,%edi
        movl    saveesi,%esi
        movl    saveebx,%ebx
end;

{$define FPC_SYSTEM_HAS_FPC_SET_IN_BYTE}

function fpc_set_in_byte(const p: fpc_normal_set; b: byte): boolean; assembler; [public,alias:'FPC_SET_IN_BYTE']; compilerproc;
{
  tests if the element b is in the set p the carryflag is set if it present
}
asm
{$ifdef REGCALL}
        xchgl %edx,%eax
        andl $0xff,%eax
{$else}
       movl   p,%edx
       movzbl b,%eax
{$endif}
       btl %eax,(%edx)
end;


{$define FPC_SYSTEM_HAS_FPC_SET_ADD_SETS}

function fpc_set_add_sets(const set1,set2: fpc_normal_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_ADD_SETS']; compilerproc;
{
  adds set1 and set2 into set dest
}
var
  saveesi,saveedi : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
{$ifdef REGCALL}
        movl    set1,%esi
        movl    __RESULT,%edi
        movl    set2,%edx
{$else}
      movl set1,%esi
      movl set2,%edx
      movl __RESULT,%edi
{$endif}
{$ifdef FPC_ENABLED_CLD}
      cld
{$endif FPC_ENABLED_CLD}
      movl $8,%ecx
   .LMADDSETS1:
      lodsl
      orl (%edx),%eax
      stosl
      addl $4,%edx
      decl %ecx
      jnz .LMADDSETS1
        movl    saveedi,%edi
        movl    saveesi,%esi
end;


{$define FPC_SYSTEM_HAS_FPC_SET_MUL_SETS}

function fpc_set_mul_sets(const set1,set2: fpc_normal_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_MUL_SETS']; compilerproc;
{
  multiplies (takes common elements of) set1 and set2 result put in dest
}
var
  saveesi,saveedi : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
{$ifdef REGCALL}
        movl    set1,%esi
        movl    __RESULT,%edi
        movl    set2,%edx
{$else}
      movl set1,%esi
      movl set2,%edx
      movl __RESULT,%edi
{$endif}
{$ifdef FPC_ENABLED_CLD}
      cld
{$endif FPC_ENABLED_CLD}
      movl $8,%ecx
  .LMMULSETS1:
      lodsl
      andl (%edx),%eax
      stosl
      addl $4,%edx
      decl %ecx
      jnz .LMMULSETS1
        movl    saveedi,%edi
        movl    saveesi,%esi
end;


{$define FPC_SYSTEM_HAS_FPC_SET_SUB_SETS}

function fpc_set_sub_sets(const set1,set2: fpc_normal_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_SUB_SETS']; compilerproc;
{
  computes the diff from set1 to set2 result in dest
}
var
  saveesi,saveedi,saveebx : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
        movl    %ebx,saveebx
{$ifdef REGCALL}
        movl    set1,%esi
        movl    __RESULT,%edi
        movl    set2,%ebx
{$else}
        movl set1,%esi
        movl set2,%ebx
      movl __RESULT,%edi
{$endif}
{$ifdef FPC_ENABLED_CLD}
        cld
{$endif FPC_ENABLED_CLD}
        movl $8,%ecx
    .LMSUBSETS1:
        lodsl
        movl (%ebx),%edx
        notl %edx
        andl %edx,%eax
        stosl
        addl $4,%ebx
        decl %ecx
        jnz .LMSUBSETS1
        movl    saveedi,%edi
        movl    saveesi,%esi
        movl    saveebx,%ebx
end;


{$define FPC_SYSTEM_HAS_FPC_SET_SYMDIF_SETS}

function fpc_set_symdif_sets(const set1,set2: fpc_normal_set): fpc_normal_set;assembler;[public,alias:'FPC_SET_SYMDIF_SETS']; compilerproc;
{
   computes the symetric diff from set1 to set2 result in dest
}
var
  saveesi,saveedi : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
{$ifdef REGCALL}
        movl    set1,%esi
        movl    __RESULT,%edi
        movl    set2,%edx
{$else}
        movl set1,%esi
        movl set2,%edx
      movl __RESULT,%edi
{$endif}
{$ifdef FPC_ENABLED_CLD}
        cld
{$endif FPC_ENABLED_CLD}
        movl $8,%ecx
    .LMSYMDIFSETS1:
        lodsl
        xorl (%edx),%eax
        stosl
        addl $4,%edx
        decl %ecx
        jnz .LMSYMDIFSETS1
        movl    saveedi,%edi
        movl    saveesi,%esi
end;


{$define FPC_SYSTEM_HAS_FPC_SET_COMP_SETS}

function fpc_set_comp_sets(const set1,set2: fpc_normal_set): boolean;assembler;[public,alias:'FPC_SET_COMP_SETS']; compilerproc;
{
  compares set1 and set2 zeroflag is set if they are equal
}
var
  saveesi,saveedi : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
        movl set1,%esi
        movl set2,%edi
        movl $8,%ecx
    .LMCOMPSETS1:
        movl (%esi),%eax
        movl (%edi),%edx
        cmpl %edx,%eax
        jne  .LMCOMPSETEND
        addl $4,%esi
        addl $4,%edi
        decl %ecx
        jnz .LMCOMPSETS1
        { we are here only if the two sets are equal
          we have zero flag set, and that what is expected }
    .LMCOMPSETEND:
        seteb %al
        movl    saveedi,%edi
        movl    saveesi,%esi
end;


{$define FPC_SYSTEM_HAS_FPC_SET_CONTAINS_SET}

function fpc_set_contains_sets(const set1,set2: fpc_normal_set): boolean;assembler;[public,alias:'FPC_SET_CONTAINS_SETS']; compilerproc;
{
  on exit, zero flag is set if set1 <= set2 (set2 contains set1)
}
var
  saveesi,saveedi : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
        movl set1,%esi
        movl set2,%edi
        movl $8,%ecx
    .LMCONTAINSSETS1:
        movl (%esi),%eax
        movl (%edi),%edx
        andl %eax,%edx
        cmpl %edx,%eax  {set1 and set2 = set1?}
        jne  .LMCONTAINSSETEND
        addl $4,%esi
        addl $4,%edi
        decl %ecx
        jnz .LMCONTAINSSETS1
        { we are here only if set2 contains set1
          we have zero flag set, and that what is expected }
    .LMCONTAINSSETEND:
        seteb %al
        movl    saveedi,%edi
        movl    saveesi,%esi
end;


{$ifdef LARGESETS}

{$error Needs to be fixed for register calling first!}

procedure fpc_largeset_set_word(p : pointer;b : word);assembler;[public,alias:'FPC_LARGESET_SET_WORD']; compilerproc;
{
  sets the element b in set p works for sets larger than 256 elements
  not yet use by the compiler so
}
asm
       pushl %eax
       movl p,%edi
       movw b,%ax
       andl $0xfff8,%eax
       shrl $3,%eax
       addl %eax,%edi
       movb 12(%ebp),%al
       andl $7,%eax
       btsl %eax,(%edi)
       popl %eax
end;


procedure fpc_largeset_in_word(p : pointer;b : word);assembler;[public,alias:'FPC_LARGESET_IN_WORD']; compilerproc;
{
  tests if the element b is in the set p the carryflag is set if it present
  works for sets larger than 256 elements
}
asm
        pushl %eax
        movl p,%edi
        movw b,%ax
        andl $0xfff8,%eax
        shrl $3,%eax
        addl %eax,%edi
        movb 12(%ebp),%al
        andl $7,%eax
        btl %eax,(%edi)
        popl %eax
end;


procedure fpc_largeset_add_sets(set1,set2,dest : pointer;size : longint);assembler;[public,alias:'FPC_LARGESET_ADD_SETS']; compilerproc;
{
  adds set1 and set2 into set dest size is the number of bytes in the set
}
asm
      movl set1,%esi
      movl set2,%ebx
      movl dest,%edi
      movl size,%ecx
{$ifdef FPC_ENABLED_CLD}
      cld
{$endif FPC_ENABLED_CLD}
  .LMADDSETSIZES1:
      lodsl
      orl (%ebx),%eax
      stosl
      addl $4,%ebx
      decl %ecx
      jnz .LMADDSETSIZES1
end;


procedure fpc_largeset_mul_sets(set1,set2,dest : pointer;size : longint);assembler;[public,alias:'FPC_LARGESET_MUL_SETS']; compilerproc;
{
  multiplies (i.E. takes common elements of) set1 and set2 result put in
  dest size is the number of bytes in the set
}
asm
         movl set1,%esi
         movl set2,%ebx
         movl dest,%edi
         movl size,%ecx
{$ifdef FPC_ENABLED_CLD}
         cld
{$endif FPC_ENABLED_CLD}
     .LMMULSETSIZES1:
         lodsl
         andl (%ebx),%eax
         stosl
         addl $4,%ebx
         decl %ecx
         jnz .LMMULSETSIZES1
end;


procedure fpc_largeset_sub_sets(set1,set2,dest : pointer;size : longint);assembler;[public,alias:'FPC_LARGESET_SUB_SETS']; compilerproc;
asm
         movl set1,%esi
         movl set2,%ebx
         movl dest,%edi
         movl size,%ecx
{$ifdef FPC_ENABLED_CLD}
         cld
{$endif FPC_ENABLED_CLD}
     .LMSUBSETSIZES1:
         lodsl
         movl (%ebx),%edx
         notl %edx
         andl %edx,%eax
         stosl
         addl $4,%ebx
         decl %ecx
         jnz .LMSUBSETSIZES1
end;


procedure fpc_largeset_symdif_sets(set1,set2,dest : pointer;size : longint);assembler;[public,alias:'FPC_LARGESET_SYMDIF_SETS']; compilerproc;
{
   computes the symetric diff from set1 to set2 result in dest
}
asm
      movl set1,%esi
      movl set2,%ebx
      movl dest,%edi
      movl size,%ecx
{$ifdef FPC_ENABLED_CLD}
      cld
{$endif FPC_ENABLED_CLD}
  .LMSYMDIFSETSIZE1:
      lodsl
      movl (%ebx),%edx
      xorl %edx,%eax
      stosl
      addl $4,%ebx
      decl %ecx
      jnz .LMSYMDIFSETSIZE1
end;


procedure fpc_largeset_comp_sets(set1,set2 : pointer;size : longint);assembler;[public,alias:'FPC_LARGESET_COMP_SETS']; compilerproc;
asm
      movl set1,%esi
      movl set2,%edi
      movl size,%ecx
{$ifdef FPC_ENABLED_CLD}
      cld
{$endif FPC_ENABLED_CLD}
  .LMCOMPSETSIZES1:
      lodsl
      movl (%edi),%edx
      cmpl %edx,%eax
      jne  .LMCOMPSETSIZEEND
      addl $4,%edi
      decl %ecx
      jnz .LMCOMPSETSIZES1
      { we are here only if the two sets are equal
        we have zero flag set, and that what is expected }
  .LMCOMPSETSIZEEND:
end;

procedure fpc_largeset_contains_sets(set1,set2 : pointer; size: longint);assembler;[public,alias:'FPC_LARGESET_CONTAINS_SETS']; compilerproc;
{
  on exit, zero flag is set if set1 <= set2 (set2 contains set1)
}
asm
        movl set1,%esi
        movl set2,%edi
        movl size,%ecx
    .LMCONTAINSSETS2:
        movl (%esi),%eax
        movl (%edi),%edx
        andl %eax,%edx
        cmpl %edx,%eax  {set1 and set2 = set1?}
        jne  .LMCONTAINSSETEND2
        addl $4,%esi
        addl $4,%edi
        decl %ecx
        jnz .LMCONTAINSSETS2
        { we are here only if set2 contains set1
          we have zero flag set, and that what is expected }
    .LMCONTAINSSETEND2:
end;


{$endif LARGESET}

<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_OLD_BIGENDIAN_SETS}
=======
{$endif ndef FPC_NEW_BIGENDIAN_SETS}
>>>>>>> graemeg/fixes_2_2
=======
{$endif ndef FPC_NEW_BIGENDIAN_SETS}
>>>>>>> origin/fixes_2_2
