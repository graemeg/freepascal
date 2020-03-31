{-
 * Copyright (c) 2018 The FreeBSD Foundation
 *
 * This software was developed by Konstantin Belousov <kib@FreeBSD.org>
 * under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 }

//#include <sys/cdefs.h>
//__FBSDID("$FreeBSD: releng/12.1/lib/csu/amd64/reloc.c 339351 2018-10-13 23:52:55Z kib $");

//#include <machine/specialreg.h>
//#include <machine/cpufunc.h>

Type
    Elf_Addr = uint64;
    PElf_Addr = ^Elf_Addr;
    TElfAddrProc = function (feat1,feat2,stdextfeat1,stdextfeat2:uint32):Elf_Addr; cdecl;

const  R_X86_64_IRELATIVE      = 37;

procedure freebsdIdentTag;nostackframe;assembler;
  asm
    .section ".note.openbsd.ident", "a"
    .p2align 2
    .long    8
    .long    4
    .long    1
    .asciz   "OpenBSD"
    .long    0

    .text
  end;

function ELF_R_TYPE(info:elf_addr):uint32;inline;
begin
  ELF_R_TYPE:=info and  uint32($ffffffff);
end;

{$asmmode intel}
procedure do_cpuid(funcnr : integer; var p);
begin
  asm
    mov  rax,rdi
    cpuid
    mov [rsi],eax
    mov [rsi+4],ebx
    mov [rsi+8],ecx
    mov [rsi+12],edx
  end['rbx']; // rbx is callee saved in sysv
end;

procedure cpuid_count(funcnr : integer;count:integer;var p);
begin
  asm
    mov  rax,rdi  // funcnr
    mov  rcx,rsi  // count
    mov  rsi,rdx  // save pointer since cpuid destroys edx
    cpuid
    mov [rsi],eax
    mov [rsi+4],ebx
    mov [rsi+8],ecx
    mov [rsi+12],edx
  end['rbx'];
end;

procedure crt1_handle_rela(r : pelf_rela);
var p : array[0..3] of uint32;
    ptr,
    where : PElf_Addr;
    target  : Elf_Addr;
    cpu_feature,
    cpu_feature2          : uint32;
    cpu_stdext_feature,
    cpu_stdext_feature2   : uint32;
begin
        do_cpuid(1, p);
        cpu_feature  := p[3];
        cpu_feature2 := p[2];
        do_cpuid(0, p);
        if (p[0] >= 7) then
          begin
            cpuid_count(7, 0, p);
            cpu_stdext_feature  := p[1];
            cpu_stdext_feature2 := p[2];
          end
        else
          begin
            cpu_stdext_feature  := 0;
            cpu_stdext_feature2 := 0;
          end;

        case (ELF_R_TYPE(r^.r_info)) of
          R_X86_64_IRELATIVE:
             begin
                ptr :=  PElf_Addr(r^.r_addend);
                where := PElf_Addr (r^.r_offset);
                target := TElfAddrProc(ptr)(cpu_feature, cpu_feature2,
                    cpu_stdext_feature, cpu_stdext_feature2);
                where^:=target;
              end;
          end;
end;

