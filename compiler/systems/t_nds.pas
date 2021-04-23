{
    This unit implements support import,export,link routines
    for the (arm) Nintendo DS target

    Copyright (c) 2001-2002 by Peter Vreman

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit t_nds;

{$i fpcdefs.inc}

interface


implementation

    uses
       aasmbase,
       SysUtils,
       cutils,cfileutl,cclasses,
       globtype,globals,systems,verbose,cscript,fmodule,i_nds,link;

    type
       TlinkerNDS=class(texternallinker)
       private
          Function  WriteResponseFile: Boolean;
       public
          constructor Create; override;
          procedure SetDefaultInfo; override;
          function  MakeExecutable:boolean; override;
       end;



{*****************************************************************************
                                  TLINKERNDS
*****************************************************************************}

Constructor TLinkerNDS.Create;
begin
  Inherited Create;
  SharedLibFiles.doubles:=true;
  StaticLibFiles.doubles:=true;
  // set arm9 as default apptype
  if (apptype <> app_arm9) and (apptype <> app_arm7) then
    apptype:=app_arm9;
end;


procedure TLinkerNDS.SetDefaultInfo;
begin
  with Info do
   begin
     ExeCmd[1]:='ld -g $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP -L. -o $EXE -T $RES';
   end;
end;


Function TLinkerNDS.WriteResponseFile: Boolean;
Var
  linkres  : TLinkRes;
  i        : longint;
  HPath    : TCmdStrListItem;
  s,s1,s2  : TCmdStr;
  prtobj, 
  cprtobj  : string[80];
  linklibc,
  linklibgcc : boolean;
  found1,
  found2   : boolean;
begin
  WriteResponseFile:=False;
  linklibc:=(SharedLibFiles.Find('c')<>nil);
  linklibgcc:=(SharedLibFiles.Find('gcc')<>nil);

  prtobj:='';
  cprtobj:='';
  case apptype of
    app_arm9:
      begin
        prtobj:='prt09';
        cprtobj:='cprt09';
      end;
    app_arm7:
      begin
        prtobj:='prt07';
        cprtobj:='cprt07';
      end;
    else
      internalerror(2019050935);
  end;
  
  if (linklibc or linklibgcc) then
    prtobj:=cprtobj;

	{ Open link.res file }
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);

  { Write path to search libraries }
  HPath:=TCmdStrListItem(current_module.locallibrarysearchpath.First);
  while assigned(HPath) do
   begin
    s:=HPath.Str;
    if (cs_link_on_target in current_settings.globalswitches) then
     s:=ScriptFixFileName(s);
    LinkRes.Add('-L'+s);
    HPath:=TCmdStrListItem(HPath.Next);
   end;
  HPath:=TCmdStrListItem(LibrarySearchPath.First);
  while assigned(HPath) do
   begin
    s:=HPath.Str;
    if s<>'' then
     LinkRes.Add('SEARCH_DIR("'+s+'")');
    HPath:=TCmdStrListItem(HPath.Next);
   end;

  LinkRes.Add('INPUT (');
  { add objectfiles, start with prt0 always }
  if prtobj<>'' then
   s:=FindObjectFile(prtobj,'',false);
  LinkRes.AddFileName(s);
  { try to add crti and crtbegin if linking to C }
  if linklibc then
   begin
     if librarysearchpath.FindFile('crti.o',false,s) then
      LinkRes.AddFileName(s);
   end;
  if linklibgcc then
   begin
     if librarysearchpath.FindFile('crtbegin.o',false,s) then
       LinkRes.AddFileName(s);
   end;
  while not ObjectFiles.Empty do
   begin
    s:=ObjectFiles.GetFirst;
    if s<>'' then
     begin
      { vlink doesn't use SEARCH_DIR for object files }
      if not(cs_link_on_target in current_settings.globalswitches) then
       s:=FindObjectFile(s,'',false);
      LinkRes.AddFileName((maybequoted(s)));
     end;
   end;

  { Write staticlibraries }
  if not StaticLibFiles.Empty then
   begin
    { vlink doesn't need, and doesn't support GROUP }
    if (cs_link_on_target in current_settings.globalswitches) then
     begin
      LinkRes.Add(')');
      LinkRes.Add('GROUP(');
     end;
    while not StaticLibFiles.Empty do
     begin
      S:=StaticLibFiles.GetFirst;
      LinkRes.AddFileName((maybequoted(s)));
     end;
   end;

  if (cs_link_on_target in current_settings.globalswitches) then
   begin
    LinkRes.Add(')');

    { Write sharedlibraries like -l<lib>, also add the needed dynamic linker
      here to be sure that it gets linked this is needed for glibc2 systems (PFV) }
    linklibc:=false;
    linklibgcc:=false;
    while not SharedLibFiles.Empty do
     begin
      S:=SharedLibFiles.GetFirst;
      if s<>'c' then
       begin
        i:=Pos(target_info.sharedlibext,S);
        if i>0 then
         Delete(S,i,255);
        LinkRes.Add('-l'+s);
       end
      else
       begin
        LinkRes.Add('-l'+s);
        linklibc:=true;
        linklibgcc:=true;
       end;
     end;
    { be sure that libc&libgcc is the last lib }
    if linklibgcc then
     begin
      LinkRes.Add('-lgcc');
     end;
    if linklibc then
     begin
      LinkRes.Add('-lc');
     end;
   end
  else
   begin
    while not SharedLibFiles.Empty do
     begin
      S:=SharedLibFiles.GetFirst;
      LinkRes.Add('lib'+s+target_info.staticlibext);
     end;
    LinkRes.Add(')');
   end;

  { objects which must be at the end }
  if linklibgcc then
   begin
     found1:=librarysearchpath.FindFile('crtend.o',false,s1);
     if found1 then
      begin
        LinkRes.Add('INPUT(');
        if found1 then
         LinkRes.AddFileName(s1);
        LinkRes.Add(')');
      end;
   end;   
  if linklibc then
   begin
     found2:=librarysearchpath.FindFile('crtn.o',false,s2);
     if found2 then
      begin
        LinkRes.Add('INPUT(');
        if found2 then
         LinkRes.AddFileName(s2);
        LinkRes.Add(')');
      end;
   end;   
   
  with linkres do
    begin
      if apptype=app_arm9 then //ARM9
      begin
        add('MEMORY {');
        add('	ewram	: ORIGIN = 0x02000000, LENGTH = 4M - 512k');
        add('	dtcm	: ORIGIN = 0x0b000000, LENGTH = 16K');
        add('	vectors	: ORIGIN = 0x01000000, LENGTH = 256');
        add('	itcm	: ORIGIN = 0x01000100, LENGTH = 32K - 256');
        add('}');

        add('/*--------------------------------------------------------------------------------');
        add('	This Source Code Form is subject to the terms of the Mozilla Public License,');
        add('	v. 2.0. If a copy of the MPL was not distributed with this file, You can');
        add('	obtain one at https://mozilla.org/MPL/2.0/.');
        add('--------------------------------------------------------------------------------*/');
        add('OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")');
        add('OUTPUT_ARCH(arm)');
        add('ENTRY(_start)');
        add('');
        add('__ewram_end	=	ORIGIN(ewram) + LENGTH(ewram);');
        add('__eheap_end	=	ORIGIN(ewram) + LENGTH(ewram);');
        add('');
        add('__dtcm_top	=	ORIGIN(dtcm) + LENGTH(dtcm);');
        add('__irq_flags	=	__dtcm_top - 0x08;');
        add('__irq_vector	=	__dtcm_top - 0x04;');
        add('');
        add('__sp_svc	=	__dtcm_top - 0x100;');
        add('__sp_irq	=	__sp_svc - 0x100;');
        add('__sp_usr	=	__sp_irq - 0x100;');
        add('');
        add('PHDRS {');
        add('	main    PT_LOAD FLAGS(7);');
        add('	dtcm    PT_LOAD FLAGS(7);');
        add('	itcm    PT_LOAD FLAGS(7);');
        add('	vectors PT_LOAD FLAGS(7);');
        add('	twl     PT_LOAD FLAGS(0x100007);');
        add('}');
        add('');
        add('SECTIONS');
        add('{');
        add('	/* Secure area crap */');
        add('	.secure : { *(.secure) } >ewram :main = 0');
        add('');
        add('	.crt0	:');
        add('	{');
        add('		__text_start = . ;');
        add('		KEEP (*(.crt0))');
        add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :main = 0x00');
        add('');
        add('	.plt : { *(.plt) } >ewram :main = 0xff');
        add('');
        add('	.init :');
        add('	{');
        add('		KEEP (*(SORT_NONE(.init)))');
        add('	} >ewram :main');
        add('');
        add('	.text :   /* ALIGN (4): */');
        add('	{');
        add('		*(EXCLUDE_FILE(*.itcm* *.vectors* *.twl*) .text)');
        add('		*(EXCLUDE_FILE(*.itcm* *.vectors* *.twl*) .stub)');
        add('		*(EXCLUDE_FILE(*.itcm* *.vectors* *.twl*) .text.*)');
        add('		/* .gnu.warning sections are handled specially by elf32.em.  */');
        add('		*(EXCLUDE_FILE(*.twl*) .gnu.warning)');
        add('		*(EXCLUDE_FILE(*.twl*) .gnu.linkonce.t*)');
        add('		*(.glue_7)');
        add('		*(.glue_7t)');
        add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :main = 0xff');
        add('');
        add('	.fini           :');
        add('	{');
        add('		KEEP (*(.fini))');
        add('	} >ewram :main =0xff');
        add('');
        add('	__text_end = . ;');
        add('');
        add('	.rodata :');
        add('	{');
        add('		*(EXCLUDE_FILE(*.twl*) .rodata)');
        add('		*all.rodata*(*)');
        add('		*(EXCLUDE_FILE(*.twl*) .roda)');
        add('		*(EXCLUDE_FILE(*.twl*) .rodata.*)');
        add('		*(EXCLUDE_FILE(*.twl*) .gnu.linkonce.r*)');
        add('		SORT(CONSTRUCTORS)');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :main = 0xff');
        add('');
        add('	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >ewram :main');
        add(' 	__exidx_start = .;');
        add('	ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >ewram :main');
        add(' 	__exidx_end = .;');
        add('');
        add('	/*	Ensure the __preinit_array_start label is properly aligned.  We');
        add('		could instead move the label definition inside the section, but');
        add('		the linker would then create the section even if it turns out to');
        add('		be empty, which isn''t pretty.  */');
        add('');
        add('	. = ALIGN(32 / 8);');
        add('');
        add('	PROVIDE (__preinit_array_start = .);');
        add('	.preinit_array     : { KEEP (*(.preinit_array)) } >ewram :main = 0xff');
        add('	PROVIDE (__preinit_array_end = .);');
        add('	PROVIDE (__init_array_start = .);');
        add('	.init_array     :');
        add('	{');
        add('		KEEP (*(SORT(.init_array.*)))');
        add('		KEEP (*(.init_array))');
        add('	} >ewram :main = 0xff');
        add('	PROVIDE (__init_array_end = .);');
        add('	PROVIDE (__fini_array_start = .);');
        add('	.fini_array     :');
        add('	{');
        add('		KEEP (*(.fini_array))');
        add('		KEEP (*(SORT(.fini_array.*)))');
        add('	} >ewram :main = 0xff');
        add('');
        add('	PROVIDE (__fini_array_end = .);');
        add('');
        add('	.ctors :');
        add('	{');
        add('	/*	gcc uses crtbegin.o to find the start of the constructors, so');
        add('		we make sure it is first.  Because this is a wildcard, it');
        add('		doesn''t matter if the user does not actually link against');
        add('		crtbegin.o; the linker won''t look for a file to match a');
        add('		wildcard.  The wildcard also means that it doesn''t matter which');
        add('		directory crtbegin.o is in.  */');
        add('		KEEP (*crtbegin.o(.ctors))');
        add('		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))');
        add('		KEEP (*(SORT(.ctors.*)))');
        add('		KEEP (*(.ctors))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :main = 0xff');
        add('');
        add('	.dtors :');
        add('	{');
        add('		KEEP (*crtbegin.o(.dtors))');
        add('		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))');
        add('		KEEP (*(SORT(.dtors.*)))');
        add('		KEEP (*(.dtors))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :main = 0xff');
        add('');
        add('	.eh_frame :');
        add('	{');
        add('		KEEP (*(.eh_frame))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :main = 0xff');
        add('');
        add('	.gcc_except_table :');
        add('	{');
        add('		*(.gcc_except_table)');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :main = 0xff');
        add('	.jcr            : { KEEP (*(.jcr)) } >ewram :main = 0');
        add('	.got            : { *(.got.plt) *(.got) *(.rel.got) } >ewram :main = 0');
        add('');
        add('	.ewram ALIGN(4) :'); 
        add('	{');
        add('		__ewram_start = ABSOLUTE(.) ;');
        add('		*(.ewram)');
        add('		*ewram.*(.text)');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :main = 0xff');
        add('');
        add('');
        add('	.data ALIGN(4) :');
        add('	{');
        add('		__data_start = ABSOLUTE(.);');
        add('		*(EXCLUDE_FILE(*.twl*) .data)');
        add('		*(EXCLUDE_FILE(*.twl*) .data.*)');
        add('		*(EXCLUDE_FILE(*.twl*) .gnu.linkonce.d*)');
        add('		CONSTRUCTORS');
        add('		. = ALIGN(4);');
        add('		__data_end = ABSOLUTE(.) ;');
        add('	} >ewram :main = 0xff');
        add('');
        add('	__bss_vma = . ;');
        add('');
        add('	.dtcm :');
        add('	{');
        add('		__dtcm_lma = LOADADDR(.dtcm);');
        add('		__dtcm_start = ABSOLUTE(.);');
        add('		*(.dtcm)');
        add('		*(.dtcm.*)');
        add('		. = ALIGN(4);');
        add('		__dtcm_end = ABSOLUTE(.);');
        add('	} >dtcm AT>ewram :dtcm = 0xff');
        add('');
        add('	.itcm :');
        add('	{');
        add('		__itcm_lma = LOADADDR(.itcm);');
        add('		__itcm_start = ABSOLUTE(.);');
        add('		*(.itcm)');
        add('		*.itcm*(.text .stub .text.*)');
        add('		. = ALIGN(4);');
        add('		__itcm_end = ABSOLUTE(.);');
        add('	} >itcm AT>ewram :itcm = 0xff');
        add('');
        add('	.vectors :');
        add('	{');
        add('		__vectors_lma = LOADADDR(.vectors);');
        add('		__vectors_start = ABSOLUTE(.);');
        add('		KEEP(*(.vectors .vectors.*))');
        add('		. = ALIGN(4);');
        add('		__vectors_end = ABSOLUTE(.);');
        add('	} >vectors AT>ewram :vectors = 0xff');
        add('	');
        add('	.sbss __dtcm_end (NOLOAD):'); 
        add('	{');
        add('		__sbss_start = ABSOLUTE(.);');
        add('		__sbss_start__ = ABSOLUTE(.);');
        add('		*(.sbss)');
        add('		. = ALIGN(4);    /* REQUIRED. LD is flaky without it. */');
        add('		__sbss_end = ABSOLUTE(.);');
        add('	} >dtcm :NONE');
        add('');
        add('	.bss __bss_vma (NOLOAD):'); 
        add('	{');
        add('		__bss_start = ABSOLUTE(.);');
        add('		__bss_start__ = ABSOLUTE(.);');
        add('		*(EXCLUDE_FILE(*.twl*) .dynbss)');
        add('		*(EXCLUDE_FILE(*.twl*) .gnu.linkonce.b*)');
        add('		*(EXCLUDE_FILE(*.twl*) .bss*)');
        add('		*(EXCLUDE_FILE(*.twl*) COMMON)');
        add('		. = ALIGN(4);    /* REQUIRED. LD is flaky without it. */');
        add('		__bss_end__ = ABSOLUTE(.) ;');
        add('		__end__ = ABSOLUTE(.) ;');
        add('	} >ewram :NONE');
        add('');
        add('	.twl __end__ : AT(MAX(0x2400000,MAX(__end__,LOADADDR(.vectors)+SIZEOF(.vectors))))');
        add('	{');
        add('		__arm9i_lma__ = LOADADDR(.twl);');
        add('		__arm9i_start__ = ABSOLUTE(.);');
        add('		*(.twl)');
        add('		*.twl*(.text .stub .text.* .gnu.linkonce.t.*)');
        add('		*.twl*(.rodata)');
        add('		*.twl*(.roda)');
        add('		*.twl*(.rodata.*)');
        add('		*.twl*(.data)');
        add('		*.twl*(.data.*)');
        add('		*.twl*(.gnu.linkonce.d*)');
        add('		__arm9i_end__ = ABSOLUTE(.);');
        add('	} :twl');
        add('');
        add('	.twl_bss __arm9i_end__ (NOLOAD):');
        add('	{');
        add('		__twl_bss_start__ = ABSOLUTE(.);');
        add('		*(.twl_bss)');
        add('		*.twl*(.dynbss)');
        add('		*.twl*(.gnu.linkonce.b*)');
        add('		*.twl*(.bss*)');
        add('		*.twl*(COMMON)');
        add('		. = ALIGN(4);    /* REQUIRED. LD is flaky without it. */');
        add('		__twl_bss_end__ = ABSOLUTE(.);');
        add('		__twl_end__ = ABSOLUTE(.);');
        add('	} :NONE');
        add('');
        add('	/* Stabs debugging sections.  */');
        add('	.stab 0 : { *(.stab) }');
        add('	.stabstr 0 : { *(.stabstr) }');
        add('	.stab.excl 0 : { *(.stab.excl) }');
        add('	.stab.exclstr 0 : { *(.stab.exclstr) }');
        add('	.stab.index 0 : { *(.stab.index) }');
        add('	.stab.indexstr 0 : { *(.stab.indexstr) }');
        add('	.comment 0 : { *(.comment) }');
        add('	/*	DWARF debug sections.');
        add('		Symbols in the DWARF debugging sections are relative to the beginning');
        add('		of the section so we begin them at 0.  */');
        add('	/* DWARF 1 */');
        add('	.debug          0 : { *(.debug) }');
        add('	.line           0 : { *(.line) }');
        add('	/* GNU DWARF 1 extensions */');
        add('	.debug_srcinfo  0 : { *(.debug_srcinfo) }');
        add('	.debug_sfnames  0 : { *(.debug_sfnames) }');
        add('	/* DWARF 1.1 and DWARF 2 */');
        add('	.debug_aranges  0 : { *(.debug_aranges) }');
        add('	.debug_pubnames 0 : { *(.debug_pubnames) }');
        add('	/* DWARF 2 */');
        add('	.debug_info     0 : { *(.debug_info) }');
        add('	.debug_abbrev   0 : { *(.debug_abbrev) }');
        add('	.debug_line     0 : { *(.debug_line) }');
        add('	.debug_frame    0 : { *(.debug_frame) }');
        add('	.debug_str      0 : { *(.debug_str) }');
        add('	.debug_loc      0 : { *(.debug_loc) }');
        add('	.debug_macinfo  0 : { *(.debug_macinfo) }');
        add('	/* SGI/MIPS DWARF 2 extensions */');
        add('	.debug_weaknames 0 : { *(.debug_weaknames) }');
        add('	.debug_funcnames 0 : { *(.debug_funcnames) }');
        add('	.debug_typenames 0 : { *(.debug_typenames) }');
        add('	.debug_varnames  0 : { *(.debug_varnames) }');
        add('	.stack 0x80000 : { _stack = .; *(.stack) }');
        add('	/* These must appear regardless of  .  */');
        add('}');
      end;
      if apptype=app_arm7 then
      begin
        add('/*--------------------------------------------------------------------------------');
        add('	This Source Code Form is subject to the terms of the Mozilla Public License,');
        add('	v. 2.0. If a copy of the MPL was not distributed with this file, You can');
        add('	obtain one at https://mozilla.org/MPL/2.0/.');
        add('--------------------------------------------------------------------------------*/');
        add('OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")');
        add('OUTPUT_ARCH(arm)');
        add('ENTRY(_start)');
        add('');
        add('');
        add('PHDRS {');
        add('	crt0  PT_LOAD FLAGS(7);');
        add('	arm7  PT_LOAD FLAGS(7);');
        add('	arm7i PT_LOAD FLAGS(0x100007);');
        add('}');
        add('');
        add('');
        add('MEMORY {');
        add('	ewram  : ORIGIN = 0x02380000, LENGTH = 12M - 512K');
        add('	rom    : ORIGIN = 0x08000000, LENGTH = 32M');
        add('	iwram  : ORIGIN = 0x037f8000, LENGTH = 96K');
        add('');
        add('	twl_ewram : ORIGIN = 0x02e80000, LENGTH = 512K - 64K');
        add('	twl_iwram : ORIGIN = 0x03000000, LENGTH = 256K');
        add('}');
        add('');
        add('__iwram_start	=	ORIGIN(iwram);');
        add('__iwram_top	=	ORIGIN(iwram)+ LENGTH(iwram);');
        add('');
        add('__sp_irq	=	__iwram_top - 0x100;');
        add('__sp_svc	=	__sp_irq - 0x100;');
        add('__sp_usr	=	__sp_svc - 0x100;');
        add('');
        add('__irq_flags	=	0x04000000 - 8;');
        add('__irq_flagsaux	=	0x04000000 - 0x40;');
        add('__irq_vector	=	0x04000000 - 4;');
        add('');
        add('SECTIONS');
        add('{');
        add('');
        add('	.twl :');
        add('	{');
        add('		__arm7i_lma__ = LOADADDR(.twl);');
        add('		__arm7i_start__ = .;');
        add('		*(.twl)');
        add('		*.twl*(.text .stub .text.* .gnu.linkonce.t.*)');
        add('		*.twl*(.rodata)');
        add('		*.twl*(.roda)');
        add('		*.twl*(.rodata.*)');
        add('		*.twl*(.data)');
        add('		*.twl*(.data.*)');
        add('		*.twl*(.gnu.linkonce.d*)');
        add('		. = ALIGN(4);');
        add('		__arm7i_end__ = .;');
        add('	} >twl_iwram AT>twl_ewram :arm7i');
        add('');
        add('	.twl_bss ALIGN(4) (NOLOAD) :');
        add('	{');
        add('		__twl_bss_start__ = .;');
        add('		*(.twl_bss)');
        add('		*.twl.*(.dynbss)');
        add('		*.twl.*(.gnu.linkonce.b*)');
        add('		*.twl.*(.bss*)');
        add('		*.twl.*(COMMON)');
        add('		. = ALIGN(4);');
        add('		__twl_bss_end__ = .;');
        add('	} >twl_iwram :NONE');
        add('');
        add('	.crt0	:');
        add('	{');
        add('		KEEP (*(.crt0))');
        add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram :crt0');
        add('');
        add('	.text :');
        add('	{');
        add('		__arm7_lma__ = LOADADDR(.text);');
        add('		__arm7_start__ = .;');
        add('		KEEP (*(SORT_NONE(.init)))');
        add('		*(.plt)');
        add('		*(.text .stub .text.* .gnu.linkonce.t.*)');
        add('		KEEP (*(.text.*personality*))');
        add('		/* .gnu.warning sections are handled specially by elf32.em.  */');
        add('		*(.gnu.warning)');
        add('		*(.glue_7t) *(.glue_7) *(.vfp11_veneer)');
        add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram AT>ewram :arm7');
        add('');
        add('	.fini           :');
        add('	{');
        add('		KEEP (*(.fini))');
        add('	} >iwram AT>ewram');
        add('');
        add('	.rodata :');
        add('	{');
        add('		*(.rodata)');
        add('		*all.rodata*(*)');
        add('		*(.roda)');
        add('		*(.rodata.*)');
        add('		*(.gnu.linkonce.r*)');
        add('		SORT(CONSTRUCTORS)');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram AT>ewram');
        add('');
        add('	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >iwram AT>ewram');
        add('');
        add('	.ARM.exidx   : {');
        add('		__exidx_start = .;');
        add('		*(.ARM.exidx* .gnu.linkonce.armexidx.*)');
        add('		__exidx_end = .;');
        add('	 } >iwram AT>ewram');
        add('');
        add('/* Ensure the __preinit_array_start label is properly aligned.  We');
        add('   could instead move the label definition inside the section, but');
        add('   the linker would then create the section even if it turns out to');
        add('   be empty, which isn''t pretty.  */');
        add('	.preinit_array     : {');
        add('		. = ALIGN(32 / 8);');
        add('		PROVIDE (__preinit_array_start = .);');
        add('		KEEP (*(.preinit_array))');
        add('		PROVIDE (__preinit_array_end = .);');
        add('	} >iwram AT>ewram');
        add('');
        add('	.init_array     : {');
        add('		PROVIDE (__init_array_start = .);');
        add('		KEEP (*(.init_array))');
        add('		PROVIDE (__init_array_end = .);');
        add('	} >iwram AT>ewram');
        add('');
        add('	.fini_array     : {');
        add('		PROVIDE (__fini_array_start = .);');
        add('		KEEP (*(.fini_array))');
        add('		PROVIDE (__fini_array_end = .);');
        add('	} >iwram AT>ewram');
        add('');
        add('	.ctors :');
        add('	{');
        add('	/* gcc uses crtbegin.o to find the start of the constructors, so');
        add('		we make sure it is first.  Because this is a wildcard, it');
        add('		doesn''t matter if the user does not actually link against');
        add('		crtbegin.o; the linker won''t look for a file to match a');
        add('		wildcard.  The wildcard also means that it doesn''t matter which');
        add('		directory crtbegin.o is in.  */');
        add('		KEEP (*crtbegin.o(.ctors))');
        add('		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))');
        add('		KEEP (*(SORT(.ctors.*)))');
        add('		KEEP (*(.ctors))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram AT>ewram');
        add('');
        add('	.dtors :');
        add('	{');
        add('		KEEP (*crtbegin.o(.dtors))');
        add('		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))');
        add('		KEEP (*(SORT(.dtors.*)))');
        add('		KEEP (*(.dtors))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram AT>ewram');
        add('');
        add('	.eh_frame :');
        add('	{');
        add('		KEEP (*(.eh_frame))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram AT>ewram');
        add('');
        add('	.gcc_except_table :');
        add('	{');
        add('		*(.gcc_except_table)');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram AT>ewram');
        add('	.jcr            : { KEEP (*(.jcr)) } >iwram AT>ewram');
        add('	.got            : { *(.got.plt) *(.got) } >iwram AT>ewram');
        add('');
        add('	.data ALIGN(4) : 	{');
        add('		__data_start = ABSOLUTE(.);');
        add('		*(.data)');
        add('		*(.data.*)');
        add('		*(.gnu.linkonce.d*)');
        add('		CONSTRUCTORS');
        add('		. = ALIGN(4);');
        add('		__data_end = ABSOLUTE(.) ;');
        add('	} >iwram AT>ewram');
        add('');
        add('	.bss ALIGN(4) (NOLOAD) :');
        add('	{');
        add('		__arm7_end__ = .;');
        add('		__bss_start = ABSOLUTE(.);');
        add('		__bss_start__ = ABSOLUTE(.);');
        add('		*(.dynbss)');
        add('		*(.gnu.linkonce.b*)');
        add('		*(.bss*)');
        add('		*(COMMON)');
        add('		. = ALIGN(4);    /* REQUIRED. LD is flaky without it. */');
        add('		__bss_end__ = ABSOLUTE(.);');
        add('		__end__ = ABSOLUTE(.);');
        add('	} >iwram');
        add('');
        add('	/* Stabs debugging sections.  */');
        add('	.stab 0 : { *(.stab) }');
        add('	.stabstr 0 : { *(.stabstr) }');
        add('	.stab.excl 0 : { *(.stab.excl) }');
        add('	.stab.exclstr 0 : { *(.stab.exclstr) }');
        add('	.stab.index 0 : { *(.stab.index) }');
        add('	.stab.indexstr 0 : { *(.stab.indexstr) }');
        add('	.comment 0 : { *(.comment) }');
        add('	/*	DWARF debug sections.');
        add('		Symbols in the DWARF debugging sections are relative to the beginning');
        add('		of the section so we begin them at 0.  */');
        add('	/* DWARF 1 */');
        add('	.debug          0 : { *(.debug) }');
        add('	.line           0 : { *(.line) }');
        add('	/* GNU DWARF 1 extensions */');
        add('	.debug_srcinfo  0 : { *(.debug_srcinfo) }');
        add('	.debug_sfnames  0 : { *(.debug_sfnames) }');
        add('	/* DWARF 1.1 and DWARF 2 */');
        add('	.debug_aranges  0 : { *(.debug_aranges) }');
        add('	.debug_pubnames 0 : { *(.debug_pubnames) }');
        add('	/* DWARF 2 */');
        add('	.debug_info     0 : { *(.debug_info) }');
        add('	.debug_abbrev   0 : { *(.debug_abbrev) }');
        add('	.debug_line     0 : { *(.debug_line) }');
        add('	.debug_frame    0 : { *(.debug_frame) }');
        add('	.debug_str      0 : { *(.debug_str) }');
        add('	.debug_loc      0 : { *(.debug_loc) }');
        add('	.debug_macinfo  0 : { *(.debug_macinfo) }');
        add('	/* SGI/MIPS DWARF 2 extensions */');
        add('	.debug_weaknames 0 : { *(.debug_weaknames) }');
        add('	.debug_funcnames 0 : { *(.debug_funcnames) }');
        add('	.debug_typenames 0 : { *(.debug_typenames) }');
        add('	.debug_varnames  0 : { *(.debug_varnames) }');
        add('	/* These must appear regardless of  .  */');
        add('}');
      end;
    end;

{ Write and Close response }
  linkres.writetodisk;
  linkres.free;

  WriteResponseFile:=True;

end;


function TLinkerNDS.MakeExecutable:boolean;
var
  binstr,
  cmdstr  : TCmdStr;
  success : boolean;
  StaticStr,
  GCSectionsStr,
  DynLinkStr,
  MapStr,
  StripStr: string;
  preName: string;
begin
  { for future use }
  StaticStr:='';
  StripStr:='';
  MapStr:='';
  DynLinkStr:='';
  GCSectionsStr:='';
  preName:='';
  case apptype of
   app_arm9: preName:='.nef';
   app_arm7: preName:='.nlf';
   else
     internalerror(2019050934);
  end;

  if (cs_link_strip in current_settings.globalswitches) and
     not(cs_link_separate_dbg_file in current_settings.globalswitches) then
   StripStr:='-s';
  if (cs_link_map in current_settings.globalswitches) then
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));
  if create_smartlink_sections then
   GCSectionsStr:='--gc-sections';
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

{ Write used files and libraries }
  WriteResponseFile();

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$OPT',Info.ExtraOptions);

  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename,preName)))));
  Replace(cmdstr,'$RES',(maybequoted(ScriptFixFileName(outputexedir+Info.ResName))));
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$MAP',MapStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);
  
  success:=DoExec(FindUtil(utilsprefix+BinStr),cmdstr,true,false);

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   DeleteFile(outputexedir+Info.ResName);

{ Post process }
  if success then
    begin
      success:=DoExec(FindUtil(utilsprefix + 'objcopy'), '-O binary '+ 
        ChangeFileExt(current_module.exefilename, preName) + ' ' + 
        ChangeFileExt(current_module.exefilename, preName+target_info.exeext),
        true,false);
    end;

  if success and (apptype=app_arm9) then 
    begin
      success:=DoExec(FindUtil('ndstool'), '-c ' + 
        ChangeFileExt(current_module.exefilename, '.nds') + ' -9 ' + 
        ChangeFileExt(current_module.exefilename, preName+target_info.exeext),
        true,false);
    end;
  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_nds,TLinkerNDS);
  RegisterTarget(system_arm_nds_info);
end.
