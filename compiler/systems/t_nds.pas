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
       globtype,globals,systems,verbose,script,fmodule,i_nds,link;

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
  if (apptype <> app_arm9) or (apptype <> app_arm7) then
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
        add('	rom	: ORIGIN = 0x08000000, LENGTH = 32M');
        add('	ewram	: ORIGIN = 0x02000000, LENGTH = 4M - 4k');
        add('	dtcm	: ORIGIN = 0x0b000000, LENGTH = 16K');
        add('	vectors	: ORIGIN = 0x01000000, LENGTH = 256');
        add('	itcm	: ORIGIN = 0x01000100, LENGTH = 32K - 256');
        add('}');
        add('');
        add('OUTPUT_ARCH(arm)');
        add('OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")');
        add('ENTRY(_start)');
        add('');
        add('__vectors_start	=	ORIGIN(vectors);');
        add('__itcm_start	=	ORIGIN(itcm);');
        add('__ewram_end	=	ORIGIN(ewram) + LENGTH(ewram);');
        add('__eheap_end	=	ORIGIN(ewram) + LENGTH(ewram);');
        add('__dtcm_start	=	ORIGIN(dtcm);');
        add('__dtcm_top	=	ORIGIN(dtcm) + LENGTH(dtcm);');
        add('__irq_flags	=	__dtcm_top - 0x08;');
        add('__irq_vector	=	__dtcm_top - 0x04;');
        add('');
        add('__sp_svc	=	__dtcm_top - 0x100;');
        add('__sp_irq	=	__sp_svc - 0x100;');
        add('__sp_usr	=	__sp_irq - 0x100;');
        add('');
        add('SECTIONS');
        add('{');
        add('	.init	:');
        add('	{');
        add('		__text_start = . ;');
        add('		KEEP (*(.init))');
        add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram = 0xff');
        add('');
        add('	.plt : { *(.plt) } >ewram = 0xff');
        add('');
        add('	.text :   /* ALIGN (4): */');
        add('	{');
        add('		*(EXCLUDE_FILE (*.itcm*) .text)');
        add('');
        add('		*(.text.*)');
        add('		*(.stub)');
        add('		/* .gnu.warning sections are handled specially by elf32.em.  */');
        add('		*(.gnu.warning)');
        add('		*(.gnu.linkonce.t*)');
        add('		*(.glue_7)');
        add('		*(.glue_7t)');
        add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram = 0xff');
        add('');
        add('	.fini           :');
        add('	{');
        add('		KEEP (*(.fini))');
        add('	} >ewram =0xff');
        add('');
        add('	__text_end = . ;');
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
        add('	} >ewram = 0xff');
        add('');
<<<<<<< HEAD
        add('	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >ewram');
        add(' 	__exidx_start = .;');
        add('	ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >ewram');
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
        add('	.preinit_array     : { KEEP (*(.preinit_array)) } >ewram = 0xff');
        add('	PROVIDE (__preinit_array_end = .);');
        add('	PROVIDE (__init_array_start = .);');
        add('	.init_array     :');
        add('	{');
        add('		KEEP (*(SORT(.init_array.*)))');
        add('		KEEP (*(.init_array))');
        add('	} >ewram = 0xff');
        add('	PROVIDE (__init_array_end = .);');
        add('	PROVIDE (__fini_array_start = .);');
        add('	.fini_array     :');
        add('	{');
        add('		KEEP (*(.fini_array))');
        add('		KEEP (*(SORT(.fini_array.*)))');
        add('	} >ewram = 0xff');
        add('');
        add('	PROVIDE (__fini_array_end = .);');
=======
        add('  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >ewram');
        add('   __exidx_start = .;');
        add('  .ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >ewram');
        add('   __exidx_end = .;');
        add('  /* Ensure the __preinit_array_start label is properly aligned.  We');
        add('     could instead move the label definition inside the section, but');
        add('     the linker would then create the section even if it turns out to');
        add('     be empty, which isn''t pretty.  */');
        add('  . = ALIGN(32 / 8);');
        add('  PROVIDE (__preinit_array_start = .);');
        add('  .preinit_array     : { KEEP (*(.preinit_array)) } >ewram = 0xff');
        add('  PROVIDE (__preinit_array_end = .);');
        add('  PROVIDE (__init_array_start = .);');
        add('  .init_array     :');
        add('  {');
        add('       KEEP (*(SORT(.init_array.*)))');
        add('       KEEP (*(.init_array))');
        add('  } >ewram = 0xff');        
        add('  PROVIDE (__init_array_end = .);');
        add('  PROVIDE (__fini_array_start = .);');
        add('  .fini_array     :');
        add('  {');
        add('       KEEP (*(.fini_array))');
        add('       KEEP (*(SORT(.fini_array.*)))');
        add('  } >ewram = 0xff');
        add('  PROVIDE (__fini_array_end = .);');
>>>>>>> graemeg/cpstrnew
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
        add('	} >ewram = 0xff');
        add('');
        add('	.dtors :');
        add('	{');
        add('		KEEP (*crtbegin.o(.dtors))');
        add('		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))');
        add('		KEEP (*(SORT(.dtors.*)))');
        add('		KEEP (*(.dtors))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram = 0xff');
        add('');
        add('	.eh_frame :');
        add('	{');
        add('		KEEP (*(.eh_frame))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram = 0xff');
        add('');
        add('	.gcc_except_table :');
        add('	{');
        add('		*(.gcc_except_table)');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram = 0xff');
        add('	.jcr            : { KEEP (*(.jcr)) } >ewram = 0');
        add('	.got            : { *(.got.plt) *(.got) *(.rel.got) } >ewram = 0');
        add('');
        add('	.ewram ALIGN(4) : ');
        add('	{');
        add('		__ewram_start = ABSOLUTE(.) ;');
        add('		*(.ewram)');
        add('		*ewram.*(.text)');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >ewram = 0xff');
        add('');
        add('');
        add('	.data ALIGN(4) :');
        add('	{');
        add('		__data_start = ABSOLUTE(.);');
        add('		*(.data)');
        add('		*(.data.*)');
        add('		*(.gnu.linkonce.d*)');
        add('		CONSTRUCTORS');
        add('		. = ALIGN(4);');
        add('		__data_end = ABSOLUTE(.) ;');
        add('	} >ewram = 0xff');
        add('');
        add('');
        add('	__dtcm_lma = . ;');
        add('	__bss_vma = . ;');
        add('');
        add('	.dtcm __dtcm_start : AT (__dtcm_lma)');
        add('	{');
        add('		*(.dtcm)');
        add('		*(.dtcm.*)');
        add('		. = ALIGN(4);');
        add('		__dtcm_end = ABSOLUTE(.);');
        add('	} >dtcm  = 0xff');
        add('');
        add('');
        add('	__itcm_lma = __dtcm_lma + SIZEOF(.dtcm);');
        add('');
        add('	.itcm __itcm_start : AT (__itcm_lma)');
        add('	{');
        add('		*(.itcm)');
        add('		*itcm.*(.text)');
        add('		. = ALIGN(4);');
        add('		__itcm_end = ABSOLUTE(.);');
        add('	} >itcm = 0xff');
        add('	');
        add('	__vectors_lma = __itcm_lma + SIZEOF(.itcm);');
        add('');
        add('	.vectors __vectors_start : AT (__vectors_lma)');
        add('	{');
        add('		*(.vectors)');
        add('		*vectors.*(.text)');
        add('		. = ALIGN(4);');
        add('		__vectors_end = ABSOLUTE(.);');
        add('	} >vectors = 0xff');
        add('	');
        add('	.sbss __dtcm_end (NOLOAD): ');
        add('	{');
        add('		__sbss_start = ABSOLUTE(.);');
        add('		__sbss_start__ = ABSOLUTE(.);');
        add('		*(.sbss)');
        add('		. = ALIGN(4);    /* REQUIRED. LD is flaky without it. */');
        add('		__sbss_end = ABSOLUTE(.);');
        add('	} >dtcm ');
        add('');
        add('	.bss __bss_vma (NOLOAD): ');
        add('	{');
        add('		__bss_start = ABSOLUTE(.);');
        add('		__bss_start__ = ABSOLUTE(.);');
        add('		*(.dynbss)');
        add('		*(.gnu.linkonce.b*)');
        add('		*(.bss*)');
        add('		*(COMMON)');
        add('		. = ALIGN(4);    /* REQUIRED. LD is flaky without it. */');
        add('		__bss_end__ = ABSOLUTE(.) ;');
        add('		__end__ = ABSOLUTE(.) ;');
        add('	} AT>ewram ');
        add('');
        add('');
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
        add('');
      end;
      if apptype=app_arm7 then
      begin
        add('OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")');
        add('OUTPUT_ARCH(arm)');
        add('ENTRY(_start)');
        add('');
        add('MEMORY {');
        add('');
        add('	rom	: ORIGIN = 0x08000000, LENGTH = 32M');
        add('	iwram	: ORIGIN = 0x037f8000, LENGTH = 96K	');
        add('}');
        add('');
        add('__iwram_start	=	ORIGIN(iwram);');
        add('__iwram_top	=	ORIGIN(iwram)+ LENGTH(iwram);');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        add('');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
        add('	.init	:');
        add('	{');
        add('		__text_start = . ;');
        add('		KEEP (*(.init))');
        add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
        add('		} >iwram = 0xff');
        add('	.plt : { *(.plt) } >iwram = 0xff');
        add('');
        add('	.text :   /* ALIGN (4): */');
        add('	{');
        add('		*(.text .stub .text.* .gnu.linkonce.t.*)');
        add('		KEEP (*(.text.*personality*))');
        add('		/* .gnu.warning sections are handled specially by elf32.em.  */');
        add('		*(.gnu.warning)');
        add('		*(.glue_7t) *(.glue_7) *(.vfp11_veneer)');
        add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram = 0xff');
        add('');
        add('	.fini           :');
        add('	{');
        add('		KEEP (*(.fini))');
        add('	} >iwram =0xff');
        add('');
        add('	__text_end = . ;');
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
        add('	} >iwram = 0xff');
        add('');
        add('	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >iwram');
        add('	__exidx_start = .;');
        add('	.ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >iwram');
        add('	__exidx_end = .;');
        add('');
        add('/* Ensure the __preinit_array_start label is properly aligned.  We');
        add('   could instead move the label definition inside the section, but');
        add('   the linker would then create the section even if it turns out to');
        add('   be empty, which isn''t pretty.  */');
        add('	. = ALIGN(32 / 8);');
        add('	PROVIDE (__preinit_array_start = .);');
        add('	.preinit_array     : { KEEP (*(.preinit_array)) } >iwram = 0xff');
        add('	PROVIDE (__preinit_array_end = .);');
        add('	PROVIDE (__init_array_start = .);');
        add('	.init_array     : { KEEP (*(.init_array)) } >iwram = 0xff');
        add('	PROVIDE (__init_array_end = .);');
        add('	PROVIDE (__fini_array_start = .);');
        add('	.fini_array     : { KEEP (*(.fini_array)) } >iwram = 0xff');
        add('	PROVIDE (__fini_array_end = .);');
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
        add('	} >iwram = 0xff');
        add('');
        add('	.dtors :');
        add('	{');
        add('		KEEP (*crtbegin.o(.dtors))');
        add('		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))');
        add('		KEEP (*(SORT(.dtors.*)))');
        add('		KEEP (*(.dtors))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram = 0xff');
        add('');
        add('	.eh_frame :');
        add('	{');
        add('		KEEP (*(.eh_frame))');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram = 0xff');
        add('');
        add('	.gcc_except_table :');
        add('	{');
        add('		*(.gcc_except_table)');
        add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
        add('	} >iwram = 0xff');
        add('	.jcr            : { KEEP (*(.jcr)) } >iwram = 0');
        add('	.got            : { *(.got.plt) *(.got) } >iwram = 0');
        add('');
        add('	.data ALIGN(4) : 	{');
        add('		__data_start = ABSOLUTE(.);');
        add('		*(.data)');
        add('		*(.data.*)');
        add('		*(.gnu.linkonce.d*)');
        add('		CONSTRUCTORS');
        add('		. = ALIGN(4);');
        add('		__data_end = ABSOLUTE(.) ;');
        add('	} >iwram = 0xff');
        add('');
        add('	.bss ALIGN(4) :');
        add('	{');
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
        add('	.stack 0x80000 : { _stack = .; *(.stack) }');
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
  end;

  if (cs_link_strip in current_settings.globalswitches) and
     not(cs_link_separate_dbg_file in current_settings.globalswitches) then
   StripStr:='-s';
  if (cs_link_map in current_settings.globalswitches) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));
=======
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename^,'.map'));
>>>>>>> graemeg/cpstrnew
=======
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename^,'.map'));
>>>>>>> graemeg/cpstrnew
=======
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename^,'.map'));
>>>>>>> graemeg/cpstrnew
=======
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename^,'.map'));
>>>>>>> origin/cpstrnew
=======
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename^,'.map'));
>>>>>>> origin/fixes_2.4
  if create_smartlink_sections then
   GCSectionsStr:='--gc-sections';
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

{ Write used files and libraries }
  WriteResponseFile();

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$OPT',Info.ExtraOptions);

<<<<<<< HEAD
<<<<<<< HEAD
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename,preName)))));
=======
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename^,preName)))));
>>>>>>> graemeg/fixes_2_2
=======
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename^,preName)))));
>>>>>>> origin/fixes_2_2
  Replace(cmdstr,'$RES',(maybequoted(ScriptFixFileName(outputexedir+Info.ResName))));
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Replace(cmdstr,'$MAP',MapStr);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  Replace(cmdstr,'$MAP',MapStr);
>>>>>>> origin/fixes_2.4
  Replace(cmdstr,'$DYNLINK',DynLinkStr);
  
  success:=DoExec(FindUtil(utilsprefix+BinStr),cmdstr,true,false);

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   DeleteFile(outputexedir+Info.ResName);

{ Post process }
  if success then
    begin
      success:=DoExec(FindUtil(utilsprefix + 'objcopy'), '-O binary '+ 
<<<<<<< HEAD
<<<<<<< HEAD
        ChangeFileExt(current_module.exefilename, preName) + ' ' + 
        ChangeFileExt(current_module.exefilename, preName+target_info.exeext),
=======
=======
>>>>>>> origin/fixes_2_2
        ChangeFileExt(current_module.exefilename^, preName) + ' ' + 
        ChangeFileExt(current_module.exefilename^, preName+target_info.exeext),
>>>>>>> graemeg/fixes_2_2
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
