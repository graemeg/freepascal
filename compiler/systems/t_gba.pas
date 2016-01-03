{
    This unit implements support import,export,link routines
    for the (arm) GameBoy Advance target

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
unit t_gba;

{$i fpcdefs.inc}

interface


implementation

    uses
       aasmbase,
       SysUtils,
       cutils,cfileutl,cclasses,
       globtype,globals,systems,verbose,script,fmodule,i_gba,link;

    type
       TlinkerGBA=class(texternallinker)
       private
          Function  WriteResponseFile: Boolean;
       public
          constructor Create; override;
          procedure SetDefaultInfo; override;
          function  MakeExecutable:boolean; override;
       end;



{*****************************************************************************
                                  TLINKERGBA
*****************************************************************************}

Constructor TLinkerGba.Create;
begin
  Inherited Create;
  SharedLibFiles.doubles:=true;
  StaticLibFiles.doubles:=true;
end;


procedure TLinkerGba.SetDefaultInfo;
begin
  with Info do
   begin
     ExeCmd[1]:='ld -g $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP -L. -o $EXE -T $RES';
   end;
end;


Function TLinkerGba.WriteResponseFile: Boolean;
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
  prtobj:='prt0';
  cprtobj:='cprt0';
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
  //s:=FindObjectFile('prt0','',false);
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
      add('/* Linker Script Original v1.3 by Jeff Frohwein     */');
      add('/*  v1.0 - Original release                         */');
      add('/*  v1.1 - Added proper .data section support       */');
      add('/*  v1.2 - Added support for c++ & iwram overlays   */');
      add('/*       - Major contributions by Jason Wilkins.    */');
      add('/*  v1.3 - .ewram section now can be used when      */');
      add('/*         compiling for MULTIBOOT mode. This fixes */');
      add('/*         malloc() in DevKitAdvance which depends  */');
      add('/*         on __eheap_start instead of end to define*/');
      add('/*         the starting location of heap space.     */');
      add('/*         External global variable __gba_iwram_heap*/');
      add('/*         support added to allow labels end, _end, */');
      add('/*         & __end__ to point to end of iwram or    */');
      add('/*         the end of ewram.                        */');
      add('/*	Additions by WinterMute				*/');
      add('/* v1.4 -	.sbss section added for unitialised	*/');
      add('/*		    data in ewram 			*/');
      add('/* v1.5 -	padding section added to stop EZF 	*/');
      add('/*		    stripping important data		*/');
      add('');
      add('/* This file is released into the public domain		*/');
      add('/* for commercial or non-commercial use with no		*/');
      add('/* restrictions placed upon it.				*/');
      add('');
      add('/* NOTE!!!: This linker script defines the RAM &  */');
      add('/*   ROM start addresses. In order for it to work */');
      add('/*   properly, remove -Ttext and -Tbss linker     */');
      add('/*   options from your makefile if they are       */');
      add('/*   present.                                     */');
      add('');
      add('/* You can use the following to view section      */');
      add('/* addresses in your .elf file:                   */');
      add('/*   objdump -h file.elf                          */');
      add('/* Please note that empty sections may incorrectly*/');
      add('/* list the lma address as the vma address for    */');
      add('/* some versions of objdump.                      */');
      add('');
      add('OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")');
      add('OUTPUT_ARCH(arm)');
      add('ENTRY(_start)');
      add('/* SEARCH_DIR(/bin/arm); */');
      add('');
      add('/* The linker script function "var1 += var2;" sometimes    */');
      add('/* reports incorrect values in the *.map file but the      */');
      add('/* actual value it calculates is usually, if not always,   */');
      add('/* correct. If you leave out the ". = ALIGN(4);" at the    */');
      add('/* end of each section then the return value of SIZEOF()   */');
      add('/* is sometimes incorrect and "var1 += var2;" appears to   */');
      add('/* not work as well. "var1 += var2" style functions are    */');
      add('/* avoided below as a result.                              */');
      add('');
      add('MEMORY {');
      add('');
      add('	rom	: ORIGIN = 0x08000000, LENGTH = 32M');
      add('	iwram	: ORIGIN = 0x03000000, LENGTH = 32K');
      add('	ewram	: ORIGIN = 0x02000000, LENGTH = 256K');
      add('}');
      add('');
      add('__text_start	=	ORIGIN(rom);');
      add('__eheap_end	=	ORIGIN(ewram) + LENGTH(ewram);');
      add('__iwram_start	=	ORIGIN(iwram);');
      add('__iwram_top	=	ORIGIN(iwram) + LENGTH(iwram);;');
      add('__sp_irq	=	__iwram_top - 0x060;');
      add('__sp_usr	=	__sp_irq - 0x0a0;');
      add('__irq_flags	=	0x03007ff8;');
      add('');
      add('SECTIONS');
      add('{');
      add('	. = __text_start;');
      add('	.init :');
      add('	{');
      add('		KEEP (*(.init))');
      add('		. = ALIGN(4);');
      add('	} >rom =0xff');
      add('');
      add('	.plt :');
      add('	{');
      add('		*(.plt)');
      add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
      add('	} >rom');
      add('');
      add('	.text  :   /* ALIGN (4): */');
      add('	{');
      add('		*(EXCLUDE_FILE (*.iwram*) .text)');
      add('		*(.text .stub .text.* .gnu.linkonce.t.*)');
      add('		KEEP (*(.text.*personality*))');
      add('		/* .gnu.warning sections are handled specially by elf32.em.  */');
      add('		*(.gnu.warning)');
      add('		*(.glue_7t) *(.glue_7) *(.vfp11_veneer)');
      add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
      add('	} >rom = 0xff');
      add('');
      add('	__text_end = .;');
      add('	.fini           :');
      add('	{');
      add('		KEEP (*(.fini))');
      add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
      add('	} >rom =0');
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
      add('	} >rom = 0xff');
      add('  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >rom');
      add('   __exidx_start = .;');
      add('  .ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >rom');
      add('   __exidx_end = .;');
      add('');
      add('	.ctors :');
      add('	{');
      add('		/*	gcc uses crtbegin.o to find the start of the constructors, so');
      add('			we make sure it is first.  Because this is a wildcard, it');
      add('			doesn''t matter if the user does not actually link against');
      add('			crtbegin.o; the linker won''t look for a file to match a');
      add('			wildcard.  The wildcard also means that it doesn''t matter which');
      add('			directory crtbegin.o is in.  */');
      add('		KEEP (*crtbegin.o(.ctors))');
      add('		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))');
      add('		KEEP (*(SORT(.ctors.*)))');
      add('		KEEP (*(.ctors))');
      add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
      add('	} >rom = 0');
      add('');
      add('	.dtors :');
      add('	{');
      add('		KEEP (*crtbegin.o(.dtors))');
      add('		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))');
      add('		KEEP (*(SORT(.dtors.*)))');
      add('		KEEP (*(.dtors))');
      add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
      add('	} >rom = 0');
      add('');
      add('');
      add('	.eh_frame :');
      add('	{');
      add('		KEEP (*(.eh_frame))');
      add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
      add('	} >rom = 0');
      add('');
      add('	.gcc_except_table :');
      add('	{');
      add('		*(.gcc_except_table)');
      add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
      add('	} >rom = 0');
      add('');
      add('	__iwram_lma = .;');
      add('');
      add('	.iwram __iwram_start : AT (__iwram_lma)');
      add('	{');
      add('		__iwram_start = ABSOLUTE(.) ;');
      add('		*(.iwram)');
      add('		*iwram.*(.text)');
      add('		. = ALIGN(4);   /* REQUIRED. LD is flaky without it. */');
      add('		__iwram_end = ABSOLUTE(.) ;');
      add('	} >iwram = 0xff');
      add('');
      add('	__data_lma = __iwram_lma + SIZEOF(.iwram) ;');
      add('');
      add('	.bss ALIGN(4) (NOLOAD) :');
      add('	{');
      add('		__bss_start = ABSOLUTE(.);');
      add('		__bss_start__ = ABSOLUTE(.);');
      add('		*(.dynbss)');
      add('		*(.gnu.linkonce.b*)');
      add('		*(.bss*)');
      add('		*(COMMON)');
      add('		. = ALIGN(4);    /* REQUIRED. LD is flaky without it. */');
      add('		__bss_end = ABSOLUTE(.) ;');
      add('');
      add('	} AT>iwram');
      add('');
      add('	__bss_end__ = __bss_end ;');
      add('');
      add('	.data ALIGN(4) : AT (__data_lma)');
      add('	{');
      add('		__data_start = ABSOLUTE(.);');
      add('		*(.data)');
      add('		*(.data.*)');
      add('		*(.gnu.linkonce.d*)');
      add('		*(.fpc*)');
      add('		CONSTRUCTORS');
      add('		. = ALIGN(4);');
      add('	} >iwram = 0xff');
      add('');
      add('	__preinit_lma = __data_lma + SIZEOF(.data);');
      add('');
      add('	PROVIDE (__preinit_array_start = .);');
      add('	.preinit_array     : AT (__preinit_lma) { KEEP (*(.preinit_array)) } >iwram');
      add('	PROVIDE (__preinit_array_end = .);');
      add('');
      add('	__init_lma = __preinit_lma + SIZEOF(.preinit_array);');
      add('');
      add('	PROVIDE (__init_array_start = .);');
      add('	.init_array     : AT (__init_lma)');
      add('	{');
      add('		KEEP (*(SORT(.init_array.*)))');
      add('		KEEP (*(.init_array))');
      add('	} >iwram');
      add('	PROVIDE (__init_array_end = .);');
      add('	PROVIDE (__fini_array_start = .);');
      add('');
      add('	__fini_lma = __init_lma + SIZEOF(.init_array);');
      add('');
      add('	.fini_array     : AT (__fini_lma)');
      add('	{');
      add('		KEEP (*(SORT(.fini_array.*)))');
      add('		KEEP (*(.fini_array))');
      add('	} >iwram');
      add('  	PROVIDE (__fini_array_end = .);');
      add('');
      add('	__jcr_lma = __fini_lma + SIZEOF(.fini_array);');
      add('	.jcr            : AT (__jcr_lma) { KEEP (*(.jcr)) } >iwram');
      add('');
      add('	__data_end  =  ABSOLUTE(.);');
      add('	__iwram_overlay_lma = __jcr_lma + SIZEOF(.jcr);');
      add('');
      add('	__iwram_overlay_start = . ;');
      add('');
      add('	OVERLAY ALIGN(4) : NOCROSSREFS AT (__iwram_overlay_lma)');
      add('	{');
      add('		.iwram0 { *(.iwram0) . = ALIGN(4);}');
      add('		.iwram1 { *(.iwram1) . = ALIGN(4);}');
      add('		.iwram2 { *(.iwram2) . = ALIGN(4);}');
      add('		.iwram3 { *(.iwram3) . = ALIGN(4);}');
      add('		.iwram4 { *(.iwram4) . = ALIGN(4);}');
      add('		.iwram5 { *(.iwram5) . = ALIGN(4);}');
      add('		.iwram6 { *(.iwram6) . = ALIGN(4);}');
      add('		.iwram7 { *(.iwram7) . = ALIGN(4);}');
      add('		.iwram8 { *(.iwram8) . = ALIGN(4);}');
      add('		.iwram9 { *(.iwram9) . = ALIGN(4);}');
      add('	}>iwram = 0xff');
      add('');
      add('	__iwram_overlay_end = . ;');
      add('  __ewram_lma = __iwram_overlay_lma + (__iwram_overlay_end - __iwram_overlay_start) ;');
      add('');
      add('	__iheap_start = . ;');
      add('');
      add('	__ewram_start = ORIGIN(ewram);');
      add('	.ewram __ewram_start : AT (__ewram_lma)');
      add('	{');
      add('		*(.ewram)');
      add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
      add('	}>ewram = 0xff');
      add('');
      add('	__pad_lma = __ewram_lma + SIZEOF(.ewram);');
      add('');
      add('	.sbss ALIGN(4)(NOLOAD):');
      add(' 	{');
      add('		__sbss_start = ABSOLUTE(.);');
      add(' 		*(.sbss)');
      add(' 		. = ALIGN(4);');
      add('		__sbss_end  = ABSOLUTE(.);');
      add(' 	} AT>ewram');
      add('');
      add('');
      add('	__ewram_end = __sbss_end ;');
      add('	__eheap_start = __sbss_end;');
      add('	__end__ = __sbss_end;');
      add('');
      add('	/* EZF Advance strips trailing 0xff bytes, add a pad section so nothing important is removed */');
      add('	.pad ALIGN(4) : AT (__pad_lma)');
      add('	{');
      add('		LONG(0x52416b64)');
      add('		LONG(0x4d)');
      add('		. = ALIGN(4);  /* REQUIRED. LD is flaky without it. */');
      add('	} = 0xff');
      add('	__rom_end__ = __pad_lma + SIZEOF(.pad);');
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
      add('  .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }');
      add('  .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) }');
      add('  /DISCARD/ : { *(.note.GNU-stack) }');
      add('}');
    end;

{ Write and Close response }
  linkres.writetodisk;
  linkres.free;

  WriteResponseFile:=True;

end;


function TLinkerGba.MakeExecutable:boolean;
var
  binstr,
  cmdstr  : TCmdStr;
  success : boolean;
  StaticStr,
  GCSectionsStr,
  DynLinkStr,
  MapStr,
  StripStr: string;
begin
  { for future use }
  StaticStr:='';
  StripStr:='';
  DynLinkStr:='';
  MapStr:='';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  GCSectionsStr:='';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

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
  //if not(cs_link_extern in current_settings.globalswitches) then
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);

{ Write used files and libraries }
  WriteResponseFile();

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
  Replace(cmdstr,'$OPT',Info.ExtraOptions);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename,'.elf')))));
=======
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename^,'.elf')))));
>>>>>>> graemeg/cpstrnew
=======
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename^,'.elf')))));
>>>>>>> graemeg/cpstrnew
=======
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename^,'.elf')))));
>>>>>>> graemeg/cpstrnew
=======
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename^,'.elf')))));
>>>>>>> origin/cpstrnew
=======
  Replace(cmdstr,'$EXE',(maybequoted(ScriptFixFileName(ChangeFileExt(current_module.exefilename^,'.elf')))));
>>>>>>> origin/fixes_2.4
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
      success:=DoExec(FindUtil(utilsprefix+'objcopy'),'-O binary '+
        ChangeFileExt(current_module.exefilename,'.elf')+' '+
        current_module.exefilename,true,false);
    end;

  if success then 
    begin
      success:=DoExec(FindUtil('gbafix'), current_module.exefilename,true,false);
    end;


  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
  RegisterLinker(ld_gba,TLinkerGba);
  RegisterTarget(system_arm_gba_info);
end.
