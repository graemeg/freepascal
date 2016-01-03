{
    Copyright (c) 2011 by Francesco Lombardi

    This unit implements support import, export, link routines
    for the Wii (PowerPC) target

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
unit t_wii;

{$i fpcdefs.inc}

interface


implementation

    uses
       aasmbase,
       SysUtils,
       cutils,cfileutl,cclasses,
       globtype,globals,systems,verbose,script,fmodule,i_wii,link;

    type
       TlinkerWii=class(texternallinker)
       private
          Function  WriteResponseFile: Boolean;
       public
          constructor Create; override;
          procedure SetDefaultInfo; override;
          function  MakeExecutable:boolean; override;
       end;



{****************************************************************************
                               TLinkerWii
****************************************************************************}

Constructor TLinkerWii.Create;
begin
  Inherited Create;
  SharedLibFiles.doubles:=true;
  StaticLibFiles.doubles:=true;
end;


procedure TLinkerWii.SetDefaultInfo;
begin
  with Info do
   begin
     ExeCmd[1]:='ld -g $OPT $DYNLINK $STATIC $GCSECTIONS $STRIP -L. -o $EXE -T $RES';
   end;
end;


Function TLinkerWii.WriteResponseFile : Boolean;
Var
  linkres  : TLinkRes;
  i        : longint;
  HPath    : TCmdStrListItem;
  s,s1,s2  : TCmdStr;
  linklibc,
  linklibgcc  : boolean;
  found1,
  found2   : boolean;    
begin
  WriteResponseFile:=False;
  linklibc:=(SharedLibFiles.Find('c')<>nil);
  linklibgcc:=(SharedLibFiles.Find('gcc')<>nil);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName,true);
=======
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName);
>>>>>>> graemeg/cpstrnew
=======
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName);
>>>>>>> graemeg/cpstrnew
=======
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName);
>>>>>>> graemeg/cpstrnew
=======
  LinkRes:=TLinkRes.Create(outputexedir+Info.ResName);
>>>>>>> origin/cpstrnew

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     LinkRes.Add('SEARCH_DIR("'+s+'")');
=======
     LinkRes.Add('SEARCH_DIR('+(maybequoted(s))+')');
>>>>>>> graemeg/cpstrnew
=======
     LinkRes.Add('SEARCH_DIR('+(maybequoted(s))+')');
>>>>>>> graemeg/cpstrnew
=======
     LinkRes.Add('SEARCH_DIR('+(maybequoted(s))+')');
>>>>>>> graemeg/cpstrnew
=======
     LinkRes.Add('SEARCH_DIR('+(maybequoted(s))+')');
>>>>>>> origin/cpstrnew
    HPath:=TCmdStrListItem(HPath.Next);
   end;

  LinkRes.Add('INPUT (');
  { add objectfiles, start with prt0 always }
//  s:=FindObjectFile('prt0','',false);
//  LinkRes.AddFileName(s);
  { try to add crti and crtbegin if linking to C }
  if linklibc then
   begin
     if librarysearchpath.FindFile('ecrti.o',false,s) then
      LinkRes.AddFileName(s);
   end;
  if linklibgcc then
   begin
     if librarysearchpath.FindFile('crtbegin.o',false,s) then
      LinkRes.AddFileName(s);
   end;
  if linklibc or linklibgcc then
   begin
     if librarysearchpath.FindFile('crtmain.o',false,s) then
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
     found2:=librarysearchpath.FindFile('ecrtn.o',false,s2);
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
    Add('/*');
    Add(' * Linkscript for Wii');
    Add(' */');
    Add('');
    Add('OUTPUT_FORMAT("elf32-powerpc", "elf32-powerpc", "elf32-powerpc");');
    Add('OUTPUT_ARCH(powerpc:common);');
    Add('EXTERN(_start);');
    Add('ENTRY(_start);');
    Add('');
    Add('PHDRS');
    Add('{');
    Add('  stub PT_LOAD FLAGS(5);');
    Add('  text PT_LOAD FLAGS(5);');
    Add('  data PT_LOAD FLAGS(6);');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Add('  bss1 PT_LOAD;');
    Add('  bss2 PT_LOAD;');
    Add('');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Add('}');
    Add('');
    Add('SECTIONS');
    Add('{');
    Add('	/* stub is loaded at physical address 0x00003400 (though both 0x80003400 and 0x00003400 are equivalent for IOS) */');
    Add('	/* This can also be used to load an arbitrary standalone stub at an arbitrary address in memory, for any purpose */');
    Add('	/* Use -Wl,--section-start,.stub=0xADDRESS to change */');
    Add('	. = 0x00003400;');
    Add('');
    Add('	.stub :');
    Add('	{');
    Add('		KEEP(*(.stub))');
    Add('	} :stub = 0');
    Add('');
    Add('	/* default base address */');
    Add('	/* use -Wl,--section-start,.init=0xADDRESS to change */');
    Add('	. = 0x80004000;');
    Add('');
    Add('	/* Program */');
    Add('	.init          :');
    Add('	{');
    Add('		KEEP (*crt0.o(*.init))');
    Add('		KEEP (*(.init))');
    Add('	} :text = 0');
    Add('	.plt      : { *(.plt)	}');
    Add('	.interp			: { *(.interp) 	}');
    Add('	.hash			: { *(.hash) }');
    Add('	.dynsym			: { *(.dynsym) }');
    Add('	.dynstr			: { *(.dynstr) }');
    Add('	.gnu.version	: { *(.gnu.version) }');
    Add('	.gnu.version_d	: { *(.gnu.version_d) }');
    Add('	.gnu.version_r	: { *(.gnu.version_r) }');
    Add('	.rel.init		: { *(.rel.init) }');
    Add('	.rela.init		: { *(.rela.init) }');
    Add('	.rel.text		: { *(.rel.text .rel.text.* .rel.gnu.linkonce.t.*) }');
    Add('	.rela.text		: { *(.rela.text .rela.text.* .rela.gnu.linkonce.t.*) }');
    Add('	.rel.fini		: { *(.rel.fini) }');
    Add('	.rela.fini		: { *(.rela.fini) }');
    Add('	.rel.rodata		: { *(.rel.rodata .rel.rodata.* .rel.gnu.linkonce.r.*) }');
    Add('	.rela.rodata	: { *(.rela.rodata .rela.rodata.* .rela.gnu.linkonce.r.*) }');
    Add('	.rel.data		: { *(.rel.data .rel.data.* .rel.gnu.linkonce.d.*) }');
    Add('	.rela.data		: { *(.rela.data .rela.data.* .rela.gnu.linkonce.d.*) }');
    Add('	.rel.tdata		: { *(.rel.tdata .rel.tdata.* .rel.gnu.linkonce.td.*) }');
    Add('	.rela.tdata		: { *(.rela.tdata .rela.tdata.* .rela.gnu.linkonce.td.*) }');
    Add('	.rel.tbss		: { *(.rel.tbss .rel.tbss.* .rel.gnu.linkonce.tb.*) }');
    Add('	.rela.tbss		: { *(.rela.tbss .rela.tbss.* .rela.gnu.linkonce.tb.*) }');
    Add('	.rel.ctors		: { *(.rel.ctors) }');
    Add('	.rela.ctors		: { *(.rela.ctors) }');
    Add('	.rel.dtors		: { *(.rel.dtors) }');
    Add('	.rela.dtors		: { *(.rela.dtors) }');
    Add('	.rel.got		: { *(.rel.got)	}');
    Add('	.rela.got		: { *(.rela.got) }');
    Add('	.rela.got1		: { *(.rela.got1) }');
    Add('	.rela.got2		: { *(.rela.got2) }');
    Add('	.rel.sdata		: { *(.rel.sdata .rel.sdata.* .rel.gnu.linkonce.s.*) }');
    Add('	.rela.sdata		: { *(.rela.sdata .rela.sdata.* .rela.gnu.linkonce.s.*) }');
    Add('	.rel.sbss		: { *(.rel.sbss .rel.sbss.* .rel.gnu.linkonce.sb.*) }');
    Add('	.rela.sbss		: { *(.rela.sbss .rela.sbss.* .rel.gnu.linkonce.sb.*) }');
    Add('	.rel.sdata2		: { *(.rel.sdata2 .rel.sdata2.* .rel.gnu.linkonce.s2.*) }');
    Add('	.rela.sdata2	: { *(.rela.sdata2 .rela.sdata2.* .rela.gnu.linkonce.s2.*) }');
    Add('	.rel.sbss2		: { *(.rel.sbss2 .rel.sbss2.* .rel.gnu.linkonce.sb2.*) }');
    Add('	.rela.sbss2		: { *(.rela.sbss2 .rela.sbss2.* .rela.gnu.linkonce.sb2.*) }');
    Add('	.rel.bss		: { *(.rel.bss .rel.bss.* .rel.gnu.linkonce.b.*) }');
    Add('	.rela.bss		: { *(.rela.bss .rela.bss.* .rela.gnu.linkonce.b.*) }');
    Add('	.rel.plt		: { *(.rel.plt) }');
    Add('	.rela.plt		: { *(.rela.plt) }');
    Add('');
    Add('	.text      :');
    Add('	{');
    Add('		*(.text)');
    Add('		*(.text.*)');
    Add('		/* .gnu.warning sections are handled specially by elf32.em.  */');
    Add('		*(.gnu.warning)');
    Add('		*(.gnu.linkonce.t.*)');
    Add('		. = ALIGN(32);   /* REQUIRED. LD is flaky without it. */');
    Add('	} = 0');
    Add('');
    Add('	.fini      :');
    Add('	{');
    Add('		KEEP (*(.fini))');
    Add('		. = ALIGN(32);   /* REQUIRED. LD is flaky without it. */');
    Add('	} = 0');
    Add('	');
    Add('	PROVIDE (__etext = .);');
    Add('	PROVIDE (_etext = .);');
    Add('	PROVIDE (etext = .);');
    Add('');
    Add('	.rodata   : { *(.rodata) *(.rodata.*) *(.gnu.linkonce.r.*) } :data');
    Add('	.rodata1   : { *(.rodata1) }');
    Add('	.sdata2   : { *(.sdata2) *(.sdata2.*) *(.gnu.linkonce.s2.*) }');
    Add('	.sbss2   : { *(.sbss2) *(.sbss2.*) *(.gnu.linkonce.sb2.*) }');
    Add('  /* Adjust the address for the data segment.  We want to adjust up to');
    Add('     the same address within the page on the next page up.  */');
    Add('  /* Ensure the __preinit_array_start label is properly aligned.  We');
    Add('     could instead move the label definition inside the section, but');
    Add('     the linker would then create the section even if it turns out to');
    Add('     be empty, which isn''t pretty.  */');
    Add('	. = ALIGN(32 / 8);');
    Add('	PROVIDE (__preinit_array_start = .);');
    Add('	.preinit_array     : { *(.preinit_array) }');
    Add('	PROVIDE (__preinit_array_end = .);');
    Add('	PROVIDE (__init_array_start = .);');
    Add('	.init_array     : { *(.init_array) }');
    Add('	PROVIDE (__init_array_end = .);');
    Add('	PROVIDE (__fini_array_start = .);');
    Add('	.fini_array     : { *(.fini_array) }');
    Add('	PROVIDE (__fini_array_end = .);');
    Add('	.data    :');
    Add('	{');
    Add('		*(.data)');
    Add('		*(.data.*)');
    Add('		*(.gnu.linkonce.d.*)');
    Add('		SORT(CONSTRUCTORS)');
    Add('		. = ALIGN(32);   /* REQUIRED. LD is flaky without it. */');
    Add('	}');
    Add('');
    Add('	.data1   : { *(.data1) }');
    Add('	.tdata	  : { *(.tdata .tdata.* .gnu.linkonce.td.*) }');
    Add('	.tbss		  : { *(.tbss .tbss.* .gnu.linkonce.tb.*) *(.tcommon) }');
    Add('	.eh_frame : { KEEP (*(.eh_frame)) }');
    Add('	.gcc_except_table : { *(.gcc_except_table) }');
    Add('	.fixup          : { *(.fixup) }');
    Add('	.got1           : { *(.got1) }');
    Add('	.got2           : { *(.got2) }');
    Add('	.dynamic       : { *(.dynamic) }');
    Add('');
    Add('	.ctors   :');
    Add('	{');
    Add('	/*	gcc uses crtbegin.o to find the start of');
    Add('		the constructors, so we make sure it is');
    Add('		first.  Because this is a wildcard, it');
    Add('		doesn''t matter if the user does not');
    Add('		actually link against crtbegin.o; the');
    Add('		linker won''t look for a file to match a');
    Add('		wildcard.  The wildcard also means that it');
    Add('		doesn''t matter which directory crtbegin.o');
    Add('		is in.  */');
    Add('');
    Add('		KEEP (*crtbegin.o(.ctors))');
    Add('');
    Add('    /*	We don''t want to include the .ctor section from');
    Add('		from the crtend.o file until after the sorted ctors.');
    Add('		The .ctor section from the crtend file contains the');
    Add('		end of ctors marker and it must be last */');
    Add('');
    Add('		KEEP (*(EXCLUDE_FILE (*crtend.o ) .ctors))');
    Add('		KEEP (*(SORT(.ctors.*)))');
    Add('		KEEP (*(.ctors))');
    Add('		. = ALIGN(32);   /* REQUIRED. LD is flaky without it. */');
    Add('	}');
    Add('');
    Add('	.dtors         :');
    Add('	{');
    Add('		KEEP (*crtbegin.o(.dtors))');
    Add('		KEEP (*(EXCLUDE_FILE (*crtend.o ) .dtors))');
    Add('		KEEP (*(SORT(.dtors.*)))');
    Add('		KEEP (*(.dtors))');
    Add('		. = ALIGN(32);   /* REQUIRED. LD is flaky without it. */');
    Add('	}');
    Add('');
    Add('	.jcr            : { KEEP (*(.jcr)) }');
    Add('	.got		  : { *(.got.plt) *(.got) }');
    Add('');
    Add('');
    Add('	/*	We want the small data sections together, so single-instruction offsets');
    Add('		can access them all, and initialized data all before uninitialized, so');
    Add('		we can shorten the on-disk segment size.  */');
    Add('');
    Add('	.sdata     :');
    Add('	{');
    Add('		*(.sdata)');
    Add('		*(.sdata.*)');
    Add('		*(.gnu.linkonce.s.*)');
    Add('		. = ALIGN(32);   /* REQUIRED. LD is flaky without it. */');
    Add('	}');
    Add('');
    Add('	_edata = .;');
    Add('	PROVIDE (edata = .);');
    Add('	');
    Add('	.sbss      :');
    Add('	{');
    Add('		__sbss_start = .;');
    Add('		PROVIDE (__sbss_start = .);');
    Add('		PROVIDE (___sbss_start = .);');
    Add('		*(.dynsbss)');
    Add('		*(.sbss)');
    Add('		*(.sbss.*)');
    Add('		*(.gnu.linkonce.sb.*)');
    Add('		*(.scommon)');
    Add('		PROVIDE (__sbss_end = .);');
    Add('		PROVIDE (___sbss_end = .);');
    Add('		. = ALIGN(32);   /* REQUIRED. LD is flaky without it. */');
    Add('		__sbss_end = .;');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Add('	} :bss1');
=======
    Add('	}');
>>>>>>> graemeg/cpstrnew
=======
    Add('	}');
>>>>>>> graemeg/cpstrnew
=======
    Add('	}');
>>>>>>> graemeg/cpstrnew
=======
    Add('	}');
>>>>>>> origin/cpstrnew
    Add('');
    Add('	.bss       :');
    Add('	{');
    Add('		__bss_start = .;');
    Add('		PROVIDE (__bss_start = .);');
    Add('		*(.dynbss)');
    Add('		*(.bss)');
    Add('		*(.bss.*)');
    Add('		*(.gnu.linkonce.b.*)');
    Add('		*(COMMON)');
    Add('	/*	Align here to ensure that the .bss section occupies space up to');
    Add('		_end.  Align after .bss to ensure correct alignment even if the');
    Add('		.bss section disappears because there are no input sections.  */');
    Add('');
    Add('		. = ALIGN(32);');
    Add('');
    Add('		PROVIDE (__bss_end = .);');
    Add('		__bss_end = .;');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Add('	} :bss2');
=======
    Add('	}');
>>>>>>> graemeg/cpstrnew
=======
    Add('	}');
>>>>>>> graemeg/cpstrnew
=======
    Add('	}');
>>>>>>> graemeg/cpstrnew
=======
    Add('	}');
>>>>>>> origin/cpstrnew
    Add('');
    Add('	_end = .;');
    Add('	PROVIDE(end = .);');
    Add('	/* Stabs debugging sections.  */');
    Add('	.stab 0 : { *(.stab) }');
    Add('	.stabstr 0 : { *(.stabstr) }');
    Add('	.stab.excl 0 : { *(.stab.excl) }');
    Add('	.stab.exclstr 0 : { *(.stab.exclstr) }');
    Add('	.stab.index 0 : { *(.stab.index) }');
    Add('	.stab.indexstr 0 : { *(.stab.indexstr) }');
    Add('	.comment 0 : { *(.comment) }');
    Add('	/*	DWARF debug sections.');
    Add('		Symbols in the DWARF debugging sections are relative to the beginning');
    Add('		of the section so we begin them at 0.  */');
    Add('	/* DWARF 1 */');
    Add('	.debug          0 : { *(.debug) }');
    Add('	.line           0 : { *(.line) }');
    Add('	/* GNU DWARF 1 extensions */');
    Add('	.debug_srcinfo  0 : { *(.debug_srcinfo) }');
    Add('	.debug_sfnames  0 : { *(.debug_sfnames) }');
    Add('	/* DWARF 1.1 and DWARF 2 */');
    Add('	.debug_aranges  0 : { *(.debug_aranges) }');
    Add('	.debug_pubnames 0 : { *(.debug_pubnames) }');
    Add('	/* DWARF 2 */');
    Add('	.debug_info     0 : { *(.debug_info) }');
    Add('	.debug_abbrev   0 : { *(.debug_abbrev) }');
    Add('	.debug_line     0 : { *(.debug_line) }');
    Add('	.debug_frame    0 : { *(.debug_frame) }');
    Add('	.debug_str      0 : { *(.debug_str) }');
    Add('	.debug_loc      0 : { *(.debug_loc) }');
    Add('	.debug_macinfo  0 : { *(.debug_macinfo) }');
    Add('	/* SGI/MIPS DWARF 2 extensions */');
    Add('	.debug_weaknames 0 : { *(.debug_weaknames) }');
    Add('	.debug_funcnames 0 : { *(.debug_funcnames) }');
    Add('	.debug_typenames 0 : { *(.debug_typenames) }');
    Add('	.debug_varnames  0 : { *(.debug_varnames) }');
    Add('	/* These must appear regardless of  .  */');
    Add('}');
    Add('');
    Add('__isIPL = 0;');
    Add('__stack_addr = (__bss_start + SIZEOF(.bss) + 0x20000 + 7) & (-8);');
    Add('__stack_end = (__bss_start + SIZEOF(.bss));');
    Add('__intrstack_addr = (__stack_addr + 0x4000);');
    Add('__intrstack_end = (__stack_addr);');
    Add('__Arena1Lo = (__intrstack_addr + 31) & (-32);');
    Add('__Arena1Hi = (0x817FEFF0);');
    Add('__Arena2Lo = (0x90002000);');
    Add('__Arena2Hi = (0x933E0000);');
    Add('');
    Add('__gxregs = (__Arena1Hi + 31) & (-32);');
    Add('__ipcbufferLo = (0x933e0000);');
    Add('__ipcbufferHi = (0x93400000);');
    Add('');
    Add('/* for backward compatibility with old crt0 */');
    Add('PROVIDE (__stack = (0x817FEFF0));');
    Add('');
    Add('PROVIDE(__isIPL = __isIPL);');
    Add('PROVIDE(__stack_addr = __stack_addr);');
    Add('PROVIDE(__stack_end = __stack_end);');
    Add('PROVIDE(__intrstack_addr = __intrstack_addr);');
    Add('PROVIDE(__intrstack_end = __intrstack_end);');
    Add('PROVIDE(__Arena1Lo = __Arena1Lo);');
    Add('PROVIDE(__Arena1Hi = __Arena1Hi);');
    Add('PROVIDE(__Arena2Lo = __Arena2Lo);');
    Add('PROVIDE(__Arena2Hi = __Arena2Hi);');
    Add('PROVIDE(__ipcbufferLo = __ipcbufferLo);');
    Add('PROVIDE(__ipcbufferHi = __ipcbufferHi);');
    Add('PROVIDE(__gxregs = __gxregs);');
   end;
   
   
{ Write and Close response }
  linkres.writetodisk;
  linkres.free;

  WriteResponseFile:=True;

end;


function TLinkerWii.MakeExecutable:boolean;
var
  binstr,
  cmdstr  : TCmdStr;
  success : boolean;
  StaticStr,
  GCSectionsStr,
  DynLinkStr,
  StripStr: string;
begin
  StaticStr:='';
  StripStr:='';
  GCSectionsStr:='';
  DynLinkStr:='';

  if (cs_link_strip in current_settings.globalswitches) and
     not(cs_link_separate_dbg_file in current_settings.globalswitches) then
   StripStr:='-s';
  if (cs_link_map in current_settings.globalswitches) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename,'.map'));
  if create_smartlink_sections then
   GCSectionsStr:='--gc-sections';
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   StripStr:='-Map '+maybequoted(ChangeFileExt(current_module.exefilename^,'.map'));
  if create_smartlink_sections then
   GCSectionsStr:='--gc-sections';
  if not(cs_link_nolink in current_settings.globalswitches) then
   Message1(exec_i_linking,current_module.exefilename^);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{ Write used files and libraries }
  WriteResponseFile();

{ Call linker }
  SplitBinCmd(Info.ExeCmd[1],binstr,cmdstr);
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
  Replace(cmdstr,'$OPT',Info.ExtraOptions);
  Replace(cmdstr,'$RES',(maybequoted(ScriptFixFileName(outputexedir+Info.ResName))));
  Replace(cmdstr,'$STATIC',StaticStr);
  Replace(cmdstr,'$STRIP',StripStr);
  Replace(cmdstr,'$GCSECTIONS',GCSectionsStr);
  Replace(cmdstr,'$DYNLINK',DynLinkStr);

  success:=DoExec(FindUtil(utilsprefix+BinStr),cmdstr,true,false);

{ Remove ReponseFile }
  if (success) and not(cs_link_nolink in current_settings.globalswitches) then
   DeleteFile(outputexedir+Info.ResName);
   
{ Post process }

  if success then 
   begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    success:=DoExec(FindUtil('elf2dol'),ChangeFileExt(current_module.exefilename,'.elf')+' '+ 
     current_module.exefilename,true,false);
=======
    success:=DoExec(FindUtil('elf2dol'),ChangeFileExt(current_module.exefilename^,'.elf')+' '+ 
     current_module.exefilename^,true,false);
>>>>>>> graemeg/cpstrnew
=======
    success:=DoExec(FindUtil('elf2dol'),ChangeFileExt(current_module.exefilename^,'.elf')+' '+ 
     current_module.exefilename^,true,false);
>>>>>>> graemeg/cpstrnew
=======
    success:=DoExec(FindUtil('elf2dol'),ChangeFileExt(current_module.exefilename^,'.elf')+' '+ 
     current_module.exefilename^,true,false);
>>>>>>> graemeg/cpstrnew
=======
    success:=DoExec(FindUtil('elf2dol'),ChangeFileExt(current_module.exefilename^,'.elf')+' '+ 
     current_module.exefilename^,true,false);
>>>>>>> origin/cpstrnew
   end;
  MakeExecutable:=success;   { otherwise a recursive call to link method }
end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  RegisterLinker(ld_wii,TLinkerWii);
=======
  RegisterExternalLinker(system_powerpc_wii_info,TLinkerWii);
>>>>>>> graemeg/cpstrnew
=======
  RegisterExternalLinker(system_powerpc_wii_info,TLinkerWii);
>>>>>>> graemeg/cpstrnew
=======
  RegisterExternalLinker(system_powerpc_wii_info,TLinkerWii);
>>>>>>> graemeg/cpstrnew
=======
  RegisterExternalLinker(system_powerpc_wii_info,TLinkerWii);
>>>>>>> origin/cpstrnew
  RegisterTarget(system_powerpc_wii_info);
end.
