{
    Copyright (c) 1998-2006 by Florian Klaempfl

    This unit implements an abstract asmoutput class for all processor types

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
{ @abstract(This unit implements an abstract asm output class for all processor types)
  This unit implements an abstract assembler output class for all processors, these
  are then overridden for each assembler writer to actually write the data in these
  classes to an assembler file.
}

unit aasmtai;

{$i fpcdefs.inc}

interface

    uses
       cutils,cclasses,
       globtype,systems,
       cpuinfo,cpubase,
{$ifdef llvm}
       { overrides max_operands }
       llvmbase,
{$endif llvm}
       cgbase,cgutils,
       symtype,
       aasmbase,aasmdata,ogbase
{$ifdef jvm}
       ,widestr
{$endif jvm}
       ;

    type
       { keep the number of elements in this enumeration less or equal than 32 as long
         as FPC knows only 4 byte and 32 byte sets (FK) }
       taitype = (
          ait_none,
          ait_align,
          ait_section,
          ait_comment,
          ait_string,
          ait_instruction,
          ait_datablock,
          ait_symbol,
          { needed to calc the size of a symbol }
          ait_symbol_end,
          ait_directive,
          ait_label,
          ait_const,
          ait_realconst,
          ait_typedconst,
          ait_stab,
          ait_force_line,
          ait_function_name,
{$ifdef m68k}
          ait_labeled_instruction,
{$endif m68k}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          ait_symbolpair,
          ait_weak,
=======
{$ifdef arm}
          ait_thumb_func,
{$endif arm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef arm}
          ait_thumb_func,
{$endif arm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef arm}
          ait_thumb_func,
{$endif arm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef arm}
          ait_thumb_func,
{$endif arm}
>>>>>>> origin/cpstrnew
          { used to split into tiny assembler files }
          ait_cutobject,
          ait_regalloc,
          ait_tempalloc,
          { used to mark assembler blocks and inlined functions }
<<<<<<< HEAD
<<<<<<< HEAD
          ait_marker,
          { used to describe a new location of a variable }
          ait_varloc,
{$ifdef JVM}
          { JVM only }
          ait_jvar,    { debug information for a local variable }
          ait_jcatch,  { exception catch clause }
{$endif JVM}
{$ifdef llvm}
          ait_llvmins, { llvm instruction }
          ait_llvmalias, { alias for a symbol }
          ait_llvmdecl, { llvm symbol declaration (global/external variable, external procdef) }
{$endif}
          { SEH directives used in ARM,MIPS and x86_64 COFF targets }
          ait_seh_directive
=======
          ait_marker
>>>>>>> graemeg/fixes_2_2
=======
          ait_marker
>>>>>>> origin/fixes_2_2
          );

        taiconst_type = (
          aitconst_128bit,
          aitconst_64bit,
          aitconst_32bit,
          aitconst_16bit,
          aitconst_8bit,
          aitconst_sleb128bit,
          aitconst_uleb128bit,
          { win32 only }
          aitconst_rva_symbol,
          aitconst_secrel32_symbol,
          { darwin only }
<<<<<<< HEAD
<<<<<<< HEAD
=======
          aitconst_indirect_symbol,
>>>>>>> graemeg/fixes_2_2
=======
          aitconst_indirect_symbol,
>>>>>>> origin/fixes_2_2
          { From gcc/config/darwin.c (darwin_asm_output_dwarf_delta):
            ***
            Output a difference of two labels that will be an assembly time
            constant if the two labels are local.  (.long lab1-lab2 will be
            very different if lab1 is at the boundary between two sections; it
            will be relocated according to the second section, not the first,
            so one ends up with a difference between labels in different
            sections, which is bad in the dwarf2 eh context for instance.)
            ***
            We cannot use this everywhere, because older versions of the
            darwin assembler don't support the construct used for these
            relsyms (nor do they support dwarf, for that matter)
          }
          aitconst_darwin_dwarf_delta64,
<<<<<<< HEAD
<<<<<<< HEAD
          aitconst_darwin_dwarf_delta32,
          { ARM Thumb-2 only }
          aitconst_half16bit, { used for table jumps. The actual value is the 16bit value shifted left once }
          { AVR }
          aitconst_gs, { Upper 16-bit of 17-bit constant }
          { for use by dwarf debugger information }
          aitconst_16bit_unaligned,
          aitconst_32bit_unaligned,
          aitconst_64bit_unaligned,
          { i8086 far pointer; emits: 'DW symbol, SEG symbol' }
          aitconst_farptr,
          { i8086 segment of symbol; emits: 'DW SEG symbol' }
          aitconst_seg,
          { i8086 data segment group; emits: 'DW dgroup'
            generated by the this inline asm:
              DW SEG @DATA
            in all memory models, except huge }
          aitconst_dgroup,
          { i8086 far data segment of the current pascal module (unit or program);
            emits: 'DW CURRENTMODULENAME_DATA'
            generated by the this inline asm:
              DW SEG @DATA
            in the huge memory model }
          aitconst_fardataseg,
          { offset of symbol's GOT slot in GOT }
          aitconst_got,
          { offset of symbol itself from GOT }
          aitconst_gotoff_symbol
        );

        tairealconsttype = (
          aitrealconst_s32bit,
          aitrealconst_s64bit,
          aitrealconst_s80bit,
          aitrealconst_s128bit,
          aitrealconst_s64comp
=======
          aitconst_darwin_dwarf_delta32
>>>>>>> graemeg/fixes_2_2
=======
          aitconst_darwin_dwarf_delta32
>>>>>>> origin/fixes_2_2
        );

    const
{$if defined(cpu64bitaddr)}
       aitconst_ptr = aitconst_64bit;
       aitconst_ptr_unaligned = aitconst_64bit_unaligned;
{$elseif defined(cpu32bitaddr)}
       aitconst_ptr = aitconst_32bit;
       aitconst_ptr_unaligned = aitconst_32bit_unaligned;
{$elseif defined(cpu16bitaddr)}
       aitconst_ptr = aitconst_16bit;
       aitconst_ptr_unaligned = aitconst_16bit_unaligned;
{$endif}

{$if defined(cpu64bitalu)}
       aitconst_aint = aitconst_64bit;
{$elseif defined(cpu32bitalu)}
       aitconst_aint = aitconst_32bit;
{$elseif defined(cpu16bitalu)}
       aitconst_aint = aitconst_16bit;
{$elseif defined(cpu8bitalu)}
       aitconst_aint = aitconst_8bit;
{$endif}

       taitypestr : array[taitype] of string[24] = (
          '<none>',
          'align',
          'section',
          'comment',
          'string',
          'instruction',
          'datablock',
          'symbol',
          'symbol_end',
          'symbol_directive',
          'label',
          'const',
          'realconst',
          'typedconst',
          'stab',
          'force_line',
          'function_name',
{$ifdef m68k}
          'labeled_instr',
{$endif m68k}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          'symbolpair',
          'weak',
=======
{$ifdef arm}
          'thumb_func',
{$endif arm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef arm}
          'thumb_func',
{$endif arm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef arm}
          'thumb_func',
{$endif arm}
>>>>>>> graemeg/cpstrnew
=======
{$ifdef arm}
          'thumb_func',
{$endif arm}
>>>>>>> origin/cpstrnew
          'cut',
          'regalloc',
          'tempalloc',
<<<<<<< HEAD
<<<<<<< HEAD
          'marker',
          'varloc',
{$ifdef JVM}
          'jvar',
          'jcatch',
{$endif JVM}
{$ifdef llvm}
          'llvmins',
          'llvmalias',
          'llvmdecl',
{$endif}
          'seh_directive'
=======
          'marker'
>>>>>>> graemeg/fixes_2_2
=======
          'marker'
>>>>>>> origin/fixes_2_2
          );

    type
      { Types of operand }
      toptype=(top_none,top_reg,top_ref,top_const,top_bool,top_local
{$ifdef arm}
       { ARM only }
       ,top_regset
<<<<<<< HEAD
       ,top_modeflags
       ,top_specialreg
=======
       ,top_shifterop
       ,top_conditioncode
       ,top_modeflags
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
{$endif arm}
{$if defined(arm) or defined(aarch64)}
       ,top_conditioncode
       ,top_shifterop
{$endif defined(arm) or defined(aarch64)}
{$ifdef m68k}
       { m68k only }
       ,top_regset
{$endif m68k}
{$ifdef jvm}
       { jvm only}
       ,top_single
       ,top_double
       ,top_string
       ,top_wstring
{$endif jvm}
{$ifdef llvm}
       { llvm only }
       ,top_single
       ,top_double
       ,top_undef
{$ifdef cpuextended}
       ,top_extended80
{$endif cpuextended}
       ,top_tai
       ,top_def
       ,top_fpcond
       ,top_cond
       ,top_para
{$endif llvm}
       );

      { kinds of operations that an instruction can perform on an operand }
      topertype = (operand_read,operand_write,operand_readwrite);

      tlocaloper = record
        localsym : pointer;
        localsymderef : tderef;
        localsymofs : longint;
        localindexreg : tregister;
        localscale : byte;
        localgetoffset,
        localforceref : boolean
      end;
      plocaloper = ^tlocaloper;

<<<<<<< HEAD
=======
      { please keep the size of this record <=12 bytes and keep it properly aligned }
      toper = record
        ot : longint;
        case typ : toptype of
          top_none   : ();
          top_reg    : (reg:tregister);
          top_ref    : (ref:preference);
          top_const  : (val:aint);
          top_bool   : (b:boolean);
          { local varsym that will be inserted in pass_generate_code }
          top_local  : (localoper:plocaloper);
      {$ifdef arm}
          top_regset : (regset:^tcpuregisterset; regtyp: tregistertype; subreg: tsubregister);
          top_shifterop : (shifterop : pshifterop);
          top_conditioncode : (cc : TAsmCond);
          top_modeflags : (modeflags : tcpumodeflags);
      {$endif arm}
      {$ifdef m68k}
          top_regset : (regset:^tcpuregisterset);
      {$endif m68k}
      end;
      poper=^toper;

>>>>>>> graemeg/cpstrnew
    const
      { ait_* types which don't result in executable code or which don't influence
        the way the program runs/behaves, but which may be encountered by the
        optimizer (= if it's sometimes added to the exprasm list). Update if you add
        a new ait type!                                                              }
      SkipInstr = [ait_comment, ait_symbol,ait_section
                   ,ait_stab, ait_function_name, ait_force_line
<<<<<<< HEAD
<<<<<<< HEAD
                   ,ait_regalloc, ait_tempalloc, ait_symbol_end
                   ,ait_directive
                   ,ait_varloc,
{$ifdef JVM}
                   ait_jvar,
{$endif JVM}
                   ait_seh_directive];
=======
                   ,ait_regalloc, ait_tempalloc, ait_symbol_end, ait_directive];
>>>>>>> graemeg/fixes_2_2
=======
                   ,ait_regalloc, ait_tempalloc, ait_symbol_end, ait_directive];
>>>>>>> origin/fixes_2_2

      { ait_* types which do not have line information (and hence which are of type
        tai, otherwise, they are of type tailineinfo }
      SkipLineInfo =[ait_label,
                     ait_regalloc,ait_tempalloc,
                     ait_stab,ait_function_name,
<<<<<<< HEAD
                     ait_cutobject,ait_marker,ait_varloc,ait_align,ait_section,ait_comment,
                     ait_const,ait_directive,
                     ait_symbolpair,ait_weak,
                     ait_realconst,
                     ait_symbol,
{$ifdef JVM}
                     ait_jvar, ait_jcatch,
{$endif JVM}
{$ifdef llvm}
                     ait_llvmdecl,
{$endif llvm}
                     ait_seh_directive
=======
                     ait_cutobject,ait_marker,ait_align,ait_section,ait_comment,
                     ait_const,
{$ifdef arm}
                     ait_thumb_func,
{$endif arm}
                     ait_real_32bit,ait_real_64bit,ait_real_80bit,ait_comp_64bit,ait_real_128bit,
                     ait_symbol
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                    ];


    type
      { cut type, required for alphanumeric ordering of the assembler filenames }
      TCutPlace=(cut_normal,cut_begin,cut_end);

      TAsmMarker = (
        mark_NoPropInfoStart,mark_NoPropInfoEnd,
        mark_AsmBlockStart,mark_AsmBlockEnd,
        mark_NoLineInfoStart,mark_NoLineInfoEnd,mark_BlockStart,
        mark_Position
      );

      TRegAllocType = (ra_alloc,ra_dealloc,ra_sync,ra_resize,ra_markused);

      TStabType = (stab_stabs,stab_stabn,stab_stabd,
                   { AIX/XCOFF stab types }
                   stab_stabx,
                   { begin/end include file }
                   stabx_bi,stabx_ei,
                   { begin/end function }
                   stabx_bf, stabx_ef,
                   { begin/end static data block }
                   stabx_bs, stabx_es,
                   { line spec, function start/end label }
                   stabx_line, stabx_function);

      TAsmDirective=(
        asd_indirect_symbol,
        asd_extern,asd_nasm_import, asd_toc_entry,
        asd_reference,asd_no_dead_strip,asd_weak_reference,asd_lazy_reference,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        asd_weak_definition,
        { for Jasmin }
        asd_jclass,asd_jinterface,asd_jsuper,asd_jfield,asd_jlimit,asd_jline,
        { .ent/.end for MIPS }
        asd_ent,asd_ent_end,
        { supported by recent clang-based assemblers for data-in-code  }
        asd_data_region, asd_end_data_region,
        { .thumb_func for ARM }
        asd_thumb_func
=======
        asd_weak_definition
>>>>>>> graemeg/cpstrnew
=======
        asd_weak_definition
>>>>>>> graemeg/cpstrnew
=======
        asd_weak_definition
>>>>>>> graemeg/cpstrnew
=======
        asd_weak_definition
>>>>>>> origin/cpstrnew
      );

      TAsmSehDirective=(
          ash_proc,ash_endproc,
          ash_endprologue,ash_handler,ash_handlerdata,
          ash_eh,ash_32,ash_no32,
          ash_setframe,ash_stackalloc,ash_pushreg,
          ash_savereg,ash_savexmm,ash_pushframe
        );

      TSymbolPairKind = (spk_set, spk_thumb_set, spk_localentry);


    const
      regallocstr : array[tregalloctype] of string[10]=('allocated','released','sync','resized','used');
      tempallocstr : array[boolean] of string[10]=('released','allocated');
      stabtypestr : array[TStabType] of string[8]=(
        'stabs','stabn','stabd',
        'stabx',
        'bi','ei',
        'bf','ef',
        'bs','es',
        'line','function');
      directivestr : array[TAsmDirective] of string[23]=(
        'indirect_symbol',
        'extern','nasm_import', 'tc', 'reference',
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        'no_dead_strip','weak_reference','lazy_reference','weak_definition',
        { for Jasmin }
        'class','interface','super','field','limit','line',
        { .ent/.end for MIPS }
        'ent','end',
        { supported by recent clang-based assemblers for data-in-code }
        'data_region','end_data_region',
        { .thumb_func for ARM }
        'thumb_func'
      );
      sehdirectivestr : array[TAsmSehDirective] of string[16]=(
        '.seh_proc','.seh_endproc',
        '.seh_endprologue','.seh_handler','.seh_handlerdata',
        '.seh_eh','.seh_32','seh_no32',
        '.seh_setframe','.seh_stackalloc','.seh_pushreg',
        '.seh_savereg','.seh_savexmm','.seh_pushframe'
      );
      symbolpairkindstr: array[TSymbolPairKind] of string[11]=(
        '.set', '.thumb_set', '.localentry'
=======
        'no_dead_strip','weak_reference','lazy_reference','weak_definition'
>>>>>>> graemeg/cpstrnew
=======
        'no_dead_strip','weak_reference','lazy_reference','weak_definition'
>>>>>>> graemeg/cpstrnew
=======
        'no_dead_strip','weak_reference','lazy_reference','weak_definition'
>>>>>>> graemeg/cpstrnew
=======
        'no_dead_strip','weak_reference','lazy_reference','weak_definition'
>>>>>>> origin/cpstrnew
      );

    type
        tai = class;

        { please keep the size of this record <=12 bytes and keep it properly aligned }
        toper = record
          ot : longint;
          case typ : toptype of
            top_none   : ();
            top_reg    : (reg:tregister);
            top_ref    : (ref:preference);
            top_const  : (val:tcgint);
            top_bool   : (b:boolean);
            { local varsym that will be inserted in pass_generate_code }
            top_local  : (localoper:plocaloper);
        {$ifdef arm}
            top_regset : (regset:^tcpuregisterset; regtyp: tregistertype; subreg: tsubregister; usermode: boolean);
            top_modeflags : (modeflags : tcpumodeflags);
            top_specialreg : (specialreg:tregister; specialflags:tspecialregflags);
        {$endif arm}
        {$if defined(arm) or defined(aarch64)}
            top_shifterop : (shifterop : pshifterop);
            top_conditioncode : (cc : TAsmCond);
        {$endif defined(arm) or defined(aarch64)}
        {$ifdef m68k}
            top_regset : (dataregset,addrregset,fpuregset:^tcpuregisterset);
        {$endif m68k}
        {$ifdef jvm}
            top_single : (sval:single);
            top_double : (dval:double);
            top_string : (pcvallen: aint; pcval: pchar);
            top_wstring : (pwstrval: pcompilerwidestring);
        {$endif jvm}
        {$ifdef llvm}
            top_single : (sval:single);
            top_double : (dval:double);
            top_undef :  ();
          {$ifdef cpuextended}
            top_extended80 : (eval:extended);
          {$endif cpuextended}
            top_tai    : (ai: tai);
            top_def    : (def: tdef);
            top_cond   : (cond: topcmp);
            top_fpcond : (fpcond: tllvmfpcmp);
            top_para   : (paras: tfplist);
        {$endif llvm}
        end;
        poper=^toper;

       { abstract assembler item }
       tai = class(TLinkedListItem)
{$ifndef NOOPT}
          { pointer to record with optimizer info about this tai object }
          optinfo  : pointer;
{$endif NOOPT}
          typ      : taitype;
          constructor Create;
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);virtual;
          procedure ppuwrite(ppufile:tcompilerppufile);virtual;
          procedure buildderefimpl;virtual;
          procedure derefimpl;virtual;
       end;

       { abstract assembler item with line information }
       tailineinfo = class(tai)
        fileinfo : tfileposinfo;
        constructor Create;
        constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
        procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;

       tai_simple = class(tai)
         constructor create(_typ : taitype);
       end;

       taiclass = class of tai;

       taiclassarray = array[taitype] of taiclass;

       { Generates an assembler string }
       tai_string = class(tailineinfo)
          str : pchar;
          { extra len so the string can contain an \0 }
          len : longint;
          constructor Create(const _str : string);
          constructor Create_pchar(_str : pchar;length : longint);
          destructor Destroy;override;
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          function getcopy:tlinkedlistitem;override;
       end;

       { Generates a common label }
       tai_symbol = class(tai)
          sym       : tasmsymbol;
          value     : puint;
          size      : longint;
          is_global,
          has_value : boolean;
          constructor Create(_sym:tasmsymbol;siz:longint);
          constructor Create_Global(_sym:tasmsymbol;siz:longint);
          constructor Createname(const _name : string;_symtyp:Tasmsymtype;siz:longint);
          constructor Createname_global(const _name : string;_symtyp:Tasmsymtype;siz:longint);
          constructor Createname_global_value(const _name : string;_symtyp:Tasmsymtype;siz:longint;val:ptruint);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure derefimpl;override;
       end;

       tai_symbol_end = class(tailineinfo)
          sym : tasmsymbol;
          constructor Create(_sym:tasmsymbol);
          constructor Createname(const _name : string);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure derefimpl;override;
       end;

       tai_directive = class(tailineinfo)
          name : ansistring;
          directive : TAsmDirective;
          constructor Create(_directive:TAsmDirective;const _name:ansistring);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;

       { Generates an assembler label }
       tai_label = class(tai)
          labsym    : tasmlabel;
{$ifdef arm}
          { set to true when the label has been moved by insertpcrelativedata to the correct location
            so one label can be used multiple times }
          moved     : boolean;
          { true, if a label has been already inserted, this is important for arm thumb where no negative
            pc relative offsets are allowed }
          inserted  : boolean;
{$endif arm}
          constructor Create(_labsym : tasmlabel);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure derefimpl;override;
       end;

       { Generates an assembler comment }
       tai_comment = class(tai)
          str : pchar;
          constructor Create(_str : pchar);
          destructor Destroy; override;
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          function getcopy:tlinkedlistitem;override;
       end;


       { Generates a section / segment directive }
       tai_section = class(tai)
          sectype  : TAsmSectiontype;
          secorder : TasmSectionorder;
          secalign : byte;
          name     : pshortstring;
          sec      : TObjSection; { used in binary writer }
          destructor Destroy;override;
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$push}{$warnings off}
         private
          { this constructor is made private on purpose }
          { because sections should be created via new_section() }
          constructor Create(Asectype:TAsmSectiontype;const Aname:string;Aalign:byte;Asecorder:TasmSectionorder=secorder_default);
{$pop}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         private
          { this constructor is made private on purpose }
          { because sections should be created via new_section() }
          constructor Create(Asectype:TAsmSectiontype;Aname:string;Aalign:byte;Asecorder:TasmSectionorder=secorder_default);
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
       end;


       { Generates an uninitializised data block }
       tai_datablock = class(tailineinfo)
          is_global : boolean;
          sym       : tasmsymbol;
          size      : asizeint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          constructor Create(const _name : string;_size : asizeint);
=======
          constructor Create(const _name : string;_size : aint);
>>>>>>> graemeg/cpstrnew
=======
          constructor Create(const _name : string;_size : aint);
>>>>>>> graemeg/cpstrnew
=======
          constructor Create(const _name : string;_size : aint);
>>>>>>> graemeg/cpstrnew
=======
          constructor Create(const _name : string;_size : aint);
>>>>>>> origin/cpstrnew
          constructor Create_global(const _name : string;_size : asizeint);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure derefimpl;override;
       end;


       { Generates an integer const }

       { tai_const }

       tai_const = class(tai)
          sym,
          endsym  : tasmsymbol;
          { if symbols and offset are provided the symofs is used,
            the value is calculated during assembling }
          symofs,
          value   : int64;
          consttype : taiconst_type;
          { we use for the 128bit int64/qword for now because I can't imagine a
            case where we need 128 bit now (FK) }
          constructor Create(_typ:taiconst_type;_value : int64);
          constructor Create_128bit(_value : int64);
          constructor Create_64bit(_value : int64);
          constructor Create_32bit(_value : longint);
          constructor Create_16bit(_value : word);
          constructor Create_64bit_unaligned(_value : int64);
          constructor Create_32bit_unaligned(_value : longint);
          constructor Create_16bit_unaligned(_value : word);
          constructor Create_8bit(_value : byte);
          constructor Create_char(size: integer; _value: dword);
          constructor Create_sleb128bit(_value : int64);
          constructor Create_uleb128bit(_value : qword);
          constructor Create_aint(_value : aint);
          constructor Create_pint(_value : pint);
          constructor Create_pint_unaligned(_value : pint);
          constructor Create_sym(_sym:tasmsymbol);
{$ifdef i8086}
          constructor Create_sym_near(_sym:tasmsymbol);
          constructor Create_sym_far(_sym:tasmsymbol);
{$endif i8086}
          constructor Create_type_sym(_typ:taiconst_type;_sym:tasmsymbol);
          constructor Create_sym_offset(_sym:tasmsymbol;ofs:asizeint);
          constructor Create_type_sym_offset(_typ:taiconst_type;_sym:tasmsymbol;ofs:asizeint);
          constructor Create_rel_sym(_typ:taiconst_type;_sym,_endsym:tasmsymbol);
          constructor Create_rel_sym_offset(_typ : taiconst_type; _sym,_endsym : tasmsymbol; _ofs : int64);
          constructor Create_rva_sym(_sym:tasmsymbol);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          constructor Createname(const name:string;ofs:asizeint);
          constructor Createname(const name:string;_symtyp:Tasmsymtype;ofs:asizeint);
          constructor Create_type_name(_typ:taiconst_type;const name:string;ofs:asizeint);
          constructor Create_type_name(_typ:taiconst_type;const name:string;_symtyp:Tasmsymtype;ofs:asizeint);
          constructor Create_nil_codeptr;
          constructor Create_nil_dataptr;
          constructor Create_int_codeptr(_value: int64);
          constructor Create_int_dataptr(_value: int64);
{$ifdef i8086}
          constructor Create_seg_name(const name:string);
          constructor Create_dgroup;
          constructor Create_fardataseg;
{$endif i8086}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          constructor Createname(const name:string;ofs:aint);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure derefimpl;override;
          function getcopy:tlinkedlistitem;override;
          function size:longint;
       end;

       { floating point const }
       tformatoptions = (fo_none,fo_hiloswapped);
       tai_realconst = class(tai)
          realtyp: tairealconsttype;
          savesize: byte;
          value: record
            case tairealconsttype of
              aitrealconst_s32bit: (s32val: ts32real);
              aitrealconst_s64bit: (s64val: ts64real);
              aitrealconst_s80bit: (s80val: ts80real);
              aitrealconst_s128bit: (s128val: ts128real);
              aitrealconst_s64comp: (s64compval: ts64comp);
          end;
{$ifdef ARM}
          formatoptions : tformatoptions;
{$endif ARM}
<<<<<<< HEAD
          constructor create_s32real(val: ts32real);
          constructor create_s64real(val: ts64real);
{$ifdef ARM}
          constructor create_s64real_hiloswapped(val : ts64real);
{$endif ARM}
          constructor create_s80real(val: ts80real; _savesize: byte);
          constructor create_s128real(val: ts128real);
          constructor create_s64compreal(val: ts64comp);
          constructor ppuload(t: taitype;ppufile: tcompilerppufile); override;
          procedure ppuwrite(ppufile: tcompilerppufile); override;
          function getcopy:tlinkedlistitem;override;
          function datasize: word;
=======
          constructor Create(_value : ts64real);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;


       { Generates an extended float (80 bit real) }
       tai_real_80bit = class(tai)
          value : ts80real;
          savesize : byte;
          constructor Create(_value : ts80real; _savesize: byte);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;


       { Generates an float128 (128 bit real) }
       tai_real_128bit = class(tai)
          value : ts128real;
          constructor Create(_value : ts128real);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
>>>>>>> graemeg/cpstrnew
       end;

       { tai_stab }

       tai_stab = class(tai)
          str : pchar;
          stabtype : TStabType;
          constructor Create(_stabtype:TStabType;_str : pchar);
          constructor Create_str(_stabtype:TStabType;const s:string);
          constructor create_ansistr(_stabtype: TStabType; const s: ansistring);
          destructor Destroy;override;
       end;

       tai_force_line = class(tailineinfo)
          constructor Create;
       end;

       tai_function_name = class(tai)
          funcname : pshortstring;
          constructor create(const s:string);
          destructor destroy;override;
       end;

       { Insert a cut to split assembler into several smaller files }
       tai_cutobject = class(tai)
          place : tcutplace;
          constructor Create;
          constructor Create_begin;
          constructor Create_end;
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;

       { Insert a marker for assembler and inline blocks }
       tai_marker = class(tai)
          Kind: TAsmMarker;
          Constructor Create(_Kind: TAsmMarker);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;

       tai_tempalloc = class(tai)
          allocation : boolean;
{$ifdef EXTDEBUG}
          problem : pshortstring;
{$endif EXTDEBUG}
          temppos,
          tempsize   : longint;
          constructor alloc(pos,size:longint);
          constructor dealloc(pos,size:longint);
{$ifdef EXTDEBUG}
          constructor allocinfo(pos,size:longint;const st:string);
{$endif EXTDEBUG}
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          destructor destroy;override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;

       tai_regalloc = class(tai)
          reg     : tregister;
          ratype  : TRegAllocType;
          { tells BuildLabelTableAndFixRegAlloc that the deallocation should be kept }
          keep    : boolean;
          { reg(de)alloc belongs to this instruction, this
            is only used for automatic inserted (de)alloc for
            imaginary register and required for spilling code }
          instr   : tai;
          constructor alloc(r : tregister;ainstr:tai);
          constructor dealloc(r : tregister;ainstr:tai);
          constructor sync(r : tregister);
          constructor resize(r : tregister);
<<<<<<< HEAD
<<<<<<< HEAD
          constructor markused(r : tregister);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
       end;

       tadd_reg_instruction_proc=procedure(instr:Tai;r:tregister) of object;

        { Class template for assembler instructions
        }
        tai_cpu_abstract = class(tailineinfo)
        protected
           procedure ppuloadoper(ppufile:tcompilerppufile;var o:toper);virtual;
           procedure ppuwriteoper(ppufile:tcompilerppufile;const o:toper);virtual;
           procedure ppubuildderefimploper(var o:toper);virtual;abstract;
           procedure ppuderefoper(var o:toper);virtual;abstract;
        public
           { Condition flags for instruction }
           condition : TAsmCond;
           { Number of operands to instruction }
           ops       : byte;
           { Number of allocate oper structures }
           opercnt   : byte;
           { Operands of instruction }
           oper      : array[0..max_operands-1] of poper;
           { Actual opcode of instruction }
           opcode    : tasmop;
{$ifdef x86}
           segprefix : tregister;
{$endif x86}
           { true if instruction is a jmp }
           is_jmp    : boolean; { is this instruction a jump? (needed for optimizer) }
           Constructor Create(op : tasmop);virtual;
           Destructor Destroy;override;
           function getcopy:TLinkedListItem;override;
           constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
           procedure ppuwrite(ppufile:tcompilerppufile);override;
           procedure buildderefimpl;override;
           procedure derefimpl;override;
           procedure SetCondition(const c:TAsmCond);
           procedure allocate_oper(opers:longint);
           procedure loadconst(opidx:longint;l:aint);
           procedure loadsymbol(opidx:longint;s:tasmsymbol;sofs:longint);
           procedure loadlocal(opidx:longint;s:pointer;sofs:longint;indexreg:tregister;scale:byte;getoffset,forceref:boolean);
           procedure loadref(opidx:longint;const r:treference);
           procedure loadreg(opidx:longint;r:tregister);
           procedure loadoper(opidx:longint;o:toper); virtual;
           procedure clearop(opidx:longint); virtual;
           procedure freeop(opidx:longint);
           { register allocator }
           function is_same_reg_move(regtype: Tregistertype):boolean;virtual;
           function spilling_get_operation_type(opnr: longint): topertype;virtual;
           function spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;virtual;

           function  Pass1(objdata:TObjData):longint;virtual;
           procedure Pass2(objdata:TObjData);virtual;

           procedure resetpass1; virtual;
           procedure resetpass2; virtual;
        end;
        tai_cpu_class = class of tai_cpu_abstract;

        { Buffer type used for alignment }
        tfillbuffer = array[0..63] of char;

        { alignment for operator }
        tai_align_abstract = class(tai)
           aligntype : byte;   { 1 = no align, 2 = word align, 4 = dword align }
           fillsize  : byte;   { real size to fill }
           fillop    : byte;   { value to fill with - optional }
           use_op    : boolean;
           constructor Create(b:byte);virtual;
           constructor Create_op(b: byte; _op: byte);virtual;
           constructor Create_zeros(b:byte);
           constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
           procedure ppuwrite(ppufile:tcompilerppufile);override;
           function calculatefillbuf(var buf : tfillbuffer;executable : boolean):pchar;virtual;
        end;
        tai_align_class = class of tai_align_abstract;

        tai_varloc = class(tai)
           oldlocation,
           oldlocationhi,
           newlocation,
           newlocationhi : tregister;
           varsym : tsym;
           constructor create(sym : tsym;loc : tregister);
           constructor create64(sym : tsym;loc,lochi : tregister);
{$ifdef cpu64bitalu}
           constructor create128(sym : tsym;loc,lochi : tregister);
{$endif cpu64bitalu}
           constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
           procedure ppuwrite(ppufile:tcompilerppufile);override;
           procedure buildderefimpl;override;
           procedure derefimpl;override;
        end;

        TSehDirectiveDatatype=(sd_none,sd_string,sd_reg,sd_offset,sd_regoffset);

        TSehDirectiveData=record
        case typ: TSehDirectiveDatatype of
          sd_none: ();
          sd_string: (name:pshortstring;flags:byte);
          sd_reg,sd_offset,sd_regoffset: (reg:TRegister;offset:dword);
        end;

        tai_seh_directive = class(tai)
          kind: TAsmSehDirective;
          data: TSehDirectiveData;
          constructor create(_kind:TAsmSehDirective);
          constructor create_name(_kind:TAsmSehDirective;const _name: string);
          constructor create_reg(_kind:TAsmSehDirective;r:TRegister);
          constructor create_offset(_kind:TAsmSehDirective;ofs:dword);
          constructor create_reg_offset(_kind:TAsmSehDirective;r:TRegister;ofs:dword);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          destructor destroy;override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure generate_code(objdata:TObjData);virtual;
          property datatype: TSehDirectiveDatatype read data.typ;
        end;
        tai_seh_directive_class=class of tai_seh_directive;

{$ifdef JVM}
        { JVM variable live range description }
        tai_jvar = class(tai)
          stackslot: longint;
          desc: pshortstring;
          startlab,stoplab: tasmsymbol;

          constructor Create(_stackslot: longint; const _desc: shortstring; _startlab, _stoplab: TAsmSymbol);
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          destructor destroy;override;
        end;
        tai_jvar_class = class of tai_jvar;

        { JVM exception catch description }
        tai_jcatch = class(tai)
          name: pshortstring;
          startlab,stoplab,handlerlab: tasmsymbol;

          constructor Create(const _name: shortstring; _startlab, _stoplab, _handlerlab: TAsmSymbol);
          destructor destroy;override;
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
        end;
        tai_jcatch_class = class of tai_jcatch;
{$endif JVM}

        tai_symbolpair = class(tai)
          kind: TSymbolPairKind;
          sym,
          value: pshortstring;
          constructor create(akind: TSymbolPairKind; const asym, avalue: string);
          destructor destroy;override;
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
        end;

        tai_weak = class(tai)
          sym: pshortstring;
          constructor create(const asym: string);
          destructor destroy;override;
          constructor ppuload(t:taitype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
        end;

    var
      { array with all class types for tais }
      aiclass : taiclassarray;

      { target specific tais, possibly overwritten in target specific aasmcpu }
      cai_align : tai_align_class = tai_align_abstract;
      cai_cpu   : tai_cpu_class = tai_cpu_abstract;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      cai_seh_directive: tai_seh_directive_class = tai_seh_directive;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

      { hook to notify uses of registers }
      add_reg_instruction_hook : tadd_reg_instruction_proc;

    procedure maybe_new_object_file(list:TAsmList);
    procedure new_section(list:TAsmList;Asectype:TAsmSectiontype;const Aname:string;Aalign:byte;Asecorder:TasmSectionorder=secorder_default);
    procedure section_symbol_start(list:TAsmList;const Aname:string;Asymtyp:Tasmsymtype;
                                   Aglobal:boolean;Asectype:TAsmSectiontype;Aalign:byte);
    procedure section_symbol_end(list:TAsmList;const Aname:string);

    function ppuloadai(ppufile:tcompilerppufile):tai;
    procedure ppuwriteai(ppufile:tcompilerppufile;n:tai);


implementation

    uses
      SysUtils,
      verbose,
      globals;

    const
      pputaimarker = 254;


{****************************************************************************
                                 Helpers
 ****************************************************************************}

    procedure maybe_new_object_file(list:TAsmList);
      begin
        if create_smartlink_library then
          list.concat(tai_cutobject.create);
      end;


    procedure new_section(list:TAsmList;Asectype:TAsmSectiontype;const Aname:string;Aalign:byte;Asecorder:TasmSectionorder=secorder_default);
      begin
        list.concat(tai_section.create(Asectype,Aname,Aalign,Asecorder));
        list.concat(cai_align.create(Aalign));
      end;


    procedure section_symbol_start(list:TAsmList;const Aname:string;Asymtyp:Tasmsymtype;
                                   Aglobal:boolean;Asectype:TAsmSectiontype;Aalign:byte);
      begin
        maybe_new_object_file(list);
        new_section(list,Asectype,Aname,Aalign);
        if Aglobal or
           create_smartlink then
          list.concat(tai_symbol.createname_global(Aname,Asymtyp,0))
        else
          list.concat(tai_symbol.createname(Aname,Asymtyp,0));
      end;


    procedure section_symbol_end(list:TAsmList;const Aname:string);
      begin
        list.concat(tai_symbol_end.createname(Aname));
      end;


    function ppuloadai(ppufile:tcompilerppufile):tai;
      var
        b : byte;
        t : taitype;
      begin
        { marker }
        b:=ppufile.getbyte;
        if b<>pputaimarker then
          internalerror(200208181);
        { load nodetype }
        t:=taitype(ppufile.getbyte);
        if t<>ait_none then
         begin
           if t>high(taitype) then
             internalerror(200208182);
           if not assigned(aiclass[t]) then
             internalerror(200208183);
           {writeln('taiload: ',taitypestr[t]);}
           { generate tai of the correct class }
           ppuloadai:=aiclass[t].ppuload(t,ppufile);
         end
        else
         ppuloadai:=nil;
      end;


    procedure ppuwriteai(ppufile:tcompilerppufile;n:tai);
      begin
        { marker, read by ppuloadnode }
        ppufile.putbyte(pputaimarker);
        if assigned(n) then
         begin
           { type, read by ppuloadnode }
           ppufile.putbyte(byte(n.typ));
           {writeln('taiwrite: ',taitypestr[n.typ]);}
           n.ppuwrite(ppufile);
         end
        else
         ppufile.putbyte(byte(ait_none));
      end;


    constructor tai_weak.create(const asym: string);
      begin
        inherited create;
        typ:=ait_weak;
        sym:=stringdup(asym);
      end;

    destructor tai_weak.destroy;
      begin
        stringdispose(sym);
        inherited destroy;
      end;

    constructor tai_weak.ppuload(t: taitype; ppufile: tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        sym:=stringdup(ppufile.getstring);
      end;

    procedure tai_weak.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putstring(sym^);
      end;

    constructor tai_symbolpair.create(akind: TSymbolPairKind; const asym, avalue: string);
      begin
        inherited create;
        kind:=akind;
        typ:=ait_symbolpair;
        sym:=stringdup(asym);
        value:=stringdup(avalue);
      end;

    destructor tai_symbolpair.destroy;
      begin
        stringdispose(sym);
        stringdispose(value);
        inherited destroy;
      end;

    constructor tai_symbolpair.ppuload(t: taitype; ppufile: tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        kind:=TSymbolPairKind(ppufile.getbyte);;
        sym:=stringdup(ppufile.getstring);
        value:=stringdup(ppufile.getstring);
      end;

    procedure tai_symbolpair.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(byte(kind));
        ppufile.putstring(sym^);
        ppufile.putstring(value^);
      end;


    constructor tai_varloc.create(sym: tsym; loc: tregister);
      begin
        inherited Create;
        typ:=ait_varloc;
        newlocation:=loc;
        newlocationhi:=NR_NO;
        varsym:=sym;
        oldlocationhi:=NR_NO;
      end;


    constructor tai_varloc.create64(sym: tsym; loc, lochi: tregister);
      begin
        inherited Create;
        typ:=ait_varloc;
        newlocation:=loc;
        newlocationhi:=lochi;
        varsym:=sym;
      end;


{$ifdef cpu64bitalu}
    constructor tai_varloc.create128(sym: tsym; loc, lochi: tregister);
      begin
        inherited Create;
        typ:=ait_varloc;
        newlocation:=loc;
        newlocationhi:=lochi;
        varsym:=sym;
      end;
{$endif cpu64bitalu}


    constructor tai_varloc.ppuload(t: taitype; ppufile: tcompilerppufile);
      begin
        inherited ppuload(t, ppufile);
      end;


    procedure tai_varloc.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
      end;


    procedure tai_varloc.buildderefimpl;
      begin
        inherited buildderefimpl;
      end;


    procedure tai_varloc.derefimpl;
      begin
        inherited derefimpl;
      end;


{****************************************************************************
                             TAI
 ****************************************************************************}

    constructor tai.Create;
      begin
{$ifndef NOOPT}
        optinfo:=nil;
{$endif NOOPT}
      end;


    constructor tai.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        typ:=t;
{$ifndef NOOPT}
        optinfo:=nil;
{$endif}
      end;


    procedure tai.ppuwrite(ppufile:tcompilerppufile);
      begin
      end;


    procedure tai.buildderefimpl;
      begin
      end;


    procedure tai.derefimpl;
      begin
      end;


{****************************************************************************
                              TAILINEINFO
 ****************************************************************************}

    constructor tailineinfo.create;
     begin
       inherited create;
       fileinfo:=current_filepos;
     end;


    constructor tailineinfo.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getposinfo(fileinfo);
      end;


    procedure tailineinfo.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putposinfo(fileinfo);
      end;


{****************************************************************************
                              TAI_SIMPLE
 ****************************************************************************}

    constructor tai_simple.create(_typ : taitype);
      begin
        inherited create;
        typ:=_typ;
      end;


{****************************************************************************
                             TAI_SECTION
 ****************************************************************************}

    constructor tai_section.Create(Asectype:TAsmSectiontype;const Aname:string;Aalign:byte;Asecorder:TasmSectionorder=secorder_default);
      begin
        inherited Create;
        typ:=ait_section;
        sectype:=asectype;
        secalign:=Aalign;
        secorder:=Asecorder;
        name:=stringdup(Aname);
        sec:=nil;
      end;


    constructor tai_section.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        sectype:=TAsmSectiontype(ppufile.getbyte);
        secalign:=ppufile.getbyte;
        name:=stringdup(ppufile.getstring);
        sec:=nil;
      end;


    destructor tai_section.Destroy;
      begin
        stringdispose(name);
      end;


    procedure tai_section.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(byte(sectype));
        ppufile.putbyte(secalign);
        ppufile.putstring(name^);
      end;


{****************************************************************************
                             TAI_DATABLOCK
 ****************************************************************************}

    constructor tai_datablock.Create(const _name : string;_size : asizeint);

      begin
         inherited Create;
         typ:=ait_datablock;
         sym:=current_asmdata.DefineAsmSymbol(_name,AB_LOCAL,AT_DATA);
         { keep things aligned }
         if _size<=0 then
           _size:=sizeof(aint);
         size:=_size;
         is_global:=false;
      end;


    constructor tai_datablock.Create_global(const _name : string;_size : asizeint);
      begin
         inherited Create;
         typ:=ait_datablock;
         sym:=current_asmdata.DefineAsmSymbol(_name,AB_GLOBAL,AT_DATA);
         { keep things aligned }
         if _size<=0 then
           _size:=sizeof(aint);
         size:=_size;
         is_global:=true;
      end;


    constructor tai_datablock.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited Create;
        sym:=ppufile.getasmsymbol;
        size:=ppufile.getaint;
        is_global:=boolean(ppufile.getbyte);
      end;


    procedure tai_datablock.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putasmsymbol(sym);
        ppufile.putaint(size);
        ppufile.putbyte(byte(is_global));
      end;


    procedure tai_datablock.derefimpl;
      begin
      end;


{****************************************************************************
                               TAI_SYMBOL
 ****************************************************************************}

    constructor tai_symbol.Create(_sym:tasmsymbol;siz:longint);
      begin
         inherited Create;
         typ:=ait_symbol;
         sym:=_sym;
         size:=siz;
         { don't redefine global/external symbols as local, as code to access
           such symbols is different on some platforms }
         if not(sym.bind in [AB_NONE,AB_LOCAL]) then
           internalerror(2013081601);
         sym.bind:=AB_LOCAL;
         is_global:=false;
      end;


    constructor tai_symbol.Create_global(_sym:tasmsymbol;siz:longint);
      begin
         inherited Create;
         typ:=ait_symbol;
         sym:=_sym;
         size:=siz;
         { don't override PRIVATE_EXTERN with GLOBAL }
         if not(sym.bind in [AB_GLOBAL,AB_PRIVATE_EXTERN]) then
           sym.bind:=AB_GLOBAL;
         is_global:=true;
      end;


    constructor tai_symbol.Createname(const _name : string;_symtyp:Tasmsymtype;siz:longint);
      begin
         inherited Create;
         typ:=ait_symbol;
         sym:=current_asmdata.DefineAsmSymbol(_name,AB_LOCAL,_symtyp);
         size:=siz;
         is_global:=false;
      end;


    constructor tai_symbol.Createname_global(const _name : string;_symtyp:Tasmsymtype;siz:longint);
      begin
         inherited Create;
         typ:=ait_symbol;
         sym:=current_asmdata.DefineAsmSymbol(_name,AB_GLOBAL,_symtyp);
         size:=siz;
         is_global:=true;
      end;


    constructor tai_symbol.createname_global_value(const _name: string;_symtyp: tasmsymtype; siz: longint; val: ptruint);
      begin
        Createname_global(_name,_symtyp,siz);
        value:=val;
        has_value:=true;
      end;


    constructor tai_symbol.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        sym:=ppufile.getasmsymbol;
        size:=ppufile.getlongint;
        is_global:=boolean(ppufile.getbyte);
      end;


    procedure tai_symbol.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putasmsymbol(sym);
        ppufile.putlongint(size);
        ppufile.putbyte(byte(is_global));
      end;


    procedure tai_symbol.derefimpl;
      begin
      end;


{****************************************************************************
                               TAI_SYMBOL_END
 ****************************************************************************}

    constructor tai_symbol_end.Create(_sym:tasmsymbol);
      begin
         inherited Create;
         typ:=ait_symbol_end;
         sym:=_sym;
      end;


    constructor tai_symbol_end.Createname(const _name : string);
      begin
         inherited Create;
         typ:=ait_symbol_end;
         sym:=current_asmdata.GetAsmSymbol(_name);
         if not assigned(sym) then
           internalerror(2013080301);
      end;


    constructor tai_symbol_end.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        sym:=ppufile.getasmsymbol;
      end;


    procedure tai_symbol_end.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putasmsymbol(sym);
      end;


    procedure tai_symbol_end.derefimpl;
      begin
      end;


{****************************************************************************
                               TAI_SYMBOL_END
 ****************************************************************************}

    constructor tai_directive.Create(_directive:TAsmDirective;const _name:ansistring);
      begin
         inherited Create;
         typ:=ait_directive;
         name:=_name;
         directive:=_directive;
      end;


    constructor tai_directive.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        name:=ppufile.getansistring;
        directive:=TAsmDirective(ppufile.getbyte);
      end;


    procedure tai_directive.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putansistring(name);
        ppufile.putbyte(byte(directive));
      end;


{****************************************************************************
                               TAI_CONST
 ****************************************************************************}

    constructor tai_const.Create(_typ:taiconst_type;_value : int64);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=_typ;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_128bit(_value : int64);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_128bit;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_64bit(_value : int64);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_64bit;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_32bit(_value : longint);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_32bit;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_16bit(_value : word);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_16bit;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;

    constructor tai_const.Create_64bit_unaligned(_value : int64);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_64bit_unaligned;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_32bit_unaligned(_value : longint);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_32bit_unaligned;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_16bit_unaligned(_value : word);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_16bit_unaligned;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_8bit(_value : byte);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_8bit;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_char(size: integer; _value: dword);
      begin
         inherited Create;
         typ:=ait_const;
         case size of
            1:
              begin
                consttype:=aitconst_8bit;
                value:=byte(_value)
              end;
             2:
               begin
                 consttype:=aitconst_16bit;
                 value:=word(_value)
               end
             else
               InternalError(2010030701)
         end
      end;


    constructor tai_const.Create_sleb128bit(_value : int64);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_sleb128bit;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_uleb128bit(_value : qword);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_uleb128bit;
         value:=int64(_value);
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_aint(_value : aint);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_aint;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_pint(_value : pint);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_ptr;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_pint_unaligned(_value: pint);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=aitconst_ptr_unaligned;
         value:=_value;
         sym:=nil;
         endsym:=nil;
      end;


    constructor tai_const.Create_type_sym(_typ:taiconst_type;_sym:tasmsymbol);
      begin
         inherited Create;
         typ:=ait_const;
         consttype:=_typ;
         sym:=_sym;
         endsym:=nil;
         value:=0;
         { update sym info }
         if assigned(sym) then
           sym.increfs;
      end;


    constructor tai_const.Create_sym(_sym:tasmsymbol);
      begin
         self.create_sym_offset(_sym,0);
<<<<<<< HEAD
<<<<<<< HEAD
      end;


{$ifdef i8086}
    constructor tai_const.Create_sym_near(_sym: tasmsymbol);
      begin
         self.create_sym(_sym);
         consttype:=aitconst_ptr;
      end;

    constructor tai_const.Create_sym_far(_sym: tasmsymbol);
      begin
        self.create_sym(_sym);
        consttype:=aitconst_farptr;
      end;
{$endif i8086}


    constructor tai_const.Create_sym_offset(_sym:tasmsymbol;ofs:asizeint);
      begin
         inherited Create;
         typ:=ait_const;
{$ifdef i8086}
         if assigned(_sym) and (_sym.typ=AT_DATA) then
           begin
             if current_settings.x86memorymodel in x86_far_data_models then
               consttype:=aitconst_farptr
             else
               consttype:=aitconst_ptr;
           end
         else
           begin
             if current_settings.x86memorymodel in x86_far_code_models then
               consttype:=aitconst_farptr
             else
               consttype:=aitconst_ptr;
           end;
{$else i8086}
{$ifdef avr}
         if assigned(_sym) and (_sym.typ=AT_FUNCTION) then
           consttype:=aitconst_gs
         else
{$endif avr}
         consttype:=aitconst_ptr;
{$endif i8086}
         { sym is allowed to be nil, this is used to write nil pointers }
         sym:=_sym;
         endsym:=nil;
         { store the original offset in symofs so that we can recalculate the
           value field in the assembler }
         symofs:=ofs;
         value:=ofs;
         { update sym info }
         if assigned(sym) then
           sym.increfs;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    constructor tai_const.Create_type_sym_offset(_typ : taiconst_type;_sym : tasmsymbol; ofs : asizeint);
      begin
         inherited Create;
         typ:=ait_const;
<<<<<<< HEAD
         consttype:=_typ;
=======
         consttype:=aitconst_ptr;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
         { sym is allowed to be nil, this is used to write nil pointers }
         sym:=_sym;
         endsym:=nil;
         { store the original offset in symofs so that we can recalculate the
           value field in the assembler }
         symofs:=ofs;
         value:=ofs;
         { update sym info }
         if assigned(sym) then
           sym.increfs;
      end;


    constructor tai_const.Create_rel_sym(_typ:taiconst_type;_sym,_endsym:tasmsymbol);
      begin
         self.create_sym_offset(_sym,0);
         consttype:=_typ;
         endsym:=_endsym;
         endsym.increfs;
      end;


    constructor tai_const.Create_rel_sym_offset(_typ: taiconst_type; _sym,_endsym: tasmsymbol; _ofs: int64);
       begin
         self.create_sym_offset(_sym,_ofs);
         consttype:=_typ;
         endsym:=_endsym;
         endsym.increfs;
       end;


    constructor tai_const.Create_rva_sym(_sym:tasmsymbol);
      begin
         self.create_sym_offset(_sym,0);
         consttype:=aitconst_rva_symbol;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    constructor tai_const.Createname(const name:string;ofs:asizeint);
      begin
<<<<<<< HEAD
<<<<<<< HEAD
         self.Createname(name,AT_NONE,ofs);
=======
         self.create_sym_offset(_sym,0);
         consttype:=aitconst_indirect_symbol;
>>>>>>> graemeg/fixes_2_2
=======
         self.create_sym_offset(_sym,0);
         consttype:=aitconst_indirect_symbol;
>>>>>>> origin/fixes_2_2
      end;


    constructor tai_const.Createname(const name:string;_symtyp:Tasmsymtype;ofs:asizeint);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    constructor tai_const.Createname(const name:string;ofs:aint);
>>>>>>> graemeg/cpstrnew
      begin
<<<<<<< HEAD
<<<<<<< HEAD
         self.create_sym_offset(current_asmdata.RefAsmSymbol(name,_symtyp),ofs);
      end;


    constructor tai_const.Create_type_name(_typ:taiconst_type;const name:string;ofs:asizeint);
      begin
         self.Create_type_name(_typ,name,AT_NONE,ofs);
      end;


    constructor tai_const.Create_type_name(_typ:taiconst_type;const name:string;_symtyp:Tasmsymtype;ofs:asizeint);
      begin
         self.create_sym_offset(current_asmdata.RefAsmSymbol(name,_symtyp),ofs);
         consttype:=_typ;
      end;


    constructor tai_const.Create_nil_codeptr;
      begin
        self.Create_int_codeptr(0);
      end;


    constructor tai_const.Create_nil_dataptr;
      begin
        self.Create_int_dataptr(0);
=======
         self.create_sym_offset(current_asmdata.RefAsmSymbol(name),ofs);
>>>>>>> graemeg/fixes_2_2
      end;


    constructor tai_const.Create_int_codeptr(_value: int64);
      begin
        inherited Create;
        typ:=ait_const;
{$ifdef i8086}
        if current_settings.x86memorymodel in x86_far_code_models then
          consttype:=aitconst_farptr
        else
{$endif i8086}
{$ifdef avr}
          consttype:=aitconst_gs;
{$else avr}
          consttype:=aitconst_ptr;
{$endif avr}
        sym:=nil;
        endsym:=nil;
        symofs:=0;
        value:=_value;
      end;


    constructor tai_const.Create_int_dataptr(_value: int64);
      begin
        inherited Create;
        typ:=ait_const;
{$ifdef i8086}
        if current_settings.x86memorymodel in x86_far_data_models then
          consttype:=aitconst_farptr
        else
{$endif i8086}
          consttype:=aitconst_ptr;
        sym:=nil;
        endsym:=nil;
        symofs:=0;
        value:=_value;
      end;


{$ifdef i8086}
    constructor tai_const.Create_seg_name(const name:string);
      begin
        self.Createname(name,0);
        self.consttype:=aitconst_seg;
=======
         self.create_sym_offset(current_asmdata.RefAsmSymbol(name),ofs);
>>>>>>> origin/fixes_2_2
      end;


    constructor tai_const.Create_dgroup;
      begin
        self.Create_16bit(0);
        self.consttype:=aitconst_dgroup;
      end;


    constructor tai_const.Create_fardataseg;
      begin
        self.Create_16bit(0);
        self.consttype:=aitconst_fardataseg;
      end;
{$endif i8086}


    constructor tai_const.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        consttype:=taiconst_type(ppufile.getbyte);
        sym:=ppufile.getasmsymbol;
        endsym:=ppufile.getasmsymbol;
        value:=ppufile.getint64;
      end;


    procedure tai_const.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(byte(consttype));
        ppufile.putasmsymbol(sym);
        ppufile.putasmsymbol(endsym);
        ppufile.putint64(value);
      end;


    procedure tai_const.derefimpl;
      begin
      end;


    function tai_const.getcopy:tlinkedlistitem;
      begin
        getcopy:=inherited getcopy;
        { we need to increase the reference number }
        if assigned(sym) then
          sym.increfs;
        if assigned(endsym) then
          endsym.increfs;
      end;


    function tai_const.size:longint;
      begin
        case consttype of
          aitconst_8bit :
            result:=1;
          aitconst_16bit,aitconst_16bit_unaligned :
            result:=2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          aitconst_32bit,aitconst_darwin_dwarf_delta32,
	  aitconst_32bit_unaligned:
            result:=4;
          aitconst_64bit,aitconst_darwin_dwarf_delta64,
	  aitconst_64bit_unaligned:
=======
          aitconst_32bit,aitconst_darwin_dwarf_delta32:
            result:=4;
          aitconst_64bit,aitconst_darwin_dwarf_delta64:
>>>>>>> graemeg/cpstrnew
=======
          aitconst_32bit,aitconst_darwin_dwarf_delta32:
            result:=4;
          aitconst_64bit,aitconst_darwin_dwarf_delta64:
>>>>>>> graemeg/cpstrnew
=======
          aitconst_32bit,aitconst_darwin_dwarf_delta32:
            result:=4;
          aitconst_64bit,aitconst_darwin_dwarf_delta64:
>>>>>>> graemeg/cpstrnew
=======
          aitconst_32bit,aitconst_darwin_dwarf_delta32:
            result:=4;
          aitconst_64bit,aitconst_darwin_dwarf_delta64:
>>>>>>> origin/cpstrnew
            result:=8;
<<<<<<< HEAD
=======
          aitconst_indirect_symbol,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          aitconst_secrel32_symbol,
          aitconst_rva_symbol :
            if target_info.system=system_x86_64_win64 then
              result:=sizeof(longint)
            else
              result:=sizeof(pint);
          aitconst_uleb128bit :
            result:=LengthUleb128(qword(value));
          aitconst_sleb128bit :
            result:=LengthSleb128(value);
          aitconst_half16bit,
          aitconst_gs:
            result:=2;
          aitconst_farptr:
            result:=4;
          aitconst_dgroup,
          aitconst_fardataseg,
          aitconst_seg:
            result:=2;
          aitconst_got:
            result:=sizeof(pint);
          aitconst_gotoff_symbol:
            result:=4;
          else
            internalerror(200603253);
        end;
      end;


{****************************************************************************
                               TAI_realconst
 ****************************************************************************}

    constructor tai_realconst.create_s32real(val: ts32real);
      begin
        inherited create;
        typ:=ait_realconst;
        realtyp:=aitrealconst_s32bit;
        savesize:=4;
        value.s32val:=val;
      end;


    constructor tai_realconst.create_s64real(val: ts64real);
      begin
        inherited create;
        typ:=ait_realconst;
        realtyp:=aitrealconst_s64bit;
        savesize:=8;
        value.s64val:=val;
      end;

{$ifdef ARM}
    constructor tai_realconst.create_s64real_hiloswapped(val : ts64real);
      begin
        inherited create;
        typ:=ait_realconst;
        realtyp:=aitrealconst_s64bit;
        value.s64val:=val;
        savesize:=8;
        formatoptions:=fo_hiloswapped;
      end;

{$endif ARM}
<<<<<<< HEAD
=======
      end;


{****************************************************************************
                               TAI_real_80bit
 ****************************************************************************}

    constructor tai_real_80bit.Create(_value : ts80real; _savesize: byte);

      begin
         inherited Create;
         typ:=ait_real_80bit;
         value:=_value;
         savesize:=_savesize;
      end;

>>>>>>> graemeg/cpstrnew

    constructor tai_realconst.create_s80real(val: ts80real; _savesize: byte);
      begin
<<<<<<< HEAD
        inherited create;
        typ:=ait_realconst;
        realtyp:=aitrealconst_s80bit;
        savesize:=_savesize;
        value.s80val:=val;
=======
        inherited ppuload(t,ppufile);
        value:=ppufile.getreal;
        savesize:=ppufile.getbyte;
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
      end;


    constructor tai_realconst.create_s128real(val: ts128real);
      begin
<<<<<<< HEAD
        inherited create;
        typ:=ait_realconst;
        realtyp:=aitrealconst_s128bit;
        savesize:=16;
        value.s128val:=val;
=======
        inherited ppuwrite(ppufile);
        ppufile.putreal(value);
        ppufile.putbyte(savesize);
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
      end;


    constructor tai_realconst.create_s64compreal(val: ts64comp);
      begin
        inherited create;
        typ:=ait_realconst;
        realtyp:=aitrealconst_s64comp;
        savesize:=8;
        value.s64compval:=val;
      end;


        constructor tai_realconst.ppuload(t: taitype; ppufile: tcompilerppufile);
      begin
        inherited;
        realtyp:=tairealconsttype(ppufile.getbyte);
{$ifdef ARM}
        formatoptions:=tformatoptions(ppufile.getbyte);
{$endif ARM}
        case realtyp of
          aitrealconst_s32bit:
            value.s32val:=ppufile.getreal;
          aitrealconst_s64bit:
            value.s64val:=ppufile.getreal;
          aitrealconst_s80bit:
            value.s80val:=ppufile.getreal;
          aitrealconst_s128bit:
            value.s128val:=ppufile.getreal;
          aitrealconst_s64comp:
            value.s64compval:=comp(ppufile.getint64);
          else
            internalerror(2014050602);
        end;
      end;


    procedure tai_realconst.ppuwrite(ppufile: tcompilerppufile);
      var
        c: comp;
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(byte(realtyp));
{$ifdef ARM}
        ppufile.putbyte(byte(formatoptions));
{$endif ARM}
        case realtyp of
          aitrealconst_s32bit:
            ppufile.putreal(value.s32val);
          aitrealconst_s64bit:
            ppufile.putreal(value.s64val);
          aitrealconst_s80bit:
            ppufile.putreal(value.s80val);
          aitrealconst_s128bit:
            ppufile.putreal(value.s128val);
          aitrealconst_s64comp:
            begin
              c:=comp(value.s64compval);
              ppufile.putint64(int64(c));
            end
          else
            internalerror(2014050601);
        end;
      end;


    function tai_realconst.getcopy: tlinkedlistitem;
      begin
        result:=inherited getcopy;
        tai_realconst(result).value:=value;
        tai_realconst(result).realtyp:=realtyp;
        tai_realconst(result).savesize:=savesize;
{$ifdef ARM}
        tai_realconst(result).formatoptions:=formatoptions;
{$endif ARM}
      end;


    function tai_realconst.datasize: word;
      begin
        case realtyp of
          aitrealconst_s32bit:
            result:=4;
          aitrealconst_s64bit,
          aitrealconst_s64comp:
            result:=8;
          aitrealconst_s80bit:
            result:=10;
          aitrealconst_s128bit:
            result:=16;
          else
            internalerror(2014050603);
        end;
      end;


{****************************************************************************
                               TAI_STRING
 ****************************************************************************}

     constructor tai_string.Create(const _str : string);
       begin
          inherited Create;
          typ:=ait_string;
          len:=length(_str);
          getmem(str,len+1);
          move(_str[1],str^,len);
          str[len]:=#0;
       end;


    constructor tai_string.Create_pchar(_str : pchar;length : longint);
       begin
          inherited Create;
          typ:=ait_string;
          str:=_str;
          len:=length;
       end;


    destructor tai_string.destroy;
      begin
         if str<>nil then
           freemem(str);
         inherited Destroy;
      end;


    constructor tai_string.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        len:=ppufile.getlongint;
        getmem(str,len+1);
        ppufile.getdata(str^,len);
        str[len]:=#0
      end;


    procedure tai_string.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putlongint(len);
        ppufile.putdata(str^,len);
      end;


    function tai_string.getcopy : tlinkedlistitem;
      var
        p : tlinkedlistitem;
      begin
        p:=inherited getcopy;
        getmem(tai_string(p).str,len);
        move(str^,tai_string(p).str^,len);
        getcopy:=p;
      end;


{****************************************************************************
                               TAI_LABEL
 ****************************************************************************}

    constructor tai_label.Create(_labsym : tasmlabel);
      begin
        inherited Create;
        typ:=ait_label;
        labsym:=_labsym;
        labsym.is_set:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
        is_global:=(labsym.bind in [AB_GLOBAL,AB_PRIVATE_EXTERN]);
>>>>>>> graemeg/cpstrnew
=======
        is_global:=(labsym.bind in [AB_GLOBAL,AB_PRIVATE_EXTERN]);
>>>>>>> graemeg/cpstrnew
=======
        is_global:=(labsym.bind in [AB_GLOBAL,AB_PRIVATE_EXTERN]);
>>>>>>> graemeg/cpstrnew
=======
        is_global:=(labsym.bind in [AB_GLOBAL,AB_PRIVATE_EXTERN]);
>>>>>>> origin/cpstrnew
      end;


    constructor tai_label.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        labsym:=tasmlabel(ppufile.getasmsymbol);
        ppufile.getbyte; { was is_global flag, now unused }
      end;


    procedure tai_label.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putasmsymbol(labsym);
        ppufile.putbyte(0); { was is_global flag, now unused }
      end;


    procedure tai_label.derefimpl;
      begin
        labsym.is_set:=true;
      end;

{****************************************************************************
          tai_comment  comment to be inserted in the assembler file
 ****************************************************************************}

     constructor tai_comment.Create(_str : pchar);

       begin
          inherited Create;
          typ:=ait_comment;
          str:=_str;
       end;

    destructor tai_comment.destroy;

      begin
         freemem(str);
         inherited Destroy;
      end;

    constructor tai_comment.ppuload(t:taitype;ppufile:tcompilerppufile);
      var
        len : longint;
      begin
        inherited ppuload(t,ppufile);
        len:=ppufile.getlongint;
        getmem(str,len+1);
        ppufile.getdata(str^,len);
        str[len]:=#0;
      end;


    procedure tai_comment.ppuwrite(ppufile:tcompilerppufile);
      var
        len : longint;
      begin
        inherited ppuwrite(ppufile);
        len:=strlen(str);
        ppufile.putlongint(len);
        ppufile.putdata(str^,len);
      end;


    function tai_comment.getcopy : tlinkedlistitem;
      var
        p : tlinkedlistitem;
      begin
        p:=inherited getcopy;
        getmem(tai_comment(p).str,strlen(str)+1);
        move(str^,tai_comment(p).str^,strlen(str)+1);
        getcopy:=p;
      end;


{****************************************************************************
                              TAI_STABS
 ****************************************************************************}

    constructor tai_stab.create(_stabtype:TStabType;_str : pchar);
      begin
         inherited create;
         typ:=ait_stab;
         str:=_str;
         stabtype:=_stabtype;
      end;

    constructor tai_stab.create_str(_stabtype:TStabType;const s:string);
      begin
         self.create(_stabtype,strpnew(s));
      end;

    constructor tai_stab.create_ansistr(_stabtype:TStabType;const s:ansistring);
      begin
         inherited create;
         typ:=ait_stab;
         stabtype:=_stabtype;
         getmem(str,length(s)+1);
         if length(s)>0 then
           move(s[1],str^,length(s)+1)
         else
           str^:=#0;
      end;

    destructor tai_stab.destroy;
      begin
         freemem(str);
         inherited destroy;
      end;


{****************************************************************************
                            TAI_FORCE_LINE
 ****************************************************************************}

    constructor tai_force_line.create;
      begin
         inherited create;
         typ:=ait_force_line;
      end;


{****************************************************************************
                              TAI_FUNCTION_NAME
 ****************************************************************************}

    constructor tai_function_name.create(const s:string);
      begin
         inherited create;
         typ:=ait_function_name;
         funcname:=stringdup(s);
      end;

    destructor tai_function_name.destroy;
      begin
         stringdispose(funcname);
         inherited destroy;
      end;


{****************************************************************************
                              TAI_CUTOBJECT
 ****************************************************************************}

     constructor tai_cutobject.Create;
       begin
          inherited Create;
          typ:=ait_cutobject;
          place:=cut_normal;
       end;


     constructor tai_cutobject.Create_begin;
       begin
          inherited Create;
          typ:=ait_cutobject;
          place:=cut_begin;
       end;


     constructor tai_cutobject.Create_end;
       begin
          inherited Create;
          typ:=ait_cutobject;
          place:=cut_end;
       end;


    constructor tai_cutobject.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        place:=TCutPlace(ppufile.getbyte);
      end;


    procedure tai_cutobject.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(byte(place));
      end;


{****************************************************************************
                             Tai_Marker
 ****************************************************************************}

    constructor Tai_Marker.Create(_Kind: TAsmMarker);
      begin
        Inherited Create;
        typ := ait_marker;
        Kind := _Kind;
      end;


    constructor Tai_Marker.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        kind:=TAsmMarker(ppufile.getbyte);
      end;


    procedure Tai_Marker.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(byte(kind));
      end;


{*****************************************************************************
                                tai_tempalloc
*****************************************************************************}

    constructor tai_tempalloc.alloc(pos,size:longint);
      begin
        inherited Create;
        typ:=ait_tempalloc;
        allocation:=true;
        temppos:=pos;
        tempsize:=size;
{$ifdef EXTDEBUG}
        problem:=nil;
{$endif EXTDEBUG}
      end;


    destructor tai_tempalloc.destroy;
      begin
{$ifdef EXTDEBUG}
        stringdispose(problem);
{$endif EXTDEBUG}
        inherited destroy;
      end;


    constructor tai_tempalloc.dealloc(pos,size:longint);
      begin
        inherited Create;
        typ:=ait_tempalloc;
        allocation:=false;
        temppos:=pos;
        tempsize:=size;
{$ifdef EXTDEBUG}
        problem:=nil;
{$endif EXTDEBUG}
      end;


{$ifdef EXTDEBUG}
    constructor tai_tempalloc.allocinfo(pos,size:longint;const st:string);
      begin
        inherited Create;
        typ:=ait_tempalloc;
        allocation:=false;
        temppos:=pos;
        tempsize:=size;
        problem:=stringdup(st);
      end;
{$endif EXTDEBUG}


    constructor tai_tempalloc.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        temppos:=ppufile.getlongint;
        tempsize:=ppufile.getlongint;
        allocation:=boolean(ppufile.getbyte);
{$ifdef EXTDEBUG}
        problem:=nil;
{$endif EXTDEBUG}
      end;


    procedure tai_tempalloc.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putlongint(temppos);
        ppufile.putlongint(tempsize);
        ppufile.putbyte(byte(allocation));
      end;


{*****************************************************************************
                                 tai_regalloc
*****************************************************************************}

    constructor tai_regalloc.alloc(r : tregister;ainstr:tai);
      begin
        inherited create;
        typ:=ait_regalloc;
        ratype:=ra_alloc;
        reg:=r;
        { ainstr must be an instruction }
        if assigned(ainstr) and
           (ainstr.typ<>ait_instruction) then
          internalerror(200411011);
        instr:=ainstr;
      end;


    constructor tai_regalloc.dealloc(r : tregister;ainstr:tai);
      begin
        inherited create;
        typ:=ait_regalloc;
        ratype:=ra_dealloc;
        reg:=r;
        { ainstr must be an instruction }
        if assigned(ainstr) and
           (ainstr.typ<>ait_instruction) then
          internalerror(200411012);
        instr:=ainstr;
      end;


    constructor tai_regalloc.sync(r : tregister);
      begin
        inherited create;
        typ:=ait_regalloc;
        ratype:=ra_sync;
        reg:=r;
      end;


    constructor tai_regalloc.resize(r : tregister);
      begin
        inherited create;
        typ:=ait_regalloc;
        ratype:=ra_resize;
        reg:=r;
      end;


    constructor tai_regalloc.markused(r : tregister);
      begin
        inherited create;
        typ:=ait_regalloc;
        ratype:=ra_markused;
        reg:=r;
      end;


    constructor tai_regalloc.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getdata(reg,sizeof(Tregister));
        ratype:=tregalloctype(ppufile.getbyte);
        keep:=boolean(ppufile.getbyte);
      end;


    procedure tai_regalloc.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putdata(reg,sizeof(Tregister));
        ppufile.putbyte(byte(ratype));
        ppufile.putbyte(byte(keep));
      end;


{*****************************************************************************
                               TaiInstruction
*****************************************************************************}

    constructor tai_cpu_abstract.Create(op : tasmop);

      begin
         inherited create;
         typ:=ait_instruction;
         is_jmp:=false;
         opcode:=op;
         ops:=0;
      end;


    destructor tai_cpu_abstract.Destroy;
      var
        i : integer;
      begin
        for i:=0 to opercnt-1 do
          freeop(i);
        inherited destroy;
      end;


{ ---------------------------------------------------------------------
    Loading of operands.
  ---------------------------------------------------------------------}

    procedure tai_cpu_abstract.allocate_oper(opers:longint);
      begin
        while (opers>opercnt) do
          begin
            new(oper[opercnt]);
            fillchar(oper[opercnt]^,sizeof(toper),0);
            inc(opercnt);
          end;
      end;


    procedure tai_cpu_abstract.loadconst(opidx:longint;l:aint);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_const then
             clearop(opidx);
           val:=l;
           typ:=top_const;
         end;
      end;


    procedure tai_cpu_abstract.loadsymbol(opidx:longint;s:tasmsymbol;sofs:longint);
      var
        r : treference;
      begin
        reference_reset_symbol(r,s,sofs,1);
        r.refaddr:=addr_full;
        loadref(opidx,r);
      end;


    procedure tai_cpu_abstract.loadlocal(opidx:longint;s:pointer;sofs:longint;indexreg:tregister;scale:byte;getoffset,forceref:boolean);
      begin
        if not assigned(s) then
         internalerror(200204251);
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_local then
             begin
               clearop(opidx);
               new(localoper);
             end;
           with oper[opidx]^.localoper^ do
             begin
               localsym:=s;
               localsymofs:=sofs;
               localindexreg:=indexreg;
               localscale:=scale;
               localgetoffset:=getoffset;
               localforceref:=forceref;
             end;
           typ:=top_local;
         end;
      end;


    procedure tai_cpu_abstract.loadref(opidx:longint;const r:treference);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
          begin
            if typ<>top_ref then
              begin
                clearop(opidx);
                new(ref);
              end;

            ref^:=r;
{$ifdef x86}
            { We allow this exception for x86, since overloading this would be
              too much of a a speed penalty}
            if (ref^.segment<>NR_NO) and (ref^.segment<>NR_DS) then
              segprefix:=ref^.segment;
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef llvm}
            if (cs_create_pic in current_settings.moduleswitches) and
              assigned(r.symbol) and
              not assigned(r.relsymbol) and
              (r.refaddr=addr_no)
{$ifdef ARM}
              and not(r.base=NR_R15)
{$endif ARM}
{$ifdef aarch64}
              and not(r.refaddr in [addr_full,addr_gotpageoffset,addr_gotpage])
{$endif aarch64}
              then
              internalerror(200502052);
{$endif not llvm}
=======
            if (cs_create_pic in current_settings.moduleswitches) and
              assigned(r.symbol) and
              not assigned(r.relsymbol) and
              (r.refaddr=addr_no) then
              internalerror(200502052);
>>>>>>> graemeg/fixes_2_2
=======
            if (cs_create_pic in current_settings.moduleswitches) and
              assigned(r.symbol) and
              not assigned(r.relsymbol) and
              (r.refaddr=addr_no) then
              internalerror(200502052);
>>>>>>> origin/fixes_2_2
            typ:=top_ref;
            if assigned(add_reg_instruction_hook) then
              begin
                add_reg_instruction_hook(self,ref^.base);
                add_reg_instruction_hook(self,ref^.index);
              end;
            { mark symbol as used }
            if assigned(ref^.symbol) then
              ref^.symbol.increfs;
            if assigned(ref^.relsymbol) then
              ref^.relsymbol.increfs;
          end;
      end;


    procedure tai_cpu_abstract.loadreg(opidx:longint;r:tregister);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_reg then
             clearop(opidx);
           reg:=r;
           typ:=top_reg;
         end;
        if assigned(add_reg_instruction_hook) then
          add_reg_instruction_hook(self,r);
{$ifdef ARM}
        { R15 is the PC on the ARM thus moves to R15 are jumps.
          Due to speed considerations we don't use a virtual overridden method here.
          Because the pc/r15 isn't handled by the reg. allocator this should never cause
          problems with iregs getting r15.
        }
        is_jmp:=(opcode=A_MOV) and (opidx=0) and (r=NR_R15);
{$endif ARM}
      end;


    procedure tai_cpu_abstract.loadoper(opidx:longint;o:toper);
      begin
        allocate_oper(opidx+1);
        clearop(opidx);
        oper[opidx]^:=o;
        { copy also the reference }
        with oper[opidx]^ do
          begin
            case typ of
              top_reg:
                begin
                  if assigned(add_reg_instruction_hook) then
                    add_reg_instruction_hook(self,reg);
                end;
              top_ref:
                begin
                  new(ref);
                  ref^:=o.ref^;
{$ifdef x86}
                  if (ref^.segment<>NR_NO) and (ref^.segment<>NR_DS) then
                    segprefix:=ref^.segment;
{$endif x86}
                  if assigned(add_reg_instruction_hook) then
                    begin
                      add_reg_instruction_hook(self,ref^.base);
                      add_reg_instruction_hook(self,ref^.index);
                    end;
                end;
{$ifdef ARM}
              top_shifterop:
                begin
                  new(shifterop);
                  shifterop^:=o.shifterop^;
                  if assigned(add_reg_instruction_hook) then
                    add_reg_instruction_hook(self,shifterop^.rs);
                end;
{$endif ARM}
             end;
          end;
      end;

    procedure tai_cpu_abstract.clearop(opidx:longint);
      begin
        with oper[opidx]^ do
          begin
            case typ of
              top_ref:
                dispose(ref);
              top_local:
                dispose(localoper);
{$ifdef ARM}
              top_shifterop:
                dispose(shifterop);
              top_regset:
                dispose(regset);
{$endif ARM}
{$ifdef m68k}
              top_regset:
                begin
                  dispose(dataregset);
                  dispose(addrregset);
                  dispose(fpuregset);
                end;
{$endif m68k}
{$ifdef jvm}
              top_string:
                freemem(pcval);
              top_wstring:
                donewidestring(pwstrval);
{$endif jvm}
            end;
            typ:=top_none;
          end;
      end;


    procedure tai_cpu_abstract.freeop(opidx:longint);
      begin
        clearop(opidx);
        dispose(oper[opidx]);
      end;


{ ---------------------------------------------------------------------
    Miscellaneous methods.
  ---------------------------------------------------------------------}

    procedure tai_cpu_abstract.SetCondition(const c:TAsmCond);
      begin
         condition:=c;
      end;


    Function tai_cpu_abstract.getcopy:TLinkedListItem;
      var
        i : longint;
        p : tai_cpu_abstract;
      begin
        p:=tai_cpu_abstract(inherited getcopy);
        { make a copy of the references }
        p.opercnt:=0;
        p.allocate_oper(ops);
        for i:=0 to ops-1 do
          begin
            p.oper[i]^:=oper[i]^;
            case oper[i]^.typ of
              top_local :
                begin
                  new(p.oper[i]^.localoper);
                  p.oper[i]^.localoper^:=oper[i]^.localoper^;
                end;
              top_ref :
                begin
                  new(p.oper[i]^.ref);
                  p.oper[i]^.ref^:=oper[i]^.ref^;
                end;
{$ifdef ARM}
              top_shifterop:
                begin
                  new(p.oper[i]^.shifterop);
                  p.oper[i]^.shifterop^:=oper[i]^.shifterop^;
                end;
{$endif ARM}
            end;
          end;
        getcopy:=p;
      end;


    function tai_cpu_abstract.is_same_reg_move(regtype: Tregistertype):boolean;
      begin
        { When the generic RA is used this needs to be overridden, we don't use
          virtual;abstract; to prevent a lot of warnings of unimplemented abstract methods
          when tai_cpu is created (PFV) }
        internalerror(2004040901);
        result:=false;
      end;


    function tai_cpu_abstract.spilling_get_operation_type(opnr: longint): topertype;
      begin
        internalerror(2004040902);
        result:=operand_readwrite;
      end;


    function tai_cpu_abstract.spilling_get_operation_type_ref(opnr: longint; reg: tregister): topertype;
      begin
        result := operand_read;
      end;


    constructor tai_cpu_abstract.ppuload(t:taitype;ppufile:tcompilerppufile);
      var
        i : integer;
      begin
        inherited ppuload(t,ppufile);
        { hopefully, we don't get problems with big/litte endian here when cross compiling :/ }
        ppufile.getdata(condition,sizeof(tasmcond));
        allocate_oper(ppufile.getbyte);
        for i:=0 to ops-1 do
          ppuloadoper(ppufile,oper[i]^);
        opcode:=tasmop(ppufile.getword);
{$ifdef x86}
        ppufile.getdata(segprefix,sizeof(Tregister));
{$endif x86}
        is_jmp:=boolean(ppufile.getbyte);
      end;


    procedure tai_cpu_abstract.ppuwrite(ppufile:tcompilerppufile);
      var
        i : integer;
      begin
        inherited ppuwrite(ppufile);
        ppufile.putdata(condition,sizeof(tasmcond));
        ppufile.putbyte(ops);
        for i:=0 to ops-1 do
          ppuwriteoper(ppufile,oper[i]^);
        ppufile.putword(word(opcode));
{$ifdef x86}
        ppufile.putdata(segprefix,sizeof(Tregister));
{$endif x86}
        ppufile.putbyte(byte(is_jmp));
      end;


    procedure tai_cpu_abstract.buildderefimpl;
      var
        i : integer;
      begin
        for i:=0 to ops-1 do
          ppubuildderefimploper(oper[i]^);
      end;


    procedure tai_cpu_abstract.derefimpl;
      var
        i : integer;
      begin
        for i:=0 to ops-1 do
          ppuderefoper(oper[i]^);
      end;


    procedure tai_cpu_abstract.resetpass1;
      begin
      end;


    procedure tai_cpu_abstract.resetpass2;
      begin
      end;


   function tai_cpu_abstract.Pass1(objdata:TObjData):longint;
      begin
        result:=0;
      end;


    procedure tai_cpu_abstract.Pass2(objdata:TObjData);
      begin
      end;


    procedure tai_cpu_abstract.ppuloadoper(ppufile:tcompilerppufile;var o:toper);
      begin
        o.typ:=toptype(ppufile.getbyte);
        o.ot:=ppufile.getlongint;
        case o.typ of
          top_reg :
            ppufile.getdata(o.reg,sizeof(Tregister));
          top_ref :
            begin
              new(o.ref);
{$ifdef x86}
              ppufile.getdata(o.ref^.segment,sizeof(Tregister));
{$endif x86}
              ppufile.getdata(o.ref^.base,sizeof(Tregister));
              ppufile.getdata(o.ref^.index,sizeof(Tregister));
              ppufile.getdata(o.ref^.refaddr,sizeof(o.ref^.refaddr));
              o.ref^.scalefactor:=ppufile.getbyte;
              o.ref^.offset:=ppufile.getaint;
              o.ref^.symbol:=ppufile.getasmsymbol;
              o.ref^.relsymbol:=ppufile.getasmsymbol;
            end;
          top_const :
            o.val:=ppufile.getaint;
          top_local :
            begin
              new(o.localoper);
              with o.localoper^ do
                begin
                  ppufile.getderef(localsymderef);
                  localsymofs:=ppufile.getaint;
                  localindexreg:=tregister(ppufile.getlongint);
                  localscale:=ppufile.getbyte;
                  localgetoffset:=(ppufile.getbyte<>0);
                end;
            end;
          else
            internalerror(2007010210);
        end;
      end;


    procedure tai_cpu_abstract.ppuwriteoper(ppufile:tcompilerppufile;const o:toper);
      begin
        ppufile.putbyte(byte(o.typ));
        ppufile.putlongint(o.ot);
        case o.typ of
          top_reg :
            ppufile.putdata(o.reg,sizeof(Tregister));
          top_ref :
            begin
{$ifdef x86}
              ppufile.putdata(o.ref^.segment,sizeof(Tregister));
{$endif x86}
              ppufile.putdata(o.ref^.base,sizeof(Tregister));
              ppufile.putdata(o.ref^.index,sizeof(Tregister));
              ppufile.putdata(o.ref^.refaddr,sizeof(o.ref^.refaddr));
              ppufile.putbyte(o.ref^.scalefactor);
              ppufile.putaint(o.ref^.offset);
              ppufile.putasmsymbol(o.ref^.symbol);
              ppufile.putasmsymbol(o.ref^.relsymbol);
            end;
          top_const :
            ppufile.putaint(o.val);
          top_local :
            begin
              with o.localoper^ do
                begin
                  ppufile.putderef(localsymderef);
                  ppufile.putaint(localsymofs);
                  ppufile.putlongint(longint(localindexreg));
                  ppufile.putbyte(localscale);
                  ppufile.putbyte(byte(localgetoffset));
                end;
            end;
          else
            internalerror(2007010211);
        end;
      end;

{****************************************************************************
                              tai_align_abstract
 ****************************************************************************}

     constructor tai_align_abstract.Create(b: byte);
       begin
          inherited Create;
          typ:=ait_align;
{$ifdef EXTDEBUG}
          if upper(classname)='TAI_ALIGN_ABSTRACT' then
            internalerror(200709191);
{$endif EXTDEBUG}
          if b in [1,2,4,8,16,32] then
            aligntype := b
          else
            aligntype := 1;
          fillsize:=0;
          fillop:=0;
          use_op:=false;
       end;


     constructor tai_align_abstract.Create_op(b: byte; _op: byte);
       begin
          inherited Create;
          typ:=ait_align;
          if b in [1,2,4,8,16,32] then
            aligntype := b
          else
            aligntype := 1;
          fillsize:=0;
          fillop:=_op;
          use_op:=true;
       end;


     constructor tai_align_abstract.Create_zeros(b: byte);
       begin
          inherited Create;
          typ:=ait_align;
          if b in [1,2,4,8,16,32] then
            aligntype := b
          else
            aligntype := 1;
         use_op:=true;
         fillsize:=0;
         fillop:=0;
       end;


     function tai_align_abstract.calculatefillbuf(var buf : tfillbuffer;executable : boolean):pchar;
       begin
         if fillsize>sizeof(buf) then
           internalerror(200404293);
         fillchar(buf,high(buf),fillop);
         calculatefillbuf:=pchar(@buf);
       end;


    constructor tai_align_abstract.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        aligntype:=ppufile.getbyte;
        fillsize:=0;
        fillop:=ppufile.getbyte;
        use_op:=boolean(ppufile.getbyte);
      end;


    procedure tai_align_abstract.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(aligntype);
        ppufile.putbyte(fillop);
        ppufile.putbyte(byte(use_op));
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{****************************************************************************
                              tai_seh_directive
 ****************************************************************************}

    const
      datatypemap: array[TAsmSehDirective] of TSehDirectiveDatatype=(
        sd_string,     { proc }
        sd_none,       { endproc }
        sd_none,       { endprologue }
        sd_string,     { handler }
        sd_none,       { handlerdata }
        sd_none,sd_none,sd_none,  { eh, 32, no32 }
        sd_regoffset,  { setframe }
        sd_offset,     { stackalloc }
        sd_reg,        { pushreg }
        sd_regoffset,  { savereg }
        sd_regoffset,  { savexmm }
        sd_none        { pushframe }
      );

    constructor tai_seh_directive.create(_kind:TAsmSehDirective);
      begin
        inherited Create;
        typ:=ait_seh_directive;
        kind:=_kind;
        data.typ:=datatypemap[_kind];
      end;

    constructor tai_seh_directive.create_name(_kind:TAsmSehDirective;const _name:string);
      begin
        create(_kind);
        data.name:=stringdup(_name);
      end;

    constructor tai_seh_directive.create_reg(_kind:TAsmSehDirective;r:TRegister);
      begin
        create(_kind);
        data.reg:=r;
      end;

    constructor tai_seh_directive.create_offset(_kind:TAsmSehDirective;ofs:dword);
      begin
        create(_kind);
        data.offset:=ofs;
      end;

    constructor tai_seh_directive.create_reg_offset(_kind:TAsmSehDirective;
      r:TRegister;ofs:dword);
      begin
        create(_kind);
        data.offset:=ofs;
        data.reg:=r;
      end;

    constructor tai_seh_directive.ppuload(t:taitype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t, ppufile);
        kind:=TAsmSehDirective(ppufile.getbyte);
        data.typ:=datatypemap[kind];
        case data.typ of
          sd_none: ;
          sd_string:
            begin
              data.name:=stringdup(ppufile.getstring);
              data.flags:=ppufile.getbyte;
            end;

          sd_reg,sd_offset,sd_regoffset:
            begin
              ppufile.getdata(data.reg,sizeof(TRegister));
              data.offset:=ppufile.getdword;
            end;
        else
          InternalError(2011091201);
        end;
      end;

    destructor tai_seh_directive.destroy;
      begin
        if data.typ=sd_string then
          stringdispose(data.name);
        inherited destroy;
      end;

    procedure tai_seh_directive.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(ord(kind));
        case data.typ of
          sd_none: ;
          sd_string:
            begin
              ppufile.putstring(data.name^);
              ppufile.putbyte(data.flags);
            end;

          sd_reg,sd_offset,sd_regoffset:
            begin
              ppufile.putdata(data.reg,sizeof(TRegister));
              ppufile.putdword(data.offset);
            end;
        else
          InternalError(2011091202);
        end;
      end;

    procedure tai_seh_directive.generate_code(objdata:TObjData);
      begin
      end;

{$ifdef JVM}

{****************************************************************************
                              tai_jvar
 ****************************************************************************}

    constructor tai_jvar.Create(_stackslot: longint; const _desc: shortstring; _startlab, _stoplab: TAsmSymbol);
      begin
        Inherited create;
        typ:=ait_jvar;
        stackslot:=_stackslot;
        desc:=stringdup(_desc);
        startlab:=_startlab;
        stoplab:=_stoplab;
      end;


    constructor tai_jvar.ppuload(t: taitype; ppufile: tcompilerppufile);
      begin
        inherited ppuload(t, ppufile);
        stackslot:=ppufile.getlongint;
        desc:=stringdup(ppufile.getstring);
        startlab:=ppufile.getasmsymbol;
        stoplab:=ppufile.getasmsymbol;
      end;


    procedure tai_jvar.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putlongint(stackslot);
        ppufile.putstring(desc^);
        ppufile.putasmsymbol(startlab);
        ppufile.putasmsymbol(stoplab);
      end;


    destructor tai_jvar.destroy;
      begin
        stringdispose(desc);
        inherited destroy;
      end;


{****************************************************************************
                              tai_jcatch
 ****************************************************************************}

    constructor tai_jcatch.Create(const _name: shortstring; _startlab, _stoplab, _handlerlab: TAsmSymbol);
      begin
        Inherited create;
        typ:=ait_jcatch;
        name:=stringdup(_name);
        startlab:=_startlab;
        startlab.increfs;
        stoplab:=_stoplab;
        stoplab.increfs;
        handlerlab:=_handlerlab;
        handlerlab.increfs;
      end;


    destructor tai_jcatch.destroy;
      begin
        stringdispose(name);
        inherited destroy;
      end;


    constructor tai_jcatch.ppuload(t: taitype; ppufile: tcompilerppufile);
      begin
        inherited ppuload(t, ppufile);
        name:=stringdup(ppufile.getstring);
        startlab:=ppufile.getasmsymbol;
        startlab.increfs;
        stoplab:=ppufile.getasmsymbol;
        stoplab.increfs;
        handlerlab:=ppufile.getasmsymbol;
        handlerlab.increfs;
      end;


    procedure tai_jcatch.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putstring(name^);
        ppufile.putasmsymbol(startlab);
        ppufile.putasmsymbol(stoplab);
        ppufile.putasmsymbol(handlerlab);
      end;

{$endif JVM}

begin
<<<<<<< HEAD
{$push}{$warnings off}
  { taitype should fit into a 4 byte set for speed reasons }
  if ord(high(taitype))>31 then
    internalerror(201108181);
{$pop}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  cai_cpu:=tai_cpu_abstract;
  { aasmcpu is earlier in the unit order and can
    already set the cai_align }
  if not assigned(cai_align) then
    cai_align:=tai_align_abstract;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
