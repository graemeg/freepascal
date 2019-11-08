  {
    Copyright (c) 1998-2006 by the Free Pascal team

    This unit implements the generic part of the GNU assembler
    (v2.8 or later) writer

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
{ Base unit for writing GNU assembler output.
}
unit aggas;

{$i fpcdefs.inc}

{ $define DEBUG_AGGAS}

interface

    uses
      globtype,globals,
      aasmbase,aasmtai,aasmdata,aasmcfi,
      assemble;

    type
      TCPUInstrWriter = class;
      {# This is a derived class which is used to write
         GAS styled assembler.
      }

      { TGNUAssembler }

      TGNUAssembler=class(texternalassembler)
      protected
        function sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;virtual;
        function sectionattrs(atype:TAsmSectiontype):string;virtual;
        function sectionattrs_coff(atype:TAsmSectiontype):string;virtual;
        function sectionalignment_aix(atype:TAsmSectiontype;secalign: longint):string;
        procedure WriteSection(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder;secalign:longint;
          secflags:TSectionFlags=[];secprogbits:TSectionProgbits=SPB_None);virtual;
        procedure WriteExtraHeader;virtual;
        procedure WriteExtraFooter;virtual;
        procedure WriteInstruction(hp: tai);
        procedure WriteWeakSymbolRef(s: tasmsymbol); virtual;
        procedure WriteHiddenSymbol(sym: TAsmSymbol);
        procedure WriteAixStringConst(hp: tai_string);
        procedure WriteAixIntConst(hp: tai_const);
        procedure WriteUnalignedIntConst(hp: tai_const);
        procedure WriteDirectiveName(dir: TAsmDirective); virtual;
       public
        function MakeCmdLine: TCmdStr; override;
        procedure WriteTree(p:TAsmList);override;
        procedure WriteAsmList;override;
        destructor destroy; override;
       private
        setcount: longint;
        procedure WriteDecodedSleb128(a: int64);
        procedure WriteDecodedUleb128(a: qword);
        procedure WriteCFI(hp: tai_cfi_base);
        function NextSetLabel: string;
       protected
        InstrWriter: TCPUInstrWriter;
      end;


      {# This is the base class for writing instructions.

         The WriteInstruction() method must be overridden
         to write a single instruction to the assembler
         file.
      }
      TCPUInstrWriter = class
        constructor create(_owner: TGNUAssembler);
        procedure WriteInstruction(hp : tai); virtual; abstract;
       protected
        owner: TGNUAssembler;
      end;


      { TAppleGNUAssembler }

      TAppleGNUAssembler=class(TGNUAssembler)
       protected
        function sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;override;
        procedure WriteWeakSymbolRef(s: tasmsymbol); override;
        procedure WriteDirectiveName(dir: TAsmDirective); override;
       end;


      TAoutGNUAssembler=class(TGNUAssembler)
        function sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;override;
       end;


implementation

    uses
      SysUtils,
      cutils,cfileutl,systems,
      fmodule,verbose,
{$ifndef DISABLE_WIN64_SEH}
      itcpugas,
{$endif DISABLE_WIN64_SEH}
{$ifdef m68k}
      cpuinfo,aasmcpu,
{$endif m68k}
      cpubase,objcasm;

    const
      line_length = 70;

    var
      symendcount  : longint;

{****************************************************************************}
{                          Support routines                                  }
{****************************************************************************}

    const
      ait_const2str : array[aitconst_128bit..aitconst_64bit_unaligned] of string[20]=(
        #9'.fixme128'#9,#9'.quad'#9,#9'.long'#9,#9'.short'#9,#9'.byte'#9,
        #9'.sleb128'#9,#9'.uleb128'#9,
        #9'.rva'#9,#9'.secrel32'#9,#9'.quad'#9,#9'.long'#9,#9'.short'#9,#9'.short'#9,
        #9'.short'#9,#9'.long'#9,#9'.quad'#9
      );

      ait_solaris_const2str : array[aitconst_128bit..aitconst_64bit_unaligned] of string[20]=(
        #9'.fixme128'#9,#9'.8byte'#9,#9'.4byte'#9,#9'.2byte'#9,#9'.byte'#9,
        #9'.sleb128'#9,#9'.uleb128'#9,
        #9'.rva'#9,#9'.secrel32'#9,#9'.8byte'#9,#9'.4byte'#9,#9'.2byte'#9,#9'.2byte'#9,
        #9'.2byte'#9,#9'.4byte'#9,#9'.8byte'#9
      );

      ait_unaligned_consts = [aitconst_16bit_unaligned..aitconst_64bit_unaligned];

      { Sparc type of unaligned pseudo-instructions }
      use_ua_sparc_systems = [system_sparc_linux];
      ait_ua_sparc_const2str : array[aitconst_16bit_unaligned..aitconst_64bit_unaligned]
        of string[20]=(
          #9'.uahalf'#9,#9'.uaword'#9,#9'.uaxword'#9
        );

      { Generic unaligned pseudo-instructions, seems ELF specific }
      use_ua_elf_systems = [system_mipsel_linux,system_mipseb_linux,system_mipsel_android,system_mipsel_embedded,system_mipseb_embedded];
      ait_ua_elf_const2str : array[aitconst_16bit_unaligned..aitconst_64bit_unaligned]
        of string[20]=(
          #9'.2byte'#9,#9'.4byte'#9,#9'.8byte'#9
        );



{****************************************************************************}
{                          GNU Assembler writer                              }
{****************************************************************************}

    destructor TGNUAssembler.Destroy;
      begin
        InstrWriter.free;
        inherited destroy;
      end;


    function TGNUAssembler.MakeCmdLine: TCmdStr;
      begin
        result := inherited MakeCmdLine;
        // MWE: disabled again. It generates dwarf info for the generated .s
        //      files as well. This conflicts with the info we generate
        // if target_dbg.id = dbg_dwarf then
        //  result := result + ' --gdwarf-2';
      end;


    function TGNUAssembler.NextSetLabel: string;
      begin
        inc(setcount);
        result := asminfo^.labelprefix+'$set$'+tostr(setcount);
      end;

    function is_smart_section(atype:TAsmSectiontype):boolean;
      begin
        { For bss we need to set some flags that are target dependent,
          it is easier to disable it for smartlinking. It doesn't take up
          filespace }
        result:=not(target_info.system in systems_darwin) and
           create_smartlink_sections and
           (atype<>sec_toc) and
           (atype<>sec_user) and
           { on embedded systems every byte counts, so smartlink bss too }
           ((atype<>sec_bss) or (target_info.system in systems_embedded));
      end;

    function TGNUAssembler.sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;
      const
        secnames : array[TAsmSectiontype] of string[length('__DATA, __datacoal_nt,coalesced')] = ('','',
          '.text',
          '.data',
{ why doesn't .rodata work? (FK) }
{ sometimes we have to create a data.rel.ro instead of .rodata, e.g. for  }
{ vtables (and anything else containing relocations), otherwise those are }
{ not relocated properly on e.g. linux/ppc64. g++ generates there for a   }
{ vtable for a class called Window:                                       }
{ .section .data.rel.ro._ZTV6Window,"awG",@progbits,_ZTV6Window,comdat    }
{ TODO: .data.ro not yet working}
{$if defined(arm) or defined(riscv64) or defined(powerpc)}
          '.rodata',
{$else arm}
          '.data',
{$endif arm}
          '.rodata',
          '.bss',
          '.threadvar',
          '.pdata',
          '', { stubs }
          '__DATA,__nl_symbol_ptr',
          '__DATA,__la_symbol_ptr',
          '__DATA,__mod_init_func',
          '__DATA,__mod_term_func',
          '.stab',
          '.stabstr',
          '.idata$2','.idata$4','.idata$5','.idata$6','.idata$7','.edata',
          '.eh_frame',
          '.debug_frame','.debug_info','.debug_line','.debug_abbrev','.debug_aranges','.debug_ranges',
          '.fpc',
          '.toc',
          '.init',
          '.fini',
          '.objc_class',
          '.objc_meta_class',
          '.objc_cat_cls_meth',
          '.objc_cat_inst_meth',
          '.objc_protocol',
          '.objc_string_object',
          '.objc_cls_meth',
          '.objc_inst_meth',
          '.objc_cls_refs',
          '.objc_message_refs',
          '.objc_symbols',
          '.objc_category',
          '.objc_class_vars',
          '.objc_instance_vars',
          '.objc_module_info',
          '.objc_class_names',
          '.objc_meth_var_types',
          '.objc_meth_var_names',
          '.objc_selector_strs',
          '.objc_protocol_ext',
          '.objc_class_ext',
          '.objc_property',
          '.objc_image_info',
          '.objc_cstring_object',
          '.objc_sel_fixup',
          '__DATA,__objc_data',
          '__DATA,__objc_const',
          '.objc_superrefs',
          '__DATA, __datacoal_nt,coalesced',
          '.objc_classlist',
          '.objc_nlclasslist',
          '.objc_catlist',
          '.obcj_nlcatlist',
          '.objc_protolist',
          '.stack',
          '.heap',
          '.gcc_except_table',
          '.ARM.attributes'
        );
        secnames_pic : array[TAsmSectiontype] of string[length('__DATA, __datacoal_nt,coalesced')] = ('','',
          '.text',
          '.data.rel',
          '.data.rel',
          '.data.rel',
          '.bss',
          '.threadvar',
          '.pdata',
          '', { stubs }
          '__DATA,__nl_symbol_ptr',
          '__DATA,__la_symbol_ptr',
          '__DATA,__mod_init_func',
          '__DATA,__mod_term_func',
          '.stab',
          '.stabstr',
          '.idata$2','.idata$4','.idata$5','.idata$6','.idata$7','.edata',
          '.eh_frame',
          '.debug_frame','.debug_info','.debug_line','.debug_abbrev','.debug_aranges','.debug_ranges',
          '.fpc',
          '.toc',
          '.init',
          '.fini',
          '.objc_class',
          '.objc_meta_class',
          '.objc_cat_cls_meth',
          '.objc_cat_inst_meth',
          '.objc_protocol',
          '.objc_string_object',
          '.objc_cls_meth',
          '.objc_inst_meth',
          '.objc_cls_refs',
          '.objc_message_refs',
          '.objc_symbols',
          '.objc_category',
          '.objc_class_vars',
          '.objc_instance_vars',
          '.objc_module_info',
          '.objc_class_names',
          '.objc_meth_var_types',
          '.objc_meth_var_names',
          '.objc_selector_strs',
          '.objc_protocol_ext',
          '.objc_class_ext',
          '.objc_property',
          '.objc_image_info',
          '.objc_cstring_object',
          '.objc_sel_fixup',
          '__DATA, __objc_data',
          '__DATA, __objc_const',
          '.objc_superrefs',
          '__DATA, __datacoal_nt,coalesced',
          '.objc_classlist',
          '.objc_nlclasslist',
          '.objc_catlist',
          '.obcj_nlcatlist',
          '.objc_protolist',
          '.stack',
          '.heap',
          '.gcc_except_table',
          '..ARM.attributes'
        );
      var
        sep     : string[3];
        secname : string;
      begin
        if (cs_create_pic in current_settings.moduleswitches) and
           not(target_info.system in systems_darwin) then
          secname:=secnames_pic[atype]
        else
          secname:=secnames[atype];

        if (atype=sec_fpc) and (Copy(aname,1,3)='res') then
          begin
            result:=secname+'.'+aname;
            exit;
          end;

        if atype=sec_threadvar then
          begin
            if (target_info.system in (systems_windows+systems_wince)) then
              secname:='.tls'
            else if (target_info.system in systems_linux) then
              secname:='.tbss';
          end;

        { go32v2 stub only loads .text and .data sections, and allocates space for .bss.
          Thus, data which normally goes into .rodata and .rodata_norel sections must
          end up in .data section }
        if (atype in [sec_rodata,sec_rodata_norel]) and
          (target_info.system in [system_i386_go32v2,system_m68k_palmos]) then
          secname:='.data';

        { Windows correctly handles reallocations in readonly sections }
        if (atype=sec_rodata) and
          (target_info.system in systems_all_windows+systems_nativent-[system_i8086_win16]) then
          secname:='.rodata';

        { Use .rodata and .data.rel.ro for Android with PIC }
        if (target_info.system in systems_android) and (cs_create_pic in current_settings.moduleswitches) then
          begin
            case atype of
              sec_rodata:
                secname:='.data.rel.ro';
              sec_rodata_norel:
                secname:='.rodata';
              else
                ;
            end;
          end;

        { section type user gives the user full controll on the section name }
        if atype=sec_user then
          secname:=aname;

        if is_smart_section(atype) and (aname<>'') then
          begin
            case aorder of
              secorder_begin :
                sep:='.b_';
              secorder_end :
                sep:='.z_';
              else
                sep:='.n_';
            end;
            result:=secname+sep+aname
          end
        else
          result:=secname;
      end;

    function TGNUAssembler.sectionattrs(atype:TAsmSectiontype):string;
      begin
        result:='';
        if (target_info.system in [system_i386_win32,system_x86_64_win64]) then
          begin
            result:=sectionattrs_coff(atype);
          end;
      end;

    function TGNUAssembler.sectionattrs_coff(atype:TAsmSectiontype):string;
      begin
        case atype of
          sec_code, sec_init, sec_fini, sec_stub:
            result:='x';

          { TODO: must be individual for each section }
          sec_user:
            result:='d';

          sec_data, sec_data_lazy, sec_data_nonlazy, sec_fpc,
          sec_idata2, sec_idata4, sec_idata5, sec_idata6, sec_idata7:
            result:='d';

          { TODO: these need a fix to become read-only }
          sec_rodata, sec_rodata_norel:
            result:='d';

          sec_bss:
            result:='b';

          { TODO: Somewhat questionable. FPC does not allow initialized threadvars,
            so no sense to mark it as containing data. But Windows allows it to
            contain data, and Linux even has .tdata and .tbss }
          sec_threadvar:
            result:='b';

          sec_pdata, sec_edata, sec_eh_frame, sec_toc:
            result:='r';

          sec_stab,sec_stabstr,
          sec_debug_frame,sec_debug_info,sec_debug_line,sec_debug_abbrev,sec_debug_aranges,sec_debug_ranges:
            result:='n';
        else
          result:='';  { defaults to data+load }
        end;
      end;


    function TGNUAssembler.sectionalignment_aix(atype:TAsmSectiontype;secalign: longint): string;
      var
        l: longint;
      begin
        if (secalign=0) or
           not(atype in [sec_code,sec_bss,sec_rodata_norel,sec_rodata,sec_data]) then
          begin
            result:='';
            exit;
          end;
        if not ispowerof2(secalign,l) then
          internalerror(2012022201);
        result:=tostr(l);
      end;


    procedure TGNUAssembler.WriteSection(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder;secalign:longint;secflags:TSectionFlags=[];secprogbits:TSectionProgbits=SPB_None);
      var
        s : string;
        secflag: TSectionFlag;
        sectionprogbits,
        sectionflags: boolean;
      begin
        writer.AsmLn;
        sectionflags:=false;
        sectionprogbits:=false;
        case target_info.system of
         system_i386_OS2,
         system_i386_EMX: ;
         system_m68k_atari, { atari tos/mint GNU AS also doesn't seem to like .section (KB) }
         system_m68k_amiga: { amiga has old GNU AS (2.14), which blews up from .section (KB) }
           begin
             { ... but vasm is GAS compatible on amiga/atari, and supports named sections }
             if create_smartlink_sections then
               begin
                 writer.AsmWrite('.section ');
                 sectionflags:=true;
                 sectionprogbits:=true;
               end;
           end;
         system_i386_win32,
         system_x86_64_win64,
         system_i386_wince,
         system_arm_wince:
           begin
             { according to the GNU AS guide AS for COFF does not support the
               progbits }
             writer.AsmWrite('.section ');
             sectionflags:=true;
           end;
         system_powerpc_darwin,
         system_i386_darwin,
         system_i386_iphonesim,
         system_powerpc64_darwin,
         system_x86_64_darwin,
         system_arm_darwin,
         system_aarch64_darwin,
         system_x86_64_iphonesim,
         system_powerpc_aix,
         system_powerpc64_aix:
           begin
             if (atype in [sec_stub]) then
               writer.AsmWrite('.section ');
           end
         else
           begin
             writer.AsmWrite('.section ');
             sectionflags:=true;
             sectionprogbits:=true;
           end
        end;
        s:=sectionname(atype,aname,aorder);
        writer.AsmWrite(s);
        { flags explicitly defined? }
        if (sectionflags or sectionprogbits) and
           ((secflags<>[]) or
            (secprogbits<>SPB_None)) then
          begin
            if sectionflags then
              begin
                s:=',"';
                for secflag in secflags do
                  case secflag of
                    SF_A:
                      s:=s+'a';
                    SF_W:
                      s:=s+'w';
                    SF_X:
                      s:=s+'x';
                  end;
                writer.AsmWrite(s+'"');
              end;
            if sectionprogbits then
              begin
                case secprogbits of
                  SPB_PROGBITS:
                    writer.AsmWrite(',%progbits');
                  SPB_NOBITS:
                    writer.AsmWrite(',%nobits');
                  SPB_NOTE:
                    writer.AsmWrite(',%note');
                  SPB_None:
                    ;
                  else
                    InternalError(2019100801);
                end;
              end;
          end
        else
          case atype of
            sec_fpc :
              if aname = 'resptrs' then
                writer.AsmWrite(', "a", @progbits');
            sec_stub :
              begin
                case target_info.system of
                  { there are processor-independent shortcuts available    }
                  { for this, namely .symbol_stub and .picsymbol_stub, but }
                  { they don't work and gcc doesn't use them either...     }
                  system_powerpc_darwin,
                  system_powerpc64_darwin:
                    if (cs_create_pic in current_settings.moduleswitches) then
                      writer.AsmWriteln('__TEXT,__picsymbolstub1,symbol_stubs,pure_instructions,32')
                    else
                      writer.AsmWriteln('__TEXT,__symbol_stub1,symbol_stubs,pure_instructions,16');
                  system_i386_darwin,
                  system_i386_iphonesim:
                    writer.AsmWriteln('__IMPORT,__jump_table,symbol_stubs,self_modifying_code+pure_instructions,5');
                  system_arm_darwin:
                    if (cs_create_pic in current_settings.moduleswitches) then
                      writer.AsmWriteln('__TEXT,__picsymbolstub4,symbol_stubs,none,16')
                    else
                      writer.AsmWriteln('__TEXT,__symbol_stub4,symbol_stubs,none,12')
                  { darwin/(x86-64/AArch64) uses PC-based GOT addressing, no
                    explicit symbol stubs }
                  else
                    internalerror(2006031101);
                end;
              end;
          else
            { GNU AS won't recognize '.text.n_something' section name as belonging
              to '.text' and assigns default attributes to it, which is not
              always correct. We have to fix it.

              TODO: This likely applies to all systems which smartlink without
              creating libraries }
            begin
              if is_smart_section(atype) and (aname<>'') then
                begin
                  s:=sectionattrs(atype);
                  if (s<>'') then
                    writer.AsmWrite(',"'+s+'"');
                end;
              if target_info.system in systems_aix then
                begin
                  s:=sectionalignment_aix(atype,secalign);
                  if s<>'' then
                    writer.AsmWrite(','+s);
                end;
            end;
          end;
        writer.AsmLn;
        LastSecType:=atype;
      end;


    procedure TGNUAssembler.WriteDecodedUleb128(a: qword);
      var
        i,len : longint;
        buf   : array[0..63] of byte;
      begin
        len:=EncodeUleb128(a,buf,0);
        for i:=0 to len-1 do
          begin
            if (i > 0) then
              writer.AsmWrite(',');
            writer.AsmWrite(tostr(buf[i]));
          end;
      end;


    procedure TGNUAssembler.WriteCFI(hp: tai_cfi_base);
      begin
        writer.AsmWrite(cfi2str[hp.cfityp]);
        case hp.cfityp of
          cfi_startproc,
          cfi_endproc:
            ;
          cfi_undefined,
          cfi_restore,
          cfi_def_cfa_register:
            begin
              writer.AsmWrite(' ');
              writer.AsmWrite(gas_regname(tai_cfi_op_reg(hp).reg1));
            end;
          cfi_def_cfa_offset:
            begin
              writer.AsmWrite(' ');
              writer.AsmWrite(tostr(tai_cfi_op_val(hp).val1));
            end;
          cfi_offset:
            begin
              writer.AsmWrite(' ');
              writer.AsmWrite(gas_regname(tai_cfi_op_reg_val(hp).reg1));
              writer.AsmWrite(',');
              writer.AsmWrite(tostr(tai_cfi_op_reg_val(hp).val));
            end;
          else
            internalerror(2019030203);
        end;
        writer.AsmLn;
      end;


    procedure TGNUAssembler.WriteDecodedSleb128(a: int64);
      var
        i,len : longint;
        buf   : array[0..255] of byte;
      begin
        len:=EncodeSleb128(a,buf,0);
        for i:=0 to len-1 do
          begin
            if (i > 0) then
              writer.AsmWrite(',');
            writer.AsmWrite(tostr(buf[i]));
          end;
      end;


    procedure TGNUAssembler.WriteTree(p:TAsmList);

      function needsObject(hp : tai_symbol) : boolean;
        begin
          needsObject :=
              (
                assigned(hp.next) and
                 (tai(hp.next).typ in [ait_const,ait_datablock,ait_realconst])
              ) or
              (hp.sym.typ in [AT_DATA,AT_METADATA]);

        end;


      procedure doalign(alignment: byte; use_op: boolean; fillop: byte; maxbytes: byte; out last_align: longint;lasthp:tai);
        var
          i: longint;
          alignment64 : int64;
{$ifdef m68k}
          instr : string;
{$endif}
        begin
          last_align:=alignment;
          if alignment>1 then
            begin
              if not(target_info.system in (systems_darwin+systems_aix)) then
                begin
{$ifdef m68k}
                  if not use_op and (lastsectype=sec_code) then
                    begin
                      if not ispowerof2(alignment,i) then
                        internalerror(2014022201);
                      { the Coldfire manual suggests the TBF instruction for
                        alignments, but somehow QEMU does not interpret that
                        correctly... }
                      {if current_settings.cputype in cpu_coldfire then
                        instr:='0x51fc'
                      else}
                        instr:='0x4e71';
                      writer.AsmWrite(#9'.balignw '+tostr(alignment)+','+instr);
                    end
                  else
                    begin
{$endif m68k}
                      alignment64:=alignment;
                      if (maxbytes<>alignment) and ispowerof2(alignment64,i) then
                        begin
                          if use_op then
                            begin
                              writer.AsmWrite(#9'.p2align '+tostr(i)+','+tostr(fillop)+','+tostr(maxbytes));
                              writer.AsmLn;
                              writer.AsmWrite(#9'.p2align '+tostr(i-1)+','+tostr(fillop));
                            end
                          else
                            begin
                              writer.AsmWrite(#9'.p2align '+tostr(i)+',,'+tostr(maxbytes));
                              writer.AsmLn;
                              writer.AsmWrite(#9'.p2align '+tostr(i-1));
                            end
                        end
                      else
                        begin
                          writer.AsmWrite(#9'.balign '+tostr(alignment));
                          if use_op then
                            writer.AsmWrite(','+tostr(fillop))
{$ifdef x86}
                          { force NOP as alignment op code }
                          else if (LastSecType=sec_code) and (asminfo^.id<>as_solaris_as) then
                            writer.AsmWrite(',0x90');
{$endif x86}
                        end;
{$ifdef m68k}
                    end;
{$endif m68k}
                end
              else
                begin
                  { darwin and aix as only support .align }
                  if not ispowerof2(alignment,i) then
                    internalerror(2003010305);
                  writer.AsmWrite(#9'.align '+tostr(i));
                  last_align:=i;
                end;
              writer.AsmLn;
            end;
        end;

    var
      ch       : char;
      lasthp,
      hp       : tai;
      constdef : taiconst_type;
      s,t      : string;
      i,pos,l  : longint;
      InlineLevel : cardinal;
      last_align : longint;
      do_line  : boolean;

      sepChar : char;
      replaceforbidden: boolean;
    begin
      if not assigned(p) then
       exit;
      replaceforbidden:=asminfo^.dollarsign<>'$';

      last_align := 2;
      InlineLevel:=0;
      { lineinfo is only needed for al_procedures (PFV) }
      do_line:=(cs_asm_source in current_settings.globalswitches) or
               ((cs_lineinfo in current_settings.moduleswitches)
                 and (p=current_asmdata.asmlists[al_procedures]));
      lasthp:=nil;
      hp:=tai(p.first);
      while assigned(hp) do
       begin
         prefetch(pointer(hp.next)^);
         if not(hp.typ in SkipLineInfo) then
          begin
            current_filepos:=tailineinfo(hp).fileinfo;
            { no line info for inlined code }
            if do_line and (inlinelevel=0) then
              WriteSourceLine(hp as tailineinfo);
          end;

         case hp.typ of

           ait_comment :
             Begin
               writer.AsmWrite(asminfo^.comment);
               writer.AsmWritePChar(tai_comment(hp).str);
               writer.AsmLn;
             End;

           ait_regalloc :
             begin
               if (cs_asm_regalloc in current_settings.globalswitches) then
                 begin
                   writer.AsmWrite(#9+asminfo^.comment+'Register ');
                   repeat
                     writer.AsmWrite(std_regname(Tai_regalloc(hp).reg));
                     if (hp.next=nil) or
                        (tai(hp.next).typ<>ait_regalloc) or
                        (tai_regalloc(hp.next).ratype<>tai_regalloc(hp).ratype) then
                       break;
                     hp:=tai(hp.next);
                     writer.AsmWrite(',');
                   until false;
                   writer.AsmWrite(' ');
                   writer.AsmWriteLn(regallocstr[tai_regalloc(hp).ratype]);
                 end;
             end;

           ait_tempalloc :
             begin
               if (cs_asm_tempalloc in current_settings.globalswitches) then
                 WriteTempalloc(tai_tempalloc(hp));
             end;

           ait_align :
             begin
               doalign(tai_align_abstract(hp).aligntype,tai_align_abstract(hp).use_op,tai_align_abstract(hp).fillop,tai_align_abstract(hp).maxbytes,last_align,lasthp);
             end;

           ait_section :
             begin
               if tai_section(hp).sectype<>sec_none then
                 if replaceforbidden then
                   WriteSection(tai_section(hp).sectype,ReplaceForbiddenAsmSymbolChars(tai_section(hp).name^),tai_section(hp).secorder,
                     tai_section(hp).secalign,tai_section(hp).secflags,tai_section(hp).secprogbits)
                 else
                   WriteSection(tai_section(hp).sectype,tai_section(hp).name^,tai_section(hp).secorder,
                     tai_section(hp).secalign,tai_section(hp).secflags,tai_section(hp).secprogbits)
               else
                 begin
{$ifdef EXTDEBUG}
                   writer.AsmWrite(asminfo^.comment);
                   writer.AsmWriteln(' sec_none');
{$endif EXTDEBUG}
                end;
             end;

           ait_datablock :
             begin
               if (target_info.system in systems_darwin) then
                 begin
                   { On Mac OS X you can't have common symbols in a shared library
                     since those are in the TEXT section and the text section is
                     read-only in shared libraries (so it can be shared among different
                     processes). The alternate code creates some kind of common symbols
                     in the data segment.
                   }

                   if tai_datablock(hp).is_global then
                     begin
                       if tai_datablock(hp).sym.bind=AB_PRIVATE_EXTERN then
                         WriteHiddenSymbol(tai_datablock(hp).sym);
                       writer.AsmWrite('.globl ');
                       writer.AsmWriteln(tai_datablock(hp).sym.name);
                       writer.AsmWriteln('.data');
                       writer.AsmWrite('.zerofill __DATA, __common, ');
                       writer.AsmWrite(tai_datablock(hp).sym.name);
                       writer.AsmWriteln(', '+tostr(tai_datablock(hp).size)+','+tostr(last_align));
                       if not(LastSecType in [sec_data,sec_none]) then
                         writesection(LastSecType,'',secorder_default,1 shl last_align);
                     end
                   else
                     begin
                       writer.AsmWrite(#9'.lcomm'#9);
                       writer.AsmWrite(tai_datablock(hp).sym.name);
                       writer.AsmWrite(','+tostr(tai_datablock(hp).size));
                       writer.AsmWrite(','+tostr(last_align));
                       writer.AsmLn;
                     end;
                 end
               else if target_info.system in systems_aix then
                 begin
                   if tai_datablock(hp).is_global then
                     begin
                       writer.AsmWrite(#9'.globl ');
                       writer.AsmWriteln(ReplaceForbiddenAsmSymbolChars(tai_datablock(hp).sym.name));
                       writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(tai_datablock(hp).sym.name));
                       writer.AsmWriteln(':');
                       writer.AsmWrite(#9'.space ');
                       writer.AsmWriteln(tostr(tai_datablock(hp).size));
                       if not(LastSecType in [sec_data,sec_none]) then
                         writesection(LastSecType,'',secorder_default,1 shl last_align);
                     end
                   else
                     begin
                       writer.AsmWrite(#9'.lcomm ');
                       writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(tai_datablock(hp).sym.name));
                       writer.AsmWrite(',');
                       writer.AsmWrite(tostr(tai_datablock(hp).size)+',');
                       writer.AsmWrite('_data.bss_,');
                       writer.AsmWriteln(tostr(last_align));
                     end;
                 end
               else
                 begin
{$ifdef USE_COMM_IN_BSS}
                   if writingpackages then
                     begin
                       { The .comm is required for COMMON symbols. These are used
                         in the shared library loading. All the symbols declared in
                         the .so file need to resolve to the data allocated in the main
                         program (PFV) }
                       if tai_datablock(hp).is_global then
                         begin
                           writer.AsmWrite(#9'.comm'#9);
                           if replaceforbidden then
                             writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(tai_datablock(hp).sym.name))
                           else
                             writer.AsmWrite(tai_datablock(hp).sym.name);
                           writer.AsmWrite(','+tostr(tai_datablock(hp).size));
                           writer.AsmWrite(','+tostr(last_align));
                           writer.AsmLn;
                         end
                       else
                         begin
                           writer.AsmWrite(#9'.lcomm'#9);
                           if replaceforbidden then
                             writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(tai_datablock(hp).sym.name));
                           else
                             writer.AsmWrite(tai_datablock(hp).sym.name);
                           writer.AsmWrite(','+tostr(tai_datablock(hp).size));
                           writer.AsmWrite(','+tostr(last_align));
                           writer.AsmLn;
                         end
                     end
                   else
{$endif USE_COMM_IN_BSS}
                     begin
                       if Tai_datablock(hp).is_global then
                         begin
                           if (tai_datablock(hp).sym.bind=AB_PRIVATE_EXTERN) then
                             WriteHiddenSymbol(tai_datablock(hp).sym);
                           writer.AsmWrite(#9'.globl ');
                           if replaceforbidden then
                             writer.AsmWriteln(ReplaceForbiddenAsmSymbolChars(Tai_datablock(hp).sym.name))
                           else
                             writer.AsmWriteln(Tai_datablock(hp).sym.name);
                         end;
                       if ((target_info.system <> system_arm_linux) and (target_info.system <> system_arm_android)) then
                         sepChar := '@'
                       else
                         sepChar := '%';
                       if replaceforbidden then
                         begin
                           if (tf_needs_symbol_type in target_info.flags) then
                             writer.AsmWriteln(#9'.type '+ReplaceForbiddenAsmSymbolChars(Tai_datablock(hp).sym.name)+','+sepChar+'object');
                           if (tf_needs_symbol_size in target_info.flags) and (tai_datablock(hp).size > 0) then
                              writer.AsmWriteln(#9'.size '+ReplaceForbiddenAsmSymbolChars(Tai_datablock(hp).sym.name)+','+tostr(Tai_datablock(hp).size));
                           writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(Tai_datablock(hp).sym.name))
                         end
                       else
                         begin
                           if (tf_needs_symbol_type in target_info.flags) then
                             writer.AsmWriteln(#9'.type '+Tai_datablock(hp).sym.name+','+sepChar+'object');
                           if (tf_needs_symbol_size in target_info.flags) and (tai_datablock(hp).size > 0) then
                             writer.AsmWriteln(#9'.size '+Tai_datablock(hp).sym.name+','+tostr(Tai_datablock(hp).size));
                           writer.AsmWrite(Tai_datablock(hp).sym.name);
                         end;
                       writer.AsmWriteln(':');
                       writer.AsmWriteln(#9'.zero '+tostr(Tai_datablock(hp).size));
                     end;
                 end;
             end;

           ait_const:
             begin
               constdef:=tai_const(hp).consttype;
               case constdef of
{$ifndef cpu64bitaddr}
                 aitconst_128bit :
                    begin
                      internalerror(200404291);
                    end;

                 aitconst_64bit :
                    begin
                      if assigned(tai_const(hp).sym) then
                        internalerror(200404292);
                      if not(target_info.system in systems_aix) then
                        begin
                          writer.AsmWrite(ait_const2str[aitconst_32bit]);
                          if target_info.endian = endian_little then
                            begin
                              writer.AsmWrite(tostr(longint(lo(tai_const(hp).value))));
                              writer.AsmWrite(',');
                              writer.AsmWrite(tostr(longint(hi(tai_const(hp).value))));
                            end
                          else
                            begin
                              writer.AsmWrite(tostr(longint(hi(tai_const(hp).value))));
                              writer.AsmWrite(',');
                              writer.AsmWrite(tostr(longint(lo(tai_const(hp).value))));
                            end;
                        end
                      else
                        WriteAixIntConst(tai_const(hp));
                      writer.AsmLn;
                    end;
                 aitconst_gottpoff:
                   begin
                     writer.AsmWrite(#9'.word'#9+tai_const(hp).sym.name+'(gottpoff)+(.-'+tai_const(hp).endsym.name+tostr_with_plus(tai_const(hp).symofs)+')');
                     writer.Asmln;
                   end;
                 aitconst_tlsgd:
                   begin
                     writer.AsmWrite(#9'.word'#9+tai_const(hp).sym.name+'(tlsgd)+(.-'+tai_const(hp).endsym.name+tostr_with_plus(tai_const(hp).symofs)+')');
                     writer.Asmln;
                   end;
                 aitconst_tlsdesc:
                   begin
                     writer.AsmWrite(#9'.word'#9+tai_const(hp).sym.name+'(tlsdesc)+(.-'+tai_const(hp).endsym.name+tostr_with_plus(tai_const(hp).symofs)+')');
                     writer.Asmln;
                   end;
                 aitconst_tpoff:
                   begin
                     if assigned(tai_const(hp).endsym) or (tai_const(hp).symofs<>0) then
                       Internalerror(2019092805);
                     writer.AsmWrite(#9'.word'#9+tai_const(hp).sym.name+'(tpoff)');
                     writer.Asmln;
                   end;
{$endif cpu64bitaddr}
                 aitconst_dtpoff:
                   begin
{$ifdef arm}
                     writer.AsmWrite(#9'.word'#9+tai_const(hp).sym.name+'(tlsldo)');
                     writer.Asmln;
{$endif arm}
{$ifdef x86_64}
                     writer.AsmWrite(#9'.long'#9+tai_const(hp).sym.name+'@dtpoff');
                     writer.Asmln;
{$endif x86_64}
{$ifdef i386}
                     writer.AsmWrite(#9'.word'#9+tai_const(hp).sym.name+'@tdpoff');
                     writer.Asmln;
{$endif i386}
                   end;
                 aitconst_got:
                   begin
                     if tai_const(hp).symofs<>0 then
                       InternalError(2015091401);  // No symbol offset is allowed for GOT.
                     writer.AsmWrite(#9'.word'#9+tai_const(hp).sym.name+'(GOT)');
                     writer.AsmLn;
                   end;

                 aitconst_gotoff_symbol:
                   begin
                     if (tai_const(hp).sym=nil) then
                       InternalError(2014022601);
                     case target_info.cpu of

                       cpu_mipseb,cpu_mipsel:
                         begin
                           writer.AsmWrite(#9'.gpword'#9);
                           writer.AsmWrite(tai_const(hp).sym.name);
                         end;

                       cpu_i386:
                         begin
                           writer.AsmWrite(ait_const2str[aitconst_32bit]);
                           writer.AsmWrite(tai_const(hp).sym.name+'-_GLOBAL_OFFSET_TABLE_');
                         end;
                     else
                       InternalError(2014022602);
                     end;
                     if (tai_const(hp).value<>0) then
                       writer.AsmWrite(tostr_with_plus(tai_const(hp).value));
                     writer.AsmLn;
                   end;

                 aitconst_uleb128bit,
                 aitconst_sleb128bit,
{$ifdef cpu64bitaddr}
                 aitconst_128bit,
                 aitconst_64bit,
{$endif cpu64bitaddr}
                 aitconst_32bit,
                 aitconst_16bit,
                 aitconst_8bit,
                 aitconst_rva_symbol,
                 aitconst_secrel32_symbol,
                 aitconst_darwin_dwarf_delta32,
                 aitconst_darwin_dwarf_delta64,
                 aitconst_half16bit,
                 aitconst_gs,
                 aitconst_16bit_unaligned,
                 aitconst_32bit_unaligned,
                 aitconst_64bit_unaligned:
                   begin
                     { the AIX assembler (and for compatibility, the GNU
                       assembler when targeting AIX) automatically aligns
                       .short/.long/.llong to a multiple of 2/4/8 bytes. We
                       don't want that, since this may be data inside a packed
                       record -> use .vbyte instead (byte stream of fixed
                       length) }
                     if (target_info.system in systems_aix) and
                        (constdef in [aitconst_128bit,aitconst_64bit,aitconst_32bit,aitconst_16bit]) and
                        not assigned(tai_const(hp).sym) then
                       begin
                         WriteAixIntConst(tai_const(hp));
                       end
                     else if (target_info.system in systems_darwin) and
                        (constdef in [aitconst_uleb128bit,aitconst_sleb128bit]) then
                       begin
                         writer.AsmWrite(ait_const2str[aitconst_8bit]);
                         case tai_const(hp).consttype of
                           aitconst_uleb128bit:
                             WriteDecodedUleb128(qword(tai_const(hp).value));
                           aitconst_sleb128bit:
                             WriteDecodedSleb128(int64(tai_const(hp).value));
                           else
                             ;
                         end
                       end
                     else
                       begin
                         if (constdef in ait_unaligned_consts) and
                            (target_info.system in use_ua_sparc_systems) then
                           writer.AsmWrite(ait_ua_sparc_const2str[constdef])
                         else if (constdef in ait_unaligned_consts) and
                                 (target_info.system in use_ua_elf_systems) then
                           writer.AsmWrite(ait_ua_elf_const2str[constdef])
                         { we can also have unaligned pointers in packed record
                           constants, which don't get translated into
                           unaligned tai -> always use vbyte }
                         else if target_info.system in systems_aix then
                            writer.AsmWrite(#9'.vbyte'#9+tostr(tai_const(hp).size)+',')
                         else if (asminfo^.id=as_solaris_as) then
                           writer.AsmWrite(ait_solaris_const2str[constdef])
                         else
                           writer.AsmWrite(ait_const2str[constdef]);
                         l:=0;
                         t := '';
                         repeat
                           if assigned(tai_const(hp).sym) then
                             begin
                               if assigned(tai_const(hp).endsym) then
                                 begin
                                   if (constdef in [aitconst_darwin_dwarf_delta32,aitconst_darwin_dwarf_delta64]) then
                                     begin
                                       s := NextSetLabel;
                                       t := #9'.set '+s+','+tai_const(hp).endsym.name+'-'+tai_const(hp).sym.name;
                                     end
                                   else
                                     s:=tai_const(hp).endsym.name+'-'+tai_const(hp).sym.name
                                  end
                               else
                                 s:=tai_const(hp).sym.name;
                               if replaceforbidden then
                                 s:=ReplaceForbiddenAsmSymbolChars(s);
                               if tai_const(hp).value<>0 then
                                 s:=s+tostr_with_plus(tai_const(hp).value);
                             end
                           else
{$ifdef cpu64bitaddr}
                             s:=tostr(tai_const(hp).value);
{$else cpu64bitaddr}
                             { 64 bit constants are already handled above in this case }
                             s:=tostr(longint(tai_const(hp).value));
{$endif cpu64bitaddr}
                           if constdef = aitconst_half16bit then
                             s:='('+s+')/2';
                           if constdef = aitconst_gs then
                             s:='gs('+s+')';

                           writer.AsmWrite(s);
                           inc(l,length(s));
                           { Values with symbols are written on a single line to improve
                             reading of the .s file (PFV) }
                           if assigned(tai_const(hp).sym) or
                              not(LastSecType in [sec_data,sec_rodata,sec_rodata_norel]) or
                              (l>line_length) or
                              (hp.next=nil) or
                              (tai(hp.next).typ<>ait_const) or
                              (tai_const(hp.next).consttype<>constdef) or
                              assigned(tai_const(hp.next).sym) then
                             break;
                           hp:=tai(hp.next);
                           writer.AsmWrite(',');
                         until false;
                         if (t <> '') then
                           begin
                             writer.AsmLn;
                             writer.AsmWrite(t);
                           end;
                       end;
                      writer.AsmLn;
                   end;
                 else
                   internalerror(200704251);
               end;
             end;

           ait_realconst :
             begin
               WriteRealConstAsBytes(tai_realconst(hp),#9'.byte'#9,do_line);
             end;

           ait_string :
             begin
               pos:=0;
               if not(target_info.system in systems_aix) then
                 begin
                   for i:=1 to tai_string(hp).len do
                    begin
                      if pos=0 then
                       begin
                         writer.AsmWrite(#9'.ascii'#9'"');
                         pos:=20;
                       end;
                      ch:=tai_string(hp).str[i-1];
                      case ch of
                                #0, {This can't be done by range, because a bug in FPC}
                           #1..#31,
                        #128..#255 : s:='\'+tostr(ord(ch) shr 6)+tostr((ord(ch) and 63) shr 3)+tostr(ord(ch) and 7);
                               '"' : s:='\"';
                               '\' : s:='\\';
                      else
                        s:=ch;
                      end;
                      writer.AsmWrite(s);
                      inc(pos,length(s));
                      if (pos>line_length) or (i=tai_string(hp).len) then
                       begin
                         writer.AsmWriteLn('"');
                         pos:=0;
                       end;
                    end;
                 end
               else
                 WriteAixStringConst(tai_string(hp));
             end;

           ait_label :
             begin
               if (tai_label(hp).labsym.is_used) then
                begin
                  if (tai_label(hp).labsym.bind=AB_PRIVATE_EXTERN) then
                    begin
                      writer.AsmWrite(#9'.private_extern ');
                      writer.AsmWriteln(tai_label(hp).labsym.name);
                    end;
                  if tai_label(hp).labsym.bind in [AB_GLOBAL,AB_PRIVATE_EXTERN] then
                   begin
{$ifdef arm}
                     { do no change arm mode accidently, .globl seems to reset the mode }
                     if GenerateThumbCode or GenerateThumb2Code then
                       writer.AsmWriteln(#9'.thumb_func'#9);
{$endif arm}
                     writer.AsmWrite('.globl'#9);
                     if replaceforbidden then
                       writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(tai_label(hp).labsym.name))
                     else
                       writer.AsmWriteLn(tai_label(hp).labsym.name);
                   end;
                  if replaceforbidden then
                    writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(tai_label(hp).labsym.name))
                  else
                    writer.AsmWrite(tai_label(hp).labsym.name);
                  writer.AsmWriteLn(':');
                end;
             end;

           ait_symbol :
             begin
               if (target_info.system=system_powerpc64_linux) and
                  (tai_symbol(hp).sym.typ=AT_FUNCTION) and
                  (cs_profile in current_settings.moduleswitches) then
                 writer.AsmWriteLn('.globl _mcount');

               if tai_symbol(hp).is_global then
                begin
                  writer.AsmWrite('.globl'#9);
                  if replaceforbidden then
                    writer.AsmWriteln(ReplaceForbiddenAsmSymbolChars(tai_symbol(hp).sym.name))
                  else
                    writer.AsmWriteln(tai_symbol(hp).sym.name);
                  if (tai_symbol(hp).sym.bind=AB_PRIVATE_EXTERN) then
                    WriteHiddenSymbol(tai_symbol(hp).sym);
                end;
               if (target_info.system=system_powerpc64_linux) and
                  use_dotted_functions and
                 (tai_symbol(hp).sym.typ=AT_FUNCTION) then
                 begin
                   writer.AsmWriteLn('.section ".opd", "aw"');
                   writer.AsmWriteLn('.align 3');
                   writer.AsmWriteLn(tai_symbol(hp).sym.name + ':');
                   writer.AsmWriteLn('.quad .' + tai_symbol(hp).sym.name + ', .TOC.@tocbase, 0');
                   writer.AsmWriteLn('.previous');
                   writer.AsmWriteLn('.size ' + tai_symbol(hp).sym.name + ', 24');
                   if (tai_symbol(hp).is_global) then
                     writer.AsmWriteLn('.globl .' + tai_symbol(hp).sym.name);
                   writer.AsmWriteLn('.type .' + tai_symbol(hp).sym.name + ', @function');
                   { the dotted name is the name of the actual function entry }
                   writer.AsmWrite('.');
                 end
               else if (target_info.system in systems_aix) and
                  (tai_symbol(hp).sym.typ = AT_FUNCTION) then
                 begin
                   if target_info.system=system_powerpc_aix then
                     begin
                       s:=#9'.long .';
                       ch:='2';
                     end
                   else
                     begin
                       s:=#9'.llong .';
                       ch:='3';
                     end;
                   writer.AsmWriteLn(#9'.csect '+ReplaceForbiddenAsmSymbolChars(tai_symbol(hp).sym.name)+'[DS],'+ch);
                   writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(tai_symbol(hp).sym.name)+':');
                   writer.AsmWriteln(s+ReplaceForbiddenAsmSymbolChars(tai_symbol(hp).sym.name)+', TOC[tc0], 0');
                   writer.AsmWriteln(#9'.csect .text[PR]');
                   if (tai_symbol(hp).is_global) then
                     writer.AsmWriteLn('.globl .'+ReplaceForbiddenAsmSymbolChars(tai_symbol(hp).sym.name))
                   else
                     writer.AsmWriteLn('.lglobl .'+ReplaceForbiddenAsmSymbolChars(tai_symbol(hp).sym.name));
                   { the dotted name is the name of the actual function entry }
                   writer.AsmWrite('.');
                 end
               else
                 begin
                   if ((target_info.system <> system_arm_linux) and (target_info.system <> system_arm_android)) then
                     sepChar := '@'
                   else
                     sepChar := '#';
                   if (tf_needs_symbol_type in target_info.flags) then
                     begin
                       writer.AsmWrite(#9'.type'#9 + tai_symbol(hp).sym.name);
                       if (needsObject(tai_symbol(hp))) then
                         writer.AsmWriteLn(',' + sepChar + 'object')
                       else
                         writer.AsmWriteLn(',' + sepChar + 'function');
                     end;
                 end;
               if replaceforbidden then
                 if not(tai_symbol(hp).has_value) then
                   writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(tai_symbol(hp).sym.name + ':'))
                 else
                   writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(tai_symbol(hp).sym.name + '=' + tostr(tai_symbol(hp).value)))
               else if not(tai_symbol(hp).has_value) then
                 writer.AsmWriteLn(tai_symbol(hp).sym.name + ':')
               else
                 writer.AsmWriteLn(tai_symbol(hp).sym.name + '=' + tostr(tai_symbol(hp).value));
             end;
           ait_symbolpair:
             begin
               writer.AsmWrite(#9);
               writer.AsmWrite(symbolpairkindstr[tai_symbolpair(hp).kind]);
               writer.AsmWrite(' ');
               if tai_symbolpair(hp).kind<>spk_localentry then
                 s:=', '
               else
                 { the .localentry directive has to specify the size from the
                   start till here of the non-local entry code as second argument }
                 s:=', .-';
               if replaceforbidden then
                 begin
                   { avoid string truncation }
                   writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(tai_symbolpair(hp).sym^));
                   writer.AsmWrite(s);
                   writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(tai_symbolpair(hp).value^));
                   if tai_symbolpair(hp).kind=spk_set_global then
                     begin
                       writer.AsmWrite(#9'.globl ');
                       writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(tai_symbolpair(hp).sym^));
                     end;
                 end
               else
                 begin
                   { avoid string truncation }
                   writer.AsmWrite(tai_symbolpair(hp).sym^);
                   writer.AsmWrite(s);
                   writer.AsmWriteLn(tai_symbolpair(hp).value^);
                   if tai_symbolpair(hp).kind=spk_set_global then
                     begin
                       writer.AsmWrite(#9'.globl ');
                       writer.AsmWriteLn(tai_symbolpair(hp).sym^);
                     end;
                 end;
             end;
           ait_symbol_end :
             begin
               if tf_needs_symbol_size in target_info.flags then
                begin
                  s:=asminfo^.labelprefix+'e'+tostr(symendcount);
                  inc(symendcount);
                  writer.AsmWriteLn(s+':');
                  writer.AsmWrite(#9'.size'#9);
                  if (target_info.system=system_powerpc64_linux) and
                     use_dotted_functions and
                     (tai_symbol_end(hp).sym.typ=AT_FUNCTION) then
                    writer.AsmWrite('.');
                  if replaceforbidden then
                    writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(tai_symbol_end(hp).sym.name))
                  else
                    writer.AsmWrite(tai_symbol_end(hp).sym.name);
                  writer.AsmWrite(', '+s+' - ');
                  if (target_info.system=system_powerpc64_linux) and
                     use_dotted_functions and
                     (tai_symbol_end(hp).sym.typ=AT_FUNCTION) then
                    writer.AsmWrite('.');
                  if replaceforbidden then
                    writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(tai_symbol_end(hp).sym.name))
                  else
                    writer.AsmWriteLn(tai_symbol_end(hp).sym.name);
                end;
             end;

           ait_instruction :
             begin
               WriteInstruction(hp);
             end;

           ait_stab :
             begin
               if assigned(tai_stab(hp).str) then
                 begin
                   writer.AsmWrite(#9'.'+stabtypestr[tai_stab(hp).stabtype]+' ');
                   writer.AsmWritePChar(tai_stab(hp).str);
                   writer.AsmLn;
                 end;
             end;

           ait_force_line,
           ait_function_name :
             begin
{$ifdef DEBUG_AGGAS}
               WriteStr(s,hp.typ);
               writer.AsmWriteLn('# '+s);
{$endif DEBUG_AGGAS}
             end;

           ait_cutobject :
             begin
{$ifdef DEBUG_AGGAS}
               writer.AsmWriteLn('# ait_cutobject');
{$endif DEBUG_AGGAS}
               if SmartAsm then
                begin
                { only reset buffer if nothing has changed }
                  if not(writer.ClearIfEmpty) then
                   begin
                     writer.AsmClose;
                     DoAssemble;
                     writer.AsmCreate(tai_cutobject(hp).place);
                   end;
                { avoid empty files }
                  while assigned(hp.next) and (tai(hp.next).typ in [ait_cutobject,ait_section,ait_comment]) do
                   begin
                     if tai(hp.next).typ=ait_section then
                       LastSecType:=tai_section(hp.next).sectype;
                     hp:=tai(hp.next);
                   end;
                  if LastSecType<>sec_none then
                    WriteSection(LastSecType,'',secorder_default,last_align);
                  writer.MarkEmpty;
                end;
             end;

           ait_marker :
             begin
{$ifdef DEBUG_AGGAS}
               WriteStr(s,tai_marker(hp).Kind);
               writer.AsmWriteLn('# ait_marker, kind: '+s);
{$endif DEBUG_AGGAS}
               if tai_marker(hp).kind=mark_NoLineInfoStart then
                 inc(InlineLevel)
               else if tai_marker(hp).kind=mark_NoLineInfoEnd then
                 dec(InlineLevel);
             end;

           ait_directive :
             begin
               WriteDirectiveName(tai_directive(hp).directive);
               if tai_directive(hp).name <>'' then
                 begin
                   if replaceforbidden then
                     writer.AsmWrite(ReplaceForbiddenAsmSymbolChars(tai_directive(hp).name))
                   else
                     writer.AsmWrite(tai_directive(hp).name);
                 end;
               writer.AsmLn;
             end;

           ait_seh_directive :
             begin
{$ifndef DISABLE_WIN64_SEH}
               writer.AsmWrite(sehdirectivestr[tai_seh_directive(hp).kind]);
               case tai_seh_directive(hp).datatype of
                 sd_none:;
                 sd_string:
                   begin
                     writer.AsmWrite(' '+tai_seh_directive(hp).data.name^);
                     if (tai_seh_directive(hp).data.flags and 1)<>0 then
                       writer.AsmWrite(',@except');
                     if (tai_seh_directive(hp).data.flags and 2)<>0 then
                       writer.AsmWrite(',@unwind');
                   end;
                 sd_reg:
                   writer.AsmWrite(' '+gas_regname(tai_seh_directive(hp).data.reg));
                 sd_offset:
                   writer.AsmWrite(' '+tostr(tai_seh_directive(hp).data.offset));
                 sd_regoffset:
                   writer.AsmWrite(' '+gas_regname(tai_seh_directive(hp).data.reg)+', '+
                     tostr(tai_seh_directive(hp).data.offset));
               end;
               writer.AsmLn;
{$endif DISABLE_WIN64_SEH}
             end;

           ait_varloc:
             begin
               if tai_varloc(hp).newlocationhi<>NR_NO then
                 writer.AsmWrite(strpnew('Var '+tai_varloc(hp).varsym.realname+' located in register '+
                   std_regname(tai_varloc(hp).newlocationhi)+':'+std_regname(tai_varloc(hp).newlocation)))
               else
                 writer.AsmWrite(strpnew('Var '+tai_varloc(hp).varsym.realname+' located in register '+
                   std_regname(tai_varloc(hp).newlocation)));
               writer.AsmLn;
             end;
           ait_cfi:
             begin
               WriteCFI(tai_cfi_base(hp));
             end;
           ait_eabi_attribute:
             begin
               case tai_eabi_attribute(hp).eattr_typ of
                 eattrtype_dword:
                   writer.AsmWrite(#9'.eabi_attribute '+tostr(tai_eabi_attribute(hp).tag)+','+tostr(tai_eabi_attribute(hp).value));
                 eattrtype_ntbs:
                   writer.AsmWrite(#9'.eabi_attribute '+tostr(tai_eabi_attribute(hp).tag)+',"'+tai_eabi_attribute(hp).valuestr^+'"');
                 else
                   Internalerror(2019100601);
               end;
               writer.AsmLn;
             end;
           else
             internalerror(2006012201);
         end;
         lasthp:=hp;
         hp:=tai(hp.next);
       end;
    end;


    procedure TGNUAssembler.WriteExtraHeader;
      begin
      end;


    procedure TGNUAssembler.WriteExtraFooter;
      begin
      end;


    procedure TGNUAssembler.WriteInstruction(hp: tai);
      begin
        InstrWriter.WriteInstruction(hp);
      end;


    procedure TGNUAssembler.WriteWeakSymbolRef(s: tasmsymbol);
      begin
        writer.AsmWrite(#9'.weak ');
        if asminfo^.dollarsign='$' then
          writer.AsmWriteLn(s.name)
        else
          writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(s.name))
      end;


    procedure TGNUAssembler.WriteHiddenSymbol(sym: TAsmSymbol);
      begin
        { on Windows/(PE)COFF, global symbols are hidden by default: global
          symbols that are not explicitly exported from an executable/library,
          become hidden }
        if target_info.system in systems_windows then
          exit;
        if target_info.system in systems_darwin then
          writer.AsmWrite(#9'.private_extern ')
        else
          writer.AsmWrite(#9'.hidden ');
        if asminfo^.dollarsign='$' then
          writer.AsmWriteLn(sym.name)
        else
          writer.AsmWriteLn(ReplaceForbiddenAsmSymbolChars(sym.name))
      end;


    procedure TGNUAssembler.WriteAixStringConst(hp: tai_string);
      type
        tterminationkind = (term_none,term_string,term_nostring);

      var
        i: longint;
        pos: longint;
        s: string;
        ch: char;
        instring: boolean;

      procedure newstatement(terminationkind: tterminationkind);
        begin
          case terminationkind of
            term_none: ;
            term_string:
              writer.AsmWriteLn('"');
            term_nostring:
              writer.AsmLn;
          end;
          writer.AsmWrite(#9'.byte'#9);
          pos:=20;
          instring:=false;
        end;

      begin
        pos:=0;
        instring:=false;
        for i:=1 to hp.len do
          begin
            if pos=0 then
              newstatement(term_none);
            ch:=hp.str[i-1];
            case ch of
              #0..#31,
              #127..#255 :
                begin
                  if instring then
                    newstatement(term_string);
                  if pos=20 then
                    s:=tostr(ord(ch))
                  else
                    s:=', '+tostr(ord(ch))
                end;
              '"' :
                if instring then
                  s:='""'
                else
                  begin
                    if pos<>20 then
                      newstatement(term_nostring);
                    s:='"""';
                    instring:=true;
                  end;
              else
                if not instring then
                  begin
                    if (pos<>20) then
                      newstatement(term_nostring);
                    s:='"'+ch;
                    instring:=true;
                  end
                else
                  s:=ch;
            end;
            writer.AsmWrite(s);
            inc(pos,length(s));
            if (pos>line_length) or (i=tai_string(hp).len) then
              begin
                if instring then
                  writer.AsmWriteLn('"')
                else
                  writer.AsmLn;
                pos:=0;
              end;
         end;
      end;


    procedure TGNUAssembler.WriteAixIntConst(hp: tai_const);
      var
        pos, size: longint;
      begin
        { only big endian AIX supported for now }
        if target_info.endian<>endian_big then
          internalerror(2012010401);
        { limitation: can only write 4 bytes at a time }
        pos:=0;
        size:=tai_const(hp).size;
        while pos<(size-4) do
          begin
            writer.AsmWrite(#9'.vbyte'#9'4, ');
            writer.AsmWriteln(tostr(longint(tai_const(hp).value shr ((size-pos-4)*8))));
            inc(pos,4);
         end;
        writer.AsmWrite(#9'.vbyte'#9);
        writer.AsmWrite(tostr(size-pos));
        writer.AsmWrite(', ');
        case size-pos of
          1: writer.AsmWrite(tostr(byte(tai_const(hp).value)));
          2: writer.AsmWrite(tostr(word(tai_const(hp).value)));
          4: writer.AsmWrite(tostr(longint(tai_const(hp).value)));
          else
            internalerror(2012010402);
        end;
      end;

    procedure TGNUAssembler.WriteUnalignedIntConst(hp: tai_const);
      var
        pos, size: longint;
      begin
        size:=tai_const(hp).size;
        writer.AsmWrite(#9'.byte'#9);
        if target_info.endian=endian_big then
          begin
            pos:=size-1;
            while pos>=0 do
              begin
                writer.AsmWrite(tostr((tai_const(hp).value shr (pos*8)) and $ff));
                dec(pos);
                if pos>=0 then
                  writer.AsmWrite(', ')
                else
                  writer.AsmLn;
              end;
          end
        else
          begin
            pos:=0;
            while pos<size do
              begin
                writer.AsmWriteln(tostr((tai_const(hp).value shr (pos*8)) and $ff));
                inc(pos);
                if pos<=size then
                  writer.AsmWrite(', ')
                else
                  writer.AsmLn;
              end;
          end;
        writer.AsmLn;
      end;


    procedure TGNUAssembler.WriteDirectiveName(dir: TAsmDirective);
    begin
      { TODO: implement asd_cpu for GAS => usually .arch or .cpu, but the CPU
        name has to be translated as well }
      if dir=asd_cpu then
        writer.AsmWrite(asminfo^.comment+' CPU ')
      else
        writer.AsmWrite('.'+directivestr[dir]+' ');
    end;


    procedure TGNUAssembler.WriteAsmList;
    var
      n : string;
      hal : tasmlisttype;
      i: longint;
    begin
{$ifdef EXTDEBUG}
      if current_module.mainsource<>'' then
       Comment(V_Debug,'Start writing gas-styled assembler output for '+current_module.mainsource);
{$endif}

      if current_module.mainsource<>'' then
        n:=ExtractFileName(current_module.mainsource)
      else
        n:=InputFileName;

      { gcc does not add it either for Darwin. Grep for
        TARGET_ASM_FILE_START_FILE_DIRECTIVE in gcc/config/*.h
      }
      if not(target_info.system in systems_darwin) then
        writer.AsmWriteLn(#9'.file "'+FixFileName(n)+'"');

      WriteExtraHeader;
      writer.MarkEmpty;
      symendcount:=0;

      for hal:=low(TasmlistType) to high(TasmlistType) do
        begin
          if not (current_asmdata.asmlists[hal].empty) then
            begin
              writer.AsmWriteLn(asminfo^.comment+'Begin asmlist '+AsmlistTypeStr[hal]);
              writetree(current_asmdata.asmlists[hal]);
              writer.AsmWriteLn(asminfo^.comment+'End asmlist '+AsmlistTypeStr[hal]);
            end;
        end;

      { add weak symbol markers }
      for i:=0 to current_asmdata.asmsymboldict.count-1 do
        if (tasmsymbol(current_asmdata.asmsymboldict[i]).bind=AB_WEAK_EXTERNAL) then
          WriteWeakSymbolRef(tasmsymbol(current_asmdata.asmsymboldict[i]));

      if create_smartlink_sections and
         (target_info.system in systems_darwin) then
        writer.AsmWriteLn(#9'.subsections_via_symbols');

      { "no executable stack" marker }
      { TODO: used by OpenBSD/NetBSD as well? }
      if (target_info.system in (systems_linux + systems_android + systems_freebsd + systems_dragonfly)) and
         not(cs_executable_stack in current_settings.moduleswitches) then
        begin
          writer.AsmWriteLn('.section .note.GNU-stack,"",%progbits');
        end;

      writer.AsmLn;
      WriteExtraFooter;
{$ifdef EXTDEBUG}
      if current_module.mainsource<>'' then
       Comment(V_Debug,'Done writing gas-styled assembler output for '+current_module.mainsource);
{$endif EXTDEBUG}
    end;


{****************************************************************************}
{                        Apple/GNU Assembler writer                          }
{****************************************************************************}

    function TAppleGNUAssembler.sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;
      begin
        if (target_info.system in systems_darwin) then
          case atype of
            sec_user:
              begin
                result:='.section '+aname;
                exit;
              end;
            sec_bss:
              { all bss (lcomm) symbols are automatically put in the right }
              { place by using the lcomm assembler directive               }
              atype := sec_none;
            sec_debug_frame,
            sec_eh_frame:
              begin
                result := '.section __DWARF,__debug_info,regular,debug';
                exit;
              end;
            sec_debug_line:
              begin
                result := '.section __DWARF,__debug_line,regular,debug';
                exit;
              end;
            sec_debug_info:
              begin
                result := '.section __DWARF,__debug_info,regular,debug';
                exit;
              end;
            sec_debug_abbrev:
               begin
                 result := '.section __DWARF,__debug_abbrev,regular,debug';
                 exit;
               end;
            sec_debug_aranges:
               begin
                 result := '.section __DWARF,__debug_aranges,regular,debug';
                 exit;
               end;
            sec_debug_ranges:
               begin
                 result := '.section __DWARF,__debug_ranges,regular,debug';
                 exit;
               end;
            sec_rodata:
              begin
                result := '.const_data';
                exit;
              end;
            sec_rodata_norel:
              begin
                result := '.const';
                exit;
              end;
            sec_fpc:
              begin
                result := '.section __TEXT, .fpc, regular, no_dead_strip';
                exit;
              end;
            sec_code:
              begin
                if (aname='fpc_geteipasebx') or
                   (aname='fpc_geteipasecx') then
                  begin
                    result:='.section __TEXT,__textcoal_nt,coalesced,pure_instructions'#10'.weak_definition '+aname+
                      #10'.private_extern '+aname;
                    exit;
                  end;
              end;
            sec_data_nonlazy:
              begin
                result:='.section __DATA, __nl_symbol_ptr,non_lazy_symbol_pointers';
                exit;
              end;
            sec_data_lazy:
              begin
                result:='.section __DATA, __la_symbol_ptr,lazy_symbol_pointers';
                exit;
              end;
            sec_init_func:
              begin
                result:='.section __DATA, __mod_init_func, mod_init_funcs';
                exit;
              end;
            sec_term_func:
              begin
                result:='.section __DATA, __mod_term_func, mod_term_funcs';
                exit;
              end;
            low(TObjCAsmSectionType)..high(TObjCAsmSectionType):
              begin
                result:='.section '+objc_section_name(atype);
                exit
              end;
            else
              ;
          end;
        result := inherited sectionname(atype,aname,aorder);
      end;


    procedure TAppleGNUAssembler.WriteWeakSymbolRef(s: tasmsymbol);
      begin
        writer.AsmWriteLn(#9'.weak_reference '+s.name);
      end;

    procedure TAppleGNUAssembler.WriteDirectiveName(dir: TAsmDirective);
      begin
        case dir of
          asd_weak_reference:
            writer.AsmWrite('.weak_reference ');
          asd_weak_definition:
            writer.AsmWrite('.weak_definition ');
          else
            inherited;
        end;
      end;


{****************************************************************************}
{                       a.out/GNU Assembler writer                           }
{****************************************************************************}

    function TAoutGNUAssembler.sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;
    const
(* Translation table - replace unsupported section types with basic ones. *)
        SecXTable: array[TAsmSectionType] of TAsmSectionType = (
         sec_none,
         sec_none,
         sec_code,
         sec_data,
         sec_data (* sec_rodata *),
         sec_data (* sec_rodata_norel *),
         sec_bss,
         sec_data (* sec_threadvar *),
         { used for wince exception handling }
         sec_code (* sec_pdata *),
         { used for darwin import stubs }
         sec_code (* sec_stub *),
         sec_data,(* sec_data_nonlazy *)
         sec_data,(* sec_data_lazy *)
         sec_data,(* sec_init_func *)
         sec_data,(* sec_term_func *)
         { stabs }
         sec_stab,sec_stabstr,
         { win32 }
         sec_data (* sec_idata2 *),
         sec_data (* sec_idata4 *),
         sec_data (* sec_idata5 *),
         sec_data (* sec_idata6 *),
         sec_data (* sec_idata7 *),
         sec_data (* sec_edata *),
         { C++ exception handling unwinding (uses dwarf) }
         sec_eh_frame,
         { dwarf }
         sec_debug_frame,
         sec_debug_info,
         sec_debug_line,
         sec_debug_abbrev,
         sec_debug_aranges,
         sec_debug_ranges,
         { ELF resources (+ references to stabs debug information sections) }
         sec_code (* sec_fpc *),
         { Table of contents section }
         sec_code (* sec_toc *),
         sec_code (* sec_init *),
         sec_code (* sec_fini *),
         sec_none (* sec_objc_class *),
         sec_none (* sec_objc_meta_class *),
         sec_none (* sec_objc_cat_cls_meth *),
         sec_none (* sec_objc_cat_inst_meth *),
         sec_none (* sec_objc_protocol *),
         sec_none (* sec_objc_string_object *),
         sec_none (* sec_objc_cls_meth *),
         sec_none (* sec_objc_inst_meth *),
         sec_none (* sec_objc_cls_refs *),
         sec_none (* sec_objc_message_refs *),
         sec_none (* sec_objc_symbols *),
         sec_none (* sec_objc_category *),
         sec_none (* sec_objc_class_vars *),
         sec_none (* sec_objc_instance_vars *),
         sec_none (* sec_objc_module_info *),
         sec_none (* sec_objc_class_names *),
         sec_none (* sec_objc_meth_var_types *),
         sec_none (* sec_objc_meth_var_names *),
         sec_none (* sec_objc_selector_strs *),
         sec_none (* sec_objc_protocol_ext *),
         sec_none (* sec_objc_class_ext *),
         sec_none (* sec_objc_property *),
         sec_none (* sec_objc_image_info *),
         sec_none (* sec_objc_cstring_object *),
         sec_none (* sec_objc_sel_fixup *),
         sec_none (* sec_objc_data *),
         sec_none (* sec_objc_const *),
         sec_none (* sec_objc_sup_refs *),
         sec_none (* sec_data_coalesced *),
         sec_none (* sec_objc_classlist *),
         sec_none (* sec_objc_nlclasslist *),
         sec_none (* sec_objc_catlist *),
         sec_none (* sec_objc_nlcatlist *),
         sec_none (* sec_objc_protlist *),
         sec_none (* sec_stack *),
         sec_none (* sec_heap *),
         sec_none (* gcc_except_table *),
         sec_none (* sec_arm_attribute *)
        );
      begin
        Result := inherited SectionName (SecXTable [AType], AName, AOrder);
      end;


{****************************************************************************}
{                        Abstract Instruction Writer                         }
{****************************************************************************}

     constructor TCPUInstrWriter.create(_owner: TGNUAssembler);
       begin
         inherited create;
         owner := _owner;
       end;

end.
