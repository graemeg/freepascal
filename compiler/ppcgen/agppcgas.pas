{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit the GAS asm writers for PowerPC/PowerPC64

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

{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

unit agppcgas;

{$i fpcdefs.inc}

  interface

    uses
       systems,aasmbase,
       aasmtai,aasmdata,
       assemble,aggas,
       cpubase,cgutils,
       globtype;

  type
    TPPCInstrWriter=class(TCPUInstrWriter)
       procedure WriteInstruction(hp : tai);override;
    end;

    TPPCGNUAssembler=class(TGNUassembler)
      constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      function MakeCmdLine: TCmdStr; override;
      procedure WriteExtraHeader; override;
    end;

    TPPCAppleGNUAssembler=class(TAppleGNUassembler)
      constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      function MakeCmdLine: TCmdStr; override;
    end;

    TPPCAIXAssembler=class(TPPCGNUAssembler)
      max_alignment : array[TAsmSectionType] of longint;
      constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
     protected
      function sectionname(atype: TAsmSectiontype; const aname: string; aorder: TAsmSectionOrder): string; override;
      procedure WriteSection(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder;secalign:longint;secflags:TSectionFlags=[];secprogbits:TSectionProgbits=SPB_None); override;
      procedure WriteAsmList; override;
      procedure WriteExtraHeader; override;
      procedure WriteExtraFooter; override;
      procedure WriteDirectiveName(dir: TAsmDirective); override;
    end;

    topstr = string[4];

    function branchmode(o: tasmop): topstr;
    function cond2str(op: tasmop; c: tasmcond): string;  

  implementation

    uses
       cutils,globals,verbose,
       cgbase,
       itcpugas,cpuinfo,
       aasmcpu;

{$ifdef cpu64bitaddr}
    const
      refaddr2str: array[trefaddr] of string[9] = ('', '', '', '', '@l', '@h', '@higher', '@highest', '@ha', '@highera', '@highesta');
      verbose_refaddrs = [addr_low, addr_high, addr_higher, addr_highest, addr_higha, addr_highera, addr_highesta];
      refaddr2str_darwin: array[trefaddr] of string[4] = ('','','','','lo16', 'hi16', '@err', '@err', 'ha16', '@err', '@err');
{$else cpu64bitaddr}
    const
      refaddr2str: array[trefaddr] of string[3] = ('','','','','@l','@h','@ha');
      refaddr2str_darwin: array[trefaddr] of string[4] = ('','','','','lo16','hi16','ha16');
      verbose_refaddrs = [addr_low,addr_high,addr_higha];
{$endif cpu64bitaddr}


    function getreferencestring(asminfo: pasminfo; var ref : treference) : string;
    var
      s : string;
    begin
       with ref do
        begin
          if ((offset < -32768) or (offset > 32767)) and
             (refaddr = addr_no) then
            internalerror(2006052501);
          case refaddr of
            addr_no:
              s := '';
            addr_pic_no_got:
              begin
                { used for TOC-based loads }
                if (base<>NR_RTOC) or
                   (index<>NR_NO) or
                   (offset<>0) or
                   not assigned(symbol) then
                  internalerror(2011122701);
                if asminfo^.dollarsign<>'$' then
                  getreferencestring:=ReplaceForbiddenAsmSymbolChars(symbol.name)+'('+gas_regname(NR_RTOC)+')'
                else
                  getreferencestring:=symbol.name+'('+gas_regname(NR_RTOC)+')';
                exit;
              end
            else
              begin
                if target_info.system in [system_powerpc_darwin,system_powerpc64_darwin] then
                  s := refaddr2str_darwin[refaddr]
                else
                  s :='';
                s := s+'(';
                if assigned(symbol) then
                  begin
                    if asminfo^.dollarsign<>'$' then
                      begin
                        s:=s+ReplaceForbiddenAsmSymbolChars(symbol.name);
                        if assigned(relsymbol) then
                          s:=s+'-'+ReplaceForbiddenAsmSymbolChars(relsymbol.name)
                      end
                    else
                      begin
                        s:=s+symbol.name;
                        if assigned(relsymbol) then
                          s:=s+'-'+relsymbol.name;
                      end;
                  end;
              end;
          end;
          if offset<0 then
           s:=s+tostr(offset)
          else
           if (offset>0) then
            begin
              if assigned(symbol) then
                s:=s+'+'+tostr(offset)
              else
                s:=s+tostr(offset);
            end;

           if not(refaddr in [addr_no,addr_pic_no_got]) then
             begin
               s := s+')';
               if (refaddr in verbose_refaddrs) and
                  not(target_info.system in [system_powerpc_darwin,system_powerpc64_darwin]) then
                 s := s+refaddr2str[refaddr];
             end;
{$ifdef cpu64bitaddr}
           if (refaddr=addr_pic) and
              (target_info.system=system_powerpc64_linux) then
             s := s + '@got';
{$endif cpu64bitaddr}

           if (index=NR_NO) then
             begin
                if offset=0 then
                  begin
                    if not (assigned(symbol)) then
                      s:=s+'0';
                  end;
                if (base<>NR_NO) then
                  s:=s+'('+gas_regname(base)+')'
                else if not assigned(symbol) and
                        not(refaddr in verbose_refaddrs) then
                  s:=s+'(0)';
             end
           else if (index<>NR_NO) and (base<>NR_NO) then
             begin
               if (offset=0) then
                 s:=s+gas_regname(base)+','+gas_regname(index)
               else
                 internalerror(2006052502);
             end;
        end;
      getreferencestring:=s;
    end;


    function getopstr_jmp(asminfo: pasminfo; const o:toper) : string;
    var
      hs : string;
    begin
      case o.typ of
        top_reg :
          getopstr_jmp:=gas_regname(o.reg);
        { no top_ref jumping for powerpc }
        top_const :
          getopstr_jmp:=tostr(o.val);
        top_ref :
          begin
            if o.ref^.refaddr<>addr_full then
              internalerror(200402267);
            hs:=o.ref^.symbol.name;
            if asminfo^.dollarsign<>'$' then
              hs:=ReplaceForbiddenAsmSymbolChars(hs);
            if o.ref^.offset>0 then
              hs:=hs+'+'+tostr(o.ref^.offset)
            else
             if o.ref^.offset<0 then
              hs:=hs+tostr(o.ref^.offset);
            getopstr_jmp:=hs;
          end;
        top_none:
          getopstr_jmp:='';
        else
          internalerror(2002070603);
      end;
    end;


    function getopstr(asminfo: pasminfo; const o:toper) : string;
    var
      hs : string;
    begin
      case o.typ of
        top_reg:
          getopstr:=gas_regname(o.reg);
        top_const:
          getopstr:=tostr(longint(o.val));
        top_ref:
          if o.ref^.refaddr=addr_full then
            begin
              hs:=o.ref^.symbol.name;
              if asminfo^.dollarsign<>'$' then
                hs:=ReplaceForbiddenAsmSymbolChars(hs);
              if o.ref^.offset>0 then
               hs:=hs+'+'+tostr(o.ref^.offset)
              else
               if o.ref^.offset<0 then
                hs:=hs+tostr(o.ref^.offset);
              getopstr:=hs;
            end
          else
            getopstr:=getreferencestring(asminfo,o.ref^);
        else
          internalerror(2002070604);
      end;
    end;


    function branchmode(o: tasmop): topstr;
      var tempstr: topstr;
      begin
        tempstr := '';
        case o of
          A_BCCTR,A_BCCTRL: tempstr := 'ctr';
          A_BCLR,A_BCLRL: tempstr := 'lr';
          else
            ;
        end;
        case o of
          A_BL,A_BLA,A_BCL,A_BCLA,A_BCCTRL,A_BCLRL: tempstr := tempstr+'l';
          else
            ;
        end;
        case o of
          A_BA,A_BLA,A_BCA,A_BCLA: tempstr:=tempstr+'a';
          else
            ;
        end;
        branchmode := tempstr;
      end;


    function cond2str(op: tasmop; c: tasmcond): string;
    { note: no checking is performed whether the given combination of }
    { conditions is valid                                             }
    var
      tempstr: string;
    begin
      tempstr:=#9;
      case c.simple of
        false:
          begin
            cond2str := tempstr+gas_op2str[op];
            case c.dirhint of
              DH_None:;
              DH_Minus:
                cond2str:=cond2str+'-';
              DH_Plus:
                cond2str:=cond2str+'+';
            end;
            cond2str:=cond2str+#9+tostr(c.bo)+','+tostr(c.bi);
          end;
        true:
          if (op >= A_B) and (op <= A_BCLRL) then
            case c.cond of
              { unconditional branch }
              C_NONE:
                cond2str := tempstr+gas_op2str[op];
              { bdnzt etc }
              else
                begin
                  tempstr := tempstr+'b'+asmcondflag2str[c.cond]+
                              branchmode(op);
                  case c.dirhint of
                    DH_None:
                      tempstr:=tempstr+#9;
                    DH_Minus:
                      tempstr:=tempstr+('-'+#9);
                    DH_Plus:
                      tempstr:=tempstr+('+'+#9);
                  end;
                  case c.cond of
                    C_LT..C_NU:
                      begin
                        if byte(c.cr)=0 then
                          Comment(V_error,'Wrong use of whole CR register for '+tempstr);
                        cond2str := tempstr+gas_regname(newreg(R_SPECIALREGISTER,c.cr,R_SUBWHOLE));
                      end;
                    C_T,C_F,C_DNZT,C_DNZF,C_DZT,C_DZF:
                      cond2str := tempstr+tostr(c.crbit);
                    else
                      cond2str := tempstr;
                  end;
                end;
            end
          { we have a trap instruction }
          else
            begin
              internalerror(2002070601);
              { not yet implemented !!!!!!!!!!!!!!!!!!!!! }
              { case tempstr := 'tw';}
            end;
      end;
    end;


{****************************************************************************}
{                        PowerPC Instruction Writer                          }
{****************************************************************************}

    Procedure TPPCInstrWriter.WriteInstruction(hp : tai);
    var op: TAsmOp;
        s: string;
        i: byte;
        sep: string[3];
    begin
      op:=taicpu(hp).opcode;
      if is_calljmp(op) then
        begin
          { direct BO/BI in op[0] and op[1] not supported, put them in condition! }
          case op of
             A_B,A_BA,A_BL,A_BLA:
               s:=#9+gas_op2str[op]+#9;
             A_BCTR,A_BCTRL,A_BLR,A_BLRL:
               s:=#9+gas_op2str[op]
             else
               begin
                 s:=cond2str(op,taicpu(hp).condition);
                 if (s[length(s)] <> #9) and
                    (taicpu(hp).ops>0) then
                   s := s + ',';
               end;
          end;

          if (taicpu(hp).ops>0) and (taicpu(hp).oper[0]^.typ<>top_none) then
            begin
              { first write the current contents of s, because the symbol }
              { may be 255 characters                                     }
              owner.writer.AsmWrite(s);
              s:=getopstr_jmp(owner.asminfo,taicpu(hp).oper[0]^);
            end;
        end
      else
        { process operands }
        begin
          s:=#9+gas_op2str[op];
          if taicpu(hp).ops<>0 then
            begin
            {
              if not is_calljmp(op) then
                sep:=','
              else
            }
                sep:=#9;
              for i:=0 to taicpu(hp).ops-1 do
                begin
                   // debug code
                   // writeln(s);
                   // writeln(taicpu(hp).fileinfo.line);
                   s:=s+sep+getopstr(owner.asminfo,taicpu(hp).oper[i]^);
                   sep:=',';
                end;
            end;
        end;
      owner.writer.AsmWriteLn(s);
    end;


{****************************************************************************}
{                         GNU PPC Assembler writer                           }
{****************************************************************************}

    constructor TPPCGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TPPCInstrWriter.create(self);
      end;


    function TPPCGNUAssembler.MakeCmdLine: TCmdStr;
      begin
        result := inherited MakeCmdLine;
{$ifdef cpu64bitaddr}
        Replace(result,'$ARCH','-a64')
{$else cpu64bitaddr}
        { MorphOS has an old 2.9.1 GNU AS, with a bunch of patches
          to support the system, and it doesn't know the -a32 argument }
        if target_info.system = system_powerpc_morphos then
          Replace(result,'$ARCH','')
        else
          Replace(result,'$ARCH','-a32');
{$endif cpu64bitaddr}
      end;


    procedure TPPCGNUAssembler.WriteExtraHeader;
      var
         i : longint;
      begin
        if target_info.abi = abi_powerpc_elfv2 then
          writer.AsmWriteln(#9'.abiversion 2');
        for i:=0 to 31 do
          writer.AsmWriteln(#9'.set'#9'r'+tostr(i)+','+tostr(i));
        for i:=0 to 31 do
          writer.AsmWriteln(#9'.set'#9'f'+tostr(i)+','+tostr(i));
      end;


{****************************************************************************}
{                      GNU/Apple PPC Assembler writer                        }
{****************************************************************************}

    constructor TPPCAppleGNUAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TPPCInstrWriter.create(self);
      end;


    function TPPCAppleGNUAssembler.MakeCmdLine: TCmdStr;
      begin
        result := inherited MakeCmdLine;
{$ifdef cpu64bitaddr}
        Replace(result,'$ARCH','ppc64')
{$else cpu64bitaddr}
        case current_settings.cputype of
          cpu_PPC7400:
            Replace(result,'$ARCH','ppc7400');
          cpu_PPC970:
            Replace(result,'$ARCH','ppc970');
          else
            Replace(result,'$ARCH','ppc')
        end;
{$endif cpu64bitaddr}
      end;


{****************************************************************************}
{                         AIX PPC Assembler writer                           }
{****************************************************************************}

    constructor TPPCAIXAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      var
         cur_sectype : TAsmSectionType;
      begin
        inherited;
        InstrWriter := TPPCInstrWriter.create(self);
        { Use 8-byte alignment as default for all sections }
        for cur_sectype:=low(TAsmSectionType) to high(TAsmSectionType) do
           max_alignment[cur_sectype]:=8;
      end;

    procedure TPPCAIXAssembler.WriteSection(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder;secalign:longint;
      secflags:TSectionFlags=[];secprogbits:TSectionProgbits=SPB_None);

      begin
        secalign:=max_alignment[atype];
        Inherited WriteSection(atype,aname,aorder,secalign,secflags,secprogbits);
      end;

    procedure TPPCAIXAssembler.WriteAsmList;
      var
        cur_sectype : TAsmSectionType;
        hal : tasmlisttype;
        hp : tai;
        max_al : longint;
      begin
        { Parse all asmlists to get maximum alignement used for all types }
        for hal:=low(TasmlistType) to high(TasmlistType) do
          begin
            if not (current_asmdata.asmlists[hal].empty) then
              begin
                cur_sectype:=sec_none;
                hp:=tai(current_asmdata.asmlists[hal].First);
                while assigned(hp) do
                  begin
                    case hp.typ of 
                     ait_align :
                       begin
                         if tai_align_abstract(hp).aligntype > max_alignment[cur_sectype] then
                           begin 
                             max_alignment[cur_sectype]:=tai_align_abstract(hp).aligntype;
                             current_asmdata.asmlists[hal].InsertAfter(tai_comment.Create(strpnew('Alignment put to '+tostr(tai_align_abstract(hp).aligntype))),hp);
                           end;
                       end;
                     ait_section :
                       begin
                         cur_sectype:=tai_section(hp).sectype;
                         if tai_section(hp).secalign > max_alignment[cur_sectype] then
                           begin
                             max_alignment[cur_sectype]:=tai_section(hp).secalign;
                             current_asmdata.asmlists[hal].InsertAfter(tai_comment.Create(strpnew('Section '
                               +sectionname(tai_section(hp).sectype,'',secorder_default)+' alignment put to '+tostr(tai_section(hp).secalign))),hp);
                           end;
                       end;
                     else
                       ;
                    end;
                    hp:=tai(hp.next);
                  end;
              end;
          end;
        { sec_data, sec_rodata and sec_bss all are converted into .data[RW],
          in WriteSection below,
          so we take the maximum alignment of the three }
        max_al:=max_alignment[sec_data];
        max_al:=max(max_al,max_alignment[sec_rodata]);
        max_al:=max(max_al,max_alignment[sec_bss]);
        max_alignment[sec_data]:=max_al;
        max_alignment[sec_rodata]:=max_al;
        max_alignment[sec_bss]:=max_al;
        Inherited WriteAsmList;
      end;

    procedure TPPCAIXAssembler.WriteExtraHeader;
      var
        i: longint;
      begin
        inherited WriteExtraHeader;
        { map cr registers to plain numbers }
        for i:=0 to 7 do
          writer.AsmWriteln(#9'.set'#9'cr'+tostr(i)+','+tostr(i));
        { Ensure .data and .rodata sections are aligned to 8-byte boundary,
          required for correct RTTI alignment.
          AIX assembler seems to only care for the first
          alignment value given }
        writer.AsmWriteln(#9'.csect .data[RW],'+sectionalignment_aix(sec_data,max_alignment[sec_data]));
        writer.AsmWriteln(#9'.csect _data.bss_[BS],'+sectionalignment_aix(sec_data,max_alignment[sec_data]));
        { .rodata is translated into .text[RO]
          see sectionname in aggas unit. }
        writer.AsmWriteln(#9'.csect .text[RO],'+sectionalignment_aix(sec_rodata_norel,max_alignment[sec_rodata_norel]));
        { make sure we always have a code and toc section,
          the linker expects that }
        writer.AsmWriteln(#9'.csect .text[PR],'+sectionalignment_aix(sec_code,max_alignment[sec_code]));
        { set _text_s, to be used by footer below } 
        writer.AsmWriteln(#9'_text_s:');
        writer.AsmWriteln(#9'.toc');
      end;


    procedure TPPCAIXAssembler.WriteExtraFooter;
      begin
        inherited WriteExtraFooter;
        { link between data and text section }
        writer.AsmWriteln(#9'.csect .data[RW],3');
{$ifdef cpu64bitaddr}
        writer.AsmWriteln('text_pos:'#9'.llong _text_s')
{$else cpu64bitaddr}
        writer.AsmWriteln('text_pos:'#9'.long _text_s')
{$endif cpu64bitaddr}
      end;


    procedure TPPCAIXAssembler.WriteDirectiveName(dir: TAsmDirective);
      begin
        case dir of
          asd_reference:
            writer.AsmWrite('.ref ');
          asd_weak_reference,
          asd_weak_definition:
            writer.AsmWrite('.weak ');
          else
            inherited WriteDirectiveName(dir);
        end;
      end;


    function TPPCAIXAssembler.sectionname(atype: TAsmSectiontype; const aname: string; aorder: TAsmSectionOrder): string;
      begin
        case atype of
          sec_code:
            result:='.csect .text[PR]';
          sec_data,
          sec_rodata,
          { don't use .bss[BS], causes relocation problems }
          sec_bss:
            result:='.csect .data[RW]';
          sec_rodata_norel:
            result:='.csect .text[RO]';
          sec_fpc:
            result:='.csect .fpc[RO]';
          sec_toc:
            result:='.toc';
          { automatically placed in the right section }
          sec_stab,
          sec_stabstr:
            result:='';
          else
            internalerror(2011122601);
        end;
      end;


{*****************************************************************************
                                  Initialize
*****************************************************************************}

  const
    as_ppc_gas_info : tasminfo =
       (
         id     : as_gas;

         idtxt  : 'AS';
         asmbin : 'as';
{$ifdef cpu64bitaddr}
         asmcmd : '-a64 $ENDIAN -o $OBJ $EXTRAOPT $ASM';
{$else cpu64bitaddr}
         asmcmd: '$ENDIAN -o $OBJ $EXTRAOPT $ARCH $ASM';
{$endif cpu64bitaddr}
         supported_targets : [system_powerpc_linux,system_powerpc_netbsd,system_powerpc_openbsd,system_powerpc_MorphOS,system_powerpc_Amiga,system_powerpc64_linux,system_powerpc_embedded,system_powerpc64_embedded];
         flags : [af_needar,af_smartlink_sections];
         labelprefix : '.L';
         comment : '# ';
         dollarsign: '$';
       );

    as_ppc_gas_legacy_info : tasminfo =
       (
         id     : as_powerpc_gas_legacy;

         idtxt  : 'AS-LEGACY';
         asmbin : 'as';
{$ifdef cpu64bitaddr}
         asmcmd : '-a64 $ENDIAN -o $OBJ $EXTRAOPT $ASM';
{$else cpu64bitaddr}
         asmcmd: '$ENDIAN -o $OBJ $EXTRAOPT $ARCH $ASM';
{$endif cpu64bitaddr}
         supported_targets : [system_powerpc_morphos];
         flags : [af_needar];
         labelprefix : '.L';
         comment : '# ';
         dollarsign: '$';
       );

    as_ppc_gas_darwin_powerpc_info : tasminfo =
       (
         id     : as_darwin;

         idtxt  : 'AS-DARWIN';
         asmbin : 'as';
         asmcmd : '-o $OBJ $EXTRAOPT $ASM -arch $ARCH';
         supported_targets : [system_powerpc_darwin,system_powerpc64_darwin];
         flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
         labelprefix : 'L';
         comment : '# ';
         dollarsign : '$';
       );


    as_ppc_aix_powerpc_info : tasminfo =
       (
         id     : as_powerpc_xcoff;

         idtxt  : 'AS-AIX';
         asmbin : 'as';
         { -u: allow using symbols before they are defined (when using native
               AIX assembler, ignore by GNU assembler)
           -mpwr5: we actually support Power3 and higher, but the AIX assembler
               has no parameter to select that one (only -mpwr3 and -mpwr5) }
{$ifdef cpu64bitaddr}
         asmcmd : '-a64 -u -o $OBJ $EXTRAOPT $ASM -mpwr5';
{$else cpu64bitaddr}
         asmcmd : '-u -o $OBJ $EXTRAOPT $ASM -mpwr5';
{$endif cpu64bitaddr}
         supported_targets : [system_powerpc_aix,system_powerpc64_aix];
         flags : [af_needar,af_smartlink_sections,af_stabs_use_function_absolute_addresses];
         labelprefix : 'L';
         comment : '# ';
         dollarsign : '.'
       );

    as_ppc_gas_aix_powerpc_info : tasminfo =
       (
         id     : as_gas_powerpc_xcoff;

         idtxt  : 'GAS';
         asmbin : 'gas';
         { -u: allow using symbols before they are defined (when using native
               AIX assembler, ignore by GNU assembler)
           -mpwr5: we actually support Power3 and higher, but the AIX assembler
               has no parameter to select that one (only -mpwr3 and -mpwr5) }
{$ifdef cpu64bitaddr}
         asmcmd : '-a64 -u -o $OBJ $EXTRAOPT $ASM -mpwr5';
{$else cpu64bitaddr}
         asmcmd : '-a32 -u -o $OBJ $EXTRAOPT $ASM -mpwr5';
{$endif cpu64bitaddr}
         supported_targets : [system_powerpc_aix,system_powerpc64_aix];
         flags : [af_needar,af_smartlink_sections,af_stabs_use_function_absolute_addresses];
         labelprefix : 'L';
         comment : '# ';
         dollarsign : '.'
       );

    as_ppc_clang_darwin_info : tasminfo =
       (
         id     : as_clang;
         idtxt  : 'CLANG';
         asmbin : 'clang';
         asmcmd : '-c -o $OBJ $EXTRAOPT -arch $ARCH $DARWINVERSION -x assembler $ASM';
         supported_targets : [system_powerpc_macos, system_powerpc_darwin, system_powerpc64_darwin];
         flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
         labelprefix : 'L';
         comment : '# ';
         dollarsign: '$';
       );

begin
  RegisterAssembler(as_ppc_gas_info,TPPCGNUAssembler);
  RegisterAssembler(as_ppc_gas_legacy_info,TPPCGNUAssembler);
  RegisterAssembler(as_ppc_gas_darwin_powerpc_info,TPPCAppleGNUAssembler);
  RegisterAssembler(as_ppc_clang_darwin_info,TPPCAppleGNUAssembler);
  RegisterAssembler(as_ppc_aix_powerpc_info,TPPCAIXAssembler);
  RegisterAssembler(as_ppc_gas_aix_powerpc_info,TPPCAIXAssembler);
end.
