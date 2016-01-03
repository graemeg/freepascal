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
<<<<<<< HEAD
<<<<<<< HEAD
       systems,aasmbase,
=======
       aasmbase,
>>>>>>> graemeg/fixes_2_2
=======
       aasmbase,
>>>>>>> origin/fixes_2_2
       aasmtai,aasmdata,
       aggas,
       cpubase,cgutils,
       globtype;

  type
    TPPCInstrWriter=class(TCPUInstrWriter)
       procedure WriteInstruction(hp : tai);override;
    end;

    TPPCGNUAssembler=class(TGNUassembler)
<<<<<<< HEAD
<<<<<<< HEAD
      constructor create(info: pasminfo; smart: boolean); override;
=======
      constructor create(smart: boolean); override;
>>>>>>> graemeg/fixes_2_2
=======
      constructor create(smart: boolean); override;
>>>>>>> origin/fixes_2_2
      procedure WriteExtraHeader; override;
    end;

    TPPCAppleGNUAssembler=class(TAppleGNUassembler)
<<<<<<< HEAD
<<<<<<< HEAD
      constructor create(info: pasminfo; smart: boolean); override;
      function MakeCmdLine: TCmdStr; override;
    end;

    TPPCAIXAssembler=class(TPPCGNUAssembler)
      constructor create(info: pasminfo; smart: boolean); override;
     protected
      function sectionname(atype: TAsmSectiontype; const aname: string; aorder: TAsmSectionOrder): string; override;
      procedure WriteExtraHeader; override;
      procedure WriteExtraFooter; override;
      procedure WriteDirectiveName(dir: TAsmDirective); override;
    end;

    topstr = string[4];

=======
=======
>>>>>>> origin/fixes_2_2
      constructor create(smart: boolean); override;
      function MakeCmdLine: TCmdStr; override;
    end;

    topstr = string[4];

    function getreferencestring(var ref : treference) : string;
    function getopstr_jmp(const o:toper) : string;
    function getopstr(const o:toper) : string;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function branchmode(o: tasmop): topstr;
    function cond2str(op: tasmop; c: tasmcond): string;  

  implementation

    uses
       cutils,globals,verbose,
<<<<<<< HEAD
<<<<<<< HEAD
       cgbase,
=======
       cgbase,systems,
>>>>>>> graemeg/fixes_2_2
=======
       cgbase,systems,
>>>>>>> origin/fixes_2_2
       assemble,
       itcpugas,cpuinfo,
       aasmcpu;

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef cpu64bit}
    const
      refaddr2str: array[trefaddr] of string[9] = ('', '', '', '@l', '@h', '@higher', '@highest', '@ha', '@highera', '@highesta');
      verbose_refaddrs = [addr_low, addr_high, addr_higher, addr_highest, addr_higha, addr_highera, addr_highesta];
      refaddr2str_darwin: array[trefaddr] of string[4] = ('','','','lo16', 'hi16', '@err', '@err', 'ha16', '@err', '@err');
{$else cpu64bit}
    const
      refaddr2str: array[trefaddr] of string[3] = ('','','','@l','@h','@ha');
      refaddr2str_darwin: array[trefaddr] of string[4] = ('','','','lo16','hi16','ha16');
      verbose_refaddrs = [addr_low,addr_high,addr_higha];
{$endif cpu64bit}


    function getreferencestring(var ref : treference) : string;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    var
      s : string;
    begin
       with ref do
        begin
          if ((offset < -32768) or (offset > 32767)) and
             (refaddr = addr_no) then
            internalerror(2006052501);
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
          if (refaddr = addr_no) then
            s := ''
          else
            begin
              if target_info.system in [system_powerpc_darwin,system_powerpc64_darwin] then
                s := refaddr2str_darwin[refaddr]
              else
                s :='';
              s := s+'(';
              if assigned(symbol) then
                begin
                  s:=s+symbol.name;
                  if assigned(relsymbol) then
                    s:=s+'-'+relsymbol.name;
                end;
            end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
           if (refaddr in verbose_refaddrs) then
             begin
               s := s+')';
               if not(target_info.system in [system_powerpc_darwin,system_powerpc64_darwin]) then
                 s := s+refaddr2str[refaddr];
             end;
{$ifdef cpu64bit}
           if (refaddr = addr_pic) then
	     if (target_info.system <> system_powerpc64_linux) then
	       s := s + ')'
	     else
	       s := s + ')@got';
{$endif cpu64bit}

           if (index=NR_NO) and (base<>NR_NO) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
             begin
                if offset=0 then
                  begin
                    if not (assigned(symbol)) then
                      s:=s+'0';
                  end;
<<<<<<< HEAD
<<<<<<< HEAD
                if (base<>NR_NO) then
                  s:=s+'('+gas_regname(base)+')'
                else if not assigned(symbol) and
                        not(refaddr in verbose_refaddrs) then
                  s:=s+'(0)';
=======
                s:=s+'('+gas_regname(base)+')';
>>>>>>> graemeg/fixes_2_2
=======
                s:=s+'('+gas_regname(base)+')';
>>>>>>> origin/fixes_2_2
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
    

<<<<<<< HEAD
<<<<<<< HEAD
    function getopstr_jmp(asminfo: pasminfo; const o:toper) : string;
=======
    function getopstr_jmp(const o:toper) : string;
>>>>>>> graemeg/fixes_2_2
=======
    function getopstr_jmp(const o:toper) : string;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
              internalerror(200402267);
            hs:=o.ref^.symbol.name;
            if asminfo^.dollarsign<>'$' then
              hs:=ReplaceForbiddenAsmSymbolChars(hs);
=======
              internalerror(200402262);
            hs:=o.ref^.symbol.name;
>>>>>>> graemeg/fixes_2_2
=======
              internalerror(200402262);
            hs:=o.ref^.symbol.name;
>>>>>>> origin/fixes_2_2
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


<<<<<<< HEAD
<<<<<<< HEAD
    function getopstr(asminfo: pasminfo; const o:toper) : string;
=======
    function getopstr(const o:toper) : string;
>>>>>>> graemeg/fixes_2_2
=======
    function getopstr(const o:toper) : string;
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
              if asminfo^.dollarsign<>'$' then
                hs:=ReplaceForbiddenAsmSymbolChars(hs);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
              if o.ref^.offset>0 then
               hs:=hs+'+'+tostr(o.ref^.offset)
              else
               if o.ref^.offset<0 then
                hs:=hs+tostr(o.ref^.offset);
              getopstr:=hs;
            end
          else
<<<<<<< HEAD
<<<<<<< HEAD
            getopstr:=getreferencestring(asminfo,o.ref^);
=======
            getopstr:=getreferencestring(o.ref^);
>>>>>>> graemeg/fixes_2_2
=======
            getopstr:=getreferencestring(o.ref^);
>>>>>>> origin/fixes_2_2
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
        end;
        case o of
          A_BL,A_BLA,A_BCL,A_BCLA,A_BCCTRL,A_BCLRL: tempstr := tempstr+'l';
        end;
        case o of
          A_BA,A_BLA,A_BCA,A_BCLA: tempstr:=tempstr+'a';
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
              else
                internalerror(2003112901);
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
                    else
                      internalerror(2003112901);
                  end;
                  case c.cond of
                    C_LT..C_NU:
                      cond2str := tempstr+gas_regname(newreg(R_SPECIALREGISTER,c.cr,R_SUBWHOLE));
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
<<<<<<< HEAD
<<<<<<< HEAD
              owner.writer.AsmWrite(s);
              s:=getopstr_jmp(owner.asminfo,taicpu(hp).oper[0]^);
=======
              owner.asmwrite(s);
              s:=getopstr_jmp(taicpu(hp).oper[0]^);
>>>>>>> graemeg/fixes_2_2
=======
              owner.asmwrite(s);
              s:=getopstr_jmp(taicpu(hp).oper[0]^);
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
                   s:=s+sep+getopstr(owner.asminfo,taicpu(hp).oper[i]^);
=======
                   s:=s+sep+getopstr(taicpu(hp).oper[i]^);
>>>>>>> graemeg/fixes_2_2
=======
                   s:=s+sep+getopstr(taicpu(hp).oper[i]^);
>>>>>>> origin/fixes_2_2
                   sep:=',';
                end;
            end;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
      owner.writer.AsmWriteLn(s);
=======
      owner.AsmWriteLn(s);
>>>>>>> graemeg/fixes_2_2
=======
      owner.AsmWriteLn(s);
>>>>>>> origin/fixes_2_2
    end;


{****************************************************************************}
{                         GNU PPC Assembler writer                           }
{****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
    constructor TPPCGNUAssembler.create(info: pasminfo; smart: boolean);
      begin
        inherited;
=======
    constructor TPPCGNUAssembler.create(smart: boolean);
      begin
        inherited create(smart);
>>>>>>> graemeg/fixes_2_2
=======
    constructor TPPCGNUAssembler.create(smart: boolean);
      begin
        inherited create(smart);
>>>>>>> origin/fixes_2_2
        InstrWriter := TPPCInstrWriter.create(self);
      end;


    procedure TPPCGNUAssembler.WriteExtraHeader;
      var
         i : longint;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
        if target_info.abi = abi_powerpc_elfv2 then
          writer.AsmWriteln(#9'.abiversion 2');
        for i:=0 to 31 do
          writer.AsmWriteln(#9'.set'#9'r'+tostr(i)+','+tostr(i));
        for i:=0 to 31 do
          writer.AsmWriteln(#9'.set'#9'f'+tostr(i)+','+tostr(i));
=======
=======
>>>>>>> origin/fixes_2_2
        for i:=0 to 31 do
          AsmWriteln(#9'.set'#9'r'+tostr(i)+','+tostr(i));
        for i:=0 to 31 do
          AsmWriteln(#9'.set'#9'f'+tostr(i)+','+tostr(i));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


{****************************************************************************}
{                      GNU/Apple PPC Assembler writer                        }
{****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
    constructor TPPCAppleGNUAssembler.create(info: pasminfo; smart: boolean);
      begin
        inherited;
=======
    constructor TPPCAppleGNUAssembler.create(smart: boolean);
      begin
        inherited create(smart);
>>>>>>> graemeg/fixes_2_2
=======
    constructor TPPCAppleGNUAssembler.create(smart: boolean);
      begin
        inherited create(smart);
>>>>>>> origin/fixes_2_2
        InstrWriter := TPPCInstrWriter.create(self);
      end;


    function TPPCAppleGNUAssembler.MakeCmdLine: TCmdStr;
      begin
        result := inherited MakeCmdLine;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef cpu64bitaddr}
        Replace(result,'$ARCH','ppc64')
{$else cpu64bitaddr}
=======
{$ifdef cpu64bit}
        Replace(result,'$ARCH','ppc64')
{$else cpu64bit}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef cpu64bit}
        Replace(result,'$ARCH','ppc64')
{$else cpu64bit}
>>>>>>> origin/fixes_2_2
        case current_settings.cputype of
          cpu_PPC7400:
            Replace(result,'$ARCH','ppc7400');
          cpu_PPC970:
            Replace(result,'$ARCH','ppc970');
          else
            Replace(result,'$ARCH','ppc')
        end;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif cpu64bitaddr}
      end;


{****************************************************************************}
{                         AIX PPC Assembler writer                           }
{****************************************************************************}

    constructor TPPCAIXAssembler.create(info: pasminfo; smart: boolean);
      begin
        inherited;
        InstrWriter := TPPCInstrWriter.create(self);
      end;


    procedure TPPCAIXAssembler.WriteExtraHeader;
      var
        i: longint;
      begin
        inherited WriteExtraHeader;
        { map cr registers to plain numbers }
        for i:=0 to 7 do
          writer.AsmWriteln(#9'.set'#9'cr'+tostr(i)+','+tostr(i));
        { make sure we always have a code and toc section, the linker expects
          that }
        writer.AsmWriteln(#9'.csect .text[PR]');
        { set _text_s, to be used by footer below } 
        writer.AsmWriteln(#9'_text_s:');
        writer.AsmWriteln(#9'.toc');
      end;


    procedure TPPCAIXAssembler.WriteExtraFooter;
      begin
        inherited WriteExtraFooter;
        { link between data and text section }
        writer.AsmWriteln(#9'.csect .data[RW],4');
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

=======
=======
>>>>>>> origin/fixes_2_2
{$endif cpu64bit}
      end;




<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{*****************************************************************************
                                  Initialize
*****************************************************************************}

  const
    as_ppc_gas_info : tasminfo =
       (
         id     : as_gas;

         idtxt  : 'AS';
         asmbin : 'as';
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef cpu64bitaddr}
         asmcmd : '-a64 -o $OBJ $EXTRAOPT $ASM';
{$else cpu64bitaddr}
         asmcmd: '-o $OBJ $EXTRAOPT $ASM';
{$endif cpu64bitaddr}
         supported_targets : [system_powerpc_linux,system_powerpc_netbsd,system_powerpc_openbsd,system_powerpc_MorphOS,system_powerpc_Amiga,system_powerpc64_linux,system_powerpc_embedded,system_powerpc64_embedded];
         flags : [af_needar,af_smartlink_sections];
         labelprefix : '.L';
         comment : '# ';
         dollarsign: '$';
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef cpu64bit}
         asmcmd : '-a64 -o $OBJ $ASM';
{$else cpu64bit}
         asmcmd: '-o $OBJ $ASM';
{$endif cpu64bit}
         supported_target : system_any;
         flags : [af_allowdirect,af_needar,af_smartlink_sections];
         labelprefix : '.L';
         comment : '# ';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
       );


    as_ppc_gas_darwin_powerpc_info : tasminfo =
       (
         id     : as_darwin;

<<<<<<< HEAD
<<<<<<< HEAD
         idtxt  : 'AS-DARWIN';
         asmbin : 'as';
         asmcmd : '-o $OBJ $EXTRAOPT $ASM -arch $ARCH';
         supported_targets : [system_powerpc_darwin,system_powerpc64_darwin];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
=======
         flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
>>>>>>> graemeg/cpstrnew
=======
         flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
>>>>>>> graemeg/cpstrnew
=======
         flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
>>>>>>> graemeg/cpstrnew
=======
         flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
>>>>>>> origin/cpstrnew
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

begin
  RegisterAssembler(as_ppc_gas_info,TPPCGNUAssembler);
  RegisterAssembler(as_ppc_gas_darwin_powerpc_info,TPPCAppleGNUAssembler);
  RegisterAssembler(as_ppc_aix_powerpc_info,TPPCAIXAssembler);
  RegisterAssembler(as_ppc_gas_aix_powerpc_info,TPPCAIXAssembler);
=======
=======
>>>>>>> origin/fixes_2_2
         idtxt  : 'AS-Darwin';
         asmbin : 'as';
         asmcmd : '-o $OBJ $ASM -arch $ARCH';
         supported_target : system_any;
         flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf];
         labelprefix : 'L';
         comment : '# ';
       );


begin
  RegisterAssembler(as_ppc_gas_info,TPPCGNUAssembler);
  RegisterAssembler(as_ppc_gas_darwin_powerpc_info,TPPCAppleGNUAssembler);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
