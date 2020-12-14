{
    Copyright (c) 2003,2014 by Florian Klaempfl and Jonas Maebe

    This unit implements an asm for AArch64

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
{ This unit implements the GNU Assembler writer for AArch64
}

unit agcpugas;

{$i fpcdefs.inc}

  interface

    uses
       globtype,systems,
       aasmtai,aasmdata,aasmbase,
       assemble,aggas,
       cpubase,cpuinfo;

    type
      TAArch64InstrWriter=class(TCPUInstrWriter)
        procedure WriteInstruction(hp : tai);override;
      end;

      TAArch64Assembler=class(TGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      end;

      TAArch64AppleAssembler=class(TAppleGNUassembler)
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      end;

      TAArch64ClangGASAssembler=class(TAArch64Assembler)
      private
        procedure TransformSEHDirectives(list:TAsmList);
      protected
        function sectionflags(secflags:TSectionFlags):string;override;
      public
        procedure WriteAsmList; override;
      end;

    const
      gas_shiftmode2str : array[tshiftmode] of string[4] = (
        '','lsl','lsr','asr','ror',
        'uxtb','uxth','uxtw','uxtx',
        'sxtb','sxth','sxtw','sxtx');

    const
      cputype_to_gas_march : array[tcputype] of string = (
        '', // cpu_none
        'armv8'
      );

  implementation

    uses
       cutils,cclasses,globals,verbose,
       aasmcpu,
       itcpugas,
       cgbase,cgutils;


{****************************************************************************}
{                      AArch64 Assembler writer                              }
{****************************************************************************}

    constructor TAArch64Assembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TAArch64InstrWriter.create(self);
      end;

{****************************************************************************}
{                      Apple AArch64 Assembler writer                        }
{****************************************************************************}

    constructor TAArch64AppleAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter := TAArch64InstrWriter.create(self);
      end;


{****************************************************************************}
{                      CLang AArch64 Assembler writer                        }
{****************************************************************************}

    procedure TAArch64ClangGASAssembler.TransformSEHDirectives(list:TAsmList);

      function convert_unwinddata(list:tasmlist):tdynamicarray;

        procedure check_offset(ofs,max:dword);
          begin
            if ((ofs and $7)<>0) or (ofs>max) then
              internalerror(2020041210);
          end;

        procedure check_reg(reg:tregister;rt:TRegisterType;min:TSuperRegister);
          begin
            if (getregtype(reg)<>rt) or (getsupreg(reg)<min) then
              internalerror(2020041211);
          end;

        procedure writebyte(b:byte); inline;
          begin
            result.write(b,sizeof(b));
          end;

        procedure writeword(w:word);
          begin
            w:=NtoBE(w);
            result.write(w,sizeof(w));
          end;

        procedure writedword(dw:dword);
          begin
            dw:=NtoBE(dw);
            result.write(dw,sizeof(dw));
          end;

        const
          min_int_reg = 19;
          min_mm_reg = 8;
        var
          hp : tai;
          seh : tai_seh_directive absolute hp;
        begin
          result:=tdynamicarray.create(0);
          hp:=tai(list.last);
          while assigned(hp) do
            begin
              if hp.typ<>ait_seh_directive then
                internalerror(2020041502);
              case seh.kind of
                ash_stackalloc:
                  begin
                    if (seh.data.offset and $f)<>0 then
                      internalerror(2020041207);
                    if seh.data.offset<((1 shl 5)*16) then
                      writebyte(byte(seh.data.offset shr 4))
                    else if seh.data.offset<((1 shl 11)*16) then
                      writeword($C000 or word(seh.data.offset shr 4))
                    else if seh.data.offset<((1 shl 24)*16) then
                      writedword($E0000000 or (seh.data.offset shr 4))
                    else begin
                      writeln(hexstr(seh.data.offset,8));
                      internalerror(2020041209);
                    end;
                  end;
                ash_addfp:
                  begin
                    check_offset(seh.data.offset,(1 shl 7)*8);
                    writeword($E200 or (seh.data.offset shr 3));
                  end;
                ash_setfp:
                  writebyte($E1);
                ash_nop:
                  writebyte($E3);
                ash_savefplr:
                  begin
                    check_offset(seh.data.offset,504);
                    writebyte($40 or (seh.data.offset shr 3));
                  end;
                ash_savefplr_x:
                  begin
                    check_offset(seh.data.offset,512);
                    writebyte($80 or (seh.data.offset shr 3)-1);
                  end;
                ash_savereg:
                  begin
                    check_offset(seh.data.offset,504);
                    check_reg(seh.data.reg,R_INTREGISTER,min_int_reg);
                    writeword($C000 or ((getsupreg(seh.data.reg)-min_int_reg) shl 6) or (seh.data.offset shr 3));
                  end;
                ash_savereg_x:
                  begin
                    check_offset(seh.data.offset,256);
                    check_reg(seh.data.reg,R_INTREGISTER,min_int_reg);
                    writeword($C400 or ((getsupreg(seh.data.reg)-min_int_reg) shl 5) or ((seh.data.offset shr 3)-1));
                  end;
                ash_saveregp:
                  begin
                    check_offset(seh.data.offset,504);
                    check_reg(seh.data.reg,R_INTREGISTER,min_int_reg);
                    writeword($C800 or ((getsupreg(seh.data.reg)-min_int_reg) shl 6) or (seh.data.offset shr 3));
                  end;
                ash_saveregp_x:
                  begin
                    check_offset(seh.data.offset,512);
                    check_reg(seh.data.reg,R_INTREGISTER,min_int_reg);
                    writeword($CC00 or ((getsupreg(seh.data.reg)-min_int_reg) shl 6) or ((seh.data.offset shr 3)-1));
                  end;
                ash_savefreg:
                  begin
                    check_offset(seh.data.offset,504);
                    check_reg(seh.data.reg,R_MMREGISTER,min_mm_reg);
                    writeword($DC00 or ((getsupreg(seh.data.reg)-min_mm_reg) shl 6) or (seh.data.offset shr 3));
                  end;
                ash_savefreg_x:
                  begin
                    check_offset(seh.data.offset,256);
                    check_reg(seh.data.reg,R_MMREGISTER,min_mm_reg);
                    writeword($CE00 or ((getsupreg(seh.data.reg)-min_mm_reg) shl 5) or ((seh.data.offset shr 3)-1));
                  end;
                ash_savefregp:
                  begin
                    check_offset(seh.data.offset,504);
                    check_reg(seh.data.reg,R_MMREGISTER,min_mm_reg);
                    writeword($D800 or ((getsupreg(seh.data.reg)-min_mm_reg) shl 6) or (seh.data.offset shr 3));
                  end;
                ash_savefregp_x:
                  begin
                    check_offset(seh.data.offset,512);
                    check_reg(seh.data.reg,R_MMREGISTER,min_mm_reg);
                    writeword($DA00 or ((getsupreg(seh.data.reg)-min_mm_reg) shl 6) or ((seh.data.offset shr 3)-1));
                  end;
                else
                  internalerror(2020041503);
              end;
              hp:=tai(hp.previous);
            end;
        end;

      var
        unwinddata : tdynamicarray;

      procedure writebyte(b:byte);
        begin
          unwinddata.write(b,sizeof(b));
        end;

      var
        hp,hpnext,hpdata : tai;
        seh : tai_seh_directive absolute hp;
        lastsym : tai_symbol;
        lastsec : tai_section;
        inprologue,
        inhandlerdata,
        deleteai : boolean;
        totalcount,
        instrcount,
        datacount : sizeint;
        handlername : tsymstr;
        handlerflags : byte;
        handlerdata : array of tai;
        handlerdataidx : sizeint;
        handlerdatacount : tai;
        sehlist,
        tmplist : TAsmList;
        xdatasym : tasmsymbol;
        unwindread,
        unwindrec : longword;
      begin
        if not assigned(list) then
          exit;

        lastsym:=nil;
        tmplist:=nil;
        sehlist:=nil;
        lastsec:=nil;
        instrcount:=0;
        datacount:=0;
        unwinddata:=nil;
        inhandlerdata:=false;
        inprologue:=false;
        handlerdata:=nil;
        handlerdataidx:=0;
        handlerdatacount:=nil;
        handlerflags:=0;
        handlername:='';

        hp:=tai(list.first);
        while assigned(hp) do
          begin
            deleteai:=false;
            case hp.typ of
              ait_section:
                begin
                  if assigned(sehlist) then
                    begin
                      if assigned(lastsec) and (tai_section(hp).name^=lastsec.name^) then
                        begin
                          { this section was only added due to the now removed SEH data }
                          deleteai:=true;
                          dec(list.section_count);
                        end
                      else
                        internalerror(2020041214);
                    end
                  else
                    begin
                      lastsec:=tai_section(hp);
                      { also reset the last encountered symbol }
                      lastsym:=nil;
                    end;

                  if assigned(tmplist) then
                    begin
                      list.insertListBefore(hp,tmplist);
                      tmplist.free;
                      tmplist:=nil;
                    end;

                end;
              ait_symbol:
                begin
                  if tai_symbol(hp).sym.typ=AT_FUNCTION then
                    lastsym:=tai_symbol(hp);
                end;
              ait_instruction:
                if assigned(sehlist) then
                  inc(instrcount);
              ait_const:
                if assigned(sehlist) then
                  inc(datacount,tai_const(hp).size);
              ait_seh_directive:
                begin
                  if not assigned(sehlist) and (seh.kind<>ash_proc) then
                    internalerror(2020041208);
                  { most seh directives are removed }
                  deleteai:=true;
                  case seh.kind of
                    ash_proc:
                      begin
                        if not assigned(lastsec) then
                          internalerror(2020041203);
                        datacount:=0;
                        instrcount:=0;
                        handlerflags:=0;
                        handlername:='';
                        sehlist:=tasmlist.create;
                        inprologue:=true;
                      end;
                    ash_endproc:
                      begin
                        if not assigned(sehlist) then
                          internalerror(2020041501);
                        if assigned(tmplist) then
                          internalerror(2020041302);
                        if not assigned(lastsym) then
                          internalerror(2020041303);
                        if inprologue then
                          cgmessage(asmw_e_missing_endprologue);

                        unwinddata:=convert_unwinddata(sehlist);

                        writebyte($E4);

                        { fill up with NOPs }
                        while unwinddata.size mod 4<>0 do
                          writebyte($E3);

                        { note: we can pass Nil here, because in case of a LLVM
                                backend this whole code shouldn't be required
                                anyway }
                        xdatasym:=current_asmdata.DefineAsmSymbol('xdata_'+lastsym.sym.name,AB_LOCAL,AT_DATA,nil);

                        tmplist:=tasmlist.create;
                        new_section(tmplist,sec_pdata,lastsec.name^,0);
                        tmplist.concat(tai_const.Create_rva_sym(lastsym.sym));
                        tmplist.concat(tai_const.Create_rva_sym(xdatasym));

                        new_section(tmplist,sec_rodata,xdatasym.name,0);
                        tmplist.concat(tai_symbol.Create(xdatasym,0));

                        tmplist.concat(tai_comment.Create(strpnew('instr: '+tostr(instrcount)+', data: '+tostr(datacount)+', unwind: '+tostr(unwinddata.size))));

                        {$ifdef EXTDEBUG}
                        comment(V_Debug,'got section: '+lastsec.name^);
                        comment(V_Debug,'got instructions: '+tostr(instrcount));
                        comment(V_Debug,'got data: '+tostr(datacount));
                        comment(V_Debug,'got unwinddata: '+tostr(unwinddata.size));
                        {$endif EXTDEBUG}

                        if datacount mod 4<>0 then
                          cgmessage(asmw_e_seh_invalid_data_size);

                        totalcount:=datacount div 4+instrcount;

                        { splitting to multiple pdata/xdata sections is not yet
                          supported, so 1 MB is our limit for now }
                        if totalcount>(1 shl 18) then
                          comment(V_Error,'Function is larger than 1 MB which is not supported for SEH currently');

                        unwindrec:=min(totalcount,(1 shl 18)-1);
                        if handlerflags<>0 then
                          unwindrec:=unwindrec or (1 shl 20);

                        { currently we only have one epilog, so E needs to be
                          set to 1 and epilog scope index needs to be 0, no
                          matter if we require the extension for the unwinddata
                          or not }
                        unwindrec:=unwindrec or (1 shl 21);

                        if unwinddata.size div 4<=31 then
                          unwindrec:=unwindrec or ((unwinddata.size div 4) shl 27);

                        { exception record headers }
                        tmplist.concat(tai_const.Create_32bit(longint(unwindrec)));
                        if cs_asm_source in init_settings.globalswitches then
                          tmplist.concat(tai_comment.create(strpnew(hexstr(unwindrec,8))));

                        if unwinddata.size div 4>31 then
                          begin
                            { once we're able to split a .pdata entry this can be
                              removed as well }
                            if unwinddata.size div 4>255 then
                              comment(V_Error,'Too many unwind codes for SEH');
                            unwindrec:=(unwinddata.size div 4) shl 16;
                            tmplist.concat(tai_const.create_32bit(longint(unwindrec)));
                            if cs_asm_source in init_settings.globalswitches then
                              tmplist.concat(tai_comment.create(strpnew(hexstr(unwindrec,8))));
                          end;

                        { unwind codes }
                        unwinddata.seek(0);
                        while unwinddata.pos<unwinddata.size do
                          begin
                            unwinddata.read(unwindrec,sizeof(longword));
                            tmplist.concat(tai_const.Create_32bit(longint(unwindrec)));
                            if cs_asm_source in init_settings.globalswitches then
                              tmplist.concat(tai_comment.create(strpnew(hexstr(unwindrec,8))));
                          end;
                        unwinddata.free;

                        if handlerflags<>0 then
                          begin
                            tmplist.concat(tai_const.Create_rva_sym(current_asmdata.RefAsmSymbol(handlername,AT_FUNCTION,false)));
                            if length(handlerdata)>0 then
                              begin
                                tmplist.concat(handlerdatacount);
                                for handlerdataidx:=0 to high(handlerdata) do
                                  tmplist.concat(handlerdata[handlerdataidx]);
                              end;
                          end;

                        handlerdata:=nil;

                        sehlist.free;
                        sehlist:=nil;
                      end;
                    ash_endprologue:
                      inprologue:=false;
                    ash_handler:
                      begin
                        handlername:=seh.data.name^;
                        handlerflags:=seh.data.flags;
                      end;
                    ash_handlerdata:
                      begin
                        if handlername='' then
                          cgmessage(asmw_e_handlerdata_no_handler);
                        hpdata:=tai(hp.next);
                        if not assigned(hpdata) or (hpdata.typ<>ait_const) or (tai_const(hpdata).consttype<>aitconst_32bit) then
                          internalerror(2020041215);
                        handlerdatacount:=hpdata;
                        setlength(handlerdata,tai_const(hpdata).value*4);
                        handlerdataidx:=0;
                        hpnext:=tai(hpdata.next);
                        list.remove(hpdata);
                        hpdata:=hpnext;
                        while (handlerdataidx<length(handlerdata)) and assigned(hpdata) do
                          begin
                            if (hpdata.typ<>ait_const) or not (tai_const(hpdata).consttype in [aitconst_32bit,aitconst_rva_symbol]) then
                              internalerror(2020041212);
                            handlerdata[handlerdataidx]:=hpdata;
                            inc(handlerdataidx);
                            hpnext:=tai(hpdata.next);
                            list.remove(hpdata);
                            hpdata:=hpnext;
                          end;
                        if handlerdataidx<length(handlerdata) then
                          internalerror(2020041213);
                      end;
                    ash_stackalloc,
                    ash_addfp,
                    ash_setfp,
                    ash_nop,
                    ash_savefplr,
                    ash_savefplr_x,
                    ash_savereg,
                    ash_savereg_x,
                    ash_saveregp,
                    ash_saveregp_x,
                    ash_savefreg,
                    ash_savefreg_x,
                    ash_savefregp,
                    ash_savefregp_x:
                      begin
                        if not assigned(sehlist) then
                          internalerror(2020041504);
                        if not inprologue then
                          internalerror(2020041505);
                        hpdata:=hp;
                        hp:=tai(hp.previous);
                        list.Remove(hpdata);
                        sehlist.concat(hpdata);
                        { don't delete this }
                        deleteai:=false;
                      end;
                    else
                      internalerror(2020041206);
                  end;
                end;
              else
                { ignore }
                ;
            end;

            if deleteai then
              begin
                hpnext:=tai(hp.next);
                list.remove(hp);
                hp.free;
                hp:=hpnext;
              end
            else
              hp:=tai(hp.next);
          end;

        if assigned(sehlist) then
          internalerror(2020041205);

        if assigned(tmplist) then
          begin
            list.concatlist(tmplist);
            tmplist.free;
          end;
      end;


    function TAArch64ClangGASAssembler.sectionflags(secflags:TSectionFlags):string;
      begin
        Result:=inherited sectionflags(secflags);
        if (target_info.system=system_aarch64_win64) then
          begin
            { we require an explicit "r" if write is not allowed }
            if not (SF_W in secflags) then
              result:=result+'r';
          end;
      end;


    procedure TAArch64ClangGASAssembler.WriteAsmList;
      begin
        { clang does not support all the directives we need, so we need to
          manually transform them to pdata/xdata records }
        if target_info.system=system_aarch64_win64 then
          begin
            TransformSEHDirectives(current_asmdata.AsmLists[al_pure_assembler]);
            TransformSEHDirectives(current_asmdata.AsmLists[al_procedures]);
          end;
        inherited WriteAsmList;
      end;


{****************************************************************************}
{                  Helper routines for Instruction Writer                    }
{****************************************************************************}

    function getreferencestring(asminfo: pasminfo; var ref : treference) : string;
      const
        darwin_addrpage2str: array[addr_page..addr_gotpageoffset] of string[11] =
           ('@PAGE','@PAGEOFF','@GOTPAGE','@GOTPAGEOFF');
        linux_addrpage2str: array[addr_page..addr_gotpageoffset] of string[10] =
           ('',':lo12:',':got:',':got_lo12:');
      begin
        if ref.base=NR_NO then
          begin
            case ref.refaddr of
              addr_gotpage,
              addr_page,
              addr_gotpageoffset,
              addr_pageoffset:
                begin
                  if not assigned(ref.symbol) or
                     (ref.base<>NR_NO) or
                     (ref.index<>NR_NO) or
                     (ref.shiftmode<>SM_None) or
                     (ref.offset<>0) then
                    internalerror(2014121501);
                  if target_info.system in systems_darwin then
                    result:=ref.symbol.name+darwin_addrpage2str[ref.refaddr]
                  else
                    result:=linux_addrpage2str[ref.refaddr]+ref.symbol.name
                end;
              addr_pic,
              { for locals replaced by temp symbols on LLVM }
              addr_no:
                result:=ref.symbol.name;
              else
                internalerror(2015022302);
            end
          end
        else
          begin
            result:='['+gas_regname(ref.base);
            if ref.addressmode=AM_POSTINDEXED then
              result:=result+']';
            if ref.index<>NR_NO then
              begin
                if (ref.offset<>0) or
                   assigned(ref.symbol) then
                  internalerror(2014121504);
                result:=result+', '+gas_regname(ref.index);
                case ref.shiftmode of
                  SM_None: ;
                  SM_LSL,
                  SM_UXTW, SM_UXTX, SM_SXTW, SM_SXTX:
                    begin
                      result:=result+', '+gas_shiftmode2str[ref.shiftmode];
                      if (ref.shiftmode=SM_LSL) or
                         (ref.shiftimm<>0) then
                        result:=result+' #'+tostr(ref.shiftimm);
                    end
                  else
                    internalerror(2014121505);
                end;
              end
            else
              begin
                if assigned(ref.symbol) then
                  begin
                    case ref.refaddr of
                      addr_gotpageoffset,
                      addr_pageoffset:
                        begin
                          if target_info.system in systems_darwin then
                            result:=result+', '+ref.symbol.name+darwin_addrpage2str[ref.refaddr]
                          else
                            result:=result+', '+linux_addrpage2str[ref.refaddr]+ref.symbol.name
                        end
                      else
                        { todo: not yet generated/don't know syntax }
                        internalerror(2014121506);
                    end;
                  end
                else
                  begin
                    if ref.refaddr<>addr_no then
                      internalerror(2014121502);
                    if (ref.offset<>0) then
                      result:=result+', #'+tostr(ref.offset);
                  end;
              end;
            case ref.addressmode of
              AM_OFFSET:
                result:=result+']';
              AM_PREINDEXED:
                result:=result+']!';
              else
                ;
            end;
          end;
      end;


    function getopstr(asminfo: pasminfo; hp: taicpu; opnr: longint; const o: toper): string;
      var
        i: longint;
        reg: tregister;
      begin
        case o.typ of
          top_reg:
            getopstr:=gas_regname(o.reg);
          top_shifterop:
            begin
              getopstr:=gas_shiftmode2str[o.shifterop^.shiftmode];
              if o.shifterop^.shiftimm<>0 then
                getopstr:=getopstr+' #'+tostr(o.shifterop^.shiftimm)
            end;
          top_const:
            if o.val>=0 then
              getopstr:='#'+tostr(o.val)
            else
              getopstr:='#0x'+hexStr(o.val,16);
          top_conditioncode:
            getopstr:=cond2str[o.cc];
          top_ref:
            if is_calljmp(hp.opcode) then
              begin
                if o.ref^.refaddr<>addr_full then
                  internalerror(2014122220);
                if not assigned(o.ref^.symbol) or
                   assigned(o.ref^.relsymbol) or
                   (o.ref^.base<>NR_NO) or
                   (o.ref^.index<>NR_NO) or
                   (o.ref^.offset<>0) then
                  internalerror(2014122221);
                getopstr:=o.ref^.symbol.name;
              end
            else
              getopstr:=getreferencestring(asminfo,o.ref^);
          top_realconst:
            begin
              str(o.val_real,Result);
              Result:='#'+Result;
            end;
          top_regset:
            begin
              reg:=o.basereg;
              result:='{'+gas_regname(reg);
              for i:=1 to o.nregs-1 do
                begin
                  setsupreg(reg,succ(getsupreg(reg)) mod 32);
                  result:=result+', '+gas_regname(reg);
                end;
              result:=result+'}';
              if o.regsetindex<>255 then
                result:=result+'['+tostr(o.regsetindex)+']'
            end;
          top_indexedreg:
            begin
              result:=gas_regname(o.indexedreg)+'['+tostr(o.regindex)+']';
            end;
          else
            internalerror(2014121507);
        end;
      end;


    procedure TAArch64InstrWriter.WriteInstruction(hp : tai);
      var
        op: TAsmOp;
        s: string;
        i: byte;
        sep: string[3];
      begin
        op:=taicpu(hp).opcode;
        s:=#9+gas_op2str[op]+oppostfix2str[taicpu(hp).oppostfix];
        if taicpu(hp).condition<>C_NONE then
          s:=s+'.'+cond2str[taicpu(hp).condition];
        if taicpu(hp).ops<>0 then
          begin
            sep:=#9;
            for i:=0 to taicpu(hp).ops-1 do
              begin
                 // debug code
                 // writeln(s);
                 // writeln(taicpu(hp).fileinfo.line);
                 s:=s+sep+getopstr(owner.asminfo,taicpu(hp),i,taicpu(hp).oper[i]^);
                 sep:=',';
              end;
          end;
        owner.writer.AsmWriteLn(s);
      end;


    const
       as_aarch64_gas_info : tasminfo =
          (
            id     : as_gas;
            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_aarch64_linux,system_aarch64_android];
            flags : [af_needar,af_smartlink_sections];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '// ';
            dollarsign: '$';
          );

       as_aarch64_clang_darwin_info : tasminfo =
          (
            id     : as_clang_asdarwin;
            idtxt  : 'CLANG';
            asmbin : 'clang';
            asmcmd : '-x assembler -c -target $TRIPLET -o $OBJ $EXTRAOPT -x assembler $ASM';
            supported_targets : [system_aarch64_ios,system_aarch64_darwin];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_llvm];
            labelprefix : 'L';
            labelmaxlen : -1;
            comment : '# ';
            dollarsign: '$';
          );

       as_aarch64_clang_gas_info : tasminfo =
          (
            id     : as_clang_gas;
            idtxt  : 'CLANG';
            asmbin : 'clang';
            asmcmd : '-x assembler -c -target $TRIPLET -o $OBJ $EXTRAOPT -x assembler $ASM';
            supported_targets : [system_aarch64_win64];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_llvm];
            labelprefix : '.L';
            labelmaxlen : -1;
            comment : '// ';
            dollarsign: '$';
          );


begin
  RegisterAssembler(as_aarch64_gas_info,TAArch64Assembler);
  RegisterAssembler(as_aarch64_clang_darwin_info,TAArch64AppleAssembler);
  RegisterAssembler(as_aarch64_clang_gas_info,TAArch64ClangGASAssembler);
end.
