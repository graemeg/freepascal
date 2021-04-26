{
    Copyright (c) 1998-2020 by the Free Pascal team

    This unit implements the llvm-mc ("llvm machine code playground")
    assembler writer for WebAssembly

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

unit agllvmmc;

{$i fpcdefs.inc}

interface

  uses
    systems,cgutils,
    globtype,globals,
    symbase,symdef,symtype,symconst,symcpu,
    aasmbase,aasmtai,aasmdata,aasmcpu,
    assemble,aggas;

  type

    { TLLVMMachineCodePlaygroundAssembler }

    TLLVMMachineCodePlaygroundAssembler=class(TGNUassembler)
    protected
      procedure WriteImports;

      function sectionname(atype:TAsmSectiontype;const aname:string;aorder:TAsmSectionOrder):string;override;
    public
      constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
      procedure WriteAsmList;override;
    end;

    { TWASM32InstrWriter }

    TWASM32InstrWriter = class(TCPUInstrWriter)
      procedure WriteInstruction(hp : tai);override;
    end;

implementation

  uses
    cutils,
    fmodule,finput,
    itcpugas,
    cpubase,
    hlcgobj,hlcgcpu,
    verbose;

  { TLLVMMachineCodePlaygroundAssembler }


  procedure TLLVMMachineCodePlaygroundAssembler.WriteImports;
    var
      i    : integer;
      def  : tdef;
      proc : tprocdef;
      list : TAsmList;
      cur_unit: tused_unit;
    begin
      for i:=0 to current_module.deflist.Count-1 do
        begin
          def:=tdef(current_module.deflist[i]);
          { since commit 48986 deflist might have NIL entries }
          if assigned(def) and (def.typ=procdef) then
            begin
              proc := tprocdef(def);
              if (po_external in proc.procoptions) and assigned(proc.import_dll) then
                begin
                  //WriteProcDef(proc);
                  list:=TAsmList.Create;
                  thlcgwasm(hlcg).g_procdef(list,proc);
                  WriteTree(list);
                  list.free;
                  writer.AsmWrite(#9'.import_module'#9);
                  writer.AsmWrite(proc.mangledname);
                  writer.AsmWrite(', ');
                  writer.AsmWriteLn(proc.import_dll^);
                  writer.AsmWrite(#9'.import_name'#9);
                  writer.AsmWrite(proc.mangledname);
                  writer.AsmWrite(', ');
                  writer.AsmWriteLn(proc.import_name^);
                end;
            end;
         end;
      list:=TAsmList.Create;
      cur_unit:=tused_unit(usedunits.First);
      while assigned(cur_unit) do
        begin
          if (cur_unit.u.moduleflags * [mf_init,mf_finalize])<>[] then
            begin
              if mf_init in cur_unit.u.moduleflags then
                list.Concat(tai_functype.create(make_mangledname('INIT$',cur_unit.u.globalsymtable,''),TWasmFuncType.Create([],[])));
              if mf_finalize in cur_unit.u.moduleflags then
                list.Concat(tai_functype.create(make_mangledname('FINALIZE$',cur_unit.u.globalsymtable,''),TWasmFuncType.Create([],[])));
            end;
          for i:=0 to cur_unit.u.deflist.Count-1 do
            begin
              def:=tdef(cur_unit.u.deflist[i]);
              if assigned(def) and (tdef(def).typ = procdef) then
                begin
                  proc := tprocdef(def);
                  if (not proc.owner.iscurrentunit or (po_external in proc.procoptions)) and
                     ((proc.paras.Count=0) or (proc.has_paraloc_info in [callerside,callbothsides])) then
                    thlcgwasm(hlcg).g_procdef(list,proc);
                end;
            end;
          cur_unit:=tused_unit(cur_unit.Next);
        end;
      WriteTree(list);
      list.free;
    end;


  function TLLVMMachineCodePlaygroundAssembler.sectionname(atype: TAsmSectiontype; const aname: string; aorder: TAsmSectionOrder): string;
    begin
      if (atype=sec_fpc) or (atype=sec_threadvar) then
        atype:=sec_data;
      Result:=inherited sectionname(atype, aname, aorder)+',"",@';
    end;


  constructor TLLVMMachineCodePlaygroundAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
    begin
      inherited;
      InstrWriter:=TWASM32InstrWriter.create(self);
    end;


  procedure TLLVMMachineCodePlaygroundAssembler.WriteAsmList;
    begin
      inherited;
      { print all global procedures/functions }
      writer.AsmWriteLn(#9'.globaltype'#9+STACK_POINTER_SYM+', i32');
      WriteImports;
    end;


  { TWASM32InstrWriter }


  procedure TWASM32InstrWriter.WriteInstruction(hp: tai);

    function getreferencestring(var ref : treference) : ansistring;
      begin
        if assigned(ref.symbol) then
          begin
            // global symbol or field -> full type and name
            // ref.base can be <> NR_NO in case an instance field is loaded.
            // This register is not part of this instruction, it will have
            // been placed on the stack by the previous one.
            result:=ref.symbol.name;
            if ref.base<>NR_NO then
              result:=result+'+'+std_regname(ref.base);
            if ref.index<>NR_NO then
              result:=result+'+'+std_regname(ref.index);
            if ref.offset>0 then
              result:=result+'+'+tostr(ref.offset)
            else if ref.offset<0 then
              result:=result+tostr(ref.offset);
          end
        else
          begin
            // local symbol -> stack slot, stored in offset
            result:='';
            if (ref.base<>NR_STACK_POINTER_REG) and (ref.base<>NR_NO) then
              result:=std_regname(ref.base);
            if ref.index<>NR_NO then
              if result<>'' then
                result:=result+'+'+std_regname(ref.index)
              else
                result:=std_regname(ref.index);
            if ref.offset>0 then
              begin
                if result<>'' then
                  result:=result+'+'+tostr(ref.offset)
                else
                  result:=tostr(ref.offset);
              end
            else if ref.offset<0 then
              result:=result+tostr(ref.offset);
            if result='' then
              result:='0';
          end;
      end;

    function constfloat(rawfloat: int64; fraction_bits, exponent_bits, exponent_bias: Integer): ansistring;
      var
        sign: boolean;
        fraction: int64;
        exponent, fraction_hexdigits: integer;
      begin
        fraction_hexdigits := (fraction_bits + 3) div 4;
        sign:=(rawfloat shr (fraction_bits+exponent_bits))<>0;
        fraction:=rawfloat and ((int64(1) shl fraction_bits)-1);
        exponent:=(rawfloat shr fraction_bits) and ((1 shl exponent_bits)-1);
        if sign then
          result:='-'
        else
          result:='';
        if (exponent=(1 shl exponent_bits)-1) then
          begin
            if fraction=0 then
              result:=result+'infinity'
            else
              begin
                result:=result+'nan';
                if fraction<>(int64(1) shl (fraction_bits-1)) then
                  result:=result+':0x'+HexStr(fraction,fraction_hexdigits);
              end;
          end
        else
          result:=result+'0x1.'+HexStr(fraction,fraction_hexdigits)+'p'+tostr(exponent-exponent_bias);
      end;

    function constsingle(s: single): ansistring;
      type
        tsingleval = record
          case byte of
            1: (s: single);
            2: (i: longword);
        end;
      begin
        result:=constfloat(tsingleval(s).i,23,8,127);
      end;

    function constdouble(d: double): ansistring;
      type
        tdoubleval = record
          case byte of
            1: (d: double);
            2: (i: int64);
        end;
      begin
        result:=constfloat(tdoubleval(d).i,52,11,1023);
      end;

    function getopstr(const o:toper) : ansistring;
      var
        d: double;
        s: single;
      begin
        case o.typ of
          top_reg:
            // should have been translated into a memory location by the
            // register allocator)
            if (cs_no_regalloc in current_settings.globalswitches) then
              getopstr:=std_regname(o.reg)
            else
              internalerror(2010122803);
          top_const:
            str(o.val,result);
          top_ref:
            getopstr:=getreferencestring(o.ref^);
          top_single:
            begin
              result:=constsingle(o.sval);
            end;
          top_double:
            begin
              result:=constdouble(o.dval);
            end;
          {top_string:
            begin
              result:=constastr(o.pcval,o.pcvallen);
            end;
          top_wstring:
            begin
              result:=constwstr(o.pwstrval^.data,getlengthwidestring(o.pwstrval));
            end}
          else
            internalerror(2010122802);
        end;
      end;

    var
      cpu : taicpu;
      i   : integer;
      writer: TExternalAssemblerOutputFile;
    begin
      writer:=owner.writer;
      cpu := taicpu(hp);
      writer.AsmWrite(#9#9);
      writer.AsmWrite(gas_op2str[cpu.opcode]);

      if cpu.ops<>0 then
        begin
          for i:=0 to cpu.ops-1 do
            begin
              writer.AsmWrite(#9);
              if cpu.oper[i]^.typ=top_functype then
                owner.WriteFuncType(cpu.oper[i]^.functype)
              else
                writer.AsmWrite(getopstr(cpu.oper[i]^));
            end;
        end;
      writer.AsmLn;
    end;


  const
    as_wasm32_llvm_mc_info : tasminfo =
       (
         id     : as_wasm32_llvm_mc;
         idtxt  : 'LLVM-MC';
         asmbin : 'llvm-mc';
         asmcmd : '--assemble --arch=wasm32 -mattr=+sign-ext --filetype=obj -o $OBJ $EXTRAOPT $ASM';
         supported_targets : [system_wasm32_embedded,system_wasm32_wasi];
         flags : [af_smartlink_sections];
         labelprefix : '.L';
         labelmaxlen : -1;
         comment : '# ';
         dollarsign : '$';
       );

initialization
  RegisterAssembler(as_wasm32_llvm_mc_info,TLLVMMachineCodePlaygroundAssembler);
end.

