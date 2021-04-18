{
    Copyright (c) 1998-2010 by the Free Pascal team

    This unit implements the WebAssembly text assembler
    The text is in S-expression format and should be consumable
    By either Binaryen or Wabt

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
unit agwat;


{$i fpcdefs.inc}

interface

  uses
    cclasses,systems, cgutils,
    globtype,globals,
    symconst,symbase,symdef,symsym, symtype,symcpu,
    aasmbase,aasmtai,aasmdata,aasmcpu,
    assemble
    ,cutils
    ,cpubase, cgbase
    ,fmodule
    ,verbose, itcpuwasm
    ,cfileutl, tgcpu;

  type
     TWatInstrWriter = class;

     {# This is a derived class which is used to write
        Binaryen-styled assembler.
     }

     { TWatAssembler }

     { TWabtTextAssembler }

     TWabtTextAssembler=class(texternalassembler)
     protected
       dataofs  : integer;

       procedure WriteOutGlobalInt32(const aname: string; val: integer; isconst: boolean = true);

       procedure WriteInstruction(hp: tai);
       procedure WriteProcDef(pd: tprocdef; stub: Boolean = false; stubUnreachable: Boolean = true);
       procedure WriteProcParams(pd: tprocdef);
       procedure WriteProcResult(pd: tprocdef);
       procedure WriteSymtableProcdefs(st: TSymtable);
       procedure WriteSymtableVarSyms(st: TSymtable);
       procedure WriteTempAlloc(p:TAsmList);
       procedure WriteExports(p: TAsmList);
       procedure WriteUnitExports(st: TSymtable);
       procedure WriteImports;

       procedure WriteOutPChar(p: pchar; ofs, len: integer);
       procedure WriteConstString(lbl: tai_label; str: tai_string);
       procedure WriteConstants(p: TAsmList);
     public
       constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
       procedure WriteTree(p:TAsmList);override;
       procedure WriteAsmList;override;
       Function  DoAssemble:boolean;override;
     end;


     {# This is the base class for writing instructions.

        The WriteInstruction() method must be overridden
        to write a single instruction to the assembler
        file.
     }

     { TBinaryenInstrWriter }

     { TWatInstrWriter }

     TWatInstrWriter = class
       constructor create(_owner: TWabtTextAssembler);
       procedure WriteInstruction(hp : tai); virtual;
      protected
       owner: TWabtTextAssembler;
     end;

implementation

    type
      t64bitarray = array[0..7] of byte;
      t32bitarray = array[0..3] of byte;


    const
      line_length = 70;

  {****************************************************************************}
  {                          Support routines                                  }
  {****************************************************************************}

     function fixline(s:string):string;
     {
       return s with all leading and ending spaces and tabs removed
     }
       var
         i,j,k : integer;
       begin
         i:=length(s);
         while (i>0) and (s[i] in [#9,' ']) do
          dec(i);
         j:=1;
         while (j<i) and (s[j] in [#9,' ']) do
          inc(j);
         for k:=j to i do
          if s[k] in [#0..#31,#127..#255] then
           s[k]:='.';
         fixline:=Copy(s,j,i-j+1);
       end;

     function GetWasmName(const st: TSymStr): ansistring;
     begin
       Result := '$'+st;
       Replace(Result, '(','');
       Replace(Result, ')','');
     end;

     function getreferencestring(var ref : treference) : ansistring;
       begin
         if (ref.index<>NR_NO) then
           internalerror(2010122809);
         if assigned(ref.symbol) then
           begin
             // global symbol or field -> full type and name
             // ref.base can be <> NR_NO in case an instance field is loaded.
             // This register is not part of this instruction, it will have
             // been placed on the stack by the previous one.
             result:=GetWasmName(ref.symbol.name);
           end
         else
           begin
             // local symbol -> stack slot, stored in offset
             if ref.base<>NR_STACK_POINTER_REG then
               internalerror(2010122810);
             result:=tostr(ref.offset);
           end;
       end;

     function constsingle(s: single): ansistring;
       begin
         // wat2wasm is using strtof() internally
         str(s, result); //'0x'+hexstr(longint(t32bitarray(s)),8);
       end;

     function constdouble(d: double): ansistring;
        begin
          // force interpretation as double (since we write it out as an
          // integer, we never have to swap the endianess). We have to
          // include the sign separately because of the way Java parses
          // hex numbers (0x8000000000000000 is not a valid long)
          //result:=hexstr(abs(int64(t64bitarray(d))),16);
          //if int64(t64bitarray(d))<0 then
          //  result:='-'+result;
          //result:='0dx'+result;
          str(d, result);
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

  { TWabtTextAssembler }

                procedure TWabtTextAssembler.WriteOutGlobalInt32(const aname: string;
          val: integer; isconst: boolean);
          begin
             writer.AsmWrite(#9);
             writer.AsmWrite('(global $');
             writer.AsmWrite(aname);
             if not isconst then
               writer.AsmWrite(' (mut i32)')
             else
               writer.AsmWrite(' i32');
             writer.AsmWrite(' (i32.const ');
             writer.AsmWrite( tostr(val));
             writer.AsmWrite(')');
             writer.AsmWriteLn(')');
          end;

    procedure TWabtTextAssembler.WriteInstruction(hp: tai);
    var
      cpu : taicpu;
      i   : integer;
      isprm : boolean;
    begin
      //writer.AsmWriteLn('instr');
      cpu := taicpu(hp);
      writer.AsmWrite(#9#9);
      writer.AsmWrite(wasm_op2str[cpu.opcode] );

      if (cpu.opcode = a_call_indirect) then begin
        // special wat2wasm syntax "call_indirect (type x)"
        writer.AsmWrite(#9);
        // todo: fix
        //isprm := true;
        //for i:=1 to length(cpu.typecode) do
        //  if cpu.typecode[i]=':' then
        //     isprm:=false
        //  else begin
        //    if isprm then writer.AsmWrite('(param ')
        //    else writer.AsmWrite('(result ');
        //    case cpu.typecode[i] of
        //      'i': writer.AsmWrite('i32');
        //      'I': writer.AsmWrite('i64');
        //      'f': writer.AsmWrite('f32');
        //      'F': writer.AsmWrite('f64');
        //    end;
        //    writer.AsmWrite(')');
        //  end;
        writer.AsmLn;
        exit;
      end;


      if (cpu.opcode = a_if)  then
        writer.AsmWrite(' (result i32)') //todo: this is a hardcode, but shouldn't
      else

      cpu := taicpu(hp);
      if cpu.ops<>0 then
        begin
          for i:=0 to cpu.ops-1 do
            begin
              writer.AsmWrite(#9);

              if (cpu.opcode in AsmOp_LoadStore) and (cpu.oper[i]^.typ = top_ref) then
                writer.AsmWrite('offset='+tostr( cpu.oper[i]^.ref^.offset))
              else
                writer.AsmWrite(getopstr(cpu.oper[i]^));

            end;
        end;

      if (cpu.opcode = a_call_indirect) then
        // special wat2wasm syntax "call_indirect (type x)"
        writer.AsmWrite(')');

      writer.AsmLn;
    end;

    procedure TWabtTextAssembler.WriteProcDef(pd: tprocdef; stub: Boolean; stubUnreachable: Boolean);
    var
      i : integer;
    begin
      if not assigned(tcpuprocdef(pd).exprasmlist) and
         not(po_abstractmethod in pd.procoptions) and
         ((pd.proctypeoption in [potype_unitinit,potype_unitfinalize])) then
      begin
        exit;
      end;

      writer.AsmWriteLn('');
      if stub and stubUnreachable then begin
        writer.AsmWriteLn(#9';;.weak');
      end;

      writer.AsmWrite(#9'(func ');

      writer.AsmWrite( GetWasmName( pd.mangledname ));
      //procsym.RealName ));
      //writer.AsmWriteln(MethodDefinition(pd));
      {if jvmtypeneedssignature(pd) then
        begin
          writer.AsmWrite('.signature "');
          writer.AsmWrite(tcpuprocdef(pd).jvmmangledbasename(true));
          writer.AsmWriteln('"');
        end;}
      writer.AsmLn;
      WriteProcParams(pd);
      WriteProcResult(pd);

      if not stub then begin
        //WriteTempAlloc(tcpuprocdef(pd).exprasmlist);
        WriteTree(tcpuprocdef(pd).exprasmlist);
      end else begin
        if stubUnreachable then
          writer.AsmWriteLn(#9#9'unreachable');
      end;
      writer.AsmWriteln(#9')');
      writer.AsmLn;
    end;

    procedure TWabtTextAssembler.WriteProcParams(pd: tprocdef);
      var
        i : integer;
        prm : tcpuparavarsym;
      begin
        if not Assigned(pd) or
           not Assigned(pd.paras) or
           (pd.paras.Count=0) then
          exit;

        for i:=0 to pd.paras.Count-1 do
          begin
            prm := tcpuparavarsym(pd.paras[i]);
            writer.AsmWrite(#9#9'(param'#9);
            case prm.paraloc[callerside].Size of
              OS_8..OS_32, OS_S8..OS_S32:
                writer.AsmWrite('i32');
              OS_64, OS_S64:
                writer.AsmWrite('i64');
              OS_F32:
                writer.AsmWrite('f32');
              OS_F64:
                writer.AsmWrite('f64');
            else
              // unsupported calleeside parameter type
              Internalerror(2019093001);
            end;
            writer.AsmWrite(')');
            writer.AsmLn;
          end;
      end;

    procedure TWabtTextAssembler.WriteProcResult(pd: tprocdef);
      var
        bt : TWasmBasicType;
      begin
        if not assigned(pd) or
          not Assigned(pd.returndef) or
          (pd.returndef.size = 0)
          then exit;

        if not defToWasmBasic(pd.returndef, bt) then
          bt := wbt_i32;

        writer.AsmWrite(#9#9'(result'#9);
        case bt of
          wbt_i64: writer.AsmWrite('i64');
          wbt_f32: writer.AsmWrite('f32');
          wbt_f64: writer.AsmWrite('f64');
        else
          writer.AsmWrite('i32');
        end;
        writer.AsmWrite(')');
        writer.AsmLn;
      end;

    procedure TWabtTextAssembler.WriteTree(p: TAsmList);
      var
        ch       : char;
        hp       : tai;
        hp1      : tailineinfo;
        s        : ansistring;
        i,pos    : longint;
        InlineLevel : longint;
        do_line  : boolean;
      const
        WasmBasicTypeStr : array [TWasmBasicType] of string = ('i32','i64','f32','f64');

      begin
        if not assigned(p) then
         exit;

        InlineLevel:=0;
        { lineinfo is only needed for al_procedures (PFV) }
        do_line:=(cs_asm_source in current_settings.globalswitches);
        hp:=tai(p.first);
        while assigned(hp) do
         begin
           prefetch(pointer(hp.next)^);
           if not(hp.typ in SkipLineInfo) then
            begin
              hp1 := hp as tailineinfo;
              current_filepos:=hp1.fileinfo;
               { no line info for inlined code }
               if do_line and (inlinelevel=0) then
                begin
                  { load infile }
                  if lastfileinfo.fileindex<>hp1.fileinfo.fileindex then
                   begin
                     infile:=current_module.sourcefiles.get_file(hp1.fileinfo.fileindex);
                     if assigned(infile) then
                      begin
                        { open only if needed !! }
                        if (cs_asm_source in current_settings.globalswitches) then
                         infile.open;
                      end;
                     { avoid unnecessary reopens of the same file !! }
                     lastfileinfo.fileindex:=hp1.fileinfo.fileindex;
                     { be sure to change line !! }
                     lastfileinfo.line:=-1;
                   end;

                { write source }
                  if (cs_asm_source in current_settings.globalswitches) and
                     assigned(infile) then
                   begin
                     if (infile<>lastinfile) then
                       begin
                         writer.AsmWriteLn(asminfo^.comment+'['+infile.name+']');
                         if assigned(lastinfile) then
                           lastinfile.close;
                       end;
                     if (hp1.fileinfo.line<>lastfileinfo.line) and
                        ((hp1.fileinfo.line<infile.maxlinebuf) or (InlineLevel>0)) then
                       begin
                         if (hp1.fileinfo.line<>0) and
                            ((infile.linebuf^[hp1.fileinfo.line]>=0) or (InlineLevel>0)) then
                           writer.AsmWriteLn(asminfo^.comment+'['+tostr(hp1.fileinfo.line)+'] '+
                             fixline(infile.GetLineStr(hp1.fileinfo.line)));
                         { set it to a negative value !
                         to make that is has been read already !! PM }
                         if (infile.linebuf^[hp1.fileinfo.line]>=0) then
                           infile.linebuf^[hp1.fileinfo.line]:=-infile.linebuf^[hp1.fileinfo.line]-1;
                       end;
                   end;
                  lastfileinfo:=hp1.fileinfo;
                  lastinfile:=infile;
                end;
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
                   begin
  {$ifdef EXTDEBUG}
                     if assigned(tai_tempalloc(hp).problem) then
                       writer.AsmWriteLn(asminfo^.comment+'Temp '+tostr(tai_tempalloc(hp).temppos)+','+
                         tostr(tai_tempalloc(hp).tempsize)+' '+tai_tempalloc(hp).problem^)
                     else
  {$endif EXTDEBUG}
                   end;
               end;

             ait_align :
               begin

               end;

             ait_section :
               begin

               end;

             ait_datablock :
               begin
//                 internalerror(2010122701);
               end;

             ait_const:
               begin
                 writer.AsmWriteln('constant');
//                 internalerror(2010122702);
               end;

             ait_realconst :
               begin
                 internalerror(2010122703);
               end;

             ait_string :
               begin
                 pos:=0;
                  for i:=1 to tai_string(hp).len do
                   begin
                     if pos=0 then
                      begin
                        writer.AsmWrite(#9'strconst: '#9'"');
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
               end;

             ait_label :
               begin
                 // don't write any labels. Wasm don't support it
                 // labels are only allowed with the respective block structures
               end;

             ait_symbol :
               begin
                  if (tai_symbol(hp).sym.typ = AT_FUNCTION) then
                    begin
                    end
                  else
                   begin
                     writer.AsmWrite('data symbol: ');
                     writer.AsmWriteln(tai_symbol(hp).sym.name);
//                     internalerror(2010122706);
                   end;
               end;
             ait_symbol_end :
               begin
               end;

             ait_instruction :
               begin
                 WriteInstruction(hp);
               end;

             ait_force_line,
             ait_function_name : ;

             ait_cutobject :
               begin
               end;

             ait_marker :
               if tai_marker(hp).kind=mark_NoLineInfoStart then
                 inc(InlineLevel)
               else if tai_marker(hp).kind=mark_NoLineInfoEnd then
                 dec(InlineLevel);

             ait_directive :
               begin
                 { the CPU directive is probably not supported by the JVM assembler,
                   so it's commented out }

                 //todo:
                 writer.AsmWrite(asminfo^.comment);

                 if tai_directive(hp).directive=asd_cpu then
                   writer.AsmWrite(asminfo^.comment);
                 writer.AsmWrite('.'+directivestr[tai_directive(hp).directive]+' ');
                 if tai_directive(hp).name<>'' then
                   writer.AsmWrite(tai_directive(hp).name);
                 writer.AsmLn;
               end;

             ait_local :
               begin
                 writer.AsmWrite(#9#9'(local ');
                 if tai_local(hp).name <> '' then
                   begin
                     writer.AsmWrite(' ');
                     writer.AsmWrite(tai_local(hp).name);
                     writer.AsmWrite(' ');
                   end;
                 writer.AsmWrite( WasmBasicTypeStr[ tai_local(hp).bastyp ] );
                 writer.AsmWrite(')');
                 writer.AsmLn;
               end;

             else
               internalerror(2010122707);
           end;
           hp:=tai(hp.next);
         end;
    end;

    procedure TWabtTextAssembler.WriteAsmList;
      var
        hal : tasmlisttype;
      begin
        writer.MarkEmpty;
        writer.AsmWriteLn('(module ');
        writer.AsmWriteLn('(import "env" "memory" (memory 0)) ;;');
        WriteImports;
        WriteConstants(current_asmdata.asmlists[al_const]);
        WriteConstants(current_asmdata.asmlists[al_typedconsts]);

        //current_asmdata.CurrAsmList.labe

        { print all global variables }
        //current_asmdata.AsmSymbolDict

        // for every unit __stack_top is a weak symbol
        // __stack_top is strong only for libraries or programs.
        if current_module.is_unit then
          writer.AsmWriteLn(#9';;.weak');
        writer.AsmWrite(#9'(global $__stack_top (mut i32) (i32.const ');
        writer.AsmWrite(tostr(globals.stacksize));
        writer.AsmWriteLn('))');

        WriteSymtableVarSyms(current_module.globalsymtable);
        WriteSymtableVarSyms(current_module.localsymtable);

        //writer.AsmLn;
        { print all global procedures/functions }
        WriteSymtableProcdefs(current_module.globalsymtable);
        WriteSymtableProcdefs(current_module.localsymtable);

        if current_module.islibrary then begin
          WriteExports(current_asmdata.asmlists[al_exports]);
        end else
          WriteUnitExports(current_module.globalsymtable);

        //WriteSymtableStructDefs(current_module.globalsymtable);
        //WriteSymtableStructDefs(current_module.localsymtable);
        //writer.decorator.LinePrefix := '';
        writer.AsmWriteLn(')');

        writer.AsmLn;
      end;

    function TWabtTextAssembler.DoAssemble: boolean;
    var
      t : tcmdstr;
      begin
        Result:=inherited DoAssemble;
        // the tool updates the symbol flags, so the linker
        // is capable of producing an executable
        if Result then
          if FindExe('wasmtool',true,t) then begin
            if current_module.is_unit then
              // making "common" global variables a week reference
              RequotedExecuteProcess(t,' --weak "$__stack_top" --symbolauto '+ObjFileName)
            else
              RequotedExecuteProcess(t,' --symbolauto '+ObjFileName)
          end
          else
            Message1(exec_e_util_not_found,'wasmtool');
      end;

    constructor TWabtTextAssembler.CreateWithWriter(info: pasminfo;
      wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited CreateWithWriter(info, wr, freewriter, smart);
      end;

    procedure TWabtTextAssembler.WriteSymtableProcdefs(st: TSymtable);
      var
        i   : longint;
        def : tdef;
      begin
        if not assigned(st) then
          exit;
        for i:=0 to st.DefList.Count-1 do
          begin
            def:=tdef(st.DefList[i]);
            case def.typ of
              procdef :
                begin
                  { methods are also in the static/globalsymtable of the unit
                    -> make sure they are only written for the objectdefs that
                    own them }
                  if (not(st.symtabletype in [staticsymtable,globalsymtable]) or
                      (def.owner=st)) and
                     not(df_generic in def.defoptions) and
                     not (po_external in tprocdef(def).procoptions)
                     then
                    begin
                      WriteProcDef(tprocdef(def));
                      if assigned(tprocdef(def).localst) then
                        WriteSymtableProcdefs(tprocdef(def).localst);
                    end;
                end;
              else
                ;
            end;
          end;
      end;

    procedure TWabtTextAssembler.WriteSymtableVarSyms(st: TSymtable);
      var
        i : integer;
        sym : tsym;
      begin
        if not assigned(st) then
          exit;

        for i:=0 to st.SymList.Count-1 do
         begin
           sym:=tsym(st.SymList[i]);
           case sym.typ of
             staticvarsym:
               begin
                 //WriteFieldSym(tabstractvarsym(sym));
                 //if (sym.typ=staticvarsym) and
                 //   assigned(tstaticvarsym(sym).defaultconstsym) then
                 //  WriteFieldSym(tabstractvarsym(tstaticvarsym(sym).defaultconstsym));
                 WriteOutGlobalInt32(tcpustaticvarsym(sym).mangledname, dataofs);
                 inc(dataofs, tcpustaticvarsym(sym).getsize);
               end;
             fieldvarsym:
               begin
                 writer.AsmWriteLn(';; field');
               end;
             constsym:
               begin
                 //if (sym.typ=staticvarsym) and
                 //   assigned(tstaticvarsym(sym).defaultconstsym) then
                 //  WriteFieldSym(tabstractvarsym(tstaticvarsym(sym).defaultconstsym));
                 //{ multiple procedures can have constants with the same name }
                 //if not assigned(sym.owner.defowner) or
                 //   (tdef(sym.owner.defowner).typ<>procdef) then
                 //  WriteConstSym(tconstsym(sym));
                 writer.AsmWriteLn(';; constant');
               end;
             {procsym:
               begin
                 for j:=0 to tprocsym(sym).procdeflist.count-1 do
                   if not(df_generic in tprocdef(tprocsym(sym).procdeflist[j]).defoptions) then
                     WriteSymtableVarSyms(tprocdef(tprocsym(sym).procdeflist[j]).localst);
               end;}
             else
               ;
           end;
         end;
      end;

    procedure TWabtTextAssembler.WriteTempAlloc(p: TAsmList);
      var
        hp: tai;
        tmp: array of tai_tempalloc;
        mx : integer;
        i  : integer;
      begin
        if not assigned(p) then
         exit;

        mx := -1;
        hp:=tai(p.first);
        while assigned(hp) do
         begin
           //prefetch(pointer(hp.next)^);
           if (hp.typ = ait_tempalloc) and
             tai_tempalloc(hp).allocation and
             (tai_tempalloc(hp).temppos >= mx) then
               mx := tai_tempalloc(hp).temppos+1;

           hp:=tai(hp.next);
         end;

        if (mx <= 0) then exit; // no temp allocations used

        SetLength(tmp, mx);

        hp:=tai(p.first);
        while assigned(hp) do
          begin
            //prefetch(pointer(hp.next)^);

            if (hp.typ = ait_tempalloc) and
              (tai_tempalloc(hp).allocation) and
              (tmp[ tai_tempalloc(hp).temppos ] = nil) then
              begin
                tmp[ tai_tempalloc(hp).temppos ] := tai_tempalloc(hp);
                dec(mx);
                if mx = 0 then break;
              end;
            hp:=tai(hp.next);
          end;

        for i:=0 to length(tmp)-1 do
          begin
            if tmp[i] = nil then continue;
            writer.AsmWrite(#9'(local'#9);
            if tmp[i].tempsize<=4 then writer.AsmWrite('i32')
            else if tmp[i].tempsize = 8 then writer.AsmWrite('i64');
            writer.AsmWrite(')');

            writer.AsmWrite(#9+asminfo^.comment+'Temp '+tostr(tmp[i].temppos)+','+
              tostr(tmp[i].tempsize)+' '+tempallocstr[tmp[i].allocation]);

            writer.AsmLn;
          end;

      end;

    procedure TWabtTextAssembler.WriteExports(p: TAsmList);
    var
      hp: tai;
      x: tai_impexp;
      cnt: integer;
    begin
      if not Assigned(p) then Exit;
      hp:=tai(p.First);
      if not Assigned(hp) then Exit;

      cnt := 0;
      while Assigned(hp) do begin
        case hp.typ of
          ait_importexport:
            inc(cnt);
          else
            ;
        end;
        hp := tai_impexp(hp.Next);
      end;

      // writting out table, so wat2wasm can create reallocation symbols

      writer.AsmWrite(#9'(table ');
      writer.AsmWrite(tostr(cnt));
      writer.AsmWrite(' anyfunc)');
      writer.AsmWriteLn(#9'(elem 0 (i32.const 0) ');
      hp:=tai(p.First);
      while Assigned(hp) do begin
        case hp.typ of
          ait_importexport:
          begin
            x:=tai_impexp(hp);
            writer.AsmWrite(#9#9);
            writer.AsmWriteLn(GetWasmName(x.intname));
          end;
          else
            ;
        end;
        hp := tai_impexp(hp.Next);
      end;
      writer.AsmWriteLn(#9') ');

      // writing export sections
      hp:=tai(p.First);
      while Assigned(hp) do begin
        case hp.typ of
          ait_importexport:
          begin
            x:=tai_impexp(hp);
            writer.AsmWrite(#9#9'(export "');
            writer.AsmWrite(x.extname);
            writer.AsmWrite('" (');
            case x.symstype of
              ie_Func: writer.AsmWrite('func');
              else
                ;
            end;
            writer.AsmWrite(' ');
            writer.AsmWrite(GetWasmName(x.intname));
            writer.AsmWrite('))');
            writer.AsmLn;
          end;
          else
            ;
        end;
        hp := tai_impexp(hp.Next);
      end;
    end;

    procedure TWabtTextAssembler.WriteUnitExports(st: TSymtable);
    var
      i   : longint;
      def : tdef;
    begin
      if not assigned(st) then
        exit;

      writer.AsmWrite(#9'(table 0 funcref)');
      writer.AsmWriteLn(#9'(elem 0 (i32.const 0) ');
      for i:=0 to st.DefList.Count-1 do
        begin
          def:=tdef(st.DefList[i]);
          case def.typ of
            procdef :
              begin
                { methods are also in the static/globalsymtable of the unit
                  -> make sure they are only written for the objectdefs that
                  own them }
                  if
                   not (po_external in tprocdef(def).procoptions)
                   then
                  begin
                    writer.AsmWrite(#9#9'$');
                    writer.AsmWriteLn(tprocdef(def).mangledname);
                  end;
              end;
            else
              ;
          end;
        end;
      writer.AsmWriteLn(#9') ');
    end;

    procedure TWabtTextAssembler.WriteImports;
      var
        i    : integer;
        def  : tdef;
        proc : tprocdef;
        sym  : tsym;
        j    : integer;
        psym : tprocsym;
      begin
        for i:=0 to current_module.deflist.Count-1 do begin
          def:=tdef(current_module.deflist[i]);
          { since commit 48986 deflist might have NIL entries }
          if assigned(def) and (def.typ=procdef) then begin
            proc := tprocdef(def);
            if (po_external in proc.procoptions) and assigned(proc.import_dll) then begin
              writer.AsmWrite(#9'(import "');
              writer.AsmWrite(proc.import_dll^);
              writer.AsmWrite('" "');
              writer.AsmWrite(proc.import_name^);
              writer.AsmWrite('" ');
              WriteProcDef(proc);
              writer.AsmWriteLn(')');
            end;
          end;
        end;

        // any symbol used from another unit must be fully declared in .wat
        // (reference by symbol name only doesn't work in Wasm)
        // the entire entry should declared (as imported) symbol.
        // The wasm-import name (name of exernal module and name)
        // is not important, as the linker would be using the symbol name
        // while linking.
        for i:=0 to current_module.unitimportsyms.Count-1 do begin
          sym := tsym(current_module.unitimportsyms[i]);
          if sym.typ = procsym then begin
            psym := tprocsym(sym);
            if psym.ProcdefList.Count>0 then
              proc := tprocdef(psym.ProcdefList[0])
            else
              proc := nil;
            if proc<>nil then begin
              {writer.AsmWrite(#9'(import "_fpc_use" "');
              writer.AsmWrite(proc.mangledname);
              writer.AsmWrite('" ');}
              WriteProcDef(proc, true);
              //writer.AsmWrite(')');
            end;
          end;
        end;
      end;


    procedure TWabtTextAssembler.WriteOutPChar(p: pchar; ofs, len: integer);
        var
          i : integer;
          s : string;
          ch : char;
        begin
          for i:=ofs to ofs+len-1 do begin
               ch:=p[i];
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
            end;
        end;

        procedure TWabtTextAssembler.WriteConstString(lbl: tai_label;
            str: tai_string);
          var
            i : integer;
            ch : char;
            s : string;
          begin
            WriteOutGlobalInt32(lbl.labsym.Name, dataofs);
            // (data (get_global $test) "00")
            writer.AsmWrite(#9);
            writer.AsmWrite('(data (i32.const ');
            writer.AsmWrite(tostr(dataOfs));
            writer.AsmWrite(') "');
            // can be broken apart in multiple data() if needs to be
            WriteOutPChar(str.str, 0, str.len);
            writer.AsmWrite('"');
            writer.AsmWriteLn(') ;; value of '+ lbl.labsym.Name);
            inc(dataofs, str.len);
          end;

    procedure TWabtTextAssembler.WriteConstants(p: TAsmList);
      var
        i : integer;
        hp : tai;
        dt : tai;
      begin
        //WriteTree(p);
        hp:=tai(p.First);
        while Assigned(hp) do begin
          if (hp.typ = ait_label) then begin
            dt:=tai(hp.Next);
            if Assigned(dt) then begin
              case dt.typ of
                ait_string:
                begin
                  WriteConstString(tai_label(hp), tai_string(dt));
                  hp:=dt;
                end;
                else
                  ;
              end;
            end;
          end;
          hp:=tai(hp.Next);
        end;
      end;


{ TWatInstrWriter }

  constructor TWatInstrWriter.create(_owner: TWabtTextAssembler);
    begin
      inherited create;
      owner := _owner;
    end;

  procedure TWatInstrWriter.WriteInstruction(hp: tai);
    begin

    end;


  const
    as_wasm_wabt_info : tasminfo =
       (
         id     : as_wasm32_wabt;
         idtxt  : 'WABT';
         asmbin : 'wasa';
         asmcmd : '-r --no-canonicalize-leb128s -o $OBJ $EXTRAOPT $ASM';
         supported_targets : [system_wasm32_embedded,system_wasm32_wasi];
         flags : [];
         labelprefix : 'L';
         labelmaxlen : -1;
         comment : ';; ';
         dollarsign : '$';
       );

initialization
  RegisterAssembler(as_wasm_wabt_info, TWabtTextAssembler);

end.
