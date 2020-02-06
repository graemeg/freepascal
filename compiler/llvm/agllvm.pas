{
    Copyright (c) 1998-2013 by the Free Pascal team

    This unit implements the generic part of the LLVM IR writer

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
unit agllvm;

{$i fpcdefs.inc}

interface

    uses
      cclasses,
      globtype,globals,systems,
      aasmbase,aasmtai,aasmdata,
      assemble,
      aasmllvm, aasmllvmmetadata;

    type
      TLLVMInstrWriter = class;

      TLLVMModuleInlineAssemblyDecorator = class(IExternalAssemblerOutputFileDecorator)
       function LineFilter(const s: AnsiString): AnsiString;
       function LinePrefix: AnsiString;
       function LinePostfix: AnsiString;
       function LineEnding(const deflineending: ShortString): ShortString;
      end;

      TLLVMFunctionInlineAssemblyDecorator = class(IExternalAssemblerOutputFileDecorator)
       function LineFilter(const s: AnsiString): AnsiString;
       function LinePrefix: AnsiString;
       function LinePostfix: AnsiString;
       function LineEnding(const deflineending: ShortString): ShortString;
      end;

      TLLVMAssember=class(texternalassembler)
      protected
        ffuncinlasmdecorator: TLLVMFunctionInlineAssemblyDecorator;
        fdecllevel: longint;

        procedure WriteExtraHeader;virtual;
        procedure WriteExtraFooter;virtual;
        procedure WriteInstruction(hp: tai);
        procedure WriteLlvmInstruction(hp: tai);
        procedure WriteDirectiveName(dir: TAsmDirective); virtual;
        procedure WriteRealConst(hp: tai_realconst; do_line: boolean);
        procedure WriteOrdConst(hp: tai_const);
        procedure WriteTai(const replaceforbidden: boolean; const do_line, inmetadata: boolean; var InlineLevel: cardinal; var asmblock: boolean; var hp: tai);
       public
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
        procedure WriteTree(p:TAsmList);override;
        procedure WriteAsmList;override;
        procedure WriteFunctionInlineAsmList(list: tasmlist);
        destructor destroy; override;
       protected
        InstrWriter: TLLVMInstrWriter;
      end;

      TLLVMLLCAssember=class(TLLVMAssember)
      public
       function MakeCmdLine: TCmdStr; override;
      end;

      TLLVMClangAssember=class(TLLVMAssember)
      public
       function MakeCmdLine: TCmdStr; override;
       function DoAssemble: boolean; override;
       function RerunAssembler: boolean; override;
      protected
       function DoPipe: boolean; override;
      private
       fnextpass: byte;
      end;


      {# This is the base class for writing instructions.

         The WriteInstruction() method must be overridden
         to write a single instruction to the assembler
         file.
      }
      TLLVMInstrWriter = class
        constructor create(_owner: TLLVMAssember);
        procedure WriteInstruction(hp : tai);
       protected
        owner: TLLVMAssember;
        fstr: TSymStr;

        function getopcodestr(hp: taillvm): TSymStr;
        function getopstr(const o:toper; refwithalign: boolean) : TSymStr;
        procedure writeparas(const paras: tfplist);
        procedure WriteAsmRegisterAllocationClobbers(list: tasmlist);
      end;


implementation

    uses
      SysUtils,
      cutils,cfileutl,
      fmodule,verbose,
      objcasm,
      aasmcnst,symconst,symdef,symtable,
      llvmbase,itllvm,llvmdef,
      cgbase,cgutils,cpubase,cpuinfo,llvminfo;

    const
      line_length = 70;

    type
{$ifdef cpuextended}
      t80bitarray = array[0..9] of byte;
{$endif cpuextended}
      t64bitarray = array[0..7] of byte;
      t32bitarray = array[0..3] of byte;

{****************************************************************************}
{                          Support routines                                  }
{****************************************************************************}

    function single2str(d : single) : string;
      var
         hs : string;
      begin
         str(d,hs);
      { replace space with + }
         if hs[1]=' ' then
          hs[1]:='+';
         single2str:=hs
      end;

    function double2str(d : double) : string;
      var
         hs : string;
      begin
         str(d,hs);
      { replace space with + }
         if hs[1]=' ' then
          hs[1]:='+';
         double2str:=hs
      end;

    function extended2str(e : extended) : string;
      var
         hs : string;
      begin
         str(e,hs);
      { replace space with + }
         if hs[1]=' ' then
          hs[1]:='+';
         extended2str:=hs
      end;


{****************************************************************************}
{               Decorator for module-level inline assembly                   }
{****************************************************************************}

    function TLLVMModuleInlineAssemblyDecorator.LineFilter(const s: AnsiString): AnsiString;
      var
        i: longint;
      begin
        result:='';
        for i:=1 to length(s) do
          begin
            case s[i] of
              #0..#31,
              #127..#255,
              '"','\':
                result:=result+
                        '\'+
                        chr((ord(s[i]) shr 4)+ord('0'))+
                        chr((ord(s[i]) and $f)+ord('0'));
            else
              result:=result+s[i];
            end;
          end;
        end;


    function TLLVMModuleInlineAssemblyDecorator.LinePrefix: AnsiString;
      begin
        result:='module asm "';
      end;


    function TLLVMModuleInlineAssemblyDecorator.LinePostfix: AnsiString;
      begin
        result:='"';
      end;


    function TLLVMModuleInlineAssemblyDecorator.LineEnding(const deflineending: ShortString): ShortString;
      begin
        result:=deflineending
      end;


{****************************************************************************}
{              Decorator for function-level inline assembly                  }
{****************************************************************************}


    function TLLVMFunctionInlineAssemblyDecorator.LineFilter(const s: AnsiString): AnsiString;
      var
        i: longint;
      begin
        result:='';
        for i:=1 to length(s) do
          begin
            case s[i] of
              { escape dollars }
              '$':
                 result:=result+'$$';
              { ` is used as placeholder for a single dollar (reference to
                 argument to the inline assembly) }
              '`':
                 result:=result+'$';
              #0..#31,
              #127..#255,
              '"','\':
                result:=result+
                        '\'+
                        chr((ord(s[i]) shr 4)+ord('0'))+
                        chr((ord(s[i]) and $f)+ord('0'));
            else
              result:=result+s[i];
            end;
          end;
        end;


    function TLLVMFunctionInlineAssemblyDecorator.LinePrefix: AnsiString;
      begin
        result:='';
      end;


    function TLLVMFunctionInlineAssemblyDecorator.LinePostfix: AnsiString;
      begin
        result:='';
      end;


    function TLLVMFunctionInlineAssemblyDecorator.LineEnding(const deflineending: ShortString): ShortString;
      begin
        result:='\0A';
      end;




 {****************************************************************************}
 {                        LLVM Instruction writer                             }
 {****************************************************************************}

    function getregisterstring(reg: tregister): ansistring;
      begin
        if getregtype(reg)=R_METADATAREGISTER then
          result:='!"'+tllvmmetadata.getregstring(reg)+'"'
        else
          begin
            if getregtype(reg)=R_TEMPREGISTER then
              result:='%tmp.'
            else
              result:='%reg.'+tostr(byte(getregtype(reg)))+'_';
            result:=result+tostr(getsupreg(reg));
          end;
      end;


    function getreferencealignstring(var ref: treference) : ansistring;
      begin
        result:=', align '+tostr(ref.alignment);
      end;


    function getreferencestring(var ref : treference; withalign: boolean) : ansistring;
      begin
        result:='';
        if assigned(ref.relsymbol) or
           (assigned(ref.symbol) and
            (ref.base<>NR_NO)) or
           (ref.index<>NR_NO) or
           (ref.offset<>0) then
          begin
            result:=' **(error ref: ';
            if assigned(ref.symbol) then
              result:=result+'sym='+ref.symbol.name+', ';
            if assigned(ref.relsymbol) then
              result:=result+'sym='+ref.relsymbol.name+', ';
            if ref.base=NR_NO then
              result:=result+'base=NR_NO, ';
            if ref.index<>NR_NO then
              result:=result+'index<>NR_NO, ';
            if ref.offset<>0 then
              result:=result+'offset='+tostr(ref.offset);
            result:=result+')**';
            internalerror(2013060225);
          end;
         if ref.base<>NR_NO then
           result:=result+getregisterstring(ref.base)
         else if assigned(ref.symbol) then
           result:=result+LlvmAsmSymName(ref.symbol)
         else
           result:=result+'null';
         if withalign then
           result:=result+getreferencealignstring(ref);
      end;


   procedure TLLVMInstrWriter.writeparas(const paras: tfplist);
     var
       i: longint;
       tmpinline: cardinal;
       para: pllvmcallpara;
       tmpasmblock: boolean;
       hp: tai;
     begin
       tmpinline:=1;
       tmpasmblock:=false;
       owner.writer.AsmWrite(fstr);
       fstr:='';
       owner.writer.AsmWrite('(');
       for i:=0 to paras.count-1 do
         begin
           if i<>0 then
             owner.writer.AsmWrite(', ');
           para:=pllvmcallpara(paras[i]);
           owner.writer.AsmWrite(llvmencodetypename(para^.def));
           if para^.valueext<>lve_none then
             owner.writer.AsmWrite(llvmvalueextension2str[para^.valueext]);
           if para^.byval then
             owner.writer.AsmWrite(' byval');
           if para^.sret then
             owner.writer.AsmWrite(' sret');
           { For byval, this means "alignment on the stack" and of the passed source data.
             For other pointer parameters, this means "alignment of the passed source data" }
           if (para^.alignment<>std_param_align) or
              (para^.alignment<0) then
             begin
               owner.writer.AsmWrite(' align ');
               owner.writer.AsmWrite(tostr(abs(para^.alignment)));
             end;
           case para^.typ of
             top_reg:
               begin
                 owner.writer.AsmWrite(' ');
                 owner.writer.AsmWrite(getregisterstring(para^.register));
               end;
             top_ref:
               begin
                 owner.writer.AsmWrite(' ');
                 owner.writer.AsmWrite(llvmasmsymname(para^.sym));
               end;
             top_const:
               begin
                 owner.writer.AsmWrite(' ');
                 owner.writer.AsmWrite(tostr(para^.value));
               end;
             top_tai:
               begin
                 tmpinline:=1;
                 tmpasmblock:=false;
                 hp:=para^.ai;
                 owner.writer.AsmWrite(fstr);
                 fstr:='';
                 owner.WriteTai(false,false,para^.def=llvm_metadatatype,tmpinline,tmpasmblock,hp);
               end;
             { empty records }
             top_undef:
               owner.writer.AsmWrite(' undef');
             else
               internalerror(2014010801);
           end;
         end;
       owner.writer.AsmWrite(')');
     end;


   function llvmdoubletostr(const d: double): TSymStr;
     type
       tdoubleval = record
         case byte of
           1: (d: double);
           2: (i: int64);
       end;
     begin
       { "When using the hexadecimal form, constants of types half,
         float, and double are represented using the 16-digit form shown
         above (which matches the IEEE754 representation for double)"

         And always in big endian form (sign bit leftmost)
       }
       result:='0x'+hexstr(tdoubleval(d).i,16);
     end;


{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
    function llvmextendedtostr(const e: extended): TSymStr;
      var
        extendedval: record
          case byte of
            1: (e: extended);
            2: (r: packed record
      {$ifdef FPC_LITTLE_ENDIAN}
                  l: int64;
                  h: word;
      {$else FPC_LITTLE_ENDIAN}
                  h: int64;
                  l: word;
      {$endif FPC_LITTLE_ENDIAN}
                end;
               );
        end;
      begin
        extendedval.e:=e;
        { hex format is always big endian in llvm }
        result:='0xK'+hexstr(extendedval.r.h,sizeof(extendedval.r.h)*2)+
                      hexstr(extendedval.r.l,sizeof(extendedval.r.l)*2);
      end;

{$endif cpuextended}


   function TLLVMInstrWriter.getopstr(const o:toper; refwithalign: boolean) : TSymStr;
     var
       hp: tai;
       tmpinline: cardinal;
       tmpasmblock: boolean;
     begin
       case o.typ of
         top_reg:
           getopstr:=getregisterstring(o.reg);
         top_const:
           getopstr:=tostr(int64(o.val));
         top_ref:
           if o.ref^.refaddr=addr_full then
             begin
               getopstr:='';
               if assigned(o.ref^.symbol) then
                 getopstr:=LlvmAsmSymName(o.ref^.symbol)
               else
                 getopstr:='null';
               if o.ref^.offset<>0 then
                 internalerror(2013060223);
             end
           else
             getopstr:=getreferencestring(o.ref^,refwithalign);
         top_def:
           begin
             getopstr:=llvmencodetypename(o.def);
           end;
         top_cond:
           begin
             getopstr:=llvm_cond2str[o.cond];
           end;
         top_fpcond:
           begin
             getopstr:=llvm_fpcond2str[o.fpcond];
           end;
         top_single,
         top_double:
           begin
             { "When using the hexadecimal form, constants of types half,
               float, and double are represented using the 16-digit form shown
               above (which matches the IEEE754 representation for double)"

               And always in big endian form (sign bit leftmost)
             }
             if o.typ=top_double then
               result:=llvmdoubletostr(o.dval)
             else
               result:=llvmdoubletostr(o.sval)
           end;
         top_para:
           begin
             writeparas(o.paras);
             result:='';
           end;
         top_tai:
           begin
             if assigned(o.ai) then
               begin
                 tmpinline:=1;
                 tmpasmblock:=false;
                 hp:=o.ai;
                 owner.writer.AsmWrite(fstr);
                 fstr:='';
                 owner.WriteTai(false,false,false,tmpinline,tmpasmblock,hp);
               end;
             result:='';
           end;
{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
         top_extended80:
           begin
             result:=llvmextendedtostr(o.eval);
           end;
{$endif cpuextended}
         top_undef:
           result:='undef';
         top_callingconvention:
           result:=llvm_callingconvention_name(o.callingconvention);
         else
           internalerror(2013060227);
       end;
     end;


   procedure TLLVMInstrWriter.WriteAsmRegisterAllocationClobbers(list: tasmlist);
     var
       hp: tai;
     begin
       hp:=tai(list.first);
       while assigned(hp) do
         begin
           if (hp.typ=ait_regalloc) and
              (tai_regalloc(hp).ratype=ra_alloc) then
             begin
               owner.writer.AsmWrite(',~{');
               owner.writer.AsmWrite(std_regname(tai_regalloc(hp).reg));
               owner.writer.AsmWrite('}');
             end;
           hp:=tai(hp.next);
         end;
     end;


  procedure TLLVMInstrWriter.WriteInstruction(hp: tai);
    var
      op: tllvmop;
      tmpstr,
      sep: TSymStr;
      i, opstart: longint;
      nested: boolean;
      opdone,
      done: boolean;
    begin
      op:=taillvm(hp).llvmopcode;
      { we write everything immediately rather than adding it into a string,
        because operands may contain other tai that will also write things out
        (and their output must come after everything that was processed in this
         instruction, such as its opcode or previous operands) }
      if owner.fdecllevel=0 then
        owner.writer.AsmWrite(#9);
      sep:=' ';
      opdone:=false;
      done:=false;
      opstart:=0;
      nested:=false;
      case op of
        la_type:
           begin
             owner.writer.AsmWrite(llvmtypeidentifier(taillvm(hp).oper[0]^.def));
             owner.writer.AsmWrite(' = type ');
             owner.writer.AsmWrite(llvmencodetypedecl(taillvm(hp).oper[0]^.def));
             done:=true;
           end;
        la_asmblock:
          begin
            owner.writer.AsmWrite('call void asm sideeffect "');
            owner.WriteFunctionInlineAsmList(taillvm(hp).oper[0]^.asmlist);
            owner.writer.AsmWrite('","');
            { we pass all accessed local variables as in/out address parameters,
              since we don't analyze the assembly code to determine what exactly
              happens to them; this is also compatible with the regular code
              generators, which always place local place local variables
              accessed from assembly code in memory }
            for i:=0 to taillvm(hp).oper[1]^.paras.Count-1 do
              begin
                owner.writer.AsmWrite('=*m,');
              end;
            owner.writer.AsmWrite('~{memory},~{fpsr},~{flags}');
            WriteAsmRegisterAllocationClobbers(taillvm(hp).oper[0]^.asmlist);
            owner.writer.AsmWrite('"');
            writeparas(taillvm(hp).oper[1]^.paras);
            done:=true;
          end;
        la_load,
        la_getelementptr:
          begin
            if (taillvm(hp).oper[0]^.typ<>top_reg) or
               (taillvm(hp).oper[0]^.reg<>NR_NO) then
              owner.writer.AsmWrite(getopstr(taillvm(hp).oper[0]^,false)+' = ')
            else
              nested:=true;
            opstart:=1;
            owner.writer.AsmWrite(getopcodestr(taillvm(hp)));
            opdone:=true;
            if nested then
              owner.writer.AsmWrite(' (')
            else
              owner.writer.AsmWrite(' ');
            { can't just dereference the type, because it may be an
              implicit pointer type such as a class -> resort to string
              manipulation... Not very clean :( }
            tmpstr:=llvmencodetypename(taillvm(hp).spilling_get_reg_type(0));
            if op=la_getelementptr then
              begin
                if tmpstr[length(tmpstr)]<>'*' then
                  begin
                    writeln(tmpstr);
                    internalerror(2016071101);
                  end
                else
                  setlength(tmpstr,length(tmpstr)-1);
              end;
            owner.writer.AsmWrite(tmpstr);
            owner.writer.AsmWrite(',');
          end;
        la_ret, la_br, la_switch, la_indirectbr,
        la_resume,
        la_unreachable,
        la_store,
        la_fence,
        la_cmpxchg,
        la_atomicrmw,
        la_catch,
        la_filter,
        la_cleanup:
          begin
            { instructions that never have a result }
          end;
        la_call,
        la_invoke:
          begin
            if taillvm(hp).oper[1]^.reg<>NR_NO then
              owner.writer.AsmWrite(getregisterstring(taillvm(hp).oper[1]^.reg)+' = ');
            opstart:=2;
            owner.writer.AsmWrite(getopcodestr(taillvm(hp)));
            tmpstr:=llvm_callingconvention_name(taillvm(hp).oper[2]^.callingconvention);
            if tmpstr<>'' then
              begin
                owner.writer.AsmWrite(' ');
                owner.writer.AsmWrite(tmpstr);
              end;
            opdone:=true;
            tmpstr:=llvmencodetypename(taillvm(hp).oper[3]^.def);
            if tmpstr[length(tmpstr)]<>'*' then
              begin
                writeln(tmpstr);
                internalerror(2016071102);
              end
            else
              setlength(tmpstr,length(tmpstr)-1);
            owner.writer.AsmWrite(tmpstr);
            opstart:=4;
          end;
        la_blockaddress:
          begin
            { nested -> no type }
            if owner.fdecllevel = 0 then
              begin
                owner.writer.AsmWrite(getopstr(taillvm(hp).oper[0]^,false));
                owner.writer.AsmWrite(' ');
              end;
            owner.writer.AsmWrite('blockaddress(');
            owner.writer.AsmWrite(getopstr(taillvm(hp).oper[1]^,false));
            { getopstr would add a "label" qualifier, which blockaddress does
              not want }
            owner.writer.AsmWrite(',%');
            with taillvm(hp).oper[2]^ do
              begin
                if (typ<>top_ref) or
                   (ref^.refaddr<>addr_full) then
                  internalerror(2016112001);
                owner.writer.AsmWrite(ref^.symbol.name);
              end;
            nested:=true;
            done:=true;
          end;
        la_alloca:
          begin
            owner.writer.AsmWrite(getreferencestring(taillvm(hp).oper[0]^.ref^,false)+' = ');
            sep:=' ';
            opstart:=1;
          end;
        la_trunc, la_zext, la_sext, la_fptrunc, la_fpext,
        la_fptoui, la_fptosi, la_uitofp, la_sitofp,
        la_ptrtoint, la_inttoptr,
        la_bitcast:
          begin
            { destination can be empty in case of nested constructs, or
              data initialisers }
            if (taillvm(hp).oper[0]^.typ<>top_reg) or
               (taillvm(hp).oper[0]^.reg<>NR_NO) then
              owner.writer.AsmWrite(getopstr(taillvm(hp).oper[0]^,false)+' = ')
            else
              nested:=true;
            owner.writer.AsmWrite(getopcodestr(taillvm(hp)));
            if not nested then
              owner.writer.AsmWrite(' ')
            else
              owner.writer.AsmWrite(' (');
            owner.writer.AsmWrite(getopstr(taillvm(hp).oper[1]^,false));
            { if there's a tai operand, its def is used instead of an
              explicit def operand }
            if taillvm(hp).ops=4 then
              begin
                owner.writer.AsmWrite(' ');
                owner.writer.AsmWrite(getopstr(taillvm(hp).oper[2]^,false));
                opstart:=3;
              end
            else
              opstart:=2;
            owner.writer.AsmWrite(' to ');
            owner.writer.AsmWrite(getopstr(taillvm(hp).oper[opstart]^,false));
            done:=true;
          end
        else
          begin
            if (taillvm(hp).oper[0]^.typ<>top_reg) or
               (taillvm(hp).oper[0]^.reg<>NR_NO) then
              begin
                owner.writer.AsmWrite(getopstr(taillvm(hp).oper[0]^,true)+' = ');
              end
            else
              nested:=true;
            sep:=' ';
            opstart:=1
          end;
      end;
      { process operands }
      if not done then
        begin
          if not opdone then
            begin
              owner.writer.AsmWrite(getopcodestr(taillvm(hp)));
              if nested then
                owner.writer.AsmWrite(' (');
            end;
          if taillvm(hp).ops<>0 then
            begin
              for i:=opstart to taillvm(hp).ops-1 do
                begin
                   owner.writer.AsmWrite(sep);
                   { special invoke interjections: "to label X unwind label Y" }
                   if (op=la_invoke) then
                     case i of
                       6: owner.writer.AsmWrite('to ');
                       7: owner.writer.AsmWrite('unwind ');
                     end;

                   owner.writer.AsmWrite(getopstr(taillvm(hp).oper[i]^,op in [la_load,la_store]));
                   if (taillvm(hp).oper[i]^.typ in [top_def,top_cond,top_fpcond]) or
                      (op in [la_call,la_invoke,la_landingpad,la_catch,la_filter,la_cleanup]) then
                     sep :=' '
                   else
                     sep:=', ';
                end;
            end;
        end;
      if op=la_alloca then
        owner.writer.AsmWrite(getreferencealignstring(taillvm(hp).oper[0]^.ref^));
      if nested then
        owner.writer.AsmWrite(')')
      else if owner.fdecllevel=0 then
        owner.writer.AsmLn;
    end;


  function TLLVMInstrWriter.getopcodestr(hp: taillvm): TSymStr;
    begin
      result:=llvm_op2str[hp.llvmopcode];
      case hp.llvmopcode of
        la_load:
          begin
            if vol_read in hp.oper[2]^.ref^.volatility then
              result:=result+' volatile';
          end;
        la_store:
          begin
            if vol_write in hp.oper[3]^.ref^.volatility then
              result:=result+' volatile';
          end;
        else
          ;
      end;
    end;


{****************************************************************************}
{                          LLVM Assembler writer                              }
{****************************************************************************}

    destructor TLLVMAssember.Destroy;
      begin
        InstrWriter.free;
        ffuncinlasmdecorator.free;
        inherited destroy;
      end;


    procedure TLLVMAssember.WriteTree(p:TAsmList);
    var
      hp       : tai;
      InlineLevel : cardinal;
      asmblock: boolean;
      do_line  : boolean;
      replaceforbidden: boolean;
    begin
      if not assigned(p) then
       exit;
      replaceforbidden:=asminfo^.dollarsign<>'$';

      InlineLevel:=0;
      asmblock:=false;
      { lineinfo is only needed for al_procedures (PFV) }
      do_line:=(cs_asm_source in current_settings.globalswitches) or
               ((cs_lineinfo in current_settings.moduleswitches)
                 and (p=current_asmdata.asmlists[al_procedures]));
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

         WriteTai(replaceforbidden, do_line, false, InlineLevel, asmblock, hp);
         hp:=tai(hp.next);
       end;
    end;


    procedure TLLVMAssember.WriteExtraHeader;
      begin
        writer.AsmWrite('target datalayout = "');
        writer.AsmWrite(target_info.llvmdatalayout);
        writer.AsmWriteln('"');
        writer.AsmWrite('target triple = "');
        writer.AsmWrite(llvm_target_name);
        writer.AsmWriteln('"');
      end;


    procedure TLLVMAssember.WriteExtraFooter;
      begin
      end;


    procedure TLLVMAssember.WriteInstruction(hp: tai);
      begin

      end;


    procedure TLLVMAssember.WriteLlvmInstruction(hp: tai);
      begin
        InstrWriter.WriteInstruction(hp);
      end;


    procedure TLLVMAssember.WriteRealConst(hp: tai_realconst; do_line: boolean);
      begin
        if fdecllevel=0 then
          begin
            case tai_realconst(hp).realtyp of
              aitrealconst_s32bit:
                writer.AsmWriteLn(asminfo^.comment+'value: '+single2str(tai_realconst(hp).value.s32val));
              aitrealconst_s64bit:
                writer.AsmWriteLn(asminfo^.comment+'value: '+double2str(tai_realconst(hp).value.s64val));
{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
              { can't write full 80 bit floating point constants yet on non-x86 }
              aitrealconst_s80bit:
                writer.AsmWriteLn(asminfo^.comment+'value: '+extended2str(tai_realconst(hp).value.s80val));
{$endif cpuextended}
              aitrealconst_s64comp:
                writer.AsmWriteLn(asminfo^.comment+'value: '+extended2str(tai_realconst(hp).value.s64compval));
              else
                internalerror(2014050604);
            end;
            internalerror(2016120202);
          end;
        case hp.realtyp of
          aitrealconst_s32bit:
            writer.AsmWrite(llvmdoubletostr(hp.value.s32val));
          aitrealconst_s64bit:
            writer.AsmWriteln(llvmdoubletostr(hp.value.s64val));
{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
          aitrealconst_s80bit:
            writer.AsmWrite(llvmextendedtostr(hp.value.s80val));
{$endif defined(cpuextended)}
          aitrealconst_s64comp:
            { handled as int64 most of the time in llvm }
            writer.AsmWrite(tostr(round(hp.value.s64compval)));
          else
            internalerror(2014062401);
        end;
      end;


    procedure TLLVMAssember.WriteOrdConst(hp: tai_const);
      var
        consttyp: taiconst_type;
      begin
        if fdecllevel=0 then
          internalerror(2016120203);
        consttyp:=hp.consttype;
        case consttyp of
          aitconst_got,
          aitconst_gotoff_symbol,
          aitconst_uleb128bit,
          aitconst_sleb128bit,
          aitconst_rva_symbol,
          aitconst_secrel32_symbol,
          aitconst_darwin_dwarf_delta32,
          aitconst_darwin_dwarf_delta64,
          aitconst_half16bit,
          aitconst_gs:
            internalerror(2014052901);
          aitconst_128bit,
          aitconst_64bit,
          aitconst_32bit,
          aitconst_16bit,
          aitconst_8bit,
          aitconst_16bit_unaligned,
          aitconst_32bit_unaligned,
          aitconst_64bit_unaligned:
            begin
              if fdecllevel=0 then
                writer.AsmWrite(asminfo^.comment);
              { can't have compile-time differences between symbols; these are
                normally for PIC, but llvm takes care of that for us }
              if assigned(hp.endsym) then
                internalerror(2014052902);
              if assigned(hp.sym) then
                begin
                  writer.AsmWrite(LlvmAsmSymName(hp.sym));
                  { can't have offsets }
                  if hp.value<>0 then
                    if fdecllevel<>0 then
                      internalerror(2014052903)
                    else
                      writer.AsmWrite(' -- symbol offset: ' + tostr(hp.value));
                end
              else if hp.value=0 then
                writer.AsmWrite('zeroinitializer')
              else
                writer.AsmWrite(tostr(hp.value));
{
              // activate in case of debugging IE 2016120203
              if fdecllevel=0 then
                writer.AsmLn;
}
            end;
          else
            internalerror(200704251);
        end;
      end;


    procedure TLLVMAssember.WriteTai(const replaceforbidden: boolean; const do_line, inmetadata: boolean; var InlineLevel: cardinal; var asmblock: boolean; var hp: tai);

      procedure WriteLinkageVibilityFlags(bind: TAsmSymBind; is_definition: boolean);
        begin
          { re-declaration of a symbol defined in the current module (in an
            assembler block) }
          if not is_definition then
            begin
              writer.AsmWrite(' external');
              exit;
            end;
          case bind of
             AB_EXTERNAL,
             AB_EXTERNAL_INDIRECT:
               writer.AsmWrite(' external');
             AB_COMMON:
               writer.AsmWrite(' common');
             AB_LOCAL:
               writer.AsmWrite(' internal');
             AB_GLOBAL,
             AB_INDIRECT:
               ;
             AB_WEAK_EXTERNAL:
               writer.AsmWrite(' extern_weak');
             AB_PRIVATE_EXTERN:
               writer.AsmWrite(' hidden')
             else
               internalerror(2014020104);
           end;
        end;


      procedure WriteFunctionFlags(pd: tprocdef);
        begin
          { function attributes }
          if (pos('FPC_SETJMP',upper(pd.mangledname))<>0) or
             (pd.mangledname=(target_info.cprefix+'setjmp')) then
            writer.AsmWrite(' returns_twice');
          if po_inline in pd.procoptions then
            writer.AsmWrite(' inlinehint');
          if (po_noinline in pd.procoptions) or
             (pio_inline_forbidden in pd.implprocoptions) then
            writer.AsmWrite(' noinline');
          { ensure that functions that happen to have the same name as a
            standard C library function, but which are implemented in Pascal,
            are not considered to have the same semantics as the C function with
            the same name }
          if not(po_external in pd.procoptions) then
            writer.AsmWrite(' nobuiltin');
          if po_noreturn in pd.procoptions then
            writer.AsmWrite(' noreturn');
          if pio_thunk in pd.implprocoptions then
            writer.AsmWrite(' "thunk"');
          if llvmflag_null_pointer_valid in llvmversion_properties[current_settings.llvmversion] then
            writer.AsmWrite(' "null-pointer-is-valid"="true"');
          if not(pio_fastmath in pd.implprocoptions) then
            writer.AsmWrite(' strictfp');
        end;


      procedure WriteTypedConstData(hp: tai_abstracttypedconst; metadata: boolean);
        var
          p: tai_abstracttypedconst;
          pval: tai;
          defstr: TSymStr;
          first, gotstring: boolean;
        begin
          if hp.def<>llvm_metadatatype then
            begin
              defstr:=llvmencodetypename(hp.def)
            end
          else
            begin
              defstr:=''
            end;
          { write the struct, array or simple type }
          case hp.adetyp of
            tck_record:
              begin
                if not(metadata) then
                  begin
                    writer.AsmWrite(defstr);
                    if not(df_llvm_no_struct_packing in hp.def.defoptions) then
                      writer.AsmWrite(' <{')
                    else
                      writer.AsmWrite(' {')
                  end
                else
                  begin
                    writer.AsmWrite(' !{');
                  end;
                first:=true;
                for p in tai_aggregatetypedconst(hp) do
                  begin
                    if not first then
                      writer.AsmWrite(', ')
                    else
                      first:=false;
                    WriteTypedConstData(p,metadata);
                  end;
                if not(metadata) then
                  begin
                    if not(df_llvm_no_struct_packing in hp.def.defoptions) then
                      writer.AsmWrite(' }>')
                    else
                      writer.AsmWrite(' }')
                  end
                else
                  begin
                    writer.AsmWrite(' }');
                  end;
              end;
            tck_array:
              begin
                if not(metadata) then
                  begin
                    writer.AsmWrite(defstr);
                  end;
                first:=true;
                gotstring:=false;
                for p in tai_aggregatetypedconst(hp) do
                  begin
                    if not first then
                      writer.AsmWrite(', ')
                    else
                      begin
                        writer.AsmWrite(' ');
                        if (tai_abstracttypedconst(p).adetyp=tck_simple) and
                           (tai_simpletypedconst(p).val.typ=ait_string) then
                          begin
                            gotstring:=true;
                          end
                        else
                          begin
                            if not metadata then
                              begin
                                writer.AsmWrite('[');
                              end
                            else
                              begin
                                writer.AsmWrite('!{');
                              end;
                          end;
                        first:=false;
                      end;
                    { cannot concat strings and other things }
                    if gotstring and
                       not metadata and
                       ((tai_abstracttypedconst(p).adetyp<>tck_simple) or
                        (tai_simpletypedconst(p).val.typ<>ait_string)) then
                      internalerror(2014062701);
                    WriteTypedConstData(p,metadata);
                  end;
                if not gotstring then
                  begin
                    if not metadata then
                      begin
                        writer.AsmWrite(']');
                      end
                    else
                      begin
                        writer.AsmWrite('}');
                      end;
                  end;
              end;
            tck_simple:
              begin
                pval:=tai_simpletypedconst(hp).val;
                if (pval.typ<>ait_string) and
                   (defstr<>'') then
                  begin
                    writer.AsmWrite(defstr);
                    writer.AsmWrite(' ');
                  end;
                WriteTai(replaceforbidden,do_line,metadata,InlineLevel,asmblock,pval);
              end;
          end;
        end;

      procedure WriteLlvmMetadataNode(hp: tai_llvmbasemetadatanode);
        begin
          { must only appear at the top level }
          if fdecllevel<>0 then
            internalerror(2019050111);
          writer.AsmWrite('!');
          writer.AsmWrite(tai_llvmbasemetadatanode(hp).name);
          writer.AsmWrite(' =');
          inc(fdecllevel);
          WriteTypedConstData(hp,true);
          writer.AsmLn;
          dec(fdecllevel);
        end;

      var
        hp2: tai;
        s: string;
        sstr: TSymStr;
        i: longint;
        ch: ansichar;
      begin
        case hp.typ of
          ait_comment :
            begin
              writer.AsmWrite(asminfo^.comment);
              writer.AsmWritePChar(tai_comment(hp).str);
              if fdecllevel<>0 then
                internalerror(2015090601);
              writer.AsmLn;
            end;

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

          ait_align,
          ait_section :
            begin
              { ignore, specified as part of declarations -- don't write
                comment, because could appear in the middle of an aggregate
                constant definition }
            end;

          ait_datablock :
            begin
              writer.AsmWrite(asminfo^.comment);
              writer.AsmWriteln('datablock');
            end;

          ait_const:
            begin
              WriteOrdConst(tai_const(hp));
            end;

          ait_realconst :
            begin
              WriteRealConst(tai_realconst(hp), do_line);
            end;

          ait_string :
            begin
              if fdecllevel=0 then
                internalerror(2016120201);
              if not inmetadata then
                writer.AsmWrite('c"')
              else
                writer.AsmWrite('!"');
              for i:=1 to tai_string(hp).len do
               begin
                 ch:=tai_string(hp).str[i-1];
                 case ch of
                           #0, {This can't be done by range, because a bug in FPC}
                      #1..#31,
                   #128..#255,
                          '"',
                          '\' : s:='\'+hexStr(ord(ch),2);
                 else
                   s:=ch;
                 end;
                 writer.AsmWrite(s);
               end;
              writer.AsmWrite('"');
            end;

          ait_label :
            begin
              if not asmblock and
                 (tai_label(hp).labsym.is_used) then
                begin
                  if (tai_label(hp).labsym.bind=AB_PRIVATE_EXTERN) then
                    begin
                     { should be emitted as part of the variable/function def }
                     internalerror(2013010703);
                   end;
                 if tai_label(hp).labsym.bind in [AB_GLOBAL, AB_PRIVATE_EXTERN] then
                   begin
                     { should be emitted as part of the variable/function def }
                     //internalerror(2013010704);
                     writer.AsmWriteln(asminfo^.comment+'global/privateextern label: '+tai_label(hp).labsym.name);
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
              if fdecllevel=0 then
                writer.AsmWrite(asminfo^.comment);
              writer.AsmWriteln(LlvmAsmSymName(tai_symbol(hp).sym));
              { todo }
              if tai_symbol(hp).has_value then
                internalerror(2014062402);
            end;
          ait_llvmdecl:
            begin
              if taillvmdecl(hp).def.typ=procdef then
                begin
                  if not(ldf_definition in taillvmdecl(hp).flags) then
                    begin
                      writer.AsmWrite('declare');
                      writer.AsmWrite(llvmencodeproctype(tprocdef(taillvmdecl(hp).def), taillvmdecl(hp).namesym.name, lpd_decl));
                      WriteFunctionFlags(tprocdef(taillvmdecl(hp).def));
                      writer.AsmLn;
                    end
                  else
                    begin
                      writer.AsmWrite('define');
                      if ldf_weak in taillvmdecl(hp).flags then
                        writer.AsmWrite(' weak');
                      WriteLinkageVibilityFlags(taillvmdecl(hp).namesym.bind, true);
                      writer.AsmWrite(llvmencodeproctype(tprocdef(taillvmdecl(hp).def), '', lpd_def));
                      WriteFunctionFlags(tprocdef(taillvmdecl(hp).def));
                      if assigned(tprocdef(taillvmdecl(hp).def).personality) then
                        begin
                          writer.AsmWrite(' personality i8* bitcast (');
                          writer.AsmWrite(llvmencodeproctype(tprocdef(taillvmdecl(hp).def).personality, '', lpd_procvar));
                          writer.AsmWrite('* ');
                          writer.AsmWrite(llvmmangledname(tprocdef(taillvmdecl(hp).def).personality.mangledname));
                          writer.AsmWrite(' to i8*)');
                        end;
                      writer.AsmWriteln(' {');
                    end;
                end
              else
                begin
                  writer.AsmWrite(LlvmAsmSymName(taillvmdecl(hp).namesym));
                  writer.AsmWrite(' =');
                  if ldf_weak in taillvmdecl(hp).flags then
                    writer.AsmWrite(' weak');
                  if ldf_appending in taillvmdecl(hp).flags then
                    writer.AsmWrite(' appending');
                  WriteLinkageVibilityFlags(taillvmdecl(hp).namesym.bind, ldf_definition in taillvmdecl(hp).flags);
                  writer.AsmWrite(' ');
                  if (ldf_tls in taillvmdecl(hp).flags) then
                    writer.AsmWrite('thread_local ');
                  if ldf_unnamed_addr in taillvmdecl(hp).flags then
                    writer.AsmWrite('unnamed_addr ');
                  if taillvmdecl(hp).sec in [sec_rodata,sec_rodata_norel] then
                    writer.AsmWrite('constant ')
                  else
                    writer.AsmWrite('global ');
                  if not assigned(taillvmdecl(hp).initdata) then
                    begin
                      writer.AsmWrite(llvmencodetypename(taillvmdecl(hp).def));
                      if ldf_definition in taillvmdecl(hp).flags then
                        writer.AsmWrite(' zeroinitializer');
                    end
                  else
                    begin
                      inc(fdecllevel);
                      { can't have an external symbol with initialisation data }
                      if taillvmdecl(hp).namesym.bind in [AB_EXTERNAL, AB_WEAK_EXTERNAL] then
                        internalerror(2014052905);
                      { bitcast initialisation data to the type of the constant }
                      { write initialisation data }
                      hp2:=tai(taillvmdecl(hp).initdata.first);
                      while assigned(hp2) do
                        begin
                          WriteTai(replaceforbidden,do_line,inmetadata,InlineLevel,asmblock,hp2);
                          hp2:=tai(hp2.next);
                        end;
                      dec(fdecllevel);
                    end;
                  { custom section name? }
                  case taillvmdecl(hp).sec of
                    sec_user:
                      begin
                        writer.AsmWrite(', section "');
                        writer.AsmWrite(taillvmdecl(hp).secname);
                        writer.AsmWrite('"');
                      end;
                    low(TObjCAsmSectionType)..high(TObjCAsmSectionType):
                      begin
                        writer.AsmWrite(', section "');
                        writer.AsmWrite(objc_section_name(taillvmdecl(hp).sec));
                        writer.AsmWrite('"');
                      end;
                    else
                      ;
                  end;
                  { sections whose name starts with 'llvm.' are for LLVM
                    internal use and don't have an alignment }
                  if pos('llvm.',taillvmdecl(hp).secname)<>1 then
                    begin
                      { alignment }
                      writer.AsmWrite(', align ');
                      writer.AsmWriteln(tostr(taillvmdecl(hp).alignment));
                    end
                  else
                    writer.AsmLn;
                end;
            end;
          ait_llvmalias:
            begin
              writer.AsmWrite(LlvmAsmSymName(taillvmalias(hp).newsym));
              writer.AsmWrite(' = alias ');
              WriteLinkageVibilityFlags(taillvmalias(hp).bind, true);
              if taillvmalias(hp).def.typ=procdef then
                sstr:=llvmencodeproctype(tabstractprocdef(taillvmalias(hp).def), '', lpd_alias)
              else
                sstr:=llvmencodetypename(taillvmalias(hp).def);
              writer.AsmWrite(sstr);
              writer.AsmWrite(', ');
              writer.AsmWrite(sstr);
              writer.AsmWrite('* ');
              writer.AsmWriteln(LlvmAsmSymName(taillvmalias(hp).oldsym));
            end;
          ait_llvmmetadatanode:
            begin
              WriteLlvmMetadataNode(tai_llvmbasemetadatanode(hp));
            end;
          ait_llvmmetadatareftypedconst:
            begin
              { must only appear as an element in a typed const }
              if fdecllevel=0 then
                internalerror(2019050110);
              writer.AsmWrite('!');
              writer.AsmWrite(tai_llvmbasemetadatanode(tai_llvmmetadatareftypedconst(hp).val).name);
            end;
          ait_llvmmetadatarefoperand:
            begin
              { must only appear as an operand }
              if fdecllevel=0 then
                internalerror(2019050110);
              writer.AsmWrite('!');
              writer.AsmWrite(tai_llvmmetadatareferenceoperand(hp).id);
              writer.AsmWrite(' !');
              writer.AsmWrite(tai_llvmmetadatareferenceoperand(hp).value.name);
            end;
          ait_symbolpair:
            begin
              { should be emitted as part of the symbol def }
              internalerror(2013010708);
            end;

          ait_symbol_end :
            begin
              if tai_symbol_end(hp).sym.typ=AT_FUNCTION then
                writer.AsmWriteln('}')
              else
                writer.AsmWriteln('; ait_symbol_end error, should not be generated');
//                internalerror(2013010711);
            end;

          ait_instruction :
            begin
              WriteInstruction(hp);
            end;

          ait_llvmins:
            begin
              WriteLlvmInstruction(hp);
            end;

          ait_stab :
            begin
              internalerror(2013010712);
            end;

          ait_force_line,
          ait_function_name :
            ;

          ait_cutobject :
            begin
            end;

          ait_marker :
            case
              tai_marker(hp).kind of
                mark_NoLineInfoStart:
                  inc(InlineLevel);
                mark_NoLineInfoEnd:
                  dec(InlineLevel);
                { these cannot be nested }
                mark_AsmBlockStart:
                  asmblock:=true;
                mark_AsmBlockEnd:
                  asmblock:=false;
                else
                  ;
              end;

          ait_directive :
            begin
              { CPU directive is commented out for the LLVM }
              if tai_directive(hp).directive=asd_cpu then
                writer.AsmWrite(asminfo^.comment);
              WriteDirectiveName(tai_directive(hp).directive);
              if tai_directive(hp).name <>'' then
                writer.AsmWrite(tai_directive(hp).name);
              if fdecllevel<>0 then
                internalerror(2015090602);
              writer.AsmLn;
            end;

          ait_seh_directive :
            begin
              internalerror(2013010713);
            end;
          ait_varloc:
            begin
              if tai_varloc(hp).newlocationhi<>NR_NO then
                writer.AsmWrite(strpnew('Var '+tai_varloc(hp).varsym.realname+' located in register '+
                  std_regname(tai_varloc(hp).newlocationhi)+':'+std_regname(tai_varloc(hp).newlocation)))
              else
                writer.AsmWrite(strpnew('Var '+tai_varloc(hp).varsym.realname+' located in register '+
                  std_regname(tai_varloc(hp).newlocation)));
              if fdecllevel<>0 then
                internalerror(2015090603);
              writer.AsmLn;
            end;
           ait_typedconst:
             begin
               WriteTypedConstData(tai_abstracttypedconst(hp),false);
             end
          else
            internalerror(2019012010);
        end;
      end;


    constructor TLLVMAssember.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter:=TLLVMInstrWriter.create(self);
      end;


    procedure TLLVMAssember.WriteDirectiveName(dir: TAsmDirective);
      begin
        writer.AsmWrite('.'+directivestr[dir]+' ');
      end;


    procedure TLLVMAssember.WriteAsmList;
      var
        hal : tasmlisttype;
        a: TExternalAssembler;
        decorator: TLLVMModuleInlineAssemblyDecorator;
      begin
        WriteExtraHeader;

        for hal:=low(TasmlistType) to high(TasmlistType) do
          begin
            if not assigned(current_asmdata.asmlists[hal]) or
               current_asmdata.asmlists[hal].Empty then
              continue;
            writer.AsmWriteLn(asminfo^.comment+'Begin asmlist '+AsmlistTypeStr[hal]);
            if not(hal in [al_pure_assembler,al_dwarf_frame]) then
              writetree(current_asmdata.asmlists[hal])
            else
              begin
                { write routines using the target-specific external assembler
                  writer, filtered using the LLVM module-level assembly
                  decorator }
                decorator:=TLLVMModuleInlineAssemblyDecorator.Create;
                writer.decorator:=decorator;
                a:=GetExternalGnuAssemblerWithAsmInfoWriter(asminfo,writer);
                a.WriteTree(current_asmdata.asmlists[hal]);
                writer.decorator:=nil;
                decorator.free;
                a.free;
              end;
            writer.AsmWriteLn(asminfo^.comment+'End asmlist '+AsmlistTypeStr[hal]);
          end;

        writer.AsmLn;
      end;


    procedure TLLVMAssember.WriteFunctionInlineAsmList(list: tasmlist);
      var
        a: TExternalAssembler;
      begin
        if not assigned(ffuncinlasmdecorator) then
          ffuncinlasmdecorator:=TLLVMFunctionInlineAssemblyDecorator.create;
        if assigned(writer.decorator) then
          internalerror(2016110201);
        writer.decorator:=ffuncinlasmdecorator;
        a:=GetExternalGnuAssemblerWithAsmInfoWriter(asminfo,writer);
        a.WriteTree(list);
        a.free;
        writer.decorator:=nil;
      end;



{****************************************************************************}
{                          LLVM Instruction Writer                           }
{****************************************************************************}

     constructor TLLVMInstrWriter.create(_owner: TLLVMAssember);
       begin
         inherited create;
         owner := _owner;
       end;


{****************************************************************************}
{                               llc Assember                                 }
{****************************************************************************}

     function TLLVMLLCAssember.MakeCmdLine: TCmdStr;
       var
         optstr: TCmdStr;
       begin
         result:=inherited;
         { standard optimization flags for llc -- todo: this needs to be split
           into a call to opt and one to llc }
         if cs_opt_level3 in current_settings.optimizerswitches then
           optstr:='-O3'
         else if cs_opt_level2 in current_settings.optimizerswitches then
           optstr:='-O2'
         else if cs_opt_level1 in current_settings.optimizerswitches then
           optstr:='-O1'
         else
           optstr:='-O0';
         { stack frame elimination }
         if not(cs_opt_stackframe in current_settings.optimizerswitches) then
           optstr:=optstr+' -disable-fp-elim';
         { fast math }
         if cs_opt_fastmath in current_settings.optimizerswitches then
           optstr:=optstr+' -enable-unsafe-fp-math -fp-contract=fast';  { -enable-fp-mad support depends on version }
         { smart linking }
         if cs_create_smart in current_settings.moduleswitches then
           optstr:=optstr+' -data-sections -function-sections';
         { pic }
         if cs_create_pic in current_settings.moduleswitches then
           optstr:=optstr+' -relocation-model=pic'
         else if not(target_info.system in systems_darwin) then
           optstr:=optstr+' -relocation-model=static'
         else
           optstr:=optstr+' -relocation-model=dynamic-no-pic';
         { force object output instead of textual assembler code }
         optstr:=optstr+' -filetype=obj';
         if fputypestrllvm[current_settings.fputype]<>'' then
           optstr:=optstr+' -mattr=+'+fputypestrllvm[current_settings.fputype];
         replace(result,'$OPT',optstr);
       end;


{****************************************************************************}
{                               clang Assember                               }
{****************************************************************************}

    function TLLVMClangAssember.MakeCmdLine: TCmdStr;
      var
        wpostr,
        optstr: TCmdStr;
      begin
        wpostr:='';
        if cs_lto in current_settings.moduleswitches then
          begin
            case fnextpass of
              0:
                begin
                  ObjFileName:=ChangeFileExt(ObjFileName,'.bc');
                  wpostr:=' -flto';
                end;
              1:
                begin
                  ObjFileName:=ChangeFileExt(ObjFileName,'.o');
                end;
            end;
          end;
        result:=inherited;
        { standard optimization flags for llc -- todo: this needs to be split
          into a call to opt and one to llc }
        if cs_opt_level3 in current_settings.optimizerswitches then
          optstr:='-O3'
        else if cs_opt_level2 in current_settings.optimizerswitches then
          optstr:='-O2'
        else if cs_opt_level1 in current_settings.optimizerswitches then
          optstr:='-O1'
        else
          optstr:='-O0';
        optstr:=optstr+wpostr;
        { stack frame elimination }
        if not(cs_opt_stackframe in current_settings.optimizerswitches) then
          optstr:=optstr+' -fno-omit-frame-pointer'
        else
          optstr:=optstr+' -fomit-frame-pointer';
        { fast math }
        if cs_opt_fastmath in current_settings.optimizerswitches then
          optstr:=optstr+' -ffast-math';
        { smart linking }
        if cs_create_smart in current_settings.moduleswitches then
          optstr:=optstr+' -fdata-sections -ffunction-sections';
        { pic }
        if cs_create_pic in current_settings.moduleswitches then
          optstr:=optstr+' -fpic'
        else if not(target_info.system in systems_darwin) then
          optstr:=optstr+' -static'
        else
          optstr:=optstr+' -mdynamic-no-pic';
        if not(target_info.system in systems_darwin) then
          begin
            optstr:=optstr+' --target='+llvm_target_name;
          end;

        if fputypestrllvm[current_settings.fputype]<>'' then
          optstr:=optstr+' -m'+fputypestrllvm[current_settings.fputype];

        replace(result,'$OPT',optstr);
        inc(fnextpass);
      end;


    function TLLVMClangAssember.DoAssemble: boolean;
      begin
        fnextpass:=0;
        result:=inherited;
      end;


    function TLLVMClangAssember.RerunAssembler: boolean;
      begin
        result:=
          (cs_lto in current_settings.moduleswitches) and
          (fnextpass<=1);
      end;


    function TLLVMClangAssember.DoPipe: boolean;
      begin
        result:=
          not(cs_lto in current_settings.moduleswitches) and
          inherited;
      end;


   const
     as_llvm_llc_info : tasminfo =
        (
          id     : as_llvm_llc;

          idtxt  : 'LLVM-LLC';
          asmbin : 'llc';
          asmcmd: '$OPT -o $OBJ $ASM';
          supported_targets : [system_x86_64_linux,system_x86_64_darwin,system_aarch64_linux,system_arm_linux];
          flags : [af_smartlink_sections];
          labelprefix : 'L';
          comment : '; ';
          dollarsign: '$';
        );

     as_llvm_clang_info : tasminfo =
        (
          id     : as_llvm_clang;

          idtxt  : 'LLVM-CLANG';
          asmbin : 'clang';
          asmcmd: '$OPT $DARWINVERSION -c -o $OBJ $ASM';
          supported_targets : [system_x86_64_linux,system_x86_64_darwin,system_aarch64_linux,system_arm_linux];
          flags : [af_smartlink_sections];
          labelprefix : 'L';
          comment : '; ';
          dollarsign: '$';
        );

begin
  RegisterAssembler(as_llvm_llc_info,TLLVMLLCAssember);
  RegisterAssembler(as_llvm_clang_info,TLLVMClangAssember);
end.
