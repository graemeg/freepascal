{
    Copyright (c) 2017 by Karoly Balogh

    This unit implements the Binaryen assembler writer

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
unit agbinaryen;

{$i fpcdefs.inc}

interface

    uses
      cclasses,systems,
      globtype,globals,
      symconst,symbase,symdef,symsym,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      assemble;

    type
      TBinaryenAssemblerOutputFile=class(TExternalAssemblerOutputFile)
        procedure RemoveAsm; override;
      end;

      TBinaryenInstrWriter = class;
      {# This is a derived class which is used to write
         Binaryen-styled assembler.
      }

      { TBinaryenAssembler }

      TBinaryenAssembler=class(texternalassembler)
       protected
        jasminjar: tcmdstr;
        asmfiles: TCmdStrList;

        procedure WriteExtraHeader(obj: tabstractrecorddef);
        procedure WriteInstruction(hp: tai);

        function CreateNewAsmWriter: TExternalAssemblerOutputFile; override;
       public
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
        procedure WriteTree(p:TAsmList);override;
        procedure WriteAsmList;override;
        destructor destroy; override;
       protected
        InstrWriter: TBinaryenInstrWriter;
      end;


      {# This is the base class for writing instructions.

         The WriteInstruction() method must be overridden
         to write a single instruction to the assembler
         file.
      }

      { TBinaryenInstrWriter }

      TBinaryenInstrWriter = class
        constructor create(_owner: TBinaryenAssembler);
        procedure WriteInstruction(hp : tai); virtual;
       protected
        owner: TBinaryenAssembler;
      end;


implementation

    uses
      SysUtils,
      cutils,cfileutl,cscript,
      fmodule,finput,verbose,
      symtype,symcpu,symtable,
      itcpuwasm,cpubase,cpuinfo,cgutils,
      widestr
      ;

    const
      line_length = 70;

    type
      t64bitarray = array[0..7] of byte;
      t32bitarray = array[0..3] of byte;

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


   function constastr(p: pchar; len: longint): ansistring;
     var
       i,runstart,runlen: longint;

       procedure flush;
         begin
           if runlen>0 then
             begin
               setlength(result,length(result)+runlen);
               move(p[runstart],result[length(result)-runlen+1],runlen);
               runlen:=0;
             end;
         end;

     begin
       result:='"';
       runlen:=0;
       runstart:=0;
       for i:=0 to len-1 do
         begin
           { escape control codes }
           case p[i] of
             { LF and CR must be escaped specially, because \uXXXX parsing
               happens in the pre-processor, so it's the same as actually
               inserting a newline in the middle of a string constant }
             #10:
               begin
                 flush;
                 result:=result+'\n';
               end;
             #13:
               begin
                 flush;
                 result:=result+'\r';
               end;
             '"','\':
               begin
                 flush;
                 result:=result+'\'+p[i];
               end
             else if p[i]<#32 then
               begin
                 flush;
                 result:=result+'\u'+hexstr(ord(p[i]),4);
               end
             else if p[i]<#127 then
               begin
                 if runlen=0 then
                   runstart:=i;
                 inc(runlen);
               end
             else
               begin
                 { see comments in njvmcon }
                 flush;
                 result:=result+'\u'+hexstr(ord(p[i]),4)
               end;
           end;
         end;
       flush;
       result:=result+'"';
     end;



{****************************************************************************}
{                       Binaryen Output File                                   }
{****************************************************************************}

    procedure TBinaryenAssemblerOutputFile.RemoveAsm;
      var
        g : file;
      begin
        inherited;
        if cs_asm_leave in current_settings.globalswitches then
         exit;
        while not TBinaryenAssembler(owner).asmfiles.empty do
          begin
            if cs_asm_extern in current_settings.globalswitches then
             AsmRes.AddDeleteCommand(TBinaryenAssembler(owner).asmfiles.GetFirst)
            else
             begin
               assign(g,TBinaryenAssembler(owner).asmfiles.GetFirst);
               {$I-}
                erase(g);
               {$I+}
               if ioresult<>0 then;
             end;
          end;
      end;


{****************************************************************************}
{                       Binaryen Assembler writer                              }
{****************************************************************************}

    destructor TBinaryenAssembler.Destroy;
      begin
        InstrWriter.free;
        asmfiles.free;
        inherited destroy;
      end;


    procedure TBinaryenAssembler.WriteTree(p:TAsmList);
      var
        ch       : char;
        hp       : tai;
        hp1      : tailineinfo;
        s        : ansistring;
        i,pos    : longint;
        InlineLevel : longint;
        do_line  : boolean;
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
                       writer.AsmWriteLn(asminfo^.comment+'Temp '+tostr(tai_tempalloc(hp).temppos)+','+
                         tostr(tai_tempalloc(hp).tempsize)+' '+tempallocstr[tai_tempalloc(hp).allocation]);
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
                 if (tai_label(hp).labsym.is_used) then
                  begin
                    writer.AsmWrite(tai_label(hp).labsym.name);
                    writer.AsmWriteLn(':');
                  end;
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
                 if tai_directive(hp).directive=asd_cpu then
                   writer.AsmWrite(asminfo^.comment);
                 writer.AsmWrite('.'+directivestr[tai_directive(hp).directive]+' ');
                 if tai_directive(hp).name<>'' then
                   writer.AsmWrite(tai_directive(hp).name);
                 writer.AsmLn;
               end;

             else
               internalerror(2010122707);
           end;
           hp:=tai(hp.next);
         end;
      end;


    procedure TBinaryenAssembler.WriteExtraHeader(obj: tabstractrecorddef);
      begin
      end;


    procedure TBinaryenAssembler.WriteInstruction(hp: tai);
      begin
        InstrWriter.WriteInstruction(hp);
      end;


    function TBinaryenAssembler.CreateNewAsmWriter: TExternalAssemblerOutputFile;
      begin
        Result:=TBinaryenAssemblerOutputFile.Create(self);
      end;


    constructor TBinaryenAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter:=TBinaryenInstrWriter.Create(self);
        asmfiles:=TCmdStrList.Create;
      end;


    procedure TBinaryenAssembler.WriteAsmList;
      var
        hal : tasmlisttype;
      begin
        writer.MarkEmpty;
        WriteExtraHeader(nil);

      for hal:=low(TasmlistType) to high(TasmlistType) do
        begin
          if not (current_asmdata.asmlists[hal].empty) then
            begin
              writer.AsmWriteLn(asminfo^.comment+'Begin asmlist '+AsmlistTypeStr[hal]);
              writetree(current_asmdata.asmlists[hal]);
              writer.AsmWriteLn(asminfo^.comment+'End asmlist '+AsmlistTypeStr[hal]);
            end;
        end;

        writer.AsmLn;
      end;


{****************************************************************************}
{                         Binaryen Instruction Writer                          }
{****************************************************************************}

     constructor TBinaryenInstrWriter.create(_owner: TBinaryenAssembler);
       begin
         inherited create;
         owner := _owner;
       end;

    function getreferencestring(var ref : treference) : ansistring;
      begin
{        if (ref.arrayreftype<>art_none) or
           (ref.index<>NR_NO) then
          internalerror(2010122809);}
        if assigned(ref.symbol) then
          begin
            // global symbol or field -> full type and name
            // ref.base can be <> NR_NO in case an instance field is loaded.
            // This register is not part of this instruction, it will have
            // been placed on the stack by the previous one.
            if (ref.offset<>0) then
              internalerror(2010122811);
            result:=ref.symbol.name;
          end
        else
          begin
            // local symbol -> stack slot, stored in offset
            if ref.base<>NR_STACK_POINTER_REG then
              internalerror(2010122810);
            result:=tostr(ref.offset);
          end;
      end;


    function getopstr(const o:toper) : ansistring;
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
          else
            internalerror(2010122802);
        end;
      end;


    procedure TBinaryenInstrWriter.WriteInstruction(hp: tai);
      var
        s: ansistring;
        i: byte;
        sep: ansistring;
      begin
        s:=#9+wasm_op2str[taicpu(hp).opcode];
        if taicpu(hp).ops<>0 then
          begin
            sep:=#9;
            for i:=0 to taicpu(hp).ops-1 do
              begin
                 s:=s+sep+getopstr(taicpu(hp).oper[i]^);
                 sep:=' ';
              end;
          end;
        owner.writer.AsmWriteLn(s);
      end;

{****************************************************************************}
{                         Binaryen Instruction Writer                        }
{****************************************************************************}

  const
    as_wasm_binaryen_info : tasminfo =
       (
         id     : as_wasm32_binaryen;
         idtxt  : 'BINARYEN';
         asmbin : 'wasm-as';
         asmcmd : '$ASM $EXTRAOPT';
         supported_targets : [system_wasm32_embedded,system_wasm32_wasi];
         flags : [];
         labelprefix : 'L';
         labelmaxlen : -1;
         comment : ';; ';
         dollarsign : '$';
       );


initialization
  RegisterAssembler(as_wasm_binaryen_info,TBinaryenAssembler);
end.
