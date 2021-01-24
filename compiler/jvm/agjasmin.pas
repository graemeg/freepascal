{
    Copyright (c) 1998-2010 by the Free Pascal team

    This unit implements the Jasmin assembler writer

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
{ Unit for writing Jasmin assembler (JVM bytecode) output.
}
unit agjasmin;

{$i fpcdefs.inc}

interface

    uses
      cclasses,systems,
      globtype,globals,
      symconst,symbase,symdef,symsym,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      assemble;

    type
      TJasminAssemblerOutputFile=class(TExternalAssemblerOutputFile)
        procedure RemoveAsm; override;
      end;

      TJasminInstrWriter = class;
      {# This is a derived class which is used to write
         Jasmin-styled assembler.
      }

      { TJasminAssembler }

      TJasminAssembler=class(texternalassembler)
       protected
        jasminjar: tcmdstr;
        asmfiles: TCmdStrList;

        procedure WriteExtraHeader(obj: tabstractrecorddef);
        procedure WriteInstruction(hp: tai);
        procedure NewAsmFileForStructDef(obj: tabstractrecorddef);

        function VisibilityToStr(vis: tvisibility): ansistring;
        function MethodDefinition(pd: tprocdef): ansistring;
        function ConstValue(csym: tconstsym): ansistring;
        function ConstAssignmentValue(csym: tconstsym): ansistring;
        function ConstDefinition(sym: tconstsym): ansistring;
        function FieldDefinition(sym: tabstractvarsym): ansistring;
        function InnerStructDef(obj: tabstractrecorddef): ansistring;

        procedure WriteProcDef(pd: tprocdef);
        procedure WriteFieldSym(sym: tabstractvarsym);
        procedure WriteConstSym(sym: tconstsym);
        procedure WriteSymtableVarSyms(st: TSymtable);
        procedure WriteSymtableProcdefs(st: TSymtable);
        procedure WriteSymtableStructDefs(st: TSymtable);

        function CreateNewAsmWriter: TExternalAssemblerOutputFile; override;
       public
        constructor CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean); override;
        function MakeCmdLine: TCmdStr;override;
        procedure WriteTree(p:TAsmList);override;
        procedure WriteAsmList;override;
        destructor destroy; override;
       protected
        InstrWriter: TJasminInstrWriter;
      end;


      {# This is the base class for writing instructions.

         The WriteInstruction() method must be overridden
         to write a single instruction to the assembler
         file.
      }

      { TJasminInstrWriter }

      TJasminInstrWriter = class
        constructor create(_owner: TJasminAssembler);
        procedure WriteInstruction(hp : tai); virtual;
       protected
        owner: TJasminAssembler;
      end;


implementation

    uses
      SysUtils,
      cutils,cfileutl,cscript,
      fmodule,finput,verbose,
      symtype,symcpu,symtable,jvmdef,
      itcpujas,cpubase,cpuinfo,cgutils,
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


   function constwstr(w: pcompilerwidechar; len: longint): ansistring;
     var
       i: longint;
     begin
       result:='"';
       for i:=0 to len-1 do
         begin
           { escape control codes }
           case w[i] of
             10:
               result:=result+'\n';
             13:
               result:=result+'\r';
             ord('"'),ord('\'):
               result:=result+'\'+chr(w[i]);
             else if (w[i]<32) or
                (w[i]>=127) then
               result:=result+'\u'+hexstr(w[i],4)
             else
               result:=result+char(w[i]);
           end;
         end;
       result:=result+'"';
     end;


   function constsingle(s: single): ansistring;
     begin
       result:='0fx'+hexstr(longint(t32bitarray(s)),8);
     end;


   function constdouble(d: double): ansistring;
      begin
        // force interpretation as double (since we write it out as an
        // integer, we never have to swap the endianess). We have to
        // include the sign separately because of the way Java parses
        // hex numbers (0x8000000000000000 is not a valid long)
       result:=hexstr(abs(int64(t64bitarray(d))),16);
       if int64(t64bitarray(d))<0 then
         result:='-'+result;
       result:='0dx'+result;
      end;


{****************************************************************************}
{                       Jasmin Output File                                   }
{****************************************************************************}

    procedure TJasminAssemblerOutputFile.RemoveAsm;
      var
        g : file;
      begin
        inherited;
        if cs_asm_leave in current_settings.globalswitches then
         exit;
        while not TJasminAssembler(owner).asmfiles.empty do
          begin
            if cs_asm_extern in current_settings.globalswitches then
             AsmRes.AddDeleteCommand(TJasminAssembler(owner).asmfiles.GetFirst)
            else
             begin
               assign(g,TJasminAssembler(owner).asmfiles.GetFirst);
               {$I-}
                erase(g);
               {$I+}
               if ioresult<>0 then;
             end;
          end;
      end;


{****************************************************************************}
{                       Jasmin Assembler writer                              }
{****************************************************************************}

    destructor TJasminAssembler.Destroy;
      begin
        InstrWriter.free;
        asmfiles.free;
        inherited destroy;
      end;


    procedure TJasminAssembler.WriteTree(p:TAsmList);
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

             ait_align :
               begin

               end;

             ait_section :
               begin

               end;

             ait_datablock :
               begin
                 internalerror(2010122701);
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

             ait_jvar:
               begin
                 writer.AsmWrite('.var ');
                 writer.AsmWrite(tostr(tai_jvar(hp).stackslot));
                 writer.AsmWrite(' is ');
                 writer.AsmWrite(tai_jvar(hp).desc^);
                 writer.AsmWrite(' from ');
                 writer.AsmWrite(tai_jvar(hp).startlab.name);
                 writer.AsmWrite(' to ');
                 writer.AsmWriteLn(tai_jvar(hp).stoplab.name);
               end;

             ait_jcatch:
               begin
                 writer.AsmWrite('.catch ');
                 writer.AsmWrite(tai_jcatch(hp).name^);
                 writer.AsmWrite(' from ');
                 writer.AsmWrite(tai_jcatch(hp).startlab.name);
                 writer.AsmWrite(' to ');
                 writer.AsmWrite(tai_jcatch(hp).stoplab.name);
                 writer.AsmWrite(' using ');
                 writer.AsmWriteLn(tai_jcatch(hp).handlerlab.name);
               end;
             else
               if not WriteComments(hp) then
                 internalerror(2010122707);
           end;
           hp:=tai(hp.next);
         end;
      end;


    procedure TJasminAssembler.WriteExtraHeader(obj: tabstractrecorddef);
      var
        superclass,
        intf: tobjectdef;
        n: ansistring;
        i: longint;
        toplevelowner: tsymtable;
      begin
        superclass:=nil;

        { JVM 1.5+ }
        writer.AsmWriteLn('.bytecode 49.0');
        // include files are not support by Java, and the directory of the main
        // source file must not be specified
        if current_module.mainsource<>'' then
          n:=ExtractFileName(current_module.mainsource)
        else
          n:=InputFileName;
        writer.AsmWriteLn('.source '+ExtractFileName(n));

        { class/interface name }
        if not assigned(obj) then
          begin
            { fake class type for unit -> name=unitname and
              superclass=java.lang.object, make final so you cannot descend
              from it }
            writer.AsmWrite('.class final public ');
            if assigned(current_module.namespace) then
              writer.AsmWrite(current_module.namespace^+'.');
            writer.AsmWriteln(current_module.realmodulename^);
            writer.AsmWriteLn('.super java/lang/Object');
          end
        else
          begin
            toplevelowner:=obj.owner;
            while not(toplevelowner.symtabletype in [staticsymtable,globalsymtable]) do
              toplevelowner:=toplevelowner.defowner.owner;
            case obj.typ of
              recorddef:
                begin
                  { can't inherit from records }
                  writer.AsmWrite('.class final ');
                  if toplevelowner.symtabletype=globalsymtable then
                    writer.AsmWrite('public ');
                  writer.AsmWriteln(obj.jvm_full_typename(true));
                  superclass:=java_fpcbaserecordtype;
                end;
              objectdef:
                begin
                  case tobjectdef(obj).objecttype of
                    odt_javaclass:
                      begin
                        writer.AsmWrite('.class ');
                        if oo_is_sealed in tobjectdef(obj).objectoptions then
                          writer.AsmWrite('final ');
                        if (oo_is_abstract in tobjectdef(obj).objectoptions) or
                           (tobjectdef(obj).abstractcnt<>0) then
                          writer.AsmWrite('abstract ');
                        if toplevelowner.symtabletype=globalsymtable then
                          writer.AsmWrite('public ');
                        if (oo_is_enum_class in tobjectdef(obj).objectoptions) then
                          writer.AsmWrite('enum ');
                        writer.AsmWriteln(obj.jvm_full_typename(true));
                        superclass:=tobjectdef(obj).childof;
                      end;
                    odt_interfacejava:
                      begin
                        writer.AsmWrite('.interface abstract ');
                        if toplevelowner.symtabletype=globalsymtable then
                          writer.AsmWrite('public ');
                        writer.AsmWriteLn(obj.jvm_full_typename(true));
                        { interfaces must always specify Java.lang.object as
                          superclass }
                        superclass:=java_jlobject;
                      end
                    else
                      internalerror(2011010906);
                  end;
                end;
              else
                ;
            end;
            { superclass }
            if assigned(superclass) then
              begin
                writer.AsmWrite('.super ');
                if assigned(superclass.import_lib) then
                  writer.AsmWrite(superclass.import_lib^+'/');
                writer.AsmWriteln(superclass.objextname^);
              end;
            { implemented interfaces }
            if (obj.typ=objectdef) and
               assigned(tobjectdef(obj).ImplementedInterfaces) then
              begin
                for i:=0 to tobjectdef(obj).ImplementedInterfaces.count-1 do
                  begin
                    intf:=TImplementedInterface(tobjectdef(obj).ImplementedInterfaces[i]).IntfDef;
                    writer.AsmWrite('.implements ');
                    writer.AsmWriteLn(intf.jvm_full_typename(true));
                  end;
              end;
            { signature for enum classes (must come after superclass and
              implemented interfaces) }
            if (obj.typ=objectdef) and
               (oo_is_enum_class in tobjectdef(obj).objectoptions) then
              writer.AsmWriteln('.signature "Ljava/lang/Enum<L'+obj.jvm_full_typename(true)+';>;"');
            { in case of nested class: relation to parent class }
            if obj.owner.symtabletype in [objectsymtable,recordsymtable] then
              writer.AsmWriteln(InnerStructDef(obj));
            { add all nested classes }
            for i:=0 to obj.symtable.deflist.count-1 do
              if (is_java_class_or_interface(tdef(obj.symtable.deflist[i])) or
                  (tdef(obj.symtable.deflist[i]).typ=recorddef)) and
                 not(df_generic in tdef(obj.symtable.deflist[i]).defoptions) then
                writer.AsmWriteln(InnerStructDef(tabstractrecorddef(obj.symtable.deflist[i])));
          end;
        writer.AsmLn;
      end;


    procedure TJasminAssembler.WriteInstruction(hp: tai);
      begin
        InstrWriter.WriteInstruction(hp);
      end;


   function TJasminAssembler.MakeCmdLine: TCmdStr;
     const
       jasminjarname = 'jasmin.jar';
     var
       filenames: tcmdstr;
       asmfile: tcmdstrlistitem;
       jasminjarfound: boolean;
     begin
       if jasminjar='' then
         begin
           jasminjarfound:=false;
           if utilsdirectory<>'' then
             jasminjarfound:=FindFile(jasminjarname,utilsdirectory,false,jasminjar);
           if not jasminjarfound then
             jasminjarfound:=FindFileInExeLocations(jasminjarname,false,jasminjar);
           if (not jasminjarfound) and not(cs_asm_extern in current_settings.globalswitches) then
             begin
               Message1(exec_e_assembler_not_found,jasminjarname);
               current_settings.globalswitches:=current_settings.globalswitches+[cs_asm_extern];
             end;
           if jasminjarfound then
             Message1(exec_t_using_assembler,jasminjar);
         end;
       result:=asminfo^.asmcmd;
       filenames:=ScriptFixFileName(AsmFileName);
       if cs_asm_extern in current_settings.globalswitches then
         filenames:=maybequoted(filenames);
       asmfile:=tcmdstrlistitem(asmfiles.First);
       while assigned(asmfile) do
         begin
           if cs_asm_extern in current_settings.globalswitches then
             filenames:=filenames+' '+maybequoted(ScriptFixFileName(asmfile.str))
           else
            filenames:=filenames+' '+ScriptFixFileName(asmfile.str);
           asmfile:=tcmdstrlistitem(asmfile.next);
        end;
       Replace(result,'$ASM',filenames);
       if (path<>'') then
         if cs_asm_extern in current_settings.globalswitches then
           Replace(result,'$OBJDIR',maybequoted(ScriptFixFileName(path)))
         else
           Replace(result,'$OBJDIR',ScriptFixFileName(path))
       else
         Replace(result,'$OBJDIR','.');
       if cs_asm_extern in current_settings.globalswitches then
         Replace(result,'$JASMINJAR',maybequoted(ScriptFixFileName(jasminjar)))
       else
         Replace(result,'$JASMINJAR',ScriptFixFileName(jasminjar));
       Replace(result,'$EXTRAOPT',asmextraopt);
     end;


   procedure TJasminAssembler.NewAsmFileForStructDef(obj: tabstractrecorddef);
      begin
        if not writer.ClearIfEmpty then
          begin
            writer.AsmClose;
            asmfiles.Concat(AsmFileName);
          end;

        AsmFileName:=obj.jvm_full_typename(false);
        AsmFileName:=Path+FixFileName(AsmFileName)+target_info.asmext;
        writer.AsmCreate(cut_normal);
      end;


    function TJasminAssembler.VisibilityToStr(vis: tvisibility): ansistring;
      begin
        case vis of
          vis_hidden,
          vis_strictprivate:
            result:='private ';
          { protected in Java means "accessible by subclasses *and* by classes
            in the same package" -> similar to regular "protected" in Pascal;
            "strict protected" is actually more strict in Pascal than in Java,
            but there's not much we can do about that }
          vis_protected,
          vis_strictprotected:
            result:='protected ';
          vis_private:
            { pick default visibility = "package" visibility; required because
              other classes in the same unit can also access these symbols }
            result:='';
          vis_public:
            result:='public '
          else
            internalerror(2010122609);
        end;
      end;


    function TJasminAssembler.MethodDefinition(pd: tprocdef): ansistring;
      begin
        result:=VisibilityToStr(pd.visibility);
        if (pd.procsym.owner.symtabletype in [globalsymtable,staticsymtable,localsymtable]) or
           (po_classmethod in pd.procoptions) then
          result:=result+'static ';
        if (po_abstractmethod in pd.procoptions) or
           is_javainterface(tdef(pd.owner.defowner)) then
          result:=result+'abstract ';
        if (pd.procsym.owner.symtabletype in [globalsymtable,staticsymtable,localsymtable]) or
           (po_finalmethod in pd.procoptions) or
           (not(po_virtualmethod in pd.procoptions) and
            not(po_classmethod in pd.procoptions) and
            not(pd.proctypeoption in [potype_constructor,potype_class_constructor])) then
          result:=result+'final ';
        result:=result+tcpuprocdef(pd).jvmmangledbasename(false);
      end;


    function TJasminAssembler.ConstValue(csym: tconstsym): ansistring;
      begin
        case csym.consttyp of
          constord:
            { always interpret as signed value, because the JVM does not
              support unsigned values }
            case csym.constdef.size of
              1:result:=tostr(shortint(csym.value.valueord.svalue));
              2:result:=tostr(smallint(csym.value.valueord.svalue));
              4:result:=tostr(longint(csym.value.valueord.svalue));
              8:result:=tostr(csym.value.valueord.svalue);
              else
                internalerror(2014082050);
            end;
          conststring:
            result:=constastr(pchar(csym.value.valueptr),csym.value.len);
          constreal:
            case tfloatdef(csym.constdef).floattype of
              s32real:
                result:=constsingle(pbestreal(csym.value.valueptr)^);
              s64real:
                result:=constdouble(pbestreal(csym.value.valueptr)^);
              else
                internalerror(2011021204);
              end;
          constset:
            result:='TODO: add support for constant sets';
          constpointer:
            { can only be null, but that's the default value and should not
              be written; there's no primitive type that can hold nill }
            internalerror(2011021201);
          constnil:
            internalerror(2011021202);
          constresourcestring:
            result:='TODO: add support for constant resource strings';
          constwstring:
            result:=constwstr(pcompilerwidestring(csym.value.valueptr)^.data,pcompilerwidestring(csym.value.valueptr)^.len);
          constguid:
            result:='TODO: add support for constant guids';
          else
            internalerror(2011021205);
        end;
      end;


    function TJasminAssembler.ConstAssignmentValue(csym: tconstsym): ansistring;
      begin
        result:='';
        { nil is the default value -> don't write explicitly }
        case csym.consttyp of
          constpointer:
            begin
              if csym.value.valueordptr<>0 then
                internalerror(2011021206);
            end;
          constnil:
            ;
        else
          begin
            { enums and sets are initialized as typed constants }
            if not assigned(csym.constdef) or
               not(csym.constdef.typ in [enumdef,setdef]) then
              result:=' = '+ConstValue(csym);
          end;
        end;
      end;


    function TJasminAssembler.ConstDefinition(sym: tconstsym): ansistring;
      begin
        result:=VisibilityToStr(sym.visibility);
        { formal constants are always class-level, not instance-level }
        result:=result+'static final ';
        if sp_internal in sym.symoptions then
          result:=result+'synthetic ';
        result:=result+jvmmangledbasename(sym,true);
        result:=result+ConstAssignmentValue(tconstsym(sym));
      end;


    function TJasminAssembler.FieldDefinition(sym: tabstractvarsym): ansistring;
      begin
        case sym.typ of
          staticvarsym:
            begin
              if sym.owner.symtabletype=globalsymtable then
                result:='public '
              else
                { package visbility }
                result:='';
            end;
          fieldvarsym,
          absolutevarsym:
            result:=VisibilityToStr(tstoredsym(sym).visibility);
          else
            internalerror(2011011204);
        end;
        if (sym.typ=staticvarsym) or
           (sp_static in sym.symoptions) then
          result:=result+'static ';
        if sym.varspez in [vs_const,vs_final] then
          result:=result+'final ';
        if sp_internal in sym.symoptions then
          result:=result+'synthetic ';
        { mark the class fields of enum classes that contain the initialised
          enum instances as "enum" (recognise them by the fact that their type
          is the same as their parent class, and that this parent class is
          marked as oo_is_enum_class) }
        if assigned(sym.owner.defowner) and
           (tdef(sym.owner.defowner).typ=objectdef) and
           (oo_is_enum_class in tobjectdef(sym.owner.defowner).objectoptions) and
           (sym.typ=staticvarsym) and
           (tstaticvarsym(sym).vardef=tdef(sym.owner.defowner)) then
          result:=result+'enum ';
        result:=result+jvmmangledbasename(sym,true);
      end;


    function TJasminAssembler.InnerStructDef(obj: tabstractrecorddef): ansistring;
      var
        extname: pshortstring;
        kindname: ansistring;
      begin
        if not(obj.owner.defowner.typ in [objectdef,recorddef]) then
          internalerror(2011021704);
        { Nested classes in the Pascal sense are equivalent to "static"
          inner classes in Java -- will be changed when support for
          Java-style non-static classes is added }
        case obj.typ of
          recorddef:
            begin
              kindname:='class static ';
              extname:=obj.symtable.realname;
            end;
          objectdef:
            begin
              extname:=tobjectdef(obj).objextname;
              case tobjectdef(obj).objecttype of
                odt_javaclass:
                  kindname:='class static ';
                odt_interfacejava:
                  kindname:='interface static abstract ';
                else
                  internalerror(2011021702);
              end;
            end;
          else
            internalerror(2011032809);
        end;
        result:=
          '.inner '+
          kindname+
          VisibilityToStr(obj.typesym.visibility)+
         extname^+
         ' inner '+
         obj.jvm_full_typename(true)+
         ' outer '+
         tabstractrecorddef(obj.owner.defowner).jvm_full_typename(true);
      end;


    procedure TJasminAssembler.WriteProcDef(pd: tprocdef);
      begin
        if not assigned(tcpuprocdef(pd).exprasmlist) and
           not(po_abstractmethod in pd.procoptions) and
           (not is_javainterface(pd.struct) or
            (pd.proctypeoption in [potype_unitinit,potype_unitfinalize])) then
          exit;
        writer.AsmWrite('.method ');
        writer.AsmWriteln(MethodDefinition(pd));
        if jvmtypeneedssignature(pd) then
          begin
            writer.AsmWrite('.signature "');
            writer.AsmWrite(tcpuprocdef(pd).jvmmangledbasename(true));
            writer.AsmWriteln('"');
          end;
        WriteTree(tcpuprocdef(pd).exprasmlist);
        writer.AsmWriteln('.end method');
        writer.AsmLn;
      end;


    procedure TJasminAssembler.WriteFieldSym(sym: tabstractvarsym);
      begin
        { internal static field definition alias -> skip }
        if (sym.owner.symtabletype in [recordsymtable,ObjectSymtable]) and
           (sym.typ=staticvarsym) then
          exit;
        { external or threadvar definition -> no definition here }
        if ([vo_is_external,vo_is_thread_var]*sym.varoptions)<>[] then
          exit;
        writer.AsmWrite('.field ');
        writer.AsmWriteln(FieldDefinition(sym));
      end;


    procedure TJasminAssembler.WriteConstSym(sym: tconstsym);
      begin
        writer.AsmWrite('.field ');
        writer.AsmWriteln(ConstDefinition(sym));
      end;


    procedure TJasminAssembler.WriteSymtableVarSyms(st: TSymtable);
      var
        sym : tsym;
        i,j : longint;
      begin
        if not assigned(st) then
          exit;
        for i:=0 to st.SymList.Count-1 do
         begin
           sym:=tsym(st.SymList[i]);
           case sym.typ of
             staticvarsym,
             fieldvarsym:
               begin
                 WriteFieldSym(tabstractvarsym(sym));
                 if (sym.typ=staticvarsym) and
                    assigned(tstaticvarsym(sym).defaultconstsym) then
                   WriteFieldSym(tabstractvarsym(tstaticvarsym(sym).defaultconstsym));
               end;
             constsym:
               begin
                 { multiple procedures can have constants with the same name }
                 if not assigned(sym.owner.defowner) or
                    (tdef(sym.owner.defowner).typ<>procdef) then
                   WriteConstSym(tconstsym(sym));
               end;
             procsym:
               begin
                 for j:=0 to tprocsym(sym).procdeflist.count-1 do
                   if not(df_generic in tprocdef(tprocsym(sym).procdeflist[j]).defoptions) then
                     WriteSymtableVarSyms(tprocdef(tprocsym(sym).procdeflist[j]).localst);
               end;
             else
               ;
           end;
         end;
      end;


    procedure TJasminAssembler.WriteSymtableProcdefs(st: TSymtable);
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
                     not(df_generic in def.defoptions) then
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

    procedure TJasminAssembler.WriteSymtableStructDefs(st: TSymtable);
      var
        i   : longint;
        def : tdef;
        obj : tabstractrecorddef;
        nestedstructs: tfpobjectlist;
      begin
        if not assigned(st) then
          exit;
        nestedstructs:=tfpobjectlist.create(false);
        for i:=0 to st.DefList.Count-1 do
          begin
            def:=tdef(st.DefList[i]);
            if df_generic in def.defoptions then
              continue;
            case def.typ of
              objectdef:
                if not(oo_is_external in tobjectdef(def).objectoptions) then
                  nestedstructs.add(def);
              recorddef:
                nestedstructs.add(def);
              else
                ;
            end;
          end;
        for i:=0 to nestedstructs.count-1 do
          begin
            obj:=tabstractrecorddef(nestedstructs[i]);
            NewAsmFileForStructDef(obj);
            WriteExtraHeader(obj);
            WriteSymtableVarSyms(obj.symtable);
            writer.AsmLn;
            WriteSymtableProcDefs(obj.symtable);
            WriteSymtableStructDefs(obj.symtable);
          end;
        nestedstructs.free;
      end;


    function TJasminAssembler.CreateNewAsmWriter: TExternalAssemblerOutputFile;
      begin
        Result:=TJasminAssemblerOutputFile.Create(self);
      end;


    constructor TJasminAssembler.CreateWithWriter(info: pasminfo; wr: TExternalAssemblerOutputFile; freewriter, smart: boolean);
      begin
        inherited;
        InstrWriter:=TJasminInstrWriter.Create(self);
        asmfiles:=TCmdStrList.Create;
      end;


    procedure TJasminAssembler.WriteAsmList;
      begin
        { the code for Java methods needs to be emitted class per class,
          so instead of iterating over all asmlists, we iterate over all types
          and global variables (a unit becomes a class, with its global
          variables static fields) }
        writer.MarkEmpty;
        WriteExtraHeader(nil);
        { print all global variables }
        WriteSymtableVarSyms(current_module.globalsymtable);
        WriteSymtableVarSyms(current_module.localsymtable);
        writer.AsmLn;
        { print all global procedures/functions }
        WriteSymtableProcdefs(current_module.globalsymtable);
        WriteSymtableProcdefs(current_module.localsymtable);

        WriteSymtableStructDefs(current_module.globalsymtable);
        WriteSymtableStructDefs(current_module.localsymtable);

        writer.AsmLn;
      end;


{****************************************************************************}
{                         Jasmin Instruction Writer                          }
{****************************************************************************}

     constructor TJasminInstrWriter.create(_owner: TJasminAssembler);
       begin
         inherited create;
         owner := _owner;
       end;

    function getreferencestring(var ref : treference) : ansistring;
      begin
        if (ref.arrayreftype<>art_none) or
           (ref.index<>NR_NO) then
          internalerror(2010122809);
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
          top_single:
            begin
              result:=constsingle(o.sval);
            end;
          top_double:
            begin
              result:=constdouble(o.dval);
            end;
          top_string:
            begin
              result:=constastr(o.pcval,o.pcvallen);
            end;
          top_wstring:
            begin
              result:=constwstr(o.pwstrval^.data,getlengthwidestring(o.pwstrval));
            end
          else
            internalerror(2010122802);
        end;
      end;


    procedure TJasminInstrWriter.WriteInstruction(hp: tai);
      var
        s: ansistring;
        i: byte;
        sep: ansistring;
      begin
        s:=#9+jas_op2str[taicpu(hp).opcode];
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
{                         Jasmin Instruction Writer                          }
{****************************************************************************}

  const
    as_jvm_jasmin_info : tasminfo =
       (
         id     : as_jvm_jasmin;
         idtxt  : 'Jasmin';
         asmbin : 'java';
         asmcmd : '-jar $JASMINJAR $ASM $EXTRAOPT -d $OBJDIR';
         supported_targets : [system_jvm_java32,system_jvm_android32];
         flags : [];
         labelprefix : 'L';
         labelmaxlen : -1;
         comment : ' ; ';
         dollarsign : '$';
       );


begin
  RegisterAssembler(as_jvm_jasmin_info,TJasminAssembler);
end.
