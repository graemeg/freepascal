{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements an asmoutput class for the Nasm assembler with
    Intel syntax for the i386+

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
unit ag386nsm;

{$i fpcdefs.inc}

interface

    uses
      cpubase,
      aasmbase,aasmtai,aasmdata,aasmcpu,assemble,cgutils;

    type
      T386NasmAssembler = class(texternalassembler)
      private
        procedure WriteReference(var ref : treference);
        procedure WriteOper(const o:toper;s : topsize; opcode: tasmop;ops:longint;dest : boolean);
        procedure WriteOper_jmp(const o:toper; op : tasmop);
        procedure WriteSection(atype:TAsmSectiontype;const aname:string);
      public
        procedure WriteTree(p:TAsmList);override;
        procedure WriteAsmList;override;
        procedure WriteExternals;
        procedure WriteSmartExternals;
      end;



  implementation

    uses
      cutils,globtype,globals,systems,cclasses,
      fmodule,finput,verbose,cpuinfo,cgbase
      ;

    type
{$ifdef cpuextended}
      t80bitarray = array[0..9] of byte;
{$endif cpuextended}
      t64bitarray = array[0..7] of byte;
      t32bitarray = array[0..3] of byte;
    const
      line_length = 64;

      nasm_regname_table : array[tregisterindex] of string[7] = (
        {r386nasm.inc contains the Nasm name of each register.}
        {$i r386nasm.inc}
      );

    function nasm_regname(r:Tregister):string;
      var
        p : tregisterindex;
      begin
        p:=findreg_by_number(r);
        if p<>0 then
          result:=nasm_regname_table[p]
        else
          result:=generic_regname(r);
      end;


   function fixline(s:string):string;
   {
     return s with all leading and ending spaces and tabs removed
   }
     var
       i,j,k : longint;
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

    function single2str(d : single) : string;
      var
         hs : string;
         p : longint;
      begin
         str(d,hs);
      { nasm expects a lowercase e }
         p:=pos('E',hs);
         if p>0 then
          hs[p]:='e';
         p:=pos('+',hs);
         if p>0 then
          delete(hs,p,1);
         single2str:=lower(hs);
      end;

    function double2str(d : double) : string;
      var
         hs : string;
         p : longint;
      begin
         str(d,hs);
      { nasm expects a lowercase e }
         p:=pos('E',hs);
         if p>0 then
          hs[p]:='e';
         p:=pos('+',hs);
         if p>0 then
          delete(hs,p,1);
         double2str:=lower(hs);
      end;

    function extended2str(e : extended) : string;
      var
         hs : string;
         p : longint;
      begin
         str(e,hs);
      { nasm expects a lowercase e }
         p:=pos('E',hs);
         if p>0 then
          hs[p]:='e';
         p:=pos('+',hs);
         if p>0 then
          delete(hs,p,1);
         extended2str:=lower(hs);
      end;


  { convert floating point values }
  { to correct endian             }
  procedure swap64bitarray(var t: t64bitarray);
    var
     b: byte;
    begin
      b:= t[7];
      t[7] := t[0];
      t[0] := b;

      b := t[6];
      t[6] := t[1];
      t[1] := b;

      b:= t[5];
      t[5] := t[2];
      t[2] := b;

      b:= t[4];
      t[4] := t[3];
      t[3] := b;
   end;


   procedure swap32bitarray(var t: t32bitarray);
    var
     b: byte;
    begin
      b:= t[1];
      t[1]:= t[2];
      t[2]:= b;

      b:= t[0];
      t[0]:= t[3];
      t[3]:= b;
    end;


    function comp2str(d : bestreal) : string;
      type
        pdouble = ^double;
      var
        c  : comp;
        dd : pdouble;
      begin
         c:=comp(d);
         dd:=pdouble(@c); { this makes a bitwise copy of c into a double }
         comp2str:=double2str(dd^);
      end;


    function sizestr(s:topsize;dest:boolean):string;
      begin
        case s of
           S_B : sizestr:='byte ';
           S_W : sizestr:='word ';
           S_L : sizestr:='dword ';
           S_Q : sizestr:='qword ';
           S_IS : sizestr:='word ';
           S_IL : sizestr:='dword ';
           S_IQ : sizestr:='qword ';
           S_FS : sizestr:='dword ';
           S_FL : sizestr:='qword ';
           S_FX : sizestr:='tword ';
           S_BW : if dest then
               sizestr:='word '
             else
               sizestr:='byte ';
           S_BL : if dest then
               sizestr:='dword '
             else
               sizestr:='byte ';
           S_WL : if dest then
               sizestr:='dword '
             else
               sizestr:='word ';
          else { S_NO }
            sizestr:='';
        end;
      end;


    Function PadTabs(const p:string;addch:char):string;
      var
        s : string;
        i : longint;
      begin
        i:=length(p);
        if addch<>#0 then
         begin
           inc(i);
           s:=p+addch;
         end
        else
         s:=p;
        if i<8 then
         PadTabs:=s+#9#9
        else
         PadTabs:=s+#9;
      end;


    type
      PExternChain = ^TExternChain;

      TExternChain = Record
        psym : pshortstring;
        is_defined : boolean;
        next : PExternChain;
      end;

    const
      FEC : PExternChain = nil;

    procedure AddSymbol(symname : string; defined : boolean);
    var
       EC : PExternChain;
    begin
      EC:=FEC;
      while assigned(EC) do
        begin
          if EC^.psym^=symname then
            begin
              if defined then
                EC^.is_defined:=true;
              exit;
            end;
          EC:=EC^.next;
        end;
      New(EC);
      EC^.next:=FEC;
      FEC:=EC;
      FEC^.psym:=stringdup(symname);
      FEC^.is_defined := defined;
    end;

    procedure FreeExternChainList;
    var
       EC : PExternChain;
    begin
      EC:=FEC;
      while assigned(EC) do
        begin
          FEC:=EC^.next;
          stringdispose(EC^.psym);
          Dispose(EC);
          EC:=FEC;
        end;
    end;

{****************************************************************************
                               T386NasmAssembler
 ****************************************************************************}

    procedure T386NasmAssembler.WriteReference(var ref : treference);
      var
        first : boolean;
      begin
        with ref do
         begin
           AsmWrite('[');
           first:=true;
           if (segment<>NR_NO) then
             AsmWrite(nasm_regname(segment)+':');
           if assigned(symbol) then
            begin
              AsmWrite(symbol.name);
              if SmartAsm then
                AddSymbol(symbol.name,false);
              first:=false;
            end;
           if (base<>NR_NO) then
            begin
              if not(first) then
               AsmWrite('+')
              else
               first:=false;
              AsmWrite(nasm_regname(base))
            end;
           if (index<>NR_NO) then
             begin
               if not(first) then
                 AsmWrite('+')
               else
                 first:=false;
               AsmWrite(nasm_regname(index));
               if scalefactor<>0 then
                 AsmWrite('*'+tostr(scalefactor));
             end;
           if offset<0 then
             begin
               AsmWrite(tostr(offset));
               first:=false;
             end
           else if (offset>0) then
             begin
               AsmWrite('+'+tostr(offset));
               first:=false;
             end;
           if first then
             AsmWrite('0');
           AsmWrite(']');
         end;
       end;


    procedure T386NasmAssembler.WriteOper(const o:toper;s : topsize; opcode: tasmop;ops:longint;dest : boolean);
      begin
        case o.typ of
          top_reg :
            AsmWrite(nasm_regname(o.reg));
          top_const :
            begin
              if (ops=1) and (opcode<>A_RET) then
               AsmWrite(sizestr(s,dest));
              AsmWrite(tostr(longint(o.val)));
            end;
          top_ref :
            begin
              if o.ref^.refaddr=addr_no then
                begin
                  if not ((opcode = A_LEA) or (opcode = A_LGS) or
                          (opcode = A_LSS) or (opcode = A_LFS) or
                          (opcode = A_LES) or (opcode = A_LDS) or
<<<<<<< HEAD
<<<<<<< HEAD
                         // (opcode = A_SHR) or (opcode = A_SHL) or
=======
                          //(opcode = A_SHR) or (opcode = A_SHL) or
>>>>>>> graemeg/fixes_2_2
=======
                          //(opcode = A_SHR) or (opcode = A_SHL) or
>>>>>>> origin/fixes_2_2
                          (opcode = A_SAR) or (opcode = A_SAL) or
                          (opcode = A_OUT) or (opcode = A_IN)) then
                    AsmWrite(sizestr(s,dest));
                  WriteReference(o.ref^);
                end
              else
                begin
                  asmwrite('dword ');
                  if assigned(o.ref^.symbol) then
                   begin
                    if SmartAsm then
                      AddSymbol(o.ref^.symbol.name,false);
                    asmwrite(o.ref^.symbol.name);
                    if o.ref^.offset=0 then
                      exit;
                   end;
                  if o.ref^.offset>0 then
                   asmwrite('+');
                  asmwrite(tostr(o.ref^.offset));

                  end;
            end;
          else
            internalerror(10001);
        end;
      end;


    procedure T386NasmAssembler.WriteOper_jmp(const o:toper; op : tasmop);
      begin
        case o.typ of
          top_reg :
            AsmWrite(nasm_regname(o.reg));
          top_ref :
            if o.ref^.refaddr=addr_no then
              WriteReference(o.ref^)
            else
              begin
                if not(
                       (op=A_JCXZ) or (op=A_JECXZ) or
{$ifdef x86_64}
                       (op=A_JRCXZ) or
{$endif x86_64}
                       (op=A_LOOP) or (op=A_LOOPE) or
                       (op=A_LOOPNE) or (op=A_LOOPNZ) or
                       (op=A_LOOPZ)
                      ) then
                  AsmWrite('NEAR ');
                AsmWrite(o.ref^.symbol.name);
                if SmartAsm then
                  AddSymbol(o.ref^.symbol.name,false);
                if o.ref^.offset>0 then
                 AsmWrite('+'+tostr(o.ref^.offset))
                else
                 if o.ref^.offset<0 then
                  AsmWrite(tostr(o.ref^.offset));
              end;
          top_const :
            AsmWrite(tostr(aint(o.val)));
          else
            internalerror(10001);
        end;
      end;


    const
<<<<<<< HEAD
      ait_const2str : array[aitconst_128bit..aitconst_secrel32_symbol] of string[20]=(
        #9'FIXME_128BIT'#9,#9'FIXME_64BIT'#9,#9'DD'#9,#9'DW'#9,#9'DB'#9,
        #9'FIXME_SLEB128BIT'#9,#9'FIXME_ULEB128BIT'#9,
        #9'RVA'#9,#9'SECREL32'#9
=======
      ait_const2str : array[aitconst_128bit..aitconst_indirect_symbol] of string[20]=(
        #9'FIXME_128BIT'#9,#9'FIXME_64BIT'#9,#9'DD'#9,#9'DW'#9,#9'DB'#9,
        #9'FIXME_SLEB128BIT'#9,#9'FIXME_ULEB128BIT'#9,
        #9'RVA'#9,#9'SECREL32'#9,#9'FIXMEINDIRECT'#9
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      );

    procedure T386NasmAssembler.WriteSection(atype:TAsmSectiontype;const aname:string);
      const
<<<<<<< HEAD
        secnames : array[TAsmSectiontype] of string[length('__DATA, __datacoal_nt,coalesced')] = ('','',
=======
        secnames : array[TAsmSectiontype] of string[17] = ('',
>>>>>>> origin/fixes_2.4
          '.text',
          '.data',
          '.data',
          '.rodata',
          '.bss',
          '.tbss',
          '.pdata',
          '.text','.data','.data','.data','.data',
          '.stab',
          '.stabstr',
          '.idata2','.idata4','.idata5','.idata6','.idata7','.edata',
          '.eh_frame',
          '.debug_frame','.debug_info','.debug_line','.debug_abbrev',
          '.fpc',
          '',
          '.init',
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
          '.objc_protolist'
=======
          '.fini'
>>>>>>> graemeg/fixes_2_2
=======
          '.fini'
>>>>>>> origin/fixes_2_2
=======
          '.fini'
>>>>>>> origin/fixes_2.4
        );
      begin
        AsmLn;
        AsmWrite('SECTION ');
        AsmWrite(secnames[atype]);
        if create_smartlink_sections and
           (atype<>sec_bss) and
           (aname<>'') then
          begin
            AsmWrite('.');
            AsmWrite(aname);
          end;
        AsmLn;
        LasTSecType:=atype;
      end;

    procedure T386NasmAssembler.WriteTree(p:TAsmList);
{$ifdef cpuextended}
    type
      t80bitarray = array[0..9] of byte;
{$endif cpuextended}
    var
      s : string;
      hp       : tai;
      hp1      : tailineinfo;
      counter,
      lines,
      i,j,l    : longint;
      InlineLevel : longint;
      consttype : taiconst_type;
      do_line,
      quoted   : boolean;
      co       : comp;
      sin      : single;
      d        : double;
{$ifdef cpuextended}
      e        : extended;
{$endif cpuextended}
    begin
      if not assigned(p) then
       exit;
      InlineLevel:=0;
      { lineinfo is only needed for al_procedures (PFV) }
      do_line:=(cs_asm_source in current_settings.globalswitches) or
               ((cs_lineinfo in current_settings.moduleswitches)
                 and (p=current_asmdata.asmlists[al_procedures]));
      hp:=tai(p.first);
      while assigned(hp) do
       begin

         if not(hp.typ in SkipLineInfo) then
           begin
             hp1:=hp as tailineinfo;
             current_filepos:=hp1.fileinfo;
             if do_line and (InlineLevel=0) then
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
                       AsmWriteLn(target_asm.comment+'['+infile.name^+']');
                       if assigned(lastinfile) then
                         lastinfile.close;
                     end;
                   if (hp1.fileinfo.line<>lastfileinfo.line) and
                      ((hp1.fileinfo.line<infile.maxlinebuf) or (InlineLevel>0)) then
                     begin
                       if (hp1.fileinfo.line<>0) and
                          ((infile.linebuf^[hp1.fileinfo.line]>=0) or (InlineLevel>0)) then
                         AsmWriteLn(target_asm.comment+'['+tostr(hp1.fileinfo.line)+'] '+
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
               AsmWrite(target_asm.comment);
               AsmWritePChar(tai_comment(hp).str);
               AsmLn;
             End;

           ait_regalloc :
             begin
               if (cs_asm_regalloc in current_settings.globalswitches) then
                 AsmWriteLn(#9#9+target_asm.comment+'Register '+nasm_regname(tai_regalloc(hp).reg)+
                   regallocstr[tai_regalloc(hp).ratype]);
             end;

           ait_tempalloc :
             begin
               if (cs_asm_tempalloc in current_settings.globalswitches) then
                 begin
{$ifdef EXTDEBUG}
                   if assigned(tai_tempalloc(hp).problem) then
                     AsmWriteLn(target_asm.comment+tai_tempalloc(hp).problem^+' ('+tostr(tai_tempalloc(hp).temppos)+','+
                       tostr(tai_tempalloc(hp).tempsize)+')')
                   else
{$endif EXTDEBUG}
                     AsmWriteLn(target_asm.comment+'Temp '+tostr(tai_tempalloc(hp).temppos)+','+
                       tostr(tai_tempalloc(hp).tempsize)+tempallocstr[tai_tempalloc(hp).allocation]);
                 end;
             end;

           ait_section :
             begin
               if tai_section(hp).sectype<>sec_none then
                 WriteSection(tai_section(hp).sectype,tai_section(hp).name^);
               LasTSecType:=tai_section(hp).sectype;
             end;

           ait_align :
             begin
               { nasm gives warnings when it finds align in bss as it
                 wants to store data }
               if (lastsectype<>sec_bss) and
                  (tai_align(hp).aligntype>1) then
                 AsmWriteLn(#9'ALIGN '+tostr(tai_align(hp).aligntype));
             end;

           ait_datablock :
             begin
               if tai_datablock(hp).is_global then
                begin
                  AsmWrite(#9'GLOBAL ');
                  AsmWriteLn(tai_datablock(hp).sym.name);
                end;
               AsmWrite(PadTabs(tai_datablock(hp).sym.name,':'));
               if SmartAsm then
                 AddSymbol(tai_datablock(hp).sym.name,true);
               AsmWriteLn('RESB'#9+tostr(tai_datablock(hp).size));
             end;

           ait_const:
             begin
               consttype:=tai_const(hp).consttype;
               case consttype of
                 aitconst_64bit :
                    begin
                      if assigned(tai_const(hp).sym) then
                        internalerror(200404292);
                      AsmWrite(ait_const2str[aitconst_32bit]);
                      AsmWrite(tostr(longint(lo(tai_const(hp).value))));
                      AsmWrite(',');
                      AsmWrite(tostr(longint(hi(tai_const(hp).value))));
                      AsmLn;
                    end;
                 aitconst_uleb128bit,
                 aitconst_sleb128bit,
                 aitconst_128bit:
                    begin
                    end;
                 aitconst_32bit,
                 aitconst_16bit,
                 aitconst_8bit,
                 aitconst_rva_symbol,
<<<<<<< HEAD
<<<<<<< HEAD
                 aitconst_secrel32_symbol :
=======
=======
>>>>>>> origin/fixes_2_2
                 aitconst_secrel32_symbol,
                 aitconst_indirect_symbol :
>>>>>>> graemeg/fixes_2_2
                   begin
                     AsmWrite(ait_const2str[tai_const(hp).consttype]);
                     l:=0;
                     repeat
                       if assigned(tai_const(hp).sym) then
                         begin
                           if SmartAsm then
                             begin
                               AddSymbol(tai_const(hp).sym.name,false);
                               if assigned(tai_const(hp).endsym) then
                                 AddSymbol(tai_const(hp).endsym.name,false);
                             end;
                           if assigned(tai_const(hp).endsym) then
                             s:=tai_const(hp).endsym.name+'-'+tai_const(hp).sym.name
                           else
                             s:=tai_const(hp).sym.name;
                           if tai_const(hp).value<>0 then
                             s:=s+tostr_with_plus(tai_const(hp).value);
                         end
                       else
                         s:=tostr(tai_const(hp).value);
                       AsmWrite(s);
                       inc(l,length(s));
                       if (l>line_length) or
                          (hp.next=nil) or
                          (tai(hp.next).typ<>ait_const) or
                          (tai_const(hp.next).consttype<>consttype) then
                         break;
                       hp:=tai(hp.next);
                       AsmWrite(',');
                     until false;
                     AsmLn;
                   end;
                 else
                   internalerror(200704252);
               end;
             end;

{$if defined(cpuextended) and defined(FPC_HAS_TYPE_EXTENDED)}
           ait_real_80bit :
             begin
               if do_line then
                AsmWriteLn(target_asm.comment+'value: '+extended2str(tai_real_80bit(hp).value));
             { Make sure e is a extended type, bestreal could be
               a different type (bestreal) !! (PFV) }
               e:=tai_real_80bit(hp).value;
               AsmWrite(#9#9'DB'#9);
               for i:=0 to 9 do
                begin
                  if i<>0 then
                   AsmWrite(',');
                  AsmWrite(tostr(t80bitarray(e)[i]));
                end;
<<<<<<< HEAD
<<<<<<< HEAD
                for i:=11 to tai_real_80bit(hp).savesize do
                  AsmWrite(',0');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
               AsmLn;
             end;
{$else cpuextended}
           ait_real_80bit :
             AsmWriteLn(#9#9'DT'#9+extended2str(tai_real_80bit(hp).value));
{$endif cpuextended}

           // ait_real_64bit :
           //   AsmWriteLn(#9#9'DQ'#9+double2str(tai_real_64bit(hp).value));
           ait_real_64bit :
             begin
               if do_line then
                AsmWriteLn(target_asm.comment+'value: '+double2str(tai_real_64bit(hp).value));
               d:=tai_real_64bit(hp).value;
               { swap the values to correct endian if required }
               if source_info.endian <> target_info.endian then
                 swap64bitarray(t64bitarray(d));
               AsmWrite(#9#9'DB'#9);
{$ifdef arm}
               if tai_real_64bit(hp).formatoptions=fo_hiloswapped then
                 begin
                   for i:=4 to 7 do
                     begin
                       if i<>4 then
                         AsmWrite(',');
                       AsmWrite(tostr(t64bitarray(d)[i]));
                     end;
                   for i:=0 to 3 do
                     begin
                       AsmWrite(',');
                       AsmWrite(tostr(t64bitarray(d)[i]));
                     end;
                 end
               else
{$endif arm}
                 begin
                   for i:=0 to 7 do
                     begin
                       if i<>0 then
                         AsmWrite(',');
                       AsmWrite(tostr(t64bitarray(d)[i]));
                     end;
                 end;
               AsmLn;
             end;
           // ait_real_32bit :
           //   AsmWriteLn(#9#9'DD'#9+single2str(tai_real_32bit(hp).value));
           ait_real_32bit :
             begin
               if do_line then
                 AsmWriteLn(target_asm.comment+'value: '+single2str(tai_real_32bit(hp).value));
               sin:=tai_real_32bit(hp).value;
               { swap the values to correct endian if required }
               if source_info.endian <> target_info.endian then
                 swap32bitarray(t32bitarray(sin));
               AsmWrite(#9#9'DB'#9);
               for i:=0 to 3 do
                begin
                  if i<>0 then
                    AsmWrite(',');
                  AsmWrite(tostr(t32bitarray(sin)[i]));
                end;
               AsmLn;
             end;
           // ait_comp_64bit :
           //   AsmWriteLn(#9#9'DQ'#9+comp2str(tai_real_80bit(hp).value));
           ait_comp_64bit :
             begin
               if do_line then
                AsmWriteLn(target_asm.comment+'value: '+extended2str(tai_comp_64bit(hp).value));
               AsmWrite(#9#9'DB'#9);
               co:=comp(tai_comp_64bit(hp).value);
               { swap the values to correct endian if required }
               if source_info.endian <> target_info.endian then
                 swap64bitarray(t64bitarray(co));
               for i:=0 to 7 do
                begin
                  if i<>0 then
                   AsmWrite(',');
                  AsmWrite(tostr(t64bitarray(co)[i]));
                end;
               AsmLn;
             end;

           ait_string :
             begin
               counter := 0;
               lines := tai_string(hp).len div line_length;
             { separate lines in different parts }
               if tai_string(hp).len > 0 then
                Begin
                  for j := 0 to lines-1 do
                   begin
                     AsmWrite(#9#9'DB'#9);
                     quoted:=false;
                     for i:=counter to counter+line_length-1 do
                        begin
                          { it is an ascii character. }
                          if (ord(tai_string(hp).str[i])>31) and
                             (ord(tai_string(hp).str[i])<128) and
                             (tai_string(hp).str[i]<>'"') then
                              begin
                                if not(quoted) then
                                    begin
                                      if i>counter then
                                        AsmWrite(',');
                                      AsmWrite('"');
                                    end;
                                AsmWrite(tai_string(hp).str[i]);
                                quoted:=true;
                              end { if > 31 and < 128 and ord('"') }
                          else
                              begin
                                  if quoted then
                                      AsmWrite('"');
                                  if i>counter then
                                      AsmWrite(',');
                                  quoted:=false;
                                  AsmWrite(tostr(ord(tai_string(hp).str[i])));
                              end;
                       end; { end for i:=0 to... }
                     if quoted then AsmWrite('"');
                       AsmWrite(target_info.newline);
                     inc(counter,line_length);
                  end; { end for j:=0 ... }
                { do last line of lines }
                if counter<tai_string(hp).len then
                  AsmWrite(#9#9'DB'#9);
                quoted:=false;
                for i:=counter to tai_string(hp).len-1 do
                  begin
                    { it is an ascii character. }
                    if (ord(tai_string(hp).str[i])>31) and
                       (ord(tai_string(hp).str[i])<128) and
                       (tai_string(hp).str[i]<>'"') then
                        begin
                          if not(quoted) then
                              begin
                                if i>counter then
                                  AsmWrite(',');
                                AsmWrite('"');
                              end;
                          AsmWrite(tai_string(hp).str[i]);
                          quoted:=true;
                        end { if > 31 and < 128 and " }
                    else
                        begin
                          if quoted then
                            AsmWrite('"');
                          if i>counter then
                              AsmWrite(',');
                          quoted:=false;
                          AsmWrite(tostr(ord(tai_string(hp).str[i])));
                        end;
                  end; { end for i:=0 to... }
                if quoted then
                  AsmWrite('"');
                end;
               AsmLn;
             end;

           ait_label :
             begin
               if tai_label(hp).labsym.is_used then
                AsmWriteLn(tai_label(hp).labsym.name+':');
               if SmartAsm then
                 AddSymbol(tai_label(hp).labsym.name,true);
             end;

           ait_symbol :
             begin
               if tai_symbol(hp).has_value then
                 internalerror(2009090803);
               if tai_symbol(hp).is_global then
                begin
                  AsmWrite(#9'GLOBAL ');
                  AsmWriteLn(tai_symbol(hp).sym.name);
                end;
               AsmWrite(tai_symbol(hp).sym.name);
               if SmartAsm then
                 AddSymbol(tai_symbol(hp).sym.name,true);
               if assigned(hp.next) and not(tai(hp.next).typ in
                  [ait_const,
                   ait_real_32bit,ait_real_64bit,ait_real_80bit,ait_comp_64bit,ait_string]) then
                AsmWriteLn(':')
             end;

           ait_symbol_end : ;

           ait_instruction :
             begin
               taicpu(hp).CheckNonCommutativeOpcodes;
               { We need intel order, no At&t }
               taicpu(hp).SetOperandOrder(op_intel);
               s:='';
               if ((taicpu(hp).opcode=A_FADDP) or
                   (taicpu(hp).opcode=A_FMULP))
                  and (taicpu(hp).ops=0) then
                 begin
                   taicpu(hp).allocate_oper(2);
                   taicpu(hp).oper[0]^.typ:=top_reg;
                   taicpu(hp).oper[0]^.reg:=NR_ST1;
                   taicpu(hp).oper[1]^.typ:=top_reg;
                   taicpu(hp).oper[1]^.reg:=NR_ST;
                 end;
               if taicpu(hp).opcode=A_FWAIT then
                AsmWriteln(#9#9'DB'#9'09bh')
               else
                begin
                  { We need to explicitely set
                    word prefix to get selectors
                    to be pushed in 2 bytes  PM }
                  if (taicpu(hp).opsize=S_W) and
                     ((taicpu(hp).opcode=A_PUSH) or
                      (taicpu(hp).opcode=A_POP)) and
                      (taicpu(hp).oper[0]^.typ=top_reg) and
                      (is_segment_reg(taicpu(hp).oper[0]^.reg)) then
                    AsmWriteln(#9#9'DB'#9'066h');
                  AsmWrite(#9#9+std_op2str[taicpu(hp).opcode]+cond2str[taicpu(hp).condition]);
                  if taicpu(hp).ops<>0 then
                   begin
                     if is_calljmp(taicpu(hp).opcode) then
                      begin
                        AsmWrite(#9);
                        WriteOper_jmp(taicpu(hp).oper[0]^,taicpu(hp).opcode);
                      end
                     else
                      begin
                        for i:=0 to taicpu(hp).ops-1 do
                         begin
                           if i=0 then
                            AsmWrite(#9)
                           else
                            AsmWrite(',');
                           WriteOper(taicpu(hp).oper[i]^,taicpu(hp).opsize,taicpu(hp).opcode,taicpu(hp).ops,(i=2));
                         end;
                      end;
                   end;
                  AsmLn;
                end;
             end;

           ait_stab,
           ait_force_line,
           ait_function_name : ;

           ait_cutobject :
             begin
               if SmartAsm then
                begin
                 { only reset buffer if nothing has changed }
                 if AsmSize=AsmStartSize then
                  AsmClear
                 else
                  begin
                    if SmartAsm then
                      begin
                        WriteSmartExternals;
                        FreeExternChainList;
                      end;
                    AsmClose;
                    DoAssemble;
                    AsmCreate(tai_cutobject(hp).place);
                  end;
               { avoid empty files }
                 while assigned(hp.next) and (tai(hp.next).typ in [ait_cutobject,ait_section,ait_comment]) do
                  begin
                    if tai(hp.next).typ=ait_section then
                      lasTSectype:=tai_section(hp.next).sectype;
                    hp:=tai(hp.next);
                  end;
                 if lasTSectype<>sec_none then
                   WriteSection(lasTSectype,'');
                 AsmStartSize:=AsmSize;
               end;
             end;

           ait_marker :
             if tai_marker(hp).kind=mark_NoLineInfoStart then
               inc(InlineLevel)
             else if tai_marker(hp).kind=mark_NoLineInfoEnd then
               dec(InlineLevel);

           ait_directive :
             begin
               case tai_directive(hp).directive of
                 asd_nasm_import :
                   AsmWrite('import ');
                 asd_extern :
                   AsmWrite('EXTERN ');
                 else
                   internalerror(200509191);
               end;
               if assigned(tai_directive(hp).name) then
                 begin

                   if SmartAsm then
                     AddSymbol(tai_directive(hp).name^,false);

                   AsmWrite(tai_directive(hp).name^);
                 end;
               AsmLn;
             end;

           else
             internalerror(10000);
         end;
         hp:=tai(hp.next);
       end;
    end;


    procedure T386NasmAssembler.WriteExternals;
      var
        sym : TAsmSymbol;
        i   : longint;
      begin
        for i:=0 to current_asmdata.AsmSymbolDict.Count-1 do
          begin
            sym:=TAsmSymbol(current_asmdata.AsmSymbolDict[i]);
            if sym.bind=AB_EXTERNAL then
              AsmWriteln('EXTERN'#9+sym.name);
          end;
      end;

    procedure T386NasmAssembler.WriteSmartExternals;
      var
        EC : PExternChain;
      begin
        EC:=FEC;
        while assigned(EC) do
          begin
            if not EC^.is_defined then
              AsmWriteln('EXTERN'#9+EC^.psym^);
            EC:=EC^.next;
          end;
      end;


    procedure T386NasmAssembler.WriteAsmList;
    var
      hal : tasmlisttype;
    begin
{$ifdef EXTDEBUG}
      if assigned(current_module.mainsource) then
       comment(v_info,'Start writing nasm-styled assembler output for '+current_module.mainsource^);
{$endif}
      AsmWriteLn('BITS 32');
      AsmLn;

      WriteExternals;

      for hal:=low(TasmlistType) to high(TasmlistType) do
        begin
          AsmWriteLn(target_asm.comment+'Begin asmlist '+AsmListTypeStr[hal]);
          writetree(current_asmdata.asmlists[hal]);
          AsmWriteLn(target_asm.comment+'End asmlist '+AsmListTypeStr[hal]);
        end;

      AsmLn;
      if SmartAsm then
        begin
          WriteSmartExternals;
          FreeExternChainList;
        end;
{$ifdef EXTDEBUG}
      if assigned(current_module.mainsource) then
       comment(v_info,'Done writing nasm-styled assembler output for '+current_module.mainsource^);
{$endif EXTDEBUG}
   end;


{*****************************************************************************
                                  Initialize
*****************************************************************************}

    const
       as_i386_nasmcoff_info : tasminfo =
          (
            id           : as_i386_nasmcoff;
            idtxt  : 'NASMCOFF';
            asmbin : 'nasm';
            asmcmd : '-f coff -o $OBJ $ASM';
            supported_targets : [system_i386_go32v2];
            flags : [af_allowdirect,af_needar,af_no_debug];
            labelprefix : '..@';
            comment : '; ';
          );

       as_i386_nasmwin32_info : tasminfo =
          (
            id           : as_i386_nasmwin32;
            idtxt  : 'NASMWIN32';
            asmbin : 'nasm';
            asmcmd : '-f win32 -o $OBJ $ASM';
            supported_targets : [system_i386_win32];
            flags : [af_allowdirect,af_needar,af_no_debug];
            labelprefix : '..@';
            comment : '; ';
          );

       as_i386_nasmobj_info : tasminfo =
          (
            id           : as_i386_nasmobj;
            idtxt  : 'NASMOBJ';
            asmbin : 'nasm';
            asmcmd : '-f obj -o $OBJ $ASM';
            supported_targets : [system_i386_embedded];
            flags : [af_allowdirect,af_needar,af_no_debug];
            labelprefix : '..@';
            comment : '; ';
          );

       as_i386_nasmwdosx_info : tasminfo =
          (
            id           : as_i386_nasmwdosx;
            idtxt  : 'NASMWDOSX';
            asmbin : 'nasm';
            asmcmd : '-f win32 -o $OBJ $ASM';
            supported_targets : [system_i386_wdosx];
            flags : [af_allowdirect,af_needar,af_no_debug];
            labelprefix : '..@';
            comment : '; ';
          );


       as_i386_nasmelf_info : tasminfo =
          (
            id           : as_i386_nasmelf;
            idtxt  : 'NASMELF';
            asmbin : 'nasm';
            asmcmd : '-f elf -o $OBJ $ASM';
            supported_targets : [system_i386_linux];
            flags : [af_allowdirect,af_needar,af_no_debug];
            labelprefix : '..@';
            comment : '; ';
          );

       as_i386_nasmbeos_info : tasminfo =
          (
            id           : as_i386_nasmbeos;
            idtxt  : 'NASMELF';
            asmbin : 'nasm';
            asmcmd : '-f elf -o $OBJ $ASM';
            supported_targets : [system_i386_beos];
            flags : [af_allowdirect,af_needar,af_no_debug];
            labelprefix : '..@';
            comment : '; ';
          );

       as_i386_nasmhaiku_info : tasminfo =
          (
            id           : as_i386_nasmhaiku;
            idtxt  : 'NASMELF';
            asmbin : 'nasm';
            asmcmd : '-f elf -o $OBJ $ASM';
            supported_targets : [system_i386_haiku];
            flags : [af_allowdirect,af_needar,af_no_debug];
            labelprefix : '..@';
            comment : '; ';
          );


initialization
  RegisterAssembler(as_i386_nasmcoff_info,T386NasmAssembler);
  RegisterAssembler(as_i386_nasmwin32_info,T386NasmAssembler);
  RegisterAssembler(as_i386_nasmwdosx_info,T386NasmAssembler);
  RegisterAssembler(as_i386_nasmobj_info,T386NasmAssembler);
  RegisterAssembler(as_i386_nasmbeos_info,T386NasmAssembler);
  RegisterAssembler(as_i386_nasmhaiku_info,T386NasmAssembler);
  RegisterAssembler(as_i386_nasmelf_info,T386NasmAssembler);
end.
