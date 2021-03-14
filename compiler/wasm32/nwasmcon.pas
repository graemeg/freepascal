{
    Copyright (c) 1998-2011 by Florian Klaempfl and Jonas Maebe

    Generate assembler for constant nodes for the WebAssembly

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
unit nwasmcon;

{$i fpcdefs.inc}

interface

    uses
       globtype,aasmbase,
       symtype,
       node,ncal,ncon,ncgcon;

    type
      (*
       tjvmordconstnode = class(tcgordconstnode)
          { normally, we convert the enum constant into a load of the
            appropriate enum class field in pass_1. In some cases (array index),
            we want to keep it as an enum constant however }
          enumconstok: boolean;
          function pass_1: tnode; override;
          function docompare(p: tnode): boolean; override;
          function dogetcopy: tnode; override;
       end;
         *)

       twasmrealconstnode = class(tcgrealconstnode)
          procedure pass_generate_code;override;
       end;

       (*tjvmstringconstnode = class(tstringconstnode)
          function pass_1: tnode; override;
          procedure pass_generate_code;override;
          class function emptydynstrnil: boolean; override;
       end;
       *)
       (*
       tjvmsetconsttype = (
         { create symbol for the set constant; the symbol will be initialized
           in the class constructor/unit init code (default) }
         sct_constsymbol,
         { normally, we convert the set constant into a constructor/factory
           method to create a set instance. In some cases (simple "in"
           expressions, adding an element to an empty set, ...) we want to
           keep the set constant instead }
         sct_notransform,
         { actually construct a JUBitSet/JUEnumSet that contains the set value
           (for initializing the sets contstants) }
         sct_construct
         );
       tjvmsetconstnode = class(tcgsetconstnode)
          setconsttype: tjvmsetconsttype;
          function pass_1: tnode; override;
          procedure pass_generate_code; override;
          constructor create(s : pconstset;def:tdef);override;
          function docompare(p: tnode): boolean; override;
          function dogetcopy: tnode; override;
         protected
          function emitvarsetconst: tasmsymbol; override;
          { in case the set has only a single run of consecutive elements,
            this function will return its starting index and length }
          function find_single_elements_run(from: longint; out start, len: longint): boolean;
          function buildbitset: tnode;
          function buildenumset(const eledef: tdef): tnode;
          function buildsetfromstring(const helpername: string; otherparas: tcallparanode): tnode;
       end;
       *)

implementation

    uses
      globals,cutils,widestr,verbose,constexp,fmodule,
      symdef,symsym,symcpu,symtable,symconst,
      aasmdata,aasmcpu,defutil,
      nutils,ncnv,nld,nmem,pass_1,
      cgbase,hlcgobj,hlcgcpu,cgutils,cpubase
      ;


{*****************************************************************************
                           TJVMORDCONSTNODE
*****************************************************************************}
              (*
    function tjvmordconstnode.pass_1: tnode;
      var
        basedef: tcpuenumdef;
        sym: tenumsym;
        classfield: tsym;
      begin
        if (resultdef.typ<>enumdef) or
           enumconstok then
          begin
            result:=inherited pass_1;
            exit;
          end;
        { convert into JVM class instance }
        { a) find the enumsym corresponding to the value (may not exist in case
             of an explicit typecast of an integer -> error) }
        sym:=nil;
        sym:=tenumsym(tenumdef(resultdef).int2enumsym(int64(value)));
        if not assigned(sym) then
          begin
            Message(parser_e_range_check_error);
            result:=nil;
            exit;
          end;
        { b) find the corresponding class field }
        basedef:=tcpuenumdef(tenumdef(resultdef).getbasedef);
        classfield:=search_struct_member(basedef.classdef,sym.name);

        { c) create loadnode of the field }
        result:=nil;
        if not handle_staticfield_access(classfield,result) then
          internalerror(2011062606);
      end;


    function tjvmordconstnode.docompare(p: tnode): boolean;
      begin
        result:=inherited docompare(p);
        if result then
          result:=(enumconstok=tjvmordconstnode(p).enumconstok);
      end;


    function tjvmordconstnode.dogetcopy: tnode;
      begin
        result:=inherited dogetcopy;
        tjvmordconstnode(result).enumconstok:=enumconstok;
      end;
   *)

{*****************************************************************************
                           TWASMREALCONSTNODE
*****************************************************************************}

    procedure twasmrealconstnode.pass_generate_code;
      begin
        location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
        location.register:=hlcg.getfpuregister(current_asmdata.CurrAsmList,resultdef);
        thlcgwasm(hlcg).a_loadfpu_const_stack(current_asmdata.CurrAsmList,resultdef,value_real);
        thlcgwasm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,resultdef,location.register);
      end;


    { tcgstringconstnode }
  (*
    function tjvmstringconstnode.pass_1: tnode;
      var
        strclass: tobjectdef;
        pw: pcompilerwidestring;
        paras: tcallparanode;
        wasansi: boolean;
      begin
        { all Java strings are utf-16. However, there is no way to
          declare a constant array of bytes (or any other type), those
          have to be constructed by declaring a final field and then
          initialising them in the class constructor element per
          element. We therefore put the straight ASCII values into
          the UTF-16 string, and then at run time extract those and
          store them in an Ansistring/AnsiChar array }
        result:=inherited pass_1;
        if assigned(result) or
           (cst_type in [cst_unicodestring,cst_widestring]) then
          exit;
        { convert the constant into a widestring representation without any
          code page conversion }
        initwidestring(pw);
        ascii2unicode(value_str,len,current_settings.sourcecodepage,pw,false);
        ansistringdispose(value_str,len);
        pcompilerwidestring(value_str):=pw;
        { and now add a node to convert the data into ansistring format at
          run time }
        wasansi:=false;
        case cst_type of
          cst_ansistring:
            begin
              if len=0 then
                begin
                  { we have to use nil rather than an empty string, because an
                    empty string has a code page and this messes up the code
                    page selection logic in the RTL }
                  exit;
                end;
              strclass:=tobjectdef(search_system_type('ANSISTRINGCLASS').typedef);
              wasansi:=true;
            end;
          cst_shortstring:
            strclass:=tobjectdef(search_system_type('SHORTSTRINGCLASS').typedef);
          cst_conststring:
            { used for array of char }
            strclass:=tobjectdef(search_system_type('ANSICHARARRAYCLASS').typedef);
          else
           internalerror(2011052401);
        end;
        cst_type:=cst_unicodestring;
        paras:=ccallparanode.create(self.getcopy,nil);
        if wasansi then
          paras:=ccallparanode.create(
            genintconstnode(tstringdef(resultdef).encoding),paras);
        { since self will be freed, have to make a copy }
        result:=ccallnode.createinternmethodres(
          cloadvmtaddrnode.create(ctypenode.create(strclass)),
          'CREATEFROMLITERALSTRINGBYTES',paras,resultdef);
      end;


    procedure tjvmstringconstnode.pass_generate_code;
      begin
        location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
        location.register:=hlcg.getaddressregister(current_asmdata.CurrAsmList,resultdef);
        case cst_type of
          cst_ansistring:
            begin
              if len<>0 then
                internalerror(2012052604);
              hlcg.a_load_const_reg(current_asmdata.CurrAsmList,resultdef,0,location.register);
              { done }
              exit;
            end;
          cst_shortstring,
          cst_conststring:
            internalerror(2012052601);
          cst_unicodestring,
          cst_widestring:
            current_asmdata.CurrAsmList.concat(taicpu.op_wstring(a_ldc,pcompilerwidestring(value_str)));
          else
            internalerror(2012052602);
        end;
        thlcgjvm(hlcg).incstack(current_asmdata.CurrAsmList,1);
        thlcgjvm(hlcg).a_load_stack_reg(current_asmdata.CurrAsmList,resultdef,location.register);
      end;

    class function tjvmstringconstnode.emptydynstrnil: boolean;
      begin
        result:=false;
      end;


    {*****************************************************************************
                               TJVMSETCONSTNODE
    *****************************************************************************}

    function tjvmsetconstnode.buildsetfromstring(const helpername: string; otherparas: tcallparanode): tnode;
      var
        pw: pcompilerwidestring;
        wc: tcompilerwidechar;
        i, j, bit, nulls: longint;
      begin
        initwidestring(pw);
        nulls:=0;
        for i:=0 to 15 do
          begin
            wc:=0;
            for bit:=0 to 15 do
              if (i*16+bit) in value_set^ then
                wc:=wc or (1 shl (15-bit));
            { don't add trailing zeroes }
            if wc=0 then
              inc(nulls)
            else
              begin
                for j:=1 to nulls do
                  concatwidestringchar(pw,0);
                nulls:=0;
                concatwidestringchar(pw,wc);
              end;
          end;
        result:=ccallnode.createintern(helpername,
          ccallparanode.create(cstringconstnode.createunistr(pw),otherparas));
        donewidestring(pw);
      end;


    function tjvmsetconstnode.buildbitset: tnode;
      var
        mp: tnode;
      begin
        if value_set^=[] then
          begin
            mp:=cloadvmtaddrnode.create(ctypenode.create(java_jubitset));
            result:=ccallnode.createinternmethod(mp,'CREATE',nil);
            exit;
          end;
        result:=buildsetfromstring('fpc_bitset_from_string',nil);
      end;


    function tjvmsetconstnode.buildenumset(const eledef: tdef): tnode;
      var
        stopnode: tnode;
        startnode: tnode;
        mp: tnode;
        len: longint;
        start: longint;
        enumele: tnode;
        paras: tcallparanode;
        hassinglerun: boolean;
      begin
        hassinglerun:=find_single_elements_run(0, start, len);
        if hassinglerun then
          begin
            mp:=cloadvmtaddrnode.create(ctypenode.create(java_juenumset));
            if len=0 then
              begin
                enumele:=cloadvmtaddrnode.create(ctypenode.create(tcpuenumdef(tenumdef(eledef).getbasedef).classdef));
                inserttypeconv_explicit(enumele,search_system_type('JLCLASS').typedef);
                paras:=ccallparanode.create(enumele,nil);
                result:=ccallnode.createinternmethod(mp,'NONEOF',paras)
              end
            else
              begin
                startnode:=cordconstnode.create(start,eledef,false);
                { immediately firstpass so the enum gets translated into a JLEnum
                  instance }
                firstpass(startnode);
                if len=1 then
                  result:=ccallnode.createinternmethod(mp,'OF',ccallparanode.create(startnode,nil))
                else
                  begin
                    stopnode:=cordconstnode.create(start+len-1,eledef,false);
                    firstpass(stopnode);
                    result:=ccallnode.createinternmethod(mp,'RANGE',ccallparanode.create(stopnode,ccallparanode.create(startnode,nil)));
                  end
              end
          end
        else
          begin
            enumele:=cordconstnode.create(tenumsym(tenumdef(eledef).symtable.symlist[0]).value,eledef,false);
            firstpass(enumele);
            paras:=ccallparanode.create(enumele,nil);
            result:=buildsetfromstring('fpc_enumset_from_string',paras);
          end;
      end;


    function tjvmsetconstnode.pass_1: tnode;
      var
        eledef: tdef;
      begin
        { we want set constants to be global, so we can reuse them. However,
          if the set's elementdef is local, we can't do that since a global
          symbol cannot have a local definition (the compiler will crash when
          loading the ppu file afterwards) }
        if tsetdef(resultdef).elementdef.owner.symtabletype=localsymtable then
          setconsttype:=sct_construct;
        result:=nil;
        case setconsttype of
          //sct_constsymbol:
          //  begin
          //    { normally a codegen pass routine, but we have to insert a typed
          //      const in case the set constant does not exist yet, and that
          //      should happen in pass_1 (especially since it involves creating
          //      new nodes, which may even have to be tacked on to this code in
          //      case it's the unit initialization code) }
          //    handlevarsetconst;
          //    { no smallsets }
          //    expectloc:=LOC_CREFERENCE;
          //  end;
          sct_notransform:
            begin
              result:=inherited pass_1;
              { no smallsets }
              expectloc:=LOC_CREFERENCE;
            end;
          sct_constsymbol,
          sct_construct:
            begin
              eledef:=tsetdef(resultdef).elementdef;
              { empty sets don't have an element type, so we don't know whether we
                have to constructor a bitset or enumset (and of which type) }
              if not assigned(eledef) then
                internalerror(2011070202);
              if eledef.typ=enumdef then
                begin
                  result:=buildenumset(eledef);
                end
              else
                begin
                  result:=buildbitset;
                end;
              inserttypeconv_explicit(result,cpointerdef.getreusable(resultdef));
              result:=cderefnode.create(result);
            end;
        end;
      end;


    procedure tjvmsetconstnode.pass_generate_code;
      begin
        case setconsttype of
          sct_constsymbol:
            begin
              { all sets are varsets for the JVM target, no setbase differences }
              handlevarsetconst;
            end;
          else
            { must be handled in pass_1 or otherwise transformed }
            internalerror(2011070201)
        end;
      end;

    constructor tjvmsetconstnode.create(s: pconstset; def: tdef);
      begin
        inherited create(s, def);
        setconsttype:=sct_constsymbol;
      end;


    function tjvmsetconstnode.docompare(p: tnode): boolean;
      begin
        result:=
          inherited docompare(p) and
          (setconsttype=tjvmsetconstnode(p).setconsttype);
      end;


    function tjvmsetconstnode.dogetcopy: tnode;
      begin
        result:=inherited dogetcopy;
        tjvmsetconstnode(result).setconsttype:=setconsttype;
      end;


    function tjvmsetconstnode.emitvarsetconst: tasmsymbol;
      var
        csym: tconstsym;
        ssym: tstaticvarsym;
        ps: pnormalset;
      begin
        { add a read-only typed constant }
        new(ps);
        ps^:=value_set^;
        csym:=cconstsym.create_ptr('_$setconst'+tostr(current_module.symlist.count),constset,ps,resultdef);
        csym.visibility:=vis_private;
        include(csym.symoptions,sp_internal);
        current_module.localsymtable.insert(csym);
        { generate assignment of the constant to the typed constant symbol }
        ssym:=jvm_add_typed_const_initializer(csym);
        result:=current_asmdata.RefAsmSymbol(ssym.mangledname,AT_DATA);
      end;


    function tjvmsetconstnode.find_single_elements_run(from: longint; out start, len: longint): boolean;
      var
        i: longint;
      begin
        i:=from;
        result:=true;
        { find first element in set }
        while (i<=255) and
              not(i in value_set^) do
          inc(i);
        start:=i;
        { go to end of the run }
        while (i<=255) and
              (i in value_set^) do
          inc(i);
        len:=i-start;
        { rest must be unset }
        while (i<=255) and
              not(i in value_set^) do
          inc(i);
        if i<>256 then
          result:=false;
      end;

    *)

begin
   //cordconstnode:=tjvmordconstnode;
   crealconstnode:=twasmrealconstnode;
   //cstringconstnode:=tjvmstringconstnode;
   //csetconstnode:=tjvmsetconstnode;
end.
