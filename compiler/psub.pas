{
    Copyright (c) 1998-2002 by Florian Klaempfl, Daniel Mantione

    Does the parsing and codegeneration at subroutine level

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
unit psub;

{$i fpcdefs.inc}

interface

    uses
      globals,
      node,nbas,
      symdef,procinfo,optdfa;

    type

      { tcgprocinfo }

      tcgprocinfo = class(tprocinfo)
      private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        procedure CreateInlineInfo;
        { returns the node which is the start of the user code, this is needed by the dfa }
        function GetUserCode: tnode;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        procedure maybe_add_constructor_wrapper(var tocode: tnode; withexceptblock: boolean);
        procedure add_entry_exit_code;
        procedure setup_tempgen;
      public
        { code for the subroutine as tree }
        code : tnode;
        { positions in the tree for init/final }
        entry_asmnode,
        loadpara_asmnode,
        exitlabel_asmnode,
        stackcheck_asmnode,
        init_asmnode,
        final_asmnode : tasmnode;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        final_used : boolean;
        dfabuilder : TDFABuilder;

        destructor  destroy;override;

        function calc_stackframe_size : longint;override;

=======
        dfabuilder : TDFABuilder;
        destructor  destroy;override;
>>>>>>> graemeg/cpstrnew
=======
        dfabuilder : TDFABuilder;
        destructor  destroy;override;
>>>>>>> graemeg/cpstrnew
=======
        dfabuilder : TDFABuilder;
        destructor  destroy;override;
>>>>>>> graemeg/cpstrnew
=======
        dfabuilder : TDFABuilder;
        destructor  destroy;override;
>>>>>>> origin/cpstrnew
        procedure printproc(pass:string);
        procedure generate_code;
        procedure generate_code_tree;
        procedure generate_exceptfilter(nestedpi: tcgprocinfo);
        procedure resetprocdef;
        procedure add_to_symtablestack;
        procedure remove_from_symtablestack;
        procedure parse_body;

        function has_assembler_child : boolean;
      end;


    procedure printnode_reset;

    { reads the declaration blocks }
    procedure read_declarations(islibrary : boolean);

    { reads declarations in the interface part of a unit }
    procedure read_interface_declarations;

    { reads any routine in the implementation, or a non-method routine
      declaration in the interface (depending on whether or not parse_only is
      true) }
    procedure read_proc(isclassmethod:boolean; usefwpd: tprocdef;isgeneric:boolean);

    procedure generate_specialization_procs;


implementation

    uses
       sysutils,
       { common }
       cutils,
       { global }
       globtype,tokens,verbose,comphook,constexp,
<<<<<<< HEAD
       systems,cpubase,aasmbase,aasmtai,aasmdata,
=======
       systems,
       { aasm }
       cpuinfo,cpubase,aasmbase,aasmtai,aasmdata,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       { symtable }
       symconst,symbase,symsym,symtype,symtable,defutil,symcreat,
       paramgr,
       ppu,fmodule,
       { pass 1 }
       nutils,ngenutil,nld,ncal,ncon,nflw,nadd,ncnv,nmem,
       pass_1,
    {$ifdef state_tracking}
       nstate,
    {$endif state_tracking}
       { pass 2 }
{$ifndef NOPASS2}
       pass_2,
{$endif}
       { parser }
       scanner,gendef,
       pbase,pstatmnt,pdecl,pdecsub,pexports,pgenutil,pparautl,pgentype,
       { codegen }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       tgobj,cgbase,cgobj,cgutils,hlcgobj,hlcgcpu,dbgbase,
{$ifdef llvm}
      { override create_hlcodegen from hlcgcpu }
      hlcgllvm,
{$endif}
=======
       tgobj,cgbase,cgobj,cgcpu,dbgbase,
>>>>>>> graemeg/cpstrnew
=======
       tgobj,cgbase,cgobj,cgcpu,dbgbase,
>>>>>>> graemeg/cpstrnew
=======
       tgobj,cgbase,cgobj,cgcpu,dbgbase,
>>>>>>> graemeg/cpstrnew
=======
       tgobj,cgbase,cgobj,cgcpu,dbgbase,
>>>>>>> origin/cpstrnew
       ncgutil,regvars,
       optbase,
       opttail,
       optcse,
       optloop,
       optconstprop,
       optdeadstore
{$if defined(arm)}
       ,cpuinfo
{$endif arm}
       {$ifndef NOOPT}
         {$ifdef i386}
           ,aopt386
         {$else i386}
           ,aopt
         {$endif i386}
       {$endif}
       ;

    function checknodeinlining(procdef: tprocdef): boolean;
      var
        i : integer;
        currpara : tparavarsym;
      begin
        result := false;
        { this code will never be used (only specialisations can be inlined),
          and moreover contains references to defs that are not stored in the
          ppu file }
        if df_generic in current_procinfo.procdef.defoptions then
          exit;
        if pi_has_assembler_block in current_procinfo.flags then
          begin
            Message1(parser_h_not_supported_for_inline,'assembler');
            Message(parser_h_inlining_disabled);
            exit;
          end;
        if pi_has_global_goto in current_procinfo.flags then
          begin
            Message1(parser_h_not_supported_for_inline,'global goto');
            Message(parser_h_inlining_disabled);
            exit;
          end;
        if pi_has_nested_exit in current_procinfo.flags then
          begin
            Message1(parser_h_not_supported_for_inline,'nested exit');
            Message(parser_h_inlining_disabled);
            exit;
          end;
        if pi_calls_c_varargs in current_procinfo.flags then
          begin
            Message1(parser_h_not_supported_for_inline,'called C-style varargs functions');
            Message(parser_h_inlining_disabled);
            exit;
          end;
        { the compiler cannot handle inherited in inlined subroutines because
          it tries to search for self in the symtable, however, the symtable
          is not available }
        if pi_has_inherited in current_procinfo.flags then
          begin
            Message1(parser_h_not_supported_for_inline,'inherited');
            Message(parser_h_inlining_disabled);
            exit;
          end;
        for i:=0 to procdef.paras.count-1 do
          begin
            currpara:=tparavarsym(procdef.paras[i]);
            case currpara.vardef.typ of
              formaldef :
                begin
                  if (currpara.varspez in [vs_out,vs_var,vs_const,vs_constref]) then
                    begin
                      Message1(parser_h_not_supported_for_inline,'formal parameter');
                      Message(parser_h_inlining_disabled);
                      exit;
                    end;
                end;
              arraydef :
                begin
                  if is_array_of_const(currpara.vardef) or
                     is_variant_array(currpara.vardef) then
                    begin
                      Message1(parser_h_not_supported_for_inline,'array of const');
                      Message(parser_h_inlining_disabled);
                      exit;
                    end;
                  { open arrays might need re-basing of the index, i.e. if you pass
                    an array[1..10] as open array, you have to add 1 to all index operations
                    if you directly inline it }
                  if is_open_array(currpara.vardef) then
                    begin
                      Message1(parser_h_not_supported_for_inline,'open array');
                      Message(parser_h_inlining_disabled);
                      exit;
                    end;
                end;
            end;
        end;
        result:=true;
      end;


{****************************************************************************
                      PROCEDURE/FUNCTION BODY PARSING
****************************************************************************}

    procedure initializedefaultvars(p:TObject;arg:pointer);
      var
        b : tblocknode;
      begin
        if tsym(p).typ<>localvarsym then
         exit;
        with tabstractnormalvarsym(p) do
         begin
           if (vo_is_default_var in varoptions) and (vardef.size>0) then
             begin
               b:=tblocknode(arg);
               b.left:=cstatementnode.create(
                         ccallnode.createintern('fpc_zeromem',
                           ccallparanode.create(
                             cordconstnode.create(vardef.size,ptruinttype,false),
                             ccallparanode.create(
                               caddrnode.create_internal(
                                 cloadnode.create(tsym(p),tsym(p).owner)),
                                 nil
                               )
                             )
                           ),
                         b.left);
             end;
         end;
      end;


    procedure initializevars(p:TObject;arg:pointer);
      var
        b : tblocknode;
      begin
        if not (tsym(p).typ in [localvarsym,staticvarsym]) then
         exit;
        with tabstractnormalvarsym(p) do
         begin
           if assigned(defaultconstsym) then
            begin
              b:=tblocknode(arg);
              b.left:=cstatementnode.create(
                        cassignmentnode.create(
                            cloadnode.create(tsym(p),tsym(p).owner),
                            cloadnode.create(defaultconstsym,defaultconstsym.owner)),
                        b.left);
            end
           else
             initializedefaultvars(p,arg);
         end;
      end;


    procedure check_finalize_paras(p:TObject;arg:pointer);
      begin
<<<<<<< HEAD
        if (tsym(p).typ=paravarsym) then
          begin
            if tparavarsym(p).needs_finalization then
              begin
                include(current_procinfo.flags,pi_needs_implicit_finally);
                include(current_procinfo.flags,pi_do_call);
              end;
            if (tparavarsym(p).varspez in [vs_value,vs_out]) and
               (cs_create_pic in current_settings.moduleswitches) and
               (tf_pic_uses_got in target_info.flags) and
               is_rtti_managed_type(tparavarsym(p).vardef) then
              include(current_procinfo.flags,pi_needs_got);
          end;
=======
        if (tsym(p).typ=paravarsym) and
           (tparavarsym(p).varspez=vs_value) and
           is_managed_type(tparavarsym(p).vardef) then
          include(current_procinfo.flags,pi_needs_implicit_finally);
>>>>>>> graemeg/cpstrnew
      end;


    procedure check_finalize_locals(p:TObject;arg:pointer);
      begin
        { include the result: it needs to be finalized in case an exception }
        { occurs                                                            }
        if (tsym(p).typ=localvarsym) and
           (tlocalvarsym(p).refs>0) and
<<<<<<< HEAD
<<<<<<< HEAD
           is_managed_type(tlocalvarsym(p).vardef) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          begin
            include(current_procinfo.flags,pi_needs_implicit_finally);
            include(current_procinfo.flags,pi_do_call);
            if is_rtti_managed_type(tlocalvarsym(p).vardef) and
              (cs_create_pic in current_settings.moduleswitches) and
              (tf_pic_uses_got in target_info.flags) then
              include(current_procinfo.flags,pi_needs_got);
          end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
           not(is_class(tlocalvarsym(p).vardef)) and
           tlocalvarsym(p).vardef.needs_inittable then
>>>>>>> graemeg/fixes_2_2
          include(current_procinfo.flags,pi_needs_implicit_finally);
>>>>>>> graemeg/cpstrnew
      end;


    function block(islibrary : boolean) : tnode;
      var
        oldfilepos: tfileposinfo;
      begin
         { parse const,types and vars }
         read_declarations(islibrary);

         { do we have an assembler block without the po_assembler?
           we should allow this for Delphi compatibility (PFV) }
         if (token=_ASM) and (m_delphi in current_settings.modeswitches) then
           include(current_procinfo.procdef.procoptions,po_assembler);

         { Handle assembler block different }
         if (po_assembler in current_procinfo.procdef.procoptions) then
          begin
            block:=assembler_block;
            exit;
          end;

         {Unit initialization?.}
         if (
             assigned(current_procinfo.procdef.localst) and
             (current_procinfo.procdef.localst.symtablelevel=main_program_level) and
             (current_module.is_unit or islibrary)
            ) then
           begin
             if (token=_END) then
                begin
                   consume(_END);
                   { We need at least a node, else the entry/exit code is not
                     generated and thus no PASCALMAIN symbol which we need (PFV) }
                   if islibrary then
                    block:=cnothingnode.create
                   else
                    block:=nil;
                end
              else
                begin
                   if token=_INITIALIZATION then
                     begin
                        { The library init code is already called and does not
                          need to be in the initfinal table (PFV) }
                        block:=statement_block(_INITIALIZATION);
                        { optimize empty initialization block away }
                        if (block.nodetype=blockn) and (tblocknode(block).left=nil) then
                          FreeAndNil(block)
                        else
                          if not islibrary then
                            current_module.flags:=current_module.flags or uf_init;
                     end
                   else if token=_FINALIZATION then
                     begin
                       { when a unit has only a finalization section, we can come to this
                         point when we try to read the nonh existing initalization section
                         so we've to check if we are really try to parse the finalization }
                       if current_procinfo.procdef.proctypeoption=potype_unitfinalize then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                         block:=statement_block(_FINALIZATION)
                       else
                         block:=nil;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                         begin
                           block:=statement_block(_FINALIZATION);
                           { optimize empty finalization block away }
                           if (block.nodetype=blockn) and (tblocknode(block).left=nil) then
                             FreeAndNil(block)
                           else
                             current_module.flags:=current_module.flags or uf_finalize;
                         end
                         else
                           block:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                     end
                   else
                     block:=statement_block(_BEGIN);
                end;
            end
         else
            begin
               { parse routine body }
               block:=statement_block(_BEGIN);
               { initialized variables }
               if current_procinfo.procdef.localst.symtabletype=localsymtable then
                 begin
                   { initialization of local variables with their initial
                     values: part of function entry }
                   oldfilepos:=current_filepos;
                   current_filepos:=current_procinfo.entrypos;
                   current_procinfo.procdef.localst.SymList.ForEachCall(@initializevars,block);
                   current_filepos:=oldfilepos;
                 end
               else if current_procinfo.procdef.localst.symtabletype=staticsymtable then
                 begin
                   { for program and unit initialization code we also need to
                     initialize the local variables used of Default() }
                   oldfilepos:=current_filepos;
                   current_filepos:=current_procinfo.entrypos;
                   current_procinfo.procdef.localst.SymList.ForEachCall(@initializedefaultvars,block);
                   current_filepos:=oldfilepos;
                 end;

               if assigned(current_procinfo.procdef.parentfpstruct) then
                 begin
                   { we only do this after the code has been parsed because
                     otherwise for-loop counters moved to the struct cause
                     errors; we still do it nevertheless to prevent false
                     "unused" symbols warnings and to assist debug info
                     generation }
                   redirect_parentfpstruct_local_syms(current_procinfo.procdef);
                   { finish the parentfpstruct (add padding, ...) }
                   finish_parentfpstruct(current_procinfo.procdef);
                 end;
            end;
      end;


{****************************************************************************
                       PROCEDURE/FUNCTION COMPILING
****************************************************************************}

    procedure printnode_reset;
      begin
        assign(printnodefile,treelogfilename);
        {$push}{$I-}
         rewrite(printnodefile);
        {$pop}
        if ioresult<>0 then
         begin
           Comment(V_Error,'Error creating '+treelogfilename);
           exit;
         end;
        close(printnodefile);
      end;


    procedure add_label_init(p:TObject;arg:pointer);
      begin
        if tstoredsym(p).typ=labelsym then
          begin
            addstatement(tstatementnode(arg^),
              cifnode.create(caddnode.create(equaln,
                ccallnode.createintern('fpc_setjmp',
                  ccallparanode.create(cloadnode.create(tlabelsym(p).jumpbuf,tlabelsym(p).jumpbuf.owner),nil)),
                cordconstnode.create(1,sinttype,true))
              ,cgotonode.create(tlabelsym(p)),nil)
            );
          end;
      end;


    function generate_bodyentry_block:tnode;
      var
        srsym        : tsym;
        para         : tcallparanode;
        call         : tcallnode;
        newstatement : tstatementnode;
        def          : tabstractrecorddef;
      begin
        result:=internalstatements(newstatement);

        if assigned(current_structdef) then
          begin
            { a constructor needs a help procedure }
            if (current_procinfo.procdef.proctypeoption=potype_constructor) then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                if is_class(current_structdef) or
                    (
                      is_objectpascal_helper(current_structdef) and
                      is_class(tobjectdef(current_structdef).extendeddef)
                    ) then
                  begin
                    if is_objectpascal_helper(current_structdef) then
                      def:=tabstractrecorddef(tobjectdef(current_structdef).extendeddef)
                    else
                      def:=current_structdef;
                    srsym:=search_struct_member(def,'NEWINSTANCE');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                if is_class(current_structdef) then
                  begin
                    include(current_procinfo.flags,pi_needs_implicit_finally);
                    srsym:=search_struct_member(current_structdef,'NEWINSTANCE');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    if assigned(srsym) and
                       (srsym.typ=procsym) then
                      begin
                        { if vmt=1 then newinstance }
                        addstatement(newstatement,cifnode.create(
                            caddnode.create_internal(equaln,
                                ctypeconvnode.create_internal(
                                    load_vmt_pointer_node,
                                    voidpointertype),
                                cpointerconstnode.create(1,voidpointertype)),
                            cassignmentnode.create(
                                ctypeconvnode.create_internal(
                                    load_self_pointer_node,
                                    voidpointertype),
                                ccallnode.create(nil,tprocsym(srsym),srsym.owner,
                                  ctypeconvnode.create_internal(load_self_pointer_node,cclassrefdef.create(current_structdef)),
                                  [],nil)),
                            nil));
                      end
                    else
                      internalerror(200305108);
                  end
                else
                  if is_object(current_structdef) then
                    begin
                      { parameter 3 : vmt_offset }
                      { parameter 2 : address of pointer to vmt,
                        this is required to allow setting the vmt to -1 to indicate
                        that memory was allocated }
                      { parameter 1 : self pointer }
                      para:=ccallparanode.create(
                                cordconstnode.create(tobjectdef(current_structdef).vmt_offset,s32inttype,false),
                            ccallparanode.create(
                                ctypeconvnode.create_internal(
                                    load_vmt_pointer_node,
                                    voidpointertype),
                            ccallparanode.create(
                                ctypeconvnode.create_internal(
                                    load_self_pointer_node,
                                    voidpointertype),
                            nil)));
                      addstatement(newstatement,cassignmentnode.create(
                          ctypeconvnode.create_internal(
                              load_self_pointer_node,
                              voidpointertype),
                          ccallnode.createintern('fpc_help_constructor',para)));
                    end
                else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  if is_javaclass(current_structdef) or
                     ((target_info.system in systems_jvm) and
                      is_record(current_structdef)) then
                    begin
                      if (current_procinfo.procdef.proctypeoption=potype_constructor) and
                         not current_procinfo.ConstructorCallingConstructor then
                       begin
                         { call inherited constructor }
                         if is_javaclass(current_structdef) then
                           srsym:=search_struct_member_no_helper(tobjectdef(current_structdef).childof,'CREATE')
                         else
                           srsym:=search_struct_member_no_helper(java_fpcbaserecordtype,'CREATE');
                         if assigned(srsym) and
                            (srsym.typ=procsym) then
                           begin
                             call:=ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[cnf_inherited],nil);
                             exclude(tcallnode(call).callnodeflags,cnf_return_value_used);
                             addstatement(newstatement,call);
                           end
                         else
                           internalerror(2011010312);
                       end;
                    end
                else
                  if not is_record(current_structdef) and
                     not (
                            is_objectpascal_helper(current_structdef) and
                            (tobjectdef(current_structdef).extendeddef.typ<>objectdef)
                         ) then
                    internalerror(200305103);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  if not is_record(current_structdef) then
                  internalerror(200305103);
>>>>>>> graemeg/cpstrnew
                { if self=nil then exit
                  calling fail instead of exit is useless because
                  there is nothing to dispose (PFV) }
                if is_class_or_object(current_structdef) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  addstatement(newstatement,cifnode.create(
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                addstatement(newstatement,cifnode.create(
>>>>>>> graemeg/cpstrnew
                    caddnode.create(equaln,
                        load_self_pointer_node,
                        cnilnode.create),
                    cexitnode.create(nil),
                    nil));
              end;

            { maybe call BeforeDestruction for classes }
            if (current_procinfo.procdef.proctypeoption=potype_destructor) and
               is_class(current_structdef) then
              begin
                srsym:=search_struct_member(current_structdef,'BEFOREDESTRUCTION');
                if assigned(srsym) and
                   (srsym.typ=procsym) then
                  begin
                    { if vmt>0 then beforedestruction }
                    addstatement(newstatement,cifnode.create(
                        caddnode.create(gtn,
                            ctypeconvnode.create_internal(
                              load_vmt_pointer_node,ptrsinttype),
                            ctypeconvnode.create_internal(
                              cnilnode.create,ptrsinttype)),
                        ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[],nil),
                        nil));
                  end
                else
                  internalerror(200305104);
              end;
          end;
        if m_non_local_goto in current_settings.modeswitches then
          tsymtable(current_procinfo.procdef.localst).SymList.ForEachCall(@add_label_init,@newstatement);
      end;


    function generate_bodyexit_block:tnode;
      var
        srsym : tsym;
        para : tcallparanode;
        newstatement : tstatementnode;
        oldlocalswitches: tlocalswitches;
      begin
        result:=internalstatements(newstatement);

        if assigned(current_structdef) then
          begin
            { Don't test self and the vmt here. The reason is that  }
            { a constructor already checks whether these are valid  }
            { before. Further, in case of TThread the thread may    }
            { free the class instance right after AfterConstruction }
            { has been called, so it may no longer be valid (JM)    }
            oldlocalswitches:=current_settings.localswitches;
            current_settings.localswitches:=oldlocalswitches-[cs_check_object,cs_check_range];

            { a destructor needs a help procedure }
            if (current_procinfo.procdef.proctypeoption=potype_destructor) then
              begin
                if is_class(current_structdef) then
                  begin
                    srsym:=search_struct_member(current_structdef,'FREEINSTANCE');
                    if assigned(srsym) and
                       (srsym.typ=procsym) then
                      begin
                        { if self<>0 and vmt<>0 then freeinstance }
                        addstatement(newstatement,cifnode.create(
                            caddnode.create(andn,
                                caddnode.create(unequaln,
                                    load_self_pointer_node,
                                    cnilnode.create),
                                caddnode.create(unequaln,
                                    ctypeconvnode.create(
                                        load_vmt_pointer_node,
                                        voidpointertype),
                                    cpointerconstnode.create(0,voidpointertype))),
                            ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[],nil),
                            nil));
                      end
                    else
                      internalerror(200305108);
                  end
                else
                  if is_object(current_structdef) then
                    begin
                      { finalize object data, but only if not in inherited call }
                      if is_managed_type(current_structdef) then
                        begin
                          addstatement(newstatement,cifnode.create(
                            caddnode.create(unequaln,
                              ctypeconvnode.create_internal(load_vmt_pointer_node,voidpointertype),
                              cnilnode.create),
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            cnodeutils.finalize_data_node(load_self_node),
=======
                            finalize_data_node(load_self_node),
>>>>>>> graemeg/cpstrnew
=======
                            finalize_data_node(load_self_node),
>>>>>>> graemeg/cpstrnew
=======
                            finalize_data_node(load_self_node),
>>>>>>> graemeg/cpstrnew
=======
                            finalize_data_node(load_self_node),
>>>>>>> origin/cpstrnew
                            nil));
                        end;
                      { parameter 3 : vmt_offset }
                      { parameter 2 : pointer to vmt }
                      { parameter 1 : self pointer }
                      para:=ccallparanode.create(
                                cordconstnode.create(tobjectdef(current_structdef).vmt_offset,s32inttype,false),
                            ccallparanode.create(
                                ctypeconvnode.create_internal(
                                    load_vmt_pointer_node,
                                    voidpointertype),
                            ccallparanode.create(
                                ctypeconvnode.create_internal(
                                    load_self_pointer_node,
                                    voidpointertype),
                            nil)));
                      addstatement(newstatement,
                          ccallnode.createintern('fpc_help_destructor',para));
                    end
                else if is_javaclass(current_structdef) then
                  begin
                    { nothing to do }
                  end
                else
                  internalerror(200305105);
              end;
            current_settings.localswitches:=oldlocalswitches;
          end;
      end;


<<<<<<< HEAD
=======
    function generate_except_block:tnode;
      var
        newstatement : tstatementnode;
        { safecall handling }
        exceptobjnode,exceptaddrnode: ttempcreatenode;
        sym,exceptsym: tsym;
      begin
        generate_except_block:=internalstatements(newstatement);

        { a constructor needs call destructor (if available) when it
          is not inherited }
        if not assigned(current_structdef) or
           (current_procinfo.procdef.proctypeoption<>potype_constructor) then
          begin
            { no constructor }
            { must be the return value finalized before reraising the exception? }
            if (not is_void(current_procinfo.procdef.returndef)) and
               is_managed_type(current_procinfo.procdef.returndef) and
               (not paramanager.ret_in_param(current_procinfo.procdef.returndef, current_procinfo.procdef.proccalloption)) and
               (not is_class(current_procinfo.procdef.returndef)) then
              addstatement(newstatement,finalize_data_node(load_result_node));
{$if defined(x86) or defined(arm)}
            { safecall handling }
            if (tf_safecall_exceptions in target_info.flags) and
               (current_procinfo.procdef.proccalloption=pocall_safecall) then
              begin
                { create a local hidden variable "safe_result"    }
                { it will be used in ncgflw unit                  }
                { to set "real" result value for safecall routine }
                sym:=tlocalvarsym.create('$safe_result',vs_value,hresultdef,[]);
                include(sym.symoptions,sp_internal);
                current_procinfo.procdef.localst.insert(sym);
                { if safecall is used for a class method we need to call }
                { SafecallException virtual method                       }
                { In other case we return E_UNEXPECTED error value       }
                if is_class(current_procinfo.procdef.struct) then
                  begin
                    { temp variable to store exception address }
                    exceptaddrnode:=ctempcreatenode.create(voidpointertype,voidpointertype.size,
                      tt_persistent,true);
                    addstatement(newstatement,exceptaddrnode);
                    addstatement(newstatement,
                      cassignmentnode.create(
                        ctemprefnode.create(exceptaddrnode),
                        ccallnode.createintern('fpc_getexceptionaddr',nil)));
                    { temp variable to store popped up exception }
                    exceptobjnode:=ctempcreatenode.create(class_tobject,class_tobject.size,
                      tt_persistent,true);
                    addstatement(newstatement,exceptobjnode);
                    addstatement(newstatement,
                      cassignmentnode.create(
                        ctemprefnode.create(exceptobjnode),
                        ccallnode.createintern('fpc_popobjectstack', nil)));
                    exceptsym:=search_struct_member(tobjectdef(current_procinfo.procdef.struct),'SAFECALLEXCEPTION');
                    addstatement(newstatement,
                      cassignmentnode.create(
                        cloadnode.create(sym,sym.Owner),
                        ccallnode.create(
                          ccallparanode.create(ctemprefnode.create(exceptaddrnode),
                          ccallparanode.create(ctemprefnode.create(exceptobjnode),nil)),
                          tprocsym(exceptsym), tprocsym(exceptsym).owner,load_self_node,[])));
                    addstatement(newstatement,ccallnode.createintern('fpc_destroyexception',
                      ccallparanode.create(ctemprefnode.create(exceptobjnode),nil)));
                    addstatement(newstatement,ctempdeletenode.create(exceptobjnode));
                    addstatement(newstatement,ctempdeletenode.create(exceptaddrnode));
                  end
                else
                  begin
                    { pop up and destroy an exception }
                    addstatement(newstatement,ccallnode.createintern('fpc_destroyexception',
                      ccallparanode.create(ccallnode.createintern('fpc_popobjectstack', nil),nil)));
                    addstatement(newstatement,
                      cassignmentnode.create(
                        cloadnode.create(sym,sym.Owner),
                        genintconstnode(HResult($8000FFFF))));
                  end;
              end;
{$endif}
          end;
      end;


>>>>>>> graemeg/cpstrnew
{****************************************************************************
                                  TCGProcInfo
****************************************************************************}

     destructor tcgprocinfo.destroy;
       begin
         if assigned(code) then
           code.free;
         if not final_used then
           final_asmnode.free;
         inherited destroy;
       end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function tcgprocinfo.calc_stackframe_size:longint;
      begin
        result:=Align(tg.direction*tg.lasttemp,current_settings.alignment.localalignmin);
      end;


=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure tcgprocinfo.printproc(pass:string);
      begin
        assign(printnodefile,treelogfilename);
        {$push}{$I-}
         append(printnodefile);
         if ioresult<>0 then
          rewrite(printnodefile);
        {$pop}
        if ioresult<>0 then
         begin
           Comment(V_Error,'Error creating '+treelogfilename);
           exit;
         end;
        writeln(printnodefile);
        writeln(printnodefile,'*******************************************************************************');
        writeln(printnodefile, pass);
        writeln(printnodefile,procdef.fullprocname(false));
        writeln(printnodefile,'*******************************************************************************');
        printnode(printnodefile,code);
        close(printnodefile);
      end;


    procedure tcgprocinfo.maybe_add_constructor_wrapper(var tocode: tnode; withexceptblock: boolean);
      var
        oldlocalswitches: tlocalswitches;
        srsym: tsym;
        afterconstructionblock,
        exceptblock,
        newblock: tblocknode;
        newstatement: tstatementnode;
        pd: tprocdef;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if assigned(procdef.struct) and
           (procdef.proctypeoption=potype_constructor) then
          begin
            withexceptblock:=
              withexceptblock and
              not(target_info.system in systems_garbage_collected_managed_types);
=======
        if assigned(current_structdef) and
           (current_procinfo.procdef.proctypeoption=potype_constructor) then
          begin
>>>>>>> graemeg/cpstrnew
=======
        if assigned(current_structdef) and
           (current_procinfo.procdef.proctypeoption=potype_constructor) then
          begin
>>>>>>> graemeg/cpstrnew
=======
        if assigned(current_structdef) and
           (current_procinfo.procdef.proctypeoption=potype_constructor) then
          begin
>>>>>>> graemeg/cpstrnew
=======
        if assigned(current_structdef) and
           (current_procinfo.procdef.proctypeoption=potype_constructor) then
          begin
>>>>>>> origin/cpstrnew
            { Don't test self and the vmt here. See generate_bodyexit_block }
            { why (JM)                                                      }
            oldlocalswitches:=current_settings.localswitches;
            current_settings.localswitches:=oldlocalswitches-[cs_check_object,cs_check_range];

            { call AfterConstruction for classes }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            if is_class(procdef.struct) then
              begin
                srsym:=search_struct_member(procdef.struct,'AFTERCONSTRUCTION');
=======
            if is_class(current_structdef) then
              begin
                srsym:=search_struct_member(current_structdef,'AFTERCONSTRUCTION');
>>>>>>> graemeg/cpstrnew
=======
            if is_class(current_structdef) then
              begin
                srsym:=search_struct_member(current_structdef,'AFTERCONSTRUCTION');
>>>>>>> graemeg/cpstrnew
=======
            if is_class(current_structdef) then
              begin
                srsym:=search_struct_member(current_structdef,'AFTERCONSTRUCTION');
>>>>>>> graemeg/cpstrnew
=======
            if is_class(current_structdef) then
              begin
                srsym:=search_struct_member(current_structdef,'AFTERCONSTRUCTION');
>>>>>>> origin/cpstrnew
                if assigned(srsym) and
                   (srsym.typ=procsym) then
                  begin
                    current_filepos:=exitpos;
                    afterconstructionblock:=internalstatements(newstatement);
                    { first execute all constructor code. If no exception
                      occurred then we will execute afterconstruction,
                      otherwise we won't (the exception will jump over us) }
                    addstatement(newstatement,tocode);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    { if implicit finally node wasn't created, then exit label and
                      finalization code must be handled here and placed before
                      afterconstruction }
                    if not ((pi_needs_implicit_finally in flags) and
                      (cs_implicit_exceptions in current_settings.moduleswitches)) then
                      begin
                        include(tocode.flags,nf_block_with_exit);
                        addstatement(newstatement,final_asmnode);
                        final_used:=true;
                      end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    { Self can be nil when fail is called }
                    { if self<>nil and vmt<>nil then afterconstruction }
                    addstatement(newstatement,cifnode.create(
                      caddnode.create(andn,
                        caddnode.create(unequaln,
                          load_self_node,
                          cnilnode.create),
                        caddnode.create(unequaln,
                          load_vmt_pointer_node,
                          cnilnode.create)),
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[],nil),
=======
                        ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[]),
>>>>>>> graemeg/cpstrnew
=======
                        ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[]),
>>>>>>> graemeg/cpstrnew
=======
                        ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[]),
>>>>>>> graemeg/cpstrnew
=======
                        ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[]),
>>>>>>> origin/cpstrnew
                        nil));
                    tocode:=afterconstructionblock;
                  end
                else
                  internalerror(200305106);
              end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            if withexceptblock and (procdef.struct.typ=objectdef) then
              begin
                { Generate the implicit "fail" code for a constructor (destroy
                  in case an exception happened) }
                pd:=tobjectdef(procdef.struct).find_destructor;
                { this will always be the case for classes, since tobject has
                  a destructor }
                if assigned(pd) or is_object(procdef.struct) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            if withexceptblock and (current_structdef.typ=objectdef) then
              begin
                { Generate the implicit "fail" code for a constructor (destroy
                  in case an exception happened) }
                pd:=tobjectdef(current_structdef).find_destructor;
                { this will always be the case for classes, since tobject has
                  a destructor }
                if assigned(pd) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  begin
                    current_filepos:=exitpos;
                    exceptblock:=internalstatements(newstatement);
                    { first free the instance if non-nil }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    if assigned(pd) then
                      { if vmt<>0 then call destructor }
                      addstatement(newstatement,
                        cifnode.create(
                          caddnode.create(unequaln,
                            load_vmt_pointer_node,
                            cnilnode.create),
                          { cnf_create_failed -> don't call BeforeDestruction }
                          ccallnode.create(nil,tprocsym(pd.procsym),pd.procsym.owner,load_self_node,[cnf_create_failed],nil),
                          nil))
                    else
                      { object without destructor, call 'fail' helper }
                      addstatement(newstatement,
                        ccallnode.createintern('fpc_help_fail',
                          ccallparanode.create(
                            cordconstnode.create(tobjectdef(procdef.struct).vmt_offset,s32inttype,false),
                          ccallparanode.create(
                            ctypeconvnode.create_internal(
                              load_vmt_pointer_node,
                              voidpointertype),
                          ccallparanode.create(
                            ctypeconvnode.create_internal(
                              load_self_pointer_node,
                              voidpointertype),
                          nil))))
                      );
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    { if vmt<>0 then call destructor }
                    addstatement(newstatement,cifnode.create(
                      caddnode.create(unequaln,
                        load_vmt_pointer_node,
                        cnilnode.create),
                      { cnf_create_failed -> don't call BeforeDestruction }
                      ccallnode.create(nil,tprocsym(pd.procsym),pd.procsym.owner,load_self_node,[cnf_create_failed]),
                      nil));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    { then re-raise the exception }
                    addstatement(newstatement,craisenode.create(nil,nil,nil));
                    current_filepos:=entrypos;
                    newblock:=internalstatements(newstatement);
                    { try
                        tocode
                      except
                        exceptblock
                      end
                    }
                    addstatement(newstatement,ctryexceptnode.create(
                      tocode,
                      nil,
                      exceptblock));
                    tocode:=newblock;
                  end;
              end;
            current_settings.localswitches:=oldlocalswitches;
          end;
      end;


    procedure tcgprocinfo.add_entry_exit_code;
      var
        finalcode,
        bodyentrycode,
        bodyexitcode,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        wrappedbody,
        newblock     : tnode;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        exceptcode,
        wrappedbody: tnode;
        newblock     : tblocknode;
>>>>>>> graemeg/cpstrnew
        codestatement,
        newstatement : tstatementnode;
        oldfilepos   : tfileposinfo;
        is_constructor: boolean;
      begin
        is_constructor:=assigned(procdef.struct) and
          (procdef.proctypeoption=potype_constructor);

        oldfilepos:=current_filepos;
        { Generate code/locations used at start of proc }
        current_filepos:=entrypos;
        entry_asmnode:=casmnode.create_get_position;
        loadpara_asmnode:=casmnode.create_get_position;
        stackcheck_asmnode:=casmnode.create_get_position;
        init_asmnode:=casmnode.create_get_position;
        bodyentrycode:=generate_bodyentry_block;
        { Generate code/locations used at end of proc }
        current_filepos:=exitpos;
        exitlabel_asmnode:=casmnode.create_get_position;
        final_asmnode:=casmnode.create_get_position;
        final_used:=false;
        bodyexitcode:=generate_bodyexit_block;

        { Generate procedure by combining init+body+final,
          depending on the implicit finally we need to add
          an try...finally...end wrapper }
        newblock:=internalstatements(newstatement);
        { initialization is common for all cases }
        addstatement(newstatement,loadpara_asmnode);
        addstatement(newstatement,stackcheck_asmnode);
        addstatement(newstatement,entry_asmnode);
        addstatement(newstatement,init_asmnode);
        addstatement(newstatement,bodyentrycode);

        if (cs_implicit_exceptions in current_settings.moduleswitches) and
           (pi_needs_implicit_finally in flags) and
           { but it's useless in init/final code of units }
           not(procdef.proctypeoption in [potype_unitfinalize,potype_unitinit]) and
           not(target_info.system in systems_garbage_collected_managed_types) then
          begin
            { Any result of managed type must be returned in parameter }
            if is_managed_type(procdef.returndef) and
               (not paramanager.ret_in_param(procdef.returndef,procdef)) and
               (not is_class(procdef.returndef)) then
               InternalError(2013121301);

            { Generate special exception block only needed when
              implicit finaly is used }
            current_filepos:=exitpos;
            { Generate code that will be in the try...finally }
            finalcode:=internalstatements(codestatement);
            addstatement(codestatement,final_asmnode);
            final_used:=true;

            current_filepos:=entrypos;
            wrappedbody:=ctryfinallynode.create_implicit(
               code,
               finalcode,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               cnothingnode.create);
=======
               exceptcode);
>>>>>>> graemeg/cpstrnew
=======
               exceptcode);
>>>>>>> graemeg/cpstrnew
=======
               exceptcode);
>>>>>>> graemeg/cpstrnew
=======
               exceptcode);
>>>>>>> origin/cpstrnew
            { afterconstruction must be called after final_asmnode, because it
               has to execute after the temps have been finalised in case of a
               refcounted class (afterconstruction decreases the refcount
               without freeing the instance if the count becomes nil, while
               the finalising of the temps can free the instance) }
            maybe_add_constructor_wrapper(wrappedbody,true);
            addstatement(newstatement,wrappedbody);
            addstatement(newstatement,exitlabel_asmnode);
            addstatement(newstatement,bodyexitcode);
            { set flag the implicit finally has been generated }
            include(flags,pi_has_implicit_finally);
          end
        else
          begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            { constructors need destroy-on-exception code even if they don't
              have managed variables/temps }
            maybe_add_constructor_wrapper(code,
              cs_implicit_exceptions in current_settings.moduleswitches);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            maybe_add_constructor_wrapper(code,false);
            addstatement(newstatement,loadpara_asmnode);
            addstatement(newstatement,stackcheck_asmnode);
            addstatement(newstatement,entry_asmnode);
            addstatement(newstatement,init_asmnode);
            addstatement(newstatement,bodyentrycode);
>>>>>>> graemeg/cpstrnew
            addstatement(newstatement,code);
            if assigned(nestedexitlabel) then
              addstatement(newstatement,clabelnode.create(cnothingnode.create,nestedexitlabel));
            addstatement(newstatement,exitlabel_asmnode);
            addstatement(newstatement,bodyexitcode);
            if not is_constructor then
              begin
                addstatement(newstatement,final_asmnode);
                final_used:=true;
              end;
          end;
        do_firstpass(tnode(newblock));
        code:=newblock;
        current_filepos:=oldfilepos;
      end;


    procedure clearrefs(p:TObject;arg:pointer);
      begin
         if (tsym(p).typ in [localvarsym,paravarsym,staticvarsym]) then
           if tabstractvarsym(p).refs>1 then
             tabstractvarsym(p).refs:=1;
      end;


    procedure translate_registers(p:TObject;list:pointer);
      begin
         if (tsym(p).typ in [localvarsym,paravarsym,staticvarsym]) and
            (tabstractnormalvarsym(p).localloc.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_MMREGISTER,
              LOC_CMMREGISTER,LOC_FPUREGISTER,LOC_CFPUREGISTER]) then
           begin
             if not(cs_no_regalloc in current_settings.globalswitches) then
               begin
                 cg.translate_register(tabstractnormalvarsym(p).localloc.register);
                 if (tabstractnormalvarsym(p).localloc.registerhi<>NR_NO) then
                   cg.translate_register(tabstractnormalvarsym(p).localloc.registerhi);
               end;
             if cs_asm_source in current_settings.globalswitches then
               begin
                 TAsmList(list).concat(Tai_comment.Create(strpnew('Var '+tabstractnormalvarsym(p).realname+' located in register '+
                   location_reg2string(tabstractnormalvarsym(p).localloc))));
               end;
           end;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(i386) or defined(x86_64) or defined(arm)}
    const
      exception_flags: array[boolean] of tprocinfoflags = (
        [],
        [pi_uses_exceptions,pi_needs_implicit_finally,pi_has_implicit_finally]
      );
{$endif}

    procedure tcgprocinfo.setup_tempgen;
      begin
        tg:=tgobjclass.create;

{$if defined(i386) or defined(x86_64) or defined(arm)}
{$if defined(arm)}
        { frame and stack pointer must be always the same on arm thumb so it makes no
          sense to fiddle with a frame pointer }
        if GenerateThumbCode then
          begin
            framepointer:=NR_STACK_POINTER_REG;
            tg.direction:=1;
          end
        else
{$endif defined(arm)}
          begin
            { try to strip the stack frame }
            { set the framepointer to esp if:
              - no assembler directive, those are handled in assembler_block
                in pstatment.pas (for cases not caught by the Delphi
                exception below)
              - no exceptions are used
              - no pushes are used/esp modifications, could be:
                * outgoing parameters on the stack on non-fixed stack target
                * incoming parameters on the stack
                * open arrays
              - no inline assembler
             or
              - Delphi mode
              - assembler directive
              - no pushes are used/esp modifications, could be:
                * outgoing parameters on the stack
                * incoming parameters on the stack
                * open arrays
              - no local variables

              - stack frame cannot be optimized if using Win64 SEH
                (at least with the current state of our codegenerator).
            }
            if ((po_assembler in procdef.procoptions) and
               (m_delphi in current_settings.modeswitches) and
               { localst at main_program_level is a staticsymtable }
                (procdef.localst.symtablelevel<>main_program_level) and
                (tabstractlocalsymtable(procdef.localst).count_locals = 0)) or
               ((cs_opt_stackframe in current_settings.optimizerswitches) and
                not(cs_generate_stackframes in current_settings.localswitches) and
                not(cs_profile in current_settings.moduleswitches) and
                not(po_assembler in procdef.procoptions) and
                not ((pi_has_stackparameter in flags)
{$ifndef arm}   { Outgoing parameter(s) on stack do not need stackframe on x86 targets
                 with fixed stack. On ARM it fails, see bug #25050 }
                  and (not paramanager.use_fixed_stack)
{$endif arm}
                  ) and
                ((flags*([pi_has_assembler_block,pi_is_assembler,
                        pi_needs_stackframe]+
                        exception_flags[(target_info.cpu=cpu_i386)
{$ifndef DISABLE_WIN64_SEH}
                        or (target_info.system=system_x86_64_win64)
{$endif DISABLE_WIN64_SEH}
                        ]))=[])
               )
            then
              begin
                { we need the parameter info here to determine if the procedure gets
                  parameters on the stack

                  calling generate_parameter_info doesn't hurt but it costs time
                  (necessary to init para_stack_size)
                }
                generate_parameter_info;

                if not(procdef.stack_tainting_parameter(calleeside)) and
                   not(has_assembler_child) and (para_stack_size=0) then
                  begin
                    { Only need to set the framepointer }
                    framepointer:=NR_STACK_POINTER_REG;
                    tg.direction:=1;
                  end
{$if defined(arm)}
                { On arm, the stack frame size can be estimated to avoid using an extra frame pointer,
                  in case parameters are passed on the stack.

                  However, the draw back is, if the estimation fails, compilation will break later on
                  with an internal error, so this switch is not enabled by default yet. To overcome this,
                  multipass compilation of subroutines must be supported
                }
                else if (cs_opt_forcenostackframe in current_settings.optimizerswitches) and
                   not(has_assembler_child) then
                  begin
                    { Only need to set the framepointer }
                    framepointer:=NR_STACK_POINTER_REG;
                    tg.direction:=1;
                    include(flags,pi_estimatestacksize);
                    set_first_temp_offset;
                    procdef.has_paraloc_info:=callnoside;
                    generate_parameter_info;
                    exit;
                  end;
{$endif defined(arm)}
              end;
          end;
<<<<<<< HEAD
{$endif defined(x86) or defined(arm)}
        { set the start offset to the start of the temp area in the stack }
        set_first_temp_offset;
=======
        do_firstpass(tnode(newblock));
        code:=newblock;
        current_filepos:=oldfilepos;
>>>>>>> origin/cpstrnew
      end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    function tcgprocinfo.has_assembler_child : boolean;
      var
        hp : tprocinfo;
      begin
        result:=false;
        hp:=get_first_nestedproc;
        while assigned(hp) do
          begin
            if (hp.flags*[pi_has_assembler_block,pi_is_assembler])<>[] then
              begin
                result:=true;
                exit;
              end;
            hp:=tprocinfo(hp.next);
<<<<<<< HEAD
          end;
      end;

    procedure tcgprocinfo.generate_code_tree;
      var
        hpi : tcgprocinfo;
      begin
        { generate code for this procedure }
        generate_code;
        { process nested procedures }
        hpi:=tcgprocinfo(get_first_nestedproc);
        while assigned(hpi) do
          begin
            hpi.generate_code_tree;
            hpi:=tcgprocinfo(hpi.next);
=======
>>>>>>> graemeg/cpstrnew
          end;
        resetprocdef;
      end;

<<<<<<< HEAD
    { For SEH, the code from 'finally' blocks must be put into a separate procedures,
      which can be called by OS during stack unwind. This resembles nested procedures,
      but finalizer procedures do not have their own local variables and work directly
      with the stack frame of parent. In particular, the tempgen must be shared, so
      1) finalizer procedure is able to finalize temps of the parent,
      2) if the finalizer procedure is complex enough to need its own temps, they are
         allocated in stack frame of parent, so second-level finalizer procedures are
         not needed.

      Due to requirement of shared tempgen we cannot process finalizer as a regular nested
      procedure (after the parent) and have to do it inline.
      This is called by platform-specific tryfinallynodes during pass2.
      Here we put away the codegen (which carries the register allocator state), process
      the 'nested' procedure, then restore previous cg and continue processing the parent
      procedure. generate_code() will create another cg, but not another tempgen because
      setup_tempgen() is not called for potype_exceptfilter procedures. }

    procedure tcgprocinfo.generate_exceptfilter(nestedpi: tcgprocinfo);
      var
        saved_cg: tcg;
        saved_hlcg: thlcgobj;
{$ifdef cpu64bitalu}
        saved_cg128 : tcg128;
{$else cpu64bitalu}
        saved_cg64 : tcg64;
{$endif cpu64bitalu}
      begin
        if nestedpi.procdef.proctypeoption<>potype_exceptfilter then
          InternalError(201201141);
        { flush code generated this far }
        aktproccode.concatlist(current_asmdata.CurrAsmList);
        { save the codegen }
        saved_cg:=cg;
        saved_hlcg:=hlcg;
        cg:=nil;
        hlcg:=nil;
{$ifdef cpu64bitalu}
        saved_cg128:=cg128;
        cg128:=nil;
{$else cpu64bitalu}
        saved_cg64:=cg64;
        cg64:=nil;
{$endif cpu64bitalu}
        nestedpi.generate_code;
        { prevents generating code the second time when processing nested procedures }
        nestedpi.resetprocdef;
        cg:=saved_cg;
        hlcg:=saved_hlcg;
{$ifdef cpu64bitalu}
        cg128:=saved_cg128;
{$else cpu64bitalu}
        cg64:=saved_cg64;
{$endif cpu64bitalu}
        add_reg_instruction_hook:=@cg.add_reg_instruction;
      end;


     procedure TCGProcinfo.CreateInlineInfo;
       begin
        new(procdef.inlininginfo);
        procdef.inlininginfo^.code:=code.getcopy;
        procdef.inlininginfo^.flags:=flags;
        { The blocknode needs to set an exit label }
        if procdef.inlininginfo^.code.nodetype=blockn then
          include(procdef.inlininginfo^.code.flags,nf_block_with_exit);
        procdef.has_inlininginfo:=true;
       end;


    function searchusercode(var n: tnode; arg: pointer): foreachnoderesult;
      begin
        if nf_usercode_entry in n.flags then
          begin
            pnode(arg)^:=n;
            result:=fen_norecurse_true
          end
        else
          result:=fen_false;
      end;


    function TCGProcinfo.GetUserCode : tnode;
      var
        n : tnode;
      begin
        n:=nil;
        foreachnodestatic(code,@searchusercode,@n);
        if not(assigned(n)) then
          internalerror(2013111004);
        result:=n;
=======

    function tcgprocinfo.has_assembler_child : boolean;
      var
        hp : tprocinfo;
      begin
        result:=false;
        hp:=get_first_nestedproc;
        while assigned(hp) do
          begin
            if (hp.flags*[pi_has_assembler_block,pi_is_assembler])<>[] then
              begin
                result:=true;
                exit;
              end;
            hp:=tprocinfo(hp.next);
          end;
>>>>>>> origin/cpstrnew
      end;


    procedure tcgprocinfo.generate_code;
      var
        old_current_procinfo : tprocinfo;
        oldmaxfpuregisters : longint;
        oldfilepos : tfileposinfo;
        old_current_structdef : tabstractrecorddef;
        templist : TAsmList;
        headertai : tai;
        i : integer;
        {RedoDFA : boolean;}

        procedure delete_marker(anode: tasmnode);
          var
            ai: tai;
          begin
            if assigned(anode) then
              begin
                ai:=anode.currenttai;
                if assigned(ai) then
                  begin
                    aktproccode.remove(ai);
                    ai.free;
                    anode.currenttai:=nil;
                  end;
              end;
          end;

      begin
        { the initialization procedure can be empty, then we
          don't need to generate anything. When it was an empty
          procedure there would be at least a blocknode }
        if not assigned(code) then
          exit;

        { We need valid code }
        if Errorcount<>0 then
          exit;

        { No code can be generated for generic template }
        if (df_generic in procdef.defoptions) then
          internalerror(200511152);

        { For regular procedures the RA and Tempgen shall not be available yet,
          but exception filters reuse Tempgen of parent }
        if assigned(tg)<>(procdef.proctypeoption=potype_exceptfilter) then
          internalerror(200309201);

        old_current_procinfo:=current_procinfo;
        oldfilepos:=current_filepos;
        old_current_structdef:=current_structdef;
        oldmaxfpuregisters:=current_settings.maxfpuregisters;

        current_procinfo:=self;
        current_filepos:=entrypos;
        current_structdef:=procdef.struct;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

        { store start of user code, it must be a block node, it will be used later one to
          check variable lifeness }
        include(code.flags,nf_usercode_entry);

        { add wrapping code if necessary (initialization of typed constants on
          some platforms, initing of local variables and out parameters with
          trashing values, ...) }
        { init/final code must be wrapped later (after code for main proc body
          has been generated) }
        if not(current_procinfo.procdef.proctypeoption in [potype_unitinit,potype_unitfinalize]) then
          code:=cnodeutils.wrap_proc_body(procdef,code);

        { automatic inlining? }
        if (cs_opt_autoinline in current_settings.optimizerswitches) and
           { inlining not turned off? }
           (cs_do_inline in current_settings.localswitches) and
           { no inlining yet? }
           not(procdef.has_inlininginfo) and not(has_nestedprocs) and
            not(procdef.proctypeoption in [potype_proginit,potype_unitinit,potype_unitfinalize,potype_constructor,
                                           potype_destructor,potype_class_constructor,potype_class_destructor]) and
            ((procdef.procoptions*[po_exports,po_external,po_interrupt,po_virtualmethod,po_iocheck])=[]) and
            (not(procdef.proccalloption in [pocall_safecall])) and
            { rough approximation if we should auto inline }
            (node_count(code)<=10) then
          begin
            { Can we inline this procedure? }
            if checknodeinlining(procdef) then
              begin
                Message1(cg_d_autoinlining,procdef.GetTypeName);
                include(procdef.procoptions,po_inline);
                CreateInlineInfo;
              end;
          end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

        templist:=TAsmList.create;

        { add parast/localst to symtablestack }
        add_to_symtablestack;

        { clear register count }
        procdef.localst.SymList.ForEachCall(@clearrefs,nil);
        procdef.parast.SymList.ForEachCall(@clearrefs,nil);

        { there's always a call to FPC_INITIALIZEUNITS/FPC_DO_EXIT in the main program }
        if (procdef.localst.symtablelevel=main_program_level) and
           (not current_module.is_unit) then
          begin
            include(flags,pi_do_call);
            { the main program never returns due to the do_exit call }
            if not(current_module.islibrary) then
              include(procdef.procoptions,po_noreturn);
          end;

        { set implicit_finally flag when there are locals/paras to be finalized }
        if not(po_assembler in current_procinfo.procdef.procoptions) then
          begin
            procdef.parast.SymList.ForEachCall(@check_finalize_paras,nil);
            procdef.localst.SymList.ForEachCall(@check_finalize_locals,nil);
          end;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef SUPPORT_SAFECALL}
        { set implicit_finally flag for if procedure is safecall }
        if (tf_safecall_exceptions in target_info.flags) and
           (procdef.proccalloption=pocall_safecall) then
=======
{$if defined(x86) or defined(arm)}
        { set implicit_finally flag for if procedure is safecall }
        if procdef.proccalloption=pocall_safecall then
>>>>>>> graemeg/fixes_2_2
=======
{$if defined(x86) or defined(arm)}
        { set implicit_finally flag for if procedure is safecall }
        if procdef.proccalloption=pocall_safecall then
>>>>>>> origin/fixes_2_2
          include(flags, pi_needs_implicit_finally);
{$endif}
        { firstpass everything }
        flowcontrol:=[];
        do_firstpass(code);

{$if defined(i386) or defined(i8086)}
        if node_resources_fpu(code)>0 then
          include(flags,pi_uses_fpu);
{$endif i386 or i8086}

        { Print the node to tree.log }
        if paraprintnodetree=1 then
          printproc( 'after the firstpass');

        { Print the node to tree.log }
        if paraprintnodetree=1 then
          printproc( 'after the firstpass');

        { Print the node to tree.log }
        if paraprintnodetree=1 then
          printproc( 'after the firstpass');

        { Print the node to tree.log }
        if paraprintnodetree=1 then
          printproc( 'after the firstpass');

        { Print the node to tree.log }
        if paraprintnodetree=1 then
          printproc( 'after the firstpass');

        { do this before adding the entry code else the tail recursion recognition won't work,
          if this causes troubles, it must be if'ed
        }
        if (cs_opt_tailrecursion in current_settings.optimizerswitches) and
          (pi_is_recursive in flags) then
          do_opttail(code,procdef);

        if cs_opt_constant_propagate in current_settings.optimizerswitches then
          do_optconstpropagate(code);

        if (cs_opt_nodedfa in current_settings.optimizerswitches) and
          { creating dfa is not always possible }
          ((flags*[pi_has_assembler_block,pi_uses_exceptions,pi_is_assembler])=[]) then
          begin
            dfabuilder:=TDFABuilder.Create;
            dfabuilder.createdfainfo(code);
<<<<<<< HEAD
=======
            { when life info is available, we can give more sophisticated warning about unintialized
              variables }

            { iterate through life info of the first node }
            for i:=0 to dfabuilder.nodemap.count-1 do
              begin
                if DFASetIn(code.optinfo^.life,i) then
                  case tnode(dfabuilder.nodemap[i]).nodetype of
                    loadn:
                      begin
                        varsym:=tabstractnormalvarsym(tloadnode(dfabuilder.nodemap[i]).symtableentry);

                        { Give warning/note for living locals }
                        if assigned(varsym.owner) and
                          not(vo_is_external in varsym.varoptions) then
                          begin
                            if (vo_is_funcret in varsym.varoptions) then
                              CGMessage(sym_w_function_result_uninitialized)
                            else
                              begin
                                if (varsym.owner=procdef.localst) and not (vo_is_typed_const in varsym.varoptions) then
                                  CGMessage1(sym_w_uninitialized_local_variable,varsym.realname);
                              end;
                          end;
                      end;
                  end;
              end;
>>>>>>> graemeg/cpstrnew
            include(flags,pi_dfaavailable);

            { when life info is available, we can give more sophisticated warning about uninitialized
              variables ...
              ... but not for the finalization section of a unit, we would need global dfa to handle
              it properly }
            if potype_unitfinalize<>procdef.proctypeoption then
              { iterate through life info of the first node }
              for i:=0 to dfabuilder.nodemap.count-1 do
                begin
                  if DFASetIn(GetUserCode.optinfo^.life,i) then
                    begin
                      { do not warn for certain parameters: }
                      if not((tnode(dfabuilder.nodemap[i]).nodetype=loadn) and (tloadnode(dfabuilder.nodemap[i]).symtableentry.typ=paravarsym) and
                        { do not warn about parameters passed by var }
                        (((tparavarsym(tloadnode(dfabuilder.nodemap[i]).symtableentry).varspez=vs_var) and
                        { function result is passed by var but it must be initialized }
                        not(vo_is_funcret in tparavarsym(tloadnode(dfabuilder.nodemap[i]).symtableentry).varoptions)) or
                        { do not warn about initialized hidden parameters }
                        ((tparavarsym(tloadnode(dfabuilder.nodemap[i]).symtableentry).varoptions*[vo_is_high_para,vo_is_parentfp,vo_is_result,vo_is_self])<>[]))) then
                        CheckAndWarn(GetUserCode,tnode(dfabuilder.nodemap[i]));
                    end
                  else
                    begin
                      if (tnode(dfabuilder.nodemap[i]).nodetype=loadn) and
                        (tloadnode(dfabuilder.nodemap[i]).symtableentry.typ in [staticvarsym,localvarsym]) then
                        tabstractnormalvarsym(tloadnode(dfabuilder.nodemap[i]).symtableentry).noregvarinitneeded:=true
                    end;
                end;
          end;

        if (pi_dfaavailable in flags) and (cs_opt_dead_store_eliminate in current_settings.optimizerswitches) then
          do_optdeadstoreelim(code);

        if (cs_opt_loopstrength in current_settings.optimizerswitches)
          { our induction variable strength reduction doesn't like
            for loops with more than one entry }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          and not(pi_has_label in flags) then
=======
          and not(pi_has_label in current_procinfo.flags) then
>>>>>>> graemeg/cpstrnew
=======
          and not(pi_has_label in current_procinfo.flags) then
>>>>>>> graemeg/cpstrnew
=======
          and not(pi_has_label in current_procinfo.flags) then
>>>>>>> graemeg/cpstrnew
=======
          and not(pi_has_label in current_procinfo.flags) then
>>>>>>> origin/cpstrnew
          begin
            {RedoDFA:=}OptimizeInductionVariables(code);
          end;

        if (cs_opt_remove_emtpy_proc in current_settings.optimizerswitches) and
          (procdef.proctypeoption in [potype_operator,potype_procedure,potype_function]) and
          (code.nodetype=blockn) and (tblocknode(code).statements=nil) then
          procdef.isempty:=true;

        { add implicit entry and exit code }
        add_entry_exit_code;

        if cs_opt_nodecse in current_settings.optimizerswitches then
          do_optcse(code);

        { only do secondpass if there are no errors }
        if (ErrorCount=0) then
          begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            create_hlcodegen;

            if (procdef.proctypeoption<>potype_exceptfilter) then
              setup_tempgen;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
            create_codegen;

            { set the start offset to the start of the temp area in the stack }
            tg:=ttgobj.create;

{$if defined(x86) or defined(arm)}
            { try to strip the stack frame }
            { set the framepointer to esp if:
              - no assembler directive, those are handled in assembler_block
                in pstatment.pas (for cases not caught by the Delphi
                exception below)
              - no exceptions are used
              - no pushes are used/esp modifications, could be:
                * outgoing parameters on the stack
                * incoming parameters on the stack
                * open arrays
              - no inline assembler
             or
              - Delphi mode
              - assembler directive
              - no pushes are used/esp modifications, could be:
                * outgoing parameters on the stack
                * incoming parameters on the stack
                * open arrays
              - no local variables
            }
            if ((po_assembler in procdef.procoptions) and
                (m_delphi in current_settings.modeswitches) and
                { localst at main_program_level is a staticsymtable }
                (procdef.localst.symtablelevel<>main_program_level) and
                (tabstractlocalsymtable(procdef.localst).count_locals = 0)) or
               ((cs_opt_stackframe in current_settings.optimizerswitches) and
                not(cs_generate_stackframes in current_settings.localswitches) and
                not(po_assembler in procdef.procoptions) and
                ((flags*[pi_has_assembler_block,pi_uses_exceptions,pi_is_assembler,
                        pi_needs_implicit_finally,pi_has_implicit_finally,pi_has_stackparameter,
                        pi_needs_stackframe])=[])
               )
             then
               begin
                 { we need the parameter info here to determine if the procedure gets
                   parameters on the stack

                   calling generate_parameter_info doesn't hurt but it costs time
                   (necessary to init para_stack_size)
                 }
                 generate_parameter_info;
                 if not(procdef.stack_tainting_parameter(calleeside)) and
                   not(has_assembler_child) and (para_stack_size=0) then
                   begin
                     { Only need to set the framepointer }
                     framepointer:=NR_STACK_POINTER_REG;
                     tg.direction:=1;
                   end;
               end;
>>>>>>> graemeg/cpstrnew

            { Create register allocator, must come after framepointer is known }
            hlcg.init_register_allocators;

            generate_parameter_info;

            { allocate got register if needed }
<<<<<<< HEAD
<<<<<<< HEAD
            allocate_got_register(aktproccode);
=======
            current_procinfo.allocate_got_register(aktproccode);
>>>>>>> graemeg/fixes_2_2
=======
            current_procinfo.allocate_got_register(aktproccode);
>>>>>>> origin/fixes_2_2

            { Allocate space in temp/registers for parast and localst }
            current_filepos:=entrypos;
            gen_alloc_symtable(aktproccode,procdef,procdef.parast);
            gen_alloc_symtable(aktproccode,procdef,procdef.localst);

            { Store temp offset for information about 'real' temps }
            tempstart:=tg.lasttemp;

            { Generate code to load register parameters in temps and insert local
              copies for values parameters. This must be done before the code for the
              body is generated because the localloc is updated.
              Note: The generated code will be inserted after the code generation of
              the body is finished, because only then the position is known }
{$ifdef oldregvars}
            assign_regvars(code);
{$endif oldreg}
            current_filepos:=entrypos;
<<<<<<< HEAD

<<<<<<< HEAD
            hlcg.gen_load_para_value(templist);
=======
            gen_load_para_value(templist);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
            { record which registers are allocated here, since all code }
            { allocating registers comes after it                       }
            cg.set_regalloc_live_range_direction(rad_backwards);

            gen_load_para_value(templist);
            cg.set_regalloc_live_range_direction(rad_forward);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

            { caller paraloc info is also necessary in the stackframe_entry
              code of the ppc (and possibly other processors)               }
            procdef.init_paraloc_info(callerside);

            { generate code for the node tree }
            do_secondpass(code);
            aktproccode.concatlist(current_asmdata.CurrAsmList);

            { The position of the loadpara_asmnode is now known }
            aktproccode.insertlistafter(loadpara_asmnode.currenttai,templist);

            { first generate entry and initialize code with the correct
              position and switches }
            current_filepos:=entrypos;
            current_settings.localswitches:=entryswitches;

            cg.set_regalloc_live_range_direction(rad_backwards);

            hlcg.gen_entry_code(templist);
            aktproccode.insertlistafter(entry_asmnode.currenttai,templist);
            hlcg.gen_initialize_code(templist);
            aktproccode.insertlistafter(init_asmnode.currenttai,templist);

            { now generate finalize and exit code with the correct position
              and switches }
            current_filepos:=exitpos;
            current_settings.localswitches:=exitswitches;

            cg.set_regalloc_live_range_direction(rad_forward);

            if assigned(finalize_procinfo) then
              generate_exceptfilter(tcgprocinfo(finalize_procinfo))
            else
              begin
                hlcg.gen_finalize_code(templist);
                { the finalcode must be concated if there was no position available,
                  using insertlistafter will result in an insert at the start
                  when currentai=nil }
                if assigned(final_asmnode) and assigned(final_asmnode.currenttai) then
                  aktproccode.insertlistafter(final_asmnode.currenttai,templist)
                else
                  aktproccode.concatlist(templist);
              end;
            { insert exit label at the correct position }
            hlcg.a_label(templist,CurrExitLabel);
            if assigned(exitlabel_asmnode.currenttai) then
              aktproccode.insertlistafter(exitlabel_asmnode.currenttai,templist)
            else
              aktproccode.concatlist(templist);
            { exit code }
            hlcg.gen_exit_code(templist);
            aktproccode.concatlist(templist);

{$ifdef OLDREGVARS}
            { note: this must be done only after as much code as possible has  }
            {   been generated. The result is that when you ungetregister() a  }
            {   regvar, it will actually free the regvar (and alse free the    }
            {   the regvars at the same time). Doing this too early will       }
            {   confuse the register allocator, as the regvars will still be   }
            {   used. It should be done before loading the result regs (so     }
            {   they don't conflict with the regvars) and before               }
            {   gen_entry_code (that one has to be able to allocate the        }
            {   regvars again) (JM)                                            }
            free_regvars(aktproccode);
{$endif OLDREGVARS}

            { generate symbol and save end of header position }
            current_filepos:=entrypos;
            hlcg.gen_proc_symbol(templist);
            headertai:=tai(templist.last);
            { insert symbol }
            aktproccode.insertlist(templist);

            { Free space in temp/registers for parast and localst, must be
              done after gen_entry_code }
            current_filepos:=exitpos;

            { make sure the got/pic register doesn't get freed in the }
            { middle of a loop                                        }
            if (cs_create_pic in current_settings.moduleswitches) and
<<<<<<< HEAD
<<<<<<< HEAD
               (pi_needs_got in flags) and
               (got<>NR_NO) then
              cg.a_reg_sync(aktproccode,got);
=======
               (pi_needs_got in current_procinfo.flags) and
               (current_procinfo.got<>NR_NO) then
              cg.a_reg_sync(aktproccode,current_procinfo.got);
>>>>>>> graemeg/fixes_2_2
=======
               (pi_needs_got in current_procinfo.flags) and
               (current_procinfo.got<>NR_NO) then
              cg.a_reg_sync(aktproccode,current_procinfo.got);
>>>>>>> origin/fixes_2_2

            gen_free_symtable(aktproccode,procdef.localst);
            gen_free_symtable(aktproccode,procdef.parast);

            { add code that will load the return value, this is not done
              for assembler routines when they didn't reference the result
              variable }
            hlcg.gen_load_return_value(templist);
            aktproccode.concatlist(templist);

            { Already reserve all registers for stack checking code and
              generate the call to the helper function }
            if not(tf_no_generic_stackcheck in target_info.flags) and
               (cs_check_stack in entryswitches) and
               not(po_assembler in procdef.procoptions) and
               (procdef.proctypeoption<>potype_proginit) then
              begin
                current_filepos:=entrypos;
                gen_stack_check_call(templist);
                aktproccode.insertlistafter(stackcheck_asmnode.currenttai,templist)
              end;

            { this code (got loading) comes before everything which has }
            { already been generated, so reset the info about already   }
            { backwards extended registers (so their live range can be  }
            { extended backwards even further if needed)                }
            { This code must be                                         }
            {  a) generated after do_secondpass has been called         }
            {     (because pi_needs_got may be set there)               }
            {  b) generated before register allocation, because the     }
            {     got/pic register can be a virtual one                 }
            {  c) inserted before the entry code, because the entry     }
            {     code may need global symbols such as init rtti        }
            {  d) inserted after the stackframe allocation, because     }
            {     this register may have to be spilled                  }
            cg.set_regalloc_live_range_direction(rad_backwards_reinit);
            current_filepos:=entrypos;
            { load got if necessary }
            cg.g_maybe_got_init(templist);

            aktproccode.insertlistafter(headertai,templist);

<<<<<<< HEAD
<<<<<<< HEAD
            { re-enable if more code at the end is ever generated here
            cg.set_regalloc_live_range_direction(rad_forward);
            }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{$ifndef NoOpt}
{$ifndef i386}
            if (cs_opt_scheduler in current_settings.optimizerswitches) and
              { do not optimize pure assembler procedures }
              not(pi_is_assembler in flags) then
              preregallocschedule(aktproccode);
{$endif i386}
{$endif NoOpt}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
            cg.set_regalloc_live_range_direction(rad_forward);
>>>>>>> graemeg/fixes_2_2
=======
            cg.set_regalloc_live_range_direction(rad_forward);
>>>>>>> origin/fixes_2_2

            { The procedure body is finished, we can now
              allocate the registers }
            cg.do_register_allocation(aktproccode,headertai);

            { translate imag. register to their real counter parts
              this is necessary for debuginfo and verbose assembler output
              when SSA will be implented, this will be more complicated because we've to
              maintain location lists }
<<<<<<< HEAD
            procdef.parast.SymList.ForEachCall(@translate_registers,templist);
            procdef.localst.SymList.ForEachCall(@translate_registers,templist);
            if (cs_create_pic in current_settings.moduleswitches) and
               (pi_needs_got in flags) and
               not(cs_no_regalloc in current_settings.globalswitches) and
               (got<>NR_NO) then
              cg.translate_register(got);
=======
            current_procinfo.procdef.parast.SymList.ForEachCall(@translate_registers,templist);
            current_procinfo.procdef.localst.SymList.ForEachCall(@translate_registers,templist);
            if (cs_create_pic in current_settings.moduleswitches) and
               (pi_needs_got in current_procinfo.flags) and
               not(cs_no_regalloc in current_settings.globalswitches) and
               (current_procinfo.got<>NR_NO) then
              cg.translate_register(current_procinfo.got);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

            { Add save and restore of used registers }
            current_filepos:=entrypos;
            gen_save_used_regs(templist);
            { Remember the last instruction of register saving block
              (may be =nil for e.g. assembler procedures) }
            endprologue_ai:=templist.last;
            aktproccode.insertlistafter(headertai,templist);
            current_filepos:=exitpos;
            gen_restore_used_regs(aktproccode);
            { We know the size of the stack, now we can generate the
              parameter that is passed to the stack checking code }
            if not(tf_no_generic_stackcheck in target_info.flags) and
               (cs_check_stack in entryswitches) and
               not(po_assembler in procdef.procoptions) and
               (procdef.proctypeoption<>potype_proginit) then
              begin
                current_filepos:=entrypos;
                gen_stack_check_size_para(templist);
                aktproccode.insertlistafter(stackcheck_asmnode.currenttai,templist)
              end;
            { Add entry code (stack allocation) after header }
            current_filepos:=entrypos;
            gen_proc_entry_code(templist);
            aktproccode.insertlistafter(headertai,templist);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef SUPPORT_SAFECALL}
            { Set return value of safecall procedure if implicit try/finally blocks are disabled }
            if not (cs_implicit_exceptions in current_settings.moduleswitches) and
               (tf_safecall_exceptions in target_info.flags) and
=======
{$if defined(x86) or defined(arm)}
            { Set return value of safecall procedure if implicit try/finally blocks are disabled }
            if not (cs_implicit_exceptions in current_settings.moduleswitches) and
>>>>>>> graemeg/fixes_2_2
=======
{$if defined(x86) or defined(arm)}
            { Set return value of safecall procedure if implicit try/finally blocks are disabled }
            if not (cs_implicit_exceptions in current_settings.moduleswitches) and
>>>>>>> origin/fixes_2_2
               (procdef.proccalloption=pocall_safecall) then
              cg.a_load_const_reg(aktproccode,OS_ADDR,0,NR_FUNCTION_RETURN_REG);
{$endif}
            { Add exit code at the end }
            current_filepos:=exitpos;
            gen_proc_exit_code(templist);
            aktproccode.concatlist(templist);

            { check if the implicit finally has been generated. The flag
              should already be set in pass1 }
            if (cs_implicit_exceptions in current_settings.moduleswitches) and
               not(procdef.proctypeoption in [potype_unitfinalize,potype_unitinit]) and
               (pi_needs_implicit_finally in flags) and
               not(pi_has_implicit_finally in flags) and
               not(target_info.system in systems_garbage_collected_managed_types) then
             internalerror(200405231);

            { Position markers are only used to insert additional code after the secondpass
              and before this point. They are of no use in optimizer. Instead of checking and
              ignoring all over the optimizer, just remove them here. }
            delete_marker(entry_asmnode);
            delete_marker(loadpara_asmnode);
            delete_marker(exitlabel_asmnode);
            delete_marker(stackcheck_asmnode);
            delete_marker(init_asmnode);
            delete_marker(final_asmnode);

{$ifndef NoOpt}
            if not(cs_no_regalloc in current_settings.globalswitches) then
              begin
                if (cs_opt_level1 in current_settings.optimizerswitches) and
                   { do not optimize pure assembler procedures }
                   not(pi_is_assembler in flags)  then
                  optimize(aktproccode);
{$ifndef i386}
                { schedule after assembler optimization, it could have brought up
                  new schedule possibilities }
                if (cs_opt_scheduler in current_settings.optimizerswitches) and
                  { do not optimize pure assembler procedures }
                  not(pi_is_assembler in flags)  then
                  preregallocschedule(aktproccode);
{$endif i386}
              end;
{$endif NoOpt}

<<<<<<< HEAD
            { Perform target-specific processing if necessary }
            postprocess_code;
=======

{$ifdef ARM}
            { because of the limited constant size of the arm, all data access is done pc relative }
            finalizearmcode(aktproccode,aktlocaldata);
{$endif ARM}
>>>>>>> graemeg/cpstrnew

            { Add end symbol and debug info }
            { this must be done after the pcrelativedata is appended else the distance calculation of
              insertpcrelativedata will be wrong, further the pc indirect data is part of the procedure
              so it should be inserted before the end symbol (FK)
            }
            current_filepos:=exitpos;
            hlcg.gen_proc_symbol_end(templist);
            aktproccode.concatlist(templist);

            { insert line debuginfo }
            if (cs_debuginfo in current_settings.moduleswitches) or
               (cs_use_lineinfo in current_settings.globalswitches) then
              current_debuginfo.insertlineinfo(aktproccode);

            hlcg.record_generated_code_for_procdef(current_procinfo.procdef,aktproccode,aktlocaldata);

            { only now we can remove the temps }
            if (procdef.proctypeoption<>potype_exceptfilter) then
              begin
                tg.resettempgen;
                tg.free;
                tg:=nil;
              end;
            { stop tempgen and ra }
<<<<<<< HEAD
            hlcg.done_register_allocators;
            destroy_hlcodegen;
=======
            tg.free;
            cg.done_register_allocators;
            destroy_codegen;
            tg:=nil;
>>>>>>> graemeg/cpstrnew
          end;

        dfabuilder.free;

        { restore symtablestack }
        remove_from_symtablestack;

        { restore }
        templist.free;
        current_settings.maxfpuregisters:=oldmaxfpuregisters;
        current_filepos:=oldfilepos;
        current_structdef:=old_current_structdef;
        current_procinfo:=old_current_procinfo;
      end;


    procedure tcgprocinfo.add_to_symtablestack;
      begin
        { insert symtables for the class, but only if it is no nested function }
        if assigned(procdef.struct) and
           not(assigned(parent) and
               assigned(parent.procdef) and
               assigned(parent.procdef.struct)) then
          push_nested_hierarchy(procdef.struct);

        { insert parasymtable in symtablestack when parsing
          a function }
        if procdef.parast.symtablelevel>=normal_function_level then
          symtablestack.push(procdef.parast);

        { insert localsymtable, except for the main procedure
          (in that case the localst is the unit's static symtable,
           which is already on the stack) }
        if procdef.localst.symtablelevel>=normal_function_level then
          symtablestack.push(procdef.localst);
      end;


    procedure tcgprocinfo.remove_from_symtablestack;
      begin
        { remove localsymtable }
        if procdef.localst.symtablelevel>=normal_function_level then
          symtablestack.pop(procdef.localst);

        { remove parasymtable }
        if procdef.parast.symtablelevel>=normal_function_level then
          symtablestack.pop(procdef.parast);

        { remove symtables for the class, but only if it is no nested function }
        if assigned(procdef.struct) and
           not(assigned(parent) and
               assigned(parent.procdef) and
               assigned(parent.procdef.struct)) then
          pop_nested_hierarchy(procdef.struct);
      end;


    procedure tcgprocinfo.resetprocdef;
      begin
         { remove code tree, if not inline procedure }
         if assigned(code) then
          begin
            { the inline procedure has already got a copy of the tree
              stored in procdef.inlininginfo }
            code.free;
            code:=nil;
          end;
       end;


<<<<<<< HEAD
=======
    function checknodeinlining(procdef: tprocdef): boolean;
      var
        i : integer;
        currpara : tparavarsym;
      begin
        result := false;
        if (pi_has_assembler_block in current_procinfo.flags) then
          begin
            Message1(parser_w_not_supported_for_inline,'assembler');
            Message(parser_w_inlining_disabled);
            exit;
          end;
        for i:=0 to procdef.paras.count-1 do
          begin
            currpara:=tparavarsym(procdef.paras[i]);
            case currpara.vardef.typ of
              formaldef :
                begin
<<<<<<< HEAD
<<<<<<< HEAD
                  if (currpara.varspez in [vs_out,vs_var,vs_const,vs_constref]) then
=======
                  if (currpara.varspez in [vs_out,vs_var,vs_const]) then
>>>>>>> graemeg/fixes_2_2
=======
                  if (currpara.varspez in [vs_out,vs_var,vs_const]) then
>>>>>>> origin/fixes_2_2
                    begin
                      Message1(parser_w_not_supported_for_inline,'formal parameter');
                      Message(parser_w_inlining_disabled);
                      exit;
                    end;
                end;
              arraydef :
                begin
                  if is_array_of_const(currpara.vardef) or
                     is_variant_array(currpara.vardef) then
                    begin
                      Message1(parser_w_not_supported_for_inline,'array of const');
                      Message(parser_w_inlining_disabled);
                      exit;
                    end;
                  { open arrays might need re-basing of the index, i.e. if you pass
                    an array[1..10] as open array, you have to add 1 to all index operations
                    if you directly inline it }
                  if is_open_array(currpara.vardef) then
                    begin
                      Message1(parser_w_not_supported_for_inline,'open array');
                      Message(parser_w_inlining_disabled);
                      exit;
                    end;
                end;
            end;
        end;
        result:=true;
      end;


>>>>>>> graemeg/cpstrnew
    procedure tcgprocinfo.parse_body;
      var
         old_current_procinfo : tprocinfo;
         old_block_type : tblock_type;
         st : TSymtable;
         old_current_structdef: tabstractrecorddef;
         old_current_genericdef,
         old_current_specializedef: tstoreddef;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         old_parse_generic: boolean;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin
         old_current_procinfo:=current_procinfo;
         old_block_type:=block_type;
         old_current_structdef:=current_structdef;
         old_current_genericdef:=current_genericdef;
         old_current_specializedef:=current_specializedef;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         old_parse_generic:=parse_generic;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

         current_procinfo:=self;
         current_structdef:=procdef.struct;
         if assigned(current_structdef) and (df_generic in current_structdef.defoptions) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           begin
             current_genericdef:=current_structdef;
             parse_generic:=true;
           end;
=======
           current_genericdef:=current_structdef;
>>>>>>> graemeg/cpstrnew
=======
           current_genericdef:=current_structdef;
>>>>>>> graemeg/cpstrnew
=======
           current_genericdef:=current_structdef;
>>>>>>> graemeg/cpstrnew
=======
           current_genericdef:=current_structdef;
>>>>>>> origin/cpstrnew
         if assigned(current_structdef) and (df_specialization in current_structdef.defoptions) then
           current_specializedef:=current_structdef;

         { calculate the lexical level }
         if procdef.parast.symtablelevel>maxnesting then
           Message(parser_e_too_much_lexlevel);
         block_type:=bt_body;

    {$ifdef state_tracking}
{    aktstate:=Tstate_storage.create;}
    {$endif state_tracking}

         { allocate the symbol for this procedure }
         alloc_proc_symbol(procdef);

         { add parast/localst to symtablestack }
         add_to_symtablestack;

         { save entry info }
         entrypos:=current_filepos;
         entryswitches:=current_settings.localswitches;

         if (df_generic in procdef.defoptions) then
           begin
             { start token recorder for generic template }
             procdef.initgeneric;
             current_scanner.startrecordtokens(procdef.generictokenbuf);
           end;

         { parse the code ... }
         code:=block(current_module.islibrary);

         if (df_generic in procdef.defoptions) then
           begin
             { stop token recorder for generic template }
             current_scanner.stoprecordtokens;

             { Give an error for accesses in the static symtable that aren't visible
               outside the current unit }
             st:=procdef.owner;
             while (st.symtabletype in [ObjectSymtable,recordsymtable]) do
               st:=st.defowner.owner;
             if (pi_uses_static_symtable in flags) and
                (st.symtabletype<>staticsymtable) then
<<<<<<< HEAD
<<<<<<< HEAD
               Message(parser_e_global_generic_references_static);
=======
               Comment(V_Error,'Global Generic template references static symtable');
>>>>>>> graemeg/fixes_2_2
=======
               Comment(V_Error,'Global Generic template references static symtable');
>>>>>>> origin/fixes_2_2
           end;

         { save exit info }
         exitswitches:=current_settings.localswitches;
         exitpos:=last_endtoken_filepos;

         { the procedure is now defined }
         procdef.forwarddef:=false;

         if assigned(code) then
           begin
             { get a better entry point }
             entrypos:=code.fileinfo;

             { Finish type checking pass }
             { type checking makes no sense in a generic definition }
             if not(df_generic in current_procinfo.procdef.defoptions) then
               do_typecheckpass(code);

             if assigned(procdef.parentfpinitblock) then
               begin
                 if assigned(tblocknode(procdef.parentfpinitblock).left) then
                   begin
                     { could be an asmn in case of a pure assembler procedure,
                       but those shouldn't access nested variables }
                     if code.nodetype<>blockn then
                       internalerror(2015122601);
                     tblocknode(code).left:=cstatementnode.create(procdef.parentfpinitblock,tblocknode(code).left);
                     do_typecheckpass(tblocknode(code).left);
                   end
                 else
                   procdef.parentfpinitblock.free;
                 procdef.parentfpinitblock:=nil;
               end;

           end;

         { Check for unused labels, forwards, symbols for procedures. Static
           symtable is checked in pmodules.
           The check must be done after the typecheckpass }
         if (Errorcount=0) and
            (tstoredsymtable(procdef.localst).symtabletype<>staticsymtable) then
           begin
             { check if forwards are resolved }
             tstoredsymtable(procdef.localst).check_forwards;
             { check if all labels are used }
             tstoredsymtable(procdef.localst).checklabels;
             { check for unused symbols, but only if there is no asm block }
             if not(pi_has_assembler_block in flags) then
               begin
                 tstoredsymtable(procdef.localst).allsymbolsused;
                 tstoredsymtable(procdef.parast).allsymbolsused;
               end;
           end;

         if (po_inline in procdef.procoptions) and
           { Can we inline this procedure? }
           checknodeinlining(procdef) then
           CreateInlineInfo;

         { Print the node to tree.log }
         if paraprintnodetree=1 then
           printproc( 'after parsing');

         { ... remove symbol tables }
         remove_from_symtablestack;

    {$ifdef state_tracking}
{    aktstate.destroy;}
    {$endif state_tracking}

         current_structdef:=old_current_structdef;
         current_genericdef:=old_current_genericdef;
         current_specializedef:=old_current_specializedef;
         current_procinfo:=old_current_procinfo;
         parse_generic:=old_parse_generic;

         { Restore old state }
         block_type:=old_block_type;
      end;


{****************************************************************************
                        PROCEDURE/FUNCTION PARSING
****************************************************************************}


    procedure check_init_paras(p:TObject;arg:pointer);
      begin
        if tsym(p).typ<>paravarsym then
         exit;
        with tparavarsym(p) do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          if (is_managed_type(vardef) and
             (varspez in [vs_value,vs_out])) or
             (is_shortstring(vardef) and
             (varspez=vs_value)) then
=======
          if is_managed_type(vardef) and
             (varspez in [vs_value,vs_out]) then
>>>>>>> graemeg/cpstrnew
=======
          if is_managed_type(vardef) and
             (varspez in [vs_value,vs_out]) then
>>>>>>> graemeg/cpstrnew
=======
          if is_managed_type(vardef) and
             (varspez in [vs_value,vs_out]) then
>>>>>>> graemeg/cpstrnew
=======
          if is_managed_type(vardef) and
             (varspez in [vs_value,vs_out]) then
>>>>>>> origin/cpstrnew
            include(current_procinfo.flags,pi_do_call);
      end;



    procedure read_proc_body(old_current_procinfo:tprocinfo;pd:tprocdef);
      {
        Parses the procedure directives, then parses the procedure body, then
        generates the code for it
      }

<<<<<<< HEAD
=======
      procedure do_generate_code(pi:tcgprocinfo);
        var
          hpi : tcgprocinfo;
        begin
          { generate code for this procedure }
          pi.generate_code;
          { process nested procs }
          hpi:=tcgprocinfo(pi.get_first_nestedproc);
          while assigned(hpi) do
           begin
             do_generate_code(hpi);
             hpi:=tcgprocinfo(hpi.next);
           end;
          pi.resetprocdef;
        end;

>>>>>>> graemeg/cpstrnew
      var
        oldfailtokenmode : tmodeswitches;
        isnestedproc     : boolean;
      begin
        Message1(parser_d_procedure_start,pd.fullprocname(false));
        oldfailtokenmode:=[];

        { create a new procedure }
        current_procinfo:=cprocinfo.create(old_current_procinfo);
        current_module.procinfo:=current_procinfo;
        current_procinfo.procdef:=pd;
        isnestedproc:=(current_procinfo.procdef.parast.symtablelevel>normal_function_level);

        { Insert mangledname }
        pd.aliasnames.insert(pd.mangledname);

        { Handle Export of this procedure }
        if (po_exports in pd.procoptions) and
           (target_info.system in [system_i386_os2,system_i386_emx]) then
          begin
            pd.aliasnames.insert(pd.procsym.realname);
            if cs_link_deffile in current_settings.globalswitches then
              deffile.AddExport(pd.mangledname);
          end;

        { Insert result variables in the localst }
        insert_funcret_local(pd);

        { check if there are para's which require initing -> set }
        { pi_do_call (if not yet set)                            }
        if not(pi_do_call in current_procinfo.flags) then
          pd.parast.SymList.ForEachCall(@check_init_paras,nil);

        { set _FAIL as keyword if constructor }
        if (pd.proctypeoption=potype_constructor) then
         begin
           oldfailtokenmode:=tokeninfo^[_FAIL].keyword;
           tokeninfo^[_FAIL].keyword:=alllanguagemodes;
         end;

        tcgprocinfo(current_procinfo).parse_body;

        { We can't support inlining for procedures that have nested
          procedures because the nested procedures use a fixed offset
          for accessing locals in the parent procedure (PFV) }
        if current_procinfo.has_nestedprocs then
          begin
            if (df_generic in current_procinfo.procdef.defoptions) then
              Comment(V_Error,'Generic methods cannot have nested procedures')
            else
             if (po_inline in current_procinfo.procdef.procoptions) then
              begin
                Message1(parser_h_not_supported_for_inline,'nested procedures');
                Message(parser_h_inlining_disabled);
                exclude(current_procinfo.procdef.procoptions,po_inline);
              end;
          end;

        { When it's a nested procedure then defer the code generation,
          when back at normal function level then generate the code
          for all defered nested procedures and the current procedure }
        if not isnestedproc then
          begin
<<<<<<< HEAD
=======
            { We can't support inlining for procedures that have nested
              procedures because the nested procedures use a fixed offset
              for accessing locals in the parent procedure (PFV) }
            if (tcgprocinfo(current_procinfo).nestedprocs.count>0) then
              begin
                if (df_generic in current_procinfo.procdef.defoptions) then
                  Comment(V_Error,'Generic methods cannot have nested procedures')
                else
                 if (po_inline in current_procinfo.procdef.procoptions) then
                  begin
                    Message1(parser_w_not_supported_for_inline,'nested procedures');
                    Message(parser_w_inlining_disabled);
                    exclude(current_procinfo.procdef.procoptions,po_inline);
                  end;
              end;
>>>>>>> graemeg/fixes_2_2
            if not(df_generic in current_procinfo.procdef.defoptions) then
              tcgprocinfo(current_procinfo).generate_code_tree;
          end;

        { reset _FAIL as _SELF normal }
        if (pd.proctypeoption=potype_constructor) then
          tokeninfo^[_FAIL].keyword:=oldfailtokenmode;

        { release procinfo }
        if tprocinfo(current_module.procinfo)<>current_procinfo then
          internalerror(200304274);
        current_module.procinfo:=current_procinfo.parent;

        { For specialization we didn't record the last semicolon. Moving this parsing
          into the parse_body routine is not done because of having better file position
          information available }
        if not(df_specialization in current_procinfo.procdef.defoptions) then
          consume(_SEMICOLON);

        if not isnestedproc then
          { current_procinfo is checked for nil later on }
          freeandnil(current_procinfo);
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure read_proc(isclassmethod:boolean; usefwpd: tprocdef;isgeneric:boolean);
=======
    procedure read_proc(isclassmethod:boolean);
>>>>>>> graemeg/cpstrnew
=======
    procedure read_proc(isclassmethod:boolean);
>>>>>>> graemeg/cpstrnew
=======
    procedure read_proc(isclassmethod:boolean);
>>>>>>> graemeg/cpstrnew
=======
    procedure read_proc(isclassmethod:boolean);
>>>>>>> origin/cpstrnew
      {
        Parses the procedure directives, then parses the procedure body, then
        generates the code for it
      }

      var
        old_current_procinfo : tprocinfo;
        old_current_structdef: tabstractrecorddef;
        old_current_genericdef,
        old_current_specializedef: tstoreddef;
        pdflags    : tpdflags;
        pd,firstpd : tprocdef;
      begin
         { save old state }
         old_current_procinfo:=current_procinfo;
         old_current_structdef:=current_structdef;
         old_current_genericdef:=current_genericdef;
         old_current_specializedef:=current_specializedef;

         { reset current_procinfo.procdef to nil to be sure that nothing is writing
           to another procdef }
         current_procinfo:=nil;
         current_structdef:=nil;
         current_genericdef:=nil;
         current_specializedef:=nil;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         if not assigned(usefwpd) then
           { parse procedure declaration }
           pd:=parse_proc_dec(isclassmethod,old_current_structdef,isgeneric)
         else
           pd:=usefwpd;
=======
         { parse procedure declaration }
         pd:=parse_proc_dec(isclassmethod,old_current_structdef);
>>>>>>> graemeg/cpstrnew
=======
         { parse procedure declaration }
         pd:=parse_proc_dec(isclassmethod,old_current_structdef);
>>>>>>> graemeg/cpstrnew
=======
         { parse procedure declaration }
         pd:=parse_proc_dec(isclassmethod,old_current_structdef);
>>>>>>> graemeg/cpstrnew
=======
         { parse procedure declaration }
         pd:=parse_proc_dec(isclassmethod,old_current_structdef);
>>>>>>> origin/cpstrnew

         { set the default function options }
         if parse_only then
          begin
            pd.forwarddef:=true;
            { set also the interface flag, for better error message when the
              implementation doesn't match this header }
            pd.interfacedef:=true;
            include(pd.procoptions,po_global);
            pdflags:=[pd_interface];
          end
         else
          begin
            pdflags:=[pd_body];
            if (not current_module.in_interface) then
              include(pdflags,pd_implemen);
            if (not current_module.is_unit) or
<<<<<<< HEAD
<<<<<<< HEAD
               create_smartlink_library then
=======
=======
>>>>>>> origin/fixes_2_2
               create_smartlink or
              {
                taking addresses of static procedures goes wrong
                if they aren't global when pic is used (FK)
              }
              (cs_create_pic in current_settings.moduleswitches) then
>>>>>>> graemeg/fixes_2_2
              include(pd.procoptions,po_global);
            pd.forwarddef:=false;
          end;

         if not assigned(usefwpd) then
           begin
             { parse the directives that may follow }
             parse_proc_directives(pd,pdflags);

<<<<<<< HEAD
             { hint directives, these can be separated by semicolons here,
               that needs to be handled here with a loop (PFV) }
             while try_consume_hintdirective(pd.symoptions,pd.deprecatedmsg) do
              Consume(_SEMICOLON);
=======
         { hint directives, these can be separated by semicolons here,
           that needs to be handled here with a loop (PFV) }
         while try_consume_hintdirective(pd.symoptions,pd.deprecatedmsg) do
          Consume(_SEMICOLON);
>>>>>>> graemeg/cpstrnew

             { Set calling convention }
             handle_calling_convention(pd);
           end;

         { search for forward declarations }
         if not proc_add_definition(pd) then
           begin
             { A method must be forward defined (in the object declaration) }
             if assigned(pd.struct) and
                (not assigned(old_current_structdef)) then
              begin
                MessagePos1(pd.fileinfo,parser_e_header_dont_match_any_member,pd.fullprocname(false));
                tprocsym(pd.procsym).write_parameter_lists(pd);
              end
             else
              begin
                { Give a better error if there is a forward def in the interface and only
                  a single implementation }
                firstpd:=tprocdef(tprocsym(pd.procsym).ProcdefList[0]);
                if (not pd.forwarddef) and
                   (not pd.interfacedef) and
                   (tprocsym(pd.procsym).ProcdefList.Count>1) and
                   firstpd.forwarddef and
                   firstpd.interfacedef and
                   not(tprocsym(pd.procsym).ProcdefList.Count>2) and
                   { don't give an error if it may be an overload }
                   not(m_fpc in current_settings.modeswitches) and
                   (not(po_overload in pd.procoptions) or
                    not(po_overload in firstpd.procoptions)) then
                 begin
                   MessagePos1(pd.fileinfo,parser_e_header_dont_match_forward,pd.fullprocname(false));
                   tprocsym(pd.procsym).write_parameter_lists(pd);
                 end;
              end;
           end;

         { Set mangled name }
         proc_set_mangledname(pd);

         { compile procedure when a body is needed }
         if (pd_body in pdflags) then
           begin
             read_proc_body(old_current_procinfo,pd);
           end
         else
           begin
             { Handle imports }
             if (po_external in pd.procoptions) then
               begin
<<<<<<< HEAD
=======
                 { External declared in implementation, and there was already a
                   forward (or interface) declaration then we need to generate
                   a stub that calls the external routine }
                 if (not pd.forwarddef) and
                    (pd.hasforward)
                    { it is unclear to me what's the use of the following condition,
                      so commented out, see also issue #18371 (FK)
                    and
                    not(
                        assigned(pd.import_dll) and
                        (target_info.system in [system_i386_wdosx,
                                                system_arm_wince,system_i386_wince])
                       ) } then
                   begin
                     s:=proc_get_importname(pd);
                     if s<>'' then
                       gen_external_stub(current_asmdata.asmlists[al_procedures],pd,s);
                   end;

>>>>>>> graemeg/cpstrnew
                 { Import DLL specified? }
                 if assigned(pd.import_dll) then
<<<<<<< HEAD
<<<<<<< HEAD
                   begin
                     if assigned (pd.import_name) then
                       current_module.AddExternalImport(pd.import_dll^,
                         pd.import_name^,proc_get_importname(pd),
                         pd.import_nr,false,false)
                     else
                       current_module.AddExternalImport(pd.import_dll^,
                         proc_get_importname(pd),proc_get_importname(pd),
                         pd.import_nr,false,true);
                   end
=======
                   current_module.AddExternalImport(pd.import_dll^,proc_get_importname(pd),pd.import_nr,false,pd.import_name=nil)
>>>>>>> graemeg/fixes_2_2
=======
                   current_module.AddExternalImport(pd.import_dll^,proc_get_importname(pd),pd.import_nr,false,pd.import_name=nil)
>>>>>>> origin/fixes_2_2
                 else
                   begin
                     { add import name to external list for DLL scanning }
                     if tf_has_dllscanner in target_info.flags then
                       current_module.dllscannerinputlist.Add(proc_get_importname(pd),pd);
                   end;
{$ifdef cpuhighleveltarget}
                 { it's hard to factor this out in a virtual method, because the
                   generic version (the one inside this ifdef) doesn't fit in
                   hlcgobj but in symcreat or here, while the other version
                   doesn't fit in symcreat (since it uses the code generator).
                   Maybe we need another class for this kind of code that could
                   either be symcreat- or hlcgobj-based
                 }
                 if (not pd.forwarddef) and
                    (pd.hasforward) and
                    (proc_get_importname(pd)<>'') then
                   call_through_new_name(pd,proc_get_importname(pd))
                 else
{$endif cpuhighleveltarget}
                   begin
                     create_hlcodegen;
                     hlcg.handle_external_proc(
                       current_asmdata.asmlists[al_procedures],
                       pd,
                       proc_get_importname(pd));
                     destroy_hlcodegen;
                   end
               end;
           end;

         { make sure that references to forward-declared functions are not }
         { treated as references to external symbols, needed for darwin.   }

         { make sure we don't change the binding of real external symbols }
         if ([po_external,po_weakexternal]*pd.procoptions)=[] then
           begin
             if (po_global in pd.procoptions) or
                (cs_profile in current_settings.moduleswitches) then
               current_asmdata.DefineAsmSymbol(pd.mangledname,AB_GLOBAL,AT_FUNCTION)
             else
               current_asmdata.DefineAsmSymbol(pd.mangledname,AB_LOCAL,AT_FUNCTION);
           end;

         current_structdef:=old_current_structdef;
         current_genericdef:=old_current_genericdef;
         current_specializedef:=old_current_specializedef;
         current_procinfo:=old_current_procinfo;
      end;


{****************************************************************************
                             DECLARATION PARSING
****************************************************************************}

    { search in symtablestack for not complete classes }
    procedure check_forward_class(p:TObject;arg:pointer);
      begin
        if (tsym(p).typ=typesym) and
           (ttypesym(p).typedef.typ=objectdef) and
           (oo_is_forward in tobjectdef(ttypesym(p).typedef).objectoptions) then
          MessagePos1(tsym(p).fileinfo,sym_e_forward_type_not_resolved,tsym(p).realname);
      end;


    procedure read_declarations(islibrary : boolean);
      var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        hadgeneric : boolean;

        procedure handle_unexpected_had_generic;
          begin
            if hadgeneric then
              begin
                Message(parser_e_procedure_or_function_expected);
                hadgeneric:=false;
              end;
          end;

      var
        is_classdef:boolean;
      begin
        is_classdef:=false;
        hadgeneric:=false;
=======
        is_classdef:boolean;
      begin
        is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
        is_classdef:boolean;
      begin
        is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
        is_classdef:boolean;
      begin
        is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
        is_classdef:boolean;
      begin
        is_classdef:=false;
>>>>>>> origin/cpstrnew
        repeat
           if not assigned(current_procinfo) then
             internalerror(200304251);
           case token of
              _LABEL:
                begin
                  handle_unexpected_had_generic;
                  label_dec;
                end;
              _CONST:
                begin
                  handle_unexpected_had_generic;
                  const_dec(hadgeneric);
                end;
              _TYPE:
                begin
                  handle_unexpected_had_generic;
                  type_dec(hadgeneric);
                end;
              _VAR:
                begin
                  handle_unexpected_had_generic;
                  var_dec(hadgeneric);
                end;
              _THREADVAR:
<<<<<<< HEAD
                begin
                  handle_unexpected_had_generic;
                  threadvar_dec(hadgeneric);
                end;
=======
                threadvar_dec;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              _CLASS:
                begin
                  is_classdef:=false;
                  if try_to_consume(_CLASS) then
                   begin
                     { class modifier is only allowed for procedures, functions, }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     { constructors, destructors                                 }
                     if not(token in [_FUNCTION,_PROCEDURE,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
=======
                     { constructors, destructors, fields and properties          }
                     if not(token in [_FUNCTION,_PROCEDURE,_PROPERTY,_VAR,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
>>>>>>> graemeg/cpstrnew
=======
                     { constructors, destructors, fields and properties          }
                     if not(token in [_FUNCTION,_PROCEDURE,_PROPERTY,_VAR,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
>>>>>>> graemeg/cpstrnew
=======
                     { constructors, destructors, fields and properties          }
                     if not(token in [_FUNCTION,_PROCEDURE,_PROPERTY,_VAR,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
>>>>>>> graemeg/cpstrnew
=======
                     { constructors, destructors, fields and properties          }
                     if not(token in [_FUNCTION,_PROCEDURE,_PROPERTY,_VAR,_CONSTRUCTOR,_DESTRUCTOR,_OPERATOR]) and
>>>>>>> origin/cpstrnew
                        not((token=_ID) and (idtoken=_OPERATOR)) then
                       Message(parser_e_procedure_or_function_expected);

                     if is_interface(current_structdef) then
                       Message(parser_e_no_static_method_in_interfaces)
                     else
                       { class methods are also allowed for Objective-C protocols }
                       is_classdef:=true;
                   end;
                end;
              _CONSTRUCTOR,
              _DESTRUCTOR,
              _FUNCTION,
              _PROCEDURE,
              _OPERATOR:
                begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  if hadgeneric and not (token in [_PROCEDURE,_FUNCTION]) then
                    begin
                      Message(parser_e_procedure_or_function_expected);
                      hadgeneric:=false;
                    end;
                  read_proc(is_classdef,nil,hadgeneric);
                  is_classdef:=false;
                  hadgeneric:=false;
=======
                  read_proc(is_classdef);
                  is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
                  read_proc(is_classdef);
                  is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
                  read_proc(is_classdef);
                  is_classdef:=false;
>>>>>>> graemeg/cpstrnew
=======
                  read_proc(is_classdef);
                  is_classdef:=false;
>>>>>>> origin/cpstrnew
                end;
              _EXPORTS:
                begin
                   handle_unexpected_had_generic;
                   if (current_procinfo.procdef.localst.symtablelevel>main_program_level) then
                     begin
                        Message(parser_e_syntax_error);
                        consume_all_until(_SEMICOLON);
                     end
                   else if islibrary or
                     (target_info.system in systems_unit_program_exports) then
                     read_exports
                   else
                     begin
                        Message(parser_w_unsupported_feature);
                        consume(_BEGIN);
                     end;
                end;
              _PROPERTY:
                begin
                  handle_unexpected_had_generic;
                  if (m_fpc in current_settings.modeswitches) then
                    property_dec
                  else
                    break;
                end;
              else
                begin
                  case idtoken of
                    _RESOURCESTRING:
                      begin
                        handle_unexpected_had_generic;
                        { m_class is needed, because the resourcestring
                          loading is in the ObjPas unit }
{                        if (m_class in current_settings.modeswitches) then}
                          resourcestring_dec(hadgeneric)
{                        else
                          break;}
                      end;
                    _OPERATOR:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                      begin
                        handle_unexpected_had_generic;
                        if is_classdef then
                          begin
                            read_proc(is_classdef,nil,false);
                            is_classdef:=false;
                          end
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                      begin
                        if is_classdef then
                          begin
                            read_proc(is_classdef);
                            is_classdef:=false;
                          end
                        else
                          break;
                      end;
                    _PROPERTY:
                      begin
                        if (m_fpc in current_settings.modeswitches) then
                        begin
                          property_dec(is_classdef);
                          is_classdef:=false;
                        end
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                        else
                          break;
                      end;
                    _GENERIC:
                      begin
                        handle_unexpected_had_generic;
                        if not (m_delphi in current_settings.modeswitches) then
                          begin
                            consume(_ID);
                            hadgeneric:=true;
                          end
                        else
                          break;
                      end
                    else
                      break;
                  end;
                end;
           end;
         until false;

         { add implementations for synthetic method declarations added by
           the compiler (not for unit/program init functions, their localst
           is the staticst -> would duplicate the work done in pmodules) }
         if current_procinfo.procdef.localst.symtabletype=localsymtable then
           add_synthetic_method_implementations(current_procinfo.procdef.localst);

         { check for incomplete class definitions, this is only required
           for fpc modes }
         if (m_fpc in current_settings.modeswitches) then
           current_procinfo.procdef.localst.SymList.ForEachCall(@check_forward_class,nil);
      end;


    procedure read_interface_declarations;
      var
        hadgeneric : boolean;

        procedure handle_unexpected_had_generic;
          begin
            if hadgeneric then
              begin
                Message(parser_e_procedure_or_function_expected);
                hadgeneric:=false;
              end;
          end;

      begin
         hadgeneric:=false;
         repeat
           case token of
             _CONST :
               begin
                 handle_unexpected_had_generic;
                 const_dec(hadgeneric);
               end;
             _TYPE :
               begin
                 handle_unexpected_had_generic;
                 type_dec(hadgeneric);
               end;
             _VAR :
               begin
                 handle_unexpected_had_generic;
                 var_dec(hadgeneric);
               end;
             _THREADVAR :
               begin
                 handle_unexpected_had_generic;
                 threadvar_dec(hadgeneric);
               end;
             _FUNCTION,
             _PROCEDURE,
             _OPERATOR :
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               begin
                 if hadgeneric and not (token in [_FUNCTION, _PROCEDURE]) then
                   begin
                     message(parser_e_procedure_or_function_expected);
                     hadgeneric:=false;
                   end;
                 read_proc(false,nil,hadgeneric);
                 hadgeneric:=false;
               end;
=======
               read_proc(false);
>>>>>>> graemeg/cpstrnew
=======
               read_proc(false);
>>>>>>> graemeg/cpstrnew
=======
               read_proc(false);
>>>>>>> graemeg/cpstrnew
=======
               read_proc(false);
>>>>>>> origin/cpstrnew
             else
               begin
                 case idtoken of
                   _RESOURCESTRING :
                     begin
                       handle_unexpected_had_generic;
                       resourcestring_dec(hadgeneric);
                     end;
                   _PROPERTY:
                     begin
                       handle_unexpected_had_generic;
                       if (m_fpc in current_settings.modeswitches) then
                         property_dec(false)
                       else
                         break;
                     end;
                   _GENERIC:
                     begin
                       handle_unexpected_had_generic;
                       if not (m_delphi in current_settings.modeswitches) then
                         begin
                           hadgeneric:=true;
                           consume(_ID);
                         end
                       else
                         break;
                     end
                   else
                     break;
                 end;
               end;
           end;
         until false;
         { check for incomplete class definitions, this is only required
           for fpc modes }
         if (m_fpc in current_settings.modeswitches) then
          symtablestack.top.SymList.ForEachCall(@check_forward_class,nil);
      end;


{****************************************************************************
                      SPECIALIZATION BODY GENERATION
****************************************************************************}


    procedure specialize_objectdefs(p:TObject;arg:pointer);
      var
<<<<<<< HEAD
        specobj : tabstractrecorddef;
        state : tspecializationstate;

        procedure process_procdef(def:tprocdef;hmodule:tmodule);
          var
            oldcurrent_filepos : tfileposinfo;
          begin
            if assigned(def.genericdef) and
                (def.genericdef.typ=procdef) and
                assigned(tprocdef(def.genericdef).generictokenbuf) then
              begin
                if not assigned(tprocdef(def.genericdef).generictokenbuf) then
                  internalerror(2015061902);
                oldcurrent_filepos:=current_filepos;
                current_filepos:=tprocdef(def.genericdef).fileinfo;
                { use the index the module got from the current compilation process }
                current_filepos.moduleindex:=hmodule.unit_index;
                current_tokenpos:=current_filepos;
                current_scanner.startreplaytokens(tprocdef(def.genericdef).generictokenbuf);
                read_proc_body(nil,def);
                current_filepos:=oldcurrent_filepos;
              end
            { synthetic routines will be implemented afterwards }
            else if def.synthetickind=tsk_none then
              MessagePos1(def.fileinfo,sym_e_forward_not_resolved,def.fullprocname(false));
          end;

      procedure process_abstractrecorddef(def:tabstractrecorddef);
        var
          i  : longint;
          hp : tdef;
          hmodule : tmodule;
        begin
          hmodule:=find_module_from_symtable(def.genericdef.owner);
          if hmodule=nil then
            internalerror(201202041);
          for i:=0 to def.symtable.DefList.Count-1 do
            begin
              hp:=tdef(def.symtable.DefList[i]);
              if hp.typ=procdef then
               begin
                 { only generate the code if we need a body }
                 if assigned(tprocdef(hp).struct) and not tprocdef(hp).forwarddef then
                   continue;
                 process_procdef(tprocdef(hp),hmodule);
               end
             else
               if hp.typ in [objectdef,recorddef] then
                 { generate code for subtypes as well }
                 process_abstractrecorddef(tabstractrecorddef(hp));
           end;
        end;

      procedure process_procsym(procsym:tprocsym);
        var
          i : longint;
          pd : tprocdef;
          state : tspecializationstate;
          hmodule : tmodule;
        begin
          for i:=0 to procsym.procdeflist.count-1 do
            begin
              pd:=tprocdef(procsym.procdeflist[i]);
              if not pd.is_specialization then
                continue;
              if not pd.forwarddef then
                continue;
              if not assigned(pd.genericdef) then
                internalerror(2015061903);
              hmodule:=find_module_from_symtable(pd.genericdef.owner);
              if hmodule=nil then
                internalerror(2015061904);

              specialization_init(pd.genericdef,state);

              process_procdef(pd,hmodule);

              specialization_done(state);
            end;
        end;

      begin
        if not((tsym(p).typ=typesym) and
               (ttypesym(p).typedef.typesym=tsym(p)) and
               (ttypesym(p).typedef.typ in [objectdef,recorddef])
              ) and
            not (tsym(p).typ=procsym) then
          exit;

        if tsym(p).typ=procsym then
          process_procsym(tprocsym(p))
        else
          if df_specialization in ttypesym(p).typedef.defoptions then
            begin
              { Setup symtablestack a definition time }
              specobj:=tabstractrecorddef(ttypesym(p).typedef);

              if not (is_class_or_object(specobj) or is_record(specobj) or is_javaclass(specobj)) then
                exit;

              specialization_init(specobj.genericdef,state);
=======
        i  : longint;
        hp : tdef;
        oldcurrent_filepos : tfileposinfo;
        oldsymtablestack   : tsymtablestack;
        pu : tused_unit;
        hmodule : tmodule;
<<<<<<< HEAD
<<<<<<< HEAD
        specobj : tabstractrecorddef;
      begin
        if not((tsym(p).typ=typesym) and
               (ttypesym(p).typedef.typesym=tsym(p)) and
               (ttypesym(p).typedef.typ in [objectdef,recorddef]) and
=======
=======
>>>>>>> origin/fixes_2_2
        specobj : tobjectdef;
      begin
        if not((tsym(p).typ=typesym) and
               (ttypesym(p).typedef.typesym=tsym(p)) and
               (ttypesym(p).typedef.typ=objectdef) and
>>>>>>> graemeg/fixes_2_2
               (df_specialization in ttypesym(p).typedef.defoptions)
              ) then
          exit;

        { Setup symtablestack a definition time }
<<<<<<< HEAD
<<<<<<< HEAD
        specobj:=tabstractrecorddef(ttypesym(p).typedef);
        oldsymtablestack:=symtablestack;
        symtablestack:=tsymtablestack.create;
        if not assigned(specobj.genericdef) then
=======
=======
>>>>>>> origin/fixes_2_2
        specobj:=tobjectdef(ttypesym(p).typedef);
        oldsymtablestack:=symtablestack;
        symtablestack:=tsymtablestack.create;
        if not assigned(tobjectdef(ttypesym(p).typedef).genericdef) then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          internalerror(200705151);
        hmodule:=find_module_from_symtable(specobj.genericdef.owner);
        if hmodule=nil then
          internalerror(200705152);
        pu:=tused_unit(hmodule.used_units.first);
        while assigned(pu) do
          begin
            if not assigned(pu.u.globalsymtable) then
              internalerror(200705153);
            symtablestack.push(pu.u.globalsymtable);
            pu:=tused_unit(pu.next);
          end;
        if assigned(hmodule.globalsymtable) then
          symtablestack.push(hmodule.globalsymtable);
        if assigned(hmodule.localsymtable) then
          symtablestack.push(hmodule.localsymtable);

        { procedure definitions for classes or objects }
<<<<<<< HEAD
<<<<<<< HEAD
        if is_class_or_object(specobj) or is_record(specobj) then
=======
        if is_class(specobj) or is_object(specobj) then
>>>>>>> graemeg/fixes_2_2
=======
        if is_class(specobj) or is_object(specobj) then
>>>>>>> origin/fixes_2_2
          begin
            for i:=0 to specobj.symtable.DefList.Count-1 do
              begin
                hp:=tdef(specobj.symtable.DefList[i]);
                if hp.typ=procdef then
                 begin
                   if assigned(tprocdef(hp).genericdef) and
                     (tprocdef(hp).genericdef.typ=procdef) and
                     assigned(tprocdef(tprocdef(hp).genericdef).generictokenbuf) then
                     begin
                       oldcurrent_filepos:=current_filepos;
                       current_filepos:=tprocdef(tprocdef(hp).genericdef).fileinfo;
                       { use the index the module got from the current compilation process }
                       current_filepos.moduleindex:=hmodule.unit_index;
                       current_tokenpos:=current_filepos;
                       current_scanner.startreplaytokens(tprocdef(tprocdef(hp).genericdef).generictokenbuf);
                       read_proc_body(nil,tprocdef(hp));
                       current_filepos:=oldcurrent_filepos;
                     end
                   else
                     MessagePos1(tprocdef(hp).fileinfo,sym_e_forward_not_resolved,tprocdef(hp).fullprocname(false));
                 end;
             end;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

              { procedure definitions for classes or objects }
              process_abstractrecorddef(specobj);

              specialization_done(state);
            end
          else
            tabstractrecorddef(ttypesym(p).typedef).symtable.symlist.whileeachcall(@specialize_objectdefs,nil);
=======
=======
>>>>>>> origin/fixes_2_2

        { Restore symtablestack }
        symtablestack.free;
        symtablestack:=oldsymtablestack;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;


    procedure generate_specialization_procs;
      begin
        if assigned(current_module.globalsymtable) then
          current_module.globalsymtable.SymList.WhileEachCall(@specialize_objectdefs,nil);
        if assigned(current_module.localsymtable) then
<<<<<<< HEAD
<<<<<<< HEAD
          current_module.localsymtable.SymList.WhileEachCall(@specialize_objectdefs,nil);
=======
          current_module.localsymtable.SymList.ForEachCall(@specialize_objectdefs,nil);
>>>>>>> graemeg/fixes_2_2
=======
          current_module.localsymtable.SymList.ForEachCall(@specialize_objectdefs,nil);
>>>>>>> origin/fixes_2_2
      end;

end.
