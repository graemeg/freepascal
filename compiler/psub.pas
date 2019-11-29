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

{ $define debug_eh}

interface

    uses
      globals,
      node,nbas,
      symdef,procinfo,optdfa;

    type
      tcgprocinfo = class(tprocinfo)
      private
        procedure CreateInlineInfo;
        { returns the node which is the start of the user code, this is needed by the dfa }
        function GetUserCode: tnode;
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
        init_asmnode    : tasmnode;
        temps_finalized : boolean;
        dfabuilder : TDFABuilder;

        destructor  destroy;override;

        function calc_stackframe_size : longint;override;

        procedure printproc(pass:string);
        procedure generate_code;
        procedure generate_code_tree;
        procedure generate_exceptfilter(nestedpi: tcgprocinfo);
        procedure resetprocdef;
        procedure add_to_symtablestack;
        procedure remove_from_symtablestack;
        procedure parse_body;

        function has_assembler_child : boolean;
        procedure set_eh_info; override;
{$ifdef DEBUG_NODE_XML}
        procedure XMLPrintProc;
{$endif DEBUG_NODE_XML}
      end;


    procedure printnode_reset;

{$ifdef DEBUG_NODE_XML}
    procedure XMLInitializeNodeFile(RootName, ModuleName: shortstring);
    procedure XMLFinalizeNodeFile(RootName: shortstring);
{$endif DEBUG_NODE_XML}
    { reads the declaration blocks }
    procedure read_declarations(islibrary : boolean);

    { reads declarations in the interface part of a unit }
    procedure read_interface_declarations;

    { reads any routine in the implementation, or a non-method routine
      declaration in the interface (depending on whether or not parse_only is
      true) }
    procedure read_proc(isclassmethod:boolean; usefwpd: tprocdef; isgeneric:boolean);

    { parses only the body of a non nested routine; needs a correctly setup pd }
    procedure read_proc_body(pd:tprocdef);

    procedure import_external_proc(pd:tprocdef);


implementation

    uses
       sysutils,
       { common }
       cutils, cmsgs, cclasses,
       { global }
       globtype,tokens,verbose,comphook,constexp,
       systems,cpubase,aasmbase,aasmtai,aasmdata,
       { symtable }
       symconst,symbase,symsym,symtype,symtable,defutil,defcmp,symcreat,
       paramgr,
       fmodule,
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
       pbase,pstatmnt,pdecl,pdecsub,pexports,pgenutil,pparautl,
       { codegen }
       tgobj,cgbase,cgobj,hlcgobj,hlcgcpu,dbgbase,

       ncgflw,
       ncgutil,

       optbase,
       opttail,
       optcse,
       optloop,
       optconstprop,
       optdeadstore,
       optloadmodifystore,
       optutils
{$if defined(arm)}
       ,cpuinfo
{$endif arm}
       {$ifndef NOOPT}
       ,aopt
       {$endif}
       ;

    function checknodeinlining(procdef: tprocdef): boolean;

      procedure _no_inline(const reason: TMsgStr);
        begin
          include(procdef.implprocoptions,pio_inline_not_possible);
          Message1(parser_n_not_supported_for_inline,reason);
          Message(parser_h_inlining_disabled);
        end;

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
            _no_inline('assembler');
            exit;
          end;
        if (pi_has_global_goto in current_procinfo.flags) or
           (pi_has_interproclabel in current_procinfo.flags) then
          begin
            _no_inline('global goto');
            exit;
          end;
        if pi_has_nested_exit in current_procinfo.flags then
          begin
            _no_inline('nested exit');
            exit;
          end;
        if pi_calls_c_varargs in current_procinfo.flags then
          begin
            _no_inline('called C-style varargs functions');
            exit;
          end;
        { the compiler cannot handle inherited in inlined subroutines because
          it tries to search for self in the symtable, however, the symtable
          is not available }
        if pi_has_inherited in current_procinfo.flags then
          begin
            _no_inline('inherited');
            exit;
          end;
        if pio_nested_access in procdef.implprocoptions then
         begin
           _no_inline('access to local from nested scope');
           exit;
         end;
        { We can't support inlining for procedures that have nested
          procedures because the nested procedures use a fixed offset
          for accessing locals in the parent procedure (PFV) }
        if current_procinfo.has_nestedprocs then
          begin
            _no_inline('nested procedures');
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
                      _no_inline('formal parameter');
                      exit;
                    end;
                end;
              arraydef :
                begin
                  if is_array_of_const(currpara.vardef) or
                     is_variant_array(currpara.vardef) then
                    begin
                      _no_inline('array of const');
                      exit;
                    end;
                  { open arrays might need re-basing of the index, i.e. if you pass
                    an array[1..10] as open array, you have to add 1 to all index operations
                    if you directly inline it }
                  if is_open_array(currpara.vardef) then
                    begin
                      _no_inline('open array');
                      exit;
                    end;
                end;
              else
                ;
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
                             cordconstnode.create(vardef.size,sizeuinttype,false),
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
        if (tsym(p).typ=paravarsym) then
          begin
            if tparavarsym(p).needs_finalization then
              begin
                include(current_procinfo.flags,pi_needs_implicit_finally);
                include(current_procinfo.flags,pi_do_call);
              end;
          end;
      end;


    procedure check_finalize_locals(p:TObject;arg:pointer);
      begin
        { include the result: it needs to be finalized in case an exception }
        { occurs                                                            }
        if (tsym(p).typ=localvarsym) and
           (tlocalvarsym(p).refs>0) and
           is_managed_type(tlocalvarsym(p).vardef) then
          begin
            include(current_procinfo.flags,pi_needs_implicit_finally);
            include(current_procinfo.flags,pi_do_call);
          end;
      end;

    procedure init_main_block_syms(block: tnode);
      var
         oldfilepos: tfileposinfo;
      begin
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

    function block(islibrary : boolean) : tnode;
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
                     end
                   else if token=_FINALIZATION then
                     begin
                       { when a unit has only a finalization section, we can come to this
                         point when we try to read the nonh existing initalization section
                         so we've to check if we are really try to parse the finalization }
                       if current_procinfo.procdef.proctypeoption=potype_unitfinalize then
                         block:=statement_block(_FINALIZATION)
                       else
                         block:=nil;
                     end
                   else
                     block:=statement_block(_BEGIN);
                end;
            end
         else
            begin
               { parse routine body }
               block:=statement_block(_BEGIN);
               init_main_block_syms(block);
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
                { if self=nil then exit
                  calling fail instead of exit is useless because
                  there is nothing to dispose (PFV) }
                if is_class_or_object(current_structdef) then
                  addstatement(newstatement,cifnode.create(
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
                            cnodeutils.finalize_data_node(load_self_node),
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


{****************************************************************************
                                  TCGProcInfo
****************************************************************************}

     destructor tcgprocinfo.destroy;
       begin
         code.free;
         inherited destroy;
       end;


    function tcgprocinfo.calc_stackframe_size:longint;
      begin
        result:=Align(tg.direction*tg.lasttemp,current_settings.alignment.localalignmin);
      end;


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
        constructionblock,
        exceptblock,
        newblock: tblocknode;
        newstatement: tstatementnode;
        pd: tprocdef;
        constructionsuccessful: tlocalvarsym;
      begin
        if assigned(procdef.struct) and
           (procdef.proctypeoption=potype_constructor) then
          begin
            withexceptblock:=
              withexceptblock and
              not(target_info.system in systems_garbage_collected_managed_types);
            { Don't test self and the vmt here. See generate_bodyexit_block }
            { why (JM)                                                      }
            oldlocalswitches:=current_settings.localswitches;
            current_settings.localswitches:=oldlocalswitches-[cs_check_object,cs_check_range];

            { call AfterConstruction for classes }
            constructionsuccessful:=nil;
            if is_class(procdef.struct) then
              begin
                constructionsuccessful:=clocalvarsym.create(internaltypeprefixName[itp_vmt_afterconstruction_local],vs_value,ptrsinttype,[]);
                procdef.localst.insert(constructionsuccessful,false);
                srsym:=search_struct_member(procdef.struct,'AFTERCONSTRUCTION');
                if not assigned(srsym) or
                   (srsym.typ<>procsym) then
                  internalerror(200305106);

                current_filepos:=entrypos;
                constructionblock:=internalstatements(newstatement);
                { initialise constructionsuccessful with -1, indicating that
                  the construction was not successful and hence
                  beforedestruction should not be called if a destructor is
                  called from the constructor }
                addstatement(newstatement,cassignmentnode.create(
                  cloadnode.create(constructionsuccessful,procdef.localst),
                  genintconstnode(-1))
                );
                { first execute all constructor code. If no exception
                  occurred then we will execute afterconstruction,
                  otherwise we won't (the exception will jump over us) }
                addstatement(newstatement,tocode);
                current_filepos:=exitpos;
                { if implicit finally node wasn't created, then exit label and
                  finalization code must be handled here and placed before
                  afterconstruction }
                if not ((pi_needs_implicit_finally in flags) and
                  (cs_implicit_exceptions in current_settings.moduleswitches)) then
                  begin
                    include(tocode.flags,nf_block_with_exit);
                    if procdef.proctypeoption<>potype_exceptfilter then
                      addstatement(newstatement,cfinalizetempsnode.create);
                    cnodeutils.procdef_block_add_implicit_finalize_nodes(procdef,newstatement);
                    temps_finalized:=true;
                  end;

                { construction successful -> beforedestruction should be called
                  if an exception happens now }
                addstatement(newstatement,cassignmentnode.create(
                  cloadnode.create(constructionsuccessful,procdef.localst),
                  genintconstnode(1))
                );
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
                    ccallnode.create(nil,tprocsym(srsym),srsym.owner,load_self_node,[],nil),
                    nil));
                tocode:=constructionblock;
              end;

            if withexceptblock and (procdef.struct.typ=objectdef) then
              begin
                { Generate the implicit "fail" code for a constructor (destroy
                  in case an exception happened) }
                pd:=tobjectdef(procdef.struct).find_destructor;
                { this will always be the case for classes, since tobject has
                  a destructor }
                if assigned(pd) or is_object(procdef.struct) then
                  begin
                    current_filepos:=exitpos;
                    exceptblock:=internalstatements(newstatement);
                    { first free the instance if non-nil }
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
        wrappedbody,
        newblock     : tnode;
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
        temps_finalized:=false;
        bodyexitcode:=generate_bodyexit_block;

        { Generate procedure by combining init+body+final,
          depending on the implicit finally we need to add
          an try...finally...end wrapper }
        current_filepos:=entrypos;
        newblock:=internalstatements(newstatement);
        { initialization is common for all cases }
        addstatement(newstatement,loadpara_asmnode);
        addstatement(newstatement,stackcheck_asmnode);
        addstatement(newstatement,entry_asmnode);
        cnodeutils.procdef_block_add_implicit_initialize_nodes(procdef,newstatement);
        addstatement(newstatement,init_asmnode);
        if assigned(procdef.parentfpinitblock) then
          begin
            if assigned(tblocknode(procdef.parentfpinitblock).left) then
              begin
                { could be an asmn in case of a pure assembler procedure,
                  but those shouldn't access nested variables }
                addstatement(newstatement,procdef.parentfpinitblock);
              end
            else
              procdef.parentfpinitblock.free;
            procdef.parentfpinitblock:=nil;
          end;
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
            if procdef.proctypeoption<>potype_exceptfilter then
              addstatement(codestatement,cfinalizetempsnode.create);
            cnodeutils.procdef_block_add_implicit_finalize_nodes(procdef,codestatement);
            temps_finalized:=true;

            current_filepos:=entrypos;
            wrappedbody:=ctryfinallynode.create_implicit(code,finalcode);
            { afterconstruction must be called after finalizetemps, because it
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
            { constructors need destroy-on-exception code even if they don't
              have managed variables/temps }
            maybe_add_constructor_wrapper(code,
              cs_implicit_exceptions in current_settings.moduleswitches);
            current_filepos:=entrypos;
            addstatement(newstatement,code);
            current_filepos:=exitpos;
            if assigned(nestedexitlabel) then
              addstatement(newstatement,clabelnode.create(cnothingnode.create,nestedexitlabel));
            addstatement(newstatement,exitlabel_asmnode);
            addstatement(newstatement,bodyexitcode);
            if not is_constructor then
              begin
                if procdef.proctypeoption<>potype_exceptfilter then
                  addstatement(newstatement,cfinalizetempsnode.create);
                cnodeutils.procdef_block_add_implicit_finalize_nodes(procdef,newstatement);
                temps_finalized:=true;
              end;
          end;
        if not temps_finalized then
          begin
            current_filepos:=exitpos;
            cnodeutils.procdef_block_add_implicit_finalize_nodes(procdef,newstatement);
          end;
        do_firstpass(newblock);
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
           end;
      end;


{$if defined(i386) or defined(x86_64) or defined(arm) or defined(riscv32) or defined(riscv64)}
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
                        exception_flags[((target_info.cpu=cpu_i386) and (not paramanager.use_fixed_stack))
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
                   not(has_assembler_child) {and (para_stack_size=0)} then
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
{$endif defined(x86) or defined(arm)}
        { set the start offset to the start of the temp area in the stack }
        set_first_temp_offset;
      end;

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
      end;


    procedure tcgprocinfo.set_eh_info;
      begin
        inherited;
         if (tf_use_psabieh in target_info.flags) and
            ((pi_uses_exceptions in flags) or
             ((cs_implicit_exceptions in current_settings.moduleswitches) and
              (pi_needs_implicit_finally in flags))) or
             (pi_has_except_table_data in flags) then
           procdef.personality:=search_system_proc('_FPC_PSABIEH_PERSONALITY_V0');
      end;


{$ifdef DEBUG_NODE_XML}
    procedure tcgprocinfo.XMLPrintProc;
      var
        T: Text;
        W: Word;
        syssym: tsyssym;

      procedure PrintType(Flag: string);
        begin
          Write(T, ' type="', Flag, '"');
        end;

      procedure PrintOption(Flag: string);
        begin
          WriteLn(T, PrintNodeIndention, '<option>', Flag, '</option>');
        end;

      begin
        if current_module.ppxfilefail then
          Exit;

        Assign(T, current_module.ppxfilename);
        {$push} {$I-}
        Append(T);
        if IOResult <> 0 then
          begin
            Message1(exec_e_cant_create_archivefile,current_module.ppxfilename);
            current_module.ppxfilefail := True;
            Exit;
          end;
        {$pop}
        Write(T, PrintNodeIndention, '<subroutine');

        { Check to see if the procedure is a class or object method }
        if Assigned(procdef.struct) then
          begin
            if Assigned(procdef.struct.objrealname) then
              Write(T, ' struct="', TNode.SanitiseXMLString(procdef.struct.objrealname^), '"')
            else
              Write(T, ' struct="&lt;NULL&gt;"');
          end;

        case procdef.proctypeoption of
          potype_none: { Do nothing };

          potype_procedure,
          potype_function:
            if po_classmethod in procdef.procoptions then
              begin
                if po_staticmethod in procdef.procoptions then
                  PrintType('static class method')
                else
                  PrintType('class method');
              end;
            { Do nothing otherwise }

          potype_proginit,
          potype_unitinit:
            PrintType('initialization');
          potype_unitfinalize:
            PrintType('finalization');
          potype_constructor:
            PrintType('constructor');
          potype_destructor:
            PrintType('destructor');
          potype_operator:
            PrintType('operator');
          potype_class_constructor:
            PrintType('class constructor');
          potype_class_destructor:
            PrintType('class destructor');
          potype_propgetter:
            PrintType('dispinterface getter');
          potype_propsetter:
            PrintType('dispinterface setter');
          potype_exceptfilter:
            PrintType('except filter');
          potype_mainstub:
            PrintType('main stub');
          potype_libmainstub:
            PrintType('library main stub');
          potype_pkgstub:
            PrintType('package stub');
        end;

        Write(T, ' name="', TNode.SanitiseXMLString(procdef.customprocname([pno_showhidden, pno_noclassmarker])), '"');

        if po_hascallingconvention in procdef.procoptions then
          Write(T, ' convention="', proccalloptionStr[procdef.proccalloption], '"');

        WriteLn(T, '>');

        PrintNodeIndent;

        if Assigned(procdef.returndef) and not is_void(procdef.returndef) then
          WriteLn(T, PrintNodeIndention, '<returndef>', TNode.SanitiseXMLString(procdef.returndef.typesymbolprettyname), '</returndef>');

        if po_reintroduce in procdef.procoptions then
          PrintOption('reintroduce');
        if po_virtualmethod in procdef.procoptions then
          PrintOption('virtual');
        if po_finalmethod in procdef.procoptions then
          PrintOption('final');
        if po_overridingmethod in procdef.procoptions then
          PrintOption('override');
        if po_overload in procdef.procoptions then
          PrintOption('overload');
        if po_compilerproc in procdef.procoptions then
          PrintOption('compilerproc');
        if po_assembler in procdef.procoptions then
          PrintOption('assembler');
        if po_nostackframe in procdef.procoptions then
          PrintOption('nostackframe');
        if po_inline in procdef.procoptions then
          PrintOption('inline');
        if po_noreturn in procdef.procoptions then
          PrintOption('noreturn');
        if po_noinline in procdef.procoptions then
          PrintOption('noinline');

        if Assigned(Code) then
          begin
            WriteLn(T, PrintNodeIndention, '<code>');
            PrintNodeIndent;
            XMLPrintNode(T, Code);
            PrintNodeUnindent;
            WriteLn(T, PrintNodeIndention, '</code>');
          end
        else
          WriteLn(T, PrintNodeIndention, '<code />');

        PrintNodeUnindent;
        WriteLn(T, PrintNodeIndention, '</subroutine>');
        WriteLn(T); { Line for spacing }
        Close(T);
      end;
{$endif DEBUG_NODE_XML}

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
          end;
        resetprocdef;
      end;

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
        export_local_ref_syms;
        export_local_ref_defs;
       end;

    procedure searchthreadvar(p: TObject; arg: pointer);
      var
        i : longint;
        pd : tprocdef;
      begin
        case tsym(p).typ of
          staticvarsym :
            begin
                  { local (procedure or unit) variables only need finalization
                    if they are used
                  }
              if (vo_is_thread_var in tstaticvarsym(p).varoptions) and
                 ((tstaticvarsym(p).refs>0) or
                  { global (unit) variables always need finalization, since
                    they may also be used in another unit
                  }
                  (tstaticvarsym(p).owner.symtabletype=globalsymtable)) and
                  (
                    (tstaticvarsym(p).varspez<>vs_const) or
                    (vo_force_finalize in tstaticvarsym(p).varoptions)
                  ) and
                 not(vo_is_funcret in tstaticvarsym(p).varoptions) and
                 not(vo_is_external in tstaticvarsym(p).varoptions) and
                 is_managed_type(tstaticvarsym(p).vardef) then
                include(current_procinfo.flags,pi_uses_threadvar);
            end;
          procsym :
            begin
              for i:=0 to tprocsym(p).ProcdefList.Count-1 do
                begin
                  pd:=tprocdef(tprocsym(p).ProcdefList[i]);
                  if assigned(pd.localst) and
                     (pd.procsym=tprocsym(p)) and
                     (pd.localst.symtabletype<>staticsymtable) then
                    pd.localst.SymList.ForEachCall(@searchthreadvar,arg);
                end;
            end;
          else
            ;
        end;
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
      end;


    procedure tcgprocinfo.generate_code;

       procedure check_for_threadvars_in_initfinal;
         begin
           if current_procinfo.procdef.proctypeoption=potype_unitfinalize then
             begin
                { this is also used for initialization of variables in a
                  program which does not have a globalsymtable }
                if assigned(current_module.globalsymtable) then
                  TSymtable(current_module.globalsymtable).SymList.ForEachCall(@searchthreadvar,nil);
                TSymtable(current_module.localsymtable).SymList.ForEachCall(@searchthreadvar,nil);
             end;
         end;

      var
        old_current_procinfo : tprocinfo;
        oldmaxfpuregisters : longint;
        oldfilepos : tfileposinfo;
        old_current_structdef : tabstractrecorddef;
        oldswitches : tlocalswitches;
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
           not(po_noinline in procdef.procoptions) and
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
            if not(current_module.islibrary) and (procdef.proctypeoption=potype_proginit) then
              include(procdef.procoptions,po_noreturn);
          end;

        { set implicit_finally flag when there are locals/paras to be finalized }
        if not(po_assembler in current_procinfo.procdef.procoptions) then
          begin
            procdef.parast.SymList.ForEachCall(@check_finalize_paras,nil);
            procdef.localst.SymList.ForEachCall(@check_finalize_locals,nil);
          end;

{$ifdef SUPPORT_SAFECALL}
        { set implicit_finally flag for if procedure is safecall }
        if (tf_safecall_exceptions in target_info.flags) and
           (procdef.proccalloption=pocall_safecall) then
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
        if paraprintnodetree <> 0 then
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
          and not(pi_has_label in flags) then
          begin
            {RedoDFA:=}OptimizeInductionVariables(code);
          end;

        if (cs_opt_remove_emtpy_proc in current_settings.optimizerswitches) and
          (procdef.proctypeoption in [potype_operator,potype_procedure,potype_function]) and
          (code.nodetype=blockn) and (tblocknode(code).statements=nil) then
          procdef.isempty:=true;

        { unit static/global symtables might contain threadvars which are not explicitly used but which might
          require a tls register, so check for such variables }
        check_for_threadvars_in_initfinal;

        { add implicit entry and exit code }
        add_entry_exit_code;

        if cs_opt_nodecse in current_settings.optimizerswitches then
          do_optcse(code);

        if cs_opt_use_load_modify_store in current_settings.optimizerswitches then
          do_optloadmodifystore(code);

        { only do secondpass if there are no errors }
        if (ErrorCount=0) then
          begin
            create_hlcodegen;

            setup_eh;

            if (procdef.proctypeoption<>potype_exceptfilter) then
              setup_tempgen;

            { Create register allocator, must come after framepointer is known }
            hlcg.init_register_allocators;

            generate_parameter_info;

            { allocate got register if needed }
            allocate_got_register(aktproccode);

            { allocate got register if needed }
            allocate_tls_register(aktproccode);

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

            hlcg.gen_load_para_value(templist);

            { caller paraloc info is also necessary in the stackframe_entry
              code of the ppc (and possibly other processors)               }
            procdef.init_paraloc_info(callerside);

            CalcExecutionWeights(code);

            { Print the node to tree.log }
            if paraprintnodetree <> 0 then
              printproc( 'right before code generation');

            { generate code for the node tree }
            do_secondpass(code);
            aktproccode.concatlist(current_asmdata.CurrAsmList);

            { The position of the loadpara_asmnode is now known }
            aktproccode.insertlistafter(loadpara_asmnode.currenttai,templist);

            oldswitches:=current_settings.localswitches;

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
            else if not temps_finalized then
              begin
                hlcg.gen_finalize_code(templist);
                { the finalcode must be concated if there was no position available,
                  using insertlistafter will result in an insert at the start
                  when currentai=nil }
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

            { reset switches }
            current_settings.localswitches:=oldswitches;

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
            if (tf_pic_uses_got in target_info.flags) and
              (pi_needs_got in flags) and
              (got<>NR_NO) then
              cg.a_reg_sync(aktproccode,got);

            if (pi_needs_tls in flags) and
              (tlsoffset<>NR_NO) then
              cg.a_reg_sync(aktproccode,tlsoffset);

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
                hlcg.gen_stack_check_call(templist);
                aktproccode.insertlistafter(stackcheck_asmnode.currenttai,templist);
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

            { init tls if needed }
            cg.g_maybe_tls_init(templist);
            aktproccode.insertlistafter(stackcheck_asmnode.currenttai,templist);

            { re-enable if more code at the end is ever generated here
            cg.set_regalloc_live_range_direction(rad_forward);
            }


{$ifndef NoOpt}
{$ifndef i386}
            if (cs_opt_scheduler in current_settings.optimizerswitches) and
              { do not optimize pure assembler procedures }
              not(pi_is_assembler in flags) then
              preregallocschedule(aktproccode);
{$endif i386}
{$endif NoOpt}

            { The procedure body is finished, we can now
              allocate the registers }
            cg.do_register_allocation(aktproccode,headertai);

            { translate imag. register to their real counter parts
              this is necessary for debuginfo and verbose assembler output
              when SSA will be implented, this will be more complicated because we've to
              maintain location lists }
            procdef.parast.SymList.ForEachCall(@translate_registers,templist);
            procdef.localst.SymList.ForEachCall(@translate_registers,templist);
            if (tf_pic_uses_got in target_info.flags) and (pi_needs_got in flags) and
               not(cs_no_regalloc in current_settings.globalswitches) and
               (got<>NR_NO) then
              cg.translate_register(got);

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
                hlcg.gen_stack_check_size_para(templist);
                aktproccode.insertlistafter(stackcheck_asmnode.currenttai,templist)
              end;

            current_procinfo.set_eh_info;

            { Add entry code (stack allocation) after header }
            current_filepos:=entrypos;
            gen_proc_entry_code(templist);
            aktproccode.insertlistafter(headertai,templist);
{$ifdef SUPPORT_SAFECALL}
            { Set return value of safecall procedure if implicit try/finally blocks are disabled }
            if not (cs_implicit_exceptions in current_settings.moduleswitches) and
               (tf_safecall_exceptions in target_info.flags) and
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

             { sanity check }
             if not(assigned(current_procinfo.procdef.personality)) and
                (tf_use_psabieh in target_info.flags) and
                ((pi_uses_exceptions in flags) or
                 ((cs_implicit_exceptions in current_settings.moduleswitches) and
                  (pi_needs_implicit_finally in flags))) then
               Internalerror(2019021005);

            { Position markers are only used to insert additional code after the secondpass
              and before this point. They are of no use in optimizer. Instead of checking and
              ignoring all over the optimizer, just remove them here. }
            delete_marker(entry_asmnode);
            delete_marker(loadpara_asmnode);
            delete_marker(exitlabel_asmnode);
            delete_marker(stackcheck_asmnode);
            delete_marker(init_asmnode);

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

            { Perform target-specific processing if necessary }
            postprocess_code;

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

            finish_eh;

            hlcg.record_generated_code_for_procdef(current_procinfo.procdef,aktproccode,aktlocaldata);

            { only now we can remove the temps }
            if (procdef.proctypeoption<>potype_exceptfilter) then
              begin
                tg.resettempgen;
                tg.free;
                tg:=nil;
              end;
            { stop tempgen and ra }
            hlcg.done_register_allocators;
            destroy_hlcodegen;
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


    procedure tcgprocinfo.parse_body;
      var
         old_current_procinfo : tprocinfo;
         old_block_type : tblock_type;
         st : TSymtable;
         old_current_structdef: tabstractrecorddef;
         old_current_genericdef,
         old_current_specializedef: tstoreddef;
         parentfpinitblock: tnode;
         old_parse_generic: boolean;
         recordtokens : boolean;

      begin
         old_current_procinfo:=current_procinfo;
         old_block_type:=block_type;
         old_current_structdef:=current_structdef;
         old_current_genericdef:=current_genericdef;
         old_current_specializedef:=current_specializedef;
         old_parse_generic:=parse_generic;

         current_procinfo:=self;
         current_structdef:=procdef.struct;


        { check if the definitions of certain types are available which might not be available in older rtls and
          which are assigned "on the fly" in types_dec }
{$ifndef jvm}
        if not assigned(rec_exceptaddr) then
          Message1(cg_f_internal_type_not_found,'TEXCEPTADDR');
        if not assigned(rec_tguid) then
          Message1(cg_f_internal_type_not_found,'TGUID');
        if not assigned(rec_jmp_buf) then
          Message1(cg_f_internal_type_not_found,'TJMPBUF');
{$endif}

         { if the procdef is truly a generic (thus takes parameters itself) then
           /that/ is our genericdef, not the - potentially - generic struct }
         if procdef.is_generic then
           begin
             current_genericdef:=procdef;
             parse_generic:=true;
           end
         else if assigned(current_structdef) and (df_generic in current_structdef.defoptions) then
           begin
             current_genericdef:=current_structdef;
             parse_generic:=true;
           end;
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

         recordtokens:=procdef.is_generic or
                       (
                         assigned(procdef.struct) and
                         (df_generic in procdef.struct.defoptions) and
                         assigned(procdef.owner) and
                         (procdef.owner.defowner=procdef.struct)
                       );

         if recordtokens then
           begin
             { start token recorder for generic template }
             procdef.initgeneric;
             current_scanner.startrecordtokens(procdef.generictokenbuf);
           end;

         { parse the code ... }
         code:=block(current_module.islibrary);

         if recordtokens then
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
               Message(parser_e_global_generic_references_static);
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
             do_typecheckpass(code);

             if assigned(procdef.parentfpinitblock) then
               begin
                 if assigned(tblocknode(procdef.parentfpinitblock).left) then
                   begin
                     parentfpinitblock:=procdef.parentfpinitblock;
                     do_typecheckpass(parentfpinitblock);
                     procdef.parentfpinitblock:=parentfpinitblock;
                   end
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
         if paraprintnodetree <> 0 then
           printproc( 'after parsing');

{$ifdef DEBUG_NODE_XML}
         printnodeindention := printnodespacing;
         XMLPrintProc;
{$endif DEBUG_NODE_XML}

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
          if (is_managed_type(vardef) and
             (varspez in [vs_value,vs_out])) or
             (is_shortstring(vardef) and
             (varspez=vs_value)) then
            include(current_procinfo.flags,pi_do_call);
      end;


    procedure read_proc_body(old_current_procinfo:tprocinfo;pd:tprocdef);
      {
        Parses the procedure directives, then parses the procedure body, then
        generates the code for it
      }

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

        { reset _FAIL as _SELF normal }
        if (pd.proctypeoption=potype_constructor) then
          tokeninfo^[_FAIL].keyword:=oldfailtokenmode;

        { When it's a nested procedure then defer the code generation,
          when back at normal function level then generate the code
          for all defered nested procedures and the current procedure }
        if not isnestedproc then
          begin
            if not(df_generic in current_procinfo.procdef.defoptions) then
              begin
                { also generate the bodies for all previously done
                  specializations so that we might inline them }
                generate_specialization_procs;
                tcgprocinfo(current_procinfo).generate_code_tree;
              end;
          end;

        { release procinfo }
        if tprocinfo(current_module.procinfo)<>current_procinfo then
          internalerror(200304274);
        current_module.procinfo:=current_procinfo.parent;

        { For specialization we didn't record the last semicolon. Moving this parsing
          into the parse_body routine is not done because of having better file position
          information available }
        if not current_procinfo.procdef.is_specialization and
            (
              not assigned(current_procinfo.procdef.struct) or
              not (df_specialization in current_procinfo.procdef.struct.defoptions)
              or not (
                assigned(current_procinfo.procdef.owner) and
                (current_procinfo.procdef.owner.defowner=current_procinfo.procdef.struct)
              )
            ) then
          consume(_SEMICOLON);

        if not isnestedproc then
          { current_procinfo is checked for nil later on }
          freeandnil(current_procinfo);
      end;


    procedure read_proc_body(pd:tprocdef);
      var
        old_module_procinfo : tobject;
        old_current_procinfo : tprocinfo;
      begin
        old_current_procinfo:=current_procinfo;
        old_module_procinfo:=current_module.procinfo;
        current_procinfo:=nil;
        current_module.procinfo:=nil;
        read_proc_body(nil,pd);
        current_procinfo:=old_current_procinfo;
        current_module.procinfo:=old_module_procinfo;
      end;


    procedure read_proc(isclassmethod:boolean; usefwpd: tprocdef; isgeneric:boolean);
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
{$ifdef genericdef_for_nested}
        def : tprocdef;
        srsym : tsym;
        i : longint;
{$endif genericdef_for_nested}
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

         if not assigned(usefwpd) then
           { parse procedure declaration }
           pd:=parse_proc_dec(isclassmethod,old_current_structdef,isgeneric)
         else
           pd:=usefwpd;

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
               create_smartlink_library then
              include(pd.procoptions,po_global);
            pd.forwarddef:=false;
          end;

         if not assigned(usefwpd) then
           begin
             { parse the directives that may follow }
             parse_proc_directives(pd,pdflags);

             { hint directives, these can be separated by semicolons here,
               that needs to be handled here with a loop (PFV) }
             while try_consume_hintdirective(pd.symoptions,pd.deprecatedmsg) do
              Consume(_SEMICOLON);

             { Set calling convention }
             if parse_only then
               handle_calling_convention(pd,hcc_default_actions_intf)
             else
               handle_calling_convention(pd,hcc_default_actions_impl)
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
                 end
                else
                  begin
                    if pd.is_generic and not assigned(pd.struct) then
                      tprocsym(pd.procsym).owner.includeoption(sto_has_generic);
                  end;
              end;
           end;

         { Set mangled name }
         proc_set_mangledname(pd);

         { inherit generic flags from parent routine }
         if assigned(old_current_procinfo) and
             (old_current_procinfo.procdef.defoptions*[df_specialization,df_generic]<>[]) then
           begin
             if df_generic in old_current_procinfo.procdef.defoptions then
               include(pd.defoptions,df_generic);
             if df_specialization in old_current_procinfo.procdef.defoptions then
               begin
                 include(pd.defoptions,df_specialization);
                 { the procdefs encountered here are nested procdefs of which
                   their complete definition also resides inside the current token
                   stream, thus access to their genericdef is not required }
                 {$ifdef genericdef_for_nested}
                 { find the corresponding routine in the generic routine }
                 if not assigned(old_current_procinfo.procdef.genericdef) then
                   internalerror(2016121701);
                 srsym:=tsym(tprocdef(old_current_procinfo.procdef.genericdef).getsymtable(gs_local).find(pd.procsym.name));
                 if not assigned(srsym) or (srsym.typ<>procsym) then
                   internalerror(2016121702);
                 { in practice the generic procdef should be at the same index
                   as the index of the current procdef, but as there *might* be
                   differences between the amount of defs generated for the
                   specialization and the generic search for the def using
                   parameter comparison }
                 for i:=0 to tprocsym(srsym).procdeflist.count-1 do
                   begin
                     def:=tprocdef(tprocsym(srsym).procdeflist[i]);
                     if (compare_paras(def.paras,pd.paras,cp_none,[cpo_ignorehidden,cpo_openequalisexact,cpo_ignoreuniv])=te_exact) and
                         (compare_defs(def.returndef,pd.returndef,nothingn)=te_exact) then
                       begin
                         pd.genericdef:=def;
                         break;
                       end;
                   end;
                 if not assigned(pd.genericdef) then
                   internalerror(2016121703);
                 {$endif}
               end;
           end;

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
                 import_external_proc(pd);
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
                   begin
                     { we cannot handle the callee-side of variadic functions (and
                       even if we could, e.g. LLVM cannot call through to something
                       else in that case) }
                     if is_c_variadic(pd) then
                       Message1(parser_e_callthrough_varargs,pd.fullprocname(false));
                     call_through_new_name(pd,proc_get_importname(pd));
                     include(pd.implprocoptions,pio_thunk);
                   end
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

         { always register public functions that are only declared in the
           implementation section as they might be called using an external
           declaration from another unit }
         if (po_global in pd.procoptions) and
             not pd.interfacedef and
             ([df_generic,df_specialization]*pd.defoptions=[]) then
           begin
             pd.register_def;
             pd.procsym.register_sym;
           end;

         { make sure that references to forward-declared functions are not }
         { treated as references to external symbols, needed for darwin.   }

         { make sure we don't change the binding of real external symbols }
         if (([po_external,po_weakexternal]*pd.procoptions)=[]) and (pocall_internproc<>pd.proccalloption) then
           current_asmdata.DefineProcAsmSymbol(pd,pd.mangledname,pd.needsglobalasmsym);

         current_structdef:=old_current_structdef;
         current_genericdef:=old_current_genericdef;
         current_specializedef:=old_current_specializedef;
         current_procinfo:=old_current_procinfo;
      end;


    procedure import_external_proc(pd:tprocdef);
      var
        name : string;
      begin
        if not (po_external in pd.procoptions) then
          internalerror(2015121101);

        { Import DLL specified? }
        if assigned(pd.import_dll) then
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
        else
          begin
            name:=proc_get_importname(pd);
            { add import name to external list for DLL scanning }
            if tf_has_dllscanner in target_info.flags then
              current_module.dllscannerinputlist.Add(name,pd);
            { needed for units that use functions in packages this way }
            current_module.add_extern_asmsym(name,AB_EXTERNAL,AT_FUNCTION);
          end;
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

{$ifdef DEBUG_NODE_XML}
    procedure XMLInitializeNodeFile(RootName, ModuleName: shortstring);
      var
        T: Text;
      begin
        Assign(T, current_module.ppxfilename);
        {$push} {$I-}
        Rewrite(T);
        if IOResult<>0 then
          begin
            Message1(exec_e_cant_create_archivefile,current_module.ppxfilename);
            current_module.ppxfilefail := True;
            Exit;
          end;
        {$pop}
        { Mark the node dump file as available for writing }
        current_module.ppxfilefail := False;
        WriteLn(T, '<?xml version="1.0" encoding="utf-8"?>');
        WriteLn(T, '<', RootName, ' name="', ModuleName, '">');
        Close(T);
      end;


    procedure XMLFinalizeNodeFile(RootName: shortstring);
      var
        T: Text;
      begin
        if current_module.ppxfilefail then
          Exit;

        current_module.ppxfilefail := True; { File is now considered closed no matter what happens }
        Assign(T, current_module.ppxfilename);
        {$push} {$I-}
        Append(T);
        if IOResult<>0 then
          begin
            Message1(exec_e_cant_create_archivefile,current_module.ppxfilename);
            Exit;
          end;
        {$pop}
        WriteLn(T, '</', RootName, '>');
        Close(T);
      end;
{$endif DEBUG_NODE_XML}

    procedure read_declarations(islibrary : boolean);
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

      var
        is_classdef:boolean;
      begin
        is_classdef:=false;
        hadgeneric:=false;
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
                begin
                  handle_unexpected_had_generic;
                  threadvar_dec(hadgeneric);
                end;
              _CLASS:
                begin
                  is_classdef:=false;
                  if try_to_consume(_CLASS) then
                   begin
                     { class modifier is only allowed for procedures, functions, }
                     { constructors, destructors                                 }
                     if not((token in [_FUNCTION,_PROCEDURE,_DESTRUCTOR,_OPERATOR]) or (token=_CONSTRUCTOR)) and
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
                  if hadgeneric and not (token in [_PROCEDURE,_FUNCTION]) then
                    begin
                      Message(parser_e_procedure_or_function_expected);
                      hadgeneric:=false;
                    end;
                  read_proc(is_classdef,nil,hadgeneric);
                  is_classdef:=false;
                  hadgeneric:=false;
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
                      begin
                        handle_unexpected_had_generic;
                        if is_classdef then
                          begin
                            read_proc(is_classdef,nil,false);
                            is_classdef:=false;
                          end
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
               begin
                 if hadgeneric and not (token in [_FUNCTION, _PROCEDURE]) then
                   begin
                     message(parser_e_procedure_or_function_expected);
                     hadgeneric:=false;
                   end;
                 read_proc(false,nil,hadgeneric);
                 hadgeneric:=false;
               end;
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
                         property_dec
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


end.
