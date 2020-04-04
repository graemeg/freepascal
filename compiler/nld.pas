{
    Copyright (c) 2000-2002 by Florian Klaempfl

    Type checking and register allocation for load/assignment nodes

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
unit nld;

{$i fpcdefs.inc}

interface

    uses
       node,
       {$ifdef state_tracking}
       nstate,
       {$endif}
       symconst,symbase,symtype,symsym,symdef;

    type
       Trttidatatype = (rdt_normal,rdt_ord2str,rdt_str2ord);

       tloadnodeflags = (
         loadnf_is_self,
         { tell the load node the address of the symbol into the location, i.e. location^ must
           be used to access the symbol
           this is for example needed to load self for objects }
         loadnf_load_addr,
         loadnf_inherited,
         { the loadnode is generated internally and a varspez=vs_const should be ignore,
           this requires that the parameter is actually passed by value
           Be really carefull when using this flag! }
         loadnf_isinternal_ignoreconst,

         loadnf_only_uninitialized_hint
        );

       tloadnode = class(tunarynode)
       protected
          fprocdef : tprocdef;
          fprocdefderef : tderef;
          function handle_threadvar_access: tnode; virtual;
       public
          loadnodeflags : set of tloadnodeflags;
          symtableentry : tsym;
          symtableentryderef : tderef;
          symtable : TSymtable;
          constructor create(v : tsym;st : TSymtable);virtual;
          constructor create_procvar(v : tsym;d:tprocdef;st : TSymtable);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          procedure set_mp(p:tnode);
          function  is_addr_param_load:boolean;virtual;
          function  dogetcopy : tnode;override;
          function  pass_1 : tnode;override;
          function  pass_typecheck:tnode;override;
          procedure mark_write;override;
          function  docompare(p: tnode): boolean; override;
          procedure printnodedata(var t:text);override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
          procedure setprocdef(p : tprocdef);
          property procdef: tprocdef read fprocdef write setprocdef;
       end;
       tloadnodeclass = class of tloadnode;

       { different assignment types }
       tassigntype = (at_normal,at_plus,at_minus,at_star,at_slash);

       tassignmentnode = class(tbinarynode)
         protected
          function direct_shortstring_assignment: boolean; virtual;
         public
          assigntype : tassigntype;
          constructor create(l,r : tnode);virtual;
          { no checks for validity of assignment }
          constructor create_internal(l,r : tnode);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function simplify(forinline : boolean) : tnode;override;
       {$ifdef state_tracking}
          function track_state_pass(exec_known:boolean):boolean;override;
       {$endif state_tracking}
          function docompare(p: tnode): boolean; override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
       end;
       tassignmentnodeclass = class of tassignmentnode;

       tarrayconstructorrangenode = class(tbinarynode)
          constructor create(l,r : tnode);virtual;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
       end;
       tarrayconstructorrangenodeclass = class of tarrayconstructorrangenode;

       tarrayconstructornode = class(tbinarynode)
          allow_array_constructor : boolean;
         private
          function has_range_node:boolean;
         protected
          procedure wrapmanagedvarrec(var n: tnode);virtual;abstract;
         public
          constructor create(l,r : tnode);virtual;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode): boolean; override;
          procedure force_type(def:tdef);
          procedure insert_typeconvs;
          function isempty : boolean;
       end;
       tarrayconstructornodeclass = class of tarrayconstructornode;

       ttypenode = class(tnode)
          allowed : boolean;
          helperallowed : boolean;
          typedef : tdef;
          typedefderef : tderef;
          typesym : tsym;
          typesymderef : tderef;
          constructor create(def:tdef);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function  dogetcopy : tnode;override;
          function docompare(p: tnode): boolean; override;
       end;
       ttypenodeclass = class of ttypenode;

       trttinode = class(tnode)
          l1,l2  : longint;
          rttitype : trttitype;
          rttidef : tstoreddef;
          rttidefderef : tderef;
          rttidatatype : Trttidatatype;
          constructor create(def:tstoreddef;rt:trttitype;dt:Trttidatatype);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function  dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode): boolean; override;
       end;
       trttinodeclass = class of trttinode;

    var
       cloadnode : tloadnodeclass = tloadnode;
       cassignmentnode : tassignmentnodeclass = tassignmentnode;
       carrayconstructorrangenode : tarrayconstructorrangenodeclass = tarrayconstructorrangenode;
       carrayconstructornode : tarrayconstructornodeclass = tarrayconstructornode;
       ctypenode : ttypenodeclass = ttypenode;
       crttinode : trttinodeclass = trttinode;

       { Current assignment node }
       aktassignmentnode : tassignmentnode;


implementation

    uses
      verbose,globtype,globals,systems,constexp,compinnr,
      ppu,
      symtable,
      defutil,defcmp,
      cpuinfo,
      htypechk,pass_1,procinfo,paramgr,
      ncon,nflw,ninl,ncnv,nmem,ncal,nutils,
      cgbase
      ;

{*****************************************************************************
                             TLOADNODE
*****************************************************************************}

    function tloadnode.handle_threadvar_access: tnode;
      begin
        { nothing special by default }
        result:=nil;
      end;


    constructor tloadnode.create(v : tsym;st : TSymtable);
      begin
         inherited create(loadn,nil);
         if not assigned(v) then
          internalerror(200108121);
         symtableentry:=v;
         symtable:=st;
         fprocdef:=nil;
      end;


    constructor tloadnode.create_procvar(v : tsym;d:tprocdef;st : TSymtable);
      begin
         inherited create(loadn,nil);
         if not assigned(v) then
          internalerror(200108122);
         symtableentry:=v;
         symtable:=st;
         fprocdef:=d;
      end;


    constructor tloadnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getderef(symtableentryderef);
        symtable:=nil;
        ppufile.getderef(fprocdefderef);
        ppufile.getset(tppuset1(loadnodeflags));
      end;


    procedure tloadnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putderef(symtableentryderef);
        ppufile.putderef(fprocdefderef);
        ppufile.putset(tppuset1(loadnodeflags));
      end;


    procedure tloadnode.buildderefimpl;
      begin
        inherited buildderefimpl;
        symtableentryderef.build(symtableentry);
        fprocdefderef.build(fprocdef);
      end;


    procedure tloadnode.derefimpl;
      begin
        inherited derefimpl;
        symtableentry:=tsym(symtableentryderef.resolve);
        symtable:=symtableentry.owner;
        fprocdef:=tprocdef(fprocdefderef.resolve);
      end;


    procedure tloadnode.set_mp(p:tnode);
      begin
        { typen nodes should not be set }
        if p.nodetype=typen then
          internalerror(200301042);
        left:=p;
      end;


    function tloadnode.dogetcopy : tnode;
      var
         n : tloadnode;
         orglabel,
         labelcopy : tlabelnode;
      begin
         n:=tloadnode(inherited dogetcopy);
         n.symtable:=symtable;
         n.symtableentry:=symtableentry;
         n.fprocdef:=fprocdef;
         n.loadnodeflags:=loadnodeflags;
         if symtableentry.typ=labelsym then
           begin
             { see the comments for the tgotonode.labelsym field }
             orglabel:=tlabelnode(tlabelsym(symtableentry).code);
             labelcopy:=tlabelnode(orglabel.dogetcopy);
             if not assigned(labelcopy.labsym) then
               begin
                 if not assigned(orglabel.labsym) then
                   internalerror(2019091301);
                 labelcopy.labsym:=clabelsym.create('$copiedlabelfrom$'+orglabel.labsym.RealName);
                 labelcopy.labsym.code:=labelcopy;
               end;
             n.symtableentry:=labelcopy.labsym;
           end;
         result:=n;
      end;


    function tloadnode.is_addr_param_load:boolean;
      begin
        result:=(symtable.symtabletype=parasymtable) and
                (symtableentry.typ=paravarsym) and
                not(vo_has_local_copy in tparavarsym(symtableentry).varoptions) and
                not(loadnf_load_addr in loadnodeflags) and
                paramanager.push_addr_param(tparavarsym(symtableentry).varspez,tparavarsym(symtableentry).vardef,tprocdef(symtable.defowner).proccalloption);
      end;


    function tloadnode.pass_typecheck:tnode;
      begin
         result:=nil;
         case symtableentry.typ of
           absolutevarsym :
             resultdef:=tabsolutevarsym(symtableentry).vardef;
           constsym:
             begin
               if tconstsym(symtableentry).consttyp=constresourcestring then
                 resultdef:=getansistringdef
               else
                 internalerror(22799);
             end;
           staticvarsym :
             begin
               tabstractvarsym(symtableentry).IncRefCountBy(1);
               { static variables referenced in procedures or from finalization,
                 variable needs to be in memory.
                 It is too hard and the benefit is too small to detect whether a
                 variable is only used in the finalization to add support for it (PFV) }
               if assigned(current_procinfo) and
                  (symtable.symtabletype=staticsymtable) and
                  (
                    (symtable.symtablelevel<>current_procinfo.procdef.localst.symtablelevel) or
                    (current_procinfo.procdef.proctypeoption=potype_unitfinalize)
                  ) then
                 make_not_regable(self,[ra_different_scope]);
               resultdef:=tabstractvarsym(symtableentry).vardef;
               if vo_is_thread_var in tstaticvarsym(symtableentry).varoptions then
                 result:=handle_threadvar_access;
             end;
           paravarsym,
           localvarsym :
             begin
               tabstractvarsym(symtableentry).IncRefCountBy(1);
               { Nested variable? The we need to load the framepointer of
                 the parent procedure }
               if assigned(current_procinfo) and
                  (symtable.symtabletype in [localsymtable,parasymtable]) and
                  (symtable.symtablelevel<>current_procinfo.procdef.parast.symtablelevel) then
                 begin
                   if assigned(left) then
                     internalerror(200309289);
                   left:=cloadparentfpnode.create(tprocdef(symtable.defowner),lpf_forload);
                   { we can't inline the referenced parent procedure }
                   include(tprocdef(symtable.defowner).implprocoptions,pio_nested_access);
                   { reference in nested procedures, variable needs to be in memory }
                   { and behaves as if its address escapes its parent block         }
                   make_not_regable(self,[ra_different_scope]);
                 end;
               resultdef:=tabstractvarsym(symtableentry).vardef;

               { e.g. self for objects is passed as var-parameter on the caller
                 side, but on the callee-side we use it as a pointer ->
                 adjust }
               if (loadnf_load_addr in loadnodeflags) then
                 resultdef:=cpointerdef.getreusable(resultdef);

               if (vo_is_self in tabstractvarsym(symtableentry).varoptions) and (resultdef=objc_idtype) and
                 (po_classmethod in tprocdef(symtableentry.owner.defowner).procoptions) then
                 resultdef:=cclassrefdef.create(tprocdef(symtableentry.owner.defowner).struct)
             end;
           procsym :
             begin
               { Return the first procdef. In case of overloaded
                 procdefs the matching procdef will be choosen
                 when the expected procvardef is known, see get_information
                 in htypechk.pas (PFV) }
               if not assigned(fprocdef) then
                 fprocdef:=tprocdef(tprocsym(symtableentry).ProcdefList[0])
               else if po_kylixlocal in fprocdef.procoptions then
                 CGMessage(type_e_cant_take_address_of_local_subroutine);

               { the result is a fprocdef, addrn and proc_to_procvar
                 typeconvn need this as resultdef so they know
                 that the address needs to be returned }
               resultdef:=fprocdef;

               { process methodpointer/framepointer }
               if assigned(left) then
                 begin
                   typecheckpass(left);
                   if (po_classmethod in fprocdef.procoptions) and
                      is_class(left.resultdef) and
                      (left.nodetype<>niln) then
                     begin
                       left:=cloadvmtaddrnode.create(left);
                       typecheckpass(left);
                     end
                 end;
             end;
           labelsym:
             begin
               tlabelsym(symtableentry).used:=true;
               resultdef:=voidtype;
             end;
           else
             internalerror(200104141);
         end;
      end;

    procedure Tloadnode.mark_write;

    begin
      include(flags,nf_write);
    end;

    function tloadnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_REFERENCE;

         case symtableentry.typ of
            absolutevarsym :
              ;
            constsym:
              begin
                if tconstsym(symtableentry).consttyp=constresourcestring then
                  expectloc:=LOC_CREFERENCE;
              end;
            staticvarsym,
            localvarsym,
            paravarsym :
              begin
                if assigned(left) then
                  firstpass(left);
                if not is_addr_param_load and
                   tabstractvarsym(symtableentry).is_regvar(is_addr_param_load) then
                  expectloc:=tvarregable2tcgloc[tabstractvarsym(symtableentry).varregable]
                else
                  if (tabstractvarsym(symtableentry).varspez=vs_const) then
                    expectloc:=LOC_CREFERENCE;
                { call to get address of threadvar }
                if (vo_is_thread_var in tabstractvarsym(symtableentry).varoptions) then
                  begin
                    include(current_procinfo.flags,pi_do_call);
                    include(current_procinfo.flags,pi_uses_threadvar);
                  end;
              end;
            procsym :
              begin
                { initialise left for nested procs if necessary }
                if (m_nested_procvars in current_settings.modeswitches) then
                  setprocdef(fprocdef);
                { method pointer or nested proc ? }
                if assigned(left) then
                  begin
                     expectloc:=LOC_CREGISTER;
                     firstpass(left);
                  end;
              end;
            labelsym :
              begin
                if not assigned(tlabelsym(symtableentry).asmblocklabel) and
                   not assigned(tlabelsym(symtableentry).code) then
                  Message(parser_e_label_outside_proc);
              end
            else
              internalerror(200104143);
         end;
      end;


    function tloadnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (symtableentry = tloadnode(p).symtableentry) and
          (fprocdef = tloadnode(p).fprocdef) and
          (symtable = tloadnode(p).symtable);
      end;


    procedure tloadnode.printnodedata(var t:text);
      begin
        inherited printnodedata(t);
        write(t,printnodeindention,'symbol = ',symtableentry.name);
        if symtableentry.typ=procsym then
          write(t,printnodeindention,'procdef = ',fprocdef.mangledname);
        writeln(t,'');
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TLoadNode.XMLPrintNodeData(var T: Text);
      begin
        inherited XMLPrintNodeData(T);
        WriteLn(T, printnodeindention, '<symbol>', symtableentry.name, '</symbol>');

        if symtableentry.typ = procsym then
          WriteLn(T, printnodeindention, '<procdef>', fprocdef.mangledname, '</procdef>');
      end;
{$endif DEBUG_NODE_XML}

    procedure tloadnode.setprocdef(p : tprocdef);
      begin
        fprocdef:=p;
        resultdef:=p;
        { nested procedure? }
        if assigned(p) and
           is_nested_pd(p) then
          begin
            if not(m_nested_procvars in current_settings.modeswitches) then
              CGMessage(type_e_cant_take_address_of_local_subroutine)
            else
              begin
                { parent frame pointer pointer as "self" }
                left.free;
                left:=cloadparentfpnode.create(tprocdef(p.owner.defowner),lpf_forpara);
              end;
          end
        { we should never go from nested to non-nested }
        else if assigned(left) and
                (left.nodetype=loadparentfpn) then
          internalerror(2010072201);
      end;

{*****************************************************************************
                             TASSIGNMENTNODE
*****************************************************************************}

    function tassignmentnode.direct_shortstring_assignment: boolean;
      begin
        result:=
          is_char(right.resultdef) or
          (right.resultdef.typ=stringdef);
      end;


    constructor tassignmentnode.create(l,r : tnode);

      begin
         inherited create(assignn,l,r);
         assigntype:=at_normal;
         if r.nodetype = typeconvn then
           ttypeconvnode(r).warn_pointer_to_signed:=false;
      end;


    constructor tassignmentnode.create_internal(l, r: tnode);
      begin
        create(l,r);
        include(flags,nf_internal);
      end;


    constructor tassignmentnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        assigntype:=tassigntype(ppufile.getbyte);
      end;


    procedure tassignmentnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(byte(assigntype));
      end;


    function tassignmentnode.dogetcopy : tnode;

      var
         n : tassignmentnode;

      begin
         n:=tassignmentnode(inherited dogetcopy);
         n.assigntype:=assigntype;
         result:=n;
      end;


    function tassignmentnode.simplify(forinline : boolean) : tnode;
      begin
        result:=nil;
        { assignment nodes can perform several floating point }
        { type conversions directly, so no typeconversions    }
        { are inserted in those cases. When inlining, a       }
        { variable may be replaced by a constant which can be }
        { converted at compile time, so check for this case   }
        if is_real(left.resultdef) and
           is_real(right.resultdef) and
           is_constrealnode(right) and
           not equal_defs(right.resultdef,left.resultdef) then
          inserttypeconv(right,left.resultdef);
      end;


    function tassignmentnode.pass_typecheck:tnode;
      var
        hp : tnode;
        useshelper : boolean;
        oldassignmentnode : tassignmentnode;
      begin
        result:=nil;
        resultdef:=voidtype;

        { must be made unique }
        set_unique(left);

        typecheckpass(left);

        left.mark_write;

        { PI. This is needed to return correct resultdef of add nodes for ansistrings
          rawbytestring return needs to be replaced by left.resultdef }
        oldassignmentnode:=aktassignmentnode;
        aktassignmentnode:=self;
        typecheckpass(right);
        aktassignmentnode:=oldassignmentnode;

        set_varstate(right,vs_read,[vsf_must_be_valid]);
        set_varstate(left,vs_written,[]);
        if codegenerror then
          exit;

        { just in case the typecheckpass of right optimized something here }
        if nf_assign_done_in_right in flags then
          begin
            result:=right;
            right:=nil;
            exit;
          end;

        { tp procvar support, when we don't expect a procvar
          then we need to call the procvar }
        if (left.resultdef.typ<>procvardef) then
          maybe_call_procvar(right,true);

        { assignments to formaldefs and open arrays aren't allowed }
        if is_open_array(left.resultdef) then
          CGMessage(type_e_assignment_not_allowed)
        else if (left.resultdef.typ=formaldef) then
          if not(target_info.system in systems_managed_vm) then
            CGMessage(type_e_assignment_not_allowed)
          else
            begin
              { on managed platforms, assigning to formaldefs is allowed (but
                typecasting them on the left hand side isn't), but primitive
                values need to be boxed first }
              if (right.resultdef.typ in [orddef,floatdef]) then
                begin
                  right:=cinlinenode.create(in_box_x,false,ccallparanode.create(right,nil));
                  typecheckpass(right);
                end;
            end;


        { test if node can be assigned, properties are allowed }
        if not(nf_internal in flags) then
          if not valid_for_assignment(left,true) then
            { errors can in situations that cause the compiler to run out of
              memory, such as assigning to an implicit pointer-to-array
              converted node (that array is 2^31 or 2^63 bytes large) }
            exit;

        { assigning nil or [] to a dynamic array clears the array }
        if is_dynamic_array(left.resultdef) and
            (
              (right.nodetype=niln) or
              (
                (right.nodetype=arrayconstructorn) and
                (right.resultdef.typ=arraydef) and
                (tarraydef(right.resultdef).elementdef=voidtype) and
                tarrayconstructornode(right).isempty
              )
            ) then
         begin
           { remove property flag to avoid errors, see comments for }
           { tf_winlikewidestring assignments below                 }
           exclude(left.flags,nf_isproperty);
           { generate a setlength node so it can be intercepted by
             target-specific code }
           result:=cinlinenode.create(in_setlength_x,false,
             ccallparanode.create(genintconstnode(0),
               ccallparanode.create(left,nil)));
           left:=nil;
           exit;
         end;

        { shortstring helpers can do the conversion directly,
          so treat them separatly }
        if (is_shortstring(left.resultdef)) then
         begin
           { insert typeconv, except for chars that are handled in
             secondpass and except for ansi/wide string that can
             be converted immediatly }
           if not direct_shortstring_assignment then
             inserttypeconv(right,left.resultdef);
           if right.resultdef.typ=stringdef then
            begin
              useshelper:=true;
              { convert constant strings to shortstrings. But
                skip empty constant strings, that will be handled
                in secondpass }
              if (right.nodetype=stringconstn) then
                begin
                  { verify if range fits within shortstring }
                  { just emit a warning, delphi gives an    }
                  { error, only if the type definition of   }
                  { of the string is less  < 255 characters }
                  if not is_open_string(left.resultdef) and
                     (tstringconstnode(right).len > tstringdef(left.resultdef).len) then
                     cgmessage(type_w_string_too_long);
                  inserttypeconv(right,left.resultdef);
                  if (right.nodetype=stringconstn) and
                     (tstringconstnode(right).len=0) then
                    useshelper:=false;
                end
              else if (tstringdef(right.resultdef).stringtype in [st_unicodestring,st_widestring]) then
                Message2(type_w_implicit_string_cast_loss,right.resultdef.typename,left.resultdef.typename);
             { rest is done in pass 1 (JM) }
             if useshelper then
               exit;
            end
         end
        { floating point assignments can also perform the conversion directly }
        else if is_real(left.resultdef) and is_real(right.resultdef) and
                not is_constrealnode(right)
{$ifdef cpufpemu}
                { the emulator can't do this obviously }
                and not(current_settings.fputype in [fpu_libgcc,fpu_soft])
{$endif cpufpemu}

{$ifdef x86}
                { the assignment node code can't convert a double in an }
                { sse register to an extended value in memory more      }
                { efficiently than a type conversion node, so don't     }
                { bother implementing support for that                  }
                and (use_vectorfpu(left.resultdef) or not(use_vectorfpu(right.resultdef)))
{$endif}

{$ifdef arm}
                { the assignment node code can't convert a single in
                  an interger register to a double in an mmregister or
                  vice versa }
                and (use_vectorfpu(left.resultdef) and
                     use_vectorfpu(right.resultdef) and
                     (tfloatdef(left.resultdef).floattype=tfloatdef(right.resultdef).floattype))
{$endif arm}
{$ifdef xtensa}
                and not((FPUXTENSA_SINGLE in fpu_capabilities[current_settings.fputype]) xor
                  (FPUXTENSA_DOUBLE in fpu_capabilities[current_settings.fputype]))
{$endif}
        then
          begin
            if not(nf_internal in flags) then
              check_ranges(fileinfo,right,left.resultdef);
          end
        else
          begin
            { check if the assignment may cause a range check error }
            if not(nf_internal in flags) then
              check_ranges(fileinfo,right,left.resultdef);

            { beginners might be confused about an error message like
              Incompatible types: got "untyped" expected "LongInt"
              when trying to assign the result of a procedure, so give
              a better error message, see also #19122 }
            if (left.resultdef.typ<>procvardef) and
              (right.nodetype=calln) and is_void(right.resultdef) then
              CGMessage(type_e_procedures_return_no_value)
            else if nf_internal in flags then
              inserttypeconv_internal(right,left.resultdef)
            else
              inserttypeconv(right,left.resultdef);
          end;

        { call helpers for interface }
        if is_interfacecom_or_dispinterface(left.resultdef) then
         begin
	   { Normal interface assignments are handled by the generic refcount incr/decr }
           if not def_is_related(right.resultdef,left.resultdef) then
             begin
               { remove property flag to avoid errors, see comments for }
               { tf_winlikewidestring assignments below                 }
               exclude(left.flags,nf_isproperty);
               hp:=
                 ccallparanode.create(
                   cguidconstnode.create(tobjectdef(left.resultdef).iidguid^),
                 ccallparanode.create(
                   ctypeconvnode.create_internal(right,voidpointertype),
                 ccallparanode.create(
                   ctypeconvnode.create_internal(left,voidpointertype),
                   nil)));
               result:=ccallnode.createintern('fpc_intf_assign_by_iid',hp);
               left:=nil;
               right:=nil;
               exit;
             end;
         end;

        { check if local proc/func is assigned to procvar }
        if right.resultdef.typ=procvardef then
          test_local_to_procvar(tprocvardef(right.resultdef),left.resultdef);
      end;


    function tassignmentnode.pass_1 : tnode;
      var
        hp: tnode;
        oldassignmentnode : tassignmentnode;
        hdef: tdef;
        hs: string;
        needrtti: boolean;
      begin
         result:=nil;
         expectloc:=LOC_VOID;

         firstpass(left);

         { Optimize the reuse of the destination of the assingment in left.
           Allow the use of the left inside the tree generated on the right.
           This is especially useful for string routines where the destination
           is pushed as a parameter. Using the final destination of left directly
           save a temp allocation and copy of data (PFV) }
         oldassignmentnode:=aktassignmentnode;
         aktassignmentnode:=self;
         firstpass(right);
         aktassignmentnode:=oldassignmentnode;
         if nf_assign_done_in_right in flags then
           begin
             result:=right;
             right:=nil;
             exit;
           end;

         if codegenerror then
           exit;

         { assignment to refcounted variable -> inc/decref }
         if is_managed_type(left.resultdef) then
           include(current_procinfo.flags,pi_do_call);

         needrtti:=false;

        if (is_shortstring(left.resultdef)) then
          begin
           if right.resultdef.typ=stringdef then
            begin
              if (right.nodetype<>stringconstn) or
                 (tstringconstnode(right).len<>0) then
               begin
                 { remove property flag to avoid errors, see comments for }
                 { tf_winlikewidestring assignments below                 }
                 exclude(left.flags, nf_isproperty);
                 hp:=ccallparanode.create
                       (right,
                  ccallparanode.create(left,nil));
                 result:=ccallnode.createintern('fpc_'+tstringdef(right.resultdef).stringtypname+'_to_shortstr',hp);
                 firstpass(result);
                 left:=nil;
                 right:=nil;
               end;
            end;
            exit;
           end
        { call helpers for composite types containing automated types }
        else if is_managed_type(left.resultdef) and
            (left.resultdef.typ in [arraydef,objectdef,recorddef]) and
            not is_interfacecom_or_dispinterface(left.resultdef) and
            not is_dynamic_array(left.resultdef) and
            not is_const(left) and
            not(target_info.system in systems_garbage_collected_managed_types) then
         begin
           hp:=ccallparanode.create(caddrnode.create_internal(
                  crttinode.create(tstoreddef(left.resultdef),initrtti,rdt_normal)),
               ccallparanode.create(ctypeconvnode.create_internal(
                 caddrnode.create_internal(left),voidpointertype),
               ccallparanode.create(ctypeconvnode.create_internal(
                 caddrnode.create_internal(right),voidpointertype),
               nil)));
           result:=ccallnode.createintern('fpc_copy_proc',hp);
           firstpass(result);
           left:=nil;
           right:=nil;
           exit;
         end
        { call helpers for variant, they can contain non ref. counted types like
          vararrays which must be really copied }
        else if (left.resultdef.typ=variantdef) and
            not(is_const(left)) and
            not(target_info.system in systems_garbage_collected_managed_types)  then
         begin
           { remove property flag to avoid errors, see comments for }
           { tf_winlikewidestring assignments below                 }
           exclude(left.flags,nf_isproperty);
           hdef:=search_system_type('TVARDATA').typedef;
           hp:=ccallparanode.create(ctypeconvnode.create_internal(
                 right,hdef),
               ccallparanode.create(ctypeconvnode.create_internal(
                 left,hdef),
               nil));
           result:=ccallnode.createintern('fpc_variant_copy',hp);
           firstpass(result);
           left:=nil;
           right:=nil;
           exit;
         end
        else if not(target_info.system in systems_garbage_collected_managed_types) and
          not(is_const(left)) then
          begin
            { call helpers for pointer-sized managed types }
            if is_widestring(left.resultdef) then
              hs:='fpc_widestr_assign'
            else if is_ansistring(left.resultdef) then
              hs:='fpc_ansistr_assign'
            else if is_unicodestring(left.resultdef) then
              hs:='fpc_unicodestr_assign'
            else if is_interfacecom_or_dispinterface(left.resultdef) then
              hs:='fpc_intf_assign'
            else if is_dynamic_array(left.resultdef) then
              begin
                hs:='fpc_dynarray_assign';
                needrtti:=true;
              end
            else
              exit;
          end
        else
          exit;

        { The first argument of these procedures is a var parameter. Properties cannot     }
        { be passed to var or out parameters, because in that case setters/getters are not }
        { used. Further, if we would allow it in case there are no getters or setters, you }
        { would need source changes in case these are introduced later on, thus defeating  }
        { part of the transparency advantages of properties. In this particular case,      }
        { however:                                                                         }
        {   a) if there is a setter, this code will not be used since then the assignment  }
        {      will be converted to a procedure call                                       }
        {   b) the getter is irrelevant, because fpc_widestr_assign must always decrease   }
        {      the refcount of the field to which we are writing                           }
        {   c) source code changes are not required if a setter is added/removed, because  }
        {      this transformation is handled at compile time                              }
        {  -> we can remove the nf_isproperty flag (if any) from left, so that in case it  }
        {     is a property which refers to a field without a setter call, we will not get }
        {     an error about trying to pass a property as a var parameter                  }
        exclude(left.flags,nf_isproperty);
        hp:=ccallparanode.create(ctypeconvnode.create_internal(right,voidpointertype),
            ccallparanode.create(ctypeconvnode.create_internal(left,voidpointertype),
            nil));
        if needrtti then
          hp:=ccallparanode.create(
            caddrnode.create_internal(
              crttinode.create(tstoreddef(left.resultdef),initrtti,rdt_normal)),
            hp);
        result:=ccallnode.createintern(hs,hp);
        firstpass(result);
        left:=nil;
        right:=nil;
      end;


    function tassignmentnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (assigntype = tassignmentnode(p).assigntype);
      end;

{$ifdef state_tracking}
    function Tassignmentnode.track_state_pass(exec_known:boolean):boolean;

    var se:Tstate_entry;

    begin
        track_state_pass:=false;
        if exec_known then
            begin
                track_state_pass:=right.track_state_pass(exec_known);
                {Force a new resultdef pass.}
                right.resultdef:=nil;
                do_typecheckpass(right);
                typecheckpass(right);
                aktstate.store_fact(left.getcopy,right.getcopy);
            end
        else
            aktstate.delete_fact(left);
    end;
{$endif}


{$ifdef DEBUG_NODE_XML}
    procedure TAssignmentNode.XMLPrintNodeData(var T: Text);
      begin
        { For assignments, put the left and right branches on the same level for clarity }
        XMLPrintNode(T, Left);
        XMLPrintNode(T, Right);
        PrintNodeUnindent;
        WriteLn(T, PrintNodeIndention, '</', nodetype2str[nodetype], '>');
      end;
{$endif DEBUG_NODE_XML}


{*****************************************************************************
                           TARRAYCONSTRUCTORRANGENODE
*****************************************************************************}

    constructor tarrayconstructorrangenode.create(l,r : tnode);

      begin
         inherited create(arrayconstructorrangen,l,r);
      end;

    function tarrayconstructorrangenode.pass_typecheck:tnode;
      begin
        result:=nil;
        typecheckpass(left);
        typecheckpass(right);
        set_varstate(left,vs_read,[vsf_must_be_valid]);
        set_varstate(right,vs_read,[vsf_must_be_valid]);
        if codegenerror then
         exit;
        resultdef:=left.resultdef;
      end;


    function tarrayconstructorrangenode.pass_1 : tnode;
      begin
        result:=nil;
        CGMessage(parser_e_illegal_expression);
      end;


{****************************************************************************
                            TARRAYCONSTRUCTORNODE
*****************************************************************************}

    constructor tarrayconstructornode.create(l,r : tnode);
      begin
         inherited create(arrayconstructorn,l,r);
         allow_array_constructor:=false;
      end;


    function tarrayconstructornode.dogetcopy : tnode;
      var
         n : tarrayconstructornode;
      begin
         n:=tarrayconstructornode(inherited dogetcopy);
         result:=n;
      end;


    function tarrayconstructornode.has_range_node:boolean;
      var
        n : tarrayconstructornode;
      begin
        result:=false;
        n:=self;
        while assigned(n) do
          begin
            if assigned(n.left) and (n.left.nodetype=arrayconstructorrangen) then
              begin
                result:=true;
                break;
              end;
            n:=tarrayconstructornode(n.right);
          end;
      end;


    function tarrayconstructornode.isempty:boolean;
      begin
        result:=not(assigned(left)) and not(assigned(right));
      end;


    function tarrayconstructornode.pass_typecheck:tnode;
      var
        hdef  : tdef;
        hp    : tarrayconstructornode;
        len   : longint;
        varia : boolean;
        eq    : tequaltype;
        hnodetype : tnodetype;
      begin
        result:=nil;

      { are we allowing array constructor? Then convert it to a set.
        Do this only if we didn't convert the arrayconstructor yet. This
        is needed for the cases where the resultdef is forced for a second
        run }
        if not allow_array_constructor or has_range_node then
         begin
           hp:=tarrayconstructornode(getcopy);
           arrayconstructor_to_set(tnode(hp));
           result:=hp;
           exit;
         end;

      { only pass left tree, right tree contains next construct if any }
        hdef:=nil;
        hnodetype:=errorn;
        len:=0;
        varia:=false;
        if assigned(left) then
         begin
           hp:=self;
           while assigned(hp) do
            begin
              typecheckpass(hp.left);
              set_varstate(hp.left,vs_read,[vsf_must_be_valid]);
              if (hdef=nil) then
                begin
                  hdef:=hp.left.resultdef;
                  hnodetype:=hp.left.nodetype;
                end
              else
               begin
                 { If we got a niln we don't know the type yet and need to take the
                   type of the next array element.
                   This is to handle things like [nil,tclass,tclass], see also tw8371 (PFV) }
                 if hnodetype=niln then
                   begin
                     eq:=compare_defs(hp.left.resultdef,hdef,hnodetype);
                     if eq>te_incompatible then
                       begin
                         hdef:=hp.left.resultdef;
                         hnodetype:=hp.left.nodetype;
                       end;
                   end
                 else
                   eq:=compare_defs(hdef,hp.left.resultdef,hp.left.nodetype);
                 if (not varia) and (eq<te_equal) then
                   begin
                     { If both are integers we need to take the type that can hold both
                       defs }
                     if is_integer(hdef) and is_integer(hp.left.resultdef) then
                       begin
                         if is_in_limit(hdef,hp.left.resultdef) then
                           hdef:=hp.left.resultdef;
                       end
                     else
                       if (nf_novariaallowed in flags) then
                         varia:=true;
                   end;
               end;
              inc(len);
              hp:=tarrayconstructornode(hp.right);
            end;
         end;
         { Set the type of empty or varia arrays to void. Also
           do this if the type is array of const/open array
           because those can't be used with setelementdef }
         if not assigned(hdef) or
            varia or
            is_array_of_const(hdef) or
            is_open_array(hdef) then
           hdef:=voidtype;
         resultdef:=carraydef.create(0,len-1,s32inttype);
         include(tarraydef(resultdef).arrayoptions,ado_IsConstructor);
         if varia then
           include(tarraydef(resultdef).arrayoptions,ado_IsVariant);
         tarraydef(resultdef).elementdef:=hdef;
      end;


    procedure tarrayconstructornode.force_type(def:tdef);
      var
        hp : tarrayconstructornode;
      begin
        tarraydef(resultdef).elementdef:=def;
        include(tarraydef(resultdef).arrayoptions,ado_IsConstructor);
        exclude(tarraydef(resultdef).arrayoptions,ado_IsVariant);
        if assigned(left) then
         begin
           hp:=self;
           while assigned(hp) do
            begin
              inserttypeconv(hp.left,def);
              hp:=tarrayconstructornode(hp.right);
            end;
         end;
      end;


    procedure tarrayconstructornode.insert_typeconvs;
      var
        hp        : tarrayconstructornode;
        dovariant : boolean;
      begin
        dovariant:=(nf_forcevaria in flags) or (ado_isvariant in tarraydef(resultdef).arrayoptions);
        { only pass left tree, right tree contains next construct if any }
        if assigned(left) then
         begin
           hp:=self;
           while assigned(hp) do
            begin
              typecheckpass(hp.left);
              { Insert typeconvs for array of const }
              if dovariant then
                { at this time C varargs are no longer an arrayconstructornode }
                insert_varargstypeconv(hp.left,false);
              hp:=tarrayconstructornode(hp.right);
            end;
         end;
      end;


    function tarrayconstructornode.pass_1 : tnode;
      var
        hp : tarrayconstructornode;
        do_variant,
        do_managed_variant:boolean;
      begin
        do_variant:=(nf_forcevaria in flags) or (ado_isvariant in tarraydef(resultdef).arrayoptions);
        do_managed_variant:=
          do_variant and
          (target_info.system in systems_managed_vm);
        result:=nil;
        { Insert required type convs, this must be
          done in pass 1, because the call must be
          typecheckpassed already }
        if assigned(left) then
          begin
            insert_typeconvs;
            { call firstpass for all nodes }
            hp:=self;
            while assigned(hp) do
              begin
                if hp.left<>nil then
                  begin
                    {This check is pessimistic; a call will happen depending
                     on the location in which the elements will be found in
                     pass 2.}
                    if not do_variant then
                      include(current_procinfo.flags,pi_do_call);
                    firstpass(hp.left);
                    if do_managed_variant then
                      wrapmanagedvarrec(hp.left);
                  end;
                hp:=tarrayconstructornode(hp.right);
              end;
          end;
        { set the elementdef to the correct type in case of a variant array }
        if do_variant then
          tarraydef(resultdef).elementdef:=search_system_type('TVARREC').typedef;
        expectloc:=LOC_CREFERENCE;
      end;


    function tarrayconstructornode.docompare(p: tnode): boolean;
      begin
        docompare:=inherited docompare(p);
      end;


{*****************************************************************************
                              TTYPENODE
*****************************************************************************}

    constructor ttypenode.create(def:tdef);
      begin
         inherited create(typen);
         typedef:=def;
         typesym:=def.typesym;
         allowed:=false;
         helperallowed:=false;
      end;


    constructor ttypenode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getderef(typedefderef);
        ppufile.getderef(typesymderef);
        allowed:=ppufile.getboolean;
        helperallowed:=ppufile.getboolean;
      end;


    procedure ttypenode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putderef(typedefderef);
        ppufile.putderef(typesymderef);
        ppufile.putboolean(allowed);
        ppufile.putboolean(helperallowed);
      end;


    procedure ttypenode.buildderefimpl;
      begin
        inherited buildderefimpl;
        typedefderef.build(typedef);
        typesymderef.build(typesym);
      end;


    procedure ttypenode.derefimpl;
      begin
        inherited derefimpl;
        typedef:=tdef(typedefderef.resolve);
        typesym:=tsym(typesymderef.resolve);
      end;


    function ttypenode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=typedef;
        { check if it's valid }
        if typedef.typ = errordef then
          CGMessage(parser_e_illegal_expression);
      end;


    function ttypenode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_VOID;
         { a typenode can't generate code, so we give here
           an error. Else it'll be an abstract error in pass_generate_code.
           Only when the allowed flag is set we don't generate
           an error }
         if not allowed then
           CGMessage(parser_e_no_type_not_allowed_here);
         if not helperallowed and is_objectpascal_helper(typedef) then
           CGMessage(parser_e_no_category_as_types);
      end;


    function ttypenode.dogetcopy : tnode;
      var
         n : ttypenode;
      begin
         n:=ttypenode(inherited dogetcopy);
         n.allowed:=allowed;
         n.typedef:=typedef;
         n.helperallowed:=helperallowed;
         result:=n;
      end;


    function ttypenode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (typedef=ttypenode(p).typedef) and
          (allowed=ttypenode(p).allowed) and
          (helperallowed=ttypenode(p).helperallowed);
      end;


{*****************************************************************************
                              TRTTINODE
*****************************************************************************}


    constructor trttinode.create(def:tstoreddef;rt:trttitype;dt:Trttidatatype);
      begin
         inherited create(rttin);
         rttidef:=def;
         rttitype:=rt;
         rttidatatype:=dt;
      end;


    constructor trttinode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getderef(rttidefderef);
        rttitype:=trttitype(ppufile.getbyte);
        rttidatatype:=trttidatatype(ppufile.getbyte);
      end;


    procedure trttinode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putderef(rttidefderef);
        ppufile.putbyte(byte(rttitype));
        ppufile.putbyte(byte(rttidatatype));
      end;


    procedure trttinode.buildderefimpl;
      begin
        inherited buildderefimpl;
        rttidefderef.build(rttidef);
      end;


    procedure trttinode.derefimpl;
      begin
        inherited derefimpl;
        rttidef:=tstoreddef(rttidefderef.resolve);
      end;


    function trttinode.dogetcopy : tnode;
      var
         n : trttinode;
      begin
         n:=trttinode(inherited dogetcopy);
         n.rttidef:=rttidef;
         n.rttitype:=rttitype;
         n.rttidatatype:=rttidatatype;
         result:=n;
      end;


    function trttinode.pass_typecheck:tnode;
      begin
        { rtti information will be returned as a void pointer }
        result:=nil;
        resultdef:=voidpointertype;
      end;


    function trttinode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_CREFERENCE;
      end;


    function trttinode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (rttidef = trttinode(p).rttidef) and
          (rttitype = trttinode(p).rttitype) and
          (rttidatatype = trttinode(p).rttidatatype);
      end;

end.
