{
    Copyright (c) 2000-2002 by Florian Klaempfl

    This unit implements some basic nodes

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
unit nbas;

{$i fpcdefs.inc}

interface

    uses
       globtype,
       cgbase,cgutils,
       aasmtai,aasmdata,aasmcpu,
       node,
       symtype;

    type
       tnothingnode = class(tnode)
          constructor create;virtual;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeTree(var T: Text); override;
{$endif DEBUG_NODE_XML}
       end;
       tnothingnodeclass = class of tnothingnode;

       terrornode = class(tnode)
          constructor create;virtual;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          procedure mark_write;override;
       end;
       terrornodeclass = class of terrornode;

       tspecializenode = class(tunarynode)
          sym:tsym;
          getaddr:boolean;
          inheriteddef:tdef;
          constructor create(l:tnode;g:boolean;s:tsym);virtual;
          constructor create_inherited(l:tnode;g:boolean;s:tsym;i:tdef);virtual;
          function pass_1:tnode;override;
          function pass_typecheck:tnode;override;
       end;
       tspecializenodeclass = class of tspecializenode;

       tfinalizetempsnode = class(tnode)
          constructor create;virtual;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode): boolean; override;
       end;
       tfinalizetempsnodeclass = class of tfinalizetempsnode;

       tasmnode = class(tnode)
          p_asm : TAsmList;
          currenttai : tai;
          { Used registers in assembler block }
          has_registerlist : boolean;
          constructor create(p : TAsmList);virtual;
          constructor create_get_position;
          destructor destroy;override;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode): boolean; override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
       end;
       tasmnodeclass = class of tasmnode;

       tstatementnode = class(tbinarynode)
          constructor create(l,r : tnode);virtual;
          function simplify(forinline : boolean) : tnode; override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          procedure printnodetree(var t:text);override;
          property statement : tnode read left write left;
          property next : tnode read right write right;
       end;
       tstatementnodeclass = class of tstatementnode;

       tblocknode = class(tunarynode)
          constructor create(l : tnode);virtual;
          destructor destroy; override;
          function simplify(forinline : boolean) : tnode; override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
{$ifdef state_tracking}
          function track_state_pass(exec_known:boolean):boolean;override;
{$endif state_tracking}
          property statements : tnode read left write left;
       end;
       tblocknodeclass = class of tblocknode;

       ttempcreatenode = class;

       ttempinfoflag = (
         { temp can be kept in a register as far as the original creator is
          concerned }
         ti_may_be_in_reg,
         { the ttempcreatenode has been process and the temp's location is
           valid (-> the ttempdeletenode has not yet been processed, or
           in case it's a "create_to_normal()" one, the final ttemprefnode
           has not yet been processed) }
         ti_valid,
         { when performing a getcopy of a nodetree, we have to hook up the
           copies of ttemprefnodes and ttempdestroynode to the copied
           ttempinfo. this is done by setting hookoncopy in the original
           ttempinfo to point to the new one. if the temp is deleted via a
           regular ttempdeletenode, the hookoncopy is simply set to nil once
           it's processed. otherwise, it sets the ti_nextref_set_hookoncopy_nil
           and after processing the final ttemprefnode, hookoncopy is set to nil
         }
         ti_nextref_set_hookoncopy_nil,
         { the address of this temp is taken (-> cannot be kept in a register,
           even if the creator didn't mind)
         }
         ti_addr_taken,
         { temps can get an extra node tree that contains the value to which
           they should be initialised when they are created. this initialisation
           has to be performed right before the first reference to the temp.
           this flag indicates that the ttempcreatenode has been
           processed by pass_generate_code, but that the first ttemprefnode
           hasn't yet and hence will have to perform the initialisation
         }
         ti_executeinitialisation,
         { in case an expression like "inc(x[func()],1)" is translated into
           a regular addition, you have to create a temp to hold the address
           representing x[func()], since otherwise func() will be called twice
           and that can spell trouble in case it has side effects. on platforms
           without pointers, we cannot just take the address though. this flag
           has to be combined with ti_executeinitialisation above and will,
           rather than loading the value at the calculated location and store
           it in the temp, keep a copy of the calculated location if possible
           and required (not possible for regvars, because SSA may change their
           register, but not required for them either since calculating their
           location has no side-effects
         }
         ti_reference,
         { this temp only allows reading (makes it possible to safely use as
           reference under more circumstances)
         }
         ti_readonly,
         { if this is a managed temp, it doesn't have to be finalised before use
         }
         ti_nofini,
         { the value described by this temp. node is const/immutable, this is important for
           managed types like ansistrings where temp. refs are pointers to the actual value
           -- in this case, assignments to the temp do not increase the
             reference count, and if the assigned value was a temp itself then
             that temp is not deallocated until this temp is deleted (since
             otherwise the assigned value may be freed before the last use of
             the temp) }
         ti_const,
         { the temp. needs no final sync instruction if it is located in a register,
           so there are no loops involved in the usage of the temp.
         }
         ti_no_final_regsync,
         { this applied only to delete nodes: the single purpose of the temp. delete node is to clean up memory. In case
           of cse it might happen that the tempcreate node is optimized away so tempinfo is never initialized properly but
           the allocated memory must be disposed
           If a temp. node has this flag set, the life time of the temp. data must be determined by reg. life, the temp.
           location (in the sense of stack space/register) is never release }
         ti_cleanup_only
         );
       ttempinfoflags = set of ttempinfoflag;

     const
       tempinfostoreflags = [ti_may_be_in_reg,ti_addr_taken,ti_reference,ti_readonly,ti_no_final_regsync];

     type
       { to allow access to the location by temp references even after the temp has }
       { already been disposed and to make sure the coherency between temps and     }
       { temp references is kept after a getcopy                                    }
       ptempinfo = ^ttempinfo;
       ttempinfo = object
        private
         flags                      : ttempinfoflags;
        public
         { set to the copy of a tempcreate pnode (if it gets copied) so that the }
         { refs and deletenode can hook to this copy once they get copied too    }
         hookoncopy                 : ptempinfo;
         typedef                    : tdef;
         typedefderef               : tderef;
         temptype                   : ttemptype;
         owner                      : ttempcreatenode;
         withnode                   : tnode;
         location                   : tlocation;
         tempinitcode               : tnode;
       end;

       ttempinfoaccessor = class
         class procedure settempinfoflags(tempinfo: ptempinfo; const flags: ttempinfoflags); virtual;
         class function gettempinfoflags(tempinfo: ptempinfo): ttempinfoflags; static; inline;
       end;
       ttempinfoaccessorclass = class of ttempinfoaccessor;

       ttempbasenode = class(tnode)
        protected
          class var tempinfoaccessor: ttempinfoaccessorclass;
        protected
          procedure settempinfoflags(const tempflags: ttempinfoflags); inline;
          function gettempinfoflags: ttempinfoflags; inline;
        public
          tempinfo: ptempinfo;
          procedure includetempflag(flag: ttempinfoflag); inline;
          procedure excludetempflag(flag: ttempinfoflag); inline;
          property tempflags: ttempinfoflags read gettempinfoflags write settempinfoflags;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeInfo(var T: Text); override;
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
       end;

       { a node which will create a (non)persistent temp of a given type with a given  }
       { size (the size is separate to allow creating "void" temps with a custom size) }
       ttempcreatenode = class(ttempbasenode)
          size: tcgint;
          ftemplvalue : tnode;
          { * persistent temps are used in manually written code where the temp }
          { be usable among different statements and where you can manually say }
          { when the temp has to be freed (using a ttempdeletenode)             }
          { * non-persistent temps are mostly used in typeconversion helpers,   }
          { where the node that receives the temp becomes responsible for       }
          { freeing it. In this last case, you must use only one reference      }
          { to it and *not* generate a ttempdeletenode                          }
          constructor create(_typedef: tdef; _size: tcgint; _temptype: ttemptype;allowreg:boolean); virtual;
          constructor create_withnode(_typedef: tdef; _size: tcgint; _temptype: ttemptype; allowreg:boolean; withnode: tnode); virtual;
          constructor create_value(_typedef:tdef; _size: tcgint; _temptype: ttemptype;allowreg:boolean; templvalue: tnode);
          constructor create_reference(_typedef:tdef; _size: tcgint; _temptype: ttemptype;allowreg:boolean; templvalue: tnode; readonly: boolean);
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function dogetcopy: tnode; override;
          function pass_1 : tnode; override;
          function pass_typecheck: tnode; override;
          function docompare(p: tnode): boolean; override;
          procedure printnodedata(var t:text);override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
        end;
       ttempcreatenodeclass = class of ttempcreatenode;

        { a node which is a reference to a certain temp }
        ttemprefnode = class(ttempbasenode)
          constructor create(const temp: ttempcreatenode); virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure resolveppuidx;override;
          function dogetcopy: tnode; override;
          function pass_1 : tnode; override;
          function pass_typecheck : tnode; override;
          procedure mark_write;override;
          function docompare(p: tnode): boolean; override;
          procedure printnodedata(var t:text);override;
         private
          tempidx : longint;
        end;
       ttemprefnodeclass = class of ttemprefnode;

        { a node which removes a temp }
        ttempdeletenode = class(ttempbasenode)
          constructor create(const temp: ttempcreatenode); virtual;
          { this will convert the persistant temp to a normal temp
            for returning to the other nodes }
          constructor create_normal_temp(const temp: ttempcreatenode);
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure resolveppuidx;override;
          function dogetcopy: tnode; override;
          function pass_1: tnode; override;
          function pass_typecheck: tnode; override;
          function docompare(p: tnode): boolean; override;
          destructor destroy; override;
          procedure printnodedata(var t:text);override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
         protected
          release_to_normal : boolean;
        private
          tempidx : longint;
        end;
       ttempdeletenodeclass = class of ttempdeletenode;

    var
       cnothingnode : tnothingnodeclass = tnothingnode;
       cerrornode : terrornodeclass = terrornode;
       cspecializenode : tspecializenodeclass = tspecializenode;
       cfinalizetempsnode: tfinalizetempsnodeclass = tfinalizetempsnode;
       casmnode : tasmnodeclass = tasmnode;
       cstatementnode : tstatementnodeclass = tstatementnode;
       cblocknode : tblocknodeclass = tblocknode;
       ctempinfoaccessor : ttempinfoaccessorclass = ttempinfoaccessor;
       ctempcreatenode : ttempcreatenodeclass = ttempcreatenode;
       ctemprefnode : ttemprefnodeclass = ttemprefnode;
       ctempdeletenode : ttempdeletenodeclass = ttempdeletenode;

       { Create a blocknode and statement node for multiple statements
         generated internally by the parser }
       function  internalstatements(out laststatement:tstatementnode):tblocknode;
       function  laststatement(block:tblocknode):tstatementnode;
       procedure addstatement(var laststatement:tstatementnode;n:tnode);

       { if the complexity of n is "high", creates a reference temp to n's
         location and replace n with a ttemprefnode referring to that location }
       function maybereplacewithtempref(var n: tnode; var block: tblocknode; var stat: tstatementnode; size: ASizeInt; readonly: boolean): ttempcreatenode;
       { same as above, but create a regular temp rather than reference temp }
       function maybereplacewithtemp(var n: tnode; var block: tblocknode; var stat: tstatementnode; size: ASizeInt; allowreg: boolean): ttempcreatenode;

implementation

    uses
      verbose,globals,systems,
      ppu,
      symconst,symdef,defutil,defcmp,
      pass_1,
      nutils,nld,
      procinfo
{$ifdef DEBUG_NODE_XML}
{$ifndef jvm}
      ,
      cpubase,
      cutils,
      itcpugas
{$endif jvm}
{$endif DEBUG_NODE_XML}
      ;


{*****************************************************************************
                                     Helpers
*****************************************************************************}

    function internalstatements(out laststatement:tstatementnode):tblocknode;
      begin
        { create dummy initial statement }
        laststatement := cstatementnode.create(cnothingnode.create,nil);
        internalstatements := cblocknode.create(laststatement);
      end;


    function laststatement(block:tblocknode):tstatementnode;
      begin
        result:=tstatementnode(block.left);
        while assigned(result) and assigned(result.right) do
          result:=tstatementnode(result.right);
      end;


    procedure addstatement(var laststatement:tstatementnode;n:tnode);
      begin
        if assigned(laststatement.right) then
          internalerror(200204201);
        laststatement.right:=cstatementnode.create(n,nil);
        laststatement:=tstatementnode(laststatement.right);
      end;


    function maybereplacewithtempref(var n: tnode; var block: tblocknode; var stat: tstatementnode; size: ASizeInt; readonly: boolean): ttempcreatenode;
      begin
        result:=nil;
        if (node_complexity(n)>4) or
           might_have_sideeffects(n) then
          begin
            result:=ctempcreatenode.create_reference(n.resultdef,size,tt_persistent,true,n,readonly);
            typecheckpass(tnode(result));
            n:=ctemprefnode.create(result);
            typecheckpass(n);
            if not assigned(stat) then
              block:=internalstatements(stat);
            addstatement(stat,result)
          end;
      end;

    function maybereplacewithtemp(var n: tnode; var block: tblocknode; var stat: tstatementnode; size: ASizeInt; allowreg: boolean): ttempcreatenode;
      begin
        result:=nil;
        if (node_complexity(n)>4) or
           might_have_sideeffects(n) then
          begin
            result:=ctempcreatenode.create_value(n.resultdef,size,tt_persistent,allowreg,n);
            typecheckpass(tnode(result));
            n:=ctemprefnode.create(result);
            typecheckpass(n);
            if not assigned(stat) then
              block:=internalstatements(stat);
            addstatement(stat,result)
          end;
      end;

{*****************************************************************************
                             TFIRSTNOTHING
*****************************************************************************}

    constructor tnothingnode.create;
      begin
        inherited create(nothingn);
      end;


    function tnothingnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=voidtype;
      end;


    function tnothingnode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_VOID;
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TNothingNode.XMLPrintNodeTree(var T: Text);
      begin
        Write(T, PrintNodeIndention, '<', nodetype2str[nodetype]);
        XMLPrintNodeInfo(T);
        { "Nothing nodes" contain no data, so just use "/>" to terminate it early }
        WriteLn(T, ' />');
      end;
{$endif DEBUG_NODE_XML}

{*****************************************************************************
                             TFIRSTERROR
*****************************************************************************}

    constructor terrornode.create;

      begin
         inherited create(errorn);
      end;


    function terrornode.pass_typecheck:tnode;
      begin
         result:=nil;
         include(flags,nf_error);
         codegenerror:=true;
         resultdef:=generrordef;
      end;


    function terrornode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_VOID;
         codegenerror:=true;
      end;


    procedure terrornode.mark_write;
      begin
      end;


{*****************************************************************************
                             TSPECIALIZENODE
*****************************************************************************}

    constructor tspecializenode.create(l:tnode;g:boolean;s:tsym);
      begin
         inherited create(specializen,l);
         sym:=s;
         getaddr:=g;
      end;

    constructor tspecializenode.create_inherited(l:tnode;g:boolean;s:tsym;i:tdef);
      begin
        create(l,g,s);
        inheriteddef:=i;
      end;


    function tspecializenode.pass_typecheck:tnode;
      begin
         result:=nil;
         resultdef:=cundefinedtype;
      end;


    function tspecializenode.pass_1:tnode;
      begin
         { such a node should not reach pass_1 }
         internalerror(2015071704);
         result:=nil;
         expectloc:=LOC_VOID;
         codegenerror:=true;
      end;


{*****************************************************************************
                             TFINALIZETEMPSNODE
*****************************************************************************}

    constructor tfinalizetempsnode.create;
      begin
        inherited create(finalizetempsn);
      end;

    function tfinalizetempsnode.pass_1: tnode;
      begin
        result:=nil;
        expectloc:=LOC_VOID;
      end;

    function tfinalizetempsnode.pass_typecheck: tnode;
      begin
        resultdef:=voidtype;
        result:=nil;
      end;

    function tfinalizetempsnode.docompare(p: tnode): boolean;
      begin
        { these nodes should never be coalesced }
        result:=false;
      end;


{*****************************************************************************
                            TSTATEMENTNODE
*****************************************************************************}

    constructor tstatementnode.create(l,r : tnode);

      begin
         inherited create(statementn,l,r);
      end;


    function is_exit_statement(var n: tnode; arg: pointer): foreachnoderesult;
      begin
        if (n.nodetype<>exitn) then
          result:=fen_false
        else
          result:=fen_norecurse_true;
      end;


    function no_exit_statement_in_block(n: tnode): boolean;
      begin
        result:=not foreachnodestatic(n,@is_exit_statement,nil);
      end;


    function tstatementnode.simplify(forinline: boolean) : tnode;
      begin
        result:=nil;
        { these "optimizations" are only to make it more easy to recognise    }
        { blocknodes which at the end of inlining only contain one single     }
        { statement. Simplifying inside blocknode.simplify could be dangerous }
        { because if the main blocknode which makes up a procedure/function   }
        { body were replaced with a statementn/nothingn, this could cause     }
        { problems elsewhere in the compiler which expects a blocknode        }

        { remove next statement if it's a nothing-statement (since if it's }
        { the last, it won't remove itself -- see next simplification)     }
        while assigned(right) and
              (tstatementnode(right).left.nodetype = nothingn) do
          begin
            result:=tstatementnode(right).right;
            tstatementnode(right).right:=nil;
            right.free;
            right:=result;
            result:=nil;
          end;

        { Remove initial nothingn if there are other statements. If there }
        { are no other statements, returning nil doesn't help (will be    }
        { interpreted as "can't be simplified") and replacing the         }
        { statementnode with a nothingnode cannot be done (because it's   }
        { possible this statementnode is a child of a blocknode, and      }
        { blocknodes are expected to only contain statementnodes)         }
        if (left.nodetype = nothingn) and
           assigned(right) then
          begin
            result:=right;
            right:=nil;
            exit;
          end;

        { if the current statement contains a block with one statement,
          replace the current statement with that block's statement
          (but only if the block does not have nf_block_with_exit set
           or has no exit statement, because otherwise it needs an own
           exit label, see tests/test/tinline10)

           Further, it might not be the user code entry
        }
        if (left.nodetype = blockn) and
           ((left.flags*[nf_block_with_exit,nf_usercode_entry]=[]) or
            ((left.flags*[nf_block_with_exit,nf_usercode_entry]=[nf_block_with_exit]) and no_exit_statement_in_block(left))) and
           assigned(tblocknode(left).left) and
           not assigned(tstatementnode(tblocknode(left).left).right) then
          begin
            result:=tblocknode(left).left;
            tstatementnode(result).right:=right;
            right:=nil;
            tblocknode(left).left:=nil;
            exit;
          end;
      end;


    function tstatementnode.pass_typecheck:tnode;
      begin
         result:=nil;
         resultdef:=voidtype;

         { left is the statement itself calln assignn or a complex one }
         typecheckpass(left);
         if codegenerror then
           exit;

         { right is the next statement in the list }
         if assigned(right) then
           typecheckpass(right);
         if codegenerror then
           exit;
      end;


    function tstatementnode.pass_1 : tnode;
      begin
         result:=nil;
         { left is the statement itself calln assignn or a complex one }
         firstpass(left);
         if codegenerror then
           exit;
         expectloc:=left.expectloc;
         { right is the next in the list }
         if assigned(right) then
           firstpass(right);
         if codegenerror then
           exit;
      end;


    procedure tstatementnode.printnodetree(var t:text);
      begin
        printnodelist(t);
      end;

{*****************************************************************************
                             TBLOCKNODE
*****************************************************************************}

    constructor tblocknode.create(l : tnode);

      begin
         inherited create(blockn,l);
      end;

    destructor tblocknode.destroy;

      var
        hp, next: tstatementnode;
      begin
        hp := tstatementnode(left);
        left := nil;
        while assigned(hp) do
          begin
            next := tstatementnode(hp.right);
            hp.right := nil;
            hp.free;
            hp := next;
          end;
        inherited destroy;
      end;


    function tblocknode.simplify(forinline : boolean): tnode;
      begin
        result := nil;
        { Warning: never replace a blocknode with another node type,      }
        {  since the block may be the main block of a procedure/function/ }
        {  main program body, and those nodes should always be blocknodes }
        {  since that's what the compiler expects elsewhere.              }

        if assigned(left) and
           not assigned(tstatementnode(left).right) then
          begin
            case tstatementnode(left).left.nodetype of
              blockn:
                begin
                  { if the current block contains only one statement, and
                    this one statement only contains another block, replace
                    this block with that other block.                       }
                  result:=tstatementnode(left).left;
                  tstatementnode(left).left:=nil;
                  { make sure the nf_block_with_exit flag is safeguarded }
                  result.flags:=result.flags+(flags*[nf_block_with_exit,nf_usercode_entry]);
                  exit;
                end;
              nothingn:
                begin
                  { if the block contains only a statement with a nothing node,
                    get rid of the statement }
                  left.Free;
                  left:=nil;
                  exit;
                end;
              else
                ;
            end;
          end;
      end;


    function tblocknode.pass_typecheck:tnode;
      var
         hp : tstatementnode;
      begin
         result:=nil;
         resultdef:=voidtype;

         hp:=tstatementnode(left);
         while assigned(hp) do
           begin
              if assigned(hp.left) then
                begin
                   codegenerror:=false;
                   typecheckpass(hp.left);
                   { the resultdef of the block is the last type that is
                     returned. Normally this is a voidtype. But when the
                     compiler inserts a block of multiple statements then the
                     last entry can return a value }
                   resultdef:=hp.left.resultdef;
                end;
              hp:=tstatementnode(hp.right);
           end;
      end;


    function tblocknode.pass_1 : tnode;
      var
         hp : tstatementnode;
         //count : longint;
      begin
         result:=nil;
         expectloc:=LOC_VOID;
         //count:=0;
         hp:=tstatementnode(left);
         while assigned(hp) do
           begin
              if assigned(hp.left) then
                begin
                   codegenerror:=false;
                   firstpass(hp.left);
                   hp.expectloc:=hp.left.expectloc;
                end;
              expectloc:=hp.expectloc;
              //inc(count);
              hp:=tstatementnode(hp.right);
           end;
      end;

{$ifdef state_tracking}
      function Tblocknode.track_state_pass(exec_known:boolean):boolean;

      var hp:Tstatementnode;

      begin
        track_state_pass:=false;
        hp:=Tstatementnode(left);
        while assigned(hp) do
            begin
                if hp.left.track_state_pass(exec_known) then
                    track_state_pass:=true;
                hp:=Tstatementnode(hp.right);
            end;
      end;
{$endif state_tracking}

{*****************************************************************************
                             TASMNODE
*****************************************************************************}

    constructor tasmnode.create(p : TAsmList);
      begin
        inherited create(asmn);
        p_asm:=p;
        currenttai:=nil;
      end;


    constructor tasmnode.create_get_position;
      begin
        inherited create(asmn);
        p_asm:=nil;
        include(flags,nf_get_asm_position);
        currenttai:=nil;
      end;


    destructor tasmnode.destroy;
      begin
        if assigned(p_asm) then
         p_asm.free;
        inherited destroy;
      end;


    constructor tasmnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      var
        hp : tai;
      begin
        inherited ppuload(t,ppufile);
        if not(nf_get_asm_position in flags) then
          begin
            p_asm:=TAsmList.create;
            repeat
              hp:=ppuloadai(ppufile);
              if hp=nil then
                break;
              p_asm.concat(hp);
              if hp.typ=ait_section then
                inc(p_asm.section_count);
            until false;
          end
        else
          p_asm:=nil;
        currenttai:=nil;
      end;


    procedure tasmnode.ppuwrite(ppufile:tcompilerppufile);
      var
        hp : tai;
      begin
        inherited ppuwrite(ppufile);
{ TODO: FIXME Add saving of register sets}
        if not(nf_get_asm_position in flags) then
          begin
            hp:=tai(p_asm.first);
            while assigned(hp) do
             begin
               ppuwriteai(ppufile,hp);
               hp:=tai(hp.next);
             end;
            { end is marked by a nil }
            ppuwriteai(ppufile,nil);
          end;
      end;


    procedure tasmnode.buildderefimpl;
      var
        hp : tai;
      begin
        inherited buildderefimpl;
        if not(nf_get_asm_position in flags) then
          begin
            hp:=tai(p_asm.first);
            while assigned(hp) do
             begin
               hp.buildderefimpl;
               hp:=tai(hp.next);
             end;
          end;
      end;


    procedure tasmnode.derefimpl;
      var
        hp : tai;
      begin
        inherited derefimpl;
        if not(nf_get_asm_position in flags) then
          begin
            hp:=tai(p_asm.first);
            while assigned(hp) do
             begin
               hp.derefimpl;
               hp:=tai(hp.next);
             end;
          end;
      end;


    function tasmnode.dogetcopy: tnode;
      var
        n: tasmnode;
      begin
        n := tasmnode(inherited dogetcopy);
        if assigned(p_asm) then
          begin
            n.p_asm:=TAsmList.create;
            n.p_asm.concatlistcopy(p_asm);
          end
        else n.p_asm := nil;
        n.currenttai:=currenttai;
        n.has_registerlist:=has_registerlist;
        result:=n;
      end;


    function tasmnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=voidtype;
        if not(nf_get_asm_position in flags) then
          include(current_procinfo.flags,pi_has_assembler_block);
      end;


    function tasmnode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_VOID;
      end;


    function tasmnode.docompare(p: tnode): boolean;
      begin
        { comparing of asmlists is not implemented (JM) }
        docompare := false;
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TAsmNode.XMLPrintNodeData(var T: Text);

      procedure PadString(var S: string; Len: Integer);
        var
          X, C: Integer;
        begin
          C := Length(S);
          if C < Len then
            begin
              SetLength(S, 7);
              for X := C + 1 to Len do
                S[X] := ' '
            end;
        end;

{$ifndef jvm}
      function FormatOp(const Oper: POper): string;
        begin
          case Oper^.typ of
            top_const:
              begin
                case Oper^.val of
                  -15..15:
                    Result := '$' + tostr(Oper^.val);
                  $10..$FF:
                    Result := '$0x' + hexstr(Oper^.val, 2);
                  $100..$FFFF:
                    Result := '$0x' + hexstr(Oper^.val, 4);
                  $10000..$FFFFFFFF:
                    Result := '$0x' + hexstr(Oper^.val, 8);
                  else
                    Result := '$0x' + hexstr(Oper^.val, 16);
                end;
              end;
            top_reg:
              Result := gas_regname(Oper^.reg);
            top_ref:
              with Oper^.ref^ do
                begin
{$if defined(x86)}
                  if segment <> NR_NO then
                    Result := gas_regname(segment) + ':'
                  else
                    Result := '';
{$endif defined(x86)}

                  if Assigned(symbol) then
                    begin
                      Result := Result + symbol.Name;
                      if offset > 0 then
                        Result := Result + '+';
                    end;

                  if offset <> 0 then
                    Result := Result + tostr(offset)
                  else
                    Result := Result;

                  if (base <> NR_NO) or (index <> NR_NO) then
                    begin
                      Result := Result + '(';

                      if base <> NR_NO then
                        begin
                          Result := Result + gas_regname(base);
                          if index <> NR_NO then
                            Result := Result + ',';
                        end;

                      if index <> NR_NO then
                        Result := Result + gas_regname(index);

                      if scalefactor <> 0 then
                        Result := Result + ',' + tostr(scalefactor) + ')'
                      else
                        Result := Result + ')';
                    end;
                end;
            top_bool:
              begin
                if Oper^.b then
                  Result := 'TRUE'
                else
                  Result := 'FALSE';
              end
            else
              Result := '';
          end;
        end;

{$if defined(x86)}
      procedure ProcessInstruction(p: tai); inline;
        var
          ThisOp, ThisOper: string;
          X: Integer;
        begin
          case p.typ of
            ait_label:
              WriteLn(T, PrintNodeIndention, tai_label(p).labsym.name);

            ait_instruction:
              begin
                ThisOp := gas_op2str[taicpu(p).opcode]+cond2str[taicpu(p).condition];
                if gas_needsuffix[taicpu(p).opcode] <> AttSufNONE then
                  ThisOp := ThisOp + gas_opsize2str[taicpu(p).opsize];

                { Pad the opcode with spaces so the succeeding operands are aligned }
                PadString(ThisOp, 7);

                Write(T, PrintNodeIndention, '  ', ThisOp); { Extra indentation to account for label formatting }
                for X := 0 to taicpu(p).ops - 1 do
                  begin
                    Write(T, ' ');

                    ThisOper := FormatOp(taicpu(p).oper[X]);
                    if X < taicpu(p).ops - 1 then
                      begin
                        ThisOper := ThisOper + ',';
                        PadString(ThisOper, 7);
                      end;

                    Write(T, ThisOper);
                  end;
                WriteLn(T);
              end;
            else
              { Do nothing };
          end;
        end;

      var
        hp: tai;
      begin
        if not Assigned(p_asm) then
          Exit;

        hp := tai(p_asm.First);
        while Assigned(hp) do
          begin
            ProcessInstruction(hp);
            hp := tai(hp.Next);
          end;
{$else defined(x86)}
      begin
        WriteLn(T, PrintNodeIndention, '(Assembler output not currently supported on this platform)');
{$endif defined(x86)}
{$else jvm}
      begin
        WriteLn(T, PrintNodeIndention, '(Should assembly language even be possible under JVM?)');
{$endif jvm}
      end;
{$endif DEBUG_NODE_XML}

{*****************************************************************************
                          TEMPBASENODE
*****************************************************************************}

    class procedure ttempinfoaccessor.settempinfoflags(tempinfo: ptempinfo; const flags: ttempinfoflags);
      begin
        tempinfo^.flags:=flags;
      end;


    class function ttempinfoaccessor.gettempinfoflags(tempinfo: ptempinfo): ttempinfoflags;
      begin
        result:=tempinfo^.flags;
      end;


{*****************************************************************************
                          TEMPBASENODE
*****************************************************************************}

    procedure ttempbasenode.settempinfoflags(const tempflags: ttempinfoflags);
      begin
        ctempinfoaccessor.settempinfoflags(tempinfo,tempflags);
      end;


    function ttempbasenode.gettempinfoflags: ttempinfoflags;
      begin
        result:=ctempinfoaccessor.gettempinfoflags(tempinfo);
      end;


    procedure ttempbasenode.includetempflag(flag: ttempinfoflag);
      begin
        { go through settempinfoflags() so it can filter out unsupported tempflags }
        settempinfoflags(gettempinfoflags+[flag])
      end;


    procedure ttempbasenode.excludetempflag(flag: ttempinfoflag);
      begin
        { go through settempinfoflags() so it can prevent required tempflags from
          being removed (if any) }
        settempinfoflags(gettempinfoflags-[flag])
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TTempBaseNode.XMLPrintNodeInfo(var T: Text);
      begin
        inherited XMLPrintNodeInfo(T);

        { The raw pointer is the only way to uniquely identify the temp }
        Write(T, ' id="', WritePointer(tempinfo), '"');
      end;


    procedure TTempBaseNode.XMLPrintNodeData(var T: Text);
      var
        Flag: TTempInfoFlag;
        NotFirst: Boolean;
      begin
        inherited XMLPrintNodeData(t);

        if not assigned(tempinfo) then
          exit;

        WriteLn(T, PrintNodeIndention, '<typedef>', SanitiseXMLString(tempinfo^.typedef.typesymbolprettyname), '</typedef>');

        NotFirst := False;
        for Flag := Low(TTempInfoFlag) to High(TTempInfoFlag) do
          if (Flag in tempinfo^.flags) then
            if not NotFirst then
              begin
                Write(T, PrintNodeIndention, '<tempflags>', Flag);
                NotFirst := True;
              end
            else
              Write(T, ',', Flag);

        if NotFirst then
          WriteLn(T, '</tempflags>')
        else
          WriteLn(T, PrintNodeIndention, '<tempflags />');

        WriteLn(T, PrintNodeIndention, '<temptype>', tempinfo^.temptype, '</temptype>');
      end;
{$endif DEBUG_NODE_XML}

{*****************************************************************************
                          TEMPCREATENODE
*****************************************************************************}

    constructor ttempcreatenode.create(_typedef:tdef; _size: tcgint; _temptype: ttemptype;allowreg:boolean);
      begin
        inherited create(tempcreaten);
        size := _size;
        new(tempinfo);
        fillchar(tempinfo^,sizeof(tempinfo^),0);
        tempinfo^.typedef := _typedef;
        tempinfo^.temptype := _temptype;
        tempinfo^.owner := self;
        tempinfo^.withnode := nil;
        if allowreg and
           { temp must fit a single register }
           (tstoreddef(_typedef).is_fpuregable or
            (tstoreddef(_typedef).is_intregable and
             (_size<=TCGSize2Size[OS_64]))) and
           { size of register operations must be known }
           (def_cgsize(_typedef)<>OS_NO) and
           { no init/final needed }
           not is_managed_type(_typedef) then
          includetempflag(ti_may_be_in_reg);
      end;


    constructor ttempcreatenode.create_withnode(_typedef: tdef; _size: tcgint; _temptype: ttemptype; allowreg:boolean; withnode: tnode);
      begin
        self.create(_typedef,_size,_temptype,allowreg);
        tempinfo^.withnode:=withnode.getcopy;
      end;


    constructor ttempcreatenode.create_value(_typedef:tdef; _size: tcgint; _temptype: ttemptype;allowreg:boolean; templvalue: tnode);
      begin
        self.create(_typedef,_size,_temptype,allowreg);
        // store in ppuwrite
        ftemplvalue:=templvalue;
        // create from stored ftemplvalue in ppuload
        tempinfo^.tempinitcode:=cassignmentnode.create(ctemprefnode.create(self),ftemplvalue);
      end;


     constructor ttempcreatenode.create_reference(_typedef: tdef; _size: tcgint; _temptype: ttemptype; allowreg: boolean; templvalue: tnode; readonly: boolean);
      begin
        // store in ppuwrite
        self.create(_typedef,_size,_temptype,allowreg);
        ftemplvalue:=templvalue;
        // no assignment node, just the tempvalue
        tempinfo^.tempinitcode:=ftemplvalue;
        includetempflag(ti_reference);
        if readonly then
          includetempflag(ti_readonly);
      end;


    function ttempcreatenode.dogetcopy: tnode;
      var
        n: ttempcreatenode;
      begin
        n := ttempcreatenode(inherited dogetcopy);
        n.size := size;

        new(n.tempinfo);
        fillchar(n.tempinfo^,sizeof(n.tempinfo^),0);
        n.tempinfo^.owner:=n;
        n.tempinfo^.typedef := tempinfo^.typedef;
        n.tempinfo^.temptype := tempinfo^.temptype;
        n.tempflags := tempflags * tempinfostoreflags;

        { when the tempinfo has already a hookoncopy then it is not
          reset by a tempdeletenode }
        if assigned(tempinfo^.hookoncopy) then
          internalerror(200211262);
        { signal the temprefs that the temp they point to has been copied, }
        { so that if the refs get copied as well, they can hook themselves }
        { to the copy of the temp                                          }
        tempinfo^.hookoncopy := n.tempinfo;
        excludetempflag(ti_nextref_set_hookoncopy_nil);

        if assigned(tempinfo^.withnode) then
          n.tempinfo^.withnode := tempinfo^.withnode.getcopy
        else
          n.tempinfo^.withnode := nil;

        if assigned(tempinfo^.tempinitcode) then
          n.tempinfo^.tempinitcode := tempinfo^.tempinitcode.getcopy
        else
          n.tempinfo^.tempinitcode := nil;

        result := n;
      end;


    constructor ttempcreatenode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);

        size:=ppufile.getlongint;
        new(tempinfo);
        fillchar(tempinfo^,sizeof(tempinfo^),0);
        ppufile.getset(tppuset2(tempinfo^.flags));
        ppufile.getderef(tempinfo^.typedefderef);
        tempinfo^.temptype := ttemptype(ppufile.getbyte);
        tempinfo^.owner:=self;
        tempinfo^.withnode:=ppuloadnode(ppufile);
        ftemplvalue:=ppuloadnode(ppufile);
      end;


    procedure ttempcreatenode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putlongint(size);
        ppufile.putset(tppuset2(tempinfo^.flags));
        ppufile.putderef(tempinfo^.typedefderef);
        ppufile.putbyte(byte(tempinfo^.temptype));
        ppuwritenode(ppufile,tempinfo^.withnode);
        ppuwritenode(ppufile,ftemplvalue);
      end;


    procedure ttempcreatenode.buildderefimpl;
      begin
        inherited buildderefimpl;
        tempinfo^.typedefderef.build(tempinfo^.typedef);
        if assigned(tempinfo^.withnode) then
          tempinfo^.withnode.buildderefimpl;
        if assigned(ftemplvalue) then
          ftemplvalue.buildderefimpl;
      end;


    procedure ttempcreatenode.derefimpl;
      begin
        inherited derefimpl;
        tempinfo^.typedef:=tdef(tempinfo^.typedefderef.resolve);
        if assigned(tempinfo^.withnode) then
          tempinfo^.withnode.derefimpl;
        if assigned(ftemplvalue) then
          begin
            ftemplvalue.derefimpl;
            tempinfo^.tempinitcode:=cassignmentnode.create(ctemprefnode.create(self),ftemplvalue);
          end;
      end;


    function ttempcreatenode.pass_1 : tnode;
      begin
        result := nil;
        expectloc:=LOC_VOID;
        { temps which are immutable do not need to be initialized/finalized }
        if (tempinfo^.typedef.needs_inittable) and not(ti_const in tempflags) then
          include(current_procinfo.flags,pi_needs_implicit_finally);
        if assigned(tempinfo^.withnode) then
          firstpass(tempinfo^.withnode);
        if assigned(tempinfo^.tempinitcode) then
          firstpass(tempinfo^.tempinitcode);
        inc(current_procinfo.estimatedtempsize,size);;
      end;


    function ttempcreatenode.pass_typecheck: tnode;
      begin
        result := nil;
        { a tempcreatenode doesn't have a resultdef, only temprefnodes do }
        resultdef := voidtype;
        if assigned(tempinfo^.withnode) then
          typecheckpass(tempinfo^.withnode);
        if assigned(tempinfo^.tempinitcode) then
          typecheckpass(tempinfo^.tempinitcode);
      end;


    function ttempcreatenode.docompare(p: tnode): boolean;
      begin
        result :=
          inherited docompare(p) and
          (ttempcreatenode(p).size = size) and
          (ttempcreatenode(p).tempflags*tempinfostoreflags=tempflags*tempinfostoreflags) and
          equal_defs(ttempcreatenode(p).tempinfo^.typedef,tempinfo^.typedef) and
          (ttempcreatenode(p).tempinfo^.withnode.isequal(tempinfo^.withnode)) and
          (ttempcreatenode(p).tempinfo^.tempinitcode.isequal(tempinfo^.tempinitcode));
      end;


    procedure ttempcreatenode.printnodedata(var t:text);
      begin
        inherited printnodedata(t);
        writeln(t,printnodeindention,'size = ',size,', temptypedef = ',tempinfo^.typedef.typesymbolprettyname,' = "',
          tempinfo^.typedef.GetTypeName,'", tempinfo = $',hexstr(ptrint(tempinfo),sizeof(ptrint)*2));
        writeln(t,printnodeindention,'tempinit =');
        printnode(t,tempinfo^.tempinitcode);
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TTempCreateNode.XMLPrintNodeData(var T: Text);
      begin
        inherited XMLPrintNodeData(T);
        WriteLn(T, PrintNodeIndention, '<size>', size, '</size>');
        if Assigned(TempInfo^.TempInitCode) then
          begin
            WriteLn(T, PrintNodeIndention, '<tempinit>');
            PrintNodeIndent;
            XMLPrintNode(T, TempInfo^.TempInitCode);
            PrintNodeUnindent;
            WriteLn(T, PrintNodeIndention, '</tempinit>');
          end
        else
          WriteLn(T, PrintNodeIndention, '<tempinit />');
      end;
{$endif DEBUG_NODE_XML}

{*****************************************************************************
                             TEMPREFNODE
*****************************************************************************}

    constructor ttemprefnode.create(const temp: ttempcreatenode);
      begin
        inherited create(temprefn);
        tempinfo := temp.tempinfo;
      end;


    function ttemprefnode.dogetcopy: tnode;
      var
        n: ttemprefnode;
      begin
        n := ttemprefnode(inherited dogetcopy);

        if assigned(tempinfo^.hookoncopy) then
          { if the temp has been copied, assume it becomes a new }
          { temp which has to be hooked by the copied reference  }
          begin
            { hook the ref to the copied temp }
            n.tempinfo := tempinfo^.hookoncopy;
            { if we passed a ttempdeletenode that changed the temp }
            { from a persistent one into a normal one, we must be  }
            { the last reference (since our parent should free the }
            { temp (JM)                                            }
            if (ti_nextref_set_hookoncopy_nil in tempflags) then
              tempinfo^.hookoncopy := nil;
          end
        else
          { if the temp we refer to hasn't been copied, assume }
          { we're just a new reference to that temp            }
          begin
            n.tempinfo := tempinfo;
          end;

        if not assigned(n.tempinfo) then
          internalerror(2005071901);

        result := n;
      end;


    constructor ttemprefnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        tempidx:=ppufile.getlongint;
      end;


    procedure ttemprefnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putlongint(tempinfo^.owner.ppuidx);
      end;


    procedure ttemprefnode.resolveppuidx;
      var
        temp : ttempcreatenode;
      begin
        temp:=ttempcreatenode(nodeppuidxget(tempidx));
        if temp.nodetype<>tempcreaten then
          internalerror(200311075);
        tempinfo:=temp.tempinfo;
      end;


    function ttemprefnode.pass_1 : tnode;
      begin
        expectloc := LOC_REFERENCE;
        if not tempinfo^.typedef.needs_inittable and
           (ti_may_be_in_reg in tempflags) then
          begin
            if tempinfo^.typedef.typ=floatdef then
              begin
                if not use_vectorfpu(tempinfo^.typedef) then
                  if (tempinfo^.temptype = tt_persistent) then
                    expectloc := LOC_CFPUREGISTER
                  else
                    expectloc := LOC_FPUREGISTER
                else
                  if (tempinfo^.temptype = tt_persistent) then
                    expectloc := LOC_CMMREGISTER
                  else
                    expectloc := LOC_MMREGISTER
              end
            else
              begin
                if (tempinfo^.temptype = tt_persistent) then
                  expectloc := LOC_CREGISTER
                else
                  expectloc := LOC_REGISTER;
              end;
          end;
        result := nil;
      end;


    function ttemprefnode.pass_typecheck: tnode;
      begin
        { check if the temp is already resultdef passed }
        if not assigned(tempinfo^.typedef) then
          internalerror(200108233);
        result := nil;
        resultdef := tempinfo^.typedef;
      end;


    function ttemprefnode.docompare(p: tnode): boolean;
      begin
        result :=
          inherited docompare(p) and
          (ttemprefnode(p).tempinfo = tempinfo);
      end;


    procedure ttemprefnode.mark_write;
      begin
        include(flags,nf_write);
      end;


    procedure ttemprefnode.printnodedata(var t:text);
      var
        f : ttempinfoflag;
        notfirst : Boolean;
      begin
        inherited printnodedata(t);
        write(t,printnodeindention,'temptypedef = ',tempinfo^.typedef.typesymbolprettyname,' = "',
          tempinfo^.typedef.GetTypeName,'", (tempinfo = $',hexstr(ptrint(tempinfo),sizeof(ptrint)*2),' flags = [');
        notfirst:=false;
        for f in tempinfo^.flags do
          begin
            if notfirst then
              write(t,',');
            write(t,f);
            notfirst:=true;
          end;
        writeln(t,'])');
      end;


{*****************************************************************************
                             TEMPDELETENODE
*****************************************************************************}

    constructor ttempdeletenode.create(const temp: ttempcreatenode);
      begin
        inherited create(tempdeleten);
        tempinfo := temp.tempinfo;
        release_to_normal := false;
      end;


    constructor ttempdeletenode.create_normal_temp(const temp: ttempcreatenode);
      begin
        inherited create(tempdeleten);
        tempinfo := temp.tempinfo;
        release_to_normal := true;
        if tempinfo^.temptype <> tt_persistent then
          internalerror(200204211);
      end;


    function ttempdeletenode.dogetcopy: tnode;
      var
        n: ttempdeletenode;
      begin
        n:=ttempdeletenode(inherited dogetcopy);
        n.release_to_normal:=release_to_normal;

        if assigned(tempinfo^.hookoncopy) then
          { if the temp has been copied, assume it becomes a new }
          { temp which has to be hooked by the copied deletenode }
          begin
            { hook the tempdeletenode to the copied temp }
            n.tempinfo:=tempinfo^.hookoncopy;
            { the temp shall not be used, reset hookoncopy    }
            { Only if release_to_normal is false, otherwise   }
            { the temp can still be referenced once more (JM) }
            if (not release_to_normal) then
              tempinfo^.hookoncopy:=nil
            else
              includetempflag(ti_nextref_set_hookoncopy_nil);
          end
        else
          { if the temp we refer to hasn't been copied, we have a }
          { problem since that means we now have two delete nodes }
          { for one temp                                          }
          internalerror(200108234);
        result:=n;
      end;


    constructor ttempdeletenode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        tempidx:=ppufile.getlongint;
        release_to_normal:=(ppufile.getbyte<>0);
      end;


    procedure ttempdeletenode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putlongint(tempinfo^.owner.ppuidx);
        ppufile.putbyte(byte(release_to_normal));
      end;


    procedure ttempdeletenode.resolveppuidx;
      var
        temp : ttempcreatenode;
      begin
        temp:=ttempcreatenode(nodeppuidxget(tempidx));
        if temp.nodetype<>tempcreaten then
          internalerror(200311075);
        tempinfo:=temp.tempinfo;
      end;


    function ttempdeletenode.pass_1 : tnode;
      begin
         expectloc:=LOC_VOID;
         result := nil;
      end;

    function ttempdeletenode.pass_typecheck: tnode;
      begin
        result := nil;
        resultdef := voidtype;
      end;

    function ttempdeletenode.docompare(p: tnode): boolean;
      begin
        result :=
          inherited docompare(p) and
          (ttemprefnode(p).tempinfo = tempinfo);
      end;

    destructor ttempdeletenode.destroy;
      begin
        tempinfo^.withnode.free;
        tempinfo^.tempinitcode.free;
        dispose(tempinfo);
        inherited destroy;
      end;

    procedure ttempdeletenode.printnodedata(var t:text);
      begin
        inherited printnodedata(t);
        writeln(t,printnodeindention,'release_to_normal: ',release_to_normal,', temptypedef = ',tempinfo^.typedef.typesymbolprettyname,' = "',
          tempinfo^.typedef.GetTypeName,'", temptype = ',tempinfo^.temptype,', tempinfo = $',hexstr(ptrint(tempinfo),sizeof(ptrint)*2));
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TTempDeleteNode.XMLPrintNodeData(var T: Text);
      begin
        inherited XMLPrintNodeData(T);
        WriteLn(T, PrintNodeIndention, '<release_to_normal>', release_to_normal, '</release_to_normal>');
      end;
{$endif DEBUG_NODE_XML}

end.
