{
    Tail recursion optimization

    Copyright (c) 2006 by Florian Klaempfl

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
unit opttail;

{$i fpcdefs.inc}

  interface

    uses
      symdef,node;

    procedure do_opttail(var n : tnode;p : tprocdef);

  implementation

    uses
      globtype,
      symconst,symsym,
      defcmp,defutil,
      nutils,nbas,nflw,ncal,nld,ncnv,nmem,
      pass_1,
      paramgr;

    procedure do_opttail(var n : tnode;p : tprocdef);

      var
        labelnode : tlabelnode;

      function find_and_replace_tailcalls(var n : tnode) : boolean;

        var
          usedcallnode : tcallnode;

        function is_recursivecall(n : tnode) : boolean;
          begin
            result:=(n.nodetype=calln) and (tcallnode(n).procdefinition=p) and not(assigned(tcallnode(n).methodpointer));
            if result then
              usedcallnode:=tcallnode(n)
            else
              { obsolete type cast? }
              result:=((n.nodetype=typeconvn) and (ttypeconvnode(n).convtype=tc_equal) and is_recursivecall(ttypeconvnode(n).left));
          end;

        function is_resultassignment(n : tnode) : boolean;
          begin
            result:=((n.nodetype=loadn) and (tloadnode(n).symtableentry=p.funcretsym)) or
              ((n.nodetype=typeconvn) and (ttypeconvnode(n).convtype=tc_equal) and is_resultassignment(ttypeconvnode(n).left));
          end;

        var
          calcnodes,
          copynodes,
          hp : tnode;
          nodes,
          calcstatements,
          copystatements : tstatementnode;
          paranode : tcallparanode;
          tempnode : ttempcreatenode;
          loadnode : tloadnode;
          oldnodetree : tnode;
          useaddr : boolean;
        begin
          { no tail call found and replaced so far }
          result:=false;
          if n=nil then
            exit;
          usedcallnode:=nil;
          case n.nodetype of
            statementn:
              begin
                hp:=n;
                { search last node }
                while assigned(tstatementnode(hp).right) do
                  hp:=tstatementnode(hp).right;
                result:=find_and_replace_tailcalls(tstatementnode(hp).left);
              end;
            ifn:
              begin
                result:=find_and_replace_tailcalls(tifnode(n).right);
                { avoid short bool eval here }
                result:=find_and_replace_tailcalls(tifnode(n).t1) or result;
              end;
            calln,
            assignn:
              begin
                if ((n.nodetype=calln) and is_recursivecall(n)) or
                   ((n.nodetype=assignn) and is_resultassignment(tbinarynode(n).left) and
                   is_recursivecall(tbinarynode(n).right)) then
                  begin
                    { found one! }
                    {
                    writeln('tail recursion optimization for ',p.mangledname);
                    printnode(output,n);
                    }
                    { create assignments for all parameters }

                    { this is hairy to do because one parameter could be used to calculate another one, so
                      assign them first to temps and then add them }

                    calcnodes:=internalstatements(calcstatements);
                    copynodes:=internalstatements(copystatements);
                    paranode:=tcallparanode(usedcallnode.left);
                    while assigned(paranode) do
                      begin
                        useaddr:=(paranode.parasym.varspez in [vs_var,vs_constref]) or
                          ((paranode.parasym.varspez=vs_const) and
                          paramanager.push_addr_param(paranode.parasym.varspez,paranode.parasym.vardef,p.proccalloption)) or
                          ((paranode.parasym.varspez=vs_value) and
                          is_open_array(paranode.parasym.vardef));
                        if useaddr then
                          begin
                            tempnode:=ctempcreatenode.create(voidpointertype,voidpointertype.size,tt_persistent,true);
                            addstatement(calcstatements,tempnode);
                            addstatement(calcstatements,
                              cassignmentnode.create(
                                ctemprefnode.create(tempnode),
                                caddrnode.create_internal(paranode.left)
                                ));
                          end
                        else
                          begin
                            tempnode:=ctempcreatenode.create(paranode.left.resultdef,paranode.left.resultdef.size,tt_persistent,true);
                            addstatement(calcstatements,tempnode);
                            addstatement(calcstatements,
                              cassignmentnode.create_internal(
                                ctemprefnode.create(tempnode),
                                paranode.left
                                ));
                          end;

                        { "cast" away const varspezs }
                        loadnode:=cloadnode.create(paranode.parasym,paranode.parasym.owner);
                        include(tloadnode(loadnode).loadnodeflags,loadnf_isinternal_ignoreconst);

                        { load the address of the symbol instead of symbol }
                        if useaddr then
                          include(tloadnode(loadnode).loadnodeflags,loadnf_load_addr);
                        addstatement(copystatements,
                          cassignmentnode.create_internal(
                            loadnode,
                            ctemprefnode.create(tempnode)
                            ));
                        addstatement(copystatements,ctempdeletenode.create_normal_temp(tempnode));

                        { reused }
                        paranode.left:=nil;
                        paranode:=tcallparanode(paranode.right);
                      end;

                    oldnodetree:=n;
                    n:=internalstatements(nodes);

                    if assigned(usedcallnode.callinitblock) then
                      begin
                        addstatement(nodes,usedcallnode.callinitblock);
                        usedcallnode.callinitblock:=nil;
                      end;

                    addstatement(nodes,calcnodes);
                    addstatement(nodes,copynodes);

                    { create goto }
                    addstatement(nodes,cgotonode.create(labelnode.labsym));

                    if assigned(usedcallnode.callcleanupblock) then
                      begin
                        { callcleanupblock should contain only temp. node clean up }
                        checktreenodetypes(usedcallnode.callcleanupblock,
                          [tempdeleten,blockn,statementn,temprefn,nothingn]);
                        addstatement(nodes,usedcallnode.callcleanupblock);
                        usedcallnode.callcleanupblock:=nil;
                      end;

                    oldnodetree.free;

                    do_firstpass(n);
                    result:=true;
                  end;
              end;
            blockn:
              result:=find_and_replace_tailcalls(tblocknode(n).left);
            else
              ;
          end;
        end;

      var
        s : tstatementnode;
        oldnodes : tnode;
        i : longint;
        labelsym : tlabelsym;
      begin
        { check if the parameters actually would support tail recursion elimination }
        for i:=0 to p.paras.count-1 do
          with tparavarsym(p.paras[i]) do
            if (varspez=vs_out) or
               { parameters requiring tables are too complicated to handle
                 and slow down things anyways so a tail recursion call
                 makes no sense
               }
               is_managed_type(vardef) then
               exit;

        labelsym:=clabelsym.create('$opttail');
        labelnode:=clabelnode.create(cnothingnode.create,labelsym);
        if find_and_replace_tailcalls(n) then
          begin
            oldnodes:=n;
            n:=internalstatements(s);
            addstatement(s,labelnode);
            addstatement(s,oldnodes);
          end
        else
          labelnode.free;
      end;

end.

