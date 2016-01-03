<<<<<<< HEAD
{
    Copyright (c) 2009 by Jonas Maebe

    This unit implements Objective-C nodes

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
{ @abstract(This unit implements Objective-C nodes)
  This unit contains various nodes to implement Objective-Pascal and to
  interface with the Objective-C runtime.
}

unit nobjc;

{$i fpcdefs.inc}

interface

uses
  node;

type
  tobjcselectornode = class(tunarynode)
   public
    constructor create(formethod: tnode);
    function pass_typecheck: tnode;override;
    function pass_1: tnode;override;
  end;
  tobjcselectornodeclass = class of tobjcselectornode;

  tobjcprotocolnode = class(tunarynode)
   public
    constructor create(forprotocol: tnode);
    function pass_typecheck: tnode;override;
    function pass_1: tnode;override;
  end;
  tobjcprotocolnodeclass = class of tobjcprotocolnode;

var
  cobjcselectornode : tobjcselectornodeclass;
  cobjcprotocolnode : tobjcprotocolnodeclass;

implementation

uses
  sysutils,
  globtype,globals,cclasses,systems,
  verbose,pass_1,
  defutil,
  symtype,symtable,symdef,symconst,symsym,
  paramgr,
  nutils,
  nbas,nld,ncnv,ncon,ncal,nmem,
  objcutil,
  cgbase;


{*****************************************************************************
                            TOBJCSELECTORNODE
*****************************************************************************}

constructor tobjcselectornode.create(formethod: tnode);
  begin
    inherited create(objcselectorn,formethod);
  end;


function tobjcselectornode.pass_typecheck: tnode;
  var
    len: longint;
    s: shortstring;
  begin
    if not(m_objectivec1 in current_settings.modeswitches) then
      Message(parser_f_modeswitch_objc_required);
    result:=nil;
    typecheckpass(left);
    { argument can be
       a) an objc method
       b) a pchar, zero-based chararray or ansistring
    }
    case left.nodetype of
      loadn:
        begin
          if (left.resultdef.typ=procdef) and
             (po_objc in tprocdef(left.resultdef).procoptions) then
            begin
              { ok }
            end
          else
            CGMessage1(type_e_expected_objc_method_but_got,left.resultdef.typename);
        end;
      stringconstn:
        begin
          if not objcvalidselectorname(tstringconstnode(left).value_str,
                                       tstringconstnode(left).len) then
            begin
              len:=tstringconstnode(left).len;
              if (len>255) then
                len:=255;
              setlength(s,len);
              move(tstringconstnode(left).value_str^,s[1],len);
              CGMessage1(type_e_invalid_objc_selector_name,s);
              exit;
            end;
        end
      else
        CGMessage(type_e_expected_objc_method);
    end;
    resultdef:=objc_seltype;
  end;


function tobjcselectornode.pass_1: tnode;
  begin
    result:=nil;
    expectloc:=LOC_CREFERENCE;
  end;


{*****************************************************************************
                            TOBJPROTOCOLNODE
*****************************************************************************}

constructor tobjcprotocolnode.create(forprotocol: tnode);
  begin
    inherited create(objcprotocoln,forprotocol);
  end;


function tobjcprotocolnode.pass_typecheck: tnode;
  begin
    if not(m_objectivec1 in current_settings.modeswitches) then
      Message(parser_f_modeswitch_objc_required);
    result:=nil;
    typecheckpass(left);
    if (left.nodetype<>typen) then
      MessagePos(left.fileinfo,type_e_type_id_expected)
    else if not is_objcprotocol(left.resultdef) then
      MessagePos2(left.fileinfo,type_e_incompatible_types,left.resultdef.typename,'ObjCProtocol');
    resultdef:=objc_protocoltype;
  end;


function tobjcprotocolnode.pass_1: tnode;
  begin
    result:=ccallnode.createinternresfromunit('OBJC','OBJC_GETPROTOCOL',
      ccallparanode.create(cstringconstnode.createstr(tobjectdef(left.resultdef).objextname^),nil),
      resultdef
    );
    typecheckpass(result);
  end;


end.

=======
>>>>>>> origin/fixes_2.4
