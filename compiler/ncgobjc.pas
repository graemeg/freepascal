<<<<<<< HEAD
{
    Copyright (c) 2009 by Jonas Maebe

    This unit implements code generator support for Objective-C nodes

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

unit ncgobjc;

{$i fpcdefs.inc}

interface

uses
  nobjc;

type
  tcgobjcselectornode = class(tobjcselectornode)
    procedure pass_generate_code; override;
  end;

  tcgobjcprotocolnode = class(tobjcprotocolnode)
    procedure pass_generate_code; override;
  end;

implementation

uses
    globtype,cclasses,
    aasmbase,aasmdata,aasmtai,
    cgbase,cgutils,defutil,objcgutl,
    symconst,symsym,symdef,
    node,nld,ncon,
    verbose;

{*****************************************************************************
                           TCGOBJCSELECTORNODE
*****************************************************************************}

procedure tcgobjcselectornode.pass_generate_code;
  var
    pool   : THashSet;
    entry  : PHashSetItem;
    name   : pshortstring;
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    if current_asmdata.ConstPools[sp_varnamerefs]=nil then
      current_asmdata.ConstPools[sp_varnamerefs]:=THashSet.Create(64, True, False);
>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[sp_varnamerefs]=nil then
      current_asmdata.ConstPools[sp_varnamerefs]:=THashSet.Create(64, True, False);
>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[sp_varnamerefs]=nil then
      current_asmdata.ConstPools[sp_varnamerefs]:=THashSet.Create(64, True, False);
>>>>>>> graemeg/cpstrnew
=======
    if current_asmdata.ConstPools[sp_varnamerefs]=nil then
      current_asmdata.ConstPools[sp_varnamerefs]:=THashSet.Create(64, True, False);
>>>>>>> origin/cpstrnew
    pool:=current_asmdata.ConstPools[sp_varnamerefs];

    case left.nodetype of
      loadn:
        begin
          if (tloadnode(left).symtableentry.typ<>procsym) then
            internalerror(2009051602);
          if (tprocsym(tloadnode(left).symtableentry).procdeflist.count<>1) then
            internalerror(2009051701);
          name:=tprocdef(tprocsym(tloadnode(left).symtableentry).procdeflist[0]).messageinf.str;
          entry:=pool.FindOrAdd(@name^[1],length(name^))
        end;
      stringconstn:
        begin
          entry:=pool.FindOrAdd(tstringconstnode(left).value_str,tstringconstnode(left).len);
        end;
      else
        internalerror(2009030701);
    end;

    objcfinishstringrefpoolentry(entry,sp_objcvarnames,sec_objc_message_refs,sec_objc_meth_var_names);

    location_reset_ref(location,LOC_CREFERENCE,def_cgsize(resultdef),sizeof(pint));
    location.reference.symbol:=tasmlabel(entry^.Data);
  end;


{*****************************************************************************
                           TCGOBJCPROTOCOLNODE
*****************************************************************************}

procedure tcgobjcprotocolnode.pass_generate_code;
  begin
    { first needs support for writing class definitions }
    internalerror(2009072601);
  end;


begin
  cobjcselectornode:=tcgobjcselectornode;
  cobjcprotocolnode:=tcgobjcprotocolnode;
end.
=======
>>>>>>> origin/fixes_2.4
