{
    Copyright (c) 1998-2011 by Florian Klaempfl, Jonas Maebe

    Generates i8086 assembler for typed constant declarations

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
unit n8086tcon;

{$i fpcdefs.inc}

interface

    uses
      node,symdef,ngtcon;


    type

      { ti8086typedconstbuilder }

      ti8086typedconstbuilder = class(tasmlisttypedconstbuilder)
       protected
        procedure tc_emit_orddef(def: torddef; var node: tnode);override;
        procedure tc_emit_pointerdef(def: tpointerdef; var node: tnode);override;
      end;


implementation

uses
  verbose,compinnr,
  ncon,ncnv,ninl,nld,
  defcmp,defutil,
  aasmtai,
  symconst,symtype,symsym,symcpu,
  htypechk;

    { ti8086typedconstbuilder }

    procedure ti8086typedconstbuilder.tc_emit_orddef(def: torddef; var node: tnode);
      var
        hp: tnode;
        srsym: tsym;
        pd: tprocdef;
      begin
        { support word/smallint constants, initialized with Seg() }
        if (def.ordtype in [u16bit,s16bit]) and (node.nodetype=inlinen) and
           (tinlinenode(node).inlinenumber=in_seg_x) then
          begin
            hp:=tunarynode(node).left;
            if (hp.nodetype=typeconvn) and
               (ttypeconvnode(hp).convtype=tc_proc_2_procvar) then
              hp:=tunarynode(hp).left;
            if hp.nodetype=loadn then
              begin
                srsym:=tloadnode(hp).symtableentry;
                case srsym.typ of
                  procsym :
                    begin
                      pd:=tprocdef(tprocsym(srsym).ProcdefList[0]);
                      if Tprocsym(srsym).ProcdefList.Count>1 then
                        Message(parser_e_no_overloaded_procvars);
                      if po_abstractmethod in pd.procoptions then
                        Message(type_e_cant_take_address_of_abstract_method)
                      else
                        ftcb.emit_tai(Tai_const.Create_seg_name(pd.mangledname),u16inttype);
                    end;
                  staticvarsym :
                    ftcb.emit_tai(Tai_const.Create_seg_name(tstaticvarsym(srsym).mangledname),u16inttype);
                  labelsym :
                    ftcb.emit_tai(Tai_const.Create_seg_name(tlabelsym(srsym).mangledname),u16inttype);
                  else
                    Message(type_e_variable_id_expected);
                end;
              end
            else
              Message(parser_e_illegal_expression);
          end
        { support word/smallint constants, initialized with Ofs() or Word(@s) }
        else if (def.ordtype in [u16bit,s16bit]) and (node.nodetype=typeconvn) and
          ((Ttypeconvnode(node).left.nodetype=addrn) or
             is_proc2procvar_load(Ttypeconvnode(node).left,pd)) then
          begin
            hp:=tunarynode(Ttypeconvnode(node).left).left;
            if (hp.nodetype=typeconvn) and
               (ttypeconvnode(hp).convtype=tc_proc_2_procvar) then
              hp:=tunarynode(hp).left;
            if hp.nodetype=loadn then
              begin
                srsym:=tloadnode(hp).symtableentry;
                case srsym.typ of
                  procsym :
                    begin
                      pd:=tprocdef(tprocsym(srsym).ProcdefList[0]);
                      if Tprocsym(srsym).ProcdefList.Count>1 then
                        Message(parser_e_no_overloaded_procvars);
                      if po_abstractmethod in pd.procoptions then
                        Message(type_e_cant_take_address_of_abstract_method)
                      else
                        ftcb.emit_tai(Tai_const.Createname_near(pd.mangledname,0),u16inttype);
                    end;
                  staticvarsym :
                    ftcb.emit_tai(Tai_const.Createname_near(tstaticvarsym(srsym).mangledname,0),u16inttype);
                  labelsym :
                    ftcb.emit_tai(Tai_const.Createname_near(tlabelsym(srsym).mangledname,0),u16inttype);
                  else
                    Message(type_e_variable_id_expected);
                end;
              end
            else
              Message(parser_e_illegal_expression);
          end
        else
          inherited;
      end;


    procedure ti8086typedconstbuilder.tc_emit_pointerdef(def: tpointerdef; var node: tnode);
      var
        hp: tnode;
      begin
        { remove equal typecasts for pointer/nil addresses }
        if (node.nodetype=typeconvn) then
          with Ttypeconvnode(node) do
            if (left.nodetype in [addrn,niln]) and equal_defs(def,node.resultdef) then
              begin
                hp:=left;
                left:=nil;
                node.free;
                node:=hp;
              end;
        { const pointer ? }
        if (node.nodetype = pointerconstn) then
          begin
            ftcb.queue_init(def);
            if is_farpointer(def) or is_hugepointer(def) then
              begin
                ftcb.queue_typeconvn(s32inttype,def);
                ftcb.queue_emit_ordconst(longint(tpointerconstnode(node).value),s32inttype);
              end
            else
              begin
                ftcb.queue_typeconvn(s16inttype,def);
                ftcb.queue_emit_ordconst(smallint(tpointerconstnode(node).value),s16inttype);
              end;
          end
        else if node.nodetype=niln then
          begin
            if is_farpointer(def) or is_hugepointer(def) then
              ftcb.emit_tai(Tai_const.Create_32bit(0),u32inttype)
            else
              ftcb.emit_tai(Tai_const.Create_16bit(0),u16inttype);
          end
        else
          inherited tc_emit_pointerdef(def, node);
      end;

begin
  ctypedconstbuilder:=ti8086typedconstbuilder;
end.

