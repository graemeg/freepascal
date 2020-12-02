{
    Copyright (c) 1998-2012 by Florian Klaempfl and others

    Generate i8086 assembler for constants

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
unit n8086con;

{$i fpcdefs.inc}

interface

    uses
       globtype,symtype,ncon,ncgcon,nx86con;

    type

      { ti8086pointerconstnode }

      ti8086pointerconstnode = class(tcgpointerconstnode)
        constructor create(v : TConstPtrUInt;def:tdef);override;
        procedure printnodedata(var t: text);override;
{$ifdef DEBUG_NODE_XML}
        procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
        procedure pass_generate_code;override;
      end;

implementation

    uses
      systems,globals,verbose,
      symconst,symdef,symcpu,
      defutil,
      cpubase,
      cga,cgx86,cgobj,cgbase,cgutils,
      node;

    {*****************************************************************************
                               T8086POINTERCONSTNODE
    *****************************************************************************}


    constructor ti8086pointerconstnode.create(v: TConstPtrUInt; def: tdef);
      begin
        { truncate near pointers }
        if (def.typ<>pointerdef) or not (tcpupointerdef(def).x86pointertyp in [x86pt_far,x86pt_huge]) then
          v := Word(v);
        inherited create(v, def);
      end;


    procedure ti8086pointerconstnode.printnodedata(var t: text);
      begin
        if (typedef.typ=pointerdef) and (tcpupointerdef(typedef).x86pointertyp in [x86pt_far,x86pt_huge]) then
          writeln(t,printnodeindention,'value = $',hexstr(word(value shr 16),4),':',hexstr(word(value),4))
        else
          inherited printnodedata(t);
      end;

{$ifdef DEBUG_NODE_XML}
    procedure Ti8086PointerConstNode.XMLPrintNodeData(var T: Text);
      begin
        if (typedef.typ=pointerdef) and (tcpupointerdef(typedef).x86pointertyp in [x86pt_far,x86pt_huge]) then
          WriteLn(T, PrintNodeIndention, '<value>$', hexstr(word(value shr 16),4),':',hexstr(word(value),4), '</value>')
        else
          inherited XMLPrintNodeData(T);
      end;
{$endif DEBUG_NODE_XML}

    procedure ti8086pointerconstnode.pass_generate_code;
      begin
        { far pointer? }
        if (typedef.typ=pointerdef) and (tcpupointerdef(typedef).x86pointertyp in [x86pt_far,x86pt_huge]) then
          begin
            location_reset(location,LOC_CONSTANT,OS_32);
            location.value:=longint(value);
          end
        else
          begin
            { an integer const. behaves as a memory reference }
            location_reset(location,LOC_CONSTANT,OS_ADDR);
            location.value:=aint(value);
          end;
      end;


begin
  cpointerconstnode:=ti8086pointerconstnode;
end.
