{
    Loop unrolling

    Copyright (c) 2005 by Florian Klaempfl

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
unit optunrol;

{$i fpcdefs.inc}

  interface

    uses
      node;

    function unroll_loop(node : tnode) : tnode;

  implementation

    uses
      globtype,globals,
      cpuinfo,
      nutils,
      nbas,nflw,ncon,ninl,ncal,nld;

    var
      nodecount : aint;

    function donodecount(var n: tnode; arg: pointer): foreachnoderesult;
      begin
        inc(nodecount);
        result:=fen_false;
      end;


    { rough estimation how large the tree "node" is }
    function countnodes(node : tnode) : aint;
      begin
        nodecount:=0;
        foreachnodestatic(node,@donodecount,nil);
        result:=nodecount;
      end;


    function number_unrolls(node : tnode) : integer;
      begin
{$ifdef i386}
        { multiply by 2 for CPUs with a long pipeline }
        if current_settings.cputype in [cpu_Pentium4] then
          number_unrolls:=60 div countnodes(node)
        else
{$endif i386}
          number_unrolls:=30 div countnodes(node);

        if number_unrolls=0 then
          number_unrolls:=1;
      end;


    function unroll_loop(node : tnode) : tnode;
      var
        unrolls,i : integer;
        counts : qword;
        unrollstatement,newforstatement : tstatementnode;
        unrollblock : tblocknode;
      begin
        result:=nil;
        if (cs_opt_size in current_settings.optimizerswitches) then
          exit;
        if not(node.nodetype in [forn]) then
          exit;
        unrolls:=number_unrolls(tfornode(node).t2);
        if unrolls>1 then
          begin
            { number of executions known? }
            if (tfornode(node).right.nodetype=ordconstn) and (tfornode(node).t1.nodetype=ordconstn) then
              begin
                if lnf_backward in tfornode(node).loopflags then
                  counts:=tordconstnode(tfornode(node).right).value-tordconstnode(tfornode(node).t1).value+1
                else
                  counts:=tordconstnode(tfornode(node).t1).value-tordconstnode(tfornode(node).right).value+1;

                { don't unroll more than we need }
                if unrolls>counts then
                  unrolls:=counts;

                { create block statement }
                unrollblock:=internalstatements(unrollstatement);

                { let's unroll (and rock of course) }
                for i:=1 to unrolls do
                  begin
                    { create and insert copy of the statement block }
                    addstatement(unrollstatement,tfornode(node).t2.getcopy);

                    { set and insert entry label? }
                    if (counts mod unrolls<>0) and
                      ((counts mod unrolls)=unrolls-i) then
                      begin
                        tfornode(node).entrylabel:=clabelnode.create(cnothingnode.create,nil);
                        addstatement(unrollstatement,tfornode(node).entrylabel);
                      end;

                    { for itself increases at the last iteration }
                    if i<unrolls then
                      begin
                        { insert incrementation of counter var }
                        addstatement(unrollstatement,
                          geninlinenode(in_inc_x,false,ccallparanode.create(tfornode(node).left.getcopy,nil)));
                      end;
                  end;
                { can we get rid of the for statement? }
                if unrolls=counts then
                  begin
                    { create block statement }
                    result:=internalstatements(newforstatement);
                    { initial assignment }
                    addstatement(newforstatement,cassignmentnode.create(
                      tfornode(node).left.getcopy,tfornode(node).right.getcopy));
                    addstatement(newforstatement,unrollblock);
                  end;
              end
            else
              begin
                { unrolling is a little bit more tricky if we don't know the
                  loop count at compile time, but the solution is to use a jump table
                  which is indexed by "loop count mod unrolls" at run time and which
                  jumps then at the appropriate place inside the loop. Because
                  a module division is expensive, we can use only unroll counts dividable
                  by 2 }
                case unrolls of
                  1..2:
                    ;
                  3:
                    unrolls:=2;
                  4..7:
                    unrolls:=4;
                  { unrolls>4 already make no sense imo, but who knows (FK) }
                  8..15:
                    unrolls:=8;
                  16..31:
                    unrolls:=16;
                  32..63:
                    unrolls:=32;
                  64..$7fff:
                    unrolls:=64;
                  else
                    exit;
                end;
                { we don't handle this yet }
                exit;
              end;
            if not(assigned(result)) then
              begin
                tfornode(node).t2.free;
                tfornode(node).t2:=unrollblock;
              end;
          end;
      end;

end.
