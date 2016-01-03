{
    Copyright (c) 1998-2002 by Jonas Maebe

    This unit implements optimized nodes

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
unit nopt;

{$i fpcdefs.inc}

interface

uses node,nbas,nadd,constexp;

type
  tsubnodetype = (
    addsstringcharoptn,  { shorstring + char }
    addsstringcsstringoptn   { shortstring + constant shortstring }
  );

  taddoptnode = class(taddnode)
     subnodetype: tsubnodetype;
     constructor create(ts: tsubnodetype; l,r : tnode); virtual; reintroduce;
     { pass_1 will be overridden by the separate subclasses    }
     { By default, pass_generate_code is the same as for addnode           }
     { Only if there's a processor specific implementation, it }
     { will be overridden.                                     }
     function dogetcopy: tnode; override;
     function docompare(p: tnode): boolean; override;
  end;

  taddsstringoptnode = class(taddoptnode)
    { maximum length of the string until now, allows us to skip a compare }
    { sometimes (it's initialized/updated by calling updatecurmaxlen)     }
    curmaxlen: byte;
    { pass_1 must be overridden, otherwise we get an endless loop }
    function pass_typecheck: tnode; override;
    function pass_1: tnode; override;
    function dogetcopy: tnode; override;
    function docompare(p: tnode): boolean; override;
   protected
    procedure updatecurmaxlen;
  end;

  { add a char to a shortstring }
  taddsstringcharoptnode = class(taddsstringoptnode)
    constructor create(l,r : tnode); virtual; reintroduce;
  end;
  taddsstringcharoptnodeclass = class of taddsstringcharoptnode;

  { add a constant string to a short string }
  taddsstringcsstringoptnode = class(taddsstringoptnode)
    constructor create(l,r : tnode); virtual; reintroduce;
    function pass_1: tnode; override;
  end;
  taddsstringcsstringoptnodeclass = class of taddsstringcsstringoptnode;

function canbeaddsstringcharoptnode(p: taddnode): boolean;
function genaddsstringcharoptnode(p: taddnode): tnode;
function canbeaddsstringcsstringoptnode(p: taddnode): boolean;
function genaddsstringcsstringoptnode(p: taddnode): tnode;
function canbemultistringadd(p: taddnode): boolean;
function genmultistringadd(p: taddnode): tnode;


function is_addsstringoptnode(p: tnode): boolean;

var
   caddsstringcharoptnode: taddsstringcharoptnodeclass;
   caddsstringcsstringoptnode: taddsstringcsstringoptnodeclass;

implementation

uses cutils, systems,
     htypechk, defutil, defcmp, globtype, globals, cpubase,
     ncnv, ncon, ncal, ninl, nld, nmem,
     verbose, symconst,symdef, cgbase, procinfo;


{*****************************************************************************
                             TADDOPTNODE
*****************************************************************************}

constructor taddoptnode.create(ts: tsubnodetype; l,r : tnode);
begin
  { we need to keep the addn nodetype, otherwise taddnode.pass_generate_code will be }
  { confused. Comparison for equal nodetypes therefore has to be         }
  { implemented using the classtype() method (JM)                        }
  inherited create(addn,l,r);
  subnodetype := ts;
end;

function taddoptnode.dogetcopy: tnode;
var
  hp: taddoptnode;
begin
  hp := taddoptnode(inherited dogetcopy);
  hp.subnodetype := subnodetype;
  dogetcopy := hp;
end;

function taddoptnode.docompare(p: tnode): boolean;
begin
  docompare :=
    inherited docompare(p) and
    (subnodetype = taddoptnode(p).subnodetype);
end;


{*****************************************************************************
                        TADDSSTRINGOPTNODE
*****************************************************************************}

function taddsstringoptnode.pass_typecheck: tnode;
begin
  result := nil;
  updatecurmaxlen;
  { left and right are already firstpass'ed by taddnode.pass_1 }
  if not is_shortstring(left.resultdef) then
   inserttypeconv(left,cshortstringtype);
  if not is_shortstring(right.resultdef) then
   inserttypeconv(right,cshortstringtype);
  resultdef := left.resultdef;
end;

function taddsstringoptnode.pass_1: tnode;
begin
  pass_1 := nil;
  expectloc:= LOC_REFERENCE;
  { here we call STRCONCAT or STRCMP or STRCOPY }
  include(current_procinfo.flags,pi_do_call);
end;

function taddsstringoptnode.dogetcopy: tnode;
var
  hp: taddsstringoptnode;
begin
  hp := taddsstringoptnode(inherited dogetcopy);
  hp.curmaxlen := curmaxlen;
  dogetcopy := hp;
end;

function taddsstringoptnode.docompare(p: tnode): boolean;
begin
  docompare :=
    inherited docompare(p) and
    (curmaxlen = taddsstringcharoptnode(p).curmaxlen);
end;


function is_addsstringoptnode(p: tnode): boolean;
begin
  is_addsstringoptnode :=
    p.inheritsfrom(taddsstringoptnode);
end;

procedure taddsstringoptnode.updatecurmaxlen;
begin
  if is_addsstringoptnode(left) then
    begin
      { made it a separate block so no other if's are processed (would be a }
      { simple waste of time) (JM)                                          }
      if (taddsstringoptnode(left).curmaxlen < 255) then
        case subnodetype of
          addsstringcharoptn:
            curmaxlen := succ(taddsstringoptnode(left).curmaxlen);
          addsstringcsstringoptn:
            curmaxlen := min(taddsstringoptnode(left).curmaxlen +
                              tstringconstnode(right).len,255)
          else
            internalerror(291220001);
        end
      else curmaxlen := 255;
    end
  else if (left.nodetype = stringconstn) then
    curmaxlen := min(tstringconstnode(left).len,255)
  else if is_char(left.resultdef) then
    curmaxlen := 1
  else if (left.nodetype = typeconvn) then
    begin
      case ttypeconvnode(left).convtype of
        tc_char_2_string:
          curmaxlen := 1;
{       doesn't work yet, don't know why (JM)
        tc_chararray_2_string:
          curmaxlen :=
            min(ttypeconvnode(left).left.resultdef.size,255); }
        else curmaxlen := 255;
      end;
    end
  else
    curmaxlen := 255;
end;

{*****************************************************************************
                        TADDSSTRINGCHAROPTNODE
*****************************************************************************}


constructor taddsstringcharoptnode.create(l,r : tnode);
begin
  inherited create(addsstringcharoptn,l,r);
end;

{*****************************************************************************
                        TADDSSTRINGCSSTRINGOPTNODE
*****************************************************************************}


constructor taddsstringcsstringoptnode.create(l,r : tnode);
begin
  inherited create(addsstringcsstringoptn,l,r);
end;


function taddsstringcsstringoptnode.pass_1: tnode;
begin
  { create the call to the concat routine both strings as arguments }
  result := ccallnode.createintern('fpc_shortstr_append_shortstr',
    ccallparanode.create(left,ccallparanode.create(right,nil)));
  left:=nil;
  right:=nil;
end;


{*****************************************************************************
                                HELPERS
*****************************************************************************}

function canbeaddsstringcharoptnode(p: taddnode): boolean;
begin
  canbeaddsstringcharoptnode :=
    (cs_opt_level1 in current_settings.optimizerswitches) and

{   the shortstring will be gotten through conversion if necessary (JM)
    is_shortstring(p.left.resultdef) and }
    ((p.nodetype = addn) and
     is_char(p.right.resultdef));
end;

function genaddsstringcharoptnode(p: taddnode): tnode;
var
  hp: tnode;
begin
  hp := caddsstringcharoptnode.create(p.left.getcopy,p.right.getcopy);
  hp.flags := p.flags;
  genaddsstringcharoptnode := hp;
end;



function canbeaddsstringcsstringoptnode(p: taddnode): boolean;
begin
  canbeaddsstringcsstringoptnode :=
    (cs_opt_level1 in current_settings.optimizerswitches) and

{   the shortstring will be gotten through conversion if necessary (JM)
    is_shortstring(p.left.resultdef) and }
    ((p.nodetype = addn) and
     (p.right.nodetype = stringconstn));
end;

function genaddsstringcsstringoptnode(p: taddnode): tnode;
var
  hp: tnode;
begin
  hp := caddsstringcsstringoptnode.create(p.left.getcopy,p.right.getcopy);
  hp.flags := p.flags;
  genaddsstringcsstringoptnode := hp;
end;


function canbemultistringadd(p: taddnode): boolean;
var
  hp : tnode;
  i  : longint;
begin
  result:=false;
  if p.resultdef.typ<>stringdef then
    exit;
  i:=0;
  hp:=p;
  while assigned(hp) and (hp.nodetype=addn) do
    begin
      inc(i);
      hp:=taddnode(hp).left;
    end;
  result:=(i>1);
end;


function genmultistringadd(p: taddnode): tnode;
var
  hp,sn : tnode;
  arrp  : tarrayconstructornode;
  newstatement : tstatementnode;
  tempnode    : ttempcreatenode;
  is_shortstr : boolean;
  para : tcallparanode;
begin
  arrp:=nil;
  hp:=p;
  is_shortstr:=is_shortstring(p.resultdef);
  while assigned(hp) and (hp.nodetype=addn) do
    begin
      sn:=taddnode(hp).right.getcopy;
      inserttypeconv(sn,p.resultdef);
      if is_shortstr then
        begin
          sn:=caddrnode.create(sn);
          include(sn.flags,nf_typedaddr);
          include(sn.flags,nf_internal);
        end;
      arrp:=carrayconstructornode.create(sn,arrp);
      hp:=taddnode(hp).left;
    end;
  sn:=hp.getcopy;
  inserttypeconv(sn,p.resultdef);
  if is_shortstr then
    begin
      sn:=caddrnode.create(sn);
      include(sn.flags,nf_internal);
    end;
  arrp:=carrayconstructornode.create(sn,arrp);
  if assigned(aktassignmentnode) and
     (aktassignmentnode.right=p) and
     (aktassignmentnode.left.resultdef=p.resultdef) and
     valid_for_var(aktassignmentnode.left,false) then
    begin
      para:=ccallparanode.create(
              arrp,
              ccallparanode.create(aktassignmentnode.left.getcopy,nil)
            );
      if is_ansistring(p.resultdef) then
        para:=ccallparanode.create(
                cordconstnode.create(
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  { don't use getparaencoding(), we have to know
                    when the result is rawbytestring }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  tstringdef(p.resultdef).encoding,
                  u16inttype,
                  true
                ),
                para
              );
      result:=ccallnode.createintern(
                'fpc_'+tstringdef(p.resultdef).stringtypname+'_concat_multi',
                para
              );
      include(aktassignmentnode.flags,nf_assign_done_in_right);
    end
  else
    begin
      result:=internalstatements(newstatement);
      tempnode:=ctempcreatenode.create(p.resultdef,p.resultdef.size,tt_persistent ,true);
      addstatement(newstatement,tempnode);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      { initialize the temp, since it will be passed to a
        var-parameter (and finalization, which is performed by the
        ttempcreate node and which takes care of the initialization
        on native targets, is a noop on managed VM targets) }
      if (target_info.system in systems_managed_vm) and
         is_managed_type(p.resultdef) then
        addstatement(newstatement,cinlinenode.create(in_setlength_x,
          false,
          ccallparanode.create(genintconstnode(0),
            ccallparanode.create(ctemprefnode.create(tempnode),nil))));
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      para:=ccallparanode.create(
              arrp,
              ccallparanode.create(ctemprefnode.create(tempnode),nil)
            );
      if is_ansistring(p.resultdef) then
        para:=ccallparanode.create(
                cordconstnode.create(
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  { don't use getparaencoding(), we have to know
                    when the result is rawbytestring }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                  tstringdef(p.resultdef).encoding,
                  u16inttype,
                  true
                ),
                para
              );
      addstatement(
        newstatement,
        ccallnode.createintern(
          'fpc_'+tstringdef(p.resultdef).stringtypname+'_concat_multi',
          para
        )
      );
      addstatement(newstatement,ctempdeletenode.create_normal_temp(tempnode));
      addstatement(newstatement,ctemprefnode.create(tempnode));
    end;
end;

begin
  caddsstringcharoptnode := taddsstringcharoptnode;
  caddsstringcsstringoptnode := taddsstringcsstringoptnode;
end.
