{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Type checking and simplification for add nodes

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
unit nadd;

{$i fpcdefs.inc}
{$modeswitch nestedprocvars}

{ define addstringopt}

interface

    uses
      node,symtype;

    type
       taddnode = class(tbinopnode)
       private
          resultrealdefderef: tderef;
          function pass_typecheck_internal:tnode;
       public
          resultrealdef : tdef;
          constructor create(tt : tnodetype;l,r : tnode);override;
          constructor create_internal(tt:tnodetype;l,r:tnode);
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function simplify(forinline: boolean) : tnode;override;
          function dogetcopy : tnode;override;
          function docompare(p: tnode): boolean; override;
    {$ifdef state_tracking}
          function track_state_pass(exec_known:boolean):boolean;override;
    {$endif}
         protected
          { override the following if you want to implement }
          { parts explicitely in the code generator (JM)    }
          function first_addstring: tnode; virtual;
          function first_addset: tnode; virtual;
          function first_adddynarray : tnode; virtual;
          { only implements "muln" nodes, the rest always has to be done in }
          { the code generator for performance reasons (JM)                 }
          function first_add64bitint: tnode; virtual;
          function first_addpointer: tnode; virtual;
          function first_cmppointer: tnode; virtual;

          { override and return false if you can handle 32x32->64 }
          { bit multiplies directly in your code generator. If    }
          { this function is overridden to return false, you can  }
          { get multiplies with left/right both s32bit or u32bit, }
          { and resultdef of the muln s64bit or u64bit           }
          function use_generic_mul32to64: boolean; virtual;

          { override and return false if code generator can handle }
          { full 64 bit multiplies.                                }
          function use_generic_mul64bit: boolean; virtual;

{$ifdef cpuneedsmulhelper}
          { override to customize to decide if the code generator }
          { can handle a given multiply node directly, or it needs helpers }
          function use_mul_helper: boolean; virtual;
{$endif cpuneedsmulhelper}

          { shall be overriden if the target cpu supports
            an fma instruction
          }
          function use_fma : boolean; virtual;
          { This routine calls internal runtime library helpers
            for all floating point arithmetic in the case
            where the emulation switches is on. Otherwise
            returns nil, and everything must be done in
            the code generation phase.
          }
          function first_addfloat : tnode; virtual;
       private
          { checks whether a muln can be calculated as a 32bit }
          { * 32bit -> 64 bit                                  }
          function try_make_mul32to64: boolean;

          { Match against the ranges, i.e.:
            var a:1..10;
            begin
              if a>0 then
                ...
            always evaluates to true. (DM)
          }
          function cmp_of_disjunct_ranges(var res : boolean) : boolean;

          { tries to replace the current node by a fma node }
          function try_fma(ld,rd : tdef) : tnode;
       end;
       taddnodeclass = class of taddnode;

    var
       { caddnode is used to create nodes of the add type }
       { the virtual constructor allows to assign         }
       { another class type to caddnode => processor      }
       { specific node types can be created               }
       caddnode : taddnodeclass = taddnode;

implementation

    uses
{$IFNDEF USE_FAKE_SYSUTILS}
      sysutils,
{$ELSE}
      fksysutl,
{$ENDIF}
      globtype,systems,constexp,compinnr,
      cutils,verbose,globals,widestr,
      tokens,
      symconst,symdef,symsym,symcpu,symtable,defutil,defcmp,
      cgbase,
      htypechk,pass_1,
      nld,nbas,nmat,ncnv,ncon,nset,nopt,ncal,ninl,nmem,nutils,
      {$ifdef state_tracking}
      nstate,
      {$endif}
      cpuinfo;


{*****************************************************************************
                                TADDNODE
*****************************************************************************}

{$maxfpuregisters 0}

    function getbestreal(t1,t2 : tdef) : tdef;
      const
        floatweight : array[tfloattype] of byte =
          (2,3,4,5,0,1,6);
      begin
        if t1.typ=floatdef then
          begin
            result:=t1;
            if t2.typ=floatdef then
              begin
                { when a comp or currency is used, use always the
                  best float type to calculate the result }
                if (tfloatdef(t1).floattype in [s64comp,s64currency]) or
                  (tfloatdef(t2).floattype in [s64comp,s64currency]) or
                  (cs_excessprecision in current_settings.localswitches) then
                  result:=pbestrealtype^
                else
                  if floatweight[tfloatdef(t2).floattype]>floatweight[tfloatdef(t1).floattype] then
                    result:=t2;
              end;
          end
        else if t2.typ=floatdef then
          result:=t2
        else internalerror(200508061);
      end;


    constructor taddnode.create(tt : tnodetype;l,r : tnode);
      begin
         inherited create(tt,l,r);
      end;


    constructor taddnode.create_internal(tt:tnodetype;l,r:tnode);
      begin
        create(tt,l,r);
        include(flags,nf_internal);
      end;


    constructor taddnode.ppuload(t: tnodetype; ppufile: tcompilerppufile);
      begin
        inherited ppuload(t, ppufile);
        ppufile.getderef(resultrealdefderef);
      end;


    procedure taddnode.ppuwrite(ppufile: tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
         ppufile.putderef(resultrealdefderef);
      end;


    procedure taddnode.buildderefimpl;
      begin
        inherited buildderefimpl;
        resultrealdefderef.build(resultrealdef);
      end;


    procedure taddnode.derefimpl;
      begin
        inherited derefimpl;
        resultrealdef:=tdef(resultrealdefderef.resolve);
      end;


    function taddnode.cmp_of_disjunct_ranges(var res : boolean) : boolean;
      var
        hp          : tnode;
        realdef     : tdef;
        v           : tconstexprint;
      begin
        result:=false;
        { check for comparision with known result because the ranges of the operands don't overlap }
        if (is_constintnode(right) and (left.resultdef.typ=orddef) and
            { don't ignore type checks }
            is_subequal(right.resultdef,left.resultdef)) or
           (is_constintnode(left) and (right.resultdef.typ=orddef) and
            { don't ignore type checks }
            is_subequal(left.resultdef,right.resultdef)) then
           begin
             if is_constintnode(right) then
               begin
                 hp:=left;
                 v:=Tordconstnode(right).value;
               end
             else
               begin
                 hp:=right;
                 v:=Tordconstnode(left).value;
               end;

             realdef:=hp.resultdef;
             { stop with finding the real def when we either encounter
                a) an explicit type conversion (then the value has to be
                   re-interpreted)
                b) an "absolute" type conversion (also requires
                   re-interpretation)
             }
             while (hp.nodetype=typeconvn) and
                   ([nf_internal,nf_explicit,nf_absolute] * hp.flags = []) do
               begin
                 hp:=ttypeconvnode(hp).left;
                 realdef:=hp.resultdef;
               end;
             if is_constintnode(left) then
               with torddef(realdef) do
                 case nodetype of
                  ltn:
                    if v<low then
                      begin
                        result:=true;
                        res:=true;
                      end
                    else if v>=high then
                      begin
                        result:=true;
                        res:=false;
                      end;
                  lten:
                    if v<=low then
                      begin
                        result:=true;
                        res:=true;
                      end
                    else if v>high then
                      begin
                        result:=true;
                        res:=false;
                      end;
                  gtn:
                    if v<=low then
                      begin
                        result:=true;
                        res:=false;
                      end
                    else if v>high then
                      begin
                        result:=true;
                        res:=true;
                      end;
                  gten :
                    if v<low then
                      begin
                        result:=true;
                        res:=false;
                      end
                    else if v>=high then
                      begin
                        result:=true;
                        res:=true;
                      end;
                  equaln:
                    if (v<low) or (v>high) then
                      begin
                        result:=true;
                        res:=false;
                      end;
                  unequaln:
                    if (v<low) or (v>high) then
                      begin
                        result:=true;
                        res:=true;
                      end;
                  else
                    ;
                 end
             else
               with torddef(realdef) do
                 case nodetype of
                  ltn:
                    if high<v then
                      begin
                        result:=true;
                        res:=true;
                      end
                    else if low>=v then
                      begin
                        result:=true;
                        res:=false;
                      end;
                  lten:
                    if high<=v then
                      begin
                        result:=true;
                        res:=true;
                      end
                    else if low>v then
                      begin
                        result:=true;
                        res:=false;
                      end;
                  gtn:
                    if high<=v then
                      begin
                        result:=true;
                        res:=false;
                      end
                    else if low>v then
                      begin
                        result:=true;
                        res:=true;
                      end;
                  gten:
                    if high<v then
                      begin
                        result:=true;
                        res:=false;
                      end
                    else if low>=v then
                      begin
                        result:=true;
                        res:=true;
                      end;
                  equaln:
                    if (v<low) or (v>high) then
                      begin
                        result:=true;
                        res:=false;
                      end;
                  unequaln:
                    if (v<low) or (v>high) then
                      begin
                        result:=true;
                        res:=true;
                      end;
                  else
                    ;
                 end;
           end;
      end;


    function taddnode.simplify(forinline : boolean) : tnode;

      function is_range_test(nodel, noder: taddnode; out value: tnode; var cl,cr: Tconstexprint): boolean;
        const
          is_upper_test: array[ltn..gten] of boolean = (true,true,false,false);
          inclusive_adjust: array[boolean,ltn..gten] of integer = ((-1,0,1,0),
                                                                   (1,0,-1,0));
        var
          swapl, swapr: Boolean;
          valuer: tnode;
          t: Tconstexprint;
        begin
          result:=false;
          swapl:=false;
          swapr:=false;

          if nodel.left.nodetype=ordconstn then
            begin
              swapl:=true;
              cl:=tordconstnode(nodel.left).value;
              value:=nodel.right;
            end
          else if nodel.right.nodetype=ordconstn then
            begin
              cl:=tordconstnode(nodel.right).value;
              value:=nodel.left;
            end
          else
            exit;

          if noder.left.nodetype=ordconstn then
            begin
              swapl:=true;
              cr:=tordconstnode(noder.left).value;
              valuer:=noder.right;
            end
          else if noder.right.nodetype=ordconstn then
            begin
              cr:=tordconstnode(noder.right).value;
              valuer:=noder.left;
            end
          else
            exit;

          if not value.isequal(valuer) then
            exit;

          { this could be simplified too, but probably never happens }
          if (is_upper_test[nodel.nodetype] xor swapl)=(is_upper_test[noder.nodetype] xor swapr) then
            exit;

          cl:=cl+inclusive_adjust[swapl,nodel.nodetype];
          cr:=cr+inclusive_adjust[swapr,noder.nodetype];

          if is_upper_test[nodel.nodetype] xor swapl then
            begin
              t:=cl;
              cl:=cr;
              cr:=t;
            end;

          if cl>cr then
            exit;

          result:=true;
        end;

      function IsLengthZero(n1,n2 : tnode) : Boolean;
        begin
          result:=is_inlinefunction(n1,in_length_x) and is_constintvalue(n2,0) and not(is_shortstring(tinlinenode(n1).left.resultdef));
        end;


      function TransformLengthZero(n1,n2 : tnode) : tnode;
        var
          len : Tconstexprint;
        begin
          if is_dynamic_array(tinlinenode(n1).left.resultdef) then
            len:=-1
          else
            len:=0;
          result:=caddnode.create_internal(orn,
            caddnode.create_internal(equaln,ctypeconvnode.create_internal(tinlinenode(n1).left.getcopy,voidpointertype),
                cpointerconstnode.create(0,voidpointertype)),
              caddnode.create_internal(equaln,
                ctypeconvnode.create_internal(
                  cderefnode.create(
                    caddnode.create_internal(subn,ctypeconvnode.create_internal(tinlinenode(n1).left.getcopy,voidpointertype),
                      cordconstnode.create(sizesinttype.size,sizesinttype,false))
                  ),sizesinttype
                ),
              cordconstnode.create(len,sizesinttype,false))
            );
        end;


      var
        t       , vl, hp: tnode;
        lt,rt   : tnodetype;
        hdef,
        rd,ld   : tdef;
        rv,lv,v : tconstexprint;
        rvd,lvd : bestreal;
        ws1,ws2 : pcompilerwidestring;
        concatstrings : boolean;
        c1,c2   : array[0..1] of char;
        s1,s2   : pchar;
        l1,l2   : longint;
        resultset : Tconstset;
        res,
        b       : boolean;
        cr, cl  : Tconstexprint;
      begin
        result:=nil;
        l1:=0;
        l2:=0;
        s1:=nil;
        s2:=nil;

        { load easier access variables }
        rd:=right.resultdef;
        ld:=left.resultdef;
        rt:=right.nodetype;
        lt:=left.nodetype;

        if (nodetype = slashn) and
           (((rt = ordconstn) and
             (tordconstnode(right).value = 0)) or
            ((rt = realconstn) and
             (trealconstnode(right).value_real = 0.0))) then
          begin
            if floating_point_range_check_error then
               begin
                 result:=crealconstnode.create(1,pbestrealtype^);
                 Message(parser_e_division_by_zero);
                 exit;
               end;
          end;

        { both are int constants }
        if  (
             is_constintnode(left) and
             is_constintnode(right)
            ) or
            (
             is_constboolnode(left) and
             is_constboolnode(right) and
             (nodetype in [slashn,ltn,lten,gtn,gten,equaln,unequaln,andn,xorn,orn])
            ) or
            (
             is_constenumnode(left) and
             is_constenumnode(right) and
             (allowenumop(nodetype) or (nf_internal in flags))
            ) or
            (
             (lt = pointerconstn) and
             is_constintnode(right) and
             (nodetype in [addn,subn])
            ) or
            (
             (rt = pointerconstn) and
             is_constintnode(left) and
             (nodetype=addn)
            ) or
            (
             (lt in [pointerconstn,niln]) and
             (rt in [pointerconstn,niln]) and
             (nodetype in [ltn,lten,gtn,gten,equaln,unequaln,subn])
            ) or
            (
             (lt = ordconstn) and (ld.typ = orddef) and is_currency(ld) and
             (rt = ordconstn) and (rd.typ = orddef) and is_currency(rd)
            ) then
          begin
             t:=nil;

             { load values }
             case lt of
               ordconstn:
                 lv:=tordconstnode(left).value;
               pointerconstn:
                 lv:=tpointerconstnode(left).value;
               niln:
                 lv:=0;
               else
                 internalerror(2002080202);
             end;
             case rt of
               ordconstn:
                 rv:=tordconstnode(right).value;
               pointerconstn:
                 rv:=tpointerconstnode(right).value;
               niln:
                 rv:=0;
               else
                 internalerror(2002080203);
             end;
             { type checking already took care of multiplying      }
             { integer constants with pointeddef.size if necessary }
             case nodetype of
               addn :
                 begin
                   v:=lv+rv;
                   if v.overflow then
                     begin
                       Message(parser_e_arithmetic_operation_overflow);
                       { Recover }
                       t:=genintconstnode(0)
                     end
                   else if (lt=pointerconstn) or (rt=pointerconstn) then
                     t := cpointerconstnode.create(qword(v),resultdef)
                   else
                     if is_integer(ld) then
                       t := create_simplified_ord_const(v,resultdef,forinline,cs_check_overflow in localswitches)
                     else
                       t := cordconstnode.create(v,resultdef,(ld.typ<>enumdef));
                 end;
               subn :
                 begin
                   v:=lv-rv;
                   if v.overflow then
                     begin
                       Message(parser_e_arithmetic_operation_overflow);
                       { Recover }
                       t:=genintconstnode(0)
                     end
                   else if (lt=pointerconstn) then
                     { pointer-pointer results in an integer }
                     if (rt=pointerconstn) then
                       begin
                         if not(nf_has_pointerdiv in flags) then
                           internalerror(2008030101);
                         t := cpointerconstnode.create(qword(v),resultdef)
                       end
                     else
                       t := cpointerconstnode.create(qword(v),resultdef)
                   else
                     if is_integer(ld) then
                       t := create_simplified_ord_const(v,resultdef,forinline,cs_check_overflow in localswitches)
                     else
                       t:=cordconstnode.create(v,resultdef,(ld.typ<>enumdef));
                 end;
               muln :
                 begin
                   v:=lv*rv;
                   if v.overflow then
                     begin
                       message(parser_e_arithmetic_operation_overflow);
                       { Recover }
                       t:=genintconstnode(0)
                     end
                   else
                     t := create_simplified_ord_const(v,resultdef,forinline,cs_check_overflow in localswitches)
                 end;
               xorn :
                 if is_integer(ld) then
                   t := create_simplified_ord_const(lv xor rv,resultdef,forinline,false)
                 else
                   t:=cordconstnode.create(lv xor rv,resultdef,true);
               orn :
                 if is_integer(ld) then
                   t:=create_simplified_ord_const(lv or rv,resultdef,forinline,false)
                 else
                   t:=cordconstnode.create(lv or rv,resultdef,true);
               andn :
                 if is_integer(ld) then
                   t:=create_simplified_ord_const(lv and rv,resultdef,forinline,false)
                 else
                   t:=cordconstnode.create(lv and rv,resultdef,true);
               ltn :
                 t:=cordconstnode.create(ord(lv<rv),pasbool1type,true);
               lten :
                 t:=cordconstnode.create(ord(lv<=rv),pasbool1type,true);
               gtn :
                 t:=cordconstnode.create(ord(lv>rv),pasbool1type,true);
               gten :
                 t:=cordconstnode.create(ord(lv>=rv),pasbool1type,true);
               equaln :
                 t:=cordconstnode.create(ord(lv=rv),pasbool1type,true);
               unequaln :
                 t:=cordconstnode.create(ord(lv<>rv),pasbool1type,true);
               slashn :
                 begin
                   { int/int becomes a real }
                   rvd:=rv;
                   lvd:=lv;
                   t:=crealconstnode.create(lvd/rvd,resultrealdef);
                 end;
               else
                 internalerror(2008022101);
             end;
             if not forinline then
               include(t.flags,nf_internal);
             result:=t;
             exit;
          end
        else if cmp_of_disjunct_ranges(res) then
          begin
            if res then
              t:=Cordconstnode.create(1,pasbool1type,true)
            else
              t:=Cordconstnode.create(0,pasbool1type,true);
            { don't do this optimization, if the variable expression might
              have a side effect }
            if (is_constintnode(left) and might_have_sideeffects(right)) or
              (is_constintnode(right) and might_have_sideeffects(left)) then
              t.free
            else
              result:=t;
            exit;
          end;

        { Add,Sub,Mul,Or,Xor,Andn with constant 0, 1 or -1?  }
        if is_constintnode(right) and is_integer(left.resultdef) then
          begin
            if tordconstnode(right).value = 0 then
              begin
                case nodetype of
                  addn,subn,orn,xorn:
                    result := left.getcopy;
                  andn,muln:
                    begin
                      if (cs_opt_level4 in current_settings.optimizerswitches) or
                         not might_have_sideeffects(left) then
                        result:=cordconstnode.create(0,resultdef,true);
                    end
                  else
                    ;
                end;
              end
            else if tordconstnode(right).value = 1 then
              begin
                case nodetype of
                  muln:
                   result := left.getcopy;
                  else
                    ;
                end;
              end
            else if tordconstnode(right).value = -1 then
              begin
                case nodetype of
                  muln:
                   result := cunaryminusnode.create(left.getcopy);
                  else
                    ;
                end;
              end
            { try to fold
                          op
                         /  \
                       op  const1
                      /  \
                  const2 val
            }
            else if left.nodetype=nodetype then
              begin
                if is_constintnode(taddnode(left).left) then
                  begin
                    case left.nodetype of
                      xorn,
                      addn,
                      andn,
                      orn,
                      muln:
                        begin
                          hp:=right;
                          right:=taddnode(left).right;
                          taddnode(left).right:=hp;
                          left:=left.simplify(false);
                          result:=getcopy;
                          result.resultdef:=nil;
                          do_typecheckpass(result);
                        end;
                      else
                        ;
                    end;
                  end
                else if is_constintnode(taddnode(left).right) then
                  begin
                    case left.nodetype of
                      xorn,
                      addn,
                      andn,
                      orn,
                      muln:
                        begin
                          hp:=right;
                          right:=taddnode(left).left;
                          taddnode(left).left:=hp;
                          left:=left.simplify(false);
                          result:=getcopy;
                          result.resultdef:=nil;
                          do_typecheckpass(result);
                        end;
                      else
                        ;
                    end;
                  end
              end;
            if assigned(result) then
              exit;
          end;
        if is_constintnode(left) and is_integer(right.resultdef) then
          begin
            if tordconstnode(left).value = 0 then
              begin
                case nodetype of
                  addn,orn,xorn:
                   result := right.getcopy;
                  subn:
                   result := cunaryminusnode.create(right.getcopy);
                  andn,muln:
                    begin
                      if (cs_opt_level4 in current_settings.optimizerswitches) or
                         not might_have_sideeffects(right) then
                        result:=cordconstnode.create(0,resultdef,true);
                    end;
                  else
                    ;
                end;
              end
            else if tordconstnode(left).value = 1 then
              begin
                case nodetype of
                  muln:
                   result := right.getcopy;
                  else
                    ;
                end;
              end
            else if tordconstnode(left).value = -1 then
              begin
                case nodetype of
                  muln:
                   result := cunaryminusnode.create(right.getcopy);
                  else
                    ;
                end;
              end
            { try to fold
                          op
                         /  \
                     const1  op
                            /  \
                        const2 val
            }
            else if right.nodetype=nodetype then
              begin
                if is_constintnode(taddnode(right).left) then
                  begin
                    case right.nodetype of
                      xorn,
                      addn,
                      andn,
                      orn,
                      muln:
                        begin
                          hp:=left;
                          left:=taddnode(right).right;
                          taddnode(right).right:=hp;
                          right:=right.simplify(false);
                          result:=getcopy;
                          result.resultdef:=nil;
                          do_typecheckpass(result);
                        end;
                      else
                        ;
                    end;
                  end
                else if is_constintnode(taddnode(right).right) then
                  begin
                    case right.nodetype of
                      xorn,
                      addn,
                      andn,
                      orn,
                      muln:
                        begin
                          hp:=left;
                          left:=taddnode(right).left;
                          taddnode(right).left:=hp;
                          right:=right.simplify(false);
                          result:=getcopy;
                          result.resultdef:=nil;
                          do_typecheckpass(result);
                        end;
                      else
                        ;
                    end;
                  end
              end;
            if assigned(result) then
              exit;
          end;

      { both real constants ? }
        if (lt=realconstn) and (rt=realconstn) then
          begin
             lvd:=trealconstnode(left).value_real;
             rvd:=trealconstnode(right).value_real;
             case nodetype of
                addn :
                  t:=crealconstnode.create(lvd+rvd,resultrealdef);
                subn :
                  t:=crealconstnode.create(lvd-rvd,resultrealdef);
                muln :
                  t:=crealconstnode.create(lvd*rvd,resultrealdef);
                starstarn:
                  begin
                    if lvd<0 then
                     begin
                       Message(parser_e_invalid_float_operation);
                       t:=crealconstnode.create(0,resultrealdef);
                     end
                    else if lvd=0 then
                      t:=crealconstnode.create(1.0,resultrealdef)
                    else
                      t:=crealconstnode.create(exp(ln(lvd)*rvd),resultrealdef);
                  end;
                slashn :
                  t:=crealconstnode.create(lvd/rvd,resultrealdef);
                ltn :
                  t:=cordconstnode.create(ord(lvd<rvd),pasbool1type,true);
                lten :
                  t:=cordconstnode.create(ord(lvd<=rvd),pasbool1type,true);
                gtn :
                  t:=cordconstnode.create(ord(lvd>rvd),pasbool1type,true);
                gten :
                  t:=cordconstnode.create(ord(lvd>=rvd),pasbool1type,true);
                equaln :
                  t:=cordconstnode.create(ord(lvd=rvd),pasbool1type,true);
                unequaln :
                  t:=cordconstnode.create(ord(lvd<>rvd),pasbool1type,true);
                else
                  internalerror(2008022102);
             end;
             result:=t;
             if nf_is_currency in flags then
               include(result.flags,nf_is_currency);
             exit;
          end;
{$if (FPC_FULLVERSION>20700) and not defined(FPC_SOFT_FPUX80)}
        { bestrealrec is 2.7.1+ only }

        { replace .../const by a multiplication, but only if fastmath is enabled or
          the division is done by a power of 2, do not mess with special floating point values like Inf etc.

          do this after constant folding to avoid unnecessary precision loss if
          an slash expresion would be first converted into a multiplication and later
          folded }
        if (nodetype=slashn) and
          { do not mess with currency and comp types }
          (not(is_currency(right.resultdef)) and
           not((right.resultdef.typ=floatdef) and
               (tfloatdef(right.resultdef).floattype=s64comp)
              )
          ) and
          (((cs_opt_fastmath in current_settings.optimizerswitches) and (rt=ordconstn)) or
           ((cs_opt_fastmath in current_settings.optimizerswitches) and (rt=realconstn) and
            (bestrealrec(trealconstnode(right).value_real).SpecialType in [fsPositive,fsNegative])
           ) or
           ((rt=realconstn) and
            (bestrealrec(trealconstnode(right).value_real).SpecialType in [fsPositive,fsNegative]) and
            { mantissa returns the mantissa/fraction without the hidden 1, so power of two means only the hidden
              bit is set => mantissa must be 0 }
            (bestrealrec(trealconstnode(right).value_real).Mantissa=0)
           )
          ) then
          case rt of
            ordconstn:
              begin
                { the normal code handles div/0 }
                if (tordconstnode(right).value<>0) then
                  begin
                    nodetype:=muln;
                    t:=crealconstnode.create(1/tordconstnode(right).value,resultdef);
                    right.free;
                    right:=t;
                    exit;
                  end;
              end;
            realconstn:
              begin
                nodetype:=muln;
                trealconstnode(right).value_real:=1.0/trealconstnode(right).value_real;
                exit;
              end;
            else
              ;
          end;
{$endif FPC_FULLVERSION>20700}

        { first, we handle widestrings, so we can check later for }
        { stringconstn only                                       }

        { widechars are converted above to widestrings too }
        { this isn't ver y efficient, but I don't think    }
        { that it does matter that much (FK)               }
        if (lt=stringconstn) and (rt=stringconstn) and
          (tstringconstnode(left).cst_type in [cst_widestring,cst_unicodestring]) and
          (tstringconstnode(right).cst_type in [cst_widestring,cst_unicodestring]) then
          begin
             initwidestring(ws1);
             initwidestring(ws2);
             copywidestring(pcompilerwidestring(tstringconstnode(left).value_str),ws1);
             copywidestring(pcompilerwidestring(tstringconstnode(right).value_str),ws2);
             case nodetype of
                addn :
                  begin
                     concatwidestrings(ws1,ws2);
                     t:=cstringconstnode.createunistr(ws1);
                  end;
                ltn :
                  t:=cordconstnode.create(byte(comparewidestrings(ws1,ws2)<0),pasbool1type,true);
                lten :
                  t:=cordconstnode.create(byte(comparewidestrings(ws1,ws2)<=0),pasbool1type,true);
                gtn :
                  t:=cordconstnode.create(byte(comparewidestrings(ws1,ws2)>0),pasbool1type,true);
                gten :
                  t:=cordconstnode.create(byte(comparewidestrings(ws1,ws2)>=0),pasbool1type,true);
                equaln :
                  t:=cordconstnode.create(byte(comparewidestrings(ws1,ws2)=0),pasbool1type,true);
                unequaln :
                  t:=cordconstnode.create(byte(comparewidestrings(ws1,ws2)<>0),pasbool1type,true);
                else
                  internalerror(2008022103);
             end;
             donewidestring(ws1);
             donewidestring(ws2);
             result:=t;
             exit;
          end;

        { concating strings ? }
        concatstrings:=false;

        if (lt=ordconstn) and (rt=ordconstn) and
           is_char(ld) and is_char(rd) then
          begin
             c1[0]:=char(int64(tordconstnode(left).value));
             c1[1]:=#0;
             l1:=1;
             c2[0]:=char(int64(tordconstnode(right).value));
             c2[1]:=#0;
             l2:=1;
             s1:=@c1[0];
             s2:=@c2[0];
             concatstrings:=true;
          end
        else if (lt=stringconstn) and (rt=ordconstn) and is_char(rd) then
          begin
             s1:=tstringconstnode(left).value_str;
             l1:=tstringconstnode(left).len;
             c2[0]:=char(int64(tordconstnode(right).value));
             c2[1]:=#0;
             s2:=@c2[0];
             l2:=1;
             concatstrings:=true;
          end
        else if (lt=ordconstn) and (rt=stringconstn) and is_char(ld) then
          begin
             c1[0]:=char(int64(tordconstnode(left).value));
             c1[1]:=#0;
             l1:=1;
             s1:=@c1[0];
             s2:=tstringconstnode(right).value_str;
             l2:=tstringconstnode(right).len;
             concatstrings:=true;
          end
        else if (lt=stringconstn) and (rt=stringconstn) then
          begin
             s1:=tstringconstnode(left).value_str;
             l1:=tstringconstnode(left).len;
             s2:=tstringconstnode(right).value_str;
             l2:=tstringconstnode(right).len;
             concatstrings:=true;
          end;
        if concatstrings then
          begin
             case nodetype of
                addn :
                  begin
                    t:=cstringconstnode.createpchar(concatansistrings(s1,s2,l1,l2),l1+l2,nil);
                    typecheckpass(t);
                    if not is_ansistring(resultdef) or
                       (tstringdef(resultdef).encoding<>globals.CP_NONE) then
                      tstringconstnode(t).changestringtype(resultdef)
                    else
                      tstringconstnode(t).changestringtype(getansistringdef)
                  end;
                ltn :
                  t:=cordconstnode.create(byte(compareansistrings(s1,s2,l1,l2)<0),pasbool1type,true);
                lten :
                  t:=cordconstnode.create(byte(compareansistrings(s1,s2,l1,l2)<=0),pasbool1type,true);
                gtn :
                  t:=cordconstnode.create(byte(compareansistrings(s1,s2,l1,l2)>0),pasbool1type,true);
                gten :
                  t:=cordconstnode.create(byte(compareansistrings(s1,s2,l1,l2)>=0),pasbool1type,true);
                equaln :
                  t:=cordconstnode.create(byte(compareansistrings(s1,s2,l1,l2)=0),pasbool1type,true);
                unequaln :
                  t:=cordconstnode.create(byte(compareansistrings(s1,s2,l1,l2)<>0),pasbool1type,true);
                else
                  internalerror(2008022104);
             end;
             result:=t;
             exit;
          end;

        { set constant evaluation }
        if (right.nodetype=setconstn) and
           not assigned(tsetconstnode(right).left) and
           (left.nodetype=setconstn) and
           not assigned(tsetconstnode(left).left) then
          begin
             case nodetype of
               addn :
                 begin
                   resultset:=tsetconstnode(right).value_set^ + tsetconstnode(left).value_set^;
                   t:=csetconstnode.create(@resultset,resultdef);
                 end;
                muln :
                  begin
                    resultset:=tsetconstnode(right).value_set^ * tsetconstnode(left).value_set^;
                    t:=csetconstnode.create(@resultset,resultdef);
                  end;
               subn :
                  begin
                    resultset:=tsetconstnode(left).value_set^ - tsetconstnode(right).value_set^;
                            t:=csetconstnode.create(@resultset,resultdef);
                  end;
               symdifn :
                  begin
                    resultset:=tsetconstnode(right).value_set^ >< tsetconstnode(left).value_set^;
                        t:=csetconstnode.create(@resultset,resultdef);
                  end;
               unequaln :
                  begin
                    b:=tsetconstnode(right).value_set^ <> tsetconstnode(left).value_set^;
                    t:=cordconstnode.create(byte(b),pasbool1type,true);
                  end;
               equaln :
                  begin
                    b:=tsetconstnode(right).value_set^ = tsetconstnode(left).value_set^;
                    t:=cordconstnode.create(byte(b),pasbool1type,true);
                  end;
               lten :
                  begin
                    b:=tsetconstnode(left).value_set^ <= tsetconstnode(right).value_set^;
                    t:=cordconstnode.create(byte(b),pasbool1type,true);
                  end;
               gten :
                  begin
                    b:=tsetconstnode(left).value_set^ >= tsetconstnode(right).value_set^;
                    t:=cordconstnode.create(byte(b),pasbool1type,true);
                  end;
                else
                  internalerror(2008022105);
             end;
             result:=t;
             exit;
          end;

        { in case of expressions having no side effect, we can simplify boolean expressions
          containing constants }
        if is_boolean(left.resultdef) and is_boolean(right.resultdef) then
          begin
            if is_constboolnode(left) and not(might_have_sideeffects(right)) then
              begin
                if ((nodetype=andn) and (tordconstnode(left).value<>0)) or
                  ((nodetype=orn) and (tordconstnode(left).value=0)) or
                  ((nodetype=xorn) and (tordconstnode(left).value=0)) then
                  begin
                    result:=right;
                    right:=nil;
                    exit;
                  end
                else if ((nodetype=orn) and (tordconstnode(left).value<>0)) or
                  ((nodetype=andn) and (tordconstnode(left).value=0)) then
                  begin
                    result:=left;
                    left:=nil;
                    exit;
                  end
                else if ((nodetype=xorn) and (tordconstnode(left).value<>0)) then
                  begin
                    result:=cnotnode.create(right);
                    right:=nil;
                    exit;
                  end
              end
            else if is_constboolnode(right) and not(might_have_sideeffects(left)) then
              begin
                if ((nodetype=andn) and (tordconstnode(right).value<>0)) or
                  ((nodetype=orn) and (tordconstnode(right).value=0)) or
                  ((nodetype=xorn) and (tordconstnode(right).value=0)) then
                  begin
                    result:=left;
                    left:=nil;
                    exit;
                  end
                else if ((nodetype=orn) and (tordconstnode(right).value<>0)) or
                  ((nodetype=andn) and (tordconstnode(right).value=0)) then
                  begin
                    result:=right;
                    right:=nil;
                    exit;
                  end
                else if ((nodetype=xorn) and (tordconstnode(right).value<>0)) then
                  begin
                    result:=cnotnode.create(left);
                    left:=nil;
                    exit;
                  end
              end;
          end;

        { slow simplifications }
        if cs_opt_level2 in current_settings.optimizerswitches then
          begin
            { the comparison is might be expensive and the nodes are usually only
              equal if some previous optimizations were done so don't check
              this simplification always
            }
            if is_boolean(left.resultdef) and is_boolean(right.resultdef) then
              begin
                { transform unsigned comparisons of (v>=x) and (v<=y)
                  into (v-x)<=(y-x)
                }
                if (nodetype=andn) and
                   (left.nodetype in [ltn,lten,gtn,gten]) and
                   (right.nodetype in [ltn,lten,gtn,gten]) and
                   (not might_have_sideeffects(left)) and
                   (not might_have_sideeffects(right)) and
                   is_range_test(taddnode(left),taddnode(right),vl,cl,cr) then
                  begin
                    hdef:=get_unsigned_inttype(vl.resultdef);
                    vl:=ctypeconvnode.create_internal(vl.getcopy,hdef);

                    result:=caddnode.create_internal(lten,
                              ctypeconvnode.create_internal(caddnode.create_internal(subn,vl,cordconstnode.create(cl,hdef,false)),hdef),
                              cordconstnode.create(cr-cl,hdef,false));
                    exit;
                  end;

                { even when short circuit boolean evaluation is active, this
                  optimization cannot be performed in case the node has
                  side effects, because this can change the result (e.g., in an
                  or-node that calls the same function twice and first returns
                  false and then true because of a global state change }
                if left.isequal(right) and not might_have_sideeffects(left) then
                  begin
                    case nodetype of
                      andn,orn:
                        begin
                          result:=left;
                          left:=nil;
                          exit;
                        end;
                      {
                      xorn:
                        begin
                          result:=cordconstnode.create(0,resultdef,true);
                          exit;
                        end;
                      }
                      else
                        ;
                    end;
                  end
                { short to full boolean evalution possible and useful? }
                else if not(might_have_sideeffects(right,[mhs_exceptions])) and not(cs_full_boolean_eval in localswitches) then
                  begin
                    case nodetype of
                      andn,orn:
                        begin
                          { full boolean evaluation is only useful if the nodes are not too complex and if no flags/jumps must be converted,
                            further, we need to know the expectloc }
                          if (node_complexity(right)<=2) and
                            not(left.expectloc in [LOC_FLAGS,LOC_JUMP,LOC_INVALID]) and not(right.expectloc in [LOC_FLAGS,LOC_JUMP,LOC_INVALID]) then
                            begin
                              { we need to copy the whole tree to force another pass_1 }
                              include(localswitches,cs_full_boolean_eval);
                              result:=getcopy;
                              exit;
                            end;
                        end;
                      else
                        ;
                    end;
                  end
              end;

            if is_integer(left.resultdef) and is_integer(right.resultdef) then
              begin
                if (cs_opt_level3 in current_settings.optimizerswitches) and
                   left.isequal(right) and not might_have_sideeffects(left) then
                  begin
                    case nodetype of
                      andn,orn:
                        begin
                          result:=left;
                          left:=nil;
                          exit;
                        end;
                      xorn,
                      subn,
                      unequaln,
                      ltn,
                      gtn:
                        begin
                          result:=cordconstnode.create(0,resultdef,true);
                          exit;
                        end;
                      equaln,
                      lten,
                      gten:
                        begin
                          result:=cordconstnode.create(1,resultdef,true);
                          exit;
                        end;
                      else
                        ;
                    end;
                  end
{$ifndef jvm}
                else if (nodetype=equaln) and MatchAndTransformNodesCommutative(left,right,@IsLengthZero,@TransformLengthZero,Result) then
                   exit
{$endif jvm}
                   ;
              end;

            { using sqr(x) for reals instead of x*x might reduces register pressure and/or
              memory accesses while sqr(<real>) has no drawback }
            if
{$ifdef cpufpemu}
               (current_settings.fputype<>fpu_soft) and
               not(cs_fp_emulation in current_settings.moduleswitches) and
{$endif cpufpemu}
               (nodetype=muln) and
               is_real(left.resultdef) and is_real(right.resultdef) and
               left.isequal(right) and
               not(might_have_sideeffects(left)) then
              begin
                result:=cinlinenode.create(in_sqr_real,false,left);
                left:=nil;
                exit;
              end;
{$ifdef cpurox}
            { optimize (i shl x) or (i shr (bitsizeof(i)-x)) into rol(x,i) (and different flavours with shl/shr swapped etc.) }
            if (nodetype=orn)
{$ifdef m68k}
               and (CPUM68K_HAS_ROLROR in cpu_capabilities[current_settings.cputype])
{$endif m68k}
{$ifndef cpu64bitalu}
               and (left.resultdef.typ=orddef) and
               not(torddef(left.resultdef).ordtype in [s64bit,u64bit,scurrency])
{$endif cpu64bitalu}
              then
              begin
                if (left.nodetype=shrn) and (right.nodetype=shln) and
                   is_constintnode(tshlshrnode(left).right) and
                   is_constintnode(tshlshrnode(right).right) and
                   (tordconstnode(tshlshrnode(right).right).value>0) and
                   (tordconstnode(tshlshrnode(left).right).value>0) and
                   tshlshrnode(left).left.isequal(tshlshrnode(right).left) and
                   not(might_have_sideeffects(tshlshrnode(left).left)) then
                   begin
                     if (tordconstnode(tshlshrnode(left).right).value=
                       tshlshrnode(left).left.resultdef.size*8-tordconstnode(tshlshrnode(right).right).value)
                        then
                       begin
                         result:=cinlinenode.create(in_ror_x_y,false,
                           ccallparanode.create(tshlshrnode(left).right,
                           ccallparanode.create(tshlshrnode(left).left,nil)));
                         tshlshrnode(left).left:=nil;
                         tshlshrnode(left).right:=nil;
                         exit;
                       end
                     else if (tordconstnode(tshlshrnode(right).right).value=
                       tshlshrnode(left).left.resultdef.size*8-tordconstnode(tshlshrnode(left).right).value)
                        then
                       begin
                         result:=cinlinenode.create(in_rol_x_y,false,
                           ccallparanode.create(tshlshrnode(right).right,
                           ccallparanode.create(tshlshrnode(left).left,nil)));
                         tshlshrnode(left).left:=nil;
                         tshlshrnode(right).right:=nil;
                         exit;
                       end;
                   end;
                if (left.nodetype=shln) and (right.nodetype=shrn) and
                   is_constintnode(tshlshrnode(left).right) and
                   is_constintnode(tshlshrnode(right).right) and
                   (tordconstnode(tshlshrnode(right).right).value>0) and
                   (tordconstnode(tshlshrnode(left).right).value>0) and
                   tshlshrnode(left).left.isequal(tshlshrnode(right).left) and
                   not(might_have_sideeffects(tshlshrnode(left).left)) then
                   begin
                     if (tordconstnode(tshlshrnode(left).right).value=
                       tshlshrnode(left).left.resultdef.size*8-tordconstnode(tshlshrnode(right).right).value)
                        then
                       begin
                         result:=cinlinenode.create(in_rol_x_y,false,
                           ccallparanode.create(tshlshrnode(left).right,
                           ccallparanode.create(tshlshrnode(left).left,nil)));
                         tshlshrnode(left).left:=nil;
                         tshlshrnode(left).right:=nil;
                         exit;
                       end
                     else if (tordconstnode(tshlshrnode(right).right).value=
                       tshlshrnode(left).left.resultdef.size*8-tordconstnode(tshlshrnode(left).right).value)
                        then
                       begin
                         result:=cinlinenode.create(in_ror_x_y,false,
                           ccallparanode.create(tshlshrnode(right).right,
                           ccallparanode.create(tshlshrnode(left).left,nil)));
                         tshlshrnode(left).left:=nil;
                         tshlshrnode(right).right:=nil;
                         exit;
                       end;
                   end;
              end;
{$endif cpurox}
          end;
      end;


    function taddnode.dogetcopy: tnode;
      var
        n: taddnode;
      begin
        n:=taddnode(inherited dogetcopy);
        n.resultrealdef:=resultrealdef;
        result:=n;
      end;


    function taddnode.docompare(p: tnode): boolean;
      begin
        result:=
          inherited docompare(p) and
          equal_defs(taddnode(p).resultrealdef,resultrealdef);
      end;


    function taddnode.pass_typecheck:tnode;
      begin
        { This function is small to keep the stack small for recursive of
          large + operations }
        typecheckpass(left);
        typecheckpass(right);
        result:=pass_typecheck_internal;
      end;


    function taddnode.pass_typecheck_internal:tnode;
      var
        hp          : tnode;
        rd,ld,nd    : tdef;
        hsym        : tfieldvarsym;
        llow,lhigh,
        rlow,rhigh  : tconstexprint;
        strtype     : tstringtype;
        res,
        b           : boolean;
        lt,rt       : tnodetype;
        ot          : tnodetype;
{$ifdef state_tracking}
        factval     : Tnode;
        change      : boolean;
{$endif}

        function maybe_cast_ordconst(var n: tnode; adef: tdef): boolean;
          begin
            result:=(tordconstnode(n).value>=torddef(adef).low) and
              (tordconstnode(n).value<=torddef(adef).high);
            if result then
              inserttypeconv(n,adef);
          end;

        function maybe_convert_to_insert:tnode;

          function element_count(arrconstr: tarrayconstructornode):asizeint;
            begin
              result:=0;
              while assigned(arrconstr) do
                begin
                  if arrconstr.nodetype=arrayconstructorrangen then
                    internalerror(2018052501);
                  inc(result);
                  arrconstr:=tarrayconstructornode(tarrayconstructornode(arrconstr).right);
                end;
            end;

          var
            elem : tnode;
            para : tcallparanode;
            isarrconstrl,
            isarrconstrr : boolean;
            index : asizeint;
          begin
            result:=nil;

            isarrconstrl:=left.nodetype=arrayconstructorn;
            isarrconstrr:=right.nodetype=arrayconstructorn;

            if not assigned(aktassignmentnode) or
                (aktassignmentnode.right<>self) or
                not(
                  isarrconstrl or
                  isarrconstrr
                ) or
                not(
                  left.isequal(aktassignmentnode.left) or
                  right.isequal(aktassignmentnode.left)
                ) or
                not valid_for_var(aktassignmentnode.left,false) or
                (isarrconstrl and (element_count(tarrayconstructornode(left))>1)) or
                (isarrconstrr and (element_count(tarrayconstructornode(right))>1)) then
              exit;

            if isarrconstrl then
              begin
                index:=0;
                elem:=tarrayconstructornode(left).left;
                tarrayconstructornode(left).left:=nil;
              end
            else
              begin
                index:=high(asizeint);
                elem:=tarrayconstructornode(right).left;
                tarrayconstructornode(right).left:=nil;
              end;

            { we use the fact that insert() caps the index to avoid a copy }
            para:=ccallparanode.create(
                    cordconstnode.create(index,sizesinttype,false),
                    ccallparanode.create(
                      aktassignmentnode.left.getcopy,
                      ccallparanode.create(
                        elem,nil)));

            result:=cinlinenode.create(in_insert_x_y_z,false,para);
            include(aktassignmentnode.flags,nf_assign_done_in_right);
          end;

      begin
         result:=nil;
         rlow:=0;
         llow:=0;
         rhigh:=0;
         lhigh:=0;

         { avoid any problems with type parameters later on }
         if is_typeparam(left.resultdef) or is_typeparam(right.resultdef) then
           begin
             resultdef:=cundefinedtype;
             exit;
           end;

         { both left and right need to be valid }
         set_varstate(left,vs_read,[vsf_must_be_valid]);
         set_varstate(right,vs_read,[vsf_must_be_valid]);
         if codegenerror then
           exit;

         { tp procvar support. Omit for converted assigned() nodes }
         if not (nf_load_procvar in flags) then
           begin
             maybe_call_procvar(left,true);
             maybe_call_procvar(right,true);
           end
         else
           if not (nodetype in [equaln,unequaln]) then
             InternalError(2013091601);

         { allow operator overloading }
         hp:=self;

         if is_array_constructor(left.resultdef) or is_array_constructor(right.resultdef) then
           begin
             { check whether there is a suitable operator for the array constructor
               (but only if the "+" array operator isn't used), if not fall back to sets }
             if (
                   (nodetype<>addn) or
                   not (m_array_operators in current_settings.modeswitches) or
                   (is_array_constructor(left.resultdef) and not is_dynamic_array(right.resultdef)) or
                   (not is_dynamic_array(left.resultdef) and is_array_constructor(right.resultdef))
                 ) and
                 not isbinaryoverloaded(hp,[ocf_check_only]) then
               begin
                 if is_array_constructor(left.resultdef) then
                   begin
                     arrayconstructor_to_set(left);
                     typecheckpass(left);
                   end;
                 if is_array_constructor(right.resultdef) then
                   begin
                     arrayconstructor_to_set(right);
                     typecheckpass(right);
                   end;
               end;
           end;

         if is_dynamic_array(left.resultdef) and is_dynamic_array(right.resultdef) and
             (nodetype=addn) and
             (m_array_operators in current_settings.modeswitches) and
             isbinaryoverloaded(hp,[ocf_check_non_overloadable,ocf_check_only]) then
           message3(parser_w_operator_overloaded_hidden_3,left.resultdef.typename,arraytokeninfo[_PLUS].str,right.resultdef.typename);

         if isbinaryoverloaded(hp,[]) then
           begin
              result:=hp;
              exit;
           end;
         { Stop checking when an error was found in the operator checking }
         if codegenerror then
           begin
             result:=cerrornode.create;
             exit;
           end;

         { Kylix allows enum+ordconstn in an enum type declaration, we need to do
           the conversion here before the constant folding }
         if (m_delphi in current_settings.modeswitches) and
            (blocktype in [bt_type,bt_const_type,bt_var_type]) then
          begin
            if (left.resultdef.typ=enumdef) and
               (right.resultdef.typ=orddef) then
             begin
               { insert explicit typecast to default signed int }
               left:=ctypeconvnode.create_internal(left,sinttype);
               typecheckpass(left);
             end
            else
             if (left.resultdef.typ=orddef) and
                (right.resultdef.typ=enumdef) then
              begin
                { insert explicit typecast to default signed int }
                right:=ctypeconvnode.create_internal(right,sinttype);
                typecheckpass(right);
              end;
          end;

        { is one a real float, then both need to be floats, this
          need to be done before the constant folding so constant
          operation on a float and int are also handled }
{$ifdef x86}
        { use extended as default real type only when the x87 fpu is used }
  {$if defined(i386) or defined(i8086)}
        if not(current_settings.fputype=fpu_x87) then
          resultrealdef:=s64floattype
        else
          resultrealdef:=pbestrealtype^;
  {$endif i386 or i8086}
  {$ifdef x86_64}
        { x86-64 has no x87 only mode, so use always double as default }
        resultrealdef:=s64floattype;
  {$endif x86_6}
{$else not x86}
        resultrealdef:=pbestrealtype^;
{$endif not x86}

        if (right.resultdef.typ=floatdef) or (left.resultdef.typ=floatdef) then
         begin
           { when both floattypes are already equal then use that
             floattype for results }
           if (right.resultdef.typ=floatdef) and
              (left.resultdef.typ=floatdef) and
              (tfloatdef(left.resultdef).floattype=tfloatdef(right.resultdef).floattype) then
             begin
               if cs_excessprecision in current_settings.localswitches then
                 resultrealdef:=pbestrealtype^
               else
                 resultrealdef:=left.resultdef
             end
           { when there is a currency type then use currency, but
             only when currency is defined as float }
           else
            if (is_currency(right.resultdef) or
                is_currency(left.resultdef)) and
               ((s64currencytype.typ = floatdef) or
                (nodetype <> slashn)) then
             begin
               resultrealdef:=s64currencytype;
               inserttypeconv(right,resultrealdef);
               inserttypeconv(left,resultrealdef);
             end
           else
            begin
              resultrealdef:=getbestreal(left.resultdef,right.resultdef);
              inserttypeconv(right,resultrealdef);
              inserttypeconv(left,resultrealdef);
            end;
         end;

        { If both operands are constant and there is a unicodestring
          or unicodestring then convert everything to unicodestring }
        if is_constnode(right) and is_constnode(left) and
           (is_unicodestring(right.resultdef) or
            is_unicodestring(left.resultdef)) then
          begin
            inserttypeconv(right,cunicodestringtype);
            inserttypeconv(left,cunicodestringtype);
          end;

        { If both operands are constant and there is a widechar
          or widestring then convert everything to widestring. This
          allows constant folding like char+widechar }
        if is_constnode(right) and is_constnode(left) and
           (is_widestring(right.resultdef) or
            is_widestring(left.resultdef) or
            is_widechar(right.resultdef) or
            is_widechar(left.resultdef)) then
          begin
            inserttypeconv(right,cwidestringtype);
            inserttypeconv(left,cwidestringtype);
          end;

        { load easier access variables }
        rd:=right.resultdef;
        ld:=left.resultdef;
        rt:=right.nodetype;
        lt:=left.nodetype;

         { 4 character constant strings are compatible with orddef }
         { in macpas mode (become cardinals)                       }
         if (m_mac in current_settings.modeswitches) and
            { only allow for comparisons, additions etc are }
            { normally program errors                       }
            (nodetype in [ltn,lten,gtn,gten,unequaln,equaln]) and
            (((lt=stringconstn) and
              (tstringconstnode(left).len=4) and
              (rd.typ=orddef)) or
             ((rt=stringconstn) and
              (tstringconstnode(right).len=4) and
              (ld.typ=orddef))) then
           begin
             if (rt=stringconstn) then
               begin
                 inserttypeconv(right,u32inttype);
                 rt:=right.nodetype;
                 rd:=right.resultdef;
               end
             else
               begin
                 inserttypeconv(left,u32inttype);
                 lt:=left.nodetype;
                 ld:=left.resultdef;
               end;
           end;

         { but an int/int gives real/real! }
         if (nodetype=slashn) and not(is_vector(left.resultdef)) and not(is_vector(right.resultdef)) then
          begin
            if is_currency(left.resultdef) and
               is_currency(right.resultdef) then
              { In case of currency, converting to float means dividing by 10000 }
              { However, since this is already a division, both divisions by     }
              { 10000 are eliminated when we divide the results -> we can skip   }
              { them.                                                            }
              if s64currencytype.typ = floatdef then
                begin
                  { there's no s64comptype or so, how do we avoid the type conversion?
                  left.resultdef := s64comptype;
                  right.resultdef := s64comptype; }
                end
              else
                begin
                  left.resultdef := s64inttype;
                  right.resultdef := s64inttype;
                end;
            if current_settings.fputype=fpu_none then
              begin
                Message(parser_e_unsupported_real);
                result:=cerrornode.create;
                exit;
              end
            else
              begin
                inserttypeconv(right,resultrealdef);
                inserttypeconv(left,resultrealdef);
              end;
          end

         { if both are orddefs then check sub types }
         else if (ld.typ=orddef) and (rd.typ=orddef) then
           begin
              { set for & and | operations in macpas mode: they only work on }
              { booleans, and always short circuit evaluation                }
              if (nf_short_bool in flags) then
                begin
                  if not is_boolean(ld) then
                    begin
                      inserttypeconv(left,pasbool1type);
                      ld := left.resultdef;
                    end;
                  if not is_boolean(rd) then
                    begin
                      inserttypeconv(right,pasbool1type);
                      rd := right.resultdef;
                    end;
                end;

             { 2 booleans? }
             if (is_boolean(ld) and is_boolean(rd)) then
              begin
                case nodetype of
                  xorn,
                  andn,
                  orn:
                    begin
                      { in case of xor, or 'and' with full  and cbool: convert both to Pascal bool and then
                        perform the xor/and to prevent issues with "longbool(1) and/xor
                        longbool(2)" }
                      if (is_cbool(ld) or is_cbool(rd)) and
                         ((nodetype=xorn) or
                          ((nodetype=andn) and
                           ((cs_full_boolean_eval in current_settings.localswitches) or
                            not(nf_short_bool in flags)
                           )
                          )
                         ) then
                        begin
                          resultdef:=nil;
                          if is_cbool(ld) then
                            begin
                              inserttypeconv(left,pasbool8type);
                              { inserttypeconv might already simplify
                                the typeconvnode after insertion,
                                thus we need to check if it still
                                really is a typeconv node }
                              if left is ttypeconvnode then
                                ttypeconvnode(left).convtype:=tc_bool_2_bool;
                              if not is_cbool(rd) or
                                 (ld.size>=rd.size) then
                                resultdef:=ld;
                            end;
                          if is_cbool(rd) then
                            begin
                              inserttypeconv(right,pasbool8type);
                              { inserttypeconv might already simplify
                                the typeconvnode after insertion,
                                thus we need to check if it still
                                really is a typeconv node }
                              if right is ttypeconvnode then
                                ttypeconvnode(right).convtype:=tc_bool_2_bool;
                              if not assigned(resultdef) then
                                resultdef:=rd;
                            end;
                          result:=ctypeconvnode.create_explicit(caddnode.create(nodetype,left,right),resultdef);
                          ttypeconvnode(result).convtype:=tc_bool_2_bool;
                          left:=nil;
                          right:=nil;
                          exit;
                        end;
                      { Make sides equal to the largest boolean }
                      if (torddef(left.resultdef).size>torddef(right.resultdef).size) or
                        (is_cbool(left.resultdef) and not is_cbool(right.resultdef)) then
                        begin
                          right:=ctypeconvnode.create_internal(right,left.resultdef);
                          ttypeconvnode(right).convtype:=tc_bool_2_bool;
                          typecheckpass(right);
                        end
                      else if (torddef(left.resultdef).size<torddef(right.resultdef).size) or
                        (not is_cbool(left.resultdef) and is_cbool(right.resultdef)) then
                        begin
                          left:=ctypeconvnode.create_internal(left,right.resultdef);
                          ttypeconvnode(left).convtype:=tc_bool_2_bool;
                          typecheckpass(left);
                        end;
                    end;
                  ltn,
                  lten,
                  gtn,
                  gten:
                    begin
                      { convert both to pasbool to perform the comparison (so
                        that longbool(4) = longbool(2), since both represent
                        "true" }
                      inserttypeconv(left,pasbool1type);
                      inserttypeconv(right,pasbool1type);
                    end;
                  unequaln,
                  equaln:
                    begin
                      if not(cs_full_boolean_eval in current_settings.localswitches) or
                         (nf_short_bool in flags) then
                       begin
                         { Remove any compares with constants }
                         if (left.nodetype=ordconstn) then
                          begin
                            hp:=right;
                            b:=(tordconstnode(left).value<>0);
                            ot:=nodetype;
                            left.free;
                            left:=nil;
                            right:=nil;
                            if (not(b) and (ot=equaln)) or
                               (b and (ot=unequaln)) then
                             begin
                               hp:=cnotnode.create(hp);
                             end;
                            result:=hp;
                            exit;
                          end;
                         if (right.nodetype=ordconstn) then
                          begin
                            hp:=left;
                            b:=(tordconstnode(right).value<>0);
                            ot:=nodetype;
                            right.free;
                            right:=nil;
                            left:=nil;
                            if (not(b) and (ot=equaln)) or
                               (b and (ot=unequaln)) then
                             begin
                               hp:=cnotnode.create(hp);
                             end;
                            result:=hp;
                            exit;
                          end;
                       end;
                      { Delphi-compatibility: convert both to pasbool to
                        perform the equality comparison }
                      inserttypeconv(left,pasbool1type);
                      inserttypeconv(right,pasbool1type);
                    end;
                  else
                    begin
                      CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
                      result:=cnothingnode.create;
                      exit;
                    end;
                end;
              end
             { Both are chars? }
             else if is_char(rd) and is_char(ld) then
               begin
                 if nodetype=addn then
                  begin
                    resultdef:=cshortstringtype;
                    if not(is_constcharnode(left) and is_constcharnode(right)) then
                     begin
                       inserttypeconv(left,cshortstringtype);
{$ifdef addstringopt}
                       hp := genaddsstringcharoptnode(self);
                       result := hp;
                       exit;
{$endif addstringopt}
                     end
                  end
                else if not(nodetype in [ltn,lten,gtn,gten,unequaln,equaln]) then
                  begin
                    CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
                    result:=cnothingnode.create;
                    exit;
                  end;
               end
             { There is a widechar? }
             else if is_widechar(rd) or is_widechar(ld) then
               begin
                 { widechar+widechar gives unicodestring }
                 if nodetype=addn then
                   begin
                     inserttypeconv(left,cunicodestringtype);
                     if (torddef(rd).ordtype<>uwidechar) then
                       inserttypeconv(right,cwidechartype);
                     resultdef:=cunicodestringtype;
                   end
                 else
                   begin
                     if (torddef(ld).ordtype<>uwidechar) then
                       inserttypeconv(left,cwidechartype);
                     if (torddef(rd).ordtype<>uwidechar) then
                       inserttypeconv(right,cwidechartype);
                   end;
               end
             { is there a currency type ? }
             else if ((torddef(rd).ordtype=scurrency) or (torddef(ld).ordtype=scurrency)) then
               begin
                  if (torddef(ld).ordtype<>scurrency) then
                   inserttypeconv(left,s64currencytype);
                  if (torddef(rd).ordtype<>scurrency) then
                   inserttypeconv(right,s64currencytype);
               end
             { leave some constant integer expressions alone in case the
               resultdef of the integer types doesn't influence the outcome,
               because the forced type conversions below can otherwise result
               in unexpected results (such as high(qword)<high(int64) returning
               true because high(qword) gets converted to int64) }
             else if is_integer(ld) and is_integer(rd) and
                     (lt=ordconstn) and (rt=ordconstn) and
                     (nodetype in [equaln,unequaln,gtn,gten,ltn,lten]) then
               begin
               end
             { "and" does't care about the sign of integers }
             { "xor", "or" and compares don't need extension to native int }
             { size either as long as both values are signed or unsigned   }
             { "xor" and "or" also don't care about the sign if the values }
             { occupy an entire register                                   }
             { don't do it if either type is 64 bit (except for "and"),    }
             { since in that case we can't safely find a "common" type     }
             else if is_integer(ld) and is_integer(rd) and
                     ((nodetype=andn) or
                      ((nodetype in [orn,xorn,equaln,unequaln,gtn,gten,ltn,lten]) and
                        not is_64bitint(ld) and not is_64bitint(rd) and
                       (is_signed(ld)=is_signed(rd)))) then
               begin
                 { Delphi-compatible: prefer unsigned type for "and", when the
                   unsigned type is bigger than the signed one, and also bigger
                   than min(native_int, 32-bit) }
                 if (is_oversizedint(rd) or is_nativeint(rd) or is_32bitint(rd)) and
                    (rd.size>=ld.size) and
                    not is_signed(rd) and is_signed(ld) then
                      inserttypeconv_internal(left,rd)
                 else if (is_oversizedint(ld) or is_nativeint(ld) or is_32bitint(ld)) and
                    (ld.size>=rd.size) and
                    not is_signed(ld) and is_signed(rd) then
                      inserttypeconv_internal(right,ld)
                 else
                   begin
                     { not to left right.resultdef, because that may
                       cause a range error if left and right's def don't
                       completely overlap }
                     nd:=get_common_intdef(torddef(ld),torddef(rd),true);
                     inserttypeconv(left,nd);
                     inserttypeconv(right,nd);
                   end;
               end
             { don't extend (sign-mismatched) comparisons if either side is a constant
               whose value is within range of opposite side }
             else if is_integer(ld) and is_integer(rd) and
                     (nodetype in [equaln,unequaln,gtn,gten,ltn,lten]) and
                     (is_signed(ld)<>is_signed(rd)) and
                     (
                       ((lt=ordconstn) and maybe_cast_ordconst(left,rd)) or
                       ((rt=ordconstn) and maybe_cast_ordconst(right,ld))
                     ) then
               begin
                 { done here }
               end
             { is there a signed 64 bit type ? }
             else if ((torddef(rd).ordtype=s64bit) or (torddef(ld).ordtype=s64bit)) then
               begin
                  if (torddef(ld).ordtype<>s64bit) then
                   inserttypeconv(left,s64inttype);
                  if (torddef(rd).ordtype<>s64bit) then
                   inserttypeconv(right,s64inttype);
               end
             { is there a unsigned 64 bit type ? }
             else if ((torddef(rd).ordtype=u64bit) or (torddef(ld).ordtype=u64bit)) then
               begin
                  if (torddef(ld).ordtype<>u64bit) then
                   inserttypeconv(left,u64inttype);
                  if (torddef(rd).ordtype<>u64bit) then
                   inserttypeconv(right,u64inttype);
               end
             { is there a larger int? }
             else if is_oversizedint(rd) or is_oversizedint(ld) then
               begin
                 nd:=get_common_intdef(torddef(ld),torddef(rd),false);
                 inserttypeconv(right,nd);
                 inserttypeconv(left,nd);
               end
             { is there a native unsigned int? }
             else if is_nativeuint(rd) or is_nativeuint(ld) then
               begin
                 { convert positive constants to uinttype }
                 if (not is_nativeuint(ld)) and
                    is_constintnode(left) and
                    (tordconstnode(left).value >= 0) then
                   inserttypeconv(left,uinttype);
                 if (not is_nativeuint(rd)) and
                    is_constintnode(right) and
                    (tordconstnode(right).value >= 0) then
                   inserttypeconv(right,uinttype);
                 { when one of the operand is signed or the operation is subn then perform
                   the operation in a larger signed type, can't use rd/ld here because there
                   could be already typeconvs inserted.
                   This is compatible with the code below for other unsigned types (PFV) }
                 if is_signed(left.resultdef) or
                    is_signed(right.resultdef) or
                    (nodetype=subn) then
                   begin
                     if nodetype<>subn then
                       CGMessage(type_h_mixed_signed_unsigned);
                     { mark as internal in case added for a subn, so }
                     { ttypeconvnode.simplify can remove the larger  }
                     { typecast again if semantically correct. Even  }
                     { if we could detect that here already, we      }
                     { mustn't do it here because that would change  }
                     { overload choosing behaviour etc. The code in  }
                     { ncnv.pas is run after that is already decided }
                     if (not is_signed(left.resultdef) and
                         not is_signed(right.resultdef)) or
                        (nodetype in [orn,xorn]) then
                       include(flags,nf_internal);
                     { get next larger signed int type }
                     nd:=get_common_intdef(torddef(sinttype),torddef(uinttype),false);
                     inserttypeconv(left,nd);
                     inserttypeconv(right,nd);
                   end
                 else
                   begin
                     if not is_nativeuint(left.resultdef) then
                       inserttypeconv(left,uinttype);
                     if not is_nativeuint(right.resultdef) then
                       inserttypeconv(right,uinttype);
                   end;
               end
             { generic ord conversion is sinttype }
             else
               begin
                 { When there is a signed type or there is a minus operation
                   we convert to signed int. Otherwise (both are unsigned) we keep
                   the result also unsigned. This is compatible with Delphi (PFV) }
                 if is_signed(ld) or
                    is_signed(rd) or
                    (nodetype=subn) then
                   begin
                     inserttypeconv(right,sinttype);
                     inserttypeconv(left,sinttype);
                   end
                 else
                   begin
                     inserttypeconv(right,uinttype);
                     inserttypeconv(left,uinttype);
                   end;
               end;
           end

         { if both are floatdefs, conversion is already done before constant folding }
         else if (ld.typ=floatdef) then
           begin
             if not(nodetype in [addn,subn,muln,slashn,equaln,unequaln,ltn,lten,gtn,gten]) then
               CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
           end

         { left side a setdef, must be before string processing,
           else array constructor can be seen as array of char (PFV) }
         else if (ld.typ=setdef) then
          begin
             if not(nodetype in [addn,subn,symdifn,muln,equaln,unequaln,lten,gten]) then
              CGMessage(type_e_set_operation_unknown);
             { right must either be a set or a set element }
             if (rd.typ<>setdef) and
                (rt<>setelementn) then
               CGMessage(type_e_mismatch)
             { Make operands the same setdef. If one's elementtype fits   }
             { entirely inside the other's, pick the one with the largest }
             { range.  Otherwise create a new setdef with a range which   }
             { can contain both.                                          }
             else if not(equal_defs(ld,rd)) then
              begin
                { note: ld cannot be an empty set with elementdef=nil in }
                { case right is not a set, arrayconstructor_to_set takes }
                { care of that                                           }

                { 1: rd is a set with an assigned elementdef, and ld is    }
                {    either an empty set without elementdef or a set whose }
                {    elementdef fits in rd's elementdef -> convert to rd   }
                if ((rd.typ=setdef) and
                    assigned(tsetdef(rd).elementdef) and
                    (not assigned(tsetdef(ld).elementdef) or
                     is_in_limit(ld,rd))) then
                  inserttypeconv(left,rd)
                { 2: rd is either an empty set without elementdef or a set }
                {    whose elementdef fits in ld's elementdef, or a set    }
                {    element whose def fits in ld's elementdef -> convert  }
                {    to ld. ld's elementdef can't be nil here, is caught   }
                {    previous case and "note:" above                       }
                else if ((rd.typ=setdef) and
                         (not assigned(tsetdef(rd).elementdef) or
                          is_in_limit(rd,ld))) or
                        ((rd.typ<>setdef) and
                         is_in_limit(rd,tsetdef(ld).elementdef)) then
                   if (rd.typ=setdef) then
                    inserttypeconv(right,ld)
                  else
                    inserttypeconv(right,tsetdef(ld).elementdef)
                { 3: otherwise create setdef which encompasses both, taking }
                {    into account empty sets without elementdef             }
                else
                  begin
                    if assigned(tsetdef(ld).elementdef) then
                      begin
                        llow:=tsetdef(ld).setbase;
                        lhigh:=tsetdef(ld).setmax;
                      end;
                    if (rd.typ=setdef) then
                      if assigned(tsetdef(rd).elementdef) then
                        begin
                          rlow:=tsetdef(rd).setbase;
                          rhigh:=tsetdef(rd).setmax;
                        end
                      else
                        begin
                          { ld's elementdef must have been valid }
                          rlow:=llow;
                          rhigh:=lhigh;
                        end
                    else
                      getrange(rd,rlow,rhigh);
                    if not assigned(tsetdef(ld).elementdef) then
                      begin
                        llow:=rlow;
                        lhigh:=rhigh;
                      end;
                    nd:=csetdef.create(tsetdef(ld).elementdef,min(llow,rlow).svalue,max(lhigh,rhigh).svalue,true);
                    inserttypeconv(left,nd);
                    if (rd.typ=setdef) then
                      inserttypeconv(right,nd)
                    else
                       inserttypeconv(right,tsetdef(nd).elementdef);
                  end;
              end;
          end
         { pointer comparision and subtraction }
         else if (
                  (rd.typ=pointerdef) and (ld.typ=pointerdef)
                 ) or
                 { compare/add pchar to variable (not stringconst) char arrays
                   by addresses like BP/Delphi }
                 (
                  (nodetype in [equaln,unequaln,subn,addn]) and
                  (
                   ((is_pchar(ld) or (lt=niln)) and is_chararray(rd) and (rt<>stringconstn)) or
                   ((is_pchar(rd) or (rt=niln)) and is_chararray(ld) and (lt<>stringconstn))
                  )
                 ) then
          begin
            { convert char array to pointer }
            if is_chararray(rd) then
              begin
                inserttypeconv(right,charpointertype);
                rd:=right.resultdef;
              end
            else if is_chararray(ld) then
              begin
                inserttypeconv(left,charpointertype);
                ld:=left.resultdef;
              end;

            case nodetype of
               equaln,unequaln :
                 begin
                    if is_voidpointer(right.resultdef) then
                      inserttypeconv(right,left.resultdef)
                    else if is_voidpointer(left.resultdef) then
                      inserttypeconv(left,right.resultdef)
                    else if not(equal_defs(ld,rd)) then
                      IncompatibleTypes(ld,rd);
                    { now that the type checking is done, convert both to charpointer, }
                    { because methodpointers are 8 bytes even though only the first 4  }
                    { bytes must be compared. This can happen here if we are in        }
                    { TP/Delphi mode, because there @methodpointer = voidpointer (but  }
                    { a voidpointer of 8 bytes). A conversion to voidpointer would be  }
                    { optimized away, since the result already was a voidpointer, so   }
                    { use a charpointer instead (JM)                                   }
{$if defined(jvm)}
                    inserttypeconv_internal(left,java_jlobject);
                    inserttypeconv_internal(right,java_jlobject);
{$elseif defined(i8086)}
                    if is_hugepointer(left.resultdef) then
                      inserttypeconv_internal(left,charhugepointertype)
                    else if is_farpointer(left.resultdef) then
                      inserttypeconv_internal(left,charfarpointertype)
                    else
                      inserttypeconv_internal(left,charnearpointertype);
                    if is_hugepointer(right.resultdef) then
                      inserttypeconv_internal(right,charhugepointertype)
                    else if is_farpointer(right.resultdef) then
                      inserttypeconv_internal(right,charfarpointertype)
                    else
                      inserttypeconv_internal(right,charnearpointertype);
{$else}
                    inserttypeconv_internal(left,charpointertype);
                    inserttypeconv_internal(right,charpointertype);
{$endif jvm}
                 end;
               ltn,lten,gtn,gten:
                 begin
                    if (cs_extsyntax in current_settings.moduleswitches) or
                       (nf_internal in flags) then
                     begin
                       if is_voidpointer(right.resultdef) then
                        inserttypeconv(right,left.resultdef)
                       else if is_voidpointer(left.resultdef) then
                        inserttypeconv(left,right.resultdef)
                       else if not(equal_defs(ld,rd)) then
                        IncompatibleTypes(ld,rd);
                     end
                    else
                     CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
                 end;
               subn:
                 begin
                    if (cs_extsyntax in current_settings.moduleswitches) or
                       (nf_internal in flags) then
                      begin
                        if is_voidpointer(right.resultdef) then
                        begin
                          if is_big_untyped_addrnode(right) then
                            CGMessage1(type_w_untyped_arithmetic_unportable,node2opstr(nodetype));
                          inserttypeconv(right,left.resultdef)
                        end
                        else if is_voidpointer(left.resultdef) then
                          inserttypeconv(left,right.resultdef)
                        else if not(equal_defs(ld,rd)) then
                          IncompatibleTypes(ld,rd);
                      end
                    else
                      CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);

                    if not(nf_has_pointerdiv in flags) and
                      (tpointerdef(rd).pointeddef.size>1) then
                      begin
                        hp:=getcopy;
                        include(hp.flags,nf_has_pointerdiv);
                        result:=cmoddivnode.create(divn,hp,
                          cordconstnode.create(tpointerdef(rd).pointeddef.size,tpointerdef(rd).pointer_subtraction_result_type,false));
                      end;
                    resultdef:=tpointerdef(rd).pointer_subtraction_result_type;
                    exit;
                 end;
               else
                 CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
            end;
          end

         { is one of the operands a string?,
           chararrays are also handled as strings (after conversion), also take
           care of chararray+chararray and chararray+char.
           Note: Must be done after pointerdef+pointerdef has been checked, else
           pchar is converted to string }
         else if (rd.typ=stringdef) or
                 (ld.typ=stringdef) or
                 { stringconstn's can be arraydefs }
                 (lt=stringconstn) or
                 (rt=stringconstn) or
                 ((is_pchar(rd) or is_chararray(rd) or is_char(rd) or is_open_chararray(rd) or
                   is_pwidechar(rd) or is_widechararray(rd) or is_widechar(rd) or is_open_widechararray(rd)) and
                  (is_pchar(ld) or is_chararray(ld) or is_char(ld) or is_open_chararray(ld) or
                   is_pwidechar(ld) or is_widechararray(ld) or is_widechar(ld) or is_open_widechararray(ld))) then
          begin
            if (nodetype in [addn,equaln,unequaln,lten,gten,ltn,gtn]) then
              begin
                { Is there a unicodestring? }
                if is_unicodestring(rd) or is_unicodestring(ld) or
                   ((m_default_unicodestring in current_settings.modeswitches) and
                    (cs_refcountedstrings in current_settings.localswitches) and
                    (
                     is_pwidechar(rd) or is_widechararray(rd) or is_open_widechararray(rd) or (lt = stringconstn) or
                     is_pwidechar(ld) or is_widechararray(ld) or is_open_widechararray(ld) or (rt = stringconstn)
                    )
                   ) then
                  strtype:=st_unicodestring
                else
                { Is there a widestring? }
                  if is_widestring(rd) or is_widestring(ld) or
                     is_pwidechar(rd) or is_widechararray(rd) or is_widechar(rd) or is_open_widechararray(rd) or
                     is_pwidechar(ld) or is_widechararray(ld) or is_widechar(ld) or is_open_widechararray(ld) then
                    strtype:=st_widestring
                else
                  if is_ansistring(rd) or is_ansistring(ld) or
                     ((cs_refcountedstrings in current_settings.localswitches) and
                     //todo: Move some of this to longstring's then they are implemented?
                      (
                       is_pchar(rd) or (is_chararray(rd) and (rd.size > 255)) or is_open_chararray(rd) or (lt = stringconstn) or
                       is_pchar(ld) or (is_chararray(ld) and (ld.size > 255)) or is_open_chararray(ld) or (rt = stringconstn)
                      )
                     ) then
                    strtype:=st_ansistring
                else
                  if is_longstring(rd) or is_longstring(ld) then
                    strtype:=st_longstring
                else
                  begin
                    { TODO: todo: add a warning/hint here if one converting a too large array}
                    { nodes is PChar, array [with size > 255] or OpenArrayOfChar.
                      Note: Delphi halts with error if "array [0..xx] of char"
                           is assigned to ShortString and string length is less
                           then array size }
                    strtype:= st_shortstring;
                  end;

                // Now convert nodes to common string type
                case strtype of
                  st_widestring :
                    begin
                      if not(is_widestring(rd)) then
                        inserttypeconv(right,cwidestringtype);
                      if not(is_widestring(ld)) then
                        inserttypeconv(left,cwidestringtype);
                    end;
                  st_unicodestring :
                    begin
                      if not(is_unicodestring(rd)) then
                        inserttypeconv(right,cunicodestringtype);
                      if not(is_unicodestring(ld)) then
                        inserttypeconv(left,cunicodestringtype);
                    end;
                  st_ansistring :
                    begin
                      { use same code page if possible (don't force same code
                        page in case both are ansistrings with code page <>
                        CP_NONE, since then data loss can occur: the ansistring
                        helpers will convert them at run time to an encoding
                        that can represent both encodings) }
                      if is_ansistring(ld) and
                         (tstringdef(ld).encoding<>0) and
                         (tstringdef(ld).encoding<>globals.CP_NONE) and
                         (not is_ansistring(rd) or
                          (tstringdef(rd).encoding=0) or
                          (tstringdef(rd).encoding=globals.CP_NONE)) then
                        inserttypeconv(right,ld)
                      else if is_ansistring(rd) and
                         (tstringdef(rd).encoding<>0) and
                         (tstringdef(rd).encoding<>globals.CP_NONE) and
                         (not is_ansistring(ld) or
                          (tstringdef(ld).encoding=0) or
                          (tstringdef(ld).encoding=globals.CP_NONE)) then
                        inserttypeconv(left,rd)
                      else
                        begin
                          if not is_ansistring(ld) then
                            inserttypeconv(left,getansistringdef);
                          if not is_ansistring(rd) then
                            inserttypeconv(right,getansistringdef);
                        end;
                    end;
                  st_longstring :
                    begin
                      if not(is_longstring(rd)) then
                        inserttypeconv(right,clongstringtype);
                      if not(is_longstring(ld)) then
                        inserttypeconv(left,clongstringtype);
                     end;
                   st_shortstring :
                     begin
                       if not(is_shortstring(ld)) then
                         inserttypeconv(left,cshortstringtype);
                       { don't convert char, that can be handled by the optimized node }
                       if not(is_shortstring(rd) or is_char(rd)) then
                         inserttypeconv(right,cshortstringtype);
                     end;
                end;
              end
            else
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
          end

         { implicit pointer object type comparison }
         else if is_implicit_pointer_object_type(rd) or is_implicit_pointer_object_type(ld) then
          begin
            if (nodetype in [equaln,unequaln]) then
              begin
                if is_implicit_pointer_object_type(rd) and is_implicit_pointer_object_type(ld) then
                 begin
                   if def_is_related(tobjectdef(rd),tobjectdef(ld)) then
                    inserttypeconv(right,left.resultdef)
                   else
                    inserttypeconv(left,right.resultdef);
                 end
                else if is_implicit_pointer_object_type(rd) then
                  inserttypeconv(left,right.resultdef)
                else
                  inserttypeconv(right,left.resultdef);
              end
            else
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
          end

         else if (rd.typ=classrefdef) and (ld.typ=classrefdef) then
          begin
            if (nodetype in [equaln,unequaln]) then
              begin
                if def_is_related(tobjectdef(tclassrefdef(rd).pointeddef),
                        tobjectdef(tclassrefdef(ld).pointeddef)) then
                  inserttypeconv(right,left.resultdef)
                else
                  inserttypeconv(left,right.resultdef);
              end
            else
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
          end

         { allow comparison with nil pointer }
         else if is_implicit_pointer_object_type(rd) or (rd.typ=classrefdef) then
          begin
            if (nodetype in [equaln,unequaln]) then
              inserttypeconv(left,right.resultdef)
            else
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
          end

         else if is_implicit_pointer_object_type(ld) or (ld.typ=classrefdef) then
          begin
            if (nodetype in [equaln,unequaln]) then
              inserttypeconv(right,left.resultdef)
            else
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
          end

       { support procvar=nil,procvar<>nil }
         else if ((ld.typ=procvardef) and (rt=niln)) or
                 ((rd.typ=procvardef) and (lt=niln)) then
          begin
            if not(nodetype in [equaln,unequaln]) then
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
            { find proc field in methodpointer record }
            hsym:=tfieldvarsym(trecorddef(methodpointertype).symtable.Find('proc'));
            if not assigned(hsym) then
              internalerror(200412043);
            { For methodpointers compare only tmethodpointer.proc }
            if (rd.typ=procvardef) and
               (not tprocvardef(rd).is_addressonly) then
              begin
                right:=csubscriptnode.create(
                           hsym,
                           ctypeconvnode.create_internal(right,methodpointertype));
                typecheckpass(right);
               end;
            if (ld.typ=procvardef) and
               (not tprocvardef(ld).is_addressonly) then
              begin
                left:=csubscriptnode.create(
                          hsym,
                          ctypeconvnode.create_internal(left,methodpointertype));
                typecheckpass(left);
              end;
            if lt=niln then
              inserttypeconv_explicit(left,right.resultdef)
            else
              inserttypeconv_explicit(right,left.resultdef)
          end

         { <dyn. array>+<dyn. array> ? }
         else if (nodetype=addn) and (is_dynamic_array(ld) or is_dynamic_array(rd)) then
           begin
              result:=maybe_convert_to_insert;
              if assigned(result) then
                exit;
              if not(is_dynamic_array(ld)) then
                inserttypeconv(left,rd);
              if not(is_dynamic_array(rd)) then
                inserttypeconv(right,ld);
           end

        { support dynamicarray=nil,dynamicarray<>nil }
         else if (is_dynamic_array(ld) and (rt=niln)) or
                 (is_dynamic_array(rd) and (lt=niln)) or
                 (is_dynamic_array(ld) and is_dynamic_array(rd)) then
          begin
            if not(nodetype in [equaln,unequaln]) then
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
            if lt=niln then
              inserttypeconv_explicit(left,right.resultdef)
            else
              inserttypeconv_explicit(right,left.resultdef)
          end

{$ifdef SUPPORT_MMX}
       { mmx support, this must be before the zero based array
         check }
         else if (cs_mmx in current_settings.localswitches) and
                 is_mmx_able_array(ld) and
                 is_mmx_able_array(rd) and
                 equal_defs(ld,rd) then
            begin
              case nodetype of
                addn,subn,xorn,orn,andn:
                  ;
                { mul is a little bit restricted }
                muln:
                  if not(mmx_type(ld) in [mmxu16bit,mmxs16bit,mmxfixed16]) then
                    CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
                else
                  CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
              end;
            end
{$endif SUPPORT_MMX}
         { vector support, this must be before the zero based array
           check }
         else if (cs_support_vectors in current_settings.globalswitches) and
                 is_vector(ld) and
                 is_vector(rd) and
                 equal_defs(ld,rd) then
            begin
              if not(nodetype in [addn,subn,xorn,orn,andn,muln,slashn]) then
                CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
              { both defs must be equal, so taking left or right as resultdef doesn't matter }
              resultdef:=left.resultdef;
            end

         { this is a little bit dangerous, also the left type }
         { pointer to should be checked! This broke the mmx support      }
         else if (rd.typ=pointerdef) or
                 (is_zero_based_array(rd) and (rt<>stringconstn)) then
          begin
            if is_zero_based_array(rd) then
              begin
                resultdef:=cpointerdef.getreusable(tarraydef(rd).elementdef);
                inserttypeconv(right,resultdef);
              end
            else
              resultdef:=right.resultdef;
            inserttypeconv(left,tpointerdef(right.resultdef).pointer_arithmetic_int_type);
            if nodetype=addn then
              begin
                if (rt=niln) then
                  CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,'NIL');
                if (not(cs_extsyntax in current_settings.moduleswitches) and not(nf_internal in flags))  or
                   (not (is_pchar(ld) or is_chararray(ld) or is_open_chararray(ld) or is_widechar(ld) or is_widechararray(ld) or is_open_widechararray(ld)) and
                    not(cs_pointermath in current_settings.localswitches) and
                    not((ld.typ=pointerdef) and tpointerdef(ld).has_pointer_math)) then
                  CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
                if (rd.typ=pointerdef) and
                   (tpointerdef(rd).pointeddef.size>1) then
                   begin
                     left:=caddnode.create(muln,left,
                       cordconstnode.create(tpointerdef(rd).pointeddef.size,tpointerdef(right.resultdef).pointer_arithmetic_int_type,true));
                     typecheckpass(left);
                   end;
              end
            else
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
          end

         else if (ld.typ=pointerdef) or
                 (is_zero_based_array(ld) and (lt<>stringconstn)) then
           begin
             if is_zero_based_array(ld) then
               begin
                  resultdef:=cpointerdef.getreusable(tarraydef(ld).elementdef);
                  inserttypeconv(left,resultdef);
               end
             else
               resultdef:=left.resultdef;

             inserttypeconv(right,tpointerdef(left.resultdef).pointer_arithmetic_int_type);
             if nodetype in [addn,subn] then
               begin
                 if (lt=niln) then
                   CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),'NIL',rd.typename);
                 if (not(cs_extsyntax in current_settings.moduleswitches) and not(nf_internal in flags)) or
                   (not (is_pchar(ld) or is_chararray(ld) or is_open_chararray(ld) or is_widechar(ld) or is_widechararray(ld) or is_open_widechararray(ld)) and
                    not(cs_pointermath in current_settings.localswitches) and
                    not((ld.typ=pointerdef) and tpointerdef(ld).has_pointer_math)) then
                   CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
                 if (ld.typ=pointerdef) then
                 begin
                   if is_big_untyped_addrnode(left) then
                     CGMessage1(type_w_untyped_arithmetic_unportable,node2opstr(nodetype));
                   if (tpointerdef(ld).pointeddef.size>1) then
                   begin
                     right:=caddnode.create(muln,right,
                       cordconstnode.create(tpointerdef(ld).pointeddef.size,tpointerdef(left.resultdef).pointer_arithmetic_int_type,true));
                     typecheckpass(right);
                   end
                 end else
                   if is_zero_based_array(ld) and
                      (tarraydef(ld).elementdef.size>1) then
                     begin
                       right:=caddnode.create(muln,right,
                         cordconstnode.create(tarraydef(ld).elementdef.size,tpointerdef(left.resultdef).pointer_arithmetic_int_type,true));
                       typecheckpass(right);
                     end;
               end
             else
               CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
           end

         else if (rd.typ=procvardef) and
                 (ld.typ=procvardef) and
                 equal_defs(rd,ld) then
          begin
            if (nodetype in [equaln,unequaln]) then
              begin
                if tprocvardef(rd).is_addressonly then
                  begin
                    inserttypeconv_internal(right,voidcodepointertype);
                    inserttypeconv_internal(left,voidcodepointertype);
                  end
                else
                  begin
                    { find proc field in methodpointer record }
                    hsym:=tfieldvarsym(trecorddef(methodpointertype).symtable.Find('proc'));
                    if not assigned(hsym) then
                      internalerror(200412043);
                    { Compare tmehodpointer(left).proc }
                    right:=csubscriptnode.create(
                                 hsym,
                                 ctypeconvnode.create_internal(right,methodpointertype));
                    typecheckpass(right);
                    left:=csubscriptnode.create(
                                 hsym,
                                 ctypeconvnode.create_internal(left,methodpointertype));
                     typecheckpass(left);
                  end;
              end
            else
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
          end

         { enums }
         else if (ld.typ=enumdef) and (rd.typ=enumdef) then
          begin
            if allowenumop(nodetype) or (nf_internal in flags) then
              inserttypeconv(right,left.resultdef)
            else
              CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),ld.typename,rd.typename);
          end

         { generic conversion, this is for error recovery }
         else
          begin
            inserttypeconv(left,sinttype);
            inserttypeconv(right,sinttype);
          end;

         if cmp_of_disjunct_ranges(res) and not(nf_internal in flags) then
           begin
             if res then
               CGMessage(type_w_comparison_always_true)
             else
               CGMessage(type_w_comparison_always_false);
           end;

         { set resultdef if not already done }
         if not assigned(resultdef) then
          begin
             case nodetype of
                ltn,lten,gtn,gten,equaln,unequaln :
                  resultdef:=pasbool1type;
                slashn :
                  resultdef:=resultrealdef;
                addn:
                  begin
                    { for strings, return is always a 255 char string }
                    if is_shortstring(left.resultdef) then
                      resultdef:=cshortstringtype
                    else
                    { for ansistrings set resultdef to assignment left node
                      if it is an assignment and left node expects ansistring }
                    if is_ansistring(left.resultdef) and
                       assigned(aktassignmentnode) and
                       (aktassignmentnode.right=self) and
                       is_ansistring(aktassignmentnode.left.resultdef) then
                      resultdef:=aktassignmentnode.left.resultdef
                    else
                      resultdef:=left.resultdef;
                  end;
                else
                  resultdef:=left.resultdef;
             end;
          end;

         { when the result is currency we need some extra code for
           multiplication and division. this should not be done when
           the muln or slashn node is created internally }
         if not(nf_is_currency in flags) and
            is_currency(resultdef) then
          begin
            case nodetype of
              slashn :
                begin
                  { slashn will only work with floats }
                  hp:=caddnode.create(muln,getcopy,crealconstnode.create(10000.0,s64currencytype));
                  include(hp.flags,nf_is_currency);
                  result:=hp;
                end;
              muln :
                begin
                  hp:=nil;
                  if s64currencytype.typ=floatdef then
                    begin
                      { if left is a currency integer constant, we can get rid of the factor 10000 }
                      { int64(...) causes a cast on currency, so it is the currency value multiplied by 10000 }
                      if (left.nodetype=realconstn) and (is_currency(left.resultdef)) and (not(nf_is_currency in left.flags)) and ((trunc(trealconstnode(left).value_real) mod 10000)=0) then
                        begin
                          { trealconstnode expects that value_real and value_currency contain valid values }
{$ifdef FPC_CURRENCY_IS_INT64}
                          trealconstnode(left).value_currency:=pint64(@(trealconstnode(left).value_currency))^ div 10000;
{$else}
                          trealconstnode(left).value_currency:=trealconstnode(left).value_currency / 10000;
{$endif}
                          trealconstnode(left).value_real:=trealconstnode(left).value_real/10000;
                        end
                      { or if right is an integer constant, we can get rid of its factor 10000 }
                      else if (right.nodetype=realconstn) and (is_currency(right.resultdef)) and (not(nf_is_currency in right.flags)) and ((trunc(trealconstnode(right).value_real) mod 10000)=0) then
                        begin
                          { trealconstnode expects that value and value_currency contain valid values }
{$ifdef FPC_CURRENCY_IS_INT64}
                          trealconstnode(right).value_currency:=pint64(@(trealconstnode(right).value_currency))^ div 10000;
{$else}
                          trealconstnode(right).value_currency:=trealconstnode(right).value_currency / 10000;
{$endif}
                          trealconstnode(right).value_real:=trealconstnode(right).value_real/10000;
                        end
                      else
                        begin
                          hp:=caddnode.create(slashn,getcopy,crealconstnode.create(10000.0,s64currencytype));
                          include(hp.flags,nf_is_currency);
                        end;
                    end
                  else
                    begin
{$ifndef VER3_0}
                      { if left is a currency integer constant, we can get rid of the factor 10000 }
                      if (left.nodetype=ordconstn) and (is_currency(left.resultdef)) and ((tordconstnode(left).value mod 10000)=0) then
                        tordconstnode(left).value:=tordconstnode(left).value div 10000
                      { or if right is an integer constant, we can get rid of its factor 10000 }
                      else if (right.nodetype=ordconstn) and (is_currency(right.resultdef)) and ((tordconstnode(right).value mod 10000)=0) then
                        tordconstnode(right).value:=tordconstnode(right).value div 10000
                      else
{$endif VER3_0}
                      if (right.nodetype=muln) and is_currency(right.resultdef) and
                        { do not test swapped here as the internal conversions are only create as "var."*"10000" }
                        is_currency(taddnode(right).right.resultdef)  and (taddnode(right).right.nodetype=ordconstn) and (tordconstnode(taddnode(right).right).value=10000) and
                        is_currency(taddnode(right).left.resultdef) and (taddnode(right).left.nodetype=typeconvn) then
                        begin
                          hp:=taddnode(right).left.getcopy;
                          include(hp.flags,nf_is_currency);
                          right.free;
                          right:=hp;
                          hp:=nil;
                        end
                      else if (left.nodetype=muln) and is_currency(left.resultdef) and
                        { do not test swapped here as the internal conversions are only create as "var."*"10000" }
                        is_currency(taddnode(left).right.resultdef)  and (taddnode(left).right.nodetype=ordconstn) and (tordconstnode(taddnode(left).right).value=10000) and
                        is_currency(taddnode(left).left.resultdef) and (taddnode(left).left.nodetype=typeconvn) then
                        begin
                          hp:=taddnode(left).left.getcopy;
                          include(hp.flags,nf_is_currency);
                          left.free;
                          left:=hp;
                          hp:=nil;
                        end
                      else
                        begin
                          hp:=cmoddivnode.create(divn,getcopy,cordconstnode.create(10000,s64currencytype,false));
                          include(hp.flags,nf_is_currency);
                        end
                    end;

                  result:=hp
                end;
              else
                ;
            end;
          end;

         if not codegenerror and
            not assigned(result) then
           result:=simplify(false);
      end;


    function taddnode.first_addstring: tnode;
      const
        swap_relation: array [ltn..unequaln] of Tnodetype=(gtn, gten, ltn, lten, equaln, unequaln);
      var
        p: tnode;
        newstatement : tstatementnode;
        tempnode (*,tempnode2*) : ttempcreatenode;
        cmpfuncname: string;
        para: tcallparanode;
      begin
        result:=nil;
        { when we get here, we are sure that both the left and the right }
        { node are both strings of the same stringtype (JM)              }
        case nodetype of
          addn:
            begin
              if (left.nodetype=stringconstn) and (tstringconstnode(left).len=0) then
                begin
                  result:=right;
                  left.free;
                  left:=nil;
                  right:=nil;
                  exit;
                end;
              if (right.nodetype=stringconstn) and (tstringconstnode(right).len=0) then
                begin
                  result:=left;
                  left:=nil;
                  right.free;
                  right:=nil;
                  exit;
                end;
              { create the call to the concat routine both strings as arguments }
              if assigned(aktassignmentnode) and
                  (aktassignmentnode.right=self) and
                  (aktassignmentnode.left.resultdef=resultdef) and
                  valid_for_var(aktassignmentnode.left,false) then
                begin
                  para:=ccallparanode.create(
                          right,
                          ccallparanode.create(
                            left,
                            ccallparanode.create(aktassignmentnode.left.getcopy,nil)
                          )
                        );
                  if is_ansistring(resultdef) then
                    para:=ccallparanode.create(
                            cordconstnode.create(
                              { don't use getparaencoding(), we have to know
                                when the result is rawbytestring }
                              tstringdef(resultdef).encoding,
                              u16inttype,
                              true
                            ),
                            para
                          );
                  result:=ccallnode.createintern(
                            'fpc_'+tstringdef(resultdef).stringtypname+'_concat',
                            para
                          );
                  include(aktassignmentnode.flags,nf_assign_done_in_right);
                  firstpass(result);
                end
              else
                begin
                  result:=internalstatements(newstatement);
                  tempnode:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);
                  addstatement(newstatement,tempnode);
                  { initialize the temp, since it will be passed to a
                    var-parameter (and finalization, which is performed by the
                    ttempcreate node and which takes care of the initialization
                    on native targets, is a noop on managed VM targets) }
                  if (target_info.system in systems_managed_vm) and
                     is_managed_type(resultdef) then
                    addstatement(newstatement,cinlinenode.create(in_setlength_x,
                      false,
                      ccallparanode.create(genintconstnode(0),
                        ccallparanode.create(ctemprefnode.create(tempnode),nil))));
                  para:=ccallparanode.create(
                          right,
                          ccallparanode.create(
                            left,
                            ccallparanode.create(ctemprefnode.create(tempnode),nil)
                          )
                        );
                  if is_ansistring(resultdef) then
                    para:=ccallparanode.create(
                            cordconstnode.create(
                              { don't use getparaencoding(), we have to know
                                when the result is rawbytestring }
                              tstringdef(resultdef).encoding,
                              u16inttype,
                              true
                            ),
                            para
                          );
                  addstatement(
                    newstatement,
                    ccallnode.createintern(
                      'fpc_'+tstringdef(resultdef).stringtypname+'_concat',
                      para
                    )
                  );
                  addstatement(newstatement,ctempdeletenode.create_normal_temp(tempnode));
                  addstatement(newstatement,ctemprefnode.create(tempnode));
                end;
              { we reused the arguments }
              left := nil;
              right := nil;
            end;
          ltn,lten,gtn,gten,equaln,unequaln :
            begin
              { generate better code for comparison with empty string, we
                only need to compare the length with 0 }
              if (nodetype in [equaln,unequaln,gtn,gten,ltn,lten]) and
                { windows widestrings are too complicated to be handled optimized }
                not(is_widestring(left.resultdef) and (target_info.system in systems_windows)) and
                 (((left.nodetype=stringconstn) and (tstringconstnode(left).len=0)) or
                  ((right.nodetype=stringconstn) and (tstringconstnode(right).len=0))) then
                begin
                  { switch so that the constant is always on the right }
                  if left.nodetype = stringconstn then
                    begin
                      p := left;
                      left := right;
                      right := p;
                      nodetype:=swap_relation[nodetype];
                    end;
                  if is_shortstring(left.resultdef) or
                     (nodetype in [gtn,gten,ltn,lten]) or
                     (target_info.system in systems_managed_vm) then
                    { compare the length with 0 }
                    result := caddnode.create(nodetype,
                      cinlinenode.create(in_length_x,false,left),
                      cordconstnode.create(0,s8inttype,false))
                  else
                    begin
                      (*
                      if is_widestring(left.resultdef) and
                        (target_info.system in system_windows) then
                        begin
                          { windows like widestrings requires that we also check the length }
                          result:=internalstatements(newstatement);
                          tempnode:=ctempcreatenode.create(voidpointertype,voidpointertype.size,tt_persistent,true);
                          tempnode2:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);
                          addstatement(newstatement,tempnode);
                          addstatement(newstatement,tempnode2);
                          { poor man's cse }
                          addstatement(newstatement,cassignmentnode.create(ctemprefnode.create(tempnode),
                            ctypeconvnode.create_internal(left,voidpointertype))
                          );
                          addstatement(newstatement,cassignmentnode.create(ctemprefnode.create(tempnode2),
                            caddnode.create(orn,
                              caddnode.create(nodetype,
                                ctemprefnode.create(tempnode),
                                cpointerconstnode.create(0,voidpointertype)
                              ),
                              caddnode.create(nodetype,
                                ctypeconvnode.create_internal(cderefnode.create(ctemprefnode.create(tempnode)),s32inttype),
                                cordconstnode.create(0,s32inttype,false)
                              )
                            )
                          ));
                          addstatement(newstatement,ctempdeletenode.create_normal_temp(tempnode));
                          addstatement(newstatement,ctempdeletenode.create_normal_temp(tempnode2));
                          addstatement(newstatement,ctemprefnode.create(tempnode2));
                        end
                      else
                      *)
                        begin
                          { compare the pointer with nil (for ansistrings etc), }
                          { faster than getting the length (JM)                 }
                          result:= caddnode.create(nodetype,
                            ctypeconvnode.create_internal(left,voidpointertype),
                            cpointerconstnode.create(0,voidpointertype));
                        end;
                    end;
                  { left is reused }
                  left := nil;
                  { right isn't }
                  right.free;
                  right := nil;
                  exit;
                end;
              { no string constant -> call compare routine }
              cmpfuncname := 'fpc_'+tstringdef(left.resultdef).stringtypname+'_compare';
              { for equality checks use optimized version }
              if nodetype in [equaln,unequaln] then
                cmpfuncname := cmpfuncname + '_equal';

              result := ccallnode.createintern(cmpfuncname,
                ccallparanode.create(right,ccallparanode.create(left,nil)));
              { and compare its result with 0 according to the original operator }
              result := caddnode.create(nodetype,result,
                cordconstnode.create(0,s8inttype,false));
              left := nil;
              right := nil;
            end;
          else
            internalerror(2019050520);
        end;
      end;


    function taddnode.first_addset : tnode;

      procedure call_varset_helper(const n : string);
        var
          newstatement : tstatementnode;
          temp    : ttempcreatenode;
        begin
          { add two var sets }
          result:=internalstatements(newstatement);

          { create temp for result }
          temp:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);
          addstatement(newstatement,temp);

          addstatement(newstatement,ccallnode.createintern(n,
            ccallparanode.create(cordconstnode.create(resultdef.size,sinttype,false),
            ccallparanode.create(ctemprefnode.create(temp),
            ccallparanode.create(right,
            ccallparanode.create(left,nil)))))
          );

          { remove reused parts from original node }
          left:=nil;
          right:=nil;
          { the last statement should return the value as
            location and type, this is done be referencing the
            temp and converting it first from a persistent temp to
            normal temp }
          addstatement(newstatement,ctempdeletenode.create_normal_temp(temp));
          addstatement(newstatement,ctemprefnode.create(temp));
        end;

      var
        procname: string[31];
        tempn: tnode;
        newstatement : tstatementnode;
        temp    : ttempcreatenode;
      begin
        result:=nil;
        case nodetype of
          equaln,unequaln,lten,gten:
            begin
              case nodetype of
                equaln,unequaln:
                  procname := 'fpc_varset_comp_sets';
                lten,gten:
                  begin
                    procname := 'fpc_varset_contains_sets';
                    { (left >= right) = (right <= left) }
                    if nodetype = gten then
                      begin
                        tempn := left;
                        left := right;
                        right := tempn;
                      end;
                    end;
                  else
                    internalerror(2013112911);
                end;
                result := ccallnode.createinternres(procname,
                  ccallparanode.create(cordconstnode.create(left.resultdef.size,sinttype,false),
                  ccallparanode.create(right,
                  ccallparanode.create(left,nil))),resultdef);
                { left and right are reused as parameters }
                left := nil;
                right := nil;
                { for an unequaln, we have to negate the result of comp_sets }
                if nodetype = unequaln then
                  result := cnotnode.create(result);
            end;
          addn:
            begin
              { optimize first loading of a set }
              if (right.nodetype=setelementn) and
                  not(assigned(tsetelementnode(right).right)) and
                  is_emptyset(left) then
                begin
                  result:=internalstatements(newstatement);

                  { create temp for result }
                  temp:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);
                  addstatement(newstatement,temp);

                  { adjust for set base }
                  tsetelementnode(right).left:=caddnode.create(subn,
                    ctypeconvnode.create_internal(tsetelementnode(right).left,sinttype),
                    cordconstnode.create(tsetdef(resultdef).setbase,sinttype,false));

                  addstatement(newstatement,ccallnode.createintern('fpc_varset_create_element',
                    ccallparanode.create(ctemprefnode.create(temp),
                    ccallparanode.create(cordconstnode.create(resultdef.size,sinttype,false),
                    ccallparanode.create(tsetelementnode(right).left,nil))))
                  );

                  { the last statement should return the value as
                    location and type, this is done be referencing the
                    temp and converting it first from a persistent temp to
                    normal temp }
                  addstatement(newstatement,ctempdeletenode.create_normal_temp(temp));
                  addstatement(newstatement,ctemprefnode.create(temp));

                  tsetelementnode(right).left := nil;
                end
              else
                begin
                  if right.nodetype=setelementn then
                    begin
                      result:=internalstatements(newstatement);

                      { create temp for result }
                      temp:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);
                      addstatement(newstatement,temp);

                      { adjust for set base }
                      tsetelementnode(right).left:=caddnode.create(subn,
                        ctypeconvnode.create_internal(tsetelementnode(right).left,sinttype),
                        cordconstnode.create(tsetdef(resultdef).setbase,sinttype,false));

                      { add a range or a single element? }
                      if assigned(tsetelementnode(right).right) then
                        begin
                          { adjust for set base }
                          tsetelementnode(right).right:=caddnode.create(subn,
                            ctypeconvnode.create_internal(tsetelementnode(right).right,sinttype),
                            cordconstnode.create(tsetdef(resultdef).setbase,sinttype,false));
                          addstatement(newstatement,ccallnode.createintern('fpc_varset_set_range',
                            ccallparanode.create(cordconstnode.create(resultdef.size,sinttype,false),
                            ccallparanode.create(tsetelementnode(right).right,
                            ccallparanode.create(tsetelementnode(right).left,
                            ccallparanode.create(ctemprefnode.create(temp),
                            ccallparanode.create(left,nil))))))
                          );
                        end
                      else
                        addstatement(newstatement,ccallnode.createintern('fpc_varset_set',
                          ccallparanode.create(cordconstnode.create(resultdef.size,sinttype,false),
                          ccallparanode.create(ctypeconvnode.create_internal(tsetelementnode(right).left,sinttype),
                          ccallparanode.create(ctemprefnode.create(temp),
                          ccallparanode.create(left,nil)))))
                        );
                      { remove reused parts from original node }
                      tsetelementnode(right).right:=nil;
                      tsetelementnode(right).left:=nil;
                      left:=nil;
                      { the last statement should return the value as
                        location and type, this is done be referencing the
                        temp and converting it first from a persistent temp to
                        normal temp }
                      addstatement(newstatement,ctempdeletenode.create_normal_temp(temp));
                      addstatement(newstatement,ctemprefnode.create(temp));
                    end
                  else
                    call_varset_helper('fpc_varset_add_sets');
                end
            end;
          subn:
            call_varset_helper('fpc_varset_sub_sets');
          symdifn:
            call_varset_helper('fpc_varset_symdif_sets');
          muln:
            call_varset_helper('fpc_varset_mul_sets');
          else
            internalerror(200609241);
        end;
      end;

    function taddnode.first_adddynarray : tnode;
      var
        newstatement : tstatementnode;
        tempnode (*,tempnode2*) : ttempcreatenode;
        para: tcallparanode;
      begin
        result:=nil;
        { when we get here, we are sure that both the left and the right }
        { node are both strings of the same stringtype (JM)              }
        case nodetype of
          addn:
            begin
              if (left.nodetype=arrayconstructorn) and (tarrayconstructornode(left).isempty) then
                begin
                  result:=right;
                  left.free;
                  left:=nil;
                  right:=nil;
                  exit;
                end;
              if (right.nodetype=arrayconstructorn) and (tarrayconstructornode(right).isempty) then
                begin
                  result:=left;
                  left:=nil;
                  right.free;
                  right:=nil;
                  exit;
                end;
              { create the call to the concat routine both strings as arguments }
              if assigned(aktassignmentnode) and
                  (aktassignmentnode.right=self) and
                  (aktassignmentnode.left.resultdef=resultdef) and
                  valid_for_var(aktassignmentnode.left,false) then
                begin
                  para:=ccallparanode.create(
                          ctypeconvnode.create_internal(right,voidcodepointertype),
                        ccallparanode.create(
                          ctypeconvnode.create_internal(left,voidcodepointertype),
                        ccallparanode.create(
                          caddrnode.create_internal(crttinode.create(tstoreddef(resultdef),initrtti,rdt_normal)),
                        ccallparanode.create(
                          ctypeconvnode.create_internal(aktassignmentnode.left.getcopy,voidcodepointertype),nil)
                        )));
                  result:=ccallnode.createintern(
                            'fpc_dynarray_concat',
                            para
                          );
                  include(aktassignmentnode.flags,nf_assign_done_in_right);
                  firstpass(result);
                end
              else
                begin
                  result:=internalstatements(newstatement);
                  tempnode:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);
                  addstatement(newstatement,tempnode);
                  { initialize the temp, since it will be passed to a
                    var-parameter (and finalization, which is performed by the
                    ttempcreate node and which takes care of the initialization
                    on native targets, is a noop on managed VM targets) }
                  if (target_info.system in systems_managed_vm) and
                     is_managed_type(resultdef) then
                    addstatement(newstatement,cinlinenode.create(in_setlength_x,
                      false,
                      ccallparanode.create(genintconstnode(0),
                        ccallparanode.create(ctemprefnode.create(tempnode),nil))));
                  para:=ccallparanode.create(
                          ctypeconvnode.create_internal(right,voidcodepointertype),
                        ccallparanode.create(
                          ctypeconvnode.create_internal(left,voidcodepointertype),
                        ccallparanode.create(
                          caddrnode.create_internal(crttinode.create(tstoreddef(resultdef),initrtti,rdt_normal)),
                        ccallparanode.create(
                          ctypeconvnode.create_internal(ctemprefnode.create(tempnode),voidcodepointertype),nil)
                        )));
                  addstatement(
                    newstatement,
                    ccallnode.createintern(
                      'fpc_dynarray_concat',
                      para
                    )
                  );
                  addstatement(newstatement,ctempdeletenode.create_normal_temp(tempnode));
                  addstatement(newstatement,ctemprefnode.create(tempnode));
                end;
              { we reused the arguments }
              left := nil;
              right := nil;
            end;
          unequaln,equaln:
            { nothing to do }
            ;
          else
            Internalerror(2018030301);
          end;
      end;


    function taddnode.use_generic_mul32to64: boolean;
      begin
        result := true;
      end;


    function taddnode.use_generic_mul64bit: boolean;
      begin
        result := true;
      end;


    function taddnode.try_make_mul32to64: boolean;

      function canbe32bitint(v: tconstexprint; out canbesignedconst, canbeunsignedconst: boolean): boolean;
        begin
          result := ((v >= int64(low(longint))) and (v <= int64(high(longint)))) or
                    ((v >= qword(low(cardinal))) and (v <= qword(high(cardinal))));
          canbesignedconst:=v<=int64(high(longint));
          canbeunsignedconst:=v>=0;
        end;

      function is_32bitordconst(n: tnode; out canbesignedconst, canbeunsignedconst: boolean): boolean;
        begin
          canbesignedconst:=false;
          canbeunsignedconst:=false;
          result := (n.nodetype = ordconstn) and
                    canbe32bitint(tordconstnode(n).value, canbesignedconst, canbeunsignedconst);
        end;

      function is_32to64typeconv(n: tnode): boolean;
        begin
          result := (n.nodetype = typeconvn) and
                    is_integer(ttypeconvnode(n).left.resultdef) and
                    not is_64bit(ttypeconvnode(n).left.resultdef);
        end;

      var
        temp: tnode;
        leftoriginallysigned,
        canbesignedconst, canbeunsignedconst: boolean;
      begin
        result := false;
        if is_32to64typeconv(left) then
          begin
            leftoriginallysigned:=is_signed(ttypeconvnode(left).left.resultdef);
            if ((is_32bitordconst(right,canbesignedconst, canbeunsignedconst) and
                 ((leftoriginallysigned and canbesignedconst) or
                  (not leftoriginallysigned and canbeunsignedconst))) or
                (is_32to64typeconv(right) and
                  ((leftoriginallysigned =
                    is_signed(ttypeconvnode(right).left.resultdef)) or
                   (leftoriginallysigned and
                    (torddef(ttypeconvnode(right).left.resultdef).ordtype in [u8bit,u16bit]))))) then
              begin
                temp := ttypeconvnode(left).left;
                ttypeconvnode(left).left := nil;
                left.free;
                left := temp;
                if (right.nodetype = typeconvn) then
                  begin
                    temp := ttypeconvnode(right).left;
                    ttypeconvnode(right).left := nil;
                    right.free;
                    right := temp;
                  end;
                if (is_signed(left.resultdef)) then
                  begin
                    inserttypeconv_internal(left,s32inttype);
                    inserttypeconv_internal(right,s32inttype);
                  end
                else
                  begin
                    inserttypeconv_internal(left,u32inttype);
                    inserttypeconv_internal(right,u32inttype);
                  end;
                firstpass(left);
                firstpass(right);
                result := true;
              end;
          end;
      end;


    function taddnode.use_fma : boolean;
      begin
        result:=false;
      end;


    function taddnode.try_fma(ld,rd : tdef) : tnode;
      var
        inlinennr : tinlinenumber;
      begin
        result:=nil;
        if (cs_opt_fastmath in current_settings.optimizerswitches) and
          use_fma and
          (nodetype in [addn,subn]) and
          (rd.typ=floatdef) and (ld.typ=floatdef) and
          (is_single(rd) or is_double(rd)) and
          equal_defs(rd,ld) and
          { transforming a*b+c into fma(a,b,c) makes only sense if c can be
            calculated easily. Consider a*b+c*d which results in

            fmul
            fmul
            fadd

            and in

            fmul
            fma

            when using the fma optimization. On a super scalar architecture, the first instruction
            sequence requires clock_cycles(fmul)+clock_cycles(fadd) clock cycles because the fmuls can be executed in parallel.
            The second sequence requires clock_cycles(fmul)+clock_cycles(fma) because the fma has to wait for the
            result of the fmul. Since typically clock_cycles(fma)>clock_cycles(fadd) applies, the first sequence is better.
          }
          (((left.nodetype=muln) and (node_complexity(right)<3)) or
           ((right.nodetype=muln) and (node_complexity(left)<3)) or
           ((left.nodetype=inlinen) and
            (tinlinenode(left).inlinenumber=in_sqr_real) and
             (node_complexity(right)<3)) or
           ((right.nodetype=inlinen) and
            (tinlinenode(right).inlinenumber=in_sqr_real) and
            (node_complexity(left)<3))
          ) then
          begin
            case tfloatdef(ld).floattype of
              s32real:
               inlinennr:=in_fma_single;
              s64real:
               inlinennr:=in_fma_double;
              s80real:
               inlinennr:=in_fma_extended;
              s128real:
               inlinennr:=in_fma_float128;
              else
                internalerror(2014042601);
            end;
            if left.nodetype=muln then
              begin
                if nodetype=subn then
                  result:=cinlinenode.create(inlinennr,false,ccallparanode.create(cunaryminusnode.create(right),
                    ccallparanode.create(taddnode(left).right,
                    ccallparanode.create(taddnode(left).left,nil
                    ))))
                else
                  result:=cinlinenode.create(inlinennr,false,ccallparanode.create(right,
                    ccallparanode.create(taddnode(left).right,
                    ccallparanode.create(taddnode(left).left,nil
                    ))));
                right:=nil;
                taddnode(left).right:=nil;
                taddnode(left).left:=nil;
              end
            else if right.nodetype=muln then
              begin
                if nodetype=subn then
                  result:=cinlinenode.create(inlinennr,false,ccallparanode.create(left,
                    ccallparanode.create(cunaryminusnode.create(taddnode(right).right),
                    ccallparanode.create(taddnode(right).left,nil
                    ))))
                else
                  result:=cinlinenode.create(inlinennr,false,ccallparanode.create(left,
                    ccallparanode.create(taddnode(right).right,
                    ccallparanode.create(taddnode(right).left,nil
                    ))));
                left:=nil;
                taddnode(right).right:=nil;
                taddnode(right).left:=nil;
              end
            else if (left.nodetype=inlinen) and (tinlinenode(left).inlinenumber=in_sqr_real) then
              begin
                if nodetype=subn then
                  result:=cinlinenode.create(inlinennr,false,ccallparanode.create(cunaryminusnode.create(right),
                    ccallparanode.create(tinlinenode(left).left.getcopy,
                    ccallparanode.create(tinlinenode(left).left.getcopy,nil
                    ))))
                else
                  result:=cinlinenode.create(inlinennr,false,ccallparanode.create(right,
                    ccallparanode.create(tinlinenode(left).left.getcopy,
                    ccallparanode.create(tinlinenode(left).left.getcopy,nil
                    ))));
                right:=nil;
              end
            { we get here only if right is a sqr node }
            else if (right.nodetype=inlinen) and (tinlinenode(right).inlinenumber=in_sqr_real) then
              begin
                if nodetype=subn then
                  result:=cinlinenode.create(inlinennr,false,ccallparanode.create(left,
                    ccallparanode.create(cunaryminusnode.create(tinlinenode(right).left.getcopy),
                    ccallparanode.create(tinlinenode(right).left.getcopy,nil
                    ))))
                else
                  result:=cinlinenode.create(inlinennr,false,ccallparanode.create(left,
                    ccallparanode.create(tinlinenode(right).left.getcopy,
                    ccallparanode.create(tinlinenode(right).left.getcopy,nil
                    ))));
                left:=nil;
              end;
          end;
      end;


    function taddnode.first_add64bitint: tnode;
      var
        procname: string[31];
        temp: tnode;
        power: longint;
      begin
        result := nil;
        { create helper calls mul }
        if nodetype <> muln then
          exit;

        { make sure that if there is a constant, that it's on the right }
        if left.nodetype = ordconstn then
          begin
            temp := right;
            right := left;
            left := temp;
          end;

        { can we use a shift instead of a mul? }
        if not (cs_check_overflow in current_settings.localswitches) and
           (right.nodetype = ordconstn) and
           ispowerof2(tordconstnode(right).value,power) then
          begin
            tordconstnode(right).value := power;
            result := cshlshrnode.create(shln,left,right);
            { left and right are reused }
            left := nil;
            right := nil;
            { return firstpassed new node }
            exit;
          end;

        if try_make_mul32to64 then
          begin
            { this uses the same criteria for signedness as the 32 to 64-bit mul
              handling in the i386 code generator }
            if is_signed(left.resultdef) and is_signed(right.resultdef) then
              procname := 'fpc_mul_longint_to_int64'
            else
              procname := 'fpc_mul_dword_to_qword';

            right := ccallparanode.create(right,ccallparanode.create(left,nil));
            result := ccallnode.createintern(procname,right);
            left := nil;
            right := nil;
          end
        else
          begin
            { can full 64-bit multiplication be handled inline? }
            if not use_generic_mul64bit then
              begin
                { generic handling replaces this node with call to fpc_mul_int64,
                  whose result is int64 }
                if is_currency(resultdef) then
                  resultdef:=s64inttype;
                exit;
              end;

            { when currency is used set the result of the
              parameters to s64bit, so they are not converted }
            if is_currency(resultdef) then
              begin
                left.resultdef:=s64inttype;
                right.resultdef:=s64inttype;
              end;

            { otherwise, create the parameters for the helper }
            right :=   ccallparanode.create(right,ccallparanode.create(left,nil));
            left := nil;
            { only qword needs the unsigned code, the
              signed code is also used for currency }
            if is_signed(resultdef) then
              procname := 'fpc_mul_int64'
            else
              procname := 'fpc_mul_qword';
            if cs_check_overflow in current_settings.localswitches then
              procname := procname + '_checkoverflow';

            result := ccallnode.createintern(procname,right);
            right := nil;
          end;
      end;


    function taddnode.first_addpointer: tnode;
      begin
        result:=nil;
        expectloc:=LOC_REGISTER;
      end;


    function taddnode.first_cmppointer: tnode;
      begin
        result:=nil;
        expectloc:=LOC_FLAGS;
      end;


    function taddnode.first_addfloat : tnode;
      var
        procname: string[31];
        { do we need to reverse the result ? }
        notnode : boolean;
        fdef : tdef;
      begin
        result := nil;
        notnode := false;
        fdef := nil;
        { In non-emulation mode, real opcodes are
          emitted for floating point values.
        }
        if not ((cs_fp_emulation in current_settings.moduleswitches)
{$ifdef cpufpemu}
                or (current_settings.fputype=fpu_soft)
{$endif cpufpemu}
                ) then
          exit;

        if not(target_info.system in systems_wince) then
          begin
            case tfloatdef(left.resultdef).floattype of
              s32real:
                begin
                  fdef:=search_system_type('FLOAT32REC').typedef;
                  procname:='float32';
                end;
              s64real:
                begin
                  fdef:=search_system_type('FLOAT64').typedef;
                  procname:='float64';
                end;
              {!!! not yet implemented
              s128real:
              }
              else
                internalerror(2005082601);
            end;

            case nodetype of
              addn:
                procname:=procname+'_add';
              muln:
                procname:=procname+'_mul';
              subn:
                procname:=procname+'_sub';
              slashn:
                procname:=procname+'_div';
              ltn:
                procname:=procname+'_lt';
              lten:
                procname:=procname+'_le';
              gtn:
                begin
                  procname:=procname+'_lt';
                  swapleftright;
                end;
              gten:
                begin
                  procname:=procname+'_le';
                  swapleftright;
                end;
              equaln:
                procname:=procname+'_eq';
              unequaln:
                begin
                  procname:=procname+'_eq';
                  notnode:=true;
                end;
              else
                CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),left.resultdef.typename,right.resultdef.typename);
            end;
          end
        else
          begin
            case nodetype of
              addn:
                procname:='add';
              muln:
                procname:='mul';
              subn:
                procname:='sub';
              slashn:
                procname:='div';
              ltn:
                procname:='lt';
              lten:
                procname:='le';
              gtn:
                procname:='gt';
              gten:
                procname:='ge';
              equaln:
                procname:='eq';
              unequaln:
                procname:='ne';
              else
                begin
                  CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),left.resultdef.typename,right.resultdef.typename);
                  exit;
                end;
            end;
            case tfloatdef(left.resultdef).floattype of
              s32real:
                begin
                  procname:=procname+'s';
                  if nodetype in [addn,muln,subn,slashn] then
                    procname:=lower(procname);
                end;
              s64real:
                procname:=procname+'d';
              {!!! not yet implemented
              s128real:
              }
              else
                internalerror(2005082602);
            end;

          end;
        { cast softfpu result? }
        if not(target_info.system in systems_wince) then
          begin
            if nodetype in [ltn,lten,gtn,gten,equaln,unequaln] then
              resultdef:=pasbool1type;
            result:=ctypeconvnode.create_internal(ccallnode.createintern(procname,ccallparanode.create(
                ctypeconvnode.create_internal(right,fdef),
                ccallparanode.create(
                  ctypeconvnode.create_internal(left,fdef),nil))),resultdef);
          end
        else
          result:=ccallnode.createintern(procname,ccallparanode.create(right,
             ccallparanode.create(left,nil)));
        left:=nil;
        right:=nil;

        { do we need to reverse the result }
        if notnode then
          result:=cnotnode.create(result);
      end;


{$ifdef cpuneedsmulhelper}
    function taddnode.use_mul_helper: boolean;
      begin
        result:=(nodetype=muln) and
                not(torddef(resultdef).ordtype in [u8bit,s8bit
                {$if defined(cpu16bitalu) or defined(avr)},u16bit,s16bit{$endif}]);
      end;
{$endif cpuneedsmulhelper}


    function taddnode.pass_1 : tnode;

      function isconstsetfewelements(p : tnode) : boolean;
        begin
          result:=(p.nodetype=setconstn) and (tsetconstnode(p).elements<=4);
        end;

      var
{$ifdef addstringopt}
         hp      : tnode;
{$endif addstringopt}
         rd,ld   : tdef;
         i,i2    : longint;
         lt,rt   : tnodetype;
{$ifdef cpuneedsmulhelper}
         procname : string[32];
{$endif cpuneedsmulhelper}
        tempn,varsetnode: tnode;
        mulnode : taddnode;
        constsetnode : tsetconstnode;
        trycreateinnodes : Boolean;
      begin
         result:=nil;
         { Can we optimize multiple string additions into a single call?
           This need to be done on a complete tree to detect the multiple
           add nodes and is therefor done before the subtrees are processed }
         if canbemultistringadd(self) then
           begin
             result:=genmultistringadd(self);
             exit;
           end;

         { Can we optimize multiple dyn. array additions into a single call?
           This need to be done on a complete tree to detect the multiple
           add nodes and is therefor done before the subtrees are processed }
         if (m_array_operators in current_settings.modeswitches) and canbemultidynarrayadd(self) then
           begin
             result:=genmultidynarrayadd(self);
             exit;
           end;

         { typical set tests like (s*[const. set])<>/=[] can be converted into an or'ed chain of in tests
           for var sets if const. set contains only a few elements }
         if (cs_opt_level1 in current_settings.optimizerswitches) and (nodetype in [unequaln,equaln]) and (left.resultdef.typ=setdef) and not(is_smallset(left.resultdef)) then
           begin
             trycreateinnodes:=false;
             mulnode:=nil;
             if (is_emptyset(right) and (left.nodetype=muln) and
                 (isconstsetfewelements(taddnode(left).right) or isconstsetfewelements(taddnode(left).left))) then
               begin
                 trycreateinnodes:=true;
                 mulnode:=taddnode(left);
               end
             else if (is_emptyset(left) and (right.nodetype=muln) and
               (isconstsetfewelements(taddnode(right).right) or isconstsetfewelements(taddnode(right).left))) then
               begin
                 trycreateinnodes:=true;
                 mulnode:=taddnode(right);
               end;

             if trycreateinnodes then
               begin
                 constsetnode:=nil;
                 varsetnode:=nil;
                 if isconstsetfewelements(mulnode.right) then
                   begin
                     constsetnode:=tsetconstnode(mulnode.right);
                     varsetnode:=mulnode.left;
                   end
                 else
                   begin
                     constsetnode:=tsetconstnode(mulnode.left);
                     varsetnode:=mulnode.right;
                   end;
                 { the node is copied so it might have no side effects, if the complexity is too, cse should fix it, so
                   do not check complexity }
                 if not(might_have_sideeffects(varsetnode)) then
                   begin
                     result:=nil;
                     for i:=low(tconstset) to high(tconstset) do
                       if i in constsetnode.value_set^ then
                         begin
                           tempn:=cinnode.create(cordconstnode.create(i,tsetdef(constsetnode.resultdef).elementdef,false),varsetnode.getcopy);
                           if assigned(result) then
                             result:=caddnode.create_internal(orn,result,tempn)
                           else
                             result:=tempn;
                         end;
                     if nodetype=equaln then
                       result:=cnotnode.create(result);
                     exit;
                   end;
               end;
           end;

         { first do the two subtrees }
         firstpass(left);
         firstpass(right);

         if codegenerror then
           exit;

         { load easier access variables }
         rd:=right.resultdef;
         ld:=left.resultdef;
         rt:=right.nodetype;
         lt:=left.nodetype;

         { int/int gives real/real! }
         if nodetype=slashn then
           begin
{$ifdef cpufpemu}
             result:=first_addfloat;
             if assigned(result) then
               exit;
{$endif cpufpemu}
             expectloc:=LOC_FPUREGISTER;
           end

         { if both are orddefs then check sub types }
         else if (ld.typ=orddef) and (rd.typ=orddef) then
           begin
             { optimize multiplacation by a power of 2 }
             if not(cs_check_overflow in current_settings.localswitches) and
                (nodetype = muln) and
                (((left.nodetype = ordconstn) and
                  ispowerof2(tordconstnode(left).value,i)) or
                 ((right.nodetype = ordconstn) and
                  ispowerof2(tordconstnode(right).value,i2))) then
               begin
                 if ((left.nodetype = ordconstn) and
                     ispowerof2(tordconstnode(left).value,i)) then
                   begin
                     tordconstnode(left).value := i;
                     result := cshlshrnode.create(shln,right,left);
                   end
                 else
                   begin
                     tordconstnode(right).value := i2;
                     result := cshlshrnode.create(shln,left,right);
                   end;
                 result.resultdef := resultdef;
                 left := nil;
                 right := nil;
                 exit;
               end;

           { 2 booleans ? }
             if is_boolean(ld) and is_boolean(rd) then
              begin
                if (not(cs_full_boolean_eval in current_settings.localswitches) or
                    (nf_short_bool in flags)) and
                   (nodetype in [andn,orn]) then
                  expectloc:=LOC_JUMP
                else
                 begin
                   if nodetype in [ltn,lten,gtn,gten,equaln,unequaln] then
                     expectloc:=LOC_FLAGS
                   else
                     expectloc:=LOC_REGISTER;
                 end;
              end
             else
             { Both are chars? only convert to shortstrings for addn }
              if is_char(ld) then
               begin
                 if nodetype=addn then
                  internalerror(200103291);
                 expectloc:=LOC_FLAGS;
               end
             else if (nodetype=muln) and
                is_64bitint(resultdef) and
                not use_generic_mul32to64 and
                try_make_mul32to64 then
               begin
                 { if the code generator can handle 32 to 64-bit muls,
                   we're done here }
                 expectloc:=LOC_REGISTER;
               end
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
              { is there a 64 bit type ? }
             else if (torddef(ld).ordtype in [s64bit,u64bit,scurrency]) then
               begin
                 result := first_add64bitint;
                 if assigned(result) then
                   exit;
                  if nodetype in [addn,subn,muln,andn,orn,xorn] then
                    expectloc:=LOC_REGISTER
                  else
                    expectloc:=LOC_JUMP;
               end
{$else if defined(llvm) and cpu32bitalu}
            { llvm does not support 128 bit math on 32 bit targets, which is
              necessary for overflow checking 64 bit operations }
            else if (torddef(ld).ordtype in [s64bit,u64bit,scurrency]) and
                    (cs_check_overflow in current_settings.localswitches) and
                    (nodetype in [addn,subn,muln]) then
              begin
                result := first_add64bitint;
                if assigned(result) then
                  exit;
                 if nodetype in [addn,subn,muln,andn,orn,xorn] then
                   expectloc:=LOC_REGISTER
                 else
                   expectloc:=LOC_JUMP;
              end
{$endif not(cpu64bitalu) and not(cpuhighleveltarget)}
             { generic 32bit conversion }
             else
               begin
{$ifdef cpuneedsmulhelper}
                 if use_mul_helper then
                   begin
                     result := nil;

                     case torddef(resultdef).ordtype of
                       s16bit:
                         procname := 'fpc_mul_integer';
                       u16bit:
                         procname := 'fpc_mul_word';
                       s32bit:
                         procname := 'fpc_mul_longint';
                       u32bit:
                         procname := 'fpc_mul_dword';
                       else
                         internalerror(2011022301);
                     end;
                     if cs_check_overflow in current_settings.localswitches then
                       procname:=procname+'_checkoverflow';
                     result := ccallnode.createintern(procname,
                       ccallparanode.create(right,
                       ccallparanode.create(left,nil)));
                     left := nil;
                     right := nil;
                     firstpass(result);
                     exit;
                   end;
{$endif cpuneedsmulhelper}
                  if nodetype in [addn,subn,muln,andn,orn,xorn] then
                    expectloc:=LOC_REGISTER
{$if not defined(cpuhighleveltarget)}
                  else if torddef(ld).size>sizeof(aint) then
                    expectloc:=LOC_JUMP
{$endif}
                  else
                    expectloc:=LOC_FLAGS;
              end;
           end

         { left side a setdef, must be before string processing,
           else array constructor can be seen as array of char (PFV) }
         else if (ld.typ=setdef) then
           begin
             { small sets are handled inline by the compiler.
               small set doesn't have support for adding ranges }
             if is_smallset(ld) and
                not(
                    (right.nodetype=setelementn) and
                    assigned(tsetelementnode(right).right)
                   ) then
               begin
                 if nodetype in [ltn,lten,gtn,gten,equaln,unequaln] then
                   expectloc:=LOC_FLAGS
                 else
                   expectloc:=LOC_REGISTER;
               end
             else
               begin
                 result := first_addset;
                 if assigned(result) then
                   exit;
                 expectloc:=LOC_CREFERENCE;
               end;
           end

         { compare pchar by addresses like BP/Delphi }
         else if is_pchar(ld) then
           begin
             if nodetype in [addn,subn,muln,andn,orn,xorn] then
               result:=first_addpointer
             else
               result:=first_cmppointer;
           end

         { is one of the operands a string }
         else if (ld.typ=stringdef) then
            begin
              if is_widestring(ld) then
                begin
                   { this is only for add, the comparisaion is handled later }
                   expectloc:=LOC_REGISTER;
                end
              else if is_unicodestring(ld) then
                begin
                   { this is only for add, the comparisaion is handled later }
                   expectloc:=LOC_REGISTER;
                end
              else if is_ansistring(ld) then
                begin
                   { this is only for add, the comparisaion is handled later }
                   expectloc:=LOC_REGISTER;
                end
              else if is_longstring(ld) then
                begin
                   { this is only for add, the comparisaion is handled later }
                   expectloc:=LOC_REFERENCE;
                end
              else
                begin
{$ifdef addstringopt}
                   { can create a call which isn't handled by callparatemp }
                   if canbeaddsstringcharoptnode(self) then
                     begin
                       hp := genaddsstringcharoptnode(self);
                       pass_1 := hp;
                       exit;
                     end
                   else
{$endif addstringopt}
                     begin
                       { Fix right to be shortstring }
                       if is_char(right.resultdef) then
                        begin
                          inserttypeconv(right,cshortstringtype);
                          firstpass(right);
                        end;
                     end;
{$ifdef addstringopt}
                   { can create a call which isn't handled by callparatemp }
                   if canbeaddsstringcsstringoptnode(self) then
                     begin
                       hp := genaddsstringcsstringoptnode(self);
                       pass_1 := hp;
                       exit;
                     end;
{$endif addstringopt}
                end;
             { otherwise, let addstring convert everything }
              result := first_addstring;
              exit;
           end

         { is one a real float ? }
         else if (rd.typ=floatdef) or (ld.typ=floatdef) then
            begin
{$ifdef cpufpemu}
             result:=first_addfloat;
             if assigned(result) then
               exit;
{$endif cpufpemu}
              if nodetype in [addn,subn,muln,andn,orn,xorn] then
                expectloc:=LOC_FPUREGISTER
              else
                expectloc:=LOC_FLAGS;

              result:=try_fma(ld,rd);
              if assigned(result) then
                exit;
            end

         { pointer comperation and subtraction }
         else if (ld.typ=pointerdef) then
            begin
              if nodetype in [addn,subn,muln,andn,orn,xorn] then
                result:=first_addpointer
              else
                result:=first_cmppointer;
           end

         else if is_implicit_pointer_object_type(ld) then
            begin
              expectloc:=LOC_FLAGS;
            end

         else if (ld.typ=classrefdef) then
            begin
              expectloc:=LOC_FLAGS;
            end

         { support procvar=nil,procvar<>nil }
         else if ((ld.typ=procvardef) and (rt=niln)) or
                 ((rd.typ=procvardef) and (lt=niln)) then
            begin
              expectloc:=LOC_FLAGS;
            end

{$ifdef SUPPORT_MMX}
       { mmx support, this must be before the zero based array
         check }
         else if (cs_mmx in current_settings.localswitches) and is_mmx_able_array(ld) and
                 is_mmx_able_array(rd) then
            begin
              expectloc:=LOC_MMXREGISTER;
            end
{$endif SUPPORT_MMX}

         else if (rd.typ=pointerdef) or (ld.typ=pointerdef) then
            begin
              result:=first_addpointer;
            end

         else  if (rd.typ=procvardef) and
                  (ld.typ=procvardef) and
                  equal_defs(rd,ld) then
           begin
             expectloc:=LOC_FLAGS;
           end

         else if (ld.typ=enumdef) then
           begin
              expectloc:=LOC_FLAGS;
           end

{$ifdef SUPPORT_MMX}
         else if (cs_mmx in current_settings.localswitches) and
                 is_mmx_able_array(ld) and
                 is_mmx_able_array(rd) then
            begin
              expectloc:=LOC_MMXREGISTER;
            end
{$endif SUPPORT_MMX}

         else if is_dynamic_array(ld) or is_dynamic_array(rd) then
           begin
             result:=first_adddynarray;
             exit;
           end
         { the general solution is to convert to 32 bit int }
         else
           begin
             expectloc:=LOC_REGISTER;
           end;
      end;

{$ifdef state_tracking}
    function Taddnode.track_state_pass(exec_known:boolean):boolean;

    var factval:Tnode;

    begin
    track_state_pass:=false;
    if left.track_state_pass(exec_known) then
      begin
        track_state_pass:=true;
        left.resultdef:=nil;
        do_typecheckpass(left);
      end;
    factval:=aktstate.find_fact(left);
    if factval<>nil then
        begin
        track_state_pass:=true;
            left.destroy;
            left:=factval.getcopy;
        end;
    if right.track_state_pass(exec_known) then
        begin
        track_state_pass:=true;
        right.resultdef:=nil;
        do_typecheckpass(right);
        end;
    factval:=aktstate.find_fact(right);
    if factval<>nil then
        begin
        track_state_pass:=true;
            right.destroy;
            right:=factval.getcopy;
        end;
    end;
{$endif}

end.
