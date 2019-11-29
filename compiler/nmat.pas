{
    Copyright (c) 2000-2005 by Florian Klaempfl

    Type checking and register allocation for math nodes

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
unit nmat;

{$i fpcdefs.inc}

interface

    uses
       node;

    type
       tmoddivnode = class(tbinopnode)
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function simplify(forinline : boolean) : tnode;override;
         protected
          { override the following if you want to implement }
          { parts explicitely in the code generator (JM)    }
          function use_moddiv64bitint_helper: boolean; virtual;
          function first_moddiv64bitint: tnode; virtual;
          function firstoptimize: tnode; virtual;
          function first_moddivint: tnode; virtual;
       end;
       tmoddivnodeclass = class of tmoddivnode;

       tshlshrnode = class(tbinopnode)
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function simplify(forinline : boolean) : tnode;override;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
          { override the following if you want to implement }
          { parts explicitely in the code generator (CEC)
            Should return nil, if everything will be handled
            in the code generator
          }
          function first_shlshr64bitint: tnode; virtual;
{$endif not cpu64bitalu and not cpuhighleveltarget}
       end;
       tshlshrnodeclass = class of tshlshrnode;

       tunaryminusnode = class(tunarynode)
          constructor create(expr : tnode);virtual;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function simplify(forinline : boolean) : tnode;override;
       end;
       tunaryminusnodeclass = class of tunaryminusnode;

       tunaryplusnode = class(tunarynode)
         constructor create(expr : tnode);virtual;
         function pass_1 : tnode;override;
         function pass_typecheck:tnode;override;
       end;
       tunaryplusnodeclass = class of tunaryplusnode;

       tnotnode = class(tunarynode)
          constructor create(expr : tnode);virtual;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function simplify(forinline : boolean) : tnode;override;
       {$ifdef state_tracking}
          function track_state_pass(exec_known:boolean):boolean;override;
       {$endif}
       end;
       tnotnodeclass = class of tnotnode;

    var
       cmoddivnode : tmoddivnodeclass = tmoddivnode;
       cshlshrnode : tshlshrnodeclass = tshlshrnode;
       cunaryminusnode : tunaryminusnodeclass = tunaryminusnode;
       cunaryplusnode : tunaryplusnodeclass = tunaryplusnode;
       cnotnode : tnotnodeclass = tnotnode;

implementation

    uses
      systems,
      verbose,globals,cutils,compinnr,
      globtype,constexp,
      symconst,symtype,symdef,
      defcmp,defutil,
      htypechk,pass_1,
      cgbase,
      ncon,ncnv,ncal,nadd,nld,nbas,nflw,ninl,
      nutils;

{****************************************************************************
                              TMODDIVNODE
 ****************************************************************************}

    function tmoddivnode.simplify(forinline : boolean):tnode;
      var
        rv,lv : tconstexprint;
      begin
        result:=nil;

        if is_constintnode(right) then
          begin
            rv:=tordconstnode(right).value;
            if rv = 1 then
              begin
                case nodetype of
                  modn:
                    result := cordconstnode.create(0,left.resultdef,true);
                  divn:
                    result := left.getcopy;
                  else
                    internalerror(2019050518);
                end;
                exit;
              end;
            if rv = 0 then
              begin
                Message(parser_e_division_by_zero);
                { recover }
                tordconstnode(right).value := 1;
              end;
            { the following simplification is also required for correctness
              on x86, as its transformation of divisions by constants to
              multiplications and shifts does not handle -1 correctly }
            if (rv=-1) and
               (nodetype=divn) then
              begin
                result:=cunaryminusnode.create(left);
                left:=nil;
                exit;
              end;
            if (nf_isomod in flags) and
              (rv<=0) then
               begin
                 Message(cg_e_mod_only_defined_for_pos_quotient);
                 { recover }
                 tordconstnode(right).value := 1;
               end;
          end;

        if is_constintnode(right) and is_constintnode(left) then
          begin
            rv:=tordconstnode(right).value;
            lv:=tordconstnode(left).value;

            case nodetype of
              modn:
                if nf_isomod in flags then
                  begin
                    if lv>=0 then
                      result:=create_simplified_ord_const(lv mod rv,resultdef,forinline,false)
                    else
                      if ((-lv) mod rv)=0 then
                        result:=create_simplified_ord_const((-lv) mod rv,resultdef,forinline,false)
                      else
                        result:=create_simplified_ord_const(rv-((-lv) mod rv),resultdef,forinline,false);
                  end
                else
                  result:=create_simplified_ord_const(lv mod rv,resultdef,forinline,false);
              divn:
                result:=create_simplified_ord_const(lv div rv,resultdef,forinline,cs_check_overflow in localswitches);
              else
                internalerror(2019050519);
            end;
         end;
      end;


    function tmoddivnode.use_moddiv64bitint_helper: boolean;
      begin
        { not with an ifdef around the call to this routine, because e.g. the
          Java VM has a signed 64 bit division opcode, but not an unsigned
          one }
{$if defined(cpu64bitalu) or defined(cpuhighleveltarget)}
        result:=false;
{$else cpu64bitalu or cpuhighleveltarget}
        result:=
          (left.resultdef.typ=orddef) and
          (right.resultdef.typ=orddef) and
          { include currency as well }
          (is_64bit(left.resultdef) or is_64bit(right.resultdef));
{$endif cpu64bitalu or cpuhighleveltarget}
      end;


    function tmoddivnode.pass_typecheck:tnode;
      var
        else_block,
        hp,t : tnode;
        rd,ld : torddef;
        else_statements,
        statements : tstatementnode;
        result_data : ttempcreatenode;
        nd : torddef;
      begin
         result:=nil;
         typecheckpass(left);
         typecheckpass(right);

         { avoid any problems with type parameters later on }
         if is_typeparam(left.resultdef) or is_typeparam(right.resultdef) then
           begin
             resultdef:=cundefinedtype;
             exit;
           end;

         set_varstate(left,vs_read,[vsf_must_be_valid]);
         set_varstate(right,vs_read,[vsf_must_be_valid]);
         if codegenerror then
           exit;

         { tp procvar support }
         maybe_call_procvar(left,true);
         maybe_call_procvar(right,true);

         { allow operator overloading }
         t:=self;
         if isbinaryoverloaded(t,[]) then
           begin
              result:=t;
              exit;
           end;

         { we need 2 orddefs always }
         if (left.resultdef.typ<>orddef) then
           inserttypeconv(left,sinttype);
         if (right.resultdef.typ<>orddef) then
           inserttypeconv(right,sinttype);
         if codegenerror then
           exit;

         { Try only now to simply constant
           as otherwise you might create
           tconstnode with return type that are
           not compatible with tconst node
           as in bug report 21566 PM }

         result:=simplify(false);
         if assigned(result) then
           exit;

         rd:=torddef(right.resultdef);
         ld:=torddef(left.resultdef);

         { if one operand is a cardinal and the other is a positive constant, convert the }
         { constant to a cardinal as well so we don't have to do a 64bit division (JM)    }
         { Do the same for qwords and positive constants as well, otherwise things like   }
         { "qword mod 10" are evaluated with int64 as result, which is wrong if the       }
         { "qword" was > high(int64) (JM)                                                 }
         { Additionally, do the same for cardinal/qwords and other positive types, but    }
         { always in a way that a smaller type is converted to a bigger type              }
         { (webtbs/tw8870)                                                                }
         if (rd.ordtype in [u8bit,u16bit,u32bit,u64bit]) and
            ((is_constintnode(left) and
              (tordconstnode(left).value >= 0) and
              (tordconstnode(left).value <= get_max_value(rd))) or
             (not is_signed(ld) and
              (rd.size >= ld.size))) then
           begin
             inserttypeconv(left,right.resultdef);
             ld:=torddef(left.resultdef);
           end;
         if (ld.ordtype in [u8bit,u16bit,u32bit,u64bit]) and
            ((is_constintnode(right) and
              (tordconstnode(right).value >= 0) and
              (tordconstnode(right).value <= get_max_value(ld))) or
             (not is_signed(rd) and
              (ld.size >= rd.size))) then
          begin
            inserttypeconv(right,left.resultdef);
            rd:=torddef(right.resultdef);
          end;

         { when there is one currency value, everything is done
           using currency }
         if (ld.ordtype=scurrency) or
            (rd.ordtype=scurrency) then
           begin
             if (ld.ordtype<>scurrency) then
              inserttypeconv(left,s64currencytype);
             if (rd.ordtype<>scurrency) then
              inserttypeconv(right,s64currencytype);
             resultdef:=left.resultdef;
           end
         else
          { when there is one 64bit value, everything is done
            in 64bit }
          if (is_64bitint(left.resultdef) or
              is_64bitint(right.resultdef)) then
           begin
             if is_signed(rd) or is_signed(ld) then
               begin
                  if (ld.ordtype<>s64bit) then
                    inserttypeconv(left,s64inttype);
                  if (rd.ordtype<>s64bit) then
                    inserttypeconv(right,s64inttype);
               end
             else
               begin
                  if (ld.ordtype<>u64bit) then
                    inserttypeconv(left,u64inttype);
                  if (rd.ordtype<>u64bit) then
                    inserttypeconv(right,u64inttype);
               end;
             resultdef:=left.resultdef;
           end
         else
          { is there a larger than the native int? }
          if is_oversizedint(ld) or is_oversizedint(rd) then
           begin
             nd:=get_common_intdef(ld,rd,false);
             if (ld.ordtype<>nd.ordtype) then
               inserttypeconv(left,nd);
             if (rd.ordtype<>nd.ordtype) then
               inserttypeconv(right,nd);
             resultdef:=left.resultdef;
           end
         else
          { when mixing unsigned and signed native ints, convert everything to a larger signed type (JM) }
          if (is_nativeuint(rd) and
              is_signed(ld)) or
             (is_nativeuint(ld) and
              is_signed(rd)) then
           begin
              CGMessage(type_h_mixed_signed_unsigned);
              { get a signed int, larger than the native int }
              nd:=get_common_intdef(torddef(sinttype),torddef(uinttype),false);
              if (ld.ordtype<>nd.ordtype) then
                inserttypeconv(left,nd);
              if (rd.ordtype<>nd.ordtype) then
                inserttypeconv(right,nd);
              resultdef:=left.resultdef;
           end
         else
           begin
              { Make everything always default singed int }
              if not(rd.ordtype in [torddef(sinttype).ordtype,torddef(uinttype).ordtype]) then
                inserttypeconv(right,sinttype);
              if not(ld.ordtype in [torddef(sinttype).ordtype,torddef(uinttype).ordtype]) then
                inserttypeconv(left,sinttype);
              resultdef:=right.resultdef;
           end;

         { when the result is currency we need some extra code for
           division. this should not be done when the divn node is
           created internally }
         if (nodetype=divn) and
            not(nf_is_currency in flags) and
            is_currency(resultdef) then
          begin
            hp:=caddnode.create(muln,getcopy,cordconstnode.create(10000,s64currencytype,false));
            include(hp.flags,nf_is_currency);
            result:=hp;
          end;

         if (nodetype=modn) and (nf_isomod in flags) then
           begin
             result:=internalstatements(statements);
             else_block:=internalstatements(else_statements);
             result_data:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);

             { right <=0? }
             addstatement(statements,cifnode.create_internal(caddnode.create_internal(lten,right.getcopy,cordconstnode.create(0,resultdef,false)),
               { then: result:=left mod right }
               ccallnode.createintern('fpc_divbyzero',nil),
               nil
               ));

             { prepare else block }
             { result:=(-left) mod right }
             addstatement(else_statements,cassignmentnode.create(ctemprefnode.create(result_data),cmoddivnode.create(modn,cunaryminusnode.create(left.getcopy),right.getcopy)));
             { result<>0? }
             addstatement(else_statements,cifnode.create_internal(caddnode.create_internal(unequaln,ctemprefnode.create(result_data),cordconstnode.create(0,resultdef,false)),
               { then: result:=right-result }
               cassignmentnode.create_internal(ctemprefnode.create(result_data),caddnode.create_internal(subn,right.getcopy,ctemprefnode.create(result_data))),
               nil
               ));

             addstatement(statements,result_data);
             { if left>=0 }
             addstatement(statements,cifnode.create_internal(caddnode.create_internal(gten,left.getcopy,cordconstnode.create(0,resultdef,false)),
               { then: result:=left mod right }
               cassignmentnode.create_internal(ctemprefnode.create(result_data),cmoddivnode.create(modn,left.getcopy,right.getcopy)),
               { else block }
               else_block
               ));

             addstatement(statements,ctempdeletenode.create_normal_temp(result_data));
             addstatement(statements,ctemprefnode.create(result_data));
           end;
      end;


    function tmoddivnode.first_moddivint: tnode;
{$ifdef cpuneedsdivhelper}
      var
        procname: string[31];
      begin
        result := nil;

        { otherwise create a call to a helper }
        if nodetype = divn then
          procname := 'fpc_div_'
        else
          procname := 'fpc_mod_';

        { only qword needs the unsigned code, the
          signed code is also used for currency }
        case torddef(resultdef).ordtype of
          u8bit:
            procname := procname + 'byte';
          s8bit:
            procname := procname + 'shortint';
          u16bit:
            procname := procname + 'word';
          s16bit:
            procname := procname + 'smallint';
          u32bit:
            procname := procname + 'dword';
          s32bit:
            procname := procname + 'longint';
          scurrency:
            procname := procname + 'currency';
          else
            internalerror(2015070501);
        end;

        result := ccallnode.createintern(procname,ccallparanode.create(left,
          ccallparanode.create(right,nil)));
        left := nil;
        right := nil;
        firstpass(result);

        if result.resultdef.typ<>orddef then
          internalerror(2013031701);
        if resultdef.typ<>orddef then
          internalerror(2013031701);
        if torddef(result.resultdef).ordtype <> torddef(resultdef).ordtype then
          inserttypeconv(result,resultdef);
      end;
{$else cpuneedsdivhelper}
      begin
        result:=nil;
      end;
{$endif cpuneedsdiv32helper}


    function tmoddivnode.first_moddiv64bitint: tnode;
      var
        procname: string[31];
      begin
        result := nil;

        { when currency is used set the result of the
          parameters to s64bit, so they are not converted }
        if is_currency(resultdef) then
          begin
            left.resultdef:=s64inttype;
            right.resultdef:=s64inttype;
          end;

        { otherwise create a call to a helper }
        if nodetype = divn then
          procname := 'fpc_div_'
        else
          procname := 'fpc_mod_';
        { only qword needs the unsigned code, the
          signed code is also used for currency }
        if is_signed(resultdef) then
          procname := procname + 'int64'
        else
          procname := procname + 'qword';

        result := ccallnode.createintern(procname,ccallparanode.create(left,
          ccallparanode.create(right,nil)));
        left := nil;
        right := nil;
        firstpass(result);
      end;


    function tmoddivnode.firstoptimize: tnode;
      var
        power,shiftval : longint;
        statements : tstatementnode;
        temp,resulttemp : ttempcreatenode;
        masknode : tnode;
        invertsign: Boolean;
      begin
        result := nil;
        { divide/mod a number by a constant which is a power of 2? }
        if (right.nodetype = ordconstn) and
          isabspowerof2(tordconstnode(right).value,power) and
{$if defined(cpu64bitalu) or defined(cpuhighleveltarget)}
          { for 64 bit, we leave the optimization to the cg }
            (not is_signed(resultdef)) then
{$else cpu64bitalu or cpuhighleveltarget}
           (((nodetype=divn) and is_oversizedord(resultdef)) or
            (nodetype=modn) or
            not is_signed(resultdef)) then
{$endif cpu64bitalu or cpuhighleveltarget}
          begin
            if nodetype=divn then
              begin
                if is_signed(resultdef) then
                  begin
                    invertsign:=tordconstnode(right).value<0;
                    if is_64bitint(left.resultdef) then
                      if not (cs_opt_size in current_settings.optimizerswitches) then
                        shiftval:=63
                      else
                        { the shift code is a lot bigger than the call to }
                        { the divide helper                               }
                        exit
                    else
                      shiftval:=left.resultdef.size*8-1;

                    result:=internalstatements(statements);
                    temp:=ctempcreatenode.create(left.resultdef,left.resultdef.size,tt_persistent,true);
                    resulttemp:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);
                    addstatement(statements,resulttemp);
                    addstatement(statements,temp);
                    addstatement(statements,cassignmentnode.create(ctemprefnode.create(temp),
                     left));
                    left:=nil;

                    { masknode is (sar(temp,shiftval) and ((1 shl power)-1))
                      for power=1 (i.e. division by 2), masknode is simply (temp shr shiftval)}
                    if power=1 then
                      masknode:=
                        cshlshrnode.create(shrn,
                          ctemprefnode.create(temp),
                          cordconstnode.create(shiftval,u8inttype,false)
                        )
                    else
                      masknode:=
                        caddnode.create(andn,
                          cinlinenode.create(in_sar_x_y,false,
                            ccallparanode.create(cordconstnode.create(shiftval,u8inttype,false),
                            ccallparanode.create(ctemprefnode.create(temp),nil))
                          ),
                          cordconstnode.create(tcgint((qword(1) shl power)-1),
                            right.resultdef,false)
                        );

                    if invertsign then
                      addstatement(statements,cassignmentnode.create(ctemprefnode.create(resulttemp),
                        cunaryminusnode.create(
                          cinlinenode.create(in_sar_x_y,false,
                            ccallparanode.create(cordconstnode.create(power,u8inttype,false),
                            ccallparanode.create(caddnode.create(addn,ctemprefnode.create(temp),
                              masknode),nil
                            )))))
                      )
                    else
                      addstatement(statements,cassignmentnode.create(ctemprefnode.create(resulttemp),
                        cinlinenode.create(in_sar_x_y,false,
                          ccallparanode.create(cordconstnode.create(power,u8inttype,false),
                          ccallparanode.create(caddnode.create(addn,ctemprefnode.create(temp),
                            masknode),nil
                          ))))
                      );
                    addstatement(statements,ctempdeletenode.create(temp));
                    addstatement(statements,ctempdeletenode.create_normal_temp(resulttemp));
                    addstatement(statements,ctemprefnode.create(resulttemp));
                    right.Free;
                  end
                else
                  begin
                    tordconstnode(right).value:=power;
                    result:=cshlshrnode.create(shrn,left,right)
                  end;
              end
            else if is_signed(resultdef) then    { signed modulus }
              begin
                if (cs_opt_size in current_settings.optimizerswitches) then
                  exit;

                shiftval:=left.resultdef.size*8-1;
                tordconstnode(right).value.uvalue:=qword((qword(1) shl power)-1);

                result:=internalstatements(statements);
                temp:=ctempcreatenode.create(left.resultdef,left.resultdef.size,tt_persistent,true);
                resulttemp:=ctempcreatenode.create(resultdef,resultdef.size,tt_persistent,true);
                addstatement(statements,resulttemp);
                addstatement(statements,temp);
                addstatement(statements,cassignmentnode.create(ctemprefnode.create(temp),left));
                { mask:=sar(left,sizeof(left)*8-1) and ((1 shl power)-1); }
                if power=1 then
                  masknode:=
                    cshlshrnode.create(shrn,
                      ctemprefnode.create(temp),
                      cordconstnode.create(shiftval,u8inttype,false)
                    )
                else
                  masknode:=
                    caddnode.create(andn,
                      cinlinenode.create(in_sar_x_y,false,
                        ccallparanode.create(cordconstnode.create(shiftval,u8inttype,false),
                        ccallparanode.create(ctemprefnode.create(temp),nil))
                      ),
                      cordconstnode.create(tcgint((qword(1) shl power)-1),
                        right.resultdef,false)
                    );
                addstatement(statements,cassignmentnode.create(ctemprefnode.create(resulttemp),masknode));

                { result:=((left+mask) and right)-mask; }
                addstatement(statements,cassignmentnode.create(ctemprefnode.create(resulttemp),
                  caddnode.create(subn,
                    caddnode.create(andn,
                      right,
                      caddnode.create(addn,
                        ctemprefnode.create(temp),
                        ctemprefnode.create(resulttemp))),
                  ctemprefnode.create(resulttemp))
                ));

                addstatement(statements,ctempdeletenode.create(temp));
                addstatement(statements,ctempdeletenode.create_normal_temp(resulttemp));
                addstatement(statements,ctemprefnode.create(resulttemp));
              end
            else
              begin
                tordconstnode(right).value.uvalue:=qword((qword(1) shl power)-1);
                result := caddnode.create(andn,left,right);
              end;
            { left and right are reused }
            left := nil;
            right := nil;
            firstpass(result);
            exit;
          end;
      end;


    function tmoddivnode.pass_1 : tnode;
      begin
         result:=nil;
         firstpass(left);
         firstpass(right);
         if codegenerror then
           exit;

         { Try to optimize mod/div }
         result := firstoptimize;
         if assigned(result) then
           exit;

         { 64bit }
         if use_moddiv64bitint_helper then
           begin
             result := first_moddiv64bitint;
             if assigned(result) then
               exit;
             expectloc:=LOC_REGISTER;
           end
         else
           begin
             result := first_moddivint;
             if assigned(result) then
               exit;
           end;
         expectloc:=LOC_REGISTER;
      end;



{****************************************************************************
                              TSHLSHRNODE
 ****************************************************************************}

    function tshlshrnode.simplify(forinline : boolean):tnode;
      var
        lvalue, rvalue, mask : Tconstexprint;
        rangedef: tdef;
        size: longint;
      begin
        result:=nil;
        { constant folding }
        if is_constintnode(right) then
          begin
            if forinline then
              begin
                case resultdef.size of
                  1,2,4:
                    rvalue:=tordconstnode(right).value and byte($1f);
                  8:
                    rvalue:=tordconstnode(right).value and byte($3f);
                  else
                    internalerror(2013122302);
                end;
              end
            else
              rvalue:=tordconstnode(right).value;
            if is_constintnode(left) then
               begin
                 lvalue:=tordconstnode(left).value;
                 getrangedefmasksize(resultdef, rangedef, mask, size);
                 { shr is an unsigned operation, so cut off upper bits }
                 if forinline then
                   lvalue:=lvalue and mask;
                 case nodetype of
                    shrn:
                      lvalue:=lvalue shr rvalue;
                    shln:
                      lvalue:=lvalue shl rvalue;
                    else
                      internalerror(2019050517);
                 end;
                 { discard shifted-out bits (shl never triggers overflow/range errors) }
                 if forinline and
                    (nodetype=shln) then
                   lvalue:=lvalue and mask;
                 result:=create_simplified_ord_const(lvalue,resultdef,forinline,false);
               end
            else if rvalue=0 then
              begin
                result:=left;
                left:=nil;
              end;
          end
        else if is_constintnode(left) then
          begin
            lvalue:=tordconstnode(left).value;
            if forinline then
              begin
                getrangedefmasksize(resultdef, rangedef, mask, size);
                lvalue:=lvalue and mask;
              end;
            { '0 shl x' and '0 shr x' are 0 }
            if (lvalue=0) and
               ((cs_opt_level4 in current_settings.optimizerswitches) or
                not might_have_sideeffects(right)) then
              result:=cordconstnode.create(0,resultdef,true);
          end;
      end;


    function tshlshrnode.pass_typecheck:tnode;
      var
         t : tnode;
      begin
         result:=nil;
         typecheckpass(left);
         typecheckpass(right);

         { avoid any problems with type parameters later on }
         if is_typeparam(left.resultdef) or is_typeparam(right.resultdef) then
           begin
             resultdef:=cundefinedtype;
             exit;
           end;

         set_varstate(right,vs_read,[vsf_must_be_valid]);
         set_varstate(left,vs_read,[vsf_must_be_valid]);
         if codegenerror then
           exit;

         { tp procvar support }
         maybe_call_procvar(left,true);
         maybe_call_procvar(right,true);

         { allow operator overloading }
         t:=self;
         if isbinaryoverloaded(t,[]) then
           begin
              result:=t;
              exit;
           end;

{$ifdef SUPPORT_MMX}
         if (cs_mmx in current_settings.localswitches) and
           is_mmx_able_array(left.resultdef) and
           ((is_mmx_able_array(right.resultdef) and
             equal_defs(left.resultdef,right.resultdef)
            ) or is_constintnode(right)) then
           begin
             if not(mmx_type(left.resultdef) in [mmxu16bit,mmxs16bit,mmxfixed16,mmxu32bit,mmxs32bit,mmxu64bit,mmxs64bit]) then
               CGMessage3(type_e_operator_not_supported_for_types,node2opstr(nodetype),left.resultdef.typename,right.resultdef.typename);
             if not(is_mmx_able_array(right.resultdef)) then
               inserttypeconv(right,sinttype);
           end
         else
{$endif SUPPORT_MMX}
           begin
             { calculations for ordinals < 32 bit have to be done in
               32 bit for backwards compatibility. That way 'shl 33' is
               the same as 'shl 1'. It's ugly but compatible with delphi/tp/gcc }
             if (not is_64bit(left.resultdef)) and
                (torddef(left.resultdef).ordtype<>u32bit) then
               begin
                 { keep singness of orignal type }
                 if is_signed(left.resultdef) then
                   begin
{$if defined(cpu64bitalu) or defined(cpu32bitalu)}
                     inserttypeconv(left,s32inttype)
{$elseif defined(cpu16bitalu) or defined(cpu8bitalu)}
                     inserttypeconv(left,get_common_intdef(torddef(left.resultdef),torddef(sinttype),true));
{$else}
                     internalerror(2013031301);
{$endif}
                   end
                 else
                   begin
{$if defined(cpu64bitalu) or defined(cpu32bitalu)}
                     inserttypeconv(left,u32inttype);
{$elseif defined(cpu16bitalu) or defined(cpu8bitalu)}
                     inserttypeconv(left,get_common_intdef(torddef(left.resultdef),torddef(uinttype),true));
{$else}
                     internalerror(2013031301);
{$endif}
                   end
               end;

             inserttypeconv(right,sinttype);
           end;

         resultdef:=left.resultdef;

         result:=simplify(false);
         if assigned(result) then
           exit;
      end;


{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
    function tshlshrnode.first_shlshr64bitint: tnode;
      var
        procname: string[31];
      begin
        result := nil;
        { Normally already done below, but called again,
          just in case it is called directly }
        firstpass(left);
        { otherwise create a call to a helper }
        if is_signed(left.resultdef) then
          procname:='int64'
        else
          procname:='qword';
        if nodetype = shln then
          procname := 'fpc_shl_'+procname
        else
          procname := 'fpc_shr_'+procname;
        { this order of parameters works at least for the arm,
          however it should work for any calling conventions (FK) }
        result := ccallnode.createintern(procname,ccallparanode.create(right,
          ccallparanode.create(left,nil)));
        left := nil;
        right := nil;
        firstpass(result);
      end;
{$endif not cpu64bitalu and not cpuhighleveltarget}


    function tshlshrnode.pass_1 : tnode;
      begin
         result:=nil;
         firstpass(left);
         firstpass(right);
         if codegenerror then
           exit;

         expectloc:=LOC_REGISTER;
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
         { 64 bit ints have their own shift handling }
         if is_64bit(left.resultdef) then
           result := first_shlshr64bitint;
{$endif not cpu64bitalu and not cpuhighleveltarget}
      end;


{****************************************************************************
                            TUNARYMINUSNODE
 ****************************************************************************}

    constructor tunaryminusnode.create(expr : tnode);
      begin
         inherited create(unaryminusn,expr);
      end;


    function tunaryminusnode.simplify(forinline : boolean):tnode;
      begin
        result:=nil;
        { constant folding }
        if is_constintnode(left) then
          begin
             result:=create_simplified_ord_const(-tordconstnode(left).value,resultdef,forinline,cs_check_overflow in localswitches);
             exit;
          end;
        if is_constrealnode(left) then
          begin
             trealconstnode(left).value_real:=-trealconstnode(left).value_real;
             { Avoid integer overflow on x86_64 CPU for currency value }
             { i386 uses fildll/fchs/fistll instructions which never seem
               to raise any coprocessor flags .. }
             {$push}{$Q-}
             trealconstnode(left).value_currency:=-trealconstnode(left).value_currency;
             result:=left;
             {$pop}
             left:=nil;
             exit;
          end;
      end;


    function tunaryminusnode.pass_typecheck : tnode;
      var
         t : tnode;
      begin
         result:=nil;
         typecheckpass(left);

         { avoid any problems with type parameters later on }
         if is_typeparam(left.resultdef) then
           begin
             resultdef:=cundefinedtype;
             exit;
           end;

         set_varstate(left,vs_read,[vsf_must_be_valid]);
         if codegenerror then
           exit;

         result:=simplify(false);
         if assigned(result) then
           exit;

         resultdef:=left.resultdef;
         if (left.resultdef.typ=floatdef) or
            is_currency(left.resultdef) then
           begin
           end
{$ifdef SUPPORT_MMX}
         else if (cs_mmx in current_settings.localswitches) and
           is_mmx_able_array(left.resultdef) then
             begin
               { if saturation is on, left.resultdef isn't
                 "mmx able" (FK)
               if (cs_mmx_saturation in current_settings.localswitches^) and
                 (torddef(tarraydef(resultdef).definition).typ in
                 [s32bit,u32bit]) then
                 CGMessage(type_e_mismatch);
               }
             end
{$endif SUPPORT_MMX}
         else if is_oversizedord(left.resultdef) then
           begin
             if is_64bit(left.resultdef) then
               inserttypeconv(left,s64inttype)
             else if is_32bit(left.resultdef) then
               inserttypeconv(left,s32inttype)
             else if is_16bit(left.resultdef) then
               inserttypeconv(left,s16inttype)
             else
               internalerror(2013040701);
             resultdef:=left.resultdef;
           end
         else if (left.resultdef.typ=orddef) then
           begin
             inserttypeconv(left,sinttype);
             resultdef:=left.resultdef
           end
         else
           begin
             { allow operator overloading }
             t:=self;
             if isunaryoverloaded(t,[]) then
               begin
                  result:=t;
                  exit;
               end;

             CGMessage(type_e_mismatch);
           end;
      end;

    { generic code     }
    { overridden by:   }
    {   i386           }
    function tunaryminusnode.pass_1 : tnode;
      var
        procname: string[31];
      begin
        result:=nil;
        firstpass(left);
        if codegenerror then
          exit;

        if (cs_fp_emulation in current_settings.moduleswitches) and (left.resultdef.typ=floatdef) then
          begin
            if not(target_info.system in systems_wince) then
              begin
                expectloc:=LOC_REGISTER;
                exit;
              end
            else
              begin
                case tfloatdef(resultdef).floattype of
                  s32real:
                    procname:='negs';
                  s64real:
                    procname:='negd';
                  {!!! not yet implemented
                  s128real:
                  }
                  else
                    internalerror(2005082802);
                end;
                result:=ccallnode.createintern(procname,ccallparanode.create(left,nil));
              end;

            left:=nil;
          end
        else
          begin
            if (left.resultdef.typ=floatdef) then
              expectloc:=LOC_FPUREGISTER
{$ifdef SUPPORT_MMX}
             else if (cs_mmx in current_settings.localswitches) and
               is_mmx_able_array(left.resultdef) then
              expectloc:=LOC_MMXREGISTER
{$endif SUPPORT_MMX}
             else if (left.resultdef.typ=orddef) then
               expectloc:=LOC_REGISTER;
          end;
      end;

{****************************************************************************
                             TUNARYPLUSNODE
 ****************************************************************************}

    constructor tunaryplusnode.create(expr: tnode);
      begin
        inherited create(unaryplusn,expr);
      end;

    function tunaryplusnode.pass_1: tnode;
      begin
        result:=nil;
        { can never happen because all the conversions happen
          in pass_typecheck }
        internalerror(201012250);
      end;

    function tunaryplusnode.pass_typecheck: tnode;
      var
        t:tnode;
      begin
        result:=nil;
        typecheckpass(left);

        { avoid any problems with type parameters later on }
        if is_typeparam(left.resultdef) then
          begin
            resultdef:=cundefinedtype;
            exit;
          end;

        set_varstate(left,vs_read,[vsf_must_be_valid]);
        if codegenerror then
          exit;

        if is_constintnode(left) or
           is_constrealnode(left) or
           (left.resultdef.typ=floatdef) or
           is_currency(left.resultdef)
{$ifdef SUPPORT_MMX}
           or ((cs_mmx in current_settings.localswitches) and
                is_mmx_able_array(left.resultdef))
{$endif SUPPORT_MMX}
        then
          begin
            result:=left;
            left:=nil;
          end
        else if is_oversizedord(left.resultdef) then
          begin
            if is_64bit(left.resultdef) then
              inserttypeconv(left,s64inttype)
            else if is_32bit(left.resultdef) then
              inserttypeconv(left,s32inttype)
            else if is_16bit(left.resultdef) then
              inserttypeconv(left,s16inttype)
            else
              internalerror(2013040702);
            result:=left;
            left:=nil;
          end
        else if (left.resultdef.typ=orddef) then
          begin
            inserttypeconv(left,sinttype);
            result:=left;
            left:=nil;
          end
        else
          begin
            { allow operator overloading }
            t:=self;
            if isunaryoverloaded(t,[]) then
              begin
                result:=t;
                exit;
             end;

             CGMessage(type_e_mismatch);
           end;
      end;


{****************************************************************************
                               TNOTNODE
 ****************************************************************************}

    const
      boolean_reverse:array[ltn..unequaln] of Tnodetype=(
        gten,gtn,lten,ltn,unequaln,equaln
      );

    constructor tnotnode.create(expr : tnode);
      begin
         inherited create(notn,expr);
      end;


    function tnotnode.simplify(forinline : boolean):tnode;
      var
        v : tconstexprint;
        t : tnode;
        def : tdef;
      begin
        result:=nil;
        { Try optmimizing ourself away }
        if left.nodetype=notn then
          begin
            { Double not. Remove both }
            result:=Tnotnode(left).left;
            tnotnode(left).left:=nil;
            exit;
          end;

        if (left.nodetype in [ltn,lten,equaln,unequaln,gtn,gten]) then
         begin
           { Not of boolean expression. Turn around the operator and remove
             the not. This is not allowed for sets with the gten/lten,
             because there is no ltn/gtn support }
           if (taddnode(left).left.resultdef.typ<>setdef) or
              (left.nodetype in [equaln,unequaln]) then
            begin
              result:=left;
              left.nodetype:=boolean_reverse[left.nodetype];
              left:=nil;
              exit;
            end;
         end;

        { constant folding }
        if (left.nodetype=ordconstn) and
          (left.resultdef.typ=orddef) then
          begin
             v:=tordconstnode(left).value;
             def:=left.resultdef;
             case torddef(left.resultdef).ordtype of
               pasbool1,
               pasbool8,
               pasbool16,
               pasbool32,
               pasbool64:
                 v:=byte(not(boolean(int64(v))));
               bool8bit,
               bool16bit,
               bool32bit,
               bool64bit:
                 begin
                   if v=0 then
                     v:=-1
                   else
                     v:=0;
                 end;
               uchar,
               uwidechar,
               u8bit,
               s8bit,
               u16bit,
               s16bit,
               s32bit,
               u32bit,
               s64bit,
               u64bit:
                 begin
                   { unsigned, equal or bigger than the native int size? }
                   if (torddef(left.resultdef).ordtype in [u64bit,u32bit,u16bit,u8bit,uchar,uwidechar]) and
                      (is_nativeord(left.resultdef) or is_oversizedord(left.resultdef)) then
                     begin
                       { Delphi-compatible: not dword = dword (not word = longint) }
                       { Extension: not qword = qword                              }
                       v:=qword(not qword(v));
                       { will be truncated by the ordconstnode for u32bit }
                     end
                   else
                     begin
                       v:=int64(not int64(v));
                       def:=get_common_intdef(torddef(left.resultdef),torddef(sinttype),false);
                     end;
                 end;
               else
                 CGMessage(type_e_mismatch);
             end;
             { not-nodes are not range checked by the code generator -> also
               don't range check while inlining; the resultdef is a bit tricky
               though: the node's resultdef gets changed in most cases compared
               to left, but the not-operation itself is caried out in the code
               generator using the size of left
               }
             if not(forinline) then
               t:=cordconstnode.create(v,def,false)
             else
               begin
                 { cut off the value if necessary }
                 t:=cordconstnode.create(v,left.resultdef,false);
                 { now convert to node's resultdef }
                 inserttypeconv_explicit(t,def);
               end;
             result:=t;
             exit;
          end;
      end;


    function tnotnode.pass_typecheck : tnode;
      var
         t : tnode;
      begin
         result:=nil;
         typecheckpass(left);

         { avoid any problems with type parameters later on }
         if is_typeparam(left.resultdef) then
           begin
             resultdef:=cundefinedtype;
             exit;
           end;

         set_varstate(left,vs_read,[vsf_must_be_valid]);
         if codegenerror then
           exit;

         { tp procvar support }
         maybe_call_procvar(left,true);

         resultdef:=left.resultdef;

         result:=simplify(false);
         if assigned(result) then
           exit;

         if is_boolean(resultdef) then
           begin
           end
         else
{$ifdef SUPPORT_MMX}
           if (cs_mmx in current_settings.localswitches) and
             is_mmx_able_array(left.resultdef) then
             begin
             end
         else
{$endif SUPPORT_MMX}
{$ifndef cpu64bitaddr}
           if is_64bitint(left.resultdef) then
             begin
             end
         else
{$endif not cpu64bitaddr}
           if is_integer(left.resultdef) then
             begin
             end
         else
           begin
             { allow operator overloading }
             t:=self;
             if isunaryoverloaded(t,[]) then
               begin
                  result:=t;
                  exit;
               end;

             CGMessage(type_e_mismatch);
           end;
      end;


    function tnotnode.pass_1 : tnode;
      begin
         result:=nil;
         firstpass(left);
         if codegenerror then
           exit;

         expectloc:=left.expectloc;
         if is_boolean(resultdef) then
           begin
             if (expectloc in [LOC_REFERENCE,LOC_CREFERENCE,LOC_CREGISTER]) then
               expectloc:=LOC_REGISTER;
            { before loading it into flags we need to load it into
              a register thus 1 register is need PM }
{$ifdef cpuflags}
             if left.expectloc<>LOC_JUMP then
               expectloc:=LOC_FLAGS;
{$endif def cpuflags}
           end
         else
{$ifdef SUPPORT_MMX}
           if (cs_mmx in current_settings.localswitches) and
             is_mmx_able_array(left.resultdef) then
             expectloc:=LOC_MMXREGISTER
         else
{$endif SUPPORT_MMX}
{$if not defined(cpu64bitalu) and not defined(cpuhighleveltarget)}
           if is_64bit(left.resultdef) then
             begin
                if (expectloc in [LOC_REFERENCE,LOC_CREFERENCE,LOC_CREGISTER]) then
                  expectloc:=LOC_REGISTER;
             end
         else
{$endif not cpu64bitalu and not cpuhighleveltarget}
           if is_integer(left.resultdef) then
             expectloc:=LOC_REGISTER;
      end;

{$ifdef state_tracking}
    function Tnotnode.track_state_pass(exec_known:boolean):boolean;
      begin
        track_state_pass:=true;
        if left.track_state_pass(exec_known) then
          begin
            left.resultdef:=nil;
            do_typecheckpass(left);
          end;
      end;
{$endif}

end.
