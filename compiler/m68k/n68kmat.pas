{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate 680x0 assembler for math nodes

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
unit n68kmat;

{$i fpcdefs.inc}

interface

    uses
      node,nmat,ncgmat,cpubase,cgbase;

    type


      tm68knotnode = class(tcgnotnode)
         procedure second_boolean;override;
      end;

      tm68kmoddivnode = class(tcgmoddivnode)
      public
        function first_moddivint: tnode;override;
        procedure emit_div_reg_reg(signed: boolean;denum,num : tregister);override;
        procedure emit_mod_reg_reg(signed: boolean;denum,num : tregister);override;
      end;

      tm68kunaryminusnode = class(tcgunaryminusnode)
        procedure second_float;override;
      end;

      tm68kshlshrnode = class(tshlshrnode)
         procedure pass_generate_code;override;
         { everything will be handled in pass_2 }
         function first_shlshr64bitint: tnode; override;
      end;


implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,symtable,aasmbase,aasmtai,aasmdata,aasmcpu,
      pass_1,pass_2,procinfo,
      ncon,
      cpuinfo,paramgr,defutil,parabase,
      tgobj,ncgutil,cgobj,hlcgobj,cgutils,rgobj,rgcpu,cgcpu,cg64f32;




{*****************************************************************************
                               TM68KNOTNODE
*****************************************************************************}

    procedure tm68knotnode.second_boolean;
      var
        hreg: tregister;
        opsize : tcgsize;
      begin
        if not handle_locjump then
          begin
            secondpass(left);
            opsize:=def_cgsize(resultdef);
            case left.location.loc of
              LOC_FLAGS :
                begin
                  location_copy(location,left.location);
                  inverse_flags(location.resflags);
                end;
              LOC_REFERENCE,
              LOC_CREFERENCE:
                begin
                  tcg68k(cg).fixref(current_asmdata.CurrAsmList,left.location.reference);
                  if is_64bit(resultdef) then
                   begin
                     hreg:=cg.GetIntRegister(current_asmdata.CurrAsmList,OS_32);
                     cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_32,OS_32,left.location.reference,hreg);
                     inc(left.location.reference.offset,4);
                     cg.a_op_ref_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,left.location.reference,hreg);
                   end
                 else
                   current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_TST,tcgsize2opsize[opsize],left.location.reference));
                   location_reset(location,LOC_FLAGS,OS_NO);
                   location.resflags:=F_E;
                end;
              LOC_REGISTER,
              LOC_CREGISTER,
              LOC_SUBSETREG,
              LOC_CSUBSETREG,
              LOC_SUBSETREF,
              LOC_CSUBSETREF:
                begin
                  if is_64bit(resultdef) then
                    begin
                      hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,false);
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_OR,S_L,left.location.register64.reghi,left.location.register64.reglo));
                    end
                  else
                    begin
                      hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,resultdef,true);
                      current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_TST,tcgsize2opsize[opsize],left.location.register));
                    end;
                  location_reset(location,LOC_FLAGS,OS_NO);
                  location.resflags:=F_E;
                end;
            else
              internalerror(200203223);
            end;
          end;
      end;


{*****************************************************************************
                               TM68KMODDIVNODE
*****************************************************************************}

  function tm68kmoddivnode.first_moddivint: tnode;
    begin
      if current_settings.cputype=cpu_MC68020 then
        result:=nil
      else
        result:=inherited first_moddivint;
    end;


  procedure tm68kmoddivnode.emit_div_reg_reg(signed: boolean;denum,num : tregister);
   begin
     if current_settings.cputype=cpu_MC68020 then
       begin
<<<<<<< HEAD
=======
         { verify if denominator is zero }
         current_asmdata.getjumplabel(continuelabel);
         { compare against zero, if not zero continue }
         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_S32,OC_NE,0,denum,continuelabel);
//       paraloc1.init;
//         cg.a_load_const_cgpara(current_asmdata.CurrAsmList,OS_S32,200,paramanager.getintparaloc(pocall_default,1,paraloc1));

         cg.a_call_name(current_asmdata.CurrAsmList,'FPC_HANDLEERROR',false);
         cg.a_label(current_asmdata.CurrAsmList, continuelabel);
>>>>>>> graemeg/cpstrnew
         if signed then
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_DIVS,S_L,denum,num))
         else
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_DIVU,S_L,denum,num));
       end
     else
       InternalError(2014062801);
   end;


  procedure tm68kmoddivnode.emit_mod_reg_reg(signed: boolean;denum,num : tregister);
    var
      tmpreg : tregister;
    begin
     if current_settings.cputype=cpu_MC68020 then
       begin
<<<<<<< HEAD
=======
         { verify if denominator is zero }
         current_asmdata.getjumplabel(continuelabel);
         { compare against zero, if not zero continue }
         cg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,OS_S32,OC_NE,0,denum,continuelabel);
//         cg.a_load_const_cgpara(current_asmdata.CurrAsmList, OS_S32,200,paramanager.getintparaloc(pocall_default,1));
         cg.a_call_name(current_asmdata.CurrAsmList,'FPC_HANDLEERROR',false);
         cg.a_label(current_asmdata.CurrAsmList, continuelabel);

>>>>>>> graemeg/cpstrnew
         tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
         { copy the numerator to the tmpreg, so we can use it as quotient, which
           means we'll get the remainder immediately in the numerator }
         cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,num,tmpreg);
         if signed then
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_DIVSL,S_L,denum,num,tmpreg))
         else
           current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(A_DIVUL,S_L,denum,num,tmpreg));
       end
     else
       InternalError(2014062802);
    end;


{*****************************************************************************
                          TM68KUNARYMINUSNODE
*****************************************************************************}

    procedure tm68kunaryminusnode.second_float;
      var
        href: treference;
      begin
        secondpass(left);
        location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('unaryminus second_float called!')));

        case left.location.loc of
          LOC_REFERENCE,
          LOC_CREFERENCE :
            begin
              location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
              href:=left.location.reference;
              tcg68k(cg).fixref(current_asmdata.CurrAsmList,href);
              current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(A_FNEG,tcgsize2opsize[left.location.size],href,location.register));
            end;
          LOC_FPUREGISTER:
            begin
              location.register:=left.location.register;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg(A_FNEG,S_FX,location.register));
            end;
          LOC_CFPUREGISTER:
            begin
               location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
               current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FNEG,S_FX,left.location.register,location.register));
            end;
          else
            internalerror(200306021);
        end;
      end;


{*****************************************************************************
                             TM68KSHLRSHRNODE
*****************************************************************************}

    function tm68kShlShrNode.first_shlshr64bitint:TNode;
      begin
        if is_64bit(left.resultdef) and not (right.nodetype=ordconstn) then
          { for 64bit shifts with anything but constants we use rtl helpers }
          result:=inherited
        else
          { 2nd pass is our friend }
          result := nil;
      end;


{ TODO: FIX ME!!! shlshrnode needs review}
    procedure tm68kshlshrnode.pass_generate_code;
      var
        hregister,resultreg,hregister1,
        hreg64hi,hreg64lo : tregister;
        op : topcg;
        shiftval: aint;
      begin
        secondpass(left);
        secondpass(right);
        if is_64bit(left.resultdef) then
          begin
            location_reset(location,LOC_REGISTER,OS_64);

            { load left operator in a register }
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,u64inttype,false);
            hreg64hi:=left.location.register64.reghi;
            hreg64lo:=left.location.register64.reglo;

            shiftval := tordconstnode(right).value.svalue;
<<<<<<< HEAD
<<<<<<< HEAD
            shiftval := shiftval and 63;
=======
	    shiftval := shiftval and 63;
>>>>>>> graemeg/fixes_2_2
=======
	    shiftval := shiftval and 63;
>>>>>>> origin/fixes_2_2
            if shiftval > 31 then
              begin
                if nodetype = shln then
                  begin
                    cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,hreg64hi);
                    if (shiftval and 31) <> 0 then
                      cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHL,OS_32,shiftval and 31,hreg64lo,hreg64lo);
                  end
                else
                  begin
                    cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,hreg64lo);
                    if (shiftval and 31) <> 0 then
                      cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_32,shiftval and 31,hreg64hi,hreg64hi);
                  end;
                location.register64.reglo:=hreg64hi;
                location.register64.reghi:=hreg64lo;
              end
            else
              begin
                hregister:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                if nodetype = shln then
                  begin
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_32,32-shiftval,hreg64lo,hregister);
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHL,OS_32,shiftval,hreg64hi,hreg64hi);
                    cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,hregister,hreg64hi,hreg64hi);
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHL,OS_32,shiftval,hreg64lo,hreg64lo);
                  end
                else
                  begin
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHL,OS_32,32-shiftval,hreg64hi,hregister);
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_32,shiftval,hreg64lo,hreg64lo);
                    cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,hregister,hreg64lo,hreg64lo);
                    cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,OP_SHR,OS_32,shiftval,hreg64hi,hreg64hi);
                  end;
                location.register64.reghi:=hreg64hi;
                location.register64.reglo:=hreg64lo;
              end;
          end
        else
          begin
            { load left operators in a register }
            hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
            location_copy(location,left.location);
            resultreg := location.register;
            hregister1 := location.register;
            if (location.loc = LOC_CREGISTER) then
              begin
                location.loc := LOC_REGISTER;
                resultreg := cg.GetIntRegister(current_asmdata.CurrAsmList,OS_INT);
                location.register := resultreg;
              end;
            { determine operator }
            if nodetype=shln then
              op:=OP_SHL
            else
              op:=OP_SHR;
            { shifting by a constant directly coded: }
            if (right.nodetype=ordconstn) then
              begin
                if tordconstnode(right).value.svalue and 31<>0 then
                  cg.a_op_const_reg_reg(current_asmdata.CurrAsmList,op,OS_32,tordconstnode(right).value.svalue and 31,hregister1,resultreg)
              end
            else
              begin
                { load shift count in a register if necessary }
                hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,right.resultdef,true);
                cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,op,OS_32,right.location.register,hregister1,resultreg);
              end;
          end;
      end;



begin
   cnotnode:=tm68knotnode;
   cmoddivnode:=tm68kmoddivnode;
   cunaryminusnode:=tm68kunaryminusnode;
   cshlshrnode:=tm68kshlshrnode;
end.
