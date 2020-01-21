{
    Copyright (c) 1998-2008 by Florian Klaempfl

    Generates AVR assembler for math nodes

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
unit navrmat;

{$i fpcdefs.inc}

interface

    uses
      node,nmat,ncgmat;

    type
      tavrnotnode = class(tcgnotnode)
        procedure second_boolean;override;
      end;

      tavrshlshrnode = class(tcgshlshrnode)
        function pass_1: tnode;override;
        procedure second_integer;override;
        procedure second_64bit;override;
      end;

implementation

    uses
      globtype,systems,
      cutils,verbose,globals,constexp,
      symtype,symdef,
      aasmbase,aasmcpu,aasmtai,aasmdata,
      defutil,
      cgbase,cgobj,hlcgobj,cgutils,
      pass_1,pass_2,procinfo,
      ncon,
      cpubase,
      ncgutil,cgcpu;

{*****************************************************************************
                               TAVRNOTNODE
*****************************************************************************}

    procedure tavrnotnode.second_boolean;
      var
        tmpreg,lreg : tregister;
        i : longint;
        falselabel,truelabel,skiplabel: TAsmLabel;
      begin
        if not handle_locjump then
          begin
            secondpass(left);
            { short code? }
            if (left.location.loc in [LOC_SUBSETREG,LOC_CSUBSETREG]) and
              (left.location.sreg.bitlen=1) then
              begin
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_const(A_SBRC,left.location.sreg.subsetreg,left.location.sreg.startbit));
                current_asmdata.getjumplabel(truelabel);
                current_asmdata.getjumplabel(falselabel);
                { sbrc does a jump without an explicit label,
                  if we do not insert skiplabel here and increase its reference count, the optimizer removes the whole true block altogether }
                current_asmdata.getjumplabel(skiplabel);
                skiplabel.increfs;
                location_reset_jump(location,truelabel,falselabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,falselabel);
                cg.a_label(current_asmdata.CurrAsmList,skiplabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,truelabel);
              end
            else if (left.location.loc in [LOC_SUBSETREF,LOC_CSUBSETREF]) and
              (left.location.sref.bitlen=1) and (left.location.sref.bitindexreg=NR_NO) then
              begin
                tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,OS_8);
                hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,u8inttype,osuinttype,left.location.sref.ref,tmpreg);
                current_asmdata.CurrAsmList.Concat(taicpu.op_reg_const(A_SBRC,tmpreg,left.location.sref.startbit));
                current_asmdata.getjumplabel(truelabel);
                current_asmdata.getjumplabel(falselabel);
                { sbrc does a jump without an explicit label,
                  if we do not insert skiplabel here and increase its reference count, the optimizer removes the whole true block altogether }
                current_asmdata.getjumplabel(skiplabel);
                skiplabel.increfs;
                location_reset_jump(location,truelabel,falselabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,falselabel);
                cg.a_label(current_asmdata.CurrAsmList,skiplabel);
                cg.a_jmp_always(current_asmdata.CurrAsmList,truelabel);
              end
            else
              case left.location.loc of
                 LOC_FLAGS :
                   begin
                     location_copy(location,left.location);
                     inverse_flags(location.resflags);
                   end;
                 LOC_SUBSETREG,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF,
                 LOC_REGISTER,LOC_CREGISTER,LOC_REFERENCE,LOC_CREFERENCE :
                   begin
                     hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
                     current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CP,GetDefaultZeroReg,left.location.register));

                     tmpreg:=left.location.register;
                     for i:=2 to tcgsize2size[left.location.size] do
                       begin
                         if i=5 then
                           tmpreg:=left.location.registerhi
                         else
                           tmpreg:=cg.GetNextReg(tmpreg);
                         current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CPC,GetDefaultZeroReg,tmpreg));
                       end;
                     location_reset(location,LOC_FLAGS,OS_NO);
                     location.resflags:=F_EQ;
                  end;
                 else
                   internalerror(2003042401);
               end;
          end;
      end;


{*****************************************************************************
                             TAVRSHLSHRNODE
*****************************************************************************}

    function tavrshlshrnode.pass_1 : tnode;
      begin
        { the avr code generator can handle 64 bit shifts by constants directly }
        if is_constintnode(right) and is_64bit(resultdef) then
          begin
            result:=nil;
            firstpass(left);
            firstpass(right);
            if codegenerror then
              exit;

            expectloc:=LOC_REGISTER;
          end
        else
          Result:=inherited pass_1;
      end;


    procedure tavrshlshrnode.second_integer;
      var
         op : topcg;
         opdef: tdef;
         hcountreg : tregister;
         opsize : tcgsize;
         shiftval : longint;
      begin
        { determine operator }
        case nodetype of
          shln: op:=OP_SHL;
          shrn: op:=OP_SHR;
          else
            internalerror(2013120102);
        end;
        opsize:=left.location.size;
        opdef:=left.resultdef;

        if not(left.location.loc in [LOC_CREGISTER,LOC_REGISTER]) or
          { location_force_reg can be also used to change the size of a register }
          (left.location.size<>opsize) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,opdef,true);
        location_reset(location,LOC_REGISTER,opsize);
        if is_64bit(resultdef) then
          begin
            location.register:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
            location.registerhi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
          end
        else
          location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);

        { shifting by a constant directly coded: }
        if (right.nodetype=ordconstn) then
          begin
             { shl/shr must "wrap around", so use ... and 31 }
             { In TP, "byte/word shl 16 = 0", so no "and 15" in case of
               a 16 bit ALU }
             if tcgsize2size[opsize]<=4 then
               shiftval:=tordconstnode(right).value.uvalue and 31
             else
               shiftval:=tordconstnode(right).value.uvalue and 63;
             if is_64bit(resultdef) then
               cg64.a_op64_const_reg_reg(current_asmdata.CurrAsmList,op,location.size,
                 shiftval,left.location.register64,location.register64)
             else
               hlcg.a_op_const_reg_reg(current_asmdata.CurrAsmList,op,opdef,
                 shiftval,left.location.register,location.register);
          end
        else
          begin
             { load right operators in a register - this
               is done since most target cpu which will use this
               node do not support a shift count in a mem. location (cec)
             }
             hlcg.location_force_reg(current_asmdata.CurrAsmList,right.location,right.resultdef,sinttype,true);
             hlcg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,op,opdef,right.location.register,left.location.register,location.register);
          end;
        { shl/shr nodes return the same type as left, which can be different
          from opdef }
        if opdef<>resultdef then
          begin
            hcountreg:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);
            hlcg.a_load_reg_reg(current_asmdata.CurrAsmList,opdef,resultdef,location.register,hcountreg);
            location.register:=hcountreg;
          end;
      end;


    procedure tavrshlshrnode.second_64bit;
      begin
        second_integer;
        // inherited second_64bit;
      end;

begin
  cnotnode:=tavrnotnode;
  cshlshrnode:=tavrshlshrnode;
end.
