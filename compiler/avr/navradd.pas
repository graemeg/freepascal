{
    Copyright (c) 2008 by Florian Klaempfl

    Code generation for add nodes on the AVR

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
unit navradd;

{$i fpcdefs.inc}

interface

    uses
       node,ncgadd,cpubase;

    type
       TAVRAddNode = class(tcgaddnode)
       private
         function  GetResFlags(unsigned:Boolean):TResFlags;
       protected
         function pass_1 : tnode;override;
         procedure second_cmpordinal;override;
         procedure second_cmpsmallset;override;
         procedure second_cmp64bit;override;
         procedure second_cmp;
       end;

  implementation

    uses
      globtype,systems,
      cutils,verbose,globals,
      symconst,symdef,paramgr,
      aasmbase,aasmtai,aasmdata,aasmcpu,defutil,htypechk,
      cgbase,cgutils,cgcpu,
      cpuinfo,pass_1,pass_2,procinfo,
      cpupara,
      ncon,nset,nadd,
      ncgutil,tgobj,rgobj,rgcpu,cgobj,cg64f32,
      hlcgobj;

{*****************************************************************************
                               TAVRAddNode
*****************************************************************************}

    function tavraddnode.GetResFlags(unsigned:Boolean):TResFlags;
      begin
        case NodeType of
          equaln:
            GetResFlags:=F_EQ;
          unequaln:
            GetResFlags:=F_NE;
          else
            if not(unsigned) then
              begin
                if nf_swapped in flags then
                  case NodeType of
                    ltn:
                      GetResFlags:=F_NotPossible;
                    lten:
                      GetResFlags:=F_GE;
                    gtn:
                      GetResFlags:=F_LT;
                    gten:
                      GetResFlags:=F_NotPossible;
                    else
                      internalerror(2014082020);
                  end
                else
                  case NodeType of
                    ltn:
                      GetResFlags:=F_LT;
                    lten:
                      GetResFlags:=F_NotPossible;
                    gtn:
                      GetResFlags:=F_NotPossible;
                    gten:
                      GetResFlags:=F_GE;
                    else
                      internalerror(2014082021);
                  end;
              end
            else
              begin
                if nf_swapped in Flags then
                  case NodeType of
                    ltn:
                      GetResFlags:=F_NotPossible;
                    lten:
                      GetResFlags:=F_SH;
                    gtn:
                      GetResFlags:=F_LO;
                    gten:
                      GetResFlags:=F_NotPossible;
                    else
                      internalerror(2014082022);
                  end
                else
                  case NodeType of
                    ltn:
                      GetResFlags:=F_LO;
                    lten:
                      GetResFlags:=F_NotPossible;
                    gtn:
                      GetResFlags:=F_NotPossible;
                    gten:
                      GetResFlags:=F_SH;
                    else
                      internalerror(2014082023);
                  end;
              end;
        end;
      end;


    procedure tavraddnode.second_cmpsmallset;

      procedure gencmp(tmpreg1,tmpreg2 : tregister);
        var
          i : byte;
        begin
          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CP,tmpreg1,tmpreg2));
          for i:=2 to tcgsize2size[left.location.size] do
            begin
              tmpreg1:=cg.GetNextReg(tmpreg1);
              tmpreg2:=cg.GetNextReg(tmpreg2);
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CPC,tmpreg1,tmpreg2));
            end;
        end;

      var
        tmpreg : tregister;
      begin
        pass_left_right;
        location_reset(location,LOC_FLAGS,OS_NO);
        force_reg_left_right(false,false);

        case nodetype of
          equaln:
            begin
              gencmp(left.location.register,right.location.register);
              location.resflags:=F_EQ;
            end;
          unequaln:
            begin
              gencmp(left.location.register,right.location.register);
              location.resflags:=F_NE;
            end;
          lten,
          gten:
            begin
              if (not(nf_swapped in flags) and
                  (nodetype = lten)) or
                 ((nf_swapped in flags) and
                  (nodetype = gten)) then
                swapleftright;
              tmpreg:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
              cg.a_op_reg_reg_reg(current_asmdata.CurrAsmList,OP_AND,location.size,
                left.location.register,right.location.register,tmpreg);
              gencmp(tmpreg,right.location.register);
              location.resflags:=F_EQ;
            end;
          else
            internalerror(2004012401);
        end;
      end;


    procedure tavraddnode.second_cmp;
      var
        unsigned : boolean;
        tmpreg1,tmpreg2 : tregister;
        i : longint;
      begin
        pass_left_right;
        force_reg_left_right(true,true);

        unsigned:=not(is_signed(left.resultdef)) or
                  not(is_signed(right.resultdef));

        if getresflags(unsigned)=F_NotPossible then
          begin
            swapleftright;
            { if we have to swap back and left is a constant, force it to a register because we cannot generate
              the needed code using a constant }
            if (left.location.loc=LOC_CONSTANT) and (left.location.value<>0) then
              hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
          end;

        if (not unsigned) and
          (right.location.loc=LOC_CONSTANT) and
          (right.location.value=0) and
          (getresflags(unsigned) in [F_LT,F_GE]) then
          begin
            { This is a simple sign test, where we can just test the msb }
            tmpreg1:=left.location.register;
            for i:=2 to tcgsize2size[left.location.size] do
              begin
                if i=5 then
                  tmpreg1:=left.location.registerhi
                else
                  tmpreg1:=cg.GetNextReg(tmpreg1);
              end;

            current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CP,tmpreg1,GetDefaultZeroReg));

            location_reset(location,LOC_FLAGS,OS_NO);
            location.resflags:=getresflags(unsigned);

            exit;
          end;

        if right.location.loc=LOC_CONSTANT then
          begin
            { decrease register pressure on registers >= r16 }
            if (right.location.value and $ff)=0 then
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CP,left.location.register,GetDefaultZeroReg))
            else
              begin
                cg.getcpuregister(current_asmdata.CurrAsmList,NR_R26);
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_const(A_LDI,NR_R26,right.location.value and $ff));
                current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CP,left.location.register,NR_R26));
                cg.ungetcpuregister(current_asmdata.CurrAsmList,NR_R26);
              end;
          end
        { on the left side, we allow only a constant if it is 0 }
        else if (left.location.loc=LOC_CONSTANT) and (left.location.value=0) then
          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CP,GetDefaultZeroReg,right.location.register))
        else
          current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CP,left.location.register,right.location.register));

        tmpreg1:=left.location.register;
        tmpreg2:=right.location.register;

        for i:=2 to tcgsize2size[left.location.size] do
          begin
            if i=5 then
              begin
                if left.location.loc<>LOC_CONSTANT then
                  tmpreg1:=left.location.registerhi;
                if right.location.loc<>LOC_CONSTANT then
                  tmpreg2:=right.location.registerhi;
              end
            else
              begin
                if left.location.loc<>LOC_CONSTANT then
                  tmpreg1:=cg.GetNextReg(tmpreg1);
                if right.location.loc<>LOC_CONSTANT then
                  tmpreg2:=cg.GetNextReg(tmpreg2);
              end;
            if right.location.loc=LOC_CONSTANT then
              begin
                { just use R1? }
                if ((right.location.value64 shr ((i-1)*8)) and $ff)=0 then
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CPC,tmpreg1,GetDefaultZeroReg))
                else
                  begin
                    tmpreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_8);
                    cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_8,(right.location.value64 shr ((i-1)*8)) and $ff,tmpreg2);
                    current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CPC,tmpreg1,tmpreg2));
                  end;
              end
            { above it is checked, if left=0, then a constant is allowed }
            else if (left.location.loc=LOC_CONSTANT) and (left.location.value=0) then
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CPC,GetDefaultZeroReg,tmpreg2))
            else
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_CPC,tmpreg1,tmpreg2));
          end;

        location_reset(location,LOC_FLAGS,OS_NO);
        location.resflags:=getresflags(unsigned);
      end;


    procedure tavraddnode.second_cmp64bit;
      begin
        second_cmp;
      end;


    function tavraddnode.pass_1 : tnode;
      begin
        result:=inherited pass_1;
{$ifdef dummy}
        if not(assigned(result)) then
          begin
            unsigned:=not(is_signed(left.resultdef)) or
              not(is_signed(right.resultdef));

            if is_64bit(left.resultdef) and
              ((nodetype in [equaln,unequaln]) or
               (unsigned and (nodetype in [ltn,lten,gtn,gten]))
              ) then
              expectloc:=LOC_FLAGS;
          end;
        { handling boolean expressions }
        if not(assigned(result)) and
           (
             not(is_boolean(left.resultdef)) or
             not(is_boolean(right.resultdef)) or
             is_dynamic_array(left.resultdef)
           ) then
          expectloc:=LOC_FLAGS;
{$endif dummy}
      end;


    procedure tavraddnode.second_cmpordinal;
      begin
        second_cmp;
      end;

begin
  caddnode:=tavraddnode;
end.
