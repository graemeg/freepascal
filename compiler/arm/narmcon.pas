{
    Copyright (c) 2005 by Florian Klaempfl

    Code generation for const nodes on the ARM

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
unit narmcon;

{$i fpcdefs.inc}

interface

    uses
      node,ncgcon,cpubase;

    type
      tarmrealconstnode = class(tcgrealconstnode)
        function pass_1 : tnode;override;
        procedure pass_generate_code;override;
      end;

  implementation

    uses
      verbose,
      globtype,globals,
      cpuinfo,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      symdef,
      defutil,
      cgbase,cgutils,cgobj,
      procinfo,
      ncon;

{*****************************************************************************
                           TARMREALCONSTNODE
*****************************************************************************}

    function tarmrealconstnode.pass_1 : tnode;
      begin
        result:=nil;
        if (FPUARM_HAS_VMOV_CONST in fpu_capabilities[current_settings.fputype]) and
           IsVFPFloatImmediate(tfloatdef(resultdef).floattype,value_real) and
           ((tfloatdef(resultdef).floattype=s32real) or
            (FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[init_settings.fputype])) then
           expectloc:=LOC_MMREGISTER
         else
           expectloc:=LOC_CREFERENCE;
      end;


    procedure tarmrealconstnode.pass_generate_code;
      { I suppose the parser/pass_1 must make sure the generated real  }
      { constants are actually supported by the target processor? (JM) }
      const
        floattype2ait:array[tfloattype] of tairealconsttype=
          (aitrealconst_s32bit,aitrealconst_s64bit,aitrealconst_s80bit,aitrealconst_s80bit,aitrealconst_s64comp,aitrealconst_s64comp,aitrealconst_s128bit);
      var
         lastlabel : tasmlabel;
         realait : tairealconsttype;
         hiloswapped : boolean;
         pf : TOpPostfix;

      begin
        if (FPUARM_HAS_VMOV_CONST in fpu_capabilities[current_settings.fputype]) and
          IsVFPFloatImmediate(tfloatdef(resultdef).floattype,value_real) and
           ((tfloatdef(resultdef).floattype=s32real) or
            (FPUARM_HAS_VFP_DOUBLE in fpu_capabilities[init_settings.fputype])) then
          begin
            location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
            if tfloatdef(resultdef).floattype=s32real then
              pf:=PF_F32
            else
              pf:=PF_F64;
            current_asmdata.CurrAsmList.concat(setoppostfix(taicpu.op_reg_realconst(A_VMOV,
               location.register,value_real),pf));
          end
        else
          begin
            location_reset_ref(location,LOC_CREFERENCE,def_cgsize(resultdef),4,[]);
            lastlabel:=nil;
            realait:=floattype2ait[tfloatdef(resultdef).floattype];
            hiloswapped:=is_double_hilo_swapped;
            { const already used ? }
            if not assigned(lab_real) then
              begin
                current_asmdata.getjumplabel(lastlabel);
                lab_real:=lastlabel;
                current_procinfo.aktlocaldata.concat(Tai_label.Create(lastlabel));
                location.reference.symboldata:=current_procinfo.aktlocaldata.last;
                case realait of
                  aitrealconst_s32bit :
                    begin
                      current_procinfo.aktlocaldata.concat(tai_realconst.create_s32real(ts32real(value_real)));
                      { range checking? }
                      if floating_point_range_check_error and
                        (tai_realconst(current_procinfo.aktlocaldata.last).value.s32val=MathInf.Value) then
                        Message(parser_e_range_check_error);
                    end;

                  aitrealconst_s64bit :
                    begin
                      if hiloswapped then
                        current_procinfo.aktlocaldata.concat(tai_realconst.create_s64real_hiloswapped(ts64real(value_real)))
                      else
                        current_procinfo.aktlocaldata.concat(tai_realconst.create_s64real(ts64real(value_real)));

                      { range checking? }
                      if floating_point_range_check_error and
                        (tai_realconst(current_procinfo.aktlocaldata.last).value.s64val=MathInf.Value) then
                        Message(parser_e_range_check_error);
                   end;

                  aitrealconst_s80bit :
                    begin
                      current_procinfo.aktlocaldata.concat(tai_realconst.create_s80real(value_real,tfloatdef(resultdef).size));

                      { range checking? }
                      if floating_point_range_check_error and
                        (tai_realconst(current_procinfo.aktlocaldata.last).value.s80val=MathInf.Value) then
                        Message(parser_e_range_check_error);
                    end;
{$ifdef cpufloat128}
                  aitrealconst_s128bit :
                    begin
                      current_procinfo.aktlocaldata.concat(tai_realconst.create_s128real(value_real));

                      { range checking? }
                      if floating_point_range_check_error and
                        (tai_realconst(current_procinfo.aktlocaldata.last).value.s128val=MathInf.Value) then
                        Message(parser_e_range_check_error);
                    end;
{$endif cpufloat128}

                  { the round is necessary for native compilers where comp isn't a float }
                  aitrealconst_s64comp :
                    if (value_real>9223372036854775807.0) or (value_real<-9223372036854775808.0) then
                      message(parser_e_range_check_error)
                    else
                      current_procinfo.aktlocaldata.concat(tai_realconst.create_s64compreal(round(value_real)));
                else
                  internalerror(2005092401);
                end;
              end;
            location.reference.symbol:=lab_real;
            location.reference.base:=NR_R15;
          end;
      end;

begin
  crealconstnode:=tarmrealconstnode;
end.
