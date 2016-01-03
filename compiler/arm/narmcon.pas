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
      ncgcon,cpubase;

    type
      tarmrealconstnode = class(tcgrealconstnode)
        procedure pass_generate_code;override;
      end;

  implementation

    uses
      verbose,
      globtype,globals,
      cpuinfo,
      aasmbase,aasmtai,aasmdata,symdef,
      defutil,
      cgbase,cgutils,
      procinfo,
      ncon;

{*****************************************************************************
                           TARMREALCONSTNODE
*****************************************************************************}

    procedure tarmrealconstnode.pass_generate_code;
      { I suppose the parser/pass_1 must make sure the generated real  }
      { constants are actually supported by the target processor? (JM) }
      const
<<<<<<< HEAD
        floattype2ait:array[tfloattype] of tairealconsttype=
          (aitrealconst_s32bit,aitrealconst_s64bit,aitrealconst_s80bit,aitrealconst_s80bit,aitrealconst_s64comp,aitrealconst_s64comp,aitrealconst_s128bit);
=======
        floattype2ait:array[tfloattype] of taitype=
          (ait_real_32bit,ait_real_64bit,ait_real_80bit,ait_real_80bit,ait_comp_64bit,ait_comp_64bit,ait_real_128bit);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      var
         lastlabel : tasmlabel;
         realait : tairealconsttype;
         hiloswapped : boolean;

      begin
        location_reset_ref(location,LOC_CREFERENCE,def_cgsize(resultdef),4);
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  current_procinfo.aktlocaldata.concat(tai_realconst.create_s80real(value_real,tfloatdef(resultdef).size));
=======
                  current_procinfo.aktlocaldata.concat(Tai_real_80bit.Create(value_real,tfloatdef(resultdef).size));
>>>>>>> graemeg/cpstrnew
=======
                  current_procinfo.aktlocaldata.concat(Tai_real_80bit.Create(value_real,tfloatdef(resultdef).size));
>>>>>>> graemeg/cpstrnew
=======
                  current_procinfo.aktlocaldata.concat(Tai_real_80bit.Create(value_real,tfloatdef(resultdef).size));
>>>>>>> graemeg/cpstrnew
=======
                  current_procinfo.aktlocaldata.concat(Tai_real_80bit.Create(value_real,tfloatdef(resultdef).size));
>>>>>>> origin/cpstrnew

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

begin
  crealconstnode:=tarmrealconstnode;
end.
