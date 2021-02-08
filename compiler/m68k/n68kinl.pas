{
    Copyright (c) 2015 by the Free Pascal Development team

    Generates Motorola 68k inline nodes

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
unit n68kinl;

{$i fpcdefs.inc}

interface

    uses
      node,ninl,ncginl,symtype,cpubase;

    type
      t68kinlinenode = class(tcgInlineNode)
        function first_abs_real: tnode; override;
        function first_sqr_real: tnode; override;
        function first_sqrt_real: tnode; override;
        {function first_arctan_real: tnode; override;
        function first_ln_real: tnode; override;}
        function first_cos_real: tnode; override;
        function first_sin_real: tnode; override;
        function first_int_real: tnode; override;
        function first_frac_real: tnode; override;
        function first_round_real: tnode; override;
        function first_trunc_real: tnode; override;

        procedure second_length; override;
        procedure second_abs_real; override;
        procedure second_sqr_real; override;
        procedure second_sqrt_real; override;
        {procedure second_arctan_real; override;
        procedure second_ln_real; override;}
        procedure second_cos_real; override;
        procedure second_sin_real; override;
        procedure second_int_real; override;
        procedure second_frac_real; override;
        procedure second_round_real; override;
        procedure second_trunc_real; override;
        {procedure second_prefetch; override;
        procedure second_abs_long; override;}
      protected 
        function second_incdec_tempregdef: tdef; override;
      private
        procedure second_do_operation(op: TAsmOp);
      end;


implementation

    uses
      globtype,verbose,globals,cutils,
      cpuinfo,defutil,symdef,aasmbase,aasmdata,aasmcpu,aasmtai,
      cgbase,cgutils,pass_1,pass_2,symconst,
      ncnv,ncgutil,cgobj,cgcpu,hlcgobj;

{*****************************************************************************
                              t68kinlinenode
*****************************************************************************}

    procedure t68kinlinenode.second_Length;
      var
        lengthlab,zerolab : tasmlabel;
        hregister : tregister;
        lendef : tdef;
        href : treference;
      begin
        secondpass(left);
        if is_shortstring(left.resultdef) then
         begin
           location_copy(location,left.location);
           location.size:=OS_8;
         end
        else
         begin
           //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_length called!')));

           { length in ansi/wide strings and high in dynamic arrays is at offset -sizeof(pint) }
           hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
           current_asmdata.getjumplabel(zerolab);
           hregister:=hlcg.getregisterfordef(current_asmdata.CurrAsmList,resultdef);
           hlcg.a_cmp_const_reg_label(current_asmdata.CurrAsmList,left.resultdef,OC_EQ,0,left.location.register,zerolab);
           { the length of a widestring is a 32 bit unsigned int. Since every
             character occupies 2 bytes, on a 32 bit platform you can express
             the maximum length using 31 bits. On a 64 bit platform, it may be
             32 bits. This means that regardless of the platform, a location
             with size OS_SINT/ossinttype can hold the length without
             overflowing (this code returns an ossinttype value) }
           if is_widestring(left.resultdef) then
             lendef:=u32inttype
           else
             lendef:=ossinttype;
           { volatility of the ansistring/widestring refers to the volatility of the
             string pointer, not of the string data }
           hlcg.reference_reset_base(href,left.resultdef,left.location.register,-lendef.size,ctempposinvalid,lendef.alignment,[]);
           hlcg.a_load_ref_reg(current_asmdata.CurrAsmList,lendef,resultdef,href,hregister);
           if is_widestring(left.resultdef) then
             hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_SHR,resultdef,1,hregister);

           { Dynamic arrays do not have their length attached but their maximum index }
           if is_dynamic_array(left.resultdef) then
             hlcg.a_op_const_reg(current_asmdata.CurrAsmList,OP_ADD,resultdef,1,hregister);
           current_asmdata.getjumplabel(lengthlab);
           hlcg.a_jmp_always(current_asmdata.CurrAsmlist,lengthlab);

           cg.a_label(current_asmdata.CurrAsmList,zerolab);
           hlcg.a_load_const_reg(current_asmdata.CurrAsmList,resultdef,0,hregister);

           cg.a_label(current_asmdata.CurrAsmList,lengthlab);
           location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
           location.register:=hregister;
         end;
      end;

    function t68kinlinenode.first_abs_real : tnode;
      begin
        if FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype] then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited first_abs_real;
      end;

    function t68kinlinenode.first_sqr_real : tnode;
      begin
        if FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype] then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited first_sqr_real;
      end;

    function t68kinlinenode.first_sqrt_real : tnode;
      begin
        if FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype] then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited first_sqrt_real;
      end;

    function t68kinlinenode.first_sin_real : tnode;
      begin
        if FPUM68K_HAS_TRIGONOMETRY in fpu_capabilities[current_settings.fputype] then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited first_sin_real;
      end;

    function t68kinlinenode.first_cos_real : tnode;
      begin
        if FPUM68K_HAS_TRIGONOMETRY in fpu_capabilities[current_settings.fputype] then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited first_cos_real;
      end;

    function t68kinlinenode.first_int_real : tnode;
      begin
        if FPUM68K_HAS_FINTRZ in fpu_capabilities[current_settings.fputype] then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited first_int_real;
      end;

    function t68kinlinenode.first_frac_real : tnode;
      begin
        if FPUM68K_HAS_FINTRZ in fpu_capabilities[current_settings.fputype] then
          begin
            expectloc:=LOC_FPUREGISTER;
            result:=nil;
          end
        else
          result:=inherited first_frac_real;
      end;


    function t68kinlinenode.first_trunc_real : tnode;
      begin
        if (FPUM68K_HAS_FINTRZ in fpu_capabilities[current_settings.fputype]) and
           (resultdef.typ=orddef) and (torddef(resultdef).ordtype in [u8bit,u16bit,s8bit,s16bit,s32bit]) then
          begin
            expectloc:=LOC_REGISTER;
            result:=nil;
          end
        else
          result:=inherited first_trunc_real;
      end;

    function t68kinlinenode.first_round_real : tnode;
      begin
        if (FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype]) and
           (resultdef.typ=orddef) and (torddef(resultdef).ordtype in [u8bit,u16bit,s8bit,s16bit,s32bit]) then
          begin
            expectloc:=LOC_REGISTER;
            result:=nil;
          end
        else
          result:=inherited first_round_real;
      end;


    procedure t68kinlinenode.second_abs_real;
      begin
        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_abs_real called!')));
        second_do_operation(A_FABS);
      end;

    procedure t68kinlinenode.second_sqr_real;
      begin
        secondpass(left);

        if not (FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype]) then
          internalerror(2015022202);

        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_sqr_real called!')));
        hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);
        location_copy(location,left.location);
        if left.location.loc=LOC_CFPUREGISTER then
          begin
            //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_sqr_real called!: left was cfpuregister!')));
            location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
            location.loc := LOC_FPUREGISTER;
            cg.a_loadfpu_reg_reg(current_asmdata.CurrAsmlist,left.location.size,location.size,left.location.register,location.register);
          end;
        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FMUL,fpuregopsize,left.location.register,location.register));
      end;

    procedure t68kinlinenode.second_sqrt_real;
      begin
        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_sqrt_real called!')));
        second_do_operation(A_FSQRT);
      end;

    procedure t68kinlinenode.second_sin_real;
      begin
        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_sin_real called!')));
        second_do_operation(A_FSIN);
      end;

    procedure t68kinlinenode.second_cos_real;
      begin
        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_cos_real called!')));
        second_do_operation(A_FCOS);
      end;

    procedure t68kinlinenode.second_int_real;
      begin
        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_int_real called!')));
        second_do_operation(A_FINTRZ);
      end;

    procedure t68kinlinenode.second_do_operation(op: TAsmOp);
      var
        href: TReference;
      begin
        secondpass(left);

        if not (FPUM68K_HAS_HARDWARE in fpu_capabilities[current_settings.fputype]) then
          internalerror(2015022204);

        location_reset(location,LOC_FPUREGISTER,left.location.size);

        case left.location.loc of
          LOC_FPUREGISTER:
            begin
              location.register:=left.location.register;
              current_asmdata.CurrAsmList.concat(taicpu.op_reg(op,fpuregopsize,location.register))
            end;
          LOC_CFPUREGISTER:
            begin
              location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,fpuregopsize,left.location.register,location.register));
            end;
          LOC_REFERENCE,LOC_CREFERENCE:
            begin
              location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
              href:=left.location.reference;
              tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,current_settings.fputype = fpu_coldfire);
              current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,tcgsize2opsize[left.location.size],href,location.register));
            end;
          else
            internalerror(2015022205);
        end;
      end;

    procedure t68kinlinenode.second_frac_real;
      var
        href: TReference;
        hreg: TRegister;
      begin
        secondpass(left);

        if not (FPUM68K_HAS_FINTRZ in fpu_capabilities[current_settings.fputype]) then
          internalerror(2017052102);

        location_reset(location,LOC_FPUREGISTER,left.location.size);

        case left.location.loc of
          LOC_FPUREGISTER,LOC_CFPUREGISTER:
            begin
              hreg:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
              location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
              cg.a_loadfpu_reg_reg(current_asmdata.CurrAsmlist,left.location.size,location.size,left.location.register,location.register);
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FINTRZ,fpuregopsize,left.location.register,hreg));
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FSUB,fpuregopsize,hreg,location.register));
            end;
          LOC_REFERENCE,LOC_CREFERENCE:
            begin
              hreg:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
              location.register:=cg.getfpuregister(current_asmdata.CurrAsmList,location.size);
              href:=left.location.reference;
              tcg68k(cg).fixref(current_asmdata.CurrAsmList,href,current_settings.fputype = fpu_coldfire);
              cg.a_loadfpu_ref_reg(current_asmdata.CurrAsmlist,left.location.size,location.size,href,location.register);
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FINTRZ,fpuregopsize,location.register,hreg));
              current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FSUB,fpuregopsize,hreg,location.register));
            end;
          else
            internalerror(2017052101);
        end;
      end;

    procedure t68kinlinenode.second_round_real;
      var
        size: tcgsize;
      begin
        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_round_real called!')));
        secondpass(left);
        size:=def_cgsize(resultdef);

        hlcg.location_force_fpureg(current_asmdata.CurrAsmList,left.location,left.resultdef,true);

        location_reset(location,LOC_REGISTER,size);
        location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);

        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FMOVE,tcgsize2opsize[size],left.location.register,location.register));
      end;

    procedure t68kinlinenode.second_trunc_real;
      var
        hreg: TRegister;
        size: tcgsize;
      begin
        //current_asmdata.CurrAsmList.concat(tai_comment.create(strpnew('second_trunc_real called!')));
        second_do_operation(A_FINTRZ);
        size:=def_cgsize(resultdef);

        hreg:=location.register;
        location_reset(location,LOC_REGISTER,size);
        location.register:=hlcg.getintregister(current_asmdata.CurrAsmList,resultdef);

        current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(A_FMOVE,tcgsize2opsize[size],hreg,location.register));
      end;


      { ideas for second_abs_long (KB) }

      { This is probably faster on 68000 than the generic implementation,
        because shifting is slow on the original 68000, maybe also on the 68020?
        Also needs to be tested on 040/060. This can also work on a CF.
        input - d0, output - d2
        move.l d0,d2
        btst   #31,d2
        sne    d1
        extb.l d1 (or ext.w + ext.l on 68000)
        eor.l  d1,d2
        sub.l  d1,d2
      }

      { Solution using bitfield extraction, we don't support the necessary asm
        construct for this yet, probably this is the fastest on 020, slower on
        040/060 than the one above, doesn't work on '000 or CF.
        input - d0, output - d2
        move.l  d0,d2
        bfexts  d0[0:1],d1
        eor.l   d1,d2
        sub.l   d1,d2
      }

      function t68kinlinenode.second_incdec_tempregdef: tdef;
        begin
          { this kludge results in the increment/decrement value of inc/dec to be loaded
            always in a datareg, regardless of the target type. This results in significantly
            better code on m68k, where if the inc/decrement is loaded to an address register
            for pointers, the compiler will generate a bunch of useless data<->address register
            shuffling, as it cannot do some operations on address registers (like shifting
            or multiplication) (KB) }
          second_incdec_tempregdef:=cgsize_orddef(def_cgsize(left.resultdef));
        end;

begin
  cinlinenode:=t68kinlinenode;
end.
