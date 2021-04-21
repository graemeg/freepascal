{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Generate for x86-64 and i386 assembler for type converting nodes

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
unit nx86cnv;

{$i fpcdefs.inc}

interface

    uses
      node,ncgcnv,defutil;

    type
       tx86typeconvnode = class(tcgtypeconvnode)
       protected
         function first_real_to_real : tnode;override;
         { procedure second_int_to_int;override; }
         { procedure second_string_to_string;override; }
         { procedure second_cstring_to_pchar;override; }
         { procedure second_string_to_chararray;override; }
         { procedure second_array_to_pointer;override; }
         { procedure second_pointer_to_array;override; }
         { procedure second_chararray_to_string;override; }
         { procedure second_char_to_string;override; }
         function first_int_to_real: tnode; override;
         procedure second_int_to_real;override;
         { procedure second_real_to_real;override; }
         { procedure second_cord_to_pointer;override; }
         { procedure second_proc_to_procvar;override; }
         { procedure second_bool_to_int;override; }
           procedure second_int_to_bool;override;
         { procedure second_set_to_set;override;  }
         { procedure second_ansistring_to_pchar;override; }
         { procedure second_pchar_to_string;override; }
         { procedure second_class_to_intf;override;  }
         { procedure second_char_to_char;override; }
       end;


implementation

   uses
      verbose,globals,globtype,
      aasmbase,aasmtai,aasmdata,aasmcpu,
      symconst,symdef,
      cgbase,cga,pass_1,pass_2,
      cpuinfo,
      ncnv,
      cpubase,
      cgutils,cgobj,hlcgobj,cgx86,
      tgobj;


    function tx86typeconvnode.first_real_to_real : tnode;
      begin
         first_real_to_real:=nil;
         if use_vectorfpu(resultdef) then
           expectloc:=LOC_MMREGISTER
         else
           expectloc:=LOC_FPUREGISTER;
      end;


    procedure tx86typeconvnode.second_int_to_bool;
      var
{$ifndef cpu64bitalu}
        hreg2,
        hregister : tregister;
        href      : treference;
        i         : integer;
{$endif not cpu64bitalu}
        resflags  : tresflags;
        hlabel    : tasmlabel;
        newsize   : tcgsize;
      begin
         secondpass(left);
         if codegenerror then
          exit;
         { Explicit typecasts from any ordinal type to a boolean type }
         { must not change the ordinal value                          }
         if (nf_explicit in flags) and
            not(left.location.loc in [LOC_FLAGS,LOC_JUMP]) then
           begin
              location_copy(location,left.location);
              newsize:=def_cgsize(resultdef);
              { change of size? change sign only if location is LOC_(C)REGISTER? Then we have to sign/zero-extend }
              if (tcgsize2size[newsize]<>tcgsize2size[left.location.size]) or
                 ((newsize<>left.location.size) and (location.loc in [LOC_REGISTER,LOC_CREGISTER])) then
                hlcg.location_force_reg(current_asmdata.CurrAsmList,location,left.resultdef,resultdef,true)
              else
                location.size:=newsize;
              exit;
           end;

         { Load left node into flag F_NE/F_E }
         resflags:=F_NE;

         if (left.location.loc in [LOC_SUBSETREG,LOC_CSUBSETREG,LOC_SUBSETREF,LOC_CSUBSETREF]) then
           hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);

         case left.location.loc of
            LOC_CREFERENCE,
            LOC_REFERENCE :
              begin
{$ifndef cpu64bitalu}
                if left.location.size in [OS_64,OS_S64{$ifdef cpu16bitalu},OS_32,OS_S32{$endif}] then
                 begin
                   hregister:=cg.getintregister(current_asmdata.CurrAsmList,OS_INT);
                   cg.a_load_ref_reg(current_asmdata.CurrAsmList,OS_INT,OS_INT,left.location.reference,hregister);
                   href:=left.location.reference;
                   for i:=2 to tcgsize2size[left.location.size] div tcgsize2size[OS_INT] do
                     begin
                       inc(href.offset,tcgsize2size[OS_INT]);
                       cg.a_op_ref_reg(current_asmdata.CurrAsmList,OP_OR,OS_INT,href,hregister);
                     end;
                 end
                else
{$endif not cpu64bitalu}
                 begin
                   hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,true);
                   cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_OR,left.location.size,left.location.register,left.location.register);
                 end;
              end;
            LOC_FLAGS :
              begin
                resflags:=left.location.resflags;
              end;
            LOC_REGISTER,LOC_CREGISTER :
              begin
{$if defined(cpu32bitalu)}
                if left.location.size in [OS_64,OS_S64] then
                 begin
                   hregister:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                   cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_32,OS_32,left.location.register64.reglo,hregister);
                   cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_32,left.location.register64.reghi,hregister);
                 end
                else
{$elseif defined(cpu16bitalu)}
                if left.location.size in [OS_64,OS_S64] then
                 begin
                   hregister:=cg.getintregister(current_asmdata.CurrAsmList,OS_16);
                   cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_16,OS_16,left.location.register64.reglo,hregister);
                   cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_16,cg.GetNextReg(left.location.register64.reglo),hregister);
                   cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_16,left.location.register64.reghi,hregister);
                   cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_16,cg.GetNextReg(left.location.register64.reghi),hregister);
                 end
                else
                  if left.location.size in [OS_32,OS_S32] then
                    cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_OR,OS_16,left.location.register,cg.GetNextReg(left.location.register))
                else
{$endif}
                  cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_OR,left.location.size,left.location.register,left.location.register);
              end;
            LOC_JUMP :
              begin
                location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
                location.register:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
                current_asmdata.getjumplabel(hlabel);
                cg.a_label(current_asmdata.CurrAsmList,left.location.truelabel);
                if not(is_cbool(resultdef)) then
                  cg.a_load_const_reg(current_asmdata.CurrAsmList,location.size,1,location.register)
                else
                  cg.a_load_const_reg(current_asmdata.CurrAsmList,location.size,-1,location.register);
                cg.a_jmp_always(current_asmdata.CurrAsmList,hlabel);
                cg.a_label(current_asmdata.CurrAsmList,left.location.falselabel);
                cg.a_load_const_reg(current_asmdata.CurrAsmList,location.size,0,location.register);
                cg.a_label(current_asmdata.CurrAsmList,hlabel);
              end;
            else
              internalerror(10062);
         end;
         if (left.location.loc<>LOC_JUMP) then
           begin
             { load flags to register }
             location_reset(location,LOC_REGISTER,def_cgsize(resultdef));
{$ifndef cpu64bitalu}
              if (location.size in [OS_64,OS_S64]) then
                begin
                  hreg2:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                  cg.g_flags2reg(current_asmdata.CurrAsmList,OS_32,resflags,hreg2);
                  if (is_cbool(resultdef)) then
                    cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NEG,OS_32,hreg2,hreg2);
                  location.register64.reglo:=hreg2;
                  location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList,OS_32);
                  if (is_cbool(resultdef)) then
                    { reglo is either 0 or -1 -> reghi has to become the same }
                    cg.a_load_reg_reg(current_asmdata.CurrAsmList,OS_32,OS_32,location.register64.reglo,location.register64.reghi)
                  else
                    { unsigned }
                    cg.a_load_const_reg(current_asmdata.CurrAsmList,OS_32,0,location.register64.reghi);
                end
             else
{$endif not cpu64bitalu}
               begin
                 location.register:=cg.getintregister(current_asmdata.CurrAsmList,location.size);
                 cg.g_flags2reg(current_asmdata.CurrAsmList,location.size,resflags,location.register);
                 if (is_cbool(resultdef)) then
                   cg.a_op_reg_reg(current_asmdata.CurrAsmList,OP_NEG,location.size,location.register,location.register);
               end
           end;
       end;


    function tx86typeconvnode.first_int_to_real : tnode;

      begin
        first_int_to_real:=nil;
        if (left.resultdef.size<4) then
          begin
            inserttypeconv(left,s32inttype);
            firstpass(left)
          end;

        if use_vectorfpu(resultdef) and
           (torddef(left.resultdef).ordtype = s32bit) then
          expectloc:=LOC_MMREGISTER
        else
          expectloc:=LOC_FPUREGISTER;
      end;


    procedure tx86typeconvnode.second_int_to_real;

      var
         leftref,
         href : treference;
         l1,l2 : tasmlabel;
         op: tasmop;
         opsize: topsize;
         signtested : boolean;
         use_bt: boolean;  { true = use BT (386+), false = use TEST (286-) }
      begin
{$ifdef i8086}
        use_bt:=current_settings.cputype>=cpu_386;
{$else i8086}
        use_bt:=true;
{$endif i8086}
        if not(left.location.loc in [LOC_REGISTER,LOC_CREGISTER,LOC_REFERENCE,LOC_CREFERENCE]) then
          hlcg.location_force_reg(current_asmdata.CurrAsmList,left.location,left.resultdef,left.resultdef,false);
        if use_vectorfpu(resultdef) and
{$ifdef cpu64bitalu}
           ((torddef(left.resultdef).ordtype in [s32bit,s64bit]) or
            ((torddef(left.resultdef).ordtype in [u32bit,u64bit]) and
             (FPUX86_HAS_AVX512F in fpu_capabilities[current_settings.fputype]))
           ) then
{$else cpu64bitalu}
           (torddef(left.resultdef).ordtype=s32bit) then
{$endif cpu64bitalu}
          begin
            location_reset(location,LOC_MMREGISTER,def_cgsize(resultdef));
            location.register:=cg.getmmregister(current_asmdata.CurrAsmList,location.size);
            if UseAVX then
              case location.size of
                OS_F32:
                  if is_signed(left.resultdef) then
                    op:=A_VCVTSI2SS
                  else
                    op:=A_VCVTUSI2SS;
                OS_F64:
                  if is_signed(left.resultdef) then
                    op:=A_VCVTSI2SD
                  else
                    op:=A_VCVTUSI2SD;
                else
                  internalerror(2007120902);
              end
            else
              begin
                { do not use is_signed here as it checks the boundaries instead
                  of the ordtype }
                if not(torddef(left.resultdef).ordtype in [s32bit,s64bit]) then
                  Internalerror(2020101001);
                case location.size of
                  OS_F32:
                    op:=A_CVTSI2SS;
                  OS_F64:
                    op:=A_CVTSI2SD;
                  else
                    internalerror(2007120904);
                end;
              end;

            { don't use left.location.size, because that one may be OS_32/OS_64
              if the lower bound of the orddef >= 0
            }
            case torddef(left.resultdef).ordtype of
              s32bit,u32bit:
                opsize:=S_L;
              s64bit,u64bit:
                opsize:=S_Q;
              else
                internalerror(2007120903);
            end;
            case left.location.loc of
              LOC_REFERENCE,
              LOC_CREFERENCE:
                begin
                  href:=left.location.reference;
                  tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,href);
                  if UseAVX then
                    { VCVTSI2.. requires a second source operand to copy bits 64..127 }
                    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg_reg(op,opsize,href,location.register,location.register))
                  else
                    current_asmdata.CurrAsmList.concat(taicpu.op_ref_reg(op,opsize,href,location.register));
                end;
              LOC_REGISTER,
              LOC_CREGISTER:
                if UseAVX then
                    { VCVTSI2.. requires a second source operand to copy bits 64..127 }
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg_reg(op,opsize,left.location.register,location.register,location.register))
                else
                  current_asmdata.CurrAsmList.concat(taicpu.op_reg_reg(op,opsize,left.location.register,location.register));
              else
                internalerror(2019050708);
            end;
          end
        else
          begin
            location_reset(location,LOC_FPUREGISTER,def_cgsize(resultdef));
            if (left.location.loc=LOC_REGISTER) and (torddef(left.resultdef).ordtype=u64bit) then
              begin
                if use_bt then
                  begin
    {$if defined(cpu64bitalu)}
                    emit_const_reg(A_BT,S_Q,63,left.location.register);
    {$elseif defined(cpu32bitalu)}
                    emit_const_reg(A_BT,S_L,31,left.location.register64.reghi);
    {$elseif defined(cpu16bitalu)}
                    emit_const_reg(A_BT,S_W,15,cg.GetNextReg(left.location.register64.reghi));
    {$endif}
                  end
                else
                  begin
    {$ifdef i8086}
                    emit_const_reg(A_TEST,S_W,aint($8000),cg.GetNextReg(left.location.register64.reghi));
    {$else i8086}
                    internalerror(2013052510);
    {$endif i8086}
                  end;
                signtested:=true;
              end
            else
              signtested:=false;
    
            { We need to load from a reference }
            hlcg.location_force_mem(current_asmdata.CurrAsmList,left.location,left.resultdef);
            { don't change left.location.reference, because if it's a temp we
              need the original location at the end so we can free it }
            leftref:=left.location.reference;
            tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,leftref);

            { For u32bit we need to load it as comp and need to
              make it 64bits }
            if (torddef(left.resultdef).ordtype=u32bit) then
              begin
                tg.GetTemp(current_asmdata.CurrAsmList,8,8,tt_normal,href);
                location_freetemp(current_asmdata.CurrAsmList,left.location);
                cg.a_load_ref_ref(current_asmdata.CurrAsmList,left.location.size,OS_32,leftref,href);
                inc(href.offset,4);
                cg.a_load_const_ref(current_asmdata.CurrAsmList,OS_32,0,href);
                dec(href.offset,4);
                { could be a temp with an offset > 32 bit on x86_64 }
                tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,href);
                leftref:=href;
              end;
    
            { Load from reference to fpu reg }
            case torddef(left.resultdef).ordtype of
              u32bit,
              scurrency,
              s64bit:
                begin
                  current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_FILD,S_IQ,leftref));
                end;
              u64bit:
                begin
                   { unsigned 64 bit ints are harder to handle:
                     we load bits 0..62 and then check bit 63:
                     if it is 1 then we add 2**64 as float.
                     Since 2**64 can be represented exactly, use a single-precision
                     constant to save space. }
                   current_asmdata.getglobaldatalabel(l1);
                   current_asmdata.getjumplabel(l2);
    
                   if not(signtested) then
                     begin
                       if use_bt then
                         begin
           {$if defined(cpu64bitalu) or defined(cpu32bitalu)}
                           inc(leftref.offset,4);
                           emit_const_ref(A_BT,S_L,31,leftref);
                           dec(leftref.offset,4);
           {$elseif defined(cpu16bitalu)}
                           inc(leftref.offset,6);
                           emit_const_ref(A_BT,S_W,15,leftref);
                           dec(leftref.offset,6);
           {$endif}
                         end
                       else
                         begin
           {$ifdef i8086}
                           { reading a byte, instead of word is faster on a true }
                           { 8088, because of the 8-bit data bus }
                           inc(leftref.offset,7);
                           emit_const_ref(A_TEST,S_B,aint($80),leftref);
                           dec(leftref.offset,7);
           {$else i8086}
                           internalerror(2013052511);
           {$endif i8086}
                         end;
                     end;
    
                   current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_FILD,S_IQ,leftref));
                   if use_bt then
                     cg.a_jmp_flags(current_asmdata.CurrAsmList,F_NC,l2)
                   else
                     cg.a_jmp_flags(current_asmdata.CurrAsmList,F_E,l2);
                   new_section(current_asmdata.asmlists[al_typedconsts],sec_rodata_norel,l1.name,const_align(sizeof(pint)));
                   current_asmdata.asmlists[al_typedconsts].concat(Tai_label.Create(l1));
                   { I got this constant from a test program (FK) }
                   current_asmdata.asmlists[al_typedconsts].concat(Tai_const.Create_32bit($5f800000));
                   reference_reset_symbol(href,l1,0,4,[]);
                   tcgx86(cg).make_simple_ref(current_asmdata.CurrAsmList,href);
                   current_asmdata.CurrAsmList.concat(Taicpu.Op_ref(A_FADD,S_FS,href));
                   cg.a_label(current_asmdata.CurrAsmList,l2);
                end
              else
                begin
                  if left.resultdef.size<4 then
                    internalerror(2007120901);
                 current_asmdata.CurrAsmList.concat(taicpu.op_ref(A_FILD,S_IL,leftref));
                end;
            end;
            tcgx86(cg).inc_fpu_stack;
            location.register:=NR_ST;
          end;
        location_freetemp(current_asmdata.CurrAsmList,left.location);
      end;

begin
  ctypeconvnode:=tx86typeconvnode
end.
