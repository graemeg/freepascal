{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Calling conventions for the SPARC

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
 *****************************************************************************}
unit cpupara;

{$i fpcdefs.inc}

interface

    uses
      globtype,
      cclasses,
      aasmtai,aasmdata,
      cpubase,cpuinfo,
      symconst,symbase,symsym,symtype,symdef,paramgr,parabase,cgbase,cgutils;

    type
      tcpuparamanager=class(TParaManager)
        function  push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
        function  get_volatile_registers_int(calloption : tproccalloption):TCpuRegisterSet;override;
        function  get_volatile_registers_fpu(calloption : tproccalloption):TCpuRegisterSet;override;
        function  create_paraloc_info(p : TAbstractProcDef; side: tcallercallee):longint;override;
        function  create_varargs_paraloc_info(p : TAbstractProcDef; varargspara:tvarargsparalist):longint;override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;
=======
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
>>>>>>> graemeg/cpstrnew
=======
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
>>>>>>> graemeg/cpstrnew
=======
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
>>>>>>> graemeg/cpstrnew
=======
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
>>>>>>> origin/cpstrnew
      private
        procedure create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                                             var intparareg,parasize:longint);
      end;

implementation

    uses
      cutils,verbose,systems,
      defutil,
      cgobj;

    type
      tparasupregs = array[0..5] of tsuperregister;
      pparasupregs = ^tparasupregs;
    const
      paraoutsupregs : tparasupregs = (RS_O0,RS_O1,RS_O2,RS_O3,RS_O4,RS_O5);
      parainsupregs  : tparasupregs = (RS_I0,RS_I1,RS_I2,RS_I3,RS_I4,RS_I5);


    function tcpuparamanager.get_volatile_registers_int(calloption : tproccalloption):TCpuRegisterSet;
      begin
        result:=[RS_G1,RS_O0,RS_O1,RS_O2,RS_O3,RS_O4,RS_O5,RS_O6,RS_O7];
      end;


    function tcpuparamanager.get_volatile_registers_fpu(calloption : tproccalloption):TCpuRegisterSet;
      begin
        result:=[RS_F0..RS_F31];
      end;


    { true if a parameter is too large to copy and only the address is pushed }
    function tcpuparamanager.push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;
      begin
        result:=false;
        { var,out,constref always require address }
        if varspez in [vs_var,vs_out,vs_constref] then
          begin
            result:=true;
            exit;
          end;
        case def.typ of
          arraydef:
            result:=(tarraydef(def).highrange>=tarraydef(def).lowrange) or
                             is_open_array(def) or
                             is_array_of_const(def) or
                             is_array_constructor(def);
          recorddef,
          variantdef,
          formaldef :
            result:=true;
          objectdef :
            result:=is_object(def);
          stringdef :
            result:=(tstringdef(def).stringtype in [st_shortstring,st_longstring]);
          procvardef :
            result:=not tprocvardef(def).is_addressonly;
          setdef :
            result:=not is_smallset(def);
        end;
      end;


<<<<<<< HEAD
    function tcpuparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;
=======
    procedure tsparcparamanager.create_funcretloc_info(p : tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;


    function tsparcparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;
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
        paraloc : pcgparalocation;
        retcgsize  : tcgsize;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if set_common_funcretloc_info(p,forcetempdef,retcgsize,result) then
          exit;

        paraloc:=result.add_location;
        { Return in FPU register? }
        if result.def.typ=floatdef then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        result.init;
        result.alignment:=get_para_align(p.proccalloption);
        { void has no location }
        if is_void(def) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          begin
            paraloc:=result.add_location;
            result.size:=OS_NO;
            result.intsize:=0;
            paraloc^.size:=OS_NO;
            paraloc^.loc:=LOC_VOID;
            exit;
          end;
        { Constructors return self instead of a boolean }
        if (p.proctypeoption=potype_constructor) then
          begin
            retcgsize:=OS_ADDR;
            result.intsize:=sizeof(pint);
          end
        else
          begin
            retcgsize:=def_cgsize(def);
            result.intsize:=def.size;
<<<<<<< HEAD
<<<<<<< HEAD
          end;
=======
          begin
            paraloc:=result.add_location;
            result.size:=OS_NO;
            result.intsize:=0;
            paraloc^.size:=OS_NO;
            paraloc^.loc:=LOC_VOID;
            exit;
          end;
<<<<<<< HEAD
        { Constructors return self instead of a boolean }
        if (p.proctypeoption=potype_constructor) then
          begin
            retcgsize:=OS_ADDR;
            result.intsize:=sizeof(pint);
          end
        else
          begin
            retcgsize:=def_cgsize(def);
            result.intsize:=def.size;
          end;
>>>>>>> graemeg/cpstrnew
=======
          end;
>>>>>>> graemeg/cpstrnew
=======
          end;
>>>>>>> origin/cpstrnew
        result.size:=retcgsize;
        { Return is passed as var parameter }
        if ret_in_param(def,p.proccalloption) then
          begin
            paraloc:=result.add_location;
            paraloc^.loc:=LOC_REFERENCE;
            paraloc^.size:=retcgsize;
            exit;
          end;
<<<<<<< HEAD

        paraloc:=result.add_location;
=======
=======
>>>>>>> origin/fixes_2_2
        { Return is passed as var parameter }
        if ret_in_param(p.returndef,p.proccalloption) then
          begin
            p.funcretloc[side].loc:=LOC_REFERENCE;
            p.funcretloc[side].size:=retcgsize;
            exit;
          end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        { Return in FPU register? }
        if def.typ=floatdef then
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
          begin
            paraloc^.loc:=LOC_FPUREGISTER;
            paraloc^.register:=NR_FPU_RESULT_REG;
            if retcgsize=OS_F64 then
              setsubreg(paraloc^.register,R_SUBFD);
            paraloc^.size:=retcgsize;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            paraloc^.def:=result.def;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          end
        else
         { Return in register }
          begin
{$ifndef cpu64bitaddr}
            if retcgsize in [OS_64,OS_S64] then
             begin
               paraloc^.loc:=LOC_REGISTER;
               { high }
<<<<<<< HEAD
               if side=callerside then
=======
               if (side=callerside) or (po_inline in p.procoptions) then
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
                 paraloc^.register:=NR_FUNCTION_RESULT64_HIGH_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_HIGH_REG;
               paraloc^.size:=OS_32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               paraloc^.def:=u32inttype;
               { low }
               paraloc:=result.add_location;
               paraloc^.loc:=LOC_REGISTER;
               if side=callerside then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
               { low }
               paraloc:=result.add_location;
               paraloc^.loc:=LOC_REGISTER;
               if (side=callerside) or (po_inline in p.procoptions) then
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
                 paraloc^.register:=NR_FUNCTION_RESULT64_LOW_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_LOW_REG;
               paraloc^.size:=OS_32;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               paraloc^.def:=u32inttype;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
             end
            else
{$endif not cpu64bitaddr}
             begin
               paraloc^.loc:=LOC_REGISTER;
               paraloc^.size:=retcgsize;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               paraloc^.def:=result.def;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
               if (side=callerside) then
                 paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RESULT_REG,cgsize2subreg(R_INTREGISTER,retcgsize))
               else
                 paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RETURN_REG,cgsize2subreg(R_INTREGISTER,retcgsize));
             end;
          end;
      end;


    procedure tcpuparamanager.create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee;paras:tparalist;
                                                           var intparareg,parasize:longint);
      var
        paraloc      : pcgparalocation;
        i            : integer;
        hp           : tparavarsym;
        paradef      : tdef;
        paracgsize   : tcgsize;
        hparasupregs : pparasupregs;
        paralen      : longint;
      begin
        if side=callerside then
          hparasupregs:=@paraoutsupregs
        else
          hparasupregs:=@parainsupregs;
        for i:=0 to paras.count-1 do
          begin
            hp:=tparavarsym(paras[i]);
            paradef:=hp.vardef;
            { currently only support C-style array of const,
              there should be no location assigned to the vararg array itself }
            if (p.proccalloption in cstylearrayofconst) and
               is_array_of_const(paradef) then
              begin
                paraloc:=hp.paraloc[side].add_location;
                { hack: the paraloc must be valid, but is not actually used }
                paraloc^.loc:=LOC_REGISTER;
                paraloc^.register:=NR_G0;
                paraloc^.size:=OS_ADDR;
                paraloc^.def:=voidpointertype;
                break;
              end;

            if push_addr_param(hp.varspez,paradef,p.proccalloption) then
              begin
                paracgsize:=OS_ADDR;
                paradef:=cpointerdef.getreusable_no_free(paradef);
              end
            else
              begin
                paracgsize:=def_cgsize(paradef);
                { for formaldef }
                if paracgsize=OS_NO then
                  begin
                    paracgsize:=OS_ADDR;
                    paradef:=voidpointertype;
                  end;
              end;
            hp.paraloc[side].reset;
            hp.paraloc[side].size:=paracgsize;
            hp.paraloc[side].def:=paradef;
            if (side = callerside) then
              hp.paraloc[side].Alignment:=std_param_align
            else
              hp.paraloc[side].Alignment:=paradef.alignment;
            paralen:=tcgsize2size[paracgsize];
            hp.paraloc[side].intsize:=paralen;
            while paralen>0 do
              begin
                paraloc:=hp.paraloc[side].add_location;
                { Floats are passed in int registers,
                  We can allocate at maximum 32 bits per register }
                if paracgsize in [OS_64,OS_S64,OS_F32,OS_F64] then
                  begin
                    paraloc^.size:=OS_32;
                    paraloc^.def:=u32inttype;
                  end
                else
                  begin
                    paraloc^.size:=paracgsize;
                    paraloc^.def:=paradef;
                  end;
                { ret in param? }
                if vo_is_funcret in hp.varoptions then
                  begin
                    paraloc^.loc:=LOC_REFERENCE;
                    if side=callerside then
                      paraloc^.reference.index:=NR_STACK_POINTER_REG
                    else
                      paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                    paraloc^.reference.offset:=64;
                  end
                { In case of po_delphi_nested_cc, the parent frame pointer
                  is always passed on the stack. }
                else if (intparareg<=high(tparasupregs)) and
                   (not(vo_is_parentfp in hp.varoptions) or
                    not(po_delphi_nested_cc in p.procoptions)) then
                  begin
                    paraloc^.loc:=LOC_REGISTER;
                    paraloc^.register:=newreg(R_INTREGISTER,hparasupregs^[intparareg],R_SUBWHOLE);
                    inc(intparareg);
                  end
                else
                  begin
                    paraloc^.loc:=LOC_REFERENCE;
                    if side=callerside then
                      paraloc^.reference.index:=NR_STACK_POINTER_REG
                    else
                      paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                    paraloc^.reference.offset:=target_info.first_parm_offset+parasize;

                    if (target_info.endian=endian_big) and
                       (paralen<tcgsize2size[OS_INT]) and
                       (paradef.typ<>recorddef) then
                      inc(paraloc^.reference.offset,4-paralen);

                    { Parameters are aligned at 4 bytes }
                    inc(parasize,align(tcgsize2size[paraloc^.size],sizeof(pint)));
                  end;
                dec(paralen,tcgsize2size[paraloc^.size]);
              end;
          end;
      end;


    function tcpuparamanager.create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;
      var
        intparareg,
        parasize : longint;
      begin
        intparareg:=0;
        parasize:=0;
        { calculate the registers for the normal parameters }
        create_paraloc_info_intern(p,callerside,p.paras,intparareg,parasize);
        { append the varargs }
        create_paraloc_info_intern(p,callerside,varargspara,intparareg,parasize);
        result:=parasize;
      end;



    function tcpuparamanager.create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;
      var
        intparareg,
        parasize : longint;
      begin
        intparareg:=0;
        parasize:=0;
        create_paraloc_info_intern(p,side,p.paras,intparareg,parasize);
        { Create Function result paraloc }
        create_funcretloc_info(p,side);
        { We need to return the size allocated on the stack }
        result:=parasize;
      end;


begin
   ParaManager:=tcpuparamanager.create;
end.
