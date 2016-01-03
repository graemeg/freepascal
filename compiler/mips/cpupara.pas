{
    Copyright (c) 1998-2002 by Florian Klaempfl and David Zhang

    Calling conventions for the MIPSEL

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
      aasmtai,
      cpubase,cpuinfo,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      symconst,symbase,symsym,symtype,symdef,paramgr,parabase,cgbase,cgutils;

    const
      { The value below is OK for O32 and N32 calling conventions }
      MIPS_MAX_REGISTERS_USED_IN_CALL = 6;

      { All ABI seem to start with $4 i.e. $a0 }
      MIPS_FIRST_REGISTER_USED_IN_CALL = RS_R4;
      { O32 ABI uses $a0 to $a3, i.e R4 to R7 }
      MIPS_LAST_REGISTER_USED_IN_CALL_ABI_O32 = RS_R7;
      { N32 ABI uses also R8 and R9 }
      MIPS_LAST_REGISTER_USED_IN_CALL_ABI_N32 = RS_R9;
      { The calculation below is based on the assumption
        that all registers used for ABI calls are
        ordered and follow each other }
      MIPS_NB_REGISTERS_USED_IN_CALL_O32 =
        MIPS_LAST_REGISTER_USED_IN_CALL_ABI_O32
        - MIPS_FIRST_REGISTER_USED_IN_CALL + 1;
      MIPS_NB_REGISTERS_USED_IN_CALL_N32 =
        MIPS_LAST_REGISTER_USED_IN_CALL_ABI_N32
        - MIPS_FIRST_REGISTER_USED_IN_CALL + 1;


      { Set O32 ABI as default }
    const
      mips_nb_used_registers  : longint = MIPS_NB_REGISTERS_USED_IN_CALL_O32;

      { Might need to be changed if we support N64 ABI later }
      mips_sizeof_register_param : longint = 4;

    type
      tparasupregs = array[0..MIPS_MAX_REGISTERS_USED_IN_CALL-1] of tsuperregister;
      tparasupregsused = array[0..MIPS_MAX_REGISTERS_USED_IN_CALL-1] of boolean;

    const

      parasupregs : tparasupregs = (RS_R4, RS_R5, RS_R6, RS_R7, RS_R8, RS_R9);

    type
      tcpuparamanager=class(TParaManager)
        function  push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
        function  get_volatile_registers_int(calloption : tproccalloption):TCpuRegisterSet;override;
        function  get_volatile_registers_fpu(calloption : tproccalloption):TCpuRegisterSet;override;
        function  create_paraloc_info(p : TAbstractProcDef; side: tcallercallee):longint;override;
        function  create_varargs_paraloc_info(p : TAbstractProcDef; varargspara:tvarargsparalist):longint;override;
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;
        function  param_use_paraloc(const cgpara: tcgpara): boolean; override;
      private
        intparareg,
        intparasize : longint;
        can_use_float : boolean;
        function is_abi_record(def: tdef): boolean;
        procedure create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      symconst,symbase,symsym,symtype,symdef,paramgr,parabase,cgbase;

    type
      TMIPSELParaManager=class(TParaManager)
        function  push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
        function  get_volatile_registers_int(calloption : tproccalloption):TCpuRegisterSet;override;
        function  get_volatile_registers_fpu(calloption : tproccalloption):TCpuRegisterSet;override;
        {Returns a structure giving the information on the storage of the parameter
        (which must be an integer parameter)
        @param(nr Parameter number of routine, starting from 1)}
        procedure getintparaloc(calloption : tproccalloption; nr : longint;var cgpara : TCGPara);override;
        function  create_paraloc_info(p : TAbstractProcDef; side: tcallercallee):longint;override;
        function  create_varargs_paraloc_info(p : TAbstractProcDef; varargspara:tvarargsparalist):longint;override;
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
      private
        procedure create_funcretloc_info(p : tabstractprocdef; side: tcallercallee);
        procedure create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                                             var intparareg,parasize:longint);
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
      end;

implementation

    uses
      cutils,verbose,systems,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      defutil, cpupi, procinfo,
      cgobj;



    function tcpuparamanager.get_volatile_registers_int(calloption : tproccalloption):TCpuRegisterSet;
      begin
        { O32 ABI values }
        result:=[RS_R1..RS_R15,RS_R24..RS_R25,RS_R31];
      end;


    function tcpuparamanager.get_volatile_registers_fpu(calloption : tproccalloption):TCpuRegisterSet;
      begin
        { O32 ABI values }
        result:=[RS_F0..RS_F19];
      end;


    { whether "def" must be treated as record when used as function result,
      i.e. its address passed in a0 }
    function tcpuparamanager.is_abi_record(def: tdef): boolean;
      begin
        result:=(def.typ=recorddef) or
          ((def.typ=procvardef) and not tprocvardef(def).is_addressonly);
      end;


    function tcpuparamanager.param_use_paraloc(const cgpara: tcgpara): boolean;
      var
        paraloc: pcgparalocation;
      begin
        paraloc:=cgpara.location;
        if not assigned(paraloc) then
          internalerror(200410102);
        result:=(paraloc^.loc=LOC_REFERENCE) and (paraloc^.next=nil);
      end;


    { true if a parameter is too large to copy and only the address is pushed }
    function tcpuparamanager.push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      defutil,
      cgutils,cgobj;

    type
      tparasupregs = array[0..5] of tsuperregister;
      pparasupregs = ^tparasupregs;
    const
      paraoutsupregs : tparasupregs = (RS_R4, RS_R5, RS_R6, RS_R7, RS_R8, RS_R9);
      parainsupregs  : tparasupregs = (RS_R4, RS_R5, RS_R6, RS_R7, RS_R8, RS_R9);


    function TMIPSELParaManager.get_volatile_registers_int(calloption : tproccalloption):TCpuRegisterSet;
      begin
        result:=[RS_R16..RS_R23];
      end;


    function tMIPSELparamanager.get_volatile_registers_fpu(calloption : tproccalloption):TCpuRegisterSet;
      begin
        result:=[RS_F0..RS_F31];
      end;


    procedure TMIPSELParaManager.GetIntParaLoc(calloption : tproccalloption; nr : longint;var cgpara : tcgpara);
      var
        paraloc : pcgparalocation;
      begin
        if nr<1 then
          InternalError(2002100806);
        cgpara.reset;
        cgpara.size:=OS_INT;
        cgpara.intsize:=tcgsize2size[OS_INT];
        cgpara.alignment:=std_param_align;
        paraloc:=cgpara.add_location;
        with paraloc^ do
          begin
            { The six first parameters are passed into registers } {MIPS first four}
            dec(nr);
            if nr<6 then //MIPSEL nr<6
              begin
                loc:=LOC_REGISTER;
                register:=newreg(R_INTREGISTER,(RS_R4+nr),R_SUBWHOLE);
              end
            else
              begin
                { The other parameters are passed on the stack }
                loc:=LOC_REFERENCE;
                reference.index:=NR_STACK_POINTER_REG;
                reference.offset:=92+(nr-6)*4;
              end;
            size:=OS_INT;
          end;
      end;

    { true if a parameter is too large to copy and only the address is pushed }
    function tMIPSELparamanager.push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;
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
        result:=false;
        { var,out,constref always require address }
        if varspez in [vs_var,vs_out,vs_constref] then
          begin
            result:=true;
            exit;
          end;
        case def.typ of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          recorddef:
            { According to 032 ABI we should have }
            result:=false;
          arraydef:
            result:=true; {(tarraydef(def).highrange>=tarraydef(def).lowrange) or
                            is_open_array(def) or
                            is_array_of_const(def) or
                            is_array_constructor(def);}
          variantdef,
          formaldef :
            result:=true;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          recorddef,
          arraydef,
          variantdef,
          formaldef :
            push_addr_param:=true;
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
          objectdef :
            result:=is_object(def);
          stringdef :
            result:=(tstringdef(def).stringtype in [st_shortstring,st_longstring]);
          procvardef :
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            { If we always push records by value, we have to handle methodpointers that way too. }
            result:=false; {not tprocvardef(def).is_addressonly;}
=======
            result:=not tprocvardef(def).is_addressonly;
>>>>>>> graemeg/cpstrnew
=======
            result:=not tprocvardef(def).is_addressonly;
>>>>>>> graemeg/cpstrnew
=======
            result:=not tprocvardef(def).is_addressonly;
>>>>>>> graemeg/cpstrnew
=======
            result:=not tprocvardef(def).is_addressonly;
>>>>>>> origin/cpstrnew
          setdef :
            result:=not(is_smallset(def));
        end;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function tcpuparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;
      var
        paraloc : pcgparalocation;
        retcgsize  : tcgsize;
        retdef : tdef;
      begin
        if set_common_funcretloc_info(p,forcetempdef,retcgsize,result) then
          begin
            { Return is passed as var parameter,
              in this case we use the first register R4 for it }
            if assigned(forcetempdef) then
              retdef:=forcetempdef
            else
              retdef:=p.returndef;
            if ret_in_param(retdef,p) and
              is_abi_record(retdef) then
              begin
                if intparareg=0 then
                  inc(intparareg);
              end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure tMIPSELparamanager.create_funcretloc_info(p : tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;


    function tMIPSELparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;
      var
        paraloc : pcgparalocation;
        retcgsize  : tcgsize;
      begin
        result.init;
        result.alignment:=get_para_align(p.proccalloption);
        { void has no location }
        if is_void(def) then
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
          end;
        result.size:=retcgsize;
        { Return is passed as var parameter }
        if ret_in_param(def,p.proccalloption) then
          begin
            paraloc:=result.add_location;
            paraloc^.loc:=LOC_REFERENCE;
            paraloc^.size:=retcgsize;
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
            exit;
          end;

        paraloc:=result.add_location;
        { Return in FPU register? }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if result.def.typ=floatdef then
=======
        if p.returndef.typ=floatdef then
>>>>>>> graemeg/cpstrnew
=======
        if p.returndef.typ=floatdef then
>>>>>>> graemeg/cpstrnew
=======
        if p.returndef.typ=floatdef then
>>>>>>> graemeg/cpstrnew
=======
        if p.returndef.typ=floatdef then
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef cpu64bitalu}
            if retcgsize in [OS_64,OS_S64] then
             begin
               { low }
               paraloc^.loc:=LOC_REGISTER;
               if side=callerside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_LOW_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_LOW_REG;
               paraloc^.size:=OS_32;
               paraloc^.def:=u32inttype;
               { high }
               paraloc:=result.add_location;
               paraloc^.loc:=LOC_REGISTER;
               if side=callerside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_HIGH_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_HIGH_REG;
               paraloc^.size:=OS_32;
               paraloc^.def:=u32inttype;
             end
            else
{$endif cpu64bitalu}
             begin
               paraloc^.loc:=LOC_REGISTER;
               paraloc^.size:=retcgsize;
               paraloc^.def:=result.def;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef cpu64bit}
            if retcgsize in [OS_64,OS_S64] then
             begin
               { high }
               paraloc^.loc:=LOC_REGISTER;
               if side=callerside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_HIGH_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_HIGH_REG;
               paraloc^.size:=OS_32;
               { low }
               paraloc:=result.add_location;
               paraloc^.loc:=LOC_REGISTER;
               if side=callerside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_LOW_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_LOW_REG;
               paraloc^.size:=OS_32;
             end
            else
{$endif cpu64bit}
             begin
               paraloc^.loc:=LOC_REGISTER;
               paraloc^.size:=retcgsize;
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
               if side=callerside then
                 paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RESULT_REG,cgsize2subreg(R_INTREGISTER,retcgsize))
               else
                 paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RETURN_REG,cgsize2subreg(R_INTREGISTER,retcgsize));
             end;
          end
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    procedure tcpuparamanager.create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee;paras:tparalist);
      var
        paraloc      : pcgparalocation;
        i,j          : integer;
        hp           : tparavarsym;
        paracgsize   : tcgsize;
        paralen      : longint;
        locdef       : tdef;
        paradef      : tdef;
        fpparareg    : integer;
        reg          : tsuperregister;
        alignment    : longint;
        tmp          : longint;
        firstparaloc : boolean;
        reg_and_stack: boolean;
      begin
        fpparareg := 0;
        for i:=0 to paras.count-1 do
          begin
            hp:=tparavarsym(paras[i]);
            paradef := hp.vardef;

            { currently only support C-style array of const }
             if (p.proccalloption in cstylearrayofconst) and
               is_array_of_const(paradef) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    var
      param_offset:array[0..20] of ^Aint;

    procedure tMIPSELparamanager.create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee;paras:tparalist;
                                                           var intparareg,parasize:longint);
      var
        paraloc      : pcgparalocation;
        i            : integer;
        hp           : tparavarsym;
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

            param_offset[i] := Nil;
            hp:=tparavarsym(paras[i]);
            { currently only support C-style array of const,
              there should be no location assigned to the vararg array itself }
            if (p.proccalloption in [pocall_cdecl,pocall_cppdecl]) and
               is_array_of_const(hp.vardef) then
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
                paraloc:=hp.paraloc[side].add_location;
                { hack: the paraloc must be valid, but is not actually used }
                paraloc^.loc:=LOC_REGISTER;
                paraloc^.register:=NR_R0;
                paraloc^.size:=OS_ADDR;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                paraloc^.def:=voidpointertype;
                break;
              end;

            if push_addr_param(hp.varspez,paradef,p.proccalloption) then
              begin
                paracgsize := OS_ADDR;
                paralen := tcgsize2size[paracgsize];
                paradef := cpointerdef.getreusable_no_free(paradef);
              end
            else
              begin
                paracgsize := def_cgsize(paradef);
                { for things like formaldef }
                if (paracgsize=OS_NO) and (paradef.typ <> recorddef) then
                  begin
                    paracgsize:=OS_ADDR;
                    paradef:=voidpointertype;
                  end;
                if not is_special_array(paradef) then
                  paralen := paradef.size
                else
                  paralen := tcgsize2size[paracgsize];
              end;

            if (paracgsize in [OS_64, OS_S64, OS_F64]) or (paradef.alignment = 8) then
              alignment := 8
            else
              alignment := 4;
            //writeln('para: ',hp.Name,' typ=',hp.vardef.typ,' paracgsize=',paracgsize,' align=',hp.vardef.alignment);
            hp.paraloc[side].reset;
            hp.paraloc[side].Alignment:=alignment;
            locdef:=paradef;
            if (paracgsize=OS_NO) or
              { Ordinals on caller side must be promoted to machine word }
              ((target_info.endian=endian_big) and     // applies to little-endian too?
              (paradef.typ<>recorddef) and
              (side=callerside) and
              (paralen<tcgsize2size[OS_INT]))then
              begin
                if is_signed(paradef) then
                  begin
                    paracgsize:=OS_S32;
                    locdef:=s32inttype;
                  end
                else
                  begin
                    paracgsize:=OS_32;
                    locdef:=u32inttype;
                  end;
                paralen:=align(paralen,4);
              end
            else
              paralen:=tcgsize2size[paracgsize];
            hp.paraloc[side].intsize:=paralen;
            hp.paraloc[side].size:=paracgsize;
            hp.paraloc[side].def:=paradef;

            if (paralen=0) then
              if (paradef.typ=recorddef) then
                begin
                  paraloc:=hp.paraloc[side].add_location;
                  paraloc^.loc:=LOC_VOID;
                end
              else
                internalerror(2013020601);

            { check the alignment, mips O32ABI require a nature alignment  }
            tmp := align(intparasize, alignment) - intparasize;
            while tmp > 0 do
              begin
                inc(intparareg);
                inc(intparasize,4);
                dec(tmp,4);
              end;

            { any non-float args will disable the use the floating regs }
            { up to two fp args }
            if (not(paracgsize in [OS_F32, OS_F64])) or (fpparareg = 2) then
              can_use_float := false;

            firstparaloc:=true;
            { Is parameter split between stack and registers? }
            reg_and_stack:=(side=calleeside) and
              (paralen+intparasize>16) and (intparasize<16);
            while paralen>0 do
              begin
                paraloc:=hp.paraloc[side].add_location;
                { We can allocate at maximum 32 bits per register }
                if (paracgsize in [OS_64,OS_S64]) or
                   ((paracgsize in [OS_F32,OS_F64]) and
                     not(can_use_float)) then
                  begin
                    paraloc^.size:=OS_32;
                    paraloc^.def:=u32inttype;
                  end
                else
                  begin
                    paraloc^.size:=paracgsize;
                    paraloc^.def:=locdef;
                  end;

                { ret in param? }
                if (vo_is_funcret in hp.varoptions) and
                  is_abi_record(hp.vardef) then
                  begin
                    { This should be the first parameter }
                    //if (intparareg<>1) then
                    //  Comment(V_Warning,'intparareg should be one for funcret in tcpuparamanager.create_paraloc_info_intern');
                    paraloc^.loc:=LOC_REGISTER;
                    paraloc^.register:=newreg(R_INTREGISTER,parasupregs[0],R_SUBWHOLE);
                    inc(intparasize,align(tcgsize2size[paraloc^.size],sizeof(aint)));
                  end
                { "In case of po_delphi_nested_cc, the parent frame pointer
                  is always passed on the stack". On other targets it is
                  used to provide caller-side stack cleanup and prevent stackframe
                  optimization. For MIPS this does not matter. }
                else if (intparareg<mips_nb_used_registers) and
                   (not reg_and_stack) {and
                   (not(vo_is_parentfp in hp.varoptions) or
                    not(po_delphi_nested_cc in p.procoptions))} then
                  begin
                    if (can_use_float) then
                      begin
                        paraloc^.loc:=LOC_FPUREGISTER;
                        if (fpparareg = 0) then
                          reg := RS_F12
                        else
                          reg := RS_F14;
                        if (paraloc^.size = OS_F64) then
                          begin
                            paraloc^.register:=newreg(R_FPUREGISTER, reg, R_SUBFD);
                            inc(fpparareg);
                            inc(intparareg);
                            inc(intparareg);
                            inc(intparasize,8);
                          end
                        else
                          begin
                            paraloc^.register:=newreg(R_FPUREGISTER, reg, R_SUBFS);
                            inc(fpparareg);
                            inc(intparareg);
                            inc(intparasize,sizeof(aint));
                          end;
                      end
                    else { not can use float }
                     begin
                       paraloc^.loc:=LOC_REGISTER;
                       paraloc^.register:=newreg(R_INTREGISTER,parasupregs[intparareg],R_SUBWHOLE);

                       { big-endian targets require that record data stored in parameter
                         registers is left-aligned }
                       if (target_info.endian=endian_big) and
                          (paradef.typ=recorddef) and
                          (paralen<tcgsize2size[OS_INT]) then
                         begin
                           paraloc^.shiftval := (sizeof(aint)-tcgsize2size[paraloc^.size])*(-8);
                           paraloc^.size := OS_INT;
                           paraloc^.def := osuinttype;
                         end;
                       inc(intparareg);
                       inc(intparasize,align(tcgsize2size[paraloc^.size],mips_sizeof_register_param));
                     end;
                  end
                else
                  begin
                    if reg_and_stack then
                      begin
                        for j:=intparareg to mips_nb_used_registers-1 do
                          tmipsprocinfo(current_procinfo).register_used[j]:=true;
                        { all registers used now }
                        intparareg:=mips_nb_used_registers;
                      end;
                    paraloc^.loc:=LOC_REFERENCE;
                    if (paradef.typ=floatdef) then
                      paraloc^.size:=int_float_cgsize(paralen)
                    else
                      paraloc^.size:=int_cgsize(paralen);
                    paraloc^.def:=get_paraloc_def(locdef,paralen,firstparaloc);

                    if side=callerside then
                      begin
                        paraloc^.reference.index := NR_STACK_POINTER_REG;
                        paraloc^.reference.offset:=intparasize;
                      end
                    else
                      begin
                        if (po_nostackframe in p.procoptions) then
                          paraloc^.reference.index := NR_STACK_POINTER_REG
                        else
                          begin
                            paraloc^.reference.index := NR_FRAME_POINTER_REG;
                            if assigned(current_procinfo) then
                              include(current_procinfo.flags,pi_needs_stackframe);
                          end;
                        paraloc^.reference.offset:=intparasize;

                        if (target_info.endian=endian_big) and
                           (paralen<tcgsize2size[OS_INT]) and
                           (paradef.typ<>recorddef) then
                          inc(paraloc^.reference.offset,4-paralen);
                      end;
                    inc(intparasize,align(paralen,mips_sizeof_register_param));
                    paralen:=0;
                  end;
                dec(paralen,tcgsize2size[paraloc^.size]);
                firstparaloc:=false;
              end;
          end;
        { O32 ABI reqires at least 16 bytes }
        if (intparasize < 16) then
          intparasize := 16;
      end;


    function tcpuparamanager.create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;
      begin
        intparareg:=0;
        intparasize:=0;
        can_use_float := true;
        { Create Function result paraloc }
        create_funcretloc_info(p,callerside);
        { calculate the registers for the normal parameters }
        create_paraloc_info_intern(p,callerside,p.paras);
        { append the varargs }
        can_use_float := false;
        { restore correct intparasize value }
        if intparareg < 4 then
          intparasize:=intparareg * 4;
        create_paraloc_info_intern(p,callerside,varargspara);
        { We need to return the size allocated on the stack }
        result:=intparasize;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                break;
              end;

            if push_addr_param(hp.varspez,hp.vardef,p.proccalloption) then
              paracgsize:=OS_ADDR
            else
              begin
                paracgsize:=def_cgSize(hp.vardef);
                if paracgsize=OS_NO then
                  paracgsize:=OS_ADDR;
              end;
            hp.paraloc[side].reset;
            hp.paraloc[side].size:=paracgsize;
            hp.paraloc[side].Alignment:=std_param_align;
            paralen:=tcgsize2size[paracgsize];
            hp.paraloc[side].intsize:=paralen;
            while paralen>0 do
              begin
                paraloc:=hp.paraloc[side].add_location;
                { Floats are passed in int registers,
                  We can allocate at maximum 32 bits per register }
                if paracgsize in [OS_64,OS_S64,OS_F32,OS_F64] then
                  paraloc^.size:=OS_32
                else
                  paraloc^.size:=paracgsize;
                { ret in param? }
                if vo_is_funcret in hp.varoptions then
                  begin
                    paraloc^.loc:=LOC_REFERENCE;
                    if side=callerside then
                    begin
                      paraloc^.reference.index := NR_STACK_POINTER_REG;
                      paraloc^.reference.offset:=target_info.first_parm_offset{1000}-12 - parasize;
                    end
                    else
                    begin
                      paraloc^.reference.index := NR_FRAME_POINTER_REG;
                      paraloc^.reference.offset:=target_info.first_parm_offset{1000}-4 - parasize;
                      param_offset[i] := @paraloc^.reference.offset;
                    end;
                    inc(parasize,align(tcgsize2size[paraloc^.size],sizeof(aint)));
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
                      begin
                        paraloc^.reference.index := {NR_R17;//}NR_STACK_POINTER_REG;
                        paraloc^.reference.offset:=target_info.first_parm_offset{1000}-12 - parasize;
                      end
                    else
                      begin
                        paraloc^.reference.index := {NR_R18;//}NR_FRAME_POINTER_REG;
                        paraloc^.reference.offset:=target_info.first_parm_offset{1000}-4 - parasize;
                        param_offset[i] := @paraloc^.reference.offset;
                      end;
                    { Parameters are aligned at 4 bytes }
                    inc(parasize,align(tcgsize2size[paraloc^.size],sizeof(aint)));
                  end;
                dec(paralen,tcgsize2size[paraloc^.size]);
              end;
          end;
        for i:=0 to paras.count-1 do
        begin
          if (side = calleeside) and (param_offset[i] <> nil) then
            param_offset[i]^ := param_offset[i]^ + parasize - 8;
        end;
      end;


    function TMIPSELParaManager.create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;
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
      end;



<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function tcpuparamanager.create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;
      begin
        intparareg:=0;
        intparasize:=0;
        can_use_float := true;
        { Create Function result paraloc }
        create_funcretloc_info(p,side);
        create_paraloc_info_intern(p,side,p.paras);
        { We need to return the size allocated on the stack }
        result:=intparasize;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function tMIPSELparamanager.create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;
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
      end;


begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   ParaManager:=tcpuparamanager.create;
=======
   ParaManager:=TMIPSELParaManager.create;
>>>>>>> graemeg/cpstrnew
=======
   ParaManager:=TMIPSELParaManager.create;
>>>>>>> graemeg/cpstrnew
=======
   ParaManager:=TMIPSELParaManager.create;
>>>>>>> graemeg/cpstrnew
=======
   ParaManager:=TMIPSELParaManager.create;
>>>>>>> origin/cpstrnew
end.
