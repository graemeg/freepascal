{
    Copyright (c) 2002 by Florian Klaempfl

    Generates the argument location information for i386

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
unit cpupara;

{$i fpcdefs.inc}

  interface

    uses
       globtype,
       aasmtai,aasmdata,cpubase,cgbase,cgutils,
       symconst,symtype,symsym,symdef,
       parabase,paramgr;

    type
       tcpuparamanager = class(tparamanager)
          function param_use_paraloc(const cgpara:tcgpara):boolean;override;
          function ret_in_param(def:tdef;pd:tabstractprocdef):boolean;override;
          function push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
          function get_para_align(calloption : tproccalloption):byte;override;
          function get_volatile_registers_int(calloption : tproccalloption):tcpuregisterset;override;
          function get_volatile_registers_fpu(calloption : tproccalloption):tcpuregisterset;override;
          function get_volatile_registers_mm(calloption : tproccalloption):tcpuregisterset;override;
          function create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;override;
          function create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;override;
          procedure createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): TCGPara;override;
=======
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): TCGPara;override;
>>>>>>> graemeg/cpstrnew
=======
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): TCGPara;override;
>>>>>>> graemeg/cpstrnew
=======
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): TCGPara;override;
>>>>>>> graemeg/cpstrnew
=======
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): TCGPara;override;
>>>>>>> origin/cpstrnew
       private
          procedure create_stdcall_paraloc_info(p : tabstractprocdef; side: tcallercallee;paras:tparalist;var parasize:longint);
          procedure create_register_paraloc_info(p : tabstractprocdef; side: tcallercallee;paras:tparalist;var parareg,parasize:longint);
       end;


  implementation

    uses
       cutils,
       systems,verbose,
       symtable,
       defutil;

      const
        parasupregs : array[0..2] of tsuperregister = (RS_EAX,RS_EDX,RS_ECX);

{****************************************************************************
                                tcpuparamanager
****************************************************************************}

    function tcpuparamanager.param_use_paraloc(const cgpara:tcgpara):boolean;
      var
        paraloc : pcgparalocation;
      begin
        if not assigned(cgpara.location) then
          internalerror(200410102);
        result:=true;
        { All locations are LOC_REFERENCE }
        paraloc:=cgpara.location;
        while assigned(paraloc) do
          begin
            if (paraloc^.loc<>LOC_REFERENCE) then
              begin
                result:=false;
                exit;
              end;
            paraloc:=paraloc^.next;
          end;
      end;


    function tcpuparamanager.ret_in_param(def:tdef;pd:tabstractprocdef):boolean;
      var
        size: longint;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if handle_common_ret_in_param(def,pd,result) then
          exit;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if (tf_safecall_exceptions in target_info.flags) and
           (calloption=pocall_safecall) then
          begin
            result:=true;
            exit;
          end;
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
        case target_info.system of
          system_i386_win32 :
            begin
              case def.typ of
                recorddef :
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                    { Win32 GCC returns small records in the FUNCTION_RETURN_REG up to 8 bytes in registers.

                      For stdcall and register we follow delphi instead of GCC which returns
                      only records of a size of 1,2 or 4 bytes in FUNCTION_RETURN_REG }
                    if ((pd.proccalloption in [pocall_stdcall,pocall_register]) and
                        (def.size in [1,2,4])) or
                       ((pd.proccalloption in [pocall_cdecl,pocall_cppdecl]) and
                        (def.size>0) and
                        (def.size<=8)) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                    { Win32 GCC returns small records in the FUNCTION_RETURN_REG.
                      For stdcall we follow delphi instead of GCC }
                    if (calloption in [pocall_cdecl,pocall_cppdecl]) and
                       (def.size>0) and
                       (def.size<=8) then
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
                       exit;
                     end;
                  end;
              end;
            end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          system_i386_os2,
          system_i386_emx:
=======
          system_i386_darwin,
          system_i386_iphonesim :
>>>>>>> graemeg/cpstrnew
=======
          system_i386_darwin,
          system_i386_iphonesim :
>>>>>>> graemeg/cpstrnew
=======
          system_i386_darwin,
          system_i386_iphonesim :
>>>>>>> graemeg/cpstrnew
=======
          system_i386_darwin,
          system_i386_iphonesim :
>>>>>>> origin/cpstrnew
            begin
              case def.typ of
                recorddef :
                  begin
                    { EMX port of GCC returns small records in the FUNCTION_RETURN_REG up to 4 bytes in registers. }
                    if ((pd.proccalloption in [pocall_cdecl,pocall_cppdecl]) and
                        (def.size>0) and
                        (def.size<=4)) then
                     begin
                       result:=false;
                       exit;
                     end;
                  end;
              end;
            end;
          system_i386_freebsd,
          system_i386_openbsd,
          system_i386_darwin,
          system_i386_iphonesim :
            begin
              if pd.proccalloption in cdecl_pocalls then
                begin
                  case def.typ of
                    recorddef :
                      begin
                        size:=def.size;
                        if (size>0) and
                           (size<=8) and
                           { only if size is a power of 2 }
                           ((size and (size-1)) = 0) then
                          begin
                            result:=false;
                            exit;
                          end;
                      end;
                    procvardef:
                      begin
                        result:=false;
                        exit;
                      end;
                  end;
              end;
            end;
        end;
        result:=inherited ret_in_param(def,pd);
      end;


    function tcpuparamanager.push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;
      begin
        result:=false;
        { var,out,constref always require address }
        if varspez in [vs_var,vs_out,vs_constref] then
          begin
            result:=true;
            exit;
          end;
        { Only vs_const, vs_value here }
        case def.typ of
          variantdef :
            begin
              { variants are small enough to be passed by value except if
                required by the windows api

                variants are somethings very delphi/windows specific so do it like
                windows/delphi (FK)
              }
              if ((target_info.system=system_i386_win32) and
                 (calloption in [pocall_stdcall,pocall_safecall]) and
                 (varspez=vs_const)) or
                 (calloption=pocall_register) then
                result:=true
              else
                result:=false;
            end;
          formaldef :
            result:=true;
          recorddef :
            begin
              { Delphi stdcall passes records on the stack for call by value }
              if (target_info.system=system_i386_win32) and
                 (calloption=pocall_stdcall) and
                 (varspez=vs_value) then
                result:=false
              else
                result:=
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  (not(calloption in (cdecl_pocalls)) and
=======
                  (not(calloption in (cdecl_pocalls+[pocall_mwpascal])) and
>>>>>>> graemeg/cpstrnew
=======
                  (not(calloption in (cdecl_pocalls+[pocall_mwpascal])) and
>>>>>>> graemeg/cpstrnew
=======
                  (not(calloption in (cdecl_pocalls+[pocall_mwpascal])) and
>>>>>>> graemeg/cpstrnew
=======
                  (not(calloption in (cdecl_pocalls+[pocall_mwpascal])) and
>>>>>>> origin/cpstrnew
                   (def.size>sizeof(aint))) or
                  (((calloption = pocall_mwpascal) or (target_info.system=system_i386_wince)) and
                   (varspez=vs_const));
            end;
          arraydef :
            begin
              { array of const values are pushed on the stack as
                well as dyn. arrays }
<<<<<<< HEAD
<<<<<<< HEAD
              if (calloption in cdecl_pocalls) then
=======
              if (calloption in [pocall_cdecl,pocall_cppdecl]) then
>>>>>>> graemeg/fixes_2_2
=======
              if (calloption in [pocall_cdecl,pocall_cppdecl]) then
>>>>>>> origin/fixes_2_2
                result:=not(is_array_of_const(def) or
                        is_dynamic_array(def))
              else
                begin
                  result:=(
                           (tarraydef(def).highrange>=tarraydef(def).lowrange) and
                           (def.size>sizeof(aint))
                          ) or
                          is_open_array(def) or
                          is_array_of_const(def) or
                          is_array_constructor(def);
                end;
            end;
          objectdef :
            result:=is_object(def);
          stringdef :
            result:= (tstringdef(def).stringtype in [st_shortstring,st_longstring]);
          procvardef :
            result:=not(calloption in cdecl_pocalls) and not tprocvardef(def).is_addressonly;
          setdef :
            result:=not(calloption in cdecl_pocalls) and (not is_smallset(def));
        end;
      end;


    function tcpuparamanager.get_para_align(calloption : tproccalloption):byte;
      begin
        if calloption=pocall_oldfpccall then
          begin
            if target_info.system in [system_i386_go32v2,system_i386_watcom] then
              result:=2
            else
              result:=4;
          end
        else
          result:=std_param_align;
      end;


    function tcpuparamanager.get_volatile_registers_int(calloption : tproccalloption):tcpuregisterset;
      begin
        case calloption of
          pocall_internproc :
            result:=[];
          pocall_register,
          pocall_safecall,
          pocall_stdcall,
          pocall_cdecl,
          pocall_syscall,
          pocall_cppdecl,
          pocall_mwpascal :
            result:=[RS_EAX,RS_EDX,RS_ECX];
          pocall_far16,
          pocall_pascal,
          pocall_oldfpccall :
            result:=[RS_EAX,RS_EDX,RS_ECX,RS_ESI,RS_EDI,RS_EBX];
          else
            internalerror(200309071);
        end;
      end;


    function tcpuparamanager.get_volatile_registers_fpu(calloption : tproccalloption):tcpuregisterset;
      begin
        result:=[0..first_fpu_imreg-1];
      end;


    function tcpuparamanager.get_volatile_registers_mm(calloption : tproccalloption):tcpuregisterset;
      begin
        result:=[0..first_mm_imreg-1];
      end;


<<<<<<< HEAD
    function  tcpuparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): TCGPara;
      var
        retcgsize  : tcgsize;
        paraloc : pcgparalocation;
        fdef,
        usedef: tdef;
        handled: boolean;
      begin
        if not assigned(forcetempdef) then
          usedef:=p.returndef
        else
          usedef:=forcetempdef;
        { on darwin/i386, if a record has only one field and that field is a
          single or double, it has to be returned like a single/double }
        if (target_info.system in [system_i386_darwin,system_i386_iphonesim,
                                   system_i386_freebsd,system_i386_openbsd,
                                   system_i386_os2,system_i386_emx]) and
           ((usedef.typ=recorddef) or
            is_object(usedef)) and
           tabstractrecordsymtable(tabstractrecorddef(usedef).symtable).has_single_field(fdef) and
           (fdef.typ=floatdef) and
           (tfloatdef(fdef).floattype in [s32real,s64real]) then
          usedef:=fdef;

        handled:=set_common_funcretloc_info(p,usedef,retcgsize,result);
        { normally forcetempdef is passed straight through to
          set_common_funcretloc_info and that one will correctly determine whether
          the location is a temporary one, but that doesn't work here because we
          sometimes have to change the type }
        result.temporary:=assigned(forcetempdef);
        if handled then
          exit;

        { darwin/x86 requires that results < sizeof(aint) are sign/zero
          extended to sizeof(aint) }
        if (target_info.system in [system_i386_darwin,system_i386_iphonesim]) and
           (side=calleeside) and
           (result.intsize>0) and
           (result.intsize<sizeof(aint)) then
          begin
            result.def:=sinttype;
            result.intsize:=sizeof(aint);
            retcgsize:=OS_SINT;
            result.size:=retcgsize;
=======
    procedure ti386paramanager.getintparaloc(calloption : tproccalloption; nr : longint;var cgpara:TCGPara);
      var
        paraloc : pcgparalocation;
      begin
        cgpara.reset;
        cgpara.size:=OS_ADDR;
        cgpara.intsize:=sizeof(pint);
        cgpara.alignment:=get_para_align(calloption);
        paraloc:=cgpara.add_location;
        with paraloc^ do
         begin
           size:=OS_INT;
           if calloption=pocall_register then
             begin
               if (nr<=high(parasupregs)+1) then
                 begin
                   if nr=0 then
                     internalerror(200309271);
                   loc:=LOC_REGISTER;
                   register:=newreg(R_INTREGISTER,parasupregs[nr-1],R_SUBWHOLE);
                 end
               else
                 begin
                   loc:=LOC_REFERENCE;
                   reference.index:=NR_STACK_POINTER_REG;
                   reference.offset:=sizeof(aint)*nr;
                 end;
             end
           else
             begin
               loc:=LOC_REFERENCE;
               reference.index:=NR_STACK_POINTER_REG;
               reference.offset:=sizeof(aint)*nr;
             end;
          end;
      end;


    procedure ti386paramanager.create_funcretloc_info(p : tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;


    function  ti386paramanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): TCGPara;
      var
        retcgsize  : tcgsize;
        paraloc : pcgparalocation;
        sym: tfieldvarsym;
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
        { on darwin/i386, if a record has only one field and that field is a
          single or double, it has to be returned like a single/double }
        if (target_info.system in [system_i386_darwin,system_i386_iphonesim]) and
           ((def.typ=recorddef) or
            is_object(def)) and
           tabstractrecordsymtable(tabstractrecorddef(def).symtable).has_single_field(sym) and
           (sym.vardef.typ=floatdef) and
           (tfloatdef(sym.vardef).floattype in [s32real,s64real]) then
          def:=sym.vardef;
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
            exit;
>>>>>>> graemeg/cpstrnew
          end;
<<<<<<< HEAD

=======
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
        { Return is passed as var parameter }
        if ret_in_param(p.returndef,p.proccalloption) then
          begin
            p.funcretloc[side].loc:=LOC_REFERENCE;
            p.funcretloc[side].size:=retcgsize;
            exit;
          end;
>>>>>>> origin/fixes_2_2
        { Return in FPU register? }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if result.def.typ=floatdef then
=======
        if def.typ=floatdef then
>>>>>>> graemeg/cpstrnew
=======
        if def.typ=floatdef then
>>>>>>> graemeg/cpstrnew
=======
        if def.typ=floatdef then
>>>>>>> graemeg/cpstrnew
=======
        if def.typ=floatdef then
>>>>>>> origin/cpstrnew
          begin
            paraloc:=result.add_location;
            paraloc^.loc:=LOC_FPUREGISTER;
            paraloc^.register:=NR_FPU_RESULT_REG;
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
            paraloc:=result.add_location;
            paraloc^.loc:=LOC_REGISTER;
            if retcgsize in [OS_64,OS_S64] then
             begin
               { low 32bits }
               if side=callerside then
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

               { high 32bits }
               paraloc:=result.add_location;
               paraloc^.loc:=LOC_REGISTER;
               if side=callerside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_HIGH_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_HIGH_REG;
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
             begin
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
               if side=callerside then
                 paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RESULT_REG,cgsize2subreg(R_INTREGISTER,retcgsize))
               else
                 paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RETURN_REG,cgsize2subreg(R_INTREGISTER,retcgsize));
             end;
          end;
      end;


    procedure tcpuparamanager.create_stdcall_paraloc_info(p : tabstractprocdef; side: tcallercallee;paras:tparalist;var parasize:longint);
      var
        i  : integer;
        hp : tparavarsym;
        paradef : tdef;
        paraloc : pcgparalocation;
        l,
        paralen,
        varalign   : longint;
        paraalign  : shortint;
        paracgsize : tcgsize;
        firstparaloc,
        pushaddr   : boolean;
      begin
        paraalign:=get_para_align(p.proccalloption);
        { we push Flags and CS as long
          to cope with the IRETD
          and we save 6 register + 4 selectors }
        if po_interrupt in p.procoptions then
          inc(parasize,8+6*4+4*2);
        { Offset is calculated like:
           sub esp,12
           mov [esp+8],para3
           mov [esp+4],para2
           mov [esp],para1
           call function
          That means for pushes the para with the
          highest offset (see para3) needs to be pushed first
        }
        if p.proccalloption in pushleftright_pocalls then
          i:=paras.count-1
        else
          i:=0;
        while ((p.proccalloption in pushleftright_pocalls) and (i>=0)) or
              (not(p.proccalloption in pushleftright_pocalls) and (i<=paras.count-1)) do
          begin
            hp:=tparavarsym(paras[i]);
            paradef:=hp.vardef;
            pushaddr:=push_addr_param(hp.varspez,paradef,p.proccalloption);
            if pushaddr then
              begin
                paralen:=sizeof(aint);
                paracgsize:=OS_ADDR;
                paradef:=cpointerdef.getreusable_no_free(paradef);
              end
            else
              begin
                paralen:=push_size(hp.varspez,paradef,p.proccalloption);
                { darwin/x86 requires that parameters < sizeof(aint) are sign/ }
                { zero extended to sizeof(aint)                                }
                if (target_info.system in [system_i386_darwin,system_i386_iphonesim]) and
                   (side = callerside) and
                   (paralen > 0) and
                   (paralen < sizeof(aint)) then
                  begin
                    paralen:=sizeof(aint);
                    paracgsize:=OS_SINT;
                    paradef:=sinttype;
                  end
                else
                  paracgsize:=def_cgsize(paradef);
              end;
            hp.paraloc[side].reset;
            hp.paraloc[side].size:=paracgsize;
            hp.paraloc[side].intsize:=paralen;
            hp.paraloc[side].def:=paradef;
            hp.paraloc[side].Alignment:=paraalign;
            { Copy to stack? }
            if (paracgsize=OS_NO) or
               (use_fixed_stack) then
              begin
                paraloc:=hp.paraloc[side].add_location;
                paraloc^.loc:=LOC_REFERENCE;
                paraloc^.size:=paracgsize;
                paraloc^.def:=paradef;
                if side=callerside then
                  paraloc^.reference.index:=NR_STACK_POINTER_REG
                else
                  paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                varalign:=used_align(size_2_align(paralen),paraalign,paraalign);

                { don't let push_size return 16, because then we can    }
                { read past the end of the heap since the value is only }
                { 10 bytes long (JM)                                    }
                if (paracgsize = OS_F80) and
                   (target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
                  paralen:=16;
                paraloc^.reference.offset:=parasize;
                if side=calleeside then
                  inc(paraloc^.reference.offset,target_info.first_parm_offset);
                parasize:=align(parasize+paralen,varalign);
              end
            else
              begin
                if paralen=0 then
                  internalerror(200501163);
                firstparaloc:=true;
                while (paralen>0) do
                  begin
                    paraloc:=hp.paraloc[side].add_location;
                    paraloc^.loc:=LOC_REFERENCE;
                    { single and double need a single location }
                    if (paracgsize in [OS_F64,OS_F32]) then
                      begin
                        paraloc^.size:=paracgsize;
                        paraloc^.def:=paradef;
                        l:=paralen;
                      end
                    else
                      begin
                        { We can allocate at maximum 32 bits per location }
                        if paralen>sizeof(aint) then
                          begin
                            l:=sizeof(aint);
                            paraloc^.def:=uinttype;
                          end
                        else
                          begin
                            l:=paralen;
                            paraloc^.def:=get_paraloc_def(paradef,l,firstparaloc);
                          end;
                        paraloc^.size:=int_cgsize(l);
                      end;
                    if (side=callerside) or
                       (po_nostackframe in p.procoptions) then
                      paraloc^.reference.index:=NR_STACK_POINTER_REG
                    else
                      paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                    varalign:=used_align(size_2_align(l),paraalign,paraalign);
                    paraloc^.reference.offset:=parasize;
                    if side=calleeside then
                      if not(po_nostackframe in p.procoptions) then
                        inc(paraloc^.reference.offset,target_info.first_parm_offset)
                      else
                        { return addres }
                        inc(paraloc^.reference.offset,4);
                    parasize:=align(parasize+l,varalign);
                    dec(paralen,l);
                    firstparaloc:=false;
                  end;
              end;
            if p.proccalloption in pushleftright_pocalls then
              dec(i)
            else
              inc(i);
          end;
      end;


    procedure tcpuparamanager.create_register_paraloc_info(p : tabstractprocdef; side: tcallercallee;paras:tparalist;
                                                            var parareg,parasize:longint);
      var
        hp : tparavarsym;
        paradef : tdef;
        paraloc : pcgparalocation;
        paracgsize : tcgsize;
        i : integer;
        l,
        paralen,
        varalign : longint;
        paraalign : shortint;
        pass : byte;
        firstparaloc,
        pushaddr : boolean;
      begin
        if paras.count=0 then
          exit;
        paraalign:=get_para_align(p.proccalloption);

        { clean up here so we can later detect properly if a parameter has been
          assigned or not
        }
        for i:=0 to paras.count-1 do
          tparavarsym(paras[i]).paraloc[side].reset;
        { Register parameters are assigned from left to right,
          stack parameters from right to left so assign first the
          register parameters in a first pass, in the second
          pass all unhandled parameters are done }
        for pass:=1 to 2 do
          begin
            if pass=1 then
              i:=0
            else
              i:=paras.count-1;
            while true do
              begin
                hp:=tparavarsym(paras[i]);
                paradef:=hp.vardef;
                if not(assigned(hp.paraloc[side].location)) then
                  begin
                    pushaddr:=push_addr_param(hp.varspez,hp.vardef,p.proccalloption);
                    if pushaddr then
                      begin
                        paralen:=sizeof(aint);
                        paracgsize:=OS_ADDR;
                        paradef:=cpointerdef.getreusable_no_free(paradef);
                      end
                    else
                      begin
                        paralen:=push_size(hp.varspez,hp.vardef,p.proccalloption);
                        paracgsize:=def_cgsize(hp.vardef);
                      end;
                    hp.paraloc[side].size:=paracgsize;
                    hp.paraloc[side].intsize:=paralen;
                    hp.paraloc[side].Alignment:=paraalign;
                    hp.paraloc[side].def:=paradef;
                    {
                      EAX
                      EDX
                      ECX
                      Stack
                      Stack

                      64bit values,floats,arrays and records are always
                      on the stack.

                      In case of po_delphi_nested_cc, the parent frame pointer
                      is also always passed on the stack.
                    }
                    if (parareg<=high(parasupregs)) and
                       (paralen<=sizeof(aint)) and
                       (not(hp.vardef.typ in [floatdef,recorddef,arraydef]) or
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        pushaddr or
                        is_dynamic_array(hp.vardef)) and
=======
                        pushaddr) and
>>>>>>> graemeg/cpstrnew
=======
                        pushaddr) and
>>>>>>> graemeg/cpstrnew
=======
                        pushaddr) and
>>>>>>> graemeg/cpstrnew
=======
                        pushaddr) and
>>>>>>> origin/cpstrnew
                       (not(vo_is_parentfp in hp.varoptions) or
                        not(po_delphi_nested_cc in p.procoptions)) then
                      begin
                        if pass=1 then
                          begin
                            paraloc:=hp.paraloc[side].add_location;
                            paraloc^.size:=paracgsize;
                            paraloc^.def:=paradef;
                            paraloc^.loc:=LOC_REGISTER;
                            paraloc^.register:=newreg(R_INTREGISTER,parasupregs[parareg],cgsize2subreg(R_INTREGISTER,paracgsize));
                            inc(parareg);
                          end;
                      end
                    else
                      if pass=2 then
                        begin
                          { Copy to stack? }
                          if (use_fixed_stack) or
                             (paracgsize=OS_NO) then
                            begin
                              paraloc:=hp.paraloc[side].add_location;
                              paraloc^.loc:=LOC_REFERENCE;
                              paraloc^.size:=paracgsize;
                              paraloc^.def:=paradef;
                              if side=callerside then
                                paraloc^.reference.index:=NR_STACK_POINTER_REG
                              else
                                paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                              varalign:=used_align(size_2_align(paralen),paraalign,paraalign);
                              paraloc^.reference.offset:=parasize;
                              if side=calleeside then
                                inc(paraloc^.reference.offset,target_info.first_parm_offset);
                              parasize:=align(parasize+paralen,varalign);
                            end
                          else
                            begin
                              if paralen=0 then
                                internalerror(200501163);
                              firstparaloc:=true;
                              while (paralen>0) do
                                begin
                                  paraloc:=hp.paraloc[side].add_location;
                                  paraloc^.loc:=LOC_REFERENCE;
                                  { Extended and double need a single location }
                                  if (paracgsize in [OS_F64,OS_F32]) then
                                    begin
                                      paraloc^.size:=paracgsize;
                                      paraloc^.def:=paradef;
                                      l:=paralen;
                                    end
                                  else
                                    begin
                                      { We can allocate at maximum 32 bits per location }
                                      if paralen>sizeof(aint) then
                                        begin
                                          l:=sizeof(aint);
                                          paraloc^.def:=uinttype;
                                        end
                                      else
                                        begin
                                          l:=paralen;
                                          paraloc^.def:=get_paraloc_def(paradef,l,firstparaloc);
                                        end;
                                      paraloc^.size:=int_cgsize(l);
                                    end;
                                  if side=callerside then
                                    paraloc^.reference.index:=NR_STACK_POINTER_REG
                                  else
                                    paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                                  varalign:=used_align(size_2_align(l),paraalign,paraalign);
                                  paraloc^.reference.offset:=parasize;
                                  if side=calleeside then
                                    inc(paraloc^.reference.offset,target_info.first_parm_offset);
                                  parasize:=align(parasize+l,varalign);
                                  dec(paralen,l);
                                  firstparaloc:=false;
                                end;
                            end;
                        end;
                  end;
                case pass of
                  1:
                    begin
                      if i=paras.count-1 then
                        break;
                      inc(i);
                    end;
                  2:
                    begin
                      if i=0 then
                        break;
                      dec(i);
                    end;
                end;
              end;
          end;
      end;


    function tcpuparamanager.create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;
      var
        parasize,
        parareg : longint;
      begin
        parasize:=0;
        parareg:=0;
        case p.proccalloption of
          pocall_register :
            create_register_paraloc_info(p,side,p.paras,parareg,parasize);
          pocall_internproc :
            begin
              { Use default calling }
{$warnings off}
              if (pocall_default=pocall_register) then
                create_register_paraloc_info(p,side,p.paras,parareg,parasize)
              else
                create_stdcall_paraloc_info(p,side,p.paras,parasize);
{$warnings on}
            end;
          else
            create_stdcall_paraloc_info(p,side,p.paras,parasize);
        end;
        create_funcretloc_info(p,side);
        result:=parasize;
      end;


    function tcpuparamanager.create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;
      var
        parasize : longint;
      begin
        parasize:=0;
        { calculate the registers for the normal parameters }
        create_stdcall_paraloc_info(p,callerside,p.paras,parasize);
        { append the varargs }
        create_stdcall_paraloc_info(p,callerside,varargspara,parasize);
        result:=parasize;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure tcpuparamanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
=======
    procedure ti386paramanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
>>>>>>> graemeg/cpstrnew
=======
    procedure ti386paramanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
>>>>>>> graemeg/cpstrnew
=======
    procedure ti386paramanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
>>>>>>> graemeg/cpstrnew
=======
    procedure ti386paramanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
>>>>>>> origin/cpstrnew
      begin
        { Never a need for temps when value is pushed (calls inside parameters
          will simply allocate even more stack space for their parameters) }
        if not(use_fixed_stack) then
          can_use_final_stack_loc:=true;
        inherited createtempparaloc(list,calloption,parasym,can_use_final_stack_loc,cgpara);
      end;


begin
   paramanager:=tcpuparamanager.create;
end.
