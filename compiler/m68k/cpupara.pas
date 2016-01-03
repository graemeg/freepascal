{
    Copyright (c) 2002 by Florian Klaempfl

    Generates the argument location information for 680x0

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
      cpubase,
      aasmdata,
      symconst,symtype,symdef,symsym,
      parabase,paramgr,cgbase,cgutils;

    type
       { Returns the location for the nr-st 32 Bit int parameter
         if every parameter before is an 32 Bit int parameter as well
         and if the calling conventions for the helper routines of the
         rtl are used.
       }
       tcpuparamanager = class(tparamanager)
          function ret_in_param(def:tdef;pd:tabstractprocdef):boolean;override;
          function param_use_paraloc(const cgpara:tcgpara):boolean;override;
          function create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;override;
          function push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;
          procedure createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);override;
          function create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;override;
          procedure createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
          procedure create_funcretloc_info(p : tabstractprocdef; side: tcallercallee);
         private
          procedure init_values(var curintreg, curfloatreg: tsuperregister; var cur_stack_offset: aword);
          function create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                                               var curintreg, curfloatreg: tsuperregister; var cur_stack_offset: aword):longint;
>>>>>>> graemeg/cpstrnew
          function parseparaloc(p : tparavarsym;const s : string) : boolean;override;
          function parsefuncretloc(p : tabstractprocdef; const s : string) : boolean;override;
          function get_volatile_registers_int(calloption:tproccalloption):tcpuregisterset;override;
          function get_volatile_registers_address(calloption:tproccalloption):tcpuregisterset;override;
          function get_volatile_registers_fpu(calloption:tproccalloption):tcpuregisterset;override;
         private
          function parse_loc_string_to_register(var locreg: tregister; const s : string): boolean;
          function create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                                                var cur_stack_offset: aword):longint;
       end;

  implementation

    uses
       verbose,
       globals,
       systems,
       cpuinfo,
       defutil;


    function tcpuparamanager.get_volatile_registers_int(calloption:tproccalloption):tcpuregisterset;
      begin
        { d0 and d1 are considered volatile }
        Result:=VOLATILE_INTREGISTERS;
      end;


    function tcpuparamanager.get_volatile_registers_address(calloption:tproccalloption):tcpuregisterset;
      begin
        { a0 and a1 are considered volatile }
        Result:=VOLATILE_ADDRESSREGISTERS;
      end;

    function tcpuparamanager.get_volatile_registers_fpu(calloption:tproccalloption):tcpuregisterset;
      begin
        { fp0 and fp1 are considered volatile }
        Result:=VOLATILE_FPUREGISTERS;
      end;

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


{ TODO: copied from ppc cg, needs work}
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
          variantdef,
          formaldef :
            result:=true;
          recorddef:
            result:=false;
          arraydef:
            result:=(tarraydef(def).highrange>=tarraydef(def).lowrange) or
                             is_open_array(def) or
                             is_array_of_const(def) or
                             is_array_constructor(def);
          objectdef :
            result:=is_object(def);
          setdef :
            result:=not is_smallset(def);
          stringdef :
            result:=tstringdef(def).stringtype in [st_shortstring,st_longstring];
          procvardef :
            { Handling of methods must match that of records }
            result:=false;
        end;
      end;

    function tcpuparamanager.ret_in_param(def:tdef;pd:tabstractprocdef):boolean;
      begin
        if handle_common_ret_in_param(def,pd,result) then
          exit;

        case def.typ of
          recorddef:
            if def.size in [1,2,4] then
              begin
                result:=false;
                exit;
              end;
        end;
        result:=inherited ret_in_param(def,pd);
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    function tcpuparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;
      var
        paraloc : pcgparalocation;
        retcgsize  : tcgsize;
      begin
        if set_common_funcretloc_info(p,forcetempdef,retcgsize,result) then
          exit;

        { always use the whole 32 bit register when returning values }
        if (side=calleeside) and
           (result.intsize>0) and
           (result.intsize<sizeof(aint)) then
          begin
            result.def:=sinttype;
            result.intsize:=sizeof(aint);
            retcgsize:=OS_SINT;
            result.size:=retcgsize;
=======
    procedure tm68kparamanager.create_funcretloc_info(p: tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;

=======
    procedure tm68kparamanager.create_funcretloc_info(p: tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;

>>>>>>> graemeg/cpstrnew
=======
    procedure tm68kparamanager.create_funcretloc_info(p: tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;

>>>>>>> graemeg/cpstrnew

=======
    procedure tm68kparamanager.create_funcretloc_info(p: tabstractprocdef; side: tcallercallee);
      begin
        p.funcretloc[side]:=get_funcretloc(p,side,p.returndef);
      end;


>>>>>>> origin/cpstrnew
    function tm68kparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; def: tdef): tcgpara;
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

        paraloc:=result.add_location;
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
        if not (cs_fp_emulation in current_settings.moduleswitches) and
           not (current_settings.fputype=fpu_soft) and (result.def.typ=floatdef) then
          begin
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
            if retcgsize in [OS_64,OS_S64] then
             begin
               { low 32bits }
               paraloc^.loc:=LOC_REGISTER;
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
               if side=callerside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_LOW_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_LOW_REG;
               { high 32bits }
               paraloc:=result.add_location;
               paraloc^.loc:=LOC_REGISTER;
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
               if side=calleeside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_HIGH_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_HIGH_REG;
             end
            else
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
               if side=callerside then
                 paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RESULT_REG,cgsize2subreg(R_INTREGISTER,retcgsize))
               else
                 paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RETURN_REG,cgsize2subreg(R_INTREGISTER,retcgsize));
             end;
          end;
      end;

    function tcpuparamanager.create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;
      var
        cur_stack_offset: aword;
      begin
        cur_stack_offset:=0;
        result:=create_paraloc_info_intern(p,side,p.paras,cur_stack_offset);

        create_funcretloc_info(p,side);
      end;

    function tcpuparamanager.create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                               var cur_stack_offset: aword):longint;
      var
        paraloc      : pcgparalocation;
        hp           : tparavarsym;
        paracgsize   : tcgsize;
        paralen      : aint;
        paradef      : tdef;
        i            : longint;
        firstparaloc : boolean;

      begin
        result:=0;

        for i:=0 to paras.count-1 do
          begin
            hp:=tparavarsym(paras[i]);
            paradef:=hp.vardef;

            { syscall for AmigaOS can have already a paraloc set }
            if (vo_has_explicit_paraloc in hp.varoptions) then
              begin
                if not(vo_is_syscall_lib in hp.varoptions) then
                  internalerror(200506051);
                continue;
              end;
            hp.paraloc[side].reset;

            { currently only support C-style array of const }
            if (p.proccalloption in cstylearrayofconst) and
               is_array_of_const(paradef) then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
{$ifdef DEBUG_CHARLIE}
                writeln('loc register');
{$endif DEBUG_CHARLIE}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef DEBUG_CHARLIE}
                writeln('loc register');
{$endif DEBUG_CHARLIE}
>>>>>>> origin/fixes_2_2
                paraloc:=hp.paraloc[side].add_location;
                { hack: the paraloc must be valid, but is not actually used }
                paraloc^.loc:=LOC_REGISTER;
                paraloc^.register:=NR_D0;
                paraloc^.size:=OS_ADDR;
                paraloc^.def:=voidpointertype;
                break;
              end;

            if push_addr_param(hp.varspez,paradef,p.proccalloption) then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
                paradef:=cpointerdef.getreusable_no_free(paradef);
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef DEBUG_CHARLIE}
                writeln('loc register');
{$endif DEBUG_CHARLIE}
                paradef:=voidpointertype;
                loc:=LOC_REGISTER;
>>>>>>> graemeg/fixes_2_2
                paracgsize := OS_ADDR;
                paralen := tcgsize2size[OS_ADDR];
              end
            else
              begin
                if not is_special_array(paradef) then
                  paralen:=paradef.size
                else
                  paralen:=tcgsize2size[def_cgsize(paradef)];

                paracgsize:=def_cgsize(paradef);
                { for things like formaldef }
                if (paracgsize=OS_NO) and (paradef.typ<>recorddef) then
                  begin
                    paracgsize:=OS_ADDR;
                    paralen := tcgsize2size[OS_ADDR];
                  end;
              end;

            hp.paraloc[side].alignment:=std_param_align;
            hp.paraloc[side].size:=paracgsize;
            hp.paraloc[side].intsize:=paralen;
            hp.paraloc[side].def:=paradef;

            if (paralen = 0) then
              if (paradef.typ = recorddef) then
                begin
                  paraloc:=hp.paraloc[side].add_location;
                  paraloc^.loc := LOC_VOID;
                end
              else
                internalerror(200506052);
            firstparaloc:=true;
            { can become < 0 for e.g. 3-byte records }
            while (paralen > 0) do
              begin
                paraloc:=hp.paraloc[side].add_location;

                paraloc^.loc:=LOC_REFERENCE;
                paraloc^.def:=get_paraloc_def(paradef,paralen,firstparaloc);
                if (paradef.typ=floatdef) then
                  paraloc^.size:=int_float_cgsize(paralen)
                else
                  paraloc^.size:=int_cgsize(paralen);

                paraloc^.reference.offset:=cur_stack_offset;
                if (side = callerside) then
                  paraloc^.reference.index:=NR_STACK_POINTER_REG
                else
                  begin
<<<<<<< HEAD
<<<<<<< HEAD
                    paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                    inc(paraloc^.reference.offset,target_info.first_parm_offset);
                    { M68K is a big-endian target }
                    if (paralen<tcgsize2size[OS_INT]) then
                      inc(paraloc^.reference.offset,4-paralen);
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef DEBUG_CHARLIE}
		    writeln('loc reference');
{$endif DEBUG_CHARLIE}
                    paraloc^.loc:=LOC_REFERENCE;
                    paraloc^.size:=int_cgsize(paralen);
                    if (side = callerside) then
                      paraloc^.reference.index:=NR_STACK_POINTER_REG
                    else
                      paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                    paraloc^.reference.offset:=stack_offset;
                    inc(stack_offset,align(paralen,4));
                    paralen := 0;
>>>>>>> graemeg/fixes_2_2
                  end;
                inc(cur_stack_offset,align(paralen,4));
                paralen := 0;

                firstparaloc:=false;
              end;
          end;
         result:=cur_stack_offset;
      end;


    function tcpuparamanager.parse_loc_string_to_register(var locreg: tregister; const s : string): boolean;
      begin
        locreg:=std_regnum_search(lowercase(s));
        result:=(locreg <> NR_NO) and (locreg <> NR_SP);
      end;

    function tcpuparamanager.parsefuncretloc(p : tabstractprocdef; const s : string) : boolean;
      begin
        case target_info.system of
          system_m68k_amiga:
            result:=parse_loc_string_to_register(p.exp_funcretloc, s);
          else
            internalerror(2005121801);
        end;
      end;

    function tcpuparamanager.parseparaloc(p : tparavarsym;const s : string) : boolean;
      var
        paraloc : pcgparalocation;
      begin
        result:=false;
        case target_info.system of
          system_m68k_amiga:
            begin
              p.paraloc[callerside].alignment:=4;
              paraloc:=p.paraloc[callerside].add_location;
              paraloc^.loc:=LOC_REGISTER;
              paraloc^.size:=def_cgsize(p.vardef);
              paraloc^.def:=p.vardef;

              if not parse_loc_string_to_register(paraloc^.register, s) then
                exit;

              { copy to callee side }
              p.paraloc[calleeside].add_location^:=paraloc^;
            end;
          else
            internalerror(200405092);
        end;
        result:=true;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure tcpuparamanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
      begin
        { Never a need for temps when value is pushed (calls inside parameters
          will simply allocate even more stack space for their parameters) }
        if not(use_fixed_stack) then
          can_use_final_stack_loc:=true;
        inherited createtempparaloc(list,calloption,parasym,can_use_final_stack_loc,cgpara);
      end;

    function tcpuparamanager.create_varargs_paraloc_info(p : tabstractprocdef; varargspara:tvarargsparalist):longint;
=======
    procedure tm68kparamanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
>>>>>>> graemeg/cpstrnew
=======
    procedure tm68kparamanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
>>>>>>> graemeg/cpstrnew
=======
    procedure tm68kparamanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
>>>>>>> graemeg/cpstrnew
=======
    procedure tm68kparamanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
>>>>>>> origin/cpstrnew
      var
        cur_stack_offset: aword;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        cur_stack_offset:=0;

        result:=create_paraloc_info_intern(p,callerside,p.paras,cur_stack_offset);
        if (p.proccalloption in cstylearrayofconst) then
          { just continue loading the parameters in the registers }
          result:=create_paraloc_info_intern(p,callerside,varargspara,cur_stack_offset)
        else
          internalerror(200410231);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        paraloc:=parasym.paraloc[callerside].location;
        { Never a need for temps when value is pushed (calls inside parameters
          will simply allocate even more stack space for their parameters) }
        if not(use_fixed_stack) then
          can_use_final_stack_loc:=true;
        inherited createtempparaloc(list,calloption,parasym,can_use_final_stack_loc,cgpara);
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
  paramanager:=tcpuparamanager.create;
end.
