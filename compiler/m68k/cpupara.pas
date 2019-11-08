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
          function get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;
          procedure createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);override;
          function create_varargs_paraloc_info(p : tabstractprocdef; side: tcallercallee; varargspara:tvarargsparalist):longint;override;
          function parsefuncretloc(p : tabstractprocdef; const s : string) : boolean;override;
          function get_volatile_registers_int(calloption:tproccalloption):tcpuregisterset;override;
          function get_volatile_registers_address(calloption:tproccalloption):tcpuregisterset;override;
          function get_volatile_registers_fpu(calloption:tproccalloption):tcpuregisterset;override;
          function get_saved_registers_int(calloption:tproccalloption):tcpuregisterarray;override;
          function get_saved_registers_address(calloption:tproccalloption):tcpuregisterarray;override;
          function get_saved_registers_fpu(calloption:tproccalloption):tcpuregisterarray;override;
          function get_para_align(calloption : tproccalloption):byte;override;
         private
          function parse_loc_string_to_register(var locreg: tregister; const s : string): boolean;
          function create_stdcall_paraloc_info(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                                                var cur_stack_offset: aword):longint;
          function create_register_paraloc_info(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                                                var cur_stack_offset: aword):longint;
       end;

  implementation

    uses
       verbose,
       globals,
       systems,
       cpuinfo,
       defutil,
       cutils,
       hlcgobj;

    const
      intparasupregs : array[0..1] of tsuperregister = (RS_D0,RS_D1);
      addrparasupregs : array[0..1] of tsuperregister = (RS_A0,RS_A1);
      floatparasupregs : array[0..1] of tsuperregister = (RS_FP0,RS_FP1);


    function tcpuparamanager.get_volatile_registers_int(calloption:tproccalloption):tcpuregisterset;
      begin
        { d0 and d1 are considered volatile }
        Result:=VOLATILE_INTREGISTERS;
        if target_info.system in [system_m68k_palmos] then
          include(result,RS_D2);
      end;


    function tcpuparamanager.get_volatile_registers_address(calloption:tproccalloption):tcpuregisterset;
      begin
        { a0 and a1 are considered volatile }
        Result:=VOLATILE_ADDRESSREGISTERS;
        if target_info.system in [system_m68k_palmos] then
          include(result,RS_A2);
      end;

    function tcpuparamanager.get_volatile_registers_fpu(calloption:tproccalloption):tcpuregisterset;
      begin
        { fp0 and fp1 are considered volatile }
        Result:=VOLATILE_FPUREGISTERS;
      end;

    function tcpuparamanager.get_saved_registers_int(calloption:tproccalloption):tcpuregisterarray;
      const
        saved_regs: {$ifndef VER3_0}tcpuregisterarray{$else}array[0..5] of tsuperregister{$endif} = (RS_D2,RS_D3,RS_D4,RS_D5,RS_D6,RS_D7);
      begin
        result:=saved_regs;
      end;

    function tcpuparamanager.get_saved_registers_address(calloption:tproccalloption):tcpuregisterarray;
      const
        saved_addr_regs: {$ifndef VER3_0}tcpuregisterarray{$else}array[0..4] of tsuperregister{$endif} = (RS_A2,RS_A3,RS_A4,RS_A5,RS_A6);
      begin
        result:=saved_addr_regs;
      end;

    function tcpuparamanager.get_saved_registers_fpu(calloption:tproccalloption):tcpuregisterarray;
      const
        saved_fpu_regs: {$ifndef VER3_0}tcpuregisterarray{$else}array[0..5] of tsuperregister{$endif} = (RS_FP2,RS_FP3,RS_FP4,RS_FP5,RS_FP6,RS_FP7);
      begin
        result:=saved_fpu_regs;
      end;

    function tcpuparamanager.get_para_align(calloption : tproccalloption):byte;
      begin
        result:=target_info.stackalign;
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
            result:=(calloption in [pocall_register]) and
                    (varspez in [vs_const]);
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
           else
             ;
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
          else
            ;
        end;
        result:=inherited ret_in_param(def,pd);
      end;


    function tcpuparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;
      var
        paraloc    : pcgparalocation;
        retcgsize  : tcgsize;
        retregtype : tregistertype;
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
          end;

        paraloc:=result.add_location;
        { Return in FPU register? }
        if not (cs_fp_emulation in current_settings.moduleswitches) and
           not (current_settings.fputype=fpu_soft) and (result.def.typ=floatdef) then
          begin
            paraloc^.loc:=LOC_FPUREGISTER;
            paraloc^.register:=NR_FPU_RESULT_REG;
            paraloc^.size:=retcgsize;
            paraloc^.def:=result.def;
          end
        else
         { Return in register }
          begin
            if retcgsize in [OS_64,OS_S64] then
             begin
               { low 32bits }
               paraloc^.loc:=LOC_REGISTER;
               paraloc^.size:=OS_32;
               paraloc^.def:=u32inttype;
               if side=callerside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_LOW_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_LOW_REG;
               { high 32bits }
               paraloc:=result.add_location;
               paraloc^.loc:=LOC_REGISTER;
               paraloc^.size:=OS_32;
               paraloc^.def:=u32inttype;
               if side=calleeside then
                 paraloc^.register:=NR_FUNCTION_RESULT64_HIGH_REG
               else
                 paraloc^.register:=NR_FUNCTION_RETURN64_HIGH_REG;
             end
            else
             begin
               paraloc^.loc:=LOC_REGISTER;
               paraloc^.size:=retcgsize;
               paraloc^.def:=result.def;

               { GCC (and SVR4 in general maybe?) requires a pointer result on the A0
                 register, as well as D0. So we init the result to be A0, then copy
                 it also to D0 in hlcg.gen_load_loc_function_result. This is not pretty,
                 but we don't really have an architecture for funcretlocs in two
                 separate locations.

                 We also have to figure out a better switch for this, because this is
                 now compiler and platform specific... (KB) }

               if (tabstractprocdef(p).proccalloption in [pocall_syscall,pocall_cdecl,pocall_cppdecl]) and
                  (target_info.system in [system_m68k_palmos,system_m68k_linux]) and
                  assigned(result.def) and
                  (result.def.typ in [stringdef,pointerdef,classrefdef,objectdef,
                                      procvardef,procdef,arraydef,formaldef]) then
                 retregtype:=R_ADDRESSREGISTER
               else
                 retregtype:=R_INTREGISTER;

               if retregtype = R_ADDRESSREGISTER then
                 begin
                   if side=callerside then
                     paraloc^.register:=newreg(R_ADDRESSREGISTER,RS_RETURN_ADDRESS_REG,cgsize2subreg(R_ADDRESSREGISTER,retcgsize))
                   else
                     paraloc^.register:=newreg(R_ADDRESSREGISTER,RS_RETURN_ADDRESS_REG,cgsize2subreg(R_ADDRESSREGISTER,retcgsize));
                 end
               else
                 begin
                   if side=callerside then
                     paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RESULT_REG,cgsize2subreg(R_INTREGISTER,retcgsize))
                   else
                     paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RETURN_REG,cgsize2subreg(R_INTREGISTER,retcgsize));
                 end;
             end;
          end;
      end;

    function tcpuparamanager.create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;
      var
        cur_stack_offset: aword;
      begin
        cur_stack_offset:=0;
        case p.proccalloption of
          pocall_register :
            result:=create_register_paraloc_info(p,side,p.paras,cur_stack_offset);
          else
            result:=create_stdcall_paraloc_info(p,side,p.paras,cur_stack_offset);
        end;

        create_funcretloc_info(p,side);
      end;

    function tcpuparamanager.create_stdcall_paraloc_info(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                               var cur_stack_offset: aword):longint;
      var
        paraloc      : pcgparalocation;
        hp           : tparavarsym;
        paracgsize   : tcgsize;
        paralen      : aint;
        paradef      : tdef;
        i            : longint;
        firstparaloc : boolean;
        paraalign    : shortint;

      begin
        result:=0;
        if paras.count=0 then
          exit;

        paraalign:=get_para_align(p.proccalloption);

        for i:=0 to paras.count-1 do
          begin
            hp:=tparavarsym(paras[i]);
            paradef:=hp.vardef;

            { syscall for AmigaOS can have already a paraloc set }
            if (vo_has_explicit_paraloc in hp.varoptions) then
              continue;

            hp.paraloc[side].reset;

            { currently only support C-style array of const }
            if (p.proccalloption in cstylearrayofconst) and
               is_array_of_const(paradef) then
              begin
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
                paradef:=cpointerdef.getreusable_no_free(paradef);
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

            hp.paraloc[side].alignment:=paraalign;
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
                paraloc^.def:=get_paraloc_def(paradef,paralen,firstparaloc);

                if (not (cs_fp_emulation in current_settings.moduleswitches)) and
                   (paradef.typ=floatdef) then
                  paraloc^.size:=int_float_cgsize(paralen)
                else
                  paraloc^.size:=int_cgsize(paralen);

                { various m68k based C ABIs used in the Unix world use a register to
                  return a struct by address. we will probably need some kind of a
                  switch to support these various ABIs when generating cdecl calls (KB) }
                if ((vo_is_funcret in hp.varoptions) and
                    (tabstractprocdef(p).proccalloption in [pocall_cdecl,pocall_cppdecl]) and
                    (target_info.system in [system_m68k_linux]) and
                    (tabstractprocdef(p).returndef.typ = recorddef)) then
                  begin
                    paraloc^.loc:=LOC_REGISTER;
                    paraloc^.register:=NR_M68K_STRUCT_RESULT_REG;
                    paralen:=0;
                    continue;
                  end;

                paraloc^.loc:=LOC_REFERENCE;
                paraloc^.reference.offset:=cur_stack_offset;
                if (side = callerside) then
                  paraloc^.reference.index:=NR_STACK_POINTER_REG
                else
                  begin
                    paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                    inc(paraloc^.reference.offset,target_info.first_parm_offset);
                  end;
                { M68K is a big-endian target }
                if (paralen<paraalign) then
                  inc(paraloc^.reference.offset,paraalign-paralen);
                inc(cur_stack_offset,align(paralen,target_info.stackalign));
                paralen := 0;

                firstparaloc:=false;
              end;
          end;
         result:=cur_stack_offset;
      end;

    function tcpuparamanager.create_register_paraloc_info(p : tabstractprocdef; side: tcallercallee;paras:tparalist;
                                                            var cur_stack_offset: aword): longint;
      var
        hp : tparavarsym;
        paradef : tdef;
        paraloc : pcgparalocation;
        paracgsize : tcgsize;
        i : integer;
        l,
        paralen,
        parareg: longint;
        addrparareg: longint;
        floatparareg: longint;
        varalign : longint;
        paraalign : shortint;
        pass : byte;
        firstparaloc,
        pushaddr : boolean;
        rt: tregistertype;
      begin
        result:=0;
        if paras.count=0 then
          exit;

        parareg:=0;
        addrparareg:=0;
        floatparareg:=0;

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
                      In case of po_delphi_nested_cc, the parent frame pointer
                      is also always passed on the stack.
                    }
                    rt:=chlcgobj.def2regtyp(paradef);
                    if (rt=R_FPUREGISTER) and
                       (floatparareg<=high(floatparasupregs)) and
                       (not pushaddr) then
                      begin
                        if pass=1 then
                          begin
                            paraloc:=hp.paraloc[side].add_location;
                            paraloc^.size:=paracgsize;
                            paraloc^.def:=paradef;
                            paraloc^.loc:=LOC_FPUREGISTER;
                            paraloc^.register:=newreg(R_FPUREGISTER,floatparasupregs[floatparareg],R_SUBNONE);
                            inc(floatparareg);
                          end;
                      end
                    else
                    if (((rt=R_INTREGISTER) and (parareg<=high(intparasupregs))) or
                        ((rt=R_ADDRESSREGISTER) and (addrparareg<=high(addrparasupregs)))) and
                       (paralen<=sizeof(aint)) and
                       (not(hp.vardef.typ in [floatdef,recorddef,arraydef]) or
                        pushaddr or
                        is_dynamic_array(hp.vardef)) and
                       (not(vo_is_parentfp in hp.varoptions) or
                        not(po_delphi_nested_cc in p.procoptions)) then
                      begin
                        if pass=1 then
                          begin
                            paraloc:=hp.paraloc[side].add_location;
                            paraloc^.size:=paracgsize;
                            paraloc^.def:=paradef;
                            paraloc^.loc:=LOC_REGISTER;
                            if (rt=R_ADDRESSREGISTER) and (addrparareg<=high(addrparasupregs)) then
                              begin
                                paraloc^.register:=newreg(R_ADDRESSREGISTER,addrparasupregs[addrparareg],R_SUBWHOLE);
                                inc(addrparareg);
                              end
                            else
                              if (rt=R_INTREGISTER) and (parareg<=high(intparasupregs)) then
                                begin
                                  paraloc^.register:=newreg(R_INTREGISTER,intparasupregs[parareg],cgsize2subreg(R_INTREGISTER,paracgsize));
                                  inc(parareg);
                                end
                              else
                                internalerror(2016051801);
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
                              paraloc^.reference.offset:=cur_stack_offset;
                              if (paralen<paraalign) then
                                inc(paraloc^.reference.offset,paraalign-paralen);
                              if side=calleeside then
                                inc(paraloc^.reference.offset,target_info.first_parm_offset);
                              cur_stack_offset:=align(cur_stack_offset+paralen,varalign);
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
                                  paraloc^.reference.offset:=cur_stack_offset;
                                  { M68K is a big-endian target }
                                  if (paralen<paraalign) then
                                    inc(paraloc^.reference.offset,paraalign-paralen);
                                  if side=calleeside then
                                    inc(paraloc^.reference.offset,target_info.first_parm_offset);
                                  cur_stack_offset:=align(cur_stack_offset+l,varalign);
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
                  else
                    ;
                end;
              end;
          end;
        result:=cur_stack_offset;
      end;



    function tcpuparamanager.parse_loc_string_to_register(var locreg: tregister; const s : string): boolean;
      begin
        locreg:=std_regnum_search(lower(s));
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


    procedure tcpuparamanager.createtempparaloc(list: TAsmList;calloption : tproccalloption;parasym : tparavarsym;can_use_final_stack_loc : boolean;var cgpara:TCGPara);
      begin
        { Never a need for temps when value is pushed (calls inside parameters
          will simply allocate even more stack space for their parameters) }
        if not(use_fixed_stack) then
          can_use_final_stack_loc:=true;
        inherited createtempparaloc(list,calloption,parasym,can_use_final_stack_loc,cgpara);
      end;

    function tcpuparamanager.create_varargs_paraloc_info(p : tabstractprocdef; side: tcallercallee; varargspara:tvarargsparalist):longint;
      var
        cur_stack_offset: aword;
      begin
        cur_stack_offset:=0;

        result:=create_stdcall_paraloc_info(p,side,p.paras,cur_stack_offset);
        if (p.proccalloption in cstylearrayofconst) then
          begin
            { just continue loading the parameters in the registers }
            if assigned(varargspara) then
              begin
                if side=callerside then
                  result:=create_stdcall_paraloc_info(p,side,varargspara,cur_stack_offset)
                else
                  internalerror(2019021923);
              end;
          end
        else
          internalerror(200410231);
        create_funcretloc_info(p,side);
      end;


begin
  paramanager:=tcpuparamanager.create;
end.
