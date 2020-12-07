{
    Copyright (c) 2013-2014 by Jonas Maebe, Florian Klaempfl and others

    AArch64 specific calling conventions

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
{ AArch64 specific calling conventions are handled by this unit
}
unit cpupara;

{$i fpcdefs.inc}

  interface

    uses
       globtype,globals,
       aasmtai,aasmdata,
       cpuinfo,cpubase,cgbase,cgutils,
       symconst,symbase,symtype,symdef,parabase,paramgr,armpara;

    type
       tcpuparamanager = class(tarmgenparamanager)
          function get_volatile_registers_int(calloption: tproccalloption): tcpuregisterset; override;
          function get_volatile_registers_fpu(calloption: tproccalloption): tcpuregisterset; override;
          function get_volatile_registers_mm(calloption: tproccalloption): tcpuregisterset; override;
          function get_saved_registers_int(calloption: tproccalloption): tcpuregisterarray; override;
          function get_saved_registers_mm(calloption: tproccalloption): tcpuregisterarray; override;
          function push_addr_param(varspez: tvarspez; def: tdef; calloption: tproccalloption): boolean; override;
          function ret_in_param(def: tdef; pd: tabstractprocdef):boolean;override;
          function create_paraloc_info(p: tabstractprocdef; side: tcallercallee):longint;override;
          function create_varargs_paraloc_info(p: tabstractprocdef; side: tcallercallee; varargspara: tvarargsparalist):longint;override;
          function get_funcretloc(p: tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;
          function param_use_paraloc(const cgpara: tcgpara): boolean; override;
         private
          curintreg,
          curmmreg: tsuperregister;
          curstackoffset: aword;

          procedure init_para_alloc_values;
          procedure alloc_para(out result: tcgpara; p: tabstractprocdef; varspez: tvarspez; side: tcallercallee; paradef: tdef; isvariadic, isdelphinestedcc: boolean);
          function getparaloc(calloption: tproccalloption; p: tdef): tcgloc;

          procedure create_paraloc_info_intern(p: tabstractprocdef; side: tcallercallee; paras: tparalist; isvariadic: boolean);
       end;

  implementation

    uses
       verbose,systems,cutils,
       rgobj,
       defutil,symsym,symtable;

    const
      RS_FIRST_INT_PARAM_SUPREG = RS_X0;
      RS_LAST_INT_PARAM_SUPREG = RS_X7;
      { Q0/D0/S0/H0/B0 all have the same superregister number }
      RS_FIRST_MM_PARAM_SUPREG = RS_D0;
      RS_LAST_MM_PARAM_SUPREG = RS_D7;


    function tcpuparamanager.get_volatile_registers_int(calloption : tproccalloption):tcpuregisterset;
      begin
        result:=VOLATILE_INTREGISTERS
      end;


    function tcpuparamanager.get_volatile_registers_fpu(calloption : tproccalloption):tcpuregisterset;
      begin
        result:=[];
      end;


    function tcpuparamanager.get_volatile_registers_mm(calloption: tproccalloption): tcpuregisterset;
      begin
        result:=VOLATILE_MMREGISTERS;
      end;


    function tcpuparamanager.get_saved_registers_int(calloption: tproccalloption): tcpuregisterarray;
      const
        saved_regs : {$ifndef VER3_0}tcpuregisterarray{$else}array[0..9] of tsuperregister{$endif} =
          (RS_X19,RS_X20,RS_X21,RS_X22,RS_X23,RS_X24,RS_X25,RS_X26,RS_X27,RS_X28);
      begin
        result:=saved_regs;
      end;


    function tcpuparamanager.get_saved_registers_mm(calloption: tproccalloption): tcpuregisterarray;
      const
        saved_mm_regs : {$ifndef VER3_0}tcpuregisterarray{$else}array[0..7] of tsuperregister{$endif} =
          (RS_D8,RS_D9,RS_D10,RS_D11,RS_D12,RS_D13,RS_D14,RS_D15);
      begin
        result:=saved_mm_regs;
      end;


    function tcpuparamanager.getparaloc(calloption: tproccalloption; p: tdef): tcgloc;
      var
        hfabasedef: tdef;
      begin
         { Later, the LOC_REFERENCE is in most cases changed into LOC_REGISTER
           if push_addr_param for the def is true
         }
         case p.typ of
            orddef:
              getparaloc:=LOC_REGISTER;
            floatdef:
              getparaloc:=LOC_MMREGISTER;
            enumdef:
              getparaloc:=LOC_REGISTER;
            pointerdef:
              getparaloc:=LOC_REGISTER;
            formaldef:
              getparaloc:=LOC_REGISTER;
            classrefdef:
              getparaloc:=LOC_REGISTER;
            recorddef:
              if not is_hfa(p,hfabasedef) then
                getparaloc:=LOC_REGISTER
              else
                getparaloc:=LOC_MMREGISTER;
            objectdef:
              getparaloc:=LOC_REGISTER;
            stringdef:
              if is_shortstring(p) or is_longstring(p) then
                getparaloc:=LOC_REFERENCE
              else
                getparaloc:=LOC_REGISTER;
            procvardef:
              getparaloc:=LOC_REGISTER;
            filedef:
              getparaloc:=LOC_REGISTER;
            arraydef:
              if not is_hfa(p,hfabasedef) then
                getparaloc:=LOC_REGISTER
              else
                getparaloc:=LOC_MMREGISTER;
            setdef:
              getparaloc:=LOC_REGISTER;
            variantdef:
              getparaloc:=LOC_REGISTER;
            { avoid problems with errornous definitions }
            errordef:
              getparaloc:=LOC_REGISTER;
            else
              internalerror(2002071001);
         end;
      end;


    function tcpuparamanager.push_addr_param(varspez: tvarspez; def :tdef; calloption: tproccalloption): boolean;
      var
        hfabasedef: tdef;
      begin
        result:=false;
        if varspez in [vs_var,vs_out,vs_constref] then
          begin
            result:=true;
            exit;
          end;
        case def.typ of
          objectdef:
            result:=is_object(def);
          recorddef:
            { ABI: any composite > 16 bytes that not a hfa/hva
              Special case: MWPascal, which passes all const parameters by
                reference for compatibility reasons
            }
            result:=
              ((varspez=vs_const) and
               (calloption=pocall_mwpascal)) or
              (not is_hfa(def,hfabasedef) and
               (def.size>16));
          variantdef,
          formaldef:
            result:=true;
          { arrays are composites and hence treated the same as records by the
            ABI (watch out for C, where an array is a pointer)
            Also: all other platforms pass const arrays by reference. Do the
              same here, because there is too much hacky code out there that
              relies on this ("array[0..0] of x" passed as const parameter and
              then indexed beyond its bounds) }
          arraydef:
            result:=
              ((calloption in cdecl_pocalls) and
               not is_dynamic_array(def)) or
              is_open_array(def) or
              is_array_of_const(def) or
              is_array_constructor(def) or
              ((tarraydef(def).highrange>=tarraydef(def).lowrange) and
               ((varspez=vs_const) or
                (not is_hfa(def,hfabasedef) and
                 (def.size>16))));
          setdef :
            result:=def.size>16;
          stringdef :
            result:=tstringdef(def).stringtype in [st_shortstring,st_longstring];
          else
            ;
        end;
      end;


    function tcpuparamanager.ret_in_param(def: tdef; pd: tabstractprocdef): boolean;
      begin
        if handle_common_ret_in_param(def,pd,result) then
          exit;
        { ABI: if the parameter would be passed in registers, it is returned
            in those registers; otherwise, it's returned by reference }
        result:=push_addr_param(vs_value,def,pd.proccalloption);
      end;


    procedure tcpuparamanager.create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist; isvariadic: boolean);
      var
        hp: tparavarsym;
        i: longint;
      begin
        for i:=0 to paras.count-1 do
          begin
            hp:=tparavarsym(paras[i]);
            { hidden function result parameter is passed in X8 (doesn't have to
              be valid on return) according to the ABI

              -- don't follow the ABI for managed types, because
               a) they are passed in registers as parameters, so we should also
                  return them in a register to be ABI-compliant (which we can't
                  because the entire compiler is built around the idea that
                  they are returned by reference, for ref-counting performance
                  and Delphi-compatibility reasons)
               b) there are hacks in the system unit that expect that you can
                  call
                    function f: com_interface;
                  as
                    procedure p(out o: obj);
                  That can only work in case we do not use x8 to return them
                  from the function, but the regular first parameter register.

              As the ABI says this behaviour is ok for C++ classes with a
              non-trivial copy constructor or destructor, it seems reasonable
              for us to do this for managed types as well.}
            if (vo_is_funcret in hp.varoptions) and
               not is_managed_type(hp.vardef) then
              begin
                hp.paraloc[side].reset;
                hp.paraloc[side].size:=OS_ADDR;
                hp.paraloc[side].alignment:=voidpointertype.alignment;
                hp.paraloc[side].intsize:=voidpointertype.size;
                hp.paraloc[side].def:=cpointerdef.getreusable_no_free(hp.vardef);
                with hp.paraloc[side].add_location^ do
                  begin
                    size:=OS_ADDR;
                    def:=hp.paraloc[side].def;
                    loc:=LOC_REGISTER;
                    register:=NR_XR;
                  end
              end
            else
              alloc_para(hp.paraloc[side],p,hp.varspez,side,hp.vardef,isvariadic,
                (vo_is_parentfp in hp.varoptions) and
                (po_delphi_nested_cc in p.procoptions));
          end;
      end;


    function  tcpuparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;
      var
        retcgsize: tcgsize;
        otherside: tcallercallee;
      begin
         if set_common_funcretloc_info(p,forcetempdef,retcgsize,result) then
           exit;

         { in this case, it must be returned in registers as if it were passed
           as the first parameter }
         init_para_alloc_values;
         { if we're on the callee side, filling the result location is actually the "callerside"
          as far passing it as a parameter value is concerned }
         if side=callerside then
           otherside:=calleeside
         else
           otherside:=callerside;
         alloc_para(result,p,vs_value,otherside,result.def,false,false);
         { sanity check (LOC_VOID for empty records) }
         if not assigned(result.location) or
            not(result.location^.loc in [LOC_REGISTER,LOC_MMREGISTER,LOC_VOID]) then
           internalerror(2014113001);
{$ifndef llvm}
         {
           According to ARM64 ABI: "If the size of the argument is less than 8 bytes then
           the size of the argument is set to 8 bytes. The effect is as if the argument
           was copied to the least significant bits of a 64-bit register and the remaining
           bits filled with unspecified values."

           Therefore at caller side force the ordinal result to be always 64-bit, so it
           will be stripped to the required size and uneeded bits are discarded.

           This is not required for iOS, where the result is zero/sign extended.
         }
         if (target_info.abi<>abi_aarch64_darwin) and
            (side=callerside) and (result.location^.loc = LOC_REGISTER) and
            (result.def.size<8) and is_ordinal(result.def) then
           begin
             result.location^.size:=OS_64;
             result.location^.def:=u64inttype;
           end;
{$endif}
      end;


    function tcpuparamanager.param_use_paraloc(const cgpara: tcgpara): boolean;
      begin
        { we always set up a stack frame -> we can always access the parameters
          this way }
        result:=
          (cgpara.location^.loc=LOC_REFERENCE) and
          not assigned(cgpara.location^.next);
      end;


    procedure tcpuparamanager.init_para_alloc_values;
      begin
        curintreg:=RS_FIRST_INT_PARAM_SUPREG;
        curmmreg:=RS_FIRST_MM_PARAM_SUPREG;
        curstackoffset:=0;
      end;


    procedure tcpuparamanager.alloc_para(out result: tcgpara; p: tabstractprocdef; varspez: tvarspez; side: tcallercallee; paradef: tdef; isvariadic, isdelphinestedcc: boolean);
      var
        hfabasedef, locdef: tdef;
        paraloc: pcgparalocation;
        paralen, stackslotlen: asizeint;
        loc: tcgloc;
        paracgsize, locsize: tcgsize;
        firstparaloc: boolean;
      begin
        result.init;

        { currently only support C-style array of const,
          there should be no location assigned to the vararg array itself }
        if (p.proccalloption in cstylearrayofconst) and
           is_array_of_const(paradef) then
          begin
            result.size:=OS_NO;
            result.def:=paradef;
            result.alignment:=std_param_align;
            result.intsize:=0;
            paraloc:=result.add_location;
            { hack: the paraloc must be valid, but is not actually used }
            paraloc^.loc:=LOC_REGISTER;
            paraloc^.register:=NR_X0;
            paraloc^.size:=OS_ADDR;
            paraloc^.def:=paradef;
            exit;
          end;

        if push_addr_param(varspez,paradef,p.proccalloption) then
          begin
            paradef:=cpointerdef.getreusable_no_free(paradef);
            loc:=LOC_REGISTER;
            paracgsize:=OS_ADDR;
            paralen:=tcgsize2size[OS_ADDR];
          end
        else
          begin
            if not is_special_array(paradef) then
              paralen:=paradef.size
            else
              paralen:=tcgsize2size[def_cgsize(paradef)];
            loc:=getparaloc(p.proccalloption,paradef);
            if (paradef.typ in [objectdef,arraydef,recorddef,setdef]) and
               not is_special_array(paradef) and
               (varspez in [vs_value,vs_const]) then
              paracgsize:=int_cgsize(paralen)
            else
              begin
                paracgsize:=def_cgsize(paradef);
                { for things like formaldef }
                if paracgsize=OS_NO then
                  begin
                    paracgsize:=OS_ADDR;
                    paralen:=tcgsize2size[OS_ADDR];
                    paradef:=voidpointertype;
                  end;
              end
          end;

          { get hfa basedef if applicable }
          if not is_hfa(paradef,hfabasedef) then
            hfabasedef:=nil;

         result.size:=paracgsize;
         result.alignment:=std_param_align;
         result.intsize:=paralen;
         result.def:=paradef;

         { empty record: skipped (explicitly defined by Apple ABI, undefined
           by general ABI; libffi also skips them in all cases) }
         if not is_special_array(paradef) and
            (paradef.size=0) then
           begin
             paraloc:=result.add_location;
             paraloc^.loc:=LOC_VOID;
             paraloc^.def:=paradef;
             paraloc^.size:=OS_NO;
             exit;
           end;

         { sufficient registers left? }
         case loc of
           LOC_REGISTER:
             begin
               { In case of po_delphi_nested_cc, the parent frame pointer
                 is always passed on the stack. }
               if isdelphinestedcc then
                 loc:=LOC_REFERENCE
               else if curintreg+((paralen-1) shr 3)>RS_LAST_INT_PARAM_SUPREG then
                 begin
                   { not enough integer registers left -> no more register
                     parameters, copy all to stack
                   }
                   curintreg:=succ(RS_LAST_INT_PARAM_SUPREG);
                   loc:=LOC_REFERENCE;
                 end;
             end;
           LOC_MMREGISTER:
             begin;
               { every hfa element must be passed in a separate register }
               if (assigned(hfabasedef) and
                   (curmmreg+((paralen-1) div hfabasedef.size)>RS_LAST_MM_PARAM_SUPREG)) or
                  (curmmreg+((paralen-1) shr 3)>RS_LAST_MM_PARAM_SUPREG) then
                 begin
                   { not enough mm registers left -> no more register
                     parameters, copy all to stack
                   }
                   curmmreg:=succ(RS_LAST_MM_PARAM_SUPREG);
                   loc:=LOC_REFERENCE;
                 end;
             end;
           else
             ;
         end;

         { allocate registers/stack locations }
         firstparaloc:=true;
         repeat
           paraloc:=result.add_location;

           { set paraloc size/def }
           if assigned(hfabasedef) then
             begin
               locsize:=def_cgsize(hfabasedef);
               locdef:=hfabasedef;
             end
           { make sure we don't lose whether or not the type is signed }
           else if (loc=LOC_REGISTER) and
                   (paradef.typ<>orddef) then
             begin
               locsize:=int_cgsize(paralen);
               locdef:=get_paraloc_def(paradef,paralen,firstparaloc);
             end
           else
             begin
               locsize:=paracgsize;
               locdef:=paradef;
             end;
           if locsize in [OS_NO,OS_128,OS_S128] then
             begin
               if paralen>4 then
                 begin
                   paraloc^.size:=OS_INT;
                   paraloc^.def:=u64inttype;
                 end
               else
                 begin
                   { for 3-byte records }
                   paraloc^.size:=OS_32;
                   paraloc^.def:=u32inttype;
                 end;
             end
           else
             begin
               paraloc^.size:=locsize;
               paraloc^.def:=locdef;
{$ifdef llvm}
               if not is_ordinal(paradef) then
                 begin
                   case locsize of
                     OS_8,OS_16,OS_32:
                       begin
                         paraloc^.size:=OS_64;
                         paraloc^.def:=u64inttype;
                       end;
                     OS_S8,OS_S16,OS_S32:
                       begin
                         paraloc^.size:=OS_S64;
                         paraloc^.def:=s64inttype;
                       end;
                     OS_F32:
                       begin
                         paraloc^.size:=OS_F32;
                         paraloc^.def:=s32floattype;
                       end;
                     OS_F64:
                       begin
                         paraloc^.size:=OS_F64;
                         paraloc^.def:=s64floattype;
                       end;
                     else
                       begin
                         if is_record(locdef) or
                            is_set(locdef) or
                            ((locdef.typ=arraydef) and
                             not is_special_array(locdef)) then
                           begin
                             paraloc^.size:=OS_64;
                             paraloc^.def:=u64inttype;
                           end
                       end;
                   end;
                 end;
{$endif llvm}
             end;

           { paraloc loc }
           paraloc^.loc:=loc;

           { assign register/stack address }
           case loc of
             LOC_REGISTER:
               begin
                 paraloc^.register:=newreg(R_INTREGISTER,curintreg,cgsize2subreg(R_INTREGISTER,paraloc^.size));
                 inc(curintreg);
                 dec(paralen,tcgsize2size[paraloc^.size]);

                 { "The general ABI specifies that it is the callee's
                    responsibility to sign or zero-extend arguments having fewer
                    than 32 bits, and that unused bits in a register are
                    unspecified. In iOS, however, the caller must perform such
                    extensions, up to 32 bits."
                    Zero extend an argument at caller side for iOS and
                    ignore the argument's unspecified high bits at callee side for
                    all other platforms. }
                 if (paradef.size<4) and is_ordinal(paradef) then
                   begin
                     if target_info.abi=abi_aarch64_darwin then
                       begin
                         if side=callerside then
                           begin
                             paraloc^.size:=OS_32;
                             paraloc^.def:=u32inttype;
                           end;
                       end
{$ifndef llvm}
                     else
                       begin
                         if side=calleeside then
                           begin
                             paraloc^.size:=OS_32;
                             paraloc^.def:=u32inttype;
                           end;
                       end;
{$endif llvm}
                   end;

                 { in case it's a composite, "The argument is passed as though
                   it had been loaded into the registers from a double-word-
                   aligned address with an appropriate sequence of LDR
                   instructions loading consecutive registers from memory" ->
                   in case of big endian, values in not completely filled
                   registers must be shifted to the top bits }
                 if (target_info.endian=endian_big) and
                    not(paraloc^.size in [OS_64,OS_S64]) and
                    (paradef.typ in [setdef,recorddef,arraydef,objectdef]) then
                   paraloc^.shiftval:=-(8-tcgsize2size[paraloc^.size])*8;
               end;
             LOC_MMREGISTER:
               begin
                 paraloc^.register:=newreg(R_MMREGISTER,curmmreg,cgsize2subreg(R_MMREGISTER,paraloc^.size));
                 inc(curmmreg);
                 dec(paralen,tcgsize2size[paraloc^.size]);
               end;
             LOC_REFERENCE:
               begin
                  paraloc^.size:=paracgsize;
                  paraloc^.loc:=LOC_REFERENCE;
                  if assigned(hfabasedef) then
                    paraloc^.def:=carraydef.getreusable_no_free(hfabasedef,paralen div hfabasedef.size)
                  else
                    paraloc^.def:=paradef;

                  { the current stack offset may not be properly aligned in
                    case we're on Darwin and have allocated a non-variadic argument
                    < 8 bytes previously }
                  if target_info.abi=abi_aarch64_darwin then
                    curstackoffset:=align(curstackoffset,paraloc^.def.alignment);

                  { on Darwin, non-variadic arguments take up their actual size
                    on the stack; on other platforms, they take up a multiple of
                    8 bytes }
                  if (target_info.abi=abi_aarch64_darwin) and
                     not isvariadic then
                    stackslotlen:=paralen
                  else
                    stackslotlen:=align(paralen,8);

                  { from the ABI: if arguments occupy partial stack space, they
                    have to occupy the lowest significant bits of a register
                    containing that value which is then stored to memory ->
                    in case of big endian, skip the alignment bytes (if any) }
                  if target_info.endian=endian_little then
                    paraloc^.reference.offset:=curstackoffset
                  else
                    paraloc^.reference.offset:=curstackoffset+stackslotlen-paralen;
                  if side=callerside then
                    paraloc^.reference.index:=NR_STACK_POINTER_REG
                  else
                    begin
                      paraloc^.reference.index:=NR_FRAME_POINTER_REG;
                      inc(paraloc^.reference.offset,16);
                    end;
                  inc(curstackoffset,stackslotlen);
                  paralen:=0
               end;
             else
               internalerror(2002071002);
           end;
         firstparaloc:=false;
         { <=0 for sign/zero-extended locations }
         until paralen<=0;
      end;


    function tcpuparamanager.create_paraloc_info(p: tabstractprocdef; side: tcallercallee):longint;
      begin
        init_para_alloc_values;

        create_paraloc_info_intern(p,side,p.paras,false);
        result:=curstackoffset;

        create_funcretloc_info(p,side);
     end;


    function tcpuparamanager.create_varargs_paraloc_info(p: tabstractprocdef; side: tcallercallee; varargspara: tvarargsparalist):longint;
      begin
        init_para_alloc_values;

        { non-variadic parameters }
        create_paraloc_info_intern(p,side,p.paras,false);
        if p.proccalloption in cstylearrayofconst then
          begin
            { on Darwin, we cannot use any registers for variadic parameters }
            if target_info.abi=abi_aarch64_darwin then
              begin
                curintreg:=succ(RS_LAST_INT_PARAM_SUPREG);
                curmmreg:=succ(RS_LAST_MM_PARAM_SUPREG);
              end;
            { continue loading the parameters  }
            if assigned(varargspara) then
              begin
                if side=callerside then
                  create_paraloc_info_intern(p,side,varargspara,true)
                else
                  internalerror(2019021916);
              end;
            result:=curstackoffset;
          end
        else
          internalerror(2004102303);

        create_funcretloc_info(p,side);
      end;

begin
   paramanager:=tcpuparamanager.create;
end.
