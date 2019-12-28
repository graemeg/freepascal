{
    Copyright (c) 2002 by Florian Klaempfl

    PowerPC64 specific calling conventions

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

{$I fpcdefs.inc}

interface

uses
  globtype,
  aasmtai,aasmdata,
  cpubase,
  symconst, symtype, symdef, symsym,
  paramgr, parabase, cgbase, cgutils;

type
  tcpuparamanager = class(tparamanager)
    function get_volatile_registers_int(calloption: tproccalloption):
      tcpuregisterset; override;
    function get_volatile_registers_fpu(calloption: tproccalloption):
      tcpuregisterset; override;
    function get_saved_registers_int(calloption: tproccalloption):
      tcpuregisterarray; override;
    function push_addr_param(varspez: tvarspez; def: tdef; calloption:
      tproccalloption): boolean; override;
    function ret_in_param(def: tdef; pd: tabstractprocdef): boolean; override;

    procedure getcgtempparaloc(list: TAsmList; pd : tabstractprocdef; nr: longint; var cgpara: tcgpara); override;
    function create_paraloc_info(p: tabstractprocdef; side: tcallercallee): longint; override;
    function create_varargs_paraloc_info(p: tabstractprocdef; side: tcallercallee; varargspara: tvarargsparalist): longint; override;
    function get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;

  private
    procedure init_values(var curintreg, curfloatreg, curmmreg: tsuperregister;
      var cur_stack_offset: aword);
    function create_paraloc_info_intern(p: tabstractprocdef; side:
      tcallercallee; paras: tparalist;
      var curintreg, curfloatreg, curmmreg: tsuperregister; var
        cur_stack_offset: aword; isVararg : boolean): longint;
    function parseparaloc(p: tparavarsym; const s: string): boolean; override;
    procedure create_paraloc_for_def(var para: TCGPara; varspez: tvarspez; paradef: tdef; var nextfloatreg, nextintreg: tsuperregister; var stack_offset: aword; const isVararg, forceintmem: boolean; const side: tcallercallee; const p: tabstractprocdef);
  end;

implementation

uses
  verbose, systems,
  defutil,symtable,symcpu,
  procinfo, cpupi;

function tcpuparamanager.get_volatile_registers_int(calloption:
  tproccalloption): tcpuregisterset;
begin
  result := [RS_R0,RS_R3..RS_R12];
  if (target_info.system = system_powerpc64_darwin) then
    include(result,RS_R2);
end;

function tcpuparamanager.get_volatile_registers_fpu(calloption:
  tproccalloption): tcpuregisterset;
begin
  result := [RS_F0..RS_F13];
end;

function tcpuparamanager.get_saved_registers_int(calloption: tproccalloption):
  tcpuregisterarray;
const
  saved_regs: {$ifndef VER3_0}tcpuregisterarray{$else}array[0..17] of tsuperregister{$endif} = (
    RS_R14, RS_R15, RS_R16, RS_R17, RS_R18, RS_R19,
    RS_R20, RS_R21, RS_R22, RS_R23, RS_R24, RS_R25,
    RS_R26, RS_R27, RS_R28, RS_R29, RS_R30, RS_R31
    );
begin
  result:=saved_regs;
end;

procedure tcpuparamanager.getcgtempparaloc(list: TAsmList; pd : tabstractprocdef; nr: longint; var cgpara: tcgpara);
var
  paraloc: pcgparalocation;
  psym: tparavarsym;
  pdef: tdef;
begin
  psym:=tparavarsym(pd.paras[nr-1]);
  pdef:=psym.vardef;
  if push_addr_param(psym.varspez,pdef,pd.proccalloption) then
    pdef:=cpointerdef.getreusable_no_free(pdef);
  cgpara.reset;
  cgpara.size := def_cgsize(pdef);
  cgpara.intsize := tcgsize2size[cgpara.size];
  cgpara.alignment := get_para_align(pd.proccalloption);
  cgpara.def:=pdef;
  paraloc := cgpara.add_location;
  with paraloc^ do begin
    size := def_cgsize(pdef);
    def := pdef;
    if (nr <= 8) then begin
      if (nr = 0) then
        internalerror(200309271);
      loc := LOC_REGISTER;
      register := newreg(R_INTREGISTER, RS_R2 + nr, R_SUBWHOLE);
    end else begin
      loc := LOC_REFERENCE;
      paraloc^.reference.index := NR_STACK_POINTER_REG;
      reference.offset := sizeof(aint) * (nr - 8);
    end;
  end;
end;

function getparaloc(p: tdef): tcgloc;

begin
  { Later, the LOC_REFERENCE is in most cases changed into LOC_REGISTER
    if push_addr_param for the def is true
  }
  case p.typ of
    orddef:
      result := LOC_REGISTER;
    floatdef:
      result := LOC_FPUREGISTER;
    enumdef:
      result := LOC_REGISTER;
    pointerdef:
      result := LOC_REGISTER;
    formaldef:
      result := LOC_REGISTER;
    classrefdef:
      result := LOC_REGISTER;
    procvardef,
    recorddef:
      result := LOC_REGISTER;
    objectdef:
      if is_object(p) then
        result := LOC_REFERENCE
      else
        result := LOC_REGISTER;
    stringdef:
      if is_shortstring(p) or is_longstring(p) then
        result := LOC_REFERENCE
      else
        result := LOC_REGISTER;
    filedef:
      result := LOC_REGISTER;
    arraydef:
      if is_dynamic_array(p) then
        getparaloc:=LOC_REGISTER
      else
        result := LOC_REFERENCE;
    setdef:
      if is_smallset(p) then
        result := LOC_REGISTER
      else
        result := LOC_REFERENCE;
    variantdef:
      result := LOC_REFERENCE;
    { avoid problems with errornous definitions }
    errordef:
      result := LOC_REGISTER;
  else
    internalerror(2002071001);
  end;
end;

function tcpuparamanager.push_addr_param(varspez: tvarspez; def: tdef;
  calloption: tproccalloption): boolean;
begin
  result := false;
  { var,out,constref always require address }
  if varspez in [vs_var, vs_out, vs_constref] then
  begin
    result := true;
    exit;
  end;
  case def.typ of
    variantdef,
    formaldef:
      result := true;
    procvardef,
    recorddef:
      result :=
        (varspez = vs_const) and
        (
         (
          (not (calloption in cdecl_pocalls) and
          (def.size > 8))
         ) or
         (calloption = pocall_mwpascal)
        );
    arraydef:
      result := (tarraydef(def).highrange >= tarraydef(def).lowrange) or
        is_open_array(def) or
        is_array_of_const(def) or
        is_array_constructor(def);
    objectdef:
      result := is_object(def);
    setdef:
      result := not is_smallset(def);
    stringdef:
      result := tstringdef(def).stringtype in [st_shortstring, st_longstring];
    else
      ;
  end;
end;

function tcpuparamanager.ret_in_param(def: tdef; pd: tabstractprocdef): boolean;
  var
    tmpdef: tdef;
  begin
    if handle_common_ret_in_param(def,pd,result) then
      exit;

    { general rule: passed in registers -> returned in registers }
    result:=push_addr_param(vs_value,def,pd.proccalloption);

    case target_info.abi of
      { elfv2: non-homogeneous aggregate larger than 2 doublewords or a
        homogeneous aggregate with more than eight registers are returned by
        reference }
      abi_powerpc_elfv2:
        begin
          if not result then
            begin
              if (def.typ=recorddef) then
                begin
                  if tcpurecorddef(def).has_single_type_elfv2(tmpdef) then
                    begin
                      if def.size>8*tmpdef.size then
                        result:=true
                    end
                  else if def.size>2*sizeof(aint) then
                    result:=true;
                end
              else if (def.typ=arraydef) then
                begin
                  if tcpuarraydef(def).has_single_type_elfv2(tmpdef) then
                    begin
                      if def.size>8*tmpdef.size then
                        result:=true
                    end
                  else if def.size>2*sizeof(aint) then
                    result:=true;
                end;
            end;
        end;
      { sysv/aix: any non-scalar/non-floating point is returned by reference }
      abi_powerpc_sysv,
      abi_powerpc_aix:
        begin
          case def.typ of
            procvardef:
              result:=def.size>8;
            recorddef:
              result:=true;
            else
              ;
          end;
        end;
      { Darwin: if completely passed in registers -> returned by registers;
        i.e., if part is passed via memory because there are not enough
        registers, return via memory }
      abi_powerpc_darwin:
        begin
          case def.typ of
            recorddef:
              begin
                { todo: fix once the Darwin/ppc64 abi is fully implemented, as it
                  requires individual fields to be passed in individual registers,
                  so a record with 9 bytes may need to be passed via memory }
                if def.size>8*sizeof(aint) then
                  result:=true;
              end;
            else
              ;
          end;
        end;
      else
        internalerror(2019051030);
    end;
  end;

procedure tcpuparamanager.init_values(var curintreg, curfloatreg, curmmreg:
  tsuperregister; var cur_stack_offset: aword);
begin
  case target_info.abi of
    abi_powerpc_elfv2:
      cur_stack_offset := 32;
    else
      cur_stack_offset := 48;
  end;
  curintreg := RS_R3;
  curfloatreg := RS_F1;
  curmmreg := RS_M2;
end;

function tcpuparamanager.get_funcretloc(p : tabstractprocdef; side:
  tcallercallee; forcetempdef: tdef): tcgpara;
var
  paraloc: pcgparalocation;
  retcgsize: tcgsize;
  nextfloatreg, nextintreg, nextmmreg: tsuperregister;
  stack_offset: aword;
begin
  if set_common_funcretloc_info(p,forcetempdef,retcgsize,result) then
    exit;

  { on Darwin and with ELFv2, results are returned the same way as they are
    passed }
  if target_info.abi in [abi_powerpc_elfv2,abi_powerpc_darwin] then
    begin
      init_values(nextintreg,nextfloatreg,nextmmreg,stack_offset);
      create_paraloc_for_def(result,vs_value,result.def,nextfloatreg,nextintreg,stack_offset,false,false,side,p);
    end
  else
    begin
      { for AIX and ELFv1, the situation is simpler: always just one register }
      paraloc:=result.add_location;
      { Return in FPU register? }
      if result.def.typ=floatdef then
        begin
          paraloc^.loc:=LOC_FPUREGISTER;
          paraloc^.register:=NR_FPU_RESULT_REG;
          paraloc^.size:=retcgsize;
          paraloc^.def:=result.def;
        end
      else
       { Return in register }
        begin
           paraloc^.loc:=LOC_REGISTER;
           if side=callerside then
             paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RESULT_REG,cgsize2subreg(R_INTREGISTER,retcgsize))
           else
             paraloc^.register:=newreg(R_INTREGISTER,RS_FUNCTION_RETURN_REG,cgsize2subreg(R_INTREGISTER,retcgsize));
           paraloc^.size:=retcgsize;
           paraloc^.def:=result.def;
         end;
    end;
end;

function tcpuparamanager.create_paraloc_info(p: tabstractprocdef; side:
  tcallercallee): longint;
var
  cur_stack_offset: aword;
  curintreg, curfloatreg, curmmreg : tsuperregister;
begin
  init_values(curintreg, curfloatreg, curmmreg, cur_stack_offset);

  result := create_paraloc_info_intern(p, side, p.paras, curintreg, curfloatreg,
    curmmreg, cur_stack_offset, false);

  create_funcretloc_info(p, side);
end;

function tcpuparamanager.create_paraloc_info_intern(p: tabstractprocdef; side:
  tcallercallee; paras: tparalist;
  var curintreg, curfloatreg, curmmreg: tsuperregister; var cur_stack_offset:
  aword; isVararg : boolean): longint;
var
  nextintreg, nextfloatreg, nextmmreg : tsuperregister;
  i: integer;
  hp: tparavarsym;
  paraloc: pcgparalocation;
  delphi_nestedfp: boolean;

begin
{$IFDEF extdebug}
  if po_explicitparaloc in p.procoptions then
    internalerror(200411141);
{$ENDIF extdebug}

  result := 0;
  nextintreg := curintreg;
  nextfloatreg := curfloatreg;
  nextmmreg := curmmreg;

  for i := 0 to paras.count - 1 do begin
    hp := tparavarsym(paras[i]);

    { Syscall for Morphos can have already a paraloc set; not supported on ppc64 }
    if (vo_has_explicit_paraloc in hp.varoptions) then begin
      internalerror(200412153);
    end;

    { currently only support C-style array of const }
    if (p.proccalloption in cstylearrayofconst) and
      is_array_of_const(hp.vardef) then begin
      paraloc := hp.paraloc[side].add_location;
      { hack: the paraloc must be valid, but is not actually used }
      paraloc^.loc := LOC_REGISTER;
      paraloc^.register := NR_R0;
      paraloc^.size := OS_ADDR;
      paraloc^.def := voidpointertype;
      break;
    end;
    delphi_nestedfp:=(vo_is_parentfp in hp.varoptions) and (po_delphi_nested_cc in p.procoptions);
    create_paraloc_for_def(hp.paraloc[side], hp.varspez, hp.vardef,
      nextfloatreg, nextintreg, cur_stack_offset, isVararg, delphi_nestedfp, side, p);
  end;

  curintreg := nextintreg;
  curfloatreg := nextfloatreg;
  curmmreg := nextmmreg;
  result := cur_stack_offset;
end;

procedure tcpuparamanager.create_paraloc_for_def(var para: TCGPara; varspez: tvarspez; paradef: tdef; var nextfloatreg, nextintreg: tsuperregister; var stack_offset: aword; const isVararg, forceintmem: boolean; const side: tcallercallee; const p: tabstractprocdef);
var
  paracgsize: tcgsize;
  loc: tcgloc;
  paraloc: pcgparalocation;
  { def to use for all paralocs if <> nil }
  alllocdef,
  { def to use for the current paraloc }
  locdef,
  tmpdef: tdef;
  paralen: aint;
  parashift: byte;
  tailpadding,
  firstparaloc,
  paraaligned: boolean;
begin
  alllocdef:=nil;
  locdef:=nil;
  parashift := 0;
  para.reset;
  { should the tail be shifted into the most significant bits? }
  tailpadding:=false;
  { have we ensured that the next parameter location will be aligned to the
    next 8 byte boundary? }
  paraaligned:=false;
  if push_addr_param(varspez, paradef, p.proccalloption) then begin
    paradef := cpointerdef.getreusable_no_free(paradef);
    loc := LOC_REGISTER;
    paracgsize := OS_ADDR;
    paralen := tcgsize2size[OS_ADDR];
  end else begin
    if not is_special_array(paradef) then
      paralen := paradef.size
    else
      paralen := tcgsize2size[def_cgsize(paradef)];
    { default rules:
      * integer parameters sign/zero-extended to 64 bit
      * floating point register used -> skip equivalent GP register
      * floating point parameters passed as is (32/64 bit)
      * floating point parameters to variable arguments -> in int registers
      * aggregates passed in consecutive integer registers
      * all *aggregate* data in integer registers exactly mirrors the data
        in memory -> on big endian it's left aligned (passed in most
        significant part of the 64 bit word if it's < 64 bit), on little
        endian it's right aligned (least significant part of the 64 bit
        word)

      special rules:

implemented
   |
   | * AIX/ELFv1/SysV ppc64 ABI (big endian only):
   x    a) single precision floats are stored in the second word of a 64 bit
           location when passed on the stack
   x    b) aggregate with 1 floating point element passed like a floating
           point parameter of the same size
   x    c) aggregates smaller than 64 bit are aligned in least significant bits
           of a single 64bit location (incl. register) (AIX exception: it puts
           them in the most significant bits)

      * ELFv2 ppc64 ABI:
   x    a) so-called "homogeneous" aggregates, i.e. struct, arrays, or unions
           that (recursively) contain only elements of the same floating-
           point or vector type, are passed as if those elements were passed as
           separate arguments. This is done for up to 8 such elements.
   x    b) other than a), it's the same as the AIX ppc64 ABI

      * Darwin ppc64 ABI:

      - as in the general case, aggregates in registers mirror their place in
        memory, so if e.g. a struct starts with a 32 bit integer, it's
        placed in the upper 32 bits of a the corresponding register. A plain
        32 bit integer para is however passed in the lower 32 bits, since it
        is promoted to a 64 bit int first (see below)

   x    a) aggregates with sizes 1, 2 and 4 bytes are padded with 0s on the left
          (-> aligned in least significant bits of 64 bit word on big endian) to
          a multiple of *4 bytes* (when passed by memory, don't occupy 8 bytes)
   x    b) other aggregates are padded with 0s on the right (-> aligned in most
           signifcant bits of 64 bit word of integer register) to a multiple of
           *4 bytes*
   x    c) all floating pointer parameters (not in aggregates) are promoted to
           double (doesn't seem to be correct: 8 bytes are reserved in the
           stack frame, but the compiler still stores a single in it (in the
           lower 4 bytes -- like with SysV a) )
   x    d) all integer parameters (not in aggregates) are promoted to 64 bit
  (x)   e) aggregates (incl. arrays) of exactly 16 bytes passed in two integer
           registers
        f) floats in *structures without unions* are processed per rule c)
           (similar for vector fields)
        g) other fields in *structures without unions* are processed
           recursively according to e) / f) if they are aggragates, and h)
           otherwise (i.e, without promotion!)
  (x)   h) everything else (structures with unions and size<>16, arrays with
           size<>16, ...) is passed "normally" in integer registers
    }
    { ELFv2 a) }
    if (target_info.abi=abi_powerpc_elfv2) and
       (((paradef.typ=recorddef) and
         tcpurecorddef(paradef).has_single_type_elfv2(tmpdef)) or
        ((paradef.typ=arraydef) and
         tcpuarraydef(paradef).has_single_type_elfv2(tmpdef))) and
       (tmpdef.typ=floatdef { or vectordef }) and
       (paradef.size<=(8*tmpdef.size)) then
      begin
        alllocdef:=tmpdef;
        loc:=getparaloc(alllocdef);
        paracgsize:=def_cgsize(paradef);
      end
    { AIX/ELFv1 b) }
    else if (target_info.abi in [abi_powerpc_aix,abi_powerpc_sysv]) and
       (paradef.typ=recorddef) and
       tabstractrecordsymtable(tabstractrecorddef(paradef).symtable).has_single_field(tmpdef) and
       (tmpdef.typ=floatdef) then
      begin
        paradef:=tmpdef;
        loc:=getparaloc(paradef);
        paracgsize:=def_cgsize(paradef)
      end
    else if (((paradef.typ=arraydef) and not
         is_special_array(paradef)) or
        (paradef.typ=recorddef)) then
      begin
        { should handle Darwin f/g/h) now, but can't model that yet }

        { general rule: aggregate data is aligned in the most significant bits
          except for ELFv1 c) and Darwin a) }
        if (target_info.endian=endian_big) and
           ((target_info.abi in [abi_powerpc_aix,abi_powerpc_elfv2]) or
            ((target_info.abi=abi_powerpc_sysv) and
             (paralen>8)) or
            ((target_info.abi=abi_powerpc_darwin) and
             not(paralen in [1,2,4]))) then
          tailpadding:=true
        { if we don't add tailpadding on the caller side, the callee will have
          to shift the value in the register before it can store it to memory }
        else if (target_info.endian=endian_big) and
           (paralen in [3,5,6,7]) then
          parashift:=(8-paralen)*8;
        { general fallback rule: pass aggregate types in integer registers
          without special adjustments (incl. Darwin h) }
        loc:=LOC_REGISTER;
        paracgsize:=int_cgsize(paralen);
      end
    else
      begin
        loc:=getparaloc(paradef);
        paracgsize:=def_cgsize(paradef);
        { for things like formaldef }
        if (paracgsize=OS_NO) then
          begin
            paracgsize:=OS_ADDR;
            paralen:=tcgsize2size[OS_ADDR];
          end;
      end
  end;

  { patch FPU values into integer registers if we are processing varargs }
  if (isVararg) and (paradef.typ = floatdef) then begin
    loc := LOC_REGISTER;
    if paracgsize = OS_F64 then
      paracgsize := OS_64
    else
      paracgsize := OS_32;
  end;

  { AIX/SysV a), Darwin c) -> skip 4 bytes in the stack frame }
 if (target_info.endian=endian_big) and
    (paradef.typ=floatdef) and
    (tfloatdef(paradef).floattype=s32real) and
    (nextfloatreg>RS_F13) then
   begin
     inc(stack_offset,4);
     paraaligned:=true;
   end;

 { Darwin d) }
  if (target_info.abi=abi_powerpc_darwin) and
     (paradef.typ in [orddef,enumdef]) and
     (paralen<8) and
     { we don't have to sign/zero extend the lower 8/16/32 bit on the callee
       side since it's done on the caller side; however, if the value is
       passed via memory, we do have to modify the stack offset since this
       is big endian and otherwise we'll load/store the wrong bytes) }
     ((side=callerside) or
      forceintmem or
      (nextintreg>RS_R10)) then
     begin
      if side=callerside then
        begin
          paralen:=8;
          paradef:=s64inttype;
          paracgsize:=OS_S64;
        end
      else
        begin
          inc(stack_offset,8-paralen);
          paraaligned:=true;
        end;
    end;

  para.alignment := std_param_align;
  para.size := paracgsize;
  para.intsize := paralen;
  para.def := paradef;
  if (paralen = 0) then
    if (paradef.typ = recorddef) then begin
      paraloc := para.add_location;
      paraloc^.loc := LOC_VOID;
    end else
      internalerror(2005011310);
  if not assigned(alllocdef) then
    locdef:=paradef
  else
    begin
      locdef:=alllocdef;
      paracgsize:=def_cgsize(locdef);
    end;
  firstparaloc:=true;
  { can become < 0 for e.g. 3-byte records }
  while (paralen > 0) do begin
    paraloc := para.add_location;
    { In case of po_delphi_nested_cc, the parent frame pointer
      is always passed on the stack. }
    if (loc = LOC_REGISTER) and
       (nextintreg <= RS_R10) and
       not forceintmem then begin
      paraloc^.loc := loc;
      paraloc^.shiftval := parashift;

      { make sure we don't lose whether or not the type is signed }
      if (paracgsize <> OS_NO) and
         (paradef.typ <> orddef) and
         not assigned(alllocdef) then
        begin
          paracgsize := int_cgsize(paralen);
          locdef:=get_paraloc_def(paradef, paralen, firstparaloc);
        end;

      { Partial aggregate data may have to be left-aligned. If so, add tail
        padding }
      if tailpadding and
         (paralen < sizeof(aint)) then
        begin
          paraloc^.shiftval := (sizeof(aint)-paralen)*(-8);
          paraloc^.size := OS_INT;
          paraloc^.def := u64inttype;
        end
      else if (paracgsize in [OS_NO, OS_128, OS_S128]) then
        begin
          if (paralen>4) or
             (parashift<>0) then
            begin
              paraloc^.size := OS_INT;
              paraloc^.def := osuinttype;
            end
          else
            begin
              { for 3-byte records aligned in the lower bits of register }
              paraloc^.size := OS_32;
              paraloc^.def := u32inttype;
            end;
        end
      else
        begin
          paraloc^.size := paracgsize;
          paraloc^.def := locdef;
        end;

      paraloc^.register := newreg(R_INTREGISTER, nextintreg, R_SUBNONE);
      inc(nextintreg);
      dec(paralen, tcgsize2size[paraloc^.size]);

      inc(stack_offset, sizeof(pint));
    end else if (loc = LOC_FPUREGISTER) and
      (nextfloatreg <= RS_F13) then begin
      paraloc^.loc := loc;
      paraloc^.size := paracgsize;
      paraloc^.def := locdef;
      paraloc^.register := newreg(R_FPUREGISTER, nextfloatreg, R_SUBWHOLE);
      { the PPC64 ABI says that the GPR index is increased for every parameter, no matter
      which type it is stored in }
      inc(nextintreg);
      inc(nextfloatreg);
      dec(paralen, tcgsize2size[paraloc^.size]);

      inc(stack_offset, tcgsize2size[OS_FLOAT]);
    end else if (loc = LOC_MMREGISTER) then begin
      { Altivec not supported }
      internalerror(200510192);
    end else begin
      { either LOC_REFERENCE, or one of the above which must be passed on the
      stack because of insufficient registers }
      paraloc^.loc := LOC_REFERENCE;
      case loc of
        LOC_FPUREGISTER:
          begin
            paraloc^.size:=int_float_cgsize(paralen);
            case paraloc^.size of
              OS_F32: paraloc^.def:=s32floattype;
              OS_F64: paraloc^.def:=s64floattype;
              else
                internalerror(2013060122);
            end;
          end;
        LOC_REGISTER,
        LOC_REFERENCE:
          begin
            paraloc^.size:=int_cgsize(paralen);
            paraloc^.def:=get_paraloc_def(paradef, paralen, firstparaloc);
          end;
        else
          internalerror(2006011101);
      end;
      if (side = callerside) then
        paraloc^.reference.index := NR_STACK_POINTER_REG
      else begin
        { during procedure entry, NR_OLD_STACK_POINTER_REG contains the old stack pointer }
        paraloc^.reference.index := NR_OLD_STACK_POINTER_REG;
        { create_paraloc_info_intern might be also called when being outside of
          code generation so current_procinfo might be not set }
        if assigned(current_procinfo) then
          tcpuprocinfo(current_procinfo).needs_frame_pointer := true;
      end;
      paraloc^.reference.offset := stack_offset;

      { align temp contents to next register size }
      if not paraaligned then
        inc(stack_offset, align(paralen, 8))
      else
        inc(stack_offset, paralen);
      paralen := 0;
    end;
    firstparaloc:=false;
  end;
end;

function tcpuparamanager.create_varargs_paraloc_info(p: tabstractprocdef; side: tcallercallee;
  varargspara: tvarargsparalist): longint;
var
  cur_stack_offset: aword;
  parasize, l: longint;
  curintreg, firstfloatreg, curfloatreg, curmmreg: tsuperregister;
  i: integer;
  hp: tparavarsym;
  paraloc: pcgparalocation;
begin
  init_values(curintreg, curfloatreg, curmmreg, cur_stack_offset);
  firstfloatreg := curfloatreg;

  result := create_paraloc_info_intern(p, side, p.paras, curintreg,
    curfloatreg, curmmreg, cur_stack_offset, false);
  if (p.proccalloption in cstylearrayofconst) then
    begin
      { just continue loading the parameters in the registers }
      if assigned(varargspara) then
        begin
          if side=callerside then
            result := create_paraloc_info_intern(p, side, varargspara, curintreg,
              curfloatreg, curmmreg, cur_stack_offset, true)
          else
            internalerror(2019021920);
        end;
      { varargs routines have to reserve at least 64 bytes for the PPC64 ABI }
      if (result < 64) then
        result := 64;
    end
  else
    internalerror(2019021911);
  if curfloatreg <> firstfloatreg then
    include(varargspara.varargsinfo, va_uses_float_reg);
  create_funcretloc_info(p, side);
end;

function tcpuparamanager.parseparaloc(p: tparavarsym; const s: string): boolean;
begin
  { not supported/required for PowerPC64-linux target }
  internalerror(200404182);
  result := true;
end;


begin
  paramanager := tcpuparamanager.create;
end.

