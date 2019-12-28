{
    Copyright (c) 1998-2010 by Florian Klaempfl, Jonas Maebe

    Calling conventions for the JVM

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

      { tcpuparamanager }

      tcpuparamanager=class(TParaManager)
        function  get_saved_registers_int(calloption: tproccalloption): tcpuregisterarray;override;
        function  push_high_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
        function  keep_para_array_range(varspez: tvarspez; def: tdef; calloption: tproccalloption): boolean; override;
        function  push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
        function  push_copyout_param(varspez: tvarspez; def: tdef; calloption: tproccalloption): boolean; override;
        function  push_size(varspez: tvarspez; def: tdef; calloption: tproccalloption): longint;override;
        {Returns a structure giving the information on the storage of the parameter
        (which must be an integer parameter)
        @param(nr Parameter number of routine, starting from 1)}
        procedure getcgtempparaloc(list: TAsmList; pd : tabstractprocdef; nr : longint; var cgpara : tcgpara);override;
        function  create_paraloc_info(p : TAbstractProcDef; side: tcallercallee):longint;override;
        function  create_varargs_paraloc_info(p : tabstractprocdef; side: tcallercallee; varargspara:tvarargsparalist):longint;override;
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;
        function param_use_paraloc(const cgpara: tcgpara): boolean; override;
        function ret_in_param(def:tdef;pd:tabstractprocdef):boolean;override;
        function is_stack_paraloc(paraloc: pcgparalocation): boolean;override;
      private
        procedure create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                                             var parasize:longint);
      end;

implementation

    uses
      cutils,verbose,systems,
      defutil,jvmdef,
      aasmcpu,
      hlcgobj;


    procedure tcpuparamanager.getcgtempparaloc(list: TAsmList; pd : tabstractprocdef; nr : longint; var cgpara : tcgpara);
      begin
        { not yet implemented/used }
        internalerror(2010121001);
      end;

    function tcpuparamanager.get_saved_registers_int(calloption: tproccalloption): tcpuregisterarray;
      const
        { dummy, not used for JVM }
        saved_regs: {$ifndef VER3_0}tcpuregisterarray{$else}array [0..0] of tsuperregister{$endif} = (RS_NO);
      begin
        result:=saved_regs;
      end;

    function tcpuparamanager.push_high_param(varspez: tvarspez; def: tdef; calloption: tproccalloption): boolean;
      begin
        { we don't need a separate high parameter, since all arrays in Java
          have an implicit associated length }
        if not is_open_array(def) and
           not is_array_of_const(def) then
          result:=inherited
        else
          result:=false;
      end;


    function tcpuparamanager.keep_para_array_range(varspez: tvarspez; def: tdef; calloption: tproccalloption): boolean;
      begin
        { even though these don't need a high parameter (see push_high_param),
          we do have to keep the original parameter's array length because it's
          used by the compiler (to determine the size of the array to construct
          to pass to an array of const parameter)  }
        if not is_array_of_const(def) then
          result:=inherited
        else
          result:=true;
      end;


    { true if a parameter is too large to copy and only the address is pushed }
    function tcpuparamanager.push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;
      begin
        result:=
          jvmimplicitpointertype(def) or
          ((def.typ=formaldef) and
           not(varspez in [vs_var,vs_out]));
      end;


    function tcpuparamanager.push_copyout_param(varspez: tvarspez; def: tdef; calloption: tproccalloption): boolean;
      begin
        { in principle also for vs_constref, but since we can't have real
          references, that won't make a difference }
        result:=
          (varspez in [vs_var,vs_out,vs_constref]) and
          not jvmimplicitpointertype(def);
      end;


    function tcpuparamanager.push_size(varspez: tvarspez; def: tdef; calloption: tproccalloption): longint;
      begin
        { all aggregate types are emulated using indirect pointer types }
        if def.typ in [arraydef,recorddef,setdef,stringdef] then
          result:=4
        else
          result:=inherited;
      end;


    function tcpuparamanager.get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;
      var
        paraloc : pcgparalocation;
        retcgsize  : tcgsize;
      begin
        result.init;
        result.alignment:=get_para_align(p.proccalloption);
        if not assigned(forcetempdef) then
          result.def:=p.returndef
        else
          begin
            result.def:=forcetempdef;
            result.temporary:=true;
          end;
        result.def:=get_para_push_size(result.def);
        { void has no location }
        if is_void(result.def) then
          begin
            paraloc:=result.add_location;
            result.size:=OS_NO;
            result.intsize:=0;
            paraloc^.size:=OS_NO;
            paraloc^.def:=voidtype;
            paraloc^.loc:=LOC_VOID;
            exit;
          end;
        { Constructors return self instead of a boolean }
        if (p.proctypeoption=potype_constructor) then
          begin
            retcgsize:=OS_INT;
            result.intsize:=sizeof(pint);
          end
        else if jvmimplicitpointertype(result.def) then
          begin
            retcgsize:=OS_ADDR;
            result.def:=cpointerdef.getreusable_no_free(result.def);
          end
        else
          begin
            retcgsize:=def_cgsize(result.def);
            result.intsize:=result.def.size;
          end;
        result.size:=retcgsize;

        paraloc:=result.add_location;
        { all values are returned on the evaluation stack }
        paraloc^.loc:=LOC_REFERENCE;
        paraloc^.reference.index:=NR_EVAL_STACK_BASE;
        paraloc^.reference.offset:=0;
        paraloc^.size:=result.size;
        paraloc^.def:=result.def;
      end;

    function tcpuparamanager.param_use_paraloc(const cgpara: tcgpara): boolean;
      begin
        { all parameters are copied by the VM to local variable locations }
        result:=true;
      end;

    function tcpuparamanager.ret_in_param(def:tdef;pd:tabstractprocdef):boolean;
      begin
        { not as efficient as returning in param for jvmimplicitpointertypes,
          but in the latter case the routines are harder to use from Java
          (especially for arrays), because the caller then manually has to
          allocate the instance/array of the right size }
        Result:=false;
      end;

    function tcpuparamanager.is_stack_paraloc(paraloc: pcgparalocation): boolean;
      begin
        { all parameters are passed on the evaluation stack }
        result:=true;
      end;


    function tcpuparamanager.create_varargs_paraloc_info(p : tabstractprocdef; side: tcallercallee; varargspara:tvarargsparalist):longint;
      var
        parasize : longint;
      begin
        parasize:=0;
        { calculate the registers for the normal parameters }
        create_paraloc_info_intern(p,side,p.paras,parasize);
        { append the varargs }
        if assigned(varargspara) then
          begin
            if side=callerside then
              create_paraloc_info_intern(p,side,varargspara,parasize)
            else
              internalerror(2019021924);
          end;
        create_funcretloc_info(p,side);
        result:=parasize;
      end;


    procedure tcpuparamanager.create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee;paras:tparalist;
                                                           var parasize:longint);
      var
        paraloc      : pcgparalocation;
        i            : integer;
        hp           : tparavarsym;
        paracgsize   : tcgsize;
        paraofs      : longint;
        paradef      : tdef;
      begin
        paraofs:=0;
        for i:=0 to paras.count-1 do
          begin
            hp:=tparavarsym(paras[i]);
            if push_copyout_param(hp.varspez,hp.vardef,p.proccalloption) then
              begin
                { passed via array reference (instead of creating a new array
                  type for every single parameter, use java_jlobject) }
                paracgsize:=OS_ADDR;
                paradef:=java_jlobject;
              end
            else if jvmimplicitpointertype(hp.vardef) then
              begin
                paracgsize:=OS_ADDR;
                paradef:=cpointerdef.getreusable_no_free(hp.vardef);
              end
            else
              begin
                paracgsize:=def_cgsize(hp.vardef);
                if paracgsize=OS_NO then
                  paracgsize:=OS_ADDR;
                paradef:=hp.vardef;
              end;
            paradef:=get_para_push_size(paradef);
            hp.paraloc[side].reset;
            hp.paraloc[side].size:=paracgsize;
            hp.paraloc[side].def:=paradef;
            hp.paraloc[side].alignment:=std_param_align;
            hp.paraloc[side].intsize:=tcgsize2size[paracgsize];
            paraloc:=hp.paraloc[side].add_location;
            { All parameters are passed on the evaluation stack, pushed from
              left to right (including self, if applicable). At the callee side,
              they're available as local variables 0..n-1 (with 64 bit values
              taking up two slots) }
            paraloc^.loc:=LOC_REFERENCE;;
            paraloc^.reference.offset:=paraofs;
            paraloc^.size:=paracgsize;
            paraloc^.def:=paradef;
            case side of
              callerside:
                begin
                  paraloc^.loc:=LOC_REFERENCE;
                  { we use a fake loc_reference to indicate the stack location;
                    the offset (set above) will be used by ncal to order the
                    parameters so they will be pushed in the right order }
                  paraloc^.reference.index:=NR_EVAL_STACK_BASE;
                end;
              calleeside:
                begin
                  paraloc^.loc:=LOC_REFERENCE;
                  paraloc^.reference.index:=NR_STACK_POINTER_REG;
                end;
              else
                ;
            end;
            { 2 slots for 64 bit integers and floats, 1 slot for the rest }
            if not(is_64bit(paradef) or
                   ((paradef.typ=floatdef) and
                    (tfloatdef(paradef).floattype=s64real))) then
              inc(paraofs)
            else
              inc(paraofs,2);
          end;
        parasize:=paraofs;
      end;


    function tcpuparamanager.create_paraloc_info(p : tabstractprocdef; side: tcallercallee):longint;
      var
        parasize : longint;
      begin
        parasize:=0;
        create_paraloc_info_intern(p,side,p.paras,parasize);
        { Create Function result paraloc }
        create_funcretloc_info(p,side);
        { We need to return the size allocated on the stack }
        result:=parasize;
      end;


begin
   ParaManager:=tcpuparamanager.create;
end.
