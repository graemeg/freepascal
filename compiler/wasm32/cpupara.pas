{
    Copyright (c) 2019 by Dmtiry Boyarintsev

    Calling conventions for the WebAssembly

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
        function  keep_para_array_range(varspez: tvarspez; def: tdef; calloption: tproccalloption): boolean; override;
        function  push_addr_param(varspez:tvarspez;def : tdef;calloption : tproccalloption) : boolean;override;
        function  push_size(varspez: tvarspez; def: tdef; calloption: tproccalloption): longint;override;
        function  create_paraloc_info(p : TAbstractProcDef; side: tcallercallee):longint;override;
        function  create_varargs_paraloc_info(p : tabstractprocdef; side: tcallercallee; varargspara:tvarargsparalist):longint;override;
        function  get_funcretloc(p : tabstractprocdef; side: tcallercallee; forcetempdef: tdef): tcgpara;override;
        { true if the location in paraloc can be reused as localloc }
        function param_use_paraloc(const cgpara: tcgpara): boolean; override;
        function is_stack_paraloc(paraloc: pcgparalocation): boolean;override;
      private
        procedure create_paraloc_info_intern(p : tabstractprocdef; side: tcallercallee; paras: tparalist;
                                             var parasize:longint);
      end;

implementation

    uses
      cutils,verbose,systems,
      defutil,wasmdef,
      aasmcpu,
      hlcgobj;


    function tcpuparamanager.get_saved_registers_int(calloption: tproccalloption): tcpuregisterarray;
      const
        { dummy, not used for WebAssembly }
        saved_regs: {$ifndef VER3_0}tcpuregisterarray{$else}array [0..0] of tsuperregister{$endif} = (RS_NO);
      begin
        result:=saved_regs;
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
        result:=false;
        { var,out,constref always require address }
        if varspez in [vs_var,vs_out,vs_constref] then
          begin
            result:=true;
            exit;
          end;
        { Only vs_const, vs_value here }
        case def.typ of
          variantdef,
          formaldef :
            result:=true;
          recorddef :
            begin
              { Delphi stdcall passes records on the stack for call by value }
              result:=(varspez=vs_const) or (not (def.size in [1,2,4,8]));
            end;
          arraydef :
            begin
              result:=(tarraydef(def).highrange>=tarraydef(def).lowrange) or
                 is_open_array(def) or
                 is_array_of_const(def) or
                 is_array_constructor(def);
            end;
          objectdef :
            result:=is_object(def);
          stringdef :
            result:= (tstringdef(def).stringtype in [st_shortstring,st_longstring]);
          procvardef :
            result:=not(calloption in cdecl_pocalls) and not tprocvardef(def).is_addressonly;
          setdef :
            result:=not is_smallset(def);
          else
            ;
        end;

      end;


    function tcpuparamanager.push_size(varspez: tvarspez; def: tdef; calloption: tproccalloption): longint;
      begin
        { all aggregate types are emulated using indirect pointer types }
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
        //todo: wasm should have the similar
        {else if jvmimplicitpointertype(result.def) then
          begin
            retcgsize:=OS_ADDR;
            result.def:=cpointerdef.getreusable_no_free(result.def);
          end}
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
        { all parameters are copied to the linear stack, so that their address
          can be taken }
        result:=false;
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
                paradef:=ptruinttype;
              end
            else if push_addr_param(hp.varspez, hp.vardef,p.proccalloption) then
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
              they're available as local variables 0..n-1  }
            paraloc^.loc:=LOC_REFERENCE;
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
            inc(paraofs);
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


initialization
  ParaManager:=tcpuparamanager.create;
end.
