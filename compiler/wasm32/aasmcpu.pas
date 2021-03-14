{
    Copyright (c) 2019 by Free Pascal and Lazarus foundation

    Contains the assembler object for the WebAssembly

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
unit aasmcpu;

{$i fpcdefs.inc}

interface

uses
  cclasses,
  globtype,globals,verbose,
  aasmbase,aasmtai,aasmdata,aasmsym,
  cgbase,cgutils,cpubase,cpuinfo,
  widestr;

    { fake, there are no "mov reg,reg" instructions here }
    const
      { "mov reg,reg" source operand number }
      O_MOV_SOURCE = 0;
      { "mov reg,reg" source operand number }
      O_MOV_DEST = 0;

    type

      { taicpu }

      taicpu = class(tai_cpu_abstract_sym)
         constructor op_none(op : tasmop);

         constructor op_reg(op : tasmop;_op1 : tregister);
         constructor op_const(op : tasmop;_op1 : aint);
         constructor op_ref(op : tasmop;const _op1 : treference);
         constructor op_sym(op : tasmop;_op1 : tasmsymbol);

         constructor op_sym_const(op : tasmop;_op1 : tasmsymbol;_op2 : aint);

         constructor op_single(op : tasmop;_op1 : single);
         constructor op_double(op : tasmop;_op1 : double);

         constructor op_functype(op : tasmop; _op1: TWasmFuncType);

         procedure loadfunctype(opidx:longint;ft:TWasmFuncType);
         procedure loadsingle(opidx:longint;f:single);
         procedure loaddouble(opidx:longint;d:double);

         { register allocation }
         function is_same_reg_move(regtype: Tregistertype):boolean; override;

         { register spilling code }
         function spilling_get_operation_type(opnr: longint): topertype;override;
      end;

      tai_align = class(tai_align_abstract)
        { nothing to add }
      end;

      TImpExpType= (
        ie_Func,   // functions
        ie_Table,  // tables (arrays of methods)
        ie_Memory, // memory reference
        ie_Global  // global variables
      );

      // the actual use is defined by the assembly section used

      { timpexp_ai }

      tai_impexp = class(tai)
        extname : ansistring; // external name
        intname : ansistring; // internal name
        extmodule : ansistring; // external unit name
        symstype: TImpExpType;
        constructor create(const aextname, aintname: ansistring; asymtype: timpexptype); overload;
        constructor create(const aextmodule, aextname, aintname: ansistring; asymtype: timpexptype); overload;
      end;

      // local variable declaration

      { tai_local }

      tai_local = class(tai)
        bastyp: TWasmBasicType;
        name : string;
        first: boolean;
        last: boolean;
        constructor create(abasictype: TWasmBasicType; const aname: string = '');
      end;

      { tai_functype }

      tai_functype = class(tai)
        funcname: string;
        functype: TWasmFuncType;
        constructor create(const afuncname: string; afunctype: TWasmFuncType);
        destructor destroy;override;
      end;

    procedure InitAsm;
    procedure DoneAsm;

    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
    function spilling_create_store(r:tregister; const ref:treference):Taicpu;

implementation

    { tai_functype }

    constructor tai_functype.create(const afuncname: string; afunctype: TWasmFuncType);
      begin
        inherited Create;
        typ:=ait_functype;
        funcname:=afuncname;
        functype:=afunctype;
      end;


    destructor tai_functype.destroy;
      begin
        functype.free;
        inherited;
      end;

    { tai_local }

    constructor tai_local.create(abasictype: TWasmBasicType; const aname: string);
      begin
        inherited Create;
        bastyp := abasictype;
        typ := ait_local;
        name := aname;
      end;

    { timpexp_ai }

      constructor tai_impexp.create(const aextname, aintname: ansistring;
          asymtype: timpexptype);
        begin
          create('', aextname, aintname, asymtype);;
        end;

      constructor tai_impexp.create(const aextmodule, aextname, aintname: ansistring; asymtype: timpexptype);
        begin
          inherited create;
          typ := ait_importexport;
          extmodule := aextmodule;
          extname := aextname;
          intname := aintname;
          symstype:= asymtype;
        end;

{*****************************************************************************
                                 taicpu Constructors
*****************************************************************************}

    constructor taicpu.op_none(op : tasmop);
      begin
        inherited create(op);
      end;


    constructor taicpu.op_reg(op : tasmop;_op1 : tregister);
      begin
        inherited create(op);
        ops:=1;
{$ifdef EXTDEBUG}
        if getregtype(_op1)=R_INVALIDREGISTER then
          InternalError(2021011901);
{$endif EXTDEBUG}
        loadreg(0,_op1);
      end;


    constructor taicpu.op_ref(op : tasmop;const _op1 : treference);
      begin
        inherited create(op);
        ops:=1;
        loadref(0,_op1);
        if op in [a_local_get,a_local_set,a_local_tee] then
          begin
            if (_op1.base<>NR_LOCAL_STACK_POINTER_REG) or (_op1.index<>NR_NO) then
              internalerror(2021010201);
          end
        else
          begin
            if (_op1.base<>NR_NO) or (_op1.index<>NR_NO) then
              internalerror(2021010202);
          end;
      end;


    constructor taicpu.op_const(op : tasmop;_op1 : aint);
      begin
        inherited create(op);
        ops:=1;
        loadconst(0,_op1);
      end;


    constructor taicpu.op_sym(op : tasmop;_op1 : tasmsymbol);
      begin
        inherited create(op);
        ops:=1;
        loadsymbol(0,_op1,0);
      end;


    constructor taicpu.op_sym_const(op: tasmop; _op1: tasmsymbol; _op2: aint);
      begin
        inherited create(op);
        ops:=2;
        loadsymbol(0,_op1,0);
        loadconst(1,_op2);
      end;


    constructor taicpu.op_single(op: tasmop; _op1: single);
      begin
        inherited create(op);
        ops:=1;
        loadsingle(0,_op1);
      end;


    constructor taicpu.op_double(op: tasmop; _op1: double);
      begin
        inherited create(op);
        ops:=1;
        loaddouble(0,_op1);
      end;

    constructor taicpu.op_functype(op: tasmop; _op1: TWasmFuncType);
      begin
       inherited create(op);
       ops:=1;
       loadfunctype(0,_op1);
      end;

    procedure taicpu.loadfunctype(opidx: longint; ft: TWasmFuncType);
      begin
       allocate_oper(opidx+1);
       with oper[opidx]^ do
        begin
          if typ<>top_functype then
            clearop(opidx);
          functype:=ft;
          typ:=top_functype;
        end;
      end;


    procedure taicpu.loadsingle(opidx:longint;f:single);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_single then
             clearop(opidx);
           sval:=f;
           typ:=top_single;
         end;
      end;


    procedure taicpu.loaddouble(opidx: longint; d: double);
      begin
        allocate_oper(opidx+1);
        with oper[opidx]^ do
         begin
           if typ<>top_double then
             clearop(opidx);
           dval:=d;
           typ:=top_double;
         end;
      end;


    function taicpu.is_same_reg_move(regtype: Tregistertype):boolean;
      begin
        result:=false;
      end;


    function taicpu.spilling_get_operation_type(opnr: longint): topertype;
      begin
        if opcode in AsmOp_Store then
          result:=operand_write
        else
          result:=operand_read;
      end;


    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
      begin
       internalerror(2010122614);
       result:=nil;
      end;


    function spilling_create_store(r:tregister; const ref:treference):Taicpu;
      begin
       internalerror(2010122615);
       result:=nil;
      end;


    procedure InitAsm;
      begin
      end;


    procedure DoneAsm;
      begin
      end;

initialization
  cai_cpu:=taicpu;
  cai_align:=tai_align;
  casmdata:=TAsmData;
end.
