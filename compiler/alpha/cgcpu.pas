{
    Copyright (c) 1998-2000 by Florian Klaempfl

    This unit implements the code generator for the Alpha

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
{
  This unit implements the code generator for the Alpha.
}
unit cgcpu;

{$i fpcdefs.inc}

interface

uses
   cgbase,cgobj,aasmbase,aasmtai,aasmdata,aasmcpu,cginfo,cpubase,cpuinfo;

type
pcgalpha = ^tcgalpha;
tcgalpha = class(tcg)
  procedure a_call_name(list : TAsmList;const s : string);override;
  procedure a_load_const_reg(list : TAsmList;size : tcgsize;a : aword;register : tregister);override;
  procedure a_load_reg_ref(list : TAsmList;size : tcgsize;register : tregister;const ref : treference);override;
  procedure a_load_ref_reg(list : TAsmList;size : tcgsize;const ref : treference;register : tregister);override;
  procedure a_load_reg_reg(list : TAsmList;fromsize, tosize : tcgsize;reg1,reg2 : tregister);override;
  procedure a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : aword;
    reg : tregister;  l : tasmlabel);override;
  procedure a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel);
  procedure a_cmp_reg_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg : tregister;l : tasmlabel);
  procedure a_cmp_ref_const_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : aword;
    reg : tregister; l : tasmlabel);
  procedure a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);override;
  procedure g_stackframe_entry(list : TAsmList;localsize : longint);override;
  procedure g_maybe_loadself(list : TAsmList);override;
  procedure g_restore_frame_pointer(list : TAsmList);override;
end;

procedure create_codegen;

implementation

uses
   globtype,globals;

procedure tcgalpha.g_stackframe_entry(list : TAsmList;localsize : longint);

begin
   list.concat(taicpu.op_reg_ref(A_LDGP,Global_pointer,new_reference(R_27,0)));
   list.concat(taicpu.op_reg_ref(A_LDA,stack_pointer_reg,new_reference(stack_pointer_reg,-LocalSize)));
   If LocalSize<>0 then
     list.concat(tai_frame.create(Global_pointer,LocalSize,R_27,0));
   { Always generate a frame pointer. }
   list.concat(taicpu.op_reg_reg_reg(A_BIS,stack_pointer_reg,stack_pointer_reg,frame_pointer_reg));
end;

procedure g_exitcode(list : TAsmList;parasize : longint; nostackframe,inlined : boolean);

begin
   { Restore stack pointer from frame pointer }
   list.Concat (taicpu.op_reg_reg_reg(A_BIS,frame_pointer_reg,frame_pointer_reg,stack_pointer_reg));
   { Restore previous stack position}
   list.Concat (taicpu.op_reg_const_reg(A_ADDQ,stack_pointer_reg,Parasize,stack_pointer_reg));
   { return... }
   list.Concat(taicpu.op_reg_ref_const(A_RET,stack_pointer_reg,new_reference(Return_pointer,0),1));
    { end directive
    Concat (paiend,init(''));
    }
end;

procedure tcgalpha.a_call_name(list : TAsmList;const s : string);

  begin
     { list^.concat(taicpu,op_sym(A_CALL,S_NO,newasmsymbol(s,AB_EXTERNAL,AT_FUNCTION)))); }
     {!!!!!!!!!1 offset is ignored }
     abstract;
  end;

procedure tcgalpha.a_load_const_reg(list : TAsmList;size : tcgsize;a : aword;register : tregister);

begin
end;


procedure tcgalpha.a_load_reg_ref(list : TAsmList;size : tcgsize;register : tregister;const ref : treference);

begin
end;


procedure tcgalpha.a_load_ref_reg(list : TAsmList;size : tcgsize;const ref : treference;register : tregister);

begin
end;


procedure tcgalpha.a_load_reg_reg(list : TAsmList;fromsize, tosize : tcgsize;reg1,reg2 : tregister);

begin
end;


procedure tcgalpha.a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : aword;reg : tregister;
  l : tasmlabel);

begin
end;


procedure tcgalpha.a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel);

begin
end;


procedure tcgalpha.a_cmp_reg_ref_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg : tregister;l : tasmlabel);

begin
end;


procedure tcgalpha.a_cmp_ref_const_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : aword;
  reg : tregister; l : tasmlabel);

begin
end;


procedure tcgalpha.a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);

begin
end;


procedure tcgalpha.g_maybe_loadself(list : TAsmList);

begin
end;


procedure tcgalpha.g_restore_frame_pointer(list : TAsmList);

begin
end;


procedure create_codegen;
  begin
    cg:=tcgalpha.create;
  end;

end.
