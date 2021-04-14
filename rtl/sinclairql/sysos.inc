{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    This file implements all the base types and limits required
    for a minimal POSIX compliant subset required to port the compiler
    to a new OS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$i qdos.inc}
{$i sms.inc}

procedure Error2InOutRes(errno: longint);
begin
  { Convert QL error code to InOutRes error code. }
  case errno of
    ERR_NC : InOutRes := 1;    { Operation not complete }
    ERR_NJ : InOutRes := 1;    { Not a (valid) job. }
    ERR_OM : InOutRes := 203;  { Out of memory. }
    ERR_OR : InOutRes := 201;  { Out of range. }
    ERR_BO : InOutRes := 106;  { Buffer overflow. }
    ERR_NO : InOutRes := 103;  { Channel not open. }
    ERR_NF : InOutRes := 2;    { File or device not found. }
    ERR_FX : InOutRes := 5;    { File already exists. }
    ERR_IU : InOutRes := 5;    { File or device already in use. }
    ERR_EF : InOutRes := 100;  { End of file. }
    ERR_DF : InOutRes := 101;  { Drive full. }
    ERR_BN : InOutRes := 15;   { Bad device. }
    ERR_TE : InOutRes := 162;  { Transmission error. }
    ERR_FF : InOutRes := 152;  { Format failed. }
    ERR_BP : InOutRes := 218;  { Bad parameter. }
    ERR_FE : InOutRes := 5;    { File error. }
    ERR_EX : InOutRes := 106;  { Expression error. }
    ERR_OV : InOutRes := 215;  { Arithmetic overflow. }
    ERR_NI : InOutRes := 1;    { Not implemented. }
    ERR_RO : InOutRes := 150;  { Read only. }
    ERR_BL : InOutRes := 1;    { UNLIKELY! Bad line of Basic. }
  end;
end;


var
  stackpointer_on_entry: pointer; external name '__stackpointer_on_entry';

function StackTop: Pointer;
begin
  StackTop:=stackpointer_on_entry;
end;


var
  binstart: byte; external name '_stext';
  binend: byte; external name '_etext';

function SysBackTraceStr (Addr: CodePointer): ShortString;
begin
  if (addr<@binstart) or (addr>@binend) then
    SysBackTraceStr:='  Addr $'+hexstr(addr)
  else
    SysBackTraceStr:='  Offs $'+hexstr(pointer(addr-@binstart));
end;
