{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 Sven Barth
    member of the Free Pascal development team.

    m68k syscall headers for Linux 

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function FpSysCall(sysnr:TSysParam):TSysResult; [public,alias:'FPC_SYSCALL0'];
begin
  asm
    move.l sysnr, d0
    trap #0
    move.l d0, @Result
  end ['d0'];
  if (ptruint(result) > ptruint(-4096)) then
  begin
    seterrno(-result);
    result:=TSysResult(-1);
  end;
end;


function FpSysCall(sysnr,param1:TSysParam):TSysResult; [public,alias:'FPC_SYSCALL1'];
begin
  asm
    move.l sysnr, d0
    move.l param1, d1
    trap #0
    move.l d0, @Result
  end ['d0', 'd1'];
  if (ptruint(result) > ptruint(-4096)) then
  begin
    seterrno(-result);
    result:=TSysResult(-1);
  end;
end;


function FpSysCall(sysnr,param1,param2:TSysParam):TSysResult; [public,alias:'FPC_SYSCALL2'];
begin
  asm
    move.l sysnr, d0
    move.l param1, d1
    move.l param2, d2
    trap #0
    move.l d0, @Result
  end ['d0', 'd1', 'd2'];
  if (ptruint(result) > ptruint(-4096)) then
  begin
    seterrno(-result);
    result:=TSysResult(-1);
  end;
end;


function FpSysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; [public,alias:'FPC_SYSCALL3'];
begin
  asm
    move.l sysnr, d0
    move.l param1, d1
    move.l param2, d2
    move.l param3, d3
    trap #0
    move.l d0, @Result
  end ['d0', 'd1', 'd2', 'd3'];
  if (ptruint(result) > ptruint(-4096)) then
  begin
    seterrno(-result);
    result:=TSysResult(-1);
  end;
end;

function FpSysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; [public,alias:'FPC_SYSCALL4'];
begin
  asm
    move.l sysnr, d0
    move.l param1, d1
    move.l param2, d2
    move.l param3, d3
    move.l param4, d4
    trap #0
    move.l d0, @Result
  end ['d0', 'd1', 'd2', 'd3', 'd4'];
    if (ptruint(result) > ptruint(-4096)) then
  begin
    seterrno(-result);
    result:=TSysResult(-1);
  end;
end;


function FpSysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult; [public,alias:'FPC_SYSCALL5'];
begin
  asm
    move.l sysnr, d0
    move.l param1, d1
    move.l param2, d2
    move.l param3, d3
    move.l param4, d4
    move.l param5, d5
    trap #0
    move.l d0, @Result
  end ['d0', 'd1', 'd2', 'd3', 'd4', 'd5'];
  if (ptruint(result) > ptruint(-4096)) then
  begin
    seterrno(-result);
    result:=TSysResult(-1);
  end;
end;


function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):TSysResult; [public,alias:'FPC_SYSCALL6'];
begin
  asm
    move.l sysnr, d0
    move.l param1, d1
    move.l param2, d2
    move.l param3, d3
    move.l param4, d4
    move.l param5, d5
    move.l param6, a0
    trap #0
    move.l d0, @Result
  end ['d0', 'd1', 'd2', 'd3', 'd4', 'd5', 'a0'];
  if (ptruint(result) > ptruint(-4096)) then
  begin
    seterrno(-result);
    result:=TSysResult(-1);
  end;
end;

