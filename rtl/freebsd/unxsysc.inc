{
   This file is part of the Free Pascal run time library.
   Copyright (c) 2003 Marco van de Voort
   member of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

(*
function clone(func:TCloneFunc;sp:pointer;flags:longint;args:pointer):longint;
{NOT IMPLEMENTED YET UNDER BSD}
begin // perhaps it is better to implement the hack from solaris then this msg
 HALT;
END;

  if (pointer(func)=nil) or (sp=nil) then
   begin
     Lfpseterrno(EsysEInval);
     exit(-1);
   end;
  asm
        { Insert the argument onto the new stack. }
        movl    sp,%ecx
        subl    $8,%ecx
        movl    args,%eax
        movl    %eax,4(%ecx)

        { Save the function pointer as the zeroth argument.
          It will be popped off in the child in the ebx frobbing below. }
        movl    func,%eax
        movl    %eax,0(%ecx)

        { Do the system call }
        pushl   %ebx
        pushl   %ebx
      //  movl    flags,%ebx
        movl    $251,%eax
        int     $0x80
        popl    %ebx
        popl    %ebx
        test    %eax,%eax
        jnz     .Lclone_end

        { We're in the new thread }
        subl    %ebp,%ebp       { terminate the stack frame }
        call    *%ebx
        { exit process }
        movl    %eax,%ebx
        movl    $1,%eax
        int     $0x80

.Lclone_end:
        movl    %eax,__RESULT
  end;
end;
*)

Function  fpfStatFS (Fd: cint; Info:pstatfs):cint;
begin
  fpfstatfs:=do_SysCall(SysCall_nr_fstatfs,fd,TSysParam(info))
end;

Function  fpStatFS  (Path:pchar; Info:pstatfs):cint;

begin
  fpstatfs:=do_SysCall(SysCall_nr_statfs,TSysParam(path),TSysParam(Info))
end;

Function  fpfsync (fd : cint) : cint;

begin
  fpfsync:=do_SysCall(syscall_nr_fsync, fd);
end;

Function  fpFlock (fd,mode : longint) : cint;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

begin
 fpFlock:=do_syscall(syscall_nr_flock,fd,mode);
end;

// needs oldfpccall;
Function intAssignPipe(var pipe_in,pipe_out:longint;var errn:cint):cint;  oldfpccall;
{
  Sets up a pair of file variables, which act as a pipe. The first one can
  be read from, the second one can be written to.
  If the operation was unsuccesful, linuxerror is set.
}
>>>>>>> graemeg/fixes_2_2

begin
 fpFlock:=do_syscall(syscall_nr_flock,fd,mode);
end;

function MUnMap (P : Pointer; Size : size_t) : cint;
begin
  MUnMap:=do_syscall(syscall_nr_munmap,TSysParam(P),Size);
end;

