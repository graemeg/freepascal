{ %CPU=aarch64 }
{ %NORUN }

program tb0669;

Type
  TSysResult = Int64;
  TSysParam  = Int64;

procedure seterrno(err:longint);

begin
end;

function FpSysCall(sysnr:TSysParam):TSysResult;
assembler; nostackframe;
asm
  (* mov w8,w0
  svc #0
  tbz x0,#63,.Ldone
  str x30,[sp,#-16]!
  neg x0,x0 *)
  bl seterrno
  (* ldr x30,[sp],#16
  mov x0,#-1
.Ldone: *)
end;

begin

end.
