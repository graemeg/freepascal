{ %cpu=i386,x86_64 }
{ %opt=-Sew -vw }
{$mode objfpc}
uses cpu;

{$asmmode att}
procedure test1; assembler;
var
  s: single;
asm
   vmovss  s, %xmm6
   vmovss  %xmm6, s
{$ifdef cpui386}
   vmovss  (%eax, %edx), %xmm7
   vmovss  %xmm7, (%eax, %edx)
{$endif}
{$ifdef cpux86_64}
   vmovss  (%rax, %rdx), %xmm7
   vmovss  %xmm7, (%rax, %rdx)
{$endif}
end;

{$asmmode intel}
procedure test2; assembler;
var
  s: single;
asm
  vmovss  [s], xmm6
  vmovss  xmm6, [s]
{$ifdef cpui386}
  vmovss  [eax+edx], xmm7
  vmovss  xmm7, [eax+edx]
{$endif}
{$ifdef cpux86_64}
  vmovss  [rax+rdx], xmm7
  vmovss  xmm7, [rax+rdx]
{$endif}
end;


type
   TVector4 = packed record
     X, Y, Z, W: Single;
   end;

 function _VectorDotProductAVX(Vector1, Vector2: TVector4): Single; assembler;
 asm
{$if defined(cpux86_64) and not(defined(win64))}
   VMOVLHPS XMM0,XMM0,XMM1
   VMOVLHPS XMM1,XMM2,XMM3
{$else defined(cpux86_64) and not(defined(win64))}
   VMOVUPS XMM0, [Vector1]
   VMOVUPS XMM1, [Vector2]
{$endif defined(cpux86_64) and not(defined(win64))}
   VDPPS XMM0, XMM0, XMM1, $71 { Only perform calculations on the X, Y and Z coordinates; only store result in the first element }
   VMOVSS Result, XMM0 { Store result - first element of XMM0 }
 end;

var
  v: tvector4;
  r: single;
begin
  v.x:=1;
  v.y:=1;
  v.z:=1;
  v.w:=1;
  if AVXSupport then
  begin
    r:=_vectordotproductavx(v,v);
    if r<>3 then
      halt(1);
    writeln('ok');
  end
  else
    writeln('No AVX support');
end.

