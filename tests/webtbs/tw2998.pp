{ %cpu=i386 }
{ %OPT=-OaVARMIN=16 -Cg- }

{ Source provided for Free Pascal Bug Report 2998 }
{ Submitted by "bartek" on  2004-03-02 }
{ e-mail: bbartek@gmx.net }

{$asmmode intel}
program SSE_test;
uses
  mmx;
type
        vector4 = array[0..3] of single;

<<<<<<< HEAD
<<<<<<< HEAD
{$codealign varmax=16}
=======
{$codealign varmax 16}
>>>>>>> graemeg/fixes_2_2
=======
{$codealign varmax 16}
>>>>>>> origin/fixes_2_2
var
        a,b,c :vector4;
begin
  if is_sse_cpu then
    asm
      movups xmm0, [a]
      addps xmm0,[b]
      movups [c], xmm0
    end;
end.
