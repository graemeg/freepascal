{ %target=linux,openbsd,netbsd,freebsd }
{ %cpu=i386 }
{ %norun }
{$goto on }
label
  l;

begin
  asm
    movl l@GOT(%eax),%eax 
    l:
  end;
  asm
    movl .Ll@GOT(%eax),%eax 
    .Ll:
  end;
end.
