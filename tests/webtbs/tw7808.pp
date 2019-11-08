{ %cpu=i386 }
{ %target=go32v2,linux,freebsd,win32,haiku}
{ %opt=-Sew -vw }

{ do not warn about the lea esp,[esp+16] }
{$WARN 7105 OFF}

{$mode delphi}

procedure test(l: longint); stdcall;
begin
  if l<>longint($deadbeef) then
    halt(1);
end;

begin
  asm
{$if FPC_STACKALIGNMENT=16}
    lea esp,[esp-12]
{$endif FPC_STACKALIGNMENT=16}  
    push word $dead
    push word $beef
    call test        
{$if FPC_STACKALIGNMENT=16}
    lea esp,[esp-4]
{$endif FPC_STACKALIGNMENT=16}  
  end;
end.
