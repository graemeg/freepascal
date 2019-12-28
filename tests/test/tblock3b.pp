{ %FAIL }
{ %target=darwin,iphonesim}
{ %skipcpu=powerpc,powerpc64 }

program tblock3b;

{$mode objfpc}
{$modeswitch cblocks}

type
  {$calling stdcall}
  tblock = reference to procedure; cblock;

begin

end.
