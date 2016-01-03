{ %norun }
<<<<<<< HEAD
<<<<<<< HEAD
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos,aix,android }
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> graemeg/fixes_2_2
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> origin/fixes_2_2
{ %NEEDLIBRARY }

{$mode delphi}

{$ifdef darwin}
{$PIC+}
{$endif darwin}

{$ifdef CPUX86_64}
{$ifndef WINDOWS}
{$PIC+}
{$endif WINDOWS}
{$endif CPUX86_64}

library tw8730b;

{$ifndef windows}
  {$linklib tw8730a}
{$endif}


uses uw8730b;

exports
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(darwin)}
=======
{$if defined(darwin) or defined(win32) or defined(wince)}
>>>>>>> graemeg/fixes_2_2
=======
{$if defined(darwin) or defined(win32) or defined(wince)}
>>>>>>> origin/fixes_2_2
Lib2Func name '_Lib2Func';
{$else}
Lib2Func;
{$endif}

end.

//= END OF FILE ===============================================================
