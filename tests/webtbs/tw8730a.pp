{ %norun }
<<<<<<< HEAD
<<<<<<< HEAD
{ %needlibrary }
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos,aix,android }
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> graemeg/fixes_2_2
=======
{ %target=win32,win64,wince,darwin,linux,freebsd,solaris,beos}
>>>>>>> origin/fixes_2_2

{$mode delphi}

{$ifdef darwin}
{$PIC+}
{$endif darwin}

{$ifdef CPUX86_64}
{$ifndef WINDOWS}
{$PIC+}
{$endif WINDOWS}
{$endif CPUX86_64}

library tw8730a;

uses uw8730a;

exports
_Lib1Func;

end.

//= END OF FILE ===============================================================
