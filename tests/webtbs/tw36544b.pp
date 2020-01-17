{ %target=win32,win64,wince }
{ %needlibrary }
{ %result=-1073741502 }
{ ToDo: check whether the exit code is the same for the following targets:  darwin,linux,freebsd,solaris,beos,aix,android,haiku }
{$mode objfpc}

uses
  sysutils;

{$ifndef windows}
{$linklib tw36544a}
{$endif}

procedure library_procedure; external {$ifdef windows}'tw36544a'{$endif};

begin
  library_procedure;
end.
