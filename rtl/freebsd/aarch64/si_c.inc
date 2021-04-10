
Type
    TCleanup = procedure; cdecl;

var
  environ : ppchar; cvar; public  name '__environ';
  progname: pchar = #0#0; cvar; public name '__progname';
  dynamic : pchar;  external name '_DYNAMIC'; // #pragma weak

procedure atexit(prc:TCleanup); cdecl external name 'atexit';
procedure cleanup(prc:TCleanup); cdecl external name 'cleanup';
procedure init_tls; cdecl; external name 'init_tls';
procedure fini; cdecl; external name '_fini';
procedure init; cdecl; external name '_init';
procedure libc_exit(exitcode:longint);cdecl; external name 'exit';
function  main(nrarg:longint;pp:ppchar;env:ppchar):longint; cdecl; external name 'main';

{$ifdef gcrt}
 procedure cmcleanup; cdecl; external name '_mcleanup';
 procedure monstratup(p,p2:pointer); cdecl; external name 'monstartup';

var
 eprol:longint; external name 'eprol';
 etext:longint; external name 'etext';
{$endif}

procedure start(ap:ppchar;cleanup:TCleanup);

var argc: longint;
    argv: ppchar;
    env : ppchar;
    s   : pchar;
begin
  argc:=plongint(ap)^;
  argv:=ppchar(ap[1]);
  env:=	ppchar(ap[2+argc]);
  environ:=env;
  if (argc>0) and (argv[0]<>#0) Then
   begin
     progname:=argv[0];
     s:=progname;
     while s^<>#0 do
        begin
          if s^='/' then
            progname:=@s[1];
          inc(s);
	end;
    end;
  if assigned(pchar(@dynamic)) then // I suspect this is a trick to find
				    // out runtime if we are shared
				    // linking, so the same code can be used
				    // for static and shared linking
    atexit(cleanup)
  else
    init_tls;
  {$ifdef GCRT}
    atexit(@_mcleanup);
  {$endif}
  atexit(@fini);
  {$ifdef GCRT}
    monstartup(@eprol,@etext);
   asm
    eprol:
   end;
  {$endif}
  init;
  libc_exit(main(argc,argv,env)); // doesn't return
 asm
     { We need this stuff to make gdb behave itself, otherwise
      gdb will chokes with SIGILL when trying to debug apps.
    }
    .section ".note.ABI-tag", "a"
    .align 4
    .long 8
    .long 4
    .long  1
    .asciz "FreeBSD"
    .align 4
    .long	900044
    .align 4
    .section	.note.GNU-stack,"",@progbits
  end;
end;



begin
end.

