
{$define CRT_IRELOC_RELA}

procedure init_tls; cdecl; external name 'init_tls';
function  cmain(nrarg:longint;pp:ppchar;env:ppchar):longint; cdecl; external name 'main';

{$ifdef gcrt}
 procedure cmcleanup; cdecl; external name '_mcleanup';
 procedure monstratup(p,p2:pointer); cdecl; external name 'monstartup';

var 
  eprol:longint; external name 'eprol';
  etext:longint; external name 'etext';
{$endif}

{$i ignore_init.inc}
procedure start(ap:ppchar;cleanup:TCleanupProc);

var argc: longint;
    argv: ppchar;
    env : ppchar;
    s   : pchar;
begin
  argc:=plongint(ap)^; 
  argv:=ppchar(ap[1]);
  env:=	ppchar(ap[2+argc]);
  environ:=env;
  handle_argv(argc,argv,env);
  if assigned(pchar(@_dynamic)) then // I suspect this is a trick to find
				    // out runtime if we are shared
				    // linking, so the same code can be used
				    // for static and shared linking
    atexit(cleanup)
  else
    begin
       process_irelocs();
       init_tls;
    end;

  {$ifdef GCRT}
    atexit(@cmcleanup);
  {$endif}
  atexit(@_fini);
  {$ifdef GCRT}
    monstartup(@eprol,@etext);
   asm
    eprol:
   end;
  {$endif} 
  handle_static_init(argc, argv, env);
  libc_exit(cmain(argc,argv,env)); // doesn't return
end;

begin
end.

