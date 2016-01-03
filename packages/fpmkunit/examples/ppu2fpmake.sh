<<<<<<< HEAD
<<<<<<< HEAD
#!/usr/bin/env bash
=======
#!/bin/sh
>>>>>>> graemeg/fixes_2_2
=======
#!/bin/sh
>>>>>>> origin/fixes_2_2

ppudump=ppudump

for f in $*; do
  $ppudump -vim $f | awk "
BEGIN { first=1;
 rtlunits=\" baseunix buildrtl charset classes cmem convutils cpu crt cthreads ctypes cwstring dateutils dl dos dynlibs errors exeinfo fgl fmtbcd fpcylix getopts gpm heaptrc initc ipc keyboard lineinfo linux linuxvcs lnfodwrf macpas math matrix messages mmx mouse objects objpas ports printer rtlconsts serial sharemem si_c si_c21 si_c21g si_dll signals si_prc si_uc sockets stdconvs strings strutils syscall sysconst sysinitcyg sysinitgprof sysinitpas system sysutils terminfo termio types typinfo ucomplex unix unixtype unixutil variants varutils video windows winevent winsock winsock2 winsysut x86 \"
}
/^Source/ {
  if (first==1)
  {
    printf(\"T:=P.Targets.AddUnit('%s');\\n\",\$5);
    first=0;
    dep=1;
  }
  else
  {
    if (dep==1) {
      dep=0;
      printf(\"  with T.Dependencies do\\n    begin\\n\",\$5);
    }
    printf(\"      AddInclude('%s');\\n\",\$5);
  }
}
/^Uses unit/ {
  if (index(rtlunits,tolower(\$3))==0) {
    if (dep==1) {
      dep=0;
      printf(\"  with T.Dependencies do\\n    begin\\n\",\$5);
    }
    printf(\"      AddUnit('%s');\\n\",tolower(\$3));
  }
}
END { if (first==0 && dep==0) { printf(\"    end;\\n\"); } }
"
done
