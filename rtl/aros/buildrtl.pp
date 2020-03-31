unit buildrtl;

  interface

    uses
      si_prc,
      athreads, dos, sysutils,

      ctypes, strings,
      rtlconsts, sysconst, math, types,

      exeinfo,
{$ifdef cpui386}
      lineinfo,
{$endif}

      typinfo, sortbase, fgl, classes,
      charset, character, getopts,
      fpwidestring, fpintres;

  implementation

end.
