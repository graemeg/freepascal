unit buildrtl;

  interface

    uses
{$ifdef cpum68k}
      si_prc,
{$endif}
      sysutils, dos,

      ctypes, strings,
      rtlconsts, sysconst, math, types,
      typinfo, sortbase, fgl, classes,
      charset, character, getopts,
      fpwidestring;

  implementation

end.
