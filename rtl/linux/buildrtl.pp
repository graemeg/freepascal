{ This unit is only used to edit the rtl with lazarus }
unit buildrtl;

  interface

    uses
      { those units are directly build using make:
        system uuchar objpas macpas iso7185 cpall lineinfo lnfodwrf
      }
      macpas, iso7185,
      fpintres, // $(SYSINIT_UNITS) \
      si_prc, si_c, si_g, si_dll,
      unixtype, ctypes, baseunix, strings, extpas, syscall, unixutil,
      heaptrc,
      termio, unix, linux, initc, cmem,
{$ifdef CPUI386}
      mmx,
{$endif CPUI386}
      linuxvcs,
      sysutils, typinfo, math,
      charset, cpall, character, unixcp, getopts,
      errors, dl, dynlibs,
      types, sysconst, fpwidestring,
      cthreads, sortbase, classes, fgl, rtlconsts, dos, cwstring, fpcylix,
      softfpu, sfpux80, ufloatx80, sfpu128, ufloat128;

  implementation

end.
