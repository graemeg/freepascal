unit buildrtl;

  interface

    uses
      ndk, ndkutils, ddk,
      ctypes, strings,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      rtlconsts, sysconst, sysutils, math, types,
      typinfo, fgl, classes,
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      matrix,
      rtlconsts, sysconst, sysutils, math, types,
      strutils, dateutils, varutils, variants, typinfo, fgl, classes,
      convutils, stdconvs,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef cpui386}
      mmx, cpu,
{$endif}
{$ifdef cpux86_64}
      cpu,
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      charset, cpall, getopts;
=======
      charset, ucomplex, getopts,
      fmtbcd;
>>>>>>> graemeg/cpstrnew
=======
      charset, ucomplex, getopts,
      fmtbcd;
>>>>>>> graemeg/cpstrnew
=======
      charset, ucomplex, getopts,
      fmtbcd;
>>>>>>> graemeg/cpstrnew
=======
      charset, ucomplex, getopts,
      fmtbcd;
>>>>>>> origin/cpstrnew

  implementation

end.
