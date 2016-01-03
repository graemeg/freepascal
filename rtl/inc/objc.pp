
unit objc;

{$ifdef darwin}
{$define targethandled}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(iphonesim) or defined(cpuarm) or defined(cpux86_64) or defined(cpupowerpc64) or defined(cpuaarch64)}
=======
{$if defined(iphonesim) or defined(cpuarm) or defined(cpux86_64) or defined(cpupowerpc64)}
>>>>>>> graemeg/cpstrnew
=======
{$if defined(iphonesim) or defined(cpuarm) or defined(cpux86_64) or defined(cpupowerpc64)}
>>>>>>> graemeg/cpstrnew
=======
{$if defined(iphonesim) or defined(cpuarm) or defined(cpux86_64) or defined(cpupowerpc64)}
>>>>>>> graemeg/cpstrnew
=======
{$if defined(iphonesim) or defined(cpuarm) or defined(cpux86_64) or defined(cpupowerpc64)}
>>>>>>> origin/cpstrnew
{$i objcnf.inc}
{$endif}

{$if defined(cpupowerpc32) or (defined(cpui386) and not defined(iphonesim))}
{$define targethandled}
{$i objc1.inc}
{$endif}
{$endif}


{$ifndef targethandled}
  {$error Target not yet supported for objc.pp unit}
{$endif}
