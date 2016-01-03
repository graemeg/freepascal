
{ This include file defines FPC_HAS_UNDERSCORE_PREFIX
  for targets for which Cprefix='_' }


{ This file was created lookinf into
  systems/i_XXX.pas files in trunk 2010-12-16 PM }

{ From i_atari.pas }
{$ifdef atari}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{ From i_bsd.pas, all i386 but freebsd seem to have underscore prefixes }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ This is not true anyore at least for OpenBSD 4.9 PM 2011-10-10 }
{ Also false for new NetBSD targets PM 2012-02-12 }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef bsd}
  {$ifdef cpui386}
    {$ifndef freebsd}
      {$define FPC_HAS_UNDERSCORE_PREFIX}
    {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    {$ifdef openbsd}
      {$undef FPC_HAS_UNDERSCORE_PREFIX}
    {$endif}
    {$ifdef netbsd}
      {$undef FPC_HAS_UNDERSCORE_PREFIX}
    {$endif}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$endif}
  {$ifdef darwin}
    {$define FPC_HAS_UNDERSCORE_PREFIX}
  {$endif}
{$endif}

{$ifdef emx}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef go32v2}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef nativent}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef os2}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef palmos}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef symbian}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef watcom}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef wdosx}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef win32}
  {$define FPC_HAS_UNDERSCORE_PREFIX}
{$endif}

{$ifdef wince}
  {$ifdef cpui386}
    {$define FPC_HAS_UNDERSCORE_PREFIX}
  {$endif}
{$endif}
