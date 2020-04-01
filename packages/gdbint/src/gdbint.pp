{
    Copyright (c) 1998 by Peter Vreman

    Lowlevel GDB interface which communicates directly with libgdb

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit GdbInt;

{$mode objfpc}
{$smartlink off}

{$define NotImplemented}

{$define COMPILING_GDBINT_UNIT}
{$ifdef USE_GDBLIBINC}
  {$i gdblib.inc}
{$else not USE_GDBLIBINC}
  {$i gdbver.inc}
{$endif not USE_GDBLIBINC}

{ Possible optional conditionals:
  GDB_DISABLE_INTL              To explicitly not use libintl

  GDB_DISABLE_PYTHON            To explicitly not use libpython,
  if gdb was configured using --without-python

  GDB_CORE_ADDR_FORCE_64BITS    To force 64 bits for CORE_ADDR

  Verbose                       To test gdbint

  DebugCommand                  To debug Command method
}

interface



{ Is create_breakpoint_hook deprecated? }
{ Seem not so for 6.1 }
{$define GDB_HAS_DEPRECATED_CBPH}


{
  Excatly one
  GDB_VXYZ macro
  where XYZ are three numbers
  needs to defined
  either inside gdblib.inc or gdbver.inc
  This corresponds to version
  X.YZ.patch_level
}

{$undef GDB_VERSION_RECOGNIZED}

{ GDB 7.12 is the last version
  that use a plain C compiler
  Later version will not work
  for gdbint unit unless we get a
  working g++ mangler into Free Pascal compiler }

{ 7.12.x }
{$ifdef GDB_V712}
  {$info using gdb 7.12.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_712}
  {$define GDB_NO_INSTREAM_VAR}
  {$define GDB_CURRENT_UIOUT_MACRO}
{$endif}

{$ifdef GDB_VER_GE_712}
  {$define GDB_VER_GE_711}
  {$define GDB_HAS_SAVED_COMMAND_LINE_BUT_NO_SIZE}
  {$define GDB_NEW_UI}
  {$define GDB_INTERP_LOOKUP_HAS_UI}
{$endif}

{ 7.11.x }
{$ifdef GDB_V711}
  {$info using gdb 7.11.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_711}
{$endif}

{$ifdef GDB_VER_GE_711}
  {$define GDB_VER_GE_710}
{$endif}

{ 7.10.x }
{$ifdef GDB_V710}
  {$info using gdb 7.10.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_710}
{$endif}

{$ifdef GDB_VER_GE_710}
  {$define GDB_VER_GE_709}
{$endif}

{ 7.9.x }
{$ifdef GDB_V709}
  {$info using gdb 7.9.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_709}
{$endif}

{$ifdef GDB_VER_GE_709}
  {$define GDB_VER_GE_708}
  {$define SYMTAB_HAS_COMPUNIT_SYMTAB}
{$endif}

{ 7.8.x }
{$ifdef GDB_V708}
  {$info using gdb 7.8.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_708}
{$endif}

{$ifdef GDB_VER_GE_708}
  {$define USE_CATCH_EXCEPTIONS}
  {$define USE_LOCAL_SET_GDB_DATA_DIRECTORY}
  {$define GDB_VER_GE_707}
{$endif}

{ 7.7.x }
{$ifdef GDB_V707}
  {$info using gdb 7.7.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_707}
{$endif}

{$ifdef GDB_VER_GE_707}
  {$define GDB_VER_GE_706}
{$endif}

{ 7.6.x }
{$ifdef GDB_V706}
  {$info using gdb 7.6.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_706}
{$endif}

{$ifdef GDB_VER_GE_706}
  {$define GDB_UI_FILE_HAS_FSEEK}
  {$define GDB_VER_GE_705}
{$endif}

{ 7.5.x }
{$ifdef GDB_V705}
  {$info using gdb 7.5.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_705}
{$endif}

{$ifdef GDB_VER_GE_705}
  {$define GDB_VER_GE_704}
  {$define GDB_BP_LOCATION_HAS_COND_BYTECODE}
  {$define GDB_BP_LOCATION_HAS_RELATED_ADDRESS}
  {$define GDB_BP_HAS_ENABLE_COUNT}
{$endif}

{ 7.4.x }
{$ifdef GDB_V704}
  {$info using gdb 7.4.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_VER_GE_704}
{$endif}

{$ifdef GDB_VER_GE_704}
  {$define GDB_V7}
  {$define GDB_BP_LOCATION_HAS_GDBARCH}
  {$define GDB_HAS_PROGRAM_SPACE}
  {$define GDB_NO_UIOUT}
  {$define GDB_NEEDS_INTERPRETER_SETUP}
  {$define GDB_NEEDS_SET_INSTREAM}
  {$define GDB_NOTIFY_BREAKPOINT_ARG_IS_BREAKPOINT_PTR}
  {$define GDB_USES_BP_OPS}
  {$define GDB_BP_TI_HAS_LENGTH}
  {$define GDB_BP_LOCATION_HAS_REFCOUNT}
  {$define GDB_BP_LOCATION_HAS_OPS}
  {$define GDB_UI_FILE_HAS_WRITE_ASYNC}
  {$ifdef win32}
      {$define GDB_USES_LIBADVAPI32}
  {$endif win32}
  {$define GDB_HAS_SAVED_COMMAND_LINE_SIZE}
{$endif def GDB_VER_GE_704}

{ 7.3.x }
{$ifdef GDB_V703}
  {$info using gdb 7.3.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V7}
  {$define GDB_BP_LOCATION_HAS_GDBARCH}
  {$define GDB_HAS_PROGRAM_SPACE}
  {$define GDB_BP_TI_HAS_LENGTH}
  {$define GDB_BP_LOCATION_HAS_REFCOUNT}
  {$ifdef GDB_CVS}
    {$define GDB_NO_UIOUT}
    {$define GDB_NEEDS_INTERPRETER_SETUP}
    {$define GDB_NEEDS_SET_INSTREAM}
    {$define GDB_NOTIFY_BREAKPOINT_ARG_IS_BREAKPOINT_PTR}
    {$define GDB_USES_BP_OPS}
    {$define GDB_BP_LOCATION_HAS_OPS}
    {$define GDB_UI_FILE_HAS_WRITE_ASYNC}
  {$endif GDB_CVS}
  {$define GDB_VERSION_RECOGNIZED}
{$endif def GDB_V703}

{ 7.2.x }
{$ifdef GDB_V702}
  {$info using gdb 7.2.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V7}
  {$define GDB_BP_LOCATION_HAS_GDBARCH}
  {$define GDB_HAS_PROGRAM_SPACE}
{$endif def GDB_V702}

{ 7.1.x }
{$ifdef GDB_V701}
  {$info using gdb 7.1.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V7}
  {$define GDB_BP_LOCATION_HAS_GDBARCH}
  {$define GDB_HAS_PROGRAM_SPACE}
{$endif def GDB_V701}



{ 7.0.x }
{$ifdef GDB_V700}
  {$info using gdb 7.0.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V7}
  {$ifdef GDB_CVS}
    {$define GDB_BP_LOCATION_HAS_GDBARCH}
    {$define GDB_HAS_PROGRAM_SPACE}
  {$endif GDB_CVS}
{$endif def GDB_V700}

{$ifdef GDB_V7}
  {$define GDB_V6}
  {$define GDB_HAS_DB_COMMANDS}
  {$define GDB_USES_BP_LOCATION}
  {$define GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
  {$define GDB_BP_LOCATION_HAS_GDBARCH}
  {$define GDB_NEEDS_NO_ERROR_INIT}
  {$define GDB_USES_EXPAT_LIB}
  {$define GDB_USES_LIBDECNUMBER}
  {$define GDB_USES_LIBINTL}
  {$ifndef GDB_DISABLE_PYTHON}
    {$define GDB_USES_LIBPYTHON}
  {$endif}
  {$define GDB_HAS_DEBUG_FILE_DIRECTORY}
  {$define GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
  {$define GDB_TARGET_CLOSE_HAS_PTARGET_ARG}
  {$define GDB_HAS_BP_NONE}
  {$define GDB_USE_XSTRVPRINTF}
  {$define GDB_ANNOTATE_FRAME_BEGIN_HAS_GDBARCH_FIELD}
{$endif def GDB_V7}


{ 6.8.x }
{$ifdef GDB_V608}
  {$info using gdb 6.8.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
  {$define GDB_USES_BP_LOCATION}
  {$define GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
  {$define GDB_NEEDS_NO_ERROR_INIT}
  {$define GDB_USES_EXPAT_LIB}
  {$define GDB_HAS_DEBUG_FILE_DIRECTORY}
  {$define GDB_USES_LIBDECNUMBER}
  // {$define GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
  {$define GDB_HAS_BP_NONE}
{$endif def GDB_V608}

{ 6.7.x }
{$ifdef GDB_V607}
  {$info using gdb 6.7.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
  {$define GDB_USES_BP_LOCATION}
  {$define GDB_NEEDS_NO_ERROR_INIT}
  {$define GDB_USES_EXPAT_LIB}
  {$define GDB_HAS_DEBUG_FILE_DIRECTORY}
{$endif def GDB_V607}

{ 6.6.x }
{$ifdef GDB_V606}
  {$info using gdb 6.6.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
  {$define GDB_USES_BP_LOCATION}
  {$define GDB_NEEDS_NO_ERROR_INIT}
  {$define GDB_USES_EXPAT_LIB}
  {Official 6.6 release doesn't have GDB_HAS_DEBUG_FILE_DIRECTORY}
{$endif def GDB_V606}

{ 6.5.x }
{$ifdef GDB_V605}
  {$info using gdb 6.5.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
  {$define GDB_NEEDS_NO_ERROR_INIT}
{$endif def GDB_V605}

{ 6.4.x }
{$ifdef GDB_V604}
  {$info using gdb 6.4.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
  {$define GDB_NEEDS_NO_ERROR_INIT}
{$endif def GDB_V604}

{ 6.3.x }
{$ifdef GDB_V603}
  {$info using gdb 6.3.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
{$endif def GDB_V603}

{ 6.2.x }
{$ifdef GDB_V602}
  {$info using gdb 6.2.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
{$endif def GDB_V602}

{ 6.1.x }
{$ifdef GDB_V601}
  {$info using gdb 6.1.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
  {$undef GDB_HAS_DEPRECATED_CBPH}
{$endif def GDB_V601}

{ 6.0.x }
{$ifdef GDB_V600}
  {$info using gdb 6.0.x}
  {$define GDB_VERSION_RECOGNIZED}
  {$define GDB_V6}
{$endif def GDB_V600}

{$ifdef GDB_V6}
  {$define GDB_HAS_SYSROOT}
  {$define GDB_HAS_DB_COMMANDS}
  {$define GDB_SYMTAB_HAS_MACROS}
  {$define GDB_INIT_HAS_ARGV0}
{$endif GDB_V6}

{$ifndef GDB_VERSION_RECOGNIZED}
  {$warning no recognized GDB_VXYZ conditional found, linking might fail. }
{$endif}


{$ifdef GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
  {$define DO_NOT_USE_CBPH}
{$endif}

{ GDB has a simulator for powerpc CPU
  it is integrated into GDB by default }
{$ifdef cpupowerpc}
  {$define GDB_HAS_SIM}
{$endif cpupowerpc}

{$ifdef Solaris}
  {$ifdef Sparc}
    { Sparc/i386 solaris gdb also supports 64bit mode, thus
      CORE_ADDR is 8-byte long }
    {$define GDB_CORE_ADDR_FORCE_64BITS}
  {$endif Sparc}
  {$ifdef i386}
    {$define GDB_CORE_ADDR_FORCE_64BITS}
  {$endif i386}
{$endif Solaris}

{$ifdef go32v2}
{$ifdef NotImplemented}
  {$undef NotImplemented}
  {$LINKLIB gdb}
  {$ifdef GDB_HAS_SIM}
    {$LINKLIB sim}
  {$endif GDB_HAS_SIM}
  {$LINKLIB bfd}
  {$LINKLIB readline}
  {$LINKLIB opcodes}
  {$LINKLIB history}
  {$LINKLIB iberty}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
  {$ifndef GDB_DISABLE_INTL}
    {$LINKLIB intl}
  {$endif ndef GDB_DISABLE_INTL}
{$endif NotImplemented}
  {$LINKLIB dbg}
  {$LINKLIB c}
{$endif go32v2}

{$ifdef linux}
{$ifdef NotImplemented}
  {$undef NotImplemented}
  {$LINKLIB libgdb.a}
  {$ifdef GDB_HAS_SIM}
    {$LINKLIB libsim.a}
  {$endif GDB_HAS_SIM}
  {$LINKLIB libbfd.a}
  {$LINKLIB libreadline.a}
  {$LINKLIB libopcodes.a}
  {$LINKLIB libhistory.a}
  {$LINKLIB libiberty.a}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
  {$LINKLIB ncurses}
{$endif NotImplemented}
  {$LINKLIB m}
  {$LINKLIB dl}
  {$LINKLIB c}
  {$LINKLIB gcc}
{$endif linux}

{$ifdef dragonfly}
{$ifdef NotImplemented}
  {$linklib kvm}
  {$undef NotImplemented}
  {$LINKLIB libgdb.a}
  {$ifdef GDB_HAS_SIM}
    {$LINKLIB libsim.a}
  {$endif GDB_HAS_SIM}
  {$LINKLIB libbfd.a}
  {$LINKLIB libreadline.a}
  {$LINKLIB libopcodes.a}
  {$LINKLIB libhistory.a}
  {$LINKLIB libiberty.a}
  {$LINKLIB libgnu.a}
  {$LINKLIB ncurses}
  {$LINKLIB z}
  {$LINKLIB m}
  {$LINKLIB iberty}
  {$ifndef GDB_DISABLE_INTL}
    {$LINKLIB intl}
  {$endif ndef GDB_DISABLE_INTL}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
{$endif NotImplemented}
  {$LINKLIB c}
  {$LINKLIB gcc}
{$endif freebsd}

{$ifdef freebsd}
{$ifdef NotImplemented}
  {$ifdef FreeBSD5}  //5.4+ ?
    {$linklib kvm}
  {$endif}
  {$undef NotImplemented}
  {$LINKLIB libgdb.a}
  {$ifdef GDB_HAS_SIM}
    {$LINKLIB libsim.a}
  {$endif GDB_HAS_SIM}
  {$LINKLIB libbfd.a}
  {$LINKLIB libreadline.a}
  {$LINKLIB libopcodes.a}
  {$LINKLIB libhistory.a}
  {$LINKLIB libiberty.a}
  {$LINKLIB libgnu.a} // at least 7.4 generates this.
  {$LINKLIB ncurses}
  {$LINKLIB z} // linked implictely by something on Linux
  {$LINKLIB m}
  {$LINKLIB iberty}
  {$ifndef GDB_DISABLE_INTL}
    {$LINKLIB intl}
  {$endif ndef GDB_DISABLE_INTL}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
     { does not seem to exist on netbsd LINKLIB dl,
                            but I use GDB CVS snapshots for the *BSDs}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
{$endif NotImplemented}
  {$LINKLIB c}
  {$LINKLIB gcc}
{$endif freebsd}

{$ifdef netbsd}
{$ifdef NotImplemented}
  {$undef NotImplemented}
  {$LINKLIB gdb}
  {$ifdef GDB_HAS_SIM}
    {$LINKLIB sim}
  {$endif GDB_HAS_SIM}
  {$LINKLIB bfd}
  {$LINKLIB readline}
  {$LINKLIB opcodes}
  {$LINKLIB history}
  {$LINKLIB iberty}
  {$LINKLIB ncurses}
  {$LINKLIB m}
  {$LINKLIB iberty}
  {$LINKLIB intl}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
  { does not seem to exist on netbsd LINKLIB dl}
{$endif NotImplemented}
  {$LINKLIB c}
  {$LINKLIB gcc}
{$endif netbsd}

{$ifdef solaris}
{$ifdef NotImplemented}
  {$undef NotImplemented}
  {$LINKLIB gdb}
  {$ifdef GDB_HAS_SIM}
    {$LINKLIB sim}
  {$endif GDB_HAS_SIM}
  {$LINKLIB bfd}
  {$LINKLIB readline}
  {$LINKLIB opcodes}
  {$LINKLIB history}
  {$LINKLIB iberty}
  {$LINKLIB curses}
  {$LINKLIB m}
  {$LINKLIB iberty}
  {$LINKLIB intl}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
{$endif NotImplemented}
  {$LINKLIB dl}
  {$LINKLIB socket}
  {$LINKLIB nsl}
  {$LINKLIB c}
{$endif solaris}

{$ifdef openbsd}
{$ifdef NotImplemented}
  {$undef NotImplemented}
  {$LINKLIB gdb}
  {$ifdef GDB_HAS_SIM}
    {$LINKLIB sim}
  {$endif GDB_HAS_SIM}
  {$LINKLIB bfd}
  {$LINKLIB readline}
  {$LINKLIB opcodes}
  {$LINKLIB history}
  {$LINKLIB iberty}
  {$LINKLIB ncurses}
  {$LINKLIB m}
  {$LINKLIB iberty}
  {$ifndef GDB_DISABLE_INTL}
    {$LINKLIB intl}
  {$endif ndef GDB_DISABLE_INTL}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
  { does not seem to exist on netbsd LINKLIB dl}
{$endif NotImplemented}
  {$LINKLIB c}
  {$LINKLIB gcc}
{$endif netbsd}

{$ifdef win32}
{$ifdef NotImplemented}
  {$undef NotImplemented}
  {$LINKLIB libgdb.a}
  {$ifdef GDB_HAS_SIM}
   {$LINKLIB libsim.a}
  {$endif GDB_HAS_SIM}
  {$LINKLIB libbfd.a}
  {$LINKLIB libreadline.a}
  {$LINKLIB libopcodes.a}
  {$LINKLIB libhistory.a}
  {$LINKLIB libiberty.a}

  {$ifdef USE_MINGW_GDB}
    {$LINKLIB libdecnumber.a}
    {$ifdef GDB_USES_LIBDECNUMBER}
      {$LINKLIB decnumber}
    {$endif GDB_USES_LIBDECNUMBER}
    {$ifdef GDB_USES_EXPAT_LIB}
      {$LINKLIB expat}
    {$endif GDB_USES_EXPAT_LIB}
    {$ifdef GDB_USES_LIBPYTHON}
      {$LINKLIB python}
    {$endif GDB_USES_LIBPYTHON}
  {$else not USE_MINGW_GDB}
    {$LINKLIB libiconv.a}
    {$LINKLIB libncurses.a}
    {$ifdef GDB_USES_LIBDECNUMBER}
      {$LINKLIB decnumber}
    {$endif GDB_USES_LIBDECNUMBER}
    {$ifdef GDB_USES_EXPAT_LIB}
      {$LINKLIB expat}
    {$endif GDB_USES_EXPAT_LIB}
    {$ifdef GDB_USES_LIBPYTHON}
      {$LINKLIB python}
    {$endif GDB_USES_LIBPYTHON}
  {$endif not USE_MINGW_GDB}
{$endif NotImplemented}
  {$ifdef USE_MINGW_GDB}
    {$LINKLIB libm.a}
    {$LINKLIB libmoldname.a}
    {$LINKLIB libgcc.a}
    {$LINKLIB libws2_32.a}
    {$LINKLIB libmingwex.a}
    {$LINKLIB libmingw32.a}
    {$LINKLIB libmsvcrt.a}
  {$else not USE_MINGW_GDB}
    {$LINKLIB gcc}
    {$LINKLIB cygwin} { alias of libm.a and libc.a }
  {$LINKLIB libintl.a}
  {$LINKLIB imagehlp}
  {$endif not USE_MINGW_GDB}
  {$ifdef GDB_USES_LIBADVAPI32}
    {$LINKLIB advapi32}
  {$endif GDB_USES_LIBADVAPI32}
  {$LINKLIB user32}
  {$LINKLIB kernel32}
{$endif win32}

{$ifdef win64}
{$ifdef NotImplemented}
  {$undef NotImplemented}
  {$LINKLIB libgdb.a}
 {$ifdef GDB_HAS_SIM}
  {$LINKLIB libsim.a}
 {$endif GDB_HAS_SIM}
  {$LINKLIB libbfd.a}
  {$LINKLIB libreadline.a}
  {$LINKLIB libopcodes.a}
  {$LINKLIB libhistory.a}
  {$LINKLIB libiberty.a}
  {$LINKLIB libintl.a}

  {$LINKLIB libdecnumber.a}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
{$endif NotImplemented}
  {$LINKLIB libm.a}
  {$LINKLIB libmoldname.a}
  {$LINKLIB libws2_32.a}
  {$LINKLIB libmingwex.a}
  {$LINKLIB libmingw32.a}
  {$LINKLIB libmsvcrt.a}
  {$LINKLIB libgcc.a}
  {$LINKLIB libws2_32.a}
  {$LINKLIB kernel32}
  {$LINKLIB user32}
{$endif win64}

{$ifdef beos}
{$ifdef NotImplemented}
  { still need some work... stollen from netbsd}
  {$undef NotImplemented}
  {$LINKLIB gdb}
  {$ifdef GDB_HAS_SIM}
    {$LINKLIB sim}
  {$endif GDB_HAS_SIM}
  {$LINKLIB bfd}
  {$LINKLIB readline}
  {$LINKLIB opcodes}
  { $ LINKLIB history}
  {$LINKLIB iberty}
  {$LINKLIB ncurses}
  { $ LINKLIB m} // include in libroot under BeOS
  {$ifndef GDB_DISABLE_INTL}
    {$LINKLIB intl}
  {$endif ndef GDB_DISABLE_INTL}
  {$ifdef GDB_USES_LIBDECNUMBER}
    {$LINKLIB decnumber}
  {$endif GDB_USES_LIBDECNUMBER}
  {$ifdef GDB_USES_EXPAT_LIB}
    {$LINKLIB expat}
  {$endif GDB_USES_EXPAT_LIB}
  {$ifdef GDB_USES_LIBPYTHON}
    {$LINKLIB python}
  {$endif GDB_USES_LIBPYTHON}
{$endif NotImplemented}
  { does not seem to exist on netbsd LINKLIB dl}
  { $ LINKLIB c} // This is libroot under BeOS, and always linked
  {$LINKLIB debug}
  {$LINKLIB gcc}
{$endif beos}

{$ifdef aix}
  { AIX linker requires more precise external/public separation }
  {$define NEED_EXTERNAL_CVAR}
  {$undef NotImplemented}
{$endif aix}

{$ifdef go32v2}
  {$define supportexceptions}
{$endif go32v2}
{$ifdef unix}
  {$define supportexceptions}
{$endif unix}

{$ifdef CROSSGDB}
  { do we need something special if cross GDB? }
{$endif CROSSGDB}

{$ifdef NotImplemented}
  {$fatal This OS is not yet supported !!!}
{$endif NotImplemented}

{$packrecords C}

type
{$if defined(CPUSPARC) and defined(LINUX)}
  {$define GDB_CORE_ADDR_FORCE_64BITS}
{$endif}
{$ifdef GDB_CORE_ADDR_FORCE_64BITS}
  CORE_ADDR = qword;
{$else}
  CORE_ADDR = ptruint; { might be target dependent PM }
{$endif}

  psyminfo=^tsyminfo;
  tsyminfo=record
    address  : ptrint;
    fname    : pchar;
    line     : longint;
    funcname : pchar;
    offset   : ptrint;
  end;

  tframeentry = object
    file_name : pchar;
    function_name : pchar;
    args : pchar;
    line_number : longint;
    address : CORE_ADDR;
    level : longint;
    constructor init;
    destructor done;
    procedure reset;
    procedure clear;
  end;
  pframeentry=^tframeentry;
  ppframeentry=^pframeentry;

{ needed for handles }
{not anymore I textrec.inc}

const
 k=1;

type
  streamtype = (afile,astring);
  C_FILE     = ptrint; { at least under DJGPP }
  P_C_FILE   = ^C_FILE;

type

  pui_file = ^ui_file;
  pstdio_file = ^stdio_file;

  ui_file_flush_ftype = procedure(stream : pui_file);cdecl;
  ui_file_write_ftype = procedure(stream : pui_file;buf : pchar;len : longint);cdecl;
  ui_file_write_async_save_ftype = procedure(stream : pui_file;buf : pchar;len : longint);cdecl;
  ui_file_fputs_ftype = procedure(buf : pchar; stream : pui_file);cdecl;
  ui_file_delete_ftype = procedure(stream : pui_file);cdecl;
  ui_file_isatty_ftype = function(stream : pui_file) : longbool;cdecl;
  ui_file_rewind_ftype = procedure(stream : pui_file);cdecl;
  ui_file_put_method_ftype = procedure(var _object; buffer : pchar;length_buffer : longint);cdecl;
  ui_file_put_ftype = procedure(stream : pui_file;method : ui_file_put_method_ftype;var context);cdecl;
  {$ifdef GDB_V6}
  ui_file_read_ftype = function (stream : pui_file; buffer : pchar; len : longint):longint;cdecl;
  {$endif}
  {$ifdef GDB_UI_FILE_HAS_FSEEK}
  ui_file_fseek_ftype = function (stream : pui_file; offset : longint{clong}; whence : longint {cint}) : longint{cint};cdecl;
  {$endif GDB_UI_FILE_HAS_FSEEK}

  ui_file = record
      magic : plongint;
      to_flush  : ui_file_flush_ftype;
      to_write  : ui_file_write_ftype;
      {$ifdef GDB_UI_FILE_HAS_WRITE_ASYNC}
      to_write_async_safe   : ui_file_write_async_save_ftype;
      {$endif}
      to_fputs  : ui_file_fputs_ftype;
      {$ifdef GDB_V6}
      to_read   : ui_file_read_ftype;
      {$endif}
      to_delete : ui_file_delete_ftype;
      to_isatty : ui_file_isatty_ftype;
      to_rewind : ui_file_rewind_ftype;
      to_put    : ui_file_put_ftype;
     {$ifdef GDB_UI_FILE_HAS_FSEEK}
     to_fseek   : ui_file_fseek_ftype;
     {$endif GDB_UI_FILE_HAS_FSEEK}
      to_data   : pointer;
    end;

  stdio_file = record
      magic : plongint;
      _file : P_C_FILE;
      df : longint;
      close_p : longint;
    end;

  { used to delete stdio_ui_file  gdb_stdout and gdb_stderr }
  procedure ui_file_delete(stream : pui_file);cdecl;external;

  { used to recreate gdb_stdout and gdb_stderr as memory streams }
  function mem_fileopen : pui_file;cdecl;external;
  function stdio_fileopen(_file : P_C_FILE) : pui_file;cdecl;external;

  { used to change the write procvar to ours }

  procedure set_ui_file_write(stream : pui_file;write : ui_file_write_ftype);cdecl;external;


  type

  (* struct ptid
  {
    /* Process id */
    int pid;

    /* Lightweight process id */
    long lwp;

    /* Thread id */
    long tid;
  }; *)
   pinferior_ptid = ^tinferior_ptid;
   tinferior_ptid = record
      pid : longint{C int};
      lwp : ptrint{ C long};
      tid : ptrint{ C long};
     end;

{$ifdef win32}

type
  { from sys/reent.h
    real structure is bigger but only std.. are wanted here PM }
  REENT = record
    err : longint;
    stdin,stdout,stderr : P_C_FILE;
  end;
  PREENT = ^REENT;

var _impure_ptr : PREENT;cvar;external;

{$endif win32}


type
  tgdbbuffer=object
    buf   : pchar;
    size,
    idx   : longint;
    gdb_file  : pui_file;
    constructor Init;
    destructor  Done;
    procedure Reset;
    procedure Resize(nsize : longint);
    procedure Append(p:pchar);
    procedure lappend(p:pchar;len : longint);
  end;

  pgdbinterface=^tgdbinterface;
  tgdbinterface=object
  private
    stop_breakpoint_number : longint;
  public
    gdberrorbuf,
    gdboutputbuf  : tgdbbuffer;
    got_error,
    reset_command,
    call_reset,
    signaled,
    Debuggee_started : boolean;
    { frames and frame info while recording a frame }
    frames        : ppframeentry;
    frame_size,
    frame_count   : longint;
    record_frames,
    frame_begin_seen : boolean;
    frame_level,
    command_level,
    current_line_number,
    signal_start,
    signal_end,
    signal_name_start,
    signal_name_end,
    error_start,
    error_end,
    function_start,
    function_end,
    args_start,
    args_end,
    file_start,
    file_end,
    line_start,
    line_end : longint;
    signal_name,
    signal_string : pchar;
    current_address,
    current_pc      : CORE_ADDR;
    { breakpoint }
    last_breakpoint_number,
    last_breakpoint_line : longint;
    last_breakpoint_address : CORE_ADDR;
    last_breakpoint_file : pchar;
    invalid_breakpoint_line : boolean;
    user_screen_shown,
    switch_to_user     : boolean;

    { init }
    constructor init;
    destructor  done;
    { Lowlevel }
    function  error:boolean;
    function  error_num:longint;
    procedure gdb_command(const s:string);
    procedure gdb__init;
    procedure gdb_done;
    procedure resize_frames;
    function  add_frameentry:pframeentry;
    function  get_frameentry(level : longint):pframeentry;
    function  get_current_frame : ptrint;
    function  set_current_frame(level : longint) : boolean;
    procedure clear_frames;
    { Highlevel }
    procedure GetAddrSyminfo(addr:ptrint;var si:tsyminfo);
    function SelectSourceline(fn:pchar;line,BreakIndex:longint): Boolean;
    procedure StartSession;
    procedure BreakSession;
    procedure EndSession(code:longint);
    procedure DebuggerScreen;
    procedure UserScreen;
    procedure FlushAll; virtual;
    function Query(question : pchar; args : pchar) : longint; virtual;
    { Hooks }
    function DoSelectSourceline(const fn:string;line,BreakIndex:longint): Boolean;virtual;
    procedure DoStartSession;virtual;
    procedure DoBreakSession;virtual;
    procedure DoEndSession(code:longint);virtual;
    procedure DoUserSignal;virtual;
    procedure DoDebuggerScreen;virtual;
    procedure DoUserScreen;virtual;
    function  AllowQuit : boolean;virtual;
  end;


const
  use_gdb_file : boolean = false;

var
  curr_gdb : pgdbinterface;
  gdb_file : text;
  inferior_ptid : tinferior_ptid;cvar;external;

function  GDBVersion : string;
function  inferior_pid : longint;

{$ifdef GDB_V6}
type
  ui_out = pointer;
{$ifdef GDB_CURRENT_UIOUT_MACRO}
type
  pui_out = ^ui_out;
function current_ui_current_uiout_ptr : ui_out;cdecl;external;
var
  cli_uiout : ui_out;
  current_uiout : ui_out;
  { out local copy for catch_exceptions call }
  our_uiout : ui_out;

type
  pui = ^ui;
  ui  = record
   { ui record }
   next : pui;
   num : longint;
  end;

{$ifdef GDB_NEW_UI}
var
  local_ui : pui;
  main_ui : pui; cvar;
  current_ui : pui; cvar;

function new_ui (instream, outstream,errstream: pui_file) : pui; cdecl;external;
{$endif GDB_NEW_UI}

{$else not GDB_CURRENT_UIOUT_MACRO}
{$ifndef GDB_NO_UIOUT}
var
  uiout : ui_out;cvar;external;
{$else  GDB_NO_UIOUT}
var
  cli_uiout : ui_out;cvar;external;
  current_uiout : ui_out;cvar;external;
  { out local copy for catch_exceptions call }
  our_uiout : ui_out;
{$endif GDB_NO_UIOUT}
{$endif not GDB_CURRENT_UIOUT_MACRO}
function cli_out_new (stream : pui_file):ui_out;cdecl;external;
{$endif GDB_V6}

{$ifdef go32v2}
  { needed to be sure %fs contains the DOS memory selector
    used in Mem[] code PM }
  procedure reload_fs;
{$endif go32v2}



implementation

uses
{$ifdef win32}
  {$ifdef USE_MINGW_GDB}
  {$else not USE_MINGW_GDB}
    initc,
  {$endif not USE_MINGW_GDB}
{$endif win32}
{$ifdef unix}
  baseunix,
{$endif}
{$ifdef go32v2}
  go32,
  dpmiexcp,
  initc,
{$endif}
  strings;

{*****************************************************************************
                          Types used by libgdb.a
*****************************************************************************}

{$ifdef go32v2}
type
  jmp_buf = dpmi_jmp_buf;
  pjmp_buf = pdpmi_jmp_buf;


  function setjmp(var rec : jmp_buf) : longint;cdecl;external;

  function malloc(size : longint) : pointer;cdecl;external;

  procedure longjmp(var rec : jmp_buf;return_value : longint);cdecl;external;

  procedure reload_fs;assembler;
  asm
     movw  dosmemselector,%ax
     movw  %ax,%fs
  end['EAX'];

{$endif}
{$ifdef win32}
type
  jmp_buf = record
  case byte of
    0 :
    { greatest value found in cygwin machine/setjmp.h for i386 }
    { mingw uses int[16] C type for i386 }
    (unknown_field : array [1..15] of longint;);
    1 :
    (eax,ebx,ecx,edx : longint;
    esi,edi,ebp,esp,eip : longint;);
  end;

  pjmp_buf = ^jmp_buf;
{$ifdef USE_MINGW_GDB}
  { for obscure reasons, longjmp and _setjmp are defined in mingw32 libmsvcrt.a }
  function _setjmp(var rec : jmp_buf) : longint; cdecl; external;
  procedure longjmp(var rec : jmp_buf;return_value : longint); cdecl; external;
  function setjmp(var rec : jmp_buf) : longint;
    begin
	  setjmp:=_setjmp(rec);
	end;
{$else not USE_MINGW_GDB}
  function setjmp(var rec : jmp_buf) : longint;cdecl;external;

  procedure longjmp(var rec : jmp_buf;return_value : longint);cdecl;external;
{$endif not USE_MINGW_GDB}

{$ifndef supportexceptions}
type
  { I don't think FPC would accept that
    the funcvar return type is the funcvar type itself ! PM }
  SignalHandler   = Procedure(Sig : LongInt);cdecl;
  function signal(sig : longint;new_signal : SignalHandler) : SignalHandler;cdecl;external;

{define supportexceptions not yet working }
{$endif now exceptions are supported for win32}
{$endif win32}

  type
     pCORE_ADDR = ^CORE_ADDR;
     pblock = ^block;

     tframe_id = record
       stack_addr, code_addr, special_addr : CORE_ADDR;
       addr_p_flags : byte;{ for three 1 bit flags
       stack_addr_p, code_addr_p, special_addr_p : cint : 1; }
       inline_depth : longint;
     end;

     tlanguage = (language_unknown,language_auto,language_c,
       language_cplus,language_java,language_chill,
       language_fortran,language_m2,language_asm,
       language_scm,language_pascal,language_objc);

     bptype = (
{$ifdef GDB_HAS_BP_NONE}
       bp_none,
{$endif GDB_HAS_BP_NONE}
       bp_breakpoint,bp_hardware_breakpoint,
       bp_until,bp_finish,bp_watchpoint,bp_hardware_watchpoint,
       bp_read_watchpoint,bp_access_watchpoint,
       bp_longjmp,bp_longjmp_resume,bp_step_resume,
       bp_through_sigtramp,bp_watchpoint_scope,
       bp_call_dummy,bp_shlib_event);

     tenable = (disabled,enabled,shlib_disabled);

     bpdisp = (del,del_at_next_stop,disable,donttouch);

     pbp_location = ^bp_location;

     bp_loc_type = (bp_loc_software_breakpoint, bp_loc_hardware_breakpoint,
                    bp_loc_hardware_watchpoint, bp_loc_other);


     target_hw_bp_type = (hw_write, hw_read, hw_access, hw_execute);

     { pointer to structures that we don't need }
     pbp_ops = pointer;
     pbp_location_ops = pointer;
     pprogram_space = pointer;
     pgdbarch = pointer;

{$PACKRECORDS C}
     pbreakpoint = ^breakpoint;
     breakpoint = record
{$ifdef GDB_USES_BP_OPS}
          ops : pbp_ops;
{$endif GDB_USES_BP_OPS}
          next : pbreakpoint;
          typ : bptype;
          enable : tenable;
          disposition : bpdisp;
          number : longint;
{$ifdef GDB_USES_BP_LOCATION}
          loc : pbp_location;
{$else not GDB_USES_BP_LOCATION}
          address : CORE_ADDR;
{$endif not GDB_USES_BP_LOCATION}
{$ifndef GDB_USES_BP_OPS}
          line_number : longint;
          source_file : pchar;
{$endif not GDB_USES_BP_OPS}
          silent : byte;
{$ifdef GDB_USES_BP_OPS}
          display_canonical: byte;
{$endif GDB_USES_BP_OPS}
          ignore_count : longint;
{$ifdef GDB_BP_HAS_ENABLE_COUNT}
          enable_count : longint;
{$endif GDB_BP_HAS_ENABLE_COUNT}
{$ifndef GDB_USES_BP_LOCATION}
          shadow_contents : array[0..15] of char;
          inserted : char;
          duplicate : char;
{$endif not GDB_USES_BP_LOCATION}

          commands : pointer; {^command_line}
{$ifdef GDB_USES_BP_OPS}
          frame_id : tframe_id;
          pspace : pprogram_space;
{$else not GDB_USES_BP_OPS}
          frame : CORE_ADDR;
          cond : pointer; {^expression}
{$endif GDB_USES_BP_OPS}
          addr_string : pchar;
{$ifdef GDB_USES_BP_OPS}
          filter : pchar;
          addr_string_range_end : pchar;
          gdbarch : pgdbarch;
{$endif GDB_USES_BP_OPS}
          language : tlanguage;
          input_radix : longint;
          cond_string : ^char;
          exp_string : ^char;
          exp : pointer; {^expression}
          exp_valid_block : pblock; {^block;}
          val : pointer; {value_ptr;}
          val_chain : pointer; {value_ptr;}
          related_breakpoint : pbreakpoint;
          watchpoint_frame : CORE_ADDR;
          thread : longint;
          hit_count : longint;
          section : pointer; {^asection}
       end;

     pagent_expr = pointer;
     tcondition_status = (condition_unchanged, condition_modified);

     bp_target_info = record
          placed_address_space : pointer;{paddress_space;}
          placed_address : CORE_ADDR;
{$ifdef GDB_BP_TI_HAS_LENGTH}
          length : longint;
{$endif GDB_BP_TI_HAS_LENGTH}
          shadow_contents : array[0..15] of char;
          shadow_len : longint;
          placed_size : longint;
       end;

     bp_location = record
         next : pbp_location;
{$ifdef GDB_BP_LOCATION_HAS_OPS}
         ops : pbp_location_ops;
{$endif GDB_BP_LOCATION_HAS_OPS}

{$ifdef GDB_BP_LOCATION_HAS_REFCOUNT}
        refc : longint;
{$else}
{$ifdef GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
         global_next : pbp_location;
{$endif GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
{$endif}
         loc_type : bp_loc_type;
         owner : pbreakpoint;
{$ifdef GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
         cond : pointer;{pexpression;}
{$ifdef GDB_BP_LOCATION_HAS_COND_BYTECODE}
         cond_bytecode : pagent_expr;
         condition_changed : tcondition_status;
         cmd_bytecode : pagent_expr;
         needs_update : byte;
{$endif}
         shlib_disabled : byte;
         enabled : byte;
{$endif GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
         inserted : byte;
         duplicate : byte;
{$ifdef GDB_BP_LOCATION_HAS_GDBARCH}
         gdbarch : pgdbarch;
{$endif GDB_BP_LOCATION_HAS_GDBARCH}
{$ifdef GDB_HAS_PROGRAM_SPACE}
         pspace : pprogram_space;
{$endif GDB_HAS_PROGRAM_SPACE}
         address : CORE_ADDR;
{$ifdef GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
         length : longint;
         watchpoint_type : target_hw_bp_type;
{$endif GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
         section : pointer;{pobj_section;}
         requested_address : CORE_ADDR;
{$ifdef GDB_BP_LOCATION_HAS_RELATED_ADDRESS}
         related_address : CORE_ADDR;
         probe : pointer; { struct probe *probe; }
{$endif}
{$ifdef GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
         function_name : ^char;
{$endif GDB_BP_LOCATION_HAS_GLOBAL_NEXT}
         target_info : bp_target_info;
         overlay_target_info : bp_target_info;
         events_till_retirement : longint;
{$ifdef GDB_USES_BP_OPS}
        { line and source file are in location }
          line_number : longint;
          source_file : pchar;
{$endif not GDB_USES_BP_OPS}
      end;

     tfreecode=(free_nothing,free_contents,free_linetable);

     psymtab = ^symtab;
     symtab = record
          next : psymtab;
{$ifdef SYMTAB_HAS_COMPUNIT_SYMTAB}
          comp_unit : pointer; {^compunit_symtab }
          linetable : pointer; {^linetable;}
          filename : pchar;
          nlines : longint;
          line_charpos : ^longint;
          language : tlanguage;
          fullname : pchar;
{$else not SYMTAB_HAS_COMPUNIT_SYMTAB}
          blockvector : pointer; {^blockvector;}
          linetable : pointer; {^linetable;}
          block_line_section : longint;
          primary : longint;
          {$ifdef GDB_SYMTAB_HAS_MACROS}
            { new field added in the middle :( }
          macro_table : pointer;
          {$endif GDB_SYMTAB_HAS_MACROS}
          filename : pchar;
          dirname : pchar;
          free_code : tfreecode;
          free_ptr : pchar;
          nlines : longint;
          line_charpos : ^longint;
          language : tlanguage;
          Debugformat : pchar;
          version : pchar;
          fullname : pchar;
          objfile : pointer; {^objfile;}
{$endif not SYMTAB_HAS_COMPUNIT_SYMTAB}
       end;

     psymtab_and_line = ^symtab_and_line;
     symtab_and_line = record
         {$ifdef GDB_HAS_PROGRAM_SPACE}
         pspace : pointer;
         {$endif GDB_HAS_PROGRAM_SPACE}
          symtab : psymtab;
          section : pointer; {^asection;}
          line : longint;
          pc : CORE_ADDR;
          _end : CORE_ADDR;
          { Added fields, not used in gdbint,
            but necessary to allocated enough space to
            avoid stack memory corruption PM }
          explicit_pc : longint;
          explicit_line : longint;
          { New field added in GDB 7.5 version }
          probe : pointer;{struct probe *probe; }
          { New field added in GDB 7.8? version }
          objfile : pointer; { struct objfile * }
       end;

     symtabs_and_lines = record
          sals : ^symtab_and_line;
          nelts : longint;
       end;

    psymbol = ^symbol;
    pminimal_symbol = ^minimal_symbol;

    general_symbol_info = record
  (* Name of the symbol.  This is a required field.  Storage for the name is
     allocated on the psymbol_obstack or symbol_obstack for the associated
     objfile. *)

      _name : pchar;

  (* Value of the symbol.  Which member of this union to use, and what
     it means, depends on what kind of symbol this is and its
     SYMBOL_CLASS.  See comments there for more details.  All of these
     are in host byte order (though what they point to might be in
     target byte order, e.g. LOC_CONST_BYTES).  *)
     value : record
       case integer of
      (* The fact that this is a long not a LONGEST mainly limits the
         range of a LOC_CONST.  Since LOC_CONST_BYTES exists, I'm not
         sure that is a big deal.  *)
       0 : (ivalue : longint;);

       1 : (block  : pblock;);

       2 : (bytes  : pchar;);

       3 : (address : CORE_ADDR;);

      (* for opaque typedef struct chain *)

       4 : (chain : psymbol;);
      end;

  (* Since one and only one language can apply, wrap the language specific
     information inside a union. *)

   (* union
    {
      struct cplus_specific      /* For C++ */
                                /*  and Java */
        {
          char *demangled_name;
        } cplus_specific;
      struct chill_specific      /* For Chill */
        {
          char *demangled_name;
        } chill_specific;
    } language_specific; *)
     demangled_name : pchar;

  (* Record the source code language that applies to this symbol.
     This is used to select one of the fields from the language specific
     union above. *)

    language : tlanguage;

  (* Which section is this symbol in?  This is an index into
     section_offsets for this objfile.  Negative means that the symbol
     does not get relocated relative to a section.
     Disclaimer: currently this is just used for xcoff, so don't
     expect all symbol-reading code to set it correctly (the ELF code
     also tries to set it correctly).  *)

    section : word;

  (* The bfd section associated with this symbol. *)

    bfd_section : pointer {^asection};
  end; { of general_symbol_info record declaration }

  tminimal_symbol_type =
    (
      mst_unknown := 0,         (* Unknown type, the default *)
      mst_text,                 (* Generally executable instructions *)
      mst_data,                 (* Generally initialized data *)
      mst_bss,                  (* Generally uninitialized data *)
      mst_abs,                  (* Generally absolute (nonrelocatable) *)
      (* GDB uses mst_solib_trampoline for the start address of a shared
         library trampoline entry.  Breakpoints for shared library functions
         are put there if the shared library is not yet loaded.
         After the shared library is loaded, lookup_minimal_symbol will
         prefer the minimal symbol from the shared library (usually
         a mst_text symbol) over the mst_solib_trampoline symbol, and the
         breakpoints will be moved to their true address in the shared
         library via breakpoint_re_set.  *)
      mst_solib_trampoline,     (* Shared library trampoline code *)
      (* For the mst_file* types, the names are only guaranteed to be unique
         within a given .o file.  *)
      mst_file_text,            (* Static version of mst_text *)
      mst_file_data,            (* Static version of mst_data *)
      mst_file_bss              (* Static version of mst_bss *)
    );

  namespace_enum = (
  (* UNDEF_NAMESPACE is used when a namespace has not been discovered or
     none of the following apply.  This usually indicates an error either
     in the symbol information or in gdb's handling of symbols. *)
  UNDEF_NAMESPACE,

  (* VAR_NAMESPACE is the usual namespace.  In C, this contains variables,
     function names, typedef names and enum type values. *)
  VAR_NAMESPACE,

  (* STRUCT_NAMESPACE is used in C to hold struct, union and enum type names.
     Thus, if `struct foo' is used in a C program, it produces a symbol named
     `foo' in the STRUCT_NAMESPACE. *)
  STRUCT_NAMESPACE,

  (* LABEL_NAMESPACE may be used for names of labels (for gotos);
     currently it is not used and labels are not recorded at all.  *)
  LABEL_NAMESPACE,

  (* Searching namespaces. These overlap with VAR_NAMESPACE, providing
     some granularity with the search_symbols function. *)
  (* Everything in VAR_NAMESPACE minus FUNCTIONS_-, TYPES_-, and
     METHODS_NAMESPACE *)
  VARIABLES_NAMESPACE,

  (* All functions -- for some reason not methods, though. *)
  FUNCTIONS_NAMESPACE,

  (* All defined types *)
  TYPES_NAMESPACE,

  (* All class methods -- why is this separated out? *)
  METHODS_NAMESPACE

  );
  address_class = (
  (* Not used; catches errors *)
  LOC_UNDEF,

  (* Value is constant int SYMBOL_VALUE, host byteorder *)
  LOC_CONST,

  (* Value is at fixed address SYMBOL_VALUE_ADDRESS *)
  LOC_STATIC,

  (* Value is in register.  SYMBOL_VALUE is the register number.  *)
  LOC_REGISTER,

  (* It's an argument; the value is at SYMBOL_VALUE offset in arglist.  *)
  LOC_ARG,

  (* Value address is at SYMBOL_VALUE offset in arglist.  *)
  LOC_REF_ARG,

  (* Value is in register number SYMBOL_VALUE.  Just like LOC_REGISTER
     except this is an argument.  Probably the cleaner way to handle
     this would be to separate address_class (which would include
     separate ARG and LOCAL to deal with FRAME_ARGS_ADDRESS versus
     FRAME_LOCALS_ADDRESS), and an is_argument flag.

     For some symbol formats (stabs, for some compilers at least),
     the compiler generates two symbols, an argument and a register.
     In some cases we combine them to a single LOC_REGPARM in symbol
     reading, but currently not for all cases (e.g. it's passed on the
     stack and then loaded into a register).  *)
  LOC_REGPARM,

  (* Value is in specified register.  Just like LOC_REGPARM except the
     register holds the address of the argument instead of the argument
     itself. This is currently used for the passing of structs and unions
     on sparc and hppa.  It is also used for call by reference where the
     address is in a register, at least by mipsread.c.  *)
  LOC_REGPARM_ADDR,

  (* Value is a local variable at SYMBOL_VALUE offset in stack frame.  *)
  LOC_LOCAL,

  (* Value not used; definition in SYMBOL_TYPE.  Symbols in the namespace
     STRUCT_NAMESPACE all have this class.  *)
  LOC_TYPEDEF,

  (* Value is address SYMBOL_VALUE_ADDRESS in the code *)
  LOC_LABEL,

  (* In a symbol table, value is SYMBOL_BLOCK_VALUE of a `struct block'.
     In a partial symbol table, SYMBOL_VALUE_ADDRESS is the start address
     of the block.  Function names have this class. *)
  LOC_BLOCK,

  (* Value is a constant byte-sequence pointed to by SYMBOL_VALUE_BYTES, in
     target byte order.  *)
  LOC_CONST_BYTES,

  (* Value is arg at SYMBOL_VALUE offset in stack frame. Differs from
     LOC_LOCAL in that symbol is an argument; differs from LOC_ARG in
     that we find it in the frame (FRAME_LOCALS_ADDRESS), not in the
     arglist (FRAME_ARGS_ADDRESS).  Added for i960, which passes args
     in regs then copies to frame.  *)
  LOC_LOCAL_ARG,

  (* Value is at SYMBOL_VALUE offset from the current value of
     register number SYMBOL_BASEREG.  This exists mainly for the same
     things that LOC_LOCAL and LOC_ARG do; but we need to do this
     instead because on 88k DWARF gives us the offset from the
     frame/stack pointer, rather than the offset from the "canonical
     frame address" used by COFF, stabs, etc., and we don't know how
     to convert between these until we start examining prologues.

     Note that LOC_BASEREG is much less general than a DWARF expression.
     We don't need the generality (at least not yet), and storing a general
     DWARF expression would presumably take up more space than the existing
     scheme.  *)
  LOC_BASEREG,

  (* Same as LOC_BASEREG but it is an argument.  *)
  LOC_BASEREG_ARG,

  (* Value is at fixed address, but the address of the variable has
     to be determined from the minimal symbol table whenever the
     variable is referenced.
     This happens if debugging information for a global symbol is
     emitted and the corresponding minimal symbol is defined
     in another object file or runtime common storage.
     The linker might even remove the minimal symbol if the global
     symbol is never referenced, in which case the symbol remains
     unresolved.  *)
  LOC_UNRESOLVED,

  (* Value is at a thread-specific location calculated by a
     target-specific method. *)
  LOC_THREAD_LOCAL_STATIC,

  (* The variable does not actually exist in the program.
     The value is ignored.  *)
  LOC_OPTIMIZED_OUT,

  (* The variable is static, but actually lives at * (address).
   * I.e. do an extra indirection to get to it.
   * This is used on HP-UX to get at globals that are allocated
   * in shared libraries, where references from images other
   * than the one where the global was allocated are done
   * with a level of indirection.
   *)
  LOC_INDIRECT
  );

   minimal_symbol = record
  (* The general symbol info required for all types of symbols.
     The SYMBOL_VALUE_ADDRESS contains the address that this symbol
     corresponds to.  *)
    ginfo : general_symbol_info;

  (* The info field is available for caching machine-specific information
     so it doesn't have to rederive the info constantly (over a serial line).
     It is initialized to zero and stays that way until target-dependent code
     sets it.  Storage for any data pointed to by this field should be allo-
     cated on the symbol_obstack for the associated objfile.
     The type would be "void *" except for reasons of compatibility with older
     compilers.  This field is optional.

     Currently, the AMD 29000 tdep.c uses it to remember things it has decoded
     from the instructions in the function header, and the MIPS-16 code uses
     it to identify 16-bit procedures.  *)

    info : pchar;

{$ifdef SOFUN_ADDRESS_MAYBE_MISSING}
  (* Which source file is this symbol in?  Only relevant for mst_file_*.  *)
    filename : pchar;
{$endif}

  (* Classification types for this symbol.  These should be taken as "advisory
     only", since if gdb can't easily figure out a classification it simply
     selects mst_unknown.  It may also have to guess when it can't figure out
     which is a better match between two types (mst_data versus mst_bss) for
     example.  Since the minimal symbol info is sometimes derived from the
     BFD library's view of a file, we need to live with what information bfd
     supplies. *)

    minimal_symbol_type : tminimal_symbol_type;
  end{ of minimal_symbol};

  block = record
  (* Addresses in the executable code that are in this block.  *)
  startaddr,
  endaddr : CORE_ADDR ;

  (* The symbol that names this block, if the block is the body of a
     function; otherwise, zero.  *)
  _function : psymbol;

  (* The `struct block' for the containing block, or 0 if none.
     The superblock of a top-level local block (i.e. a function in the
     case of C) is the STATIC_BLOCK.  The superblock of the
     STATIC_BLOCK is the GLOBAL_BLOCK.  *)

  superblock : pblock;

  (* Version of GCC used to compile the function corresponding
     to this block, or 0 if not compiled with GCC.  When possible,
     GCC should be compatible with the native compiler, or if that
     is not feasible, the differences should be fixed during symbol
     reading.  As of 16 Apr 93, this flag is never used to distinguish
     between gcc2 and the native compiler.

     If there is no function corresponding to this block, this meaning
     of this flag is undefined.  *)

  gcc_compile_flag : byte;

  (* Number of local symbols.  *)
  nsyms : longint;

  (* The symbols.  If some of them are arguments, then they must be
     in the order in which we would like to print them.  *)
  sym : array [0..0] of psymbol;
  end { of block definition };

  symbol = record
  (* The general symbol info required for all types of symbols. *)
    ginfo : general_symbol_info;

  (* Data type of value *)
    _type : pointer{ptype};

  (* Name space code.  *)
  namespace : namespace_enum;

  (* Address class *)

  aclass : address_class;

  (* Line number of definition.  FIXME:  Should we really make the assumption
     that nobody will try to debug files longer than 64K lines?  What about
     machine generated programs? *)

  line : word;

  (* Some symbols require an additional value to be recorded on a per-
     symbol basis.  Stash those values here. *)

  (*union
    {
      /* Used by LOC_BASEREG and LOC_BASEREG_ARG.  */
      short basereg;
    } *)
  aux_value_base_reg : word;

  (* Link to a list of aliases for this symbol.
     Only a "primary/main symbol may have aliases.  *)
  aliases : pointer{palias_list};

  (* List of ranges where this symbol is active.  This is only
     used by alias symbols at the current time.  *)
  ranges : pointer{prange_list};
  end;

     target_signal = (TARGET_SIGNAL_FIRST := 0,
       TARGET_SIGNAL_HUP := 1,TARGET_SIGNAL_INT := 2,
       TARGET_SIGNAL_QUIT := 3,TARGET_SIGNAL_ILL := 4,
       TARGET_SIGNAL_TRAP := 5,TARGET_SIGNAL_ABRT := 6,
       TARGET_SIGNAL_EMT := 7,TARGET_SIGNAL_FPE := 8,
       TARGET_SIGNAL_KILL := 9,TARGET_SIGNAL_BUS := 10,
       TARGET_SIGNAL_SEGV := 11,TARGET_SIGNAL_SYS := 12,
       TARGET_SIGNAL_PIPE := 13,TARGET_SIGNAL_ALRM := 14,
       TARGET_SIGNAL_TERM := 15,TARGET_SIGNAL_URG := 16,
       TARGET_SIGNAL_STOP := 17,TARGET_SIGNAL_TSTP := 18,
       TARGET_SIGNAL_CONT := 19,TARGET_SIGNAL_CHLD := 20,
       TARGET_SIGNAL_TTIN := 21,TARGET_SIGNAL_TTOU := 22,
       TARGET_SIGNAL_IO := 23,TARGET_SIGNAL_XCPU := 24,
       TARGET_SIGNAL_XFSZ := 25,TARGET_SIGNAL_VTALRM := 26,
       TARGET_SIGNAL_PROF := 27,TARGET_SIGNAL_WINCH := 28,
       TARGET_SIGNAL_LOST := 29,TARGET_SIGNAL_USR1 := 30,
       TARGET_SIGNAL_USR2 := 31,TARGET_SIGNAL_PWR := 32,
       TARGET_SIGNAL_POLL := 33,TARGET_SIGNAL_WIND := 34,
       TARGET_SIGNAL_PHONE := 35,TARGET_SIGNAL_WAITING := 36,
       TARGET_SIGNAL_LWP := 37,TARGET_SIGNAL_DANGER := 38,
       TARGET_SIGNAL_GRANT := 39,TARGET_SIGNAL_RETRACT := 40,
       TARGET_SIGNAL_MSG := 41,TARGET_SIGNAL_SOUND := 42,
       TARGET_SIGNAL_SAK := 43,TARGET_SIGNAL_PRIO := 44,
       TARGET_SIGNAL_REALTIME_33 := 45,TARGET_SIGNAL_REALTIME_34 := 46,
       TARGET_SIGNAL_REALTIME_35 := 47,TARGET_SIGNAL_REALTIME_36 := 48,
       TARGET_SIGNAL_REALTIME_37 := 49,TARGET_SIGNAL_REALTIME_38 := 50,
       TARGET_SIGNAL_REALTIME_39 := 51,TARGET_SIGNAL_REALTIME_40 := 52,
       TARGET_SIGNAL_REALTIME_41 := 53,TARGET_SIGNAL_REALTIME_42 := 54,
       TARGET_SIGNAL_REALTIME_43 := 55,TARGET_SIGNAL_REALTIME_44 := 56,
       TARGET_SIGNAL_REALTIME_45 := 57,TARGET_SIGNAL_REALTIME_46 := 58,
       TARGET_SIGNAL_REALTIME_47 := 59,TARGET_SIGNAL_REALTIME_48 := 60,
       TARGET_SIGNAL_REALTIME_49 := 61,TARGET_SIGNAL_REALTIME_50 := 62,
       TARGET_SIGNAL_REALTIME_51 := 63,TARGET_SIGNAL_REALTIME_52 := 64,
       TARGET_SIGNAL_REALTIME_53 := 65,TARGET_SIGNAL_REALTIME_54 := 66,
       TARGET_SIGNAL_REALTIME_55 := 67,TARGET_SIGNAL_REALTIME_56 := 68,
       TARGET_SIGNAL_REALTIME_57 := 69,TARGET_SIGNAL_REALTIME_58 := 70,
       TARGET_SIGNAL_REALTIME_59 := 71,TARGET_SIGNAL_REALTIME_60 := 72,
       TARGET_SIGNAL_REALTIME_61 := 73,TARGET_SIGNAL_REALTIME_62 := 74,
       TARGET_SIGNAL_REALTIME_63 := 75,TARGET_SIGNAL_UNKNOWN,
       TARGET_SIGNAL_DEFAULT,TARGET_SIGNAL_LAST
       );

     strata = (dummy_stratum,file_stratum,core_stratum,download_stratum,process_stratum);

     ptarget_ops = ^target_ops;
     target_ops = record
          to_shortname : pchar;
          to_longname : pchar;
          to_doc : pchar;
          to_open : procedure (_para1:pchar; _para2:longint);
          to_close : procedure (_para1:longint);
          to_attach : procedure (_para1:pchar; _para2:longint);
          to_detach : procedure (_para1:pchar; _para2:longint);
          to_resume : procedure (_para1:longint; _para2:longint; _para3:target_signal);
          to_wait : pointer; {function (_para1:longint; _para2:ptarget_waitstatus):longint;}
          to_fetch_registers : procedure (_para1:longint);
          to_store_registers : procedure (_para1:longint);
          to_prepare_to_store : procedure ;
          to_xfer_memory : function (memaddr:CORE_ADDR; myaddr:pchar; len:longint; write:longint; target:ptarget_ops):longint;
          to_files_info : procedure (_para1:ptarget_ops);
          to_insert_breakpoint : function (_para1:CORE_ADDR; _para2:pchar):longint;
          to_remove_breakpoint : function (_para1:CORE_ADDR; _para2:pchar):longint;
          to_terminal_init : procedure ;
          to_terminal_inferior : procedure ;
          to_terminal_ours_for_output : procedure ;
          to_terminal_ours : procedure ;
          to_terminal_info : procedure (_para1:pchar; _para2:longint);
          to_kill : procedure ;
          to_load : procedure (_para1:pchar; _para2:longint);
          to_lookup_symbol : function (_para1:pchar; _para2:pCORE_ADDR):longint;
          to_create_inferior : procedure (_para1:pchar; _para2:pchar; _para3:ppchar);
          to_mourn_inferior : procedure ;
          to_can_run : function :longint;
          to_notice_signals : procedure (pid:longint);
          to_thread_alive : function (pid:longint):longint;
          to_stop : procedure ;
          to_stratum : strata;
          DONT_USE : pointer;
          to_has_all_memory : longint;
          to_has_memory : longint;
          to_has_stack : longint;
          to_has_registers : longint;
          to_has_execution : longint;
          to_sections : pointer; {^section_table}
          to_sections_end : pointer; {^section_table}
          to_magic : longint;
       end;

{$PACKRECORDS C}

{*****************************************************************************
                   Define external calls to libgdb.a
*****************************************************************************}

var
{ external variables }
  error_return : jmp_buf;cvar;public;
  quit_return  : jmp_buf;cvar;public;
  deprecated_query_hook : pointer;cvar;
{$ifdef NEED_EXTERNAL_CVAR}external;{$else}public;{$endif}

  {$ifndef GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
    {$ifdef GDB_HAS_DEPRECATED_CBPH}
    deprecated_create_breakpoint_hook : pointer;cvar;external;
    {$else}
    create_breakpoint_hook : pointer;cvar;external;
    {$endif}
  {$endif ndef GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
  current_target : target_ops;cvar;external;
  stop_pc      : CORE_ADDR;cvar;external;
  { Only used from GDB 5.0 but doesn't hurst otherwise }
  { This global variable is declared in defs.h as external
    and instanciated in main.c since version 5.0. }
  interpreter_p : pchar;cvar;public;

{ we need also to declare some vars }
  watchdog      : longint;cvar;external;
  gdb_error     : longint;cvar;public;
  display_time  : longbool;cvar;public;
  display_space : longbool;cvar;public;

{ Whether this is the command line version or not }
  tui_version : longint;cvar;public;

{ Whether xdb commands will be handled }
{$ifdef GDB_HAS_DB_COMMANDS}
  { These two global variables are declared in defs.h
    since version 4.18 }
  xdb_commands : longint;cvar;public;

{ Whether dbx commands will be handled }
  dbx_commands : longint;cvar;public;
{$endif GDB_HAS_DB_COMMANDS}

{$ifdef GDB_NEEDS_SET_INSTREAM}
{$ifndef GDB_NO_INSTREAM_VAR}
var
  instream : P_C_FILE;cvar;external;
{$endif not GDB_NO_INSTREAM_VAR}

  function gdb_fopen (filename : pchar; mode : pchar) : pui_file;cdecl;external;
{$ifdef LIBGDB_HAS_GET_STDIN}
  { this function is generated by the gen-libgdb-inc.sh script
    in a object called gdb_get_stdin.o added to the libgdb.a archive }
  function gdb_get_stdin : P_C_FILE; cdecl; external;
  function gdb_get_stdout : P_C_FILE; cdecl; external;
  function gdb_get_stderr : P_C_FILE; cdecl; external;
{$ifdef GDB_HAS_SAVED_COMMAND_LINE_BUT_NO_SIZE}
var
  saved_command_line : pchar;cvar;external; { defined in top.c source }
const
  saved_command_line_size : longint = 100; {not anymore in top.c source }
  {$define GDB_SET_SAVED_COMMAND_LINE}
{$else}
 {$ifdef GDB_HAS_SAVED_COMMAND_LINE_SIZE}
  { In some GDB versions, saved_command_line needs to 
    be explicitly allocated at startup }
var
  saved_command_line : pchar;cvar;external; { defined in top.c source }
  saved_command_line_size : longint;cvar;external; {defined in top.c source }
  {$define GDB_SET_SAVED_COMMAND_LINE}
 {$endif def GDB_HAS_SAVED_COMMAND_LINE_SIZE}
{$endif def GDB_HAS_SAVED_COMMAND_LINE_BUT_NO_SIZE}
{$endif}
{$endif GDB_NEEDS_SET_INSTREAM}
var
  { The four following variables are defined in defs.h
    and instanciated in main.c since version 5.0 }
  gdb_stdout : pui_file;cvar;public;
  gdb_stderr : pui_file;cvar;public;
  gdb_stdlog : pui_file;cvar;public;
  gdb_stdtarg : pui_file;cvar;public;
  event_loop_p : longint;cvar;public;
{$ifdef GDB_V6}
(* target IO streams *)
  { The three following variables are declared in defs.h
    and instanciated in main.c since version 6.0 }
  gdb_stdin : pui_file;cvar;public;
  gdb_stdtargin : pui_file;cvar;public;
  gdb_stdtargerr : pui_file;cvar;public;
{$endif}

{ used for gdb_stdout and gdb_stderr }
function  xmalloc(size : longint) : pointer;cdecl;external;
{ used for QueryHook }
{ xvasprintf is present at least from GDB 5.3
  while xstrvprintf only appears in version 6.2,
  so only use xvasprintf function }
{$ifdef GDB_USE_XSTRVPRINTF}
function xstrvprintf(msg : pchar) : pchar; varargs; cdecl; external;
{$else}
function xvasprintf(ret : ppchar; msg : pchar) : pchar; varargs; cdecl; external;
{$endif}
procedure xfree(p : pointer); cdecl; external;
function  find_pc_line(i:CORE_ADDR;l:longint):symtab_and_line;cdecl;external;
function  find_pc_function(i:CORE_ADDR):psymbol;cdecl;external;
function  lookup_minimal_symbol_by_pc(i : CORE_ADDR):pminimal_symbol;cdecl;external;
{$ifdef GDB_INIT_HAS_ARGV0}
procedure gdb_init(argv0 : pchar);cdecl;external;
{$else not GDB_INIT_HAS_ARGV0}
procedure gdb_init;cdecl;external;
{$endif not GDB_INIT_HAS_ARGV0}
procedure execute_command(p:pchar;i:longint);cdecl;external;
{$ifdef GDB_TARGET_CLOSE_HAS_PTARGET_ARG}
procedure target_kill;cdecl;external;
procedure target_close(pt : ptarget_ops; i:longint);cdecl;external;
{$else not GDB_TARGET_CLOSE_HAS_PTARGET_ARG}
procedure target_close(i:longint);cdecl;external;
{$endif ndef GDB_TARGET_CLOSE_HAS_PTARGET_ARG}


{*****************************************************************************
                                 Helpers
*****************************************************************************}

procedure Debug(const s:string);
begin
  if use_gdb_file then
    Writeln(gdb_file,s)
  else
    Writeln(s);
end;


{*****************************************************************************
                               TFrameEntry
*****************************************************************************}

constructor tframeentry.init;
begin
  Reset;
end;

destructor tframeentry.done;
begin
  Clear;
end;

procedure tframeentry.reset;
begin
  file_name:=nil;
  function_name:=nil;
  args:=nil;
  line_number:=0;
  address:=0;
  level:=0;
end;

procedure tframeentry.clear;
begin
  if assigned(file_name) then
   strdispose(file_name);
  if assigned(function_name) then
   strdispose(function_name);
  if assigned(args) then
   strdispose(args);
  reset;
end;


{*****************************************************************************
                                 tgdbbuffer
*****************************************************************************}

const
  blocksize=2048;

constructor tgdbbuffer.init;
begin
  Buf:=nil;
  gdb_file:=nil;
  Size:=0;
  Resize(blocksize);
  Reset;
end;


destructor tgdbbuffer.done;
begin
  if assigned(buf) then
    freemem(buf,size);
end;



procedure tgdbbuffer.reset;
begin
  idx:=0;
  Buf[0]:=#0;
end;


procedure tgdbbuffer.append(p:pchar);
var
  len : longint;
begin
  if not assigned(p) then
   exit;
  len:=Strlen(p);
  if len+1+idx>size then
   Resize(len+1+idx);
  Move(p^,buf[idx],len);
  inc(idx,len);
  buf[idx]:=#0;
end;


procedure tgdbbuffer.lappend(p:pchar;len : longint);
begin
  if not assigned(p) then
   exit;
  if len+idx+1>size then
   Resize(len+idx+1);
  Move(p^,buf[idx],len);
  inc(idx,len);
  buf[idx]:=#0;
end;


procedure tgdbbuffer.resize(nsize : longint);
var
  np    : pchar;
begin
  nsize:=((nsize+blocksize-1) div blocksize)*blocksize;
  getmem(np,nsize);
  if assigned(buf) then
    begin
       move(buf^,np^,size);
       freemem(buf,size);
    end;
  buf:=np;
  size:=nsize;
end;


{*****************************************************************************
                         Hook calls from libgdb.a
*****************************************************************************}

{$ifdef go32v2}
procedure gdbpas_prev_exception_handler;cdecl;public;
begin
end;
{$endif go32v2}

procedure init_proc;cdecl;public;
begin
end;


procedure annotate_signalled;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|signalled|');
{$endif}
end;


procedure annotate_signal_name;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|signal_name|');
{$endif}
  with curr_gdb^ do
   signal_name_start:=gdboutputbuf.idx;
end;


procedure annotate_signal_name_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|signal_name_end|');
{$endif}
  with curr_gdb^ do
   signal_name_end:=gdboutputbuf.idx;
end;


procedure annotate_signal_string;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|signal_string|');
{$endif}
  with curr_gdb^ do
   signal_start:=gdboutputbuf.idx;
end;


procedure annotate_signal_string_end;cdecl;public;
var
  c : char;
begin
{$ifdef Verbose}
  Debug('|signal_string_end|');
{$endif}
  with curr_gdb^ do
   begin
     signal_end:=gdboutputbuf.idx;
     c:=gdboutputbuf.buf[signal_end];
     gdboutputbuf.buf[signal_end]:=#0;
     if assigned(signal_string) then
       strdispose(signal_string);
     signal_string:=strnew(gdboutputbuf.buf+signal_start);
     gdboutputbuf.buf[signal_end]:=c;
     c:=gdboutputbuf.buf[signal_name_end];
     gdboutputbuf.buf[signal_name_end]:=#0;
     if assigned(signal_name) then
       strdispose(signal_name);
     signal_name:=strnew(gdboutputbuf.buf+signal_name_start);
     gdboutputbuf.buf[signal_name_end]:=c;
     if (user_screen_shown) then
       begin
         DebuggerScreen;
         DoUserSignal;
         UserScreen;
       end
     else
       DoUserSignal;
     call_reset:=true;
     signaled:=false;
   end;
end;


procedure annotate_signal;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|signal|');
{$endif}
  with curr_gdb^ do
   signaled:=true;
end;


procedure annotate_exited(exitstatus:longint);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|exited|');
{$endif}
{#ifdef __DJGPP__
 /* this is very important. The exit code of a djgpp program
   disables interrupts and after this there is no other interrupt
   called, which enables interrupts with the iret. */
  __dpmi_get_and_enable_virtual_interrupt_state();
#endif }
{$ifdef go32v2}
   {$asmmode att}
     asm
        movw $0x901,%ax
        int  $0x31
     end;
   {$asmmode default}
   reload_fs;
{$endif def go32v2}

  curr_gdb^.DebuggerScreen;
{  DeleteBreakPoints; }
  curr_gdb^.EndSession(exitstatus);
end;


procedure annotate_error;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|error|');
{$endif}
end;


procedure annotate_error_begin;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|error begin|');
{$endif}
  with curr_gdb^ do
   begin
     error_start:=gdboutputbuf.idx+strlen(gdboutputbuf.buf);
     got_error:=true;
   end;
{$ifdef Verbose}
  Debug('|end of error begin|');
{$endif}
end;


procedure annotate_starting;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|starting|');
{$endif}
{$ifdef go32v2}
     reload_fs;
{$endif go32v2}
  curr_gdb^.UserScreen;
end;


procedure annotate_stopped;cdecl;public;
var
  sym : symtab_and_line;
  fname : pchar;
begin
{$ifdef Verbose}
  Debug('|stopped|');
{$endif}
  with curr_gdb^ do
   begin
{$ifdef go32v2}
     reload_fs;
{$endif go32v2}
     DebuggerScreen;
     current_pc:=stop_pc;
     Debuggee_started:=inferior_pid<>0;
     if not Debuggee_started then exit;
     if reset_command then exit;
      sym:=find_pc_line(stop_pc,0);
     if assigned(sym.symtab) then
      fname:=sym.symtab^.filename
     else
      fname:=nil;
     if not SelectSourceLine(fname,sym.line,stop_breakpoint_number) then
       gdb_command('continue');
   end;
end;


function inferior_pid : longint;
begin
  inferior_pid:=inferior_ptid.pid;
end;


procedure proc_remove_foreign(pid:longint);cdecl;public;
begin
end;


procedure breakpoints_changed;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|breakpoints_changed|');
{$endif}
end;


procedure annotate_ignore_count_change;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|annotate_ignore_count_change()|');
{$endif}
end;

procedure annotate_new_thread;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|annotate_new_thread()|');
{$endif}
end;

procedure annotate_thread_changed;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|annotate_thread_changed()|');
{$endif}
end;


procedure annotate_breakpoint(num:longint);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|breakpoint(%d)|');
{$endif}
  With Curr_gdb^ do
    stop_breakpoint_number:=num;
end;

procedure annotate_breakpoints_changed;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|breakpoints_changed|');
{$endif}
end;


procedure annotate_watchpoint(num:longint);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|watchpoint(%d)|');
{$endif}
  With Curr_gdb^ do
    stop_breakpoint_number:=num;
end;

procedure annotate_catchpoint(num:longint);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|catchpoint(%d)|');
{$endif}
  With Curr_gdb^ do
    stop_breakpoint_number:=num;
end;


procedure annotate_breakpoints_headers;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|breakpoints_headers|');
{$endif}
end;


procedure annotate_breakpoints_table;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|breakpoints_table|');
{$endif}
end;


procedure annotate_record;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|record|');
{$endif}
end;


procedure annotate_breakpoints_table_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|breakpoints_table_end|');
{$endif}
end;


procedure annotate_frames_invalid;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frames_invalid|');
{$endif}
end;


procedure annotate_frame_begin(level:longint;
{$ifdef GDB_ANNOTATE_FRAME_BEGIN_HAS_GDBARCH_FIELD}
  gdbarch : pgdbarch;
{$endif GDB_ANNOTATE_FRAME_BEGIN_HAS_GDBARCH_FIELD}
pc:CORE_ADDR);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_begin(%d,%ld)|');
{$endif}
  with curr_gdb^ do
   begin
     frame_begin_seen:=true;
     frame_level:=level;
     current_address:=pc;
     current_line_number:=-1;
     function_start:=-1;
     function_end:=-1;
     args_start:=-1;
     args_end:=-1;
     file_start:=-1;
     file_end:=-1;
     line_start:=-1;
     line_end:=-1;
   end;
end;


procedure annotate_frame_address;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_address|');
{$endif}
end;


procedure annotate_frame_address_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_address_end|');
{$endif}
end;

procedure annotate_frame_function_name;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_function_name|');
{$endif}
  with curr_gdb^ do
   function_start:=gdboutputbuf.idx;
end;


procedure annotate_frame_args;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_args|');
{$endif}
  with curr_gdb^ do
   begin
     function_end:=gdboutputbuf.idx;
     args_start:=gdboutputbuf.idx;
   end;
end;

procedure annotate_frame_source_begin;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_source_begin|');
{$endif}
  with curr_gdb^ do
   args_end:=gdboutputbuf.idx;
end;


procedure annotate_frame_source_file;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_source_file|');
{$endif}
  with curr_gdb^ do
   file_start:=gdboutputbuf.idx;
end;

procedure annotate_frame_source_file_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_source_file_end|');
{$endif}
  with curr_gdb^ do
   file_end:=gdboutputbuf.idx;
end;


procedure annotate_frame_source_line;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_source_line|');
{$endif}
  with curr_gdb^ do
   line_start:=gdboutputbuf.idx;
end;


procedure annotate_frame_source_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_source_end|');
{$endif}
  with curr_gdb^ do
   line_end:=gdboutputbuf.idx;
end;


procedure annotate_frame_where;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|frame_where|');
{$endif}
end;


procedure annotate_frame_end;cdecl;public;
var
  fe : pframeentry;
  c  : char;
  err : integer;
begin
{$ifdef Verbose}
  Debug('|frame_end|');
{$endif}
  with curr_gdb^ do
   begin
     if (not record_frames) or (not frame_begin_seen) then
      exit;
     { This can happen, when the function has no Debugging information }
     if (args_start >= 0) and (args_end < 0) then
      args_end:=gdboutputbuf.idx;
     frame_begin_seen:=false;
     fe:=get_frameentry(frame_level);
     fe^.address:=current_address;
     fe^.level:=frame_level;
     if (function_start>=0) then
      begin
        c:=gdboutputbuf.buf[function_end];
        gdboutputbuf.buf[function_end]:=#0;
        fe^.function_name:=strnew(gdboutputbuf.buf+function_start);
        gdboutputbuf.buf[function_end]:=c;
      end;
     if (file_start>=0)  then
      begin
        c:=gdboutputbuf.buf[file_end];
        gdboutputbuf.buf[file_end]:=#0;
        fe^.file_name:=strnew(gdboutputbuf.buf+file_start);
        gdboutputbuf.buf[file_end]:=c;
      end;
     if (args_start>=0) then
      begin
        {$warning FIXME}  {sometimes the ide crashes here because ars_end is 0, AD}
        if args_end > 0 then
        begin
          if (gdboutputbuf.buf[args_end-1]=#10) then
           dec(args_end);
          { Flushing is not always correct for args,
            try to move on to next closing brace }
          while (args_end<file_start) and (gdboutputbuf.buf[args_end-1]<>')') do
            inc(args_end);
          c:=gdboutputbuf.buf[args_end];
          gdboutputbuf.buf[args_end]:=#0;
          fe^.args:=strnew(gdboutputbuf.buf+args_start);
          gdboutputbuf.buf[args_end]:=c;
        end;
      end;
     if (line_start>=0) then
      begin
        c:=gdboutputbuf.buf[line_end];
        gdboutputbuf.buf[line_end]:=#0;
{     sscanf(gdb_output_buffer+line_start,'%d',&fe^.line_number); }
        val(strpas(pchar(@gdboutputbuf.buf[line_start])),fe^.line_number,err);
        gdboutputbuf.buf[line_end]:=c;
      end;
   end;
end;


procedure annotate_quit;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|quit|');
{$endif}
end;


procedure annotate_arg_begin;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|arg_begin|');
{$endif}
end;


procedure annotate_arg_name_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|arg_name_end|');
{$endif}
end;


procedure annotate_arg_value(typ:pointer);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|arg_value|');
{$endif}
end;


procedure annotate_arg_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|arg_end|');
{$endif}
end;

procedure annotate_source(filename:pchar;line,character,mid:longint;
{$ifdef GDB_ANNOTATE_FRAME_BEGIN_HAS_GDBARCH_FIELD}
  gdbarch : pgdbarch;
{$endif GDB_ANNOTATE_FRAME_BEGIN_HAS_GDBARCH_FIELD}
pc:CORE_ADDR);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|source|');
{$endif}
end;


procedure annotate_function_call;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|function_call|');
{$endif}
end;


procedure annotate_signal_handler_caller;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|signal_handler_caller|');
{$endif}
end;


procedure annotate_array_section_begin(index:longint;elttype:pointer);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|array_section_begin()|');
{$endif}
end;


procedure annotate_elt_rep(repcount:longint);cdecl;public;
begin
{$ifdef Verbose}
  Debug('|elt_rep()|');
{$endif}
end;

procedure annotate_elt_rep_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|elt_rep_end|');
{$endif}
end;


procedure annotate_elt;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|elt|');
{$endif}
end;


procedure annotate_array_section_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|array_section_end|');
{$endif}
end;

procedure annotate_display_prompt;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|display_prompt|');
{$endif}
end;


procedure annotate_display_begin;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|display_begin|');
{$endif}
end;


procedure annotate_display_number_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|display_number_end|');
{$endif}
end;


procedure annotate_display_format;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|display_format|');
{$endif}
end;

procedure annotate_display_expression;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|display_expression|');
{$endif}
end;


procedure annotate_display_expression_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|display_expression_end|');
{$endif}
end;


procedure annotate_display_value;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|display_value|');
{$endif}
end;


procedure annotate_display_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('|display_end|');
{$endif}
end;


procedure annotate_field (num:longint);cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_field(%d)');
{$endif}
end;


procedure annotate_field_begin(typ:pointer);cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_field_begin\n');
{$endif}
end;


procedure annotate_field_name_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_field_name_end\n');
{$endif}
end;


procedure annotate_field_value;cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_field_value\n');
{$endif}
end;


procedure annotate_field_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_field_end\n');
{$endif}
end;


procedure annotate_value_history_begin (histindex:longint;typ:pointer);cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_value_history_begin(%d)\n');
{$endif}
end;


procedure annotate_value_begin (typ:pointer);cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_value_begin\n');
{$endif}
end;


procedure annotate_value_history_value;cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_value_history_value\n');
{$endif}
end;


procedure annotate_value_history_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_value_history_end\n');
{$endif}
end;


procedure annotate_value_end;cdecl;public;
begin
{$ifdef Verbose}
  Debug('a_value_end\n');
{$endif}
end;


procedure _initialize_annotate;cdecl;public;
begin
end;


procedure gdbint_ui_file_write(stream : pui_file; p : pchar; len : longint);cdecl;
begin
  if assigned(curr_gdb) then
   with curr_gdb^ do
    if stream = gdb_stderr then
       gdberrorbuf.lappend(p,len)
    else if stream = gdb_stdout then
       gdboutputbuf.lappend(p,len)
    else
      begin
       gdberrorbuf.append('Unknown gdb ui_file');
       gdberrorbuf.lappend(p,len);
      end;
end;


function QueryHook(question : pchar; arg : ppchar) : longint; cdecl;
var local : pchar;

begin
  if not assigned(curr_gdb) then
    QueryHook:=0
  else
    begin
      if curr_gdb^.reset_command and ((pos('Kill',question)>0) or
         (pos('Discard symbol table',question)>0)) then
        QueryHook:=1
      else if pos('%',question)>0 then
        begin
{$ifdef GDB_USE_XSTRVPRINTF}
          local:=xstrvprintf(question,arg);
{$else}
          xvasprintf(@local,question,arg);
{$endif}
          { xvasprintf can failed, in that case local is set to nil }
          if not assigned(local) then
            local:=question;
          QueryHook:=curr_gdb^.Query(local, nil);
          xfree(local);
        end
      else
        QueryHook:=curr_gdb^.Query(question, nil);
    end;
end;

procedure CreateBreakPointHook(var b:breakpoint);cdecl;
var
  sym : symtab_and_line;

{ this procedure is only here to avoid the problems
  with different version of gcc having different stack
  handling:
  on older versions find_pc_line uses just "ret"
  while on newer gcc version "ret $4" is used
  if this call is within the CreateBreakPointHook function
  it changes %esp and thus the registers are
  not restored correctly PM }
  procedure get_pc_line;
    begin

{$ifdef GDB_USES_BP_LOCATION}
      if assigned (b.loc) then
        sym:=find_pc_line(b.loc^.address,0)
{$else not GDB_USES_BP_LOCATION}
      if (b.address <> 0) then
        sym:=find_pc_line(b.address,0)
{$endif not GDB_USES_BP_LOCATION}
      else
        fillchar (sym, sizeof(sym), #0);
    end;
begin
  get_pc_line;
  with curr_gdb^ do
   begin
     last_breakpoint_number:=b.number;
     { function breakpoints have zero as file and as line !!
       but they are valid !! }
{$ifndef GDB_USES_BP_OPS}
     invalid_breakpoint_line:=(b.line_number<>sym.line) and (b.line_number<>0);
{$else GDB_USES_BP_OPS}
     invalid_breakpoint_line:=(b.loc=nil) or
       ((b.loc^.line_number<>sym.line) and (b.loc^.line_number<>0));
{$endif GDB_USES_BP_OPS}
{$ifdef GDB_USES_BP_LOCATION}
     if assigned (b.loc) then
       last_breakpoint_address:=b.loc^.address
     else
       last_breakpoint_address:=0;
{$else not GDB_USES_BP_LOCATION}
     last_breakpoint_address:=b.address;
{$endif not GDB_USES_BP_LOCATION}
     last_breakpoint_line:=sym.line;
     if assigned(sym.symtab) then
      last_breakpoint_file:=sym.symtab^.filename
     else
      last_breakpoint_file:=nil;
   end;
end;

{$ifdef GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}

type
{$ifdef GDB_NOTIFY_BREAKPOINT_ARG_IS_BREAKPOINT_PTR}
  breakpoint_created_function_type = procedure (bpp : pbreakpoint); cdecl;
{$else not GDB_NOTIFY_BREAKPOINT_ARG_IS_BREAKPOINT_PTR}
  breakpoint_created_function_type = procedure (bpnum : longint); cdecl;
{$endif not GDB_NOTIFY_BREAKPOINT_ARG_IS_BREAKPOINT_PTR}
  pobserver = pointer;
var
  breakpoint_created_observer : pobserver = nil;

function observer_attach_breakpoint_created(create_func : breakpoint_created_function_type) : pobserver;cdecl;external;
procedure observer_detach_breakpoint_created(pob : pobserver);cdecl;external;


{$ifdef GDB_NOTIFY_BREAKPOINT_ARG_IS_BREAKPOINT_PTR}
procedure notify_breakpoint_created(bpp : pbreakpoint); cdecl;
begin
  CreateBreakpointHook(bpp^);
end;
{$else not GDB_NOTIFY_BREAKPOINT_ARG_IS_BREAKPOINT_PTR}
var breakpoint_chain : pbreakpoint ;cvar;external;

procedure notify_breakpoint_created(bpnum : longint);cdecl;
var
  pb : pbreakpoint;
begin
  pb:=breakpoint_chain;
  while assigned(pb) do
    begin
      if pb^.number=bpnum then
        begin
          CreateBreakPointHook(pb^);
          exit;
        end
      else
        pb:=pb^.next;
    end;
end;
{$endif not GDB_NOTIFY_BREAKPOINT_ARG_IS_BREAKPOINT_PTR}
{$endif def GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}

{ Avoid loading of main.o object by providing a
  stripped down version of relocate_gdb_directory function }
function relocate_gdb_directory(path : pchar) : pchar; cdecl; public;
begin
  relocate_gdb_directory:=path;
end;

{*****************************************************************************
                                 tgdbinterface
*****************************************************************************}

constructor tgdbinterface.init;
begin
  gdboutputbuf.init;
  gdberrorbuf.init;
  record_frames:=true;

  { This must be placed before gdb__init is called
    as gdb_init might issue output PM }
  curr_gdb:=@self;
  gdb__init;
  command_level:=0;
{ set output mode for GDB }
{ only these values disable filtering
  DONT CHANGE THEM !!! PM }
  gdb_command('set width 0xffffffff');
  gdb_command('set height 0xffffffff');
{ other standard commands used for fpc debugging }
  gdb_command('set print demangle off');
  gdb_command('set gnutarget auto');
  gdb_command('set language auto');
  gdb_command('set print vtbl on');
  gdb_command('set print object on');
  gdb_command('set print null-stop');
  {$ifdef USE_MINGW_GDB}  // maybe this also should be done for newer cygwin gdbs.
  //gdb_command('set confirm off');
  {$endif}
end;


destructor tgdbinterface.done;
begin
  clear_frames;
  gdb_done;
  gdboutputbuf.done;
  gdberrorbuf.done;
end;


procedure tgdbinterface.gdb__init;
begin
  gdboutputbuf.reset;
  gdberrorbuf.reset;
  {$ifdef GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
    breakpoint_created_observer:=observer_attach_breakpoint_created(@notify_breakpoint_created);
  {$else not GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
    {$ifdef GDB_HAS_DEPRECATED_CBPH}
    deprecated_create_breakpoint_hook:=@CreateBreakPointHook;
    {$else}
    create_breakpoint_hook:=@CreateBreakPointHook;
    {$endif}
  {$endif}
  deprecated_query_hook :=@QueryHook;

  signal_string:=nil;
  signal_name:=nil;
end;



procedure tgdbinterface.gdb_done;
begin
  if debuggee_started then
    begin
{$ifdef GDB_TARGET_CLOSE_HAS_PTARGET_ARG}
      target_kill;
      target_close(@current_target,1);
{$else not GDB_TARGET_CLOSE_HAS_PTARGET_ARG}
      current_target.to_kill;
      target_close(1);
{$endif ndef GDB_TARGET_CLOSE_HAS_PTARGET_ARG}
    end;
  {$ifdef GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
    observer_detach_breakpoint_created(breakpoint_created_observer);
    breakpoint_created_observer:=nil;
  {$else not GDB_HAS_OBSERVER_NOTIFY_BREAKPOINT_CREATED}
    {$ifdef GDB_HAS_DEPRECATED_CBPH}
    deprecated_create_breakpoint_hook:=nil;
    {$else}
    create_breakpoint_hook:=nil;
    {$endif}
  {$endif}
end;

procedure tgdbinterface.FlushAll;
begin
end;

function tgdbinterface.Query(question : pchar; args : pchar) : longint;
begin
  Query:=0;
end;

function tgdbinterface.error:boolean;
begin
  error:=got_error;
end;

function tgdbinterface.error_num:longint;
begin
  error_num:=gdb_error;
end;

var
   top_level_val : longint;

{$ifdef USE_CATCH_EXCEPTIONS}
function catch_exceptions(uiout : ui_out; func : pointer; command : pchar; mask : longint) : longint;cdecl;external;

function gdbint_execute_command(uiout : ui_out; command : pchar) : longint;cdecl;
begin
  gdbint_execute_command:=1;
  execute_command(command,1);
  gdbint_execute_command:=0;
end;
{$else not USE_CATCH_EXCEPTIONS}
function catch_command_errors(func : pointer; command : pchar; from_tty,mask : longint) : longint;cdecl;external;

function gdbint_execute_command(command : pchar; from_tty : longint) : longint;cdecl;
begin
  gdbint_execute_command:=1;
  execute_command(command,from_tty);
  gdbint_execute_command:=0;
end;
{$endif not USE_CATCH_EXCEPTIONS}

{$ifdef cpui386}
type
  tfpustate = word;

const
  MaskAllExceptions = $ff;
{$else}
type
  tfpustate = longint;
const
  MaskAllExceptions = 0;
{$endif}

procedure SaveFPUState(var control :TFPUState);
begin
{$ifdef cpui386}
  asm
    movl control, %edi
    fstcw (%edi)
  end;
{$else}
  control:=0;
{$endif}
end;

procedure SetFPUState(control : TFPUState);
begin
{$ifdef cpui386}
  asm
    fnclex
    fldcw control
  end;
{$else}
{$endif}
end;

function MaskAllFPUExceptions(control : TFPUState) : TFPUState;
begin
{$ifdef cpui386}
  MaskAllFPUExceptions := control or MaskAllExceptions;
{$else}
  MaskAllFPUExceptions:=0;
{$endif}
end;

procedure tgdbinterface.gdb_command(const s:string);
var
  command          : array[0..256] of char;
  prev_stop_breakpoint_number,
  mask : longint;
  s2 : string;
  old_quit_return,
  old_error_return : jmp_buf;
  control : TFPUState;
begin
  inc(command_level);
  SaveFPUState(control);
  SetFPUState(MaskAllFPUExceptions(control));
  move(s[1],command,length(s));
  command[length(s)]:=#0;
  old_quit_return:=quit_return;
  old_error_return:=error_return;
  gdb_error:=0;
  got_error:=false;
  if command_level=1 then
    prev_stop_breakpoint_number:=0
  else
    prev_stop_breakpoint_number:=stop_breakpoint_number;

  stop_breakpoint_number:=0;
  { Trap quit commands }
  s2:=s;
  while (length(s2)>0) and ((s2[1]=' ') or (s2[1]=#9)) do
    s2:=copy(s2,2,255);
  if (length(s2)>0) and
     (UpCase(s2[1])='Q') and
     ((length(s2)=1) or
      (s2[2]=' ') or
      ((UpCase(s2[2])='U') and
      ((length(s2)=2) or
       (s2[3]=' ') or
       ((UpCase(s2[3])='I') and
        ((length(s2)=3) or
         (s2[4]=' ') or
         ((UpCase(s2[4])='T') and
          ((length(s2)=4) or
           (s2[5]=' ')
     ))))))) then
    begin
      if not AllowQuit then
        exit;
    end;
{$ifdef DebugCommand}
  Debug('start of handle_gdb_command ('+s+')');
{$endif}
  top_level_val:=setjmp(error_return);
  if top_level_val=0 then
   begin
     quit_return:=error_return;
     mask:=longint($ffffffff);
{$ifdef USE_CATCH_EXCEPTIONS}
     catch_exceptions(our_uiout, @gdbint_execute_command,@command,mask);
{$else i.e. not USE_CATCH_EXCEPTIONS}
     catch_command_errors(@gdbint_execute_command,@command,
       1,mask);
{$endif not def USE_CATCH_EXCEPTIONS}
{$ifdef go32v2}
     reload_fs;
{$endif go32v2}
   end
  else
{$ifdef Verbose}
    Debug('error longjmp in handle_gdb_command ('+s+')');
{$endif}
   ;
{$ifdef DebugCommand}
  Debug('end of handle_gdb_command ('+s+')');
{$endif}
  quit_return:=old_quit_return;
  error_return:=old_error_return;
  dec(command_level);
  stop_breakpoint_number:=prev_stop_breakpoint_number;
  SetFPUState(control);
end;


procedure tgdbinterface.resize_frames;
var
  i : longint;
  new_frames : ppframeentry;
begin
  if (frame_count>=frame_size) then
   begin
     getmem(new_frames,sizeof(pointer)*(frame_count+1));
     for i:=0 to frame_size-1 do
       new_frames[i]:=frames[i];
     if assigned(frames) then
       freemem(frames,sizeof(pointer)*frame_size);
     frames:=new_frames;
     frame_size:=frame_count+1;
     for i:=frame_count to frame_size-1 do
      frames[i]:=new(pframeentry,init);
  end;
end;


function tgdbinterface.add_frameentry:pframeentry;
begin
  resize_frames;
  add_frameentry:=frames[frame_count];
  inc(frame_count);
end;

function tgdbinterface.get_frameentry(level : longint) : pframeentry;
begin
  { only climb values one by one PM }
  if level>=frame_count then
    resize_frames;
  get_frameentry:=frames[level];
  frames[level]^.clear;
  if level>=frame_count then
    inc(frame_count);
end;


procedure tgdbinterface.clear_frames;
var
  i : longint;
begin
  for i:=0 to frame_size-1 do
   dispose(frames[i],done);
  if assigned(frames) then
    begin
      freemem(frames,sizeof(pointer)*Frame_size);
      frames:=nil;
    end;
  frame_count:=0;
  frame_size:=0;
end;

function tgdbinterface.get_current_frame : ptrint;
begin
  record_frames:=false;
  gdb_command('f');
  get_current_frame:=frame_level;
  record_frames:=true;
end;

function tgdbinterface.set_current_frame(level : longint) : boolean;
var
  s : string;
begin
  record_frames:=false;
  str(level,s);
  gdb_command('f '+s);
  if level=frame_level then
    set_current_frame:=true
  else
    set_current_frame:=false;
  record_frames:=true;
end;


{*****************************************************************************
                      Highlevel tgdbinterface
*****************************************************************************}

procedure tgdbinterface.GetAddrSyminfo(addr:ptrint;var si:tsyminfo);
var
  sym : symtab_and_line;
  symbol : psymbol;
begin
  sym:=find_pc_line(addr,1);
  fillchar(si,sizeof(tsyminfo),0);
  si.address:=addr;
  si.offset:=addr-sym.pc;
  if assigned(sym.symtab) then
   si.fname:=sym.symtab^.filename
  else
   si.fname:=nil;
  si.line:=sym.line;
  symbol:=find_pc_function(addr);
  if assigned(symbol) then
   si.funcname:=symbol^.ginfo._name
  else
   si.funcname:=nil;
end;


function tgdbinterface.SelectSourceLine(fn:pchar;line,BreakIndex:longint): Boolean;
begin
  if assigned(fn) then
    SelectSourceLine:=DoSelectSourceLine(StrPas(fn),line,BreakIndex)
  else
    SelectSourceLine:=DoSelectSourceLine('',line,BreakIndex);
end;


procedure tgdbinterface.StartSession;
begin
  DoStartSession;
end;


procedure tgdbinterface.BreakSession;
begin
  DoBreakSession;
end;


procedure tgdbinterface.EndSession(code:longint);
begin
  Debuggee_started:=false;
  { inferior_ptid.pid:=0;
    This leads to an assertion failure
    from generic_mount_inferior }
  DoEndSession(code);
  if assigned(signal_name) then
    strdispose(signal_name);
  signal_name:=nil;
  if assigned(signal_string) then
    strdispose(signal_string);
  signal_string:=nil;
end;


procedure tgdbinterface.DebuggerScreen;
begin
{$ifdef Verbose}
  Debug('|DebuggerScreen|');
{$endif}
  if user_screen_shown then
   DoDebuggerScreen;
  user_screen_shown:=false;
end;


procedure tgdbinterface.UserScreen;
begin
{$ifdef Verbose}
  Debug('|UserScreen|');
{$endif}
  if switch_to_user then
   begin
     if (not user_screen_shown) then
      DoUserScreen;
     user_screen_shown:=true;
   end;
end;



{---------------------------------------
          Default Hooks
---------------------------------------}

function tgdbinterface.DoSelectSourceLine(const fn:string;line,BreakIndex:longint): Boolean;
{$ifdef Verbose}
var
  s,bs : string;
{$endif}
begin
{$ifdef Verbose}
  Str(line,S);
  Str(BreakIndex,BS);
  Debug('|SelectSource '+fn+':'+s+','+bs+'|');
{$endif}
end;

procedure tgdbinterface.DoStartSession;
begin
end;

procedure tgdbinterface.DoBreakSession;
begin
end;

procedure tgdbinterface.DoEndSession(code:longint);
begin
end;

procedure tgdbinterface.DoUserSignal;
begin
end;

procedure tgdbinterface.DoDebuggerScreen;
begin
end;

procedure tgdbinterface.DoUserScreen;
begin
end;

function  tgdbinterface.AllowQuit : boolean;
begin
  AllowQuit:=true;
end;

var
  version : array[0..0] of char;cvar;external;

{$ifndef GDB_NEEDS_NO_ERROR_INIT}
{ doesn't seem to exist anymore. Seems to work fine without }
procedure error_init;cdecl;external;
{$endif GDB_NEEDS_NO_ERROR_INIT}

function  GDBVersion : string;
begin
  GDBVersion:='GDB '+StrPas(version);
end;


const next_exit : pointer = nil;
procedure DoneLibGDB;
begin
  exitproc:=next_exit;
end;

{$ifdef go32v2}
var
  c_environ : ppchar;external name '__environ';
  c_argc : longint;external name '___crt0_argc';
  c_argv : ppchar;external name '___crt0_argv';

  procedure ReallocateEnvironUsingCMalloc;

  var
    neededsize , i, count : longint;
    penv : pchar;
    newenv : ppchar;
  begin
    if not assigned(c_environ) then
      neededsize:=sizeof(pchar)
    else
      begin
        count:=0;
        penv:=c_environ[count];
        while assigned(penv) do
          begin
            inc(count);
            penv:=c_environ[count];
          end;
        inc(count);
        neededsize:=count*sizeof(pchar);
      end;
    newenv:=malloc(neededsize);
    system.move(c_environ^,newenv^,neededsize);
    if assigned(c_environ) then
      begin
        for i:=0 to count-1 do
          begin
            penv:=c_environ[i];
            if assigned(penv) then
              begin
                neededsize:=strlen(penv)+1;
                newenv[i]:=malloc(neededsize);
                system.move(penv^,newenv[i]^,neededsize);
              end
            else
              newenv[i]:=nil;
          end;
      end;
    c_environ:=newenv;
  end;

{$endif def go32v2}
var
  current_directory : pchar; cvar; external;
  gdb_dirbuf : array[0..0] of char; cvar; external;
  CurrentDir : AnsiString;
{$ifdef GDB_NEEDS_INTERPRETER_SETUP}
  type
     interpreter_struct_p = pointer; { to opaque type }
  function interp_lookup ({$ifdef GDB_INTERP_LOOKUP_HAS_UI} ui :pui ;{$endif} name : pchar) : interpreter_struct_p;cdecl; external;
  function interp_set (interp : interpreter_struct_p;top_level : cint) : longbool;cdecl; external;
{$endif GDB_NEEDS_INTERPRETER_SETUP}
const
  DIRBUF_SIZE = 1024;

procedure InitLibGDB;
{$ifdef supportexceptions}
var
  OldSigInt : SignalHandler;
{$endif supportexceptions}
{$ifdef GDB_NEEDS_SET_INSTREAM}
var
  dummy_file : pui_file;
{$endif GDB_NEEDS_SET_INSTREAM}

{$ifdef GDB_INIT_HAS_ARGV0}
var
  argv0 : pchar;
{$endif not GDB_INIT_HAS_ARGV0}
{$ifdef GDB_NEEDS_INTERPRETER_SETUP}
var
  interp : interpreter_struct_p;
{$endif GDB_NEEDS_INTERPRETER_SETUP}
var
 save_gdb_stdin,
 save_gdb_stdout,
 save_gdb_stderr : pui_file;
begin
{$ifdef go32v2}
  { c_environ:=system.envp; }
  { DJGPP libC presupposes the c_enivron was malloc'ated }
  ReallocateEnvironUsingCMalloc;
  c_argc:=system.argc;
  c_argv:=system.argv;
{$endif def go32v2}
{$ifdef supportexceptions}
{$ifdef go32v2}
  OldSigInt:=Signal(SIGINT,SignalHandler(@SIG_DFL));
{$else}
  {$ifdef Unix}
    OldSigInt:=fpSignal(SIGINT,SignalHandler(SIG_DFL));
  {$else}
    OldSigInt:=Signal(SIGINT,SignalHandler(SIG_DFL));
  {$endif}
{$endif}
{$endif supportexceptions}

  if assigned(gdb_stderr) then
    ui_file_delete(gdb_stderr);
  if assigned(gdb_stdout) then
    ui_file_delete(gdb_stdout);
{$ifdef GDB_NEEDS_SET_INSTREAM}
  if assigned(gdb_stdin) then
    ui_file_delete(gdb_stdin);
  gdb_stdin:=mem_fileopen;
  save_gdb_stdin:=gdb_stdin;
{$ifdef LIBGDB_HAS_GET_STDIN}
{$ifndef GDB_NO_INSTREAM_VAR}
  instream:=gdb_get_stdin;
{$endif ndef GDB_NO_INSTREAM_VAR}
{$ifdef GDB_SET_SAVED_COMMAND_LINE}
  saved_command_line:=xmalloc(saved_command_line_size);
{$endif def GDB_SET_SAVED_COMMAND_LINE}
{$else}
  dummy_file :=gdb_fopen('dummy.$$$','a');
  {in captured_main code, this is simply
   instream:=stdin; but stdin is a highly system dependent macro
   so that we try to avoid it here }
  if assigned(dummy_file) then
    instream:=pstdio_file(dummy_file^.to_data)^._file
  else
    instream:=nil;
{$endif}
{$endif GDB_NEEDS_SET_INSTREAM}

{$ifdef LIBGDB_HAS_GET_STDIN}
  gdb_stderr:=stdio_fileopen(gdb_get_stderr);
  gdb_stdout:=stdio_fileopen(gdb_get_stdout);
{$else}
  gdb_stderr:=mem_fileopen;
  gdb_stdout:=mem_fileopen;
{$endif}
  save_gdb_stderr:=gdb_stderr;
  save_gdb_stdout:=gdb_stdout;
  gdb_stdlog:=gdb_stderr;
  gdb_stdtarg:=gdb_stderr;
  set_ui_file_write(gdb_stdout,@gdbint_ui_file_write);
  set_ui_file_write(gdb_stderr,@gdbint_ui_file_write);
{$ifndef GDB_NEEDS_NO_ERROR_INIT}
  error_init;
{$endif GDB_NEEDS_NO_ERROR_INIT}
{$ifdef GDB_V6}
{$ifdef GDB_NEEDS_SET_INSTREAM}
  gdb_stdtargin := gdb_stdin;
{$endif GDB_NEEDS_SET_INSTREAM}
  gdb_stdtargerr := gdb_stderr;
{$endif}
  GetDir(0, CurrentDir);
  if length(CurrentDir)<DIRBUF_SIZE then
    strpcopy(@gdb_dirbuf,CurrentDir)
  else
    gdb_dirbuf[0]:=#0;
  current_directory:=@gdb_dirbuf[0];
  next_exit:=exitproc;
  exitproc:=@DoneLibGDB;
{$ifdef GDB_V6}
{$ifndef GDB_NO_UIOUT}
  uiout := cli_out_new (gdb_stdout);
{$endif not GDB_NO_UIOUT}
{$endif GDB_V6}
{$ifdef GDB_NEW_UI}
  local_ui := new_ui (gdb_stdin,gdb_stdout,gdb_stderr);
  main_ui:=local_ui;
  current_ui:=main_ui;
{$endif not GDB_NEW_UI}
{$ifdef GDB_INIT_HAS_ARGV0}
  getmem(argv0,length(paramstr(0))+1);
  strpcopy(argv0,paramstr(0));
  gdb_init(argv0);
  freemem(argv0,length(paramstr(0))+1);
{$else not GDB_INIT_HAS_ARGV0}
  gdb_init;
{$endif not GDB_INIT_HAS_ARGV0}
{$ifdef GDB_NEEDS_INTERPRETER_SETUP}
  { interpreter can only be set after all files are
    initialized, which is done in gdb_init function. }
  interp := interp_lookup ({$ifdef GDB_INTERP_LOOKUP_HAS_UI}current_ui,{$endif}'console');
  if not interp_set (interp,1) then
    writeln(stderr,'Failed to set GDB console interpreter');
  { We need to re-set gdb_stdXX ui_files }
  if assigned(gdb_stderr) then
    ui_file_delete(gdb_stderr);
  if assigned(gdb_stdout) then
    ui_file_delete(gdb_stdout);
  if assigned(gdb_stdin) then
    ui_file_delete(gdb_stdin);
  gdb_stdin:=save_gdb_stdin;
  gdb_stderr:=save_gdb_stderr;
  gdb_stdout:=save_gdb_stdout;
  gdb_stdlog:=gdb_stderr;
  gdb_stdtarg:=gdb_stderr;
  set_ui_file_write(gdb_stdout,@gdbint_ui_file_write);
  set_ui_file_write(gdb_stderr,@gdbint_ui_file_write);
{$ifdef GDB_NO_UIOUT}
  cli_uiout := cli_out_new (gdb_stdout);
  current_uiout:=cli_uiout;
  our_uiout:=cli_uiout;
{$endif GDB_NO_UIOUT}
{$ifdef GDB_NEW_UI}
  local_ui := new_ui (gdb_stdin,gdb_stdout,gdb_stderr);
  main_ui:=local_ui;
  current_ui:=main_ui;
{$endif not GDB_NEW_UI}
{$endif GDB_NEEDS_INTERPRETER_SETUP}
{$ifdef supportexceptions}
  {$ifdef unix}
    fpsignal(SIGINT,OldSigInt);
  {$else}
    Signal(SIGINT,OldSigInt);
  {$endif}
{$endif supportexceptions}
  if setjmp(error_return)=0 then
    begin
       quit_return:=error_return;
       exit;
    end
  else
    begin
{$ifdef Verbose}
       Debug('|LongJump to Init|');
{$endif}
{$ifdef go32v2}
       RunError(99);
{$endif def go32v2}
    end;
  WatchDog:=0;
end;

{$ifdef GDB_HAS_SYSROOT}
  { Here we declare as cvar;public; a bunch of global
    variables that are defined in main.c source.
    We must not load main.o otherwise, we will get
    into multiply defined symbols troubles. }
var
    gdb_sysrootc : char;
    { used locally only to provide a pchar pointing to '\0' }
    gdb_sysroot  : pchar; cvar;public;
    { gdb_sysroot global variable is declared in defs.h and
      instanciated in main.c since version 6.0 }
    gdb_datadir  : pchar; cvar;public;
    { gdb_datadir global variable is declared in defs.h and
      instanciated in main.c since version 7.0 }
    python_libdir : pchar;cvar;public;
    { python_libdir global variable is declared in defs.h and instanciated
      in main.c since version 7.2 }
    return_child_result : longbool;cvar;public;
    { return_chlid_result global variable is declared in main.h and
      instanciated in main.c since version 6.4 }
    return_child_result_value : longint;cvar;public;
    { return_child_result_value global variable is declared in main.h and
      instanciated in main.c since version 6.4 with a startup value of -1 }
    batch_silent : longbool;cvar;public;
    { batch_silent global variable is declared in main.h since 7.0, but
      instanciated in main.c since version 6.4 }
    batch_flag : longbool;cvar;public;
    { batch_flag global variable is declared in main.h and
      instanciated in main.c since version 7.2 }
{$endif}
{$ifdef GDB_HAS_DEBUG_FILE_DIRECTORY}
var
  debug_file_directory : pchar; cvar; external;
{$endif GDB_HAS_DEBUG_FILE_DIRECTORY}

{$ifdef USE_LOCAL_SET_GDB_DATA_DIRECTORY}
{ Avoid loading of main.o object by providing a
  stripped down version of relocate_gdb_directory function }
procedure set_gdb_data_directory(path : pchar); cdecl; public;
begin
  gdb_datadir:=path;
end;
{$endif USE_LOCAL_SET_GDB_DATA_DIRECTORY}

begin
{$ifdef GDB_HAS_SYSROOT}
  gdb_sysrootc := #0;
  return_child_result_value := -1;
  gdb_sysroot := @gdb_sysrootc;
  gdb_datadir := @gdb_sysrootc;
  python_libdir := @gdb_sysrootc;
{$endif}
{$ifdef GDB_HAS_DEBUG_FILE_DIRECTORY}
  debug_file_directory := '/usr/local/lib';
{$endif GDB_HAS_DEBUG_FILE_DIRECTORY}
  gdb_stderr:=nil;
  gdb_stdout:=nil;
  InitLibGDB;
end.
