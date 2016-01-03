#!/usr/bin/env bash

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# Function to display help
usage ()
{
=======

if [ "$1" == "--help" ]; then
>>>>>>> graemeg/cpstrnew
=======

if [ "$1" == "--help" ]; then
>>>>>>> graemeg/cpstrnew
=======

if [ "$1" == "--help" ]; then
>>>>>>> graemeg/cpstrnew
=======

if [ "$1" == "--help" ]; then
>>>>>>> origin/cpstrnew
  echo "Script used to easily create collection of libraries needed"
  echo "to generate a Free Pascal IDE with debugger support."
  echo "Usage: Copy this script to the directory where you just compile"
  echo "a specific GNU debugger (for a specific target)"
  echo "and run ./$0 in that directory"
  echo "After, you will need to run a second script, copy-libs.sh"
  echo "with a single parameter specifying to which directory the libraries"
  echo "should be copied."
  echo "Possible parameters for this script:"
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  echo "--forcestatic, to convert all -lname into $LINKLIB libname.a"
  echo "removedir=\"space separated list of directories to remove\""
  echo "implicitlibs=\"space separated list of used system libraries\""
  echo "libdir=\"space separated list of library directories\""
}


# Try to find out which C compiler is used in Makefile
# Look for CC make variable
MAKE_CC=`sed -n "s:^[[:space:]]*CC[[:space:]]*=[[:space:]]*\(.*\):\1:p" \
		Makefile | head -1`

if [ "x$MAKE_CC" != "x" ] ; then
  echo "Found CC=\"$MAKE_CC\" in Makefile"
fi

# Try to find used make executable

# Try to find MAKE inside Makefile
MAKE_MAKE=`sed -n "s:^[[:space:]]*MAKE[[:space:]]*=[[:space:]]*\(.*\):\1:p" \
          Makefile | head -1`

if [ "x$MAKE_MAKE" != "x" ] ; then
  echo "Found MAKE=\"$MAKE_MAKE\" in Makefile"
  MAKE=$MAKE_MAKE
else
  MAKE=`which gmake 2> /dev/null`
  if [ "x${MAKE}" == "x" ] ; then
    # Assume make is OK if MAKE is not found inside Makefile
    MAKE=make
  fi
fi

# Try to find used awk executable
# Try to use AWK from Makefile
MAKE_AWK=`sed -n "s:^[[:space:]]*AWK[[:space:]]*=[[:space:]]*\(.*\):\1:p" \
  		Makefile | head -1`

if [ "x$MAKE_AWK" != "x" ] ; then
    echo "Found AWK=\"$MAKE_AWK\""
    AWK=$MAKE_AWK
else
  AWK=`which gawk 2> /dev/null`
  if [ "x$AWK" == "x" ] ; then
    #Assume awk is OK if gawk is not found
    AWK=awk
  fi
fi

# Set CC_is_gcc if GNU C compiler used
if [ "${MAKE_CC}" != "${MAKE_CC/gcc/}" ] ; then
  CC_is_gcc=1
  echo "Found compiler is gcc"
else
  CC_is_gcc=0
fi

# Possible extra option to pass when rebuilding gdb executable below
MAKEOPT=
if [ $CC_is_gcc -eq 1 ] ; then
  echo "Adding --verbose option to parse collect2 command line"
  LDFLAGS=--verbose
else
  LDFLAGS=
fi

MAKE_EXEEXT=`sed -n "s:^[[:space:]]*EXEEXT[[:space:]]*=[[:space:]]*\(.*\):\1:p" \
		Makefile | head -1`

if [ "x$MAKE_EXEEXT" != "x" ] ; then
  PATHEXT=$MAKE_EXEEXT
fi

if [ "${PATHEXT}" != "" ] ; then
  EXEEXT=.exe
  if [ "${DJDIR}" != "" ] ; then
    libdir=${DJDIR}/lib
  else
    # Do not add /lib, it is wrong, at least for msys systems
    libdir=
  fi
else
  EXEEXT=
  if [ "$libdir" == "" ]; then
    # Do not add /lib, if -print-search-dirs can be used
    if [ $CC_is_gcc -eq 1 ] ; then
      libdir=
    else
      libdir=/lib
    fi
  fi
fi

force64bitcoreaddr=0
CONFIGURE_ENABLE_64_BIT_BFD=`grep -w -- "--enable-64-bit-bfd" ./config.status`
if [ "x$CONFIGURE_ENABLE_64_BIT_BFD" != "x" ] ; then
  echo "--enable-64-bit-bfd configure option found"
  force64bitcoreaddr=1
fi

if [ "$1" == "--help" ]; then
  usage
  exit
fi


forcestatic=0

# Function to treat all command line option
handle_option ()
{
opt_handled=0
if [ "$1" == "" ] ; then
  return
fi

if [ "$1" == "--forcestatic" ]; then
  echo "Using only static libraries in gdblib.inc"
  forcestatic=1
  LDFLAGS="$LDFLAGS -static"
  opt_handled=1
fi

if [ "${1#implicitlibs=}" != "$1" ]; then
  implicitlibs=${1#implicitlibs=}
  echo "Also adding implicit libs \"$implicitlibs\""
  opt_handled=1
fi

if [ "${1#libdir=}" != "$1" ]; then
  libdir=${1#libdir=}
  echo "libdir is set to \"$libdir\""
  opt_handled=1
fi

if [ "${1#removedir=}" != "$1" ]; then
  removedir=${1#removedir=}
  echo "removedir is set to \"$removedir\""
  opt_handled=1
fi

if [ $opt_handled -eq 0 ] ; then
  if [ "${1//=/ }" != "$1" ]; then
    # Some variable set explicitly
    echo "Evaluating \"$1\""
    export "$1"
    opt_handled=1
  fi
fi
}

# Try to handle all command line options
opt_handled=1
while [ $opt_handled -eq 1 ]
do
  handle_option "$1"
  if [ $opt_handled -eq 1 ] ; then
    shift
  fi
done

if [ "$1" != "" ]; then
  echo "Unrecognized option \"$1\""
  usage
  exit
fi

if [ "x$FORCEAWK" != "x" ] ; then
  echo "Forcing use of AWK=${FORCEAWK}"
  AWK=${FORCEAWK}
fi

if [ "$OSTYPE" == "msys" ]; then
  echo "MSYS system detected"
  in_msys=1
else
  in_msys=0
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  echo "implicitlibs=\"space separated list if system librairies used\""

fi

if [ "${1#implicitlibs=}" != "$1" ]; then
  implicitlibs=${1#implicitlibs=}
  echo "Also adding implicit libs \"$implicitlibs\""
fi
if [ "${PATHEXT}" != "" ]; then
  EXEEXT=.exe
  if [ "${DJDIR}" != "" ]; then
    libdir=${DJDIR}/lib
  else
    libdir=/lib
  fi
else
  EXEEXT=
  libdir=/lib
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
fi

echo "Deleting gdb${EXEEXT} to force recompile"
rm -f gdb${EXEEXT}
echo "Rebuilding gdb${EXEEXT}"
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

${MAKE} gdb${EXEEXT} ${MAKEOPT} LDFLAGS="$LDFLAGS" 2>&1 | tee make.log


# Create gdb_get_stdin.c source file
# To avoid stdin macro expansion hell.

cat > gdb_get_stdin.c <<EOF
#include "defs.h"

/* Missing prototypes.  */

FILE * gdb_get_stdin (void);

FILE *
gdb_get_stdin (void)
{
  return stdin;
}
EOF

echo "Trying to compile gdb_get_stdin.c file"
${MAKE} gdb_get_stdin.o
res=$?

if [ $res -eq 0 ] ; then
  XM_ADD_FILES=gdb_get_stdin.o
  has_get_stdin=1
else
  has_get_stdin=0
fi


# libgdb.a will not be built automatically anymore after
# GDB release 7.4, so we need to explicitly generate it.
if [ -f libgdb.a ] ; then
  rm -f libgdb.a
fi

echo "Rebuilding GDB library to include gdb_get_stdin.o"
${MAKE} libgdb.a ${MAKEOPT} XM_ADD_FILES=${XM_ADD_FILES}

# version.c is an automatically generated file from gdb/version.in
# We extract GDB version from that file.
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
MAKE=`which gmake 2> /dev/null`

if [ "${MAKE}" == "" ]; then
  MAKE=make
fi

${MAKE} gdb${EXEEXT} | tee make.log

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
gdb_full_version=`sed -n "s:.*version.*\"\(.*\)\".*:\1:p" version.c`
gdbcvs=`sed -n "s:.*version.*\"\(.*\)cvs\(.*\)\".*:\1cvs\2:p" version.c`
gdb_version1=`sed -n "s:.*version.*\"\([0-9]*\)\.\([0-9]*\).*:\1:p" version.c`
gdb_version2=`sed -n "s:.*version.*\"\([0-9]*\)\.\([0-9]*\).*:\2:p" version.c`
gdb_version=`sed -n "s:.*version.*\"\([0-9]*\)\.\([0-9]*\).*:\1.\2:p" version.c`

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
echo "found GDB full version is ${gdb_full_version}"
echo "found GDB version is ${gdb_version}"
if [ ${gdb_version2} -lt 10 ]; then
  gdbversion=GDB_V${gdb_version1}0${gdb_version2}
else
  gdbversion=GDB_V${gdb_version1}${gdb_version2}
fi

echo "Using macro $gdbversion"

make_log_has_collect2=`grep collect2 make.log`

if [ "x$make_log_has_collect2" != "x" ] ; then
  find_cmd=collect2
else
  find_cmd=cc
fi

cat make.log | ${AWK} -v find_cmd=$find_cmd '
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
echo found full version is ${gdb_full_version}
echo found version is ${gdb_version}
if [ ${gdb_version2} -lt 10 ]; then
  gdbversion=${gdb_version1}0${gdb_version2}
else
  gdbversion=${gdb_version1}${gdb_version2}
fi

cat make.log | gawk '
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
BEGIN {
doprint=0
}
# We look for the compilation line
# either gcc or cc
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
$0 ~ find_cmd { doprint=1; }
=======
/cc / { doprint=1; }
>>>>>>> graemeg/cpstrnew
=======
/cc / { doprint=1; }
>>>>>>> graemeg/cpstrnew
=======
/cc / { doprint=1; }
>>>>>>> graemeg/cpstrnew
=======
/cc / { doprint=1; }
>>>>>>> origin/cpstrnew

{
if ( doprint == 1 ) {
  print $0
}
}

! /\\$/ { doprint=0; }
' | tee comp-cmd.log

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
if [ "x$MAKE_CC" = "x" ] ; then
  gcccompiler=`sed -n "s:\([A-Za-z0-9_-]*gcc\) .*:\1:p" comp-cmd.log`
else
  gcccompiler=$MAKE_CC
fi

if [ "$gcccompiler" != "" ]; then
  gcclibs=`$gcccompiler -print-search-dirs | sed -n "s#.*libraries: =\(.*\)#\1#p" `
  if [ "$gcclibs" != "" ]; then
    if [ $in_msys -eq 1 ]; then
      # If we are on msys, gcc is mingw, so that it uses c:/dir
      # while find is an msys utility that needs /c/dir path
      # we do this conversion below
      for let in a b c d e f g h i j k l m n o p q r s t u v w x y z; do
        gcclibs=${gcclibs//$let:/\/$let}
      done
      for let in A B C D E F G H I J K L M N O P Q R S T U V W X Y Z; do
        gcclibs=${gcclibs//$let:/\/$let}
      done
      libdir="$libdir ${gcclibs//;/ }"
    else
      # if ; is present in gcclibs,assume this is the separator instead of :
      if [ "${gcclibs//;/ }" != "${gcclibs}" ]; then
	if [ "${gcclibs// /_}" != "${gccclibs}" ]; then
          # list also contains spaces, convert ' ' into '\ '
          gcclibs=${gcclibs// /\\ }
        fi
        libdir="$libdir ${gcclibs//;/ }"
      else
        libdir="$libdir ${gcclibs//:/ }"
      fi
    fi
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
gcccompiler=`sed -n "s:\([A-Za-z0-9_-]*gcc\) .*:\1:p" comp-cmd.log`
if [ "$gcccompiler" != "" ]; then
  gcclibs=`$gcccompiler -print-search-dirs | sed -n "s#.*libraries: =\(.*\)#\1#p" `
  if [ "$gcclibs" != "" ]; then
    libdir=${gcclibs//:/ }
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
    echo "gcc libs are \"$libdir\""
  fi
fi

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
newlibdir=
for dir in $libdir ; do
  echo "Handling dir $dir"
  newdir=`cd $dir 2> /dev/null && pwd -P`
  if [ "X$newdir" != "X" ] ; then
    adddir=1
    for rdir in $removedir ; do
      if [ "$rdir" == "$newdir" ] ; then
        adddir=0
      fi
    done
    if [ $adddir -eq 1 ] ; then
      newlibdir="$newlibdir $newdir"
      echo "Adding $dir as $newdir"
    fi
  else
    echo "$dir not found"
  fi
done
libdir="$newlibdir"

# Try to locate all libraries
echo Creating ./copy-libs.sh script
has_libgdb=`cat comp-cmd.log | grep "libgdb\.a"`
if [  "x$has_libgdb" != "x" ] ; then
  add_libgdb=0
else
  add_libgdb=1
fi

cat comp-cmd.log | ${AWK} -v libdir="${libdir}" -v implibs="${implicitlibs}" \
  -v add_libgdb=${add_libgdb} '
BEGIN {
  isdynlinker=0
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
# Try to locate all libraries
echo Creating ./copy-libs.sh script
cat comp-cmd.log | gawk -v libdir="${libdir}" -v implibs="${implicitlibs}" '
BEGIN {
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
  print "#!/usr/bin/env bash"
  print "# copy-libs.sh generated by awk script"
  print "INSTALL=`which ginstall 2> /dev/null `"
  print "if [ "$INSTALL" == "" ]; then"
  print "  INSTALL=install"
  print "fi"
  print "if [ \"$1\" != \"\" ]; then"
  print "  destdir=$1"
  print "  $INSTALL  -d ${destdir}"
  print "else"
  print "  echo $0 destdir"
  print "  echo destdir should be the location where libgdb.a"
  print "  echo and all other archives should be copied"
  print "  exit"
  print "fi"
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  print "libdir=\"" libdir "\""
  print "# Copy gdblib.inc file"
  print "cp -p gdblib.inc ${destdir}"
  if (add_libgdb == 1) {
    print "# Adding libgdb.a"
    print "cp -p libgdb.a ${destdir}"
  }
=======
  print "# Copy gdblib.inc file"
  print "cp gdblib.inc ${destdir}"
>>>>>>> graemeg/cpstrnew
=======
  print "# Copy gdblib.inc file"
  print "cp gdblib.inc ${destdir}"
>>>>>>> graemeg/cpstrnew
=======
  print "# Copy gdblib.inc file"
  print "cp gdblib.inc ${destdir}"
>>>>>>> graemeg/cpstrnew
=======
  print "# Copy gdblib.inc file"
  print "cp gdblib.inc ${destdir}"
>>>>>>> origin/cpstrnew
}

{
  nb = split ($0,list);

  for (i=1; i<=nb; i++) {
    if ( list[i] ~ /lib[^ ]*\.a/ ) {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      print "# Looking for static libs"
      staticlib = gensub (/([^ ]*)(lib[^ ]*\.a)/,"\\1\\2 ","g",list[i]);
      print "cp -p " staticlib " ${destdir}";
    }
    if ( list[i] ~ /^-dynamic-linker$/ ) {
      i++;
      print "echo dynamic linker " list[i] " skipped";
      continue
    }
    if ( list[i] ~ /^-plugin$/ ) {
      i++;
      print "echo collect2 -plugin " list[i] " skipped";
      continue
=======
      staticlib = gensub (/([^ ]*)(lib[^ ]*\.a)/,"\\1\\2 ","g",list[i]);
      print "cp " staticlib " ${destdir}";
>>>>>>> graemeg/cpstrnew
=======
      staticlib = gensub (/([^ ]*)(lib[^ ]*\.a)/,"\\1\\2 ","g",list[i]);
      print "cp " staticlib " ${destdir}";
>>>>>>> graemeg/cpstrnew
=======
      staticlib = gensub (/([^ ]*)(lib[^ ]*\.a)/,"\\1\\2 ","g",list[i]);
      print "cp " staticlib " ${destdir}";
>>>>>>> graemeg/cpstrnew
=======
      staticlib = gensub (/([^ ]*)(lib[^ ]*\.a)/,"\\1\\2 ","g",list[i]);
      print "cp " staticlib " ${destdir}";
>>>>>>> origin/cpstrnew
    }
    if ( list[i] ~ /lib[^ ]*\.so/ ) {
      dynamiclib = gensub (/([^ ]*)(lib[^ ]*\.so)/,"\\1\\2 ","g",list[i]);
      print "echo " dynamiclib " found";
    }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if ( list[i] ~ /^-l/ ) {
      print "#Looking for shared libs"
      systemlib = gensub (/-l([^ ]*)/,"lib\\1.a ","g",list[i]);
      print "systemlib=`find $libdir -maxdepth 1 -iname " systemlib " -print 2> /dev/null `" ;
      print "if [ \"${systemlib}\" != \"\" ]; then";
      print "  echo System lib found: ${systemlib}";
      print "  cp -p ${systemlib%%[$IFS]*} ${destdir}";
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    if ( list[i] ~ /-l/ ) {
      systemlib = gensub (/-l([^ ]*)/,"lib\\1.a ","g",list[i]);
      print "systemlib=`find " libdir " -name " systemlib " 2> /dev/null `" ;
      print "if [ \"${systemlib}\" != \"\" ]; then";
      print "  echo System lib found: ${systemlib}";
      print "  cp ${systemlib} ${destdir}";
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
      print "else";
      print "  echo Library " systemlib " not found, shared library assumed";
      print "fi";
  }
  }
}
END {
  nb = split (implibs,list);
  for (i=1;i<=nb; i++) {
    systemlib = "lib" list[i] ".a";
    print "echo Adding system library " systemlib;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    print "systemlib=`find $libdir -maxdepth 1 -iname " systemlib " -print 2> /dev/null `" ;
    print "if [ \"${systemlib}\" != \"\" ]; then";
    print "  echo System lib found: ${systemlib}";
    print "  cp -p ${systemlib%%[$IFS]*} ${destdir}";
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    print "systemlib=`find " libdir " -name " systemlib " 2> /dev/null `" ;
    print "if [ \"${systemlib}\" != \"\" ]; then";
    print "  echo System lib found: ${systemlib}";
    print "  cp ${systemlib} ${destdir}";
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
    print "else";
    print "  echo Library " systemlib " not found, shared library assumed";
    print "fi";
  }
}
' | tee copy-libs.sh
chmod u+x ./copy-libs.sh
# For later

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# Check if mingw executable contains
# __cpu_features_init function
if [ -f gdb.exe ] ; then
  has_cpu_features_init=`objdump -t gdb.exe | grep cpu_features_init `
  if [ "X$has_cpu_features_init" == "X" ] ; then
    mingw_no_cpu_features_init=1
  else
    mingw_no_cpu_features_init=0
  fi
else
  mingw_no_cpu_features_init=0
fi

echo Creating ./gdblib.inc file
# Generate gdblib.inc file
cat comp-cmd.log |${AWK} -v gdbcvs=${gdbcvs} \
  -v implibs="${implicitlibs}" -v libdir="${libdir}" \
  -v gdbversion=${gdbversion} -v forcestatic=${forcestatic} \
  -v force64bitcoreaddr=${force64bitcoreaddr} \
  -v has_get_stdin=${has_get_stdin} \
  -v mingw_no_cpu_features_init=${mingw_no_cpu_features_init} \
  -v add_libgdb=${add_libgdb} '
BEGIN {
  use_mingw=0;
  print "{ libgdb.inc file generated by awk script }"
  print "{$define " gdbversion " }"
  if (gdbcvs) {
    print "{$define GDB_CVS}"
  }
  if (force64bitcoreaddr) {
    print "{$define GDB_CORE_ADDR_FORCE_64BITS}"
  }
  print "{$ifdef COMPILING_GDBINT_UNIT }"
  if (add_libgdb == 1) {
    print "{$LINKLIB libgdb.a} { Added here because Makefile does not use the libgdb library anymore }"
  }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
echo Creating ./gdblib.inc file
# Generate gdblib.inc file
cat comp-cmd.log |gawk -v gdbcvs=${gdbcvs} -v implibs="${implicitlibs}" \
  -v gdbversion=${gdbversion} '
BEGIN {
  use_mingw=0;
  print "{ libgdb.inc file generated by awk script }"
  print "{$define GDB_V" gdbversion " }"
  if (gdbcvs) {
    print "{$define GDB_CVS}"
  }
  print "{$ifdef COMPILING_GDBINT_UNIT }"
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
}

{
  nb = split ($0,list);

  for (i=1; i<=nb; i++) {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if ( list[i] ~ /lib[^ ]*\.a/ ) {
      staticlib = gensub (/([^ ]*)(lib[^ ]*\.a)/,"{$LINKLIB \\2} { found in \\1 }","g",list[i]);
      print staticlib;
      if ( list[i] ~ /mingw/ ) {
      use_mingw=1
      }
    }
    if ( list[i] ~ /^-dynamic-linker$/ ) {
      i++;
      print "{ Dynamic linker found " list[i] " }";
      continue
    }
    if ( list[i] ~ /^-plugin$/ ) {
      i++;
      print "{ collect2 -plugin " list[i] " ignored }";
      continue
    }
    if ( list[i] ~ /-D__USE_MINGW_/ ) {
      use_mingw=1
    }
    if ( list[i] ~ /lib[^ ]*\.so/ ) {
      dynamiclib = gensub (/([^ ]*)(lib[^ ]*\.so[^ ]*)(.*)/,"{$LINKLIB \\2} { found in \\1 \\3 }","g",list[i]);
      librarypath = gensub (/([^ ]*)(lib[^ ]*\.so[^ ]*)(.*)/,"{$LIBRARYPATH \\1} { for \\2 \\3 }","g",list[i]);
      print dynamiclib;
      print librarypath;
    }
    if ( list[i] ~ /^-l/ ) {
      systemlib = gensub (/-l([^ ]*)/,"\\1","g",list[i]);
      if (forcestatic == 1) {
        systemlib="lib" systemlib ".a"
      }
      if ( systemlib ~ /mingw/ ) {
      use_mingw=1
      }
      print "{$LINKLIB " systemlib "} { with -l gcc option}";
    }
  }
}
END {
  print "{ List implicit libraries }"
  nb = split (implibs,list);
  for (i=1;i<=nb; i++) {
    if ( list[i] ~ /lib.*\.a/ ) {
      lib=list[i];
    } else {
      if ( forcestatic == 1 ) {
        lib="lib" list[i] ".a";
      } else {
        lib=list[i];
      }
    }
    print "{$LINKLIB " lib "} { implicit library } "
  }
  print "{$endif COMPILING_GDBINT_UNIT }"
  print "{ List library dirs }"
  nb = split (libdir,list);
  for (i=1;i<=nb; i++) {
    dir=list[i];
    print "{$LIBRARYPATH " dir "} { library path } "
    print "{$OBJECTPATH " dir "} { library path } "
  }
  print "{$undef NotImplemented}"
  if ( use_mingw == 1 ) {
    print "{$define USE_MINGW_GDB}"
    if ( mingw_no_cpu_features_init == 1 ) {
      print "{$define DISABLE_CPU_FEATURES_INIT}"
    }
  }
  if ( has_get_stdin == 1 ) {
    print "{$define LIBGDB_HAS_GET_STDIN}"
  }
}
' | tee  gdblib.inc
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if ( list[i] ~ /lib[^ ]*\.a/ ) {
    staticlib = gensub (/([^ ]*)(lib[^ ]*\.a)/,"{$LINKLIB \\2} { found in \\1 }","g",list[i]);
    print staticlib;
    if ( list[i] ~/mingw/ ) {
    use_mingw=1
    }
  }
  if ( list[i] ~ /-D__USE_MINGW_/ ) {
    use_mingw=1
  }
  if ( list[i] ~ /lib[^ ]*\.so/ ) {
    dynamiclib = gensub (/([^ ]*)(lib[^ ]*\.so)/,"{$LINKLIB \\2} { found in \\1 }","g",list[i]);
    librarypath = gensub (/([^ ]*)(lib[^ ]*\.so)/,"{$LIBRARYPATH \\1} { for \\2 }","g",list[i]);
    print dynamiclib;
    print librarypath;
  }
  if ( list[i] ~ /-l/ ) {
    systemlib = gensub (/-l([^ ]*)/,"{$LINKLIB \\1} { with -l gcc option}","g",list[i]);
    print systemlib;
  }
  }
}
END {
  nb = split (implibs,list);
  for (i=1;i<=nb; i++) {
    print "{$LINKLIB " list[i] "} { implicit library } "
  }
  print "{$endif COMPILING_GDBINT_UNIT }"
  print "{$undef NotImplemented}"
  if ( use_mingw == 1 ) {
    print "{$define USE_MINGW_GDB}"
  }
}
' | tee  gdblib.inc


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
