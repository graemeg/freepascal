Usage
-----
To start the test suite execute:

  make full TEST_FPC=path_to_your_compiler

This should run all tests, scan the created log file and output some
statistics.

  make digest TEST_FPC=path_to_your_compiler

scans the created log file and outputs some statistics

  make digest USESQL=YES TEST_FPC=path_to_your_compiler

sends the results to an SQL database

When the tests are performed, first the units (e.g. rtl) needed by the
tests are compiled in a clean determined way and put in the units
directory. Then webtbs/webtbf/test/tbs/tbf are searched for t*.pp to be
compiled and executed as tests.


Parallel test suite runs
------------------------

It is possible to run the test suite in parallel, but only when using the
"all", "full" or "fulldb" targets and only if SINGLEDOTESTRUNS is not used.
Under those circumstances, it is safe to use -jx, with x the number of tests
that can be compiled and run in parallel. E.g.

  make full TEST_FPC=path_to_your_compiler -j 2

Make sure to clean the test suite between two runs for the same platform.


Directories
-----------
webtbs...........Tests for web-bug-database bugs (should compile/run)
                   Digits in filename refer to bug database entry
webtbf...........Tests for web-bug-database bugs (should not compile/run)
                   Digits in filename refer to bug database entry
test.............Test suites for different aspects of the compiler/rtl etc
test/packages....Tests depending on packages. The tests are sorted by package
                 though this is not mandatory because tests can depend on
                 multiple packages
tbs..............Tests for other bugs, added by the fpc core team
                   (success in compilation) Digits in filename is a serial no
tbf..............Tests for other bugs, added by the fpc core team
                   (fail compile) Digits in filename is a serial no
units............Helper units for doing the tests
utils............Utilities for processing tests


Writing a test
--------------
A test should have a name on the form t*.pp, to be recognized as a test.
It should return 0 on success, any other value indicates failure.



Test directives
---------------
At the top of the test source code, some directives can be used to
determine how the tests will be processed (if processed automatically via
make), e.g. {%CPU=i386}:

OPT................Compiler option required to compile
CPU................Only for these CPU's (i386,m68k,etc). Might be a list.
SKIPCPU............Not for these CPU's (i386,m68k,etc). Might be a list.
TARGET.............Only for these OS targets (win32,MacOS,etc).
                   Might be a list.
SKIPTARGET.........Not for these OS targets (win32,MacOS,etc).
                   Might be a list.
VERSION............Compiler with at lest this version number required.
MAXVERSION.........Compiler with at most this version number required.
RESULT.............Exit code of execution of test expected
TIMEOUT............Timeout indication for test in seconds (only used if
                   enabled by defining TEST_TIMEOUT)
FILES..............List of files (separated by spaces) required by test,
                   will be copied to remote before execution. If TEST_DELTEMP
                   is set, all these files will be deleted after the test.
DELFILES...........List of files (separated by spaces) to be deleted after
                   the test. Files will be deleted only if TEST_DELTEMP is
                   set. If no extension is specified for a file, the dotest
                   program will try to delete executable and shared library
                   of that name, using appropriate exe and dll extensions
CONFIGFILE         Specifies a file in ./config that is to be copied to the
                   test. If two names are given the first is the source filename
				   and the second is the destination filename. If only one is
				   given than it acts both as source and destination filename.
GRAPH..............Requires graph unit
FAIL...............Compilation must fail
RECOMPILE..........After compiling a test, recompile the test for a second
                   time. This is needed to test ppu loading issues. Can
		   optionally specify a command line option to use only
		   during the second compilation.
NORUN..............Do not execute test, only compile it
INTERACTIVE........Do not execute test, as it requires user intervention
NOTE...............Output note when compiling/executing test
NEEDLIBRARY........Adds -rpath to the linker for unix. This is needed to
                   test runtime library tests. The library needs the -FE.
                   option to place the .so in the correct directory.
NEEDEDAFTER........Use it if the files are necessary for a later test.
                   this option will avoid delteting generated file even when
                   TEST_DELTEMP is used.
KNOWNRUNERROR......Known bug, which manifest itself at runtime. To the
                   right of the equal sign is the expected exit code,
                   followed by an optional note. Will not be logged
                   as a bug.
KNOWNCOMPILEERROR..Known bug, which manifest itself at compile time. To
                   the right of the equal sign is the expected exit code
                   from compiler, followed by an optional note. Will not
                   be logged as a bug.
QUICKTEST..........If set, only tests without package dependencies are executed
WPOPARAS...........Parameters to be added after -OW/-Ow to perform whole
                   program optimization tests
WPOPASSES..........Number of whole program optimization iterations to perform
                   ("1" means compile once with "-FWsomefile -OW<wpoparas>"
                    and then again with "-FWsomefile2 -OW<wpoparas>
                    -Fwsomefile1 -Ow<wpoparas>", "2" means another pass but
                   using somefile2 as input and somefile3 as output, etc.)

  NOTE: A list consists of comma separated items, e. g. CPU=i386,m68k,powerpc
        No space between the elements and the comma.


Controling testing in more detail
---------------------------------
Calling "make full" will perform tests in a standard manner. To have
more control of the test process one must distinguish between:

* Driver environment: compiler/rtl etc. to be used by the tools which
  runs and analyze the tests. All normal options to make, like FPC
  OS_TARGET, OPT etc. controls this.

* Test environment:  compiler/rtl etc. to be tested, to be used
  *in* the tests. Ususal options, prepended with TEST_ , controls
  this. If no such options are given, test and driver environment
  will be the same.

This differentiation also enables cross testing.

The following test options can be given:

TEST_FPC               compiler to test (no default)
TEST_OS_TARGET         defaults to default target of TEST_FPC
TEST_CPU_TARGET        defaults to default target of TEST_FPC
TEST_OPT               defaults to ""
TEST_FPC_VERSION       defaults to version of TEST_FPC
TEST_CCOMPILER         defaults to installed gcc compiler, but only
                       if driver and test full-targets are the same.
TEST_VERBOSE           let dotest be more verbose, only useful for debugging
TEST_BENCH             display compilation/execution time of each test
TEST_DELTEMP           delete temporary executable/object/ppu file,
                       default is off
TEST_TIMEOUT           use timeout wrapper for (remote) execution
V                      print dotest commandline
TEST_ABI               test a certain abi, this influences where the
                       c object files are taken from: TEST_ABI=eabi
                       takes the c*.o files from
                       test/cg/obj/linux/arm-eabi
CHUNKSIZE              Number of tests per chunk that can be potentially executed
                       in parallel with other chunks

  (Please add more test options if needed)

NOTE: To clean after a test session, "make clean TEST_FPC=path_to_your_compiler"
must be given the same options as when running the tests.

The utils directory is considerd to belong to the driver environment,
all other directories belong to the test environment.


Remote execution
----------------
Also remote execution of the test suite is possible.

Requirements:
- rsh/ssh must work without keyboard interaction or extra parameters

Test options:
TEST_RSH             set this to the hostname when you want to use rsh/rcp
                     to execute/copy the test
TEST_SSH             set this to use ssh/scp to execute the test
TEST_PUTTY           test using putty when remote testing (pscp and plink)
TEST_ADB             run tests on a remote Android device using ADB. Specify
                     TEST_ADB=1 to run on a default connected device. Specify
                     TEST_ADB=<serial> to run on a specific connected device.
TEST_REMOTEOPT       extra options to remote program
TEST_REMOTEPATH      set remote path to use, default is /tmp
TEST_DELBEFORE       delete remote executable before uploading
TEST_DELTEMP         delete executable after running, so the remote system
                     doesn't need much free disk space
TEST_REMOTEPW        pass a password with -pw to remote tools,
                     mainly useful for putty


Examples:
-------
  make TEST_FPC=$HOME/fpc/compiler/ppcsparc TEST_BINUTILSPREFIX=sparc-linux- \
       TEST_RSH=sunny TEST_REMOTEPATH=/tmp/tests

  make TEST_FPC=$HOME/fpc/compiler/ppcsparc TEST_BINUTILSPREFIX=sparc-linux- \
       TEST_SSH=fpc@sunny TEST_REMOTEPATH=/tmp/tests

Example for win32/putty:

  make TEST_FPC=c:\fpc\compiler\ppcarm TEST_BINUTILSPREFIX=arm-linux- \
       TEST_PUTTY=root@192.168.42.210 TEST_REMOTEPATH=/tmp TEST_DELTEMP=1 \
       "TEST_REMOTEPW=xxx" FPC=c:\fpc\compiler\ppc386


Emulator execution
------------------
Emulator execution is possible as well. It can't be combined with remote
execution though.

EMULATOR: name of the emulator to use
EMULATOR_OPTS: pass the given options to the emulator

Examples:

  make TEST_FPC=~/fpc/compiler/ppcrossarm TEST_OPT=-XParm-linux- \
       EMULATOR=qemu-arm
  make TEST_FPC=~/fpc/compiler/ppcrossarm TEST_OPT=-XParm-linux- \
       EMULATOR=qemu-arm digest DBDIGESTOPT="-C qemu-arm" USESQL=YES


Example cross testing of target Mac OS with driver Darwin
--------------------------------------------------------
NOTE: Today, it is possible to run the test suite Mac OS native.

A machine with both Mac OS X and classic Mac OS installed is required.
Note that make will not run the tests.  This has to be done in MPW with the
scripts in utils/MacOS.

  make clean alltest TEST_OS_TARGET=MacOS TEST_OPT="-WT -st" \
       USEUNITDIR=/Projekt/Freepascal/fpc/rtl/MacOS

To clean. Note that same options as above has to be given so that the
correct files will be removed.

  make clean TEST_OS_TARGET=MacOS USEUNITDIR=/Projekt/Freepascal/fpc/rtl/MacOS


Example cross testing of target arm-wince
-----------------------------------------
//arm-wince example:
see FPCTRUNK\DEMO\WINCE\TESTEMU\ for additionally required tools
Connect your device via ActiveSync and execute:

  make TEST_FPC=ppcrossarm TEST_CPU_TARGET=arm TEST_OS_TARGET=wince \
       TEST_OPT="-XParm-wince- -WC -Xs" \
       EMULATOR=MyDisc:\My\Path\to\wcetemu.exe

Tests will be performed in the \fpctests folder on the device.


Result uploading
----------------
Results can be uploaded to the test suite result DB
(http://www.freepascal.org/testsuite/cgi-bin/testsuite.cgi) by executing

  make uploadrun

On unix:
- The testing machine must be enabled to login on www.freepascal.org
  automatically as user fpc with ssh.

On Windows:
- The putty utilities plink.exe and pscp.exe must be in the path
- There must be a putty session named fpc@www.freepascal.org which is
  enabled to login automatically into www.freepascal.org

Use TEST_USE_LONGLOG=1 as make parameter to pass information
to server using longlog file instead of the whole output.
