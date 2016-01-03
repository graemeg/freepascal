Testing FCL DOM implementation with official test suite from w3.org
-------------------------------------------------------------------

*** IMPORTANT: READ CAREFULLY!

IF YOU ARE ABOUT TO RUN THESE TESTS, CONSIDER DOING SO IN AN ENVIRONMENT
THAT YOU MAY ALLOW TO BE TRASHED.

As of writing this at 3 June 2008, FCL DOM memory model is
not compatible - at all - with the way that w3.org tests use. In
particular, tests acquire (and use) references to objects that DOM
implementation frees. Therefore, running the tests WILL result in heap
corruption, executing arbitrary code, and any other imaginable kind of
disaster. Be warned.

*** End of notice
--------------------------------------------------------------------


To test the FCL DOM implementation, follow these steps:

1) Checkout the DOM test suite from w3.org CVS repository. The project name is
2001/DOM-Test-Suite. Only 'tests' subdirectory is needed, everything else
is irrelevant for our purposes.
Use the following commands:

  CVSROOT=:pserver:anonymous@dev.w3.org:/sources/public
  cvs login
  (enter the password anonymous when prompted)
  cvs checkout 2001/DOM-Test-Suite/tests

2) Compile the testgen utility. A simple

  fpc testgen.pp

should do it.

3) Use testgen to convert DOM test suites into Pascal code. Specify path to the
directory that contains 'alltests.xml' file, and the name of resulting FPC unit.
Testgen expects the API description file 'api.xml' present in its directory.
Successful conversion of the following test modules is possible:

Level 1 Core (527 tests):
  testgen 2001/DOM-Test-Suite/tests/level1/core core1.pp

Level 2 Core (282 tests):
  testgen 2001/DOM-Test-Suite/tests/level2/core core2.pp

Level 3 Core (partial only, 131 out of 722 tests):
  testgen 2001/DOM-Test-Suite/tests/level3/core core3.pp

In the examples above, output names (core1.pp, etc.) carry no defined meaning, you may
use anything instead.

Normally, tests that contain properties/methods unsupported by FCL DOM, or
other elements not yet known to testgen, will be skipped. The conversion may be forced
by using -f commandline switch, but in this case the resulting Pascal unit will likely
fail to compile.
 
4) Now, pick up your preferred fpcunit test runner, include the generated units into
its uses clause, and compile. During compilation, path to 'domunit.pp' should be added
to the unit search paths.

5) During runtime, tests must be able to read test files which are located
within CVS source tree ('files' subdirectory of each module directory).
