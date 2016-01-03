Test runner for w3.org XML compliance suite
-------------------------------------------

The xmlts is intended to run the XML compliance suite from W3.org.
The suite includes 2500+ tests. It may be downloaded from
http://www.w3.org/XML/Test/xmlts20080205.zip  (approx. 1.7 mBytes)
After compiling xmlts.pp, run it with the following command line:

xmlts <path-to-xmlconf.xml> <report-filename> [-t template.xml] [-v]

Two required commandline parameters include path to test database file and report
filename. Optionally, you may specify validating mode with -v switch and report
template filename with -t (by default, 'template.xml' is used).
The test suite includes several test databases (all named 'xmlconf.xml'). There is
master database located in root dir, and several individual databases in different
subdirs.

for example, to run all tests included into the suite in non-validating mode, use:

xmlts xmlconf/xmlconf.xml myreport.html

Report is produced in xhtml format, use your favourite browser to view it.

As of 10.03.2007, the xml package does not support namespaces yet, so you might wish
to exclude namespace tests. To do this, edit xmlconf/xmlconf.xml file and comment out
the lines that contain references &eduni-ns10; &eduni-ns11; and &eduni-nse;


Testsuite errata
--------------------------------------------
The following issues were encountered while testing the parser. Fortunately, none
of these change the category of any test, but in some cases cause incorrect error
message and/or postion to be reported.

1) xmltest/not-wf/sa/081.xml
   xmltest/not-wf/sa/082.xml
   xmltest/not-wf/sa/083.xml
   xmltest/not-wf/sa/084.xml

All four reference an external entity with SystemID 'nul', which is a reserved
name under Windows (you won't be able to create such file). The archive contains
a file named 'nul.ent' that differs from entity's SystemID, so it won't resolve
anyway even in non-Windows.
This issue does not have any effect on FCL parser.
Additionally, tests 083.xml and 084.xml contain a reference to undefined notation.
This cause an extra validation error to be reported before the fatal error.

2) oasis/p49fail1.xml
   oasis/p50fail1.xml

Both tests are missing ']' that should close the internal DTD subset.

3) oasis/p58fail1.xml
   oasis/p58fail2.xml
   oasis/p58fail3.xml

All three have a NOTATION attribute declared on EMPTY element. This causes an extra
validation error to be reported before the fatal one.

4) ibm/xml-1.1/not-wf/p02/ibm02n66.ent

Presumably, missing '<' at start of CDATA. Does not change the diagnostic, though.

5) ibm/not-wf/p23/ibm23n05.xml

Contains encoding name 'ASCII' which is not supported by the parser. As a result, it aborts
before detecting the illegal XML declaration closing sequence.

6) ibm/not-wf/p72/ibm72n09.xml

Missing whitespace between 'ENTITY' and '%' at line 6 is detected before the bad tag closing
sequence.

7) ibm/not-wf/p77/ibm77n01.ent

Invalid encoding name 'UTF8' is detected before the wrong token order.

8) sun/invalid/attr03.xml
   sun/invalid/attr04.xml
   sun/invalid/attr15.xml

Have a NOTATION attribute is declared on EMPTY element. Diagnostics incorrect.

9) ibm/invalid/p56/ibm56i11.xml
   ibm/invalid/p56/ibm56i12.xml
   ibm/invalid/p56/ibm56i14.xml
   ibm/invalid/p56/ibm56i15.xml

Contain a reference to undeclared notation 'gif'. Diagnostics incorrect.

10) eduni/xml-1.1/052.xml
    eduni/xml-1.1/053.xml

Intended to test handling of NEL and LSEP chars as element content whitespace, these
tests enclose NEL and LSEP within ordinary ascii chars ('abc_def') that are clearly not
a whitespace. A 'correct' error is therefore reported regardless of actual NEL/LSEP handling.

11) ibm/not-wf/p69/ibm69n06.xml
    ibm/not-wf/p69/ibm69n07.xml

Designed to check parameter entity recursion, both tests contain PE references within entity
value declarations in internal DTD subset, which is a fatal error by itself.

12) ibm/not-wf/p21/ibm21n01.xml

Tests illegal CDEnd, but has an extra '[' in CDStart, which is detected earlier.

13) ibm/not-wf/p21/ibm21n02.xml

Tests illegal CDEnd, but has lowercase 'cdata' in CDStart, which is detected earlier.

14) ibm/xml-1.1/not-wf/p02/ibm02n58.xml

The first illegal character 0x99 is at position (2, 24), but another one at position (4,7) is
represented with malformed UTF-8 sequence (0xC1 0xA3, while correct one is 0xC2 0x99).
An 'xml-unaware' decoder can detect this before processing any 'normal' characters,
so diagnostics may be wrong.

