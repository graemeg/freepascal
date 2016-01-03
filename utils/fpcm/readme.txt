This utility is used to re-create the complete
Free Pascal makefiles, depending on the rules
specified in the Makefile.fpc files.

Some tips:
- To regenerate the makefiles for all supported
  targets, simply do : fpcmake -Tall
- To regenerate the complete makefile tree
  for all targets (under UNIX): 
    fpcmake -Tall -w `find . -name Makefile.fpc`
<<<<<<< HEAD
<<<<<<< HEAD

  To avoid svn conflicts for each locally changed Makefile
instead of using the current date for fpcmake executable,
revision.inc file is automatically updated to contain
the lastest date and revision of the files that do matter for
the Makefile generation.
  Each time any of these files "fpc*.pp fpcmake.ini Makefile.fpc"
is modified, revision.inc content is modified.
  This modified content should also be committed, to have correct
revision.inc content even on systems that do not have
svnversion binary or on exported trees.
PM 2013-04-26
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
