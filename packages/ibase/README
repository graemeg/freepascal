This is the README file for the INTERBASE package of FPC

How to compile
--------------

You can compile by hand, or use the makefile. The makefile uses the
makefile.fpc that is distributed by the FPC team.

1) Using the makefile.
   Edit the makefile: 
     set the DEFAULTFPCDIR
     if libgds is not in /usr/lib, set GDSLIBDIR
   type 'make'. It should all compiler

2) By hand:
   simply type:
   ppc386 -S2 ibase60
   (or ibase40 if you have the older version)

   If your libgds.so file is not in /usr/lib, you may need to add
   -Fl/path/to/libgds

Using the testprogram
---------------------

The mkdb script (mkdb.pas batch file) creates a database.
default testdb.gdb is assumed. You can override that by 
specifying an alternate database as an option.

The testib program connects to the testdatabase. You must edit
the testib.pp file and set the database, password and username.
After editing, compile the program. 
Run it and it will display the contents of the database.

Enjoy !

Michael.

   

   


