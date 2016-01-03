This is the PostGresSQL interface of Free Pascal.

It has been tested with versions 6.2.xxx and 6.3.xxx of PostGreSQL.
You need at least compiler version 0.99.8 to compile this.

To Compile
- You must know where the PostGreSQL libraries (libpq) are.
- You must set the variable OLD in the makefile to 'yes', if your
  postgresql version is 6.2.xxx or earlier.

Both these things must be set in the Makefile. After that a simple 'make'
and 'make install' should compile and install everything.

You can test with 'make test', but this supposes that there is a 'testdb'
database available. If not, you must run
  mkdb databasename
  testdb databasename
  rmdb databasename
manually. You need create permission on the database for this to work.

Michael.
