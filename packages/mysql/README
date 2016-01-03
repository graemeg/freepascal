This is the MySQL interface of Free Pascal.

You need at least compiler version 0.99.8 to compile this.

To Compile under Linux
- You must know where the mysql libraries are.
- You must know where libgcc is.

The units come in 2 flavours:
- mysql3*.pp  : version 3.23 of mysql.
- mysql4*.pp  : version 4.0 of mysql.
These versions are substantially different.

Both these things must be set in the Makefile. After that a simple 'make'
and 'make install' should compile and install everything.

You can test with 'make test', but this supposes that there is a 'testdb'
database available. If not, you must run
  mkdb databasename
  testdb databasename
  rmdb databasename
manually. You need create permission on the database for this to work.

Win32 users: There are multiple .dll files circulating around. 
you should fix the statement
mysqllib = 'libmysql'
in the mysql unit, so it matches the version you have.

Enjoy !



Michael.
