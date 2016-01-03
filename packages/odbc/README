Testing raw ODBC access:

For windows:
============

1. Compile testodbc. No options should be needed.

2. In the ODBC manager in the Windows Control Panel, create a new system or
   file DSN called 'FPC' with the 'Microsoft Access (*.mdb)' driver
   (At least MS-Access 97 or higher)

   Do not enter any username or password.
   
   The DSN should point to the testodbc.mdb database file provided with the
   testodbc.pp program.
   
3. Run the program.

For Linux:
==========

1. Change testodbc.pp and set the UserName and Password constants if needed.

2. Compile testodbc. No options should be needed.

3. Create a MySQL database and table with the mkdb script in the mysql
   directory. make sure the password and username as set in step 1 have
   access to this databse.

4. Install a DSN called FPC for the newly created database.
   (I used the unixODBC ODBCConfig program for this)

5. Run the program.

Enjoy !

Michael.


   
