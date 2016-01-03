
This is the Free Pascal interface to the GDBM library routines. 
Essentially, this is a translation of the gdbm.h header files, with some
additional routines.

The headers translated without any problems, the only thing that should
be taken into account is that the 
GDBM_SYNC constant (for open flags) has been renamed to GDMB_DOSYNC
because it conflicts with the gdbm_sync function. 

Be careful: the TDatum.dptr data pointer which is allocated by the
gdbm routines should be freed by the C free() call, NOT with the 
pascal FreeMem() call.

A solution for this is to use the 'cmem' unit, which replaces the standard
FPC memory manager with the C memory manager. In that case, freemem()
may be used to free the dptr field of the TDatum record.

On top of the plain C header translations, The GDBM routines have been 
overloaded with routines that accept plain strings as key or data 
parameters. This means the following routines have been added:

function gdbm_open(Const para1:string; para2:longint; para3:longint; para4:longint; para5:TGDBMErrorCallBack ):PGDBM_FILE;
function gdbm_store(para1:PGDBM_FILE; Const para2:string; Const para3:string; para4:longint):Boolean;
function gdbm_fetch(para1:PGDBM_FILE; Const para2:string):string;
function gdbm_delete(para1:PGDBM_FILE; Const para2:string):boolean;
procedure gdbm_firstkey(para1:PGDBM_FILE; var key :string);
function gdbm_nextkey(para1:PGDBM_FILE; Const para2:string):string;
function gdbm_exists(para1:PGDBM_FILE; Const para2:string):boolean;

They are just the C routines, but with the TDatum type (a record) 
replaced by a string. The routines take automatically care of memory
deallocation.

Functions that returned an integer to indicate success or failure have been
replaced by functions that return a boolean.

Careful: 
When using ansistrings, make sure the gdbm unit has been compiled
with the -Sh switch. The unit should work with both kinds of strings.

There are 2 test programs:
testgdbm tests the raw C header translation
testgdbm2 tests the String interface to the GDBM routines.

That's about it. 

Enjoy!

Michael. (Michael.VanCanneyt@Wisa.be)
