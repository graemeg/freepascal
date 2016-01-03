This is the FPC translation of the C library header files.
This unit is meant to
- Give full access to the GNU libc library functionality.
- Be compatible to Borland's Libc.pas unit.

There will often exist 2 versions of a function call: one with a 
pointer type argument - this is  the straight translation of a libc call.
One is with a var/const argument, this is a more 'pascal' like call,
which is usually also the way Borland did it.

Each C header file has been translated to an .inc file. 
The translation of header file xxx.h is named xxxh.inc. 
If macros were converted, they are in a file xxx.inc

All files are included in libc.pp and their origin is noted there.

the file glue.inc is meant to form a bridge between the pascal
and C types, and to take care of some problems in the ordering 
of the header files.

The translation was done on a SuSE 8.1 machine:
Kernel version: 2.4.18
glibc version: 2.3

Note on the Libc errno variable. On recent systems the errno symbol is no
longer published in libc. It has been replaced by a __errno_location
pointer, with a macro in the C header files to mask this. The pascal 
Libc files assume this by default. This also means you cannot set the 
errno value directly, you must use the 'seterrno' procedure for that.
(see errno.inc)

The old mechanism of a published errno libc variable is still available 
by setting the LIBC_OLDERRNO define when compiling the libc units.

Michael.
