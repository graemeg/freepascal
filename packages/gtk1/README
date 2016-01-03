Free Pascal interface to GDK/GTK
================================

Prerequisites:
--------------

In order for the makefile to work, you NEED the following file:
  makefile.fpc
and it MUST be located in the directory above this directory.

If you have this file on another place in your directory tree, just 
set the FPCDIR variable so it points to that directory.
  (e.g. FPCDIR=/usr/lib/fpc/0.99.13; export FPCDIR in bash, or
        SETENV FPCDIR /usr/lib/fpc/0.99.13 in csh)

If you don't have this file, you can get it from
  ftp://tflily.fys.kuleuven.ac.be/pub/fpc/source/base.zip
or one of the mirrors.
 
Compiling the units:
--------------------

there are 4 targets for the makefile

make all      : compile all units
make install  : make all first and then install the units
make examples : compile the examples
make clean    : clean up object and unit files.


Using the units:
----------------

1) In C, you only need to input gtk.h. Here you need to input all files
   that you're likely to use, so you may have to experiment.

2) Names :
   + Pascal reserved words in types, record element names etc. have been
     prepended with the word 'the'. so 'label' has become 'thelabel'
   + functions have been kept with the same names.
   + for types : gdk names have been kept. Pointers to a type are defined
     as the type name, prepended with P. So 'GtkWidget *' becomes
     'PGtkWidget'.
     In gtkobject, names also have been kept.
     In all other files, types have been prepended with T, that is, the C
     type 'GtkWidget' has become 'TGtkWidget'
     
     This is annoying, but C is case sensitive, and Pascal is not, so
     there you have it...
     
     When translating, I've tried to stick to this scheme as closely as I
     could. One day, however, all will be done in a uniform manner...

3) Macros. Many C macros have not been translated. The typecasting macros 
   have been dropped, since they're useless under pascal.
   Macros to access record members have been translated, BUT they are 
   to be considered as READ-ONLY. So they can be used to retrieve a value,
   but not to store one.
   e.g.
      function GTK_WIDGET_FLAGS(wid : pgtkwidget) : longint;
   can be used to retrieve the widget flags, but not to set them.
   so things like 
     GTK_WIDGET_FLAGS(wid):=GTK_WIDGET_FLAGS(wid) and someflag;
   will not work, since this is a function, and NOT a macro as in C.
  
4) Packed records. GCC allows you to specify members of a record in
   bit format. Since this is impossible in pascal, functions and procedures
   to get/set these elements have been made.
   e.g.
     function width_set(var a : TGtkCListColumn) : gint;
     procedure set_width_set(var a : TGtkCListColumn; __width_set : gint);
   can be used to get or set the width in a TGtkCListColumn...
   in general, it's the name with '_set' appended for getting a value
   (set from 'a set') , and  'set_' prepended (from 'to set') and again
   '_set' appended.
   
   
And that is about all there is to say about it.

Enjoy !
Michael.
