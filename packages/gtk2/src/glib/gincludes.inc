{%MainUnit glib2.pas}

{$include gtypes.inc}
{$include gtype.inc}
{$include glibconfig.inc}

{$include gvalue.inc}
{$include gvaluearray.inc}
{$include gvaluecollector.inc}
{$include gvaluetypes.inc}

{$include gparam.inc}
{$include gclosure.inc}
{$include gsignal.inc}
{$include gtypeplugin.inc}
{$include gobject.inc}

{$include gmacros.inc}
{$include genums.inc}
{$include gstrfuncs.inc}
{$include gutils.inc}
{$include galloca.inc}
{$include ghash.inc}
{$include gquark.inc}
{$include gerror.inc}
{$include gbacktrace.inc}
{$include gmem.inc}
{$include garray.inc}
{$include gslist.inc}
{$include glist.inc}
{$include gcache.inc}
{$include gcompletion.inc}
{$include gconvert.inc}
{$include gdataset.inc}
{$include gdate.inc}
{$include gdir.inc}
{$include gfileutils.inc}
{$include ghook.inc}

{$include gthread.inc}
{$include gthreadpool.inc}
{$include gtimer.inc}
{$include gmain.inc}
{$include gasyncqueue.inc}

{$include gunicode.inc}
{$include gstring.inc}
{$include giochannel.inc}

{$include gmessages.inc}
{$include gmarkup.inc}
{$include gnode.inc}
{$include gtree.inc}
{$include gpattern.inc}
{$include gprimes.inc}
{$include gqsort.inc}
{$include gqueue.inc}
{$include grand.inc}
{$include grel.inc}
{$include gscanner.inc}
{$include gshell.inc}
{$include gspawn.inc}
{$include gboxed.inc}

{$include gmodule.inc}

{$include gmarshal.inc}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF winDOWS}
=======
{$IFDEF win32}
>>>>>>> graemeg/fixes_2_2
=======
{$IFDEF win32}
>>>>>>> origin/fixes_2_2
{$include gwin32.inc}
{$ENDIF}

{$include goption.inc}

{$ifdef HasGLIB2_10}
{$include gslice.inc}
{$endif}

// included by glib2.pas
