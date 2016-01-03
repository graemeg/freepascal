// included by glib2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
  PGThreadError = ^TGThreadError;
  TGThreadError = ( G_THREAD_ERROR_AGAIN  { Resource temporarily unavailable  });

  TGThreadFunc = function (data:gpointer):gpointer;cdecl;

  PGThreadPriority = ^TGThreadPriority;
  TGThreadPriority = (G_THREAD_PRIORITY_LOW,
                      G_THREAD_PRIORITY_NORMAL,
                      G_THREAD_PRIORITY_HIGH,
                      G_THREAD_PRIORITY_URGENT);

  PGThread = ^TGThread;
  TGThread = record
               func : TGThreadFunc;
               data : gpointer;
               joinable : gboolean;
               priority : TGThreadPriority;
             end;

  PPGMutex  = ^PGMutex;
  PGMutex   = pointer; //       GMutex;
  PGCond    = pointer; //       GCond;
  PGPrivate = pointer; //       GPrivate;

//    typedef struct _GMutex* GStaticMutex;
  PGStaticMutex = ^TGStaticMutex;
  TGStaticMutex = PGMutex;

  PGThreadFunctions = ^TGThreadFunctions;
  TGThreadFunctions = record
            mutex_new       : function :PGMutex; cdecl;
            mutex_lock      : procedure (mutex:PGMutex); cdecl;
            mutex_trylock   : function (mutex:PGMutex):gboolean; cdecl;
            mutex_unlock    : procedure (mutex:PGMutex); cdecl;
            mutex_free      : procedure (mutex:PGMutex); cdecl;
            cond_new        : function :PGCond; cdecl;
            cond_signal     : procedure (cond:PGCond); cdecl;
            cond_broadcast  : procedure (cond:PGCond); cdecl;
            cond_wait       : procedure (cond:PGCond; mutex:PGMutex); cdecl;
            cond_timed_wait : function (cond:PGCond; mutex:PGMutex; end_time:PGTimeVal):gboolean; cdecl;
            cond_free       : procedure (cond:PGCond); cdecl;
            private_new     : function (dest:TGDestroyNotify):PGPrivate; cdecl;
            private_get     : function (private_key:PGPrivate):gpointer; cdecl;
            private_set     : procedure (private_key:PGPrivate; data:gpointer); cdecl;
            thread_create   : procedure (func:TGThreadFunc; data:gpointer; stack_size:gulong; joinable:gboolean; bound:gboolean;
                                         priority:TGThreadPriority; thread:gpointer; error:PPGError); cdecl;
            thread_yield    : procedure ; cdecl;
            thread_join     : procedure (thread:gpointer); cdecl;
            thread_exit     : procedure ; cdecl;
            thread_set_priority : procedure (thread:gpointer; priority:TGThreadPriority); cdecl;
            thread_self         : procedure (thread:gpointer); cdecl;
            thread_equal        : function (thread1:gpointer; thread2:gpointer):gboolean; cdecl;
         end;

  PGStaticPrivate = ^TGStaticPrivate;
  TGStaticPrivate = record
                      index : guint;
                    end;
  PGStaticRecMutex = ^TGStaticRecMutex;
  TGStaticRecMutex = record
          mutex : TGStaticMutex;
          depth : guint;
          owner : TGSystemThread;      // defined in glibconfig.inc
       end;

  PGStaticRWLock = ^TGStaticRWLock;
  TGStaticRWLock = record
         mutex         : TGStaticMutex;
         read_cond     : PGCond;
         write_cond    : PGCond;
         read_counter  : guint;
         write         : gboolean;
         want_to_read  : guint;
         want_to_write : guint;
      end;
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
function  g_thread_error_quark :  TGQuark; cdecl; external gthreadlib name 'g_thread_error_quark';
function G_THREAD_ERROR: TGQuark;


{$IFNDEF KYLIX}
  var
<<<<<<< HEAD
<<<<<<< HEAD
    {$IFDEF WINDOWS}
=======
    {$IFDEF WIN32}
>>>>>>> graemeg/fixes_2_2
=======
    {$IFDEF WIN32}
>>>>>>> origin/fixes_2_2
    g_thread_functions_for_glib_use : TGThreadFunctions; external gliblib name 'g_thread_functions_for_glib_use';
    g_thread_use_default_impl       : gboolean; external gliblib name 'g_thread_use_default_impl';
    g_threads_got_initialized       : gboolean; external gliblib name 'g_threads_got_initialized';
    {$ELSE}
    g_thread_functions_for_glib_use : TGThreadFunctions;cvar;external;      // ?????
    g_thread_use_default_impl       : gboolean;cvar;external;
    g_threads_got_initialized       : gboolean;cvar;external;
    {$ENDIF}
{$ENDIF}


procedure g_thread_init(vtable:PGThreadFunctions);cdecl;external gthreadlib name 'g_thread_init';
procedure g_thread_init_with_errorcheck_mutexes(vtable:PGThreadFunctions);cdecl;external gthreadlib name 'g_thread_init_with_errorcheck_mutexes';

{ A random number to recognize debug calls to g_mutex_...  }

const
  G_MUTEX_DEBUG_MAGIC = $f8e18ad7;


function g_static_mutex_get_mutex_impl(mutex:PPGMutex):PGMutex;cdecl;external gthreadlib name 'g_static_mutex_get_mutex_impl';

function  g_thread_supported: gboolean;
procedure g_mutex_lock     (mutex: PGMutex);
function  g_mutex_trylock  (mutex: PGMutex):gboolean;
procedure g_mutex_unlock   (mutex: PGMutex);
procedure g_mutex_free     (mutex: PGMutex);
procedure g_cond_wait      (cond: PGCond; mutex: PGMutex);
function g_cond_timed_wait (cond: PGCond;
                            mutex: PGMutex;
                            end_time: PGTimeVal):gboolean;

function  g_mutex_new : PGMutex;
function  g_cond_new  : PGCond;

procedure g_cond_signal    (cond: PGCond);
procedure g_cond_broadcast (cond: PGCond);
procedure g_cond_free      (cond: PGCond);
function  g_private_new    (dest: TGDestroyNotify): PGPrivate;
function  g_private_get    (private_key: PGPrivate): gpointer;
procedure g_private_set    (var private_key: PGPrivate; data: gpointer);

procedure g_thread_yield;


function g_thread_create      (func: TGThreadFunc;
                               data: gpointer;
                               joinable: gboolean;
                               error: PPGError): PGThread;

function g_thread_create_full (func : TGThreadFunc;
                               data : gpointer;
                               stack_size: gulong;
                               joinable, bound: gboolean;
                               priority : TGThreadPriority;
                               error    : ppGError): PGThread; cdecl; external gthreadlib name 'g_thread_create_full';


function g_thread_self:PGThread;cdecl;external gthreadlib name 'g_thread_self';
procedure g_thread_exit(retval:gpointer);cdecl;external gthreadlib name 'g_thread_exit';
function g_thread_join(thread:PGThread):gpointer;cdecl;external gthreadlib name 'g_thread_join';
procedure g_thread_set_priority(thread:PGThread; priority:TGThreadPriority);cdecl;external gthreadlib name 'g_thread_set_priority';

procedure g_static_mutex_lock   (mutex: PGStaticMutex);
function  g_static_mutex_trylock (mutex: PGStaticMutex): gboolean;
procedure g_static_mutex_unlock (mutex: PGStaticMutex);
procedure g_static_mutex_free(mutex:PGStaticMutex);cdecl;external gthreadlib name 'g_static_mutex_free';

const
  nG_STATIC_PRIVATE_INIT = 0;       //renamed because of conflict with function_name

procedure g_static_private_init(private_key:PGStaticPrivate);cdecl;external gthreadlib name 'g_static_private_init';
function g_static_private_get(private_key:PGStaticPrivate):gpointer;cdecl;external gthreadlib name 'g_static_private_get';
procedure g_static_private_set(private_key:PGStaticPrivate; data:gpointer; notify:TGDestroyNotify);cdecl;external gthreadlib name 'g_static_private_set';
procedure g_static_private_free(private_key:PGStaticPrivate);cdecl;external gthreadlib name 'g_static_private_free';

const
  nG_STATIC_MUTEX_INIT     = nil;      // from glibconfig.h
  nG_STATIC_REC_MUTEX_INIT = nG_STATIC_MUTEX_INIT;


procedure g_static_rec_mutex_init(mutex:PGStaticRecMutex);cdecl;external gthreadlib name 'g_static_rec_mutex_init';
procedure g_static_rec_mutex_lock(mutex:PGStaticRecMutex);cdecl;external gthreadlib name 'g_static_rec_mutex_lock';
function g_static_rec_mutex_trylock(mutex:PGStaticRecMutex):gboolean;cdecl;external gthreadlib name 'g_static_rec_mutex_trylock';
procedure g_static_rec_mutex_unlock(mutex:PGStaticRecMutex);cdecl;external gthreadlib name 'g_static_rec_mutex_unlock';
procedure g_static_rec_mutex_lock_full(mutex:PGStaticRecMutex; depth:guint);cdecl;external gthreadlib name 'g_static_rec_mutex_lock_full';
function g_static_rec_mutex_unlock_full(mutex:PGStaticRecMutex):guint;cdecl;external gthreadlib name 'g_static_rec_mutex_unlock_full';
procedure g_static_rec_mutex_free(mutex:PGStaticRecMutex);cdecl;external gthreadlib name 'g_static_rec_mutex_free';


procedure g_static_rw_lock_init(lock:PGStaticRWLock);cdecl;external gthreadlib name 'g_static_rw_lock_init';
procedure g_static_rw_lock_reader_lock(lock:PGStaticRWLock);cdecl;external gthreadlib name 'g_static_rw_lock_reader_lock';
function g_static_rw_lock_reader_trylock(lock:PGStaticRWLock):gboolean;cdecl;external gthreadlib name 'g_static_rw_lock_reader_trylock';
procedure g_static_rw_lock_reader_unlock(lock:PGStaticRWLock);cdecl;external gthreadlib name 'g_static_rw_lock_reader_unlock';
procedure g_static_rw_lock_writer_lock(lock:PGStaticRWLock);cdecl;external gthreadlib name 'g_static_rw_lock_writer_lock';
function g_static_rw_lock_writer_trylock(lock:PGStaticRWLock):gboolean;cdecl;external gthreadlib name 'g_static_rw_lock_writer_trylock';
procedure g_static_rw_lock_writer_unlock(lock:PGStaticRWLock);cdecl;external gthreadlib name 'g_static_rw_lock_writer_unlock';
procedure g_static_rw_lock_free(lock:PGStaticRWLock);cdecl;external gthreadlib name 'g_static_rw_lock_free';

{$ENDIF read_interface_rest}
// included by glib2.pas
