// included by glib2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
  PGMainContext = pointer;
  PGMainLoop    = pointer;

  PGSourceFunc = ^TGSourceFunc;
  TGSourceFunc = function (data:gpointer):gboolean;cdecl;

  {< private > }
  PGSourceCallbackFuncs = ^TGSourceCallbackFuncs;

  PGSource = ^TGSource;
  PGSourceFuncs = ^TGSourceFuncs;

  TGSource = record
    callback_data  : gpointer;
    callback_funcs : PGSourceCallbackFuncs;
    source_funcs   : PGSourceFuncs;
    ref_count      : guint;
    context        : PGMainContext;
    priority       : gint;
    flags          : guint;
    source_id      : guint;
    poll_fds       : PGSList;
    prev           : PGSource;
    next           : PGSource;
    reserved1      : gpointer;
    reserved2      : gpointer;
  end;

  TGSourceCallbackFuncs = record
    ref   : procedure (cb_data:gpointer); cdecl;
    unref : procedure (cb_data:gpointer); cdecl;
    get   : procedure (cb_data:gpointer; source:PGSource; func:PGSourceFunc; data:Pgpointer); cdecl;
  end;

  TGSourceDummyMarshal = procedure {no parameters} ;cdecl;
  { Can be NULL  }
  { For use by g_source_set_closure  }
  { Really is of type GClosureMarshal  }

  TGSourceFuncs = record
    prepare  : function  (source:PGSource; timeout:gint):gboolean; cdecl;
    check    : function  (source:PGSource):gboolean; cdecl;
    dispatch : function  (source:PGSource; callback:TGSourceFunc; user_data:gpointer):gboolean; cdecl;
    finalize : procedure (source:PGSource); cdecl;
    closure_callback : TGSourceFunc;
    closure_marshal  : TGSourceDummyMarshal;
  end;

  PGPollFD = ^TGPollFD;
  TGPollFD = record
    fd      : gint;
    events  : gushort;
    revents : gushort;
  end;

  TGPollFunc = function (ufds:PGPollFD; nfsd:guint; timeout:gint):gint;cdecl;
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
{ Standard priorities  }

const
  G_PRIORITY_HIGH         = -(100);
  G_PRIORITY_DEFAULT      = 0;
  G_PRIORITY_HIGH_IDLE    = 100;
  G_PRIORITY_DEFAULT_IDLE = 200;
  G_PRIORITY_LOW          = 300;
{ GMainContext:  }

function g_main_context_new:PGMainContext;cdecl;external gliblib name 'g_main_context_new';
procedure g_main_context_ref(context:PGMainContext);cdecl;external gliblib name 'g_main_context_ref';
procedure g_main_context_unref(context:PGMainContext);cdecl;external gliblib name 'g_main_context_unref';
function g_main_context_default:PGMainContext;cdecl;external gliblib name 'g_main_context_default';
function g_main_context_iteration(context:PGMainContext; may_block:gboolean):gboolean;cdecl;external gliblib name 'g_main_context_iteration';
function g_main_context_pending(context:PGMainContext):gboolean;cdecl;external gliblib name 'g_main_context_pending';

{ For implementation of legacy interfaces }
function g_main_context_find_source_by_id(context:PGMainContext; source_id:guint):PGSource;cdecl;external gliblib name 'g_main_context_find_source_by_id';
function g_main_context_find_source_by_user_data(context:PGMainContext; user_data:gpointer):PGSource;cdecl;external gliblib name 'g_main_context_find_source_by_user_data';
function g_main_context_find_source_by_funcs_user_data(context:PGMainContext; funcs:PGSourceFuncs; user_data:gpointer):PGSource;cdecl;external gliblib name 'g_main_context_find_source_by_funcs_user_data';

{ Low level functions for implementing custom main loops. }
procedure g_main_context_wakeup(context:PGMainContext);cdecl;external gliblib name 'g_main_context_wakeup';
function g_main_context_acquire(context:PGMainContext):gboolean;cdecl;external gliblib name 'g_main_context_acquire';
procedure g_main_context_release(context:PGMainContext);cdecl;external gliblib name 'g_main_context_release';
function g_main_context_wait(context:PGMainContext; cond:PGCond; mutex:PGMutex):gboolean;cdecl;external gliblib name 'g_main_context_wait';
function g_main_context_prepare(context:PGMainContext; priority:Pgint):gboolean;cdecl;external gliblib name 'g_main_context_prepare';
function g_main_context_query(context:PGMainContext; max_priority:gint; timeout:Pgint; fds:PGPollFD; n_fds:gint):gint;cdecl;external gliblib name 'g_main_context_query';
function g_main_context_check(context:PGMainContext; max_priority:gint; fds:PGPollFD; n_fds:gint):gint;cdecl;external gliblib name 'g_main_context_check';
procedure g_main_context_dispatch(context:PGMainContext);cdecl;external gliblib name 'g_main_context_dispatch';
procedure g_main_context_set_poll_func(context:PGMainContext; func:TGPollFunc);cdecl;external gliblib name 'g_main_context_set_poll_func';
function g_main_context_get_poll_func(context:PGMainContext):TGPollFunc;cdecl;external gliblib name 'g_main_context_get_poll_func';

{ Low level functions for use by source implementations }
procedure g_main_context_add_poll(context:PGMainContext; fd:PGPollFD; priority:gint);cdecl;external gliblib name 'g_main_context_add_poll';
procedure g_main_context_remove_poll(context:PGMainContext; fd:PGPollFD);cdecl;external gliblib name 'g_main_context_remove_poll';

{ GMainLoop:  }
function g_main_loop_new(context:PGMainContext; is_running:gboolean):PGMainLoop;cdecl;external gliblib name 'g_main_loop_new';
procedure g_main_loop_run(loop:PGMainLoop);cdecl;external gliblib name 'g_main_loop_run';
procedure g_main_loop_quit(loop:PGMainLoop);cdecl;external gliblib name 'g_main_loop_quit';
function g_main_loop_ref(loop:PGMainLoop):PGMainLoop;cdecl;external gliblib name 'g_main_loop_ref';
procedure g_main_loop_unref(loop:PGMainLoop);cdecl;external gliblib name 'g_main_loop_unref';
function g_main_loop_is_running(loop:PGMainLoop):gboolean;cdecl;external gliblib name 'g_main_loop_is_running';
function g_main_loop_get_context(loop:PGMainLoop):PGMainContext;cdecl;external gliblib name 'g_main_loop_get_context';

{ GSource:  }
function g_source_new(source_funcs:PGSourceFuncs; struct_size:guint):PGSource;cdecl;external gliblib name 'g_source_new';
function g_source_ref(source:PGSource):PGSource;cdecl;external gliblib name 'g_source_ref';
procedure g_source_unref(source:PGSource);cdecl;external gliblib name 'g_source_unref';
function g_source_attach(source:PGSource; context:PGMainContext):guint;cdecl;external gliblib name 'g_source_attach';
procedure g_source_destroy(source:PGSource);cdecl;external gliblib name 'g_source_destroy';
procedure g_source_set_priority(source:PGSource; priority:gint);cdecl;external gliblib name 'g_source_set_priority';
function g_source_get_priority(source:PGSource):gint;cdecl;external gliblib name 'g_source_get_priority';
procedure g_source_set_can_recurse(source:PGSource; can_recurse:gboolean);cdecl;external gliblib name 'g_source_set_can_recurse';
function g_source_get_can_recurse(source:PGSource):gboolean;cdecl;external gliblib name 'g_source_get_can_recurse';
function g_source_get_id(source:PGSource):guint;cdecl;external gliblib name 'g_source_get_id';
function g_source_get_context(source:PGSource):PGMainContext;cdecl;external gliblib name 'g_source_get_context';
procedure g_source_set_callback(source:PGSource; func:TGSourceFunc; data:gpointer; notify:TGDestroyNotify);cdecl;external gliblib name 'g_source_set_callback';

{ Used to implement g_source_connect_closure and internally }
procedure g_source_set_callback_indirect(source:PGSource; callback_data:gpointer; callback_funcs:PGSourceCallbackFuncs);cdecl;external gliblib name 'g_source_set_callback_indirect';
procedure g_source_add_poll(source:PGSource; fd:PGPollFD);cdecl;external gliblib name 'g_source_add_poll';
procedure g_source_remove_poll(source:PGSource; fd:PGPollFD);cdecl;external gliblib name 'g_source_remove_poll';
procedure g_source_get_current_time(source:PGSource; timeval:PGTimeVal);cdecl;external gliblib name 'g_source_get_current_time';

{ void g_source_connect_closure (GSource         source,
                                 GClosure        closure);
 }

{ Specific source types }
function g_idle_source_new:PGSource;cdecl;external gliblib name 'g_idle_source_new';
function g_timeout_source_new(interval:guint):PGSource;cdecl;external gliblib name 'g_timeout_source_new';

{ Miscellaneous functions }
procedure g_get_current_time(result:PGTimeVal);cdecl;external gliblib name 'g_get_current_time';

{ ============== Compat main loop stuff ==================  }

function g_main_new(is_running: gboolean): PGMainLoop;


{ these functions are maped to their newer versions }
procedure g_main_run        (loop:PGMainLoop);cdecl;external gliblib name 'g_main_loop_run';
procedure g_main_quit       (loop:PGMainLoop);cdecl;external gliblib name 'g_main_loop_quit';
procedure g_main_destroy    (loop:PGMainLoop);cdecl;external gliblib name 'g_main_loop_unref';
function  g_main_is_running (loop:PGMainLoop):gboolean;cdecl;external gliblib name 'g_main_loop_is_running';


{ Functions to manipulate the default main loop}
function  g_main_iteration  (may_block: gboolean): gboolean;
function  g_main_pending: gboolean;
//             g_main_context_pending        (NULL)
procedure g_main_set_poll_func(func:  TGPollFunc);
//   g_main_context_set_poll_func (NULL, func)

{ ======= end of Compat main loop stuff ==================  }

{ Source manipulation by ID  }
function g_source_remove(tag:guint):gboolean;cdecl;external gliblib name 'g_source_remove';
function g_source_remove_by_user_data(user_data:gpointer):gboolean;cdecl;external gliblib name 'g_source_remove_by_user_data';
function g_source_remove_by_funcs_user_data(funcs:PGSourceFuncs; user_data:gpointer):gboolean;cdecl;external gliblib name 'g_source_remove_by_funcs_user_data';

{ Idles and timeouts  }
function g_timeout_add_full(priority:gint; interval:guint; _function:TGSourceFunc; data:gpointer; notify:TGDestroyNotify):guint;cdecl;external gliblib name 'g_timeout_add_full';
function g_timeout_add(interval:guint; _function:TGSourceFunc; data:gpointer):guint;cdecl;external gliblib name 'g_timeout_add';
function g_idle_add(_function:TGSourceFunc; data:gpointer):guint;cdecl;external gliblib name 'g_idle_add';
function g_idle_add_full(priority:gint; _function:TGSourceFunc; data:gpointer; notify:TGDestroyNotify):guint;cdecl;external gliblib name 'g_idle_add_full';
function g_idle_remove_by_data(data:gpointer):gboolean;cdecl;external gliblib name 'g_idle_remove_by_data';

{ Hook for GClosure / GSource integration. Don't touch  }
// GLIB_VAR GSourceFuncs g_timeout_funcs;
// GLIB_VAR GSourceFuncs g_idle_funcs;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef windows}
=======
{$ifdef win32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef win32}
>>>>>>> origin/fixes_2_2
procedure g_main_poll_win32_msg_add(priority:gint; fd:PGPollFD; hwnd:guint);cdecl;external gliblib name 'g_main_poll_win32_msg_add';
{$endif}

{$ENDIF read_interface_rest}
// included by glib2.pas

