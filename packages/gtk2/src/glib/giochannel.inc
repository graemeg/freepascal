// included by glib2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
  PGIOError = ^TGIOError;
  TGIOError = (
    G_IO_ERROR_NONE,
    G_IO_ERROR_AGAIN,
    G_IO_ERROR_INVAL,
    G_IO_ERROR_UNKNOWN
  );

  PGIOChannelError = ^TGIOChannelError;
  TGIOChannelError = (
    G_IO_CHANNEL_ERROR_FBIG,
    G_IO_CHANNEL_ERROR_INVAL,
    G_IO_CHANNEL_ERROR_IO,
    G_IO_CHANNEL_ERROR_ISDIR,
    G_IO_CHANNEL_ERROR_NOSPC,
    G_IO_CHANNEL_ERROR_NXIO,
    G_IO_CHANNEL_ERROR_OVERFLOW,
    G_IO_CHANNEL_ERROR_PIPE,
    G_IO_CHANNEL_ERROR_FAILED
  );

  PGIOStatus = ^TGIOStatus;
  TGIOStatus = (
    G_IO_STATUS_ERROR,
    G_IO_STATUS_NORMAL,
    G_IO_STATUS_EOF,
    G_IO_STATUS_AGAIN
  );

  PGSeekType = ^TGSeekType;
  TGSeekType = (
    G_SEEK_CUR,
    G_SEEK_SET,
    G_SEEK_END
  );


  PGIOCondition = ^TGIOCondition;
  TGIOCondition = gint;

  PGIOFlags = ^TGIOFlags;
  TGIOFlags = gint;
  PGIOChannel = ^TGIOChannel;
  TGIOFunc = function (source:PGIOChannel; condition:TGIOCondition; data:gpointer):gboolean;cdecl;
  PGIOFuncs = ^TGIOFuncs;

  TGIOFuncs = record
    io_read  : function (channel:PGIOChannel; buf:Pgchar; count:gsize; bytes_read:Pgsize; err:PPGError):TGIOStatus;cdecl;
    io_write : function (channel:PGIOChannel; buf:Pgchar; count:gsize; bytes_written:Pgsize; err:PPGError):TGIOStatus;cdecl;
    io_seek  : function (channel:PGIOChannel; offset:gint64; _type:TGSeekType; err:PPGError):TGIOStatus;cdecl;
    io_close : function (channel:PGIOChannel; err:PPGError):TGIOStatus;cdecl;
    io_create_watch : function (channel:PGIOChannel; condition:TGIOCondition):PGSource;cdecl;
    io_free : procedure (channel:PGIOChannel);cdecl;
    io_set_flags : function (channel:PGIOChannel; flags:TGIOFlags; err:PPGError):TGIOStatus;cdecl;
    io_get_flags : function (channel:PGIOChannel):TGIOFlags;cdecl;
  end;

  TGIOChannel = record
    ref_count : guint;
    funcs     : PGIOFuncs;
    encoding  : Pgchar;
    read_cd   : TGIConv;
    write_cd  : TGIConv;
    line_term : Pgchar;                        // String which indicates the end of a line of text
    line_term_len : guint;                     // So we can have null in the line term
    buf_size  : gsize;
    read_buf  : PGString;                      // Raw data from the channel
    encoded_read_buf : PGString;               // Channel data converted to UTF-8
    write_buf : PGString;                      // Data ready to be written to the file
    partial_write_buf : array[0..5] of  gchar; // UTF-8 partial characters, null terminated

     { Group the flags together, immediately after partial_write_buf, to save memory  }
    flag0 : word;
    reserved1 : gpointer;
    reserved2 : gpointer;
  end;
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}

function G_IO_CHANNEL_ERROR : TGQuark;

const
  G_IO_IN      = GLIB_SYSDEF_POLLIN;
  G_IO_OUT     = GLIB_SYSDEF_POLLOUT;
  G_IO_PRI     = GLIB_SYSDEF_POLLPRI;
  G_IO_ERR     = GLIB_SYSDEF_POLLERR;
  G_IO_HUP     = GLIB_SYSDEF_POLLHUP;
  G_IO_NVAL    = GLIB_SYSDEF_POLLNVAL;

const
  G_IO_FLAG_APPEND       = 1 shl 0;
  G_IO_FLAG_NONBLOCK     = 1 shl 1;
  G_IO_FLAG_IS_READABLE  = 1 shl 2;
  G_IO_FLAG_IS_WRITEABLE = 1 shl 3;
  G_IO_FLAG_IS_SEEKABLE  = 1 shl 4;
  G_IO_FLAG_MASK         = (1 shl 5) - 1;
  G_IO_FLAG_GET_MASK     = G_IO_FLAG_MASK;
  G_IO_FLAG_SET_MASK     = G_IO_FLAG_APPEND or G_IO_FLAG_NONBLOCK;


const
  bm_TGIOChannel_use_buffer     = $1;
  bp_TGIOChannel_use_buffer     = 0;
  bm_TGIOChannel_do_encode      = $2;
  bp_TGIOChannel_do_encode      = 1;
  bm_TGIOChannel_close_on_unref = $4;
  bp_TGIOChannel_close_on_unref = 2;
  bm_TGIOChannel_is_readable    = $8;
  bp_TGIOChannel_is_readable    = 3;
  bm_TGIOChannel_is_writeable   = $10;
  bp_TGIOChannel_is_writeable   = 4;
  bm_TGIOChannel_is_seekable    = $20;
  bp_TGIOChannel_is_seekable    = 5;

{ use this to get/set information to the TGIOChannel struct}
function  TGIOChannel_use_buffer          (var a : TGIOChannel) : guint;
procedure TGIOChannel_set_use_buffer      (var a : TGIOChannel; __use_buffer : guint);
function  TGIOChannel_do_encode           (var a : TGIOChannel) : guint;
procedure TGIOChannel_set_do_encode       (var a : TGIOChannel; __do_encode : guint);
function  TGIOChannel_close_on_unref      (var a : TGIOChannel) : guint;
procedure TGIOChannel_set_close_on_unref  (var a : TGIOChannel; __close_on_unref : guint);
function  TGIOChannel_is_readable         (var a : TGIOChannel) : guint;
procedure TGIOChannel_set_is_readable     (var a : TGIOChannel; __is_readable : guint);
function  TGIOChannel_is_writeable        (var a : TGIOChannel) : guint;
procedure TGIOChannel_set_is_writeable    (var a : TGIOChannel; __is_writeable : guint);
function  TGIOChannel_is_seekable         (var a : TGIOChannel) : guint;
procedure TGIOChannel_set_is_seekable     (var a : TGIOChannel; __is_seekable : guint);


procedure g_io_channel_init(channel:PGIOChannel);cdecl;external gliblib name 'g_io_channel_init';
procedure g_io_channel_ref(channel:PGIOChannel);cdecl;external gliblib name 'g_io_channel_ref';
procedure g_io_channel_unref(channel:PGIOChannel);cdecl;external gliblib name 'g_io_channel_unref';

{DEPRECATED functions}

function g_io_channel_read(channel:PGIOChannel; buf:Pgchar; count:gsize; bytes_read:Pgsize):TGIOError;cdecl;external gliblib name 'g_io_channel_read';
function g_io_channel_write(channel:PGIOChannel; buf:Pgchar; count:gsize; bytes_written:Pgsize):TGIOError;cdecl;external gliblib name 'g_io_channel_write';
function g_io_channel_seek(channel:PGIOChannel; offset:gint64; _type:TGSeekType):TGIOError;cdecl;external gliblib name 'g_io_channel_seek';
procedure g_io_channel_close(channel:PGIOChannel);cdecl;external gliblib name 'g_io_channel_close';

{end of DEPRECATED functions}

function g_io_channel_shutdown(channel:PGIOChannel; flush:gboolean; err:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_shutdown';
function g_io_add_watch_full(channel:PGIOChannel; priority:gint; condition:TGIOCondition; func:TGIOFunc; user_data:gpointer;
                             notify:TGDestroyNotify):guint;cdecl;external gliblib name 'g_io_add_watch_full';
function g_io_create_watch(channel:PGIOChannel; condition:TGIOCondition):PGSource;cdecl;external gliblib name 'g_io_create_watch';
function g_io_add_watch(channel:PGIOChannel; condition:TGIOCondition; func:TGIOFunc; user_data:gpointer):guint;cdecl;external gliblib name 'g_io_add_watch';

{ character encoding conversion involved functions. }

procedure g_io_channel_set_buffer_size(channel:PGIOChannel; size:gsize);cdecl;external gliblib name 'g_io_channel_set_buffer_size';
function  g_io_channel_get_buffer_size(channel:PGIOChannel):gsize;cdecl;external gliblib name 'g_io_channel_get_buffer_size';
function  g_io_channel_get_buffer_condition(channel:PGIOChannel):TGIOCondition;cdecl;external gliblib name 'g_io_channel_get_buffer_condition';
function  g_io_channel_set_flags(channel:PGIOChannel; flags:TGIOFlags; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_set_flags';
function  g_io_channel_get_flags(channel:PGIOChannel):TGIOFlags;cdecl;external gliblib name 'g_io_channel_get_flags';
procedure g_io_channel_set_line_term(channel:PGIOChannel; line_term:Pgchar; length:gint);cdecl;external gliblib name 'g_io_channel_set_line_term';
function  g_io_channel_get_line_term(channel:PGIOChannel; length:Pgint):pgchar;cdecl;external gliblib name 'g_io_channel_get_line_term';
procedure g_io_channel_set_buffered(channel:PGIOChannel; buffered:gboolean);cdecl;external gliblib name 'g_io_channel_set_buffered';
function g_io_channel_get_buffered(channel:PGIOChannel):gboolean;cdecl;external gliblib name 'g_io_channel_get_buffered';
function g_io_channel_set_encoding(channel:PGIOChannel; encoding:Pgchar; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_set_encoding';
function  g_io_channel_get_encoding(channel:PGIOChannel):pgchar;cdecl;external gliblib name 'g_io_channel_get_encoding';
procedure g_io_channel_set_close_on_unref(channel:PGIOChannel; do_close:gboolean);cdecl;external gliblib name 'g_io_channel_set_close_on_unref';
function g_io_channel_get_close_on_unref(channel:PGIOChannel):gboolean;cdecl;external gliblib name 'g_io_channel_get_close_on_unref';
function g_io_channel_flush(channel:PGIOChannel; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_flush';
function g_io_channel_read_line(channel:PGIOChannel; str_return:PPgchar; length:Pgsize; terminator_pos:Pgsize; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_read_line';
function g_io_channel_read_line_string(channel:PGIOChannel; buffer:PGString; terminator_pos:Pgsize; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_read_line_string';
function g_io_channel_read_to_end(channel:PGIOChannel; str_return:PPgchar; length:Pgsize; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_read_to_end';
function g_io_channel_read_chars(channel:PGIOChannel; buf:Pgchar; count:gsize; bytes_read:Pgsize; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_read_chars';
function g_io_channel_read_unichar(channel:PGIOChannel; thechar:Pgunichar; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_read_unichar';
function g_io_channel_write_chars(channel:PGIOChannel; buf:Pgchar; count:gssize; bytes_written:Pgsize; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_write_chars';
function g_io_channel_write_unichar(channel:PGIOChannel; thechar:gunichar; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_write_unichar';
function g_io_channel_seek_position(channel:PGIOChannel; offset:gint64; _type:TGSeekType; error:PPGError):TGIOStatus;cdecl;external gliblib name 'g_io_channel_seek_position';
function g_io_channel_new_file(filename:Pgchar; mode:Pgchar; error:PPGError):PGIOChannel;cdecl;external gliblib name 'g_io_channel_new_file';


{ Error handling  }
function g_io_channel_error_quark:TGQuark;cdecl;external gliblib name 'g_io_channel_error_quark';
function g_io_channel_error_from_errno(en:gint):TGIOChannelError;cdecl;external gliblib name 'g_io_channel_error_from_errno';

function g_io_channel_unix_new(fd:longint):PGIOChannel;cdecl;external gliblib name 'g_io_channel_unix_new';
function g_io_channel_unix_get_fd(channel:PGIOChannel):gint;cdecl;external gliblib name 'g_io_channel_unix_get_fd';

{ Hook for GClosure / GSource integration. Don't touch  }
//GLIB_VAR GSourceFuncs g_io_watch_funcs;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef WINDOWS}
=======
{$ifdef WIN32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef WIN32}
>>>>>>> origin/fixes_2_2

const
  G_WIN32_MSG_HANDLE = 19981206;

procedure g_io_channel_win32_make_pollfd(channel:PGIOChannel; condition:TGIOCondition; fd:PGPollFD);cdecl;external gliblib name 'g_io_channel_win32_make_pollfd';
function g_io_channel_win32_poll(fds:PGPollFD; n_fds:gint; timeout:gint):gint;cdecl;external gliblib name 'g_io_channel_win32_poll';
function g_io_channel_win32_new_messages(hwnd:guint):PGIOChannel;cdecl;external gliblib name 'g_io_channel_win32_new_messages';
function g_io_channel_win32_new_fd(fd:gint):PGIOChannel;cdecl;external gliblib name 'g_io_channel_win32_new_fd';
function g_io_channel_win32_get_fd(channel:PGIOChannel):gint;cdecl;external gliblib name 'g_io_channel_win32_get_fd';
function g_io_channel_win32_new_socket(socket:gint):PGIOChannel;cdecl;external gliblib name 'g_io_channel_win32_new_socket';

{$endif}


{$ENDIF read_interface_rest}
// included by glib2.pas
