// included by glib2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef WINDOWS}
=======
{$ifdef WIN32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef WIN32}
>>>>>>> origin/fixes_2_2

  { On native Win32, directory separator is the backslash, and search path
     separator is the semicolon.
    }

    const
       G_DIR_SEPERATOR   = '\';
       G_DIR_SEPARATOR_S = '\';
       G_SEARCHPATH_SEPARATOR = ';';
       G_SEARCHPATH_SEPARATOR_S = ';';
{$else}
    { !WIN32  }
    { Unix  }

    const
       G_DIR_SEPARATOR = '/';
       G_DIR_SEPARATOR_S = '/';
       G_SEARCHPATH_SEPARATOR = ':';
       G_SEARCHPATH_SEPARATOR_S = ':';
{$endif}


    function  g_get_user_name:Pgchar;cdecl;external gliblib name 'g_get_user_name';

    function  g_get_real_name:Pgchar;cdecl;external gliblib name 'g_get_real_name';

    function  g_get_home_dir:Pgchar;cdecl;external gliblib name 'g_get_home_dir';

    function  g_get_tmp_dir:Pgchar;cdecl;external gliblib name 'g_get_tmp_dir';

    function  g_get_prgname:Pgchar;cdecl;external gliblib name 'g_get_prgname';

    procedure g_set_prgname(prgname:Pgchar);cdecl;external gliblib name 'g_set_prgname';


    type
       PGDebugKey = ^TGDebugKey;
       TGDebugKey = record
            key : Pgchar;
            value : guint;
         end;

    { Miscellaneous utility functions
      }

    function g_parse_debug_string(_string:Pgchar; keys:PGDebugKey; nkeys:guint):guint;cdecl;external gliblib name 'g_parse_debug_string';

{$IFNDEF KYLIX}
    function g_snprintf(_string:Pgchar; n:gulong; format:Pgchar; args:array of const):gint;cdecl;overload;external gliblib name 'g_snprintf';
    function g_snprintf(_string:Pgchar; n:gulong; format:Pgchar):gint;cdecl;overload;varargs;external gliblib name 'g_snprintf';
    function g_vsnprintf(_string:Pgchar; n:gulong; format:Pgchar; args:array of const):gint;cdecl;external gliblib name 'g_vsnprintf';
{$ELSE}
    function g_snprintf(_string:Pgchar; n:gulong; format:Pgchar):gint;varargs;cdecl;external gliblib name 'g_snprintf';
    function g_vsnprintf(_string:Pgchar; n:gulong; format:Pgchar):gint;varargs;cdecl;external gliblib name 'g_vsnprintf';
{$ENDIF}

    { Check if a file name is an absolute path  }

    function g_path_is_absolute(file_name:Pgchar):gboolean;cdecl;external gliblib name 'g_path_is_absolute';

    { In case of absolute paths, skip the root part  }
    function g_path_skip_root(file_name:Pgchar):Pgchar;cdecl;external gliblib name 'g_path_skip_root';

{DEPRECATED}
    { These two functions are deprecated and will be removed in the next
       major release of GLib. Use g_path_get_dirname/g_path_get_basename
       instead. Whatch out! The string returned by g_path_get_basename
       must be g_freed, while the string returned by g_basename must not. }

    function g_basename(file_name:Pgchar):Pgchar;cdecl;external gliblib name 'g_basename';
    function g_dirname(file_name:Pgchar):Pgchar;cdecl;external gliblib name 'g_path_get_dirname';



    { The returned strings are newly allocated with g_malloc()  }

    function g_get_current_dir:Pgchar;cdecl;external gliblib name 'g_get_current_dir';

    function g_path_get_basename(file_name:Pgchar):Pgchar;cdecl;external gliblib name 'g_path_get_basename';

    function g_path_get_dirname(file_name:Pgchar):Pgchar;cdecl;external gliblib name 'g_path_get_dirname';

    { Set the pointer at the specified location to NULL  }
    procedure g_nullify_pointer(nullify_location:Pgpointer);cdecl;external gliblib name 'g_nullify_pointer';

    { Get the codeset for the current locale  }
    { gchar   g_get_codeset    (void);  }


    { return the environment string for the variable. The returned memory
       must not be freed.  }
    function g_getenv(variable:Pgchar):Pgchar;cdecl;external gliblib name 'g_getenv';

<<<<<<< HEAD
<<<<<<< HEAD
    { we try to provide a useful equivalent for ATEXIT if it is
=======
    { we try to provide a usefull equivalent for ATEXIT if it is
>>>>>>> graemeg/fixes_2_2
=======
    { we try to provide a usefull equivalent for ATEXIT if it is
>>>>>>> origin/fixes_2_2
       not defined, but use is actually abandoned. people should
       use g_atexit() instead.
      }

    type
       TGVoidFunc = procedure;cdecl;

    { we use a GLib function as a replacement for ATEXIT, so
       the programmer is not required to check the return value
       (if there is any in the implementation) and doesn't encounter
       missing include files.
      }

    procedure g_atexit(func:TGVoidFunc);cdecl;external gliblib name 'g_atexit';

    { Look for an executable in PATH, following execvp() rules  }

    function g_find_program_in_path(_program:Pgchar):Pgchar;cdecl;external gliblib name 'g_find_program_in_path';

    { Bit tests
      }
    function g_bit_nth_lsf(mask:gulong; nth_bit:gint):gint;cdecl;external gliblib name 'g_bit_nth_lsf';

    function g_bit_nth_msf(mask:gulong; nth_bit:gint):gint;cdecl;external gliblib name 'g_bit_nth_msf';

    function g_bit_storage(number:gulong):guint;cdecl;external gliblib name 'g_bit_storage';

    { Trash Stacks
       elements need to be >= sizeof (gpointer)
      }

    type
       PPGTrashStack = ^PGTrashStack;
       PGTrashStack = ^TGTrashStack;
       TGTrashStack = record
            next : PGTrashStack;
         end;


    procedure g_trash_stack_push(stack_p:PPGTrashStack; data_p:gpointer);cdecl;external gliblib name 'g_trash_stack_push';

    function  g_trash_stack_pop(stack_p:PPGTrashStack):gpointer;cdecl;external gliblib name 'g_trash_stack_pop';

    function  g_trash_stack_peek(stack_p:PPGTrashStack):gpointer;cdecl;external gliblib name 'g_trash_stack_peek';

    function  g_trash_stack_height(stack_p:PPGTrashStack):guint;cdecl;external gliblib name 'g_trash_stack_height';

{$IFNDEF KYLIX}
    { Glib version.
       we prefix variable declarations so they can
       properly get exported in windows dlls.
      }
    var
       glib_major_version : guint;external gliblib name 'glib_major_version';
       glib_minor_version : guint;external gliblib name 'glib_minor_version';
       glib_micro_version : guint;external gliblib name 'glib_micro_version';
       glib_interface_age : guint;external gliblib name 'glib_interface_age';
       glib_binary_age    : guint;external gliblib name 'glib_binary_age';

    function GLIB_CHECK_VERSION (major, minor, micro: guint):boolean;
{$ENDIF}
{$ENDIF read_interface_rest}
