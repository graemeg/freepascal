{*
 * tcl.h --
 *
 * This header file describes the externally-visible facilities of the Tcl
 * interpreter.
 *
 * Translated to Pascal Copyright (c) 2002 by Max Artemev
 * aka Bert Raccoon (bert@furry.ru, bert_raccoon@freemail.ru)
 *
 *
 * Copyright (c) 1998-2000 by Scriptics Corporation.
 * Copyright (c) 1994-1998 Sun Microsystems, Inc.
 * Copyright (c) 1993-1996 Lucent Technologies.
 * Copyright (c) 1987-1994 John Ousterhout, The Regents of the
 *                         University of California, Berkeley.
 *
 * ***********************************************************************
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * ***********************************************************************
 *}
unit Tcl80;

{$MODE OBJFPC}
{$ifdef CPUI386}
  {$ASMMODE INTEL}
{$endif CPUI386}

{$IFNDEF WIN32}
  {$IFNDEF OS2}
    {$LINKLIB c}
    {$LINKLIB m}
    {$define USE_C}
  {$ENDIF}
{$ENDIF}

{$PACKRECORDS C}

{ $DEFINE USE_C}
{*
 * I recommend you to compile and link "argv.o" file to this unit.
 * If you don't have the GCC and you working on the Intel platform
 * undefine/comment the `USE_C` macro
 *}
{$IFDEF USE_C}
   {$LINK argv.o}
{$ENDIF}

interface

// M$ Win32?
{$IFDEF WIN32}
uses windows;
{$ENDIF}

const
{$IFDEF WIN32}
    TCL_LIBRARY = 'tcl80.dll';
{$ELSE}
    TCL_LIBRARY = 'tcl80';
{$ENDIF}

    TCL_DESTROYED                   = integer($DEADDEAD);  // yeah it dead ;)
    TCL_OK                          = 0;
    TCL_ERROR                       = 1;
    TCL_RETURN                      = 2;
    TCL_BREAK                       = 3;
    TCL_CONTINUE                    = 4;
    TCL_RESULT_SIZE                 = 200;
    MAX_ARGV                        = $7FFF;
    TCL_VERSION_MAJOR: integer      = 0;
    TCL_VERSION_MINOR: integer      = 0;

    TCL_NO_EVAL                     = $10000;
    TCL_EVAL_GLOBAL                 = $20000;

{* Flag values passed to variable-related procedures. *}
    TCL_GLOBAL_ONLY                 = 1;
    TCL_NAMESPACE_ONLY              = 2;
    TCL_APPEND_VALUE                = 4;
    TCL_LIST_ELEMENT                = 8;
    TCL_TRACE_READS                 = $10;
    TCL_TRACE_WRITES                = $20;
    TCL_TRACE_UNSETS                = $40;
    TCL_TRACE_DESTROYED             = $80;
    TCL_INTERP_DESTROYED            = $100;
    TCL_LEAVE_ERR_MSG               = $200;
    TCL_PARSE_PART1                 = $400;

{* Types for linked variables: *}
    TCL_LINK_INT                    = 1;
    TCL_LINK_DOUBLE                 = 2;
    TCL_LINK_BOOLEAN                = 3;
    TCL_LINK_STRING                 = 4;
    TCL_LINK_READ_ONLY              = $80;
    TCL_SMALL_HASH_TABLE            = 4;

{* Hash Table *}
    TCL_STRING_KEYS                 = 0;
    TCL_ONE_WORD_KEYS               = 1;

{* Const/enums Tcl_QueuePosition *}
// typedef enum {
    TCL_QUEUE_TAIL                  = 0;
    TCL_QUEUE_HEAD                  = 1;
    TCL_QUEUE_MARK                  = 2;
//} Tcl_QueuePosition;

// Event Flags
    TCL_DONT_WAIT                   = 1 shl 1;
    TCL_WINDOW_EVENTS               = 1 shl 2;
    TCL_FILE_EVENTS                 = 1 shl 3;
    TCL_TIMER_EVENTS                = 1 shl 4;
    TCL_IDLE_EVENTS                 = 1 shl 5;  {* WAS 0x10 ???? *}
    TCL_ALL_EVENTS                  = ($FFFFFFFF xor TCL_DONT_WAIT); // (~TCL_DONT_WAIT)

// Result type
    TCL_VOLATILE                    = 1;
    TCL_STATIC                      = 0;
    TCL_DYNAMIC                     = 3;

// Channel
    TCL_STDIN                       = 1 shl 1;
    TCL_STDOUT                      = 1 shl 2;
    TCL_STDERR                      = 1 shl 3;
    TCL_ENFORCE_MODE                = 1 shl 4;

    TCL_READABLE                    = 1 shl 1;
    TCL_WRITABLE                    = 1 shl 2;
    TCL_EXCEPTION                   = 1 shl 3;

{* POSIX *}

    EPERM                           = 1;
{* Operation not permitted; only the owner of the file (or other
 * resource) or processes with special privileges can perform the
 * operation.
 *}

    ENOENT                          = 2;
{* No such file or directory.  This is a "file doesn't exist" error
 * for ordinary files that are referenced in contexts where they are
 * expected to already exist.
 *}

    ESRCH                           = 3;
{* No process matches the specified process ID. *}

    EINTR                           = 4;
{* Interrupted function call; an asynchronous signal occurred and
 * prevented completion of the call.  When this happens, you should
 * try the call again.
 *}

    EIO                             = 5;
{* Input/output error; usually used for physical read or write errors. *}

    ENXIO                           = 6;
{* No such device or address.  The system tried to use the device
 * represented by a file you specified, and it couldn't find the
 * device.  This can mean that the device file was installed
 * incorrectly, or that the physical device is missing or not
 * correctly attached to the computer.
 *}

    E2BIG                           = 7;
{* Argument list too long; used when the arguments passed to a new
 * program being executed with one of the `exec' functions (*note
 * Executing a File::.) occupy too much memory space.  This condition
 * never arises in the GNU system.
 *}

    ENOEXEC                         = 8;
{* Invalid executable file format.  This condition is detected by the
 * `exec' functions; see *Note Executing a File::.
 *}

    EBADF                           = 9;
{* Bad file descriptor; for example, I/O on a descriptor that has been
 * closed or reading from a descriptor open only for writing (or vice
 * versa).
 *}

    ECHILD                          = 10;
{* There are no child processes.  This error happens on operations
 * that are supposed to manipulate child processes, when there aren't
 * any processes to manipulate.
 *}

    EDEADLK                         = 11;
{* Deadlock avoided; allocating a system resource would have resulted
 * in a deadlock situation.  The system does not guarantee that it
 * will notice all such situations.  This error means you got lucky
 * and the system noticed; it might just hang.  *Note File Locks::,
 * for an example.
 *}

    ENOMEM                          = 12;
{* No memory available.  The system cannot allocate more virtual
 * memory because its capacity is full.
 *}

    EACCES                          = 13;
{* Permission denied; the file permissions do not allow the attempted
 * operation.
 *}

    EFAULT                          = 14;
{* Bad address; an invalid pointer was detected.  In the GNU system,
 * this error never happens; you get a signal instead.
 *}

    ENOTBLK                         = 15;
{* A file that isn't a block special file was given in a situation
 * that requires one.  For example, trying to mount an ordinary file
 * as a file system in Unix gives this error.
 *}

    EBUSY                           = 16;
{* Resource busy; a system resource that can't be shared is already
 * in use.  For example, if you try to delete a file that is the root
 * of a currently mounted filesystem, you get this error.
 *}

    EEXIST                          = 17;
{* File exists; an existing file was specified in a context where it
 * only makes sense to specify a new file.
 *}

    EXDEV                           = 18;
{* An attempt to make an improper link across file systems was
 * detected.  This happens not only when you use `link' (*note Hard
 * Links::.) but also when you rename a file with `rename' (*note
 * Renaming Files::.).
 *}

    ENODEV                          = 19;
{* The wrong type of device was given to a function that expects a
 * particular sort of device.
 *}

    ENOTDIR                         = 20;
{* A file that isn't a directory was specified when a directory is
 * required.
 *}

    EISDIR                          = 21;
{* File is a directory; you cannot open a directory for writing, or
 * create or remove hard links to it.
 *}

    EINVAL                          = 22;
{* Invalid argument.  This is used to indicate various kinds of
 * problems with passing the wrong argument to a library function.
 *}

    EMFILE                          = 24;
{* The current process has too many files open and can't open any
 * more.  Duplicate descriptors do count toward this limit.
 *
 * In BSD and GNU, the number of open files is controlled by a
 * resource limit that can usually be increased.  If you get this
 * error, you might want to increase the `RLIMIT_NOFILE' limit or
 * make it unlimited; *note Limits on Resources::..
 *}

    ENFILE                          = 23;
{* There are too many distinct file openings in the entire system.
 * Note that any number of linked channels count as just one file
 * opening; see *Note Linked Channels::.  This error never occurs in
 * the GNU system.
 *}

    ENOTTY                          = 25;
{* Inappropriate I/O control operation, such as trying to set terminal
 * modes on an ordinary file.
 *}

    ETXTBSY                         = 26;
{* An attempt to execute a file that is currently open for writing, or
 * write to a file that is currently being executed.  Often using a
 * debugger to run a program is considered having it open for writing
 * and will cause this error.  (The name stands for "text file
 * busy".)  This is not an error in the GNU system; the text is
 * copied as necessary.
 *}

    EFBIG                           = 27;
{* File too big; the size of a file would be larger than allowed by
 * the system.
 *}

    ENOSPC                          = 28;
{* No space left on device; write operation on a file failed because
 * the disk is full.
 *}

    ESPIPE                          = 29;
{* Invalid seek operation (such as on a pipe).  *}

    EROFS                           = 30;
{* An attempt was made to modify something on a read-only file system.  *}

    EMLINK                          = 31;
{* Too many links; the link count of a single file would become too
 * large.  `rename' can cause this error if the file being renamed
 * already has as many links as it can take (*note Renaming Files::.).
 *}

    EPIPE                           = 32;
{* Broken pipe; there is no process reading from the other end of a
 * pipe.  Every library function that returns this error code also
 * generates a `SIGPIPE' signal; this signal terminates the program
 * if not handled or blocked.  Thus, your program will never actually
 * see `EPIPE' unless it has handled or blocked `SIGPIPE'.
 *}

    EDOM                            = 33;
{* Domain error; used by mathematical functions when an argument
 * value does not fall into the domain over which the function is
 * defined.
 *}

    ERANGE                          = 34;
{* Range error; used by mathematical functions when the result value
 * is not representable because of overflow or underflow.
 *}

    EAGAIN                          = 35;
{* Resource temporarily unavailable; the call might work if you try
 * again later.  The macro `EWOULDBLOCK' is another name for `EAGAIN';
 * they are always the same in the GNU C library.
 *}

    EWOULDBLOCK                     = EAGAIN;
{* In the GNU C library, this is another name for `EAGAIN' (above).
 * The values are always the same, on every operating system.
 * C libraries in many older Unix systems have `EWOULDBLOCK' as a
 * separate error code.
 *}

    EINPROGRESS                     = 36;
{* An operation that cannot complete immediately was initiated on an
 * object that has non-blocking mode selected.  Some functions that
 * must always block (such as `connect'; *note Connecting::.) never
 * return `EAGAIN'.  Instead, they return `EINPROGRESS' to indicate
 * that the operation has begun and will take some time.  Attempts to
 * manipulate the object before the call completes return `EALREADY'.
 * You can use the `select' function to find out when the pending
 * operation has completed; *note Waiting for I/O::..
 *}

    EALREADY                        = 37;
{* An operation is already in progress on an object that has
 * non-blocking mode selected.
 *}

    ENOTSOCK                        = 38;
{* A file that isn't a socket was specified when a socket is required.  *}

    EDESTADDRREQ                    = 39;
{* No default destination address was set for the socket.  You get
 * this error when you try to transmit data over a connectionless
 * socket, without first specifying a destination for the data with
 * `connect'.
 *}

    EMSGSIZE                        = 40;
{* The size of a message sent on a socket was larger than the
 * supported maximum size.
 *}

    EPROTOTYPE                      = 41;
{* The socket type does not support the requested communications
 * protocol.
 *}

    ENOPROTOOPT                     = 42;
{* You specified a socket option that doesn't make sense for the
 * particular protocol being used by the socket.  *Note Socket
 * Options::.
 *}

    EPROTONOSUPPORT                 = 43;
{* The socket domain does not support the requested communications
 * protocol (perhaps because the requested protocol is completely
 * invalid.) *Note Creating a Socket::.
 *}

    ESOCKTNOSUPPORT                 = 44;
{* The socket type is not supported.  *}

    EOPNOTSUPP                      = 45;
{* The operation you requested is not supported.  Some socket
 * functions don't make sense for all types of sockets, and others
 * may not be implemented for all communications protocols.  In the
 * GNU system, this error can happen for many calls when the object
 * does not support the particular operation; it is a generic
 * indication that the server knows nothing to do for that call.
 *}

    EPFNOSUPPORT                    = 46;
{* The socket communications protocol family you requested is not
 * supported.
 *}

    EAFNOSUPPORT                    = 47;
{* The address family specified for a socket is not supported; it is
 * inconsistent with the protocol being used on the socket.  *Note
 * Sockets::.
 *}

    EADDRINUSE                      = 48;
{* The requested socket address is already in use.  *Note Socket
 * Addresses::.
 *}

    EADDRNOTAVAIL                   = 49;
{* The requested socket address is not available; for example, you
 * tried to give a socket a name that doesn't match the local host
 * name.  *Note Socket Addresses::.
 *}

    ENETDOWN                        = 50;
{* A socket operation failed because the network was down.  *}

    ENETUNREACH                     = 51;
{* A socket operation failed because the subnet containing the remote
 * host was unreachable.
 *}

    ENETRESET                       = 52;
{* A network connection was reset because the remote host crashed.  *}

    ECONNABORTED                    = 53;
{* A network connection was aborted locally. *}

    ECONNRESET                      = 54;
{* A network connection was closed for reasons outside the control of
 * the local host, such as by the remote machine rebooting or an
 * unrecoverable protocol violation.
 *}

    ENOBUFS                         = 55;
{* The kernel's buffers for I/O operations are all in use.  In GNU,
 * this error is always synonymous with `ENOMEM'; you may get one or
 * the other from network operations.
 *}

    EISCONN                         = 56;
{* You tried to connect a socket that is already connected.  *Note
 * Connecting::.
 *}

    ENOTCONN                        = 57;
{* The socket is not connected to anything.  You get this error when
 * you try to transmit data over a socket, without first specifying a
 * destination for the data.  For a connectionless socket (for
 * datagram protocols, such as UDP), you get `EDESTADDRREQ' instead.
 *}

    ESHUTDOWN                       = 58;
{* The socket has already been shut down.  *}

    ETOOMANYREFS                    = 59;
{* ???  *}

    ETIMEDOUT                       = 60;
{* A socket operation with a specified timeout received no response
 * during the timeout period.
 *}

    ECONNREFUSED                    = 61;
{* A remote host refused to allow the network connection (typically
 * because it is not running the requested service).
 *}

    ELOOP                           = 62;
{* Too many levels of symbolic links were encountered in looking up a
 * file name.  This often indicates a cycle of symbolic links.
 *}

    ENAMETOOLONG                    = 63;
{* Filename too long (longer than `PATH_MAX'; *note Limits for
 * Files::.) or host name too long (in `gethostname' or
 * `sethostname'; *note Host Identification::.).
 *}

    EHOSTDOWN                       = 64;
{* The remote host for a requested network connection is down.  *}

    EHOSTUNREACH                    = 65;
{* The remote host for a requested network connection is not
 * reachable.
 *}

    ENOTEMPTY                       = 66;
{* Directory not empty, where an empty directory was expected.
 * Typically, this error occurs when you are trying to delete a
 * directory.
 *}

    EPROCLIM                        = 67;
{* This means that the per-user limit on new process would be
 * exceeded by an attempted `fork'.  *Note Limits on Resources::, for
 * details on the `RLIMIT_NPROC' limit.
 *}

    EUSERS                          = 68;
{* The file quota system is confused because there are too many users.  *}

    EDQUOT                          = 69;
{* The user's disk quota was exceeded.  *}

    ESTALE                          = 70;
{* Stale NFS file handle.  This indicates an internal confusion in
 * the NFS system which is due to file system rearrangements on the
 * server host.  Repairing this condition usually requires unmounting
 * and remounting the NFS file system on the local host.
 *}

    EREMOTE                         = 71;
{* An attempt was made to NFS-mount a remote file system with a file
 * name that already specifies an NFS-mounted file.  (This is an
 * error on some operating systems, but we expect it to work properly
 * on the GNU system, making this error code impossible.)
 *}

    EBADRPC                         = 72;
{* ???  *}
    ERPCMISMATCH                    = 73;
{* ???  *}
    EPROGUNAVAIL                    = 74;
{* ???  *}
    EPROGMISMATCH                   = 75;
{* ???  *}
    EPROCUNAVAIL                    = 76;
{* ???  *}
    ENOLCK                          = 77;
{* No locks available.  This is used by the file locking facilities;
 * see *Note File Locks::.  This error is never generated by the GNU
 * system, but it can result from an operation to an NFS server
 * running another operating system.
 *}

    ENOSYS                          = 78;
{* Function not implemented.  Some functions have commands or options
 * defined that might not be supported in all implementations, and
 * this is the kind of error you get if you request them and they are
 * not supported.
 *}

    EFTYPE                          = 79;
{* Inappropriate file type or format.  The file was the wrong type
 * for the operation, or a data file had the wrong format.
 * On some systems `chmod' returns this error if you try to set the
 * sticky bit on a non-directory file; *note Setting Permissions::..
 *}


type
    PPChar          = ^PChar;
    Tcl_Argv        = PPChar;
    Tcl_ClientData  = pointer;
    Tcl_FreeProc    = procedure(block : pointer); cdecl;
    PTcl_Interp     = ^Tcl_Interp;

    Tcl_Interp = packed record
        result  : PChar; {* Do not access this directly. Use
                          * Tcl_GetStringResult since result
                          * may be pointing to an object
                          *}
        freeProc : Tcl_FreeProc;
        errorLine: integer;
    end;

{*  Event Definitions  *}
    TTcl_EventSetupProc = procedure(clientData: Tcl_ClientData; flags: integer); cdecl;
    TTcl_EventCheckProc = TTcl_EventSetupProc;

    PTcl_Event          = ^Tcl_Event;
    TTcl_EventProc      = function(evPtr: PTcl_Event; flags: integer): integer; cdecl;

    Tcl_Event = packed record
        proc      : TTcl_EventProc;
        nextPtr   : PTcl_Event;
        ClientData: TObject;    {* ClientData is just pointer.*}
    end;


    PTcl_Time = ^Tcl_Time;
    Tcl_Time = packed record
        sec: longInt;           { * Seconds. * }
        usec: longInt;          { * Microseconds. * }
    end;

    Tcl_TimerToken  = pointer;
    PInteger        = ^integer;

    PTcl_HashTable  = pointer;
    PTcl_HashEntry  = ^Tcl_HashEntry;
    PPTcl_HashEntry = ^PTcl_HashEntry;

    Tcl_HashEntry  = packed record
        nextPtr    : PTcl_HashEntry;
        tablePtr   : PTcl_HashTable;
        bucketPtr  : PPTcl_HashEntry;
        clientData : Tcl_ClientData;
        key        : array[0..3] of Char;
    end;
{      case key: integer of
        0: (oneWordValue: pChar);
         1: (words      : pInteger);
         2: (str        : pChar);
}
    Tcl_HashFindProc   = function(tablePtr: PTcl_HashTable; key: PChar): PTcl_HashEntry; cdecl;
    Tcl_HashCreateProc = function(tablePtr: PTcl_HashTable; key: PChar; newPtr: PInteger): PTcl_HashEntry; cdecl;

    PHashTable = ^Tcl_HashTable;
    Tcl_HashTable = packed record
        buckets         : ppTcl_HashEntry;
        staticBuckets   : array[0..TCL_SMALL_HASH_TABLE - 1] of PTcl_HashEntry;
        numBuckets      : integer;
        numEntries      : integer;
        rebuildSize     : integer;
        downShift       : integer;
        mask            : integer;
        keyType         : integer;
        findProc        : Tcl_HashFindProc;
        createProc      : Tcl_HashCreateProc;
    end;

    PTcl_HashSearch = ^Tcl_HashSearch;
    Tcl_HashSearch = packed record
         tablePtr    : PTcl_HashTable;
         nextIndex   : integer;
         nextEntryPtr: PTcl_HashEntry;
    end;

    TTclAppInitProc         = function(interp: pTcl_Interp): integer; cdecl;
    TTclPackageInitProc     = function(interp: pTcl_Interp): integer; cdecl;
    TTclCmdProc             = function(clientData : Tcl_ClientData; interp : pTcl_Interp; argc: integer; argv : Tcl_Argv): integer; cdecl;
    TTclVarTraceProc        = function (clientData: Tcl_ClientData; interp: pTcl_Interp;
                                        varName: PChar; elemName: PChar; flags: integer): PChar; cdecl;
    TTclFreeProc            = procedure(block: pointer); cdecl;
    TTclInterpDeleteProc    = procedure(clientData: Tcl_ClientData; interp: pTcl_Interp); cdecl;
    TTclCmdDeleteProc       = procedure(clientData: Tcl_ClientData); cdecl;
    TTclNamespaceDeleteProc = procedure(clientData: Tcl_ClientData); cdecl;


const
    TCL_DSTRING_STATIC_SIZE = 200;

type
    PTcl_DString = ^Tcl_DString;
    Tcl_DString = packed record
        str        : PChar;
        length     : integer;
        spaceAvl   : integer;
        staticSpace: array[0..TCL_DSTRING_STATIC_SIZE - 1] of char;
    end;

    PTcl_Channel = ^Tcl_Channel;
    Tcl_Channel = packed record
    end;

    TTclDriverBlockModeProc     = function(instanceData: Tcl_ClientData; mode: integer): integer; cdecl;
    TTclDriverCloseProc         = function(instanceData: Tcl_ClientData; interp: PTcl_Interp): integer; cdecl;
    TTclDriverInputProc         = function(instanceData: Tcl_ClientData; buf: PChar; toRead: integer;
                                           errorCodePtr: PInteger): integer; cdecl;
    TTclDriverOutputProc        = function(instanceData: Tcl_ClientData; buf: PChar; toWrite: integer;
                                           errorCodePtr: PInteger): integer; cdecl;
    TTclDriverSeekProc          = function(instanceData: Tcl_ClientData; offset: longint; mode: integer;
                                           errorCodePtr: PInteger): integer; cdecl;
    TTclDriverSetOptionProc     = function(instanceData: Tcl_ClientData; interp: PTcl_Interp; optionName: PChar;
                                           value: PChar): integer; cdecl;
    TTclDriverGetOptionProc     = function(instanceData: Tcl_ClientData; interp: pTcl_Interp; optionName: PChar;
                                           dsPtr: PTcl_DString): integer; cdecl;
    TTclDriverWatchProc         = procedure(instanceData: Tcl_ClientData; mask: integer); cdecl;
    TTclDriverGetHandleProc     = function(instanceData: Tcl_ClientData; direction: integer;
                                           var handlePtr: Tcl_ClientData): integer; cdecl;
    PTcl_ChannelType = ^Tcl_ChannelType;
    Tcl_ChannelType  = packed record
        typeName     : PChar;
        blockModeProc: TTclDriverBlockModeProc;
        closeProc    : TTclDriverCloseProc;
        inputProc    : TTclDriverInputProc;
        ouputProc    : TTclDriverOutputProc;
        seekProc     : TTclDriverSeekProc;
        setOptionProc: TTclDriverSetOptionProc;
        getOptionProc: TTclDriverGetOptionProc;
        watchProc    : TTclDriverWatchProc;
        getHandleProc: TTclDriverGetHandleProc;
    end;

    TTclChannelProc = procedure(clientData: Tcl_ClientData; mask: integer); cdecl;

    PTcl_Obj = ^Tcl_Obj;
    PPTcl_Obj = ^PTcl_Obj;
    Tcl_Obj = packed record
        refCount: integer;
        // ...
    end;

    TTclObjCmdProc      = function(clientData: Tcl_ClientData; interp: PTcl_Interp; objc: integer; PPObj: PPTcl_Obj): integer; cdecl;

    PTcl_Namespace = ^Tcl_Namespace;
    Tcl_Namespace = packed record
        name      : pchar;
        fullName  : PChar;
        clientData: Tcl_ClientData;
        deleteProc: TTclNamespaceDeleteProc;
        parentPtr : PTcl_Namespace;
    end;

    PTcl_CallFrame = ^Tcl_CallFrame;
    Tcl_CallFrame  = packed record
        nsPtr  : PTcl_Namespace;
        dummy1 : integer;
        dummy2 : integer;
        dummy3 : PChar;
        dummy4 : PChar;
        dummy5 : PChar;
        dummy6 : integer;
        dummy7 : PChar;
        dummy8 : PChar;
        dummy9 : integer;
        dummy10: PChar;
    end;

    PTcl_CmdInfo = ^Tcl_CmdInfo;
    Tcl_CmdInfo  = packed record
        isNativeObjectProc: integer;
        objProc           : TTclObjCmdProc;
        objClientData     : Tcl_ClientData;
        proc              : TTclCmdProc;
        clientData        : Tcl_ClientData;
        deleteProc        : TTclCmdDeleteProc;
        deleteData        : Tcl_ClientData;
        namespacePtr      : pTcl_Namespace;
    end;

    pTcl_Command = ^Tcl_Command;
    Tcl_Command = packed record
    end;

{       hPtr            : pTcl_HashEntry;
        nsPtr           : pTcl_Namespace;
        refCount        : integer;
        isCmdEpoch      : integer;
        compileProc     : pointer;
        objProc         : pointer;
        objClientData   : Tcl_ClientData;
        proc            : pointer;
        clientData      : Tcl_ClientData;
        deleteProc      : TTclCmdDeleteProc;
        deleteData      : Tcl_ClientData;
        deleted         : integer;
        importRefPtr    : pointer;
}

type
    ulong      = longint;
    uint       = integer;
    bool       = longbool;

    TTclPanicProc          = procedure(fmt, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8: PChar); cdecl; // 1/15/97 orig. Tcl style
    TTclClientDataProc     = procedure (clientData: Tcl_ClientData); cdecl;
    TTclIdleProc           = procedure (clientData: Tcl_ClientData); cdecl;
    TTclTimerProc          = TTclIdleProc;
    TTclCreateCloseHandler = procedure  (channel: pTcl_Channel; proc: TTclClientDataProc; clientData: Tcl_ClientData); cdecl;
    TTclDeleteCloseHandler = TTclCreateCloseHandler;
    TTclEventDeleteProc    = function(evPtr: pTcl_Event; clientData: Tcl_ClientData): integer; cdecl;

    function Tcl_Alloc(size: Cardinal): PChar; cdecl; external TCL_LIBRARY;
    function Tcl_CreateInterp : pTcl_Interp; cdecl; external TCL_LIBRARY;
    procedure Tcl_DeleteInterp(interp: pTcl_Interp); cdecl; external TCL_LIBRARY;
    procedure Tcl_ResetResult(interp: pTcl_Interp); cdecl; external TCL_LIBRARY;
    function Tcl_Eval(interp: pTcl_Interp; script : PChar):integer; cdecl; external TCL_LIBRARY;
    function Tcl_EvalFile(interp: pTcl_Interp; filename: PChar):integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_AddErrorInfo(interp: pTcl_Interp; message: PChar); cdecl; external TCL_LIBRARY;
    procedure Tcl_BackgroundError(interp: pTcl_Interp); cdecl; external TCL_LIBRARY;
    function Tcl_CreateCommand(interp: pTcl_Interp; name: PChar; cmdProc: TTclCmdProc;
                               clientData: Tcl_ClientData; deleteProc: TTclCmdDeleteProc): pTcl_Command; cdecl; external TCL_LIBRARY;

    function  Tcl_DeleteCommand(interp: pTcl_Interp; name: PChar): integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_CallWhenDeleted(interp: pTcl_Interp; proc: TTclInterpDeleteProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    procedure Tcl_DontCallWhenDeleted(interp: pTcl_Interp; proc: TTclInterpDeleteProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    function  Tcl_CommandComplete(cmd: PChar): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_LinkVar(interp: pTcl_Interp; varName: PChar; var addr; typ: integer): integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_UnlinkVar(interp: pTcl_Interp; varName: PChar); cdecl; external TCL_LIBRARY;
    function  Tcl_TraceVar(interp: pTcl_Interp; varName: PChar; flags: integer; proc: TTclVarTraceProc;
                           clientData: Tcl_ClientData): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_TraceVar2(interp: pTcl_Interp; varName: PChar; elemName: PChar; flags : integer; proc: TTclVarTraceProc;
                            clientData: Tcl_ClientData): integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_UntraceVar(interp: pTcl_Interp; varName: PChar; flags: integer;
                             proc: TTclVarTraceProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    procedure Tcl_UntraceVar2(interp: pTcl_Interp; varName: PChar; elemName: PChar; flags: integer;
                              proc: TTclVarTraceProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    function  Tcl_GetVar(interp: pTcl_Interp; varName: PChar; flags: integer): PChar; cdecl; external TCL_LIBRARY;
    function  Tcl_GetVar2(interp: pTcl_Interp; varName: PChar; elemName: PChar; flags: integer): PChar; cdecl; external TCL_LIBRARY;
    function  Tcl_SetVar(interp: pTcl_Interp; varName: PChar; newValue: PChar; flags: integer): PChar; cdecl; external TCL_LIBRARY;
    function  Tcl_SetVar2(interp: pTcl_Interp; varName: PChar; elemName: PChar; newValue: PChar; flags: integer): PChar; cdecl; external TCL_LIBRARY;
    function  Tcl_UnsetVar(interp: pTcl_Interp; varName: PChar; flags: integer): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_UnsetVar2(interp: pTcl_Interp; varName: PChar; elemName: PChar; flags: integer): integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_SetResult(interp: pTcl_Interp; newValue: PChar; freeProc: TTclFreeProc); cdecl; external TCL_LIBRARY;
    function  Tcl_FirstHashEntry(hashTbl: pTcl_HashTable; var searchInfo: Tcl_HashSearch): pTcl_HashEntry; cdecl; external TCL_LIBRARY;
    function  Tcl_NextHashEntry(var searchInfo: Tcl_HashSearch): pTcl_HashEntry; cdecl; external TCL_LIBRARY;
    procedure Tcl_InitHashTable(hashTbl: pTcl_HashTable; keyType: integer); cdecl; external TCL_LIBRARY;
    function  Tcl_StringMatch(str: PChar; pattern: PChar): integer; cdecl; external TCL_LIBRARY;
    function  _Tcl_GetHashKey(hashTbl: pTcl_HashTable; hashEntry: pTcl_HashEntry): PChar; cdecl;
    function  Tcl_GetErrno:integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_SetErrno(val: integer); cdecl; external TCL_LIBRARY;


    procedure Tcl_SetPanicProc(proc: TTclPanicProc); cdecl; external TCL_LIBRARY;
    function  Tcl_PkgProvide(interp: pTcl_Interp; name: PChar; version: PChar): integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_StaticPackage(interp: pTcl_Interp; pkgName: PChar; initProc: TTclPackageInitProc;
                                safeInitProc: TTclPackageInitProc); cdecl; external TCL_LIBRARY;
    procedure Tcl_CreateEventSource(setupProc: TTcl_EventSetupProc;
                                    checkProc: TTcl_EventCheckProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    procedure Tcl_DeleteEventSource(setupProc: TTcl_EventSetupProc;
                                    checkProc: TTcl_EventCheckProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    procedure Tcl_QueueEvent(evPtr: pTcl_Event; pos: integer); cdecl; external TCL_LIBRARY;
    procedure Tcl_SetMaxBlockTime(timePtr: pTcl_Time); cdecl; external TCL_LIBRARY;

    procedure Tcl_DeleteEvents(proc: TTclEventDeleteProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    function  Tcl_DoOneEvent(flags: integer): integer; cdecl; external TCL_LIBRARY;


    procedure Tcl_DoWhenIdle(proc: TTclIdleProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    procedure Tcl_CancelIdleCall(proc: TTclIdleProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    function  Tcl_CreateTimerHandler(milliseconds: integer; proc: TTclTimerProc;
                                     clientData: Tcl_ClientData): Tcl_TimerToken; cdecl; external TCL_LIBRARY;
    procedure Tcl_DeleteTimerHandler(token: Tcl_TimerToken); cdecl; external TCL_LIBRARY;
//    procedure Tcl_CreateModalTimeout(milliseconds: integer; proc: TTclTimerProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
//    procedure Tcl_DeleteModalTimeout(proc: TTclTimerProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;

    function  Tcl_SplitList(interp: pTcl_Interp; list: PChar; var argcPtr: integer; var argvPtr: Tcl_Argv): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_Merge(argc: integer; argv: Tcl_Argv):PChar; cdecl; external TCL_LIBRARY;
    procedure Tcl_Free( ptr: PChar ); cdecl; external TCL_LIBRARY;
    function  Tcl_Init(interp: pTcl_Interp): integer; cdecl; external TCL_LIBRARY;
//    procedure Tcl_InterpDeleteProc(clientData: Tcl_ClientData; interp: pTcl_Interp); cdecl; external TCL_LIBRARY;
    function  Tcl_GetAssocData(interp:pTcl_Interp; key: PChar; var proc: TTclInterpDeleteProc): Tcl_ClientData; cdecl; external TCL_LIBRARY;
    procedure Tcl_DeleteAssocData(interp: pTcl_Interp; key: PChar); cdecl; external TCL_LIBRARY;
    procedure Tcl_SetAssocData(interp: pTcl_Interp; key: PChar; proc: TTclInterpDeleteProc;
                               clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    function  Tcl_IsSafe(interp: pTcl_Interp): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_MakeSafe(interp: pTcl_Interp): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_CreateSlave(interp: pTcl_Interp; slaveName: PChar; isSafe: integer): pTcl_Interp; cdecl; external TCL_LIBRARY;
    function  Tcl_GetSlave(interp: pTcl_Interp; slaveName: PChar): pTcl_Interp; cdecl; external TCL_LIBRARY;
    function  Tcl_GetMaster(interp: pTcl_Interp): pTcl_Interp; cdecl; external TCL_LIBRARY;
    function  Tcl_GetInterpPath(askingInterp: pTcl_Interp; slaveInterp: pTcl_Interp): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_CreateAlias(slaveInterp: pTcl_Interp; srcCmd: PChar; targetInterp: pTcl_Interp; targetCmd: PChar;
                             argc: integer; argv: Tcl_Argv): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_GetAlias(interp: pTcl_Interp; srcCmd: PChar; var targetInterp: pTcl_Interp; var targetCmd: PChar;
                          var argc: integer; var argv: Tcl_Argv): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_ExposeCommand(interp: pTcl_Interp; hiddenCmdName: PChar; cmdName: PChar): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_HideCommand(interp: pTcl_Interp; cmdName: PChar; hiddenCmdName: PChar): integer; cdecl; external TCL_LIBRARY;


    procedure Tcl_EventuallyFree(clientData: Tcl_ClientData; freeProc: TTclFreeProc); cdecl; external TCL_LIBRARY;
    procedure Tcl_Preserve(clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    procedure Tcl_Release(clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    function  Tcl_InterpDeleted(interp: pTcl_Interp): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_GetCommandInfo(interp: pTcl_Interp; cmdName: PChar; var info: Tcl_CmdInfo): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_SetCommandInfo(interp: pTcl_Interp; cmdName: PChar; var info: Tcl_CmdInfo): integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_FindExecutable(path: PChar); cdecl; external TCL_LIBRARY;
    function  Tcl_GetStringResult(interp: pTcl_Interp): PChar; cdecl; external TCL_LIBRARY; //v1.0
    function  Tcl_FindCommand(interp: pTcl_Interp; cmdName: PChar;
                             contextNsPtr: pTcl_Namespace; flags: integer): Tcl_Command; cdecl; external TCL_LIBRARY; //v1.0
    function  Tcl_DeleteCommandFromToken(interp: pTcl_Interp; cmd: pTcl_Command): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_CreateNamespace(interp: pTcl_Interp; name: PChar; clientData: Tcl_ClientData;
                                  deleteProc: TTclNamespaceDeleteProc): pTcl_Namespace; cdecl; external TCL_LIBRARY; //v1.0
    procedure Tcl_DeleteNamespace(namespacePtr: pTcl_Namespace); cdecl; external TCL_LIBRARY;
    function  Tcl_FindNamespace(interp: pTcl_Interp; name: PChar; contextNsPtr: pTcl_Namespace; flags: integer): pTcl_Namespace; cdecl; external TCL_LIBRARY;
    function  Tcl_Export(interp: pTcl_Interp; namespacePtr: pTcl_Namespace; pattern: PChar;
                         resetListFirst: integer): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_Import(interp: pTcl_Interp; namespacePtr: pTcl_Namespace; pattern: PChar;
                         allowOverwrite: integer): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_GetCurrentNamespace(interp: pTcl_Interp): pTcl_Namespace; cdecl; external TCL_LIBRARY;
    function  Tcl_GetGlobalNamespace(interp: pTcl_Interp): pTcl_Namespace; cdecl; external TCL_LIBRARY;
    function  Tcl_PushCallFrame(interp: pTcl_Interp; var callFramePtr: Tcl_CallFrame;
                                namespacePtr: pTcl_Namespace; isProcCallFrame: integer): integer; cdecl; external TCL_LIBRARY;

    procedure Tcl_PopCallFrame(interp: pTcl_Interp); cdecl; external TCL_LIBRARY;
    function  Tcl_VarEval(interp: pTcl_Interp; args: array of const):integer; cdecl; external TCL_LIBRARY;

{* For TkConsole.c *}
    function  Tcl_RecordAndEval(interp: pTcl_Interp; cmd: PChar; flags: integer): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_GlobalEval(interp: pTcl_Interp; command: PChar): integer; cdecl; external TCL_LIBRARY;
    procedure Tcl_DStringFree(dsPtr: pTcl_DString); cdecl; external TCL_LIBRARY;
    function  Tcl_DStringAppend(dsPtr: pTcl_DString; str: PChar; len: integer): PChar; cdecl; external TCL_LIBRARY;
    function  Tcl_DStringAppendElement(dsPtr: pTcl_DString; str: PChar): PChar; cdecl; external TCL_LIBRARY;
    procedure Tcl_DStringInit(dsPtr: pTcl_DString); cdecl; external TCL_LIBRARY;
    procedure Tcl_AppendResult(interp: pTcl_Interp; args: array of const); cdecl; external TCL_LIBRARY; // actually a "C" var array
    procedure Tcl_SetStdChannel(channel: pTcl_Channel; typ: integer); cdecl; external TCL_LIBRARY;
    function  Tcl_SetChannelOption(interp: pTcl_Interp; chan: pTcl_Channel; optionName: PChar; newValue: PChar): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_GetChannelOption(interp: pTcl_Interp; chan: pTcl_Channel; optionName: PChar; dsPtr: pTcl_DString): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_CreateChannel(typePtr: pTcl_ChannelType; chanName: PChar;
                                instanceData: Tcl_ClientData; mask: integer):pTcl_Channel; cdecl; external TCL_LIBRARY;
    procedure Tcl_RegisterChannel(interp: pTcl_Interp; channel: pTcl_Channel); cdecl; external TCL_LIBRARY;
    function  Tcl_UnregisterChannel(interp: pTcl_Interp; channel: pTcl_Channel): integer; cdecl; external TCL_LIBRARY;

    procedure Tcl_CreateChannelHandler(chan: pTcl_Channel; mask: integer; proc: TTclChannelProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    function Tcl_GetChannel(interp: pTcl_Interp; chanName: PChar; modePtr: pInteger): pTcl_Channel; cdecl; external TCL_LIBRARY;
    function Tcl_GetStdChannel(typ: integer): pTcl_Channel; cdecl; external TCL_LIBRARY;
    function Tcl_Gets(chan: pTcl_Channel; dsPtr: pTcl_DString): integer; cdecl; external TCL_LIBRARY;
    function Tcl_Write(chan: pTcl_Channel; s: PChar; slen: integer): integer; cdecl; external TCL_LIBRARY;
    function Tcl_Flush(chan: pTcl_Channel): integer; cdecl; external TCL_LIBRARY;
//    TclWinLoadLibrary      = function(name: PChar): HMODULE; cdecl; external TCL_LIBRARY;
    procedure Tcl_CreateExitHandler(proc: TTclClientDataProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    procedure Tcl_DeleteExitHandler(proc: TTclClientDataProc; clientData: Tcl_ClientData); cdecl; external TCL_LIBRARY;
    function  Tcl_GetStringFromObj(pObj: pTcl_Obj; pLen: pInteger): PChar; cdecl; external TCL_LIBRARY;
    function  Tcl_CreateObjCommand(interp: pTcl_Interp; name: PChar; cmdProc: TTclObjCmdProc;
                                   clientData: Tcl_ClientData; deleteProc: TTclCmdDeleteProc): pTcl_Command; cdecl; external TCL_LIBRARY;
    function  Tcl_NewStringObj(bytes: PChar; len: integer): pTcl_Obj; cdecl; external TCL_LIBRARY;
//    procedure TclFreeObj(pObj: pTcl_Obj); cdecl; external TCL_LIBRARY;
    function  Tcl_EvalObj(interp: pTcl_Interp; pObj: pTcl_Obj): integer; cdecl; external TCL_LIBRARY;
    function  Tcl_GlobalEvalObj(interp: pTcl_Interp; pObj: pTcl_Obj): integer; cdecl; external TCL_LIBRARY;
    function  TclRegComp(exp: PChar): pointer; cdecl; external TCL_LIBRARY;
    function  TclRegExec(prog: pointer; str: PChar; start: PChar): integer; cdecl; external TCL_LIBRARY;
    procedure TclRegError(msg: PChar); cdecl; external TCL_LIBRARY;
    function  TclGetRegError: PChar; cdecl; external TCL_LIBRARY;
    procedure Tcl_RegExpRange(prog: pointer; index: integer; var head: PChar; var tail: PChar); cdecl; external TCL_LIBRARY;

// C Macro Emulation
    function  Tcl_GetCommandTable(interp: pTcl_Interp): pHashTable;
    function  Tcl_CreateHashEntry(tablePtr: pTcl_HashTable; key: PChar; newPtr: pInteger): pTcl_HashEntry;
    function  Tcl_FindHashEntry(tablePtr: pTcl_HashTable; key: PChar): pTcl_HashEntry;
    procedure Tcl_SetHashValue(h: pTcl_HashEntry; clientData: Tcl_ClientData);
    function  Tcl_GetHashValue(h: pTcl_HashEntry): Tcl_ClientData;
    procedure Tcl_IncrRefCount(pObj: pTcl_Obj); cdecl;
    procedure Tcl_DecrRefCount(pObj: pTcl_Obj); cdecl;
    function  Tcl_IsShared(pObj: pTcl_Obj): integer; cdecl;

{$IFDEF USE_C}
    function ArgvItem(argv: PPChar; idx: integer): PChar; cdecl; external; // argv.c must be compiled by GCC
{$ELSE}
    function ArgvItem(argv: PPChar; idx: integer): PChar; cdecl;
{$ENDIF}


implementation
uses SysUtils {, Classes};


// Macro emulation
function Tcl_CreateHashEntry(tablePtr: pTcl_HashTable; key: PChar; newPtr: pInteger): pTcl_HashEntry;
begin
     result := pHashTable(tablePtr)^.createProc(tablePtr, key, newPtr);
end;

function Tcl_FindHashEntry(tablePtr: pTcl_HashTable; key: PChar): pTcl_HashEntry;
begin
     result := pHashTable(tablePtr)^.findProc(tablePtr, key);
end;

procedure Tcl_SetHashValue(h: pTcl_HashEntry; clientData: Tcl_ClientData);
begin
     h^.clientData := clientData;
end;

function Tcl_GetHashValue(h: pTcl_HashEntry): Tcl_ClientData;
begin
     result := h^.clientData;
end;


function _Tcl_GetHashKey(hashTbl: pTcl_HashTable; hashEntry: pTcl_HashEntry): PChar; cdecl;
begin
     if (hashTbl = nil) or (hashEntry = nil) then
        result := nil
     else if pHashTable(hashTbl)^.keyType = 1 then
        result :=  PChar(pptrInt(@(hashEntry^.key[0]))^)
     else
        result := hashEntry^.key;
end;


procedure Tcl_IncrRefCount(pObj: pTcl_Obj); cdecl;
begin
     inc(pObj^.refCount);
end;

procedure Tcl_DecrRefCount(pObj: pTcl_Obj); cdecl;
begin
     dec(pObj^.refCount);
     if pObj^.refCount <= 0 then
        FreeMem(pObj);
end;

function Tcl_IsShared(pObj: pTcl_Obj): integer; cdecl;
begin
     if pObj^.refCount > 0 then
        result := 1
     else
        result := 0;
end;


function Tcl_GetCommandTable(interp: pTcl_Interp): pHashTable;
begin
     if interp = nil then
        result := nil
     else if TCL_VERSION_MAJOR >= 8 then // pretty sure it happened in this version
        result := pHashTable(longint(interp) + sizeof(Tcl_Interp) + sizeof(pointer))
     else
        result := pHashTable(longint(interp) + sizeof(Tcl_Interp));
end;

{$IFNDEF USE_C}
{*
 *  Use this if you don't have the C compiler and you're on
 *  the Intel platform.
 *  Otherwise define `USE_C` macro.
 *}
function ArgvItem(argv: PPChar; idx: integer): PChar; cdecl;
var
   Buf: LongWord;
begin
     asm
        MOV   EAX,idx             //* index please
        MOV   EDX,[argv]          //* gotcha argv^
        MOV   EAX,[EDX + EAX*4]   //* PChar is 32bit pointer, so EAX*4 its offset for
                                  //* one item in array.
                                  //* gotcha something like this: (argv^)^[idx]
                                  //*
        MOV   Buf,EAX
     end;
     ArgvItem:=PChar(Buf);
end;
{$ENDIF}

end.
