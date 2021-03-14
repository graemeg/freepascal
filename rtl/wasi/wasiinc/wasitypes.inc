{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020,2021 by the Free Pascal development team.

    Types and consts for The WebAssembly System Interface (WASI).

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
  size_t = longint;
  PPUInt8 = ^PUInt8;

  P__wasi_size_t = ^__wasi_size_t;
  __wasi_size_t = longint;
  P__wasi_filesize_t = ^__wasi_filesize_t;
  __wasi_filesize_t = UInt64;
  P__wasi_timestamp_t = ^__wasi_timestamp_t;
  __wasi_timestamp_t = UInt64;
  P__wasi_clockid_t = ^__wasi_clockid_t;
  __wasi_clockid_t = UInt32;

const
  __WASI_CLOCKID_REALTIME           = 0;
  __WASI_CLOCKID_MONOTONIC          = 1;
  __WASI_CLOCKID_PROCESS_CPUTIME_ID = 2;
  __WASI_CLOCKID_THREAD_CPUTIME_ID  = 3;

type
  P__wasi_errno_t = ^__wasi_errno_t;
  __wasi_errno_t = UInt16;

const
  __WASI_ERRNO_SUCCESS        = 0;
  __WASI_ERRNO_2BIG           = 1;
  __WASI_ERRNO_ACCES          = 2;
  __WASI_ERRNO_ADDRINUSE      = 3;
  __WASI_ERRNO_ADDRNOTAVAIL   = 4;
  __WASI_ERRNO_AFNOSUPPORT    = 5;
  __WASI_ERRNO_AGAIN          = 6;
  __WASI_ERRNO_ALREADY        = 7;
  __WASI_ERRNO_BADF           = 8;
  __WASI_ERRNO_BADMSG         = 9;
  __WASI_ERRNO_BUSY           = 10;
  __WASI_ERRNO_CANCELED       = 11;
  __WASI_ERRNO_CHILD          = 12;
  __WASI_ERRNO_CONNABORTED    = 13;
  __WASI_ERRNO_CONNREFUSED    = 14;
  __WASI_ERRNO_CONNRESET      = 15;
  __WASI_ERRNO_DEADLK         = 16;
  __WASI_ERRNO_DESTADDRREQ    = 17;
  __WASI_ERRNO_DOM            = 18;
  __WASI_ERRNO_DQUOT          = 19;
  __WASI_ERRNO_EXIST          = 20;
  __WASI_ERRNO_FAULT          = 21;
  __WASI_ERRNO_FBIG           = 22;
  __WASI_ERRNO_HOSTUNREACH    = 23;
  __WASI_ERRNO_IDRM           = 24;
  __WASI_ERRNO_ILSEQ          = 25;
  __WASI_ERRNO_INPROGRESS     = 26;
  __WASI_ERRNO_INTR           = 27;
  __WASI_ERRNO_INVAL          = 28;
  __WASI_ERRNO_IO             = 29;
  __WASI_ERRNO_ISCONN         = 30;
  __WASI_ERRNO_ISDIR          = 31;
  __WASI_ERRNO_LOOP           = 32;
  __WASI_ERRNO_MFILE          = 33;
  __WASI_ERRNO_MLINK          = 34;
  __WASI_ERRNO_MSGSIZE        = 35;
  __WASI_ERRNO_MULTIHOP       = 36;
  __WASI_ERRNO_NAMETOOLONG    = 37;
  __WASI_ERRNO_NETDOWN        = 38;
  __WASI_ERRNO_NETRESET       = 39;
  __WASI_ERRNO_NETUNREACH     = 40;
  __WASI_ERRNO_NFILE          = 41;
  __WASI_ERRNO_NOBUFS         = 42;
  __WASI_ERRNO_NODEV          = 43;
  __WASI_ERRNO_NOENT          = 44;
  __WASI_ERRNO_NOEXEC         = 45;
  __WASI_ERRNO_NOLCK          = 46;
  __WASI_ERRNO_NOLINK         = 47;
  __WASI_ERRNO_NOMEM          = 48;
  __WASI_ERRNO_NOMSG          = 49;
  __WASI_ERRNO_NOPROTOOPT     = 50;
  __WASI_ERRNO_NOSPC          = 51;
  __WASI_ERRNO_NOSYS          = 52;
  __WASI_ERRNO_NOTCONN        = 53;
  __WASI_ERRNO_NOTDIR         = 54;
  __WASI_ERRNO_NOTEMPTY       = 55;
  __WASI_ERRNO_NOTRECOVERABLE = 56;
  __WASI_ERRNO_NOTSOCK        = 57;
  __WASI_ERRNO_NOTSUP         = 58;
  __WASI_ERRNO_NOTTY          = 59;
  __WASI_ERRNO_NXIO           = 60;
  __WASI_ERRNO_OVERFLOW       = 61;
  __WASI_ERRNO_OWNERDEAD      = 62;
  __WASI_ERRNO_PERM           = 63;
  __WASI_ERRNO_PIPE           = 64;
  __WASI_ERRNO_PROTO          = 65;
  __WASI_ERRNO_PROTONOSUPPORT = 66;
  __WASI_ERRNO_PROTOTYPE      = 67;
  __WASI_ERRNO_RANGE          = 68;
  __WASI_ERRNO_ROFS           = 69;
  __WASI_ERRNO_SPIPE          = 70;
  __WASI_ERRNO_SRCH           = 71;
  __WASI_ERRNO_STALE          = 72;
  __WASI_ERRNO_TIMEDOUT       = 73;
  __WASI_ERRNO_TXTBSY         = 74;
  __WASI_ERRNO_XDEV           = 75;
  __WASI_ERRNO_NOTCAPABLE     = 76;

type
  P__wasi_rights_t = ^__wasi_rights_t;
  __wasi_rights_t = UInt64;

const
  __WASI_RIGHTS_FD_DATASYNC             = 1;
  __WASI_RIGHTS_FD_READ                 = 2;
  __WASI_RIGHTS_FD_SEEK                 = 4;
  __WASI_RIGHTS_FD_FDSTAT_SET_FLAGS     = 8;
  __WASI_RIGHTS_FD_SYNC                 = 16;
  __WASI_RIGHTS_FD_TELL                 = 32;
  __WASI_RIGHTS_FD_WRITE                = 64;
  __WASI_RIGHTS_FD_ADVISE               = 128;
  __WASI_RIGHTS_FD_ALLOCATE             = 256;
  __WASI_RIGHTS_PATH_CREATE_DIRECTORY   = 512;
  __WASI_RIGHTS_PATH_CREATE_FILE        = 1024;
  __WASI_RIGHTS_PATH_LINK_SOURCE        = 2048;
  __WASI_RIGHTS_PATH_LINK_TARGET        = 4096;
  __WASI_RIGHTS_PATH_OPEN               = 8192;
  __WASI_RIGHTS_FD_READDIR              = 16384;
  __WASI_RIGHTS_PATH_READLINK           = 32768;
  __WASI_RIGHTS_PATH_RENAME_SOURCE      = 65536;
  __WASI_RIGHTS_PATH_RENAME_TARGET      = 131072;
  __WASI_RIGHTS_PATH_FILESTAT_GET       = 262144;
  __WASI_RIGHTS_PATH_FILESTAT_SET_SIZE  = 524288;
  __WASI_RIGHTS_PATH_FILESTAT_SET_TIMES = 1048576;
  __WASI_RIGHTS_FD_FILESTAT_GET         = 2097152;
  __WASI_RIGHTS_FD_FILESTAT_SET_SIZE    = 4194304;
  __WASI_RIGHTS_FD_FILESTAT_SET_TIMES   = 8388608;
  __WASI_RIGHTS_PATH_SYMLINK            = 16777216;
  __WASI_RIGHTS_PATH_REMOVE_DIRECTORY   = 33554432;
  __WASI_RIGHTS_PATH_UNLINK_FILE        = 67108864;
  __WASI_RIGHTS_POLL_FD_READWRITE       = 134217728;
  __WASI_RIGHTS_SOCK_SHUTDOWN           = 268435456;

type
  P__wasi_fd_t = ^__wasi_fd_t;
  __wasi_fd_t = longint;

  P__wasi_iovec_t = ^__wasi_iovec_t;
  __wasi_iovec_t = record
    buf: PUInt8;
    buf_len: __wasi_size_t;
  end;

  P__wasi_ciovec_t = ^__wasi_ciovec_t;
  __wasi_ciovec_t = record
    buf: PUInt8;
    buf_len: __wasi_size_t;
  end;

  P__wasi_filedelta_t = ^__wasi_filedelta_t;
  __wasi_filedelta_t = Int64;

  P__wasi_whence_t = ^__wasi_whence_t;
  __wasi_whence_t = UInt8;

const
  __WASI_WHENCE_SET = 0;
  __WASI_WHENCE_CUR = 1;
  __WASI_WHENCE_END = 2;

type
  P__wasi_dircookie_t = ^__wasi_dircookie_t;
  __wasi_dircookie_t = UInt64;

  P__wasi_dirnamlen_t = ^__wasi_dirnamlen_t;
  __wasi_dirnamlen_t = UInt32;

  P__wasi_inode_t = ^__wasi_inode_t;
  __wasi_inode_t = UInt64;

  P__wasi_filetype_t = ^__wasi_filetype_t;
  __wasi_filetype_t = UInt8;

const
  __WASI_FILETYPE_UNKNOWN          = 0;
  __WASI_FILETYPE_BLOCK_DEVICE     = 1;
  __WASI_FILETYPE_CHARACTER_DEVICE = 2;
  __WASI_FILETYPE_DIRECTORY        = 3;
  __WASI_FILETYPE_REGULAR_FILE     = 4;
  __WASI_FILETYPE_SOCKET_DGRAM     = 5;
  __WASI_FILETYPE_SOCKET_STREAM    = 6;
  __WASI_FILETYPE_SYMBOLIC_LINK    = 7;

type
  P__wasi_dirent_t = ^__wasi_dirent_t;
  __wasi_dirent_t = record
    d_next: __wasi_dircookie_t;
    d_ino: __wasi_inode_t;
    d_namlen: __wasi_dirnamlen_t;
    d_type: __wasi_filetype_t;
  end;

  P__wasi_advice_t = ^__wasi_advice_t;
  __wasi_advice_t = UInt8;

const
  __WASI_ADVICE_NORMAL     = 0;
  __WASI_ADVICE_SEQUENTIAL = 1;
  __WASI_ADVICE_RANDOM     = 2;
  __WASI_ADVICE_WILLNEED   = 3;
  __WASI_ADVICE_DONTNEED   = 4;
  __WASI_ADVICE_NOREUSE    = 5;

type
  P__wasi_fdflags_t = ^__wasi_fdflags_t;
  __wasi_fdflags_t = UInt16;

const
  __WASI_FDFLAGS_APPEND   = 1;
  __WASI_FDFLAGS_DSYNC    = 2;
  __WASI_FDFLAGS_NONBLOCK = 4;
  __WASI_FDFLAGS_RSYNC    = 8;
  __WASI_FDFLAGS_SYNC     = 16;

type
  P__wasi_fdstat_t = ^__wasi_fdstat_t;
  __wasi_fdstat_t = record
    fs_filetype: __wasi_filetype_t;
    fs_flags: __wasi_fdflags_t;
    fs_rights_base: __wasi_rights_t;
    fs_rights_inheriting: __wasi_rights_t;
  end;

type
  P__wasi_device_t = ^__wasi_device_t;
  __wasi_device_t = UInt64;

  P__wasi_fstflags_t = ^__wasi_fstflags_t;
  __wasi_fstflags_t = UInt16;

const
  __WASI_FSTFLAGS_ATIM     = 1;
  __WASI_FSTFLAGS_ATIM_NOW = 2;
  __WASI_FSTFLAGS_MTIM     = 4;
  __WASI_FSTFLAGS_MTIM_NOW = 8;

type
  P__wasi_lookupflags_t = ^__wasi_lookupflags_t;
  __wasi_lookupflags_t = UInt32;

const
  __WASI_LOOKUPFLAGS_SYMLINK_FOLLOW = 1;

type
  P__wasi_oflags_t = ^__wasi_oflags_t;
  __wasi_oflags_t = UInt16;

const
  __WASI_OFLAGS_CREAT     = 1;
  __WASI_OFLAGS_DIRECTORY = 2;
  __WASI_OFLAGS_EXCL      = 4;
  __WASI_OFLAGS_TRUNC     = 8;

type
  P__wasi_linkcount_t = ^__wasi_linkcount_t;
  __wasi_linkcount_t = UInt64;

  P__wasi_filestat_t = ^__wasi_filestat_t;
  __wasi_filestat_t = record
    dev: __wasi_device_t;
    ino: __wasi_inode_t;
    filetype: __wasi_filetype_t;
    nlink: __wasi_linkcount_t;
    size: __wasi_filesize_t;
    atim: __wasi_timestamp_t;
    mtim: __wasi_timestamp_t;
    ctim: __wasi_timestamp_t;
  end;

  P__wasi_userdata_t = ^__wasi_userdata_t;
  __wasi_userdata_t = UInt64;

  P__wasi_eventtype_t = ^__wasi_eventtype_t;
  __wasi_eventtype_t = UInt8;

const
  __WASI_EVENTTYPE_CLOCK    = 0;
  __WASI_EVENTTYPE_FD_READ  = 1;
  __WASI_EVENTTYPE_FD_WRITE = 2;

type
  P__wasi_eventrwflags_t = ^__wasi_eventrwflags_t;
  __wasi_eventrwflags_t = UInt16;

const
  __WASI_EVENTRWFLAGS_FD_READWRITE_HANGUP = 1;

type
  P__wasi_event_fd_readwrite_t = ^__wasi_event_fd_readwrite_t;
  __wasi_event_fd_readwrite_t = record
    nbytes: __wasi_filesize_t;
    flags: __wasi_eventrwflags_t;
  end;

  P__wasi_event_t = ^__wasi_event_t;
  __wasi_event_t = record
    userdata: __wasi_userdata_t;
    error: __wasi_errno_t;
    type_: __wasi_eventtype_t;
    fd_readwrite: __wasi_event_fd_readwrite_t;
  end;

  P__wasi_subclockflags_t = ^__wasi_subclockflags_t;
  __wasi_subclockflags_t = UInt16;

const
  __WASI_SUBCLOCKFLAGS_SUBSCRIPTION_CLOCK_ABSTIME = 1;

type
  P__wasi_subscription_clock_t = ^__wasi_subscription_clock_t;
  __wasi_subscription_clock_t = record
    id: __wasi_clockid_t;
    timeout: __wasi_timestamp_t;
    precision: __wasi_timestamp_t;
    flags: __wasi_subclockflags_t;
  end;

  P__wasi_subscription_fd_readwrite_t = ^__wasi_subscription_fd_readwrite_t;
  __wasi_subscription_fd_readwrite_t = record
    file_descriptor: __wasi_fd_t;
  end;

  P__wasi_subscription_u_u_t = ^__wasi_subscription_u_u_t;
  __wasi_subscription_u_u_t = record
    case integer of
      0: (clock: __wasi_subscription_clock_t);
      1: (fd_read: __wasi_subscription_fd_readwrite_t);
      2: (fd_write: __wasi_subscription_fd_readwrite_t);
  end;
  P__wasi_subscription_u_t = ^__wasi_subscription_u_t;
  __wasi_subscription_u_t = record
    tag: __wasi_eventtype_t;
    u: __wasi_subscription_u_u_t;
  end;

  P__wasi_subscription_t = ^__wasi_subscription_t;
  __wasi_subscription_t = record
    userdata: __wasi_userdata_t;
    u: __wasi_subscription_u_t;
  end;

  P__wasi_exitcode_t = ^__wasi_exitcode_t;
  __wasi_exitcode_t = UInt32;

  P__wasi_signal_t = ^__wasi_signal_t;
  __wasi_signal_t = UInt8;

const
  __WASI_SIGNAL_NONE   = 0;
  __WASI_SIGNAL_HUP    = 1;
  __WASI_SIGNAL_INT    = 2;
  __WASI_SIGNAL_QUIT   = 3;
  __WASI_SIGNAL_ILL    = 4;
  __WASI_SIGNAL_TRAP   = 5;
  __WASI_SIGNAL_ABRT   = 6;
  __WASI_SIGNAL_BUS    = 7;
  __WASI_SIGNAL_FPE    = 8;
  __WASI_SIGNAL_KILL   = 9;
  __WASI_SIGNAL_USR1   = 10;
  __WASI_SIGNAL_SEGV   = 11;
  __WASI_SIGNAL_USR2   = 12;
  __WASI_SIGNAL_PIPE   = 13;
  __WASI_SIGNAL_ALRM   = 14;
  __WASI_SIGNAL_TERM   = 15;
  __WASI_SIGNAL_CHLD   = 16;
  __WASI_SIGNAL_CONT   = 17;
  __WASI_SIGNAL_STOP   = 18;
  __WASI_SIGNAL_TSTP   = 19;
  __WASI_SIGNAL_TTIN   = 20;
  __WASI_SIGNAL_TTOU   = 21;
  __WASI_SIGNAL_URG    = 22;
  __WASI_SIGNAL_XCPU   = 23;
  __WASI_SIGNAL_XFSZ   = 24;
  __WASI_SIGNAL_VTALRM = 25;
  __WASI_SIGNAL_PROF   = 26;
  __WASI_SIGNAL_WINCH  = 27;
  __WASI_SIGNAL_POLL   = 28;
  __WASI_SIGNAL_PWR    = 29;
  __WASI_SIGNAL_SYS    = 30;

type
  P__wasi_riflags_t = ^__wasi_riflags_t;
  __wasi_riflags_t = UInt16;

const
  __WASI_RIFLAGS_RECV_PEEK    = 1;
  __WASI_RIFLAGS_RECV_WAITALL = 2;

type
  P__wasi_roflags_t = ^__wasi_roflags_t;
  __wasi_roflags_t = UInt16;

const
  __WASI_ROFLAGS_RECV_DATA_TRUNCATED = 1;

type
  P__wasi_siflags_t = ^__wasi_siflags_t;
  __wasi_siflags_t = UInt16;

  P__wasi_sdflags_t = ^__wasi_sdflags_t;
  __wasi_sdflags_t = UInt8;

const
  __WASI_SDFLAGS_RD = 1;
  __WASI_SDFLAGS_WR = 2;

type
  P__wasi_preopentype_t = ^__wasi_preopentype_t;
  __wasi_preopentype_t = UInt8;

const
  __WASI_PREOPENTYPE_DIR = 0;

type
  P__wasi_prestat_dir_t = ^__wasi_prestat_dir_t;
  __wasi_prestat_dir_t = record
    pr_name_len: __wasi_size_t;
  end;

  P__wasi_prestat_u_t = ^__wasi_prestat_u_t;
  __wasi_prestat_u_t = record
    case integer of
      0: (dir: __wasi_prestat_dir_t);
  end;
  P__wasi_prestat_t = ^__wasi_prestat_t;
  __wasi_prestat_t = record
    tag: __wasi_preopentype_t;
    u: __wasi_prestat_u_t;
  end;

