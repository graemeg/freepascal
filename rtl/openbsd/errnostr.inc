{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2010 by Jasper Lievisse Adriaanse <jasper@openbsd.org>

    Contains BSD specific errors for error.pp in rtl/unix

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
  sys_errn=92;
  sys_errlist:array[0..sys_errn-1] of pchar = (
        'Success',                             { 0 }
        'Operation not permitted',             { EPERM }
        'No such file or directory',           { ENOENT }
        'No such process',                     { ESRCH }
        'Interrupted system call',             { EINTR }
        'I/O error',                           { EIO }
        'No such device or address',           { ENXIO }
        'Arg list too long',                   { E2BIG }
        'Exec format error',                   { ENOEXEC }
        'Bad file number',                     { EBADF }
        'No child processes',                  { ECHILD }
        'Resource deadlock avoided',           { EDEADLK was EAGAIN }
        'Out of memory',                       { ENOMEM }
        'Permission denied',                   { EACCES }
        'Bad address',                         { EFAULT }
        'Block device required',               { ENOTBLK }
        'Device or resource busy',             { EBUSY }
        'File exists',                         { EEXIST }
        'Cross-device link',                   { EXDEV }
        'No such device',                      { ENODEV }
        'Not a directory',                     { ENOTDIR }
        'Is a directory',                      { EISDIR }
        'Invalid argument',                    { EINVAL }
        'File table overflow',                 { ENFILE }
        'Too many open files',                 { EMFILE }
        'Not a typewriter',                    { ENOTTY }
        'Text file busy',                      { ETXTBSY }
        'File too large',                      { EFBIG }
        'No space left on device',             { ENOSPC }
        'Illegal seek',                        { ESPIPE }
        'Read-only file system',               { EROFS }
        'Too many links',                      { EMLINK }
        'Broken pipe',                         { EPIPE }
// math software
	'Numerical argument out of domain',    { EDOM }
	'Result too large',		       { ERANGE }
// non-blocking and interrupt io
        'Resource temporarily unavailable',    { EAGAIN }
        'Operation now in progress',           { EINPROGRESS }
//      'Operation would block',	       { EAGAIN }
        'Operation already in progress',       { EALREADY }
// ipc/network software -- argument errors
        'Socket operation on non-socket',      { ENOTSOCK }
        'Destination address required',        { EDESTADDRREQ }
        'Message too long',                    { EMSGSIZE }
        'Protocol wrong type for socket',      { EPROTOTYPE }
        'Protocol not available',              { ENOPROTOOPT }
        'Protocol not supported',              { EPROTONOSUPPORT }
        'Socket type not supported',           { ESOCKTNOSUPPORT }
        'Operation not supported',             { EOPNOTSUPP }
        'Protocol family not supported',       { EPFNOSUPPORT }
        'Address family not supported by protocol family', { EAFNOSUPPORT }
        'Address already in use',              { EADDRINUSE }
        'Can''t assign requested address',     { EADDRNOTAVAIL }
// ipc/network software -- operational errors
        'Network is down',                     { ENETDOWN }
        'Network is unreachable',              { ENETUNREACH }
        'Network dropped connection on reset', { ENETRESET }
        'Software caused connection abort',    { ECONNABORTED }
        'Connection reset by peer',            { ECONNRESET }
        'No buffer space available',           { ENOBUFS }
        'Socket is already connected',         { EISCONN }
        'Socket is not connected',             { ENOTCONN }
        'Can''t send after socket shutdown',   { ESHUTDOWN }
        'Too many references: can''t splice',  { ETOOMANYREFS }
        'Operation timed out',                 { ETIMEDOUT }
        'Connection refused',                  { ECONNREFUSED }
        'Too many levels of symbolic links',   { ELOOP }
        'File name too long',                  { ENAMETOOLONG }
        'Host is down',                        { EHOSTDOWN }
        'No route to host',                    { EHOSTUNREACH }
        'Directory not empty',                 { ENOTEMPTY }
// quotas & mush
        'Too many processes',                  { EPROCLIM }
        'Too many users',                      { EUSERS }
        'Disc quota exceeded',                 { EDQUOT }
// Network File System
        'Stale NFS file handle',               { ESTALE }
        'Too many levels of remote in path',   { EREMOTE }
        'RPC struct is bad',                   { EBADRPC }
        'RPC version wrong',                   { ERPCMISMATCH }
        'RPC prog. not avail',                 { EPROGUNAVAIL }
        'Program version wrong',               { EPROGMISMATCH }
        'Bad procedure for program',           { EPROCUNAVAIL }
        'No locks available',                  { ENOLCK }
        'Function not implemented',            { ENOSYS }
        'Inappropriate file type or format',   { EFTYPE }
        'Authentication error',                { EAUTH }
        'Need authenticator',                  { ENEEDAUTH }
	'IPsec processing failure',            { EIPSEC }
	'Attribute not found',                 { ENOATTR }
        'Illegal byte sequence',               { EILSEQ }
	'No medium found',                     { ENOMEDIUM }
	'Wrong Meda Type',                     { EMEDIUMTYPE }
        'Value too large to be stored in data type', { EOVERFLOW }
        'Operation canceled',                  { ECANCELED }
        'Identifier removed',                  { EIDRM }
        'No message of desired type',          { ENOMSG }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	'Not supported'                        { ENOTSUP }
//      'Must be equal to largest errno'       { ELAST }
=======
	'Not supported',                       { ENOTSUP }
//	'Must be equal to largest errno'       { ELAST }
>>>>>>> graemeg/cpstrnew
=======
	'Not supported',                       { ENOTSUP }
//	'Must be equal to largest errno'       { ELAST }
>>>>>>> graemeg/cpstrnew
=======
	'Not supported',                       { ENOTSUP }
//	'Must be equal to largest errno'       { ELAST }
>>>>>>> graemeg/cpstrnew
=======
	'Not supported',                       { ENOTSUP }
//	'Must be equal to largest errno'       { ELAST }
>>>>>>> origin/cpstrnew
);
