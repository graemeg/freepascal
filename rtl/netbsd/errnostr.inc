{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Ales Katona

    Contains BSD specific errors for error.pp in rtl/unix

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ Checked using ../unix/scipts/check_errnostr.sh  }
const
  sys_errn=97;
=======
{$warning FIX ME, I'am from FreeBSD }
const
  sys_errn=93;
>>>>>>> graemeg/cpstrnew
=======
{$warning FIX ME, I'am from FreeBSD }
const
  sys_errn=93;
>>>>>>> graemeg/cpstrnew
=======
{$warning FIX ME, I'am from FreeBSD }
const
  sys_errn=93;
>>>>>>> graemeg/cpstrnew
=======
{$warning FIX ME, I'am from FreeBSD }
const
  sys_errn=93;
>>>>>>> origin/cpstrnew
=======
{$warning FIX ME, I'am from FreeBSD }
const
  sys_errn=93;
>>>>>>> origin/fixes_2.4
  sys_errlist:array[0..sys_errn-1] of pchar = (
        'Success',                              { 0 }
        'Operation not permitted',              { EPERM }
        'No such file or directory',            { ENOENT }
        'No such process',                      { ESRCH }
        'Interrupted system call',              { EINTR }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        'Input/output error',                   { EIO }
        'Device not configured',                { ENXIO }
        'Argument list too long',               { E2BIG }
        'Exec format error',                    { ENOEXEC }
        'Bad file descriptor',                  { EBADF }
        'No child processes',                   { ECHILD }
        'Resource deadlock avoided',            { EDEADLK was EAGAIN }
        'Cannot allocate memory',               { ENOMEM }
        'Permission denied',                    { EACCES }
        'Bad address',                          { EFAULT }
        'Block device required',                { ENOTBLK }
        'Device busy',                          { EBUSY }
        'File exists',                          { EEXIST }
        'Cross-device link',                    { EXDEV }
        'Operation not supported by device',    { ENODEV }
        'Not a directory',                      { ENOTDIR }
        'Is a directory',                       { EISDIR }
        'Invalid argument',                     { EINVAL }
        'Too many open files in system',        { ENFILE }
        'Too many open files',                  { EMFILE }
        'Inappropriate ioctl for device',       { ENOTTY }
        'Text file busy',                       { ETXTBSY  Text file busy.  The new process was
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
        'I/O error',                            { EIO }
        'No such device or address',            { ENXIO }
        'Arg list too long',                    { E2BIG }
        'Exec format error',                    { ENOEXEC }
        'Bad file number',                      { EBADF }
        'No child processes',                   { ECHILD }
        'Resource deadlock avoided',   { EDEADLK was EAGAIN }
        'Out of memory',                        { ENOMEM }
        'Permission denied',                    { EACCES }
        'Bad address',                          { EFAULT }
        'Block device required',                { ENOTBLK }
        'Device or resource busy',              { EBUSY }
        'File exists',                          { EEXIST }
        'Cross-device link',                    { EXDEV }
        'No such device',                       { ENODEV }
        'Not a directory',                      { ENOTDIR }
        'Is a directory',                       { EISDIR }
        'Invalid argument',                     { EINVAL }
        'File table overflow',                  { ENFILE }
        'Too many open files',                  { EMFILE }
        'Not a typewriter',                     { ENOTTY }
        'Text (code segment) file busy',        { ETXTBSY  Text file busy.  The new process was
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                                                    a pure procedure (shared text) file which was
                                                    open for writing by another process, or file
                                                    which was open for writing by another process,
                                                    or while the pure procedure file was being
                                                    executed an open(2) call requested write access
                                                    requested write access.}
        'File too large',                       { EFBIG }
        'No space left on device',              { ENOSPC }
        'Illegal seek',                         { ESPIPE }
        'Read-only file system',                { EROFS }
        'Too many links',                       { EMLINK }
        'Broken pipe',                          { EPIPE }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        'Numerical argument out of domain',     { EDOM }
        'Result too large',                     { ERANGE }
        'Resource temporarily unavailable',     { EAGAIN }
        'Operation now in progress',            { EINPROGRESS }
        'Operation already in progress',        { EALREADY	}
// ipc/network software -- argument errors
        'Socket operation on non-socket',       { ENOTSOCK }
        'Destination address required',         { EDESTADDRREQ }
        'Message too long',                     { EMSGSIZE }
        'Protocol wrong type for socket',       { EPROTOTYPE }
        'Protocol not available',               { ENOPROTOOPT }
        'Protocol not supported',               { EPROTONOSUPPORT }
        'Socket type not supported',            { ESOCKTNOSUPPORT }
        'Operation not supported',              { EOPNOTSUPP }
        'Protocol family not supported',        { EPFNOSUPPORT }
        'Address family not supported by protocol family',  { EAFNOSUPPORT }
        'Address already in use',               { EADDRINUSE }
        'Can''t assign requested address',      { EADDRNOTAVAIL }
// ipc/network software -- operational errors
        'Network is down',                      { ENETDOWN }
        'Network is unreachable',               { ENETUNREACH }
        'Network dropped connection on reset',  { ENETRESET }
        'Software caused connection abort',     { ECONNABORTED }
        'Connection reset by peer',             { ECONNRESET }
        'No buffer space available',            { ENOBUFS }
        'Socket is already connected',          { EISCONN }
        'Socket is not connected',              { ENOTCONN }
        'Can''t send after socket shutdown',    { ESHUTDOWN }
        'Too many references: can''t splice',   { ETOOMANYREFS }
        'Operation timed out',                  { ETIMEDOUT }
        'Connection refused',                   { ECONNREFUSED }
        'Too many levels of symbolic links',    { ELOOP }
        'File name too long',                   { ENAMETOOLONG }
        'Host is down',                         { EHOSTDOWN }
        'No route to host',                     { EHOSTUNREACH }
        'Directory not empty',                  { ENOTEMPTY }
        'Too many processes',                   { EPROCLIM }
        'Too many users',                       { EUSERS }
        'Disc quota exceeded',                  { EDQUOT }
// Network File System
        'Stale NFS file handle',                { ESTALE }
        'Too many levels of remote in path',    { EREMOTE }
        'RPC struct is bad',                    { EBADRPC }
        'RPC version wrong',                    { ERPCMISMATCH }
        'RPC prog. not avail',                  { EPROGUNAVAIL }
        'Program version wrong',                { EPROGMISMATCH }
        'Bad procedure for program',            { EPROCUNAVAIL }
        'No locks available',                   { ENOLCK }
        'Function not implemented',             { ENOSYS }
        'Inappropriate file type or format',    { EFTYPE }
        'Authentication error',                 { EAUTH }
        'Need authenticator',                   { ENEEDAUTH }
        'Identifier removed',                   { EIDRM }
        'No message of desired type',           { ENOMSG }
        'Value too large to be stored in data type', { EOVERFLOW }
        'Illegal byte sequence',                { EILSEQ }
        'Not supported',                        { ENOTSUP }
        'Operation canceled',                   { ECANCELED }
        'Bad or Corrupt message',               { EBADMSG }
        'No message available',                 { ENODATA }
        'No STREAM resources',                  { ENOSR }
        'Not a STREAM',                         { ENOSTR }
        'STREAM ioctl timeout',                 { ETIME }
        'Attribute not found',                  { ENOATTR }
        'Multihop attempted',                   { EMULTIHOP }
        'Link has been severed',                { ENOLINK }
        'Protocol error'                        { EPROTO }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
        'Math argument out of domain of func',  { EDOM }
        'Math result not representable',        { ERANGE }
        'Resource temporarily unavailable',        { EAGAIN }
        'Operation now in progress',      { EINPROGRESS }
        'Operation already in progress', { EALREADY	}
// ipc/network software -- argument errors
        'Socket operation on non-socket',    { ENOTSOCK }
        'Destination address required',     { EDESTADDRREQ }
        'Message too long',                     { EMSGSIZE }
        'Protocol wrong type for socket', { EPROTOTYPE }
        'Protocol not available',               { ENOPROTOOPT }
        'Protocol not supported',              { EPROTONOSUPPORT }
        'Socket type not supported',         { ESOCKTNOSUPPORT }
        'Operation not supported',             { EOPNOTSUPP }
        'Protocol family not supported',      { EPFNOSUPPORT }
        'Address family not supported by protocol family',  { EAFNOSUPPORT }
        'Address already in use',                { EADDRINUSE }
        'Can''t assign requested address',  { EADDRNOTAVAIL }
// ipc/network software -- operational errors
        'Network is down',                            { ENETDOWN }
        'Network is unreachable',                 { ENETUNREACH }
        'Network dropped connection on reset', { ENETRESET }
        'Software caused connection abort',   { ECONNABORTED }
        'Connection reset by peer',                 { ECONNRESET }
        'No buffer space available',                { ENOBUFS }
        'Socket is already connected',            { EISCONN }
        'Socket is not connected',                    { ENOTCONN }
        'Can''t send after socket shutdown',    { ESHUTDOWN }
        'Too many references: can''t splice',   { ETOOMANYREFS }
        'Operation timed out',                           { ETIMEDOUT }
        'Connection refused',                           { ECONNREFUSED }
        'Too many levels of symbolic links',     { ELOOP }
        'File name too long',                              { ENAMETOOLONG }
        'Host is down',                                       { EHOSTDOWN }
        'No route to host',                                  { EHOSTUNREACH }
        'Directory not empty',                            { ENOTEMPTY }
        'Too many processes',                           { EPROCLIM }
        'Too many users',                                   { EUSERS }
        'Disc quota exceeded',                           { EDQUOT }
// Network File System
        'Stale NFS file handle',                              { ESTALE }
        'Too many levels of remote in path',        { EREMOTE }
        'RPC struct is bad',                                    { EBADRPC }
        'RPC version wrong',                                { ERPCMISMATCH }
        'RPC prog. not avail',                                { EPROGUNAVAIL }
        'Program version wrong',                          { EPROGMISMATCH }
        'Bad procedure for program',                    { EPROCUNAVAIL }
        'No locks available',                                   { ENOLCK }
        'Function not implemented',                       { ENOSYS }
        'Inappropriate file type or format',              { EFTYPE }
        'Authentication error',                                 { EAUTH }
        'Need authenticator',                                   { ENEEDAUTH }
        'Identifier removed',                                   { EIDRM }
        'No message of desired type',                   { ENOMSG }
        'Value too large to be stored in data type', { EOVERFLOW }
        'Operation canceled',                                  { ECANCELED }
        'Illegal byte sequence',                                { EILSEQ }
        'Attribute not found',                                    { ENOATTR }
        'Programming error',                                    { EDOOFUS }
        'Bad message',                                            { EBADMSG }
        'Multihop attempted',                                   { EMULTIHOP }
        'Link has been severed',                              { ENOLINK }
        'Protocol error'                                             { EPROTO }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
);

