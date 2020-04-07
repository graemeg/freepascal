{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Jonas Maebe, (c) 2005 Thomas Schatzl,
    members of the Free Pascal development team.

    Contains the definition of the stat type for the Xtensa platform.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ This structure was adapted from

    include/uapi/asm-generic/stat.h

  in Linux 4.0. Note that the stat record is the same for direct
  syscalls as for when linking to libc.
}

{$PACKRECORDS C}
  stat = record
      st_dev        : culonglong;
      st_ino        : culonglong;
      st_mode       : cuint;
      st_nlink      : cuint;
      st_uid        : cuint;
      st_gid        : cuint;
      st_rdev       : culonglong;
      st_size       : clonglong;
      st_blksize    : culong;
      __unused2     : culong;
      st_blocks     : culonglong;
      st_atime      : cULong;
      st_atime_nsec : cULong;
      st_mtime      : cULong;
      st_mtime_nsec : cULong;
      st_ctime      : cULong;
      st_ctime_nsec : cULong;
      __unused4a    : cULong;
      __unused5a    : cULong;
  end;
