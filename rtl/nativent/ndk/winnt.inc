{%MainUnit ndk.pas}
{
    Native Development Kit for Native NT

    This file is part of the Free Pascal run time library.
    This units contains some types and constants used by NT
    Copyright (c) 2010 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
  WCHAR = WideChar;
  PWSTR = PWideChar;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  CCHAR = Byte;
  PCCHAR = ^CCHAR;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
  NT_DELETE           = $00010000;
  NT_SYNCHRONIZE      = $00100000;

  GENERIC_READ	      = $80000000;
  GENERIC_WRITE	      = $40000000;
  GENERIC_EXECUTE     = $20000000;
  GENERIC_ALL	      = $10000000;

  FILE_LIST_DIRECTORY		= $00000001;
  FILE_READ_DATA		= $00000001;
  FILE_ADD_FILE			= $00000002;
  FILE_WRITE_DATA		= $00000002;
  FILE_ADD_SUBDIRECTORY		= $00000004;
  FILE_APPEND_DATA		= $00000004;
  FILE_CREATE_PIPE_INSTANCE	= $00000004;
  FILE_READ_EA			= $00000008;
  FILE_READ_PROPERTIES		= $00000008;
  FILE_WRITE_EA			= $00000010;
  FILE_WRITE_PROPERTIES		= $00000010;
  FILE_EXECUTE			= $00000020;
  FILE_TRAVERSE			= $00000020;
  FILE_DELETE_CHILD		= $00000040;
  FILE_READ_ATTRIBUTES		= $00000080;
  FILE_WRITE_ATTRIBUTES		= $00000100;

  FILE_SHARE_READ			= $00000001;
  FILE_SHARE_WRITE		        = $00000002;
  FILE_SHARE_DELETE		        = $00000004;
  FILE_SHARE_VALID_FLAGS		= $00000007;

  FILE_ATTRIBUTE_READONLY		= $00000001;
  FILE_ATTRIBUTE_HIDDEN			= $00000002;
  FILE_ATTRIBUTE_SYSTEM			= $00000004;
  FILE_ATTRIBUTE_DIRECTORY		= $00000010;
  FILE_ATTRIBUTE_ARCHIVE		= $00000020;
  FILE_ATTRIBUTE_DEVICE			= $00000040;
  FILE_ATTRIBUTE_NORMAL			= $00000080;
  FILE_ATTRIBUTE_TEMPORARY		= $00000100;
  FILE_ATTRIBUTE_SPARSE_FILE		= $00000200;
  FILE_ATTRIBUTE_REPARSE_POINT		= $00000400;
  FILE_ATTRIBUTE_COMPRESSED		= $00000800;
  FILE_ATTRIBUTE_OFFLINE		= $00001000;
  FILE_ATTRIBUTE_NOT_CONTENT_INDEXED	= $00002000;
  FILE_ATTRIBUTE_ENCRYPTED		= $00004000;
  FILE_ATTRIBUTE_VALID_FLAGS		= $00007fb7;
  FILE_ATTRIBUTE_VALID_SET_FLAGS	= $000031a7;

type
  ACCESS_MASK = DWORD;
  PACCESS_MASK = ^ACCESS_MASK;
