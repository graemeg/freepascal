{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2012 by Free Pascal development team

    This file contains platform-specific defines that are used in
    multiple RTL units.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ the single byte OS APIs always use UTF-8 }
{ define FPCRTL_FILESYSTEM_UTF8}

{ The OS supports a single byte file system operations API that we use }
{$define FPCRTL_FILESYSTEM_SINGLE_BYTE_API}

{ The OS supports a two byte file system operations API that we use }
{ define FPCRTL_FILESYSTEM_TWO_BYTE_API}

{ Use the shortstring version of do_getdir (should be converted to rawbytestring or
  unicodestring for all but the OSes that don't support code pages and/or are
  resource limited) }
{$define FPCRTL_DO_GETDIR_SHORTSTRING}
