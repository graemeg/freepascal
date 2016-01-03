{
Test for FUSE Freepascal bindings.
Copyright (C) 2008 Danny Milosavljevic <danny_milo@yahoo.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

}
{$MODE OBJFPC}{$H+} // octal

uses BaseUNIX, Strings, FUSE;

const
  hello_path : String = '/hello';
  hello_str : String = 'Hello World!'#10;

function hello_getattr(const aNameC : PChar;var aStat : tStat) : cint; cdecl;
var
  aName : String;
begin
  Result := 0;

  aName := aNameC;

  FillChar(aStat, Sizeof(TStat), 0);

  if (aName = '/') then begin
    aSTAT.st_mode := S_IFDIR or &0755;  // 0755;
    aSTAT.st_nlink := 2;
  end else if aName = hello_path then begin
    aSTAT.st_mode := S_IFREG or &0444; // 0444;
    aSTAT.st_nlink := 1;
    aSTAT.st_size := Length(hello_str);
  end else
    Result := -ESysENOENT;
end;

var
  xx : PChar = 'hello';

function hello_readdir(const ANameC : PChar; aBuffer : Pointer; filler : TFUseFillDir; aFileOffset : off_t; aFileInfo : PFuseFileInfo) : Integer; cdecl;
begin
  if (aNameC[0] <> '/') or (aNameC[1] <> #0) then
    Result := -ESysENOENT
  else begin
    filler(aBuffer, '.', nil, 0);
    filler(aBuffer, '..', nil, 0);
    filler(aBuffer, xx, nil, 0); // PChar(hello_path) + 1, nil, 0);

    Result := 0;
  end;
end;

function hello_open(const aNameC : PChar; aFileInfo : PFuseFileInfo) : cint; cdecl;
var
  aName : String;
begin
  aName := aNameC;

  if aName <> hello_path then
    Result := -ESysENOENT
  else begin
    if ((aFileInfo^.flags and 3) <> O_RDONLY) then
      Result := -ESysEACCES
    else
      Result := 0;
  end;
end;

function hello_read(const aNameC : PChar; aBuffer : Pointer; aBufferSize : size_t; aFileOffset : off_t; aFileInfo : PFuseFileInfo) : Integer; cdecl;
var
  len : size_t;
  aName : String;
begin
  aName := aNameC;

  if aName <> hello_path then
    Result := -ESysENOENT
  else begin
    len := Length(hello_str);
    if (aFileOffset < len) then begin
      if (aFileOffset + aBufferSize > len) then
        aBufferSize := len - aFileOffset;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      move((PChar(hello_str) + aFileOffset)^,ABuffer^, aBufferSize);
=======
      move(aBuffer^, (PChar(hello_str) + aFileOffset)^, aBufferSize);
>>>>>>> graemeg/cpstrnew
=======
      move(aBuffer^, (PChar(hello_str) + aFileOffset)^, aBufferSize);
>>>>>>> graemeg/cpstrnew
=======
      move(aBuffer^, (PChar(hello_str) + aFileOffset)^, aBufferSize);
>>>>>>> graemeg/cpstrnew
=======
      move(aBuffer^, (PChar(hello_str) + aFileOffset)^, aBufferSize);
>>>>>>> origin/cpstrnew
    end else
      aBufferSize := 0;

    Result := aBufferSize;
  end;
end;

var
  hello_oper : TFuseOperations = (
    getattr	: @hello_getattr;
    readlink    : nil;
    getdir      : nil;
    mknod       : nil;
    mkdir       : nil;
    unlink      : nil;
    rmdir       : nil;
    symlink     : nil;
    rename      : nil;
    link        : nil;
    chmod       : nil;
    chown       : nil;
    truncate    : nil;
    utime       : nil;
    open	: @hello_open;
    read	: @hello_read;
    write       : nil;
    statfs      : nil;
    flush       : nil;
    release     : nil;
    fsync       : nil;
    setxattr    : nil;
    getxattr    : nil;
    listxattr   : nil;
    removexattr : nil;
    opendir     : nil;
    readdir	: @hello_readdir;
    releasedir  : nil;
    fsyncdir    : nil;
    init        : nil;
    destroy     : nil;
    access      : nil;
    create      : nil;
    ftruncate   : nil;
    fgetattr    : nil;
    lock        : nil;
    utimens     : nil;
    bmap        : nil;
  );

begin
  Halt(fuse_main(argc, argv, @hello_oper, Sizeof(hello_oper), nil));
end.
