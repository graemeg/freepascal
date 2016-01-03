{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2002 by the Free Pascal development team

    BIOS functions unit for Nintendo DS
    Copyright (c) 2006 by Francesco Lombardi

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 *****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function IsARM9(): boolean;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
type
  TDoOpen     = procedure (var f; p: pchar; flags: longint);
  TDoClose    = procedure (handle: THandle);
  TDoWrite    = function  (h: THandle; addr: pointer; len: longint): longint;
  TDoRead     = function  (h: THandle; addr: pointer; len: longint): longint;
  TDoSeek     = procedure (handle: THandle; pos: longint);
  TDoSeekend  = function  (handle: THandle): longint;
  TDoErase    = procedure (p: pchar);
  TDoRename   = procedure (p1, p2: pchar);
  TDoFilepos  = function  (handle: THandle): longint;
  TDoFilesize = function  (handle: THandle): longint;
  TDoTruncate = procedure (handle: THandle; pos: longint);
  TDoIsdevice = function  (handle: THandle): boolean;

  TFileIO = packed record
    DoOpen    : TDoOpen; 
    DoClose   : TDoClose;
    DoWrite   : TDoWrite;
    DoRead    : TDoRead;
    DoSeek    : TDoSeek;
    DoSeekend : TDoSeekend;
    DoErase   : TDoErase;
    DoRename  : TDoRename;
    DoFilepos : TDoFilepos;
    DoFilesize: TDoFilesize;
    DoTruncate: TDoTruncate;
    DoIsdevice: TDoIsdevice;
  end;
  PFileIO = ^TFileIO;

  TDoMkdir  = procedure (const s: string);
  TDoRmdir  = procedure (const s: string);
  TDoChdir  = procedure (const s: string);
  TDoGetdir = procedure (DriveNr: byte; var Dir: ShortString);

  TDirIO = packed record
    DoMkdir : TDoMkdir;
    DoRmdir : TDoRmdir;
    DoChdir : TDoChdir;
    DoGetdir: TDoGetdir;
  end;
  PDirIO = ^TDirIO;

  TFileIODevice = packed record
    FileIO: TFileIO;
    DirIO: TDirIO;
  end;
  PFileIODevice = ^TFileIODevice;


function IsARM9(): boolean;


procedure AssignDevice(const FIOD: TFileIODevice);

var
  FileIODevice: TFileIODevice = (
    FileIO: ( 
              DoOpen: nil;
              DoClose: nil;
              DoWrite: nil;
              DoRead: nil;
              DoSeek: nil;
              DoSeekend: nil;
              DoErase: nil;
              DoRename: nil;
              DoFilepos: nil;
              DoFilesize: nil;
              DoTruncate: nil;
              DoIsdevice: nil;
            );
    DirIO:  ( 
              DoMkdir:  nil;
              DoRmdir:  nil;
              DoChdir:  nil;
              DoGetdir: nil;
            );
  );
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
