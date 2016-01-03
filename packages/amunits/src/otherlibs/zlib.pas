{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for zlib.library

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{
  History:

  First version of this unit.
  17 Jan 2003.

  Changed cardinal > longword.
  Changed startcode for unit.
  12 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se
}


{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT ZLIB;

INTERFACE
USES Exec;

VAR ZLibBase : pLibrary;

const
    ZLIBNAME : PChar = 'zlib.library';

  { Version 1.0  }
  { Compression strategy  }
     GZ_STRATEGY_DEFAULT = 0;
     GZ_STRATEGY_FILTERED = 1;
     GZ_STRATEGY_HUFFMAN = 2;
  { some often used compression levels  }
     GZ_COMPRESS_NO = 0;
     GZ_COMPRESS_FASTEST = 1;
     GZ_COMPRESS_DEFAULT = 6;
     GZ_COMPRESS_BEST = 9;


FUNCTION GZ_Close(handle : POINTER) : LONGINT;
FUNCTION GZ_CompressMem(srcbuf : POINTER; srclen : longword; destbuf : POINTER; destlen : longword; strategy : longword; level : longword; VAR poutlen : longword) : LONGINT;
FUNCTION GZ_DecompressMem(srcbuf : POINTER; srclen : longword; destbuf : POINTER; destlen : longword) : LONGINT;
FUNCTION GZ_FGetC(handle : POINTER) : pLONGINT;
FUNCTION GZ_FGetS(handle : POINTER; buf : pCHAR; len : longword) : pCHAR;
FUNCTION GZ_FileLength(handle : POINTER) : longword;
FUNCTION GZ_Open(filename : pCHAR; openmode : longword; strategy : longword; level : longword) : POINTER;
FUNCTION GZ_OpenFromFH(fh : LONGINT; openmode : longword; strategy : longword; level : longword) : POINTER;
FUNCTION GZ_Read(handle : POINTER; buf : POINTER; len : longword) : LONGINT;
FUNCTION GZ_Write(handle : POINTER; buf : POINTER; len : longword) : LONGINT;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitZLIBLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    ZLIBIsCompiledHow : longint;

IMPLEMENTATION

{$ifndef dont_use_openlib}
uses msgbox;
{$endif dont_use_openlib}

FUNCTION GZ_Close(handle : POINTER) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L handle,A0
        MOVEA.L ZLibBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_CompressMem(srcbuf : POINTER; srclen : longword; destbuf : POINTER; destlen : longword; strategy : longword; level : longword; VAR poutlen : longword) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L srcbuf,A0
        MOVE.L  srclen,D0
        MOVEA.L destbuf,A1
        MOVE.L  destlen,D1
        MOVE.L  strategy,D2
        MOVE.L  level,D3
        MOVEA.L poutlen,A2
        MOVEA.L ZLibBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_DecompressMem(srcbuf : POINTER; srclen : longword; destbuf : POINTER; destlen : longword) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L srcbuf,A0
        MOVE.L  srclen,D0
        MOVEA.L destbuf,A1
        MOVE.L  destlen,D1
        MOVEA.L ZLibBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_FGetC(handle : POINTER) : pLONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L handle,A0
        MOVEA.L ZLibBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_FGetS(handle : POINTER; buf : pCHAR; len : longword) : pCHAR;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L handle,A0
        MOVEA.L buf,A1
        MOVE.L  len,D0
        MOVEA.L ZLibBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_FileLength(handle : POINTER) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L handle,A0
        MOVEA.L ZLibBase,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_Open(filename : pCHAR; openmode : longword; strategy : longword; level : longword) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L filename,A0
        MOVE.L  openmode,D0
        MOVE.L  strategy,D1
        MOVE.L  level,D2
        MOVEA.L ZLibBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_OpenFromFH(fh : LONGINT; openmode : longword; strategy : longword; level : longword) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L fh,A0
        MOVE.L  openmode,D0
        MOVE.L  strategy,D1
        MOVE.L  level,D2
        MOVEA.L ZLibBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_Read(handle : POINTER; buf : POINTER; len : longword) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L handle,A0
        MOVEA.L buf,A1
        MOVE.L  len,D0
        MOVEA.L ZLibBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GZ_Write(handle : POINTER; buf : POINTER; len : longword) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L handle,A0
        MOVEA.L buf,A1
        MOVE.L  len,D0
        MOVEA.L ZLibBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of zlib.library}
  {$Info don't forget to use InitZLIBLibrary in the beginning of your program}

var
    zlib_exit : Pointer;

procedure ClosezlibLibrary;
begin
    ExitProc := zlib_exit;
    if ZLibBase <> nil then begin
        CloseLibrary(ZLibBase);
        ZLibBase := nil;
    end;
end;

procedure InitZLIBLibrary;
begin
    ZLibBase := nil;
    ZLibBase := OpenLibrary(ZLIBNAME,LIBVERSION);
    if ZLibBase <> nil then begin
        zlib_exit := ExitProc;
        ExitProc := @ClosezlibLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open zlib.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    ZLIBIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of zlib.library}

var
    zlib_exit : Pointer;

procedure ClosezlibLibrary;
begin
    ExitProc := zlib_exit;
    if ZLibBase <> nil then begin
        CloseLibrary(ZLibBase);
        ZLibBase := nil;
    end;
end;

begin
    ZLibBase := nil;
    ZLibBase := OpenLibrary(ZLIBNAME,LIBVERSION);
    if ZLibBase <> nil then begin
        zlib_exit := ExitProc;
        ExitProc := @ClosezlibLibrary;
        ZLIBIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open zlib.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    ZLIBIsCompiledHow := 3;
   {$Warning No autoopening of zlib.library compiled}
   {$Warning Make sure you open zlib.library yourself}
{$endif dont_use_openlib}

END. (* UNIT ZLIB *)



