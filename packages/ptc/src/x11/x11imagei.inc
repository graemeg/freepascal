<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2011 Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew
    Original C++ version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor TX11Image.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: IPTCFormat);
=======
constructor TX11Image.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11Image.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11Image.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11Image.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> origin/cpstrnew
begin
  FWidth := AWidth;
  FHeight := AHeight;
  FDisplay := ADisplay;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor TX11NormalImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: IPTCFormat);
=======
constructor TX11NormalImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11NormalImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11NormalImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11NormalImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> origin/cpstrnew
var
  xpad, xpitch: Integer;
  tmp_FPixels: PChar;
begin
  inherited;

  xpad := AFormat.Bits;
  if AFormat.Bits = 24 then
    xpad := 32;
  xpitch := AWidth * AFormat.Bits div 8;
  Inc(xpitch, 3);
  xpitch := xpitch and (not 3);
=======
=======
>>>>>>> origin/fixes_2_2
Const
{$WARNING this belongs to the ipc unit}
  IPC_PRIVATE = 0;

Constructor TX11Image.Create(ADisplay : PDisplay; AScreen, AWidth, AHeight : Integer; AFormat : TPTCFormat);

Begin
  FWidth := AWidth;
  FHeight := AHeight;
  FDisplay := ADisplay;
End;

Constructor TX11NormalImage.Create(ADisplay : PDisplay; AScreen, AWidth, AHeight : Integer; AFormat : TPTCFormat);

Var
  xpad, xpitch : Integer;
  tmp_FPixels : PChar;

Begin
  Inherited;

  xpad := AFormat.Bits;
  If AFormat.Bits = 24 Then
    xpad := 32;
  xpitch := AWidth * AFormat.Bits Div 8;
  Inc(xpitch, 3);
  xpitch := xpitch And (Not 3);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FPixels := GetMem(xpitch * AHeight);
  Pointer(tmp_FPixels) := Pointer(FPixels);
  FImage := XCreateImage(ADisplay, DefaultVisual(ADisplay, AScreen),
                         DefaultDepth(ADisplay, AScreen),
                         ZPixmap, 0, tmp_FPixels,
                         AWidth, AHeight, xpad, 0);
<<<<<<< HEAD
<<<<<<< HEAD
  if FImage = nil then
    raise TPTCError.Create('cannot create XImage');
end;

destructor TX11NormalImage.Destroy;
begin
  if FImage <> nil then
  begin
    { Restore XImage's buffer pointer }
    FImage^.data := nil;
    XDestroyImage(FImage);
  end;

  if FPixels <> nil then
    FreeMem(FPixels);

  inherited Destroy;
end;

procedure TX11NormalImage.Put(AWindow: TWindow; AGC: TGC; AX, AY: Integer);
begin
  XPutImage(FDisplay, AWindow, AGC, FImage, 0, 0, AX, AY, FWidth, FHeight);
  XSync(FDisplay, False);
end;

procedure TX11NormalImage.Put(AWindow: TWindow; AGC: TGC; ASX, ASY, ADX, ADY,
                    AWidth, AHeight: Integer);
begin
  XPutImage(FDisplay, AWindow, AGC, FImage, ASX, ASY, ADX, ADY, AWidth, AHeight);
  XSync(FDisplay, False);
end;

function TX11NormalImage.Lock: Pointer;
begin
  Result := FPixels;
end;

function TX11NormalImage.Pitch: Integer;
begin
  Result := FImage^.bytes_per_line;
end;

function TX11NormalImage.Name: string;
begin
  Result := 'XImage';
end;

{$IFDEF ENABLE_X11_EXTENSION_XSHM}

var
  Fshm_error: Boolean;
  Fshm_oldhandler: Function(disp: PDisplay; xev: PXErrorEvent): Integer; cdecl;

function Fshm_errorhandler(disp: PDisplay; xev: PXErrorEvent): Integer; cdecl;
begin
  if xev^.error_code=BadAccess then
  begin
    Fshm_error := True;
    Result := 0;
  end
  else
    Result := Fshm_oldhandler(disp, xev);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor TX11ShmImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: IPTCFormat);
=======
constructor TX11ShmImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11ShmImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11ShmImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TX11ShmImage.Create(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer; AFormat: TPTCFormat);
>>>>>>> origin/cpstrnew
begin
  inherited;
=======
=======
>>>>>>> origin/fixes_2_2
  If FImage = Nil Then
    Raise TPTCError.Create('cannot create XImage');
End;

Destructor TX11NormalImage.Destroy;

Begin
  If FImage <> Nil Then
  Begin
    { Restore XImage's buffer pointer }
    FImage^.data := Nil;
    XDestroyImage(FImage);
  End;

  If FPixels <> Nil Then
    FreeMem(FPixels);

  Inherited Destroy;
End;

Procedure TX11NormalImage.Put(AWindow : TWindow; AGC : TGC; AX, AY : Integer);

Begin
  XPutImage(FDisplay, AWindow, AGC, FImage, 0, 0, AX, AY, FWidth, FHeight);
  XSync(FDisplay, False);
End;

Procedure TX11NormalImage.Put(AWindow : TWindow; AGC : TGC; ASX, ASY, ADX, ADY,
                    AWidth, AHeight : Integer);

Begin
  XPutImage(FDisplay, AWindow, AGC, FImage, ASX, ASY, ADX, ADY, AWidth, AHeight);
  XSync(FDisplay, False);
End;

Function TX11NormalImage.Lock : Pointer;

Begin
  Result := FPixels;
End;

Function TX11NormalImage.Pitch : Integer;

Begin
  Result := FImage^.bytes_per_line;
End;

Function TX11NormalImage.Name : String;

Begin
  Result := 'XImage';
End;

{$IFDEF ENABLE_X11_EXTENSION_XSHM}

Var
  Fshm_error : Boolean;
  Fshm_oldhandler : Function(disp : PDisplay; xev : PXErrorEvent) : Integer; CDecl;

Function Fshm_errorhandler(disp : PDisplay; xev : PXErrorEvent) : Integer; CDecl;

Begin
  If xev^.error_code=BadAccess Then
  Begin
    Fshm_error := True;
    Result := 0;
  End
  Else
    Result := Fshm_oldhandler(disp, xev);
End;

Constructor TX11ShmImage.Create(ADisplay : PDisplay; AScreen, AWidth, AHeight : Integer; AFormat : TPTCFormat);

Begin
  Inherited;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FShmInfo.shmid := -1;
  FShmInfo.shmaddr := Pointer(-1);
  FImage := XShmCreateImage(ADisplay, DefaultVisual(ADisplay, AScreen),
                            DefaultDepth(ADisplay, AScreen),
                            ZPixmap, Nil, @FShmInfo, AWidth, AHeight);
<<<<<<< HEAD
<<<<<<< HEAD
  if FImage = nil then
    raise TPTCError.Create('cannot create SHM image');

  FShmInfo.shmid := shmget(IPC_PRIVATE, FImage^.bytes_per_line * FImage^.height,
                           IPC_CREAT or &777);
  if FShmInfo.shmid = -1 then
    raise TPTCError.Create('cannot get shared memory segment');

  FShmInfo.shmaddr := shmat(FShmInfo.shmid, Nil, 0);
  FShmInfo.readOnly := Ord(False);
  FImage^.data := FShmInfo.shmaddr;

  if Pointer(FShmInfo.shmaddr) = Pointer(-1) then
    raise TPTCError.Create('cannot allocate shared memory');

  // try and attach the segment to the server. Bugfix: Have to catch
  // bad access errors in case it runs over the net.
  Fshm_error := False;
  Fshm_oldhandler := XSetErrorHandler(@Fshm_errorhandler);
  try
    if XShmAttach(ADisplay, @FShmInfo) = 0 then
      raise TPTCError.Create('cannot attach shared memory segment to display');

    XSync(ADisplay, False);
    if Fshm_error then
      raise TPTCError.Create('cannot attach shared memory segment to display');
    FShmAttached := True;
  finally
    XSetErrorHandler(Fshm_oldhandler);
  end;
end;

destructor TX11ShmImage.Destroy;
begin
  if FShmAttached then
  begin
    XShmDetach(FDisplay, @FShmInfo);
    XSync(FDisplay, False);
  end;
  if FImage <> nil then
    XDestroyImage(FImage);
  if Pointer(FShmInfo.shmaddr) <> Pointer(-1) then
    shmdt(FShmInfo.shmaddr);
  if FShmInfo.shmid <> -1 then
    shmctl(FShmInfo.shmid, IPC_RMID, Nil);

  inherited Destroy;
end;

procedure TX11ShmImage.Put(AWindow: TWindow; AGC: TGC; AX, AY: Integer);
begin
  XShmPutImage(FDisplay, AWindow, AGC, FImage, 0, 0, AX, AY, FWidth, FHeight, False);
  XSync(FDisplay, False);
end;

procedure TX11ShmImage.Put(AWindow: TWindow; AGC: TGC; ASX, ASY, ADX, ADY,
                    AWidth, AHeight: Integer);
begin
  XShmPutImage(FDisplay, AWindow, AGC, FImage, ASX, ASY, ADX, ADY, FWidth, FHeight, False);
  XSync(FDisplay, False);
end;

function TX11ShmImage.Lock: Pointer;
begin
  Result := Pointer(FShmInfo.shmaddr);
end;

function TX11ShmImage.Pitch: Integer;
begin
  Result := FImage^.bytes_per_line;
end;

function TX11ShmImage.Name: string;
begin
  Result := 'MIT-Shm';
end;
=======
=======
>>>>>>> origin/fixes_2_2
  If FImage = Nil Then
    Raise TPTCError.Create('cannot create SHM image');

  FShmInfo.shmid := shmget(IPC_PRIVATE, FImage^.bytes_per_line * FImage^.height,
                           IPC_CREAT Or &777);
  If FShmInfo.shmid = -1 Then
    Raise TPTCError.Create('cannot get shared memory segment');

  FShmInfo.shmaddr := shmat(FShmInfo.shmid, Nil, 0);
  FShmInfo.readOnly := False;
  FImage^.data := FShmInfo.shmaddr;

  If Pointer(FShmInfo.shmaddr) = Pointer(-1) Then
    Raise TPTCError.Create('cannot allocate shared memory');

  // Try and attach the segment to the server. Bugfix: Have to catch
  // bad access errors in case it runs over the net.
  Fshm_error := False;
  Fshm_oldhandler := XSetErrorHandler(@Fshm_errorhandler);
  Try
    If XShmAttach(ADisplay, @FShmInfo) = 0 Then
      Raise TPTCError.Create('cannot attach shared memory segment to display');

    XSync(ADisplay, False);
    If Fshm_error Then
      Raise TPTCError.Create('cannot attach shared memory segment to display');
    FShmAttached := True;
  Finally
    XSetErrorHandler(Fshm_oldhandler);
  End;
End;

Destructor TX11ShmImage.Destroy;

Begin
  If FShmAttached Then
  Begin
    XShmDetach(FDisplay, @FShmInfo);
    XSync(FDisplay, False);
  End;
  If FImage <> Nil Then
    XDestroyImage(FImage);
  If Pointer(FShmInfo.shmaddr) <> Pointer(-1) Then
    shmdt(FShmInfo.shmaddr);
  If FShmInfo.shmid <> -1 Then
    shmctl(FShmInfo.shmid, IPC_RMID, Nil);

  Inherited Destroy;
End;

Procedure TX11ShmImage.Put(AWindow : TWindow; AGC : TGC; AX, AY : Integer);

Begin
  XShmPutImage(FDisplay, AWindow, AGC, FImage, 0, 0, AX, AY, FWidth, FHeight, False);
  XSync(FDisplay, False);
End;

Procedure TX11ShmImage.Put(AWindow : TWindow; AGC : TGC; ASX, ASY, ADX, ADY,
                    AWidth, AHeight : Integer);

Begin
  XShmPutImage(FDisplay, AWindow, AGC, FImage, ASX, ASY, ADX, ADY, FWidth, FHeight, False);
  XSync(FDisplay, False);
End;

Function TX11ShmImage.Lock : Pointer;

Begin
  Result := Pointer(FShmInfo.shmaddr);
End;

Function TX11ShmImage.Pitch : Integer;

Begin
  Result := FImage^.bytes_per_line;
End;

Function TX11ShmImage.Name : String;

Begin
  Result := 'MIT-Shm';
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF ENABLE_X11_EXTENSION_XSHM}
