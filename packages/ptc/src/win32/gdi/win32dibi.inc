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


{TODO: create DIBs with the same color depth as the desktop}

constructor TWin32DIB.Create(AWidth, AHeight: Integer);
begin
  FBitmapInfo := GetMem(SizeOf(BITMAPINFOHEADER) + 12);

  FillChar(FBitmapInfo^.bmiHeader, SizeOf(BITMAPINFOHEADER), 0);
  with FBitmapInfo^.bmiHeader do
  begin
=======
=======
>>>>>>> origin/fixes_2_2

{TODO: create DIBs with the same color depth as the desktop}

Constructor TWin32DIB.Create(AWidth, AHeight : Integer);

Begin
  FBitmapInfo := GetMem(SizeOf(BITMAPINFOHEADER) + 12);

  FillChar(FBitmapInfo^.bmiHeader, SizeOf(BITMAPINFOHEADER), 0);
  With FBitmapInfo^.bmiHeader Do
  Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    biSize := SizeOf(BITMAPINFOHEADER);
    biWidth := AWidth;
    biHeight := -AHeight;
    biPlanes := 1;
    biBitCount := 32;
    biCompression := BI_BITFIELDS;
    biSizeImage := 0;
    biXPelsPerMeter := 0;
    biYPelsPerMeter := 0;
    biClrUsed := 0;
    biClrImportant := 0;
<<<<<<< HEAD
<<<<<<< HEAD
  end;
=======
  End;
>>>>>>> graemeg/fixes_2_2
=======
  End;
>>>>>>> origin/fixes_2_2

  PDWord(@FBitmapInfo^.bmiColors)[0] := $FF0000;
  PDWord(@FBitmapInfo^.bmiColors)[1] := $00FF00;
  PDWord(@FBitmapInfo^.bmiColors)[2] := $0000FF;

  FWidth := AWidth;
  FHeight := AHeight;
  FFormat := TPTCFormat.Create(32, $FF0000, $FF00, $FF);
  FPitch := FWidth * 4;

  FPixels := GetMem(AWidth * AHeight * 4);
  FillChar(FPixels^, AWidth * AHeight * 4, 0);
<<<<<<< HEAD
<<<<<<< HEAD
end;

destructor TWin32DIB.Destroy;
begin
  FreeMem(FPixels);
  FreeMem(FBitmapInfo);
<<<<<<< HEAD
=======
  FFormat.Free;
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
  inherited Destroy;
end;
=======
=======
>>>>>>> origin/fixes_2_2
End;

Destructor TWin32DIB.Destroy;

Begin
  FreeMem(FPixels);
  FreeMem(FBitmapInfo);
  FFormat.Free;
  Inherited Destroy;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
