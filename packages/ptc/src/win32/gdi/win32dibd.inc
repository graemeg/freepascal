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

type
  TWin32DIB = class
  private
    FBitmapInfo: PBITMAPINFO;
    FPixels: Pointer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FFormat: IPTCFormat;
=======
    FFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
    FFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
    FFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
    FFormat: TPTCFormat;
>>>>>>> origin/cpstrnew
    FWidth, FHeight, FPitch: Integer;
  public
    constructor Create(AWidth, AHeight: Integer);
    destructor Destroy; override;
    property BMI: PBITMAPINFO read FBitmapInfo;
    property Width: Integer read FWidth;
    property Height: Integer read FHeight;
    property Pitch: Integer read FPitch;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property Format: IPTCFormat read FFormat;
=======
    property Format: TPTCFormat read FFormat;
>>>>>>> graemeg/cpstrnew
=======
    property Format: TPTCFormat read FFormat;
>>>>>>> graemeg/cpstrnew
=======
    property Format: TPTCFormat read FFormat;
>>>>>>> graemeg/cpstrnew
=======
    property Format: TPTCFormat read FFormat;
>>>>>>> origin/cpstrnew
    property Pixels: Pointer read FPixels;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TWin32DIB = Class(TObject)
  Private
    FBitmapInfo : PBITMAPINFO;
    FPixels : Pointer;
    FFormat : TPTCFormat;
    FWidth, FHeight, FPitch : Integer;
  Public
    Constructor Create(AWidth, AHeight : Integer);
    Destructor Destroy; Override;
    Property BMI : PBITMAPINFO Read FBitmapInfo;
    Property Width : Integer Read FWidth;
    Property Height : Integer Read FHeight;
    Property Pitch : Integer Read FPitch;
    Property Format : TPTCFormat Read FFormat;
    Property Pixels : Pointer Read FPixels;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
