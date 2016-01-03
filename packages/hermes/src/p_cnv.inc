{
    Free Pascal port of the Hermes C library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

{
   C converter main loops for the HERMES library
   Copyright (c) 1998 Christian Nentwich (c.nentwich@cs.ucl.ac.uk)
   This source code is licensed under the GNU LGPL

   Please refer to the file COPYING.LIB contained in the distribution for
   licensing conditions
}

Procedure ConvertP(iface : PHermesConverterInterface); CDecl;

Begin
  { Simply loop through all scanlines }
  Repeat
    iface^.func(iface^.s_pixels, iface^.d_pixels, iface^.d_width, 1);

    Inc(iface^.s_pixels, iface^.s_pitch);
    Inc(iface^.d_pixels, iface^.d_pitch);
    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;

Procedure ConvertPStretch(iface : PHermesConverterInterface); CDecl;

Var
  dx, dy : DWord;
  y : DWord;

Begin
  y := 0;
  dy := (iface^.s_height Shl 16) Div iface^.d_height;
  dx := (iface^.s_width Shl 16) Div iface^.d_width;

  { We have the increment of y and x on the source surface now let's start }
  Repeat
    iface^.func(iface^.s_pixels, iface^.d_pixels, iface^.d_width, dx);

    Inc(iface^.d_pixels, iface^.d_pitch);

    Inc(y, dy);

    { Check how many lines we need to step forward }
    Inc(iface^.s_pixels, (y Shr 16)*DWord(iface^.s_pitch));
    y := y And $ffff;
    Dec(iface^.d_height);
  Until iface^.d_height = 0;
End;
