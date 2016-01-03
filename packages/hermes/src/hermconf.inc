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


Const
  HERMES_INITIAL = 8; { Initial size and growth of dynamic }
  HERMES_GROWTH = 4; { array structures }

  A_32 = 3;		      { oh yes, hardcode it... tsk tsk tsk }
  R_32 = 2;                   { Position of R,G,B in a 32 bit dword }
  G_32 = 1;
  B_32 = 0;
  R_24 = 2;                   { Position of R,G,B in a 24 bit pixel }
  G_24 = 1;
  B_24 = 0;
{  #define DWORD_BYTE0(s) s         /* Shift values to get to bytes in dwords */
  #define DWORD_BYTE1(s) ((s)<<8)  /* Check out the bracket paranoia to */   
  #define DWORD_BYTE2(s) ((s)<<16) /* prevent bugs :)) */     
  #define DWORD_BYTE3(s) ((s)<<24)       
  #define DWORD_SHORT0(s) s
  #define DWORD_SHORT1(s) ((s)<<16)
  #define DWORD_SHORT0(s) s
  #define DWORD_SHORT1(s) ((s)<<16)}
