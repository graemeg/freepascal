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

{ This little definition makes everything much nicer below here }
{$MACRO ON}
{$define CONVERT_PARAMETERS:=source, dest : Pchar8; count, inc_source : DWord}

{$I p_16.inc}
{$I p_24.inc}
{$I p_32.inc}
{ $I p32aoblt.pp}
{$I p_clr.inc}
{$I p_cnv.inc}
{$I p_cpy.inc}
{$I p_g.inc}
{$I p_ga.inc}
{$I p_gac.inc}
{ $I p_gaoblt.pp}
{$I p_gca.inc}
{$I p_gcc.inc}
{ $I p_gccblt.pp}
{ $I p_gcoblt.pp}
{$I p_i8.inc}
{$I p_muhmu.inc}
{$I d_32.inc}
