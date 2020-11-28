{
    Copyright (c) 2018 by Florian Klaempfl

    Basic infrastructure for measuring timings of different compilation steps

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}

{$macro on}
{ "fix" the unit name }
{$define epiktimer:=cepiktimer}
{ do not depend on the classes unit }
{$DEFINE NOCLASSES}

{ include the original file }
{$i ../../epiktimer/epiktimer.pas}

