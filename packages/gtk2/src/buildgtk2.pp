{
   Dummy unit to compile everything in one go

   This unit is part of gtk2forpascal.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the
<<<<<<< HEAD
<<<<<<< HEAD
   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
unit buildgtk2; // keep unit name lowercase for kylix

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
  gtk2, libglade2,gdkglext,gtkglext, gtk2ext
{$ifdef unix}  
  ,gdk2x
{$endif unix}
  , pangocairo;
=======
=======
>>>>>>> origin/fixes_2_2
  gtk2, libglade2,gdkglext,gtkglext,gtkhtml,gtk2ext
{$ifdef unix}  
  ,gdk2x
{$endif unix}
  ;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

end.
