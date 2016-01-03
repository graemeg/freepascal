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

{Procedure Hermes_Calculate_Generic_Info(s_r, s_g, s_b, s_a, 
                                        d_r, d_g, d_b, d_a : int32;
					info : PHermesGenericInfo);
Function Hermes_Topbit(mask : int32) : Integer;}

Procedure Hermes_Calculate_Generic_Info(s_r, s_g, s_b, s_a, 
                                        d_r, d_g, d_b, d_a : Integer;
					info : PHermesGenericInfo);

Var
  r_right, g_right, b_right, a_right : Integer;

Begin
  {Calculate right shift amount for red. If it's <0, then set it to 0
   and calculate left shift amount}
  r_right := s_r - d_r;
  If r_right < 0 Then
  Begin
    info^.r_left := -r_right;
    info^.r_right := 0;
  End
  Else
  Begin
    info^.r_left := 0;
    info^.r_right := r_right;
  End;
  
  {Same for green}
  g_right := s_g - d_g;
  If g_right < 0 Then
  Begin
    info^.g_left := -g_right;
    info^.g_right := 0;
  End
  Else
  Begin
    info^.g_left := 0;
    info^.g_right := g_right;
  End;
  
  {Same for blue}
  b_right := s_b - d_b;
  If b_right < 0 Then
  Begin
    info^.b_left := -b_right;
    info^.b_right := 0;
  End
  Else
  Begin
    info^.b_left := 0;
    info^.b_right := b_right;
  End;
  
  {Alpha}
  a_right := s_a - d_a;
  If a_right < 0 Then
  Begin
    info^.a_left := -a_right;
    info^.a_right := 0;
  End
  Else
  Begin
    info^.a_left := 0;
    info^.a_right := a_right;
  End;
End;

Function Hermes_Topbit(mask : int32) : Integer;

Var
  i : Integer;

Begin
  If mask = 0 Then
  Begin
    Hermes_Topbit := 0;
    Exit;
  End;
  i := 0;
  While (mask And 1) = 0 Do
  Begin
    mask := mask Shr 1;
    Inc(i);
  End;
  While (mask And 1) = 1 Do
  Begin
    mask := mask Shr 1;
    Inc(i);
  End;
  Hermes_Topbit := i;
End;
