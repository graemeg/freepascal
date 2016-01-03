(*****************************************************************************
*   This program is free software;  you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY;  without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
*   the GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program;  if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*****************************************************************************)
uses lvm;

// lvmtest.cpp, program to test the LVM library
//
// By John Martin Alfredsson, jma@jmast.se
// Pascal conversion by Yuri Prokushev

var
  vca: Volume_Control_Array;
  vir: Volume_Information_Record;
  iCounter: integer;
const
  Error_Code: CARDINAL32 = 99999;
begin
  Open_LVM_Engine(True, addr(Error_Code));
  if Error_Code<>0 then
  begin
    writeln('Open_LVM_Engine Error !!');
    halt(1);
  end;

  vca:=Get_Volume_Control_Data(addr(Error_Code));
  if Error_Code<>0 then
  begin
    writeln('Get_Volume_Control_Data Error !!');
    halt(1);
  end;

  for iCounter:=0 to vca.Count-1 do
  begin
    writeln('--------------------------------------');
    vir:=Get_Volume_Information(vca.Volume_Control_Data[iCounter].Volume_Handle, addr(Error_Code));
    writeln('Volname      : [', vir.Current_Drive_Letter, ':] ', vir.Volume_Name);
    writeln('FileSystem   : ', vir.File_System_Name);
    case vir.Status of
      0: writeln('Status       : None');
      1: writeln('Status       : Bootable');
      2: writeln('Status       : Startable');
      3: writeln('Status       : Installable');
    end;

    if vca.Volume_Control_Data[iCounter].Compatibility_Volume then
      writeln('Volume type  : Compatibility Volume')
    else
      writeln('Volume type  : LVM Volume');
  end;

  writeln('--------------------------------------');

  Free_Engine_Memory(vca.Volume_Control_Data);
  Close_LVM_Engine;
end.
