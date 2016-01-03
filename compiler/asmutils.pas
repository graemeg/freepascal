{
    Copyright (c) 1998-2006 by Florian Klaempfl

    This unit contains utility functions for assembler output

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
unit asmutils;

interface

{$i fpcdefs.inc}

uses
  globtype,
  aasmbase,
  aasmdata;


    function emit_ansistring_const(list:TAsmList;data:PChar;len:LongInt;encoding:tstringencoding;NewSection:Boolean=True):TAsmLabel;
    function emit_unicodestring_const(list:TAsmList;data:Pointer;encoding:tstringencoding;Winlike:Boolean):TAsmLabel;


implementation

uses
  globals,
  systems,
  verbose,
  aasmtai,
  widestr,
  symdef;

    function emit_ansistring_const(list:TAsmList;data:PChar;len:LongInt;encoding:tstringencoding;NewSection:Boolean): TAsmLabel;
      var
        referencelab: TAsmLabel;
        s: PChar;
      begin
        current_asmdata.getdatalabel(result);
        if NewSection then
          new_section(list,sec_rodata,result.name,const_align(sizeof(pint)));
        referencelab := nil;
        if target_info.system in systems_darwin then
          begin
            current_asmdata.getdatalabel(referencelab);
            list.concat(tai_label.create(referencelab));
          end;
        list.concat(tai_const.create_16bit(encoding));
        list.concat(tai_const.create_16bit(1));
{$ifdef cpu64bitaddr}
        { dummy for alignment }
        list.concat(tai_const.create_32bit(0));
{$endif cpu64bitaddr}
        list.concat(tai_const.create_pint(-1));
        list.concat(tai_const.create_pint(len));
        { make sure the string doesn't get dead stripped if the header is referenced }
        if target_info.system in systems_darwin then
          list.concat(tai_directive.create(asd_reference,result.name));
        list.concat(tai_label.create(result));
        { and vice versa }
        if target_info.system in systems_darwin then
          list.concat(tai_directive.create(asd_reference,referencelab.name));

        getmem(s,len+1);
        move(data^,s^,len);
        s[len]:=#0;
        list.concat(tai_string.create_pchar(s,len+1)); { terminating zero included }
      end;


    function emit_unicodestring_const(list:TAsmList;data:Pointer;encoding:tstringencoding;Winlike:Boolean):TAsmLabel;
      var
        referencelab: TAsmLabel;
        i, strlength: SizeInt;
      begin
        current_asmdata.getdatalabel(result);
        new_section(list,sec_rodata,result.name,const_align(sizeof(pint)));
        referencelab := nil;
        if target_info.system in systems_darwin then
          begin
            current_asmdata.getdatalabel(referencelab);
            list.concat(tai_label.create(referencelab));
          end;
        strlength := getlengthwidestring(pcompilerwidestring(data));
        list.concat(tai_const.create_16bit(encoding));
        list.concat(tai_const.create_16bit(2));
{$ifdef cpu64bitaddr}
        { dummy for alignment }
        list.concat(Tai_const.Create_32bit(0));
{$endif cpu64bitaddr}
        if Winlike then
          list.concat(Tai_const.Create_32bit(strlength*cwidechartype.size))
        else
          begin
            list.concat(Tai_const.Create_pint(-1));
            list.concat(Tai_const.Create_pint(strlength));
          end;
        { make sure the string doesn't get dead stripped if the header is referenced }
        if (target_info.system in systems_darwin) then
          list.concat(tai_directive.create(asd_reference,result.name));
        list.concat(Tai_label.Create(result));
        { ... and vice versa }
        if (target_info.system in systems_darwin) then
          list.concat(tai_directive.create(asd_reference,referencelab.name));
        if cwidechartype.size = 2 then
          begin
            for i:=0 to strlength-1 do
              list.concat(Tai_const.Create_16bit(pcompilerwidestring(data)^.data[i]));
            { ending #0 }
            list.concat(Tai_const.Create_16bit(0));
          end
        else
          InternalError(200904271); { codegeneration for other sizes must be written }
      end;


end.
