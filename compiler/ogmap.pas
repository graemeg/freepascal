{
    Copyright (c) 2001-2002 by Peter Vreman

    Contains the class for generating a map file

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
unit ogmap;

{$i fpcdefs.inc}

interface

    uses
       { common }
       globtype,
       { object writer }
       ogbase
       ;

    type
       texemap = class
       private
         t : text;
         FImageBase : aword;
       public
         constructor Create(const s:string);
         destructor Destroy;override;
         procedure Flush;
         procedure Add(const s:string);
         procedure AddHeader(const s:string);
         procedure AddCommonSymbolsHeader;
         procedure AddCommonSymbol(p:TObjSymbol);
         procedure AddMemoryMapHeader(abase:aint);
         procedure AddMemoryMapExeSection(p:texesection);
         procedure AddMemoryMapObjectSection(p:TObjSection);
         procedure AddMemoryMapSymbol(p:TObjSymbol);
       end;

    var
      exemap : texemap;


implementation

    uses
      cutils,cfileutl;

    const
      HexTbl : array[0..15] of char='0123456789abcdef';

    function sizestr(v:aword):string;
      var
        tmp:array [0..19] of char;
        i:longint;
      begin
        if v=0 then
          result:='0x0'
        else
          begin
            i:=high(tmp);
            while (v>0) do
              begin
                tmp[i]:=hextbl[v and $f];
                v:=v shr 4;
                dec(i);
              end;
            tmp[i]:='x';
            tmp[i-1]:='0';
            setstring(result,@tmp[i-1],high(tmp)+2-i);
          end;
      end;

    function PadSpaceLeft(const s:string;len:longint):string;
      begin
        if length(s)<len then
          result:=Space(len-length(s))+s
        else
          result:=s;
      end;


{****************************************************************************
                                  TExeMap
****************************************************************************}

     constructor TExeMap.Create(const s:string);
       begin
         Assign(t,FixFileName(s));
         Rewrite(t);
         FImageBase:=0;
       end;


     destructor TExeMap.Destroy;
       begin
         Close(t);
       end;


     procedure TExeMap.Flush;
       begin
         System.Flush(t);
       end;


     procedure TExeMap.Add(const s:string);
       begin
         writeln(t,s);
       end;


     procedure TExeMap.AddHeader(const s:string);
       begin
         Add('');
         Add(s);
       end;


     procedure TExeMap.AddCommonSymbolsHeader;
       begin
         AddHeader('Allocating common symbols');
         Add('Common symbol       size              file');
         Add('');
       end;


     procedure TExeMap.AddCommonSymbol(p:TObjSymbol);
       var
         s : string;
       begin
         { Common symbol       size              file }
         s:=p.name;
         if length(s)>20 then
          begin
            writeln(t,p.name);
            s:='';
          end;
         Add(PadSpace(s,20)+PadSpace(sizestr(p.size),16)+p.objdata.name);
       end;


     procedure TExeMap.AddMemoryMapHeader(abase:aint);
       var
         imagebasestr : string;
       begin
         FImageBase:=abase;
         if FImageBase<>0 then
           imagebasestr:=' (ImageBase=0x'+HexStr(FImageBase,sizeof(pint)*2)+')'
         else
           imagebasestr:='';
         AddHeader('Memory map'+imagebasestr);
         Add('');
       end;


     procedure TExeMap.AddMemoryMapExeSection(p:texesection);
       begin
         { .text           0x000018a8     0xd958 }
         Add(PadSpace(p.name,15)+PadSpace(' '+p.MemPosStr(Fimagebase),12)+
             ' '+PadSpaceLeft(sizestr(p.size),9));
       end;


     procedure TExeMap.AddMemoryMapObjectSection(p:TObjSection);
       var
         secname : string;
       begin
         { .text           0x000018a8     0xd958     object.o }
         secname:=p.name;
         if Length(secname)>14 then
           begin
             Add(' '+secname);
             secname:='';
           end;
         Add(' '+PadSpace(secname,14)+PadSpace(' '+p.MemPosStr(FImageBase),12)+
             ' '+PadSpaceLeft(sizestr(p.size),9)+' '+p.objdata.name);
       end;


     procedure TExeMap.AddMemoryMapSymbol(p:TObjSymbol);
       begin
         {                 0x00001e30                setup_screens }
         Add(Space(16)+PadSpace(p.AddressStr(FImageBase),25)+' '+p.name);
       end;

end.
