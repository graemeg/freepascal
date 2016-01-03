{
    Copyright (c) 1998-2000 by Florian Klaempfl

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

 ****************************************************************************}
unit options;
interface

const
   version = '0.99.16';

var
   inputfilename, outputfilename : string; { Filenames }
   LibFileName, unitname         : string; { external library name }
   CompactMode,
   stripinfo,                 { Don't write info comments to output }
   UseLib,                    { Append external to implementation ?  }
   UseName,                   { Append 'libname name 'funcname ' }
   UsePPOinters,              { Use P instead of ^ for pointers    }
   EnumToConst,               { Write enumeration types as constants }
   Win32headers,              { allows dec_specifier }
   stripcomment,              { strip comments from inputfile }
   PrependTypes,              { Print T in front of type names ?   }
   UseCTypesUnit,             { Use types defined in the ctypes unit}
   createdynlib,              { creates a unit which loads dynamically the imports to proc vars }
   RemoveUnderscore : Boolean;
   usevarparas : boolean;     { generate var parameters, when a pointer }
                              { is passed                               }
   includefile : boolean;     { creates an include file instead of a unit }
   palmpilot : boolean;       { handling of PalmOS SYS_CALLs }
   packrecords: boolean;      { All records should be packed in the file }

{ Helpers }
Function ForceExtension(Const HStr,ext:String):String;
Function MaybeExtension(Const HStr,ext:String):String;

{ Options }
Procedure ProcessOptions;


Implementation


{*****************************************************************************
                                 Helpers
*****************************************************************************}

Function ForceExtension(Const HStr,ext:String):String;
{
  Return a filename which certainly has the extension ext
  (no dot in ext !!)
}
var
  j : longint;
begin
  j:=length(Hstr);
  while (j>0) and (Hstr[j]<>'.') do
   dec(j);
  if j=0 then
   j:=255;
  ForceExtension:=Copy(Hstr,1,j-1)+'.'+Ext;
end;


Function MaybeExtension(Const HStr,ext:String):String;
{
  Return a filename which certainly has the extension ext
  (no dot in ext !!)
}
var
  j : longint;
begin
  j:=length(Hstr);
  while (j>0) and (Hstr[j]<>'.') do
   dec(j);
  if j=0 then
   MaybeExtension:=Hstr+'.'+Ext
  else
   MaybeExtension:=Hstr;
end;

function ExtractFileName(const AFilePath : String): String;
var i : Integer;
begin
  i := Length(AFilePath);
  while (i>0) and (AFilePath[i]<>DirectorySeparator) and
    (AFilePath[i]<>DriveSeparator) do
  begin
    Dec(i);
  end;
  ExtractFileName := Copy(AFilePath,i+1,Length(AFilePath)); 
end;

{*****************************************************************************
                                Options
*****************************************************************************}

Procedure Usage;
begin
  writeln ('Usage : ',paramstr(0),' [options]  filename');
  writeln ('        Where [options] is one or more of:');
  writeln ('        -d                 Use external;');
  writeln ('        -D                 use external libname name ''func_name'';');
  writeln ('        -e                 change enum type to list of constants');
  writeln ('        -c                 Compact outputmode, less spaces and empty lines');
  WriteLn ('        -C                 Use types in ctypes unit');
  writeln ('        -i                 create include files (no unit header)');
  writeln ('        -l libname         Specify the library name for external');
  writeln ('        -o outputfilename  Specify the outputfilename');
  writeln ('        -p                 Use "P" instead of "^" for pointers');
  writeln ('        -pr                Pack all records (1 byte alignment)');
  writeln ('        -P                 use proc. vars for imports');
  writeln ('        -s                 strip comments from inputfile');
  writeln ('        -S                 strip comments and don''t write info to outputfile.');
  writeln ('        -t                 Prepend typedef type names with T');
  writeln ('        -T                 Prepend typedef type names with T, and remove _');
  writeln ('        -u unitname        Specify the name of the unit.');
  writeln ('        -v                 replace pointer parameters by call by');
  writeln ('                           reference parameters');
  writeln ('        -w                 special for win32 headers');
  writeln ('        -x                 handle SYS_TRAP of PalmOS header files');
  halt (0);
end;


Procedure ProcessOptions;
Var
  cp : string;
  I : longint;

  Function GetNextParam (const Opt,Name : String) : string;
  begin
   if i=paramcount then
    begin
      writeln ('Error : -',Opt,' : ',name,' expected');
      halt(1);
    end
   else
    begin
      GetNextParam:=paramstr(i+1);
      inc(i);
    end;
  end;

begin
  if paramcount=0 then
    Usage;
  inputfilename:='';
  outputfilename:='';
  LibFileName:='';
  UnitName:='';
  CompactMode:=false;
  UseLib:=false;
  UseName:=false;
  StripComment:=false;
  StripInfo:=false;
  UsePPointers:=false;
  UseCTypesUnit := false;
  EnumToCOnst:=false;
  usevarparas:=false;
  palmpilot:=false;
  includefile:=false;
  packrecords:=false;
  createdynlib:=false;
  i:=1;
  while i<=paramcount do
   begin
     cp:=paramstr(i);
     if cp[1]='-' then
      begin
        case cp[2] of
         'c' : CompactMode:=true;
         'C' : UseCTypesUnit := true;
         'e' : EnumToConst :=true;
         'd' : UseLib      :=true;
         'D' : begin
                 UseLib      :=true;
                 usename     :=true;
               end;
         'i' : includefile:=true;
         'l' : LibFileName:=GetNextParam ('l','libname');
         'o' : outputfilename:=GetNextParam('o','outputfilename');
         'P' : createdynlib:=true;
         'p' : begin
                  if (cp[3] = 'r') then
                     begin
                        PackRecords := true;
                     end
                  else
                      UsePPointers:=true;
               end;
         's' : stripcomment:=true;
         'S' : begin
                 stripcomment:=true;
                 stripinfo:=true;
               end;
         't' : PrependTypes:=true;
         'T' : begin
                 PrependTypes:=true;
                 RemoveUnderscore:=true;
               end;
         'u' : UnitName:=GetNextParam ('u','unitname');
         'v' : usevarparas:=true;
         'w' : begin
                  Win32headers:=true;
                  UseLib:=true;
                  usename:=true;
                  usevarparas:=true;
                  LibFileName:='kernel32';
               end;
         'x' : palmpilot:=true;
         else
           Writeln ('Illegal option : ',cp);
         end
      end
     else
      begin { filename }
        if inputfilename<>'' then
         begin
           writeln ('Error : only one filename supported. Found also :',cp);
           halt(1);
         end;
        inputfilename:=MaybeExtension(cp,'h');
        if outputfilename='' then
         outputfilename:=ForceExtension (inputfilename,'pp');
      end;
     inc(i);
   end;
  If inputfilename='' then
    Usage;
  if UnitName='' then
   begin
     UnitName := ExtractFileName(outputfilename);
     i:=pos('.',UnitName)-1;
     if i>0 then
      UnitName:=Copy(UnitName,1,i);
   end;
end;

end.
