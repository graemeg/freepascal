{ Copyright (C) <2005> <Andrew Haines> chmspecialfiles.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
<<<<<<< HEAD
<<<<<<< HEAD
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
{
  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.
}
unit chmspecialfiles;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, chmtypes;
  

  
  function WriteNameListToStream(const AStream: TStream; SectionNames: TSectionNames): Integer;
  function WriteControlDataToStream(const AStream: TStream; const LZXResetInterval, WindowSize, CacheSize: DWord): Integer;
  function WriteSpanInfoToStream(const AStream: TStream; UncompressedSize: QWord): Integer;
  function WriteTransformListToStream(const AStream: TStream): Integer;
  function WriteResetTableToStream(const AStream: TStream; ResetTableStream: TMemoryStream): Integer;
  function WriteContentToStream(const AStream: TStream; ContentStream: TStream): Integer;
  
implementation

function WriteNameListToStream(const AStream: TStream; SectionNames: TSectionNames): Integer;
var
  MSCompressedName: WideString = 'MSCompressed'#0; // Length 13
  UnCompressedName: WideString = 'Uncompressed'#0;
{$IFDEF ENDIAN_BIG}
  I: Integer;
{$ENDIF}
  Size: Word = 2;
  NEntries: Word = 0;
begin
  //  ::DataSpace/NameList
  {$IFDEF ENDIAN_BIG}
    for I := 1 to 13 do begin
      PWord(@MSCompressedName[I])^ := NToLE(PWord(@MSCompressedName[I])^);
      PWord(@UnCompressedName[I])^ := NToLE(PWord(@UnCompressedName[I])^);
    end;
  {$ENDIF}

  if snUnCompressed in SectionNames then begin
    Inc(Size, 14);
    Inc(NEntries);
  end;

  if snMSCompressed in SectionNames then begin
    Inc(Size, 14);
    Inc(NEntries);
  end;
  
  AStream.WriteWord(NToLE(Size));
  AStream.WriteWord(NToLE(NEntries));
  if snUnCompressed in SectionNames then begin
    AStream.WriteWord(NToLE(Word(12)));
    AStream.Write(UnCompressedName[1], 13*2);
  end;
  if snMSCompressed in SectionNames then begin
    AStream.WriteWord(NToLE(Word(12)));
    AStream.Write(MSCompressedName[1], 13*2);
  end;
  
  Result := Size * SizeOf(Word);
end;

function WriteControlDataToStream(const AStream: TStream; const LZXResetInterval,
  WindowSize, CacheSize: DWord): Integer;
var
  LZXC: array [0..3] of char = 'LZXC';
begin
  //  ::DataSpace/Storage/MSCompressed/ControlData
  Result := AStream.Position;
  AStream.WriteDWord(NToLE(DWord(6))); // number of dwords following this one
  AStream.Write(LZXC, 4);
  AStream.WriteDWord(NToLE(DWord(2))); // Version
  AStream.WriteDWord(NToLE(LZXResetInterval));
  AStream.WriteDWord(NToLE(WindowSize));
  AStream.WriteDWord(NToLE(CacheSize)); // what is this??
  AStream.WriteDWord(0);
  Result := AStream.Position - Result;
end;

function WriteSpanInfoToStream(const AStream: TStream; UncompressedSize: QWord): Integer;
begin
  //  ::DataSpace/Storage/MSCompressed/SpanInfo
  Result := AStream.Write(NToLE(UncompressedSize), SizeOf(QWord));
end;

function WriteTransformListToStream(const AStream: TStream): Integer;
const
//AGuid = '{7FC28940-9D31-11D0-9B27-00A0C91E9C7C}';
// use the broken guid
AGuid = '{'#0'7'#0'F'#0'C'#0'2'#0'8'#0'9'#0'4'#0'0'#0'-'#0'9'#0'D'#0'3'#0'1'#0'-'#0'1'#0'1'#0'D'#0'0'#0; //-9B27-00A0C91E9C7C}';
begin
  //  ::DataSpace/Storage/MSCompressed/Transform/List
  Result := AStream.Write(AGuid, SizeOf(AGuid));
end;

function WriteResetTableToStream(const AStream: TStream;
  ResetTableStream: TMemoryStream): Integer;
begin
  //  ::DataSpace/Storage/MSCompressed/Transform/{7FC28940-9D31-11D0-9B27-00A0C91E9C7C}/InstanceData/
  //  ::DataSpace/Storage/MSCompressed/Transform/{7FC28940-9D31-11D0-9B27-00A0C91E9C7C}/InstanceData/ResetTable
  ResetTableStream.Position := 0;
  Result := AStream.CopyFrom(ResetTableStream, ResetTableStream.Size-SizeOf(QWord));
end;

function WriteContentToStream(const AStream: TStream; ContentStream: TStream): Integer;
begin
  //  ::DataSpace/Storage/MSCompressed/Content
  ContentStream.Position := 0;
  //WriteLn('Compressed Data start''s at: ', AStream.Position,' Size is: ', ContentStream.Size);
  Result := AStream.CopyFrom(ContentStream, ContentStream.Size);

end;

end.

