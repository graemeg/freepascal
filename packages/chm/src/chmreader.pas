{ Copyright (C) <2005> <Andrew Haines> chmreader.pas

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
  See the file COPYING.modifiedLGPL, included in this distribution,
  for details about the copyright.
}
unit chmreader;

{$mode objfpc}{$H+}

//{$DEFINE CHM_DEBUG}
{ $DEFINE CHM_DEBUG_CHUNKS}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils, Contnrs, chmbase, paslzx, chmFIftiMain, chmsitemap;

type

  TLZXResetTableArr = array of QWord;

=======
=======
>>>>>>> origin/fixes_2_2
  Classes, SysUtils, chmbase, paslzx, chmFIftiMain;
=======
  Classes, SysUtils, chmbase, paslzx, chmFIftiMain, chmsitemap;
>>>>>>> origin/fixes_2.4
  
type

  TLZXResetTableArr = array of QWord;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  PContextItem = ^TContextItem;
  TContextItem = record
    Context: THelpContext;
    Url: String;
  end;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  TContextList = class(TList)
  public
    procedure AddContext(Context: THelpContext; Url: String);
    function GetURL(Context: THelpContext): String;
    procedure Clear; override;
  end;
  { TITSFReader }

  TFileEntryForEach = procedure(Name: String; Offset, UncompressedSize, Section: Integer) of object;

  TITSFReader = class(TObject)
  protected
    fStream: TStream;
    fFreeStreamOnDestroy: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    fITSFHeader: TITSFHeader;
=======
    fChmHeader: TITSFHeader;
>>>>>>> graemeg/fixes_2_2
=======
    fChmHeader: TITSFHeader;
>>>>>>> origin/fixes_2_2
    fHeaderSuffix: TITSFHeaderSuffix;
    fDirectoryHeader: TITSPHeader;
    fDirectoryHeaderPos: QWord;
    fDirectoryHeaderLength: QWord;
    fDirectoryEntriesStartPos: QWord;
<<<<<<< HEAD
<<<<<<< HEAD
    fCachedEntry: TPMGListChunkEntry; //contains the last entry found by ObjectExists
    fDirectoryEntriesCount: LongWord;
    procedure ReadHeader; virtual;
    procedure ReadHeaderEntries; virtual;
    function  GetChunkType(Stream: TMemoryStream; ChunkIndex: LongInt): TDirChunkType;
    procedure GetSections(out Sections: TStringList);
  private
=======
=======
>>>>>>> origin/fixes_2_2
    fDirectoryEntries: array of TPMGListChunkEntry;
    fCachedEntry: TPMGListChunkEntry; //contains the last entry found by ObjectExists
    fDirectoryEntriesCount: LongWord;
  private
    procedure ReadHeader;
    function  GetChunkType(Stream: TMemoryStream; ChunkIndex: LongInt): TPMGchunktype;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function  GetDirectoryChunk(Index: Integer; OutStream: TStream): Integer;
    function  ReadPMGLchunkEntryFromStream(Stream: TMemoryStream; var PMGLEntry: TPMGListChunkEntry): Boolean;
    function  ReadPMGIchunkEntryFromStream(Stream: TMemoryStream; var PMGIEntry: TPMGIIndexChunkEntry): Boolean;
    procedure LookupPMGLchunk(Stream: TMemoryStream; out PMGLChunk: TPMGListChunk);
    procedure LookupPMGIchunk(Stream: TMemoryStream; out PMGIChunk: TPMGIIndexChunk);

<<<<<<< HEAD
<<<<<<< HEAD

=======
    procedure GetSections(out Sections: TStringList);
>>>>>>> graemeg/fixes_2_2
=======
    procedure GetSections(out Sections: TStringList);
>>>>>>> origin/fixes_2_2
    function  GetBlockFromSection(SectionPrefix: String; StartPos: QWord; BlockLength: QWord): TMemoryStream;
    function  FindBlocksFromUnCompressedAddr(var ResetTableEntry: TPMGListChunkEntry;
       out CompressedSize: QWord; out UnCompressedSize: QWord; out LZXResetTable: TLZXResetTableArr): QWord;  // Returns the blocksize
  public
    constructor Create(AStream: TStream; FreeStreamOnDestroy: Boolean); virtual;
    destructor Destroy; override;
  public
    ChmLastError: LongInt;
    function IsValidFile: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure GetCompleteFileList(ForEach: TFileEntryForEach; AIncludeInternalFiles: Boolean = True); virtual;
    function ObjectExists(Name: String): QWord; virtual; // zero if no. otherwise it is the size of the object
                                                // NOTE directories will return zero size even if they exist
    function GetObject(Name: String): TMemoryStream; virtual; // YOU must Free the stream
    property CachedEntry: TPMGListChunkEntry read fCachedEntry;
  end;

=======
=======
>>>>>>> origin/fixes_2_2
    procedure GetCompleteFileList(ForEach: TFileEntryForEach);
    function ObjectExists(Name: String): QWord; // zero if no. otherwise it is the size of the object
                                                // NOTE directories will return zero size even if they exist
    function GetObject(Name: String): TMemoryStream; // YOU must Free the stream
    property CachedEntry: TPMGListChunkEntry read fCachedEntry;
  end;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { TChmReader }

  TChmReader = class(TITSFReader)
  protected
    fDefaultPage: String;
    fIndexFile: String;
    fTOCFile: String;
    fTitle: String;
    fPreferedFont: String;
    fContextList: TContextList;
    fTOPICSStream,
    fURLSTRStream,
    fURLTBLStream,
    fStringsStream: TMemoryStream;
    fLocaleID: DWord;
<<<<<<< HEAD
<<<<<<< HEAD
    fWindowsList : TObjectList;
    fDefaultWindow: String;
  private
    FSearchReader: TChmSearchReader;
  public
    procedure ReadCommonData;
    function  ReadStringsEntry(APosition: DWord): String;
    function  ReadStringsEntryFromStream ( strm:TStream ) : String;
    function  ReadURLSTR(APosition: DWord): String;
    function  CheckCommonStreams: Boolean;
    procedure ReadWindows(mem:TMemoryStream);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  public
>>>>>>> graemeg/cpstrnew
    constructor Create(AStream: TStream; FreeStreamOnDestroy: Boolean); override;
    destructor Destroy; override;
    function GetContextUrl(Context: THelpContext): String;
    function LookupTopicByID(ATopicID: Integer; out ATitle: String): String; // returns a url
    function GetTOCSitemap(ForceXML:boolean=false): TChmSiteMap;
    function GetIndexSitemap(ForceXML:boolean=false): TChmSiteMap;
=======
=======
>>>>>>> origin/fixes_2_2
  private
    FSearchReader: TChmSearchReader;
    procedure ReadCommonData;
    function  ReadStringsEntry(APosition: DWord): String;
    function  ReadURLSTR(APosition: DWord): String;
    function  CheckCommonStreams: Boolean;
  public
    constructor Create(AStream: TStream; FreeStreamOnDestroy: Boolean); override;
    destructor Destroy; override;
  public
    function GetContextUrl(Context: THelpContext): String;
    function LookupTopicByID(ATopicID: Integer; out ATitle: String): String; // returns a url
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    function GetTOCSitemap(ForceXML:boolean=false): TChmSiteMap;
    function GetIndexSitemap(ForceXML:boolean=false): TChmSiteMap;
>>>>>>> origin/fixes_2.4
    function HasContextList: Boolean;
    property DefaultPage: String read fDefaultPage;
    property IndexFile: String read fIndexFile;
    property TOCFile: String read fTOCFile;
    property Title: String read fTitle write fTitle;
    property PreferedFont: String read fPreferedFont;
    property LocaleID: dword read fLocaleID;
    property SearchReader: TChmSearchReader read FSearchReader write FSearchReader;
<<<<<<< HEAD
<<<<<<< HEAD
    property contextlist : tcontextlist read fcontextlist;
    property Windows : TObjectlist read fWindowsList;
    property DefaultWindow : string read fdefaultwindow;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  { TChmFileList }
  TChmFileList = class;
  TChmFileOpenEvent = procedure(ChmFileList: TChmFileList; Index: Integer) of object;
  TChmFileList = class(TStringList)
  protected
    fLastChm: TChmReader;
    fUnNotifiedFiles: TList;
    fOnOpenNewFile: TChmFileOpenEvent;
    procedure Delete(Index: Integer); override;
    function GetChm(AIndex: Integer): TChmReader;
    function GetFileName(AIndex: Integer): String;
    procedure OpenNewFile(AFileName: String);
    function CheckOpenFile(AFileName: String): Boolean;
    function MetaObjectExists(var Name: String): QWord;
    function MetaGetObject(Name: String): TMemoryStream;
    procedure SetOnOpenNewFile(AValue: TChmFileOpenEvent);
  public
    constructor Create(PrimaryFileName: String);
    destructor Destroy; override;
    function GetObject(Name: String): TMemoryStream;
    function IsAnOpenFile(AFileName: String): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
    function ObjectExists(Name: String; var fChm: TChmReader): QWord;
=======
    function ObjectExists(Name: String; var fChm: TChmReader = nil): QWord;
>>>>>>> graemeg/fixes_2_2
=======
    function ObjectExists(Name: String; var fChm: TChmReader = nil): QWord;
>>>>>>> origin/fixes_2_2
    //properties
    property Chm[Index: Integer]: TChmReader read GetChm;
    property FileName[Index: Integer]: String read GetFileName;
    property OnOpenNewFile: TChmFileOpenEvent read fOnOpenNewFile write SetOnOpenNewFile;
  end;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
//ErrorCodes
const
  ERR_NO_ERR = 0;
  ERR_STREAM_NOT_ASSIGNED = 1;
  ERR_NOT_SUPPORTED_VERSION = 2;
  ERR_NOT_VALID_FILE = 3;
  ERR_UNKNOWN_ERROR = 10;
<<<<<<< HEAD
<<<<<<< HEAD

  function ChmErrorToStr(Error: Integer): String;

implementation
uses ChmTypes;
=======
=======
>>>>>>> origin/fixes_2_2
  
  function ChmErrorToStr(Error: Integer): String;

implementation
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
uses ChmTypes;
>>>>>>> origin/fixes_2.4

function ChmErrorToStr(Error: Integer): String;
begin
  Result := '';
  case Error of
    ERR_STREAM_NOT_ASSIGNED    : Result := 'ERR_STREAM_NOT_ASSIGNED';
    ERR_NOT_SUPPORTED_VERSION  : Result := 'ERR_NOT_SUPPORTED_VERSION';
    ERR_NOT_VALID_FILE         : Result := 'ERR_NOT_VALID_FILE';
    ERR_UNKNOWN_ERROR          : Result := 'ERR_UNKNOWN_ERROR';
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function ChunkType(Stream: TMemoryStream): TDirChunkType;
=======
function ChunkType(Stream: TMemoryStream): TPMGchunktype;
>>>>>>> graemeg/fixes_2_2
=======
function ChunkType(Stream: TMemoryStream): TPMGchunktype;
>>>>>>> origin/fixes_2_2
var
  ChunkID: array[0..3] of char;
begin
  Result := ctUnknown;
  if Stream.Size< 4 then exit;
  Move(Stream.Memory^, ChunkId[0], 4);
  if ChunkID = 'PMGL' then Result := ctPMGL
<<<<<<< HEAD
<<<<<<< HEAD
  else if ChunkID = 'PMGI' then Result := ctPMGI
  else if ChunkID = 'AOLL' then Result := ctAOLL
  else if ChunkID = 'AOLI' then Result := ctAOLI;
=======
  else if ChunkID = 'PMGI' then Result := ctPMGI;
>>>>>>> graemeg/fixes_2_2
=======
  else if ChunkID = 'PMGI' then Result := ctPMGI;
>>>>>>> origin/fixes_2_2
end;

{ TITSFReader }

procedure TITSFReader.ReadHeader;
<<<<<<< HEAD
<<<<<<< HEAD
begin
  fStream.Read(fITSFHeader,SizeOf(fITSFHeader));

  // Fix endian issues
  {$IFDEF ENDIAN_BIG}
  fITSFHeader.Version := LEtoN(fITSFHeader.Version);
  fITSFHeader.HeaderLength := LEtoN(fITSFHeader.HeaderLength);
  //Unknown_1
  fITSFHeader.TimeStamp := BEtoN(fITSFHeader.TimeStamp);//bigendian
  fITSFHeader.LanguageID := LEtoN(fITSFHeader.LanguageID);
  {$ENDIF}

  if fITSFHeader.Version < 4 then
   fStream.Seek(SizeOf(TGuid)*2, soCurrent);

  if not IsValidFile then Exit;

  ReadHeaderEntries;
end;

procedure TITSFReader.ReadHeaderEntries;
var
fHeaderEntries: array [0..1] of TITSFHeaderEntry;
begin
  // Copy EntryData into memory
  fStream.Read(fHeaderEntries[0], SizeOf(fHeaderEntries));

  if fITSFHeader.Version = 3 then
    fStream.Read(fHeaderSuffix.Offset, SizeOf(QWord));
  fHeaderSuffix.Offset := LEtoN(fHeaderSuffix.Offset);
  // otherwise this is set in fill directory entries

=======
=======
>>>>>>> origin/fixes_2_2
var
fHeaderEntries: array [0..1] of TITSFHeaderEntry;
begin
  fStream.Position := 0;
  fStream.Read(fChmHeader,SizeOf(fChmHeader));

  // Fix endian issues
  {$IFDEF ENDIAN_BIG}
  fChmHeader.Version := LEtoN(fChmHeader.Version);
  fChmHeader.HeaderLength := LEtoN(fChmHeader.HeaderLength);
  //Unknown_1
  fChmHeader.TimeStamp := BEtoN(fChmHeader.TimeStamp);//bigendian
  fChmHeader.LanguageID := LEtoN(fChmHeader.LanguageID);
  //Guid1
  //Guid2
  {$ENDIF}
  
  if not IsValidFile then Exit;
  
  // Copy EntryData into memory
  fStream.Read(fHeaderEntries[0], SizeOf(fHeaderEntries));

  if fChmHeader.Version > 2 then
    fStream.Read(fHeaderSuffix.Offset, SizeOf(QWord));
  fHeaderSuffix.Offset := LEtoN(fHeaderSuffix.Offset);
  // otherwise this is set in fill directory entries
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  fStream.Position := LEtoN(fHeaderEntries[1].PosFromZero);
  fDirectoryHeaderPos := LEtoN(fHeaderEntries[1].PosFromZero);
  fStream.Read(fDirectoryHeader, SizeOf(fDirectoryHeader));
  {$IFDEF ENDIAN_BIG}
  with fDirectoryHeader do begin
    Version := LEtoN(Version);
    DirHeaderLength := LEtoN(DirHeaderLength);
    //Unknown1
    ChunkSize := LEtoN(ChunkSize);
    Density := LEtoN(Density);
    IndexTreeDepth := LEtoN(IndexTreeDepth);
    IndexOfRootChunk := LEtoN(IndexOfRootChunk);
    FirstPMGLChunkIndex := LEtoN(FirstPMGLChunkIndex);
    LastPMGLChunkIndex := LEtoN(LastPMGLChunkIndex);
    //Unknown2
    DirectoryChunkCount := LEtoN(DirectoryChunkCount);
    LanguageID := LEtoN(LanguageID);
    //GUID: TGuid;
    LengthAgain := LEtoN(LengthAgain);
  end;
  {$ENDIF}
  {$IFDEF CHM_DEBUG}
  WriteLn('PMGI depth = ', fDirectoryHeader.IndexTreeDepth);
  WriteLn('PMGI Root =  ', fDirectoryHeader.IndexOfRootChunk);
<<<<<<< HEAD
<<<<<<< HEAD
  Writeln('DirCount  =  ', fDirectoryHeader.DirectoryChunkCount);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$ENDIF}
  fDirectoryEntriesStartPos := fStream.Position;
  fDirectoryHeaderLength := LEtoN(fHeaderEntries[1].Length);
end;

procedure TChmReader.ReadCommonData;
   // A little helper proc to make reading a null terminated string easier
<<<<<<< HEAD
<<<<<<< HEAD
   function ReadString(const Stream: TStream; StartPos: DWord; FixURL: Boolean): String;
=======
   function ReadString(const Stream: TStream): String;
>>>>>>> graemeg/fixes_2_2
=======
   function ReadString(const Stream: TStream): String;
>>>>>>> origin/fixes_2_2
   var
     buf: array[0..49] of char;
   begin
     Result := '';
<<<<<<< HEAD
<<<<<<< HEAD
     Stream.Position := StartPos;
     repeat
       Stream.Read(buf, 50);
       Result := Result + buf;
<<<<<<< HEAD
     until IndexByte(buf, 50, 0) <> -1;
=======
     until Pos(#0, buf) > -1;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     if FixURL then
       Result := StringReplace(Result, '\', '/', [rfReplaceAll]);
=======
     repeat
       Stream.Read(buf, 50);
       Result := Result + buf;
     until Pos(#0, buf) > -1;
>>>>>>> graemeg/fixes_2_2
=======
     repeat
       Stream.Read(buf, 50);
       Result := Result + buf;
     until Pos(#0, buf) > -1;
>>>>>>> origin/fixes_2_2
   end;
   procedure ReadFromSystem;
   var
     //Version: DWord;
     EntryType: Word;
     EntryLength: Word;
     Data: array[0..511] of char;
     fSystem: TMemoryStream;
     Tmp: String;
   begin
     fSystem := TMemoryStream(GetObject('/#SYSTEM'));
     if fSystem = nil then begin
       exit;
     end;
     fSystem.Position := 0;
     if fSystem.Size < SizeOf(DWord) then begin
       fSystem.Free;
       Exit;
     end;
     {Version := }LEtoN(fSystem.ReadDWord);
     while fSystem.Position < fSystem.Size do begin
       EntryType := LEtoN(fSystem.ReadWord);
       EntryLength := LEtoN(fSystem.ReadWord);
       case EntryType of
         0: // Table of contents
         begin
           if EntryLength > 511 then EntryLength := 511;
           fSystem.Read(Data[0], EntryLength);
           Data[EntryLength] := #0;
           fTOCFile := '/'+Data;
         end;
         1: // Index File
         begin
           if EntryLength > 511 then EntryLength := 511;
           fSystem.Read(Data[0], EntryLength);
           Data[EntryLength] := #0;
           fIndexFile := '/'+Data;
         end;
         2: // DefaultPage
         begin
           if EntryLength > 511 then EntryLength := 511;
           fSystem.Read(Data[0], EntryLength);
           Data[EntryLength] := #0;
           fDefaultPage := '/'+Data;
         end;
         3: // Title of chm
         begin
           if EntryLength > 511 then EntryLength := 511;
           fSystem.Read(Data[0], EntryLength);
           Data[EntryLength] := #0;
           fTitle := Data;
         end;
         4: // Locale ID
         begin
           fLocaleID := LEtoN(fSystem.ReadDWord);
           fSystem.Position := (fSystem.Position + EntryLength) - SizeOf(DWord);
         end;
         6: // chm file name. use this to get the index and toc name
         begin
           if EntryLength > 511 then EntryLength := 511;
           fSystem.Read(Data[0], EntryLength);
           Data[EntryLength] := #0;
           if (fIndexFile = '') then begin
             Tmp := '/'+Data+'.hhk';
             if (ObjectExists(Tmp) > 0) then begin
               fIndexFile := Tmp;
             end
           end;
           if (fTOCFile = '') then begin
             Tmp := '/'+Data+'.hhc';
             if (ObjectExists(Tmp) > 0) then begin
               fTOCFile := Tmp;
             end;
           end;
         end;
         16: // Prefered font
         begin
           if EntryLength > 511 then EntryLength := 511;
           fSystem.Read(Data[0], EntryLength);
           Data[EntryLength] := #0;
           fPreferedFont := Data;
         end;
       else
         // Skip entries we are not interested in
         fSystem.Position := fSystem.Position + EntryLength;
       end;
     end;
     fSystem.Free;
   end;
   procedure ReadFromWindows;
   var
     fWindows,
     fStrings: TMemoryStream;
     EntryCount,
     EntrySize: DWord;
     EntryStart: QWord;
<<<<<<< HEAD
<<<<<<< HEAD
=======
     StrPosition: DWord;
>>>>>>> graemeg/fixes_2_2
=======
     StrPosition: DWord;
>>>>>>> origin/fixes_2_2
     X: Integer;
     OffSet: QWord;
   begin
     fWindows := TMemoryStream(GetObject('/#WINDOWS'));
     if fWindows = nil then begin
       exit;
     end;
     fStrings := TMemoryStream(GetObject('/#STRINGS'));
     if fStrings = nil then begin
       if fWindows <> nil then fWindows.Free;
       Exit;
     end;
     fWindows.Position := 0;
     if (fWindows.Size = 0) or (fStrings.Size = 0) then begin
       fWindows.Free;
       fStrings.Free;
       Exit;
     end;
     EntryCount := LEtoN(fWindows.ReadDWord);
     EntrySize := LEtoN(fWindows.ReadDWord);
     OffSet := fWindows.Position;
     for X := 0 to EntryCount -1 do begin
       EntryStart := OffSet + (X*EntrySize);
       if fTitle = '' then begin
         fWindows.Position := EntryStart + $14;
<<<<<<< HEAD
<<<<<<< HEAD
         fTitle := '/'+ReadString(fStrings, LEtoN(fWindows.ReadDWord), False);
       end;
       if fTOCFile = '' then begin
         fWindows.Position := EntryStart + $60;
         fTOCFile := '/'+ReadString(fStrings, LEtoN(fWindows.ReadDWord), True);
       end;
       if fIndexFile = '' then begin
         fWindows.Position := EntryStart + $64;
         fIndexFile := '/'+ReadString(fStrings, LEtoN(fWindows.ReadDWord), True);
       end;
       if fDefaultPage = '' then begin
         fWindows.Position := EntryStart + $68;
         fDefaultPage := '/'+ReadString(fStrings, LEtoN(fWindows.ReadDWord), True);
       end;
     end;
     ReadWindows(FWindows);
=======
=======
>>>>>>> origin/fixes_2_2
         StrPosition := LEtoN(fWindows.ReadDWord);
         fStrings.Position := StrPosition;
         fTitle := '/'+ReadString(fStrings);
       end;
       if fTOCFile = '' then begin
         fWindows.Position := EntryStart + $60;
         StrPosition := LEtoN(fWindows.ReadDWord);
         fStrings.Position := StrPosition;
         fTOCFile := '/'+ReadString(fStrings);
       end;
       if fIndexFile = '' then begin
         fWindows.Position := EntryStart + $64;
         StrPosition := LEtoN(fWindows.ReadDWord);
         fStrings.Position := StrPosition;
         fIndexFile := '/'+ReadString(fStrings);
       end;
       if fDefaultPage = '' then begin
         fWindows.Position := EntryStart + $68;
         StrPosition := LEtoN(fWindows.ReadDWord);
         fStrings.Position := StrPosition;
         fDefaultPage := '/'+ReadString(fStrings);
       end;
     end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   end;
   procedure ReadContextIds;
   var
     fIVB,
     fStrings: TStream;
     Str: String;
     Value: DWord;
     OffSet: DWord;
     //TotalSize: DWord;
   begin
<<<<<<< HEAD
<<<<<<< HEAD
     fIVB := GetObject('/#IVB');
=======
     fIVB := GetObject('/#IBV');
>>>>>>> graemeg/fixes_2_2
=======
     fIVB := GetObject('/#IBV');
>>>>>>> origin/fixes_2_2
     if fIVB = nil then Exit;
     fStrings := GetObject('/#STRINGS');
     if fStrings = nil then begin
       fIVB.Free;
       Exit;
     end;
     fIVB.Position := 0;
     {TotalSize := }LEtoN(fIVB.ReadDWord);
     while fIVB.Position < fIVB.Size do begin
       Value := LEtoN(fIVB.ReadDWord);
       OffSet := LEtoN(fIVB.ReadDWord);
<<<<<<< HEAD
<<<<<<< HEAD
       Str := '/'+ ReadString(fStrings, Offset, True);
=======
       fStrings.Position := Offset;
       Str := '/'+ReadString(fStrings);
>>>>>>> graemeg/fixes_2_2
=======
       fStrings.Position := Offset;
       Str := '/'+ReadString(fStrings);
>>>>>>> origin/fixes_2_2
       fContextList.AddContext(Value, Str);
     end;
   end;
begin
   ReadFromSystem;
   ReadFromWindows;
   ReadContextIds;
<<<<<<< HEAD
<<<<<<< HEAD
   {$IFDEF CHM_DEBUG}
=======
   {$IFDEF CHM_DEBUG}   
>>>>>>> graemeg/fixes_2_2
=======
   {$IFDEF CHM_DEBUG}   
>>>>>>> origin/fixes_2_2
   WriteLn('TOC=',fTocfile);
   WriteLn('DefaultPage=',fDefaultPage);
   {$ENDIF}
end;

function TChmReader.ReadStringsEntry ( APosition: DWord ) : String;
begin
  Result := '';
  if fStringsStream = nil then
    fStringsStream := GetObject('/#STRINGS');
  if fStringsStream = nil then
    Exit;
  if APosition < fStringsStream.Size-1 then
  begin
    Result := PChar(fStringsStream.Memory+APosition);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TChmReader.ReadStringsEntryFromStream ( strm:TStream ) : String;
var APosition : DWord;
begin
  APosition:=LEtoN(strm.ReadDWord);
  result:=ReadStringsEntry(APosition);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function TChmReader.ReadURLSTR ( APosition: DWord ) : String;
var
  URLStrURLOffset: DWord;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  if not CheckCommonStreams then
    Exit;

  fURLTBLStream.Position := APosition;
  fURLTBLStream.ReadDWord; // unknown
  fURLTBLStream.ReadDWord; // TOPIC index #
  fURLSTRStream.Position := LEtoN(fURLTBLStream.ReadDWord);
  fURLSTRStream.ReadDWord;
  fURLSTRStream.ReadDWord;
  if fURLSTRStream.Position < fURLSTRStream.Size-1 then
<<<<<<< HEAD
    Result := PChar(fURLSTRStream.Memory+fURLSTRStream.Position);
=======
    Result := '/'+PChar(fURLSTRStream.Memory+fURLSTRStream.Position);
>>>>>>> origin/fixes_2.4
end;

function TChmReader.CheckCommonStreams: Boolean;
begin
  if fTOPICSStream = nil then
    fTOPICSStream := GetObject('/#TOPICS');
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
  if fURLSTRStream = nil then
    fURLSTRStream := GetObject('/#URLSTR');
  if fURLTBLStream = nil then
    fURLTBLStream := GetObject('/#URLTBL');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  Result :=     (fTOPICSStream <> nil)
            and (fURLSTRStream <> nil)
            and (fURLTBLStream <> nil);
end;

procedure TChmReader.ReadWindows(mem:TMemoryStream);

var
  i,cnt,
  version   : integer;
  x         : TChmWindow;
begin
 if not assigned(fwindowslist) then
 fWindowsList.Clear;
 mem.Position:=0;
 cnt  := LEtoN(mem.ReadDWord);
 version  := LEtoN(mem.ReadDWord);
 while (cnt>0) do
   begin
     x:=TChmWindow.Create;
     version            := LEtoN(mem.ReadDWord);                        //  0 size of entry.
     mem.readDWord;                                                     //  4 unknown (bool Unicodestrings?)
     x.window_type      :=ReadStringsEntryFromStream(mem);              //  8 Arg 0, name of window
     x.flags            := TValidWindowFields(LEtoN(mem.ReadDWord));    //  C valid fields
     x.nav_style        := LEtoN(mem.ReadDWord);                        // 10 arg 10 navigation pane style
     x.title_bar_text   :=ReadStringsEntryFromStream(mem);              // 14 Arg 1,  title bar text
     x.styleflags       := LEtoN(mem.ReadDWord);                        // 18 Arg 14, style flags
     x.xtdstyleflags    := LEtoN(mem.ReadDWord);                        // 1C Arg 15, xtd style flags
     x.left             := LEtoN(mem.ReadDWord);                        // 20 Arg 13, rect.left
     x.right            := LEtoN(mem.ReadDWord);                        // 24 Arg 13, rect.top
     x.top              := LEtoN(mem.ReadDWord);                        // 28 Arg 13, rect.right
     x.bottom           := LEtoN(mem.ReadDWord);                        // 2C Arg 13, rect.bottom
     x.window_show_state:= LEtoN(mem.ReadDWord);                        // 30 Arg 16, window show state
     mem.readdword;                                                     // 34  -    , HWND hwndhelp                OUT: window handle"
     mem.readdword;                                                     // 38  -    , HWND hwndcaller              OUT: who called this window"
     mem.readdword;                                                     // 3C  -    , HH_INFO_TYPE paINFO_TYPES    IN: Pointer to an array of Information Types"
     mem.readdword;                                                     // 40  -    , HWND hwndtoolbar             OUT: toolbar window in tri-pane window"
     mem.readdword;                                                     // 44  -    , HWND hwndnavigation          OUT: navigation window in tri-pane window"
     mem.readdword;                                                     // 48  -    , HWND hwndhtml                OUT: window displaying HTML in tri-pane window"
     x.navpanewidth     := LEtoN(mem.ReadDWord);                        // 4C Arg 11, width of nav pane
     mem.readdword;                                                     // 50  -    , rect.left,   OUT:Specifies the coordinates of the Topic pane
     mem.readdword;                                                     // 54  -    , rect.top ,   OUT:Specifies the coordinates of the Topic pane
     mem.readdword;                                                     // 58  -    , rect.right,  OUT:Specifies the coordinates of the Topic pane
     mem.readdword;                                                     // 5C  -    , rect.bottom, OUT:Specifies the coordinates of the Topic pane
     x.toc_file         :=ReadStringsEntryFromStream(mem);              // 60 Arg 2,  toc file
     x.index_file       :=ReadStringsEntryFromStream(mem);              // 64 Arg 3,  index file
     x.default_file     :=ReadStringsEntryFromStream(mem);              // 68 Arg 4,  default file
     x.home_button_file :=ReadStringsEntryFromStream(mem);              // 6c Arg 5,  home button file.
     x.buttons          := LEtoN(mem.ReadDWord);                        // 70 arg 12,
     x.navpane_initially_closed    := LEtoN(mem.ReadDWord);             // 74 arg 17
     x.navpane_default  := LEtoN(mem.ReadDWord);                        // 78 arg 18,
     x.navpane_location := LEtoN(mem.ReadDWord);                        // 7C arg 19,
     x.wm_notify_id     := LEtoN(mem.ReadDWord);                        // 80 arg 20,
     for i:=0 to 4 do
       mem.ReadDWord;                                                   // 84  -      byte[20] unknown -  "BYTE tabOrder[HH_MAX_TABS + 1]; // IN/OUT: tab order: Contents, Index, Search, History, Favorites, Reserved 1-5, Custom tabs"
     mem.ReadDWord;                                                     // 94  -      int cHistory; // IN/OUT: number of history items to keep (default is 30)
     x.jumpbutton_1_text:=ReadStringsEntryFromStream(mem);              // 9C Arg 7,  The text of the Jump 1 button.
     x.jumpbutton_2_text:=ReadStringsEntryFromStream(mem);              // A0 Arg 9,  The text of the Jump 2 button.
     x.jumpbutton_1_file:=ReadStringsEntryFromStream(mem);              // A4 Arg 6,  The file shown for Jump 1 button.
     x.jumpbutton_2_file:=ReadStringsEntryFromStream(mem);              // A8 Arg 8,  The file shown for Jump 1 button.
     for i:=0 to 3 do
       mem.ReadDWord;
     dec(version,188);                                              // 1.1 specific onesf
     while (version>=4) do
       begin
         mem.readdword;
         dec(version,4);
       end;

     fWindowslist.Add(x);
     dec(cnt);
   end;
=======
=======
>>>>>>> origin/fixes_2_2
  if (fURLTBLStream <> nil) and (fURLSTRStream <> nil) then
  begin

    fURLTBLStream.Position := APosition;
    fURLTBLStream.ReadDWord; // unknown
    fURLTBLStream.ReadDWord; // TOPIC index #
    fURLSTRStream.Position := LEtoN(fURLTBLStream.ReadDWord);
    fURLSTRStream.ReadDWord;
    fURLSTRStream.ReadDWord;
    if fURLSTRStream.Position < fURLSTRStream.Size-1 then
      Result := '/'+PChar(fURLSTRStream.Memory+fURLSTRStream.Position);
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

  Result :=     (fTOPICSStream <> nil)
            and (fURLSTRStream <> nil)
            and (fURLTBLStream <> nil);
>>>>>>> origin/fixes_2.4
end;

constructor TChmReader.Create(AStream: TStream; FreeStreamOnDestroy: Boolean);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  fContextList := TContextList.Create;
  fWindowslist      := TObjectlist.Create(True);
  fDefaultWindow:='';

  inherited Create(AStream, FreeStreamOnDestroy);
  if not IsValidFile then exit;

=======
=======
>>>>>>> origin/fixes_2_2
  inherited Create(AStream, FreeStreamOnDestroy);
  if not IsValidFile then exit;

  fContextList := TContextList.Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  ReadCommonData;
end;

destructor TChmReader.Destroy;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FreeAndNil(fContextList);
  FreeAndNil(FWindowslist);
=======
  fContextList.Free;
>>>>>>> graemeg/fixes_2_2
=======
  fContextList.Free;
>>>>>>> origin/fixes_2_2
  FreeAndNil(FSearchReader);
  FreeAndNil(fTOPICSStream);
  FreeAndNil(fURLSTRStream);
  FreeAndNil(fURLTBLStream);
  FreeAndNil(fStringsStream);
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TITSFReader.GetChunkType(Stream: TMemoryStream; ChunkIndex: LongInt): TDirChunkType;
=======
function TITSFReader.GetChunkType(Stream: TMemoryStream; ChunkIndex: LongInt): TPMGchunktype;
>>>>>>> graemeg/fixes_2_2
=======
function TITSFReader.GetChunkType(Stream: TMemoryStream; ChunkIndex: LongInt): TPMGchunktype;
>>>>>>> origin/fixes_2_2
var
  Sig: array[0..3] of char;
begin
  Result := ctUnknown;
  Stream.Position := fDirectoryEntriesStartPos + (fDirectoryHeader.ChunkSize * ChunkIndex);

  Stream.Read(Sig, 4);
  if Sig = 'PMGL' then Result := ctPMGL
<<<<<<< HEAD
<<<<<<< HEAD
  else if Sig = 'PMGI' then Result := ctPMGI
  else if Sig = 'AOLL' then Result := ctAOLL
  else if Sig = 'AOLI' then Result := ctAOLI;
=======
  else if Sig = 'PMGI' then Result := ctPMGI;
>>>>>>> graemeg/fixes_2_2
=======
  else if Sig = 'PMGI' then Result := ctPMGI;
>>>>>>> origin/fixes_2_2
end;

function TITSFReader.GetDirectoryChunk(Index: Integer; OutStream: TStream): Integer;
begin
  Result := Index;
  fStream.Position := fDirectoryEntriesStartPos + (fDirectoryHeader.ChunkSize * Index);
  OutStream.Position := 0;
  OutStream.Size := fDirectoryHeader.ChunkSize;
  OutStream.CopyFrom(fStream, fDirectoryHeader.ChunkSize);
  OutStream.Position := 0;
end;

procedure TITSFReader.LookupPMGLchunk(Stream: TMemoryStream; out PMGLChunk: TPMGListChunk);
begin
  //Stream.Position := fDirectoryEntriesStartPos + (fDirectoryHeader.ChunkSize * ChunkIndex);
  Stream.Read(PMGLChunk, SizeOf(PMGLChunk));
  {$IFDEF ENDIAN_BIG}
  with PMGLChunk do begin
    UnusedSpace := LEtoN(UnusedSpace);
    //Unknown1
    PreviousChunkIndex := LEtoN(PreviousChunkIndex);
    NextChunkIndex := LEtoN(NextChunkIndex);
  end;
  {$ENDIF}
end;

function TITSFReader.ReadPMGLchunkEntryFromStream(Stream: TMemoryStream; var PMGLEntry: TPMGListChunkEntry): Boolean;
var
Buf: array [0..1023] of char;
NameLength: LongInt;
begin
  Result := False;
  //Stream.Position := fDirectoryEntriesStartPos + (fDirectoryHeader.ChunkSize * ChunkIndex);
  NameLength := LongInt(GetCompressedInteger(Stream));

  if NameLength > 1022 then NameLength := 1022;
  Stream.Read(buf[0], NameLength);
  buf[NameLength] := #0;
  PMGLEntry.Name := buf;
  PMGLEntry.ContentSection := LongWord(GetCompressedInteger(Stream));
  PMGLEntry.ContentOffset := GetCompressedInteger(Stream);
  PMGLEntry.DecompressedLength := GetCompressedInteger(Stream);
  if NameLength = 0 then Exit; // failed GetCompressedInteger sanity check
  Result := True;
end;

procedure TITSFReader.LookupPMGIchunk(Stream: TMemoryStream; out PMGIChunk: TPMGIIndexChunk);
begin
  //Stream.Position := fDirectoryEntriesStartPos + (fDirectoryHeader.ChunkSize * ChunkIndex);
  Stream.Read(PMGIChunk, SizeOf(PMGIChunk));
  {$IFDEF ENDIAN_BIG}
  with PMGIChunk do begin
    UnusedSpace := LEtoN(UnusedSpace);
  end;
  {$ENDIF}
end;

function TITSFReader.ReadPMGIchunkEntryFromStream(Stream: TMemoryStream;
  var PMGIEntry: TPMGIIndexChunkEntry): Boolean;
var
Buf: array [0..1023] of char;
NameLength: LongInt;
begin
  Result := False;
  //Stream.Position := fDirectoryEntriesStartPos + (fDirectoryHeader.ChunkSize * ChunkIndex);
  NameLength := LongInt(GetCompressedInteger(Stream));
  if NameLength > 1023 then NameLength := 1023;
  Stream.Read(buf, NameLength);

  buf[NameLength] := #0;
  PMGIEntry.Name := buf;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  PMGIEntry.ListingChunk := GetCompressedInteger(Stream);
  if NameLength = 0 then Exit; // failed GetCompressedInteger sanity check
  Result := True;
end;

constructor TITSFReader.Create(AStream: TStream; FreeStreamOnDestroy: Boolean);
begin
  fStream := AStream;
<<<<<<< HEAD
<<<<<<< HEAD
  fStream.Position := 0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  fFreeStreamOnDestroy := FreeStreamOnDestroy;
  ReadHeader;
  if not IsValidFile then Exit;
end;

destructor TITSFReader.Destroy;
begin
<<<<<<< HEAD
<<<<<<< HEAD
=======
  SetLength(fDirectoryEntries, 0);
>>>>>>> graemeg/fixes_2_2
=======
  SetLength(fDirectoryEntries, 0);
>>>>>>> origin/fixes_2_2
  if fFreeStreamOnDestroy then FreeAndNil(fStream);

  inherited Destroy;
end;

function TITSFReader.IsValidFile: Boolean;
begin
  if (fStream = nil) then ChmLastError := ERR_STREAM_NOT_ASSIGNED
<<<<<<< HEAD
<<<<<<< HEAD
  else if (fITSFHeader.ITSFsig <> 'ITSF') then ChmLastError := ERR_NOT_VALID_FILE
  //else if (fITSFHeader.Version <> 2) and (fITSFHeader.Version <> 3)
  else if not (fITSFHeader.Version in [2..4])
  then
=======
  else if (fChmHeader.ITSFsig <> 'ITSF') then ChmLastError := ERR_NOT_VALID_FILE
  else if (fChmHeader.Version <> 2) and (fChmHeader.Version <> 3) then
>>>>>>> graemeg/fixes_2_2
=======
  else if (fChmHeader.ITSFsig <> 'ITSF') then ChmLastError := ERR_NOT_VALID_FILE
  else if (fChmHeader.Version <> 2) and (fChmHeader.Version <> 3) then
>>>>>>> origin/fixes_2_2
    ChmLastError := ERR_NOT_SUPPORTED_VERSION;
  Result := ChmLastError = ERR_NO_ERR;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TITSFReader.GetCompleteFileList(ForEach: TFileEntryForEach; AIncludeInternalFiles: Boolean = True);
=======
procedure TITSFReader.GetCompleteFileList(ForEach: TFileEntryForEach);
>>>>>>> graemeg/fixes_2_2
=======
procedure TITSFReader.GetCompleteFileList(ForEach: TFileEntryForEach);
>>>>>>> origin/fixes_2_2
var
  ChunkStream: TMemoryStream;
  I : Integer;
  Entry: TPMGListChunkEntry;
  PMGLChunk: TPMGListChunk;
  CutOffPoint: Integer;
  NameLength: Integer;
  {$IFDEF CHM_DEBUG_CHUNKS}
  PMGIChunk: TPMGIIndexChunk;
  PMGIndex: Integer;
  {$ENDIF}
begin
  if ForEach = nil then Exit;
  ChunkStream := TMemoryStream.Create;
  {$IFDEF CHM_DEBUG_CHUNKS}
  WriteLn('ChunkCount = ',fDirectoryHeader.DirectoryChunkCount);
  {$ENDIF}
  for I := 0 to fDirectoryHeader.DirectoryChunkCount-1 do begin
    GetDirectoryChunk(I, ChunkStream);
    case ChunkType(ChunkStream) of
    ctPMGL:
     begin
       LookupPMGLchunk(ChunkStream, PMGLChunk);
       {$IFDEF CHM_DEBUG_CHUNKS}
        WriteLn('PMGL: ', I, ' Prev PMGL: ', PMGLChunk.PreviousChunkIndex, ' Next PMGL: ', PMGLChunk.NextChunkIndex);
       {$ENDIF}
       CutOffPoint := ChunkStream.Size - PMGLChunk.UnusedSpace;
       while ChunkStream.Position <  CutOffPoint do begin
         NameLength := GetCompressedInteger(ChunkStream);
         if (ChunkStream.Position > CutOffPoint) then Continue; // we have entered the quickref section
         SetLength(Entry.Name, NameLength);
         ChunkStream.ReadBuffer(Entry.Name[1], NameLength);
         if (Entry.Name = '') or (ChunkStream.Position > CutOffPoint) then Break; // we have entered the quickref section
         Entry.ContentSection := GetCompressedInteger(ChunkStream);
         if ChunkStream.Position > CutOffPoint then Break; // we have entered the quickref section
         Entry.ContentOffset := GetCompressedInteger(ChunkStream);
         if ChunkStream.Position > CutOffPoint then Break; // we have entered the quickref section
         Entry.DecompressedLength := GetCompressedInteger(ChunkStream);
         if ChunkStream.Position > CutOffPoint then Break; // we have entered the quickref section
         fCachedEntry := Entry; // if the caller trys to get this data we already know where it is :)
<<<<<<< HEAD
<<<<<<< HEAD
         if  (Length(Entry.Name) = 1)
         or (AIncludeInternalFiles
              or
             ((Length(Entry.Name) > 1) and (not(Entry.Name[2] in ['#','$',':']))))
         then
          ForEach(Entry.Name, Entry.ContentOffset, Entry.DecompressedLength, Entry.ContentSection);
=======
         ForEach(Entry.Name, Entry.ContentOffset, Entry.DecompressedLength, Entry.ContentSection);
>>>>>>> graemeg/fixes_2_2
=======
         ForEach(Entry.Name, Entry.ContentOffset, Entry.DecompressedLength, Entry.ContentSection);
>>>>>>> origin/fixes_2_2
       end;
     end;
    {$IFDEF CHM_DEBUG_CHUNKS}
    ctPMGI:
     begin
       WriteLn('PMGI: ', I);
       LookupPMGIchunk(ChunkStream, PMGIChunk);
       CutOffPoint := ChunkStream.Size - PMGIChunk.UnusedSpace - 10;
       while ChunkStream.Position <  CutOffPoint do begin
         NameLength := GetCompressedInteger(ChunkStream);
         SetLength(Entry.Name, NameLength);
         ChunkStream.ReadBuffer(Entry.Name[1], NameLength);
         PMGIndex := GetCompressedInteger(ChunkStream);
         WriteLn(Entry.Name, '  ', PMGIndex);
       end;
     end;
    ctUnknown: WriteLn('UNKNOWN CHUNKTYPE!' , I);
    {$ENDIF}
    end;
  end;
end;

function TITSFReader.ObjectExists(Name: String): QWord;
var
  ChunkStream: TMemoryStream;
  QuickRefCount: Word;
  QuickRefIndex: array of Word;
  ItemCount: Integer;
  procedure ReadQuickRefSection;
  var
    OldPosn: QWord;
    Posn: Integer;
    I: Integer;
  begin
    OldPosn := ChunkStream.Position;
<<<<<<< HEAD
<<<<<<< HEAD
    Posn := ChunkStream.Size-SizeOf(Word);
    ChunkStream.Position := Posn;

=======
    Posn := ChunkStream.Size-1-SizeOf(Word);
    ChunkStream.Position := Posn;
    
>>>>>>> graemeg/fixes_2_2
=======
    Posn := ChunkStream.Size-1-SizeOf(Word);
    ChunkStream.Position := Posn;
    
>>>>>>> origin/fixes_2_2
    ItemCount := LEToN(ChunkStream.ReadWord);
    //WriteLn('Max ITems for next block = ', ItemCount-1);
    QuickRefCount := ItemCount  div (1 + (1 shl fDirectoryHeader.Density));
    //WriteLn('QuickRefCount = ' , QuickRefCount);
    SetLength(QuickRefIndex, QuickRefCount+1);
    for I := 1 to QuickRefCount do begin
      Dec(Posn, SizeOf(Word));
      ChunkStream.Position := Posn;
      QuickRefIndex[I] := LEToN(ChunkStream.ReadWord);
    end;
    Inc(QuickRefCount);
    ChunkStream.Position := OldPosn;
  end;
  function ReadString(StreamPosition: Integer = -1): String;
  var
    NameLength: Integer;
  begin
    if StreamPosition > -1 then ChunkStream.Position := StreamPosition;

    NameLength := GetCompressedInteger(ChunkStream);
    SetLength(Result, NameLength);
<<<<<<< HEAD
<<<<<<< HEAD
    if NameLength>0 then
      ChunkStream.Read(Result[1], NameLength);
=======
    ChunkStream.Read(Result[1], NameLength);
>>>>>>> graemeg/fixes_2_2
=======
    ChunkStream.Read(Result[1], NameLength);
>>>>>>> origin/fixes_2_2
  end;
var
  PMGLChunk: TPMGListChunk;
  PMGIChunk: TPMGIIndexChunk;
<<<<<<< HEAD
<<<<<<< HEAD
  //ChunkStream: TMemoryStream; declared above
  Entry: TPMGListChunkEntry;
  NextIndex: Integer;
  EntryName: String;
=======
=======
>>>>>>> origin/fixes_2_2
  //ChunkStream: TMemoryStream; declared above  
  Entry: TPMGListChunkEntry;
  NextIndex: Integer;
  EntryName: String;  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CRes: Integer;
  I: Integer;
begin
  Result := 0;
  //WriteLn('Looking for URL : ', Name);
  if Name = '' then Exit;
  if fDirectoryHeader.DirectoryChunkCount = 0 then exit;

  //WriteLn('Looking for ', Name);
  if Name = fCachedEntry.Name then
    Exit(fCachedEntry.DecompressedLength); // we've already looked it up

  ChunkStream := TMemoryStream.Create;

  try
<<<<<<< HEAD
<<<<<<< HEAD

  NextIndex := fDirectoryHeader.IndexOfRootChunk;
  if NextIndex < 0 then NextIndex := 0; // no PMGI chunks

=======
=======
>>>>>>> origin/fixes_2_2
  
  NextIndex := fDirectoryHeader.IndexOfRootChunk;
  if NextIndex < 0 then NextIndex := 0; // no PMGI chunks
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  while NextIndex > -1  do begin
    GetDirectoryChunk(NextIndex, ChunkStream);
    NextIndex := -1;
    ReadQuickRefSection;
<<<<<<< HEAD
<<<<<<< HEAD
    {$IFDEF CHM_DEBUG}
    WriteLn('In Block ', NextIndex);
    {$endif}
    case ChunkType(ChunkStream) of
      ctUnknown: // something is wrong
        begin
          {$IFDEF CHM_DEBUG}WriteLn(NextIndex, ' << Unknown BlockType!');{$ENDIF}
=======
=======
>>>>>>> origin/fixes_2_2
    //WriteLn('In Block ', ChunkIndex);
    case ChunkType(ChunkStream) of
      ctUnknown: // something is wrong
        begin
          {$IFDEF CHM_DEBUG}WriteLn(ChunkIndex, ' << Unknown BlockType!');{$ENDIF}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          Break;
        end;
      ctPMGI: // we must follow the PMGI tree until we reach a PMGL block
        begin
<<<<<<< HEAD
<<<<<<< HEAD
          LookupPMGIchunk(ChunkStream, PMGIChunk);
=======
          LookupPMGIchunk(ChunkStream, PMGIChunk);          
>>>>>>> graemeg/fixes_2_2
=======
          LookupPMGIchunk(ChunkStream, PMGIChunk);          
>>>>>>> origin/fixes_2_2

          //QuickRefIndex[0] := ChunkStream.Position;

          I := 0;
          while ChunkStream.Position <= ChunkStream.Size - PMGIChunk.UnusedSpace do begin;
            EntryName := ReadString;
            if EntryName = '' then break;
            if ChunkStream.Position >= ChunkStream.Size - PMGIChunk.UnusedSpace then break;
            CRes := ChmCompareText(Name, EntryName);
            if CRes = 0 then begin
<<<<<<< HEAD
<<<<<<< HEAD
              // no more need of this block. onto the next!
=======
              // no more need of this block. onto the next!              
>>>>>>> graemeg/fixes_2_2
=======
              // no more need of this block. onto the next!              
>>>>>>> origin/fixes_2_2
              NextIndex := GetCompressedInteger(ChunkStream);
              Break;
            end;
            if  CRes < 0 then begin
              if I = 0 then Break; // File doesn't exist
<<<<<<< HEAD
<<<<<<< HEAD
              // file is in previous entry
=======
              // file is in previous entry              
>>>>>>> graemeg/fixes_2_2
=======
              // file is in previous entry              
>>>>>>> origin/fixes_2_2
              Break;
            end;
            NextIndex := GetCompressedInteger(ChunkStream);
            Inc(I);
          end;
        end;
      ctPMGL:
        begin
          LookupPMGLchunk(ChunkStream, PMGLChunk);
          QuickRefIndex[0] := ChunkStream.Position;
          I := 0;
          while ChunkStream.Position <= ChunkStream.Size - PMGLChunk.UnusedSpace do begin
<<<<<<< HEAD
<<<<<<< HEAD
            // we consume the entry by reading it
=======
            // we consume the entry by reading it            
>>>>>>> graemeg/fixes_2_2
=======
            // we consume the entry by reading it            
>>>>>>> origin/fixes_2_2
            Entry.Name := ReadString;
            if Entry.Name = '' then break;
            if ChunkStream.Position >= ChunkStream.Size - PMGLChunk.UnusedSpace then break;

            Entry.ContentSection := GetCompressedInteger(ChunkStream);
            Entry.ContentOffset := GetCompressedInteger(ChunkStream);
            Entry.DecompressedLength := GetCompressedInteger(ChunkStream);

            CRes := ChmCompareText(Name, Entry.Name);
            if CRes = 0 then begin
              fCachedEntry := Entry;
<<<<<<< HEAD
<<<<<<< HEAD
              Result := Entry.DecompressedLength;
=======
              Result := Entry.DecompressedLength;              
>>>>>>> graemeg/fixes_2_2
=======
              Result := Entry.DecompressedLength;              
>>>>>>> origin/fixes_2_2
              Break;
            end;
            Inc(I);
          end;
        end; // case
    end;
  end;
  finally
  ChunkStream.Free;
  end;
end;

function TITSFReader.GetObject(Name: String): TMemoryStream;
var
  SectionNames: TStringList;
  Entry: TPMGListChunkEntry;
  SectionName: String;
begin
  Result := nil;
  if ObjectExists(Name) = 0 then begin
    //WriteLn('Object ', name,' Doesn''t exist or is zero sized.');
    Exit;
  end;

  Entry := fCachedEntry;
  if Entry.ContentSection = 0 then begin
    Result := TMemoryStream.Create;
    fStream.Position := fHeaderSuffix.Offset+ Entry.ContentOffset;
    Result.CopyFrom(fStream, fCachedEntry.DecompressedLength);
  end
  else begin // we have to get it from ::DataSpace/Storage/[MSCompressed,Uncompressed]/ControlData
    GetSections(SectionNames);
<<<<<<< HEAD
<<<<<<< HEAD
    FmtStr(SectionName, '::DataSpace/Storage/%s/',[SectionNames[Entry.ContentSection]]);
=======
    FmtStr(SectionName, '::DataSpace/Storage/%s/',[SectionNames[Entry.ContentSection-1]]);
>>>>>>> graemeg/fixes_2_2
=======
    FmtStr(SectionName, '::DataSpace/Storage/%s/',[SectionNames[Entry.ContentSection-1]]);
>>>>>>> origin/fixes_2_2
    Result := GetBlockFromSection(SectionName, Entry.ContentOffset, Entry.DecompressedLength);
    SectionNames.Free;
  end;
  if Result <> nil then Result.Position := 0;
end;

function TChmReader.GetContextUrl(Context: THelpContext): String;
begin
  // will get '' if context not found
 Result := fContextList.GetURL(Context);
end;

function TChmReader.LookupTopicByID ( ATopicID: Integer; out ATitle: String) : String;
var
  TopicURLTBLOffset: DWord;
  TopicTitleOffset: DWord;
begin
  Result := '';
  ATitle := '';
  //WriteLn('Getting topic# ',ATopicID);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if not CheckCommonStreams then
=======
  if fTOPICSStream = nil then
    fTOPICSStream := GetObject('/#TOPICS');
  if fTOPICSStream = nil then
>>>>>>> graemeg/fixes_2_2
=======
  if fTOPICSStream = nil then
    fTOPICSStream := GetObject('/#TOPICS');
  if fTOPICSStream = nil then
>>>>>>> origin/fixes_2_2
=======
  if not CheckCommonStreams then
>>>>>>> origin/fixes_2.4
    Exit;
  fTOPICSStream.Position := ATopicID * 16;
  if fTOPICSStream.Position = ATopicID * 16 then
  begin
    fTOPICSStream.ReadDWord;
    TopicTitleOffset := LEtoN(fTOPICSStream.ReadDWord);
    TopicURLTBLOffset := LEtoN(fTOPICSStream.ReadDWord);
    if TopicTitleOffset <> $FFFFFFFF then
      ATitle := ReadStringsEntry(TopicTitleOffset);
     //WriteLn('Got a title: ', ATitle);
    Result := ReadURLSTR(TopicURLTBLOffset);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
const DefBlockSize = 2048;

function LoadBtreeHeader(m:TMemoryStream;var btreehdr:TBtreeHeader):boolean;

begin
  if m.size<sizeof(TBtreeHeader) Then
    Exit(False);
  result:=true;
  m.read(btreeHdr,sizeof(TBtreeHeader));
  {$IFDEF ENDIAN_BIG}
     btreehdr.flags         :=LEToN(btreehdr.flags);
     btreehdr.blocksize     :=LEToN(btreehdr.blocksize);
     btreehdr.lastlstblock  :=LEToN(btreehdr.lastlstblock);
     btreehdr.indexrootblock:=LEToN(btreehdr.indexrootblock);
     btreehdr.nrblock       :=LEToN(btreehdr.nrblock);
     btreehdr.treedepth     :=LEToN(btreehdr.treedepth);
     btreehdr.nrkeywords    :=LEToN(btreehdr.nrkeywords);
     btreehdr.codepage      :=LEToN(btreehdr.codepage);
     btreehdr.lcid          :=LEToN(btreehdr.lcid);
     btreehdr.ischm         :=LEToN(btreehdr.ischm);
  {$endif}
end;

function readwcharstring(var head:pbyte;tail:pbyte;var readv : ansistring):boolean;

var pw      : PWord;
    oldhead : PByte;
    ws      : WideString;
    n       : Integer;
begin
  oldhead:=head;
  pw:=pword(head);
  while (pw<pword(tail)) and (pw^<>word(0)) do
    inc(pw);
  inc(pw); // skip #0#0.
  head:=pbyte(pw);
  result:=head<tail;

  n:=head-oldhead;
  setlength(ws,n div sizeof(widechar));
  move(oldhead^,ws[1],n);
  for n:=1 to length(ws) do
    word(ws[n]):=LEToN(word(ws[n]));
  readv:=ws; // force conversion for now, and hope it doesn't require cwstring
end;

function TChmReader.GetIndexSitemap(ForceXML:boolean=false): TChmSiteMap;
var Index   : TMemoryStream;
    sitemap : TChmSiteMap;
    Item    : TChmSiteMapItem;
<<<<<<< HEAD

=======
    
>>>>>>> origin/fixes_2.4
function  AbortAndTryTextual:tchmsitemap;

begin
     if Assigned(Index) Then Index.Free;
     // Second Try text Index
     Index := GetObject(IndexFile);
     if Index <> nil then
     begin
       Result := TChmSiteMap.Create(stIndex);
       Result.LoadFromStream(Index);
       Index.Free;
     end
    else
      result:=nil;
end;

procedure createentry(Name:ansistring;CharIndex:integer;Topic,Title:ansistring);
var litem : TChmSiteMapItem;
    shortname : ansistring;
    longpart  : ansistring;
begin
 if charindex=0 then
   begin
     item:=sitemap.items.NewItem;
     item.keyword:=Name;
     item.local:=topic;
     item.text:=title;
   end
 else
   begin
     shortname:=copy(name,1,charindex-2);
     longpart:=copy(name,charindex,length(name)-charindex+1);
     if assigned(item) and (shortname=item.text) then
       begin
         litem:=item.children.newitem;
         litem.local:=topic;
         litem.keyword :=longpart; // recursively split this? No examples.
         litem.text:=title;
       end
      else
       begin
         item:=sitemap.items.NewItem;
         item.keyword:=shortname;
         item.local:=topic;
         item.text:=title;
         litem:=item.children.newitem;
         litem.keyword:=longpart;
         litem.local:=topic;
         litem.text :=Title; // recursively split this? No examples.
       end;
<<<<<<< HEAD
   end;
end;

procedure parselistingblock(p:pbyte);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
    itemstack:TObjectStack;
    curitemdepth : integer;
    parentitem:TChmSiteMap;

procedure updateparentitem(entrydepth:integer);
begin
  if entrydepth>curitemdepth then
    begin
      if curitemdepth<>0 then
        itemstack.push(parentitem);
      curitemdepth:=entrydepth;
    end
  else
   if entrydepth>curitemdepth then
    begin
      if curitemdepth<>0 then
        itemstack.push(parentitem);
      curitemdepth:=entrydepth;
    end
end;

var hdr:PBTreeBlockHeader;
    head,tail : pbyte;
    isseealso,
    entrydepth,
=======
var hdr:PBTreeBlockHeader;
    head,tail : pbyte;
    isseealso,
>>>>>>> graemeg/cpstrnew
=======
var hdr:PBTreeBlockHeader;
    head,tail : pbyte;
    isseealso,
>>>>>>> graemeg/cpstrnew
=======
var hdr:PBTreeBlockHeader;
    head,tail : pbyte;
    isseealso,
>>>>>>> graemeg/cpstrnew
=======
var hdr:PBTreeBlockHeader;
    head,tail : pbyte;
    isseealso,
>>>>>>> origin/cpstrnew
=======
   end;  
end;

procedure parselistingblock(p:pbyte);
var hdr:PBTreeBlockHeader;
    head,tail : pbyte;
    isseealso,
>>>>>>> origin/fixes_2.4
    nrpairs : Integer;
    i : integer;
    PE : PBtreeBlockEntry;
    title : string;
    CharIndex,
    ind:integer;
    seealsostr,
    topic,
    Name : AnsiString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  //setlength (curitem,10);
=======
    item : TChmSiteMapItem;
begin
>>>>>>> graemeg/cpstrnew
=======
    item : TChmSiteMapItem;
begin
>>>>>>> graemeg/cpstrnew
=======
    item : TChmSiteMapItem;
begin
>>>>>>> graemeg/cpstrnew
=======
    item : TChmSiteMapItem;
begin
>>>>>>> origin/cpstrnew
=======
    item : TChmSiteMapItem;
begin
>>>>>>> origin/fixes_2.4
  hdr:=PBTreeBlockHeader(p);
  hdr^.Length          :=LEToN(hdr^.Length);
  hdr^.NumberOfEntries :=LEToN(hdr^.NumberOfEntries);
  hdr^.IndexOfPrevBlock:=LEToN(hdr^.IndexOfPrevBlock);
  hdr^.IndexOfNextBlock:=LEToN(hdr^.IndexOfNextBlock);

  tail:=p+(2048-hdr^.length);
  head:=p+sizeof(TBtreeBlockHeader);
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  itemstack:=TObjectStack.create;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  
>>>>>>> origin/fixes_2.4
  {$ifdef binindex}
  writeln('previndex  : ',hdr^.IndexOfPrevBlock);
  writeln('nextindex  : ',hdr^.IndexOfNextBlock);
  {$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  curitemdepth:=0;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
  while head<tail do
    begin
      if not ReadWCharString(Head,Tail,Name) Then
        Break;
      {$ifdef binindex}
         Writeln('name : ',name);
      {$endif}
       if (head+sizeof(TBtreeBlockEntry))>=tail then
         break;
      PE :=PBtreeBlockEntry(head);
      NrPairs  :=LEToN(PE^.nrpairs);
      IsSeealso:=LEToN(PE^.isseealso);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      EntryDepth:=LEToN(PE^.entrydepth);
      CharIndex:=LEToN(PE^.CharIndex);
      {$ifdef binindex}
        Writeln('seealso   :  ',IsSeeAlso);
        Writeln('entrydepth:  ',EntryDepth);
        Writeln('charindex :  ',charindex );
        Writeln('Nrpairs   :  ',NrPairs);
        Writeln('CharIndex :  ',charindex);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
      CharIndex:=LEToN(PE^.CharIndex);
      {$ifdef binindex}
        Writeln('seealso:     ',IsSeeAlso);
        Writeln('entrydepth:  ',LEToN(PE^.entrydepth));
        Writeln('charindex :  ',charindex );
        Writeln('Nrpairs   :  ',NrPairs);
        writeln('seealso data : ');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
      {$endif}

      inc(head,sizeof(TBtreeBlockEntry));
      if isseealso>0 then
        begin
          if not ReadWCharString(Head,Tail,SeeAlsoStr) Then
            Break;
          // have to figure out first what to do with it.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          // is See Also really mutually exclusive with pairs?
          // or is the number of pairs equal to the number of seealso
          // strings?
          {$ifdef binindex}
            writeln('seealso: ',seealsostr);
          {$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
        end
      else
        begin
         if NrPairs>0 Then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          begin
            {$ifdef binindex}
             writeln('Pairs   : ');
            {$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
            for i:=0 to nrpairs-1 do
              begin
                if head<tail Then
                  begin
                    ind:=LEToN(plongint(head)^);
                    topic:=lookuptopicbyid(ind,title);
                    {$ifdef binindex}
                      writeln(i:3,' topic: ',topic);
                      writeln('    title: ',title);
                    {$endif}
                    inc(head,4);
                  end;
              end;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
      if nrpairs<>0 Then
        createentry(Name,CharIndex,Topic,Title);
      inc(head,4); // always 1
      {$ifdef binindex}
        if head<tail then
        writeln('Zero based index (13 higher than last) :',plongint(head)^);
      {$endif}
      inc(head,4); // zero based index (13 higher than last
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ItemStack.Free;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
end;

var TryTextual : boolean;
    BHdr       : TBTreeHeader;
    block      : Array[0..2047] of Byte;
    i          : Integer;
begin
   Result := nil;  SiteMap:=Nil;
   // First Try Binary
   Index := GetObject('/$WWKeywordLinks/BTree');
   if (Index = nil) or ForceXML then
   begin
     Result:=AbortAndTryTextual;
     Exit;
   end;
   if not CheckCommonStreams then
   begin
     Result:=AbortAndTryTextual;
     Exit;
   end;
   SiteMap:=TChmSitemap.Create(StIndex);
<<<<<<< HEAD
   Item   :=Nil;  // cached last created item, in case we need to make
                  // a child.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

   TryTextual:=True;
   BHdr.LastLstBlock:=0;
   if LoadBtreeHeader(index,BHdr) and (BHdr.LastLstBlock>=0) Then
=======
   TryTextual:=True;
   BHdr.LastLstBlock:=0;
   if LoadBtreeHeader(index,BHdr) and (BHdr.LastLstBlock>0) Then
>>>>>>> graemeg/cpstrnew
=======
   TryTextual:=True;
   BHdr.LastLstBlock:=0;
   if LoadBtreeHeader(index,BHdr) and (BHdr.LastLstBlock>0) Then
>>>>>>> graemeg/cpstrnew
=======
   TryTextual:=True;
   BHdr.LastLstBlock:=0;
   if LoadBtreeHeader(index,BHdr) and (BHdr.LastLstBlock>0) Then
>>>>>>> graemeg/cpstrnew
=======
   TryTextual:=True;
   BHdr.LastLstBlock:=0;
   if LoadBtreeHeader(index,BHdr) and (BHdr.LastLstBlock>0) Then
>>>>>>> origin/cpstrnew
    begin
=======
   Item   :=Nil;  // cached last created item, in case we need to make 
                  // a child.
   TryTextual:=True;
   BHdr.LastLstBlock:=0;
   if LoadBtreeHeader(index,BHdr) and (BHdr.LastLstBlock>0) Then
    begin 
>>>>>>> origin/fixes_2.4
       if BHdr.BlockSize=defblocksize then
         begin
           for i:=0 to BHdr.lastlstblock do
             begin
               if (index.size-index.position)>=defblocksize then
                 begin
                   Index.read(block,defblocksize);
                   parselistingblock(@block)
                end;
             end;
            trytextual:=false;
<<<<<<< HEAD
            result:=sitemap;
          end;
=======
            result:=sitemap; 
          end;   
>>>>>>> origin/fixes_2.4
    end;
  if trytextual then
    begin
      sitemap.free;
      Result:=AbortAndTryTextual;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    end
  else Index.Free;
=======
    end;
>>>>>>> graemeg/cpstrnew
=======
    end;
>>>>>>> graemeg/cpstrnew
=======
    end;
>>>>>>> graemeg/cpstrnew
=======
    end;
>>>>>>> origin/cpstrnew
=======
    end;
>>>>>>> origin/fixes_2.4
end;

function TChmReader.GetTOCSitemap(ForceXML:boolean=false): TChmSiteMap;
    function AddTOCItem(TOC: TStream; AItemOffset: DWord; SiteMapITems: TChmSiteMapItems): DWord;
    var
      Props: DWord;
      Item: TChmSiteMapItem;
      NextEntry: DWord;
      TopicsIndex: DWord;
      Title: String;
    begin
      Toc.Position:= AItemOffset + 4;
      Item := SiteMapITems.NewItem;
      Props := LEtoN(TOC.ReadDWord);
      if (Props and TOC_ENTRY_HAS_LOCAL) = 0 then
        Item.Text:= ReadStringsEntry(LEtoN(TOC.ReadDWord))
      else
      begin
        TopicsIndex := LEtoN(TOC.ReadDWord);
        Item.Local := LookupTopicByID(TopicsIndex, Title);
        Item.Text := Title;

      end;
      TOC.ReadDWord;
      Result := LEtoN(TOC.ReadDWord);
      if Props and TOC_ENTRY_HAS_CHILDREN > 0 then
      begin
        NextEntry := LEtoN(TOC.ReadDWord);
        repeat
          NextEntry := AddTOCItem(TOC, NextEntry, Item.Children);
        until NextEntry = 0;
      end;

    end;

var
  TOC: TStream;
  TOPICSOffset: DWord;
  EntriesOffset: DWord;
  EntryCount: DWord;
  EntryInfoOffset: DWord;
  NextItem: DWord;
begin
   Result := nil;
   // First Try Binary
   TOC := GetObject('/#TOCIDX');
   if (TOC = nil) or ForceXML then
   begin
     if Assigned(TOC) Then Toc.Free;
     // Second Try text toc
     TOC := GetObject(TOCFile);
     if TOC <> nil then
     begin
       Result := TChmSiteMap.Create(stTOC);
       Result.LoadFromStream(TOC);
       Toc.Free;
     end;
     Exit;
   end;

   // TOPICS URLSTR URLTBL must all exist to read binary toc
   // if they don't then try text file
   if not CheckCommonStreams then
   begin
     TOC.Free;
     TOC := GetObject(TOCFile);
     if TOC <> nil then
     begin
       Result := TChmSiteMap.Create(stTOC);
       Result.LoadFromStream(TOC);
       Toc.Free;
     end;
     Exit;
   end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     // Binary Toc Exists
=======
   // Binary Toc Exists
>>>>>>> graemeg/cpstrnew
=======
   // Binary Toc Exists
>>>>>>> graemeg/cpstrnew
=======
   // Binary Toc Exists
>>>>>>> graemeg/cpstrnew
=======
   // Binary Toc Exists
>>>>>>> origin/cpstrnew
=======
   // Binary Toc Exists
>>>>>>> origin/fixes_2.4
   Result := TChmSiteMap.Create(stTOC);

   EntryInfoOffset := NtoLE(TOC.ReadDWord);
   EntriesOffset   := NtoLE(TOC.ReadDWord);
   EntryCount      := NtoLE(TOC.ReadDWord);
   TOPICSOffset    := NtoLE(TOC.ReadDWord);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   if EntryCount = 0 then
     begin
       Toc.Free;
       Exit;
     end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
   NextItem := EntryInfoOffset;
   repeat
     NextItem := AddTOCItem(Toc, NextItem, Result.Items);
   until NextItem = 0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   TOC.Free;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
end;

>>>>>>> origin/fixes_2.4
function TChmReader.HasContextList: Boolean;
begin
  Result := fContextList.Count > 0;
end;

procedure TITSFReader.GetSections(out Sections: TStringList);
var
  Stream: TStream;
  EntryCount: Word;
  X: Integer;
  {$IFDEF ENDIAN_BIG}
  I: Integer;
  {$ENDIF}
  WString: array [0..31] of WideChar;
  StrLength: Word;
begin
  Sections := TStringList.Create;
  //WriteLn('::DataSpace/NameList Size = ', ObjectExists('::DataSpace/NameList'));
  Stream := GetObject('::DataSpace/NameList');

  if Stream = nil then begin
    //WriteLn('Failed to get ::DataSpace/NameList!');
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  Stream.Position := 2;
  EntryCount := LEtoN(Stream.ReadWord);
  for X := 0 to EntryCount -1 do begin
    StrLength := LEtoN(Stream.ReadWord);
    if StrLength > 31 then StrLength := 31;
    Stream.Read(WString, SizeOf(WideChar)*(StrLength+1)); // the strings are stored null terminated
    {$IFDEF ENDIAN_BIG}
    for I := 0 to StrLength-1 do
      WString[I] := WideChar(LEtoN(Ord(WString[I])));
    {$ENDIF}
    Sections.Add(WString);
  end;
<<<<<<< HEAD
<<<<<<< HEAD
=======
  // the sections are sorted alphabetically, this way section indexes will jive
  Sections.Sort;
>>>>>>> graemeg/fixes_2_2
=======
  // the sections are sorted alphabetically, this way section indexes will jive
  Sections.Sort;
>>>>>>> origin/fixes_2_2
  Stream.Free;
end;

function TITSFReader.GetBlockFromSection(SectionPrefix: String; StartPos: QWord;
  BlockLength: QWord): TMemoryStream;
var
  Compressed: Boolean;
  Sig: Array [0..3] of char;
  CompressionVersion: LongWord;
  CompressedSize: QWord;
  UnCompressedSize: QWord;
  //LZXResetInterval: LongWord;
  //LZXWindowSize: LongWord;
  //LZXCacheSize: LongWord;
  ResetTableEntry: TPMGListChunkEntry;
  ResetTable: TLZXResetTableArr;
  WriteCount: QWord;
  BlockWriteLength: QWord;
  WriteStart: LongWord;
  ReadCount:LongInt;
  LZXState: PLZXState;
  InBuf: array of Byte;
  OutBuf: PByte;
  BlockSize: QWord;
  X: Integer;
  FirstBlock, LastBlock: LongInt;
  ResultCode: LongInt;
  procedure ReadBlock;
  begin
    if ReadCount > Length(InBuf) then
      SetLength(InBuf, ReadCount);
    fStream.Read(InBuf[0], ReadCount);
  end;
begin
  // okay now the fun stuff ;)
  Result := nil;
  Compressed := ObjectExists(SectionPrefix+'Transform/{7FC28940-9D31-11D0-9B27-00A0C91E9C7C}/InstanceData/ResetTable')>0;
  // the easy method
  if Not(Compressed) then begin
    if ObjectExists(SectionPrefix+'Content') > 0 then begin
      Result := TMemoryStream.Create;
      fStream.Position := fHeaderSuffix.Offset + fCachedEntry.ContentOffset + StartPos;
      Result.CopyFrom(fStream, BlockLength);
    end;
    Exit;
  end
  else
    ResetTableEntry := fCachedEntry;

  // First make sure that it is a compression we can read
  if ObjectExists(SectionPrefix+'ControlData') > 0 then begin
    fStream.Position := fHeaderSuffix.Offset + fCachedEntry.ContentOffset + 4;
    fStream.Read(Sig, 4);
    if Sig <> 'LZXC' then Exit;
    CompressionVersion := LEtoN(fStream.ReadDWord);
    if CompressionVersion > 2 then exit;
    {LZXResetInterval := }LEtoN(fStream.ReadDWord);
    {LZXWindowSize := }LEtoN(fStream.ReadDWord);
    {LZXCacheSize := }LEtoN(fStream.ReadDWord);


    BlockSize := FindBlocksFromUnCompressedAddr(ResetTableEntry, CompressedSize, UnCompressedSize, ResetTable);
    if UncompressedSize > 0 then ; // to avoid a compiler note
    if StartPos > 0 then
      FirstBlock := StartPos div BlockSize
    else
      FirstBlock := 0;
    LastBlock := (StartPos+BlockLength) div BlockSize;

    if ObjectExists(SectionPrefix+'Content') = 0 then exit;
    //WriteLn('Compressed Data start''s at: ', fHeaderSuffix.Offset + fCachedEntry.ContentOffset,' Size is: ', fCachedEntry.DecompressedLength);
    Result := TMemoryStream.Create;
    Result.Size := BlockLength;
    SetLength(InBuf,BlockSize);
    OutBuf := GetMem(BlockSize);
    // First Init a PLZXState
    LZXState := LZXinit(16);
    if LZXState = nil then begin
      Exit;
    end;
    // if FirstBlock is odd (1,3,5,7 etc) we have to read the even block before it first.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if FirstBlock and 1 = 1 then begin
=======
    if (FirstBlock <> 0) and (FirstBlock mod 2 > 0) then begin
>>>>>>> graemeg/fixes_2_2
=======
    if (FirstBlock <> 0) and (FirstBlock mod 2 > 0) then begin
>>>>>>> origin/fixes_2_2
=======
    if FirstBlock and 1 = 1 then begin
>>>>>>> origin/fixes_2.4
      fStream.Position := fHeaderSuffix.Offset + fCachedEntry.ContentOffset + (ResetTable[FirstBLock-1]);
      ReadCount := ResetTable[FirstBlock] - ResetTable[FirstBlock-1];
      BlockWriteLength:=BlockSize;
      ReadBlock;
      ResultCode := LZXdecompress(LZXState, @InBuf[0], OutBuf, ReadCount, LongInt(BlockWriteLength));
    end;
    // now start the actual decompression loop
    for X := FirstBlock to LastBlock do begin
      fStream.Position := fHeaderSuffix.Offset + fCachedEntry.ContentOffset + (ResetTable[X]);

      if X = FirstBLock then
        WriteStart := StartPos - (X*BlockSize)
      else
        WriteStart := 0;

      if X = High(ResetTable) then
        ReadCount := CompressedSize - ResetTable[X]
      else
        ReadCount := ResetTable[X+1] - ResetTable[X];

      BlockWriteLength := BlockSize;
<<<<<<< HEAD
<<<<<<< HEAD

=======
      
>>>>>>> graemeg/fixes_2_2
=======
      
>>>>>>> origin/fixes_2_2
      if FirstBlock = LastBlock then begin
        WriteCount := BlockLength;
      end
      else if X = LastBlock then
        WriteCount := (StartPos+BlockLength) - (X*BlockSize)
      else WriteCount := BlockSize - WriteStart;

      ReadBlock;
      ResultCode := LZXdecompress(LZXState, @InBuf[0], OutBuf, ReadCount, LongInt(BlockWriteLength));

      //now write the decompressed data to the stream
      if ResultCode = DECR_OK then begin
        Result.Write(OutBuf[WriteStart], QWord(WriteCount));
      end
      else begin
        {$IFDEF CHM_DEBUG} // windows gui program will cause an exception with writeln's
        WriteLn('Decompress FAILED with error code: ', ResultCode);
        {$ENDIF}
        Result.Free;
        Result := Nil;
        FreeMem(OutBuf);
        SetLength(ResetTable,0);
        LZXteardown(LZXState);
        Exit;
      end;
<<<<<<< HEAD
<<<<<<< HEAD

      // if the next block is an even numbered block we have to reset the decompressor state
      if (X < LastBlock) and (X and 1 = 1) then LZXreset(LZXState);
=======
      
      // if the next block is an even numbered block we have to reset the decompressor state
<<<<<<< HEAD
      if (X < LastBlock) and (X mod 2 > 0) then LZXreset(LZXState);
>>>>>>> graemeg/fixes_2_2
=======
      
      // if the next block is an even numbered block we have to reset the decompressor state
      if (X < LastBlock) and (X mod 2 > 0) then LZXreset(LZXState);
>>>>>>> origin/fixes_2_2
=======
      if (X < LastBlock) and (X and 1 = 1) then LZXreset(LZXState);
>>>>>>> origin/fixes_2.4

    end;
    FreeMem(OutBuf);
    SetLength(ResetTable,0);
    LZXteardown(LZXState);
  end;
end;

function TITSFReader.FindBlocksFromUnCompressedAddr(var ResetTableEntry: TPMGListChunkEntry;
  out CompressedSize: QWord; out UnCompressedSize: QWord; out LZXResetTable: TLZXResetTableArr): QWord;
var
  BlockCount: LongWord;
  {$IFDEF ENDIAN_BIG}
  I: Integer;
  {$ENDIF}
begin
  Result := 0;
  fStream.Position := fHeaderSuffix.Offset + ResetTableEntry.ContentOffset;
  fStream.ReadDWord;
  BlockCount := LEtoN(fStream.ReadDWord);
  fStream.ReadDWord;
  fStream.ReadDWord; // TableHeaderSize;
  fStream.Read(UnCompressedSize, SizeOf(QWord));
  UnCompressedSize := LEtoN(UnCompressedSize);
  fStream.Read(CompressedSize, SizeOf(QWord));
  CompressedSize := LEtoN(CompressedSize);
  fStream.Read(Result, SizeOf(QWord)); // block size
  Result := LEtoN(Result);

  // now we are located at the first block index

  SetLength(LZXResetTable, BlockCount);
  fStream.Read(LZXResetTable[0], SizeOf(QWord)*BlockCount);
  {$IFDEF ENDIAN_BIG}
  for I := 0 to High(LZXResetTable) do
    LZXResetTable[I] := LEtoN(LZXResetTable[I]);
  {$ENDIF}
end;

{ TContextList }

procedure TContextList.AddContext(Context: THelpContext; Url: String);
var
  ContextItem: PContextItem;
begin
  New(ContextItem);
  Add(ContextItem);
  ContextItem^.Context := Context;
  ContextItem^.Url := Url;
end;

function TContextList.GetURL(Context: THelpContext): String;
var
  X: Integer;
begin
  Result := '';
  for X := 0 to Count-1 do begin
    if PContextItem(Get(X))^.Context = Context then begin
      Result := PContextItem(Get(X))^.Url;
      Exit;
    end;
  end;
end;

procedure TContextList.Clear;
var
  X: Integer;
begin
  for X := Count-1 downto 0 do begin
    Dispose(PContextItem(Get(X)));
    Delete(X);
  end;
end;


{ TChmFileList }

procedure TChmFileList.Delete(Index: Integer);
begin
  Chm[Index].Free;
  inherited Delete(Index);
end;

function TChmFileList.GetChm(AIndex: Integer): TChmReader;
begin
  if AIndex = -1 then
    Result := fLastChm
  else
    Result := TChmReader(Objects[AIndex]);
end;

function TChmFileList.GetFileName(AIndex: Integer): String;
begin
  if AIndex = -1 then
    AIndex := IndexOfObject(fLastChm);

   Result := Strings[AIndex];
end;

procedure TChmFileList.OpenNewFile(AFileName: String);
var
AStream: TFileStream;
AChm: TChmReader;
AIndex: Integer;
begin
  if not FileExists(AFileName) then exit;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AStream := TFileStream.Create(AFileName, fmOpenRead or fmShareDenyWrite);
=======
  AStream := TFileStream.Create(AFileName, fmOpenRead, fmShareDenyWrite);
>>>>>>> graemeg/cpstrnew
=======
  AStream := TFileStream.Create(AFileName, fmOpenRead, fmShareDenyWrite);
>>>>>>> graemeg/cpstrnew
=======
  AStream := TFileStream.Create(AFileName, fmOpenRead, fmShareDenyWrite);
>>>>>>> graemeg/cpstrnew
=======
  AStream := TFileStream.Create(AFileName, fmOpenRead, fmShareDenyWrite);
>>>>>>> origin/cpstrnew
=======
  AStream := TFileStream.Create(AFileName, fmOpenRead);
>>>>>>> graemeg/fixes_2_2
=======
  AStream := TFileStream.Create(AFileName, fmOpenRead);
>>>>>>> origin/fixes_2_2
=======
  AStream := TFileStream.Create(AFileName, fmOpenRead, fmShareDenyWrite);
>>>>>>> origin/fixes_2.4
  AChm := TChmReader.Create(AStream, True);
  AIndex := AddObject(AFileName, AChm);
  fLastChm := AChm;
  if Assigned(fOnOpenNewFile) then fOnOpenNewFile(Self, AIndex)
  else fUnNotifiedFiles.Add(AChm);
end;

function TChmFileList.CheckOpenFile(AFileName: String): Boolean;
var
  X: Integer;
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
begin
  Result := False;
  for X := 0 to Count-1 do begin
    if ExtractFileName(FileName[X]) = AFileName then begin
      fLastChm := Chm[X];
      Result := True;
      Exit;
    end;
  end;
  if not Result then begin
    AFileName := ExtractFilePath(FileName[0])+AFileName;
    if FileExists(AFileName) and (ExtractFileExt(AFileName) = '.chm') then OpenNewFile(AFileName);
    Result := True;
  end;
end;

function TChmFileList.MetaObjectExists(var Name: String): QWord;
var
  AFileName: String;
  URL: String;
  fStart, fEnd: Integer;
  Found: Boolean;
begin
  Found := False;
  Result := 0;
  //Known META file link types
  //       ms-its:name.chm::/topic.htm
  //mk:@MSITStore:name.chm::/topic.htm
  if Pos('ms-its:', Name) > 0 then begin
    fStart := Pos('ms-its:', Name)+Length('ms-its:');
    fEnd := Pos('::', Name)-fStart;
    AFileName := Copy(Name, fStart, fEnd);
    fStart := fEnd+fStart+2;
    fEnd := Length(Name) - (fStart-1);
    URL := Copy(Name, fStart, fEnd);
    Found := True;
  end
  else if Pos('mk:@MSITStore:', Name) > 0 then begin
    fStart := Pos('mk:@MSITStore:', Name)+Length('mk:@MSITStore:');
    fEnd := Pos('::', Name)-fStart;
    AFileName := Copy(Name, fStart, fEnd);
    fStart := fEnd+fStart+2;
    fEnd := Length(Name) - (fStart-1);
    URL := Copy(Name, fStart, fEnd);
    Found := True;
  end;
  if not Found then exit;
  //WriteLn('Looking for URL ', URL, ' in ', AFileName);
  if CheckOpenFile(AFileName) then
    Result := fLastChm.ObjectExists(URL);
  if Result > 0 then NAme := Url;
end;

function TChmFileList.MetaGetObject(Name: String): TMemoryStream;
begin
  Result := nil;
  if MetaObjectExists(Name) > 0 then Result := fLastChm.GetObject(Name);
end;

constructor TChmFileList.Create(PrimaryFileName: String);
begin
  inherited Create;
  fUnNotifiedFiles := TList.Create;
  OpenNewFile(PrimaryFileName);
end;

destructor TChmFileList.Destroy;
begin
  fUnNotifiedFiles.Free;
  inherited Destroy;
end;

procedure TChmFileList.SetOnOpenNewFile(AValue: TChmFileOpenEvent);
var
  X: Integer;
begin
  fOnOpenNewFile := AValue;
  if AValue = nil then exit;
  for X := 0 to fUnNotifiedFiles.Count-1 do
    AValue(Self, X);
  fUnNotifiedFiles.Clear;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TChmFileList.ObjectExists(Name: String; var fChm: TChmReader): QWord;
=======
function TChmFileList.ObjectExists(Name: String; var fChm: TChmReader = nil): QWord;
>>>>>>> graemeg/fixes_2_2
=======
function TChmFileList.ObjectExists(Name: String; var fChm: TChmReader = nil): QWord;
>>>>>>> origin/fixes_2_2
begin
  Result := 0;
  if Count = 0 then exit;
  if fChm <> nil then fLastChm := fChm;
  Result := fLastChm.ObjectExists(Name);
  if Result = 0 then begin
    Result := Chm[0].ObjectExists(Name);
    if Result > 0 then fLastChm := Chm[0];
  end;
  if Result = 0 then begin
    Result := MetaObjectExists(Name);
  end;
  if (Result <> 0) and (fChm = nil) then
    fChm := fLastChm;
end;

function TChmFileList.GetObject(Name: String): TMemoryStream;
begin
  Result := nil;
  if Count = 0 then exit;
  Result := fLastChm.GetObject(Name);
  if Result = nil then Result := MetaGetObject(Name);
end;

function TChmFileList.IsAnOpenFile(AFileName: String): Boolean;
var
  X: Integer;
begin
  Result := False;
  for X := 0 to Count-1 do begin
    if AFileName = FileName[X] then Exit(True);
  end;
end;

end.

