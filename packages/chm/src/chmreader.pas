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
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
}
{
  See the file COPYING.modifiedLGPL, included in this distribution,
  for details about the copyright.
}
unit chmreader;

{$mode delphi}

//{$DEFINE CHM_DEBUG}
{ $DEFINE CHM_DEBUG_CHUNKS}
{define binindex}
{define nonumber}
interface

uses
  Generics.Collections, Classes, SysUtils,  Contnrs,
  chmbase, paslzx, chmFIftiMain, chmsitemap;

type

  TLZXResetTableArr = array of QWord;

  PContextItem = ^TContextItem;
  TContextItem = record
    Context: THelpContext;
    Url: String;
  end;

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
    fITSFHeader: TITSFHeader;
    fHeaderSuffix: TITSFHeaderSuffix;
    fDirectoryHeader: TITSPHeader;
    fDirectoryHeaderPos: QWord;
    fDirectoryHeaderLength: QWord;
    fDirectoryEntriesStartPos: QWord;
    fCachedEntry: TPMGListChunkEntry; //contains the last entry found by ObjectExists
    fDirectoryEntriesCount: LongWord;
    procedure ReadHeader; virtual;
    procedure ReadHeaderEntries; virtual;
    function  GetChunkType(Stream: TMemoryStream; ChunkIndex: LongInt): TDirChunkType;
    procedure GetSections(out Sections: TStringList);
  private
    function  GetDirectoryChunk(Index: Integer; OutStream: TStream): Integer;
    function  ReadPMGLchunkEntryFromStream(Stream: TMemoryStream; var PMGLEntry: TPMGListChunkEntry): Boolean;
    function  ReadPMGIchunkEntryFromStream(Stream: TMemoryStream; var PMGIEntry: TPMGIIndexChunkEntry): Boolean;
    procedure LookupPMGLchunk(Stream: TMemoryStream; out PMGLChunk: TPMGListChunk);
    procedure LookupPMGIchunk(Stream: TMemoryStream; out PMGIChunk: TPMGIIndexChunk);


    function  GetBlockFromSection(SectionPrefix: String; StartPos: QWord; BlockLength: QWord): TMemoryStream;
    function  FindBlocksFromUnCompressedAddr(var ResetTableEntry: TPMGListChunkEntry;
       out CompressedSize: QWord; out UnCompressedSize: QWord; out LZXResetTable: TLZXResetTableArr): QWord;  // Returns the blocksize
  public
    constructor Create(AStream: TStream; FreeStreamOnDestroy: Boolean); virtual;
    destructor Destroy; override;
  public
    ChmLastError: LongInt;
    function IsValidFile: Boolean;
    procedure GetCompleteFileList(ForEach: TFileEntryForEach; AIncludeInternalFiles: Boolean = True); virtual;
    function ObjectExists(Name: String): QWord; virtual; // zero if no. otherwise it is the size of the object
                                                // NOTE directories will return zero size even if they exist
    function GetObject(Name: String): TMemoryStream; virtual; // YOU must Free the stream
    property CachedEntry: TPMGListChunkEntry read fCachedEntry;
  end;

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
    fWindowsList : TObjectList;
    fDefaultWindow: String;
  private
    FSearchReader: TChmSearchReader;
  public
    procedure ReadCommonData;
    function  ReadStringsEntry(APosition: DWord): String;
    function  ReadStringsEntryFromStream ( strm:TStream ) : String;
    { Return LocalUrl string from #URLSTR }
    function  ReadURLSTR(APosition: DWord): String;
    function  CheckCommonStreams: Boolean;
    procedure ReadWindows(mem:TMemoryStream);
    constructor Create(AStream: TStream; FreeStreamOnDestroy: Boolean); override;
    destructor Destroy; override;
    function GetContextUrl(Context: THelpContext): String;
    function LookupTopicByID(ATopicID: Integer; out ATitle: String): String; // returns a url
    function GetTOCSitemap(ForceXML:boolean=false): TChmSiteMap;
    function GetIndexSitemap(ForceXML:boolean=false): TChmSiteMap;
    function HasContextList: Boolean;
    property DefaultPage: String read fDefaultPage;
    property IndexFile: String read fIndexFile;
    property TOCFile: String read fTOCFile;
    property Title: String read fTitle write fTitle;
    property PreferedFont: String read fPreferedFont;
    property LocaleID: dword read fLocaleID;
    property SearchReader: TChmSearchReader read FSearchReader write FSearchReader;
    property contextlist : tcontextlist read fcontextlist;
    property Windows : TObjectlist read fWindowsList;
    property DefaultWindow : string read fdefaultwindow;
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
    function ObjectExists(Name: String; var fChm: TChmReader): QWord;
    //properties
    property Chm[Index: Integer]: TChmReader read GetChm;
    property FileName[Index: Integer]: String read GetFileName;
    property OnOpenNewFile: TChmFileOpenEvent read fOnOpenNewFile write SetOnOpenNewFile;
  end;

//ErrorCodes
const
  ERR_NO_ERR = 0;
  ERR_STREAM_NOT_ASSIGNED = 1;
  ERR_NOT_SUPPORTED_VERSION = 2;
  ERR_NOT_VALID_FILE = 3;
  ERR_UNKNOWN_ERROR = 10;

  function ChmErrorToStr(Error: Integer): String;

implementation
uses ChmTypes;

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

function ChunkType(Stream: TMemoryStream): TDirChunkType;
var
  ChunkID: array[0..3] of char;
begin
  Result := ctUnknown;
  if Stream.Size< 4 then exit;
  Move(Stream.Memory^, ChunkId[0], 4);
  if ChunkID = 'PMGL' then Result := ctPMGL
  else if ChunkID = 'PMGI' then Result := ctPMGI
  else if ChunkID = 'AOLL' then Result := ctAOLL
  else if ChunkID = 'AOLI' then Result := ctAOLI;
end;

{ TITSFReader }

procedure TITSFReader.ReadHeader;
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
  Writeln('DirCount  =  ', fDirectoryHeader.DirectoryChunkCount);
  {$ENDIF}
  fDirectoryEntriesStartPos := fStream.Position;
  fDirectoryHeaderLength := LEtoN(fHeaderEntries[1].Length);
end;

procedure TChmReader.ReadCommonData;
   // A little helper proc to make reading a null terminated string easier
   function ReadString(const Stream: TStream; StartPos: DWord; FixURL: Boolean): String;
   var
     buf: array[0..49] of char;
   begin
     Result := '';
     Stream.Position := StartPos;
     repeat
       Stream.Read(buf, 50);
       Result := Result + buf;
     until IndexByte(buf, 50, 0) <> -1;
     if FixURL then
       Result := StringReplace(Result, '\', '/', [rfReplaceAll]);
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
     fWindows.Free;
     fStrings.Free;
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
     fIVB := GetObject('/#IVB');
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
       Str := '/'+ ReadString(fStrings, Offset, True);
       fContextList.AddContext(Value, Str);
     end;
     fIVB.Free;
     fStrings.Free;
   end;
begin
   ReadFromSystem;
   ReadFromWindows;
   ReadContextIds;
   {$IFDEF CHM_DEBUG}
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

function TChmReader.ReadStringsEntryFromStream ( strm:TStream ) : String;
var APosition : DWord;
begin
  APosition:=LEtoN(strm.ReadDWord);
  result:=ReadStringsEntry(APosition);
end;

function TChmReader.ReadURLSTR ( APosition: DWord ) : String;
begin
  result:='';
  if not CheckCommonStreams then
    Exit;

  fURLTBLStream.Position := APosition;
  fURLTBLStream.ReadDWord; // unknown
  fURLTBLStream.ReadDWord; // TOPIC index #
  fURLSTRStream.Position := LEtoN(fURLTBLStream.ReadDWord);
  fURLSTRStream.ReadDWord; // URL
  fURLSTRStream.ReadDWord; // FrameName
  if fURLSTRStream.Position < fURLSTRStream.Size-1 then
    Result := PChar(fURLSTRStream.Memory+fURLSTRStream.Position);
end;

function TChmReader.CheckCommonStreams: Boolean;
begin
  if fTOPICSStream = nil then
    fTOPICSStream := GetObject('/#TOPICS');
  if fURLSTRStream = nil then
    fURLSTRStream := GetObject('/#URLSTR');
  if fURLTBLStream = nil then
    fURLTBLStream := GetObject('/#URLTBL');

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
end;

constructor TChmReader.Create(AStream: TStream; FreeStreamOnDestroy: Boolean);
begin
  fContextList := TContextList.Create;
  fWindowslist      := TObjectlist.Create(True);
  fDefaultWindow:='';

  inherited Create(AStream, FreeStreamOnDestroy);
  if not IsValidFile then exit;

  ReadCommonData;
end;

destructor TChmReader.Destroy;
begin
  FreeAndNil(fContextList);
  FreeAndNil(FWindowslist);
  FreeAndNil(FSearchReader);
  FreeAndNil(fTOPICSStream);
  FreeAndNil(fURLSTRStream);
  FreeAndNil(fURLTBLStream);
  FreeAndNil(fStringsStream);
  inherited Destroy;
end;

function TITSFReader.GetChunkType(Stream: TMemoryStream; ChunkIndex: LongInt): TDirChunkType;
var
  Sig: array[0..3] of char;
begin
  Result := ctUnknown;
  Stream.Position := fDirectoryEntriesStartPos + (fDirectoryHeader.ChunkSize * ChunkIndex);

  Stream.Read(Sig, 4);
  if Sig = 'PMGL' then Result := ctPMGL
  else if Sig = 'PMGI' then Result := ctPMGI
  else if Sig = 'AOLL' then Result := ctAOLL
  else if Sig = 'AOLI' then Result := ctAOLI;
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

  PMGIEntry.ListingChunk := GetCompressedInteger(Stream);
  if NameLength = 0 then Exit; // failed GetCompressedInteger sanity check
  Result := True;
end;

constructor TITSFReader.Create(AStream: TStream; FreeStreamOnDestroy: Boolean);
begin
  fStream := AStream;
  fStream.Position := 0;
  fFreeStreamOnDestroy := FreeStreamOnDestroy;
  ReadHeader;
  if not IsValidFile then Exit;
end;

destructor TITSFReader.Destroy;
begin
  if fFreeStreamOnDestroy then FreeAndNil(fStream);

  inherited Destroy;
end;

function TITSFReader.IsValidFile: Boolean;
begin
  if (fStream = nil) then ChmLastError := ERR_STREAM_NOT_ASSIGNED
  else if (fITSFHeader.ITSFsig <> 'ITSF') then ChmLastError := ERR_NOT_VALID_FILE
  //else if (fITSFHeader.Version <> 2) and (fITSFHeader.Version <> 3)
  else if not (fITSFHeader.Version in [2..4])
  then
    ChmLastError := ERR_NOT_SUPPORTED_VERSION;
  Result := ChmLastError = ERR_NO_ERR;
end;

procedure TITSFReader.GetCompleteFileList(ForEach: TFileEntryForEach; AIncludeInternalFiles: Boolean = True);
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
  if not assigned(ForEach) then Exit;
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
         if  (Length(Entry.Name) = 1)
         or (AIncludeInternalFiles
              or
             ((Length(Entry.Name) > 1) and (not(Entry.Name[2] in ['#','$',':']))))
         then
          ForEach(Entry.Name, Entry.ContentOffset, Entry.DecompressedLength, Entry.ContentSection);
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
    Posn := ChunkStream.Size-SizeOf(Word);
    ChunkStream.Position := Posn;

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
    if NameLength>0 then
      ChunkStream.Read(Result[1], NameLength);
  end;
var
  PMGLChunk: TPMGListChunk;
  PMGIChunk: TPMGIIndexChunk;
  //ChunkStream: TMemoryStream; declared above
  Entry: TPMGListChunkEntry;
  NextIndex: Integer;
  EntryName: String;
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

  NextIndex := fDirectoryHeader.IndexOfRootChunk;
  if NextIndex < 0 then NextIndex := 0; // no PMGI chunks

  while NextIndex > -1  do begin
    GetDirectoryChunk(NextIndex, ChunkStream);
    NextIndex := -1;
    ReadQuickRefSection;
    {$IFDEF CHM_DEBUG}
    WriteLn('In Block ', NextIndex);
    {$endif}
    case ChunkType(ChunkStream) of
      ctUnknown: // something is wrong
        begin
          {$IFDEF CHM_DEBUG}WriteLn(NextIndex, ' << Unknown BlockType!');{$ENDIF}
          Break;
        end;
      ctPMGI: // we must follow the PMGI tree until we reach a PMGL block
        begin
          LookupPMGIchunk(ChunkStream, PMGIChunk);

          //QuickRefIndex[0] := ChunkStream.Position;

          I := 0;
          while ChunkStream.Position <= ChunkStream.Size - PMGIChunk.UnusedSpace do begin;
            EntryName := ReadString;
            if EntryName = '' then break;
            if ChunkStream.Position >= ChunkStream.Size - PMGIChunk.UnusedSpace then break;
            CRes := ChmCompareText(Name, EntryName);
            if CRes = 0 then begin
              // no more need of this block. onto the next!
              NextIndex := GetCompressedInteger(ChunkStream);
              Break;
            end;
            if  CRes < 0 then begin
              if I = 0 then Break; // File doesn't exist
              // file is in previous entry
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
            // we consume the entry by reading it
            Entry.Name := ReadString;
            if Entry.Name = '' then break;
            if ChunkStream.Position >= ChunkStream.Size - PMGLChunk.UnusedSpace then break;

            Entry.ContentSection := GetCompressedInteger(ChunkStream);
            Entry.ContentOffset := GetCompressedInteger(ChunkStream);
            Entry.DecompressedLength := GetCompressedInteger(ChunkStream);

            CRes := ChmCompareText(Name, Entry.Name);
            if CRes = 0 then begin
              fCachedEntry := Entry;
              Result := Entry.DecompressedLength;
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
    FmtStr(SectionName, '::DataSpace/Storage/%s/',[SectionNames[Entry.ContentSection]]);
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
  if not CheckCommonStreams then
    Exit;
  fTOPICSStream.Position := ATopicID * 16;
  if fTOPICSStream.Position = ATopicID * 16 then
  begin
    fTOPICSStream.ReadDWord;
    TopicTitleOffset := LEtoN(fTOPICSStream.ReadDWord);
    TopicURLTBLOffset := LEtoN(fTOPICSStream.ReadDWord);
    {$ifdef binindex}
    {$ifndef nonumber}
    writeln('titleid:',TopicTitleOffset);
    writeln('urlid  :',TopicURLTBLOffset);
    {$endif}
    {$endif}
    if TopicTitleOffset <> $FFFFFFFF then
      ATitle := ReadStringsEntry(TopicTitleOffset);
     //WriteLn('Got a title: ', ATitle);
    Result := ReadURLSTR(TopicURLTBLOffset);
  end;
end;

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

  pw:=pword(@oldhead[n]);
  if (n>1) and (pw[-1]=0) then
    dec(n,2); // remove trailing #0
  setlength(ws,n div sizeof(widechar));
  move(oldhead^,ws[1],n);
  for n:=1 to length(ws) do
    word(ws[n]):=LEToN(word(ws[n]));
  readv:=ws; // force conversion for now, and hope it doesn't require cwstring
end;


Type TLookupRec = record
                   item : TChmSiteMapItems;
                   depth : integer;
                   end;
     TLookupDict = TDictionary<string,TLookupRec>;
function TChmReader.GetIndexSitemap(ForceXML:boolean=false): TChmSiteMap;
var Index   : TMemoryStream;


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

var
   parentitem:TChmSiteMapItems;
   itemstack :TObjectList;
   lookup  : TLookupDict;
   curitemdepth : integer;
   sitemap : TChmSiteMap;

function getitem(anentrydepth:integer):Tchmsitemapitems;
begin
   if anentrydepth<itemstack.count then
     result:=tchmsitemapitems(itemstack[anentrydepth])
   else
     begin
       {$ifdef binindex}
         writeln('pop from emptystack at ',anentrydepth,' ',itemstack.count);
       {$endif}
       result:=tchmsitemapitems(itemstack[itemstack.Count-1]);
     end;
end;

procedure pushitem(anentrydepth:integer;anitem:tchmsitemapitem);
begin

 if anentrydepth<itemstack.count then
   itemstack[anentrydepth]:=anitem.children
 else
   if anentrydepth=itemstack.count then
     itemstack.add(anitem.Children)
   else
     begin
       {$ifdef binindex}
         writeln('push more than 1 larger ' ,anentrydepth,' ',itemstack.count);
       {$endif}
       itemstack.add(anitem.Children)
     end;
end;
procedure parselistingblock(p:pbyte);
var

    Item    : TChmSiteMapItem;

    hdr:PBTreeBlockHeader;
    head,tail : pbyte;
    isseealso,
    entrydepth,
    nrpairs : Integer;
    i : integer;
    PE : PBtreeBlockEntry;
    title : string;
    CharIndex,
    ind:integer;
    seealsostr,
    s,
    Name : AnsiString;
    path,
    shortname : AnsiString;
    anitem:TChmSiteMapItems;
    litem : TChmSiteMapItem;
    lookupitem : TLookupRec;

function readvalue:string;
begin
  result:='';
  title:='';
  if head<tail Then
    begin
      ind:=LEToN(plongint(head)^);

      result:=lookuptopicbyid(ind,title);
      {$ifdef binindex}
        writeln(i:3,' topic: ' {$ifndef nonumber},'  (',ind,')' {$endif});
        writeln('    title: ',title);
        writeln('    result: ',result);
      {$endif}
      inc(head,4);
    end;
end;

procedure dumpstack;
var fp : TChmSiteMapItems;
     ix : Integer;
begin
  for ix:=0 to itemstack.Count-1 do
    begin
      fp :=TChmSiteMapItems(itemstack[ix]);
      writeln(ix:3,' ',fp.parentname);
    end;
end;

begin
  //setlength (curitem,10);
  hdr:=PBTreeBlockHeader(p);
  hdr^.Length          :=LEToN(hdr^.Length);
  hdr^.NumberOfEntries :=LEToN(hdr^.NumberOfEntries);
  hdr^.IndexOfPrevBlock:=LEToN(hdr^.IndexOfPrevBlock);
  hdr^.IndexOfNextBlock:=LEToN(hdr^.IndexOfNextBlock);

  {$ifdef binindex}
  writeln('hdr:',hdr^.length);
  {$endif}
  tail:=p+(2048-hdr^.length);
  head:=p+sizeof(TBtreeBlockHeader);

  {$ifdef binindex}
  {$ifndef nonumber}
  writeln('previndex  : ',hdr^.IndexOfPrevBlock);
  writeln('nextindex  : ',hdr^.IndexOfNextBlock);
  {$endif}
  {$endif}
  while head<tail do
    begin
      //writeln(tail-head);
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
      EntryDepth:=LEToN(PE^.entrydepth);
      CharIndex:=LEToN(PE^.CharIndex);
      Path:='';

      if charindex<>0 then
        begin
          Path:=Trim(Copy(Name,1,charindex-2));
          Shortname:=trim(copy(Name,charindex,Length(Name)-Charindex+1));
        end
      else
        shortname:=name;
      {$ifdef binindex}
      writeln('depth:', curitemdepth, ' ' ,entrydepth);
      {$endif}
      if curitemdepth=entrydepth then // same level, so of same parent
         begin
           item:=parentitem.newitem;
           pushitem(entrydepth+1,item);
         end
      else
        if curitemdepth=entrydepth-1 then // new child, one lower.
          begin
            parentitem:=getitem(entrydepth);
            item:=parentitem.newitem;
            pushitem(entrydepth+1,item);
          end
        else
         if entrydepth<curitemdepth then
          begin
            parentitem:=getitem(entrydepth);
            {$ifdef binindex}
            writeln('bingo!', parentitem.parentname);
            dumpstack;
            {$endif}
            item:=parentitem.newitem;
            pushitem(entrydepth+1,item);
          end;

      curitemdepth:=entrydepth;
      {$ifdef binindex}
      writeln('lookup:', Name, ' = ', path,' = ',shortname);
      {$endif}

    (*  if lookup.trygetvalue(path,lookupitem) then
        begin
//          if lookupitem.item<>parentitem then
//             writeln('mismatch: ',lookupitem.item.item[0].name,' ',name);
{          if curitemdepth<entrydepth then
            begin
              writeln('lookup ok!',curitemdepth,' ' ,entrydepth);
              curitemdepth:=entrydepth;
            end
          else
           begin
             writeln('lookup odd!',curitemdepth,' ' ,entrydepth);
           end;
          curitemdepth:=lookupitem.depth+1;
          parentitem:=lookupitem.item;}
        end
      else
        begin
 //            parentitem:=sitemap.Items;
          if not curitemdepth=entrydepth then
             writeln('no lookup odd!',curitemdepth,' ' ,entrydepth);
        end;  *)
{      item:=parentitem.newitem;}
      lookupitem.item:=item.children;
      lookupitem.depth:=entrydepth;
      lookup.addorsetvalue(name,lookupitem);
      item.AddName(Shortname);

      {$ifdef binindex}
        Writeln('seealso   :  ',IsSeeAlso);
        Writeln('entrydepth:  ',EntryDepth);
        Writeln('charindex :  ',charindex );
        Writeln('Nrpairs   :  ',NrPairs);
        Writeln('CharIndex :  ',charindex);
      {$endif}

      inc(head,sizeof(TBtreeBlockEntry));
      if isseealso>0 then
        begin
          if not ReadWCharString(Head,Tail,SeeAlsoStr) Then
            Break;
          // have to figure out first what to do with it.
          // is See Also really mutually exclusive with pairs?
          // or is the number of pairs equal to the number of seealso
          // strings?
          {$ifdef binindex}
            writeln('seealso: ',seealsostr);
          {$endif}
          item.AddSeeAlso(seealsostr);
        end
      else
        begin
         if NrPairs>0 Then
          begin
            {$ifdef binindex}
             writeln('Pairs   : ');
            {$endif}

            for i:=0 to nrpairs-1 do
              begin
               s:=readvalue;
             //  if not ((i=0) and (title=shortname)) then
               item.addname(title);
               item.addlocal(s);
              end;
          end;
         end;
      inc(head,4); // always 1
      {$ifdef binindex}
        if head<tail then
        writeln('Zero based index (13 higher than last) :',plongint(head)^);
      {$endif}
      inc(head,4); // zero based index (13 higher than last
    end;
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
     Result:=AbortAndTryTextual; // frees index if needed
     Exit;
   end;
   if not CheckCommonStreams then
   begin
     index.free;
     Result:=AbortAndTryTextual; // frees index if needed
     Exit;
   end;

   lookup:=TDictionary<string,TLookupRec>.create;
   SiteMap:=TChmSitemap.Create(StIndex);
   itemstack :=TObjectList.create(false);
   //Item   :=Nil;  // cached last created item, in case we need to make
                  // a child.
   parentitem:=sitemap.Items;
   itemstack.add(parentitem); // level 0
   curitemdepth:=0;
   TryTextual:=True;
   BHdr.LastLstBlock:=0;
   if LoadBtreeHeader(index,BHdr) and (BHdr.LastLstBlock>=0) Then
    begin
       if BHdr.BlockSize=defblocksize then
         begin
           for i:=0 to BHdr.lastlstblock do
             begin
               if (index.size-index.position)>=defblocksize then // skips last incomplete block?
                 begin
                   Index.read(block,defblocksize);
                   parselistingblock(@block)
                end;
             end;
            trytextual:=false;
            result:=sitemap;
          end;
    end;
  if trytextual then
    begin
      sitemap.free;
      Result:=AbortAndTryTextual; // frees index if needed
    end
  else Index.Free;
  itemstack.free;
  lookup.free;
end;

function TChmReader.GetTOCSitemap(ForceXML:boolean=false): TChmSiteMap;
    function AddTOCItem(TOC: TStream; AItemOffset: DWord; SiteMapITems: TChmSiteMapItems): DWord;
    var
      Props: DWord;
      Item: TChmSiteMapItem;
      NextEntry: DWord;
      TopicsIndex: DWord;
      Title, Local : String;
    begin
      Toc.Position:= AItemOffset + 4;
      Item := SiteMapITems.NewItem;
      Props := LEtoN(TOC.ReadDWord);
      if (Props and TOC_ENTRY_HAS_LOCAL) = 0 then
        Item.AddName(ReadStringsEntry(LEtoN(TOC.ReadDWord)))
      else
      begin
        TopicsIndex := LEtoN(TOC.ReadDWord);
        Local:=LookupTopicByID(TopicsIndex, Title);
        Item.AddName(Title);
        Item.AddLocal(Local);
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

     // Binary Toc Exists
   Result := TChmSiteMap.Create(stTOC);

   EntryInfoOffset := NtoLE(TOC.ReadDWord);
   EntriesOffset   := NtoLE(TOC.ReadDWord);
   EntryCount      := NtoLE(TOC.ReadDWord);
   TOPICSOffset    := NtoLE(TOC.ReadDWord);

   if EntryCount = 0 then
     begin
       Toc.Free;
       Exit;
     end;

   NextItem := EntryInfoOffset;
   repeat
     NextItem := AddTOCItem(Toc, NextItem, Result.Items);
   until NextItem = 0;
   TOC.Free;
end;

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
    if FirstBlock and 1 = 1 then begin
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

      // if the next block is an even numbered block we have to reset the decompressor state
      if (X < LastBlock) and (X and 1 = 1) then LZXreset(LZXState);

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
  AStream := TFileStream.Create(AFileName, fmOpenRead or fmShareDenyWrite);
  AChm := TChmReader.Create(AStream, True);
  AIndex := AddObject(AFileName, AChm);
  fLastChm := AChm;
  if Assigned(fOnOpenNewFile) then fOnOpenNewFile(Self, AIndex)
  else fUnNotifiedFiles.Add(AChm);
end;

function TChmFileList.CheckOpenFile(AFileName: String): Boolean;
var
  X: Integer;

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
    if FileExists(AFileName) and (LowerCase(ExtractFileExt(AFileName)) = '.chm') then OpenNewFile(AFileName);
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
  if not assigned(AValue)  then exit;
  for X := 0 to fUnNotifiedFiles.Count-1 do
    AValue(Self, X);
  fUnNotifiedFiles.Clear;
end;

function TChmFileList.ObjectExists(Name: String; var fChm: TChmReader): QWord;
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

