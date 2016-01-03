{ Copyright (C) <2005> <Andrew Haines> chmtypes.pas

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
unit chmtypes;

{$mode objfpc}{$H+}

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Classes, SysUtils,xmlcfg;

type
  TSectionName = (snMSCompressed, snUnCompressed);

  TSectionNames = set of TSectionName;

=======
=======
>>>>>>> origin/fixes_2_2
  Classes, SysUtils; 
  
=======
  Classes, SysUtils;

>>>>>>> origin/fixes_2.4
type
  TSectionName = (snMSCompressed, snUnCompressed);

  TSectionNames = set of TSectionName;
<<<<<<< HEAD
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
   { TDirectoryChunk }

  TDirectoryChunk = class(TObject)
  private
    FHeaderSize: Integer;
    FQuickRefEntries: Word;
    Buffer: array[0..$1000-1] of byte;
    CurrentPos: Integer;
    FItemCount: Word;
    FClearCount: Integer;
  public
    function CanHold(ASize: Integer): Boolean;
    function FreeSpace: Integer;
    procedure WriteHeader(AHeader: Pointer);
    procedure WriteEntry(Size: Integer; Data: Pointer);
    procedure WriteChunkToStream(Stream: TStream); overload;
    procedure Clear;
    property ItemCount: Word read FItemCount;
    constructor Create(AHeaderSize: Integer);
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  { TPMGIDirectoryChunk }

  TPMGIDirectoryChunk = class(TDirectoryChunk)
  private
    FChunkLevelCount: Integer;
    FParentChunk: TPMGIDirectoryChunk;
  public
    procedure WriteChunkToStream(Stream: TStream; var AIndex: Integer; Final: Boolean = False); overload;
    property ParentChunk: TPMGIDirectoryChunk read FParentChunk write FParentChunk;
    property ChunkLevelCount: Integer read FChunkLevelCount write FChunkLevelCount;
  end;

  PFileEntryRec = ^TFileEntryRec;
  TFileEntryRec = record
    Path: String;
    Name: String;
    DecompressedOffset: QWord;
    DecompressedSize: QWord;
    Compressed: Boolean; // True means it goes in section1 False means section0
  end;

  { TFileEntryList }

  TFileEntryList = class(TList)
  private
    FPaths: TStringList;
    function GetFileEntry(Index: Integer): TFileEntryRec;
    procedure SetFileEntry(Index: Integer; const AValue: TFileEntryRec);
  public
    function AddEntry(AFileEntry: TFileEntryRec; CheckPathIsAdded: Boolean = True): Integer;
    procedure Delete(Index: Integer);
    property FileEntry[Index: Integer]: TFileEntryRec read GetFileEntry write SetFileEntry;
    procedure Sort;
    constructor Create;
    destructor Destroy; override;

  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TValidWindowFieldsEnum = (valid_Unknown1 {:=1},
                            valid_Navigation_pane_style {:= 2},
                            valid_Window_style_flags {:= 4},
                            valid_Window_extended_style_flags {:= 8},
                            valid_Initial_window_position    {:= $10},
                            valid_Navigation_pane_width {:= $20},
                            valid_Window_show_state {:= $40},
                            valid_Info_types {:= $80},
                            valid_Buttons {:= $100},
                            valid_Navigation_Pane_initially_closed_state {:= $200},
                            valid_Tab_position {:= $400},
                            valid_Tab_order {:= $800},
                            valid_History_count{ := $1000},
                            valid_Default_Pane {:= $2000});

  TValidWindowFields     = Set Of TValidWindowFieldsEnum;
  TCHMWindow = Class
                window_type,
                Title_bar_text,
                Toc_file,
                index_file,
                Default_File,
                Home_button_file,
                Jumpbutton_1_File,
                Jumpbutton_1_Text,
                Jumpbutton_2_File,
                Jumpbutton_2_Text : string;
                nav_style    : integer;  // overlay with bitfields (next 2 also)
                navpanewidth : integer;
                buttons      : integer;
                left,
                top,
                right,
                bottom       : integer;
                styleflags   ,
                xtdstyleflags,
                window_show_state,
                navpane_initially_closed,
                navpane_default,
                navpane_location,
                wm_notify_id : integer;
                flags : TValidWindowFields; // bitset that keeps track of which fields are filled.
                                            // of certain fields. Needs to be inserted into #windows stream
                Constructor create(s:string='');
                procedure load_from_ini(txt:string);
                procedure savetoxml(cfg:TXMLConfig;key:string);
                procedure loadfromxml(cfg:TXMLConfig;key:string);
                procedure assign(obj : TCHMWindow);
                end;


=======
>>>>>>> origin/fixes_2.4
  TTOCIdxHeader = record
    BlockSize: DWord; // 4096
    EntriesOffset: DWord;
    EntriesCount: DWord;
    TopicsOffset: DWord;
    EmptyBytes: array[0..4079] of byte;
  end;

const
  TOC_ENTRY_HAS_NEW      = 2;
  TOC_ENTRY_HAS_CHILDREN = 4;
  TOC_ENTRY_HAS_LOCAL    = 8;

type
  PTOCEntryPageBookInfo = ^TTOCEntryPageBookInfo;
  TTOCEntryPageBookInfo = record
    Unknown1: Word; //  = 0
    EntryIndex: Word; // multiple entry info's can have this value but the TTocEntry it points to points back to the first item with this number. Wierd.
    Props: DWord; // BitField. See TOC_ENTRY_*
    TopicsIndexOrStringsOffset: DWord; // if TOC_ENTRY_HAS_LOCAL is in props it's the Topics Index
                                       // else it's the Offset In Strings of the Item Text
    ParentPageBookInfoOffset: DWord;
    NextPageBookOffset: DWord; // same level of tree only

    // Only if TOC_ENTRY_HAS_CHILDREN is set are these written
    FirstChildOffset: DWord;
    Unknown3: DWord; // = 0
  end;

  TTocEntry = record
    PageBookInfoOffset: DWord;
    IncrementedInt: DWord; // first is $29A
    TopicsIndex: DWord; // Index of Entry in #TOPICS file
  end;

  TTopicEntry = record
    TocOffset,
    StringsOffset,
    URLTableOffset: DWord;
    InContents: Word;// 2 = in contents 6 = not in contents
    Unknown: Word; // 0,2,4,8,10,12,16,32
  end;

  TBtreeHeader = packed record
                        ident          : array[0..1] of ansichar; // $3B $29
                        flags          : word;	// bit $2 is always 1, bit $0400 1 if dir? (always on)
                        blocksize      : word;  // size of blocks (2048)
                        dataformat     : array[0..15] of ansichar;  // "X44" always the same, see specs.
                        unknown0       : dword; // always 0
			lastlstblock   : dword; // index of last listing block in the file;
                        indexrootblock : dword; // Index of the root block in the file.
                        unknown1       : dword; // always -1
                        nrblock	       : dword; // Number of blocks
                        treedepth      : word;  // The depth of the tree of blocks (1 if no index blocks, 2 one level of index blocks, ...)
                        nrkeywords     : dword; // number of keywords in the file.
                        codepage       : dword; // Windows code page identifier (usually 1252 - Windows 3.1 US (ANSI))
			lcid	       : dword; // LCID from the HHP file.
                        ischm	       : dword; // 0 if this a BTREE and is part of a CHW file, 1 if it is a BTree and is part of a CHI or CHM file
                        unknown2       : dword; // Unknown. Almost always 10031. Also 66631 (accessib.chm, ieeula.chm, iesupp.chm, iexplore.chm, msoe.chm, mstask.chm, ratings.chm, wab.chm).
                        unknown3       : dword; // unknown 0
		        unknown4       : dword; // unknown 0
			unknown5       : dword; // unknown 0
                      end;
  PBTreeBlockHeader = ^TBtreeBlockHeader;
  TBtreeBlockHeader = packed record
                        Length             : word;  // Length of free space at the end of the block.
                        NumberOfEntries    : word;  // Number of entries in the block.
                        IndexOfPrevBlock   : dword; // Index of the previous block. -1 if this is the first listing block.
                        IndexOfNextBlock   : dword; // Index of the next block. -1 if this is the last listing block.
                      end;

  PBtreeBlockEntry = ^TBtreeBlockEntry;
  TBtreeBlockEntry = packed record
                        isseealso  : word; // 2 if this keyword is a See Also keyword, 0 if it is not.
                        entrydepth : word; // Depth of this entry into the tree.
                        charindex  : dword;// Character index of the last keyword in the ", " separated list.
                        unknown0   : dword;// 0 (unknown)
                        NrPairs    : dword;// Number of Name, Local pairs
                      end;

  PBtreeIndexBlockHeader = ^TBtreeIndexBlockHeader;
  TBtreeIndexBlockHeader = packed record
                        length             : word;  // Length of free space at the end of the block.
                        NumberOfEntries    : word;  // Number of entries in the block.
                        IndexOfChildBlock  : dword; // Index of Child Block
                      end;

  PBtreeIndexBlockEntry = ^TBtreeIndexBlockEntry;
  TBtreeIndexBlockEntry = packed record
                        isseealso  : word; // 2 if this keyword is a See Also keyword, 0 if it is not.
                        entrydepth : word; // Depth of this entry into the tree.
                        charindex  : dword;// Character index of the last keyword in the ", " separated list.
                        unknown0   : dword;// 0 (unknown)
                        NrPairs    : dword;// Number of Name, Local pairs
                      end;

function PageBookInfoRecordSize(ARecord: PTOCEntryPageBookInfo): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

Const defvalidflags = [valid_Navigation_pane_style,valid_Window_style_flags,valid_Initial_window_position,valid_Navigation_pane_width,valid_Buttons,valid_Tab_position];
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4

implementation
uses chmbase;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
function PageBookInfoRecordSize(ARecord: PTOCEntryPageBookInfo): Integer;
begin
  if (TOC_ENTRY_HAS_CHILDREN and ARecord^.Props) > 0 then
    Result := 28
  else
    Result := 20;
end;

<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{ TDirectoryChunk }

function TDirectoryChunk.CanHold(ASize: Integer): Boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result := CurrentPos < $1000 - ASize - (SizeOf(Word) * (FQuickRefEntries+2));
=======
  Result := CurrentPos < $1000-1 - ASize - (SizeOf(Word) * (FQuickRefEntries+2));
>>>>>>> graemeg/fixes_2_2
=======
  Result := CurrentPos < $1000-1 - ASize - (SizeOf(Word) * (FQuickRefEntries+2));
>>>>>>> origin/fixes_2_2
end;

function TDirectoryChunk.FreeSpace: Integer;
begin
  Result := $1000 - CurrentPos;
end;

procedure TDirectoryChunk.WriteHeader(AHeader: Pointer);
begin
  Move(AHeader^, Buffer[0], FHeaderSize);
end;

procedure TDirectoryChunk.WriteEntry(Size: Integer; Data: Pointer);
var
  ReversePos: Integer;
  Value: Word;
begin
  if not CanHold(Size) then Raise Exception.Create('Trying to write past the end of the buffer');
  Move(Data^, Buffer[CurrentPos], Size);
  Inc(CurrentPos, Size);
  Inc(FItemCount);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4

  // now put a quickref entry if needed
  if ItemCount mod 5 = 0 then begin
    Inc(FQuickRefEntries);
    ReversePos := ($1000) - SizeOf(Word) - (SizeOf(Word)*FQuickRefEntries);
    Value := NtoLE(Word(CurrentPos - Size - FHeaderSize));
=======
=======
>>>>>>> origin/fixes_2_2
  
  // now put a quickref entry if needed
  if ItemCount mod 5 = 0 then begin
    Inc(FQuickRefEntries);
    ReversePos := ($1000-1) - SizeOf(Word) - (SizeOf(Word)*FQuickRefEntries);
    Value := NtoLE(Word(CurrentPos - Size));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Move(Value, Buffer[ReversePos], SizeOf(Word));
  end;
end;

procedure TDirectoryChunk.WriteChunkToStream(Stream: TStream);
var
  ReversePos: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
  TmpItemCount: Word;
begin
  ReversePos := $1000 - SizeOf(Word);
  TmpItemCount := NtoLE(Word(FItemCount));
  Move(TmpItemCount, Buffer[ReversePos], SizeOf(Word));
=======
=======
>>>>>>> origin/fixes_2_2
begin
  ReversePos := $1000-1 - SizeOf(Word);
  FItemCount := NtoLE(ItemCount);
  Move(ItemCount, Buffer[ReversePos], SizeOf(Word));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  Stream.Write(Buffer[0], $1000);
  {$IFDEF DEBUG_CHM_CHUNKS}
  WriteLn('Writing ', Copy(PChar(@Buffer[0]),0,4),' ChunkToStream');
  {$ENDIF}
end;

procedure TDirectoryChunk.Clear;
begin
  FillChar(Buffer, $1000, 0);
  FItemCount := 0;
  CurrentPos := FHeaderSize;
  FQuickRefEntries := 0;
  Inc(FClearCount);
end;

constructor TDirectoryChunk.Create(AHeaderSize: Integer);
begin
  FHeaderSize := AHeaderSize;
  CurrentPos := FHeaderSize;
end;

{ TFileEntryList }

function TFileEntryList.GetFileEntry(Index: Integer): TFileEntryRec;
begin
  Result := PFileEntryRec(Items[Index])^;
end;

procedure TFileEntryList.SetFileEntry(Index: Integer; const AValue: TFileEntryRec);
begin
  PFileEntryRec(Items[Index])^ := AValue;
end;

function TFileEntryList.AddEntry(AFileEntry: TFileEntryRec; CheckPathIsAdded: Boolean = True): Integer;
var
  TmpEntry: PFileEntryRec;
begin
  New(TmpEntry);
  //WriteLn('Adding: ', AFileEntry.Path+AFileEntry.Name,' Size = ', AFileEntry.DecompressedSize,' Offset = ', AFileEntry.DecompressedOffset);
  if CheckPathIsAdded and (FPaths.IndexOf(AFileEntry.Path) < 0) then begin
    // all paths are included in the list of files in section 0 with a size and offset of 0
    FPaths.Add(AFileEntry.Path);
    TmpEntry^.Path := AFileEntry.Path;
    TmpEntry^.Name := '';
    TmpEntry^.DecompressedOffset := 0;
    TmpEntry^.DecompressedSize := 0;
    TmpEntry^.Compressed := False;
    (Self as TList).Add(TmpEntry);
    New(TmpEntry);
  end;
  TmpEntry^ := AFileEntry;
  Result := (Self as TList).Add(TmpEntry);
end;

procedure TFileEntryList.Delete(Index: Integer);
begin
  Dispose(PFileEntryRec(Items[Index]));
  Inherited Delete(Index);
end;

function FileEntrySortFunc(Item1, Item2: PFileEntryRec): Integer;
var
  Str1, Str2: String;
begin
  Str1 := Item1^.Path + Item1^.Name;
  Str2 := Item2^.Path + Item2^.Name;
  Result := ChmCompareText(Str1, Str2);
end;

procedure TFileEntryList.Sort;
begin
  Inherited Sort(TListSortCompare(@FileEntrySortFunc));
end;

constructor TFileEntryList.Create;
begin
  Inherited Create;
  FPaths := TStringList.Create;
end;

destructor TFileEntryList.Destroy;
var
  I: Integer;
begin
  for I := Count-1 downto 0 do
    Delete(I);
  FPaths.Free;
  inherited Destroy;
end;

{ TPMGIDirectoryChunk }
procedure TPMGIDirectoryChunk.WriteChunkToStream(Stream: TStream; var AIndex: Integer
  ; Final: Boolean = False);
var
  NewBuffer: array[0..512] of byte;
  EntryLength,
  WriteSize: Integer;
  OldPos, NewPos, NewStart: Int64;
  procedure FinishBlock;
  var
    Header: TPMGIIndexChunk;
  begin
    Inc(AIndex);
    Header.PMGIsig := 'PMGI';
    Header.UnusedSpace := FParentChunk.FreeSpace;
    FParentChunk.WriteHeader(@Header);
    FParentChunk.WriteChunkToStream(Stream, AIndex, Final);
    FParentChunk.Clear;
  end;
begin
  if FItemCount < 1 then begin
<<<<<<< HEAD
<<<<<<< HEAD
    {$ifdef chm_debug}
    WriteLn('WHAT ARE YOU DOING!!');
    {$endif}
=======
    WriteLn('WHAT ARE YOU DOING!!');
>>>>>>> graemeg/fixes_2_2
=======
    WriteLn('WHAT ARE YOU DOING!!');
>>>>>>> origin/fixes_2_2
    Dec(AIndex);
    Exit;
  end;
  OldPos := Stream.Position;
  WriteChunkToStream(Stream);
  NewPos := Stream.Position;
  Inc(FChunkLevelCount);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  if Final and (ChunkLevelCount < 2) then begin
    FParentChunk.Free;
    FParentChunk := nil;
    Exit;
  end;
  if FParentChunk = nil then FParentChunk := TPMGIDirectoryChunk.Create(FHeaderSize);

  NewStart := OldPos+FHeaderSize;
  Stream.Position := NewStart;
  EntryLength := GetCompressedInteger(Stream);
  WriteSize := (Stream.Position - NewStart) + EntryLength;
  Move(Buffer[FHeaderSize], NewBuffer[0], WriteSize);
  Inc(WriteSize, WriteCompressedInteger(@NewBuffer[WriteSize], AIndex));

  Stream.Position := NewPos;

  if not FParentChunk.CanHold(WriteSize) then begin
    FinishBlock;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
=======

>>>>>>> origin/fixes_2.4
  FParentChunk.WriteEntry(WriteSize, @NewBuffer[0]);
  if Final then FinishBlock;
  //WriteLn(ChunkLevelCount);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function getnext(const s:string;var i: integer;len:integer):string;
var
    ind : integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

begin
 if i>len then exit('');
 ind:=i;
 if s[ind]='"' then
   begin
     inc(ind);
     while (ind<=len) and (s[ind]<>'"') do inc(ind);
     result:=copy(s,i+1,ind-i-1);
     inc(ind); // skip "
   end
 else
   begin
     while (ind<=len) and (s[ind]<>',') do inc(ind);
     result:=copy(s,i,ind-i);
   end;
 i:=ind+1; // skip ,
end;
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
=======

begin
 if i>len then exit('');
 ind:=i;
 if s[ind]='"' then
   begin
     inc(ind);
     while (ind<=len) and (s[ind]<>'"') do inc(ind);
     result:=copy(s,i+1,ind-i-1);
     inc(ind); // skip "
   end
 else
   begin
     while (ind<=len) and (s[ind]<>',') do inc(ind);
     result:=copy(s,i,ind-i);
   end;
 i:=ind+1; // skip ,
end;

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
function getnextint(const txt:string;var ind: integer;len:integer;var flags : TValidWindowFields;x:TValidWindowFieldsEnum):integer;

var s : string;
    i:integer;
begin

  i:=ind;
  s:=getnext(txt,ind,len);
  // set a flag if the field was empty (,,)
  if (ind=(i+1)) and (x<>valid_unknown1) then
    include(flags,x);
  result:=strtointdef(s,0);  // I think this does C style hex, if not fixup here.
end;

procedure TCHMWindow.load_from_ini(txt:string);
var ind,len,
    j,k     : integer;
    arr     : array[0..3] of integer;
    s2      : string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    bArr    : Boolean;
begin
=======
begin
  flags:=[];
>>>>>>> graemeg/cpstrnew
=======
begin
  flags:=[];
>>>>>>> graemeg/cpstrnew
=======
begin
  flags:=[];
>>>>>>> graemeg/cpstrnew
=======
begin
  flags:=[];
>>>>>>> origin/cpstrnew
  j:=pos('=',txt);
  if j>0 then
    txt[j]:=',';
  ind:=1; len:=length(txt);
  window_type       :=getnext(txt,ind,len);
  Title_bar_text    :=getnext(txt,ind,len);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Toc_file          :=getnext(txt,ind,len);
  index_file        :=getnext(txt,ind,len);
=======
  index_file        :=getnext(txt,ind,len);
  Toc_file          :=getnext(txt,ind,len);
>>>>>>> graemeg/cpstrnew
=======
  index_file        :=getnext(txt,ind,len);
  Toc_file          :=getnext(txt,ind,len);
>>>>>>> graemeg/cpstrnew
=======
  index_file        :=getnext(txt,ind,len);
  Toc_file          :=getnext(txt,ind,len);
>>>>>>> graemeg/cpstrnew
=======
  index_file        :=getnext(txt,ind,len);
  Toc_file          :=getnext(txt,ind,len);
>>>>>>> origin/cpstrnew
  Default_File      :=getnext(txt,ind,len);
  Home_button_file  :=getnext(txt,ind,len);
  Jumpbutton_1_File :=getnext(txt,ind,len);
  Jumpbutton_1_Text :=getnext(txt,ind,len);
  Jumpbutton_2_File :=getnext(txt,ind,len);
  Jumpbutton_2_Text :=getnext(txt,ind,len);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  nav_style         :=getnextint(txt,ind,len,flags,valid_navigation_pane_style);
  navpanewidth      :=getnextint(txt,ind,len,flags,valid_navigation_pane_width);
  buttons           :=getnextint(txt,ind,len,flags,valid_buttons);
  
  (* initialize arr[] *)
  arr[0] :=0;
  arr[1] :=0;
  arr[2] :=0;
  arr[3] :=0;
  k:=0;
  bArr   := False;
  (* "[" int,int,int,int "]", |,  *)
  s2:=getnext(txt,ind,len);
  if length(s2)>0 then begin
    (* check if first chart is "[" *)
    if (s2[1]='[') then begin
      delete(s2,1,1);
      bArr := True;
    end;
    (* looking for a max 4 int followed by a closing "]" *)
    repeat
      if k > 0 then s2:=getnext(txt,ind,len);
      
      j:=pos(']',s2);
      if j>0 then delete(s2,j,1);
      if length(trim(s2))>0 then
        include(flags,valid_tab_position);
      arr[k]:=strtointdef(s2,0);
      inc(k);
    until (bArr <> True) or (j<>0) or (ind>len);
  end;
   
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  nav_style         :=getnextint(txt,ind,len,flags,valid_navigation_pane_style);
  navpanewidth      :=getnextint(txt,ind,len,flags,valid_navigation_pane_width);
  buttons           :=getnextint(txt,ind,len,flags,valid_buttons);
  k:=0;
  repeat
   s2:=getnext(txt,ind,len);
   if (length(s2)>0) and (s2[1]='[') then delete(s2,1,1);
   j:=pos(']',s2);
   if j>0 then delete(s2,j,1);
   if length(trim(s2))>0 then
     include(flags,valid_tab_position);
   arr[k]:=strtointdef(s2,0);
   inc(k);
  until (j<>0) or (ind>len);
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
  left  :=arr[0];
  top   :=arr[1];
  right :=arr[2];
  bottom:=arr[3];
  styleflags               :=getnextint(txt,ind,len,flags,valid_buttons);
  xtdstyleflags            :=getnextint(txt,ind,len,flags,valid_window_style_flags);
  window_show_state        :=getnextint(txt,ind,len,flags,valid_window_extended_style_flags);
  navpane_initially_closed :=getnextint(txt,ind,len,flags,valid_navigation_pane_initially_closed_state);
  navpane_default          :=getnextint(txt,ind,len,flags,valid_default_pane);
  navpane_location         :=getnextint(txt,ind,len,flags,valid_tab_position);
  wm_notify_id             :=getnextint(txt,ind,len,flags,valid_unknown1);
end;

procedure TCHMWindow.savetoxml(cfg:TXMLConfig;key:string);
begin
  cfg.setvalue(key+'window_type',window_type);
  cfg.setvalue(key+'title_bar_text',title_bar_text);
  cfg.setvalue(key+'toc_file',   Toc_file  );
  cfg.setvalue(key+'index_file',   index_file  );
  cfg.setvalue(key+'default_file',   Default_File    );
  cfg.setvalue(key+'home_button_file',   Home_button_file);
  cfg.setvalue(key+'jumpbutton_1_file',   Jumpbutton_1_File     );
  cfg.setvalue(key+'jumpbutton_1_text',   Jumpbutton_1_Text     );
  cfg.setvalue(key+'jumpbutton_2_file',   Jumpbutton_2_File   );
  cfg.setvalue(key+'jumpbutton_2_text',   Jumpbutton_2_Text     );
  cfg.setvalue(key+'nav_style',   nav_style );
  cfg.setvalue(key+'navpanewidth',   navpanewidth    );
  cfg.setvalue(key+'buttons',   buttons   );
  cfg.setvalue(key+'left',   left);
  cfg.setvalue(key+'top',   top );
  cfg.setvalue(key+'right',   right     );
  cfg.setvalue(key+'bottom',   bottom    );
  cfg.setvalue(key+'styleflags',   styleflags);
  cfg.setvalue(key+'xtdstyleflags',   xtdstyleflags   );
  cfg.setvalue(key+'window_show_state',   window_show_state     );
  cfg.setvalue(key+'navpane_initially_closed',navpane_initially_closed  );
  cfg.setvalue(key+'navpane_default',navpane_default);
  cfg.setvalue(key+'navpane_location',navpane_location    );
  cfg.setvalue(key+'wm_notify_id',wm_notify_id  );
end;

procedure TCHMWindow.loadfromxml(cfg:TXMLConfig;key:string);

begin
  window_type           :=cfg.getvalue(key+'window_type','');
  Title_bar_text        :=cfg.getvalue(key+'title_bar_text','');
  Toc_file              :=cfg.getvalue(key+'toc_file','');
  Index_file            :=cfg.getvalue(key+'index_file','');
  Default_File          :=cfg.getvalue(key+'default_file','');
  Home_button_file      :=cfg.getvalue(key+'home_button_file','');
  Jumpbutton_1_File     :=cfg.getvalue(key+'jumpbutton_1_file','');
  Jumpbutton_1_Text     :=cfg.getvalue(key+'jumpbutton_1_text','');
  Jumpbutton_2_File     :=cfg.getvalue(key+'jumpbutton_2_file','');
  Jumpbutton_2_Text     :=cfg.getvalue(key+'jumpbutton_2_text','');
  nav_style             :=cfg.getvalue(key+'nav_style',0);
  navpanewidth          :=cfg.getvalue(key+'navpanewidth',0);
  buttons               :=cfg.getvalue(key+'buttons',0);
  left                  :=cfg.getvalue(key+'left',0);
  top                   :=cfg.getvalue(key+'top',0);
  right                 :=cfg.getvalue(key+'right',0);
  bottom                :=cfg.getvalue(key+'bottom',0);
  styleflags            :=cfg.getvalue(key+'styleflags',0);
  xtdstyleflags         :=cfg.getvalue(key+'xtdstyleflags',0);
  window_show_state     :=cfg.getvalue(key+'window_show_state',0);
  navpane_initially_closed :=cfg.getvalue(key+'navpane_initially_closed',0);
  navpane_default       :=cfg.getvalue(key+'navpane_default',0);
  navpane_location      :=cfg.getvalue(key+'navpane_location',0);
  wm_notify_id          :=cfg.getvalue(key+'wm_notify_id',0);
end;

Constructor TCHMWindow.create(s:string='');

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 flags:=defvalidflags;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 if s<>'' then
   load_from_ini(s);
end;


procedure TCHMWindow.assign(obj : TCHMWindow);

begin
  window_type      :=obj.window_type;
  Title_bar_text   :=obj.Title_bar_text;
  Toc_file         :=obj.Toc_file;
  Index_file       :=obj.Index_file;
  Default_File     :=obj.Default_File;
  Home_button_file :=obj.Home_button_file;
  Jumpbutton_1_File:=obj.Jumpbutton_1_File;
  Jumpbutton_1_Text:=obj.Jumpbutton_1_Text;
  Jumpbutton_2_File:=obj.Jumpbutton_2_File;
  Jumpbutton_2_Text:=obj.Jumpbutton_2_Text;
  nav_style        :=obj.nav_style;
  navpanewidth     :=obj.navpanewidth;
  buttons          :=obj.buttons;
  left             :=obj.left;
  top              :=obj.top;
  right            :=obj.right;
  bottom           :=obj.bottom;
  styleflags       :=obj.styleflags;
  xtdstyleflags    :=obj.xtdstyleflags;
  window_show_state:=obj.window_show_state;
  navpane_initially_closed :=obj.navpane_initially_closed;
  navpane_default  :=obj.navpane_default;
  navpane_location :=obj.navpane_location;
  wm_notify_id     :=obj.wm_notify_id;
end;

end.
=======
end.

>>>>>>> graemeg/fixes_2_2
=======
end.

>>>>>>> origin/fixes_2_2
