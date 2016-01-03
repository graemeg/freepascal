{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998 by Berczi Gabor

    Help support & Borland OA .HLP reader objects and routines

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$R-}
unit WHelp;

interface

uses
{$ifdef Windows}
   { placed here to avoid TRect to be found in windows unit
     for Windows target whereas its found in objects unit for other targets PM }
     windows,
{$endif Windows}
     Objects,
     WUtils;

const
      hscLineBreak   = #0;
      hscLink        = #2;
      hscLineStart   = #3;
      hscCode        = #5;
      hscDirect      = #6; { add the next char directly }
      hscCenter      = #10;
      hscRight       = #11;
      hscNamedMark   = #12;
      hscTextAttr    = #13;
      hscTextColor   = #14;
      hscNormText    = #15;
      hscInImage     = #16;

type
      THelpCtx = longint;

      TRecord = packed record
        SClass   : word;
        Size     : word;
        Data     : pointer;
      end;

      PIndexEntry = ^TIndexEntry;
      TIndexEntry = packed record
        Tag        : PString;
        HelpCtx    : THelpCtx;
        FileID     : word;
      end;

      PKeywordDescriptor = ^TKeywordDescriptor;
      TKeywordDescriptor = packed record
        FileID     : word;
        Context    : THelpCtx;
      end;

      PKeywordDescriptors = ^TKeywordDescriptors;
      TKeywordDescriptors = array[0..MaxBytes div sizeof(TKeywordDescriptor)-1] of TKeywordDescriptor;

      PTopic = ^TTopic;
      TTopic = object
        HelpCtx       : THelpCtx;
        FileOfs       : longint;
        TextSize      : sw_word;
        Text          : PByteArray;
        LinkCount     : sw_word;
        Links         : PKeywordDescriptors;
        LastAccess    : longint;
        FileID        : word;
        Param         : PString;
        StartNamedMark: integer;
        NamedMarks    : PUnsortedStringCollection;
        ExtData       : pointer;
        ExtDataSize   : longint;
        function LinkSize: sw_word;
        function GetNamedMarkIndex(const MarkName: string): sw_integer;
      end;

      PTopicCollection = ^TTopicCollection;
      TTopicCollection = object(TSortedCollection)
        function   At(Index: sw_Integer): PTopic;
        procedure  FreeItem(Item: Pointer); virtual;
        function   Compare(Key1, Key2: Pointer): Sw_Integer; virtual;
        function   SearchTopic(AHelpCtx: THelpCtx): PTopic;
      end;

      PIndexEntryCollection = ^TIndexEntryCollection;
      TIndexEntryCollection = object(TSortedCollection)
        function   At(Index: Sw_Integer): PIndexEntry;
        procedure  FreeItem(Item: Pointer); virtual;
        function   Compare(Key1, Key2: Pointer): Sw_Integer; virtual;
      end;

      PUnsortedIndexEntryCollection = ^TUnsortedIndexEntryCollection;
      TUnsortedIndexEntryCollection = object(TCollection)
        function   At(Index: Sw_Integer): PIndexEntry;
        procedure  FreeItem(Item: Pointer); virtual;
      end;

      PHelpFile = ^THelpFile;
      THelpFile = object(TObject)
        ID           : word;
        Topics       : PTopicCollection;
        IndexEntries : PUnsortedIndexEntryCollection;
        constructor Init(AID: word);
        function    LoadTopic(HelpCtx: THelpCtx): PTopic; virtual;
        procedure   AddTopic(HelpCtx: THelpCtx; Pos: longint; const Param: string;
                    ExtData: pointer; ExtDataSize: longint);
        procedure   AddIndexEntry(const Text: string; AHelpCtx: THelpCtx);
        destructor  Done; virtual;
      public
        function    LoadIndex: boolean; virtual;
        function    SearchTopic(HelpCtx: THelpCtx): PTopic; virtual;
        function    ReadTopic(T: PTopic): boolean; virtual;
        function    GetTopicInfo(T: PTopic) : string; virtual;
      private
        procedure MaintainTopicCache;
      end;

      PHelpFileCollection = PCollection;

      PHelpFacility = ^THelpFacility;
      THelpFacility = object(TObject)
        HelpFiles: PHelpFileCollection;
        IndexTabSize: sw_integer;
        constructor Init;
        function    AddFile(const FileName, Param: string): PHelpFile;
        function    AddHelpFile(H: PHelpFile): boolean;
        function    LoadTopic(SourceFileID: word; Context: THelpCtx): PTopic; virtual;
        function    GetTopicInfo(SourceFileID: word; Context: THelpCtx) : string; virtual;
        function    TopicSearch(Keyword: string; var FileID: word; var Context: THelpCtx): boolean; virtual;
        function    BuildIndexTopic: PTopic; virtual;
        destructor  Done; virtual;
      private
        LastID: word;
        function  SearchFile(ID: byte): PHelpFile;
        function  SearchTopicInHelpFile(F: PHelpFile; Context: THelpCtx): PTopic;
        function  SearchTopicOwner(SourceFileID: word; Context: THelpCtx): PHelpFile;
      end;

      THelpFileOpenProc = function(const FileName,Param: string;Index : longint): PHelpFile;

      PHelpFileType = ^THelpFileType;
      THelpFileType = record
        OpenProc   : THelpFileOpenProc;
      end;

const TopicCacheSize    : sw_integer = 10;
      HelpStreamBufSize : sw_integer = 4096;
      HelpFacility      : PHelpFacility = nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      MaxHelpTopicSize  : sw_word = 1024*1024;
=======
      MaxHelpTopicSize  : sw_word = 3*65520;
>>>>>>> graemeg/fixes_2_2
=======
      MaxHelpTopicSize  : sw_word = 3*65520;
>>>>>>> origin/fixes_2_2
=======
      MaxHelpTopicSize  : sw_word = 1024*1024;
>>>>>>> origin/fixes_2.4

function  NewTopic(FileID: byte; HelpCtx: THelpCtx; Pos: longint; Param: string;
          ExtData: pointer; ExtDataSize: longint): PTopic;
procedure DisposeTopic(P: PTopic);

procedure RenderTopic(Lines: PUnsortedStringCollection; T: PTopic);
procedure BuildTopic(Lines: PUnsortedStringCollection; T: PTopic);
procedure AddLinkToTopic(T: PTopic; AFileID: word; ACtx: THelpCtx);

function  NewIndexEntry(Tag: string; FileID: word; HelpCtx: THelpCtx): PIndexEntry;
procedure DisposeIndexEntry(P: PIndexEntry);

procedure DisposeRecord(var R: TRecord);

procedure RegisterHelpFileType(AOpenProc: THelpFileOpenProc);
function  GetHelpFileTypeCount: integer;
procedure GetHelpFileType(Index: sw_integer; var HT: THelpFileType);
procedure DoneHelpFilesTypes;

implementation

uses
{$ifdef Unix}
  baseunix,
  unix,
{$endif Unix}
{$IFDEF OS2}
  DosCalls,
{$ENDIF OS2}
{$ifdef netwlibc}
  Libc,
{$endif}
{$ifdef netware_clib}
  nwserv,
{$endif}
{$ifdef HASAMIGA}
  dos,
{$endif}
  Strings,
  WConsts;

type
  PHelpFileTypeCollection = ^THelpFileTypeCollection;
  THelpFileTypeCollection = object(TCollection)
    function At(Index: sw_Integer): PHelpFileType;
    procedure FreeItem(Item: Pointer); virtual;
  end;

{$ifdef HASAMIGA}
var
  StartupTicks: Int64;
{$endif}

const
  HelpFileTypes : PHelpFileTypeCollection = nil;


function NewHelpFileType(AOpenProc: THelpFileOpenProc): PHelpFileType;
var P: PHelpFileType;
begin
  New(P);
  with P^ do begin OpenProc:=AOpenProc; end;
  NewHelpFileType:=P;
end;

procedure DisposeHelpFileType(P: PHelpFileType);
begin
  if Assigned(P) then
    Dispose(P);
end;

procedure DoneHelpFilesTypes;
begin
  if Assigned(HelpFileTypes) then
    Dispose(HelpFileTypes, Done);
end;

function THelpFileTypeCollection.At(Index: sw_Integer): PHelpFileType;
begin
  At:=inherited At(Index);
end;

procedure THelpFileTypeCollection.FreeItem(Item: Pointer);
begin
  if Assigned(Item) then
    DisposeHelpFileType(Item);
end;

procedure RegisterHelpFileType(AOpenProc: THelpFileOpenProc);
begin
  if not Assigned(HelpFileTypes) then
    New(HelpFileTypes, Init(10,10));
  HelpFileTypes^.Insert(NewHelpFileType(AOpenProc));
end;

function  GetHelpFileTypeCount: integer;
var Count: integer;
begin
  if not Assigned(HelpFileTypes) then
    Count:=0
  else
    Count:=HelpFileTypes^.Count;
  GetHelpFileTypeCount:=Count;
end;

procedure GetHelpFileType(Index: sw_integer; var HT: THelpFileType);
begin
  HT:=HelpFileTypes^.At(Index)^;
end;

{$R-}
{$Q-}

Function GetDosTicks:longint; { returns ticks at 18.2 Hz, just like DOS }
{$IFDEF OS2}
  const
    QSV_MS_COUNT = 14;
  var
    L: longint;
  begin
    DosQuerySysInfo (QSV_MS_COUNT, QSV_MS_COUNT, L, 4);
    GetDosTicks := L div 55;
  end;
{$ENDIF}
{$IFDEF Unix}
  var
    tv : TimeVal;
    tz : TimeZone;
  begin
    fpGetTimeOfDay(@tv,@tz);
    GetDosTicks:=((tv.tv_Sec mod 86400) div 60)*1092+((tv.tv_Sec mod 60)*1000000+tv.tv_USec) div 54945;
  end;
{$endif Unix}
{$ifdef Windows}
  begin
    GetDosTicks:=(Windows.GetTickCount*5484) div 100;
  end;
{$endif Windows}
{$ifdef go32v2}
  begin
    GetDosTicks:=MemL[$40:$6c];
  end;
{$endif go32v2}
{$ifdef netwlibc}
var
  tv : TTimeVal;
  tz : TTimeZone;
begin
  fpGetTimeOfDay(tv,tz);
  GetDosTicks:=((tv.tv_sec mod 86400) div 60)*1092+((tv.tv_Sec mod 60)*1000000+tv.tv_USec) div 549
end;
{$endif}
{$ifdef netware_clib}
begin
  GetDosTicks := Nwserv.GetCurrentTicks;
end;
{$endif}
{$ifdef HASAMIGA}
begin
  GetDosTicks := ((dos.GetMsCount div 55) - StartupTicks) and $7FFFFFFF;
end;
{$endif}


procedure DisposeRecord(var R: TRecord);
begin
  with R do
  if (Size>0) and (Data<>nil) then FreeMem(Data, Size);
  FillChar(R, SizeOf(R), 0);
end;

function NewTopic(FileID: byte; HelpCtx: THelpCtx; Pos: longint; Param: string;
         ExtData: pointer; ExtDataSize: longint): PTopic;
var P: PTopic;
begin
  New(P); FillChar(P^,SizeOf(P^), 0);
  P^.HelpCtx:=HelpCtx; P^.FileOfs:=Pos; P^.FileID:=FileID;
  P^.Param:=NewStr(Param);
  if Assigned(ExtData) and (ExtDataSize>0) then
  begin
    P^.ExtDataSize:=ExtDataSize;
    GetMem(P^.ExtData,ExtDataSize);
    Move(ExtData^,P^.ExtData^,ExtDataSize);
  end;
  New(P^.NamedMarks, Init(100,100));
  NewTopic:=P;
end;

procedure DisposeTopic(P: PTopic);
begin
  if P<>nil then
  begin
    if (P^.TextSize>0) and (P^.Text<>nil) then
       FreeMem(P^.Text,P^.TextSize);
    P^.Text:=nil;
    if {(P^.LinkCount>0) and }(P^.Links<>nil) then
       FreeMem(P^.Links,P^.LinkSize);
    P^.Links:=nil;
    if P^.Param<>nil then DisposeStr(P^.Param); P^.Param:=nil;
    if Assigned(P^.ExtData) then
      FreeMem(P^.ExtData);
    if Assigned(P^.NamedMarks) then Dispose(P^.NamedMarks, Done); P^.NamedMarks:=nil;
    Dispose(P);
  end;
end;

function CloneTopic(T: PTopic): PTopic;
var NT: PTopic;
procedure CloneMark(P: PString);
begin
  NT^.NamedMarks^.InsertStr(GetStr(P));
end;
begin
  New(NT);
  Move(T^,NT^,SizeOf(NT^));
  if NT^.Text<>nil then
     begin GetMem(NT^.Text,NT^.TextSize); Move(T^.Text^,NT^.Text^,NT^.TextSize); end;
  if NT^.Links<>nil then
     begin
       GetMem(NT^.Links,NT^.LinkSize);
       Move(T^.Links^,NT^.Links^,NT^.LinkSize);
     end;
  if NT^.Param<>nil then
     NT^.Param:=NewStr(T^.Param^);
  if Assigned(T^.NamedMarks) then
  begin
    New(NT^.NamedMarks, Init(T^.NamedMarks^.Count,10));
    T^.NamedMarks^.ForEach(@CloneMark);
  end;
  NT^.ExtDataSize:=T^.ExtDataSize;
  if Assigned(T^.ExtData) and (T^.ExtDataSize>0) then
  begin
    GetMem(NT^.ExtData,NT^.ExtDataSize);
    Move(T^.ExtData^,NT^.ExtData^,NT^.ExtDataSize);
  end;
  CloneTopic:=NT;
end;

procedure RenderTopic(Lines: PUnsortedStringCollection; T: PTopic);
var Size,CurPtr,I,MSize: sw_word;
    S: string;
begin
  CurPtr:=0;
  for I:=0 to Lines^.Count-1 do
  begin
    S:=GetStr(Lines^.At(I));
    Size:=length(S)+1;
    Inc(CurPtr,Size);
  end;
  Size:=CurPtr;
  T^.TextSize:=Size; GetMem(T^.Text,T^.TextSize);
  CurPtr:=0;
  for I:=0 to Lines^.Count-1 do
  begin
    S:=GetStr(Lines^.At(I)); Size:=length(S); MSize:=Size;
    if CurPtr+Size>=T^.TextSize then
      MSize:=T^.TextSize-CurPtr;
    Move(S[1],PByteArray(T^.Text)^[CurPtr],MSize);
    if MSize<>Size then
      Break;
    Inc(CurPtr,Size);
    PByteArray(T^.Text)^[CurPtr]:=ord(hscLineBreak);
    Inc(CurPtr);
    if CurPtr>=T^.TextSize then Break;
  end;
end;

procedure BuildTopic(Lines: PUnsortedStringCollection; T: PTopic);
var Size,CurPtr,MSize: sw_word;
    I: sw_integer;
    S: string;
begin
  CurPtr:=0;
  for I:=0 to Lines^.Count-1 do
  begin
    S:=GetStr(Lines^.At(I));
    Size:=length(S);
    Inc(CurPtr,Size);
  end;
  Size:=CurPtr;
  T^.TextSize:=Size; GetMem(T^.Text,T^.TextSize);
  CurPtr:=0;
  for I:=0 to Lines^.Count-1 do
  begin
    S:=GetStr(Lines^.At(I)); Size:=length(S); MSize:=Size;
    if Size>0 then
    begin
      if CurPtr+Size>=T^.TextSize then
        MSize:=T^.TextSize-CurPtr;
      Move(S[1],PByteArray(T^.Text)^[CurPtr],MSize);
      if MSize<>Size then
        Break;
      Inc(CurPtr,Size);
    end;
    if CurPtr>=T^.TextSize then Break;
  end;
end;

procedure AddLinkToTopic(T: PTopic; AFileID: word; ACtx: THelpCtx);
var NewSize: word;
    NewPtr: pointer;
begin
  NewSize:=longint(T^.LinkCount+1)*sizeof(T^.Links^[0]);
  GetMem(NewPtr,NewSize);
  if Assigned(T^.Links) then
  begin
    Move(T^.Links^,NewPtr^,T^.LinkSize);
    FreeMem(T^.Links,T^.LinkSize);
  end;
  T^.Links:=NewPtr;
  with T^.Links^[T^.LinkCount] do
  begin
    FileID:=AFileID;
    Context:=ACtx;
  end;
  Inc(T^.LinkCount);
end;

function NewIndexEntry(Tag: string; FileID: word; HelpCtx: THelpCtx): PIndexEntry;
var P: PIndexEntry;
begin
  New(P); FillChar(P^,SizeOf(P^), 0);
  P^.Tag:=NewStr(Tag); P^.FileID:=FileID; P^.HelpCtx:=HelpCtx;
  NewIndexEntry:=P;
end;

procedure DisposeIndexEntry(P: PIndexEntry);
begin
  if P<>nil then
  begin
    if P^.Tag<>nil then DisposeStr(P^.Tag);
    Dispose(P);
  end;
end;

function TTopic.LinkSize: sw_word;
begin
  LinkSize:=LinkCount*SizeOf(Links^[0]);
end;

function TTopic.GetNamedMarkIndex(const MarkName: string): sw_integer;
var I,Index: sw_integer;
begin
  Index:=-1;
  if Assigned(NamedMarks) then
  for I:=0 to NamedMarks^.Count-1 do
    if CompareText(GetStr(NamedMarks^.At(I)),MarkName)=0 then
     begin
       Index:=I;
       Break;
     end;
  GetNamedMarkIndex:=Index;
end;

function TTopicCollection.At(Index: sw_Integer): PTopic;
begin
  At:=inherited At(Index);
end;

procedure TTopicCollection.FreeItem(Item: Pointer);
begin
  if Item<>nil then DisposeTopic(Item);
end;

function TTopicCollection.Compare(Key1, Key2: Pointer): Sw_Integer;
var K1: PTopic absolute Key1;
    K2: PTopic absolute Key2;
    R: Sw_integer;
begin
  if K1^.HelpCtx<K2^.HelpCtx then R:=-1 else
  if K1^.HelpCtx>K2^.HelpCtx then R:= 1 else
  R:=0;
  Compare:=R;
end;

function TTopicCollection.SearchTopic(AHelpCtx: THelpCtx): PTopic;
var T: TTopic;
    P: PTopic;
    Index: sw_integer;
begin
  T.HelpCtx:=AHelpCtx;
  if Search(@T,Index) then
    P:=At(Index)
  else
    P:=nil;
  SearchTopic:=P;
end;

function TIndexEntryCollection.At(Index: Sw_Integer): PIndexEntry;
begin
  At:=inherited At(Index);
end;

procedure TIndexEntryCollection.FreeItem(Item: Pointer);
begin
  if Item<>nil then DisposeIndexEntry(Item);
end;

function TUnsortedIndexEntryCollection.At(Index: Sw_Integer): PIndexEntry;
begin
  At:=inherited At(Index);
end;

procedure TUnsortedIndexEntryCollection.FreeItem(Item: Pointer);
begin
  if Item<>nil then DisposeIndexEntry(Item);
end;

function TIndexEntryCollection.Compare(Key1, Key2: Pointer): Sw_Integer;
var K1: PIndexEntry absolute Key1;
    K2: PIndexEntry absolute Key2;
    R: Sw_integer;
    S1,S2: string;
    T1,T2 : PTopic;
begin
  S1:=UpcaseStr(K1^.Tag^); S2:=UpcaseStr(K2^.Tag^);
  if S1<S2 then
    begin
      Compare:=-1;
      exit;
    end;
  if S1>S2 then
    begin
      Compare:=1;
      exit;
    end;
  (* if assigned(HelpFacility) then
    begin
      { Try to read the title of the topic }
      T1:=HelpFacility^.LoadTopic(K1^.FileID,K1^.HelpCtx);
      T2:=HelpFacility^.LoadTopic(K2^.FileID,K2^.HelpCtx);
      if assigned(T1^.Text) and assigned(T2^.Text) then
        r:=strcomp(pchar(T1^.Text),pchar(T2^.Text))
      else
        r:=0;
      if r>0 then
        begin
          Compare:=1;
          exit;
        end;
      if r<0 then
        begin
          Compare:=-1;
          exit;
        end;
    end; *)
  if K1^.FileID<K2^.FileID then R:=-1
  else if K1^.FileID>K2^.FileID then R:= 1
  else if K1^.HelpCtx<K2^.HelpCtx then
    r:=-1
  else if K1^.HelpCtx>K2^.HelpCtx then
    r:=1
  else
    R:=0;
  Compare:=R;
end;

constructor THelpFile.Init(AID: word);
begin
  inherited Init;
  ID:=AID;
  New(Topics, Init(2000,1000));
  New(IndexEntries, Init(2000,1000));
end;

procedure THelpFile.AddTopic(HelpCtx: THelpCtx; Pos: longint; const Param: string; ExtData: pointer; ExtDataSize: longint);
begin
  Topics^.Insert(NewTopic(ID,HelpCtx,Pos,Param,ExtData,ExtDataSize));
end;

procedure THelpFile.AddIndexEntry(const Text: string; AHelpCtx: THelpCtx);
begin
  IndexEntries^.Insert(NewIndexEntry(Text,ID,AHelpCtx));
end;

function THelpFile.LoadTopic(HelpCtx: THelpCtx): PTopic;
var T: PTopic;
begin
  T:=SearchTopic(HelpCtx);
  if (T<>nil) then
   if T^.Text=nil then
     begin
       MaintainTopicCache;
       if ReadTopic(T)=false then
           T:=nil;
       if (T<>nil) and (T^.Text=nil) then T:=nil;
     end;
  if T<>nil then
     begin
       T^.LastAccess:=GetDosTicks;
       T:=CloneTopic(T);
     end;
  LoadTopic:=T;
end;

function THelpFile.LoadIndex: boolean;
begin
  Abstract;
  LoadIndex:=false; { remove warning }
end;

function THelpFile.SearchTopic(HelpCtx: THelpCtx): PTopic;
var T: PTopic;
begin
  T:=Topics^.SearchTopic(HelpCtx);
  SearchTopic:=T;
end;

function THelpFile.ReadTopic(T: PTopic): boolean;
begin
  Abstract;
  ReadTopic:=false; { remove warning }
end;

function THelpFile.GetTopicInfo(T: PTopic) : string;
begin
  Abstract;
  GetTopicInfo:=''; { remove warning }
end;

procedure THelpFile.MaintainTopicCache;
var Count: sw_integer;
    MinLRU: longint;
procedure CountThem(P: PTopic);
begin if (P^.Text<>nil) or (P^.Links<>nil) then Inc(Count); end;
procedure SearchLRU(P: PTopic);
begin if P^.LastAccess<MinLRU then begin MinLRU:=P^.LastAccess; end; end;
var P: PTopic;
begin
  Count:=0; Topics^.ForEach(@CountThem);
  if (Count>=TopicCacheSize) then
  begin
    MinLRU:=MaxLongint; P:=nil; Topics^.ForEach(@SearchLRU);
    if P<>nil then
    begin
      FreeMem(P^.Text,P^.TextSize); P^.TextSize:=0; P^.Text:=nil;
      FreeMem(P^.Links,P^.LinkSize); P^.LinkCount:=0; P^.Links:=nil;
    end;
  end;
end;

destructor THelpFile.Done;
begin
  if Topics<>nil then Dispose(Topics, Done);
  if IndexEntries<>nil then Dispose(IndexEntries, Done);
  inherited Done;
end;

constructor THelpFacility.Init;
begin
  inherited Init;
  New(HelpFiles, Init(10,10));
  IndexTabSize:=40;
end;

function THelpFacility.AddFile(const FileName, Param: string): PHelpFile;
var H: PHelpFile;
    OK: boolean;
    I: integer;
    HT: THelpFileType;
begin
  OK:=false; H:=nil;
  for I:=0 to GetHelpFileTypeCount-1 do
  begin
    GetHelpFileType(I,HT);
    H:=HT.OpenProc(FileName,Param,LastID+1);
    if Assigned(H) then
      Break;
  end;
  if Assigned(H) then
    OK:=AddHelpFile(H);
  if (not OK) and Assigned(H) then begin Dispose(H, Done); H:=nil; end;
  AddFile:=H;
end;

function THelpFacility.AddHelpFile(H: PHelpFile): boolean;
begin
  if H<>nil then
    begin
      HelpFiles^.Insert(H);
      Inc(LastID);
      { H^.ID:=LastID; now already set by OpenProc PM }
    end;
  AddHelpFile:=H<>nil;
end;

function THelpFacility.SearchTopicOwner(SourceFileID: word; Context: THelpCtx): PHelpFile;
var P: PTopic;
    HelpFile: PHelpFile;
function Search(F: PHelpFile): boolean;
begin
  P:=SearchTopicInHelpFile(F,Context); if P<>nil then HelpFile:=F;
  Search:=P<>nil;
end;
begin
  HelpFile:=nil;
  if SourceFileID=0 then P:=nil else
     begin
       HelpFile:=SearchFile(SourceFileID);
       P:=SearchTopicInHelpFile(HelpFile,Context);
     end;
  if P=nil then HelpFiles^.FirstThat(@Search);
  if P=nil then HelpFile:=nil;
  SearchTopicOwner:=HelpFile;
end;

function THelpFacility.LoadTopic(SourceFileID: word; Context: THelpCtx): PTopic;
var P: PTopic;
    H: PHelpFile;
begin
  if (SourceFileID=0) and (Context=0) then
     P:=BuildIndexTopic else
  begin
    H:=SearchTopicOwner(SourceFileID,Context);
    if (H=nil) then P:=nil else
       P:=H^.LoadTopic(Context);
  end;
  LoadTopic:=P;
end;

function THelpFacility.GetTopicInfo(SourceFileID: word; Context: THelpCtx) : string;
var P: PTopic;
    H: PHelpFile;
begin
  if (SourceFileID=0) and (Context=0) then
     begin
       P:=BuildIndexTopic;
     end
  else
    begin
      H:=SearchTopicOwner(SourceFileID,Context);
      if (H=nil) then P:=nil else
         P:=H^.SearchTopic(Context);
    end;
  If not assigned(P) then
    GetTopicInfo:='Not found'
  else
    GetTopicInfo:=H^.GetTopicInfo(P);
end;



function THelpFacility.TopicSearch(Keyword: string; var FileID: word; var Context: THelpCtx): boolean;
function ScanHelpFileExact(H: PHelpFile): boolean;
function SearchExact(P: PIndexEntry): boolean;
begin
  SearchExact:=UpcaseStr(P^.Tag^)=Keyword;
end;
var P: PIndexEntry;
begin
  H^.LoadIndex;
  P:=H^.IndexEntries^.FirstThat(@SearchExact);
  if P<>nil then begin FileID:=H^.ID; Context:=P^.HelpCtx; end;
  ScanHelpFileExact:=P<>nil;
end;
function ScanHelpFile(H: PHelpFile): boolean;
function Search(P: PIndexEntry): boolean;
begin
  Search:=copy(UpcaseStr(P^.Tag^),1,length(Keyword))=Keyword;
end;
var P: PIndexEntry;
begin
  H^.LoadIndex;
  P:=H^.IndexEntries^.FirstThat(@Search);
  if P<>nil then begin FileID:=H^.ID; Context:=P^.HelpCtx; end;
  ScanHelpFile:=P<>nil;
end;
var
  PH : PHelpFile;
begin
  Keyword:=UpcaseStr(Keyword);
  PH:=HelpFiles^.FirstThat(@ScanHelpFileExact);
  if not assigned(PH) then
    PH:=HelpFiles^.FirstThat(@ScanHelpFile);
  TopicSearch:=PH<>nil;
end;

function THelpFacility.BuildIndexTopic: PTopic;
var T: PTopic;
    Keywords: PIndexEntryCollection;
    Lines: PUnsortedStringCollection;
procedure InsertKeywordsOfFile(H: PHelpFile);
function InsertKeywords(P: PIndexEntry): boolean;
begin
  Keywords^.Insert(P);
  InsertKeywords:=Keywords^.Count>=MaxCollectionSize;
end;
begin
  H^.LoadIndex;
  if Keywords^.Count<MaxCollectionSize then
  H^.IndexEntries^.FirstThat(@InsertKeywords);
end;
procedure AddLine(S: string);
begin
  if S='' then S:=' ';
  Lines^.Insert(NewStr(S));
end;
var Line: string;
procedure FlushLine;
begin
  if Line<>'' then AddLine(Line); Line:='';
end;
var KWCount,NLFlag: sw_integer;
    LastFirstChar: char;
procedure NewSection(FirstChar: char);
begin
  if FirstChar<=#64 then FirstChar:=#32;
  FlushLine;
  AddLine('');
  AddLine(FirstChar);
  AddLine('');
  LastFirstChar:=FirstChar;
  NLFlag:=0;
end;
function FormatAlias(Alias: string): string;
var StartP,EndP: sw_integer;
begin
  repeat
    StartP:=Pos('  ',Alias);
    if StartP>0 then
    begin
      EndP:=StartP;
      while (EndP+1<=length(Alias)) and (Alias[EndP+1]=' ') do Inc(EndP);
      Alias:=copy(Alias,1,StartP-1)+' | '+copy(Alias,EndP+1,High(Alias));
    end;
  until StartP=0;
  if Assigned(HelpFacility) then
    if length(Alias)>IndexTabSize-4 then
      Alias:=Trim(copy(Alias,1,IndexTabSize-4-2))+'..';
  FormatAlias:=Alias;
end;
procedure AddKeyword(KWS: string);
begin
  Inc(KWCount); if KWCount=1 then NLFlag:=0;
  if (KWCount=1) or
     ( (Upcase(KWS[1])<>LastFirstChar) and ( (LastFirstChar>#64) or (KWS[1]>#64) ) ) then
     NewSection(Upcase(KWS[1]));

  KWS:=FormatAlias(KWS);

  if (NLFlag mod 2)=0
     then Line:=' '+#2+KWS+#2
     else begin
            Line:=RExpand(Line,IndexTabSize)+#2+KWS+#2;
            FlushLine;
          end;
  Inc(NLFlag);
end;
var KW: PIndexEntry;
    I,p : sw_integer;
    IsMultiple : boolean;
    MultiCount : longint;
    St,LastTag : String;
begin
  New(Keywords, Init(5000,5000));
  HelpFiles^.ForEach(@InsertKeywordsOfFile);
  New(Lines, Init((Keywords^.Count div 2)+100,1000));
  T:=NewTopic(0,0,0,'',nil,0);
  if HelpFiles^.Count=0 then
    begin
      AddLine('');
      AddLine(msg_nohelpfilesinstalled1);
      AddLine(msg_nohelpfilesinstalled2);
      AddLine(msg_nohelpfilesinstalled3);
      AddLine(msg_nohelpfilesinstalled4);
      AddLine(msg_nohelpfilesinstalled5);
    end else
  begin
    AddLine(' '+msg_helpindex);
    KWCount:=0; Line:='';
    T^.LinkCount:=Min(Keywords^.Count,MaxBytes div sizeof(T^.Links^[0])-1);
    GetMem(T^.Links,T^.LinkSize);
    MultiCount:=0;
    LastTag:='';
    for I:=0 to T^.LinkCount-1 do
    begin
      KW:=Keywords^.At(I);
      if (LastTag<>KW^.Tag^) then
        Begin
          MultiCount:=0;
          IsMultiple:=(I<T^.LinkCount-1) and (KW^.Tag^=Keywords^.At(I+1)^.Tag^);
        End
      else
        IsMultiple:=true;
      if IsMultiple then
        Begin
          Inc(MultiCount);
          (* St:=Trim(strpas(pchar(HelpFacility^.LoadTopic(KW^.FileID,KW^.HelpCtx)^.Text))); *)
          St:=KW^.Tag^+' ['+IntToStr(MultiCount)+']';
          (* { Remove all special chars }
          for p:=1 to Length(st) do
            if ord(st[p])<=16 then
              st[p]:=' ';
          p:=pos(KW^.Tag^,St);
          if (p=1) then
            AddKeyword(St)
          else
            AddKeyword(KW^.Tag^+' '+St); *)
          AddKeyWord(St);
        End
      else
        AddKeyword(KW^.Tag^);
      LastTag:=KW^.Tag^;
      T^.Links^[I].Context:=longint(KW^.HelpCtx);
      T^.Links^[I].FileID:=KW^.FileID;
    end;
    FlushLine;
    AddLine('');
  end;
  RenderTopic(Lines,T);
  Dispose(Lines, Done);
  Keywords^.DeleteAll; Dispose(Keywords, Done);
  BuildIndexTopic:=T;
end;

function THelpFacility.SearchFile(ID: byte): PHelpFile;
function Match(P: PHelpFile): boolean;
begin
  Match:=(P^.ID=ID);
end;
begin
  SearchFile:=HelpFiles^.FirstThat(@Match);
end;

function THelpFacility.SearchTopicInHelpFile(F: PHelpFile; Context: THelpCtx): PTopic;
var P: PTopic;
begin
  if F=nil then P:=nil else
  P:=F^.SearchTopic(Context);
  SearchTopicInHelpFile:=P;
end;

destructor THelpFacility.Done;
begin
  inherited Done;
  Dispose(HelpFiles, Done);
end;

{$ifdef HASAMIGA}
INITIALIZATION
  StartupTicks := dos.GetMsCount div 55;
{$endif}

END.
