{
    Copyright (c) 1998-2002 by Berczi Gabor
    Modifications Copyright (c) 1999-2002 Florian Klaempfl and Pierre Muller

    Support routines for getting browser info in collections

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
{$ifdef TP}
  {$N+,E+}
{$endif}

unit browcol;

{$i fpcdefs.inc}
{ $define use_refs}
{$H-}

{$ifdef cpullvm}
{$modeswitch nestedprocvars}
{$endif}

interface

uses
  SysUtils,
  CUtils,
  objects,
  cclasses,
  symconst,symtable;

const
  SymbolTypLen : integer = 6;

  RecordTypes : set of tsymtyp =
    ([typesym,unitsym]);

    sfRecord        = $00000001;
    sfObject        = $00000002;
    sfClass         = $00000004;
    sfPointer       = $00000008;
    sfHasMemInfo    = $40000000;

type
    TStoreCollection = object(TStringCollection)
      function Add(const S: string): PString;
    end;

    PModuleNameCollection = ^TModuleNameCollection;
    TModuleNameCollection = object(TStoreCollection)
    end;

    PTypeNameCollection = ^TTypeNameCollection;
    TTypeNameCollection = object(TStoreCollection)
    end;

    PSymbolCollection       = ^TSymbolCollection;
    PSortedSymbolCollection = ^TSortedSymbolCollection;
    PReferenceCollection    = ^TReferenceCollection;

    PReference = ^TReference;
    TReference = object(TObject)
      FileName  : PString;
      Position  : TPoint;
      constructor Init(AFileName: PString; ALine, AColumn: Sw_integer);
      function    GetFileName: string;
      destructor  Done; virtual;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
    end;

    PSymbolMemInfo = ^TSymbolMemInfo;
    TSymbolMemInfo = record
      Addr      : longint;
      Size      : longint;
      PushSize  : longint;
    end;

    PSymbol = ^TSymbol;
    TSymbol = object(TObject)
      Name       : PString;
      Typ        : tsymtyp;
      varoptions : tvaroptions;
      varspez    : tvarspez;  { sets the type of access }
      Params     : PString;
      References : PReferenceCollection;
      Items      : PSymbolCollection;
      DType      : PString;
      VType      : PString;
      TypeID     : Ptrint;
      RelatedTypeID : Ptrint;
      DebuggerCount : longint;
      Ancestor   : PSymbol;
      Flags      : longint;
      MemInfo    : PSymbolMemInfo;
      constructor Init(const AName: string; ATyp: tsymtyp; AParams: string; AMemInfo: PSymbolMemInfo);
      procedure   SetMemInfo(const AMemInfo: TSymbolMemInfo);
      function    GetReferenceCount: Sw_integer;
      function    GetReference(Index: Sw_integer): PReference;
      function    GetItemCount: Sw_integer;
      function    GetItem(Index: Sw_integer): PSymbol;
      function    GetName: string;
      function    GetText: string;
      function    GetTypeName: string;
      destructor  Done; virtual;
      procedure   SetVarSpez(const AVarSpez : TVarSpez);
      procedure   SetVarOptions(const AVarOptions : TVarOptions);
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
    end;

    PExport = ^TExport;
    TExport = object(TObject)
      constructor Init(const AName: string; AIndex: longint; ASymbol: PSymbol);
      function    GetDisplayText: string;
      destructor  Done; virtual;
    private
      Name: PString;
      Index: longint;
      Symbol: PSymbol;
    end;

    PExportCollection = ^TExportCollection;
    TExportCollection = object(TSortedCollection)
      function At(Index: sw_Integer): PExport;
      function Compare(Key1, Key2: Pointer): sw_Integer; virtual;
    end;

    PImport = ^TImport;
    TImport = object(TObject)
      constructor Init(const ALibName, AFuncName,ARealName: string; AIndex: longint);
      function    GetDisplayText: string;
      destructor  Done; virtual;
    private
      LibName: PString;
      FuncName: PString;
      RealName: PString;
      Index: longint;
    end;

    PImportCollection = ^TImportCollection;
    TImportCollection = object(TSortedCollection)
      function At(Index: sw_Integer): PImport;
      function Compare(Key1, Key2: Pointer): sw_Integer; virtual;
    end;

    PObjectSymbolCollection = ^TObjectSymbolCollection;

    PObjectSymbol = ^TObjectSymbol;
    TObjectSymbol = object(TObject)
      Parent     : PObjectSymbol;
      Symbol     : PSymbol;
      Expanded   : boolean;
      constructor Init(AParent: PObjectSymbol; ASymbol: PSymbol);
      constructor InitName(const AName: string);
      function    GetName: string;
      function    GetDescendantCount: sw_integer;
      function    GetDescendant(Index: sw_integer): PObjectSymbol;
      procedure   AddDescendant(P: PObjectSymbol);
      destructor  Done; virtual;
      constructor Load(var S: TStream);
      procedure   Store(S: TStream);
    private
      Name: PString;
      Descendants: PObjectSymbolCollection;
    end;

    TSymbolCollection = object(TSortedCollection)
       constructor Init(ALimit, ADelta: Integer);
       function  At(Index: Sw_Integer): PSymbol;
       procedure Insert(Item: Pointer); virtual;
       function  LookUp(const S: string; var Idx: sw_integer): string; virtual;
    end;

    TSortedSymbolCollection = object(TSymbolCollection)
      function  Compare(Key1, Key2: Pointer): Sw_Integer; virtual;
      procedure Insert(Item: Pointer); virtual;
      function  LookUp(const S: string; var Idx: sw_integer): string; virtual;
    end;

    PIDSortedSymbolCollection = ^TIDSortedSymbolCollection;
    TIDSortedSymbolCollection = object(TSymbolCollection)
      function  Compare(Key1, Key2: Pointer): Sw_Integer; virtual;
      procedure Insert(Item: Pointer); virtual;
      function  SearchSymbolByID(AID: longint): PSymbol;
    end;

    TObjectSymbolCollection = object(TSortedCollection)
      constructor Init(ALimit, ADelta: Integer);
      function  Compare(Key1, Key2: Pointer): Sw_Integer; virtual;
      function  LookUp(const S: string; var Idx: sw_integer): string; virtual;
       function At(Index: Sw_Integer): PObjectSymbol;
    end;

    TReferenceCollection = object(TCollection)
       function At(Index: Sw_Integer): PReference;
    end;

    PSourceFile = ^TSourceFile;
    TSourceFile = object(TObject)
      SourceFileName: PString;
      ObjFileName: PString;
      PPUFileName: PString;
      constructor Init(ASourceFileName, AObjFileName, APPUFileName: string);
      destructor  Done; virtual;
      function    GetSourceFilename: string;
      function    GetObjFileName: string;
      function    GetPPUFileName: string;
    end;

    PSourceFileCollection = ^TSourceFileCollection;
    TSourceFileCollection = object(TCollection)
      function At(Index: sw_Integer): PSourceFile;
    end;

    PModuleSymbol = ^TModuleSymbol;
    TModuleSymbol = object(TSymbol)
      Exports_   : PExportCollection;
      Imports    : PImportCollection;
      LoadedFrom : PString;
      UsedUnits  : PSymbolCollection;
      DependentUnits: PSymbolCollection;
      MainSource: PString;
      SourceFiles: pstringCollection;
      constructor Init(const AName, AMainSource: string);
      procedure   SetLoadedFrom(const AModuleName: string);
      procedure   AddUsedUnit(P: PSymbol);
      procedure   AddDependentUnit(P: PSymbol);
      procedure   AddSourceFile(const Path: string);
      destructor  Done; virtual;
    end;

const
  Modules     : PSymbolCollection = nil;
  ModuleNames : PModuleNameCollection = nil;
  TypeNames   : PTypeNameCollection = nil;
  ObjectTree  : PObjectSymbol = nil;
  SourceFiles : PSourceFileCollection = nil;

procedure DisposeBrowserCol;
procedure NewBrowserCol;
procedure CreateBrowserCol;
procedure InitBrowserCol;
procedure DoneBrowserCol;

function  LoadBrowserCol(S: PStream): boolean;
function  StoreBrowserCol(S: PStream) : boolean;

procedure BuildObjectInfo;

procedure BuildSourceList;

function SearchObjectForSymbol(O: PSymbol): PObjectSymbol;

procedure RegisterSymbols;

implementation

uses
  globtype,globals,comphook,constexp,
{$ifdef DEBUG}
  verbose,
{$endif DEBUG}
  finput,fmodule,
  crefs,cpuinfo,cgbase,
  aasmbase,aasmtai,aasmdata,paramgr,
  symsym,symdef,symtype,symbase,defutil;

const
  RModuleNameCollection: TStreamRec = (
     ObjType: 3001;
     VmtLink: Ofs(TypeOf(TModuleNameCollection)^);
     Load:    @TModuleNameCollection.Load;
     Store:   @TModuleNameCollection.Store;
     Next: nil
  );
  RTypeNameCollection: TStreamRec = (
     ObjType: 3002;
     VmtLink: Ofs(TypeOf(TTypeNameCollection)^);
     Load:    @TTypeNameCollection.Load;
     Store:   @TTypeNameCollection.Store;
     Next: nil
  );
  RReference: TStreamRec = (
     ObjType: 3003;
     VmtLink: Ofs(TypeOf(TReference)^);
     Load:    @TReference.Load;
     Store:   @TReference.Store;
     Next: nil
  );
  RSymbol: TStreamRec = (
     ObjType: 3004;
     VmtLink: Ofs(TypeOf(TSymbol)^);
     Load:    @TSymbol.Load;
     Store:   @TSymbol.Store;
     Next: nil
  );
  RObjectSymbol: TStreamRec = (
     ObjType: 3005;
     VmtLink: Ofs(TypeOf(TObjectSymbol)^);
     Load:    @TObjectSymbol.Load;
     Store:   @TObjectSymbol.Store;
     Next: nil
  );
  RSymbolCollection: TStreamRec = (
     ObjType: 3006;
     VmtLink: Ofs(TypeOf(TSymbolCollection)^);
     Load:    @TSymbolCollection.Load;
     Store:   @TSymbolCollection.Store;
     Next: nil
  );
  RSortedSymbolCollection: TStreamRec = (
     ObjType: 3007;
     VmtLink: Ofs(TypeOf(TSortedSymbolCollection)^);
     Load:    @TSortedSymbolCollection.Load;
     Store:   @TSortedSymbolCollection.Store;
     Next: nil
  );
  RIDSortedSymbolCollection: TStreamRec = (
     ObjType: 3008;
     VmtLink: Ofs(TypeOf(TIDSortedSymbolCollection)^);
     Load:    @TIDSortedSymbolCollection.Load;
     Store:   @TIDSortedSymbolCollection.Store;
     Next: nil
  );
  RObjectSymbolCollection: TStreamRec = (
     ObjType: 3009;
     VmtLink: Ofs(TypeOf(TObjectSymbolCollection)^);
     Load:    @TObjectSymbolCollection.Load;
     Store:   @TObjectSymbolCollection.Store;
     Next: nil
  );
  RReferenceCollection: TStreamRec = (
     ObjType: 3010;
     VmtLink: Ofs(TypeOf(TReferenceCollection)^);
     Load:    @TReferenceCollection.Load;
     Store:   @TReferenceCollection.Store;
     Next: nil
  );
  RModuleSymbol: TStreamRec = (
     ObjType: 3011;
     VmtLink: Ofs(TypeOf(TModuleSymbol)^);
     Load:    @TModuleSymbol.Load;
     Store:   @TModuleSymbol.Store;
     Next: nil
  );

  SymbolCount : longint = 0;

{****************************************************************************
                                   Helpers
****************************************************************************}

function GetStr(P: PString): string;
begin
  if P=nil then
    GetStr:=''
  else
    GetStr:=P^;
end;

function IntToStr(L: longint): string;
var S: string;
begin
  Str(L,S);
  IntToStr:=S;
end;

function UpcaseStr(S: string): string;
var I: integer;
begin
  for I:=1 to length(S) do
      S[I]:=Upcase(S[I]);
  UpcaseStr:=S;
end;

function FloatToStr(E: extended): string;
var S: string;
begin
  Str(E:0:24,S);
  if Pos('.',S)>0 then
    begin
      while (length(S)>0) and (S[length(S)]='0') do
        Delete(S,length(S),1);
      if (length(S)>0) and (S[length(S)]='.') then
        Delete(S,length(S),1);
    end;
  if S='' then S:='0';
  FloatToStr:=S;
end;

{****************************************************************************
                                TStoreCollection
****************************************************************************}

function TStoreCollection.Add(const S: string): PString;
var P: PString;
    Index: Sw_integer;
begin
  if S='' then P:=nil else
  if Search(@S,Index) then P:=At(Index) else
    begin
      P:=NewStr(S);
      Insert(P);
    end;
  Add:=P;
end;


{****************************************************************************
                                TSymbolCollection
****************************************************************************}

constructor TSymbolCollection.Init(ALimit, ADelta: Integer);
begin
  inherited Init(ALimit,ADelta);
  Duplicates:=true;
end;

function TSymbolCollection.At(Index: Sw_Integer): PSymbol;
begin
  At:=inherited At(Index);
end;

procedure TSymbolCollection.Insert(Item: Pointer);
begin

  TCollection.Insert(Item);
end;

function TSymbolCollection.LookUp(const S: string; var Idx: sw_integer): string;
begin
  Idx:=-1;
  LookUp:='';
end;

{****************************************************************************
                               TReferenceCollection
****************************************************************************}

function TReferenceCollection.At(Index: Sw_Integer): PReference;
begin
  At:=inherited At(Index);
end;


{****************************************************************************
                            TSortedSymbolCollection
****************************************************************************}

function TSortedSymbolCollection.Compare(Key1, Key2: Pointer): Sw_Integer;
var K1: PSymbol absolute Key1;
    K2: PSymbol absolute Key2;
    R: Sw_integer;
    S1,S2: string;
begin
  S1:=Upper(K1^.GetName);
  S2:=Upper(K2^.GetName);
  if S1<S2 then R:=-1 else
  if S1>S2 then R:=1 else
   if K1^.TypeID=K2^.TypeID then
     R:=0
   else
    begin
      S1:=K1^.GetName;
      S2:=K2^.GetName;
      if S1<S2 then R:=-1 else
      if S1>S2 then R:=1 else
       if K1^.TypeID<K2^.TypeID then R:=-1 else
       if K1^.TypeID>K2^.TypeID then R:= 1 else
         begin
           { Handle overloaded functions }
           if (K1^.Typ=procsym) then
             begin
               S1:=K1^.GetText;
               S2:=K2^.GetText;
               if S1<S2 then R:=-1 else
               if S1>S2 then R:=1 else
                 R:=0;
             end
           else
             R:=0;
         end
    end;
  Compare:=R;
end;

procedure TSortedSymbolCollection.Insert(Item: Pointer);
begin
  TSortedCollection.Insert(Item);
end;

function TSortedSymbolCollection.LookUp(const S: string; var Idx: sw_integer): string;
var OLI,ORI,Left,Right,Mid: integer;
    LeftP,RightP,MidP: PSymbol;
    LeftS,MidS,RightS: string;
    FoundS: string;
    UpS : string;
begin
  Idx:=-1; FoundS:='';
  Left:=0; Right:=Count-1;
  UpS:=Upper(S);
  if Left<Right then
  begin
    while (Left<Right) do
    begin
      OLI:=Left; ORI:=Right;
      Mid:=Left+(Right-Left) div 2;
      MidP:=At(Mid);
{$ifdef DEBUG}
      LeftP:=At(Left); RightP:=At(Right);
      LeftS:=Upper(LeftP^.GetName);
      RightS:=Upper(RightP^.GetName);
{$endif DEBUG}
      MidS:=Upper(MidP^.GetName);
      if copy(MidS,1,length(UpS))=UpS then
        begin
          Idx:=Mid;
          FoundS:=MidS;
        end;
{      else}
        if UpS<MidS then
          Right:=Mid
        else
          Left:=Mid;
      if (OLI=Left) and (ORI=Right) then
        begin
          if idX<>-1 then
            break;
          if Mid=Left then
            begin
              RightP:=At(Right);
              RightS:=Upper(RightP^.GetName);
              if copy(RightS,1,length(UpS))=UpS then
                begin
                  Idx:=Right;
                  FoundS:=RightS;
                end;
            end;
          if Mid=Right then
            begin
              LeftP:=At(Left);
              LeftS:=Upper(LeftP^.GetName);
              if copy(LeftS,1,length(UpS))=UpS then
                begin
                  Idx:=Left;
                  FoundS:=LeftS;
                end;
            end;
          Break;
        end;
    end;
  end;
  LookUp:=FoundS;
end;

{****************************************************************************
                           TIDSortedSymbolCollection
****************************************************************************}

function TIDSortedSymbolCollection.Compare(Key1, Key2: Pointer): Sw_Integer;
var K1: PSymbol absolute Key1;
    K2: PSymbol absolute Key2;
    R: Sw_integer;
begin
  if K1^.TypeID<K2^.TypeID then R:=-1 else
  if K1^.TypeID>K2^.TypeID then R:= 1 else
  R:=0;
  Compare:=R;
end;

procedure TIDSortedSymbolCollection.Insert(Item: Pointer);
begin
  TSortedCollection.Insert(Item);
end;

function TIDSortedSymbolCollection.SearchSymbolByID(AID: longint): PSymbol;
var S: TSymbol;
    Index: sw_integer;
    P: PSymbol;
begin
  S.TypeID:=AID;
  if Search(@S,Index)=false then P:=nil else
    P:=At(Index);
  SearchSymbolByID:=P;
end;

{****************************************************************************
                           TObjectSymbolCollection
****************************************************************************}

function TObjectSymbolCollection.At(Index: Sw_Integer): PObjectSymbol;
begin
  At:=inherited At(Index);
end;

constructor TObjectSymbolCollection.Init(ALimit, ADelta: Integer);
begin
  inherited Init(ALimit,ADelta);
end;

function TObjectSymbolCollection.Compare(Key1, Key2: Pointer): Sw_Integer;
var K1: PObjectSymbol absolute Key1;
    K2: PObjectSymbol absolute Key2;
    R: Sw_integer;
    S1,S2: string;
begin
  S1:=Upper(K1^.GetName);
  S2:=Upper(K2^.GetName);
  if S1<S2 then R:=-1 else
  if S1>S2 then R:=1 else
  { make sure that we distinguish between different objects with the same name }
  if PtrUInt(K1^.Symbol)<PtrUInt(K2^.Symbol) then R:=-1 else
  if PtrUInt(K1^.Symbol)>PtrUInt(K2^.Symbol) then R:= 1 else
  R:=0;
  Compare:=R;
end;

function TObjectSymbolCollection.LookUp(const S: string; var Idx: sw_integer): string;
var OLI,ORI,Left,Right,Mid: integer;
    {LeftP,RightP,}MidP: PObjectSymbol;
    {LeftS,RightS,}MidS: string;
    FoundS: string;
    UpS : string;
begin
  Idx:=-1; FoundS:='';
  Left:=0; Right:=Count-1;
  UpS:=Upper(S);
  if Left<Right then
  begin
    while (Left<Right) do
    begin
      OLI:=Left; ORI:=Right;
      Mid:=Left+(Right-Left) div 2;
      {LeftP:=At(Left);
       LeftS:=Upper(LeftP^.GetName);}
      MidP:=At(Mid);
      MidS:=Upper(MidP^.GetName);
      {RightP:=At(Right);
       RightS:=Upper(RightP^.GetName);}
      if copy(MidS,1,length(UpS))=UpS then
        begin
          Idx:=Mid;
          FoundS:=MidS;
        end;
{      else}
        if UpS<MidS then
          Right:=Mid
        else
          Left:=Mid;
      if (OLI=Left) and (ORI=Right) then
        Break;
    end;
  end;
  LookUp:=FoundS;
end;

{****************************************************************************
                                TReference
****************************************************************************}

constructor TReference.Init(AFileName: PString; ALine, AColumn: Sw_integer);
begin
  inherited Init;
  FileName:=AFileName;
  Position.X:=AColumn;
  Position.Y:=ALine;
end;

function TReference.GetFileName: string;
begin
  GetFileName:=GetStr(FileName);
end;

destructor TReference.Done;
begin
  inherited Done;
end;

constructor TReference.Load(var S: TStream);
begin
  S.Read(Position, SizeOf(Position));

  { --- items needing fixup --- }
  S.Read(FileName, SizeOf(FileName)); { ->ModulesNames^.Item }
end;

procedure TReference.Store(var S: TStream);
begin
  S.Write(Position, SizeOf(Position));

  { --- items needing fixup --- }
  S.Write(FileName, SizeOf(FileName));
end;

{****************************************************************************
                                   TSymbol
****************************************************************************}

constructor TSymbol.Init(const AName: string; ATyp: tsymtyp; AParams: string; AMemInfo: PSymbolMemInfo);
begin
  inherited Init;
  inc(SymbolCount);
  VarSpez:=vs_value;
  VarOptions:=[];
  Name:=NewStr(AName); Typ:=ATyp;
  if AMemInfo<>nil then
    SetMemInfo(AMemInfo^);
  New(References, Init(20,50));
  if ATyp in RecordTypes then
    begin
      Items:=New(PSortedSymbolCollection, Init(50,100));
    end;
end;

procedure TSymbol.SetVarSpez(const AVarSpez : TVarSpez);
begin
  VarSpez:=AVarSpez;
end;

procedure TSymbol.SetVarOptions(const AVarOptions : TVarOptions);
begin
  VarOptions:=AVarOptions;
end;

procedure TSymbol.SetMemInfo(const AMemInfo: TSymbolMemInfo);
begin
  if MemInfo=nil then New(MemInfo);
  Move(AMemInfo,MemInfo^,SizeOf(MemInfo^));
  Flags:=Flags or sfHasMemInfo;
end;

function TSymbol.GetReferenceCount: Sw_integer;
var Count: Sw_integer;
begin
  if References=nil then Count:=0 else
    Count:=References^.Count;
  GetReferenceCount:=Count;
end;

function TSymbol.GetReference(Index: Sw_integer): PReference;
begin
  GetReference:=References^.At(Index);
end;

function TSymbol.GetItemCount: Sw_integer;
var Count: Sw_integer;
begin
  if Items=nil then Count:=0 else
    Count:=Items^.Count;
  GetItemCount:=Count;
end;

function TSymbol.GetItem(Index: Sw_integer): PSymbol;
begin
  GetItem:=Items^.At(Index);
end;

function TSymbol.GetName: string;
begin
  GetName:=GetStr(Name);
end;

function TSymbol.GetText: string;
var S: string;
begin
  S:=GetTypeName;
  if length(S)>SymbolTypLen then
   S:=Copy(S,1,SymbolTypLen)
  else
   begin
     while length(S)<SymbolTypLen do
      S:=S+' ';
   end;
  S:=S+' '+GetName;
  if (Flags and sfRecord)<>0 then
    S:=S+' = record'
  else
  if (Flags and sfObject)<>0 then
    begin
      S:=S+' = ';
      if (Flags and sfClass)<>0 then
        S:=S+'class'
      else
        S:=S+'object';
      if Ancestor<>nil then
        S:=S+'('+Ancestor^.GetName+')';
    end
  else
    begin
      if Assigned(DType) then
        S:=S+' = '+DType^;
      if Assigned(Params) then
        S:=S+'('+Params^+')';
      if Assigned(VType) then
        S:=S+': '+VType^;
    end;
  if Typ=ProcSym then
    S:=S+';';
  GetText:=S;
end;

function TSymbol.GetTypeName: string;
var S: string;
begin
  case Typ of
    abstractsym  : S:='abst';
    fieldvarsym  : S:='member';
    staticvarsym,
    localvarsym,
    paravarsym   :
      begin
        if (vo_is_hidden_para in varoptions) then
          S:='hidden'
        else
          S:='var';
      end;
    typesym      : S:='type';
    procsym      : if VType=nil then
                     S:='proc'
                   else
                     S:='func';
    unitsym      : S:='unit';
    constsym     : S:='const';
    enumsym      : S:='enum';
    errorsym     : S:='error';
    syssym       : S:='sys';
    labelsym     : S:='label';
    absolutevarsym :
      if (vo_is_funcret in varoptions) then
        S:='ret'
      else
        S:='abs';
    propertysym  : S:='prop';
    macrosym     : S:='macro';
  else S:='';
  end;
  GetTypeName:=S;
end;

destructor TSymbol.Done;
begin
  if assigned(MemInfo) then
    Dispose(MemInfo);
  if assigned(References) then
    Dispose(References, Done);
  if assigned(Items) then
    Dispose(Items, Done);
  if assigned(Name) then
    DisposeStr(Name);
{  if assigned(Params) then
    DisposeStr(Params); in TypeNames
  if assigned(VType) then
    DisposeStr(VType);
  if assigned(DType) then
    DisposeStr(DType);
  if assigned(Ancestor) then
    DisposeStr(Ancestor);}
  dec(SymbolCount);
  inherited Done;
end;

constructor TSymbol.Load(var S: TStream);
var MI: TSymbolMemInfo;
    W: word;
begin
  TObject.Init;
  inc(SymbolCount);

  S.Read(Typ,SizeOf(Typ));
  case Typ of
    abstractsym,
    absolutevarsym,
    staticvarsym,
    localvarsym,
    paravarsym :
    begin
      S.Read(VarSpez,SizeOf(VarSpez));
      S.Read(VarOptions,SizeOf(VarOptions));
    end;
  else
    begin
      VarSpez:=vs_value;
      VarOptions:=[];
    end;
  end;
  S.Read(TypeID, SizeOf(TypeID));
  S.Read(RelatedTypeID, SizeOf(RelatedTypeID));
  S.Read(Flags, SizeOf(Flags));
  Name:=S.ReadStr;
  if (Flags and sfHasMemInfo)<>0 then
    begin
      S.Read(MI,SizeOf(MI));
      SetMemInfo(MI);
    end;

  W:=0;
  S.Read(W,SizeOf(W));
  if (W and 1)<>0 then
    New(References, Load(S));
  if (W and 2)<>0 then
    New(Items, Load(S));

  { --- items needing fixup --- }
  S.Read(DType, SizeOf(DType));
  S.Read(VType, SizeOf(VType));
  S.Read(Params, SizeOf(Params));
end;

procedure TSymbol.Store(var S: TStream);
var W: word;
begin
  S.Write(Typ,SizeOf(Typ));
  case Typ of
    abstractsym,
    absolutevarsym,
    staticvarsym,
    localvarsym,
    paravarsym :
    begin
      S.Write(VarSpez,SizeOf(VarSpez));
      S.Write(VarOptions,SizeOf(VarOptions));
    end;
  end;
  S.Write(TypeID, SizeOf(TypeID));
  S.Write(RelatedTypeID, SizeOf(RelatedTypeID));
  S.Write(Flags, SizeOf(Flags));
  S.WriteStr(Name);

  if (Flags and sfHasMemInfo)<>0 then
    S.Write(MemInfo^,SizeOf(MemInfo^));

  W:=0;
  if Assigned(References) then W:=W or 1;
  if Assigned(Items) then W:=W or 2;
  S.Write(W,SizeOf(W));
  if Assigned(References) then References^.Store(S);
  if Assigned(Items) then Items^.Store(S);

  { --- items needing fixup --- }
  S.Write(DType, SizeOf(DType));
  S.Write(VType, SizeOf(VType));
  S.Write(Params, SizeOf(Params));
end;

constructor TExport.Init(const AName: string; AIndex: longint; ASymbol: PSymbol);
begin
  inherited Init;
  Name:=NewStr(AName); Index:=AIndex;
  Symbol:=ASymbol;
end;

function TExport.GetDisplayText: string;
var S: string;
begin
  S:=GetStr(Name)+' '+IntToStr(Index);
  if Assigned(Symbol) and (UpcaseStr(Symbol^.GetName)<>UpcaseStr(GetStr(Name))) then
    S:=S+' ('+Symbol^.GetName+')';
  GetDisplayText:=S;
end;

destructor TExport.Done;
begin
  if Assigned(Name) then DisposeStr(Name);
  inherited Done;
end;

constructor TImport.Init(const ALibName, AFuncName,ARealName: string; AIndex: longint);
begin
  inherited Init;
  LibName:=NewStr(ALibName);
  FuncName:=NewStr(AFuncName); RealName:=NewStr(ARealName);
  Index:=AIndex;
end;

function TImport.GetDisplayText: string;
var S: string;
begin
  S:=GetStr(RealName);
  if Assigned(FuncName) then S:=GetStr(FuncName)+' ('+S+')';
  if S='' then S:=IntToStr(Index);
  S:=GetStr(LibName)+' '+S;
  GetDisplayText:=S;
end;

destructor TImport.Done;
begin
  if Assigned(LibName) then DisposeStr(LibName);
  if Assigned(FuncName) then DisposeStr(FuncName);
  if Assigned(RealName) then DisposeStr(RealName);
  inherited Done;
end;

function TImportCollection.At(Index: sw_Integer): PImport;
begin
  At:=inherited At(Index);
end;

function TImportCollection.Compare(Key1, Key2: Pointer): sw_Integer;
var K1: PImport absolute Key1;
    K2: PImport absolute Key2;
    S1: string;
    S2: string;
    R: sw_integer;
begin
  if (K1^.RealName=nil) and (K2^.RealName<>nil) then R:= 1 else
  if (K1^.RealName<>nil) and (K2^.RealName=nil) then R:=-1 else
  if (K1^.RealName=nil) and (K2^.RealName=nil) then
    begin
      if K1^.Index<K2^.Index then R:=-1 else
      if K1^.Index>K2^.Index then R:= 1 else
      R:=0;
    end
  else
    begin
      if K1^.FuncName=nil then S1:=GetStr(K1^.RealName) else S1:=GetStr(K1^.FuncName);
      if K2^.FuncName=nil then S2:=GetStr(K2^.RealName) else S2:=GetStr(K2^.FuncName);
      S1:=UpcaseStr(S1); S2:=UpcaseStr(S2);
      if S1<S2 then R:=-1 else
      if S1>S2 then R:= 1 else
      R:=0;
    end;
  Compare:=R;
end;

function TExportCollection.At(Index: sw_Integer): PExport;
begin
  At:=inherited At(Index);
end;

function TExportCollection.Compare(Key1, Key2: Pointer): sw_Integer;
var K1: PExport absolute Key1;
    K2: PExport absolute Key2;
    S1: string;
    S2: string;
    R: sw_integer;
begin
  S1:=UpcaseStr(GetStr(K1^.Name)); S2:=UpcaseStr(GetStr(K2^.Name));
  if S1<S2 then R:=-1 else
  if S1>S2 then R:= 1 else
  R:=0;
  Compare:=R;
end;

constructor TModuleSymbol.Init(const AName, AMainSource: string);
begin
  inherited Init(AName,unitsym,'',nil);
  MainSource:=NewStr(AMainSource);
end;

procedure TModuleSymbol.SetLoadedFrom(const AModuleName: string);
begin
  SetStr(LoadedFrom,AModuleName);
end;

procedure TModuleSymbol.AddUsedUnit(P: PSymbol);
begin
  if Assigned(UsedUnits)=false then
    New(UsedUnits, Init(10,10));
  UsedUnits^.Insert(P);
end;

procedure TModuleSymbol.AddDependentUnit(P: PSymbol);
begin
  if Assigned(DependentUnits)=false then
    New(DependentUnits, Init(10,10));
  DependentUnits^.Insert(P);
end;

procedure TModuleSymbol.AddSourceFile(const Path: string);
begin
  if Assigned(SourceFiles)=false then
    New(SourceFiles, Init(10,10));
  sourcefiles^.Insert(NewStr(Path));
end;

destructor TModuleSymbol.Done;
begin
  if Assigned(MainSource) then DisposeStr(MainSource);
  if assigned(Exports_) then
    Dispose(Exports_, Done);
  if Assigned(Imports) then
    Dispose(Imports, Done);
  if Assigned(LoadedFrom) then
    DisposeStr(LoadedFrom);
  if Assigned(UsedUnits) then
  begin
    UsedUnits^.DeleteAll;
    Dispose(UsedUnits, Done);
  end;
  if Assigned(DependentUnits) then
  begin
    DependentUnits^.DeleteAll;
    Dispose(DependentUnits, Done);
  end;
  if Assigned(SourceFiles) then Dispose(SourceFiles, Done);
  inherited Done;
end;


constructor TObjectSymbol.Init(AParent: PObjectSymbol; ASymbol: PSymbol);
begin
  inherited Init;
  Parent:=AParent;
  Symbol:=ASymbol;
end;

constructor TObjectSymbol.InitName(const AName: string);
begin
  inherited Init;
  Name:=NewStr(AName);
end;

function TObjectSymbol.GetName: string;
begin
  if Name<>nil then
    GetName:=Name^
  else
    GetName:=Symbol^.GetName;
end;

function TObjectSymbol.GetDescendantCount: sw_integer;
var Count: sw_integer;
begin
  if Descendants=nil then Count:=0 else
    Count:=Descendants^.Count;
  GetDescendantCount:=Count;
end;

function TObjectSymbol.GetDescendant(Index: sw_integer): PObjectSymbol;
begin
  GetDescendant:=Descendants^.At(Index);
end;

procedure TObjectSymbol.AddDescendant(P: PObjectSymbol);
begin
  if Descendants=nil then
    New(Descendants, Init(50,10));
  Descendants^.Insert(P);
end;

destructor TObjectSymbol.Done;
begin
  if Assigned(Name) then DisposeStr(Name); Name:=nil;
  if Assigned(Descendants) then Dispose(Descendants, Done); Descendants:=nil;
  inherited Done;
end;

constructor TObjectSymbol.Load(var S: TStream);
begin
end;

procedure TObjectSymbol.Store(S: TStream);
begin
end;

{****************************************************************************
                                TSourceFile
****************************************************************************}

constructor TSourceFile.Init(ASourceFileName, AObjFileName, APPUFileName: string);
begin
  inherited Init;
  SourceFileName:=NewStr(ASourceFileName);
  ObjFileName:=NewStr(AObjFileName);
  PPUFileName:=NewStr(APPUFileName);
end;

destructor TSourceFile.Done;
begin
  if assigned(SourceFileName) then DisposeStr(SourceFileName);
  if assigned(ObjFileName) then DisposeStr(ObjFileName);
  if assigned(PPUFileName) then DisposeStr(PPUFileName);
  inherited Done;
end;

function TSourceFile.GetSourceFilename: string;
begin
  GetSourceFilename:=GetStr(SourceFileName);
end;

function TSourceFile.GetObjFileName: string;
begin
  GetObjFilename:=GetStr(ObjFileName);
end;

function TSourceFile.GetPPUFileName: string;
begin
  GetPPUFilename:=GetStr(PPUFileName);
end;

function TSourceFileCollection.At(Index: sw_Integer): PSourceFile;
begin
  At:=inherited At(Index);
end;

{*****************************************************************************
                              Main Routines
*****************************************************************************}

procedure DisposeBrowserCol;
begin
  if assigned(Modules) then
   begin
     dispose(Modules,Done);
     Modules:=nil;
   end;
  if assigned(ModuleNames) then
   begin
     dispose(ModuleNames,Done);
     ModuleNames:=nil;
   end;
  if assigned(TypeNames) then
   begin
     dispose(TypeNames,Done);
     TypeNames:=nil;
   end;
  if assigned(ObjectTree) then
    begin
      Dispose(ObjectTree, Done);
      ObjectTree:=nil;
    end;
end;


procedure NewBrowserCol;
begin
  New(Modules, Init(50,50));
  New(ModuleNames, Init(50,50));
  New(TypeNames, Init(1000,5000));
end;


  procedure ProcessSymTable(OwnerSym: PSymbol; var Owner: PSymbolCollection; Table: TSymTable);
  var I: longint;
      Sym: TSym;
      pd : TProcDef;
      Symbol: PSymbol;
      Reference: PReference;
      inputfile : Tinputfile;
      Ref : TRefItem;
      DefPos : TFilePosInfo;
  procedure SetVType(Symbol: PSymbol; VType: string);
  begin
    Symbol^.VType:=TypeNames^.Add(VType);
  end;
  procedure SetDType(Symbol: PSymbol; DType: string);
  begin
    Symbol^.DType:=TypeNames^.Add(DType);
  end;
  function GetDefinitionStr(def: tdef): string; forward;
  function GetEnumDefStr(def: tenumdef): string;
  var
    Name: string;
    esym: tenumsym;
    i: integer;
  begin
    Name:='(';
    for i := 0 to def.symtable.SymList.Count - 1 do
      begin
        if i>0 then
          Name:=Name+', ';
        Name:=Name+tenumsym(def.symtable.SymList[i]).name;
      end;
    Name:=Name+')';
    GetEnumDefStr:=Name;
  end;
  function GetArrayDefStr(def: tarraydef): string;
  var Name: string;
  begin
    Name:='array ['+IntToStr(def.lowrange)+'..'+IntToStr(def.highrange)+'] of ';
    if assigned(def.elementdef) then
      Name:=Name+GetDefinitionStr(def.elementdef);
    GetArrayDefStr:=Name;
  end;
  function GetFileDefStr(def: tfiledef): string;
  var Name: string;
  begin
    Name:='';
    case def.filetyp of
      ft_text    : Name:='text';
      ft_untyped : Name:='file';
      ft_typed   : Name:='file of '+GetDefinitionStr(def.typedfiledef);
    end;
    GetFileDefStr:=Name;
  end;
  function GetStringDefStr(def: tstringdef): string;
  var Name: string;
  begin
    Name:='';
    case def.stringtype of
      st_shortstring :
        if def.len=255 then
          Name:='shortstring'
        else
          Name:='string['+IntToStr(def.len)+']';
      st_longstring :
        Name:='longstring';
      st_ansistring :
        Name:='ansistring';
      st_widestring :
        Name:='widestring';
      st_unicodestring :
        Name:='unicodestring';
    else ;
    end;
    GetStringDefStr:=Name;
  end;
  function retdefassigned(def: tabstractprocdef): boolean;
  var OK: boolean;
  begin
    OK:=false;
    if assigned(def.returndef) then
      if UpcaseStr(GetDefinitionStr(def.returndef))<>'VOID' then
        OK:=true;
    retdefassigned:=OK;
  end;
  function GetAbsProcParmDefStr(def: tabstractprocdef): string;
  var Name: string;
      dc: tabstractvarsym;
      i,
      Count: integer;
      CurName: string;
  begin
    Name:='';
    Count:=0;
    for i:=0 to def.paras.count-1 do
     begin
       dc:=tabstractvarsym(def.paras[i]);
       if not (vo_is_hidden_para in dc.VarOptions) then
         begin
           CurName:='';
           if assigned(dc.vardef) then
             CurName:=': '+GetDefinitionStr(dc.vardef);
           CurName:=dc.RealName+CurName;
           case dc.varspez of
             vs_Value    : ;
             vs_Const    : CurName:='const '+CurName;
             vs_Var      : CurName:='var '+CurName;
             vs_Out      : CurName:='out '+CurName;
             vs_Constref : CurName:='constref '+CurName;
           end;
           if Count>0 then
             CurName:='; '+CurName;
           Name:=Name+CurName;
           Inc(Count);
         end;
     end;
    GetAbsProcParmDefStr:=Name;
  end;
  function GetAbsProcDefStr(def: tabstractprocdef): string;
  var Name: string;
  begin
    Name:=GetAbsProcParmDefStr(def);
    if Name<>'' then Name:='('+Name+')';
    if retdefassigned(def) then
      Name:='function'+Name+': '+GetDefinitionStr(def.returndef)+';'
    else
      Name:='procedure'+Name+';';
    GetAbsProcDefStr:=Name;
  end;
  function GetProcDefStr(def: tprocdef): string;
  var DName: string;
      {J: integer;}
  begin
{    DName:='';
    if assigned(def) then
    begin
      if assigned(def.parast) then
        begin
          with def.parast^ do
          for J:=1 to number_symbols do
            begin
              if J<>1 then DName:=DName+', ';
              ParSym:=GetsymNr(J);
              if ParSym=nil then Break;
              DName:=DName+ParSym^.Name;
            end;
        end
    end;}
    DName:=GetAbsProcDefStr(def);
    GetProcDefStr:=DName;
  end;
  function GetProcVarDefStr(def: tprocvardef): string;
  begin
    GetProcVarDefStr:=GetAbsProcDefStr(def);
  end;
  function GetSetDefStr(def: tsetdef): string;
  var Name: string;
  begin
    Name:='set of '+GetDefinitionStr(def.elementdef);
    GetSetDefStr:=Name;
  end;
  function GetPointerDefStr(def: tpointerdef): string;
  begin
    GetPointerDefStr:='^'+GetDefinitionStr(def.pointeddef);
  end;
  function GetDefinitionStr(def: tdef): string;
  var Name: string;
  begin
    Name:='';
    if def<>nil then
    begin
      if assigned(def.typesym) then
        Name:=def.typesym.name;
      if Name='' then
      case def.typ of
        arraydef :
          Name:=GetArrayDefStr(tarraydef(def));
        stringdef :
          Name:=GetStringDefStr(tstringdef(def));
        enumdef :
          Name:=GetEnumDefStr(tenumdef(def));
        procdef :
          Name:=GetProcDefStr(tprocdef(def));
        procvardef :
          Name:=GetProcVarDefStr(tprocvardef(def));
        filedef :
          Name:=GetFileDefStr(tfiledef(def));
        setdef :
          Name:=GetSetDefStr(tsetdef(def));
      end;
    end;
    GetDefinitionStr:=Name;
  end;
  function GetEnumItemName(Sym: tenumsym): string;
  var Name: string;
      {ES: tenumsym;}
  begin
    Name:='';
    if assigned(sym) and assigned(sym.definition) then
      if assigned(sym.definition.typesym) then
      begin
{        ES:=sym.definition.First;
        while (ES<>nil) and (ES^.Value<>sym.value) do
          ES:=ES^.next;
        if assigned(es) and (es^.value=sym.value) then
          Name:=}
        Name:=sym.definition.typesym.name;
        if Name<>'' then
          Name:=Name+'('+IntToStr(sym.value)+')';
      end;
    GetEnumItemName:=Name;
  end;
  function GetConstValueName(sym: tconstsym): string;
  var Name: string;
  begin
    Name:='';
    if Name='' then
    case sym.consttyp of
      constord :
        begin
          if sym.constdef.typ=enumdef then
            Name:=sym.constdef.typesym.name+'('+tostr(sym.value.valueord)+')'
          else
            if is_boolean(sym.constdef) then
              Name:='Longbool('+tostr(sym.value.valueord)+')'
          else
            if is_char(sym.constdef) or
               is_widechar(sym.constdef) then
              Name:=''''+chr(sym.value.valueord.svalue)+''''
          else
            Name:=tostr(sym.value.valueord);
        end;
      constresourcestring,
      conststring :
        Name:=''''+StrPas(pchar(sym.value.valueptr))+'''';
      constreal:
        Name:=FloatToStr(PBestReal(sym.value.valueptr)^);
      constset:
{        Name:=SetToStr(pnormalset(sym.value.valueptr)) };
      constnil: ;
    end;
    GetConstValueName:=Name;
  end;
  procedure ProcessDefIfStruct(definition: tdef);
  begin
    { still led to infinite recursions
      only useful for unamed types PM }
    if assigned(definition) and not assigned(definition.typesym) then
    begin
      case definition.typ of
        recorddef :
          if trecorddef(definition).symtable<>Table then
            ProcessSymTable(Symbol,Symbol^.Items,trecorddef(definition).symtable);
        objectdef :
          if tobjectdef(definition).symtable<>Table then
            ProcessSymTable(Symbol,Symbol^.Items,tobjectdef(definition).symtable);
        { leads to infinite loops !!
        pointerdef :
          with tpointerdef(definition)^ do
            if assigned(definition) then
              if assigned(definition.sym) then
                ProcessDefIfStruct(definition.sym.definition);}
      end;
    end;
  end;
  var MemInfo: TSymbolMemInfo;
      ObjDef: tobjectdef;
      symidx : longint;
  begin
    if not Assigned(Table) then
     Exit;
    Symbol:=nil;
    if Owner=nil then
     Owner:=New(PSortedSymbolCollection, Init(10,50));
    for symidx:=0 to Table.SymList.Count-1 do
      begin
        sym:=tsym(Table.SymList[symidx]);
        New(Symbol, Init(Sym.Name,Sym.Typ,'',nil));
        case Sym.Typ of
          staticvarsym,
          localvarsym,
          absolutevarsym,
          paravarsym :
            begin
              Symbol^.SetVarOptions(tabstractvarsym(sym).VarOptions);
              Symbol^.SetVarSpez(tabstractvarsym(sym).VarSpez);
            end;
        end;
        case Sym.Typ of
          staticvarsym,
          localvarsym,
          paravarsym :
             with tabstractvarsym(sym) do
             begin
               if (vo_is_funcret in varoptions) then
                 begin
                   if Assigned(OwnerSym) then
                       if assigned(vardef) then
                         if assigned(vardef.typesym) then
                           SetVType(OwnerSym,vardef.typesym.name)
                         else
                           SetVType(OwnerSym,GetDefinitionStr(vardef));
                 end;
               if assigned(vardef) then
                 if assigned(vardef.typesym) then
                   SetVType(Symbol,vardef.typesym.name)
                 else
                   SetVType(Symbol,GetDefinitionStr(vardef));
               ProcessDefIfStruct(vardef);
               if assigned(vardef) then
                 if (vardef.typ=pointerdef) and
                    assigned(tpointerdef(vardef).pointeddef) then
                 begin
                   Symbol^.Flags:=(Symbol^.Flags or sfPointer);
                   Symbol^.RelatedTypeID:=Ptrint(tpointerdef(vardef).pointeddef);
                 end;
               if tabstractnormalvarsym(sym).localloc.loc=LOC_REFERENCE then
                 MemInfo.Addr:=tabstractnormalvarsym(sym).localloc.reference.offset
               else
                 MemInfo.Addr:=0;
               if assigned(vardef) and (vardef.typ=arraydef) then
                 begin
                   if tarraydef(vardef).highrange<tarraydef(vardef).lowrange then
                     MemInfo.Size:=-1
                   else
                     MemInfo.Size:=getsize;
                 end
               else
                 MemInfo.Size:=getsize;
               { this is not completely correct... }
               if assigned(vardef) then
                 MemInfo.PushSize:=paramanager.push_size(varspez,vardef,pocall_default)
               else
                 begin
                   { This can happen, why? }
                   MemInfo.PushSize:=0;
                 end;
               Symbol^.SetMemInfo(MemInfo);
             end;
          fieldvarsym :
             with tfieldvarsym(sym) do
             begin
               if assigned(vardef) and (vardef.typ=arraydef) then
                 begin
                   if tarraydef(vardef).highrange<tarraydef(vardef).lowrange then
                     MemInfo.Size:=-1
                   else
                     MemInfo.Size:=getsize;
                 end
               else
                 MemInfo.Size:=getsize;
               Symbol^.SetMemInfo(MemInfo);
             end;
          constsym :
             SetDType(Symbol,GetConstValueName(tconstsym(sym)));
          enumsym :
            if assigned(tenumsym(sym).definition) then
             SetDType(Symbol,GetEnumItemName(tenumsym(sym)));
          unitsym :
            begin
  {            ProcessSymTable(Symbol^.Items,tunitsym(sym).unitsymtable);}
            end;
          syssym :
{            if assigned(Table.Name) then
            if Table.Name^='SYSTEM' then}
              begin
                Symbol^.Params:=TypeNames^.Add('...');
              end;
          procsym :
            begin
              for i:=0 to tprocsym(sym).ProcdefList.Count-1 do
                begin
                  if i>0 then
                    begin
                      if Assigned(Symbol) then
                        Owner^.Insert(Symbol);
                      New(Symbol, Init(Sym.Name,Sym.Typ,'',nil));
                    end;
                  with tprocsym(sym) do
                    begin
                      pd:=tprocdef(procdeflist[i]);
                      if assigned(pd) then
                        begin
                          ProcessSymTable(Symbol,Symbol^.Items,pd.parast);
                          if assigned(pd.parast) then
                            begin
                              Symbol^.Params:=TypeNames^.Add(
                                GetAbsProcParmDefStr(pd));
                            end
                          else { param-definition is NOT assigned }
                            if assigned(Table.Name) and
                               (Table.Name^='SYSTEM') then
                            begin
                              Symbol^.Params:=TypeNames^.Add('...');
                            end;
                          if cs_local_browser in current_settings.moduleswitches then
                           begin
                             if assigned(pd.localst) and
                               (pd.localst.symtabletype<>staticsymtable) then
                              ProcessSymTable(Symbol,Symbol^.Items,pd.localst);
                           end;
                        end;
                    end;
                end;
            end;
          typesym :
            begin
            with ttypesym(sym) do
              if assigned(typedef) then
               begin
                Symbol^.TypeID:=Ptrint(typedef);
                case typedef.typ of
                  arraydef :
                    SetDType(Symbol,GetArrayDefStr(tarraydef(typedef)));
                  enumdef :
                    SetDType(Symbol,GetEnumDefStr(tenumdef(typedef)));
                  procdef :
                    SetDType(Symbol,GetProcDefStr(tprocdef(typedef)));
                  procvardef :
                    SetDType(Symbol,GetProcVarDefStr(tprocvardef(typedef)));
                  objectdef :
                    with tobjectdef(typedef) do
                    begin
                      ObjDef:=childof;
                      if ObjDef<>nil then
                        Symbol^.RelatedTypeID:=Ptrint(ObjDef);{TypeNames^.Add(S);}
                      Symbol^.Flags:=(Symbol^.Flags or sfObject);
                      if tobjectdef(typedef).objecttype=odt_class then
                        Symbol^.Flags:=(Symbol^.Flags or sfClass);
                      if tobjectdef(typedef).objecttype=odt_class then
                      if not(df_generic in typedef.defoptions) then
                      ProcessSymTable(Symbol,Symbol^.Items,tobjectdef(typedef).symtable);
                    end;
                  recorddef :
                    begin
                      Symbol^.Flags:=(Symbol^.Flags or sfRecord);
                      ProcessSymTable(Symbol,Symbol^.Items,trecorddef(typedef).symtable);
                    end;
                  pointerdef :
                    begin
                      Symbol^.Flags:=(Symbol^.Flags or sfPointer);
                      Symbol^.RelatedTypeID:=Ptrint(tpointerdef(typedef).pointeddef);{TypeNames^.Add(S);}
                      SetDType(Symbol,GetPointerDefStr(tpointerdef(typedef)));
                    end;

                  filedef :
                    SetDType(Symbol,GetFileDefStr(tfiledef(typedef)));
                  setdef :
                    SetDType(Symbol,GetSetDefStr(tsetdef(typedef)));
                end;
               end;
            end;
        end;
        if assigned(sym) then
          begin
            DefPos:=tstoredsym(sym).FileInfo;
            inputfile:=get_source_file(defpos.moduleindex,defpos.fileindex);
            if Assigned(inputfile) and (inputfile.name<>'') then
              begin
                New(Reference, Init(ModuleNames^.Add(inputfile.name),
                  DefPos.line,DefPos.column));
                Symbol^.References^.Insert(Reference);
              end;
          end;
        if assigned(Symbol) and assigned(sym.RefList) then
          begin
            Ref:=TRefItem(tstoredsym(sym).RefList.First);
            while assigned(Ref) do
              begin
                inputfile:=get_source_file(ref.refinfo.moduleindex,ref.refinfo.fileindex);
                if Assigned(inputfile) and (inputfile.name<>'') then
                  begin
                    New(Reference, Init(ModuleNames^.Add(inputfile.name),
                      ref.refinfo.line,ref.refinfo.column));
                    Symbol^.References^.Insert(Reference);
                  end;
                Ref:=TRefItem(Ref.next);
              end;
          end;
        if Assigned(Symbol) then
          begin
            (* if not Owner^.Search(Symbol,J) then *)
              Owner^.Insert(Symbol)
            (*else
              begin
                Dispose(Symbol,done);
                Symbol:=nil;
              end;*)
          end;
      end;
  end;

function SearchModule(const Name: string): PModuleSymbol;
function Match(P: PModuleSymbol): boolean;
begin
  Match:=CompareText(P^.GetName,Name)=0;
end;
var P: PModuleSymbol;
begin
  P:=nil;
  if Assigned(Modules) then
    P:=Modules^.FirstThat(TCallbackFunBoolParam(@Match));
  SearchModule:=P;
end;

procedure CreateBrowserCol;
var
  path,module,
  name,msource : string;

  T: TSymTable;
  UnitS,PM: PModuleSymbol;
  hp : tmodule;
  puu: tused_unit;
  pdu: tdependent_unit;
  pif: tinputfile;
begin
  DisposeBrowserCol;
  if (cs_browser in current_settings.moduleswitches) then
    NewBrowserCol;
  hp:=tmodule(loaded_units.first);
  if (cs_browser in current_settings.moduleswitches) then
   while assigned(hp) do
    begin
       if hp.is_unit then
         t:=tsymtable(hp.globalsymtable)
       else
         t:=tsymtable(hp.localsymtable);
       if assigned(t) then
         begin
           name:=GetStr(T.Name);
           msource:=hp.mainsource;
           New(UnitS, Init(Name,msource));
           if Assigned(hp.loaded_from) then
             if assigned(hp.loaded_from.globalsymtable) then
               UnitS^.SetLoadedFrom(tsymtable(hp.loaded_from.globalsymtable).name^);
{           pimportlist(current_module^.imports^.first);}

           if assigned(hp.sourcefiles) then
           begin
             pif:=hp.sourcefiles.files;
             while (pif<>nil) do
             begin
               path:=pif.path;
               name:=pif.name;
               UnitS^.AddSourceFile(path+name);
               pif:=pif.next;
             end;
           end;

           Modules^.Insert(UnitS);
           ProcessSymTable(UnitS,UnitS^.Items,T);
           if hp.is_unit then
           if cs_local_browser in current_settings.moduleswitches then
             begin
                t:=tsymtable(hp.localsymtable);
                if assigned(t) then
                  ProcessSymTable(UnitS,UnitS^.Items,T);
             end;
         end;
       hp:=tmodule(hp.next);
    end;

  hp:=tmodule(loaded_units.first);
  if (cs_browser in current_settings.moduleswitches) then
   while assigned(hp) do
    begin
       t:=tsymtable(hp.globalsymtable);
       if assigned(t) then
         begin
           name:=GetStr(T.Name);
           UnitS:=SearchModule(Name);
           puu:=tused_unit(hp.used_units.first);
           while (puu<>nil) do
           begin
             module:=GetStr(puu.u.modulename);
             PM:=SearchModule(module);
             if Assigned(PM) then
               UnitS^.AddUsedUnit(PM);
             puu:=tused_unit(puu.next);
           end;
           pdu:=tdependent_unit(hp.dependent_units.first);
           while (pdu<>nil) do
           begin
             name:=GetStr(tsymtable(pdu.u.globalsymtable).name);
             PM:=SearchModule(Name);
             if Assigned(PM) then
               UnitS^.AddDependentUnit(PM);
             pdu:=tdependent_unit(pdu.next);
           end;
         end;
       hp:=tmodule(hp.next);
    end;

  if (cs_browser in current_settings.moduleswitches) then
    BuildObjectInfo;
  { can allways be done
    needed to know when recompilation of sources is necessary }
  BuildSourceList;
end;

procedure BuildObjectInfo;
var C,D: PIDSortedSymbolCollection;
    E : PCollection;
    ObjectC: PObjectSymbolCollection;
    ObjectsSymbol: PObjectSymbol;
procedure InsertSymbolCollection(Symbols: PSymbolCollection);
var I: sw_integer;
    P: PSymbol;
begin
  for I:=0 to Symbols^.Count-1 do
    begin
      P:=Symbols^.At(I);
      if (P^.Flags and (sfObject or sfClass))<>0 then
        C^.Insert(P);
      if (P^.typ=typesym) then
        D^.Insert(P);
      if (P^.typ in [staticvarsym,localvarsym,paravarsym]) and ((P^.flags and sfPointer)<>0) then
        E^.Insert(P);
      if P^.Items<>nil then
        InsertSymbolCollection(P^.Items);
    end;
end;
function SearchObjectForSym(O: PSymbol): PObjectSymbol;
var I: sw_integer;
    OS,P: PObjectSymbol;
begin
  P:=nil;
  for I:=0 to ObjectC^.Count-1 do
    begin
      OS:=ObjectC^.At(I);
      if OS^.Symbol=O then
        begin P:=OS; Break; end;
    end;
  SearchObjectForSym:=P;
end;
procedure BuildTree;
var I: sw_integer;
    Symbol: PSymbol;
    Parent,OS: PObjectSymbol;
begin
  I:=0;
  while (I<C^.Count) do
    begin
      Symbol:=C^.At(I);
      if Symbol^.Ancestor=nil then
        Parent:=ObjectsSymbol
      else
        Parent:=SearchObjectForSym(Symbol^.Ancestor);
      if Parent<>nil then
        begin
          New(OS, Init(Parent, Symbol));
          Parent^.AddDescendant(OS);
          ObjectC^.Insert(OS);
          C^.AtDelete(I);
        end
      else
        Inc(I);
    end;
end;
var Pass: integer;
    I: sw_integer;
    P: PSymbol;
begin
  New(C, Init(1000,5000));
  New(D, Init(1000,5000));
  New(E, Init(1000,5000));
  InsertSymbolCollection(Modules);

  { --- Resolve ancestor<->descendant references --- }
  for I:=0 to C^.Count-1 do
    begin
      P:=C^.At(I);
      if P^.RelatedTypeID<>0 then
        P^.Ancestor:=C^.SearchSymbolByID(P^.RelatedTypeID);
    end;

  { --- Resolve pointer definition references --- }
  for I:=0 to D^.Count-1 do
    begin
      P:=D^.At(I);
      if P^.RelatedTypeID<>0 then
        P^.Ancestor:=D^.SearchSymbolByID(P^.RelatedTypeID);
    end;

  { --- Resolve  pointer var definition references --- }
  for I:=0 to E^.Count-1 do
    begin
      P:=PSymbol(E^.At(I));
      if P^.RelatedTypeID<>0 then
        P^.Ancestor:=D^.SearchSymbolByID(P^.RelatedTypeID);
    end;

  { E is not needed anymore }
  E^.DeleteAll;
  Dispose(E,Done);

  { D is not needed anymore }
  D^.DeleteAll;
  Dispose(D,Done);

  { --- Build object tree --- }
  if assigned(ObjectTree) then
    Dispose(ObjectTree, Done);
  New(ObjectsSymbol, InitName('Objects'));
  ObjectTree:=ObjectsSymbol;

  New(ObjectC, Init(C^.Count,100));

  Pass:=0;
  if C^.Count>0 then
  repeat
    BuildTree;
    Inc(Pass);
  until (C^.Count=0) or (Pass>20); { more than 20 levels ? - then there must be a bug }

  ObjectC^.DeleteAll; Dispose(ObjectC, Done);
  C^.DeleteAll; Dispose(C, Done);
end;

function SearchObjectForSymbol(O: PSymbol): PObjectSymbol;
function ScanObjectCollection(Parent: PObjectSymbol): PObjectSymbol;
var I: sw_integer;
    OS,P: PObjectSymbol;
    ObjectC: PObjectSymbolCollection;
begin
  P:=nil;
  if Parent<>nil then
  if Parent^.Descendants<>nil then
  begin
    ObjectC:=Parent^.Descendants;
    for I:=0 to ObjectC^.Count-1 do
      begin
        OS:=ObjectC^.At(I);
        if OS^.Symbol=O then
          begin P:=OS; Break; end;
        if OS^.Descendants<>nil then
          begin
            P:=ScanObjectCollection(OS);
            if P<>nil then Break;
          end;
      end;
  end;
  ScanObjectCollection:=P;
end;
begin
  SearchObjectForSymbol:=ScanObjectCollection(ObjectTree);
end;

procedure BuildSourceList;
var m: tmodule;
    s: tinputfile;
    ppu,obj: string;
    source: string;
begin
  if Assigned(SourceFiles) then
    begin
      Dispose(SourceFiles, Done);
      SourceFiles:=nil;
    end;
  if assigned(loaded_units.first) then
  begin
    New(SourceFiles, Init(50,10));
    m:=tmodule(loaded_units.first);
    while assigned(m) do
    begin
      obj:=ExpandFileName(m.objfilename);
      ppu:=''; source:='';
      if m.is_unit then
        ppu:=ExpandFileName(m.ppufilename);
      if (m.is_unit=false) and (m.islibrary=false) then
        ppu:=ExpandFileName(m.exefilename);
      if assigned(m.sourcefiles) then
        begin
          s:=m.sourcefiles.files;
          while assigned(s) do
          begin
            source:=s.path+s.name;
            source:=ExpandFileName(source);

            sourcefiles^.Insert(New(PSourceFile, Init(source,obj,ppu)));
            s:=s.ref_next;
          end;
        end;
      m:=tmodule(m.next);
    end;
  end;
end;

{*****************************************************************************
                                 Initialize
*****************************************************************************}



var
  oldexit : pointer;

procedure browcol_exit;
begin
  exitproc:=oldexit;
  DisposeBrowserCol;
  if Assigned(SourceFiles) then
    begin
      Dispose(SourceFiles, Done);
      SourceFiles:=nil;
    end;
  if assigned(ObjectTree) then
    begin
      Dispose(ObjectTree, Done);
      ObjectTree:=nil;
    end;
end;


procedure InitBrowserCol;
begin
end;


procedure DoneBrowserCol;
begin
  { nothing, the collections are freed in the exitproc - ??? }
  { nothing? then why do we've this routine for ? IMHO, either we should
    remove this, or it should destroy the browser info when it's called. - BG }
end;

type
     PPointerXRef = ^TPointerXRef;
     TPointerXRef = record
       PtrValue : pointer;
       DataPtr  : pointer;
     end;

     PPointerDictionary = ^TPointerDictionary;
     TPointerDictionary = object(TSortedCollection)
       function  At(Index: sw_Integer): PPointerXRef;
       function  Compare(Key1, Key2: Pointer): sw_Integer; virtual;
       procedure FreeItem(Item: Pointer); virtual;
       function  SearchXRef(PtrValue: pointer): PPointerXRef;
       function  AddPtr(PtrValue, DataPtr: pointer): PPointerXRef;
       procedure Resolve(var P);
     end;

function NewPointerXRef(APtrValue, ADataPtr: pointer): PPointerXRef;
var P: PPointerXRef;
begin
  New(P); FillChar(P^,SizeOf(P^),0);
  with P^ do begin PtrValue:=APtrValue; DataPtr:=ADataPtr; end;
  NewPointerXRef:=P;
end;

procedure DisposePointerXRef(P: PPointerXRef);
begin
  if Assigned(P) then Dispose(P);
end;

function TPointerDictionary.At(Index: sw_Integer): PPointerXRef;
begin
  At:=inherited At(Index);
end;

function TPointerDictionary.Compare(Key1, Key2: Pointer): sw_Integer;
var K1: PPointerXRef absolute Key1;
    K2: PPointerXRef absolute Key2;
    R: integer;
begin
  if PtrUInt(K1^.PtrValue)<PtrUInt(K2^.PtrValue) then R:=-1 else
  if PtrUInt(K1^.PtrValue)>PtrUInt(K2^.PtrValue) then R:= 1 else
  R:=0;
  Compare:=R;
end;

procedure TPointerDictionary.FreeItem(Item: Pointer);
begin
  if Assigned(Item) then DisposePointerXRef(Item);
end;

function TPointerDictionary.SearchXRef(PtrValue: pointer): PPointerXRef;
var P: PPointerXRef;
    T: TPointerXRef;
    Index: sw_integer;
begin
  T.PtrValue:=PtrValue;
  if Search(@T,Index)=false then P:=nil else
    P:=At(Index);
  SearchXRef:=P;
end;

function TPointerDictionary.AddPtr(PtrValue, DataPtr: pointer): PPointerXRef;
var P: PPointerXRef;
begin
  P:=SearchXRef(PtrValue);
  if P=nil then
    begin
      P:=NewPointerXRef(PtrValue,DataPtr);
      Insert(P);
{$ifdef DEBUG}
    end
  else
    begin
      if P^.DataPtr<>DataPtr then
        InternalError(987654);
{$endif DEBUG}
    end;
  AddPtr:=P;
end;

procedure TPointerDictionary.Resolve(var P);
var X: PPointerXRef;
    V: pointer;
begin
  Move(P,V,SizeOf(V));
  X:=SearchXRef(V);
  if X=nil then V:=nil else
    V:=X^.DataPtr;
  Move(V,P,SizeOf(V));
end;

procedure ReadPointers(S: PStream; C: PCollection; D: PPointerDictionary);
var W,I: sw_integer;
    P: pointer;
begin
  S^.Read(W,SizeOf(W));
  for I:=0 to W-1 do
  begin
    S^.Read(P,SizeOf(P));
    D^.AddPtr(P,C^.At(I));
  end;
end;

function LoadBrowserCol(S: PStream): boolean;
var PD: PPointerDictionary;
procedure FixupPointers;
procedure FixupReference(P: PReference);
begin
  PD^.Resolve(P^.FileName);
end;
procedure FixupSymbol(P: PSymbol);
var I: sw_integer;
begin
  PD^.Resolve(P^.DType);
  PD^.Resolve(P^.VType);
  PD^.Resolve(P^.Params);
  if Assigned(P^.References) then
    with P^.References^ do
     for I:=0 to Count-1 do
       FixupReference(At(I));
  if Assigned(P^.Items) then
    with P^.Items^ do
     for I:=0 to Count-1 do
       FixupSymbol(At(I));
end;
begin
  Modules^.ForEach(TCallbackProcParam(@FixupSymbol));
end;
procedure ReadSymbolPointers(P: PSymbol);
var I: sw_integer;
    PV: pointer;
begin
  S^.Read(PV, SizeOf(PV));
  PD^.AddPtr(PV,P);
  if Assigned(P^.Items) then
    with P^.Items^ do
     for I:=0 to Count-1 do
       ReadSymbolPointers(At(I));
end;
begin
  DisposeBrowserCol;

  New(ModuleNames, Load(S^));
  New(TypeNames, Load(S^));
  New(Modules, Load(S^));

  New(PD, Init(4000,1000));
  ReadPointers(S,ModuleNames,PD);
  ReadPointers(S,TypeNames,PD);
  ReadPointers(S,Modules,PD);
  Modules^.ForEach(TCallbackProcParam(@ReadSymbolPointers));
  FixupPointers;
  Dispose(PD, Done);

  BuildObjectInfo;
  LoadBrowserCol:=(S^.Status=stOK);
end;

procedure StorePointers(S: PStream; C: PCollection);
var W,I: sw_integer;
    P: pointer;
begin
  W:=C^.Count;
  S^.Write(W,SizeOf(W));
  for I:=0 to W-1 do
  begin
    P:=C^.At(I);
    S^.Write(P,SizeOf(P));
  end;
end;

function StoreBrowserCol(S: PStream) : boolean;
procedure WriteSymbolPointers(P: PSymbol);
var I: sw_integer;
begin
  S^.Write(P, SizeOf(P));
  if Assigned(P^.Items) then
    with P^.Items^ do
     for I:=0 to Count-1 do
       WriteSymbolPointers(At(I));
end;
begin
  ModuleNames^.Store(S^);
  TypeNames^.Store(S^);
  Modules^.Store(S^);

  StorePointers(S,ModuleNames);
  StorePointers(S,TypeNames);
  StorePointers(S,Modules);
  Modules^.ForEach(TCallbackProcParam(@WriteSymbolPointers));
  StoreBrowserCol:=(S^.Status=stOK);
end;

procedure RegisterSymbols;
begin
  RegisterType(RModuleNameCollection);
  RegisterType(RTypeNameCollection);
  RegisterType(RReference);
  RegisterType(RSymbol);
  RegisterType(RObjectSymbol);
  RegisterType(RSymbolCollection);
  RegisterType(RSortedSymbolCollection);
  RegisterType(RIDSortedSymbolCollection);
  RegisterType(RObjectSymbolCollection);
  RegisterType(RReferenceCollection);
  RegisterType(RModuleSymbol);
end;

begin
  oldexit:=exitproc;
  exitproc:=@browcol_exit;
end.
