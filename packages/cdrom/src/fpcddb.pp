{
    Copyright (c) 2008 by Michael Van Canneyt

    Unit to parse CDDB responses and construct a list
    of tracks in a CD.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD

{
  Some notes:

  Disc.Year and Disc.Genre only have values if proto = 5 or above as specified in the request.
  With protocol 5 and under the responses are in ISO-8859-1. In version 6 it's UTF-8

  A more complete explanation of the protocol can be found here:
  http://ftp.freedb.org/pub/freedb/latest/CDDBPROTO


}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
unit fpcddb;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils; 

Type
  TCDDisk = Class;

  { TCDTrack }

  TCDTrack = Class(TCollectionItem)
  private
    FDuration: TDateTime;
    FExtra: String;
    FPerformer: String;
    FTitle: String;
    function GetPerformer: String;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    Property Title : String Read FTitle Write FTitle;
    Property Performer : String Read GetPerformer Write FPerformer;
    Property Extra : String Read FExtra Write FExtra;
    Property Duration : TDateTime Read FDuration Write FDuration;
  end;

  { TCDTracks }

  TCDTracks = Class(TCollection)
  private
    FCDDisk: TCDDisk;
    function GetT(AIndex : Integer): TCDTrack;
    procedure SetT(AIndex : Integer; const AValue: TCDTrack);
  Public
    Property CDDisk : TCDDisk Read FCDDisk;
    Function AddTrack(Const ATitle,AExtra : String; ADuration : TDateTime) : TCDTrack;
    Function AddTrack(Const ATitle,AExtra : String) : TCDTrack;
    Function AddTrack(Const ATitle : String) : TCDTrack;
    Property Track[AIndex : Integer] : TCDTrack Read GetT Write SetT; default;
  end;


  { TCDDisk }

  TCDDisk = Class(TCollectionItem)
  private
    FDiskID: Integer;
    FExtra: String;
<<<<<<< HEAD
<<<<<<< HEAD
    FGenre: String;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FPerformer: String;
    FPlayOrder: String;
    FTitle: String;
    FTracks: TCDTracks;
    FYear: Word;
    function GetDiskID: String;
    procedure SetDiskID(const AValue: String);
    procedure SetTracks(const AValue: TCDTracks);
  Protected
    Function CreateTracks : TCDTracks; virtual;
  Public
    Constructor Create(ADiskID : Integer);
    Constructor Create(ACollection : TCollection); override;
    Procedure Assign(Source : TPersistent); override;
    Property IntDiscID : Integer Read FDiskID Write FDiskID;
  Published
    Property PlayOrder : String Read FPlayOrder Write FPlayOrder;
<<<<<<< HEAD
<<<<<<< HEAD
    Property Year : Word Read FYear Write FYear; // proto=5
    Property Title : String Read FTitle Write FTitle;
    Property Performer : String Read FPerformer Write FPerformer;
    Property Genre : String Read FGenre write FGenre; //proto=5
=======
    Property Year : Word Read FYear Write FYear;
    Property Title : String Read FTitle Write FTitle;
    Property Performer : String Read FPerformer Write FPerformer;
>>>>>>> graemeg/fixes_2_2
=======
    Property Year : Word Read FYear Write FYear;
    Property Title : String Read FTitle Write FTitle;
    Property Performer : String Read FPerformer Write FPerformer;
>>>>>>> origin/fixes_2_2
    Property Extra : String Read FExtra Write FExtra;
    Property DiscID : String Read GetDiskID Write SetDiskID;
    property Tracks : TCDTracks Read FTracks Write SetTracks;
  end;

  { TCDDisks }

  TCDDisks = Class(TCollection)
  private
    function GetD(AIndex : Integer): TCDDisk;
    procedure SetD(AIndex : Integer; const AValue: TCDDisk);
  Public
    Function AddDisk(ADiscID : String) : TCDDisk;
    Function AddDisk : TCDDisk;
    Property Disk[AIndex : Integer] : TCDDisk Read GetD Write SetD; default;
  end;

  { TCDDBQueryMatch }
  TCDDBQueryMatch = Class(TCollectionItem)
  private
    FCategory: String;
    FDiscID: Integer;
    FPerformer: String;
    FTitle: String;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    Property DiscID : Integer Read FDiscID Write FDiscID;
    Property Category : String Read FCategory Write FCategory;
    Property Title : String Read FTitle Write FTitle;
    Property Performer : String Read FPerformer Write FPerformer;
  end;

  { TCDDBQueryMatches }

  TCDDBQueryMatches = Class(TCollection)
  private
    function GetM(AIndex : Integer): TCDDBQueryMatch;
    procedure SetM(AIndex : Integer; const AValue: TCDDBQueryMatch);
  Public
    Function AddMatch(Const ADiscID: Integer; Const ACategory,ATitle, APerformer : String) : TCDDBQueryMatch;
    Function AddMatch(Const ADiscID,ACategory,ATitle, APerformer : String) : TCDDBQueryMatch;
    Function AddMatch : TCDDBQueryMatch;
    Property Match[AIndex : Integer] :TCDDBQueryMatch Read GetM Write SetM; default;
  end;
  { TCDDBParser }

  TCDDBParser = Class(TComponent)
  private
    FDisks: TCDDisks;
    FDisk : TCDDisk;
    function ParseExtraDiskData(AData: String): Boolean;
    function ParseExtraTrackData(ATrack: TCDTrack; AData: String): Boolean;
    procedure SetDisks(const AValue: TCDDisks);
    procedure SplitQueryResponse(AResponse: String; var ACategory, ADiscID, ATitle, APerformer: String);
    procedure SplitTitle(const ALine: String; var AArtist, ATitle: String;
      PreferTitle: boolean);
    function StdReplacements(S: String): String;
  Protected
    Procedure CheckDisk;
    function CheckCDDBCmdResult(var S: String): Integer;
    Function CreateDisks :TCDDisks; virtual;
    Function IsComment(Const L : String) : Boolean;
    Function GetTrack(Const TrackNo : Integer) : TCDTrack;
    Property Disk : TCDDisk Read FDisk;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Function ParseCDDBReadResponse(Response : TStrings; WithHeader : Boolean = True) : Integer;
    Function ParseCDDBReadResponse(Response : TStream; WithHeader : Boolean = True) : Integer;
    Function ParseCDDBQueryResponse(Response : TStrings; Matches : TCDDBQueryMatches; WithHeader : Boolean = True) : Integer;
    Function ParseCDDBQueryResponse(Response : TStream; Matches : TCDDBQueryMatches; WithHeader : Boolean = True) : Integer;
  Published
    Property Disks : TCDDisks Read FDisks Write SetDisks;
  end;

  ECDDBParser = Class(Exception);

Function DiscIDToStr(ID : Integer) : String;
Function StrToDiscID(S : String) : Integer;

implementation

Resourcestring
  SErrNoDisk         = 'No disk active';
  SErrInvalidTrackNo = 'Invalid track number: %d';
  SErrParsingLine    = 'An error occured while parsing line %d of the response: %s';
  SErrCDDBResponse   = 'CDDB error in command response: %s';

function DiscIDToStr(ID: Integer): String;
begin
  Result:=LowerCase(Format('%.8x',[ID]));
end;

function StrToDiscID(S: String): Integer;
begin
  Result:=StrToIntDef('$'+S,-1);
end;

{ TCDTrack }

function TCDTrack.GetPerformer: String;
begin
  Result:=FPerformer;
  If (Result='') and Assigned(Collection) and (Collection is TCDTracks) then
    If Assigned(TCDTracks(Collection).CDDisk) then
      Result:=TCDTracks(Collection).CDDisk.Performer;
end;

procedure TCDTrack.Assign(Source: TPersistent);

Var
  T : TCDTrack;

begin
  if (Source is TCDTrack) then
    begin
    T:=Source as TCDTrack;
    FTitle:=T.FTitle;
    FExtra:=T.FExtra;
    FPerformer:=T.FPerformer;
    FDuration:=T.FDuration;
    end
  else
    inherited Assign(Source);
end;

{ TCDDisk }

procedure TCDDisk.SetTracks(const AValue: TCDTracks);
begin
  if FTracks=AValue then exit;
  FTracks.Assign(AValue);
end;

function TCDDisk.GetDiskID: String;
begin
  Result:=DiscIDToStr(FdiskID);
end;

procedure TCDDisk.SetDiskID(const AValue: String);
begin
  FDiskID:=StrToDiscID(AValue);
end;

function TCDDisk.CreateTracks: TCDTracks;
begin
 Result:=TCDTracks.Create(TCDTrack);
end;

constructor TCDDisk.Create(ADiskID: Integer);
begin
  FDiskID:=ADiskID;
  Create(Nil);
end;

constructor TCDDisk.Create(ACollection: TCollection);
begin
  FTracks:=CreateTracks;
  FTracks.FCDDisk:=Self;
  inherited Create(ACollection);
end;

procedure TCDDisk.Assign(Source: TPersistent);

Var
  D : TCDDisk;

begin
  if Source is TCDDisk then
    begin
    D:=Source as TCDDisk;
    FTitle:=D.FTitle;
    FExtra:=D.FExtra;
    FPerformer:=D.FPerformer;
    FYear:=D.FYear;
    FTracks.Assign(D.FTracks);
    FPLayOrder:=D.FPlayOrder;
    end
  else
    inherited Assign(Source);
end;

{ TCDTracks }

function TCDTracks.GetT(AIndex : Integer): TCDTrack;
begin
  Result:=Items[AIndex] as TCDTrack;
end;

procedure TCDTracks.SetT(AIndex : Integer; const AValue: TCDTrack);
begin
  Items[AIndex]:=AValue;
end;

function TCDTracks.AddTrack(const ATitle, AExtra: String; ADuration: TDateTime
  ): TCDTrack;
begin
  Result:=Add as TCDTrack;
  Result.Title:=ATitle;
  Result.Extra:=AExtra;
  Result.Duration:=ADuration;
end;

function TCDTracks.AddTrack(const ATitle, AExtra: String): TCDTrack;
begin
  Result:=AddTrack(ATitle,AExtra,0);
end;

function TCDTracks.AddTrack(const ATitle: String): TCDTrack;
begin
  Result:=AddTrack(ATitle,'',0);
end;

{ TCDDisks }

function TCDDisks.GetD(AIndex : Integer): TCDDisk;
begin
  Result:=Items[AIndex] as TCDDisk;
end;

procedure TCDDisks.SetD(AIndex : Integer; const AValue: TCDDisk);
begin
  Items[AIndex]:=AValue;
end;

function TCDDisks.AddDisk(ADiscID: String): TCDDisk;
begin
  Result:=Self.AddDisk();
  Result.DiscID:=ADiscID;
end;

function TCDDisks.AddDisk: TCDDisk;
begin
  Result:=Add as TCDDisk;
end;

{ TCDDBParser }

procedure TCDDBParser.SetDisks(const AValue: TCDDisks);
begin
  if FDisks=AValue then exit;
  FDisks.Assign(AValue);
end;

procedure TCDDBParser.CheckDisk;
begin
  If (FDisk=Nil) then
    Raise ECDDBParser.Create(SErrNoDisk)
end;

function TCDDBParser.CreateDisks: TCDDisks;
begin
  Result:=TCDDisks.Create(TCDDisk);
end;

function TCDDBParser.IsComment(const L: String): Boolean;
begin
  Result:=(Length(L)=0) or (L[1]='#');
end;

function TCDDBParser.GetTrack(const TrackNo: Integer): TCDTrack;
begin
  If (TrackNo<0) then
    Raise ECDDBParser.CreateFmt(SErrInvalidTrackNo,[TrackNo]);
  CheckDisk;
  If (TrackNo>FDisk.Tracks.Count) then
    Raise ECDDBParser.CreateFmt(SErrInvalidTrackNo,[TrackNo]);
  If (TrackNo=FDisk.Tracks.Count) then
    Result:=FDisk.Tracks.AddTrack('')
  else
    Result:=FDisk.Tracks[TrackNo]
end;

constructor TCDDBParser.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FDisks:=CreateDisks;
end;

destructor TCDDBParser.Destroy;
begin
  FreeAndNil(FDisks);
  inherited Destroy;
end;

Function TCDDBParser.StdReplacements(S : String) : String;

begin
  Result:=StringReplace(S,'\n',sLineBreak,[rfReplaceAll]);
end;

Function TCDDBParser.ParseExtraDiskData(AData : String) : Boolean;

begin
  FDisk.Extra:=FDisk.Extra+StdReplacements(AData);
end;

Function TCDDBParser.ParseExtraTrackData(ATrack : TCDTrack; AData : String) : Boolean;

begin
  ATrack.Extra:=ATrack.Extra+StdReplacements(AData);
end;

Procedure TCDDBParser.SplitTitle(Const ALine: String; Var AArtist, ATitle : String; PreferTitle : boolean);

Var
  P,L : Integer;

begin
  // Artist / Title
  L:=Length(ALine);
  P:=Pos('/',ALine);
  If (P=0) and Not PreferTitle then
    P:=L+1;
  AArtist:=Trim(Copy(ALine,1,P-1));
  ATitle:=Trim(Copy(ALine,P+1,L-P));
end;

Function TCDDBParser.ParseCDDBReadResponse(Response: TStrings; WithHeader : Boolean = True) : Integer;

Var
  I,P : Integer;
  L,Args,A,T : String;
  TrackID : Integer;
  Track : TCDTrack;

begin
  Result:=-1;
  FDisks.Clear;
  If WithHeader and (Response.Count>0) then
    begin
    L:=Response[0];
    If Not (CheckCDDBCmdResult(L) in [200,210]) then
      Raise ECDDBParser.CreateFmt(SErrCDDBResponse,[L]);
    end;
  FDisk:=Nil;
  Result:=0;
  Try
    Try
      I:=Ord(WithHeader);
      While (I<Response.Count) do
        begin
        L:=Response[i];
        If Not IsComment(L) then
          begin
          P:=Pos('=',L);
          Args:=Copy(L,P+1,Length(L)-P);
          L:=Uppercase(Copy(L,1,P-1));
          If (L='DISCID') then
            FDisk:=FDisks.AddDisk(Args)
          else
            begin
            CheckDisk;
            If (L='DTITLE') then
              begin
              SplitTitle(Args,A,T,True);
              FDisk.Title:=T;
              FDisk.Performer:=A;
              end
<<<<<<< HEAD
<<<<<<< HEAD
            else if (L='DYEAR') then
              begin
              FDisk.Year:=StrToIntDef(Trim(Args),0);
              end
            else if (L='DGENRE') then
              begin
              FDisk.Genre:=Trim(Args);
              end
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            else if (L='EXTD') then
              ParseExtraDiskData(Args)
            else if (Copy(L,1,6)='TTITLE') then
              begin
              Delete(L,1,6);
              TrackID:=StrToIntDef(L,-1);
              Track:=GetTrack(TrackID);
              SplitTitle(Args,A,T,True);
              Track.Title:=T;
              Track.Performer:=A;
              end
            else if (Copy(L,1,6)='EXTT') then
              begin
              Delete(L,1,6);
              TrackID:=StrToIntDef(L,-1);
              Track:=GetTrack(TrackID);
              ParseExtraTrackData(Track,Args);
              end
            else if (Copy(L,1,9)='PLAYORDER') then
              begin
              FDisk.PlayOrder:=Trim(Args);
              end;
            end;
          end;
        Inc(I);
        end;
    except
      On E : Exception do
        begin
        E.Message:=Format(SErrParsingLine,[I,E.MEssage]);
        Raise;
        end;
    end;
    Result:=FDisks.Count;
  Finally
    FDisk:=Nil;
  end;
end;

Function TCDDBParser.ParseCDDBReadResponse(Response: TStream; WithHeader : Boolean = True) : Integer;

Var
  L : TStringList;

begin
  L:=TStringList.Create;
  try
    L.LoadFromStream(Response);
    Result:=ParseCDDBReadResponse(L,WithHeader);
  finally
    L.Free;
  end;
end;

function TCDDBParser.ParseCDDBQueryResponse(Response: TStrings;
  Matches: TCDDBQueryMatches; WithHeader: Boolean): Integer;

Var
  I,CmdRes : Integer;
  L : String;
  D,C,T,P : String;

begin
  Matches.Clear;
  Result:=-1;
  If WithHeader and (Response.Count>0) then
    begin
    L:=Response[0];
    CmdRes:=CheckCDDBCmdResult(L);
    If (CmdRes=200) then
      begin
      SplitQueryResponse(L,C,D,T,P);
      Matches.AddMatch(D,C,T,P);
      Result:=1;
      Exit;
      end
<<<<<<< HEAD
<<<<<<< HEAD
    else if not (CmdRes in [210,211]) then
      Raise ECDDBParser.CreateFmt(SerrCDDBResponse,[L]);
    end;
  For I:=Ord(WithHeader or (CMDRes=211)) to Response.Count-1 do
    If (Response[i]<>'.') then
      begin
      SplitQueryResponse(Response[i],C,D,T,P);
      Matches.AddMatch(D,C,T,P);
      end;
=======
=======
>>>>>>> origin/fixes_2_2
    else if (CmdRes<>210) then
      Raise ECDDBParser.CreateFmt(SerrCDDBResponse,[L]);
    end;
  For I:=Ord(WithHeader) to Response.Count-1 do
    begin
    SplitQueryResponse(Response[i],C,D,T,P);
    Matches.AddMatch(D,C,T,P);
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result:=Matches.Count;
end;

function TCDDBParser.ParseCDDBQueryResponse(Response: TStream;
  Matches: TCDDBQueryMatches; WithHeader: Boolean): Integer;

Var
  L : TStringList;

begin
  L:=TStringList.Create;
  try
    L.LoadFromStream(Response);
    Result:=ParseCDDBQueryResponse(L,Matches,WithHeader);
  finally
    L.Free;
  end;
end;

Function TCDDBParser.CheckCDDBCmdResult(Var S : String) : Integer;

Var
  P : integer;

begin
  P:=Pos(' ',S);
  If (P=0) then
    P:=Length(S)+1;
  Result:=StrToIntDef(Copy(S,1,P-1),0);
  Delete(S,1,P);
end;

Procedure TCDDBParser.SplitQueryResponse(AResponse :String; Var ACategory, ADiscID, ATitle, APerformer : String);

Var
  P : Integer;

begin
  P:=Pos(' ',AResponse);
  ACategory:=Copy(AResponse,1,P-1);
  Delete(AResponse,1,P);
  P:=Pos(' ',AResponse);
  ADiscId:=Copy(AResponse,1,P-1);
  Delete(AResponse,1,P);
  SplitTitle(AResponse,APerformer,ATitle,True);
end;

{ TCDDBQueryMatches }

function TCDDBQueryMatches.GetM(AIndex : Integer): TCDDBQueryMatch;
begin
  Result:=TCDDBQueryMatch(Items[AIndex]);
end;

procedure TCDDBQueryMatches.SetM(AIndex : Integer; const AValue: TCDDBQueryMatch
  );
begin
  Items[AIndex]:=AValue;
end;

function TCDDBQueryMatches.AddMatch(const ADiscID: Integer; const ACategory,
  ATitle, APerformer: String): TCDDBQueryMatch;
begin
  Result:=AddMatch();
  Result.DiscID:=ADiscID;
  Result.Category:=ACategory;
  Result.Title:=ATitle;
  Result.Performer:=APerformer;
end;

function TCDDBQueryMatches.AddMatch(const ADiscID, ACategory, ATitle, APerformer : String): TCDDBQueryMatch;

begin
  Result:=AddMatch(StrToDiscID(ADiscID),ACategory,ATitle,APerformer);
end;

function TCDDBQueryMatches.AddMatch: TCDDBQueryMatch;
begin
  Result:=Add as TCDDBQueryMatch;
end;

{ TCDDBQueryMatch }

procedure TCDDBQueryMatch.Assign(Source: TPersistent);

Var
  M : TCDDBQueryMatch;

begin
  if Source is TCDDBQueryMatch then
    begin
    M:=Source as TCDDBQueryMatch;
    FDiscID:=M.FDiscID;
    FCategory:=M.FCategory;
    FPerformer:=M.FPerformer;
    FTitle:=M.FTitle;
    end
  else
    inherited Assign(Source);
end;

end.

