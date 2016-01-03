{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************}
{*                             TPersistent                                  *}
{****************************************************************************}

procedure TPersistent.AssignError(Source: TPersistent);

Var SourceName : String;

begin
  If Source<>Nil then
    SourceName:=Source.ClassName
  else
    SourceName:='Nil';
  raise EConvertError.CreateFmt (SAssignError,[SourceName,ClassName]);
end;



procedure TPersistent.AssignTo(Dest: TPersistent);


begin
  Dest.AssignError(Self);
end;


procedure TPersistent.DefineProperties(Filer: TFiler);

begin
end;


function  TPersistent.GetOwner: TPersistent;

begin
  Result:=Nil;
end;

destructor TPersistent.Destroy;
begin
  If Assigned(FObservers) then
    begin
    FPONotifyObservers(Self,ooFree,Nil);
    FreeAndNil(FObservers);
    end;
  inherited Destroy;
end;

procedure TPersistent.FPOAttachObserver(AObserver: TObject);
Var
   I : IFPObserver;

begin
   If Not AObserver.GetInterface(SGUIDObserver,I) then
     Raise EObserver.CreateFmt(SErrNotObserver,[AObserver.ClassName]);
   If not Assigned(FObservers) then
     FObservers:=TFPList.Create;
   FObservers.Add(I);
end;

procedure TPersistent.FPODetachObserver(AObserver: TObject);
Var
  I : IFPObserver;

begin
  If Not AObserver.GetInterface(SGUIDObserver,I) then
    Raise EObserver.CreateFmt(SErrNotObserver,[AObserver.ClassName]);
  If Assigned(FObservers) then
    begin
    FObservers.Remove(I);
    If (FObservers.Count=0) then
      FreeAndNil(FObservers);
    end;
end;

procedure TPersistent.FPONotifyObservers(ASender: TObject;
  AOperation: TFPObservedOperation; Data : Pointer);
Var
  I : Integer;
  Obs : IFPObserver;

begin
  If Assigned(FObservers) then
    For I:=FObservers.Count-1 downto 0 do
      begin
      Obs:=IFPObserver(FObservers[i]);
      Obs.FPOObservedChanged(Self,AOperation,Data);
      end;
end;

procedure TPersistent.Assign(Source: TPersistent);

begin
  If Source<>Nil then
    Source.AssignTo(Self)
  else
    AssignError(Nil);
end;

function  TPersistent.GetNamePath: string;

Var OwnerName :String;
    TheOwner: TPersistent;

begin
 Result:=ClassName;
 TheOwner:=GetOwner;
 If TheOwner<>Nil then
   begin
   OwnerName:=TheOwner.GetNamePath;
   If OwnerName<>'' then Result:=OwnerName+'.'+Result;
   end;
end;


{****************************************************************************}
{*                          TInterfacedPersistent                           *}
{****************************************************************************}

procedure TInterfacedPersistent.AfterConstruction;

Var TheOwner: TPersistent;

begin
  inherited;
  TheOwner:=GetOwner;
  if assigned(TheOwner) then
    TheOwner.GetInterface(IUnknown,FOwnerInterface);
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TInterfacedPersistent._AddRef: Longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
=======
function TInterfacedPersistent._AddRef: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TInterfacedPersistent._AddRef: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TInterfacedPersistent._AddRef: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TInterfacedPersistent._AddRef: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> origin/cpstrnew
begin
  if assigned(FOwnerInterface) then
    Result:=FOwnerInterface._AddRef
  else
    Result:=-1;
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TInterfacedPersistent._Release: Longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
=======
function TInterfacedPersistent._Release: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TInterfacedPersistent._Release: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TInterfacedPersistent._Release: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TInterfacedPersistent._Release: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> origin/cpstrnew
begin
  if assigned(FOwnerInterface) then
    Result:=FOwnerInterface._Release
  else
    Result:=-1;
end;


function TInterfacedPersistent.QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} IID: TGUID; out Obj): HResult;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
begin
  if GetInterface(IID, Obj) then
    Result:=0
  else
    Result:=HResult($80004002);
end;


{****************************************************************************}
{*                                TRecall                                   *}
{****************************************************************************}

constructor TRecall.Create(AStorage,AReference: TPersistent);
begin
  inherited Create;
  FStorage:=AStorage;
  FReference:=AReference;
  Store;
end;


destructor TRecall.Destroy;
begin
  if Assigned(FReference) then
   FReference.Assign(FStorage);
  Forget;
  inherited;
end;


procedure TRecall.Forget;
begin
  FReference:=nil;
  FreeAndNil(FStorage);
end;


procedure TRecall.Store;
begin
  if Assigned(FStorage) and Assigned(FReference) then
    FStorage.Assign(FReference);
end;


