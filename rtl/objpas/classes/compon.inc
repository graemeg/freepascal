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
{*                        TComponentEnumerator                              *}
{****************************************************************************}

constructor TComponentEnumerator.Create(AComponent: TComponent);
begin
  inherited Create;
  FComponent := AComponent;
  FPosition := -1;
end;

function TComponentEnumerator.GetCurrent: TComponent;
begin
  Result := FComponent.Components[FPosition];
end;

function TComponentEnumerator.MoveNext: Boolean;
begin
  Inc(FPosition);
  Result := FPosition < FComponent.ComponentCount;
end;


{****************************************************************************}
{*                             TComponent                                   *}
{****************************************************************************}

function TComponent.GetComObject: IUnknown;
begin
  { Check if VCLComObject is not assigned - we need to create it by    }
  { the call to CreateVCLComObject routine. If in the end we are still }
  { have no valid VCLComObject pointer we need to raise an exception   }
  if not Assigned(VCLComObject) then
    begin
      if Assigned(CreateVCLComObjectProc) then
        CreateVCLComObjectProc(Self);
      if not Assigned(VCLComObject) then
        raise EComponentError.CreateFmt(SNoComSupport,[Name]);
    end;
  { VCLComObject is IVCComObject but we need to return IUnknown }
  IVCLComObject(VCLComObject).QueryInterface(IUnknown, Result);
end;

Function  TComponent.GetComponent(AIndex: Integer): TComponent;

begin
  If not assigned(FComponents) then
    Result:=Nil
  else
    Result:=TComponent(FComponents.Items[Aindex]);
end;

function TComponent.IsImplementorOf (const Intf:IInterface):boolean;
var ref : IInterfaceComponentReference;
begin
 result:=assigned(intf) and supports(intf,IInterfaceComponentReference,ref);
 if result then
   result:=ref.getcomponent=self;
end;

procedure TComponent.ReferenceInterface(const intf:IInterface;op:TOperation);
var ref : IInterfaceComponentReference;
    comp : TComponent;
begin
 if assigned(intf) and supports(intf,IInterfaceComponentReference,ref) then
   begin
    comp:=ref.getcomponent;
<<<<<<< HEAD
<<<<<<< HEAD
    if op = opInsert then
      comp.FreeNotification(Self)
    else
      comp.RemoveFreeNotification(Self); 
=======
    comp.notification(self,op); 
>>>>>>> graemeg/fixes_2_2
=======
    comp.notification(self,op); 
>>>>>>> origin/fixes_2_2
   end;
end;

Function  TComponent.GetComponentCount: Integer;

begin
  If not assigned(FComponents) then
    result:=0
  else
    Result:=FComponents.Count;
end;


Function  TComponent.GetComponentIndex: Integer;

begin
  If Assigned(FOwner) and Assigned(FOwner.FComponents) then
    Result:=FOWner.FComponents.IndexOf(Self)
  else
    Result:=-1;
end;


Procedure TComponent.Insert(AComponent: TComponent);

begin
  If not assigned(FComponents) then
    FComponents:=TFpList.Create;
  FComponents.Add(AComponent);
  AComponent.FOwner:=Self;
end;


Procedure TComponent.ReadLeft(Reader: TReader);

begin
  LongRec(FDesignInfo).Lo:=Reader.ReadInteger;
end;


Procedure TComponent.ReadTop(Reader: TReader);

begin
  LongRec(FDesignInfo).Hi:=Reader.ReadInteger;
end;


Procedure TComponent.Remove(AComponent: TComponent);

begin
  AComponent.FOwner:=Nil;
  If assigned(FCOmponents) then
    begin
    FComponents.Remove(AComponent);
    IF FComponents.Count=0 then
      begin
      FComponents.Free;
      FComponents:=Nil;
      end;
    end;
end;


Procedure TComponent.RemoveNotification(AComponent: TComponent);

begin
  if FFreeNotifies<>nil then
    begin
    FFreeNotifies.Remove(AComponent);
    if FFreeNotifies.Count=0 then
      begin
      FFreeNotifies.Free;
      FFreeNotifies:=nil;
      Exclude(FComponentState,csFreeNotification);
      end;
    end;
end;


Procedure TComponent.SetComponentIndex(Value: Integer);

Var Temp,Count : longint;

begin
  If Not assigned(Fowner) then exit;
  Temp:=getcomponentindex;
  If temp<0 then exit;
  If value<0 then value:=0;
  Count:=Fowner.FComponents.Count;
  If Value>=Count then value:=count-1;
  If Value<>Temp then
    begin
    FOWner.FComponents.Delete(Temp);
    FOwner.FComponents.Insert(Value,Self);
    end;
end;


Procedure TComponent.SetReference(Enable: Boolean);

var
  Field: ^TComponent;
begin
  if Assigned(Owner) then
  begin
    Field := Owner.FieldAddress(Name);
    if Assigned(Field) then
      if Enable then
        Field^ := Self
      else
        Field^ := nil;
  end;
end;


Procedure TComponent.WriteLeft(Writer: TWriter);

begin
  Writer.WriteInteger(LongRec(FDesignInfo).Lo);
end;


Procedure TComponent.WriteTop(Writer: TWriter);

begin
  Writer.WriteInteger(LongRec(FDesignInfo).Hi);
end;


Procedure TComponent.ChangeName(const NewName: TComponentName);

begin
  FName:=NewName;
end;


Procedure TComponent.DefineProperties(Filer: TFiler);

Var Ancestor : TComponent;
    Temp : longint;

begin
  Temp:=0;
  Ancestor:=TComponent(Filer.Ancestor);
  If Assigned(Ancestor) then Temp:=Ancestor.FDesignInfo;
  Filer.Defineproperty('left',@readleft,@writeleft,
                       (longrec(FDesignInfo).Lo<>Longrec(temp).Lo));
  Filer.Defineproperty('top',@readtop,@writetop,
                       (longrec(FDesignInfo).Hi<>Longrec(temp).Hi));
end;


Procedure TComponent.GetChildren(Proc: TGetChildProc; Root: TComponent);

begin
  // Does nothing.
end;


Function  TComponent.GetChildOwner: TComponent;

begin
 Result:=Nil;
end;


Function  TComponent.GetChildParent: TComponent;

begin
  Result:=Self;
end;


Function  TComponent.GetEnumerator: TComponentEnumerator;

begin
  Result:=TComponentEnumerator.Create(Self);
end;

Function  TComponent.GetNamePath: string;

begin
  Result:=FName;
end;


Function  TComponent.GetOwner: TPersistent;

begin
  Result:=FOwner;
end;


Procedure TComponent.Loaded;

begin
  Exclude(FComponentState,csLoading);
end;

Procedure TComponent.Loading;

begin
  Include(FComponentState,csLoading);
end;


Procedure TComponent.Notification(AComponent: TComponent;
  Operation: TOperation);

Var Runner : Longint;

begin
  If (Operation=opRemove) and Assigned(FFreeNotifies) then
    begin
    FFreeNotifies.Remove(AComponent);
            If FFreeNotifies.Count=0 then
      begin
      FFreeNotifies.Free;
      FFreenotifies:=Nil;
      end;
    end;
  If assigned(FComponents) then
    For Runner:=0 To FComponents.Count-1 do
      TComponent(FComponents.Items[Runner]).Notification(AComponent,Operation);
end;


procedure TComponent.PaletteCreated;
  begin
  end;


Procedure TComponent.ReadState(Reader: TReader);

begin
  Reader.ReadData(Self);
end;


Procedure TComponent.SetAncestor(Value: Boolean);

Var Runner : Longint;

begin
  If Value then
    Include(FComponentState,csAncestor)
  else
    Exclude(FCOmponentState,csAncestor);
  if Assigned(FComponents) then
    For Runner:=0 To FComponents.Count-1 do
      TComponent(FComponents.Items[Runner]).SetAncestor(Value);
end;


Procedure TComponent.SetDesigning(Value: Boolean; SetChildren : Boolean = True);

Var Runner : Longint;

begin
  If Value then
    Include(FComponentState,csDesigning)
  else
    Exclude(FComponentState,csDesigning);
  if Assigned(FComponents) and SetChildren then
    For Runner:=0 To FComponents.Count - 1 do
      TComponent(FComponents.items[Runner]).SetDesigning(Value);
end;

Procedure TComponent.SetDesignInstance(Value: Boolean);

begin
  If Value then
    Include(FComponentState,csDesignInstance)
  else
    Exclude(FComponentState,csDesignInstance);
end;

Procedure TComponent.SetInline(Value: Boolean);

begin
  If Value then
    Include(FComponentState,csInline)
  else
    Exclude(FComponentState,csInline);
end;


Procedure TComponent.SetName(const NewName: TComponentName);

begin
  If FName=NewName then exit;
  If (NewName<>'') and not IsValidIdent(NewName) then
    Raise EComponentError.CreateFmt(SInvalidName,[NewName]);
  If Assigned(FOwner) Then
    FOwner.ValidateRename(Self,FName,NewName)
  else
    ValidateRename(Nil,FName,NewName);
  SetReference(False);
  ChangeName(NewName);
  Setreference(True);
end;


Procedure TComponent.SetChildOrder(Child: TComponent; Order: Integer);

begin
  // does nothing
end;


Procedure TComponent.SetParentComponent(Value: TComponent);

begin
  // Does nothing
end;


Procedure TComponent.Updating;

begin
  Include (FComponentState,csUpdating);
end;


Procedure TComponent.Updated;

begin
  Exclude(FComponentState,csUpdating);
end;


class Procedure TComponent.UpdateRegistry(Register: Boolean; const ClassID, ProgID: string);

begin
  // For compatibility only.
end;


Procedure TComponent.ValidateRename(AComponent: TComponent;
  const CurName, NewName: string);

begin
//!! This contradicts the Delphi manual.
  If (AComponent<>Nil) and (CompareText(CurName,NewName)<>0) and (AComponent.Owner = Self) and
     (FindComponent(NewName)<>Nil) then
      raise EComponentError.Createfmt(SDuplicateName,[newname]);
  If (csDesigning in FComponentState) and (FOwner<>Nil) then
    FOwner.ValidateRename(AComponent,Curname,Newname);
end;


Procedure TComponent.ValidateContainer(AComponent: TComponent);

begin
  AComponent.ValidateInsert(Self);
end;


Procedure TComponent.ValidateInsert(AComponent: TComponent);

begin
  // Does nothing.
end;


Procedure TComponent.WriteState(Writer: TWriter);

begin
  Writer.WriteComponentData(Self);
end;


Constructor TComponent.Create(AOwner: TComponent);

begin
  FComponentStyle:=[csInheritable];
  If Assigned(AOwner) then AOwner.InsertComponent(Self);
end;


Destructor TComponent.Destroy;

Var
  I : Integer;
  C : TComponent;

begin
  Destroying;
  If Assigned(FFreeNotifies) then
    begin
    I:=FFreeNotifies.Count-1;
    While (I>=0) do
      begin
      C:=TComponent(FFreeNotifies.Items[I]);
      // Delete, so one component is not notified twice, if it is owned.
      FFreeNotifies.Delete(I);
      C.Notification (self,opRemove);
      If (FFreeNotifies=Nil) then
        I:=0
      else if (I>FFreeNotifies.Count) then
        I:=FFreeNotifies.Count;
      dec(i);
      end;
    FreeAndNil(FFreeNotifies);
    end;
  DestroyComponents;
  If FOwner<>Nil Then FOwner.RemoveComponent(Self);
  inherited destroy;
end;


Procedure TComponent.BeforeDestruction;
begin
  if not(csDestroying in FComponentstate) then
    Destroying;
end;


Procedure TComponent.DestroyComponents;

Var acomponent: TComponent;

begin
  While assigned(FComponents) do
    begin
    aComponent:=TComponent(FComponents.Last);
    Remove(aComponent);
    Acomponent.Destroy;
    end;
end;


Procedure TComponent.Destroying;

Var Runner : longint;

begin
  If csDestroying in FComponentstate Then Exit;
  include (FComponentState,csDestroying);
  If Assigned(FComponents) then
    for Runner:=0 to FComponents.Count-1 do
      TComponent(FComponents.Items[Runner]).Destroying;
end;


function TComponent.ExecuteAction(Action: TBasicAction): Boolean;
begin
  if Action.HandlesTarget(Self) then
   begin
     Action.ExecuteTarget(Self);
     Result := True;
   end
  else
   Result := False;
end;


Function  TComponent.FindComponent(const AName: string): TComponent;

Var I : longint;

begin
  Result:=Nil;
  If (AName='') or Not assigned(FComponents) then exit;
  For i:=0 to FComponents.Count-1 do
    if (CompareText(TComponent(FComponents[I]).Name,AName)=0) then
      begin
      Result:=TComponent(FComponents.Items[I]);
      exit;
      end;
end;


Procedure TComponent.FreeNotification(AComponent: TComponent);

begin
  If (Owner<>Nil) and (AComponent=Owner) then exit;
  If not (Assigned(FFreeNotifies)) then
    FFreeNotifies:=TFpList.Create;
  If FFreeNotifies.IndexOf(AComponent)=-1 then
    begin
<<<<<<< HEAD
    FFreeNotifies.Add(AComponent);
    AComponent.FreeNotification (self);
=======
    If not (Assigned(FFreeNotifies)) then
      FFreeNotifies:=TFpList.Create;
    If FFreeNotifies.IndexOf(AComponent)=-1 then
      begin
      FFreeNotifies.Add(AComponent);
      AComponent.FreeNotification (self);
      end;
>>>>>>> graemeg/cpstrnew
    end;
end;


procedure TComponent.RemoveFreeNotification(AComponent: TComponent);
begin
  RemoveNotification(AComponent);
  AComponent.RemoveNotification (self);
end;


Procedure TComponent.FreeOnRelease;
begin
  if Assigned(VCLComObject) then
    IVCLComObject(VCLComObject).FreeOnRelease;
end;


Function  TComponent.GetParentComponent: TComponent;

begin
  Result:=Nil;
end;


Function  TComponent.HasParent: Boolean;

begin
  Result:=False;
end;


Procedure TComponent.InsertComponent(AComponent: TComponent);

begin
  AComponent.ValidateContainer(Self);
  ValidateRename(AComponent,'',AComponent.FName);
  Insert(AComponent);
  AComponent.SetReference(True);
  If csDesigning in FComponentState then
    AComponent.SetDesigning(true);
  Notification(AComponent,opInsert);
end;


Procedure TComponent.RemoveComponent(AComponent: TComponent);

begin
  Notification(AComponent,opRemove);
  AComponent.SetReference(False);
  Remove(AComponent);
  Acomponent.Setdesigning(False);
  ValidateRename(AComponent,AComponent.FName,'');
end;


Function  TComponent.SafeCallException(ExceptObject: TObject;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ExceptAddr: CodePointer): HResult;
=======
  ExceptAddr: Pointer): HResult;
>>>>>>> graemeg/cpstrnew
=======
  ExceptAddr: Pointer): HResult;
>>>>>>> graemeg/cpstrnew
=======
  ExceptAddr: Pointer): HResult;
>>>>>>> graemeg/cpstrnew
=======
  ExceptAddr: Pointer): HResult;
>>>>>>> origin/cpstrnew
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).SafeCallException(ExceptObject, ExceptAddr)
  else
    Result := inherited SafeCallException(ExceptObject, ExceptAddr);
end;

procedure TComponent.SetSubComponent(ASubComponent: Boolean);
begin
  if ASubComponent then
    Include(FComponentStyle, csSubComponent)
  else
    Exclude(FComponentStyle, csSubComponent);
end;


function TComponent.UpdateAction(Action: TBasicAction): Boolean;
begin
  if Action.HandlesTarget(Self) then
    begin
      Action.UpdateTarget(Self);
      Result := True;
    end
  else
    Result := False;
end;

function TComponent.QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} IID: TGUID; out Obj): HResult;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).QueryInterface(IID, Obj)
  else
  if GetInterface(IID, Obj) then
    Result := S_OK
  else
    Result := E_NOINTERFACE;
<<<<<<< HEAD
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TComponent._AddRef: Longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
=======
function TComponent._AddRef: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TComponent._AddRef: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TComponent._AddRef: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject)._AddRef
  else
    Result := -1;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TComponent._Release: Longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
=======
function TComponent._Release: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TComponent._Release: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
=======
function TComponent._Release: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
>>>>>>> graemeg/cpstrnew
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject)._Release
  else
    Result := -1;
end;

function TComponent.iicrGetComponent: TComponent;

begin
  result:=self;
end;

function TComponent.GetTypeInfoCount(out Count: Integer): HResult; stdcall;
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).GetTypeInfoCount(Count)
  else
    Result := E_NOTIMPL;
end;

<<<<<<< HEAD
function TComponent.GetTypeInfo(Index, LocaleID: Integer; out TypeInfo): HResult; stdcall;
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).GetTypeInfo(Index, LocaleID, TypeInfo)
  else
    Result := E_NOTIMPL;
=======
end;

function TComponent._AddRef: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject)._AddRef
  else
    Result := -1;
end;

function TComponent._Release: Integer;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject)._Release
  else
    Result := -1;
>>>>>>> origin/cpstrnew
end;

function TComponent.GetIDsOfNames(const IID: TGUID; Names: Pointer; NameCount,
  LocaleID: Integer; DispIDs: Pointer): HResult; stdcall;
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).GetIDsOfNames(IID, Names, NameCount, LocaleID, DispIDs)
  else
    Result := E_NOTIMPL;
end;

function TComponent.Invoke(DispID: Integer; const IID: TGUID;
  LocaleID: Integer; Flags: Word; var Params; VarResult, ExcepInfo,
  ArgErr: Pointer): HResult; stdcall;
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).Invoke(DispID, IID, LocaleID, Flags, Params,
      VarResult, ExcepInfo, ArgErr)
  else
    Result := E_NOTIMPL;
end;

function TComponent.GetTypeInfoCount(out Count: Integer): HResult; stdcall;
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).GetTypeInfoCount(Count)
  else
    Result := E_NOTIMPL;
end;

function TComponent.GetTypeInfo(Index, LocaleID: Integer; out TypeInfo): HResult; stdcall;
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).GetTypeInfo(Index, LocaleID, TypeInfo)
  else
    Result := E_NOTIMPL;
end;

<<<<<<< HEAD
function TComponent.GetIDsOfNames(const IID: TGUID; Names: Pointer; NameCount,
  LocaleID: Integer; DispIDs: Pointer): HResult; stdcall;
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).GetIDsOfNames(IID, Names, NameCount, LocaleID, DispIDs)
  else
    Result := E_NOTIMPL;
end;

function TComponent.Invoke(DispID: Integer; const IID: TGUID;
  LocaleID: Integer; Flags: Word; var Params; VarResult, ExcepInfo,
  ArgErr: Pointer): HResult; stdcall;
begin
  if Assigned(VCLComObject) then
    Result := IVCLComObject(VCLComObject).Invoke(DispID, IID, LocaleID, Flags, Params,
      VarResult, ExcepInfo, ArgErr)
  else
    Result := E_NOTIMPL;
=======
function TComponent.iicrGetComponent: TComponent;

begin
  result:=self;
>>>>>>> graemeg/fixes_2_2
=======
function TComponent.iicrGetComponent: TComponent;

begin
  result:=self;
>>>>>>> origin/fixes_2_2
end;
