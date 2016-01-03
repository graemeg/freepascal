{$mode objfpc}
{$h+}

unit xmlreg;

Interface

uses
  sysutils,classes,dom,xmlread,xmlwrite;

Type

  TDataType = (dtUnknown,dtDWORD,dtString,dtBinary);
  TDataInfo = record
    DataType : TDataType;
    DataSize : Integer;
  end;

  TKeyInfo = record
    SubKeys,
    SubKeyLen,
    Values,
    ValueLen,
    DataLen   : Integer;
    FTime     : TDateTime;
  end;


  { TXmlRegistry }

  TXmlRegistry = Class(TObject)
  Private
    FAutoFlush,
    FDirty : Boolean;
    FFileName : String;
    FRootKey : String;
    FDocument : TXMLDocument;
    FCurrentElement : TDomElement;
    FCurrentKey : String;
    Procedure SetFileName(Value : String);
  Protected
    Procedure LoadFromStream(S : TStream);
    Function  NormalizeKey(KeyPath : String) : String;
    Procedure CreateEmptyDoc;
    Function  FindKey (S : String) : TDomElement;
    Function  FindSubKey (S : String; N : TDomElement) : TDomElement;
    Function  CreateSubKey (S : String; N : TDomElement) : TDomElement;
    Function  FindValueKey (S : String) : TDomElement;
    Function  CreateValueKey (S : String) : TDomElement;
    Function  BufToHex(Const Buf; Len : Integer) : String;
    Function  hexToBuf(Const Str : String; Var Buf; Var Len : Integer ) : Integer;
    Procedure MaybeFlush;
    Property  Document : TXMLDocument Read FDocument;
    Property  Dirty : Boolean Read FDirty write FDirty;
  Public
    Constructor Create(AFileName : String);
    Destructor  Destroy;override;
    Function  SetKey(KeyPath : String; AllowCreate : Boolean) : Boolean ;
    Procedure SetRootKey(Value : String);
    Function  DeleteKey(KeyPath : String) : Boolean;
    Function  CreateKey(KeyPath : String) : Boolean;
    Function  GetValueSize(Name : String) : Integer;
    Function  GetValueType(Name : String) : TDataType;
    Function  GetValueInfo(Name : String; Var Info : TDataInfo) : Boolean;
    Function  GetKeyInfo(Var Info : TKeyInfo) : Boolean;
    Function  EnumSubKeys(List : TStrings) : Integer;
    Function  EnumValues(List : TStrings) : Integer;
    Function  KeyExists(KeyPath : String) : Boolean;
    Function  ValueExists(ValueName : String) : Boolean;
    Function  RenameValue(Const OldName,NewName : String) : Boolean;
    Function  DeleteValue(S : String) : Boolean;
    Procedure Flush;
    Procedure Load;
    Function GetValueData(Name : String; Var DataType : TDataType; Var Data; Var DataSize : Integer) : Boolean;
    Function SetValueData(Name : String; DataType : TDataType; Const Data; DataSize : Integer) : Boolean;
    Property FileName : String Read FFileName Write SetFileName;
    Property RootKey : String Read FRootKey Write SetRootkey;
    Property AutoFlush : Boolean Read FAutoFlush Write FAutoFlush;
  end;

// used Key types

Const
  SXmlReg = 'XMLReg';
  SKey    = 'Key';
  SValue  = 'Value';
  SName   = 'Name';
  SType   = 'Type';
  SData   = 'Data';

Implementation


Constructor TXmlRegistry.Create(AFileName : String);

begin
  FFileName:=AFileName;
  FautoFlush:=True;
  If (AFileName<>'') then
    Load
  else
    CreateEmptyDoc;
end;

destructor TXmlRegistry.Destroy;
begin
  if Assigned(FDocument) then  FDocument.Free;
  inherited Destroy;
end;

Procedure TXmlRegistry.SetFileName(Value : String);

begin
  If Value<>FFileName then
    begin
    FFilename:=Value;
    Flush;
    end;
end;

Procedure TXmlRegistry.CreateEmptyDoc;

Const
  template = '<?xml version="1.0" encoding="ISO8859-1"?>'+LineEnding+
             '<'+SXMLReg+'>'+LineEnding+
             '</'+SXMLReg+'>'+LineEnding;

Var
  S : TStream;

begin
  S:=TStringStream.Create(Template);
  S.Seek(0,soFromBeginning);
  Try
    LoadFromStream(S);
  Finally
    S.Free;
  end;
end;

Function TXmlRegistry.NormalizeKey(KeyPath : String) : String;

Var
  L : Integer;

begin
  Result:=StringReplace(KeyPath,'\','/',[rfReplaceAll]);
  L:=Length(Result);
  If (L>0) and (Result[L]<>'/') then
    Result:=Result+'/';
  If (L>0) and (Result[1]<>'/') then
    Result:='/' + Result;
end;

Function TXmlRegistry.SetKey(KeyPath : String; AllowCreate : Boolean) : boolean;

Var
  SubKey,ResultKey : String;
  P : Integer;
  Node,Node2 : TDomElement;

begin
  Result:=(Length(KeyPath)>0);
  If Not Result then
    Exit;
  If (KeyPath[1] in ['/','\']) then
    FCurrentElement:=Nil;
  KeyPath:=NormalizeKey(KeyPath);
  If (FCurrentElement<>nil) then
    begin
    Delete(Keypath,1,1);
    Node:=FCurrentElement;
    Resultkey:=FCurrentKey;
    end
  else
    begin
    Delete(Keypath,1,1);
    Node:=FDocument.DocumentElement;
    If (FRootKey<>'') then
      KeyPath:=FRootKey+KeyPath;
    ResultKey:='';
    end;
  Result:=True;
  repeat
    P:=Pos('/',KeyPath);
    If (P<>0) then
      begin
      SubKey:=Copy(KeyPath,1,P-1);
      Delete(KeyPath,1,P);
      Node2:=FindSubKey(SubKey,Node);
      Result:=(Node2<>Nil);
      If Result then
        Node:=Node2
      else
        begin
        If AllowCreate then
          Begin
          Node2:=CreateSubKey(SubKey,Node);
          Result:=Node2<>Nil;
          If Result Then
            Node:=Node2;
          end;
        end;
      If Result then
        ResultKey:=ResultKey+SubKey+'/';
      end;
  Until (Not Result) or (Length(KeyPath)=0);
  If Result then
    begin
    FCurrentkey:=ResultKey;
    FCurrentElement:=Node;
    end;
  MaybeFlush;
end;

Procedure TXmlRegistry.SetRootKey(Value : String);

begin
  FRootKey:=NormalizeKey(Value);
  If (Length(FRootKey)>1) and (FRootKey[1]='/') then
    Delete(FRootKey,1,1);
  FCurrentKey:='';
  FCurrentElement:=Nil;
end;

Function TXmlRegistry.DeleteKey(KeyPath : String) : Boolean;

Var
  N : TDomElement;

begin
 N:=FindKey(KeyPath);
 Result:=(N<>Nil);
 If Result then
   begin
   (N.ParentNode as TDomElement).RemoveChild(N);
   FDirty:=True;
   MaybeFlush;
   end;
end;

Function TXmlRegistry.CreateKey(KeyPath : String) : Boolean;

Var
  SubKey : String;
  P : Integer;
  Node,Node2 : TDomElement;

begin
  Result:=(Length(KeyPath)>0);
  If Not Result then
    Exit;
  KeyPath:=NormalizeKey(KeyPath);
  If (FCurrentElement<>nil) then
  begin
    Delete(Keypath,1,1);
    Node:=FCurrentElement;
  end
  else
    begin
    Delete(Keypath,1,1);
    Node:=FDocument.DocumentElement;
    If (FRootKey<>'') then
      KeyPath:=FRootKey+KeyPath;
    end;
  Result:=True;
  repeat
    P:=Pos('/',KeyPath);
    If (P<>0) then
      begin
      SubKey:=Copy(KeyPath,1,P-1);
      Delete(KeyPath,1,P);
      Node2:=FindSubKey(SubKey,Node);
      Result:=(Node2<>Nil);
      If Result then
        Node:=Node2
      else
        begin
        Node2:=CreateSubKey(SubKey,Node);
        Result:=Node2<>Nil;
        Node:=Node2
        end;
      end;
  Until (Not Result) or (Length(KeyPath)=0);
  MaybeFlush;
end;

Function TXmlRegistry.GetValueData(Name : String; Var DataType : TDataType; Var Data; Var DataSize : Integer) : Boolean;

Type
  PCardinal = ^Cardinal;

Var
  Node  : TDomElement;
  DataNode : TDomNode;
  ND : Integer;
  S : AnsiString;
  HasData: Boolean;
  IntValue: Integer;
begin
  Node:=FindValueKey(Name);
  Result:=Node<>Nil;
  If Result then
    begin
    DataNode:=Node.FirstChild;
    HasData:=Assigned(DataNode) and (DataNode.NodeType=TEXT_NODE);
    ND:=StrToIntDef(Node[Stype],0);
    Result:=ND<=Ord(High(TDataType));
    If Result then
      begin
      DataType:=TDataType(ND);
      Case DataType of
        dtDWORD : begin   // DataNode is required
                  if HasData and TryStrToInt(DataNode.NodeValue,IntValue) then
                    begin
                    PCardinal(@Data)^:=IntValue;
                    DataSize:=SizeOf(Cardinal);
                    end
                  else
                    Result:=False;
                  end;
        dtString : begin  // DataNode is optional
                   if HasData then
                     begin
                     S:=DataNode.NodeValue; // Convert to ansistring
                     DataSize:=Length(S);
                     if (DataSize>0) then
                       Move(S[1],Data,DataSize);
                     end
                   else
                     DataSize:=0;
                   end;
        dtBinary : begin  // DataNode is optional
                   if HasData then
                     begin
                     DataSize:=Length(DataNode.NodeValue);
                     If (DataSize>0) then
                       HexToBuf(DataNode.NodeValue,Data,DataSize);
                     end
                   else
                     DataSize:=0;
                   end;
      end;
      end;
    end;
end;

Function TXmlRegistry.SetValueData(Name : String; DataType : TDataType; Const Data; DataSize : Integer) : Boolean;

Type
  PCardinal = ^Cardinal;

Var
  Node  : TDomElement;
  DataNode : TDomNode;
  S : String;
begin
  Node:=FindValueKey(Name);
  If Node=Nil then
    Node:=CreateValueKey(Name);
  Result:=(Node<>Nil);
  If Result then
    begin
    Node[SType]:=IntToStr(Ord(DataType));
    DataNode:=Node.FirstChild;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    Case DataType of
      dtDWORD : S:=IntToStr(PCardinal(@Data)^);
      dtString : SetString(S, PAnsiChar(@Data), DataSize);
      dtBinary : S:=BufToHex(Data,DataSize);
    else
      s:='';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    // Reading <value></value> results in <value/>, i.e. no subkey exists any more. Create textnode.
    if (DataNode=nil) then
      begin
      DataNode:=FDocument.CreateTextNode('');
      Node.AppendChild(DataNode);
      end;
    Case DataType of
      dtDWORD : DataNode.NodeValue:=IntToStr(PCardinal(@Data)^);
      dtString : begin
                 SetLength(S,DataSize);
                 If (DataSize>0) then
                   Move(Data,S[1],DataSize);
                 DataNode.NodeValue:=S;
                 end;
      dtBinary : begin
                 S:=BufToHex(Data,DataSize);
                 DataNode.NodeValue:=S;
                 end;
      end;
>>>>>>> graemeg/cpstrnew
    end;
    if s <> '' then
      begin
      if DataNode=nil then
        begin
        // may happen if previous value was empty;
        // XML does not handle empty textnodes.
        DataNode:=FDocument.CreateTextNode(s);
        Node.AppendChild(DataNode);
        end
      else
        DataNode.NodeValue:=s;
      end
    else
      DataNode.Free;
=======
=======
>>>>>>> origin/fixes_2_2
    Result:=DataNode<>Nil;  // Bug 9879. Create child here?
    If Result Then
      begin 
        Case DataType of
          dtDWORD : DataNode.NodeValue:=IntToStr(PCardinal(@Data)^);
          dtString : begin
                     SetLength(S,DataSize);
                     If (DataSize>0) then
                       Move(Data,S[1],DataSize);
                     DataNode.NodeValue:=S;
                     end;
          dtBinary : begin
                     S:=BufToHex(Data,DataSize);
                     DataNode.NodeValue:=S;
                     end;
        end;
      end;
    end;
  If Result then
    begin
>>>>>>> graemeg/fixes_2_2
    FDirty:=True;
    MaybeFlush;
    end;
end;

Function TXmlRegistry.FindSubKey (S : String; N : TDomElement) : TDomElement;

Var
  Node : TDOMNode;

begin
  Result:=Nil;
  If N<>Nil then
    begin
    Node:=N.FirstChild;
    While (Result=Nil) and (Assigned(Node)) do
      begin
      If (Node.NodeType=ELEMENT_NODE) and (Node.NodeName=SKey) then
        If CompareText(TDomElement(Node)[SName],S)=0 then
          Result:=TDomElement(Node);
      Node:=Node.NextSibling;
      end;
    end;
end;

Function TXmlRegistry.CreateSubKey (S : String; N : TDomElement) : TDomElement;

begin
  Result:=FDocument.CreateElement(SKey);
  Result[SName]:=S;
  if N<>nil then
    N.AppendChild(Result);
  FDirty:=True;
end;

Function  TXmlRegistry.FindValueKey (S : String) : TDomElement;

Var
  Node : TDOMNode;

begin
  Result:=Nil;
  If FCurrentElement<>Nil then
    begin
    Node:=FCurrentElement.FirstChild;
    While (Result=Nil) and (Assigned(Node)) do
      begin
      If (Node.NodeType=ELEMENT_NODE) and (Node.NodeName=SValue) then
        If CompareText(TDomElement(Node)[SName],S)=0 then
          Result:=TDomElement(Node);
      Node:=Node.NextSibling;
      end;
    end;
end;

Function  TXmlRegistry.CreateValueKey (S : String) : TDomElement;

begin
  If Assigned(FCurrentElement) then
    begin
    Result:=FDocument.CreateElement(SValue);
    Result[SName]:=S;
    // textnode to hold the value;
    Result.AppendChild(FDocument.CreateTextNode(''));
    FCurrentElement.AppendChild(Result);
    FDirty:=True;
    end
  else
    Result:=Nil;
end;

Procedure TXMLregistry.MaybeFlush;

begin
  If FAutoFlush then
    Flush;
end;

Procedure TXmlRegistry.Flush;

Var
  S : TStream;

begin
  If FDirty then
    begin
    S:=TFileStream.Create(FFileName,fmCreate);
    Try
      WriteXMLFile(FDocument,S);
      FDirty:=False;
    finally
      S.Free;
    end;
    end;
end;


Procedure TXmlRegistry.Load;

Var
  S : TStream;

begin
  If Not FileExists(FFileName) then
    CreateEmptyDoc
  else
    begin
    S:=TFileStream.Create(FFileName,fmOpenReadWrite);
    try
      LoadFromStream(S);
    finally
      S.Free;
    end;
    end;
end;

Procedure TXmlRegistry.LoadFromStream(S : TStream);

begin
  If Assigned(FDocument) then
    begin
    FDocument.Free;
    FDocument:=Nil;
    end;
  ReadXMLFile(FDocument,S);
  if (FDocument=Nil) then
    CreateEmptyDoc;
  SetRootKey('HKEY_CURRENT_USER');
  FDirty:=False;
end;

Function TXmlRegistry.BufToHex(Const Buf; Len : Integer) : String;

Var
  P : PByte;
  S : String;
  I : Integer;

begin
  SetLength(Result,Len*2);
  P:=@Buf;
  For I:=0 to Len-1 do
    begin
    S:=HexStr(P[I],2);
    Move(S[1],Result[I*2+1],2);
    end;
end;

Function TXMLRegistry.hexToBuf(Const Str : String; Var Buf; Var Len : Integer ) : Integer;

Var
  I : Integer;
  P : PByte;
  S : String;
  B : Byte;
  Code : Integer;

begin
  P:=@Buf;
  Len:= Length(Str) div 2;
  Result:=0;
  For I:=0 to Len-1 do
    begin
    S:='$'+Copy(Str,(I*2)+1,2);
    Val(S,B,Code);
    If Code<>0 then
      begin
      Inc(Result);
      B:=0;
      end;
    P[I]:=B;
    end;
end;

Function TXMLRegistry.DeleteValue(S : String) : Boolean;

Var
  N : TDomElement;

begin
  N:=FindValueKey(S);
  Result:=(N<>Nil);
  If Result then
    begin
    FCurrentElement.RemoveChild(N);
    FDirty:=True;
    MaybeFlush;
    end;
end;

Function TXMLRegistry.GetValueSize(Name : String) : Integer;

Var
  Info : TDataInfo;

begin
  If GetValueInfo(Name,Info) then
    Result:=Info.DataSize
  else
    Result:=-1;
end;

Function TXMLRegistry.GetValueType(Name : String) : TDataType;

Var
  Info : TDataInfo;

begin
  If GetValueInfo(Name,Info) then
    Result:=Info.DataType
  else
    Result:=dtUnknown;
end;

Function TXMLRegistry.GetValueInfo(Name : String; Var Info : TDataInfo) : Boolean;

Var
  N  : TDomElement;
  DN : TDomNode;
  L : Integer;
begin
  N:=FindValueKey(Name);
  Result:=(N<>Nil);
  If Result then
    begin
    DN:=N.FirstChild;
    if Assigned(DN) and (DN.NodeType=TEXT_NODE) then
      L:=TDOMText(DN).Length
    else
      L:=0;
    With Info do
      begin
      DataType:=TDataType(StrToIntDef(N[SType],0));
      Case DataType of
        dtUnknown : DataSize:=0;
        dtDword   : Datasize:=SizeOf(Cardinal);
        dtString  : DataSize:=L;
        dtBinary  : DataSize:=L div 2;
      end;
      end;
    end;
end;

Function  TXMLRegistry.GetKeyInfo(Var Info : TKeyInfo) : Boolean;

Var
  Node,DataNode : TDOMNode;
  L    : Integer;

begin
  FillChar(Info,SizeOf(Info),0);
  Result:=FCurrentElement<>Nil;
  If Result then
    With Info do
      begin
      If (FFileName<>'') Then
        FTime:=FileAge(FFileName);
      Node:=FCurrentElement.FirstChild;
      While Assigned(Node) do
        begin
        If (Node.NodeType=ELEMENT_NODE) then
          If (Node.NodeName=SKey) then
            begin
            Inc(SubKeys);
            L:=Length(TDomElement(Node)[SName]);
            If (L>SubKeyLen) then
              SubKeyLen:=L;
            end
          else if (Node.NodeName=SValue) then
            begin
            Inc(Values);
            L:=Length(TDomElement(Node)[SName]);
            If (L>ValueLen) then
              ValueLen:=L;
            DataNode:=TDomElement(Node).FirstChild;
            If (DataNode<>Nil) and (DataNode is TDomText) then
              Case TDataType(StrToIntDef(TDomElement(Node)[SType],0)) of
                dtUnknown : L:=0;
                dtDWord   : L:=4;
                DtString  : L:=Length(DataNode.NodeValue);
                dtBinary  : L:=Length(DataNode.NodeValue) div 2;
              end
            else
              L:=0;
            If (L>DataLen) Then
              DataLen:=L;
            end;
        Node:=Node.NextSibling;
        end;
      end;
end;

Function TXMLRegistry.EnumSubKeys(List : TStrings) : Integer;

Var
  Node : TDOMNode;

begin
  List.Clear;
  Result:=0;
  If FCurrentElement<>Nil then
    begin
    Node:=FCurrentElement.FirstChild;
    While Assigned(Node) do
      begin
      If (Node.NodeType=ELEMENT_NODE) and (Node.NodeName=SKey) then
        List.Add(TDomElement(Node)[SName]);
      Node:=Node.NextSibling;
      end;
    Result:=List.Count;
    end;
end;

Function TXMLRegistry.EnumValues(List : TStrings) : Integer;

Var
  Node : TDOMNode;

begin
  List.Clear;
  Result:=0;
  If FCurrentElement<>Nil then
    begin
    Node:=FCurrentElement.FirstChild;
    While Assigned(Node) do
      begin
      If (Node.NodeType=ELEMENT_NODE) and (Node.NodeName=SValue) then
        List.Add(TDomElement(Node)[SName]);
      Node:=Node.NextSibling;
      end;
    Result:=List.Count;
    end;
end;

Function TXMLRegistry.KeyExists(KeyPath : String) : Boolean;

begin
  Result:=FindKey(KeyPath)<>Nil;
end;

Function TXMLRegistry.RenameValue(Const OldName,NewName : String) : Boolean;

Var
  N : TDomElement;

begin
  N:=FindValueKey(OldName);
  result:=n<>nil;
  If (Result) then
    begin
      N[SName]:=NewName;
      FDirty:=True;
      MaybeFlush;
    end;
end;

Function TXMLRegistry.FindKey (S : String) : TDomElement;

Var
  SubKey : String;
  P : Integer;
  Node : TDomElement;

begin
  Result:=Nil;
  If (Length(S)=0) then
    Exit;
  S:=NormalizeKey(S);
  If (FCurrentElement<>nil) then
  begin
    Delete(S,1,1);
    Node:=FCurrentElement;
  end
  else
    begin
    Delete(S,1,1);
    Node:=FDocument.DocumentElement;
    If (FRootKey<>'') then
      S:=FRootKey+S;
    end;
  repeat
    P:=Pos('/',S);
    If (P<>0) then
      begin
      SubKey:=Copy(S,1,P-1);
      Delete(S,1,P);
      Result:=FindSubKey(SubKey,Node);
      Node:=Result;
      end;
  Until (Result=Nil) or (Length(S)=0);
end;

Function  TXmlRegistry.ValueExists(ValueName : String) : Boolean;

begin
  Result:=FindValueKey(ValueName)<>Nil;
end;


end.
