{$mode objfpc}
{$h+}

unit xmlreg;

Interface

uses
  sysutils,classes,dom,xmlread,xmlwrite;

Type

  TDataType = (dtUnknown,dtDWORD,dtString,dtBinary,dtStrings,dtQWord);
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

  TUnicodeStringArray = Array of UnicodeString;

  { TXmlRegistry }

  TXmlRegistry = Class(TObject)
  Private
    FAutoFlush,
    FDirty : Boolean;
    FFileName : String;
    FRootKey : UnicodeString;
    FDocument : TXMLDocument;
    FCurrentElement : TDomElement;
    FCurrentKey : UnicodeString;
    Procedure SetFileName(Value : String);
  Protected
    function DoGetValueData(Name: UnicodeString; out DataType: TDataType; Var Data; Var DataSize: Integer; IsUnicode: Boolean): Boolean; virtual;
    function DoSetValueData(Name: UnicodeString; DataType: TDataType; const Data; DataSize: Integer; IsUnicode: Boolean): Boolean; virtual;
    Procedure LoadFromStream(S : TStream);
    Function  NormalizeKey(KeyPath : UnicodeString) : UnicodeString;
    Procedure CreateEmptyDoc;
    Function  FindKey (S : UnicodeString) : TDomElement;
    Function  FindSubKey (S : UnicodeString; N : TDomElement) : TDomElement;
    Function  CreateSubKey (S : UnicodeString; N : TDomElement) : TDomElement;
    Function  FindValueKey (S : UnicodeString) : TDomElement;
    Function  CreateValueKey (S : UnicodeString) : TDomElement;
    Function  BufToHex(Const Buf; Len : Integer) : String;
    Function  HexToBuf(Const Str : UnicodeString; Var Buf; Var Len : Integer ) : Integer;
    Procedure MaybeFlush;
    Property  Document : TXMLDocument Read FDocument;
    Property  Dirty : Boolean Read FDirty write FDirty;
  Public
    Constructor Create(AFileName : String);
    Destructor  Destroy;override;
    Function  SetKey(KeyPath : UnicodeString; AllowCreate : Boolean) : Boolean ;
    Procedure SetRootKey(Value : UnicodeString);
    Function  DeleteKey(KeyPath : UnicodeString) : Boolean;
    Function  CreateKey(KeyPath : UnicodeString) : Boolean;
    Function  GetValueSize(Name : UnicodeString) : Integer;
    Function  GetValueType(Name : UnicodeString) : TDataType;
    Function  GetValueInfo(Name : UnicodeString; Out Info : TDataInfo; AsUnicode : Boolean = False) : Boolean;
    Function  GetKeyInfo(Out Info : TKeyInfo) : Boolean;
    Function  EnumSubKeys(List : TStrings) : Integer;
    Function  EnumSubKeys: TUnicodeStringArray;
    Function  EnumValues(List : TStrings) : Integer;
    Function  EnumValues: TUnicodeStringArray;
    Function  KeyExists(KeyPath : UnicodeString) : Boolean;
    Function  ValueExists(ValueName : UnicodeString) : Boolean;
    Function  RenameValue(Const OldName,NewName : UnicodeString) : Boolean;
    Function  DeleteValue(S : UnicodeString) : Boolean;
    Procedure Flush;
    Procedure Load;
    Function GetValueData(Name : UnicodeString; Out DataType : TDataType; Var Data; Var DataSize : Integer) : Boolean;
    Function SetValueData(Name : UnicodeString; DataType : TDataType; Const Data; DataSize : Integer) : Boolean;
    // These interpret the Data buffer as unicode data
    Function GetValueDataUnicode(Name : UnicodeString; Out DataType : TDataType; Var Data; Var DataSize : Integer) : Boolean;
    Function SetValueDataUnicode(Name : UnicodeString; DataType : TDataType; Const Data; DataSize : Integer) : Boolean;
    Property CurrentKey: UnicodeString read FCurrentKey; //used by TRegistry
    Property FileName : String Read FFileName Write SetFileName;
    Property RootKey : UnicodeString Read FRootKey Write SetRootkey;
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

Function TXmlRegistry.NormalizeKey(KeyPath : UnicodeString) : UnicodeString;

Var
  L : Integer;

begin
  Result:=UnicodeStringReplace(KeyPath,'\','/',[rfReplaceAll]);
  L:=Length(Result);
  If (L>0) and (Result[L]<>'/') then
    Result:=Result+'/';
  If (L>0) and (Result[1]<>'/') then
    Result:='/' + Result;
end;

Function TXmlRegistry.SetKey(KeyPath : UnicodeString; AllowCreate : Boolean) : boolean;

Var
  SubKey,ResultKey : UnicodeString;
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

Procedure TXmlRegistry.SetRootKey(Value : UnicodeString);

begin
  FRootKey:=NormalizeKey(Value);
  If (Length(FRootKey)>1) and (FRootKey[1]='/') then
    Delete(FRootKey,1,1);
  FCurrentKey:='';
  FCurrentElement:=Nil;
end;

Function TXmlRegistry.DeleteKey(KeyPath : UnicodeString) : Boolean;

Var
  N, Curr : TDomElement;
  Node: TDOMNode;

begin
 N:=FindKey(KeyPath);
 Result:=(N<>Nil);
 If Result then
   begin
   //if a key has subkeys, result shall be false and nothing shall be deleted
   Curr:=N;
   Node:=Curr.FirstChild;
   While Assigned(Node) do
     begin
     If (Node.NodeType=ELEMENT_NODE) and (Node.NodeName=SKey) then
       Exit(False);
     Node:=Node.NextSibling;
     end;
   (N.ParentNode as TDomElement).RemoveChild(N);
   FDirty:=True;
   MaybeFlush;
   end;
end;

Function TXmlRegistry.CreateKey(KeyPath : UnicodeString) : Boolean;

Var
  SubKey : UnicodeString;
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

Function TXmlRegistry.DoGetValueData(Name : UnicodeString; Out DataType : TDataType; Var Data; Var DataSize : Integer; IsUnicode : Boolean) : Boolean;

Type
  PCardinal = ^Cardinal;

Var
  Node  : TDomElement;
  DataNode : TDomNode;
  BL,ND,NS : Integer;
  S : UTF8String;
  U : UnicodeString;
  HasData: Boolean;
  D : DWord;
  Q : QWord;
  
begin
  //writeln('TXmlRegistry.DoGetValueData: Name=',Name,' IsUnicode=',IsUnicode);
  Node:=FindValueKey(Name);
  Result:=Node<>Nil;
  If Result then
    begin
    //writeln('TXmlRegistry.DoGetValueData: Node<>nil');
    DataNode:=Node.FirstChild;
    HasData:=Assigned(DataNode) and (DataNode.NodeType=TEXT_NODE);
    //writeln('TXmlRegistry.DoGetValueData: HasData=',hasdata);
    ND:=StrToIntDef(String(Node[Stype]),0);
    //writeln('TXmlRegistry.DoGetValueData: ND=',ND);
    Result:=ND<=Ord(High(TDataType));
    If Result then
      begin
      DataType:=TDataType(ND);
      //writeln('TXmlRegistry.DoGetValueData: DataType=',DataType);
      NS:=0; // Initialize, for optional nodes.
      Case DataType of
        dtDWORD : begin   // DataNode is required
                  NS:=SizeOf(Cardinal);
                  Result:=HasData and TryStrToDWord(String(DataNode.NodeValue),D) and (DataSize>=NS);
                  if Result then
                    PCardinal(@Data)^:=D;
                  end;
        dtQWORD : begin   // DataNode is required
                  NS:=SizeOf(QWORD);
                  Result:=HasData and TryStrToQWord(String(DataNode.NodeValue),Q) and (DataSize>=NS);
                  if Result then
                    PUInt64(@Data)^:=Q;
                  end;
        dtString : // DataNode is optional
                   if HasData then
                     begin
                     if not IsUnicode then
                       begin
                       S:=DataNode.NodeValue; // Convert to ansistring
                       NS:=Length(S);
                       Result:=(DataSize>=NS);
                       if Result then
                         Move(S[1],Data,NS);
                       end
                     else
                       begin
                       U:=DataNode.NodeValue;
                       NS:=Length(U)*SizeOf(UnicodeChar);
                       Result:=(DataSize>=NS);
                       if Result then
                         Move(U[1],Data,NS);
                       end
                     end;

        dtBinary,
        dtStrings : // DataNode is optional
                   if HasData then
                     begin
                     BL:=Length(DataNode.NodeValue);
                     //writeln('TXmlRegistry.DoGetValueData: BL=',BL);
                     NS:=BL div 2;
                     Result:=DataSize>=NS;
                     //writeln('TXmlRegistry.DoGetValueData: Result=',Result);
                     If Result then
                       // No need to check for -1, We checked NS before calling.
                       NS:=HexToBuf(DataNode.NodeValue,Data,BL);
                     end;
      end;
      // Report needed/used size in all cases
      DataSize:=NS;
      end;
    end;
end;

Function TXmlRegistry.DoSetValueData(Name : UnicodeString; DataType : TDataType; Const Data; DataSize : Integer; IsUnicode : Boolean) : Boolean;

Type
  PCardinal = ^Cardinal;

Var
  Node  : TDomElement;
  DataNode : TDomNode;
  SW : UnicodeString;

begin
  //writeln('TXmlRegistry.DoSetValueData A: Name=',Name,', DataType=',DataType,', DataSize=',DataSize,', IsUnicode=',IsUnicode);
  Node:=FindValueKey(Name);
  If Node=Nil then
    Node:=CreateValueKey(Name);
  Result:=(Node<>Nil);
  If Result then
    begin
    Node[SType]:=UnicodeString(IntToStr(Ord(DataType)));
    DataNode:=Node.FirstChild;

    Case DataType of
      dtDWORD : SW:=UnicodeString(IntToStr(PCardinal(@Data)^));
      dtQWORD : SW:=UnicodeString(IntToStr(PUInt64(@Data)^));
      dtString : begin
                 if IsUnicode then
                   SW:=UnicodeString(PUnicodeChar(@Data))
                 else
                   SW:=UnicodeString(PAnsiChar(@Data));
                   //S:=SW;
                 end;
      dtBinary : SW:=UnicodeString(BufToHex(Data,DataSize));
      dtStrings : SW:=UnicodeString(BufToHex(Data,DataSize));
    else
      sw:='';
    end;
    if sw <> '' then
      begin
      if DataNode=nil then
        begin
        // may happen if previous value was empty;
        // XML does not handle empty textnodes.
        DataNode:=FDocument.CreateTextNode(sw);
        Node.AppendChild(DataNode);
        end
      else
        DataNode.NodeValue:=sw;
      end
    else
      DataNode.Free;
    FDirty:=True;
    MaybeFlush;
    end;
end;

Function TXmlRegistry.SetValueData(Name : UnicodeString; DataType : TDataType; Const Data; DataSize : Integer) : Boolean;

begin
  Result:=DoSetValueData(Name,DataType,Data,DataSize,False);
end;

Function TXmlRegistry.GetValueData(Name : UnicodeString; Out DataType : TDataType; Var Data; Var DataSize : Integer) : Boolean;

begin
  Result:=DoGetValueData(Name,DataType,Data,DataSize,False);
end;

function TXmlRegistry.GetValueDataUnicode(Name: UnicodeString; out DataType: TDataType; Var Data; Var DataSize: Integer): Boolean;
begin
  Result:=DoGetValueData(Name,DataType,Data,DataSize,True);
end;

function TXmlRegistry.SetValueDataUnicode(Name: UnicodeString; DataType: TDataType; const Data; DataSize: Integer): Boolean;
begin
  Result:=DoSetValueData(Name,DataType,Data,DataSize,True)
end;

Function TXmlRegistry.FindSubKey (S : UnicodeString; N : TDomElement) : TDomElement;

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
        If UnicodeCompareText(TDomElement(Node)[SName],S)=0 then
          Result:=TDomElement(Node);
      Node:=Node.NextSibling;
      end;
    end;
end;

Function TXmlRegistry.CreateSubKey (S : UnicodeString; N : TDomElement) : TDomElement;

begin
  Result:=FDocument.CreateElement(SKey);
  Result[SName]:=S;
  if N<>nil then
    N.AppendChild(Result);
  FDirty:=True;
end;

Function  TXmlRegistry.FindValueKey (S : UnicodeString) : TDomElement;

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
        If UnicodeCompareText(TDomElement(Node)[SName],S)=0 then
          Result:=TDomElement(Node);
      Node:=Node.NextSibling;
      end;
    end;
end;

Function  TXmlRegistry.CreateValueKey (S : UnicodeString) : TDomElement;

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

Function TXMLRegistry.HexToBuf(Const Str : UnicodeString; Var Buf; Var Len : Integer ) : Integer;

Var
  NLeN,I : Integer;
  P : PByte;
  S : UnicodeString;
  B : Byte;
  Code : Integer;

begin
  //writeln('TXMLRegistry.HexToBuf A: Str=',Str,', Len=',Len);
  Result:=0;
  P:=@Buf;
  //writeln('TXMLRegistry.HexToBuf B: (p=nil)=',p=nil);
  NLen:= Length(Str) div 2;
  //writeln('TXMLRegistry.HexToBuf C: NLen=',NLen,', SizeOf(TDateTime)=',SizeOf(TDateTime));
  If (NLen>Len) then
    begin
    Len:=NLen;
    Exit(-1);
    end;
  For I:=0 to NLen-1 do
    begin
    //write('TXMLRegistry.HexToBuf: i=',i);
    S:='$'+Copy(Str,(I*2)+1,2);
    //write(', S=',S);
    Val(S,B,Code);
    //writeln(', Code=',Code);
    If Code<>0 then
      begin    //This means invalid data in the registry, why continue and increment result? Why not Exit(-1)?
      //Inc(Result);   //the whole function only worked because this was called as often as when Code=0, so by change
      //B:=0;          //it causes AV's
      Exit(-1);
      end;
    Inc(Result);
    P[I]:=B;
    end;
  //writeln('TXMLRegistry.HexToBuf End: Result=',Result);
end;

Function TXMLRegistry.DeleteValue(S : UnicodeString) : Boolean;

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

Function TXMLRegistry.GetValueSize(Name : UnicodeString) : Integer;

Var
  Info : TDataInfo;

begin
  If GetValueInfo(Name,Info,True) then
    Result:=Info.DataSize
  else
    Result:=-1;
end;

Function TXMLRegistry.GetValueType(Name : UnicodeString) : TDataType;

Var
  Info : TDataInfo;

begin
  If GetValueInfo(Name,Info,True) then
    Result:=Info.DataType
  else
    Result:=dtUnknown;
end;

function TXmlRegistry.GetValueInfo(Name: UnicodeString; out Info: TDataInfo; AsUnicode: Boolean): Boolean;

Var
  N  : TDomElement;
  DN : TDomNode;
  L : Integer;
  S: Ansistring; 
begin
  N:=FindValueKey(Name);
  Result:=(N<>Nil);
  If Result then
    begin
    DN:=N.FirstChild;
    if Assigned(DN) and (DN.NodeType=TEXT_NODE) then
      begin
      if AsUnicode then
        L:=Length(DN.NodeValue)*SizeOf(UnicodeChar)
      else
        begin
        S := DN.NodeValue;
        L:=Length(S);
        end
      end
    else
      L:=0;
    With Info do
      begin
      DataType:=TDataType(StrToIntDef(String(N[SType]),0));
      Case DataType of
        dtUnknown : DataSize:=0;
        dtDword   : Datasize:=SizeOf(Cardinal);
        dtString  : DataSize:=L;
        dtStrings,
        dtBinary  : DataSize:=L div 2;
      end;
      end;
    end;
end;

Function  TXMLRegistry.GetKeyInfo(Out Info : TKeyInfo) : Boolean;

Var
  Node,DataNode : TDOMNode;
  L    : Integer;

begin
  Info:=Default(TKeyInfo);
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
              Case TDataType(StrToIntDef(String(TDomElement(Node)[SType]),0)) of
                dtUnknown : L:=0;
                dtDWord   : L:=4;
                DtString  : L:=Length(String(DataNode.NodeValue));
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

function TXmlRegistry.EnumSubKeys: TUnicodeStringArray;

Var
  Node : TDOMNode;
  Len, Count: Integer;

begin
  Result:=nil;
  If FCurrentElement<>Nil then
    begin
    Node:=FCurrentElement.FirstChild;
    Len:=0;
    Count:=0;
    While Assigned(Node) do
      begin
      If (Node.NodeType=ELEMENT_NODE) and (Node.NodeName=SKey) then
        begin
        Inc(Count);
        if (Count>Len) then
          begin
          Inc(Len,10); //avoid calling SetLength on each addition
          SetLength(Result,Len);
          end;
        Result[Count-1]:=TDomElement(Node)[SName];
        end;
      Node:=Node.NextSibling;
      end;
    SetLength(Result,Count);
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
        If (Node.NodeType=ELEMENT_NODE) and (Node.NodeName=SValue) then
          List.Add(TDomElement(Node)[SName]);
      Node:=Node.NextSibling;
      end;
    Result:=List.Count;
    end;
end;

Function TXMLRegistry.EnumValues: TUnicodeStringArray;

Var
  Node : TDOMNode;
  Len, Count: Integer;
begin
  Result:=nil;
  If FCurrentElement<>Nil then
    begin
    Node:=FCurrentElement.FirstChild;
    Count:=0;
    Len:=0;
    While Assigned(Node) do
      begin
      If (Node.NodeType=ELEMENT_NODE) and (Node.NodeName=SValue) then
        begin
        Inc(Count);
        if (Count>Len) then
          begin
          Inc(Len,10); //avoid calling SetLength on each addition
          SetLength(Result,Len);
          end;
        Result[Count-1]:=TDomElement(Node)[SName];
        end;
      Node:=Node.NextSibling;
      end;
    SetLength(Result,Count);
    end;
end;


Function TXMLRegistry.KeyExists(KeyPath : UnicodeString) : Boolean;

begin
  Result:=FindKey(KeyPath)<>Nil;
end;

Function TXMLRegistry.RenameValue(Const OldName,NewName : UnicodeString) : Boolean;

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

Function TXMLRegistry.FindKey (S : UnicodeString) : TDomElement;

Var
  SubKey : UnicodeString;
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

Function  TXmlRegistry.ValueExists(ValueName : UnicodeString) : Boolean;

begin
  Result:=FindValueKey(ValueName)<>Nil;
end;


end.
