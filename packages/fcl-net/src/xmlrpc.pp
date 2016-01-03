{

    XML-RPC server and client library
    Copyright (c) 2003-2004 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}{$H+}
unit XMLRPC;

interface

uses SysUtils, Classes, fpAsync, ssockets, DOM, HTTPClient, HTTPSvlt;

type
  EXMLRPCParser = class(Exception);

  TXMLRPCParams = class(TDOMElement);
  TXMLRPCValue = class(TDOMElement);
  TXMLRPCStruct = class(TXMLRPCValue);
  TXMLRPCArray = class(TXMLRPCValue);

  TXMLRPCWriter = class
  private
    Doc: TXMLDocument;
  protected
    function CreateValueEl: TXMLRPCValue;
  public
    constructor Create;
    destructor Destroy; override;
    function MakeStream: TMemoryStream;

    procedure WriteMethodCall(const AMethodName: DOMString;
      Params: TXMLRPCParams);
    procedure WriteResponse(Value: TXMLRPCValue);
    procedure WriteFaultResponse(FaultCode: LongInt;
      const FaultString: DOMString);
    function CreateParams: TXMLRPCParams;
    procedure AddParam(Params: TXMLRPCParams; Value: TXMLRPCValue);
    function CreateIntValue(i: LongInt): TXMLRPCValue;
    function CreateBooleanValue(b: Boolean): TXMLRPCValue;
    function CreateStringValue(const s: DOMString): TXMLRPCValue;
    function CreateDoubleValue(d: Double): TXMLRPCValue;
    function CreateDateTimeValue(dt: TDateTime): TXMLRPCValue;
    function CreateStruct: TXMLRPCStruct;
    procedure AddStructMember(Struct: TXMLRPCStruct; const Name: DOMString;
      Member: TXMLRPCValue);
    function CreateArray: TXMLRPCArray;
    procedure AddArrayElement(AArray: TXMLRPCArray; Value: TXMLRPCValue);
    // !!!: Missing: Binary data
  end;

  TXMLRPCPostType = (
    xmlrpcInvalid,              // Invalid post type
    xmlrpcMethodCall,           // Method call
    xmlrpcResponse,             // Method call response (successfull)
    xmlrpcFaultResponse);       // Method call response (failed)

  TXMLRPCParser = class
  private
    Doc: TXMLDocument;
    CurDataNode: TDOMNode;
    InArray: Boolean;
    procedure NextNode;
    procedure PrevNode;
    function GetValue: String;
    function FindStructMember(AStruct: TXMLRPCStruct;
      const AMemberName: String): TDOMElement;
    function GetStructMemberValue(MemberNode: TDOMElement): String;
  public
    constructor Create(AStream: TStream);
    destructor Destroy; override;
    function GetPostType: TXMLRPCPostType;
    function GetMethodName: String;
    procedure ResetValueCursor;
    // Simple values
    function GetNextInt: LongInt;
    function GetPrevInt: LongInt;
    function GetNextBoolean: Boolean;
    function GetPrevBoolean: Boolean;
    function GetNextString: String;
    function GetPrevString: String;
    function GetNextDouble: Double;
    function GetPrevDouble: Double;
    // !!!: Missing: DateTime, Binary data
    // Struct values
    function GetNextStruct: TXMLRPCStruct;
    function GetIntMember(AStruct: TXMLRPCStruct; const AName: String;
      ADefault: Integer): Integer;
    function GetBooleanMember(AStruct: TXMLRPCStruct; const AName: String;
      ADefault: Boolean): Boolean;
    function GetStringMember(AStruct: TXMLRPCStruct; const AName: String;
      const ADefault: String): String;
    function GetDoubleMember(AStruct: TXMLRPCStruct; const AName: String;
      ADefault: Double): Double;
    // Array values
    procedure BeginArray;
    procedure EndArray;
  end;


{$ifdef NEVERTRUE}
  TOnXMLRPCCallCompleted = procedure(AParser: TXMLRPCParser) of object;

  TXMLRPCClient = class
  private
    FEventLoop: TEventLoop;
    FServerURL: String;
    FOnBeginRPC, FOnEndRPC: TNotifyEvent;
    RequestStream, ResponseStream: TMemoryStream;
    CurCallback: TOnXMLRPCCallCompleted;
    LocalEventLoop: TEventLoop;
    Connection: TCustomHttpClient;

    procedure MakeRequest(const AProcName: String; AArgs: array of const);
    procedure ProcessAnswer;
    procedure StreamSent(Sender: TObject);
    procedure DataAvailable(Sender: TObject);
  public
    constructor Create(AEventLoop: TEventLoop);
    procedure Call(ACallback: TOnXMLRPCCallCompleted;
      const AProcName: String; AArgs: array of const);
    procedure CallAsync(ACallback: TOnXMLRPCCallCompleted;
      const AProcName: String; AArgs: array of const);

    property EventLoop: TEventLoop read FEventLoop;
    property ServerURL: String read FServerURL write FServerURL;

    property OnBeginRPC: TNotifyEvent read FOnBeginRPC write FOnBeginRPC;
    property OnEndRPC: TNotifyEvent read FOnEndRPC write FOnEndRPC;
  end;
{$endif}

  TCheckCallEvent = procedure(AParser: TXMLRPCParser; const APath: TStrings;
    var ExecCall: Boolean) of object;
  TExceptionEvent = procedure(e: Exception) of object;

  TXMLRPCServlet = class(THttpServlet)
  private
    FOnCheckCall: TCheckCallEvent;
    FOnException: TExceptionEvent;
  protected
    procedure DoPost(Req: THttpServletRequest; Resp: THttpServletResponse);
      override;
  public
    procedure Dispatch(AParser: TXMLRPCParser; AWriter: TXMLRPCWriter;
      APath: TStrings); virtual; abstract;
    property OnCheckCall: TCheckCallEvent read FOnCheckCall write FOnCheckCall;
    property OnException: TExceptionEvent read FOnException write FOnException;
  end;


implementation

uses XMLWrite, XMLRead;


// Debugging stuff

{$IFDEF XMLRPCDebug}
const
  NodeNames: array[ELEMENT_NODE..NOTATION_NODE] of String = (
    'Element',
    'Attribute',
    'Text',
    'CDATA section',
    'Entity reference',
    'Entity',
    'Processing instruction',
    'Comment',
    'Document',
    'Document type',
    'Document fragment',
    'Notation'
  );

procedure DumpNode(node: TDOMNode; spc: String);
var
  i: Integer;
  attr: TDOMNode;
begin
  Write(spc, NodeNames[node.NodeType]);
  if Copy(node.NodeName, 1, 1) <> '#' then
    Write(' "', node.NodeName, '"');
  if node.NodeValue <> '' then
    Write(' "', node.NodeValue, '"');

  if (node.Attributes <> nil) and (node.Attributes.Length > 0) then begin
    Write(',');
    for i := 0 to node.Attributes.Length - 1 do begin
      attr := node.Attributes.Item[i];
      Write(' ', attr.NodeName, ' = "', attr.NodeValue, '"');
    end;
  end;
  WriteLn;

  node := node.FirstChild;
  while Assigned(node) do
  begin
    DumpNode(node, spc + '  ');
    node := node.NextSibling;
  end;
end;
{$ENDIF}


// XML-RPC Writer

constructor TXMLRPCWriter.Create;
begin
  inherited Create;
  Doc := TXMLDocument.Create;
end;

destructor TXMLRPCWriter.Destroy;
begin
  Doc.Free;
  inherited Destroy;
end;

function TXMLRPCWriter.MakeStream: TMemoryStream;
begin
  Result := TMemoryStream.Create;
  try
    WriteXMLFile(Doc, Result);
//    WriteXMLFile(Doc, THandleStream.Create(StdOutputHandle));
    Result.Position := 0;
  except
    on e: Exception do
      Result.Free;
  end;
end;

procedure TXMLRPCWriter.WriteMethodCall(const AMethodName: DOMString;
  Params: TXMLRPCParams);
var
  El, El2: TDOMElement;
begin
  El := Doc.CreateElement('methodCall');
  Doc.AppendChild(El);
  El2 := Doc.CreateElement('methodName');
  El.AppendChild(El2);
  El2.AppendChild(Doc.CreateTextNode(AMethodName));
  El.AppendChild(Params);
end;

procedure TXMLRPCWriter.WriteResponse(Value: TXMLRPCValue);
var
  El, El2: TDOMElement;
begin
  ASSERT(Value is TXMLRPCValue);
  El := Doc.CreateElement('methodResponse');
  Doc.AppendChild(El);
  El2 := Doc.CreateElement('params');
  El.AppendChild(El2);
  if not Assigned(Value) then
    Value := CreateBooleanValue(True);
  El := Doc.CreateElement('param');
  El2.AppendChild(El);
  El.AppendChild(Value);
end;

procedure TXMLRPCWriter.WriteFaultResponse(FaultCode: LongInt;
  const FaultString: DOMString);
var
  El, El2: TDOMElement;
  Struct: TXMLRPCStruct;
begin
  El := Doc.CreateElement('methodResponse');
  Doc.AppendChild(El);
  El2 := Doc.CreateElement('fault');
  El.AppendChild(El2);
  Struct := CreateStruct;
  AddStructMember(Struct, 'faultCode', CreateIntValue(FaultCode));
  AddStructMember(Struct, 'faultString', CreateStringValue(FaultString));
  El2.AppendChild(Struct);
end;

function TXMLRPCWriter.CreateParams: TXMLRPCParams;
begin
  Result := TXMLRPCParams(Doc.CreateElement('params'));
end;

procedure TXMLRPCWriter.AddParam(Params: TXMLRPCParams; Value: TXMLRPCValue);
var
  El: TDOMElement;
begin
  ASSERT((Params is TXMLRPCParams) and (Value is TXMLRPCValue));
  El := Doc.CreateElement('param');
  Params.AppendChild(El);
  El.AppendChild(Value);
end;

function TXMLRPCWriter.CreateIntValue(i: LongInt): TXMLRPCValue;
var
  El: TDOMElement;
begin
  Result := CreateValueEl;
  El := Doc.CreateElement('int');
  Result.AppendChild(El);
  El.AppendChild(Doc.CreateTextNode(IntToStr(i)));
end;

function TXMLRPCWriter.CreateBooleanValue(b: Boolean): TXMLRPCValue;
var
  El: TDOMElement;
begin
  Result := CreateValueEl;
  El := Doc.CreateElement('boolean');
  Result.AppendChild(El);
  El.AppendChild(Doc.CreateTextNode(IntToStr(Ord(b))));
end;

function TXMLRPCWriter.CreateStringValue(const s: DOMString): TXMLRPCValue;
var
  El: TDOMElement;
begin
  Result := CreateValueEl;
  El := Doc.CreateElement('string');
  Result.AppendChild(El);
  if Length(s) > 0 then
    El.AppendChild(Doc.CreateTextNode(s));
end;

function TXMLRPCWriter.CreateDoubleValue(d: Double): TXMLRPCValue;
var
  El: TDOMElement;
begin
  Result := CreateValueEl;
  El := Doc.CreateElement('double');
  Result.AppendChild(El);
  El.AppendChild(Doc.CreateTextNode(FloatToStr(d)));
end;

function TXMLRPCWriter.CreateDateTimeValue(dt: TDateTime): TXMLRPCValue;
var
  El: TDOMElement;
begin
  Result := CreateValueEl;
  El := Doc.CreateElement('dateTime.iso8601');
  Result.AppendChild(El);
  El.AppendChild(Doc.CreateTextNode(FormatDateTime('ddmmyyyyThh:nn:ss', dt)));
end;

function TXMLRPCWriter.CreateStruct: TXMLRPCStruct;
begin
  Result := TXMLRPCStruct(CreateValueEl);
  Result.AppendChild(Doc.CreateElement('struct'));
end;

procedure TXMLRPCWriter.AddStructMember(Struct: TXMLRPCStruct;
  const Name: DOMString; Member: TXMLRPCValue);
var
  MemberEl, El: TDOMElement;
begin
  ASSERT((Struct is TXMLRPCStruct) and (Name <> '') and
    (Member is TXMLRPCValue));
  MemberEl := Doc.CreateElement('member');
  Struct.FirstChild.AppendChild(MemberEl);
  El := Doc.CreateElement('name');
  MemberEl.AppendChild(El);
  El.AppendChild(Doc.CreateTextNode(Name));
  MemberEl.AppendChild(Member);
end;

function TXMLRPCWriter.CreateArray: TXMLRPCArray;
var
  ArrayEl: TDOMElement;
begin
  Result := TXMLRPCArray(CreateValueEl);
  ArrayEl := Doc.CreateElement('array');
  Result.AppendChild(ArrayEl);
  ArrayEl.AppendChild(Doc.CreateElement('data'));
end;

procedure TXMLRPCWriter.AddArrayElement(AArray: TXMLRPCArray;
  Value: TXMLRPCValue);
begin
  ASSERT((AArray is TXMLRPCArray) and (Value is TXMLRPCValue));
  AArray.FirstChild.FirstChild.AppendChild(Value);
end;

function TXMLRPCWriter.CreateValueEl: TXMLRPCValue;
begin
  Result := TXMLRPCValue(Doc.CreateElement('value'));
end;


// XML-RPC Parser

constructor TXMLRPCParser.Create(AStream: TStream);
var
  Node: TDOMNode;
begin
  inherited Create;
  ReadXMLFile(Doc, AStream);
  Node := Doc.DocumentElement;
  {$IFDEF XMLRPCDebug}DumpNode(Node, 'Parser> ');{$ENDIF}
  if (Node.NodeName = 'methodCall') or (Node.NodeName = 'methodResponse') then
  begin
    Node := Node.FirstChild;
    while Assigned(Node) and (Node.NodeName <> 'params') do
      Node := Node.NextSibling;
    if Assigned(Node) then
    begin
      Node := Node.FirstChild;
      while Assigned(Node) and (Node.NodeName <> 'param') do
        Node := Node.NextSibling;
      CurDataNode := Node;
    end;
  end;
end;

destructor TXMLRPCParser.Destroy;
begin
  Doc.Free;
  inherited Destroy;
end;

function TXMLRPCParser.GetPostType: TXMLRPCPostType;
var
  Node: TDOMNode;
begin
  Result := xmlrpcInvalid;
  Node := Doc.DocumentElement;
  if Node.NodeName = 'methodCall' then
    Result := xmlrpcMethodCall
  else if Node.NodeName = 'methodResponse' then
  begin
    Node := Node.FirstChild;
    while Assigned(Node) and (Node.NodeType <> ELEMENT_NODE) do
      Node := Node.NextSibling;
    if Assigned(Node) then
      if Node.NodeName = 'params' then
        Result := xmlrpcResponse
      else if Node.NodeName = 'fault' then
        Result := xmlrpcFaultResponse;
  end;
end;

function TXMLRPCParser.GetMethodName: String;
var
  Node: TDOMNode;
begin
  SetLength(Result, 0);
  Node := Doc.DocumentElement;
  if (not Assigned(Node)) or (Node.NodeName <> 'methodCall') then
    exit;
  Node := Node.FindNode('methodName');
  if not Assigned(Node) then
    exit;
  Node := Node.FirstChild;
  while Assigned(Node) do
  begin
    if Node.NodeType = TEXT_NODE then
      Result := Result + Node.NodeValue;
    Node := Node.NextSibling;
  end;
end;

procedure TXMLRPCParser.ResetValueCursor;
begin
  CurDataNode := CurDataNode.ParentNode.FirstChild;
  {$IFDEF XMLRPCDebug}DumpNode(CurDataNode, 'ResetValueCursor> ');{$ENDIF}
end;

function TXMLRPCParser.GetNextInt: LongInt;
begin
  Result := StrToInt(GetValue);
  NextNode;
end;

function TXMLRPCParser.GetPrevInt: LongInt;
begin
  PrevNode;
  Result := StrToInt(GetValue);
end;

function TXMLRPCParser.GetNextBoolean: Boolean;
begin
  Result := GetValue = '1';
  NextNode;
end;

function TXMLRPCParser.GetPrevBoolean: Boolean;
begin
  PrevNode;
  Result := GetValue = '1';
end;

function TXMLRPCParser.GetNextString: String;
begin
  Result := GetValue;
  NextNode;
end;

function TXMLRPCParser.GetPrevString: String;
begin
  PrevNode;
  Result := GetValue;
end;

function TXMLRPCParser.GetNextDouble: Double;
begin
  Result := StrToFloat(GetValue);
  NextNode;
end;

function TXMLRPCParser.GetPrevDouble: Double;
begin
  PrevNode;
  Result := StrToFloat(GetValue);
end;

function TXMLRPCParser.GetNextStruct: TXMLRPCStruct;
begin
  if Assigned(CurDataNode) and Assigned(CurDataNode.FirstChild) then
  begin
    Result := TXMLRPCStruct(CurDataNode.FirstChild);
    while Assigned(Result) and (Result.NodeName <> 'struct') do
      Result := TXMLRPCStruct(Result.NextSibling);
    NextNode;
  end else
    Result := nil;
end;

function TXMLRPCParser.GetIntMember(AStruct: TXMLRPCStruct;
  const AName: String; ADefault: Integer): Integer;
var
  MemberNode: TDOMElement;
begin
  MemberNode := FindStructMember(AStruct, AName);
  if Assigned(MemberNode) then
    Result := StrToInt(GetStructMemberValue(MemberNode))
  else
    Result := ADefault;
end;

function TXMLRPCParser.GetBooleanMember(AStruct: TXMLRPCStruct;
  const AName: String; ADefault: Boolean): Boolean;
var
  MemberNode: TDOMElement;
begin
  MemberNode := FindStructMember(AStruct, AName);
  if Assigned(MemberNode) then
    Result := GetStructMemberValue(MemberNode) = '1'
  else
    Result := ADefault;
end;

function TXMLRPCParser.GetStringMember(AStruct: TXMLRPCStruct;
  const AName: String; const ADefault: String): String;
var
  MemberNode: TDOMElement;
begin
  MemberNode := FindStructMember(AStruct, AName);
  if Assigned(MemberNode) then
    Result := GetStructMemberValue(MemberNode)
  else
    Result := ADefault;
end;

function TXMLRPCParser.GetDoubleMember(AStruct: TXMLRPCStruct;
  const AName: String; ADefault: Double): Double;
var
  MemberNode: TDOMElement;
begin
  MemberNode := FindStructMember(AStruct, AName);
  if Assigned(MemberNode) then
    Result := StrToFloat(GetStructMemberValue(MemberNode))
  else
    Result := ADefault;
end;

procedure TXMLRPCParser.BeginArray;
begin
  if Assigned(CurDataNode) then
  begin
    CurDataNode := CurDataNode.FirstChild;
    while Assigned(CurDataNode) and (CurDataNode.NodeName <> 'array') do
      CurDataNode := CurDataNode.NextSibling;
    if Assigned(CurDataNode) then
    begin
      CurDataNode := CurDataNode.FirstChild;
      while Assigned(CurDataNode) and (CurDataNode.NodeName <> 'data') do
        CurDataNode := CurDataNode.NextSibling;
{      if Assigned(CurDataNode) then
      begin
        CurDataNodeParent := CurDataNode;
        CurDataNode := nil;
        ResetValueCursor;
      end;}
    end;
    //NextNode;
  end;
end;

procedure TXMLRPCParser.EndArray;
begin
end;

procedure TXMLRPCParser.NextNode;
begin
  repeat
    CurDataNode := CurDataNode.NextSibling;
  until (not Assigned(CurDataNode)) or (CurDataNode.NodeType = ELEMENT_NODE);
end;

procedure TXMLRPCParser.PrevNode;
begin
  {$IFDEF XMLRPCDebug}DumpNode(CurDataNode, 'PrevNode before> ');{$ENDIF}
  if Assigned(CurDataNode.PreviousSibling) then
    CurDataNode := CurDataNode.PreviousSibling
  else
    CurDataNode := CurDataNode.ParentNode.LastChild;
  {$IFDEF XMLRPCDebug}DumpNode(CurDataNode, 'PrevNode result> ');{$ENDIF}
end;

function TXMLRPCParser.GetValue: String;
var
  Node: TDOMNode;
begin
  if not Assigned(CurDataNode) then
    Result := ''
  else
  begin
    Node := CurDataNode;
    if Node.NodeName <> 'value' then
      Node := Node.FirstChild;
    Node := Node.FirstChild;
    if Node.NodeType = TEXT_NODE then
      Result := Node.NodeValue
    else begin
      while Assigned(Node) and (Node.NodeType <> ELEMENT_NODE) do
        Node := Node.NextSibling;
      if Assigned(Node) then
      begin
        Node := Node.FirstChild;
        if Assigned(Node) and (Node.NodeType = TEXT_NODE) then
          Result := Node.NodeValue
        else
          Result := '';
      end;
    end;
  end;
end;

function TXMLRPCParser.FindStructMember(AStruct: TXMLRPCStruct;
  const AMemberName: String): TDOMElement;
var
  Node: TDOMNode;
begin
  Result := TDOMElement(AStruct.FirstChild);
  while Assigned(Result) and (Result.NodeName = 'member') do
  begin
    Node := Result.FirstChild;
    while Assigned(Node) do
    begin
      if Node.NodeName = 'name' then
      begin
        if Assigned(Node.FirstChild) and
          (CompareText(Node.FirstChild.NodeValue, AMemberName) = 0) then
          exit;
      end;
      Node := Node.NextSibling;
    end;
    Result := TDOMElement(Result.NextSibling);
  end;
end;

function TXMLRPCParser.GetStructMemberValue(MemberNode: TDOMElement): String;
var
  Node, Subnode: TDOMNode;
begin
  Node := MemberNode.FirstChild;
  while Assigned(Node) do
  begin
    if Node.NodeName = 'value' then
    begin
       Subnode := Node.FirstChild;
       if Assigned(Subnode) and (Subnode.NodeType = TEXT_NODE) then
       begin
         Result := Subnode.NodeValue;
         exit;
       end;
       while Assigned(Subnode) do
       begin
         if Subnode.NodeType = ELEMENT_NODE then
         begin
           if Assigned(Subnode.FirstChild) then
             Result := Subnode.FirstChild.NodeValue
           else
             Result := '';
           exit;
         end;
         Subnode := Subnode.NextSibling;
       end;
    end;
    Node := Node.NextSibling;
  end;
end;


{$IFDEF NEVERTRUE}
// XML-RPC Client

constructor TXMLRPCClient.Create(AEventLoop: TEventLoop);
begin
  inherited Create;
  FEventLoop := AEventLoop;
end;

procedure TXMLRPCClient.Call(ACallback: TOnXMLRPCCallCompleted;
  const AProcName: String; AArgs: array of const);
var
  Host: String;
  Port: Word;
  Socket: TInetSocket;
begin
  CurCallback := ACallback;
  MakeRequest(AProcName, AArgs);
  try
    ResponseStream := TMemoryStream.Create;
    if Assigned(OnBeginRPC) then
      OnBeginRPC(Self);

    Host := 'localhost';
    Port := 12345;

    Socket := TInetSocket.Create(Host, Port);
    try
      RequestStream.Position := 0;
//    Socket.Write(RequestStream.Memory^, RequestStream.Size);
      LocalEventLoop := TEventLoop.Create;
      try
        Connection := TCustomHttpClient.Create(LocalEventLoop, Socket);
        try
          Connection.HeaderToSend := THttpRequestHeader.Create;
          with THttpRequestHeader(Connection.HeaderToSend) do
          begin
            Command := 'POST';
            URI := '/xmlrpc';
            UserAgent := 'Free Pascal XML-RPC';
            ContentType := 'text/xml';
            ContentLength := RequestStream.Size;
          end;
          Connection.StreamToSend := RequestStream;
          Connection.ReceivedHeader := THttpResponseHeader.Create;
          Connection.ReceivedStream := ResponseStream;
          Connection.OnStreamSent := @StreamSent;
          Connection.Send;
          LocalEventLoop.Run;
        finally
          if Assigned(Connection) then
          begin
            Connection.HeaderToSend.Free;
            Connection.ReceivedHeader.Free;
          end;
          Connection.Free;
        end;
      finally
        LocalEventLoop.Free;
      end;
    finally
      Socket.Free;
    end;
  finally
    FreeAndNil(RequestStream);
  end;

//  HTTPConnection.Post(ServerURL, RequestStream, ResponseStream);
  ProcessAnswer;
end;

procedure TXMLRPCClient.CallAsync(ACallback: TOnXMLRPCCallCompleted;
  const AProcName: String; AArgs: array of const);
begin
  CurCallback := ACallback;
  MakeRequest(AProcName, AArgs);
  ResponseStream := TMemoryStream.Create;
  if Assigned(OnBeginRPC) then
    OnBeginRPC(Self);

//  CurRPCThread := TRPCThread.Create(Self);
end;

procedure TXMLRPCClient.MakeRequest(const AProcName: String;
  AArgs: array of const);
var
  Writer: TXMLRPCWriter;
  Params: TXMLRPCParams;
  i: Integer;
begin
  Writer := TXMLRPCWriter.Create;
  try
    Params := Writer.CreateParams;
    try
      for i := Low(AArgs) to High(AArgs) do
        with AArgs[i] do
          case VType of
            vtInteger: Writer.AddParam(Params, Writer.CreateIntValue(VInteger));
            vtBoolean: Writer.AddParam(Params, Writer.CreateBooleanValue(VBoolean));
            vtChar: Writer.AddParam(Params, Writer.CreateStringValue(VChar));
            vtExtended: Writer.AddParam(Params, Writer.CreateDoubleValue(VExtended^));
            vtString: Writer.AddParam(Params, Writer.CreateStringValue(VString^));
            vtPChar: Writer.AddParam(Params, Writer.CreateStringValue(VPChar));
            vtWideChar: Writer.AddParam(Params, Writer.CreateStringValue(VWideChar));
            vtPWideChar: Writer.AddParam(Params, Writer.CreateStringValue(VPWideChar));
            vtAnsiString: Writer.AddParam(Params, Writer.CreateStringValue(String(VAnsiString)));
            // vtCurrency: ?
            // vtVariant: ?
            vtInt64: Writer.AddParam(Params, Writer.CreateIntValue(VInt64^));
          else
            raise Exception.Create('Unsupported data type in RPC argument list');
          end;
      Writer.WriteMethodCall(AProcName, Params);
      RequestStream := Writer.MakeStream;
    except
      Params.Free;
    end;
  finally
    Writer.Free;
  end;
end;

procedure TXMLRPCClient.ProcessAnswer;
var
  Parser: TXMLRPCParser;
begin
  ResponseStream.Position := 0;
  Parser := TXMLRPCParser.Create(ResponseStream);
  FreeAndNil(ResponseStream);
  try
    case Parser.GetPostType of
      xmlrpcFaultResponse:
}        {raise Exception.Create(Format('%d - %s', [Parser.GetNextInt,
          Parser.GetNextString]));}
{        raise Exception.Create('Fehler bei XML-RPC-Befehlsausführung');
      xmlrpcResponse:
        if Assigned(CurCallback) then
          CurCallback(Parser);
    else
      raise Exception.Create('Invalid response');
    end;
  finally
    Parser.Free;
    if Assigned(OnEndRPC) then
      OnEndRPC(Self);
  end;
end;

procedure TXMLRPCClient.StreamSent(Sender: TObject);
begin
//  LocalEventLoop.Break;
  Connection.Receive;
end;

procedure TXMLRPCClient.DataAvailable(Sender: TObject);
begin
  LocalEventLoop.Break;
end;
}
{$ENDIF NEVERTRUE}

// XML-RPC Server

procedure TXMLRPCServlet.DoPost(Req: THttpServletRequest;
  Resp: THttpServletResponse);
var
  Parser: TXMLRPCParser;
  Writer: TXMLRPCWriter;
  Path: TStringList;
  LastDot, i: Integer;
  s, PathStr: String;
  AnswerStream: TStream;
  ExecCall: Boolean;
begin
  Parser := TXMLRPCParser.Create(Req.InputStream);
  try
    if Parser.GetPostType <> xmlrpcMethodCall then
      exit;

    Resp.ContentType := 'text/xml';

    Writer := TXMLRPCWriter.Create;
    try
      try
        // ...Header auswerten und zum Dispatcher springen...
        PathStr := Parser.GetMethodName + '.';
        Path := TStringList.Create;
        try
          LastDot := 1;
          for i := 1 to Length(PathStr) do
            if PathStr[i] = '.' then
            begin
              Path.Add(UpperCase(Copy(PathStr, LastDot, i - LastDot)));
              LastDot := i + 1;
            end;
          ExecCall := True;
          if Assigned(OnCheckCall) then
            OnCheckCall(Parser, Path, ExecCall);
          if ExecCall then
            Dispatch(Parser, Writer, Path)
          else
            Writer.WriteFaultResponse(2, 'May not execute request');
        finally
          Path.Free;
        end;
      except
        on e: Exception do
        begin
          if Assigned(OnException) then
            OnException(e);
          Writer.WriteFaultResponse(2,
            'Execution error: ' + e.ClassName + ': ' + e.Message);
        end;
      end;

      AnswerStream := Writer.MakeStream;
      try
        Resp.ContentLength := AnswerStream.Size;
        Resp.OutputStream.CopyFrom(AnswerStream, AnswerStream.Size);
      finally
        AnswerStream.Free;
      end;
    finally
      Writer.Free;
    end;
  finally
    Parser.Free;
  end;
end;


end.
