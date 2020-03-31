program htmlwithsax;

uses sysutils, classes, sax,sax_html, custapp;

Type

  { TMyApp }

  TMyApp = Class(TCustomApplication)
  Private
    Indent : string;
    procedure DoEndDocument(Sender: TObject);
    procedure DoEndElement(Sender: TObject; const NamespaceURI, LocalName, QName: SAXString);
    procedure DoFile(const aFileName: String);
    procedure DoStartDocument(Sender: TObject);
    procedure DoStartElement(Sender: TObject; const NamespaceURI, LocalName, QName: SAXString; Atts: TSAXAttributes);
  Protected
    Procedure DoRun; override;
  end;


{ TMyApp }

procedure TMyApp.DoFile(const aFileName : String);

var
  F : TFileStream;
  MyReader : THTMLReader;

begin
  MyReader:=Nil;
  F:=TFileStream.Create(aFileName,fmOpenRead or fmShareDenyWrite);
  try
    MyReader:=THTMLReader.Create;
    MyReader.OnStartDocument:=@DoStartDocument;
    MyReader.OnStartElement:=@DoStartElement;
    MyReader.OnEndElement:=@DoEndElement;
    MyReader.OnEndDocument:=@DoEndDocument;
    MyReader.ParseStream(F);
  finally
    FreeAndNil(MyReader);
    F.Free;
  end;
end;

procedure TMyApp.DoRun;

var
  I : Integer;

begin
  StopOnException:=True;
  Terminate;
  if ParamCount<1 then
    begin
    Writeln('Usage : ',ExtractFileName(ExeName),' <htmlfile1> [htmlfile2 [htmlfile3]]');
    Exit;
    end;
  for I:=1 to ParamCount do
      DoFile(Params[i]);
end;

procedure TMyApp.DoStartDocument(Sender: TObject);
begin
  Writeln('Document start');
  Indent:='';
end;

procedure TMyApp.DoEndElement(Sender: TObject; const NamespaceURI, LocalName, QName: SAXString);
begin
  Indent:=Copy(Indent,1,Length(Indent)-2);
end;

procedure TMyApp.DoEndDocument(Sender: TObject);
begin
  Writeln('Document end');
  Indent:='';
end;

procedure TMyApp.DoStartElement(Sender: TObject; const NamespaceURI, LocalName, QName: SAXString; Atts: TSAXAttributes);

Var
  I : Integer;
  S : unicodestring;

begin
  S:='';
  if Assigned(Atts) then
    for I:=0 to Atts.Length-1 do
      begin
      if S<>'' then S:=S+', ';
      S:=S+Atts.LocalNames[i];
      end;
  Write(Indent,'Tag: <',LocalName,'>');
  if NameSpaceURI<>'' then
    Write(' xmlns: ',NameSpaceURI);
  if QName<>'' then
    Write(', full tag: ',QName);
  If S<>'' then
    Write(', attrs: ',S);
  Writeln;
  Indent:=Indent+'  ';
end;



begin
  With TMyApp.Create(Nil) do
    try
      Initialize;
      Run;
    finally
      Free;
    end;

end.

