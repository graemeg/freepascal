{
    $Id: header,v 1.1 2000/07/13 06:33:45 michael Exp $
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fphtml; 

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, htmlelements, htmlwriter, httpdefs, fphttp, db;

type
  THtmlEntities = (heHtml,heBody,heHead,heDiv,heParagraph);

const
  THtmlEntitiesClasses : array[THtmlEntities] of THTMLElementClass =
    (THTML_html, THTML_body, THTML_head, THTML_div, THTML_p);

type

  { THTMLContentProducer }

  THTMLContentProducer = Class(THTTPContentProducer)
  private
    FDocument: THTMLDocument;
    FElement: THTMLCustomElement;
    FWriter: THTMLWriter;
    procedure SetDocument(const AValue: THTMLDocument);
    procedure SetWriter(const AValue: THTMLWriter);
  Protected
    function CreateWriter (Doc : THTMLDocument) : THTMLWriter; virtual;
  public
    function WriteContent (aWriter : THTMLWriter) : THTMLCustomElement; virtual; abstract;
    Function ProduceContent : String; override; // Here to test the output. Replace to protected after tests
    property ParentElement : THTMLCustomElement read FElement write FElement;
    property Writer : THTMLWriter read FWriter write SetWriter;
    Property HTMLDocument : THTMLDocument read FDocument write SetDocument;
  end;

  TWriterElementEvent = procedure (Sender:THTMLContentProducer; aWriter : THTMLWriter; var anElement : THTMLCustomElement) of object;
  TAfterElementEvent = procedure (Sender:THTMLContentProducer; anElement : THTMLCustomElement) of object;
  TWriterEvent = procedure (Sender:THTMLContentProducer; aWriter : THTMLWriter) of object;
  TBooleanEvent = procedure (Sender:THTMLContentProducer; var flag : boolean) of object;

  { THTMLCustomEntityProducer }

  THTMLCustomEntityProducer = class (THTMLContentProducer)
  private
    FOnWritePage: TWriterEvent;
    FEntity: THtmlEntities;
  protected
    function WriteContent (aWriter : THTMLWriter) : THTMLCustomElement; override;
    procedure DoWritePage (aWriter : THTMLWriter); virtual;
  public
    constructor Create(AOwner: TComponent); override;
    Property OnWritePage : TWriterEvent read FOnWritePage write FOnWritePage;
    Property Entity : THtmlEntities read FEntity write FEntity default heHtml;
  end;

  { THTMLCustomPagContentProducer }

  THTMLEntityProducer = class (THTMLCustomEntityProducer)
  published
    Property OnWritePage;
    Property Entity;
  end;

  { THTMLCustomDatasetContentProducer }

  THTMLCustomDatasetContentProducer = class (THTMLContentProducer)
  private
    FDatasource: TDatasource;
    FOnWriteFooter: TWriterEvent;
    FOnWriteHeader: TWriterElementEvent;
    FOnWriteRecord: TWriterEvent;
    function WriteHeader (aWriter : THTMLWriter) : THTMLCustomElement;
    procedure WriteFooter (aWriter : THTMLWriter);
    procedure WriteRecord (aWriter : THTMLWriter);
  protected
    function WriteContent (aWriter : THTMLWriter) : THTMLCustomElement; override;
    procedure DoWriteHeader (aWriter : THTMLWriter; var el : THTMLCustomElement); virtual;
    procedure DoWriteFooter (aWriter : THTMLWriter); virtual;
    procedure DoWriteRecord (aWriter : THTMLWriter); virtual;
  public
    Property OnWriteHeader : TWriterElementEvent read FOnWriteHeader write FOnWriteHeader;
    Property OnWriteFooter : TWriterEvent read FOnWriteFooter write FOnWriteFooter;
    Property OnWriteRecord : TWriterEvent read FOnWriteRecord write FOnWriteRecord;
  published
    Property DataSource : TDataSource read FDataSource write FDataSource;
  end;

  { THTMLDatasetContentProducer }

  THTMLDatasetContentProducer = class (THTMLCustomDatasetContentProducer)
  published
    Property OnWriteHeader;
    Property OnWriteFooter;
    Property OnWriteRecord;
  end;
  
  { THTMLSelectProducer }

  THTMLSelectProducer = class (THTMLContentProducer)
  private
    FControlName: string;
    FItems: TStrings;
    FPreSelected: string;
    FSize: integer;
    FUseValues: boolean;
    procedure SetItems(const AValue: TStrings);
  protected
    function WriteContent (aWriter : THTMLWriter) : THTMLCustomElement; override;
  public
    constructor create (aOwner : TComponent); override;
    destructor destroy; override;
  published
    property Items : TStrings read FItems write SetItems;
    property UseValues : boolean read FUseValues write FUseValues default false;
    property PreSelected : string read FPreSelected write FPreSelected;
    property Size : integer read FSize write FSize default 1;
    property ControlName : string read FControlName write FControlName;
  end;

  { THTMLDatasetSelectProducer }

  THTMLDatasetSelectProducer = class (THTMLCustomDatasetContentProducer)
  private
    FControlName: string;
    FIsPreSelected: TBooleanEvent;
    FItemField: string;
    FSize: integer;
    FValueField: string;
    FValue, FItem : TField;
    FPreSelected: string;
    FUseValues: boolean;
  protected
    procedure DoWriteHeader (aWriter : THTMLWriter; var el : THTMLCustomElement); override;
    procedure DoWriteFooter (aWriter : THTMLWriter); override;
    procedure DoWriteRecord (aWriter : THTMLWriter); override;
  public
    constructor create (aOwner : TComponent); override;
  published
    property UseValues : boolean read FUseValues write FUseValues default false;
    property PreSelected : string read FPreSelected write FPreSelected;
    property ItemField : string read FItemField write FItemField;
    property ValueField : string read FValueField write FValueField;
    property OnIsPreSelected : TBooleanEvent read FIsPreSelected write FIsPreSelected;
    property Size : integer read FSize write FSize;
    property ControlName : string read FControlName write FControlName;
    property OnWriteHeader;
  end;
  
  { THTMLDataModule }
  THTMLGetContentEvent = Procedure (Sender : TObject; ARequest : TRequest; HTMLPage : THTMLWriter; Var Handled : Boolean) of object;
  TCreateDocumentEvent = Procedure(Sender : TObject; var ADocument : THTMLDocument) of object;
  TCreateWriterEvent = Procedure(Sender : TObject; ADocument : THTMLDocument; Var AWriter : THTMLWriter) of object;

  { THTMLContentAction }

  THTMLContentAction = Class(TCustomWebAction)
  private
    FOnGetContent: THTMLGetContentEvent;
  Public
    Procedure HandleRequest(ARequest : TRequest; HTMLPage : THTMLWriter; Var Handled : Boolean);
  Published
    Property OnGetContent : THTMLGetContentEvent Read FOnGetContent Write FOnGetContent;
  end;
  
  { THTMLContentActions }

  THTMLContentActions = Class(TCustomWebActions)
    Procedure HandleRequest(ARequest : TRequest; HTMLPage : THTMLWriter; Var Handled : Boolean);
  end;


  { TCustomHTMLDataModule }

  { TCustomHTMLModule }

  TCustomHTMLModule = Class(TCustomHTTPModule)
  private
    FDocument : THTMLDocument;
    FActions: THTMLContentActions;
    FOnCreateDocument: TCreateDocumentEvent;
    FOnCreateWriter: TCreateWriterEvent;
    FOnGetContent: THTMLGetContentEvent;
    procedure SetActions(const AValue: THTMLContentActions);
  Protected
    Function CreateWriter(ADocument : THTMLDocument) : THTMLWriter;
    Function CreateDocument : THTMLDocument;
    Property OnGetContent : THTMLGetContentEvent Read FOnGetContent Write FOnGetContent;
    Property Actions : THTMLContentActions Read FActions Write SetActions;
    Property OnCreateDocument : TCreateDocumentEvent Read FOnCreateDocument Write FOnCreateDocument;
    Property OnCreateWriter : TCreateWriterEvent Read FOnCreateWriter Write FOnCreateWriter;
  Public
    Constructor Create(AOwner : TComponent);override;
    Procedure HandleRequest(ARequest : TRequest; AResponse : TResponse); override;
  end;
  
  TFPHTMLModule=Class(TCustomHTMLModule)
  Published
    Property OnGetContent;
    Property Actions;
    Property OnCreateDocument;
    Property OnCreateWriter;
  end;
  
  EHTMLError = Class(Exception);
  
implementation

{$ifdef cgidebug}
Uses dbugintf;
{$endif cgidebug}

resourcestring
  SErrRequestNotHandled = 'Web request was not handled by actions.';
  SErrNoContentProduced = 'The content producer "%s" didn''t produce any content.';

{ THTMLContentProducer }

procedure THTMLContentProducer.SetWriter(const AValue: THTMLWriter);
begin
  FWriter := AValue;
  if not assigned (FDocument) then
    FDocument := AValue.Document
  else if FDocument <> AValue.Document then
    AValue.document := FDocument;
end;

procedure THTMLContentProducer.SetDocument(const AValue: THTMLDocument);
begin
  FDocument := AValue;
  if assigned (FWriter) and (AValue <> FWriter.Document) then
    FWriter.Document := AValue;
end;

function THTMLContentProducer.ProduceContent: String;
var WCreated, created : boolean;
    el : THtmlCustomElement;
begin
  created := not assigned (FDocument);
  if created then
    FDocument := THTMLDocument.Create;
  try
    WCreated := not assigned(FWriter);
    if WCreated then
      FWriter := CreateWriter (FDocument);
    try
      FWriter.CurrentElement := ParentElement;
      el := WriteContent (FWriter);
      if not assigned(el) then
        Raise EHTMLError.CreateFmt(SErrNoContentProduced,[Self.Name]);
      result := el.asstring;
    finally
      if WCreated then
        FreeAndNil(FWriter);
    end;
  finally
    if created then
      FreeAndNil(FDocument);
  end;
end;

function THTMLContentProducer.CreateWriter (Doc : THTMLDocument): THTMLWriter;
begin
  FDocument := Doc;
  result := THTMLWriter.Create (Doc);
end;

{ THTMLCustomDatasetContentProducer }

function THTMLCustomDatasetContentProducer.WriteHeader(aWriter: THTMLWriter): THTMLCustomElement;
var el : THTmlCustomElement;
begin
  el := nil;
  DoWriteHeader (aWriter, el);
  result := el;
end;

procedure THTMLCustomDatasetContentProducer.WriteFooter(aWriter: THTMLWriter);
begin
  DoWriteFooter (aWriter);
end;

procedure THTMLCustomDatasetContentProducer.WriteRecord(aWriter: THTMLWriter);
begin
  DoWriteRecord (aWriter);
end;

function THTMLCustomDatasetContentProducer.WriteContent(aWriter: THTMLWriter): THTMLCustomElement;
var opened : boolean;
begin
  if assigned (FDataSource) and assigned(datasource.dataset) then
    begin
    result := WriteHeader (aWriter);
    try
        with FDataSource.dataset do
          try
            opened := Active;
            if not opened then
              Open;
            first;
            while not eof do
              begin
              WriteRecord(aWriter);
              next;
              end;
          finally
            if opened then
              close;
          end;
    finally
      WriteFooter (aWriter);
    end;
    end;
end;

procedure THTMLCustomDatasetContentProducer.DoWriteHeader(aWriter: THTMLWriter; var el : THTMLCustomElement);
begin
  if assigned (FOnWriteHeader) then
    FOnWriteHeader (self, aWriter, el);
end;

procedure THTMLCustomDatasetContentProducer.DoWriteFooter(aWriter: THTMLWriter);
begin
  if assigned (FOnWriteFooter) then
    FOnWriteFooter (self, aWriter);
end;

procedure THTMLCustomDatasetContentProducer.DoWriteRecord(aWriter: THTMLWriter);
begin
  if assigned (FOnWriteRecord) then
    FOnWriteRecord (self, aWriter);
end;

{ THTMLSelectProducer }

procedure THTMLSelectProducer.SetItems(const AValue: TStrings);
begin
  if FItems<>AValue then
    FItems.assign(AValue);
end;

function THTMLSelectProducer.WriteContent(aWriter: THTMLWriter): THTMLCustomElement;
begin
  result := aWriter.FormSelect(FControlName, FPreselected, FSize, FItems, FUseValues);
end;

constructor THTMLSelectProducer.create(aOwner: TComponent);
begin
  inherited create (aOwner);
  FUseValues := False;
  FItems := TStringlist.Create;
  size := 1;
end;

destructor THTMLSelectProducer.destroy;
begin
  FItems.Free;
  inherited;
end;

{ THTMLDatasetSelectProducer }

procedure THTMLDatasetSelectProducer.DoWriteHeader (aWriter : THTMLWriter; var el : THTMLCustomElement);
var s : THTML_Select;
begin
  s := aWriter.StartSelect;
  s.size := IntToStr(FSize);
  s.name := FControlName;
  el := s;
  if FValueField <> '' then
    FValue := datasource.dataset.findfield (FValueField);
  if FItemField <> '' then
    FItem := DataSource.dataset.findfield (FItemField);
  inherited DoWriteHeader(aWriter, el);
end;

procedure THTMLDatasetSelectProducer.DoWriteFooter(aWriter: THTMLWriter);
begin
  inherited DoWriteFooter(aWriter);
  aWriter.EndSelect;
end;

procedure THTMLDatasetSelectProducer.DoWriteRecord(aWriter: THTMLWriter);
var sel : boolean;
begin
  if assigned (FItem) then
    with aWriter.Option(FItem.asstring) do
      begin
      if FUseValues then
        sel := (FValue.AsString = FPreSelected)
      else
        sel := (FItem.AsString = FPreSelected);
      if assigned (FIsPreSelected) then
        FIsPreSelected (self, sel);
      selected := sel;
      if assigned (FValue) then
        Value := FValue.Asstring;
      end;
end;

constructor THTMLDatasetSelectProducer.create(aOwner: TComponent);
begin
  inherited create(aOwner);
  Size := 1;
  FUseValues := False;
end;

{ TCustomHTMLDataModule }

Function TCustomHTMLModule.CreateDocument : THTMLDocument;

begin
  If Assigned(FOnCreateDocument) then
    FOnCreateDocument(Self,Result);
  If (Result=Nil) then
    Result:=THTMLDocument.Create;
end;

constructor TCustomHTMLModule.Create(AOwner: TComponent);
begin
  FActions:=THTMLContentActions.Create(THTMLContentAction);
  inherited Create(AOwner);
end;

procedure TCustomHTMLModule.SetActions(const AValue: THTMLContentActions);
begin

end;

Function TCustomHTMLModule.CreateWriter(ADocument : THTMLDocument) : THTMLWriter;

begin
  If Assigned(FOnCreateWriter) then
    FOnCreateWriter(Self,ADocument,Result);
  if (Result=Nil) then
    Result:=THTMLWriter.Create(ADocument);
end;


procedure TCustomHTMLModule.HandleRequest(ARequest: TRequest; AResponse: TResponse);

Var
  FWriter : THTMLWriter;
  B : Boolean;
  M : TMemoryStream;
  
begin
  FDocument := CreateDocument;
  Try
    FWriter:=CreateWriter(FDocument);
    Try
      B:=False;
      If Assigned(OnGetContent) then
        OnGetContent(Self,ARequest,FWriter,B);
      If Not B then
        Raise EHTMLError.Create(SErrRequestNotHandled);
      If (AResponse.ContentStream=Nil) then
        begin
        M:=TMemoryStream.Create;
        AResponse.ContentStream:=M;
        end;
      FDocument.SaveToStream(AResponse.ContentStream);
    Finally
      FreeAndNil(FWriter);
    end;
  Finally
    FreeAndNil(FDocument);
  end;
end;

{ THTMLContentActions }

procedure THTMLContentActions.HandleRequest(ARequest: TRequest;
  HTMLPage: THTMLWriter; var Handled: Boolean);
  
Var
  A : TCustomWebAction;

begin
{$ifdef cgidebug}SendMethodEnter('HTMLContentWebActions.handlerequest');{$endif cgidebug}
  A:=GetRequestAction(ARequest);
  if Assigned(A) then
    (A as THTMLContentAction).HandleRequest(ARequest,HTMLPage,Handled);
{$ifdef cgidebug}SendMethodEnter('HTMLContentWebActions.handlerequest');{$endif cgidebug}
end;


{ THTMLContentAction }

procedure THTMLContentAction.HandleRequest(ARequest: TRequest;
  HTMLPage: THTMLWriter; var Handled: Boolean);
begin
  If Assigned(FOngetContent) then
    FOnGetContent(Self,ARequest,HTMLPage,Handled);
end;

{ THTMLCustomEntityProducer }

function THTMLCustomEntityProducer.WriteContent(aWriter: THTMLWriter
  ): THTMLCustomElement;
begin
  result := aWriter.StartElement(THtmlEntitiesClasses[FEntity]);
  DoWritePage(aWriter);
  aWriter.EndElement(THtmlEntitiesClasses[FEntity]);
end;

procedure THTMLCustomEntityProducer.DoWritePage(aWriter: THTMLWriter);
begin
  if assigned (FOnWritePage) then
    FOnWritePage (self, aWriter);
end;

constructor THTMLCustomEntityProducer.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FEntity := heHtml;
end;

end.

