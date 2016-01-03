{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2008 by Joost van der Sluis, member of the
    Free Pascal development team

    TXMLDatapacketReader implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit XMLDatapacketReader;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Bufdataset, dom, db;

type
  TChangeLogEntry = record
       UpdateKind : TUpdateKind;
       OrigEntry  : integer;
       NewEntry   : integer;
  end;
  TChangeLogEntryArr = array of TChangeLogEntry;

type
  { TXMLDatapacketReader }

  TXMLDatapacketReader = class(TDataPacketReader)
    XMLDocument    : TXMLDocument;
    DataPacketNode : TDOMElement;
    MetaDataNode   : TDOMNode;
    FieldsNode     : TDOMNode;
    FChangeLogNode,
    FParamsNode,
    FRowDataNode,
    FRecordNode    : TDOMNode;
    FChangeLog     : TChangeLogEntryArr;
    FEntryNr       : integer;
    FLastChange    : integer;
  public
    destructor destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure StoreFieldDefs(AnAutoIncValue : integer); override;
    procedure StoreRecord(ARowState : TRowState; AUpdOrder : integer = 0); override;
=======
    procedure StoreFieldDefs(AFieldDefs : TFieldDefs); override;
    procedure StoreRecord(ADataset : TCustomBufDataset; ARowState : TRowState; AUpdOrder : integer = 0); override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure FinalizeStoreRecords; override;
    procedure LoadFieldDefs(var AnAutoIncValue : integer); override;
    procedure InitLoadRecords; override;
    function GetCurrentRecord : boolean; override;
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure RestoreRecord; override;
=======
    procedure RestoreRecord(ADataset : TCustomBufDataset); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure RestoreRecord(ADataset : TCustomBufDataset); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure RestoreRecord(ADataset : TCustomBufDataset); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure RestoreRecord(ADataset : TCustomBufDataset); override;
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    procedure StoreFieldDefs(AFieldDefs : TFieldDefs); override;
    procedure StoreRecord(ADataset : TBufDataset; ARowState : TRowState; AUpdOrder : integer = 0); override;
    procedure FinalizeStoreRecords; override;
    procedure LoadFieldDefs(AFieldDefs : TFieldDefs); override;
    procedure InitLoadRecords; override;
    function GetCurrentRecord : boolean; override;
    function GetRecordRowState(out AUpdOrder : Integer) : TRowState; override;
    procedure RestoreRecord(ADataset : TBufDataset); override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure GotoNextRecord; override;
    class function RecognizeStream(AStream : TStream) : boolean; override;
  end;

implementation

<<<<<<< HEAD
<<<<<<< HEAD
uses xmlwrite, xmlread, base64;

const
  XMLFieldtypenames : Array [TFieldType] of String[16] =
=======
=======
>>>>>>> origin/fixes_2_2
uses xmlwrite, xmlread;

const
  XMLFieldtypenames : Array [TFieldType] of String[15] =
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    (
      'Unknown',
      'string',
      'i2',
      'i4',
      'i4',
      'boolean',
      'r8',
<<<<<<< HEAD
<<<<<<< HEAD
      'r8:Money',
=======
      'r8',
>>>>>>> graemeg/fixes_2_2
=======
      'r8',
>>>>>>> origin/fixes_2_2
      'fixed',
      'date',
      'time',
      'datetime',
      'bin.hex',
      'bin.hex',
<<<<<<< HEAD
<<<<<<< HEAD
      'i4:Autoinc',
      'bin.hex:Binary',
      'bin.hex:Text',
      'bin.hex:Graphics',
      'bin.hex:Formatted',
      'bin.hex:Ole',
      'bin.hex:Ole',
      'bin.hex:Graphics',
      '',
      'string',             // ftFixedChar
      'string.uni',         // ftWideString
=======
=======
>>>>>>> origin/fixes_2_2
      'i4',
      'bin.hex',
      'bin.hex',
      'bin.hex',
      'bin.hex',
      'bin.hex',
      'bin.hex',
      'bin.hex',
      '',
      'string',
      'string',
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      'i8',
      '',
      '',
      '',
      '',
      '',
      '',
      '',
      '',
      '',
<<<<<<< HEAD
<<<<<<< HEAD
      'string:Guid',        // ftGuid
      '',
      'fixedFMT',           // ftFmtBCD
      'string.uni',         // ftFixedWideChar
      'bin.hex:WideText'    // ftWideMemo
=======
=======
>>>>>>> origin/fixes_2_2
      '',
      '',
      '',
      '',
      ''
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    );

resourcestring
  sUnknownXMLDatasetFormat = 'Unknown XML Dataset format';

{ TXMLDatapacketReader }

destructor TXMLDatapacketReader.destroy;
begin
  FieldsNode.Free;
  MetaDataNode.Free;
  DataPacketNode.Free;
  XMLDocument.Free;
  inherited destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLDatapacketReader.LoadFieldDefs(var AnAutoIncValue: integer);
=======
procedure TXMLDatapacketReader.LoadFieldDefs(AFieldDefs : TFieldDefs);
>>>>>>> graemeg/fixes_2_2
=======
procedure TXMLDatapacketReader.LoadFieldDefs(AFieldDefs : TFieldDefs);
>>>>>>> origin/fixes_2_2

  function GetNodeAttribute(const aNode : TDOMNode; AttName : String) : string;
  var AnAttr : TDomNode;
  begin
    AnAttr := ANode.Attributes.GetNamedItem(AttName);
    if assigned(AnAttr) then result := AnAttr.NodeValue
    else result := '';
  end;

var i           : integer;
    AFieldDef   : TFieldDef;
    iFieldType  : TFieldType;
    FTString    : string;
<<<<<<< HEAD
<<<<<<< HEAD
    SubFTString : string;
    AFieldNode  : TDOMNode;
    AnAutoIncNode: TDomNode;
=======
    AFieldNode  : TDOMNode;
>>>>>>> graemeg/fixes_2_2
=======
    AFieldNode  : TDOMNode;
>>>>>>> origin/fixes_2_2

begin
  ReadXMLFile(XMLDocument,Stream);
  DataPacketNode := XMLDocument.FindNode('DATAPACKET') as TDOMElement;
  if not assigned(DataPacketNode) then DatabaseError(sUnknownXMLDatasetFormat);

  MetaDataNode := DataPacketNode.FindNode('METADATA');
  if not assigned(MetaDataNode) then DatabaseError(sUnknownXMLDatasetFormat);

  FieldsNode := MetaDataNode.FindNode('FIELDS');
  if not assigned(FieldsNode) then DatabaseError(sUnknownXMLDatasetFormat);

  with FieldsNode.ChildNodes do for i := 0 to Count - 1 do
    begin
    AFieldNode := item[i];
    if AFieldNode.CompareName('FIELD')=0 then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
      AFieldDef := Dataset.FieldDefs.AddFieldDef;
=======
      AFieldDef := TFieldDef.create(AFieldDefs);
>>>>>>> graemeg/fixes_2_2
=======
      AFieldDef := TFieldDef.create(AFieldDefs);
>>>>>>> origin/fixes_2_2
      AFieldDef.DisplayName:=GetNodeAttribute(AFieldNode,'fieldname');
      AFieldDef.Name:=GetNodeAttribute(AFieldNode,'attrname');
      AFieldDef.Size:=StrToIntDef(GetNodeAttribute(AFieldNode,'width'),0);
      FTString:=GetNodeAttribute(AFieldNode,'fieldtype');
<<<<<<< HEAD
<<<<<<< HEAD
      SubFTString:=GetNodeAttribute(AFieldNode,'subtype');
      if SubFTString<>'' then
        FTString:=FTString+':'+SubFTString;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

      AFieldDef.DataType:=ftUnknown;
      for iFieldType:=low(TFieldType) to high(TFieldType) do
       if SameText(XMLFieldtypenames[iFieldType],FTString) then
        begin
        AFieldDef.DataType:=iFieldType;
        break;
        end;
      end;
    end;

<<<<<<< HEAD
<<<<<<< HEAD
  FParamsNode := MetaDataNode.FindNode('PARAMS');
  if assigned(FParamsNode) then
    begin
    FChangeLogNode := FParamsNode.Attributes.GetNamedItem('CHANGE_LOG');
    AnAutoIncNode := FParamsNode.Attributes.GetNamedItem('AUTOINCVALUE');
    if assigned(AnAutoIncNode) then
      AnAutoIncValue := StrToIntDef(AnAutoIncNode.NodeValue,-1);
    end;
=======
  FChangeLogNode := MetaDataNode.FindNode('PARAMS');
  if assigned(FChangeLogNode) then
    FChangeLogNode := FChangeLogNode.Attributes.GetNamedItem('CHANGE_LOG');
>>>>>>> graemeg/fixes_2_2
=======
  FChangeLogNode := MetaDataNode.FindNode('PARAMS');
  if assigned(FChangeLogNode) then
    FChangeLogNode := FChangeLogNode.Attributes.GetNamedItem('CHANGE_LOG');
>>>>>>> origin/fixes_2_2

  FRowDataNode := DataPacketNode.FindNode('ROWDATA');
  FRecordNode := nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLDatapacketReader.StoreFieldDefs(AnAutoIncValue: integer);

var i,p         : integer;
    AFieldNode  : TDOMElement;
    AStringFT   : string;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TXMLDatapacketReader.StoreFieldDefs(AFieldDefs: TFieldDefs);

var i           : integer;
    AFieldNode  : TDOMElement;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  XMLDocument := TXMLDocument.Create;
  DataPacketNode := XMLDocument.CreateElement('DATAPACKET');
  DataPacketNode.SetAttribute('Version','2.0');

  MetaDataNode := XMLDocument.CreateElement('METADATA');
  FieldsNode := XMLDocument.CreateElement('FIELDS');

<<<<<<< HEAD
<<<<<<< HEAD
  for i := 0 to DataSet.FieldDefs.Count - 1 do with DataSet.FieldDefs[i] do
=======
  for i := 0 to AFieldDefs.Count -1 do with AFieldDefs[i] do
>>>>>>> graemeg/fixes_2_2
=======
  for i := 0 to AFieldDefs.Count -1 do with AFieldDefs[i] do
>>>>>>> origin/fixes_2_2
    begin
    AFieldNode := XMLDocument.CreateElement('FIELD');
    if Name <> '' then AFieldNode.SetAttribute('fieldname',Name);
    AFieldNode.SetAttribute('attrname',DisplayName);
    if size <> 0 then AFieldNode.SetAttribute('width',IntToStr(Size));
<<<<<<< HEAD
<<<<<<< HEAD
    AStringFT:=XMLFieldtypenames[DataType];
    p := pos(':',AStringFT);
    if p > 1 then
      begin
      AFieldNode.SetAttribute('fieldtype',copy(AStringFT,1,p-1));
      AFieldNode.SetAttribute('subtype',copy(AStringFT,p+1,25));
      end
    else
      AFieldNode.SetAttribute('fieldtype',AStringFT);
=======
=======
>>>>>>> origin/fixes_2_2
    AFieldNode.SetAttribute('fieldtype',XMLFieldtypenames[DataType]);
    case DataType of
      ftAutoInc : begin
                  AFieldNode.SetAttribute('readonly','true');
                  AFieldNode.SetAttribute('subtype','Autoinc');
                  end;
      ftCurrency: AFieldNode.SetAttribute('subtype','Money');
      ftVarBytes,
        ftBlob  : AFieldNode.SetAttribute('subtype','Binary');
      ftMemo    : AFieldNode.SetAttribute('subtype','Text');
      ftTypedBinary,
        ftGraphic: AFieldNode.SetAttribute('subtype','Graphics');
      ftFmtMemo : AFieldNode.SetAttribute('subtype','Formatted');
      ftParadoxOle,
        ftDBaseOle : AFieldNode.SetAttribute('subtype','Ole');
    end; {case}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    if faReadonly in Attributes then AFieldNode.SetAttribute('readonly','true');

    FieldsNode.AppendChild(AFieldNode);
    end;

  MetaDataNode.AppendChild(FieldsNode);
  FParamsNode := XMLDocument.CreateElement('PARAMS');
<<<<<<< HEAD
<<<<<<< HEAD
  if AnAutoIncValue>-1 then
    (FParamsNode as TDomElement).SetAttribute('AUTOINCVALUE',IntToStr(AnAutoIncValue));

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  MetaDataNode.AppendChild(FParamsNode);
  DataPacketNode.AppendChild(MetaDataNode);
  FRowDataNode := XMLDocument.CreateElement('ROWDATA');
  setlength(FChangeLog,0);
  FEntryNr:=0;
  FLastChange:=-1;
end;

procedure TXMLDatapacketReader.FinalizeStoreRecords;
var ChangeLogStr : String;
    i            : integer;
begin
  ChangeLogStr:='';
  for i := 0 to length(FChangeLog)-1 do with FChangeLog[i] do
    begin
    ChangeLogStr:=ChangeLogStr+' '+inttostr(NewEntry)+' '+inttostr(OrigEntry)+' ';
    if UpdateKind=ukModify then ChangeLogStr := ChangeLogStr+'8';
    if UpdateKind=ukInsert then ChangeLogStr := ChangeLogStr+'4';
    if UpdateKind=ukDelete then ChangeLogStr := ChangeLogStr+'2';
    end;
  setlength(FChangeLog,0);

  if ChangeLogStr<>'' then
    (FParamsNode as TDomElement).SetAttribute('CHANGE_LOG',Trim(ChangeLogStr));

  DataPacketNode.AppendChild(FRowDataNode);
  XMLDocument.AppendChild(DataPacketNode);

  WriteXML(XMLDocument,Stream);
end;

function TXMLDatapacketReader.GetCurrentRecord: boolean;
begin
  Result := assigned(FRecordNode);
end;

function TXMLDatapacketReader.GetRecordRowState(out AUpdOrder: Integer
  ): TRowState;
var ARowStateNode  : TDOmNode;
    ARowState      : integer;
    i              : integer;
begin
  ARowStateNode := FRecordNode.Attributes.GetNamedItem('RowState');
  if ARowStateNode = nil then // This item is not edited
    Result := []
  else
    begin
    Result := ByteToRowState(StrToIntDef(ARowStateNode.NodeValue,0));
    if Result = [rsvOriginal] then
      begin
      for i := 0 to length(FChangeLog)-1 do
        if FChangeLog[i].NewEntry=FEntryNr then break;
      assert(FChangeLog[i].NewEntry=FEntryNr);
      end
    else
      begin
      for i := 0 to length(FChangeLog)-1 do
        if FChangeLog[i].OrigEntry=FEntryNr then break;
      assert(FChangeLog[i].OrigEntry=FEntryNr);
      end;
    AUpdOrder:=i;
    end;
end;

procedure TXMLDatapacketReader.InitLoadRecords;

var ChangeLogStr : String;
    i,cp         : integer;
    ps           : string;

begin
  FRecordNode := FRowDataNode.FirstChild;
  FEntryNr := 1;
  setlength(FChangeLog,0);
  if assigned(FChangeLogNode) then
    ChangeLogStr:=FChangeLogNode.NodeValue
  else
    ChangeLogStr:='';
  ps := '';
  cp := 0;
  if ChangeLogStr<>'' then for i := 1 to length(ChangeLogStr)+1 do
    begin
    if not (ChangeLogStr[i] in [' ',#0]) then
      ps := ps + ChangeLogStr[i]
    else
      begin
      case (cp mod 3) of
        0 : begin
            SetLength(FChangeLog,length(FChangeLog)+1);
            FChangeLog[cp div 3].OrigEntry:=StrToIntDef(ps,0);
            end;
        1 : FChangeLog[cp div 3].NewEntry:=StrToIntDef(ps,0);
        2 : begin
            if ps = '2' then
              FChangeLog[cp div 3].UpdateKind:=ukDelete
            else if ps = '4' then
              FChangeLog[cp div 3].UpdateKind:=ukInsert
            else if ps = '8' then
              FChangeLog[cp div 3].UpdateKind:=ukModify;
            end;
      end; {case}
      ps := '';
      inc(cp);
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLDatapacketReader.RestoreRecord;
var FieldNr      : integer;
    AFieldNode   : TDomNode;
    ABufBlobField: TBufBlobField;
    AField: TField;
    s: string;
    ws: widestring;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TXMLDatapacketReader.RestoreRecord(ADataset : TCustomBufDataset);
var FieldNr    : integer;
    AFieldNode : TDomNode;
>>>>>>> graemeg/cpstrnew
begin
  with DataSet do for FieldNr:=0 to FieldDefs.Count-1 do
    begin
    AField := Fields.FieldByNumber(FieldDefs[FieldNr].FieldNo);
    AFieldNode := FRecordNode.Attributes.GetNamedItem(FieldDefs[FieldNr].Name);
    if assigned(AFieldNode) then
      begin
      s := AFieldNode.NodeValue;
      if (FieldDefs[FieldNr].DataType in [ftBlob, ftBytes, ftVarBytes]) and (s <> '') then
        s := DecodeStringBase64(s);
      case FieldDefs[FieldNr].DataType of
        ftBlob, ftMemo:
          RestoreBlobField(AField, @s[1], length(s));
        ftWideMemo:
          begin
          ws := s;
          RestoreBlobField(AField, @ws[1], length(ws)*sizeof(WideChar));
          end
        else;
          AField.AsString := s;  // set it to the filterbuffer
      end;
      end
    else
      AField.SetData(nil);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TXMLDatapacketReader.StoreRecord(ARowState : TRowState; AUpdOrder : integer = 0);
=======
procedure TXMLDatapacketReader.StoreRecord(ADataset : TCustomBufDataset; ARowState : TRowState; AUpdOrder : integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLDatapacketReader.StoreRecord(ADataset : TCustomBufDataset; ARowState : TRowState; AUpdOrder : integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLDatapacketReader.StoreRecord(ADataset : TCustomBufDataset; ARowState : TRowState; AUpdOrder : integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure TXMLDatapacketReader.StoreRecord(ADataset : TCustomBufDataset; ARowState : TRowState; AUpdOrder : integer = 0);
>>>>>>> origin/cpstrnew
var FieldNr : Integer;
    AFieldDef: TFieldDef;
    AField: TField;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TXMLDatapacketReader.RestoreRecord(ADataset : TBufDataset);
var FieldNr    : integer;
    AFieldNode : TDomNode;
begin
  with ADataset do for FieldNr:=0 to FieldCount-1 do
    begin
    AFieldNode := FRecordNode.Attributes.GetNamedItem(Fields[FieldNr].FieldName);
    if assigned(AFieldNode) then
      begin
      Fields[FieldNr].AsString := AFieldNode.NodeValue;  // set it to the filterbuffer
      end
    end;
end;

procedure TXMLDatapacketReader.StoreRecord(ADataset : TBufDataset; ARowState : TRowState; AUpdOrder : integer = 0);
var FieldNr : Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    ARecordNode : TDOMElement;
begin
  inc(FEntryNr);
  ARecordNode := XMLDocument.CreateElement('ROW');
<<<<<<< HEAD
<<<<<<< HEAD
  with DataSet do for FieldNr := 0 to FieldDefs.Count-1 do
    begin
    AFieldDef := FieldDefs[FieldNr];
    AField := Fields.FieldByNumber(AFieldDef.FieldNo);
    if not AField.IsNull then
      if AFieldDef.DataType in [ftBlob, ftBytes, ftVarBytes] then
        ARecordNode.SetAttribute(AFieldDef.Name, EncodeStringBase64(AField.AsString))
      else
        ARecordNode.SetAttribute(AFieldDef.Name, AField.AsString);
=======
  for FieldNr := 0 to ADataset.Fields.Count-1 do
    begin
    ARecordNode.SetAttribute(ADataset.fields[FieldNr].FieldName,ADataset.fields[FieldNr].AsString);
>>>>>>> graemeg/fixes_2_2
=======
  for FieldNr := 0 to ADataset.Fields.Count-1 do
    begin
    ARecordNode.SetAttribute(ADataset.fields[FieldNr].FieldName,ADataset.fields[FieldNr].AsString);
>>>>>>> origin/fixes_2_2
    end;
  if ARowState<>[] then
    begin
    ARecordNode.SetAttribute('RowState',inttostr(RowStateToByte(ARowState)));
    if AUpdOrder>=length(FChangeLog) then
      setlength(FChangeLog,AUpdOrder+1);
    if (rsvOriginal in ARowState) or (rsvDeleted in ARowState) then
      FChangeLog[AUpdOrder].OrigEntry:=FEntryNr;
    if (rsvDeleted in ARowState) or (rsvUpdated in ARowState) or (rsvInserted in ARowState) then
      FChangeLog[AUpdOrder].NewEntry:=FEntryNr;
    if ARowState=[rsvUpdated] then
      FChangeLog[AUpdOrder].UpdateKind := ukModify;
    if ARowState=[rsvInserted] then
      FChangeLog[AUpdOrder].UpdateKind := ukInsert;
    if ARowState=[rsvDeleted] then
      FChangeLog[AUpdOrder].UpdateKind := ukDelete;
    end;
  FRowDataNode.AppendChild(ARecordNode);
end;

class function TXMLDatapacketReader.RecognizeStream(AStream: TStream): boolean;
const XmlStart = '<?xml';
var s        : string;
    len      : integer;
begin
  Len := length(XmlStart);
  setlength(s,len);
  if (AStream.Read (s[1],len) = len)
  and (s=XmlStart) then
    Result := True
  else
    Result := False;
end;

procedure TXMLDatapacketReader.GotoNextRecord;
begin
  FRecordNode := FRecordNode.NextSibling;
  inc(FEntryNr);
  while assigned(FRecordNode) and (FRecordNode.CompareName('ROW')<>0) do
    FRecordNode := FRecordNode.NextSibling;
end;

initialization
  RegisterDatapacketReader(TXMLDatapacketReader,dfXML);
end.
