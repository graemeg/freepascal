unit fpXMLXSDExport;

{Output XML with (or without) XSD data suitable for input in:
- databases (e.g. Microsoft Access, Microsoft SQL Server)
- spreadsheets (e.g. Microsoft Excel)
- .Net ADO.NET database applications
- Delphi (using ClientDataset)
}
{
Data type conversions I'm not sure about are marked with todo:
the export will probably work but might not give you the results you want. Patches welcome.
You'll need to modify both the part where the XSD datatype info is generated as well as the handling of data output.

I've added possible tasks to do using Lazarus to do list items; priority 1 (high) to 10 (low), categories Must have, Should have, Nice to have
I've marked source code with todo if there is something that could be looked at but seems not essential for functionality.

See the readme file for more details.

(c) Reinier Olislagers 2011, MIT license: all use permitted but no liability accepted.
Also licensed according to FreePascal/FCL license.
Take your pick, but don't blame me if something goes wrong ;)
}

{ TODO 7 -oAnybody -cNice to have : Apparently, ClientDatasets have insert new data and update data modes; as of now, only the insert new data mode is supported. Implement other modes.}
{ TODO 8 -oAnybody -cNice to have : Test Microsoft SQL Server xml import with Access format xml, using e.g. openxml or SSIS }
{ TODO 7 -oAnybody -cNice to have : Inherited settings
BooleanFalse
BooleanTrue
DateTimeFormat
CurrencyDigits
CurrencySymbol
DateFormat
IntegerFormat
TimeFormat
are not needed. Is there any way to remove them from the settings?}

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, DB, fpDBExport, DOM, XMLWrite, base64, typinfo, dateutils {$IFDEF Unix}, clocale{$ENDIF Unix};


type
  TXMLExportFormat = (AccessCompatible, ADONETCompatible, DelphiClientDataset,
    ExcelCompatible);
  {
  Possible export types:
  AccessCompatible should cover at least Microsoft Access XP/2002, 2003, 2007, and 2010
  ADONETCompatible should cover at least .Net framework 2, 3, 3.5 and 4. ADONETCompatible is the most clearcut, simple export, and could be use for other environments as well.
  DelphiClientDataset is based on TurboDelphi (Delphi 2006) unicode XML export; additions/improvements for other versions are welcome.
  ExcelCompatible should cover at least Microsoft Excel XP/2002, 2003, 2007, and 2010
  }

  { TODO 9 -oAnyone -cNice to have : If we can support databases that store timezone info with date/time, e.g. Oracle, add an option StoredInDatabase }
  { TXMLXSDFormatSettings }
  TXMLXSDFormatSettings = class(TExportFormatSettings)
  private
    FExportFormat: TXMLExportFormat;
    FXSDUsed: boolean;
  public
    procedure Assign(Source: TPersistent); override;
  published
    property CreateXSD: boolean read FXSDUsed write FXSDUsed;
    property ExportFormat: TXMLExportFormat read FExportFormat write FExportFormat;
  end;

  { TCustomXMLXSDExporter }
  TCustomXMLXSDExporter = class(TCustomFileExporter)
  const
    DefaultDatasetName = 'Table1';
    // Name used for the exported table/dataset if no other name can be found.
  private
    FNode: TDOMNode; //Just a placeholder for a node which can be reused
    FDatasetExportName: string; //Table name to be used in export
    FFormatSettingsOriginal: TFormatSettings;
    FOutputDoc: TXMLDocument; //The XML document being constructed
    FRootNode: TDOMNode; //Root node for XML doc
    FRowDataNode: TDOMNode;
    //Node at the beginning of each data row. Contains actual field data.
    FTableDataParentNode: TDOMNode;
    //dataroot element; Parent node for actual table data; named ROWDATA in DelphiClientDataset
    FXSDUsed: boolean;
    procedure ExportFieldDataAccess(const EF: Texportfielditem); //Export for Access
    procedure ExportFieldDataADO(const EF: Texportfielditem); //Export for ADO.Net
    procedure ExportFieldDataClientDataset(const EF: Texportfielditem);
    //Export for DelphiClientDataset
    procedure ExportFieldDataExcel(const EF: Texportfielditem);
    //Export for Access, ADO.Net
    function GetXMLFormatsettings: TXMLXSDFormatSettings;
    procedure SetXMLFormatSettings(const AValue: TXMLXSDFormatSettings);
  protected
    function CreateFormatSettings: TCustomExportFormatSettings; override;
    procedure DoBeforeExecute; override;
    procedure DoAfterExecute; override;
    procedure DoDataRowStart; override;
    procedure DoDataHeader; override;
    procedure DoDataHeaderAccess;
    procedure DoDataHeaderADO;
    procedure DoDataHeaderClientDataset;
    procedure DoDataHeaderExcel;
    procedure DoDataFooter; override;
    procedure DoDataRowEnd; override;
    procedure ExportField(EF: TExportFieldItem); override;
  public
    property FormatSettings: TXMLXSDFormatSettings
      read GetXMLFormatsettings write SetXMLFormatSettings;
  end;

  TXMLXSDExporter = class(TCustomXMLXSDExporter)
  published
    property FileName;
    property Dataset;
    property ExportFields;
    property FromCurrent;
    property RestorePosition;
    property FormatSettings;
    property OnExportRow;
  end;

procedure RegisterXMLXSDExportFormat;
procedure UnRegisterXMLXSDExportFormat;

const
  SXMLXSD = 'XMLXSD';
  SXMLXSDExtensions = '.xml';

resourcestring
  SXMLXSDDescription = 'Unicode XML file';

implementation


{ TCustomXMLXSDExporter }

function TCustomXMLXSDExporter.GetXMLFormatsettings: TXMLXSDFormatSettings;
begin
  Result := TXMLXSDFormatSettings(inherited FormatSettings);
end;

procedure TCustomXMLXSDExporter.ExportFieldDataAccess(const EF: Texportfielditem);
//Export for Access
{Ranges/limits:
Access:
http://office.microsoft.com/en-us/access-help/access-specifications-HP005186808.aspx
Number of characters in an table/column name   64
Number of fields in a table   255
Number of characters in a Text field   255
Number of characters in a Memo field   1 gigabyte of character storage
Size of an OLE Object field   1 gigabyte
http://msdn.microsoft.com/en-us/library/bb208866%28printer%29.aspx
Datetime: 8 bytes,   A date or time value between the years 100 and 9999.
Long integer: 4 bytes,    –2,147,483,648 and 2,147,483,647
}
var
  FieldHasData: boolean; //Should we output the field value or nothing (a null value)?
  Fieldnode: Tdomnode;
begin
  FieldHasData:=true;//Assume we've got data in our field.
  Fieldnode := Foutputdoc.CreateElement(Utf8decode(EF.Fieldname));
  //If you change the export types here, you'll also have to change the
  //metadata export in the XSD export section...
  if EF.Field.IsNull then
  begin
    FieldHasData:=false; //Indicate this is a null field.
  end
  else
  begin
    // Write data out to node depending on data type
    case EF.Field.Datatype of
      Ftarray:
      begin
        //ftArray: maybe Firebird array type. For now, pretend it's a string.
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftadt:
      begin
        //ftAdt: No idea what this does; pretend it's a string
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftautoinc:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftbcd:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftboolean:
      begin
        FNode :=
          Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftblob:
      begin
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftbytes:
      begin
        //ftbytes is some kind of blob field
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftcurrency:
      begin
        //make sure there's no currency symbol in there...
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftcursor:
      begin
        //No idea what this does; pretend it's a string.
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftdataset:
      begin
        //No idea what this does; pretend it's a string
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      ftDate:
      begin
        //Apparently no conversion of local time to UTC
        if (YearOf(EF.Field.AsDateTime) < 100) then
        begin
          //Too early, just save null data.
          FieldHasData:=false;
          { TODO 4 -oAnyone -cShould have : Generate warning to user that data is out of range }
        end
        else
        begin
          FNode := Foutputdoc.CreateTextNode(
            Formatdatetime('yyyy"-"mm"-"dd"T00:00:00"', EF.Field.AsDateTime));
        end;
      end;

      Ftdatetime:
      begin
        //Apparently no conversion of local time to UTC
        if (YearOf(EF.Field.AsDateTime) < 100) then
        begin
          //Too early, just save null data.
          FieldHasData:=false;
          { TODO 4 -oAnyone -cShould have : Generate warning to user that data is out of range }
        end
        else
        begin
          FNode := Foutputdoc.CreateTextNode(
            Formatdatetime('yyyy"-"mm"-"dd"T"hh":"nn":"ss', EF.Field.AsDateTime));
        end;
      end;

      Ftdbaseole:
      begin
        //Hope the FPC library gives the right version of Base64...
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftfixedchar:
      begin
        //Let's not assuma all blank field is actually null.
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftfixedwidechar:
      begin
        FNode :=
          Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      FtFloat:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      FtFmtbcd:
      begin
        //Assuming some kind of BCD/Binary Coded Decimal, so this might just work
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftfmtmemo:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftgraphic:
      begin
        //Hope the FPC library gives the right version of Base64...
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      ftGUID:
      begin
        //Apparently, ftGUID is stored in its string representation.
        FNode := Foutputdoc.CreateTextNode(Utf8Decode(EF.Field.AsString));
      end;

      Ftidispatch:
      begin
        //No idea what this does; pretend it's a string.
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftinteger:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftinterface:
      begin
        //No idea what this does; pretend it's a string.
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftlargeint:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftmemo:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftorablob:
      begin
        //Hope the FPC library gives the right version of Base64...
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftoraclob:
      begin
        //Hope the FPC library gives the right version of Base64...
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftparadoxole:
      begin
        //Hope the FPC library gives the right version of Base64...
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftreference:
      begin
        //No idea what this does; pretend it's a string
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftsmallint:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftstring:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Fttime:
      begin
        //Just guessing here. Maybe needs a date part, too.
        FNode := Foutputdoc.CreateTextNode(
          Formatdatetime('hh":"nn":"ss', EF.Field.AsDateTime));
      end;

      Fttimestamp:
      begin
        //Apparently no conversion of local time to UTC
        if (YearOf(EF.Field.AsDateTime) < 100) then
        begin
          //Too early, just save null data.
          FieldHasData:=false;
          { TODO 4 -oAnyone -cShould have : Generate warning to user that data is out of range }
        end
        else
        begin
          FNode := Foutputdoc.CreateTextNode(
            Formatdatetime('yyyy"-"mm"-"dd"T"hh":"nn":"ss', EF.Field.AsDateTime));
        end;
      end;

      Fttypedbinary:
      begin
        //Assume ftTypedBinary is just a binary/blob
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftvarbytes:
      begin
        //VariBytes: assume just a binary/blob...
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftvariant:
      begin
        // ftVariant: assume just a binary/blob...
        FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
      end;

      Ftwidememo:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftwidestring:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftword:
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;

      Ftunknown:
      begin
        //raise Exception.Create('Unknown datatype for dataset field while exporting.');
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;
      else
      begin
        //raise Exception.Create('Unknown datatype for dataset field while exporting.');
        FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
      end;
    end; //field type
  end;

  if (FieldHasData=true) then
  begin
    FieldNode.AppendChild(FNode);
    FRowDataNode.AppendChild(Fieldnode);
  end
  else
  begin
    //We have null data. Access wants to see
    //no field node at all in this case, so
    //let's not append the Fieldnode then.
  end;
end;

procedure TCustomXMLXSDExporter.ExportFieldDataADO(const EF: Texportfielditem);
//Export for ADO.Net
{Ranges/limits:
ADO.Net:
Date/time:
smallest 00:00:00.0000000, January 1, 0001 (http://msdn.microsoft.com/en-us/library/system.datetime.minvalue.aspx)
largest 23:59:59.9999999, December 31, 9999, exactly one 100-nanosecond tick before 00:00:00, January 1, 10000. http://msdn.microsoft.com/en-us/library/system.datetime.maxvalue.aspx
}
var
  Fieldnode: Tdomnode;
begin
  Fieldnode := Foutputdoc.CreateElement(Utf8decode(EF.Fieldname));
  //If you change the export types here, you'll also have to change the
  //metadata export in the XSD export section...
  case EF.Field.Datatype of
    Ftarray:
    begin
      //ftArray: maybe Firebird array type. For now, pretend it's a string.
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftadt:
    begin
      //ftAdt: No idea what this does; pretend it's a string
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftautoinc:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftbcd:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftboolean:
    begin
      if (EF.Field.AsBoolean = True) then
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode('true'));
      end
      else
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode('false'));
      end;
    end;

    Ftblob:
    begin
      //Hope the FPC library gives the right version of Base64...
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftbytes:
    begin
      //ftbytes is some kind of blob field
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftcurrency:
    begin
      //make sure there's no currency symbol in there...
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftcursor:
    begin
      //No idea what this does; pretend it's a string.
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftdataset:
    begin
      //No idea what this does; pretend it's a string
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    ftDate:
    begin
      //Apparently no conversion of local time to UTC
      if (YearOf(EF.Field.AsDateTime) < 1) then
      begin
        //Too early, just save a null.
        { TODO 4 -oAnyone -cShould have : Generate warning to user that data is out of range }
        FNode := Foutputdoc.CreateTextNode(Utf8decode(''));
      end
      else
      begin
        FNode := Foutputdoc.CreateTextNode(
          Formatdatetime('yyyy"-"mm"-"dd"T00:00:00"', EF.Field.AsDateTime));
      end;
    end;

    ftDateTime:
    begin
      //Apparently no conversion of local time to UTC
      if (YearOf(EF.Field.AsDateTime) < 1) then
      begin
        //Too early, just save a null.
        { TODO 4 -oAnyone -cShould have : Generate warning to user that data is out of range }
        FNode := Foutputdoc.CreateTextNode(Utf8decode(''));
      end
      else
      begin
        FNode := Foutputdoc.CreateTextNode(
          Formatdatetime('yyyy"-"mm"-"dd"T"hh":"nn":"ss', EF.Field.AsDateTime));
      end;
    end;

    Ftdbaseole:
    begin
      //Hope the FPC library gives the right version of Base64...
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftfixedchar:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftfixedwidechar:
    begin
      FNode :=
        Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    FtFloat:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    FtFmtbcd:
    begin
      //Assuming some kind of BCD/Binary Coded Decimal, so this might just work
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftfmtmemo:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftgraphic:
    begin
      //Hope the FPC library gives the right version of Base64...
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftguid:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8Decode(EF.Field.AsString));
    end;

    Ftidispatch:
    begin
      //No idea what this does; pretend it's a string.
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftinteger:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftinterface:
    begin
      //No idea what this does; pretend it's a string.
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftlargeint:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftmemo:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftorablob:
    begin
      //Hope the FPC library gives the right version of Base64...
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftoraclob:
    begin
      //Hope the FPC library gives the right version of Base64...
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftparadoxole:
    begin
      //Hope the FPC library gives the right version of Base64...
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftreference:
    begin
      //No idea what this does; pretend it's a string
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftsmallint:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftstring:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    ftTime:
    begin
      //Just guessing here. Maybe needs a date part, too.
      FNode := Foutputdoc.CreateTextNode(
        Formatdatetime('hh":"nn":"ss', EF.Field.AsDateTime));
    end;

    ftTimeStamp:
    begin
      //Apparently no conversion of local time to UTC
      if (YearOf(EF.Field.AsDateTime) < 1) then
      begin
        //Too early, just save a null.
        { TODO 4 -oAnyone -cShould have : Generate warning to user that data is out of range }
        FNode := Foutputdoc.CreateTextNode(Utf8decode(''));
      end
      else
      begin
        FNode := Foutputdoc.CreateTextNode(
          Formatdatetime('yyyy"-"mm"-"dd"T"hh":"nn":"ss', EF.Field.AsDateTime));
      end;
    end;

    Fttypedbinary:
    begin
      //Assume ftTypedBinary is just a binary/blob
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftvarbytes:
    begin
      //VariBytes: assume just a binary/blob...
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftvariant:
    begin
      // ftVariant: assume just a binary/blob...
      FNode := Foutputdoc.CreateTextNode(Encodestringbase64(EF.Field.AsString));
    end;

    Ftwidememo:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftwidestring:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftword:
    begin
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;

    Ftunknown:
    begin
      //raise Exception.Create('Unknown datatype for dataset field while exporting.');
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));
    end;
    else
    begin
      //raise Exception.Create('Unknown datatype for dataset field while exporting.');
      FNode := Foutputdoc.CreateTextNode(Utf8decode(EF.Field.AsString));

    end;
  end; //case
  FieldNode.AppendChild(FNode);
  FRowDataNode.AppendChild(Fieldnode);
end;

procedure TCustomXMLXSDExporter.ExportFieldDataClientDataset(const EF: Texportfielditem);

begin
  //If you change the export types here, you'll also have to change the
  //metadata export elsewhere...
  case EF.Field.Datatype of
    Ftarray:
    begin
      //ftArray: maybe Firebird array type. For now, pretend it's a memo.
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftadt:
    begin
      //ftAdt: No idea what this does; pretend it's a memo
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftautoinc:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftbcd:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftboolean:
    begin
      if (EF.Field.AsBoolean = True) then
      begin
        TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
          Utf8decode('TRUE'));
      end
      else
      begin
        TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
          Utf8decode('FALSE'));
      end;
    end;

    Ftblob:
    begin
      //Hope the FPC library gives the right version of Base64...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftbytes:
    begin
      //ftbytes is some kind of blob field
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftcurrency:
    begin
      //make sure there's no currency symbol in there...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftcursor:
    begin
      //No idea what this does; pretend it's a memo.
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftdataset:
    begin
      //No idea what this does; pretend it's a memo
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftdate:
    begin
      //Apparently no conversion of local time to UTC
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Formatdatetime('yyyymmdd', EF.Field.AsDateTime));
    end;

    Ftdatetime:
    begin
      //Apparently no conversion of local time to UTC
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Formatdatetime('yyyymmdd"T"hh:nn:ss', EF.Field.AsDateTime));
    end;

    Ftdbaseole:
    begin
      //Hope the FPC library gives the right version of Base64...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftfixedchar:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftfixedwidechar:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    FtFloat:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    FtFmtbcd:
    begin
      //Assuming some kind of BCD/Binary Coded Decimal, so this might just work
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftfmtmemo:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftgraphic:
    begin
      //Hope the FPC library gives the right version of Base64...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftguid:
    begin
      //Might even work, depending on the .AsString implementation for GUID
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8Decode(EF.Field.AsString));
    end;

    Ftidispatch:
    begin
      //No idea what this does; pretend it's a memo.
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftinteger:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftinterface:
    begin
      //No idea what this does; pretend it's a memo.
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftlargeint:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftmemo:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftorablob:
    begin
      //Hope the FPC library gives the right version of Base64...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftoraclob:
    begin
      //Hope the FPC library gives the right version of Base64...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftparadoxole:
    begin
      //Hope the FPC library gives the right version of Base64...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftreference:
    begin
      //No idea what this does; pretend it's a memo
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftsmallint:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftstring:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Fttime:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Formatdatetime('yyyymmdd"T"hh:nn:ss', EF.Field.AsDateTime));
    end;

    Fttimestamp:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Formatdatetime('yyyymmdd"T"hh:nn:ss', EF.Field.AsDateTime));
    end;

    Fttypedbinary:
    begin
      //Assume ftTypedBinary is just a binary/blob
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftvarbytes:
    begin
      //VariBytes: assume just a binary/blob...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftvariant:
    begin
      // ftVariant: assume just a binary/blob...
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Encodestringbase64(EF.Field.AsString));
    end;

    Ftwidememo:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftwidestring:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftword:
    begin
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;

    Ftunknown:
    begin
      //raise Exception.Create('Unknown datatype for dataset field while exporting.');
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;
    else
    begin
      //raise Exception.Create('Unknown datatype for dataset field while exporting.');
      TDOMElement(FRowDataNode).SetAttribute(Utf8decode(EF.ExportedName),
        Utf8decode(EF.Field.AsString));
    end;
  end; //case
end;

procedure TCustomXMLXSDExporter.ExportFieldDataExcel(const EF: Texportfielditem);
{ Limits: from
http://office.microsoft.com/en-us/excel-help/excel-specifications-and-limits-HP005199291.aspx
for Excel 2003:
Tested for in code below (date/time maximums seem to match FPC maximums, so no action needed):
- 32767 string characters
- Earliest date allowed for calculation   January 1, 1900 (January 1, 1904, if 1904 date system is used)
- Latest date allowed for calculation   December 31, 9999
- Largest amount of time that can be entered   9999:59:59
Not tested for in code below:
- Largest allowed positive number   1.79769313486231E+308
- Smallest allowed positive number   2.229E-308
- Smallest allowed negative number   -2.2251E-308
- Largest allowed negative number   -1E-307
}
var
  FieldNode: TDOMNode;
  CellNode: TDOMNode;

  procedure ExcelString;
  {var
    FieldValue: String;}
  begin
    TDOMElement(FieldNode).SetAttribute('ss:Type', 'String');
    {
    //todo: convert crlf or lf inside text to an explicitly coded
    //&#10; Excel will pick this up as a line ending inside the text cell.
    FieldValue:=StringReplace(EF.Field.AsString, #13#10, #10, [rfReplaceAll,rfIgnoreCase]);
    FieldValue:=UTF8Decode(FieldValue);
    }
    FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
  end;

  procedure ExcelBinary;
  begin
    TDOMElement(FieldNode).SetAttribute('ss:Type', 'String');
    FNode := Foutputdoc.CreateTextNode(UTF8Decode('Binary import not supported'));
  end;

begin
  CellNode := FoutputDoc.CreateElement(Utf8decode('Cell'));
  FRowDataNode.AppendChild(CellNode);

  FieldNode := FOutputDoc.CreateElement(UTF8Decode('Data'));
  case EF.Field.Datatype of
    Ftarray:
    begin
      //ftArray: maybe Firebird array type. For now, pretend it's a string.
      ExcelString;
    end;

    Ftadt:
    begin
      //ftAdt: No idea what this does; pretend it's a string
      ExcelString;
    end;

    Ftautoinc:
    begin
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    Ftbcd:
    begin
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    Ftboolean:
    begin
      // Boolean datatype is not supported, so just use numeric 0/1 output
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      if (EF.Field.AsBoolean = True) then
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode('1'));
      end
      else
      begin
        FNode := Foutputdoc.CreateTextNode(Utf8decode('0'));
      end;
    end;

    ftBLOB:
    begin
      ExcelBinary;
    end;

    Ftbytes:
    begin
      //ftbytes is some kind of blob field
      ExcelBinary;
    end;

    Ftcurrency:
    begin
      //make sure there's no currency symbol in there...
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    Ftcursor:
    begin
      //No idea what this does; pretend it's a string.
      ExcelString;
    end;

    Ftdataset:
    begin
      //No idea what this does; pretend it's a string
      ExcelString;
    end;

    Ftdate:
    begin
      //Apparently no conversion of local time to UTC
      // Observe Excel limits, see beginning of procedure
      if (YearOf(EF.Field.AsDateTime) < 1904) then
      begin
        //Too early, so we'll give the user a text value
        TDOMElement(FieldNode).SetAttribute('ss:Type', 'String');
        FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
      end
      else
      begin
        //Valid year
        TDOMElement(FieldNode).SetAttribute('ss:Type', 'DateTime');
        FNode := Foutputdoc.CreateTextNode(
          Formatdatetime('yyyy"-"mm"-"dd"T00:00:00"', EF.Field.AsDateTime));
        TDOMElement(CellNode).SetAttribute('ss:StyleID', 's23');
      end;
    end;

    Ftdatetime:
    begin
      //Apparently no conversion of local time to UTC
      // Observe Excel limits, see beginning of procedure
      if (YearOf(EF.Field.AsDateTime) < 1904) then
      begin
        //Too early, so we'll give the user a text value
        TDOMElement(FieldNode).SetAttribute('ss:Type', 'String');
        FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
      end
      else
      begin
        //Valid year
        TDOMElement(FieldNode).SetAttribute('ss:Type', 'DateTime');
        FNode := Foutputdoc.CreateTextNode(
          Formatdatetime('yyyy"-"mm"-"dd"T"hh":"nn":"ss', EF.Field.AsDateTime));
        TDOMElement(CellNode).SetAttribute('ss:StyleID', 's21');
      end;
    end;

    ftDBaseole:
    begin
      ExcelBinary;
    end;

    Ftfixedchar:
    begin
      ExcelString;
    end;

    Ftfixedwidechar:
    begin
      ExcelString;
    end;

    FtFloat:
    begin
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    FtFmtbcd:
    begin
      //Assuming some kind of BCD/Binary Coded Decimal, so this might just work
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    Ftfmtmemo:
    begin
      ExcelString;
    end;

    Ftgraphic:
    begin
      ExcelBinary;
    end;

    Ftguid:
    begin
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'String');
      FNode := Foutputdoc.CreateTextNode(Utf8Decode(EF.Field.AsString));
    end;

    Ftidispatch:
    begin
      //No idea what this does; pretend it's a string.
      ExcelString;
    end;

    Ftinteger:
    begin
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    Ftinterface:
    begin
      //No idea what this does; pretend it's a string.
      ExcelString;
    end;

    Ftlargeint:
    begin
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    ftMemo:
    begin
      ExcelString;
    end;

    Ftorablob:
    begin
      ExcelBinary;
    end;

    Ftoraclob:
    begin
      ExcelBinary;
    end;

    Ftparadoxole:
    begin
      ExcelBinary;
    end;

    Ftreference:
    begin
      //No idea what this does; pretend it's a string
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'String');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    Ftsmallint:
    begin
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    Ftstring:
    begin
      ExcelString;
    end;

    Fttime:
    begin
      //Just guessing here. Maybe needs a date part, too.
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'DateTime');
      FNode := Foutputdoc.CreateTextNode(
        Formatdatetime('"1899-12-31T"hh":"nn":"ss', EF.Field.AsDateTime));
      //Apparently Day 0 in Excel land.
      TDOMElement(CellNode).SetAttribute('ss:StyleID', 's22');
    end;

    Fttimestamp:
    begin
      //Apparently no conversion of local time to UTC
      // Observe Excel limits, see beginning of procedure
      if (YearOf(EF.Field.AsDateTime) < 1904) then
      begin
        //Too early, so we'll give the user a text value
        TDOMElement(FieldNode).SetAttribute('ss:Type', 'String');
        FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
      end
      else
      begin
        //Valid year
        TDOMElement(FieldNode).SetAttribute('ss:Type', 'DateTime');
        FNode := Foutputdoc.CreateTextNode(
          Formatdatetime('yyyy"-"mm"-"dd"T"hh":"nn":"ss', EF.Field.AsDateTime));
        TDOMElement(CellNode).SetAttribute('ss:StyleID', 's21');
      end;
    end;

    Fttypedbinary:
    begin
      //Assume ftTypedBinary is just a binary/blob
      ExcelBinary;
    end;

    Ftvarbytes:
    begin
      //VariBytes: assume just a binary/blob...
      ExcelBinary;
    end;

    Ftvariant:
    begin
      // ftVariant: assume just a binary/blob...
      ExcelBinary;
    end;

    Ftwidememo:
    begin
      ExcelString;
    end;

    Ftwidestring:
    begin
      ExcelString;
    end;

    Ftword:
    begin
      TDOMElement(FieldNode).SetAttribute('ss:Type', 'Number');
      FNode := Foutputdoc.CreateTextNode(LeftStr(UTF8Decode(EF.Field.AsString), 32767));
    end;

    FtUnknown:
    begin
      //raise Exception.Create('Unknown datatype for dataset field while exporting.');
      ExcelString;
    end;
    else
    begin
      //raise Exception.Create('Unknown datatype for dataset field while exporting.');
      ExcelString;
    end;
  end; //case
  CellNode.AppendChild(FieldNode);
  FieldNode.AppendChild(FNode);
end; //Excel

procedure TCustomXMLXSDExporter.SetXMLFormatSettings(
  const AValue: TXMLXSDFormatSettings);
begin
  inherited FormatSettings := AValue;
end;

function TCustomXMLXSDExporter.CreateFormatSettings: TCustomExportFormatSettings;
begin
  Result := TXMLXSDFormatSettings.Create(False);
end;

procedure TCustomXMLXSDExporter.DoBeforeExecute;
begin
  inherited DoBeforeExecute;
  //Don't know if this should go somewhere else, but this works:
  //Exported table/datasetname
  if DataSet.Name = '' then
  begin
    // Maybe there's another way to get a name?
    if Self.FileName = '' then
    begin
      FDatasetExportName := DefaultDatasetName;
    end
    else
    begin
      FDatasetExportName := DefaultDatasetName;
      //todo: need to sanitize filename if we want to use it (no .s, no special characters)
      //todo: would be nice to leave out the extension.
      //FDatasetExportName := ExtractFileName(Self.FileName);
    end;
  end
  else
  begin
    FDatasetExportName := DataSet.Name;
  end;

  FOutputDoc := TXMLDocument.Create;
  case FormatSettings.ExportFormat of
    AccessCompatible:
    begin
      {Though UTF-8 is one of the defaults in XML, the original Access XP export
      explicitly specifies it.
      The current XML unit will write out the encoding in any case}
    end;
    ADONETCompatible:
    begin
      { This has a standalone=yes attribute in the first line.}
      FOutputDoc.XMLStandalone := True;
    end;
    DelphiClientDataset:
    begin
      { Encoding attribute is required for Delphi not to get confused.
      Current version of XML unit writes this out, so we're covered}
    end;
  end;

  OpenTextFile;

  // Set up settings
  FXSDUsed := FormatSettings.CreateXSD;

  //Decimal separator - only relevant for Access export:
  //probably can use DefaultFormatsettings too, have to test:
  FFormatSettingsOriginal := SysUtils.FormatSettings;
  case FormatSettings.ExportFormat of
    AccessCompatible:
    begin
      // Use locale decimal separator by default, this allows Access -
      // RUNNING WITH THE SAME decimal separator - to import the data.
      // This can be overridden by user specification, if anything
      // else than ASCII 0 is specified:
      if (FormatSettings.DecimalSeparator <> #0) then
      begin
        SysUtils.FormatSettings.DecimalSeparator := FormatSettings.DecimalSeparator;
      end;
    end;
    else
    begin
      // ADO.Net, Delphi, Excel export format seem to always expect a . which is much saner.
      // This means you can import a file exported on a computer with another locale setting ;)
      SysUtils.FormatSettings.DecimalSeparator := '.';
    end;
  end;
  //Date/time settings required for translating dates to strings
  SysUtils.FormatSettings.DateSeparator := '-';
  SysUtils.FormatSettings.ShortDateFormat:='yyyy-mm-dd';
  SysUtils.FormatSettings.LongDateFormat:='yyyy-mm-dd';
  SysUtils.FormatSettings.TimeSeparator:=':';
  SysUtils.FormatSettings.ShortTimeFormat:='h:nn:ss';
  SysUtils.FormatSettings.LongTimeFormat:='h:nn:ss';
end;

procedure TCustomXMLXSDExporter.DoAfterExecute;
var
  TargetFile: Text;
begin
  // Actually write the XML file to disk:
  TargetFile := TextFile;
  WriteXMLFile(FOutputDoc, TargetFile);
  FOutputDoc.Free;
  CloseTextFile;
  //Restore original, client locale specific setting for formatting
  SysUtils.FormatSettings := FFormatSettingsOriginal;
  inherited DoAfterExecute;
end;

procedure TCustomXMLXSDExporter.DoDataRowStart;
begin
  // Start data node
  case FormatSettings.ExportFormat of
    DelphiClientDataset: FRowDataNode := FOutputDoc.CreateElement(UTF8Decode('ROW'));
    ExcelCompatible: FRowDataNode := FOutputDoc.CreateElement(UTF8Decode('Row'));
    else
      FRowDataNode := FOutputDoc.CreateElement(UTF8Decode(FDatasetExportName));
  end;
end;

procedure TCustomXMLXSDExporter.DoDataHeader;
begin
  // Start header
  case FormatSettings.ExportFormat of
    AccessCompatible: DoDataHeaderAccess;
    ADONETCompatible: DoDataHeaderADO;
    DelphiClientDataset: DoDataHeaderClientDataset;
    ExcelCompatible: DoDataHeaderExcel;
    else
      raise Exception.Create('Unknown export format in DoDataHeader.');
  end;
end;

procedure TCustomXMLXSDExporter.DoDataHeaderAccess;
// Exports metadata, such as column definitions.
var
  XSDTableListNode: TDOMNode;
  //Contains xs:element node with list of tables to be exported
  XSDSchemaNode: TDOMNode; //Top node for XSD
  XSDTableAnnotationNode: TDOMNode;
  //Contains xs:annotation for a table, which contains appinfo node with primary key info etc.
  XSDTableAppinfoNode: TDOMNode; //Contains xs:appinfo node for a table
  XSDTableDataColumnParent: TDOMNode;
  //xs:sequenc node; contains all columns for a table.
  XSDTableNode: TDOMNode;
  //Contains xs:Element for table, inside: primary key info etc, table data types
  XSDTableDataTypesNode: TDOMNode;
  //Contains xs:ComplexType element listing data types for table

  ItemCounter: integer;
  Index: TIndexDef;
  ColumnNode: TDOMNode; //xs:element node, contains column metadata; parent for column
  IndexDefs: TIndexDefs;

  procedure ExportIndexesAccess;
  var
    DescendingIndex: string;
    FieldCounter: integer;
    IndexCounter: integer;
    IndexFieldsList: TStringList;
    IndexDirection: string;
  begin
    if FXSDUsed then
    begin
      // Apparently, in e.g. MySQL fcl-db, IndexDefs only has one index, DEFAULT_ORDER;
      // use ServerIndexDefs instead
      if IsPublishedProp(DataSet, 'ServerIndexDefs') then
      begin
        IndexDefs := GetObjectProp(DataSet, 'ServerIndexDefs') as TIndexDefs;
        //Ensure IndexDefs are up-to-date:
        IndexDefs.Update;
        //for each indexdef in
        for IndexCounter := 0 to IndexDefs.Count - 1 do
        begin
          Index := IndexDefs[IndexCounter];
          // Only add index if it is based on at least one field
          if (Index.Fields <> '') then
          begin
            FNode := FOutputDoc.CreateElement('od:index');
            XSDTableAppinfoNode.AppendChild(FNode);
            if Index.Name = '' then
            begin
              TDOMElement(FNode).SetAttribute('index-name',
                UTF8Decode('idx' + StringReplace(Index.Fields, ';',
                '_', [rfReplaceAll, rfIgnoreCase]) + IntToStr(Index.ID)));
              //Avoids risk for name collision by adding collection id.
            end
            else
            begin
              TDOMElement(FNode).SetAttribute('index-name', UTF8Decode(Index.Name));
            end;

            { TODO 6 -oAnyone -cShould have : Really should get a list of exported field names (which are IIRC user-selectable) because they can differ from dataset field names...
  This would involve building a mapping table + extraction/lookup functionality...
  First see if the index functionality works at all. }
            // Semicolon-delimited list of fields=> space separated list of fields
            TDOMElement(FNode).SetAttribute('index-key',
              StringReplace(Index.Fields, ';', ' ',
              [rfReplaceAll, rfIgnoreCase]));

            // Access XP XML format does not have support for descending indexes, but Access 2010 does, so we'll put it in and hope XP will ignore it:
            IndexDirection := '';
            DescendingIndex := ';' + Index.DescFields + ';';
            //So we can search on it properly
            IndexFieldsList := TStringList.Create;
            try
              IndexFieldsList.Delimiter := ';';
              IndexFieldsList.StrictDelimiter := True;
              IndexFieldsList.DelimitedText := Index.Fields;
              for FieldCounter := 0 to IndexFieldsList.Count - 1 do
              begin
                if AnsiPos(';' + IndexFieldsList[FieldCounter] + ';',
                  DescendingIndex) > 0 then
                begin
                  //Descending
                  IndexDirection := IndexDirection + 'desc '; //note trailing space
                end
                else
                begin
                  // Not a descending index, so it probably is ascending ;)
                  IndexDirection := IndexDirection + 'asc '; //note trailing space
                end;
              end;
            finally
              IndexFieldsList.Free;
            end;
            IndexDirection := Trim(IndexDirection); //Get rid of trailing space
            TDOMElement(FNode).SetAttribute('order', IndexDirection);

            if ixPrimary in Index.Options then
            begin
              TDOMElement(FNode).SetAttribute('primary', 'yes');
            end
            else
            begin
              TDOMElement(FNode).SetAttribute('primary', 'no');
            end;

            if ixUnique in Index.Options then
            begin
              TDOMElement(FNode).SetAttribute('unique', 'yes');
            end
            else
            begin
              TDOMElement(FNode).SetAttribute('unique', 'no');
            end;

            { TODO 9 -oAnyone -cNice to have : Find out what clustered means in the XML output, and implement that}
            TDOMElement(FNode).SetAttribute('clustered', 'no');
          end; //Index usable for export
        end; //end index iteration
      end; //RTTI Published indexes
    end; //XSD used
  end;

  procedure FieldMetadataAccess;
  var
    SimpleTypeNode: TDOMNode; //xs:simpletype node, child of ColumnNode
    RestrictionNode: TDOMNode; //xs:restriction node; contains datatype
  begin
    //Data types for each field:
    ColumnNode := FOutputDoc.CreateElement('xs:element');
    TDOMElement(ColumnNode).SetAttribute(
      'name', UTF8Decode(ExportFields.Fields[ItemCounter].ExportedName));
    XSDTableDataTypesNode.AppendChild(ColumnNode);

    SimpleTypeNode := FOutputDoc.CreateElement('xs:simpleType');
    ColumnNode.AppendChild(SimpleTypeNode);

    RestrictionNode := FOutputDoc.CreateElement('xs:restriction');
    SimpleTypeNode.AppendChild(RestrictionNode);

    //Todo: we might have to tweak field conversion a bit.
    case ExportFields.Fields[ItemCounter].Field.DataType of
      ftArray: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftADT: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftAutoInc:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'autonumber');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'int');
        TDOMElement(ColumnNode).SetAttribute('od:autoUnique', 'yes');
        TDOMElement(ColumnNode).SetAttribute('od:nonNullable', 'yes');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:integer');
      end;

      ftBCD: //use same as float
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'double');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'float');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:double');
      end;

      ftBoolean:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'yesno');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'bit');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('od:nonNullable', 'yes');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:byte');
      end;

      ftBlob:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftBytes:
        //Seems to be some type of blob field
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftCurrency:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'currency');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'money');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:double');
      end;

      ftCursor: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftDataSet: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftDate:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'datetime');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'datetime');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:timeInstant');
      end;

      ftDateTime:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'datetime');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'datetime');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:timeInstant');
      end;

      ftDBaseOle: //blob type of thing
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftFixedChar: //pretend it's a string - maybe fixed width?
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftFixedWideChar: //pretend it's a string - maybe fixed width?
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftFloat:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'double');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'float');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:double');
      end;

      ftFMTBcd: //pretend it's a float.
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'double');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'float');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:double');
      end;

      ftFmtMemo: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftGraphic:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftGuid: //basically a 38 character fixed width string
        //e.g.{9F5FBC24-EFE2-4f90-B498-EC0FB7D47D15}
      begin
        {
        //for now, we'll export it as a simple text field. In future,
        //perhaps export as Access primary key/guid field or long integer
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'replicationid');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'uniqueidentifier');
        }
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'text');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'nvarchar');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '38');
        RestrictionNode.AppendChild(FNode);
      end;

      ftIDispatch: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftInteger: //might be jet integer/longinteger, and sql smallint/int
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'longinteger');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        //TDOMElement(RestrictionNode).SetAttribute('base', 'xs:integer'); //only for jet integer/sql smallint
      end;

      ftInterface: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftLargeint:
        //might be jet integer/longinteger, and sql smallint/int
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'longinteger');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        //TDOMElement(RestrictionNode).SetAttribute('base', 'xs:integer'); //only for jet integer/sql smallint
      end;

      ftMemo: //variable length string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftOraBlob:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftOraClob: //blob type of thing
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftParadoxOle: //blob type of thing
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftReference: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftSmallint:
        //might be jet integer/longinteger, and sql smallint/int
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'longinteger');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        //TDOMElement(RestrictionNode).SetAttribute('base', 'xs:integer'); //only for jet integer/sql smallint
      end;

      ftString: //fixed length or at least max length string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'text');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'nvarchar');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        { In Access, the WIDTH denotes the number of characters, not the byte count.
        Therfore we use .Size, not .DataSize}
        TDOMElement(FNode).SetAttribute('value',
          IntToStr(ExportFields.Fields[ItemCounter].Field.Size));
        RestrictionNode.AppendChild(FNode);
      end;

      ftTime:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'datetime');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'datetime');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:timeInstant');
      end;

      ftTimeStamp:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'datetime');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'datetime');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:timeInstant');
      end;

      ftTypedBinary:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftVarBytes: //suppose it's some kind of blob type of thing: todo: find out
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftVariant: //let's treat it like a binary/blob.
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'oleobject');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'image');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:binary');
        FNode := FOutputDoc.CreateElement('xs:encoding');
        TDOMElement(FNode).SetAttribute('value', 'base64');
        RestrictionNode.AppendChild(FNode);

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '1476395008');
        //Pulled this out of an Access example. Don't know how this is calculated.
        RestrictionNode.AppendChild(FNode);
      end;

      ftWideMemo:
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;

      ftWideString: //fixed length or at least max length string
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'text');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'nvarchar');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        { In Access, the WIDTH denotes the number of characters, not the byte count.
        Therefore we use .Size, not .DataSize}
        TDOMElement(FNode).SetAttribute('value',
          IntToStr(ExportFields.Fields[ItemCounter].Field.Size));
        RestrictionNode.AppendChild(FNode);
      end;

      ftWord: //might be jet integer/longinteger, and sql smallint/int
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'longinteger');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        //TDOMElement(RestrictionNode).SetAttribute('base', 'xs:integer'); //only for jet integer/sql smallint
      end;

      ftUnknown:
        //raise Exception.Create('Unknown datatype for dataset field while exporting.');
        //Treat like memo field
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //todo: Pulled this out of Access XP sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;
      else //Unknown data type, just treat it as a memo and hope for the best.
      begin
        TDOMElement(ColumnNode).SetAttribute('od:jetType', 'ntext');
        TDOMElement(ColumnNode).SetAttribute('od:sqlSType', 'memo');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');

        TDOMElement(RestrictionNode).SetAttribute('base', 'xs:string');

        FNode := FOutputDoc.CreateElement('xs:maxLength');
        TDOMElement(FNode).SetAttribute('value', '536870910');
        //Pulled this out of Access sample, don't know how this is calculated
        RestrictionNode.AppendChild(FNode);
      end;
    end; //case
  end; //Fill Function

begin
  // Root node, before any XSD or data:
  FRootNode := FOutputDoc.CreateElement('root');
  FOutputDoc.AppendChild(FRootNode); //save root node

  if FXSDUsed then
  begin
        {
        Access XP/2002 exports this line, which is not supported by Visual Studio 2002 (and probably
        ADO.NET. See:
        http://support.microsoft.com/kb/307422
        So one would use
        TDomElement(FRootNode).SetAttribute('xmlns:xs',
          'http://www.w3.org/2001/XMLSchema');
        However, Access XP chokes on importing the newer version, so leave it as is.
        }
    TDomElement(FRootNode).SetAttribute('xmlns:xs',
      UTF8Decode('http://www.w3.org/2000/10/XMLSchema'));
    TDOMElement(FRootNode).SetAttribute('xmlns:od',
      UTF8Decode('urn:schemas-microsoft-com:officedata'));
  end; //xsd used


  // Inline XSD:
  if FXSDUsed then
  begin
    // Include XSD schema:
    XSDSchemaNode := FOutputDoc.CreateElement('xs:schema');
    FRootNode.AppendChild(XSDSchemaNode); //save schema node

    // Add table list. If we're exporting an FPC dataset, there
    // will only be one table.
    XSDTableListNode := FOutputDoc.CreateElement('xs:element');
    TDOMElement(XSDTableListNode).SetAttribute('name', UTF8Decode('dataroot'));
    XSDSchemaNode.AppendChild(XSDTableListNode);

    FNode := FOutputDoc.CreateElement('xs:complexType');
    XSDTableListNode.AppendChild(FNode);

    FNode := FOutputDoc.CreateElement('xs:choice');
    TDOMElement(FNode).SetAttribute('maxOccurs', 'unbounded');
    XSDTableListNode.ChildNodes.Item[0].AppendChild(FNode);

    FNode := FOutputDoc.CreateElement('xs:element');
    TDOMElement(FNode).SetAttribute('ref', UTF8Decode(FDatasetExportName));
    XSDTableListNode.ChildNodes.Item[0].ChildNodes.Item[0].AppendChild(FNode);

    //Start table metadata/datatype list:
    XSDTableNode := FOutputDoc.CreateElement('xs:element');
    TDOMElement(XSDTableNode).SetAttribute('name', UTF8Decode(FDatasetExportName));
    XSDSchemaNode.AppendChild(XSDTableNode);

    //Add nodes for primary key/index info
    XSDTableAnnotationNode := FOutputDoc.CreateElement('xs:annotation');
    XSDTableNode.AppendChild(XSDTableAnnotationNode);

    XSDTableAppinfoNode := FOutputDoc.CreateElement('xs:appinfo');
    XSDTableAnnotationNode.AppendChild(XSDTableAppinfoNode);

    // Add node for column name and data type info
    XSDTableDataColumnParent := FOutputDoc.CreateElement('xs:sequence');
    XSDTableNode.ChildNodes.Item[0].AppendChild(XSDTableDataColumnParent);
  end; //XSD used


  // Add data root node below root node, as a sibling to xsd schema:
  FTableDataParentNode := FOutputDoc.CreateElement(UTF8Decode('dataroot'));
  if (FormatSettings.CreateXSD = True) then
  begin
    TDOMElement(FTableDataParentNode).SetAttribute(
      'xmlns:xsi', 'http://www.w3.org/2000/10/XMLSchema-instance');
  end
  else
  begin
    { No XSD format apparently has a different namespace, but it isn't used further on,
    so might not be even necessary...}
    TDOMElement(FTableDataParentNode).SetAttribute(
      'xmlns:od', UTF8Decode('urn:schemas-microsoft-com:officedata'));
  end;
  FRootNode.AppendChild(FTableDataParentNode);


  if FXSDUsed then
  begin
    FNode := FOutputDoc.CreateElement('xs:complexType');
    XSDTableNode.AppendChild(FNode);

    XSDTableDataTypesNode := FOutputDoc.CreateElement('xs:sequence');
    FNode.AppendChild(XSDTableDataTypesNode);

    for ItemCounter := 0 to ExportFields.Count - 1 do
    begin
      FieldMetadataAccess; //Only do this if XSD is specified
    end; //column iteration
  end; //xsd inline

  // Metadata/indexes
  ExportIndexesAccess;
end;

procedure TCustomXMLXSDExporter.DoDataHeaderADO;
// Exports metadata, such as column definitions.
var
  XSDTableListNode: TDOMNode;
  //Contains xs:element node with list of tables to be exported
  XSDSchemaNode: TDOMNode; //Top node for XSD
  XSDTableNode: TDOMNode;
  //Contains xs:Element for table, inside: primary key info etc, table data types
  XSDTableDataTypesNode: TDOMNode;
  //Contains xs:ComplexType element listing data types for table
  ItemCounter: integer;
  ColumnNode: TDOMNode; //xs:element node, contains column metadata; parent for column

  procedure FieldMetadataADO;
  begin
    //Data types for each field:
    ColumnNode := FOutputDoc.CreateElement('xs:element');
    TDOMElement(ColumnNode).SetAttribute(
      'name', UTF8Decode(ExportFields.Fields[ItemCounter].ExportedName));
    XSDTableDataTypesNode.AppendChild(ColumnNode);

    //Todo: we might have to tweak field conversion a bit.
    case ExportFields.Fields[ItemCounter].Field.DataType of
      ftArray: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftADT: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftAutoInc: //seems to be treated like a simple int
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftBCD: //use same as float
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:double');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftBoolean:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:boolean');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftBlob:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftBytes: //Seems to be some kind of blob field
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftCurrency: //map to decimal
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:decimal');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftCursor: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftDataSet: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftDate:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:dateTime');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftDateTime:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:dateTime');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftDBaseOle: //blob type of thing
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftFixedChar: //pretend it's a string - maybe fixed width?
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftFixedWideChar: //pretend it's a string - maybe fixed width?
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftFloat: //map to double, could possibly be mapped to float, too...
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:double');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftFMTBcd: //pretend it's a float.
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:double');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftFmtMemo: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftGraphic: //blob type of thing
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftGuid: //basically a 38 character fixed width string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
        TDOMElement(ColumnNode).SetAttribute('msdata:DataType',
          'System.Guid, mscorlib, Version=2.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089');
        //Corresponds to ADO.Net 2.0 version. Should work in later versions
      end;

      ftIDispatch: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftInteger:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftInterface: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftLargeint:
        //Maybe this datatype is bigger than an xs:int? No, should still be mapped to integer I suppose
        //Integer is a derived datatype of decimal: see http://www.w3.org/TR/xmlschema-2/
        // so probably int is too, I hope...
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftMemo: //variable length string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftOraBlob: //blob type of thing
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftOraClob: //blob type of thing
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftParadoxOle: //blob type of thing
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftReference: //pretend it's a string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftSmallint:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftString: //fixed length or at least max length string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftTime:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:dateTime');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftTimeStamp:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:dateTime');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftTypedBinary: //blob type of thing?
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftVarBytes: //suppose it's some kind of blob type of thing...
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftVariant: //let's treat it like a blob
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:base64Binary');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftWideMemo:
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftWideString: //fixed length or at least max length string
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftWord: //map to integer
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:int');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;

      ftUnknown:
        //raise Exception.Create('Unknown datatype for dataset field while exporting.');
        //Treat like memo field
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;
      else //Unknown data type, just treat it as a memo and hope for the best.
      begin
        TDOMElement(ColumnNode).SetAttribute('type', 'xs:string');
        TDOMElement(ColumnNode).SetAttribute('minOccurs', '0');
      end;
    end; //case
  end; //ADO.Net Fill Function

begin
  // Root node, before any XSD or data:
  FRootNode := FOutputDoc.CreateElement('NewDataSet');
  FOutputDoc.AppendChild(FRootNode); //save root node

  // Inline XSD:
  if FXSDUsed then
  begin
    // Include XSD schema:
    XSDSchemaNode := FOutputDoc.CreateElement('xs:schema');
    TDOMElement(XSDSchemaNode).SetAttribute('id', UTF8Decode('NewDataSet'));
    TDOMElement(XSDSchemaNode).SetAttribute('xmlns', UTF8Decode(''));
    TDOMElement(XSDSchemaNode).SetAttribute('xmlns:xs',
      UTF8Decode('http://www.w3.org/2001/XMLSchema'));
    TDOMElement(XSDSchemaNode).SetAttribute('xmlns:msdata',
      UTF8Decode('urn:schemas-microsoft-com:xml-msdata'));
    FRootNode.AppendChild(XSDSchemaNode); //save schema node

    // Add table list. If we're exporting an FPC dataset, there
    // will only be one table.
    FNode := FOutputDoc.CreateElement('xs:element');
    TDOMElement(FNode).SetAttribute('name', UTF8Decode('NewDataSet'));
    TDOMElement(FNode).SetAttribute('msdata:IsDataSet', 'true');
    TDOMElement(FNode).SetAttribute('msdata:UseCurrentLocale', 'true');
    XSDSchemaNode.AppendChild(FNode);

    XSDTableListNode := FOutputDoc.CreateElement('xs:complexType');
    FNode.AppendChild(XSDTableListNode);

    FNode := FOutputDoc.CreateElement('xs:choice');
    TDOMElement(FNode).SetAttribute('minOccurs', '0');
    TDOMElement(FNode).SetAttribute('maxOccurs', 'unbounded');
    XSDTableListNode.AppendChild(FNode);

    //Start table metadata/datatype list:
    XSDTableNode := FOutputDoc.CreateElement('xs:element');
    TDOMElement(XSDTableNode).SetAttribute('name', UTF8Decode(FDatasetExportName));
    XSDTableListNode.ChildNodes.Item[0].AppendChild(XSDTableNode);
  end; //XSD used

  { In this mode, the table data starts directly, no dataroot present, so set this to the proper node so other nodes can be added }
  FTableDataParentNode := FRootNode;

  if FXSDUsed then
  begin
    FNode := FOutputDoc.CreateElement('xs:complexType');
    XSDTableNode.AppendChild(FNode);

    XSDTableDataTypesNode := FOutputDoc.CreateElement('xs:sequence');
    FNode.AppendChild(XSDTableDataTypesNode);

    for ItemCounter := 0 to ExportFields.Count - 1 do
    begin
      FieldMetadataADO; //Only do this if XSD is specified
    end; //column iteration
  end; //xsd inline

end;

procedure TCustomXMLXSDExporter.DoDataHeaderClientDataset;
// Exports metadata, such as column definitions.
var
  ItemCounter: integer;
  ColumnNode: TDOMNode; //xs:element node, contains column metadata; parent for column
  DelphiFieldsNode: TDOMNode; // For DelphiClientDataset: <FIELDS> tag
  DelphiMetadataNode: TDOMNode; // For DelphiClientDataSet: <METADATA> tag

  procedure DelphiMemo;
  begin
    TDOMElement(ColumnNode).SetAttribute('fieldtype', 'bin.hex');
    TDOMElement(ColumnNode).SetAttribute('SUBTYPE', 'Text');
  end;

  procedure DelphiBinary;
  begin
    TDOMElement(ColumnNode).SetAttribute('fieldtype', 'bin.hex');
    TDOMElement(ColumnNode).SetAttribute('SUBTYPE', 'Binary');
  end;

  procedure FieldMetadataClientDataset;
  begin
    //Data types for each field:
    ColumnNode := FOutputDoc.CreateElement('FIELD');
    TDOMElement(ColumnNode).SetAttribute(
      'attrname', UTF8Decode(ExportFields.Fields[ItemCounter].ExportedName));
    { TODO 8 -oAnyone -cNice to have : Implement support for required fields (required="true" attribute) }
    DelphiFieldsNode.AppendChild(ColumnNode);


    { TODO 7 -oAnyone -cNice to have : Todo: we might have to tweak field conversion; have mapped a lot of types to memo and binary. Please update }
    case ExportFields.Fields[ItemCounter].Field.DataType of
      ftArray: //pretend it's a memo
      begin
        DelphiMemo;
      end;

      ftADT: //pretend it's a memo
      begin
        DelphiMemo;
      end;

      ftAutoInc: //int and required, and todo:
        //support a line like this
        //<PARAMS AUTOINCVALUE="3" CHANGE_LOG="1 0 4 2 0 4"/>
        //as a sibling of the <FIELDS> list
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'i4');
        TDOMElement(ColumnNode).SetAttribute('required', 'true');
        TDOMElement(ColumnNode).SetAttribute('SUBTYPE', 'Autoinc');
      end;

      ftBCD: //WIDTH=precision, DECIMALS=scale
        //negative DECIMALS not allowed! (However, Oracle does allow negative scale)
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'fixed');
        TDOMElement(ColumnNode).SetAttribute('WIDTH', '20');
        TDOMElement(ColumnNode).SetAttribute('DECIMALS',
          IntToStr(ExportFields.Fields[ItemCounter].Field.size));
        {From bug report: Reason for the fixed width in above code: poor support in SQL-DB.
        20 should fit double type values since scale is limited to 4. So no exponents.
        Haven't tested negative scales as allowed in Oracle yet.
        I know Delphi 2006 crashes on negative values but I can't tell yet if negative scales ripple through to TFieldDef.size.}
      end;

      ftBoolean:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'boolean');
      end;

      ftBlob:
      begin
        DelphiBinary;
      end;

      ftBytes: //Seems to be some kind of blob field
      begin
        DelphiBinary;
      end;

      ftCurrency:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'r8');
        TDOMElement(ColumnNode).SetAttribute('SUBTYPE', 'Money');
      end;

      ftCursor: //pretend it's a memo
      begin
        DelphiMemo;
      end;

      ftDataSet: //pretend it's a memo
      begin
        DelphiMemo;
      end;

      ftDate:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'date');
      end;

      ftDateTime:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'dateTime');
      end;

      ftDBaseOle: //blob type of thing
      begin
        DelphiBinary;
      end;

      ftFixedChar: //pretend it's a memo - maybe fixed width?
      begin
        DelphiMemo;
      end;

      ftFixedWideChar: //pretend it's a memo - maybe fixed width?
      begin
        DelphiMemo;
      end;

      ftFloat:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'r8');
      end;

      ftFMTBcd: //pretend it's a float.
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'r8');
      end;

      ftFmtMemo: //pretend it's a memo
      begin
        DelphiMemo;
      end;

      ftGraphic: //blob type of thing
      begin
        DelphiBinary;
      end;

      ftGuid: //basically a 38 character fixed width string
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'string');
        TDOMElement(ColumnNode).SetAttribute('WIDTH', '38');
      end;

      ftIDispatch: //pretend it's a memo
      begin
        DelphiMemo;
      end;

      ftInteger:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'i4');
      end;

      ftInterface: //pretend it's a memo
      begin
        DelphiMemo;
      end;

      ftLargeint:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'i8');
      end;

      ftMemo: //variable length string
      begin
        DelphiMemo;
      end;

      ftOraBlob: //blob type of thing
      begin
        DelphiBinary;
      end;

      ftOraClob: //blob type of thing
      begin
        DelphiBinary;
      end;

      ftParadoxOle: //blob type of thing
      begin
        DelphiBinary;
      end;

      ftReference: //pretend it's a memo
      begin
        DelphiMemo;
      end;

      ftSmallint:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'i2');
      end;

      ftString: //fixed length or at least max length string
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'string');
        { We're setting the width to the number of bytes required - DataSize, not the number of characters - Size}
        TDOMElement(ColumnNode).SetAttribute('WIDTH',
          UTF8Decode(IntToStr(ExportFields.Fields[ItemCounter].Field.DataSize)));
      end;

      ftTime:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'time');
      end;

      ftTimeStamp:
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'dateTime');
      end;

      ftTypedBinary: //blob type of thing?
      begin
        DelphiBinary;
      end;

      ftVarBytes: //suppose it's some kind of blob type of thing...
      begin
        DelphiBinary;
      end;

      ftVariant: //let's treat it like a blob.
      begin
        DelphiBinary;
      end;

      ftWideMemo:
      begin
        DelphiMemo;
      end;

      ftWideString: //fixed length or at least max length string
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'string');
        { We're setting the width to the number of bytes required - DataSize, not the number of characters - Size}
        TDOMElement(ColumnNode).SetAttribute('WIDTH',
          UTF8Decode(IntToStr(ExportFields.Fields[ItemCounter].Field.DataSize)));
      end;

      ftWord: //map to integer
      begin
        TDOMElement(ColumnNode).SetAttribute('fieldtype', 'i8');
      end;

      ftUnknown:
        //raise Exception.Create('Unknown datatype for dataset field while exporting.');
        //Treat like memo field
      begin
        DelphiMemo;
      end;
      else //Unknown data type, just treat it as a memo and hope for the best.
      begin
        DelphiMemo;
      end;
    end; //case
  end; //Clientdataset Fill Function

begin
  // Root node, before any XSD or data:
  FRootNode := FOutputDoc.CreateElement('DATAPACKET');
  TDOMElement(FRootNode).SetAttribute('Version', '2.0');
  //Todo: don't know if another version is necessary for other Delphi versions

  FOutputDoc.AppendChild(FRootNode); //save root node

  { First some metadata stuff }
  DelphiMetadataNode := FOutputDoc.CreateElement('METADATA');
  FRootNode.AppendChild(DelphiMetadataNode);

  { Create a ROWDATA node under the root node, add all ROW nodes under that}
  FTableDataParentNode := FOutputDoc.CreateElement(UTF8Decode('ROWDATA'));
  FRootNode.AppendChild(FTableDataParentNode);

  //Metadata/indexes
  DelphiFieldsNode := FOutputDoc.CreateElement('FIELDS');
  DelphiMetadataNode.AppendChild(DelphiFieldsNode);
  for ItemCounter := 0 to ExportFields.Count - 1 do
  begin
    FieldMetadataClientDataset; //Always do this
  end;
end;

procedure TCustomXMLXSDExporter.DoDataHeaderExcel;
// Exports metadata, such as column definitions.
// In Excel export, we don't generated a XSD metadata.
var
  ExcelStyles: TDOMNode; //  <Styles> node for Excel export
  StyleElement: TDOMNode; //   <Style ss:ID="bla"> node for Excel export

begin
  // Root node, before any XSD or data:
  FRootNode := FOutputDoc.CreateElement('Workbook');
  TDOMElement(FRootNode).SetAttribute(
    'xmlns', 'urn:schemas-microsoft-com:office:spreadsheet');
  TDOMElement(FRootNode).SetAttribute(
    'xmlns:ss', 'urn:schemas-microsoft-com:office:spreadsheet');
  // Add some style info for date/time fields that will be referered to by
  // the data cells

  ExcelStyles := FOutputDoc.CreateElement('Styles');
  FRootNode.AppendChild(ExcelStyles);

  StyleElement := FOutputDoc.CreateElement('Style');
  TDOMElement(StyleElement).SetAttribute('ss:ID', 's21');

  FNode := FOutputDoc.CreateElement('NumberFormat');
  TDOMElement(FNode).SetAttribute('ss:Format', 'General Date');
  StyleElement.AppendChild(FNode);
  ExcelStyles.AppendChild(StyleElement);

  StyleElement := FOutputDoc.CreateElement('Style');
  TDOMElement(StyleElement).SetAttribute('ss:ID', 's22');
  FNode := FOutputDoc.CreateElement('NumberFormat');
  TDOMElement(FNode).SetAttribute('ss:Format', 'Short Time');
  StyleElement.AppendChild(FNode);
  ExcelStyles.AppendChild(StyleElement);

  StyleElement := FOutputDoc.CreateElement('Style');
  TDOMElement(StyleElement).SetAttribute('ss:ID', 's23');
  FNode := FOutputDoc.CreateElement('NumberFormat');
  TDOMElement(FNode).SetAttribute('ss:Format', 'Short Date');
  StyleElement.AppendChild(FNode);
  ExcelStyles.AppendChild(StyleElement);
  FOutputDoc.AppendChild(FRootNode); //save root node

  // Add data root node below root node
  FNode := FOutputDoc.CreateElement('Worksheet');
  TDOMElement(FNode).SetAttribute(
    'ss:Name', UTF8Decode(FDatasetExportName));
  FRootNode.AppendChild(FNode);

  FTableDataParentNode := FOutputDoc.CreateElement('Table');
  FNode.AppendChild(FTableDataParentNode);
end;


procedure TCustomXMLXSDExporter.DoDataFooter;

begin
  inherited DoDataFooter;
end;

procedure TCustomXMLXSDExporter.DoDataRowEnd;

begin
  //Close off FDataRowNode by adding it to TableData node
  FTableDataParentNode.AppendChild(FRowDataNode);
end;

procedure TCustomXMLXSDExporter.ExportField(EF: Texportfielditem);
begin
  case FormatSettings.ExportFormat of
    AccessCompatible: ExportFieldDataAccess(EF);
    ADONETCompatible: ExportFieldDataADO(EF);
    DelphiClientDataset: ExportFieldDataClientDataset(EF);
    ExcelCompatible: ExportFieldDataExcel(EF);
    else
      raise Exception.Create('Unknown export format in function ExportField.');
  end;
end;

{ TXMLXSDFormatSettings }

procedure TXMLXSDFormatSettings.Assign(Source: TPersistent);

var
  XS: TXMLXSDFormatSettings;

begin
  // Default settings:
  { TODO 4 -oAnyone -cShould have : Is this the proper way of assigning default values to settings? }
  FExportFormat := ExcelCompatible;
  { It's a toss-up what the default data format should be.
  I suppose people are more likely to have Excel installed & are more likely to be interested in Excel import compared to the other options.
  }

  CreateXSD := True;
  DecimalSeparator := #0; //Don't override decimal separator by default

  if Source is TXMLXSDFormatSettings then
  begin
    Xs := TXMLXSDFormatSettings(Source);
    FExportFormat := XS.FExportFormat;
    CreateXSD := XS.CreateXSD;
  end;
  inherited Assign(Source);
end;

procedure RegisterXMLXSDExportFormat;

begin
  ExportFormats.RegisterExportFormat(SXMLXSD, SXMLXSDDescription,
    SXMLXSDExtensions, TXMLXSDExporter);
end;

procedure UnRegisterXMLXSDExportFormat;

begin
  ExportFormats.UnregisterExportFormat(SXMLXSD);
end;

end.

