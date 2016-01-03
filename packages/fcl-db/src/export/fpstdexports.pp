{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by Michael Van Canneyt, member of the
    Free Pascal development team

    Standard export formats registration.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
  This unit has a routine and a component to register standard distributed
  export formats in an application. The Component version is meant for
  use in Lazarus: Drop it on a form, set the formats you want to see
  registered, and set active to true. When the form is created a run-time,
  the selected formats will be registered.
  
  The simple call takes an optional single argument, a set which tells
  the call which formats to register. If none is specified, all formats
  are registered.

}
unit fpstdexports;


{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpDBExport;
  
Type
<<<<<<< HEAD
<<<<<<< HEAD
  TStdExportformat = (sefCSV,sefFixedLength,sefSimpleXMl,sefXMLXSD,sefSimpleJSON,sefSQL,seTeX,seRTF,sefDBF);
  TStdExportformats = Set of TStdExportFormat;

Const
  AllStdExportFormats = [sefCSV,sefFixedLength,sefSimpleXMl,sefXMLXSD,sefSimpleJSON,sefSQL,seTeX,seRTF,sefDBF];
=======
=======
>>>>>>> origin/fixes_2_2
  TStdExportformat = (sefCSV,sefFixedLength,sefSimpleXMl,sefSimpleJSON,sefSQL,seTeX,seRTF,sefDBF);
  TStdExportformats = Set of TStdExportFormat;

Const
  AllStdExportFormats = [sefCSV,sefFixedLength,sefSimpleXMl,sefSimpleJSON,sefSQL,seTeX,seRTF,sefDBF];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Type

  { TStandardExportFormats }

  TStandardExportFormats = Class(TComponent)
  private
    FActive: Boolean;
    FFormats: TStdExportFormats;
    FRegistered : TStdExportFormats;
    procedure SetActive(const AValue: Boolean);
  Protected
    Procedure Loaded; override;
    Procedure DoRegister;
    Procedure DoUnregister;
  Public
    Constructor Create(AOwner : TComponent); override;
  Published
    Property Active : Boolean Read FActive Write SetActive;
    Property Formats : TStdExportFormats Read FFormats Write FFormats Default AllStdExportFormats;
  end;
  
Function RegisterStdFormats(Fmts : TStdExportFormats) : TStdExportFormats; overload;
Function RegisterStdFormats : TStdExportFormats; overload;
Function UnRegisterStdFormats(Fmts : TStdExportFormats) : TStdExportFormats;

implementation

uses
  fpcsvexport,
  fpfixedexport,
  fpsimplexmlexport,
<<<<<<< HEAD
<<<<<<< HEAD
  fpxmlxsdexport,
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  fpsimplejsonexport,
  fpsqlexport,
  fptexexport,
  fprtfexport,
  fpdbfexport;

Const
  StdExportNames : Array[TStdExportFormat] of string
<<<<<<< HEAD
<<<<<<< HEAD
                 = (SCSVExport,SFixedLengthExport,SSimpleXML, SXMLXSD,
                    SSimpleJSON,SSQLExport,STexExport,SRTFExport,SDBFExport);
  StdExportRegProcs : Array[TStdExportFormat] of Procedure
                 = (@RegisterCSVExportFormat,@RegisterFixedExportFormat,@RegisterSimpleXMLExportFormat,@RegisterXMLXSDExportFormat,
                    @RegisterSimpleJSONExportFormat,@RegisterSQLExportFormat,@RegisterTexExportFormat
                    ,@RegisterRTFExporter,@RegisterDBFExportFormat);
  StdExportUnRegProcs : Array[TStdExportFormat] of Procedure
                 = (@UnRegisterCSVExportFormat,@UNRegisterFixedExportFormat,@UnRegisterSimpleXMLExportFormat,@UnRegisterXMLXSDExportFormat,
=======
=======
>>>>>>> origin/fixes_2_2
                 = (SCSVExport,SFixedLengthExport,SSimpleXML,
                    SSimpleJSON,SSQLExport,STexExport,SRTFExport,SDBFExport);
  StdExportRegProcs : Array[TStdExportFormat] of Procedure
                 = (@RegisterCSVExportFormat,@RegisterFixedExportFormat,@RegisterSimpleXMLExportFormat,
                    @RegisterSimpleJSONExportFormat,@RegisterSQLExportFormat,@RegisterTexExportFormat
                    ,@RegisterRTFExporter,@RegisterDBFExportFormat);
  StdExportUnRegProcs : Array[TStdExportFormat] of Procedure
                 = (@UnRegisterCSVExportFormat,@UNRegisterFixedExportFormat,@UnRegisterSimpleXMLExportFormat,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                    @UnRegisterSimpleJSONExportFormat,@UnRegisterSQLExportFormat,@UnRegisterTexExportFormat,
                    @UnRegisterRTFExporter,@UnRegisterDBFExportFormat);

Function RegisterStdFormats : TStdExportFormats;

begin
  Result:=RegisterStdFormats(AllStdExportFormats);
end;


function RegisterStdFormats(Fmts: TStdExportFormats): TStdExportFormats;

Var
  F : TStdExportFormat;

begin
  Result:=[];
  For F:=Low(TStdExportFormat) to High(TStdExportFormat) do
    If (F in Fmts) and (ExportFormats.IndexOfFormat(StdExportNames[f])=-1) then
      begin
      StdExportRegProcs[f];
      Include(Result,F);
      end;
end;

function UnRegisterStdFormats(Fmts: TStdExportFormats): TStdExportFormats;

Var
  F : TStdExportFormat;

begin
  Result:=[];
  For F:=Low(TStdExportFormat) to High(TStdExportFormat) do
    If (F in Fmts) and (ExportFormats.IndexOfFormat(StdExportNames[f])<>-1) then
      begin
      StdExportUnRegProcs[f];
      Include(Result,F);
      end;

end;

{ TStandardExportFormats }

procedure TStandardExportFormats.SetActive(const AValue: Boolean);
begin
  if FActive=AValue then
    exit;
  FActive:=AValue;
  If Not (csLoading in ComponentState) then
    If Active then
      DoRegister
    else
      DoUnregister;
end;

procedure TStandardExportFormats.Loaded;
begin
  If FActive then
    DoRegister;
end;

procedure TStandardExportFormats.DoRegister;
begin
  FRegistered:=RegisterSTdFormats(FFormats);
end;

procedure TStandardExportFormats.DoUnRegister;
begin
  FRegistered:=RegisterSTdFormats(FRegistered);
end;

constructor TStandardExportFormats.Create(AOwner: TComponent);
begin
  Inherited;
  FFormats:=AllStdExportFormats;
end;

end.

