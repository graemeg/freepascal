{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T : TTarget;
  P : TPackage;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}
    P:=AddPackage('fcl-sdo');
    P.ShortName:='fcsd';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('fcl-db');
    P.Version:='3.3.1';
    P.Author := 'Inoussa Ouedraogo';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := 'inoussa12@gmail.com';
    P.Description := 'Free Pascal implementation of Service Data Objects';
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    // P.NeedLibC:= false;
    P.SourcePath.Add('src/base');
    P.SourcePath.Add('src/das');
    P.IncludePath.Add('src/das');

    T:=P.Targets.AddUnit('sdo_consts.pas');
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('sdo_fpc_xml.pas');
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('sdo_logger_intf.pas');
    T:=P.Targets.AddUnit('sdo_types.pas');
    T:=P.Targets.AddUnit('xsd_consts.pas');
    T:=P.Targets.AddUnit('sdo_binary_streamer.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      end;
    T:=P.Targets.AddUnit('sdo_linked_list.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      end;
    T:=P.Targets.AddUnit('sdo_date_utils.pas');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      end;
    T:=P.Targets.AddUnit('sdo.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo_linked_list');
      AddUnit('sdo_date_utils');
      end;
    T:=P.Targets.AddUnit('sdo_utils.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      end;
    T:=P.Targets.AddUnit('sdo_changesummary.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      AddUnit('sdo_utils');
      end;
    T:=P.Targets.AddUnit('sdo_cursor_intf.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      end;
    T:=P.Targets.AddUnit('sdo_imp_utils.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      end;
    T:=P.Targets.AddUnit('sdo_type.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      AddUnit('sdo_date_utils');
      AddUnit('sdo_imp_utils');
      end;
    T:=P.Targets.AddUnit('sdo_xpath_helper.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      end;
    T:=P.Targets.AddUnit('sdo_field_imp.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      AddUnit('sdo_type');
      AddUnit('sdo_date_utils');
      AddUnit('sdo_imp_utils');
      end;
    T:=P.Targets.AddUnit('sdo_dataobject.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo_linked_list');
      AddUnit('sdo');
      AddUnit('sdo_changesummary');
      AddUnit('sdo_type');
      AddUnit('sdo_xpath_helper');
      AddUnit('sdo_field_imp');
      AddUnit('sdo_utils');
      AddUnit('sdo_imp_utils');
      end;
    T:=P.Targets.AddUnit('sdo_datafactory.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      AddUnit('sdo_type');
      AddUnit('sdo_consts');
      AddUnit('sdo_imp_utils');
      AddUnit('sdo_dataobject');
      end;
    T:=P.Targets.AddUnit('sdo_dom_cursors.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_fpc_xml');
      AddUnit('sdo_types');
      AddUnit('sdo_cursor_intf');
      end;
    T:=P.Targets.AddUnit('sdo_rtti_filters.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo_cursor_intf');
      end;
    T:=P.Targets.AddUnit('sdo_parserutils.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('xsd_consts');
      AddUnit('sdo_cursor_intf');
      AddUnit('sdo_dom_cursors');
      AddUnit('sdo_rtti_filters');
      end;
    T:=P.Targets.AddUnit('sdo_xsdintf.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo');
      end;
    T:=P.Targets.AddUnit('sdo_xsdparser.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_fpc_xml');
      AddUnit('sdo_logger_intf');
      AddUnit('sdo_types');
      AddUnit('sdo');
      AddUnit('sdo_cursor_intf');
      AddUnit('sdo_rtti_filters');
      AddUnit('sdo_consts');
      AddUnit('xsd_consts');
      AddUnit('sdo_utils');
      AddUnit('sdo_dom_cursors');
      AddUnit('sdo_parserutils');
      AddUnit('sdo_xsdintf');
      end;
    T:=P.Targets.AddUnit('sdo_locators.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_xsdparser');
      end;
    T:=P.Targets.AddUnit('sdo_serialization_utils.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      end;
    T:=P.Targets.AddUnit('sdo_serialization_binary.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_consts');
      AddUnit('sdo_types');
      AddUnit('sdo_binary_streamer');
      AddUnit('sdo');
      AddUnit('sdo_serialization_utils');
      end;
    T:=P.Targets.AddUnit('sdo_serialization_xml.pas');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('sdo_consts');
      AddUnit('sdo_fpc_xml');
      AddUnit('sdo_types');
      AddUnit('sdo');
      AddUnit('sdo_serialization_utils');
      AddUnit('sdo_imp_utils');
      end;
    T:=P.Targets.AddUnit('xsd_generator.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_fpc_xml');
      AddUnit('sdo');
      AddUnit('sdo_types');
      AddUnit('xsd_consts');
      AddUnit('sdo_parserutils');
      AddUnit('sdo_xsdintf');
      end;
    T:=P.Targets.AddUnit('pas_generator.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo');
      AddUnit('sdo_types');
      AddUnit('xsd_consts');
      AddUnit('sdo_parserutils');
      AddUnit('sdo_xsdintf');
      end;
    T:=P.Targets.AddUnit('sdo_xsd_helper.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo');
      AddUnit('sdo_type');
      AddUnit('sdo_datafactory');
      AddUnit('sdo_consts');
      AddUnit('sdo_fpc_xml');
      AddUnit('sdo_types');
      AddUnit('xsd_consts');
      AddUnit('sdo_imp_utils');
      AddUnit('sdo_parserutils');
      AddUnit('sdo_xsdintf');
      AddUnit('sdo_xsdparser');
      AddUnit('xsd_generator');
      end;
    T:=P.Targets.AddUnit('sdo_serialization.pas');
    with T.Dependencies do
      begin
      AddUnit('sdo_types');
      AddUnit('sdo');
      AddUnit('sdo_serialization_utils');
      AddUnit('sdo_serialization_xml');
      AddUnit('sdo_consts');
      AddUnit('sdo_utils');
      AddUnit('sdo_changesummary');
      AddUnit('sdo_imp_utils');
      AddUnit('sdo_xpath_helper');
      AddUnit('sdo_dataobject');
      AddUnit('sdo_xsd_helper');
      end;
    T:=P.Targets.AddUnit('data_acces_intf.pas');
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('sdo_das.pas');
    with T.Dependencies do
      begin
      AddUnit('data_acces_intf');
      AddInclude('sdo_global.inc');
      end;
    T:=P.Targets.AddUnit('sdo_das_utils.pas');
    with T.Dependencies do
      begin
      AddUnit('data_acces_intf');
      AddUnit('sdo_das');
      end;
    T:=P.Targets.AddUnit('sdo_das_imp.pas');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('data_acces_intf');
      AddUnit('sdo_das');
      AddUnit('sdo_das_utils');
      end;
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif}
