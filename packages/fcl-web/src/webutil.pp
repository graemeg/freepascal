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
unit webutil;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, httpdefs;

procedure DumpRequest (ARequest : TRequest; Dump : TStrings);

implementation



procedure DumpRequest (ARequest : TRequest; Dump : TStrings);

  Procedure AddNV(Const N,V : String);
  
  begin
    Dump.Add('<TR><TD>'+N+'</TD><TD>'+V+'</TD></TR>');
  end;

Var
  I   : integer;
  N,V : String;
begin
  With ARequest, Dump do
    begin
    // All possible headers
    Add('<H1>All possible request headers:</H1>');
    Add('<TABLE BORDER="1"><TR><TD>Header</TD><TD>Value</TD></TR>');
    For I:=1 to NoHTTPFields do
      begin
      AddNV(HTTPFieldNames[i],GetFieldByName(HTTPFieldNames[i]));
      end;
    Add('</TABLE><P>');

    // Actually sent headers
    Add('<H1>Actually sent request headers:</H1>');
    Add('<TABLE BORDER="1"><TR><TD>Header</TD><TD>Value</TD></TR>');
    For I:=0 to FieldCount-1 do
      AddNV(FieldNames[I],FieldValues[I]);
    Add('</TABLE><P>');

    // Actually sent headers, as text
    Add('<H1>Actually sent request headers as text:</H1>');
    For I:=0 to FieldCount-1 do
      Add(Fields[I]+'<BR>');
      
    // Additional headers
    Add('<H1>Additional headers:</H1>');
    Add('<TABLE BORDER="1"><TR><TD>Header</TD><TD>Value</TD></TR>');
    AddNV('PathInfo',PathInfo);
    AddNV('PathTranslated',PathTranslated);
    AddNV('RemoteAddress',RemoteAddress);
    AddNV('RemoteHost',RemoteHost);
    AddNV('ScriptName',ScriptName);
    AddNV('ServerPort',IntToStr(ServerPort));
    AddNV('Method',Method);
    AddNV('URL',URL);
    AddNV('Query',Query);
    AddNV('Host',Host);
    AddNV('Content',Content);
    Add('</TABLE><P>');
    // Additional headers
    If (QueryFields.Count>0) then
      begin
      Add('<H1>Request variables: ('+IntToStr(QueryFields.Count)+') </H1>');
      Add('<TABLE BORDER="1"><TR><TD>Name</TD><TD>Value</TD></TR>');
      For I:=0 to QueryFields.Count-1 do
        begin
        QueryFields.GetNameValue(i,N,V);
        AddNV(N,V);
        end;
      Add('</TABLE><P>');
      end;
    If (Files.Count>0) then
      begin
      Add('<H1>Uploaded files: ('+IntToStr(Files.Count)+') </H1>');
      Add('<TABLE BORDER="1">');
      Add('<TR><TD>Name</TD><TD>FileName</TD><TD>Size</TD>');
      Add('<TD>Temp FileName</TD><TD>Disposition</TD><TD>Content-Type</TD></TR>');
      For I:=0 to Files.Count-1 do
        With Files[i] do
          begin
          Add('<TR><TD>'+FieldName+'</TD><TD>'+FileName+'</TD>');
          Add('<TD>'+IntToStr(Size)+'</TD><TD>'+LocalFileName+'</TD>');
          Add('<TD>'+Disposition+'</TD><TD>'+ContentType+'</TD></TR>');
          end;
      Add('</TABLE><P>');
      end;
    end;
end;

end.

