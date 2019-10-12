{
    This file is part of the Free Component Library
    Copyright (c) 2019 by the Free Pascal development team

    Demo for SQL source syntax parser

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program parsesql;

uses sysutils, classes, fpsqlparser, fpsqlscanner,fpsqltree;

Procedure parseScript(const aFilename:String; AScript  :TStringList);

var
  i: integer;
  Parser: TSQLParser;
  ResultList: TSQLElementList;
  ScriptStream:TFileStream;
begin
  ScriptStream:=TFileStream.Create(aFilename, fmopenreadwrite or fmshareexclusive);
  try
    ScriptStream.Position:=0;
    Parser := TSQLParser.Create(ScriptStream);
    try
      ResultList := Parser.ParseScript([poAllowSetTerm]);
      for i:=0 to ResultList.Count-1 do
        AScript.Add(ResultList[i].GetAsSQL([sfoDoubleQuoteIdentifier]));
    finally
      Parser.Free;
    end;
  finally
    ScriptStream.Free;
    ResultList.Free;
  end;
end;

Var
  L : TStringList;
  S : String;

begin
  if ParamCount<>1 then
    begin
    Writeln('Parse & Dump SQL');
    Writeln('Usage : parsesql <filename>');
    Halt(1);
    end;
  L:=TStringList.Create;
  try
    ParseScript(ParamStr(1),L);
    for S in L do Writeln(S);
  Finally
    L.Free;
  end;
end.

