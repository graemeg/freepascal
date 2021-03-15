{
    This file is part of the Free Pascal test suite.
    Copyright (c) 2013 by Karoly Balogh <karoly.balogh@viprinet.com>
    Copyright (c) 2013 by Viprinet Europe GmbH

    This program can convert Free Pascal Testsute results to JUnit
    results to be used (for example) with Jenkins CI suite.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$MODE DELPHI}
{$I+}
program fpts2junit;

uses
  classes, sysutils, strutils,
  DOM, XMLWrite;

const
  MAX_XML_CHARS = 1000;
  LOG_SHORT = 'log';
  LOG_LONG  = 'longlog';

  DEFAULT_JUNIT_XML = 'fpc_testsuite.xml';

  PATTERN_SUCCESS = 'Success';
  PATTERN_FAILED  = 'Fail';
  PATTERN_SKIPPED = 'Skip';

  IE_FUBAR = 'internalerror generated';

function getIndexInList(l: TStringList; className: String; caseName: String): LongInt;
var
  line: String;
  filename: String;
begin
  result:=-1;
  filename:=className+'/'+caseName;
  for line in l do
    if (Pos(filename, line) > 0) then
      begin
        result:=l.IndexOf(line);
        break;
      end;
end;

procedure convertLogFiles(testOutputPath: String; junitXMLName: String);
var
  logShort: TStringList;
  logLong: TStringList;

  junitXML: TXMLDocument;

  rootNode: TDOMNode;
  caseNode: TDOMNode;
  tmpNode: TDOMNode;

  failed: LongInt;
  error: LongInt;
  skipped: LongInt;
  success: LongInt;
  tmpLine: String;
  Line: string;

  startIdx: LongInt;
  tmpString: String;
  className: String;
  caseName: String;
  i: Integer;
begin
  logShort:=TStringList.Create;
  logLong:=TStringList.Create;

  // sanity check arguments
  testOutputPath:=IncludeTrailingPathDelimiter(ExpandFileName(testOutputPath));
  if not DirectoryExists(testOutputPath) then
    begin
      writeln('Path: ',testOutputPath,' is not valid.');
      halt(1);
    end;

  if not AnsiEndsText('.xml', junitXMLName) then
    junitXMLName:=DEFAULT_JUNIT_XML;

  // read *ALL* the logs! (ok, some of them)
  writeln('Reading logs from directory: ',testOutputPath);
  logShort.LoadFromFile(testOutputPath+LOG_SHORT);
  logLong.LoadFromFile(testOutputPath+LOG_LONG);

  junitXML:=TXMLDocument.Create;

  // convert
  failed:=0;
  error:=0;
  skipped:=0;
  success:=0;

  rootNode:=junitXML.CreateElement('testsuite');
  junitXML.AppendChild(rootNode);

  for Line in logShort do
    begin
      tmpline := Line;
      // this is pretty fubar in the logfile, to break the format
      // lets fix it up...
      if AnsiEndsText(IE_FUBAR, tmpLine) then
        begin
          tmpLine:=AnsiReplaceText(tmpLine, IE_FUBAR, '');
        end;

      // extract useful stuff
      tmpString:=ExtractWord(WordCount(tmpLine,[' '])-2,tmpLine,[' ']);
      className:=AnsiLeftStr(tmpString,RPos(DirectorySeparator,tmpString)-1);
      caseName:=ExtractWord(WordCount(tmpString,[DirectorySeparator]),tmpString,[DirectorySeparator]);

      // create testcase node
      caseNode:=junitXML.CreateElement('testcase');
      if pos('../', classname) = 1 then
        Delete(classname, 1, 3);
      TDOMElement(caseNode).SetAttribute('classname',WideString(className));
      TDOMElement(caseNode).SetAttribute('name',WideString(caseName));
      rootNode.AppendChild(caseNode);

      if AnsiStartsText(PATTERN_FAILED, tmpLine) then
        begin
          tmpString:=TrimSet(AnsiLeftStr(tmpLine, AnsiPos(className, tmpLine)-1),[' ']);

          // handle compiler errors as errors, otherwise failures
          if AnsiPos('compile',tmpString) <> 0 then
            begin
              Inc(error);
              tmpNode:=junitXML.CreateElement('error');
            end
          else
            begin
              Inc(failed);
              tmpNode:=junitXML.CreateElement('failure');
            end;

          TDOMElement(tmpNode).SetAttribute('message',WideString(tmpString));
          startIdx:=getIndexInList(logLong, className, caseName);
          tmpString:='';
          while startIdx > 0 do
            begin
              tmpString:=tmpString + #10 + logLong[startIdx];
              Inc(startIdx);
              if (startIdx >= logLong.Count) or
                 AnsiStartsText('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>', logLong[startIdx]) then break;
            end;
          if tmpString <> '' then
          begin
            if Length(tmpString) > MAX_XML_CHARS then
              tmpString := '--- ' + IntToStr(Length(tmpstring) - MAX_XML_CHARS) + 'bytes cut away --- '+ #10 + Copy(tmpString, Length(tmpstring) - MAX_XML_CHARS, MAX_XML_CHARS);
            for i := 1 to Length(tmpString) do
              if tmpString[i] in [#0..#9,#11..#31] then
                tmpString[i] := ' ';
            tmpNode.AppendChild(junitXML.CreateTextNode(WideString(tmpString+#10)));
          end;
          caseNode.AppendChild(tmpNode);
          continue;
        end;
      if AnsiStartsText(PATTERN_SKIPPED, tmpLine) then
        begin
          Inc(skipped);
          caseNode.AppendChild(junitXML.CreateElement('skipped'));
          continue;
        end;
      if AnsiStartsText(PATTERN_SUCCESS, tmpLine) then
        begin
          Inc(success);
          continue;
        end;
      writeln('Unparseable line: [',tmpLine,']');
      Halt(1);
    end;

  // set required elements in the root node
  TDOMElement(rootNode).SetAttribute('errors',WideString(IntToStr(error)));
  TDOMElement(rootNode).SetAttribute('failures',WideString(IntToStr(failed)));
  TDOMElement(rootNode).SetAttribute('tests',WideString(IntToStr(logShort.Count)));
  TDOMElement(rootNode).SetAttribute('name','Compiler.Testsuite');
  TDOMElement(rootNode).SetAttribute('package','FPC');

  writeln('Writing results to file: ',junitXMLName);
  writeXMLFile(junitXML, junitXMLName);
end;

procedure printusage;
begin
  writeln('Usage:');
  writeln('  ',ExtractFileName(ParamStr(0)),' <path_to_test_output_dir> [output.xml]');
  writeln('    * if no output filename is specified, "',DEFAULT_JUNIT_XML,'" will be used.');
  writeln('    * if specified, output filename must end with ".xml", otherwise default ');
  writeln('      name will be used.');
  halt(1);
end;

begin
  if (ParamCount < 1) or (ParamCount > 2) then
    printusage;

  convertLogFiles(ParamStr(1),ParamStr(2));
end.
