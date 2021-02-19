{   Unicode CLDR's collation parser.
    Copyright (c) 2013-2017 by Inoussa OUEDRAOGO

    It creates units from CLDR's collation files.

    The source code is distributed under the Library GNU
    General Public License with the following modification:

        - object files and libraries linked into an application may be
          distributed without source code.

    If you didn't receive a copy of the file COPYING, contact:
          Free Software Foundation
          675 Mass Ave
          Cambridge, MA  02139
          USA

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
program cldrparser;

{$mode objfpc}{$H+}
{ $define WINCE_TEST}
{$TYPEDADDRESS ON}

uses    //heaptrc,
  SysUtils, classes, getopts,{$ifdef WINCE}StreamIO,{$endif}
  cldrhelper, helper, cldrtest, cldrxml, unicodeset, cldrtxt;

const
  SROOT_RULES_FILE = 'UCA_Rules_SHORT.txt';
  SUsageText =
    'This program creates pascal units from CLDR''s collation files for usage ' + sLineBreak +
    'with the FreePascal Native Unicode Manager.' + sLineBreak + sLineBreak +
    'Usage : cldrparser <collationFileName> [<typeName>] [-a<alt>] [-d<dataDir>] [-o<outputDir>] [-t<HaltOnFail>]' + sLineBreak + sLineBreak +
    '  where :' + sLineBreak +
    ' ' + sLineBreak +
    '   - collationFileName : specify the target file.' + sLineBreak +
    '   - typeName : optional, specify the collation'' type-name to be parse;' + sLineBreak +
    '     If this argument is not supplied, a default type-name' + sLineBreak +
    '     is chosen from the type-name list in the following order  : ' + sLineBreak +
    '          * the "default" specified type-name indicated by the collation' + sLineBreak +
    '          * the type named "standard" ' + sLineBreak +
    '          * the type named "search" ' + sLineBreak +
    '          * the first type.' + sLineBreak +
    '   - a : this provides the "alt" property to select specific "type".' + sLineBreak +
    '   - dataDir : specify the directory that contains the collation files.' + sLineBreak +
    '               The default value is the program''s directory.' + sLineBreak +
    '   - outputDir : specify the directory where the generated files will be stored.' + sLineBreak +
    '                 The default value is the program''s directory.' + sLineBreak +
    '   - t : to execute parser the test suite. The program will execute only the test suite and exit.' + sLineBreak +
    '         <HaltOnFail> may be one of (y, Y, t, T, 1) to halt the execution on the first failing.' + sLineBreak +
    ' ' + sLineBreak +
    '  The program expects some files to be present in the <dataDir> folder : ' + sLineBreak +
    '     - UCA_Rules_SHORT.txt ' + sLineBreak +
    '     - allkeys.txt this is the file allkeys_CLDR.txt renamed to allkeys.txt' + sLineBreak +
    '  These files are in the core.zip file of the CLDR release files. The CLDR''version used should be synchronized the' + sLineBreak +
    '  version of the Unicode version used, for example for Uniocde 7 it will be CLDR 26.' + sLineBreak +
    '  The CLDR files are provided by the Unicode Consortium at http://cldr.unicode.org/index/downloads';


function ParseOptions(
  var ADataDir,
      AOuputDir,
      ACollationFileName,
      ACollationTypeName,
      ACollationTypeAlt  : string;
  var AExecTestSuite,
      ATestHaltOnFail    : Boolean
) : Boolean;
var
  c : Char;
  idx, k : Integer;
  s : string;
begin
{$ifdef WINCE_TEST}
  ADataDir := ExtractFilePath(ParamStr(0))+'data';
  AOuputDir := ADataDir;
  ACollationFileName := 'sv.xml';
  exit(True);
{$endif WINCE_TEST}
  if (ParamCount() = 0) then
    exit(False);
  Result := True;
  AExecTestSuite := False;
  repeat
    c := GetOpt('a:d:o:ht:');
    case c of
      'a' : ACollationTypeAlt := Trim(OptArg);
      'd' : ADataDir := ExpandFileName(Trim(OptArg));
      'o' : AOuputDir := ExpandFileName(Trim(OptArg));
      'h', '?' :
        begin
          WriteLn(SUsageText);
          Result := False;
        end;
      't' :
        begin
          AExecTestSuite := True;
          s := Trim(OptArg);
          ATestHaltOnFail := (s <> '') and CharInSet(s[1],['y','Y','t','T','1']);
        end;
    end;
  until (c = EndOfOptions);
  idx := 0;
  for k := 1 to ParamCount() do begin
    s := Trim(ParamStr(k));
    if (s <> '') and (s[1] <> '-') then begin
      if (idx = 0) then
        ACollationFileName := s
      else if (idx = 1) then
        ACollationTypeName := s;
      Inc(idx);
      if (idx >= 2) then
        Break;
    end;
  end;
end;

procedure Main;
var
  orderedChars : TOrderedCharacters;
  settings : TSettingRecArray;
  ucaBook : TUCA_DataBook;
  stream, streamNE, streamOE, binaryStreamNE, binaryStreamOE : TMemoryStream;
  s, collationFileName, collationTypeName, collationTypeAlt : string;
  i , c: Integer;
  repository : TCldrCollationRepository;
  collation : TCldrCollation;
  dataPath, outputPath : string;
  collationItem : TCldrCollationItem;
  testSuiteFlag, testSuiteHaltOnFailFlag : Boolean;
{$ifdef WINCE}
  fs : TFileStream;
{$endif WINCE}
begin
{$ifdef WINCE}
  s := ExtractFilePath(ParamStr(0))+'cldr-log.txt';
  DeleteFile(s);
  fs := TFileStream.Create(s,fmCreate);
  AssignStream(Output,fs);
  Rewrite(Output);
  s := ExtractFilePath(ParamStr(0))+'cldr-err.txt';
  DeleteFile(s);
  fs := TFileStream.Create(s,fmCreate);
  AssignStream(ErrOutput,fs);
  Rewrite(ErrOutput);
{$endif WINCE}
{$ifdef WINCE_TEST}
  testSuiteFlag := True;
  try
    exec_tests();
  except
    on e : Exception do begin
      WriteLn('Exception : '+e.Message);
      raise;
    end;
  end;
  exit;
{$endif WINCE_TEST}
  dataPath := '';
  outputPath := '';
  collationFileName := '';
  collationTypeName := '';
  collationTypeAlt := '';
  testSuiteFlag := False;
  testSuiteHaltOnFailFlag := True;
  if not ParseOptions(
           dataPath,outputPath,collationFileName,collationTypeName,
           collationTypeAlt,testSuiteFlag,testSuiteHaltOnFailFlag
         )
  then begin
    WriteLn(SUsageText);
    Halt(1);
  end;
  if testSuiteFlag then begin
    WriteLn('Executing the test suite ...');
    exec_tests(testSuiteHaltOnFailFlag);
    Halt;
  end;
  if (dataPath <> '') and not(DirectoryExists(dataPath)) then begin
    WriteLn('This directory does not exist : ',dataPath);
    Halt(1);
  end;
  if (dataPath = '') then
    dataPath := ExtractFilePath(ParamStr(0))
  else
    dataPath := IncludeTrailingPathDelimiter(dataPath);
  if (outputPath = '') then
    outputPath := dataPath
  else
    outputPath := IncludeTrailingPathDelimiter(outputPath);
{$ifndef WINCE_TEST}
  if (ParamCount() = 0) then begin
    WriteLn(SUsageText);
    Halt(1);
  end;
{$endif WINCE_TEST}
  if not(
       FileExists(dataPath+SROOT_RULES_FILE) and
       FileExists(dataPath+'allkeys.txt')
     )
  then begin
    WriteLn(Format('File not found : %s or %s.',[dataPath+SROOT_RULES_FILE,dataPath+'allkeys.txt']));
    Halt(1);
  end;

  {collationFileName := dataPath + collationFileName;
  if not FileExists(collationFileName) then begin
    WriteLn('File not found : "',collationFileName,'"');
    Halt(1);
  end;}

  WriteLn(sLineBreak,'Collation Parsing ',QuotedStr(collationFileName),'  ...');
  stream := nil;
  streamNE := nil;
  streamOE := nil;
  binaryStreamNE := nil;
  binaryStreamOE := nil;
  repository := TCldrCollationRepository.Create(
                  TCldrCollationFileLoader.Create(dataPath) as ICldrCollationLoader
                );
  try
    collation := repository.Load(collationFileName,TCldrParserMode.HeaderParsing);
    WriteLn(Format('  Collation Count = %d',[collation.FindPublicItemCount()]));
    if (collation.FindPublicItemCount() = 0) then begin
      WriteLn('No collation in this file.');
    end else begin
      for i := 0 to collation.ItemCount - 1 do begin
        if not collation.Items[i].IsPrivate() then begin
          s := collation.Items[i].TypeName;
          if (collation.Items[i].Alt <> '') then
            s := s + ', Alt = ' + collation.Items[i].Alt;
          WriteLn(Format('  Item[%d] = (Type = %s)',[i,s]));
        end;
      end;
      if (collationTypeAlt = '') then
        collationItem := collation.Find(collationTypeName)
      else
        collationItem := collation.Find(collationTypeName,collationTypeAlt);
      if (collationItem = nil) then begin
        collationTypeName := FindCollationDefaultItemName(collation);
        collationItem := collation.Find(collationTypeName);
        collationTypeAlt := collationItem.Alt;
      end;
      s := collationTypeName;
      if (collationTypeAlt <> '') then
        s := Format('%s (%s)',[s,collationTypeAlt]);
      WriteLn(Format('Parsing Collation Item "%s" ...',[s]));
      collationItem := repository.LoadType(collationFileName,collationTypeName,collationTypeAlt);

      s := dataPath + SROOT_RULES_FILE;
      WriteLn;
      WriteLn('Parsing ',QuotedStr(s),'  ...');
      FillByte(orderedChars,SizeOf(orderedChars),0);
      orderedChars.Clear();
      SetLength(settings,0);
      ParseInitialDocument(@orderedChars,s,settings);
      WriteLn('File parsed, ',orderedChars.ActualLength,' characters.');

      WriteLn('Loading CLDR root''s key table ...');
      stream := TMemoryStream.Create();
      s := dataPath + 'allkeys.txt';
      stream.LoadFromFile(s);
      FillChar(ucaBook,SizeOf(ucaBook),#0);
      ParseUCAFile(stream,ucaBook);
      //WriteLn('  LEVEL-2''s items Value = ',CalcMaxLevel2Value(ucaBook.Lines));
      //RewriteLevel2Values(@ucaBook.Lines[0],Length(ucaBook.Lines));
      //WriteLn('  LEVEL-2''s items Value (after rewrite) = ',CalcMaxLevel2Value(ucaBook.Lines));
      c := FillInitialPositions(@orderedChars.Data[0],orderedChars.ActualLength,ucaBook.Lines);
      if (c > 0) then
        WriteLn('  Missed Initial Positions = ',c);
      WriteLn('   Loaded.');

      WriteLn('Start generation ...');
      stream.Clear();
      streamNE := TMemoryStream.Create();
      streamOE := TMemoryStream.Create();
      binaryStreamNE := TMemoryStream.Create();
      binaryStreamOE := TMemoryStream.Create();
      s := COLLATION_FILE_PREFIX + ChangeFileExt(LowerCase(ExtractFileName(collationFileName)),'.pas');
      GenerateCdlrCollation(
        collation,collationTypeName,s,stream,streamNE,streamOE,
        binaryStreamNE,binaryStreamOE,
        orderedChars,ucaBook.Lines
      );
      stream.SaveToFile(outputPath+s);
      if (streamNE.Size > 0) then begin
        streamNE.SaveToFile(outputPath+GenerateEndianIncludeFileName(s,ENDIAN_NATIVE));
        streamOE.SaveToFile(outputPath+GenerateEndianIncludeFileName(s,ENDIAN_NON_NATIVE));
      end;
      if (binaryStreamNE.Size > 0) then begin
        binaryStreamNE.SaveToFile(
          outputPath +
          ChangeFileExt(s,Format('_%s.bco',[ENDIAN_SUFFIX[ENDIAN_NATIVE]]))
        );
        binaryStreamOE.SaveToFile(
          outputPath +
          ChangeFileExt(s,Format('_%s.bco',[ENDIAN_SUFFIX[ENDIAN_NON_NATIVE]]))
        );
      end;
    end;
    repository.Clear();
  finally
    binaryStreamOE.Free();
    binaryStreamNE.Free();
    streamOE.Free();
    streamNE.Free();
    stream.Free();
    repository.Free();
  end;

  WriteLn(sLineBreak,'Finished.');
end;

begin
  Main();
end.

