{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Sebastian Guenther
    Added .rc and OS/2 MSG support in 2002 by Yuri Prokushev

    .rst resource string table file converter.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$MODE objfpc}
{$H+}

program rstconv;

uses
{$ifdef unix}
  cwstring,
{$endif}
  sysutils, classes, jsonparser, fpjson, charset, cpall;

resourcestring
  help =
    'rstconv [-h|--help]    Displays this help'+LineEnding+
    'rstconv options        Convert .rsj/.rst file'+LineEnding+LineEnding+
    'Options are:'+LineEnding+
    '  -i file        Use specified file instead of stdin as input .rsj/.rst (OPTIONAL)'+LineEnding+
    '  -o file        Write output to specified file (REQUIRED)'+LineEnding+
    '  -f format      Specifies the output format:'+LineEnding+
    '                 po    GNU gettext .po (portable) format (DEFAULT)'+LineEnding+
    '                 msg   IBM OS/2 MSG file format'+LineEnding+
    '                 rc    Resource compiler .rc format'+LineEnding+LineEnding+
    '.po format only options are:'+LineEnding+
    '  -c char set    Adds a header specifying the given character set (OPTIONAL).'+LineEnding+LineEnding+
    'OS/2 MSG file only options are:'+LineEnding+
    '  -c identifier  Specifies the component identifier (REQUIRED).'+LineEnding+
    '                 Identifier is any three chars in upper case.'+LineEnding+
    '  -n number      Specifies the first message number [1-9999] (OPTIONAL).'+LineEnding+LineEnding+
    'Resource compiler script only options are:'+LineEnding+
    '  -s             Use STRINGTABLE instead of MESSAGETABLE'+LineEnding+
    '  -c identifier  Use identifier as ID base (ID+n) (OPTIONAL)'+LineEnding+
    '  -n number      Specifies the first ID number (OPTIONAL)'+LineEnding+
    '.rsj-input format-only options are:'+LineEnding+
    '  -p codepage    Convert the string data to the specified code page before'+LineEnding+
    '                 writing it to the output file. Possible values:';


  InvalidOption = 'Invalid option - ';
  RequiredOption = 'Required option is absent - ';
  OptionAlreadySpecified = 'Option has already been specified - ';
  NoOutFilename = 'No output filename specified';
  InvalidOutputFormat = 'Invalid output format -';
  MessageNumberTooBig = 'Message number too big';
  InvalidRange = 'Invalid range of the first message number';
  MissingOption = 'Missing option after parameter ';
  UnsupportedOutputCodePage = 'Unsupported output code page specified: ';
  RstNoOutputCodePage = 'It is not possible to specify an output code page when using a .rst file';

type

  TConstItem = class(TCollectionItem)
  public
    ModuleName, ConstName, Value: String;
  end;

var
  InFilename, OutFilename: String;
  ConstItems: TCollection;
<<<<<<< HEAD
<<<<<<< HEAD
  HeaderCharSet: String;
=======
  CharSet: String;
>>>>>>> graemeg/fixes_2_2
=======
  CharSet: String;
>>>>>>> origin/fixes_2_2
  Identifier: String;
  OutputCodePage: Longint;
  FirstMessage: Word;
  MessageTable: Boolean;

procedure ReadRSTFile;
var
  f: Text;
  s: String;
  item: TConstItem;
  DotPos, EqPos, i, j: Integer;
begin
  Assign(f, InFilename);
  Reset(f);

  while not eof(f) do begin
    ReadLn(f, s);
    If (Length(S)=0) or (S[1]='#') then
      continue;
    item := TConstItem(ConstItems.Add);

    DotPos := Pos('.', s);
    EqPos := Pos('=', s);
    if DotPos > EqPos then // paranoia checking.
      DotPos := 0;
    item.ModuleName := Copy(s, 1, DotPos - 1);
    item.ConstName := Copy(s, DotPos + 1, EqPos - DotPos - 1);

    item.Value := '';
    i := EqPos + 1;
    while i <= Length(s) do begin
      if s[i] = '''' then begin
        Inc(i);
        j := i;
        while (i <= Length(s)) and (s[i] <> '''') do
          Inc(i);
        item.Value := item.Value + Copy(s, j, i - j);
        Inc(i);
      end else if s[i] = '#' then begin
        Inc(i);
        j := i;
        while (i <= Length(s)) and (s[i] in ['0'..'9']) do
          Inc(i);
        item.Value := item.Value + Chr(StrToInt(Copy(s, j, i - j)));
      end else if s[i] = '+' then begin
        ReadLn(f, s);
        i := 1;
      end else
        Inc(i);
    end;
  end;

  Close(f);
end;

procedure ReadRSJFile;
var
  Stream: TFileStream;
  Parser: TJSONParser;
  JsonItems,
  RawStringData: TJSONArray;
  JsonData, JsonItem: TJSONObject;
  S: String;
  item: TConstItem;
  DotPos, I, J: Integer;
begin
  if OutputCodePage<>-1 then
    DefaultSystemCodePage:=OutputCodePage;
  Stream := TFileStream.Create(InFilename, fmOpenRead or fmShareDenyNone);
  Parser := TJSONParser.Create(Stream);
  try
    JsonData := Parser.Parse as TJSONObject;
    try
      JsonItems := JsonData.Arrays['strings'];
      for I := 0 to JsonItems.Count - 1 do
      begin
        item := TConstItem(ConstItems.Add);
        JsonItem := JsonItems.Items[I] as TJSONObject;
        S := JsonItem.Get('name');
        DotPos := Pos('.', s);
        item.ModuleName := Copy(s, 1, DotPos - 1);
        item.ConstName := Copy(s, DotPos + 1, Length(S) - DotPos);
        if OutputCodePage=-1 then
          begin
            RawStringData:=JsonItem.Get('sourcebytes',TJSONArray(nil));
            SetLength(item.Value, RawStringData.Count);
            for J := 1 to Length(item.Value) do
              item.Value[J]:=char(RawStringData.Integers[J-1]);
          end
        else
          { automatically converts from UTF-16 to the correct code page due
            to the change of DefaultSystemCodePage to OutputCodePage above }
          item.Value := JsonItem.Get('value');
      end;
    finally
      JsonData.Free;
    end;
  finally
    Parser.Free;
    Stream.Free;
  end;
end;

procedure ConvertToGettextPO;
var
  i, j: Integer;
  f: Text;
  item: TConstItem;
  s: String;
  c: Char;

begin
  Assign(f, OutFilename);
  Rewrite(f);
  
<<<<<<< HEAD
<<<<<<< HEAD
  if HeaderCharSet<>'' then begin
=======
  if CharSet<>'' then begin
>>>>>>> graemeg/fixes_2_2
=======
  if CharSet<>'' then begin
>>>>>>> origin/fixes_2_2
    // Write file header  with
    WriteLn(f, 'msgid ""');
    WriteLn(f, 'msgstr ""');
    WriteLn(f, '"MIME-Version: 1.0\n"');
<<<<<<< HEAD
<<<<<<< HEAD
    WriteLn(f, '"Content-Type: text/plain; charset=', HeaderCharSet, '\n"');
=======
    WriteLn(f, '"Content-Type: text/plain; charset=', CharSet, '\n"');
>>>>>>> graemeg/fixes_2_2
=======
    WriteLn(f, '"Content-Type: text/plain; charset=', CharSet, '\n"');
>>>>>>> origin/fixes_2_2
    WriteLn(f, '"Content-Transfer-Encoding: 8bit\n"');
    WriteLn(f);
  end;

  for i := 0 to ConstItems.Count - 1 do begin
    item := TConstItem(ConstItems.items[i]);

    // Convert string to C-style syntax
    s := '';
    for j := 1 to Length(item.Value) do begin
      c := item.Value[j];
      case c of
        #9:  s := s + '\t';
        #10: s := s + '\n';
{$IFNDEF UNIX}
        #13: ;
        #1..#8, #11..#12, #14..#31, #128..#255:
{$ELSE}
        #1..#8, #11..#31, #128..#255:
{$ENDIF}
          s := s + '\' +
            Chr(Ord(c) shr 6 + 48) +
            Chr((Ord(c) shr 3) and 7 + 48) +
            Chr(Ord(c) and 7 + 48);
        '\': s := s + '\\';
        '"': s := s + '\"';
      else s := s + c;
      end;
    end;

    // Write msg entry
    WriteLn(f, '#: ', item.ModuleName, ':', item.ConstName);
    j := Pos('\n', s);
    if j > 0 then begin
      WriteLn(f, 'msgid ""');
      while j > 0 do begin
        Writeln(f, '"',copy(s, 1, j+1),'"');
        Delete(s, 1, j+1);
        j := Pos('\n', s);
      end;
      if s <> '' then
        Writeln(f, '"',s,'"');
    end
    else
      WriteLn(f, 'msgid "', s, '"');
    WriteLn(f, 'msgstr ""');
    WriteLn(f);
  end;

  Close(f);
end;

// This routine stores rst file in rc format. Can be written as MESSAGETABLE
// as STRINGTABLE. Beware! OS/2 RC doesn't support lines longer whan 255 chars.
procedure ConvertToRC;
var
  i, j: Integer;
  f: Text;
  item: TConstItem;
  s: String;
  c: Char;

begin
  Assign(f, OutFilename);
  Rewrite(f);

  If MessageTable then
    WriteLn(F, 'MESSAGETABLE')
  else
    WriteLn(F, 'STRINGTABLE');

  WriteLn(F, 'BEGIN');
  If Identifier<>'' then WriteLn(F, '#define ', Identifier);

  for i := 0 to ConstItems.Count - 1 do begin
    item := TConstItem(ConstItems.items[i]);

    // Convert string to C-style syntax
    s := '';
    for j := 1 to Length(item.Value) do begin
      c := item.Value[j];
      case c of
        #9:  s := s + '\t';
        #10: s := s + '\n"'#13#10'"';
{$IFNDEF UNIX}
        #13: ;
        #1..#8, #11..#12, #14..#31, #128..#255:
{$ELSE}
        #1..#8, #11..#31, #128..#255:
{$ENDIF}
          s := s + '\' +
            Chr(Ord(c) shr 6 + 48) +
            Chr((Ord(c) shr 3) and 7 + 48) +
            Chr(Ord(c) and 7 + 48);
        '\': s := s + '\\';
        '"': s := s + '\"';
      else s := s + c;
      end;
    end;

    // Write msg entry
    WriteLn(f, '/* ', item.ModuleName, ':', item.ConstName, '*/');
    WriteLn(f, '/* ', s, ' */');
    If Identifier<>'' then Write(F, Identifier, '+');
    WriteLn(f, I+FirstMessage,' "', s, '"');
    WriteLn(f);
  end;

  WriteLn(F, 'END');
  Close(f);
end;

// This routine stores rst file in OS/2 msg format. This format is preffered
// for help screens, messages, etc.
procedure ConvertToOS2MSG;
var
  i, j: Integer;
  f: Text;
  item: TConstItem;
  s: String;
begin
  If (ConstItems.Count+FirstMessage-1)>9999 then
  begin
    WriteLn(MessageNumberTooBig);
    Halt(1);
  end;

  Identifier:=Copy(UpperCase(Identifier), 1, 3);

  Assign(f, OutFilename);
  Rewrite(f);

  WriteLn(f, Identifier);

  // Fake entry, because MKMSGF limitation
  WriteLn(f, Format('%s%.4d?: ',[Identifier, FirstMessage-1]));

  for i := 0 to ConstItems.Count - 1 do begin
    item := TConstItem(ConstItems.items[i]);

    // Prepare comment string
    // Convert string to C-style syntax
    s := '';
    j:=1;
    while j<=Length(item.Value) do
    begin
      if copy(item.Value, j, 2)=#13#10 then
      begin
        s:=s+#13#10';';
        Inc(j, 2);
      end else begin
        s := s + item.Value[j];
        Inc(j);
      end;
    end;

    // Write msg entry
    WriteLn(f, ';', item.ModuleName, '.', item.ConstName);
    WriteLn(f, Format(';%s%.4dP: %s %%0',[Identifier, i+FirstMessage, s]));
    WriteLn(f, Format('%s%.4dP: %s %%0',[Identifier, i+FirstMessage, Item.Value]));
  end;

  Close(f);
end;

type
  TConversionProc = procedure;
var
  i: Integer;
  ConversionProc: TConversionProc;
  OutputFormat: String;
begin

  if (ParamStr(1) = '-h') or (ParamStr(1) = '--help') then begin
    WriteLn(help);
    for i:=low(word) to high(word) do
      if mappingavailable(i) then
        writeln('                   ',getmap(i)^.cpname);
    { UTF-8 is not supported via the CharSet unit }
    writeln('                   UTF-8');
    exit;
  end;

  ConversionProc := @ConvertToGettextPO;
  OutputFormat:='';
<<<<<<< HEAD
<<<<<<< HEAD
  HeaderCharSet:='';
=======
  CharSet:='';
>>>>>>> graemeg/fixes_2_2
=======
  CharSet:='';
>>>>>>> origin/fixes_2_2
  Identifier:='';
  FirstMessage:=0;
  MessageTable:=True;
  OutputCodePage:=-1;

  i := 1;
  while i <= ParamCount do begin
    if ParamStr(i) = '-i' then begin
      if InFilename <> '' then begin
        WriteLn(StdErr, OptionAlreadySpecified, '-i');
        Halt(1);
      end;
      InFilename := ParamStr(i + 1);
      Inc(i, 2);
    end else if ParamStr(i) = '-o' then begin
      if OutFilename <> '' then begin
        WriteLn(StdErr, OptionAlreadySpecified, '-o');
        Halt(1);
      end;
      OutFilename := ParamStr(i + 1);
      Inc(i, 2);
    end else if ParamStr(i) = '-f' then begin
      if OutputFormat <> '' then begin
        WriteLn(StdErr, OptionAlreadySpecified, '-f');
        Halt(1);
      end;
      if ParamStr(i + 1) = 'po' then
        OutputFormat:='po'
      else if ParamStr(i + 1) = 'msg' then begin
        OutputFormat:='msg';
        ConversionProc := @ConvertToOS2MSG;
      end else if ParamStr(i + 1) = 'rc' then begin
        OutputFormat:='rc';
        ConversionProc := @ConvertToRC;
      end else begin
        WriteLn(StdErr, InvalidOutputFormat, ParamStr(i + 1));
        Halt(1);
      end;
      Inc(i, 2);
    end else if ParamStr(i) = '-c' then begin
      if (OutputFormat='') or (OutputFormat='po') then begin
<<<<<<< HEAD
<<<<<<< HEAD
        if HeaderCharSet <> '' then begin
          WriteLn(StdErr, OptionAlreadySpecified, '-c');
          Halt(1);
        end;
        HeaderCharSet:=ParamStr(i+1);
=======
=======
>>>>>>> origin/fixes_2_2
        if CharSet <> '' then begin
          WriteLn(StdErr, OptionAlreadySpecified, '-c');
          Halt(1);
        end;
        CharSet:=ParamStr(i+1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end else
      begin
        if Identifier <> '' then begin
          WriteLn(StdErr, OptionAlreadySpecified, '-c');
          Halt(1);
        end;
        Identifier:=ParamStr(i+1);
      end;
      Inc(i, 2);
    end else if ParamStr(i) = '-s' then begin
      if not MessageTable then begin
        WriteLn(StdErr, OptionAlreadySpecified, '-s');
        Halt(1);
      end;
      MessageTable:=False;
      Inc(i);
    end else if ParamStr(i) = '-n' then begin
      if FirstMessage <> 0 then begin
        WriteLn(StdErr, OptionAlreadySpecified, '-n');
        Halt(1);
      end;
      try
        FirstMessage := StrToInt(ParamStr(i + 1));
        If (FirstMessage<1) then raise EConvertError.Create(InvalidRange+' '+ParamStr(i+1));
      except
        on EConvertError do
        begin
          WriteLn(StdErr, InvalidOption, ParamStr(i));
          Halt(1);
        end;
      end;
      Inc(i, 2);
    end else if ParamStr(i) = '-p' then
      begin
        if paramcount=i then
          begin
            WriteLn(StdErr, MissingOption,'-p');
            Halt(1)
          end;
        if UpperCase(paramstr(i+1))<>'UTF-8' then
          if not mappingavailable(ParamStr(i+1)) then
            begin
              WriteLn(StdErr, UnsupportedOutputCodePage, ParamStr(i+1));
              Halt(1);
            end
          else
            OutputCodePage:=getmap(ParamStr(i+1))^.cp
        else
          OutputCodePage:=CP_UTF8;
        Inc(i, 2);
      end
    else begin
      WriteLn(StdErr, InvalidOption, ParamStr(i));
      Halt(1);
    end;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  If ((OutputFormat<>'') and (OutputFormat<>'po')) and (HeaderCharSet<>'')  then begin
=======
  If ((OutputFormat<>'') and (OutputFormat<>'po')) and (CharSet<>'')  then begin
>>>>>>> graemeg/fixes_2_2
=======
  If ((OutputFormat<>'') and (OutputFormat<>'po')) and (CharSet<>'')  then begin
>>>>>>> origin/fixes_2_2
    WriteLn(StdErr, InvalidOption, '');
    Halt(1);
  end;

  If ((OutputFormat<>'msg') and (OutputFormat<>'rc')) and ((Identifier<>'') or (FirstMessage<>0)) then begin
    WriteLn(StdErr, InvalidOption, '');
    Halt(1);
  end;

  If (OutputFormat='msg') and (Identifier='') then begin
    WriteLn(StdErr, RequiredOption, '-c');
    Halt(1);
  end;

  if OutFilename = '' then begin
    WriteLn(StdErr, NoOutFilename);
    Halt(1);
  end;

  ConstItems := TCollection.Create(TConstItem);

  if ExtractFileExt(InFilename) = '.rsj' then
    ReadRSJFile
  else
    begin
      if OutputCodePage<>-1 then
        begin
          WriteLn(StdErr, RstNoOutputCodePage);
          Halt(1);
        end;
      ReadRSTFile;
    end;

  ConversionProc;
end.
