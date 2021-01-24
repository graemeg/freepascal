{ %TARGET=win32,win64,wince,linux,solaris,openbsd }

{
  This unit tests mostly TRegIniFile to work properly and be Delphi compatible.
  This test also runs on non-Windows platforms where XML registry is used.
  Please keep this test Delphi compatible.
}

{$ifdef FPC} {$mode delphi}  {$endif}
uses
{$ifdef unix}
  cwstring,
{$endif unix}
  SysUtils, Classes, registry;

{$ifdef FPC}
  {$WARN implicit_string_cast_loss off}
  {$WARN symbol_deprecated off}
{$endif FPC}

const
  STestRegPath = 'Software\FPC-RegTest';
  
procedure TestFailed(ErrCode: integer);
begin
  writeln('Test FAILED. Error code: ' + IntToStr(ErrCode));
  Halt(ErrCode);
end;

procedure ClearReg(const KeyName: string = '');
begin
  with TRegistry.Create do
    try
      DeleteKey(STestRegPath);
    finally
      Free;
    end;
end;

function NormPath(const s: string): string;
begin
  Result:=StringReplace(s, '/', '\', [rfReplaceAll]);
end;

procedure DoRegTest2;
var
  reg: TRegistry;
  ri: TRegIniFile;
  rini: TRegistryIniFile;
  sl: TStringList;
begin
  ClearReg;
  try
    reg:=TRegistry.Create;
    try
      { The test key must be deleted by ClearReg() }
      if reg.KeyExists(STestRegPath) then
        TestFailed(1);
      if reg.OpenKey(STestRegPath, False) then
        TestFailed(2);

      if not reg.OpenKey(STestRegPath, True) then
        TestFailed(5);
      if NormPath(reg.CurrentPath) <> STestRegPath then
        TestFailed(6);
      reg.WriteString('Item1', '1');
      if not reg.OpenKey('\' + STestRegPath + '\1', True) then
        TestFailed(10);
      reg.WriteString('Item2', '2');
      if NormPath(reg.CurrentPath) <> STestRegPath + '\1' then
        TestFailed(15);
      reg.CloseKey;
      if NormPath(reg.CurrentPath) <> '' then
        TestFailed(20);
      if reg.KeyExists(STestRegPath + '\' + STestRegPath) then
        TestFailed(21);
    finally
      reg.Free;
    end;

    ri:=TRegIniFile.Create(STestRegPath);
    with ri do
    try
      if ReadString('', 'Item1', '') <> '1' then
        TestFailed(101);
      if ReadString('1', 'Item2', '') <> '2' then
        TestFailed(105);
      if NormPath(ri.CurrentPath) <> STestRegPath then
        TestFailed(110);
      if ReadString('', 'Item1', '') <> '1' then
        TestFailed(115);
      if not ValueExists('Item1') then
        TestFailed(120);

      WriteInteger('1', 'Item3', 3);

      sl:=TStringList.Create;
      try
        ReadSectionValues('1', sl);
        if sl.Count <> 2 then
          TestFailed(125);
        if sl.Values['Item2'] <> '2' then
          TestFailed(130);
        if sl.Values['Item3'] <> '3' then
          TestFailed(135);
      finally
        sl.Free;
      end;

      WriteInteger('', 'Item4', 4);
      WriteInteger('', 'Item41', 41);
      WriteInteger('', 'Item42', 42);
      if GetDataType('Item4') <> rdString then
        TestFailed(140);
      if ReadString('', 'Item41', '') <> '41' then
        TestFailed(141);
      if ReadString('', 'Item42', '') <> '42' then
        TestFailed(142);
    finally
      Free;
    end;

    { \ at the beginning of the path must be accepted }
    ri:=TRegIniFile.Create('\' + STestRegPath);
    with ri do
    try
      if ReadString('', 'Item1', '') <> '1' then
        TestFailed(145);
    finally
      Free;
    end;

    { Write to non-existing key must work }
    ri:=TRegIniFile.Create(STestRegPath + '\2\3\4');
    with ri do
    try
      if FileName <> NormPath(CurrentPath) then
        TestFailed(147);
      if CurrentKey = 0 then
        TestFailed(148);
      WriteInteger('', 'Item5', 5);
      WriteInteger('5', 'Item6', 6);
      if ReadInteger('', 'Item5', 0) <> 5 then
        TestFailed(150);
      if ReadInteger('5', 'Item6', 0) <> 6 then
        TestFailed(160);
    finally
      Free;
    end;


    rini:=TRegistryIniFile.Create(STestRegPath);
    with rini do
    try
      if ReadString('', 'Item1', '') <> '1' then
        TestFailed(201);
      { \ is not allowed as a section name }
      if ReadString('\', 'Item1', '') = '1' then
        TestFailed(202);
      if ReadString('1', 'Item2', '') <> '2' then
        TestFailed(205);
      { Trailing \ is allowed }
      if ReadString('1\', 'Item2', '') <> '2' then
        TestFailed(206);
      if ReadString('', 'Item1', '') <> '1' then
        TestFailed(210);
      if not ValueExists('', 'Item4') then
        TestFailed(215);
      if not ValueExists('1', 'Item2') then
        TestFailed(220);
      if ReadInteger('2\3\4\5', 'Item6', 0) <> 6 then
        TestFailed(225);
      if ReadInteger('2\3\4', 'Item5', 0) <> 5 then
        TestFailed(230);

      EraseSection('2');
      if SectionExists('2\3') then
        TestFailed(245);
      if ValueExists('2\3\4', 'Item5') then
        TestFailed(240);

      WriteString('2\3\4', 'Item10', '10');
      if ReadInteger('2\3\4', 'Item10', 0) <> 10 then
        TestFailed(245);

      { Check access via a full path }
      if not SectionExists('\' + STestRegPath) then
        TestFailed(250);
      if ReadInteger('\2\3\4', 'Item10', 0) = 10 then
        TestFailed(255);
      if ReadInteger('\' + STestRegPath + '\2\3\4', 'Item10', 0) <> 10 then
        TestFailed(260);
    finally
      Free;
    end;

  finally
    ClearReg;
  end;

  { Test if all test keys have been deleted by ClearReg() }
  reg:=TRegistry.Create;
  try
    if reg.KeyExists(STestRegPath) then
      TestFailed(501);
    if reg.OpenKey(STestRegPath, False) then
      TestFailed(502);
    if reg.OpenKey(STestRegPath + '\2', False) then
      TestFailed(503);
  finally
    reg.Free;
  end;
end;

procedure DeleteUserXmlFile;
begin
{$ifdef FPC}
  DeleteFile(Includetrailingpathdelimiter(GetAppConfigDir(False))+'reg.xml');
  RemoveDir(GetAppConfigDir(False));
{$endif FPC}
end;

begin
  try
    DoRegTest2;
  finally
    DeleteUserXmlFile;
  end;
end.

