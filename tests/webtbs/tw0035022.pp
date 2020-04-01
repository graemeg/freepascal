{ %TARGET=win32,win64,wince }

program tw0035022;

{$apptype console}
{$mode objfpc}{$h+}
{$ASSERTIONS ON}

uses
    registry, sysutils, classes;

const
  ROOT = 'Software';
  subFPCREGINITEST = 'FreePascalRegIniTest';
  subRegIni = 'RegIni';
  subStrings = 'FPCTESTString';
  fqFREEPASCALREGINITEST = Root + '\'+ subFPCREGINITEST;
  fqFPCTESTRegIni = fqFREEPASCALREGINITEST + '\' + subRegIni;
  fqFPCTESTStrings = fqFPCTESTRegIni+'\' + subStrings;
  fqWrongFPCTESTStrings = Root + '\' + subStrings;
  idString1 = 'String1';
  valValue1 = 'Value1';

procedure CheckCreate;
var
  Reg: TRegistry;
  S, SKey: String;
  B: Boolean;
begin
  write('CheckCreate: ');
  Reg := TRegistry.Create(KEY_READ);
  try
    Reg.RootKey := HKEY_CURRENT_USER;
    SKey := fqFPCTESTRegIni;
    B := Reg.OpenKeyReadOnly(SKey);
    Assert(B,format('Error: RegOpenKeyReadOnly(''%s'') failed.',[SKey]));

    SKey := subStrings;
    B := Reg.OpenKeyReadOnly(Skey);
    Assert(B,format('Error: RegOpenKeyReadOnly(''%s'') failed.',[fqFPCTESTStrings]));

    S := Reg.ReadString(idString1);
    Assert(S=valValue1,format('ReadString(''%s''): expected '+'%s, but found: ''%s''',[idString1,valValue1,S]));

    Reg.CloseKey;

    writeln('OK');
  finally
    Reg.Free;
  end;

end;

procedure FindErroneousEntries;
var
  Reg: TRegistry;
  B: Boolean;
begin
  write('FindErroneousEntries: ');
  Reg := TRegistry.Create(KEY_READ);
  try
    B := Reg.OpenKeyReadOnly(fqWrongFPCTESTStrings);
    Reg.CloseKey;
    Assert(not B, format('RegOpenKeyReadOnly found %s, which at this point is unexpected.',[fqWrongFPCTESTStrings]));
    writeln(' no erroneous entries found (OK).');
  finally
    Reg.Free;
  end;
end;

procedure CreateTestEntries;
var
  RegIni: TRegIniFile;
  B: Boolean;
  function TryOpenKey(Key: String; CanCreate: Boolean): Boolean;
  begin
    Result := RegIni.OpenKey(Key, CanCreate);
  end;

  function TryWriteString(Section, Ident, Value: String): Boolean;
  begin
    Result := False;
    try
      RegIni.WriteString(Section, Ident, Value);
      Result := True;
    except
      on E: Exception do
    end;
  end;

begin
  write('CreateTestEntries: ');
  RegIni := TRegIniFile.Create(Root);
  try
    Assert(RegIni.CurrentPath=Root,'Expected: CurrenPath='+Root);
    B := RegIni.CreateKey(subFPCREGINITEST);
    Assert(B,format('Error: CreateKey(''%s'') failed.',[fqFREEPASCALREGINITEST]));

    B := TryOpenKey(subFPCREGINITEST,False);
    Assert(B,format('Error: OpenKey(''%s'') failed.',[fqFREEPASCALREGINITEST]));

    Assert(RegIni.CurrentPath=fqFREEPASCALREGINITEST,'Expected: CurrenPath='+fqFREEPASCALREGINITEST);

    B := TryOpenKey(subRegIni,True);
    Assert(B,format('Error: OpenKey(''%s'') failed.',[fqFPCTESTRegIni]));
    Assert(RegIni.CurrentPath=fqFPCTESTRegIni,'Expected: CurrenPath='+fqFPCTESTRegIni);

    B := TryWriteString(subStrings,idString1,valValue1);
    Assert(B,format('Error: WriteString(''%s'',''%s'',''%s'') failed.',[fqFPCTESTStrings,idString1,valValue1]));

    writeln('OK');
  finally
    RegIni.Free;
  end;

end;

procedure DeleteFPCTESTEntries;
  procedure DeleteStrings;
  var
    Reg: TRegistry;
    B: Boolean;
  begin
    Reg := TRegistry.Create(KEY_ALL_ACCESS);
    try
      Reg.RootKey := HKEY_CURRENT_USER;
      if Reg.KeyExists(fqFPCTESTStrings) then
      begin
        B := Reg.OpenKey(fqFPCTESTStrings, False);
        //writeln('OpenKey: ',B);
        if B then
        begin
          B := not Reg.ValueExists(idString1) or Reg.DeleteValue(idString1);
          Assert(B, format('Error DeleteValue(''%s'') in %s',[idString1,fqFPCTESTStrings]));
        end;
        Reg.CloseKey;
      end;

      if Reg.KeyExists(fqWrongFPCTESTStrings) then
      begin
        B := Reg.OpenKey(fqWrongFPCTESTStrings, False);
        //writeln('OpenKey: ',B);
        if B then
        begin
          B := not Reg.ValueExists(idString1) or Reg.DeleteValue(idString1);
          Assert(B, format('Error DeleteValue(''%s'') in %s',[idString1,fqWrongFPCTESTStrings]));
        end;
        Reg.CloseKey;
      end;
    finally
      Reg.Free;
    end;
  end;

  procedure DeleteEmptyKey(Key: String);
  var
    Reg: TRegistry;
    B: Boolean;
  begin
    Reg := TRegistry.Create(KEY_ALL_ACCESS);
    try
      Reg.RootKey := HKEY_CURRENT_USER;
      if Reg.KeyExists(Key) then
      begin
        B := Reg.DeleteKey(Key);
        Assert(B, format('Error DeleteKey(''%s'')',[Key]));
      end;
    finally
      Reg.Free;
    end;
  end;

begin
  DeleteStrings;
  DeleteEmptyKey(fqFPCTESTStrings);
  DeleteEmptyKey(fqWrongFPCTESTStrings);
  DeleteEmptyKey(fqFPCTESTRegIni);
  DeleteEmptyKey(fqFREEPASCALREGINITEST);
end;

begin
  DeleteFPCTESTEntries;
  CreateTestEntries;
  CheckCreate;
  FindErroneousEntries;
  DeleteFPCTESTEntries;
end.
