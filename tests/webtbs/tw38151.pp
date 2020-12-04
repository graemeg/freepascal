{ %TARGET = win32,win64,wince }

program tw38151;

{$mode objfpc}{$H+}

uses
  ActiveX, ComObj, Variants;

procedure TestVoice;
var
  SpVoice, SpVoicesList, Voice: Variant;
begin
  CoInitialize(Nil);
  try
    SpVoice := CreateOleObject('SAPI.SpVoice');
    if VarIsNull(SpVoice) or VarIsEmpty(SpVoice) then
      Exit;
    SpVoicesList := SpVoice.GetVoices();
    if VarIsNull(SpVoicesList) or VarIsEmpty(SpVoicesList) then
      Exit;
    if SpVoicesList.Count = 0 then
      Exit;
    SpVoice.Voice := SpVoicesList.Item(0);
    Voice := SpVoicesList.Item(0);
    SpVoice.Voice := Voice;
  finally
    VarClear(Voice);
    VarClear(SpVoicesList);
    VarClear(SpVoice);
    CoUninitialize;
  end;
end;

begin
  TestVoice;
end.
