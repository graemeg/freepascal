
Const
  GraphDir = 'inc/graph/';
  SysUtilsDir = 'objpas/sysutils/';
<<<<<<< HEAD
  Unixes = [darwin,dragonfly,freebsd,linux,netbsd,openbsd];
=======
  Unixes = [darwin,freebsd,linux,netbsd,openbsd];
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


Function CurrentOS : String;

begin
  Result:=OSToString(Defaults.OS);
end;

Function CurrentCPU : String;

begin
  Result:=CPUToString(Defaults.CPU);
end;

Function OSDir(CorrectUnix : Boolean = True) : String;

begin
  If CorrectUnix and (Defaults.OS in Unixes) then
    Result:='unix/'
  else
    Result:=IncludeTrailingPathDelimiter(CurrentOS);
end;


Function CPUDir : String;

begin
  Result:=IncludeTrailingPathDelimiter(CurrentCPU);
end;

Procedure InitRTL(Installer : TInstaller);

Var
  O : String;


begin
  With Installer Do
    begin
    StartPackage('rtl');
    Version:={$i %FPCVERSION%};
    Author:='The FPC team';
    O:='-Fi'+OSDir(false)+' -Fiinc -Fi'+CurrentCPU;
    If Defaults.OS in Unixes then
      O:=O+' -Fiunix';
    DefaultPackage.Options:=O;
    end;
end;

Procedure AddSystemDependencies(Targets : TTargets);

Var
  I : Integer;

begin
  With Targets do
    For I:=0 to Count-1 do
      if (TargetItems[i].Name<>'system') then
        TargetItems[i].Dependencies.Add('system');
end;

Procedure AddCurrentOS(T : TTarget = Nil);

Var
  I : Integer;
  O : TOSes;

begin
  If (T<>Nil) then
    begin
    O:=T.OS;
    If (O<>[]) then
      Include(O,Defaults.OS)
    else
      // Don't do anything.  If empty, all will be compiled.
    T.OS:=O;
    end
  else
    With Installer.Targets do
      For I:=0 to Count-1 do
        AddCurrentOS(TargetItems[i]);
end;

Procedure ExcludeCurrentOS(T : TTarget = Nil);

Var
  I : Integer;
  O : TOSes;

begin
  If (T<>Nil) then
    begin
    O:=T.OS;
    If (O=[]) then
      O:=AllOSs;
    Exclude(O,Defaults.OS);
    T.OS:=O;
    end
  else
    With Installer.Targets do
      For I:=0 to Count-1 do
        AddCurrentOS(TargetItems[i]);
end;

Procedure AddDefaultTargets(Installer : TInstaller);

Var
  T : TTarget;

begin
  With Installer.Targets do
    begin

    { Turbo Pascal RTL units }
    T:=AddUnit(OSDir+'dos.pp');
    T.Dependencies.Add('inc/dosh.inc');
    T:=AddUnit(OSDir+'crt.pp');
    T.Directory:=OSDir;
    T.Dependencies.Add('inc/crth.inc');
    T:=AddUnit(OSDir+'graph.pp');
    T.IncludePath.Add(GraphDir);
    With T.Dependencies do
      begin
      Add(GraphDir+'clip.inc');
      Add(GraphDir+'fontdata.inc');
      Add(GraphDir+'graph.inc');
      Add(GraphDir+'gtext.inc');
      Add(GraphDir+'modes.inc');
      Add(GraphDir+'fills.inc');
      Add(GraphDir+'graphh.inc');
      Add(GraphDir+'palette.inc');
      end;
    T:=AddUnit('inc/strings.pp');
    With T.Dependencies do
      begin
      Add(CPUDir+'strings.inc');
      Add(CPUDir+'stringss.inc');
      Add('inc/genstr.inc');
      Add('inc/genstrs.inc');
      Add('inc/stringsi.inc');
      end;

    { Delphi RTL units }
    T:=AddUnit('objpas/rtlconsts.pp');
    T.Resourcestrings:=True;
    T.Dependencies.Add('objpas/rtlconst.inc');
    T:=AddUnit(OSDir+'sysutils.pp');
    T.IncludePath.Add(SysUtilsDir);
    With T.Dependencies do
      begin
      Add(SysUtilsDir+'datih.inc');
      Add(SysUtilsDir+'finah.inc');
      Add(SysUtilsDir+'osutilsh.inc');
      Add(SysUtilsDir+'sysansi.inc');
      Add(SysUtilsDir+'syspchh.inc');
      Add(SysUtilsDir+'systhrdh.inc');
      Add(SysUtilsDir+'syswideh.inc');
      Add(SysUtilsDir+'sysunih.inc');
      Add(SysUtilsDir+'dati.inc');
      Add(SysUtilsDir+'fina.inc');
      Add(SysUtilsDir+'stre.inc');
      Add(SysUtilsDir+'sysformt.inc');
      Add(SysUtilsDir+'syspch.inc');
      Add(SysUtilsDir+'sysuintf.inc');
      Add(SysUtilsDir+'syswide.inc');
      Add(SysUtilsDir+'sysuni.inc');
      Add(SysUtilsDir+'diskh.inc');
      Add(SysUtilsDir+'intfh.inc');
      Add(SysUtilsDir+'strg.inc');
      Add(SysUtilsDir+'sysinth.inc');
      Add(SysUtilsDir+'sysstrh.inc');
      Add(SysUtilsDir+'sysutilh.inc');
      Add(SysUtilsDir+'filutilh.inc');
      Add(SysUtilsDir+'osutil.inc');
      Add(SysUtilsDir+'sysansih.inc');
      Add(SysUtilsDir+'sysint.inc');
      Add(SysUtilsDir+'sysstr.inc');
      Add(SysUtilsDir+'sysutils.inc');
      end;
    T:=AddUnit(OSDIR+'varutils.pp');
    T.ResourceStrings:=True;
    T.IncludePath.Add('objpas');
    T:=AddUnit('inc/variants.pp');
    With T.Dependencies do
      begin
      Add('math');
      Add('varutils');
      end;
    T.ResourceStrings:=True;
    T:=AddUnit('objpas/convutils.pp');
    T.Dependencies.Add('objpas/convutil.inc');
    T.ResourceStrings:=True;
    T:=AddUnit('objpas/dateutils.pp');
    T.Dependencies.Add('objpas/dateutil.inc');
    T:=AddUnit('objpas/strutils.pp');
    T.Dependencies.Add('sysutils');
    T:=AddUnit('objpas/math.pp');
    T.Dependencies.Add('sysutils');
    T.Dependencies.Add(CPUDir+'mathu.inc');
    T.ResourceStrings:=True;
    T:=AddUnit('objpas/sysconst.pp');
    T.Resourcestrings:=true;
    T:=AddUnit('objpas/types.pp');
    T:=AddUnit('objpas/typinfo.pp');
    T.Dependencies.Add('sysutils');
    T.Dependencies.Add('rtlconsts');
    T:=AddUnit('objpas/utf8bidi.pp');
    T:=AddUnit('objpas/character.pas');
    { Free Pascal additions}
    T:=AddUnit(OSDir+'keyboard.pp');
    With T.Dependencies do
      begin
      Add('inc/keybrdh.inc');
      Add('inc/keyboard.inc');
      Add('inc/keyscan.inc');
      end;
    T:=AddUnit(OSDir+'mouse.pp');
    With T.Dependencies do
      begin
      Add('inc/mouseh.inc');
      Add('inc/mouse.inc');
      end;
    T:=AddUnit(OSDir+'video.pp');
    With T.Dependencies do
      begin
      Add('inc/video.inc');
      Add('inc/videoh.inc');
      end;
    T:=AddUnit('inc/matrix.pp');
    T.Dependencies.Add('inc/mmatimp.inc');
    T:=AddUnit('inc/dynlibs.pp');
    T.Dependencies.Add(OSDir+'dynlibs.inc');
    T:=AddUnit('inc/getopts.pp');
    T:=AddUnit('inc/cmem.pp');
    T:=AddUnit('inc/ctypes.pp');
    T:=AddUnit('inc/softfpu.pp');
    T:=AddUnit('inc/ucomplex.pp');
    T:=AddUnit('inc/heaptrc.pp');
    T:=AddUnit('inc/lineinfo.pp');
    T:=AddUnit('inc/lnfodwrf.pp');
    T:=AddUnit('inc/charset.pp');
    // T:=AddUnit('inc/pagemem.pp');
    end;
  AddSystemDependencies(Installer.Targets);
<<<<<<< HEAD
<<<<<<< HEAD
end;
=======
end;
>>>>>>> graemeg/fixes_2_2
=======
end;
>>>>>>> origin/fixes_2_2
