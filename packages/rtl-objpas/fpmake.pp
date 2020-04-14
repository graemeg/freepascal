{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_rtl_objpas(const ADirectory: string);

Const 
  // All Unices have full set of KVM+Crt in unix/ except QNX which is not
  // in workable state atm.
  UnixLikes = AllUnixOSes -[QNX]; // qnx never was active in 2.x afaik

//  AllUnixOSes  = [Linux,FreeBSD,NetBSD,OpenBSD,Darwin,QNX,BeOS,Solaris,Haiku,iphonesim,aix,Android];
//    unixlikes-[beos];
//
  StrUtilsOSes  = [atari,emx,gba,go32v2,msdos,nds,netware,wince,nativent,os2,netwlibc,symbian,watcom,wii,win32,win64,freertos]+UnixLikes+AllAmigaLikeOSes;
  VarUtilsOSes  = [atari,emx,gba,go32v2,msdos,nds,netware,wince,nativent,os2,netwlibc,symbian,watcom,wii,win32,win64,freertos]+UnixLikes+AllAmigaLikeOSes;
  ConvUtilsOSes = [nativent,netware,netwlibc,win32,win64,wince]+AllAmigaLikeOSes+UnixLikes-[BeOS];
  ConvUtilOSes  = [atari,Go32v2,msdos,os2,emx,freertos];
  DateUtilsOSes = [gba,nativent,nds,netware,netwlibc,symbian,wii,win32,win64,wince,freertos]+UnixLikes+AllAmigaLikeOSes;
  DateUtilOSes  = [atari,Go32v2,msdos,os2,emx,freertos];
  StdConvsOSes  = [NativeNT,Win32,win64,os2,msdos,go32v2,freertos]+UnixLikes-[BeOS];
  FmtBCDOSes    = [atari,emx,gba,go32v2,msdos,nativent,nds,netware,netwlibc,os2,symbian,watcom,wii,win32,win64,wince,freertos]+UnixLikes+AllAmigaLikeOSes;
  VariantsOSes  = [atari,emx,gba,go32v2,msdos,nativent,nds,netware,netwlibc,os2,symbian,watcom,wii,win32,win64,wince,freertos]+UnixLikes+AllAmigaLikeOSes;
  RttiOSes      = [atari,emx,gba,go32v2,msdos,nativent,nds,netware,netwlibc,os2,watcom,wii,win32,win64,wince,freertos]+UnixLikes+AllAmigaLikeOSes;
  UItypesOSes   = [atari,emx,gba,go32v2,msdos,nativent,nds,netware,netwlibc,os2,watcom,wii,win32,win64,wince,freertos]+UnixLikes+AllAmigaLikeOSes-ConvUtilOSes;
  AllTargetsObjPas = DateUtilsOses +DateUtilOSes+
                  VarutilsOses + ConvutilsOSes + ConvutilOSes + StdConvsOSes+
                  FmtBCDOSes + StrUtilsOSes + UITypesOSes;

  CommonSrcOSes = [atari,emx,gba,go32v2,msdos,nds,netware,wince,nativent,os2,netwlibc,symbian,watcom,wii,freertos]+UnixLikes+AllAmigaLikeOSes;

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('rtl-objpas');
    P.ShortName:='rtlo';
    P.Directory:=ADirectory;
    P.Version:='3.3.1';
    P.Author := 'FPC core team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.OSes:=AllTargetsObjPas;
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Email := '';
    P.Description := 'Rtl-objpas, aux. Delphi compat units';
    P.NeedLibC:= false;

    P.SourcePath.Add('src/inc');
    P.SourcePath.Add('src/$(OS)');
    P.SourcePath.Add('src/win',[win32,win64]);
    P.SourcePath.Add('src/common',CommonSrcOSes);

    P.IncludePath.Add('src/inc');
    P.IncludePath.Add('src/$(OS)');
    P.IncludePath.Add('src/$(CPU)');
    P.IncludePath.Add('src/common',CommonSrcOSes);

    T:=P.Targets.AddUnit('system.uitypes.pp',uitypesOses);

    T:=P.Targets.AddUnit('strutils.pp',StrUtilsOses);
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('widestrutils.pp',StrUtilsOses-ConvUtilOSes);
    T:=P.Targets.AddUnit('varutils.pp',VarUtilsOses);
    with T.Dependencies do
      begin
        AddInclude('varutilh.inc');
        AddInclude('varerror.inc');
        AddInclude('varutils.inc',VarUtilsOSes-[win32,win64]);
        AddInclude('cvarutil.inc');
      end;

    // 8.3
    T:=P.Targets.AddUnit('convutil.pp',ConvutilOSes);
    with T.Dependencies do
     begin
       AddInclude('convutil.inc');
     end;
 
    // normal
    T:=P.Targets.AddUnit('convutils.pp',ConvutilsOSes);
    with T.Dependencies do
     begin
       AddInclude('convutil.inc');
     end;

    // 8.3
    T:=P.Targets.AddUnit('dateutil.pp',dateutilOSes);
    with T.Dependencies do
     begin
       AddInclude('dateutil.inc');
     end;
 
    // normal
    T:=P.Targets.AddUnit('dateutils.pp',dateutilsOSes);
    with T.Dependencies do
     begin
       AddInclude('dateutil.inc');
     end;

    T:=P.Targets.AddUnit('stdconvs.pp',StdConvsOSes);
    T.ResourceStrings:=true;
    with T.Dependencies do
     begin
      AddUnit('convutils',ConvUtilsOSes);
      AddUnit('convutil',ConvUtilOSes);
     end;

    T:=P.Targets.AddUnit('fmtbcd.pp',FmtBCDOSes);
    with T.Dependencies do
      AddUnit('variants');

    T:=P.Targets.AddUnit('variants.pp',VariantsOSes);
    T.ResourceStrings:=true; 
    with T.Dependencies do
     begin
       AddUnit('varutils');
       // AddUnit('Math');
     end;

    T:=P.Targets.AddUnit('rtti.pp',RttiOSes);
    with T.Dependencies do
       begin
         AddInclude('invoke.inc',[x86_64],RttiOSes);
       end;
    T.ResourceStrings:=true;
  end
end;
 
{$ifndef ALLPACKAGES}
begin
  add_rtl_objpas('');
  Installer.Run;
end.
{$endif ALLPACKAGES}

