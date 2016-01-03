unit pkgmkconv;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,pkghandler;
  { TMakeFileConverter }

Type
  TSectionType = (stNone,stPackage,stTarget,stclean,stinstall,stCompiler,
                  stDefault,stRequire,stRules,stPrerules);

  TMakeFileConverter = Class(TPackagehandler)
  Private
    FSection : TSectionType;
    FPackageName,
    FpackageDir,
    FPackageOptions,
    FPackageDeps,
    FBuilDUnit,
    FSubName,
    FPackageVersion : String;
    // Reading;
    procedure DoPackageLine(Const S : String);
    Procedure DoTargetLine(Line : String; Var T,R,D : TStrings);
    Procedure DoInstallLine(Line : String; Var IFL : TStrings);
    procedure DoCleanLine(Line : String; Var CFL : TStrings);
    procedure DoRequireLine(Line : String);
    procedure DoCompilerLine(Line : String;Var SD : TStrings);
    // Writing;
    procedure WriteOSCPUCheck(Src: TStrings;OS,CPU : String);
    procedure StartPackage(Src : TStrings; Dir,OS : String);
    procedure EndPackage(Src : TStrings; Dir,OS : String);
    procedure DoTargets(Src,T,R,SD : TStrings; Dir,Prefix : String);
    procedure DoCleans(Src,CFL : TStrings);
    procedure DoInstalls(Src,IFL : TStrings);
    Procedure StartInstaller(Src : TStrings);
    Procedure EndInstaller(Src : TStrings);
    Function GetLine (L : TStrings; Var I : Integer) : String;
    procedure ConvertFile(const AFileName: String; Src: TStrings; Dir,OS : String);
    Procedure ConvertFile(Const Source,Dest: String);
  Public
    Procedure Execute;override;
  end;


implementation

uses
  TypInfo,
  pkgglobals,
  pkgmessages;

Function GetWord(var S : String; Sep : Char) : String;

Var
  L : Integer;

begin
  L:=Pos(Sep,S);
  If (L=0) then
    L:=Length(S)+1;
  Result:=Copy(S,1,L-1);
  Delete(S,1,L);
  S:=Trim(S);
end;

Function GetWord(var S : String) : String;

begin
  Result:=GetWord(S,' ');
end;


Function IsCPU (S: String) : Boolean;

begin
  Result:=Pos(lowercase(S)+',','i386,powerpc,arm,alpha,sparc,x86_64,powerpc64,')<>0
end;

Procedure GetOSCPU(L : String; var OS,CPU : String);

  Procedure Add(Var A : String; ad : String);

  begin
    If (A<>'') then
      A:=A+',';
    A:=A+ad;
  end;


Var
  S : String;

begin
  OS:='';
  CPU:='';
  S:=GetWord(L,',');
  While (S<>'') do
    begin
    If (S<>'all') then
      If IsCPU(S) then
        Add(CPU,S)
      else
        Add(OS,S);
    S:=GetWord(L,',');
    end;
end;



{ TMakeFileConverter }

procedure TMakeFileConverter.StartInstaller(Src: TStrings);

begin
  With Src do
    begin
    Add('{$mode objfpc}{$H+}');
    Add('program fpmake;');
    Add('');
    Add(' { Generated automatically by '+ExtractFileName(Paramstr(0))+' on '+DateToStr(Sysutils.Date)+' }');
    Add('');
    Add('uses fpmkunit;');
    Add('');
    Add('Var');
    Add('  P : TPackage;');
    Add('  T : TTarget;');
    Add('');
    Add('begin');
    Add('  With Installer do ');
    Add('    begin');
    end;
end;

procedure TMakeFileConverter.EndInstaller(Src: TStrings);
begin
  With Src do
    begin
    Add('    Run;');
    Add('    end;');
    Add('end.');
    Add('');
    end;
end;

Function TMakeFileConverter.GetLine (L : TStrings; Var I : Integer) : String;

Var
  P : Integer;
  OK : Boolean;

begin
  OK:=False;
  Result:='';
  Repeat
    Result:=Result+L[i];
    P:=Pos('#',Result);
    If (P>0) then
      Result:=Copy(Result,1,P-1);
    Result:=Trim(Result);
    P:=Length(Result);
    If (P>0) and (Result[P]='\') then
      Result:=Copy(Result,1,P-1)
    else
      OK:=(Result<>'');
    if Not OK then
      Inc(I);
  Until OK or (I>L.Count-1);
end;

Function SplitNamevalue(Const S : String; Var AName,AValue : String) : boolean;

var
  L : Integer;

begin
  L:=Pos('=',S);
  Result:=(L<>0);
  If Result then
    begin
    AName:=LowerCase(trim(Copy(S,1,L-1)));
    AValue:=S;
    Delete(AValue,1,L);
    AValue:=Trim(Avalue);
    end
  else
    begin
    AName:='';
    AValue:='';
    end;
end;


procedure TMakeFileConverter.StartPackage(Src : TStrings; Dir,OS : String);

Var
  S : String;

begin
  With Src do
    begin
    Add('    { ');
    Add('      '+FPackageName);
    Add('    } ');
    Add('    P:=AddPackage('''+FPackageName+''');');
    If (Dir<>'') then
      Add('    P.Directory:='''+ExcludeTrailingPathDelimiter(Dir)+''';');
    If (OS<>'') and (OS<>'all') then
      Add('    P.OS:=['+OS+'];');
    If (FPackageVersion<>'') then
      Add('    P.Version:='''+FPackageVersion+''';');
    If (FPackageOptions<>'') then
      Add('    P.Options:='''+FPackageOptions+''';');
    If (FPackageDeps<>'') then
      begin
      S:=GetWord(FPackageDeps);
      While S<>'' do
        begin
        Add('    P.Dependencies.Add('''+S+''');');
        S:=GetWord(FPackageDeps);
        end;
      end;
    end;
end;

procedure TMakeFileConverter.EndPackage(Src : TStrings; Dir,OS : String);

begin
  FPackageName:='';
  FPackageVersion:='';
  FPackageOptions:='';
  FBuilDUnit:='';
  FPackageDeps:='';
end;


procedure TMakeFileConverter.DoPackageLine(Const S : String);

Var V,N : String;

begin
  SplitNameValue(S,N,V);
  If (N='name') then
    FPackageName:=V
  else If (N='version') then
    FPackageVersion:=V
  else If (N='main') then
    begin
    FPackageName:='sub';
    FSubName:=V;
    end
  else
    Writeln(StdErr,'Unknown name/value pair in package section :',N);
end;


{
  Convert various entries of type
    XXYY_OSN=words
  to entries of type
    prefix_word=OS1,OS2,OS3
  OS is never empty, 'all' is default.
  }
Procedure AddStrings(Var L : TStrings; Values,Prefix,OS : String) ;

Var
  S,O : String;
  i : integer;

begin
  If (L=Nil) then
    L:=TstringList.Create;
  If prefix<>'' then
    prefix:=prefix+'_';
  S:=GetWord(Values);
  While (S<>'') do
    begin
    S:=Prefix+S;
    I:=L.IndexOfName(S);
    If (I<>-1) then
      begin
      O:=L.Values[S];
      If (O='all') then
        O:='';
      If (O<>'') then
        O:=O+',';
      O:=O+OS;
      L.Values[S]:=O;
      end
    else
      L.Add(S+'='+OS);
    S:=GetWord(Values);
    end;
end;


procedure TMakeFileConverter.DoTargetLine(Line : String; Var T,R,D : TStrings);

Var
  V,N,OS : String;
  P : Integer;

begin
  SplitNameValue(Line,N,V);
  P:=Pos('_',N);
  If (P=0) then
    OS:='all'
  else
    begin
    OS:=N;
    Delete(OS,1,P);
    N:=Copy(N,1,P-1);
    end;
  If (N='dirs') then
    AddStrings(D,V,'',OS)
  else If (N='units') then
    AddStrings(T,V,'unit',OS)
  else If (N='implicitunits') then
    AddStrings(T,V,'unit',OS)
  else If (N='programs') then
    AddStrings(T,V,'program',OS)
  else If (N='examples') then
    AddStrings(T,V,'exampleunit',OS)
  else If (N='rsts') then
    AddStrings(R,V,'',OS)
  else
    Writeln(StdErr,'Unknown name/value pair in target section : ',Line);
end;

procedure TMakeFileConverter.DoInstallLine(Line : String; Var IFL : TStrings);

Var
  S,V,N,OS : String;
  P : Integer;

begin
  SplitNameValue(Line,N,V);
  P:=Pos('_',N);
  If (P=0) then
    OS:='all'
  else
    begin
    OS:=N;
    Delete(OS,1,P);
    N:=Copy(N,1,P-1);
    end;
  If (N='fpcpackage') then
    P:=0 // temporary, needs fixing.
  else If (N='buildunit') then
    FBuildUnit:=V // temporary, needs fixing.
  else If (N='units') then
    begin
    S:=GetWord(V);
    While (S<>'') do
      begin
      AddStrings(IFL,S+'.o','',OS);
      AddStrings(IFL,S+'.ppu','',OS);
      S:=GetWord(V);
      end;
    end
  else
    Writeln(StdErr,'Unknown name/value pair in install section : ',N);
end;

procedure TMakeFileConverter.DoCleanLine(Line : String; Var CFL : TStrings);

Var
  V,N,S,OS : String;
  P : Integer;

begin
  SplitNameValue(Line,N,V);
  P:=Pos('_',N);
  If (P=0) then
    OS:='all'
  else
    begin
    OS:=N;
    Delete(OS,1,P);
    N:=Copy(N,1,P-1);
    end;
  If (N='fpcpackage') then
    P:=0 // temporary, needs fixing.
  else If (N='units') then
    begin
    S:=GetWord(V);
    While (S<>'') do
      begin
      AddStrings(CFL,S+'.o','',OS);
      AddStrings(CFL,S+'.ppu','',OS);
      S:=GetWord(V);
      end;
    end
  else
    Writeln(StdErr,'Unknown name/value pair in clean section : ',N);
end;

procedure TMakeFileConverter.DoRequireLine(Line : String);

Var
  V,N,OS : String;
  P : Integer;

begin
  SplitNameValue(Line,N,V);
  P:=Pos('_',N);
  If (P=0) then
    OS:='all'
  else
    begin
    OS:=N;
    Delete(OS,1,P);
    N:=Copy(N,1,P-1);
    end;
  if (N='packages') then
    FPackageDeps:=V
  else If (N='libc') and (Upcase(V)='Y') then
     P:=0 // Set options ?
  else
    Writeln(StdErr,'Unknown name/value pair in require section : ',N);
end;


procedure TMakeFileConverter.DoCompilerLine(Line : String;Var SD : TStrings);

Var
  V,N,OS : String;
  P : Integer;

begin
  SplitNameValue(Line,N,V);
  P:=Pos('_',N);
  If (P=0) then
    OS:='all'
  else
    begin
    OS:=N;
    Delete(OS,1,P);
    N:=Copy(N,1,P-1);
    end;
  If (N='includedir') then
    FPackageOptions:=Trim(FPackageOptions+' -Fi'+V)
  else If (N='options') then
    FPackageOptions:=Trim(FPackageOptions+' '+V)
  else If (N='targetdir') then
    P:=0 // Ignore
  else if (N='sourcedir') or (N='unitdir') then
    begin
    If (SD=Nil) then
      SD:=TStringList.Create;
    SD.Add(OS+'='+V);
    end
  else
    Writeln(StdErr,'Unknown name/value pair in compiler section : ',N);
end;

Function SearchInDirs(Prefix,AName, Dirs : String) : string;

Var
  D,S : String;

begin
  S:=GetWord(Dirs);
  Result:='';
  While (Result='') and (S<>'') do
    begin
    D:=Prefix+S+PathDelim;
    If FileExists(D+AName+'.pp') or FileExists(D+AName+'.pas') then
      Result:=S;
    S:=GetWord(Dirs);
    end;
end;

procedure TMakeFileConverter.DoTargets(Src,T,R,SD : TStrings; Dir,Prefix : String);

Var
  I,J,P : Integer;
  Pre,N,V,D,DOS,OS,CPU : String;
  Res : Boolean;

begin
  If (Dir<>'') then
    Dir:=IncludeTrailingPathDelimiter(Dir);
  If (Prefix<>'') then
    Prefix:=IncludeTrailingPathDelimiter(Prefix);
  Dir:=Prefix+Dir;
  Res:=False;
  If Assigned(T) then
    For I:=0 to T.Count-1 do
      begin
      T.GetNamevalue(I,N,V);
      P:=Pos('_',N);
      If (P<>0) then
        begin
        Pre:=Copy(N,1,P-1);
        Delete(N,1,P);
        end;
      If Assigned(R) then
        Res:=R.IndexOfName(N)<>-1;
      GetOSCPU(V,OS,CPU);
      Pre[1]:=Upcase(Pre[1]);
      Src.Add('    T:=P.Targets.Add'+Pre+'('''+Prefix+N+'.pp'');');
      If (CPU<>'') then
        Src.Add('    T.CPU:=['+CPU+'];');
      If (OS<>'') then
        Src.Add('    T.OS:=['+OS+'];');
      If res then
        Src.add('    T.ResourceStrings:=True;');
      If (CompareText(FBuildUnit,N)=0) then
        Src.add('    T.Install:=False;');
      if Assigned(SD) then
        for J:=0 to SD.Count-1 do
          begin
          SD.GetNameValue(J,DOS,D);
          If (DOS<>'all') then
            Src.Add('    if (Defaults.OS='+DOS+') then');
          Src.add('      T.Directory:='''+SearchInDirs(Dir,N,D)+''';');
          end;
      end;
end;

procedure TMakeFileConverter.WriteOSCPUCheck(Src: TStrings;OS,CPU : String);

Var
  S : String;

begin
  If (CPU<>'') then
    S:='(Defaults.CPU='+CPU+')';
  If (OS<>'') then
    begin
    IF (S<>'') then
      S:=S+' OR ';
    S:=S+'(Defaults.OS='+CPU+')';
    end;
  If (S<>'') then
    Src.Add('    If '+S+' then');
end;

procedure TMakeFileConverter.DoInstalls(Src,IFL : TStrings);

Var
  I : Integer;
  N,V,OS,CPU : String;

begin
  If Assigned(IFL) then
    For I:=0 to IFL.Count-1 do
      begin
      IFL.GetNamevalue(I,N,V);
      GetOSCPU(V,OS,CPU);
      WriteOSCPUCheck(Src,OS,CPU);
      Src.add('      P.InstallFiles.Add('''+N+''');');
      end;
end;

procedure TMakeFileConverter.DoCleans(Src,CFL : TStrings);

Var
  I : Integer;
  N,V,OS,CPU : String;


begin
  If Assigned(CFL) then
    For I:=0 to CFL.Count-1 do
      begin
      CFL.GetNamevalue(I,N,V);
      GetOSCPU(V,OS,CPU);
      WriteOSCPUCheck(Src,OS,CPU);
      Src.add('      P.CleanFiles.Add('''+N+''');');
      end;
end;



procedure TMakeFileConverter.ConvertFile(const AFileName: String; Src: TStrings; Dir,OS : String);

  Function IsSection(var S : String) : Boolean;

  Var
    L : Integer;

  begin
    L:=Length(S);
    Result:=(L>0) and (S[1]='[') and (S[L]=']');
    If Result then
      S:=trim(Copy(S,2,L-2));
  end;

Var
  R,L,T,D,S,SD,IFL,CFL : TStrings;
  I,J : Integer;
  Prefix,Line,DN : String;
  B : Boolean;

begin
  Log(vlDebug,'Converting '+AFileName);
  T:=Nil;
  D:=Nil;
  S:=Nil;
  SD:=Nil;
  R:=Nil;
  IFL:=Nil;
  CFL:=Nil;
  FPackageOptions:='';
  FPackageDir:='';
  L:=TStringList.Create;
  try
    L.LoadFromFile(AFileName);
    I:=0;
    While (I<L.Count) do
      begin
      Line:=GetLine(L,I);
      If IsSection(Line) then
        begin
        J:=GetEnumValue(TypeInfo(TSectionType),'st'+Line);
        If (J=-1) then
          begin
            FSection:=stNone;
            Error('Unsupported section: '+Line);
          end
        else
          FSection:=TSectiontype(J);
        end
      else
        case FSection of
          stPackage : DoPackageLine(Line);
          stTarget  : DoTargetLine(Line,T,R,D);
          stInstall : DoInstallLine(Line,IFL);
          stClean   : DoCleanLine(Line,CFL);
          stCompiler : DoCompilerLine(Line,SD);
          strequire : DoRequireLine(Line);
        end;
      inc(I);
      end;
    // If there are only 'dir' entries, then there is no package name.
    B:=False;
    if (FPackageName<>'') then
      begin
      Prefix:='';
      B:=FPackageName<>'sub';
      If B then
        StartPackage(Src,Dir,OS)
      else
        Prefix:=Dir;
      DoTargets(Src,T,R,SD,Dir,Prefix);
      DoInstalls(Src,IFL);
      DoCleans(Src,CFL);
      end;
    If Assigned(D) then
      begin
      If (Dir<>'') then
        Dir:=IncludeTrailingPathDelimiter(Dir);
      For I:=0 to D.Count-1 do
        begin
        D.GetNameValue(I,DN,Line);
        If (Line<>'all') and (Line<>'') then
          OS:=Line;
        DN:=Dir+DN+PathDelim;
        If FileExists(DN+'Makefile.fpc') then
          ConvertFile(DN+'Makefile.fpc',Src,DN,OS);
        end;
      end;
    If B then
      EndPackage(Src,Dir,OS);
  Finally
    S.Free;
    IFL.Free;
    CFL.Free;
    D.Free;
    SD.Free;
    T.Free;
    L.Free;
  end;
end;

procedure TMakeFileConverter.ConvertFile(const Source, Dest: String);

Var
  L : TStrings;

begin
  Log(vlInfo,SLogGeneratingFPMake);
  L:=TStringList.Create;
  Try
    StartInstaller(L);
    ConvertFile(Source,L,'','');
    EndInstaller(L);
    L.SaveToFile(Dest);
  Finally
    L.Free;
  end;
end;

procedure TMakeFileConverter.Execute;
begin
  if not FileExists('fpmake.pp') then
    ConvertFile('Makefile.fpc','fpmake.pp')
  else
    Error(SErrConvertFPMakeExists);
end;

begin
  RegisterPkgHandler('convertmk',TMakeFileConverter);
end.
