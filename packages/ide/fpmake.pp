{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses
  fpmkunit,
  sysutils;
{$endif ALLPACKAGES}

const
  NoGDBOption: boolean = false;
  GDBMIOption: boolean = false;
  GDBMI_Disabled: boolean = false;
  GDBMI_DEFAULT_OSes = [aix, darwin, freebsd, haiku,linux, netbsd, openbsd, solaris, win32, win64];

procedure ide_check_gdb_availability(Sender: TObject);

  function DetectLibGDBDir: string;

  var
    SearchPath: string;

  const
    LibGDBName = 'libgdb.a';

  begin
    result:='';
    // First look for the GDBLIBDIR environment variable
    SearchPath := GetEnvironmentVariable('GDBLIBDIR');
    if (SearchPath<>'') and DirectoryExists(SearchPath) and FileExists(IncludeTrailingPathDelimiter(SearchPath)+LibGDBName) then
      begin
        result:=IncludeTrailingPathDelimiter(SearchPath);
        exit;
      end;
    // Search in the default locations
    SearchPath := '..'+PathDelim+'libgdb'+PathDelim+OSToString(Defaults.OS)+PathDelim+CPUToString(Defaults.CPU)+PathDelim;
    if DirectoryExists(SearchPath) and FileExists(SearchPath+LibGDBName) then
      begin
        result := SearchPath;
        exit;
      end;
    SearchPath := '..'+PathDelim+'libgdb'+PathDelim+OSToString(Defaults.OS)+PathDelim;
    if DirectoryExists(SearchPath) and FileExists(SearchPath+LibGDBName) then
      begin
        result := SearchPath;
        exit;
      end;
    // No custom libgdb.a found, try using system default library if available
    SearchPath := '..'+PathDelim+'lib'+PathDelim;
    if DirectoryExists(SearchPath) and FileExists(SearchPath+LibGDBName) then
      begin
        result := SearchPath;
        installer.BuildEngine.Log(vlWarning, 'Using system default libgdb file located in '+Result);
        exit;
      end;
    SearchPath := '..'+PathDelim+'usr'+PathDelim+'lib'+PathDelim;
    if DirectoryExists(SearchPath) and FileExists(SearchPath+LibGDBName) then
      begin
        result := SearchPath;
        installer.BuildEngine.Log(vlWarning, 'Using system default libgdb file located in '+Result);
        exit;
      end;
    SearchPath := '..'+PathDelim+'usr'+PathDelim+'local'+PathDelim+'lib'+PathDelim;
    if DirectoryExists(SearchPath) and FileExists(SearchPath+LibGDBName) then
      begin
        result := SearchPath;
        installer.BuildEngine.Log(vlWarning, 'Using system default libgdb file located in '+Result);
        exit;
      end;
  end;

var
  GDBLibDir: string;
  P: TPackage;

begin
  P := sender as TPackage;
  with installer do
    begin
    if GDBMIOption then
      begin
        BuildEngine.log(vlCommand, 'Compiling IDE with GDB/MI debugger support, LibGDB is not needed');
        P.Options.Add('-dGDBMI');
        { AIX also requires -CTsmalltoc for gdbmi }
        if Defaults.OS=aix then
          P.Options.Add('-CTsmalltoc');
      end
    else if not (NoGDBOption) then
      begin
        // Detection of GDB.
        GDBLibDir := DetectLibGDBDir;
        if GDBLibDir<>'' then
          begin
            // Include GDB
            BuildEngine.log(vlCommand, 'LibGDB was found, build IDE with debugger support');
            if FileExists(GDBLibDir+'gdblib.inc') then
              begin
                P.Options.Add('-dUSE_GDBLIBINC');
                P.Options.Add('-I'+GDBLibDir);
              end;
            P.Options.Add('-Fl'+GDBLibDir);

            case Defaults.OS of
              win32,
              win64 :   begin
                          P.Options.Add('-Xe');
                          P.Options.Add('-k--allow-multiple-definition');
                        end;
              freebsd : begin
                          P.Options.Add('-Fl/usr/local/lib');
                          P.Options.Add('-Xd');
                        end;
              openbsd : begin
                          P.Options.Add('-Fl/usr/local/lib');
                          P.Options.Add('-Fl/usr/lib');
                          P.Options.Add('-Xd');
                        end;
              netbsd  : P.Options.Add('-Xd');
              linux   : P.Options.Add('-Xd');
              aix     : begin
                          P.Options.Add('-Xd');
                          P.Options.Add('-Fl/opt/freeware/lib');
                          P.Options.Add('-k-bbigtoc');
                        end;
            end; {case}

            P.NeedLibc := true;
          end
        else
          begin
          BuildEngine.log(vlCommand, 'LibGDB was not found, IDE has no debugger support');
          P.Options.Add('-dNODEBUG');
          end;
      end
    else
      begin
      BuildEngine.log(vlCommand, 'Debugger support disabled');
      P.Options.Add('-dNODEBUG');
      end;
    end;
end;

procedure add_ide_comandlineoptions();
begin
  AddCustomFpmakeCommandlineOption('CompilerTarget','Target CPU for the IDE''s compiler');
  AddCustomFpmakeCommandlineOption('NoGDB','If value=1 or ''Y'', no GDB support');
  AddCustomFpmakeCommandlineOption('NoGDBMI','If value=1 or ''Y'', explicitly disable GDB/MI option');
  AddCustomFpmakeCommandlineOption('GDBMI','If value=1 or ''Y'', builds IDE with GDB/MI support (no need for LibGDB)');
  AddCustomFpmakeCommandlineOption('NoIDE','If value=1 or ''Y'', the IDE will be skipped');
  AddCustomFpmakeCommandlineOption('IDE','If value=1 or ''Y'', the IDE will be build for each target');
end;

procedure add_ide(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;
  CompilerTarget : TCpu;
  CompilerDir,
  s: string;

begin
  With Installer do
    begin
    s := GetCustomFpmakeCommandlineOptionValue('NoIDE');
    if (s='1') or (s='Y') then
      Exit;

    s := GetCustomFpmakeCommandlineOptionValue('NoGDB');
    if (s='1') or (s='Y') then
     NoGDBOption := true;
    s := GetCustomFpmakeCommandlineOptionValue('NOGDBMI');
    if (s='1') or (s='Y') then
     GDBMI_Disabled := true;
    if not GDBMI_Disabled then
      begin
        s := GetCustomFpmakeCommandlineOptionValue('GDBMI');
        if (s='1') or (s='Y') then
          GDBMIOption := true;
        if (Defaults.OS in GDBMI_DEFAULT_OSes) then
          GDBMIOption := True;
      end;
    s :=GetCustomFpmakeCommandlineOptionValue('CompilerTarget');
    if s <> '' then
      CompilerTarget:=StringToCPU(s)
    else
      CompilerTarget:=Defaults.CPU;
    { Only try to build natively }
    { or for cross-compile if the resulting executable
      does not depend on C libs }
    if ((GDBMIOption or NoGDBOption) and
        ((Defaults.BuildOS=Defaults.OS) and (Defaults.BuildCPU=Defaults.CPU)
         or (Defaults.OS in [go32v2,win32,win64,linux,freebsd])
         or not Defaults.SkipCrossPrograms)) or
       { This is the list of native targets that can be compiled natively with gdbint packages }
       ((((Defaults.BuildOS=Defaults.OS) and (Defaults.BuildCPU=Defaults.CPU)) or
          not Defaults.SkipCrossPrograms) and
        (Defaults.OS in [go32v2,win32,win64,linux,freebsd,os2,emx,beos,haiku])
       ) then
      begin
        P:=AddPackage('ide');
        P.Version:='3.3.1';
{$ifdef ALLPACKAGES}
        P.Directory:=ADirectory;
{$endif ALLPACKAGES}

        s :=GetCustomFpmakeCommandlineOptionValue('IDE');
        if (s='1') or (s='Y') then
          P.OSes := AllOSes
        else
          P.OSes := AllOSes-[darwin];

        P.Dependencies.Add('rtl-extra');
        P.Dependencies.Add('fv');
        P.Dependencies.Add('chm');
        { This one is only needed if DEBUG is set }
        P.Dependencies.Add('regexpr');
        if not (NoGDBOption) and not (GDBMIOption) then
          P.Dependencies.Add('gdbint',AllOSes-AllAmigaLikeOSes);
        if GDBMIOption then
          P.Dependencies.Add('fcl-process');
        P.Dependencies.Add('graph',[go32v2]);
        P.Dependencies.Add('ami-extra',AllAmigaLikeOSes);

        P.SupportBuildModes:=[bmOneByOne];

        P.Options.Add('-Ur');
        P.Options.Add('-dNOCATCH');
        P.Options.Add('-dBrowserCol');
        P.Options.Add('-dGDB');
        
        CompilerDir:=P.Directory +'../../compiler';

        P.Options.Add('-d'+CPUToString(CompilerTarget));
        P.Options.Add('-Fu'+CompilerDir);
        P.Options.Add('-Fu'+CompilerDir+'/'+CPUToString(CompilerTarget));
        P.Options.Add('-Fu'+CompilerDir+'/targets');
        P.Options.Add('-Fu'+CompilerDir+'/systems');
        P.Options.Add('-Fi'+CompilerDir+'/'+CPUToString(CompilerTarget));
        P.Options.Add('-Fi'+CompilerDir);
        
        if CompilerTarget in [x86_64, i386, i8086] then
          P.Options.Add('-Fu'+CompilerDir+'/x86');
        
        if CompilerTarget in [powerpc, powerpc64] then
          P.Options.Add('-Fu'+CompilerDir+'/ppcgen');

        if CompilerTarget in [arm, aarch64] then
          P.Options.Add('-Fu'+CompilerDir+'/armgen');

        if CompilerTarget in [sparc, sparc64] then
          begin
              P.Options.Add('-Fu'+CompilerDir+'/sparcgen');
              P.Options.add('-Fi'+CompilerDir+'/sparcgen');
          end;
        if CompilerTarget in [riscv32, riscv64] then
          begin
              P.Options.Add('-Fu'+CompilerDir+'/riscv');
          end;
        
        if CompilerTarget = mipsel then
          P.Options.Add('-Fu'+CompilerDir+'/mips');

        { powerpc64-aix compiled IDE needs -CTsmalltoc option }
        if (Defaults.OS=aix) and (Defaults.CPU=powerpc64) then
        P.Options.Add('-CTsmalltoc');
        
        { Handle SPECIALLINK environment variable if available }
        s:=GetEnvironmentVariable('SPECIALLINK');
        if s<>'' then
          P.Options.Add(s);
        
        P.Options.Add('-Sg');
        P.IncludePath.Add('compiler');

        T:=P.Targets.AddProgram('fp.pas');
        if CompilerTarget<>Defaults.CPU then
          begin
            T.SetExeName(CPUToString(CompilerTarget)+'-fp');
            P.SetUnitsOutputDir(P.GetUnitsOutputDir(Defaults.BuildCPU,Defaults.BuildOS)+CPUToString(CompilerTarget));
            P.Options.Add('-dCROSSGDB');
          end;

        with T.Dependencies do
          begin
            AddUnit('compunit');
          end;

        T:=P.Targets.AddUnit('compunit.pas');
        T.Directory:='compiler';
        T.Install:=false;

        P.InstallFiles.Add('fp.ans','$(bininstalldir)');
        P.InstallFiles.Add('gplprog.pt','$(bininstalldir)');
        P.InstallFiles.Add('gplunit.pt','$(bininstalldir)');
        P.InstallFiles.Add('program.pt','$(bininstalldir)');
        P.InstallFiles.Add('unit.pt','$(bininstalldir)');
        P.InstallFiles.Add('cvsco.tdf','$(bininstalldir)');
        P.InstallFiles.Add('cvsdiff.tdf','$(bininstalldir)');
        P.InstallFiles.Add('cvsup.tdf','$(bininstalldir)');
        P.InstallFiles.Add('grep.tdf','$(bininstalldir)');
        P.InstallFiles.Add('tpgrep.tdf','$(bininstalldir)');
        P.InstallFiles.Add('fp32.ico', [win32, win64], '$(bininstalldir)');

        with P.Sources do
        begin
        AddDoc('readme.ide');
        AddSrc('README.txt');
        AddSrc('TODO.txt');
        AddSrc('fp.ans');
        AddSrcFiles('*.tdf',P.Directory);
        AddSrcFiles('*.pas',P.Directory,true);
        AddSrcFiles('*.inc',P.Directory,true);
        AddSrcFiles('*.rc',P.Directory);
        AddSrcFiles('*.ico',P.Directory);
        AddSrcFiles('*.term',P.Directory);
        AddSrcFiles('*.pt',P.Directory);
        end;

        P.CleanFiles.Add('$(UNITSOUTPUTDIR)ppheap.ppu');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)compiler.ppu');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)comphook.ppu');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)cpuinfo.ppu');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)browcol.ppu');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)ppheap.o');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)compiler.o');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)comphook.o');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)cpuinfo.o');
        P.CleanFiles.Add('$(UNITSOUTPUTDIR)browcol.o');

        P.BeforeCompileProc:=@ide_check_gdb_availability;
      end;
  end;
end;

{$ifndef ALLPACKAGES}
begin
  add_ide_comandlineoptions();
  add_ide('');
  Installer.Run;
end.
{$endif ALLPACKAGES}

