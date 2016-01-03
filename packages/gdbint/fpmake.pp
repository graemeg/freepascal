{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

<<<<<<< HEAD
<<<<<<< HEAD
uses fpmkunit, classes, sysutils;

{$endif ALLPACKAGES}

const
  GdbLibName = 'libgdb.a';
  MinGWGdbLibName = 'libmingw32.a';

procedure BeforeCompile_gdbint(Sender: TObject);
var
  L : TStrings;
  P : TPackage;
  GdbLibDir, GdbLibFile: string;
  GdbLibFound: boolean;
  GdbintTarget, GdbVerTarget: TTarget;
begin
  P := Sender as TPackage;
  // Search for a libgdb file.
  GdbLibFound:=false;

  // First try the environment setting GDBLIBDIR
  GdbLibDir := GetEnvironmentVariable('GDBLIBDIR');
  if (GdbLibDir<>'') then
    begin
      if DirectoryExists(GdbLibDir) then
        begin
          GdbLibFile:=IncludeTrailingPathDelimiter(GdbLibDir)+GdbLibName;
          if not FileExists(GdbLibFile) then
            Installer.BuildEngine.Log(vlCommand,
              'GDBLIBDIR environment variable set, but libgdb not found. ('+GdbLibFile+')')
          else
            GdbLibFound:=true;
        end
      else
        Installer.BuildEngine.Log(vlCommand,
          'GDBLIBDIR environment variable set, but directory does not exist. ('+GdbLibDir+')');
    end;

  // Try the default locations
  if not GdbLibFound then
    begin
      GdbLibDir:=Installer.BuildEngine.AddPathPrefix(p,'..'+PathDelim+'..'+PathDelim+'libgdb');
      if DirectoryExists(GdbLibDir) then
        begin
          GdbLibDir:=GdbLibDir+PathDelim+OSToString(Defaults.OS);
          GdbLibFile:=GdbLibDir+PathDelim+GdbLibName;
          if FileExists(GdbLibFile) then
            GdbLibFound:=true
          else
            begin
              GdbLibDir:=GdbLibDir+PathDelim+CPUToString(Defaults.CPU);
              GdbLibFile:=GdbLibDir+PathDelim+GdbLibName;
              GdbLibFound:=FileExists(GdbLibFile);
            end;
        end;
    end;

  GdbVerTarget:=TTarget(p.Targets.ItemByName('gdbver'));
  GdbintTarget:=TTarget(p.Targets.ItemByName('gdbint'));

  if GdbLibFound then
    Installer.BuildEngine.Log(vlCommand,'File libgdb.a found ('+GdbLibFile+')')
  else
    Installer.BuildEngine.Log(vlCommand,'File libgdb.a not found');
  if GdbLibFound then
    begin
      // Detect if gdblib.inc is available
      if FileExists(GDBLibDir+PathDelim+'gdblib.inc') then
        begin
          P.Options.Add('-dUSE_GDBLIBINC');
          P.Options.Add('-Fi'+GdbLibDir);
          P.Options.Add('-Fl'+GdbLibDir);
          // No need to use gdbver in this case
          p.Targets.Delete(GdbVerTarget.Index);
          Installer.BuildEngine.Log(vlCommand,'Using gdblib.inc include file')
        end
     // When we're cross-compiling, running the gdbver executable to detect the
     // gdb-version is not possible, unless a i386-win32 to
     // i386-go32v2 compilation is performed.
     else if (not Defaults.IsBuildDifferentFromTarget
       or ((Defaults.CPU=i386) and (Defaults.OS=go32v2) and
           (Defaults.BuildOS=win32) and (Defaults.BuildCPU=i386))
     ) then
    begin
      P.Options.Add('-Fl'+GdbLibDir);
      Installer.BuildEngine.CreateOutputDir(p);
      Installer.BuildEngine.Log(vlCommand,'GDB-lib found, compiling and running gdbver to obtain GDB-version');
      Installer.BuildEngine.Compile(P,GdbVerTarget);
      Installer.BuildEngine.ExecuteCommand(Installer.BuildEngine.AddPathPrefix(p,p.
        GetBinOutputDir(Defaults.CPU, Defaults.OS))+PathDelim+
        AddProgramExtension('gdbver',Defaults.BuildOS),'-o ' +
        Installer.BuildEngine.AddPathPrefix(p,'src'+PathDelim+'gdbver.inc'));

      with GdbintTarget.Dependencies do
        AddInclude('gdbver.inc');
      // Pass -dUSE_MINGW_GDB to the compiler when a MinGW gdb is used
      if FileExists(GdbLibDir+PathDelim+MinGWGdbLibName) then
        begin
          P.Options.Add('-dUSE_MINGW_GDB');
          Installer.BuildEngine.Log(vlCommand,'Using GDB (MinGW)')
        end
      else
        begin
          Installer.BuildEngine.Log(vlCommand,'Using GDB')
        end;
    end
    end
  else
    begin
      // No suitable gdb found
      // No need to compile gdbver.
      p.Targets.Delete(GdbVerTarget.Index);
      // use gdb_nogdb.inc
      L := TStringList.Create;
      try
        if P.Directory<>'' then
          L.values['src'+DirectorySeparator+'gdbver_nogdb.inc'] := IncludeTrailingPathDelimiter(P.Directory) +'src'+DirectorySeparator+'gdbver.inc'
        else
          L.values['src'+DirectorySeparator+'gdbver_nogdb.inc'] := 'src'+DirectorySeparator+'gdbver.inc';
        Installer.BuildEngine.cmdcopyfiles(L, Installer.BuildEngine.StartDir, nil);
        with GdbintTarget.Dependencies do
          AddInclude('gdbver.inc');
      finally
        L.Free;
      end;

    end;
end;

procedure AfterCompile_gdbint(Sender: TObject);
var
  L : TStrings;
begin
  // Remove the generated gdbver.inc
  L := TStringList.Create;
  try
    L.add(IncludeTrailingPathDelimiter(Installer.BuildEngine.StartDir)+'src/gdbver.inc');
    Installer.BuildEngine.CmdDeleteFiles(L);
  finally
    L.Free;
  end;
end;

procedure add_gdbint(const ADirectory: string);
=======
uses fpmkunit;
>>>>>>> graemeg/fixes_2_2
=======
uses fpmkunit;
>>>>>>> origin/fixes_2_2

Var
  P : TPackage;
  T : TTarget;
begin
  With Installer do
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    P:=AddPackage('gdbint');
    P.ShortName:='gdb';
    P.Directory:=ADirectory;
    P.Version:='3.1.1';
    P.Author := 'Library : Cygnus, header: Peter Vreman';
    P.License := 'Library: GPL2 or later, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Interface to libgdb, the GDB debugger in library format';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?
    // In case of using a buildunit, it is not possible to compile a single
    // file within the BeforeCompile event.
    P.SupportBuildModes:= [bmOneByOne];

    P.OSes:=[aix,beos,haiku,freebsd,netbsd,openbsd,linux,win32,win64,go32v2,dragonfly,solaris];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    P.BeforeCompileProc:=@BeforeCompile_gdbint;
    P.AfterCompileProc:=@AfterCompile_gdbint;

    T := p.Targets.AddProgram('src'+PathDelim+'gdbver.pp');
    T.Install := false;
    //
    // NOTE: the gdbver.inc dependencies gives warnings because the makefile.fpc
    // does a "cp src/gdbver_nogdb.inc src/gdbver.inc" to create it
=======
=======
>>>>>>> origin/fixes_2_2
{$endif ALLPACKAGES}

    P:=AddPackage('gdbint');
{$ifdef ALLPACKAGES}
    P.Directory:='gdbint';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('gdbcon.pp');
      with T.Dependencies do
        begin
          AddUnit('gdbint');
        end;
    T:=P.Targets.AddUnit('gdbint.pp');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.add('examples');
    P.Targets.AddExampleProgram('testgdb.pp');
    P.Targets.AddExampleProgram('symify.pp');
    P.Targets.AddExampleUnit('mingw.pas');

    P.Sources.AddSrc('src/gdbver_nogdb.inc');
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_gdbint('');
  Installer.Run;
end.
{$endif ALLPACKAGES}

=======
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('gdbver.inc');
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
