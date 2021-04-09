{$IFDEF HASAMIGA}
 {$DEFINE NO_UNIT_PROCESS}
{$ENDIF}

{$IFDEF OS2}
 {$DEFINE NO_UNIT_PROCESS}
{$ENDIF OS2}

{$IFDEF GO32V2}
 {$DEFINE NO_UNIT_PROCESS}
{$ENDIF GO32V2}

{$ifndef NO_UNIT_PROCESS}
  {$define HAS_UNIT_PROCESS}
{$endif NO_UNIT_PROCESS}

{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses
  fpmkunit,
{$IFDEF HAS_UNIT_PROCESS}
  process,
{$ENDIF HAS_UNIT_PROCESS}
  sysutils;
{$endif ALLPACKAGES}

{$ifdef HAS_UNIT_PROCESS}
procedure fpcm_update_revision_info(Sender: TObject);

  function ReadSVNLine(AProcess: TProcess; var ALine: string): boolean;
  var
    b,i: byte;
  begin
    result := true;
    ALine := '';
    i := 1;
    repeat
    if i = 0 then
      sleep(100);
    i := AProcess.Output.Read(b,1);
    if i > 0 then
      begin
        if b = 13 then
          continue;
        if b = 10 then
          exit;
        ALine := ALine + chr(b);
      end;
    until not AProcess.Running and (i=0);

    result := (ALine <> '');
  end;

var
  P : TPackage;
  SVNBin : String;
  SVNProcess: TProcess;
  f: text;
  fileurl, line, date, lastdate,
  revision, oldrevstring, olddate : string;
  i, io : longint;
  rev, lastrev, oldrev : longint;

begin
  // If revision.inc does exist, try to update the file with the latest
  // revision from svn. And include this information in the fpcmake
  // executable.
  With installer do
    begin
      if not FileExists(BuildEngine.AddPathPrefix(P,'revision.inc')) then
        begin
          BuildEngine.Log(vlWarning, 'File revision.inc not found. Svn-revision will not be included in fpcmake executable.');
          Exit;
        end;

      // Run svn info, and catch output.
      P := sender as TPackage;
      P.Options.Add('-dREVINC');
      SVNBin := ExeSearch(AddProgramExtension('svn', Defaults.BuildOS), GetEnvironmentvariable('PATH'));
      if SVNBin<>'' then
        begin
          SVNProcess := TProcess.create(nil);
          try
            SVNProcess.Executable := SVNBin;
            SVNProcess.Parameters.Add('info');
            SVNProcess.Parameters.Add('-R');
            SVNProcess.Parameters.Add(BuildEngine.AddPathPrefix(P,'fpcmpkg.pp'));
            SVNProcess.Parameters.Add(BuildEngine.AddPathPrefix(P,'fpcmake.pp'));
            SVNProcess.Parameters.Add(BuildEngine.AddPathPrefix(P,'fpcmwr.pp'));
            SVNProcess.Parameters.Add(BuildEngine.AddPathPrefix(P,'fpcmmain.pp'));
            SVNProcess.Parameters.Add(BuildEngine.AddPathPrefix(P,'fpcmdic.pp'));
            SVNProcess.Parameters.Add(BuildEngine.AddPathPrefix(P,'fpcmake.ini'));
            SVNProcess.Parameters.Add(BuildEngine.AddPathPrefix(P,'Makefile.fpc'));
            SVNProcess.Options:=[poUsePipes];
            SVNProcess.Execute;

            // Search for latest revision in output:
            lastrev:=0;
            lastdate:='0';
            while ReadSVNLine(SVNProcess, Line) do
              begin
                i:=pos('URL: ',line);
                if i>0 then
                  begin
                    fileurl:=copy(line,i+length('URL: '),length(line));
                    BuildEngine.Log(vlCommand,'fileurl='+fileurl);
                  end;
                i:=pos('Last Changed Date: ',line);
                if i>0 then
                  begin
                    date:=copy(line,i+length('Last Changed Date: '),length(line));
                    i:=pos(' ',date);
                    if i>0 then
                      date:=copy(date,1,i-1);
                    BuildEngine.Log(vlCommand,'date='+date);
                    if date>lastdate then
                      lastdate:=date;
                  end;
                i:=pos('Last Changed Rev: ',line);
                if i>0 then
                  begin
                    revision:=copy(line,i+length('Last Changed Rev: '),length(line));
                    BuildEngine.Log(vlCommand,'rev='+revision);
                    val(revision,rev);
                    if rev>lastrev then
                      lastrev:=rev;
                  end;
              end;
          finally
            SVNProcess.Free;
          end;

          oldrev:=0;
          olddate:='';
          // Write the latest change-date and revision to file revision.inc
          system.assign(f,BuildEngine.AddPathPrefix(P,'revision.inc'));
          io:=ioresult;
          reset(f);
          io:=ioresult;
          if io<>0 then
            begin
              BuildEngine.Log(vlWarning,'revision.inc reset failed, io='+IntToStr(io));
            end
          else
            begin
              readln(f,oldrevstring);
              close(f);
              BuildEngine.Log(vlCommand, 'oldrevstring '+oldrevstring);
              if oldrevstring[1]='''' then
                oldrevstring:=copy(oldrevstring,2,length(oldrevstring));
              i:=length(oldrevstring);
              if oldrevstring[i]='''' then
                oldrevstring:=copy(oldrevstring,1,i-1);
              i:=pos(' rev ',oldrevstring);
              if i>0 then
                begin
                  val(copy(oldrevstring,i+5,length(oldrevstring)),oldrev);
                  olddate:=copy(oldrevstring,1,i-1);
                  BuildEngine.Log(vlCommand,'Old values '+olddate+' '+IntToStr(oldrev));
                  if (olddate >= lastdate) and (oldrev >= lastrev) then
                    begin
                      BuildEngine.Log(vlCommand,'New values '+lastdate+' '+IntToStr(lastrev));
                      BuildEngine.Log(vlCommand,'Keeping old values');
                      lastrev:=oldrev;
                      lastdate:=olddate;
                    end;
                end;

            end;
          if (lastdate=olddate) and (lastrev=oldrev) then
            BuildEngine.Log(vlCommand,'revision.inc unchanged')
           else
            begin
              BuildEngine.Log(vlCommand,'revision.inc set to '''+lastdate+' rev '+IntToStr(lastrev)+'''');

              system.assign(f,BuildEngine.AddPathPrefix(P,'revision.inc'));
              rewrite(f);
              io:=ioresult;
              if io <> 0 then
                begin
                  BuildEngine.Log(vlError, 'Error opening revision.inc for writing');
                  halt(3);
                end;
              Writeln(f,'''',lastdate,' rev ',lastrev,'''');
              close(f);
            end
        end
      else
        BuildEngine.Log(vlWarning,'Subversion executable (svn) not found. Svn-revision in fpcmake executable might be out of date.');
    end;
end;
{$endif HAS_UNIT_PROCESS}

procedure add_fpcm(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;
  Data2IncBin : String;

begin
  With Installer do
    begin
    P:=AddPackage('utils-fpcm');
    P.ShortName:='fpcm';
    P.OSes:=AllOSes-[embedded,msdos,nativent,win16,macosclassic,atari,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Author := '<various>';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Tool to generate Makefile''s out of Makefile.fpc files';
    P.NeedLibC:= false;

{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';

    P.Dependencies.Add('fcl-base');

    T:=P.Targets.AddProgram('fpcmake.pp');

{$ifdef HAS_UNIT_PROCESS}
    P.BeforeCompileProc:=@fpcm_update_revision_info;
{$else HAS_UNIT_PROCESS}
    writeln('Process-unit not available. Svn-revision in fpmake executable might be out-of-date.');
{$endif HAS_UNIT_PROCESS}

    Data2IncBin := AddProgramExtension('data2inc',Defaults.BuildOS);
    p.Commands.AddCommand(caBeforeCompile, Data2IncBin, '-b -s ' + P.Directory + 'fpcmake.ini ' + P.Directory + 'fpcmake.inc fpcmakeini','fpcmake.inc','fpcmake.ini');
    T:=P.Targets.AddUnit('fpcmmain.pp');
    T.install:=false;
    T.ResourceStrings:=true;
    P.Targets.AddUnit('fpcmdic.pp').install:=false;
    P.Targets.AddUnit('fpcmwr.pp').install:=false;
    P.Targets.AddUnit('fpcmpkg.pp').install:=false;

    // P.Sources.AddSrc('fpcmake.ini');
    P.Sources.AddSrc('fpcmake.inc');
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_fpcm('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




