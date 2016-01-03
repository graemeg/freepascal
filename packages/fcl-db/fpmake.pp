{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif ALLPACKAGES}

procedure add_fcl_db(const ADirectory: string);

const
  ParadoxOSes         = [beos,haiku,linux,freebsd,netbsd,openbsd,win32,dragonfly];
  DatadictOSes        = [aix,beos,darwin,haiku,linux,freebsd,win32,win64,wince,android,dragonfly];
  SqldbConnectionOSes = [aix,beos,haiku,linux,freebsd,darwin,iphonesim,netbsd,openbsd,solaris,win32,win64,wince,android,dragonfly];
  SqliteOSes          = [aix,beos,haiku,linux,freebsd,darwin,iphonesim,netbsd,openbsd,solaris,win32,win64,wince,android,dragonfly];
  DBaseOSes           = [aix,beos,haiku,linux,freebsd,darwin,iphonesim,netbsd,openbsd,solaris,win32,win64,wince,android,os2,dragonfly];
  MSSQLOSes           = [beos,haiku,linux,freebsd,netbsd,openbsd,solaris,win32,win64,android,dragonfly];
  SqldbWithoutOracleOSes   = [win64];

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Var
  P : TPackage;
  T : TTarget;
<<<<<<< HEAD
<<<<<<< HEAD

begin
  With Installer do
    begin
    P:=AddPackage('fcl-db');
    P.ShortName:='fcld';

    P.Author := '<various>';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Database library of Free Component Libraries(FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16];

    P.Directory:=ADirectory;
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.SourcePath.Add('src/base');
    P.SourcePath.Add('src/paradox', ParadoxOSes);
    P.SourcePath.Add('src/sqldb');
    P.SourcePath.Add('src/sqldb/postgres', SqldbConnectionOSes);
    P.SourcePath.Add('src/sqldb/sqlite', SqldbConnectionOSes);
    P.SourcePath.Add('src/sqldb/interbase', SqldbConnectionOSes);
    P.SourcePath.Add('src/sqldb/mysql', SqldbConnectionOSes);
    P.SourcePath.Add('src/sqldb/odbc', SqldbConnectionOSes);
    P.SourcePath.Add('src/sqldb/examples', SqldbConnectionOSes);
    P.SourcePath.Add('src/sqldb/oracle', SqldbConnectionOSes-SqldbWithoutOracleOSes);
    P.SourcePath.Add('src/sqldb/mssql', MSSQLOSes);
    P.SourcePath.Add('src/sdf');
    P.SourcePath.Add('src/json');
    P.SourcePath.Add('src/datadict', DatadictOSes);
    P.SourcePath.Add('src/memds');
    P.SourcePath.Add('src/codegen', DatadictOSes);
    P.SourcePath.Add('src/export', DatadictOSes);
    P.SourcePath.Add('src/sqlite', SqliteOSes);
    P.SourcePath.Add('src/dbase');
    P.IncludePath.Add('src/base');
    P.IncludePath.Add('src/sqldb');
    P.IncludePath.Add('src/sqldb/postgres', SqldbConnectionOSes);
    P.IncludePath.Add('src/sqldb/mysql', SqldbConnectionOSes);
    P.IncludePath.Add('src/sdf');
    P.IncludePath.Add('src/memds');
    P.IncludePath.Add('src/sqlite',SqliteOSes);
    P.IncludePath.Add('src/dbase');
    P.SourcePath.Add('src/sql');

    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('rtl-objpas');
    P.Dependencies.Add('rtl-extra'); // clocale
    P.Dependencies.Add('ibase', SqldbConnectionOSes);
    P.Dependencies.Add('mysql', SqldbConnectionOSes);
    P.Dependencies.Add('odbc', SqldbConnectionOSes);
    P.Dependencies.Add('oracle', SqldbConnectionOSes-SqldbWithoutOracleOSes);
    P.Dependencies.Add('postgres', SqldbConnectionOSes);
    P.Dependencies.Add('sqlite', SqldbConnectionOSes+SqliteOSes);
    P.Dependencies.Add('dblib', MSSQLOSes);
    P.Dependencies.Add('pxlib',ParadoxOSes);
    P.Dependencies.Add('fcl-json');

<<<<<<< HEAD
<<<<<<< HEAD
    P.Options.Add('-S2h');

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('ibase');
    P.Dependencies.Add('mysql');
    P.Dependencies.Add('odbc');
    P.Dependencies.Add('oracle');
    P.Dependencies.Add('postgres');
    P.Dependencies.Add('sqlite');
    P.Dependencies.Add('pxlib');

    P.Options.Add('-S2h');

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('ibase');
    P.Dependencies.Add('mysql');
    P.Dependencies.Add('odbc');
    P.Dependencies.Add('oracle');
    P.Dependencies.Add('postgres');
    P.Dependencies.Add('sqlite');
    P.Dependencies.Add('pxlib');

    P.Options.Add('-S2h');

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    // base
=======
=======
>>>>>>> origin/fixes_2_2
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('fcl-db');
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-db';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('bufdataset.pas');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('bufdataset_parser');
          AddUnit('dbconst');
        end;
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('csvdataset.pp');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T:=P.Targets.AddUnit('bufdataset_parser.pp');
>>>>>>> graemeg/cpstrnew
      with T.Dependencies do
        begin
        AddUnit('db');
        AddUnit('sqldb');
        AddUnit('bufdataset');
        end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('bufdataset_parser.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbf_prscore');
          AddUnit('dbf_prsdef');
          AddUnit('dbconst');
        end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('bufdataset_parser.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbf_prscore');
          AddUnit('dbf_prsdef');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('customsqliteds.pas');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbconst');
        end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('db.pas');
      with T.Dependencies do
        begin
          AddInclude('dataset.inc');
          AddInclude('fields.inc');
          AddInclude('datasource.inc');
          AddInclude('database.inc');
          AddInclude('dsparams.inc');
          AddUnit('dbconst');
        end;
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('dbcoll.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings:=true;


    T:=P.Targets.AddUnit('dbconst.pas');
    T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('sqlscript.pp');
    T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('dbwhtml.pp');
    with T.Dependencies do
      begin
        AddUnit('db');
        AddUnit('dbconst');
      end;

    T:=P.Targets.AddUnit('xmldatapacketreader.pp');
    T.ResourceStrings:=true;
    with T.Dependencies do
      begin
        AddUnit('bufdataset');
        AddUnit('db');
      end;

    // dbase
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf.pas', DBaseOSes);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T:=P.Targets.AddUnit('dbf.pas');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('dbconst.pas');
    T:=P.Targets.AddUnit('dbf.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbconst.pas');
    T:=P.Targets.AddUnit('dbf.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('db');
          AddUnit('dbf_common');
          AddUnit('dbf_dbffile');
          AddUnit('dbf_parser');
          AddUnit('dbf_prsdef');
          AddUnit('dbf_cursor');
          AddUnit('dbf_fields');
          AddUnit('dbf_pgfile');
          AddUnit('dbf_idxfile');
          AddUnit('dbf_wtil');
          AddUnit('dbf_idxcur');
          AddUnit('dbf_memo');
          AddUnit('dbf_str');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_collate.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_collate.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_collate.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_lang');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_common.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_common.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_common.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('db');
          AddUnit('dbf_wtil');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_cursor.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_cursor.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_cursor.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_pgfile');
          AddUnit('dbf_common');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_dbffile.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_dbffile.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_dbffile.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddInclude('dbf_struct.inc');
          AddUnit('dbf_wtil');
          AddUnit('db');
          AddUnit('dbf_common');
          AddUnit('dbf_cursor');
          AddUnit('dbf_pgfile');
          AddUnit('dbf_fields');
          AddUnit('dbf_memo');
          AddUnit('dbf_idxfile');
          AddUnit('dbf_str');
          AddUnit('dbf_lang');
          AddUnit('dbf_prssupp');
          AddUnit('dbf_prsdef');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_fields.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_fields.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_fields.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddInclude('dbf_struct.inc');
          AddUnit('db');
          AddUnit('dbf_common');
          AddUnit('dbf_str');
          AddUnit('dbf_dbffile');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_idxcur.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_idxcur.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_idxcur.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_cursor');
          AddUnit('dbf_idxfile');
          AddUnit('dbf_prsdef');
          AddUnit('dbf_wtil');
          AddUnit('dbf_common');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_idxfile.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_idxfile.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_idxfile.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_wtil');
          AddUnit('db');
          AddUnit('dbf_pgfile');
          AddUnit('dbf_parser');
          AddUnit('dbf_prsdef');
          AddUnit('dbf_cursor');
          AddUnit('dbf_collate');
          AddUnit('dbf_common');
          AddUnit('dbf_dbffile');
          AddUnit('dbf_fields');
          AddUnit('dbf_str');
          AddUnit('dbf_prssupp');
          AddUnit('dbf_prscore');
          AddUnit('dbf_lang');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_lang.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_lang.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_lang.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_wtil');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_memo.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_memo.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_memo.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_pgfile');
          AddUnit('dbf_common');
          AddUnit('dbf_dbffile');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_parser.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_parser.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_parser.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_wtil');
          AddUnit('db');
          AddUnit('dbf_prscore');
          AddUnit('dbf_common');
          AddUnit('dbf_fields');
          AddUnit('dbf_prsdef');
          AddUnit('dbf_prssupp');
          AddUnit('dbf');
          AddUnit('dbf_dbffile');
          AddUnit('dbf_str');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_pgfile.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_pgfile.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_pgfile.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_common');
          AddUnit('dbf_wtil');
          AddUnit('dbf_str');
        end;
    T:=P.Targets.AddUnit('dbf_prscore.pas');
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('db');
          AddUnit('dbf_prssupp');
          AddUnit('dbf_prsdef');
        end;
    T:=P.Targets.AddUnit('dbf_prsdef.pas');
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('db');
          AddUnit('dbf_prssupp');
        end;
    T:=P.Targets.AddUnit('dbf_prssupp.pas');
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddInclude('getstrfromint.inc');
          AddInclude('getstrfromint.inc');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_str.pas', DBaseOSes);
=======
    T:=P.Targets.AddUnit('dbf_str.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('dbf_str.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddInclude('dbf_str.inc');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('dbf_wtil.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('dbf_wnix.inc', AllOSes-AllWindowsOSes-[os2]);
          AddInclude('dbf_wos2.inc', [os2]);
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpcgcreatedbf.pp', DatadictOSes);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
        end;
>>>>>>> graemeg/cpstrnew
=======
        end;
>>>>>>> origin/cpstrnew
    T:=P.Targets.AddUnit('fpcgcreatedbf.pp');
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('dbf_wtil.pas');
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
        end;
    T:=P.Targets.AddUnit('dbwhtml.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('whtml');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('fpcgcreatedbf.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fpddcodegen');
          AddUnit('db');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpcgdbcoll.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpcgdbcoll.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpcgdbcoll.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpddcodegen');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpcgsqlconst.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpcgsqlconst.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpcgsqlconst.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fpddcodegen');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpcgtiopf.pp', DatadictOSes);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T:=P.Targets.AddUnit('fpcgtiopf.pp');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('fpcgtiopf.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpcgtiopf.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpddcodegen');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpcsvexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpcsvexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('fpcsvexport.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpcsvexport.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpdatadict.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpdatadict.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
        end;
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpdbexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpdbexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
        end;
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpdbfexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpdbfexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('fpdatadict.pp');
      with T.Dependencies do
        begin
          AddUnit('inicol');
          AddUnit('inifiles');
          AddUnit('contnrs');
          AddUnit('db');
        end;
    T:=P.Targets.AddUnit('fpdbexport.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('streamio');
        end;
    T:=P.Targets.AddUnit('fpdbfexport.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbf');
          AddUnit('fpdbexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddpopcode.pp', DatadictOSes);
    T.ResourceStrings:=true;
    T.Dependencies.AddUnit('fpdatadict');

    T:=P.Targets.AddUnit('fpdddiff.pp', DatadictOSes);
    T.ResourceStrings:=true;
    T.Dependencies.AddUnit('fpdatadict');

    T:=P.Targets.AddUnit('fpddcodegen.pp', DatadictOSes);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T:=P.Targets.AddUnit('fpddpopcode.pp');
    T.ResourceStrings:=true;
    T.Dependencies.AddUnit('fpdatadict');

    T:=P.Targets.AddUnit('fpdddiff.pp');
    T.ResourceStrings:=true;
    T.Dependencies.AddUnit('fpdatadict');

    T:=P.Targets.AddUnit('fpddcodegen.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('fpddcodegen.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddcodegen.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdatadict');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpdddbf.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpdddbf.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpdddbf.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbf');
          AddUnit('fpdatadict');
          AddUnit('dbf_idxfile');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddfb.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpddfb.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddfb.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('ibconnection');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddmysql40.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpddmysql40.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddmysql40.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql40conn');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddmysql41.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpddmysql41.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddmysql41.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql41conn');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddmysql50.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpddmysql50.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddmysql50.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql50conn');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddmysql51.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql51conn');
        end;
    T:=P.Targets.AddUnit('fpddmysql55.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql55conn');
        end;
    T:=P.Targets.AddUnit('fpddodbc.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpddodbc.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddodbc.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('odbcconn');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddoracle.pp', DatadictOSes-SqldbWithoutOracleOSes);
=======
    T:=P.Targets.AddUnit('fpddoracle.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddoracle.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('oracleconnection');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddpq.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpddpq.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddpq.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('pqconnection');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddmssql.pp', DatadictOSes*MSSQLOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mssqlconn');
        end;
    T:=P.Targets.AddUnit('fpddregstd.pp', DatadictOSes-SqldbWithoutOracleOSes);
=======
    T:=P.Targets.AddUnit('fpddregstd.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddregstd.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fpdatadict');
          AddUnit('fpdddbf');
          AddUnit('fpddfb');
          AddUnit('fpddpq');
          AddUnit('fpddoracle');
          AddUnit('fpddsqlite3');
          AddUnit('fpddmysql40');
          AddUnit('fpddmysql41');
          AddUnit('fpddmysql50');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('fpddmssql');
          AddUnit('fpddodbc');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('customsqliteds.pas', SqliteOSes);
=======
    T:=P.Targets.AddUnit('customsqliteds.pas');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('customsqliteds.pas');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('customsqliteds.pas');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('customsqliteds.pas');
>>>>>>> origin/cpstrnew
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbconst');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    T:=P.Targets.AddUnit('fpddsqldb.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpddsqldb.pp');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('fpddsqldb.pp');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('fpddsqldb.pp');
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('fpddsqldb.pp');
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
          AddUnit('fpddodbc');
        end;
    T:=P.Targets.AddUnit('fpddsqldb.pp');
>>>>>>> graemeg/fixes_2_2
=======
          AddUnit('fpddodbc');
        end;
    T:=P.Targets.AddUnit('fpddsqldb.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('sqldb');
          AddUnit('fpdatadict');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpddsqlite3.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpddsqlite3.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpddsqlite3.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('sqlite3conn');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpfixedexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpfixedexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('fpfixedexport.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpfixedexport.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fprtfexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fprtfexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('fprtfexport.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fprtfexport.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpsimplejsonexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpsimplejsonexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('fpsimplejsonexport.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpsimplejsonexport.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpsimplexmlexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpsimplexmlexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('fpsimplexmlexport.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpsimplexmlexport.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpsqlexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpsqlexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('fpsqlexport.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpsqlexport.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpstdexports.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fpstdexports.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fpstdexports.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fpdbexport');
          AddUnit('fpcsvexport');
          AddUnit('fpfixedexport');
          AddUnit('fpsimplexmlexport');
          AddUnit('fpsimplejsonexport');
          AddUnit('fpsqlexport');
          AddUnit('fptexexport');
          AddUnit('fprtfexport');
          AddUnit('fpdbfexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fptexexport.pp', DatadictOSes);
=======
    T:=P.Targets.AddUnit('fptexexport.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('fpxmlxsdexport.pp', DatadictOSes);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('fptexexport.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('fptexexport.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('ibconnection.pp', SqldbConnectionOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
<<<<<<< HEAD
        end;
    T:=P.Targets.AddUnit('fbadmin.pp', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('ibconnection');
        end;
    T:=P.Targets.AddUnit('fbeventmonitor.pp', SqldbConnectionOSes);
=======
    T:=P.Targets.AddUnit('ibconnection.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('ibconnection.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('ibconnection');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        end;
    T:=P.Targets.AddUnit('memds.pp');
    T.ResourceStrings:=true;
=======
          AddUnit('ibase60dyn');
        end;
    T:=P.Targets.AddUnit('memds.pp');
>>>>>>> graemeg/fixes_2_2
=======
          AddUnit('ibase60dyn');
        end;
    T:=P.Targets.AddUnit('memds.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    T:=P.Targets.AddUnit('mysql40conn.pas', SqldbConnectionOSes);
=======
    T:=P.Targets.AddUnit('mysql40conn.pas');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('mysql40conn.pas');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('mysql40conn.pas');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('dbconst');
        end;
<<<<<<< HEAD
    T:=P.Targets.AddUnit('mysql41conn.pas', SqldbConnectionOSes);
=======
    T:=P.Targets.AddUnit('mysql41conn.pas');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('mysql40dyn');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('mysql41conn.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('dbconst');
        end;
<<<<<<< HEAD

    T:=P.Targets.AddUnit('mysql50conn.pas', SqldbConnectionOSes);
=======
    T:=P.Targets.AddUnit('mysql4conn.pas');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('mysql41dyn');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('mysql4conn.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('dbconst');
        end;
<<<<<<< HEAD

    T:=P.Targets.AddUnit('mysql51conn.pas', SqldbConnectionOSes);
=======
    T:=P.Targets.AddUnit('mysql50conn.pas');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('mysql40dyn');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('mysql50conn.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
<<<<<<< HEAD
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('mysql55conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('mysql56conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;
<<<<<<< HEAD

    T:=P.Targets.AddUnit('mysql57conn.pas', SqldbConnectionOSes);
=======
    T:=P.Targets.AddUnit('oracleconnection.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
<<<<<<< HEAD
          AddUnit('dbconst');
        end;
<<<<<<< HEAD

    T:=P.Targets.AddUnit('odbcconn.pas', SqldbConnectionOSes);
=======
    T.ResourceStrings:=true;
>>>>>>> graemeg/cpstrnew
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('bufdataset');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('oracleconnection.pp', SqldbConnectionOSes-SqldbWithoutOracleOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('paradox.pp',ParadoxOSes);
=======
    T:=P.Targets.AddUnit('paradox.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('bufdataset_parser');
        end;
<<<<<<< HEAD
    T:=P.Targets.AddUnit('pqconnection.pp', SqldbConnectionOSes);
=======
    T:=P.Targets.AddUnit('pqconnection.pp');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
        end;
    T:=P.Targets.AddUnit('pqeventmonitor.pp', SqldbConnectionOSes);
=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('mysql50dyn');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('odbcconn.pas');
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('odbcsqldyn');
          AddUnit('bufdataset');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('oracleconnection.pp');
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('ocidyn');
          AddUnit('oratypes');
        end;
    T:=P.Targets.AddUnit('paradox.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('pxlib');
          AddUnit('bufdataset_parser');
        end;
    T:=P.Targets.AddUnit('pqconnection.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('pqconnection');
        end;
    T:=P.Targets.AddUnit('mssqlconn.pp', MSSQLOSes);
    with T.Dependencies do
      begin
        AddUnit('sqldb');
        AddUnit('db');
        AddUnit('dbconst');
        AddUnit('bufdataset');
      end;
=======
          AddUnit('postgres3dyn');
        end;
>>>>>>> graemeg/fixes_2_2
=======
          AddUnit('postgres3dyn');
        end;
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('sdfdata.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
        end;
    T:=P.Targets.AddUnit('sqldb.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('bufdataset');
          AddUnit('dbconst');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('sqlscript');
        end;
    T:=P.Targets.AddUnit('sqldblib.pp');
      with T.Dependencies do
        begin
          AddUnit('sqldb');
        end;
    T:=P.Targets.AddUnit('sqlite3conn.pp', SqldbConnectionOSes);
=======
        end;
    T:=P.Targets.AddUnit('sqlite3conn.pp');
>>>>>>> graemeg/fixes_2_2
=======
        end;
    T:=P.Targets.AddUnit('sqlite3conn.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('bufdataset');
          AddUnit('sqldb');
<<<<<<< HEAD
<<<<<<< HEAD
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('sqlite3ds.pas', SqliteOSes);
      with T.Dependencies do
        begin
          AddUnit('customsqliteds');
          AddUnit('db');
        end;
    T:=P.Targets.AddUnit('sqliteds.pas', SqliteOSes);
      with T.Dependencies do
        begin
          AddUnit('customsqliteds');
          AddUnit('db');
        end;
    T:=P.Targets.AddUnit('sqlite3backup.pas', SqldbConnectionOSes);
      with T.Dependencies do
        begin
          AddUnit('sqlite3conn');
        end;
    // SQL
    T:=P.Targets.AddUnit('fpsqltree.pp');
    T:=P.Targets.AddUnit('fpsqlscanner.pp');
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('fpsqlparser.pas');
      with T.Dependencies do
        begin
          AddUnit('fpsqltree');
          AddUnit('fpsqlscanner');
        end;
    T.ResourceStrings := True;

    T:=P.Targets.AddUnit('fpjsondataset.pp');

    P.ExamplePath.Add('tests');
    T:=P.Targets.AddExampleProgram('dbftoolsunit.pas', DBaseOSes);
    T:=P.Targets.AddExampleProgram('dbtestframework.pas');
    T:=P.Targets.AddExampleProgram('memdstoolsunit.pas');
    T:=P.Targets.AddExampleProgram('sdfdstoolsunit.pas');
    T:=P.Targets.AddExampleProgram('sqldbtoolsunit.pas');
    T:=P.Targets.AddExampleProgram('testbasics.pas');
    T:=P.Targets.AddExampleProgram('testdatasources.pas');
    T:=P.Targets.AddExampleProgram('testdbbasics.pas');
    T:=P.Targets.AddExampleProgram('testdddiff.pp');
    T:=P.Targets.AddExampleProgram('testfieldtypes.pas');
    T:=P.Targets.AddExampleProgram('testsqlscript.pas');
    T:=P.Targets.AddExampleProgram('toolsunit.pas');
    // database.ini.txt
    // README.txt

    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_fcl_db('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




=======
=======
>>>>>>> origin/fixes_2_2
          AddUnit('sqlite3dyn');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('sqlite3ds.pas');
      with T.Dependencies do
        begin
          AddUnit('customsqliteds');
          AddUnit('sqlite3');
          AddUnit('db');
        end;
    T:=P.Targets.AddUnit('sqliteds.pas');
      with T.Dependencies do
        begin
          AddUnit('customsqliteds');
          AddUnit('sqlite');
          AddUnit('db');
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
