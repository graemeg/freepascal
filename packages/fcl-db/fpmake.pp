{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_fcl_db(const ADirectory: string);

const
  ParadoxOSes         = [beos,haiku,linux,freebsd,netbsd,openbsd,win32,dragonfly];
  DatadictOSes        = [aix,beos,darwin,haiku,linux,freebsd,win32,win64,wince,android,dragonfly];
  SqldbConnectionOSes = [aix,beos,haiku,linux,freebsd,darwin,iphonesim,ios,netbsd,openbsd,solaris,win32,win64,wince,android,dragonfly];
  SqliteOSes          = [aix,beos,haiku,linux,freebsd,darwin,iphonesim,ios,netbsd,openbsd,solaris,win32,win64,wince,android,dragonfly];
  DBaseOSes           = [aix,beos,haiku,linux,freebsd,darwin,iphonesim,ios,netbsd,openbsd,solaris,win32,win64,wince,android,os2,dragonfly];
  MSSQLOSes           = [beos,haiku,linux,freebsd,netbsd,openbsd,solaris,win32,win64,android,dragonfly];


Var
  P : TPackage;
  T : TTarget;

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
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Directory:=ADirectory;
    P.Version:='3.3.1';
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
    P.SourcePath.Add('src/sqldb/oracle', SqldbConnectionOSes);
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
    P.Dependencies.Add('oracle', SqldbConnectionOSes);
    P.Dependencies.Add('postgres', SqldbConnectionOSes);
    P.Dependencies.Add('sqlite', SqldbConnectionOSes+SqliteOSes);
    P.Dependencies.Add('dblib', MSSQLOSes);
    P.Dependencies.Add('pxlib',ParadoxOSes);
    P.Dependencies.Add('fcl-json');

    P.Options.Add('-S2h');

    // base
    T:=P.Targets.AddUnit('bufdataset.pas');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('bufdataset_parser');
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('csvdataset.pp');
      with T.Dependencies do
        begin
        AddUnit('db');
        AddUnit('sqldb');
        AddUnit('bufdataset');
        end;

    T:=P.Targets.AddUnit('bufdataset_parser.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbf_prscore');
          AddUnit('dbf_prsdef');
          AddUnit('dbconst');
        end;

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

    T:=P.Targets.AddUnit('dbcoll.pp');
      with T.Dependencies do
        begin
          AddUnit('db');
        end;
    T.ResourceStrings:=true;


    T:=P.Targets.AddUnit('dbconst.pas');
    T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('sqltypes.pp');

    T:=P.Targets.AddUnit('sqlscript.pp');
    T.ResourceStrings:=true;
    
    T:=P.Targets.AddUnit('fieldmap.pp');
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
    T:=P.Targets.AddUnit('dbf.pas', DBaseOSes);
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
    T:=P.Targets.AddUnit('dbf_collate.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_lang');
        end;
    T:=P.Targets.AddUnit('dbf_common.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('db');
          AddUnit('dbf_wtil');
        end;
    T:=P.Targets.AddUnit('dbf_cursor.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_pgfile');
          AddUnit('dbf_common');
        end;
    T:=P.Targets.AddUnit('dbf_dbffile.pas', DBaseOSes);
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
    T:=P.Targets.AddUnit('dbf_fields.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddInclude('dbf_struct.inc');
          AddUnit('db');
          AddUnit('dbf_common');
          AddUnit('dbf_str');
          AddUnit('dbf_dbffile');
        end;
    T:=P.Targets.AddUnit('dbf_idxcur.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_cursor');
          AddUnit('dbf_idxfile');
          AddUnit('dbf_prsdef');
          AddUnit('dbf_wtil');
          AddUnit('dbf_common');
        end;
    T:=P.Targets.AddUnit('dbf_idxfile.pas', DBaseOSes);
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
    T:=P.Targets.AddUnit('dbf_lang.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_wtil');
        end;
    T:=P.Targets.AddUnit('dbf_memo.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddUnit('dbf_pgfile');
          AddUnit('dbf_common');
          AddUnit('dbf_dbffile');
        end;
    T:=P.Targets.AddUnit('dbf_parser.pas', DBaseOSes);
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
    T:=P.Targets.AddUnit('dbf_pgfile.pas', DBaseOSes);
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
    T:=P.Targets.AddUnit('dbf_str.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddInclude('dbf_str.inc');
        end;
    T:=P.Targets.AddUnit('dbf_wtil.pas', DBaseOSes);
      with T.Dependencies do
        begin
          AddInclude('dbf_common.inc');
          AddInclude('dbf_wnix.inc', AllOSes-AllWindowsOSes-[os2]);
          AddInclude('dbf_wos2.inc', [os2]);
        end;
    T:=P.Targets.AddUnit('fpcgcreatedbf.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('fpddcodegen');
          AddUnit('db');
        end;
    T:=P.Targets.AddUnit('fpcgdbcoll.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpddcodegen');
        end;
    T:=P.Targets.AddUnit('fpcgsqlconst.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('fpddcodegen');
        end;
    T.ResourceStrings:=true;    
    T:=P.Targets.AddUnit('fpcgfieldmap.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('fpddcodegen');
        end;
    T:=P.Targets.AddUnit('fpcgtypesafedataset.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('fpddcodegen');
          AddUnit('fpcgfieldmap');
        end;
    T:=P.Targets.AddUnit('fpcgtiopf.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpddcodegen');
        end;
    T:=P.Targets.AddUnit('fpcsvexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('fpdatadict.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
        end;
    T:=P.Targets.AddUnit('fpdbexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
        end;
    T:=P.Targets.AddUnit('fpdbfexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbf');
          AddUnit('fpdbexport');
        end;

    T:=P.Targets.AddUnit('fpddpopcode.pp', DatadictOSes);
    T.ResourceStrings:=true;
    T.Dependencies.AddUnit('fpdatadict');

    T:=P.Targets.AddUnit('fpdddiff.pp', DatadictOSes);
    T.ResourceStrings:=true;
    T.Dependencies.AddUnit('fpdatadict');

    T:=P.Targets.AddUnit('fpddcodegen.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdatadict');
        end;
    T:=P.Targets.AddUnit('fpdddbf.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbf');
          AddUnit('fpdatadict');
          AddUnit('dbf_idxfile');
        end;
    T:=P.Targets.AddUnit('fpddfb.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('ibconnection');
        end;
    T:=P.Targets.AddUnit('fpddmysql40.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql40conn');
        end;
    T:=P.Targets.AddUnit('fpddmysql41.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql41conn');
        end;
    T:=P.Targets.AddUnit('fpddmysql50.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql50conn');
        end;
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
    T:=P.Targets.AddUnit('fpddmysql56.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql56conn');
        end;
    T:=P.Targets.AddUnit('fpddmysql57.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql57conn');
        end;
    T:=P.Targets.AddUnit('fpddmysql80.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mysql80conn');
        end;
    T:=P.Targets.AddUnit('fpddodbc.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('odbcconn');
        end;
    T:=P.Targets.AddUnit('fpddoracle.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('oracleconnection');
        end;
    T:=P.Targets.AddUnit('fpddpq.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('pqconnection');
        end;
    T:=P.Targets.AddUnit('fpddmssql.pp', DatadictOSes*MSSQLOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('mssqlconn');
        end;
    T:=P.Targets.AddUnit('fpddregstd.pp', (DatadictOSes*MSSQLOses));
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
          AddUnit('fpddmssql');
          AddUnit('fpddodbc');
        end;
    T:=P.Targets.AddUnit('customsqliteds.pas', SqliteOSes);
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('fpddsqldb.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('sqldb');
          AddUnit('sqltypes');
          AddUnit('fpdatadict');
        end;
    T:=P.Targets.AddUnit('fpddsqlite3.pp', DatadictOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('fpdatadict');
          AddUnit('fpddsqldb');
          AddUnit('sqlite3conn');
        end;
    T:=P.Targets.AddUnit('fpfixedexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('fprtfexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('fpsimplejsonexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('fpsimplexmlexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('fpsqlexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('fpstdexports.pp', DatadictOSes);
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
    T:=P.Targets.AddUnit('fptexexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('fpxmlxsdexport.pp', DatadictOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('fpdbexport');
        end;
    T:=P.Targets.AddUnit('ibconnection.pp', SqldbConnectionOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
        end;
    T:=P.Targets.AddUnit('fbadmin.pp', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
          AddUnit('ibconnection');
        end;
    T:=P.Targets.AddUnit('fbeventmonitor.pp', SqldbConnectionOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
          AddUnit('ibconnection');
        end;
    T:=P.Targets.AddUnit('memds.pp');
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
        end;

    T:=P.Targets.AddUnit('mysql40conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('mysql41conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('mysql50conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('mysql51conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
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
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('mysql56conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('mysql57conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('mysql80conn.pas', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('mysqlconn.inc');
          AddUnit('bufdataset');
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;

    T:=P.Targets.AddUnit('odbcconn.pas', SqldbConnectionOSes);
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('bufdataset');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('oracleconnection.pp', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
        end;
    T:=P.Targets.AddUnit('paradox.pp',ParadoxOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('bufdataset_parser');
        end;
    T:=P.Targets.AddUnit('pqconnection.pp', SqldbConnectionOSes);
    T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
        end;
    T:=P.Targets.AddUnit('pqeventmonitor.pp', SqldbConnectionOSes);
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('sqldb');
          AddUnit('db');
          AddUnit('dbconst');
          AddUnit('bufdataset');
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
          AddUnit('sqlscript');
          AddUnit('sqltypes');
        end;
    T:=P.Targets.AddUnit('sqldblib.pp');
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('sqldb');
        end;
    T:=P.Targets.AddUnit('sqlite3conn.pp', SqldbConnectionOSes);
      with T.Dependencies do
        begin
          AddUnit('db');
          AddUnit('bufdataset');
          AddUnit('sqldb');
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
    with T.Dependencies do
      AddUnit('db');
    
    T:=P.Targets.AddUnit('extjsdataset.pp');
    with T.Dependencies do
      AddUnit('fpjsondataset');

    T:=P.Targets.AddUnit('sqldbini.pp');
    with T.Dependencies do
      AddUnit('sqldb');

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
