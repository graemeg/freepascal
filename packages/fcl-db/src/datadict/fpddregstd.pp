{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by Michael Van Canneyt, member of the
    Free Pascal development team

    Standard Data Dictionary Engines registration.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
  This unit has a routine and a component to register standard distributed
  Data dictionary engines in an application. The Component version is meant for
  use in Lazarus: Drop it on a form, set the engines you want to see
  registered, and set active to true. When the form is created a run-time,
  the selected engines will be registered.

  The simple call takes an optional single argument, a set which tells
  the call which engines to register. If none is specified, all formats
  are registered.

}
unit fpddregstd;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpdatadict;

Type
  TDataDictEngine = (teDBF,teFirebird,teOracle,teMySQL40,teMySQL41,teMySQL50,
<<<<<<< HEAD
<<<<<<< HEAD
                       tePostgreSQL,teSQLite3,teODBC, teMSSQL);
=======
                       tePostgreSQL,teSQLite3,teODBC);
>>>>>>> graemeg/fixes_2_2
=======
                       tePostgreSQL,teSQLite3,teODBC);
>>>>>>> origin/fixes_2_2
  TDataDictEngines = set of TDataDictEngine;

Const
  AllStdDDEngines = [teDBF,teFirebird,teOracle,teMySQL40,teMySQL41,teMySQL50,
<<<<<<< HEAD
<<<<<<< HEAD
                     tePostgreSQL,teSQLite3,teODBC,teMSSQL];
=======
                     tePostgreSQL,teSQLite3,teODBC];
>>>>>>> graemeg/fixes_2_2
=======
                     tePostgreSQL,teSQLite3,teODBC];
>>>>>>> origin/fixes_2_2
                     
Type

  { TStandardDDEngines }

  TStandardDDEngines = Class(TComponent)
  Private
    FActive: Boolean;
    FRegistered,
    FEngines: TDataDictEngines;
    procedure SetActive(const AValue: Boolean);
    Procedure DoRegister;
    Procedure DoUnregister;
  Public
    Constructor Create(AOwner : TComponent); override;
    Procedure Loaded; override;
  Published
    Property Active : Boolean Read FActive Write SetActive;
    Property Engines : TDataDictEngines Read FEngines Write FEngines Default AllStdDDEngines;
  end;

Function RegisterStdDDEngines(Engines : TDataDictEngines) : TDataDictEngines; overload;
Function RegisterStdDDEngines : TDataDictEngines; overload;
Function UnRegisterStdDDEngines(Engines : TDataDictEngines) : TDataDictEngines;

implementation

uses
  fpdddbf,
  fpddfb,
  fpddpq,
  fpddOracle,
  fpddsqlite3,
  fpddmysql40,
  fpddmysql41,
  fpddmysql50,
<<<<<<< HEAD
<<<<<<< HEAD
  fpddmssql,
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  fpddodbc;
  
Const

  StdEngineClasses : Array [TDataDictEngine] of TFPDDEngineClass
                   = (TDBFDDEngine, TSQLDBFBDDEngine, TSQLDBOracleDDEngine,
                      TSQLDBMySql40DDEngine, TSQLDBMySql41DDEngine ,
                      TSQLDBMySql5DDEngine, TSQLDBPostGreSQLDDEngine,
<<<<<<< HEAD
<<<<<<< HEAD
                      TSQLDBSQLite3DDEngine,TSQLDBODBCDDEngine, TSQLDBMSSQLDDEngine);
=======
                      TSQLDBSQLite3DDEngine,TSQLDBODBCDDEngine);
>>>>>>> graemeg/fixes_2_2
=======
                      TSQLDBSQLite3DDEngine,TSQLDBODBCDDEngine);
>>>>>>> origin/fixes_2_2

  StdEngineRegs : Array [TDataDictEngine] of procedure
                = (@InitDBFImporter, @RegisterFBDDEngine, @RegisterOracleDDEngine,
                  @RegisterMySQL40DDEngine, @RegisterMySQL41DDEngine,
                  @RegisterMySQL50DDEngine, @RegisterPostgreSQLDDengine,
<<<<<<< HEAD
<<<<<<< HEAD
                  @RegisterSQLite3DDEngine, @RegisterODBCDDengine,@RegisterMSSQLDDEngine);
=======
                  @RegisterSQLite3DDEngine, @RegisterODBCDDengine);
>>>>>>> graemeg/fixes_2_2
=======
                  @RegisterSQLite3DDEngine, @RegisterODBCDDengine);
>>>>>>> origin/fixes_2_2

  StdEngineUnRegs : Array [TDataDictEngine] of procedure
                = (@DoneDBFImporter, @UnRegisterFBDDEngine, @UnRegisterOracleDDEngine,
                  @UnRegisterMySQL40DDEngine, @UnRegisterMySQL41DDEngine,
                  @UnRegisterMySQL50DDEngine, @UnRegisterPostgreSQLDDengine,
<<<<<<< HEAD
<<<<<<< HEAD
                  @UnRegisterSQLite3DDEngine, @UnRegisterODBCDDengine,@UnRegisterMSSQLDDEngine);
=======
                  @UnRegisterSQLite3DDEngine, @UnRegisterODBCDDengine);
>>>>>>> graemeg/fixes_2_2
=======
                  @UnRegisterSQLite3DDEngine, @UnRegisterODBCDDengine);
>>>>>>> origin/fixes_2_2
                  
function RegisterStdDDEngines(Engines: TDataDictEngines): TDataDictEngines;

Var
  E : TDataDictEngine;

begin
  Result:=[];
  For E:=Low(TDataDictEngine) to High(TDataDictEngine) do
    If (E in Engines) and (Not IsDictionaryEngineRegistered(StdEngineClasses[E])) then
      begin
      StdEngineRegs[E];
      Include(Result,E);
      end;
end;

function RegisterStdDDEngines: TDataDictEngines;
begin
  Result:=RegisterStdDDEngines(AllStdDDEngines);
end;

function UnRegisterStdDDEngines(Engines: TDataDictEngines): TDataDictEngines;

Var
  E : TDataDictEngine;

begin
  Result:=[];
  For E:=Low(TDataDictEngine) to High(TDataDictEngine) do
    If (E in Engines) and IsDictionaryEngineRegistered(StdEngineClasses[E]) then
      begin
      StdEngineUnRegs[E];
      Include(Result,E);
      end;

end;

{ TStandardDDEngines }

procedure TStandardDDEngines.SetActive(const AValue: Boolean);
begin
  if FActive=AValue then
    exit;
  FActive:=AValue;
  If Not (csLoading in ComponentState) then
    If Active then
      DoRegister
    else
      DoUnregister;
end;

procedure TStandardDDEngines.Loaded;
begin
  If FActive then
    DoRegister;
end;

procedure TStandardDDEngines.DoRegister;
begin
  FRegistered:=RegisterSTDDDengines(FEngines);
end;

procedure TStandardDDEngines.DoUnregister;
begin
  UnRegisterSTDDDengines(FRegistered);
end;

constructor TStandardDDEngines.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
end;

end.

