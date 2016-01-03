This directory contains a framework to test several TDataset descendents.
A lot of these tests are only applicable for SQL databases, but there are several tests that also apply to other objects, such as TBufDataset.

The framework is based on the fpcunit unit test system. The tests can be
executed using any fpcunit testrunner. For example the console and graphical
fpcunit test runners from Lazarus.
Simply add the test* units in this directory to the uses statement of the
test runner and all tests will get registered and executed.

A simple test runner (dbtestframework.pas) which generates XML output is
included in this directory. 
Additionally, a GUI Lazarus unit (dbtestframework_gui.lpr) is included for convenience.

DBTestframework architecture
============================
To test a TDataset descendent, a 'connector' is needed to test the database.
To add a new connector, create a new *toolsunit.pas file, then add it to 
the uses section in 'dbtestframework.pas' and 'dbtestframework_gui.lpr'.
Several connectors are available in the '*toolsunit.pas' files.

<<<<<<< HEAD
The connector must inherit from TDBConnector in toolsunit.pas.
The connector implements two different kinds of datasets: 
- a dataset with as many different kinds of fields as possible (see the *FieldDataSets subroutines).
- a dataset with only a few fields (ID and NAME), but a lot (well, MaxDataset) of different records (see the *NDataSets subroutines)

CreateNDatasets and CreateFieldDataset should be implemented to set up data stores (e.g. database tables) and fill these stores with test data for the respective datasets.
The corresponding Drop*Dataset procedures must drop the tables/delete the data.

GetNDataset and GetFieldsDataset should return the relevant dataset in closed state so the tests can open them and work with them.
They call InternalGetNDataset and InternalGetFieldDataset which should be implemented in all descendents and returns the relevant dataset, closed, with all data.
=======
Which connector is currently used is dependent on the 'database.ini'
configuration file. Also some settings which are connector-dependent can be set
in that file. See 'database.ini.txt' for an example.
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

Toolsunit.pas defines some variables for use, e.g.
- testValuesCount is the number of records/test values in the FieldDataset dataset
- MaxDataset is the same for NDataset.
See e.g. the SQLDBToolsUnit for the implementation for SQL databases.
=======
>>>>>>> origin/fixes_2_2

Tests
=====
In your test units, you can specify that you only want it to run for certain groups/connectors.
E.g. this example to only run for Bufdataset tests:
  TTestSpecificTBufDataset = class(TTestCase)
  ...
initialization  
  if uppercase(dbconnectorname)='BUFDATASET' then
    begin
    RegisterTestDecorator(TDBBasicsTestSetup, TTestSpecificTBufDataset);
    end;
	
In your individual tests, you can indicate you want to run tests only in certain cases, e.g. for certain SQLDB databases:
  if not(SQLConnType in [interbase]) then Ignore(STestNotApplicable);
  
Setting up your database
========================
Some tests are file based (e.g. those for bufdataset); others by their nature need databases (e.g. a Firebird SQLDB test).
File-based tests will generally write to the current/test directory, a subdirectory or a temp file.

For SQLDB database servers, please make sure you have a username/password and a database set up that the test suite can use and abuse.
The database can be empty: the test suite will create and delete tables etc. in this database as needed.

Specifying databases, connector names
=====================================
Which connector is currently used is determined by the 'database.ini'
configuration file. Also some settings which are connector-dependent can be set
in that file. See 'database.ini.txt' for a template/example.

The connector names to be used are derived from the connector classes.

For example, the SQL RDBMS connector defined in sqldbtoolsunit:
- it has this class definition
TSQLDBConnector = class(TDBConnector)
- its name in database.ini is sqldb
- incidentally, in databases.ini, more parameters such as
connectorparams=postgresql (which specify db type) are needed
The parameters used depend on the connector type (sql,...)

If you specify the wrong (or no) name (or don't have database.ini), you will get an exception in your test runner:
Unknown db connector specified

Joost van der Sluis (30-12-2006), 
amended by Reinier Olislagers (2012)
