{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt, member of the
    Free Pascal development team

    Constants used for displaying messages in DB units

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit dbconst;

Interface

Resourcestring
  SActiveDataset           = 'Operation cannot be performed on an active dataset';
  SBadParamFieldType       = 'Bad fieldtype for parameter "%s".';
  SCantSetAutoIncFields    = 'AutoInc Fields are read-only';
<<<<<<< HEAD
<<<<<<< HEAD
  SConnected               = 'Operation cannot be performed on a connected database';
=======
  SConnected               = 'Operation cannot be performed on an connected database';
>>>>>>> graemeg/fixes_2_2
=======
  SConnected               = 'Operation cannot be performed on an connected database';
>>>>>>> origin/fixes_2_2
  SDatasetReadOnly         = 'Dataset is read-only.';
  SDatasetRegistered       = 'Dataset already registered : "%s"';
  SDuplicateFieldName      = 'Duplicate fieldname : "%s"';
  SErrAssTransaction       = 'Cannot assign transaction while old transaction active!';
  SErrColumnNotFound       = 'Column "%s" not found.';
  SErrDatabasenAssigned    = 'Database not assigned!';
  SErrNoDatabaseAvailable  = 'Invalid operation: Not attached to database';
  SErrNoDatabaseName       = 'Database connect string (DatabaseName) not filled in!';
  SErrNoSelectStatement    = 'Cannot open a non-select statement';
  SErrNoStatement          = 'SQL statement not set';
  SErrTransAlreadyActive   = 'Transaction already active';
  SErrTransactionnSet      = 'Transaction not set';
  SErrIndexResultTooLong   = 'Index result for "%s" too long, >100 characters (%d).';
<<<<<<< HEAD
<<<<<<< HEAD
  SErrIndexBasedOnInvField = 'Field "%s" has an invalid field type (%s) to base index on.';
=======
  SErrIndexBasedOnInvField = 'Field "%s" is an invalid field type to base index on.';
>>>>>>> graemeg/fixes_2_2
=======
  SErrIndexBasedOnInvField = 'Field "%s" is an invalid field type to base index on.';
>>>>>>> origin/fixes_2_2
  SErrIndexBasedOnUnkField = 'Index based on unknown field "%s".';
  SErrConnTransactionnSet  = 'Transaction of connection not set';
  SErrNotASQLConnection    = '"%s" is not a TSQLConnection';
  SErrNotASQLQuery         = '"%s" is not a TCustomSQLQuery';
  STransNotActive          = 'Operation cannot be performed on an inactive transaction';
  STransActive             = 'Operation cannot be performed on an active transaction';
  SFieldNotFound           = 'Field not found : "%s"';
  SInactiveDataset         = 'Operation cannot be performed on an inactive dataset';
  SInvalidDisplayValues    = '"%s" are not valid boolean displayvalues';
  SInvalidFieldKind        = '%s : invalid field kind : ';
  SInvalidBookmark         = 'Invalid bookmark';
  SInvalidFieldSize        = 'Invalid field size : %d';
  SInvalidTypeConversion   = 'Invalid type conversion to %s in field %s';
  SNeedField               = 'Field %s is required, but not supplied.';
  SNeedFieldName           = 'Field needs a name';
  SNoDataset               = 'No dataset asssigned for field : "%s"';
  SNoDatasetRegistered     = 'No such dataset registered : "%s"';
  SNoDatasets              = 'No datasets are attached to the database';
  SNoSuchRecord            = 'Could not find the requested record.';
  SNoTransactionRegistered = 'No such transaction registered : "%s"';
  SNoTransactions          = 'No transactions are attached to the database';
  SNotABoolean             = '"%s" is not a valid boolean';
  SNotAFloat               = '"%s" is not a valid float';
  SNotAninteger            = '"%s" is not a valid integer';
  SNotConnected            = 'Operation cannot be performed on an disconnected database';
  SNotEditing              = 'Operation not allowed, dataset "%s" is not in an edit or insert state.';
  SParameterNotFound       = 'Parameter "%s" not found';
  SRangeError              = '%f is not between %f and %f for %s';
  SReadOnlyField           = 'Field %s cannot be modified, it is read-only.';
  STransactionRegistered   = 'Transaction already registered : "%s"';
  SUniDirectional          = 'Operation cannot be performed on an unidirectional dataset';
  SUnknownField            = 'No field named "%s" was found in dataset "%s"';
  SUnknownFieldType        = 'Unknown field type : %s';
  SUnknownParamFieldType   = 'Unknown fieldtype for parameter "%s".';
  SMetadataUnavailable     = 'The metadata is not available for this type of database.';
  SDeletedRecord           = 'The record is deleted.';
  SIndexNotFound           = 'Index ''%s'' not found';
  SParameterCountIncorrect = 'The number of parameters is incorrect.';
  SUnsupportedParameter    = 'Parameters of the type ''%s'' are not (yet) supported.';
  SFieldValueError         = 'Invalid value for field ''%s''';
  SInvalidCalcType         = 'Field ''%s'' cannot be a calculated or lookup field';
  SDuplicateName           = 'Duplicate name ''%s'' in %s';
  SNoParseSQL              = '%s is only possible if ParseSQL is True';
  SLookupInfoError         = 'Lookup information for field ''%s'' is incomplete';
  SUnsupportedFieldType    = 'Fieldtype %s is not supported';
  SInvPacketRecordsValue   = 'PacketRecords has to be larger then 0';
<<<<<<< HEAD
<<<<<<< HEAD
  SInvPacketRecordsValueFieldNames = 'PacketRecords must be -1 if IndexFieldNames is set';
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SInvalidSearchFieldType  = 'Searching in fields of type %s is not supported';
  SDatasetEmpty            = 'The dataset is empty';
  SFieldIsNull             = 'The field is null';
  SOnUpdateError           = 'An error occured while applying the updates in a record: %s';
  SApplyRecNotSupported    = 'Applying updates is not supported by this TDataset descendent';
  SNoWhereFields           = 'No %s query specified and failed to generate one. (No fields for inclusion in where statement found)';
  SNoUpdateFields          = 'No %s query specified and failed to generate one. (No fields found for insert- or update-statement found)';
  SNotSupported            = 'Operation is not supported by this type of database';
  SDBCreateDropFailed      = 'Creation or dropping of database failed';
  SMaxIndexes              = 'The maximum amount of indexes is reached';
  SMinIndexes              = 'The minimum amount of indexes is 1';
  STooManyFields           = 'More fields specified then really exist';
// These are added for Delphi-compatilility, but not used by the fcl:
  SFieldIndexError         = 'Field index out of range';
  SIndexFieldMissing       = 'Cannot access index field ''%s''';
  SNoFieldIndexes          = 'No index currently active';
  SNotIndexField           = 'Field ''%s'' is not indexed and cannot be modified';
<<<<<<< HEAD
<<<<<<< HEAD
  SErrUnknownConnectorType = 'Unknown connector type: "%s"';
=======
  SErrUnknownConnectorType = 'Unknown connector type';
>>>>>>> graemeg/fixes_2_2
=======
  SErrUnknownConnectorType = 'Unknown connector type';
>>>>>>> origin/fixes_2_2
  SNoIndexFieldNameGiven   = 'There are no fields selected to base the index on';
  SStreamNotRecognised     = 'The data-stream format is not recognized';
  SNoReaderClassRegistered = 'There is no TDatapacketReaderClass registered for this kind of data-stream';
  SErrCircularDataSourceReferenceNotAllowed = 'Circular datasource references are not allowed.';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SCommitting               = 'Committing transaction';
  SRollingBack              = 'Rolling back transaction';
  SCommitRetaining          = 'Commit and retaining transaction';
  SRollBackRetaining        = 'Rollback and retaining transaction';
  SErrNoFieldsDefined       = 'Can not create a dataset when there are no fielddefinitions or fields defined';
  SErrApplyUpdBeforeRefresh = 'Must apply updates before refreshing data';
  SErrNoDataset             = 'Missing (compatible) underlying dataset, can not open';
  SErrDisconnectedPacketRecords = 'For disconnected TSQLQuery instances, packetrecords must be -1';
  SErrImplicitNoRollBack      = 'Implicit use of transactions does not allow rollback.';
  SErrNoImplicitTransaction   = 'Connection %s does not allow implicit transactions.';
  SErrImplictTransactionStart = 'Error: attempt to implicitly start a transaction on Connection "%s", transaction "%s".';
  SErrImplicitConnect         = 'Error: attempt to implicitly activate connection "%s".';
  SErrFailedToUpdateRecord    = 'Failed to apply record updates: %d rows updated.';
  SErrRefreshNotSingleton     = 'Refresh SQL resulted in multiple records: %d.';
  SErrRefreshEmptyResult      = 'Refresh SQL resulted in empty result set.';
  SErrNoKeyFieldForRefreshClause = 'No key field found to construct refresh SQL WHERE clause';
  SErrFailedToFetchReturningResult = 'Failed to fetch returning result';
  SLogParamValue              = 'Parameter "%s" value : "%s"';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  SCommitting              = 'Committing transaction';
  SRollingBack             = 'Rolling back transaction';
  SCommitRetaining         = 'Commit and retaining transaction';
  SRollBackRetaining       = 'Rollback and retaining transaction';
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
=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2

Implementation

end.
