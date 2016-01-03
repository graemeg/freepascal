
{$MODE objfpc}
{$MACRO on}

interface

uses ctypes,
{$IFDEF LinkDynamically}
 sysutils,
{$ENDIF}
    Dynlibs, // always for sharedsuffix
    oratypes;

{
  Automatically converted by H2Pas 1.0.0 from oci.h
  The following command line parameters were used:
    -p
    -D
    -l
    jojo.so
    oci.h
}

{$IFDEF Unix}
  {$DEFINE extdecl:=cdecl}
  const
    ocilib = 'libociei.'+sharedsuffix;
{$ENDIF}
{$IFDEF Windows}
  {$DEFINE extdecl:=stdcall}
  const
    ocilib = 'oci.dll';
{$ENDIF}

{$PACKRECORDS C}


  {
   * $Header: oci.h 11-jan-2005.15:11:45 aahluwal Exp $
    }
  { Copyright (c) 1995, 2005, Oracle. All rights reserved.   }
  { 
     NAME 
       oci.h - V8 Oracle Call Interface public definitions
  
     DESCRIPTION 
       This file defines all the constants and structures required by a V8
       OCI programmer.
  
     RELATED DOCUMENTS 
       V8 OCI Functional Specification 
       Oracle Call Interface Programmer's Guide Vol 1 and 2
   
     INSPECTION STATUS 
       Inspection date: 
       Inspection status: 
       Estimated increasing cost defects per page: 
       Rule sets: 
   
     ACCEPTANCE REVIEW STATUS 
       Review date: 
       Review status: 
       Reviewers: 
   
     PUBLIC FUNCTION(S) 
       None
  
     PRIVATE FUNCTION(S) 
       None
   
     EXAMPLES 
   
     NOTES 
  
  
     MODIFIED   (MM/DD/YY)
     aahluwal    03/15/05 - [Bug 4235014]:add ASM, Preconnect events 
     ssappara    08/12/04 - Bug3669429 add OCI_ATTR_DESC_SYNBAS
     absaxena    03/24/05 - remove OCI_AQ_RESERVED_5 
     mbastawa    03/01/05 - add OCI_EXEC_RESERVED_5 
     msakayed    02/15/05 - Bug #3147299: Add OCI_ATTR_CURRENT_ERRCOL
     aahluwal    01/11/05 - [Bug 3944589]: add OCI_AUTH_RESERVED_5 
     nikeda      11/15/04 - Add OCIP_IIO 
     rvissapr    11/10/04 - bug 3843644 - isencrypted 
     hohung      11/22/04 - add OCI_BIND_RESERVED_3
     cchui       10/25/04 - add OCI_ATTR_PROXY_CLIENT 
     aahluwal    09/27/04 - add incarnation, reason, cardinality to event handle 
     msakayed    09/14/04 - column encryption support (project id 5578) 
     jacao       08/17/04 - Add OCI_ATTR_DB_CHARSET_ID 
     mhho        08/29/04 - resolve conflicting mode declaration
     sgollapu    05/28/04 - Add OCI_AUTH_RESERVED_3 
     mbastawa    08/05/04 - add OCI_ATTR_RESERVED_21
     ebatbout    07/27/04 - add OCI_ATTR_DIRPATH_RESERVED_9 and move all direct
                            path attributes into a separate area in this file.
     clei        06/29/04 - add OCI_ATTR_ENCC_SIZE
     weiwang     05/06/04 - add OCIAQListenOpts and OCIAQLisMsgProps 
     weiwang     04/30/04 - add OCI_AQ_RESERVED_5
     nbhatt      04/27/04 - add new attribute 
     ssvemuri    06/19/04 -  change notification descriptors and attributes
     ksurlake    06/01/04 - grabtrans 'ksurlake_txn_skmishra_clone' 
     ksurlake    05/13/04 - add subscriber handle attributes
     mbastawa    06/01/04 - add 3 more OCI_FETCH_RESERVED modes
     chliang     05/28/04 - add nchar literal replacement modes
     nikeda      05/14/04 - [OLS on RAC] new authentication mode 
     debanerj    05/17/04 - 13064: add fncodes for LOB array Read and Write
     nikeda      05/20/04 - [OCI Events] Add incarnation, cardinality,reason 
     nikeda      05/18/04 - [OCI Events] Add OCI_ATTR_SERVICENAME 
     nikeda      05/17/04 - Add event handle 
     nikeda      05/13/04 - [OCI Events] Rename HACBK->EVTCBK, HACTX->EVTCTX 
     nikeda      05/10/04 - [OCI Events] code review changes 
     nikeda      04/15/04 - [OCI Events] OCI_SESSRLS_DROPSESS_FORCE 
     nikeda      04/12/04 - [OCI Events] Add OCI_ATTR_USER_MEMORY 
     aahluwal    04/12/04 - add OCI_HNDLFR_RESERVED5 
     vraja       04/28/04 - add options for redo sync on commit 
     aahluwal    05/29/04 - [OCI Events]: add support for svc, svc member events 
     nikeda      05/28/04 - grabtrans 'nikeda_oci_events_copy' 
     nikeda      05/18/04 - [OCI Events] Add OCI_ATTR_SERVICENAME 
     nikeda      05/17/04 - Add event handle 
     nikeda      05/13/04 - [OCI Events] Rename HACBK->EVTCBK, HACTX->EVTCTX 
     nikeda      05/10/04 - [OCI Events] code review changes 
     nikeda      04/15/04 - [OCI Events] OCI_SESSRLS_DROPSESS_FORCE 
     nikeda      04/12/04 - [OCI Events] Add OCI_ATTR_USER_MEMORY 
     aahluwal    04/12/04 - add OCI_HNDLFR_RESERVED5 
     jciminsk    04/28/04 - merge from RDBMS_MAIN_SOLARIS_040426 
     jacao       03/06/04 - add OCI_ATTR_CURRENT_SCHEMA 
     aahluwal    01/20/04 - remove OCI_KEEP_FETCH_STATE
     aahluwal    03/25/04 - [OCI Events] add OCI_HTYPE_HAEVENT and related attrs 
     nikeda      03/19/04 - [OCI Events] Add OCI_ATTR_HACBK and OCI_ATTR_HACTX 
     dfrumkin    12/04/03 - Add database startup/shutdown
     chliang     12/22/03 - grid/main merge: add OCI_ATTR_RESERVED_20 
     jciminsk    12/12/03 - merge from RDBMS_MAIN_SOLARIS_031209 
     sgollapu    09/19/03 - Add fetch modes 
     sgollapu    07/30/03 - Add TSM attributes
     sgollapu    06/26/03 - Add OCI_MUTEX_TRY
     aime        06/23/03 - sync grid with main
     sgollapu    06/07/03 - Add reserved attribute
     sgollapu    06/05/03 - Add reserved auth flag
     rpingte     05/22/03 - Add OCI_ATCH_RESERVED_5
     sgollapu    05/06/03 - Add TSM attributes
     sgollapu    04/10/03 - Session migration Flags/interfaces
     dfrumkin    04/23/04 - add OCI_PREP2_RESERVED_1
     rpingte     05/06/04 - add major and minor version information
     bsinha      04/06/04 - add new OCI_TRANS flag
     chliang     11/26/03 - add OCI_ATTR_RESERVED_19 
     preilly     10/23/03 - Make OCI_ATTR_DIRPATH_METADATA_BUF private 
     chliang     08/07/03 - add OCI_ATTR_SKIP_BUFFER
     srseshad    03/12/03 - convert public oci api to ansi
     weiwang     05/14/03 - remove iot creation for rule sets
     rkoti       04/15/03 - [2746515] add fntcodes for Unlimited size LOB 6003
     tcruanes    05/13/03 - add slave SQL OCI execution mode
     rkoti       02/21/03 - [2761455] add OCI_FNCODE_AQENQARRAY,
                            OCI_FNCODE_AQDEQARRAY and update OCI_FNCODE_MAXFCN
     tkeefe      01/29/03 - bug-2773794: Add new interface for setting Kerb attrs
     aahluwal    02/06/03 - add OCI_ATTR_TRANSFORMATION_NO
     weiwang     12/05/02 - add OCI_ATTR_USER_PROPERTY
     ataracha    01/03/03 - includeocixmldb.h
     preilly     12/05/02 - Add wait attribute for locking when using dir path
     tkeefe      01/03/03 - bug-2623771: Added OCI_ATTR_KERBEROS_KEY
     lchidamb    12/13/02 - end-to-end tracing attributes
     msakayed    10/28/02 - Bug #2643907: add OCI_ATTR_DIRPATH_SKIPINDEX_METHOD
     rphillip    11/13/02 - Add OCIP_ATTR_DIRPATH_INDEX
     sagrawal    10/13/02 - liniting
     sagrawal    10/03/02 - PL/SQL Compiler warnings
     jstenois    11/07/02 - remove ocixad.h
     chliang     10/21/02 - add OCI_ATTR_RESERVED_16,17
     hsbedi      10/30/02 - grabtrans 'jstenois_fix_xt_convert'
     aahluwal    10/12/02 - add OCI_ATTR_AQ_NUM_E_ERRORS/OCI_ATTR_AQ_ERROR_INDEX
     bdagevil    10/21/02 - add SQL analyze internal exec mode
     csteinba    10/11/02 - add OCI_ATTR_RESERVED_16
     chliang     10/12/02 - add bind row callback attributes
     preilly     10/25/02 - Add new reserved parameters
     tkeefe      10/31/02 - bug-2623771: Added OCI_ATTR_AUDIT_SESSION_ID
     csteinba    10/04/02 - Add OCI_ATTR_RESERVED_15
     mhho        10/11/02 - add new credential constant
     thoang      09/25/02 - Add OCI_XMLTYPE_CREATE_CLOB
     skaluska    10/07/02 - describe rules objects
     csteinba    09/16/02 - Remove OCI_CACHE
     gtarora     10/03/02 - OCI_ATTR_COL_SUBS => OCI_ATTR_OBJ_SUBS
     msakayed    09/09/02 - Bug #2482469: add OCI_ATTR_DIRPATH_RESERVED_[3-6]
     aahluwal    08/30/02 - adding dequeue across txn group
     srseshad    04/24/02 - Add attribute OCI_ATTR_SPOOL_STMTCACHESIZE.
     ebatbout    07/22/02 - Remove OCI_ATTR_RESERVED_11.
     abande      01/17/02 - Bug 1788921; Add external attribute.
     aahluwal    06/04/02 - bug 2360115
     pbagal      05/24/02 - Incorporate review comments
     pbagal      05/22/02 - Introduce instance type attribute.
     whe         07/01/02 - add OCI_BIND_DEFINE_SOFT flags
     gtarora     07/01/02 - Add OCI_ATTR_COL_SUBS
     tkeefe      05/30/02 - Add support for new proxy authentication credentials
     dgprice     12/18/01 - bug 2102779 add reserved force describe
     schandir    11/19/01 - add/modify modes.
     schandir    11/15/01 - add OCI_SPC_STMTCACHE.
     schandir    12/06/01 - change mode value of OCI_SPOOL.
     msakayed    11/02/01 - Bug #2094292: add OCI_ATTR_DIRPATH_INPUT
     dsaha       11/09/01 - add OCI_DTYPE_RESERVED1
     skabraha    11/05/01 - new method flag
     skabraha    10/25/01 - another flag for XML
     skabraha    10/11/01 - describe flags for subtypes
     nbhatt      09/18/01 - new reserved AQ flags
     celsbern    10/19/01 - merge LOG to MAIN
     ksurlake    10/12/01 - add OCI_ATTR_RESERVED_13
     ksurlake    08/13/01 - add OCI_ATTR_RESERVED_12
     schandir    09/24/01 - Adding stmt caching
     abande      09/04/01 - Adding session pooling
     sagrawal    10/23/01 - add new bit for OCIPHandleFree
     preilly     10/25/01 - Add support for specifying metadata on DirPathCtx
     skabraha    09/24/01 - describe flags for XML type
     schandir    09/24/01 - Adding stmt caching
     abande      09/04/01 - Adding session pooling
     stakeda     09/17/01 - add OCI_NLS_CHARSET_ID
     whe         09/19/01 - add OCIXMLType create options
     rpingte     09/11/01 - add OCI_MUTEX_ENV_ONLY and OCI_NO_MUTEX_STMT
     cmlim       08/28/01 - mod datecache attrs to use same naming as dpapi attrs
     wzhang      08/24/01 - Add new keywords for OCINlsNameMap.
     rphillip    05/02/01 - Add date cache attributes
     rphillip    08/22/01 - Add new stream version
     ebatbout    04/13/01 - add definition, OCI_ATTR_RESERVED_11
     chliang     04/12/01 - add shortnames for newer oci funcation
     wzhang      04/11/01 - Add new OCI NLS constants.
     cmlim       04/13/01 - remove attrs not used by dpapi (151 & 152 avail)
     rkambo      03/23/01 - bugfix 1421793
     cmlim       04/02/01 - remove OCI_ATTR_DIRPATH_NESTED_TBL, SUBST_OBJ_TBL
                          - note: attribute #s 186 & 205 available
     whe         03/28/01 - add OCI_AFC_PAD_ON/OFF mode
     preilly     03/05/01 - Add stream versioning support to DirPath context
     schandir    12/18/00 - remove attr CONN_INCR_DELAY.
     schandir    12/12/00 - change mode from OCI_POOL to OCI_CPOOL.
     cbarclay    01/12/01 - add atribute for OCIP_ATTR_TMZ
     whe         01/07/01 - add attributes related to UTF16 env mode
     slari       12/29/00 - add blank line
     slari       12/28/00 - OCI_ATTR_RESERVED_10
     whe         12/19/00 - add OCI_ENVCR_RESERVED3
     rpang       11/29/00 - Added OCI_ATTR_ORA_DEBUG_JDWP attribute
     cmlim       11/28/00 - support substitutable object tables in dpapi
     akatti      10/09/00 - [198379]:add OCIRowidToChar
     sgollapu    10/11/00 - Add OCI_PREP_RESERVED_1
     sgollapu    08/27/00 - add attribute to get erroneous column
     sgollapu    07/29/00 - Add snapshot attributes
     kmohan      09/18/00 - add OCI_FNCODE_LOGON2
     abrumm      10/08/00 - include ocixad.h
     mbastawa    10/04/00 - add OCI_ATTR_ROWS_FETCHED
     nbhatt      08/24/00 - add transformation attribute
     dmwong      08/22/00 - OCI_ATTR_CID_VALUE -> OCI_ATTR_CLIENT_IDENTIFIER.
     cmlim       08/30/00 - add OCI_ATTR_DIRPATH_SID
     dsaha       08/18/00 - add OCI_ATTR_RESERVED_5
     amangal     08/17/00 - Merge into 8.2 : 1194361
     slari       08/03/00 - add OCI_ATTR_HANDLE_POSITION
     dsaha       07/20/00 - 2rt exec
     sgollapu    07/04/00 - Add virtual session flag
     cmlim       07/07/00 - add OCI_ATTR_DIRPATH_OID, OCI_ATTR_DIRPATH_NESTED_TBL
     etucker     07/28/00 - add OCIIntervalFromTZ
     rwessman    06/26/00 - N-tier: added new credential attributes
     whe         07/27/00 - add OCI_UTF16 mode
     vjayaram    07/18/00 - add connection pooling changes
     etucker     07/12/00 - add dls apis
     cmlim       07/07/00 - add OCI_ATTR_DIRPATH_OID, OCI_ATTR_DIRPATH_NESTED_TBL
     sgollapu    07/04/00 - Add virtual session flag
     najain      05/01/00 - AQ Signature support
     sgollapu    06/14/00 - Add reserved OCI mode
     rkambo      06/08/00 - notification presentation support
     sagrawal    06/04/00 - ref cursor to c
     ksurlake    06/07/00 - define OCI_POOL
     mbastawa    06/05/00 - added scrollable cursor attributes
     weiwang     03/31/00 - add LDAP support
     whe         05/30/00 - add OCI_ATTR_MAXCHAR_SIZE
     whe         05/23/00 - validate OCI_NO_CACHE mode
     dsaha       02/02/00 - Add no-cache attr in statement handle
     whe         05/23/00 - add OCIP_ICACHE
     allee       05/17/00 - describe support for JAVA implmented TYPE
     preilly     05/30/00 - Continue adding support for objects in direct path lo
     cmlim       05/16/00 - 8.2 dpapi support of ADTs
     rxgovind    05/04/00 - OCIAnyDataSet changes
     rkasamse    05/25/00 - add OCIAnyDataCtx
     rmurthy     04/26/00 - describe support for inheritance
     ksurlake    04/18/00 - Add credential type
     whe         05/24/00 - add OCI_ATTR_CHAR_ attrs
     rkambo      04/19/00 - subscription enhancement
     rmurthy     04/26/00 - describe support for inheritance
     delson      03/28/00 - add OCI_ATTR_RESERVED_2
     abrumm      03/31/00 - external table support
     rkasamse    03/13/00 - add declarations for OCIAnyData
     najain      02/24/00 - support for dequeue as select
     dsaha       03/10/00 - Add OCI_ALWAYS_BLOCKING
     esoyleme    04/25/00 - separated transactions
     sgollapu    12/23/99 - OCIServerAttach extensions
     slari       08/23/99 - add OCI_DTYPE_UCB
     slari       08/20/99 - add OCI_UCBTYPE_REPLACE
     hsbedi      08/31/99 - Memory Stats .
     sgollapu    08/02/99 - oci sql routing
     slari       08/06/99 - rename values for OCI_SERVER_STATUS
     slari       08/02/99 - add OCI_ATTR_SERVER_STATUS
     tnbui       07/28/99 - Remove OCI_DTYPE_TIMESTAMP_ITZ                       
     amangal     07/19/99 - Merge into 8.1.6 : bug 785797
     tnbui       07/07/99 - Change ADJUSTMENT modes                              
     dsaha       07/07/99 - OCI_SAHRED_EXT
     dmwong      06/08/99 - add OCI_ATTR_APPCTX_*
     vyanaman    06/23/99 -
     vyanaman    06/21/99 - Add new OCI Datetime and Interval descriptors
     esoyleme    06/29/99 - expose MTS performance enhancements                  
     rshaikh     04/23/99 - add OCI_SQL_VERSION_*
     tnbui       05/24/99 - Remove OCIAdjStr                                     
     dsaha       05/21/99 - Add OCI_ADJUST_UNK
     mluong      05/17/99 - fix merge
     tnbui       04/05/99 - ADJUSTMENT values
     abrumm      04/16/99 - dpapi: more attributes
     dsaha       02/24/99 - Add OCI_SHOW_DML_WARNINGS
     jiyang      12/07/98 - Add OCI_NLS_DUAL_CURRENCY
     slari       12/07/98 - change OCI_NOMUTEX to OCI_NO_MUTEX
     aroy        11/30/98 - change OCI_NOCALLBACK to OCI_NO_UCB
     aroy        11/13/98 - add env modes to process modes
     slari       09/08/98 - add OCI_FNCODE_SVC2HST and _SVCRH
     aroy        09/04/98 - Add OCI_ATTR_MIGSESSION
     skray       08/14/98 - server groups for session switching
     mluong      08/11/98 - add back OCI_HTYPE_LAST.
     aroy        05/25/98 - add process handle type                              
     aroy        04/06/98 - add shared mode                                      
     slari       07/13/98 -  merge forward to 8.1.4
     slari       07/09/98 -  add OCI_BIND_RESERVED_2
     slari       07/08/98 -  add OCI_EXACT_FETCH_RESERVED_1
     dsaha       07/07/98 -  Add OCI_PARSE_ONLY
     dsaha       06/29/98 -  Add OCI_PARSE_ONLY
     slari       07/01/98 -  add OCI_BIND_RESERVED_2
     sgollapu    06/25/98 -  Fix bug 683565
     slari       06/17/98 -  remove OC_FETCH_RESERVED_2
     slari       06/11/98 -  add OCI_FETCH_RESERVED_1 and 2
     jhasenbe    05/27/98 -  Remove definitions for U-Calls (Unicode)
     jiyang      05/18/98 - remove OCI_ATTR_CARTLANG
     nbhatt      05/20/98 -  OCI_DEQ_REMOVE_NODATA
     nbhatt      05/19/98 - correct AQ opcode
     skmishra    05/06/98 - Add precision attribute to Attributes list
     aroy        04/20/98 - merge forward 8.0.5 -> 8.1.3
     schandra    05/01/98 - OCI sender id
     sgollapu    02/19/98 - enhanced array DML
     nbhatt      05/15/98 -  AQ listen call
     sgollapu    04/27/98 - more attributes
     skaluska    04/06/98 - Add OCI_PTYPE_SCHEMA, OCI_PTYPE_DATABASE
     slari       04/28/98 - add OCI_ATTR_PDPRC
     lchidamb    05/05/98 - change OCI_NAMESPACE_AQ to 1
     nbhatt      04/27/98 - AQ Notification Descriptor
     abrumm      06/24/98 - more direct path attributes
     abrumm      05/27/98 - OCI direct path interface support
     abrumm      05/08/98 - OCI direct path interface support
     lchidamb    03/02/98 - client notification additions
     kkarun      04/17/98 - Add more Interval functions
     vyanaman    04/16/98 - Add get/set TZ
     kkarun      04/14/98 - Add OCI Datetime shortnames
     vyanaman    04/13/98 - Add OCI DateTime and Interval check error codes
     kkarun      04/07/98 - Add OCI_DTYPE_DATETIME and OCI_DTYPE_INTERVAL
     esoyleme    12/15/97 - support failover callback retry
     esoyleme    04/22/98 - merge support for failover callback retry
     mluong      04/16/98 - add OCI_FNCODE_LOBLOCATORASSIGN
     rkasamse    04/17/98 - add short names for OCIPickler(Memory/Ctx) cart servi
     slari       04/10/98 - add OCI_FNCODE_SVCCTXTOLDA
     slari       04/09/98 - add OCI_FNCODE_RESET
     slari       04/07/98 - add OCI_FNCODE_LOBFILEISOPEN
     slari       04/06/98 - add OCI_FNCODE_LOBOPEN
     slari       03/20/98 - change OCI_CBTYPE_xxx to OCI_UCBTYPE_xxx
     slari       03/18/98 - add OCI_FNCODE_MAXFCN
     slari       02/12/98 - add OCI_ENV_NO_USRCB
     skabraha    04/09/98 - adding shortnames for OCIFile
     rhwu        04/03/98 - Add short names for the OCIThread package
     tanguyen    04/03/98 - add OCI_ATTR_xxxx for type inheritance
     rkasamse    04/02/98 - add OCI_ATTR_UCI_REFRESH
     nramakri    04/01/98 - Add short names for the OCIExtract package
     ewaugh      03/31/98 - Add short names for the OCIFormat package.
     jhasenbe    04/06/98 - Add definitions for U-Calls (Unicode)
                            (OCI_TEXT, OCI_UTEXT, OCI_UTEXT4)
     skmishra    03/03/98 - Add OCI_ATTR_PARSE_ERROR_OFFSET
     rwessman    03/11/98 - Added OCI_CRED_PROXY for proxy authentication
     abrumm      03/31/98 - OCI direct path interface support
     nmallava    03/03/98 - add constants for temp lob apis
     skotsovo    03/05/98 - resolve merge conflicts
     skotsovo    02/24/98 - add OCI_DTYPE_LOC
     skaluska    01/21/98 - Add OCI_ATTR_LTYPE
     rkasamse    01/06/98 - add OCI_ATTR* for obj cache enhancements
     dchatter    01/08/98 - more comments
     skabraha    12/02/97 - moved oci1.h to the front of include files.
     jiyang      12/18/97 - Add OCI_NLS_MAX_BUFSZ
     rhwu        12/02/97 - move oci1.h up
     ewaugh      12/15/97 - Add short names for the OCIFormat package.
     rkasamse    12/02/97 - Add a constant for memory cartridge services -- OCI_M
     nmallava    12/31/97 - open/close for internal lobs
     khnguyen    11/27/97 - add OCI_ATTR_LFPRECISION, OCI_ATTR_FSPRECISION
     rkasamse    11/03/97 - add types for pickler cartridge services
     mluong      11/20/97 - changed ubig_ora to ub4 per skotsovo
     ssamu       11/14/97 - add oci1.h
     jiyang      11/13/97 - Add NLS service for cartridge
     esoyleme    12/15/97 - support failover callback retry
     jwijaya     10/21/97 - change OCILobOffset/Length from ubig_ora to ub4
     cxcheng     07/28/97 - fix compile with SLSHORTNAME
     schandra    06/25/97 - AQ OCI interface
     sgollapu    07/25/97 - Add OCI_ATTR_DESC_PUBLIC
     cxcheng     06/16/97 - add OCI_ATTR_TDO
     skotsovo    06/05/97 - add fntcodes for lob buffering subsystem
     esoyleme    05/13/97 - move failover callback prototype
     skmishra    05/06/97 - stdc compiler fixes
     skmishra    04/22/97 - Provide C++ compatibility
     lchidamb    04/19/97 - add OCI_ATTR_SESSLANG
     ramkrish    04/15/97 - Add OCI_LOB_BUFFER_(NO)FREE
     sgollapu    04/18/97 - Add OCI_ATTR_TABLESPACE
     skaluska    04/17/97 - Add OCI_ATTR_SUB_NAME
     schandra    04/10/97 - Use long OCI names
     aroy        03/27/97 - add OCI_DTYPE_FILE
     sgollapu    03/26/97 - Add OCI_OTYPEs
     skmishra    04/09/97 - Added constant OCI_ROWID_LEN
     dchatter    03/21/97 - add attr OCI_ATTR_IN_V8_MODE
     lchidamb    03/21/97 - add OCI_COMMIT_ON_SUCCESS execution mode
     skmishra    03/20/97 - Added OCI_ATTR_LOBEMPTY
     sgollapu    03/19/97 - Add OCI_ATTR_OVRLD_ID
     aroy        03/17/97 - add postprocessing callback
     sgollapu    03/15/97 - Add OCI_ATTR_PARAM
     cxcheng     02/07/97 - change OCI_PTYPE codes for type method for consistenc
     cxcheng     02/05/97 - add OCI_PTYPE_TYPE_RESULT
     cxcheng     02/04/97 - rename OCI_PTYPE constants to be more consistent
     cxcheng     02/03/97 - add OCI_ATTR, OCI_PTYPE contants for describe type
     esoyleme    01/23/97 - merge neerja callback
     sgollapu    12/30/96 - Remove OCI_DTYPE_SECURITY
     asurpur     12/26/96 - CHanging OCI_NO_AUTH to OCI_AUTH
     sgollapu    12/23/96 - Add more attrs to COL, ARG, and SEQ
     sgollapu    12/12/96 - Add OCI_DESCRIBE_ONLY
     slari       12/11/96 - change prototype of OCICallbackInBind
     nbhatt      12/05/96 - "callback"
     lchidamb    11/19/96 - handle subclassing
     sgollapu    11/09/96 - OCI_PATTR_*
     dchatter    11/04/96 - add attr OCI_ATTR_CHRCNT
     mluong      11/01/96 - test
     cxcheng     10/31/96 - add #defines for OCILobLength etc
     dchatter    10/31/96 - add lob read write call back fp defs
     dchatter    10/30/96 - more changes
     rhari       10/30/96 - Include ociextp.h at the very end
     lchidamb    10/22/96 - add fdo attribute for bind/server handle
     dchatter    10/22/96 - change attr defn for prefetch parameters & lobs/file
                            calls
     slari       10/21/96 - add OCI_ENV_NO_MUTEX
     rhari       10/25/96 - Include ociextp.h
     rxgovind    10/25/96 - add OCI_LOBMAXSIZE, remove OCI_FILE_READWRITE
     sgollapu    10/24/96 - Correct OCILogon and OCILogoff
     sgollapu    10/24/96 - Correct to OCILogon and OCILogoff
     sgollapu    10/21/96 - Add ocilon and ociloff
     skaluska    10/31/96 - Add OCI_PTYPE values
     sgollapu    10/17/96 - correct OCI_ATTR_SVCCTX to OCI_ATTR_SERVER
     rwessman    10/16/96 - Added security functions and fixed olint errors.
     sthakur     10/14/96 - add more COR attributes
     cxcheng     10/14/96 - re-enable LOB functions
     sgollapu    10/10/96 - Add ocibdp and ocibdn
     slari       10/07/96 - add back OCIRowid
     aroy        10/08/96 -  add typedef ocibfill for PRO*C
     mluong      10/11/96 - replace OCI_ATTR_CHARSET* with OCI_ATTR_CHARSET_*
     cxcheng     10/10/96 - temporarily take out #define for lob functions
     sgollapu    10/02/96 - Rename OCI functions and datatypes
     skotsovo    10/01/96 - move orl lob fnts to oci
     aroy        09/10/96 - fix merge errors
     aroy        08/19/96 - NCHAR support
     jboonleu    09/05/96 - add OCI attributes for object cache
     dchatter    08/20/96 - HTYPE ranges from 1-50; DTYPE from 50-255
     slari       08/06/96 - define OCI_DTYPE_ROWID
     sthakur     08/14/96 - complex object support
     schandra    06/17/96 - Convert XA to use new OCI
     abrik       08/15/96 - OCI_ATTR_HEAPALLOC added
     aroy        07/17/96 - terminology change: ocilobd => ocilobl
     aroy        07/03/96 - add lob typedefs for Pro*C
     slari       06/28/96 - add OCI_ATTR_STMT_TYPE
     lchidamb    06/26/96 - reorg #ifndef
     schandra    05/31/96 - attribute types for internal and external client name
     asurpur     05/30/96 - Changing the value of mode
     schandra    05/18/96 - OCI_TRANS_TWOPHASE -> 0x00000001 to 0x00100000
     slari       05/30/96 - add callback function prototypes
     jbellemo    05/23/96 - remove ociisc
     schandra    04/23/96 - loosely-coupled branches
     asurpur     05/15/96 - New mode for ocicpw
     aroy        04/24/96 - making ocihandles opaque
     slari       04/18/96 - add missing defines
     schandra    03/27/96 - V8OCI - add transaction related calls
     dchatter    04/01/96 - add OCI_FILE options
     dchatter    03/21/96 - add oci2lda conversion routines
     dchatter    03/07/96 - add OCI piece definition
     slari       03/12/96 - add describe attributes
     slari       03/12/96 - add OCI_OTYPE_QUERY
     aroy        02/28/96 - Add column attributes
     slari       02/09/96 - add OCI_OBJECT
     slari       02/07/96 - add OCI_HYTPE_DSC
     aroy        01/10/96 - adding function code defines...
     dchatter    01/03/96 - define OCI_NON_BLOCKING
     dchatter    01/02/96 - Add Any descriptor
     dchatter    01/02/96 - Add Select List descriptor
     dchatter    12/29/95 - V8 OCI definitions
     dchatter    12/29/95 - Creation
  
   }

  const // forward declarations which aren't possible in pascal
  { Attach using server handle from pool  }
     OCI_CPOOL = $0200;
  { char set id  }
     OCI_ATTR_NCHARSET_ID = 262;

{$include ocidfn.inc}

  {---------------------------------------------------------------------------
   Short names provided for platforms which do not allow extended symbolic names 
    --------------------------------------------------------------------------- }
{$ifdef SLSHORTNAME}
  { Translation of the long function/type names to short names for IBM only  }
  { maybe lint will use this too  }

  const
     OCISessionEnd = ocitac;     
     OCIResultSetToStmt = ocirs2sh;     
     OCISessionBegin = ociauth;     
     OCIServerAttach = ociatch;     
     OCIDescriptorAlloc = ocigdesc;     
     OCIServerDetach = ocidtch;     
     OCIDescriptorFree = ocifdesc;     
     OCIServerVersion = ocivers;     
     OCIDescribeAny = ocidsca;     
     OCIBindDynamic = ocibda;     
     OCIBindByName = ocibdn;     
     OCIBindByPos = ocibdp;     
     OCIErrorGet = ocigdr;     
     OCIBindArrayOfStruct = ocibsa;     
     OCIEnvInit = ociinit;     
     OCIBindObject = ocibndt;     
     OCIHandleAlloc = ocighndl;     
     OCIHandleFree = ocifhndl;     
     OCIRowidToChar = ociri2c;     
{$ifdef NEVER}

  const
     OCIStmtBindByPos = ocibndp;     
     OCIStmtBindByName = ocibndn;     
{$endif}

  const
     OCIAttrGet = ocigattr;     
     OCIDefineByPos = ocidfne;     
     OCIAttrSet = ocisattr;     
     OCIDefineDynamic = ociddf;     
     OCILdaToSvcCtx = ocild2sv;     
     OCIDefineArrayOfStruct = ocidarr;     
     OCIInitialize = ocipi;     
     OCIDefineObject = ocidndt;     
     OCIStmtExecute = ociexec;     
     OCILobAppend = ocilfap;     
     OCILobOpenFile = ocifopn;     
     OCILobCloseFile = ocifcls;     
     OCILobLocator = ocilobd;     
     OCILobCopy = ocilfcp;     
     OCILobFileCreate = ocifcrt;     
     OCILobFileDelete = ocifdel;     
     OCILobGetLength = ocilfln;     
     OCILobWrite = ocilfwr;     
     OCILobRead = ocilfrd;     
     OCILobErase = ocilfer;     
     OCILobTrim = ocilftr;     
     OCIStmtFetch = ocifch;     
     OCIStmtGetBindInfo = ocigbp;     
     OCIStmtGetPieceInfo = ocigpi;     
     OCIStmtPrepare = ocireq;     
     OCIStmtSetPieceInfo = ocispi;     
     OCISvcCtxToLda = ocisv2ld;     
     OCITransCommit = ocitxcm;     
     OCITransDetach = ocitxdt;     
     OCITransForget = ocitxfgt;     
     OCITransPrepare = ocitxpre;     
     OCITransRollback = ocitxrl;     
     OCIPasswordChange = ocicpw;     
     OCITransStart = ocitxst;     
     OCITransMultiPrepare = ocitxmp;     
     OCIBreak = ocibreak;     
     OCIParamGet = ocigparm;     
     OCIParamSet = ocisparm;     
     OCISecurityOpenWallet = ocizwOpenWallet;     
     OCISecurityCloseWallet = ocizwCloseWallet;     
     OCISecurityCreateWallet = ocizwCreateWallet;     
     OCISecurityDestroyWallet = ocizwDestroyWallet;     
     OCISecurityStorePersona = ocizeStorePersona;     
     OCISecurityOpenPersona = ocizeOpenPersona;     
     OCISecurityClosePersona = ocizeClosePersona;     
     OCISecurityRemovePersona = ocizeRemovePersona;     
     OCISecurityCreatePersona = ocizeCreatePersona;     
     OCISecuritySetProtection = ocizeSetProtection;     
     OCISecurityGetProtection = ocizeGetProtection;     
     OCISecurityRemoveIdentity = ociziRemoveIdentity;     
     OCISecurityCreateIdentity = ociziCreateIdentity;     
     OCISecurityAbortIdentity = ociziAbortIdentity;     
     OCISecurityFreeIdentity = ociziFreeIdentity;     
     OCISecurityStoreTrustedIdentity = ociziStoreTrustedIdentity;     
     OCISecuritySign = ocizSign;     
     OCISecuritySignExpansion = ocizxSignExpansion;     
     OCISecurityVerify = ocizVerify;     
     OCISecurityValidate = ocizValidate;     
     OCISecuritySignDetached = ocizsd_SignDetached;     
     OCISecuritySignDetExpansion = ocizxsd_SignDetachedExpansion;     
     OCISecurityVerifyDetached = ocizved_VerifyDetached;     
     OCISecurity_PKEncrypt = ocizkec_PKEncrypt;     
     OCISecurityPKEncryptExpansion = ocizxkec_PKEncryptExpansion;     
     OCISecurityPKDecrypt = ocizkdc_PKDecrypt;     
     OCISecurityEncrypt = ocizEncrypt;     
     OCISecurityEncryptExpansion = ocizxEncryptExpansion;     
     OCISecurityDecrypt = ocizDecrypt;     
     OCISecurityEnvelope = ocizEnvelope;     
     OCISecurityDeEnvelope = ocizDeEnvelope;     
     OCISecurityKeyedHash = ocizKeyedHash;     
     OCISecurityKeyedHashExpansion = ocizxKeyedHashExpansion;     
     OCISecurityHash = ocizHash;     
     OCISecurityHashExpansion = ocizxHashExpansion;     
     OCISecuritySeedRandom = ocizSeedRandom;     
     OCISecurityRandomBytes = ocizrb_RandomBytes;     
     OCISecurityRandomNumber = ocizrn_RandomNumber;     
     OCISecurityInitBlock = ocizibInitBlock;     
     OCISecurityReuseBlock = ocizrbReuseBlock;     
     OCISecurityPurgeBlock = ocizpbPurgeBlock;     
     OCISecuritySetBlock = ocizsbSetBlock;     
     OCISecurityGetIdentity = ocizgi_GetIdentity;     
     OCIExtractInit = ocixeini;     
     OCIExtractTerm = ocixetrm;     
     OCIExtractReset = ocixerst;     
     OCIExtractSetNumKeys = ocixesnk;     
     OCIExtractSetKey = ocixesk;     
     OCIExtractFromFile = ocixeff;     
     OCIExtractFromStr = ocixefs;     
     OCIExtractToInt = ocixeti;     
     OCIExtractToBool = ocixetb;     
     OCIExtractToStr = ocixets;     
     OCIExtractToOCINum = ocixeton;     
     OCIExtractToList = ocixetl;     
     OCIExtractFromList = ocixefl;     
     OCIDateTimeGetTime = ocidt01_GetTime;     
     OCIDateTimeGetDate = ocidt02_GetDate;     
     OCIDateTimeGetTimeZoneOffset = ocidt03_GetTZ;     
     OCIDateTimeSysTimeStamp = ocidt07_SysTS;     
     OCIDateTimeAssign = ocidt08_Assign;     
     OCIDateTimeToText = ocidt09_ToText;     
     OCIDateTimeFromText = ocidt10_FromText;     
     OCIDateTimeCompare = ocidt11_Compare;     
     OCIDateTimeCheck = ocidt12_Check;     
     OCIDateTimeConvert = ocidt13_Convert;     
     OCIDateTimeSubtract = ocidt14_Subtract;     
     OCIDateTimeIntervalAdd = ocidt15_IntervalAdd;     
     OCIDateTimeIntervalSub = ocidt16_IntervalSub;     
     OCIDateTimeGetTimeZoneName = ocidt17_Gettzname;     
     OCIDateTimeToArray = ocidt18_ToArray;     
     OCIDateTimeFromArray = ocidt19_FromArray;     
     OCIIntervalSubtract = ociint01_Subtract;     
     OCIIntervalAdd = ociint02_Add;     
     OCIIntervalMultiply = ociint03_Multiply;     
     OCIIntervalDivide = ociint04_Divide;     
     OCIIntervalCompare = ociint05_Compare;     
     OCIIntervalFromText = ociint06_FromText;     
     OCIIntervalToText = ociint07_ToText;     
     OCIIntervalToNumber = ociint08_ToNumber;     
     OCIIntervalCheck = ociint09_Check;     
     OCIIntervalAssign = ociint10_Assign;     
     OCIIntervalGetYearMonth = ociint11_GetYearMonth;     
     OCIIntervalSetYearMonth = ociint12_SetYearMonth;     
     OCIIntervalGetDaySecond = ociint13_GetDaySecond;     
     OCIIntervalSetDaySecond = ociint14_SetDaySecond;     
     OCIIntervalFromNumber = ociint15_FromNumber;     
     OCIIntervalFromTZ = ociint16_FromTZ;     
     OCIFormatInit = ocixs01_Init;     
     OCIFormatString = ocixs02_Format;     
     OCIFormatTerm = ocixs03_Term;     
     OCIFormatTUb1 = ocixs04_TUb1;     
     OCIFormatTUb2 = ocixs05_TUb2;     
     OCIFormatTUb4 = ocixs06_TUb4;     
     OCIFormatTUword = ocixs07_TUword;     
     OCIFormatTUbig_ora = ocixs08_TUbig_ora;     
     OCIFormatTSb1 = ocixs09_TSb1;     
     OCIFormatTSb2 = ocixs10_TSb2;     
     OCIFormatTSb4 = ocixs11_TSb4;     
     OCIFormatTSword = ocixs12_TSword;     
     OCIFormatTSbig_ora = ocixs13_TSbig_ora;     
     OCIFormatTEb1 = ocixs14_TEb1;     
     OCIFormatTEb2 = ocixs15_TEb2;     
     OCIFormatTEb4 = ocixs16_TEb4;     
     OCIFormatTEword = ocixs17_TEword;     
     OCIFormatTChar = ocixs18_TChar;     
     OCIFormatTText = ocixs19_TText;     
     OCIFormatTDouble = ocixs20_TDouble;     
     OCIFormatTDvoid = ocixs21_TDvoid;     
     OCIFormatTEnd = ocixs22_TEnd;     
     OCIFileInit = ocifinit;     
     OCIFileTerm = ocifterm;     
     OCIFileOpen = ocifopen;     
     OCIFileClose = ocifclose;     
     OCIFileRead = ocifread;     
     OCIFileWrite = ocifwrite;     
     OCIFileSeek = ocifseek;     
     OCIFileExists = ocifexists;     
     OCIFileGetLength = ocifglen;     
     OCIFileFlush = ocifflush;     
  { OCIThread short name  }
     OCIThreadProcessInit = ocitt01_ProcessInit;     
     OCIThreadInit = ocitt02_Init;     
     OCIThreadTerm = ocitt03_Term;     
     OCIThreadIsMulti = ocitt04_IsMulti;     
     OCIThreadMutexInit = ocitt05_MutexInit;     
     OCIThreadMutexDestroy = ocitt06_MutexDestroy;     
     OCIThreadMutexAcquire = ocitt07_MutexAcquire;     
     OCIThreadMutexRelease = ocitt08_MutexRelease;     
     OCIThreadKeyInit = ocitt09_KeyInit;     
     OCIThreadKeyDestroy = ocitt10_KeyDestroy;     
     OCIThreadKeyGet = ocitt11_KeyGet;     
     OCIThreadKeySet = ocitt12_KeySet;     
     OCIThreadIdInit = ocitt13_IdInit;     
     OCIThreadIdDestroy = ocitt14_IdDestroy;     
     OCIThreadIdSet = ocitt15_IdSet;     
     OCIThreadIdSetNull = ocitt16_IdSetNull;     
     OCIThreadIdGet = ocitt17_IdGet;     
     OCIThreadIdSame = ocitt18_IdSame;     
     OCIThreadIdNull = ocitt19_IdNull;     
     OCIThreadHndInit = ocitt20_HndInit;     
     OCIThreadHndDestroy = ocitt21_HndDestroy;     
     OCIThreadCreate = ocitt22_Create;     
     OCIThreadJoin = ocitt23_Join;     
     OCIThreadClose = ocitt24_Close;     
     OCIThreadHandleGet = ocitt25_HandleGet;     
  { Translation between the old and new datatypes  }
     OCISession = ociusrh;     
     OCIBind = ocibndh;     
     OCIDescribe = ocidsch;     
     OCIDefine = ocidfnh;     
     OCIEnv = ocienvh;     
     OCIError = ocierrh;     
     OCICPool = ocicpool;     
     OCISPool = ocispool;     
     OCIAuthInfo = ociauthinfo;     
     OCILob = ocilobd;     
     OCILobLength = ocillen;     
     OCILobMode = ocilmo;     
     OCILobOffset = ociloff;     
     OCILobLocator = ocilobd;     
     OCIBlobLocator = ociblobl;     
     OCIClobLocator = ociclobl;     
     OCIBFileLocator = ocibfilel;     
     OCIParam = ocipard;     
     OCIResult = ocirstd;     
     OCISnapshot = ocisnad;     
     OCIServer = ocisrvh;     
     OCIStmt = ocistmh;     
     OCISvcCtx = ocisvch;     
     OCITrans = ocitxnh;     
     OCICallbackInBind = ocibicfp;     
     OCICallbackOutBind = ocibocfp;     
     OCICallbackDefine = ocidcfp;     
     OCICallbackLobRead = ocilrfp;     
     OCICallbackLobWrite = ocilwfp;     
     OCISecurity = ociossh;     
     OCIComplexObject = ocicorh;     
     OCIComplexObjectComp = ocicord;     
     OCIRowid = ociridd;     
     OCIAQDeq = ociaqdeq;     
     OCIAQEnq = ociaqenq;     
     OCIConnectionPoolCreate = ociconpc;     
     OCIConnectionPoolDestroy = ociconpd;     
     OCIEnvCreate = ocienvct;     
     OCILobAssign = ociloass;     
     OCILobCharSetForm = ocilocfm;     
     OCILobCharSetId = ocilocid;     
     OCILobDisableBuffering = ocilodbf;     
     OCILobEnableBuffering = ociloebf;     
     OCILobFileClose = ocilofcl;     
     OCILobFileCloseAll = ocilofca;     
     OCILobFileExists = ocilofex;     
     OCILobFileGetName = ocilofgn;     
     OCILobFileIsOpen = ocifiopn;     
     OCILobFileOpen = ocilofop;     
     OCILobFileSetName = ocilofsn;     
     OCILobFlushBuffer = ocilofbf;     
     OCILobIsEqual = ociloieq;     
     OCILobLoadFromFile = ocilolff;     
     OCILobLocatorIsInit = ocilolii;     
     OCILobLocatorAssign = ocilolas;     
     OCILogon = ocilogon;     
     OCILogon2 = ocilgon2;     
     OCILogoff = ocilgoff;     
{$endif}
  { ifdef SLSHORTNAME  }
  {--------------------------------------------------------------------------- 
                       PUBLIC TYPES AND CONSTANTS 
    --------------------------------------------------------------------------- }
  {-----------------------------Handle Types---------------------------------- }
  { handle types range from 1 - 49  }
  { start value of handle type  }

  const
     OCI_HTYPE_FIRST = 1;     
  { environment handle  }
     OCI_HTYPE_ENV = 1;     
  { error handle  }
     OCI_HTYPE_ERROR = 2;     
  { service handle  }
     OCI_HTYPE_SVCCTX = 3;     
  { statement handle  }
     OCI_HTYPE_STMT = 4;     
  { bind handle  }
     OCI_HTYPE_BIND = 5;     
  { define handle  }
     OCI_HTYPE_DEFINE = 6;     
  { describe handle  }
     OCI_HTYPE_DESCRIBE = 7;     
  { server handle  }
     OCI_HTYPE_SERVER = 8;     
  { authentication handle  }
     OCI_HTYPE_SESSION = 9;     
  { SessionGet auth handle  }
     OCI_HTYPE_AUTHINFO = OCI_HTYPE_SESSION;     
  { transaction handle  }
     OCI_HTYPE_TRANS = 10;     
  { complex object retrieval handle  }
     OCI_HTYPE_COMPLEXOBJECT = 11;     
  { security handle  }
     OCI_HTYPE_SECURITY = 12;     
  { subscription handle  }
     OCI_HTYPE_SUBSCRIPTION = 13;     
  { direct path context  }
     OCI_HTYPE_DIRPATH_CTX = 14;     
  { direct path column array  }
     OCI_HTYPE_DIRPATH_COLUMN_ARRAY = 15;     
  { direct path stream  }
     OCI_HTYPE_DIRPATH_STREAM = 16;     
  { process handle  }
     OCI_HTYPE_PROC = 17;     
  { direct path function context  }
     OCI_HTYPE_DIRPATH_FN_CTX = 18;     
  { dp object column array  }
     OCI_HTYPE_DIRPATH_FN_COL_ARRAY = 19;     
  { access driver session  }
     OCI_HTYPE_XADSESSION = 20;     
  { access driver table  }
     OCI_HTYPE_XADTABLE = 21;     
  { access driver field  }
     OCI_HTYPE_XADFIELD = 22;     
  { access driver granule  }
     OCI_HTYPE_XADGRANULE = 23;     
  { access driver record  }
     OCI_HTYPE_XADRECORD = 24;     
  { access driver I/O  }
     OCI_HTYPE_XADIO = 25;     
  { connection pool handle  }
     OCI_HTYPE_CPOOL = 26;     
  { session pool handle  }
     OCI_HTYPE_SPOOL = 27;     
  { admin handle  }
     OCI_HTYPE_ADMIN = 28;     
  { HA event handle  }
     OCI_HTYPE_EVENT = 29;     
  { last value of a handle type  }
     OCI_HTYPE_LAST = 29;     
  {--------------------------------------------------------------------------- }
  {-------------------------Descriptor Types---------------------------------- }
  { descriptor values range from 50 - 255  }
  { start value of descriptor type  }
     OCI_DTYPE_FIRST = 50;     
  { lob  locator  }
     OCI_DTYPE_LOB = 50;     
  { snapshot descriptor  }
     OCI_DTYPE_SNAP = 51;     
  { result set descriptor  }
     OCI_DTYPE_RSET = 52;     
  { a parameter descriptor obtained from ocigparm  }
     OCI_DTYPE_PARAM = 53;     
  { rowid descriptor  }
     OCI_DTYPE_ROWID = 54;     
     OCI_DTYPE_COMPLEXOBJECTCOMP = 55;     
  { complex object retrieval descriptor  }
  { File Lob locator  }
     OCI_DTYPE_FILE = 56;     
  { enqueue options  }
     OCI_DTYPE_AQENQ_OPTIONS = 57;     
  { dequeue options  }
     OCI_DTYPE_AQDEQ_OPTIONS = 58;     
  { message properties  }
     OCI_DTYPE_AQMSG_PROPERTIES = 59;     
  { aq agent  }
     OCI_DTYPE_AQAGENT = 60;     
  { LOB locator  }
     OCI_DTYPE_LOCATOR = 61;     
  { Interval year month  }
     OCI_DTYPE_INTERVAL_YM = 62;     
  { Interval day second  }
     OCI_DTYPE_INTERVAL_DS = 63;     
  { AQ notify descriptor  }
     OCI_DTYPE_AQNFY_DESCRIPTOR = 64;     
  { Date  }
     OCI_DTYPE_DATE = 65;     
  { Time  }
     OCI_DTYPE_TIME = 66;     
  { Time with timezone  }
     OCI_DTYPE_TIME_TZ = 67;     
  { Timestamp  }
     OCI_DTYPE_TIMESTAMP = 68;     
  { Timestamp with timezone  }
     OCI_DTYPE_TIMESTAMP_TZ = 69;     
  { Timestamp with local tz  }
     OCI_DTYPE_TIMESTAMP_LTZ = 70;     
  { user callback descriptor  }
     OCI_DTYPE_UCB = 71;     
  { server DN list descriptor  }
     OCI_DTYPE_SRVDN = 72;     
  { signature  }
     OCI_DTYPE_SIGNATURE = 73;     
  { reserved for internal use  }
     OCI_DTYPE_RESERVED_1 = 74;     
  { AQ listen options  }
     OCI_DTYPE_AQLIS_OPTIONS = 75;     
  { AQ listen msg props  }
     OCI_DTYPE_AQLIS_MSG_PROPERTIES = 76;     
  { Top level change notification desc  }
     OCI_DTYPE_CHDES = 77;     
  { Table change descriptor            }
     OCI_DTYPE_TABLE_CHDES = 78;     
  { Row change descriptor             }
     OCI_DTYPE_ROW_CHDES = 79;     
  { last value of a descriptor type  }
     OCI_DTYPE_LAST = 79;     
  {--------------------------------------------------------------------------- }
  {--------------------------------LOB types --------------------------------- }
  { LOB type - BLOB ------------------  }
     OCI_TEMP_BLOB = 1;     
  { LOB type - CLOB ------------------  }
     OCI_TEMP_CLOB = 2;     
  {--------------------------------------------------------------------------- }
  {-------------------------Object Ptr Types---------------------------------- }
  { object name  }
     OCI_OTYPE_NAME = 1;     
  { REF to TDO  }
     OCI_OTYPE_REF = 2;     
  { PTR to TDO  }
     OCI_OTYPE_PTR = 3;     
  {--------------------------------------------------------------------------- }
  {=============================Attribute Types=============================== }
  { 
     Note: All attributes are global.  New attibutes should be added to the end
     of the list. Before you add an attribute see if an existing one can be 
     used for your handle. 
  
     If you see any holes please use the holes first. 
   
   }
  {=========================================================================== }
  { the OCI function code  }
     OCI_ATTR_FNCODE = 1;     
  { is the environment initialized in object mode  }
     OCI_ATTR_OBJECT = 2;     
  { non blocking mode  }
     OCI_ATTR_NONBLOCKING_MODE = 3;     
  { the SQL verb  }
     OCI_ATTR_SQLCODE = 4;     
  { the environment handle  }
     OCI_ATTR_ENV = 5;     
  { the server handle  }
     OCI_ATTR_SERVER = 6;     
  { the user session handle  }
     OCI_ATTR_SESSION = 7;     
  { the transaction handle  }
     OCI_ATTR_TRANS = 8;     
  { the rows processed so far  }
     OCI_ATTR_ROW_COUNT = 9;     
  { the SQL verb of the statement  }
     OCI_ATTR_SQLFNCODE = 10;     
  { sets the number of rows to prefetch  }
     OCI_ATTR_PREFETCH_ROWS = 11;     
  { the prefetch rows of nested table }
     OCI_ATTR_NESTED_PREFETCH_ROWS = 12;     
  { memory limit for rows fetched  }
     OCI_ATTR_PREFETCH_MEMORY = 13;     
  { memory limit for nested rows  }
     OCI_ATTR_NESTED_PREFETCH_MEMORY = 14;     
     OCI_ATTR_CHAR_COUNT = 15;     
  { this specifies the bind and define size in characters  }
  { packed decimal scale  }
     OCI_ATTR_PDSCL = 16;     
     OCI_ATTR_FSPRECISION = OCI_ATTR_PDSCL;     
  { fs prec for datetime data types  }
  { packed decimal format  }
     OCI_ATTR_PDPRC = 17;     
     OCI_ATTR_LFPRECISION = OCI_ATTR_PDPRC;     
  { fs prec for datetime data types  }
  { number of column in the select list  }
     OCI_ATTR_PARAM_COUNT = 18;     
  { the rowid  }
     OCI_ATTR_ROWID = 19;     
  { the character set value  }
     OCI_ATTR_CHARSET = 20;     
  { NCHAR type  }
     OCI_ATTR_NCHAR = 21;     
  { username attribute  }
     OCI_ATTR_USERNAME = 22;     
  { password attribute  }
     OCI_ATTR_PASSWORD = 23;     
  { statement type  }
     OCI_ATTR_STMT_TYPE = 24;     
  { user friendly global name  }
     OCI_ATTR_INTERNAL_NAME = 25;     
  { the internal name for global txn  }
     OCI_ATTR_EXTERNAL_NAME = 26;     
  { XOPEN defined global transaction id  }
     OCI_ATTR_XID = 27;     
  {  }
     OCI_ATTR_TRANS_LOCK = 28;     
  { string to identify a global transaction  }
     OCI_ATTR_TRANS_NAME = 29;     
  { memory allocated on the heap  }
     OCI_ATTR_HEAPALLOC = 30;     
  { Character Set ID  }
     OCI_ATTR_CHARSET_ID = 31;     
  { Character Set Form  }
     OCI_ATTR_CHARSET_FORM = 32;     
  { Maximumsize of data on the server   }
     OCI_ATTR_MAXDATA_SIZE = 33;     
  { object cache optimal size  }
     OCI_ATTR_CACHE_OPT_SIZE = 34;     
  { object cache maximum size percentage  }
     OCI_ATTR_CACHE_MAX_SIZE = 35;     
  { object cache default pin option  }
     OCI_ATTR_PINOPTION = 36;     
     OCI_ATTR_ALLOC_DURATION = 37;     
  { object cache default allocation duration  }
  { object cache default pin duration  }
     OCI_ATTR_PIN_DURATION = 38;     
  { Format Descriptor object attribute  }
     OCI_ATTR_FDO = 39;     
     OCI_ATTR_POSTPROCESSING_CALLBACK = 40;     
  { Callback to process outbind data  }
     OCI_ATTR_POSTPROCESSING_CONTEXT = 41;     
  { Callback context to process outbind data  }
     OCI_ATTR_ROWS_RETURNED = 42;     
  { Number of rows returned in current iter - for Bind handles  }
  { Failover Callback attribute  }
     OCI_ATTR_FOCBK = 43;     
  { is the server/service context in V8 mode  }
     OCI_ATTR_IN_V8_MODE = 44;     
  { empty lob ?  }
     OCI_ATTR_LOBEMPTY = 45;     
  { session language handle  }
     OCI_ATTR_SESSLANG = 46;     
  { visibility  }
     OCI_ATTR_VISIBILITY = 47;     
  { relative message id  }
     OCI_ATTR_RELATIVE_MSGID = 48;     
  { sequence deviation  }
     OCI_ATTR_SEQUENCE_DEVIATION = 49;     
  { consumer name  }
     OCI_ATTR_CONSUMER_NAME = 50;     
  { dequeue mode  }
     OCI_ATTR_DEQ_MODE = 51;     
  { navigation  }
     OCI_ATTR_NAVIGATION = 52;     
  { wait  }
     OCI_ATTR_WAIT = 53;     
  { dequeue message id  }
     OCI_ATTR_DEQ_MSGID = 54;     
  { priority  }
     OCI_ATTR_PRIORITY = 55;     
  { delay  }
     OCI_ATTR_DELAY = 56;     
  { expiration  }
     OCI_ATTR_EXPIRATION = 57;     
  { correlation id  }
     OCI_ATTR_CORRELATION = 58;     
  { # of attempts  }
     OCI_ATTR_ATTEMPTS = 59;     
  { recipient list  }
     OCI_ATTR_RECIPIENT_LIST = 60;     
  { exception queue name  }
     OCI_ATTR_EXCEPTION_QUEUE = 61;     
  { enqueue time (only OCIAttrGet)  }
     OCI_ATTR_ENQ_TIME = 62;     
  { message state (only OCIAttrGet)  }
     OCI_ATTR_MSG_STATE = 63;     
  { NOTE: 64-66 used below  }
  { agent name  }
     OCI_ATTR_AGENT_NAME = 64;     
  { agent address  }
     OCI_ATTR_AGENT_ADDRESS = 65;     
  { agent protocol  }
     OCI_ATTR_AGENT_PROTOCOL = 66;     
  { user property  }
     OCI_ATTR_USER_PROPERTY = 67;     
  { sender id  }
     OCI_ATTR_SENDER_ID = 68;     
  { original message id  }
     OCI_ATTR_ORIGINAL_MSGID = 69;     
  { queue name  }
     OCI_ATTR_QUEUE_NAME = 70;     
  { message id  }
     OCI_ATTR_NFY_MSGID = 71;     
  { message properties  }
     OCI_ATTR_MSG_PROP = 72;     
  { num of errs in array DML  }
     OCI_ATTR_NUM_DML_ERRORS = 73;     
  { row offset in the array  }
     OCI_ATTR_DML_ROW_OFFSET = 74;     
  { AQ array error handling uses DML method of accessing errors  }
     OCI_ATTR_AQ_NUM_ERRORS = OCI_ATTR_NUM_DML_ERRORS;     
     OCI_ATTR_AQ_ERROR_INDEX = OCI_ATTR_DML_ROW_OFFSET;     
  { default date format string  }
     OCI_ATTR_DATEFORMAT = 75;     
  { buffer address  }
     OCI_ATTR_BUF_ADDR = 76;     
  { buffer size  }
     OCI_ATTR_BUF_SIZE = 77;     
  { For values 78 - 80, see DirPathAPI attribute section in this file  }
  { number of rows in column array  }
     OCI_ATTR_NUM_ROWS = 81;     
  { NOTE that OCI_ATTR_NUM_COLS is a column
                                     * array attribute too.
                                      }
  { columns of column array
                                                       processed so far.        }
     OCI_ATTR_COL_COUNT = 82;     
  { str off of last row processed  }
     OCI_ATTR_STREAM_OFFSET = 83;     
  { Shared Heap Allocation Size  }
     OCI_ATTR_SHARED_HEAPALLOC = 84;     
  { server group name  }
     OCI_ATTR_SERVER_GROUP = 85;     
  { migratable session attribute  }
     OCI_ATTR_MIGSESSION = 86;     
  { Temporary LOBs  }
     OCI_ATTR_NOCACHE = 87;     
  { Pool Size  }
     OCI_ATTR_MEMPOOL_SIZE = 88;     
  { Instance name  }
     OCI_ATTR_MEMPOOL_INSTNAME = 89;     
  { Application name  }
     OCI_ATTR_MEMPOOL_APPNAME = 90;     
  { Home Directory name  }
     OCI_ATTR_MEMPOOL_HOMENAME = 91;     
  { Pool Model (proc,thrd,both) }
     OCI_ATTR_MEMPOOL_MODEL = 92;     
  { Modes  }
     OCI_ATTR_MODES = 93;     
  { name of subscription  }
     OCI_ATTR_SUBSCR_NAME = 94;     
  { associated callback  }
     OCI_ATTR_SUBSCR_CALLBACK = 95;     
  { associated callback context  }
     OCI_ATTR_SUBSCR_CTX = 96;     
  { associated payload  }
     OCI_ATTR_SUBSCR_PAYLOAD = 97;     
  { associated namespace  }
     OCI_ATTR_SUBSCR_NAMESPACE = 98;     
  { Proxy user credentials  }
     OCI_ATTR_PROXY_CREDENTIALS = 99;     
  { Initial client role list  }
     OCI_ATTR_INITIAL_CLIENT_ROLES = 100;     
  { unknown attribute  }
     OCI_ATTR_UNK = 101;     
  { number of columns  }
     OCI_ATTR_NUM_COLS = 102;     
  { parameter of the column list  }
     OCI_ATTR_LIST_COLUMNS = 103;     
  { DBA of the segment header  }
     OCI_ATTR_RDBA = 104;     
  { whether the table is clustered  }
     OCI_ATTR_CLUSTERED = 105;     
  { whether the table is partitioned  }
     OCI_ATTR_PARTITIONED = 106;     
  { whether the table is index only  }
     OCI_ATTR_INDEX_ONLY = 107;     
  { parameter of the argument list  }
     OCI_ATTR_LIST_ARGUMENTS = 108;     
  { parameter of the subprogram list  }
     OCI_ATTR_LIST_SUBPROGRAMS = 109;     
  { REF to the type descriptor  }
     OCI_ATTR_REF_TDO = 110;     
  { the database link name  }
     OCI_ATTR_LINK = 111;     
  { minimum value  }
     OCI_ATTR_MIN = 112;     
  { maximum value  }
     OCI_ATTR_MAX = 113;     
  { increment value  }
     OCI_ATTR_INCR = 114;     
  { number of sequence numbers cached  }
     OCI_ATTR_CACHE = 115;     
  { whether the sequence is ordered  }
     OCI_ATTR_ORDER = 116;     
  { high-water mark  }
     OCI_ATTR_HW_MARK = 117;     
  { type's schema name  }
     OCI_ATTR_TYPE_SCHEMA = 118;     
  { timestamp of the object  }
     OCI_ATTR_TIMESTAMP = 119;     
  { number of sttributes  }
     OCI_ATTR_NUM_ATTRS = 120;     
  { number of parameters  }
     OCI_ATTR_NUM_PARAMS = 121;     
  { object id for a table or view  }
     OCI_ATTR_OBJID = 122;     
  { type of info described by  }
     OCI_ATTR_PTYPE = 123;     
  { parameter descriptor  }
     OCI_ATTR_PARAM = 124;     
  { overload ID for funcs and procs  }
     OCI_ATTR_OVERLOAD_ID = 125;     
  { table name space  }
     OCI_ATTR_TABLESPACE = 126;     
  { TDO of a type  }
     OCI_ATTR_TDO = 127;     
  { list type  }
     OCI_ATTR_LTYPE = 128;     
  { Parse Error offset  }
     OCI_ATTR_PARSE_ERROR_OFFSET = 129;     
  { whether table is temporary  }
     OCI_ATTR_IS_TEMPORARY = 130;     
  { whether table is typed  }
     OCI_ATTR_IS_TYPED = 131;     
  { duration of temporary table  }
     OCI_ATTR_DURATION = 132;     
  { is invoker rights  }
     OCI_ATTR_IS_INVOKER_RIGHTS = 133;     
  { top level schema obj name  }
     OCI_ATTR_OBJ_NAME = 134;     
  { schema name  }
     OCI_ATTR_OBJ_SCHEMA = 135;     
  { top level schema object id  }
     OCI_ATTR_OBJ_ID = 136;     
  { For values 137 - 141, see DirPathAPI attribute section in this file  }
  { transaction timeout  }
     OCI_ATTR_TRANS_TIMEOUT = 142;     
  { state of the server handle  }
     OCI_ATTR_SERVER_STATUS = 143;     
  { statement txt in stmt hdl  }
     OCI_ATTR_STATEMENT = 144;     
  { For value 145, see DirPathAPI attribute section in this file  }
  { dequeue condition  }
     OCI_ATTR_DEQCOND = 146;     
  { reserved  }
     OCI_ATTR_RESERVED_2 = 147;     
  { recepient of subscription  }
     OCI_ATTR_SUBSCR_RECPT = 148;     
  { protocol for recepient  }
     OCI_ATTR_SUBSCR_RECPTPROTO = 149;     
  { For values 150 - 151, see DirPathAPI attribute section in this file  }
  { LDAP host to connect to  }
     OCI_ATTR_LDAP_HOST = 153;     
  { LDAP port to connect to  }
     OCI_ATTR_LDAP_PORT = 154;     
  { bind DN  }
     OCI_ATTR_BIND_DN = 155;     
  { credentials to connect to LDAP  }
     OCI_ATTR_LDAP_CRED = 156;     
  { client wallet location  }
     OCI_ATTR_WALL_LOC = 157;     
  { LDAP authentication method  }
     OCI_ATTR_LDAP_AUTH = 158;     
  { LDAP adminstration context DN  }
     OCI_ATTR_LDAP_CTX = 159;     
  { list of registration server DNs  }
     OCI_ATTR_SERVER_DNS = 160;     
  { the number of server DNs  }
     OCI_ATTR_DN_COUNT = 161;     
  { server DN attribute  }
     OCI_ATTR_SERVER_DN = 162;     
  { max char size of data  }
     OCI_ATTR_MAXCHAR_SIZE = 163;     
  { for scrollable result sets }
     OCI_ATTR_CURRENT_POSITION = 164;     
  { Added to get attributes for ref cursor to statement handle  }
  { reserved  }
     OCI_ATTR_RESERVED_3 = 165;     
  { reserved  }
     OCI_ATTR_RESERVED_4 = 166;     
  { For value 167, see DirPathAPI attribute section in this file  }
  { digest algorithm  }
     OCI_ATTR_DIGEST_ALGO = 168;     
  { certificate  }
     OCI_ATTR_CERTIFICATE = 169;     
  { signature algorithm  }
     OCI_ATTR_SIGNATURE_ALGO = 170;     
  { canonicalization algo.  }
     OCI_ATTR_CANONICAL_ALGO = 171;     
  { private key  }
     OCI_ATTR_PRIVATE_KEY = 172;     
  { digest value  }
     OCI_ATTR_DIGEST_VALUE = 173;     
  { signature value  }
     OCI_ATTR_SIGNATURE_VAL = 174;     
  { signature  }
     OCI_ATTR_SIGNATURE = 175;     
  { attributes for setting OCI stmt caching specifics in svchp  }
  { size of the stm cache  }
     OCI_ATTR_STMTCACHESIZE = 176;     
  { --------------------------- Connection Pool Attributes ------------------  }
     OCI_ATTR_CONN_NOWAIT = 178;     
     OCI_ATTR_CONN_BUSY_COUNT = 179;     
     OCI_ATTR_CONN_OPEN_COUNT = 180;     
     OCI_ATTR_CONN_TIMEOUT = 181;     
     OCI_ATTR_STMT_STATE = 182;     
     OCI_ATTR_CONN_MIN = 183;     
     OCI_ATTR_CONN_MAX = 184;     
     OCI_ATTR_CONN_INCR = 185;     
  { For value 187, see DirPathAPI attribute section in this file  }
  { open stmts in session  }
     OCI_ATTR_NUM_OPEN_STMTS = 188;     
  { get native info via desc  }
     OCI_ATTR_DESCRIBE_NATIVE = 189;     
  { number of bind postions  }
     OCI_ATTR_BIND_COUNT = 190;     
  { pos of bind/define handle  }
     OCI_ATTR_HANDLE_POSITION = 191;     
  { reserverd  }
     OCI_ATTR_RESERVED_5 = 192;     
  { call in progress on server }
     OCI_ATTR_SERVER_BUSY = 193;     
  { For value 194, see DirPathAPI attribute section in this file  }
  { notification presentation for recipient  }
     OCI_ATTR_SUBSCR_RECPTPRES = 195;     
  { AQ message transformation  }
     OCI_ATTR_TRANSFORMATION = 196;     
  { rows fetched in last call  }
     OCI_ATTR_ROWS_FETCHED = 197;     
  { --------------------------- Snapshot attributes -------------------------  }
  { snapshot base  }
     OCI_ATTR_SCN_BASE = 198;     
  { snapshot wrap  }
     OCI_ATTR_SCN_WRAP = 199;     
  { --------------------------- Miscellanous attributes ---------------------  }
  { reserved  }
     OCI_ATTR_RESERVED_6 = 200;     
  { txn is readonly  }
     OCI_ATTR_READONLY_TXN = 201;     
  { reserved  }
     OCI_ATTR_RESERVED_7 = 202;     
  { position of erroneous col  }
     OCI_ATTR_ERRONEOUS_COLUMN = 203;     
  { reserved  }
     OCI_ATTR_RESERVED_8 = 204;     
  { For value 206, see DirPathAPI attribute section in this file  }
  { oracle instance type  }
     OCI_ATTR_INST_TYPE = 207;     
  {*****USED attribute 208 for  OCI_ATTR_SPOOL_STMTCACHESIZE****************** }
  { is env in utf16 mode?  }
     OCI_ATTR_ENV_UTF16 = 209;     
  { reserved for TMZ  }
     OCI_ATTR_RESERVED_9 = 210;     
  { reserved  }
     OCI_ATTR_RESERVED_10 = 211;     
  { For values 212 and 213, see DirPathAPI attribute section in this file  }
  { reserved  }
     OCI_ATTR_RESERVED_12 = 214;     
  { reserved  }
     OCI_ATTR_RESERVED_13 = 215;     
  { whether table is external  }
     OCI_ATTR_IS_EXTERNAL = 216;     
  { -------------------------- Statement Handle Attributes ------------------  }
  { reserved  }
     OCI_ATTR_RESERVED_15 = 217;     
  { stmt has returning clause  }
     OCI_ATTR_STMT_IS_RETURNING = 218;     
  { reserved  }
     OCI_ATTR_RESERVED_16 = 219;     
  { reserved  }
     OCI_ATTR_RESERVED_17 = 220;     
  { reserved  }
     OCI_ATTR_RESERVED_18 = 221;     
  { --------------------------- session attributes --------------------------- }
  { reserved  }
     OCI_ATTR_RESERVED_19 = 222;     
  { reserved  }
     OCI_ATTR_RESERVED_20 = 223;     
  { Current Schema  }
     OCI_ATTR_CURRENT_SCHEMA = 224;     
  { reserved  }
     OCI_ATTR_RESERVED_21 = 415;     
  { ------------------------- notification subscription ---------------------- }
  { QOS flags  }
     OCI_ATTR_SUBSCR_QOSFLAGS = 225;     
  { Payload callback  }
     OCI_ATTR_SUBSCR_PAYLOADCBK = 226;     
  { Timeout  }
     OCI_ATTR_SUBSCR_TIMEOUT = 227;     
  { Namespace context  }
     OCI_ATTR_SUBSCR_NAMESPACE_CTX = 228;     
  { ----------------------- row callback attributes -------------------------  }
  { bind row callback  }
     OCI_ATTR_BIND_ROWCBK = 301;     
  { ctx for bind row callback  }
     OCI_ATTR_BIND_ROWCTX = 302;     
  { skip buffer in array ops  }
     OCI_ATTR_SKIP_BUFFER = 303;     
  { ------------- DB Change Notification reg handle attributes --------------- }
  { out: array of table names      }
     OCI_ATTR_CHNF_TABLENAMES = 401;     
  { in: rowids needed  }     OCI_ATTR_CHNF_ROWIDS = 402;     
  { in: notification operation filter }     OCI_ATTR_CHNF_OPERATIONS = 403;     
  { txn lag between notifications   }
     OCI_ATTR_CHNF_CHANGELAG = 404;     
  { DB Change: Notification Descriptor attributes ----------------------- }
  { source database     }
     OCI_ATTR_CHDES_DBNAME = 405;     
  { notification type flags  }
     OCI_ATTR_CHDES_NFYTYPE = 406;     
  { XID  of the transaction  }
     OCI_ATTR_CHDES_XID = 407;     
  { array of table chg descriptors  }
     OCI_ATTR_CHDES_TABLE_CHANGES = 408;     
  { table name  }
     OCI_ATTR_CHDES_TABLE_NAME = 409;     
  { table operation flags  }
     OCI_ATTR_CHDES_TABLE_OPFLAGS = 410;     
  { array of changed rows    }
     OCI_ATTR_CHDES_TABLE_ROW_CHANGES = 411;     
  { rowid of changed row     }
     OCI_ATTR_CHDES_ROW_ROWID = 412;     
  { row operation flags      }
     OCI_ATTR_CHDES_ROW_OPFLAGS = 413;     
  { Statement handle attribute for db change notification  }
  { IN: subscription handle   }
     OCI_ATTR_CHNF_REGHANDLE = 414;     
  { client name for single session proxy  }
     OCI_ATTR_PROXY_CLIENT = 416;     
  { 415 is already taken - see OCI_ATTR_RESERVED_21  }
  { TDE attributes on the Table  }
  { does table have any encrypt columns  }
     OCI_ATTR_TABLE_ENC = 417;     
  { Table encryption Algorithm  }
     OCI_ATTR_TABLE_ENC_ALG = 418;     
  { Internal Id of encryption Algorithm }
     OCI_ATTR_TABLE_ENC_ALG_ID = 419;     
  { Please use from 143  }
  { DB Change: Event types --------------- }
  { None  }
     OCI_EVENT_NONE = $0;     
  { Startup database  }
     OCI_EVENT_STARTUP = $1;     
  { Shutdown database  }
     OCI_EVENT_SHUTDOWN = $2;     
  { Startup instance  }
     OCI_EVENT_SHUTDOWN_ANY = $3;     
  { Drop database     }
     OCI_EVENT_DROP_DB = $4;     
  { Subscription deregistered  }
     OCI_EVENT_DEREG = $5;     
  { Object change notification  }
     OCI_EVENT_OBJCHANGE = $6;     
  { DB Change: Operation types ----------- }
  { all rows invalidated   }
     OCI_OPCODE_ALLROWS = $1;     
  { interested in all operations  }
     OCI_OPCODE_ALLOPS = $0;     
  {  INSERT  }
     OCI_OPCODE_INSERT = $2;     
  {  UPDATE  }
     OCI_OPCODE_UPDATE = $4;     
  { DELETE  }
     OCI_OPCODE_DELETE = $8;     
  { ALTER  }
     OCI_OPCODE_ALTER = $10;     
  { DROP TABLE  }
     OCI_OPCODE_DROP = $20;     
  { GENERIC/ UNKNOWN }
     OCI_OPCODE_UNKNOWN = $40;     
  { -------- client side character and national character set ids -----------  }
  { charset id in env  }
     OCI_ATTR_ENV_CHARSET_ID = OCI_ATTR_CHARSET_ID;     
  { ncharset id in env  }
     OCI_ATTR_ENV_NCHARSET_ID = OCI_ATTR_NCHARSET_ID;
  { ----------------------- ha event callback attributes --------------------  }
  { ha callback  }
     OCI_ATTR_EVTCBK = 304;     
  { ctx for ha callback  }
     OCI_ATTR_EVTCTX = 305;     
  { ------------------ User memory attributes (all handles) -----------------  }
  { pointer to user memory  }
     OCI_ATTR_USER_MEMORY = 306;     
  { ----------------- port no attribute in subscription handle  -------------  }
  { port no to listen         }
     OCI_ATTR_SUBSCR_PORTNO = 390;     
  {------------- Supported Values for protocol for recepient ----------------- }
  { oci  }
     OCI_SUBSCR_PROTO_OCI = 0;     
  { mail  }
     OCI_SUBSCR_PROTO_MAIL = 1;     
  { server  }
     OCI_SUBSCR_PROTO_SERVER = 2;     
  { http  }
     OCI_SUBSCR_PROTO_HTTP = 3;     
  { max current protocols  }
     OCI_SUBSCR_PROTO_MAX = 4;     
  {------------- Supported Values for presentation for recepient ------------- }
  { default  }
     OCI_SUBSCR_PRES_DEFAULT = 0;     
  { xml  }
     OCI_SUBSCR_PRES_XML = 1;     
  { max current presentations  }
     OCI_SUBSCR_PRES_MAX = 2;     
  {------------- Supported QOS values for notification registrations --------- }
  { reliable  }
     OCI_SUBSCR_QOS_RELIABLE = $01;     
  { payload delivery  }
     OCI_SUBSCR_QOS_PAYLOAD = $02;     
  { replicate to director  }
     OCI_SUBSCR_QOS_REPLICATE = $04;     
  { secure payload delivery  }
     OCI_SUBSCR_QOS_SECURE = $08;     
  { purge on first ntfn  }
     OCI_SUBSCR_QOS_PURGE_ON_NTFN = $10;     
  { multi instance callback  }
     OCI_SUBSCR_QOS_MULTICBK = $20;     
  { ----- Temporary attribute value for UCS2/UTF16 character set ID --------  }  { UCS2 charset ID  }
     OCI_UCS2ID = 1000;     
  { UTF16 charset ID  }
     OCI_UTF16ID = 1000;     
  {============================== End OCI Attribute Types ==================== }
  {---------------- Server Handle Attribute Values --------------------------- }
  { OCI_ATTR_SERVER_STATUS  }
     OCI_SERVER_NOT_CONNECTED = $0;     
     OCI_SERVER_NORMAL = $1;     
  {--------------------------------------------------------------------------- }
  {------------------------- Supported Namespaces  --------------------------- }
  { Anonymous Namespace  }
     OCI_SUBSCR_NAMESPACE_ANONYMOUS = 0;     
  { Advanced Queues  }
     OCI_SUBSCR_NAMESPACE_AQ = 1;     
  { change notification  }
     OCI_SUBSCR_NAMESPACE_DBCHANGE = 2;     
  { Max Name Space Number  }
     OCI_SUBSCR_NAMESPACE_MAX = 3;     
  {-------------------------Credential Types---------------------------------- }
  { database username/password  }
     OCI_CRED_RDBMS = 1;     
  { externally provided credentials  }
     OCI_CRED_EXT = 2;     
  { proxy authentication  }
     OCI_CRED_PROXY = 3;     
  { reserved  }
     OCI_CRED_RESERVED_1 = 4;     
  { reserved  }
     OCI_CRED_RESERVED_2 = 5;     
  {--------------------------------------------------------------------------- }
  {------------------------Error Return Values-------------------------------- }
  { maps to SQL_SUCCESS of SAG CLI  }
     OCI_SUCCESS = 0;     
  { maps to SQL_SUCCESS_WITH_INFO  }
     OCI_SUCCESS_WITH_INFO = 1;     
  { reserved  }     OCI_RESERVED_FOR_INT_USE = 200;     
  { maps to SQL_NO_DATA  }
     OCI_NO_DATA = 100;     
  { maps to SQL_ERROR  }
     OCI_ERROR = -(1);     
  { maps to SQL_INVALID_HANDLE  }
     OCI_INVALID_HANDLE = -(2);     
  { maps to SQL_NEED_DATA  }
     OCI_NEED_DATA = 99;     
  { OCI would block error  }
     OCI_STILL_EXECUTING = -(3123);     
  {--------------------------------------------------------------------------- }
  {--------------------- User Callback Return Values ------------------------- }
  { Continue with the body of the OCI function  }
     OCI_CONTINUE = -(24200);     
  { done with user row callback  }
     OCI_ROWCBK_DONE = -(24201);     
  {--------------------------------------------------------------------------- }
  {------------------DateTime and Interval check Error codes------------------ }
  { DateTime Error Codes used by OCIDateTimeCheck()  }
  { Bad day  }
     OCI_DT_INVALID_DAY = $1;     
  { Bad DAy Low/high bit (1=low) }
     OCI_DT_DAY_BELOW_VALID = $2;     
  {  Bad MOnth  }
     OCI_DT_INVALID_MONTH = $4;     
  { Bad MOnth Low/high bit (1=low)  }
     OCI_DT_MONTH_BELOW_VALID = $8;     
  { Bad YeaR  }
     OCI_DT_INVALID_YEAR = $10;     
  {  Bad YeaR Low/high bit (1=low)  }
     OCI_DT_YEAR_BELOW_VALID = $20;     
  {  Bad HouR  }
     OCI_DT_INVALID_HOUR = $40;     
  { Bad HouR Low/high bit (1=low)  }
     OCI_DT_HOUR_BELOW_VALID = $80;     
  { Bad MiNute  }
     OCI_DT_INVALID_MINUTE = $100;     
  {Bad MiNute Low/high bit (1=low)  }
     OCI_DT_MINUTE_BELOW_VALID = $200;     
  {  Bad SeCond  }
     OCI_DT_INVALID_SECOND = $400;     
  {bad second Low/high bit (1=low) }
     OCI_DT_SECOND_BELOW_VALID = $800;     
     OCI_DT_DAY_MISSING_FROM_1582 = $1000;     
  {  Day is one of those "missing" from 1582  }
  { Year may not equal zero  }
     OCI_DT_YEAR_ZERO = $2000;     
  {  Bad Timezone  }
     OCI_DT_INVALID_TIMEZONE = $4000;     
  { Bad date format input  }
     OCI_DT_INVALID_FORMAT = $8000;     
  { Interval Error Codes used by OCIInterCheck()  }
  { Bad day  }
     OCI_INTER_INVALID_DAY = $1;     
  { Bad DAy Low/high bit (1=low)  }
     OCI_INTER_DAY_BELOW_VALID = $2;     
  { Bad MOnth  }
     OCI_INTER_INVALID_MONTH = $4;     
  {Bad MOnth Low/high bit (1=low)  }
     OCI_INTER_MONTH_BELOW_VALID = $8;     
  { Bad YeaR  }
     OCI_INTER_INVALID_YEAR = $10;     
  {Bad YeaR Low/high bit (1=low)  }
     OCI_INTER_YEAR_BELOW_VALID = $20;     
  { Bad HouR  }
     OCI_INTER_INVALID_HOUR = $40;     
  {Bad HouR Low/high bit (1=low)  }
     OCI_INTER_HOUR_BELOW_VALID = $80;     
  { Bad MiNute  }
     OCI_INTER_INVALID_MINUTE = $100;     
     OCI_INTER_MINUTE_BELOW_VALID = $200;     
  {Bad MiNute Low/high bit(1=low)  }
  { Bad SeCond  }
     OCI_INTER_INVALID_SECOND = $400;     
     OCI_INTER_SECOND_BELOW_VALID = $800;     
  {bad second Low/high bit(1=low)  }
  { Bad Fractional second  }
     OCI_INTER_INVALID_FRACSEC = $1000;     
     OCI_INTER_FRACSEC_BELOW_VALID = $2000;     
  { Bad fractional second Low/High  }
  {------------------------Parsing Syntax Types------------------------------- }
  { V815 language - for backwards compatibility  }
     OCI_V7_SYNTAX = 2;     
  { V815 language - for backwards compatibility  }
     OCI_V8_SYNTAX = 3;     
  { Use what so ever is the native lang of server  }
     OCI_NTV_SYNTAX = 1;     
  { these values must match the values defined in kpul.h  }
  {--------------------------------------------------------------------------- }
  {------------------------(Scrollable Cursor) Fetch Options------------------- 
   * For non-scrollable cursor, the only valid (and default) orientation is 
   * OCI_FETCH_NEXT
    }
  { refetching current position   }
     OCI_FETCH_CURRENT = $00000001;     
  { next row  }
     OCI_FETCH_NEXT = $00000002;     
  { first row of the result set  }
     OCI_FETCH_FIRST = $00000004;     
  { the last row of the result set  }
     OCI_FETCH_LAST = $00000008;     
  { previous row relative to current  }
     OCI_FETCH_PRIOR = $00000010;     
  { absolute offset from first  }
     OCI_FETCH_ABSOLUTE = $00000020;     
  { offset relative to current  }
     OCI_FETCH_RELATIVE = $00000040;     
  { reserved  }
     OCI_FETCH_RESERVED_1 = $00000080;     
  { reserved  }
     OCI_FETCH_RESERVED_2 = $00000100;     
  { reserved  }
     OCI_FETCH_RESERVED_3 = $00000200;     
  { reserved  }
     OCI_FETCH_RESERVED_4 = $00000400;     
  { reserved  }
     OCI_FETCH_RESERVED_5 = $00000800;     
  {--------------------------------------------------------------------------- }
  {------------------------Bind and Define Options---------------------------- }
  { unused  }
     OCI_SB2_IND_PTR = $00000001;     
  { data at execute time  }
     OCI_DATA_AT_EXEC = $00000002;     
  { fetch dynamically  }
     OCI_DYNAMIC_FETCH = $00000002;     
  { piecewise DMLs or fetch  }
     OCI_PIECEWISE = $00000004;     
  { reserved  }
     OCI_DEFINE_RESERVED_1 = $00000008;     
  { reserved  }
     OCI_BIND_RESERVED_2 = $00000010;     
  { reserved  }
     OCI_DEFINE_RESERVED_2 = $00000020;     
  { soft bind or define  }
     OCI_BIND_SOFT = $00000040;     
  { soft bind or define  }
     OCI_DEFINE_SOFT = $00000080;     
  { reserved  }
     OCI_BIND_RESERVED_3 = $00000100;     
  {--------------------------------------------------------------------------- }
  {-----------------------------  Various Modes ------------------------------ }
     OCI_DEFAULT = $00000000;     
  { the default value for parameters and attributes  }
  {-------------OCIInitialize Modes / OCICreateEnvironment Modes ------------- }
  { appl. in threaded environment  }
     OCI_THREADED = $00000001;     
  { application in object environment  }
     OCI_OBJECT = $00000002;     
  { application is enabled for events  }
     OCI_EVENTS = $00000004;     
  { reserved  }
     OCI_RESERVED1 = $00000008;     
  { the application is in shared mode  }
     OCI_SHARED = $00000010;     
  { reserved  }
     OCI_RESERVED2 = $00000020;     
  { The following *TWO* are only valid for OCICreateEnvironment call  }
  { No user callback called during ini  }
     OCI_NO_UCB = $00000040;     
  { the environment handle will not be  }
     OCI_NO_MUTEX = $00000080;     
  {  protected by a mutex internally  }
  { Used for shared forms  }
     OCI_SHARED_EXT = $00000100;     
  {************************* 0x00000200 free ********************************* }
  { all connections always blocking  }
     OCI_ALWAYS_BLOCKING = $00000400;     
  {************************* 0x00000800 free ********************************* }
  { allow  LDAP connections  }
     OCI_USE_LDAP = $00001000;     
  { only register to LDAP  }
     OCI_REG_LDAPONLY = $00002000;     
  { mode for all UTF16 metadata  }
     OCI_UTF16 = $00004000;     
     OCI_AFC_PAD_ON = $00008000;     
  { turn on AFC blank padding when rlenp present  }
  { reserved  }
     OCI_ENVCR_RESERVED3 = $00010000;     
  { adopt new length semantics  }
     OCI_NEW_LENGTH_SEMANTICS = $00020000;     
  { the new length semantics, always bytes, is used by OCIEnvNlsCreate  }
  { Do not mutex stmt handle  }
     OCI_NO_MUTEX_STMT = $00040000;     
  { Mutex only the environment handle  }
     OCI_MUTEX_ENV_ONLY = $00080000;     
  { reserved  }
     OCI_STM_RESERVED4 = $00100000;     
  { try and acquire mutex  }
     OCI_MUTEX_TRY = $00200000;     
  { nchar literal replace on  }
     OCI_NCHAR_LITERAL_REPLACE_ON = $00400000;     
  { nchar literal replace off }
     OCI_NCHAR_LITERAL_REPLACE_OFF = $00800000;     
  { reserved  }
     OCI_SRVATCH_RESERVED5 = $01000000;     
  {--------------------------------------------------------------------------- }
  {------------------------OCIConnectionpoolCreate Modes---------------------- }
     OCI_CPOOL_REINITIALIZE = $111;     
  {--------------------------------------------------------------------------- }
  {--------------------------------- OCILogon2 Modes ------------------------- }
  { Use session pool  }
     OCI_LOGON2_SPOOL = $0001;     
  { Use connection pool  }
     OCI_LOGON2_CPOOL = OCI_CPOOL;     
  { Use Stmt Caching  }
     OCI_LOGON2_STMTCACHE = $0004;     
  { Proxy authentiaction  }
     OCI_LOGON2_PROXY = $0008;     
  {--------------------------------------------------------------------------- }
  {------------------------- OCISessionPoolCreate Modes ---------------------- }
  { Reinitialize the session pool  }
     OCI_SPC_REINITIALIZE = $0001;     
  { Session pool is homogeneneous  }
     OCI_SPC_HOMOGENEOUS = $0002;     
  { Session pool has stmt cache  }
     OCI_SPC_STMTCACHE = $0004;     
  {--------------------------------------------------------------------------- }
  {--------------------------- OCISessionGet Modes --------------------------- }
  { SessionGet called in SPOOL mode  }
     OCI_SESSGET_SPOOL = $0001;     
  { SessionGet called in CPOOL mode  }
     OCI_SESSGET_CPOOL = OCI_CPOOL;     
  { Use statement cache  }
     OCI_SESSGET_STMTCACHE = $0004;     
  { SessionGet called in proxy mode  }
     OCI_SESSGET_CREDPROXY = $0008;     
     OCI_SESSGET_CREDEXT = $0010;     
     OCI_SESSGET_SPOOL_MATCHANY = $0020;     
  {--------------------------------------------------------------------------- }
  {------------------------ATTR Values for Session Pool----------------------- }
  { Attribute values for OCI_ATTR_SPOOL_GETMODE  }
  { block till you get a session  }
     OCI_SPOOL_ATTRVAL_WAIT = 0;     
  { error out if no session avaliable  }
     OCI_SPOOL_ATTRVAL_NOWAIT = 1;     
  { get session even if max is exceeded  }
     OCI_SPOOL_ATTRVAL_FORCEGET = 2;     
  {--------------------------------------------------------------------------- }
  {--------------------------- OCISessionRelease Modes ----------------------- }
  { Drop the Session  }
     OCI_SESSRLS_DROPSESS = $0001;     
  { Retag the session  }
     OCI_SESSRLS_RETAG = $0002;     
  {--------------------------------------------------------------------------- }
  {----------------------- OCISessionPoolDestroy Modes ----------------------- }
  { Force the sessions to terminate. 
                                               Even if there are some busy 
                                               sessions close them  }
     OCI_SPD_FORCE = $0001;     
  {--------------------------------------------------------------------------- }
  {----------------------------- Statement States ---------------------------- }
     OCI_STMT_STATE_INITIALIZED = $0001;     
     OCI_STMT_STATE_EXECUTED = $0002;     
     OCI_STMT_STATE_END_OF_FETCH = $0003;     
  {--------------------------------------------------------------------------- }
  {----------------------------- OCIMemStats Modes --------------------------- }
     OCI_MEM_INIT = $01;     
     OCI_MEM_CLN = $02;     
     OCI_MEM_FLUSH = $04;     
     OCI_DUMP_HEAP = $80;     
     OCI_CLIENT_STATS = $10;     
     OCI_SERVER_STATS = $20;     
  {----------------------------- OCIEnvInit Modes ---------------------------- }
  { NOTE: NO NEW MODES SHOULD BE ADDED HERE BECAUSE THE RECOMMENDED METHOD 
   * IS TO USE THE NEW OCICreateEnvironment MODES.
    }
  { A user callback will not be called in
                                         OCIEnvInit()  }
     OCI_ENV_NO_UCB = $01;     
  { the environment handle will not be protected
                                   by a mutex internally  }
     OCI_ENV_NO_MUTEX = $08;     
  {--------------------------------------------------------------------------- }
  {------------------------ Prepare Modes ------------------------------------ }
  { turn off statement handle sharing  }
     OCI_NO_SHARING = $01;     
  { reserved  }
     OCI_PREP_RESERVED_1 = $02;     
  { turn on blank padding for AFC  }
     OCI_PREP_AFC_PAD_ON = $04;     
  { turn off blank padding for AFC  }
     OCI_PREP_AFC_PAD_OFF = $08;     
  {--------------------------------------------------------------------------- }
  {--------------------------------------------------------------------------- }
  {----------------------- Execution Modes ----------------------------------- }
  { batch the oci statement for execution  }
     OCI_BATCH_MODE = $01;     
  { fetch the exact rows specified  }
     OCI_EXACT_FETCH = $02;     
  { #define                    0x04                                 available  }
  { if result set is scrollable  }
     OCI_STMT_SCROLLABLE_READONLY = $08;     
  { only describe the statement  }
     OCI_DESCRIBE_ONLY = $10;     
  { commit, if successful execution  }
     OCI_COMMIT_ON_SUCCESS = $20;     
  { non-blocking  }
     OCI_NON_BLOCKING = $40;     
  { batch errors in array dmls  }
     OCI_BATCH_ERRORS = $80;     
  { only parse the statement  }
     OCI_PARSE_ONLY = $100;     
  { reserved  }
     OCI_EXACT_FETCH_RESERVED_1 = $200;     
     OCI_SHOW_DML_WARNINGS = $400;     
  { return OCI_SUCCESS_WITH_INFO for delete/update w/no where clause  }
  { reserved  }
     OCI_EXEC_RESERVED_2 = $800;     
  { reserved  }
     OCI_DESC_RESERVED_1 = $1000;     
  { reserved  }
     OCI_EXEC_RESERVED_3 = $2000;     
  { reserved  }
     OCI_EXEC_RESERVED_4 = $4000;     
  { reserved  }
     OCI_EXEC_RESERVED_5 = $8000;     
  {--------------------------------------------------------------------------- }
  {------------------------Authentication Modes------------------------------- }
  { migratable auth context  }
     OCI_MIGRATE = $00000001;     
  { for SYSDBA authorization  }
     OCI_SYSDBA = $00000002;     
  { for SYSOPER authorization  }
     OCI_SYSOPER = $00000004;     
  { for preliminary authorization  }
     OCI_PRELIM_AUTH = $00000008;     
  { Private OCI cache mode  }
     OCIP_ICACHE = $00000010;     
  { reserved  }
     OCI_AUTH_RESERVED_1 = $00000020;     
  { enable OCI Stmt Caching  }
     OCI_STMT_CACHE = $00000040;     
  { stateless at call boundary  }
     OCI_STATELESS_CALL = $00000080;     
  { stateless at txn boundary  }
     OCI_STATELESS_TXN = $00000100;     
  { stateless at user-specified pts  }
     OCI_STATELESS_APP = $00000200;     
  { reserved  }
     OCI_AUTH_RESERVED_2 = $00000400;     
  { reserved  }
     OCI_AUTH_RESERVED_3 = $00000800;     
  { reserved  }
     OCI_AUTH_RESERVED_4 = $00001000;     
  { reserved  }
     OCI_AUTH_RESERVED_5 = $00002000;     
  {--------------------------------------------------------------------------- }
  {------------------------Session End Modes---------------------------------- }
  { reserved  }
     OCI_SESSEND_RESERVED_1 = $0001;     
  { reserved  }
     OCI_SESSEND_RESERVED_2 = $0002;     
  {--------------------------------------------------------------------------- }
  {------------------------Attach Modes--------------------------------------- }
  { The following attach modes are the same as the UPI modes defined in 
   * UPIDEF.H.  Do not use these values externally.
    }
  { Attach in fast path mode  }
     OCI_FASTPATH = $0010;     
  { reserved  }
     OCI_ATCH_RESERVED_1 = $0020;     
  { reserved  }
     OCI_ATCH_RESERVED_2 = $0080;     
  { reserved  }
     OCI_ATCH_RESERVED_3 = $0100;     
  { reserved  }
     OCI_ATCH_RESERVED_4 = $0400;     
  { reserved  }
     OCI_ATCH_RESERVED_5 = $2000;     
  {---------------------OCIStmtPrepare2 Modes--------------------------------- }
  { ONly Search  }
     OCI_PREP2_CACHE_SEARCHONLY = $0010;     
  { Get PL/SQL warnings   }
     OCI_PREP2_GET_PLSQL_WARNINGS = $0020;     
  { reserved  }
     OCI_PREP2_RESERVED_1 = $0040;     
  {---------------------OCIStmtRelease Modes---------------------------------- }
  { Delete from Cache  }
     OCI_STRLS_CACHE_DELETE = $0010;     
  {-----------------------------End Various Modes ---------------------------- }
  {------------------------Piece Information---------------------------------- }
  { in parameter  }
     OCI_PARAM_IN = $01;     
  { out parameter  }
     OCI_PARAM_OUT = $02;     
  {--------------------------------------------------------------------------- }
  {------------------------ Transaction Start Flags -------------------------- }
  { NOTE: OCI_TRANS_JOIN and OCI_TRANS_NOMIGRATE not supported in 8.0.X        }
  { starts a new transaction branch  }
     OCI_TRANS_NEW = $00000001;     
  { join an existing transaction  }
     OCI_TRANS_JOIN = $00000002;     
  { resume this transaction  }
     OCI_TRANS_RESUME = $00000004;     
     OCI_TRANS_STARTMASK = $000000ff;     
  { starts a readonly transaction  }
     OCI_TRANS_READONLY = $00000100;     
  { starts a read-write transaction  }
     OCI_TRANS_READWRITE = $00000200;     
     OCI_TRANS_SERIALIZABLE = $00000400;     
  { starts a serializable transaction  }
     OCI_TRANS_ISOLMASK = $0000ff00;     
  { a loosely coupled branch  }
     OCI_TRANS_LOOSE = $00010000;     
  { a tightly coupled branch  }
     OCI_TRANS_TIGHT = $00020000;     
     OCI_TRANS_TYPEMASK = $000f0000;     
  { non migratable transaction  }
     OCI_TRANS_NOMIGRATE = $00100000;     
  { separable transaction (8.1.6+)  }
     OCI_TRANS_SEPARABLE = $00200000;     
  { OTS resuming a transaction  }
     OCI_TRANS_OTSRESUME = $00400000;     
  {--------------------------------------------------------------------------- }
  {------------------------ Transaction End Flags ---------------------------- }
  { use two phase commit  }
     OCI_TRANS_TWOPHASE = $01000000;     
  { force cmt-redo for local txns  }
     OCI_TRANS_WRITEBATCH = $00000001;     
  { no force cmt-redo  }
     OCI_TRANS_WRITEIMMED = $00000002;     
  { no sync cmt-redo  }
     OCI_TRANS_WRITEWAIT = $00000004;     
  { sync cmt-redo for local txns  }
     OCI_TRANS_WRITENOWAIT = $00000008;     
  {--------------------------------------------------------------------------- }
  {------------------------- AQ Constants ------------------------------------
   * NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE
   * The following constants must match the PL/SQL dbms_aq constants
   * NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE NOTE
    }
  { ------------------------- Visibility flags ------------------------------- }
  { enqueue is an independent transaction  }
     OCI_ENQ_IMMEDIATE = 1;     
  { enqueue is part of current transaction  }
     OCI_ENQ_ON_COMMIT = 2;     
  { ----------------------- Dequeue mode flags ------------------------------- }
  { read message without acquiring a lock  }
     OCI_DEQ_BROWSE = 1;     
  { read and obtain write lock on message  }
     OCI_DEQ_LOCKED = 2;     
  { read the message and delete it  }
     OCI_DEQ_REMOVE = 3;     
  { delete message w'o returning payload  }
     OCI_DEQ_REMOVE_NODATA = 4;     
  { get signature only  }
     OCI_DEQ_GETSIG = 5;     
  { ----------------- Dequeue navigation flags ------------------------------- }
  { get first message at head of queue  }
     OCI_DEQ_FIRST_MSG = 1;     
  { next message that is available  }
     OCI_DEQ_NEXT_MSG = 3;     
  { get first message of next txn group  }
     OCI_DEQ_NEXT_TRANSACTION = 2;     
  { array dequeue across txn groups  }
     OCI_DEQ_MULT_TRANSACTION = 5;     
  { ----------------- Dequeue Option Reserved flags -------------------------  }
     OCI_DEQ_RESERVED_1 = $000001;     
  { --------------------- Message states ------------------------------------- }
  { the message delay has not yet completed  }
     OCI_MSG_WAITING = 1;     
  { the message is ready to be processed  }
     OCI_MSG_READY = 0;     
  { the message has been processed  }
     OCI_MSG_PROCESSED = 2;     
  { message has moved to exception queue  }
     OCI_MSG_EXPIRED = 3;     
  { --------------------- Sequence deviation --------------------------------- }
  { enqueue message before another message  }
     OCI_ENQ_BEFORE = 2;     
  { enqueue message before all messages  }
     OCI_ENQ_TOP = 3;     
  { ------------------------- Visibility flags ------------------------------- }
  { dequeue is an independent transaction  }
     OCI_DEQ_IMMEDIATE = 1;     
  { dequeue is part of current transaction  }
     OCI_DEQ_ON_COMMIT = 2;     
  { ------------------------ Wait -------------------------------------------- }
  { wait forever if no message available  }
     OCI_DEQ_WAIT_FOREVER = -(1);     
  { do not wait if no message is available  }
     OCI_DEQ_NO_WAIT = 0;     
  { ------------------------ Delay ------------------------------------------- }
  { message is available immediately  }
     OCI_MSG_NO_DELAY = 0;     
  { ------------------------- Expiration ------------------------------------- }
  { message will never expire  }
     OCI_MSG_NO_EXPIRATION = -(1);     
     OCI_MSG_PERSISTENT_OR_BUFFERED = 3;     
     OCI_MSG_BUFFERED = 2;     
     OCI_MSG_PERSISTENT = 1;     
  { ------------------------- Reserved --------------------------------------- }
     OCI_AQ_RESERVED_1 = $0002;     
     OCI_AQ_RESERVED_2 = $0004;     
     OCI_AQ_RESERVED_3 = $0008;     
     OCI_AQ_RESERVED_4 = $0010;     
  { -------------------------- END AQ Constants -----------------------------  }
  { --------------------END DateTime and Interval Constants ------------------ }
  {-----------------------Object Types---------------------------------------- }
  {-----------Object Types **** Not to be Used **** -------------------------- }
  { Deprecated  }
     OCI_OTYPE_UNK = 0;     
     OCI_OTYPE_TABLE = 1;     
     OCI_OTYPE_VIEW = 2;     
     OCI_OTYPE_SYN = 3;     
     OCI_OTYPE_PROC = 4;     
     OCI_OTYPE_FUNC = 5;     
     OCI_OTYPE_PKG = 6;     
     OCI_OTYPE_STMT = 7;     
  {--------------------------------------------------------------------------- }
  {=======================Describe Handle Parameter Attributes =============== }
  { 
     These attributes are orthogonal to the other set of attributes defined 
     above.  These attrubutes are to be used only for the describe handle. 
   }
  {=========================================================================== }
  { Attributes common to Columns and Stored Procs  }
  { maximum size of the data  }
     OCI_ATTR_DATA_SIZE = 1;     
  { the SQL type of the column/argument  }
     OCI_ATTR_DATA_TYPE = 2;     
  { the display size  }
     OCI_ATTR_DISP_SIZE = 3;     
  { the name of the column/argument  }
     OCI_ATTR_NAME = 4;     
  { precision if number type  }
     OCI_ATTR_PRECISION = 5;     
  { scale if number type  }
     OCI_ATTR_SCALE = 6;     
  { is it null ?  }
     OCI_ATTR_IS_NULL = 7;     
     OCI_ATTR_TYPE_NAME = 8;     
  { name of the named data type or a package name for package private types  }
  { the schema name  }
     OCI_ATTR_SCHEMA_NAME = 9;     
  { type name if package private type  }
     OCI_ATTR_SUB_NAME = 10;     
     OCI_ATTR_POSITION = 11;     
  { relative position of col/arg in the list of cols/args  }
  { complex object retrieval parameter attributes  }
     OCI_ATTR_COMPLEXOBJECTCOMP_TYPE = 50;     
     OCI_ATTR_COMPLEXOBJECTCOMP_TYPE_LEVEL = 51;     
     OCI_ATTR_COMPLEXOBJECT_LEVEL = 52;     
     OCI_ATTR_COMPLEXOBJECT_COLL_OUTOFLINE = 53;     
  { Only Columns  }
  { the display name  }
     OCI_ATTR_DISP_NAME = 100;     
  { encrypted data size  }
     OCI_ATTR_ENCC_SIZE = 101;     
  { column is encrypted ?  }
     OCI_ATTR_COL_ENC = 102;     
  { is encrypted column salted ?  }
     OCI_ATTR_COL_ENC_SALT = 103;     
  {Only Stored Procs  }
  { is this position overloaded  }
     OCI_ATTR_OVERLOAD = 210;     
  { level for structured types  }
     OCI_ATTR_LEVEL = 211;     
  { has a default value  }
     OCI_ATTR_HAS_DEFAULT = 212;     
  { in, out inout  }
     OCI_ATTR_IOMODE = 213;     
  { returns a radix  }
     OCI_ATTR_RADIX = 214;     
  { total number of arguments  }
     OCI_ATTR_NUM_ARGS = 215;     
  { only named type attributes  }
  { object or collection  }
     OCI_ATTR_TYPECODE = 216;     
  { varray or nested table  }
     OCI_ATTR_COLLECTION_TYPECODE = 217;     
  { user assigned version  }
     OCI_ATTR_VERSION = 218;     
  { is this an incomplete type  }
     OCI_ATTR_IS_INCOMPLETE_TYPE = 219;     
  { a system type  }
     OCI_ATTR_IS_SYSTEM_TYPE = 220;     
  { a predefined type  }
     OCI_ATTR_IS_PREDEFINED_TYPE = 221;     
  { a transient type  }
     OCI_ATTR_IS_TRANSIENT_TYPE = 222;     
  { system generated type  }
     OCI_ATTR_IS_SYSTEM_GENERATED_TYPE = 223;     
  { contains nested table attr  }
     OCI_ATTR_HAS_NESTED_TABLE = 224;     
  { has a lob attribute  }
     OCI_ATTR_HAS_LOB = 225;     
  { has a file attribute  }
     OCI_ATTR_HAS_FILE = 226;     
  { has a collection attribute  }
     OCI_ATTR_COLLECTION_ELEMENT = 227;     
  { number of attribute types  }
     OCI_ATTR_NUM_TYPE_ATTRS = 228;     
  { list of type attributes  }
     OCI_ATTR_LIST_TYPE_ATTRS = 229;     
  { number of type methods  }
     OCI_ATTR_NUM_TYPE_METHODS = 230;     
  { list of type methods  }
     OCI_ATTR_LIST_TYPE_METHODS = 231;     
  { map method of type  }
     OCI_ATTR_MAP_METHOD = 232;     
  { order method of type  }
     OCI_ATTR_ORDER_METHOD = 233;     
  { only collection element  }
  { number of elements  }
     OCI_ATTR_NUM_ELEMS = 234;     
  { only type methods  }
  { encapsulation level  }
     OCI_ATTR_ENCAPSULATION = 235;     
  { method selfish  }
     OCI_ATTR_IS_SELFISH = 236;     
  { virtual  }
     OCI_ATTR_IS_VIRTUAL = 237;     
  { inline  }
     OCI_ATTR_IS_INLINE = 238;     
  { constant  }
     OCI_ATTR_IS_CONSTANT = 239;     
  { has result  }
     OCI_ATTR_HAS_RESULT = 240;     
  { constructor  }
     OCI_ATTR_IS_CONSTRUCTOR = 241;     
  { destructor  }
     OCI_ATTR_IS_DESTRUCTOR = 242;     
  { operator  }
     OCI_ATTR_IS_OPERATOR = 243;     
  { a map method  }
     OCI_ATTR_IS_MAP = 244;     
  { order method  }
     OCI_ATTR_IS_ORDER = 245;     
  { read no data state method  }
     OCI_ATTR_IS_RNDS = 246;     
  { read no process state  }
     OCI_ATTR_IS_RNPS = 247;     
  { write no data state method  }
     OCI_ATTR_IS_WNDS = 248;     
  { write no process state  }
     OCI_ATTR_IS_WNPS = 249;     
  { public object  }
     OCI_ATTR_DESC_PUBLIC = 250;     
  { Object Cache Enhancements : attributes for User Constructed Instances      }
     OCI_ATTR_CACHE_CLIENT_CONTEXT = 251;     
     OCI_ATTR_UCI_CONSTRUCT = 252;     
     OCI_ATTR_UCI_DESTRUCT = 253;     
     OCI_ATTR_UCI_COPY = 254;     
     OCI_ATTR_UCI_PICKLE = 255;     
     OCI_ATTR_UCI_UNPICKLE = 256;     
     OCI_ATTR_UCI_REFRESH = 257;     
  { for type inheritance  }
     OCI_ATTR_IS_SUBTYPE = 258;     
     OCI_ATTR_SUPERTYPE_SCHEMA_NAME = 259;     
     OCI_ATTR_SUPERTYPE_NAME = 260;     
  { for schemas  }
  { list of objects in schema  }
     OCI_ATTR_LIST_OBJECTS = 261;     
  { for database  }
  { list of schemas  }
     OCI_ATTR_LIST_SCHEMAS = 263;     
  { max procedure length  }
     OCI_ATTR_MAX_PROC_LEN = 264;     
  { max column name length  }
     OCI_ATTR_MAX_COLUMN_LEN = 265;     
  { cursor commit behavior  }
     OCI_ATTR_CURSOR_COMMIT_BEHAVIOR = 266;     
  { catalog namelength  }
     OCI_ATTR_MAX_CATALOG_NAMELEN = 267;     
  { catalog location  }
     OCI_ATTR_CATALOG_LOCATION = 268;     
  { savepoint support  }
     OCI_ATTR_SAVEPOINT_SUPPORT = 269;     
  { nowait support  }
     OCI_ATTR_NOWAIT_SUPPORT = 270;     
  { autocommit DDL  }
     OCI_ATTR_AUTOCOMMIT_DDL = 271;     
  { locking mode  }
     OCI_ATTR_LOCKING_MODE = 272;     
  { for externally initialized context  }
  { count of context to be init }
     OCI_ATTR_APPCTX_SIZE = 273;     
  { count of context to be init }
     OCI_ATTR_APPCTX_LIST = 274;     
  { name  of context to be init }
     OCI_ATTR_APPCTX_NAME = 275;     
  { attr  of context to be init }
     OCI_ATTR_APPCTX_ATTR = 276;     
  { value of context to be init }
     OCI_ATTR_APPCTX_VALUE = 277;     
  { for client id propagation  }
  { value of client id to set }
     OCI_ATTR_CLIENT_IDENTIFIER = 278;     
  { for inheritance - part 2  }
  { is final type ?  }
     OCI_ATTR_IS_FINAL_TYPE = 279;     
  { is instantiable type ?  }
     OCI_ATTR_IS_INSTANTIABLE_TYPE = 280;     
  { is final method ?  }
     OCI_ATTR_IS_FINAL_METHOD = 281;     
  { is instantiable method ?  }
     OCI_ATTR_IS_INSTANTIABLE_METHOD = 282;     
  { is overriding method ?  }
     OCI_ATTR_IS_OVERRIDING_METHOD = 283;     
  { Describe the base object  }
     OCI_ATTR_DESC_SYNBASE = 284;     
  { char length semantics  }
     OCI_ATTR_CHAR_USED = 285;     
  { char length  }
     OCI_ATTR_CHAR_SIZE = 286;     
  { SQLJ support  }
  { is java implemented type ?  }
     OCI_ATTR_IS_JAVA_TYPE = 287;     
  { N-Tier support  }
  { use DN as user name  }
     OCI_ATTR_DISTINGUISHED_NAME = 300;     
  { Kerberos ticket as cred.  }
     OCI_ATTR_KERBEROS_TICKET = 301;     
  { for multilanguage debugging  }
  { ORA_DEBUG_JDWP attribute  }
     OCI_ATTR_ORA_DEBUG_JDWP = 302;     
  { reserved  }
     OCI_ATTR_RESERVED_14 = 303;     
  {---------------------------End Describe Handle Attributes ----------------- }
  { For values 303 - 307, see DirPathAPI attribute section in this file  }
  { ----------------------- Session Pool Attributes -------------------------  }
  { session timeout  }
     OCI_ATTR_SPOOL_TIMEOUT = 308;     
  { session get mode  }
     OCI_ATTR_SPOOL_GETMODE = 309;     
  { busy session count  }
     OCI_ATTR_SPOOL_BUSY_COUNT = 310;     
  { open session count  }
     OCI_ATTR_SPOOL_OPEN_COUNT = 311;     
  { min session count  }
     OCI_ATTR_SPOOL_MIN = 312;     
  { max session count  }
     OCI_ATTR_SPOOL_MAX = 313;     
  { session increment count  }
     OCI_ATTR_SPOOL_INCR = 314;     
  {Stmt cache size of pool   }
     OCI_ATTR_SPOOL_STMTCACHESIZE = 208;     
  {------------------------------End Session Pool Attributes ----------------- }
  {---------------------------- For XML Types -------------------------------  }
  { For table, view and column  }
  { Is the type an XML type?  }
     OCI_ATTR_IS_XMLTYPE = 315;     
  { Name of XML Schema  }
     OCI_ATTR_XMLSCHEMA_NAME = 316;     
  { Name of XML Element  }
     OCI_ATTR_XMLELEMENT_NAME = 317;     
  { SQL type's schema for XML Ele  }
     OCI_ATTR_XMLSQLTYPSCH_NAME = 318;     
  { Name of SQL type for XML Ele  }
     OCI_ATTR_XMLSQLTYPE_NAME = 319;     
  { XML type stored as object?  }
     OCI_ATTR_XMLTYPE_STORED_OBJ = 320;     
  {---------------------------- For Subtypes -------------------------------  }
  { For type  }
  { Has subtypes?  }
     OCI_ATTR_HAS_SUBTYPES = 321;     
  { Number of subtypes  }
     OCI_ATTR_NUM_SUBTYPES = 322;     
  { List of subtypes  }
     OCI_ATTR_LIST_SUBTYPES = 323;     
  { XML flag  }
  { hierarchy enabled?  }
     OCI_ATTR_XML_HRCHY_ENABLED = 324;     
  { Method flag  }
  { Method is overridden?  }
     OCI_ATTR_IS_OVERRIDDEN_METHOD = 325;     
  { For values 326 - 335, see DirPathAPI attribute section in this file  }
  {------------- Attributes for 10i Distributed Objects ---------------------- }
  { obj col/tab substitutable  }
     OCI_ATTR_OBJ_SUBS = 336;     
  { For values 337 - 338, see DirPathAPI attribute section in this file  }
  {---------- Attributes for 10i XADFIELD (NLS language, territory ----------- }
  { reserved  }
     OCI_ATTR_XADFIELD_RESERVED_1 = 339;     
  { reserved  }
     OCI_ATTR_XADFIELD_RESERVED_2 = 340;     
  {------------- Kerberos Secure Client Identifier --------------------------- }
  { Kerberos db service ticket }
     OCI_ATTR_KERBEROS_CID = 341;     
  {------------------------ Attributes for Rules objects --------------------- }
  { rule condition  }
     OCI_ATTR_CONDITION = 342;     
  { comment  }
     OCI_ATTR_COMMENT = 343;     
  { Anydata value  }
     OCI_ATTR_VALUE = 344;     
  { eval context owner  }
     OCI_ATTR_EVAL_CONTEXT_OWNER = 345;     
  { eval context name  }
     OCI_ATTR_EVAL_CONTEXT_NAME = 346;     
  { eval function name  }
     OCI_ATTR_EVALUATION_FUNCTION = 347;     
  { variable type  }
     OCI_ATTR_VAR_TYPE = 348;     
  { variable value function  }
     OCI_ATTR_VAR_VALUE_FUNCTION = 349;     
  { variable method function  }
     OCI_ATTR_VAR_METHOD_FUNCTION = 350;     
  { action context  }
     OCI_ATTR_ACTION_CONTEXT = 351;     
  { list of table aliases  }
     OCI_ATTR_LIST_TABLE_ALIASES = 352;     
  { list of variable types  }
     OCI_ATTR_LIST_VARIABLE_TYPES = 353;     
  { table name  }
     OCI_ATTR_TABLE_NAME = 356;     
  { For values 357 - 359, see DirPathAPI attribute section in this file  }
  { message cscn  }
     OCI_ATTR_MESSAGE_CSCN = 360;     
  { message dscn  }
     OCI_ATTR_MESSAGE_DSCN = 361;     
  {--------------------- Audit Session ID ------------------------------------ }
  { Audit session ID  }
     OCI_ATTR_AUDIT_SESSION_ID = 362;     
  {--------------------- Kerberos TGT Keys ----------------------------------- }
  { n-tier Kerberos cred key  }
     OCI_ATTR_KERBEROS_KEY = 363;     
  { SCID Kerberos cred key  }
     OCI_ATTR_KERBEROS_CID_KEY = 364;     
  { AQ enq txn number  }
     OCI_ATTR_TRANSACTION_NO = 365;     
  {----------------------- Attributes for End To End Tracing ----------------- }
  { module for tracing  }
     OCI_ATTR_MODULE = 366;     
  { action for tracing  }
     OCI_ATTR_ACTION = 367;     
  { client info  }
     OCI_ATTR_CLIENT_INFO = 368;     
  { collect call time  }
     OCI_ATTR_COLLECT_CALL_TIME = 369;     
  { extract call time  }
     OCI_ATTR_CALL_TIME = 370;     
  { execution-id context  }
     OCI_ATTR_ECONTEXT_ID = 371;     
  {execution-id sequence num  }
     OCI_ATTR_ECONTEXT_SEQ = 372;     
  {------------------------------ Session attributes ------------------------- }
  { session state  }
     OCI_ATTR_SESSION_STATE = 373;     
  { valid states  }
     OCI_SESSION_STATELESS = 1;     
     OCI_SESSION_STATEFUL = 2;     
  { session state type  }
     OCI_ATTR_SESSION_STATETYPE = 374;     
  { valid state types  }
     OCI_SESSION_STATELESS_DEF = 0;     
     OCI_SESSION_STATELESS_CAL = 1;     
     OCI_SESSION_STATELESS_TXN = 2;     
     OCI_SESSION_STATELESS_APP = 3;     
  { session state cleared  }
     OCI_ATTR_SESSION_STATE_CLEARED = 376;     
  { did session migrate  }
     OCI_ATTR_SESSION_MIGRATED = 377;     
  { preserve session state  }
     OCI_ATTR_SESSION_PRESERVE_STATE = 388;     
  { -------------------------- Admin Handle Attributes ----------------------  }
  { client-side param file  }
     OCI_ATTR_ADMIN_PFILE = 389;     
  {----------------------- Attributes for End To End Tracing ----------------- }
  { -------------------------- HA Event Handle Attributes -------------------  }
  { SYS_CONTEXT hostname  }
     OCI_ATTR_HOSTNAME = 390;     
  { SYS_CONTEXT dbname  }
     OCI_ATTR_DBNAME = 391;     
  { SYS_CONTEXT instance name  }
     OCI_ATTR_INSTNAME = 392;     
  { SYS_CONTEXT service name  }
     OCI_ATTR_SERVICENAME = 393;     
  { v$instance instance start time  }
     OCI_ATTR_INSTSTARTTIME = 394;     
  { event time  }
     OCI_ATTR_HA_TIMESTAMP = 395;     
  { reserved  }
     OCI_ATTR_RESERVED_22 = 396;     
  { reserved  }
     OCI_ATTR_RESERVED_23 = 397;     
  { reserved  }
     OCI_ATTR_RESERVED_24 = 398;     
  { db domain  }
     OCI_ATTR_DBDOMAIN = 399;     
  { event type  }
     OCI_ATTR_EVENTTYPE = 400;     
  { valid value for OCI_ATTR_EVENTTYPE  }
     OCI_EVENTTYPE_HA = 0;     
     OCI_ATTR_HA_SOURCE = 401;     
  { valid values for OCI_ATTR_HA_SOURCE  }
     OCI_HA_SOURCE_INSTANCE = 0;     
     OCI_HA_SOURCE_DATABASE = 1;     
     OCI_HA_SOURCE_NODE = 2;     
     OCI_HA_SOURCE_SERVICE = 3;     
     OCI_HA_SOURCE_SERVICE_MEMBER = 4;     
     OCI_HA_SOURCE_ASM_INSTANCE = 5;     
     OCI_HA_SOURCE_SERVICE_PRECONNECT = 6;     
     OCI_ATTR_HA_STATUS = 402;     
  { valid values for OCI_ATTR_HA_STATUS  }
     OCI_HA_STATUS_DOWN = 0;     
     OCI_HA_STATUS_UP = 1;     
     OCI_ATTR_HA_SRVFIRST = 403;     
     OCI_ATTR_HA_SRVNEXT = 404;     
  { ------------------------- Server Handle Attributes ----------------------- }
     OCI_ATTR_TAF_ENABLED = 405;     
  { Extra notification attributes  }
     OCI_ATTR_NFY_FLAGS = 406;     
  { msg delivery mode  }
     OCI_ATTR_MSG_DELIVERY_MODE = 407;     
  { database charset ID  }
     OCI_ATTR_DB_CHARSET_ID = 416;     
  { database ncharset ID  }
     OCI_ATTR_DB_NCHARSET_ID = 417;     
  { reserved  }
     OCI_ATTR_RESERVED_25 = 418;     
  {--------------------------------------------------------------------------- }
  { ------------------DirPathAPI attribute Section---------------------------- }
  { All DirPathAPI attributes are in this section of the file.  Existing       }
  { attributes prior to this section being created are assigned values < 2000  }
  { Add new DirPathAPI attributes to this section and their assigned value     }
  { should be whatever the last entry is + 1.                                  }
  {------------- Supported Values for Direct Path Stream Version ------------- }
     OCI_DIRPATH_STREAM_VERSION_1 = 100;     
     OCI_DIRPATH_STREAM_VERSION_2 = 200;     
  { default  }
     OCI_DIRPATH_STREAM_VERSION_3 = 300;     
  { mode of direct path operation  }
     OCI_ATTR_DIRPATH_MODE = 78;     
  { nologging option  }
     OCI_ATTR_DIRPATH_NOLOG = 79;     
  { parallel (temp seg) option  }
     OCI_ATTR_DIRPATH_PARALLEL = 80;     
  { index that data is sorted on  }
     OCI_ATTR_DIRPATH_SORTED_INDEX = 137;     
  { direct path index maint method (see oci8dp.h)  }
     OCI_ATTR_DIRPATH_INDEX_MAINT_METHOD = 138;     
  { parallel load: db file, initial and next extent sizes  }
  { DB file to load into  }
     OCI_ATTR_DIRPATH_FILE = 139;     
  { initial extent size  }
     OCI_ATTR_DIRPATH_STORAGE_INITIAL = 140;     
  { next extent size  }
     OCI_ATTR_DIRPATH_STORAGE_NEXT = 141;     
  { direct path index maint method (see oci8dp.h)  }
     OCI_ATTR_DIRPATH_SKIPINDEX_METHOD = 145;     
  { 8.2 dpapi support of ADTs  }
  { expr type of OCI_ATTR_NAME  }
     OCI_ATTR_DIRPATH_EXPR_TYPE = 150;     
  { input in text or stream format  }
     OCI_ATTR_DIRPATH_INPUT = 151;     
     OCI_DIRPATH_INPUT_TEXT = $01;     
     OCI_DIRPATH_INPUT_STREAM = $02;     
     OCI_DIRPATH_INPUT_UNKNOWN = $04;     
  { fn ctx ADT attrs or args  }
     OCI_ATTR_DIRPATH_FN_CTX = 167;     
  { loading into an OID col  }
     OCI_ATTR_DIRPATH_OID = 187;     
  { loading into an SID col  }
     OCI_ATTR_DIRPATH_SID = 194;     
  { obj type of subst obj tbl  }
     OCI_ATTR_DIRPATH_OBJ_CONSTR = 206;     
  { Attr to allow setting of the stream version PRIOR to calling Prepare  }
  { version of the stream }
     OCI_ATTR_DIRPATH_STREAM_VERSION = 212;     
  { varray index column  }
     OCIP_ATTR_DIRPATH_VARRAY_INDEX = 213;     
  {------------- Supported Values for Direct Path Date cache ----------------- }
  { date cache entries  }
     OCI_ATTR_DIRPATH_DCACHE_NUM = 303;     
  { date cache limit  }
     OCI_ATTR_DIRPATH_DCACHE_SIZE = 304;     
  { date cache misses  }
     OCI_ATTR_DIRPATH_DCACHE_MISSES = 305;     
  { date cache hits  }
     OCI_ATTR_DIRPATH_DCACHE_HITS = 306;     
  { on set: disable datecache 
                                                  * on overflow.
                                                  * on get: datecache disabled? 
                                                  * could be due to overflow
                                                  * or others                   }
     OCI_ATTR_DIRPATH_DCACHE_DISABLE = 307;     
  {------------- Attributes for 10i Updates to the DirPath API --------------- }
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_7 = 326;     
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_8 = 327;     
  { stream conversion needed?  }
     OCI_ATTR_DIRPATH_CONVERT = 328;     
  { info about bad row  }
     OCI_ATTR_DIRPATH_BADROW = 329;     
  { length of bad row info  }
     OCI_ATTR_DIRPATH_BADROW_LENGTH = 330;     
  { column fill order  }
     OCI_ATTR_DIRPATH_WRITE_ORDER = 331;     
  { granule size for unload  }
     OCI_ATTR_DIRPATH_GRANULE_SIZE = 332;     
  { offset to last granule  }
     OCI_ATTR_DIRPATH_GRANULE_OFFSET = 333;     
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_1 = 334;     
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_2 = 335;     
  {------ Attributes for 10i DirPathAPI conversion (NLS lang, terr, cs) ------ }
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_3 = 337;     
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_4 = 338;     
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_5 = 357;     
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_6 = 358;     
  { wait for lock in dpapi  }
     OCI_ATTR_DIRPATH_LOCK_WAIT = 359;     
  { reserved  }
     OCI_ATTR_DIRPATH_RESERVED_9 = 2000;     
  {------ Attribute for 10iR2 for column encryption for Direct Path API ------ }
  { vector of functions  }
     OCI_ATTR_DIRPATH_RESERVED_10 = 2001;     
  { encryption metadata  }
     OCI_ATTR_DIRPATH_RESERVED_11 = 2002;     
  {------ Attribute to determine last column successfully converted ---------- }
  { current error column  }
     OCI_ATTR_CURRENT_ERRCOL = 2003;     
  { Add DirPathAPI attributes above.  Next value to be assigned is 2004        }
  { ------------------End of DirPathAPI attribute Section -------------------- }
  {--------------------------------------------------------------------------- }
  {---------------- Describe Handle Parameter Attribute Values --------------- }
  { OCI_ATTR_CURSOR_COMMIT_BEHAVIOR  }
     OCI_CURSOR_OPEN = 0;     
     OCI_CURSOR_CLOSED = 1;     
  { OCI_ATTR_CATALOG_LOCATION  }
     OCI_CL_START = 0;     
     OCI_CL_END = 1;     
  { OCI_ATTR_SAVEPOINT_SUPPORT  }
     OCI_SP_SUPPORTED = 0;     
     OCI_SP_UNSUPPORTED = 1;     
  { OCI_ATTR_NOWAIT_SUPPORT  }
     OCI_NW_SUPPORTED = 0;     
     OCI_NW_UNSUPPORTED = 1;     
  { OCI_ATTR_AUTOCOMMIT_DDL  }
     OCI_AC_DDL = 0;     
     OCI_NO_AC_DDL = 1;     
  { OCI_ATTR_LOCKING_MODE  }
     OCI_LOCK_IMMEDIATE = 0;     
     OCI_LOCK_DELAYED = 1;     
  { ------------------- Instance type attribute values ----------------------- }
     OCI_INSTANCE_TYPE_UNKNOWN = 0;     
     OCI_INSTANCE_TYPE_RDBMS = 1;     
     OCI_INSTANCE_TYPE_OSM = 2;     
  {--------------------------------------------------------------------------- }
  {---------------------------OCIPasswordChange------------------------------- }
  { Change the password but do not login  }
     OCI_AUTH = $08;     
  {------------------------Other Constants------------------------------------ }
  { max number of OCI Functions  }
     OCI_MAX_FNS = 100;     
     OCI_SQLSTATE_SIZE = 5;     
  { max size of an error message  }
     OCI_ERROR_MAXMSG_SIZE = 1024;     
  { maximum lob data size  }
     OCI_LOBMAXSIZE = 4294967295; // was MINUB4MAXVAL
     OCI_ROWID_LEN = 23;     
  {--------------------------------------------------------------------------- }
  {------------------------ Fail Over Events --------------------------------- }
     OCI_FO_END = $00000001;     
     OCI_FO_ABORT = $00000002;     
     OCI_FO_REAUTH = $00000004;     
     OCI_FO_BEGIN = $00000008;     
     OCI_FO_ERROR = $00000010;     
  {--------------------------------------------------------------------------- }
  {------------------------ Fail Over Callback Return Codes ------------------ }
     OCI_FO_RETRY = 25410;     
  {--------------------------------------------------------------------------- }
  {------------------------- Fail Over Types --------------------------------- }
     OCI_FO_NONE = $00000001;     
     OCI_FO_SESSION = $00000002;     
     OCI_FO_SELECT = $00000004;     
     OCI_FO_TXNAL = $00000008;     
  {--------------------------------------------------------------------------- }
  {-----------------------Function Codes-------------------------------------- }
  { OCIInitialize  }
     OCI_FNCODE_INITIALIZE = 1;     
  { OCIHandleAlloc  }
     OCI_FNCODE_HANDLEALLOC = 2;     
  { OCIHandleFree  }
     OCI_FNCODE_HANDLEFREE = 3;     
  { OCIDescriptorAlloc  }
     OCI_FNCODE_DESCRIPTORALLOC = 4;     
  { OCIDescriptorFree  }
     OCI_FNCODE_DESCRIPTORFREE = 5;     
  { OCIEnvInit  }
     OCI_FNCODE_ENVINIT = 6;     
  { OCIServerAttach  }
     OCI_FNCODE_SERVERATTACH = 7;     
  { OCIServerDetach  }
     OCI_FNCODE_SERVERDETACH = 8;     
  { unused         9  }  { OCISessionBegin  }
     OCI_FNCODE_SESSIONBEGIN = 10;     
  { OCISessionEnd  }
     OCI_FNCODE_SESSIONEND = 11;     
  { OCIPasswordChange  }
     OCI_FNCODE_PASSWORDCHANGE = 12;     
  { OCIStmtPrepare  }
     OCI_FNCODE_STMTPREPARE = 13;     
  { unused       14- 16  }
  { OCIBindDynamic  }
     OCI_FNCODE_BINDDYNAMIC = 17;     
  { OCIBindObject  }
     OCI_FNCODE_BINDOBJECT = 18;     
  { 19 unused  }
  { OCIBindArrayOfStruct  }
     OCI_FNCODE_BINDARRAYOFSTRUCT = 20;     
  { OCIStmtExecute  }
     OCI_FNCODE_STMTEXECUTE = 21;     
  { unused 22-24  }
  { OCIDefineObject  }
     OCI_FNCODE_DEFINEOBJECT = 25;     
  { OCIDefineDynamic  }
     OCI_FNCODE_DEFINEDYNAMIC = 26;     
  { OCIDefineArrayOfStruct  }
     OCI_FNCODE_DEFINEARRAYOFSTRUCT = 27;     
  { OCIStmtFetch  }
     OCI_FNCODE_STMTFETCH = 28;     
  { OCIStmtGetBindInfo  }
     OCI_FNCODE_STMTGETBIND = 29;     
  { 30, 31 unused  }
  { OCIDescribeAny  }
     OCI_FNCODE_DESCRIBEANY = 32;     
  { OCITransStart  }
     OCI_FNCODE_TRANSSTART = 33;     
  { OCITransDetach  }
     OCI_FNCODE_TRANSDETACH = 34;     
  { OCITransCommit  }
     OCI_FNCODE_TRANSCOMMIT = 35;     
  { 36 unused  }
  { OCIErrorGet  }
     OCI_FNCODE_ERRORGET = 37;     
  { OCILobFileOpen  }
     OCI_FNCODE_LOBOPENFILE = 38;     
  { OCILobFileClose  }
     OCI_FNCODE_LOBCLOSEFILE = 39;     
  { 40 was LOBCREATEFILE, unused  }
  { 41 was OCILobFileDelete, unused   }
  { OCILobCopy  }
     OCI_FNCODE_LOBCOPY = 42;     
  { OCILobAppend  }
     OCI_FNCODE_LOBAPPEND = 43;     
  { OCILobErase  }
     OCI_FNCODE_LOBERASE = 44;     
  { OCILobGetLength  }
     OCI_FNCODE_LOBLENGTH = 45;     
  { OCILobTrim  }
     OCI_FNCODE_LOBTRIM = 46;     
  { OCILobRead  }
     OCI_FNCODE_LOBREAD = 47;     
  { OCILobWrite  }
     OCI_FNCODE_LOBWRITE = 48;     
  { 49 unused  }
  { OCIBreak  }
     OCI_FNCODE_SVCCTXBREAK = 50;     
  { OCIServerVersion  }
     OCI_FNCODE_SERVERVERSION = 51;     
  { OCIKerbAttrSet  }
     OCI_FNCODE_KERBATTRSET = 52;     
  { unused 53  }
  { OCIAttrGet  }
     OCI_FNCODE_ATTRGET = 54;     
  { OCIAttrSet  }
     OCI_FNCODE_ATTRSET = 55;     
  { OCIParamSet  }
     OCI_FNCODE_PARAMSET = 56;     
  { OCIParamGet  }
     OCI_FNCODE_PARAMGET = 57;     
  { OCIStmtGetPieceInfo  }
     OCI_FNCODE_STMTGETPIECEINFO = 58;     
  { OCILdaToSvcCtx  }
     OCI_FNCODE_LDATOSVCCTX = 59;     
  { 60 unused  }
  { OCIStmtSetPieceInfo  }
     OCI_FNCODE_STMTSETPIECEINFO = 61;     
  { OCITransForget  }
     OCI_FNCODE_TRANSFORGET = 62;     
  { OCITransPrepare  }
     OCI_FNCODE_TRANSPREPARE = 63;     
  { OCITransRollback  }
     OCI_FNCODE_TRANSROLLBACK = 64;     
  { OCIDefineByPos  }
     OCI_FNCODE_DEFINEBYPOS = 65;     
  { OCIBindByPos  }
     OCI_FNCODE_BINDBYPOS = 66;     
  { OCIBindByName  }
     OCI_FNCODE_BINDBYNAME = 67;     
  { OCILobAssign  }
     OCI_FNCODE_LOBASSIGN = 68;     
  { OCILobIsEqual  }
     OCI_FNCODE_LOBISEQUAL = 69;     
  { OCILobLocatorIsInit  }
     OCI_FNCODE_LOBISINIT = 70;     
  { OCILobEnableBuffering  }
     OCI_FNCODE_LOBENABLEBUFFERING = 71;     
  { OCILobCharSetID  }
     OCI_FNCODE_LOBCHARSETID = 72;     
  { OCILobCharSetForm  }
     OCI_FNCODE_LOBCHARSETFORM = 73;     
  { OCILobFileSetName  }
     OCI_FNCODE_LOBFILESETNAME = 74;     
  { OCILobFileGetName  }
     OCI_FNCODE_LOBFILEGETNAME = 75;     
  { OCILogon  }
     OCI_FNCODE_LOGON = 76;     
  { OCILogoff  }
     OCI_FNCODE_LOGOFF = 77;     
  { OCILobDisableBuffering  }
     OCI_FNCODE_LOBDISABLEBUFFERING = 78;     
  { OCILobFlushBuffer  }
     OCI_FNCODE_LOBFLUSHBUFFER = 79;     
  { OCILobLoadFromFile  }
     OCI_FNCODE_LOBLOADFROMFILE = 80;     
  { OCILobOpen  }
     OCI_FNCODE_LOBOPEN = 81;     
  { OCILobClose  }
     OCI_FNCODE_LOBCLOSE = 82;     
  { OCILobIsOpen  }
     OCI_FNCODE_LOBISOPEN = 83;     
  { OCILobFileIsOpen  }
     OCI_FNCODE_LOBFILEISOPEN = 84;     
  { OCILobFileExists  }
     OCI_FNCODE_LOBFILEEXISTS = 85;     
  { OCILobFileCloseAll  }
     OCI_FNCODE_LOBFILECLOSEALL = 86;     
  { OCILobCreateTemporary  }
     OCI_FNCODE_LOBCREATETEMP = 87;     
  { OCILobFreeTemporary  }
     OCI_FNCODE_LOBFREETEMP = 88;     
  { OCILobIsTemporary  }
     OCI_FNCODE_LOBISTEMP = 89;     
  { OCIAQEnq  }
     OCI_FNCODE_AQENQ = 90;     
  { OCIAQDeq  }
     OCI_FNCODE_AQDEQ = 91;     
  { OCIReset  }
     OCI_FNCODE_RESET = 92;     
  { OCISvcCtxToLda  }
     OCI_FNCODE_SVCCTXTOLDA = 93;     
  { OCILobLocatorAssign  }
     OCI_FNCODE_LOBLOCATORASSIGN = 94;     
     OCI_FNCODE_UBINDBYNAME = 95;     
  { OCIAQListen  }
     OCI_FNCODE_AQLISTEN = 96;     
  { reserved  }
     OCI_FNCODE_SVC2HST = 97;     
  { reserved  }
     OCI_FNCODE_SVCRH = 98;     
  { 97 and 98 are reserved for Oracle internal use  }
  { OCITransMultiPrepare  }
     OCI_FNCODE_TRANSMULTIPREPARE = 99;     
  { OCIConnectionPoolCreate  }
     OCI_FNCODE_CPOOLCREATE = 100;     
  { OCIConnectionPoolDestroy  }
     OCI_FNCODE_CPOOLDESTROY = 101;     
  { OCILogon2  }
     OCI_FNCODE_LOGON2 = 102;     
  { OCIRowidToChar  }
     OCI_FNCODE_ROWIDTOCHAR = 103;     
  { OCISessionPoolCreate  }
     OCI_FNCODE_SPOOLCREATE = 104;     
  { OCISessionPoolDestroy  }
     OCI_FNCODE_SPOOLDESTROY = 105;     
  { OCISessionGet  }
     OCI_FNCODE_SESSIONGET = 106;     
  { OCISessionRelease  }
     OCI_FNCODE_SESSIONRELEASE = 107;     
  { OCIStmtPrepare2  }
     OCI_FNCODE_STMTPREPARE2 = 108;     
  { OCIStmtRelease  }
     OCI_FNCODE_STMTRELEASE = 109;     
  { OCIAQEnqArray  }
     OCI_FNCODE_AQENQARRAY = 110;     
  { OCIAQDeqArray  }
     OCI_FNCODE_AQDEQARRAY = 111;     
  { OCILobCopy2  }
     OCI_FNCODE_LOBCOPY2 = 112;     
  { OCILobErase2  }
     OCI_FNCODE_LOBERASE2 = 113;     
  { OCILobGetLength2  }
     OCI_FNCODE_LOBLENGTH2 = 114;     
  { OCILobLoadFromFile2  }
     OCI_FNCODE_LOBLOADFROMFILE2 = 115;     
  { OCILobRead2  }
     OCI_FNCODE_LOBREAD2 = 116;     
  { OCILobTrim2  }
     OCI_FNCODE_LOBTRIM2 = 117;     
  { OCILobWrite2  }
     OCI_FNCODE_LOBWRITE2 = 118;     
  { OCILobGetStorageLimit  }
     OCI_FNCODE_LOBGETSTORAGELIMIT = 119;     
  { OCIDBStartup  }
     OCI_FNCODE_DBSTARTUP = 120;     
  { OCIDBShutdown  }
     OCI_FNCODE_DBSHUTDOWN = 121;     
  { OCILobArrayRead  }
     OCI_FNCODE_LOBARRAYREAD = 122;     
  { OCILobArrayWrite  }
     OCI_FNCODE_LOBARRAYWRITE = 123;     
  { maximum OCI function code  }
     OCI_FNCODE_MAXFCN = 123;     
  {--------------------------------------------------------------------------- }

  type
  {-----------------------Handle Definitions---------------------------------- }
    POCIEnv = pointer;          // OCI environment handle
    POCIError = pointer;        // OCI error handle
    POCISvcCtx = pointer;       // OCI service handle
    POCIStmt = pointer;         // OCI statement handle
    POCIBind = pointer;         // OCI bind handle
    POCIDefine = pointer;       // OCI Define handle
    POCIDescribe = pointer;     // OCI Describe handle
    POCIServer = pointer;       // OCI Server handle
    POCISession = pointer;      // OCI Authentication handle
    POCIComplexObject = pointer;// OCI COR handle
    POCITrans = pointer;        // OCI Transaction handle
    POCISecurity = pointer;     // OCI Security handle
    POCISubscription = pointer; // subscription handle

    POCICPool = pointer;        // connection pool handle
    POCISPool = pointer;        // session pool handle
    POCIAuthInfo = pointer;     // auth handle
    POCIAdmin = pointer;        // admin handle
    POCIEvent = pointer;        // HA event handle



{    OCIEnv = record end;          // OCI environment handle
    OCIError = record end;        // OCI error handle
    OCISvcCtx = record end;       // OCI service handle
    OCIStmt = record end;         // OCI statement handle
    OCIBind = record end;         // OCI bind handle
    OCIDefine = record end;       // OCI Define handle
    OCIDescribe = record end;     // OCI Describe handle
    OCIServer = record end;       // OCI Server handle
    OCISession = record end;      // OCI Authentication handle
    OCIComplexObject = record end;// OCI COR handle
    OCITrans = record end;        // OCI Transaction handle
    OCISecurity = record end;     // OCI Security handle
    OCISubscription = record end; // subscription handle

    OCICPool = record end;        // connection pool handle
    OCISPool = record end;        // session pool handle
    OCIAuthInfo = record end;     // auth handle
    OCIAdmin = record end;        // admin handle
    OCIEvent = record end;        // HA event handle

    POCIEnv = ^OCIEnv;
    POCIError = ^OCIError;
    POCISvcCtx = ^OCISvcCtx;
    POCIStmt = ^OCIStmt;
    POCIBind = ^OCIBind;
    POCIDefine = ^OCIDefine;
    POCIDescribe = ^OCIDescribe;
    POCIServer = ^OCIServer;
    POCISession = ^OCISession;
    POCIComplexObject = ^OCIComplexObject;
    POCITrans = ^OCITrans;
    POCISecurity = ^OCISecurity;
    POCISubscription = ^OCISubscription;
    POCICPool = ^OCICPool;
    POCISPool = ^OCISPool;
    POCIAuthInfo = ^OCIAuthinfo;
    POCIAdmin = ^OCIAdmin;
    POCIEvent = ^OCIEvent;}

  {-----------------------Descriptor Definitions------------------------------ }
    POCISnapshot = pointer;       // OCI snapshot descriptor
    POCIResult = pointer;         // OCI Result Set Descriptor
    POCILobLocator = pointer;     // OCI Lob Locator descriptor
    POCIParam = pointer;          // OCI PARameter descriptor
    POCIComplexObjectComp = pointer; // OCI COR descriptor
    POCIROWID = pointer;          // OCI ROWID descriptor
//    POCIDateTime = pointer;       // OCI DateTime descriptor
    POCIInterval = pointer;       // OCI Interval descriptor
    POCIUcb = pointer;            // OCI User Callback descriptor
    POCIServerDNs = pointer;      // OCI server DN descriptor
  {-------------------------- AQ Descriptors --------------------------------- }
    POCIAQEnqOptions = pointer;       // AQ Enqueue Options hdl
    POCIAQDeqOptions = pointer;       // AQ Dequeue Options hdl
    POCIAQMsgProperties = pointer;    // AQ Mesg Properties
    POCIAQAgent = pointer;            // AQ Agent descriptor
    POCIAQNfyDescriptor = pointer;    // AQ Nfy descriptor
    POCIAQSignature = pointer;        // AQ Siganture
    POCIAQListenOpts = pointer;       // AQ listen options
    POCIAQLisMsgProps = pointer;      // AQ listen msg props
  {--------------------------------------------------------------------------- }
  { Lob typedefs for Pro*C  }
// Only for OCCI, which isn't supported in Pascal
//     OCILobLocator = OCIClobLocator;
  { OCI Character LOB Locator  }
// Only for OCCI, which isn't supported in Pascal
//     OCILobLocator = OCIBlobLocator;
  { OCI Binary LOB Locator  }
// Only for OCCI, which isn't supported in Pascal
//     OCILobLocator = OCIBFileLocator;
  { OCI Binary LOB File Locator  }
  {--------------------------------------------------------------------------- }
  { Undefined value for tz in interval types }

  const
     OCI_INTHR_UNK = 24;     
  { These defined adjustment values  }
     OCI_ADJUST_UNK = 10;     
     OCI_ORACLE_DATE = 0;     
     OCI_ANSI_DATE = 1;     
  {------------------------ Lob-specific Definitions ------------------------- }
  {
   * ociloff - OCI Lob OFFset
   *
   * The offset in the lob data.  The offset is specified in terms of bytes for
   * BLOBs and BFILes.  Character offsets are used for CLOBs, NCLOBs.
   * The maximum size of internal lob data is 4 gigabytes.  FILE LOB 
   * size is limited by the operating system.
    }

  type

     POCILobOffset = ^OCILobOffset;
     OCILobOffset = ub4;
  {
   * ocillen - OCI Lob LENgth (of lob data)
   *
   * Specifies the length of lob data in bytes for BLOBs and BFILes and in 
   * characters for CLOBs, NCLOBs.  The maximum length of internal lob
   * data is 4 gigabytes.  The length of FILE LOBs is limited only by the
   * operating system.
    }

     POCILobLength = ^OCILobLength;
     OCILobLength = ub4;
  {
   * ocilmo - OCI Lob open MOdes
   *
   * The mode specifies the planned operations that will be performed on the
   * FILE lob data.  The FILE lob can be opened in read-only mode only.
   * 
   * In the future, we may include read/write, append and truncate modes.  Append
   * is equivalent to read/write mode except that the FILE is positioned for
   * writing to the end.  Truncate is equivalent to read/write mode except that
   * the FILE LOB data is first truncated to a length of 0 before use.
    }
  { read-only  }
  { read_write for internal lobs only  }
     OCILobMode = (OCI_LOBMODE_READONLY := 1,OCI_LOBMODE_READWRITE := 2
       );


     POCILobMode = ^OCILobMode;

  {--------------------------------------------------------------------------- }
  {--------------------------- FILE open modes ------------------------------- }
  { readonly mode open for FILE types  }

  const
     OCI_FILE_READONLY = 1;     
  {--------------------------------------------------------------------------- }
  {--------------------------- LOB open modes -------------------------------- }
  { readonly mode open for ILOB types  }
     OCI_LOB_READONLY = 1;     
  { read write mode open for ILOBs  }
     OCI_LOB_READWRITE = 2;     
  {----------------------- LOB Buffering Flush Flags ------------------------- }
     OCI_LOB_BUFFER_FREE = 1;     
     OCI_LOB_BUFFER_NOFREE = 2;     
  {--------------------------------------------------------------------------- }
  {--------------------------- OCI Statement Types --------------------------- }
  { select statement  }
     OCI_STMT_SELECT = 1;     
  { update statement  }
     OCI_STMT_UPDATE = 2;     
  { delete statement  }
     OCI_STMT_DELETE = 3;     
  { Insert Statement  }
     OCI_STMT_INSERT = 4;     
  { create statement  }
     OCI_STMT_CREATE = 5;     
  { drop statement  }
     OCI_STMT_DROP = 6;     
  { alter statement  }
     OCI_STMT_ALTER = 7;     
  { begin ... (pl/sql statement) }
     OCI_STMT_BEGIN = 8;     
  { declare .. (pl/sql statement )  }
     OCI_STMT_DECLARE = 9;     
  {--------------------------------------------------------------------------- }
  {--------------------------- OCI Parameter Types --------------------------- }
  { unknown    }
     OCI_PTYPE_UNK = 0;     
  { table      }
     OCI_PTYPE_TABLE = 1;     
  { view       }
     OCI_PTYPE_VIEW = 2;     
  { procedure  }
     OCI_PTYPE_PROC = 3;     
  { function   }
     OCI_PTYPE_FUNC = 4;     
  { package    }
     OCI_PTYPE_PKG = 5;     
  { user-defined type  }
     OCI_PTYPE_TYPE = 6;     
  { synonym    }
     OCI_PTYPE_SYN = 7;     
  { sequence   }
     OCI_PTYPE_SEQ = 8;     
  { column     }
     OCI_PTYPE_COL = 9;     
  { argument   }
     OCI_PTYPE_ARG = 10;     
  { list       }
     OCI_PTYPE_LIST = 11;     
  { user-defined type's attribute  }
     OCI_PTYPE_TYPE_ATTR = 12;     
  { collection type's element  }
     OCI_PTYPE_TYPE_COLL = 13;     
  { user-defined type's method  }
     OCI_PTYPE_TYPE_METHOD = 14;     
  { user-defined type method's arg  }
     OCI_PTYPE_TYPE_ARG = 15;     
  { user-defined type method's result  }
     OCI_PTYPE_TYPE_RESULT = 16;     
  { schema  }
     OCI_PTYPE_SCHEMA = 17;     
  { database  }
     OCI_PTYPE_DATABASE = 18;     
  { rule  }
     OCI_PTYPE_RULE = 19;     
  { rule set  }
     OCI_PTYPE_RULE_SET = 20;     
  { evaluation context  }
     OCI_PTYPE_EVALUATION_CONTEXT = 21;     
  { table alias  }
     OCI_PTYPE_TABLE_ALIAS = 22;     
  { variable type  }
     OCI_PTYPE_VARIABLE_TYPE = 23;     
  { name value pair  }
     OCI_PTYPE_NAME_VALUE = 24;     
  {--------------------------------------------------------------------------- }
  {----------------------------- OCI List Types ------------------------------ }
  { unknown    }
     OCI_LTYPE_UNK = 0;     
  { column list  }
     OCI_LTYPE_COLUMN = 1;     
  { procedure argument list  }
     OCI_LTYPE_ARG_PROC = 2;     
  { function argument list  }
     OCI_LTYPE_ARG_FUNC = 3;     
  { subprogram list  }
     OCI_LTYPE_SUBPRG = 4;     
  { type attribute  }
     OCI_LTYPE_TYPE_ATTR = 5;     
  { type method  }
     OCI_LTYPE_TYPE_METHOD = 6;     
  { type method w/o result argument list  }
     OCI_LTYPE_TYPE_ARG_PROC = 7;     
  { type method w/result argument list  }
     OCI_LTYPE_TYPE_ARG_FUNC = 8;     
  { schema object list  }
     OCI_LTYPE_SCH_OBJ = 9;     
  { database schema list  }
     OCI_LTYPE_DB_SCH = 10;     
  { subtype list  }
     OCI_LTYPE_TYPE_SUBTYPE = 11;     
  { table alias list  }
     OCI_LTYPE_TABLE_ALIAS = 12;     
  { variable type list  }
     OCI_LTYPE_VARIABLE_TYPE = 13;     
  { name value list  }
     OCI_LTYPE_NAME_VALUE = 14;     
  {--------------------------------------------------------------------------- }
  {-------------------------- Memory Cartridge Services --------------------- }
     OCI_MEMORY_CLEARED = 1;     
  {-------------------------- Pickler Cartridge Services --------------------- }

  type
     POCIPicklerTdsCtx = pointer;
     POCIPicklerTds = pointer;
     POCIPicklerImage = pointer;
     POCIPicklerFdo = pointer;
     POCIPicklerTdsElement = ^OCIPicklerTdsElement;
     OCIPicklerTdsElement = ub4;

     POCIAnyData = pointer;

     POCIAnyDataSet = pointer;
     POCIAnyDataCtx = pointer;

  {--------------------------------------------------------------------------- }
  {--------------------------- User Callback Constants ----------------------- }
  { entry callback  }

  const
     OCI_UCBTYPE_ENTRY = 1;     
  { exit callback  }
     OCI_UCBTYPE_EXIT = 2;     
  { replacement callback  }
     OCI_UCBTYPE_REPLACE = 3;     
  {--------------------------------------------------------------------------- }
  {--------------------- NLS service type and constance ---------------------- }
  { Native name for Monday  }
     OCI_NLS_DAYNAME1 = 1;     
  { Native name for Tuesday  }
     OCI_NLS_DAYNAME2 = 2;     
  { Native name for Wednesday  }
     OCI_NLS_DAYNAME3 = 3;     
  { Native name for Thursday  }
     OCI_NLS_DAYNAME4 = 4;     
  { Native name for Friday  }
     OCI_NLS_DAYNAME5 = 5;     
  { Native name for for Saturday  }
     OCI_NLS_DAYNAME6 = 6;     
  { Native name for for Sunday  }
     OCI_NLS_DAYNAME7 = 7;     
  { Native abbreviated name for Monday  }
     OCI_NLS_ABDAYNAME1 = 8;     
  { Native abbreviated name for Tuesday  }
     OCI_NLS_ABDAYNAME2 = 9;     
  { Native abbreviated name for Wednesday  }
     OCI_NLS_ABDAYNAME3 = 10;     
  { Native abbreviated name for Thursday  }
     OCI_NLS_ABDAYNAME4 = 11;     
  { Native abbreviated name for Friday  }
     OCI_NLS_ABDAYNAME5 = 12;     
  { Native abbreviated name for for Saturday  }
     OCI_NLS_ABDAYNAME6 = 13;     
  { Native abbreviated name for for Sunday  }
     OCI_NLS_ABDAYNAME7 = 14;     
  { Native name for January  }
     OCI_NLS_MONTHNAME1 = 15;     
  { Native name for February  }
     OCI_NLS_MONTHNAME2 = 16;     
  { Native name for March  }
     OCI_NLS_MONTHNAME3 = 17;     
  { Native name for April  }
     OCI_NLS_MONTHNAME4 = 18;     
  { Native name for May  }
     OCI_NLS_MONTHNAME5 = 19;     
  { Native name for June  }
     OCI_NLS_MONTHNAME6 = 20;     
  { Native name for July  }
     OCI_NLS_MONTHNAME7 = 21;     
  { Native name for August  }
     OCI_NLS_MONTHNAME8 = 22;     
  { Native name for September  }
     OCI_NLS_MONTHNAME9 = 23;     
  { Native name for October  }
     OCI_NLS_MONTHNAME10 = 24;     
  { Native name for November  }
     OCI_NLS_MONTHNAME11 = 25;     
  { Native name for December  }
     OCI_NLS_MONTHNAME12 = 26;     
  { Native abbreviated name for January  }
     OCI_NLS_ABMONTHNAME1 = 27;     
  { Native abbreviated name for February  }
     OCI_NLS_ABMONTHNAME2 = 28;     
  { Native abbreviated name for March  }
     OCI_NLS_ABMONTHNAME3 = 29;     
  { Native abbreviated name for April  }
     OCI_NLS_ABMONTHNAME4 = 30;     
  { Native abbreviated name for May  }
     OCI_NLS_ABMONTHNAME5 = 31;     
  { Native abbreviated name for June  }
     OCI_NLS_ABMONTHNAME6 = 32;     
  { Native abbreviated name for July  }
     OCI_NLS_ABMONTHNAME7 = 33;     
  { Native abbreviated name for August  }
     OCI_NLS_ABMONTHNAME8 = 34;     
  { Native abbreviated name for September  }
     OCI_NLS_ABMONTHNAME9 = 35;     
  { Native abbreviated name for October  }
     OCI_NLS_ABMONTHNAME10 = 36;     
  { Native abbreviated name for November  }
     OCI_NLS_ABMONTHNAME11 = 37;     
  { Native abbreviated name for December  }
     OCI_NLS_ABMONTHNAME12 = 38;     
  { Native string for affirmative response  }
     OCI_NLS_YES = 39;     
  { Native negative response  }
     OCI_NLS_NO = 40;     
  { Native equivalent string of AM  }
     OCI_NLS_AM = 41;     
  { Native equivalent string of PM  }
     OCI_NLS_PM = 42;     
  { Native equivalent string of AD  }
     OCI_NLS_AD = 43;     
  { Native equivalent string of BC  }
     OCI_NLS_BC = 44;     
  { decimal character  }
     OCI_NLS_DECIMAL = 45;     
  { group separator  }
     OCI_NLS_GROUP = 46;     
  { Native symbol of debit  }
     OCI_NLS_DEBIT = 47;     
  { Native sumbol of credit  }
     OCI_NLS_CREDIT = 48;     
  { Oracle date format  }
     OCI_NLS_DATEFORMAT = 49;     
  { International currency symbol  }
     OCI_NLS_INT_CURRENCY = 50;     
  { Locale currency symbol  }
     OCI_NLS_LOC_CURRENCY = 51;     
  { Language name  }
     OCI_NLS_LANGUAGE = 52;     
  { Abbreviation for language name  }
     OCI_NLS_ABLANGUAGE = 53;     
  { Territory name  }
     OCI_NLS_TERRITORY = 54;     
  { Character set name  }
     OCI_NLS_CHARACTER_SET = 55;     
  { Linguistic name  }
     OCI_NLS_LINGUISTIC_NAME = 56;     
  { Calendar name  }
     OCI_NLS_CALENDAR = 57;     
  { Dual currency symbol  }
     OCI_NLS_DUAL_CURRENCY = 78;     
  { Language writing direction  }
     OCI_NLS_WRITINGDIR = 79;     
  { Territory Abbreviation  }
     OCI_NLS_ABTERRITORY = 80;     
  { Oracle default date format  }
     OCI_NLS_DDATEFORMAT = 81;     
  { Oracle default time format  }
     OCI_NLS_DTIMEFORMAT = 82;     
  { Local string formatted date format  }
     OCI_NLS_SFDATEFORMAT = 83;     
  { Local string formatted time format  }
     OCI_NLS_SFTIMEFORMAT = 84;     
  { Number grouping fields  }
     OCI_NLS_NUMGROUPING = 85;     
  { List separator  }
     OCI_NLS_LISTSEP = 86;     
  { Monetary decimal character  }
     OCI_NLS_MONDECIMAL = 87;     
  { Monetary group separator  }
     OCI_NLS_MONGROUP = 88;     
  { Monetary grouping fields  }
     OCI_NLS_MONGROUPING = 89;     
  { International currency separator  }
     OCI_NLS_INT_CURRENCYSEP = 90;     
  { Maximum character byte size       }
     OCI_NLS_CHARSET_MAXBYTESZ = 91;     
  { Fixed-width charset byte size     }
     OCI_NLS_CHARSET_FIXEDWIDTH = 92;     
  { Character set id  }
     OCI_NLS_CHARSET_ID = 93;     
  { NCharacter set id  }
     OCI_NLS_NCHARSET_ID = 94;     
  { Max buffer size may need for OCINlsGetInfo  }
     OCI_NLS_MAXBUFSZ = 100;     
  { for the binary comparison  }
     OCI_NLS_BINARY = $1;     
  { for linguistic comparison  }
     OCI_NLS_LINGUISTIC = $2;     
  { for case-insensitive comparison  }
     OCI_NLS_CASE_INSENSITIVE = $10;     
  { convert to uppercase  }
     OCI_NLS_UPPERCASE = $20;     
  { convert to lowercase  }
     OCI_NLS_LOWERCASE = $40;     
  { Map charset name from IANA to Oracle  }
     OCI_NLS_CS_IANA_TO_ORA = 0;     
  { Map charset name from Oracle to IANA  }
     OCI_NLS_CS_ORA_TO_IANA = 1;     
  { Map language name from ISO to Oracle  }
     OCI_NLS_LANG_ISO_TO_ORA = 2;     
  { Map language name from Oracle to ISO  }
     OCI_NLS_LANG_ORA_TO_ISO = 3;     
  { Map territory name from ISO to Oracle }
     OCI_NLS_TERR_ISO_TO_ORA = 4;     
  { Map territory name from Oracle to ISO }
     OCI_NLS_TERR_ORA_TO_ISO = 5;     
  { Map territory name from 3-letter ISO  }
     OCI_NLS_TERR_ISO3_TO_ORA = 6;     
  { abbreviation to Oracle                }
  { Map territory name from Oracle to     }
     OCI_NLS_TERR_ORA_TO_ISO3 = 7;     
  { 3-letter ISO abbreviation             }

  type
     POCIMsg = pointer;

     POCIWchar = ^OCIWchar;
     OCIWchar = ub4;

  const
     OCI_XMLTYPE_CREATE_OCISTRING = 1;     
     OCI_XMLTYPE_CREATE_CLOB = 2;     
     OCI_XMLTYPE_CREATE_BLOB = 3;     
  {------------------------- Kerber Authentication Modes --------------------- }
  { Apply Kerberos Creds for Proxy  }
     OCI_KERBCRED_PROXY = 1;     
  {Apply Creds for Secure Client ID  }
     OCI_KERBCRED_CLIENT_IDENTIFIER = 2;     
  {------------------------- Database Startup Flags -------------------------- }
  { Abort running instance, start  }
     OCI_DBSTARTUPFLAG_FORCE = $00000001;     
  { Restrict access to DBA  }
     OCI_DBSTARTUPFLAG_RESTRICT = $00000002;     
  {------------------------- Database Shutdown Modes ------------------------- }
  { Wait for all the transactions  }
     OCI_DBSHUTDOWN_TRANSACTIONAL = 1;     
  { Wait for local transactions  }
     OCI_DBSHUTDOWN_TRANSACTIONAL_LOCAL = 2;     
  { Terminate and roll back  }
     OCI_DBSHUTDOWN_IMMEDIATE = 3;     
  { Terminate and don't roll back  }
     OCI_DBSHUTDOWN_ABORT = 4;     
  { Orderly shutdown  }
     OCI_DBSHUTDOWN_FINAL = 5;     
  {------------------------- Version information ----------------------------- }
  { Major release version  }
     OCI_MAJOR_VERSION = 10;     
  { Minor release version  }
     OCI_MINOR_VERSION = 2;     
  {--------------------------------------------------------------------------- 
                       PRIVATE TYPES AND CONSTANTS 
    --------------------------------------------------------------------------- }
  { None  }
  {--------------------------------------------------------------------------- 
                             PUBLIC FUNCTIONS 
    --------------------------------------------------------------------------- }
  { see ociap.h or ocikp.h  }
  {--------------------------------------------------------------------------- 
                            PRIVATE FUNCTIONS 
    --------------------------------------------------------------------------- }
  { None  }

  { more includes  }
{$include oci1.inc}
{$include oro_interface.inc}

{$include orl.inc}

{$include ort.inc}

{$include ociap.inc}

{$IFDEF LinkDynamically}
<<<<<<< HEAD
<<<<<<< HEAD
Function InitialiseOCI(const LibraryName: AnsiString): integer;
Function InitialiseOCI(): integer;
Procedure ReleaseOCI;

var OCILibraryHandle : TLibHandle;
    OCILoadedLibrary : AnsiString;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure InitialiseOCI;
Procedure ReleaseOCI;

var OCILibraryHandle : TLibHandle;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF}

implementation

{$include oro_implementation.inc}

{$IFDEF LinkDynamically}

<<<<<<< HEAD
<<<<<<< HEAD
var 
  RefCount : integer;

Function InitialiseOCI(): integer;

begin
  if (OCILoadedLibrary='') then
    Result:=InitialiseOCI(OCILib)
 else
    Result:=InitialiseOCI(OCILoadedLibrary);
end;

function InitialiseOCI(const LibraryName: AnsiString): integer;

begin
  inc(RefCount);
  Result:=RefCount;
  if RefCount > 1 then
    begin
    if CompareText(OCILoadedLibrary,LibraryName)<>0 then
      Raise EInOutError.CreateFmt('Can not load Oracle client library "%s". it is already loaded as "%s".',[LibraryName,OCILoadedLibrary]);
    exit; 
    end
  else
    begin
    OCILibraryHandle:=loadlibrary(LibraryName);
    if (OCILibraryHandle=nilhandle) then
      begin
      RefCount := 0;
      Raise EInOutError.CreateFmt('Can not load Oracle client library "%s". Is it installed?',[LibraryName]);
      end;
    OCILoadedLibrary:=LibraryName;
    end;  
    
{ ORL.inc}
  pointer(OCINumberInc) := GetProcedureAddress(OCILibraryHandle,'OCINumberInc');
  pointer(OCINumberDec) := GetProcedureAddress(OCILibraryHandle,'OCINumberDec');
  pointer(OCINumberSetZero) := GetProcedureAddress(OCILibraryHandle,'OCINumberSetZero');
  pointer(OCINumberSetPi) := GetProcedureAddress(OCILibraryHandle,'OCINumberSetPi');
  pointer(OCINumberAdd) := GetProcedureAddress(OCILibraryHandle,'OCINumberAdd');
  pointer(OCINumberSub) := GetProcedureAddress(OCILibraryHandle,'OCINumberSub');
  pointer(OCINumberMul) := GetProcedureAddress(OCILibraryHandle,'OCINumberMul');
  pointer(OCINumberDiv) := GetProcedureAddress(OCILibraryHandle,'OCINumberDiv');
  pointer(OCINumberMod) := GetProcedureAddress(OCILibraryHandle,'OCINumberMod');
  pointer(OCINumberIntPower) := GetProcedureAddress(OCILibraryHandle,'OCINumberIntPower');
  pointer(OCINumberShift) := GetProcedureAddress(OCILibraryHandle,'OCINumberShift');
  pointer(OCINumberNeg) := GetProcedureAddress(OCILibraryHandle,'OCINumberNeg');
  pointer(OCINumberToText) := GetProcedureAddress(OCILibraryHandle,'OCINumberToText');
  pointer(OCINumberFromText) := GetProcedureAddress(OCILibraryHandle,'OCINumberFromText');
  pointer(OCINumberToInt) := GetProcedureAddress(OCILibraryHandle,'OCINumberToInt');
  pointer(OCINumberFromInt) := GetProcedureAddress(OCILibraryHandle,'OCINumberFromInt');
  pointer(OCINumberToReal) := GetProcedureAddress(OCILibraryHandle,'OCINumberToReal');
  pointer(OCINumberToRealArray) := GetProcedureAddress(OCILibraryHandle,'OCINumberToRealArray');
  pointer(OCINumberFromReal) := GetProcedureAddress(OCILibraryHandle,'OCINumberFromReal');
  pointer(OCINumberCmp) := GetProcedureAddress(OCILibraryHandle,'OCINumberCmp');
  pointer(OCINumberSign) := GetProcedureAddress(OCILibraryHandle,'OCINumberSign');
  pointer(OCINumberIsZero) := GetProcedureAddress(OCILibraryHandle,'OCINumberIsZero');
  pointer(OCINumberIsInt) := GetProcedureAddress(OCILibraryHandle,'OCINumberIsInt');
  pointer(OCINumberAssign) := GetProcedureAddress(OCILibraryHandle,'OCINumberAssign');
  pointer(OCINumberAbs) := GetProcedureAddress(OCILibraryHandle,'OCINumberAbs');
  pointer(OCINumberCeil) := GetProcedureAddress(OCILibraryHandle,'OCINumberCeil');
  pointer(OCINumberFloor) := GetProcedureAddress(OCILibraryHandle,'OCINumberFloor');
  pointer(OCINumberSqrt) := GetProcedureAddress(OCILibraryHandle,'OCINumberSqrt');
  pointer(OCINumberTrunc) := GetProcedureAddress(OCILibraryHandle,'OCINumberTrunc');
  pointer(OCINumberPower) := GetProcedureAddress(OCILibraryHandle,'OCINumberPower');
  pointer(OCINumberRound) := GetProcedureAddress(OCILibraryHandle,'OCINumberRound');
  pointer(OCINumberPrec) := GetProcedureAddress(OCILibraryHandle,'OCINumberPrec');
  pointer(OCINumberSin) := GetProcedureAddress(OCILibraryHandle,'OCINumberSin');
  pointer(OCINumberArcSin) := GetProcedureAddress(OCILibraryHandle,'OCINumberArcSin');
  pointer(OCINumberHypSin) := GetProcedureAddress(OCILibraryHandle,'OCINumberHypSin');
  pointer(OCINumberCos) := GetProcedureAddress(OCILibraryHandle,'OCINumberCos');
  pointer(OCINumberArcCos) := GetProcedureAddress(OCILibraryHandle,'OCINumberArcCos');
  pointer(OCINumberHypCos) := GetProcedureAddress(OCILibraryHandle,'OCINumberHypCos');
  pointer(OCINumberTan) := GetProcedureAddress(OCILibraryHandle,'OCINumberTan');
  pointer(OCINumberArcTan) := GetProcedureAddress(OCILibraryHandle,'OCINumberArcTan');
  pointer(OCINumberArcTan2) := GetProcedureAddress(OCILibraryHandle,'OCINumberArcTan2');
  pointer(OCINumberHypTan) := GetProcedureAddress(OCILibraryHandle,'OCINumberHypTan');
  pointer(OCINumberExp) := GetProcedureAddress(OCILibraryHandle,'OCINumberExp');
  pointer(OCINumberLn) := GetProcedureAddress(OCILibraryHandle,'OCINumberLn');
  pointer(OCINumberLog) := GetProcedureAddress(OCILibraryHandle,'OCINumberLog');
  pointer(OCIDateToText) := GetProcedureAddress(OCILibraryHandle,'var OCIDateToText');
  pointer(OCIDateFromText) := GetProcedureAddress(OCILibraryHandle,'OCIDateFromText');
  pointer(OCIDateCompare) := GetProcedureAddress(OCILibraryHandle,'OCIDateCompare');
  pointer(OCIDateAddMonths) := GetProcedureAddress(OCILibraryHandle,'OCIDateAddMonths');
  pointer(OCIDateAddDays) := GetProcedureAddress(OCILibraryHandle,'OCIDateAddDays');
  pointer(OCIDateLastDay) := GetProcedureAddress(OCILibraryHandle,'OCIDateLastDay');
  pointer(OCIDateDaysBetween) := GetProcedureAddress(OCILibraryHandle,'OCIDateDaysBetween');
  pointer(OCIDateZoneToZone) := GetProcedureAddress(OCILibraryHandle,'OCIDateZoneToZone');
  pointer(OCIDateNextDay) := GetProcedureAddress(OCILibraryHandle,'OCIDateNextDay');
  pointer(OCIDateCheck) := GetProcedureAddress(OCILibraryHandle,'OCIDateCheck');
  pointer(OCIDateSysDate) := GetProcedureAddress(OCILibraryHandle,'OCIDateSysDate');
  pointer(OCIStringAssign) := GetProcedureAddress(OCILibraryHandle,'OCIStringAssign');
  pointer(OCIStringAssignText) := GetProcedureAddress(OCILibraryHandle,'OCIStringAssignText');
  pointer(OCIStringResize) := GetProcedureAddress(OCILibraryHandle,'OCIStringResize');
  pointer(OCIStringSize) := GetProcedureAddress(OCILibraryHandle,'OCIStringSize');
  pointer(OCIStringPtr) := GetProcedureAddress(OCILibraryHandle,'OCIStringPtr');
  pointer(OCIStringAllocSize) := GetProcedureAddress(OCILibraryHandle,'OCIStringAllocSize');
  pointer(OCIRawAssignRaw) := GetProcedureAddress(OCILibraryHandle,'OCIRawAssignRaw');
  pointer(OCIRawAssignBytes) := GetProcedureAddress(OCILibraryHandle,'OCIRawAssignBytes');
  pointer(OCIRawResize) := GetProcedureAddress(OCILibraryHandle,'OCIRawResize');
  pointer(OCIRawSize) := GetProcedureAddress(OCILibraryHandle,'OCIRawSize');
  pointer(OCIRawPtr) := GetProcedureAddress(OCILibraryHandle,'OCIRawPtr');
  pointer(OCIRawAllocSize) := GetProcedureAddress(OCILibraryHandle,'OCIRawAllocSize');
  pointer(OCIRefClear) := GetProcedureAddress(OCILibraryHandle,'OCIRefClear');
  pointer(OCIRefToHex) := GetProcedureAddress(OCILibraryHandle,'OCIRefToHex');
  pointer(OCICollSize) := GetProcedureAddress(OCILibraryHandle,'OCICollSize');
  pointer(OCICollMax) := GetProcedureAddress(OCILibraryHandle,'OCICollMax');
  pointer(OCICollGetElem) := GetProcedureAddress(OCILibraryHandle,'OCICollGetElem');
  pointer(OCICollAssignElem) := GetProcedureAddress(OCILibraryHandle,'OCICollAssignElem');
  pointer(OCICollTrim) := GetProcedureAddress(OCILibraryHandle,'OCICollTrim');
  pointer(OCICollIsLocator) := GetProcedureAddress(OCILibraryHandle,'OCICollIsLocator');
  pointer(OCIIterCreate) := GetProcedureAddress(OCILibraryHandle,'OCIIterCreate');
  pointer(OCIIterDelete) := GetProcedureAddress(OCILibraryHandle,'OCIIterDelete');
  pointer(OCIIterInit) := GetProcedureAddress(OCILibraryHandle,'OCIIterInit');
  pointer(OCIIterGetCurrent) := GetProcedureAddress(OCILibraryHandle,'OCIIterGetCurrent');
  pointer(OCIIterPrev) := GetProcedureAddress(OCILibraryHandle,'OCIIterPrev');

{ ORT.inc}

  pointer(OCITypeIterNew) := GetProcedureAddress(OCILibraryHandle,'OCITypeIterNew');
  pointer(OCITypeArrayByName) := GetProcedureAddress(OCILibraryHandle,'OCITypeArrayByName');
  pointer(OCITypeArrayByRef) := GetProcedureAddress(OCILibraryHandle,'OCITypeArrayByRef');
  pointer(OCITypeSchema) := GetProcedureAddress(OCILibraryHandle,'OCITypeSchema');
  pointer(OCITypeCollTypeCode) := GetProcedureAddress(OCILibraryHandle,'OCITypeCollTypeCode');
  pointer(OCITypeAttrs) := GetProcedureAddress(OCILibraryHandle,'OCITypeAttrs');
  pointer(OCITypeElemName) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemName');
  pointer(OCITypeElemType) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemType');
  pointer(OCITypeElemFlags) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemFlags');
  pointer(OCITypeElemNumScale) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemNumScale');
  pointer(OCITypeElemCharSetID) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemCharSetID');
  pointer(OCITypeElemParameterizedType) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemParameterizedType');
  pointer(OCITypeAttrByName) := GetProcedureAddress(OCILibraryHandle,'OCITypeAttrByName');
  pointer(OCITypeMethodNext) := GetProcedureAddress(OCILibraryHandle,'OCITypeMethodNext');
  pointer(OCITypeMethodName) := GetProcedureAddress(OCILibraryHandle,'OCITypeMethodName');
  pointer(OCITypeMethodFlags) := GetProcedureAddress(OCILibraryHandle,'OCITypeMethodFlags');
  pointer(OCITypeMethodOrder) := GetProcedureAddress(OCILibraryHandle,'OCITypeMethodOrder');
  pointer(OCITypeResult) := GetProcedureAddress(OCILibraryHandle,'OCITypeResult');
  pointer(OCITypeParamByName) := GetProcedureAddress(OCILibraryHandle,'OCITypeParamByName');
  pointer(OCITypeElemDefaultValue) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemDefaultValue');
  pointer(OCITypeVTInsert) := GetProcedureAddress(OCILibraryHandle,'OCITypeVTInsert');
  pointer(ortgcty) := GetProcedureAddress(OCILibraryHandle,'ortgcty');
  pointer(OCITypeBeginCreate) := GetProcedureAddress(OCILibraryHandle,'OCITypeBeginCreate');
  pointer(OCITypeSetCollection) := GetProcedureAddress(OCILibraryHandle,'OCITypeSetCollection');
  pointer(OCITypeSetBuiltin) := GetProcedureAddress(OCILibraryHandle,'OCITypeSetBuiltin');
  pointer(OCITypeAddAttr) := GetProcedureAddress(OCILibraryHandle,'OCITypeAddAttr');
  pointer(OCITypeEndCreate) := GetProcedureAddress(OCILibraryHandle,'OCITypeEndCreate');

{ nzt.inc}

  pointer(nzteStorePersona) := GetProcedureAddress(OCILibraryHandle,'nzteStorePersona');
  pointer(nzteRemovePersona) := GetProcedureAddress(OCILibraryHandle,'nzteRemovePersona');
  pointer(nzteCreatePersona) := GetProcedureAddress(OCILibraryHandle,'nzteCreatePersona');
  pointer(nztiStoreTrustedIdentity) := GetProcedureAddress(OCILibraryHandle,'nztiStoreTrustedIdentity');
  pointer(nzteSetProtection) := GetProcedureAddress(OCILibraryHandle,'nzteSetProtection');
  pointer(nzteGetProtection) := GetProcedureAddress(OCILibraryHandle,'nzteGetProtection');
  pointer(nztiRemoveIdentity) := GetProcedureAddress(OCILibraryHandle,'nztiRemoveIdentity');
  pointer(nztifdn) := GetProcedureAddress(OCILibraryHandle,'nztifdn');
  pointer(nztxSignExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxSignExpansion');
  pointer(nztxsd_SignDetachedExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxsd_SignDetachedExpansion');
  pointer(nztEncrypt) := GetProcedureAddress(OCILibraryHandle,'nztEncrypt');
  pointer(nztxEncryptExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxEncryptExpansion');
  pointer(nztDecrypt) := GetProcedureAddress(OCILibraryHandle,'nztDecrypt');
  pointer(nztEnvelope) := GetProcedureAddress(OCILibraryHandle,'nztEnvelope');
  pointer(nztDeEnvelope) := GetProcedureAddress(OCILibraryHandle,'nztDeEnvelope');
  pointer(nztKeyedHash) := GetProcedureAddress(OCILibraryHandle,'nztKeyedHash');
  pointer(nztxKeyedHashExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxKeyedHashExpansion');
  pointer(nztxHashExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxHashExpansion');
  pointer(nztiae_IsAuthEnabled) := GetProcedureAddress(OCILibraryHandle,'nztiae_IsAuthEnabled');
  pointer(nztiee_IsEncrEnabled) := GetProcedureAddress(OCILibraryHandle,'nztiee_IsEncrEnabled');
  pointer(nztihe_IsHashEnabled) := GetProcedureAddress(OCILibraryHandle,'nztihe_IsHashEnabled');
  pointer(nztGetIssuerName) := GetProcedureAddress(OCILibraryHandle,'nztGetIssuerName');
  pointer(nztGetSubjectName) := GetProcedureAddress(OCILibraryHandle,'nztGetSubjectName');
  pointer(nztGetBase64Cert) := GetProcedureAddress(OCILibraryHandle,'nztGetBase64Cert');
  pointer(nztGetSerialNumber) := GetProcedureAddress(OCILibraryHandle,'nztGetSerialNumber');
  pointer(nztGetValidDate) := GetProcedureAddress(OCILibraryHandle,'nztGetValidDate');
  pointer(nztGetVersion) := GetProcedureAddress(OCILibraryHandle,'nztGetVersion');
  pointer(nztGetPublicKey) := GetProcedureAddress(OCILibraryHandle,'nztGetPublicKey');
  pointer(nztGenericDestroy) := GetProcedureAddress(OCILibraryHandle,'nztGenericDestroy');
  pointer(nztSetAppDefaultLocation) := GetProcedureAddress(OCILibraryHandle,'nztSetAppDefaultLocation');
  pointer(nztSearchNZDefault) := GetProcedureAddress(OCILibraryHandle,'nztSearchNZDefault');

{ ociap.inc}

  pointer(OCIInitialize) := GetProcedureAddress(OCILibraryHandle,'OCIInitialize');
  pointer(OCITerminate) := GetProcedureAddress(OCILibraryHandle,'OCITerminate');
  pointer(OCIEnvCreate) := GetProcedureAddress(OCILibraryHandle,'OCIEnvCreate');
  pointer(OCIEnvNlsCreate) := GetProcedureAddress(OCILibraryHandle,'OCIEnvNlsCreate');
  pointer(OCIFEnvCreate) := GetProcedureAddress(OCILibraryHandle,'OCIFEnvCreate');
  pointer(OCIHandleAlloc) := GetProcedureAddress(OCILibraryHandle,'OCIHandleAlloc');
  pointer(OCIHandleFree) := GetProcedureAddress(OCILibraryHandle,'OCIHandleFree');
  pointer(OCIDescriptorAlloc) := GetProcedureAddress(OCILibraryHandle,'OCIDescriptorAlloc');
  pointer(OCIDescriptorFree) := GetProcedureAddress(OCILibraryHandle,'OCIDescriptorFree');
  pointer(OCIEnvInit) := GetProcedureAddress(OCILibraryHandle,'OCIEnvInit');
  pointer(OCIServerAttach) := GetProcedureAddress(OCILibraryHandle,'OCIServerAttach');
  pointer(OCIServerDetach) := GetProcedureAddress(OCILibraryHandle,'OCIServerDetach');
  pointer(OCISessionBegin) := GetProcedureAddress(OCILibraryHandle,'OCISessionBegin');
  pointer(OCISessionEnd) := GetProcedureAddress(OCILibraryHandle,'OCISessionEnd');
  pointer(OCILogon) := GetProcedureAddress(OCILibraryHandle,'OCILogon');
  pointer(OCILogon2) := GetProcedureAddress(OCILibraryHandle,'OCILogon2');
  pointer(OCILogoff) := GetProcedureAddress(OCILibraryHandle,'OCILogoff');
  pointer(OCIPasswordChange) := GetProcedureAddress(OCILibraryHandle,'OCIPasswordChange');
  pointer(OCIStmtPrepare) := GetProcedureAddress(OCILibraryHandle,'OCIStmtPrepare');
  pointer(OCIStmtPrepare2) := GetProcedureAddress(OCILibraryHandle,'OCIStmtPrepare2');
  pointer(OCIStmtRelease) := GetProcedureAddress(OCILibraryHandle,'OCIStmtRelease');
  pointer(OCIBindByPos) := GetProcedureAddress(OCILibraryHandle,'OCIBindByPos');
  pointer(OCIBindByName) := GetProcedureAddress(OCILibraryHandle,'OCIBindByName');
  pointer(OCIBindObject) := GetProcedureAddress(OCILibraryHandle,'OCIBindObject');
  pointer(OCIBindDynamic) := GetProcedureAddress(OCILibraryHandle,'OCIBindDynamic');
  pointer(OCIBindArrayOfStruct) := GetProcedureAddress(OCILibraryHandle,'OCIBindArrayOfStruct');
  pointer(OCIStmtGetPieceInfo) := GetProcedureAddress(OCILibraryHandle,'OCIStmtGetPieceInfo');
  pointer(OCIStmtSetPieceInfo) := GetProcedureAddress(OCILibraryHandle,'OCIStmtSetPieceInfo');
  pointer(OCIStmtExecute) := GetProcedureAddress(OCILibraryHandle,'OCIStmtExecute');
  pointer(OCIDefineByPos) := GetProcedureAddress(OCILibraryHandle,'OCIDefineByPos');
  pointer(OCIDefineObject) := GetProcedureAddress(OCILibraryHandle,'OCIDefineObject');
  pointer(OCIDefineDynamic) := GetProcedureAddress(OCILibraryHandle,'OCIDefineDynamic');
  pointer(OCIRowidToChar) := GetProcedureAddress(OCILibraryHandle,'OCIRowidToChar');
  pointer(OCIDefineArrayOfStruct) := GetProcedureAddress(OCILibraryHandle,'OCIDefineArrayOfStruct');
  pointer(OCIStmtFetch) := GetProcedureAddress(OCILibraryHandle,'OCIStmtFetch');
  pointer(OCIStmtFetch2) := GetProcedureAddress(OCILibraryHandle,'OCIStmtFetch2');
  pointer(OCIStmtGetBindInfo) := GetProcedureAddress(OCILibraryHandle,'OCIStmtGetBindInfo');
  pointer(OCIDescribeAny) := GetProcedureAddress(OCILibraryHandle,'OCIDescribeAny');
  pointer(OCIParamGet) := GetProcedureAddress(OCILibraryHandle,'OCIParamGet');
  pointer(OCIParamSet) := GetProcedureAddress(OCILibraryHandle,'OCIParamSet');
  pointer(OCITransStart) := GetProcedureAddress(OCILibraryHandle,'OCITransStart');
  pointer(OCITransDetach) := GetProcedureAddress(OCILibraryHandle,'OCITransDetach');
  pointer(OCITransCommit) := GetProcedureAddress(OCILibraryHandle,'OCITransCommit');
  pointer(OCITransRollback) := GetProcedureAddress(OCILibraryHandle,'OCITransRollback');
  pointer(OCITransPrepare) := GetProcedureAddress(OCILibraryHandle,'OCITransPrepare');
  pointer(OCITransMultiPrepare) := GetProcedureAddress(OCILibraryHandle,'OCITransMultiPrepare');
  pointer(OCITransForget) := GetProcedureAddress(OCILibraryHandle,'OCITransForget');
  pointer(OCIErrorGet) := GetProcedureAddress(OCILibraryHandle,'OCIErrorGet');
  pointer(OCILobAppend) := GetProcedureAddress(OCILibraryHandle,'OCILobAppend');
  pointer(OCILobAssign) := GetProcedureAddress(OCILibraryHandle,'OCILobAssign');
  pointer(OCILobCharSetForm) := GetProcedureAddress(OCILibraryHandle,'OCILobCharSetForm');
  pointer(OCILobCharSetId) := GetProcedureAddress(OCILibraryHandle,'OCILobCharSetId');
  pointer(OCILobCopy) := GetProcedureAddress(OCILibraryHandle,'OCILobCopy');
  pointer(OCILobCreateTemporary) := GetProcedureAddress(OCILibraryHandle,'OCILobCreateTemporary');
  pointer(OCILobClose) := GetProcedureAddress(OCILibraryHandle,'OCILobClose');
  pointer(OCILobDisableBuffering) := GetProcedureAddress(OCILibraryHandle,'OCILobDisableBuffering');
  pointer(OCILobEnableBuffering) := GetProcedureAddress(OCILibraryHandle,'OCILobEnableBuffering');
  pointer(OCILobErase) := GetProcedureAddress(OCILibraryHandle,'OCILobErase');
  pointer(OCILobFileClose) := GetProcedureAddress(OCILibraryHandle,'OCILobFileClose');
  pointer(OCILobFileCloseAll) := GetProcedureAddress(OCILibraryHandle,'OCILobFileCloseAll');
  pointer(OCILobFileExists) := GetProcedureAddress(OCILibraryHandle,'OCILobFileExists');
  pointer(OCILobFileGetName) := GetProcedureAddress(OCILibraryHandle,'OCILobFileGetName');
  pointer(OCILobFileIsOpen) := GetProcedureAddress(OCILibraryHandle,'OCILobFileIsOpen');
  pointer(OCILobFileOpen) := GetProcedureAddress(OCILibraryHandle,'OCILobFileOpen');
  pointer(OCILobFileSetName) := GetProcedureAddress(OCILibraryHandle,'OCILobFileSetName');
  pointer(OCILobFlushBuffer) := GetProcedureAddress(OCILibraryHandle,'OCILobFlushBuffer');
  pointer(OCILobFreeTemporary) := GetProcedureAddress(OCILibraryHandle,'OCILobFreeTemporary');
  pointer(OCILobGetChunkSize) := GetProcedureAddress(OCILibraryHandle,'OCILobGetChunkSize');
  pointer(OCILobGetLength) := GetProcedureAddress(OCILibraryHandle,'OCILobGetLength');
  pointer(OCILobIsEqual) := GetProcedureAddress(OCILibraryHandle,'OCILobIsEqual');
  pointer(OCILobIsOpen) := GetProcedureAddress(OCILibraryHandle,'OCILobIsOpen');
  pointer(OCILobIsTemporary) := GetProcedureAddress(OCILibraryHandle,'OCILobIsTemporary');
  pointer(OCILobLoadFromFile) := GetProcedureAddress(OCILibraryHandle,'OCILobLoadFromFile');
  pointer(OCILobLocatorAssign) := GetProcedureAddress(OCILibraryHandle,'OCILobLocatorAssign');
  pointer(OCILobLocatorIsInit) := GetProcedureAddress(OCILibraryHandle,'OCILobLocatorIsInit');
  pointer(OCILobOpen) := GetProcedureAddress(OCILibraryHandle,'OCILobOpen');
  pointer(OCILobRead) := GetProcedureAddress(OCILibraryHandle,'OCILobRead');
  pointer(OCILobTrim) := GetProcedureAddress(OCILibraryHandle,'OCILobTrim');
  pointer(OCILobWrite) := GetProcedureAddress(OCILibraryHandle,'OCILobWrite');
  pointer(OCILobWriteAppend) := GetProcedureAddress(OCILibraryHandle,'OCILobWriteAppend');
  pointer(OCIBreak) := GetProcedureAddress(OCILibraryHandle,'OCIBreak');
  pointer(OCIReset) := GetProcedureAddress(OCILibraryHandle,'OCIReset');
  pointer(OCIServerVersion) := GetProcedureAddress(OCILibraryHandle,'OCIServerVersion');
  pointer(OCIServerRelease) := GetProcedureAddress(OCILibraryHandle,'OCIServerRelease');
  pointer(OCIAttrGet) := GetProcedureAddress(OCILibraryHandle,'OCIAttrGet');
  pointer(OCIAttrSet) := GetProcedureAddress(OCILibraryHandle,'OCIAttrSet');
  pointer(OCISvcCtxToLda) := GetProcedureAddress(OCILibraryHandle,'OCISvcCtxToLda');
  pointer(OCILdaToSvcCtx) := GetProcedureAddress(OCILibraryHandle,'OCILdaToSvcCtx');
  pointer(OCIResultSetToStmt) := GetProcedureAddress(OCILibraryHandle,'OCIResultSetToStmt');
  pointer(OCIFileClose) := GetProcedureAddress(OCILibraryHandle,'OCIFileClose');
  pointer(OCISharedLibInit) := GetProcedureAddress(OCILibraryHandle,'OCISharedLibInit');
  pointer(OCIFileExists) := GetProcedureAddress(OCILibraryHandle,'OCIFileExists');
  pointer(OCIFileFlush) := GetProcedureAddress(OCILibraryHandle,'OCIFileFlush');
  pointer(OCIFileGetLength) := GetProcedureAddress(OCILibraryHandle,'OCIFileGetLength');
  pointer(OCIFileInit) := GetProcedureAddress(OCILibraryHandle,'OCIFileInit');
  pointer(OCIFileOpen) := GetProcedureAddress(OCILibraryHandle,'OCIFileOpen');
  pointer(OCIFileRead) := GetProcedureAddress(OCILibraryHandle,'OCIFileRead');
  pointer(OCIFileSeek) := GetProcedureAddress(OCILibraryHandle,'OCIFileSeek');
  pointer(OCIFileTerm) := GetProcedureAddress(OCILibraryHandle,'OCIFileTerm');
  pointer(OCIFileWrite) := GetProcedureAddress(OCILibraryHandle,'OCIFileWrite');
{$ifdef ORAXB8_DEFINED}
  pointer(OCILobCopy2) := GetProcedureAddress(OCILibraryHandle,'OCILobCopy2');
  pointer(OCILobErase2) := GetProcedureAddress(OCILibraryHandle,'OCILobErase2');
  pointer(OCILobGetLength2) := GetProcedureAddress(OCILibraryHandle,'OCILobGetLength2');
  pointer(OCILobLoadFromFile2) := GetProcedureAddress(OCILibraryHandle,'OCILobLoadFromFile2');
  pointer(OCILobRead2) := GetProcedureAddress(OCILibraryHandle,'OCILobRead2');
  pointer(OCILobArrayRead) := GetProcedureAddress(OCILibraryHandle,'OCILobArrayRead');
  pointer(OCILobTrim2) := GetProcedureAddress(OCILibraryHandle,'OCILobTrim2');
  pointer(OCILobWrite2) := GetProcedureAddress(OCILibraryHandle,'OCILobWrite2');
  pointer(OCILobArrayWrite) := GetProcedureAddress(OCILibraryHandle,'OCILobArrayWrite');
  pointer(OCILobWriteAppend2) := GetProcedureAddress(OCILibraryHandle,'OCILobWriteAppend2');
  pointer(OCILobGetStorageLimit) := GetProcedureAddress(OCILibraryHandle,'OCILobGetStorageLimit');
{$endif}
  pointer(OCISecurityInitialize) := GetProcedureAddress(OCILibraryHandle,'OCISecurityInitialize');
  pointer(OCISecurityTerminate) := GetProcedureAddress(OCILibraryHandle,'OCISecurityTerminate');
  pointer(OCISecurityOpenWallet) := GetProcedureAddress(OCILibraryHandle,'OCISecurityOpenWallet');
  pointer(OCISecurityCloseWallet) := GetProcedureAddress(OCILibraryHandle,'OCISecurityCloseWallet');
  pointer(OCISecurityCreateWallet) := GetProcedureAddress(OCILibraryHandle,'OCISecurityCreateWallet');
  pointer(OCISecurityDestroyWallet) := GetProcedureAddress(OCILibraryHandle,'OCISecurityDestroyWallet');
  pointer(OCISecurityStorePersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityStorePersona');
  pointer(OCISecurityOpenPersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityOpenPersona');
  pointer(OCISecurityClosePersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityClosePersona');
  pointer(OCISecurityRemovePersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityRemovePersona');
  pointer(OCISecurityCreatePersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityCreatePersona');
  pointer(OCISecuritySetProtection) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySetProtection');
  pointer(OCISecurityGetProtection) := GetProcedureAddress(OCILibraryHandle,'OCISecurityGetProtection');
  pointer(OCISecurityRemoveIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityRemoveIdentity');
  pointer(OCISecurityCreateIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityCreateIdentity');
  pointer(OCISecurityAbortIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityAbortIdentity');
  pointer(OCISecurityFreeIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityFreeIdentity');
  pointer(OCISecurityStoreTrustedIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityStoreTrustedIdentity');
  pointer(OCISecuritySign) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySign');
  pointer(OCISecuritySignExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySignExpansion');
  pointer(OCISecurityVerify) := GetProcedureAddress(OCILibraryHandle,'OCISecurityVerify');
  pointer(OCISecurityValidate) := GetProcedureAddress(OCILibraryHandle,'OCISecurityValidate');
  pointer(OCISecuritySignDetached) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySignDetached');
  pointer(OCISecuritySignDetExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySignDetExpansion');
  pointer(OCISecurityVerifyDetached) := GetProcedureAddress(OCILibraryHandle,'OCISecurityVerifyDetached');
  pointer(OCISecurity_PKEncrypt) := GetProcedureAddress(OCILibraryHandle,'OCISecurity_PKEncrypt');
  pointer(OCISecurityPKEncryptExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecurityPKEncryptExpansion');
  pointer(OCISecurityPKDecrypt) := GetProcedureAddress(OCILibraryHandle,'OCISecurityPKDecrypt');
  pointer(OCISecurityEncrypt) := GetProcedureAddress(OCILibraryHandle,'OCISecurityEncrypt');
  pointer(OCISecurityEncryptExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecurityEncryptExpansion');
  pointer(OCISecurityDecrypt) := GetProcedureAddress(OCILibraryHandle,'OCISecurityDecrypt');
  pointer(OCISecurityEnvelope) := GetProcedureAddress(OCILibraryHandle,'OCISecurityEnvelope');
  pointer(OCISecurityDeEnvelope) := GetProcedureAddress(OCILibraryHandle,'OCISecurityDeEnvelope');
  pointer(OCISecurityKeyedHash) := GetProcedureAddress(OCILibraryHandle,'OCISecurityKeyedHash');
  pointer(OCISecurityKeyedHashExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecurityKeyedHashExpansion');
  pointer(OCISecurityHash) := GetProcedureAddress(OCILibraryHandle,'OCISecurityHash');
  pointer(OCISecurityHashExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecurityHashExpansion');
  pointer(OCISecuritySeedRandom) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySeedRandom');
  pointer(OCISecurityRandomBytes) := GetProcedureAddress(OCILibraryHandle,'OCISecurityRandomBytes');
  pointer(OCISecurityRandomNumber) := GetProcedureAddress(OCILibraryHandle,'OCISecurityRandomNumber');
  pointer(OCISecurityInitBlock) := GetProcedureAddress(OCILibraryHandle,'OCISecurityInitBlock');
  pointer(OCISecurityReuseBlock) := GetProcedureAddress(OCILibraryHandle,'OCISecurityReuseBlock');
  pointer(OCISecurityPurgeBlock) := GetProcedureAddress(OCILibraryHandle,'OCISecurityPurgeBlock');
  pointer(OCISecuritySetBlock) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySetBlock');
  pointer(OCISecurityGetIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityGetIdentity');
  pointer(OCIAQEnq) := GetProcedureAddress(OCILibraryHandle,'OCIAQEnq');
  pointer(OCIAQDeq) := GetProcedureAddress(OCILibraryHandle,'OCIAQDeq');
  pointer(OCIAQEnqArray) := GetProcedureAddress(OCILibraryHandle,'OCIAQEnqArray');
  pointer(OCIAQDeqArray) := GetProcedureAddress(OCILibraryHandle,'OCIAQDeqArray');
  pointer(OCIAQListen) := GetProcedureAddress(OCILibraryHandle,'OCIAQListen');
  pointer(OCIAQListen2) := GetProcedureAddress(OCILibraryHandle,'OCIAQListen2');
  pointer(OCIExtractInit) := GetProcedureAddress(OCILibraryHandle,'OCIExtractInit');
  pointer(OCIExtractTerm) := GetProcedureAddress(OCILibraryHandle,'OCIExtractTerm');
  pointer(OCIExtractReset) := GetProcedureAddress(OCILibraryHandle,'OCIExtractReset');
  pointer(OCIExtractSetNumKeys) := GetProcedureAddress(OCILibraryHandle,'OCIExtractSetNumKeys');
  pointer(OCIExtractSetKey) := GetProcedureAddress(OCILibraryHandle,'OCIExtractSetKey');
  pointer(OCIExtractFromFile) := GetProcedureAddress(OCILibraryHandle,'OCIExtractFromFile');
  pointer(OCIExtractFromStr) := GetProcedureAddress(OCILibraryHandle,'OCIExtractFromStr');
  pointer(OCIExtractToInt) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToInt');
  pointer(OCIExtractToBool) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToBool');
  pointer(OCIExtractToStr) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToStr');
  pointer(OCIExtractToOCINum) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToOCINum');
  pointer(OCIExtractToList) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToList');
  pointer(OCIExtractFromList) := GetProcedureAddress(OCILibraryHandle,'OCIExtractFromList');
  pointer(OCIMemoryAlloc) := GetProcedureAddress(OCILibraryHandle,'OCIMemoryAlloc');
  pointer(OCIMemoryResize) := GetProcedureAddress(OCILibraryHandle,'OCIMemoryResize');
  pointer(OCIMemoryFree) := GetProcedureAddress(OCILibraryHandle,'OCIMemoryFree');
  pointer(OCIContextSetValue) := GetProcedureAddress(OCILibraryHandle,'OCIContextSetValue');
  pointer(OCIContextGetValue) := GetProcedureAddress(OCILibraryHandle,'OCIContextGetValue');
  pointer(OCIContextClearValue) := GetProcedureAddress(OCILibraryHandle,'OCIContextClearValue');
  pointer(OCIContextGenerateKey) := GetProcedureAddress(OCILibraryHandle,'OCIContextGenerateKey');
  pointer(OCIMemorySetCurrentIDs) := GetProcedureAddress(OCILibraryHandle,'OCIMemorySetCurrentIDs');
  pointer(OCIPicklerTdsCtxInit) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCtxInit');
  pointer(OCIPicklerTdsCtxFree) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCtxFree');
  pointer(OCIPicklerTdsInit) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsInit');
  pointer(OCIPicklerTdsFree) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsFree');
  pointer(OCIPicklerTdsCreateElementNumber) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElementNumber');
  pointer(OCIPicklerTdsCreateElementChar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElementChar');
  pointer(OCIPicklerTdsCreateElementVarchar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElementVarchar');
  pointer(OCIPicklerTdsCreateElementRaw) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElementRaw');
  pointer(OCIPicklerTdsCreateElement) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElement');
  pointer(OCIPicklerTdsAddAttr) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsAddAttr');
  pointer(OCIPicklerTdsGenerate) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsGenerate');
  pointer(OCIPicklerTdsGetAttr) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsGetAttr');
  pointer(OCIPicklerFdoInit) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerFdoInit');
  pointer(OCIPicklerFdoFree) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerFdoFree');
  pointer(OCIPicklerImageInit) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageInit');
  pointer(OCIPicklerImageFree) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageFree');
  pointer(OCIPicklerImageAddScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageAddScalar');
  pointer(OCIPicklerImageAddNullScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageAddNullScalar');
  pointer(OCIPicklerImageGenerate) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageGenerate');
  pointer(OCIPicklerImageGetScalarSize) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageGetScalarSize');
  pointer(OCIPicklerImageGetScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageGetScalar');
  pointer(OCIPicklerImageCollBegin) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollBegin');
  pointer(OCIPicklerImageCollAddScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollAddScalar');
  pointer(OCIPicklerImageCollEnd) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollEnd');
  pointer(OCIPicklerImageCollBeginScan) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollBeginScan');
  pointer(OCIPicklerImageCollGetScalarSize) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollGetScalarSize');
  pointer(OCIPicklerImageCollGetScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollGetScalar');
  pointer(OCIAnyDataGetType) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataGetType');
  pointer(OCIAnyDataIsNull) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataIsNull');
  pointer(OCIAnyDataConvert) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataConvert');
  pointer(OCIAnyDataBeginCreate) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataBeginCreate');
  pointer(OCIAnyDataDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataDestroy');
  pointer(OCIAnyDataAttrSet) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataAttrSet');
  pointer(OCIAnyDataCollAddElem) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataCollAddElem');
  pointer(OCIAnyDataEndCreate) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataEndCreate');
  pointer(OCIAnyDataAccess) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataAccess');
  pointer(OCIAnyDataGetCurrAttrNum) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataGetCurrAttrNum');
  pointer(OCIAnyDataAttrGet) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataAttrGet');
  pointer(OCIAnyDataCollGetElem) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataCollGetElem');
  pointer(OCIAnyDataSetBeginCreate) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetBeginCreate');
  pointer(OCIAnyDataSetDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetDestroy');
  pointer(OCIAnyDataSetAddInstance) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetAddInstance');
  pointer(OCIAnyDataSetEndCreate) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetEndCreate');
  pointer(OCIAnyDataSetGetType) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetGetType');
  pointer(OCIAnyDataSetGetCount) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetGetCount');
  pointer(OCIAnyDataSetGetInstance) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetGetInstance');
  pointer(OCIFormatInit) := GetProcedureAddress(OCILibraryHandle,'OCIFormatInit');
  pointer(OCIFormatString) := GetProcedureAddress(OCILibraryHandle,'OCIFormatString');
  pointer(OCIFormatTerm) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTerm');
  pointer(OCIFormatTUb1) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUb1');
  pointer(OCIFormatTUb2) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUb2');
  pointer(OCIFormatTUb4) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUb4');
  pointer(OCIFormatTUword) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUword');
  pointer(OCIFormatTUbig_ora) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUbig_ora');
  pointer(OCIFormatTSb1) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSb1');
  pointer(OCIFormatTSb2) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSb2');
  pointer(OCIFormatTSb4) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSb4');
  pointer(OCIFormatTSword) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSword');
  pointer(OCIFormatTSbig_ora) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSbig_ora');
  pointer(OCIFormatTEb1) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEb1');
  pointer(OCIFormatTEb2) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEb2');
  pointer(OCIFormatTEb4) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEb4');
  pointer(OCIFormatTEword) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEword');
  pointer(OCIFormatTChar) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTChar');
  pointer(OCIFormatTText) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTText');
  pointer(OCIFormatTDouble) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTDouble');
  pointer(OCIFormatTDvoid) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTDvoid');
  pointer(OCIFormatTEnd) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEnd');
  pointer(xaosvch) := GetProcedureAddress(OCILibraryHandle,'xaosvch');
  pointer(xaoSvcCtx) := GetProcedureAddress(OCILibraryHandle,'xaoSvcCtx');
  pointer(xaoEnv) := GetProcedureAddress(OCILibraryHandle,'xaoEnv');
  pointer(xaosterr) := GetProcedureAddress(OCILibraryHandle,'xaosterr');
  pointer(OCINlsGetInfo) := GetProcedureAddress(OCILibraryHandle,'OCINlsGetInfo');
  pointer(OCINlsNumericInfoGet) := GetProcedureAddress(OCILibraryHandle,'OCINlsNumericInfoGet');
  pointer(OCINlsCharSetNameToId) := GetProcedureAddress(OCILibraryHandle,'OCINlsCharSetNameToId');
  pointer(OCIThreadProcessInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadProcessInit');
  pointer(OCIThreadInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadInit');
  pointer(OCIThreadTerm) := GetProcedureAddress(OCILibraryHandle,'OCIThreadTerm');
  pointer(OCIThreadIsMulti) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIsMulti');
  pointer(OCIThreadMutexInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadMutexInit');
  pointer(OCIThreadMutexDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIThreadMutexDestroy');
  pointer(OCIThreadMutexAcquire) := GetProcedureAddress(OCILibraryHandle,'OCIThreadMutexAcquire');
  pointer(OCIThreadMutexRelease) := GetProcedureAddress(OCILibraryHandle,'OCIThreadMutexRelease');
  pointer(OCIThreadKeyInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadKeyInit');
  pointer(OCIThreadKeyDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIThreadKeyDestroy');
  pointer(OCIThreadKeyGet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadKeyGet');
  pointer(OCIThreadKeySet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadKeySet');
  pointer(OCIThreadIdInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdInit');
  pointer(OCIThreadIdDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdDestroy');
  pointer(OCIThreadIdSet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdSet');
  pointer(OCIThreadIdSetNull) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdSetNull');
  pointer(OCIThreadIdGet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdGet');
  pointer(OCIThreadIdSame) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdSame');
  pointer(OCIThreadIdNull) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdNull');
  pointer(OCIThreadHndInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadHndInit');
  pointer(OCIThreadHndDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIThreadHndDestroy');
  pointer(OCIThreadCreate) := GetProcedureAddress(OCILibraryHandle,'OCIThreadCreate');
  pointer(OCIThreadJoin) := GetProcedureAddress(OCILibraryHandle,'OCIThreadJoin');
  pointer(OCIThreadClose) := GetProcedureAddress(OCILibraryHandle,'OCIThreadClose');
  pointer(OCIThreadHandleGet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadHandleGet');
  pointer(OCISubscriptionRegister) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionRegister');
  pointer(OCISubscriptionPost) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionPost');
  pointer(OCISubscriptionUnRegister) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionUnRegister');
  pointer(OCISubscriptionDisable) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionDisable');
  pointer(OCISubscriptionEnable) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionEnable');
  pointer(OCIDateTimeGetTime) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeGetTime');
  pointer(OCIDateTimeGetDate) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeGetDate');
  pointer(OCIDateTimeGetTimeZoneOffset) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeGetTimeZoneOffset');
  pointer(OCIDateTimeConstruct) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeConstruct');
  pointer(OCIDateTimeSysTimeStamp) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeSysTimeStamp');
  pointer(OCIDateTimeAssign) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeAssign');
  pointer(OCIDateTimeToText) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeToText');
  pointer(OCIDateTimeFromText) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeFromText');
  pointer(OCIDateTimeCompare) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeCompare');
  pointer(OCIDateTimeCheck) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeCheck');
  pointer(OCIDateTimeConvert) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeConvert');
  pointer(OCIDateTimeSubtract) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeSubtract');
  pointer(OCIDateTimeIntervalAdd) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeIntervalAdd');
  pointer(OCIDateTimeIntervalSub) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeIntervalSub');
  pointer(OCIIntervalSubtract) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalSubtract');
  pointer(OCIIntervalAdd) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalAdd');
  pointer(OCIIntervalMultiply) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalMultiply');
  pointer(OCIIntervalDivide) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalDivide');
  pointer(OCIIntervalCompare) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalCompare');
  pointer(OCIIntervalFromNumber) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalFromNumber');
  pointer(OCIIntervalFromText) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalFromText');
  pointer(OCIIntervalToText) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalToText');
  pointer(OCIIntervalToNumber) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalToNumber');
  pointer(OCIIntervalCheck) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalCheck');
  pointer(OCIIntervalAssign) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalAssign');
  pointer(OCIIntervalSetYearMonth) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalSetYearMonth');
  pointer(OCIIntervalGetYearMonth) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalGetYearMonth');
  pointer(OCIIntervalSetDaySecond) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalSetDaySecond');
  pointer(OCIIntervalGetDaySecond) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalGetDaySecond');
  pointer(OCIDateTimeToArray) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeToArray');
  pointer(OCIDateTimeFromArray) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeFromArray');
  pointer(OCIDateTimeGetTimeZoneName) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeGetTimeZoneName');
  pointer(OCIIntervalFromTZ) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalFromTZ');
  pointer(OCIConnectionPoolCreate) := GetProcedureAddress(OCILibraryHandle,'OCIConnectionPoolCreate');
  pointer(OCIConnectionPoolDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIConnectionPoolDestroy');
  pointer(OCISessionPoolCreate) := GetProcedureAddress(OCILibraryHandle,'OCISessionPoolCreate');
  pointer(OCISessionPoolDestroy) := GetProcedureAddress(OCILibraryHandle,'OCISessionPoolDestroy');
  pointer(OCISessionGet) := GetProcedureAddress(OCILibraryHandle,'OCISessionGet');
  pointer(OCISessionRelease) := GetProcedureAddress(OCILibraryHandle,'OCISessionRelease');
  pointer(OCIAppCtxSet) := GetProcedureAddress(OCILibraryHandle,'OCIAppCtxSet');
  pointer(OCIAppCtxClearAll) := GetProcedureAddress(OCILibraryHandle,'OCIAppCtxClearAll');
  pointer(OCIPing) := GetProcedureAddress(OCILibraryHandle,'OCIPing');
  pointer(OCIKerbAttrSet) := GetProcedureAddress(OCILibraryHandle,'OCIKerbAttrSet');
  pointer(OCIDBStartup) := GetProcedureAddress(OCILibraryHandle,'OCIDBStartup');
  pointer(OCIDBShutdown) := GetProcedureAddress(OCILibraryHandle,'OCIDBShutdown');
  pointer(OCIClientVersion) := GetProcedureAddress(OCILibraryHandle,'OCIClientVersion');
  pointer(OCIInitEventHandle) := GetProcedureAddress(OCILibraryHandle,'OCIInitEventHandle');
=======
=======
>>>>>>> origin/fixes_2_2
var RefCount : integer;

Procedure InitialiseOCI;

begin
  inc(RefCount);
  if RefCount = 1 then
    begin
    OCILibraryHandle:=loadlibrary(ocilib);
    if (OCILibraryHandle=nilhandle) then
      begin
      RefCount := 0;
      Raise EInOutError.Create('Can not load Oracle client. Is it installed? ('+ocilib+')');
      end;
{ ORL.inc}
    pointer(OCINumberInc) := GetProcedureAddress(OCILibraryHandle,'OCINumberInc');
    pointer(OCINumberDec) := GetProcedureAddress(OCILibraryHandle,'OCINumberDec');
    pointer(OCINumberSetZero) := GetProcedureAddress(OCILibraryHandle,'OCINumberSetZero');
    pointer(OCINumberSetPi) := GetProcedureAddress(OCILibraryHandle,'OCINumberSetPi');
    pointer(OCINumberAdd) := GetProcedureAddress(OCILibraryHandle,'OCINumberAdd');
    pointer(OCINumberSub) := GetProcedureAddress(OCILibraryHandle,'OCINumberSub');
    pointer(OCINumberMul) := GetProcedureAddress(OCILibraryHandle,'OCINumberMul');
    pointer(OCINumberDiv) := GetProcedureAddress(OCILibraryHandle,'OCINumberDiv');
    pointer(OCINumberMod) := GetProcedureAddress(OCILibraryHandle,'OCINumberMod');
    pointer(OCINumberIntPower) := GetProcedureAddress(OCILibraryHandle,'OCINumberIntPower');
    pointer(OCINumberShift) := GetProcedureAddress(OCILibraryHandle,'OCINumberShift');
    pointer(OCINumberNeg) := GetProcedureAddress(OCILibraryHandle,'OCINumberNeg');
    pointer(OCINumberToText) := GetProcedureAddress(OCILibraryHandle,'OCINumberToText');
    pointer(OCINumberFromText) := GetProcedureAddress(OCILibraryHandle,'OCINumberFromText');
    pointer(OCINumberToInt) := GetProcedureAddress(OCILibraryHandle,'OCINumberToInt');
    pointer(OCINumberFromInt) := GetProcedureAddress(OCILibraryHandle,'OCINumberFromInt');
    pointer(OCINumberToReal) := GetProcedureAddress(OCILibraryHandle,'OCINumberToReal');
    pointer(OCINumberToRealArray) := GetProcedureAddress(OCILibraryHandle,'OCINumberToRealArray');
    pointer(OCINumberFromReal) := GetProcedureAddress(OCILibraryHandle,'OCINumberFromReal');
    pointer(OCINumberCmp) := GetProcedureAddress(OCILibraryHandle,'OCINumberCmp');
    pointer(OCINumberSign) := GetProcedureAddress(OCILibraryHandle,'OCINumberSign');
    pointer(OCINumberIsZero) := GetProcedureAddress(OCILibraryHandle,'OCINumberIsZero');
    pointer(OCINumberIsInt) := GetProcedureAddress(OCILibraryHandle,'OCINumberIsInt');
    pointer(OCINumberAssign) := GetProcedureAddress(OCILibraryHandle,'OCINumberAssign');
    pointer(OCINumberAbs) := GetProcedureAddress(OCILibraryHandle,'OCINumberAbs');
    pointer(OCINumberCeil) := GetProcedureAddress(OCILibraryHandle,'OCINumberCeil');
    pointer(OCINumberFloor) := GetProcedureAddress(OCILibraryHandle,'OCINumberFloor');
    pointer(OCINumberSqrt) := GetProcedureAddress(OCILibraryHandle,'OCINumberSqrt');
    pointer(OCINumberTrunc) := GetProcedureAddress(OCILibraryHandle,'OCINumberTrunc');
    pointer(OCINumberPower) := GetProcedureAddress(OCILibraryHandle,'OCINumberPower');
    pointer(OCINumberRound) := GetProcedureAddress(OCILibraryHandle,'OCINumberRound');
    pointer(OCINumberPrec) := GetProcedureAddress(OCILibraryHandle,'OCINumberPrec');
    pointer(OCINumberSin) := GetProcedureAddress(OCILibraryHandle,'OCINumberSin');
    pointer(OCINumberArcSin) := GetProcedureAddress(OCILibraryHandle,'OCINumberArcSin');
    pointer(OCINumberHypSin) := GetProcedureAddress(OCILibraryHandle,'OCINumberHypSin');
    pointer(OCINumberCos) := GetProcedureAddress(OCILibraryHandle,'OCINumberCos');
    pointer(OCINumberArcCos) := GetProcedureAddress(OCILibraryHandle,'OCINumberArcCos');
    pointer(OCINumberHypCos) := GetProcedureAddress(OCILibraryHandle,'OCINumberHypCos');
    pointer(OCINumberTan) := GetProcedureAddress(OCILibraryHandle,'OCINumberTan');
    pointer(OCINumberArcTan) := GetProcedureAddress(OCILibraryHandle,'OCINumberArcTan');
    pointer(OCINumberArcTan2) := GetProcedureAddress(OCILibraryHandle,'OCINumberArcTan2');
    pointer(OCINumberHypTan) := GetProcedureAddress(OCILibraryHandle,'OCINumberHypTan');
    pointer(OCINumberExp) := GetProcedureAddress(OCILibraryHandle,'OCINumberExp');
    pointer(OCINumberLn) := GetProcedureAddress(OCILibraryHandle,'OCINumberLn');
    pointer(OCINumberLog) := GetProcedureAddress(OCILibraryHandle,'OCINumberLog');
    pointer(OCIDateToText) := GetProcedureAddress(OCILibraryHandle,'var OCIDateToText');
    pointer(OCIDateFromText) := GetProcedureAddress(OCILibraryHandle,'OCIDateFromText');
    pointer(OCIDateCompare) := GetProcedureAddress(OCILibraryHandle,'OCIDateCompare');
    pointer(OCIDateAddMonths) := GetProcedureAddress(OCILibraryHandle,'OCIDateAddMonths');
    pointer(OCIDateAddDays) := GetProcedureAddress(OCILibraryHandle,'OCIDateAddDays');
    pointer(OCIDateLastDay) := GetProcedureAddress(OCILibraryHandle,'OCIDateLastDay');
    pointer(OCIDateDaysBetween) := GetProcedureAddress(OCILibraryHandle,'OCIDateDaysBetween');
    pointer(OCIDateZoneToZone) := GetProcedureAddress(OCILibraryHandle,'OCIDateZoneToZone');
    pointer(OCIDateNextDay) := GetProcedureAddress(OCILibraryHandle,'OCIDateNextDay');
    pointer(OCIDateCheck) := GetProcedureAddress(OCILibraryHandle,'OCIDateCheck');
    pointer(OCIDateSysDate) := GetProcedureAddress(OCILibraryHandle,'OCIDateSysDate');
    pointer(OCIStringAssign) := GetProcedureAddress(OCILibraryHandle,'OCIStringAssign');
    pointer(OCIStringAssignText) := GetProcedureAddress(OCILibraryHandle,'OCIStringAssignText');
    pointer(OCIStringResize) := GetProcedureAddress(OCILibraryHandle,'OCIStringResize');
    pointer(OCIStringSize) := GetProcedureAddress(OCILibraryHandle,'OCIStringSize');
    pointer(OCIStringPtr) := GetProcedureAddress(OCILibraryHandle,'OCIStringPtr');
    pointer(OCIStringAllocSize) := GetProcedureAddress(OCILibraryHandle,'OCIStringAllocSize');
    pointer(OCIRawAssignRaw) := GetProcedureAddress(OCILibraryHandle,'OCIRawAssignRaw');
    pointer(OCIRawAssignBytes) := GetProcedureAddress(OCILibraryHandle,'OCIRawAssignBytes');
    pointer(OCIRawResize) := GetProcedureAddress(OCILibraryHandle,'OCIRawResize');
    pointer(OCIRawSize) := GetProcedureAddress(OCILibraryHandle,'OCIRawSize');
    pointer(OCIRawPtr) := GetProcedureAddress(OCILibraryHandle,'OCIRawPtr');
    pointer(OCIRawAllocSize) := GetProcedureAddress(OCILibraryHandle,'OCIRawAllocSize');
    pointer(OCIRefClear) := GetProcedureAddress(OCILibraryHandle,'OCIRefClear');
    pointer(OCIRefToHex) := GetProcedureAddress(OCILibraryHandle,'OCIRefToHex');
    pointer(OCICollSize) := GetProcedureAddress(OCILibraryHandle,'OCICollSize');
    pointer(OCICollMax) := GetProcedureAddress(OCILibraryHandle,'OCICollMax');
    pointer(OCICollGetElem) := GetProcedureAddress(OCILibraryHandle,'OCICollGetElem');
    pointer(OCICollAssignElem) := GetProcedureAddress(OCILibraryHandle,'OCICollAssignElem');
    pointer(OCICollTrim) := GetProcedureAddress(OCILibraryHandle,'OCICollTrim');
    pointer(OCICollIsLocator) := GetProcedureAddress(OCILibraryHandle,'OCICollIsLocator');
    pointer(OCIIterCreate) := GetProcedureAddress(OCILibraryHandle,'OCIIterCreate');
    pointer(OCIIterDelete) := GetProcedureAddress(OCILibraryHandle,'OCIIterDelete');
    pointer(OCIIterInit) := GetProcedureAddress(OCILibraryHandle,'OCIIterInit');
    pointer(OCIIterGetCurrent) := GetProcedureAddress(OCILibraryHandle,'OCIIterGetCurrent');
    pointer(OCIIterPrev) := GetProcedureAddress(OCILibraryHandle,'OCIIterPrev');

{ ORT.inc}

    pointer(OCITypeIterNew) := GetProcedureAddress(OCILibraryHandle,'OCITypeIterNew');
    pointer(OCITypeArrayByName) := GetProcedureAddress(OCILibraryHandle,'OCITypeArrayByName');
    pointer(OCITypeArrayByRef) := GetProcedureAddress(OCILibraryHandle,'OCITypeArrayByRef');
    pointer(OCITypeSchema) := GetProcedureAddress(OCILibraryHandle,'OCITypeSchema');
    pointer(OCITypeCollTypeCode) := GetProcedureAddress(OCILibraryHandle,'OCITypeCollTypeCode');
    pointer(OCITypeAttrs) := GetProcedureAddress(OCILibraryHandle,'OCITypeAttrs');
    pointer(OCITypeElemName) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemName');
    pointer(OCITypeElemType) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemType');
    pointer(OCITypeElemFlags) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemFlags');
    pointer(OCITypeElemNumScale) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemNumScale');
    pointer(OCITypeElemCharSetID) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemCharSetID');
    pointer(OCITypeElemParameterizedType) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemParameterizedType');
    pointer(OCITypeAttrByName) := GetProcedureAddress(OCILibraryHandle,'OCITypeAttrByName');
    pointer(OCITypeMethodNext) := GetProcedureAddress(OCILibraryHandle,'OCITypeMethodNext');
    pointer(OCITypeMethodName) := GetProcedureAddress(OCILibraryHandle,'OCITypeMethodName');
    pointer(OCITypeMethodFlags) := GetProcedureAddress(OCILibraryHandle,'OCITypeMethodFlags');
    pointer(OCITypeMethodOrder) := GetProcedureAddress(OCILibraryHandle,'OCITypeMethodOrder');
    pointer(OCITypeResult) := GetProcedureAddress(OCILibraryHandle,'OCITypeResult');
    pointer(OCITypeParamByName) := GetProcedureAddress(OCILibraryHandle,'OCITypeParamByName');
    pointer(OCITypeElemDefaultValue) := GetProcedureAddress(OCILibraryHandle,'OCITypeElemDefaultValue');
    pointer(OCITypeVTInsert) := GetProcedureAddress(OCILibraryHandle,'OCITypeVTInsert');
    pointer(ortgcty) := GetProcedureAddress(OCILibraryHandle,'ortgcty');
    pointer(OCITypeBeginCreate) := GetProcedureAddress(OCILibraryHandle,'OCITypeBeginCreate');
    pointer(OCITypeSetCollection) := GetProcedureAddress(OCILibraryHandle,'OCITypeSetCollection');
    pointer(OCITypeSetBuiltin) := GetProcedureAddress(OCILibraryHandle,'OCITypeSetBuiltin');
    pointer(OCITypeAddAttr) := GetProcedureAddress(OCILibraryHandle,'OCITypeAddAttr');
    pointer(OCITypeEndCreate) := GetProcedureAddress(OCILibraryHandle,'OCITypeEndCreate');

{ nzt.inc}

    pointer(nzteStorePersona) := GetProcedureAddress(OCILibraryHandle,'nzteStorePersona');
    pointer(nzteRemovePersona) := GetProcedureAddress(OCILibraryHandle,'nzteRemovePersona');
    pointer(nzteCreatePersona) := GetProcedureAddress(OCILibraryHandle,'nzteCreatePersona');
    pointer(nztiStoreTrustedIdentity) := GetProcedureAddress(OCILibraryHandle,'nztiStoreTrustedIdentity');
    pointer(nzteSetProtection) := GetProcedureAddress(OCILibraryHandle,'nzteSetProtection');
    pointer(nzteGetProtection) := GetProcedureAddress(OCILibraryHandle,'nzteGetProtection');
    pointer(nztiRemoveIdentity) := GetProcedureAddress(OCILibraryHandle,'nztiRemoveIdentity');
    pointer(nztifdn) := GetProcedureAddress(OCILibraryHandle,'nztifdn');
    pointer(nztxSignExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxSignExpansion');
    pointer(nztxsd_SignDetachedExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxsd_SignDetachedExpansion');
    pointer(nztEncrypt) := GetProcedureAddress(OCILibraryHandle,'nztEncrypt');
    pointer(nztxEncryptExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxEncryptExpansion');
    pointer(nztDecrypt) := GetProcedureAddress(OCILibraryHandle,'nztDecrypt');
    pointer(nztEnvelope) := GetProcedureAddress(OCILibraryHandle,'nztEnvelope');
    pointer(nztDeEnvelope) := GetProcedureAddress(OCILibraryHandle,'nztDeEnvelope');
    pointer(nztKeyedHash) := GetProcedureAddress(OCILibraryHandle,'nztKeyedHash');
    pointer(nztxKeyedHashExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxKeyedHashExpansion');
    pointer(nztxHashExpansion) := GetProcedureAddress(OCILibraryHandle,'nztxHashExpansion');
    pointer(nztiae_IsAuthEnabled) := GetProcedureAddress(OCILibraryHandle,'nztiae_IsAuthEnabled');
    pointer(nztiee_IsEncrEnabled) := GetProcedureAddress(OCILibraryHandle,'nztiee_IsEncrEnabled');
    pointer(nztihe_IsHashEnabled) := GetProcedureAddress(OCILibraryHandle,'nztihe_IsHashEnabled');
    pointer(nztGetIssuerName) := GetProcedureAddress(OCILibraryHandle,'nztGetIssuerName');
    pointer(nztGetSubjectName) := GetProcedureAddress(OCILibraryHandle,'nztGetSubjectName');
    pointer(nztGetBase64Cert) := GetProcedureAddress(OCILibraryHandle,'nztGetBase64Cert');
    pointer(nztGetSerialNumber) := GetProcedureAddress(OCILibraryHandle,'nztGetSerialNumber');
    pointer(nztGetValidDate) := GetProcedureAddress(OCILibraryHandle,'nztGetValidDate');
    pointer(nztGetVersion) := GetProcedureAddress(OCILibraryHandle,'nztGetVersion');
    pointer(nztGetPublicKey) := GetProcedureAddress(OCILibraryHandle,'nztGetPublicKey');
    pointer(nztGenericDestroy) := GetProcedureAddress(OCILibraryHandle,'nztGenericDestroy');
    pointer(nztSetAppDefaultLocation) := GetProcedureAddress(OCILibraryHandle,'nztSetAppDefaultLocation');
    pointer(nztSearchNZDefault) := GetProcedureAddress(OCILibraryHandle,'nztSearchNZDefault');

{ ociap.inc}

    pointer(OCIInitialize) := GetProcedureAddress(OCILibraryHandle,'OCIInitialize');
    pointer(OCITerminate) := GetProcedureAddress(OCILibraryHandle,'OCITerminate');
    pointer(OCIEnvCreate) := GetProcedureAddress(OCILibraryHandle,'OCIEnvCreate');
    pointer(OCIEnvNlsCreate) := GetProcedureAddress(OCILibraryHandle,'OCIEnvNlsCreate');
    pointer(OCIFEnvCreate) := GetProcedureAddress(OCILibraryHandle,'OCIFEnvCreate');
    pointer(OCIHandleAlloc) := GetProcedureAddress(OCILibraryHandle,'OCIHandleAlloc');
    pointer(OCIHandleFree) := GetProcedureAddress(OCILibraryHandle,'OCIHandleFree');
    pointer(OCIDescriptorAlloc) := GetProcedureAddress(OCILibraryHandle,'OCIDescriptorAlloc');
    pointer(OCIDescriptorFree) := GetProcedureAddress(OCILibraryHandle,'OCIDescriptorFree');
    pointer(OCIEnvInit) := GetProcedureAddress(OCILibraryHandle,'OCIEnvInit');
    pointer(OCIServerAttach) := GetProcedureAddress(OCILibraryHandle,'OCIServerAttach');
    pointer(OCIServerDetach) := GetProcedureAddress(OCILibraryHandle,'OCIServerDetach');
    pointer(OCISessionBegin) := GetProcedureAddress(OCILibraryHandle,'OCISessionBegin');
    pointer(OCISessionEnd) := GetProcedureAddress(OCILibraryHandle,'OCISessionEnd');
    pointer(OCILogon) := GetProcedureAddress(OCILibraryHandle,'OCILogon');
    pointer(OCILogon2) := GetProcedureAddress(OCILibraryHandle,'OCILogon2');
    pointer(OCILogoff) := GetProcedureAddress(OCILibraryHandle,'OCILogoff');
    pointer(OCIPasswordChange) := GetProcedureAddress(OCILibraryHandle,'OCIPasswordChange');
    pointer(OCIStmtPrepare) := GetProcedureAddress(OCILibraryHandle,'OCIStmtPrepare');
    pointer(OCIStmtPrepare2) := GetProcedureAddress(OCILibraryHandle,'OCIStmtPrepare2');
    pointer(OCIStmtRelease) := GetProcedureAddress(OCILibraryHandle,'OCIStmtRelease');
    pointer(OCIBindByPos) := GetProcedureAddress(OCILibraryHandle,'OCIBindByPos');
    pointer(OCIBindByName) := GetProcedureAddress(OCILibraryHandle,'OCIBindByName');
    pointer(OCIBindObject) := GetProcedureAddress(OCILibraryHandle,'OCIBindObject');
    pointer(OCIBindDynamic) := GetProcedureAddress(OCILibraryHandle,'OCIBindDynamic');
    pointer(OCIBindArrayOfStruct) := GetProcedureAddress(OCILibraryHandle,'OCIBindArrayOfStruct');
    pointer(OCIStmtGetPieceInfo) := GetProcedureAddress(OCILibraryHandle,'OCIStmtGetPieceInfo');
    pointer(OCIStmtSetPieceInfo) := GetProcedureAddress(OCILibraryHandle,'OCIStmtSetPieceInfo');
    pointer(OCIStmtExecute) := GetProcedureAddress(OCILibraryHandle,'OCIStmtExecute');
    pointer(OCIDefineByPos) := GetProcedureAddress(OCILibraryHandle,'OCIDefineByPos');
    pointer(OCIDefineObject) := GetProcedureAddress(OCILibraryHandle,'OCIDefineObject');
    pointer(OCIDefineDynamic) := GetProcedureAddress(OCILibraryHandle,'OCIDefineDynamic');
    pointer(OCIRowidToChar) := GetProcedureAddress(OCILibraryHandle,'OCIRowidToChar');
    pointer(OCIDefineArrayOfStruct) := GetProcedureAddress(OCILibraryHandle,'OCIDefineArrayOfStruct');
    pointer(OCIStmtFetch) := GetProcedureAddress(OCILibraryHandle,'OCIStmtFetch');
    pointer(OCIStmtFetch2) := GetProcedureAddress(OCILibraryHandle,'OCIStmtFetch2');
    pointer(OCIStmtGetBindInfo) := GetProcedureAddress(OCILibraryHandle,'OCIStmtGetBindInfo');
    pointer(OCIDescribeAny) := GetProcedureAddress(OCILibraryHandle,'OCIDescribeAny');
    pointer(OCIParamGet) := GetProcedureAddress(OCILibraryHandle,'OCIParamGet');
    pointer(OCIParamSet) := GetProcedureAddress(OCILibraryHandle,'OCIParamSet');
    pointer(OCITransStart) := GetProcedureAddress(OCILibraryHandle,'OCITransStart');
    pointer(OCITransDetach) := GetProcedureAddress(OCILibraryHandle,'OCITransDetach');
    pointer(OCITransCommit) := GetProcedureAddress(OCILibraryHandle,'OCITransCommit');
    pointer(OCITransRollback) := GetProcedureAddress(OCILibraryHandle,'OCITransRollback');
    pointer(OCITransPrepare) := GetProcedureAddress(OCILibraryHandle,'OCITransPrepare');
    pointer(OCITransMultiPrepare) := GetProcedureAddress(OCILibraryHandle,'OCITransMultiPrepare');
    pointer(OCITransForget) := GetProcedureAddress(OCILibraryHandle,'OCITransForget');
    pointer(OCIErrorGet) := GetProcedureAddress(OCILibraryHandle,'OCIErrorGet');
    pointer(OCILobAppend) := GetProcedureAddress(OCILibraryHandle,'OCILobAppend');
    pointer(OCILobAssign) := GetProcedureAddress(OCILibraryHandle,'OCILobAssign');
    pointer(OCILobCharSetForm) := GetProcedureAddress(OCILibraryHandle,'OCILobCharSetForm');
    pointer(OCILobCharSetId) := GetProcedureAddress(OCILibraryHandle,'OCILobCharSetId');
    pointer(OCILobCopy) := GetProcedureAddress(OCILibraryHandle,'OCILobCopy');
    pointer(OCILobCreateTemporary) := GetProcedureAddress(OCILibraryHandle,'OCILobCreateTemporary');
    pointer(OCILobClose) := GetProcedureAddress(OCILibraryHandle,'OCILobClose');
    pointer(OCILobDisableBuffering) := GetProcedureAddress(OCILibraryHandle,'OCILobDisableBuffering');
    pointer(OCILobEnableBuffering) := GetProcedureAddress(OCILibraryHandle,'OCILobEnableBuffering');
    pointer(OCILobErase) := GetProcedureAddress(OCILibraryHandle,'OCILobErase');
    pointer(OCILobFileClose) := GetProcedureAddress(OCILibraryHandle,'OCILobFileClose');
    pointer(OCILobFileCloseAll) := GetProcedureAddress(OCILibraryHandle,'OCILobFileCloseAll');
    pointer(OCILobFileExists) := GetProcedureAddress(OCILibraryHandle,'OCILobFileExists');
    pointer(OCILobFileGetName) := GetProcedureAddress(OCILibraryHandle,'OCILobFileGetName');
    pointer(OCILobFileIsOpen) := GetProcedureAddress(OCILibraryHandle,'OCILobFileIsOpen');
    pointer(OCILobFileOpen) := GetProcedureAddress(OCILibraryHandle,'OCILobFileOpen');
    pointer(OCILobFileSetName) := GetProcedureAddress(OCILibraryHandle,'OCILobFileSetName');
    pointer(OCILobFlushBuffer) := GetProcedureAddress(OCILibraryHandle,'OCILobFlushBuffer');
    pointer(OCILobFreeTemporary) := GetProcedureAddress(OCILibraryHandle,'OCILobFreeTemporary');
    pointer(OCILobGetChunkSize) := GetProcedureAddress(OCILibraryHandle,'OCILobGetChunkSize');
    pointer(OCILobGetLength) := GetProcedureAddress(OCILibraryHandle,'OCILobGetLength');
    pointer(OCILobIsEqual) := GetProcedureAddress(OCILibraryHandle,'OCILobIsEqual');
    pointer(OCILobIsOpen) := GetProcedureAddress(OCILibraryHandle,'OCILobIsOpen');
    pointer(OCILobIsTemporary) := GetProcedureAddress(OCILibraryHandle,'OCILobIsTemporary');
    pointer(OCILobLoadFromFile) := GetProcedureAddress(OCILibraryHandle,'OCILobLoadFromFile');
    pointer(OCILobLocatorAssign) := GetProcedureAddress(OCILibraryHandle,'OCILobLocatorAssign');
    pointer(OCILobLocatorIsInit) := GetProcedureAddress(OCILibraryHandle,'OCILobLocatorIsInit');
    pointer(OCILobOpen) := GetProcedureAddress(OCILibraryHandle,'OCILobOpen');
    pointer(OCILobRead) := GetProcedureAddress(OCILibraryHandle,'OCILobRead');
    pointer(OCILobTrim) := GetProcedureAddress(OCILibraryHandle,'OCILobTrim');
    pointer(OCILobWrite) := GetProcedureAddress(OCILibraryHandle,'OCILobWrite');
    pointer(OCILobWriteAppend) := GetProcedureAddress(OCILibraryHandle,'OCILobWriteAppend');
    pointer(OCIBreak) := GetProcedureAddress(OCILibraryHandle,'OCIBreak');
    pointer(OCIReset) := GetProcedureAddress(OCILibraryHandle,'OCIReset');
    pointer(OCIServerVersion) := GetProcedureAddress(OCILibraryHandle,'OCIServerVersion');
    pointer(OCIServerRelease) := GetProcedureAddress(OCILibraryHandle,'OCIServerRelease');
    pointer(OCIAttrGet) := GetProcedureAddress(OCILibraryHandle,'OCIAttrGet');
    pointer(OCIAttrSet) := GetProcedureAddress(OCILibraryHandle,'OCIAttrSet');
    pointer(OCISvcCtxToLda) := GetProcedureAddress(OCILibraryHandle,'OCISvcCtxToLda');
    pointer(OCILdaToSvcCtx) := GetProcedureAddress(OCILibraryHandle,'OCILdaToSvcCtx');
    pointer(OCIResultSetToStmt) := GetProcedureAddress(OCILibraryHandle,'OCIResultSetToStmt');
    pointer(OCIFileClose) := GetProcedureAddress(OCILibraryHandle,'OCIFileClose');
    pointer(OCISharedLibInit) := GetProcedureAddress(OCILibraryHandle,'OCISharedLibInit');
    pointer(OCIFileExists) := GetProcedureAddress(OCILibraryHandle,'OCIFileExists');
    pointer(OCIFileFlush) := GetProcedureAddress(OCILibraryHandle,'OCIFileFlush');
    pointer(OCIFileGetLength) := GetProcedureAddress(OCILibraryHandle,'OCIFileGetLength');
    pointer(OCIFileInit) := GetProcedureAddress(OCILibraryHandle,'OCIFileInit');
    pointer(OCIFileOpen) := GetProcedureAddress(OCILibraryHandle,'OCIFileOpen');
    pointer(OCIFileRead) := GetProcedureAddress(OCILibraryHandle,'OCIFileRead');
    pointer(OCIFileSeek) := GetProcedureAddress(OCILibraryHandle,'OCIFileSeek');
    pointer(OCIFileTerm) := GetProcedureAddress(OCILibraryHandle,'OCIFileTerm');
    pointer(OCIFileWrite) := GetProcedureAddress(OCILibraryHandle,'OCIFileWrite');
{$ifdef ORAXB8_DEFINED}
    pointer(OCILobCopy2) := GetProcedureAddress(OCILibraryHandle,'OCILobCopy2');
    pointer(OCILobErase2) := GetProcedureAddress(OCILibraryHandle,'OCILobErase2');
    pointer(OCILobGetLength2) := GetProcedureAddress(OCILibraryHandle,'OCILobGetLength2');
    pointer(OCILobLoadFromFile2) := GetProcedureAddress(OCILibraryHandle,'OCILobLoadFromFile2');
    pointer(OCILobRead2) := GetProcedureAddress(OCILibraryHandle,'OCILobRead2');
    pointer(OCILobArrayRead) := GetProcedureAddress(OCILibraryHandle,'OCILobArrayRead');
    pointer(OCILobTrim2) := GetProcedureAddress(OCILibraryHandle,'OCILobTrim2');
    pointer(OCILobWrite2) := GetProcedureAddress(OCILibraryHandle,'OCILobWrite2');
    pointer(OCILobArrayWrite) := GetProcedureAddress(OCILibraryHandle,'OCILobArrayWrite');
    pointer(OCILobWriteAppend2) := GetProcedureAddress(OCILibraryHandle,'OCILobWriteAppend2');
    pointer(OCILobGetStorageLimit) := GetProcedureAddress(OCILibraryHandle,'OCILobGetStorageLimit');
{$endif}
    pointer(OCISecurityInitialize) := GetProcedureAddress(OCILibraryHandle,'OCISecurityInitialize');
    pointer(OCISecurityTerminate) := GetProcedureAddress(OCILibraryHandle,'OCISecurityTerminate');
    pointer(OCISecurityOpenWallet) := GetProcedureAddress(OCILibraryHandle,'OCISecurityOpenWallet');
    pointer(OCISecurityCloseWallet) := GetProcedureAddress(OCILibraryHandle,'OCISecurityCloseWallet');
    pointer(OCISecurityCreateWallet) := GetProcedureAddress(OCILibraryHandle,'OCISecurityCreateWallet');
    pointer(OCISecurityDestroyWallet) := GetProcedureAddress(OCILibraryHandle,'OCISecurityDestroyWallet');
    pointer(OCISecurityStorePersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityStorePersona');
    pointer(OCISecurityOpenPersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityOpenPersona');
    pointer(OCISecurityClosePersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityClosePersona');
    pointer(OCISecurityRemovePersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityRemovePersona');
    pointer(OCISecurityCreatePersona) := GetProcedureAddress(OCILibraryHandle,'OCISecurityCreatePersona');
    pointer(OCISecuritySetProtection) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySetProtection');
    pointer(OCISecurityGetProtection) := GetProcedureAddress(OCILibraryHandle,'OCISecurityGetProtection');
    pointer(OCISecurityRemoveIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityRemoveIdentity');
    pointer(OCISecurityCreateIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityCreateIdentity');
    pointer(OCISecurityAbortIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityAbortIdentity');
    pointer(OCISecurityFreeIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityFreeIdentity');
    pointer(OCISecurityStoreTrustedIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityStoreTrustedIdentity');
    pointer(OCISecuritySign) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySign');
    pointer(OCISecuritySignExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySignExpansion');
    pointer(OCISecurityVerify) := GetProcedureAddress(OCILibraryHandle,'OCISecurityVerify');
    pointer(OCISecurityValidate) := GetProcedureAddress(OCILibraryHandle,'OCISecurityValidate');
    pointer(OCISecuritySignDetached) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySignDetached');
    pointer(OCISecuritySignDetExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySignDetExpansion');
    pointer(OCISecurityVerifyDetached) := GetProcedureAddress(OCILibraryHandle,'OCISecurityVerifyDetached');
    pointer(OCISecurity_PKEncrypt) := GetProcedureAddress(OCILibraryHandle,'OCISecurity_PKEncrypt');
    pointer(OCISecurityPKEncryptExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecurityPKEncryptExpansion');
    pointer(OCISecurityPKDecrypt) := GetProcedureAddress(OCILibraryHandle,'OCISecurityPKDecrypt');
    pointer(OCISecurityEncrypt) := GetProcedureAddress(OCILibraryHandle,'OCISecurityEncrypt');
    pointer(OCISecurityEncryptExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecurityEncryptExpansion');
    pointer(OCISecurityDecrypt) := GetProcedureAddress(OCILibraryHandle,'OCISecurityDecrypt');
    pointer(OCISecurityEnvelope) := GetProcedureAddress(OCILibraryHandle,'OCISecurityEnvelope');
    pointer(OCISecurityDeEnvelope) := GetProcedureAddress(OCILibraryHandle,'OCISecurityDeEnvelope');
    pointer(OCISecurityKeyedHash) := GetProcedureAddress(OCILibraryHandle,'OCISecurityKeyedHash');
    pointer(OCISecurityKeyedHashExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecurityKeyedHashExpansion');
    pointer(OCISecurityHash) := GetProcedureAddress(OCILibraryHandle,'OCISecurityHash');
    pointer(OCISecurityHashExpansion) := GetProcedureAddress(OCILibraryHandle,'OCISecurityHashExpansion');
    pointer(OCISecuritySeedRandom) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySeedRandom');
    pointer(OCISecurityRandomBytes) := GetProcedureAddress(OCILibraryHandle,'OCISecurityRandomBytes');
    pointer(OCISecurityRandomNumber) := GetProcedureAddress(OCILibraryHandle,'OCISecurityRandomNumber');
    pointer(OCISecurityInitBlock) := GetProcedureAddress(OCILibraryHandle,'OCISecurityInitBlock');
    pointer(OCISecurityReuseBlock) := GetProcedureAddress(OCILibraryHandle,'OCISecurityReuseBlock');
    pointer(OCISecurityPurgeBlock) := GetProcedureAddress(OCILibraryHandle,'OCISecurityPurgeBlock');
    pointer(OCISecuritySetBlock) := GetProcedureAddress(OCILibraryHandle,'OCISecuritySetBlock');
    pointer(OCISecurityGetIdentity) := GetProcedureAddress(OCILibraryHandle,'OCISecurityGetIdentity');
    pointer(OCIAQEnq) := GetProcedureAddress(OCILibraryHandle,'OCIAQEnq');
    pointer(OCIAQDeq) := GetProcedureAddress(OCILibraryHandle,'OCIAQDeq');
    pointer(OCIAQEnqArray) := GetProcedureAddress(OCILibraryHandle,'OCIAQEnqArray');
    pointer(OCIAQDeqArray) := GetProcedureAddress(OCILibraryHandle,'OCIAQDeqArray');
    pointer(OCIAQListen) := GetProcedureAddress(OCILibraryHandle,'OCIAQListen');
    pointer(OCIAQListen2) := GetProcedureAddress(OCILibraryHandle,'OCIAQListen2');
    pointer(OCIExtractInit) := GetProcedureAddress(OCILibraryHandle,'OCIExtractInit');
    pointer(OCIExtractTerm) := GetProcedureAddress(OCILibraryHandle,'OCIExtractTerm');
    pointer(OCIExtractReset) := GetProcedureAddress(OCILibraryHandle,'OCIExtractReset');
    pointer(OCIExtractSetNumKeys) := GetProcedureAddress(OCILibraryHandle,'OCIExtractSetNumKeys');
    pointer(OCIExtractSetKey) := GetProcedureAddress(OCILibraryHandle,'OCIExtractSetKey');
    pointer(OCIExtractFromFile) := GetProcedureAddress(OCILibraryHandle,'OCIExtractFromFile');
    pointer(OCIExtractFromStr) := GetProcedureAddress(OCILibraryHandle,'OCIExtractFromStr');
    pointer(OCIExtractToInt) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToInt');
    pointer(OCIExtractToBool) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToBool');
    pointer(OCIExtractToStr) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToStr');
    pointer(OCIExtractToOCINum) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToOCINum');
    pointer(OCIExtractToList) := GetProcedureAddress(OCILibraryHandle,'OCIExtractToList');
    pointer(OCIExtractFromList) := GetProcedureAddress(OCILibraryHandle,'OCIExtractFromList');
    pointer(OCIMemoryAlloc) := GetProcedureAddress(OCILibraryHandle,'OCIMemoryAlloc');
    pointer(OCIMemoryResize) := GetProcedureAddress(OCILibraryHandle,'OCIMemoryResize');
    pointer(OCIMemoryFree) := GetProcedureAddress(OCILibraryHandle,'OCIMemoryFree');
    pointer(OCIContextSetValue) := GetProcedureAddress(OCILibraryHandle,'OCIContextSetValue');
    pointer(OCIContextGetValue) := GetProcedureAddress(OCILibraryHandle,'OCIContextGetValue');
    pointer(OCIContextClearValue) := GetProcedureAddress(OCILibraryHandle,'OCIContextClearValue');
    pointer(OCIContextGenerateKey) := GetProcedureAddress(OCILibraryHandle,'OCIContextGenerateKey');
    pointer(OCIMemorySetCurrentIDs) := GetProcedureAddress(OCILibraryHandle,'OCIMemorySetCurrentIDs');
    pointer(OCIPicklerTdsCtxInit) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCtxInit');
    pointer(OCIPicklerTdsCtxFree) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCtxFree');
    pointer(OCIPicklerTdsInit) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsInit');
    pointer(OCIPicklerTdsFree) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsFree');
    pointer(OCIPicklerTdsCreateElementNumber) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElementNumber');
    pointer(OCIPicklerTdsCreateElementChar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElementChar');
    pointer(OCIPicklerTdsCreateElementVarchar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElementVarchar');
    pointer(OCIPicklerTdsCreateElementRaw) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElementRaw');
    pointer(OCIPicklerTdsCreateElement) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsCreateElement');
    pointer(OCIPicklerTdsAddAttr) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsAddAttr');
    pointer(OCIPicklerTdsGenerate) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsGenerate');
    pointer(OCIPicklerTdsGetAttr) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerTdsGetAttr');
    pointer(OCIPicklerFdoInit) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerFdoInit');
    pointer(OCIPicklerFdoFree) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerFdoFree');
    pointer(OCIPicklerImageInit) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageInit');
    pointer(OCIPicklerImageFree) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageFree');
    pointer(OCIPicklerImageAddScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageAddScalar');
    pointer(OCIPicklerImageAddNullScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageAddNullScalar');
    pointer(OCIPicklerImageGenerate) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageGenerate');
    pointer(OCIPicklerImageGetScalarSize) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageGetScalarSize');
    pointer(OCIPicklerImageGetScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageGetScalar');
    pointer(OCIPicklerImageCollBegin) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollBegin');
    pointer(OCIPicklerImageCollAddScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollAddScalar');
    pointer(OCIPicklerImageCollEnd) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollEnd');
    pointer(OCIPicklerImageCollBeginScan) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollBeginScan');
    pointer(OCIPicklerImageCollGetScalarSize) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollGetScalarSize');
    pointer(OCIPicklerImageCollGetScalar) := GetProcedureAddress(OCILibraryHandle,'OCIPicklerImageCollGetScalar');
    pointer(OCIAnyDataGetType) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataGetType');
    pointer(OCIAnyDataIsNull) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataIsNull');
    pointer(OCIAnyDataConvert) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataConvert');
    pointer(OCIAnyDataBeginCreate) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataBeginCreate');
    pointer(OCIAnyDataDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataDestroy');
    pointer(OCIAnyDataAttrSet) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataAttrSet');
    pointer(OCIAnyDataCollAddElem) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataCollAddElem');
    pointer(OCIAnyDataEndCreate) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataEndCreate');
    pointer(OCIAnyDataAccess) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataAccess');
    pointer(OCIAnyDataGetCurrAttrNum) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataGetCurrAttrNum');
    pointer(OCIAnyDataAttrGet) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataAttrGet');
    pointer(OCIAnyDataCollGetElem) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataCollGetElem');
    pointer(OCIAnyDataSetBeginCreate) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetBeginCreate');
    pointer(OCIAnyDataSetDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetDestroy');
    pointer(OCIAnyDataSetAddInstance) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetAddInstance');
    pointer(OCIAnyDataSetEndCreate) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetEndCreate');
    pointer(OCIAnyDataSetGetType) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetGetType');
    pointer(OCIAnyDataSetGetCount) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetGetCount');
    pointer(OCIAnyDataSetGetInstance) := GetProcedureAddress(OCILibraryHandle,'OCIAnyDataSetGetInstance');
    pointer(OCIFormatInit) := GetProcedureAddress(OCILibraryHandle,'OCIFormatInit');
    pointer(OCIFormatString) := GetProcedureAddress(OCILibraryHandle,'OCIFormatString');
    pointer(OCIFormatTerm) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTerm');
    pointer(OCIFormatTUb1) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUb1');
    pointer(OCIFormatTUb2) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUb2');
    pointer(OCIFormatTUb4) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUb4');
    pointer(OCIFormatTUword) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUword');
    pointer(OCIFormatTUbig_ora) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTUbig_ora');
    pointer(OCIFormatTSb1) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSb1');
    pointer(OCIFormatTSb2) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSb2');
    pointer(OCIFormatTSb4) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSb4');
    pointer(OCIFormatTSword) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSword');
    pointer(OCIFormatTSbig_ora) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTSbig_ora');
    pointer(OCIFormatTEb1) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEb1');
    pointer(OCIFormatTEb2) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEb2');
    pointer(OCIFormatTEb4) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEb4');
    pointer(OCIFormatTEword) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEword');
    pointer(OCIFormatTChar) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTChar');
    pointer(OCIFormatTText) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTText');
    pointer(OCIFormatTDouble) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTDouble');
    pointer(OCIFormatTDvoid) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTDvoid');
    pointer(OCIFormatTEnd) := GetProcedureAddress(OCILibraryHandle,'OCIFormatTEnd');
    pointer(xaosvch) := GetProcedureAddress(OCILibraryHandle,'xaosvch');
    pointer(xaoSvcCtx) := GetProcedureAddress(OCILibraryHandle,'xaoSvcCtx');
    pointer(xaoEnv) := GetProcedureAddress(OCILibraryHandle,'xaoEnv');
    pointer(xaosterr) := GetProcedureAddress(OCILibraryHandle,'xaosterr');
    pointer(OCINlsGetInfo) := GetProcedureAddress(OCILibraryHandle,'OCINlsGetInfo');
    pointer(OCINlsNumericInfoGet) := GetProcedureAddress(OCILibraryHandle,'OCINlsNumericInfoGet');
    pointer(OCINlsCharSetNameToId) := GetProcedureAddress(OCILibraryHandle,'OCINlsCharSetNameToId');
    pointer(OCIThreadProcessInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadProcessInit');
    pointer(OCIThreadInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadInit');
    pointer(OCIThreadTerm) := GetProcedureAddress(OCILibraryHandle,'OCIThreadTerm');
    pointer(OCIThreadIsMulti) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIsMulti');
    pointer(OCIThreadMutexInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadMutexInit');
    pointer(OCIThreadMutexDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIThreadMutexDestroy');
    pointer(OCIThreadMutexAcquire) := GetProcedureAddress(OCILibraryHandle,'OCIThreadMutexAcquire');
    pointer(OCIThreadMutexRelease) := GetProcedureAddress(OCILibraryHandle,'OCIThreadMutexRelease');
    pointer(OCIThreadKeyInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadKeyInit');
    pointer(OCIThreadKeyDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIThreadKeyDestroy');
    pointer(OCIThreadKeyGet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadKeyGet');
    pointer(OCIThreadKeySet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadKeySet');
    pointer(OCIThreadIdInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdInit');
    pointer(OCIThreadIdDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdDestroy');
    pointer(OCIThreadIdSet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdSet');
    pointer(OCIThreadIdSetNull) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdSetNull');
    pointer(OCIThreadIdGet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdGet');
    pointer(OCIThreadIdSame) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdSame');
    pointer(OCIThreadIdNull) := GetProcedureAddress(OCILibraryHandle,'OCIThreadIdNull');
    pointer(OCIThreadHndInit) := GetProcedureAddress(OCILibraryHandle,'OCIThreadHndInit');
    pointer(OCIThreadHndDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIThreadHndDestroy');
    pointer(OCIThreadCreate) := GetProcedureAddress(OCILibraryHandle,'OCIThreadCreate');
    pointer(OCIThreadJoin) := GetProcedureAddress(OCILibraryHandle,'OCIThreadJoin');
    pointer(OCIThreadClose) := GetProcedureAddress(OCILibraryHandle,'OCIThreadClose');
    pointer(OCIThreadHandleGet) := GetProcedureAddress(OCILibraryHandle,'OCIThreadHandleGet');
    pointer(OCISubscriptionRegister) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionRegister');
    pointer(OCISubscriptionPost) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionPost');
    pointer(OCISubscriptionUnRegister) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionUnRegister');
    pointer(OCISubscriptionDisable) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionDisable');
    pointer(OCISubscriptionEnable) := GetProcedureAddress(OCILibraryHandle,'OCISubscriptionEnable');
    pointer(OCIDateTimeGetTime) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeGetTime');
    pointer(OCIDateTimeGetDate) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeGetDate');
    pointer(OCIDateTimeGetTimeZoneOffset) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeGetTimeZoneOffset');
    pointer(OCIDateTimeConstruct) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeConstruct');
    pointer(OCIDateTimeSysTimeStamp) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeSysTimeStamp');
    pointer(OCIDateTimeAssign) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeAssign');
    pointer(OCIDateTimeToText) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeToText');
    pointer(OCIDateTimeFromText) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeFromText');
    pointer(OCIDateTimeCompare) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeCompare');
    pointer(OCIDateTimeCheck) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeCheck');
    pointer(OCIDateTimeConvert) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeConvert');
    pointer(OCIDateTimeSubtract) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeSubtract');
    pointer(OCIDateTimeIntervalAdd) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeIntervalAdd');
    pointer(OCIDateTimeIntervalSub) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeIntervalSub');
    pointer(OCIIntervalSubtract) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalSubtract');
    pointer(OCIIntervalAdd) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalAdd');
    pointer(OCIIntervalMultiply) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalMultiply');
    pointer(OCIIntervalDivide) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalDivide');
    pointer(OCIIntervalCompare) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalCompare');
    pointer(OCIIntervalFromNumber) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalFromNumber');
    pointer(OCIIntervalFromText) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalFromText');
    pointer(OCIIntervalToText) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalToText');
    pointer(OCIIntervalToNumber) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalToNumber');
    pointer(OCIIntervalCheck) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalCheck');
    pointer(OCIIntervalAssign) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalAssign');
    pointer(OCIIntervalSetYearMonth) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalSetYearMonth');
    pointer(OCIIntervalGetYearMonth) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalGetYearMonth');
    pointer(OCIIntervalSetDaySecond) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalSetDaySecond');
    pointer(OCIIntervalGetDaySecond) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalGetDaySecond');
    pointer(OCIDateTimeToArray) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeToArray');
    pointer(OCIDateTimeFromArray) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeFromArray');
    pointer(OCIDateTimeGetTimeZoneName) := GetProcedureAddress(OCILibraryHandle,'OCIDateTimeGetTimeZoneName');
    pointer(OCIIntervalFromTZ) := GetProcedureAddress(OCILibraryHandle,'OCIIntervalFromTZ');
    pointer(OCIConnectionPoolCreate) := GetProcedureAddress(OCILibraryHandle,'OCIConnectionPoolCreate');
    pointer(OCIConnectionPoolDestroy) := GetProcedureAddress(OCILibraryHandle,'OCIConnectionPoolDestroy');
    pointer(OCISessionPoolCreate) := GetProcedureAddress(OCILibraryHandle,'OCISessionPoolCreate');
    pointer(OCISessionPoolDestroy) := GetProcedureAddress(OCILibraryHandle,'OCISessionPoolDestroy');
    pointer(OCISessionGet) := GetProcedureAddress(OCILibraryHandle,'OCISessionGet');
    pointer(OCISessionRelease) := GetProcedureAddress(OCILibraryHandle,'OCISessionRelease');
    pointer(OCIAppCtxSet) := GetProcedureAddress(OCILibraryHandle,'OCIAppCtxSet');
    pointer(OCIAppCtxClearAll) := GetProcedureAddress(OCILibraryHandle,'OCIAppCtxClearAll');
    pointer(OCIPing) := GetProcedureAddress(OCILibraryHandle,'OCIPing');
    pointer(OCIKerbAttrSet) := GetProcedureAddress(OCILibraryHandle,'OCIKerbAttrSet');
    pointer(OCIDBStartup) := GetProcedureAddress(OCILibraryHandle,'OCIDBStartup');
    pointer(OCIDBShutdown) := GetProcedureAddress(OCILibraryHandle,'OCIDBShutdown');
    pointer(OCIClientVersion) := GetProcedureAddress(OCILibraryHandle,'OCIClientVersion');
    pointer(OCIInitEventHandle) := GetProcedureAddress(OCILibraryHandle,'OCIInitEventHandle');

    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Procedure ReleaseOCI;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  if RefCount > 0 then 
    dec(RefCount);
  if RefCount = 0 then
    begin
    if not UnloadLibrary(OCILibraryHandle) then 
      inc(RefCount);
    OCILoadedLibrary:='';
=======
=======
>>>>>>> origin/fixes_2_2
  if RefCount > 0 then dec(RefCount);
  if RefCount = 0 then
    begin
    if not UnloadLibrary(OCILibraryHandle) then inc(RefCount);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;

{$ENDIF}


end.
