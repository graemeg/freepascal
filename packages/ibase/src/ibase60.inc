{
}

{$MODE objfpc}
{$MACRO on}

<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
interface

{$IFDEF LinkDynamically}
uses Dynlibs, sysutils,ctypes;

Var
  UseEmbeddedFirebird : Boolean = False;

{$ELSE}
uses Dynlibs,ctypes;
{$ENDIF}

{$IFDEF Unix}
  {$DEFINE extdecl:=cdecl}
  const
<<<<<<< HEAD
<<<<<<< HEAD
    gdslib = 'libgds.'+sharedsuffix; // Needs completion ?
    libfc  = 'libfbclient.'+sharedsuffix;
    libem  = 'libfbembed.'+sharedsuffix;
    v2  = {$ifndef darwin}'.2'{$else}''{$endif};
    v21 = {$ifndef darwin}'.2.1'{$else}''{$endif};
    v25 = {$ifndef darwin}'.2.5'{$else}''{$endif};
    v251 = {$ifndef darwin}'.2.5.1'{$else}''{$endif};
    fbclib2      = libfc+v2;
    fbembedlib2  = libem+v2;
    fbclib21     = libfc+v21;
    fbembedlib21 = libem+v21;
    fbclib25     = libfc+v25;
    fbembedlib25 = libem+v25;
    fbclib251    = libfc+v251;
    fbembedlib251 = libem+v25;
    // Set default here
    fbclib       = fbclib251;
    fbembedlib   = fbembedlib251;
=======
    gdslib = 'libgds.'+sharedsuffix;
    fbclib = 'libfbclient.'+sharedsuffix;
    fbembedlib = 'libfbembed.'+sharedsuffix;
>>>>>>> graemeg/fixes_2_2
=======
    gdslib = 'libgds.'+sharedsuffix;
    fbclib = 'libfbclient.'+sharedsuffix;
    fbembedlib = 'libfbembed.'+sharedsuffix;
>>>>>>> origin/fixes_2_2
{$ENDIF}
{$IFDEF Windows}
  {$DEFINE extdecl:=stdcall}
  const
    gdslib = 'gds32.dll';
    fbclib = 'fbclient.dll';
    fbembedlib = 'fbembed.dll';
<<<<<<< HEAD
<<<<<<< HEAD
    libfc  = fbclib;
    libem  = fbembedlib;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF}

type
  {  Unsigned types }

  UChar                = Byte;
  UShort               = Word;
  UInt                 = DWord;
  ULong                = DWord;

  { Signed types }

  Int                  = LongInt;
  Long                 = LongInt;
  Short                = SmallInt;
  Float                = Single;

  { Pointers to basic types }

  PInt                 = ^Int;
  PShort               = ^Short;
  PUShort              = ^UShort;
  PLong                = ^Long;
  PULong               = ^ULong;
  PFloat               = ^Float;
  PUChar               = ^UChar;
  PVoid                = ^Pointer;

{$PACKRECORDS C}

  const
     ISC_TRUE = 1;
     ISC_FALSE = 0;
  const
     ISC__TRUE = ISC_TRUE;
     ISC__FALSE = ISC_FALSE;

Type
   ISC_USHORT    = word;
<<<<<<< HEAD
<<<<<<< HEAD
   ISC_STATUS    = ptrint;  { CLong according the sources but they ifdef
				for win64, the only significant platform
				where clong<>ptrint}
=======
   ISC_STATUS    = clong;
>>>>>>> graemeg/fixes_2_2
=======
   ISC_STATUS    = clong;
>>>>>>> origin/fixes_2_2
   ISC_INT64     = int64;
   ISC_UINT64    = qword;
   ISC_LONG      = Longint;

   PISC_USHORT = ^ISC_USHORT;
   PISC_STATUS = ^ISC_STATUS;
   PPISC_STATUS = ^PISC_STATUS;
   PISC_INT64 = ^ISC_INT64;
   PISC_UINT64 = ^ISC_UINT64;
   PISC_LONG = ^ISC_LONG;

<<<<<<< HEAD
<<<<<<< HEAD
   {$IFDEF CPU64}
    FB_API_HANDLE = cuint;
   {$ELSE}
    FB_API_HANDLE = pointer;
   {$ENDIF}


  const
     {$IFDEF CPU64}
     FB_API_NULLHANDLE = 0;
     {$ELSE}
     FB_API_NULLHANDLE = nil;
     {$ENDIF}
=======
  const
>>>>>>> graemeg/fixes_2_2
=======
  const
>>>>>>> origin/fixes_2_2
     DSQL_close = 1;
     DSQL_drop = 2;

  {!!MVC
    Removed all ISC_FAR, ISC_EXPORT_VARARG and ISC_EXPORT
    macros.
    They confuse h2pas...
  !!MVC }
  {                                                                  }
  { Time & Date Support                                              }
  {                                                                  }
{$ifndef _ISC_TIMESTAMP_}

  type

     ISC_DATE = longint;
     ISC_TIME = dword;
     ISC_TIMESTAMP = record
          timestamp_date : ISC_DATE;
          timestamp_time : ISC_TIME;
       end;
     TISC_DATE = ISC_DATE;
     TISC_TIME = ISC_TIME;
     TISC_TIMESTAMP = ISC_TIMESTAMP;
     PISC_DATE = ^ISC_DATE;
     PISC_TIME = ^ISC_TIME;
     PISC_TIMESTAMP = ^ISC_TIMESTAMP;
  const
     _ISC_TIMESTAMP_ = 1;
{$endif}

  const
     ISC_TIME_SECONDS_PRECISION = 10000;
     ISC_TIME_SECONDS_PRECISION_SCALE = -4;
  {                                                                  }
  { Blob id structure                                                }
  {                                                                  }

Type

   GDS_QUAD = record
      gds_quad_high : ISC_LONG;
      gds_quad_low : ISC_LONG;
   end;
   TGDS_QUAD = GDS_QUAD;
   PGDS_QUAD = ^GDS_QUAD;

Type
     ISC_QUAD = GDS_QUAD;
     TISC_QUAD = ISC_QUAD;
     PISC_QUAD = ^ISC_QUAD;

{ !!field redefinitions !!
     isc_quad_high = gds_quad_high;
     isc_quad_low = gds_quad_low;
}

type

     ISC_ARRAY_BOUND = record
          array_bound_lower : smallint;
          array_bound_upper : smallint;
       end;
     TISC_ARRAY_BOUND = ISC_ARRAY_BOUND;
     PISC_ARRAY_BOUND = ^ISC_ARRAY_BOUND;

     ISC_ARRAY_DESC = record
          array_desc_dtype : byte;
          array_desc_scale : char;
          array_desc_length : word;
          array_desc_field_name : array[0..31] of char;
          array_desc_relation_name : array[0..31] of char;
          array_desc_dimensions : smallint;
          array_desc_flags : smallint;
          array_desc_bounds : array[0..15] of ISC_ARRAY_BOUND;
       end;
     TISC_ARRAY_DESC = ISC_ARRAY_DESC;
     PISC_ARRAY_DESC = ^ISC_ARRAY_DESC;

     ISC_BLOB_DESC = record
          blob_desc_subtype : smallint;
          blob_desc_charset : smallint;
          blob_desc_segment_size : smallint;
          blob_desc_field_name : array[0..31] of byte;
          blob_desc_relation_name : array[0..31] of byte;
       end;
     TISC_BLOB_DESC = ISC_BLOB_DESC;
     PISC_BLOB_DESC = ^ISC_BLOB_DESC ;
  {                          }
  { Blob control structure   }
  {                          }
  {!!MVC
  !!MVC }
  { Source filter  }
  {!!MVC
  !!MVC }
  { Argument to pass to source  }
  { filter  }
  { Target type  }
  { Source type  }
  { Length of buffer  }
  { Length of current segment  }
  { Length of blob parameter  }
  { block  }
  { Address of blob parameter  }
  { block  }
  { Address of segment buffer  }
  { Length of longest segment  }
  { Total number of segments  }
  { Total length of blob  }
  { Address of status vector  }
  { Application specific data  }
  TCTLSourceFunction  = function : isc_long;

     PISC_BLOB_CTL = ^ISC_BLOB_CTL ;
     ISC_BLOB_CTL = record
          ctl_source : TCTLSourceFunction;     //  was ISC_STATUS ( *ctl_source)();
          ctl_source_handle : pisc_blob_ctl  ; // was struct isc_blob_ctl  * ctl_source_handle;
          ctl_to_sub_type : smallint;
          ctl_from_sub_type : smallint;
          ctl_buffer_length : word;
          ctl_segment_length : word;
          ctl_bpb_length : word;
          ctl_bpb : Pchar;
          ctl_buffer : Pbyte;
          ctl_max_segment : ISC_LONG;
          ctl_number_segments : ISC_LONG;
          ctl_total_length : ISC_LONG;
          ctl_status : PISC_STATUS;
          ctl_data : array[0..7] of longint;
       end;
     TISC_BLOB_CTL = ISC_BLOB_CTL;

  {                          }
  { Blob stream definitions  }
  {                          }
  { Blob handle  }
  { Address of buffer  }
  { Next character  }
  { Length of buffer  }
  { Characters in buffer  }
  { (mode) ? OUTPUT : INPUT  }

     BSTREAM = record
          bstr_blob : pointer;
          bstr_buffer : Pchar;
          bstr_ptr : Pchar;
          bstr_length : smallint;
          bstr_cnt : smallint;
          bstr_mode : char;
       end;
     TBSTREAM = BSTREAM;
     PBstream = ^BSTREAM;
  {!!MVC

  #define getb(p)       (--(p)->bstr_cnt >= 0 ?  (p)->bstr_ptr++ & 0377: BLOB_get (p))
  #define putb(x,p) (((x) == '\n' || (!(--(p)->bstr_cnt))) ? BLOB_put ((x),p) : ((int) ( (p)->bstr_ptr++ = (unsigned) (x))))
  #define putbx(x,p) ((!(--(p)->bstr_cnt)) ? BLOB_put ((x),p) : ((int) ( (p)->bstr_ptr++ = (unsigned) (x))))

  !!MVC  }
  {                          }
  { Dynamic SQL definitions  }
  {                          }
  {                             }
  { Declare the extended SQLDA  }
  {                             }
  { datatype of field  }
  { scale factor  }
  { datatype subtype - BLOBs & Text  }
  { types only  }
  { length of data area  }
  { address of data  }
  { address of indicator variable  }
  { length of sqlname field  }
  { name of field, name length + space  }
  { for NULL  }
  { length of relation name  }
  { field's relation name + space for  }
  { NULL  }
  { length of owner name  }
  { relation's owner name + space for  }
  { NULL  }
  { length of alias name  }
  { relation's alias name + space for  }
  { NULL  }

     XSQLVAR = record
          sqltype : smallint;
          sqlscale : smallint;
          sqlsubtype : smallint;
          sqllen : smallint;
          sqldata : Pchar;
          sqlind : Psmallint;
          sqlname_length : smallint;
          sqlname : array[0..31] of char;
          relname_length : smallint;
          relname : array[0..31] of char;
          ownname_length : smallint;
          ownname : array[0..31] of char;
          aliasname_length : smallint;
          aliasname : array[0..31] of char;
       end;
     TXSQLVAR = XSQLVAR;
     PXSQLVAR =^XSQLVAR;
  { version of this XSQLDA  }
  { XSQLDA name field  }
  { length in bytes of SQLDA  }
  { number of fields allocated  }
  { actual number of fields  }
  { first field address  }

     XSQLDA = record
          version : smallint;
          sqldaid : array[0..7] of char;
          sqldabc : ISC_LONG;
          sqln : smallint;
          sqld : smallint;
          sqlvar : array[0..0] of XSQLVAR;
       end;
     TXSQLDA = XSQLDA;
     PXSQLDA =^XSQLDA;

  function XSQLDA_LENGTH(n: Integer): Integer;


  const
     SQLDA_VERSION1 = 1;
  { meaning is same as DIALECT_xsqlda  }
     SQL_DIALECT_V5 = 1;
  { flagging anything that is delimited
                                              by double quotes as an error and
                                              flagging keyword DATE as an error  }
     SQL_DIALECT_V6_TRANSITION = 2;
  { supports SQL delimited identifier,
                                              SQLDATE/DATE, TIME, TIMESTAMP,
                                              CURRENT_DATE, CURRENT_TIME,
                                              CURRENT_TIMESTAMP, and 64-bit exact
                                              numeric type  }
     SQL_DIALECT_V6 = 3;
  { latest IB DIALECT  }
     SQL_DIALECT_CURRENT = SQL_DIALECT_V6;
  {                               }
  { InterBase Handle Definitions  }
  {                               }

  type
<<<<<<< HEAD
<<<<<<< HEAD

     isc_att_handle  = FB_API_HANDLE;
     isc_blob_handle = FB_API_HANDLE;
     isc_db_handle   = FB_API_HANDLE;
     isc_form_handle = FB_API_HANDLE; // can't find in 2.5.1
     isc_req_handle  = FB_API_HANDLE;
     isc_stmt_handle = FB_API_HANDLE;
     isc_svc_handle  = FB_API_HANDLE;
     isc_tr_handle   = FB_API_HANDLE;
     isc_win_handle  = pointer;       // can't find in 2.5.1 
=======
=======
>>>>>>> origin/fixes_2_2
     isc_att_handle = pointer;
     isc_blob_handle = pointer;
     isc_db_handle = pointer;
     isc_form_handle = pointer;
     isc_req_handle = pointer;
     isc_stmt_handle = pointer;
     isc_svc_handle = pointer;
     isc_tr_handle = pointer;
     isc_win_handle = pointer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     isc_callback = procedure ;extdecl;
     isc_resv_handle = ISC_LONG;
     tisc_att_handle = isc_att_handle;
     tisc_blob_handle = isc_blob_handle;
     tisc_db_handle = isc_db_handle;
     tisc_form_handle = isc_form_handle;
     tisc_req_handle = isc_req_handle;
     tisc_stmt_handle = isc_stmt_handle;
     tisc_svc_handle = isc_svc_handle;
     tisc_tr_handle = isc_tr_handle;
     tisc_win_handle = isc_win_handle;
     tisc_callback = isc_callback;
     tisc_resv_handle = isc_resv_handle;
     pisc_att_handle  =^isc_att_handle ;
     pisc_blob_handle  =^isc_blob_handle ;
     pisc_db_handle  =^isc_db_handle ;
     pisc_form_handle  =^isc_form_handle ;
     pisc_req_handle  =^isc_req_handle ;
     pisc_stmt_handle  =^isc_stmt_handle ;
     pisc_svc_handle  =^isc_svc_handle ;
     pisc_tr_handle  =^isc_tr_handle ;
     pisc_win_handle  =^isc_win_handle ;
     pisc_callback  = ^isc_callback;
     pisc_resv_handle  =^isc_resv_handle ;


  {                                    }
  { Security structures                }
  {                                    }

  const
     sec_uid_spec = $01;
     sec_gid_spec = $02;
     sec_server_spec = $04;
     sec_password_spec = $08;
     sec_group_name_spec = $10;
     sec_first_name_spec = $20;
     sec_middle_name_spec = $40;
     sec_last_name_spec = $80;
     sec_dba_user_name_spec = $100;
     sec_dba_password_spec = $200;
     sec_protocol_tcpip = 1;
     sec_protocol_netbeui = 2;
     sec_protocol_spx = 3;
     sec_protocol_local = 4;
  { which fields are specified  }
  { the user's id  }
  { the user's group id  }
  { protocol to use for connection  }
  { server to administer  }
  { the user's name  }
  { the user's password  }
  { the group name  }
  { the user's first name  }
  { the user's middle name  }
  { the user's last name  }
  { the dba user name  }
  { the dba password  }

  type

     USER_SEC_DATA = record
          sec_flags : smallint;
          uid : longint;
          gid : longint;
          protocol : longint;
          server : Pchar;
          user_name : Pchar;
          password : Pchar;
          group_name : Pchar;
          first_name : Pchar;
          middle_name : Pchar;
          last_name : Pchar;
          dba_user_name : Pchar;
          dba_password : Pchar;
       end;
     TUSER_SEC_DATA = USER_SEC_DATA;
     PUSER_SEC_DATA = ^USER_SEC_DATA;

  {                                                  }
  { Actions to pass to the blob filter (ctl_source)  }
  {                                                  }

  const
     isc_blob_filter_open = 0;
     isc_blob_filter_get_segment = 1;
     isc_blob_filter_close = 2;
     isc_blob_filter_create = 3;
     isc_blob_filter_put_segment = 4;
     isc_blob_filter_alloc = 5;
     isc_blob_filter_free = 6;
     isc_blob_filter_seek = 7;
  {                  }
  { Blr definitions  }
  {                  }
{$ifndef _JRD_BLR_H_}
  {!!MVC
  #define blr_word(n) ((n) % 256), ((n) / 256)
  !!MVC }

  const
     blr_text = 14;
     blr_text2 = 15;
     blr_short = 7;
     blr_long = 8;
     blr_quad = 9;
     blr_int64 = 16;
     blr_float = 10;
     blr_double = 27;
     blr_d_float = 11;
     blr_timestamp = 35;
     blr_varying = 37;
     blr_varying2 = 38;
     blr_blob = 261;
     blr_cstring = 40;
     blr_cstring2 = 41;
     blr_blob_id = 45;
     blr_sql_date = 12;
     blr_sql_time = 13;
  { Historical alias for pre V6 applications  }
     blr_date = blr_timestamp;
     blr_inner = 0;
     blr_left = 1;
     blr_right = 2;
     blr_full = 3;
     blr_gds_code = 0;
     blr_sql_code = 1;
     blr_exception = 2;
     blr_trigger_code = 3;
     blr_default_code = 4;
     blr_version4 = 4;
     blr_version5 = 5;
     blr_eoc = 76;
     blr_end = 255;
     blr_assignment = 1;
     blr_begin = 2;
     blr_dcl_variable = 3;
     blr_message = 4;
     blr_erase = 5;
     blr_fetch = 6;
     blr_for = 7;
     blr_if = 8;
     blr_loop = 9;
     blr_modify = 10;
     blr_handler = 11;
     blr_receive = 12;
     blr_select = 13;
     blr_send = 14;
     blr_store = 15;
     blr_label = 17;
     blr_leave = 18;
     blr_store2 = 19;
     blr_post = 20;
     blr_literal = 21;
     blr_dbkey = 22;
     blr_field = 23;
     blr_fid = 24;
     blr_parameter = 25;
     blr_variable = 26;
     blr_average = 27;
     blr_count = 28;
     blr_maximum = 29;
     blr_minimum = 30;
     blr_total = 31;
     blr_add = 34;
     blr_subtract = 35;
     blr_multiply = 36;
     blr_divide = 37;
     blr_negate = 38;
     blr_concatenate = 39;
     blr_substring = 40;
     blr_parameter2 = 41;
     blr_from = 42;
     blr_via = 43;
     blr_user_name = 44;
     blr_null = 45;
     blr_eql = 47;
     blr_neq = 48;
     blr_gtr = 49;
     blr_geq = 50;
     blr_lss = 51;
     blr_leq = 52;
     blr_containing = 53;
     blr_matching = 54;
     blr_starting = 55;
     blr_between = 56;
     blr_or = 57;
     blr_and = 58;
     blr_not = 59;
     blr_any = 60;
     blr_missing = 61;
     blr_unique = 62;
     blr_like = 63;
     blr_stream = 65;
     blr_set_index = 66;
     blr_rse = 67;
     blr_first = 68;
     blr_project = 69;
     blr_sort = 70;
     blr_boolean = 71;
     blr_ascending = 72;
     blr_descending = 73;
     blr_relation = 74;
     blr_rid = 75;
     blr_union = 76;
     blr_map = 77;
     blr_group_by = 78;
     blr_aggregate = 79;
     blr_join_type = 80;
     blr_agg_count = 83;
     blr_agg_max = 84;
     blr_agg_min = 85;
     blr_agg_total = 86;
     blr_agg_average = 87;
     blr_parameter3 = 88;
     blr_run_count = 118;
     blr_run_max = 89;
     blr_run_min = 90;
     blr_run_total = 91;
     blr_run_average = 92;
     blr_agg_count2 = 93;
     blr_agg_count_distinct = 94;
     blr_agg_total_distinct = 95;
     blr_agg_average_distinct = 96;
     blr_function = 100;
     blr_gen_id = 101;
     blr_prot_mask = 102;
     blr_upcase = 103;
     blr_lock_state = 104;
     blr_value_if = 105;
     blr_matching2 = 106;
     blr_index = 107;
     blr_ansi_like = 108;
     blr_bookmark = 109;
     blr_crack = 110;
     blr_force_crack = 111;
     blr_seek = 112;
     blr_find = 113;
     blr_continue = 0;
     blr_forward = 1;
     blr_backward = 2;
     blr_bof_forward = 3;
     blr_eof_backward = 4;
     blr_lock_relation = 114;
     blr_lock_record = 115;
     blr_set_bookmark = 116;
     blr_get_bookmark = 117;
     blr_rs_stream = 119;
     blr_exec_proc = 120;
     blr_begin_range = 121;
     blr_end_range = 122;
     blr_delete_range = 123;
     blr_procedure = 124;
     blr_pid = 125;
     blr_exec_pid = 126;
     blr_singular = 127;
     blr_abort = 128;
     blr_block = 129;
     blr_error_handler = 130;
     blr_cast = 131;
     blr_release_lock = 132;
     blr_release_locks = 133;
     blr_start_savepoint = 134;
     blr_end_savepoint = 135;
     blr_find_dbkey = 136;
     blr_range_relation = 137;
     blr_delete_ranges = 138;
     blr_plan = 139;
     blr_merge = 140;
     blr_join = 141;
     blr_sequential = 142;
     blr_navigational = 143;
     blr_indices = 144;
     blr_retrieve = 145;
     blr_relation2 = 146;
     blr_rid2 = 147;
     blr_reset_stream = 148;
     blr_release_bookmark = 149;
     blr_set_generator = 150;
     blr_ansi_any = 151;
     blr_exists = 152;
     blr_cardinality = 153;
  { get tid of record  }
     blr_record_version = 154;
  { fake server stall  }
     blr_stall = 155;
     blr_seek_no_warn = 156;
     blr_find_dbkey_version = 157;
     blr_ansi_all = 158;
     blr_extract = 159;
  { sub parameters for blr_extract  }
     blr_extract_year = 0;
     blr_extract_month = 1;
     blr_extract_day = 2;
     blr_extract_hour = 3;
     blr_extract_minute = 4;
     blr_extract_second = 5;
     blr_extract_weekday = 6;
     blr_extract_yearday = 7;
     blr_current_date = 160;
     blr_current_timestamp = 161;
     blr_current_time = 162;
  { These verbs were added in 6.0, primarily to support 64-bit integers  }
     blr_add2 = 163;
     blr_subtract2 = 164;
     blr_multiply2 = 165;
     blr_divide2 = 166;
     blr_agg_total2 = 167;
     blr_agg_total_distinct2 = 168;
     blr_agg_average2 = 169;
     blr_agg_average_distinct2 = 170;
     blr_average2 = 171;
     blr_gen_id2 = 172;
     blr_set_generator2 = 173;
{$endif}
  { _JRD_BLR_H_  }
  {                                 }
  { Database parameter block stuff  }
  {                                 }

  const
     isc_dpb_version1 = 1;
     isc_dpb_cdd_pathname = 1;
     isc_dpb_allocation = 2;
     isc_dpb_journal = 3;
     isc_dpb_page_size = 4;
     isc_dpb_num_buffers = 5;
     isc_dpb_buffer_length = 6;
     isc_dpb_debug = 7;
     isc_dpb_garbage_collect = 8;
     isc_dpb_verify = 9;
     isc_dpb_sweep = 10;
     isc_dpb_enable_journal = 11;
     isc_dpb_disable_journal = 12;
     isc_dpb_dbkey_scope = 13;
     isc_dpb_number_of_users = 14;
     isc_dpb_trace = 15;
     isc_dpb_no_garbage_collect = 16;
     isc_dpb_damaged = 17;
     isc_dpb_license = 18;
     isc_dpb_sys_user_name = 19;
     isc_dpb_encrypt_key = 20;
     isc_dpb_activate_shadow = 21;
     isc_dpb_sweep_interval = 22;
     isc_dpb_delete_shadow = 23;
     isc_dpb_force_write = 24;
     isc_dpb_begin_log = 25;
     isc_dpb_quit_log = 26;
     isc_dpb_no_reserve = 27;
     isc_dpb_user_name = 28;
     isc_dpb_password = 29;
     isc_dpb_password_enc = 30;
     isc_dpb_sys_user_name_enc = 31;
     isc_dpb_interp = 32;
     isc_dpb_online_dump = 33;
     isc_dpb_old_file_size = 34;
     isc_dpb_old_num_files = 35;
     isc_dpb_old_file = 36;
     isc_dpb_old_start_page = 37;
     isc_dpb_old_start_seqno = 38;
     isc_dpb_old_start_file = 39;
     isc_dpb_drop_walfile = 40;
     isc_dpb_old_dump_id = 41;
     isc_dpb_wal_backup_dir = 42;
     isc_dpb_wal_chkptlen = 43;
     isc_dpb_wal_numbufs = 44;
     isc_dpb_wal_bufsize = 45;
     isc_dpb_wal_grp_cmt_wait = 46;
     isc_dpb_lc_messages = 47;
     isc_dpb_lc_ctype = 48;
     isc_dpb_cache_manager = 49;
     isc_dpb_shutdown = 50;
     isc_dpb_online = 51;
     isc_dpb_shutdown_delay = 52;
     isc_dpb_reserved = 53;
     isc_dpb_overwrite = 54;
     isc_dpb_sec_attach = 55;
     isc_dpb_disable_wal = 56;
     isc_dpb_connect_timeout = 57;
     isc_dpb_dummy_packet_interval = 58;
     isc_dpb_gbak_attach = 59;
     isc_dpb_sql_role_name = 60;
     isc_dpb_set_page_buffers = 61;
     isc_dpb_working_directory = 62;
     isc_dpb_SQL_dialect = 63;
     isc_dpb_set_db_readonly = 64;
     isc_dpb_set_db_SQL_dialect = 65;
     isc_dpb_gfix_attach = 66;
     isc_dpb_gstat_attach = 67;
  {                                }
  { isc_dpb_verify specific flags  }
  {                                }
     isc_dpb_pages = 1;
     isc_dpb_records = 2;
     isc_dpb_indices = 4;
     isc_dpb_transactions = 8;
     isc_dpb_no_update = 16;
     isc_dpb_repair = 32;
     isc_dpb_ignore = 64;
  {                                  }
  { isc_dpb_shutdown specific flags  }
  {                                  }
     isc_dpb_shut_cache = 1;
     isc_dpb_shut_attachment = 2;
     isc_dpb_shut_transaction = 4;
     isc_dpb_shut_force = 8;
  {                                     }
  { Bit assignments in RDB$SYSTEM_FLAG  }
  {                                     }
     RDB_system = 1;
     RDB_id_assigned = 2;
  {                                    }
  { Transaction parameter block stuff  }
  {                                    }
     isc_tpb_version1 = 1;
     isc_tpb_version3 = 3;
     isc_tpb_consistency = 1;
     isc_tpb_concurrency = 2;
     isc_tpb_shared = 3;
     isc_tpb_protected = 4;
     isc_tpb_exclusive = 5;
     isc_tpb_wait = 6;
     isc_tpb_nowait = 7;
     isc_tpb_read = 8;
     isc_tpb_write = 9;
     isc_tpb_lock_read = 10;
     isc_tpb_lock_write = 11;
     isc_tpb_verb_time = 12;
     isc_tpb_commit_time = 13;
     isc_tpb_ignore_limbo = 14;
     isc_tpb_read_committed = 15;
     isc_tpb_autocommit = 16;
     isc_tpb_rec_version = 17;
     isc_tpb_no_rec_version = 18;
     isc_tpb_restart_requests = 19;
     isc_tpb_no_auto_undo = 20;
  {                       }
  { Blob Parameter Block  }
  {                       }
     isc_bpb_version1 = 1;
     isc_bpb_source_type = 1;
     isc_bpb_target_type = 2;
     isc_bpb_type = 3;
     isc_bpb_source_interp = 4;
     isc_bpb_target_interp = 5;
     isc_bpb_filter_parameter = 6;
     isc_bpb_type_segmented = 0;
     isc_bpb_type_stream = 1;
  {                                }
  { Service parameter block stuff  }
  {                                }
     isc_spb_version1 = 1;
     isc_spb_current_version = 2;
     isc_spb_version = isc_spb_current_version;
     isc_spb_user_name = isc_dpb_user_name;
     isc_spb_sys_user_name = isc_dpb_sys_user_name;
     isc_spb_sys_user_name_enc = isc_dpb_sys_user_name_enc;
     isc_spb_password = isc_dpb_password;
     isc_spb_password_enc = isc_dpb_password_enc;
     isc_spb_command_line = 105;
     isc_spb_dbname = 106;
     isc_spb_verbose = 107;
     isc_spb_options = 108;
     isc_spb_connect_timeout = isc_dpb_connect_timeout;
     isc_spb_dummy_packet_interval = isc_dpb_dummy_packet_interval;
     isc_spb_sql_role_name = isc_dpb_sql_role_name;
  {                                }
  { Information call declarations  }
  {                                }
  {                           }
  { Common, structural codes  }
  {                           }
     isc_info_end = 1;
     isc_info_truncated = 2;
     isc_info_error = 3;
     isc_info_data_not_ready = 4;
     isc_info_flag_end = 127;
  {                             }
  { Database information items  }
  {                             }
     isc_info_db_id = 4;
     isc_info_reads = 5;
     isc_info_writes = 6;
     isc_info_fetches = 7;
     isc_info_marks = 8;
     isc_info_implementation = 11;
     isc_info_version = 12;
     isc_info_base_level = 13;
     isc_info_page_size = 14;
     isc_info_num_buffers = 15;
     isc_info_limbo = 16;
     isc_info_current_memory = 17;
     isc_info_max_memory = 18;
     isc_info_window_turns = 19;
     isc_info_license = 20;
     isc_info_allocation = 21;
     isc_info_attachment_id = 22;
     isc_info_read_seq_count = 23;
     isc_info_read_idx_count = 24;
     isc_info_insert_count = 25;
     isc_info_update_count = 26;
     isc_info_delete_count = 27;
     isc_info_backout_count = 28;
     isc_info_purge_count = 29;
     isc_info_expunge_count = 30;
     isc_info_sweep_interval = 31;
     isc_info_ods_version = 32;
     isc_info_ods_minor_version = 33;
     isc_info_no_reserve = 34;
     isc_info_logfile = 35;
     isc_info_cur_logfile_name = 36;
     isc_info_cur_log_part_offset = 37;
     isc_info_num_wal_buffers = 38;
     isc_info_wal_buffer_size = 39;
     isc_info_wal_ckpt_length = 40;
     isc_info_wal_cur_ckpt_interval = 41;
     isc_info_wal_prv_ckpt_fname = 42;
     isc_info_wal_prv_ckpt_poffset = 43;
     isc_info_wal_recv_ckpt_fname = 44;
     isc_info_wal_recv_ckpt_poffset = 45;
     isc_info_wal_grpc_wait_usecs = 47;
     isc_info_wal_num_io = 48;
     isc_info_wal_avg_io_size = 49;
     isc_info_wal_num_commits = 50;
     isc_info_wal_avg_grpc_size = 51;
     isc_info_forced_writes = 52;
     isc_info_user_names = 53;
     isc_info_page_errors = 54;
     isc_info_record_errors = 55;
     isc_info_bpage_errors = 56;
     isc_info_dpage_errors = 57;
     isc_info_ipage_errors = 58;
     isc_info_ppage_errors = 59;
     isc_info_tpage_errors = 60;
     isc_info_set_page_buffers = 61;
     isc_info_db_SQL_dialect = 62;
     isc_info_db_read_only = 63;
     isc_info_db_size_in_pages = 64;
  {                                     }
  { Database information return values  }
  {                                     }
     isc_info_db_impl_rdb_vms = 1;
     isc_info_db_impl_rdb_eln = 2;
     isc_info_db_impl_rdb_eln_dev = 3;
     isc_info_db_impl_rdb_vms_y = 4;
     isc_info_db_impl_rdb_eln_y = 5;
     isc_info_db_impl_jri = 6;
     isc_info_db_impl_jsv = 7;
     isc_info_db_impl_isc_a = 25;
     isc_info_db_impl_isc_u = 26;
     isc_info_db_impl_isc_v = 27;
     isc_info_db_impl_isc_s = 28;
     isc_info_db_impl_isc_apl_68K = 25;
     isc_info_db_impl_isc_vax_ultr = 26;
     isc_info_db_impl_isc_vms = 27;
     isc_info_db_impl_isc_sun_68k = 28;
     isc_info_db_impl_isc_os2 = 29;
     isc_info_db_impl_isc_sun4 = 30;
     isc_info_db_impl_isc_hp_ux = 31;
     isc_info_db_impl_isc_sun_386i = 32;
     isc_info_db_impl_isc_vms_orcl = 33;
     isc_info_db_impl_isc_mac_aux = 34;
     isc_info_db_impl_isc_rt_aix = 35;
     isc_info_db_impl_isc_mips_ult = 36;
     isc_info_db_impl_isc_xenix = 37;
     isc_info_db_impl_isc_dg = 38;
     isc_info_db_impl_isc_hp_mpexl = 39;
     isc_info_db_impl_isc_hp_ux68K = 40;
     isc_info_db_impl_isc_sgi = 41;
     isc_info_db_impl_isc_sco_unix = 42;
     isc_info_db_impl_isc_cray = 43;
     isc_info_db_impl_isc_imp = 44;
     isc_info_db_impl_isc_delta = 45;
     isc_info_db_impl_isc_next = 46;
     isc_info_db_impl_isc_dos = 47;
     isc_info_db_impl_isc_winnt = 48;
     isc_info_db_impl_isc_epson = 49;
     isc_info_db_class_access = 1;
     isc_info_db_class_y_valve = 2;
     isc_info_db_class_rem_int = 3;
     isc_info_db_class_rem_srvr = 4;
     isc_info_db_class_pipe_int = 7;
     isc_info_db_class_pipe_srvr = 8;
     isc_info_db_class_sam_int = 9;
     isc_info_db_class_sam_srvr = 10;
     isc_info_db_class_gateway = 11;
     isc_info_db_class_cache = 12;
  {                            }
  { Request information items  }
  {                            }
     isc_info_number_messages = 4;
     isc_info_max_message = 5;
     isc_info_max_send = 6;
     isc_info_max_receive = 7;
     isc_info_state = 8;
     isc_info_message_number = 9;
     isc_info_message_size = 10;
     isc_info_request_cost = 11;
     isc_info_access_path = 12;
     isc_info_req_select_count = 13;
     isc_info_req_insert_count = 14;
     isc_info_req_update_count = 15;
     isc_info_req_delete_count = 16;
  {                    }
  { Access path items  }
  {                    }
     isc_info_rsb_end = 0;
     isc_info_rsb_begin = 1;
     isc_info_rsb_type = 2;
     isc_info_rsb_relation = 3;
     isc_info_rsb_plan = 4;
  {            }
  { Rsb types  }
  {            }
     isc_info_rsb_unknown = 1;
     isc_info_rsb_indexed = 2;
     isc_info_rsb_navigate = 3;
     isc_info_rsb_sequential = 4;
     isc_info_rsb_cross = 5;
     isc_info_rsb_sort = 6;
     isc_info_rsb_first = 7;
     isc_info_rsb_boolean = 8;
     isc_info_rsb_union = 9;
     isc_info_rsb_aggregate = 10;
     isc_info_rsb_merge = 11;
     isc_info_rsb_ext_sequential = 12;
     isc_info_rsb_ext_indexed = 13;
     isc_info_rsb_ext_dbkey = 14;
     isc_info_rsb_left_cross = 15;
     isc_info_rsb_select = 16;
     isc_info_rsb_sql_join = 17;
     isc_info_rsb_simulate = 18;
     isc_info_rsb_sim_cross = 19;
     isc_info_rsb_once = 20;
     isc_info_rsb_procedure = 21;
  {                     }
  { Bitmap expressions  }
  {                     }
     isc_info_rsb_and = 1;
     isc_info_rsb_or = 2;
     isc_info_rsb_dbkey = 3;
     isc_info_rsb_index = 4;
     isc_info_req_active = 2;
     isc_info_req_inactive = 3;
     isc_info_req_send = 4;
     isc_info_req_receive = 5;
     isc_info_req_select = 6;
     isc_info_req_sql_stall = 7;
  {                         }
  { Blob information items  }
  {                         }
     isc_info_blob_num_segments = 4;
     isc_info_blob_max_segment = 5;
     isc_info_blob_total_length = 6;
     isc_info_blob_type = 7;
  {                                }
  { Transaction information items  }
  {                                }
     isc_info_tra_id = 4;
  {
     Service action items
                                }
  { Starts database backup process on the server  }
     isc_action_svc_backup = 1;
  { Starts database restore process on the server  }
     isc_action_svc_restore = 2;
  { Starts database repair process on the server  }
     isc_action_svc_repair = 3;
  { Adds a new user to the security database  }
     isc_action_svc_add_user = 4;
  { Deletes a user record from the security database  }
     isc_action_svc_delete_user = 5;
  { Modifies a user record in the security database  }
     isc_action_svc_modify_user = 6;
  { Displays a user record from the security database  }
     isc_action_svc_display_user = 7;
  { Sets database properties  }
     isc_action_svc_properties = 8;
  { Adds a license to the license file  }
     isc_action_svc_add_license = 9;
  { Removes a license from the license file  }
     isc_action_svc_remove_license = 10;
  { Retrieves database statistics  }
     isc_action_svc_db_stats = 11;
  { Retrieves the InterBase log file from the server  }
     isc_action_svc_get_ib_log = 12;
  {
     Service information items
                                }
  { Retrieves the number of attachments and databases  }
     isc_info_svc_svr_db_info = 50;
  { Retrieves all license keys and IDs from the license file  }
     isc_info_svc_get_license = 51;
  { Retrieves a bitmask representing licensed options on the server  }
     isc_info_svc_get_license_mask = 52;
  { Retrieves the parameters and values for IB_CONFIG  }
     isc_info_svc_get_config = 53;
  { Retrieves the version of the services manager  }
     isc_info_svc_version = 54;
  { Retrieves the version of the InterBase server  }
     isc_info_svc_server_version = 55;
  { Retrieves the implementation of the InterBase server  }
     isc_info_svc_implementation = 56;
  { Retrieves a bitmask representing the server's capabilities  }
     isc_info_svc_capabilities = 57;
  { Retrieves the path to the security database in use by the server  }
     isc_info_svc_user_dbpath = 58;
  { Retrieves the setting of $INTERBASE  }
     isc_info_svc_get_env = 59;
  { Retrieves the setting of $INTERBASE_LCK  }
     isc_info_svc_get_env_lock = 60;
  { Retrieves the setting of $INTERBASE_MSG  }
     isc_info_svc_get_env_msg = 61;
  { Retrieves 1 line of service output per call  }
     isc_info_svc_line = 62;
  { Retrieves as much of the server output as will fit in the supplied buffer  }
     isc_info_svc_to_eof = 63;
  { Sets / signifies a timeout value for reading service information  }
     isc_info_svc_timeout = 64;
  { Retrieves the number of users licensed for accessing the server  }
     isc_info_svc_get_licensed_users = 65;
  { Retrieve the limbo transactions  }
     isc_info_svc_limbo_trans = 66;
  { Checks to see if a service is running on an attachment  }
     isc_info_svc_running = 67;
  { Returns the user information from isc_action_svc_display_users  }
     isc_info_svc_get_users = 68;
  {
     Parameters for isc_action_(add|delete|modify)_user
   }
     isc_spb_sec_userid = 5;
     isc_spb_sec_groupid = 6;
     isc_spb_sec_username = 7;
     isc_spb_sec_password = 8;
     isc_spb_sec_groupname = 9;
     isc_spb_sec_firstname = 10;
     isc_spb_sec_middlename = 11;
     isc_spb_sec_lastname = 12;
  {
     Parameters for isc_action_svc_(add|remove)_license,
     isc_info_svc_get_license
                                                          }
     isc_spb_lic_key = 5;
     isc_spb_lic_id = 6;
     isc_spb_lic_desc = 7;
  {
     Parameters for isc_action_svc_backup
                                            }
     isc_spb_bkp_file = 5;
     isc_spb_bkp_factor = 6;
     isc_spb_bkp_length = 7;
     isc_spb_bkp_ignore_checksums = $01;
     isc_spb_bkp_ignore_limbo = $02;
     isc_spb_bkp_metadata_only = $04;
     isc_spb_bkp_no_garbage_collect = $08;
     isc_spb_bkp_old_descriptions = $10;
     isc_spb_bkp_non_transportable = $20;
     isc_spb_bkp_convert = $40;
     isc_spb_bkp_expand = $80;
  {
     Parameters for isc_action_svc_properties
                                               }
     isc_spb_prp_page_buffers = 5;
     isc_spb_prp_sweep_interval = 6;
     isc_spb_prp_shutdown_db = 7;
     isc_spb_prp_deny_new_attachments = 9;
     isc_spb_prp_deny_new_transactions = 10;
     isc_spb_prp_reserve_space = 11;
     isc_spb_prp_write_mode = 12;
     isc_spb_prp_access_mode = 13;
     isc_spb_prp_set_sql_dialect = 14;
     isc_spb_prp_activate = $0100;
     isc_spb_prp_db_online = $0200;
  {
     Parameters for isc_spb_prp_reserve_space
                                               }
     isc_spb_prp_res_use_full = 35;
     isc_spb_prp_res = 36;
  {
     Parameters for isc_spb_prp_write_mode
                                             }
     isc_spb_prp_wm_async = 37;
     isc_spb_prp_wm_sync = 38;
  {
     Parameters for isc_spb_prp_access_mode
                                             }
     isc_spb_prp_am_readonly = 39;
     isc_spb_prp_am_readwrite = 40;
  {
     Parameters for isc_action_svc_repair
                                            }
     isc_spb_rpr_commit_trans = 15;
     isc_spb_rpr_rollback_trans = 34;
     isc_spb_rpr_recover_two_phase = 17;
     isc_spb_tra_id = 18;
     isc_spb_single_tra_id = 19;
     isc_spb_multi_tra_id = 20;
     isc_spb_tra_state = 21;
     isc_spb_tra_state_limbo = 22;
     isc_spb_tra_state_commit = 23;
     isc_spb_tra_state_rollback = 24;
     isc_spb_tra_state_unknown = 25;
     isc_spb_tra_host_site = 26;
     isc_spb_tra_remote_site = 27;
     isc_spb_tra_db_path = 28;
     isc_spb_tra_advise = 29;
     isc_spb_tra_advise_commit = 30;
     isc_spb_tra_advise_rollback = 31;
     isc_spb_tra_advise_unknown = 33;
     isc_spb_rpr_validate_db = $01;
     isc_spb_rpr_sweep_db = $02;
     isc_spb_rpr_mend_db = $04;
     isc_spb_rpr_list_limbo_trans = $08;
     isc_spb_rpr_check_db = $10;
     isc_spb_rpr_ignore_checksum = $20;
     isc_spb_rpr_kill_shadows = $40;
     isc_spb_rpr_full = $80;
  {
     Parameters for isc_action_svc_restore
                                            }
     isc_spb_res_buffers = 9;
     isc_spb_res_page_size = 10;
     isc_spb_res_length = 11;
     isc_spb_res_access_mode = 12;
<<<<<<< HEAD
<<<<<<< HEAD
     isc_spb_res_fix_fss_data = 13;
     isc_spb_res_fix_fss_metadata = 14;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     isc_spb_res_deactivate_idx = $0100;
     isc_spb_res_no_shadow = $0200;
     isc_spb_res_no_validity = $0400;
     isc_spb_res_one_at_a_time = $0800;
     isc_spb_res_replace = $1000;
     isc_spb_res_create = $2000;
     isc_spb_res_use_all_space = $4000;
  {
     Parameters for isc_spb_res_access_mode
                                             }
     isc_spb_res_am_readonly = isc_spb_prp_am_readonly;
     isc_spb_res_am_readwrite = isc_spb_prp_am_readwrite;
  {
     Parameters for isc_info_svc_svr_db_info
                                              }
     isc_spb_num_att = 5;
     isc_spb_num_db = 6;
  {
     Parameters for isc_info_svc_db_stats
                                            }
     isc_spb_sts_data_pages = $01;
     isc_spb_sts_db_log = $02;
     isc_spb_sts_hdr_pages = $04;
     isc_spb_sts_idx_pages = $08;
     isc_spb_sts_sys_relations = $10;
<<<<<<< HEAD
<<<<<<< HEAD
     isc_spb_sts_record_versions = $20;
     isc_spb_sts_table = $40;
     isc_spb_sts_nocreation = $80;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {                        }
  { SQL information items  }
  {                        }
     isc_info_sql_select = 4;
     isc_info_sql_bind = 5;
     isc_info_sql_num_variables = 6;
     isc_info_sql_describe_vars = 7;
     isc_info_sql_describe_end = 8;
     isc_info_sql_sqlda_seq = 9;
     isc_info_sql_message_seq = 10;
     isc_info_sql_type = 11;
     isc_info_sql_sub_type = 12;
     isc_info_sql_scale = 13;
     isc_info_sql_length = 14;
     isc_info_sql_null_ind = 15;
     isc_info_sql_field = 16;
     isc_info_sql_relation = 17;
     isc_info_sql_owner = 18;
     isc_info_sql_alias = 19;
     isc_info_sql_sqlda_start = 20;
     isc_info_sql_stmt_type = 21;
     isc_info_sql_get_plan = 22;
     isc_info_sql_records = 23;
     isc_info_sql_batch_fetch = 24;
  {                                }
  { SQL information return values  }
  {                                }
     isc_info_sql_stmt_select = 1;
     isc_info_sql_stmt_insert = 2;
     isc_info_sql_stmt_update = 3;
     isc_info_sql_stmt_delete = 4;
     isc_info_sql_stmt_ddl = 5;
     isc_info_sql_stmt_get_segment = 6;
     isc_info_sql_stmt_put_segment = 7;
     isc_info_sql_stmt_exec_procedure = 8;
     isc_info_sql_stmt_start_trans = 9;
     isc_info_sql_stmt_commit = 10;
     isc_info_sql_stmt_rollback = 11;
     isc_info_sql_stmt_select_for_upd = 12;
     isc_info_sql_stmt_set_generator = 13;
  {                                  }
  { Server configuration key values  }
  {                                  }
     ISCCFG_LOCKMEM_KEY = 0;
     ISCCFG_LOCKSEM_KEY = 1;
     ISCCFG_LOCKSIG_KEY = 2;
     ISCCFG_EVNTMEM_KEY = 3;
     ISCCFG_DBCACHE_KEY = 4;
     ISCCFG_PRIORITY_KEY = 5;
     ISCCFG_IPCMAP_KEY = 6;
     ISCCFG_MEMMIN_KEY = 7;
     ISCCFG_MEMMAX_KEY = 8;
     ISCCFG_LOCKORDER_KEY = 9;
     ISCCFG_ANYLOCKMEM_KEY = 10;
     ISCCFG_ANYLOCKSEM_KEY = 11;
     ISCCFG_ANYLOCKSIG_KEY = 12;
     ISCCFG_ANYEVNTMEM_KEY = 13;
     ISCCFG_LOCKHASH_KEY = 14;
     ISCCFG_DEADLOCK_KEY = 15;
     ISCCFG_LOCKSPIN_KEY = 16;
     ISCCFG_CONN_TIMEOUT_KEY = 17;
     ISCCFG_DUMMY_INTRVL_KEY = 18;
  { Internal Use only  }
     ISCCFG_TRACE_POOLS_KEY = 19;
     ISCCFG_REMOTE_BUFFER_KEY = 20;
  {              }
  { Error codes  }
  {              }
     isc_facility = 20;
     isc_err_base = 335544320;
     isc_err_factor = 1;
     isc_arg_end = 0;
     isc_arg_gds = 1;
     isc_arg_string = 2;
     isc_arg_cstring = 3;
     isc_arg_number = 4;
     isc_arg_interpreted = 5;
     isc_arg_vms = 6;
     isc_arg_unix = 7;
     isc_arg_domain = 8;
     isc_arg_dos = 9;
     isc_arg_mpexl = 10;
     isc_arg_mpexl_ipc = 11;
     isc_arg_next_mach = 15;
     isc_arg_netware = 16;
     isc_arg_win32 = 17;
     isc_arg_warning = 18;

  {                                             }
  { Dynamic Data Definition Language operators  }
  {                                             }
  {                 }
  { Version number  }
  {                 }

  const
     isc_dyn_version_1 = 1;
     isc_dyn_eoc = 255;
  {                             }
  { Operations (may be nested)  }
  {                             }
     isc_dyn_begin = 2;
     isc_dyn_end = 3;
     isc_dyn_if = 4;
     isc_dyn_def_database = 5;
     isc_dyn_def_global_fld = 6;
     isc_dyn_def_local_fld = 7;
     isc_dyn_def_idx = 8;
     isc_dyn_def_rel = 9;
     isc_dyn_def_sql_fld = 10;
     isc_dyn_def_view = 12;
     isc_dyn_def_trigger = 15;
     isc_dyn_def_security_class = 120;
     isc_dyn_def_dimension = 140;
     isc_dyn_def_generator = 24;
     isc_dyn_def_function = 25;
     isc_dyn_def_filter = 26;
     isc_dyn_def_function_arg = 27;
     isc_dyn_def_shadow = 34;
     isc_dyn_def_trigger_msg = 17;
     isc_dyn_def_file = 36;
     isc_dyn_mod_database = 39;
     isc_dyn_mod_rel = 11;
     isc_dyn_mod_global_fld = 13;
     isc_dyn_mod_idx = 102;
     isc_dyn_mod_local_fld = 14;
     isc_dyn_mod_sql_fld = 216;
     isc_dyn_mod_view = 16;
     isc_dyn_mod_security_class = 122;
     isc_dyn_mod_trigger = 113;
     isc_dyn_mod_trigger_msg = 28;
     isc_dyn_delete_database = 18;
     isc_dyn_delete_rel = 19;
     isc_dyn_delete_global_fld = 20;
     isc_dyn_delete_local_fld = 21;
     isc_dyn_delete_idx = 22;
     isc_dyn_delete_security_class = 123;
     isc_dyn_delete_dimensions = 143;
     isc_dyn_delete_trigger = 23;
     isc_dyn_delete_trigger_msg = 29;
     isc_dyn_delete_filter = 32;
     isc_dyn_delete_function = 33;
     isc_dyn_delete_shadow = 35;
     isc_dyn_grant = 30;
     isc_dyn_revoke = 31;
     isc_dyn_def_primary_key = 37;
     isc_dyn_def_foreign_key = 38;
     isc_dyn_def_unique = 40;
     isc_dyn_def_procedure = 164;
     isc_dyn_delete_procedure = 165;
     isc_dyn_def_parameter = 135;
     isc_dyn_delete_parameter = 136;
     isc_dyn_mod_procedure = 175;
     isc_dyn_def_log_file = 176;
     isc_dyn_def_cache_file = 180;
     isc_dyn_def_exception = 181;
     isc_dyn_mod_exception = 182;
     isc_dyn_del_exception = 183;
     isc_dyn_drop_log = 194;
     isc_dyn_drop_cache = 195;
     isc_dyn_def_default_log = 202;
  {                      }
  { View specific stuff  }
  {                      }
     isc_dyn_view_blr = 43;
     isc_dyn_view_source = 44;
     isc_dyn_view_relation = 45;
     isc_dyn_view_context = 46;
     isc_dyn_view_context_name = 47;
  {                     }
  { Generic attributes  }
  {                     }
     isc_dyn_rel_name = 50;
     isc_dyn_fld_name = 51;
     isc_dyn_new_fld_name = 215;
     isc_dyn_idx_name = 52;
     isc_dyn_description = 53;
     isc_dyn_security_class = 54;
     isc_dyn_system_flag = 55;
     isc_dyn_update_flag = 56;
     isc_dyn_prc_name = 166;
     isc_dyn_prm_name = 137;
     isc_dyn_sql_object = 196;
     isc_dyn_fld_character_set_name = 174;
  {                               }
  { Relation specific attributes  }
  {                               }
     isc_dyn_rel_dbkey_length = 61;
     isc_dyn_rel_store_trig = 62;
     isc_dyn_rel_modify_trig = 63;
     isc_dyn_rel_erase_trig = 64;
     isc_dyn_rel_store_trig_source = 65;
     isc_dyn_rel_modify_trig_source = 66;
     isc_dyn_rel_erase_trig_source = 67;
     isc_dyn_rel_ext_file = 68;
     isc_dyn_rel_sql_protection = 69;
     isc_dyn_rel_constraint = 162;
     isc_dyn_delete_rel_constraint = 163;
  {                                   }
  { Global field specific attributes  }
  {                                   }
     isc_dyn_fld_type = 70;
     isc_dyn_fld_length = 71;
     isc_dyn_fld_scale = 72;
     isc_dyn_fld_sub_type = 73;
     isc_dyn_fld_segment_length = 74;
     isc_dyn_fld_query_header = 75;
     isc_dyn_fld_edit_string = 76;
     isc_dyn_fld_validation_blr = 77;
     isc_dyn_fld_validation_source = 78;
     isc_dyn_fld_computed_blr = 79;
     isc_dyn_fld_computed_source = 80;
     isc_dyn_fld_missing_value = 81;
     isc_dyn_fld_default_value = 82;
     isc_dyn_fld_query_name = 83;
     isc_dyn_fld_dimensions = 84;
     isc_dyn_fld_not_null = 85;
     isc_dyn_fld_precision = 86;
     isc_dyn_fld_char_length = 172;
     isc_dyn_fld_collation = 173;
     isc_dyn_fld_default_source = 193;
     isc_dyn_del_default = 197;
     isc_dyn_del_validation = 198;
     isc_dyn_single_validation = 199;
     isc_dyn_fld_character_set = 203;
  {                                  }
  { Local field specific attributes  }
  {                                  }
     isc_dyn_fld_source = 90;
     isc_dyn_fld_base_fld = 91;
     isc_dyn_fld_position = 92;
     isc_dyn_fld_update_flag = 93;
  {                            }
  { Index specific attributes  }
  {                            }
     isc_dyn_idx_unique = 100;
     isc_dyn_idx_inactive = 101;
     isc_dyn_idx_type = 103;
     isc_dyn_idx_foreign_key = 104;
     isc_dyn_idx_ref_column = 105;
     isc_dyn_idx_statistic = 204;
  {                              }
  { Trigger specific attributes  }
  {                              }
     isc_dyn_trg_type = 110;
     isc_dyn_trg_blr = 111;
     isc_dyn_trg_source = 112;
     isc_dyn_trg_name = 114;
     isc_dyn_trg_sequence = 115;
     isc_dyn_trg_inactive = 116;
     isc_dyn_trg_msg_number = 117;
     isc_dyn_trg_msg = 118;
  {                                     }
  { Security Class specific attributes  }
  {                                     }
     isc_dyn_scl_acl = 121;
     isc_dyn_grant_user = 130;
     isc_dyn_grant_proc = 186;
     isc_dyn_grant_trig = 187;
     isc_dyn_grant_view = 188;
     isc_dyn_grant_options = 132;
     isc_dyn_grant_user_group = 205;
  {                                 }
  { Dimension specific information  }
  {                                 }
     isc_dyn_dim_lower = 141;
     isc_dyn_dim_upper = 142;
  {                           }
  { File specific attributes  }
  {                           }
     isc_dyn_file_name = 125;
     isc_dyn_file_start = 126;
     isc_dyn_file_length = 127;
     isc_dyn_shadow_number = 128;
     isc_dyn_shadow_man_auto = 129;
     isc_dyn_shadow_conditional = 130;
  {                               }
  { Log file specific attributes  }
  {                               }
     isc_dyn_log_file_sequence = 177;
     isc_dyn_log_file_partitions = 178;
     isc_dyn_log_file_serial = 179;
     isc_dyn_log_file_overflow = 200;
     isc_dyn_log_file_raw = 201;
  {                          }
  { Log specific attributes  }
  {                          }
     isc_dyn_log_group_commit_wait = 189;
     isc_dyn_log_buffer_size = 190;
     isc_dyn_log_check_point_length = 191;
     isc_dyn_log_num_of_buffers = 192;
  {                               }
  { Function specific attributes  }
  {                               }
     isc_dyn_function_name = 145;
     isc_dyn_function_type = 146;
     isc_dyn_func_module_name = 147;
     isc_dyn_func_entry_point = 148;
     isc_dyn_func_return_argument = 149;
     isc_dyn_func_arg_position = 150;
     isc_dyn_func_mechanism = 151;
     isc_dyn_filter_in_subtype = 152;
     isc_dyn_filter_out_subtype = 153;
     isc_dyn_description2 = 154;
     isc_dyn_fld_computed_source2 = 155;
     isc_dyn_fld_edit_string2 = 156;
     isc_dyn_fld_query_header2 = 157;
     isc_dyn_fld_validation_source2 = 158;
     isc_dyn_trg_msg2 = 159;
     isc_dyn_trg_source2 = 160;
     isc_dyn_view_source2 = 161;
     isc_dyn_xcp_msg2 = 184;
  {                                }
  { Generator specific attributes  }
  {                                }
     isc_dyn_generator_name = 95;
     isc_dyn_generator_id = 96;
  {                                }
  { Procedure specific attributes  }
  {                                }
     isc_dyn_prc_inputs = 167;
     isc_dyn_prc_outputs = 168;
     isc_dyn_prc_source = 169;
     isc_dyn_prc_blr = 170;
     isc_dyn_prc_source2 = 171;
  {                                }
  { Parameter specific attributes  }
  {                                }
     isc_dyn_prm_number = 138;
     isc_dyn_prm_type = 139;
  {                               }
  { Relation specific attributes  }
  {                               }
     isc_dyn_xcp_msg = 185;
  {                                             }
  { Cascading referential integrity values      }
  {                                             }
     isc_dyn_foreign_key_update = 205;
     isc_dyn_foreign_key_delete = 206;
     isc_dyn_foreign_key_cascade = 207;
     isc_dyn_foreign_key_default = 208;
     isc_dyn_foreign_key_null = 209;
     isc_dyn_foreign_key_none = 210;
  {                      }
  { SQL role values      }
  {                      }
     isc_dyn_def_sql_role = 211;
     isc_dyn_sql_role_name = 212;
     isc_dyn_grant_admin_options = 213;
     isc_dyn_del_sql_role = 214;
  {                           }
  { Last $dyn value assigned  }
  {                           }
     isc_dyn_last_dyn_value = 216;
  {                                         }
  { Array slice description language (SDL)  }
  {                                         }
     isc_sdl_version1 = 1;
     isc_sdl_eoc = 255;
     isc_sdl_relation = 2;
     isc_sdl_rid = 3;
     isc_sdl_field = 4;
     isc_sdl_fid = 5;
     isc_sdl_struct = 6;
     isc_sdl_variable = 7;
     isc_sdl_scalar = 8;
     isc_sdl_tiny_integer = 9;
     isc_sdl_short_integer = 10;
     isc_sdl_long_integer = 11;
     isc_sdl_literal = 12;
     isc_sdl_add = 13;
     isc_sdl_subtract = 14;
     isc_sdl_multiply = 15;
     isc_sdl_divide = 16;
     isc_sdl_negate = 17;
     isc_sdl_eql = 18;
     isc_sdl_neq = 19;
     isc_sdl_gtr = 20;
     isc_sdl_geq = 21;
     isc_sdl_lss = 22;
     isc_sdl_leq = 23;
     isc_sdl_and = 24;
     isc_sdl_or = 25;
     isc_sdl_not = 26;
     isc_sdl_while = 27;
     isc_sdl_assignment = 28;
     isc_sdl_label = 29;
     isc_sdl_leave = 30;
     isc_sdl_begin = 31;
     isc_sdl_end = 32;
     isc_sdl_do3 = 33;
     isc_sdl_do2 = 34;
     isc_sdl_do1 = 35;
     isc_sdl_element = 36;
  {                                           }
  { International text interpretation values  }
  {                                           }
     isc_interp_eng_ascii = 0;
     isc_interp_jpn_sjis = 5;
     isc_interp_jpn_euc = 6;
  {                  }
  { SQL definitions  }
  {                  }
     SQL_TEXT = 452;
     SQL_VARYING = 448;
     SQL_SHORT = 500;
     SQL_LONG = 496;
     SQL_FLOAT = 482;
     SQL_DOUBLE = 480;
     SQL_D_FLOAT = 530;
     SQL_TIMESTAMP = 510;
     SQL_BLOB = 520;
     SQL_ARRAY = 540;
     SQL_QUAD = 550;
     SQL_TYPE_TIME = 560;
     SQL_TYPE_DATE = 570;
     SQL_INT64 = 580;
  { Historical alias for pre V6 applications  }
     SQL_DATE = SQL_TIMESTAMP;
  {                }
  { Blob Subtypes  }
  {                }
  { types less than zero are reserved for customer use  }
     isc_blob_untyped = 0;
  { internal subtypes  }
     isc_blob_text = 1;
     isc_blob_blr = 2;
     isc_blob_acl = 3;
     isc_blob_ranges = 4;
     isc_blob_summary = 5;
     isc_blob_format = 6;
     isc_blob_tra = 7;
     isc_blob_extfile = 8;
  { the range 20-30 is reserved for dBASE and Paradox types  }
     isc_blob_formatted_memo = 20;
     isc_blob_paradox_ole = 21;
     isc_blob_graphic = 22;
     isc_blob_dbase_ole = 23;
     isc_blob_typed_binary = 24;

{$IFNDEF LinkDynamically}

  {                          }
  { OSRI database functions  }
  {                          }

  function isc_attach_database(_para1:PISC_STATUS; _para2:smallint; _para3:Pchar; _para4:Pisc_db_handle; _para5:smallint;
             _para6:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_array_gen_sdl(_para1:PISC_STATUS; _para2:PISC_ARRAY_DESC; _para3:Psmallint; _para4:Pchar; _para5:Psmallint):ISC_STATUS; extdecl; external gdslib;

  function isc_array_get_slice(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:PISC_QUAD; _para5:PISC_ARRAY_DESC;
             _para6:pointer; _para7:PISC_LONG):ISC_STATUS; extdecl; external gdslib;

  function isc_array_lookup_bounds(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pchar; _para5:Pchar;
             _para6:PISC_ARRAY_DESC):ISC_STATUS; extdecl; external gdslib;

  function isc_array_lookup_desc(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pchar; _para5:Pchar;
             _para6:PISC_ARRAY_DESC):ISC_STATUS; extdecl; external gdslib;

  function isc_array_set_desc(_para1:PISC_STATUS; _para2:Pchar; _para3:Pchar; _para4:Psmallint; _para5:Psmallint;
             _para6:Psmallint; _para7:PISC_ARRAY_DESC):ISC_STATUS; extdecl; external gdslib;

  function isc_array_put_slice(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:PISC_QUAD; _para5:PISC_ARRAY_DESC;
             _para6:pointer; _para7:PISC_LONG):ISC_STATUS; extdecl; external gdslib;

  procedure isc_blob_default_desc(_para1:PISC_BLOB_DESC; _para2:Pbyte; _para3:Pbyte); extdecl; external gdslib;

  function isc_blob_gen_bpb(_para1:PISC_STATUS; _para2:PISC_BLOB_DESC; _para3:PISC_BLOB_DESC; _para4:word; _para5:Pbyte;
             _para6:Pword):ISC_STATUS; extdecl; external gdslib;

  function isc_blob_info(_para1:PISC_STATUS; _para2:Pisc_blob_handle; _para3:smallint; _para4:Pchar; _para5:smallint;
             _para6:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_blob_lookup_desc(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pbyte; _para5:Pbyte;
             _para6:PISC_BLOB_DESC; _para7:Pbyte):ISC_STATUS; extdecl; external gdslib;

  function isc_blob_set_desc(_para1:PISC_STATUS; _para2:Pbyte; _para3:Pbyte; _para4:smallint; _para5:smallint;
             _para6:smallint; _para7:PISC_BLOB_DESC):ISC_STATUS; extdecl; external gdslib;

  function isc_cancel_blob(_para1:PISC_STATUS; _para2:Pisc_blob_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_cancel_events(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:PISC_LONG):ISC_STATUS; extdecl; external gdslib;

  function isc_close_blob(_para1:PISC_STATUS; _para2:Pisc_blob_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_commit_retaining(_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_commit_transaction(_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_create_blob(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_blob_handle; _para5:PISC_QUAD):ISC_STATUS; extdecl; external gdslib;

  function isc_create_blob2(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_blob_handle; _para5:PISC_QUAD;
             _para6:smallint; _para7:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_create_database(_para1:PISC_STATUS; _para2:smallint; _para3:Pchar; _para4:Pisc_db_handle; _para5:smallint;
             _para6:Pchar; _para7:smallint):ISC_STATUS; extdecl; external gdslib;

  function isc_database_info(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:smallint; _para4:Pchar; _para5:smallint;
             _para6:Pchar):ISC_STATUS; extdecl; external gdslib;

  procedure isc_decode_date(_para1:PISC_QUAD; _para2:pointer); extdecl; external gdslib;

  procedure isc_decode_sql_date(_para1:PISC_DATE; _para2:pointer); extdecl; external gdslib;

  procedure isc_decode_sql_time(_para1:PISC_TIME; _para2:pointer); extdecl; external gdslib;

  procedure isc_decode_timestamp(_para1:PISC_TIMESTAMP; _para2:pointer); extdecl; external gdslib;

  function isc_detach_database(_para1:PISC_STATUS; _para2:Pisc_db_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_drop_database(_para1:PISC_STATUS; _para2:Pisc_db_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_allocate_statement(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_stmt_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_alloc_statement2(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_stmt_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_describe(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_describe_bind(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_exec_immed2(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:PXSQLDA; _para8:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_execute(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_execute2(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:PXSQLDA;
             _para6:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_execute_immediate(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_fetch(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_finish(_para1:Pisc_db_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_free_statement(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_insert(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_prepare(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_set_cursor_name(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:Pchar; _para4:word):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_sql_info(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:smallint; _para4:Pchar; _para5:smallint;
             _para6:Pchar):ISC_STATUS; extdecl; external gdslib;

  procedure isc_encode_date(_para1:pointer; _para2:PISC_QUAD); extdecl; external gdslib;

  procedure isc_encode_sql_date(_para1:pointer; _para2:PISC_DATE); extdecl; external gdslib;

  procedure isc_encode_sql_time(_para1:pointer; _para2:PISC_TIME); extdecl; external gdslib;

  procedure isc_encode_timestamp(_para1:pointer; _para2:PISC_TIMESTAMP); extdecl; external gdslib;

  function isc_event_block(_para1:PPchar; _para2:PPchar; _para3:word; args:array of const):ISC_LONG; cdecl; external gdslib;

  procedure isc_event_counts(_para1: PISC_STATUS; _para2: short; _para3: pchar; _para4: pchar); extdecl;  external gdslib;

  procedure isc_expand_dpb(_para1:PPchar; _para2:Psmallint; args:array of const); cdecl; external gdslib;

  function isc_modify_dpb(_para1:PPchar; _para2:Psmallint; _para3:word; _para4:Pchar; _para5:smallint):longint; extdecl; external gdslib;

  function isc_free(_para1:Pchar):ISC_LONG; extdecl; external gdslib;

  function isc_get_segment(_para1:PISC_STATUS; _para2:Pisc_blob_handle; _para3:Pword; _para4:word; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_get_slice(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:PISC_QUAD; _para5:smallint;
             _para6:Pchar; _para7:smallint; _para8:PISC_LONG; _para9:ISC_LONG; _para10:pointer;
             _para11:PISC_LONG):ISC_STATUS; extdecl; external gdslib;

  function isc_interprete(_para1:Pchar; _para2:PPISC_STATUS):ISC_STATUS; extdecl; external gdslib;

  function isc_open_blob(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_blob_handle; _para5:PISC_QUAD):ISC_STATUS; extdecl; external gdslib;

  function isc_open_blob2(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_blob_handle; _para5:PISC_QUAD;
             _para6:smallint; _para7:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_prepare_transaction2(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint; _para4:Pchar):ISC_STATUS; extdecl; external gdslib;

  procedure isc_print_sqlerror(_para1:smallint; _para2:PISC_STATUS); extdecl; external gdslib;

  function isc_print_status(_para1:PISC_STATUS):ISC_STATUS; extdecl; external gdslib;

  function isc_put_segment(_para1:PISC_STATUS; _para2:Pisc_blob_handle; _para3:word; _para4:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_put_slice(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:PISC_QUAD; _para5:smallint;
             _para6:Pchar; _para7:smallint; _para8:PISC_LONG; _para9:ISC_LONG; _para10:pointer):ISC_STATUS; extdecl; external gdslib;

  function isc_que_events(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:PISC_LONG; _para4:smallint; _para5:Pchar;
             _para6:isc_callback; _para7:pointer):ISC_STATUS; extdecl; external gdslib;

  function isc_rollback_retaining(_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_rollback_transaction(_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_start_multiple(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint; _para4:pointer):ISC_STATUS; extdecl; external gdslib;

  function isc_start_transaction(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint; args:array of const):ISC_STATUS; cdecl; external gdslib;

  function isc_sqlcode(_para1:PISC_STATUS):ISC_LONG; extdecl; external gdslib;

<<<<<<< HEAD
<<<<<<< HEAD
  // Available in Firebird 2.5+
  function fb_sqlstate(_para1: PAnsiChar;_para2:PISC_STATUS): ISC_STATUS ; extdecl;  external gdslib;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  procedure isc_sql_interprete(_para1:smallint; _para2:Pchar; _para3:smallint); extdecl; external gdslib;

  function isc_transaction_info(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint; _para4:Pchar; _para5:smallint;
             _para6:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_transact_request(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:Pchar; _para8:word; _para9:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_vax_integer(_para1:Pchar; _para2:smallint):ISC_LONG; extdecl; external gdslib;

  function isc_portable_integer(_para1:Pbyte; _para2:smallint):ISC_INT64; extdecl; external gdslib;

  {                                    }
  { Security Functions                 }
  {                                    }


  function isc_add_user(_para1:PISC_STATUS; _para2:PUSER_SEC_DATA):longint; extdecl; external gdslib;

  function isc_delete_user(_para1:PISC_STATUS; _para2:PUSER_SEC_DATA):longint; extdecl; external gdslib;

  function isc_modify_user(_para1:PISC_STATUS; _para2:PUSER_SEC_DATA):longint; extdecl; external gdslib;

  {                                 }
  {  Other OSRI functions           }
  {                                 }
  function isc_compile_request(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_req_handle; _para4:smallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_compile_request2(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_req_handle; _para4:smallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_ddl(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:smallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_prepare_transaction(_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_receive(_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:smallint; _para4:smallint; _para5:pointer;
             _para6:smallint):ISC_STATUS; extdecl; external gdslib;

  function isc_reconnect_transaction(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:smallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_release_request(_para1:PISC_STATUS; _para2:Pisc_req_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_request_info(_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:smallint; _para4:smallint; _para5:Pchar;
             _para6:smallint; _para7:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_seek_blob(_para1:PISC_STATUS; _para2:Pisc_blob_handle; _para3:smallint; _para4:ISC_LONG; _para5:PISC_LONG):ISC_STATUS; extdecl; external gdslib;

  function isc_send(_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:smallint; _para4:smallint; _para5:pointer;
             _para6:smallint):ISC_STATUS; extdecl; external gdslib;

  function isc_start_and_send(_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:Pisc_tr_handle; _para4:smallint; _para5:smallint;
             _para6:pointer; _para7:smallint):ISC_STATUS; extdecl; external gdslib;

  function isc_start_request(_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:Pisc_tr_handle; _para4:smallint):ISC_STATUS; extdecl; external gdslib;

  function isc_unwind_request(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint):ISC_STATUS; extdecl; external gdslib;

  function isc_wait_for_event(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:smallint; _para4:Pchar; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  {                            }
  { Other Sql functions        }
  {                            }
  function isc_close(_para1:PISC_STATUS; _para2:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_declare(_para1:PISC_STATUS; _para2:Pchar; _para3:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_describe(_para1:PISC_STATUS; _para2:Pchar; _para3:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_describe_bind(_para1:PISC_STATUS; _para2:Pchar; _para3:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_execute(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_execute_immediate(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_fetch(_para1:PISC_STATUS; _para2:Pchar; _para3:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_open(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_prepare(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pchar; _para5:Psmallint;
             _para6:Pchar; _para7:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  {                                    }
  { Other Dynamic sql functions        }
  {                                    }
  function isc_dsql_execute_m(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:word; _para8:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_execute2_m(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:word; _para8:Pchar; _para9:word; _para10:Pchar;
             _para11:word; _para12:word; _para13:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_execute_immediate_m(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:word; _para8:Pchar; _para9:word; _para10:word;
             _para11:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_exec_immed3_m(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:word; _para8:Pchar; _para9:word; _para10:word;
             _para11:Pchar; _para12:word; _para13:Pchar; _para14:word; _para15:word;
             _para16:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_fetch_m(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:Pchar; _para5:word;
             _para6:word; _para7:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_insert_m(_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:Pchar; _para5:word;
             _para6:word; _para7:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_prepare_m(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:word; _para8:Pchar; _para9:word; _para10:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_dsql_release(_para1:PISC_STATUS; _para2:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_close(_para1:PISC_STATUS; _para2:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_declare(_para1:PISC_STATUS; _para2:Pchar; _para3:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_describe(_para1:PISC_STATUS; _para2:Pchar; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_describe_bind(_para1:PISC_STATUS; _para2:Pchar; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_execute(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:word; _para5:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_execute2(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:word; _para5:PXSQLDA;
             _para6:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_execute_immed(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_fetch(_para1:PISC_STATUS; _para2:Pchar; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_open(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:word; _para5:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_open2(_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:word; _para5:PXSQLDA;
             _para6:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_insert(_para1:PISC_STATUS; _para2:Pchar; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_prepare(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pchar; _para5:word;
             _para6:Pchar; _para7:word; _para8:PXSQLDA):ISC_STATUS; extdecl; external gdslib;

  function isc_embed_dsql_release(_para1:PISC_STATUS; _para2:Pchar):ISC_STATUS; extdecl; external gdslib;

  {                             }
  { Other Blob functions        }
  {                             }
  function BLOB_open(_para1:isc_blob_handle; _para2:Pchar; _para3:longint):PBSTREAM; extdecl; external gdslib;

  function BLOB_put(_para1:char; _para2:PBSTREAM):longint; extdecl; external gdslib;

  function BLOB_close(_para1:PBSTREAM):longint; extdecl; external gdslib;

  function BLOB_get(_para1:PBSTREAM):longint; extdecl; external gdslib;

  function BLOB_display(_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl; external gdslib;

  function BLOB_dump(_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl; external gdslib;

  function BLOB_edit(_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl; external gdslib;

  function BLOB_load(_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl; external gdslib;

  function BLOB_text_dump(_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl; external gdslib;

  function BLOB_text_load(_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl; external gdslib;

  function Bopen(_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):PBSTREAM; extdecl; external gdslib;

{$IFDEF Unix}
  function Bopen2(_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar; _para5:word):PBSTREAM; extdecl; external gdslib;
{$ENDIF}

  {                             }
  { Other Misc functions        }
  {                             }
  function isc_ftof(_para1:Pchar; _para2:word; _para3:Pchar; _para4:word):ISC_LONG; extdecl; external gdslib;

  function isc_print_blr(_para1:Pchar; _para2:isc_callback; _para3:pointer; _para4:smallint):ISC_STATUS; extdecl; external gdslib;

  procedure isc_set_debug(_para1:longint); extdecl; external gdslib;

  procedure isc_qtoq(_para1:PISC_QUAD; _para2:PISC_QUAD); extdecl; external gdslib;

  procedure isc_vtof(_para1:Pchar; _para2:Pchar; _para3:word); extdecl; external gdslib;

  procedure isc_vtov(_para1:Pchar; _para2:Pchar; _para3:smallint); extdecl; external gdslib;

  function isc_version(_para1:Pisc_db_handle; _para2:isc_callback; _para3:pointer):longint; extdecl; external gdslib;

{$IFDEF Unix}
  function isc_reset_fpe(_para1:word):ISC_LONG; extdecl; external gdslib;
{$ENDIF}

  {                                        }
  { Service manager functions              }
  {                                        }
  (*!!MVC
  #define ADD_SPB_LENGTH(p, length)     { (p)++ = (length); \
                                          (p)++ = (length) >> 8;}

  #define ADD_SPB_NUMERIC(p, data)      { (p)++ = (data); \
                                          (p)++ = (data) >> 8; \
                                          (p)++ = (data) >> 16; \
                                          (p)++ = (data) >> 24;}
  !!MVC *)

  function isc_service_attach(_para1:PISC_STATUS; _para2:word; _para3:Pchar; _para4:Pisc_svc_handle; _para5:word;
             _para6:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_service_detach(_para1:PISC_STATUS; _para2:Pisc_svc_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_service_query(_para1:PISC_STATUS; _para2:Pisc_svc_handle; _para3:Pisc_resv_handle; _para4:word; _para5:Pchar;
             _para6:word; _para7:Pchar; _para8:word; _para9:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_service_start(_para1:PISC_STATUS; _para2:Pisc_svc_handle; _para3:Pisc_resv_handle; _para4:word; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  {                              }
<<<<<<< HEAD
<<<<<<< HEAD
  { Shutdown and cancel          }
  {                              }
  // Available in Firebird 2.5+
  function fb_shutdown(_para1: UINT;  _para2: integer): int; extdecl; external gdslib;

  // Available in Firebird 2.5+
  //function fb_shutdown_callback{!!!3 unknown typedef}: ISC_EXPORT{ISC_STATUS}{<= !!!4 unknown type};

  // Available in Firebird 2.5+
  //function fb_cancel_operation{!!!3 unknown typedef}: ISC_EXPORT{ISC_STATUS}{<= !!!4 unknown type};


  {                              }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { Forms functions              }
  {                              }
{$IFDEF Unix}
  function isc_compile_map(_para1:PISC_STATUS; _para2:Pisc_form_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_compile_menu(_para1:PISC_STATUS; _para2:Pisc_form_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_compile_sub_map(_para1:PISC_STATUS; _para2:Pisc_win_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_create_window(_para1:PISC_STATUS; _para2:Pisc_win_handle; _para3:Psmallint; _para4:Pchar; _para5:Psmallint;
             _para6:Psmallint):ISC_STATUS; extdecl; external gdslib;

  function isc_delete_window(_para1:PISC_STATUS; _para2:Pisc_win_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_drive_form(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_win_handle; _para5:Pisc_req_handle;
             _para6:Pbyte; _para7:Pbyte):ISC_STATUS; extdecl; external gdslib;

  function isc_drive_menu(_para1:PISC_STATUS; _para2:Pisc_win_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar;
             _para6:Psmallint; _para7:Pchar; _para8:Psmallint; _para9:Psmallint; _para10:Pchar;
             _para11:PISC_LONG):ISC_STATUS; extdecl; external gdslib;

  function isc_form_delete(_para1:PISC_STATUS; _para2:Pisc_form_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_form_fetch(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_req_handle; _para5:Pbyte):ISC_STATUS; extdecl; external gdslib;

  function isc_form_insert(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_req_handle; _para5:Pbyte):ISC_STATUS; extdecl; external gdslib;

  function isc_get_entree(_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:Psmallint; _para4:Pchar; _para5:PISC_LONG;
             _para6:Psmallint):ISC_STATUS; extdecl; external gdslib;

  function isc_initialize_menu(_para1:PISC_STATUS; _para2:Pisc_req_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_menu(_para1:PISC_STATUS; _para2:Pisc_win_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_load_form(_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_form_handle; _para5:Psmallint;
             _para6:Pchar):ISC_STATUS; extdecl; external gdslib;

  function isc_pop_window(_para1:PISC_STATUS; _para2:Pisc_win_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_put_entree(_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:Psmallint; _para4:Pchar; _para5:PISC_LONG):ISC_STATUS; extdecl; external gdslib;

  function isc_reset_form(_para1:PISC_STATUS; _para2:Pisc_req_handle):ISC_STATUS; extdecl; external gdslib;

  function isc_suspend_window(_para1:PISC_STATUS; _para2:Pisc_win_handle):ISC_STATUS; extdecl; external gdslib;
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
  // Available in Firebird 2.5+
  function fb_shutdown:int; extdecl; external gdslib;

  // Available in Firebird 2.5+  
  function fb_sqlstate:ISC_STATUS; extdecl;external gdslib; 

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  function isc_attach_database:ISC_STATUS; extdecl; external gdslib;

  function isc_array_gen_sdl:ISC_STATUS; extdecl; external gdslib;

  function isc_array_get_slice:ISC_STATUS; extdecl; external gdslib;

  function isc_array_lookup_bounds:ISC_STATUS; extdecl; external gdslib;

  function isc_array_lookup_desc:ISC_STATUS; extdecl; external gdslib;

  function isc_array_set_desc:ISC_STATUS; extdecl; external gdslib;

  function isc_array_put_slice:ISC_STATUS; extdecl; external gdslib;

  function isc_blob_gen_bpb:ISC_STATUS; extdecl; external gdslib;

  function isc_blob_info:ISC_STATUS; extdecl; external gdslib;

  function isc_blob_lookup_desc:ISC_STATUS; extdecl; external gdslib;

  function isc_blob_set_desc:ISC_STATUS; extdecl; external gdslib;

  function isc_cancel_blob:ISC_STATUS; extdecl; external gdslib;

  function isc_cancel_events:ISC_STATUS; extdecl; external gdslib;

  function isc_close_blob:ISC_STATUS; extdecl; external gdslib;

  function isc_commit_retaining:ISC_STATUS; extdecl; external gdslib;

  function isc_commit_transaction:ISC_STATUS; extdecl; external gdslib;

  function isc_compile_request:ISC_STATUS; extdecl; external gdslib;

  function isc_compile_request2:ISC_STATUS; extdecl; external gdslib;

  function isc_create_blob:ISC_STATUS; extdecl; external gdslib;

  function isc_create_blob2:ISC_STATUS; extdecl; external gdslib;

  function isc_create_database:ISC_STATUS; extdecl; external gdslib;

  function isc_database_info:ISC_STATUS; extdecl; external gdslib;

  function isc_ddl:ISC_STATUS; extdecl; external gdslib;

  procedure isc_decode_date; extdecl; external gdslib;

  procedure isc_decode_sql_date; extdecl; external gdslib;

  procedure isc_decode_sql_time; extdecl; external gdslib;

  procedure isc_decode_timestamp; extdecl; external gdslib;

  function isc_detach_database:ISC_STATUS; extdecl; external gdslib;

  function isc_drop_database:ISC_STATUS; extdecl; external gdslib;

  procedure isc_encode_date; extdecl; external gdslib;

  procedure isc_encode_sql_date; extdecl; external gdslib;

  procedure isc_encode_sql_time; extdecl; external gdslib;

  procedure isc_encode_timestamp; extdecl; external gdslib;

  function isc_event_block:ISC_LONG; cdecl; external gdslib;

  procedure isc_event_counts; extdecl; external gdslib;

  procedure isc_expand_dpb; cdecl; external gdslib;

  function isc_modify_dpb:longint; extdecl; external gdslib;

  function isc_free:ISC_LONG; extdecl; external gdslib;

  function isc_get_segment:ISC_STATUS; extdecl; external gdslib;

  function isc_get_slice:ISC_STATUS; extdecl; external gdslib;

  function isc_interprete:ISC_STATUS; extdecl; external gdslib;

  function isc_open_blob:ISC_STATUS; extdecl; external gdslib;

  function isc_open_blob2:ISC_STATUS; extdecl; external gdslib;

  function isc_prepare_transaction:ISC_STATUS; extdecl; external gdslib;

  function isc_prepare_transaction2:ISC_STATUS; extdecl; external gdslib;

  procedure isc_print_sqlerror; extdecl; external gdslib;

  function isc_print_status:ISC_STATUS; extdecl; external gdslib;

  function isc_put_segment:ISC_STATUS; extdecl; external gdslib;

  function isc_put_slice:ISC_STATUS; extdecl; external gdslib;

  function isc_que_events:ISC_STATUS; extdecl; external gdslib;

  function isc_receive:ISC_STATUS; extdecl; external gdslib;

  function isc_reconnect_transaction:ISC_STATUS; extdecl; external gdslib;

  function isc_release_request:ISC_STATUS; extdecl; external gdslib;

  function isc_request_info:ISC_STATUS; extdecl; external gdslib;

{$IFDEF Unix}
  function isc_reset_fpe:ISC_LONG; extdecl; external gdslib;
{$ENDIF}

  function isc_rollback_transaction:ISC_STATUS; extdecl; external gdslib;

  function isc_rollback_retaining:ISC_STATUS; extdecl; external gdslib;

  function isc_seek_blob:ISC_STATUS; extdecl; external gdslib;

  function isc_send:ISC_STATUS; extdecl; external gdslib;

  function isc_service_attach:ISC_STATUS; extdecl; external gdslib;

  function isc_service_detach:ISC_STATUS; extdecl; external gdslib;

  function isc_service_query:ISC_STATUS; extdecl; external gdslib;

  function isc_service_start:ISC_STATUS; extdecl; external gdslib;

  function isc_start_and_send:ISC_STATUS; extdecl; external gdslib;

  function isc_start_multiple:ISC_STATUS; extdecl; external gdslib;

  function isc_start_request:ISC_STATUS; extdecl; external gdslib;

  function isc_start_transaction:ISC_STATUS; cdecl; external gdslib;

  function isc_sqlcode:ISC_LONG; extdecl; external gdslib;

  function isc_transaction_info:ISC_STATUS; extdecl; external gdslib;

  function isc_transact_request:ISC_STATUS; extdecl; external gdslib;

  function isc_unwind_request:ISC_STATUS; extdecl; external gdslib;

  function isc_wait_for_event:ISC_STATUS; extdecl; external gdslib;

  function isc_ftof:ISC_LONG; extdecl; external gdslib;

  function isc_print_blr:ISC_STATUS; extdecl; external gdslib;

  procedure isc_set_debug; extdecl; external gdslib;

  procedure isc_qtoq; extdecl; external gdslib;

  function isc_vax_integer:ISC_LONG; extdecl; external gdslib;

  procedure isc_vtof; extdecl; external gdslib;

  procedure isc_vtov; extdecl; external gdslib;

  function isc_version:longint; extdecl; external gdslib;

  {                 }
  { Blob functions  }
  {                 }

  function Bopen:PBSTREAM; extdecl; external gdslib;

  function BLOB_open:PBSTREAM; extdecl; external gdslib;

{$IFDEF Unix}
  function Bopen2:PBSTREAM; extdecl; external gdslib;
{$ENDIF}

{$ELSE} // LinkDynamically

var

{                          }
{ OSRI database functions  }
{                          }
  isc_attach_database : function (_para1:PISC_STATUS; _para2:smallint; _para3:Pchar; _para4:Pisc_db_handle; _para5:smallint;_para6:Pchar):ISC_STATUS; extdecl;
  isc_array_gen_sdl : function (_para1:PISC_STATUS; _para2:PISC_ARRAY_DESC; _para3:Psmallint; _para4:Pchar; _para5:Psmallint):ISC_STATUS; extdecl;
  isc_array_get_slice : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:PISC_QUAD; _para5:PISC_ARRAY_DESC;_para6:pointer; _para7:PISC_LONG):ISC_STATUS; extdecl;
  isc_array_lookup_bounds : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pchar; _para5:Pchar;_para6:PISC_ARRAY_DESC):ISC_STATUS; extdecl;
  isc_array_lookup_desc : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pchar; _para5:Pchar;_para6:PISC_ARRAY_DESC):ISC_STATUS; extdecl;
  isc_array_set_desc : function (_para1:PISC_STATUS; _para2:Pchar; _para3:Pchar; _para4:Psmallint; _para5:Psmallint;_para6:Psmallint; _para7:PISC_ARRAY_DESC):ISC_STATUS; extdecl;
  isc_array_put_slice : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:PISC_QUAD; _para5:PISC_ARRAY_DESC;_para6:pointer; _para7:PISC_LONG):ISC_STATUS; extdecl;
  isc_blob_default_desc : procedure (_para1:PISC_BLOB_DESC; _para2:Pbyte; _para3:Pbyte); extdecl;
  isc_blob_gen_bpb : function (_para1:PISC_STATUS; _para2:PISC_BLOB_DESC; _para3:PISC_BLOB_DESC; _para4:word; _para5:Pbyte;_para6:Pword):ISC_STATUS; extdecl;
  isc_blob_info : function (_para1:PISC_STATUS; _para2:Pisc_blob_handle; _para3:smallint; _para4:Pchar; _para5:smallint;_para6:Pchar):ISC_STATUS; extdecl;
  isc_blob_lookup_desc : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pbyte; _para5:Pbyte;_para6:PISC_BLOB_DESC; _para7:Pbyte):ISC_STATUS; extdecl;
  isc_blob_set_desc : function (_para1:PISC_STATUS; _para2:Pbyte; _para3:Pbyte; _para4:smallint; _para5:smallint;_para6:smallint; _para7:PISC_BLOB_DESC):ISC_STATUS; extdecl;
  isc_cancel_blob : function (_para1:PISC_STATUS; _para2:Pisc_blob_handle):ISC_STATUS; extdecl;
  isc_cancel_events : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:PISC_LONG):ISC_STATUS; extdecl;
  isc_close_blob : function (_para1:PISC_STATUS; _para2:Pisc_blob_handle):ISC_STATUS; extdecl;
  isc_commit_retaining : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl;
  isc_commit_transaction : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl;
  isc_create_blob : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_blob_handle; _para5:PISC_QUAD):ISC_STATUS; extdecl;
  isc_create_blob2 : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_blob_handle; _para5:PISC_QUAD;_para6:smallint; _para7:Pchar):ISC_STATUS; extdecl;
  isc_create_database : function (_para1:PISC_STATUS; _para2:smallint; _para3:Pchar; _para4:Pisc_db_handle; _para5:smallint;_para6:Pchar; _para7:smallint):ISC_STATUS; extdecl;
  isc_database_info : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:smallint; _para4:Pchar; _para5:smallint;_para6:Pchar):ISC_STATUS; extdecl;
  isc_decode_date : procedure (_para1:PISC_QUAD; _para2:pointer); extdecl;
  isc_decode_sql_date : procedure (_para1:PISC_DATE; _para2:pointer); extdecl;
  isc_decode_sql_time : procedure (_para1:PISC_TIME; _para2:pointer); extdecl;
  isc_decode_timestamp : procedure (_para1:PISC_TIMESTAMP; _para2:pointer); extdecl;
  isc_detach_database : function (_para1:PISC_STATUS; _para2:Pisc_db_handle):ISC_STATUS; extdecl;
  isc_drop_database : function (_para1:PISC_STATUS; _para2:Pisc_db_handle):ISC_STATUS; extdecl;
  isc_dsql_allocate_statement : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_stmt_handle):ISC_STATUS; extdecl;
  isc_dsql_alloc_statement2 : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_stmt_handle):ISC_STATUS; extdecl;
  isc_dsql_describe : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_describe_bind : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_exec_immed2 : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;_para6:word; _para7:PXSQLDA; _para8:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_execute : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_execute2 : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:PXSQLDA;_para6:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_execute_immediate : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;_para6:word; _para7:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_fetch : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_finish : function (_para1:Pisc_db_handle):ISC_STATUS; extdecl;
  isc_dsql_free_statement : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word):ISC_STATUS; extdecl;
  isc_dsql_insert : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_prepare : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:Pchar;_para6:word; _para7:PXSQLDA):ISC_STATUS; extdecl;
  isc_dsql_set_cursor_name : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:Pchar; _para4:word):ISC_STATUS; extdecl;
  isc_dsql_sql_info : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:smallint; _para4:Pchar; _para5:smallint;_para6:Pchar):ISC_STATUS; extdecl;
  isc_encode_date : procedure (_para1:pointer; _para2:PISC_QUAD); extdecl;
  isc_encode_sql_date : procedure (_para1:pointer; _para2:PISC_DATE); extdecl;
  isc_encode_sql_time : procedure (_para1:pointer; _para2:PISC_TIME); extdecl;
  isc_encode_timestamp : procedure (_para1:pointer; _para2:PISC_TIMESTAMP); extdecl;
  isc_event_block : function (_para1:PPchar; _para2:PPchar; _para3:word; args:array of const):ISC_LONG; cdecl;
  isc_event_counts: procedure (_para1: PISC_STATUS; _para2: short; _para3: pchar; _para4: pchar); extdecl;
  isc_expand_dpb : procedure (_para1:PPchar; _para2:Psmallint; args:array of const); cdecl;
  isc_modify_dpb : function (_para1:PPchar; _para2:Psmallint; _para3:word; _para4:Pchar; _para5:smallint):longint; extdecl;
  isc_free : function (_para1:Pchar):ISC_LONG; extdecl;
  isc_get_segment : function (_para1:PISC_STATUS; _para2:Pisc_blob_handle; _para3:Pword; _para4:word; _para5:Pchar):ISC_STATUS; extdecl;
  isc_get_slice : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:PISC_QUAD; _para5:smallint;_para6:Pchar; _para7:smallint; _para8:PISC_LONG; _para9:ISC_LONG; _para10:pointer;_para11:PISC_LONG):ISC_STATUS; extdecl;
  isc_interprete : function (_para1:Pchar; _para2:PPISC_STATUS):ISC_STATUS; extdecl;
  isc_open_blob : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_blob_handle; _para5:PISC_QUAD):ISC_STATUS; extdecl;
  isc_open_blob2 : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_blob_handle; _para5:PISC_QUAD;_para6:smallint; _para7:Pchar):ISC_STATUS; extdecl;
  isc_prepare_transaction2 : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint; _para4:Pchar):ISC_STATUS; extdecl;
  isc_print_sqlerror : procedure (_para1:smallint; _para2:PISC_STATUS); extdecl;
  isc_print_status : function (_para1:PISC_STATUS):ISC_STATUS; extdecl;
  isc_put_segment : function (_para1:PISC_STATUS; _para2:Pisc_blob_handle; _para3:word; _para4:Pchar):ISC_STATUS; extdecl;
  isc_put_slice : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:PISC_QUAD; _para5:smallint;_para6:Pchar; _para7:smallint; _para8:PISC_LONG; _para9:ISC_LONG; _para10:pointer):ISC_STATUS; extdecl;
  isc_que_events : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:PISC_LONG; _para4:smallint; _para5:Pchar;_para6:isc_callback; _para7:pointer):ISC_STATUS; extdecl;
  isc_rollback_retaining : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl;
  isc_rollback_transaction : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl;
  isc_start_multiple : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint; _para4:pointer):ISC_STATUS; extdecl;
  isc_start_transaction : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint; args:array of const):ISC_STATUS; cdecl;
  isc_sqlcode : function (_para1:PISC_STATUS):ISC_LONG; extdecl;
<<<<<<< HEAD
<<<<<<< HEAD
  // Available in Firebird 2.5+
  fb_sqlstate : function(_para1: PAnsiChar;_para2:PISC_STATUS):ISC_STATUS; extdecl; 
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  isc_sql_interprete : procedure (_para1:smallint; _para2:Pchar; _para3:smallint); extdecl;
  isc_transaction_info : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint; _para4:Pchar; _para5:smallint;_para6:Pchar):ISC_STATUS; extdecl;
  isc_transact_request : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;_para6:word; _para7:Pchar; _para8:word; _para9:Pchar):ISC_STATUS; extdecl;
  isc_vax_integer : function (_para1:Pchar; _para2:smallint):ISC_LONG; extdecl;
  isc_portable_integer : function (_para1:Pbyte; _para2:smallint):ISC_INT64; extdecl;
{                                    }
{ Security Functions                 }
{                                    }
  isc_add_user : function (_para1:PISC_STATUS; _para2:PUSER_SEC_DATA):longint; extdecl;
  isc_delete_user : function (_para1:PISC_STATUS; _para2:PUSER_SEC_DATA):longint; extdecl;
  isc_modify_user : function (_para1:PISC_STATUS; _para2:PUSER_SEC_DATA):longint; extdecl;
{                                 }
{  Other OSRI functions           }
{                                 }
  isc_compile_request : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_req_handle; _para4:smallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_compile_request2 : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_req_handle; _para4:smallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_ddl : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:smallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_prepare_transaction : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle):ISC_STATUS; extdecl;
  isc_receive : function (_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:smallint; _para4:smallint; _para5:pointer;_para6:smallint):ISC_STATUS; extdecl;
  isc_reconnect_transaction : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:smallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_release_request : function (_para1:PISC_STATUS; _para2:Pisc_req_handle):ISC_STATUS; extdecl;
  isc_request_info : function (_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:smallint; _para4:smallint; _para5:Pchar;_para6:smallint; _para7:Pchar):ISC_STATUS; extdecl;
  isc_seek_blob : function (_para1:PISC_STATUS; _para2:Pisc_blob_handle; _para3:smallint; _para4:ISC_LONG; _para5:PISC_LONG):ISC_STATUS; extdecl;
  isc_send : function (_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:smallint; _para4:smallint; _para5:pointer;_para6:smallint):ISC_STATUS; extdecl;
  isc_start_and_send : function (_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:Pisc_tr_handle; _para4:smallint; _para5:smallint;_para6:pointer; _para7:smallint):ISC_STATUS; extdecl;
  isc_start_request : function (_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:Pisc_tr_handle; _para4:smallint):ISC_STATUS; extdecl;
  isc_unwind_request : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:smallint):ISC_STATUS; extdecl;
  isc_wait_for_event : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:smallint; _para4:Pchar; _para5:Pchar):ISC_STATUS; extdecl;
{                            }
{ Other Sql functions        }
{                            }
  isc_close : function (_para1:PISC_STATUS; _para2:Pchar):ISC_STATUS; extdecl;
  isc_declare : function (_para1:PISC_STATUS; _para2:Pchar; _para3:Pchar):ISC_STATUS; extdecl;
  isc_describe : function (_para1:PISC_STATUS; _para2:Pchar; _para3:PXSQLDA):ISC_STATUS; extdecl;
  isc_describe_bind : function (_para1:PISC_STATUS; _para2:Pchar; _para3:PXSQLDA):ISC_STATUS; extdecl;
  isc_execute : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_execute_immediate : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_fetch : function (_para1:PISC_STATUS; _para2:Pchar; _para3:PXSQLDA):ISC_STATUS; extdecl;
  isc_open : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_prepare : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pchar; _para5:Psmallint;_para6:Pchar; _para7:PXSQLDA):ISC_STATUS; extdecl;
{                                    }
{ Other Dynamic sql functions        }
{                                    }
  isc_dsql_execute_m : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:Pchar;_para6:word; _para7:word; _para8:Pchar):ISC_STATUS; extdecl;
  isc_dsql_execute2_m : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:Pchar;_para6:word; _para7:word; _para8:Pchar; _para9:word; _para10:Pchar;_para11:word; _para12:word; _para13:Pchar):ISC_STATUS; extdecl;
  isc_dsql_execute_immediate_m : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;_para6:word; _para7:word; _para8:Pchar; _para9:word; _para10:word;_para11:Pchar):ISC_STATUS; extdecl;
  isc_dsql_exec_immed3_m : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;_para6:word; _para7:word; _para8:Pchar; _para9:word; _para10:word;_para11:Pchar; _para12:word; _para13:Pchar; _para14:word; _para15:word;_para16:Pchar):ISC_STATUS; extdecl;
  isc_dsql_fetch_m : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:Pchar; _para5:word;_para6:word; _para7:Pchar):ISC_STATUS; extdecl;
  isc_dsql_insert_m : function (_para1:PISC_STATUS; _para2:Pisc_stmt_handle; _para3:word; _para4:Pchar; _para5:word;_para6:word; _para7:Pchar):ISC_STATUS; extdecl;
  isc_dsql_prepare_m : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pisc_stmt_handle; _para4:word; _para5:Pchar;_para6:word; _para7:word; _para8:Pchar; _para9:word; _para10:Pchar):ISC_STATUS; extdecl;
  isc_dsql_release : function (_para1:PISC_STATUS; _para2:Pchar):ISC_STATUS; extdecl;
  isc_embed_dsql_close : function (_para1:PISC_STATUS; _para2:Pchar):ISC_STATUS; extdecl;
  isc_embed_dsql_declare : function (_para1:PISC_STATUS; _para2:Pchar; _para3:Pchar):ISC_STATUS; extdecl;
  isc_embed_dsql_describe : function (_para1:PISC_STATUS; _para2:Pchar; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_describe_bind : function (_para1:PISC_STATUS; _para2:Pchar; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_execute : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:word; _para5:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_execute2 : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:word; _para5:PXSQLDA;_para6:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_execute_immed : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:word; _para5:Pchar;_para6:word; _para7:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_fetch : function (_para1:PISC_STATUS; _para2:Pchar; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_open : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:word; _para5:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_open2 : function (_para1:PISC_STATUS; _para2:Pisc_tr_handle; _para3:Pchar; _para4:word; _para5:PXSQLDA;_para6:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_insert : function (_para1:PISC_STATUS; _para2:Pchar; _para3:word; _para4:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_prepare : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pchar; _para5:word;_para6:Pchar; _para7:word; _para8:PXSQLDA):ISC_STATUS; extdecl;
  isc_embed_dsql_release : function (_para1:PISC_STATUS; _para2:Pchar):ISC_STATUS; extdecl;
{                             }
{ Other Blob functions        }
{                             }
  BLOB_open : function (_para1:isc_blob_handle; _para2:Pchar; _para3:longint):PBSTREAM; extdecl;
  BLOB_put : function (_para1:char; _para2:PBSTREAM):longint; extdecl;
  BLOB_close : function (_para1:PBSTREAM):longint; extdecl;
  BLOB_get : function (_para1:PBSTREAM):longint; extdecl;
  BLOB_display : function (_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl;
  BLOB_dump : function (_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl;
  BLOB_edit : function (_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl;
  BLOB_load : function (_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl;
  BLOB_text_dump : function (_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl;
  BLOB_text_load : function (_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):longint; extdecl;
  Bopen : function (_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar):PBSTREAM; extdecl;
{$IFDEF Unix}
  Bopen2 : function (_para1:PISC_QUAD; _para2:isc_db_handle; _para3:isc_tr_handle; _para4:Pchar; _para5:word):PBSTREAM; extdecl;
{$ENDIF}
{                             }
{ Other Misc functions        }
{                             }
  isc_ftof : function (_para1:Pchar; _para2:word; _para3:Pchar; _para4:word):ISC_LONG; extdecl;
  isc_print_blr : function (_para1:Pchar; _para2:isc_callback; _para3:pointer; _para4:smallint):ISC_STATUS; extdecl;
  isc_set_debug : procedure (_para1:longint); extdecl;
  isc_qtoq : procedure (_para1:PISC_QUAD; _para2:PISC_QUAD); extdecl;
  isc_vtof : procedure (_para1:Pchar; _para2:Pchar; _para3:word); extdecl;
  isc_vtov : procedure (_para1:Pchar; _para2:Pchar; _para3:smallint); extdecl;
  isc_version : function (_para1:Pisc_db_handle; _para2:isc_callback; _para3:pointer):longint; extdecl;
{$IFDEF Unix}
  isc_reset_fpe : function (_para1:word):ISC_LONG; extdecl;
{$ENDIF}
{                                        }
{ Service manager functions              }
{                                        }
(*!!MVC
  #define ADD_SPB_LENGTH(p, length)     { (p)++ = (length); \
  (p)++ = (length) >> 8;}

#define ADD_SPB_NUMERIC(p, data)      { (p)++ = (data); \
  (p)++ = (data) >> 8; \
  (p)++ = (data) >> 16; \
  (p)++ = (data) >> 24;}
!!MVC *)
  isc_service_attach : function (_para1:PISC_STATUS; _para2:word; _para3:Pchar; _para4:Pisc_svc_handle; _para5:word;_para6:Pchar):ISC_STATUS; extdecl;
  isc_service_detach : function (_para1:PISC_STATUS; _para2:Pisc_svc_handle):ISC_STATUS; extdecl;
  isc_service_query : function (_para1:PISC_STATUS; _para2:Pisc_svc_handle; _para3:Pisc_resv_handle; _para4:word; _para5:Pchar;_para6:word; _para7:Pchar; _para8:word; _para9:Pchar):ISC_STATUS; extdecl;
  isc_service_start : function (_para1:PISC_STATUS; _para2:Pisc_svc_handle; _para3:Pisc_resv_handle; _para4:word; _para5:Pchar):ISC_STATUS; extdecl;
<<<<<<< HEAD
<<<<<<< HEAD


{                              }
{ Shutdown and cancel          }
{                              }
  // Available in Firebird 2.5+
  fb_shutdown : function(_para1: UINT; _para2: integer):int; extdecl;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{                              }
{ Forms functions              }
{                              }
{$IFDEF Unix}
  isc_compile_map : function (_para1:PISC_STATUS; _para2:Pisc_form_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_compile_menu : function (_para1:PISC_STATUS; _para2:Pisc_form_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_compile_sub_map : function (_para1:PISC_STATUS; _para2:Pisc_win_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_create_window : function (_para1:PISC_STATUS; _para2:Pisc_win_handle; _para3:Psmallint; _para4:Pchar; _para5:Psmallint;_para6:Psmallint):ISC_STATUS; extdecl;
  isc_delete_window : function (_para1:PISC_STATUS; _para2:Pisc_win_handle):ISC_STATUS; extdecl;
  isc_drive_form : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_win_handle; _para5:Pisc_req_handle;_para6:Pbyte; _para7:Pbyte):ISC_STATUS; extdecl;
  isc_drive_menu : function (_para1:PISC_STATUS; _para2:Pisc_win_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar;_para6:Psmallint; _para7:Pchar; _para8:Psmallint; _para9:Psmallint; _para10:Pchar;_para11:PISC_LONG):ISC_STATUS; extdecl;
  isc_form_delete : function (_para1:PISC_STATUS; _para2:Pisc_form_handle):ISC_STATUS; extdecl;
  isc_form_fetch : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_req_handle; _para5:Pbyte):ISC_STATUS; extdecl;
  isc_form_insert : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_req_handle; _para5:Pbyte):ISC_STATUS; extdecl;
  isc_get_entree : function (_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:Psmallint; _para4:Pchar; _para5:PISC_LONG;_para6:Psmallint):ISC_STATUS; extdecl;
  isc_initialize_menu : function (_para1:PISC_STATUS; _para2:Pisc_req_handle):ISC_STATUS; extdecl;
  isc_menu : function (_para1:PISC_STATUS; _para2:Pisc_win_handle; _para3:Pisc_req_handle; _para4:Psmallint; _para5:Pchar):ISC_STATUS; extdecl;
  isc_load_form : function (_para1:PISC_STATUS; _para2:Pisc_db_handle; _para3:Pisc_tr_handle; _para4:Pisc_form_handle; _para5:Psmallint;_para6:Pchar):ISC_STATUS; extdecl;
  isc_pop_window : function (_para1:PISC_STATUS; _para2:Pisc_win_handle):ISC_STATUS; extdecl;
  isc_put_entree : function (_para1:PISC_STATUS; _para2:Pisc_req_handle; _para3:Psmallint; _para4:Pchar; _para5:PISC_LONG):ISC_STATUS; extdecl;
  isc_reset_form : function (_para1:PISC_STATUS; _para2:Pisc_req_handle):ISC_STATUS; extdecl;
  isc_suspend_window : function (_para1:PISC_STATUS; _para2:Pisc_win_handle):ISC_STATUS; extdecl;
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
function InitialiseIBase60(Const LibraryName : AnsiString) : integer;
function InitialiseIBase60 : integer;
procedure ReleaseIBase60;

Function IBaseLibraryHandle : TLibHandle;
Function IBaseLoadedLibrary : AnsiString;
=======
=======
>>>>>>> origin/fixes_2_2
function InitialiseIBase60(Const LibraryName : String) : integer;
function InitialiseIBase60 : integer;
procedure ReleaseIBase60;

var IBaseLibraryHandle : TLibHandle;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF}

implementation

{$IFDEF LinkDynamically}

ResourceString
  SErrEmbeddedFailed = 'Can not load embedded Firebird client "%s". Check your installation.';
<<<<<<< HEAD
<<<<<<< HEAD
  SErrDefaultsFailed = 'Can not load default Firebird clients ("%s" or "%s" or "%s"). Check your installation.';
=======
  SErrDefaultsFailed = 'Can not load default Firebird clients ("%s" or "%s"). Check your installation.';
>>>>>>> graemeg/fixes_2_2
=======
  SErrDefaultsFailed = 'Can not load default Firebird clients ("%s" or "%s"). Check your installation.';
>>>>>>> origin/fixes_2_2
  SErrLoadFailed     = 'Can not load Firebird client library "%s". Check your installation.';
  SErrAlreadyLoaded  = 'Firebird interface already initialized from library %s.';
  
var 
  RefCount : integer;
<<<<<<< HEAD
<<<<<<< HEAD
  LoadedLibrary : AnsiString;
  LibHandle : TLibHandle;
  
Function IBaseLibraryHandle : TLibHandle;

begin
  result:=LibHandle;
end;

Function IBaseLoadedLibrary : AnsiString;
  
begin
  Result:=LoadedLibrary;
end;  

Function TryInitialiseIBase60(Const LibraryName : AnsiString) : integer;
=======
  LoadedLibrary : String;

Function TryInitialiseIBase60(Const LibraryName : String) : integer;
>>>>>>> graemeg/fixes_2_2
=======
  LoadedLibrary : String;

Function TryInitialiseIBase60(Const LibraryName : String) : integer;
>>>>>>> origin/fixes_2_2

begin
  Result := 0;
  if (RefCount=0) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    LibHandle:=LoadLibrary(LibraryName);
    if (LibHandle=nilhandle) then
      Exit;
    inc(RefCount);
    LoadedLibrary:=LibraryName;
    // Available in Firebird 2.5+
    pointer(fb_shutdown) := GetProcedureAddress(IBaseLibraryHandle,'fb_shutdown');
    // Available in Firebird 2.5+    
    pointer(fb_sqlstate) := GetProcedureAddress(IBaseLibraryHandle,'fb_sqlstate');
=======
=======
>>>>>>> origin/fixes_2_2
    IBaseLibraryHandle:=LoadLibrary(LibraryName);
    if (IBaseLibraryHandle=nilhandle) then
      Exit;
    inc(RefCount);
    LoadedLibrary:=LibraryName;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    pointer(isc_attach_database) := GetProcedureAddress(IBaseLibraryHandle,'isc_attach_database');
    pointer(isc_array_gen_sdl) := GetProcedureAddress(IBaseLibraryHandle,'isc_array_gen_sdl');
    pointer(isc_array_get_slice) := GetProcedureAddress(IBaseLibraryHandle,'isc_array_get_slice');
    pointer(isc_array_lookup_bounds) := GetProcedureAddress(IBaseLibraryHandle,'isc_array_lookup_bounds');
    pointer(isc_array_lookup_desc) := GetProcedureAddress(IBaseLibraryHandle,'isc_array_lookup_desc');
    pointer(isc_array_set_desc) := GetProcedureAddress(IBaseLibraryHandle,'isc_array_set_desc');
    pointer(isc_array_put_slice) := GetProcedureAddress(IBaseLibraryHandle,'isc_array_put_slice');
    pointer(isc_blob_default_desc) := GetProcedureAddress(IBaseLibraryHandle,'isc_blob_default_desc');
    pointer(isc_blob_gen_bpb) := GetProcedureAddress(IBaseLibraryHandle,'isc_blob_gen_bpb');
    pointer(isc_blob_info) := GetProcedureAddress(IBaseLibraryHandle,'isc_blob_info');
    pointer(isc_blob_lookup_desc) := GetProcedureAddress(IBaseLibraryHandle,'isc_blob_lookup_desc');
    pointer(isc_blob_set_desc) := GetProcedureAddress(IBaseLibraryHandle,'isc_blob_set_desc');
    pointer(isc_cancel_blob) := GetProcedureAddress(IBaseLibraryHandle,'isc_cancel_blob');
    pointer(isc_cancel_events) := GetProcedureAddress(IBaseLibraryHandle,'isc_cancel_events');
    pointer(isc_close_blob) := GetProcedureAddress(IBaseLibraryHandle,'isc_close_blob');
    pointer(isc_commit_retaining) := GetProcedureAddress(IBaseLibraryHandle,'isc_commit_retaining');
    pointer(isc_commit_transaction) := GetProcedureAddress(IBaseLibraryHandle,'isc_commit_transaction');
    pointer(isc_create_blob) := GetProcedureAddress(IBaseLibraryHandle,'isc_create_blob');
    pointer(isc_create_blob2) := GetProcedureAddress(IBaseLibraryHandle,'isc_create_blob2');
    pointer(isc_create_database) := GetProcedureAddress(IBaseLibraryHandle,'isc_create_database');
    pointer(isc_database_info) := GetProcedureAddress(IBaseLibraryHandle,'isc_database_info');
    pointer(isc_decode_date) := GetProcedureAddress(IBaseLibraryHandle,'isc_decode_date');
    pointer(isc_decode_sql_date) := GetProcedureAddress(IBaseLibraryHandle,'isc_decode_sql_date');
    pointer(isc_decode_sql_time) := GetProcedureAddress(IBaseLibraryHandle,'isc_decode_sql_time');
    pointer(isc_decode_timestamp) := GetProcedureAddress(IBaseLibraryHandle,'isc_decode_timestamp');
    pointer(isc_detach_database) := GetProcedureAddress(IBaseLibraryHandle,'isc_detach_database');
    pointer(isc_drop_database) := GetProcedureAddress(IBaseLibraryHandle,'isc_drop_database');
    pointer(isc_dsql_allocate_statement) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_allocate_statement');
    pointer(isc_dsql_alloc_statement2) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_alloc_statement2');
    pointer(isc_dsql_describe) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_describe');
    pointer(isc_dsql_describe_bind) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_describe_bind');
    pointer(isc_dsql_exec_immed2) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_exec_immed2');
    pointer(isc_dsql_execute) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_execute');
    pointer(isc_dsql_execute2) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_execute2');
    pointer(isc_dsql_execute_immediate) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_execute_immediate');
    pointer(isc_dsql_fetch) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_fetch');
    pointer(isc_dsql_finish) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_finish');
    pointer(isc_dsql_free_statement) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_free_statement');
    pointer(isc_dsql_insert) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_insert');
    pointer(isc_dsql_prepare) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_prepare');
    pointer(isc_dsql_set_cursor_name) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_set_cursor_name');
    pointer(isc_dsql_sql_info) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_sql_info');
    pointer(isc_encode_date) := GetProcedureAddress(IBaseLibraryHandle,'isc_encode_date');
    pointer(isc_encode_sql_date) := GetProcedureAddress(IBaseLibraryHandle,'isc_encode_sql_date');
    pointer(isc_encode_sql_time) := GetProcedureAddress(IBaseLibraryHandle,'isc_encode_sql_time');
    pointer(isc_encode_timestamp) := GetProcedureAddress(IBaseLibraryHandle,'isc_encode_timestamp');
    pointer(isc_event_block) := GetProcedureAddress(IBaseLibraryHandle,'isc_event_block');
    pointer(isc_event_counts) := GetProcedureAddress(IBaseLibraryHandle,'isc_event_counts');
    pointer(isc_expand_dpb) := GetProcedureAddress(IBaseLibraryHandle,'isc_expand_dpb');
    pointer(isc_modify_dpb) := GetProcedureAddress(IBaseLibraryHandle,'isc_modify_dpb');
    pointer(isc_free) := GetProcedureAddress(IBaseLibraryHandle,'isc_free');
    pointer(isc_get_segment) := GetProcedureAddress(IBaseLibraryHandle,'isc_get_segment');
    pointer(isc_get_slice) := GetProcedureAddress(IBaseLibraryHandle,'isc_get_slice');
    pointer(isc_interprete) := GetProcedureAddress(IBaseLibraryHandle,'isc_interprete');
    pointer(isc_open_blob) := GetProcedureAddress(IBaseLibraryHandle,'isc_open_blob');
    pointer(isc_open_blob2) := GetProcedureAddress(IBaseLibraryHandle,'isc_open_blob2');
    pointer(isc_prepare_transaction2) := GetProcedureAddress(IBaseLibraryHandle,'isc_prepare_transaction2');
    pointer(isc_print_sqlerror) := GetProcedureAddress(IBaseLibraryHandle,'isc_print_sqlerror');
    pointer(isc_print_status) := GetProcedureAddress(IBaseLibraryHandle,'isc_print_status');
    pointer(isc_put_segment) := GetProcedureAddress(IBaseLibraryHandle,'isc_put_segment');
    pointer(isc_put_slice) := GetProcedureAddress(IBaseLibraryHandle,'isc_put_slice');
    pointer(isc_que_events) := GetProcedureAddress(IBaseLibraryHandle,'isc_que_events');
    pointer(isc_rollback_retaining) := GetProcedureAddress(IBaseLibraryHandle,'isc_rollback_retaining');
    pointer(isc_rollback_transaction) := GetProcedureAddress(IBaseLibraryHandle,'isc_rollback_transaction');
    pointer(isc_start_multiple) := GetProcedureAddress(IBaseLibraryHandle,'isc_start_multiple');
    pointer(isc_start_transaction) := GetProcedureAddress(IBaseLibraryHandle,'isc_start_transaction');
    pointer(isc_sqlcode) := GetProcedureAddress(IBaseLibraryHandle,'isc_sqlcode');
    pointer(isc_sql_interprete) := GetProcedureAddress(IBaseLibraryHandle,'isc_sql_interprete');
    pointer(isc_transaction_info) := GetProcedureAddress(IBaseLibraryHandle,'isc_transaction_info');
    pointer(isc_transact_request) := GetProcedureAddress(IBaseLibraryHandle,'isc_transact_request');
    pointer(isc_vax_integer) := GetProcedureAddress(IBaseLibraryHandle,'isc_vax_integer');
    pointer(isc_portable_integer) := GetProcedureAddress(IBaseLibraryHandle,'isc_portable_integer');
    pointer(isc_add_user) := GetProcedureAddress(IBaseLibraryHandle,'isc_add_user');
    pointer(isc_delete_user) := GetProcedureAddress(IBaseLibraryHandle,'isc_delete_user');
    pointer(isc_modify_user) := GetProcedureAddress(IBaseLibraryHandle,'isc_modify_user');
    pointer(isc_compile_request) := GetProcedureAddress(IBaseLibraryHandle,'isc_compile_request');
    pointer(isc_compile_request2) := GetProcedureAddress(IBaseLibraryHandle,'isc_compile_request2');
    pointer(isc_ddl) := GetProcedureAddress(IBaseLibraryHandle,'isc_ddl');
    pointer(isc_prepare_transaction) := GetProcedureAddress(IBaseLibraryHandle,'isc_prepare_transaction');
    pointer(isc_receive) := GetProcedureAddress(IBaseLibraryHandle,'isc_receive');
    pointer(isc_reconnect_transaction) := GetProcedureAddress(IBaseLibraryHandle,'isc_reconnect_transaction');
    pointer(isc_release_request) := GetProcedureAddress(IBaseLibraryHandle,'isc_release_request');
    pointer(isc_request_info) := GetProcedureAddress(IBaseLibraryHandle,'isc_request_info');
    pointer(isc_seek_blob) := GetProcedureAddress(IBaseLibraryHandle,'isc_seek_blob');
    pointer(isc_send) := GetProcedureAddress(IBaseLibraryHandle,'isc_send');
    pointer(isc_start_and_send) := GetProcedureAddress(IBaseLibraryHandle,'isc_start_and_send');
    pointer(isc_start_request) := GetProcedureAddress(IBaseLibraryHandle,'isc_start_request');
    pointer(isc_unwind_request) := GetProcedureAddress(IBaseLibraryHandle,'isc_unwind_request');
    pointer(isc_wait_for_event) := GetProcedureAddress(IBaseLibraryHandle,'isc_wait_for_event');
    pointer(isc_close) := GetProcedureAddress(IBaseLibraryHandle,'isc_close');
    pointer(isc_declare) := GetProcedureAddress(IBaseLibraryHandle,'isc_declare');
    pointer(isc_describe) := GetProcedureAddress(IBaseLibraryHandle,'isc_describe');
    pointer(isc_describe_bind) := GetProcedureAddress(IBaseLibraryHandle,'isc_describe_bind');
    pointer(isc_execute) := GetProcedureAddress(IBaseLibraryHandle,'isc_execute');
    pointer(isc_execute_immediate) := GetProcedureAddress(IBaseLibraryHandle,'isc_execute_immediate');
    pointer(isc_fetch) := GetProcedureAddress(IBaseLibraryHandle,'isc_fetch');
    pointer(isc_open) := GetProcedureAddress(IBaseLibraryHandle,'isc_open');
    pointer(isc_prepare) := GetProcedureAddress(IBaseLibraryHandle,'isc_prepare');
    pointer(isc_dsql_execute_m) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_execute_m');
    pointer(isc_dsql_execute2_m) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_execute2_m');
    pointer(isc_dsql_execute_immediate_m) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_execute_immediate_m');
    pointer(isc_dsql_exec_immed3_m) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_exec_immed3_m');
    pointer(isc_dsql_fetch_m) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_fetch_m');
    pointer(isc_dsql_insert_m) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_insert_m');
    pointer(isc_dsql_prepare_m) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_prepare_m');
    pointer(isc_dsql_release) := GetProcedureAddress(IBaseLibraryHandle,'isc_dsql_release');
    pointer(isc_embed_dsql_close) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_close');
    pointer(isc_embed_dsql_declare) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_declare');
    pointer(isc_embed_dsql_describe) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_describe');
    pointer(isc_embed_dsql_describe_bind) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_describe_bind');
    pointer(isc_embed_dsql_execute) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_execute');
    pointer(isc_embed_dsql_execute2) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_execute2');
    pointer(isc_embed_dsql_execute_immed) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_execute_immed');
    pointer(isc_embed_dsql_fetch) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_fetch');
    pointer(isc_embed_dsql_open) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_open');
    pointer(isc_embed_dsql_open2) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_open2');
    pointer(isc_embed_dsql_insert) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_insert');
    pointer(isc_embed_dsql_prepare) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_prepare');
    pointer(isc_embed_dsql_release) := GetProcedureAddress(IBaseLibraryHandle,'isc_embed_dsql_release');
    pointer(BLOB_open) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_open');
    pointer(BLOB_put) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_put');
    pointer(BLOB_close) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_close');
    pointer(BLOB_get) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_get');
    pointer(BLOB_display) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_display');
    pointer(BLOB_dump) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_dump');
    pointer(BLOB_edit) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_edit');
    pointer(BLOB_load) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_load');
    pointer(BLOB_text_dump) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_text_dump');
    pointer(BLOB_text_load) := GetProcedureAddress(IBaseLibraryHandle,'BLOB_text_load');
    pointer(Bopen) := GetProcedureAddress(IBaseLibraryHandle,'Bopen');
{$IFDEF Unix}
    pointer(Bopen2) := GetProcedureAddress(IBaseLibraryHandle,'Bopen2');
{$ENDIF}
    pointer(isc_ftof) := GetProcedureAddress(IBaseLibraryHandle,'isc_ftof');
    pointer(isc_print_blr) := GetProcedureAddress(IBaseLibraryHandle,'isc_print_blr');
    pointer(isc_set_debug) := GetProcedureAddress(IBaseLibraryHandle,'isc_set_debug');
    pointer(isc_qtoq) := GetProcedureAddress(IBaseLibraryHandle,'isc_qtoq');
    pointer(isc_vtof) := GetProcedureAddress(IBaseLibraryHandle,'isc_vtof');
    pointer(isc_vtov) := GetProcedureAddress(IBaseLibraryHandle,'isc_vtov');
    pointer(isc_version) := GetProcedureAddress(IBaseLibraryHandle,'isc_version');
{$IFDEF Unix}
    pointer(isc_reset_fpe) := GetProcedureAddress(IBaseLibraryHandle,'isc_reset_fpe');
{$ENDIF}
    pointer(isc_service_attach) := GetProcedureAddress(IBaseLibraryHandle,'isc_service_attach');
    pointer(isc_service_detach) := GetProcedureAddress(IBaseLibraryHandle,'isc_service_detach');
    pointer(isc_service_query) := GetProcedureAddress(IBaseLibraryHandle,'isc_service_query');
    pointer(isc_service_start) := GetProcedureAddress(IBaseLibraryHandle,'isc_service_start');
{$IFDEF Unix}
    pointer(isc_compile_map) := GetProcedureAddress(IBaseLibraryHandle,'isc_compile_map');
    pointer(isc_compile_menu) := GetProcedureAddress(IBaseLibraryHandle,'isc_compile_menu');
    pointer(isc_compile_sub_map) := GetProcedureAddress(IBaseLibraryHandle,'isc_compile_sub_map');
    pointer(isc_create_window) := GetProcedureAddress(IBaseLibraryHandle,'isc_create_window');
    pointer(isc_delete_window) := GetProcedureAddress(IBaseLibraryHandle,'isc_delete_window');
    pointer(isc_drive_form) := GetProcedureAddress(IBaseLibraryHandle,'isc_drive_form');
    pointer(isc_drive_menu) := GetProcedureAddress(IBaseLibraryHandle,'isc_drive_menu');
    pointer(isc_form_delete) := GetProcedureAddress(IBaseLibraryHandle,'isc_form_delete');
    pointer(isc_form_fetch) := GetProcedureAddress(IBaseLibraryHandle,'isc_form_fetch');
    pointer(isc_form_insert) := GetProcedureAddress(IBaseLibraryHandle,'isc_form_insert');
    pointer(isc_get_entree) := GetProcedureAddress(IBaseLibraryHandle,'isc_get_entree');
    pointer(isc_initialize_menu) := GetProcedureAddress(IBaseLibraryHandle,'isc_initialize_menu');
    pointer(isc_menu) := GetProcedureAddress(IBaseLibraryHandle,'isc_menu');
    pointer(isc_load_form) := GetProcedureAddress(IBaseLibraryHandle,'isc_load_form');
    pointer(isc_pop_window) := GetProcedureAddress(IBaseLibraryHandle,'isc_pop_window');
    pointer(isc_put_entree) := GetProcedureAddress(IBaseLibraryHandle,'isc_put_entree');
    pointer(isc_reset_form) := GetProcedureAddress(IBaseLibraryHandle,'isc_reset_form');
    pointer(isc_suspend_window) := GetProcedureAddress(IBaseLibraryHandle,'isc_suspend_window');
{$ENDIF}
    end
  else
    inc(RefCount);
  Result := RefCount;
end;

function InitialiseIBase60 : integer;

begin
  Result := 0;
<<<<<<< HEAD
<<<<<<< HEAD
  if (RefCount<>0) then
    // pretend to load whatever is already loaded, so we do not get a library name conflict.
    Inc(RefCount)
  else If UseEmbeddedFirebird then
    begin
    If (TryInitialiseIBase60(fbembedlib)=0) and (TryInitialiseIBase60(libem)=0) then
=======
  If UseEmbeddedFirebird then
    begin
    If (TryInitialiseIBase60(fbembedlib)=0) then
>>>>>>> graemeg/fixes_2_2
=======
  If UseEmbeddedFirebird then
    begin
    If (TryInitialiseIBase60(fbembedlib)=0) then
>>>>>>> origin/fixes_2_2
      Raise EInOutError.CreateFmt(SErrEmbeddedFailed,[fbembedlib]);
    end
  else
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    If (TryInitialiseIBase60(fbembedlib)=0) and
       (TryInitialiseIBase60(libem)=0) and
       (TryInitialiseIBase60(fbclib)=0) and
       (TryInitialiseIBase60(libfc)=0) and
       (TryInitialiseIBase60(gdslib)=0) then
        Raise EInOutError.CreateFmt(SErrDefaultsFailed,[fbclib,gdslib,fbembedlib]);
=======
    If (TryInitialiseIBase60(fbclib)=0) and
       (TryInitialiseIBase60(gdslib)=0) then
        Raise EInOutError.CreateFmt(SErrDefaultsFailed,[gdslib,fbclib]);
>>>>>>> graemeg/fixes_2_2
=======
    If (TryInitialiseIBase60(fbclib)=0) and
       (TryInitialiseIBase60(gdslib)=0) then
        Raise EInOutError.CreateFmt(SErrDefaultsFailed,[gdslib,fbclib]);
>>>>>>> origin/fixes_2_2
    end;    
  Result := RefCount;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function InitialiseIBase60(Const LibraryName : AnsiString) : integer;
=======
function InitialiseIBase60(Const LibraryName : String) : integer;
>>>>>>> graemeg/fixes_2_2
=======
function InitialiseIBase60(Const LibraryName : String) : integer;
>>>>>>> origin/fixes_2_2

begin
  Result := TryInitialiseIBase60(LibraryName);
  If Result = 0 then
    Raise EInOutError.CreateFmt(SErrLoadFailed,[LibraryName])
<<<<<<< HEAD
<<<<<<< HEAD
  else If (LibraryName<>IBaseLoadedLibrary) then
    begin
    Dec(RefCount);
    Result := RefCount;
    Raise EInOUtError.CreateFmt(SErrAlreadyLoaded,[IBaseLoadedLibrary]);
=======
=======
>>>>>>> origin/fixes_2_2
  else If (LibraryName<>LoadedLibrary) then
    begin
    Dec(RefCount);
    Result := RefCount;
    Raise EInOUtError.CreateFmt(SErrAlreadyLoaded,[LoadedLibrary]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;


Procedure ReleaseIBase60;

begin
  if RefCount>1 then
    Dec(RefCount)
<<<<<<< HEAD
<<<<<<< HEAD
  else
    begin
    // Shutdown embedded subsystem with timeout 300ms (Firebird 2.5+)
    // Required before unloading library; has no effect on non-embedded client
    if (pointer(fb_shutdown)<>nil) and (fb_shutdown(300,1)<>0) then
      begin
      //todo: log error; still try to unload library below as the timeout may have been insufficient
      end;
    if UnloadLibrary(LibHandle) then
      begin
      Dec(RefCount);
      LibHandle := NilHandle;
      LoadedLibrary:='';
      end;
=======
=======
>>>>>>> origin/fixes_2_2
  else if UnloadLibrary(IBaseLibraryHandle) then 
    begin
    Dec(RefCount);
    IBaseLibraryHandle := NilHandle;
    LoadedLibrary:='';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    end;
end;

{$ENDIF}

function XSQLDA_LENGTH(n: Integer): Integer;
begin
  Result := SizeOf(XSQLDA) + (n - 1) * SizeOf(XSQLVAR);
end;


