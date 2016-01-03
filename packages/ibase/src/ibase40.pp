{
}
unit ibase40;

interface

const
  {$ifdef Unix}
  LibName = 'gds';
  {$linklib c}
  {$else}
  LibName = 'gds32.dll';
  {$endif}


{ C Compatibility types }

type
  {  Unsigned types }

  UChar                = Byte;
  UShort               = Word;
  UInt                 = DWord;
  ULong                = DWord;

  { Signed types }

  Int                  = LongInt;
  Long                 = LongInt;
  Short                = Integer;
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

  { Interbase redefinitions }

  ISC_LONG             = Long;
  UISC_LONG            = ULong;
  ISC_STATUS           = Long;
  UISC_STATUS          = ULong;
  Void                 = Pointer;
  PISC_LONG            = ^ISC_LONG;
  PUISC_LONG           = ^UISC_LONG;
  PISC_STATUS          = ^ISC_STATUS;
  PPISC_STATUS         = ^PISC_STATUS;
  PUISC_STATUS         = ^UISC_STATUS;

  { C Date/Time Structure }
  TCTimeStructure = record
    tm_sec,
    tm_min,
    tm_hour,
    tm_mday,
    tm_mon,
    tm_year,
    tm_wday,
    tm_yday,
    tm_isdst : Longint;
  end;
  PCTimeStructure = ^TCTimeStructure;
  TM              = TCTimeStructure;
  PTM             = ^TM;

  TISC_VARYING = record
    strlen: Short;
    str: array[0..0] of Char;
  end;

  TISC_BlobGetSegment = function(BlobHandle: PInt; Buffer: PChar; BufferSize: Long; var ResultLength: Long): Short; cdecl;
  TISC_BlobPutSegment = procedure(BlobHandle: PInt; Buffer: PChar; BufferLength: Short); cdecl;

  TBlob = record
    GetSegment         : TISC_BlobGetSegment;
    BlobHandle         : PInt;
    SegmentCount       : Long;
    MaxSegmentLength   : Long;
    TotalSize          : Long;
    PutSegment         : TISC_BlobPutSegment;
  end;
  PBlob = ^TBlob;

Const
  {
    Huge amount of constants.
    Look for TYPESTART to get to types,
    Look For FUNCSTART to get to functions and procedures
  }

  ISC_TRUE = 1;
  ISC_FALSE = 0;
  DSQL_close = 1;
  DSQL_drop = 2;

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

  isc_blob_filter_open = 0;
  isc_blob_filter_get_segment = 1;
  isc_blob_filter_close = 2;
  isc_blob_filter_create = 3;
  isc_blob_filter_put_segment = 4;
  isc_blob_filter_alloc = 5;
  isc_blob_filter_free = 6;
  isc_blob_filter_seek = 7;

  blr_text = 14;
  blr_text2 = 15;
  blr_short = 7;
  blr_long = 8;
  blr_quad = 9;
  blr_float = 10;
  blr_double = 27;
  blr_d_float = 11;
  blr_date = 35;
  blr_varying = 37;
  blr_varying2 = 38;
  blr_blob = 261;
  blr_cstring = 40;
  blr_cstring2 = 41;
  blr_blob_id = 45;

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
  blr_eoc = 76;
  blr_end = -1;

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

  blr_record_version = 154;
  blr_stall = 155;
  blr_seek_no_warn = 156;
  blr_find_dbkey_version = 157;
  blr_ansi_all = 158;

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
  isc_dpb_last_dpb_constant = isc_dpb_working_directory;


  isc_dpb_pages = 1;
  isc_dpb_records = 2;
  isc_dpb_indices = 4;
  isc_dpb_transactions = 8;
  isc_dpb_no_update = 16;
  isc_dpb_repair = 32;
  isc_dpb_ignore = 64;

  isc_dpb_shut_cache = 1;
  isc_dpb_shut_attachment = 2;
  isc_dpb_shut_transaction = 4;
  isc_dpb_shut_force = 8;

  RDB_system = 1;
  RDB_id_assigned = 2;


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
  isc_tpb_last_tpb_constant = isc_tpb_no_auto_undo;


  isc_bpb_version1 = 1;
  isc_bpb_source_type = 1;
  isc_bpb_target_type = 2;
  isc_bpb_type = 3;
  isc_bpb_source_interp = 4;
  isc_bpb_target_interp = 5;
  isc_bpb_filter_parameter = 6;

  isc_bpb_type_segmented = 0;
  isc_bpb_type_stream = 1;


  isc_spb_version1 = 1;
  isc_spb_user_name = 2;
  isc_spb_sys_user_name = 3;
  isc_spb_sys_user_name_enc = 4;
  isc_spb_password = 5;
  isc_spb_password_enc = 6;
  isc_spb_command_line = 7;
  isc_spb_connect_timeout = isc_dpb_connect_timeout;
  isc_spb_dummy_packet_interval = isc_dpb_dummy_packet_interval;
  isc_spb_sql_role_name = isc_dpb_sql_role_name;



  isc_info_end = 1;
  isc_info_truncated = 2;
  isc_info_error = 3;

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


  isc_info_rsb_end = 0;
  isc_info_rsb_begin = 1;
  isc_info_rsb_type = 2;
  isc_info_rsb_relation = 3;
  isc_info_rsb_plan = 4;

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

  isc_info_blob_num_segments = 4;
  isc_info_blob_max_segment = 5;
  isc_info_blob_total_length = 6;
  isc_info_blob_type = 7;

  isc_info_tra_id = 4;

  isc_info_svc_version = 4;
  isc_info_svc_message = 5;
  isc_info_svc_total_length = 6;
  isc_info_svc_response = 7;
  isc_info_svc_response_more = 8;
  isc_info_svc_line = 9;
  isc_info_svc_to_eof = 10;
  isc_info_svc_timeout = 11;
  isc_info_svc_server_version = 12;
  isc_info_svc_implementation = 13;
  isc_info_svc_capabilities = 14;
  isc_info_svc_user_dbpath = 15;
  isc_info_svc_svr_db_info = 16;
  isc_info_svc_svr_online = 17;
  isc_info_svc_svr_offline = 18;
  isc_info_svc_get_config = 19;
  isc_info_svc_set_config = 20;
  isc_info_svc_default_config = 21;
  isc_info_svc_get_env = 22;
  isc_info_svc_get_env_lock = 23;
  isc_info_svc_get_env_msg = 24;

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

  isc_arith_except = 335544321;
  isc_bad_dbkey = 335544322;
  isc_bad_db_format = 335544323;
  isc_bad_db_handle = 335544324;
  isc_bad_dpb_content = 335544325;
  isc_bad_dpb_form = 335544326;
  isc_bad_req_handle = 335544327;
  isc_bad_segstr_handle = 335544328;
  isc_bad_segstr_id = 335544329;
  isc_bad_tpb_content = 335544330;
  isc_bad_tpb_form = 335544331;
  isc_bad_trans_handle = 335544332;
  isc_bug_check = 335544333;
  isc_convert_error = 335544334;
  isc_db_corrupt = 335544335;
  isc_deadlock = 335544336;
  isc_excess_trans = 335544337;
  isc_from_no_match = 335544338;
  isc_infinap = 335544339;
  isc_infona = 335544340;
  isc_infunk = 335544341;
  isc_integ_fail = 335544342;
  isc_invalid_blr = 335544343;
  isc_io_error = 335544344;
  isc_lock_conflict = 335544345;
  isc_metadata_corrupt = 335544346;
  isc_not_valid = 335544347;
  isc_no_cur_rec = 335544348;
  isc_no_dup = 335544349;
  isc_no_finish = 335544350;
  isc_no_meta_update = 335544351;
  isc_no_priv = 335544352;
  isc_no_recon = 335544353;
  isc_no_record = 335544354;
  isc_no_segstr_close = 335544355;
  isc_obsolete_metadata = 335544356;
  isc_open_trans = 335544357;
  isc_port_len = 335544358;
  isc_read_only_field = 335544359;
  isc_read_only_rel = 335544360;
  isc_read_only_trans = 335544361;
  isc_read_only_view = 335544362;
  isc_req_no_trans = 335544363;
  isc_req_sync = 335544364;
  isc_req_wrong_db = 335544365;
  isc_segment = 335544366;
  isc_segstr_eof = 335544367;
  isc_segstr_no_op = 335544368;
  isc_segstr_no_read = 335544369;
  isc_segstr_no_trans = 335544370;
  isc_segstr_no_write = 335544371;
  isc_segstr_wrong_db = 335544372;
  isc_sys_request = 335544373;
  isc_stream_eof = 335544374;
  isc_unavailable = 335544375;
  isc_unres_rel = 335544376;
  isc_uns_ext = 335544377;
  isc_wish_list = 335544378;
  isc_wrong_ods = 335544379;
  isc_wronumarg = 335544380;
  isc_imp_exc = 335544381;
  isc_random = 335544382;
  isc_fatal_conflict = 335544383;
  isc_badblk = 335544384;
  isc_invpoolcl = 335544385;
  isc_nopoolids = 335544386;
  isc_relbadblk = 335544387;
  isc_blktoobig = 335544388;
  isc_bufexh = 335544389;
  isc_syntaxerr = 335544390;
  isc_bufinuse = 335544391;
  isc_bdbincon = 335544392;
  isc_reqinuse = 335544393;
  isc_badodsver = 335544394;
  isc_relnotdef = 335544395;
  isc_fldnotdef = 335544396;
  isc_dirtypage = 335544397;
  isc_waifortra = 335544398;
  isc_doubleloc = 335544399;
  isc_nodnotfnd = 335544400;
  isc_dupnodfnd = 335544401;
  isc_locnotmar = 335544402;
  isc_badpagtyp = 335544403;
  isc_corrupt = 335544404;
  isc_badpage = 335544405;
  isc_badindex = 335544406;
  isc_dbbnotzer = 335544407;
  isc_tranotzer = 335544408;
  isc_trareqmis = 335544409;
  isc_badhndcnt = 335544410;
  isc_wrotpbver = 335544411;
  isc_wroblrver = 335544412;
  isc_wrodpbver = 335544413;
  isc_blobnotsup = 335544414;
  isc_badrelation = 335544415;
  isc_nodetach = 335544416;
  isc_notremote = 335544417;
  isc_trainlim = 335544418;
  isc_notinlim = 335544419;
  isc_traoutsta = 335544420;
  isc_connect_reject = 335544421;
  isc_dbfile = 335544422;
  isc_orphan = 335544423;
  isc_no_lock_mgr = 335544424;
  isc_ctxinuse = 335544425;
  isc_ctxnotdef = 335544426;
  isc_datnotsup = 335544427;
  isc_badmsgnum = 335544428;
  isc_badparnum = 335544429;
  isc_virmemexh = 335544430;
  isc_blocking_signal = 335544431;
  isc_lockmanerr = 335544432;
  isc_journerr = 335544433;
  isc_keytoobig = 335544434;
  isc_nullsegkey = 335544435;
  isc_sqlerr = 335544436;
  isc_wrodynver = 335544437;
  isc_funnotdef = 335544438;
  isc_funmismat = 335544439;
  isc_bad_msg_vec = 335544440;
  isc_bad_detach = 335544441;
  isc_noargacc_read = 335544442;
  isc_noargacc_write = 335544443;
  isc_read_only = 335544444;
  isc_ext_err = 335544445;
  isc_non_updatable = 335544446;
  isc_no_rollback = 335544447;
  isc_bad_sec_info = 335544448;
  isc_invalid_sec_info = 335544449;
  isc_misc_interpreted = 335544450;
  isc_update_conflict = 335544451;
  isc_unlicensed = 335544452;
  isc_obj_in_use = 335544453;
  isc_nofilter = 335544454;
  isc_shadow_accessed = 335544455;
  isc_invalid_sdl = 335544456;
  isc_out_of_bounds = 335544457;
  isc_invalid_dimension = 335544458;
  isc_rec_in_limbo = 335544459;
  isc_shadow_missing = 335544460;
  isc_cant_validate = 335544461;
  isc_cant_start_journal = 335544462;
  isc_gennotdef = 335544463;
  isc_cant_start_logging = 335544464;
  isc_bad_segstr_type = 335544465;
  isc_foreign_key = 335544466;
  isc_high_minor = 335544467;
  isc_tra_state = 335544468;
  isc_trans_invalid = 335544469;
  isc_buf_invalid = 335544470;
  isc_indexnotdefined = 335544471;
  isc_login = 335544472;
  isc_invalid_bookmark = 335544473;
  isc_bad_lock_level = 335544474;
  isc_relation_lock = 335544475;
  isc_record_lock = 335544476;
  isc_max_idx = 335544477;
  isc_jrn_enable = 335544478;
  isc_old_failure = 335544479;
  isc_old_in_progress = 335544480;
  isc_old_no_space = 335544481;
  isc_no_wal_no_jrn = 335544482;
  isc_num_old_files = 335544483;
  isc_wal_file_open = 335544484;
  isc_bad_stmt_handle = 335544485;
  isc_wal_failure = 335544486;
  isc_walw_err = 335544487;
  isc_logh_small = 335544488;
  isc_logh_inv_version = 335544489;
  isc_logh_open_flag = 335544490;
  isc_logh_open_flag2 = 335544491;
  isc_logh_diff_dbname = 335544492;
  isc_logf_unexpected_eof = 335544493;
  isc_logr_incomplete = 335544494;
  isc_logr_header_small = 335544495;
  isc_logb_small = 335544496;
  isc_wal_illegal_attach = 335544497;
  isc_wal_invalid_wpb = 335544498;
  isc_wal_err_rollover = 335544499;
  isc_no_wal = 335544500;
  isc_drop_wal = 335544501;
  isc_stream_not_defined = 335544502;
  isc_wal_subsys_error = 335544503;
  isc_wal_subsys_corrupt = 335544504;
  isc_no_archive = 335544505;
  isc_shutinprog = 335544506;
  isc_range_in_use = 335544507;
  isc_range_not_found = 335544508;
  isc_charset_not_found = 335544509;
  isc_lock_timeout = 335544510;
  isc_prcnotdef = 335544511;
  isc_prcmismat = 335544512;
  isc_wal_bugcheck = 335544513;
  isc_wal_cant_expand = 335544514;
  isc_codnotdef = 335544515;
  isc_xcpnotdef = 335544516;
  isc_except = 335544517;
  isc_cache_restart = 335544518;
  isc_bad_lock_handle = 335544519;
  isc_jrn_present = 335544520;
  isc_wal_err_rollover2 = 335544521;
  isc_wal_err_logwrite = 335544522;
  isc_wal_err_jrn_comm = 335544523;
  isc_wal_err_expansion = 335544524;
  isc_wal_err_setup = 335544525;
  isc_wal_err_ww_sync = 335544526;
  isc_wal_err_ww_start = 335544527;
  isc_shutdown = 335544528;
  isc_existing_priv_mod = 335544529;
  isc_primary_key_ref = 335544530;
  isc_primary_key_notnull = 335544531;
  isc_ref_cnstrnt_notfound = 335544532;
  isc_foreign_key_notfound = 335544533;
  isc_ref_cnstrnt_update = 335544534;
  isc_check_cnstrnt_update = 335544535;
  isc_check_cnstrnt_del = 335544536;
  isc_integ_index_seg_del = 335544537;
  isc_integ_index_seg_mod = 335544538;
  isc_integ_index_del = 335544539;
  isc_integ_index_mod = 335544540;
  isc_check_trig_del = 335544541;
  isc_check_trig_update = 335544542;
  isc_cnstrnt_fld_del = 335544543;
  isc_cnstrnt_fld_rename = 335544544;
  isc_rel_cnstrnt_update = 335544545;
  isc_constaint_on_view = 335544546;
  isc_invld_cnstrnt_type = 335544547;
  isc_primary_key_exists = 335544548;
  isc_systrig_update = 335544549;
  isc_not_rel_owner = 335544550;
  isc_grant_obj_notfound = 335544551;
  isc_grant_fld_notfound = 335544552;
  isc_grant_nopriv = 335544553;
  isc_nonsql_security_rel = 335544554;
  isc_nonsql_security_fld = 335544555;
  isc_wal_cache_err = 335544556;
  isc_shutfail = 335544557;
  isc_check_constraint = 335544558;
  isc_bad_svc_handle = 335544559;
  isc_shutwarn = 335544560;
  isc_wrospbver = 335544561;
  isc_bad_spb_form = 335544562;
  isc_svcnotdef = 335544563;
  isc_no_jrn = 335544564;
  isc_transliteration_failed = 335544565;
  isc_start_cm_for_wal = 335544566;
  isc_wal_ovflow_log_required = 335544567;
  isc_text_subtype = 335544568;
  isc_dsql_error = 335544569;
  isc_dsql_command_err = 335544570;
  isc_dsql_constant_err = 335544571;
  isc_dsql_cursor_err = 335544572;
  isc_dsql_datatype_err = 335544573;
  isc_dsql_decl_err = 335544574;
  isc_dsql_cursor_update_err = 335544575;
  isc_dsql_cursor_open_err = 335544576;
  isc_dsql_cursor_close_err = 335544577;
  isc_dsql_field_err = 335544578;
  isc_dsql_internal_err = 335544579;
  isc_dsql_relation_err = 335544580;
  isc_dsql_procedure_err = 335544581;
  isc_dsql_request_err = 335544582;
  isc_dsql_sqlda_err = 335544583;
  isc_dsql_var_count_err = 335544584;
  isc_dsql_stmt_handle = 335544585;
  isc_dsql_function_err = 335544586;
  isc_dsql_blob_err = 335544587;
  isc_collation_not_found = 335544588;
  isc_collation_not_for_charset = 335544589;
  isc_dsql_dup_option = 335544590;
  isc_dsql_tran_err = 335544591;
  isc_dsql_invalid_array = 335544592;
  isc_dsql_max_arr_dim_exceeded = 335544593;
  isc_dsql_arr_range_error = 335544594;
  isc_dsql_trigger_err = 335544595;
  isc_dsql_subselect_err = 335544596;
  isc_dsql_crdb_prepare_err = 335544597;
  isc_specify_field_err = 335544598;
  isc_num_field_err = 335544599;
  isc_col_name_err = 335544600;
  isc_where_err = 335544601;
  isc_table_view_err = 335544602;
  isc_distinct_err = 335544603;
  isc_key_field_count_err = 335544604;
  isc_subquery_err = 335544605;
  isc_expression_eval_err = 335544606;
  isc_node_err = 335544607;
  isc_command_end_err = 335544608;
  isc_index_name = 335544609;
  isc_exception_name = 335544610;
  isc_field_name = 335544611;
  isc_token_err = 335544612;
  isc_union_err = 335544613;
  isc_dsql_construct_err = 335544614;
  isc_field_aggregate_err = 335544615;
  isc_field_ref_err = 335544616;
  isc_order_by_err = 335544617;
  isc_return_mode_err = 335544618;
  isc_extern_func_err = 335544619;
  isc_alias_conflict_err = 335544620;
  isc_procedure_conflict_error = 335544621;
  isc_relation_conflict_err = 335544622;
  isc_dsql_domain_err = 335544623;
  isc_idx_seg_err = 335544624;
  isc_node_name_err = 335544625;
  isc_table_name = 335544626;
  isc_proc_name = 335544627;
  isc_idx_create_err = 335544628;
  isc_wal_shadow_err = 335544629;
  isc_dependency = 335544630;
  isc_idx_key_err = 335544631;
  isc_dsql_file_length_err = 335544632;
  isc_dsql_shadow_number_err = 335544633;
  isc_dsql_token_unk_err = 335544634;
  isc_dsql_no_relation_alias = 335544635;
  isc_indexname = 335544636;
  isc_no_stream_plan = 335544637;
  isc_stream_twice = 335544638;
  isc_stream_not_found = 335544639;
  isc_collation_requires_text = 335544640;
  isc_dsql_domain_not_found = 335544641;
  isc_index_unused = 335544642;
  isc_dsql_self_join = 335544643;
  isc_stream_bof = 335544644;
  isc_stream_crack = 335544645;
  isc_db_or_file_exists = 335544646;
  isc_invalid_operator = 335544647;
  isc_conn_lost = 335544648;
  isc_bad_checksum = 335544649;
  isc_page_type_err = 335544650;
  isc_ext_readonly_err = 335544651;
  isc_sing_select_err = 335544652;
  isc_psw_attach = 335544653;
  isc_psw_start_trans = 335544654;
  isc_invalid_direction = 335544655;
  isc_dsql_var_conflict = 335544656;
  isc_dsql_no_blob_array = 335544657;
  isc_dsql_base_table = 335544658;
  isc_duplicate_base_table = 335544659;
  isc_view_alias = 335544660;
  isc_index_root_page_full = 335544661;
  isc_dsql_blob_type_unknown = 335544662;
  isc_req_max_clones_exceeded = 335544663;
  isc_dsql_duplicate_spec = 335544664;
  isc_unique_key_violation = 335544665;
  isc_srvr_version_too_old = 335544666;
  isc_drdb_completed_with_errs = 335544667;
  isc_dsql_procedure_use_err = 335544668;
  isc_dsql_count_mismatch = 335544669;
  isc_blob_idx_err = 335544670;
  isc_array_idx_err = 335544671;
  isc_key_field_err = 335544672;
  isc_no_delete = 335544673;
  isc_del_last_field = 335544674;
  isc_sort_err = 335544675;
  isc_sort_mem_err = 335544676;
  isc_version_err = 335544677;
  isc_inval_key_posn = 335544678;
  isc_no_segments_err = 335544679;
  isc_crrp_data_err = 335544680;
  isc_rec_size_err = 335544681;
  isc_dsql_field_ref = 335544682;
  isc_req_depth_exceeded = 335544683;
  isc_no_field_access = 335544684;
  isc_no_dbkey = 335544685;
  isc_jrn_format_err = 335544686;
  isc_jrn_file_full = 335544687;
  isc_dsql_open_cursor_request = 335544688;
  isc_ib_error = 335544689;
  isc_cache_redef = 335544690;
  isc_cache_too_small = 335544691;
  isc_log_redef = 335544692;
  isc_log_too_small = 335544693;
  isc_partition_too_small = 335544694;
  isc_partition_not_supp = 335544695;
  isc_log_length_spec = 335544696;
  isc_precision_err = 335544697;
  isc_scale_nogt = 335544698;
  isc_expec_short = 335544699;
  isc_expec_long = 335544700;
  isc_expec_ushort = 335544701;
  isc_like_escape_invalid = 335544702;
  isc_svcnoexe = 335544703;
  isc_net_lookup_err = 335544704;
  isc_service_unknown = 335544705;
  isc_host_unknown = 335544706;
  isc_grant_nopriv_on_base = 335544707;
  isc_dyn_fld_ambiguous = 335544708;
  isc_dsql_agg_ref_err = 335544709;
  isc_complex_view = 335544710;
  isc_unprepared_stmt = 335544711;
  isc_expec_positive = 335544712;
  isc_dsql_sqlda_value_err = 335544713;
  isc_invalid_array_id = 335544714;
  isc_extfile_uns_op = 335544715;
  isc_svc_in_use = 335544716;
  isc_err_stack_limit = 335544717;
  isc_invalid_key = 335544718;
  isc_net_init_error = 335544719;
  isc_loadlib_failure = 335544720;
  isc_network_error = 335544721;
  isc_net_connect_err = 335544722;
  isc_net_connect_listen_err = 335544723;
  isc_net_event_connect_err = 335544724;
  isc_net_event_listen_err = 335544725;
  isc_net_read_err = 335544726;
  isc_net_write_err = 335544727;
  isc_integ_index_deactivate = 335544728;
  isc_integ_deactivate_primary = 335544729;
  isc_cse_not_supported = 335544730;
  isc_tra_must_sweep = 335544731;
  isc_unsupported_network_drive = 335544732;
  isc_io_create_err = 335544733;
  isc_io_open_err = 335544734;
  isc_io_close_err = 335544735;
  isc_io_read_err = 335544736;
  isc_io_write_err = 335544737;
  isc_io_delete_err = 335544738;
  isc_io_access_err = 335544739;
  isc_udf_exception = 335544740;
  isc_lost_db_connection = 335544741;
  isc_no_write_user_priv = 335544742;
  isc_token_too_long = 335544743;
  isc_max_att_exceeded = 335544744;
  isc_login_same_as_role_name = 335544745;
  isc_usrname_too_long = 335544747;
  isc_password_too_long = 335544748;
  isc_usrname_required = 335544749;
  isc_password_required = 335544750;
  isc_bad_protocol = 335544751;
  isc_dup_usrname_found = 335544752;
  isc_usrname_not_found = 335544753;
  isc_error_adding_sec_record = 335544754;
  isc_error_modifying_sec_record = 335544755;
  isc_error_deleting_sec_record = 335544756;
  isc_error_updating_sec_db = 335544757;
  isc_err_max = 425;


  isc_dyn_version_1 = 1;
  isc_dyn_eoc = -1;

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

  isc_dyn_view_blr = 43;
  isc_dyn_view_source = 44;
  isc_dyn_view_relation = 45;
  isc_dyn_view_context = 46;
  isc_dyn_view_context_name = 47;

  isc_dyn_rel_name = 50;
  isc_dyn_fld_name = 51;
  isc_dyn_idx_name = 52;
  isc_dyn_description = 53;
  isc_dyn_security_class = 54;
  isc_dyn_system_flag = 55;
  isc_dyn_update_flag = 56;
  isc_dyn_prc_name = 166;
  isc_dyn_prm_name = 137;
  isc_dyn_sql_object = 196;
  isc_dyn_fld_character_set_name = 174;

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
  isc_dyn_fld_char_length = 172;
  isc_dyn_fld_collation = 173;
  isc_dyn_fld_default_source = 193;
  isc_dyn_del_default = 197;
  isc_dyn_del_validation = 198;
  isc_dyn_single_validation = 199;
  isc_dyn_fld_character_set = 203;

  isc_dyn_fld_source = 90;
  isc_dyn_fld_base_fld = 91;
  isc_dyn_fld_position = 92;
  isc_dyn_fld_update_flag = 93;

  isc_dyn_idx_unique = 100;
  isc_dyn_idx_inactive = 101;
  isc_dyn_idx_type = 103;
  isc_dyn_idx_foreign_key = 104;
  isc_dyn_idx_ref_column = 105;
  isc_dyn_idx_statistic = 204;

  isc_dyn_trg_type = 110;
  isc_dyn_trg_blr = 111;
  isc_dyn_trg_source = 112;
  isc_dyn_trg_name = 114;
  isc_dyn_trg_sequence = 115;
  isc_dyn_trg_inactive = 116;
  isc_dyn_trg_msg_number = 117;
  isc_dyn_trg_msg = 118;

  isc_dyn_scl_acl = 121;
  isc_dyn_grant_user = 130;
  isc_dyn_grant_proc = 186;
  isc_dyn_grant_trig = 187;
  isc_dyn_grant_view = 188;
  isc_dyn_grant_options = 132;
  isc_dyn_grant_user_group = 205;

  isc_dyn_dim_lower = 141;
  isc_dyn_dim_upper = 142;

  isc_dyn_file_name = 125;
  isc_dyn_file_start = 126;
  isc_dyn_file_length = 127;
  isc_dyn_shadow_number = 128;
  isc_dyn_shadow_man_auto = 129;
  isc_dyn_shadow_conditional = 130;

  isc_dyn_log_file_sequence = 177;
  isc_dyn_log_file_partitions = 178;
  isc_dyn_log_file_serial = 179;
  isc_dyn_log_file_overflow = 200;
  isc_dyn_log_file_raw = 201;

  isc_dyn_log_group_commit_wait = 189;
   isc_dyn_log_buffer_size = 190;
  isc_dyn_log_check_point_length = 191;
  isc_dyn_log_num_of_buffers = 192;

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

  isc_dyn_generator_name = 95;
  isc_dyn_generator_id = 96;

  isc_dyn_prc_inputs = 167;
  isc_dyn_prc_outputs = 168;
  isc_dyn_prc_source = 169;
  isc_dyn_prc_blr = 170;
  isc_dyn_prc_source2 = 171;

  isc_dyn_prm_number = 138;
  isc_dyn_prm_type = 139;

  isc_dyn_xcp_msg = 185;

  isc_dyn_foreign_key_update = 205;
  isc_dyn_foreign_key_delete = 206;
  isc_dyn_foreign_key_cascade = 207;
  isc_dyn_foreign_key_default = 208;
  isc_dyn_foreign_key_null = 209;
  isc_dyn_foreign_key_none = 210;

  isc_dyn_def_sql_role = 211;
  isc_dyn_sql_role_name = 212;
  isc_dyn_grant_admin_options = 213;
  isc_dyn_del_sql_role = 214;

  isc_dyn_last_dyn_value = 214;

  isc_sdl_version1 = 1;
  isc_sdl_eoc = -1;
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

  isc_interp_eng_ascii = 0;
  isc_interp_jpn_sjis = 5;
  isc_interp_jpn_euc = 6;

  isc_fetch_next = 0;
  isc_fetch_prior = 1;
  isc_fetch_first = 2;
  isc_fetch_last = 3;
  isc_fetch_absolute = 4;
  isc_fetch_relative = 5;

  SQLDA_VERSION1 = 1;

  SQL_VARYING = 448;
  SQL_TEXT = 452;
  SQL_DOUBLE = 480;
  SQL_FLOAT = 482;
  SQL_LONG = 496;
  SQL_SHORT = 500;
  SQL_DATE = 510;
  SQL_BLOB = 520;
  SQL_D_FLOAT = 530;
  SQL_ARRAY = 540;
  SQL_QUAD = 550;


  isc_blob_untyped = 0;

  isc_blob_text = 1;
  isc_blob_blr = 2;
  isc_blob_acl = 3;
  isc_blob_ranges = 4;
  isc_blob_summary = 5;
  isc_blob_format = 6;
  isc_blob_tra = 7;
  isc_blob_extfile = 8;

  isc_blob_formatted_memo = 20;
  isc_blob_paradox_ole = 21;
  isc_blob_graphic = 22;
  isc_blob_dbase_ole = 23;
  isc_blob_typed_binary = 24;

  // TYPESTART  (to quickly get here, look for TYPESTART)

type

  TISC_ATT_HANDLE = PVoid;
  PISC_ATT_HANDLE = ^TISC_ATT_HANDLE;
  TISC_BLOB_HANDLE = PVoid;
  PISC_BLOB_HANDLE = ^TISC_BLOB_HANDLE;
  TISC_DB_HANDLE = PVoid;
  PISC_DB_HANDLE = ^TISC_DB_HANDLE;
  TISC_FORM_HANDLE = PVoid;
  PISC_FORM_HANDLE = ^TISC_FORM_HANDLE;
  TISC_REQ_HANDLE = PVoid;
  PISC_REQ_HANDLE = ^TISC_REQ_HANDLE;
  TISC_STMT_HANDLE = PVoid;
  PISC_STMT_HANDLE = ^TISC_STMT_HANDLE;
  TISC_SVC_HANDLE = PVoid;
  PISC_SVC_HANDLE = ^TISC_SVC_HANDLE;
  TISC_TR_HANDLE = PVoid;
  PISC_TR_HANDLE = ^TISC_TR_HANDLE;
  TISC_WIN_HANDLE = PVoid;
  PISC_WIN_HANDLE = ^TISC_WIN_HANDLE;
  TISC_CALLBACK = procedure;

  TGDS_QUAD = record
    gds_quad_high : ISC_LONG;
    gds_quad_low : UISC_LONG;
  end;
  TGDS__QUAD = TGDS_QUAD;
  TISC_QUAD = TGDS_QUAD;
  PGDS_QUAD = ^TGDS_QUAD;
  PGDS__QUAD = ^TGDS__QUAD;
  PISC_QUAD = ^TISC_QUAD;

  TISC_ARRAY_BOUND = record
    array_bound_lower : short;
    array_bound_upper : short;
  end;
  PISC_ARRAY_BOUND = ^TISC_ARRAY_BOUND;
  TISC_ARRAY_DESC = record
    array_desc_dtype : UChar;
    array_desc_scale : Char;
    array_desc_length : UShort;
    array_desc_field_name : array[0..31] of Char;
    array_desc_relation_name : array[0..31] of Char;
    array_desc_dimensions : Short;
    array_desc_flags : Short;
    array_desc_bounds : array[0..15] of TISC_ARRAY_BOUND;
  end;
  PISC_ARRAY_DESC = ^TISC_ARRAY_DESC;

  TISC_BLOB_DESC = record
    blob_desc_subtype : Short;
    blob_desc_charset : Short;
    blob_desc_segment_size : Short;
    blob_desc_field_name : array[0..31] of UChar;
    blob_desc_relation_name : array[0..31] of UChar;
  end;
  PISC_BLOB_DESC = ^TISC_BLOB_DESC;

  TISC_BLOB_CTL_SOURCE_FUNCTION = function : ISC_STATUS; // ISC_FAR
  PISC_BLOB_CTL = ^TISC_BLOB_CTL;        // ISC_FAR
  TISC_BLOB_CTL = record
    ctl_source : TISC_BLOB_CTL_SOURCE_FUNCTION;
    ctl_source_handle : PISC_BLOB_CTL;
    ctl_to_sub_type : Short;
    ctl_from_sub_type : Short;
    ctl_buffer_length : UShort;
    ctl_segment_length : UShort;
    ctl_bpb_length : UShort;
    ctl_bpb : PChar;
    ctl_buffer : PUChar;
    ctl_max_segment : ISC_LONG;
    ctl_number_segments : ISC_LONG;
    ctl_total_length : ISC_LONG;
    ctl_status : PISC_STATUS;
    ctl_data : array[0..7] of long;
  end;

  TBSTREAM = record
    bstr_blob : PVoid;
    bstr_buffer : PChar;
    bstr_ptr : PChar;
    bstr_length : Short;
    bstr_cnt : Short;
    bstr_mode : Char;
  end;
  PBSTREAM = ^TBSTREAM;

  TSQLVAR = record
    sqltype : Short;
    sqllen : Short;
    sqldata : PChar;
    sqlind : PShort;
    sqlname_length : Short;
    sqlname : array[0..29] of Char;
  end;
  PSQLVAR = ^TSQLVAR;

  TSQLDA = record
    sqldaid : array[0..7] of Char;
    sqldabc : ISC_LONG;
    sqln : Short;
    sqld : Short;
    sqlvar : array[0..0] of TSQLVAR;
  end;
  PSQLDA = ^TSQLDA;

  TXSQLVAR = record
    sqltype : Short;
    sqlscale : Short;
    sqlsubtype : Short;
    sqllen : Short;
    sqldata : PChar;
    sqlind : PShort;
    sqlname_length : Short;
    sqlname : array[0..31] of Char;
    relname_length : Short;
    relname : array[0..31] of Char;
    ownname_length : Short;
    ownname : array[0..31] of Char;
    aliasname_length : Short;
    aliasname : array[0..31] of Char;
  end;
  PXSQLVAR = ^TXSQLVAR;

  TXSQLDA = record
    version : Short;
    pad : Short;
    sqldaid : array[0..7] of Char;
    sqldabc : ISC_LONG;
    sqln : Short;
    sqld : Short;
    sqlvar : array[0..0] of TXSQLVAR;
  end;
  PXSQLDA = ^TXSQLDA;

  TISC_START_TRANS = record
    db_handle : PISC_DB_HANDLE;
    tpb_length : UShort;   // Should it be long?
    tpb_address : PChar;
  end;

  TISC_TEB = record
    db_handle : PISC_DB_HANDLE;
    tpb_length : Long;
    tpb_address : PChar;
  end;
  PISC_TEB = ^TISC_TEB;
  TISC_TEB_ARRAY = array[0..0] of TISC_TEB;
  PISC_TEB_ARRAY = ^TISC_TEB_ARRAY;

TUserSecData = record
  sec_flags : Short;
  uid : Int;
  gid : int;
  protocol : Int;
  server : PChar;
  user_name : PChar;
  password : PChar;
  group_name : PChar;
  first_name : PChar;
  middle_name : PChar;
  last_name : PChar;
  dba_user_name : PChar;
  dba_password : PChar;
  end;
PUserSecData = ^TUserSecData;

// FUNCSTART  (to quickly get here, look for FUNCSTART)

function isc_attach_database (status_vector : PISC_STATUS;db_name_length: Short;db_name: PChar;db_handle: PISC_DB_HANDLE;parm_buffer_length:Short;parm_buffer: PChar): ISC_STATUS; stdcall; external LibName;
function isc_array_gen_sdl (status_vector : PISC_STATUS;isc_array_desc: PISC_ARRAY_DESC;isc_arg3 : PShort;isc_arg4 : PChar;isc_arg5 : PShort): ISC_STATUS;stdcall; external LibName;
function isc_array_get_slice (status_vector : PISC_STATUS;db_handle: PISC_DB_HANDLE;trans_handle : PISC_TR_HANDLE;array_id                 : PISC_QUAD;descriptor               : PISC_ARRAY_DESC;dest_array : PVoid;slice_length : ISC_LONG): ISC_STATUS;stdcall; external LibName;
function isc_array_lookup_bounds (status_vector : PISC_STATUS;db_handle: PISC_DB_HANDLE;trans_handle : PISC_TR_HANDLE;table_name,column_name              : PChar;descriptor               : PISC_ARRAY_DESC): ISC_STATUS;stdcall; external LibName;
function isc_array_lookup_desc (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;trans_handle : PISC_TR_HANDLE;table_name,column_name : PChar;descriptor : PISC_ARRAY_DESC) : ISC_STATUS; stdcall; external LibName;
function isc_array_set_desc (status_vector : PISC_STATUS;table_name : PChar;column_name : PChar;sql_dtype,sql_length,sql_dimensions : PShort;descriptor : PISC_ARRAY_DESC) : ISC_STATUS;stdcall; external LibName;
function isc_array_put_slice (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;trans_handle : PISC_TR_HANDLE;array_id : PISC_QUAD;descriptor : PISC_ARRAY_DESC;source_array : PVoid;slice_length : PISC_LONG) : ISC_STATUS;stdcall; external LibName;
procedure isc_blob_default_desc (descriptor : PISC_BLOB_DESC;table_name : PUChar;column_name : PUChar);stdcall; external LibName;
function isc_blob_gen_bpb (status_vector : PISC_STATUS;to_descriptor,from_descriptor : PISC_BLOB_DESC;bpb_buffer_length : UShort;bpb_buffer : PUChar;bpb_length : PUShort) : ISC_STATUS;stdcall; external LibName;
function isc_blob_info (status_vector : PISC_STATUS;blob_handle : PISC_BLOB_HANDLE;item_list_buffer_length : Short;item_list_buffer : PChar;result_buffer_length : Short;result_buffer : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_blob_lookup_desc (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;trans_handle : PISC_TR_HANDLE;table_name,column_name : PChar;descriptor : PISC_BLOB_DESC;global : PUChar) : ISC_STATUS;stdcall; external LibName;
function isc_blob_set_desc (status_vector : PISC_STATUS;table_name,column_name : PChar;subtype,charset,segment_size : Short;descriptor : PISC_BLOB_DESC) : ISC_STATUS;stdcall; external LibName;
function isc_cancel_blob (status_vector : PISC_STATUS;blob_handle : PISC_BLOB_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_cancel_events (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;event_id : PISC_LONG) : ISC_STATUS;stdcall; external LibName;
function isc_close_blob (status_vector : PISC_STATUS;blob_handle : PISC_BLOB_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_commit_retaining (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_commit_transaction (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_create_blob (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;blob_handle : PISC_BLOB_HANDLE;blob_id : PISC_QUAD) : ISC_STATUS;stdcall; external LibName;
function isc_create_blob2 (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;blob_handle : PISC_BLOB_HANDLE;blob_id : PISC_QUAD;bpb_length : Short;bpb_address : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_create_database (status_vector : PISC_STATUS;isc_arg2 : Short;isc_arg3 : PChar;db_handle : PISC_DB_HANDLE;isc_arg5 : Short;isc_arg6 : PChar;isc_arg7 : Short) : ISC_STATUS;stdcall; external LibName;
function isc_database_info (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;item_list_buffer_length : Short;item_list_buffer : PChar;result_buffer_length : Short;result_buffer : PChar) : ISC_STATUS;stdcall; external LibName;
procedure isc_decode_date (ib_date : PISC_QUAD;tm_date : PCTimeStructure);stdcall; external LibName;
function isc_detach_database (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_drop_database (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE) : ISC_STATUS;stdcall; external LibName;
function SQLDA_LENGTH (n : Long) : Long;function XSQLDA_LENGTH (n : Long) : Long;function isc_dsql_allocate_statement (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;stmt_handle : PISC_STMT_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_alloc_statement2 (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;stmt_handle : PISC_STMT_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_describe (status_vector : PISC_STATUS;stmt_handle : PISC_STMT_HANDLE;dialect : UShort;xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_describe_bind (status_vector : PISC_STATUS;stmt_handle : PISC_STMT_HANDLE;dialect : UShort;xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_exec_immed2 (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;length : UShort;statement : PChar;dialect : UShort;in_xsqlda,out_xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_execute (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;stmt_handle : PISC_STMT_HANDLE;dialect : UShort;xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_execute2 (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;stmt_handle : PISC_STMT_HANDLE;dialect : UShort;in_xsqlda,out_xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_execute_immediate (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;length : UShort;statement : PChar;dialect : UShort;xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_fetch (status_vector : PISC_STATUS;stmt_handle : PISC_STMT_HANDLE;dialect : UShort;xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_finish (db_handle : PISC_DB_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_free_statement (status_vector : PISC_STATUS;stmt_handle : PISC_STMT_HANDLE;options : UShort) : ISC_STATUS; stdcall; external LibName;
function isc_dsql_insert (status_vector : PISC_STATUS;stmt_handle : PISC_STMT_HANDLE;arg3 : UShort;xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_prepare (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;stmt_handle : PISC_STMT_HANDLE;length : UShort;statement : PChar;dialect : UShort;xsqlda : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_set_cursor_name (status_vector : PISC_STATUS;stmt_handle : PISC_STMT_HANDLE;cursor_name : PChar;_type : UShort) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_sql_info (status_vector : PISC_STATUS;stmt_handle : PISC_STMT_HANDLE;item_length : UShort; items : PChar;buffer_length : UShort; buffer : PChar) : ISC_STATUS;stdcall; external LibName;
procedure isc_encode_date (tm_date : PCTimeStructure;ib_date : PISC_QUAD);stdcall; external LibName;
function  isc_event_block (event_buffer : PPChar;result_buffer : PPChar;id_count : UShort;event_list : array of PChar) : ISC_LONG;cdecl; external LibName;
procedure isc_event_counts (status_vector : PISC_STATUS;buffer_length : Short;event_buffer : PChar;result_buffer : PChar);stdcall; external LibName;
procedure isc_expand_dpb (dpb : PPChar;dpb_length : PShort;item_list : array of Pointer);cdecl; external LibName;
function isc_modify_dpb (dpb : PPChar;isc_arg2,isc_arg3 : PShort;isc_arg4 : UShort;isc_arg5 : PChar;isc_arg6 : Short) : Int;stdcall; external LibName;
function isc_free (isc_arg1 : PChar) : ISC_LONG;stdcall; external LibName;
function isc_get_segment (status_vector : PISC_STATUS;blob_handle : PISC_BLOB_HANDLE;actual_seg_length : PUShort;seg_buffer_length : UShort;seg_buffer : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_get_slice (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : PISC_QUAD;isc_arg5 : Short;isc_arg6 : PChar;isc_arg7 : Short;isc_arg8 : PISC_LONG;isc_arg9 : ISC_LONG;isc_arg10 : PVoid;isc_arg11 : PISC_LONG) : ISC_STATUS;stdcall; external LibName;
function isc_interprete (buffer : PChar;status_vector : PPISC_STATUS) : ISC_STATUS;stdcall; external LibName;
function isc_open_blob (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;blob_handle : PISC_BLOB_HANDLE;blob_id : PISC_QUAD) : ISC_STATUS;stdcall; external LibName;
function isc_open_blob2 (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;blob_handle : PISC_BLOB_HANDLE;blob_id : PISC_QUAD;bpb_length : Short;bpb_buffer : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_prepare_transaction2 (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;msg_length : Short;msg : PChar) : ISC_STATUS;stdcall; external LibName;
procedure isc_print_sqlerror (sqlcode : Short;status_vector : PISC_STATUS);stdcall; external LibName;
function isc_print_status (status_vector : PISC_STATUS) : ISC_STATUS;stdcall; external LibName;
function isc_put_segment (status_vector : PISC_STATUS;blob_handle : PISC_BLOB_HANDLE;seg_buffer_len : UShort;seg_buffer : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_put_slice (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : PISC_QUAD;isc_arg5 : Short;isc_arg6 : PChar;isc_arg7 : Short;isc_arg8 : PISC_LONG;isc_arg9 : ISC_LONG;isc_arg10 : PVoid) : ISC_STATUS;stdcall; external LibName;
function isc_que_events (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;event_id : PISC_LONG;length : Short;event_buffer : PChar;event_function : TISC_CALLBACK;event_function_arg : PVoid) : ISC_STATUS;stdcall; external LibName;
function isc_rollback_transaction (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_start_multiple (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;db_handle_count : Short;teb_vector_address : PISC_TEB) : ISC_STATUS;stdcall; external LibName;
function isc_start_transaction (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;db_handle_count : Short;db_handle : PISC_DB_HANDLE;tpb_length : UShort;tpb_address : PChar) : ISC_STATUS;cdecl; external LibName;
function isc_sqlcode (status_vector : PISC_STATUS) : ISC_LONG;stdcall; external LibName;
procedure isc_sql_interprete (sqlcode : Short;buffer : PChar;buffer_length : Short);stdcall; external LibName;
function isc_transaction_info (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;item_list_buffer_length : Short;item_list_buffer : PChar;result_buffer_length : Short;result_buffer : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_transact_request (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : UShort;isc_arg5 : PChar;isc_arg6 : UShort;isc_arg7 : PChar;isc_arg8 : UShort;isc_arg9 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_vax_integer (buffer : PChar;length : Short) : ISC_LONG;stdcall; external LibName;
function isc_add_user (status_vector : PISC_STATUS;user_sec_data : PUserSecData) : ISC_STATUS;stdcall; external LibName;
function isc_delete_user (status_vector : PISC_STATUS;user_sec_data : PUserSecData) : ISC_STATUS;stdcall; external LibName;
function isc_modify_user (status_vector : PISC_STATUS;user_sec_data : PUserSecData) : ISC_STATUS;stdcall; external LibName;
function isc_compile_request (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg4 : Short;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_compile_request2 (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg4 : Short;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_ddl (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : Short;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_prepare_transaction (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_receive (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE;isc_arg3,isc_arg4 : Short;isc_arg5 : PVoid;isc_arg6 : Short) : ISC_STATUS;stdcall; external LibName;{$ifndef fpc}function isc_receive2 (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE;isc_arg3,isc_arg4 : Short;isc_arg5 : PVoid;isc_arg6,isc_arg7 : Short;isc_arg8 : Long) : ISC_STATUS;stdcall; external LibName;{$endif}
function isc_reconnect_transaction (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : Short;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_release_request (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_request_info (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE;isc_arg3 : Short;isc_arg4 : Short;isc_arg5 : PChar;isc_arg6 : Short;isc_arg7 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_seek_blob (status_vector : PISC_STATUS;blob_handle : PISC_BLOB_HANDLE;isc_arg3 : Short;isc_arg4 : ISC_LONG;isc_arg5 : PISC_LONG) : ISC_STATUS;stdcall; external LibName;
function isc_send (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE;isc_arg3,isc_arg4 : Short;isc_arg5 : PVoid;isc_arg6 : Short) : ISC_STATUS;stdcall; external LibName;
function isc_start_and_send (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4,isc_arg5 : Short;isc_arg6 : PVoid;isc_arg7 : Short) : ISC_STATUS;stdcall; external LibName;
function isc_start_request (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : Short) : ISC_STATUS;stdcall; external LibName;
function isc_unwind_request (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;isc_arg3 : Short) : ISC_STATUS;stdcall; external LibName;
function isc_wait_for_event (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;length : Short;event_buffer,result_buffer : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_close (status_vector : PISC_STATUS;isc_arg2 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_declare (status_vector : PISC_STATUS;isc_arg2,isc_arg3 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_describe (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : PSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_describe_bind (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : PSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_execute (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;isc_arg3 : PChar;isc_arg4 : PSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_execute_immediate (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : PShort;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_fetch (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : PSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_open (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;isc_arg3 : PChar;isc_arg4 : PSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_prepare (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : PChar;isc_arg5 : PShort;isc_arg6 : PChar;isc_arg7 : PSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_execute_m (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;statement_handle : PISC_STMT_HANDLE;isc_arg4 : UShort;isc_arg5 : PChar;isc_arg6 : UShort;isc_arg7 : UShort;isc_arg8 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_execute2_m (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;statement_handle : PISC_STMT_HANDLE;isc_arg4 : UShort;isc_arg5 : PChar;isc_arg6 : UShort;isc_arg7 : UShort;isc_arg8 : PChar;isc_arg9 : UShort;isc_arg10 : PChar;isc_arg11 : UShort;isc_arg12 : UShort;isc_arg13 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_execute_immediate_m (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : UShort;isc_arg5 : PChar;isc_arg6 : UShort;isc_arg7 : UShort;isc_arg8 : PChar;isc_arg9 : UShort;isc_arg10 : UShort;isc_arg11 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_exec_immed3_m (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : UShort;isc_arg5 : PChar;isc_arg6 : UShort;isc_arg7 : UShort;isc_arg8 : PChar;isc_arg9 : UShort;isc_arg10 : UShort;isc_arg11 : PChar;isc_arg12 : UShort;isc_arg13 : PChar;isc_arg14 : UShort;isc_arg15 : UShort;isc_arg16 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_fetch_m (status_vector : PISC_STATUS;statement_handle : PISC_STMT_HANDLE;isc_arg3 : UShort;isc_arg4 : PChar;isc_arg5 : UShort;isc_arg6 : UShort;isc_arg7 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_insert_m (status_vector : PISC_STATUS;statement_handle : PISC_STMT_HANDLE;isc_arg3 : UShort;isc_arg4 : PChar;isc_arg5 : UShort;isc_arg6 : UShort;isc_arg7 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_prepare_m (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;statement_handle : PISC_STMT_HANDLE;isc_arg4 : UShort;isc_arg5 : PChar;isc_arg6 : UShort;isc_arg7 : UShort;isc_arg8 : PChar;isc_arg9 : UShort;isc_arg10 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_release (status_vector : PISC_STATUS;isc_arg2 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_close (status_vector : PISC_STATUS;isc_arg2 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_declare (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_describe (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : UShort;isc_arg4 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_describe_bind (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : UShort;isc_arg4 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_execute (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;isc_arg3 : PChar;isc_arg4 : UShort;isc_arg5 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_execute2 (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;isc_arg3 : PChar;isc_arg4 : UShort;isc_arg5 : PXSQLDA;isc_arg6 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_execute_immed (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : UShort;isc_arg5 : PChar;isc_arg6 : UShort;isc_arg7 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_fetch (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : UShort;isc_arg4 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_open (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;isc_arg3 : PChar;isc_arg4 : UShort;isc_arg5 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_open2 (status_vector : PISC_STATUS;tran_handle : PISC_TR_HANDLE;isc_arg3 : PChar;isc_arg4 : UShort;isc_arg5 : PXSQLDA;isc_arg6 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_insert (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : UShort;isc_arg4 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_prepare (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;isc_arg4 : PChar;isc_arg5 : UShort;isc_arg6 : PChar;isc_arg7 : UShort;isc_arg8 : PXSQLDA) : ISC_STATUS;stdcall; external LibName;
function isc_embed_dsql_release (status_vector : PISC_STATUS;isc_arg2 : PChar) : ISC_STATUS;stdcall; external LibName;
function BLOB_open (blob_handle : TISC_BLOB_HANDLE;isc_arg2 : PChar;isc_arg3 : int) : PBSTREAM;stdcall; external LibName;
function BLOB_put (isc_arg1 : char;isc_arg2 : PBSTREAM) : Int;stdcall; external LibName;
function BLOB_close (isc_arg1 : PBSTREAM) : Int;stdcall; external LibName;
function BLOB_get (isc_arg1 : PBSTREAM) : Int;stdcall; external LibName;
function BLOB_display (isc_arg1 : PISC_QUAD;db_handle : TISC_DB_HANDLE;tran_handle : TISC_TR_HANDLE;isc_arg4 : PChar) : Int;stdcall; external LibName;
function BLOB_dump (isc_arg1 : PISC_QUAD;db_handle : TISC_DB_HANDLE;tran_handle : TISC_TR_HANDLE;isc_arg4 : PChar) : Int;stdcall; external LibName;
function BLOB_edit (isc_arg1 : PISC_QUAD;db_handle : TISC_DB_HANDLE;tran_handle : TISC_TR_HANDLE;isc_arg4 : PChar) : Int;stdcall; external LibName;
function BLOB_load (isc_arg1 : PISC_QUAD;db_handle : TISC_DB_HANDLE;tran_handle : TISC_TR_HANDLE;isc_arg4 : PChar) : Int;stdcall; external LibName;
function BLOB_text_dump (isc_arg1 : PISC_QUAD;db_handle : TISC_DB_HANDLE;tran_handle : TISC_TR_HANDLE;isc_arg4 : PChar) : Int;stdcall; external LibName;
function BLOB_text_load (isc_arg1 : PISC_QUAD;db_handle : TISC_DB_HANDLE;tran_handle : TISC_TR_HANDLE;isc_arg4 : PChar) : Int;stdcall; external LibName;
function Bopen (isc_arg1 : PISC_QUAD;db_handle : TISC_DB_HANDLE;tran_handle : TISC_TR_HANDLE;isc_arg4 : PChar) : Int;stdcall; external LibName;
function isc_ftof (isc_arg1 : PChar;isc_arg2 : UShort;isc_arg3 : PChar;isc_arg4 : UShort) : ISC_LONG;stdcall; external LibName;
function isc_print_blr (isc_arg1 : PChar;isc_arg2 : TISC_CALLBACK;isc_arg3 : PVoid;isc_arg4 : Short) : ISC_STATUS;stdcall; external LibName;
procedure isc_set_debug (isc_arg1 : Int);stdcall; external LibName;
procedure isc_qtoq (isc_arg1 : PISC_QUAD;isc_arg2 : PISC_QUAD);stdcall; external LibName;
procedure isc_vtof (isc_arg1 : PChar;isc_arg2 : PChar;isc_arg3 : UShort);stdcall; external LibName;
procedure isc_vtov (isc_arg1 : PChar;isc_arg2 : PChar;isc_arg3 : Short);stdcall; external LibName;
function isc_version (db_handle : PISC_DB_HANDLE;isc_arg2 : TISC_CALLBACK;isc_arg3 : PVoid) : Int;stdcall; external LibName;
function isc_attach_service (status_vector : PISC_STATUS;isc_arg2 : UShort;isc_arg3 : PChar;service_handle : PISC_SVC_HANDLE;isc_arg5 : UShort;isc_arg6 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_detach_service (status_vector : PISC_STATUS;service_handle : PISC_SVC_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_query_service (status_vector : PISC_STATUS;service_handle : PISC_SVC_HANDLE;isc_arg3 : UShort;isc_arg4 : PChar;isc_arg5 : UShort;isc_arg6 : PChar;isc_arg7 : UShort;isc_arg8 : PChar) : ISC_STATUS;stdcall; external LibName;

{$ifdef IB_CURSORS}
function isc_embed_dsql_fetch2 (status_vector : PISC_STATUS;isc_arg2 : PChar;isc_arg3 : UShort;isc_arg4 : PXSQLDA;isc_arg5 : UShort;isc_arg6 : Long) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_fetch2 (status_vector : PISC_STATUS;stmt_handle : PISC_STMT_HANDLE;dialect : UShort;xsqlda : PXSQLDA;isc_arg5 : UShort;isc_arg6 : Long) : ISC_STATUS;stdcall; external LibName;
function isc_dsql_fetch2_m (status_vector : PISC_STATUS;statement_handle : PISC_STMT_HANDLE;isc_arg3 : UShort;isc_arg4 : PChar;isc_arg5 : UShort;isc_arg6 : UShort;isc_arg7 : PChar;isc_arg8 : UShort;isc_arg9 : Long) : ISC_STATUS;stdcall; external LibName;
{$endif}

{$ifdef IB_Extensions}
function Bopen2 (isc_arg1 : PISC_QUAD;db_handle : TISC_DB_HANDLE;tran_handle : TISC_TR_HANDLE;isc_arg4 : PChar;isc_arg5 : UShort) : PBSTREAM;stdcall; external LibName;
function isc_reset_fpe (isc_arg1 : UShort) : ISC_LONG;stdcall; external LibName;
function isc_compile_map (status_vector : PISC_STATUS;form_handle : PISC_FORM_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg4 : PShort;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_compile_menu (status_vector : PISC_STATUS;form_handle : PISC_FORM_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg4 : PShort;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_compile_sub_map (status_vector : PISC_STATUS;win_handle : PISC_WIN_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg4 : PShort;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_create_window (status_vector : PISC_STATUS;win_handle : PISC_WIN_HANDLE;isc_arg3 : PShort;isc_arg4 : PChar;isc_arg5 : PShort;isc_arg6 : PShort) : ISC_STATUS;stdcall; external LibName;
function isc_delete_window (status_vector : PISC_STATUS;win_handle : PISC_WIN_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_drive_form (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;win_handle : PISC_WIN_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg6 : PUChar;isc_arg7 : PUChar) : ISC_STATUS;stdcall; external LibName;
function isc_drive_menu (status_vector : PISC_STATUS;win_handle : PISC_WIN_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg4 : PShort;isc_arg5 : PChar;isc_arg6 : PShort;isc_arg7 : PChar;isc_arg8 : PShort;isc_arg9 : PShort;isc_arg10 : PChar;isc_arg11 : PISC_LONG) : ISC_STATUS;stdcall; external LibName;
function isc_form_delete (status_vector : PISC_STATUS;form_handle : PISC_FORM_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_form_fetch (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg5 : PUChar) : ISC_STATUS;stdcall; external LibName;
function isc_form_insert (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg5 : PUChar) : ISC_STATUS;stdcall; external LibName;
function isc_get_entree (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE;isc_arg3 : PShort;isc_arg4 : PChar;isc_arg5 : PISC_LONG;isc_arg6 : PShort) : ISC_STATUS;stdcall; external LibName;
function isc_initialize_menu (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_menu (status_vector : PISC_STATUS;win_handle : PISC_WIN_HANDLE;request_handle : PISC_REQ_HANDLE;isc_arg4 : PShort;isc_arg5 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_load_form (status_vector : PISC_STATUS;db_handle : PISC_DB_HANDLE;tran_handle : PISC_TR_HANDLE;form_handle : PISC_FORM_HANDLE;isc_arg5 : PShort;isc_arg6 : PChar) : ISC_STATUS;stdcall; external LibName;
function isc_pop_window (status_vector : PISC_STATUS;win_handle : PISC_WIN_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_put_entree (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE;isc_arg3 : PShort;isc_arg4 : PChar;isc_arg5 : PISC_LONG) : ISC_STATUS;stdcall; external LibName;
function isc_reset_form (status_vector : PISC_STATUS;request_handle : PISC_REQ_HANDLE) : ISC_STATUS;stdcall; external LibName;
function isc_suspend_window (status_vector : PISC_STATUS;win_handle : PISC_WIN_HANDLE) : ISC_STATUS;stdcall; external LibName;
{$endif}


implementation

function SQLDA_LENGTH(n: Long): Long;
begin
  SQLDA_LENGTH := sizeof(TSQLDA) + ((n - 1) * sizeof(TSQLVAR));
end;


function XSQLDA_LENGTH(n: Long): Long;
begin
  XSQLDA_LENGTH := SizeOf(TXSQLDA) + ((n - 1) * SizeOf(TXSQLVAR));
end;


function getb(p: PBSTREAM): Char;
begin
  Dec(p^.bstr_cnt);
  if (p^.bstr_cnt >= 0) then
    begin
    getb := Chr(Byte(p^.bstr_ptr^) and 248);
    Inc(p^.bstr_ptr);
    end
  else
    getb := Char(BLOB_get(p));
end;


function putb(x: Char; p: PBSTREAM): Int;
begin
  Dec(p^.bstr_cnt);
  if (x = Chr(Byte('n') - Byte('a'))) or (p^.bstr_cnt = 0) then
    putb := BLOB_put(x, p)
      else
      begin    p^.bstr_ptr^ := Char(x);
    putb := Byte(x);
    Inc(p^.bstr_ptr^);
  end;
end;


function putbx(x: Char; p: PBSTREAM): Int;
begin  Dec(p^.bstr_cnt);
  if (p^.bstr_cnt = 0) then
    putbx := BLOB_put(x, p)
  else
    begin
    p^.bstr_ptr^ := Char(x);
    Inc(p^.bstr_ptr^);
    putbx := Byte(x);
  end;
end;


end.
